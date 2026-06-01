/// @file bc_conformance_trace.cpp
/// @brief LP/frontier conformance diagnostics shared by the legacy B&C core.

#include "mipsolvers/engine/detail/bc_conformance_trace.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Sparse>
#include <fmt/format.h>

#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/detail/bc_pools.hpp"

namespace mipsolvers::engine::detail {

const char* bc_var_type_name(const VariableMeta& v) {
  switch (v.type) {
    case VarType::Binary:
      return "bin";
    case VarType::Integer:
      return "int";
    case VarType::Continuous:
      return "cont";
    default:
      return "var";
  }
}

std::vector<char> bc_original_basic_mask(int n_orig,
                                         const SimplexBasis* basis_hint) {
  std::vector<char> basic(static_cast<std::size_t>(std::max(0, n_orig)), 0);
  if (basis_hint == nullptr) return basic;
  for (int col : basis_hint->basis_indices()) {
    if (col >= 0 && col < n_orig) {
      basic[static_cast<std::size_t>(col)] = 1;
    }
  }
  return basic;
}

char bc_native_basis_status(int col,
                            const std::vector<char>& basic,
                            const SimplexBasis* basis_hint) {
  if (col >= 0 && col < static_cast<int>(basic.size()) &&
      basic[static_cast<std::size_t>(col)] != 0) {
    return 'B';
  }
  if (basis_hint != nullptr && col >= 0 &&
      col < static_cast<int>(basis_hint->at_upper.size()) &&
      basis_hint->at_upper[static_cast<std::size_t>(col)] != 0) {
    return 'U';
  }
  return 'L';
}

double bc_native_reduced_cost(int col, const SimplexBasis* basis_hint) {
  if (basis_hint == nullptr || !basis_hint->cached_reduced_costs ||
      col < 0 || col >= basis_hint->cached_reduced_costs->size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (*basis_hint->cached_reduced_costs)[col];
}

std::uint64_t bc_model_side_state_signature(const LPModel& lp) {
  auto mix = [](std::uint64_t seed, std::uint64_t value) {
    seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  };
  // Explicit return type avoids 'inconsistent deduced types' on Linux GCC
  // where uint64_t is 'unsigned long' but 0x...ULL literals are 'unsigned long long'.
  auto dbl = [](double value) -> std::uint64_t {
    if (std::isinf(value)) {
      return value > 0.0 ? 0x7ff0000000000000ULL : 0xfff0000000000000ULL;
    }
    if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
    return static_cast<std::uint64_t>(
        static_cast<std::int64_t>(std::llround(value * 1e9)));
  };
  std::uint64_t sig = 0x4841434448505353ULL;
  sig = mix(sig, static_cast<std::uint64_t>(lp.vars.size()));
  sig = mix(sig, static_cast<std::uint64_t>(lp.A.rows()));
  sig = mix(sig, static_cast<std::uint64_t>(lp.Aeq.rows()));
  sig = mix(sig, static_cast<std::uint64_t>(lp.A.nonZeros()));
  sig = mix(sig, static_cast<std::uint64_t>(lp.Aeq.nonZeros()));
  sig = mix(sig, static_cast<std::uint64_t>(lp.sense == Sense::Maximize));
  for (int j = 0; j < static_cast<int>(lp.vars.size()); ++j) {
    sig = mix(sig, static_cast<std::uint64_t>(j));
    sig = mix(sig, static_cast<std::uint64_t>(lp.vars[j].type));
    sig = mix(sig, dbl(lp.c[j]));
    sig = mix(sig, dbl(lp.vars[j].lb));
    sig = mix(sig, dbl(lp.vars[j].ub));
  }
  for (int j = 0; j < lp.A.outerSize(); ++j) {
    sig = mix(sig, static_cast<std::uint64_t>(j));
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
      sig = mix(sig, static_cast<std::uint64_t>(it.row()));
      sig = mix(sig, dbl(it.value()));
    }
  }
  for (int j = 0; j < lp.Aeq.outerSize(); ++j) {
    sig = mix(sig, static_cast<std::uint64_t>(j) ^ 0x4551ULL);
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
      sig = mix(sig, static_cast<std::uint64_t>(it.row()));
      sig = mix(sig, dbl(it.value()));
    }
  }
  for (int r = 0; r < static_cast<int>(lp.b.size()); ++r) {
    sig = mix(sig, dbl(lp_row_lhs_or_neg_inf(lp, r)));
    sig = mix(sig, dbl(lp.b[r]));
  }
  for (int r = 0; r < static_cast<int>(lp.beq.size()); ++r) {
    sig = mix(sig, dbl(lp.beq[r]));
  }
  return sig;
}

const HiGHSPresolvedModelStats& cached_highs_presolve_side_state(
    const LPModel& lp,
    bool* cache_hit) {
  static std::mutex cache_mutex;
  static std::unordered_map<std::uint64_t, HiGHSPresolvedModelStats> cache;
  const std::uint64_t sig = bc_model_side_state_signature(lp);
  {
    std::lock_guard<std::mutex> lock(cache_mutex);
    auto it = cache.find(sig);
    if (it != cache.end()) {
      if (cache_hit != nullptr) *cache_hit = true;
      return it->second;
    }
  }
  HiGHSPresolvedModelStats computed = highs_presolve_model_stats(lp);
  std::lock_guard<std::mutex> lock(cache_mutex);
  auto [it, inserted] = cache.emplace(sig, std::move(computed));
  (void)inserted;
  if (cache_hit != nullptr) *cache_hit = false;
  return it->second;
}

std::uint64_t bc_trace_hash_mix(std::uint64_t seed, std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

std::uint64_t bc_trace_hash_double(double value) {
  if (std::isinf(value)) {
    return value > 0.0 ? 0x7ff0000000000000ULL : 0xfff0000000000000ULL;
  }
  if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
  return static_cast<std::uint64_t>(
      static_cast<std::int64_t>(std::llround(value * 1e9)));
}

[[maybe_unused]] void trace_native_frontier_conformance(
    const char* phase,
    const LPModel& lp,
    const StandardFormLP* native_sf,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    double objective,
    double int_tol);

[[maybe_unused]] void trace_native_frontier_conformance(
    const char* phase,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    double objective,
    double int_tol) {
  trace_native_frontier_conformance(phase, lp, nullptr, implied_integer_cols, x,
                                    lb, ub, basis_hint, objective, int_tol);
}

void trace_native_frontier_conformance(
    const char* phase,
    const LPModel& lp,
    const StandardFormLP* native_sf,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    double objective,
    double int_tol) {
  if (!bc_frontier_conformance_enabled()) return;
  const int n = static_cast<int>(lp.vars.size());
  if (x.size() != n || lb.size() != n || ub.size() != n) {
    fmt::print(stderr,
               "[B&C-FRONTIER] phase={} invalid_sizes n={} x={} lb={} ub={}\n",
               phase != nullptr ? phase : "unknown", n, x.size(), lb.size(),
               ub.size());
    return;
  }

  const std::vector<char> basic = bc_original_basic_mask(n, basis_hint);
  int basic_cols = 0;
  int nonbasic_lower = 0;
  int nonbasic_upper = 0;
  int fixed_cols = 0;
  int integer_like_total = 0;
  int fractional_total = 0;
  int fractional_binary = 0;
  int fractional_integer = 0;
  int fractional_implied = 0;
  std::uint64_t status_hash = 0x4841434450435354ULL;
  std::vector<char> native_status(static_cast<std::size_t>(n), '?');
  std::vector<double> native_value(static_cast<std::size_t>(n),
                                   std::numeric_limits<double>::quiet_NaN());
  std::vector<double> native_rc(static_cast<std::size_t>(n),
                                std::numeric_limits<double>::quiet_NaN());
  std::vector<char> integer_like_mask(static_cast<std::size_t>(n), 0);
  std::vector<char> native_frontier_mask(static_cast<std::size_t>(n), 0);
  std::vector<char> native_nonbasic_mask(static_cast<std::size_t>(n), 0);

  struct FrontierTerm {
    int col;
    double value;
    double rounded;
    double frac;
    char status;
    double rc;
    bool implied;
    const char* type;
  };
  struct SideTerm {
    int col;
    double value;
    double lb;
    double ub;
    char status;
    double rc;
    bool implied;
    const char* type;
  };
  std::vector<FrontierTerm> frontier;
  std::vector<SideTerm> side_terms;
  frontier.reserve(256);
  side_terms.reserve(256);

  for (int j = 0; j < n; ++j) {
    const bool is_basic = basic[static_cast<std::size_t>(j)] != 0;
    const char status = bc_native_basis_status(j, basic, basis_hint);
    const double rcj = bc_native_reduced_cost(j, basis_hint);
    native_status[static_cast<std::size_t>(j)] = status;
    native_value[static_cast<std::size_t>(j)] = x[j];
    native_rc[static_cast<std::size_t>(j)] = rcj;
    status_hash = bc_trace_hash_mix(status_hash,
                                    static_cast<std::uint64_t>(j));
    status_hash = bc_trace_hash_mix(status_hash,
                                    static_cast<std::uint64_t>(status));
    status_hash = bc_trace_hash_mix(status_hash, bc_trace_hash_double(x[j]));
    const bool is_fixed =
        std::isfinite(lb[j]) && std::isfinite(ub[j]) &&
        ub[j] - lb[j] <= std::max(1e-9, int_tol);
    if (is_basic) {
      ++basic_cols;
    } else if (basis_hint != nullptr &&
               j < static_cast<int>(basis_hint->at_upper.size()) &&
               basis_hint->at_upper[static_cast<std::size_t>(j)] != 0) {
      ++nonbasic_upper;
    } else {
      ++nonbasic_lower;
    }
    if (is_fixed) ++fixed_cols;

    const bool implied =
        j < static_cast<int>(implied_integer_cols.size()) &&
        implied_integer_cols[static_cast<std::size_t>(j)] != 0;
    const bool integer_like =
        is_integer_type(lp.vars[static_cast<std::size_t>(j)]) || implied;
    if (!integer_like) continue;
    integer_like_mask[static_cast<std::size_t>(j)] = 1;
    ++integer_like_total;
    if (!is_basic) {
      native_nonbasic_mask[static_cast<std::size_t>(j)] = 1;
      side_terms.push_back(SideTerm{
          j, x[j], lb[j], ub[j], status, rcj, implied,
          bc_var_type_name(lp.vars[static_cast<std::size_t>(j)])});
    }
    const double rounded =
        std::min(ub[j], std::max(lb[j], std::round(x[j])));
    const double frac = std::abs(x[j] - rounded);
    if (!(frac > int_tol)) continue;
    ++fractional_total;
    if (lp.vars[static_cast<std::size_t>(j)].type == VarType::Binary) {
      ++fractional_binary;
    } else if (lp.vars[static_cast<std::size_t>(j)].type == VarType::Integer) {
      ++fractional_integer;
    } else if (implied) {
      ++fractional_implied;
    }
    native_frontier_mask[static_cast<std::size_t>(j)] = 1;
    frontier.push_back(FrontierTerm{
        j, x[j], rounded, frac, status, rcj, implied,
        bc_var_type_name(lp.vars[static_cast<std::size_t>(j)])});
  }

  std::sort(frontier.begin(), frontier.end(),
            [](const FrontierTerm& a, const FrontierTerm& b) {
              return a.col < b.col;
            });
  std::sort(side_terms.begin(), side_terms.end(),
            [](const SideTerm& a, const SideTerm& b) {
              return a.col < b.col;
            });
  std::uint64_t hash = 0x4841434450434655ULL;
  for (const FrontierTerm& t : frontier) {
    hash = bc_trace_hash_mix(hash, static_cast<std::uint64_t>(t.col));
    hash = bc_trace_hash_mix(hash, bc_trace_hash_double(t.value));
    hash = bc_trace_hash_mix(hash, static_cast<std::uint64_t>(t.status));
  }

  fmt::memory_buffer sample;
  const int max_terms =
      bc_conformance_trace_terms("HACDCPF_FRONTIER_CONFORM_TERMS", 32);
  for (int k = 0; k < static_cast<int>(frontier.size()) && k < max_terms; ++k) {
    const FrontierTerm& t = frontier[static_cast<std::size_t>(k)];
    if (k > 0) fmt::format_to(std::back_inserter(sample), ";");
    fmt::format_to(std::back_inserter(sample),
                   "{}:{:.12g}:r{:.12g}:f{:.6g}:{}:rc{:.6g}:{}:{}",
                   t.col, t.value, t.rounded, t.frac, t.status, t.rc,
                   t.type, t.implied ? "impl" : "decl");
  }
  fmt::memory_buffer side_sample;
  for (int k = 0; k < static_cast<int>(side_terms.size()) && k < max_terms;
       ++k) {
    const SideTerm& t = side_terms[static_cast<std::size_t>(k)];
    if (k > 0) fmt::format_to(std::back_inserter(side_sample), ";");
    fmt::format_to(std::back_inserter(side_sample),
                   "{}:{:.12g}:lb{:.12g}:ub{:.12g}:{}:rc{:.6g}:{}:{}",
                   t.col, t.value, t.lb, t.ub, t.status, t.rc, t.type,
                   t.implied ? "impl" : "decl");
  }

  fmt::print(stderr,
             "[B&C-FRONTIER] phase={} obj={:.12g} cols={} "
             "basis=basic{}:lower{}:upper{}:fixed{} "
             "integer_like={} frac={} bin={} int={} impl={} hash={:016x} "
             "status_hash={:016x} "
             "basis_rows={} basis_cols={} at_upper_size={} sample=[{}]\n",
             phase != nullptr ? phase : "unknown", objective, n, basic_cols,
             nonbasic_lower, nonbasic_upper, fixed_cols, integer_like_total,
             fractional_total, fractional_binary, fractional_integer,
             fractional_implied, hash, status_hash,
             basis_hint != nullptr ? basis_hint->rows : -1,
             basis_hint != nullptr ? basis_hint->cols : -1,
             basis_hint != nullptr
                 ? static_cast<int>(basis_hint->at_upper.size())
                 : -1,
             fmt::to_string(sample));
  fmt::print(stderr,
             "[B&C-LPSTATE] phase={} status_hash={:016x} "
             "nonbasic_integer_like={} side_sample=[{}]\n",
             phase != nullptr ? phase : "unknown", status_hash,
             side_terms.size(), fmt::to_string(side_sample));

  if (bc_env_flag_enabled("HACDCPF_LPSTATE_CONFORM")) {
    const HiGHSRootLpStateStats highs_state = highs_root_lp_state_stats(
        lp, implied_integer_cols, int_tol,
        bc_conformance_trace_terms("HACDCPF_FRONTIER_CONFORM_TERMS", 32));
    fmt::print(stderr,
               "[HIGHS-LPSTATE] phase={} avail={} pass={} solve={} "
               "status={} obj={:.12g} rows={} cols={} nnz={} iter={} "
               "basis_valid={} basis=basic{}:lower{}:upper{}:zero{}:"
               "other{}:fixed{} integer_like={} frac={} bin={} int={} "
               "impl={} hash={:016x} status_hash={:016x} sample=[{}]\n",
               phase != nullptr ? phase : "unknown",
               highs_state.available ? 1 : 0,
               highs_state.pass_ok ? 1 : 0,
               highs_state.solve_ok ? 1 : 0, highs_state.model_status,
               highs_state.objective, highs_state.rows, highs_state.cols,
               highs_state.nnz, highs_state.iterations,
               highs_state.basis_valid ? 1 : 0, highs_state.basic_cols,
               highs_state.nonbasic_lower, highs_state.nonbasic_upper,
               highs_state.nonbasic_zero, highs_state.nonbasic_other,
               highs_state.fixed_cols, highs_state.integer_like_total,
               highs_state.fractional_total, highs_state.fractional_binary,
               highs_state.fractional_integer, highs_state.fractional_implied,
               highs_state.frontier_hash, highs_state.status_hash,
               highs_state.frontier_sample);
    fmt::print(stderr,
               "[HIGHS-LPSTATE-SIDE] phase={} nonbasic_integer_like={} "
               "side_sample=[{}]\n",
               phase != nullptr ? phase : "unknown",
               highs_state.nonbasic_integer_like,
               highs_state.side_sample);
    if (static_cast<int>(highs_state.col_status.size()) == n &&
        static_cast<int>(highs_state.col_value.size()) == n &&
        static_cast<int>(highs_state.col_dual.size()) == n) {
      int status_diff = 0;
      int basic_diff = 0;
      int side_flip = 0;
      int degenerate_side_flip = 0;
      int degenerate_basic_boundary_diff = 0;
      int frontier_basic_boundary_diff = 0;
      int nondegenerate_status_diff = 0;
      int status_after_degenerate_remap = 0;
      int value_diff = 0;
      int frontier_both = 0;
      int frontier_native_only = 0;
      int frontier_highs_only = 0;
      int frontier_status_diff = 0;
      int side_integer_diff = 0;
      double max_abs_dx = 0.0;
      int max_abs_dx_col = -1;
      fmt::memory_buffer status_sample;
      fmt::memory_buffer frontier_native_sample;
      fmt::memory_buffer frontier_highs_sample;
      fmt::memory_buffer side_diff_sample;
      int status_emit = 0;
      int frontier_native_emit = 0;
      int frontier_highs_emit = 0;
      int side_diff_emit = 0;
      auto append_sep = [](fmt::memory_buffer& buf, int emit) {
        if (emit > 0) fmt::format_to(std::back_inserter(buf), ";");
      };
      const int diff_terms =
          bc_conformance_trace_terms("HACDCPF_LPSTATE_DIFF_TERMS", max_terms);
      for (int j = 0; j < n; ++j) {
        const char ns = native_status[static_cast<std::size_t>(j)];
        const char hs = highs_state.col_status[static_cast<std::size_t>(j)];
        const double nx = native_value[static_cast<std::size_t>(j)];
        const double hx = highs_state.col_value[static_cast<std::size_t>(j)];
        const double nrc = native_rc[static_cast<std::size_t>(j)];
        const double hrc = highs_state.col_dual[static_cast<std::size_t>(j)];
        const double dx =
            (std::isfinite(nx) && std::isfinite(hx))
                ? std::abs(nx - hx)
                : (nx == hx ? 0.0 : std::numeric_limits<double>::infinity());
        if (dx > std::max(1e-8, 10.0 * int_tol)) ++value_diff;
        if (dx > max_abs_dx) {
          max_abs_dx = dx;
          max_abs_dx_col = j;
        }
        if (ns != hs) {
          ++status_diff;
          const bool basic_membership_diff = (ns == 'B') != (hs == 'B');
          const bool lower_upper_flip =
              (ns == 'L' && hs == 'U') || (ns == 'U' && hs == 'L');
          const bool nx_at_lower = std::abs(nx - lb[j]) <=
                                   std::max(1e-8, 10.0 * int_tol);
          const bool hx_at_lower = std::abs(hx - lb[j]) <=
                                   std::max(1e-8, 10.0 * int_tol);
          const bool nx_at_upper =
              std::isfinite(ub[j]) &&
              std::abs(nx - ub[j]) <= std::max(1e-8, 10.0 * int_tol);
          const bool hx_at_upper =
              std::isfinite(ub[j]) &&
              std::abs(hx - ub[j]) <= std::max(1e-8, 10.0 * int_tol);
          const bool same_value = dx <= std::max(1e-8, 10.0 * int_tol);
          const bool same_bound =
              (nx_at_lower && hx_at_lower) || (nx_at_upper && hx_at_upper);
          const bool native_zero_rc =
              std::isfinite(nrc) &&
              std::abs(nrc) <= std::max(1e-8, 10.0 * int_tol);
          const bool highs_zero_rc =
              std::isfinite(hrc) &&
              std::abs(hrc) <= std::max(1e-8, 10.0 * int_tol);
          const bool on_degenerate_bound =
              same_value && same_bound && (native_zero_rc || highs_zero_rc);
          if (basic_membership_diff) {
            ++basic_diff;
            if (on_degenerate_bound) {
              ++degenerate_basic_boundary_diff;
              const bool nf =
                  native_frontier_mask[static_cast<std::size_t>(j)] != 0;
              const double hrounded = std::min(
                  ub[j], std::max(lb[j], std::round(hx)));
              const bool hf =
                  std::isfinite(hx) && std::abs(hx - hrounded) > int_tol;
              if (nf || hf) ++frontier_basic_boundary_diff;
            } else {
              ++nondegenerate_status_diff;
            }
          } else if (lower_upper_flip) {
            ++side_flip;
            if (on_degenerate_bound) {
              ++degenerate_side_flip;
            } else {
              ++nondegenerate_status_diff;
            }
          } else {
            ++nondegenerate_status_diff;
          }
          if (status_emit < diff_terms) {
            append_sep(status_sample, status_emit++);
            fmt::format_to(std::back_inserter(status_sample),
                           "{}:{}>{}:nx{:.12g}:hx{:.12g}:nrc{:.6g}:hrc{:.6g}:{}",
                           j, ns, hs, nx, hx, nrc, hrc,
                           integer_like_mask[static_cast<std::size_t>(j)]
                               ? "intlike"
                               : "cont");
          }
        }
        const bool integer_like =
            integer_like_mask[static_cast<std::size_t>(j)] != 0;
        if (!integer_like) continue;
        const bool native_nonbasic =
            native_nonbasic_mask[static_cast<std::size_t>(j)] != 0;
        const bool highs_nonbasic = hs != 'B';
        if (native_nonbasic != highs_nonbasic) {
          ++side_integer_diff;
          if (side_diff_emit < diff_terms) {
            append_sep(side_diff_sample, side_diff_emit++);
            fmt::format_to(std::back_inserter(side_diff_sample),
                           "{}:{}>{}:nx{:.12g}:hx{:.12g}:nrc{:.6g}:hrc{:.6g}:{}",
                           j, ns, hs, nx, hx, nrc, hrc,
                           (j < static_cast<int>(implied_integer_cols.size()) &&
                            implied_integer_cols[static_cast<std::size_t>(j)] != 0)
                               ? "impl"
                               : "decl");
          }
        }
        const double hrounded = std::min(
            ub[j], std::max(lb[j], std::round(hx)));
        const bool highs_frontier =
            std::isfinite(hx) && std::abs(hx - hrounded) > int_tol;
        const bool native_frontier =
            native_frontier_mask[static_cast<std::size_t>(j)] != 0;
        if (native_frontier && highs_frontier) {
          ++frontier_both;
          if (ns != hs) ++frontier_status_diff;
        } else if (native_frontier) {
          ++frontier_native_only;
          if (frontier_native_emit < diff_terms) {
            append_sep(frontier_native_sample, frontier_native_emit++);
            fmt::format_to(std::back_inserter(frontier_native_sample),
                           "{}:nx{:.12g}:hx{:.12g}:{}>{}:nrc{:.6g}:hrc{:.6g}:{}",
                           j, nx, hx, ns, hs, nrc, hrc,
                           (j < static_cast<int>(implied_integer_cols.size()) &&
                            implied_integer_cols[static_cast<std::size_t>(j)] != 0)
                               ? "impl"
                               : "decl");
          }
        } else if (highs_frontier) {
          ++frontier_highs_only;
          if (frontier_highs_emit < diff_terms) {
            append_sep(frontier_highs_sample, frontier_highs_emit++);
            fmt::format_to(std::back_inserter(frontier_highs_sample),
                           "{}:nx{:.12g}:hx{:.12g}:{}>{}:nrc{:.6g}:hrc{:.6g}:{}",
                           j, nx, hx, ns, hs, nrc, hrc,
                           (j < static_cast<int>(implied_integer_cols.size()) &&
                            implied_integer_cols[static_cast<std::size_t>(j)] != 0)
                               ? "impl"
                               : "decl");
          }
        }
      }
      status_after_degenerate_remap =
          status_diff - degenerate_side_flip -
          degenerate_basic_boundary_diff;
      fmt::print(stderr,
                 "[LPSTATE-DIFF] phase={} status_diff={} basic_diff={} "
                 "side_flip={} deg_side={} deg_basic={} "
                 "frontier_deg_basic={} nondeg_status={} post_remap_status={} "
                 "value_diff={} max_dx={:.6g}@{} "
                 "frontier=both{}:native_only{}:highs_only{}:"
                 "status_diff{} side_integer_diff={} sample=[{}]\n",
                 phase != nullptr ? phase : "unknown", status_diff,
                 basic_diff, side_flip, degenerate_side_flip,
                 degenerate_basic_boundary_diff, frontier_basic_boundary_diff,
                 nondegenerate_status_diff, status_after_degenerate_remap,
                 value_diff, max_abs_dx,
                 max_abs_dx_col, frontier_both, frontier_native_only,
                 frontier_highs_only, frontier_status_diff, side_integer_diff,
                 fmt::to_string(status_sample));
      fmt::print(stderr,
                 "[LPSTATE-FRONTIER-DIFF] phase={} native_only=[{}] "
                 "highs_only=[{}] side_diff=[{}]\n",
                 phase != nullptr ? phase : "unknown",
                 fmt::to_string(frontier_native_sample),
                 fmt::to_string(frontier_highs_sample),
                 fmt::to_string(side_diff_sample));
    }
    if (native_sf != nullptr) {
      const HiGHSRootLpStateStats sf_state =
          highs_standard_form_lp_state_stats(
              *native_sf, lp, implied_integer_cols, int_tol,
              bc_conformance_trace_terms("HACDCPF_FRONTIER_CONFORM_TERMS", 32));
      fmt::print(stderr,
                 "[HIGHS-SF-LPSTATE] phase={} avail={} pass={} solve={} "
                 "status={} obj={:.12g} rows={} cols={} nnz={} iter={} "
                 "basis_valid={} basis=basic{}:lower{}:upper{}:zero{}:"
                 "other{}:fixed{} integer_like={} frac={} bin={} int={} "
                 "impl={} hash={:016x} status_hash={:016x} sample=[{}]\n",
                 phase != nullptr ? phase : "unknown",
                 sf_state.available ? 1 : 0,
                 sf_state.pass_ok ? 1 : 0,
                 sf_state.solve_ok ? 1 : 0, sf_state.model_status,
                 sf_state.objective, sf_state.rows, sf_state.cols,
                 sf_state.nnz, sf_state.iterations,
                 sf_state.basis_valid ? 1 : 0, sf_state.basic_cols,
                 sf_state.nonbasic_lower, sf_state.nonbasic_upper,
                 sf_state.nonbasic_zero, sf_state.nonbasic_other,
                 sf_state.fixed_cols, sf_state.integer_like_total,
                 sf_state.fractional_total, sf_state.fractional_binary,
                 sf_state.fractional_integer, sf_state.fractional_implied,
                 sf_state.frontier_hash, sf_state.status_hash,
                 sf_state.frontier_sample);
      fmt::print(stderr,
                 "[HIGHS-SF-LPSTATE-SIDE] phase={} nonbasic_integer_like={} "
                 "side_sample=[{}]\n",
                 phase != nullptr ? phase : "unknown",
                 sf_state.nonbasic_integer_like, sf_state.side_sample);
      if (static_cast<int>(sf_state.col_status.size()) == n &&
          static_cast<int>(sf_state.col_value.size()) == n &&
          static_cast<int>(sf_state.col_dual.size()) == n) {
        int status_diff = 0;
        int basic_diff = 0;
        int side_flip = 0;
        int degenerate_side_flip = 0;
        int degenerate_basic_boundary_diff = 0;
        int frontier_basic_boundary_diff = 0;
        int degenerate_frontier_value_move = 0;
        int nondegenerate_status_diff = 0;
        int value_diff = 0;
        int frontier_both = 0;
        int frontier_native_only = 0;
        int frontier_highs_only = 0;
        int frontier_status_diff = 0;
        int side_integer_diff = 0;
        double max_abs_dx = 0.0;
        int max_abs_dx_col = -1;
        fmt::memory_buffer status_sample;
        fmt::memory_buffer frontier_native_sample;
        fmt::memory_buffer frontier_highs_sample;
        fmt::memory_buffer side_diff_sample;
        int status_emit = 0;
        int frontier_native_emit = 0;
        int frontier_highs_emit = 0;
        int side_diff_emit = 0;
        auto append_sep = [](fmt::memory_buffer& buf, int emit) {
          if (emit > 0) fmt::format_to(std::back_inserter(buf), ";");
        };
        const int diff_terms =
            bc_conformance_trace_terms("HACDCPF_LPSTATE_DIFF_TERMS",
                                       max_terms);
        const double state_tol = std::max(1e-8, 10.0 * int_tol);
        for (int j = 0; j < n; ++j) {
          const char ns = native_status[static_cast<std::size_t>(j)];
          const char hs = sf_state.col_status[static_cast<std::size_t>(j)];
          const double nx = native_value[static_cast<std::size_t>(j)];
          const double hx = sf_state.col_value[static_cast<std::size_t>(j)];
          const double nrc = native_rc[static_cast<std::size_t>(j)];
          const double hrc = sf_state.col_dual[static_cast<std::size_t>(j)];
          const double dx =
              (std::isfinite(nx) && std::isfinite(hx))
                  ? std::abs(nx - hx)
                  : (nx == hx ? 0.0 : std::numeric_limits<double>::infinity());
          if (dx > state_tol) ++value_diff;
          if (dx > max_abs_dx) {
            max_abs_dx = dx;
            max_abs_dx_col = j;
          }

          const bool integer_like =
              integer_like_mask[static_cast<std::size_t>(j)] != 0;
          const double hrounded = std::min(
              ub[j], std::max(lb[j], std::round(hx)));
          const bool highs_frontier =
              std::isfinite(hx) && std::abs(hx - hrounded) > int_tol;
          const bool native_frontier =
              native_frontier_mask[static_cast<std::size_t>(j)] != 0;
          if (integer_like && native_frontier && highs_frontier) {
            ++frontier_both;
            if (ns != hs) ++frontier_status_diff;
          } else if (integer_like && native_frontier) {
            ++frontier_native_only;
            if (frontier_native_emit < diff_terms) {
              append_sep(frontier_native_sample, frontier_native_emit++);
              fmt::format_to(
                  std::back_inserter(frontier_native_sample),
                  "{}:nx{:.12g}:hx{:.12g}:{}>{}:nrc{:.6g}:hrc{:.6g}:{}",
                  j, nx, hx, ns, hs, nrc, hrc,
                  (j < static_cast<int>(implied_integer_cols.size()) &&
                   implied_integer_cols[static_cast<std::size_t>(j)] != 0)
                      ? "impl"
                      : "decl");
            }
          } else if (integer_like && highs_frontier) {
            ++frontier_highs_only;
            if (frontier_highs_emit < diff_terms) {
              append_sep(frontier_highs_sample, frontier_highs_emit++);
              fmt::format_to(
                  std::back_inserter(frontier_highs_sample),
                  "{}:nx{:.12g}:hx{:.12g}:{}>{}:nrc{:.6g}:hrc{:.6g}:{}",
                  j, nx, hx, ns, hs, nrc, hrc,
                  (j < static_cast<int>(implied_integer_cols.size()) &&
                   implied_integer_cols[static_cast<std::size_t>(j)] != 0)
                      ? "impl"
                      : "decl");
            }
          }

          const bool native_zero_rc =
              std::isfinite(nrc) && std::abs(nrc) <= state_tol;
          const bool highs_zero_rc =
              std::isfinite(hrc) && std::abs(hrc) <= state_tol;
          if (integer_like && (native_frontier || highs_frontier) &&
              dx > state_tol && ns == 'B' && hs == 'B' &&
              native_zero_rc && highs_zero_rc) {
            ++degenerate_frontier_value_move;
          }

          if (ns != hs) {
            ++status_diff;
            const bool basic_membership_diff = (ns == 'B') != (hs == 'B');
            const bool lower_upper_flip =
                (ns == 'L' && hs == 'U') || (ns == 'U' && hs == 'L');
            const bool nx_at_lower = std::abs(nx - lb[j]) <= state_tol;
            const bool hx_at_lower = std::abs(hx - lb[j]) <= state_tol;
            const bool nx_at_upper =
                std::isfinite(ub[j]) && std::abs(nx - ub[j]) <= state_tol;
            const bool hx_at_upper =
                std::isfinite(ub[j]) && std::abs(hx - ub[j]) <= state_tol;
            const bool same_value = dx <= state_tol;
            const bool same_bound =
                (nx_at_lower && hx_at_lower) || (nx_at_upper && hx_at_upper);
            const bool on_degenerate_bound =
                same_value && same_bound && (native_zero_rc || highs_zero_rc);
            if (basic_membership_diff) {
              ++basic_diff;
              if (on_degenerate_bound) {
                ++degenerate_basic_boundary_diff;
                if (native_frontier || highs_frontier) {
                  ++frontier_basic_boundary_diff;
                }
              } else {
                ++nondegenerate_status_diff;
              }
            } else if (lower_upper_flip) {
              ++side_flip;
              if (on_degenerate_bound) {
                ++degenerate_side_flip;
              } else {
                ++nondegenerate_status_diff;
              }
            } else {
              ++nondegenerate_status_diff;
            }
            if (status_emit < diff_terms) {
              append_sep(status_sample, status_emit++);
              fmt::format_to(
                  std::back_inserter(status_sample),
                  "{}:{}>{}:nx{:.12g}:hx{:.12g}:nrc{:.6g}:hrc{:.6g}:{}",
                  j, ns, hs, nx, hx, nrc, hrc,
                  integer_like ? "intlike" : "cont");
            }
          }

          if (!integer_like) continue;
          const bool native_nonbasic =
              native_nonbasic_mask[static_cast<std::size_t>(j)] != 0;
          const bool highs_nonbasic = hs != 'B';
          if (native_nonbasic != highs_nonbasic) {
            ++side_integer_diff;
            if (side_diff_emit < diff_terms) {
              append_sep(side_diff_sample, side_diff_emit++);
              fmt::format_to(
                  std::back_inserter(side_diff_sample),
                  "{}:{}>{}:nx{:.12g}:hx{:.12g}:nrc{:.6g}:hrc{:.6g}:{}",
                  j, ns, hs, nx, hx, nrc, hrc,
                  (j < static_cast<int>(implied_integer_cols.size()) &&
                   implied_integer_cols[static_cast<std::size_t>(j)] != 0)
                      ? "impl"
                      : "decl");
            }
          }
        }
        const int status_after_degenerate_remap =
            status_diff - degenerate_side_flip -
            degenerate_basic_boundary_diff;
        fmt::print(stderr,
                   "[LPSTATE-SF-DIFF] phase={} status_diff={} basic_diff={} "
                   "side_flip={} deg_side={} deg_basic={} "
                   "frontier_deg_basic={} deg_frontier_move={} "
                   "nondeg_status={} post_remap_status={} value_diff={} "
                   "max_dx={:.6g}@{} frontier=both{}:native_only{}:"
                   "highs_only{}:status_diff{} side_integer_diff={} "
                   "sample=[{}]\n",
                   phase != nullptr ? phase : "unknown", status_diff,
                   basic_diff, side_flip, degenerate_side_flip,
                   degenerate_basic_boundary_diff,
                   frontier_basic_boundary_diff,
                   degenerate_frontier_value_move,
                   nondegenerate_status_diff, status_after_degenerate_remap,
                   value_diff, max_abs_dx, max_abs_dx_col, frontier_both,
                   frontier_native_only, frontier_highs_only,
                   frontier_status_diff, side_integer_diff,
                   fmt::to_string(status_sample));
        fmt::print(stderr,
                   "[LPSTATE-SF-FRONTIER-DIFF] phase={} native_only=[{}] "
                   "highs_only=[{}] side_diff=[{}]\n",
                   phase != nullptr ? phase : "unknown",
                   fmt::to_string(frontier_native_sample),
                   fmt::to_string(frontier_highs_sample),
                   fmt::to_string(side_diff_sample));
      }
    }
  }
}

void trace_native_repair_bounds(
    const char* phase,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& root_x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    double int_tol,
    const char* detail) {
  if (!bc_frontier_conformance_enabled()) return;
  (void)int_tol;
  const int n = static_cast<int>(lp.vars.size());
  if (root_x.size() != n || lb.size() != n || ub.size() != n) return;
  const std::vector<char> basic = bc_original_basic_mask(n, basis_hint);
  int fixed = 0;
  int free = 0;
  int fractional_free = 0;
  fmt::memory_buffer fixed_sample;
  fmt::memory_buffer free_sample;
  const int max_terms =
      bc_conformance_trace_terms("HACDCPF_FRONTIER_CONFORM_TERMS", 32);
  int fixed_emit = 0;
  int free_emit = 0;
  for (int j = 0; j < n; ++j) {
    if (!is_integer_type(lp.vars[static_cast<std::size_t>(j)])) continue;
    const bool is_fixed =
        std::isfinite(lb[j]) && std::isfinite(ub[j]) &&
        ub[j] - lb[j] <= std::max(1e-9, int_tol);
    const double rounded =
        std::min(ub[j], std::max(lb[j], std::round(root_x[j])));
    const double frac = std::abs(root_x[j] - rounded);
    const char st = bc_native_basis_status(j, basic, basis_hint);
    const bool implied =
        j < static_cast<int>(implied_integer_cols.size()) &&
        implied_integer_cols[static_cast<std::size_t>(j)] != 0;
    if (is_fixed) {
      ++fixed;
      if (fixed_emit < max_terms) {
        if (fixed_emit++ > 0) fmt::format_to(std::back_inserter(fixed_sample), ";");
        fmt::format_to(std::back_inserter(fixed_sample),
                       "{}:{:.12g}:x{:.12g}:f{:.6g}:{}:{}",
                       j, lb[j], root_x[j], frac, st,
                       implied ? "impl" : "decl");
      }
    } else {
      ++free;
      if (frac > int_tol) ++fractional_free;
      if (free_emit < max_terms) {
        if (free_emit++ > 0) fmt::format_to(std::back_inserter(free_sample), ";");
        fmt::format_to(std::back_inserter(free_sample),
                       "{}:x{:.12g}:r{:.12g}:f{:.6g}:{}:{}",
                       j, root_x[j], rounded, frac, st,
                       implied ? "impl" : "decl");
      }
    }
  }
  fmt::print(stderr,
             "[B&C-REPAIR] phase={} fixed={} free={} frac_free={} detail={} "
             "fixed_sample=[{}] free_sample=[{}]\n",
             phase != nullptr ? phase : "unknown", fixed, free,
             fractional_free, detail != nullptr ? detail : "",
             fmt::to_string(fixed_sample), fmt::to_string(free_sample));
}

void trace_native_repair_candidate_order(
    const char* phase,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    const std::vector<int>& ordered_cols,
    double int_tol,
    const char* detail) {
  if (!bc_frontier_conformance_enabled()) return;
  (void)int_tol;
  const int n = static_cast<int>(lp.vars.size());
  if (x.size() != n || lb.size() != n || ub.size() != n) return;
  const std::vector<char> basic = bc_original_basic_mask(n, basis_hint);
  fmt::memory_buffer sample;
  const int max_terms =
      bc_conformance_trace_terms("HACDCPF_FRONTIER_CONFORM_TERMS", 32);
  int emitted = 0;
  int valid = 0;
  for (int col : ordered_cols) {
    if (col < 0 || col >= n) continue;
    ++valid;
    if (emitted >= max_terms) continue;
    const bool implied =
        col < static_cast<int>(implied_integer_cols.size()) &&
        implied_integer_cols[static_cast<std::size_t>(col)] != 0;
    const double rounded =
        std::min(ub[col], std::max(lb[col], std::round(x[col])));
    const double frac = std::abs(x[col] - rounded);
    if (emitted++ > 0) fmt::format_to(std::back_inserter(sample), ";");
    fmt::format_to(std::back_inserter(sample),
                   "{}:{:.12g}:r{:.12g}:f{:.6g}:{}:rc{:.6g}:{}:{}",
                   col, x[col], rounded, frac,
                   bc_native_basis_status(col, basic, basis_hint),
                   bc_native_reduced_cost(col, basis_hint),
                   bc_var_type_name(lp.vars[static_cast<std::size_t>(col)]),
                   implied ? "impl" : "decl");
  }

  fmt::print(stderr,
             "[B&C-REPAIR-CAND] phase={} count={} detail={} sample=[{}]\n",
             phase != nullptr ? phase : "unknown", valid,
             detail != nullptr ? detail : "", fmt::to_string(sample));
}

void trace_native_presolve_state_conformance(
    const char* phase,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const VariableBoundTable& variable_bound_table) {
  if (!bc_frontier_conformance_enabled()) return;
  const int n = static_cast<int>(lp.vars.size());
  int binary_cols = 0;
  int integer_cols = 0;
  int implied_cols = 0;
  int continuous_cols = 0;
  std::uint64_t implied_hash = 0x4841434450434949ULL;
  fmt::memory_buffer implied_sample;
  const int max_terms =
      bc_conformance_trace_terms("HACDCPF_FRONTIER_CONFORM_TERMS", 32);
  int implied_emit = 0;

  for (int j = 0; j < n; ++j) {
    const bool implied =
        j < static_cast<int>(implied_integer_cols.size()) &&
        implied_integer_cols[static_cast<std::size_t>(j)] != 0;
    if (implied) {
      ++implied_cols;
      implied_hash =
          bc_trace_hash_mix(implied_hash, static_cast<std::uint64_t>(j));
      implied_hash = bc_trace_hash_mix(implied_hash,
                                       bc_trace_hash_double(lp.vars[j].lb));
      implied_hash = bc_trace_hash_mix(implied_hash,
                                       bc_trace_hash_double(lp.vars[j].ub));
      if (implied_emit < max_terms) {
        if (implied_emit++ > 0)
          fmt::format_to(std::back_inserter(implied_sample), ";");
        fmt::format_to(std::back_inserter(implied_sample),
                       "{}:lb{:.12g}:ub{:.12g}:{}",
                       j, lp.vars[j].lb, lp.vars[j].ub,
                       bc_var_type_name(lp.vars[j]));
      }
    }
    switch (lp.vars[static_cast<std::size_t>(j)].type) {
      case VarType::Binary:
        ++binary_cols;
        break;
      case VarType::Integer:
        ++integer_cols;
        break;
      case VarType::Continuous:
        ++continuous_cols;
        break;
      default:
        break;
    }
  }

  int ranged_rows = 0;
  std::uint64_t row_hash = 0x484143445043524fULL;
  fmt::memory_buffer row_sample;
  int row_emit = 0;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = lp.A;
  for (int r = 0; r < Arow.rows(); ++r) {
    const double lhs = lp_row_lhs_or_neg_inf(lp, r);
    if (std::isfinite(lhs)) ++ranged_rows;
    row_hash = bc_trace_hash_mix(row_hash, static_cast<std::uint64_t>(r));
    row_hash = bc_trace_hash_mix(row_hash, bc_trace_hash_double(lhs));
    row_hash = bc_trace_hash_mix(row_hash, bc_trace_hash_double(lp.b[r]));
    row_hash = bc_trace_hash_mix(
        row_hash, static_cast<std::uint64_t>(Arow.outerIndexPtr()[r + 1] -
                                             Arow.outerIndexPtr()[r]));
    if (row_emit < max_terms) {
      if (row_emit++ > 0) fmt::format_to(std::back_inserter(row_sample), ";");
      fmt::format_to(std::back_inserter(row_sample),
                     "{}:lhs{:.12g}:rhs{:.12g}:nnz{}",
                     r, lhs, lp.b[r],
                     Arow.outerIndexPtr()[r + 1] - Arow.outerIndexPtr()[r]);
    }
  }

  std::uint64_t vub_hash = 0x4841434450435655ULL;
  std::uint64_t vlb_hash = 0x484143445043564cULL;
  std::uint64_t vub_count = 0;
  std::uint64_t vlb_count = 0;
  fmt::memory_buffer vub_sample;
  fmt::memory_buffer vlb_sample;
  int vub_emit = 0;
  int vlb_emit = 0;
  for (int col = 0; col < variable_bound_table.num_vars(); ++col) {
    for (const auto& entry : variable_bound_table.vubs(col)) {
      ++vub_count;
      vub_hash = bc_trace_hash_mix(vub_hash,
                                   static_cast<std::uint64_t>(col));
      vub_hash = bc_trace_hash_mix(
          vub_hash, static_cast<std::uint64_t>(entry.trigger_col));
      vub_hash = bc_trace_hash_mix(vub_hash,
                                   bc_trace_hash_double(entry.bound.coef));
      vub_hash = bc_trace_hash_mix(
          vub_hash, bc_trace_hash_double(entry.bound.constant));
      if (vub_emit < max_terms) {
        if (vub_emit++ > 0)
          fmt::format_to(std::back_inserter(vub_sample), ";");
        fmt::format_to(std::back_inserter(vub_sample),
                       "{}<= {:.12g}*{} + {:.12g}:min{:.12g}",
                       col, entry.bound.coef, entry.trigger_col,
                       entry.bound.constant, entry.bound.min_value());
      }
    }
    for (const auto& entry : variable_bound_table.vlbs(col)) {
      ++vlb_count;
      vlb_hash = bc_trace_hash_mix(vlb_hash,
                                   static_cast<std::uint64_t>(col));
      vlb_hash = bc_trace_hash_mix(
          vlb_hash, static_cast<std::uint64_t>(entry.trigger_col));
      vlb_hash = bc_trace_hash_mix(vlb_hash,
                                   bc_trace_hash_double(entry.bound.coef));
      vlb_hash = bc_trace_hash_mix(
          vlb_hash, bc_trace_hash_double(entry.bound.constant));
      if (vlb_emit < max_terms) {
        if (vlb_emit++ > 0)
          fmt::format_to(std::back_inserter(vlb_sample), ";");
        fmt::format_to(std::back_inserter(vlb_sample),
                       "{}>= {:.12g}*{} + {:.12g}:max{:.12g}",
                       col, entry.bound.coef, entry.trigger_col,
                       entry.bound.constant, entry.bound.max_value());
      }
    }
  }

  fmt::print(stderr,
             "[B&C-PRESOLVE-STATE] phase={} rows={} cols={} nnz={} "
             "ranged={} eq={} bin={} int={} impl={} cont={} "
             "implied_hash={:016x} row_hash={:016x} "
             "vub={}:{} vlb={}:{} vb_total={} "
             "stats=vub{}/{}/{}:vlb{}/{}/{}:mir{}/{}:red{} "
             "implied_sample=[{}] row_sample=[{}] "
             "vub_sample=[{}] vlb_sample=[{}]\n",
             phase != nullptr ? phase : "presolve_state",
             lp.A.rows(), n, lp.A.nonZeros() + lp.Aeq.nonZeros(),
             ranged_rows, lp.Aeq.rows(), binary_cols, integer_cols,
             implied_cols, continuous_cols, implied_hash, row_hash,
             vub_count, vub_hash, vlb_count, vlb_hash,
             variable_bound_table.size(),
             variable_bound_table.stats().vub_accepted,
             variable_bound_table.stats().vub_attempts,
             variable_bound_table.stats().vub_replaced,
             variable_bound_table.stats().vlb_accepted,
             variable_bound_table.stats().vlb_attempts,
             variable_bound_table.stats().vlb_replaced,
             variable_bound_table.stats().mir_strengthened,
             variable_bound_table.stats().mir_attempts,
             variable_bound_table.stats().redundant,
             fmt::to_string(implied_sample), fmt::to_string(row_sample),
             fmt::to_string(vub_sample), fmt::to_string(vlb_sample));
}


}  // namespace mipsolvers::engine::detail
