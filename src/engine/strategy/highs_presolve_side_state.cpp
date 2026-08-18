/// @file highs_presolve_side_state.cpp
/// @brief In-process HiGHS presolve/conformance bridge.

#include "mipsolvers/engine/strategy/highs_presolve_side_state.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#include "Highs.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace mipsolvers::engine {

namespace {

constexpr double kBridgeInf = 1e20;

std::uint64_t bridge_hash_mix(std::uint64_t seed, std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

std::uint64_t bridge_hash_double(double value) {
  if (std::isinf(value)) {
    return value > 0.0 ? 0x7ff0000000000000ULL : 0xfff0000000000000ULL;
  }
  if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
  return static_cast<std::uint64_t>(
      static_cast<std::int64_t>(std::llround(value * 1e9)));
}

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
std::string highs_model_status_name(HighsModelStatus status) {
  switch (status) {
    case HighsModelStatus::kNotset:
      return "not_set";
    case HighsModelStatus::kLoadError:
      return "load_error";
    case HighsModelStatus::kModelError:
      return "model_error";
    case HighsModelStatus::kPresolveError:
      return "presolve_error";
    case HighsModelStatus::kSolveError:
      return "solve_error";
    case HighsModelStatus::kPostsolveError:
      return "postsolve_error";
    case HighsModelStatus::kModelEmpty:
      return "model_empty";
    case HighsModelStatus::kOptimal:
      return "optimal";
    case HighsModelStatus::kInfeasible:
      return "infeasible";
    case HighsModelStatus::kUnboundedOrInfeasible:
      return "unbounded_or_infeasible";
    case HighsModelStatus::kUnbounded:
      return "unbounded";
    case HighsModelStatus::kObjectiveBound:
      return "objective_bound";
    case HighsModelStatus::kObjectiveTarget:
      return "objective_target";
    case HighsModelStatus::kTimeLimit:
      return "time_limit";
    case HighsModelStatus::kIterationLimit:
      return "iteration_limit";
    default:
      return "other";
  }
}

std::string highs_presolve_status_name(HighsPresolveStatus status) {
  switch (status) {
    case HighsPresolveStatus::kNotSet:
      return "not_set";
    case HighsPresolveStatus::kNotPresolved:
      return "not_presolved";
    case HighsPresolveStatus::kNotReduced:
      return "not_reduced";
    case HighsPresolveStatus::kInfeasible:
      return "infeasible";
    case HighsPresolveStatus::kUnboundedOrInfeasible:
      return "unbounded_or_infeasible";
    case HighsPresolveStatus::kReduced:
      return "reduced";
    case HighsPresolveStatus::kReducedToEmpty:
      return "reduced_to_empty";
    case HighsPresolveStatus::kTimeout:
      return "timeout";
    case HighsPresolveStatus::kOutOfMemory:
      return "out_of_memory";
    case HighsPresolveStatus::kNullError:
      return "null_error";
    case HighsPresolveStatus::kOptionsError:
      return "options_error";
  }
  return "unknown";
}

double highs_bound_or_inf(double value, double inf_sign) {
  if (!std::isfinite(value)) return inf_sign < 0.0 ? -kHighsInf : kHighsInf;
  if (value <= -kBridgeInf) return -kHighsInf;
  if (value >= kBridgeInf) return kHighsInf;
  return value;
}

double native_bound_or_inf(double value, double inf_sign) {
  if (!std::isfinite(value)) {
    return inf_sign < 0.0 ? -std::numeric_limits<double>::infinity()
                          : std::numeric_limits<double>::infinity();
  }
  if (value <= -0.5 * kHighsInf) {
    return -std::numeric_limits<double>::infinity();
  }
  if (value >= 0.5 * kHighsInf) {
    return std::numeric_limits<double>::infinity();
  }
  return value;
}

bool highs_finite_lower(double value) {
  return std::isfinite(value) && value > -0.5 * kHighsInf;
}

bool highs_finite_upper(double value) {
  return std::isfinite(value) && value < 0.5 * kHighsInf;
}

VarType native_var_type_from_highs(HighsVarType type,
                                   double lb,
                                   double ub) {
  switch (type) {
    case HighsVarType::kInteger:
    case HighsVarType::kSemiInteger:
      if (highs_finite_lower(lb) && highs_finite_upper(ub) &&
          std::abs(lb) <= 1e-9 && std::abs(ub - 1.0) <= 1e-9) {
        return VarType::Binary;
      }
      return VarType::Integer;
    case HighsVarType::kImplicitInteger:
    case HighsVarType::kContinuous:
    case HighsVarType::kSemiContinuous:
      return VarType::Continuous;
  }
  return VarType::Continuous;
}

LPModel convert_highs_presolved_lp_to_native(const HighsLp& presolved,
                                             Sense native_sense) {
  LPModel out;
  out.sense = native_sense;
  const int ncols = static_cast<int>(presolved.num_col_);
  const int nrows = static_cast<int>(presolved.num_row_);
  out.vars.resize(static_cast<std::size_t>(std::max(0, ncols)));
  out.c = Eigen::VectorXd::Zero(std::max(0, ncols));
  for (int j = 0; j < ncols; ++j) {
    const double lb =
        j < static_cast<int>(presolved.col_lower_.size())
            ? presolved.col_lower_[static_cast<std::size_t>(j)]
            : -kHighsInf;
    const double ub =
        j < static_cast<int>(presolved.col_upper_.size())
            ? presolved.col_upper_[static_cast<std::size_t>(j)]
            : kHighsInf;
    const double cost =
        j < static_cast<int>(presolved.col_cost_.size())
            ? presolved.col_cost_[static_cast<std::size_t>(j)]
            : 0.0;
    HighsVarType type = HighsVarType::kContinuous;
    if (j < static_cast<int>(presolved.integrality_.size())) {
      type = presolved.integrality_[static_cast<std::size_t>(j)];
    }
    out.c[j] = native_sense == Sense::Maximize ? -cost : cost;
    auto& v = out.vars[static_cast<std::size_t>(j)];
    v.lb = native_bound_or_inf(lb, -1.0);
    v.ub = native_bound_or_inf(ub, 1.0);
    v.type = native_var_type_from_highs(type, lb, ub);
    v.name = j < static_cast<int>(presolved.col_names_.size())
                 ? presolved.col_names_[static_cast<std::size_t>(j)]
                 : std::string();
  }

  std::vector<Eigen::Triplet<double>> row_trips;
  row_trips.reserve(static_cast<std::size_t>(presolved.a_matrix_.numNz()));
  std::vector<Eigen::Triplet<double>> eq_trips;
  eq_trips.reserve(static_cast<std::size_t>(presolved.a_matrix_.numNz()));
  std::vector<int> row_to_native(static_cast<std::size_t>(std::max(0, nrows)),
                                 -1);
  std::vector<int> row_to_native_eq(
      static_cast<std::size_t>(std::max(0, nrows)), -1);
  std::vector<int> highs_row_to_native_row(
      static_cast<std::size_t>(std::max(0, nrows)), -1);
  std::vector<double> row_lhs;
  std::vector<double> row_rhs;
  std::vector<double> eq_rhs;
  row_lhs.reserve(static_cast<std::size_t>(std::max(0, nrows)));
  row_rhs.reserve(static_cast<std::size_t>(std::max(0, nrows)));

  for (int r = 0; r < nrows; ++r) {
    const double lhs =
        r < static_cast<int>(presolved.row_lower_.size())
            ? presolved.row_lower_[static_cast<std::size_t>(r)]
            : -kHighsInf;
    const double rhs =
        r < static_cast<int>(presolved.row_upper_.size())
            ? presolved.row_upper_[static_cast<std::size_t>(r)]
            : kHighsInf;
    const bool has_lhs = highs_finite_lower(lhs);
    const bool has_rhs = highs_finite_upper(rhs);
    if (!has_lhs && !has_rhs) continue;
    if (has_lhs && has_rhs && lhs == rhs) {
      const int native_eq = static_cast<int>(eq_rhs.size());
      row_to_native_eq[static_cast<std::size_t>(r)] = native_eq;
      eq_rhs.push_back(rhs);
      continue;
    }
    const int native_row = static_cast<int>(row_rhs.size());
    highs_row_to_native_row[static_cast<std::size_t>(r)] = native_row;
    row_to_native[static_cast<std::size_t>(r)] = native_row;
    row_lhs.push_back(has_lhs ? lhs : -std::numeric_limits<double>::infinity());
    row_rhs.push_back(has_rhs ? rhs : std::numeric_limits<double>::infinity());
  }

  HighsSparseMatrix matrix = presolved.a_matrix_;
  matrix.ensureColwise();
  if (static_cast<int>(matrix.start_.size()) >= ncols + 1) {
    for (int col = 0; col < ncols; ++col) {
      const HighsInt start = matrix.start_[static_cast<std::size_t>(col)];
      const HighsInt end = matrix.start_[static_cast<std::size_t>(col + 1)];
      for (HighsInt p = start; p < end; ++p) {
        if (p < 0 || p >= static_cast<HighsInt>(matrix.index_.size()) ||
            p >= static_cast<HighsInt>(matrix.value_.size())) {
          continue;
        }
        const int row = static_cast<int>(matrix.index_[static_cast<std::size_t>(p)]);
        const double value = matrix.value_[static_cast<std::size_t>(p)];
        if (row < 0 || row >= nrows || value == 0.0) continue;
        const int native_row = row_to_native[static_cast<std::size_t>(row)];
        if (native_row >= 0) {
          row_trips.emplace_back(native_row, col, value);
          continue;
        }
        const int native_eq = row_to_native_eq[static_cast<std::size_t>(row)];
        if (native_eq >= 0) {
          eq_trips.emplace_back(native_eq, col, value);
        }
      }
    }
  }

  const int m_rows = static_cast<int>(row_rhs.size());
  out.A.resize(m_rows, ncols);
  out.A.setFromTriplets(row_trips.begin(), row_trips.end());
  out.A.makeCompressed();
  out.row_lhs.resize(m_rows);
  out.b.resize(m_rows);
  for (int r = 0; r < m_rows; ++r) {
    out.row_lhs[r] = row_lhs[static_cast<std::size_t>(r)];
    out.b[r] = row_rhs[static_cast<std::size_t>(r)];
  }
  const int m_eq = static_cast<int>(eq_rhs.size());
  out.Aeq.resize(m_eq, ncols);
  out.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  out.Aeq.makeCompressed();
  out.beq.resize(m_eq);
  for (int r = 0; r < m_eq; ++r)
    out.beq[r] = eq_rhs[static_cast<std::size_t>(r)];
  for (int r = 0; r < nrows; ++r) {
    const int native_eq = row_to_native_eq[static_cast<std::size_t>(r)];
    if (native_eq >= 0) {
      highs_row_to_native_row[static_cast<std::size_t>(r)] =
          m_rows + native_eq;
    }
  }
  out.highs_row_to_native_row = std::move(highs_row_to_native_row);
  HighsSparseMatrix row_matrix = presolved.a_matrix_;
  row_matrix.ensureRowwise();
  out.highs_row_start.reserve(row_matrix.start_.size());
  for (const HighsInt v : row_matrix.start_) {
    out.highs_row_start.push_back(static_cast<int>(v));
  }
  out.highs_row_index.reserve(row_matrix.index_.size());
  for (const HighsInt v : row_matrix.index_) {
    out.highs_row_index.push_back(static_cast<int>(v));
  }
  out.highs_row_value = row_matrix.value_;
  return out;
}

char highs_basis_status_char(HighsBasisStatus status) {
  switch (status) {
    case HighsBasisStatus::kBasic:
      return 'B';
    case HighsBasisStatus::kUpper:
      return 'U';
    case HighsBasisStatus::kLower:
      return 'L';
    case HighsBasisStatus::kZero:
      return 'Z';
    case HighsBasisStatus::kNonbasic:
      return 'N';
  }
  return '?';
}
#endif

}  // namespace

namespace {

void fill_projected_lp_state_stats(
    HiGHSRootLpStateStats& stats,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    double int_tol,
    int max_terms,
    const std::vector<char>& col_status,
    const std::vector<double>& col_value,
    const std::vector<double>& col_dual) {
  const int ncols = static_cast<int>(lp.vars.size());
  stats.basis_valid = static_cast<int>(col_status.size()) >= ncols;
  if (static_cast<int>(col_value.size()) < ncols) return;
  stats.col_status.assign(static_cast<std::size_t>(ncols), '?');
  stats.col_value.assign(static_cast<std::size_t>(ncols),
                         std::numeric_limits<double>::quiet_NaN());
  stats.col_dual.assign(static_cast<std::size_t>(ncols),
                        std::numeric_limits<double>::quiet_NaN());
  stats.status_hash = 0x4841434450435354ULL;
  stats.frontier_hash = 0x4841434450434655ULL;
  std::ostringstream frontier_sample;
  std::ostringstream side_sample;
  int frontier_emit = 0;
  int side_emit = 0;
  max_terms = std::max(0, std::min(max_terms, 256));

  for (int j = 0; j < ncols; ++j) {
    const char status =
        j < static_cast<int>(col_status.size())
            ? col_status[static_cast<std::size_t>(j)]
            : '?';
    const double xj = col_value[static_cast<std::size_t>(j)];
    const double rc = j < static_cast<int>(col_dual.size())
                          ? col_dual[static_cast<std::size_t>(j)]
                          : std::numeric_limits<double>::quiet_NaN();
    stats.col_status[static_cast<std::size_t>(j)] = status;
    stats.col_value[static_cast<std::size_t>(j)] = xj;
    stats.col_dual[static_cast<std::size_t>(j)] = rc;
    stats.status_hash = bridge_hash_mix(
        stats.status_hash, static_cast<std::uint64_t>(j));
    stats.status_hash = bridge_hash_mix(
        stats.status_hash, static_cast<std::uint64_t>(status));
    stats.status_hash = bridge_hash_mix(stats.status_hash,
                                        bridge_hash_double(xj));

    switch (status) {
      case 'B':
        ++stats.basic_cols;
        break;
      case 'U':
        ++stats.nonbasic_upper;
        break;
      case 'L':
        ++stats.nonbasic_lower;
        break;
      case 'Z':
        ++stats.nonbasic_zero;
        break;
      default:
        ++stats.nonbasic_other;
        break;
    }
    const bool is_fixed =
        std::isfinite(lp.vars[static_cast<std::size_t>(j)].lb) &&
        std::isfinite(lp.vars[static_cast<std::size_t>(j)].ub) &&
        lp.vars[static_cast<std::size_t>(j)].ub -
                lp.vars[static_cast<std::size_t>(j)].lb <=
            std::max(1e-9, int_tol);
    if (is_fixed) ++stats.fixed_cols;
    const bool implied =
        j < static_cast<int>(implied_integer_cols.size()) &&
        implied_integer_cols[static_cast<std::size_t>(j)] != 0;
    const bool integer_like =
        lp.vars[static_cast<std::size_t>(j)].type == VarType::Binary ||
        lp.vars[static_cast<std::size_t>(j)].type == VarType::Integer ||
        implied;
    if (!integer_like) continue;
    ++stats.integer_like_total;
    if (status != 'B') {
      ++stats.nonbasic_integer_like;
      if (side_emit < max_terms) {
        if (side_emit++ > 0) side_sample << ";";
        side_sample << j << ":" << xj << ":lb"
                    << lp.vars[static_cast<std::size_t>(j)].lb << ":ub"
                    << lp.vars[static_cast<std::size_t>(j)].ub << ":" << status
                    << ":rc" << rc << ":"
                    << (lp.vars[static_cast<std::size_t>(j)].type ==
                                VarType::Binary
                            ? "bin"
                            : (lp.vars[static_cast<std::size_t>(j)].type ==
                                       VarType::Integer
                                   ? "int"
                                   : "cont"))
                    << ":" << (implied ? "impl" : "decl");
      }
    }
    const double rounded = std::min(
        lp.vars[static_cast<std::size_t>(j)].ub,
        std::max(lp.vars[static_cast<std::size_t>(j)].lb, std::round(xj)));
    const double frac = std::abs(xj - rounded);
    if (!(frac > int_tol)) continue;
    ++stats.fractional_total;
    if (lp.vars[static_cast<std::size_t>(j)].type == VarType::Binary) {
      ++stats.fractional_binary;
    } else if (lp.vars[static_cast<std::size_t>(j)].type == VarType::Integer) {
      ++stats.fractional_integer;
    } else if (implied) {
      ++stats.fractional_implied;
    }
    stats.frontier_hash = bridge_hash_mix(
        stats.frontier_hash, static_cast<std::uint64_t>(j));
    stats.frontier_hash =
        bridge_hash_mix(stats.frontier_hash, bridge_hash_double(xj));
    stats.frontier_hash = bridge_hash_mix(
        stats.frontier_hash, static_cast<std::uint64_t>(status));
    if (frontier_emit < max_terms) {
      if (frontier_emit++ > 0) frontier_sample << ";";
      frontier_sample << j << ":" << xj << ":r" << rounded << ":f" << frac
                      << ":" << status << ":rc" << rc << ":"
                      << (lp.vars[static_cast<std::size_t>(j)].type ==
                                  VarType::Binary
                              ? "bin"
                              : (lp.vars[static_cast<std::size_t>(j)].type ==
                                         VarType::Integer
                                     ? "int"
                                     : "cont"))
                      << ":" << (implied ? "impl" : "decl");
    }
  }
  stats.frontier_sample = frontier_sample.str();
  stats.side_sample = side_sample.str();
}

}  // namespace

HiGHSPresolveBridgeInfo highs_presolve_bridge_info() {
  HiGHSPresolveBridgeInfo info;
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  info.available = true;
#ifdef MIPSOLVERS_HIGHS_LIB_SOURCE
  info.source = MIPSOLVERS_HIGHS_LIB_SOURCE;
#else
  info.source = "linked";
#endif
  info.embedded_source = (info.source == "embedded-source");
  info.version = highsVersion();
  info.githash = highsGithash();
#else
  info.available = false;
  info.source = "unavailable";
#endif
  return info;
}

HiGHSPresolvedModelStats highs_presolve_model_stats(const LPModel& lp,
                                                    double time_limit_sec,
                                                    bool retain_postsolve) {
  HiGHSPresolvedModelStats stats;
  const auto bridge = highs_presolve_bridge_info();
  stats.available = bridge.available;
  stats.bridge_source = bridge.source;
  stats.original_cols = static_cast<int>(lp.vars.size());

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  if (!bridge.available) return stats;

  const auto start_time = std::chrono::steady_clock::now();
  auto remaining_time = [&]() {
    if (!(time_limit_sec > 0.0) || !std::isfinite(time_limit_sec)) {
      return std::numeric_limits<double>::infinity();
    }
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();
    return time_limit_sec - elapsed;
  };
  auto deadline_expired = [&]() {
    return time_limit_sec > 0.0 && std::isfinite(time_limit_sec) &&
           remaining_time() <= 0.0;
  };
  auto mark_timeout = [&]() {
    stats.presolve_ok = false;
    stats.highs_status = "timeout";
  };

  const int ncols = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int nrows = m_ineq + m_eq;
  if (ncols == 0) {
    stats.pass_ok = true;
    stats.presolve_ok = true;
    stats.highs_status = "empty";
    return stats;
  }
  if (lp.c.size() != ncols || lp.b.size() != m_ineq ||
      lp.beq.size() != m_eq || (m_ineq > 0 && lp.A.cols() != ncols) ||
      (m_eq > 0 && lp.Aeq.cols() != ncols)) {
    stats.highs_status = "invalid_dimensions";
    return stats;
  }

  std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(ncols), -kHighsInf);
  std::vector<double> col_upper(static_cast<std::size_t>(ncols), kHighsInf);
  std::vector<HighsInt> integrality(static_cast<std::size_t>(ncols),
                                    static_cast<HighsInt>(HighsVarType::kContinuous));
  for (int j = 0; j < ncols; ++j) {
    if ((j & 255) == 0 && deadline_expired()) {
      mark_timeout();
      return stats;
    }
    col_cost[static_cast<std::size_t>(j)] =
        (lp.sense == Sense::Maximize ? -lp.c[j] : lp.c[j]);
    col_lower[static_cast<std::size_t>(j)] =
        highs_bound_or_inf(lp.vars[static_cast<std::size_t>(j)].lb, -1.0);
    col_upper[static_cast<std::size_t>(j)] =
        highs_bound_or_inf(lp.vars[static_cast<std::size_t>(j)].ub, 1.0);
    if (lp.vars[static_cast<std::size_t>(j)].type != VarType::Continuous) {
      integrality[static_cast<std::size_t>(j)] =
          static_cast<HighsInt>(HighsVarType::kInteger);
    }
  }

  std::vector<double> row_lower(static_cast<std::size_t>(nrows), -kHighsInf);
  std::vector<double> row_upper(static_cast<std::size_t>(nrows), kHighsInf);
  for (int r = 0; r < m_ineq; ++r) {
    row_lower[static_cast<std::size_t>(r)] =
        highs_bound_or_inf(lp_row_lhs_or_neg_inf(lp, r), -1.0);
    row_upper[static_cast<std::size_t>(r)] = highs_bound_or_inf(lp.b[r], 1.0);
  }
  for (int r = 0; r < m_eq; ++r) {
    const int rr = m_ineq + r;
    const double rhs = highs_bound_or_inf(lp.beq[r], 1.0);
    row_lower[static_cast<std::size_t>(rr)] = rhs;
    row_upper[static_cast<std::size_t>(rr)] = rhs;
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(lp.A.nonZeros() + lp.Aeq.nonZeros()));
  value.reserve(index.capacity());
  for (int j = 0; j < ncols; ++j) {
    if ((j & 255) == 0 && deadline_expired()) {
      mark_timeout();
      return stats;
    }
    start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(index.size());
    if (m_ineq > 0) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
        if (it.value() == 0.0) continue;
        index.push_back(static_cast<HighsInt>(it.row()));
        value.push_back(it.value());
      }
    }
    if (m_eq > 0) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
        if (it.value() == 0.0) continue;
        index.push_back(static_cast<HighsInt>(m_ineq + it.row()));
        value.push_back(it.value());
      }
    }
  }
  start[static_cast<std::size_t>(ncols)] = static_cast<HighsInt>(index.size());

  // shared_ptr so a retain_postsolve pass can hand the presolve-only
  // instance (and its intact postsolve lifecycle) to the side-state struct.
  auto highs_owner = std::make_shared<Highs>();
  Highs& highs = *highs_owner;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("log_to_console", false);
  highs.setOptionValue("threads", 1);
  const auto pass_status = highs.passModel(
      static_cast<HighsInt>(ncols), static_cast<HighsInt>(nrows),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMinimize), 0.0, col_cost.data(),
      col_lower.data(), col_upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), integrality.data());
  stats.pass_ok = (pass_status == HighsStatus::kOk);
  if (!stats.pass_ok) {
    stats.highs_status = "pass_error";
    return stats;
  }

  const double presolve_budget = remaining_time();
  if (presolve_budget <= 0.0) {
    mark_timeout();
    return stats;
  }
  if (std::isfinite(presolve_budget)) {
    highs.setOptionValue("time_limit", std::max(0.001, presolve_budget));
  }

  const auto presolve_status = highs.presolve();
  stats.presolve_ok = (presolve_status == HighsStatus::kOk ||
                       presolve_status == HighsStatus::kWarning);
  stats.highs_status =
      highs_presolve_status_name(highs.getModelPresolveStatus());
  if (stats.highs_status == "timeout" || deadline_expired()) {
    mark_timeout();
    return stats;
  }
  if (!stats.presolve_ok) return stats;

  const HighsLp& presolved = highs.getPresolvedLp();
  stats.presolved_objective_offset = presolved.offset_;
  if (presolved.num_col_ >= 0 && presolved.num_row_ >= 0 &&
      static_cast<int>(presolved.col_cost_.size()) >= presolved.num_col_ &&
      static_cast<int>(presolved.col_lower_.size()) >= presolved.num_col_ &&
      static_cast<int>(presolved.col_upper_.size()) >= presolved.num_col_ &&
      static_cast<int>(presolved.row_lower_.size()) >= presolved.num_row_ &&
      static_cast<int>(presolved.row_upper_.size()) >= presolved.num_row_) {
    stats.presolved_lp = convert_highs_presolved_lp_to_native(presolved,
                                                              lp.sense);
    stats.presolved_lp_available =
        static_cast<int>(stats.presolved_lp.vars.size()) ==
            static_cast<int>(presolved.num_col_) &&
        static_cast<int>(stats.presolved_lp.A.rows()) +
                static_cast<int>(stats.presolved_lp.Aeq.rows()) <=
            static_cast<int>(presolved.num_row_);
  }
  if (retain_postsolve && stats.presolved_lp_available &&
      stats.highs_status == "reduced") {
    // Presolve-only lifecycle: Highs::postsolve stays valid for kReduced, so
    // this instance can publish any working-space primal in original
    // coordinates. Derivation: docs/native_milp_root_quality_restart_
    // prerequisites_2026-08-13.md, "Strict Tree-Incumbent Postsolve
    // Ownership".
    stats.impl = highs_owner;
    stats.impl_mutex = std::make_shared<std::mutex>();
  }

  const auto& side_state = highs.getPresolveSideState();
  if (side_state.available) {
    stats.side_state_available = true;
    stats.rows = static_cast<int>(side_state.rows);
    stats.cols = static_cast<int>(side_state.cols);
    stats.nnz = static_cast<int>(side_state.nnz);
    stats.ranged_rows = static_cast<int>(side_state.ranged_rows);
    stats.binary_cols = static_cast<int>(side_state.binary_cols);
    stats.integer_cols = static_cast<int>(side_state.integer_cols);
    stats.implied_integer_cols =
        static_cast<int>(side_state.implied_integer_cols);
    stats.continuous_cols = static_cast<int>(side_state.continuous_cols);
    stats.fixed_cols = static_cast<int>(side_state.fixed_cols);
    stats.vub_count = static_cast<int>(side_state.vub_count);
    stats.vlb_count = static_cast<int>(side_state.vlb_count);
    stats.vub_attempts = static_cast<int>(side_state.vub_attempts);
    stats.vub_accepted = static_cast<int>(side_state.vub_accepted);
    stats.vub_replaced = static_cast<int>(side_state.vub_replaced);
    stats.vlb_attempts = static_cast<int>(side_state.vlb_attempts);
    stats.vlb_accepted = static_cast<int>(side_state.vlb_accepted);
    stats.vlb_replaced = static_cast<int>(side_state.vlb_replaced);
    stats.probing_calls = static_cast<int>(side_state.probing_calls);
    stats.probing_conflicts = static_cast<int>(side_state.probing_conflicts);
    stats.probing_reductions = static_cast<int>(side_state.probing_reductions);
    stats.probing_substitutions =
        static_cast<int>(side_state.probing_substitutions);
    stats.vub_hash = side_state.vub_hash;
    stats.vlb_hash = side_state.vlb_hash;
    stats.presolved_row_lower = side_state.presolved_row_lower;
    stats.presolved_row_upper = side_state.presolved_row_upper;
    stats.presolved_col_lower = side_state.presolved_col_lower;
    stats.presolved_col_upper = side_state.presolved_col_upper;
    stats.presolved_col_orig.reserve(side_state.presolved_col_orig.size());
    for (const HighsInt orig : side_state.presolved_col_orig) {
      stats.presolved_col_orig.push_back(static_cast<int>(orig));
    }
    stats.presolved_col_type.reserve(side_state.presolved_col_type.size());
    for (const HighsVarType type : side_state.presolved_col_type) {
      stats.presolved_col_type.push_back(static_cast<unsigned char>(type));
    }
    stats.presolved_col_scale = side_state.presolved_col_scale;
    stats.presolved_col_constant = side_state.presolved_col_constant;
    stats.presolved_col_linearly_transformable =
        side_state.presolved_col_linearly_transformable;
    stats.presolved_a_start.reserve(side_state.presolved_a_start.size());
    for (const HighsInt v : side_state.presolved_a_start) {
      stats.presolved_a_start.push_back(static_cast<int>(v));
    }
    stats.presolved_a_index.reserve(side_state.presolved_a_index.size());
    for (const HighsInt v : side_state.presolved_a_index) {
      stats.presolved_a_index.push_back(static_cast<int>(v));
    }
    stats.presolved_a_value = side_state.presolved_a_value;
    stats.var_bounds.reserve(side_state.var_bounds.size());
    for (const auto& rec : side_state.var_bounds) {
      HiGHSPresolvedModelStats::VarBoundRecord out;
      out.target_col = static_cast<int>(rec.target_col);
      out.trigger_col = static_cast<int>(rec.trigger_col);
      out.target_orig_col = static_cast<int>(rec.target_orig_col);
      out.trigger_orig_col = static_cast<int>(rec.trigger_orig_col);
      out.coef = rec.coef;
      out.constant = rec.constant;
      out.target_scale = rec.target_scale;
      out.target_constant = rec.target_constant;
      out.trigger_scale = rec.trigger_scale;
      out.trigger_constant = rec.trigger_constant;
      out.upper = rec.upper;
      out.target_linearly_transformable = rec.target_linearly_transformable;
      out.trigger_linearly_transformable = rec.trigger_linearly_transformable;
      stats.var_bounds.push_back(out);
    }
  } else {
    stats.rows = static_cast<int>(presolved.num_row_);
    stats.cols = static_cast<int>(presolved.num_col_);
    stats.nnz = static_cast<int>(presolved.a_matrix_.numNz());
    const int row_count = static_cast<int>(presolved.row_lower_.size());
    for (int r = 0; r < row_count; ++r) {
      const double lhs = presolved.row_lower_[static_cast<std::size_t>(r)];
      const double rhs = presolved.row_upper_[static_cast<std::size_t>(r)];
      if (std::isfinite(lhs) && std::isfinite(rhs) &&
          std::abs(lhs - rhs) > 1e-9) {
        ++stats.ranged_rows;
      }
    }
    const int col_count = static_cast<int>(presolved.col_lower_.size());
    for (int j = 0; j < col_count; ++j) {
      const double lb = presolved.col_lower_[static_cast<std::size_t>(j)];
      const double ub = presolved.col_upper_[static_cast<std::size_t>(j)];
      if (std::isfinite(lb) && std::isfinite(ub) &&
          std::abs(lb - ub) <= 1e-9) {
        ++stats.fixed_cols;
      }
      HighsVarType type = HighsVarType::kContinuous;
      if (j < static_cast<int>(presolved.integrality_.size())) {
        type = presolved.integrality_[static_cast<std::size_t>(j)];
      }
      switch (type) {
        case HighsVarType::kContinuous:
        case HighsVarType::kSemiContinuous:
          ++stats.continuous_cols;
          break;
        case HighsVarType::kImplicitInteger:
          ++stats.implied_integer_cols;
          break;
        case HighsVarType::kInteger:
        case HighsVarType::kSemiInteger:
          if (std::isfinite(lb) && std::isfinite(ub) &&
              std::abs(lb) <= 1e-9 && std::abs(ub - 1.0) <= 1e-9) {
            ++stats.binary_cols;
          } else {
            ++stats.integer_cols;
          }
          break;
      }
    }
  }
#else
  stats.highs_status = "unavailable";
#endif

  return stats;
}

HiGHSRootLpStateStats highs_root_lp_state_stats(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    double int_tol,
    int max_terms) {
  HiGHSRootLpStateStats stats;
  const auto bridge = highs_presolve_bridge_info();
  stats.available = bridge.available;
  stats.bridge_source = bridge.source;

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  if (!bridge.available) return stats;

  const int ncols = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int nrows = m_ineq + m_eq;
  stats.rows = nrows;
  stats.cols = ncols;
  stats.nnz = static_cast<int>(lp.A.nonZeros() + lp.Aeq.nonZeros());
  if (ncols == 0) {
    stats.pass_ok = true;
    stats.solve_ok = true;
    stats.model_status = "empty";
    return stats;
  }

  std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(ncols), -kHighsInf);
  std::vector<double> col_upper(static_cast<std::size_t>(ncols), kHighsInf);
  for (int j = 0; j < ncols; ++j) {
    col_cost[static_cast<std::size_t>(j)] =
        (lp.sense == Sense::Maximize ? -lp.c[j] : lp.c[j]);
    col_lower[static_cast<std::size_t>(j)] =
        highs_bound_or_inf(lp.vars[static_cast<std::size_t>(j)].lb, -1.0);
    col_upper[static_cast<std::size_t>(j)] =
        highs_bound_or_inf(lp.vars[static_cast<std::size_t>(j)].ub, 1.0);
  }

  std::vector<double> row_lower(static_cast<std::size_t>(nrows), -kHighsInf);
  std::vector<double> row_upper(static_cast<std::size_t>(nrows), kHighsInf);
  for (int r = 0; r < m_ineq; ++r) {
    row_lower[static_cast<std::size_t>(r)] =
        highs_bound_or_inf(lp_row_lhs_or_neg_inf(lp, r), -1.0);
    row_upper[static_cast<std::size_t>(r)] = highs_bound_or_inf(lp.b[r], 1.0);
  }
  for (int r = 0; r < m_eq; ++r) {
    const int rr = m_ineq + r;
    const double rhs = highs_bound_or_inf(lp.beq[r], 1.0);
    row_lower[static_cast<std::size_t>(rr)] = rhs;
    row_upper[static_cast<std::size_t>(rr)] = rhs;
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(stats.nnz));
  value.reserve(index.capacity());
  for (int j = 0; j < ncols; ++j) {
    start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(index.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(it.row()));
      value.push_back(it.value());
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(m_ineq + it.row()));
      value.push_back(it.value());
    }
  }
  start[static_cast<std::size_t>(ncols)] = static_cast<HighsInt>(index.size());

  Highs highs;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("log_to_console", false);
  highs.setOptionValue("threads", 1);
  highs.setOptionValue("presolve", "off");
  highs.setOptionValue("solver", "simplex");
  highs.setOptionValue("simplex_strategy", 1);
  const auto pass_status = highs.passModel(
      static_cast<HighsInt>(ncols), static_cast<HighsInt>(nrows),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMinimize), 0.0, col_cost.data(),
      col_lower.data(), col_upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), nullptr);
  stats.pass_ok = (pass_status == HighsStatus::kOk);
  if (!stats.pass_ok) {
    stats.model_status = "pass_error";
    return stats;
  }

  const auto run_status = highs.run();
  stats.model_status = highs_model_status_name(highs.getModelStatus());
  stats.solve_ok =
      run_status == HighsStatus::kOk &&
      highs.getModelStatus() == HighsModelStatus::kOptimal;
  stats.objective = lp.sense == Sense::Maximize
                        ? -highs.getInfo().objective_function_value
                        : highs.getInfo().objective_function_value;
  stats.iterations = static_cast<int>(highs.getInfo().simplex_iteration_count);
  if (!stats.solve_ok) return stats;

  const HighsSolution& sol = highs.getSolution();
  const HighsBasis& basis = highs.getBasis();
  stats.basis_valid = basis.valid &&
                      static_cast<int>(basis.col_status.size()) >= ncols;
  if (static_cast<int>(sol.col_value.size()) < ncols) return stats;

  stats.col_status.assign(static_cast<std::size_t>(ncols), '?');
  stats.col_value.assign(static_cast<std::size_t>(ncols),
                         std::numeric_limits<double>::quiet_NaN());
  stats.col_dual.assign(static_cast<std::size_t>(ncols),
                        std::numeric_limits<double>::quiet_NaN());
  stats.status_hash = 0x4841434450435354ULL;
  stats.frontier_hash = 0x4841434450434655ULL;
  std::ostringstream frontier_sample;
  std::ostringstream side_sample;
  int frontier_emit = 0;
  int side_emit = 0;
  max_terms = std::max(0, std::min(max_terms, 256));

  for (int j = 0; j < ncols; ++j) {
    const HighsBasisStatus raw_status =
        (stats.basis_valid ? basis.col_status[static_cast<std::size_t>(j)]
                           : HighsBasisStatus::kNonbasic);
    const char status = highs_basis_status_char(raw_status);
    const double xj = sol.col_value[static_cast<std::size_t>(j)];
    const double rc = (static_cast<int>(sol.col_dual.size()) > j)
                          ? sol.col_dual[static_cast<std::size_t>(j)]
                          : std::numeric_limits<double>::quiet_NaN();
    stats.col_status[static_cast<std::size_t>(j)] = status;
    stats.col_value[static_cast<std::size_t>(j)] = xj;
    stats.col_dual[static_cast<std::size_t>(j)] = rc;
    stats.status_hash = bridge_hash_mix(
        stats.status_hash, static_cast<std::uint64_t>(j));
    stats.status_hash = bridge_hash_mix(
        stats.status_hash, static_cast<std::uint64_t>(status));
    stats.status_hash = bridge_hash_mix(stats.status_hash,
                                        bridge_hash_double(xj));

    switch (raw_status) {
      case HighsBasisStatus::kBasic:
        ++stats.basic_cols;
        break;
      case HighsBasisStatus::kUpper:
        ++stats.nonbasic_upper;
        break;
      case HighsBasisStatus::kLower:
        ++stats.nonbasic_lower;
        break;
      case HighsBasisStatus::kZero:
        ++stats.nonbasic_zero;
        break;
      case HighsBasisStatus::kNonbasic:
        ++stats.nonbasic_other;
        break;
    }
    const bool is_fixed =
        std::isfinite(lp.vars[static_cast<std::size_t>(j)].lb) &&
        std::isfinite(lp.vars[static_cast<std::size_t>(j)].ub) &&
        lp.vars[static_cast<std::size_t>(j)].ub -
                lp.vars[static_cast<std::size_t>(j)].lb <=
            std::max(1e-9, int_tol);
    if (is_fixed) ++stats.fixed_cols;
    const bool implied =
        j < static_cast<int>(implied_integer_cols.size()) &&
        implied_integer_cols[static_cast<std::size_t>(j)] != 0;
    const bool integer_like =
        lp.vars[static_cast<std::size_t>(j)].type == VarType::Binary ||
        lp.vars[static_cast<std::size_t>(j)].type == VarType::Integer ||
        implied;
    if (!integer_like) continue;
    ++stats.integer_like_total;
    if (raw_status != HighsBasisStatus::kBasic) {
      ++stats.nonbasic_integer_like;
      if (side_emit < max_terms) {
        if (side_emit++ > 0) side_sample << ";";
        side_sample << j << ":" << xj << ":lb"
                    << lp.vars[static_cast<std::size_t>(j)].lb << ":ub"
                    << lp.vars[static_cast<std::size_t>(j)].ub << ":" << status
                    << ":rc" << rc << ":"
                    << (lp.vars[static_cast<std::size_t>(j)].type ==
                                VarType::Binary
                            ? "bin"
                            : (lp.vars[static_cast<std::size_t>(j)].type ==
                                       VarType::Integer
                                   ? "int"
                                   : "cont"))
                    << ":" << (implied ? "impl" : "decl");
      }
    }
    const double rounded = std::min(
        lp.vars[static_cast<std::size_t>(j)].ub,
        std::max(lp.vars[static_cast<std::size_t>(j)].lb, std::round(xj)));
    const double frac = std::abs(xj - rounded);
    if (!(frac > int_tol)) continue;
    ++stats.fractional_total;
    if (lp.vars[static_cast<std::size_t>(j)].type == VarType::Binary) {
      ++stats.fractional_binary;
    } else if (lp.vars[static_cast<std::size_t>(j)].type == VarType::Integer) {
      ++stats.fractional_integer;
    } else if (implied) {
      ++stats.fractional_implied;
    }
    stats.frontier_hash = bridge_hash_mix(
        stats.frontier_hash, static_cast<std::uint64_t>(j));
    stats.frontier_hash =
        bridge_hash_mix(stats.frontier_hash, bridge_hash_double(xj));
    stats.frontier_hash = bridge_hash_mix(
        stats.frontier_hash, static_cast<std::uint64_t>(status));
    if (frontier_emit < max_terms) {
      if (frontier_emit++ > 0) frontier_sample << ";";
      frontier_sample << j << ":" << xj << ":r" << rounded << ":f" << frac
                      << ":" << status << ":rc" << rc << ":"
                      << (lp.vars[static_cast<std::size_t>(j)].type ==
                                  VarType::Binary
                              ? "bin"
                              : (lp.vars[static_cast<std::size_t>(j)].type ==
                                         VarType::Integer
                                     ? "int"
                                     : "cont"))
                      << ":" << (implied ? "impl" : "decl");
    }
  }
  stats.frontier_sample = frontier_sample.str();
  stats.side_sample = side_sample.str();
#else
  stats.model_status = "unavailable";
  (void)lp;
  (void)implied_integer_cols;
  (void)int_tol;
  (void)max_terms;
#endif

  return stats;
}

HiGHSRootLpStateStats highs_standard_form_lp_state_stats(
    const StandardFormLP& sf,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    double int_tol,
    int max_terms) {
  HiGHSRootLpStateStats stats;
  const auto bridge = highs_presolve_bridge_info();
  stats.available = bridge.available;
  stats.bridge_source = bridge.source;

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  if (!bridge.available) return stats;
  const int nstd = static_cast<int>(sf.A.cols());
  const int m = static_cast<int>(sf.A.rows());
  const int norig = static_cast<int>(lp.vars.size());
  stats.rows = m;
  stats.cols = norig;
  stats.nnz = static_cast<int>(sf.A.nonZeros());
  if (nstd <= 0 || m <= 0 || sf.b.size() != m || sf.c_max.size() != nstd ||
      sf.var_ub.size() != nstd || sf.n_original < norig ||
      sf.lb_shift.size() < norig) {
    stats.model_status = "invalid_sf";
    return stats;
  }

  std::vector<double> col_cost(static_cast<std::size_t>(nstd), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(nstd), 0.0);
  std::vector<double> col_upper(static_cast<std::size_t>(nstd), kHighsInf);
  for (int j = 0; j < nstd; ++j) {
    col_cost[static_cast<std::size_t>(j)] = -sf.c_max[j];
    col_upper[static_cast<std::size_t>(j)] =
        highs_bound_or_inf(sf.var_ub[j], 1.0);
  }
  const int first_artificial = sf.n_original + sf.n_slack + sf.n_surplus;
  const int end_artificial = first_artificial + sf.n_artificial;
  for (int j = std::max(0, first_artificial);
       j < std::min(nstd, end_artificial); ++j) {
    // Artificial columns are Phase-I/recovery devices in the native standard
    // form.  They are not legal variables in the Phase-II LP consumed by cuts
    // and frontier selection.
    col_lower[static_cast<std::size_t>(j)] = 0.0;
    col_upper[static_cast<std::size_t>(j)] = 0.0;
  }
  std::vector<double> row_lower(static_cast<std::size_t>(m), 0.0);
  std::vector<double> row_upper(static_cast<std::size_t>(m), 0.0);
  for (int r = 0; r < m; ++r) {
    const double rhs = highs_bound_or_inf(sf.b[r], 1.0);
    row_lower[static_cast<std::size_t>(r)] = rhs;
    row_upper[static_cast<std::size_t>(r)] = rhs;
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(nstd + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(sf.A.nonZeros()));
  value.reserve(index.capacity());
  for (int j = 0; j < nstd; ++j) {
    start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(index.size());
    for (StandardColumnMatrix::InnerIterator it(sf.A, j); it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(it.row()));
      value.push_back(it.value());
    }
  }
  start[static_cast<std::size_t>(nstd)] = static_cast<HighsInt>(index.size());

  Highs highs;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("log_to_console", false);
  highs.setOptionValue("threads", 1);
  highs.setOptionValue("presolve", "off");
  highs.setOptionValue("solver", "simplex");
  highs.setOptionValue("simplex_strategy", 1);
  const auto pass_status = highs.passModel(
      static_cast<HighsInt>(nstd), static_cast<HighsInt>(m),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMinimize), 0.0, col_cost.data(),
      col_lower.data(), col_upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), nullptr);
  stats.pass_ok = (pass_status == HighsStatus::kOk);
  if (!stats.pass_ok) {
    stats.model_status = "pass_error";
    return stats;
  }

  const auto run_status = highs.run();
  stats.model_status = highs_model_status_name(highs.getModelStatus());
  stats.solve_ok =
      run_status == HighsStatus::kOk &&
      highs.getModelStatus() == HighsModelStatus::kOptimal;
  stats.objective = sf.objective_const +
                    highs.getInfo().objective_function_value;
  stats.iterations = static_cast<int>(highs.getInfo().simplex_iteration_count);
  if (!stats.solve_ok) return stats;

  const HighsSolution& sol = highs.getSolution();
  const HighsBasis& basis = highs.getBasis();
  const bool basis_valid =
      basis.valid && static_cast<int>(basis.col_status.size()) >= nstd;
  if (static_cast<int>(sol.col_value.size()) < nstd) return stats;

  std::vector<char> projected_status(static_cast<std::size_t>(norig), '?');
  std::vector<double> projected_value(
      static_cast<std::size_t>(norig),
      std::numeric_limits<double>::quiet_NaN());
  std::vector<double> projected_dual(
      static_cast<std::size_t>(norig),
      std::numeric_limits<double>::quiet_NaN());
  for (int j = 0; j < norig; ++j) {
    const HighsBasisStatus raw_status =
        (basis_valid ? basis.col_status[static_cast<std::size_t>(j)]
                     : HighsBasisStatus::kNonbasic);
    projected_status[static_cast<std::size_t>(j)] =
        highs_basis_status_char(raw_status);
    const double x_scaled = sol.col_value[static_cast<std::size_t>(j)];
    const double col_scale =
        (j < sf.col_scale.size() && std::isfinite(sf.col_scale[j]) &&
         std::abs(sf.col_scale[j]) > 1e-12)
            ? sf.col_scale[j]
            : 1.0;
    projected_value[static_cast<std::size_t>(j)] =
        sf.lb_shift[j] + col_scale * x_scaled;
    const double cost_scale = col_scale;
    const double std_dual = (static_cast<int>(sol.col_dual.size()) > j)
                                ? sol.col_dual[static_cast<std::size_t>(j)]
                                : std::numeric_limits<double>::quiet_NaN();
    projected_dual[static_cast<std::size_t>(j)] =
        std::isfinite(std_dual) ? std_dual / cost_scale : std_dual;
  }
  fill_projected_lp_state_stats(stats, lp, implied_integer_cols, int_tol,
                                max_terms, projected_status, projected_value,
                                projected_dual);
#else
  stats.model_status = "unavailable";
  (void)sf;
  (void)lp;
  (void)implied_integer_cols;
  (void)int_tol;
  (void)max_terms;
#endif

  return stats;
}

// ─────────────────────────────────────────────────────────────────────────────
// Adaptive HiGHS presolve wrapper for the native LP kernels
// ─────────────────────────────────────────────────────────────────────────────

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
namespace {

/// Build a HiGHS model (minimize sense) from a native LPModel. Returns false if
/// passModel failed. Mirrors the construction in highs_presolve_model_stats.
bool native_lp_to_highs(const LPModel& lp, Highs& highs) {
  const int ncols = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int nrows = m_ineq + m_eq;
  if (ncols == 0) return false;
  // Incremental model construction can leave an empty constraint family with
  // its previous column extent. Empty families contain no coefficients, so
  // skip their sparse outer dimension; nonempty families must match the
  // variable dimension before constructing Eigen inner iterators.
  if (lp.c.size() != ncols || lp.b.size() != m_ineq ||
      lp.beq.size() != m_eq || (m_ineq > 0 && lp.A.cols() != ncols) ||
      (m_eq > 0 && lp.Aeq.cols() != ncols)) {
    return false;
  }

  std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(ncols), -kHighsInf);
  std::vector<double> col_upper(static_cast<std::size_t>(ncols), kHighsInf);
  std::vector<HighsInt> integrality(
      static_cast<std::size_t>(ncols),
      static_cast<HighsInt>(HighsVarType::kContinuous));
  for (int j = 0; j < ncols; ++j) {
    const auto& v = lp.vars[static_cast<std::size_t>(j)];
    col_cost[static_cast<std::size_t>(j)] =
        (lp.sense == Sense::Maximize ? -lp.c[j] : lp.c[j]);
    col_lower[static_cast<std::size_t>(j)] = highs_bound_or_inf(v.lb, -1.0);
    col_upper[static_cast<std::size_t>(j)] = highs_bound_or_inf(v.ub, 1.0);
    if (v.type != VarType::Continuous) {
      integrality[static_cast<std::size_t>(j)] =
          static_cast<HighsInt>(HighsVarType::kInteger);
    }
  }

  std::vector<double> row_lower(static_cast<std::size_t>(nrows), -kHighsInf);
  std::vector<double> row_upper(static_cast<std::size_t>(nrows), kHighsInf);
  for (int r = 0; r < m_ineq; ++r) {
    row_lower[static_cast<std::size_t>(r)] =
        highs_bound_or_inf(lp_row_lhs_or_neg_inf(lp, r), -1.0);
    row_upper[static_cast<std::size_t>(r)] = highs_bound_or_inf(lp.b[r], 1.0);
  }
  for (int r = 0; r < m_eq; ++r) {
    const double rhs = highs_bound_or_inf(lp.beq[r], 1.0);
    row_lower[static_cast<std::size_t>(m_ineq + r)] = rhs;
    row_upper[static_cast<std::size_t>(m_ineq + r)] = rhs;
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(lp.A.nonZeros() + lp.Aeq.nonZeros()));
  value.reserve(index.capacity());
  for (int j = 0; j < ncols; ++j) {
    start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(index.size());
    if (m_ineq > 0) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
        if (it.value() == 0.0) continue;
        index.push_back(static_cast<HighsInt>(it.row()));
        value.push_back(it.value());
      }
    }
    if (m_eq > 0) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
        if (it.value() == 0.0) continue;
        index.push_back(static_cast<HighsInt>(m_ineq + it.row()));
        value.push_back(it.value());
      }
    }
  }
  start[static_cast<std::size_t>(ncols)] = static_cast<HighsInt>(index.size());

  const auto pass_status = highs.passModel(
      static_cast<HighsInt>(ncols), static_cast<HighsInt>(nrows),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMinimize), 0.0, col_cost.data(),
      col_lower.data(), col_upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), integrality.data());
  return pass_status == HighsStatus::kOk;
}

}  // namespace
#endif  // MIPSOLVERS_HAVE_HIGHS_LIB

HighsLpPresolveConfig highs_lp_presolve_config_from_env(
    HighsLpPresolveConfig base) {
  if (const char* e = std::getenv("MIPSOLVERS_PRESOLVE")) {
    base.enabled = !(e[0] == '0' && e[1] == '\0');
  }
  if (const char* e = std::getenv("MIPSOLVERS_PRESOLVE_NNZ_FLOOR")) {
    char* end = nullptr;
    const long v = std::strtol(e, &end, 10);
    if (end != e) base.nnz_floor = v;
  }
  if (const char* e = std::getenv("MIPSOLVERS_PRESOLVE_NNZ_CAP")) {
    char* end = nullptr;
    const long v = std::strtol(e, &end, 10);
    if (end != e) base.nnz_cap = v;
  }
  if (const char* e = std::getenv("MIPSOLVERS_PRESOLVE_MIN_SHRINK")) {
    char* end = nullptr;
    const double v = std::strtod(e, &end);
    if (end != e) base.min_shrink = v;
  }
  if (std::getenv("MIPSOLVERS_PRESOLVE_VERBOSE")) base.verbose = true;
  return base;
}

HighsLpPresolveResult highs_presolve_lp(const LPModel& lp,
                                        const HighsLpPresolveConfig& cfg) {
  HighsLpPresolveResult out;
  out.orig_rows = static_cast<int>(lp.A.rows() + lp.Aeq.rows());
  out.orig_cols = static_cast<int>(lp.vars.size());
  out.orig_nnz = lp.A.nonZeros() + lp.Aeq.nonZeros();
  out.status = "disabled";
  if (!cfg.enabled) return out;
  if (out.orig_cols == 0) return out;
  if (cfg.nnz_floor > 0 && out.orig_nnz < cfg.nnz_floor) {
    out.status = "skipped_nnz_floor";
    if (cfg.verbose) {
      std::fprintf(
          stderr,
          "[HIGHS-PRESOLVE] skip: nnz %ld < floor %ld (rows=%d cols=%d)\n",
          out.orig_nnz, cfg.nnz_floor, out.orig_rows, out.orig_cols);
    }
    return out;
  }
  if (cfg.nnz_cap > 0 && out.orig_nnz > cfg.nnz_cap) {
    out.status = "skipped_nnz_cap";
    if (cfg.verbose) {
      std::fprintf(
          stderr,
          "[HIGHS-PRESOLVE] skip: nnz %ld > cap %ld (rows=%d cols=%d)\n",
          out.orig_nnz, cfg.nnz_cap, out.orig_rows, out.orig_cols);
    }
    return out;
  }

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  const auto bridge = highs_presolve_bridge_info();
  if (!bridge.available) {
    out.status = "bridge_unavailable";
    return out;
  }

  const auto t0 = std::chrono::steady_clock::now();
  auto highs = std::make_shared<Highs>();
  highs->setOptionValue("output_flag", false);
  highs->setOptionValue("log_to_console", false);
  highs->setOptionValue("threads", 1);
  if (cfg.time_limit_sec > 0.0 && std::isfinite(cfg.time_limit_sec)) {
    highs->setOptionValue("time_limit", std::max(0.001, cfg.time_limit_sec));
  }
  if (!native_lp_to_highs(lp, *highs)) {
    out.status = "pass_error";
    return out;
  }

  const auto presolve_status = highs->presolve();
  out.presolve_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
  out.status = highs_presolve_status_name(highs->getModelPresolveStatus());
  if (!(presolve_status == HighsStatus::kOk ||
        presolve_status == HighsStatus::kWarning)) {
    return out;
  }

  const HighsPresolveStatus mps = highs->getModelPresolveStatus();
  out.impl = highs;  // retain the HiGHS instance for a later postsolve
  out.attempted = true;

  switch (mps) {
    case HighsPresolveStatus::kInfeasible:
      out.infeasible = true;
      break;
    case HighsPresolveStatus::kReducedToEmpty:
      out.solved_by_presolve = true;
      break;
    case HighsPresolveStatus::kReduced: {
      const HighsLp& presolved = highs->getPresolvedLp();
      out.objective_offset = presolved.offset_;
      const bool sizes_ok =
          presolved.num_col_ >= 0 && presolved.num_row_ >= 0 &&
          static_cast<int>(presolved.col_cost_.size()) >= presolved.num_col_ &&
          static_cast<int>(presolved.col_lower_.size()) >= presolved.num_col_ &&
          static_cast<int>(presolved.col_upper_.size()) >= presolved.num_col_ &&
          static_cast<int>(presolved.row_lower_.size()) >= presolved.num_row_ &&
          static_cast<int>(presolved.row_upper_.size()) >= presolved.num_row_;
      if (!sizes_ok) {
        out.status = "presolved_lp_invalid";
        break;
      }
      LPModel reduced =
          convert_highs_presolved_lp_to_native(presolved, lp.sense);
      const auto& side_state = highs->getPresolveSideState();
      const bool mapping_ok =
          side_state.available &&
          static_cast<int>(side_state.presolved_col_orig.size()) ==
              presolved.num_col_ &&
          static_cast<int>(side_state.presolved_col_scale.size()) ==
              presolved.num_col_ &&
          static_cast<int>(side_state.presolved_col_constant.size()) ==
              presolved.num_col_ &&
          static_cast<int>(
              side_state.presolved_col_linearly_transformable.size()) ==
              presolved.num_col_;
      if (!mapping_ok) {
        // Cold native LP solves need only the reduced LP and the retained
        // HighsPostsolveStack: Highs::postsolve accepts a primal-only reduced
        // solution for kReduced without a basis (Highs.cpp:3533-3565;
        // HighsPostsolveStack.h:649-796).  The side-state affine map is a
        // stricter forward-map/warm-start contract and is not an adoption
        // prerequisite for this path.  Keep its vectors empty so any caller
        // requesting forward projection still fails loudly.
        out.status = "reduced_primal_postsolve_only";
        out.reduced_rows =
            static_cast<int>(reduced.A.rows() + reduced.Aeq.rows());
        out.reduced_cols = static_cast<int>(reduced.vars.size());
        out.reduced_nnz = reduced.A.nonZeros() + reduced.Aeq.nonZeros();
        const bool worth_it =
            cfg.min_shrink <= 0.0 ||
            out.reduced_nnz < static_cast<long>(
                                  cfg.min_shrink *
                                  static_cast<double>(out.orig_nnz));
        if (worth_it) {
          out.reduced = std::move(reduced);
          out.use_reduced = true;
        }
        break;
      }
      out.reduced_to_orig_col.reserve(
          side_state.presolved_col_orig.size());
      for (const HighsInt col : side_state.presolved_col_orig) {
        out.reduced_to_orig_col.push_back(static_cast<int>(col));
      }
      out.orig_to_reduced_col.assign(
          static_cast<std::size_t>(out.orig_cols), -1);
      for (int j = 0; j < static_cast<int>(out.reduced_to_orig_col.size()); ++j) {
        const int orig = out.reduced_to_orig_col[static_cast<std::size_t>(j)];
        if (orig >= 0 && orig < out.orig_cols) {
          out.orig_to_reduced_col[static_cast<std::size_t>(orig)] = j;
        }
      }
      out.reduced_col_scale = side_state.presolved_col_scale;
      out.reduced_col_constant = side_state.presolved_col_constant;
      out.reduced_col_linearly_transformable =
          side_state.presolved_col_linearly_transformable;
      out.side_state.available = true;
      out.side_state.pass_ok = true;
      out.side_state.presolve_ok = true;
      out.side_state.bridge_source = bridge.source;
      out.side_state.highs_status = out.status;
      out.side_state.side_state_available = true;
      out.side_state.rows = static_cast<int>(side_state.rows);
      out.side_state.cols = static_cast<int>(side_state.cols);
      out.side_state.nnz = static_cast<int>(side_state.nnz);
      out.side_state.ranged_rows = static_cast<int>(side_state.ranged_rows);
      out.side_state.binary_cols = static_cast<int>(side_state.binary_cols);
      out.side_state.integer_cols = static_cast<int>(side_state.integer_cols);
      out.side_state.implied_integer_cols =
          static_cast<int>(side_state.implied_integer_cols);
      out.side_state.continuous_cols =
          static_cast<int>(side_state.continuous_cols);
      out.side_state.fixed_cols = static_cast<int>(side_state.fixed_cols);
      out.side_state.vub_count = static_cast<int>(side_state.vub_count);
      out.side_state.vlb_count = static_cast<int>(side_state.vlb_count);
      out.side_state.vub_attempts = static_cast<int>(side_state.vub_attempts);
      out.side_state.vub_accepted = static_cast<int>(side_state.vub_accepted);
      out.side_state.vub_replaced = static_cast<int>(side_state.vub_replaced);
      out.side_state.vlb_attempts = static_cast<int>(side_state.vlb_attempts);
      out.side_state.vlb_accepted = static_cast<int>(side_state.vlb_accepted);
      out.side_state.vlb_replaced = static_cast<int>(side_state.vlb_replaced);
      out.side_state.probing_calls = static_cast<int>(side_state.probing_calls);
      out.side_state.probing_conflicts =
          static_cast<int>(side_state.probing_conflicts);
      out.side_state.probing_reductions =
          static_cast<int>(side_state.probing_reductions);
      out.side_state.probing_substitutions =
          static_cast<int>(side_state.probing_substitutions);
      out.side_state.vub_hash = side_state.vub_hash;
      out.side_state.vlb_hash = side_state.vlb_hash;
      out.side_state.presolved_lp_available = true;
      out.side_state.presolved_objective_offset = presolved.offset_;
      out.side_state.presolved_row_lower = side_state.presolved_row_lower;
      out.side_state.presolved_row_upper = side_state.presolved_row_upper;
      out.side_state.presolved_col_lower = side_state.presolved_col_lower;
      out.side_state.presolved_col_upper = side_state.presolved_col_upper;
      out.side_state.presolved_col_orig = out.reduced_to_orig_col;
      out.side_state.presolved_col_scale = out.reduced_col_scale;
      out.side_state.presolved_col_constant = out.reduced_col_constant;
      out.side_state.presolved_col_linearly_transformable =
          out.reduced_col_linearly_transformable;
      out.side_state.presolved_col_type.reserve(
          side_state.presolved_col_type.size());
      for (const HighsVarType type : side_state.presolved_col_type) {
        out.side_state.presolved_col_type.push_back(
            static_cast<unsigned char>(type));
      }
      out.side_state.presolved_a_start.reserve(
          side_state.presolved_a_start.size());
      for (const HighsInt value : side_state.presolved_a_start) {
        out.side_state.presolved_a_start.push_back(static_cast<int>(value));
      }
      out.side_state.presolved_a_index.reserve(
          side_state.presolved_a_index.size());
      for (const HighsInt value : side_state.presolved_a_index) {
        out.side_state.presolved_a_index.push_back(static_cast<int>(value));
      }
      out.side_state.presolved_a_value = side_state.presolved_a_value;
      out.side_state.var_bounds.reserve(side_state.var_bounds.size());
      for (const auto& record : side_state.var_bounds) {
        HiGHSPresolvedModelStats::VarBoundRecord copied;
        copied.target_col = static_cast<int>(record.target_col);
        copied.trigger_col = static_cast<int>(record.trigger_col);
        copied.target_orig_col = static_cast<int>(record.target_orig_col);
        copied.trigger_orig_col = static_cast<int>(record.trigger_orig_col);
        copied.coef = record.coef;
        copied.constant = record.constant;
        copied.target_scale = record.target_scale;
        copied.target_constant = record.target_constant;
        copied.trigger_scale = record.trigger_scale;
        copied.trigger_constant = record.trigger_constant;
        copied.upper = record.upper;
        copied.target_linearly_transformable =
            record.target_linearly_transformable;
        copied.trigger_linearly_transformable =
            record.trigger_linearly_transformable;
        out.side_state.var_bounds.push_back(copied);
      }
      out.reduced_rows =
          static_cast<int>(reduced.A.rows() + reduced.Aeq.rows());
      out.reduced_cols = static_cast<int>(reduced.vars.size());
      out.reduced_nnz = reduced.A.nonZeros() + reduced.Aeq.nonZeros();
      const bool worth_it =
          cfg.min_shrink <= 0.0 ||
          out.reduced_nnz <
              static_cast<long>(cfg.min_shrink * static_cast<double>(out.orig_nnz));
      if (worth_it) {
        out.reduced = std::move(reduced);
        out.use_reduced = true;
      }
      break;
    }
    case HighsPresolveStatus::kNotReduced:
    default:
      break;
  }

  if (cfg.verbose) {
    std::fprintf(stderr,
                 "[HIGHS-PRESOLVE] %s: rows %d->%d cols %d->%d nnz %ld->%ld "
                 "use_reduced=%d %.1fms\n",
                 out.status.c_str(), out.orig_rows, out.reduced_rows,
                 out.orig_cols, out.reduced_cols, out.orig_nnz, out.reduced_nnz,
                 static_cast<int>(out.use_reduced), out.presolve_ms);
  }
  return out;
#else
  out.status = "no_highs";
  return out;
#endif
}

Eigen::VectorXd highs_postsolve_primal(const HighsLpPresolveResult& ps,
                                       const Eigen::VectorXd& x_reduced) {
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  if (!ps.impl) return {};
  auto* highs = static_cast<Highs*>(ps.impl.get());
  HighsSolution sol;
  sol.col_value.assign(x_reduced.data(), x_reduced.data() + x_reduced.size());
  sol.value_valid = true;
  const auto st = highs->postsolve(sol);
  if (!(st == HighsStatus::kOk || st == HighsStatus::kWarning)) return {};
  const HighsSolution& full = highs->getSolution();
  if (!full.value_valid ||
      static_cast<int>(full.col_value.size()) != ps.orig_cols) {
    return {};
  }
  Eigen::VectorXd x(ps.orig_cols);
  for (int j = 0; j < ps.orig_cols; ++j) {
    x[j] = full.col_value[static_cast<std::size_t>(j)];
  }
  return x;
#else
  (void)ps;
  (void)x_reduced;
  return {};
#endif
}

Eigen::VectorXd highs_presolve_forward_map(const HighsLpPresolveResult& ps,
                                           const Eigen::VectorXd& x_original) {
  const int n_reduced = static_cast<int>(ps.reduced_to_orig_col.size());
  if (x_original.size() != ps.orig_cols || n_reduced <= 0 ||
      static_cast<int>(ps.reduced_col_scale.size()) != n_reduced ||
      static_cast<int>(ps.reduced_col_constant.size()) != n_reduced ||
      static_cast<int>(ps.reduced_col_linearly_transformable.size()) !=
          n_reduced) {
    return {};
  }
  Eigen::VectorXd x_reduced(n_reduced);
  for (int j = 0; j < n_reduced; ++j) {
    const int orig = ps.reduced_to_orig_col[static_cast<std::size_t>(j)];
    const double scale = ps.reduced_col_scale[static_cast<std::size_t>(j)];
    const double constant =
        ps.reduced_col_constant[static_cast<std::size_t>(j)];
    // HiGHS HighsPostsolveStack::linearTransform; derivation in
    // docs/native_milp_root_source_eligibility_2026-08-13.md.
    // x_original = scale * x_reduced + constant.
    if (orig < 0 || orig >= x_original.size() ||
        ps.reduced_col_linearly_transformable[static_cast<std::size_t>(j)] ==
            0 ||
        !std::isfinite(scale) || !std::isfinite(constant) ||
        std::abs(scale) <= 1e-12) {
      return {};
    }
    x_reduced[j] = (x_original[orig] - constant) / scale;
  }
  return x_reduced;
}

Eigen::VectorXd highs_presolve_forward_map(
    const HiGHSPresolvedModelStats& side_state,
    int original_cols,
    const Eigen::VectorXd& x_original) {
  const int n_reduced = static_cast<int>(side_state.presolved_col_orig.size());
  if (!side_state.side_state_available || original_cols < 0 ||
      x_original.size() != original_cols || n_reduced <= 0 ||
      static_cast<int>(side_state.presolved_col_scale.size()) != n_reduced ||
      static_cast<int>(side_state.presolved_col_constant.size()) != n_reduced ||
      static_cast<int>(side_state.presolved_col_linearly_transformable.size()) !=
          n_reduced) {
    return {};
  }

  Eigen::VectorXd x_reduced(n_reduced);
  for (int j = 0; j < n_reduced; ++j) {
    const int orig = side_state.presolved_col_orig[static_cast<std::size_t>(j)];
    const double scale =
        side_state.presolved_col_scale[static_cast<std::size_t>(j)];
    const double constant =
        side_state.presolved_col_constant[static_cast<std::size_t>(j)];
    // HiGHS HighsPostsolveStack::linearTransform; affine inverse derived in
    // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md,
    // "Coupled HiGHS Root-Primal Ownership".
    if (orig < 0 || orig >= original_cols ||
        side_state.presolved_col_linearly_transformable[
            static_cast<std::size_t>(j)] == 0 ||
        !std::isfinite(scale) || !std::isfinite(constant) ||
        std::abs(scale) <= 1e-12) {
      return {};
    }
    x_reduced[j] = (x_original[orig] - constant) / scale;
  }
  return x_reduced;
}

bool highs_presolve_recover_primal(const LPModel& lp,
                                   const HighsLpPresolveResult& ps,
                                   const Eigen::VectorXd& x_reduced,
                                   double audit_tol,
                                   Eigen::VectorXd& x_orig_out,
                                   double& objective_out) {
  Eigen::VectorXd x = highs_postsolve_primal(ps, x_reduced);
  if (x.size() != static_cast<int>(lp.vars.size())) {
    if (std::getenv("MIPSOLVERS_PRESOLVE_VERBOSE")) {
      std::fprintf(stderr,
                   "[HIGHS-PRESOLVE] primal_recovery postsolve_size=%lld "
                   "expected=%lld\n",
                   static_cast<long long>(x.size()),
                   static_cast<long long>(lp.vars.size()));
    }
    return false;
  }

  if (!lp_solution_residual_acceptable(lp, x, audit_tol)) {
    if (std::getenv("MIPSOLVERS_PRESOLVE_VERBOSE")) {
      // Diagnostic decomposition of the authoritative audit in
      // dual_simplex.cpp:1852-1907.  It reports, but does not alter, the same
      // sentinel-aware max violation and side scale used for publication.
      constexpr double kSideSentinel = 1e19;
      double row_violation = 0.0;
      double equality_violation = 0.0;
      double bound_violation = 0.0;
      double scale = 1.0;
      if (lp.A.rows() > 0) {
        const Eigen::VectorXd ax = lp.A * x;
        for (int i = 0; i < ax.size(); ++i) {
          if (std::abs(lp.b[i]) < kSideSentinel) {
            row_violation = std::max(row_violation, ax[i] - lp.b[i]);
            scale = std::max(scale, std::abs(lp.b[i]));
          }
          const double lhs = lp_row_lhs_or_neg_inf(lp, i);
          if (std::isfinite(lhs) && std::abs(lhs) < kSideSentinel) {
            row_violation = std::max(row_violation, lhs - ax[i]);
            scale = std::max(scale, std::abs(lhs));
          }
        }
      }
      if (lp.Aeq.rows() > 0) {
        const Eigen::VectorXd eq_residual = lp.Aeq * x - lp.beq;
        equality_violation = eq_residual.lpNorm<Eigen::Infinity>();
        scale = std::max(scale, lp.beq.lpNorm<Eigen::Infinity>());
      }
      for (int j = 0; j < x.size(); ++j) {
        const auto& var = lp.vars[static_cast<std::size_t>(j)];
        if (std::isfinite(var.lb))
          bound_violation = std::max(bound_violation, var.lb - x[j]);
        if (std::isfinite(var.ub))
          bound_violation = std::max(bound_violation, x[j] - var.ub);
      }
      const double violation =
          std::max({0.0, row_violation, equality_violation, bound_violation});
      std::fprintf(stderr,
                   "[HIGHS-PRESOLVE] primal_recovery audit_rejected "
                   "row=%.3e equality=%.3e bound=%.3e scale=%.3e "
                   "relative=%.3e limit=%.3e\n",
                   row_violation, equality_violation, bound_violation, scale,
                   violation / scale, audit_tol);
    }
    return false;
  }

  objective_out = lp.c.dot(x);
  x_orig_out = std::move(x);
  return true;
}

Eigen::VectorXd highs_side_state_postsolve_primal(
    const HiGHSPresolvedModelStats& side_state,
    const Eigen::VectorXd& x_working) {
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  if (!side_state.impl || !side_state.impl_mutex ||
      side_state.original_cols <= 0 ||
      static_cast<int>(x_working.size()) !=
          static_cast<int>(side_state.presolved_lp.vars.size())) {
    return {};
  }
  // Highs::postsolve mutates the retained instance and side-state cache
  // entries are shared across solves; serialize the recovery.
  std::lock_guard<std::mutex> lock(*side_state.impl_mutex);
  auto* highs = static_cast<Highs*>(side_state.impl.get());
  HighsSolution sol;
  sol.col_value.assign(x_working.data(), x_working.data() + x_working.size());
  sol.value_valid = true;
  // Vendored Highs::callRunPostsolve MIP branch: primal-only column solution
  // of presolved size, row values ignored, eliminated original columns
  // reconstructed through the HighsPostsolveStack.
  const auto st = highs->postsolve(sol);
  if (!(st == HighsStatus::kOk || st == HighsStatus::kWarning)) return {};
  const HighsSolution& full = highs->getSolution();
  if (!full.value_valid ||
      static_cast<int>(full.col_value.size()) != side_state.original_cols) {
    return {};
  }
  Eigen::VectorXd x(side_state.original_cols);
  for (int j = 0; j < side_state.original_cols; ++j) {
    x[j] = full.col_value[static_cast<std::size_t>(j)];
  }
  return x;
#else
  (void)side_state;
  (void)x_working;
  return {};
#endif
}

}  // namespace mipsolvers::engine
