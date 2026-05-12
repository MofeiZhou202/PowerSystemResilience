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
#include <cmath>
#include <cstdint>
#include <limits>
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
  std::vector<int> row_to_native(static_cast<std::size_t>(std::max(0, nrows)),
                                 -1);
  std::vector<int> highs_row_to_native_row(
      static_cast<std::size_t>(std::max(0, nrows)), -1);
  std::vector<double> row_lhs;
  std::vector<double> row_rhs;
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
  out.Aeq.resize(0, ncols);
  out.beq.resize(0);
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

HiGHSPresolvedModelStats highs_presolve_model_stats(const LPModel& lp) {
  HiGHSPresolvedModelStats stats;
  const auto bridge = highs_presolve_bridge_info();
  stats.available = bridge.available;
  stats.bridge_source = bridge.source;

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  if (!bridge.available) return stats;

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

  std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(ncols), -kHighsInf);
  std::vector<double> col_upper(static_cast<std::size_t>(ncols), kHighsInf);
  std::vector<HighsInt> integrality(static_cast<std::size_t>(ncols),
                                    static_cast<HighsInt>(HighsVarType::kContinuous));
  for (int j = 0; j < ncols; ++j) {
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

  const auto presolve_status = highs.presolve();
  stats.presolve_ok = (presolve_status == HighsStatus::kOk ||
                       presolve_status == HighsStatus::kWarning);
  stats.highs_status =
      highs_presolve_status_name(highs.getModelPresolveStatus());
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
    for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
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

}  // namespace mipsolvers::engine
