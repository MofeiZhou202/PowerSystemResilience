#pragma once
/// \file warmstart.hpp
/// \brief Warm-start payload for the native branch-and-cut engine.
///
/// A `BCWarmStart` allows the caller to seed the solver with:
///   * a primal incumbent (for example, from a previous run or a heuristic),
///   * a dual solution + basis (for LP relaxation warm-start),
///   * a cut pool snapshot (rows to re-seed the global cut pool),
///   * an inherited node order / best bound (for restart after time-out).
///
/// All fields are optional: an empty `BCWarmStart{}` disables warm-starting.
/// The solver will silently ignore payload components it cannot validate
/// against the current model (size mismatch, infeasibility, etc.).

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace mipsolvers::engine {

/// Status codes for variables and constraints in a simplex basis.
enum class BasisStatus : std::uint8_t {
  Basic,
  NonBasicLower,
  NonBasicUpper,
  NonBasicFree,
  Fixed,
};

/// Opaque simplex basis descriptor suitable for seeding the dual simplex.
struct BCSimplexBasis {
  std::vector<BasisStatus> col_status;  ///< one entry per variable
  std::vector<BasisStatus> row_status;  ///< one entry per constraint
  int generation{0};                    ///< monotonic version tag
};

/// A single primal solution together with optional metadata.
struct BCPrimalHint {
  Eigen::VectorXd x;           ///< primal vector in original-variable space
  double          obj{0.0};    ///< cached objective value (if known)
  bool            verified{false}; ///< true if caller verified feasibility
};

/// A cut rehydration record: A_i · x  R  b_i   where R is <=, =, >=.
struct BCCutRecord {
  Eigen::SparseVector<double> coeffs;
  double rhs{0.0};
  char   sense{'L'};   ///< 'L'=<=, 'G'=>=, 'E'=='='
  double efficacy{0.0};
  int    age{0};
};

/// Complete warm-start payload.
struct BCWarmStart {
  std::vector<BCPrimalHint> primal_hints;          ///< zero or more incumbents
  Eigen::VectorXd           dual_row;              ///< optional row duals
  Eigen::VectorXd           dual_col;              ///< optional reduced-cost / bound duals
  BCSimplexBasis            basis;                 ///< optional simplex basis
  std::vector<BCCutRecord>  cut_pool;              ///< optional cut-pool snapshot

  double                    inherited_best_bound{-1e30}; ///< from a previous run
  double                    inherited_best_obj{1e30};

  /// True when no payload is attached (fast short-circuit in the solver).
  bool empty() const {
    return primal_hints.empty() && dual_row.size() == 0 && dual_col.size() == 0
        && basis.col_status.empty() && basis.row_status.empty()
        && cut_pool.empty();
  }
};

}  // namespace mipsolvers::engine
