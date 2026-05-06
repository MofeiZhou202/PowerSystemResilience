/// @file detail/bc/cuts/cut_types.hpp
/// @brief Cut family and tracking types (Phase 4.5 reorganization).
///
/// Extracted from bc_types.hpp.
/// Defines cut families, cut structures, and efficacy tracking.

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Sparse>

namespace hacdcpf::engine::detail {

/// @brief Supported cut family enumeration.
enum class CutFamily : int {
  Gomory = 0,          ///< Gomory mixed-integer cuts
  MIR = 1,             ///< Mixed integer rounding cuts
  Cover = 2,           ///< Cover cuts
  FlowCover = 3,       ///< Flow cover cuts
  ImpliedBound = 4,    ///< Implied bound cuts
  Clique = 5,          ///< Clique cuts
  StrongCG = 6,        ///< Strong Chvátal-Gomory cuts
  Knapsack = 7,        ///< Knapsack covers
  BinaryImplied = 8,   ///< Binary implied bound cuts
  GCD = 9,             ///< GCD-based cuts
  DisjunctiveCuts = 10, ///< Disjunctive cutting planes

  NumFamilies = 11     ///< Total number of cut families
};

/// @brief Cut structure with constraint representation.
struct PoolCut {
  Eigen::SparseVector<double> coeff;
  double rhs{0.0};
};

/// @brief Per-family cut efficacy statistics.
struct CutFamilyStats {
  CutFamily family{CutFamily::Gomory};
  int total_separated{0};       ///< Total cuts separated from this family
  int total_added{0};           ///< Total added to the LP
  int total_pruned{0};          ///< Total removed as ineffective
  int total_applied{0};         ///< Total applied in re-solves
  double total_efficacy{0.0};   ///< Sum of efficacy values

  /// @brief Average efficacy (violations reduced per re-solve).
  double avg_efficacy() const {
    return (total_applied > 0) ? (total_efficacy / total_applied) : 0.0;
  }

  /// @brief Acceptance rate (added / separated).
  double acceptance_rate() const {
    return (total_separated > 0) ? 
           (static_cast<double>(total_added) / total_separated) : 0.0;
  }
};

/// @brief Aggregated cut statistics across all families.
struct CutFamilyTracker {
  std::vector<CutFamilyStats> stats;

  CutFamilyTracker() {
    stats.resize(static_cast<int>(CutFamily::NumFamilies));
    for (int i = 0; i < static_cast<int>(CutFamily::NumFamilies); ++i) {
      stats[i].family = static_cast<CutFamily>(i);
    }
  }

  /// @brief Get stats for a specific cut family.
  CutFamilyStats& operator[](CutFamily family) {
    return stats[static_cast<int>(family)];
  }

  /// @brief Get const stats for a specific cut family.
  const CutFamilyStats& operator[](CutFamily family) const {
    return stats[static_cast<int>(family)];
  }
};

}  // namespace hacdcpf::engine::detail
