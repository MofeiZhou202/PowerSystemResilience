/// @file bc_clique_table.hpp
/// @brief Persistent clique table for binary variables.
///
/// A clique table encodes pairwise conflicts among binary variables of the
/// form `x_i + x_j <= 1` (i.e. at most one of {x_i, x_j} may be 1 in any
/// feasible integer solution). The table is built once at the root from
/// pure-binary "<=" rows of the LP (including degenerate equality rows
/// encoded as two inequalities) and reused across:
///   1. Clique cut separation (avoid rebuilding the O(m) conflict graph
///      each cut round).
///   2. Domain propagation at probing and tree nodes: fixing `x_i = 1`
///      immediately forces `x_j = 0` for every `j` adjacent to `i` in the
///      conflict graph.
///   3. Presolve-time fixings when one variable's bound collapses.
///
/// The representation uses persistent sorted adjacency vectors. Incremental
/// insertions touch only the endpoints of new edges, while binary-search-based
/// intersection remains cheap during greedy clique extension.

#pragma once

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine::detail {

class BinaryImplicationGraph;

class CliqueTable {
 public:
  CliqueTable() = default;

  struct Literal {
    int col{-1};
    bool value_one{false};
  };

  struct CutCliqueExtractionStats {
    std::uint64_t rows_scanned{0};
    std::uint64_t rows_with_binary_literals{0};
    std::uint64_t rows_with_pair_excess{0};
    std::uint64_t rows_with_unit_excess{0};
    std::uint64_t literal_edges_generated{0};
    double max_pair_excess{0.0};
    double max_unit_excess{0.0};
    double best_pair_margin{-1e100};
    double best_unit_margin{-1e100};
  };

  /// @brief Build (or rebuild) the clique table from an LP model.
  /// @details Scans every finite inequality and equality row side after
  /// shifting finite variable bounds, converts binary-like variables to
  /// positive literal coefficients, and adds a conflict edge for each pair
  /// whose combined coefficient exceeds the residual row capacity. Rows with
  /// more than `max_row_nnz` entries are skipped to keep construction cost
  /// bounded. The optional override marks implied-integer 0/1 columns as
  /// binary-like, matching the HiGHS clique source convention.
  /// @return The number of conflict edges (undirected pair count).
  std::size_t build(const LPModel& lp,
                    int max_row_nnz = 256,
                    const std::vector<char>* binary_like_override = nullptr);

  /// @brief Merge explicit binary one-one conflict edges into the table.
  /// @details Used by implied-integer/ranged-row root analysis for conflicts
  /// that are not visible as pure-binary <= rows.  Invalid/non-binary/self
  /// pairs are ignored.  Returns the number of newly added undirected edges.
  std::size_t add_edges(const LPModel& lp,
                        const std::vector<std::pair<int, int>>& edges);

  /// @brief Merge explicit literal-level conflict edges into the table.
  /// @details A literal edge `(a=va,b=vb)` means those assignments cannot both
  /// hold.  This mirrors HiGHS' `CliqueVar(col,val)` table.  When both values
  /// are one, the legacy variable graph is also updated.
  std::size_t add_literal_edges(
      const LPModel& lp,
      const std::vector<std::pair<Literal, Literal>>& edges);

  /// @brief Extract literal conflicts from one valid cut row.
  /// @details Mirrors the HiGHS cut-pool path
  /// `HighsCliqueTable::extractCliquesFromCut`: compute minimum activity from
  /// finite bounds, convert binary terms to positive literal coefficients, and
  /// declare two assignments conflicting when their combined contribution
  /// exceeds the residual row capacity.  The optional binary-like override is
  /// used for implied-integer 0/1 columns that remain continuous in LPModel.
  static std::vector<std::pair<Literal, Literal>> extract_literal_edges_from_cut(
      const LPModel& lp,
      const Eigen::SparseVector<double>& row_le,
      double rhs_le,
      int max_row_nnz = 256,
      const std::vector<char>* binary_like_override = nullptr,
      const BinaryImplicationGraph* implication_graph = nullptr,
      CutCliqueExtractionStats* stats = nullptr,
      const Eigen::VectorXd* lp_solution = nullptr);

  /// @brief Extract and merge literal conflicts from one valid cut row.
  std::size_t add_literal_edges_from_cut(
      const LPModel& lp,
      const Eigen::SparseVector<double>& row_le,
      double rhs_le,
      int max_row_nnz = 256,
      const std::vector<char>* binary_like_override = nullptr,
      const BinaryImplicationGraph* implication_graph = nullptr,
      CutCliqueExtractionStats* stats = nullptr,
      const Eigen::VectorXd* lp_solution = nullptr);

  /// @brief True when no variable or literal conflicts have been recorded.
  bool empty() const { return n_edges_ == 0 && n_literal_edges_ == 0; }

  /// @brief Number of columns covered by the table.
  int n_cols() const { return n_; }

  /// @brief Number of undirected conflict edges.
  std::size_t n_edges() const { return n_edges_; }

  /// @brief Number of undirected literal-level conflict edges.
  std::size_t n_literal_edges() const { return n_literal_edges_; }

  /// @brief Neighbours of variable `j` (sorted ascending).
  /// @details The returned range remains valid until that variable's
  /// neighbourhood is modified or the table is rebuilt.
  std::pair<const int*, const int*> neighbours(int j) const {
    if (j < 0 || j >= n_) return {nullptr, nullptr};
    const auto& row = adjacency_[static_cast<std::size_t>(j)];
    if (row.empty()) return {nullptr, nullptr};
    return {row.data(), row.data() + row.size()};
  }

  /// @brief Size of the neighbourhood of `j`.
  int degree(int j) const {
    if (j < 0 || j >= n_) return 0;
    return static_cast<int>(adjacency_[static_cast<std::size_t>(j)].size());
  }

  /// @brief True iff (i, j) is a recorded conflict edge.
  bool has_edge(int i, int j) const {
    if (i < 0 || j < 0 || i >= n_ || j >= n_ || i == j) return false;
    const auto& row = adjacency_[static_cast<std::size_t>(i)];
    return std::binary_search(row.begin(), row.end(), j);
  }

  /// @brief Neighbours of literal `(col,value)` in the literal conflict table.
  std::pair<const int*, const int*> literal_neighbours(int col,
                                                       bool value_one) const {
    const int idx = literal_index(col, value_one);
    if (idx < 0 || static_cast<std::size_t>(idx) >= literal_adjacency_.size()) {
      return {nullptr, nullptr};
    }
    const auto& row = literal_adjacency_[static_cast<std::size_t>(idx)];
    if (row.empty()) return {nullptr, nullptr};
    return {row.data(), row.data() + row.size()};
  }

  /// @brief True iff two literals are mutually exclusive.
  bool has_literal_edge(int col_a,
                        bool value_a_one,
                        int col_b,
                        bool value_b_one) const {
    const int ia = literal_index(col_a, value_a_one);
    const int ib = literal_index(col_b, value_b_one);
    if (ia < 0 || ib < 0 || ia == ib ||
        static_cast<std::size_t>(ia) >= literal_adjacency_.size()) {
      return false;
    }
    const auto& row = literal_adjacency_[static_cast<std::size_t>(ia)];
    return std::binary_search(row.begin(), row.end(), ib);
  }

  /// @brief Propagate clique table implications through bounds.
  /// @details For every variable `i` whose lower bound reaches 1 (fixed to 1),
  /// forces `ub[j] = 0` for all neighbours `j`. Returns the number of bounds
  /// tightened. Safe to call repeatedly; fixing `j` to 0 does not cascade
  /// (we do not assume transitive implication without a fresh `x_i = 1`
  /// trigger).
  int propagate(Eigen::VectorXd& lb,
                Eigen::VectorXd& ub,
                std::vector<BoundChangeInfo>* changes_out = nullptr) const;

  /// @brief Enumerate greedy cliques that are violated by the current LP
  /// solution `x` (i.e. `sum_{i in clique} x_i > 1 + tol`).
  /// @details Seeds from fractional binaries (0.1 < x_j < 0.9) sorted by
  /// descending `x_j`. Returns at most `max_cliques` cliques, each of size
  /// `>= 2` and sorted in ascending column index.
  struct ViolatedClique {
    std::vector<int> members;
    double violation;  // sum(x_i) - 1
  };
  std::vector<ViolatedClique> find_violated_cliques(
      const Eigen::VectorXd& x,
      int max_cliques,
      double tol = 1e-7,
      int max_seeds = 512) const;

 private:
  static int literal_index(int col, bool value_one) {
    if (col < 0) return -1;
    return 2 * col + (value_one ? 1 : 0);
  }

  void ensure_storage(int n);

  int n_{0};
  std::size_t n_edges_{0};
  std::size_t n_literal_edges_{0};
  std::vector<std::vector<int>> adjacency_;
  // Adjacency on literal ids 2*col+value.
  std::vector<std::vector<int>> literal_adjacency_;
};

}  // namespace mipsolvers::engine::detail
