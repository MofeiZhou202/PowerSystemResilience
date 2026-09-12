/// @file bc_implied_bounds.hpp
/// @brief Implied-integrality and variable-bound source helpers for legacy B&C.

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/strategy/highs_presolve_side_state.hpp"

namespace mipsolvers::engine::detail {

class BinaryImplicationGraph;
class CliqueTable;
class VariableBoundTable;

struct IntegralRowTighteningStats {
  std::uint64_t integral_rows{0};
  std::uint64_t upper_tightened{0};
  std::uint64_t lower_tightened{0};
  double max_upper_delta{0.0};
  double max_lower_delta{0.0};
};

struct RootImpliedIntegerArtifactStats {
  std::uint64_t row_sides_scanned{0};
  std::uint64_t row_sides_with_binary_literals{0};
  std::uint64_t bound_candidates{0};
  std::uint64_t bound_improvements{0};
  std::uint64_t implications_added{0};
  std::uint64_t conflicts_added{0};
  std::uint64_t clique_edges_added{0};
  std::uint64_t literal_pair_tests{0};
  std::uint64_t literal_pair_conflicts{0};
  std::uint64_t objective_literal_pair_tests{0};
  std::uint64_t objective_literal_pair_conflicts{0};
};

struct NativeVariableBoundSourceStats {
  std::uint64_t row_sides_scanned{0};
  std::uint64_t single_binary_row_sides{0};
  std::uint64_t skipped_multi_binary{0};
  std::uint64_t candidates{0};
  std::uint64_t exported_implications{0};
  std::uint64_t cut_rows_scanned{0};
  std::uint64_t cut_mixed_rows{0};
  std::uint64_t cut_vub_candidates{0};
  std::uint64_t cut_vlb_candidates{0};
  std::uint64_t cut_exported_implications{0};
  std::uint64_t presolve_vub_candidates{0};
  std::uint64_t presolve_vlb_candidates{0};
  std::uint64_t presolve_varbounds_replayed{0};
  std::uint64_t presolve_exported_implications{0};
  std::uint64_t implied_bound_passes{0};
  std::uint64_t implied_bound_lower{0};
  std::uint64_t implied_bound_upper{0};
  std::uint64_t probe_vub_candidates{0};
  std::uint64_t probe_vlb_candidates{0};
  std::uint64_t probe_varbounds_replayed{0};
  std::uint64_t probe_global_tightenings{0};
  std::uint64_t probe_exported_implications{0};
};

struct NativeVariableBoundSourceEntry {
  bool is_vub{true};
  int col{-1};
  int trigger_col{-1};
  double coef{0.0};
  double constant{0.0};
};

std::vector<char> detect_implied_integer_columns(const LPModel& lp);

IntegralRowTighteningStats tighten_integral_row_sides(
    LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    double feastol);

void accumulate_variable_bound_source_stats(
    NativeVariableBoundSourceStats& dst,
    const NativeVariableBoundSourceStats& src);

std::vector<NativeVariableBoundSourceEntry> collect_variable_bound_sources(
    const VariableBoundTable& table);

/// Implied per-column bounds derived from row-activity propagation over the
/// model rows (Savelsbergh 1994, §2; Achterberg 2007, §3.2).  lower/upper are
/// the tightened bounds (-inf/+inf when untouched); lower_source/upper_source
/// record the producing row index (-1 when untouched).
struct NativeImpliedColumnBounds {
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<int> lower_source;
  std::vector<int> upper_source;
  std::uint64_t passes{0};
  std::uint64_t lower_tightened{0};
  std::uint64_t upper_tightened{0};
};

NativeImpliedColumnBounds compute_implied_column_bounds_from_rows(
    const LPModel& lp,
    int max_passes = 16,
    int max_row_nnz = 512,
    double tol = 1e-9);

void trace_highs_native_varbound_diff(
    const char* phase,
    const HiGHSPresolvedModelStats& highs_state,
    const LPModel* lp,
    const VariableBoundTable& table,
    int max_samples = 8);

NativeVariableBoundSourceStats build_variable_bound_table_from_rows(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    VariableBoundTable& table,
    BinaryImplicationGraph* implication_graph = nullptr,
    int max_row_nnz = 512);

NativeVariableBoundSourceStats augment_variable_bound_table_from_cut_rows(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    int first_row,
    int last_row,
    VariableBoundTable& table,
    BinaryImplicationGraph* implication_graph = nullptr,
    int max_cut_nnz = 100,
    double feastol = 1e-7);

NativeVariableBoundSourceStats augment_variable_bound_table_from_sparse_cut_rows(
    const LPModel& domain_lp,
    const std::vector<Eigen::SparseVector<double>>& cut_rows,
    const std::vector<double>& cut_rhs,
    const std::vector<char>& implied_integer_cols,
    VariableBoundTable& table,
    BinaryImplicationGraph* implication_graph = nullptr,
    int max_cut_nnz = 100,
    double feastol = 1e-7);

RootImpliedIntegerArtifactStats build_implied_integer_row_artifacts(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    CliqueTable& clique_table,
    BinaryImplicationGraph& implication_graph,
    int max_row_nnz = 256,
    int max_binary_literals_per_row = 96,
    std::uint64_t max_implications = 250000,
    const Eigen::VectorXd* lp_solution = nullptr);

}  // namespace mipsolvers::engine::detail
