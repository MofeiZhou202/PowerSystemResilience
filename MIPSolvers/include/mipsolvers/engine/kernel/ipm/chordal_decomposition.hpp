#pragma once

// Exact chordal decomposition for sparse SDP blocks.
//
// A sparse PSD slack is represented as a sum of embedded clique slacks
// (Agler decomposition).  Linear consistency rows tie that sum to the
// original affine matrix.  This is an exact conversion, not the weaker
// PSD-completion relaxation obtained by checking clique principal minors.

#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

struct ChordalDecompositionOptions {
  bool enabled{true};
  int min_block_order{32};
  double max_clique_ratio{0.75};
  double max_expansion_ratio{2.0};
  double sparsity_tolerance{1e-14};
};

struct ChordalBlockRecovery {
  int original_offset{0};
  int order{0};
  int equality_offset{0};
  std::vector<int> packed_rows;
  std::vector<int> elimination_order;
  std::vector<std::vector<int>> later_neighbors;
};

struct ChordalDecompositionMap {
  bool applied{false};
  int original_num_variables{0};
  int original_num_equalities{0};
  int original_conic_rows{0};
  int clique_count{0};
  int max_clique_order{0};
  std::vector<int> transformed_conic_to_original;
  std::vector<ChordalBlockRecovery> blocks;
};

/// Build an exact clique-converted model when at least one SDP block passes
/// the structural benefit gates.  Returns false and leaves transformed
/// unspecified when no block should be decomposed.
[[nodiscard]] bool chordal_decompose(
    const ConicModel& original, const ChordalDecompositionOptions& options,
    ConicModel& transformed, ChordalDecompositionMap& map);

/// Recover the original variable/cone layout.  transformed_y contains both
/// original equalities and generated consistency rows; the latter define a
/// chordal partial dual matrix, completed here through the elimination tree.
void chordal_recover(
    const ConicModel& original, const ChordalDecompositionMap& map,
    const Eigen::VectorXd& transformed_x,
    const Eigen::VectorXd& transformed_y,
    const Eigen::VectorXd& transformed_z, Eigen::VectorXd& x,
    Eigen::VectorXd& y, Eigen::VectorXd& s, Eigen::VectorXd& z);

}  // namespace mipsolvers::engine
