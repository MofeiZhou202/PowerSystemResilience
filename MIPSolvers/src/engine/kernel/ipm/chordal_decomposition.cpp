#include "mipsolvers/engine/kernel/ipm/chordal_decomposition.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/ipm/cones.hpp"

namespace mipsolvers::engine {
namespace {

using RowSparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;

struct BlockPlan {
  int original_offset = 0;
  int order = 0;
  std::vector<std::vector<int>> cliques;
  std::vector<int> packed_rows;
  std::vector<int> elimination_order;
  std::vector<std::vector<int>> later_neighbors;
  std::vector<int> clique_variable_offsets;
  std::vector<int> clique_cone_offsets;
  int equality_offset = 0;
};

[[nodiscard]] bool is_subset(const std::vector<int>& small,
                             const std::vector<int>& large) {
  return std::includes(large.begin(), large.end(), small.begin(), small.end());
}

[[nodiscard]] std::optional<BlockPlan> analyze_block(
    const RowSparseMatrix& grow, const Eigen::VectorXd& h, int offset, int p,
    const ChordalDecompositionOptions& options) {
  if (p < options.min_block_order) return std::nullopt;

  std::vector<std::unordered_set<int>> adjacency(static_cast<std::size_t>(p));
  std::size_t edge_count = 0;
  for (int j = 0; j < p; ++j) {
    for (int i = j + 1; i < p; ++i) {
      const int packed = svec_index(i, j, p);
      const int row = offset + packed;
      const bool active_h = std::abs(h[row]) > options.sparsity_tolerance;
      const bool active_g = grow.outerIndexPtr()[row] !=
                            grow.outerIndexPtr()[row + 1];
      if (!active_h && !active_g) continue;
      adjacency[static_cast<std::size_t>(i)].insert(j);
      adjacency[static_cast<std::size_t>(j)].insert(i);
      ++edge_count;
    }
  }

  const double possible_edges = 0.5 * static_cast<double>(p) * (p - 1);
  if (possible_edges > 0.0 &&
      static_cast<double>(edge_count) / possible_edges >
          options.max_clique_ratio) {
    return std::nullopt;
  }

  BlockPlan plan;
  plan.original_offset = offset;
  plan.order = p;
  plan.later_neighbors.resize(static_cast<std::size_t>(p));
  plan.elimination_order.reserve(static_cast<std::size_t>(p));
  std::vector<char> alive(static_cast<std::size_t>(p), 1);
  std::vector<std::vector<int>> candidates;
  candidates.reserve(static_cast<std::size_t>(p));

  // Deterministic greedy minimum-degree fill.  The saved later-neighbor sets
  // form a perfect elimination ordering of the generated chordal extension.
  for (int step = 0; step < p; ++step) {
    int vertex = -1;
    std::size_t best_degree = std::numeric_limits<std::size_t>::max();
    for (int v = 0; v < p; ++v) {
      if (!alive[static_cast<std::size_t>(v)]) continue;
      const std::size_t degree = adjacency[static_cast<std::size_t>(v)].size();
      if (degree < best_degree) {
        best_degree = degree;
        vertex = v;
      }
    }

    std::vector<int> neighbors(
        adjacency[static_cast<std::size_t>(vertex)].begin(),
        adjacency[static_cast<std::size_t>(vertex)].end());
    std::sort(neighbors.begin(), neighbors.end());
    plan.later_neighbors[static_cast<std::size_t>(vertex)] = neighbors;
    plan.elimination_order.push_back(vertex);

    std::vector<int> clique = neighbors;
    clique.push_back(vertex);
    std::sort(clique.begin(), clique.end());
    candidates.push_back(std::move(clique));

    for (std::size_t a = 0; a < neighbors.size(); ++a) {
      for (std::size_t b = a + 1; b < neighbors.size(); ++b) {
        const int u = neighbors[a];
        const int v = neighbors[b];
        adjacency[static_cast<std::size_t>(u)].insert(v);
        adjacency[static_cast<std::size_t>(v)].insert(u);
      }
    }
    for (int u : neighbors) {
      adjacency[static_cast<std::size_t>(u)].erase(vertex);
    }
    adjacency[static_cast<std::size_t>(vertex)].clear();
    alive[static_cast<std::size_t>(vertex)] = 0;
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) {
              if (a.size() != b.size()) return a.size() > b.size();
              return a < b;
            });
  for (const auto& candidate : candidates) {
    bool contained = false;
    for (const auto& maximal : plan.cliques) {
      if (is_subset(candidate, maximal)) {
        contained = true;
        break;
      }
    }
    if (!contained) plan.cliques.push_back(candidate);
  }

  int max_clique = 0;
  std::size_t clique_entries = 0;
  for (const auto& clique : plan.cliques) {
    const int size = static_cast<int>(clique.size());
    max_clique = std::max(max_clique, size);
    clique_entries += static_cast<std::size_t>(svec_size(size));
  }
  const std::size_t original_entries = static_cast<std::size_t>(svec_size(p));
  if (plan.cliques.size() <= 1 ||
      static_cast<double>(max_clique) > options.max_clique_ratio * p ||
      static_cast<double>(clique_entries) >
          options.max_expansion_ratio * original_entries) {
    return std::nullopt;
  }

  plan.packed_rows.reserve(original_entries);
  for (int v = 0; v < p; ++v) {
    plan.packed_rows.push_back(svec_index(v, v, p));
    for (int u : plan.later_neighbors[static_cast<std::size_t>(v)]) {
      plan.packed_rows.push_back(svec_index(std::max(u, v), std::min(u, v), p));
    }
  }
  std::sort(plan.packed_rows.begin(), plan.packed_rows.end());
  plan.packed_rows.erase(
      std::unique(plan.packed_rows.begin(), plan.packed_rows.end()),
      plan.packed_rows.end());
  return plan;
}

void complete_dual(const ChordalBlockRecovery& block,
                   const Eigen::VectorXd& transformed_y,
                   Eigen::Ref<Eigen::VectorXd> packed_out) {
  const int p = block.order;
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(p, p);
  for (std::size_t k = 0; k < block.packed_rows.size(); ++k) {
    const int packed = block.packed_rows[k];
    int cursor = 0;
    for (int j = 0; j < p; ++j) {
      const int count = p - j;
      if (packed < cursor + count) {
        const int i = j + packed - cursor;
        const double raw = transformed_y[block.equality_offset +
                                         static_cast<int>(k)];
        const double value = (i == j) ? raw : raw / std::sqrt(2.0);
        matrix(i, j) = value;
        matrix(j, i) = value;
        break;
      }
      cursor += count;
    }
  }

  std::vector<int> position(static_cast<std::size_t>(p));
  for (int k = 0; k < p; ++k) {
    position[static_cast<std::size_t>(block.elimination_order[k])] = k;
  }
  for (int ord = p - 1; ord >= 0; --ord) {
    const int v = block.elimination_order[static_cast<std::size_t>(ord)];
    const auto& neighbors = block.later_neighbors[static_cast<std::size_t>(v)];
    std::vector<char> is_neighbor(static_cast<std::size_t>(p), 0);
    for (int u : neighbors) is_neighbor[static_cast<std::size_t>(u)] = 1;
    std::vector<int> remainder;
    for (int u = 0; u < p; ++u) {
      if (position[static_cast<std::size_t>(u)] > ord &&
          !is_neighbor[static_cast<std::size_t>(u)]) {
        remainder.push_back(u);
      }
    }
    if (neighbors.empty()) continue;

    Eigen::MatrixXd nn(neighbors.size(), neighbors.size());
    Eigen::VectorXd nv(neighbors.size());
    for (std::size_t i = 0; i < neighbors.size(); ++i) {
      nv[static_cast<Eigen::Index>(i)] = matrix(neighbors[i], v);
      for (std::size_t j = 0; j < neighbors.size(); ++j) {
        nn(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
            matrix(neighbors[i], neighbors[j]);
      }
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(nn);
    const double scale = std::max(1.0, es.eigenvalues().cwiseAbs().maxCoeff());
    Eigen::VectorXd inv = es.eigenvalues().unaryExpr(
        [scale](double value) { return value > 1e-10 * scale ? 1.0 / value : 0.0; });
    const Eigen::VectorXd coeff =
        es.eigenvectors() * inv.asDiagonal() * es.eigenvectors().transpose() * nv;
    for (int u : remainder) {
      double value = 0.0;
      for (std::size_t k = 0; k < neighbors.size(); ++k) {
        value += coeff[static_cast<Eigen::Index>(k)] * matrix(neighbors[k], u);
      }
      matrix(v, u) = value;
      matrix(u, v) = value;
    }
  }
  packed_out = svec(matrix);
}

}  // namespace

bool chordal_decompose(const ConicModel& original,
                       const ChordalDecompositionOptions& options,
                       ConicModel& transformed,
                       ChordalDecompositionMap& map) {
  map = {};
  if (!options.enabled || original.dims.s.empty()) return false;

  const int old_n = static_cast<int>(original.c.size());
  const int old_meq = static_cast<int>(original.A.rows());
  const ConeLayout layout = ConeLayout::build(original.dims);
  const RowSparseMatrix grow = original.G;
  std::vector<std::optional<BlockPlan>> plans(layout.s_orders.size());
  int selected = 0;
  for (std::size_t b = 0; b < layout.s_orders.size(); ++b) {
    plans[b] = analyze_block(grow, original.h, layout.s_offsets[b],
                             layout.s_orders[b], options);
    if (plans[b]) ++selected;
  }
  if (selected == 0) return false;

  int new_n = old_n;
  int generated_equalities = 0;
  ConeDims new_dims;
  new_dims.l = original.dims.l;
  new_dims.q = original.dims.q;
  for (std::size_t b = 0; b < plans.size(); ++b) {
    if (!plans[b]) {
      new_dims.s.push_back(layout.s_orders[b]);
      continue;
    }
    BlockPlan& plan = *plans[b];
    plan.clique_variable_offsets.reserve(plan.cliques.size());
    for (const auto& clique : plan.cliques) {
      plan.clique_variable_offsets.push_back(new_n);
      new_n += svec_size(static_cast<int>(clique.size()));
      new_dims.s.push_back(static_cast<int>(clique.size()));
    }
    plan.equality_offset = old_meq + generated_equalities;
    generated_equalities += static_cast<int>(plan.packed_rows.size());
  }

  const int new_meq = old_meq + generated_equalities;
  const int old_m = layout.total;
  const int new_m = new_dims.total();
  std::vector<int> old_to_new_row(static_cast<std::size_t>(old_m), -1);
  map.transformed_conic_to_original.assign(static_cast<std::size_t>(new_m), -1);

  const int scalar_soc_rows = layout.s_offsets.empty()
                                  ? old_m
                                  : layout.s_offsets.front();
  for (int row = 0; row < scalar_soc_rows; ++row) {
    old_to_new_row[static_cast<std::size_t>(row)] = row;
    map.transformed_conic_to_original[static_cast<std::size_t>(row)] = row;
  }
  int cone_cursor = scalar_soc_rows;
  for (std::size_t b = 0; b < plans.size(); ++b) {
    if (!plans[b]) {
      const int count = svec_size(layout.s_orders[b]);
      for (int k = 0; k < count; ++k) {
        old_to_new_row[static_cast<std::size_t>(layout.s_offsets[b] + k)] =
            cone_cursor + k;
        map.transformed_conic_to_original[static_cast<std::size_t>(cone_cursor + k)] =
            layout.s_offsets[b] + k;
      }
      cone_cursor += count;
      continue;
    }
    BlockPlan& plan = *plans[b];
    for (const auto& clique : plan.cliques) {
      plan.clique_cone_offsets.push_back(cone_cursor);
      cone_cursor += svec_size(static_cast<int>(clique.size()));
    }
  }

  transformed = {};
  transformed.sense = original.sense;
  transformed.dims = std::move(new_dims);
  transformed.c = Eigen::VectorXd::Zero(new_n);
  transformed.c.head(old_n) = original.c;
  transformed.h = Eigen::VectorXd::Zero(new_m);
  transformed.b = Eigen::VectorXd::Zero(new_meq);
  transformed.b.head(old_meq) = original.b;

  std::vector<Eigen::Triplet<double>> gtriplets;
  std::vector<Eigen::Triplet<double>> atriplets;
  gtriplets.reserve(static_cast<std::size_t>(original.G.nonZeros()) +
                    static_cast<std::size_t>(new_n - old_n));
  atriplets.reserve(static_cast<std::size_t>(original.A.nonZeros()) +
                    static_cast<std::size_t>(original.G.nonZeros()) +
                    static_cast<std::size_t>(new_n - old_n));

  for (int col = 0; col < old_n; ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(original.G, col); it; ++it) {
      const int new_row = old_to_new_row[static_cast<std::size_t>(it.row())];
      if (new_row >= 0) gtriplets.emplace_back(new_row, col, it.value());
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(original.A, col); it; ++it) {
      atriplets.emplace_back(it.row(), col, it.value());
    }
  }
  for (int old_row = 0; old_row < old_m; ++old_row) {
    const int new_row = old_to_new_row[static_cast<std::size_t>(old_row)];
    if (new_row >= 0) transformed.h[new_row] = original.h[old_row];
  }

  for (std::size_t b = 0; b < plans.size(); ++b) {
    if (!plans[b]) continue;
    BlockPlan& plan = *plans[b];
    const int mp = svec_size(plan.order);
    std::vector<int> consistency_row(static_cast<std::size_t>(mp), -1);
    for (std::size_t k = 0; k < plan.packed_rows.size(); ++k) {
      const int eq = plan.equality_offset + static_cast<int>(k);
      const int packed = plan.packed_rows[k];
      consistency_row[static_cast<std::size_t>(packed)] = eq;
      transformed.b[eq] = original.h[plan.original_offset + packed];
      for (RowSparseMatrix::InnerIterator it(grow,
                                             plan.original_offset + packed);
           it; ++it) {
        atriplets.emplace_back(eq, it.col(), it.value());
      }
    }

    for (std::size_t c = 0; c < plan.cliques.size(); ++c) {
      const auto& clique = plan.cliques[c];
      const int q = static_cast<int>(clique.size());
      const int var_offset = plan.clique_variable_offsets[c];
      const int cone_offset = plan.clique_cone_offsets[c];
      int local = 0;
      for (int lj = 0; lj < q; ++lj) {
        for (int li = lj; li < q; ++li) {
          const int variable = var_offset + local;
          gtriplets.emplace_back(cone_offset + local, variable, -1.0);
          const int full_packed = svec_index(clique[li], clique[lj], plan.order);
          const int eq = consistency_row[static_cast<std::size_t>(full_packed)];
          atriplets.emplace_back(eq, variable, 1.0);
          ++local;
        }
      }
    }

    ChordalBlockRecovery recovery;
    recovery.original_offset = plan.original_offset;
    recovery.order = plan.order;
    recovery.equality_offset = plan.equality_offset;
    recovery.packed_rows = plan.packed_rows;
    recovery.elimination_order = plan.elimination_order;
    recovery.later_neighbors = plan.later_neighbors;
    map.max_clique_order = std::max(
        map.max_clique_order,
        static_cast<int>(plan.cliques.front().size()));
    map.clique_count += static_cast<int>(plan.cliques.size());
    map.blocks.push_back(std::move(recovery));
  }

  transformed.G.resize(new_m, new_n);
  transformed.G.setFromTriplets(gtriplets.begin(), gtriplets.end());
  transformed.G.makeCompressed();
  transformed.A.resize(new_meq, new_n);
  transformed.A.setFromTriplets(atriplets.begin(), atriplets.end());
  transformed.A.makeCompressed();
  if (!original.vars.empty()) {
    transformed.vars = original.vars;
    transformed.vars.reserve(static_cast<std::size_t>(new_n));
    for (int j = old_n; j < new_n; ++j) {
      transformed.vars.push_back(
          VariableMeta{VarType::Continuous, -1e20, 1e20,
                       "chordal_" + std::to_string(j - old_n)});
    }
  }

  map.applied = true;
  map.original_num_variables = old_n;
  map.original_num_equalities = old_meq;
  map.original_conic_rows = old_m;
  return true;
}

void chordal_recover(const ConicModel& original,
                     const ChordalDecompositionMap& map,
                     const Eigen::VectorXd& transformed_x,
                     const Eigen::VectorXd& transformed_y,
                     const Eigen::VectorXd& transformed_z,
                     Eigen::VectorXd& x, Eigen::VectorXd& y,
                     Eigen::VectorXd& s, Eigen::VectorXd& z) {
  x = transformed_x.head(map.original_num_variables);
  y = transformed_y.head(map.original_num_equalities);
  s = original.h - original.G * x;
  z = Eigen::VectorXd::Zero(map.original_conic_rows);
  for (std::size_t row = 0; row < map.transformed_conic_to_original.size();
       ++row) {
    const int original_row = map.transformed_conic_to_original[row];
    if (original_row >= 0 && static_cast<int>(row) < transformed_z.size()) {
      z[original_row] = transformed_z[static_cast<Eigen::Index>(row)];
    }
  }
  for (const ChordalBlockRecovery& block : map.blocks) {
    complete_dual(block, transformed_y,
                  z.segment(block.original_offset, svec_size(block.order)));
  }
}

}  // namespace mipsolvers::engine
