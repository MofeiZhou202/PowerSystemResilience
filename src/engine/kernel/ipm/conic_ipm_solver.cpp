// ═══════════════════════════════════════════════════════════════════════════
// Native conic interior-point solver — implementation.
//
// Mehrotra predictor-corrector IPM for cvxopt standard form with Nesterov-
// Todd scalings (CVXOPT conelp algorithm).  Per iteration the Newton system
//   [ 0   A'  G' ] [dx]   [ -rc          ]
//   [ A   0   0  ] [dy] = [ -rx          ]
//   [ G   0  -H  ] [dz]   [ -rs - W r_l  ]
// is solved through the reduced symmetric KKT matrix
//   K = [ G' H^{-1} G + delta I , A' ; A , -delta I ]
// whose sparsity pattern is fixed across iterations.  K is factored with
// CHOLMOD when it is SPD (no equality rows; delta escalates x10 up to 1e-6
// on failure); otherwise the code falls back to MUMPS (symmetric indefinite
// LDL^T) for the rest of the solve when it is compiled in.
// ═══════════════════════════════════════════════════════════════════════════

#include "mipsolvers/engine/kernel/ipm/conic_ipm_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <fmt/core.h>

#include "mipsolvers/engine/kernel/ipm/chordal_decomposition.hpp"
#include "mipsolvers/engine/kernel/ipm/cones.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp"
#if defined(MIPSOLVERS_HAVE_MUMPS)
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"
#endif

// OpenMP for the s-block Schur assembly (parallel over columns; each column
// writes only its own CSC entries, so results are deterministic regardless
// of the thread count).  Mirrors the guard style of ipm_lp_solver.cpp.
#if defined(MIPSOLVERS_USE_OPENMP)
#include <omp.h>
#define MIPSOLVERS_OMP_STR_(x) #x
#define MIPSOLVERS_OMP_STR(x) MIPSOLVERS_OMP_STR_(x)
#define MIPSOLVERS_OMP_PARALLEL_IF(cond) \
  _Pragma(MIPSOLVERS_OMP_STR(omp parallel for if(cond)))
#else
#define MIPSOLVERS_OMP_PARALLEL_IF(cond)
#endif

namespace mipsolvers::engine {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDelta0 = 1e-9;     ///< Initial KKT diagonal regularization.
constexpr double kDeltaMax = 1e-6;   ///< Largest delta tried before fallback.
constexpr double kStepDamp = 0.99;   ///< Damping factor on the max step.
constexpr double kSqrt2 = 1.41421356237309504880168872420969808;
[[maybe_unused]] constexpr long kSchurParallelThreshold = 4096;  ///< Work gate for OpenMP.

/// Sets the OpenMP thread count for the lifetime of the object (no-op when
/// OpenMP is disabled or num_threads <= 0).
struct OmpThreadsGuard {
#if defined(MIPSOLVERS_USE_OPENMP)
  explicit OmpThreadsGuard(int num_threads) {
    if (num_threads > 0) {
      prev_ = omp_get_max_threads();
      omp_set_num_threads(num_threads);
      active_ = true;
    }
  }
  ~OmpThreadsGuard() {
    if (active_) omp_set_num_threads(prev_);
  }
  int prev_ = 0;
  bool active_ = false;
#else
  explicit OmpThreadsGuard(int) {}
#endif
};

// ═══════════════════════════════════════════════════════════════════════════
// KKT sparsity pattern + per-iteration value assembly for
// K = [G' H^{-1} G + delta I, A'; A, -delta I]  (lower-triangular CSC).
// ═══════════════════════════════════════════════════════════════════════════
struct KktAssembler {
  struct Entry {        ///< (index, value) inside a cached row/col list.
    int idx;
    double val;
  };
  struct FlatEntries {
    std::vector<int> outer;
    std::vector<Entry> entries;

    [[nodiscard]] std::size_t size() const {
      return outer.empty() ? 0 : outer.size() - 1;
    }
    [[nodiscard]] std::span<const Entry> operator[](std::size_t i) const {
      const std::size_t begin = static_cast<std::size_t>(outer[i]);
      const std::size_t end = static_cast<std::size_t>(outer[i + 1]);
      if (begin == end) return {};
      return {entries.data() + begin, end - begin};
    }
  };
  struct QColRef {      ///< Membership of a column in a q-block clique.
    int block;
    int jidx;
  };
  struct SColRef {      ///< Membership of a column in an s-block clique.
    int block;
    int jidx;
  };
  struct QBlock {
    int offset = 0;
    int size = 0;
    std::vector<int> clique;                     ///< Sorted columns touching the block.
    std::vector<std::vector<Entry>> col_entries; ///< [clique idx] -> (local row, value)
    std::vector<std::vector<Entry>> row_entries; ///< [local row] -> (column, value)
    std::vector<std::vector<int>> rank1_pos;     ///< [jidx] -> CSC positions of (clique[iidx>=jidx], j)
  };
  struct SBlock {
    int offset = 0;
    int order = 0;
    std::vector<int> clique;
    std::vector<std::vector<Entry>> col_entries;  ///< [clique idx] -> (local packed row, value)
    std::vector<std::vector<int>> schur_pos;      ///< [jidx] -> CSC positions of (clique[iidx>=jidx], j)
    std::vector<int> row_i;  ///< Packed row -> lower-triangle matrix row.
    std::vector<int> row_j;  ///< Packed row -> lower-triangle matrix column.
  };

  int n = 0;
  int meq = 0;
  int dim = 0;  ///< N = n + meq.

  // Cached static entries (G/A are fixed across iterations).
  FlatEntries scalar_rows;                       ///< l rows, then size-1 q rows.
  std::vector<int> scalar_extra;                 ///< l_extra_ index for size-1 q rows (-1 for l rows).
  FlatEntries scalar_cols;                       ///< [column] -> (scalar row, value).
  std::vector<QBlock> q_blocks;                  ///< Size >= 2 blocks only.
  std::vector<std::vector<QColRef>> q_col_blocks;///< [column] -> clique memberships.
  std::vector<SBlock> s_blocks;
  std::vector<std::vector<SColRef>> s_col_blocks;///< [column] -> clique memberships.
  FlatEntries a_cols;                            ///< [column] -> (eq row, value).

  // CSC of K (lower triangle, sorted rows per column).
  std::vector<int> outer;
  std::vector<int> inner;
  std::vector<double> values;
  std::vector<double> diag_base;  ///< B diagonal without the delta shift.

  /// Build the fixed pattern and all cached entry lists.
  void build(const ConicModel& model, const ConeLayout& lay);
  /// Assemble the numeric values of K for the current NT scaling.
  void assemble(const ConeNtScaling& sc, double delta);
  /// Change the diagonal regularization without touching the off-diagonals.
  void apply_delta(double delta);

 private:
  std::vector<int> scatter_;  ///< Reused row -> CSC position workspace.
};

void KktAssembler::build(const ConicModel& model, const ConeLayout& lay) {
  n = static_cast<int>(model.c.size());
  meq = static_cast<int>(model.A.rows());
  dim = n + meq;

  Eigen::SparseMatrix<double> gt = model.G.transpose();
  gt.makeCompressed();

  // ── Scalar (l-style) rows: orthant rows first, then size-1 q blocks ────
  scalar_rows.outer.reserve(static_cast<std::size_t>(lay.l) +
                            lay.q_sizes.size() + 1);
  scalar_rows.outer.push_back(0);
  for (int r = 0; r < lay.l; ++r) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(gt, r); it; ++it) {
      scalar_rows.entries.push_back({static_cast<int>(it.row()), it.value()});
    }
    scalar_rows.outer.push_back(static_cast<int>(scalar_rows.entries.size()));
    scalar_extra.push_back(-1);
  }
  int extra_idx = 0;
  for (std::size_t b = 0; b < lay.q_sizes.size(); ++b) {
    if (lay.q_sizes[b] != 1) continue;
    const int r = lay.q_offsets[b];
    for (Eigen::SparseMatrix<double>::InnerIterator it(gt, r); it; ++it) {
      scalar_rows.entries.push_back({static_cast<int>(it.row()), it.value()});
    }
    scalar_rows.outer.push_back(static_cast<int>(scalar_rows.entries.size()));
    scalar_extra.push_back(extra_idx++);
  }
  scalar_cols.outer.assign(static_cast<std::size_t>(n) + 1, 0);
  for (std::size_t sr = 0; sr < scalar_rows.size(); ++sr) {
    for (const Entry& e : scalar_rows[sr]) {
      ++scalar_cols.outer[static_cast<std::size_t>(e.idx) + 1];
    }
  }
  for (int j = 0; j < n; ++j) {
    scalar_cols.outer[static_cast<std::size_t>(j) + 1] +=
        scalar_cols.outer[static_cast<std::size_t>(j)];
  }
  scalar_cols.entries.resize(scalar_rows.entries.size());
  std::vector<int> scalar_cursor = scalar_cols.outer;
  for (std::size_t sr = 0; sr < scalar_rows.size(); ++sr) {
    for (const Entry& e : scalar_rows[sr]) {
      const int dst = scalar_cursor[static_cast<std::size_t>(e.idx)]++;
      scalar_cols.entries[static_cast<std::size_t>(dst)] =
          {static_cast<int>(sr), e.val};
    }
  }

  // ── SOC blocks of size >= 2 ────────────────────────────────────────────
  const bool has_q_blocks = std::any_of(
      lay.q_sizes.begin(), lay.q_sizes.end(), [](int k) { return k >= 2; });
  // Reuse one sparse-reset column-to-clique map across all non-scalar cone
  // blocks.  Pure orthant models do not allocate this O(n) workspace.
  std::vector<int> clique_index;
  if (has_q_blocks || !lay.s_orders.empty()) {
    clique_index.assign(static_cast<std::size_t>(n), -1);
  }
  if (has_q_blocks) q_col_blocks.resize(static_cast<std::size_t>(n));
  for (std::size_t b = 0; b < lay.q_sizes.size(); ++b) {
    const int k = lay.q_sizes[b];
    if (k < 2) continue;
    const int off = lay.q_offsets[b];
    QBlock qb;
    qb.offset = off;
    qb.size = k;
    qb.row_entries.resize(static_cast<std::size_t>(k));
    for (int lr = 0; lr < k; ++lr) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(gt, off + lr); it;
           ++it) {
        const int col = static_cast<int>(it.row());
        qb.row_entries[static_cast<std::size_t>(lr)].push_back({col, it.value()});
        if (clique_index[static_cast<std::size_t>(col)] < 0) {
          clique_index[static_cast<std::size_t>(col)] = 0;
          qb.clique.push_back(col);
        }
      }
    }
    std::sort(qb.clique.begin(), qb.clique.end());
    qb.col_entries.resize(qb.clique.size());
    for (std::size_t jidx = 0; jidx < qb.clique.size(); ++jidx) {
      const int col = qb.clique[jidx];
      clique_index[static_cast<std::size_t>(col)] = static_cast<int>(jidx);
      q_col_blocks[static_cast<std::size_t>(col)].push_back(
          {static_cast<int>(q_blocks.size()), static_cast<int>(jidx)});
    }
    for (int lr = 0; lr < k; ++lr) {
      for (const Entry& e : qb.row_entries[static_cast<std::size_t>(lr)]) {
        const int jidx = clique_index[static_cast<std::size_t>(e.idx)];
        qb.col_entries[static_cast<std::size_t>(jidx)].push_back({lr, e.val});
      }
    }
    for (int col : qb.clique) clique_index[static_cast<std::size_t>(col)] = -1;
    qb.rank1_pos.resize(qb.clique.size());
    q_blocks.push_back(std::move(qb));
  }

  // ── SDP blocks ─────────────────────────────────────────────────────────
  if (!lay.s_orders.empty()) s_col_blocks.resize(static_cast<std::size_t>(n));
  for (std::size_t b = 0; b < lay.s_orders.size(); ++b) {
    const int p = lay.s_orders[b];
    const int off = lay.s_offsets[b];
    const int mp = svec_size(p);
    SBlock sb;
    sb.offset = off;
    sb.order = p;
    sb.row_i.resize(static_cast<std::size_t>(mp));
    sb.row_j.resize(static_cast<std::size_t>(mp));
    int packed_row = 0;
    for (int j = 0; j < p; ++j) {
      for (int i = j; i < p; ++i) {
        sb.row_i[static_cast<std::size_t>(packed_row)] = i;
        sb.row_j[static_cast<std::size_t>(packed_row)] = j;
        ++packed_row;
      }
    }
    for (int lr = 0; lr < mp; ++lr) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(gt, off + lr); it;
           ++it) {
        const int col = static_cast<int>(it.row());
        if (clique_index[static_cast<std::size_t>(col)] < 0) {
          clique_index[static_cast<std::size_t>(col)] = 0;
          sb.clique.push_back(col);
        }
      }
    }
    std::sort(sb.clique.begin(), sb.clique.end());
    sb.col_entries.resize(sb.clique.size());
    for (std::size_t jidx = 0; jidx < sb.clique.size(); ++jidx) {
      const int col = sb.clique[jidx];
      clique_index[static_cast<std::size_t>(col)] = static_cast<int>(jidx);
      s_col_blocks[static_cast<std::size_t>(col)].push_back(
          {static_cast<int>(s_blocks.size()), static_cast<int>(jidx)});
    }
    for (int lr = 0; lr < mp; ++lr) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(gt, off + lr); it;
           ++it) {
        const int col = static_cast<int>(it.row());
        const int jidx = clique_index[static_cast<std::size_t>(col)];
        sb.col_entries[static_cast<std::size_t>(jidx)].push_back({lr, it.value()});
      }
    }
    for (int col : sb.clique) clique_index[static_cast<std::size_t>(col)] = -1;
    sb.schur_pos.resize(sb.clique.size());
    s_blocks.push_back(std::move(sb));
  }

  // ── Equality rows ──────────────────────────────────────────────────────
  if (meq > 0) {
    a_cols.outer.resize(static_cast<std::size_t>(n) + 1);
    a_cols.entries.reserve(static_cast<std::size_t>(model.A.nonZeros()));
    for (int j = 0; j < n; ++j) {
      a_cols.outer[static_cast<std::size_t>(j)] =
          static_cast<int>(a_cols.entries.size());
      for (Eigen::SparseMatrix<double>::InnerIterator it(model.A, j); it; ++it) {
        a_cols.entries.push_back(
            {static_cast<int>(it.row()), it.value()});
      }
    }
    a_cols.outer[static_cast<std::size_t>(n)] =
        static_cast<int>(a_cols.entries.size());
  }

  // ── CSC pattern of the lower triangle of K ─────────────────────────────
  outer.assign(static_cast<std::size_t>(dim) + 1, 0);
  std::vector<std::vector<int>> col_rows(static_cast<std::size_t>(dim));
  for (int j = 0; j < n; ++j) {
    std::vector<int>& rows = col_rows[static_cast<std::size_t>(j)];
    rows.push_back(j);  // diagonal (B + delta I)
    for (const Entry& cref : scalar_cols[static_cast<std::size_t>(j)]) {
      for (const Entry& e : scalar_rows[static_cast<std::size_t>(cref.idx)]) {
        if (e.idx >= j) rows.push_back(e.idx);
      }
    }
    if (!q_col_blocks.empty()) {
      for (const QColRef& mem : q_col_blocks[static_cast<std::size_t>(j)]) {
        const QBlock& qb = q_blocks[static_cast<std::size_t>(mem.block)];
        for (int col : qb.clique) {
          if (col >= j) rows.push_back(col);
        }
      }
    }
    if (!s_col_blocks.empty()) {
      for (const SColRef& mem : s_col_blocks[static_cast<std::size_t>(j)]) {
        const SBlock& sb = s_blocks[static_cast<std::size_t>(mem.block)];
        for (int col : sb.clique) {
          if (col >= j) rows.push_back(col);
        }
      }
    }
    if (meq > 0) {
      for (const Entry& e : a_cols[static_cast<std::size_t>(j)]) {
        rows.push_back(n + e.idx);
      }
    }
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  }
  for (int j = n; j < dim; ++j) {
    col_rows[static_cast<std::size_t>(j)].push_back(j);  // -delta I diagonal
  }
  int nnz = 0;
  for (int j = 0; j < dim; ++j) {
    outer[static_cast<std::size_t>(j)] = nnz;
    nnz += static_cast<int>(col_rows[static_cast<std::size_t>(j)].size());
  }
  outer[static_cast<std::size_t>(dim)] = nnz;
  inner.resize(static_cast<std::size_t>(nnz));
  for (int j = 0; j < dim; ++j) {
    std::copy(col_rows[static_cast<std::size_t>(j)].begin(),
              col_rows[static_cast<std::size_t>(j)].end(),
              inner.begin() + outer[static_cast<std::size_t>(j)]);
  }
  values.assign(static_cast<std::size_t>(nnz), 0.0);
  diag_base.assign(static_cast<std::size_t>(n), 0.0);

  // ── CSC positions of the q rank-1 and s Schur entries ──────────────────
  scatter_.assign(static_cast<std::size_t>(dim), -1);
  for (int j = 0; j < n; ++j) {
    const int begin = outer[static_cast<std::size_t>(j)];
    const int end = outer[static_cast<std::size_t>(j) + 1];
    for (int p = begin; p < end; ++p) {
      scatter_[static_cast<std::size_t>(inner[static_cast<std::size_t>(p)])] = p;
    }
    if (!q_col_blocks.empty()) {
      for (const QColRef& mem : q_col_blocks[static_cast<std::size_t>(j)]) {
        QBlock& qb = q_blocks[static_cast<std::size_t>(mem.block)];
        auto& pos = qb.rank1_pos[static_cast<std::size_t>(mem.jidx)];
        for (std::size_t iidx = static_cast<std::size_t>(mem.jidx);
             iidx < qb.clique.size(); ++iidx) {
          pos.push_back(scatter_[static_cast<std::size_t>(qb.clique[iidx])]);
        }
      }
    }
    if (!s_col_blocks.empty()) {
      for (const SColRef& mem : s_col_blocks[static_cast<std::size_t>(j)]) {
        SBlock& sb = s_blocks[static_cast<std::size_t>(mem.block)];
        auto& pos = sb.schur_pos[static_cast<std::size_t>(mem.jidx)];
        for (std::size_t iidx = static_cast<std::size_t>(mem.jidx);
             iidx < sb.clique.size(); ++iidx) {
          pos.push_back(scatter_[static_cast<std::size_t>(sb.clique[iidx])]);
        }
      }
    }
    for (int p = begin; p < end; ++p) {
      scatter_[static_cast<std::size_t>(inner[static_cast<std::size_t>(p)])] = -1;
    }
  }
}

void KktAssembler::assemble(const ConeNtScaling& sc, double delta) {
  std::fill(values.begin(), values.end(), 0.0);

  // Per-row weights 1/d^2 of the scalar rows.
  std::vector<double> scalar_w(scalar_rows.size(), 0.0);
  for (int r = 0; r < static_cast<int>(sc.l_scaling().size()); ++r) {
    const double d = sc.l_scaling()[r];
    scalar_w[static_cast<std::size_t>(r)] = 1.0 / (d * d);
  }
  for (std::size_t sr = 0; sr < scalar_rows.size(); ++sr) {
    if (scalar_extra[sr] >= 0) {
      const double d = sc.l_extra()[static_cast<std::size_t>(scalar_extra[sr])].second;
      scalar_w[sr] = 1.0 / (d * d);
    }
  }

  // q-block rank-1 vectors u = G_q' J wbar (dense over the clique).
  std::vector<std::vector<double>> q_u(q_blocks.size());
  for (std::size_t b = 0; b < q_blocks.size(); ++b) {
    const QBlock& qb = q_blocks[b];
    const auto& qs = sc.q_scalings()[b];
    std::vector<double>& u = q_u[b];
    u.assign(qb.clique.size(), 0.0);
    for (std::size_t jidx = 0; jidx < qb.clique.size(); ++jidx) {
      double dot = 0.0;
      for (const Entry& e : qb.col_entries[jidx]) {
        dot += e.val * ((e.idx == 0) ? qs.wbar[0] : -qs.wbar[e.idx]);
      }
      u[jidx] = dot;
    }
  }

  // Serial scatter pass: scalar rows, q G'JG parts, q rank-1, A entries.
  for (int j = 0; j < n; ++j) {
    const int begin = outer[static_cast<std::size_t>(j)];
    const int end = outer[static_cast<std::size_t>(j) + 1];
    for (int p = begin; p < end; ++p) {
      scatter_[static_cast<std::size_t>(inner[static_cast<std::size_t>(p)])] = p;
    }
    for (const Entry& cref : scalar_cols[static_cast<std::size_t>(j)]) {
      const double w = scalar_w[static_cast<std::size_t>(cref.idx)];
      for (const Entry& e : scalar_rows[static_cast<std::size_t>(cref.idx)]) {
        if (e.idx >= j) {
          values[static_cast<std::size_t>(scatter_[static_cast<std::size_t>(e.idx)])] +=
              w * e.val * cref.val;
        }
      }
    }
    if (!q_col_blocks.empty()) {
      for (const QColRef& mem : q_col_blocks[static_cast<std::size_t>(j)]) {
        const QBlock& qb = q_blocks[static_cast<std::size_t>(mem.block)];
        const auto& qs = sc.q_scalings()[static_cast<std::size_t>(mem.block)];
        const double beta2inv = 1.0 / (qs.beta * qs.beta);
        for (const Entry& ce : qb.col_entries[static_cast<std::size_t>(mem.jidx)]) {
          const double wr = -beta2inv * ((ce.idx == 0) ? 1.0 : -1.0);
          for (const Entry& e : qb.row_entries[static_cast<std::size_t>(ce.idx)]) {
            if (e.idx >= j) {
              values[static_cast<std::size_t>(scatter_[static_cast<std::size_t>(e.idx)])] +=
                  wr * e.val * ce.val;
            }
          }
        }
        const std::vector<double>& u = q_u[static_cast<std::size_t>(mem.block)];
        const auto& pos = qb.rank1_pos[static_cast<std::size_t>(mem.jidx)];
        for (std::size_t iidx = static_cast<std::size_t>(mem.jidx);
             iidx < qb.clique.size(); ++iidx) {
          values[static_cast<std::size_t>(pos[iidx - static_cast<std::size_t>(mem.jidx)])] +=
              2.0 * beta2inv * u[static_cast<std::size_t>(mem.jidx)] * u[iidx];
        }
      }
    }
    if (meq > 0) {
      for (const Entry& e : a_cols[static_cast<std::size_t>(j)]) {
        values[static_cast<std::size_t>(
            scatter_[static_cast<std::size_t>(n + e.idx)])] = e.val;
      }
    }
    for (int p = begin; p < end; ++p) {
      scatter_[static_cast<std::size_t>(inner[static_cast<std::size_t>(p)])] = -1;
    }
  }

  // SDP Schur complements, parallel over clique columns.  Sparse G columns
  // use the analytic svec basis kernel below, avoiding one zero-filled p x p
  // matrix and two dense GEMMs per column.  Dense blocks retain the BLAS path.
  for (std::size_t b = 0; b < s_blocks.size(); ++b) {
    const SBlock& sb = s_blocks[b];
    const Eigen::MatrixXd& m = sc.s_scalings()[b].h_inv_cong;
    const int p = sb.order;
    const int ncols = static_cast<int>(sb.clique.size());
    long double suffix_nnz = 0.0L;
    long double sparse_work = 0.0L;
    for (int jidx = ncols - 1; jidx >= 0; --jidx) {
      const long double nnz = static_cast<long double>(
          sb.col_entries[static_cast<std::size_t>(jidx)].size());
      suffix_nnz += nnz;
      sparse_work += nnz * suffix_nnz;
    }
    const long double dense_work =
        2.0L * ncols * p * p * p + suffix_nnz * ncols;
    // A scalar analytic kernel has substantially less arithmetic but poorer
    // locality than GEMM.  Require a clear flop advantage before selecting it.
    const bool use_sparse_kernel = 4.0L * sparse_work <= dense_work;
    [[maybe_unused]] const bool run_parallel =
        std::max(sparse_work, dense_work) > kSchurParallelThreshold;

    if (use_sparse_kernel) {
      MIPSOLVERS_OMP_PARALLEL_IF(run_parallel)
      for (int jidx = 0; jidx < ncols; ++jidx) {
        const auto& jentries = sb.col_entries[static_cast<std::size_t>(jidx)];
        const auto& pos = sb.schur_pos[static_cast<std::size_t>(jidx)];
        for (int iidx = jidx; iidx < ncols; ++iidx) {
          double dot = 0.0;
          for (const Entry& je : jentries) {
            const int ji = sb.row_i[static_cast<std::size_t>(je.idx)];
            const int jj = sb.row_j[static_cast<std::size_t>(je.idx)];
            for (const Entry& ie :
                 sb.col_entries[static_cast<std::size_t>(iidx)]) {
              const int ii = sb.row_i[static_cast<std::size_t>(ie.idx)];
              const int ij = sb.row_j[static_cast<std::size_t>(ie.idx)];
              double kernel;
              if (ji == jj) {
                kernel = (ii == ij)
                             ? m(ji, ii) * m(ji, ii)
                             : kSqrt2 * m(ji, ii) * m(ji, ij);
              } else if (ii == ij) {
                kernel = kSqrt2 * m(ji, ii) * m(jj, ii);
              } else {
                kernel = m(ji, ii) * m(jj, ij) +
                         m(ji, ij) * m(jj, ii);
              }
              dot += je.val * ie.val * kernel;
            }
          }
          values[static_cast<std::size_t>(pos[static_cast<std::size_t>(
              iidx - jidx)])] += dot;
        }
      }
    } else {
      MIPSOLVERS_OMP_PARALLEL_IF(run_parallel)
      for (int jidx = 0; jidx < ncols; ++jidx) {
        Eigen::MatrixXd xm = Eigen::MatrixXd::Zero(p, p);
        for (const Entry& e : sb.col_entries[static_cast<std::size_t>(jidx)]) {
          const int i = sb.row_i[static_cast<std::size_t>(e.idx)];
          const int j = sb.row_j[static_cast<std::size_t>(e.idx)];
          const double v = (i == j) ? e.val : e.val / kSqrt2;
          xm(i, j) = v;
          xm(j, i) = v;
        }
        const Eigen::MatrixXd transformed = m * xm * m;
        const auto& pos = sb.schur_pos[static_cast<std::size_t>(jidx)];
        for (int iidx = jidx; iidx < ncols; ++iidx) {
          double dot = 0.0;
          for (const Entry& e : sb.col_entries[static_cast<std::size_t>(iidx)]) {
            const int i = sb.row_i[static_cast<std::size_t>(e.idx)];
            const int j = sb.row_j[static_cast<std::size_t>(e.idx)];
            dot += e.val * ((i == j) ? transformed(i, j)
                                     : kSqrt2 * transformed(i, j));
          }
          values[static_cast<std::size_t>(pos[static_cast<std::size_t>(
              iidx - jidx)])] += dot;
        }
      }
    }
  }

  // Diagonal: remember B's diagonal, then shift by delta.
  for (int j = 0; j < n; ++j) {
    const int dp = outer[static_cast<std::size_t>(j)];  // first entry is the diagonal
    diag_base[static_cast<std::size_t>(j)] = values[static_cast<std::size_t>(dp)];
  }
  apply_delta(delta);
}

void KktAssembler::apply_delta(double delta) {
  for (int j = 0; j < n; ++j) {
    values[static_cast<std::size_t>(outer[static_cast<std::size_t>(j)])] =
        diag_base[static_cast<std::size_t>(j)] + delta;
  }
  for (int j = n; j < dim; ++j) {
    values[static_cast<std::size_t>(outer[static_cast<std::size_t>(j)])] = -delta;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// KKT factorization backend: CHOLMOD for SPD systems (no equality rows),
// MUMPS symmetric-indefinite LDL^T fallback otherwise / on failure.
// ═══════════════════════════════════════════════════════════════════════════
struct KktBackend {
  KktAssembler sys;
  CholmodLDLT chol;
  bool use_mumps = false;
  bool available = false;
#if defined(MIPSOLVERS_HAVE_MUMPS)
  std::unique_ptr<MumpsSolver> mumps;
  std::unique_ptr<Eigen::Map<Eigen::SparseMatrix<double>>> kmap;
#endif

  bool init(const ConicModel& model, const ConeLayout& lay) {
    sys.build(model, lay);
    if (sys.dim == 0) return false;
    // With equality rows the saddle matrix is quasidefinite (indefinite but
    // unpivotable LDLᵀ exists): CHOLMOD's simplicial LDLᵀ handles it, while
    // the supernodal path is SPD-only.  Without equalities K is SPD and the
    // default (auto) mode keeps the faster supernodal path.
    chol.set_simplicial(sys.meq > 0);
    available = chol.analyze(sys.dim, sys.outer.data(), sys.inner.data(),
                             sys.values.data(),
                             static_cast<int64_t>(sys.inner.size()));
    if (!available) {
      use_mumps = switch_to_mumps();
    }
    return available || use_mumps;
  }

  bool switch_to_mumps() {
#if defined(MIPSOLVERS_HAVE_MUMPS)
    if (!mumps) {
      mumps = std::make_unique<MumpsSolver>();
      kmap = std::make_unique<Eigen::Map<Eigen::SparseMatrix<double>>>(
          sys.dim, sys.dim, static_cast<int>(sys.inner.size()),
          sys.outer.data(), sys.inner.data(), sys.values.data());
      mumps->analyze_pattern(*kmap);
    }
    use_mumps = true;
    return mumps->factorize(*kmap);
#else
    return false;
#endif
  }

  /// Factor K for the current scaling (delta escalation, MUMPS fallback).
  bool factor(const ConeNtScaling& sc) {
    sys.assemble(sc, kDelta0);
    if (!use_mumps) {
      double delta = kDelta0;
      bool ok = chol.factorize(sys.values.data());
      while (!ok && delta < kDeltaMax) {
        delta *= 10.0;
        sys.apply_delta(delta);
        ok = chol.factorize(sys.values.data());
      }
      if (ok) return true;
      return switch_to_mumps();  // rest of the solve uses MUMPS
    }
#if defined(MIPSOLVERS_HAVE_MUMPS)
    return mumps->factorize(*kmap);
#else
    return false;
#endif
  }

  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& out) {
    out.resize(sys.dim);
#if defined(MIPSOLVERS_HAVE_MUMPS)
    if (use_mumps) return mumps->solve(rhs, out);
#endif
    return chol.solve(rhs.data(), out.data());
  }
};

// ═══════════════════════════════════════════════════════════════════════════
// Small vector helpers around the (void, out-param) scaling operators.
// ═══════════════════════════════════════════════════════════════════════════
[[nodiscard]] Eigen::VectorXd scaled_w(const ConeNtScaling& sc,
                                       const Eigen::VectorXd& x) {
  Eigen::VectorXd out(x.size());
  sc.apply_w(x, out);
  return out;
}

[[nodiscard]] Eigen::VectorXd scaled_h_inv(const ConeNtScaling& sc,
                                           const Eigen::VectorXd& x) {
  Eigen::VectorXd out(x.size());
  sc.apply_h_inv(x, out);
  return out;
}

/// SDP boundary step when the current matrix is diagonal.  NT-scaled lambda
/// always has this form, so forming an LLT and doing two triangular solves in
/// the generic s_max_step() is redundant in every IPM line search.
[[nodiscard]] double s_diagonal_max_step(
    const Eigen::Ref<const Eigen::VectorXd>& diagonal_point,
    const Eigen::Ref<const Eigen::VectorXd>& direction, int p) {
  Eigen::VectorXd inv_sqrt(p);
  for (int j = 0; j < p; ++j) {
    const double d = diagonal_point[svec_index(j, j, p)];
    if (d <= 0.0) return 0.0;
    inv_sqrt[j] = 1.0 / std::sqrt(d);
  }
  Eigen::MatrixXd scaled = smat(direction, p);
  scaled.array().colwise() *= inv_sqrt.array();
  scaled.array().rowwise() *= inv_sqrt.transpose().array();
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(
      scaled, Eigen::EigenvaluesOnly);
  const double lambda_min = es.eigenvalues()[0];
  return (lambda_min >= 0.0) ? kInf : -1.0 / lambda_min;
}

/// Maximum step over all cone blocks (min of the per-block steps).
[[nodiscard]] double cone_max_step(const ConeLayout& lay,
                                   const Eigen::VectorXd& lam,
                                   const Eigen::VectorXd& dlam) {
  double alpha = l_max_step(lam.data(), dlam.data(), lay.l);
  const int nq = static_cast<int>(lay.q_sizes.size());
  const int ns = static_cast<int>(lay.s_orders.size());
  std::vector<double> block_alpha(static_cast<std::size_t>(nq + ns), kInf);
  [[maybe_unused]] long work = 0;
  for (int q : lay.q_sizes) work += q;
  for (int p : lay.s_orders) work += static_cast<long>(p) * p * p;
  MIPSOLVERS_OMP_PARALLEL_IF((nq + ns > 1) && (work > kSchurParallelThreshold))
  for (int bi = 0; bi < nq + ns; ++bi) {
    if (bi < nq) {
      const int off = lay.q_offsets[static_cast<std::size_t>(bi)];
      const int k = lay.q_sizes[static_cast<std::size_t>(bi)];
      block_alpha[static_cast<std::size_t>(bi)] =
          (k == 1) ? l_max_step(lam.data() + off, dlam.data() + off, 1)
                   : q_max_step(lam.segment(off, k), dlam.segment(off, k));
    } else {
      const int si = bi - nq;
      const int off = lay.s_offsets[static_cast<std::size_t>(si)];
      const int p = lay.s_orders[static_cast<std::size_t>(si)];
      block_alpha[static_cast<std::size_t>(bi)] =
          s_diagonal_max_step(lam.segment(off, svec_size(p)),
                              dlam.segment(off, svec_size(p)), p);
    }
  }
  for (double a : block_alpha) alpha = std::min(alpha, a);
  return alpha;
}

/// True when u is strictly interior in every cone block.
[[nodiscard]] bool cone_interior(const ConeLayout& lay,
                                 const Eigen::VectorXd& u) {
  if (!l_interior(u.data(), lay.l)) return false;
  for (std::size_t b = 0; b < lay.q_sizes.size(); ++b) {
    const int off = lay.q_offsets[b];
    if (!q_interior(u.segment(off, lay.q_sizes[b]))) return false;
  }
  for (std::size_t b = 0; b < lay.s_orders.size(); ++b) {
    const int off = lay.s_offsets[b];
    if (!s_interior(u.segment(off, svec_size(lay.s_orders[b])),
                    lay.s_orders[b])) {
      return false;
    }
  }
  return true;
}

/// Minimal shift alpha such that u + alpha * e touches the cone boundary.
[[nodiscard]] double cone_shift(const ConeLayout& lay,
                                const Eigen::VectorXd& u) {
  double alpha = l_shift_to_interior(u.data(), lay.l);
  for (std::size_t b = 0; b < lay.q_sizes.size(); ++b) {
    alpha = std::max(alpha, q_shift_to_interior(
                                u.segment(lay.q_offsets[b], lay.q_sizes[b])));
  }
  for (std::size_t b = 0; b < lay.s_orders.size(); ++b) {
    alpha = std::max(alpha,
                     s_shift_to_interior(
                         u.segment(lay.s_offsets[b], svec_size(lay.s_orders[b])),
                         lay.s_orders[b]));
  }
  return alpha;
}

/// Combined (centering + corrector) lambda-space right-hand side per block:
/// r_l = -lambda - lambda^{-1} o\ (dlam_s o dlam_z) + sigma*mu*lambda^{-1},
/// where o\ is the Jordan division L(lambda)^{-1} (NOT the element product
/// with lambda^{-1} -- the two differ by Jordan non-associativity).
[[nodiscard]] Eigen::VectorXd combined_lambda_rhs(
    const ConeLayout& lay,
    const Eigen::VectorXd& lam,
    const Eigen::VectorXd& dlam_s,
    const Eigen::VectorXd& dlam_z,
    double sigma_mu) {
  Eigen::VectorXd out(lam.size());
  // Orthant rows (scalar Jordan algebra: division is elementwise).
  for (int i = 0; i < lay.l; ++i) {
    out[i] = -lam[i] - dlam_s[i] * dlam_z[i] / lam[i] + sigma_mu / lam[i];
  }
  // SOC blocks (size 1 reduces to the scalar formula).
  for (std::size_t b = 0; b < lay.q_sizes.size(); ++b) {
    const int off = lay.q_offsets[b];
    const int k = lay.q_sizes[b];
    if (k == 1) {
      out[off] = -lam[off] - dlam_s[off] * dlam_z[off] / lam[off] +
                 sigma_mu / lam[off];
      continue;
    }
    const auto lv = lam.segment(off, k);
    const auto dsv = dlam_s.segment(off, k);
    const auto dzv = dlam_z.segment(off, k);
    const Eigen::VectorXd prod = q_jordan_product(dsv, dzv);
    const Eigen::VectorXd linv = q_jordan_inverse(lv);  // = L(lam)^{-1} e
    out.segment(off, k) = -lv - q_jordan_solve(lv, prod) + sigma_mu * linv;
  }
  // SDP blocks: lambda = diag(sigma); Jordan ops in matrix space.
  for (std::size_t b = 0; b < lay.s_orders.size(); ++b) {
    const int off = lay.s_offsets[b];
    const int p = lay.s_orders[b];
    const int mp = svec_size(p);
    Eigen::VectorXd sigma_diag = Eigen::VectorXd::Zero(p);
    for (int j = 0; j < p; ++j) {
      sigma_diag[j] = lam[off + svec_index(j, j, p)];
    }
    const Eigen::MatrixXd ds = smat(dlam_s.segment(off, mp), p);
    const Eigen::MatrixXd dz = smat(dlam_z.segment(off, mp), p);
    const Eigen::MatrixXd prod = 0.5 * (ds * dz + dz * ds);
    // r_l = -L - L o\ prod + sigma_mu * L^{-1}, with the Jordan division
    // (L o\ X)(i,j) = 2 * X(i,j) / (sigma_i + sigma_j) and L = diag(sigma).
    Eigen::MatrixXd rl(p, p);
    for (int j = 0; j < p; ++j) {
      for (int i = 0; i < p; ++i) {
        const double inv_pair = 2.0 / (sigma_diag[i] + sigma_diag[j]);
        rl(i, j) = -prod(i, j) * inv_pair;
        if (i == j) {
          rl(i, j) += -sigma_diag[i] + sigma_mu / sigma_diag[i];
        }
      }
    }
    out.segment(off, mp) = svec(rl);
  }
  return out;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// ConicIPMSolver
// ═══════════════════════════════════════════════════════════════════════════

ConicIPMSolver::ConicIPMSolver(ConicIPMOptions options)
    : options_(options) {}

ConicIPMResult ConicIPMSolver::solve(const ConicModel& model) const {
  const auto t0 = std::chrono::steady_clock::now();
  const OmpThreadsGuard threads_guard(options_.num_threads);

  ConicIPMResult res;
  const int n = static_cast<int>(model.c.size());
  const int m = model.dims.total();
  const int meq = static_cast<int>(model.A.rows());
  const ConeLayout lay = ConeLayout::build(model.dims);
  const double sense_sign = (model.sense == Sense::Maximize) ? -1.0 : 1.0;

  if (n <= 0 || m <= 0 || lay.degree <= 0 ||
      static_cast<int>(model.G.rows()) != m) {
    res.status = "unknown";
    return res;
  }

  if (options_.chordal_decomposition && !lay.s_orders.empty()) {
    ChordalDecompositionOptions chordal_options;
    chordal_options.enabled = true;
    chordal_options.min_block_order = options_.chordal_min_order;
    chordal_options.max_clique_ratio = options_.chordal_max_clique_ratio;
    chordal_options.max_expansion_ratio =
        options_.chordal_max_expansion_ratio;
    ConicModel transformed;
    ChordalDecompositionMap chordal_map;
    if (chordal_decompose(model, chordal_options, transformed, chordal_map)) {
      ConicIPMOptions inner_options = options_;
      inner_options.chordal_decomposition = false;
      ConicIPMResult inner = ConicIPMSolver(inner_options).solve(transformed);
      if (inner.x.size() >= n &&
          inner.y.size() >= chordal_map.original_num_equalities &&
          inner.z.size() == transformed.dims.total()) {
        chordal_recover(model, chordal_map, inner.x, inner.y, inner.z, res.x,
                        res.y, res.s, res.z);
        const Eigen::VectorXd c_min = sense_sign * model.c;
        const Eigen::VectorXd rx = model.A * res.x - model.b;
        const Eigen::VectorXd rs = model.G * res.x + res.s - model.h;
        const Eigen::VectorXd rc = model.G.transpose() * res.z +
                                   model.A.transpose() * res.y + c_min;
        const double norm_b1 = 1.0 + model.b.norm();
        const double norm_h1 = 1.0 + model.h.norm();
        const double norm_c1 = 1.0 + c_min.norm();
        const double pobj = c_min.dot(res.x);
        const double dobj = -model.h.dot(res.z) - model.b.dot(res.y);
        res.status = inner.status;
        res.primal_objective = sense_sign * pobj;
        res.dual_objective = sense_sign * dobj;
        res.gap = res.s.dot(res.z);
        if (pobj < 0.0) {
          res.relative_gap = res.gap / (-pobj);
        } else if (dobj > 0.0) {
          res.relative_gap = res.gap / dobj;
        } else {
          res.relative_gap = kInf;
        }
        res.primal_infeasibility =
            std::max(rx.norm() / norm_b1, rs.norm() / norm_h1);
        res.dual_infeasibility = rc.norm() / norm_c1;
        res.iterations = inner.iterations;
        res.chordal_decomposition_used = true;
        res.chordal_clique_count = chordal_map.clique_count;
        res.chordal_max_clique_order = chordal_map.max_clique_order;
        const bool gap_ok = res.gap <= options_.abstol ||
                            res.relative_gap <= options_.reltol;
        if (res.status == "optimal" &&
            (res.primal_infeasibility > 10.0 * options_.feastol ||
             res.dual_infeasibility > 10.0 * options_.feastol || !gap_ok)) {
          res.status = "unknown";
        }
        res.runtime_sec = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - t0)
                              .count();
        if (options_.verbose) {
          fmt::print("chordal: {} cliques, max order {}\n",
                     res.chordal_clique_count,
                     res.chordal_max_clique_order);
        }
        return res;
      }
      ConicIPMOptions fallback_options = options_;
      fallback_options.chordal_decomposition = false;
      return ConicIPMSolver(fallback_options).solve(model);
    }
  }

  const Eigen::VectorXd c = sense_sign * model.c;
  const Eigen::VectorXd& h = model.h;
  const Eigen::VectorXd& b = model.b;

  // ── Starting point: x = 0, y = 0, s = h (shifted interior), z = e ─────
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(meq);
  Eigen::VectorXd s = h;
  if (!cone_interior(lay, s)) {
    const double alpha = cone_shift(lay, s);
    s += (1.0 + alpha) * lay.e;
  }
  Eigen::VectorXd z = lay.e;

  KktBackend kkt;
  if (!kkt.init(model, lay)) {
    res.status = "unknown";
    res.x = x;
    res.y = y;
    res.s = s;
    res.z = z;
    return res;
  }

  const bool refine = options_.refinement > 0 &&
                      (!lay.q_sizes.empty() || !lay.s_orders.empty());
  const double norm_b1 = 1.0 + b.norm();
  const double norm_h1 = 1.0 + h.norm();
  const double norm_c1 = 1.0 + c.norm();

  if (options_.verbose) {
    fmt::print("{:>4} {:>12} {:>12} {:>10} {:>9} {:>9} {:>7} {:>7} {:>7}\n",
               "iter", "pobj", "dobj", "gap", "pres", "dres", "sigma",
               "a_pri", "a_dual");
  }

  ConeNtScaling scaling;
  double sigma = 1.0;
  double alpha_p = 0.0;
  double alpha_d = 0.0;

  for (int iter = 0;; ++iter) {
    // ── Residuals, gap, termination ──────────────────────────────────────
    const Eigen::VectorXd rx = model.A * x - b;
    const Eigen::VectorXd rs = model.G * x + s - h;
    const Eigen::VectorXd rc =
        model.G.transpose() * z + model.A.transpose() * y + c;

    const double gap = s.dot(z);
    const double mu = gap / lay.degree;
    const double pres =
        std::max(rx.norm() / norm_b1, rs.norm() / norm_h1);
    const double dres = rc.norm() / norm_c1;
    const double pobj = c.dot(x);
    const double dobj = -h.dot(z) - b.dot(y);
    double relgap = kInf;
    if (pobj < 0.0) {
      relgap = gap / (-pobj);
    } else if (dobj > 0.0) {
      relgap = gap / dobj;
    }

    // Report scalar progress now; the potentially very large iterate vectors
    // are copied once after the loop instead of once per iteration.
    res.primal_objective = sense_sign * pobj;
    res.dual_objective = sense_sign * dobj;
    res.gap = gap;
    res.relative_gap = relgap;
    res.primal_infeasibility = pres;
    res.dual_infeasibility = dres;
    res.iterations = iter;

    if (pres <= options_.feastol && dres <= options_.feastol &&
        (gap <= options_.abstol || relgap <= options_.reltol)) {
      res.status = "optimal";
      break;
    }
    const double g_dual = h.dot(z) + b.dot(y);
    if (g_dual < 0.0) {
      const Eigen::VectorXd cert = rc - c;
      const double nrm = (cert.size() > 0) ? cert.cwiseAbs().maxCoeff() : 0.0;
      if (nrm / (-g_dual) <= options_.feastol) {
        res.status = "primal infeasible";
        break;
      }
    }
    const double ctx = c.dot(x);
    if (ctx < 0.0) {
      const Eigen::VectorXd ax = rx + b;
      const Eigen::VectorXd gxs = rs + h;
      const double nrm = std::max(
          (ax.size() > 0) ? ax.cwiseAbs().maxCoeff() : 0.0,
          gxs.cwiseAbs().maxCoeff());
      if (nrm / (-ctx) <= options_.feastol) {
        res.status = "dual infeasible";
        break;
      }
    }
    if (iter >= options_.max_iterations) {
      res.status = "unknown";
      break;
    }

    // ── Nesterov-Todd scaling ────────────────────────────────────────────
    if (!scaling.compute(lay, s, z)) {
      res.status = "unknown";
      break;
    }
    const Eigen::VectorXd& lambda = scaling.lambda();

    // ── Factor K for this scaling ────────────────────────────────────────
    if (!kkt.factor(scaling)) {
      res.status = "unknown";
      break;
    }

    // Direction solve (used for both the affine and the combined step):
    //   t  = -rs - W r_l
    //   bx = -rc + G' H^{-1} t,  by = -rx
    //   K [dx; dy] = [bx; by]
    //   dz = H^{-1}(G dx - t),  ds = -rs - G dx
    auto solve_direction = [&](const Eigen::VectorXd& r_c,
                               const Eigen::VectorXd& r_x,
                               const Eigen::VectorXd& r_s,
                               const Eigen::VectorXd& r_l, Eigen::VectorXd& dx,
                               Eigen::VectorXd& dy, Eigen::VectorXd& dz,
                               Eigen::VectorXd& ds) {
      const Eigen::VectorXd t = -r_s - scaled_w(scaling, r_l);
      const Eigen::VectorXd u = scaled_h_inv(scaling, t);
      Eigen::VectorXd rhs(kkt.sys.dim);
      rhs.head(n) = -r_c + model.G.transpose() * u;
      if (meq > 0) rhs.tail(meq) = -r_x;
      Eigen::VectorXd sol;
      if (!kkt.solve(rhs, sol)) return false;
      dx = sol.head(n);
      dy = sol.tail(meq);
      const Eigen::VectorXd gdx = model.G * dx;
      dz = scaled_h_inv(scaling, gdx - t);
      ds = -r_s - gdx;
      if (refine) {
        // Iterative refinement on the 3x3 KKT residual.
        for (int ref = 0; ref < options_.refinement; ++ref) {
          Eigen::VectorXd hz(dz.size());
          scaling.apply_h(dz, hz);
          const Eigen::VectorXd f1 =
              model.A.transpose() * dy + model.G.transpose() * dz + r_c;
          const Eigen::VectorXd f2 = model.A * dx + r_x;
          const Eigen::VectorXd f3 = model.G * dx - hz + r_s +
                                     scaled_w(scaling, r_l);
          const Eigen::VectorXd t3 = -f3;
          const Eigen::VectorXd u3 = scaled_h_inv(scaling, t3);
          Eigen::VectorXd rhs3(kkt.sys.dim);
          rhs3.head(n) = -f1 + model.G.transpose() * u3;
          if (meq > 0) rhs3.tail(meq) = -f2;
          Eigen::VectorXd sol3;
          if (!kkt.solve(rhs3, sol3)) return false;
          const Eigen::VectorXd cdx = sol3.head(n);
          dx += cdx;
          dy += sol3.tail(meq);
          const Eigen::VectorXd gcdx = model.G * cdx;
          dz += scaled_h_inv(scaling, gcdx - t3);
          ds -= gcdx;  // keep G dx + ds = -rs exact
        }
      }
      return true;
    };

    // ── Affine (predictor) direction: r_l = -lambda ──────────────────────
    Eigen::VectorXd dx, dy, dz, ds;
    {
      const Eigen::VectorXd r_l = -lambda;
      if (!solve_direction(rc, rx, rs, r_l, dx, dy, dz, ds)) {
        res.status = "unknown";
        break;
      }
    }
    Eigen::VectorXd dlam_s(lambda.size()), dlam_z(lambda.size());
    scaling.apply_w_inv(ds, dlam_s);
    scaling.apply_wt(dz, dlam_z);
    const double a_aff =
        std::min(1.0, cone_max_step(lay, lambda, dlam_s));
    const double b_aff =
        std::min(1.0, cone_max_step(lay, lambda, dlam_z));
    const double mu_aff =
        (s + a_aff * ds).dot(z + b_aff * dz) / lay.degree;
    sigma = (mu > 0.0 && std::isfinite(mu_aff))
                ? std::clamp(std::pow(mu_aff / mu, 3.0), 0.0, 1.0)
                : 1.0;

    // ── Combined (corrector + centering) direction ───────────────────────
    {
      const Eigen::VectorXd r_l =
          combined_lambda_rhs(lay, lambda, dlam_s, dlam_z, sigma * mu);
      if (!solve_direction(rc, rx, rs, r_l, dx, dy, dz, ds)) {
        res.status = "unknown";
        break;
      }
    }
    scaling.apply_w_inv(ds, dlam_s);
    scaling.apply_wt(dz, dlam_z);
    alpha_p = std::min(1.0, kStepDamp * cone_max_step(lay, lambda, dlam_s));
    alpha_d = std::min(1.0, kStepDamp * cone_max_step(lay, lambda, dlam_z));

    x += alpha_p * dx;
    s += alpha_p * ds;
    y += alpha_d * dy;
    z += alpha_d * dz;

    if (options_.verbose) {
      fmt::print("{:4d} {:12.5e} {:12.5e} {:10.3e} {:9.2e} {:9.2e} {:7.3f} "
                 "{:7.4f} {:7.4f}\n",
                 iter, pobj, dobj, gap, pres, dres, sigma, alpha_p, alpha_d);
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  res.x = std::move(x);
  res.y = std::move(y);
  res.s = std::move(s);
  res.z = std::move(z);
  res.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  return res;
}

}  // namespace mipsolvers::engine
