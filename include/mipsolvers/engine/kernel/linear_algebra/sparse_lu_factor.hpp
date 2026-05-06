#pragma once
// ═══════════════════════════════════════════════════════════════════════════
// SparseLUFactor: Native sparse LU factorisation with Forrest-Tomlin update
// for the dual simplex kernel.
//
// Extracted from UMFPACK P*B*Q = L*U, then updated in-place via FT column
// replacement.  Eliminates the need for product-form eta vectors.
// ═══════════════════════════════════════════════════════════════════════════

#include <cmath>
#include <cstring>
#include <vector>

#include <Eigen/Sparse>

namespace mipsolvers::engine {

struct SparseLUFactor {
  bool valid = false;
  int m = 0;

  // ── L factor (unit lower triangular, column-wise CCS) ──
  std::vector<int> Lc_start;     // size m+1
  std::vector<int> Lc_index;     // row indices (i > j)
  std::vector<double> Lc_value;

  // ── L row-wise (for BTRAN backward L^T solve) ──
  std::vector<int> Lr_start;     // size m+1
  std::vector<int> Lr_index;     // col indices (j < i)
  std::vector<double> Lr_value;

  // ── U factor (upper triangular, column-wise CCS) ──
  std::vector<int> Uc_start;     // size m+1
  std::vector<int> Uc_index;     // row indices (i < j)
  std::vector<double> Uc_value;
  std::vector<double> Uc_diag;   // U[j][j], size m

  // ── U row-wise (for BTRAN forward U^T solve) ──
  std::vector<int> Ur_start;     // size m+1
  std::vector<int> Ur_index;     // col indices (j > i)
  std::vector<double> Ur_value;

  // ── Permutations: P*B*Q = L*U ──
  // P[k]=i means original row i maps to LU position k.
  std::vector<int> P, Q, Pinv, Qinv;

  // ── Row scaling ──
  std::vector<double> Rs;
  bool do_recip = false;

  // ── L identity flag (skip L-solve when L has no off-diag entries) ──
  bool l_is_identity = false;

  // ── Update tracking ──
  int n_updates = 0;
  double max_growth = 1.0;
  double min_pivot = 1e30;
  int fresh_U_nnz = 0;              // nnz(U) right after factorisation

  // ── Forrest-Tomlin update storage ──
  // Each FT update produces a column-cyclic shift followed by Hessenberg
  // elimination.  The elimination multipliers must be replayed during
  // FTRAN / BTRAN / compute_spike.
  struct FTEntry {
    int col_pos;              // q = Qinv of the leaving row at update time
    std::vector<double> mult; // mult[k - col_pos] for k = col_pos..m-2
    // Parallel to mult[]: if is_swap[k] is non-zero, the step at k is a
    // row-swap operation (swap y[col_pos+k] ↔ y[col_pos+k+1] during replay)
    // rather than a Hessenberg elimination multiplier.  Row swaps are
    // inserted when h_diag ≈ 0 but h_sub ≠ 0 during Hessenberg elimination,
    // effectively adding a permutation matrix S between L and U in the
    // factorisation:
    //   B = P^T * Rs^{-1} * L * (S_1 * E_1) * (S_2 * E_2) * ... * U * Q^T
    // where each FT update contributes one S*E pair (possibly with S or E
    // being identity).  See update_ft for the derivation.
    std::vector<uint8_t> is_swap;
  };
  std::vector<FTEntry> ft_entries;

  // ── Solve workspace ──
  mutable std::vector<double> work;
  mutable std::vector<char> mark;
  mutable std::vector<int> topo;
  mutable std::vector<int> stk_node;
  mutable std::vector<int> stk_edge;
  mutable std::vector<int> seeds;
  // P11 (2026-04-23): hyper-sparse triangular solves.  `pattern_buf` holds
  // the union of (L-solve topo | FT-touched positions) used as seed set for
  // the U back-substitution DFS reachability.  Same for BTRAN's L^T solve.
  mutable std::vector<int> pattern_buf;

  void alloc_workspace() {
    work.assign(m, 0.0);
    mark.assign(m, 0);
    topo.resize(m);
    stk_node.resize(m);
    stk_edge.resize(m);
    seeds.resize(m);
    pattern_buf.resize(m + 8);  // L-topo (<=m) + small FT margin
  }

  // ════════════════════════════════════════════════════════════════════════
  // EXTRACTION from UMFPACK
  // ════════════════════════════════════════════════════════════════════════
#ifdef HACDCPF_HAVE_UMFPACK
  bool extract(void* Numeric, int m_in);
#endif

  // ════════════════════════════════════════════════════════════════════════
  // DFS reachability for sparse triangular solves
  // ════════════════════════════════════════════════════════════════════════
  void dfs_reach(const int* seed, int n_seeds,
                 const int* col_start, const int* col_index,
                 int& topo_count) const {
    topo_count = 0;
    for (int s = 0; s < n_seeds; ++s) {
      const int root = seed[s];
      if (mark[root]) continue;
      int top = 0;
      stk_node[top] = root;
      stk_edge[top] = col_start[root];
      while (top >= 0) {
        const int node = stk_node[top];
        const int end = col_start[node + 1];
        bool pushed = false;
        for (int& p = stk_edge[top]; p < end; ++p) {
          const int child = col_index[p];
          if (!mark[child]) {
            ++top;
            stk_node[top] = child;
            stk_edge[top] = col_start[child];
            pushed = true;
            break;
          }
        }
        if (!pushed) {
          mark[node] = 1;
          topo[topo_count++] = node;
          --top;
        }
      }
    }
    // Reset marks.
    for (int t = 0; t < topo_count; ++t) mark[topo[t]] = 0;
  }

  // ════════════════════════════════════════════════════════════════════════
  // FTRAN: solve B*x = rhs
  // If `out_nz` is provided, the solver appends nonzero indices of the
  // result (without duplicates).  This lets callers skip an O(m) post-scan
  // when the hyper-sparse path was used.
  // ════════════════════════════════════════════════════════════════════════
  void ftran(const double* rhs, double* result,
             std::vector<int>* out_nz = nullptr) const;

  // ════════════════════════════════════════════════════════════════════════
  // BTRAN: solve B^T*y = rhs
  // ════════════════════════════════════════════════════════════════════════
  void btran(const double* rhs, double* result,
             std::vector<int>* out_nz = nullptr) const;

  // Sparse BTRAN with pre-computed nonzero indices.
  void btran_sparse(const int* rhs_nz_idx, const double* rhs_nz_val,
                    int rhs_nnz, double* result,
                    int* out_nz_idx, int& out_nnz) const;

  // ════════════════════════════════════════════════════════════════════════
  // FORREST-TOMLIN UPDATE
  // ════════════════════════════════════════════════════════════════════════
  //
  // Replace basis column at pivot_row with new column `spike` (after L-solve).
  // The spike is L^{-1} * P * a_new.
  //
  // Returns false if the update would be numerically unstable (trigger refac).
  bool update_ft(int pivot_row, const double* spike_data, int spike_m,
                 int entering_orig = -1);

  // ════════════════════════════════════════════════════════════════════════
  // SPIKE COMPUTATION (L-solve only, for FT update)
  // ════════════════════════════════════════════════════════════════════════
  // Compute s = L^{-1} * P * a, where a is the new entering column.
  // This is the first half of FTRAN, needed for the FT update.
  void compute_spike(const double* a_col, double* spike) const;

  // ════════════════════════════════════════════════════════════════════════
  // Refactorisation check
  // ════════════════════════════════════════════════════════════════════════
  bool needs_refactorise() const {
    // P9.1 (2026-04-22, revised after chain-broken bug fix):
    // Raised max_updates ceiling toward HiGHS-class values (60--200).
    // The prior conservative cap (max_updates ~ 10--30) was a workaround
    // for a correctness bug in the fallback path, not a numerical
    // necessity.  Drift remains bounded by min_pivot / max_growth /
    // fill_ratio guards below.
    const int max_updates = std::clamp(m / 50, 60, 200);
    if (n_updates >= max_updates) return true;
    if (min_pivot < 1e-9) return true;
    if (max_growth > 1e5) return true;
    // Check fill-in growth in U.
    int cur_U_nnz = 0;
    for (int j = 0; j < m; ++j)
      cur_U_nnz += Uc_start[j + 1] - Uc_start[j] + 1; // +1 for diag
    if (fresh_U_nnz > 0 && cur_U_nnz > 3 * fresh_U_nnz) return true;
    return false;
  }

  void reset_update_tracking() {
    n_updates = 0;
    max_growth = 1.0;
    min_pivot = 1e30;
    fresh_U_nnz = 0;
    for (int j = 0; j < m; ++j)
      fresh_U_nnz += Uc_start[j + 1] - Uc_start[j] + 1;
    ft_entries.clear();
  }
};

}  // namespace mipsolvers::engine
