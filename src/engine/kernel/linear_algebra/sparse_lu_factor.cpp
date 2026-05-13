// ═══════════════════════════════════════════════════════════════════════════
// SparseLUFactor implementation: UMFPACK extraction, FTRAN, BTRAN, and
// Forrest-Tomlin column replacement update.
// ═══════════════════════════════════════════════════════════════════════════

#include "mipsolvers/engine/kernel/linear_algebra/sparse_lu_factor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <numeric>

#ifdef HACDCPF_HAVE_UMFPACK
extern "C" {
#include <umfpack.h>
}
#endif

namespace mipsolvers::engine {

// ═══════════════════════════════════════════════════════════════════════════
// P11 (2026-04-23) hyper-sparse triangular solve gating.
//
// FTRAN step 4 (U back-substitution) and BTRAN step 4 (L^T back-substitution)
// historically iterated j = m-1 down to 0 densely, producing O(m) work per
// solve even when the result vector has only a few nonzeros.  HiGHS avoids
// this via nnz-driven reverse-topological iteration.  We do the same, guarded
// by a density check: if the pre-solve pattern exceeds `kHyperSparseDensity`
// fraction of m, the dense fallback is preserved (safer on near-dense bases).
//
// Env overrides:
//   HACDCPF_HYPER_SPARSE=0          disable entirely (debug)
//   HACDCPF_HYPER_SPARSE_DENSITY=x  override density gate (default 0.95)
// ═══════════════════════════════════════════════════════════════════════════
namespace {
inline bool hyper_sparse_enabled() {
  static const bool v = [] {
    const char* e = std::getenv("HACDCPF_HYPER_SPARSE");
    return !e || e[0] != '0';
  }();
  return v;
}
inline double hyper_sparse_density() {
  static const double v = [] {
    const char* e = std::getenv("HACDCPF_HYPER_SPARSE_DENSITY");
    if (!e) return 0.95;
    double d = std::atof(e);
    return (d > 0.0 && d < 1.0) ? d : 0.95;
  }();
  return v;
}
}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// UMFPACK EXTRACTION
// ═══════════════════════════════════════════════════════════════════════════
#ifdef HACDCPF_HAVE_UMFPACK
bool SparseLUFactor::extract(void* Numeric, int m_in) {
  m = m_in;
  valid = false;
  ft_entries.clear();

  int lnz, unz, n_row, n_col, nz_udiag;
  if (umfpack_di_get_lunz(&lnz, &unz, &n_row, &n_col, &nz_udiag, Numeric) != UMFPACK_OK)
    return false;
  if (n_row != m || n_col != m) return false;

  // Temporary extraction buffers.
  std::vector<int> Lp(m + 1), Lj(lnz), Up(m + 1), Ui(unz);
  std::vector<double> Lx(lnz), Ux(unz);
  P.resize(m); Q.resize(m); Rs.resize(m);
  int do_recip_int;

  if (umfpack_di_get_numeric(Lp.data(), Lj.data(), Lx.data(),
                             Up.data(), Ui.data(), Ux.data(),
                             P.data(), Q.data(), nullptr, &do_recip_int,
                             Rs.data(), Numeric) != UMFPACK_OK)
    return false;
  do_recip = (do_recip_int != 0);

  // Inverse permutations.
  Pinv.resize(m); Qinv.resize(m);
  for (int k = 0; k < m; ++k) { Pinv[P[k]] = k; Qinv[Q[k]] = k; }

  // ── Build L column-wise (UMFPACK gives L in row-major Lp,Lj,Lx) ──
  // UMFPACK L format: Lp[row], Lj[k], Lx[k] where Lj[k] < row (lower tri).
  // We want L in column-major: for each col j, the entries L[i][j] with i > j.
  l_is_identity = true;
  std::vector<int> Lc_count(m, 0);
  for (int i = 0; i < m; ++i) {
    for (int k = Lp[i]; k < Lp[i + 1]; ++k) {
      if (Lj[k] < i) { Lc_count[Lj[k]]++; l_is_identity = false; }
    }
  }
  Lc_start.resize(m + 1);
  Lc_start[0] = 0;
  for (int j = 0; j < m; ++j) Lc_start[j + 1] = Lc_start[j] + Lc_count[j];
  const int Lc_total = Lc_start[m];
  Lc_index.resize(Lc_total);
  Lc_value.resize(Lc_total);
  std::fill(Lc_count.begin(), Lc_count.end(), 0);
  for (int i = 0; i < m; ++i) {
    for (int k = Lp[i]; k < Lp[i + 1]; ++k) {
      const int j = Lj[k];
      if (j < i) {
        const int pos = Lc_start[j] + Lc_count[j]++;
        Lc_index[pos] = i;
        Lc_value[pos] = Lx[k];
      }
    }
  }

  // ── Build L row-wise (for BTRAN L^T backward solve) ──
  Lr_start.resize(m + 1);
  Lr_start[0] = 0;
  for (int i = 0; i < m; ++i) {
    int cnt = 0;
    for (int k = Lp[i]; k < Lp[i + 1]; ++k)
      if (Lj[k] < i) cnt++;
    Lr_start[i + 1] = Lr_start[i] + cnt;
  }
  const int Lr_total = Lr_start[m];
  Lr_index.resize(Lr_total);
  Lr_value.resize(Lr_total);
  for (int i = 0; i < m; ++i) {
    int pos = Lr_start[i];
    for (int k = Lp[i]; k < Lp[i + 1]; ++k) {
      if (Lj[k] < i) {
        Lr_index[pos] = Lj[k];
        Lr_value[pos] = Lx[k];
        pos++;
      }
    }
  }

  // ── Build U column-wise from UMFPACK column-major (Up,Ui,Ux) ──
  Uc_diag.assign(m, 0.0);
  std::vector<int> Uc_count(m, 0);
  for (int j = 0; j < m; ++j) {
    for (int k = Up[j]; k < Up[j + 1]; ++k) {
      if (Ui[k] == j) Uc_diag[j] = Ux[k];
      else if (Ui[k] < j) Uc_count[j]++;
    }
  }
  Uc_start.resize(m + 1);
  Uc_start[0] = 0;
  for (int j = 0; j < m; ++j) Uc_start[j + 1] = Uc_start[j] + Uc_count[j];
  const int Uc_total = Uc_start[m];
  Uc_index.resize(Uc_total);
  Uc_value.resize(Uc_total);
  std::fill(Uc_count.begin(), Uc_count.end(), 0);
  for (int j = 0; j < m; ++j) {
    for (int k = Up[j]; k < Up[j + 1]; ++k) {
      if (Ui[k] < j) {
        const int pos = Uc_start[j] + Uc_count[j]++;
        Uc_index[pos] = Ui[k];
        Uc_value[pos] = Ux[k];
      }
    }
  }

  // ── Build U row-wise (for BTRAN forward U^T solve) ──
  std::vector<int> Ur_count(m, 0);
  for (int j = 0; j < m; ++j) {
    for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k)
      Ur_count[Uc_index[k]]++;
  }
  Ur_start.resize(m + 1);
  Ur_start[0] = 0;
  for (int i = 0; i < m; ++i) Ur_start[i + 1] = Ur_start[i] + Ur_count[i];
  const int Ur_total = Ur_start[m];
  Ur_index.resize(Ur_total);
  Ur_value.resize(Ur_total);
  std::fill(Ur_count.begin(), Ur_count.end(), 0);
  for (int j = 0; j < m; ++j) {
    for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k) {
      const int i = Uc_index[k];
      const int pos = Ur_start[i] + Ur_count[i]++;
      Ur_index[pos] = j;
      Ur_value[pos] = Uc_value[k];
    }
  }

  alloc_workspace();
  valid = true;
  n_updates = 0;
  max_growth = 1.0;
  min_pivot = 1e30;
  for (int j = 0; j < m; ++j) {
    const double d = std::abs(Uc_diag[j]);
    if (d < min_pivot && d > 0.0) min_pivot = d;
  }
  fresh_U_nnz = 0;
  for (int j = 0; j < m; ++j)
    fresh_U_nnz += Uc_start[j + 1] - Uc_start[j] + 1;
  ft_entries.clear();
  return true;
}
#endif  // HACDCPF_HAVE_UMFPACK

// ═══════════════════════════════════════════════════════════════════════════
// COMPUTE SPIKE: s = L^{-1} * P * a  (L-solve only)
// ═══════════════════════════════════════════════════════════════════════════
void SparseLUFactor::compute_spike(const double* a_col, double* spike) const {
  constexpr double tol = 1e-20;

  // Step 1: spike = P * scaled(a_col), collect seeds.
  std::memset(spike, 0, m * sizeof(double));
  int n_seeds = 0;
  for (int orig_row = 0; orig_row < m; ++orig_row) {
    if (a_col[orig_row] == 0.0) continue;
    double v = a_col[orig_row];
    if (do_recip) v *= Rs[orig_row]; else v /= Rs[orig_row];
    const int k = Pinv[orig_row];
    spike[k] = v;
    seeds[n_seeds++] = k;
  }

  if (!l_is_identity) {
    // Step 2: DFS reachability on L column graph.
    int topo_count = 0;
    dfs_reach(seeds.data(), n_seeds, Lc_start.data(), Lc_index.data(), topo_count);

    // Step 3: L forward solve in topological order.
    for (int t = topo_count - 1; t >= 0; --t) {
      const int j = topo[t];
      const double sj = spike[j];
      if (std::abs(sj) > tol) {
        for (int k = Lc_start[j]; k < Lc_start[j + 1]; ++k)
          spike[Lc_index[k]] -= Lc_value[k] * sj;
      }
    }
  }

  // Step 4: Apply accumulated FT forward replay: for each step, apply
  // the row-swap (S_i) first, then the Hessenberg elimination (E_i).
  // Replay in forward order (oldest first).
  for (const auto& ft : ft_entries) {
    const int q = ft.col_pos;
    const int n_steps = static_cast<int>(ft.mult.size());
    for (int k = 0; k < n_steps; ++k) {
      if (k < static_cast<int>(ft.is_swap.size()) && ft.is_swap[k])
        std::swap(spike[q + k], spike[q + k + 1]);
      spike[q + k + 1] -= ft.mult[k] * spike[q + k];
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// FTRAN: solve B*x = rhs
// ═══════════════════════════════════════════════════════════════════════════
void SparseLUFactor::ftran(const double* rhs, double* result,
                           std::vector<int>* out_nz) const {
  constexpr double tol = 1e-20;
  if (out_nz) out_nz->clear();

  // Step 1: work = P * diag(S) * rhs, collect seeds.
  int n_seeds = 0;
  for (int orig_row = 0; orig_row < m; ++orig_row) {
    if (rhs[orig_row] == 0.0) continue;
    double v = rhs[orig_row];
    if (do_recip) v *= Rs[orig_row]; else v /= Rs[orig_row];
    const int k = Pinv[orig_row];
    work[k] = v;
    seeds[n_seeds++] = k;
  }

  // Step 2: L forward solve.  Snapshot L-reach into pattern_buf for P11.
  const bool hs = hyper_sparse_enabled();
  const int hs_cap = hs ? static_cast<int>(hyper_sparse_density() * m) + 1
                        : 0;
  int pat_n = 0;
  bool pat_overflow = false;
  if (!l_is_identity) {
    int topo_count = 0;
    dfs_reach(seeds.data(), n_seeds, Lc_start.data(), Lc_index.data(), topo_count);
    for (int t = topo_count - 1; t >= 0; --t) {
      const int j = topo[t];
      const double wj = work[j];
      if (std::abs(wj) > tol) {
        for (int k = Lc_start[j]; k < Lc_start[j + 1]; ++k)
          work[Lc_index[k]] -= Lc_value[k] * wj;
      }
    }
    if (hs) {
      if (topo_count > hs_cap) {
        pat_overflow = true;
      } else {
        // Copy topo into pattern_buf BEFORE any subsequent dfs_reach call
        // overwrites `topo`.
        std::memcpy(pattern_buf.data(), topo.data(),
                    topo_count * sizeof(int));
        pat_n = topo_count;
      }
    }
  } else if (hs) {
    if (n_seeds > hs_cap) {
      pat_overflow = true;
    } else {
      std::memcpy(pattern_buf.data(), seeds.data(),
                  n_seeds * sizeof(int));
      pat_n = n_seeds;
    }
  }

  // Step 3: FT forward replay: E * work (oldest to newest).
  // P11: track touched positions in pattern_buf for the hyper-sparse U solve.

  for (const auto& ft : ft_entries) {
    const int q = ft.col_pos;
    const int n_steps = static_cast<int>(ft.mult.size());
    for (int k = 0; k < n_steps; ++k) {
      if (k < static_cast<int>(ft.is_swap.size()) && ft.is_swap[k])
        std::swap(work[q + k], work[q + k + 1]);
      work[q + k + 1] -= ft.mult[k] * work[q + k];
      if (hs && !pat_overflow) {
        if (pat_n + 2 > hs_cap) { pat_overflow = true; }
        else {
          pattern_buf[pat_n++] = q + k;
          pattern_buf[pat_n++] = q + k + 1;
        }
      }
    }
  }

  // Step 4: U backward solve.  Hyper-sparse when pattern is sparse enough.
  if (hs && !pat_overflow) {
    int u_topo_count = 0;
    dfs_reach(pattern_buf.data(), pat_n, Uc_start.data(), Uc_index.data(),
              u_topo_count);
    for (int t = u_topo_count - 1; t >= 0; --t) {
      const int j = topo[t];
      if (std::abs(work[j]) > tol) {
        work[j] /= Uc_diag[j];
        const double wj = work[j];
        for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k)
          work[Uc_index[k]] -= Uc_value[k] * wj;
      } else {
        work[j] = 0.0;
      }
    }
    // Step 5 (sparse): result = Q * work, iterate only touched positions.
    std::memset(result, 0, m * sizeof(double));
    for (int t = 0; t < u_topo_count; ++t) {
      const int k = topo[t];
      if (work[k] != 0.0) {
        const int idx = Q[k];
        result[idx] = work[k];
        work[k] = 0.0;
        if (out_nz) out_nz->push_back(idx);
      }
    }
    // Clear any original seeds that weren't touched by U-solve.
    for (int s = 0; s < n_seeds; ++s) work[seeds[s]] = 0.0;
    return;
  }

  // Step 4 (dense fallback).
  for (int j = m - 1; j >= 0; --j) {
    if (work[j] != 0.0 && std::abs(work[j]) > tol) {
      work[j] /= Uc_diag[j];
      const double wj = work[j];
      for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k)
        work[Uc_index[k]] -= Uc_value[k] * wj;
    } else {
      work[j] = 0.0;
    }
  }

  // Step 5: result = Q * work, clear work.
  std::memset(result, 0, m * sizeof(double));
  for (int k = 0; k < m; ++k) {
    if (work[k] != 0.0) {
      const int idx = Q[k];
      result[idx] = work[k];
      work[k] = 0.0;
      if (out_nz) out_nz->push_back(idx);
    }
  }
  // Clear any remaining seeds.
  for (int s = 0; s < n_seeds; ++s) work[seeds[s]] = 0.0;
}

// ═══════════════════════════════════════════════════════════════════════════
// BTRAN: solve B^T*y = rhs
// ═══════════════════════════════════════════════════════════════════════════
void SparseLUFactor::btran(const double* rhs, double* result,
                           std::vector<int>* out_nz) const {
  constexpr double tol = 1e-20;

  // Step 1: work = Q^T * rhs, collect seeds.
  int n_seeds = 0;
  for (int k = 0; k < m; ++k) {
    const double v = rhs[Q[k]];
    if (v != 0.0) {
      work[k] = v;
      seeds[n_seeds++] = k;
    }
  }

  // Step 2: DFS + U^T forward solve.
  int topo_count = 0;
  dfs_reach(seeds.data(), n_seeds, Ur_start.data(), Ur_index.data(), topo_count);
  for (int t = topo_count - 1; t >= 0; --t) {
    const int i = topo[t];
    if (std::abs(work[i]) > tol) {
      work[i] /= Uc_diag[i];
      const double wi = work[i];
      for (int k = Ur_start[i]; k < Ur_start[i + 1]; ++k)
        work[Ur_index[k]] -= Ur_value[k] * wi;
    } else {
      work[i] = 0.0;
    }
  }

  // P11: snapshot U^T-reach before FT/L^T rewrites topo.
  const bool hs = hyper_sparse_enabled();
  const int hs_cap = hs ? static_cast<int>(hyper_sparse_density() * m) + 1
                        : 0;
  int pat_n = 0;
  bool pat_overflow = false;
  auto append_pattern_idx = [&](int idx) {
    if (!hs || pat_overflow || mark[idx]) return;
    if (pat_n >= hs_cap) {
      pat_overflow = true;
      return;
    }
    mark[idx] = 1;
    pattern_buf[pat_n++] = idx;
  };
  if (hs) {
    if (topo_count > hs_cap) {
      pat_overflow = true;
    } else {
      for (int t = 0; t < topo_count; ++t) {
        const int idx = topo[t];
        mark[idx] = 1;
        pattern_buf[pat_n++] = idx;
      }
    }
  }

  // Step 3: FT backward replay: E^T then S^T (newest to oldest).
  // Reverse of forward: first reverse elimination, then reverse swap.
  for (int f = static_cast<int>(ft_entries.size()) - 1; f >= 0; --f) {
    const auto& ft = ft_entries[f];
    const int q = ft.col_pos;
    const int n_steps = static_cast<int>(ft.mult.size());
    for (int k = n_steps - 1; k >= 0; --k) {
      work[q + k] -= ft.mult[k] * work[q + k + 1];
      if (k < static_cast<int>(ft.is_swap.size()) && ft.is_swap[k])
        std::swap(work[q + k], work[q + k + 1]);
      if (hs && !pat_overflow) {
        append_pattern_idx(q + k);
        append_pattern_idx(q + k + 1);
      }
    }
  }

  // `mark[]` was used only for pattern de-duplication above. Clear those bits
  // before dfs_reach(), which uses the same workspace as its visitation map.
  if (hs) {
    for (int t = 0; t < pat_n; ++t) mark[pattern_buf[t]] = 0;
  }

  // Step 4: L^T backward solve.  Hyper-sparse path when pattern is sparse.
  int lt_topo_count = 0;
  bool used_sparse_lt = false;
  if (!l_is_identity) {
    if (hs && !pat_overflow) {
      dfs_reach(pattern_buf.data(), pat_n, Lr_start.data(), Lr_index.data(),
                lt_topo_count);
      for (int t = lt_topo_count - 1; t >= 0; --t) {
        const int i = topo[t];
        if (std::abs(work[i]) > tol) {
          const double wi = work[i];
          for (int k = Lr_start[i]; k < Lr_start[i + 1]; ++k)
            work[Lr_index[k]] -= Lr_value[k] * wi;
        }
      }
      used_sparse_lt = true;
    } else {
      for (int i = m - 1; i >= 0; --i) {
        if (work[i] != 0.0 && std::abs(work[i]) > tol) {
          const double wi = work[i];
          for (int k = Lr_start[i]; k < Lr_start[i + 1]; ++k)
            work[Lr_index[k]] -= Lr_value[k] * wi;
        }
      }
    }
  } else if (hs && !pat_overflow) {
    // L == I: no L^T step, but work's nonzero set = pat_n seeds.  Mark
    // "sparse path" so step 5 can iterate pattern instead of 0..m.
    lt_topo_count = pat_n;
    std::memcpy(topo.data(), pattern_buf.data(), pat_n * sizeof(int));
    used_sparse_lt = true;
  }

  // Step 5: result = inv_scale * P^T * work, clear work.
  std::memset(result, 0, m * sizeof(double));
  if (out_nz) out_nz->clear();
  if (used_sparse_lt) {
    // Sparse gather over lt_topo (stored in `topo`) only.
    if (do_recip) {
      for (int t = 0; t < lt_topo_count; ++t) {
        const int k = topo[t];
        if (work[k] != 0.0) {
          const double v = work[k] * Rs[P[k]];
          result[P[k]] = v;
          if (out_nz && v != 0.0) out_nz->push_back(P[k]);
          work[k] = 0.0;
        }
      }
    } else {
      for (int t = 0; t < lt_topo_count; ++t) {
        const int k = topo[t];
        if (work[k] != 0.0) {
          const double v = work[k] / Rs[P[k]];
          result[P[k]] = v;
          if (out_nz && v != 0.0) out_nz->push_back(P[k]);
          work[k] = 0.0;
        }
      }
    }
    // Clear any residual seeds that weren't visited by L^T reach.
    for (int s = 0; s < n_seeds; ++s) work[seeds[s]] = 0.0;
    return;
  }
  if (do_recip) {
    for (int k = 0; k < m; ++k) {
      if (work[k] != 0.0) {
        const double v = work[k] * Rs[P[k]];
        result[P[k]] = v;
        if (out_nz && v != 0.0) out_nz->push_back(P[k]);
      }
      work[k] = 0.0;
    }
  } else {
    for (int k = 0; k < m; ++k) {
      if (work[k] != 0.0) {
        const double v = work[k] / Rs[P[k]];
        result[P[k]] = v;
        if (out_nz && v != 0.0) out_nz->push_back(P[k]);
      }
      work[k] = 0.0;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// FORREST-TOMLIN UPDATE
//
// When basis column p is replaced with a_new:
// 1. Compute spike s = L^{-1} * P * a_new (caller provides this, after
//    FT forward replay of all prior updates).
// 2. Find position q = Qinv[p] in the permuted order.
// 3. Cyclically shift columns q..m-1 in U so the spike moves to
//    position m-1.  The result is an upper-Hessenberg matrix.
// 4. Eliminate the Hessenberg sub-diagonals using adjacent-row operations.
//    Store the elimination multipliers in an FTEntry for replay.
// 5. Rebuild CCS/CSR and update Q/Qinv.
// ═══════════════════════════════════════════════════════════════════════════
bool SparseLUFactor::update_ft(int pivot_row, const double* spike_data,
                               int spike_m, int entering_orig) {
  // NOTE (Apr 2026 debug): `entering_orig` is accepted for API compatibility
  // but MUST NOT be written into Q/Qinv: Q is UMFPACK's internal permutation
  // of B's m columns (range [0,m)), while the caller typically passes an
  // A-column index which can reach n-1 >= m.  The previous line
  // `Q[m-1] = entering_orig; Qinv[entering_orig] = m-1;` caused a
  // reproducible SIGBUS on any LP with n > m because it wrote past the end
  // of Qinv (size m).  The cyclic column shift is a pure B-space operation;
  // the vacated slot `old_Qq` is recycled at position m-1 so Q stays a valid
  // permutation of [0, m).  External basis-to-A bookkeeping is the caller's
  // responsibility.
  (void)entering_orig;

  static const bool ft_diag = []() {
    const char* e = std::getenv("HACDCPF_P9_FT_DIAG");
    return e && e[0] == '1';
  }();
  static int r_spike_m = 0, r_q_bad = 0, r_n_zero = 0,
             r_new_diag = 0;

  if (spike_m != m) {
    if (ft_diag && r_spike_m++ < 3)
      std::fprintf(stderr, "[FT-FAIL] spike_m=%d m=%d\n", spike_m, m);
    return false;
  }

  const int q = Qinv[pivot_row];
  if (q < 0 || q >= m) {
    if (ft_diag && r_q_bad++ < 3)
      std::fprintf(stderr, "[FT-FAIL] q=%d m=%d pivot_row=%d\n", q, m, pivot_row);
    return false;
  }

  // NOTE (2026-04-22): Previous guard `if (std::abs(spike_data[q]) < 1e-14)
  // return false;` was an incorrect zero-pivot check.  spike_data[q] is
  // NOT the new pivot — after the cyclic column shift it becomes the
  // above-diagonal entry U_new[q][m-1], which is allowed to be zero for
  // sparse bases.  The real pivot is new_diags[n-1] after Hessenberg
  // elimination; that is checked below.  On uc_500g_24t with sparse UC
  // columns (~15 nnz each), spike_data[q] is zero for >80% of pivots,
  // so this check was rejecting nearly every FT attempt (verified via
  // HACDCPF_P9_FT_DIAG: ft_ok=0 over hundreds of refactor cycles).

  const int n = m - q;
  // NOTE (2026-04-22): Previous guard `if (n > 5000) return false;` was a
  // leftover from early-stage development and made FT unusable at large m:
  // most pivots on uc_500g_24t (m=59024) land at q < m - 5000, so FT
  // aborted on >99% of attempts (verified via HACDCPF_P9_FT_DIAG).  The
  // cost of the Hessenberg elimination is O(n * avg_nnz_row), which at
  // n=m for sparse UC bases is well under a full UMFPACK refactorisation
  // (typ. 50-80ms at m=60k); there is no correctness-motivated cap.
  if (n <= 0) {
    if (ft_diag && r_n_zero++ < 3)
      std::fprintf(stderr, "[FT-FAIL] n<=0 q=%d m=%d\n", q, m);
    return false;
  }

  // (entering_orig handling — see note above; no longer stored into Q.)

  // Stability tracking is deferred until new_diags[n-1] (the real new
  // pivot of column m-1 after Hessenberg elimination) is computed below.

  // ═══════════════════════════════════════════════════════════════════════
  // Forrest-Tomlin column replacement with cyclic column shift.
  //
  // After replacing column q of U with the spike, we cyclically shift
  // columns q..m-1 so that the spike moves to position m-1.  The result
  // is an upper-Hessenberg matrix where the sub-diagonals are the old
  // diagonal entries that shifted left.  We then eliminate sub-diags
  // using adjacent-row operations without partial pivoting.
  //
  // Memory: two dense row buffers of size n = m - q.
  // Cost:   O(n * average_nnz_row).
  // ═══════════════════════════════════════════════════════════════════════

  // ── Helper lambda: build shifted row k (k >= q) into buf[0..n-1] ──
  // After the cyclic column shift (old col q+1→q, q+2→q+1, ..., m-1→m-2,
  // spike→m-1), the dense row in local coordinates is:
  //   buf[j_local] = U_old[k][q + j_local + 1]   for j_local = 0..n-2
  //   buf[n-1]     = spike_data[k]                (spike column at m-1)
  //
  // The old diagonal Uc_diag[k] at column k maps to shifted column k-1
  // (local index k-1-q), which is one left of the new diagonal → sub-diag.

  auto build_shifted_row = [&](int k, double* buf) {
    std::memset(buf, 0, n * sizeof(double));

    // (a) Old diagonal Uc_diag[k] → shifted column k-1 → sub-diagonal.
    if (k > q) {
      const int loc = k - 1 - q;
      if (loc >= 0 && loc < n) buf[loc] = Uc_diag[k];
    }

    // (b) Off-diagonal entries from Ur (columns j > k, strict upper triangle).
    //     Old column j → shifted column j-1 → local index j-1-q.
    for (int p = Ur_start[k]; p < Ur_start[k + 1]; ++p) {
      const int old_col = Ur_index[p]; // old_col > k >= q  →  old_col > q
      const int loc = old_col - 1 - q;
      if (loc >= 0 && loc < n) buf[loc] = Ur_value[p];
    }

    // (c) Spike column at local position n-1.
    buf[n - 1] = spike_data[k];
  };

  // ── Rolling Hessenberg elimination ──
  std::vector<double> prev_buf(n);
  std::vector<double> cur_buf(n);

  // Collect results per row: (new diagonal, above-diagonal nonzeros).
  std::vector<double> new_diags(n);
  std::vector<std::vector<std::pair<int, double>>> new_row_entries(n);

  // Store FT elimination multipliers and row-swap flags for replay.
  std::vector<double> ft_mult(n > 1 ? n - 1 : 0, 0.0);
  std::vector<uint8_t> ft_is_swap(n > 1 ? n - 1 : 0, 0);

  build_shifted_row(q, prev_buf.data());

  for (int k = q; k < m - 1; ++k) {
    const int lk = k - q; // local row index of prev_row

    build_shifted_row(k + 1, cur_buf.data());

    // Sub-diagonal at (k+1, k) → local (lk+1, lk) = cur_buf[lk].
    const double h_sub = cur_buf[lk];
    const double h_diag = prev_buf[lk]; // diagonal of row k

    if (std::abs(h_sub) > 1e-20) {
      if (std::abs(h_diag) < 1e-14) {
        // ── Partial pivot: swap rows k and k+1 ──
        // The new diagonal at (k,k) after the cyclic column shift is
        // U_old[k][k+1], which is structurally zero for >90% of pivots
        // on sparse UC-style bases (uc_500g_24t, uc_1000g_24t).  The
        // sub-diagonal h_sub = U_old[k+1][k+1] (the next proper diagonal
        // pivot) is always non-zero in a valid LU factorisation.
        //
        // Solution: swap rows k and k+1 in the working Hessenberg buffers,
        // record the swap in ft_is_swap[lk], then apply elimination on the
        // (now zero) sub-diagonal.  The swap is replayed in FTRAN (before
        // elimination) and BTRAN (after reverse-elimination) so that
        //   B = P^T Rs^{-1} L (S_1 E_1)(S_2 E_2)... U Q^T
        // remains a valid factorisation with each (S_i, E_i) pair.
        ft_is_swap[lk] = 1;
        std::swap(prev_buf, cur_buf);
        // After swap: prev_buf[lk] = old h_sub (non-zero new diagonal),
        //             cur_buf[lk]  = old h_diag ≈ 0 → mult ≈ 0.
        const double h_sub_new = cur_buf[lk];  // old h_diag ≈ 0
        if (std::abs(h_sub_new) > 1e-20) {
          const double mult = h_sub_new / prev_buf[lk];
          ft_mult[lk] = mult;
          cur_buf[lk] = 0.0;
          for (int j = lk + 1; j < n; ++j)
            cur_buf[j] -= mult * prev_buf[j];
        }
        if (ft_diag)
          std::fprintf(stderr, "[FT-SWAP] k=%d q=%d m=%d h_sub=%.2e\n",
                       k, q, m, h_sub);
      } else {
        const double mult = h_sub / h_diag;
        ft_mult[lk] = mult;
        cur_buf[lk] = 0.0;
        for (int j = lk + 1; j < n; ++j)
          cur_buf[j] -= mult * prev_buf[j];
      }
    }

    // Store completed row k (prev_buf): diagonal + above-diagonal entries.
    new_diags[lk] = prev_buf[lk];
    for (int j = lk + 1; j < n; ++j) {
      if (std::abs(prev_buf[j]) > 1e-20)
        new_row_entries[lk].emplace_back(q + j, prev_buf[j]);
    }

    std::swap(prev_buf, cur_buf);
  }

  // Last row (m-1): diagonal only within the sub-block.
  new_diags[n - 1] = prev_buf[n - 1];

  // ── Pivot check on all new diagonals ──
  for (int r = 0; r < n; ++r) {
    if (std::abs(new_diags[r]) < 1e-14) {
      if (ft_diag && r_new_diag++ < 3)
        std::fprintf(stderr, "[FT-FAIL] new_diag~0 r=%d n=%d q=%d m=%d val=%.2e\n",
                     r, n, q, m, new_diags[r]);
      return false;
    }
    if (std::abs(new_diags[r]) < min_pivot) min_pivot = std::abs(new_diags[r]);
  }

  // ── Growth tracking on the newly-introduced column pivot (column m-1) ──
  {
    const double new_pivot = std::abs(new_diags[n - 1]);
    const double old_pivot = std::abs(Uc_diag[q]);
    if (old_pivot > 0.0 && new_pivot > 0.0) {
      const double g = std::max(new_pivot / old_pivot, old_pivot / new_pivot);
      if (g > max_growth) max_growth = g;
    }
  }

  // ── Build column-oriented view from the Hessenberg row results ──
  // hess_col[j_local] = {(global_row, value)} for the above-diagonal entries.
  std::vector<std::vector<std::pair<int, double>>> hess_col(n);
  for (int r = 0; r < n; ++r) {
    for (auto& [col, val] : new_row_entries[r]) {
      const int jl = col - q;
      if (jl >= 0 && jl < n) hess_col[jl].emplace_back(q + r, val);
    }
  }

  // ── Rebuild CCS (Uc_start / Uc_index / Uc_value) ──
  // Columns  0..q-1  : unchanged.
  // Columns  q..m-2  : shifted from old column j+1.
  //   - Rows < q  from old Uc of old-column j+1.
  //   - Rows >= q from hess_col.
  // Column m-1       : spike column.
  //   - Rows < q  from spike_data[i].
  //   - Rows >= q from hess_col[n-1].

  std::vector<int>    nUc_start(m + 1);
  std::vector<int>    nUc_index;
  std::vector<double> nUc_value;
  nUc_index.reserve(Uc_index.size() + n * 4);
  nUc_value.reserve(Uc_value.size() + n * 4);

  for (int j = 0; j < m; ++j) {
    nUc_start[j] = static_cast<int>(nUc_index.size());

    if (j < q) {
      // Copy old column verbatim.
      for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k) {
        nUc_index.push_back(Uc_index[k]);
        nUc_value.push_back(Uc_value[k]);
      }
    } else if (j < m - 1) {
      // Shifted column: old column j+1.
      const int old_col = j + 1;
      for (int k = Uc_start[old_col]; k < Uc_start[old_col + 1]; ++k) {
        if (Uc_index[k] < q) {
          nUc_index.push_back(Uc_index[k]);
          nUc_value.push_back(Uc_value[k]);
        }
      }
      // Rows >= q from Hessenberg result.
      for (auto& [row, val] : hess_col[j - q]) {
        nUc_index.push_back(row);
        nUc_value.push_back(val);
      }
    } else {
      // Column m-1: spike.
      for (int i = 0; i < q; ++i) {
        if (std::abs(spike_data[i]) > 1e-20) {
          nUc_index.push_back(i);
          nUc_value.push_back(spike_data[i]);
        }
      }
      for (auto& [row, val] : hess_col[n - 1]) {
        nUc_index.push_back(row);
        nUc_value.push_back(val);
      }
    }
  }
  nUc_start[m] = static_cast<int>(nUc_index.size());

  Uc_start = std::move(nUc_start);
  Uc_index = std::move(nUc_index);
  Uc_value = std::move(nUc_value);

  // Diagonals.
  for (int r = 0; r < n; ++r) Uc_diag[q + r] = new_diags[r];

  // ── Rebuild Ur (row-wise CSR) from Uc ──
  {
    std::vector<int> cnt(m, 0);
    for (int j = 0; j < m; ++j)
      for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k)
        cnt[Uc_index[k]]++;
    Ur_start.resize(m + 1);
    Ur_start[0] = 0;
    for (int i = 0; i < m; ++i) Ur_start[i + 1] = Ur_start[i] + cnt[i];
    Ur_index.resize(Ur_start[m]);
    Ur_value.resize(Ur_start[m]);
    std::fill(cnt.begin(), cnt.end(), 0);
    for (int j = 0; j < m; ++j)
      for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k) {
        const int i = Uc_index[k];
        const int pos = Ur_start[i] + cnt[i]++;
        Ur_index[pos] = j;
        Ur_value[pos] = Uc_value[k];
      }
  }

  // ── Update Q / Qinv for the cyclic column shift ──
  // Q must remain a permutation of [0, m).  The slot `old_Qq` that the
  // leaving column occupied is recycled at position m-1; the rest of the
  // range [q, m-1) shifts left.  NOTE: we deliberately do NOT store any
  // external (A-column) identity here — see the SIGBUS note at the top of
  // update_ft().
  {
    const int old_Qq = Q[q];
    for (int j = q; j < m - 1; ++j) Q[j] = Q[j + 1];
    Q[m - 1] = old_Qq;  // recycle vacated slot
    for (int j = q; j < m; ++j) Qinv[Q[j]] = j;
  }

  // ── Store FT entry for replay during FTRAN/BTRAN ──
  {
    FTEntry ft;
    ft.col_pos = q;
    ft.mult = std::move(ft_mult);
    ft.is_swap = std::move(ft_is_swap);
    ft_entries.push_back(std::move(ft));
  }

  ++n_updates;
  return true;
}

}  // namespace mipsolvers::engine
