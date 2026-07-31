#pragma once
// ═══════════════════════════════════════════════════════════════════════════
// HFactorBackend
//
// Thin wrapper around the vendored HiGHS HFactor (in
// src/engine/kernel/linear_algebra/highs_factor/) that mirrors the public
// surface of `mipsolvers::engine::SparseLUFactor` so it can be plugged into the
// dual-simplex driver as a drop-in alternative LU backend.
//
// Phase 2 of U.7.118: this header + its implementation must compile and
// provide a usable API but NO call site in the rest of mipsolvers is required
// to use it yet.  Phase 3 wires it into `SparseBasis` behind a new
// `FactorBackendKind::HFactorPort` enum value.
// ═══════════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Sparse>

// HFactor is forward-declared so this header does not pull in HiGHS' include
// tree.  The pimpl owns the actual `HFactor` instance.
class HFactor;

namespace mipsolvers::engine {

class HFactorBackend {
 public:
  HFactorBackend();
  ~HFactorBackend();

  HFactorBackend(const HFactorBackend&) = delete;
  HFactorBackend& operator=(const HFactorBackend&) = delete;
  HFactorBackend(HFactorBackend&&) noexcept;
  HFactorBackend& operator=(HFactorBackend&&) noexcept;

  // ── State ──────────────────────────────────────────────────────────────
  // `valid` mirrors `SparseLUFactor::valid`: true iff the last factorize()
  // returned a full-rank decomposition.  Cleared on update failure.
  bool valid = false;
  // Order m of B (== A.rows()).
  int m = 0;
  // Number of successful FT updates since the last factorize() call.
  int n_updates = 0;
  // Rank deficiency reported by the last factorize().  0 on success.
  int rank_deficiency = 0;

  // Rows that received no pivot in the last failed build (HiGHS-style
  // basis repair input): replace each such row's basic column with the
  // row's logical (slack/artificial) and refactorize.  Valid only right
  // after a factorize() that returned false with rank_deficiency > 0.
  const std::vector<int>& no_pivot_rows() const { return no_pivot_rows_; }
  int no_pivot_var(int k) const { return no_pivot_vars_[k]; }

  // ── Build / refactorize ─────────────────────────────────────────────────
  //
  // Bind the constraint matrix `A` (column-major Eigen sparse) and factorize
  // the basis B = A[:, basic_index].
  //
  // `basic_index` must be of length m == A.rows() and contain valid column
  // indices in [0, A.cols()).
  //
  // Returns `true` on full-rank factorization, `false` on rank deficiency or
  // any internal failure (including HFactor::build returning a non-zero
  // rank_deficiency).  On `false`, `valid` is left clear.
  //
  // It is safe to call `factorize` repeatedly with different bases or even
  // different A matrices — internal HFactor state is reset.
  bool factorize(const Eigen::SparseMatrix<double>& A,
                 const int* basic_index,
                 int n_basic);

  // Factorize a standard-form basis while representing one +1 unit column
  // per row as HFactor's implicit logical variable.  This is the same model
  // used by HiGHS: if rank deficiency is detected, HFactor replaces only the
  // columns without pivots by the corresponding logicals. `repaired_basis`
  // preserves the caller's basis-position order exactly, except at positions
  // whose input columns received no pivot. The backend refactorizes those real
  // (possibly scaled) columns and keeps HFactor's internal pivot permutation
  // private from FTRAN, BTRAN, and update callers. A true return therefore
  // means that the final factor is usable, not that the input basis was full
  // rank;
  // `rank_deficiency` records how many columns were repaired.
  bool factorize_with_logicals(
      const Eigen::SparseMatrix<double>& A,
      const int* basic_index,
      int n_basic,
      const std::vector<int>& logical_col_by_row,
      std::vector<int>& repaired_basis);

  // ── FTRAN: solve B x = rhs ─────────────────────────────────────────────
  // `rhs` and `result` are dense pointers of length m.  In-place is
  // permitted (`rhs == result`).
  //
  // On entry, `result` need not be initialised.  On exit it holds B^{-1} rhs.
  void ftran(const double* rhs, double* result,
             const std::vector<int>* rhs_pattern = nullptr,
             std::vector<int>* result_pattern = nullptr) const;
  void ftran_for_update(const double* rhs, double* result,
                        const std::vector<int>* rhs_pattern = nullptr,
                        std::vector<int>* result_pattern = nullptr) const;

  // ── BTRAN: solve B^T y = rhs ───────────────────────────────────────────
  void btran(const double* rhs, double* result,
             const std::vector<int>* rhs_pattern = nullptr,
             std::vector<int>* result_pattern = nullptr) const;
  void btran_for_update(const double* rhs, double* result,
                        const std::vector<int>* rhs_pattern = nullptr,
                        std::vector<int>* result_pattern = nullptr) const;

  bool ftran_indexed(const std::vector<int>& rhs_index,
                     const std::vector<double>& rhs_value,
                     std::vector<int>& result_index,
                     std::vector<double>& result_value,
                     std::vector<int>& result_lookup,
                     bool capture_update = false) const;
  bool btran_indexed(const std::vector<int>& rhs_index,
                     const std::vector<double>& rhs_value,
                     std::vector<int>& result_index,
                     std::vector<double>& result_value,
                     std::vector<int>& result_lookup,
                     bool capture_update = false) const;

  // ── Product-form basis update ──────────────────────────────────────────
  //
  // Replace basis column at `pivot_row` (row index in [0,m)) with the new
  // column whose FTRAN-image is `a_q_after_ftran` and BTRAN-image of the
  // unit pivot row is `btran_e_p`.  Both inputs are dense length-m arrays;
  // this matches the calling convention of the dual-simplex driver where
  // FTRAN(a_q) and BTRAN(e_p) are computed prior to the update.
  //
  // The final vectors identify E_p(v), but HFactor's sparse FT update also
  // consumes triangular-solve intermediate packs captured by the matching
  // ftran_for_update()/btran_for_update() calls. Captures are generation-bound
  // and must not be used if either pivotal solve is subsequently refined; the
  // simplex driver then commits the exchange and INVERTs the new basis.
  bool update(int pivot_row,
              int entering_col,
              const double* a_q_after_ftran,
              const double* btran_e_p);
  bool update_captured(int pivot_row, int entering_col);

  // ── Refactor heuristic ─────────────────────────────────────────────────
  // Mirrors HiGHS' `u_total_x > u_merit_x` fill-based trigger as set by
  // the most recent update().  Returns true if HFactor advised the caller
  // to refactorize, false otherwise.
  bool needs_refactorise() const noexcept { return refactor_hint_ != 0; }

  // Reset book-keeping; does not invalidate the factorization itself.
  void reset_update_tracking() noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> p_;
  std::vector<int> no_pivot_rows_;
  std::vector<int> no_pivot_vars_;
  // Updated by every call to update(); kept out of Impl so the inline
  // accessor above stays cheap.
  mutable int refactor_hint_ = 0;
};

}  // namespace mipsolvers::engine
