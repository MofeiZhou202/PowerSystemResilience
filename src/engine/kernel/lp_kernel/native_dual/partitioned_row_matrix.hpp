#ifndef MIPSOLVERS_ENGINE_KERNEL_LP_KERNEL_NATIVE_DUAL_PARTITIONED_ROW_MATRIX_HPP
#define MIPSOLVERS_ENGINE_KERNEL_LP_KERNEL_NATIVE_DUAL_PARTITIONED_ROW_MATRIX_HPP

#include <cstdint>
#include <vector>

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::native_dual::detail {

// Row-major view of the standard-form matrix whose stored columns are, per row,
// partitioned into a nonbasic prefix [row_start, nonbasic_end) followed by a
// basic suffix.  Maintained incrementally across basis exchanges so the dual
// PRICE step (A_N^T rho) can scan only the nonbasic columns that survive into
// the pivotal row, skipping the basic columns whose products are discarded.
//
// It owns its arrays (built once from the CSC standard form, re-partitioned on
// rebuild) and is kept separate from the shared StandardRowMatrix, whose sorted
// within-row order coeff()/cut generation depend on.
class PartitionedRowMatrix {
 public:
  using Index = std::int64_t;

  bool empty() const noexcept { return rows_ == 0; }
  Index rows() const noexcept { return rows_; }
  Index cols() const noexcept { return cols_; }
  Index row_start(Index row) const noexcept {
    return outer_[static_cast<std::size_t>(row)];
  }
  Index row_end(Index row) const noexcept {
    return outer_[static_cast<std::size_t>(row) + 1];
  }
  // Exclusive upper bound of the nonbasic prefix for `row`.
  Index nonbasic_end(Index row) const noexcept {
    return nonbasic_end_[static_cast<std::size_t>(row)];
  }
  int col_at(Index slot) const noexcept {
    return inner_[static_cast<std::size_t>(slot)];
  }
  double value_at(Index slot) const noexcept {
    return values_[static_cast<std::size_t>(slot)];
  }

  // Transposes the CSC standard form into an (initially unpartitioned) CSR view
  // and records each slot's CSC source position for later value re-binding.
  void build(const StandardColumnMatrix& matrix) {
    rows_ = matrix.rows();
    cols_ = matrix.cols();
    const Index nnz = matrix.nonZeros();
    outer_.assign(static_cast<std::size_t>(rows_) + 1, 0);
    for (Index col = 0; col < cols_; ++col) {
      for (StandardColumnMatrix::InnerIterator it(matrix, col); it; ++it) {
        ++outer_[static_cast<std::size_t>(it.row()) + 1];
      }
    }
    for (Index row = 0; row < rows_; ++row) {
      outer_[static_cast<std::size_t>(row) + 1] +=
          outer_[static_cast<std::size_t>(row)];
    }
    inner_.assign(static_cast<std::size_t>(nnz), 0);
    values_.assign(static_cast<std::size_t>(nnz), 0.0);
    csc_pos_.assign(static_cast<std::size_t>(nnz), 0);
    slot_of_csc_.assign(static_cast<std::size_t>(nnz), 0);
    std::vector<Index> cursor(outer_.begin(), outer_.end() - 1);
    for (Index col = 0; col < cols_; ++col) {
      for (StandardColumnMatrix::InnerIterator it(matrix, col); it; ++it) {
        const Index row = it.row();
        const Index slot = cursor[static_cast<std::size_t>(row)]++;
        inner_[static_cast<std::size_t>(slot)] = static_cast<int>(col);
        values_[static_cast<std::size_t>(slot)] = it.value();
        csc_pos_[static_cast<std::size_t>(slot)] =
            static_cast<Index>(it.position());
        slot_of_csc_[static_cast<std::size_t>(it.position())] = slot;
      }
    }
    // Unpartitioned: treat every column as nonbasic until partition() runs.
    nonbasic_end_.assign(static_cast<std::size_t>(rows_), 0);
    for (Index row = 0; row < rows_; ++row) {
      nonbasic_end_[static_cast<std::size_t>(row)] = row_end(row);
    }
  }

  // Re-materializes the flat value stream from the (possibly Ruiz-rescaled) CSC
  // values without changing the partition, mirroring StandardRowMatrix.
  void rebind_values(const StandardColumnMatrix& matrix) {
    const double* values = matrix.valuePtr();
    if (values == nullptr) return;
    for (std::size_t slot = 0; slot < inner_.size(); ++slot) {
      values_[slot] = values[static_cast<std::size_t>(csc_pos_[slot])];
    }
  }

  // Establishes the nonbasic/basic partition per row from a basis-membership
  // predicate (basic[col] != 0).  Stable within each partition side.
  void partition(const std::vector<char>& basic) {
    for (Index row = 0; row < rows_; ++row) {
      const Index begin = row_start(row);
      const Index end = row_end(row);
      Index write = begin;
      for (Index slot = begin; slot < end; ++slot) {
        if (!basic[static_cast<std::size_t>(col_at(slot))]) {
          if (slot != write) swap_slots(slot, write);
          ++write;
        }
      }
      nonbasic_end_[static_cast<std::size_t>(row)] = write;
    }
  }

  // Column `col` has entered the basis: move it into the basic suffix of every
  // row it occupies.  O(nnz(col)).
  void set_basic(int col, const StandardColumnMatrix& matrix) {
    for (StandardColumnMatrix::InnerIterator it(matrix, col); it; ++it) {
      const Index row = it.row();
      const Index slot = slot_of_csc_[static_cast<std::size_t>(it.position())];
      const Index last_nonbasic = nonbasic_end_[static_cast<std::size_t>(row)] - 1;
      swap_slots(slot, last_nonbasic);
      nonbasic_end_[static_cast<std::size_t>(row)] = last_nonbasic;
    }
  }

  // Column `col` has left the basis: move it into the nonbasic prefix of every
  // row it occupies.  O(nnz(col)).
  void set_nonbasic(int col, const StandardColumnMatrix& matrix) {
    for (StandardColumnMatrix::InnerIterator it(matrix, col); it; ++it) {
      const Index row = it.row();
      const Index first_basic = nonbasic_end_[static_cast<std::size_t>(row)];
      const Index slot = slot_of_csc_[static_cast<std::size_t>(it.position())];
      swap_slots(slot, first_basic);
      nonbasic_end_[static_cast<std::size_t>(row)] = first_basic + 1;
    }
  }

  // Full consistency audit (debug/validation only): every prefix slot is a
  // nonbasic column, every suffix slot basic, and the CSC<->slot maps invert.
  bool verify(const std::vector<char>& basic) const {
    if (static_cast<Index>(basic.size()) < cols_) return false;
    for (Index row = 0; row < rows_; ++row) {
      const Index begin = row_start(row);
      const Index end = row_end(row);
      const Index split = nonbasic_end_[static_cast<std::size_t>(row)];
      if (split < begin || split > end) return false;
      for (Index slot = begin; slot < split; ++slot) {
        if (basic[static_cast<std::size_t>(col_at(slot))]) return false;
      }
      for (Index slot = split; slot < end; ++slot) {
        if (!basic[static_cast<std::size_t>(col_at(slot))]) return false;
      }
    }
    for (std::size_t slot = 0; slot < inner_.size(); ++slot) {
      if (slot_of_csc_[static_cast<std::size_t>(csc_pos_[slot])] !=
          static_cast<Index>(slot)) {
        return false;
      }
    }
    return true;
  }

 private:
  void swap_slots(Index a, Index b) noexcept {
    if (a == b) return;
    std::swap(inner_[static_cast<std::size_t>(a)],
              inner_[static_cast<std::size_t>(b)]);
    std::swap(values_[static_cast<std::size_t>(a)],
              values_[static_cast<std::size_t>(b)]);
    std::swap(csc_pos_[static_cast<std::size_t>(a)],
              csc_pos_[static_cast<std::size_t>(b)]);
    slot_of_csc_[static_cast<std::size_t>(csc_pos_[static_cast<std::size_t>(a)])] = a;
    slot_of_csc_[static_cast<std::size_t>(csc_pos_[static_cast<std::size_t>(b)])] = b;
  }

  Index rows_{0};
  Index cols_{0};
  std::vector<Index> outer_;         // size rows+1
  std::vector<Index> nonbasic_end_;  // size rows
  std::vector<int> inner_;           // size nnz: column per slot
  std::vector<double> values_;       // size nnz: value per slot
  std::vector<Index> csc_pos_;       // size nnz: CSC source position per slot
  std::vector<Index> slot_of_csc_;   // size nnz: inverse of csc_pos_
};

}  // namespace mipsolvers::engine::native_dual::detail

#endif  // MIPSOLVERS_ENGINE_KERNEL_LP_KERNEL_NATIVE_DUAL_PARTITIONED_ROW_MATRIX_HPP
