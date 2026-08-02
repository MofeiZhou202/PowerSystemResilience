#pragma once

#include <atomic>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/backend.hpp"
#include "mipsolvers/engine/solver/solver_adapter.hpp"

class Highs;

namespace mipsolvers::engine {

using StandardFormIndex = std::int64_t;

// Column-major standard-form matrix with adaptive Eigen storage. Models whose
// dimensions and nnz fit signed 32-bit indices use compact CSC; the container
// promotes to a 64-bit CSC before an operation that can exceed that range.
// This preserves the audited large-model path without charging every ordinary
// LP 8 bytes for each row index and column offset.
class StandardColumnMatrix {
 public:
  using StorageIndex = StandardFormIndex;
  using NarrowMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;
  using WideMatrix =
      Eigen::SparseMatrix<double, Eigen::ColMajor, StandardFormIndex>;

 private:
  std::variant<NarrowMatrix, WideMatrix> storage_{NarrowMatrix{}};

  template <typename Function>
  decltype(auto) visit(Function&& function) {
    return std::visit(std::forward<Function>(function), storage_);
  }
  template <typename Function>
  decltype(auto) visit(Function&& function) const {
    return std::visit(std::forward<Function>(function), storage_);
  }

 public:

  class InnerIterator {
   public:
    InnerIterator(const StandardColumnMatrix& matrix, StandardFormIndex col)
        : narrow_(matrix.uses_32_bit_indices()), col_(col) {
      if (narrow_) {
        narrow_it_.emplace(std::get<NarrowMatrix>(matrix.storage_),
                           static_cast<int>(col));
        position_ = static_cast<StandardFormIndex>(
            std::get<NarrowMatrix>(matrix.storage_)
                .outerIndexPtr()[static_cast<std::size_t>(col)]);
      } else {
        wide_it_.emplace(std::get<WideMatrix>(matrix.storage_), col);
        position_ = std::get<WideMatrix>(matrix.storage_)
                        .outerIndexPtr()[static_cast<std::size_t>(col)];
      }
    }
    explicit operator bool() const {
      return narrow_ ? static_cast<bool>(*narrow_it_)
                     : static_cast<bool>(*wide_it_);
    }
    InnerIterator& operator++() {
      if (narrow_) {
        ++(*narrow_it_);
      } else {
        ++(*wide_it_);
      }
      ++position_;
      return *this;
    }
    StandardFormIndex row() const {
      return narrow_ ? static_cast<StandardFormIndex>(narrow_it_->row())
                     : wide_it_->row();
    }
    StandardFormIndex col() const { return col_; }
    StandardFormIndex index() const { return row(); }
    StandardFormIndex position() const { return position_; }
    double value() const {
      return narrow_ ? narrow_it_->value() : wide_it_->value();
    }
    double& valueRef() {
      return narrow_ ? narrow_it_->valueRef() : wide_it_->valueRef();
    }

   private:
    bool narrow_{true};
    StandardFormIndex col_{0};
    StandardFormIndex position_{0};
    std::optional<NarrowMatrix::InnerIterator> narrow_it_;
    std::optional<WideMatrix::InnerIterator> wide_it_;
  };

  StandardColumnMatrix() = default;
  StandardColumnMatrix(StandardFormIndex rows, StandardFormIndex cols) {
    resize(rows, cols);
  }

  template <int Options, typename Index>
  StandardColumnMatrix(
      const Eigen::SparseMatrix<double, Options, Index>& matrix) {
    assign_eigen(matrix);
  }

  template <typename Derived>
  StandardColumnMatrix& operator=(
      const Eigen::SparseMatrixBase<Derived>& matrix) {
    WideMatrix evaluated = matrix.derived();
    assign_eigen(evaluated);
    return *this;
  }

  template <int Options, typename Index>
  StandardColumnMatrix& operator=(
      const Eigen::SparseMatrix<double, Options, Index>& matrix) {
    assign_eigen(matrix);
    return *this;
  }

  void resize(StandardFormIndex rows, StandardFormIndex cols) {
    if (fits_narrow(rows) && fits_narrow(cols)) {
      storage_.template emplace<NarrowMatrix>(static_cast<int>(rows),
                                              static_cast<int>(cols));
    } else {
      storage_.template emplace<WideMatrix>(rows, cols);
    }
  }

  void reserve(StandardFormIndex nnz) {
    if (uses_32_bit_indices() && !fits_narrow(nnz)) promote_to_wide();
    visit([&](auto& matrix) {
      using Index = typename std::decay_t<decltype(matrix)>::StorageIndex;
      matrix.reserve(static_cast<Index>(nnz));
    });
  }

  void startVec(StandardFormIndex col) {
    visit([&](auto& matrix) {
      using Index = typename std::decay_t<decltype(matrix)>::StorageIndex;
      matrix.startVec(static_cast<Index>(col));
    });
  }

  double& insertBackByOuterInner(StandardFormIndex outer,
                                 StandardFormIndex inner) {
    return visit([&](auto& matrix) -> double& {
      using Index = typename std::decay_t<decltype(matrix)>::StorageIndex;
      return matrix.insertBackByOuterInner(static_cast<Index>(outer),
                                           static_cast<Index>(inner));
    });
  }

  double& insert(StandardFormIndex row, StandardFormIndex col) {
    return visit([&](auto& matrix) -> double& {
      using Index = typename std::decay_t<decltype(matrix)>::StorageIndex;
      return matrix.insert(static_cast<Index>(row), static_cast<Index>(col));
    });
  }

  template <typename InputIt>
  void setFromTriplets(InputIt begin, InputIt end) {
    visit([&](auto& matrix) { matrix.setFromTriplets(begin, end); });
  }

  void makeCompressed() { visit([](auto& matrix) { matrix.makeCompressed(); }); }
  void finalize() { visit([](auto& matrix) { matrix.finalize(); }); }
  bool isCompressed() const {
    return visit([](const auto& matrix) { return matrix.isCompressed(); });
  }
  StandardFormIndex rows() const {
    return visit([](const auto& matrix) {
      return static_cast<StandardFormIndex>(matrix.rows());
    });
  }
  StandardFormIndex cols() const {
    return visit([](const auto& matrix) {
      return static_cast<StandardFormIndex>(matrix.cols());
    });
  }
  StandardFormIndex outerSize() const { return cols(); }
  StandardFormIndex nonZeros() const {
    return visit([](const auto& matrix) {
      return static_cast<StandardFormIndex>(matrix.nonZeros());
    });
  }
  double coeff(StandardFormIndex row, StandardFormIndex col) const {
    return visit([&](const auto& matrix) {
      using Index = typename std::decay_t<decltype(matrix)>::StorageIndex;
      return matrix.coeff(static_cast<Index>(row), static_cast<Index>(col));
    });
  }
  double* valuePtr() {
    return visit([](auto& matrix) { return matrix.valuePtr(); });
  }
  const double* valuePtr() const {
    return visit([](const auto& matrix) { return matrix.valuePtr(); });
  }
  bool uses_32_bit_indices() const noexcept {
    return std::holds_alternative<NarrowMatrix>(storage_);
  }
  std::size_t index_memory_bytes() const noexcept {
    return visit([](const auto& matrix) {
      using Index = typename std::decay_t<decltype(matrix)>::StorageIndex;
      return (static_cast<std::size_t>(matrix.outerSize()) + 1 +
              static_cast<std::size_t>(matrix.nonZeros())) *
             sizeof(Index);
    });
  }
  const NarrowMatrix* narrow_matrix() const noexcept {
    return std::get_if<NarrowMatrix>(&storage_);
  }
  const WideMatrix* wide_matrix() const noexcept {
    return std::get_if<WideMatrix>(&storage_);
  }

  Eigen::VectorXd operator*(const Eigen::VectorXd& x) const {
    if (x.size() != cols()) return {};
    Eigen::VectorXd result = Eigen::VectorXd::Zero(rows());
    for (StandardFormIndex col = 0; col < cols(); ++col) {
      const double multiplier = x[col];
      if (multiplier == 0.0) continue;
      for (InnerIterator it(*this, col); it; ++it) {
        result[it.row()] += it.value() * multiplier;
      }
    }
    return result;
  }

  Eigen::VectorXd transpose_multiply(const Eigen::VectorXd& y) const {
    if (y.size() != rows()) return {};
    Eigen::VectorXd result = Eigen::VectorXd::Zero(cols());
    for (StandardFormIndex col = 0; col < cols(); ++col) {
      double value = 0.0;
      for (InnerIterator it(*this, col); it; ++it) {
        value += it.value() * y[it.row()];
      }
      result[col] = value;
    }
    return result;
  }

  class TransposeView {
   public:
    explicit TransposeView(const StandardColumnMatrix& matrix)
        : matrix_(matrix) {}
    Eigen::VectorXd operator*(const Eigen::VectorXd& y) const {
      return matrix_.transpose_multiply(y);
    }

   private:
    const StandardColumnMatrix& matrix_;
  };
  TransposeView transpose() const { return TransposeView(*this); }

  template <int Options = Eigen::ColMajor, typename Index = int>
  Eigen::SparseMatrix<double, Options, Index> to_eigen() const {
    std::vector<Eigen::Triplet<double, Index>> entries;
    entries.reserve(static_cast<std::size_t>(nonZeros()));
    for (StandardFormIndex col = 0; col < cols(); ++col) {
      for (InnerIterator it(*this, col); it; ++it) {
        entries.emplace_back(static_cast<Index>(it.row()),
                             static_cast<Index>(col), it.value());
      }
    }
    Eigen::SparseMatrix<double, Options, Index> result(
        static_cast<Index>(rows()), static_cast<Index>(cols()));
    result.setFromTriplets(entries.begin(), entries.end());
    result.makeCompressed();
    return result;
  }

 private:
  static bool fits_narrow(StandardFormIndex value) noexcept {
    return value >= 0 &&
           value <= static_cast<StandardFormIndex>(
                        (std::numeric_limits<int>::max)());
  }
  void promote_to_wide() {
    if (!uses_32_bit_indices()) return;
    WideMatrix wide = std::get<NarrowMatrix>(storage_);
    storage_ = std::move(wide);
  }
  template <int Options, typename Index>
  void assign_eigen(const Eigen::SparseMatrix<double, Options, Index>& source) {
    const bool narrow = fits_narrow(source.rows()) &&
                        fits_narrow(source.cols()) &&
                        fits_narrow(source.nonZeros());
    if (narrow) {
      NarrowMatrix matrix = source;
      matrix.makeCompressed();
      storage_ = std::move(matrix);
    } else {
      WideMatrix matrix = source;
      matrix.makeCompressed();
      storage_ = std::move(matrix);
    }
  }

};

// CSR row-access view over StandardColumnMatrix. It owns row pointers and an
// adaptive-width (row-offset, column, CSC-position) indices, but shares the CSC
// value array. At the audited 10^7-by-10^7 / 10^8-nnz scale all three indices
// are 32-bit, so the row view costs 4 bytes/row + 8 bytes/nnz instead of
// duplicating 8-byte values and 64-bit indices.
class StandardRowMatrix {
 public:
  using StorageIndex = StandardFormIndex;

  class InnerIterator {
   public:
    InnerIterator(const StandardRowMatrix& matrix, StandardFormIndex row)
        : matrix_(&matrix), row_(row) {
      if (row >= 0 && row < matrix.rows_) {
        position_ = matrix.outer_at(row);
        end_ = matrix.outer_at(row + 1);
      }
    }
    explicit operator bool() const { return position_ < end_; }
    InnerIterator& operator++() {
      ++position_;
      return *this;
    }
    StandardFormIndex row() const { return row_; }
    StandardFormIndex col() const { return matrix_->inner_at(position_); }
    StandardFormIndex index() const { return col(); }
    double value() const { return matrix_->value_at(position_); }

   private:
    const StandardRowMatrix* matrix_{nullptr};
    StandardFormIndex row_{-1};
    StandardFormIndex position_{0};
    StandardFormIndex end_{0};
  };

  StandardRowMatrix() = default;
  StandardRowMatrix& operator=(const StandardColumnMatrix& matrix) {
    rebuild(matrix);
    return *this;
  }

  void rebuild(const StandardColumnMatrix& matrix) {
    rows_ = matrix.rows();
    cols_ = matrix.cols();
    const StandardFormIndex nnz = matrix.nonZeros();
    constexpr auto kUint32Max =
        static_cast<StandardFormIndex>(
            (std::numeric_limits<std::uint32_t>::max)());
    wide_columns_ = cols_ > kUint32Max;
    wide_positions_ = nnz > kUint32Max;
    wide_outer_ = nnz > kUint32Max;
    if (wide_outer_) {
      outer64_.assign(static_cast<std::size_t>(rows_) + 1, 0);
      outer32_.clear();
    } else {
      outer32_.assign(static_cast<std::size_t>(rows_) + 1, 0);
      outer64_.clear();
    }
    for (StandardFormIndex col = 0; col < cols_; ++col) {
      for (StandardColumnMatrix::InnerIterator it(matrix, col); it; ++it) {
        increment_outer(it.row() + 1);
      }
    }
    for (StandardFormIndex row = 0; row < rows_; ++row) {
      set_outer(row + 1, outer_at(row + 1) + outer_at(row));
    }
    std::vector<std::uint32_t> cursor32;
    std::vector<StandardFormIndex> cursor64;
    if (wide_outer_) {
      cursor64 = outer64_;
    } else {
      cursor32 = outer32_;
    }
    if (wide_columns_) {
      inner64_.assign(static_cast<std::size_t>(nnz), 0);
      inner32_.clear();
    } else {
      inner32_.assign(static_cast<std::size_t>(nnz), 0);
      inner64_.clear();
    }
    if (wide_positions_) {
      position64_.assign(static_cast<std::size_t>(nnz), 0);
      position32_.clear();
    } else {
      position32_.assign(static_cast<std::size_t>(nnz), 0);
      position64_.clear();
    }
    for (StandardFormIndex col = 0; col < cols_; ++col) {
      for (StandardColumnMatrix::InnerIterator it(matrix, col); it; ++it) {
        const StandardFormIndex row = it.row();
        const StandardFormIndex target =
            wide_outer_
                ? cursor64[static_cast<std::size_t>(row)]++
                : static_cast<StandardFormIndex>(
                      cursor32[static_cast<std::size_t>(row)]++);
        if (wide_columns_) {
          inner64_[static_cast<std::size_t>(target)] = col;
        } else {
          inner32_[static_cast<std::size_t>(target)] =
              static_cast<std::uint32_t>(col);
        }
        if (wide_positions_) {
          position64_[static_cast<std::size_t>(target)] = it.position();
        } else {
          position32_[static_cast<std::size_t>(target)] =
              static_cast<std::uint32_t>(it.position());
        }
      }
    }
    values_ = matrix.valuePtr();
    source_nnz_ = nnz;
  }

  void rebind_values(const StandardColumnMatrix& matrix) noexcept {
    if (matrix.rows() != rows_ || matrix.cols() != cols_ ||
        matrix.nonZeros() != source_nnz_) {
      values_ = nullptr;
      return;
    }
    values_ = matrix.valuePtr();
  }

  StandardFormIndex rows() const noexcept { return rows_; }
  StandardFormIndex cols() const noexcept { return cols_; }
  StandardFormIndex outerSize() const noexcept { return rows_; }
  StandardFormIndex nonZeros() const noexcept { return source_nnz_; }
  StandardFormIndex row_start(StandardFormIndex row) const noexcept {
    return outer_at(row);
  }
  StandardFormIndex row_end(StandardFormIndex row) const noexcept {
    return outer_at(row + 1);
  }
  bool uses_32_bit_row_offsets() const noexcept { return !wide_outer_; }
  bool shares_values_with(const StandardColumnMatrix& matrix) const noexcept {
    return values_ == matrix.valuePtr() && rows_ == matrix.rows() &&
           cols_ == matrix.cols() && source_nnz_ == matrix.nonZeros();
  }
  std::size_t index_memory_bytes() const noexcept {
    return outer32_.capacity() * sizeof(std::uint32_t) +
           outer64_.capacity() * sizeof(StandardFormIndex) +
           inner32_.capacity() * sizeof(std::uint32_t) +
           inner64_.capacity() * sizeof(StandardFormIndex) +
           position32_.capacity() * sizeof(std::uint32_t) +
           position64_.capacity() * sizeof(StandardFormIndex);
  }
  double coeff(StandardFormIndex row, StandardFormIndex col) const {
    if (row < 0 || row >= rows_ || col < 0 || col >= cols_ ||
        values_ == nullptr) {
      return 0.0;
    }
    StandardFormIndex first = outer_at(row);
    StandardFormIndex last = outer_at(row + 1);
    while (first < last) {
      const StandardFormIndex middle = first + (last - first) / 2;
      const StandardFormIndex candidate = inner_at(middle);
      if (candidate < col) {
        first = middle + 1;
      } else {
        last = middle;
      }
    }
    return first < outer_at(row + 1) &&
                   inner_at(first) == col
               ? value_at(first)
               : 0.0;
  }

 private:
  StandardFormIndex outer_at(StandardFormIndex row) const noexcept {
    return wide_outer_
               ? outer64_[static_cast<std::size_t>(row)]
               : static_cast<StandardFormIndex>(
                     outer32_[static_cast<std::size_t>(row)]);
  }
  void set_outer(StandardFormIndex row, StandardFormIndex value) noexcept {
    if (wide_outer_) {
      outer64_[static_cast<std::size_t>(row)] = value;
    } else {
      outer32_[static_cast<std::size_t>(row)] =
          static_cast<std::uint32_t>(value);
    }
  }
  void increment_outer(StandardFormIndex row) noexcept {
    if (wide_outer_) {
      ++outer64_[static_cast<std::size_t>(row)];
    } else {
      ++outer32_[static_cast<std::size_t>(row)];
    }
  }
  StandardFormIndex inner_at(StandardFormIndex position) const {
    return wide_columns_
               ? inner64_[static_cast<std::size_t>(position)]
               : static_cast<StandardFormIndex>(
                     inner32_[static_cast<std::size_t>(position)]);
  }
  StandardFormIndex source_position(StandardFormIndex position) const {
    return wide_positions_
               ? position64_[static_cast<std::size_t>(position)]
               : static_cast<StandardFormIndex>(
                     position32_[static_cast<std::size_t>(position)]);
  }
  double value_at(StandardFormIndex position) const {
    return values_ == nullptr ? 0.0 : values_[source_position(position)];
  }

  StandardFormIndex rows_{0};
  StandardFormIndex cols_{0};
  StandardFormIndex source_nnz_{0};
  bool wide_columns_{false};
  bool wide_positions_{false};
  bool wide_outer_{false};
  std::vector<std::uint32_t> outer32_;
  std::vector<StandardFormIndex> outer64_;
  std::vector<std::uint32_t> inner32_;
  std::vector<StandardFormIndex> inner64_;
  std::vector<std::uint32_t> position32_;
  std::vector<StandardFormIndex> position64_;
  const double* values_{nullptr};
};

struct StandardFormMatrixStorage {
  StandardColumnMatrix A;
  StandardRowMatrix A_row;

  StandardFormMatrixStorage() = default;
  StandardFormMatrixStorage(const StandardFormMatrixStorage& other)
      : A(other.A), A_row(other.A_row) {
    A_row.rebind_values(A);
  }
  StandardFormMatrixStorage(StandardFormMatrixStorage&& other) noexcept
      : A(std::move(other.A)), A_row(std::move(other.A_row)) {
    A_row.rebind_values(A);
  }
  StandardFormMatrixStorage& operator=(const StandardFormMatrixStorage& other) {
    if (this == &other) return *this;
    A = other.A;
    A_row = other.A_row;
    A_row.rebind_values(A);
    return *this;
  }
  StandardFormMatrixStorage& operator=(StandardFormMatrixStorage&& other) noexcept {
    if (this == &other) return *this;
    A = std::move(other.A);
    A_row = std::move(other.A_row);
    A_row.rebind_values(A);
    return *this;
  }
};

struct SimplexBasis;  // forward declaration
struct SimplexOptions;
struct StandardFormLP;
struct SimplexResult;
struct SparseFactorTelemetry;

enum class BasisOpsKind {
  NativeSparse = 0,
  VendoredHighs = 1,
};

/// First-class standard-form basis operations used by cut/proof code.
///
/// Implementations must live in the exact same canonical StandardFormLP column
/// and row space as the SimplexResult that owns them.  Native SparseBasis uses
/// the in-house LU/FT stack; VendoredHighsBasis delegates to HiGHS' basis
/// inverse/tableau APIs while retaining the solved HiGHS instance.
struct BasisOps {
  virtual ~BasisOps() = default;
  virtual BasisOpsKind kind() const = 0;
  virtual Eigen::VectorXd ftran(const Eigen::VectorXd& rhs) const = 0;
  virtual Eigen::VectorXd btran(const Eigen::VectorXd& rhs) const = 0;
  /// Return row i of B^{-1} in the owner LP row space.  HiGHS' tableau
  /// separator uses this exact object as rowEp before aggregation.
  virtual bool basis_inverse_row(int row, Eigen::VectorXd& out) const = 0;
  virtual bool basis_inverse_row_sparse_entries(
      int,
      std::vector<std::pair<int, double>>&) const {
    return false;
  }
  virtual bool tableau_row(int row, Eigen::RowVectorXd& out) const = 0;
  virtual void clear_etas() {}
  virtual void truncate_etas_to(int) {}
  virtual int eta_count() const { return 0; }
  virtual int generation() const { return -1; }
  virtual SparseFactorTelemetry factor_telemetry() const;
  virtual void rebind_A(const StandardColumnMatrix&) {}
  virtual bool bound_to_A(const StandardColumnMatrix&) const {
    return false;
  }
  // A transaction snapshots the mutable solver model and basis represented by
  // this object. Rollback must also leave basis solves usable, i.e. restore or
  // deterministically rebuild the corresponding numeric factorization.
  virtual bool begin_lp_transaction() { return false; }
  virtual bool commit_lp_transaction() { return false; }
  virtual bool rollback_lp_transaction() { return false; }
  virtual bool lp_transaction_active() const { return false; }
  virtual bool supports_incremental_rows() const { return false; }
  // Reoptimize an unchanged standard-form matrix after bounds/RHS updates.
  // Implementations must be transactional: false leaves the owner snapshot
  // active and usable.
  virtual bool resolve_same_structure(const StandardFormLP&,
                                      const SimplexBasis*,
                                      const SimplexOptions&,
                                      SimplexResult&) {
    return false;
  }
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  virtual std::shared_ptr<Highs> highs_handle() const { return nullptr; }
#endif
  virtual bool delete_rows_cols_and_resolve(
      const StandardFormLP&,
      const SimplexBasis*,
      const std::vector<int>&,
      const std::vector<int>&,
      const SimplexOptions&,
      SimplexResult&) {
    return false;
  }
};

// Basis-factor backend selector for dual simplex SparseBasis internals.
//
// BackendA_UmfpackNative: current production path (UMFPACK numeric + native
// sparse triangular solves; FT update path available but currently gated off in
// production policy).
// BackendB_HiGHSSafe / ForceFT / ShortChain: experimental HiGHS-like FT-first
// modes that differ only in how aggressively they keep FT active before
// refactorization or eta fallback.
enum class SimplexFactorBackend {
  BackendA_UmfpackNative = 0,
  BackendB_HiGHSSafe = 1,
  BackendB_ForceFT = 2,
  BackendB_ShortChain = 3,
};

inline SimplexFactorBackend simplex_factor_backend_from_id(int id) {
  switch (id) {
    case 3:
      return SimplexFactorBackend::BackendB_ShortChain;
    case 2:
      return SimplexFactorBackend::BackendB_ForceFT;
    case 1:
      return SimplexFactorBackend::BackendB_HiGHSSafe;
    case 0:
    default:
      return SimplexFactorBackend::BackendA_UmfpackNative;
  }
}

struct SparseFactorTelemetry {
  bool ft_valid{false};
  int ft_updates{0};
  double max_growth{1.0};
  double u_fill_ratio{1.0};
  std::uint64_t matrix_copies{0};
  std::uint64_t dense_solves{0};
  std::uint64_t indexed_solves{0};
};

struct SimplexOptions {
  int max_iter{2000};
  double time_limit_sec{0.0};
  bool* time_limit_hit{nullptr};
  double feasibility_tol{1e-8};
  double optimality_tol{1e-8};
  bool verbose{false};
  // Legacy ABI field. Cold versus warm is determined solely by whether a
  // basis hint is supplied; a failed warm solve is never restarted cold.
  bool allow_cold_start{true};
  // Opt-in for IPM crossover bases. If a validated warm basis is not already
  // dual feasible, run primal Phase I from that basis instead of rejecting it.
  // This preserves the recovered basis and never restarts from a logical one.
  bool allow_warm_primal_phase_one{false};
  // Pointer to atomic incumbent bound for early termination in parallel B&C.
  // When non-null, simplex checks every incumbent_check_interval iterations and
  // aborts if obj >= *incumbent_bound (node will be pruned by caller).
  const std::atomic<double>* incumbent_bound{nullptr};
  int incumbent_check_interval{50};
  // Legacy ABI field; the rewritten kernels use deterministic full pricing.
  bool use_partial_pricing{true};
  // Legacy ABI field; primal perturbation was removed.
  bool perturb_degenerate_primal{false};
  // ABI compatibility only. The rewritten native kernel never retries a
  // different basis after a failed warm start.
  const SimplexBasis* fallback_basis{nullptr};
  // Opt-in exact cold-start DSE initialization. The scalable default uses a
  // unit-weight Devex framework and lets the pivot recurrence refine it;
  // setting this true performs one checked BTRAN per basis row.
  bool exact_dse_initialization{false};
  // Parallel PRICE/BFRT worker budget. 0 selects a bounded hardware-aware
  // default, 1 forces deterministic serial execution, and values >1 cap the
  // number of persistent thread-pool tasks. Small or sparse scans remain
  // serial regardless. MIPSOLVERS_LP_KERNEL_THREADS overrides this value.
  int lp_kernel_threads{0};
  // A cold basis may become dual feasible by shifting the costs of one-sided
  // nonbasic columns. This is cheap for sparse defects, but a dense set of
  // shifts creates an expensive mandatory original-cost cleanup. The default
  // therefore selects genuine dual Phase I once the number of required shifts
  // reaches max(count, ceil(fraction * standard-form columns)).
  int dual_shift_start_max_count{256};
  double dual_shift_start_max_fraction{0.125};
  // Legacy ABI fields; post-optimal frontier remapping was removed.
  bool enable_degenerate_frontier_remap{false};
  bool suppress_degenerate_frontier_remap{false};
  // Opt-in for root cut-pool resolves where the caller has appended only
  // <= rows to a scale-compatible standard form and made the new row slacks
  // basic.  In that case the new basis is block triangular around the parent
  // basis, so SparseBasis can reuse the parent factor for FTRAN/BTRAN instead
  // of rebuilding a large factorization.
  bool allow_incremental_row_append_factor{false};
  // Internal ownership contract: the caller guarantees this solve belongs to
  // one sequential workspace, so a cached HiGHS model may be mutated in place.
  // Parallel workers leave this false and retain thread-local solver state.
  bool allow_persistent_lp_state{false};
  // Preserve a copy of the caller-owned StandardFormLP in solve_lp_from_sf()
  // results for cut generation. Disable for transient auxiliary solves that
  // consume only the primal/basis; this avoids duplicating both sparse matrix
  // orientations in every retained result.
  bool retain_standard_form{false};
  // Full canonical A*x=b residual scans performed during non-terminal INVERT
  // reconstruction. The first INVERT is always checked; subsequent scans run
  // at this interval. Terminal publication always performs an exact scan.
  // Values <= 0 disable periodic scans after the checked first INVERT.
  int intermediate_audit_interval{32};
  // A HiGHS rejection is terminal; this field never describes a fallback
  // chain. ExperimentalNative is an explicit development-only selection.
  LpKernelBackend lp_kernel_backend{LpKernelBackend::HiGHS};
  // Selects which basis-factor backend SparseBasis should use.
  // Defaults to current production backend (A).
  SimplexFactorBackend factor_backend{SimplexFactorBackend::BackendA_UmfpackNative};
  // Legacy ABI fields retained while downstream callers migrate. The rewritten
  // native simplex executes exactly one numerical attempt and ignores all
  // escalation level/backend fields. escalation_residual_tol remains the
  // fail-closed original-space publication audit tolerance.
  int escalation_max_level{2};
  double escalation_umfpack_pivot_tolerance{1.0};
  int escalation_ruiz_rounds{25};
  // Acceptance threshold for the original-space feasibility audit applied to
  // cold-start results: max violation <= tol * max(1, |b|_inf, |beq|_inf).
  double escalation_residual_tol{1e-6};
  // Legacy ABI fields; numerical escalation and rescue backends were removed.
  int escalation_start_level{0};
  int escalation_max_level_sf{2};
  // Opt-in: run adaptive HiGHS presolve before the native solve, solve the
  // reduced LP, then postsolve the primal back to original space (primal-only,
  // no basis needed).  Only applied on cold solves (no basis hint).  On any
  // failure the solver falls back to a direct solve, so a wrong or infeasible
  // answer is never published.  The MIPSOLVERS_PRESOLVE* env vars override the
  // effective config (see highs_lp_presolve_config_from_env).
  bool use_highs_presolve{false};
};

struct SimplexBasis {
  std::vector<int> indices;
  // Optional shared backing storage for basis indices.
  // When engaged, `indices` may be empty to avoid per-node duplicate buffers.
  std::shared_ptr<const std::vector<int>> shared_indices;
  int rows{0};
  int cols{0};
  // Cached reduced costs (depend only on basis/binv/c, not b).
  std::shared_ptr<const Eigen::VectorXd> cached_reduced_costs;
  // Column scale used by cached_reduced_costs. Native simplex stores reduced
  // costs in scaled standard-form maximization convention; HiGHS MIP
  // propagation consumes unscaled minimization col_dual, recovered as
  // col_dual[j] = -reduced_costs[j] / cached_col_scale[j].
  std::shared_ptr<const Eigen::VectorXd> cached_col_scale;
  // Non-basic variable status: 1 = at upper bound, 0 = at lower bound.
  std::vector<char> at_upper;
  // Cached first-class standard-form basis operations.
  // Persisted across B&C node solves to avoid re-factorization when basis
  // indices are unchanged (only bounds differ between parent and child).
  std::shared_ptr<BasisOps> cached_sparse_basis;
  // Move-safe dual steepest-edge state. Unlike cached_sparse_basis, these
  // vectors do not retain a pointer to the StandardFormLP matrix and can be
  // carried through LPModel-level result/certificate caches safely.
  std::shared_ptr<const std::vector<double>> cached_dse_weights;
  std::shared_ptr<const std::vector<int>> cached_dse_basis;
  // Eta count at the time cached_sparse_basis was persisted.
  // Siblings can detect mutation: if current eta_count > this, another consumer modified it.
  int persist_eta_count{0};
  // Standard-form column counts from the solve that produced this basis.
  // Used to correctly remap columns when extending the basis for cut rows.
  int sf_n_slack{-1};
  int sf_n_surplus{-1};
  int sf_n_artificial{-1};

  /// Returns basis indices regardless of local/shared storage mode.
  const std::vector<int>& basis_indices() const {
    return shared_indices ? *shared_indices : indices;
  }

  /// Number of basis indices available.
  std::size_t index_count() const {
    return basis_indices().size();
  }

  /// Ensures `indices` owns mutable storage (materializes from shared mode).
  void ensure_owned_indices() {
    if (!shared_indices) return;
    indices = *shared_indices;
    shared_indices.reset();
  }

  /// Compacts local `indices` into shared immutable storage.
  void compact_indices_storage() {
    if (shared_indices || indices.empty()) return;
    shared_indices = std::make_shared<const std::vector<int>>(std::move(indices));
    indices.clear();
    indices.shrink_to_fit();
  }

  /// If basis indices are identical, reuse the same shared backing buffer.
  void try_share_indices_from(const SimplexBasis& other) {
    const auto& mine = basis_indices();
    const auto& theirs = other.basis_indices();
    if (mine.size() != theirs.size() || mine != theirs) return;
    if (other.shared_indices) {
      shared_indices = other.shared_indices;
    } else {
      shared_indices = std::make_shared<const std::vector<int>>(theirs);
    }
    indices.clear();
    indices.shrink_to_fit();
  }
};

struct StandardFormLP : StandardFormMatrixStorage {
  // Immutable across copies and bound/cost updates; regenerated whenever the
  // matrix structure is rebuilt. Enables O(1) persistent-model validation.
  std::uint64_t structure_id{0};
  Eigen::VectorXd b;
  Eigen::VectorXd c_max;
  Eigen::VectorXd lb_shift;
  // Upper bounds for each column in shifted space (var_ub[j] = ub_j - lb_shift_j).
  // +inf for unbounded variables and slack/surplus/artificial columns.
  Eigen::VectorXd var_ub;
  double objective_const{0.0};
  int n_original{0};
  int n_slack{0};
  int n_surplus{0};
  int n_artificial{0};
  std::vector<VarType> original_types;
  std::vector<int> row_to_artificial_col;
  std::vector<int> row_to_slack_col;
  std::vector<int> row_to_surplus_col;
  // Reverse lookup: aux_col_to_row[col] = row if col is the slack/surplus for
  // that row, -1 otherwise.  Populated by build_standard_form_lp and
  // append_leq_rows_to_standard_form for O(1) aux-column→row queries.
  // Size = A.cols(); indices 0..n_original-1 are always -1.
  std::vector<int> aux_col_to_row;
  std::vector<int> row_sign;    // +1 or -1: sign flip applied at construction
  // Bound value used to form each standard-form RHS before lb_shift:
  //   row_sign * (row_rhs_value - A_i * lb_shift).
  // For ranged rows oriented from their lower side this is row_lhs[i], not b[i].
  Eigen::VectorXd row_rhs_value;
  std::vector<int> ub_row_var;  // For upper-bound rows: var index; -1 otherwise
  std::vector<int> source_highs_row;  // Native SF row -> HiGHS presolved row.
  std::vector<int> source_row_start;
  std::vector<int> source_row_index;
  std::vector<double> source_row_value;
  // Ruiz equilibration scale factors (populated by ruiz_scale_standard_form).
  // Scaled LP: D_r * A * D_c,  D_r * b,  D_c * c_max,  var_ub / D_c.
  Eigen::VectorXd row_scale;    // length m; empty if unscaled
  Eigen::VectorXd col_scale;    // length n; empty if unscaled
};

struct SimplexResult {
  SolveResult result;
  StandardFormLP form;
  SimplexBasis basis;
  // Deprecated compatibility field. Native and HiGHS paths leave it empty;
  // use basis.cached_sparse_basis for on-demand basis solves.
  Eigen::MatrixXd basis_inverse;
  Eigen::VectorXd x_std;
  Eigen::VectorXd x_basic;
  Eigen::VectorXd reduced_costs;
  double max_objective{0.0};
  bool solved_from_hint{false};
  bool dual_reoptimized{false};
  bool exact_optimal{false};
};

StandardFormLP build_standard_form_lp(const LPModel& lp);

// Append <= rows, expressed in original variable space, to an existing
// StandardFormLP while preserving all old row/column scaling.  The appended
// rows keep their logical <= orientation even if the shifted RHS is negative:
// a first-class LP object may start from the resulting primal-infeasible
// slack-basic state and repair it by dual simplex.  This differs deliberately
// from a cold standard-form rebuild, which may flip negative-RHS rows.
bool append_leq_rows_to_standard_form(const StandardFormLP& base_sf,
                                      const std::vector<Eigen::SparseVector<double>>& rows,
                                      const std::vector<double>& rhs,
                                      StandardFormLP& out,
                                      double feasibility_tol = 1e-12);

// Append <= rows using the same canonical row orientation and Ruiz scaling
// policy as a fresh build_standard_form_lp(lp)+ruiz_scale_standard_form(lp)
// rebuild.  This is the first-class root-LP path for cut admission: old rows
// keep their membership/order, new inequality rows are inserted before the
// equality block, negative shifted RHS rows are flipped to >= with
// surplus/artificial columns, and the whole resulting standard form is scaled
// by the native Ruiz policy.
bool append_leq_rows_to_canonical_standard_form(
    const StandardFormLP& base_sf,
    const std::vector<Eigen::SparseVector<double>>& rows,
    const std::vector<double>& rhs,
    StandardFormLP& out,
    int ruiz_rounds = 10,
    double feasibility_tol = 1e-12);

// Apply Ruiz equilibration scaling to a StandardFormLP in-place.
// Iteratively balances row/column infinity norms of A toward 1.0.
// Populates sf.row_scale and sf.col_scale; scales A, A_row, b, c_max, var_ub.
void ruiz_scale_standard_form(StandardFormLP& sf, int rounds = 10);

// Rebuild only the bounds-dependent parts of a StandardFormLP (b, lb_shift,
// objective_const) while reusing the constraint matrix A.  The structural
// flipping decisions (row sign, slack/surplus assignment) are taken from
// base_sf; only numeric values are updated for the new variable bounds.
void update_standard_form_bounds(StandardFormLP& sf,
                                 const LPModel& lp,
                                 const Eigen::VectorXd& node_lb,
                                 const Eigen::VectorXd& node_ub);

// Rewrite only the cost vector (c_max) and objective_const of an existing
// StandardFormLP in place. `new_c` has length == sf.n_original and is
// expressed in the user (LPModel) sense given by `sense`. The constraint
// matrix A, RHS b, bounds var_ub, row/col signs and Ruiz scale factors are
// left untouched, so any SparseBasis / LU factorization cached on a
// SimplexBasis hint referencing this sf remains structurally valid.
//
// Callers reusing a basis hint after a cost change MUST clear its stale
// reduced-cost cache (hint.cached_reduced_costs.reset()) because it depends
// on c_max.
// After a cost-only change, the basis is generally primal feasible but
// dual-infeasible, so solve_lp_from_sf selects primal simplex reoptimization.
void update_standard_form_cost(StandardFormLP& sf,
                               Sense sense,
                               const Eigen::VectorXd& new_c);

SimplexResult solve_lp_with_basis(const LPModel& lp,
                                  const SimplexOptions& opt = {},
                                  const SimplexBasis* basis_hint = nullptr);

/// Describes a bound change for incremental warm-start in B&C.
struct BoundChangeInfo {
  int var_idx{-1};   ///< Original variable index
  double delta{0.0}; ///< Legacy diagnostic delta (new - old when finite)
  bool is_lb{false}; ///< true = lower bound changed, false = upper bound changed
  double old_value{std::numeric_limits<double>::quiet_NaN()};
  double new_value{std::numeric_limits<double>::quiet_NaN()};
};

// Fast-path incremental update when exact bound changes are known.
// Avoids the O(n) scan to detect which bounds changed. Returns false without
// modifying sf if a change lacks exact old/new values or does not match sf.
bool update_standard_form_bounds_incremental(
    StandardFormLP& sf,
    const LPModel& lp,
    const std::vector<BoundChangeInfo>& changes);

// Exact rollback guard for short-lived bound probes. Only the original-bound
// scalars and RHS entries touched by lower-bound shifts are snapshotted; the
// standard-form matrix and all unaffected vectors remain shared in place.
class StandardFormBoundTransaction {
 public:
  explicit StandardFormBoundTransaction(StandardFormLP& sf) noexcept
      : sf_(&sf) {}
  ~StandardFormBoundTransaction() { (void)rollback(); }

  StandardFormBoundTransaction(const StandardFormBoundTransaction&) = delete;
  StandardFormBoundTransaction& operator=(
      const StandardFormBoundTransaction&) = delete;
  StandardFormBoundTransaction(StandardFormBoundTransaction&&) = delete;
  StandardFormBoundTransaction& operator=(
      StandardFormBoundTransaction&&) = delete;

  bool apply(const LPModel& lp,
             const std::vector<BoundChangeInfo>& changes);
  bool rollback() noexcept;
  bool active() const noexcept { return active_; }
  std::size_t snapshot_value_count() const noexcept {
    return variable_snapshots_.size() * 2 + row_snapshots_.size() +
           (active_ ? 1 : 0);
  }

 private:
  struct VariableSnapshot {
    int column{-1};
    double lower_shift{0.0};
    double upper{0.0};
  };
  struct RowSnapshot {
    int row{-1};
    double rhs{0.0};
  };

  StandardFormLP* sf_{nullptr};
  std::vector<VariableSnapshot> variable_snapshots_;
  std::vector<RowSnapshot> row_snapshots_;
  double objective_const_{0.0};
  bool used_{false};
  bool active_{false};
};

// Solve an LP from a pre-built StandardFormLP (avoids rebuilding from LPModel).
SimplexResult solve_lp_from_sf(const StandardFormLP& sf,
                               const SimplexOptions& opt = {},
                               const SimplexBasis* basis_hint = nullptr);

// Original-units feasibility audit for a standard-form solution x_std (the
// scaled standard-form variable vector, as returned in SimplexResult::x_std).
// Undoes the Ruiz row/column scaling recorded on sf, then accepts iff
//   max(row residual, bound violation) <= tol * max(1, |b_orig|_inf).
// Used as the fail-closed publication audit; exposed for tests.
bool sf_solution_residual_acceptable(const StandardFormLP& sf,
                                     const Eigen::VectorXd& x_std,
                                     double tol);

// Original-space feasibility audit for an LPModel solution x (as returned in
// SolveResult::x).  Accepts iff
//   max(row violation, bound violation) <= tol * max(1, |b|_inf, |beq|_inf).
// Used as the fail-closed publication audit; exposed for tests.
bool lp_solution_residual_acceptable(const LPModel& lp,
                                     const Eigen::VectorXd& x,
                                     double tol);

/// ABI-compatible no-op; the legacy PATH_A counters no longer exist.
void dump_solve_lp_counters();

// Perform BTRAN (B^{-T} * rhs) using a type-erased SparseBasis.
// Returns zero vector if cached_sparse_basis is null.
Eigen::VectorXd sparse_basis_btran(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                                   const Eigen::VectorXd& rhs);

/// Return row i of B^{-1}; this is the HiGHS rowEp object used by tableau
/// separation before row aggregation.
bool sparse_basis_inverse_row(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                              int row,
                              Eigen::VectorXd& out);

bool sparse_basis_inverse_row_sparse_entries(
  const std::shared_ptr<BasisOps>& cached_sparse_basis,
  int row,
  std::vector<std::pair<int, double>>& out);

/// Return a tableau row e_i^T B^{-1} A in the owning StandardFormLP space.
bool sparse_basis_tableau_row(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                              int row,
                              Eigen::RowVectorXd& out);

/// Clear accumulated eta vectors from a type-erased SparseBasis, restoring
/// it to the state at its last refactorization.  Used by root probing to
/// keep the SparseBasis clean between probes so each probe can warm-start
/// without re-factorization.
void clear_sparse_basis_etas(const std::shared_ptr<BasisOps>& sb);

/// Truncate eta vectors in a type-erased SparseBasis back to a saved count.
/// Used between sibling node solves: save the eta count before child1's solve,
/// then truncate back to that count before child2's solve, so child2 sees the
/// same clean factorization state that child1 started from.
void truncate_sparse_basis_etas(const std::shared_ptr<BasisOps>& sb, int target_count);

/// Return the current eta count of a type-erased SparseBasis.
/// Returns 0 if sb is null.
int sparse_basis_eta_count(const std::shared_ptr<BasisOps>& sb);

/// Return the refactorization generation counter of a type-erased SparseBasis.
/// Returns -1 if sb is null.  Used to detect mid-solve refactorizations:
/// if the generation changes across a solve, clearing etas is unsafe.
int sparse_basis_generation(const std::shared_ptr<BasisOps>& sb);

/// Return the current factor/update telemetry of a type-erased SparseBasis.
/// Returns default-initialized telemetry when sb is null.
SparseFactorTelemetry get_sparse_basis_factor_telemetry(
  const std::shared_ptr<BasisOps>& sb);

/// Rebind a type-erased SparseBasis to a StandardFormLP matrix that is owned
/// by a persistent SimplexResult/form object.  Cached SparseBasis instances
/// store a non-owning matrix reference; callers that persist a simplex result
/// after solving from a stack-local StandardFormLP must rebind before carrying
/// the factor into the next warm start.
void rebind_sparse_basis_matrix(const std::shared_ptr<BasisOps>& sb,
                                const StandardColumnMatrix& A);

/// Return true iff the cached SparseBasis is currently bound to this exact
/// matrix object and has matching row dimension.
bool sparse_basis_bound_to_matrix(const std::shared_ptr<BasisOps>& sb,
                                  const StandardColumnMatrix& A);

}  // namespace mipsolvers::engine
