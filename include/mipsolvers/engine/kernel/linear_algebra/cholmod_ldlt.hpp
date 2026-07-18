#pragma once

#include <cstdint>

namespace mipsolvers::engine {

/// RAII wrapper around CHOLMOD's int64 (`cholmod_l_*`) sparse Cholesky —
/// the supernodal BLAS-3 open-source backend for SPD systems.
///
/// Usage contract: analyze once per sparsity pattern, factorize per
/// numerical refresh, solve many times.  The values buffer is ALIASED
/// (zero-copy): factorize() re-reads it on every call, so the caller may
/// refill it in place between factorizations — exactly the IPM
/// normal-equations pattern (constant structure, changing values).
///
/// Compiled only when MIPSOLVERS_HAVE_CHOLMOD is defined (CMake sets it
/// when SuiteSparse CHOLMOD is detected); all members return false
/// otherwise.
class CholmodLDLT {
 public:
  CholmodLDLT();
  ~CholmodLDLT();
  CholmodLDLT(const CholmodLDLT&) = delete;
  CholmodLDLT& operator=(const CholmodLDLT&) = delete;
  CholmodLDLT(CholmodLDLT&&) noexcept;
  CholmodLDLT& operator=(CholmodLDLT&&) noexcept;

  /// Symbolic analysis of a lower-triangular CSC pattern (int32 input,
  /// widened to int64 once here).  `outer` has m+1 entries, `inner` has nnz.
  bool analyze(int64_t m, const int* outer, const int* inner, int64_t nnz);

  /// Numeric factorization of the analyzed matrix.  `values` (nnz entries,
  /// matching the analyzed pattern) is aliased, not copied; later calls
  /// re-read it.  Returns false when the matrix is not positive definite
  /// (caller should escalate diagonal regularization and retry).
  bool factorize(const double* values);

  /// Solve A x = b with the current factorization into `out` (length m).
  bool solve(const double* rhs, double* out);

  bool valid() const;
  int64_t dim() const;

 private:
  struct Impl;
  Impl* impl_;  // raw, owned via explicit ctor/dtor (keeps cholmod.h out of this header)
};

}  // namespace mipsolvers::engine
