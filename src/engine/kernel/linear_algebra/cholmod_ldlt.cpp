// ═══════════════════════════════════════════════════════════════════════════
// CholmodLDLT — implementation (SuiteSparse CHOLMOD, int64 cholmod_l_* API).
//
// Only compiled when MIPSOLVERS_HAVE_CHOLMOD is defined; the class is a
// no-op stub otherwise so call sites can be written unconditionally.
// ═══════════════════════════════════════════════════════════════════════════

#include "mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp"

#include <cstring>
#include <vector>

#ifdef MIPSOLVERS_HAVE_CHOLMOD
// SuiteSparse v7 headers are C++-safe (own extern "C" guards) — no wrapper.
#include <cholmod.h>
#endif

namespace mipsolvers::engine {

struct CholmodLDLT::Impl {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  cholmod_common c{};
  cholmod_sparse A{};        // by-value wrapper around caller-owned arrays
  cholmod_dense B{};         // by-value wrapper around the caller's rhs
  cholmod_factor* L = nullptr;
  cholmod_dense* X = nullptr;  // preallocated solution buffer
  cholmod_dense* Y = nullptr;  // solve2 workspace (allocated on first solve)
  cholmod_dense* E = nullptr;  // solve2 workspace
  std::vector<int64_t> p64, i64;
  int64_t m = 0;
  int64_t nnz = 0;
  bool started = false;
#endif
};

CholmodLDLT::CholmodLDLT() : impl_(new Impl) {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  impl_->started = cholmod_l_start(&impl_->c) != 0;
  if (impl_->started) {
    // Status is reported via return codes — keep CHOLMOD quiet on stdout.
    impl_->c.print = 0;
  }
#endif
}

CholmodLDLT::~CholmodLDLT() {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  if (impl_->started) {
    if (impl_->X) cholmod_l_free_dense(&impl_->X, &impl_->c);
    if (impl_->Y) cholmod_l_free_dense(&impl_->Y, &impl_->c);
    if (impl_->E) cholmod_l_free_dense(&impl_->E, &impl_->c);
    if (impl_->L) cholmod_l_free_factor(&impl_->L, &impl_->c);
    cholmod_l_finish(&impl_->c);
  }
#endif
  delete impl_;
}

CholmodLDLT::CholmodLDLT(CholmodLDLT&& other) noexcept : impl_(other.impl_) {
  other.impl_ = new Impl;
}

CholmodLDLT& CholmodLDLT::operator=(CholmodLDLT&& other) noexcept {
  if (this != &other) {
    this->~CholmodLDLT();
    impl_ = other.impl_;
    other.impl_ = new Impl;
  }
  return *this;
}

bool CholmodLDLT::analyze(int64_t m, const int* outer, const int* inner,
                          const double* values, int64_t nnz) {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  if (!impl_->started || m <= 0 || nnz < 0 || values == nullptr) return false;
  impl_->m = m;
  impl_->nnz = nnz;
  // One-time int32 → int64 widening of the pattern (values are never
  // touched by analyze; factorize re-reads the aliased value buffer).
  impl_->p64.assign(outer, outer + m + 1);
  impl_->i64.assign(inner, inner + nnz);

  impl_->A.nrow = static_cast<size_t>(m);
  impl_->A.ncol = static_cast<size_t>(m);
  impl_->A.nzmax = static_cast<size_t>(nnz);
  impl_->A.p = impl_->p64.data();
  impl_->A.i = impl_->i64.data();
  impl_->A.nz = nullptr;
  impl_->A.x = const_cast<double*>(values);  // required non-null for REAL
  impl_->A.z = nullptr;
  impl_->A.stype = -1;  // lower triangle stored (matches N_sparse layout)
  impl_->A.itype = CHOLMOD_LONG;
  impl_->A.xtype = CHOLMOD_REAL;
  impl_->A.dtype = CHOLMOD_DOUBLE;
  impl_->A.sorted = 1;
  impl_->A.packed = 1;

  if (impl_->L) {
    cholmod_l_free_factor(&impl_->L, &impl_->c);
    impl_->L = nullptr;
  }
  impl_->L = cholmod_l_analyze(&impl_->A, &impl_->c);
  if (!impl_->L) return false;

  // Preallocate the solution buffer; Y/E workspaces are allocated lazily
  // by solve2 on first use and reused afterwards.
  if (!impl_->X) {
    impl_->X = cholmod_l_allocate_dense(static_cast<size_t>(m), 1,
                                        static_cast<size_t>(m),
                                        CHOLMOD_REAL + CHOLMOD_DOUBLE,
                                        &impl_->c);
  }
  return impl_->X != nullptr;
#else
  (void)m; (void)outer; (void)inner; (void)nnz;
  return false;
#endif
}

bool CholmodLDLT::factorize(const double* values) {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  if (!impl_->L || values == nullptr) return false;
  impl_->A.x = const_cast<double*>(values);
  return cholmod_l_factorize(&impl_->A, impl_->L, &impl_->c) != 0 &&
         impl_->c.status == CHOLMOD_OK;
#else
  (void)values;
  return false;
#endif
}

bool CholmodLDLT::solve(const double* rhs, double* out) {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  if (!impl_->L || !impl_->X || rhs == nullptr || out == nullptr)
    return false;
  impl_->B.nrow = static_cast<size_t>(impl_->m);
  impl_->B.ncol = 1;
  impl_->B.nzmax = static_cast<size_t>(impl_->m);
  impl_->B.d = static_cast<size_t>(impl_->m);
  impl_->B.x = const_cast<double*>(rhs);
  impl_->B.z = nullptr;
  impl_->B.xtype = CHOLMOD_REAL;
  impl_->B.dtype = CHOLMOD_DOUBLE;
  const int ok = cholmod_l_solve2(CHOLMOD_A, impl_->L, &impl_->B, nullptr,
                                  &impl_->X, nullptr, &impl_->Y, &impl_->E,
                                  &impl_->c);
  if (!ok || impl_->c.status != CHOLMOD_OK) return false;
  std::memcpy(out, impl_->X->x,
              static_cast<size_t>(impl_->m) * sizeof(double));
  return true;
#else
  (void)rhs; (void)out;
  return false;
#endif
}

bool CholmodLDLT::valid() const {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  return impl_->L != nullptr;
#else
  return false;
#endif
}

void CholmodLDLT::set_simplicial(bool simplicial) {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  if (impl_->started) {
    impl_->c.supernodal = simplicial ? CHOLMOD_SIMPLICIAL : CHOLMOD_AUTO;
  }
#else
  (void)simplicial;
#endif
}

int64_t CholmodLDLT::dim() const {
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  return impl_->m;
#else
  return 0;
#endif
}

}  // namespace mipsolvers::engine
