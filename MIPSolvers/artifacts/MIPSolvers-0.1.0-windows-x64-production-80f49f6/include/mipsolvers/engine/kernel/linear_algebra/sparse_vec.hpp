#pragma once
// ═══════════════════════════════════════════════════════════════════════════
// SparseVec: Scatter-gather sparse vector for simplex linear algebra kernel.
//
// Layout: dense value array (O(1) random access) + packed nonzero index list
// (O(nnz) iteration).  This is the standard design used in HiGHS/CLP/GLPK.
// ═══════════════════════════════════════════════════════════════════════════

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace mipsolvers::engine {

struct SparseVec {
  int dim = 0;
  std::vector<int> nz;         // nonzero indices
  std::vector<double> val;     // dense values (size = dim)
  std::vector<char> has;       // membership flags (size = dim)

  SparseVec() = default;
  explicit SparseVec(int d) : dim(d), val(d, 0.0), has(d, 0) { nz.reserve(d / 4); }

  void resize(int d) {
    dim = d;
    val.assign(d, 0.0);
    has.assign(d, 0);
    nz.clear();
    nz.reserve(d / 4);
  }

  /// Clear to zero in O(nnz).
  void clear() {
    for (int i : nz) { val[i] = 0.0; has[i] = 0; }
    nz.clear();
  }

  /// Set val[i] = v, tracking nonzero index.
  void scatter(int i, double v) {
    val[i] = v;
    if (v != 0.0 && !has[i]) { has[i] = 1; nz.push_back(i); }
  }

  /// Add v to val[i], tracking nonzero index.
  void scatter_add(int i, double v) {
    val[i] += v;
    if (!has[i]) { has[i] = 1; nz.push_back(i); }
  }

  /// Drop entries below tolerance, compact nz list.
  void gather_drop(double tol) {
    int w = 0;
    for (int k = 0, sz = static_cast<int>(nz.size()); k < sz; ++k) {
      const int i = nz[k];
      if (std::abs(val[i]) > tol) {
        nz[w++] = i;
      } else {
        val[i] = 0.0;
        has[i] = 0;
      }
    }
    nz.resize(w);
  }

  /// Sparse axpy: this += alpha * x.
  void axpy(double alpha, const SparseVec& x) {
    for (int i : x.nz) {
      val[i] += alpha * x.val[i];
      if (!has[i]) { has[i] = 1; nz.push_back(i); }
    }
  }

  int nnz() const { return static_cast<int>(nz.size()); }
  double density() const { return dim > 0 ? static_cast<double>(nz.size()) / dim : 0.0; }
};

}  // namespace mipsolvers::engine
