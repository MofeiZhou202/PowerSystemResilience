#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

namespace mipsolvers::engine {

namespace {

bool same_sparse_pattern(const Eigen::SparseMatrix<double>& matrix,
                         int cached_rows,
                         const std::vector<int>& outer,
                         const std::vector<int>& inner) {
  if (cached_rows != matrix.rows() ||
      outer.size() != static_cast<size_t>(matrix.outerSize() + 1) ||
      inner.size() != static_cast<size_t>(matrix.nonZeros())) {
    return false;
  }
  for (int i = 0; i <= matrix.outerSize(); ++i) {
    if (outer[static_cast<size_t>(i)] != matrix.outerIndexPtr()[i]) {
      return false;
    }
  }
  for (int i = 0; i < matrix.nonZeros(); ++i) {
    if (inner[static_cast<size_t>(i)] != matrix.innerIndexPtr()[i]) {
      return false;
    }
  }
  return true;
}

void remember_sparse_pattern(const Eigen::SparseMatrix<double>& matrix,
                             std::vector<int>& outer,
                             std::vector<int>& inner) {
  outer.assign(matrix.outerIndexPtr(),
               matrix.outerIndexPtr() + matrix.outerSize() + 1);
  inner.assign(matrix.innerIndexPtr(),
               matrix.innerIndexPtr() + matrix.nonZeros());
}

bool factor_cached_sparse_matrix(SparseKKTCache& cache,
                                 Eigen::SparseMatrix<double> matrix,
                                 int n,
                                 int meq) {
  matrix.makeCompressed();
  cache.kkt = std::move(matrix);
  return factor_current_kkt(cache, n, meq);
}

Eigen::SparseMatrix<double> assemble_augmented_kkt(
    const Eigen::SparseMatrix<double>& w,
    const Eigen::SparseMatrix<double>& jg,
    double delta_w,
    double delta_c) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  const int dim = n + meq;
  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(w.nonZeros() + 2 * jg.nonZeros() + dim));

  for (int col = 0; col < w.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(w, col); it; ++it) {
      tri.emplace_back(it.row(), it.col(), it.value());
    }
  }
  for (int i = 0; i < n; ++i) {
    tri.emplace_back(i, i, delta_w);
  }
  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      tri.emplace_back(it.col(), n + it.row(), it.value());
      tri.emplace_back(n + it.row(), it.col(), it.value());
    }
  }
  for (int i = 0; i < meq; ++i) {
    tri.emplace_back(n + i, n + i, -delta_c);
  }

  Eigen::SparseMatrix<double> kkt(dim, dim);
  kkt.setFromTriplets(tri.begin(), tri.end());
  kkt.makeCompressed();
  return kkt;
}

// ── Analyze-once assembly for the augmented KKT structure ───────────────────
// The IPM rebuilds [W + δ_W I, Jg'; Jg, -δ_C I] every iteration (and several
// times per iteration in the inertia-correction retry loop).  The sparsity
// pattern is invariant per problem, so the triplet assembly is replaced by a
// precomputed scatter map: slow path builds the pattern once and records, for
// every source entry, its position in the KKT value array; the fast path then
// refills values in O(nnz) with no allocation.

// Compute scatter positions of every (W, Jg) entry inside cache.kkt's CSC
// value array (pattern must already hold the augmented structure).
void build_augmented_scatter(SparseKKTCache& cache,
                             const Eigen::SparseMatrix<double>& w,
                             const Eigen::SparseMatrix<double>& jg) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  const int* ko = cache.kkt.outerIndexPtr();
  const int* ki = cache.kkt.innerIndexPtr();
  // Position of (row, col) in the KKT CSC arrays (entry must exist).
  auto find = [&](int col, int row) {
    const int* b = ki + ko[col];
    const int* e = ki + ko[col + 1];
    const int* p = std::lower_bound(b, e, row);
    return static_cast<int>(p - ki);
  };

  // Direct CSC loops: p is the storage offset in the source value array
  // (InnerIterator::index() would be the ROW index, not the offset).
  cache.asm_w_pos.resize(static_cast<size_t>(w.nonZeros()));
  for (int j = 0; j < w.outerSize(); ++j)
    for (int p = w.outerIndexPtr()[j]; p < w.outerIndexPtr()[j + 1]; ++p)
      cache.asm_w_pos[static_cast<size_t>(p)] =
          find(j, w.innerIndexPtr()[p]);

  cache.asm_jg_top.resize(static_cast<size_t>(jg.nonZeros()));
  cache.asm_jg_bot.resize(static_cast<size_t>(jg.nonZeros()));
  for (int j = 0; j < jg.outerSize(); ++j)
    for (int p = jg.outerIndexPtr()[j]; p < jg.outerIndexPtr()[j + 1]; ++p) {
      cache.asm_jg_top[static_cast<size_t>(p)] = find(j, n + jg.innerIndexPtr()[p]);
      cache.asm_jg_bot[static_cast<size_t>(p)] = find(n + jg.innerIndexPtr()[p], j);
    }

  cache.asm_diag_w.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i)
    cache.asm_diag_w[static_cast<size_t>(i)] = find(i, i);
  cache.asm_diag_c.resize(static_cast<size_t>(meq));
  for (int i = 0; i < meq; ++i)
    cache.asm_diag_c[static_cast<size_t>(i)] = find(n + i, n + i);

  // Structure fingerprints for change detection on later calls.
  cache.asm_w_nnz = static_cast<int>(w.nonZeros());
  cache.asm_jg_nnz = static_cast<int>(jg.nonZeros());
  cache.asm_w_outer.assign(w.outerIndexPtr(), w.outerIndexPtr() + w.outerSize() + 1);
  cache.asm_w_inner.assign(w.innerIndexPtr(), w.innerIndexPtr() + w.nonZeros());
  cache.asm_jg_outer.assign(jg.outerIndexPtr(), jg.outerIndexPtr() + jg.outerSize() + 1);
  cache.asm_jg_inner.assign(jg.innerIndexPtr(), jg.innerIndexPtr() + jg.nonZeros());
}

bool augmented_structure_matches(const SparseKKTCache& cache,
                                 const Eigen::SparseMatrix<double>& w,
                                 const Eigen::SparseMatrix<double>& jg) {
  const int w_nnz = static_cast<int>(w.nonZeros());
  const int jg_nnz = static_cast<int>(jg.nonZeros());
  return cache.asm_w_nnz == w_nnz && cache.asm_jg_nnz == jg_nnz &&
         cache.asm_w_outer.size() == static_cast<size_t>(w.outerSize() + 1) &&
         cache.asm_jg_outer.size() == static_cast<size_t>(jg.outerSize() + 1) &&
         std::memcmp(cache.asm_w_outer.data(), w.outerIndexPtr(),
                     (w.outerSize() + 1) * sizeof(int)) == 0 &&
         std::memcmp(cache.asm_jg_outer.data(), jg.outerIndexPtr(),
                     (jg.outerSize() + 1) * sizeof(int)) == 0 &&
         std::memcmp(cache.asm_w_inner.data(), w.innerIndexPtr(),
                     w_nnz * sizeof(int)) == 0 &&
         std::memcmp(cache.asm_jg_inner.data(), jg.innerIndexPtr(),
                     jg_nnz * sizeof(int)) == 0;
}

// Assemble [W + δ_W I, Jg'; Jg, -δ_C I] into cache.kkt, reusing the cached
// scatter map whenever the (w, jg) structure is unchanged.
void assemble_augmented_kkt_cached(SparseKKTCache& cache,
                                   const Eigen::SparseMatrix<double>& w,
                                   const Eigen::SparseMatrix<double>& jg,
                                   double delta_w, double delta_c) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  if (!augmented_structure_matches(cache, w, jg)) {
    // Slow path: triplet-assemble once to obtain the pattern, then build
    // the scatter map for subsequent fast refills.
    cache.kkt = assemble_augmented_kkt(w, jg, 0.0, 0.0);
    build_augmented_scatter(cache, w, jg);
  }
  double* v = cache.kkt.valuePtr();
  std::memset(v, 0, static_cast<size_t>(cache.kkt.nonZeros()) * sizeof(double));
  const double* wv = w.valuePtr();
  const int w_nnz = static_cast<int>(w.nonZeros());
  for (int k = 0; k < w_nnz; ++k)
    v[cache.asm_w_pos[static_cast<size_t>(k)]] += wv[k];
  const double* jv = jg.valuePtr();
  const int jg_nnz = static_cast<int>(jg.nonZeros());
  for (int k = 0; k < jg_nnz; ++k) {
    v[cache.asm_jg_top[static_cast<size_t>(k)]] += jv[k];
    v[cache.asm_jg_bot[static_cast<size_t>(k)]] += jv[k];
  }
  for (int i = 0; i < n; ++i)
    v[cache.asm_diag_w[static_cast<size_t>(i)]] += delta_w;
  for (int i = 0; i < meq; ++i)
    v[cache.asm_diag_c[static_cast<size_t>(i)]] -= delta_c;
}

}  // namespace

// Factor cache.kkt as currently assembled (pattern re-analysis only when the
// exact compressed structure changed).  Used by cached-assembly paths that
// keep cache.kkt resident across calls (kkt scatter assembly here and the
// augmented Newton assembler in ipm_solver.cpp).
bool factor_current_kkt(SparseKKTCache& cache, int n, int meq) {
  cache.n = n;
  cache.meq = meq;
  cache.kkt_orig = cache.kkt;

  if (!cache.solver) {
    cache.solver = make_default_sparse_solver();
    cache.pattern_analyzed = false;
  }
  const bool pattern_changed =
      !cache.pattern_analyzed ||
      !same_sparse_pattern(cache.kkt, cache.dim, cache.pattern_outer,
                           cache.pattern_inner);
  if (pattern_changed) {
    cache.solver->analyze_pattern(cache.kkt);
    remember_sparse_pattern(cache.kkt, cache.pattern_outer,
                            cache.pattern_inner);
    cache.pattern_analyzed = true;
    ++cache.symbolic_analyses;
  }
  if (cache.kkt.rows() > std::numeric_limits<int>::max() ||
      cache.kkt.nonZeros() > std::numeric_limits<int>::max()) {
    // The solver backend is int32-indexed: fail loudly rather than
    // silently truncating the dimension/nnz counters to int.
    cache.factored = false;
    return false;
  }
  cache.dim = static_cast<int>(cache.kkt.rows());
  cache.nnz = static_cast<int>(cache.kkt.nonZeros());
  ++cache.numeric_factorizations;
  if (!cache.solver->factorize(cache.kkt)) {
    cache.factored = false;
    return false;
  }
  cache.factored = true;
  return true;
}

bool factor_kkt_sparse(SparseKKTCache& cache,
                       const Eigen::SparseMatrix<double>& w,
                       const Eigen::SparseMatrix<double>& jg,
                       double reg) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  assemble_augmented_kkt_cached(cache, w, jg, reg, reg);
  return factor_current_kkt(cache, n, meq);
}

bool solve_kkt_sparse(SparseKKTCache& cache,
                      const Eigen::VectorXd& rhs,
                      Eigen::VectorXd& dx,
                      Eigen::VectorXd& dlambda) {
  if (!cache.factored) {
    return false;
  }
  Eigen::VectorXd sol;
  if (!cache.solver || !cache.solver->solve(rhs, sol) || !sol.allFinite()) {
    return false;
  }
  ++cache.linear_solves;
  for (int ref = 0; ref < 2; ++ref) {
    Eigen::VectorXd residual = rhs - cache.kkt_orig * sol;
    if (residual.cwiseAbs().maxCoeff() < 1e-14 * std::max(1.0, rhs.cwiseAbs().maxCoeff())) {
      break;
    }
    Eigen::VectorXd correction;
    if (!cache.solver->solve(residual, correction) || !correction.allFinite()) {
      break;
    }
    ++cache.linear_solves;
    sol += correction;
  }
  dx = sol.head(cache.n);
  dlambda = sol.tail(cache.meq);
  return true;
}

bool solve_kkt_reduced_sparse(const Eigen::SparseMatrix<double>& w,
                              const Eigen::SparseMatrix<double>& jg,
                              const Eigen::VectorXd& rhs,
                              Eigen::VectorXd& dx,
                              Eigen::VectorXd& dlambda,
                              int max_eq_dim,
                              double min_reg,
                              double max_reg) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  if (meq <= 0 || meq > max_eq_dim) {
    return false;
  }

  const Eigen::SparseMatrix<double> jgt = jg.transpose();
  for (double reg = min_reg; reg <= max_reg; reg *= 10.0) {
    Eigen::SparseMatrix<double> w_reg = w;
    w_reg.reserve(w.nonZeros() + n);
    for (int i = 0; i < n; ++i) {
      w_reg.coeffRef(i, i) += reg;
    }
    w_reg.makeCompressed();

    auto solver = make_default_sparse_solver();
    solver->analyze_pattern(w_reg);
    if (!solver->factorize(w_reg)) {
      continue;
    }

    const Eigen::VectorXd rhs_x = rhs.head(n);
    const Eigen::VectorXd rhs_eq = rhs.tail(meq);
    Eigen::VectorXd z_rhs;
    if (!solver->solve(rhs_x, z_rhs) || !z_rhs.allFinite()) {
      continue;
    }

    Eigen::MatrixXd z_cols = Eigen::MatrixXd::Zero(n, meq);
    bool column_failure = false;
    for (int i = 0; i < meq; ++i) {
      Eigen::VectorXd col_rhs = Eigen::VectorXd::Zero(n);
      for (Eigen::SparseMatrix<double>::InnerIterator it(jgt, i); it; ++it) {
        col_rhs[it.row()] = it.value();
      }
      Eigen::VectorXd z_col;
      if (!solver->solve(col_rhs, z_col) || !z_col.allFinite()) {
        column_failure = true;
        break;
      }
      z_cols.col(i) = z_col;
    }
    if (column_failure) {
      continue;
    }

    Eigen::MatrixXd schur = Eigen::MatrixXd(jg * z_cols);
    schur.diagonal().array() += reg;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(schur);
    if (ldlt.info() != Eigen::Success) {
      continue;
    }

    dlambda = ldlt.solve(Eigen::VectorXd(jg * z_rhs) - rhs_eq);
    if (!dlambda.allFinite()) {
      continue;
    }
    dx = z_rhs - z_cols * dlambda;
    if (dx.allFinite()) {
      return true;
    }
  }
  return false;
}

namespace {

#ifndef HACDCPF_HAVE_MUMPS
Eigen::SparseMatrix<double> add_scaled_identity(
    const Eigen::SparseMatrix<double>& a, double delta) {
  const int n = static_cast<int>(a.rows());
  Eigen::SparseMatrix<double> out = a;
  out.makeCompressed();
  if (delta == 0.0) return out;
  for (int i = 0; i < n; ++i) {
    out.coeffRef(i, i) += delta;
  }
  out.makeCompressed();
  return out;
}
#endif

double increased_primal_regularization(double delta_w,
                                       bool& delta_w_was_zero,
                                       const InertiaSettings& settings) {
  if (delta_w_was_zero) {
    delta_w_was_zero = false;
    return settings.delta_w_0;
  }
  const double multiplier = (delta_w < settings.delta_w_0 * 1.1)
      ? settings.kappa_w_plus_first
      : settings.kappa_w_plus;
  return std::max(delta_w * multiplier, settings.delta_w_0);
}

#ifndef HACDCPF_HAVE_MUMPS
bool certify_primal_positive_definite(
    const Eigen::SparseMatrix<double>& h_delta,
    SparseInertiaKKTCache& cache) {
  const bool pattern_changed =
      !cache.primal_pattern_analyzed ||
      !same_sparse_pattern(h_delta, cache.primal_rows,
                           cache.primal_pattern_outer,
                           cache.primal_pattern_inner);
  if (pattern_changed) {
    cache.primal_ldlt.analyzePattern(h_delta);
    remember_sparse_pattern(h_delta, cache.primal_pattern_outer,
                            cache.primal_pattern_inner);
    cache.primal_rows = static_cast<int>(h_delta.rows());
    cache.primal_pattern_analyzed = true;
    ++cache.primal_symbolic_analyses;
  }
  cache.primal_ldlt.factorize(h_delta);
  ++cache.primal_numeric_factorizations;
  if (cache.primal_ldlt.info() != Eigen::Success) {
    return false;
  }
  const Eigen::VectorXd diagonal = cache.primal_ldlt.vectorD();
  return (diagonal.array() > 0.0).all();
}
#endif

void remember_tangent_partition_pattern(
    const Eigen::SparseMatrix<double>& jg,
    SparseInertiaKKTCache& cache) {
  remember_sparse_pattern(jg, cache.equality_pattern_outer,
                          cache.equality_pattern_inner);
  cache.equality_rows = static_cast<int>(jg.rows());
  cache.tangent_partition_valid = true;
  cache.tangent_basis = SparseKKTCache{};
  ++cache.tangent_partition_analyses;
}

void select_natural_tangent_partition(
    const Eigen::SparseMatrix<double>& jg,
    SparseInertiaKKTCache& cache) {
  const int meq = static_cast<int>(jg.rows());
  const int n = static_cast<int>(jg.cols());
  cache.basic_columns.resize(static_cast<size_t>(meq));
  cache.free_columns.resize(static_cast<size_t>(n - meq));
  for (int col = 0; col < meq; ++col) {
    cache.basic_columns[static_cast<size_t>(col)] = col;
  }
  for (int col = meq; col < n; ++col) {
    cache.free_columns[static_cast<size_t>(col - meq)] = col;
  }
  cache.tangent_partition_strategy = 2;
  remember_tangent_partition_pattern(jg, cache);
}

bool select_preferred_tangent_partition(
    const Eigen::SparseMatrix<double>& jg,
    SparseInertiaKKTCache& cache) {
  const int meq = static_cast<int>(jg.rows());
  const int n = static_cast<int>(jg.cols());
  const int nullity = n - meq;
  if (static_cast<int>(cache.preferred_free_columns.size()) != nullity) {
    return false;
  }
  std::vector<bool> is_free(static_cast<size_t>(n), false);
  for (int col : cache.preferred_free_columns) {
    if (col < 0 || col >= n || is_free[static_cast<size_t>(col)]) {
      return false;
    }
    is_free[static_cast<size_t>(col)] = true;
  }
  cache.free_columns = cache.preferred_free_columns;
  cache.basic_columns.clear();
  cache.basic_columns.reserve(static_cast<size_t>(meq));
  for (int col = 0; col < n; ++col) {
    if (!is_free[static_cast<size_t>(col)]) {
      cache.basic_columns.push_back(col);
    }
  }
  if (static_cast<int>(cache.basic_columns.size()) != meq) return false;
  cache.tangent_partition_strategy = 1;
  remember_tangent_partition_pattern(jg, cache);
  return true;
}

bool select_qr_tangent_partition(
    const Eigen::SparseMatrix<double>& jg,
    SparseInertiaKKTCache& cache) {
  const int meq = static_cast<int>(jg.rows());
  const int n = static_cast<int>(jg.cols());
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
  qr.compute(jg);
  if (qr.info() != Eigen::Success || qr.rank() != meq) {
    cache.tangent_partition_valid = false;
    return false;
  }

  const auto& permutation = qr.colsPermutation().indices();
  if (permutation.size() != n) {
    cache.tangent_partition_valid = false;
    return false;
  }
  cache.basic_columns.resize(static_cast<size_t>(meq));
  cache.free_columns.resize(static_cast<size_t>(n - meq));
  for (int col = 0; col < meq; ++col) {
    cache.basic_columns[static_cast<size_t>(col)] = permutation[col];
  }
  for (int col = meq; col < n; ++col) {
    cache.free_columns[static_cast<size_t>(col - meq)] = permutation[col];
  }
  cache.tangent_partition_strategy = 3;
  remember_tangent_partition_pattern(jg, cache);
  return true;
}

Eigen::SparseMatrix<double> extract_basic_jacobian(
    const Eigen::SparseMatrix<double>& jg,
    const std::vector<int>& basic_columns) {
  const int meq = static_cast<int>(jg.rows());
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<size_t>(jg.nonZeros()));
  for (int basic = 0; basic < static_cast<int>(basic_columns.size()); ++basic) {
    const int source_col = basic_columns[static_cast<size_t>(basic)];
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, source_col); it; ++it) {
      trips.emplace_back(it.row(), basic, it.value());
    }
  }
  Eigen::SparseMatrix<double> basis(meq, meq);
  basis.setFromTriplets(trips.begin(), trips.end());
  basis.makeCompressed();
  return basis;
}

bool factor_tangent_basis(const Eigen::SparseMatrix<double>& jg,
                          SparseInertiaKKTCache& cache) {
  const bool equality_pattern_changed =
      !cache.tangent_partition_valid ||
      !same_sparse_pattern(jg, cache.equality_rows,
                           cache.equality_pattern_outer,
                           cache.equality_pattern_inner);
  if (equality_pattern_changed) {
    // Most NLP models order state variables before a small control/interface
    // block. Try that physically meaningful partition first and validate it by
    // numerical factorization. Rank-revealing sparse QR remains a fallback.
    if (!select_preferred_tangent_partition(jg, cache)) {
      select_natural_tangent_partition(jg, cache);
    }
  }

  Eigen::SparseMatrix<double> basis =
      extract_basic_jacobian(jg, cache.basic_columns);
  if (factor_cached_sparse_matrix(cache.tangent_basis, basis,
                                  static_cast<int>(jg.rows()), 0)) {
    return true;
  }

  if (cache.tangent_partition_strategy == 1) {
    select_natural_tangent_partition(jg, cache);
    basis = extract_basic_jacobian(jg, cache.basic_columns);
    if (factor_cached_sparse_matrix(cache.tangent_basis, basis,
                                    static_cast<int>(jg.rows()), 0)) {
      return true;
    }
  }

  // A numerically singular cached state block can occur even when the exact
  // Jacobian pattern is unchanged. Re-pivot numerically once before giving up.
  if (!select_qr_tangent_partition(jg, cache)) return false;
  basis = extract_basic_jacobian(jg, cache.basic_columns);
  return factor_cached_sparse_matrix(cache.tangent_basis, basis,
                                     static_cast<int>(jg.rows()), 0);
}

struct ReducedSpaceCertificate {
  double min_curvature{0.0};
  double margin{0.0};
  double nullspace_residual{0.0};
  int dimension{0};
};

bool build_reduced_space_certificate(
    const Eigen::SparseMatrix<double>& w,
    const Eigen::SparseMatrix<double>& jg,
    const InertiaSettings& settings,
    SparseInertiaKKTCache& cache,
    ReducedSpaceCertificate& certificate) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  const int nullity = n - meq;
  certificate = ReducedSpaceCertificate{};
  certificate.dimension = nullity;
  if (meq < 0 || nullity < 0 ||
      nullity > settings.max_tangent_dimension) {
    return false;
  }
  if (nullity == 0) {
    if (!factor_tangent_basis(jg, cache)) return false;
    certificate.margin = settings.reduced_curvature_tolerance;
    certificate.min_curvature = certificate.margin;
    return true;
  }

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n, nullity);
  if (meq == 0) {
    z.setIdentity();
  } else {
    if (!factor_tangent_basis(jg, cache)) return false;
    for (int tangent = 0; tangent < nullity; ++tangent) {
      const int free_col = cache.free_columns[static_cast<size_t>(tangent)];
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(meq);
      for (Eigen::SparseMatrix<double>::InnerIterator it(jg, free_col); it; ++it) {
        rhs[it.row()] = -it.value();
      }
      Eigen::VectorXd basic_values;
      Eigen::VectorXd unused;
      if (!solve_kkt_sparse(cache.tangent_basis, rhs,
                            basic_values, unused)) {
        return false;
      }
      for (int basic = 0; basic < meq; ++basic) {
        z(cache.basic_columns[static_cast<size_t>(basic)], tangent) =
            basic_values[basic];
      }
      z(free_col, tangent) = 1.0;
    }
  }

  const Eigen::MatrixXd null_residual = jg * z;
  certificate.nullspace_residual = null_residual.size() == 0
      ? 0.0 : null_residual.cwiseAbs().maxCoeff();
  Eigen::VectorXd jacobian_row_sum = Eigen::VectorXd::Zero(meq);
  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      jacobian_row_sum[it.row()] += std::abs(it.value());
    }
  }
  const double jacobian_inf = meq == 0 ? 0.0 : jacobian_row_sum.maxCoeff();
  const double z_inf = z.cwiseAbs().rowwise().sum().maxCoeff();
  const double residual_limit = settings.nullspace_residual_tolerance *
      std::max(1.0, jacobian_inf * z_inf);
  if (!z.allFinite() || !std::isfinite(certificate.nullspace_residual) ||
      certificate.nullspace_residual > residual_limit) {
    return false;
  }

  Eigen::MatrixXd reduced = z.transpose() * (w * z);
  reduced = 0.5 * (reduced + reduced.transpose());
  Eigen::MatrixXd metric = z.transpose() * z;
  metric = 0.5 * (metric + metric.transpose());
  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver;
  eigensolver.compute(reduced, metric, Eigen::EigenvaluesOnly);
  if (eigensolver.info() != Eigen::Success ||
      !eigensolver.eigenvalues().allFinite()) {
    return false;
  }
  certificate.min_curvature = eigensolver.eigenvalues().minCoeff();
  const double spectral_scale = std::max(
      1.0, eigensolver.eigenvalues().cwiseAbs().maxCoeff());
  // A symmetric eigensolve is backward stable: the absolute eigenvalue error
  // is O(d * eps * ||C||_2) for the d-dimensional orthonormalized projected
  // problem. Use that bound, rather than a fixed relative gap, so a large
  // positive barrier eigenvalue cannot manufacture a large primal shift while
  // the smallest tangent eigenvalue remains safely positive.
  constexpr double kDenseEigenRoundoffMultiplier = 64.0;
  const double roundoff_margin = kDenseEigenRoundoffMultiplier *
      std::max(1, nullity) * std::numeric_limits<double>::epsilon() *
      spectral_scale;
  certificate.margin = std::max(
      settings.reduced_curvature_tolerance, roundoff_margin);
  return true;
}

#ifdef HACDCPF_HAVE_MUMPS
void ensure_mumps_augmented_solver(SparseKKTCache& augmented) {
  if (dynamic_cast<MumpsSolver*>(augmented.solver.get()) != nullptr) return;
  augmented.solver = std::make_unique<MumpsSolver>();
  augmented.pattern_analyzed = false;
  augmented.pattern_outer.clear();
  augmented.pattern_inner.clear();
}

bool factor_with_direct_mumps_inertia(
    const Eigen::SparseMatrix<double>& w,
    const Eigen::SparseMatrix<double>& jg,
    const InertiaSettings& settings,
    double& delta_w_last,
    SparseInertiaKKTCache& cache,
    InertiaStatus& status) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  const int dim = n + meq;

  ensure_mumps_augmented_solver(cache.augmented);
  status.direct_factor_inertia = true;

  double delta_w = std::max(0.0, delta_w_last);
  bool delta_w_was_zero = (delta_w == 0.0);
  const double delta_c_repair = settings.delta_c_stripe *
      std::pow(std::max(settings.mu, 1e-20), 0.25);
  bool last_factor_succeeded = false;

  auto factor_and_check = [&](double delta_c) {
    assemble_augmented_kkt_cached(cache.augmented, w, jg, delta_w, delta_c);
    ++status.factorization_attempts;
    const bool factored = factor_current_kkt(cache.augmented, n, meq);
    last_factor_succeeded = factored;
    const int deficiency = cache.augmented.solver->estimated_deficiency();
    const int negative = cache.augmented.solver->negative_eigenvalues();
    status.n_zero = std::max(0, deficiency);
    status.n_neg = std::max(0, negative);
    status.n_pos = (negative >= 0 && deficiency >= 0)
        ? std::max(0, dim - negative - deficiency) : 0;
    status.delta_w_used = delta_w;
    status.delta_c_used = delta_c;
    return factored && deficiency == 0 && negative == meq;
  };

  while (status.factorization_attempts < 60) {
    if (factor_and_check(0.0) ||
        ((status.n_zero > 0 || !last_factor_succeeded) &&
         factor_and_check(delta_c_repair))) {
      cache.factored = true;
      status.correct = true;
      status.n_pos = n;
      status.n_neg = meq;
      status.n_zero = 0;
      delta_w_last = std::max(
          settings.delta_w_min, delta_w * settings.kappa_w_minus);
      return true;
    }

    cache.factored = false;
    cache.augmented.factored = false;
    delta_w = increased_primal_regularization(
        delta_w, delta_w_was_zero, settings);
    if (delta_w > settings.delta_w_max) {
      status.delta_w_used = delta_w;
      return false;
    }
  }
  return false;
}
#endif

}  // namespace

bool factor_kkt_inertia_corrected_sparse(
    const Eigen::SparseMatrix<double>& w,
    const Eigen::SparseMatrix<double>& jg,
    const InertiaSettings& settings,
    double& delta_w_last,
    SparseInertiaKKTCache& cache,
    InertiaStatus& status) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  status = InertiaStatus{};
  cache.factored = false;
  if (w.cols() != n || jg.cols() != n) return false;

#ifdef HACDCPF_HAVE_MUMPS
  // The inertia-corrected path is symmetric by construction. Use the
  // symmetric-indefinite LDLT backend even when a small reduced-space
  // certificate supplies the regularization threshold; generic KLU/UMFPACK
  // would discard symmetry and roughly double fill/flops.
  ensure_mumps_augmented_solver(cache.augmented);
#endif

  // For full-row-rank Jg, the bordered-Hessian identity gives
  // inertia(K) = (meq, meq, 0) + inertia(Z' (W + delta_W I) Z).
  // The generalized eigenvalues of (Z'WZ, Z'Z) therefore provide the exact
  // scalar regularization threshold on the tangent space. This avoids forcing
  // negative curvature in constrained normal directions to become positive.
  ReducedSpaceCertificate reduced_certificate;
  if (build_reduced_space_certificate(
          w, jg, settings, cache, reduced_certificate)) {
    status.reduced_space_certificate = true;
    status.tangent_dimension = reduced_certificate.dimension;
    status.min_reduced_curvature = reduced_certificate.min_curvature;
    status.reduced_curvature_margin = reduced_certificate.margin;
    status.nullspace_residual = reduced_certificate.nullspace_residual;
    const double required_shift = std::max(
        0.0, reduced_certificate.margin - reduced_certificate.min_curvature);
    // A mathematically positive O(eps) pivot is still a numerical null pivot
    // to a sparse threshold factorization. When a shift is required, place it
    // safely inside the positive half-line at sqrt(eps) relative scale so the
    // direct inertia check does not trigger an 8x regularization jump.
    const double factor_margin = required_shift > 0.0
        ? 64.0 * std::sqrt(std::numeric_limits<double>::epsilon()) *
              std::max(1.0, std::abs(reduced_certificate.min_curvature))
        : 0.0;
    double delta_w = required_shift + factor_margin;
    bool delta_w_was_zero = (delta_w == 0.0);
    for (; status.factorization_attempts < 8;
         ++status.factorization_attempts) {
      assemble_augmented_kkt_cached(cache.augmented, w, jg, delta_w, 0.0);
      const bool factored = factor_current_kkt(cache.augmented, n, meq);
      const int factor_negative =
          cache.augmented.solver->negative_eigenvalues();
      const int factor_deficiency =
          cache.augmented.solver->estimated_deficiency();
      const bool factor_inertia_available =
          factor_negative >= 0 && factor_deficiency >= 0;
      const bool factor_inertia_matches =
          !factor_inertia_available ||
          (factor_negative == meq && factor_deficiency == 0);
      if (factored && factor_inertia_matches) {
        cache.factored = true;
        status.correct = true;
        status.direct_factor_inertia = factor_inertia_available;
        status.n_pos = n;
        status.n_neg = meq;
        status.delta_w_used = delta_w;
        status.delta_c_used = 0.0;
        delta_w_last = std::max(
            settings.delta_w_min, delta_w * settings.kappa_w_minus);
        return true;
      }
      delta_w = increased_primal_regularization(
          delta_w, delta_w_was_zero, settings);
      if (delta_w > settings.delta_w_max) {
        status.delta_w_used = delta_w;
        return false;
      }
    }
    status.reduced_space_certificate = false;
  }

#ifdef HACDCPF_HAVE_MUMPS
  // The explicit null-space eigensolve is deliberately capped. Beyond that
  // cap, use the negative-pivot count of the symmetric-indefinite factor
  // instead of forcing the entire primal block to be positive definite.
  return factor_with_direct_mumps_inertia(
      w, jg, settings, delta_w_last, cache, status);
#else

  // Wächter-Biegler δ_W schedule: start from last successful δ_W; on failure,
  // if δ_W was zero → jump to δ_w_0; else multiply by κ_W⁺_first (first
  // repair) or κ_W⁺ (subsequent).
  double delta_w = std::max(0.0, delta_w_last);
  bool delta_w_was_zero = (delta_w == 0.0);

  for (; status.factorization_attempts < 60; ++status.factorization_attempts) {
    const Eigen::SparseMatrix<double> h_delta =
        add_scaled_identity(w, delta_w);
    if (!certify_primal_positive_definite(h_delta, cache)) {
      delta_w = increased_primal_regularization(
          delta_w, delta_w_was_zero, settings);
      if (delta_w > settings.delta_w_max) {
        status.delta_w_used = delta_w;
        status.delta_c_used = 0.0;
        return false;
      }
      continue;
    }

    status.n_pos = n;
    status.n_neg = meq;

    // With H_delta positive definite and delta_C=0, nonsingularity of the
    // augmented matrix is equivalent to full row rank of Jg. The block LDLT
    // congruence then gives inertia (n, meq, 0) without materializing the dense
    // Schur complement.
    assemble_augmented_kkt_cached(cache.augmented, w, jg, delta_w, 0.0);
    if (factor_current_kkt(cache.augmented, n, meq)) {
      cache.factored = true;
      status.correct = true;
      status.delta_w_used = delta_w;
      status.delta_c_used = 0.0;
      delta_w_last = std::max(
          settings.delta_w_min, delta_w * settings.kappa_w_minus);
      return true;
    }

    // A positive dual regularization makes
    // Jg H_delta^{-1} Jg' + delta_C I positive definite even when Jg is rank
    // deficient, so the augmented inertia remains (n, meq, 0).
    const double delta_c = settings.delta_c_stripe *
        std::pow(std::max(settings.mu, 1e-20), 0.25);
    assemble_augmented_kkt_cached(cache.augmented, w, jg, delta_w, delta_c);
    if (factor_current_kkt(cache.augmented, n, meq)) {
      cache.factored = true;
      status.correct = true;
      status.n_zero = 0;
      status.delta_w_used = delta_w;
      status.delta_c_used = delta_c;
      delta_w_last = std::max(
          settings.delta_w_min, delta_w * settings.kappa_w_minus);
      return true;
    }

    delta_w = increased_primal_regularization(
        delta_w, delta_w_was_zero, settings);
    if (delta_w > settings.delta_w_max) {
      status.delta_w_used = delta_w;
      status.delta_c_used = delta_c;
      return false;
    }
  }
  return false;
#endif
}

bool solve_kkt_inertia_corrected_sparse(
    SparseInertiaKKTCache& cache,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& dx,
    Eigen::VectorXd& dlambda) {
  if (!cache.factored) return false;
  return solve_kkt_sparse(cache.augmented, rhs, dx, dlambda);
}

bool factor_and_solve_kkt_inertia_corrected(
    const Eigen::SparseMatrix<double>& w,
    const Eigen::SparseMatrix<double>& jg,
    const Eigen::VectorXd& rhs,
    const InertiaSettings& settings,
    double& delta_w_last,
    Eigen::VectorXd& dx,
    Eigen::VectorXd& dlambda,
    InertiaStatus& status) {
  SparseInertiaKKTCache cache;
  return factor_kkt_inertia_corrected_sparse(
             w, jg, settings, delta_w_last, cache, status) &&
         solve_kkt_inertia_corrected_sparse(cache, rhs, dx, dlambda);
}

Eigen::SparseMatrix<double> assemble_primal_hessian_with_bounds(
    const Eigen::SparseMatrix<double>& hess,
    const Eigen::VectorXd& sigma_diag) {
  const int n = static_cast<int>(hess.rows());
  if (sigma_diag.size() != n) return hess;
  Eigen::SparseMatrix<double> out = hess;
  out.makeCompressed();
  for (int i = 0; i < n; ++i) {
    const double sigma_i = sigma_diag[i];
    if (sigma_i != 0.0) {
      out.coeffRef(i, i) += sigma_i;
    }
  }
  out.makeCompressed();
  return out;
}

void split_inequalities(const NLPModel& prob,
                        const Eigen::VectorXd& x,
                        const std::vector<int>& lb_cols,
                        const std::vector<int>& ub_cols,
                        Eigen::VectorXd& h,
                        Eigen::SparseMatrix<double>& jh) {
  const int n = static_cast<int>(prob.vars.size());

  Eigen::VectorXd h_nonlin;
  Eigen::SparseMatrix<double> jh_nonlin;
  if (prob.h) {
    prob.h(x, h_nonlin);
  } else {
    h_nonlin = Eigen::VectorXd::Zero(0);
  }

  const int m_nonlinear = static_cast<int>(h_nonlin.size());

  if (prob.jac_h && m_nonlinear > 0) {
    prob.jac_h(x, jh_nonlin);
  } else {
    jh_nonlin.resize(m_nonlinear, n);
    jh_nonlin.setZero();
  }

  const int m = m_nonlinear + static_cast<int>(lb_cols.size()) + static_cast<int>(ub_cols.size());
  h = Eigen::VectorXd::Zero(m);

  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(jh_nonlin.nonZeros() + lb_cols.size() + ub_cols.size()));

  if (m_nonlinear > 0) {
    h.head(m_nonlinear) = h_nonlin;
    for (int col = 0; col < jh_nonlin.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(jh_nonlin, col); it; ++it) {
        tri.emplace_back(it.row(), it.col(), it.value());
      }
    }
  }

  int row = m_nonlinear;
  for (int c : lb_cols) {
    h[row] = prob.vars[c].lb - x[c];
    tri.emplace_back(row, c, -1.0);
    ++row;
  }
  for (int c : ub_cols) {
    h[row] = x[c] - prob.vars[c].ub;
    tri.emplace_back(row, c, 1.0);
    ++row;
  }

  jh.resize(m, n);
  jh.setFromTriplets(tri.begin(), tri.end());
  jh.makeCompressed();
}

}  // namespace mipsolvers::engine
