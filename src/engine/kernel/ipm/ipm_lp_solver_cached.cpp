/// @file ipm_lp_solver_cached.cpp
/// @brief Cached repeated-node LP path for NativeIPMLPAdapter (split from ipm_lp_solver.cpp).

#include "ipm_lp_solver_internal.hpp"

namespace mipsolvers::engine {

// AUDIT-NAV: 节点 LP 缓存从此建立；仅数值/界改变时才可复用符号结构，任何
// 模式或事务签名不匹配都必须转入完整重建路径。
void NativeIPMLPAdapter::prepare_for_node_solves(const LPModel& base_lp) {
  auto cs = std::make_unique<CachedState>();

  // Keep a copy so CSC pointers remain valid
  cs->base_lp_copy = base_lp;

  // One-sided (G-type) row normalization (same as solve_lp): rows of the
  // form A x >= lhs (b = +inf) are negated to -A x <= -lhs so every row has
  // a finite upper bound; duals are negated back at extraction.
  {
    const int mi0 = static_cast<int>(cs->base_lp_copy.A.rows());
    cs->flip_row.assign(static_cast<size_t>(mi0), 0);
    bool any_flip = false;
    for (int i = 0; i < mi0; ++i) {
      const double lhs_i = lp_row_lhs_or_neg_inf(cs->base_lp_copy, i);
      if (!std::isfinite(cs->base_lp_copy.b[i]) && std::isfinite(lhs_i)) {
        cs->flip_row[static_cast<size_t>(i)] = 1;
        any_flip = true;
      }
    }
    if (any_flip) {
      for (int k = 0; k < cs->base_lp_copy.A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(cs->base_lp_copy.A, k);
             it; ++it)
          if (cs->flip_row[static_cast<size_t>(it.row())])
            it.valueRef() = -it.value();
      for (int i = 0; i < mi0; ++i) {
        if (!cs->flip_row[static_cast<size_t>(i)]) continue;
        cs->base_lp_copy.b[i] = -lp_row_lhs_or_neg_inf(base_lp, i);
        if (cs->base_lp_copy.row_lhs.size() == cs->base_lp_copy.A.rows())
          cs->base_lp_copy.row_lhs[i] =
              -std::numeric_limits<double>::infinity();
      }
    } else {
      cs->flip_row.clear();
    }
  }

  const auto& lp = cs->base_lp_copy;

  cs->n_orig = static_cast<int>(lp.c.size());
  if (cs->n_orig == 0) { cached_state_ = std::move(cs); return; }

  cs->sense_sign = (lp.sense == Sense::Maximize) ? -1 : 1;
  cs->mi = static_cast<int>(lp.A.rows());
  cs->me = static_cast<int>(lp.Aeq.rows());
  cs->m = cs->mi + cs->me;
  cs->nn = cs->n_orig + cs->mi;

  const int n_orig = cs->n_orig;
  const int mi = cs->mi;
  const int me = cs->me;
  const int m = cs->m;
  const int nn = cs->nn;

  // Ruiz equilibration (same option as solve_lp).  Scaled matrices and the
  // dr/dc vectors move into the cache; all downstream structures (CSR,
  // scatter maps, symbolic factorization) are built from the scaled data.
  {
    RuizScaling scal =
        (opt_.ruiz_rounds > 0 && m > 0)
            ? ruiz_equilibrate(lp.A, lp.Aeq, n_orig, opt_.ruiz_rounds)
            : RuizScaling{};
    cs->scaling_active = scal.active;
    if (scal.active) {
      cs->A_scaled = std::move(scal.A);
      cs->Aeq_scaled = std::move(scal.Aeq);
      cs->scal_dr = std::move(scal.dr);
      cs->scal_dc = std::move(scal.dc);
    }
  }
  const Eigen::SparseMatrix<double>& A_mat =
      cs->scaling_active ? cs->A_scaled : lp.A;
  const Eigen::SparseMatrix<double>& Aeq_mat =
      cs->scaling_active ? cs->Aeq_scaled : lp.Aeq;

  // Cost
  cs->c.resize(nn, 0.0);
  for (int j = 0; j < n_orig; ++j)
    cs->c[j] = cs->sense_sign * lp.c(j) *
               (cs->scaling_active ? cs->scal_dc[static_cast<size_t>(j)] : 1.0);

  // RHS
  cs->b.resize(m);
  for (int i = 0; i < mi; ++i)
    cs->b[i] = (cs->scaling_active ? cs->scal_dr[static_cast<size_t>(i)] : 1.0) * lp.b(i);
  for (int i = 0; i < me; ++i)
    cs->b[mi + i] = (cs->scaling_active ? cs->scal_dr[static_cast<size_t>(mi + i)] : 1.0) * lp.beq(i);

  // CSC pointers from our copy (scaled matrices when Ruiz is active)
  cs->A_o = A_mat.outerIndexPtr();
  cs->A_i = A_mat.innerIndexPtr();
  cs->A_v = A_mat.valuePtr();
  cs->Aeq_o = me > 0 ? Aeq_mat.outerIndexPtr() : nullptr;
  cs->Aeq_i = me > 0 ? Aeq_mat.innerIndexPtr() : nullptr;
  cs->Aeq_v = me > 0 ? Aeq_mat.valuePtr() : nullptr;

  // Build CSR
  auto build_csr = [](const Eigen::SparseMatrix<double>& M,
                       std::vector<int>& rp, std::vector<int>& ci, std::vector<double>& rv) {
    int rows = static_cast<int>(M.rows());
    rp.assign(rows + 1, 0);
    const int* Mo = M.outerIndexPtr();
    const int* Mi = M.innerIndexPtr();
    const double* Mv = M.valuePtr();
    int cols = static_cast<int>(M.cols());
    for (int k = 0; k < cols; ++k)
      for (int p = Mo[k]; p < Mo[k + 1]; ++p)
        rp[Mi[p] + 1]++;
    for (int i = 0; i < rows; ++i) rp[i + 1] += rp[i];
    int nnz = rp[rows];
    ci.resize(nnz); rv.resize(nnz);
    std::vector<int> pos(rp.begin(), rp.begin() + rows);
    for (int k = 0; k < cols; ++k)
      for (int p = Mo[k]; p < Mo[k + 1]; ++p) {
        int r = Mi[p];
        int q = pos[r]++;
        ci[q] = k; rv[q] = Mv[p];
      }
  };
  if (mi > 0) build_csr(A_mat, cs->A_rp, cs->A_ci, cs->A_rv);
  if (me > 0) build_csr(Aeq_mat, cs->Aeq_rp, cs->Aeq_ci, cs->Aeq_rv);

  // Bandwidth
  for (int j = 0; j < n_orig; ++j) {
    int rmin = m, rmax = -1;
    for (int p = cs->A_o[j]; p < cs->A_o[j + 1]; ++p) {
      rmin = std::min(rmin, cs->A_i[p]);
      rmax = std::max(rmax, cs->A_i[p]);
    }
    if (me > 0) {
      for (int p = cs->Aeq_o[j]; p < cs->Aeq_o[j + 1]; ++p) {
        int r = mi + cs->Aeq_i[p];
        rmin = std::min(rmin, r);
        rmax = std::max(rmax, r);
      }
    }
    if (rmax >= 0) cs->bandwidth = std::max(cs->bandwidth, rmax - rmin);
  }
  cs->use_banded = (cs->bandwidth <= kBandedThreshold);

  if (cs->use_banded) {
    const int bw = cs->bandwidth;
    cs->band_storage.resize(static_cast<size_t>(bw + 1) * m, 0.0);

    // Tight reserve bound (same as solve_lp): rows pair only within the
    // band, so nz^2 would over-allocate and a single dense column would
    // request ~1e10 entries -> bad_alloc.
    size_t total = 0;
    for (int j = 0; j < n_orig; ++j) {
      const size_t nz =
          static_cast<size_t>(cs->A_o[j + 1] - cs->A_o[j]) +
          (me > 0 ? static_cast<size_t>(cs->Aeq_o[j + 1] - cs->Aeq_o[j]) : 0);
      total += nz * std::min(nz, static_cast<size_t>(bw) + 1);
    }
    if (total > kMaxScatterEntries) {
      // Scatter map too large — skip caching; node LPs fall back to the
      // full solve_lp path (which has its own memory-feasibility gates).
      cached_state_.reset();
      return;
    }
    cs->band_scatter.reserve(total);
    cs->band_scatter_col_start.resize(n_orig + 1);
    cs->band_scatter_col_start[0] = 0;

    std::vector<int> col_rows;
    std::vector<double> col_vals;
    for (int j = 0; j < n_orig; ++j) {
      col_rows.clear(); col_vals.clear();
      for (int p = cs->A_o[j]; p < cs->A_o[j + 1]; ++p) {
        col_rows.push_back(cs->A_i[p]);
        col_vals.push_back(cs->A_v[p]);
      }
      if (me > 0) {
        for (int p = cs->Aeq_o[j]; p < cs->Aeq_o[j + 1]; ++p) {
          col_rows.push_back(mi + cs->Aeq_i[p]);
          col_vals.push_back(cs->Aeq_v[p]);
        }
      }
      int nz = static_cast<int>(col_rows.size());
      for (int pi = 0; pi < nz; ++pi)
        for (int pk = 0; pk < nz; ++pk) {
          int r1 = col_rows[pi], r2 = col_rows[pk];
          if (r1 < r2) continue;
          int d = r1 - r2;
          if (d <= bw)
            cs->band_scatter.push_back({d * m + r2, col_vals[pi] * col_vals[pk]});
        }
      cs->band_scatter_col_start[j + 1] = static_cast<int>(cs->band_scatter.size());
    }
  } else {
    // Estimate the scatter map size (Theta(sum_j nz_j^2), also an upper
    // proxy for nnz of N = Ae*Ae') before building anything: a single
    // dense column (nz = 1e5) implies ~1e10 entries -> bad_alloc.  Skip
    // caching in that case; node LPs fall back to the full solve_lp path.
    {
      size_t scatter_est = 0;
      for (int j = 0; j < n_orig; ++j) {
        const size_t nz =
            static_cast<size_t>(cs->A_o[j + 1] - cs->A_o[j]) +
            (me > 0 ? static_cast<size_t>(cs->Aeq_o[j + 1] - cs->Aeq_o[j]) : 0);
        scatter_est += nz * nz;
      }
      if (scatter_est > kMaxScatterEntries) {
        cached_state_.reset();
        return;
      }
    }
    // Sparse path: build Ae, N_sparse, scatter maps, symbolic factorization
    using T = Eigen::Triplet<double>;
    std::vector<T> trips;
    trips.reserve(A_mat.nonZeros() + Aeq_mat.nonZeros() + mi);
    for (int k = 0; k < A_mat.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A_mat, k); it; ++it)
        trips.emplace_back(it.row(), it.col(), it.value());
    for (int i = 0; i < mi; ++i)
      trips.emplace_back(i, n_orig + i, 1.0);
    for (int k = 0; k < Aeq_mat.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(Aeq_mat, k); it; ++it)
        trips.emplace_back(mi + it.row(), it.col(), it.value());
    Eigen::SparseMatrix<double> Ae(m, nn);
    Ae.setFromTriplets(trips.begin(), trips.end());
    Ae.makeCompressed();
    const int* Ao = Ae.outerIndexPtr();
    const int* Ai = Ae.innerIndexPtr();
    const double* Av = Ae.valuePtr();

    cs->N_sparse = Ae * Ae.transpose();
    for (int i = 0; i < m; ++i) cs->N_sparse.coeffRef(i, i) += 1e-10;
    cs->N_sparse = cs->N_sparse.triangularView<Eigen::Lower>();
    cs->N_sparse.makeCompressed();
    cs->sparse_n_nnz = static_cast<int>(cs->N_sparse.nonZeros());

    const int* No = cs->N_sparse.outerIndexPtr();
    const int* Ni = cs->N_sparse.innerIndexPtr();
    size_t total = 0;
    for (int j = 0; j < n_orig; ++j) {
      int nz = Ao[j + 1] - Ao[j];
      total += static_cast<size_t>(nz) * nz;
    }
    cs->sparse_scatter.reserve(total);
    cs->sparse_scatter_col_start.resize(n_orig + 1);
    cs->sparse_scatter_col_start[0] = 0;
    for (int j = 0; j < n_orig; ++j) {
      for (int pi = Ao[j]; pi < Ao[j + 1]; ++pi)
        for (int pk = Ao[j]; pk < Ao[j + 1]; ++pk) {
          int r1 = Ai[pi], r2 = Ai[pk];
          const int* pos = std::lower_bound(Ni + No[r2], Ni + No[r2 + 1], r1);
          if (pos != Ni + No[r2 + 1] && *pos == r1)
            cs->sparse_scatter.push_back({static_cast<int>(pos - Ni), Av[pi] * Av[pk]});
        }
      cs->sparse_scatter_col_start[j + 1] = static_cast<int>(cs->sparse_scatter.size());
    }
    cs->sparse_diag_offsets.resize(m);
    for (int i = 0; i < m; ++i) {
      const int* pos = std::lower_bound(Ni + No[i], Ni + No[i + 1], i);
      cs->sparse_diag_offsets[i] = static_cast<int>(pos - Ni);
    }

#if MIPSOLVERS_HAVE_CHOLMOD && !MIPSOLVERS_USE_ACCELERATE
    // CHOLMOD symbolic analysis — once per cache lifetime; numeric
    // factorization re-reads the (per-solve refilled) N_local values, which
    // share this exact pattern.  Priority: Accelerate > CHOLMOD > Eigen
    // (Accelerate measured ~1.9x faster on Apple Silicon).
    cs->cholmod_ok = cs->cholmod.analyze(
        m, cs->N_sparse.outerIndexPtr(), cs->N_sparse.innerIndexPtr(),
        cs->N_sparse.valuePtr(), cs->sparse_n_nnz);
#endif
#if MIPSOLVERS_USE_ACCELERATE
    if (!cs->cholmod_ok) {
    const int* No2 = cs->N_sparse.outerIndexPtr();
    cs->accel_col_starts.resize(m + 1);
    for (int i = 0; i <= m; ++i)
      cs->accel_col_starts[i] = static_cast<long>(No2[i]);

    SparseMatrixStructure s{};
    s.rowCount = m;
    s.columnCount = m;
    s.columnStarts = cs->accel_col_starts.data();
    s.rowIndices = cs->N_sparse.innerIndexPtr();
    s.attributes.transpose = false;
    s.attributes.triangle = SparseLowerTriangle;
    s.attributes.kind = SparseSymmetric;
    s.attributes._reserved = 0;
    s.attributes._allocatedBySparse = false;
    s.blockSize = 1;
    cs->accel_symbolic = SparseFactor(SparseFactorizationCholesky, s,
                                      ipm_accel_symbolic_options());
    cs->accel_symbolic_valid = true;
    }
#else
    if (!cs->cholmod_ok) cs->ldlt.analyzePattern(cs->N_sparse);
#endif
  }

  // Compute initial least-squares dual: y0 = (A*A^T + reg*I)^{-1} * A*c.
  // One factorize+solve (~8ms) that saves 30+ iterations on every first node LP.
  if (m > 0 && !cs->use_banded) {
    // Fill N with theta=1 (i.e. N = A*A^T + reg*I)
    double* Nv = cs->N_sparse.valuePtr();
    std::memset(Nv, 0, sizeof(double) * cs->sparse_n_nnz);
    for (int j = 0; j < n_orig; ++j) {
      for (int si = cs->sparse_scatter_col_start[j]; si < cs->sparse_scatter_col_start[j + 1]; ++si)
        Nv[cs->sparse_scatter[si].ni] += cs->sparse_scatter[si].a_prod;  // theta=1
    }
    for (int i = 0; i < mi; ++i)
      Nv[cs->sparse_diag_offsets[i]] += 1.0;  // slack columns with theta=1
    for (int i = 0; i < m; ++i)
      Nv[cs->sparse_diag_offsets[i]] += 1e-10;  // regularization

    // RHS = A * c (including slack cost = 0)
    cs->last_dual.resize(m, 0.0);
    for (int i = 0; i < mi; ++i) {
      double s = 0.0;
      for (int p = cs->A_rp[i]; p < cs->A_rp[i + 1]; ++p)
        s += cs->A_rv[p] * cs->c[cs->A_ci[p]];
      cs->last_dual[i] = s;
    }
    for (int i = 0; i < me; ++i) {
      double s = 0.0;
      for (int p = cs->Aeq_rp[i]; p < cs->Aeq_rp[i + 1]; ++p)
        s += cs->Aeq_rv[p] * cs->c[cs->Aeq_ci[p]];
      cs->last_dual[mi + i] = s;
    }

    // Factorize and solve
#if MIPSOLVERS_HAVE_CHOLMOD
    if (cs->cholmod_ok) {
      if (cs->cholmod.factorize(cs->N_sparse.valuePtr()))
        cs->cholmod.solve(cs->last_dual.data(), cs->last_dual.data());
    } else
#endif
#if MIPSOLVERS_USE_ACCELERATE
    {
    SparseMatrixStructure ds{};
    ds.rowCount = m;
    ds.columnCount = m;
    ds.columnStarts = cs->accel_col_starts.data();
    ds.rowIndices = cs->N_sparse.innerIndexPtr();
    ds.attributes.transpose = false;
    ds.attributes.triangle = SparseLowerTriangle;
    ds.attributes.kind = SparseSymmetric;
    ds.attributes._reserved = 0;
    ds.attributes._allocatedBySparse = false;
    ds.blockSize = 1;
    SparseMatrix_Double apple_N{};
    apple_N.structure = ds;
    apple_N.data = cs->N_sparse.valuePtr();
    auto num_f = SparseFactor(cs->accel_symbolic, apple_N);
    if (num_f.status == SparseStatusOK) {
      DenseVector_Double dv{};
      dv.count = m;
      dv.data = cs->last_dual.data();
      SparseSolve(num_f, dv);
      SparseCleanup(num_f);
    }
    }
#else
    {
    cs->ldlt.factorize(cs->N_sparse);
    if (cs->ldlt.info() == Eigen::Success) {
      Eigen::Map<Eigen::VectorXd> y_map(cs->last_dual.data(), m);
      y_map = cs->ldlt.solve(y_map.eval());
    }
    }
#endif
  }

  cached_state_ = std::move(cs);
}

void NativeIPMLPAdapter::update_cached_cost(const Eigen::VectorXd& new_c) {
  if (!cached_state_ || cached_state_->n_orig == 0) return;
  auto& cs = *cached_state_;
  const int n_orig = cs.n_orig;
  // new_c is in the original LP convention; the cache stores Dc-scaled cost.
  for (int j = 0; j < n_orig && j < static_cast<int>(new_c.size()); ++j)
    cs.c[j] = cs.sense_sign * new_c[j] *
              (cs.scaling_active ? cs.scal_dc[static_cast<size_t>(j)] : 1.0);
  // Slack costs (indices n_orig..nn-1) remain 0.
}

SolveResult NativeIPMLPAdapter::solve_structure_aware_node_lp(
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const Eigen::VectorXd& x0) const {
  if (!cached_state_) {
    SolveResult out;
    out.stats.solver_name = name();
    out.stats.status = "NoCachedState";
    return out;
  }
  const bool use_augmented_direct =
      !cached_state_->use_banded && cached_state_->m > 256 &&
      cached_state_->bandwidth > cached_state_->m / 4;
  if (!use_augmented_direct) {
    return solve_cached_node_lp(node_lb, node_ub, x0);
  }

  LPModel node_lp = cached_state_->base_lp_copy;
  const int n = cached_state_->n_orig;
  if (node_lb.size() != n || node_ub.size() != n ||
      (x0.size() != 0 && x0.size() != n)) {
    SolveResult out;
    out.stats.solver_name = name();
    out.stats.status = "Invalid structure-aware IPM node domain";
    return out;
  }
  for (int j = 0; j < n; ++j) {
    node_lp.vars[static_cast<std::size_t>(j)].lb = node_lb[j];
    node_lp.vars[static_cast<std::size_t>(j)].ub = node_ub[j];
  }
  return solve_lp_impl(node_lp, x0, opt_.ruiz_rounds);
}

SolveResult NativeIPMLPAdapter::solve_cached_node_lp(
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const Eigen::VectorXd& x0) const {
  SolveResult out;
  out.stats.solver_name = name();
  const auto t0 = std::chrono::steady_clock::now();

  if (!cached_state_ || cached_state_->n_orig == 0) {
    out.stats.status = "NoCachedState";
    if (opt_.verbose)
      fprintf(stderr, "[IPM-CACHE] solve_cached: NO cached state!\n");
    return out;
  }

  if (opt_.verbose)
    fprintf(stderr, "[IPM-CACHE] solve_cached: n_orig=%d m=%d bw=%d %s ws=%d\n",
            cached_state_->n_orig, cached_state_->m, cached_state_->bandwidth,
            cached_state_->use_banded ? "BANDED" : "SPARSE",
            (int)(x0.size() == cached_state_->n_orig));

  const auto& cs = *cached_state_;
  const int n_orig = cs.n_orig;
  const int mi = cs.mi;
  const int me = cs.me;
  const int m = cs.m;
  const int nn = cs.nn;
  const int sense_sign = cs.sense_sign;
  const bool has_warm_start = (x0.size() == n_orig);
  const bool use_banded = cs.use_banded;
  const int bw = cs.bandwidth;

  // CSC pointers (from cached copy)
  const int* A_o = cs.A_o;
  const int* A_i = cs.A_i;
  const double* A_v = cs.A_v;
  const int* Aeq_o = cs.Aeq_o;
  const int* Aeq_i = cs.Aeq_i;
  const double* Aeq_v = cs.Aeq_v;

  // Bounds — only thing that changes per node.  Node bounds arrive in
  // original coordinates; the cache holds the Ruiz-scaled problem, so
  // finite bounds are scaled by 1/dc here (flags from original bounds).
  const bool scaling = cs.scaling_active;
  std::vector<double> lb(nn), ub(nn), flb(nn), fub(nn);
  for (int j = 0; j < n_orig; ++j) {
    double lo = (j < (int)node_lb.size()) ? node_lb[j] : -kBig;
    double hi = (j < (int)node_ub.size()) ? node_ub[j] : kBig;
    if (lo < -kBig + 1) lo = -kBig;
    if (hi > kBig - 1) hi = kBig;
    flb[j] = (lo > -kBig + 1) ? 1.0 : 0.0;
    fub[j] = (hi < kBig - 1) ? 1.0 : 0.0;
    if (scaling) {
      const double inv_d = 1.0 / cs.scal_dc[static_cast<size_t>(j)];
      if (flb[j]) lo *= inv_d;
      if (fub[j]) hi *= inv_d;
    }
    lb[j] = lo; ub[j] = hi;
  }
  for (int i = 0; i < mi; ++i) {
    lb[n_orig + i] = 0.0;
    const double lhs = lp_row_lhs_or_neg_inf(cs.base_lp_copy, i);
    // cs.b is stored scaled; scale (b - lhs) by the same dr_i.
    const double dri = scaling ? cs.scal_dr[static_cast<size_t>(i)] : 1.0;
    ub[n_orig + i] =
        std::isfinite(lhs) && std::isfinite(cs.base_lp_copy.b[i])
            ? std::max(0.0, dri * (cs.base_lp_copy.b[i] - lhs))
            : kBig;
    flb[n_orig + i] = 1.0;
    fub[n_orig + i] = (ub[n_orig + i] < kBig - 1) ? 1.0 : 0.0;
  }

  // Detect fixed variables (lb ≈ ub) and remove from barrier formulation.
  std::vector<bool> is_fixed(nn, false);
  int n_fixed_vars = 0;
  for (int j = 0; j < n_orig; ++j) {
    if (flb[j] && fub[j] && ub[j] - lb[j] < 1e-9) {
      is_fixed[j] = true;
      ++n_fixed_vars;
      flb[j] = 0.0;
      fub[j] = 0.0;
    }
  }

  // Alias cached data
  const auto& c_vec = cs.c;
  const auto& b_vec = cs.b;
  const auto& A_rp = cs.A_rp;
  const auto& A_ci = cs.A_ci;
  const auto& A_rv = cs.A_rv;
  const auto& Aeq_rp = cs.Aeq_rp;
  const auto& Aeq_ci = cs.Aeq_ci;
  const auto& Aeq_rv = cs.Aeq_rv;

  // SpMV lambdas (use cached CSR/CSC)
  auto ae_mul_sub = [&](const double* MIPSOLVERS_RESTRICT x, double* MIPSOLVERS_RESTRICT y) {
    for (int i = 0; i < mi; ++i) {
      double s = x[n_orig + i];
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        s += A_rv[p] * x[A_ci[p]];
      y[i] -= s;
    }
    for (int k = 0; k < me; ++k) {
      double s = 0.0;
      for (int p = Aeq_rp[k]; p < Aeq_rp[k + 1]; ++p)
        s += Aeq_rv[p] * x[Aeq_ci[p]];
      y[mi + k] -= s;
    }
  };

  auto aet_mul = [&](const double* MIPSOLVERS_RESTRICT w, double* MIPSOLVERS_RESTRICT y) {
    for (int j = 0; j < n_orig; ++j) {
      double s = 0.0;
      for (int p = A_o[j]; p < A_o[j + 1]; ++p)
        s += A_v[p] * w[A_i[p]];
      if (me > 0)
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p)
          s += Aeq_v[p] * w[mi + Aeq_i[p]];
      y[j] = s;
    }
    for (int i = 0; i < mi; ++i) y[n_orig + i] = w[i];
  };

  auto compute_residuals = [&](const double* MIPSOLVERS_RESTRICT xv, const double* MIPSOLVERS_RESTRICT yv,
                               double* MIPSOLVERS_RESTRICT rp, double* MIPSOLVERS_RESTRICT rd) {
    std::memcpy(rp, b_vec.data(), sizeof(double) * m);
    std::memcpy(rd, c_vec.data(), sizeof(double) * nn);
    for (int j = 0; j < n_orig; ++j) {
      double xj = xv[j];
      double dj = 0.0;
      for (int p = A_o[j]; p < A_o[j + 1]; ++p) {
        int i = A_i[p];
        double aij = A_v[p];
        rp[i] -= aij * xj;
        dj += aij * yv[i];
      }
      if (me > 0)
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p) {
          int i = Aeq_i[p];
          double aij = Aeq_v[p];
          rp[mi + i] -= aij * xj;
          dj += aij * yv[mi + i];
        }
      rd[j] -= dj;
    }
    for (int i = 0; i < mi; ++i) {
      rp[i] -= xv[n_orig + i];
      rd[n_orig + i] -= yv[i];
    }
  };

  // === Initialization ===
  std::vector<double> xv(nn), yv(m);
  std::vector<double> gl(nn), gu(nn), zl(nn), zu(nn);
  double* x_d = xv.data();
  double* y_d = yv.data();
  double* gl_d = gl.data();
  double* gu_d = gu.data();
  double* zl_d = zl.data();
  double* zu_d = zu.data();

  {
    if (has_warm_start) {
      for (int j = 0; j < n_orig; ++j) {
        double lo = flb[j] ? lb[j] : -kBig;
        double hi = fub[j] ? ub[j] : kBig;
        // Caller-provided x0 is in original coordinates: x̂0 = x0 / Dc.
        const double x0j =
            scaling ? x0[j] / cs.scal_dc[static_cast<size_t>(j)] : x0[j];
        if (hi - lo < 2e-6) {
          x_d[j] = 0.5 * (lo + hi);
        } else {
          x_d[j] = std::clamp(x0j, lo + 1e-6, hi - 1e-6);
        }
      }
    } else {
      for (int j = 0; j < n_orig; ++j) {
        double lo = flb[j] ? lb[j] : -10.0;
        double hi = fub[j] ? ub[j] : 10.0;
        x_d[j] = 0.5 * (lo + hi);
      }
    }

    // Compute inequality slacks: s_i = b_i - A_i * x (must be > 0)
    for (int i = 0; i < mi; ++i) {
      double ax = 0.0;
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        ax += A_rv[p] * x_d[A_ci[p]];
      x_d[n_orig + i] = [&] {
        // Interior slack, finite even for G-type rows (b = +inf) and for
        // inits violating the row's two-sided range (see solve_lp).
        double s_init = b_vec[i] - ax;
        if (!std::isfinite(s_init)) s_init = 1.0;
        return std::clamp(s_init, 1e-4, std::max(1e-4, ub[n_orig + i] - 1e-4));
      }();
    }

    // Dual warm-start: reuse previous solve's dual if available.
    // This dramatically reduces iteration count (from ~60 to ~15-20).
    if (static_cast<int>(cs.last_dual.size()) == m) {
      std::memcpy(yv.data(), cs.last_dual.data(), sizeof(double) * m);
    } else {
      std::fill(yv.begin(), yv.end(), 0.0);
    }
  }

  // Lock fixed variables at their exact value before general clamping.
  for (int j = 0; j < n_orig; ++j) {
    if (is_fixed[j]) x_d[j] = lb[j];
  }

  // Clamp original variables to bounds, then recompute slacks.
  for (int j = 0; j < n_orig; ++j) {
    if (is_fixed[j]) continue;  // already locked
    double lo = flb[j] ? lb[j] : (x_d[j] - 10.0);
    double hi = fub[j] ? ub[j] : (x_d[j] + 10.0);
    double range = hi - lo;
    x_d[j] = std::clamp(x_d[j], lo + 0.01 * range, hi - 0.01 * range);
    if (x_d[j] <= lo || x_d[j] >= hi) x_d[j] = 0.5 * (lo + hi);
  }
  for (int i = 0; i < mi; ++i) {
    double ax = 0.0;
    for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
      ax += A_rv[p] * x_d[A_ci[p]];
    x_d[n_orig + i] = [&] {
      // Interior slack, finite even for G-type rows (b = +inf) and for
      // inits violating the row's two-sided range (see solve_lp).
      double s_init = b_vec[i] - ax;
      if (!std::isfinite(s_init)) s_init = 1.0;
      return std::clamp(s_init, 1e-4, std::max(1e-4, ub[n_orig + i] - 1e-4));
    }();
  }
  // Bound slacks and initial multipliers.
  // If we have previous zl/zu, use them (scaled to current gap sizes).
  const bool have_prev_z = (static_cast<int>(cs.last_zl.size()) == nn &&
                            static_cast<int>(cs.last_zu.size()) == nn);
  for (int j = 0; j < nn; ++j) {
    gl_d[j] = flb[j] ? std::max(x_d[j] - lb[j], 1e-4) : kBig;
    gu_d[j] = fub[j] ? std::max(ub[j] - x_d[j], 1e-4) : kBig;
    if (have_prev_z) {
      // Scale previous multiplier to maintain approx complementarity:
      // zl * gl ≈ mu_target. Use zl_prev but cap to prevent extreme values.
      zl_d[j] = flb[j] ? std::clamp(cs.last_zl[j], 1e-6, 1e4) : 0.0;
      zu_d[j] = fub[j] ? std::clamp(cs.last_zu[j], 1e-6, 1e4) : 0.0;
    } else {
      zl_d[j] = flb[j] ? 1.0 : 0.0;
      zu_d[j] = fub[j] ? 1.0 : 0.0;
    }
  }

  // === IPM iteration (same loop as solve_lp but using cached structures) ===
  std::vector<double> theta(nn), dx(nn), dy(m), dzl(nn), dzu(nn);
  std::vector<double> dx_aff(nn), dy_aff(m), dzl_aff(nn), dzu_aff(nn);
  std::vector<double> rhs(m), tmp_n(nn), Atdy(nn);
  std::vector<double> r_p(m), r_d(nn);
  std::vector<double> inv_gl(nn), inv_gu(nn);

  double* theta_d = theta.data();
  double* dx_d = dx.data();
  double* dy_d = dy.data();
  double* dx_aff_d = dx_aff.data();
  double* dy_aff_d = dy_aff.data();
  double* dzl_aff_d = dzl_aff.data();
  double* dzu_aff_d = dzu_aff.data();
  double* rhs_d = rhs.data();
  double* tmp_d = tmp_n.data();
  double* Atdy_d = Atdy.data();
  double* r_p_d = r_p.data();
  double* r_d_d = r_d.data();
  double* inv_gl_d = inv_gl.data();
  double* inv_gu_d = inv_gu.data();
  const double* flb_d = flb.data();
  const double* fub_d = fub.data();
  const double* lb_d = lb.data();
  const double* ub_d = ub.data();

  const double reg = (n_fixed_vars > n_orig / 4) ? 1e-6 : 1e-10;
  const int max_iter = opt_.max_iter;
  bool converged = false;

  // Working copies for Cholesky
  std::vector<double> band_work;
  if (use_banded) band_work.resize(cs.band_storage.size());

  // Mutable copy of band_storage for fill
  std::vector<double> band_storage_local;
  if (use_banded) band_storage_local.resize(cs.band_storage.size());

  // For sparse path, we need mutable N_sparse values
  Eigen::SparseMatrix<double> N_local;
#if MIPSOLVERS_USE_ACCELERATE
  std::vector<long> accel_col_starts_local;
  SparseOpaqueFactorization_Double accel_numeric{};
  bool accel_numeric_valid = false;
#else
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, Eigen::AMDOrdering<int>> ldlt_local;
#endif
  if (!use_banded) {
    N_local = cs.N_sparse;  // copy structure + values (values will be overwritten)
#if MIPSOLVERS_USE_ACCELERATE
    accel_col_starts_local = cs.accel_col_starts;
#else
    ldlt_local.analyzePattern(N_local);
#endif
  }

  // Fill + factor lambdas using cached scatter maps
  auto fill_and_factor_banded = [&]() -> bool {
    double* bs = band_storage_local.data();
    std::memset(bs, 0, sizeof(double) * band_storage_local.size());
    for (int j = 0; j < n_orig; ++j) {
      const double th = theta_d[j];
      for (int si = cs.band_scatter_col_start[j]; si < cs.band_scatter_col_start[j + 1]; ++si)
        bs[cs.band_scatter[si].offset] += th * cs.band_scatter[si].a_prod;
    }
    for (int i = 0; i < mi; ++i) bs[i] += theta_d[n_orig + i];
    for (int i = 0; i < m; ++i) bs[i] += reg;
    std::memcpy(band_work.data(), bs, sizeof(double) * band_storage_local.size());
    return banded_chol_factor(band_work.data(), m, bw);
  };

  auto fill_and_factor_sparse = [&]() -> bool {
    double* Nv = N_local.valuePtr();
    std::memset(Nv, 0, sizeof(double) * cs.sparse_n_nnz);
    for (int j = 0; j < n_orig; ++j) {
      const double th = theta_d[j];
      for (int si = cs.sparse_scatter_col_start[j]; si < cs.sparse_scatter_col_start[j + 1]; ++si)
        Nv[cs.sparse_scatter[si].ni] += th * cs.sparse_scatter[si].a_prod;
    }
    for (int i = 0; i < mi; ++i) Nv[cs.sparse_diag_offsets[i]] += theta_d[n_orig + i];
    for (int i = 0; i < m; ++i) Nv[cs.sparse_diag_offsets[i]] += reg;
#if MIPSOLVERS_HAVE_CHOLMOD
    if (cs.cholmod_ok) {
      return cs.cholmod.factorize(N_local.valuePtr());
    }
#endif
#if MIPSOLVERS_USE_ACCELERATE
    SparseMatrixStructure s_struct{};
    s_struct.rowCount = m;
    s_struct.columnCount = m;
    s_struct.columnStarts = accel_col_starts_local.data();
    s_struct.rowIndices = N_local.innerIndexPtr();
    s_struct.attributes.transpose = false;
    s_struct.attributes.triangle = SparseLowerTriangle;
    s_struct.attributes.kind = SparseSymmetric;
    s_struct.attributes._reserved = 0;
    s_struct.attributes._allocatedBySparse = false;
    s_struct.blockSize = 1;
    SparseMatrix_Double apple_N{};
    apple_N.structure = s_struct;
    apple_N.data = N_local.valuePtr();
    if (accel_numeric_valid) {
      SparseRefactor(apple_N, &accel_numeric);
    } else {
      accel_numeric = SparseFactor(cs.accel_symbolic, apple_N);
      accel_numeric_valid = true;
    }
    return accel_numeric.status == SparseStatusOK;
#else
    ldlt_local.factorize(N_local);
    return ldlt_local.info() == Eigen::Success;
#endif
  };

  // Solve normal equations with conditional iterative refinement (same
  // scheme as the non-cached path: residual against the pristine matrix,
  // up to 2 correction solves with the existing factorization).
  std::vector<double> ir_ndy(m), ir_r(m), ir_corr(m);
  auto solve_normal = [&](double* rhs_buf, double* dy_buf) {
    auto raw_solve = [&](const double* rhs, double* dy) {
      if (use_banded) {
        std::memcpy(dy, rhs, sizeof(double) * m);
        banded_chol_solve(band_work.data(), m, bw, dy);
      } else {
#if MIPSOLVERS_HAVE_CHOLMOD
        if (cs.cholmod_ok) {
          if (!cs.cholmod.solve(rhs, dy)) {
            // Poison with NaN so the finiteness guard aborts the loop.
            std::fill(dy, dy + m, std::numeric_limits<double>::quiet_NaN());
          }
          return;
        }
#endif
#if MIPSOLVERS_USE_ACCELERATE
        std::memcpy(dy, rhs, sizeof(double) * m);
        DenseVector_Double xb{};
        xb.count = m;
        xb.data = dy;
        SparseSolve(accel_numeric, xb);
#else
        Eigen::Map<const Eigen::VectorXd> rhs_map(rhs, m);
        Eigen::Map<Eigen::VectorXd> dy_map(dy, m);
        dy_map = ldlt_local.solve(rhs_map);
#endif
      }
    };
    raw_solve(rhs_buf, dy_buf);
    if (m == 0) return;
    double rhs_norm = 0.0;
    for (int i = 0; i < m; ++i)
      rhs_norm = std::max(rhs_norm, std::abs(rhs_buf[i]));
    const double ir_tol = 1e-12 * std::max(1.0, rhs_norm);
    for (int ref = 0; ref < 2; ++ref) {
      // ndy = N * dy against the pristine matrix
      if (use_banded) {
        banded_sym_matvec(band_storage_local.data(), m, bw, dy_buf, ir_ndy.data());
      } else {
        Eigen::Map<const Eigen::VectorXd> dy_map(dy_buf, m);
        Eigen::Map<Eigen::VectorXd> ndy_map(ir_ndy.data(), m);
        ndy_map = N_local.selfadjointView<Eigen::Lower>() * dy_map;
      }
      double r_norm = 0.0;
      for (int i = 0; i < m; ++i) {
        ir_r[i] = rhs_buf[i] - ir_ndy[i];
        r_norm = std::max(r_norm, std::abs(ir_r[i]));
      }
      if (r_norm <= ir_tol) break;
      raw_solve(ir_r.data(), ir_corr.data());
      for (int i = 0; i < m; ++i) dy_buf[i] += ir_corr[i];
    }
  };

  double last_pfeas = 0.0, last_dfeas = 0.0, last_mu = 0.0;
  for (int iter = 0; iter < max_iter; ++iter) {
    if (opt_.time_limit_sec > 0.0 && std::isfinite(opt_.time_limit_sec)) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
              .count();
      if (elapsed >= opt_.time_limit_sec) {
        out.stats.success = false;
        out.stats.iterations = iter;
        out.stats.status = "Time limit";
        out.stats.primal_feas = last_pfeas;
        out.stats.dual_feas = last_dfeas;
        out.stats.complementarity = last_mu;
        break;
      }
    }
    // Residuals
    compute_residuals(x_d, y_d, r_p_d, r_d_d);

    int n_compl = 0;
    double comp_sum = 0.0, pfeas = 0.0, dfeas = 0.0;
    for (int j = 0; j < nn; ++j) {
      r_d_d[j] -= flb_d[j] * zl_d[j];
      r_d_d[j] += fub_d[j] * zu_d[j];
      comp_sum += flb_d[j] * gl_d[j] * zl_d[j] + fub_d[j] * gu_d[j] * zu_d[j];
      n_compl += static_cast<int>(flb_d[j]) + static_cast<int>(fub_d[j]);
      double ad = std::abs(r_d_d[j]);
      if (ad > dfeas && !(j < n_orig && is_fixed[j])) dfeas = ad;
    }
    for (int i = 0; i < m; ++i) {
      double ap = std::abs(r_p_d[i]);
      if (ap > pfeas) pfeas = ap;
    }
    double mu = (n_compl > 0) ? comp_sum / n_compl : 0.0;
    last_pfeas = pfeas; last_dfeas = dfeas; last_mu = mu;

    if (pfeas < opt_.tol_primal && dfeas < opt_.tol_dual && mu < opt_.tol_gap) {
      out.stats.success = true;
      out.stats.iterations = iter;
      out.stats.primal_feas = pfeas;
      out.stats.dual_feas = dfeas;
      out.stats.complementarity = mu;
      out.stats.status = "Optimal";
      converged = true;
      break;
    }

    // Theta + factorize
    for (int j = 0; j < nn; ++j) {
      if (j < n_orig && is_fixed[j]) {
        inv_gl_d[j] = 0.0;
        inv_gu_d[j] = 0.0;
        theta_d[j] = 0.0;  // remove fixed var from normal equations
        continue;
      }
      double igl = flb_d[j] / gl_d[j];
      double igu = fub_d[j] / gu_d[j];
      inv_gl_d[j] = igl;
      inv_gu_d[j] = igu;
      double d = zl_d[j] * igl + zu_d[j] * igu;
      theta_d[j] = (d > kMinVal) ? 1.0 / d : 1e14;
    }

    bool factor_ok = use_banded ? fill_and_factor_banded() : fill_and_factor_sparse();
    if (!factor_ok) {
      // Dynamic regularization: retry with progressively larger reg
      double dyn_reg = std::max(reg * 1e4, 1e-6);
      for (int retry = 0; retry < 5 && !factor_ok; ++retry) {
        // Add extra regularization to diagonal
        if (use_banded) {
          // Restore the pristine matrix before perturbing (same fix as the
          // non-cached path): band_work was left partially factorized by
          // the failed attempt; band_storage_local holds the fresh fill.
          std::memcpy(band_work.data(), band_storage_local.data(),
                      sizeof(double) * band_storage_local.size());
          double* bs = band_work.data();
          for (int i = 0; i < m; ++i) bs[i] += dyn_reg;
          factor_ok = banded_chol_factor(band_work.data(), m, bw);
        } else {
          double* Nv = N_local.valuePtr();
          for (int i = 0; i < m; ++i) Nv[cs.sparse_diag_offsets[i]] += dyn_reg;
#if MIPSOLVERS_HAVE_CHOLMOD
          if (cs.cholmod_ok) {
            factor_ok = cs.cholmod.factorize(N_local.valuePtr());
          } else
#endif
#if MIPSOLVERS_USE_ACCELERATE
          {
          SparseMatrixStructure s_struct{};
          s_struct.rowCount = m;
          s_struct.columnCount = m;
          s_struct.columnStarts = accel_col_starts_local.data();
          s_struct.rowIndices = N_local.innerIndexPtr();
          s_struct.attributes.transpose = false;
          s_struct.attributes.triangle = SparseLowerTriangle;
          s_struct.blockSize = 1;
          SparseMatrix_Double s_mat{s_struct, N_local.valuePtr()};
          if (accel_numeric_valid) SparseCleanup(accel_numeric);
          accel_numeric = SparseFactor(cs.accel_symbolic, s_mat);
          accel_numeric_valid = (accel_numeric.status == SparseStatusOK);
          factor_ok = accel_numeric_valid;
          }
#else
          {
          ldlt_local.factorize(N_local);
          factor_ok = (ldlt_local.info() == Eigen::Success);
          }
#endif
        }
        dyn_reg *= 10.0;
      }
      if (!factor_ok) {
        out.stats.status = "Cholesky failed";
        out.stats.iterations = iter;
        break;
      }
    }

    // Predictor
    for (int j = 0; j < nn; ++j) {
      double xi = -r_d_d[j] - flb_d[j] * zl_d[j] + fub_d[j] * zu_d[j];
      tmp_d[j] = theta_d[j] * xi;
    }
    std::memcpy(rhs_d, r_p_d, sizeof(double) * m);
    ae_mul_sub(tmp_d, rhs_d);
    solve_normal(rhs_d, dy_aff_d);
    aet_mul(dy_aff_d, Atdy_d);
    simd_fma(theta_d, Atdy_d, tmp_d, dx_aff_d, nn);

    for (int j = 0; j < nn; ++j) {
      dzl_aff_d[j] = flb_d[j] * (-zl_d[j] - zl_d[j] * inv_gl_d[j] * dx_aff_d[j]);
      dzu_aff_d[j] = fub_d[j] * (-zu_d[j] + zu_d[j] * inv_gu_d[j] * dx_aff_d[j]);
    }

    double ap_aff = 1.0, ad_aff = 1.0;
    for (int j = 0; j < nn; ++j) {
      if (dx_aff_d[j] < -kMinVal && flb_d[j]) {
        double a = -kTau * gl_d[j] / dx_aff_d[j];
        if (a < ap_aff) ap_aff = a;
      }
      if (dx_aff_d[j] > kMinVal && fub_d[j]) {
        double a = kTau * gu_d[j] / dx_aff_d[j];
        if (a < ap_aff) ap_aff = a;
      }
      if (dzl_aff_d[j] < -kMinVal) {
        double a = -kTau * zl_d[j] / dzl_aff_d[j];
        if (a < ad_aff) ad_aff = a;
      }
      if (dzu_aff_d[j] < -kMinVal) {
        double a = -kTau * zu_d[j] / dzu_aff_d[j];
        if (a < ad_aff) ad_aff = a;
      }
    }
    ap_aff = std::max(ap_aff, kMinVal);
    ad_aff = std::max(ad_aff, kMinVal);

    double mu_aff = 0.0;
    for (int j = 0; j < nn; ++j) {
      mu_aff += flb_d[j] * (gl_d[j] + ap_aff * dx_aff_d[j]) * (zl_d[j] + ad_aff * dzl_aff_d[j]);
      mu_aff += fub_d[j] * (gu_d[j] - ap_aff * dx_aff_d[j]) * (zu_d[j] + ad_aff * dzu_aff_d[j]);
    }
    mu_aff = (n_compl > 0) ? mu_aff / n_compl : 0.0;
    // n_compl == 0 (no finite bounds anywhere) gives mu = mu_aff = 0, so
    // mu_aff / mu would be 0/0 = NaN.  With no complementarity there is
    // nothing to center — take a pure affine step instead of letting NaN
    // propagate into the corrector rhs and the returned solution.
    double sigma = (mu > 0.0) ? std::min(std::pow(mu_aff / mu, 3.0), 0.5) : 0.0;
    double sigma_mu = sigma * mu;

    double ap, ad;
    const bool skip_corrector = (ap_aff > 0.9 && ad_aff > 0.9 && sigma < 0.02);

    if (skip_corrector) {
      ap = ap_aff;
      ad = ad_aff;
      std::memcpy(dx_d, dx_aff_d, sizeof(double) * nn);
      std::memcpy(dy_d, dy_aff_d, sizeof(double) * m);
      std::memcpy(dzl.data(), dzl_aff_d, sizeof(double) * nn);
      std::memcpy(dzu.data(), dzu_aff_d, sizeof(double) * nn);
    } else {
      // Corrector
      for (int j = 0; j < nn; ++j) {
        double xi = -r_d_d[j];
        xi += flb_d[j] * (sigma_mu * inv_gl_d[j] - zl_d[j] - dzl_aff_d[j] * dx_aff_d[j] * inv_gl_d[j]);
        xi -= fub_d[j] * (sigma_mu * inv_gu_d[j] - zu_d[j] + dzu_aff_d[j] * dx_aff_d[j] * inv_gu_d[j]);
        tmp_d[j] = theta_d[j] * xi;
      }
      std::memcpy(rhs_d, r_p_d, sizeof(double) * m);
      ae_mul_sub(tmp_d, rhs_d);
      solve_normal(rhs_d, dy_d);
      aet_mul(dy_d, Atdy_d);
      simd_fma(theta_d, Atdy_d, tmp_d, dx_d, nn);

      for (int j = 0; j < nn; ++j) {
        dzl[j] = flb_d[j] * ((sigma_mu - dzl_aff_d[j] * dx_aff_d[j]) * inv_gl_d[j] - zl_d[j] - zl_d[j] * inv_gl_d[j] * dx_d[j]);
        dzu[j] = fub_d[j] * ((sigma_mu + dzu_aff_d[j] * dx_aff_d[j]) * inv_gu_d[j] - zu_d[j] + zu_d[j] * inv_gu_d[j] * dx_d[j]);
      }

      ap = 1.0; ad = 1.0;
      for (int j = 0; j < nn; ++j) {
        if (dx_d[j] < -kMinVal && flb_d[j]) {
          double a = -kTau * gl_d[j] / dx_d[j];
          if (a < ap) ap = a;
        }
        if (dx_d[j] > kMinVal && fub_d[j]) {
          double a = kTau * gu_d[j] / dx_d[j];
          if (a < ap) ap = a;
        }
        if (dzl[j] < -kMinVal) {
          double a = -kTau * zl_d[j] / dzl[j];
          if (a < ad) ad = a;
        }
        if (dzu[j] < -kMinVal) {
          double a = -kTau * zu_d[j] / dzu[j];
          if (a < ad) ad = a;
        }
      }
      ap = std::max(ap, kMinVal);
      ad = std::max(ad, kMinVal);
    }

    // Finiteness guard (same as the non-cached path): abort on a NaN
    // search direction instead of returning a NaN "solution" after
    // MaxIter.  dzl/dzu derive from dx, so dx/dy cover all NaN sources.
    bool step_finite = true;
    for (int j = 0; j < nn; ++j)
      if (!std::isfinite(dx_d[j])) { step_finite = false; break; }
    if (step_finite)
      for (int i = 0; i < m; ++i)
        if (!std::isfinite(dy_d[i])) { step_finite = false; break; }
    if (!step_finite) {
      out.stats.status = "NumericalError";
      out.stats.iterations = iter;
      break;
    }

    // Update
    simd_axpy(ap, dx_d, x_d, nn);
    simd_axpy(ad, dy_d, y_d, m);
    simd_axpy_max(ad, dzl.data(), zl_d, kMinVal, nn);
    simd_axpy_max(ad, dzu.data(), zu_d, kMinVal, nn);
    simd_sub_max(x_d, lb_d, gl_d, kMinVal, nn);
    simd_rsub_max(x_d, ub_d, gu_d, kMinVal, nn);
    // Re-lock fixed variables (prevent drift from numerical arithmetic)
    for (int j = 0; j < n_orig; ++j) {
      if (is_fixed[j]) {
        x_d[j] = lb[j];
        gl_d[j] = kBig;
        gu_d[j] = kBig;
      }
    }
  }

  // Cleanup
#if MIPSOLVERS_USE_ACCELERATE
  if (!use_banded && accel_numeric_valid) SparseCleanup(accel_numeric);
#endif

  if (!converged && out.stats.status.empty()) {
    out.stats.status = "MaxIter";
    out.stats.success = false;
    out.stats.iterations = max_iter;
    out.stats.primal_feas = last_pfeas;
    out.stats.dual_feas = last_dfeas;
    out.stats.complementarity = last_mu;
  }
  out.x.resize(n_orig);
  for (int j = 0; j < n_orig; ++j)
    out.x(j) = scaling ? x_d[j] * cs.scal_dc[static_cast<size_t>(j)] : x_d[j];
  out.stats.objective = cs.base_lp_copy.c.dot(out.x);
  if (m > 0) {
    out.constraint_duals.resize(m);
    for (int i = 0; i < m; ++i) {
      // G-type rows were negated to L-type at prepare time: flip back.
      const double row_sign =
          (!cs.flip_row.empty() && i < mi &&
           cs.flip_row[static_cast<size_t>(i)] != 0) ? -1.0 : 1.0;
      out.constraint_duals(i) = row_sign * sense_sign * y_d[i] *
                                (scaling ? cs.scal_dr[static_cast<size_t>(i)] : 1.0);
    }
  }
  out.box_dual_lb.resize(n_orig);
  out.box_dual_ub.resize(n_orig);
  for (int j = 0; j < n_orig; ++j) {
    out.box_dual_lb(j) = scaling ? zl_d[j] / cs.scal_dc[static_cast<size_t>(j)]
                                 : zl_d[j];
    out.box_dual_ub(j) = scaling ? zu_d[j] / cs.scal_dc[static_cast<size_t>(j)]
                                 : zu_d[j];
  }
  out.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  out.stats.residual_inf = std::max(out.stats.primal_feas, out.stats.dual_feas);

  // Save converged dual for warm-starting subsequent node LPs.
  if (out.stats.success) {
    cs.last_dual.resize(m);
    std::memcpy(cs.last_dual.data(), y_d, sizeof(double) * m);
    cs.last_zl.resize(nn);
    cs.last_zu.resize(nn);
    std::memcpy(cs.last_zl.data(), zl_d, sizeof(double) * nn);
    std::memcpy(cs.last_zu.data(), zu_d, sizeof(double) * nn);
  }

  if (opt_.verbose)
    fprintf(stderr, "[IPM-CACHE] done: iters=%d success=%d time=%.1fms status=%s\n",
            out.stats.iterations, (int)out.stats.success,
            out.stats.runtime_sec * 1000.0, out.stats.status.c_str());
  return out;
}

std::vector<IPMNodeBatchEntry>
NativeIPMLPAdapter::solve_cached_bound_change_batch(
    const Eigen::VectorXd& parent_lb,
    const Eigen::VectorXd& parent_ub,
    const Eigen::VectorXd& x0,
    const std::vector<IPMNodeBoundChange>& changes,
    double batch_time_limit_sec) {
  std::vector<IPMNodeBatchEntry> entries(changes.size());
  if (changes.empty()) return entries;

  const int n = cached_state_ ? cached_state_->n_orig : 0;
  if (n <= 0 || parent_lb.size() != n || parent_ub.size() != n ||
      (x0.size() != 0 && x0.size() != n)) {
    for (auto& entry : entries) {
      entry.result.stats.solver_name = name();
      entry.result.stats.status = "Invalid cached IPM batch domain";
    }
    return entries;
  }

  Eigen::VectorXd node_lb = parent_lb;
  Eigen::VectorXd node_ub = parent_ub;
  const bool use_augmented_direct =
      !cached_state_->use_banded && cached_state_->m > 256 &&
      cached_state_->bandwidth > cached_state_->m / 4;
  LPModel direct_lp;
  if (use_augmented_direct) {
    direct_lp = cached_state_->base_lp_copy;
    for (int j = 0; j < n; ++j) {
      direct_lp.vars[static_cast<std::size_t>(j)].lb = parent_lb[j];
      direct_lp.vars[static_cast<std::size_t>(j)].ub = parent_ub[j];
    }
  }
  const double previous_solve_limit = opt_.time_limit_sec;
  const bool has_batch_deadline =
      std::isfinite(batch_time_limit_sec) && batch_time_limit_sec > 0.0;
  const auto batch_start = std::chrono::steady_clock::now();
  int previous_var = -1;

  for (std::size_t i = 0; i < changes.size(); ++i) {
    if (previous_var >= 0) {
      node_lb[previous_var] = parent_lb[previous_var];
      node_ub[previous_var] = parent_ub[previous_var];
      if (use_augmented_direct) {
        direct_lp.vars[static_cast<std::size_t>(previous_var)].lb =
            parent_lb[previous_var];
        direct_lp.vars[static_cast<std::size_t>(previous_var)].ub =
            parent_ub[previous_var];
      }
      previous_var = -1;
    }

    auto& entry = entries[i];
    entry.result.stats.solver_name = name();
    const auto& change = changes[i];
    if (change.variable < 0 || change.variable >= n ||
        std::isnan(parent_lb[change.variable]) ||
        std::isnan(parent_ub[change.variable]) ||
        parent_lb[change.variable] > parent_ub[change.variable]) {
      entry.result.stats.status = "Invalid cached IPM batch bound change";
      continue;
    }

    node_lb[change.variable] =
        std::max(parent_lb[change.variable], change.lower_bound);
    node_ub[change.variable] =
        std::min(parent_ub[change.variable], change.upper_bound);
    previous_var = change.variable;
    if (node_lb[change.variable] > node_ub[change.variable]) {
      entry.result.stats.status = "Infeasible (batch bounds)";
      continue;
    }
    if (use_augmented_direct) {
      direct_lp.vars[static_cast<std::size_t>(change.variable)].lb =
          node_lb[change.variable];
      direct_lp.vars[static_cast<std::size_t>(change.variable)].ub =
          node_ub[change.variable];
    }

    double solve_limit = previous_solve_limit;
    if (has_batch_deadline) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - batch_start).count();
      const double remaining = batch_time_limit_sec - elapsed;
      if (remaining <= 0.001) {
        entry.result.stats.status = "Time limit (batch not started)";
        continue;
      }
      solve_limit = remaining /
          static_cast<double>(std::max<std::size_t>(1, changes.size() - i));
    }

    opt_.time_limit_sec = has_batch_deadline
        ? std::max(0.001, solve_limit)
        : previous_solve_limit;
    entry.attempted = true;
    entry.result = use_augmented_direct
        ? solve_lp_impl(direct_lp, x0, opt_.ruiz_rounds)
        : solve_cached_node_lp(node_lb, node_ub, x0);
  }

  opt_.time_limit_sec = previous_solve_limit;
  return entries;
}

}  // namespace mipsolvers::engine
