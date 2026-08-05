// High-performance Mehrotra Predictor-Corrector IPM for LP
//
// Solves: min c'x s.t. A x <= b, Aeq x = beq, lb <= x <= ub
//
// Bounded-variable formulation: handles bounds directly (no upper-bound slacks).
// Uses banded Cholesky for narrow-banded normal equations, sparse LDLT otherwise.

#include "ipm_lp_solver_internal.hpp"

namespace mipsolvers::engine {

NativeIPMLPAdapter::NativeIPMLPAdapter(IPMLPOptions opt)
    : opt_(std::move(opt)), accel_cache_(std::make_unique<AccelSparseCache>()) {}
NativeIPMLPAdapter::~NativeIPMLPAdapter() = default;
void NativeIPMLPAdapter::set_solve_time_limit(double time_limit_sec) {
  opt_.time_limit_sec =
      std::isfinite(time_limit_sec) ? std::max(0.0, time_limit_sec) : 0.0;
}
std::string NativeIPMLPAdapter::name() const { return "NativeIPMLP"; }
bool NativeIPMLPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LP;
}

SolveResult NativeIPMLPAdapter::solve_lp(const LPModel& prob) const {
  static const Eigen::VectorXd empty;
  return solve_lp(prob, empty);
}

SolveResult NativeIPMLPAdapter::solve_lp(const LPModel& prob, const Eigen::VectorXd& x0) const {
  // Direct solve with the standard Ruiz-escalation robustness ladder.
  auto direct_solve = [this](const LPModel& p,
                             const Eigen::VectorXd& start) -> SolveResult {
    // Scaling fallback: solve with the configured Ruiz rounds first; if it
    // fails to converge, retry without scaling.  Equilibration helps most
    // problems (e.g. NETLIB afiro) but stalls some degenerate ones
    // (e.g. stocfor1) — retrying unscaled is the standard robustness answer.
    SolveResult res = solve_lp_impl(p, start, opt_.ruiz_rounds);
    if (!res.stats.success && opt_.ruiz_rounds > 0) {
      SolveResult raw = solve_lp_impl(p, start, 0);
      if (raw.stats.success) {
        res = std::move(raw);
      } else {
        // Extreme-scaling escalation: tighter Ruiz equilibration shrinks the
        // residual column-scale range that otherwise leaves the barrier
        // ill-conditioned and stalled (adversarial 10^6-scaled probes).
        SolveResult more =
            solve_lp_impl(p, start, std::max(opt_.ruiz_rounds * 4, 40));
        if (more.stats.success) res = std::move(more);
      }
    }
    return res;
  };

  // Adaptive HiGHS presolve (opt-in via IPMLPOptions::use_highs_presolve or the
  // MIPSOLVERS_PRESOLVE env var).  Solve the reduced LP with the IPM, then
  // postsolve the primal to original space.  Only on cold solves (a warm start
  // refers to the original variable space).  Any failure falls through to a
  // direct solve, so a wrong or infeasible answer is never published.
  {
    HighsLpPresolveConfig pcfg;
    pcfg.enabled = opt_.use_highs_presolve;
    pcfg = highs_lp_presolve_config_from_env(pcfg);
    const bool has_warm_start = (x0.size() == prob.c.size());
    if (pcfg.enabled && !has_warm_start) {
      HighsLpPresolveResult ps = highs_presolve_lp(prob, pcfg);
      if (ps.infeasible) {
        SolveResult r;
        r.stats.solver_name = name();
        r.stats.success = false;
        r.stats.status = "Infeasible (presolve)";
        return r;
      }
      if (ps.solved_by_presolve || ps.use_reduced) {
        Eigen::VectorXd x_reduced;
        int iters = 0;
        bool reduced_ok = true;
        if (ps.use_reduced) {
          static const Eigen::VectorXd empty;
          SolveResult rr = direct_solve(ps.reduced, empty);
          reduced_ok = rr.stats.success;
          x_reduced = rr.x;
          iters = rr.stats.iterations;
        }  // else: reduced-to-empty -> postsolve an empty primal.
        if (reduced_ok) {
          Eigen::VectorXd x_orig;
          double obj = 0.0;
          if (highs_presolve_recover_primal(prob, ps, x_reduced, 1e-6, x_orig,
                                            obj)) {
            SolveResult r;
            r.x = std::move(x_orig);
            r.stats.solver_name = name();
            r.stats.success = true;
            r.stats.status = "Optimal";
            r.stats.objective = obj;
            r.stats.iterations = iters;
            return r;
          }
        }
        // Postsolve/audit/reduced-solve failure: fall through to a direct solve.
      }
    }
  }

  return direct_solve(prob, x0);
}

SolveResult NativeIPMLPAdapter::solve_lp_impl(const LPModel& prob, const Eigen::VectorXd& x0, int ruiz_rounds) const {
  const bool has_warm_start = (x0.size() == prob.c.size());
  const bool ipm_verbose_env = (std::getenv("MIPSOLVERS_IPM_VERBOSE") != nullptr);
  SolveResult out;
  out.stats.solver_name = name();
  const auto t0 = std::chrono::steady_clock::now();

  const int n_orig = static_cast<int>(prob.c.size());
  if (n_orig == 0) {
    out.stats.success = true;
    out.stats.status = "Empty";
    out.x = Eigen::VectorXd();
    return out;
  }

  const int sense_sign = (prob.sense == Sense::Maximize) ? -1 : 1;
  const int mi = static_cast<int>(prob.A.rows());
  const int me = static_cast<int>(prob.Aeq.rows());
  const int m = mi + me;
  const int nn = n_orig + mi;  // original vars + inequality slacks

  // === One-sided (G-type) row normalization ===
  // The IPM's slack form A x + s = b requires a finite upper bound on every
  // inequality row.  Rows of the form A x >= lhs (b = +inf) are negated to
  // -A x <= -lhs (L-type); constraint_duals for flipped rows are negated
  // back at extraction.  `lp` is the effective problem for all constraint
  // data below (identical to prob when no one-sided rows exist).
  std::vector<char> flip_row(static_cast<size_t>(mi), 0);
  bool any_flip = false;
  for (int i = 0; i < mi; ++i) {
    const double lhs_i = lp_row_lhs_or_neg_inf(prob, i);
    if (!std::isfinite(prob.b[i]) && std::isfinite(lhs_i)) {
      flip_row[static_cast<size_t>(i)] = 1;
      any_flip = true;
    }
  }
  LPModel norm_lp;
  if (any_flip) {
    norm_lp = prob;
    for (int k = 0; k < norm_lp.A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(norm_lp.A, k); it; ++it)
        if (flip_row[static_cast<size_t>(it.row())]) it.valueRef() = -it.value();
    for (int i = 0; i < mi; ++i) {
      if (!flip_row[static_cast<size_t>(i)]) continue;
      norm_lp.b[i] = -lp_row_lhs_or_neg_inf(prob, i);
      if (norm_lp.row_lhs.size() == norm_lp.A.rows())
        norm_lp.row_lhs[i] = -std::numeric_limits<double>::infinity();
    }
  }
  const LPModel& lp = any_flip ? norm_lp : prob;

  // === Ruiz equilibration (honors the caller's round count) ===
  // Scales A/Aeq/b/c and finite variable bounds; slacks keep coefficient 1.
  // Outputs are unscaled at extraction: x = Dc·x̂, y = Dr·ŷ, z = ẑ/Dc.
  const RuizScaling scal =
      (ruiz_rounds > 0 && m > 0)
          ? ruiz_equilibrate(lp.A, lp.Aeq, n_orig, ruiz_rounds)
          : RuizScaling{};

  // === Bounds — use double flags (0.0/1.0) for branchless arithmetic ===
  std::vector<double> lb(nn), ub(nn), flb(nn), fub(nn);
  for (int j = 0; j < n_orig; ++j) {
    double lo = (j < (int)prob.vars.size()) ? prob.vars[j].lb : -kBig;
    double hi = (j < (int)prob.vars.size()) ? prob.vars[j].ub : kBig;
    if (lo < -kBig + 1) lo = -kBig;
    if (hi > kBig - 1) hi = kBig;
    flb[j] = (lo > -kBig + 1) ? 1.0 : 0.0;
    fub[j] = (hi < kBig - 1) ? 1.0 : 0.0;
    if (scal.active) {
      const double inv_d = 1.0 / scal.dc[static_cast<size_t>(j)];
      if (flb[j]) lo *= inv_d;
      if (fub[j]) hi *= inv_d;
    }
    lb[j] = lo; ub[j] = hi;
  }
  for (int i = 0; i < mi; ++i) {
    lb[n_orig + i] = 0.0;
    const double lhs = lp_row_lhs_or_neg_inf(lp, i);
    // Both b_i and lhs_i scale with dr_i, so the slack range scales once.
    const double dri = scal.active ? scal.dr[static_cast<size_t>(i)] : 1.0;
    ub[n_orig + i] =
        std::isfinite(lhs) && std::isfinite(lp.b[i])
            ? std::max(0.0, dri * (lp.b[i] - lhs))
            : kBig;
    flb[n_orig + i] = 1.0;
    fub[n_orig + i] = (ub[n_orig + i] < kBig - 1) ? 1.0 : 0.0;
  }

  // === Detect fixed variables (lb ≈ ub) ===
  // Fixed vars are removed from the barrier formulation: clear flb/fub so they
  // don't generate ill-conditioned barrier terms (gl→0, zl→∞).  Their primal
  // value is locked at lb, theta set to 0 to remove them from the normal
  // equations, and they are excluded from the dual convergence check.
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

  // === Cost vector (with sense) ===
  std::vector<double> c(nn, 0.0);
  for (int j = 0; j < n_orig; ++j)
    c[j] = sense_sign * prob.c(j) *
           (scal.active ? scal.dc[static_cast<size_t>(j)] : 1.0);

  std::vector<double> b(m);
  for (int i = 0; i < mi; ++i)
    b[i] = (scal.active ? scal.dr[static_cast<size_t>(i)] : 1.0) * lp.b(i);
  for (int i = 0; i < me; ++i)
    b[mi + i] = (scal.active ? scal.dr[static_cast<size_t>(mi + i)] : 1.0) * lp.beq(i);

  // Deactivate spurious far bounds.  After Ruiz the constraint matrix is O(1),
  // so the solution magnitude is O(||b||inf); a variable bound many orders
  // beyond that scale cannot be active and only injects a huge barrier gap
  // (gu ~ 1e18) that pollutes mu and stalls the method.  This notably rescues
  // originally-infinite bounds whose 1e20 sentinel was scaled below kBig by a
  // large column factor (the adversarial 10^6 scaling probe).  Treating them as
  // unbounded is safe: the original-space feasibility audit rejects the rare
  // case where such a bound was genuinely active.
  {
    double bscale = 1.0;
    for (int i = 0; i < m; ++i) bscale = std::max(bscale, std::abs(b[i]));
    const double bound_cap = 1e8 * bscale;
    for (int j = 0; j < n_orig; ++j) {
      if (flb[j] != 0.0 && std::abs(lb[j]) > bound_cap) flb[j] = 0.0;
      if (fub[j] != 0.0 && std::abs(ub[j]) > bound_cap) fub[j] = 0.0;
    }
  }

  // CSC pointers — from the scaled copies when Ruiz is active, else direct
  const Eigen::SparseMatrix<double>& A_mat = scal.active ? scal.A : lp.A;
  const Eigen::SparseMatrix<double>& Aeq_mat = scal.active ? scal.Aeq : lp.Aeq;
  const int* A_o = A_mat.outerIndexPtr();
  const int* A_i = A_mat.innerIndexPtr();
  const double* A_v = A_mat.valuePtr();
  const int* Aeq_o = me > 0 ? Aeq_mat.outerIndexPtr() : nullptr;
  const int* Aeq_i = me > 0 ? Aeq_mat.innerIndexPtr() : nullptr;
  const double* Aeq_v = me > 0 ? Aeq_mat.valuePtr() : nullptr;

  // Build CSR for A (inequality) — needed for forward SpMV (sequential y writes)
  std::vector<int> A_rp, A_ci, Aeq_rp, Aeq_ci;
  std::vector<double> A_rv, Aeq_rv;
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
  if (mi > 0) build_csr(A_mat, A_rp, A_ci, A_rv);
  if (me > 0) build_csr(Aeq_mat, Aeq_rp, Aeq_ci, Aeq_rv);

  // === SpMV operations ===
  // y -= Ae * x  (CSR-based: sequential y writes)
  auto ae_mul_sub = [&](const double* MIPSOLVERS_RESTRICT x, double* MIPSOLVERS_RESTRICT y) {
    MIPSOLVERS_OMP_PARALLEL_IF(mi > MIPSOLVERS_OMP_THRESHOLD)
    for (int i = 0; i < mi; ++i) {
      double s = x[n_orig + i];
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        s += A_rv[p] * x[A_ci[p]];
      y[i] -= s;
    }
    MIPSOLVERS_OMP_PARALLEL_IF(me > MIPSOLVERS_OMP_THRESHOLD)
    for (int k = 0; k < me; ++k) {
      double s = 0.0;
      for (int p = Aeq_rp[k]; p < Aeq_rp[k + 1]; ++p)
        s += Aeq_rv[p] * x[Aeq_ci[p]];
      y[mi + k] -= s;
    }
  };

  // y = Ae' * w  (CSC-based: gather from w, natural for CSC)
  auto aet_mul = [&](const double* MIPSOLVERS_RESTRICT w, double* MIPSOLVERS_RESTRICT y) {
    MIPSOLVERS_OMP_PARALLEL_IF(n_orig > MIPSOLVERS_OMP_THRESHOLD)
    for (int j = 0; j < n_orig; ++j) {
      double s = 0.0;
      for (int p = A_o[j]; p < A_o[j + 1]; ++p)
        s += A_v[p] * w[A_i[p]];
      if (me > 0)
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p)
          s += Aeq_v[p] * w[mi + Aeq_i[p]];
      y[j] = s;
    }
    for (int i = 0; i < mi; ++i)
      y[n_orig + i] = w[i];
  };

  // Merged: r_p = b - Ae*x, r_d = c - Ae'*y (single CSC pass)
  auto compute_residuals = [&](const double* MIPSOLVERS_RESTRICT xv, const double* MIPSOLVERS_RESTRICT yv,
                               double* MIPSOLVERS_RESTRICT rp, double* MIPSOLVERS_RESTRICT rd) {
    std::memcpy(rp, b.data(), sizeof(double) * m);
    std::memcpy(rd, c.data(), sizeof(double) * nn);
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

  // === Compute bandwidth of N = AeΘAe' from A's and Aeq's structure ===
  int bandwidth = 0;
  // For original columns j: rows touched = {A's rows for col j} ∪ {mi + Aeq's rows for col j}
  for (int j = 0; j < n_orig; ++j) {
    int rmin = m, rmax = -1;
    for (int p = A_o[j]; p < A_o[j + 1]; ++p) {
      rmin = std::min(rmin, A_i[p]);
      rmax = std::max(rmax, A_i[p]);
    }
    if (me > 0) {
      for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p) {
        int r = mi + Aeq_i[p];
        rmin = std::min(rmin, r);
        rmax = std::max(rmax, r);
      }
    }
    if (rmax >= 0) bandwidth = std::max(bandwidth, rmax - rmin);
  }
  // Slack columns: each touches exactly 1 row, so bandwidth contribution = 0

  const bool use_banded = (bandwidth <= kBandedThreshold);
  bool use_dense = false;  // set below when scatter table would exceed kDenseScatterThreshold
  // Sparse augmented-KKT path.  When the normal equations N = Ae*Ae' would be
  // too dense to form (large SCUC/DC-power-flow LPs — the ProblemTooLarge gate
  // below), solve the sparse quasidefinite augmented system
  //   [ diag(d)+reg   -Ae' ] [dx]   [ xi ]
  //   [ -Ae           -reg ] [dy] = [-r_p]
  // directly via factor_kkt_sparse/solve_kkt_sparse instead — this avoids the
  // dense Ae*Ae' fill entirely.  Off by default (env override for testing).
  bool use_augmented =
      !use_banded && (std::getenv("MIPSOLVERS_IPM_FORCE_AUGMENTED") != nullptr);
#ifdef MIPSOLVERS_HAVE_CHOLMOD
  // Dense normal equations (nearly-full bandwidth — SCUC / DC power flow, whose
  // transmission constraints couple everything) make N = Ae*Ae' expensive to
  // form and ill-conditioned.  The sparse augmented KKT is both better
  // conditioned (fewer IPM iterations) and cheaper (no Ae*Ae' fill), so route
  // there whenever CHOLMOD's simplicial LDLT can factor the quasidefinite
  // system.  Measured ~1.7x faster on 39-bus (55 vs 114 iters) and ~10x on the
  // 6-bus relaxation.  Small/sparse-normal problems keep the parallel
  // Accelerate Cholesky path where it is faster.
  if (!use_banded && m > 256 && bandwidth > m / 4) use_augmented = true;
#endif

  // === Banded storage: band[(row-col)*m + col] for row >= col, row-col <= bw ===
  const int bw = bandwidth;
  std::vector<double> band_storage;
  std::vector<BandScatterEntry> band_scatter;
  std::vector<int> band_scatter_col_start;

  // === Sparse path — normal equations N = Ae·Θ·Ae' ===
  Eigen::SparseMatrix<double> N_sparse;
#if MIPSOLVERS_HAVE_CHOLMOD
  // CHOLMOD is the preferred sparse Cholesky backend (supernodal BLAS-3).
  CholmodLDLT cholmod_ldlt;
#endif
  // True when the CHOLMOD backend owns the sparse normal-equations path;
  // false when CHOLMOD is not compiled or analyze failed, in which case
  // the platform backend below (Accelerate / Eigen) handles factorization.
  bool cholmod_ok = false;
#if MIPSOLVERS_USE_ACCELERATE
  std::vector<long> accel_col_starts;
  SparseOpaqueSymbolicFactorization accel_symbolic{};
  SparseOpaqueFactorization_Double accel_numeric{};
  bool accel_numeric_valid = false;
#else
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, Eigen::AMDOrdering<int>> ldlt;
#endif
  // Scatter entries for sparse path
  struct SparseScatterEntry { int ni; double a_prod; };
  std::vector<SparseScatterEntry> sparse_scatter;
  std::vector<int> sparse_scatter_col_start;
  std::vector<int> sparse_diag_offsets;
  int sparse_n_nnz = 0;

  // Dense BLAS path (activated when A is too dense for the scatter approach)
  Eigen::MatrixXd Ae_dense, Ae_sqrt, N_dense;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_dense;

  if (use_banded) {
    band_storage.resize(static_cast<size_t>(bw + 1) * m, 0.0);

    // Build banded scatter map for original variable columns
    // Column j of Ae touches: A's rows for col j (at rows 0..mi-1)
    //                         plus Aeq's rows for col j (at rows mi..m-1)
    // Tight reserve bound: per column, each row pairs only with rows
    // inside the band (r1 - r2 <= bw), i.e. at most min(nz, bw+1) partners
    // per row.  Reserving nz^2 would over-allocate — a single dense column
    // (nz = 1e5) would request 1e10 entries -> bad_alloc.
    size_t total = 0;
    for (int j = 0; j < n_orig; ++j) {
      const size_t nz =
          static_cast<size_t>(A_o[j + 1] - A_o[j]) +
          (me > 0 ? static_cast<size_t>(Aeq_o[j + 1] - Aeq_o[j]) : 0);
      total += nz * std::min(nz, static_cast<size_t>(bw) + 1);
    }
    band_scatter.reserve(total);
    band_scatter_col_start.resize(n_orig + 1);
    band_scatter_col_start[0] = 0;

    // Temporary buffer for rows/values of column j in Ae
    std::vector<int> col_rows;
    std::vector<double> col_vals;

    for (int j = 0; j < n_orig; ++j) {
      col_rows.clear();
      col_vals.clear();
      for (int p = A_o[j]; p < A_o[j + 1]; ++p) {
        col_rows.push_back(A_i[p]);
        col_vals.push_back(A_v[p]);
      }
      if (me > 0) {
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p) {
          col_rows.push_back(mi + Aeq_i[p]);
          col_vals.push_back(Aeq_v[p]);
        }
      }
      int nz = static_cast<int>(col_rows.size());
      for (int pi = 0; pi < nz; ++pi) {
        int r1 = col_rows[pi];
        double a1 = col_vals[pi];
        for (int pk = 0; pk < nz; ++pk) {
          int r2 = col_rows[pk];
          if (r1 < r2) continue;  // lower triangle only
          double a2 = col_vals[pk];
          int d = r1 - r2;
          if (d <= bw) {
            band_scatter.push_back({d * m + r2, a1 * a2});
          }
        }
      }
      band_scatter_col_start[j + 1] = static_cast<int>(band_scatter.size());
    }
  } else {
    // Estimate scatter table size before committing to the sparse path.
    // For dense A (e.g. random LP benchmarks), each column has O(m) non-zeros,
    // making the scatter O(n*m^2) entries — catastrophic in both memory and time.
    // Switch to a dense BLAS path in that case.
    {
      size_t estimated_scatter = 0;
      for (int j = 0; j < n_orig; ++j) {
        size_t nz_j = static_cast<size_t>(A_o[j + 1] - A_o[j]);
        if (me > 0) nz_j += static_cast<size_t>(Aeq_o[j + 1] - Aeq_o[j]);
        estimated_scatter += nz_j * nz_j;
      }
      // Memory feasibility gate: the dense path allocates m x nn (Ae_dense,
      // Ae_sqrt) plus m x m (N_dense).  Huge sparse LPs (n ~ 1e6) always
      // exceed the scatter threshold, which would request terabytes of
      // dense storage — stay sparse unless the dominant m x nn matrix
      // fits a bounded budget.
      const size_t dense_bytes =
          sizeof(double) * static_cast<size_t>(m) * static_cast<size_t>(nn);
      use_dense = (estimated_scatter > kDenseScatterThreshold) &&
                  (dense_bytes <= kDenseMaxBytes);
      if (!use_dense && estimated_scatter > kMaxScatterEntries) {
        // The dense normal-equations matrices exceed the byte budget and the
        // sparse N = Ae*Ae' scatter map exceeds its cap.  Rather than fail,
        // route to the sparse augmented-KKT path, which never forms Ae*Ae'.
        use_augmented = true;
      }
    }

    if (use_augmented) {
      // Skip all normal-equations setup; the augmented path builds its own
      // operands (Ae, diag(d)) below.
    } else if (use_dense) {
      // Dense BLAS path: build Ae_dense (m x nn) once.  Each IPM iteration computes
      //   N = Ae_sqrt * Ae_sqrt'  where Ae_sqrt[:,k] = sqrt(theta[k]) * Ae[:,k]
      // via Eigen (backed by BLAS/Accelerate), then factorizes with dense LDLT.
      // Cost per iter: O(m*nn) scale + O(m^2*nn) syrk + O(m^3/3) factorization.
      Ae_dense.resize(m, nn);
      Ae_dense.setZero();
      for (int j = 0; j < n_orig; ++j) {
        for (int p = A_o[j]; p < A_o[j + 1]; ++p)
          Ae_dense(A_i[p], j) = A_v[p];
        if (me > 0)
          for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p)
            Ae_dense(mi + Aeq_i[p], j) = Aeq_v[p];
      }
      for (int i = 0; i < mi; ++i)
        Ae_dense(i, n_orig + i) = 1.0;
      Ae_sqrt.resize(m, nn);
      N_dense.resize(m, m);
    } else {
      // Sparse LDLT path — build Ae and compute N_sparse = Ae * Ae'
      using T = Eigen::Triplet<double>;
      std::vector<T> trips;
      trips.reserve(prob.A.nonZeros() + prob.Aeq.nonZeros() + mi);
      for (int k = 0; k < prob.A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(prob.A, k); it; ++it)
          trips.emplace_back(it.row(), it.col(), it.value());
      for (int i = 0; i < mi; ++i)
        trips.emplace_back(i, n_orig + i, 1.0);
      for (int k = 0; k < prob.Aeq.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(prob.Aeq, k); it; ++it)
          trips.emplace_back(mi + it.row(), it.col(), it.value());
      Eigen::SparseMatrix<double> Ae(m, nn);
      Ae.setFromTriplets(trips.begin(), trips.end());
      Ae.makeCompressed();
      const int* Ao = Ae.outerIndexPtr();
      const int* Ai = Ae.innerIndexPtr();
      const double* Av = Ae.valuePtr();

      N_sparse = Ae * Ae.transpose();
      for (int i = 0; i < m; ++i) N_sparse.coeffRef(i, i) += 1e-10;
      // Store only lower triangle: CHOLMOD requires this, and Eigen SimplicialLDLT
      // with Eigen::Lower also reads only the lower triangle.
      N_sparse = N_sparse.triangularView<Eigen::Lower>();
      N_sparse.makeCompressed();
      sparse_n_nnz = static_cast<int>(N_sparse.nonZeros());

      const int* No = N_sparse.outerIndexPtr();
      const int* Ni = N_sparse.innerIndexPtr();
      size_t total = 0;
      for (int j = 0; j < n_orig; ++j) {
        int nz = Ao[j + 1] - Ao[j];
        total += static_cast<size_t>(nz) * nz;
      }
      sparse_scatter.reserve(total);
      sparse_scatter_col_start.resize(n_orig + 1);
      sparse_scatter_col_start[0] = 0;
      for (int j = 0; j < n_orig; ++j) {
        for (int pi = Ao[j]; pi < Ao[j + 1]; ++pi)
          for (int pk = Ao[j]; pk < Ao[j + 1]; ++pk) {
            int r1 = Ai[pi], r2 = Ai[pk];
            const int* pos = std::lower_bound(Ni + No[r2], Ni + No[r2 + 1], r1);
            if (pos != Ni + No[r2 + 1] && *pos == r1)
              sparse_scatter.push_back({static_cast<int>(pos - Ni), Av[pi] * Av[pk]});
          }
        sparse_scatter_col_start[j + 1] = static_cast<int>(sparse_scatter.size());
      }
      sparse_diag_offsets.resize(m);
      for (int i = 0; i < m; ++i) {
        const int* pos = std::lower_bound(Ni + No[i], Ni + No[i + 1], i);
        sparse_diag_offsets[i] = static_cast<int>(pos - Ni);
      }
#if MIPSOLVERS_HAVE_CHOLMOD && !MIPSOLVERS_USE_ACCELERATE
      // CHOLMOD symbolic analysis — once per sparsity pattern; the numeric
      // factorization later re-reads the aliased N_sparse.valuePtr().
      // Priority: Accelerate on macOS (measured ~2.7x faster factorization
      // than CHOLMOD on the same BLAS), then CHOLMOD, then Eigen.
      cholmod_ok = cholmod_ldlt.analyze(m, N_sparse.outerIndexPtr(),
                                        N_sparse.innerIndexPtr(),
                                        N_sparse.valuePtr(), sparse_n_nnz);
#endif
#if !MIPSOLVERS_USE_ACCELERATE
      if (!cholmod_ok) ldlt.analyzePattern(N_sparse);
#endif
    }
  }

#if MIPSOLVERS_USE_ACCELERATE
  // Apple Accelerate sparse Cholesky setup (after N_sparse is built).
  // Helper lambdas to create Apple Sparse wrappers around N_sparse data.
  auto make_apple_structure = [&]() {
    SparseMatrixStructure s{};
    s.rowCount = m;
    s.columnCount = m;
    s.columnStarts = accel_col_starts.data();
    s.rowIndices = N_sparse.innerIndexPtr();
    s.attributes.transpose = false;
    s.attributes.triangle = SparseLowerTriangle;
    s.attributes.kind = SparseSymmetric;
    s.attributes._reserved = 0;
    s.attributes._allocatedBySparse = false;
    s.blockSize = 1;
    return s;
  };

  auto make_apple_matrix = [&]() {
    SparseMatrix_Double mat{};
    mat.structure = make_apple_structure();
    mat.data = N_sparse.valuePtr();
    return mat;
  };

  if (!use_banded && !use_dense && !use_augmented && !cholmod_ok) {
    // Convert Eigen CSC column starts (int) to Apple format (long)
    const int* No = N_sparse.outerIndexPtr();
    accel_col_starts.resize(m + 1);
    for (int i = 0; i <= m; ++i)
      accel_col_starts[i] = static_cast<long>(No[i]);

    // Create or reuse symbolic factorization
    if (accel_cache_ && accel_cache_->valid &&
        accel_cache_->cached_m == m &&
        accel_cache_->cached_nnz == sparse_n_nnz) {
      accel_symbolic = accel_cache_->symbolic;
    } else {
      accel_symbolic = SparseFactor(SparseFactorizationCholesky, make_apple_structure());
      if (accel_cache_) {
        if (accel_cache_->valid) SparseCleanup(accel_cache_->symbolic);
        accel_cache_->symbolic = accel_symbolic;
        accel_cache_->col_starts = accel_col_starts;
        accel_cache_->cached_m = m;
        accel_cache_->cached_nnz = sparse_n_nnz;
        accel_cache_->valid = true;
      }
    }
  }
#endif

  // === Initialization ===
  std::vector<double> xv(nn), yv(m);
  std::vector<double> gl(nn), gu(nn), zl(nn), zu(nn);
  double* x_d = xv.data();
  double* y_d = yv.data();
  double* gl_d = gl.data();
  double* gu_d = gu.data();
  double* zl_d = zl.data();
  double* zu_d = zu.data();

  // ae_mul for init (CSR-based)
  auto ae_mul = [&](const double* MIPSOLVERS_RESTRICT x, double* MIPSOLVERS_RESTRICT y) {
    MIPSOLVERS_OMP_PARALLEL_IF(mi > MIPSOLVERS_OMP_THRESHOLD)
    for (int i = 0; i < mi; ++i) {
      double s = x[n_orig + i];
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        s += A_rv[p] * x[A_ci[p]];
      y[i] = s;
    }
    MIPSOLVERS_OMP_PARALLEL_IF(me > MIPSOLVERS_OMP_THRESHOLD)
    for (int k = 0; k < me; ++k) {
      double s = 0.0;
      for (int p = Aeq_rp[k]; p < Aeq_rp[k + 1]; ++p)
        s += Aeq_rv[p] * x[Aeq_ci[p]];
      y[mi + k] = s;
    }
  };

  // Initial point from least-squares (cold) or warm-start (from x0)
  {
    if (has_warm_start) {
      // Warm-start: use provided x0 as initial primal point.
      // Set original variables from x0, compute slacks from constraints.
      for (int j = 0; j < n_orig; ++j) {
        double lo = flb[j] ? lb[j] : -kBig;
        double hi = fub[j] ? ub[j] : kBig;
        // Caller-provided x0 is in original coordinates: x̂0 = x0 / Dc.
        const double x0j =
            scal.active ? x0[j] / scal.dc[static_cast<size_t>(j)] : x0[j];
        if (hi - lo < 2e-6) {
          x_d[j] = 0.5 * (lo + hi);
        } else {
          x_d[j] = std::clamp(x0j, lo + 1e-6, hi - 1e-6);
        }
      }
      // Compute inequality slacks: s_i = b_i - A_i * x  (must be > 0)
      for (int i = 0; i < mi; ++i) {
        double ax = 0.0;
        for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
          ax += A_rv[p] * x_d[A_ci[p]];
        // Interior slack, finite even for G-type rows (b = +inf) and for
        // inits violating the row's two-sided range — otherwise the barrier
        // gap gu = ub - s goes negative/inf and NaNs the normal equations.
        double s_init = b[i] - ax;
        if (!std::isfinite(s_init)) s_init = 1.0;
        x_d[n_orig + i] =
            std::clamp(s_init, 1e-4, std::max(1e-4, ub[n_orig + i] - 1e-4));
      }
      // Dual initial: y = 0 (least-squares dual was tested but adds ~1ms
      // factorization overhead with no iteration reduction — primal warm-start
      // already drives convergence).
      std::fill(yv.begin(), yv.end(), 0.0);
    } else if (use_dense) {
      // Dense mode init: N = Ae_dense * Ae_dense' (Theta=I), solve for x and y.
      N_dense.noalias() = Ae_dense * Ae_dense.transpose();
      N_dense.diagonal().array() += 1e-10;
      ldlt_dense.compute(N_dense);
      if (ldlt_dense.info() == Eigen::Success) {
        Eigen::VectorXd tmp = ldlt_dense.solve(
            Eigen::Map<const Eigen::VectorXd>(b.data(), m));
        aet_mul(tmp.data(), x_d);
        std::vector<double> tmp2(m);
        ae_mul(c.data(), tmp2.data());
        Eigen::VectorXd yy = ldlt_dense.solve(
            Eigen::Map<const Eigen::VectorXd>(tmp2.data(), m));
        std::memcpy(y_d, yy.data(), sizeof(double) * m);
      } else {
        std::fill(xv.begin(), xv.end(), 0.5);
        std::fill(yv.begin(), yv.end(), 0.0);
      }
    } else if (use_banded) {
      std::fill(band_storage.begin(), band_storage.end(), 0.0);
      for (int j = 0; j < n_orig; ++j)
        for (int si = band_scatter_col_start[j]; si < band_scatter_col_start[j + 1]; ++si)
          band_storage[band_scatter[si].offset] += band_scatter[si].a_prod;
      for (int i = 0; i < mi; ++i) band_storage[i] += 1.0;
      for (int i = 0; i < m; ++i) band_storage[i] += 1e-10;

      std::vector<double> bw_init(band_storage);
      if (banded_chol_factor(bw_init.data(), m, bw)) {
        std::memcpy(y_d, b.data(), sizeof(double) * m);
        banded_chol_solve(bw_init.data(), m, bw, y_d);
        aet_mul(y_d, x_d);

        ae_mul(c.data(), y_d);
        banded_chol_solve(bw_init.data(), m, bw, y_d);
      } else {
        std::fill(xv.begin(), xv.end(), 0.5);
        std::fill(yv.begin(), yv.end(), 0.0);
      }
    } else if (!use_augmented) {
#if MIPSOLVERS_HAVE_CHOLMOD
      if (cholmod_ok) {
        // N_sparse currently holds the theta=1 fill (N = Ae*Ae' + reg).
        if (cholmod_ldlt.factorize(N_sparse.valuePtr())) {
          // x_init = Ae' * (N \ b)
          std::memcpy(y_d, b.data(), sizeof(double) * m);
          cholmod_ldlt.solve(y_d, y_d);
          aet_mul(y_d, x_d);
          // y_init = N \ (Ae * c)
          std::vector<double> tmp2(m);
          ae_mul(c.data(), tmp2.data());
          cholmod_ldlt.solve(tmp2.data(), tmp2.data());
          std::memcpy(y_d, tmp2.data(), sizeof(double) * m);
        } else {
          std::fill(xv.begin(), xv.end(), 0.5);
          std::fill(yv.begin(), yv.end(), 0.0);
        }
      } else
#endif
#if MIPSOLVERS_USE_ACCELERATE
      {
      SparseOpaqueFactorization_Double init_fac = SparseFactor(accel_symbolic, make_apple_matrix());
      if (init_fac.status == SparseStatusOK) {
        // x_init = Ae' * (N \ b)
        std::memcpy(y_d, b.data(), sizeof(double) * m);
        DenseVector_Double rhs1{};
        rhs1.count = m;
        rhs1.data = y_d;
        SparseSolve(init_fac, rhs1);
        aet_mul(y_d, x_d);
        // y_init = N \ (Ae * c)
        std::vector<double> tmp2(m);
        ae_mul(c.data(), tmp2.data());
        DenseVector_Double rhs2{};
        rhs2.count = m;
        rhs2.data = tmp2.data();
        SparseSolve(init_fac, rhs2);
        std::memcpy(y_d, tmp2.data(), sizeof(double) * m);
      } else {
        std::fill(xv.begin(), xv.end(), 0.5);
        std::fill(yv.begin(), yv.end(), 0.0);
      }
      SparseCleanup(init_fac);
      }
#else
      {
      ldlt.factorize(N_sparse);
      if (ldlt.info() == Eigen::Success) {
        Eigen::Map<Eigen::VectorXd> bmap(b.data(), m);
        Eigen::VectorXd tmp = ldlt.solve(bmap);
        aet_mul(tmp.data(), x_d);
        std::vector<double> tmp2(m);
        ae_mul(c.data(), tmp2.data());
        Eigen::Map<Eigen::VectorXd> t2map(tmp2.data(), m);
        Eigen::VectorXd yy = ldlt.solve(t2map);
        std::memcpy(y_d, yy.data(), sizeof(double) * m);
      } else {
        std::fill(xv.begin(), xv.end(), 0.5);
        std::fill(yv.begin(), yv.end(), 0.0);
      }
      }
#endif
    }

    // Lock fixed variables at their exact value before general clamping.
    for (int j = 0; j < n_orig; ++j) {
      if (is_fixed[j]) x_d[j] = lb[j];
    }

    // If the least-squares init produced non-finite iterates or
    // astronomically large ones (near-singular initial normal matrix —
    // e.g. problems with many one-sided rows), fall back to the plain
    // interior point instead of propagating inf/huge values into the
    // barrier (comp_sum/mu or pf = inf at iteration 0).
    bool init_ok = true;
    for (int j = 0; j < nn && init_ok; ++j) {
      const double xj = x_d[j];
      init_ok = std::isfinite(xj) && std::abs(xj) < 1e10;
    }
    for (int i = 0; i < m && init_ok; ++i) {
      const double yi = y_d[i];
      init_ok = std::isfinite(yi) && std::abs(yi) < 1e10;
    }
    if (!init_ok) {
      std::fill(xv.begin(), xv.end(), 0.5);
      std::fill(yv.begin(), yv.end(), 0.0);
    }

    // Clamp original variables strictly inside their bounds, then recompute
    // slacks.  For one-sided variables the interior point must be built from
    // the *bounded* side only: the old `hi = x + 10` (lb-only case) fell below
    // lb whenever the least-squares init was negative, seeding x < lb and
    // leaving a permanent bound violation that the barrier gl = max(x-lb, eps)
    // silently hid (observed on afiro variable 24: x=-50, lb=0).
    for (int j = 0; j < n_orig; ++j) {
      if (is_fixed[j]) continue;  // already locked
      if (flb[j] && fub[j]) {
        const double range = ub[j] - lb[j];
        // Keep the (accurate) least-squares value when it is already interior;
        // only pull it off a bound by a small margin.  A fixed 1%-of-range
        // margin is catastrophic when the range is huge (extreme column
        // scaling): it would force x ~ 1e16 and destroy a good x_LS ~ 1e4,
        // seeding a starting point with pf ~ 1e18.
        const double margin = std::min(0.01 * range, 1.0);
        x_d[j] = std::clamp(x_d[j], lb[j] + margin, ub[j] - margin);
      } else if (flb[j]) {
        x_d[j] = std::max(x_d[j], lb[j] + 1.0);
      } else if (fub[j]) {
        x_d[j] = std::min(x_d[j], ub[j] - 1.0);
      }
      // free variables: keep the least-squares value as-is
    }
    // Recompute inequality slack variables: s_i = b_i - A_i * x (must be > 0)
    for (int i = 0; i < mi; ++i) {
      double ax = 0.0;
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        ax += A_rv[p] * x_d[A_ci[p]];
      // Interior slack, finite even for G-type rows (b = +inf) and for
      // inits violating the row's two-sided range (see warm-start path).
      double s_init = b[i] - ax;
      if (!std::isfinite(s_init)) s_init = 1.0;
      x_d[n_orig + i] =
          std::clamp(s_init, 1e-4, std::max(1e-4, ub[n_orig + i] - 1e-4));
    }
    for (int j = 0; j < nn; ++j) {
      gl_d[j] = flb[j] ? std::max(x_d[j] - lb[j], 1e-4) : kBig;
      gu_d[j] = fub[j] ? std::max(ub[j] - x_d[j], 1e-4) : kBig;
    }
    // Mehrotra-style dual start (Mehrotra 1992).  Setting zl=zu=1 arbitrarily
    // leaves the initial dual residual rd = c - Ae'y - flb*zl + fub*zu huge
    // (df ~ 1e7 on SCUC / scaled LPs) — the dominant cause of divergence.
    // Instead choose zl - zu = rc := c - Ae'y so rd starts near zero, then
    // shift both toward strict positivity and center the products g*z.
    {
      std::vector<double> mh_rp(m), mh_rc(nn);
      compute_residuals(x_d, y_d, mh_rp.data(), mh_rc.data());  // mh_rc = c - Ae'y
      double zmin = 0.0;
      for (int j = 0; j < nn; ++j) {
        if (j < n_orig && is_fixed[j]) { zl_d[j] = 0.0; zu_d[j] = 0.0; continue; }
        const double rcj = mh_rc[j];
        double zl0 = 0.0, zu0 = 0.0;
        if (flb[j] && fub[j]) {
          zl0 = std::max(rcj, 0.0);
          zu0 = std::max(-rcj, 0.0);
        } else if (flb[j]) {
          zl0 = rcj;
        } else if (fub[j]) {
          zu0 = -rcj;
        }
        zl_d[j] = zl0;
        zu_d[j] = zu0;
        if (flb[j]) zmin = std::min(zmin, zl0);
        if (fub[j]) zmin = std::min(zmin, zu0);
      }
      const double dz = std::max(-1.5 * zmin, 0.0);
      double gz = 0.0, gsum = 0.0;
      for (int j = 0; j < nn; ++j) {
        if (j < n_orig && is_fixed[j]) continue;
        if (flb[j] && gl_d[j] < kBig) { gz += gl_d[j] * (zl_d[j] + dz); gsum += gl_d[j]; }
        if (fub[j] && gu_d[j] < kBig) { gz += gu_d[j] * (zu_d[j] + dz); gsum += gu_d[j]; }
      }
      const double dzc = dz + ((gsum > 1e-30) ? 0.5 * gz / gsum : 1.0);
      for (int j = 0; j < nn; ++j) {
        if (j < n_orig && is_fixed[j]) continue;
        if (flb[j]) zl_d[j] = std::max(zl_d[j] + dzc, 1e-6);
        if (fub[j]) zu_d[j] = std::max(zu_d[j] + dzc, 1e-6);
      }
    }
  }

  // === IPM iteration ===
  std::vector<double> theta(nn), dx(nn), dy(m), dzl(nn), dzu(nn);
  std::vector<double> dx_aff(nn), dy_aff(m), dzl_aff(nn), dzu_aff(nn);
  std::vector<double> rhs(m), tmp_n(nn), Atdy(nn);
  std::vector<double> r_p(m), r_d(nn);
  std::vector<double> inv_gl(nn), inv_gu(nn);  // precomputed reciprocals
  std::vector<double> dvec(nn, 0.0);  // barrier Hessian diagonal d (augmented path)

  // Augmented-KKT operands, built once when use_augmented.  jg = -Ae (m x nn);
  // aug_w = diag(d) (nn x nn, values refreshed per iteration).  When CHOLMOD is
  // available the symmetric quasidefinite system is factored with its
  // simplicial LDLᵀ (fast, exploits symmetry); otherwise factor_kkt_sparse
  // (unsymmetric LU) is the portable fallback.
  Eigen::SparseMatrix<double> aug_negAe;
  Eigen::SparseMatrix<double> aug_w;
  SparseKKTCache aug_cache;
  Eigen::VectorXd aug_rhs, aug_dx, aug_dy;
  CholmodLDLT aug_chol;
  std::vector<int> aug_ko, aug_ki, aug_diag;      // lower-tri CSC of K + diag positions
  std::vector<double> aug_kv, aug_rhsbuf, aug_solbuf;
  bool aug_use_cholmod = false;
  int aug_kdim = 0;
  bool aug_use_accel = false;  // Apple Accelerate unpivoted LDLᵀ (parallel, quasidefinite)
#if MIPSOLVERS_USE_ACCELERATE
  // Factor the quasidefinite augmented KKT [diag(d)+reg, -Ae'; -Ae, -reg] with
  // Accelerate's multithreaded unpivoted LDLᵀ instead of the serial CHOLMOD
  // simplicial LDLᵀ.  Measured ~4x faster on the 39-/118-bus SCUC LP
  // relaxations; the parallel factorization only pays off for large systems, so
  // gate on the KKT dimension (nn+m) — small augmented systems (e.g. the 6-bus)
  // keep CHOLMOD, which has less per-factor overhead.  The quasidefinite matrix
  // makes the unpivoted factorization numerically stable.  MIPSOLVERS_IPM_AUG_ACCEL
  // forces it on ("1"/present) or off ("0").
  const char* aug_accel_env = std::getenv("MIPSOLVERS_IPM_AUG_ACCEL");
  const bool aug_prefer_accel =
      aug_accel_env ? (aug_accel_env[0] != '0') : (nn + m >= 5000);
  std::vector<long> aug_accel_colstarts;
  SparseOpaqueSymbolicFactorization aug_accel_symbolic{};
  SparseOpaqueFactorization_Double aug_accel_numeric{};
  bool aug_accel_symbolic_valid = false;
  bool aug_accel_symbolic_cached = false;
  bool aug_accel_numeric_valid = false;
#endif
  if (use_augmented) {
    std::vector<Eigen::Triplet<double>> tri;
    tri.reserve(static_cast<size_t>(prob.A.nonZeros() + prob.Aeq.nonZeros() + mi));
    for (int j = 0; j < n_orig; ++j) {
      for (int p = A_o[j]; p < A_o[j + 1]; ++p)
        tri.emplace_back(A_i[p], j, -A_v[p]);
      if (me > 0)
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p)
          tri.emplace_back(mi + Aeq_i[p], j, -Aeq_v[p]);
    }
    for (int i = 0; i < mi; ++i) tri.emplace_back(i, n_orig + i, -1.0);
    aug_negAe.resize(m, nn);
    aug_negAe.setFromTriplets(tri.begin(), tri.end());
    aug_negAe.makeCompressed();
    std::vector<Eigen::Triplet<double>> wtri;
    wtri.reserve(static_cast<size_t>(nn));
    for (int j = 0; j < nn; ++j) wtri.emplace_back(j, j, 1.0);
    aug_w.resize(nn, nn);
    aug_w.setFromTriplets(wtri.begin(), wtri.end());
    aug_w.makeCompressed();
    aug_rhs.resize(nn + m);
#if defined(MIPSOLVERS_HAVE_CHOLMOD) || MIPSOLVERS_USE_ACCELERATE
    // Lower-triangle CSC of K = [ diag(d)+reg  -Ae'; -Ae  -reg ] (dim nn+m).
    // Column j<nn: diagonal (row j) then the -Ae entries (rows nn+i, sorted);
    // column nn+i: the single -reg diagonal.  Only diagonals change per iter.
    const int* nAo = aug_negAe.outerIndexPtr();
    const int* nAi = aug_negAe.innerIndexPtr();
    const double* nAv = aug_negAe.valuePtr();
    const int kdim = nn + m;
    aug_ko.assign(static_cast<size_t>(kdim + 1), 0);
    aug_diag.assign(static_cast<size_t>(kdim), 0);
    aug_ki.reserve(static_cast<size_t>(nn) +
                   static_cast<size_t>(aug_negAe.nonZeros()) +
                   static_cast<size_t>(m));
    aug_kv.reserve(aug_ki.capacity());
    int pos = 0;
    for (int j = 0; j < nn; ++j) {
      aug_ko[static_cast<size_t>(j)] = pos;
      aug_diag[static_cast<size_t>(j)] = pos;
      aug_ki.push_back(j);
      aug_kv.push_back(0.0);
      ++pos;
      for (int p = nAo[j]; p < nAo[j + 1]; ++p) {
        aug_ki.push_back(nn + nAi[p]);
        aug_kv.push_back(nAv[p]);
        ++pos;
      }
    }
    for (int i = 0; i < m; ++i) {
      aug_ko[static_cast<size_t>(nn + i)] = pos;
      aug_diag[static_cast<size_t>(nn + i)] = pos;
      aug_ki.push_back(nn + i);
      aug_kv.push_back(0.0);
      ++pos;
    }
    aug_ko[static_cast<size_t>(kdim)] = pos;
    aug_kdim = kdim;
    aug_rhsbuf.resize(static_cast<size_t>(kdim));
    aug_solbuf.resize(static_cast<size_t>(kdim));
#if MIPSOLVERS_USE_ACCELERATE
    if (aug_prefer_accel) {
      // Apple Sparse symbolic factor of the symmetric (lower-triangle stored)
      // quasidefinite K, unpivoted LDLᵀ (multithreaded on Apple Silicon).
      aug_accel_colstarts.assign(aug_ko.begin(), aug_ko.end());
      const int aug_nnz = static_cast<int>(aug_ki.size());
      if (accel_cache_->augmented_valid &&
          accel_cache_->augmented_cached_dim == kdim &&
          accel_cache_->augmented_cached_nnz == aug_nnz) {
        aug_accel_symbolic = accel_cache_->augmented_symbolic;
        aug_accel_symbolic_cached = true;
      } else {
        SparseMatrixStructure s{};
        s.rowCount = kdim;
        s.columnCount = kdim;
        s.columnStarts = aug_accel_colstarts.data();
        s.rowIndices = aug_ki.data();
        s.attributes.transpose = false;
        s.attributes.triangle = SparseLowerTriangle;
        s.attributes.kind = SparseSymmetric;
        s.attributes._reserved = 0;
        s.attributes._allocatedBySparse = false;
        s.blockSize = 1;
        aug_accel_symbolic =
            SparseFactor(SparseFactorizationLDLTUnpivoted, s);
        if (aug_accel_symbolic.status == SparseStatusOK) {
          if (accel_cache_->augmented_valid) {
            SparseCleanup(accel_cache_->augmented_symbolic);
          }
          accel_cache_->augmented_symbolic = aug_accel_symbolic;
          accel_cache_->augmented_col_starts = aug_accel_colstarts;
          accel_cache_->augmented_cached_dim = kdim;
          accel_cache_->augmented_cached_nnz = aug_nnz;
          accel_cache_->augmented_valid = true;
          aug_accel_symbolic_cached = true;
        }
      }
      aug_accel_symbolic_valid = true;
      aug_use_accel = (aug_accel_symbolic.status == SparseStatusOK);
    }
#endif
#ifdef MIPSOLVERS_HAVE_CHOLMOD
    if (!aug_use_accel) {
      aug_chol.set_simplicial(true);  // quasidefinite (indefinite, unpivoted) LDLᵀ
      aug_use_cholmod = aug_chol.analyze(kdim, aug_ko.data(), aug_ki.data(),
                                         aug_kv.data(), static_cast<int64_t>(pos));
    }
#endif
#endif
  }

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

  // Primal-dual regularization (IP-PMM style).  reg is the diagonal added to
  // the normal equations (dual reg delta) and, via 1/(d+reg), also bounds
  // theta (primal reg rho).  It starts at reg_floor and is refreshed each
  // iteration to track the barrier scale mu (see the loop), so it stabilizes
  // the near-singular degenerate-LP normal equations early then vanishes as
  // mu -> 0 to keep the solution accurate.
  const double reg_floor = (n_fixed_vars > n_orig / 4) ? 1e-6 : 1e-10;
  double reg = reg_floor;
  const int max_iter = std::min(opt_.max_iter, 200);
  bool converged = false;

  double t_resid = 0, t_setup = 0, t_pred = 0, t_corr = 0, t_update = 0;
  double t_fill = 0, t_factor = 0;
  double t_init_overhead = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  auto tnow = []() { return std::chrono::steady_clock::now(); };

  // Working copy of banded Cholesky factor
  std::vector<double> band_work;
  if (use_banded) band_work.resize(band_storage.size());

  // Lambda: fill banded N from θ, factorize
  auto fill_and_factor_banded = [&]() -> bool {
    auto t_ff = tnow();
    double* bs = band_storage.data();
    const size_t bs_sz = band_storage.size();
    std::memset(bs, 0, sizeof(double) * bs_sz);

    // Original variable columns
    for (int j = 0; j < n_orig; ++j) {
      const double th = theta_d[j];
      for (int si = band_scatter_col_start[j]; si < band_scatter_col_start[j + 1]; ++si)
        bs[band_scatter[si].offset] += th * band_scatter[si].a_prod;
    }
    // Slack columns: identity → theta of slack goes to diagonal
    for (int i = 0; i < mi; ++i)
      bs[i] += theta_d[n_orig + i];
    // Regularization
    for (int i = 0; i < m; ++i) bs[i] += reg;

    auto t_ff2 = tnow();
    t_fill += std::chrono::duration<double, std::milli>(t_ff2 - t_ff).count();

    // Copy to working buffer and factorize
    std::memcpy(band_work.data(), bs, sizeof(double) * bs_sz);
    bool ok = banded_chol_factor(band_work.data(), m, bw);
    t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
    return ok;
  };

  auto fill_and_factor_dense = [&]() -> bool {
    auto t_ff = tnow();
    // Ae_sqrt[:,k] = sqrt(theta[k]) * Ae_dense[:,k]
    for (int k = 0; k < nn; ++k)
      Ae_sqrt.col(k) = Ae_dense.col(k) * std::sqrt(theta_d[k]);
    auto t_ff2 = tnow();
    t_fill += std::chrono::duration<double, std::milli>(t_ff2 - t_ff).count();
    // N = Ae_sqrt * Ae_sqrt'  (symmetric rank-nn update = Ae * diag(theta) * Ae')
#if MIPSOLVERS_USE_ACCELERATE
    // Use Accelerate cblas_dsyrk for full BLAS performance (>100 GFLOPS on M4).
    // Fills only the lower triangle; Eigen::LDLT<MatrixXd> reads only lower by default.
    N_dense.setZero();
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    cblas_dsyrk(CblasColMajor, CblasLower, CblasNoTrans,
                m, nn, 1.0, Ae_sqrt.data(), m, 0.0, N_dense.data(), m);
#pragma clang diagnostic pop
#else
    N_dense.noalias() = Ae_sqrt * Ae_sqrt.transpose();
#endif
    N_dense.diagonal().array() += reg;
    ldlt_dense.compute(N_dense);
    t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
    return ldlt_dense.info() == Eigen::Success;
  };

  auto fill_and_factor_sparse = [&]() -> bool {
    auto t_ff = tnow();
    double* Nv = N_sparse.valuePtr();
    std::memset(Nv, 0, sizeof(double) * sparse_n_nnz);
    for (int j = 0; j < n_orig; ++j) {
      const double th = theta_d[j];
      for (int si = sparse_scatter_col_start[j]; si < sparse_scatter_col_start[j + 1]; ++si)
        Nv[sparse_scatter[si].ni] += th * sparse_scatter[si].a_prod;
    }
    for (int i = 0; i < mi; ++i)
      Nv[sparse_diag_offsets[i]] += theta_d[n_orig + i];
    for (int i = 0; i < m; ++i) Nv[sparse_diag_offsets[i]] += reg;
    auto t_ff2 = tnow();
    t_fill += std::chrono::duration<double, std::milli>(t_ff2 - t_ff).count();
#if MIPSOLVERS_HAVE_CHOLMOD
    if (cholmod_ok) {
      const bool ok = cholmod_ldlt.factorize(N_sparse.valuePtr());
      t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
      return ok;
    }
#endif
#if MIPSOLVERS_USE_ACCELERATE
    SparseMatrix_Double apple_N = make_apple_matrix();
    if (accel_numeric_valid) {
      SparseRefactor(apple_N, &accel_numeric);
    } else {
      accel_numeric = SparseFactor(accel_symbolic, apple_N);
      accel_numeric_valid = true;
    }
    t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
    return accel_numeric.status == SparseStatusOK;
#else
    ldlt.factorize(N_sparse);
    t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
    return ldlt.info() == Eigen::Success;
#endif
  };

  // Lambda: solve normal equations, with conditional iterative refinement
  // (mirrors the KKT path): after each solve, the residual r = rhs - N·dy
  // is checked against the pristine matrix; up to 2 correction solves with
  // the existing factorization run when ||r||∞ > 1e-12·max(1, ||rhs||∞).
  // Cost: one matvec per linear solve — negligible next to factorization.
  std::vector<double> ir_ndy(m), ir_r(m), ir_corr(m);
  auto solve_normal = [&](double* rhs_buf, double* dy_buf) {
    auto raw_solve = [&](const double* rhs, double* dy) {
      if (use_banded) {
        std::memcpy(dy, rhs, sizeof(double) * m);
        banded_chol_solve(band_work.data(), m, bw, dy);
      } else if (use_dense) {
        Eigen::Map<const Eigen::VectorXd> rhs_map(rhs, m);
        Eigen::Map<Eigen::VectorXd> dy_map(dy, m);
        dy_map = ldlt_dense.solve(rhs_map);
      } else {
#if MIPSOLVERS_HAVE_CHOLMOD
        if (cholmod_ok) {
          if (!cholmod_ldlt.solve(rhs, dy)) {
            // Poison with NaN so the finiteness guard aborts the loop
            // (same pattern as the UMFPACK failure handling).
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
        dy_map = ldlt.solve(rhs_map);
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
        banded_sym_matvec(band_storage.data(), m, bw, dy_buf, ir_ndy.data());
      } else {
        Eigen::Map<const Eigen::VectorXd> dy_map(dy_buf, m);
        Eigen::Map<Eigen::VectorXd> ndy_map(ir_ndy.data(), m);
        if (use_dense) {
          ndy_map = N_dense.selfadjointView<Eigen::Lower>() * dy_map;
        } else {
          ndy_map = N_sparse.selfadjointView<Eigen::Lower>() * dy_map;
        }
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

  // Compute the Newton step (dx, dy) from the per-variable rhs xi and the
  // current primal residual r_p, dispatching to the normal-equations path or
  // the sparse augmented-KKT path.  Both solve the same system:
  //   normal:    N dy = r_p - Ae*(theta*xi);  dx = theta*(Ae'dy) + theta*xi
  //   augmented: [diag(d)+reg, -Ae'; -Ae, -reg] [dx;dy] = [xi; -r_p]
  std::vector<double> xi_store(nn);
  double* xi_d = xi_store.data();
  // Scratch for gated KKT iterative refinement of the Newton step.
  std::vector<double> kkt_neg_rp(m), kkt_r2(m), kkt_ddy(m), kkt_adx(m);
  std::vector<double> kkt_r1(nn), kkt_ddx(nn);
#if MIPSOLVERS_USE_ACCELERATE
  // Apple Sparse matrix view over the augmented KKT CSC (values are refreshed
  // in place each iteration; the structure is fixed).
  auto make_aug_apple_matrix = [&]() {
    SparseMatrixStructure s{};
    s.rowCount = aug_kdim;
    s.columnCount = aug_kdim;
    s.columnStarts = aug_accel_colstarts.data();
    s.rowIndices = aug_ki.data();
    s.attributes.transpose = false;
    s.attributes.triangle = SparseLowerTriangle;
    s.attributes.kind = SparseSymmetric;
    s.attributes._reserved = 0;
    s.attributes._allocatedBySparse = false;
    s.blockSize = 1;
    SparseMatrix_Double mat{};
    mat.structure = s;
    mat.data = aug_kv.data();
    return mat;
  };
#endif
  auto raw_kkt_solve = [&](const double* xi, const double* prim_rhs,
                           double* dx_out, double* dy_out) {
    if (use_augmented) {
#if MIPSOLVERS_USE_ACCELERATE
      if (aug_use_accel) {
        for (int j = 0; j < nn; ++j) aug_solbuf[static_cast<size_t>(j)] = xi[j];
        for (int i = 0; i < m; ++i)
          aug_solbuf[static_cast<size_t>(nn + i)] = prim_rhs[i];
        DenseVector_Double bx{};
        bx.count = aug_kdim;
        bx.data = aug_solbuf.data();
        SparseSolve(aug_accel_numeric, bx);  // in-place: b <- K^{-1} b
        for (int j = 0; j < nn; ++j) dx_out[j] = aug_solbuf[static_cast<size_t>(j)];
        for (int i = 0; i < m; ++i)
          dy_out[i] = aug_solbuf[static_cast<size_t>(nn + i)];
        return;
      }
#endif
      if (aug_use_cholmod) {
        for (int j = 0; j < nn; ++j) aug_rhsbuf[static_cast<size_t>(j)] = xi[j];
        for (int i = 0; i < m; ++i) aug_rhsbuf[static_cast<size_t>(nn + i)] = prim_rhs[i];
        if (!aug_chol.solve(aug_rhsbuf.data(), aug_solbuf.data())) {
          for (int j = 0; j < nn; ++j) dx_out[j] = std::numeric_limits<double>::quiet_NaN();
          for (int i = 0; i < m; ++i) dy_out[i] = std::numeric_limits<double>::quiet_NaN();
          return;
        }
        for (int j = 0; j < nn; ++j) dx_out[j] = aug_solbuf[static_cast<size_t>(j)];
        for (int i = 0; i < m; ++i) dy_out[i] = aug_solbuf[static_cast<size_t>(nn + i)];
        return;
      }
      for (int j = 0; j < nn; ++j) aug_rhs[j] = xi[j];
      for (int i = 0; i < m; ++i) aug_rhs[nn + i] = prim_rhs[i];
      if (!solve_kkt_sparse(aug_cache, aug_rhs, aug_dx, aug_dy) ||
          aug_dx.size() != nn || aug_dy.size() != m) {
        for (int j = 0; j < nn; ++j) dx_out[j] = std::numeric_limits<double>::quiet_NaN();
        for (int i = 0; i < m; ++i) dy_out[i] = std::numeric_limits<double>::quiet_NaN();
        return;
      }
      for (int j = 0; j < nn; ++j) dx_out[j] = aug_dx[j];
      for (int i = 0; i < m; ++i) dy_out[i] = aug_dy[i];
      return;
    }
    for (int j = 0; j < nn; ++j) tmp_d[j] = theta_d[j] * xi[j];
    for (int i = 0; i < m; ++i) rhs_d[i] = -prim_rhs[i];
    ae_mul_sub(tmp_d, rhs_d);
    solve_normal(rhs_d, dy_out);
    aet_mul(dy_out, Atdy_d);
    simd_fma(theta_d, Atdy_d, tmp_d, dx_out, nn);  // dx = theta*Atdy + theta*xi
  };

  // Newton step with gated KKT iterative refinement.  The regularized
  // factorization biases the direction: the primal residual cannot drop below
  // ~reg*||dy||, which stalls then destabilizes ill-conditioned LPs (e.g. agg,
  // lotfi).  Refining against the *unregularized* KKT residual removes that
  // bias.  On well-conditioned problems the residual is already below tol, so
  // no correction solves run and the fast path is unchanged.
  auto solve_step = [&](const double* xi, double* dx_out, double* dy_out) {
    for (int i = 0; i < m; ++i) kkt_neg_rp[i] = -r_p_d[i];
    raw_kkt_solve(xi, kkt_neg_rp.data(), dx_out, dy_out);
    double rhs_scale = 1.0;
    for (int i = 0; i < m; ++i) rhs_scale = std::max(rhs_scale, std::abs(r_p_d[i]));
    for (int j = 0; j < nn; ++j) rhs_scale = std::max(rhs_scale, std::abs(xi[j]));
    // Refinement gate.  A well-conditioned solve leaves a tiny relative KKT
    // residual (measured: afiro 1.5e-4, sc205 6e-7) and skips refinement
    // entirely, preserving the fast path.  An ill-conditioned one (agg ~0.23,
    // i.e. the reg bias dominates) trips the 1% trigger and is then refined
    // down to the tight target so the bias is removed and it converges.
    const double refine_trigger = 1e-2 * rhs_scale;
    const double refine_target = 1e-9 * rhs_scale;
    auto kkt_residual = [&]() -> double {
      aet_mul(dy_out, Atdy_d);         // Ae'dy
      ae_mul(dx_out, kkt_adx.data());  // Ae dx
      double res = 0.0;
      for (int j = 0; j < nn; ++j) {
        const double v = xi[j] - dvec[j] * dx_out[j] + Atdy_d[j];
        kkt_r1[j] = v;
        const double a = std::abs(v);
        if (a > res) res = a;
      }
      for (int i = 0; i < m; ++i) {
        const double v = -r_p_d[i] + kkt_adx[i];
        kkt_r2[i] = v;
        const double a = std::abs(v);
        if (a > res) res = a;
      }
      return res;
    };
    double res = kkt_residual();
    const bool need_refine = std::isfinite(res) && res > refine_trigger;
    for (int ref = 0; ref < 5 && need_refine && std::isfinite(res) && res > refine_target; ++ref) {
      raw_kkt_solve(kkt_r1.data(), kkt_r2.data(), kkt_ddx.data(), kkt_ddy.data());
      for (int j = 0; j < nn; ++j) dx_out[j] += kkt_ddx[j];
      for (int i = 0; i < m; ++i) dy_out[i] += kkt_ddy[i];
      const double res_new = kkt_residual();
      if (!std::isfinite(res_new) || res_new >= res) {
        // Correction stopped helping (or the reg=0 system is singular): revert
        // it and keep the last good direction.
        for (int j = 0; j < nn; ++j) dx_out[j] -= kkt_ddx[j];
        for (int i = 0; i < m; ++i) dy_out[i] -= kkt_ddy[i];
        break;
      }
      res = res_new;
    }
  };

  // Gondzio multiple centrality corrector scratch + a shared fraction-to-
  // boundary step-length helper (used to score each corrector's step gain).
  std::vector<double> gc_xi(nn), gc_ddx(nn), gc_ddy(m), gc_ddzl(nn),
      gc_ddzu(nn), gc_rgl(nn), gc_rgu(nn), gc_zero_rp(m, 0.0);
  auto compute_step_lengths = [&](const double* dxv, const double* dzlv,
                                  const double* dzuv, double& ap_out,
                                  double& ad_out) {
    double ap_l = 1.0, ad_l = 1.0;
    for (int j = 0; j < nn; ++j) {
      if (dxv[j] < -kMinVal && flb_d[j]) {
        const double a = -kTau * gl_d[j] / dxv[j];
        if (a < ap_l) ap_l = a;
      }
      if (dxv[j] > kMinVal && fub_d[j]) {
        const double a = kTau * gu_d[j] / dxv[j];
        if (a < ap_l) ap_l = a;
      }
      if (dzlv[j] < -kMinVal) {
        const double a = -kTau * zl_d[j] / dzlv[j];
        if (a < ad_l) ad_l = a;
      }
      if (dzuv[j] < -kMinVal) {
        const double a = -kTau * zu_d[j] / dzuv[j];
        if (a < ad_l) ad_l = a;
      }
    }
    ap_out = std::max(ap_l, kMinVal);
    ad_out = std::max(ad_l, kMinVal);
  };

  for (int iter = 0; iter < max_iter; ++iter) {
    if (opt_.time_limit_sec > 0.0 && std::isfinite(opt_.time_limit_sec)) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
              .count();
      if (elapsed >= opt_.time_limit_sec) {
        out.stats.success = false;
        out.stats.iterations = iter;
        out.stats.status = "Time limit";
        break;
      }
    }
    auto t_s = tnow();

    // Residuals: r_p = b - Ae*x, r_d = c - Ae'*y (merged single pass)
    compute_residuals(x_d, y_d, r_p_d, r_d_d);

    // Merge: r_d adjustment + complementarity + norms — single pass over nn
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

    if ((opt_.verbose || ipm_verbose_env) && (iter < 3 || iter % 5 == 0))
      fprintf(stderr, "IPM-LP %3d: pf=%.2e df=%.2e mu=%.2e\n", iter, pfeas, dfeas, mu);

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
    t_resid += std::chrono::duration<double, std::milli>(tnow() - t_s).count();

    // Build Θ, precompute inv_gl, inv_gu, and factorize
    auto t_su = tnow();
    // Refresh the primal-dual regularization to track the barrier scale mu.
    // reg damps the (otherwise unbounded on degenerate LPs) Newton step early
    // then vanishes as mu -> 0.  Applied as primal reg in theta = 1/(d+reg) —
    // bounding theta at 1/reg instead of the old hard 1e14 cap that amplified
    // dx = theta*(Ae'dy) into 1e25+ garbage — and as dual reg on the
    // normal-equations diagonal (added inside fill_and_factor).
    reg = std::clamp(1e-6 * mu, reg_floor, 1e-2);
    MIPSOLVERS_OMP_PARALLEL_IF(nn > MIPSOLVERS_OMP_THRESHOLD)
    for (int j = 0; j < nn; ++j) {
      if (j < n_orig && is_fixed[j]) {
        inv_gl_d[j] = 0.0;
        inv_gu_d[j] = 0.0;
        theta_d[j] = 0.0;  // remove fixed var from normal equations
        dvec[j] = 1e12;    // pin dx~0 for fixed vars in the augmented system
        continue;
      }
      double igl = flb_d[j] / gl_d[j];  // 0 if no lb (flb=0, gl=kBig)
      double igu = fub_d[j] / gu_d[j];
      inv_gl_d[j] = igl;
      inv_gu_d[j] = igu;
      double d = zl_d[j] * igl + zu_d[j] * igu;
      dvec[j] = d;
      theta_d[j] = 1.0 / (d + reg);
    }

    bool factor_ok;
    if (use_augmented) {
      if (aug_use_cholmod || aug_use_accel) {
        // Refresh the KKT diagonal (d[j]+rr on the primal block, -rr on the
        // dual block) and factor the quasidefinite system (CHOLMOD simplicial
        // LDLᵀ, or Accelerate unpivoted LDLᵀ when aug_use_accel).
        auto set_aug_diag = [&](double rr) {
          for (int j = 0; j < nn; ++j)
            aug_kv[static_cast<size_t>(aug_diag[static_cast<size_t>(j)])] =
                dvec[j] + rr;
          for (int i = 0; i < m; ++i)
            aug_kv[static_cast<size_t>(aug_diag[static_cast<size_t>(nn + i)])] =
                -rr;
        };
        auto aug_numeric_factor = [&]() -> bool {
#if MIPSOLVERS_USE_ACCELERATE
          if (aug_use_accel) {
            SparseMatrix_Double K = make_aug_apple_matrix();
            if (aug_accel_numeric_valid) {
              SparseRefactor(K, &aug_accel_numeric);
            } else {
              aug_accel_numeric = SparseFactor(aug_accel_symbolic, K);
              aug_accel_numeric_valid = true;
            }
            return aug_accel_numeric.status == SparseStatusOK;
          }
#endif
          return aug_chol.factorize(aug_kv.data());
        };
        set_aug_diag(reg);
        factor_ok = aug_numeric_factor();
        for (int retry = 0; retry < 4 && !factor_ok; ++retry) {
          const double dyn = reg * std::pow(100.0, retry + 1);
          set_aug_diag(dyn);
          factor_ok = aug_numeric_factor();
        }
      } else {
        // Portable fallback: unsymmetric LU via factor_kkt_sparse.
        double* wv = aug_w.valuePtr();
        for (int j = 0; j < nn; ++j) wv[j] = dvec[j];
        factor_ok = factor_kkt_sparse(aug_cache, aug_w, aug_negAe, reg);
        for (int retry = 0; retry < 4 && !factor_ok; ++retry) {
          const double dyn_reg = reg * std::pow(100.0, retry + 1);
          factor_ok = factor_kkt_sparse(aug_cache, aug_w, aug_negAe, dyn_reg);
        }
      }
    } else {
      factor_ok = use_banded ? fill_and_factor_banded()
                             : (use_dense ? fill_and_factor_dense() : fill_and_factor_sparse());
    }
    // Dynamic regularization retry: if Cholesky fails (ill-conditioned normal
    // equations from near-parallel constraints), increase diagonal perturbation.
    if (!factor_ok && !use_augmented) {
      double dyn_reg = std::max(reg * 1e4, 1e-8);
      for (int retry = 0; retry < 4 && !factor_ok; ++retry) {
        if (use_banded) {
          // Restore the pristine matrix before perturbing: the failed
          // attempt left band_work partially factorized, so adding dyn_reg
          // without this refill would factorize garbage.  band_storage
          // still holds the untouched fill from fill_and_factor_banded().
          std::memcpy(band_work.data(), band_storage.data(),
                      sizeof(double) * band_storage.size());
          double* bw_data = band_work.data();
          for (int i = 0; i < m; ++i) bw_data[i] += dyn_reg;
          factor_ok = banded_chol_factor(bw_data, m, bw);
        } else if (use_dense) {
          N_dense.diagonal().array() += dyn_reg;
          ldlt_dense.compute(N_dense);
          factor_ok = (ldlt_dense.info() == Eigen::Success);
        } else {
          // Re-fill sparse normal equations with extra diagonal reg
          double* Nv = N_sparse.valuePtr();
          for (int i = 0; i < m; ++i) Nv[sparse_diag_offsets[i]] += dyn_reg;
#if MIPSOLVERS_HAVE_CHOLMOD
          if (cholmod_ok) {
            factor_ok = cholmod_ldlt.factorize(N_sparse.valuePtr());
          } else
#endif
#if MIPSOLVERS_USE_ACCELERATE
          {
          SparseMatrix_Double apple_N = make_apple_matrix();
          if (accel_numeric_valid) {
            SparseRefactor(apple_N, &accel_numeric);
          } else {
            accel_numeric = SparseFactor(accel_symbolic, apple_N);
            accel_numeric_valid = true;
          }
          factor_ok = (accel_numeric.status == SparseStatusOK);
          }
#else
          {
          ldlt.factorize(N_sparse);
          factor_ok = (ldlt.info() == Eigen::Success);
          }
#endif
        }
        if (!factor_ok) dyn_reg *= 100.0;
      }
    }
    if (!factor_ok) {
      out.stats.status = "Cholesky failed";
      out.stats.iterations = iter;
      break;
    }
    t_setup += std::chrono::duration<double, std::milli>(tnow() - t_su).count();

    // ---- Predictor (affine, σ=0) ----
    auto t_p = tnow();

    // ξ_aff = -r_d - z_l + z_u; solve for (Δx_aff, Δy_aff).
    for (int j = 0; j < nn; ++j)
      xi_d[j] = -r_d_d[j] - flb_d[j] * zl_d[j] + fub_d[j] * zu_d[j];
    solve_step(xi_d, dx_aff_d, dy_aff_d);

    // Δz_aff from complementarity
    for (int j = 0; j < nn; ++j) {
      dzl_aff_d[j] = flb_d[j] * (-zl_d[j] - zl_d[j] * inv_gl_d[j] * dx_aff_d[j]);
      dzu_aff_d[j] = fub_d[j] * (-zu_d[j] + zu_d[j] * inv_gu_d[j] * dx_aff_d[j]);
    }

    // Affine step sizes
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

    // Centering parameter
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

    t_pred += std::chrono::duration<double, std::milli>(tnow() - t_p).count();

    // Conservative corrector skip: only when affine step is near-optimal
    // Skip when: excellent affine step (both > 0.9) AND sigma is tiny (< 0.01)
    double ap, ad;
    auto t_c = tnow();
    const bool skip_corrector = (ap_aff > 0.9 && ad_aff > 0.9 && sigma < 0.02);
    
    if (skip_corrector) {
      // Use affine direction directly (with very small centering)
      ap = ap_aff;
      ad = ad_aff;
      std::memcpy(dx_d, dx_aff_d, sizeof(double) * nn);
      std::memcpy(dy_d, dy_aff_d, sizeof(double) * m);
      std::memcpy(dzl.data(), dzl_aff_d, sizeof(double) * nn);
      std::memcpy(dzu.data(), dzu_aff_d, sizeof(double) * nn);
    } else {
      // ---- Corrector (centering + Mehrotra second-order) ----
      for (int j = 0; j < nn; ++j) {
        double xi = -r_d_d[j];
        xi += flb_d[j] * (sigma_mu * inv_gl_d[j] - zl_d[j] - dzl_aff_d[j] * dx_aff_d[j] * inv_gl_d[j]);
        xi -= fub_d[j] * (sigma_mu * inv_gu_d[j] - zu_d[j] + dzu_aff_d[j] * dx_aff_d[j] * inv_gu_d[j]);
        xi_d[j] = xi;
      }
      solve_step(xi_d, dx_d, dy_d);

      for (int j = 0; j < nn; ++j) {
        dzl[j] = flb_d[j] * ((sigma_mu - dzl_aff_d[j] * dx_aff_d[j]) * inv_gl_d[j] - zl_d[j] - zl_d[j] * inv_gl_d[j] * dx_d[j]);
        dzu[j] = fub_d[j] * ((sigma_mu + dzu_aff_d[j] * dx_aff_d[j]) * inv_gu_d[j] - zu_d[j] + zu_d[j] * inv_gu_d[j] * dx_d[j]);
      }

      // Corrector step sizes
      ap = 1.0;
      ad = 1.0;
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

    // ---- Gondzio multiple centrality correctors ----
    // Each corrector reuses the current factorization (one extra back-solve via
    // raw_kkt_solve with a ZERO primal rhs — it only re-centers complementarity)
    // and is kept only if it enlarges the fraction-to-boundary step, pulling
    // trial products back into the symmetric neighborhood [beta_min*mu,
    // beta_max*mu].  Gated to the THROTTLED regime (min step < 0.9): when the
    // Mehrotra step is already good, correcting only perturbs it and wastes
    // solves (measured: +7 iters on 39-bus, +6.5s on 118-bus).  Throttling is
    // the degenerate slow-start that stalls the affine step (6-bus: 64->30).
    if (!skip_corrector && opt_.max_correctors > 0 && mu > 0.0 &&
        std::min(ap, ad) < 0.9) {
      constexpr double kBetaMin = 0.1, kBetaMax = 10.0;
      constexpr double kDeltaAlpha = 0.1;   // step-enlargement probe
      constexpr double kGammaAccept = 0.1;  // min total step gain to keep
      const double lo = kBetaMin * mu, hi = kBetaMax * mu;
      for (int kc = 0; kc < opt_.max_correctors; ++kc) {
        const double ap_t = std::min(ap + kDeltaAlpha, 1.0);
        const double ad_t = std::min(ad + kDeltaAlpha, 1.0);
        for (int j = 0; j < nn; ++j) {
          double rgl = 0.0, rgu = 0.0;
          if (flb_d[j]) {
            const double v =
                (gl_d[j] + ap_t * dx_d[j]) * (zl_d[j] + ad_t * dzl[j]);
            rgl = ((v < lo) ? lo : (v > hi ? hi : v)) - v;
          }
          if (fub_d[j]) {
            const double v =
                (gu_d[j] - ap_t * dx_d[j]) * (zu_d[j] + ad_t * dzu[j]);
            rgu = ((v < lo) ? lo : (v > hi ? hi : v)) - v;
          }
          gc_rgl[j] = rgl;
          gc_rgu[j] = rgu;
          gc_xi[j] =
              flb_d[j] * (rgl * inv_gl_d[j]) - fub_d[j] * (rgu * inv_gu_d[j]);
        }
        raw_kkt_solve(gc_xi.data(), gc_zero_rp.data(), gc_ddx.data(),
                      gc_ddy.data());
        bool corr_finite = true;
        for (int j = 0; j < nn && corr_finite; ++j)
          corr_finite = std::isfinite(gc_ddx[j]);
        if (!corr_finite) break;
        for (int j = 0; j < nn; ++j) {
          gc_ddzl[j] = flb_d[j] * (gc_rgl[j] * inv_gl_d[j] -
                                   zl_d[j] * inv_gl_d[j] * gc_ddx[j]);
          gc_ddzu[j] = fub_d[j] * (gc_rgu[j] * inv_gu_d[j] +
                                   zu_d[j] * inv_gu_d[j] * gc_ddx[j]);
        }
        for (int j = 0; j < nn; ++j) {
          dx_d[j] += gc_ddx[j];
          dzl[j] += gc_ddzl[j];
          dzu[j] += gc_ddzu[j];
        }
        for (int i = 0; i < m; ++i) dy_d[i] += gc_ddy[i];
        double ap_new = 0.0, ad_new = 0.0;
        compute_step_lengths(dx_d, dzl.data(), dzu.data(), ap_new, ad_new);
        if (ap_new + ad_new > ap + ad + kGammaAccept * kDeltaAlpha) {
          ap = ap_new;
          ad = ad_new;
        } else {
          for (int j = 0; j < nn; ++j) {
            dx_d[j] -= gc_ddx[j];
            dzl[j] -= gc_ddzl[j];
            dzu[j] -= gc_ddzu[j];
          }
          for (int i = 0; i < m; ++i) dy_d[i] -= gc_ddy[i];
          break;
        }
      }
    }

    t_corr += std::chrono::duration<double, std::milli>(tnow() - t_c).count();

    // Finiteness guard: a failed solve or NaN in the search direction must
    // abort the loop instead of polluting the iterate and returning a NaN
    // "solution" after MaxIter.  dzl/dzu derive from dx, so checking the
    // two solve outputs dx/dy covers every NaN source upstream.
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

    // ---- Update (SIMD-accelerated) ----
    auto t_u = tnow();
    simd_axpy(ap, dx_d, x_d, nn);                   // x += ap * dx
    simd_axpy(ad, dy_d, y_d, m);                    // y += ad * dy
    simd_axpy_max(ad, dzl.data(), zl_d, kMinVal, nn);  // zl = max(zl + ad*dzl, kMin)
    simd_axpy_max(ad, dzu.data(), zu_d, kMinVal, nn);  // zu = max(zu + ad*dzu, kMin)
    simd_sub_max(x_d, lb_d, gl_d, kMinVal, nn);    // gl = max(x - lb, kMin)
    simd_rsub_max(x_d, ub_d, gu_d, kMinVal, nn);   // gu = max(ub - x, kMin)
    // Re-lock fixed variables (prevent drift from numerical arithmetic)
    for (int j = 0; j < n_orig; ++j) {
      if (is_fixed[j]) {
        x_d[j] = lb[j];
        gl_d[j] = kBig;
        gu_d[j] = kBig;
      }
    }
    t_update += std::chrono::duration<double, std::milli>(tnow() - t_u).count();
  }

  if (opt_.verbose || ipm_verbose_env) {
    auto t_end = std::chrono::steady_clock::now();
    printf("  IPM timing: total=%.3f ms (m=%d, nn=%d, bw=%d, %s)\n",
           std::chrono::duration<double, std::milli>(t_end - t0).count(), m, nn, bandwidth,
           use_banded ? "BANDED" : (use_dense ? "DENSE" : (use_augmented ? "AUGMENTED" : "SPARSE")));
    printf("    init=%.3f resid=%.3f setup=%.3f pred=%.3f corr=%.3f update=%.3f\n",
           t_init_overhead, t_resid, t_setup, t_pred, t_corr, t_update);
    printf("    setup_sub: fill=%.3f factor=%.3f\n", t_fill, t_factor);
  }

#if MIPSOLVERS_USE_ACCELERATE
  // Release the Accelerate augmented-KKT factorizations (numeric + symbolic).
  if (aug_accel_numeric_valid) SparseCleanup(aug_accel_numeric);
  if (aug_accel_symbolic_valid && !aug_accel_symbolic_cached) {
    SparseCleanup(aug_accel_symbolic);
  }
#endif

  // === Extract solution ===
#if MIPSOLVERS_USE_ACCELERATE
  if (!use_banded && !use_dense && accel_numeric_valid) {
    SparseCleanup(accel_numeric);
  }
#endif
  if (!converged && out.stats.status.empty()) {
    out.stats.status = "MaxIter";
    out.stats.success = false;
  }
  out.x.resize(n_orig);
  for (int j = 0; j < n_orig; ++j)
    out.x(j) = scal.active ? x_d[j] * scal.dc[static_cast<size_t>(j)] : x_d[j];
  out.stats.objective = prob.c.dot(out.x);
  // Original-space feasibility audit.  A scaled (Ruiz) solve can converge in
  // scaled coordinates yet map back to an infeasible original-space point
  // (observed on afiro).  Reject such false-optima so solve_lp's unscaled retry
  // can recover, and so "Optimal" is never reported for a violated solution.
  if (out.stats.success) {
    double bnd_viol = 0.0;
    for (int j = 0; j < n_orig; ++j) {
      const double lbj = prob.vars[static_cast<size_t>(j)].lb;
      const double ubj = prob.vars[static_cast<size_t>(j)].ub;
      if (lbj > -1e19) bnd_viol = std::max(bnd_viol, lbj - out.x(j));
      if (ubj < 1e19) bnd_viol = std::max(bnd_viol, out.x(j) - ubj);
    }
    double row_viol = 0.0;
    if (prob.A.rows() > 0) {
      const Eigen::VectorXd ax = prob.A * out.x;
      for (int i = 0; i < ax.size(); ++i)
        row_viol = std::max(row_viol, ax(i) - prob.b(i));
    }
    if (prob.Aeq.rows() > 0) {
      const Eigen::VectorXd ae = prob.Aeq * out.x;
      for (int i = 0; i < ae.size(); ++i)
        row_viol = std::max(row_viol, std::abs(ae(i) - prob.beq(i)));
    }
    double scale = 1.0;
    if (prob.b.size()) scale = std::max(scale, prob.b.cwiseAbs().maxCoeff());
    if (prob.beq.size()) scale = std::max(scale, prob.beq.cwiseAbs().maxCoeff());
    if (out.x.size()) scale = std::max(scale, out.x.cwiseAbs().maxCoeff());
    const double audit_tol = 1e-6 * scale;
    if (std::max(bnd_viol, row_viol) > audit_tol) {
      out.stats.success = false;
      out.stats.status = "Residual audit rejected";
    }
  }
  if (m > 0) {
    out.constraint_duals.resize(m);
    for (int i = 0; i < m; ++i) {
      // G-type rows were negated to L-type at input: flip the dual sign back
      // (KKT multiplier of -A x <= -lhs is the negative of A x >= lhs).
      const double row_sign =
          (any_flip && i < mi && flip_row[static_cast<size_t>(i)]) ? -1.0 : 1.0;
      out.constraint_duals(i) = row_sign * sense_sign * y_d[i] *
                                (scal.active ? scal.dr[static_cast<size_t>(i)] : 1.0);
    }
  }
  // Export bound multipliers for original variables (used by IPM→simplex crossover).
  out.box_dual_lb.resize(n_orig);
  out.box_dual_ub.resize(n_orig);
  for (int j = 0; j < n_orig; ++j) {
    out.box_dual_lb(j) = scal.active ? zl_d[j] / scal.dc[static_cast<size_t>(j)]
                                     : zl_d[j];
    out.box_dual_ub(j) = scal.active ? zu_d[j] / scal.dc[static_cast<size_t>(j)]
                                     : zu_d[j];
  }
  out.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  out.stats.residual_inf = std::max(out.stats.primal_feas, out.stats.dual_feas);
  return out;
}

SolveResult NativeIPMLPAdapter::solve_node_lp(const LPModel& base_lp,
                                               const Eigen::VectorXd& node_lb,
                                               const Eigen::VectorXd& node_ub) const {
  // Use cached path if available
  if (cached_state_) {
    static const Eigen::VectorXd empty;
    return solve_cached_node_lp(node_lb, node_ub, empty);
  }
  LPModel node_lp = base_lp;
  const int n = static_cast<int>(node_lp.vars.size());
  for (int i = 0; i < n; ++i) {
    node_lp.vars[i].lb = node_lb[i];
    node_lp.vars[i].ub = node_ub[i];
  }
  return solve_lp(node_lp);
}

SolveResult NativeIPMLPAdapter::solve_node_lp(const LPModel& base_lp,
                                               const Eigen::VectorXd& node_lb,
                                               const Eigen::VectorXd& node_ub,
                                               const Eigen::VectorXd& x0) const {
  // Use cached path if available
  if (cached_state_) {
    return solve_cached_node_lp(node_lb, node_ub, x0);
  }
  LPModel node_lp = base_lp;
  const int n = static_cast<int>(node_lp.vars.size());
  for (int i = 0; i < n; ++i) {
    node_lp.vars[i].lb = node_lb[i];
    node_lp.vars[i].ub = node_ub[i];
  }
  return solve_lp(node_lp, x0);
}

}  // namespace mipsolvers::engine
