#include "hacdcpf/optimal_power_flow/native_ipm_solver.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

// High-performance sparse KKT factorizations from SuiteSparse, used for large
// systems (≳1500 KKT unknowns).  UMFPACK and KLU provide stronger numerical
// pivoting than Eigen's built-in SparseLU and separate the symbolic analysis
// (reused across iterations) from the per-iteration numeric factorization.
// Gated on HACDCPF_OPF_HAVE_* so the build still works without SuiteSparse.
#if defined(HACDCPF_OPF_HAVE_UMFPACK)
#include <Eigen/UmfPackSupport>
#endif
#if defined(HACDCPF_OPF_HAVE_KLU)
#include <Eigen/KLUSupport>
#endif

// MUMPS multifrontal symmetric-indefinite LDLᵀ solver from MIPSolvers.  The
// KKT is symmetric; exploiting that halves both the factorization work and
// the fill versus an unsymmetric LU, and MUMPS' nested-dissection orderings
// are near-optimal on the planar/mesh graphs of power networks.  The define
// is inherited from the mipsolvers target's public compile definitions.
#if defined(HACDCPF_HAVE_MUMPS)
#include <mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp>
#endif

#include "hacdcpf/detail/logging.hpp"

namespace hacdcpf::opf::parity {

namespace {

double inf_norm(const Eigen::VectorXd& v) {
  return (v.size() == 0) ? 0.0 : v.cwiseAbs().maxCoeff();
}

// Fraction-to-boundary rule: max alpha s.t. v + alpha*dv > 0
double ftb(const Eigen::VectorXd& v, const Eigen::VectorXd& dv, double tau) {
  double alpha = 1.0;
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (dv[i] < 0.0) {
      const double candidate = -tau * v[i] / dv[i];
      if (candidate < alpha) {
        alpha = candidate;
      }
    }
  }
  return alpha;
}

// Sparse KKT linear-solver backend.  MUMPS (multifrontal symmetric-indefinite
// LDLᵀ) is the scalability path for large grids; UMFPACK and KLU (SuiteSparse)
// offer stronger pivoting than Eigen's SparseLU with separable symbolic/numeric
// factorization; Eigen's SparseLU is the always-available fallback.
enum class SparseBackend { Auto, Umfpack, Klu, EigenLU, Mumps };

std::string linear_solver_preference_env() {
  const char* env = std::getenv("HACDCPF_OPF_LINEAR_SOLVER");
  if (env == nullptr || std::string(env).empty()) {
    // Backward-compatible spelling used before the dense backend could be
    // selected explicitly.
    env = std::getenv("HACDCPF_OPF_SPARSE_SOLVER");
  }
  if (env != nullptr) {
    std::string s(env);
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    return s;
  }
  return {};
}

bool has_linear_solver_override() {
  const std::string s = linear_solver_preference_env();
  return !s.empty() && s != "auto";
}

bool dense_backend_forced() {
  const std::string s = linear_solver_preference_env();
  return s == "dense" || s == "dense_lu" || s == "lu";
}

// One-time backend override for benchmarking/diagnostics:
//   HACDCPF_OPF_LINEAR_SOLVER = dense | mumps | umfpack | klu | eigen | auto
//   HACDCPF_OPF_SPARSE_SOLVER = mumps | umfpack | klu | eigen | auto (legacy alias)
SparseBackend sparse_backend_preference() {
  const std::string s = linear_solver_preference_env();
  if (!s.empty()) {
    if (s == "mumps") return SparseBackend::Mumps;
    if (s == "umfpack") return SparseBackend::Umfpack;
    if (s == "klu") return SparseBackend::Klu;
    if (s == "eigen" || s == "sparselu" || s == "sparse_eigen_lu") {
      return SparseBackend::EigenLU;
    }
  }
  return SparseBackend::Auto;
}

// ── Lightweight cumulative profiler (HACDCPF_OPF_PROF=1) ────────────────────
// Decomposes parity-IPM wall time into the three candidate bottlenecks:
// KKT numeric factorization, KKT back-solve, and KKT assembly (triplet build +
// equilibration).  Accumulators are process-global and printed at exit; the
// IPM is single-threaded today, so plain doubles suffice.
struct IpmProf {
  double t_factorize{0.0}, t_solve{0.0}, t_assembly{0.0};
  long   n_factorize{0}, n_solve{0}, n_assembly{0};
  // Per-backend factorize breakdown (index: 1=UMFPACK, 2=KLU, 3=Eigen, 4=MUMPS).
  double t_fbe[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  long   n_fbe[5]{0, 0, 0, 0, 0};
  bool   enabled{false};
  IpmProf() : enabled(std::getenv("HACDCPF_OPF_PROF") != nullptr) {}
  static const char* be_name(int b) {
    switch (b) {
      case 1: return "umfpack"; case 2: return "klu";
      case 3: return "eigen";   case 4: return "mumps";
      default: return "?";
    }
  }
  ~IpmProf() {
    if (!enabled) return;
    std::fprintf(stderr,
        "[prof] factorize %.3fs (%ld, %.4fs avg) | solve %.3fs (%ld, %.4fs) | "
        "assembly %.3fs (%ld)\n",
        t_factorize, n_factorize, n_factorize ? t_factorize / n_factorize : 0.0,
        t_solve, n_solve, n_solve ? t_solve / n_solve : 0.0,
        t_assembly, n_assembly);
    for (int b = 1; b <= 4; ++b) {
      if (n_fbe[b])
        std::fprintf(stderr, "[prof]   %-8s factorize %.3fs (%ld, %.4fs avg)\n",
                     be_name(b), t_fbe[b], n_fbe[b], t_fbe[b] / n_fbe[b]);
    }
  }
};
inline IpmProf& ipm_prof() { static IpmProf p; return p; }
struct IpmProfTimer {
  double& acc; long& cnt; bool on;
  std::chrono::steady_clock::time_point t0;
  IpmProfTimer(double& a, long& c, bool on_)
      : acc(a), cnt(c), on(on_), t0(std::chrono::steady_clock::now()) {
    if (on) ++cnt;
  }
  void stop() {
    if (on) {
      acc += std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
      on = false;
    }
  }
  ~IpmProfTimer() { stop(); }
};

// Newton-system form for the KKT solve.  The condensed form eliminates the
// inequality multipliers into W = Lxx + JhᵀΣJh: a sparse triple product per
// iteration, extra fill-in, and — decisively — a squared condition number.
// With the barrier ratio σ = μ/z spanning ~1e±10 near the boundary, κ(W)
// reaches the regime where the computed Newton step is numerically
// perpendicular to the true one ("accurately-solved but wrong step"),
// observed as filter line-search stalls on the PEGASE cases with *every*
// linear solver.  The augmented form keeps the −ZM⁻¹ diagonal block
// explicitly: no triple product, no squaring, and with an LDLᵀ backend the
// inertia is exactly checkable (negevals must equal meq+niq), which drives
// the Wächter–Biegler δ_W correction below.
enum class KktForm { Condensed, Augmented };

// HACDCPF_OPF_KKT_FORM = augmented | condensed.
//
// Condensed is the production default.  It is the mature fast path for the
// ordinary economic OPF benchmarks (case2000_acdc: ~2 s, case2869pegase:
// 54 iterations in Release).  Merely compiling MUMPS must not silently switch
// every model to the augmented trajectory: that policy regressed both cases
// into long filter stalls.  Augmented KKT remains available explicitly for
// stiff models that benefit from inertia-controlled regularization.
KktForm kkt_form_preference() {
  const char* env = std::getenv("HACDCPF_OPF_KKT_FORM");
  if (env != nullptr) {
    std::string s(env);
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    if (s == "condensed") return KktForm::Condensed;
    if (s == "augmented") return KktForm::Augmented;
  }
  return KktForm::Condensed;
}

// Cache for the sparse KKT system.  Holds the assembled matrix plus whichever
// factorization backend is active; the symbolic analysis is computed once and
// reused across iterations (the KKT sparsity pattern is invariant — only the
// numeric values change with the barrier and regularization).
struct SparseKKTCache {
  Eigen::SparseMatrix<double> kkt;
  Eigen::SparseMatrix<double> kkt_orig;
  // Symmetric Ruiz equilibration scaling: the factored matrix is D·K·D, so a
  // solve of K·x=b is run as (D·K·D)·y = D·b with x = D·y.  Balancing the rows
  // and columns is essential for hybrid AC/DC KKTs, where AC balance rows are
  // pre-scaled to O(1) while DC/converter rows can be orders of magnitude
  // smaller (e.g. a sub-MW microgrid on a 100 MVA base) — left unbalanced the
  // DC block looks singular to the factorization ("KKT factorization failed").
  Eigen::VectorXd scale;
#if defined(HACDCPF_OPF_HAVE_UMFPACK)
  Eigen::UmfPackLU<Eigen::SparseMatrix<double>> umf;
#endif
#if defined(HACDCPF_OPF_HAVE_KLU)
  Eigen::KLU<Eigen::SparseMatrix<double>> klu;
#endif
  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu;
#if defined(HACDCPF_HAVE_MUMPS)
  mipsolvers::engine::MumpsSolver mumps;
#endif
  int active{0};         ///< 0=unset, 1=UMFPACK, 2=KLU, 3=Eigen SparseLU, 4=MUMPS
  bool pure_ac{false};   ///< no DC/VSC/ER subsystems → KLU-eligible (see below)
  // Per-backend symbolic-analysis validity.  The KKT sparsity pattern is FIXED
  // across IPM iterations (same structure, changing values), so each backend
  // only needs to analyse the pattern ONCE — recording it per backend lets the
  // inertia gate switch KLU↔MUMPS without re-analysing on every switch (that
  // churn was the dominant gate overhead: ~18 s at case13659).  Indexed by
  // backend id; -1 = not analysed for the current pattern.
  int analyzed_dim[5]{-1, -1, -1, -1, -1};
  int analyzed_nnz[5]{-1, -1, -1, -1, -1};
  std::uint64_t analyzed_pattern[5]{0, 0, 0, 0, 0};
  bool factored{false};
  bool solve_degraded{false};  ///< last solve was inaccurate → escalate backend
  int symbolic_analyze_calls{0};
  int backend_escalations{0};
  int scaling_rebuilds{0};
  int nn{0};
  int meq{0};
  int niq{0};        ///< inequality-multiplier block size (augmented form only)
};

std::uint64_t sparse_pattern_signature(
    const Eigen::SparseMatrix<double>& matrix) {
  // FNV-1a over the compressed sparse coordinates. Symbolic factorization is
  // reusable only for the identical graph; equal dimensions and nnz alone do
  // not establish that contract (Davis, Direct Methods for Sparse Linear
  // Systems, SIAM 2006, Sections 4.1 and 7.6).
  std::uint64_t hash = 1469598103934665603ULL;
  const auto add = [&](std::uint64_t value) {
    hash ^= value;
    hash *= 1099511628211ULL;
  };
  add(static_cast<std::uint64_t>(matrix.rows()));
  add(static_cast<std::uint64_t>(matrix.cols()));
  add(static_cast<std::uint64_t>(matrix.nonZeros()));
  for (int col = 0; col < matrix.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, col); it; ++it) {
      add(static_cast<std::uint64_t>(it.row()));
      add(static_cast<std::uint64_t>(it.col()));
    }
  }
  return hash == 0 ? 1 : hash;
}

// Ordered candidate backends for a preference (most-preferred first), filtered
// to those compiled in and de-duplicated.  The Auto order is structure-aware:
//
//  * Pure-AC problems (pure_ac=true, no DC/VSC/energy-router subsystems) put
//    KLU first.  KLU's block-triangular-form (BTF) decomposition matches the
//    near-block-triangular structure of network KKTs, giving far less fill
//    than a general multifrontal LDLᵀ — measured on case2869 (factorize
//    0.006 s vs MUMPS 0.015 s, end-to-end 5.97 s vs 12.05 s, SAME exact
//    reference objective 133999) and case13659 (22.9 s vs 33.9 s, with ~100×
//    tighter stationarity).  MUMPS and UMFPACK follow as escalation failovers.
//
//  * Hybrid AC/DC problems (pure_ac=false) keep MUMPS first: the augmented
//    KKT's Wächter–Biegler δ_W regularisation needs the LDLᵀ inertia (negative
//    pivot count), which only MUMPS reports — on stiff hybrid cases KLU/LU
//    accept the unregularised factorisation and stagnate (measured:
//    case1354+500 MW dc converges with MUMPS, stagnates with KLU).  MUMPS'
//    symmetric LDLᵀ also halves work/fill versus unsymmetric LU here, and its
//    nested-dissection ordering suits the mesh-like network graph.
//
// UMFPACK's pivoting stays accurate on near-singular KKTs; Eigen SparseLU is
// ~3–7× faster on well-conditioned KKTs but returns an accurately-solved-yet
// -wrong step on very large/ill-conditioned grids.  A caller can pin any
// backend via HACDCPF_OPF_SPARSE_SOLVER (overrides this structure logic).
std::vector<int> backend_order(SparseBackend pref, bool pure_ac) {
  std::vector<int> order;
#if defined(HACDCPF_OPF_HAVE_UMFPACK)
  const bool have_umf = true;
#else
  const bool have_umf = false;
#endif
#if defined(HACDCPF_OPF_HAVE_KLU)
  const bool have_klu = true;
#else
  const bool have_klu = false;
#endif
#if defined(HACDCPF_HAVE_MUMPS)
  const bool have_mumps = true;
#else
  const bool have_mumps = false;
#endif
  switch (pref) {
    case SparseBackend::Mumps:
      if (have_mumps) order.push_back(4);
      if (have_umf) order.push_back(1);
      if (have_klu) order.push_back(2);
      order.push_back(3);
      break;
    case SparseBackend::Umfpack:
      if (have_umf) order.push_back(1);
      order.push_back(3);
      if (have_klu) order.push_back(2);
      break;
    case SparseBackend::Klu:
      if (have_klu) order.push_back(2);
      order.push_back(3);
      if (have_umf) order.push_back(1);
      break;
    case SparseBackend::EigenLU:
      order.push_back(3);
      break;
    case SparseBackend::Auto:
    default:
      if (pure_ac) {
        // Pure-AC: KLU first (BTF structure, measured fastest on network KKTs).
        if (have_klu) order.push_back(2);
        if (have_mumps) order.push_back(4);
        if (have_umf) order.push_back(1);
        order.push_back(3);
      } else {
        // Hybrid AC/DC: MUMPS first (LDLᵀ inertia needed for δ_W regularisation).
        if (have_mumps) order.push_back(4);
        if (have_umf) order.push_back(1);
        if (have_klu) order.push_back(2);
        order.push_back(3);
      }
      break;
  }
  std::vector<int> uniq;
  for (int b : order) {
    if (std::find(uniq.begin(), uniq.end(), b) == uniq.end()) uniq.push_back(b);
  }
  return uniq;
}

// Numeric factorization for the active backend; analyzes the pattern only when
// it has changed (first call or a genuine pattern change).
bool sparse_factorize_active(SparseKKTCache& cache, bool pattern_changed) {
  const bool _prof = ipm_prof().enabled;
  const int  _be = cache.active;  // backend is fixed for this call
  const auto _t0 = std::chrono::steady_clock::now();
  bool ok = false;
  if (pattern_changed) ++cache.symbolic_analyze_calls;
  switch (cache.active) {
#if defined(HACDCPF_OPF_HAVE_UMFPACK)
    case 1:
      if (pattern_changed) {
        cache.umf.analyzePattern(cache.kkt);
      }
      cache.umf.factorize(cache.kkt);
      ok = cache.umf.info() == Eigen::Success;
      break;
#endif
#if defined(HACDCPF_OPF_HAVE_KLU)
    case 2:
      if (pattern_changed) {
        cache.klu.analyzePattern(cache.kkt);
      }
      cache.klu.factorize(cache.kkt);
      ok = cache.klu.info() == Eigen::Success;
      break;
#endif
#if defined(HACDCPF_HAVE_MUMPS)
    case 4:
      if (pattern_changed) {
        cache.mumps.analyze_pattern(cache.kkt);
      }
      ok = cache.mumps.factorize(cache.kkt);
      break;
#endif
    default:
      if (pattern_changed) {
        cache.lu.analyzePattern(cache.kkt);
      }
      cache.lu.factorize(cache.kkt);
      ok = cache.lu.info() == Eigen::Success;
      break;
  }
  if (_prof) {
    const double dt = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - _t0).count();
    ipm_prof().t_factorize += dt;
    ++ipm_prof().n_factorize;
    if (_be >= 1 && _be <= 4) {
      ipm_prof().t_fbe[_be] += dt;
      ++ipm_prof().n_fbe[_be];
    }
  }
  return ok;
}

// Back-solve for the active backend.
Eigen::VectorXd sparse_solve_active(SparseKKTCache& cache, const Eigen::VectorXd& rhs) {
  IpmProfTimer _pt(ipm_prof().t_solve, ipm_prof().n_solve, ipm_prof().enabled);
  switch (cache.active) {
#if defined(HACDCPF_OPF_HAVE_UMFPACK)
    case 1: return cache.umf.solve(rhs);
#endif
#if defined(HACDCPF_OPF_HAVE_KLU)
    case 2: return cache.klu.solve(rhs);
#endif
#if defined(HACDCPF_HAVE_MUMPS)
    case 4: {
      Eigen::VectorXd x(rhs.size());
      // A failed solve surfaces as non-finite values so the caller flags the
      // backend as degraded and escalates to the next candidate.
      if (!cache.mumps.solve(rhs, x)) {
        return Eigen::VectorXd::Constant(
            rhs.size(), std::numeric_limits<double>::quiet_NaN());
      }
      return x;
    }
#endif
    default: return cache.lu.solve(rhs);
  }
}

// Symmetric Ruiz equilibration: find a positive diagonal D so the rows (and, by
// symmetry, columns) of D·K·D have unit ∞-norm.  A few sweeps are enough to pull
// a badly-scaled symmetric indefinite KKT into a factorable range.  Zero/empty
// rows keep D=1 (no division).
Eigen::VectorXd compute_ruiz_scaling(const Eigen::SparseMatrix<double>& K, int sweeps = 5) {
  const int dim = static_cast<int>(K.rows());
  Eigen::VectorXd D = Eigen::VectorXd::Ones(dim);
  Eigen::VectorXd rmax(dim);
  for (int s = 0; s < sweeps; ++s) {
    rmax.setZero();
    for (int col = 0; col < K.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(K, col); it; ++it) {
        // Magnitude of the entry under the current accumulated scaling.
        const double v = std::abs(it.value()) * D[it.row()] * D[it.col()];
        if (v > rmax[it.row()]) rmax[it.row()] = v;
      }
    }
    double max_dev = 0.0;
    for (int i = 0; i < dim; ++i) {
      if (rmax[i] > 0.0) {
        D[i] /= std::sqrt(rmax[i]);
        max_dev = std::max(max_dev, std::abs(std::log(rmax[i])));
      }
    }
    if (max_dev < 1e-3) break;  // already balanced
  }
  return D;
}

bool factor_assembled_kkt(SparseKKTCache& cache,
                          std::vector<Eigen::Triplet<double>>& trips);

bool factor_kkt_sparse(SparseKKTCache& cache,
                       const Eigen::SparseMatrix<double>& w,
                       const Eigen::SparseMatrix<double>& jg,
                       double reg) {
  const int nn = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  cache.nn = nn;
  cache.meq = meq;
  const int dim = nn + meq;

  // Assemble KKT as sparse matrix
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<size_t>(w.nonZeros() + 2 * jg.nonZeros() + dim));

  // (1,1) block: W + δI
  for (int col = 0; col < w.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(w, col); it; ++it) {
      trips.emplace_back(it.row(), it.col(), it.value());
    }
  }
  for (int i = 0; i < nn; ++i) {
    trips.emplace_back(i, i, reg);
  }

  // (1,2) block: Jg'  and (2,1) block: Jg
  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      trips.emplace_back(it.col(), nn + it.row(), it.value());
      trips.emplace_back(nn + it.row(), it.col(), it.value());
    }
  }

  // (2,2) block: -δI
  for (int i = 0; i < meq; ++i) {
    trips.emplace_back(nn + i, nn + i, -reg);
  }

  return factor_assembled_kkt(cache, trips);
}

// Equilibrate and factor the assembled KKT triplets with the ordered backend
// candidates (sticky active backend + escalation).  Shared by the condensed
// and augmented forms; the caller fills cache.nn/meq/niq to its block sizes.
bool factor_assembled_kkt(SparseKKTCache& cache,
                          std::vector<Eigen::Triplet<double>>& trips) {
  IpmProfTimer _asm(ipm_prof().t_assembly, ipm_prof().n_assembly,
                    ipm_prof().enabled);
  const int dim = cache.nn + cache.meq + cache.niq;
  cache.kkt.resize(dim, dim);
  cache.kkt.setFromTriplets(trips.begin(), trips.end());
  cache.kkt.makeCompressed();

  // Equilibrate: replace K with D·K·D so the factorization sees a balanced
  // matrix.  The scaling is computed ONCE per solve and frozen by default:
  // rebuilding the equilibration every iteration lets the extreme −z/μ
  // ratios swing the row norms by orders of magnitude, which jitters the
  // effective Newton direction on stiff (hybrid AC/DC) KKTs and shows up as
  // a feasibility limit cycle above the tolerance (root cause of the
  // IEEE24-3area-expanded OPF stall: 640 iters → 36 with frozen scaling).
  // HACDCPF_OPF_RESCALE_PER_ITER=1 restores the old per-iteration rebuild.
  const bool freeze_scaling = []() {
    static const bool v =
        std::getenv("HACDCPF_OPF_RESCALE_PER_ITER") == nullptr;
    return v;
  }();
  if (!freeze_scaling || cache.scale.size() != dim) {
    cache.scale = compute_ruiz_scaling(cache.kkt);
    ++cache.scaling_rebuilds;
  }
  for (int col = 0; col < cache.kkt.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(cache.kkt, col); it; ++it) {
      it.valueRef() *= cache.scale[it.row()] * cache.scale[it.col()];
    }
  }
  cache.kkt_orig = cache.kkt;
  _asm.stop();  // assembly + equilibration ends here; factorization follows

  const int nnz = static_cast<int>(cache.kkt.nonZeros());
  const std::uint64_t pattern_signature =
      sparse_pattern_signature(cache.kkt);
  const std::vector<int> order =
      backend_order(sparse_backend_preference(), cache.pure_ac);
  if (cache.active == 0) cache.active = order.front();

  // If the previous solve was inaccurate (a sign the current backend cannot
  // handle this KKT), escalate to the next, more robust backend in the order.
  if (cache.solve_degraded) {
    const auto it = std::find(order.begin(), order.end(), cache.active);
    if (it != order.end() && std::next(it) != order.end()) {
      cache.active = *std::next(it);
      ++cache.backend_escalations;
    }
    cache.solve_degraded = false;
  }

  // Try the sticky active backend first (its symbolic analysis is reused when
  // the pattern is unchanged), then escalate through the remaining candidates.
  std::vector<int> trylist;
  trylist.push_back(cache.active);
  for (int b : order) {
    if (b != cache.active) trylist.push_back(b);
  }

  for (int backend : trylist) {
    cache.active = backend;
    const bool need_analyze =
        cache.analyzed_dim[backend] != dim ||
        cache.analyzed_nnz[backend] != nnz ||
        cache.analyzed_pattern[backend] != pattern_signature;
    if (sparse_factorize_active(cache, need_analyze)) {
      if (need_analyze) {
        cache.analyzed_dim[backend] = dim;
        cache.analyzed_nnz[backend] = nnz;
        cache.analyzed_pattern[backend] = pattern_signature;
      }
      cache.factored = true;
      return true;
    }
  }

  cache.factored = false;
  return false;
}

// Augmented-form assembly: keep the inequality multipliers μ as unknowns
// instead of condensing them into W = Lxx + JhᵀΣJh.
//
//   [ Lxx+δW·I   Jgᵀ      Jhᵀ  ] [dx]   [−rd ]
//   [ Jg        −δC·I     0   ] [dλ] = [−req]
//   [ Jh          0     −ZM⁻¹ ] [dμ]   [rhs3]
//
// No triple product is formed, the conditioning is linear (not squared) in
// the barrier ratio σ = μ/z, and the LDLᵀ inertia is exactly checkable:
// with the reduced Hessian positive definite, inertia = (n, meq+niq, 0).
bool factor_kkt_sparse_augmented(SparseKKTCache& cache,
                                 const Eigen::SparseMatrix<double>& lxx,
                                 const Eigen::SparseMatrix<double>& jg,
                                 const Eigen::SparseMatrix<double>& dh,
                                 const Eigen::VectorXd& z_safe,
                                 const Eigen::VectorXd& mu_safe,
                                 double delta_w, double delta_c) {
  const int nn = static_cast<int>(lxx.rows());
  const int meq = static_cast<int>(jg.rows());
  const int niq = static_cast<int>(dh.rows());
  cache.nn = nn;
  cache.meq = meq;
  cache.niq = niq;

  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<size_t>(lxx.nonZeros() +
                                    2 * (jg.nonZeros() + dh.nonZeros()) +
                                    nn + meq + niq));

  // (1,1) block: Lxx + δW·I
  for (int col = 0; col < lxx.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lxx, col); it; ++it) {
      trips.emplace_back(it.row(), it.col(), it.value());
    }
  }
  for (int i = 0; i < nn; ++i) {
    trips.emplace_back(i, i, delta_w);
  }

  // (1,2)/(2,1) blocks: Jgᵀ / Jg
  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      trips.emplace_back(it.col(), nn + it.row(), it.value());
      trips.emplace_back(nn + it.row(), it.col(), it.value());
    }
  }

  // (2,2) block: −δC·I
  for (int i = 0; i < meq; ++i) {
    trips.emplace_back(nn + i, nn + i, -delta_c);
  }

  // (1,3)/(3,1) blocks: Jhᵀ / Jh
  for (int col = 0; col < dh.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(dh, col); it; ++it) {
      trips.emplace_back(it.col(), nn + meq + it.row(), it.value());
      trips.emplace_back(nn + meq + it.row(), it.col(), it.value());
    }
  }

  // (3,3) block: −ZM⁻¹ (strictly negative diagonal)
  for (int i = 0; i < niq; ++i) {
    trips.emplace_back(nn + meq + i, nn + meq + i,
                       -z_safe[i] / mu_safe[i]);
  }

  return factor_assembled_kkt(cache, trips);
}

// Scaled-space KKT solve with iterative refinement: returns the full solution
// vector (unscaled).  Shared by the condensed and augmented forms.
bool kkt_solve_scaled(SparseKKTCache& cache,
                      const Eigen::VectorXd& rhs,
                      Eigen::VectorXd& sol) {
  if (!cache.factored) {
    return false;
  }
  // The factored matrix is the equilibrated D·K·D, so solve in the scaled space:
  //   (D·K·D)·y = D·b,   x = D·y.   kkt_orig is the scaled matrix, so iterative
  //   refinement below is self-consistent in the scaled space.
  const Eigen::VectorXd& D = cache.scale;
  const Eigen::VectorXd rhs_s = D.cwiseProduct(rhs);
  sol = sparse_solve_active(cache, rhs_s);
  if (!sol.allFinite()) {
    cache.solve_degraded = true;
    return false;
  }
  // Iterative refinement (2 steps) — reuses the existing factorization.
  // Divergence guard: refinement is contractive only while κ·ε ≪ 1; on a
  // numerically singular KKT the "correction" amplifies near-null-space noise
  // instead of shrinking the residual (observed: ‖d‖ ~ 1e85 after refinement
  // in the rte endgame).  Apply a correction only if it provably reduces the
  // residual; otherwise keep the unrefined solve.
  for (int ref = 0; ref < 2; ++ref) {
    Eigen::VectorXd residual = rhs_s - cache.kkt_orig * sol;
    const double res_norm = residual.cwiseAbs().maxCoeff();
    if (res_norm < 1e-14 * rhs_s.cwiseAbs().maxCoeff()) {
      break;
    }
    Eigen::VectorXd correction = sparse_solve_active(cache, residual);
    if (!correction.allFinite()) {
      break;
    }
    const double new_res =
        (rhs_s - cache.kkt_orig * (sol + correction)).cwiseAbs().maxCoeff();
    if (new_res >= res_norm) {
      break;
    }
    sol += correction;
  }
  // Flag an inaccurate Eigen SparseLU solve so the next factorization can try
  // another backend when one is available.  Do not downgrade away from
  // UMFPACK/KLU on a warning residual: those SuiteSparse backends are already
  // the robust sparse path, and switching backend mid-IPM changes the trajectory
  // enough to break otherwise convergent hybrid AC/DC cases.
  const double rhs_norm = rhs_s.cwiseAbs().maxCoeff();
  if (rhs_norm > 0.0) {
    const double resid = (rhs_s - cache.kkt_orig * sol).cwiseAbs().maxCoeff();
    if (cache.active == 3 && resid > 1e-6 * rhs_norm) {
      cache.solve_degraded = true;
    }
  }
  // Undo the column scaling: x = D·y.
  sol = D.cwiseProduct(sol);
  return true;
}

bool kkt_solve_sparse(SparseKKTCache& cache,
                      const Eigen::VectorXd& rhs,
                      Eigen::VectorXd& dx,
                      Eigen::VectorXd& dlambda) {
  Eigen::VectorXd sol;
  if (!kkt_solve_scaled(cache, rhs, sol)) {
    return false;
  }
  dx = sol.head(cache.nn);
  dlambda = sol.tail(cache.meq);
  return true;
}

// Augmented-form solve: the solution carries the inequality multipliers dμ in
// the tail block directly — no Σ-weighted recovery formula is needed.
bool kkt_solve_sparse_augmented(SparseKKTCache& cache,
                                const Eigen::VectorXd& rhs,
                                Eigen::VectorXd& dx,
                                Eigen::VectorXd& dlambda,
                                Eigen::VectorXd& dmu) {
  Eigen::VectorXd sol;
  if (!kkt_solve_scaled(cache, rhs, sol)) {
    return false;
  }
  dx = sol.head(cache.nn);
  dlambda = sol.segment(cache.nn, cache.meq);
  dmu = sol.tail(cache.niq);
  return true;
}

// Dense KKT cache (for small systems where dense is competitive)
struct DenseKKTCache {
  Eigen::MatrixXd buf;
  Eigen::MatrixXd buf_orig;  // copy for iterative refinement
  Eigen::PartialPivLU<Eigen::MatrixXd> lu;
  bool factored{false};
  int nn{0};
  int meq{0};
};

// Fill KKT = [W+δI, Jg'; Jg, -δI] into dense buffer and factor with pivoted LU
bool factor_kkt_dense(DenseKKTCache& cache,
                      const Eigen::SparseMatrix<double>& w,
                      const Eigen::SparseMatrix<double>& jg,
                      double reg) {
  const int nn = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  cache.nn = nn;
  cache.meq = meq;
  const int dim = nn + meq;

  cache.buf.setZero(dim, dim);

  // (1,1) block: W + δI
  for (int col = 0; col < w.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(w, col); it; ++it) {
      cache.buf(it.row(), it.col()) += it.value();
    }
  }
  for (int i = 0; i < nn; ++i) {
    cache.buf(i, i) += reg;
  }

  // (1,2) and (2,1) blocks: Jg' and Jg
  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      cache.buf(it.col(), nn + it.row()) = it.value();  // Jg' (top-right)
      cache.buf(nn + it.row(), it.col()) = it.value();  // Jg  (bottom-left)
    }
  }

  // (2,2) block: -δI
  for (int i = 0; i < meq; ++i) {
    cache.buf(nn + i, nn + i) = -reg;
  }

  cache.buf_orig = cache.buf;  // save for iterative refinement
  // The KKT operator on R^0 has the unique empty solution. Eigen's dynamic
  // dense LU must not be asked to factor a 0-by-0 buffer (Nocedal & Wright,
  // Numerical Optimization, 2nd ed., Section 16.1, empty-block specialization).
  if (dim == 0) {
    cache.factored = true;
    return true;
  }
  cache.lu.compute(cache.buf);
  cache.factored = true;
  return true;
}

bool kkt_solve_dense(DenseKKTCache& cache,
                     const Eigen::VectorXd& rhs,
                     Eigen::VectorXd& dx,
                     Eigen::VectorXd& dlambda) {
  if (!cache.factored) {
    return false;
  }
  const Eigen::Index dim = static_cast<Eigen::Index>(cache.nn + cache.meq);
  if (rhs.size() != dim) {
    return false;
  }
  if (dim == 0) {
    dx.resize(cache.nn);
    dlambda.resize(cache.meq);
    return true;
  }
  Eigen::VectorXd sol = cache.lu.solve(rhs);
  if (!sol.allFinite()) {
    return false;
  }
  // Iterative refinement: 2 steps to combat ill-conditioning from extreme σ ratios
  for (int ref = 0; ref < 2; ++ref) {
    Eigen::VectorXd residual = rhs - cache.buf_orig * sol;
    if (residual.cwiseAbs().maxCoeff() < 1e-14 * rhs.cwiseAbs().maxCoeff()) {
      break;
    }
    Eigen::VectorXd correction = cache.lu.solve(residual);
    if (!correction.allFinite()) {
      break;
    }
    sol += correction;
  }
  dx = sol.head(cache.nn);
  dlambda = sol.tail(cache.meq);
  return true;
}

void assemble_inequalities(const Problem& prob,
                           const Eigen::VectorXd& x,
                           const Eigen::VectorXd& xmin,
                           const Eigen::VectorXd& xmax,
                           const std::vector<int>& lb_cols,
                           const std::vector<int>& ub_cols,
                           Eigen::VectorXd& h,
                           Eigen::SparseMatrix<double>& jh) {
  Eigen::VectorXd h_nonlin;
  Eigen::SparseMatrix<double> jh_nonlin;
  nonlinear_inequality_constraints(prob, x, h_nonlin);
  nonlinear_inequality_jacobian(prob, x, jh_nonlin);

  const int nn = prob.vidx.n_total;
  const int m_nonlin = prob.cidx.n_ineq_nonlin;
  const int m = m_nonlin + static_cast<int>(lb_cols.size()) + static_cast<int>(ub_cols.size());

  h = Eigen::VectorXd::Zero(m);
  std::vector<Eigen::Triplet<double>> t;
  t.reserve(static_cast<size_t>(jh_nonlin.nonZeros() + lb_cols.size() + ub_cols.size()));

  if (m_nonlin > 0) {
    h.head(m_nonlin) = h_nonlin;
    for (int col = 0; col < jh_nonlin.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(jh_nonlin, col); it; ++it) {
        t.emplace_back(it.row(), it.col(), it.value());
      }
    }
  }

  int row = m_nonlin;
  for (int c : lb_cols) {
    h[row] = xmin[c] - x[c];
    t.emplace_back(row, c, -1.0);
    ++row;
  }
  for (int c : ub_cols) {
    h[row] = x[c] - xmax[c];
    t.emplace_back(row, c, 1.0);
    ++row;
  }

  jh.resize(m, nn);
  jh.setFromTriplets(t.begin(), t.end());
  jh.makeCompressed();
}

struct PhaseOneState {
  double initial_violation{std::numeric_limits<double>::infinity()};
  double final_violation{std::numeric_limits<double>::infinity()};
  double dual_fit_residual{std::numeric_limits<double>::infinity()};
  bool primal_feasible{false};
  bool in_handoff_corridor{false};
  bool dual_initialized{false};
  bool central_start_accepted{false};
  double handoff_primal_tolerance{0.0};
  double perturbed_primal_residual{
      std::numeric_limits<double>::infinity()};
  double centrality{std::numeric_limits<double>::infinity()};
  double barrier_mu{0.0};
  std::string rejection_reason;
  bool budget_exhausted{false};
  int iterations{0};
  int factorizations{0};
  int backtracks{0};
  bool structural_step_attempted{false};
  bool structural_step_accepted{false};
  int structural_factorizations{0};
  double structural_violation{std::numeric_limits<double>::infinity()};
  std::string structure{"ac-state-basic"};
  double runtime_ms{0.0};
  std::string termination{"not-run"};
  std::string linear_solver{"unselected"};
  std::vector<int> basic_columns;
};

struct DispatchDualPredictorDiagnostics {
  bool attempted{false};
  bool accepted{false};
  double runtime_ms{0.0};
  double baseline_raw{std::numeric_limits<double>::infinity()};
  double candidate_raw{std::numeric_limits<double>::infinity()};
  double baseline_normalized{std::numeric_limits<double>::infinity()};
  double candidate_normalized{std::numeric_limits<double>::infinity()};
  std::string status{"not-attempted"};
};

DispatchDualPredictorDiagnostics predict_component_dispatch_duals(
    const Problem& prob,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& xmin,
    const Eigen::VectorXd& xmax,
    const Eigen::VectorXd& gradient,
    double objective_scale,
    const Eigen::SparseMatrix<double>& equality_jacobian,
    const Eigen::SparseMatrix<double>& inequality_jacobian,
    const Eigen::VectorXd& inequality_dual,
    double minimum_improvement,
    Eigen::VectorXd& equality_dual) {
  DispatchDualPredictorDiagnostics diagnostics;
  diagnostics.attempted = true;
  const auto started = std::chrono::steady_clock::now();
  const auto finish = [&](std::string status) {
    diagnostics.status = std::move(status);
    diagnostics.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return diagnostics;
  };
  const int buses = prob.vidx.n_va;
  if (buses <= 0 || prob.vidx.n_pg <= 0 ||
      equality_dual.size() != prob.cidx.n_eq_total) {
    return finish("rejected: no AC generator component");
  }

  // Dommel--Tinney (1968): an OPF reduced-gradient/adjoint seed follows the
  // power-network graph. We retain only its zeroth-order island-price term;
  // unlike a PF-adjoint fit, this requires no sparse factorization and is
  // rebuilt safely after topology changes. Taheri--Molzahn (2026,
  // arXiv:2606.08984) motivates the strict full-residual rejection below:
  // partial primal/dual coverage often makes ACOPF restarts slower or fragile.
  std::vector<int> parent(static_cast<std::size_t>(buses));
  std::vector<unsigned char> rank(static_cast<std::size_t>(buses), 0);
  for (int bus = 0; bus < buses; ++bus) {
    parent[static_cast<std::size_t>(bus)] = bus;
  }
  const auto find_root = [&](int bus) {
    int root = bus;
    while (parent[static_cast<std::size_t>(root)] != root) {
      root = parent[static_cast<std::size_t>(root)];
    }
    while (parent[static_cast<std::size_t>(bus)] != bus) {
      const int next = parent[static_cast<std::size_t>(bus)];
      parent[static_cast<std::size_t>(bus)] = root;
      bus = next;
    }
    return root;
  };
  const auto unite = [&](int lhs, int rhs) {
    int root_lhs = find_root(lhs);
    int root_rhs = find_root(rhs);
    if (root_lhs == root_rhs) return;
    if (rank[static_cast<std::size_t>(root_lhs)] <
        rank[static_cast<std::size_t>(root_rhs)]) {
      std::swap(root_lhs, root_rhs);
    }
    parent[static_cast<std::size_t>(root_rhs)] = root_lhs;
    if (rank[static_cast<std::size_t>(root_lhs)] ==
        rank[static_cast<std::size_t>(root_rhs)]) {
      ++rank[static_cast<std::size_t>(root_lhs)];
    }
  };
  for (int col = 0; col < prob.data.ybus.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(
             prob.data.ybus, col); it; ++it) {
      if (it.row() != it.col() && std::abs(it.value()) > 0.0) {
        unite(static_cast<int>(it.row()), static_cast<int>(it.col()));
      }
    }
  }
  for (int bus = 0; bus < buses; ++bus) {
    parent[static_cast<std::size_t>(bus)] = find_root(bus);
  }

  const Eigen::VectorXd stationarity_without_equalities =
      objective_scale * gradient +
      inequality_jacobian.transpose() * inequality_dual;
  diagnostics.baseline_raw = inf_norm(stationarity_without_equalities);
  diagnostics.baseline_normalized = diagnostics.baseline_raw /
      (1.0 + inf_norm(inequality_dual));

  std::vector<std::vector<double>> active_prices(
      static_cast<std::size_t>(buses));
  std::vector<std::vector<double>> reactive_prices(
      static_cast<std::size_t>(buses));
  std::vector<std::vector<double>> all_active_prices(
      static_cast<std::size_t>(buses));
  std::vector<std::vector<double>> all_reactive_prices(
      static_cast<std::size_t>(buses));
  for (int generator = 0; generator < prob.vidx.n_pg; ++generator) {
    const int bus = prob.gen_bus[static_cast<std::size_t>(generator)];
    const int root = parent[static_cast<std::size_t>(bus)];
    const int p_col = prob.vidx.i_pg + generator;
    const int q_col = prob.vidx.i_qg + generator;
    const double p_price = stationarity_without_equalities[p_col] /
                           std::max(std::abs(prob.scale_p), 1e-12);
    const double q_price = stationarity_without_equalities[q_col] /
                           std::max(std::abs(prob.scale_q), 1e-12);
    all_active_prices[static_cast<std::size_t>(root)].push_back(p_price);
    all_reactive_prices[static_cast<std::size_t>(root)].push_back(q_price);
    const auto is_interior = [&](int col) {
      const double width = xmax[col] - xmin[col];
      const double margin = 1e-5 * std::max(1.0, std::abs(width));
      return std::isfinite(width) && width > 2.0 * margin &&
             x[col] > xmin[col] + margin && x[col] < xmax[col] - margin;
    };
    if (is_interior(p_col)) {
      active_prices[static_cast<std::size_t>(root)].push_back(p_price);
    }
    if (is_interior(q_col)) {
      reactive_prices[static_cast<std::size_t>(root)].push_back(q_price);
    }
  }

  const auto median = [](std::vector<double>& values) {
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    return values[middle];
  };
  Eigen::VectorXd candidate = Eigen::VectorXd::Zero(equality_dual.size());
  std::vector<double> active_price_by_root(
      static_cast<std::size_t>(buses), 0.0);
  std::vector<double> reactive_price_by_root(
      static_cast<std::size_t>(buses), 0.0);
  std::vector<unsigned char> has_active_price(
      static_cast<std::size_t>(buses), 0);
  std::vector<unsigned char> has_reactive_price(
      static_cast<std::size_t>(buses), 0);
  int priced_components = 0;
  for (int root = 0; root < buses; ++root) {
    if (parent[static_cast<std::size_t>(root)] != root) continue;
    auto& p_values = active_prices[static_cast<std::size_t>(root)].empty()
        ? all_active_prices[static_cast<std::size_t>(root)]
        : active_prices[static_cast<std::size_t>(root)];
    auto& q_values = reactive_prices[static_cast<std::size_t>(root)].empty()
        ? all_reactive_prices[static_cast<std::size_t>(root)]
        : reactive_prices[static_cast<std::size_t>(root)];
    if (!p_values.empty()) {
      active_price_by_root[static_cast<std::size_t>(root)] = median(p_values);
      has_active_price[static_cast<std::size_t>(root)] = 1;
      ++priced_components;
    }
    if (!q_values.empty()) {
      reactive_price_by_root[static_cast<std::size_t>(root)] = median(q_values);
      has_reactive_price[static_cast<std::size_t>(root)] = 1;
    }
  }
  for (int bus = 0; bus < buses; ++bus) {
    const int root = parent[static_cast<std::size_t>(bus)];
    if (has_active_price[static_cast<std::size_t>(root)] != 0) {
      candidate[prob.cidx.i_pbal_ac + bus] =
          active_price_by_root[static_cast<std::size_t>(root)];
    }
    if (has_reactive_price[static_cast<std::size_t>(root)] != 0) {
      candidate[prob.cidx.i_qbal_ac + bus] =
          reactive_price_by_root[static_cast<std::size_t>(root)];
    }
  }
  if (priced_components == 0 || !candidate.allFinite()) {
    return finish("rejected: no finite component price");
  }

  const Eigen::VectorXd candidate_stationarity =
      stationarity_without_equalities + equality_jacobian.transpose() * candidate;
  diagnostics.candidate_raw = inf_norm(candidate_stationarity);
  diagnostics.candidate_normalized = diagnostics.candidate_raw /
      (1.0 + std::max(inf_norm(inequality_dual), inf_norm(candidate)));
  const double required = std::clamp(minimum_improvement, 0.0, 1.0);
  const bool raw_improves = std::isfinite(diagnostics.candidate_raw) &&
      diagnostics.candidate_raw <=
          (1.0 - required) * diagnostics.baseline_raw;
  const bool normalized_improves =
      std::isfinite(diagnostics.candidate_normalized) &&
      diagnostics.candidate_normalized <=
          (1.0 - required) * diagnostics.baseline_normalized;
  if (!raw_improves || !normalized_improves) {
    return finish("rejected: exact dual residual did not improve");
  }
  equality_dual = std::move(candidate);
  diagnostics.accepted = true;
  return finish("accepted: component dispatch-price predictor");
}

std::vector<int> parity_basic_columns(const Problem& prob) {
  const auto& v = prob.vidx;
  const int required = prob.cidx.n_eq_total;
  std::vector<int> columns;
  columns.reserve(static_cast<std::size_t>(required));
  std::vector<unsigned char> selected(
      static_cast<std::size_t>(v.n_total), 0);
  const auto add = [&](int col) {
    if (col >= 0 && col < v.n_total &&
        selected[static_cast<std::size_t>(col)] == 0 &&
        static_cast<int>(columns.size()) < required) {
      selected[static_cast<std::size_t>(col)] = 1;
      columns.push_back(col);
    }
  };
  const auto add_block = [&](int offset, int count) {
    for (int k = 0; k < count; ++k) add(offset + k);
  };

  // Nocedal--Wright (2006), Section 11.1: voltage coordinates are states;
  // device powers needed by coupling/reference equations complete the square
  // basic Jacobian. The remaining device powers are independent controls.
  add_block(v.i_va, v.n_va);
  std::vector<int> reactive_generator_by_bus(
      static_cast<std::size_t>(v.n_vm), -1);
  for (int k = 0; k < v.n_qg &&
                  k < static_cast<int>(prob.gen_bus.size()); ++k) {
    const int bus = prob.gen_bus[static_cast<std::size_t>(k)];
    if (bus >= 0 && bus < v.n_vm &&
        reactive_generator_by_bus[static_cast<std::size_t>(bus)] < 0) {
      reactive_generator_by_bus[static_cast<std::size_t>(bus)] = k;
    }
  }
  for (int bus = 0; bus < v.n_vm; ++bus) {
    const int generator =
        reactive_generator_by_bus[static_cast<std::size_t>(bus)];
    if (generator >= 0) add(v.i_qg + generator);
    else add(v.i_vm + bus);
  }
  add_block(v.i_vdc, v.n_vdc);
  add_block(v.i_pdc, v.n_pdc);
  add_block(v.i_pdcdc, v.n_pdcdc);
  for (int router : prob.er_router_to_data) {
    const auto found = std::find_if(
        prob.er_ports.begin(), prob.er_ports.end(),
        [router](const Problem::ERPortInfo& port) {
          return port.router_idx == router;
        });
    if (found != prob.er_ports.end()) add(v.i_erp + found->pvar);
  }
  for (int reference_bus : prob.ac_angle_reference_buses) {
    const auto found = std::find(prob.gen_bus.begin(), prob.gen_bus.end(),
                                 reference_bus);
    if (found != prob.gen_bus.end()) {
      add(v.i_pg + static_cast<int>(found - prob.gen_bus.begin()));
    }
  }
  for (int k = 0; k < prob.cidx.n_dc_ref; ++k) {
    if (k < v.n_pac) add(v.i_pac + k);
    else if (k < v.n_pstordc) add(v.i_pstordc + k);
  }

  // Complete unusual component mixes deterministically. Numerical
  // factorization remains the final validity check before this basis is used.
  for (const auto [offset, count] : std::array<std::pair<int, int>, 13>{
           std::pair{v.i_pg, v.n_pg}, {v.i_qg, v.n_qg},
           {v.i_pac, v.n_pac}, {v.i_qac, v.n_qac},
           {v.i_dpd, v.n_dpd}, {v.i_dqd, v.n_dqd},
           {v.i_pren, v.n_pren}, {v.i_qren, v.n_qren},
           {v.i_pstor, v.n_pstor}, {v.i_qstor, v.n_qstor},
           {v.i_pstordc, v.n_pstordc}, {v.i_pflex, v.n_pflex},
           {v.i_erp, v.n_erp}}) {
    add_block(offset, count);
  }
  if (static_cast<int>(columns.size()) != required) columns.clear();
  return columns;
}

Eigen::SparseMatrix<double> select_columns(
    const Eigen::SparseMatrix<double>& matrix,
    const std::vector<int>& columns) {
  std::vector<int> source_to_selected(
      static_cast<std::size_t>(matrix.cols()), -1);
  for (int local = 0; local < static_cast<int>(columns.size()); ++local) {
    source_to_selected[static_cast<std::size_t>(
        columns[static_cast<std::size_t>(local)])] = local;
  }
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<std::size_t>(matrix.nonZeros()));
  for (int col = 0; col < matrix.outerSize(); ++col) {
    const int selected_col =
        source_to_selected[static_cast<std::size_t>(col)];
    if (selected_col < 0) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, col); it;
         ++it) {
      trips.emplace_back(it.row(), selected_col, it.value());
    }
  }
  Eigen::SparseMatrix<double> selected(
      matrix.rows(), static_cast<int>(columns.size()));
  selected.setFromTriplets(trips.begin(), trips.end());
  selected.makeCompressed();
  return selected;
}

bool pivot_bounded_basic_column(const Problem& prob,
                                int limiting_col,
                                double direction,
                                const Eigen::VectorXd& x,
                                const Eigen::VectorXd& xmin,
                                const Eigen::VectorXd& xmax,
                                std::vector<int>& basic_columns) {
  const auto& v = prob.vidx;
  const std::array<std::pair<int, int>, 12> families{
      std::pair{v.i_pg, v.n_pg}, {v.i_qg, v.n_qg},
      {v.i_pac, v.n_pac}, {v.i_qac, v.n_qac},
      {v.i_pdc, v.n_pdc}, {v.i_pdcdc, v.n_pdcdc},
      {v.i_pren, v.n_pren}, {v.i_qren, v.n_qren},
      {v.i_pstor, v.n_pstor}, {v.i_qstor, v.n_qstor},
      {v.i_pstordc, v.n_pstordc}, {v.i_erp, v.n_erp}};
  const auto family = std::find_if(
      families.begin(), families.end(),
      [limiting_col](const auto& block) {
        return limiting_col >= block.first &&
               limiting_col < block.first + block.second;
      });
  if (family == families.end()) return false;

  const auto position = std::find(
      basic_columns.begin(), basic_columns.end(), limiting_col);
  if (position == basic_columns.end()) return false;
  std::vector<unsigned char> selected(
      static_cast<std::size_t>(v.n_total), 0);
  for (int col : basic_columns) {
    selected[static_cast<std::size_t>(col)] = 1;
  }
  int replacement = -1;
  double best_room = 0.0;
  for (int col = family->first; col < family->first + family->second; ++col) {
    if (selected[static_cast<std::size_t>(col)] != 0) continue;
    const double room = direction < 0.0
        ? x[col] - xmin[col] : xmax[col] - x[col];
    if (std::isfinite(room) && room > best_room) {
      best_room = room;
      replacement = col;
    }
  }
  if (replacement < 0) return false;
  *position = replacement;
  return true;
}

std::vector<int> phase_one_free_columns(
    const Problem& prob,
    const std::vector<unsigned char>& fixed_at_bound,
    int required_rows) {
  const auto& v = prob.vidx;
  std::vector<int> columns;
  std::vector<unsigned char> selected(
      static_cast<std::size_t>(v.n_total), 0);
  const auto add = [&](int col) {
    if (col >= 0 && col < v.n_total &&
        fixed_at_bound[static_cast<std::size_t>(col)] == 0 &&
        selected[static_cast<std::size_t>(col)] == 0) {
      selected[static_cast<std::size_t>(col)] = 1;
      columns.push_back(col);
    }
  };
  const auto add_block = [&](int offset, int count) {
    for (int k = 0; k < count; ++k) add(offset + k);
  };
  for (const auto [offset, count] : std::array<std::pair<int, int>, 16>{
           std::pair{v.i_va, v.n_va}, {v.i_vm, v.n_vm},
           {v.i_pg, v.n_pg}, {v.i_qg, v.n_qg},
           {v.i_vdc, v.n_vdc}, {v.i_pac, v.n_pac},
           {v.i_qac, v.n_qac}, {v.i_pdc, v.n_pdc},
           {v.i_pstor, v.n_pstor}, {v.i_qstor, v.n_qstor},
           {v.i_pstordc, v.n_pstordc}, {v.i_pdcdc, v.n_pdcdc},
           {v.i_erp, v.n_erp}, {v.i_erq, v.n_erq},
           {v.i_pren, v.n_pren}, {v.i_qren, v.n_qren}}) {
    add_block(offset, count);
  }
  if (static_cast<int>(columns.size()) < required_rows) {
    add_block(v.i_pflex, v.n_pflex);
    add_block(v.i_dpd, v.n_dpd);
    add_block(v.i_dqd, v.n_dqd);
  }
  return columns;
}

double phase_one_violation(const Problem& prob,
                           const Eigen::VectorXd& x,
                           const Eigen::VectorXd& xmin,
                           const Eigen::VectorXd& xmax,
                           const std::vector<int>& lb_cols,
                           const std::vector<int>& ub_cols) {
  EvalWorkspace ws;
  Eigen::VectorXd g;
  Eigen::VectorXd h;
  equality_constraints(prob, x, ws, g);
  nonlinear_inequality_constraints(prob, x, h);
  if (!g.allFinite() || !h.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  const double equality_violation = inf_norm(g);
  double inequality_violation = h.size() > 0
      ? std::max(0.0, h.maxCoeff()) : 0.0;
  for (int col : lb_cols) {
    inequality_violation = std::max(
        inequality_violation, xmin[col] - x[col]);
  }
  for (int col : ub_cols) {
    inequality_violation = std::max(
        inequality_violation, x[col] - xmax[col]);
  }
  return std::max(equality_violation, inequality_violation);
}

struct HybridSchurRepair {
  bool attempted{false};
  bool accepted{false};
  int factorizations{0};
  int backtracks{0};
  double global_violation{std::numeric_limits<double>::infinity()};
  double block_violation{std::numeric_limits<double>::infinity()};
  std::string status{"not-applicable"};
};

double hybrid_dc_converter_violation(const Problem& prob,
                                     const Eigen::VectorXd& x) {
  EvalWorkspace ws;
  Eigen::VectorXd equality;
  equality_constraints(prob, x, ws, equality);
  if (!equality.allFinite()) return std::numeric_limits<double>::infinity();
  const auto& c = prob.cidx;
  double value = 0.0;
  const auto accumulate = [&](int offset, int count) {
    for (int row = 0; row < count; ++row)
      value = std::max(value, std::abs(equality[offset + row]));
  };
  accumulate(c.i_pbal_dc, c.n_pbal_dc);
  accumulate(c.i_conv_bal, c.n_conv_bal);
  accumulate(c.i_dc_ref, c.n_dc_ref);
  return value;
}

HybridSchurRepair repair_hybrid_dc_converter_block(
    const Problem& prob,
    Eigen::VectorXd& x,
    const Eigen::VectorXd& xmin,
    const Eigen::VectorXd& xmax,
    const std::vector<int>& lb_cols,
    const std::vector<int>& ub_cols,
    const IPMOptions& opt,
    double initial_violation) {
  HybridSchurRepair repair;
  const auto& v = prob.vidx;
  const auto& c = prob.cidx;
  if (v.n_vdc <= 0 || v.n_pdc <= 0 || v.n_pac != v.n_pdc ||
      c.n_pbal_dc != v.n_vdc || c.n_conv_bal != v.n_pdc) {
    return repair;
  }
  repair.attempted = true;
  const double initial_structural_violation =
      hybrid_dc_converter_violation(prob, x);

  EvalWorkspace ws;
  Eigen::VectorXd equality;
  Eigen::SparseMatrix<double> jacobian;
  equality_constraints(prob, x, ws, equality);
  equality_jacobian(prob, x, ws, jacobian);
  const int rows = c.n_pbal_dc + c.n_dc_ref;
  const int cols = v.n_vdc + v.n_pac;
  if (!equality.allFinite() || rows <= 0 || cols < rows) {
    repair.status = "hybrid-schur-invalid-shape";
    return repair;
  }

  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<std::size_t>(
      prob.data.gdc.nonZeros() + c.n_dc_ref + 2 * v.n_pac));
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(rows);
  for (int local_row = 0; local_row < c.n_pbal_dc; ++local_row) {
    const int source_row = c.i_pbal_dc + local_row;
    rhs[local_row] = -equality[source_row];
    for (int dc = 0; dc < v.n_vdc; ++dc) {
      const double value = jacobian.coeff(source_row, v.i_vdc + dc);
      if (value != 0.0) trips.emplace_back(local_row, dc, value);
    }
    for (int k = 0; k < v.n_pdc; ++k) {
      const double j_dc_pdc = jacobian.coeff(source_row, v.i_pdc + k);
      if (j_dc_pdc == 0.0) continue;
      const int conv_row = c.i_conv_bal + k;
      const double j_conv_pdc = jacobian.coeff(conv_row, v.i_pdc + k);
      const double j_conv_pac = jacobian.coeff(conv_row, v.i_pac + k);
      if (std::abs(j_conv_pdc) <= 1e-12) {
        repair.status = "hybrid-schur-singular-converter-block";
        return repair;
      }
      // Exact block Gaussian elimination:
      // dpdc = -(gconv + Jpac*dpac)/Jpdc, substituted into the DC rows.
      // See Golub--Van Loan (2013), Section 3.2.
      rhs[local_row] +=
          j_dc_pdc * equality[conv_row] / j_conv_pdc;
      const double schur = -j_dc_pdc * j_conv_pac / j_conv_pdc;
      if (schur != 0.0)
        trips.emplace_back(local_row, v.n_vdc + k, schur);
    }
  }
  for (int local = 0; local < c.n_dc_ref; ++local) {
    const int row = c.n_pbal_dc + local;
    const int source_row = c.i_dc_ref + local;
    rhs[row] = -equality[source_row];
    for (int dc = 0; dc < v.n_vdc; ++dc) {
      const double value = jacobian.coeff(source_row, v.i_vdc + dc);
      if (value != 0.0) trips.emplace_back(row, dc, value);
    }
  }

  Eigen::SparseMatrix<double> schur(rows, cols);
  schur.setFromTriplets(trips.begin(), trips.end());
  schur.makeCompressed();
  Eigen::VectorXd row_scale = Eigen::VectorXd::Ones(rows);
  for (int col = 0; col < schur.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(schur, col); it; ++it)
      row_scale[it.row()] = std::max(row_scale[it.row()], std::abs(it.value()));
  }
  row_scale = row_scale.cwiseInverse();
  for (int col = 0; col < schur.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(schur, col); it; ++it)
      it.valueRef() *= row_scale[it.row()];
  }
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
  qr.compute(schur);
  ++repair.factorizations;
  if (qr.info() != Eigen::Success) {
    repair.status = "hybrid-schur-factorization-failed";
    return repair;
  }
  const Eigen::VectorXd reduced_step = qr.solve(row_scale.cwiseProduct(rhs));
  if (qr.info() != Eigen::Success || !reduced_step.allFinite()) {
    repair.status = "hybrid-schur-solve-failed";
    return repair;
  }
  Eigen::VectorXd step = Eigen::VectorXd::Zero(x.size());
  for (int dc = 0; dc < v.n_vdc; ++dc)
    step[v.i_vdc + dc] = reduced_step[dc];
  for (int k = 0; k < v.n_pac; ++k) {
    step[v.i_pac + k] = reduced_step[v.n_vdc + k];
    const int conv_row = c.i_conv_bal + k;
    const double j_pdc = jacobian.coeff(conv_row, v.i_pdc + k);
    const double j_pac = jacobian.coeff(conv_row, v.i_pac + k);
    step[v.i_pdc + k] =
        -(equality[conv_row] + j_pac * step[v.i_pac + k]) / j_pdc;
  }

  double alpha = 1.0;
  for (int col = 0; col < step.size(); ++col) {
    if (step[col] < 0.0 && std::isfinite(xmin[col]))
      alpha = std::min(alpha, 0.995 * (xmin[col] - x[col]) / step[col]);
    else if (step[col] > 0.0 && std::isfinite(xmax[col]))
      alpha = std::min(alpha, 0.995 * (xmax[col] - x[col]) / step[col]);
  }
  alpha = std::clamp(alpha, 0.0, 1.0);
  for (int attempt = 0;
       attempt <= std::max(0, opt.phase_one_max_backtracks); ++attempt) {
    Eigen::VectorXd trial = x + alpha * step;
    const double violation = phase_one_violation(
        prob, trial, xmin, xmax, lb_cols, ub_cols);
    const double structural_violation =
        hybrid_dc_converter_violation(prob, trial);
    // Block-coordinate Newton must decrease the block it actually solves;
    // an unrelated AC row may continue to define the global infinity norm.
    // Requiring the global norm not to increase preserves monotonicity before
    // the following AC state/basic Newton step (Nocedal--Wright, 2006, §11.1).
    if (std::isfinite(violation) &&
        structural_violation < initial_structural_violation &&
        violation <= initial_violation) {
      x = std::move(trial);
      repair.accepted = true;
      repair.global_violation = violation;
      repair.block_violation = structural_violation;
      repair.status = "hybrid-dc-converter-schur";
      return repair;
    }
    if (attempt < std::max(0, opt.phase_one_max_backtracks)) {
      ++repair.backtracks;
      alpha *= 0.5;
    }
  }
  repair.global_violation = initial_violation;
  repair.block_violation = initial_structural_violation;
  repair.status = "hybrid-schur-no-decreasing-step";
  return repair;
}

PhaseOneState restore_phase_one_primal(
    const Problem& prob,
    Eigen::VectorXd& x,
    const Eigen::VectorXd& xmin,
    const Eigen::VectorXd& xmax,
    const std::vector<int>& lb_cols,
    const std::vector<int>& ub_cols,
    const IPMOptions& opt) {
  PhaseOneState state;
  state.barrier_mu = std::clamp(
      std::isfinite(opt.phase_one_barrier_mu) &&
              opt.phase_one_barrier_mu > 0.0
          ? opt.phase_one_barrier_mu
          : 0.1,
      1e-12, 0.1);
  const double primal_mu_factor =
      std::isfinite(opt.phase_one_primal_mu_factor)
      ? std::max(0.0, opt.phase_one_primal_mu_factor) : 0.1;
  // Phase I only has to enter the central neighborhood from which Phase II's
  // filter IPM is effective; final feasibility remains a Phase-II contract.
  // See Waechter--Biegler (2006), Sections 2--3.
  state.handoff_primal_tolerance = std::max(
      opt.tol_primal, primal_mu_factor * state.barrier_mu);
  const auto started = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
  };
  double violation = phase_one_violation(
      prob, x, xmin, xmax, lb_cols, ub_cols);
  state.initial_violation = violation;
  state.final_violation = violation;
  if (prob.vidx.n_vdc > 0 &&
      state.factorizations < std::max(0, opt.phase_one_max_factorizations)) {
    const HybridSchurRepair repair = repair_hybrid_dc_converter_block(
        prob, x, xmin, xmax, lb_cols, ub_cols, opt, violation);
    state.structural_step_attempted = repair.attempted;
    state.structural_step_accepted = repair.accepted;
    state.structural_factorizations = repair.factorizations;
    state.factorizations += repair.factorizations;
    state.backtracks += repair.backtracks;
    state.structural_violation = repair.block_violation;
    if (repair.attempted) state.structure = repair.status;
    if (repair.accepted) violation = repair.global_violation;
    state.final_violation = violation;
  }
  const double admission_mu_factor =
      std::isfinite(opt.phase_one_admission_mu_factor)
      ? std::max(0.0, opt.phase_one_admission_mu_factor) : 1.0;
  if (!std::isfinite(violation) ||
      violation > admission_mu_factor * state.barrier_mu) {
    state.termination = "outside-phase-one-admission";
    state.rejection_reason = "raw primal violation exceeds Phase-I admission";
    state.runtime_ms = elapsed_ms();
    return state;
  }
  Eigen::VectorXd best_x = x;
  state.basic_columns = parity_basic_columns(prob);
  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
      basis_solver;
  bool basis_pattern_analyzed = false;
  std::vector<unsigned char> fixed_at_bound(
      static_cast<std::size_t>(x.size()), 0);
  for (int col = 0; col < x.size(); ++col) {
    if (std::isfinite(xmin[col]) && std::isfinite(xmax[col]) &&
        xmax[col] - xmin[col] <= 2.0 * opt.tol_primal) {
      fixed_at_bound[static_cast<std::size_t>(col)] = 1;
    }
  }

  for (; state.iterations < std::max(0, opt.phase_one_max_iterations) &&
         violation > state.handoff_primal_tolerance;) {
    if (state.factorizations >=
        std::max(0, opt.phase_one_max_factorizations)) {
      state.budget_exhausted = true;
      state.termination = "factorization-budget";
      break;
    }
    if (opt.phase_one_time_limit_ms > 0.0 &&
        elapsed_ms() >= opt.phase_one_time_limit_ms) {
      state.budget_exhausted = true;
      state.termination = "time-budget";
      break;
    }

    EvalWorkspace ws;
    Eigen::VectorXd equality;
    Eigen::SparseMatrix<double> jacobian;
    equality_constraints(prob, x, ws, equality);
    equality_jacobian(prob, x, ws, jacobian);
    if (!equality.allFinite() || jacobian.rows() != equality.size() ||
        jacobian.cols() != x.size()) {
      state.termination = "invalid-jacobian";
      break;
    }

    Eigen::VectorXd inequality;
    Eigen::SparseMatrix<double> inequality_jacobian;
    assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols,
                          inequality, inequality_jacobian);
    std::vector<int> violated_rows;
    for (int row = 0; row < inequality.size(); ++row) {
      if (inequality[row] > state.handoff_primal_tolerance) {
        violated_rows.push_back(row);
      }
    }
    Eigen::VectorXd restoration_residual(
        equality.size() + static_cast<int>(violated_rows.size()));
    restoration_residual.head(equality.size()) = equality;
    std::vector<Eigen::Triplet<double>> restoration_trips;
    restoration_trips.reserve(static_cast<std::size_t>(
        jacobian.nonZeros() + inequality_jacobian.nonZeros()));
    for (int col = 0; col < jacobian.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(jacobian, col); it;
           ++it) {
        restoration_trips.emplace_back(it.row(), it.col(), it.value());
      }
    }
    std::vector<int> inequality_to_active(
        static_cast<std::size_t>(inequality.size()), -1);
    for (int local = 0; local < static_cast<int>(violated_rows.size());
         ++local) {
      const int source = violated_rows[static_cast<std::size_t>(local)];
      inequality_to_active[static_cast<std::size_t>(source)] = local;
      restoration_residual[equality.size() + local] = inequality[source];
    }
    for (int col = 0; col < inequality_jacobian.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               inequality_jacobian, col); it; ++it) {
        const int local = inequality_to_active[
            static_cast<std::size_t>(it.row())];
        if (local >= 0) {
          restoration_trips.emplace_back(
              equality.size() + local, it.col(), it.value());
        }
      }
    }
    Eigen::SparseMatrix<double> restoration_jacobian(
        restoration_residual.size(), x.size());
    restoration_jacobian.setFromTriplets(
        restoration_trips.begin(), restoration_trips.end());
    restoration_jacobian.makeCompressed();

    Eigen::VectorXd row_scale =
        Eigen::VectorXd::Ones(restoration_residual.size());
    for (int col = 0; col < restoration_jacobian.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               restoration_jacobian, col); it;
           ++it) {
        row_scale[it.row()] =
            std::max(row_scale[it.row()], std::abs(it.value()));
      }
    }
    row_scale = row_scale.cwiseInverse();
    for (int col = 0; col < restoration_jacobian.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               restoration_jacobian, col); it;
           ++it) {
        it.valueRef() *= row_scale[it.row()];
      }
    }

    Eigen::VectorXd step = Eigen::VectorXd::Zero(x.size());
    bool solved_step = false;
    if (violated_rows.empty() && !state.basic_columns.empty()) {
      const Eigen::SparseMatrix<double> basis =
          select_columns(restoration_jacobian, state.basic_columns);
      if (!basis_pattern_analyzed) {
        basis_solver.analyzePattern(basis);
        basis_pattern_analyzed = true;
      }
      basis_solver.factorize(basis);
      ++state.factorizations;
      if (basis_solver.info() == Eigen::Success) {
        const Eigen::VectorXd basic_step =
            basis_solver.solve(-row_scale.cwiseProduct(restoration_residual));
        if (basis_solver.info() == Eigen::Success && basic_step.allFinite()) {
          for (int local = 0;
               local < static_cast<int>(state.basic_columns.size()); ++local) {
            step[state.basic_columns[static_cast<std::size_t>(local)]] =
                basic_step[local];
          }
          solved_step = true;
          state.linear_solver = "sparse-basis-lu";
        }
      }
    }
    if (!solved_step) {
      if (state.factorizations >=
          std::max(0, opt.phase_one_max_factorizations)) {
        state.budget_exhausted = true;
        state.termination = "factorization-budget";
        break;
      }
      // Nocedal--Wright (2006), Section 11.1: violated h_i <= 0 rows join the
      // constraint-Newton residual. Direct SparseQR avoids dense materialization
      // and the squared conditioning of J*J' normal equations.
      const std::vector<int> free_columns = phase_one_free_columns(
          prob, fixed_at_bound, restoration_jacobian.rows());
      if (free_columns.empty()) {
        state.termination = "all-columns-bound";
        break;
      }
      const Eigen::SparseMatrix<double> free_jacobian =
          select_columns(restoration_jacobian, free_columns);
      Eigen::SparseMatrix<double> scaled_free_jacobian = free_jacobian;
      Eigen::VectorXd column_scale = Eigen::VectorXd::Ones(free_columns.size());
      for (int local = 0; local < static_cast<int>(free_columns.size());
           ++local) {
        const int source = free_columns[static_cast<std::size_t>(local)];
        if (std::isfinite(xmin[source]) && std::isfinite(xmax[source])) {
          column_scale[local] =
              std::max(1e-12, xmax[source] - xmin[source]);
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 scaled_free_jacobian, local); it; ++it) {
          it.valueRef() *= column_scale[local];
        }
      }
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
      // Nocedal--Wright (2006), Section 7.5: solve in range-scaled variable
      // coordinates so a unit step means a comparable fraction of each
      // variable's available interval.
      qr.compute(scaled_free_jacobian);
      ++state.factorizations;
      if (qr.info() == Eigen::Success) {
        const Eigen::VectorXd free_step =
            qr.solve(-row_scale.cwiseProduct(restoration_residual));
        solved_step = qr.info() == Eigen::Success && free_step.allFinite();
        if (solved_step) {
          for (int local = 0; local < static_cast<int>(free_columns.size());
               ++local) {
            step[free_columns[static_cast<std::size_t>(local)]] =
                column_scale[local] * free_step[local];
          }
        }
      }
      if (solved_step) state.linear_solver = "sparse-qr";
    }
    ++state.iterations;
    if (!solved_step) {
      state.termination = "linear-solve-failed";
      break;
    }

    bool accepted = false;
    double alpha = 1.0;
    int limiting_col = -1;
    for (int col = 0; col < step.size(); ++col) {
      if (step[col] < 0.0 && std::isfinite(xmin[col])) {
        const double candidate =
            0.995 * (xmin[col] - x[col]) / step[col];
        if (candidate < alpha) {
          alpha = candidate;
          limiting_col = col;
        }
      } else if (step[col] > 0.0 && std::isfinite(xmax[col])) {
        const double candidate =
            0.995 * (xmax[col] - x[col]) / step[col];
        if (candidate < alpha) {
          alpha = candidate;
          limiting_col = col;
        }
      }
    }
    alpha = std::clamp(alpha, 0.0, 1.0);
    if (opt.verbose && limiting_col >= 0) {
      std::fprintf(stderr,
                   "parity Phase I bound step: iteration=%d alpha=%.12g "
                   "column=%d x=%.12g step=%.12g bounds=[%.12g,%.12g]\n",
                   state.iterations, alpha, limiting_col, x[limiting_col],
                   step[limiting_col], xmin[limiting_col],
                   xmax[limiting_col]);
    }
    const int backtrack_limit = std::max(0, opt.phase_one_max_backtracks);
    for (int attempt = 0; attempt <= backtrack_limit; ++attempt) {
      Eigen::VectorXd trial = x + alpha * step;
      const double trial_violation = phase_one_violation(
          prob, trial, xmin, xmax, lb_cols, ub_cols);
      // Deuflhard (2011), Sections 2.2--2.3: accept only strict monotone
      // improvement; no filter, barrier schedule, or second-order correction.
      if (std::isfinite(trial_violation) && trial_violation < violation) {
        x = std::move(trial);
        violation = trial_violation;
        best_x = x;
        state.final_violation = violation;
        accepted = true;
        // Nocedal--Wright (2006), Section 16.5: once a basic variable reaches
        // a bound, the fixed basis no longer represents the local free
        // manifold. Fall back to the all-column constraint Newton solve on the
        // next iteration instead of taking vanishing steps against that bound.
        if (limiting_col >= 0 && !state.basic_columns.empty()) {
          const bool pivoted = pivot_bounded_basic_column(
              prob, limiting_col, step[limiting_col], x, xmin, xmax,
              state.basic_columns);
          if (pivoted) {
            basis_pattern_analyzed = false;
          } else {
            state.basic_columns.clear();
          }
        }
        if (limiting_col >= 0) {
          fixed_at_bound[static_cast<std::size_t>(limiting_col)] = 1;
        }
        break;
      }
      if (attempt < backtrack_limit) {
        ++state.backtracks;
        alpha *= 0.5;
      }
    }
    if (!accepted) {
      state.termination = "no-decreasing-step";
      break;
    }
  }

  x = std::move(best_x);
  state.final_violation = phase_one_violation(
      prob, x, xmin, xmax, lb_cols, ub_cols);
  if (opt.verbose) {
    EvalWorkspace final_ws;
    Eigen::VectorXd final_g;
    Eigen::VectorXd final_h;
    Eigen::SparseMatrix<double> final_jh;
    equality_constraints(prob, x, final_ws, final_g);
    assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols,
                          final_h, final_jh);
    Eigen::Index worst_g = 0;
    Eigen::Index worst_h = 0;
    const double g_inf = final_g.size() > 0
        ? final_g.cwiseAbs().maxCoeff(&worst_g) : 0.0;
    const double h_inf = final_h.size() > 0
        ? std::max(0.0, final_h.maxCoeff(&worst_h)) : 0.0;
    std::fprintf(stderr,
                 "parity Phase I final: g_inf=%.12g row=%td "
                 "h_plus=%.12g row=%td\n",
                 g_inf, worst_g, h_inf, worst_h);
  }
  state.in_handoff_corridor = std::isfinite(state.final_violation) &&
      state.final_violation <= state.handoff_primal_tolerance;
  state.primal_feasible = std::isfinite(state.final_violation) &&
      state.final_violation <= opt.tol_primal;
  if (state.in_handoff_corridor) {
    state.termination = "corridor-certified";
  } else if (state.termination == "not-run") {
    state.budget_exhausted =
        state.iterations >= std::max(0, opt.phase_one_max_iterations);
    state.termination = state.budget_exhausted
        ? "iteration-budget" : "incomplete";
  }
  state.runtime_ms = elapsed_ms();
  return state;
}

bool initialize_phase_one_duals(
    const Problem& prob,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& rh,
    const Eigen::SparseMatrix<double>& dh,
    const IPMOptions& opt,
    PhaseOneState& state,
    Eigen::VectorXd& z,
    Eigen::VectorXd& mu,
    Eigen::VectorXd& lambda) {
  if (!state.in_handoff_corridor ||
      state.factorizations >= std::max(0, opt.phase_one_max_factorizations)) {
    if (state.in_handoff_corridor) {
      state.budget_exhausted = true;
      state.termination = "factorization-budget-after-primal";
    }
    if (!state.in_handoff_corridor) {
      state.rejection_reason = "raw primal violation exceeds corridor";
    }
    return false;
  }

  // Spend half of the remaining primal corridor on a positive slack floor:
  // if v=max(0,max h_i)<=epsilon_p and
  // s_floor=(epsilon_p-v)/2, then h_i+s_i <= (epsilon_p+v)/2<=epsilon_p.
  // This avoids the 1/s blow-up produced by a representability-only floor
  // while preserving z_i*mu_i=mu_0 without another factorization.
  const double max_positive_inequality = rh.size() > 0
      ? std::max(0.0, rh.maxCoeff()) : 0.0;
  const double central_slack_floor = std::max(
      2e-10, 0.5 * (state.handoff_primal_tolerance -
                     max_positive_inequality));
  z = (-rh.array()).max(central_slack_floor).matrix();
  mu = (state.barrier_mu / z.array()).matrix();
  Eigen::VectorXd gradient;
  Eigen::VectorXd hdiag;
  objective_gradient_hessian_diag(prob, x, gradient, hdiag);
  const double gradient_norm = inf_norm(gradient);
  const double objective_scale = gradient_norm > 100.0
      ? 100.0 / gradient_norm : 1.0;

  EvalWorkspace ws;
  Eigen::VectorXd equality;
  Eigen::SparseMatrix<double> jg;
  equality_constraints(prob, x, ws, equality);
  equality_jacobian(prob, x, ws, jg);
  const Eigen::VectorXd stationarity_without_equalities =
      objective_scale * gradient + dh.transpose() * mu;

  bool solved_dual = false;
  if (!state.basic_columns.empty()) {
    const Eigen::SparseMatrix<double> basis =
        select_columns(jg, state.basic_columns);
    Eigen::SparseLU<Eigen::SparseMatrix<double>,
                    Eigen::COLAMDOrdering<int>> lu;
    lu.compute(Eigen::SparseMatrix<double>(basis.transpose()));
    ++state.factorizations;
    if (lu.info() == Eigen::Success) {
      Eigen::VectorXd rhs(state.basic_columns.size());
      for (int local = 0;
           local < static_cast<int>(state.basic_columns.size()); ++local) {
        rhs[local] = -stationarity_without_equalities[
            state.basic_columns[static_cast<std::size_t>(local)]];
      }
      lambda = lu.solve(rhs);
      solved_dual = lu.info() == Eigen::Success && lambda.allFinite();
    }
  }
  if (!solved_dual) {
    if (state.factorizations >=
        std::max(0, opt.phase_one_max_factorizations)) {
      state.budget_exhausted = true;
      state.termination = "factorization-budget-after-primal";
      return false;
    }
    // Nocedal--Wright (2006), Section 11.1: structural fallback fits equality
    // multipliers once by least squares on Jg' lambda = -r.
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
    qr.compute(jg.transpose());
    ++state.factorizations;
    if (qr.info() != Eigen::Success) return false;
    lambda = qr.solve(-stationarity_without_equalities);
    solved_dual = qr.info() == Eigen::Success && lambda.allFinite();
  }
  if (!solved_dual) return false;
  const Eigen::VectorXd stationarity =
      stationarity_without_equalities + jg.transpose() * lambda;
  const double multiplier_scale = 1.0 + std::max(
      inf_norm(lambda), inf_norm(mu));
  if (!state.basic_columns.empty()) {
    double basic_residual = 0.0;
    for (int col : state.basic_columns) {
      basic_residual = std::max(basic_residual, std::abs(stationarity[col]));
    }
    state.dual_fit_residual = basic_residual / multiplier_scale;
  } else {
    state.dual_fit_residual = inf_norm(jg * stationarity) / multiplier_scale;
  }
  state.dual_initialized = std::isfinite(state.dual_fit_residual) &&
      state.dual_fit_residual <= std::max(1e-6, 10.0 * opt.tol_dual) &&
      (z.array() > 0.0).all() && (mu.array() > 0.0).all();
  state.perturbed_primal_residual = std::max(
      inf_norm(equality), inf_norm(rh + z));
  state.centrality = z.size() == 0 ? 0.0 :
      (z.cwiseProduct(mu).array() / state.barrier_mu - 1.0)
          .abs().maxCoeff();
  const double centrality_tolerance =
      std::isfinite(opt.phase_one_centrality_tolerance)
      ? std::max(0.0, opt.phase_one_centrality_tolerance) : 0.5;
  state.central_start_accepted = state.dual_initialized &&
      std::isfinite(state.perturbed_primal_residual) &&
      state.perturbed_primal_residual <= state.handoff_primal_tolerance &&
      std::isfinite(state.centrality) &&
      state.centrality <= centrality_tolerance;
  if (!state.dual_initialized) {
    state.rejection_reason = "dual initialization failed";
  } else if (state.perturbed_primal_residual >
             state.handoff_primal_tolerance) {
    state.rejection_reason = "perturbed primal residual exceeds corridor";
  } else if (state.centrality > centrality_tolerance) {
    state.rejection_reason = "complementarity centrality exceeds tolerance";
  }
  if (state.central_start_accepted) {
    state.termination = "central-corridor-certified";
  }
  if (state.dual_initialized && state.linear_solver == "unselected") {
    state.linear_solver = "sparse-qr";
  }
  return state.central_start_accepted;
}

}  // namespace

struct IPMPreparedState::Impl {
  SparseKKTCache sparse_cache;
};

IPMPreparedState::IPMPreparedState() : impl_(std::make_unique<Impl>()) {}
IPMPreparedState::~IPMPreparedState() = default;
IPMPreparedState::IPMPreparedState(IPMPreparedState&&) noexcept = default;
IPMPreparedState& IPMPreparedState::operator=(IPMPreparedState&&) noexcept =
    default;
void IPMPreparedState::reset() { impl_ = std::make_unique<Impl>(); }

IPMInitialPointDiagnostics evaluate_ipm_initial_point(
    const Problem& prob, const Eigen::VectorXd& primal_start) {
  IPMInitialPointDiagnostics diagnostics;
  const int n = prob.vidx.n_total;
  if (primal_start.size() != n || !primal_start.allFinite()) {
    return diagnostics;
  }
  Eigen::VectorXd xmin;
  Eigen::VectorXd xmax;
  build_variable_bounds(prob, xmin, xmax);
  Eigen::VectorXd x = primal_start;
  for (int i = 0; i < n; ++i) {
    if (std::isfinite(xmin[i]) && std::isfinite(xmax[i])) {
      const double width = std::max(0.0, xmax[i] - xmin[i]);
      const double eps =
          std::min(1e-6 * std::max(1.0, width), 0.49 * width);
      x[i] = std::clamp(x[i], xmin[i] + eps, xmax[i] - eps);
    } else if (std::isfinite(xmin[i])) {
      x[i] = std::max(x[i], xmin[i] + 1e-8);
    } else if (std::isfinite(xmax[i])) {
      x[i] = std::min(x[i], xmax[i] - 1e-8);
    }
  }
  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  for (int i = 0; i < n; ++i) {
    if (std::isfinite(xmin[i])) lb_cols.push_back(i);
    if (std::isfinite(xmax[i])) ub_cols.push_back(i);
  }
  Eigen::VectorXd rh;
  Eigen::SparseMatrix<double> dh;
  assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols, rh, dh);
  if (!rh.allFinite()) return diagnostics;
  Eigen::VectorXd z(rh.size());
  Eigen::VectorXd mu(rh.size());
  for (int i = 0; i < rh.size(); ++i) {
    z[i] = rh[i] >= 0.0 ? std::max(rh[i], 1.0)
                        : std::max(-rh[i], 1e-2);
    mu[i] = std::max(1.0 / z[i], 1e-2);
  }
  Eigen::VectorXd gradient;
  Eigen::VectorXd hdiag;
  objective_gradient_hessian_diag(prob, x, gradient, hdiag);
  const double gradient_norm = inf_norm(gradient);
  const double objective_scale =
      gradient_norm > 100.0 ? 100.0 / gradient_norm : 1.0;
  EvalWorkspace workspace;
  Eigen::VectorXd equality;
  equality_constraints(prob, x, workspace, equality);
  const Eigen::VectorXd stationarity =
      objective_scale * gradient + dh.transpose() * mu;
  const double raw_primal = std::max(
      inf_norm(equality), rh.size() > 0 ? std::max(0.0, rh.maxCoeff()) : 0.0);
  diagnostics.primal_inf = raw_primal /
      (1.0 + std::max(inf_norm(x), inf_norm(z)));
  diagnostics.dual_inf = inf_norm(stationarity) /
      (1.0 + inf_norm(mu));
  diagnostics.complementarity = rh.size() > 0
      ? z.dot(mu) / static_cast<double>(rh.size()) /
            (1.0 + inf_norm(x))
      : 0.0;
  diagnostics.valid = std::isfinite(diagnostics.primal_inf) &&
      std::isfinite(diagnostics.dual_inf) &&
      std::isfinite(diagnostics.complementarity);
  return diagnostics;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Mehrotra Predictor-Corrector IPM
// ═══════════════════════════════════════════════════════════════════════════════

IPMResult solve_primal_dual_ipm(const Problem& prob, const IPMOptions& opt) {
  const int n = prob.vidx.n_total;
  const int meq = prob.cidx.n_eq_total;
  if (n <= 0 || meq < 0) {
    throw std::runtime_error("solve_primal_dual_ipm: invalid problem size.");
  }

  Eigen::VectorXd xmin;
  Eigen::VectorXd xmax;
  Eigen::VectorXd x;
  build_variable_bounds(prob, xmin, xmax);
  bool warm_start_used = false;
  if (opt.primal_start != nullptr && opt.primal_start->size() == n &&
      opt.primal_start->allFinite()) {
    x = *opt.primal_start;
    // Keep a very small interior margin.  A previous optimum often lies on an
    // active bound; moving it by 1% would discard most warm-start benefit.
    for (int i = 0; i < n; ++i) {
      const double lo = xmin[i];
      const double hi = xmax[i];
      if (std::isfinite(lo) && std::isfinite(hi)) {
        const double width = std::max(0.0, hi - lo);
        const double eps = std::min(1e-6 * std::max(1.0, width), 0.49 * width);
        x[i] = std::clamp(x[i], lo + eps, hi - eps);
      } else if (std::isfinite(lo)) {
        x[i] = std::max(x[i], lo + 1e-8);
      } else if (std::isfinite(hi)) {
        x[i] = std::min(x[i], hi - 1e-8);
      }
    }
    warm_start_used = true;
  } else {
    build_initial_point(prob, xmin, xmax, x);
  }

  // Identify finite-bound variable indices
  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  lb_cols.reserve(static_cast<size_t>(n));
  ub_cols.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    if (std::isfinite(xmin[i])) {
      lb_cols.push_back(i);
    }
    if (std::isfinite(xmax[i])) {
      ub_cols.push_back(i);
    }
  }

  // Evaluate initial inequality constraints
  Eigen::VectorXd rh;
  Eigen::SparseMatrix<double> dh;
  assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols, rh, dh);
  const int niq = static_cast<int>(rh.size());
  if (niq <= 0) {
    throw std::runtime_error("solve_primal_dual_ipm: no inequality constraints assembled.");
  }

  const bool compatible_dual_start =
      warm_start_used && opt.equality_dual_start != nullptr &&
      opt.inequality_dual_start != nullptr && opt.slack_start != nullptr &&
      opt.equality_dual_start->size() == meq &&
      opt.inequality_dual_start->size() == niq &&
      opt.slack_start->size() == niq &&
      opt.equality_dual_start->allFinite() &&
      opt.inequality_dual_start->allFinite() && opt.slack_start->allFinite() &&
      opt.inequality_dual_start->minCoeff() > 0.0 &&
      opt.slack_start->minCoeff() > 0.0;

  const Eigen::VectorXd phase_two_fallback_x = x;
  PhaseOneState phase_one;
  const auto phase_one_started = std::chrono::steady_clock::now();
  if (opt.enable_phase_one && !compatible_dual_start) {
    phase_one = restore_phase_one_primal(
        prob, x, xmin, xmax, lb_cols, ub_cols, opt);
    assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols, rh, dh);
  } else if (compatible_dual_start) {
    phase_one.termination = "continuation-state-preserved";
  } else {
    phase_one.termination = "disabled";
  }

  // Slack initialization: use a larger minimum floor (1.0) to ensure a
  // well-conditioned condensed KKT from a cold start.  A compatible warm
  // start instead preserves the previous central-path state.
  Eigen::VectorXd z(niq);
  if (compatible_dual_start) {
    z = *opt.slack_start;
  } else {
    for (int i = 0; i < niq; ++i) {
      z[i] = (rh[i] >= 0.0) ? std::max(rh[i], 1.0) : std::max(-rh[i], 1e-2);
    }
  }

  // Multiplier initialization: μ_i ≈ 1/z_i to maintain μ·z ≈ 1
  Eigen::VectorXd mu(niq);
  if (compatible_dual_start) {
    mu = *opt.inequality_dual_start;
  } else {
    for (int i = 0; i < niq; ++i) {
      mu[i] = std::max(1.0 / z[i], 1e-2);
    }
  }

  Eigen::VectorXd lambda = compatible_dual_start
                               ? *opt.equality_dual_start
                               : Eigen::VectorXd::Zero(meq);
  DispatchDualPredictorDiagnostics dispatch_dual_predictor;
  if (opt.enable_phase_one && !compatible_dual_start) {
    const bool phase_one_accepted = initialize_phase_one_duals(
        prob, x, rh, dh, opt, phase_one, z, mu, lambda);
    if (!phase_one_accepted) {
      // Phase I is a warm-start producer, not an alternative infeasible-start
      // trajectory. Only its certified primal/dual state may alter Phase II.
      x = phase_two_fallback_x;
      assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols, rh, dh);
      for (int i = 0; i < niq; ++i) {
        z[i] = (rh[i] >= 0.0) ? std::max(rh[i], 1.0)
                              : std::max(-rh[i], 1e-2);
        mu[i] = std::max(1.0 / z[i], 1e-2);
      }
      lambda.setZero();
    }
    phase_one.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - phase_one_started).count();
  }

  const double tau = opt.alpha_max;  // fraction-to-boundary factor

  // Evaluate at starting point
  EvalWorkspace eq_ws;
  Eigen::VectorXd grad;
  Eigen::VectorXd hdiag;
  Eigen::VectorXd rg;
  Eigen::SparseMatrix<double> jg;
  objective_gradient_hessian_diag(prob, x, grad, hdiag);

  // ── Gradient-based objective scaling (à la Ipopt nlp_scaling_method) ────────
  // The economic objective is expressed in engineering units ($/MWh · MW), so
  // ‖∇f‖ can reach 1e4–1e6 while the AC/DC balance rows are O(1).  This unscaled
  // KKT system makes the stationarity residual dominated by the cost gradient,
  // stalling the IPM on large/ill-conditioned grids (RTE, Polish).  Damping the
  // objective by obj_scale = min(1, gmax_ref/‖∇f‖∞) brings the Lagrangian
  // gradient and the optimal multipliers back to O(1).  It is a no-op when the
  // objective gradient is already well-scaled (obj_scale == 1).
  double obj_scale = 1.0;
  {
    const double gmax = inf_norm(grad);
    constexpr double kObjGradRef = 100.0;
    if (gmax > kObjGradRef) {
      obj_scale = kObjGradRef / gmax;
    }
  }
  // Add the (obj_scale-1)·∇²f correction to a Lagrangian Hessian computed with
  // unit objective weight.  ∇²f is diagonal (separable generator costs), so only
  // structurally present diagonal entries are touched.
  auto apply_obj_scale_hessian = [&](Eigen::SparseMatrix<double>& H,
                                     const Eigen::VectorXd& hd) {
    if (obj_scale == 1.0) {
      return;
    }
    for (Eigen::Index i = 0; i < hd.size(); ++i) {
      if (hd[i] != 0.0) {
        H.coeffRef(static_cast<int>(i), static_cast<int>(i)) +=
            (obj_scale - 1.0) * hd[i];
      }
    }
  };

  equality_constraints(prob, x, eq_ws, rg);
  equality_jacobian(prob, x, eq_ws, jg);

  if (opt.enable_phase_one && opt.phase_one_dispatch_dual_predictor &&
      warm_start_used && !compatible_dual_start &&
      !phase_one.central_start_accepted) {
    dispatch_dual_predictor = predict_component_dispatch_duals(
        prob, x, xmin, xmax, grad, obj_scale, jg, dh, mu,
        opt.phase_one_dispatch_dual_min_improvement, lambda);
  } else if (!opt.phase_one_dispatch_dual_predictor) {
    dispatch_dual_predictor.status = "disabled";
  } else if (!warm_start_used) {
    dispatch_dual_predictor.status = "not-attempted: no primal warm start";
  } else if (compatible_dual_start) {
    dispatch_dual_predictor.status =
        "not-attempted: continuation state preserved";
  } else if (phase_one.central_start_accepted) {
    dispatch_dual_predictor.status =
        "not-attempted: central Phase-I state accepted";
  }

  // Lagrangian gradient: Lx = obj_scale·∇f + Jg'·λ + Jh'·μ
  Eigen::VectorXd Lx = obj_scale * grad + jg.transpose() * lambda + dh.transpose() * mu;

  // Lagrangian Hessian
  const int m_nonlin = prob.cidx.n_ineq_nonlin;
  auto get_nu_ptr = [&](const Eigen::VectorXd& mu_vec) -> const Eigen::VectorXd* {
    static thread_local Eigen::VectorXd nu_buf;
    if (m_nonlin > 0) {
      nu_buf = mu_vec.head(m_nonlin);
      return &nu_buf;
    }
    return nullptr;
  };

  Eigen::SparseMatrix<double> Lxx;
  lagrangian_hessian(prob, x, lambda, get_nu_ptr(mu), Lxx, 1e-12);
  apply_obj_scale_hessian(Lxx, hdiag);

  // Convergence measures (normalized)
  double obj = obj_scale * objective(prob, x);
  const double obj0 = obj;  // reference for cost condition (NOT updated each iter)

  auto compute_convergence = [&](const Eigen::VectorXd& x_v, const Eigen::VectorXd& z_v,
                                 const Eigen::VectorXd& lambda_v, const Eigen::VectorXd& mu_v,
                                 const Eigen::VectorXd& rg_v, const Eigen::VectorXd& rh_v,
                                 const Eigen::VectorXd& Lx_v, double obj_v) {
    const double maxh = (niq > 0) ? std::max(rh_v.maxCoeff(), 0.0) : 0.0;
    const double raw_feascond = std::max(inf_norm(rg_v), maxh);
    const double scale_x = std::max(inf_norm(x_v), inf_norm(z_v));
    const double feascond = raw_feascond / (1.0 + scale_x);
    const double gradcond = inf_norm(Lx_v) / (1.0 + std::max(inf_norm(mu_v), inf_norm(lambda_v)));
    // Average complementarity per constraint — normalizes by niq to be problem-size-independent.
    const double compcond = (niq > 0) ? z_v.dot(mu_v) / static_cast<double>(niq) / (1.0 + inf_norm(x_v)) : 0.0;
    const double costcond = std::abs(obj_v - obj0) / (1.0 + std::abs(obj0));
    return std::make_tuple(feascond, raw_feascond, gradcond, compcond, costcond);
  };

  auto [feascond, raw_feascond, gradcond, compcond, costcond] =
      compute_convergence(x, z, lambda, mu, rg, rh, Lx, obj);

  const double feas_tol = opt.tol_primal;
  // Complementarity tolerance: for large problems the average z·μ/niq can stall
  // at ~1e-3 due to barrier ill-conditioning, while feas and grad are fully converged.
  // Use a relaxed tolerance for complementarity (minimum 2e-3).
  const double comp_tol = std::max(feas_tol * 10.0, 2e-3);
  bool converged = (feascond < feas_tol && raw_feascond < feas_tol &&
                    gradcond < feas_tol && compcond < comp_tol);

  IPMResult out;
  out.status = "maximum iterations reached";
  out.warm_start_used = warm_start_used;
  out.phase_one_initial_violation = phase_one.initial_violation;
  out.phase_one_constraint_violation = phase_one.final_violation;
  out.phase_one_dual_fit_residual = phase_one.dual_fit_residual;
  out.phase_one_primal_feasible = phase_one.primal_feasible;
  out.phase_one_in_handoff_corridor = phase_one.in_handoff_corridor;
  out.phase_one_dual_initialized = phase_one.dual_initialized;
  out.phase_one_handoff_primal_tolerance =
      phase_one.handoff_primal_tolerance;
  out.phase_one_perturbed_primal_residual =
      phase_one.perturbed_primal_residual;
  out.phase_one_centrality = phase_one.centrality;
  out.phase_one_barrier_mu = phase_one.barrier_mu;
  out.phase_one_budget_exhausted = phase_one.budget_exhausted;
  out.phase_one_iterations = phase_one.iterations;
  out.phase_one_factorizations = phase_one.factorizations;
  out.phase_one_backtracks = phase_one.backtracks;
  out.phase_one_structural_step_attempted =
      phase_one.structural_step_attempted;
  out.phase_one_structural_step_accepted =
      phase_one.structural_step_accepted;
  out.phase_one_structural_factorizations =
      phase_one.structural_factorizations;
  out.phase_one_structural_violation = phase_one.structural_violation;
  out.phase_one_structure = phase_one.structure;
  out.phase_one_runtime_ms = phase_one.runtime_ms;
  out.phase_one_termination = phase_one.termination;
  out.phase_one_linear_solver = phase_one.linear_solver;
  out.dispatch_dual_predictor_attempted =
      dispatch_dual_predictor.attempted;
  out.dispatch_dual_predictor_accepted =
      dispatch_dual_predictor.accepted;
  out.dispatch_dual_predictor_runtime_ms =
      dispatch_dual_predictor.runtime_ms;
  out.dispatch_dual_predictor_baseline_raw =
      dispatch_dual_predictor.baseline_raw;
  out.dispatch_dual_predictor_candidate_raw =
      dispatch_dual_predictor.candidate_raw;
  out.dispatch_dual_predictor_baseline_normalized =
      dispatch_dual_predictor.baseline_normalized;
  out.dispatch_dual_predictor_candidate_normalized =
      dispatch_dual_predictor.candidate_normalized;
  out.dispatch_dual_predictor_status =
      std::move(dispatch_dual_predictor.status);
  out.phase_two_start_accepted = compatible_dual_start ||
                                 phase_one.central_start_accepted;
  out.phase_two_start_rejection_reason = phase_one.rejection_reason;
  out.initial_primal_inf = feascond;
  out.initial_dual_inf = gradcond;
  out.initial_complementarity = compcond;

  // Best-iterate tracking: store the iterate with the smallest max(feas, grad, comp).
  // For oscillating problems on constraint boundaries, the best iterate may satisfy
  // convergence even if the final iterate does not.
  double best_metric = std::max({raw_feascond, gradcond, compcond});
  Eigen::VectorXd best_x = x;
  Eigen::VectorXd best_lambda = lambda;
  Eigen::VectorXd best_mu = mu;
  Eigen::VectorXd best_z = z;
  double best_feas = feascond;
  double best_raw_feas = raw_feascond;
  double best_grad = gradcond;
  double best_comp = compcond;
  int best_iter = 0;

  // Dense pivoted LU is faster and materially more stable for small hybrid
  // KKTs, where sparse condensation can amplify extreme mu/z ratios. Larger
  // systems retain Ruiz-scaled sparse factorization and backend escalation.
  // HACDCPF_OPF_LINEAR_SOLVER can still force either path for diagnostics.
  constexpr int kDenseAutoKktDim = 512;
  const int kkt_dim = n + meq;
  const bool use_dense =
      dense_backend_forced() ||
      (!has_linear_solver_override() && kDenseAutoKktDim > 0 && kkt_dim <= kDenseAutoKktDim);
  DenseKKTCache dense_cache;
  SparseKKTCache local_sparse_cache;
  SparseKKTCache& sparse_cache =
      opt.prepared_state != nullptr
          ? opt.prepared_state->impl_->sparse_cache
          : local_sparse_cache;
  const int symbolic_analyze_calls_before = sparse_cache.symbolic_analyze_calls;
  const int backend_escalations_before = sparse_cache.backend_escalations;
  const int scaling_rebuilds_before = sparse_cache.scaling_rebuilds;
  const bool had_prepared_symbolic = std::any_of(
      std::begin(sparse_cache.analyzed_dim),
      std::end(sparse_cache.analyzed_dim),
      [](int dimension) { return dimension >= 0; });
  // Numeric values and Ruiz scaling belong to one solve. Only backend ordering
  // and analyzePattern state survive through IPMPreparedState.
  sparse_cache.factored = false;
  sparse_cache.solve_degraded = false;
  sparse_cache.scale.resize(0);
  // Structure-aware backend selection (see backend_order): a problem is KLU-
  // eligible only when it has no DC / VSC / DC-DC / energy-router subsystems,
  // i.e. it is a pure AC network.  Hybrid AC/DC KKTs need the MUMPS LDLᵀ
  // inertia for the δ_W regularisation and must not take the KLU path.
  sparse_cache.pure_ac = prob.data.dc_buses.empty() &&
                         prob.data.dc_branches.empty() &&
                         prob.data.converters.empty() &&
                         prob.data.dcdc_converters.empty() &&
                         prob.data.energy_routers.empty();

  // Newton-system form.  The dense path has no inertia oracle and no
  // inequality-block structure, so it always uses the condensed form; the
  // augmented form needs inequalities to be present at all.
  const KktForm kkt_form =
      (use_dense || niq == 0) ? KktForm::Condensed : kkt_form_preference();

  // Barrier-parameter strategy for the corrector (augmented form only):
  //   mehrotra — σ = (μ_aff/μ)³ probe (default, historical);
  //   abo      — Armand–Benoist–Orban dynamic-μ (see the corrector below).
  // HACDCPF_OPF_MU_STRATEGY = mehrotra | abo
  const bool mu_strategy_abo = []() {
    const char* env = std::getenv("HACDCPF_OPF_MU_STRATEGY");
    return env != nullptr && std::string(env) == "abo";
  }();
  double mu_dyn = -1.0;  // tracked dynamic barrier parameter (ABO)
  double mu0_abo = 0.0;  // its initial value μ₀ (θ_S scale)

  // Unified factor/solve lambdas
  auto factor_kkt = [&](const Eigen::SparseMatrix<double>& w,
                        const Eigen::SparseMatrix<double>& jg_mat,
                        double reg) -> bool {
    ++out.factorization_calls;
    if (use_dense) {
      return factor_kkt_dense(dense_cache, w, jg_mat, reg);
    }
    return factor_kkt_sparse(sparse_cache, w, jg_mat, reg);
  };
  auto solve_kkt = [&](const Eigen::VectorXd& rhs,
                       Eigen::VectorXd& dx_out,
                       Eigen::VectorXd& dlambda_out) -> bool {
    ++out.linear_solve_calls;
    if (use_dense) {
      return kkt_solve_dense(dense_cache, rhs, dx_out, dlambda_out);
    }
    return kkt_solve_sparse(sparse_cache, rhs, dx_out, dlambda_out);
  };

  // Last accepted Wächter–Biegler δ_W (augmented form) — persists across
  // iterations to warm-start the inertia loop (see below).
  double delta_w_prev = 0.0;

  // Periodic inertia re-check for the pure-AC KLU fast path.  KLU/LU cannot
  // report the KKT inertia (negative-pivot count), so the δ_W regularisation
  // is silently bypassed while KLU is active.  Every kInertiaGatePeriod
  // factorizations we re-check the inertia with MUMPS and apply δ_W to that
  // step if the reduced Hessian is indefinite — this both regularises the
  // step (large trajectory win: case2869 homotopy 154→73 iterations) and
  // surfaces persistent difficulty.  KLU is always resumed afterwards (a
  // one-off indefinite KKT is not grounds to abandon it — the (θ,φ) filter
  // tolerates those steps); a genuine persistent KLU failure is left to the
  // solve_degraded → escalation backstop.
  int inertia_gate_countdown = 0;  // factorizations since the last MUMPS re-check

  // Objective-stagnation tracking (endgame early stop — see the epilogue).
  double obj_prev = obj0;
  int stagnant_count = 0;

  // (θ,φ) filter history (Wächter–Biegler §3.2) — see the acceptance test
  // below for why domination memory is required.  φ is re-evaluated at the
  // current γ for each history pair, so only (θ, f̃, Σlog z) is stored.
  struct FilterPair {
    double theta;
    double obj;
    double logz;
  };
  std::vector<FilterPair> filter_history;

  for (int iter = 0; iter < opt.max_iter && !converged; ++iter) {
    out.iterations = iter;
    out.primal_inf = feascond;
    out.dual_inf = gradcond;
    out.complementarity = compcond;

    if (opt.verbose) {
      [[maybe_unused]] const double maxh_dbg = (niq > 0) ? std::max(rh.maxCoeff(), 0.0) : 0.0;
      HACDCPF_LOG_DEBUG("[parity-ipm] iter={} feas={} raw_feas={} grad={} comp={} cost={} |rg|={} maxh={} |Lx|={}",
                iter, feascond, raw_feascond, gradcond, compcond, costcond,
                inf_norm(rg), maxh_dbg, inf_norm(Lx));
    }

    // Safeguard z, μ — use 1e-12 floor to prevent extreme σ = μ/z ratios
    const Eigen::VectorXd z_safe = z.cwiseMax(1e-12);
    const Eigen::VectorXd mu_safe = mu.cwiseMax(1e-12);
    const Eigen::VectorXd sigma_s = mu_safe.cwiseQuotient(z_safe);  // Σ = diag(μ/z)

    // Build condensed reduced Hessian: W = Lxx + Jh' · diag(μ/z) · Jh
    // (condensed form only — the augmented form keeps Jh and −ZM⁻¹ explicit
    // and never forms the triple product).
    Eigen::SparseMatrix<double> W;
    if (kkt_form == KktForm::Condensed) {
      Eigen::SparseMatrix<double> dh_scaled = dh;
      for (int col = 0; col < dh_scaled.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(dh_scaled, col); it; ++it) {
          it.valueRef() *= sigma_s[it.row()];
        }
      }
      W = Lxx + dh.transpose() * dh_scaled;
      W.makeCompressed();
    }

    // Equality Jacobian in the format needed for KKT: jg is meq × n
    // dg_for_kkt = Jg transposed for KKT fill: we pass jg directly (meq × n)
    // The reference formulation stores dg as n x neq; our jg is meq x n.
    // The KKT uses Jg (rows=constraints, cols=vars), matching our jg.

    Eigen::VectorXd dx_aff;
    Eigen::VectorXd dlambda_aff;
    Eigen::VectorXd dz_aff;
    Eigen::VectorXd dmu_aff;
    Eigen::VectorXd dx;
    Eigen::VectorXd dlambda;
    Eigen::VectorXd dz;
    Eigen::VectorXd dmu;
    // Corrector RHS of the current iteration (augmented form) — kept for the
    // second-order correction re-solve in the filter line search below.
    Eigen::VectorXd rhs_corr_aug;
    // Effective barrier target γ of the current corrector step (set by
    // whichever Newton form ran) — the (θ,φ) filter's merit φ uses it.
    double gamma_corr = 0.0;
    // Whether the full-α trial was feasibility-passing (drives the
    // centrality-recovery retry in the augmented corrector).
    bool bt0_feas_ok = false;
    // Current duality measure (set by whichever Newton form ran) — the
    // centering-corrector target for the centrality-recovery retry.
    double mu_cur = 0.0;

    if (kkt_form == KktForm::Augmented) {
      // ── Augmented Newton system ──────────────────────────────────────
      // Wächter–Biegler inertia correction: with the reduced Hessian
      // positive definite the augmented KKT has exactly (n, meq+niq, 0)
      // inertia.  An LDLᵀ backend reports the negative-pivot count for
      // free; when it differs, escalate δ_W (kick 1e-8·‖Lxx‖, ×8 per retry,
      // cap 1e-2·‖Lxx‖) and refactor.  LU backends report no inertia, so
      // the factorization is accepted as-is (the pre-augmentation policy).
      bool factor_ok = false;
      const double lxx_norm = std::max(1.0, Lxx.coeffs().cwiseAbs().maxCoeff());
      // δ_C for the (2,2) equality block.  The regularized saddle system
      // permits equality residuals of order δ_C·‖λ_eq‖ — with stiff DC
      // networks (line conductances ~250 pu and large LMPs) that floor can
      // dominate the convergence tolerance, so it is exposed for tuning.
      // HACDCPF_OPF_DELTA_C (absolute, default 1e-8·‖Lxx‖).
      const double delta_c = []() {
        const char* env = std::getenv("HACDCPF_OPF_DELTA_C");
        if (env != nullptr) {
          const double v = std::atof(env);
          if (v > 0.0) return v;
        }
        return 1e-8;
      }() * lxx_norm;
      // Inertia gate: on the pure-AC KLU fast path, re-check the inertia with
      // MUMPS every kInertiaGatePeriod factorizations (KLU cannot report it).
      // HACDCPF_OPF_INERTIA_GATE = N (default 8; ≤0 disables the gate, keeping
      // KLU throughout with no periodic MUMPS re-check).
      // Inertia re-check (OPT-IN, disabled by default).  On the pure-AC KLU
      // fast path, KLU cannot report the KKT inertia, so the δ_W regularisation
      // is bypassed; this opt-in periodically re-checks the inertia with MUMPS
      // and applies δ_W to that step.  It is OFF by default because its effect
      // is strongly case-dependent and not predictable a priori: the periodic
      // δ_W regularisation materially improves SOME trajectories (case2869
      // homotopy 154 → 73 iterations, 59 s → 15 s) but DEGRADES others
      // (case9241 homotopy stalls at t = 0.625 instead of reaching the 315837
      // reference).  The robust default is the plain pure-AC→KLU path; a
      // genuine persistent KLU failure is caught by solve_degraded →
      // escalation.  HACDCPF_OPF_INERTIA_GATE = N enables a re-check every N
      // factorizations (≤ 0 disables; default 0).
      const int kInertiaGatePeriod = []() {
        const char* env = std::getenv("HACDCPF_OPF_INERTIA_GATE");
        return env != nullptr ? std::atoi(env) : 0;
      }();
      bool inertia_gate_check = false;
      int klu_resume = 0;
      if (kInertiaGatePeriod > 0 && sparse_cache.pure_ac &&
          sparse_cache.active == 2) {
        if (++inertia_gate_countdown >= kInertiaGatePeriod) {
          inertia_gate_countdown = 0;
          inertia_gate_check = true;
          klu_resume = sparse_cache.active;  // = KLU(2)
          sparse_cache.active = 4;           // force MUMPS for the check
        }
      }
      // δ_W warm-start applies ONLY to MUMPS factorizations (which report
      // inertia).  KLU factorizations are left UNREGULARISED (δ_W = 0) so a
      // MUMPS re-check's escalated δ_W does not leak into and perturb the
      // healthy KLU trajectory — that leak was measured to cost case13659
      // ~44 extra iterations.  The warm-start itself (Ipopt's κ_W⁻ rule)
      // skips the 5–8 escalate-from-scratch refactorizations per iteration.
      const bool mumps_step = (sparse_cache.active == 4);
      double delta_w = (mumps_step && delta_w_prev > 0.0)
                           ? std::max(1e-8 * lxx_norm, 0.5 * delta_w_prev)
                           : 0.0;
      for (int attempt = 0; attempt < 8; ++attempt) {
        ++out.factorization_calls;
        if (!factor_kkt_sparse_augmented(sparse_cache, Lxx, jg, dh,
                                         z_safe, mu_safe, delta_w, delta_c)) {
          break;  // backend failure — escalating δ_W cannot help
        }
        int negs = -1;
#if defined(HACDCPF_HAVE_MUMPS)
        if (sparse_cache.active == 4) {
          negs = sparse_cache.mumps.negative_eigenvalues();
        }
#endif
        if (negs < 0 || negs == meq + niq) {
          factor_ok = true;
          break;
        }
        delta_w = (delta_w == 0.0)
                      ? 1e-8 * lxx_norm
                      : std::min(8.0 * delta_w, 1e-2 * lxx_norm);
        if (attempt == 7) factor_ok = true;  // accept best effort at the cap
      }
      // Gate resolution: ALWAYS resume the KLU fast path after the periodic
      // re-check.  The re-check's purpose is twofold — (a) apply δ_W
      // regularisation to THIS step where the reduced Hessian is indefinite
      // (which materially improves the trajectory: measured case2869 homotopy
      // 154 → 73 iterations, 59 s → 17 s), and (b) detect a trajectory that
      // persistently needs regularisation.  We deliberately do NOT lock to
      // MUMPS on a single bad-inertia check: the (θ,φ) filter tolerates the
      // mildly indefinite steps that KLU/LU still solves accurately, so a
      // one-off indefinite KKT is not evidence that KLU must be abandoned
      // (measured: case13659 locked prematurely and ran 2.4× slower / less
      // accurately than pure KLU).  A genuine, persistent KLU failure surfaces
      // instead through solve_degraded → escalation, the existing backstop.
      if (inertia_gate_check) {
        sparse_cache.active = klu_resume;  // resume the KLU fast path
      }
      // Only MUMPS steps contribute to the δ_W warm-start, so a re-check's
      // escalated value never leaks into subsequent KLU steps.
      if (factor_ok && mumps_step) delta_w_prev = delta_w;
      if (!factor_ok) {
        out.status = "KKT factorization failed";
        break;
      }

      // Predictor (γ = 0):  rhs = [−Lx; −rg; −rh]
      Eigen::VectorXd rhs_aff(n + meq + niq);
      rhs_aff.head(n) = -Lx;
      rhs_aff.segment(n, meq) = -rg;
      rhs_aff.tail(niq) = -rh;
      ++out.linear_solve_calls;
      if (!kkt_solve_sparse_augmented(sparse_cache, rhs_aff,
                                      dx_aff, dlambda_aff, dmu_aff)) {
        out.status = "predictor back-solve failed";
        break;
      }
      const Eigen::VectorXd jh_dx_aff = dh * dx_aff;
      dz_aff = -(rh + z_safe) - jh_dx_aff;

      double alpha_p_aff = std::min(ftb(z_safe, dz_aff, 1.0), 1.0);
      double alpha_d_aff = std::min(ftb(mu_safe, dmu_aff, 1.0), 1.0);
      mu_cur = z_safe.dot(mu_safe) / static_cast<double>(niq);
      double gamma;
      if (mu_strategy_abo) {
        // Armand–Benoist–Orban dynamic-μ (2008, §1/§6): the μ-update is a
        // Newton step on the augmented (w, μ) system with updating function
        // θ_S (q-superlinear, γ_q = 1, b = μ₀):
        //   μ⁺ = μ²/μ₀,  γ = μ + α_aff·(μ⁺ − μ) = μ(1−α) + α·μ²/μ₀.
        // The affine fraction-to-boundary steplength damps the decrease:
        // far from the solution (α small) μ holds — centering is preserved,
        // keeping ftb steps large; as full steps return (α → 1) μ collapses
        // quadratically.  No artificial linear→superlinear switch, and {μ_k}
        // stays synchronized with the optimality residual by construction.
        if (mu_dyn < 0.0) {
          mu_dyn = mu_cur;
          mu0_abo = mu_cur;
        }
        const double mu_plus = mu_dyn * mu_dyn / std::max(mu0_abo, 1e-30);
        const double alpha_aff = std::min({alpha_p_aff, alpha_d_aff, 1.0});
        gamma = std::clamp(mu_dyn + alpha_aff * (mu_plus - mu_dyn), 1e-14, 1e4);
      } else {
        const double mu_aff_val =
            (z_safe + alpha_p_aff * dz_aff).dot(mu_safe + alpha_d_aff * dmu_aff)
            / static_cast<double>(niq);
        const double ratio = std::clamp(mu_aff_val / (mu_cur + 1e-16), 0.0, 1.0);
        const double sigma = ratio * ratio * ratio;
        gamma = std::clamp(sigma * mu_cur, 1e-14, 1e4);
      }

      Eigen::VectorXd cross = dmu_aff.cwiseProduct(dz_aff);
      const double cross_limit = 0.1 * gamma;
      for (int i = 0; i < niq; ++i) {
        cross[i] = std::clamp(cross[i], -cross_limit, cross_limit);
      }

      // Corrector with re-centering.  Near the barrier boundary the KKT
      // conditioning degrades like κ ~ 1/μ; once μ has collapsed far enough
      // the solved direction is numerically orthogonal to the true one and
      // its norm explodes (observed: ‖d‖ ~ 1e85 with ftb α pinned at the
      // 1e-10 floor, trial residuals ~1e75).  The standard remedy is a
      // centering step: raise γ toward the current μ so the corrector pulls
      // z∘μ back toward the central path — this restores the (3,3)-block
      // scaling and a bounded direction.  Detect the explosion by the
      // fraction-to-boundary limit (α < 1e-9) and retry ≤ 2 times.
      bool corrector_failed = false;
      gamma_corr = gamma;
      Eigen::VectorXd cross_corr = cross;
      for (int recenter = 0; ; ++recenter) {
        // Corrector:  rhs = [−Lx; −rg; −(rh + (γe − cross)/μ)]
        rhs_corr_aug.resize(n + meq + niq);
        rhs_corr_aug.head(n) = -Lx;
        rhs_corr_aug.segment(n, meq) = -rg;
        for (int i = 0; i < niq; ++i) {
          rhs_corr_aug[n + meq + i] = -(rh[i] + (gamma_corr - cross_corr[i]) / mu_safe[i]);
        }
        ++out.linear_solve_calls;
        if (!kkt_solve_sparse_augmented(sparse_cache, rhs_corr_aug, dx, dlambda, dmu)) {
          out.status = "corrector back-solve failed";
          corrector_failed = true;
          break;
        }
        const Eigen::VectorXd jh_dx = dh * dx;
        dz = -(rh + z_safe) - jh_dx;
        if (!dx.allFinite() || !dlambda.allFinite()) {
          out.status = "NaN in corrector step";
          corrector_failed = true;
          break;
        }

        // Gondzio extra correctors:  rhs = [0; 0; −eps_gc/μ]
        if (gamma_corr > 1e-12) {
          constexpr double kBetaGC = 0.1;
          for (int gc = 0; gc < 2; ++gc) {
            const double alpha_p_t = std::min(ftb(z_safe, dz, tau), 1.0);
            const double alpha_d_t = std::min(ftb(mu_safe, dmu, tau), 1.0);
            const Eigen::VectorXd z_t = z_safe + alpha_p_t * dz;
            const Eigen::VectorXd mu_t = mu_safe + alpha_d_t * dmu;
            Eigen::VectorXd eps_gc =
                (Eigen::VectorXd::Constant(niq, kBetaGC * gamma_corr) - z_t.cwiseProduct(mu_t)).cwiseMax(0.0);
            if (eps_gc.maxCoeff() < 1e-12 * gamma_corr) {
              break;
            }
            Eigen::VectorXd rhs_gc = Eigen::VectorXd::Zero(n + meq + niq);
            rhs_gc.tail(niq) = -eps_gc.cwiseQuotient(mu_safe);
            Eigen::VectorXd dx_gc;
            Eigen::VectorXd dlambda_gc;
            Eigen::VectorXd dmu_gc;
            ++out.linear_solve_calls;
            if (!kkt_solve_sparse_augmented(sparse_cache, rhs_gc,
                                            dx_gc, dlambda_gc, dmu_gc)) {
              break;
            }
            if (!dx_gc.allFinite()) {
              break;
            }
            const Eigen::VectorXd dz_gc = -(dh * dx_gc);
            dx += dx_gc;
            dlambda += dlambda_gc;
            dz += dz_gc;
            dmu += dmu_gc;
          }
        }

        const double ftb_p = ftb(z_safe, dz, tau);
        const double ftb_d = ftb(mu_safe, dmu, tau);
        if (std::min(ftb_p, ftb_d) >= 1e-9 || recenter >= 2) {
          break;
        }
        gamma_corr = std::max(gamma_corr, 0.5 * mu_cur);
        cross_corr.setZero();
      }
      if (corrector_failed) {
        break;
      }
      // ABO: the next iteration's μ is the barrier target actually used
      // (including any re-centering that fired).
      if (mu_strategy_abo) mu_dyn = gamma_corr;
    } else {
    // ────────────────────────────────────────────────────────────────
    // STEP 1: Affine (predictor) step with γ = 0
    // ────────────────────────────────────────────────────────────────
    // N_aff = Lx + Jh' · (μ .* rh ./ z)    [condensed RHS, γ=0]
    const Eigen::VectorXd N_aff = Lx + dh.transpose() * (mu_safe.cwiseProduct(rh).cwiseQuotient(z_safe));
    const Eigen::VectorXd rhs_eq = -rg;

    // Progressive regularization
    bool factor_ok = false;
    const double W_norm = std::max(1.0, W.coeffs().cwiseAbs().maxCoeff());
    constexpr std::array<double, 4> kRegLevels{0.0, 1.0, 1e-6, 1e-4};
    constexpr double kDeltaMin = 1e-8;
    for (double level : kRegLevels) {
      const double reg = (level == 0.0) ? 0.0 :
                         (level == 1.0) ? kDeltaMin * W_norm : level * W_norm;
      const double reg_eff = std::max(reg, 1e-12);
      if (!factor_kkt(W, jg, reg_eff)) {
        continue;
      }
      Eigen::VectorXd rhs_aff(n + meq);
      rhs_aff.head(n) = -N_aff;
      rhs_aff.tail(meq) = rhs_eq;
      if (!solve_kkt(rhs_aff, dx_aff, dlambda_aff)) {
        continue;
      }
      factor_ok = true;
      break;
    }
    if (!factor_ok) {
      out.status = "KKT factorization failed";
      break;
    }

    // Recover affine Δz, Δμ
    const Eigen::VectorXd jh_dx_aff = dh * dx_aff;
    dz_aff = -(rh + z_safe) - jh_dx_aff;
    dmu_aff = -(mu_safe + sigma_s.cwiseProduct(dz_aff));

    // Affine step lengths (probe with τ = 1.0)
    double alpha_p_aff = std::min(ftb(z_safe, dz_aff, 1.0), 1.0);
    double alpha_d_aff = std::min(ftb(mu_safe, dmu_aff, 1.0), 1.0);

    // Adaptive centering parameter σ = (μ_aff / μ_cur)³
    mu_cur = z_safe.dot(mu_safe) / static_cast<double>(niq);
    const double mu_aff_val =
        (z_safe + alpha_p_aff * dz_aff).dot(mu_safe + alpha_d_aff * dmu_aff)
        / static_cast<double>(niq);
    const double ratio = std::clamp(mu_aff_val / (mu_cur + 1e-16), 0.0, 1.0);
    const double sigma = ratio * ratio * ratio;
    double gamma = std::clamp(sigma * mu_cur, 1e-14, 1e4);
    gamma_corr = gamma;  // shared merit-φ barrier target for the (θ,φ) filter

    // ────────────────────────────────────────────────────────────────
    // STEP 2: Corrector step with Mehrotra second-order centering
    // ────────────────────────────────────────────────────────────────
    // cross = clamp(Δμ_aff ⊙ Δz_aff, ±0.1γ)
    Eigen::VectorXd cross = dmu_aff.cwiseProduct(dz_aff);
    const double cross_limit = 0.1 * gamma;
    for (int i = 0; i < niq; ++i) {
      cross[i] = std::clamp(cross[i], -cross_limit, cross_limit);
    }

    // N_corr = Lx + Jh' · ((μ.*rh + γ·e - cross) ./ z)
    Eigen::VectorXd ineq_rhs(niq);
    for (int i = 0; i < niq; ++i) {
      ineq_rhs[i] = (mu_safe[i] * rh[i] + gamma - cross[i]) / z_safe[i];
    }
    const Eigen::VectorXd N_corr = Lx + dh.transpose() * ineq_rhs;

    Eigen::VectorXd rhs_corr(n + meq);
    rhs_corr.head(n) = -N_corr;
    rhs_corr.tail(meq) = rhs_eq;

    // Reuse LU factorization — just a back-solve
    if (!solve_kkt(rhs_corr, dx, dlambda)) {
      out.status = "corrector back-solve failed";
      break;
    }

    // Recover corrector Δz, Δμ
    const Eigen::VectorXd jh_dx = dh * dx;
    dz = -(rh + z_safe) - jh_dx;
    // Δμ = -(μ + σ_s·Δz + (cross - γ·e) / z)
    dmu.resize(niq);
    for (int i = 0; i < niq; ++i) {
      dmu[i] = -(mu_safe[i] + sigma_s[i] * dz[i] + (cross[i] - gamma) / z_safe[i]);
    }

    if (!dx.allFinite() || !dlambda.allFinite()) {
      out.status = "NaN in corrector step";
      break;
    }

    // ────────────────────────────────────────────────────────────────
    // Gondzio extra correctors (up to 2)
    // ────────────────────────────────────────────────────────────────
    if (gamma > 1e-12) {
      constexpr double kBetaGC = 0.1;
      for (int gc = 0; gc < 2; ++gc) {
        const double alpha_p_t = std::min(ftb(z_safe, dz, tau), 1.0);
        const double alpha_d_t = std::min(ftb(mu_safe, dmu, tau), 1.0);
        const Eigen::VectorXd z_t = z_safe + alpha_p_t * dz;
        const Eigen::VectorXd mu_t = mu_safe + alpha_d_t * dmu;

        // Find under-corrected complementarity pairs
        Eigen::VectorXd eps_gc =
            (Eigen::VectorXd::Constant(niq, kBetaGC * gamma) - z_t.cwiseProduct(mu_t)).cwiseMax(0.0);
        if (eps_gc.maxCoeff() < 1e-12 * gamma) {
          break;
        }

        // Solve extra correction (equality residual = 0)
        Eigen::VectorXd rhs_gc(n + meq);
        rhs_gc.head(n) = -(dh.transpose() * eps_gc.cwiseQuotient(z_safe));
        rhs_gc.tail(meq).setZero();

        Eigen::VectorXd dx_gc;
        Eigen::VectorXd dlambda_gc;
        if (!solve_kkt(rhs_gc, dx_gc, dlambda_gc)) {
          break;
        }
        if (!dx_gc.allFinite()) {
          break;
        }

        const Eigen::VectorXd dz_gc = -(dh * dx_gc);
        const Eigen::VectorXd dmu_gc = eps_gc.cwiseQuotient(z_safe) - sigma_s.cwiseProduct(dz_gc);

        dx += dx_gc;
        dlambda += dlambda_gc;
        dz += dz_gc;
        dmu += dmu_gc;
      }
    }
    }  // condensed vs augmented Newton system

    // ────────────────────────────────────────────────────────────────
    // Fraction-to-boundary step lengths (separate primal/dual)
    // ────────────────────────────────────────────────────────────────
    double alpha_p = std::max(std::min(ftb(z_safe, dz, tau), 1.0), 1e-10);
    double alpha_d = std::max(std::min(ftb(mu_safe, dmu, tau), 1.0), 1e-10);

    // Vacuous-step rule: once the fraction-to-boundary limit pins the step
    // below ~1e-7, the filter's progress margins (η·α, γθ·θ) become
    // numerically vacuous — any wiggle passes, and the IPM random-walks
    // hundreds of iterations while comp slowly improves toward an
    // *infeasible* stationary point of the barrier problem (observed on
    // case13659 from iter ~166: α ~ 1e-9 "accepted" steps, feas stuck at
    // 0.16, viol ending at 30).  Declare the direction unusable so the
    // restoration phase fires instead of accepting noise.
    const bool step_vacuous = std::min(alpha_p, alpha_d) < 1e-7;

    if (opt.verbose) {
      HACDCPF_LOG_DEBUG("  α_p={} α_d={} |dx|={} min(z)={} max(σ)={} min(σ)={}",
                alpha_p, alpha_d, inf_norm(dx),
                z_safe.minCoeff(), sigma_s.maxCoeff(), sigma_s.minCoeff());
    }

    // ────────────────────────────────────────────────────────────────
    // Residual filter line search
    // ────────────────────────────────────────────────────────────────
    // A full nonlinear OPF Newton step is not globally convergent.  The former
    // unconditional update allowed a locally accurate KKT step to increase the
    // nonlinear AC/DC residual repeatedly until the condensed KKT became
    // singular (case300_acdc typically failed around iteration 700).  Evaluate
    // the actual nonlinear residuals and accept a trial when it improves at
    // least one filter axis without materially degrading the others.
    constexpr int kMaxBacktracks = 14;
    constexpr double kBacktrack = 0.5;
    constexpr double kFilterEta = 1e-4;
    // (θ,φ) filter constants (Wächter–Biegler §3.2, Ipopt defaults).
    constexpr double kGammaTheta = 1e-5;
    constexpr double kGammaPhi = 1e-5;
    constexpr double kDelta = 1.0;
    constexpr double kSTheta = 1.1;
    constexpr double kEtaPhi = 1e-8;
    constexpr double kThetaMin = 1e-4;
    // θ_max safeguard (Ipopt's filter initialization): trials above this
    // absolute feasibility ceiling are rejected regardless of φ progress —
    // without it, a φ-greedy trajectory can trade feasibility for objective
    // unboundedly (observed on case13659: θ drifted to a raw violation of 30
    // over 640 accepted steps).  Exceeding it eventually exhausts the line
    // search, which is what triggers the restoration phase.
    const double theta_max = 1e4 * std::max(1.0, out.initial_primal_inf);
    // Small OPFs historically converge reliably with the undamped
    // predictor-corrector step and can require a temporary increase in all
    // three filter axes.  Enable the more expensive globalization where the
    // KKT is large enough for that undamped behavior to become brittle.
    constexpr int kLargeKktFilterThreshold = 512;
    // HACDCPF_OPF_FILTER_ALL=1 forces the residual filter on regardless of
    // KKT size: small-but-stiff systems (e.g. hybrid AC/DC with ~250-pu line
    // conductances) diverge on the unsupervised full-step path even though
    // their dimension is below the historical 512 cutoff.
    const bool use_residual_filter =
        kkt_dim >= kLargeKktFilterThreshold ||
        std::getenv("HACDCPF_OPF_FILTER_ALL") != nullptr;
    const bool strict_theta_filter =
        std::getenv("HACDCPF_OPF_STRICT_THETA") != nullptr;
    const bool no_legacy_paths =
        std::getenv("HACDCPF_OPF_NO_LEGACY") != nullptr;
    bool step_accepted = false;
    Eigen::VectorXd x_trial, z_trial, mu_trial, lambda_trial;
    Eigen::VectorXd rg_trial, rh_trial, grad_trial, hdiag_trial, Lx_trial;
    Eigen::SparseMatrix<double> jg_trial, dh_trial;
    double obj_trial = obj;
    double feas_trial = feascond;
    double raw_feas_trial = raw_feascond;
    double grad_trial_cond = gradcond;
    double comp_trial = compcond;
    double cost_trial = costcond;

    // Barrier merit quantities for the (θ,φ) filter:
    //   φ_γ(x,z) = f̃(x) − γ·Σ log z_i
    // with γ the current corrector's barrier target.  With correct KKT
    // inertia the Newton direction is guaranteed descent on φ — the property
    // the raw stationarity axis lacks at κ ~ 1/μ (Wächter–Biegler §3.2).
    // phi_k/dphi are recomputed inside run_backtracks for each new direction
    // (the centrality-recovery retry changes both γ and the direction).
    double log_z_sum = 0.0;
    for (int i = 0; i < niq; ++i) {
      log_z_sum += std::log(z_safe[i]);
    }
    double phi_k = 0.0;
    double dphi = 0.0;

    // A trial must improve on the current pair AND on every filter-history
    // pair (domination memory) — otherwise hundreds of φ-wiggle steps can
    // wander while θ/comp slowly degrade (observed: 600 iterations drifting
    // after a near-converged point at iter 48).
    double log_zt_last = 0.0;
    bool log_zt_valid = false;

    // Evaluate the nonlinear residuals/Jacobians at a trial point and run the
    // filter acceptance test; (re)fills the trial-state variables either way.
    auto eval_and_test_trial = [&](int bt_now) -> bool {
      log_zt_valid = false;  // new trial point invalidates the φ log-sum
      obj_trial = obj_scale * objective(prob, x_trial);
      equality_constraints(prob, x_trial, eq_ws, rg_trial);
      equality_jacobian(prob, x_trial, eq_ws, jg_trial);
      assemble_inequalities(prob, x_trial, xmin, xmax, lb_cols, ub_cols,
                            rh_trial, dh_trial);
      objective_gradient_hessian_diag(prob, x_trial, grad_trial, hdiag_trial);
      Lx_trial = obj_scale * grad_trial + jg_trial.transpose() * lambda_trial +
                 dh_trial.transpose() * mu_trial;
      std::tie(feas_trial, raw_feas_trial, grad_trial_cond, comp_trial,
               cost_trial) =
          compute_convergence(x_trial, z_trial, lambda_trial, mu_trial,
                              rg_trial, rh_trial, Lx_trial, obj_trial);

      const double ap_progress = kFilterEta * std::max(alpha_p, 1e-8);
      const double ad_progress = kFilterEta * std::max(alpha_d, 1e-8);
      const double feas_guard = std::max(10.0 * feas_tol, 1.02 * feascond);
      const double grad_guard = std::max(10.0 * opt.tol_dual, 1.02 * gradcond);
      // Catastrophe guard on every acceptance path: a step that worsens the
      // dual axes by orders of magnitude is poisonous even when feasibility
      // improves (observed: comp 1.1e-3 → 7.6e8 accepted on the feasibility
      // axis alone at rte iter 104, which blew up the next iteration's μ).
      const bool dual_axes_bounded =
          grad_trial_cond <= 100.0 * std::max(gradcond, 1e-12) &&
          comp_trial <= 100.0 * std::max(compcond, 1e-12);
      const bool feasibility_progress =
          dual_axes_bounded &&
          feas_trial <= (1.0 - ap_progress) * feascond;
      if (bt_now == 0) {
        bt0_feas_ok = (feas_trial <= feas_guard);
      }
      const bool stationarity_progress =
          feas_trial <= feas_guard &&
          grad_trial_cond <= (1.0 - ad_progress) * gradcond;
      const bool complementarity_progress =
          feas_trial <= feas_guard && grad_trial_cond <= grad_guard &&
          comp_trial <= (1.0 - ad_progress) * compcond;

      // (θ,φ) filter with domination memory.  φ_t costs an O(niq) log-sum,
      // so it is evaluated at most once per trial and only when some pair
      // (current or history) cannot be dominated on θ alone — most accepted
      // steps never need it (~8% of runtime in logs without the shortcut).
      const double alpha_min = std::min(alpha_p, alpha_d);
      const bool switching =
          feascond <= kThetaMin &&
          alpha_min * (-dphi) >= kDelta * std::pow(feascond, kSTheta);
      const bool theta_decrease =
          feas_trial <= (1.0 - kGammaTheta) * feascond;
      bool filter_ok = false;
      if (dual_axes_bounded) {
        bool phi_done = false;
        double phi_t = 0.0;
        auto eval_phi = [&]() -> double {
          if (!phi_done) {
            double s = 0.0;
            for (int i = 0; i < niq; ++i) s += std::log(z_trial[i]);
            log_zt_last = s;
            log_zt_valid = true;
            phi_t = obj_trial - gamma_corr * s;
            phi_done = true;
          }
          return phi_t;
        };
        // Current pair: θ dominates, else φ must drop by the θ-margin.
        filter_ok = theta_decrease ||
                    (eval_phi() <= phi_k - kGammaPhi * feascond);
        // History pairs: dominate each on θ or φ.
        if (filter_ok) {
          for (const auto& hp : filter_history) {
            if (feas_trial <= (1.0 - kGammaTheta) * hp.theta) continue;
            const double phi_h = hp.obj - gamma_corr * hp.logz;
            if (!(eval_phi() <= phi_h - kGammaPhi * hp.theta)) {
              filter_ok = false;
              break;
            }
          }
        }
        // Switching condition: near feasibility with genuine φ-descent,
        // require Armijo decrease on φ — the certificate that the step is a
        // good Newton step for the barrier problem (not just a θ fluke).
        if (filter_ok && switching) {
          filter_ok = eval_phi() <= phi_k + kEtaPhi * alpha_min * dphi;
        }
      }
      const bool filter_accept = filter_ok;

      if (!use_residual_filter) return true;
      if (feas_trial > theta_max) return false;  // θ_max safeguard
      // Diagnostic gate (HACDCPF_OPF_STRICT_THETA=1): accept only on strict
      // feasibility decrease — used to isolate whether φ-greedy acceptance is
      // what keeps a trajectory oscillating above the tolerance.
      if (strict_theta_filter) {
        return dual_axes_bounded &&
               feas_trial <= (1.0 - 1e-4 * std::max(alpha_p, 1e-8)) * feascond;
      }
      // HACDCPF_OPF_NO_LEGACY=1 disables the legacy 3-axis acceptance paths
      // (diagnostic): the complementarity axis' loose grad guard (2%/step,
      // compounding) lets a trajectory ride comp-decrease into a wrong corner
      // — observed on case2869pegase: feas/comp converging while obj and
      // stationarity inflate ~3×.
      if (no_legacy_paths) {
        return filter_accept;
      }
      return filter_accept || feasibility_progress ||
             stationarity_progress || complementarity_progress;
    };

    // Second-order correction (Wächter–Biegler §3.2): when the first trial is
    // rejected *because feasibility worsened* (θ_trial > θ_k), the Newton step
    // is suffering the Maratos effect — the linearized constraints predict
    // well but the nonlinear residual rises.  Re-solve the SAME factorization
    // with the equality block re-evaluated at the trial point,
    //   rhs_eq ← −(α·rg + rg(x + α·dx)),
    // and try the composite step x + α·dx + dx_soc once per iteration.  This
    // cancels the first-order residual increase at back-solve cost.
    bool soc_attempted = false;

    // One backtracking line-search pass over the current direction; called
    // again after a centering-corrector retry (which changes γ, dx, dz).
    auto run_backtracks = [&]() {
      soc_attempted = false;
      double dz_over_z_sum = 0.0;
      for (int i = 0; i < niq; ++i) {
        dz_over_z_sum += dz[i] / z_safe[i];
      }
      phi_k = obj - gamma_corr * log_z_sum;
      dphi = obj_scale * grad.dot(dx) - gamma_corr * dz_over_z_sum;
      for (int bt = 0; bt <= kMaxBacktracks && !step_vacuous; ++bt) {
      x_trial = x + alpha_p * dx;
      z_trial = (z + alpha_p * dz).cwiseMax(1e-12);
      mu_trial = (mu + alpha_d * dmu).cwiseMax(1e-12);
      lambda_trial = lambda + alpha_d * dlambda;
      if (!x_trial.allFinite() || !z_trial.allFinite() ||
          !mu_trial.allFinite() || !lambda_trial.allFinite()) {
        alpha_p *= kBacktrack;
        alpha_d *= kBacktrack;
        continue;
      }

      if (eval_and_test_trial(bt)) {
        step_accepted = true;
        ++out.accepted_steps;
        if (!log_zt_valid) {
          double s = 0.0;
          for (int i = 0; i < niq; ++i) s += std::log(z_trial[i]);
          log_zt_last = s;
        }
        filter_history.push_back({feas_trial, obj_trial, log_zt_last});
        break;
      }

      if (!soc_attempted && bt == 0 && kkt_form == KktForm::Augmented &&
          feas_trial > feascond && meq > 0) {
        soc_attempted = true;
        Eigen::VectorXd rhs_soc = rhs_corr_aug;
        rhs_soc.segment(n, meq) = -(alpha_p * rg + rg_trial);
        Eigen::VectorXd dx_soc;
        Eigen::VectorXd dlambda_soc;
        Eigen::VectorXd dmu_soc;
        ++out.linear_solve_calls;
        if (kkt_solve_sparse_augmented(sparse_cache, rhs_soc,
                                       dx_soc, dlambda_soc, dmu_soc) &&
            dx_soc.allFinite()) {
          x_trial = x + alpha_p * dx + dx_soc;
          z_trial = (z + alpha_p * dz - dh * dx_soc).cwiseMax(1e-12);
          mu_trial = (mu + alpha_d * dmu + dmu_soc).cwiseMax(1e-12);
          lambda_trial = lambda + alpha_d * dlambda + dlambda_soc;
          if (x_trial.allFinite() && z_trial.allFinite() &&
              mu_trial.allFinite() && lambda_trial.allFinite() &&
              eval_and_test_trial(bt)) {
            step_accepted = true;
            ++out.accepted_steps;
            if (!log_zt_valid) {
              double s = 0.0;
              for (int i = 0; i < niq; ++i) s += std::log(z_trial[i]);
              log_zt_last = s;
            }
            filter_history.push_back({feas_trial, obj_trial, log_zt_last});
            break;
          }
        }
      }

        ++out.rejected_steps;
        alpha_p *= kBacktrack;
        alpha_d *= kBacktrack;
      }
    };
    run_backtracks();

    // Centrality-recovery retry (Gondzio–Grothey): a warm point off the
    // central path makes the economic corrector's steps transiently raise φ,
    // so the filter rejects every one of them even though feasibility is not
    // the blocker (observed: 639 iterations with zero normal accepts on the
    // PEGASE continuation).  Re-solve the corrector as a pure centering step
    // (γ = μ_cur, cross = 0) — pulling z/μ back toward the path — and run
    // the line search once more before falling to restoration.
    if (!step_accepted && kkt_form == KktForm::Augmented && bt0_feas_ok) {
      gamma_corr = std::max(gamma_corr, mu_cur);
      rhs_corr_aug.head(n) = -Lx;
      rhs_corr_aug.segment(n, meq) = -rg;
      for (int i = 0; i < niq; ++i) {
        rhs_corr_aug[n + meq + i] = -(rh[i] + gamma_corr / mu_safe[i]);
      }
      ++out.linear_solve_calls;
      if (kkt_solve_sparse_augmented(sparse_cache, rhs_corr_aug, dx, dlambda, dmu) &&
          dx.allFinite() && dlambda.allFinite()) {
        dz = -(rh + z_safe) - dh * dx;
        alpha_p = std::max(std::min(ftb(z_safe, dz, tau), 1.0), 1e-10);
        alpha_d = std::max(std::min(ftb(mu_safe, dmu, tau), 1.0), 1e-10);
        if (std::min(alpha_p, alpha_d) >= 1e-7) {
          run_backtracks();
        }
      }
    }

    if (!step_accepted && kkt_form == KktForm::Augmented) {
      // Restoration phase (Wächter–Biegler §3.3, bounded form): the filter
      // rejected every trial, so the current Newton direction is unusable for
      // the barrier problem.  Take one feasibility step with the (1,1) block
      // replaced by ρI — a safely positive-definite Gauss-Newton-like step on
      // ½‖θ‖² reusing the same augmented KKT machinery — and accept the first
      // trial that decreases θ without wrecking the dual axes.  The next main
      // iteration re-factors with the true Lxx; repeated filter failures give
      // repeated restoration steps, which is the bounded analogue of Ipopt's
      // restoration mode.
      const double lxx_norm_r = std::max(1.0, Lxx.coeffs().cwiseAbs().maxCoeff());
      Eigen::SparseMatrix<double> rho_i(n, n);
      rho_i.setIdentity();
      rho_i *= 1e-8 * lxx_norm_r;
      ++out.factorization_calls;
      if (factor_kkt_sparse_augmented(sparse_cache, rho_i, jg, dh,
                                      z_safe, mu_safe, 0.0, 1e-8 * lxx_norm_r)) {
        Eigen::VectorXd rhs_r = Eigen::VectorXd::Zero(n + meq + niq);
        rhs_r.segment(n, meq) = -rg;
        for (int i = 0; i < niq; ++i) {
          rhs_r[n + meq + i] = -(rh[i] + gamma_corr / mu_safe[i]);
        }
        Eigen::VectorXd dx_r;
        Eigen::VectorXd dlambda_r;
        Eigen::VectorXd dmu_r;
        ++out.linear_solve_calls;
        if (kkt_solve_sparse_augmented(sparse_cache, rhs_r, dx_r, dlambda_r, dmu_r) &&
            dx_r.allFinite()) {
          const Eigen::VectorXd dz_r = -(rh + z_safe) - dh * dx_r;
          double alpha_r = 1.0;
          for (int bt2 = 0; bt2 <= 8; ++bt2) {
            x_trial = x + alpha_r * dx_r;
            z_trial = (z + alpha_r * dz_r).cwiseMax(1e-12);
            mu_trial = (mu + alpha_r * dmu_r).cwiseMax(1e-12);
            lambda_trial = lambda + alpha_r * dlambda_r;
            if (x_trial.allFinite() && z_trial.allFinite() &&
                mu_trial.allFinite() && lambda_trial.allFinite()) {
              eval_and_test_trial(-1);  // for the trial state, not its verdict
              const bool theta_progress =
                  feas_trial <= (1.0 - 1e-4 * alpha_r) * feascond;
              const bool bounded =
                  grad_trial_cond <= 100.0 * std::max(gradcond, 1e-12) &&
                  comp_trial <= 100.0 * std::max(compcond, 1e-12);
              if (theta_progress && bounded) {
                step_accepted = true;
                ++out.accepted_steps;
                break;
              }
            }
            alpha_r *= 0.5;
          }
        }
      }
    }

    if (!step_accepted) {
      out.status = "filter line search failed";
      break;
    }

    x = std::move(x_trial);
    z = std::move(z_trial);
    mu = std::move(mu_trial);
    lambda = std::move(lambda_trial);
    rg = std::move(rg_trial);
    rh = std::move(rh_trial);
    grad = std::move(grad_trial);
    hdiag = std::move(hdiag_trial);
    Lx = std::move(Lx_trial);
    jg = std::move(jg_trial);
    dh = std::move(dh_trial);
    obj = obj_trial;

    // Rebuild curvature at the accepted point.
    lagrangian_hessian(prob, x, lambda, get_nu_ptr(mu), Lxx, 1e-12);
    apply_obj_scale_hessian(Lxx, hdiag);

    feascond = feas_trial;
    raw_feascond = raw_feas_trial;
    gradcond = grad_trial_cond;
    compcond = comp_trial;
    costcond = cost_trial;

    // Track best iterate
    const double cur_metric = std::max({raw_feascond, gradcond, compcond});
    if (cur_metric < best_metric) {
      best_metric = cur_metric;
      best_x = x;
      best_lambda = lambda;
      best_mu = mu;
      best_z = z;
      best_feas = feascond;
      best_raw_feas = raw_feascond;
      best_grad = gradcond;
      best_comp = compcond;
      best_iter = iter + 1;
    }

    // Objective-stagnation early stop: hard trajectories reach their endpoint
    // long before any tolerance test fires — the tail is pure restoration
    // crawl whose steps carry zero stationarity rhs (docs §9.5/§12), so the
    // dual residual can never improve there.  Once feasibility and
    // complementarity are met and the objective has stagnated over a window
    // of accepted iterations, stop and let the dual least-squares polish
    // (below) certify the point.  The feasibility gate keeps this from
    // cutting a healthy midgame tail, where obj also stagnates while θ is
    // still being reduced.
    constexpr int kStagnationWindow = 20;
    if (raw_feascond < 1e-4 && compcond < comp_tol) {
      const double d_obj = std::abs(obj - obj_prev);
      stagnant_count = (d_obj <= 1e-6 * (1.0 + std::abs(obj))) ? stagnant_count + 1 : 0;
      if (stagnant_count >= kStagnationWindow) {
        out.status = "objective-stagnant (dual-polish certification pending)";
        break;
      }
    } else {
      stagnant_count = 0;
    }
    obj_prev = obj;

    if (feascond < feas_tol && raw_feascond < feas_tol &&
        gradcond < feas_tol && compcond < comp_tol) {
      converged = true;
      out.converged = true;
      out.status = "converged";
      out.iterations = iter + 1;
    }
  }

  // Acceptable-convergence termination (Ipopt's Solved_To_Acceptable_Level
  // convention): when the strict tolerance is out of reach — typically
  // because the filter stalls at the endgame conditioning limit — accept
  // the best iterate when it satisfies a relaxed 100× primal/dual tolerance
  // with complementarity already inside comp_tol.  The status string states
  // the relaxation honestly; this is NOT the strict tolerance.
  const double acceptable_tol = 100.0 * feas_tol;
  const double acceptable_raw_feas_tol = 100.0 * feas_tol;
  if (!converged && opt.verbose) {
    HACDCPF_LOG_DEBUG("[parity-ipm] best iterate at iter={} feas={} grad={} comp={} metric={}",
              best_iter, best_feas, best_grad, best_comp, best_metric);
  }
  const bool current_acceptable =
      feascond < acceptable_tol && raw_feascond < acceptable_raw_feas_tol &&
      gradcond < acceptable_tol && compcond < comp_tol;
  const bool best_acceptable =
      best_feas < acceptable_tol && best_raw_feas < acceptable_raw_feas_tol &&
      best_grad < acceptable_tol && best_comp < comp_tol;
  if (!converged && (current_acceptable || best_acceptable)) {
    const bool use_best = best_acceptable &&
                          (!current_acceptable || best_raw_feas < raw_feascond);
    if (use_best) {
      x = best_x;
      lambda = best_lambda;
      mu = best_mu;
      z = best_z;
      feascond = best_feas;
      raw_feascond = best_raw_feas;
      gradcond = best_grad;
      compcond = best_comp;
      out.iterations = best_iter;
    }
    converged = true;
    out.converged = true;
    out.status = use_best ? "converged (acceptable tolerance, best iterate)"
                          : "converged (acceptable tolerance, final iterate)";
  }

  // Dual least-squares polish (docs §12): re-estimate (λ, μ) at the final
  // point from the exact stationarity system — one KKT solve with (1,1) = I
  // and a zero (3,3) block:
  //   [I  Jgᵀ  Jhᵀ][r ]   [−f̃∇f]
  //   [Jg  0   0  ][λ ] = [ 0   ]
  //   [Jh  0   0  ][μ ]   [ 0   ]
  // which is the KKT system of min_{λ,μ} ½‖f̃∇f + Jgᵀλ + Jhᵀμ‖².  Inactive-row
  // multipliers are clipped to μ ≥ 0 (standard practice).  When the endpoint
  // is feasible and the polished stationarity meets the acceptable tolerance,
  // the point is a certified KKT point — this certifies endpoints whose
  // trajectory stalled at the dual-residual floor (e.g. the PEGASE Phase-II
  // and continuation steps, where restoration steps carry zero stationarity
  // rhs and can never improve the dual residual).
  if (niq > 0 && !use_dense) {
    Eigen::SparseMatrix<double> eye(n, n);
    eye.setIdentity();
    SparseKKTCache ls_cache;
    const Eigen::VectorXd z_zero = Eigen::VectorXd::Zero(niq);
    const Eigen::VectorXd mu_one = Eigen::VectorXd::Ones(niq);
    Eigen::VectorXd rhs_ls(n + meq + niq);
    rhs_ls.head(n) = -obj_scale * grad;
    rhs_ls.segment(n, meq).setZero();
    rhs_ls.tail(niq).setZero();
    Eigen::VectorXd r_stat;
    Eigen::VectorXd lambda_ls;
    Eigen::VectorXd mu_ls;
    if (factor_kkt_sparse_augmented(ls_cache, eye, jg, dh,
                                    z_zero, mu_one, 0.0, 1e-8) &&
        kkt_solve_sparse_augmented(ls_cache, rhs_ls, r_stat, lambda_ls, mu_ls) &&
        lambda_ls.allFinite() && mu_ls.allFinite()) {
      const Eigen::VectorXd mu_clip = mu_ls.cwiseMax(0.0);
      const Eigen::VectorXd stat_vec =
          obj_scale * grad + jg.transpose() * lambda_ls + dh.transpose() * mu_clip;
      const double stat_ls = inf_norm(stat_vec) /
          (1.0 + std::max(inf_norm(mu_clip), inf_norm(lambda_ls)));
      // Adopt the polished multipliers when they explain stationarity at
      // least as well as the trajectory's own duals.
      if (stat_ls <= std::max(gradcond, 1e-12) || !converged) {
        lambda = lambda_ls;
        mu = mu_clip;
        gradcond = stat_ls;
      }
      if (!converged && feascond < acceptable_tol &&
          raw_feascond < acceptable_raw_feas_tol &&
          compcond < comp_tol && stat_ls < acceptable_tol) {
        converged = true;
        out.converged = true;
        out.status = "converged (dual least-squares certified)";
      }
    }
  }

  out.x = x;
  out.lambda_eq = lambda;
  out.mu = mu;
  out.z = z;
  out.primal_inf = feascond;
  out.dual_inf = gradcond;
  out.complementarity = compcond;
  if (use_dense) {
    out.linear_solver = "dense_lu";
  } else {
    switch (sparse_cache.active) {
      case 1: out.linear_solver = "sparse_umfpack"; break;
      case 2: out.linear_solver = "sparse_klu"; break;
      case 3: out.linear_solver = "sparse_eigen_lu"; break;
      default: out.linear_solver = "sparse"; break;
    }
  }
  out.symbolic_analyze_calls =
      sparse_cache.symbolic_analyze_calls - symbolic_analyze_calls_before;
  out.symbolic_reused = opt.prepared_state != nullptr &&
                        had_prepared_symbolic &&
                        out.factorization_calls > 0 &&
                        out.symbolic_analyze_calls == 0;
  out.backend_escalations =
      sparse_cache.backend_escalations - backend_escalations_before;
  out.scaling_rebuilds = sparse_cache.scaling_rebuilds - scaling_rebuilds_before;
  if (opt.prepared_numeric_refactor) {
    // Eigen's SuiteSparse adapters and the current MUMPS wrapper expose a fresh
    // numeric factorize after symbolic analysis, but no distinct cross-solve
    // factor object whose drift can be audited before use. Fail closed rather
    // than labelling ordinary numeric factorization as reuse.
    out.numeric_refactor_status =
        "unsupported: backend recomputes numeric factors after symbolic reuse";
  } else {
    out.numeric_refactor_status = "not-requested";
  }
  return out;
}

// Davidenko tangent of the objective homotopy — see the header doc and
// docs/numerical_methods.md §13.  One augmented-KKT solve with the same
// LDLᵀ machinery (inertia-controlled) the IPM uses per iteration.
bool homotopy_tangent(const Problem& prob,
                      const Eigen::VectorXd& x,
                      const Eigen::VectorXd& z,
                      const Eigen::VectorXd& lambda,
                      const Eigen::VectorXd& mu,
                      double t,
                      Eigen::VectorXd& dx_dt,
                      Eigen::VectorXd& dz_dt,
                      Eigen::VectorXd& dlambda_dt,
                      Eigen::VectorXd& dmu_dt) {
  if (t <= 1e-12) return false;
  const int n = prob.vidx.n_total;
  Eigen::VectorXd xmin;
  Eigen::VectorXd xmax;
  build_variable_bounds(prob, xmin, xmax);
  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  for (int i = 0; i < n; ++i) {
    if (std::isfinite(xmin[i])) lb_cols.push_back(i);
    if (std::isfinite(xmax[i])) ub_cols.push_back(i);
  }

  Eigen::VectorXd grad;
  Eigen::VectorXd hdiag;
  objective_gradient_hessian_diag(prob, x, grad, hdiag);

  EvalWorkspace ws;
  Eigen::SparseMatrix<double> jg;
  Eigen::SparseMatrix<double> dh;
  Eigen::VectorXd rg;
  Eigen::VectorXd rh;
  // equality_jacobian reads ws.p_calc/q_calc — prime the workspace first
  // (it documents "must already be populated by a prior equality_constraints
  // call"; a fresh workspace otherwise segfaults).
  equality_constraints(prob, x, ws, rg);
  equality_jacobian(prob, x, ws, jg);
  assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols, rh, dh);
  const int meq = static_cast<int>(jg.rows());
  const int niq = static_cast<int>(dh.rows());
  // A tangent is optional continuation metadata.  A solver result from an
  // early/failed path may legitimately omit one of the primal-dual blocks;
  // never turn that into an Eigen block assertion while solving the OPF.
  if (x.size() != n || lambda.size() != meq || z.size() != niq ||
      mu.size() != niq || niq == 0) {
    return false;
  }

  // Lagrangian Hessian of the t-scaled problem (the path curvature at w(t)).
  const int m_nonlin = prob.cidx.n_ineq_nonlin;
  Eigen::VectorXd nu_buf;
  const Eigen::VectorXd* nu_ptr = nullptr;
  if (m_nonlin > mu.size()) return false;
  if (m_nonlin > 0) {
    nu_buf = mu.head(m_nonlin);
    nu_ptr = &nu_buf;
  }
  Eigen::SparseMatrix<double> Lxx;
  lagrangian_hessian(prob, x, lambda, nu_ptr, Lxx, 1e-12);

  const Eigen::VectorXd z_safe = z.cwiseMax(1e-12);
  const Eigen::VectorXd mu_safe = mu.cwiseMax(1e-12);

  // Assemble + factor with Wächter–Biegler inertia control (mirrors the IPM).
  SparseKKTCache cache;
  cache.pure_ac = prob.data.dc_buses.empty() &&
                  prob.data.dc_branches.empty() &&
                  prob.data.converters.empty() &&
                  prob.data.dcdc_converters.empty() &&
                  prob.data.energy_routers.empty();
  const double lxx_norm = std::max(1.0, Lxx.coeffs().cwiseAbs().maxCoeff());
  const double delta_c = 1e-8 * lxx_norm;
  double delta_w = 0.0;
  bool factored = false;
  for (int attempt = 0; attempt < 8; ++attempt) {
    if (!factor_kkt_sparse_augmented(cache, Lxx, jg, dh, z_safe, mu_safe,
                                     delta_w, delta_c)) {
      break;
    }
    int negs = -1;
#if defined(HACDCPF_HAVE_MUMPS)
    if (cache.active == 4) negs = cache.mumps.negative_eigenvalues();
#endif
    if (negs < 0 || negs == meq + niq) {
      factored = true;
      break;
    }
    delta_w = (delta_w == 0.0) ? 1e-8 * lxx_norm
                               : std::min(8.0 * delta_w, 1e-2 * lxx_norm);
    if (attempt == 7) factored = true;
  }
  if (!factored) return false;

  // rhs = (−∇f/t, 0, 0): ∂/∂t of the stationarity is the full cost gradient.
  Eigen::VectorXd rhs(n + meq + niq);
  rhs.head(n) = -(grad / t);
  rhs.segment(n, meq).setZero();
  rhs.tail(niq).setZero();
  if (!kkt_solve_sparse_augmented(cache, rhs, dx_dt, dlambda_dt, dmu_dt) ||
      !dx_dt.allFinite()) {
    return false;
  }
  dz_dt = -(dh * dx_dt);
  return dz_dt.allFinite() && dlambda_dt.allFinite() && dmu_dt.allFinite();
}

}  // namespace hacdcpf::opf::parity
