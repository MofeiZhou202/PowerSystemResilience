/// conic_benchmark.cpp
///
/// Benchmark driver for the native conic interior-point solver
/// (mipsolvers::engine::ConicIPMSolver) on synthetic sparse LP, SOCP and SDP
/// suites.
///
/// The SOCP/SDP cases use a planted self-dual construction (standard
/// benchmark trick): pick a random primal point x0, random equality
/// multipliers y0
/// and strictly interior, well-centered cone points s0, z0 (every cone
/// "eigenvalue" comfortably away from 0), then set
///
///     h = G x0 + s0,   b = A x0,   c = -G'z0 - A'y0.
///
/// (x0, s0) is then exactly primal feasible and (y0, z0) exactly dual
/// feasible, so both Slater conditions hold by construction and the planted
/// pair sits in the central-path neighbourhood with duality gap s0'z0; a
/// Mehrotra predictor-corrector IPM converges from there in a handful of
/// iterations.  Keeping s0/z0 well centered (SOC eigenvalues in [1, 2.5],
/// SDP eigenvalues >= 0.75) makes the default tolerances bite in well under
/// 60 iterations on every case.
///
/// Two construction choices are workarounds for known solver weaknesses
/// (observed 2026-07, ConicIPMSolver via CHOLMOD + MUMPS fallback):
///  - Equality rows are OFF by default (--with-equalities opts in).  With
///    equalities present the reduced KKT is indefinite and used to fall back
///    to MUMPS; since the corrector-division fix and the simplicial-LDLT
///    KKT path (see docs/conic_sdp.md §6.4, §7.3), these cases converge in
///    5-11 iterations like the no-equality ones.  The flag keeps a
///    dedicated reproducer/regression for that path.
///  - SDP G gets >= 5 nonzeros per column (SOCP keeps ~1% density).  With
///    ~1 nnz/col the Schur complement G'H^{-1}G is too ill-conditioned and
///    the IPM stalls at gap ~2e-2 with steps hitting the cone boundary,
///    then diverges (p = 10 draw with seed 42).  A per-column floor keeps
///    the Newton direction accurate enough to converge in ~5 iterations.
///
/// Per (case, threads) the driver reports status, iterations, wall time and
/// duality gap, plus a thread-scaling summary (speedup vs 1 thread).  The
/// thread count is passed through ConicIPMOptions::num_threads — the solver
/// applies it to its OpenMP regions for the duration of solve(), so no
/// omp_set_num_threads is needed here.  Exit code is non-zero unless every
/// run reports status "optimal", so the driver doubles as a smoke test.
///
/// Usage:
///   conic_benchmark                          # all suites, full sizes
///   conic_benchmark --quick                  # small sizes only
///   conic_benchmark --suite socp             # socp | sdp | all
///   conic_benchmark --suite chordal_sdp      # sparse path-graph SDP
///   conic_benchmark --suite sparse_lp        # 100k and 1m structured cases
///   conic_benchmark --threads-list 1,2,4,8   # thread counts to sweep
///   conic_benchmark --with-equalities        # add m_eq = n/10 equality rows
///   conic_benchmark --json out.json          # write JSON records

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <nlohmann/json.hpp>

#include "mipsolvers/engine/kernel/ipm/cones.hpp"
#include "mipsolvers/engine/kernel/ipm/conic_ipm_solver.hpp"

using json = nlohmann::json;
using mipsolvers::engine::ConeDims;
using mipsolvers::engine::ConicIPMOptions;
using mipsolvers::engine::ConicIPMResult;
using mipsolvers::engine::ConicIPMSolver;
using mipsolvers::engine::ConicModel;

namespace {

// Mirrors the library's PRIVATE OpenMP define (exposed to this target in
// CMakeLists.txt exactly like for test_conic_ipm).  Only used to pick the
// default --threads-list and to annotate the banner; thread control itself
// goes through ConicIPMOptions::num_threads.
#if defined(MIPSOLVERS_USE_OPENMP)
constexpr bool kOpenMP = true;
#else
constexpr bool kOpenMP = false;
#endif

struct BenchCase {
  std::string suite;  ///< "sparse_lp" | "socp" | "sdp" | "chordal_sdp".
  std::string name;
  ConicModel model;
};

struct RunRecord {
  std::string suite;
  std::string problem;
  int n{0};           ///< Variables.
  int m{0};           ///< Conic rows (dims.total()).
  int m_eq{0};        ///< Equality rows.
  long nnz{0};        ///< nnz(G) + nnz(A).
  int threads{1};
  int iterations{0};
  int chordal_cliques{0};
  int chordal_max_order{0};
  double time_s{0.0};
  double gap{0.0};         ///< Absolute duality gap s'z.
  double rel_gap{0.0};     ///< Solver-reported relative gap.
  double primal_objective{0.0};
  std::string backend;
  int kkt_dimension{0};
  int kkt_nonzeros{0};
  double kkt_symbolic_flops{0.0};
  double kkt_symbolic_nonzeros{0.0};
  int kkt_factorizations{0};
  int kkt_linear_solves{0};
  int kkt_refinements{0};
  double max_initial_backward_error{0.0};
  double max_final_backward_error{0.0};
  std::string status;
};

// ─────────────────────────────────────────────────────────────────────────────
// Random model ingredients (fixed seed => fully reproducible cases)
// ─────────────────────────────────────────────────────────────────────────────

/// Random sparse rows x cols matrix with ~density fraction of N(0,1)
/// nonzeros; at least max(1, min_per_col) nonzeros per column and one per
/// row so no variable or constraint vanishes from the model.
Eigen::SparseMatrix<double> random_sparse(int rows, int cols, double density,
                                          int min_per_col,
                                          std::mt19937_64& rng) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  const int per_col =
      std::max(min_per_col,
               std::max(1, static_cast<int>(std::lround(density * rows))));
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(cols) * static_cast<size_t>(per_col) +
                   static_cast<size_t>(rows));
  std::vector<int> row_count(static_cast<size_t>(rows), 0);
  std::vector<int> perm(static_cast<size_t>(rows));
  std::iota(perm.begin(), perm.end(), 0);
  for (int j = 0; j < cols; ++j) {
    // Partial Fisher-Yates draw of per_col distinct rows.  perm is a running
    // permutation across columns; only within-column distinctness matters.
    for (int k = 0; k < per_col; ++k) {
      std::uniform_int_distribution<int> pick(k, rows - 1);
      const int idx = pick(rng);
      std::swap(perm[static_cast<size_t>(k)], perm[static_cast<size_t>(idx)]);
      const int i = perm[static_cast<size_t>(k)];
      triplets.emplace_back(i, j, gauss(rng));
      ++row_count[static_cast<size_t>(i)];
    }
  }
  std::uniform_int_distribution<int> col_pick(0, cols - 1);
  for (int i = 0; i < rows; ++i) {
    if (row_count[static_cast<size_t>(i)] == 0) {
      triplets.emplace_back(i, col_pick(rng), gauss(rng));
    }
  }
  Eigen::SparseMatrix<double> mat(rows, cols);
  mat.setFromTriplets(triplets.begin(), triplets.end());
  mat.makeCompressed();
  return mat;
}

/// Strictly interior point of R^l_+: entries in [0.75, 1.75].
Eigen::VectorXd orthant_interior(int l, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  Eigen::VectorXd s(l);
  for (int i = 0; i < l; ++i) s[i] = 0.75 + uni(rng);
  return s;
}

/// Strictly interior point of the Lorentz cone Q^k: t in [1.5, 2.0],
/// ||v|| <= 0.5, so both eigenvalues t +/- ||v|| lie in [1.0, 2.5].
Eigen::VectorXd soc_interior(int k, std::mt19937_64& rng) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  Eigen::VectorXd s(k);
  s[0] = 1.5 + 0.5 * uni(rng);
  Eigen::VectorXd v(k - 1);
  for (int i = 0; i < k - 1; ++i) v[i] = gauss(rng);
  const double nrm = v.norm();
  if (nrm > 0.0) v /= nrm;
  v *= 0.5 * uni(rng);
  s.tail(k - 1) = v;
  return s;
}

/// Well-centered SPD matrix of order p: S = M M' + 0.75 I with
/// M_ij ~ N(0, 1/p), hence lambda_min(S) >= 0.75 for every draw.
Eigen::MatrixXd spd_interior(int p, std::mt19937_64& rng) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  const double scale = 1.0 / std::sqrt(static_cast<double>(p));
  Eigen::MatrixXd m(p, p);
  for (int j = 0; j < p; ++j)
    for (int i = 0; i < p; ++i) m(i, j) = scale * gauss(rng);
  Eigen::MatrixXd s = m * m.transpose();
  s.diagonal().array() += 0.75;
  return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Planted self-dual case construction (see file header for the trick)
// ─────────────────────────────────────────────────────────────────────────────

/// Given G, A, dims and interior s0/z0, draw x0/y0 and derive h, b, c so the
/// planted pair (x0, s0) / (y0, z0) is exactly primal/dual feasible.  A may
/// have zero rows (no equalities).
ConicModel plant_model(const Eigen::SparseMatrix<double>& G,
                       const Eigen::SparseMatrix<double>& A,
                       const ConeDims& dims, const Eigen::VectorXd& s0,
                       const Eigen::VectorXd& z0, std::mt19937_64& rng) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  const int n = static_cast<int>(G.cols());
  const int meq = static_cast<int>(A.rows());
  Eigen::VectorXd x0(n);
  for (int j = 0; j < n; ++j) x0[j] = gauss(rng);
  Eigen::VectorXd y0(meq);
  for (int i = 0; i < meq; ++i) y0[i] = gauss(rng);

  ConicModel cm;
  cm.G = G;
  cm.A = A;
  cm.dims = dims;
  cm.h = G * x0 + s0;  // (x0, s0) exactly primal feasible
  cm.b = A * x0;       // equality-feasible by construction
  cm.c = -(G.transpose() * z0) - (A.transpose() * y0);  // (y0, z0) dual feasible
  return cm;
}

/// Empty 0 x n equality matrix (equalities are opt-in, see file header).
Eigen::SparseMatrix<double> no_equalities(int n) {
  Eigen::SparseMatrix<double> A(0, n);
  A.makeCompressed();
  return A;
}

/// Large structured orthant case with G = -I.  Its KKT matrix is diagonal,
/// so this isolates the solver's linear-memory path from fill-in effects and
/// provides a reproducible million-variable scalability check.
BenchCase make_sparse_lp_case(int n) {
  Eigen::SparseMatrix<double> G(n, n);
  G.setIdentity();
  G *= -1.0;
  G.makeCompressed();

  ConeDims dims;
  dims.l = n;
  ConicModel cm;
  cm.G = std::move(G);
  cm.A = no_equalities(n);
  cm.dims = dims;
  cm.h = Eigen::VectorXd::Ones(n);
  cm.c = Eigen::VectorXd::Ones(n);
  cm.b.resize(0);

  BenchCase bc;
  bc.suite = "sparse_lp";
  bc.name = "sparse_lp_n" + std::to_string(n);
  bc.model = std::move(cm);
  return bc;
}

/// SOCP suite: n variables, l = n/2 nonnegative rows, 10 SOC blocks of
/// size 10, G ~1% dense; with equalities, m_eq = n/10 rows of A (~1% dense).
BenchCase make_socp_case(int n, bool with_eq) {
  std::mt19937_64 rng(42);  // fixed seed: reproducible cases
  const int l = n / 2;
  constexpr int kNumSoc = 10;
  constexpr int kSocSize = 10;
  const int m = l + kNumSoc * kSocSize;
  const int meq = with_eq ? n / 10 : 0;

  ConeDims dims;
  dims.l = l;
  dims.q.assign(kNumSoc, kSocSize);

  const Eigen::SparseMatrix<double> G = random_sparse(m, n, 0.01, 1, rng);
  const Eigen::SparseMatrix<double> A =
      (meq > 0) ? random_sparse(meq, n, 0.01, 1, rng) : no_equalities(n);

  Eigen::VectorXd s0(m), z0(m);
  s0.head(l) = orthant_interior(l, rng);
  z0.head(l) = orthant_interior(l, rng);
  for (int q = 0; q < kNumSoc; ++q) {
    s0.segment(l + q * kSocSize, kSocSize) = soc_interior(kSocSize, rng);
    z0.segment(l + q * kSocSize, kSocSize) = soc_interior(kSocSize, rng);
  }

  BenchCase bc;
  bc.suite = "socp";
  bc.name = "socp_n" + std::to_string(n);
  bc.model = plant_model(G, A, dims, s0, z0, rng);
  return bc;
}

/// SDP suite: one PSD block of order p, n = 2 * p(p+1)/2 columns; G ~1%
/// dense with a floor of 5 nonzeros per column (see file header); with
/// equalities, m_eq = n/10 rows of A.
BenchCase make_sdp_case(int p, bool with_eq) {
  std::mt19937_64 rng(42);
  const int packed = mipsolvers::engine::svec_size(p);
  const int n = 2 * packed;
  const int meq = with_eq ? n / 10 : 0;

  ConeDims dims;
  dims.s = {p};

  const Eigen::SparseMatrix<double> G = random_sparse(packed, n, 0.01, 5, rng);
  const Eigen::SparseMatrix<double> A =
      (meq > 0) ? random_sparse(meq, n, 0.01, 1, rng) : no_equalities(n);

  const Eigen::VectorXd s0 = mipsolvers::engine::svec(spd_interior(p, rng));
  const Eigen::VectorXd z0 = mipsolvers::engine::svec(spd_interior(p, rng));

  BenchCase bc;
  bc.suite = "sdp";
  bc.name = "sdp_p" + std::to_string(p);
  bc.model = plant_model(G, A, dims, s0, z0, rng);
  return bc;
}

/// Sparse tridiagonal SDP with a path aggregate graph.  The original order-p
/// block decomposes exactly into p-1 overlapping order-2 PSD cliques.
BenchCase make_chordal_sdp_case(int p) {
  std::mt19937_64 rng(42);
  std::normal_distribution<double> gauss(0.0, 1.0);
  const int packed = mipsolvers::engine::svec_size(p);
  const int n = 2 * p;
  std::vector<int> active_rows;
  active_rows.reserve(static_cast<std::size_t>(2 * p - 1));
  for (int i = 0; i < p; ++i) {
    active_rows.push_back(mipsolvers::engine::svec_index(i, i, p));
  }
  for (int i = 1; i < p; ++i) {
    active_rows.push_back(mipsolvers::engine::svec_index(i, i - 1, p));
  }
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(3 * n));
  for (int col = 0; col < n; ++col) {
    for (int k = 0; k < 3; ++k) {
      const int row = active_rows[static_cast<std::size_t>(
          (3 * col + k) % static_cast<int>(active_rows.size()))];
      triplets.emplace_back(row, col, gauss(rng));
    }
  }
  Eigen::SparseMatrix<double> G(packed, n);
  G.setFromTriplets(triplets.begin(), triplets.end());
  G.makeCompressed();

  Eigen::VectorXd s0 = Eigen::VectorXd::Zero(packed);
  Eigen::VectorXd z0 = Eigen::VectorXd::Zero(packed);
  for (int i = 0; i < p; ++i) {
    s0[mipsolvers::engine::svec_index(i, i, p)] = 2.0;
    z0[mipsolvers::engine::svec_index(i, i, p)] = 1.5;
  }
  for (int i = 1; i < p; ++i) {
    s0[mipsolvers::engine::svec_index(i, i - 1, p)] =
        std::sqrt(2.0) * 0.1;
    z0[mipsolvers::engine::svec_index(i, i - 1, p)] =
        std::sqrt(2.0) * 0.05;
  }
  ConeDims dims;
  dims.s = {p};
  BenchCase bc;
  bc.suite = "chordal_sdp";
  bc.name = "chordal_p" + std::to_string(p);
  bc.model = plant_model(G, no_equalities(n), dims, s0, z0, rng);
  return bc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Reporting
// ─────────────────────────────────────────────────────────────────────────────

void print_suite_table(const char* title, const std::string& suite,
                       const std::vector<RunRecord>& records) {
  std::printf("\n## %s suite\n\n", title);
  std::printf(
      "| %-10s | %6s | %6s | %5s | %7s | %3s | %5s | %8s | %9s | %s |\n",
      "problem", "n", "m", "m_eq", "nnz", "thr", "iters", "time_s", "gap",
      "status");
  std::printf(
      "|------------|-------:|-------:|------:|--------:|----:|------:|------"
      "---:|----------:|:--------|\n");
  for (const RunRecord& r : records) {
    if (r.suite != suite) continue;
    std::printf(
        "| %-10s | %6d | %6d | %5d | %7ld | %3d | %5d | %8.4f | %9.2e | %s "
        "|\n",
        r.problem.c_str(), r.n, r.m, r.m_eq, r.nnz, r.threads, r.iterations,
        r.time_s, r.gap, r.status.c_str());
  }
}

void print_scaling_summary(const std::vector<RunRecord>& records,
                           const std::vector<int>& threads_list) {
  std::printf("\n## Thread scaling (speedup vs 1 thread)\n\n");
  std::printf("| %-10s |", "problem");
  for (const int t : threads_list) std::printf(" %6d |", t);
  std::printf("\n|------------|");
  for (size_t i = 0; i < threads_list.size(); ++i) std::printf("-------:|");
  std::printf("\n");
  std::vector<std::string> seen;
  for (const RunRecord& r : records) {
    if (std::find(seen.begin(), seen.end(), r.problem) != seen.end()) {
      continue;
    }
    seen.push_back(r.problem);
    double t1 = 0.0;
    for (const RunRecord& q : records) {
      if (q.problem == r.problem && q.threads == 1) t1 = q.time_s;
    }
    std::printf("| %-10s |", r.problem.c_str());
    for (const int t : threads_list) {
      double tt = -1.0;
      for (const RunRecord& q : records) {
        if (q.problem == r.problem && q.threads == t) tt = q.time_s;
      }
      if (tt > 0.0 && t1 > 0.0) {
        std::printf(" %6.2f |", t1 / tt);
      } else {
        std::printf(" %6s |", "-");
      }
    }
    std::printf("\n");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────────────────────

void print_usage(const char* prog) {
  std::printf(
      "Usage: %s [--suite sparse_lp|socp|sdp|chordal_sdp|all] "
      "[--threads-list 1,2,4,8]\n"
      "          [--json path] [--quick] [--help]\n"
      "\n"
      "  --suite         Which case suite to run (default: all).\n"
      "  --threads-list  Comma-separated thread counts to sweep (default:\n"
      "                  1,2,4,8 with OpenMP, else 1).\n"
      "  --json          Write per-run records as JSON to <path>.\n"
      "  --with-equalities  Add m_eq = n/10 equality rows to exercise the\n"
      "                  indefinite-KKT path described in the header.\n"
      "  --verbose       Print the solver's per-iteration table.\n"
      "  --quick         Small sizes only (sparse LP n in {10k,100k}, "
      "socp n in {100,400}, sdp p in {10,20}, chordal p in {40,80}).\n",
      prog);
}

std::vector<int> parse_threads_list(const std::string& s) {
  std::vector<int> out;
  size_t pos = 0;
  while (pos <= s.size()) {
    const size_t comma = s.find(',', pos);
    const std::string tok =
        s.substr(pos, comma == std::string::npos ? comma : comma - pos);
    if (!tok.empty()) out.push_back(std::max(1, std::atoi(tok.c_str())));
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::string suite = "all";
  std::string json_path;
  bool quick = false;
  bool verbose = false;
  bool with_eq = false;  // equalities are opt-in; see file header
  std::vector<int> threads_list =
      kOpenMP ? std::vector<int>{1, 2, 4, 8} : std::vector<int>{1};

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--suite") == 0 && i + 1 < argc) {
      suite = argv[++i];
    } else if (std::strcmp(argv[i], "--threads-list") == 0 && i + 1 < argc) {
      threads_list = parse_threads_list(argv[++i]);
    } else if (std::strcmp(argv[i], "--json") == 0 && i + 1 < argc) {
      json_path = argv[++i];
    } else if (std::strcmp(argv[i], "--quick") == 0) {
      quick = true;
    } else if (std::strcmp(argv[i], "--with-equalities") == 0) {
      with_eq = true;
    } else if (std::strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
    } else if (std::strcmp(argv[i], "--help") == 0 ||
               std::strcmp(argv[i], "-h") == 0) {
      print_usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "Unknown argument: %s\n", argv[i]);
      print_usage(argv[0]);
      return 2;
    }
  }

  if (suite != "all" && suite != "sparse_lp" && suite != "socp" &&
      suite != "sdp" && suite != "chordal_sdp") {
    std::fprintf(stderr,
                 "Invalid --suite '%s' (expected sparse_lp|socp|sdp|"
                 "chordal_sdp|all)\n",
                 suite.c_str());
    return 2;
  }
  if (threads_list.empty()) {
    std::fprintf(stderr, "Empty --threads-list\n");
    return 2;
  }

  std::printf("# conic_benchmark — native conic IPM (LP/SOCP/SDP) suites\n");
  std::printf("# OpenMP: %s | hardware threads: %u | mode: %s | equalities: %s\n",
              kOpenMP ? "yes" : "no", std::thread::hardware_concurrency(),
              quick ? "quick" : "full", with_eq ? "on" : "off");
  std::printf("# threads-list:");
  for (const int t : threads_list) std::printf(" %d", t);
  std::printf("\n");
  if (with_eq) {
    std::printf(
        "# note: equality rows make the KKT indefinite (simplicial-LDLT"
        " path);\n# cases are expected to converge like the no-equality"
        " ones (see file header)\n");
  }
  if (!kOpenMP) {
    bool any_multi = false;
    for (const int t : threads_list) any_multi = any_multi || (t > 1);
    if (any_multi) {
      std::printf(
          "# note: library built without OpenMP; threads > 1 runs serial\n");
    }
  }

  std::vector<BenchCase> cases;
  // Deliberately opt-in: the million-variable case should not make the
  // historical "all" suite unexpectedly expensive.
  if (suite == "sparse_lp") {
    const std::vector<int> sizes = quick ? std::vector<int>{10'000, 100'000}
                                         : std::vector<int>{100'000, 1'000'000};
    for (const int n : sizes) cases.push_back(make_sparse_lp_case(n));
  }
  if (suite == "all" || suite == "socp") {
    const std::vector<int> sizes =
        quick ? std::vector<int>{100, 400} : std::vector<int>{100, 400, 1600};
    for (const int n : sizes) cases.push_back(make_socp_case(n, with_eq));
  }
  if (suite == "all" || suite == "sdp") {
    const std::vector<int> orders =
        quick ? std::vector<int>{10, 20} : std::vector<int>{10, 20, 40};
    for (const int p : orders) cases.push_back(make_sdp_case(p, with_eq));
  }
  if (suite == "chordal_sdp") {
    const std::vector<int> orders =
        quick ? std::vector<int>{40, 80} : std::vector<int>{80, 200};
    for (const int p : orders) cases.push_back(make_chordal_sdp_case(p));
  }

  std::vector<RunRecord> records;
  int failures = 0;
  for (const BenchCase& bc : cases) {
    const ConicModel& cm = bc.model;
    for (const int t : threads_list) {
      ConicIPMOptions opt;
      opt.num_threads = t;
      opt.verbose = verbose;
      const ConicIPMSolver solver(opt);
      const auto t0 = std::chrono::steady_clock::now();
      const ConicIPMResult res = solver.solve(cm);
      const double dt = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
      RunRecord rec;
      rec.suite = bc.suite;
      rec.problem = bc.name;
      rec.n = static_cast<int>(cm.c.size());
      rec.m = cm.dims.total();
      rec.m_eq = static_cast<int>(cm.A.rows());
      rec.nnz = cm.G.nonZeros() + cm.A.nonZeros();
      rec.threads = t;
      rec.iterations = res.iterations;
      rec.chordal_cliques = res.chordal_clique_count;
      rec.chordal_max_order = res.chordal_max_clique_order;
      rec.time_s = dt;
      rec.gap = res.gap;
      rec.rel_gap = res.relative_gap;
      rec.primal_objective = res.primal_objective;
      rec.backend = res.linear_solver_backend;
      rec.kkt_dimension = res.kkt_dimension;
      rec.kkt_nonzeros = res.kkt_nonzeros;
      rec.kkt_symbolic_flops = res.kkt_symbolic_flops;
      rec.kkt_symbolic_nonzeros = res.kkt_symbolic_nonzeros;
      rec.kkt_factorizations = res.kkt_factorizations;
      rec.kkt_linear_solves = res.kkt_linear_solves;
      rec.kkt_refinements = res.kkt_refinements;
      rec.max_initial_backward_error =
          res.max_initial_kkt_backward_error;
      rec.max_final_backward_error = res.max_final_kkt_backward_error;
      rec.status = res.status;
      records.push_back(rec);
      if (res.status != "optimal" ||
          (bc.suite == "chordal_sdp" && !res.chordal_decomposition_used)) {
        ++failures;
        std::printf("FAIL: %s threads=%d status=%s iters=%d\n",
                    bc.name.c_str(), t, res.status.c_str(), res.iterations);
      }
    }
  }

  if (suite == "sparse_lp") {
    print_suite_table("Sparse LP", "sparse_lp", records);
  }
  if (suite == "all" || suite == "socp") print_suite_table("SOCP", "socp", records);
  if (suite == "all" || suite == "sdp") print_suite_table("SDP", "sdp", records);
  if (suite == "chordal_sdp") {
    print_suite_table("Chordal SDP", "chordal_sdp", records);
    for (const RunRecord& r : records) {
      std::printf("# %s: cliques=%d max_order=%d\n", r.problem.c_str(),
                  r.chordal_cliques, r.chordal_max_order);
    }
  }
  print_scaling_summary(records, threads_list);

  if (!json_path.empty()) {
    json jout = json::array();
    for (const RunRecord& r : records) {
      jout.push_back({{"suite", r.suite},
                      {"problem", r.problem},
                      {"n", r.n},
                      {"m", r.m},
                      {"m_eq", r.m_eq},
                      {"nnz", r.nnz},
                      {"threads", r.threads},
                      {"iterations", r.iterations},
                      {"chordal_cliques", r.chordal_cliques},
                      {"chordal_max_order", r.chordal_max_order},
                      {"time_s", r.time_s},
                      {"gap", r.gap},
                      {"rel_gap", r.rel_gap},
                      {"primal_objective", r.primal_objective},
                      {"backend", r.backend},
                      {"kkt_dimension", r.kkt_dimension},
                      {"kkt_nonzeros", r.kkt_nonzeros},
                      {"kkt_symbolic_flops", r.kkt_symbolic_flops},
                      {"kkt_symbolic_nonzeros", r.kkt_symbolic_nonzeros},
                      {"kkt_factorizations", r.kkt_factorizations},
                      {"kkt_linear_solves", r.kkt_linear_solves},
                      {"kkt_refinements", r.kkt_refinements},
                      {"max_initial_kkt_backward_error",
                       r.max_initial_backward_error},
                      {"max_final_kkt_backward_error",
                       r.max_final_backward_error},
                      {"status", r.status}});
    }
    std::ofstream f(json_path);
    if (f) {
      f << jout.dump(2) << "\n";
      std::printf("\nResults written to %s\n", json_path.c_str());
    } else {
      std::fprintf(stderr, "Could not open %s for writing\n",
                   json_path.c_str());
      return 2;
    }
  }

  if (failures > 0) {
    std::printf("\n%d run(s) did not reach 'optimal'\n", failures);
    return 1;
  }
  std::printf("\nAll %zu runs optimal\n", records.size());
  return 0;
}
