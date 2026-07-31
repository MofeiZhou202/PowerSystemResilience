/// test_numerical_stability.cpp
/// Phase-1 numerical stability tests:
///   - Ruiz equilibration of the LP IPM (item 8b)
///   - O(nnz) row-norm sweep in compute_scaling_factors (item 8a)
///   - Conditional iterative refinement on the normal equations (item 9b)
///   - UMFPACK iterative refinement in the dual simplex basis (item 9a)
///   - Cached node-LP path consistency with scaling enabled (item 8b)
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_scaling.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

// 2-variable base LP:  min -3x - 2y  s.t.  x + y <= 6, 2x + y <= 8, x,y >= 0
// Optimum (2, 4), objective -14 (unique vertex: the objective slope -1.5 lies
// strictly between the two constraint slopes -1 and -2, so the optimal face
// is a single point).  Returned with diagonal row/col scalings applied
// (Ã = R·A·C, b̃ = R·b, c̃ = C·c), so the solver sees an ill-conditioned but
// equivalent problem: optimum x̂* = C⁻¹(2,4), obj -14.
LPModel make_scaled_lp(double r1, double r2, double c1, double c2) {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -3.0 * c1, -2.0 * c2;

  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = r1 * c1;
  A.insert(0, 1) = r1 * c2;
  A.insert(1, 0) = r2 * 2.0 * c1;
  A.insert(1, 1) = r2 * c2;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << r1 * 6.0, r2 * 8.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  return lp;
}

SimplexOptions native_simplex_options() {
  SimplexOptions options;
  options.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  return options;
}

}  // namespace

// ─── Ruiz rescues an ill-scaled LP ─────────────────────────────────────────
// Matrix entries span 1e-10..1e10: the unscaled normal equations are nearly
// singular.  With ruiz_rounds=10 the solve must recover the exact optimum.
TEST_CASE("Ruiz equilibration rescues an ill-scaled LP", "[numerical][ruiz]") {
  LPModel lp = make_scaled_lp(1e6, 1e-6, 1e-4, 1e4);
  // Scaled-problem optimum: x̂* = (2/1e-4, 4/1e4) = (2e4, 4e-4), obj = -14.

  IPMLPOptions opt;
  opt.ruiz_rounds = 10;
  NativeIPMLPAdapter solver(opt);
  const SolveResult res = solver.solve_lp(lp);
  REQUIRE(res.stats.success);
  CHECK(res.stats.objective == Approx(-14.0).margin(1e-4));
  REQUIRE(res.x.size() == 2);
  CHECK(res.x[0] == Approx(2.0e4).margin(2.0));
  CHECK(res.x[1] == Approx(4.0e-4).margin(1e-6));

  // Comparison run with scaling disabled (outcome reported, not asserted).
  IPMLPOptions off;
  off.ruiz_rounds = 0;
  NativeIPMLPAdapter raw(off);
  const SolveResult res0 = raw.solve_lp(lp);
  INFO("ruiz=0: success=" << res0.stats.success
       << " obj=" << res0.stats.objective
       << " iters=" << res0.stats.iterations
       << " | ruiz=10: iters=" << res.stats.iterations);
}

// ─── O(nnz) row-norm sweep regression (item 8a) ────────────────────────────
// The single-sweep row norms must match the direct per-row reference exactly.
TEST_CASE("compute_scaling_factors matches row-wise reference",
          "[numerical][scaling]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(2);
  nlp.jac_g = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& J) {
    J.resize(2, 2);
    J.insert(0, 0) = 3.0;
    J.insert(0, 1) = -200.0;  // row 0 norm = 200
    J.insert(1, 0) = 5.0;     // row 1 norm = 5
    J.makeCompressed();
  };
  nlp.jac_h = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& J) {
    J.resize(1, 2);
    J.insert(0, 1) = 4.0;  // row 0 norm = 4
    J.makeCompressed();
  };

  const ScalingFactors f = compute_scaling_factors(nlp, nlp.x0, 100.0);
  REQUIRE(f.s_g.size() == 2);
  REQUIRE(f.s_h.size() == 1);
  CHECK(f.s_g[0] == Approx(0.5));  // 100 / 200
  CHECK(f.s_g[1] == Approx(1.0));  // 5 <= g_max
  CHECK(f.s_h[0] == Approx(1.0));  // 4 <= g_max
  CHECK(f.s_f == Approx(1.0));     // no gradient callback
}

// ─── Normal-equation IR, banded path (item 9b) ─────────────────────────────
// Near-parallel equalities: cond(A) ~ 1/eps, cond(normal equations) ~ 1/eps².
// Unique solution x = (1,1,1).  eps = 1e-5 keeps cond(N) ~ 4e10 — ill-
// conditioned but comfortably within reach with iterative refinement.
TEST_CASE("IPM solves near-parallel equality LP accurately (banded IR)",
          "[numerical][ir]") {
  const double eps = 1e-5;
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Ones(3);
  lp.A.resize(0, 3);
  lp.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(3, 3);
  Aeq.insert(0, 0) = 1.0; Aeq.insert(0, 1) = 1.0;       Aeq.insert(0, 2) = 1.0;
  Aeq.insert(1, 0) = 1.0; Aeq.insert(1, 1) = 1.0 + eps; Aeq.insert(1, 2) = 1.0;
  Aeq.insert(2, 0) = 1.0; Aeq.insert(2, 1) = 1.0;       Aeq.insert(2, 2) = 1.0 + eps;
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq.resize(3);
  lp.beq << 3.0, 3.0 + eps, 3.0 + eps;

  for (int i = 0; i < 3; ++i)
    lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  NativeIPMLPAdapter solver{};  // default options (ruiz on, IR on)
  const SolveResult res = solver.solve_lp(lp);
  REQUIRE(res.stats.success);
  CHECK(res.stats.objective == Approx(3.0).margin(1e-5));
  CHECK(res.x[0] == Approx(1.0).margin(1e-4));
  CHECK(res.x[1] == Approx(1.0).margin(1e-4));
  CHECK(res.x[2] == Approx(1.0).margin(1e-4));
}

// ─── Normal-equation IR, sparse path (item 9b) ─────────────────────────────
// A long-range coupling entry pushes the bandwidth past kBandedThreshold,
// forcing the sparse (non-banded) normal-equations path.
TEST_CASE("IPM solves sparse-path LP accurately (sparse IR path)",
          "[numerical][ir]") {
  const int n = 300;
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Ones(n);
  lp.A.resize(0, n);
  lp.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(n, n);
  for (int i = 0; i < n; ++i) Aeq.insert(i, i) = 1.0;
  Aeq.insert(n - 1, 0) = 1.0;  // row n-1: x0 + x_{n-1} = 2, bandwidth n-1
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq = Eigen::VectorXd::Ones(n);
  lp.beq[n - 1] = 2.0;

  for (int i = 0; i < n; ++i)
    lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  NativeIPMLPAdapter solver{};
  const SolveResult res = solver.solve_lp(lp);
  REQUIRE(res.stats.success);
  CHECK(res.stats.objective == Approx(static_cast<double>(n)).margin(1e-4 * n));
  for (int i = 0; i < n; ++i)
    CHECK(res.x[i] == Approx(1.0).margin(1e-4));
}

// ─── UMFPACK iterative refinement in the dual simplex basis (item 9a) ──────
TEST_CASE("Dual simplex stays accurate on ill-conditioned basis",
          "[numerical][ir][dual_simplex]") {
  // x1 + x2 = 2 ; x1 + (1+eps)x2 = 2+eps ; x >= 0, min x1+x2.
  // Unique optimum (1,1) obj 2; the optimal basis is nearly singular.
  // eps = 1e-6 keeps the alternative vertex (2,0) clearly infeasible
  // (violation eps > feasibility tolerance) while stressing basis solves.
  const double eps = 1e-6;
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 1.0, 1.0;
  lp.A.resize(0, 2);
  lp.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(2, 2);
  Aeq.insert(0, 0) = 1.0; Aeq.insert(0, 1) = 1.0;
  Aeq.insert(1, 0) = 1.0; Aeq.insert(1, 1) = 1.0 + eps;
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq.resize(2);
  lp.beq << 2.0, 2.0 + eps;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  opts.max_iter = 100;
  const auto res = solve_lp_with_basis(lp, opts);
  INFO("ill-conditioned status=" << res.result.stats.status);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(2.0).margin(1e-6));
  CHECK(res.result.x[0] == Approx(1.0).margin(1e-5));
  CHECK(res.result.x[1] == Approx(1.0).margin(1e-5));
}

// ─── Cached node LP ≡ fresh solve with scaling enabled ─────────────────────
// Exercises the Ruiz-scaled cached path end to end: bounds scaling,
// primal/dual unscaling must agree with a fresh full solve.
TEST_CASE("Cached node LP matches fresh solve with Ruiz scaling on",
          "[numerical][ruiz][cached]") {
  LPModel lp = make_scaled_lp(1e3, 1e-3, 1e-2, 1e2);
  // Base optimum: x̂* = (2/1e-2, 4/1e2) = (200, 0.04), obj = -14.
  // Node bound x1 <= 150 → unique optimum (150, 0.045), obj = -13.5.

  IPMLPOptions opt;
  opt.ruiz_rounds = 10;
  NativeIPMLPAdapter solver(opt);
  solver.prepare_for_node_solves(lp);

  Eigen::VectorXd node_lb(2), node_ub(2);
  node_lb << 0.0, 0.0;
  node_ub << 150.0, 1e20;
  const SolveResult cached =
      solver.solve_cached_node_lp(node_lb, node_ub, Eigen::VectorXd());

  LPModel node_lp = lp;
  node_lp.vars[0].ub = 150.0;
  const SolveResult fresh = solver.solve_lp(node_lp);

  REQUIRE(cached.stats.success);
  REQUIRE(fresh.stats.success);
  CHECK(cached.stats.objective == Approx(-13.5).margin(1e-5));
  CHECK(cached.stats.objective == Approx(fresh.stats.objective).margin(1e-6));
  REQUIRE(cached.x.size() == 2);
  CHECK(cached.x[0] == Approx(fresh.x[0]).margin(1e-3));
  CHECK(cached.x[1] == Approx(fresh.x[1]).margin(1e-5));
  REQUIRE(cached.constraint_duals.size() == fresh.constraint_duals.size());
  for (int i = 0; i < cached.constraint_duals.size(); ++i) {
    // Slack-row duals are mu/s-level quantities and differ run to run;
    // a broken Dr unscaling would be off by factors of dr (~1e3 here).
    CHECK(cached.constraint_duals[i] ==
          Approx(fresh.constraint_duals[i]).margin(1e-4));
  }
}

TEST_CASE("Cached IPM bound-change batch matches independent node solves",
          "[numerical][ipm][cached][batch]") {
  LPModel lp = make_scaled_lp(1e3, 1e-3, 1e-2, 1e2);
  IPMLPOptions opt;
  opt.ruiz_rounds = 10;
  NativeIPMLPAdapter batch_solver(opt);
  batch_solver.prepare_for_node_solves(lp);

  Eigen::VectorXd parent_lb(2), parent_ub(2);
  parent_lb << 0.0, 0.0;
  parent_ub << 1e20, 1e20;
  std::vector<IPMNodeBoundChange> changes(2);
  changes[0].variable = 0;
  changes[0].upper_bound = 150.0;
  changes[1].variable = 1;
  changes[1].upper_bound = 0.03;

  auto batch = batch_solver.solve_cached_bound_change_batch(
      parent_lb, parent_ub, Eigen::VectorXd(), changes, 2.0);
  REQUIRE(batch.size() == 2);

  for (std::size_t i = 0; i < batch.size(); ++i) {
    INFO("batch entry " << i << " status=" << batch[i].result.stats.status);
    REQUIRE(batch[i].attempted);
    REQUIRE(batch[i].result.stats.success);

    LPModel node_lp = lp;
    const auto& change = changes[i];
    node_lp.vars[change.variable].lb =
        std::max(node_lp.vars[change.variable].lb, change.lower_bound);
    node_lp.vars[change.variable].ub =
        std::min(node_lp.vars[change.variable].ub, change.upper_bound);
    NativeIPMLPAdapter fresh_solver(opt);
    const SolveResult fresh = fresh_solver.solve_lp(node_lp);
    REQUIRE(fresh.stats.success);
    CHECK(batch[i].result.stats.objective ==
          Approx(fresh.stats.objective).margin(1e-6));
  }
}

// ─── CHOLMOD backend wrapper ────────────────────────────────────────────────
// Direct test of the vendored CHOLMOD int64 sparse Cholesky: analyze once,
// factorize per numerical refresh, solve.  Also covers the not-PD failure
// path used by the IPM's dynamic-regularization retry.
TEST_CASE("CholmodLDLT: factorize/solve known SPD systems",
          "[numerical][cholmod]") {
#if MIPSOLVERS_HAVE_CHOLMOD
  // A1 = [[4,1,0,0],[1,3,1,0],[0,1,2,1],[0,0,1,1]] — lower-triangle CSC.
  const int64_t m = 4;
  std::vector<int> outer = {0, 2, 4, 6, 7};
  std::vector<int> inner = {0, 1, 1, 2, 2, 3, 3};
  std::vector<double> vals = {4.0, 1.0, 3.0, 1.0, 2.0, 1.0, 1.0};

  CholmodLDLT ldlt;
  REQUIRE(ldlt.analyze(m, outer.data(), inner.data(), vals.data(),
                       static_cast<int64_t>(vals.size())));
  REQUIRE(ldlt.factorize(vals.data()));

  const double b[4] = {1.0, 2.0, 3.0, 4.0};
  double x[4] = {0.0, 0.0, 0.0, 0.0};
  REQUIRE(ldlt.solve(b, x));
  const double A1[4][4] = {{4,1,0,0},{1,3,1,0},{0,1,2,1},{0,0,1,1}};
  for (int i = 0; i < 4; ++i) {
    double s = 0.0;
    for (int j = 0; j < 4; ++j) s += A1[i][j] * x[j];
    CHECK(s == Approx(b[i]).margin(1e-12));
  }

  // Analyze-once / factorize-many: new values in the same pattern.
  std::vector<double> vals2 = {5.0, 1.0, 4.0, 1.0, 3.0, 1.0, 2.0};
  REQUIRE(ldlt.factorize(vals2.data()));
  REQUIRE(ldlt.solve(b, x));
  const double A2[4][4] = {{5,1,0,0},{1,4,1,0},{0,1,3,1},{0,0,1,2}};
  for (int i = 0; i < 4; ++i) {
    double s = 0.0;
    for (int j = 0; j < 4; ++j) s += A2[i][j] * x[j];
    CHECK(s == Approx(b[i]).margin(1e-12));
  }

  // Not positive definite → factorize must report failure (IPM dyn-reg
  // retry relies on this).
  std::vector<double> vals3 = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  CHECK_FALSE(ldlt.factorize(vals3.data()));
#else
  SUCCEED("CHOLMOD not enabled in this build");
#endif
}

// ─── Augmented (uncondensed) Newton system (P2.12) ──────────────────────────
// The augmented form [H + δI, Jgᵀ, Jhᵀ; Jg, 0, 0; Jh, 0, -SM⁻¹] must produce
// the same iterates/solution as the condensed [H + Jhᵀ(M/S)Jh] form.
TEST_CASE("Augmented Newton path matches condensed path on inequality NLP",
          "[numerical][augmented]") {
  // min 0.5*x0^2 + x1^2  s.t.  x0 + x1 >= 2,  x0 <= 1.5
  // Optimum: (4/3, 2/3), obj = 4/3 (constraint x0+x1=2 active, x0<=1.5 slack).
  auto make_nlp = []() {
    NLPModel nlp;
    nlp.sense = Sense::Minimize;
    nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
    nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
    nlp.x0 = Eigen::VectorXd::Zero(2);
    nlp.f = [](const Eigen::VectorXd& x) {
      return 0.5 * x[0] * x[0] + x[1] * x[1];
    };
    nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
      g.resize(2);
      g[0] = x[0];
      g[1] = 2.0 * x[1];
    };
    nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& H) {
      H.resize(2, 2);
      H.insert(0, 0) = 1.0;
      H.insert(1, 1) = 2.0;
      H.makeCompressed();
    };
    nlp.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& v) {
      v.resize(2);
      v[0] = 2.0 - x[0] - x[1];
      v[1] = x[0] - 1.5;
    };
    nlp.jac_h = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& J) {
      J.resize(2, 2);
      J.insert(0, 0) = -1.0;
      J.insert(0, 1) = -1.0;
      J.insert(1, 0) = 1.0;
      J.makeCompressed();
    };
    return nlp;
  };

  auto solve = [&](bool augmented) {
    NLPModel nlp = make_nlp();
    IPMOptions options;
    options.max_iter = 100;
    options.tol_primal = 1e-8;
    options.tol_dual = 1e-8;
    options.tol_complementarity = 1e-8;
    options.tol_accept = 0.0;
    options.scale_problem = false;
    options.use_restoration_phase = false;
    options.use_augmented_newton = augmented;
    NativeIPMAdapter solver(options);
    return solver.solve_nlp_detail(nlp).first;
  };

  const SolveResult rc = solve(false);
  const SolveResult ra = solve(true);
  REQUIRE(rc.stats.success);
  REQUIRE(ra.stats.success);
  CHECK(ra.stats.objective == Approx(4.0 / 3.0).margin(1e-5));
  CHECK(ra.x[0] == Approx(4.0 / 3.0).margin(1e-5));
  CHECK(ra.x[1] == Approx(2.0 / 3.0).margin(1e-5));
  // The two Newton forms must agree to solver tolerance.
  CHECK(ra.stats.objective == Approx(rc.stats.objective).margin(1e-7));
  CHECK(ra.x[0] == Approx(rc.x[0]).margin(1e-6));
  CHECK(ra.x[1] == Approx(rc.x[1]).margin(1e-6));
}

// ─── MUMPS symmetric-indefinite backend ─────────────────────────────────────
// MumpsSolver (multifrontal LDLᵀ) on a symmetric indefinite KKT system:
// analyze once, factorize, solve, then re-factorize new values in the same
// pattern (the IPM's "analyze once, factorize many" contract).
TEST_CASE("MumpsSolver solves symmetric indefinite KKT systems",
          "[numerical][mumps]") {
#if MIPSOLVERS_HAVE_MUMPS
  // K = [[2, 1, 1], [1, 3, 0], [1, 0, -1]] — indefinite (pos, pos, neg).
  Eigen::SparseMatrix<double> K(3, 3);
  K.insert(0, 0) = 2.0;
  K.insert(0, 1) = 1.0;
  K.insert(0, 2) = 1.0;
  K.insert(1, 0) = 1.0;
  K.insert(1, 1) = 3.0;
  K.insert(2, 0) = 1.0;
  K.insert(2, 2) = -1.0;
  K.makeCompressed();

  MumpsSolver solver;
  solver.analyze_pattern(K);
  REQUIRE(solver.factorize(K));
  const Eigen::VectorXd b = (Eigen::VectorXd(3) << 1.0, 2.0, 0.5).finished();
  Eigen::VectorXd x;
  REQUIRE(solver.solve(b, x));
  CHECK((K * x - b).lpNorm<Eigen::Infinity>() < 1e-10);

  // New values, same pattern → reuse analysis (analyze-once contract).
  Eigen::SparseMatrix<double> K2 = K;
  K2.coeffRef(0, 0) = 4.0;
  K2.coeffRef(1, 1) = 5.0;
  K2.makeCompressed();
  REQUIRE(solver.factorize(K2));
  REQUIRE(solver.solve(b, x));
  CHECK((K2 * x - b).lpNorm<Eigen::Infinity>() < 1e-10);
#else
  SUCCEED("MUMPS not enabled in this build");
#endif
}

// ─── KLU rescue backend (cold-start escalation level 2) ─────────────────────
// Large degenerate assignment LP (m = 240 > kSparseSimplexThreshold so the
// sparse SparseBasis path is used): S supply rows sum_j x_ij <= 1, T demand
// rows sum_i x_ij = 1, planted optimal permutation with cost 0.  The KLU
// rescue backend (forced via escalation_start_level = 2) must find the same
// optimum as the default UMFPACK path.
TEST_CASE("KLU rescue backend solves a large degenerate assignment LP",
          "[numerical][klu][escalation]") {
#if defined(MIPSOLVERS_HAVE_KLU)
  const int S = 120, T = 120;
  LPModel lp;
  lp.sense = Sense::Minimize;
  const int n = S * T;
  lp.c.resize(n);
  // Planted optimum: x_{i, p(i)} = 1 with p(i) = (i * 7 + 3) % T (a
  // permutation since gcd(7,120)=1), cost 0 on the permutation, positive
  // elsewhere → unique optimal objective 0.
  auto planted = [&](int i) { return (i * 7 + 3) % T; };
  std::vector<Eigen::Triplet<double>> trips, trips_eq;
  trips.reserve(static_cast<std::size_t>(n));
  trips_eq.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < S; ++i) {
    for (int j = 0; j < T; ++j) {
      const int v = i * T + j;
      lp.c[v] = (j == planted(i)) ? 0.0
                                  : 1.0 + static_cast<double>((i * T + j) % 11) * 1e-3;
      trips.emplace_back(i, v, 1.0);       // supply row i
      trips_eq.emplace_back(j, v, 1.0);    // demand row j
      lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
    }
  }
  lp.A.resize(S, n);
  lp.A.setFromTriplets(trips.begin(), trips.end());
  lp.b = Eigen::VectorXd::Ones(S);
  lp.Aeq.resize(T, n);
  lp.Aeq.setFromTriplets(trips_eq.begin(), trips_eq.end());
  lp.beq = Eigen::VectorXd::Ones(T);

  auto def = native_simplex_options();
  def.max_iter = 100000;
  const auto res_def = solve_lp_with_basis(lp, def);
  REQUIRE(res_def.result.stats.success);
  CHECK(res_def.result.stats.objective == Approx(0.0).margin(1e-6));

  auto klu = native_simplex_options();
  klu.max_iter = 100000;
  klu.escalation_start_level = 2;  // force FactorBackendKind::KluRescue
  const auto res_klu = solve_lp_with_basis(lp, klu);
  REQUIRE(res_klu.result.stats.success);
  CHECK(res_klu.result.stats.objective == Approx(0.0).margin(1e-6));
  CHECK(res_klu.result.stats.objective ==
        Approx(res_def.result.stats.objective).margin(1e-9));

  // Original-space feasibility audit of the KLU solution.
  double max_row_viol = 0.0;
  const Eigen::VectorXd ax = lp.A * res_klu.result.x;
  for (int i = 0; i < ax.size(); ++i)
    max_row_viol = std::max(max_row_viol, ax[i] - lp.b[i]);
  const Eigen::VectorXd aeqx = lp.Aeq * res_klu.result.x;
  for (int i = 0; i < aeqx.size(); ++i)
    max_row_viol = std::max(max_row_viol, std::abs(aeqx[i] - lp.beq[i]));
  CHECK(max_row_viol < 1e-8);
#else
  SUCCEED("KLU not compiled; rescue backend unavailable");
#endif
}

// ─── Escalation chain integration ───────────────────────────────────────────
// The full chain (levels 0→2) on the same degenerate LP must succeed and
// agree with the single-attempt default within tolerance.
TEST_CASE("Cold-start escalation chain agrees with default solve",
          "[numerical][escalation]") {
  const int S = 110, T = 110;
  LPModel lp;
  lp.sense = Sense::Minimize;
  const int n = S * T;
  lp.c.resize(n);
  auto planted = [&](int i) { return (i * 7 + 3) % T; };
  std::vector<Eigen::Triplet<double>> trips, trips_eq;
  for (int i = 0; i < S; ++i) {
    for (int j = 0; j < T; ++j) {
      const int v = i * T + j;
      lp.c[v] = (j == planted(i)) ? 0.0
                                  : 1.0 + static_cast<double>((i * T + j) % 11) * 1e-3;
      trips.emplace_back(i, v, 1.0);
      trips_eq.emplace_back(j, v, 1.0);
      lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
    }
  }
  lp.A.resize(S, n);
  lp.A.setFromTriplets(trips.begin(), trips.end());
  lp.b = Eigen::VectorXd::Ones(S);
  lp.Aeq.resize(T, n);
  lp.Aeq.setFromTriplets(trips_eq.begin(), trips_eq.end());
  lp.beq = Eigen::VectorXd::Ones(T);

  auto opt = native_simplex_options();
  opt.max_iter = 100000;
  opt.escalation_max_level = 2;
  const auto res = solve_lp_with_basis(lp, opt);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(0.0).margin(1e-6));

  auto off = native_simplex_options();
  off.max_iter = 100000;
  off.escalation_max_level = 0;  // legacy behaviour
  const auto res_off = solve_lp_with_basis(lp, off);
  REQUIRE(res_off.result.stats.success);
  CHECK(res.result.stats.objective ==
        Approx(res_off.result.stats.objective).margin(1e-9));

  auto start1 = native_simplex_options();
  start1.max_iter = 100000;
  start1.escalation_start_level = 1;  // begin directly at the hardened level
  const auto res_start1 = solve_lp_with_basis(lp, start1);
  REQUIRE(res_start1.result.stats.success);
  CHECK(res_start1.result.stats.objective ==
        Approx(res_off.result.stats.objective).margin(1e-9));
}

// ─── False-optimal residual audit (round 2) ──────────────────────────────────
// The audit that guards cold-start "Optimal" results must accept a true
// solution and reject a corrupted one (the HiGHS-style "Optimal but wrong"
// failure mode from the scaling probe).
TEST_CASE("Residual audit accepts true solutions and rejects corrupted ones",
          "[numerical][escalation][audit]") {
  LPModel lp = make_scaled_lp(1.0, 1.0, 1.0, 1.0);  // optimum (2, 4), obj -14
  const auto res = solve_lp_with_basis(lp, native_simplex_options());
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(-14.0).margin(1e-8));

  CHECK(lp_solution_residual_acceptable(lp, res.result.x, 1e-8));

  // Corrupted iterate: violates the row 2x + y <= 8 by a wide margin.
  Eigen::VectorXd bad = res.result.x;
  bad[0] += 1.0;
  CHECK_FALSE(lp_solution_residual_acceptable(lp, bad, 1e-8));

  // Bound violation is also caught.
  Eigen::VectorXd below = res.result.x;
  below[1] = -1.0;
  CHECK_FALSE(lp_solution_residual_acceptable(lp, below, 1e-8));

  // Wrong-sized vector is rejected outright.
  CHECK_FALSE(lp_solution_residual_acceptable(lp, Eigen::VectorXd::Zero(1), 1e-8));

  // A cold-start "Optimal" that fails the audit at every escalation level is
  // demoted to an honest failure, never returned as a false optimum (the
  // stocfor1-1e6 scaling probe is the deterministic real-world trigger).
  auto paranoid = native_simplex_options();
  // Negative tol rejects even an exact zero-residual solution (viol >= 0 by
  // construction), forcing the audit to fail at every escalation level.
  paranoid.escalation_residual_tol = -1.0;
  const auto demoted = solve_lp_with_basis(lp, paranoid);
  CHECK_FALSE(demoted.result.stats.success);
  CHECK(demoted.result.stats.status == "Residual audit rejected");
}

// ─── solve_lp_from_sf escalation chain (round 2) ─────────────────────────────
// The standard-form entry point (cut re-solves, IPM-repair crossover) gets the
// same cold-start escalation as the LPModel path.  Levels >= 1 solve a
// re-equilibrated copy and publish through a warm re-solve of the caller's sf,
// so all outputs must land in caller space.

namespace {

// Degenerate assignment LP (same family as the escalation tests above):
// m = S + T rows > kSparseSimplexThreshold so the sparse path is exercised.
LPModel make_assignment_lp(int S, int T) {
  LPModel lp;
  lp.sense = Sense::Minimize;
  const int n = S * T;
  lp.c.resize(n);
  auto planted = [&](int i) { return (i * 7 + 3) % T; };
  std::vector<Eigen::Triplet<double>> trips, trips_eq;
  for (int i = 0; i < S; ++i) {
    for (int j = 0; j < T; ++j) {
      const int v = i * T + j;
      lp.c[v] = (j == planted(i)) ? 0.0
                                  : 1.0 + static_cast<double>((i * T + j) % 11) * 1e-3;
      trips.emplace_back(i, v, 1.0);
      trips_eq.emplace_back(j, v, 1.0);
      lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
    }
  }
  lp.A.resize(S, n);
  lp.A.setFromTriplets(trips.begin(), trips.end());
  lp.b = Eigen::VectorXd::Ones(S);
  lp.Aeq.resize(T, n);
  lp.Aeq.setFromTriplets(trips_eq.begin(), trips_eq.end());
  lp.beq = Eigen::VectorXd::Ones(T);
  return lp;
}

}  // namespace

TEST_CASE("solve_lp_from_sf escalation levels agree with the default path",
          "[numerical][escalation][sf]") {
  LPModel lp = make_assignment_lp(110, 110);
  StandardFormLP sf = build_standard_form_lp(lp);
  ruiz_scale_standard_form(sf, 10);

  auto base = native_simplex_options();
  base.max_iter = 100000;
  const auto res0 = solve_lp_from_sf(sf, base);
  INFO("cold status=" << res0.result.stats.status);
  REQUIRE(res0.result.stats.success);
  CHECK(res0.result.stats.objective == Approx(0.0).margin(1e-6));

  SimplexOptions lvl1 = base;
  lvl1.escalation_start_level = 1;
  const auto res1 = solve_lp_from_sf(sf, lvl1);
  REQUIRE(res1.result.stats.success);
  CHECK(res1.result.stats.objective ==
        Approx(res0.result.stats.objective).margin(1e-9));

#if defined(MIPSOLVERS_HAVE_KLU)
  SimplexOptions lvl2 = base;
  lvl2.escalation_start_level = 2;
  const auto res2 = solve_lp_from_sf(sf, lvl2);
  REQUIRE(res2.result.stats.success);
  CHECK(res2.result.stats.objective ==
        Approx(res0.result.stats.objective).margin(1e-9));
#endif
}

TEST_CASE("solve_lp_from_sf escalation publishes in caller sf space",
          "[numerical][escalation][sf]") {
  LPModel lp = make_assignment_lp(110, 110);
  StandardFormLP sf = build_standard_form_lp(lp);
  ruiz_scale_standard_form(sf, 10);

  auto lvl1 = native_simplex_options();
  lvl1.max_iter = 100000;
  lvl1.escalation_start_level = 1;  // forces the copy-solve + publish path
  const auto res = solve_lp_from_sf(sf, lvl1);
  INFO("publish status=" << res.result.stats.status);
  REQUIRE(res.result.stats.success);

  // x_std must satisfy the CALLER's scaled system, not the re-equilibrated
  // copy's: a hand-mapping bug between the two scalings would show up here.
  REQUIRE(res.x_std.size() == static_cast<int>(sf.A.cols()));
  const Eigen::VectorXd r = sf.A * res.x_std - sf.b;
  CHECK(r.cwiseAbs().maxCoeff() < 1e-7);
  CHECK(sf_solution_residual_acceptable(sf, res.x_std, 1e-6));
}

TEST_CASE("solve_lp_from_sf warm hints are never audited or escalated",
          "[numerical][escalation][sf]") {
  LPModel lp = make_assignment_lp(110, 110);
  StandardFormLP sf = build_standard_form_lp(lp);
  ruiz_scale_standard_form(sf, 10);

  auto opt = native_simplex_options();
  opt.max_iter = 100000;
  const auto cold = solve_lp_from_sf(sf, opt);
  INFO("warm-root status=" << cold.result.stats.status);
  REQUIRE(cold.result.stats.success);

  // Absurd residual tolerance: if the chain audited warm-hint results, this
  // would reject the (perfectly good) solution and trigger cold escalation;
  // with allow_cold_start=false the single warm attempt must stand.
  SimplexOptions warm = opt;
  warm.allow_cold_start = false;
  warm.escalation_residual_tol = 1e-300;
  const auto res = solve_lp_from_sf(sf, warm, &cold.basis);
  REQUIRE(res.result.stats.success);
  CHECK(res.solved_from_hint);
  CHECK(res.result.stats.objective ==
        Approx(cold.result.stats.objective).margin(1e-9));
}
