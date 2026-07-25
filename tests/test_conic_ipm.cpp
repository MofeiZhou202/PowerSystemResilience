/// test_conic_ipm.cpp
/// Tests for the native conic (LP/SOCP/SDP) interior-point solver:
/// cone algebra identities, NT scalings, max_step formulas, end-to-end
/// LP/SOCP/SDP solves with analytic optima, infeasibility certificates,
/// parallel determinism, and the engine adapter.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <tuple>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/kernel/ipm/cones.hpp"
#include "mipsolvers/engine/kernel/ipm/conic_ipm_solver.hpp"
#include "mipsolvers/engine/problem_types.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

std::mt19937_64 rng(12345);

/// Random vector strictly inside Q^k.
Eigen::VectorXd random_soc_interior(int k) {
  std::normal_distribution<double> nd(0.0, 1.0);
  std::uniform_real_distribution<double> ud(0.5, 2.0);
  Eigen::VectorXd u(k);
  for (int i = 1; i < k; ++i) u[i] = nd(rng);
  u[0] = u.tail(k - 1).norm() + ud(rng);
  return u;
}

/// Random SPD matrix (p x p).
Eigen::MatrixXd random_spd(int p) {
  std::normal_distribution<double> nd(0.0, 1.0);
  Eigen::MatrixXd a(p, p);
  for (int i = 0; i < p; ++i)
    for (int j = 0; j < p; ++j) a(i, j) = nd(rng);
  return a * a.transpose() + p * Eigen::MatrixXd::Identity(p, p);
}

/// J = diag(1, -1, ..., -1) applied to u.
Eigen::VectorXd jmul(const Eigen::VectorXd& u) {
  Eigen::VectorXd out = u;
  out.tail(u.size() - 1) = -out.tail(u.size() - 1);
  return out;
}

/// Brute-force max step to the SOC boundary via bisection on the
/// interior predicate; returns +inf when no exit is found below 1e12.
double q_max_step_brute(const Eigen::VectorXd& u, const Eigen::VectorXd& du) {
  auto interior = [&](double a) { return q_interior(u + a * du); };
  if (!interior(0.0)) return 0.0;
  double hi = 1.0;
  while (interior(hi)) {
    hi *= 2.0;
    if (hi > 1e12) return std::numeric_limits<double>::infinity();
  }
  double lo = 0.0;
  for (int it = 0; it < 200; ++it) {
    const double mid = 0.5 * (lo + hi);
    if (interior(mid)) lo = mid; else hi = mid;
  }
  return lo;
}

/// Build a sparse matrix from (row, col, value) triplets.
Eigen::SparseMatrix<double> make_sparse(
    int rows, int cols,
    const std::vector<std::tuple<int, int, double>>& triplets) {
  std::vector<Eigen::Triplet<double>> t;
  t.reserve(triplets.size());
  for (const auto& [r, c, v] : triplets) t.emplace_back(r, c, v);
  Eigen::SparseMatrix<double> m(rows, cols);
  m.setFromTriplets(t.begin(), t.end());
  m.makeCompressed();
  return m;
}

/// Negative identity as a sparse matrix.
Eigen::SparseMatrix<double> neg_identity(int k) {
  std::vector<std::tuple<int, int, double>> t;
  for (int i = 0; i < k; ++i) t.emplace_back(i, i, -1.0);
  return make_sparse(k, k, t);
}

/// The mixed l+q+s test problem used for determinism/adapter checks.
ConicModel make_mixed_model() {
  // variables x in R^6;  l rows (6), q block (3), s blocks S^2 and S^20.
  // l: 3-x0 >= 0, 4-x1 >= 0, 5-x2 >= 0, x1 >= 0, x0 >= 0, 8-x5 >= 0
  // q: (1+x2, x3, x4) in Q^3
  // s: I + [[x5, x0],[x0, x1]] psd;  second block I + x0*E11 in S^20 (mostly
  //    empty rows; makes the per-block parallel regions big enough to run).
  const int p2 = 20;
  const int mp2 = svec_size(p2);
  const int m = 12 + mp2;
  ConicModel cm;
  cm.c.resize(6);
  cm.c << 1.0, 1.0, -1.0, 0.5, 0.5, -1.0;
  const double sq2 = std::sqrt(2.0);
  std::vector<std::tuple<int, int, double>> tri{
      {0, 0, 1.0}, {1, 1, 1.0}, {2, 2, 1.0}, {3, 1, -1.0},
      {4, 0, -1.0}, {5, 5, 1.0}, {6, 2, -1.0}, {7, 3, -1.0},
      {8, 4, -1.0}, {9, 5, -1.0}, {10, 0, -sq2}, {11, 1, -1.0},
      {12, 0, -1.0}};
  cm.G = make_sparse(m, 6, tri);
  cm.h = Eigen::VectorXd::Zero(m);
  cm.h << 3.0, 4.0, 5.0, 0.0, 0.0, 8.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0;
  // svec(I) entries of the order-p2 block (first diag entry is 1 + x0).
  for (int j = 0; j < p2; ++j) {
    cm.h[12 + svec_index(j, j, p2)] = 1.0;
  }
  cm.A.resize(0, 6);
  cm.b.resize(0);
  cm.dims.l = 6;
  cm.dims.q = {3};
  cm.dims.s = {2, p2};
  return cm;
}

}  // namespace

// ───────────────────────────────────────────────────────────────────────────
// 1. svec/smat packing
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("cones: svec/smat roundtrip and inner product", "[conic][cones]") {
  const int p = 4;
  Eigen::MatrixXd s = random_spd(p);
  Eigen::MatrixXd z = random_spd(p);
  const Eigen::VectorXd sv = svec(s);
  REQUIRE(sv.size() == svec_size(p));
  // Roundtrip.
  const Eigen::MatrixXd s2 = smat(sv, p);
  CHECK((s2 - s).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-14));
  // sqrt(2) scaling of off-diagonals.
  CHECK(sv[svec_index(0, 0, p)] == Approx(s(0, 0)));
  CHECK(sv[svec_index(1, 0, p)] == Approx(std::sqrt(2.0) * s(1, 0)));
  CHECK(sv[svec_index(3, 2, p)] == Approx(std::sqrt(2.0) * s(3, 2)));
  // Inner product preservation: svec(S)'svec(Z) = tr(SZ).
  const Eigen::VectorXd zv = svec(z);
  const double tr_sz = (s * z).trace();
  CHECK(sv.dot(zv) == Approx(tr_sz).margin(1e-10));
}

// ───────────────────────────────────────────────────────────────────────────
// 2. q-block Jordan algebra and NT scaling
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("cones: SOC Jordan algebra identities", "[conic][cones]") {
  const int k = 5;
  const Eigen::VectorXd u = random_soc_interior(k);
  // u o u^{-1} = e
  const Eigen::VectorXd e1 = q_jordan_product(u, q_jordan_inverse(u));
  CHECK(e1[0] == Approx(1.0).margin(1e-12));
  CHECK(e1.tail(k - 1).norm() == Approx(0.0).margin(1e-12));
  // sqrt(u) o sqrt(u) = u
  const Eigen::VectorXd sq = q_jordan_sqrt(u);
  const Eigen::VectorXd sq2 = q_jordan_product(sq, sq);
  CHECK((sq2 - u).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
  // Jordan division: y = u o\ b solves u o y = b (L(u)^{-1}, not L(u^{-1})).
  std::normal_distribution<double> nd(0.0, 1.0);
  for (int trial = 0; trial < 10; ++trial) {
    const Eigen::VectorXd ui = random_soc_interior(k);
    Eigen::VectorXd b(k);
    for (auto& v : b) v = nd(rng);
    const Eigen::VectorXd y = q_jordan_solve(ui, b);
    CHECK((q_jordan_product(ui, y) - b).cwiseAbs().maxCoeff() ==
          Approx(0.0).margin(1e-10));
    // For k == 1 the division reduces to scalar division.
  }
  {
    const Eigen::VectorXd u1 = Eigen::VectorXd::Constant(1, 2.5);
    const Eigen::VectorXd b1 = Eigen::VectorXd::Constant(1, 5.0);
    CHECK(q_jordan_solve(u1, b1)[0] == Approx(2.0).margin(1e-14));
  }
}

TEST_CASE("cones: SOC NT scaling", "[conic][cones]") {
  const int k = 5;
  const Eigen::VectorXd s = random_soc_interior(k);
  const Eigen::VectorXd z = random_soc_interior(k);

  ConeDims dims;
  dims.l = 0;
  dims.q = {k};
  const ConeLayout lay = ConeLayout::build(dims);
  CHECK(lay.degree == 1);

  ConeNtScaling sc;
  REQUIRE(sc.compute(lay, s, z));
  REQUIRE(sc.q_scalings().size() == 1);
  const auto& qs = sc.q_scalings()[0];

  // lambda = W z = W^{-1} s (the defining NT identity).
  Eigen::VectorXd wz(k), wis(k);
  sc.apply_wt(z, wz);
  sc.apply_w_inv(s, wis);
  CHECK((wz - wis).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-12));
  CHECK((wz - sc.lambda()).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-12));
  // lambda is interior.
  CHECK(q_interior(sc.lambda()));

  // H = W'W = beta^2 (2 wbar wbar' - J): check apply_h against the formula.
  std::normal_distribution<double> nd(0.0, 1.0);
  for (int trial = 0; trial < 5; ++trial) {
    Eigen::VectorXd x(k);
    for (auto& v : x) v = nd(rng);
    Eigen::VectorXd hx(k);
    sc.apply_h(x, hx);
    const Eigen::VectorXd expected =
        (qs.beta * qs.beta) *
        (2.0 * qs.wbar.dot(x) * qs.wbar - jmul(x));
    CHECK((hx - expected).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
    // H^{-1} H = I and W^{-1} W = I.
    Eigen::VectorXd hh(k), ww(k);
    sc.apply_h_inv(hx, hh);
    CHECK((hh - x).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
    Eigen::VectorXd wx(k);
    sc.apply_w(x, wx);
    sc.apply_w_inv(wx, ww);
    CHECK((ww - x).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
  }
}

// ───────────────────────────────────────────────────────────────────────────
// 3. s-block NT scaling
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("cones: SDP NT scaling", "[conic][cones]") {
  const int p = 3;
  const Eigen::MatrixXd sm = random_spd(p);
  const Eigen::MatrixXd zm = random_spd(p);
  const Eigen::VectorXd s = svec(sm);
  const Eigen::VectorXd z = svec(zm);

  ConeDims dims;
  dims.s = {p};
  const ConeLayout lay = ConeLayout::build(dims);
  CHECK(lay.degree == p);

  ConeNtScaling sc;
  REQUIRE(sc.compute(lay, s, z));
  REQUIRE(sc.s_scalings().size() == 1);
  const auto& ss = sc.s_scalings()[0];
  const Eigen::MatrixXd& r = ss.r;
  const Eigen::MatrixXd sig = ss.sigma.asDiagonal();

  // R'ZR = diag(sigma), R^{-1} S R^{-T} = diag(sigma).
  CHECK((r.transpose() * zm * r - sig).cwiseAbs().maxCoeff() ==
        Approx(0.0).margin(1e-10));
  const Eigen::MatrixXd rinv = r.inverse();
  CHECK((rinv * sm * rinv.transpose() - sig).cwiseAbs().maxCoeff() ==
        Approx(0.0).margin(1e-10));

  // lambda = P s = D z = svec(diag(sigma)).
  Eigen::VectorXd ps(svec_size(p)), dz(svec_size(p));
  sc.apply_w_inv(s, ps);
  sc.apply_wt(z, dz);
  CHECK((ps - sc.lambda()).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
  CHECK((dz - sc.lambda()).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
  CHECK(s_interior(sc.lambda(), p));

  // Operator roundtrips on a random packed vector.
  std::normal_distribution<double> nd(0.0, 1.0);
  Eigen::VectorXd x(svec_size(p));
  for (auto& v : x) v = nd(rng);
  Eigen::VectorXd t1(x.size()), t2(x.size());
  sc.apply_w(x, t1);
  sc.apply_w_inv(t1, t2);
  CHECK((t2 - x).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
  sc.apply_wt(x, t1);
  sc.apply_wt_inv(t1, t2);
  CHECK((t2 - x).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
  sc.apply_h(x, t1);
  sc.apply_h_inv(t1, t2);
  CHECK((t2 - x).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
  // H = P^{-1} D (i.e. apply_w o apply_wt).
  Eigen::VectorXd h1(x.size()), h2(x.size());
  sc.apply_wt(x, t1);
  sc.apply_w(t1, h1);
  sc.apply_h(x, h2);
  CHECK((h1 - h2).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));
}

// ───────────────────────────────────────────────────────────────────────────
// 4. q-block max_step vs brute force
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("cones: SOC max_step matches bisection", "[conic][cones]") {
  std::normal_distribution<double> nd(0.0, 1.0);
  int bounded_cases = 0;
  for (int trial = 0; trial < 30; ++trial) {
    const int k = 2 + static_cast<int>(rng() % 5);
    const Eigen::VectorXd u = random_soc_interior(k);
    Eigen::VectorXd du(k);
    for (auto& v : du) v = nd(rng);
    const double alpha = q_max_step(u, du);
    const double brute = q_max_step_brute(u, du);
    if (std::isinf(brute)) {
      CHECK(std::isinf(alpha));
    } else {
      ++bounded_cases;
      REQUIRE(!std::isinf(alpha));
      CHECK(alpha == Approx(brute).epsilon(1e-7));
      // At the returned step the point is on the boundary (or past it by
      // less than a tiny tolerance), and just before it is interior.
      CHECK(q_det(u + alpha * du) >= -1e-12);
    }
  }
  CHECK(bounded_cases > 10);  // sanity: most random directions hit the boundary
  // Obvious edge cases.
  const Eigen::VectorXd u = random_soc_interior(4);
  CHECK(std::isinf(q_max_step(u, Eigen::VectorXd::Zero(4))));
  CHECK(std::isinf(q_max_step(u, u)));  // ray along an interior direction
}

// ───────────────────────────────────────────────────────────────────────────
// 5. LP in conic form vs the engine LP solver
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("conic ipm: LP in cvxopt form matches engine LP", "[conic][ipm]") {
  // min -x - 2y  s.t.  x + y <= 6, 2x + y <= 8, x,y >= 0
  // s = h - Gx >= 0 rows: [x+y<=6, 2x+y<=8, -x<=0, -y<=0].
  // Unique optimum x* = (0, 6), obj = -12.
  ConicModel cm;
  cm.c.resize(2);
  cm.c << -1.0, -2.0;
  cm.G = make_sparse(4, 2,
                     {{0, 0, 1.0}, {0, 1, 1.0},
                      {1, 0, 2.0}, {1, 1, 1.0},
                      {2, 0, -1.0},
                      {3, 1, -1.0}});
  cm.h.resize(4);
  cm.h << 6.0, 8.0, 0.0, 0.0;
  cm.A.resize(0, 2);
  cm.b.resize(0);
  cm.dims.l = 4;

  const ConicIPMResult res = ConicIPMSolver().solve(cm);
  REQUIRE(res.status == "optimal");
  CHECK(res.primal_objective == Approx(-12.0).margin(1e-6));
  CHECK(res.x[0] == Approx(0.0).margin(1e-5));
  CHECK(res.x[1] == Approx(6.0).margin(1e-5));

  SolverEngine eng;
  LPModel lp;
  lp.c = cm.c;
  lp.A = make_sparse(2, 2, {{0, 0, 1.0}, {0, 1, 1.0}, {1, 0, 2.0}, {1, 1, 1.0}});
  lp.b.resize(2);
  lp.b << 6.0, 8.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  const auto lp_res = eng.solve_lp(lp);
  REQUIRE(lp_res.stats.success);
  CHECK(res.primal_objective == Approx(lp_res.stats.objective).margin(1e-6));
}

// ───────────────────────────────────────────────────────────────────────────
// 6. SOCP problems with analytic optima
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("conic ipm: SOCP projection onto the cone vertex", "[conic][ipm]") {
  // min t  s.t.  ||x - a|| <= t   (a = (1,2)):  t* = 0, x* = a
  ConicModel cm;
  cm.c.resize(3);
  cm.c << 1.0, 0.0, 0.0;
  cm.G = neg_identity(3);
  cm.h.resize(3);
  cm.h << 0.0, -1.0, -2.0;
  cm.A.resize(0, 3);
  cm.b.resize(0);
  cm.dims.q = {3};

  const ConicIPMResult res = ConicIPMSolver().solve(cm);
  REQUIRE(res.status == "optimal");
  CHECK(res.primal_objective == Approx(0.0).margin(1e-5));
  CHECK(res.x[1] == Approx(1.0).margin(1e-4));
  CHECK(res.x[2] == Approx(2.0).margin(1e-4));
  // KKT residuals: dual feasibility, z in K, small gap.
  const Eigen::VectorXd rc = cm.G.transpose() * res.z + cm.c;
  CHECK(rc.cwiseAbs().maxCoeff() < 1e-5);
  CHECK(res.z[0] >= -1e-8);
  CHECK(q_det(res.z) >= -1e-8);
  CHECK(res.gap < 1e-5);
}

TEST_CASE("conic ipm: SOCP with equality constraint", "[conic][ipm]") {
  // min t  s.t.  ||x|| <= t,  d'x = 1  with d = (3,4):
  // t* = 1/||d|| = 0.2,  x* = d/||d||^2 = (0.12, 0.16)
  ConicModel cm;
  cm.c.resize(3);
  cm.c << 1.0, 0.0, 0.0;
  cm.G = neg_identity(3);
  cm.h = Eigen::VectorXd::Zero(3);
  cm.A = make_sparse(1, 3, {{0, 1, 3.0}, {0, 2, 4.0}});
  cm.b.resize(1);
  cm.b << 1.0;
  cm.dims.q = {3};

  const ConicIPMResult res = ConicIPMSolver().solve(cm);
  REQUIRE(res.status == "optimal");
  CHECK(res.primal_objective == Approx(0.2).margin(1e-6));
  CHECK(res.x[1] == Approx(0.12).margin(1e-5));
  CHECK(res.x[2] == Approx(0.16).margin(1e-5));
  // KKT residuals.
  const Eigen::VectorXd rc =
      cm.G.transpose() * res.z + cm.A.transpose() * res.y + cm.c;
  CHECK(rc.cwiseAbs().maxCoeff() < 1e-6);
  CHECK((cm.A * res.x - cm.b).cwiseAbs().maxCoeff() < 1e-6);
  CHECK(res.gap < 1e-6);
  CHECK((q_interior(res.z) || (res.z[0] >= 0.0 && q_det(res.z) >= -1e-10)));
}

// ───────────────────────────────────────────────────────────────────────────
// 7. SDP with analytic optimum
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("conic ipm: SDP min X11 with fixed off-diagonal", "[conic][ipm]") {
  // min X11  s.t.  X = [[X11, 2], [2, 1]] psd   ->  X11* = 4
  const double a = 2.0;
  ConicModel cm;
  cm.c.resize(1);
  cm.c << 1.0;
  cm.G = make_sparse(3, 1, {{0, 0, -1.0}});
  cm.h.resize(3);
  cm.h << 0.0, std::sqrt(2.0) * a, 1.0;
  cm.A.resize(0, 1);
  cm.b.resize(0);
  cm.dims.s = {2};

  const ConicIPMResult res = ConicIPMSolver().solve(cm);
  REQUIRE(res.status == "optimal");
  CHECK(res.primal_objective == Approx(a * a).margin(1e-6));
  CHECK(res.x[0] == Approx(a * a).margin(1e-5));
  // z in K: smat(z) positive semidefinite.
  const Eigen::MatrixXd zs = smat(res.z, 2);
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(zs);
  CHECK(es.eigenvalues().minCoeff() >= -1e-7);
  // KKT: dual feasibility and gap.
  const Eigen::VectorXd rc = cm.G.transpose() * res.z + cm.c;
  CHECK(rc.cwiseAbs().maxCoeff() < 1e-6);
  CHECK(res.gap < 1e-6);
}

// ───────────────────────────────────────────────────────────────────────────
// 8. Infeasibility certificates
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("conic ipm: primal infeasible detection", "[conic][ipm]") {
  // x <= -1 and x >= 1 (orthant rows) -> no feasible x.
  ConicModel cm;
  cm.c.resize(1);
  cm.c << 1.0;
  cm.G = make_sparse(2, 1, {{0, 0, 1.0}, {1, 0, -1.0}});
  cm.h.resize(2);
  cm.h << -1.0, -1.0;
  cm.A.resize(0, 1);
  cm.b.resize(0);
  cm.dims.l = 2;

  const ConicIPMResult res = ConicIPMSolver().solve(cm);
  CHECK(res.status == "primal infeasible");
}

TEST_CASE("conic ipm: dual infeasible (unbounded) detection", "[conic][ipm]") {
  // min -x  s.t.  x >= 0 -> unbounded below.
  ConicModel cm;
  cm.c.resize(1);
  cm.c << -1.0;
  cm.G = make_sparse(1, 1, {{0, 0, -1.0}});
  cm.h.resize(1);
  cm.h << 0.0;
  cm.A.resize(0, 1);
  cm.b.resize(0);
  cm.dims.l = 1;

  const ConicIPMResult res = ConicIPMSolver().solve(cm);
  CHECK(res.status == "dual infeasible");
}

// ───────────────────────────────────────────────────────────────────────────
// 9. Mixed-cone problem: correctness + parallel determinism
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("conic ipm: mixed l+q+s problem", "[conic][ipm]") {
  const ConicModel cm = make_mixed_model();
  const ConicIPMResult res = ConicIPMSolver().solve(cm);
  REQUIRE(res.status == "optimal");
  CHECK(res.primal_infeasibility < 1e-6);
  CHECK(res.dual_infeasibility < 1e-6);
  CHECK(res.gap < 1e-5);
}

#if defined(MIPSOLVERS_USE_OPENMP)
TEST_CASE("conic ipm: parallel determinism", "[conic][ipm][openmp]") {
  const ConicModel cm = make_mixed_model();
  ConicIPMOptions o1, o4;
  o1.num_threads = 1;
  o4.num_threads = 4;
  const ConicIPMResult r1 = ConicIPMSolver(o1).solve(cm);
  const ConicIPMResult r4 = ConicIPMSolver(o4).solve(cm);
  REQUIRE(r1.status == "optimal");
  REQUIRE(r4.status == "optimal");
  CHECK(r1.primal_objective == Approx(r4.primal_objective).margin(1e-10));
}
#endif

// ───────────────────────────────────────────────────────────────────────────
// 10. Engine adapter end-to-end
// ───────────────────────────────────────────────────────────────────────────
TEST_CASE("conic ipm: engine adapter end-to-end", "[conic][adapter]") {
  const ConicModel cm = make_mixed_model();
  SolverEngine eng;
  const auto res = eng.solve_conic(cm);
  REQUIRE(res.stats.success);
  CHECK(res.stats.solver_name == "NativeConicIPM");
  CHECK(res.stats.primal_feas < 1e-6);
  CHECK(res.stats.dual_feas < 1e-6);
  CHECK(res.stats.complementarity < 1e-5);
  // constraint_duals = [z (m) | y (m_eq)].
  REQUIRE(res.constraint_duals.size() == cm.dims.total());
  // Compare against the direct kernel solve.
  const ConicIPMResult direct = ConicIPMSolver().solve(cm);
  CHECK(res.stats.objective == Approx(direct.primal_objective).margin(1e-9));
}
