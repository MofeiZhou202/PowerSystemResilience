/// test_aml_conic.cpp
/// Tests for the AML conic (SOCP / PSD) modeling layer: add_soc_constraint,
/// add_rotated_soc_constraint, add_psd_constraint, the compile_conic
/// dispatch, and the conic dual result mapping.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "mipsolvers/aml/aml.hpp"

using namespace mipsolvers::aml;
using Catch::Approx;

namespace {

/// Asset universe atoms shared by the Markowitz cross-check.
const std::vector<Atom> kAssets = {"a0", "a1", "a2"};
/// Expected returns.
const double kMu[3] = {0.10, 0.15, 0.12};
/// Return weight (risk tolerance) in the objective 0.5*x'*Sigma*x - rho*mu'x.
const double kRho = 0.5;
/// Cholesky factor of the covariance matrix (lower triangular).
const double kL[3][3] = {
    {0.20, 0.0,  0.0 },
    {0.05, 0.15, 0.0 },
    {0.03, 0.04, 0.10},
};

/// Covariance entry Sigma(i,j) = (L L')_ij.
double sigma_entry(int i, int j) {
  double s = 0.0;
  for (int k = 0; k < 3; ++k) s += kL[i][k] * kL[j][k];
  return s;
}

/// Add the standard Markowitz constraints (budget row + nonnegativity rows)
/// to model `m` for the 3-asset variable array `xv`.
void add_markowitz_constraints(Model& m, const VarArray& xv) {
  LinearExpr budget;
  for (const auto& a : kAssets) budget += xv(a);
  m.add_constraint(budget == 1.0, "budget");
  for (const auto& a : kAssets) m.add_constraint(xv(a) >= 0.0, "nn_" + a);
}

/// 0.5 * x' * (L L') * x for a 3-vector x (i.e. 0.5 * ||L'x||^2).
double quad_risk_term(const double x[3]) {
  double u[3] = {0.0, 0.0, 0.0};
  for (int k = 0; k < 3; ++k)
    for (int i = 0; i < 3; ++i) u[k] += kL[i][k] * x[i];
  double v = 0.0;
  for (int k = 0; k < 3; ++k) v += u[k] * u[k];
  return 0.5 * v;
}

}  // namespace

TEST_CASE("AML SOC: ||x - a|| <= t minimizes to the projection onto a",
          "[aml][conic][soc]") {
  Model m("soc_projection");
  auto& S = m.add_set("S", {"i0"});
  auto& I = m.add_set("I", {"i1", "i2"});
  auto& tv = m.add_var("t", S, VarType::Continuous);  // free; t >= 0 is implied
  auto& xv = m.add_var("x", I, VarType::Continuous);  // free

  const double     a1 = 3.0, a2 = -4.0;
  const LinearExpr t  = tv("i0");
  const LinearExpr x1 = xv("i1");
  const LinearExpr x2 = xv("i2");

  m.add_soc_constraint(t, {x1 - a1, x2 - a2}, "shifted_norm");
  m.minimize(t);

  const SolveResult r = m.solve();
  REQUIRE(r.is_optimal());
  CHECK(r.objective_value == Approx(0.0).margin(1e-5));
  CHECK(r.var_value(tv("i0")) == Approx(0.0).margin(1e-5));
  CHECK(r.var_value(xv("i1")) == Approx(a1).margin(1e-5));
  CHECK(r.var_value(xv("i2")) == Approx(a2).margin(1e-5));
}

TEST_CASE("AML SOC: min ||x|| subject to d'x = 1 gives x = d/||d||^2",
          "[aml][conic][soc]") {
  Model m("soc_equality");
  auto& S = m.add_set("S", {"i0"});
  auto& I = m.add_set("I", {"i1", "i2"});
  auto& tv = m.add_var("t", S, VarType::Continuous);
  auto& xv = m.add_var("x", I, VarType::Continuous);

  const LinearExpr t  = tv("i0");
  const LinearExpr x1 = xv("i1");
  const LinearExpr x2 = xv("i2");

  m.add_soc_constraint(t, {x1, x2}, "norm");
  m.add_constraint(3.0 * x1 + 4.0 * x2 == 1.0, "dx_eq_1");
  m.minimize(t);

  const SolveResult r = m.solve();
  REQUIRE(r.is_optimal());
  CHECK(r.objective_value == Approx(0.2).margin(1e-5));  // t* = 1/||d||
  CHECK(r.var_value(xv("i1")) == Approx(0.12).margin(1e-5));
  CHECK(r.var_value(xv("i2")) == Approx(0.16).margin(1e-5));
}

TEST_CASE("AML SOC: Maximize objective is sign-flipped back in the result",
          "[aml][conic][soc][maximize]") {
  // max d'x over the unit ball: optimum ||d|| at x = d/||d||, d = (3, 4).
  Model m("soc_maximize");
  auto& S = m.add_set("S", {"i0"});
  auto& I = m.add_set("I", {"i1", "i2"});
  auto& tv = m.add_var("t", S, VarType::Continuous);
  auto& xv = m.add_var("x", I, VarType::Continuous);

  const LinearExpr t  = tv("i0");
  const LinearExpr x1 = xv("i1");
  const LinearExpr x2 = xv("i2");

  m.add_soc_constraint(t, {x1, x2}, "norm");
  m.add_constraint(t == 1.0, "t_eq_1");
  m.maximize(3.0 * x1 + 4.0 * x2);

  const SolveResult r = m.solve();
  REQUIRE(r.is_optimal());
  CHECK(r.objective_value == Approx(5.0).margin(1e-5));
  CHECK(r.var_value(xv("i1")) == Approx(0.6).margin(1e-5));
  CHECK(r.var_value(xv("i2")) == Approx(0.8).margin(1e-5));
}

TEST_CASE("AML conic: Markowitz QP matches its rotated-SOC epigraph",
          "[aml][conic][soc][markowitz]") {
  // (a) Quadratic objective 0.5*x'*Sigma*x - rho*mu'x via the QP path.
  Model mq("markowitz_qp");
  auto& Aq = mq.add_set("A", kAssets);
  auto& xq = mq.add_var("x", Aq, VarType::Continuous);
  add_markowitz_constraints(mq, xq);

  QuadExpr qobj;
  for (int i = 0; i < 3; ++i) {
    qobj += QuadExpr::sq(xq(kAssets[i]).id(), 0.5 * sigma_entry(i, i));
    for (int j = i + 1; j < 3; ++j)
      qobj += QuadExpr::bilinear(xq(kAssets[i]).id(), xq(kAssets[j]).id(),
                                 sigma_entry(i, j));
    qobj += LinearExpr::from_var(xq(kAssets[i]).id(), -kRho * kMu[i]);
  }
  mq.minimize(qobj);

  SolveOptions qp_opts;
  qp_opts.solver_name = "NativeLCQP";
  const SolveResult rq = mq.solve(qp_opts);
  REQUIRE(rq.is_optimal());

  // (b) Rotated-SOC epigraph: min theta - rho*mu'x with
  //     ||L'x||^2 <= 2*theta*1  <=>  0.5*x'*Sigma*x <= theta.
  Model ms("markowitz_soc");
  auto& As  = ms.add_set("A", kAssets);
  auto& Tss = ms.add_set("T", {"t0"});
  auto& xs  = ms.add_var("x", As, VarType::Continuous);
  auto& ths = ms.add_var("theta", Tss, VarType::Continuous);
  add_markowitz_constraints(ms, xs);

  std::vector<LinearExpr> u(3);
  for (int k = 0; k < 3; ++k)
    for (int i = 0; i < 3; ++i)
      if (kL[i][k] != 0.0)
        u[k] += LinearExpr::from_var(xs(kAssets[i]).id(), kL[i][k]);

  ms.add_rotated_soc_constraint(ths("t0"), LinearExpr::const_expr(1.0), u,
                                "risk");

  LinearExpr obj = ths("t0");
  for (int i = 0; i < 3; ++i)
    obj += LinearExpr::from_var(xs(kAssets[i]).id(), -kRho * kMu[i]);
  ms.minimize(obj);

  const SolveResult rs = ms.solve();
  REQUIRE(rs.is_optimal());

  // Both formulations reach the same optimum.
  CHECK(rs.objective_value == Approx(rq.objective_value).margin(1e-5));
  // The rotated-cone epigraph optimum sits on the cone boundary, so at
  // default IPM reltol the primal iterates of the two paths agree less
  // tightly than the objective (cvxopt shows the same behavior).
  for (int i = 0; i < 3; ++i)
    CHECK(rs.var_value(xs(kAssets[i])) ==
          Approx(rq.var_value(xq(kAssets[i]))).margin(5e-4));

  // theta* captures the quadratic risk term 0.5*x*'Sigma*x*.
  double x_star[3];
  for (int i = 0; i < 3; ++i) x_star[i] = rs.var_value(xs(kAssets[i]));
  CHECK(rs.var_value(ths("t0")) == Approx(quad_risk_term(x_star)).margin(1e-4));
}

TEST_CASE("AML PSD: min X11 s.t. [[X11, 2], [2, 1]] psd gives X11 = 4",
          "[aml][conic][psd]") {
  Model m("psd_det");
  auto& S  = m.add_set("S", {"i0"});
  auto& Xv = m.add_var("X11", S, VarType::Continuous);

  const LinearExpr X11 = Xv("i0");
  m.add_psd_constraint(2,
                       {{X11},
                        {LinearExpr::const_expr(2.0),
                         LinearExpr::const_expr(1.0)}},
                       "psd2");
  m.minimize(X11);

  const SolveResult r = m.solve();
  REQUIRE(r.is_optimal());
  CHECK(r.objective_value == Approx(4.0).margin(1e-5));
  CHECK(r.var_value(Xv("i0")) == Approx(4.0).margin(1e-5));
}

TEST_CASE("AML conic: mixed linear + SOC + PSD model solves and reports duals",
          "[aml][conic][soc][psd][duals]") {
  // min t + X11  s.t.  ||(x1, x2)|| <= t,  [[X11, 1], [1, 1]] psd,
  //                   x1 + x2 == 2,  x1 >= 0.5
  // Optimum: x1 = x2 = 1, t = sqrt(2), X11 = 1, obj = 1 + sqrt(2).
  Model m("mixed_conic");
  auto& S  = m.add_set("S", {"i0"});
  auto& I  = m.add_set("I", {"i1", "i2"});
  auto& tv = m.add_var("t", S, VarType::Continuous);
  auto& xv = m.add_var("x", I, VarType::Continuous);
  auto& Xv = m.add_var("X11", S, VarType::Continuous);

  const LinearExpr t   = tv("i0");
  const LinearExpr x1  = xv("i1");
  const LinearExpr x2  = xv("i2");
  const LinearExpr X11 = Xv("i0");

  const auto soc = m.add_soc_constraint(t, {x1, x2}, "norm");
  const auto psd = m.add_psd_constraint(2,
                                        {{X11},
                                         {LinearExpr::const_expr(1.0),
                                          LinearExpr::const_expr(1.0)}},
                                        "psd2");
  const auto eq  = m.add_constraint(x1 + x2 == 2.0, "balance");
  const auto nn  = m.add_constraint(x1 >= 0.5, "nn_x1");

  m.minimize(t + X11);

  const SolveResult r = m.solve();
  REQUIRE(r.is_optimal());
  CHECK(r.objective_value == Approx(1.0 + std::sqrt(2.0)).margin(1e-5));
  // The optimal PSD block [[1, 1], [1, 1]] is singular; on this boundary
  // optimum gap-driven termination at default reltol leaves ~1e-5..1e-4
  // primal error while the objective stays tight.
  CHECK(r.var_value(xv("i1")) == Approx(1.0).margin(1e-4));
  CHECK(r.var_value(xv("i2")) == Approx(1.0).margin(1e-4));
  CHECK(r.var_value(tv("i0")) == Approx(std::sqrt(2.0)).margin(1e-4));
  CHECK(r.var_value(Xv("i0")) == Approx(1.0).margin(1e-4));

  // ---- Conic duals ----
  REQUIRE(r.has_duals());

  // SOC dual has one entry per cone row (k+1 = 3) and lies in the dual cone.
  const auto zs = r.conic_dual_vals.find(soc.id());
  REQUIRE(zs != r.conic_dual_vals.end());
  REQUIRE(zs->second.size() == 3);
  const double z0 = zs->second[0], z1 = zs->second[1], z2 = zs->second[2];
  CHECK(z0 >= std::sqrt(z1 * z1 + z2 * z2) - 1e-6);

  // PSD dual is svec-packed (n(n+1)/2 = 3) and unpacks to a PSD matrix:
  // (v0, v1, v2) -> [[v0, v1/sqrt(2)], [v1/sqrt(2), v2]].
  const auto zp = r.conic_dual_vals.find(psd.id());
  REQUIRE(zp != r.conic_dual_vals.end());
  REQUIRE(zp->second.size() == 3);
  const double Z00 = zp->second[0];
  const double Z10 = zp->second[1] / std::sqrt(2.0);
  const double Z11 = zp->second[2];
  CHECK(Z00 >= -1e-7);
  CHECK(Z11 >= -1e-7);
  CHECK(Z00 * Z11 - Z10 * Z10 >= -1e-6);

  // Conic duals are not mixed into the scalar (linear row) dual map.
  CHECK_FALSE(r.dual(soc.id()).has_value());
  CHECK_FALSE(r.dual(psd.id()).has_value());

  // Equality row dual is present; the inactive inequality row has dual ~0.
  CHECK(r.dual(eq.id()).has_value());
  const auto d_nn = r.dual(nn.id());
  REQUIRE(d_nn.has_value());
  CHECK(std::fabs(*d_nn) < 1e-5);
}

TEST_CASE("AML conic: incompatible model features are rejected",
          "[aml][conic][reject]") {
  SECTION("integer variable + SOC constraint") {
    Model m("rej_int");
    auto& S  = m.add_set("S", {"i0"});
    auto& tv = m.add_var("t", S, VarType::Continuous);
    auto& xv = m.add_var("x", S, VarType::Integer, 0, 10);
    m.add_soc_constraint(tv("i0"), {xv("i0")});
    m.minimize(tv("i0"));
    CHECK_THROWS_AS(m.solve(), std::invalid_argument);
  }

  SECTION("quadratic objective + SOC constraint") {
    Model m("rej_qp");
    auto& S  = m.add_set("S", {"i0"});
    auto& tv = m.add_var("t", S, VarType::Continuous);
    auto& xv = m.add_var("x", S, VarType::Continuous);
    m.add_soc_constraint(tv("i0"), {xv("i0")});
    QuadExpr q = QuadExpr::sq(xv("i0").id());
    q += tv("i0");
    m.minimize(q);
    CHECK_THROWS_AS(m.solve(), std::invalid_argument);
  }

  SECTION("malformed PSD entries") {
    Model m("rej_psd_shape");
    auto& S  = m.add_set("S", {"i0"});
    auto& Xv = m.add_var("X", S, VarType::Continuous);
    // entries[1] must have exactly 2 elements.
    CHECK_THROWS_AS(
        m.add_psd_constraint(2, {{Xv("i0")}, {LinearExpr::const_expr(1.0)}}),
        std::invalid_argument);
    // entries.size() must equal order.
    CHECK_THROWS_AS(
        m.add_psd_constraint(3,
                             {{Xv("i0")},
                              {LinearExpr::const_expr(1.0),
                               LinearExpr::const_expr(1.0)}}),
        std::invalid_argument);
    // order must be >= 1.
    CHECK_THROWS_AS(m.add_psd_constraint(0, {}), std::invalid_argument);
  }

  SECTION("empty xs in SOC cones") {
    Model m("rej_empty_soc");
    auto& S  = m.add_set("S", {"i0"});
    auto& tv = m.add_var("t", S, VarType::Continuous);
    CHECK_THROWS_AS(m.add_soc_constraint(tv("i0"), {}), std::invalid_argument);
    CHECK_THROWS_AS(
        m.add_rotated_soc_constraint(tv("i0"), LinearExpr::const_expr(1.0), {}),
        std::invalid_argument);
  }
}
