/// Tests for the two-stage decomposition module (design doc:
/// docs/two_stage_decomposition_design.md).  Every decomposition optimum is
/// cross-checked against the monolithic extensive form and, where available,
/// an analytic optimum.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/SparseCore>

#include "mipsolvers/engine/decomposition/two_stage.hpp"

using Catch::Approx;
using namespace mipsolvers::engine;
using namespace mipsolvers::engine::decomposition;

namespace {

Eigen::SparseMatrix<double> sparse(int rows, int cols,
                                   const std::vector<Eigen::Triplet<double>>& t) {
  Eigen::SparseMatrix<double> m(rows, cols);
  m.setFromTriplets(t.begin(), t.end());
  m.makeCompressed();
  return m;
}

// Newsvendor-style capacity model shared by the stochastic and robust tests.
//   first stage : x in [0,10], cost 1  (capacity)
//   recourse s  : Q_s(x) = 10 * max(0, demand_s - x)
//                 via  min 10 y  s.t.  y >= demand_s - x,  y >= 0
// so  W=[1], T=[1], h=[demand_s], d=[10].
Recourse make_capacity_recourse(double demand, double prob) {
  Recourse r;
  r.d = Eigen::VectorXd::Constant(1, 10.0);
  r.T = sparse(1, 1, {{0, 0, 1.0}});
  r.W = sparse(1, 1, {{0, 0, 1.0}});
  r.h = Eigen::VectorXd::Constant(1, demand);
  r.vars = {{VarType::Continuous, 0.0, 1e20, "shortfall"}};
  r.probability = prob;
  return r;
}

FirstStage make_capacity_first_stage() {
  FirstStage f;
  f.c = Eigen::VectorXd::Constant(1, 1.0);
  f.A.resize(0, 1);
  f.b.resize(0);
  f.Aeq.resize(0, 1);
  f.beq.resize(0);
  f.vars = {{VarType::Continuous, 0.0, 10.0, "capacity"}};
  return f;
}

}  // namespace

// ── Stochastic: L-shaped multi-cut matches extensive form and the analytic
//    optimum (x*=8, obj=8; see design doc §1 example). ────────────────────────
TEST_CASE("Benders stochastic multi-cut matches extensive form", "[benders]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();
  model.scenarios = {make_capacity_recourse(4.0, 0.5),
                     make_capacity_recourse(8.0, 0.5)};

  const auto ext = solve_extensive_form_stochastic(model);
  INFO(ext.status);
  REQUIRE(ext.success);
  CHECK(ext.objective == Approx(8.0).margin(1e-6));

  BendersOptions opt;
  opt.cut_mode = BendersCutMode::MultiCut;
  const auto ben = solve_benders_stochastic(model, opt);
  INFO(ben.status);
  REQUIRE(ben.success);
  CHECK(ben.objective == Approx(ext.objective).margin(1e-6));
  CHECK(ben.x[0] == Approx(8.0).margin(1e-5));
  CHECK(ben.lower_bound <= ben.upper_bound + 1e-6);
  CHECK(ben.relative_gap <= 1e-6);
}

// ── Stochastic: single-cut aggregation reaches the same optimum. ─────────────
TEST_CASE("Benders stochastic single-cut matches extensive form", "[benders]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();
  model.scenarios = {make_capacity_recourse(4.0, 0.3),
                     make_capacity_recourse(8.0, 0.7)};

  const auto ext = solve_extensive_form_stochastic(model);
  INFO(ext.status);
  REQUIRE(ext.success);

  BendersOptions opt;
  opt.cut_mode = BendersCutMode::SingleCut;
  const auto ben = solve_benders_stochastic(model, opt);
  REQUIRE(ben.success);
  CHECK(ben.objective == Approx(ext.objective).margin(1e-6));
  CHECK(ben.lower_bound <= ben.upper_bound + 1e-6);
}

// ── Stochastic: three scenarios, both cut modes agree with the extensive form.
TEST_CASE("Benders stochastic three scenarios", "[benders]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();
  model.scenarios = {make_capacity_recourse(3.0, 0.25),
                     make_capacity_recourse(6.0, 0.5),
                     make_capacity_recourse(9.0, 0.25)};

  const auto ext = solve_extensive_form_stochastic(model);
  REQUIRE(ext.success);

  for (auto mode : {BendersCutMode::MultiCut, BendersCutMode::SingleCut}) {
    BendersOptions opt;
    opt.cut_mode = mode;
    const auto ben = solve_benders_stochastic(model, opt);
    REQUIRE(ben.success);
    CHECK(ben.objective == Approx(ext.objective).margin(1e-6));
  }
}

// ── Feasibility cuts: x=0 makes recourse infeasible; Benders must cut it off
//    and reach x=1, obj=1 (design doc §2.2). ──────────────────────────────────
TEST_CASE("Benders feasibility cuts recover a feasible first stage",
          "[benders]") {
  // first stage : x in {0,1}, cost 1
  // recourse    : y >= 1  and  y <= 2x    (infeasible at x=0)
  //   row1:  1*y >= 1 - 0*x
  //   row2: -1*y >= 0 - 2*x
  TwoStageModel model;
  FirstStage f;
  f.c = Eigen::VectorXd::Constant(1, 1.0);
  f.A.resize(0, 1);
  f.b.resize(0);
  f.Aeq.resize(0, 1);
  f.beq.resize(0);
  f.vars = {{VarType::Binary, 0.0, 1.0, "enable"}};
  model.first = f;

  Recourse r;
  r.d = Eigen::VectorXd::Zero(1);
  r.T = sparse(2, 1, {{1, 0, 2.0}});          // row2 couples x
  r.W = sparse(2, 1, {{0, 0, 1.0}, {1, 0, -1.0}});
  r.h = Eigen::VectorXd(2);
  r.h << 1.0, 0.0;
  r.vars = {{VarType::Continuous, 0.0, 1e20, "y"}};
  r.probability = 1.0;
  model.scenarios = {r};

  const auto ext = solve_extensive_form_stochastic(model);
  REQUIRE(ext.success);
  CHECK(ext.objective == Approx(1.0).margin(1e-6));

  BendersOptions opt;
  const auto ben = solve_benders_stochastic(model, opt);
  REQUIRE(ben.success);
  CHECK(ben.objective == Approx(1.0).margin(1e-6));
  CHECK(ben.x[0] == Approx(1.0).margin(1e-5));
  CHECK(ben.cuts_added >= 1);
}

// ── Robust: CCG matches the robust epigraph extensive form and the analytic
//    optimum (x*=8, obj=8). ────────────────────────────────────────────────────
TEST_CASE("CCG robust matches extensive form", "[ccg]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();
  model.scenarios = {make_capacity_recourse(4.0, 1.0),
                     make_capacity_recourse(6.0, 1.0),
                     make_capacity_recourse(8.0, 1.0)};

  const auto ext = solve_extensive_form_robust(model);
  INFO(ext.status);
  REQUIRE(ext.success);
  CHECK(ext.objective == Approx(8.0).margin(1e-6));

  CCGOptions opt;
  const auto ccg = solve_ccg_robust(model, opt);
  REQUIRE(ccg.success);
  CHECK(ccg.objective == Approx(ext.objective).margin(1e-6));
  CHECK(ccg.x[0] == Approx(8.0).margin(1e-5));
  CHECK(ccg.lower_bound <= ccg.upper_bound + 1e-6);
  CHECK(ccg.scenarios_generated <= 3);
}

// ── Robust with mixed-integer recourse: CCG stays exact (design doc §3.2)
//    while Benders is inapplicable. ───────────────────────────────────────────
TEST_CASE("CCG robust handles integer recourse", "[ccg]") {
  // first stage : x in [0,10], cost 1  (primary capacity)
  // recourse s  : binary backup unit of size 5 at cost 12
  //   min 12 y  s.t.  x + 5 y >= demand_s,  y in {0,1}
  //   => W=[5], T=[1], h=[demand_s]
  auto make_int_recourse = [](double demand) {
    Recourse r;
    r.d = Eigen::VectorXd::Constant(1, 12.0);
    r.T = sparse(1, 1, {{0, 0, 1.0}});
    r.W = sparse(1, 1, {{0, 0, 5.0}});
    r.h = Eigen::VectorXd::Constant(1, demand);
    r.vars = {{VarType::Binary, 0.0, 1.0, "backup"}};
    r.probability = 1.0;
    return r;
  };

  TwoStageModel model;
  model.first = make_capacity_first_stage();
  model.scenarios = {make_int_recourse(4.0), make_int_recourse(7.0),
                     make_int_recourse(9.0)};
  for (auto& scenario : model.scenarios) scenario.probability = 1.0 / 3.0;

  const auto ext = solve_extensive_form_robust(model);
  INFO(ext.status);
  REQUIRE(ext.success);

  CCGOptions opt;
  const auto ccg = solve_ccg_robust(model, opt);
  REQUIRE(ccg.success);
  CHECK(ccg.objective == Approx(ext.objective).margin(1e-5));

  // Benders must refuse integer recourse rather than silently relax it.
  const auto ben = solve_benders_stochastic(model, {});
  CHECK_FALSE(ben.success);
  CHECK(ben.status.find("continuous recourse") != std::string::npos);
}

// ── Integer L-shaped: stochastic program with integer recourse and a binary
//    first stage; must match the extensive form (design doc §3.3). ────────────
TEST_CASE("Integer L-shaped matches extensive form", "[benders][integer]") {
  // first stage : x1,x2 in {0,1}, open cost [5,8], capacities [4,7]
  // recourse s  : integer backup units of size 2 at cost 10
  //   min 10 y  s.t.  4 x1 + 7 x2 + 2 y >= demand_s,  y integer >= 0
  //   => W=[2], T=[4,7], h=[demand_s]
  auto make_int_recourse = [](double demand, double prob) {
    Recourse r;
    r.d = Eigen::VectorXd::Constant(1, 10.0);
    r.T = sparse(1, 2, {{0, 0, 4.0}, {0, 1, 7.0}});
    r.W = sparse(1, 1, {{0, 0, 2.0}});
    r.h = Eigen::VectorXd::Constant(1, demand);
    r.vars = {{VarType::Integer, 0.0, 100.0, "backup"}};
    r.probability = prob;
    return r;
  };

  TwoStageModel model;
  FirstStage f;
  f.c = Eigen::VectorXd(2);
  f.c << 5.0, 8.0;
  f.A.resize(0, 2);
  f.b.resize(0);
  f.Aeq.resize(0, 2);
  f.beq.resize(0);
  f.vars = {{VarType::Binary, 0.0, 1.0, "open1"},
            {VarType::Binary, 0.0, 1.0, "open2"}};
  model.first = f;
  model.scenarios = {make_int_recourse(3.0, 0.5), make_int_recourse(10.0, 0.5)};

  const auto ext = solve_extensive_form_stochastic(model);
  INFO(ext.status);
  REQUIRE(ext.success);
  CHECK(ext.objective == Approx(13.0).margin(1e-6));

  BendersOptions opt;
  opt.cut_mode = BendersCutMode::IntegerLShaped;
  const auto benl = solve_benders_stochastic(model, opt);
  REQUIRE(benl.success);
  CHECK(benl.objective == Approx(ext.objective).margin(1e-6));
  CHECK(benl.x[0] == Approx(1.0).margin(1e-5));
  CHECK(benl.x[1] == Approx(1.0).margin(1e-5));
  CHECK(benl.lower_bound <= benl.upper_bound + 1e-6);

  // Lagrangian/SDDiP cuts (design doc §3.6) reach the same optimum.
  BendersOptions lag;
  lag.cut_mode = BendersCutMode::Lagrangian;
  const auto benlag = solve_benders_stochastic(model, lag);
  INFO(benlag.status);
  REQUIRE(benlag.success);
  CHECK(benlag.objective == Approx(ext.objective).margin(1e-6));
  CHECK(benlag.lower_bound <= benlag.upper_bound + 1e-6);
}

// ── Integer L-shaped rejects a non-binary first stage. ───────────────────────
TEST_CASE("Integer L-shaped requires binary first stage", "[benders][integer]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();  // continuous first stage
  model.scenarios = {make_capacity_recourse(4.0, 1.0)};
  BendersOptions opt;
  opt.cut_mode = BendersCutMode::IntegerLShaped;
  const auto res = solve_benders_stochastic(model, opt);
  CHECK_FALSE(res.success);
  CHECK(res.status.find("binary first stage") != std::string::npos);
}

// ── Polyhedral robust: CCG with a KKT max–min oracle over a continuous
//    uncertainty set; cross-checked against finite CCG over the polytope
//    vertices, and against the analytic optimum (design doc §3.4). ────────────
TEST_CASE("Polyhedral CCG matches finite CCG over vertices", "[ccg][polyhedral]") {
  // demand(u) = 4 + u1 + u2,  Q(x,u) = 10*max(0, demand - x),  x in [0,20].
  //   W=[1], T=[1], h0=[4], P=[1,1], recourse y>=0.
  RobustRecourse rr;
  rr.d = Eigen::VectorXd::Constant(1, 10.0);
  rr.T = sparse(1, 1, {{0, 0, 1.0}});
  rr.W = sparse(1, 1, {{0, 0, 1.0}});
  rr.h0 = Eigen::VectorXd::Constant(1, 4.0);
  rr.P = sparse(1, 2, {{0, 0, 1.0}, {0, 1, 1.0}});
  rr.vars = {{VarType::Continuous, 0.0, 1e20, "shortfall"}};

  FirstStage f;
  f.c = Eigen::VectorXd::Constant(1, 1.0);
  f.A.resize(0, 1);
  f.b.resize(0);
  f.Aeq.resize(0, 1);
  f.beq.resize(0);
  f.vars = {{VarType::Continuous, 0.0, 20.0, "capacity"}};

  // Finite-CCG cross-check: one scenario per realization, demand = 4+u1+u2.
  auto finite_over = [&](const std::vector<std::pair<double, double>>& us) {
    TwoStageModel fm;
    fm.first = f;
    for (auto [u1, u2] : us) {
      Recourse r;
      r.d = rr.d;
      r.T = rr.T;
      r.W = rr.W;
      r.h = Eigen::VectorXd::Constant(1, 4.0 + u1 + u2);
      r.vars = rr.vars;
      r.probability = 1.0;
      fm.scenarios.push_back(r);
    }
    return solve_ccg_robust(fm, {});
  };

  SECTION("budget u1+u2<=3 caps the worst-case demand at 7") {
    PolyhedralRobustModel model;
    model.first = f;
    model.recourse = rr;
    model.uncertainty.u_lb = Eigen::VectorXd::Zero(2);
    model.uncertainty.u_ub = Eigen::VectorXd::Constant(2, 5.0);
    model.uncertainty.G = sparse(1, 2, {{0, 0, 1.0}, {0, 1, 1.0}});
    model.uncertainty.g = Eigen::VectorXd::Constant(1, 3.0);

    const auto poly = solve_ccg_polyhedral_robust(model, {});
    INFO(poly.status);
    REQUIRE(poly.success);
    CHECK(poly.objective == Approx(7.0).margin(1e-5));
    CHECK(poly.x[0] == Approx(7.0).margin(1e-4));

    // Worst case of a convex-in-u recourse is a vertex of {u>=0, u1+u2<=3}.
    const auto fin = finite_over({{0, 0}, {3, 0}, {0, 3}});
    REQUIRE(fin.success);
    CHECK(poly.objective == Approx(fin.objective).margin(1e-5));
  }

  SECTION("box [0,5]^2 without budget yields demand 14") {
    PolyhedralRobustModel model;
    model.first = f;
    model.recourse = rr;
    model.uncertainty.u_lb = Eigen::VectorXd::Zero(2);
    model.uncertainty.u_ub = Eigen::VectorXd::Constant(2, 5.0);
    model.uncertainty.G.resize(0, 2);  // no budget rows
    model.uncertainty.g.resize(0);

    const auto poly = solve_ccg_polyhedral_robust(model, {});
    INFO(poly.status);
    REQUIRE(poly.success);
    CHECK(poly.objective == Approx(14.0).margin(1e-5));

    const auto fin = finite_over({{0, 0}, {5, 0}, {0, 5}, {5, 5}});
    REQUIRE(fin.success);
    CHECK(poly.objective == Approx(fin.objective).margin(1e-5));
  }
}

// ── Parallel scenario subproblems must match the sequential result. ──────────
TEST_CASE("Parallel Benders matches sequential", "[benders][parallel]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();
  for (int k = 1; k <= 12; ++k)
    model.scenarios.push_back(make_capacity_recourse(static_cast<double>(k), 1.0 / 12.0));

  const auto ext = solve_extensive_form_stochastic(model);
  REQUIRE(ext.success);

  BendersOptions seq;
  seq.threads = 1;
  const auto r_seq = solve_benders_stochastic(model, seq);
  BendersOptions par;
  par.threads = 4;
  const auto r_par = solve_benders_stochastic(model, par);

  REQUIRE(r_seq.success);
  REQUIRE(r_par.success);
  CHECK(r_seq.objective == Approx(ext.objective).margin(1e-6));
  CHECK(r_par.objective == Approx(r_seq.objective).margin(1e-6));
}

// ── Parallel CCG oracle must match the sequential result. ────────────────────
TEST_CASE("Parallel CCG matches sequential", "[ccg][parallel]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();
  for (int k = 1; k <= 10; ++k)
    model.scenarios.push_back(make_capacity_recourse(static_cast<double>(k), 1.0));

  CCGOptions seq;
  seq.threads = 1;
  CCGOptions par;
  par.threads = 4;
  const auto r_seq = solve_ccg_robust(model, seq);
  const auto r_par = solve_ccg_robust(model, par);
  REQUIRE(r_seq.success);
  REQUIRE(r_par.success);
  CHECK(r_par.objective == Approx(r_seq.objective).margin(1e-6));
}

// ── In-out stabilization reaches the same optimum as pure Kelley. ────────────
TEST_CASE("Stabilized Benders matches unstabilized", "[benders][stabilization]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();  // continuous first stage
  model.scenarios = {make_capacity_recourse(4.0, 0.5),
                     make_capacity_recourse(8.0, 0.5)};

  BendersOptions opt;
  opt.stabilization_alpha = 0.6;  // in-out separation
  const auto stab = solve_benders_stochastic(model, opt);
  REQUIRE(stab.success);
  CHECK(stab.objective == Approx(8.0).margin(1e-6));
  CHECK(stab.lower_bound <= stab.upper_bound + 1e-6);
}

// ── Polyhedral CCG must reject integer recourse (KKT oracle is invalid). ─────
TEST_CASE("Polyhedral CCG rejects integer recourse", "[ccg][polyhedral]") {
  PolyhedralRobustModel model;
  model.first = make_capacity_first_stage();
  RobustRecourse rr;
  rr.d = Eigen::VectorXd::Constant(1, 10.0);
  rr.T = sparse(1, 1, {{0, 0, 1.0}});
  rr.W = sparse(1, 1, {{0, 0, 1.0}});
  rr.h0 = Eigen::VectorXd::Constant(1, 4.0);
  rr.P = sparse(1, 1, {{0, 0, 1.0}});
  rr.vars = {{VarType::Integer, 0.0, 100.0, "backup"}};  // integer recourse
  model.recourse = rr;
  model.uncertainty.u_lb = Eigen::VectorXd::Zero(1);
  model.uncertainty.u_ub = Eigen::VectorXd::Constant(1, 3.0);
  model.uncertainty.G.resize(0, 1);
  model.uncertainty.g.resize(0);

  const auto res = solve_ccg_polyhedral_robust(model, {});
  CHECK_FALSE(res.success);
  CHECK(res.status.find("continuous recourse") != std::string::npos);
}

// ── Contract: malformed models are rejected with a diagnostic, not a crash. ──
TEST_CASE("two-stage validation rejects malformed models", "[decomposition]") {
  TwoStageModel model;  // no first stage, no scenarios
  const auto r1 = solve_benders_stochastic(model, {});
  CHECK_FALSE(r1.success);
  CHECK_FALSE(r1.status.empty());
  CHECK(std::isnan(r1.objective));

  const auto r2 = solve_ccg_robust(model, {});
  CHECK_FALSE(r2.success);
  CHECK_FALSE(r2.status.empty());
  CHECK(std::isnan(r2.objective));

  TwoStageModel infinite_bound;
  infinite_bound.first = make_capacity_first_stage();
  infinite_bound.first.vars[0].lb =
      -std::numeric_limits<double>::infinity();
  infinite_bound.scenarios = {make_capacity_recourse(4.0, 1.0)};
  const auto r3 = solve_extensive_form_stochastic(infinite_bound);
  CHECK_FALSE(r3.success);
  CHECK(r3.status.find("invalid variable bounds") != std::string::npos);
  CHECK(std::isnan(r3.objective));
}

TEST_CASE("stochastic probabilities must define a simplex",
          "[benders][validation]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();
  model.scenarios = {make_capacity_recourse(4.0, 0.6),
                     make_capacity_recourse(8.0, 0.6)};

  const auto ext = solve_extensive_form_stochastic(model);
  CHECK_FALSE(ext.success);
  CHECK(ext.status.find("sum to one") != std::string::npos);
  CHECK(std::isnan(ext.objective));

  model.scenarios[0].probability = -0.2;
  model.scenarios[1].probability = 1.2;
  const auto ben = solve_benders_stochastic(model, {});
  CHECK_FALSE(ben.success);
  CHECK(ben.status.find("positive") != std::string::npos);
}

TEST_CASE("decomposition options reject invalid numerical contracts",
          "[decomposition][validation]") {
  TwoStageModel model;
  model.first = make_capacity_first_stage();
  model.scenarios = {make_capacity_recourse(4.0, 1.0)};

  BendersOptions bopt;
  bopt.threads = 0;
  const auto ben = solve_benders_stochastic(model, bopt);
  CHECK_FALSE(ben.success);
  CHECK(ben.status.find("threads") != std::string::npos);

  CCGOptions copt;
  copt.gap_tolerance = -1.0;
  const auto ccg = solve_ccg_robust(model, copt);
  CHECK_FALSE(ccg.success);
  CHECK(ccg.status.find("gap_tolerance") != std::string::npos);
}

TEST_CASE("polyhedral CCG enforces the KKT recourse domain",
          "[ccg][polyhedral][validation]") {
  PolyhedralRobustModel model;
  model.first = make_capacity_first_stage();
  model.recourse.d = Eigen::VectorXd::Constant(1, 10.0);
  model.recourse.T = sparse(1, 1, {{0, 0, 1.0}});
  model.recourse.W = sparse(1, 1, {{0, 0, 1.0}});
  model.recourse.h0 = Eigen::VectorXd::Constant(1, 4.0);
  model.recourse.P = sparse(1, 1, {{0, 0, 1.0}});
  model.uncertainty.u_lb = Eigen::VectorXd::Zero(1);
  model.uncertainty.u_ub = Eigen::VectorXd::Ones(1);
  model.uncertainty.G.resize(0, 1);
  model.uncertainty.g.resize(0);

  SECTION("free recourse is rejected") {
    model.recourse.vars = {{VarType::Continuous, -1e20, 1e20, "y"}};
    const auto res = solve_ccg_polyhedral_robust(model, {});
    CHECK_FALSE(res.success);
    CHECK(res.status.find("lb=0") != std::string::npos);
  }

  SECTION("finite recourse upper bounds are rejected") {
    model.recourse.vars = {{VarType::Continuous, 0.0, 10.0, "y"}};
    const auto res = solve_ccg_polyhedral_robust(model, {});
    CHECK_FALSE(res.success);
    CHECK(res.status.find("upper bound") != std::string::npos);
  }

  SECTION("malformed first-stage matrices are rejected") {
    model.recourse.vars = {{VarType::Continuous, 0.0, 1e20, "y"}};
    model.first.A.resize(0, 2);
    const auto res = solve_ccg_polyhedral_robust(model, {});
    CHECK_FALSE(res.success);
    CHECK(res.status.find("matrix width") != std::string::npos);
  }
}
