#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <vector>

#include <Eigen/SparseCore>

#include "mipsolvers/engine/decomposition/benders.hpp"

using Catch::Approx;
using namespace mipsolvers::engine;

namespace {

BendersModel make_generic_capacity_model() {
  // min 6*x + 10*y
  // s.t. y >= 1 - x, x binary, y >= 0.
  // The decomposition master is min 6*x + theta and the parameterized LP is
  // min 10*y s.t. -y <= -1 - (-1)*x.  Optimum: x=1, y=0, objective 6.
  BendersModel model;
  auto& master = model.master;
  master.linear_part.sense = Sense::Minimize;
  master.linear_part.c.resize(2);
  master.linear_part.c << 6.0, 1.0;
  master.linear_part.A.resize(0, 2);
  master.linear_part.b.resize(0);
  master.linear_part.Aeq.resize(0, 2);
  master.linear_part.beq.resize(0);
  master.linear_part.vars = {
      {VarType::Binary, 0.0, 1.0, "build"},
      {VarType::Continuous, 0.0, 1e20, "theta"}};
  master.binary_idx = {0};
  model.theta_col = 1;

  auto& sub = model.subproblem;
  sub.sense = Sense::Minimize;
  sub.c.resize(1);
  sub.c << 10.0;
  sub.A.resize(1, 1);
  sub.A.insert(0, 0) = -1.0;
  sub.A.makeCompressed();
  sub.b.resize(1);
  sub.b << -1.0;
  sub.Aeq.resize(0, 1);
  sub.beq.resize(0);
  sub.vars = {{VarType::Continuous, 0.0, 1e20, "recourse"}};

  model.coupling_ineq.resize(1, 2);
  model.coupling_ineq.insert(0, 0) = -1.0;
  model.coupling_ineq.makeCompressed();
  model.coupling_eq.resize(0, 2);
  return model;
}

BendersModel make_infeasible_then_feasible_model() {
  // min x + theta
  // s.t. y = 1, y <= 2*x, x binary, y >= 0.
  // The first master optimum x=0 has infeasible recourse.  The feasibility
  // cut must move the master to x=1, where y=1 and the objective is 1.
  BendersModel model;
  auto& master = model.master;
  master.linear_part.sense = Sense::Minimize;
  master.linear_part.c.resize(2);
  master.linear_part.c << 1.0, 1.0;
  master.linear_part.A.resize(0, 2);
  master.linear_part.b.resize(0);
  master.linear_part.Aeq.resize(0, 2);
  master.linear_part.beq.resize(0);
  master.linear_part.vars = {
      {VarType::Binary, 0.0, 1.0, "enable_capacity"},
      {VarType::Continuous, 0.0, 1e20, "theta"}};
  master.binary_idx = {0};
  model.theta_col = 1;

  auto& sub = model.subproblem;
  sub.sense = Sense::Minimize;
  sub.c = Eigen::VectorXd::Zero(1);
  sub.A.resize(1, 1);
  sub.A.insert(0, 0) = 1.0;
  sub.A.makeCompressed();
  sub.b.resize(1);
  sub.b << 0.0;
  sub.Aeq.resize(1, 1);
  sub.Aeq.insert(0, 0) = 1.0;
  sub.Aeq.makeCompressed();
  sub.beq.resize(1);
  sub.beq << 1.0;
  sub.vars = {{VarType::Continuous, 0.0, 1e20, "recourse"}};

  model.coupling_ineq.resize(1, 2);
  model.coupling_ineq.insert(0, 0) = -2.0;
  model.coupling_ineq.makeCompressed();
  model.coupling_eq.resize(1, 2);
  return model;
}

MIPModel make_monolithic_facility_model() {
  // Original column order is [y0, x1, y2, x3].  The requested master order is
  // deliberately [x3, x1] to exercise the explicit mapping contract.
  //
  // min y0 + 5*x1 + 2*y2 + x3
  // s.t. x1 + x3 <= 1
  //      y0 + y2 + 2*x1 + 3*x3 >= 4
  MIPModel model;
  auto& lp = model.linear_part;
  lp.sense = Sense::Minimize;
  lp.c.resize(4);
  lp.c << 1.0, 5.0, 2.0, 1.0;
  lp.vars = {
      {VarType::Continuous, 0.0, 1.0, "flow_a"},
      {VarType::Binary, 0.0, 1.0, "facility_a"},
      {VarType::Continuous, 0.0, 1.0, "flow_b"},
      {VarType::Binary, 0.0, 1.0, "facility_b"}};
  lp.A.resize(2, 4);
  std::vector<Eigen::Triplet<double>> terms = {
      {0, 1, 1.0}, {0, 3, 1.0}, {1, 0, 1.0},
      {1, 1, 2.0}, {1, 2, 1.0}, {1, 3, 3.0}};
  lp.A.setFromTriplets(terms.begin(), terms.end());
  lp.b.resize(2);
  lp.b << 1.0, std::numeric_limits<double>::infinity();
  lp.row_lhs.resize(2);
  lp.row_lhs << -std::numeric_limits<double>::infinity(), 4.0;
  lp.Aeq.resize(0, 4);
  lp.beq.resize(0);
  model.binary_idx = {1, 3};
  return model;
}

}  // namespace

TEST_CASE("Generic Benders module solves a parameterized MILP plus LP",
          "[benders][integration]") {
  BendersOptions options;
  options.max_iterations = 20;
  options.time_limit_sec = 30.0;
  options.gap_tolerance = 1e-8;
  options.master_options.use_vendored_highs_lp_kernel = true;
  options.master_options.gap_tol = 0.0;
  options.master_options.root_cut_rounds = 0;
  options.cold_master_cut_rounds = 0;
  options.warm_master_cut_rounds = 0;
  options.master_options.use_feasibility_pump = false;
  options.master_options.use_progressive_rounding = false;
  options.master_options.enable_feasibility_jump = false;
  options.subproblem_options.max_iter = 1000;
  options.subproblem_options.feasibility_tol = 1e-9;
  options.subproblem_options.optimality_tol = 1e-9;

  const BendersResult result =
      solve_benders(make_generic_capacity_model(), options);

  INFO("status=" << result.status << " iterations=" << result.stats.iterations
                 << " cuts=" << result.stats.cuts_added
                 << " lower=" << result.lower_bound
                 << " upper=" << result.upper_bound
                 << " gap=" << result.relative_gap);
  REQUIRE(result.success);
  REQUIRE(result.has_incumbent);
  REQUIRE(result.master_x.size() == 2);
  REQUIRE(result.subproblem_x.size() == 1);
  CHECK(result.objective == Approx(6.0).margin(1e-6));
  CHECK(result.master_x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.subproblem_x[0] == Approx(0.0).margin(1e-6));
  CHECK(result.stats.cuts_added > 0);
  CHECK(result.stats.coupled_master_variables == 1);
  CHECK(result.stats.coupling_nonzeros == 1);
  CHECK(result.stats.subproblem_solves > 0);
  CHECK(result.stats.master_lp_solves > 0);
}

TEST_CASE("Generic Benders module rejects malformed coupling dimensions",
          "[benders][unit]") {
  BendersModel model = make_generic_capacity_model();
  model.coupling_ineq.resize(2, 2);
  const BendersResult result = solve_benders(std::move(model));
  CHECK_FALSE(result.success);
  CHECK(result.status.find("coupling dimensions") != std::string::npos);
}

TEST_CASE("Benders rebuilds recourse after an infeasible Phase-I solve",
          "[benders][integration][regression]") {
  BendersOptions options;
  options.max_iterations = 20;
  options.time_limit_sec = 30.0;
  options.gap_tolerance = 1e-8;
  options.master_options.use_vendored_highs_lp_kernel = true;
  options.master_options.gap_tol = 0.0;
  options.master_options.root_cut_rounds = 0;
  options.cold_master_cut_rounds = 0;
  options.warm_master_cut_rounds = 0;
  options.master_options.use_feasibility_pump = false;
  options.master_options.use_progressive_rounding = false;
  options.master_options.enable_feasibility_jump = false;
  options.subproblem_options.max_iter = 1000;
  options.subproblem_options.feasibility_tol = 1e-9;
  options.subproblem_options.optimality_tol = 1e-9;

  const BendersResult result =
      solve_benders(make_infeasible_then_feasible_model(), options);

  INFO("status=" << result.status << " iterations=" << result.stats.iterations
                 << " cuts=" << result.stats.cuts_added
                 << " lower=" << result.lower_bound
                 << " upper=" << result.upper_bound
                 << " gap=" << result.relative_gap);
  REQUIRE(result.success);
  REQUIRE(result.has_incumbent);
  REQUIRE(result.master_x.size() == 2);
  REQUIRE(result.subproblem_x.size() == 1);
  CHECK(result.objective == Approx(1.0).margin(1e-6));
  CHECK(result.master_x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.subproblem_x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.stats.iterations >= 2);
  CHECK(result.stats.cuts_added >= 1);
  CHECK(result.stats.subproblem_solves >= 2);
}

TEST_CASE("Generic partitioner extracts a compact model from a monolithic MILP",
          "[benders][partition][integration]") {
  BendersPartition partition =
      partition_benders_model(make_monolithic_facility_model(), {3, 1});
  REQUIRE(partition.success);
  REQUIRE(partition.master_to_original == std::vector<int>{3, 1});
  REQUIRE(partition.subproblem_to_original == std::vector<int>{0, 2});
  CHECK(partition.model.theta_col == 2);
  CHECK(partition.model.master.linear_part.A.rows() == 1);
  CHECK(partition.model.subproblem.A.rows() == 1);
  CHECK(partition.model.coupling_ineq.rows() == 1);
  CHECK(partition.model.coupling_ineq.cols() == 3);
  CHECK(partition.model.coupling_ineq.nonZeros() == 2);
  CHECK(partition.model.master.linear_part.row_lhs[0] ==
        -std::numeric_limits<double>::infinity());
  CHECK(partition.model.subproblem.row_lhs[0] == Approx(4.0));

  BendersOptions options;
  options.max_iterations = 20;
  options.time_limit_sec = 30.0;
  options.gap_tolerance = 1e-8;
  options.master_options.use_vendored_highs_lp_kernel = true;
  options.master_options.gap_tol = 0.0;
  options.master_options.root_cut_rounds = 0;
  options.cold_master_cut_rounds = 0;
  options.warm_master_cut_rounds = 0;
  options.use_binary_no_good_fallback = false;
  options.master_options.use_feasibility_pump = false;
  options.master_options.use_progressive_rounding = false;
  options.master_options.enable_feasibility_jump = false;

  const BendersResult result =
      solve_benders(std::move(partition.model), options);
  INFO("status=" << result.status << " iterations=" << result.stats.iterations
                 << " cuts=" << result.stats.cuts_added
                 << " lower=" << result.lower_bound
                 << " upper=" << result.upper_bound
                 << " gap=" << result.relative_gap);
  REQUIRE(result.success);
  REQUIRE(result.has_incumbent);
  CHECK(result.objective == Approx(2.0).margin(1e-6));
  CHECK(result.master_x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.master_x[1] == Approx(0.0).margin(1e-6));
  CHECK(result.subproblem_x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.subproblem_x[1] == Approx(0.0).margin(1e-6));
}

TEST_CASE("Generic partitioner rejects integer recourse",
          "[benders][partition][unit]") {
  BendersPartition partition =
      partition_benders_model(make_monolithic_facility_model(), {3});
  CHECK_FALSE(partition.success);
  CHECK(partition.status.find("non-continuous") != std::string::npos);
}
