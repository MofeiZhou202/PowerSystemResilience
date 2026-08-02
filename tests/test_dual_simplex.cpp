/// test_dual_simplex.cpp
/// Tests for the native dual-simplex LP kernel via solve_lp_with_basis().
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Sparse>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <type_traits>

#include "Highs.h"

#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/detail/bc_solver_dispatch.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/hfactor_backend.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual_core.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual/factor.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual/pricing.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual/primal_pricing.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual/state.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

class EnvVarGuard {
 public:
  EnvVarGuard(const char* name, const char* value) : name_(name) {
    if (const char* previous = std::getenv(name)) {
      had_previous_ = true;
      previous_ = previous;
    }
    ::setenv(name, value, 1);
  }

  ~EnvVarGuard() {
    if (had_previous_) {
      ::setenv(name_.c_str(), previous_.c_str(), 1);
    } else {
      ::unsetenv(name_.c_str());
    }
  }

 private:
  std::string name_;
  std::string previous_;
  bool had_previous_{false};
};

double solve_with_highs(const LPModel& lp) {
  const int cols = static_cast<int>(lp.vars.size());
  const int ineq_rows = static_cast<int>(lp.A.rows());
  const int eq_rows = static_cast<int>(lp.Aeq.rows());
  const int rows = ineq_rows + eq_rows;
  std::vector<double> costs(static_cast<size_t>(cols));
  std::vector<double> lower(static_cast<size_t>(cols));
  std::vector<double> upper(static_cast<size_t>(cols));
  for (int j = 0; j < cols; ++j) {
    costs[static_cast<size_t>(j)] = lp.c[j];
    lower[static_cast<size_t>(j)] = lp.vars[static_cast<size_t>(j)].lb;
    upper[static_cast<size_t>(j)] = lp.vars[static_cast<size_t>(j)].ub;
  }
  std::vector<double> row_lower(static_cast<size_t>(rows), -kHighsInf);
  std::vector<double> row_upper(static_cast<size_t>(rows), kHighsInf);
  for (int i = 0; i < ineq_rows; ++i) {
    row_upper[static_cast<size_t>(i)] = lp.b[i];
  }
  for (int i = 0; i < eq_rows; ++i) {
    row_lower[static_cast<size_t>(ineq_rows + i)] = lp.beq[i];
    row_upper[static_cast<size_t>(ineq_rows + i)] = lp.beq[i];
  }
  std::vector<HighsInt> start(static_cast<size_t>(cols + 1));
  std::vector<HighsInt> index;
  std::vector<double> value;
  for (int j = 0; j < cols; ++j) {
    start[static_cast<size_t>(j)] = static_cast<HighsInt>(index.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
      index.push_back(it.row());
      value.push_back(it.value());
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
      index.push_back(ineq_rows + it.row());
      value.push_back(it.value());
    }
  }
  start[static_cast<size_t>(cols)] = static_cast<HighsInt>(index.size());

  Highs highs;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("solver", "simplex");
  const auto pass = highs.passModel(
      cols, rows, static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMinimize), 0.0, costs.data(),
      lower.data(), upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data());
  REQUIRE(pass != HighsStatus::kError);
  REQUIRE(highs.run() == HighsStatus::kOk);
  REQUIRE(highs.getModelStatus() == HighsModelStatus::kOptimal);
  return highs.getInfo().objective_function_value;
}

LPModel make_degenerate_transport_lp(int side) {
  const int cols = side * side;
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(2 * side - 1, cols);
  lp.beq = Eigen::VectorXd::Ones(2 * side - 1);
  lp.vars.reserve(static_cast<size_t>(cols));
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(2 * cols));
  for (int source = 0; source < side; ++source) {
    for (int sink = 0; sink < side; ++sink) {
      const int col = source * side + sink;
      triplets.emplace_back(source, col, 1.0);
      if (sink < side - 1) triplets.emplace_back(side + sink, col, 1.0);
      lp.c[col] = source == sink ? 0.0
                                 : 1.0 + 1e-4 * ((source + 3 * sink) % side);
      // Keep the root optimum strictly inside the upper bound so every
      // positive transportation arc must be basic. Later bound reductions
      // therefore create a genuine primal violation in the inherited basis.
      lp.vars.push_back({VarType::Continuous, 0.0, 2.0});
    }
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();
  return lp;
}

SimplexOptions native_simplex_options() {
  SimplexOptions options;
  options.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  return options;
}

}  // namespace

TEST_CASE("LP kernel defaults to HiGHS", "[dual_simplex][dispatch]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Ones(1);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 3.0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 2.0, 3.0});

  const auto result = solve_lp_with_basis(lp, SimplexOptions{});
  REQUIRE(result.result.stats.success);
  CHECK(result.result.stats.solver_name == "VendoredHighsLpKernel");
  CHECK(result.result.stats.objective == Approx(2.0));
}

TEST_CASE("Short LP budgets do not relabel immediate failures as timeouts",
          "[dual_simplex][status][regression]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.A.resize(2, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(1, 0) = -1.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 0.0, -1.0;
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, -10.0, 10.0});

  BCOptions options;
  options.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  options.use_ipm_root = false;
  options.use_simplex_lp_nodes = true;
  options.time_limit_sec = 3.0;

  const auto start = std::chrono::steady_clock::now();
  const auto result =
      detail::solve_lp_relaxation(lp, nullptr, nullptr, options);
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

  CHECK_FALSE(result.primal.stats.success);
  CHECK(result.primal.stats.status != "Time limit");
  CHECK(elapsed < 1.0);
}

TEST_CASE("HiGHS infeasibility never falls through to native dual",
          "[dual_simplex][dispatch]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, -1.0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  const auto result = solve_lp_with_basis(lp, SimplexOptions{});
  CHECK_FALSE(result.result.stats.success);
  CHECK(result.result.stats.solver_name == "VendoredHighsLpKernel");
}

TEST_CASE("HiGHS persistent LP state reuses the model and rolls back failure",
          "[dual_simplex][persistent][transaction]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Constant(1, -1.0);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 10.0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0, 10.0});

  SimplexOptions options;
  options.lp_kernel_backend = LpKernelBackend::HiGHS;
  options.allow_persistent_lp_state = true;
  StandardFormLP root_sf = build_standard_form_lp(lp);
  auto root = solve_lp_from_sf(root_sf, options);
  REQUIRE(root.result.stats.success);
  REQUIRE(root.basis.cached_sparse_basis);
  CHECK(root.basis_inverse.size() == 0);
  Eigen::VectorXd inverse_row;
  REQUIRE(root.basis.cached_sparse_basis->basis_inverse_row(0, inverse_row));
  REQUIRE(inverse_row.size() == root_sf.A.rows());
  CHECK(inverse_row.allFinite());
  auto root_handle = root.basis.cached_sparse_basis->highs_handle();
  REQUIRE(root_handle);

  StandardFormLP child_sf = root_sf;
  Eigen::VectorXd child_lb = Eigen::VectorXd::Constant(1, 0.0);
  Eigen::VectorXd child_ub = Eigen::VectorXd::Constant(1, 4.0);
  update_standard_form_bounds(child_sf, lp, child_lb, child_ub);
  auto child = solve_lp_from_sf(child_sf, options, &root.basis);
  REQUIRE(child.result.stats.success);
  CHECK(child.result.stats.solver_name == "VendoredHighsPersistentLpKernel");
  CHECK(child.result.stats.objective == Approx(-4.0).margin(1e-9));
  REQUIRE(child.basis.cached_sparse_basis);
  CHECK(child.basis.cached_sparse_basis->highs_handle() == root_handle);

  StandardFormLP rejected_sf = root_sf;
  rejected_sf.b[0] = -1.0;
  auto rejected = solve_lp_from_sf(rejected_sf, options, &root.basis);
  CHECK_FALSE(rejected.result.stats.success);

  const HighsLp& restored_lp = root_handle->getLp();
  REQUIRE(restored_lp.num_row_ == root_sf.A.rows());
  REQUIRE(restored_lp.num_col_ == root_sf.A.cols());
  CHECK(restored_lp.row_lower_[0] == Approx(root_sf.b[0]));
  CHECK(restored_lp.row_upper_[0] == Approx(root_sf.b[0]));
  CHECK(restored_lp.col_upper_[0] == Approx(root_sf.var_ub[0]));
  std::vector<double> rhs(static_cast<std::size_t>(root_sf.A.rows()), 1.0);
  std::vector<double> solution(rhs.size(), 0.0);
  CHECK(root_handle->getBasisSolve(rhs.data(), solution.data()) ==
        HighsStatus::kOk);

  child_ub[0] = 7.0;
  StandardFormLP next_sf = root_sf;
  update_standard_form_bounds(next_sf, lp, child_lb, child_ub);
  auto next = solve_lp_from_sf(next_sf, options, &root.basis);
  REQUIRE(next.result.stats.success);
  CHECK(next.result.stats.objective == Approx(-7.0).margin(1e-9));
  CHECK(next.basis.cached_sparse_basis->highs_handle() == root_handle);
}

TEST_CASE("Node LP dispatcher owns persistent state instead of queue nodes",
          "[dual_simplex][persistent][dispatcher][ownership]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Constant(1, -1.0);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 10.0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0, 10.0});

  detail::DispatcherConfig config;
  config.lp_kernel_backend = LpKernelBackend::HiGHS;
  config.allow_persistent_lp_state = true;
  detail::SolverDispatcher dispatcher(config, nullptr);

  StandardFormLP first_sf = build_standard_form_lp(lp);
  auto first = dispatcher.solve_no_fallback(
      first_sf, nullptr, detail::SolveContext::NodeLP);
  REQUIRE(first.result.stats.success);
  CHECK(first.form.A.rows() == first_sf.A.rows());
  CHECK_FALSE(first.basis.cached_sparse_basis);

  auto probe = dispatcher.solve_no_fallback(
      first_sf, &first.basis, detail::SolveContext::Probing);
  REQUIRE(probe.result.stats.success);
  CHECK(probe.form.A.rows() == 0);
  CHECK(probe.form.A.cols() == 0);

  Eigen::VectorXd lb = Eigen::VectorXd::Constant(1, 0.0);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(1, 4.0);
  StandardFormLP child_sf = first_sf;
  update_standard_form_bounds(child_sf, lp, lb, ub);
  auto child = dispatcher.solve_no_fallback(
      child_sf, &first.basis, detail::SolveContext::NodeLP);
  REQUIRE(child.result.stats.success);
  CHECK(child.result.stats.solver_name == "VendoredHighsPersistentLpKernel");
  CHECK(child.result.stats.objective == Approx(-4.0).margin(1e-9));
  CHECK_FALSE(child.basis.cached_sparse_basis);

  ub[0] = 7.0;
  StandardFormLP sibling_sf = first_sf;
  update_standard_form_bounds(sibling_sf, lp, lb, ub);
  auto sibling = dispatcher.solve_no_fallback(
      sibling_sf, &first.basis, detail::SolveContext::NodeLP);
  REQUIRE(sibling.result.stats.success);
  CHECK(sibling.result.stats.solver_name == "VendoredHighsPersistentLpKernel");
  CHECK(sibling.result.stats.objective == Approx(-7.0).margin(1e-9));
  CHECK_FALSE(sibling.basis.cached_sparse_basis);
}

TEST_CASE("HiGHS persistent LP honors the caller iteration limit",
          "[dual_simplex][persistent][iteration_limit]") {
  constexpr int rows = 64;
  constexpr int cols = 2 * rows;
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Ones(rows);
  lp.vars.reserve(cols);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    triplets.emplace_back(row, 2 * row, 1.0);
    triplets.emplace_back(row, 2 * row + 1, 1.0);
    lp.c[2 * row + 1] = 1.0;
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  SimplexOptions root_options;
  root_options.lp_kernel_backend = LpKernelBackend::HiGHS;
  root_options.allow_persistent_lp_state = true;
  root_options.max_iter = 1000;
  StandardFormLP root_sf = build_standard_form_lp(lp);
  auto root = solve_lp_from_sf(root_sf, root_options);
  REQUIRE(root.result.stats.success);
  REQUIRE(root.basis.cached_sparse_basis);
  const auto root_handle = root.basis.cached_sparse_basis->highs_handle();
  REQUIRE(root_handle);

  Eigen::VectorXd child_lb = Eigen::VectorXd::Zero(cols);
  Eigen::VectorXd child_ub = Eigen::VectorXd::Ones(cols);
  for (int row = 0; row < rows; ++row) {
    child_ub[2 * row] = 0.4;
  }
  StandardFormLP child_sf = root_sf;
  update_standard_form_bounds(child_sf, lp, child_lb, child_ub);
  SimplexOptions probe_options = root_options;
  probe_options.max_iter = 3;
  auto limited = solve_lp_from_sf(child_sf, probe_options, &root.basis);
  CHECK_FALSE(limited.result.stats.success);
  CHECK(limited.result.stats.solver_name ==
        "VendoredHighsPersistentLpKernel");
  CHECK(limited.result.stats.status.find("Iteration limit") !=
        std::string::npos);
  CHECK(limited.result.stats.iterations <= probe_options.max_iter);

  auto completed = solve_lp_from_sf(child_sf, root_options, &root.basis);
  REQUIRE(completed.result.stats.success);
  REQUIRE(completed.basis.cached_sparse_basis);
  CHECK(completed.basis.cached_sparse_basis->highs_handle() == root_handle);
}

TEST_CASE("Incremental standard-form bounds handle infinite transitions",
          "[dual_simplex][bounds][incremental]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Constant(1, 2.0);
  lp.A.resize(0, 1);
  lp.b.resize(0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars = {{VarType::Continuous,
              -std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity()}};

  StandardFormLP sf = build_standard_form_lp(lp);
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<BoundChangeInfo> tighten{
      {0, 0.0, true, -inf, -2.0},
      {0, 0.0, false, inf, 5.0},
  };
  REQUIRE(update_standard_form_bounds_incremental(sf, lp, tighten));
  CHECK(sf.lb_shift[0] == Approx(-2.0));
  CHECK(sf.var_ub[0] == Approx(7.0));
  CHECK(sf.objective_const == Approx(-4.0));

  const StandardFormLP before_reject = sf;
  const std::vector<BoundChangeInfo> stale{{0, 0.0, false, inf, 4.0}};
  CHECK_FALSE(update_standard_form_bounds_incremental(sf, lp, stale));
  CHECK(sf.lb_shift.isApprox(before_reject.lb_shift));
  CHECK(sf.var_ub.isApprox(before_reject.var_ub));
  CHECK(sf.objective_const == Approx(before_reject.objective_const));

  std::vector<BoundChangeInfo> relax{
      {0, 0.0, true, -2.0, -inf},
      {0, 0.0, false, 5.0, inf},
  };
  REQUIRE(update_standard_form_bounds_incremental(sf, lp, relax));
  CHECK(sf.lb_shift[0] == Approx(0.0));
  CHECK(std::isinf(sf.var_ub[0]));
  CHECK(sf.objective_const == Approx(0.0));
}

TEST_CASE("Standard-form bound transactions restore sparse state exactly",
          "[dual_simplex][bounds][transaction]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 2.0, -3.0;
  lp.A.resize(2, 2);
  lp.A.insert(0, 0) = 4.0;
  lp.A.insert(1, 1) = -2.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 8.0, 1.0;
  lp.Aeq.resize(1, 2);
  lp.Aeq.insert(0, 0) = 3.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Constant(1, 2.0);
  lp.vars = {{VarType::Continuous, 0.0, 5.0},
             {VarType::Continuous, -1.0, 4.0}};

  StandardFormLP sf = build_standard_form_lp(lp);
  ruiz_scale_standard_form(sf);
  const StandardFormLP original = sf;
  const std::vector<BoundChangeInfo> changes{
      {0, 1.0, true, 0.0, 1.0},
      {1, -2.0, false, 4.0, 2.0},
  };
  {
    StandardFormBoundTransaction transaction(sf);
    REQUIRE(transaction.apply(lp, changes));
    CHECK(transaction.active());
    CHECK(transaction.snapshot_value_count() == 7);
    CHECK(sf.lb_shift[0] != original.lb_shift[0]);
    CHECK(sf.var_ub[1] != original.var_ub[1]);
    REQUIRE(transaction.rollback());
    CHECK_FALSE(transaction.active());
  }
  CHECK((sf.lb_shift.array() == original.lb_shift.array()).all());
  CHECK((sf.var_ub.array() == original.var_ub.array()).all());
  CHECK((sf.b.array() == original.b.array()).all());
  CHECK(sf.objective_const == original.objective_const);

  {
    StandardFormBoundTransaction transaction(sf);
    REQUIRE(transaction.apply(lp, changes));
  }
  CHECK((sf.lb_shift.array() == original.lb_shift.array()).all());
  CHECK((sf.var_ub.array() == original.var_ub.array()).all());
  CHECK((sf.b.array() == original.b.array()).all());
  CHECK(sf.objective_const == original.objective_const);

  StandardFormBoundTransaction rejected(sf);
  const std::vector<BoundChangeInfo> stale{
      {0, 0.0, true, 9.0, 2.0}};
  CHECK_FALSE(rejected.apply(lp, stale));
  CHECK_FALSE(rejected.active());
  CHECK((sf.lb_shift.array() == original.lb_shift.array()).all());
  CHECK((sf.var_ub.array() == original.var_ub.array()).all());
  CHECK((sf.b.array() == original.b.array()).all());
  CHECK(sf.objective_const == original.objective_const);
}

TEST_CASE("Standard-form bound transactions match full bound updates",
          "[dual_simplex][bounds][transaction][differential]") {
  LPModel lp;
  lp.sense = Sense::Maximize;
  lp.c.resize(4);
  lp.c << 1.5, -2.0, 0.75, 4.0;
  lp.A.resize(3, 4);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = -3.0;
  lp.A.insert(0, 3) = 2.0;
  lp.A.insert(1, 0) = -2.0;
  lp.A.insert(1, 2) = 5.0;
  lp.A.insert(2, 1) = 4.0;
  lp.A.insert(2, 2) = -1.0;
  lp.A.insert(2, 3) = 0.5;
  lp.A.makeCompressed();
  lp.b.resize(3);
  lp.b << 7.0, 11.0, -2.0;
  lp.row_lhs.resize(3);
  lp.row_lhs << -4.0, -std::numeric_limits<double>::infinity(), -8.0;
  lp.Aeq.resize(2, 4);
  lp.Aeq.insert(0, 0) = 3.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.insert(1, 1) = -2.0;
  lp.Aeq.insert(1, 2) = 1.0;
  lp.Aeq.insert(1, 3) = 6.0;
  lp.Aeq.makeCompressed();
  lp.beq.resize(2);
  lp.beq << 5.0, -1.0;
  const double inf = std::numeric_limits<double>::infinity();
  lp.vars = {
      {VarType::Continuous, 0.0, 10.0},
      {VarType::Continuous, -inf, inf},
      {VarType::Continuous, -5.0, 8.0},
      {VarType::Continuous, 1.0, inf},
  };

  StandardFormLP actual = build_standard_form_lp(lp);
  ruiz_scale_standard_form(actual);
  const StandardFormLP original = actual;

  Eigen::VectorXd target_lb(4);
  target_lb << 2.0, -3.0, -5.0, 3.0;
  Eigen::VectorXd target_ub(4);
  target_ub << 9.0, 7.0, 4.0, inf;
  StandardFormLP expected = original;
  update_standard_form_bounds(expected, lp, target_lb, target_ub);

  const std::vector<BoundChangeInfo> changes{
      {0, 2.0, true, 0.0, 2.0},
      {0, -1.0, false, 10.0, 9.0},
      {1, 0.0, true, -inf, -3.0},
      {1, 0.0, false, inf, 7.0},
      {2, -4.0, false, 8.0, 4.0},
      {3, 1.0, true, 1.0, 2.0},
      {3, 1.0, true, 2.0, 3.0},
  };
  {
    StandardFormBoundTransaction transaction(actual);
    REQUIRE(transaction.apply(lp, changes));
    CHECK(actual.lb_shift.isApprox(expected.lb_shift, 1e-13));
    REQUIRE(actual.var_ub.size() == expected.var_ub.size());
    for (int column = 0; column < actual.var_ub.size(); ++column) {
      if (std::isfinite(expected.var_ub[column])) {
        CHECK(actual.var_ub[column] ==
              Approx(expected.var_ub[column]).epsilon(1e-13));
      } else {
        CHECK(actual.var_ub[column] == expected.var_ub[column]);
      }
    }
    CHECK(actual.b.isApprox(expected.b, 1e-13));
    CHECK(actual.objective_const ==
          Approx(expected.objective_const).epsilon(1e-13));
  }

  CHECK((actual.lb_shift.array() == original.lb_shift.array()).all());
  CHECK((actual.var_ub.array() == original.var_ub.array()).all());
  CHECK((actual.b.array() == original.b.array()).all());
  CHECK(actual.objective_const == original.objective_const);
}

TEST_CASE("Direct HiGHS cut rows commit or roll back atomically",
          "[dual_simplex][persistent][transaction][cuts]") {
  LPModel root_lp;
  root_lp.sense = Sense::Minimize;
  root_lp.c = Eigen::VectorXd::Constant(1, -1.0);
  root_lp.A.resize(1, 1);
  root_lp.A.insert(0, 0) = 1.0;
  root_lp.A.makeCompressed();
  root_lp.b = Eigen::VectorXd::Constant(1, 10.0);
  root_lp.Aeq.resize(0, 1);
  root_lp.beq.resize(0);
  root_lp.vars.push_back({VarType::Continuous, 0.0, 10.0});

  BCOptions options;
  options.use_simplex_lp_nodes = true;
  options.lp_kernel_backend = LpKernelBackend::HiGHS;
  auto root = detail::solve_lp_relaxation(root_lp, nullptr, nullptr, options);
  REQUIRE(root.primal.stats.success);
  REQUIRE(root.basis_hint);
  auto state = root.basis_hint->cached_sparse_basis;
  REQUIRE(state);
  REQUIRE(state->supports_incremental_rows());
  auto handle = state->highs_handle();
  REQUIRE(handle);

  LPModel cut_lp = root_lp;
  cut_lp.A.resize(2, 1);
  cut_lp.A.insert(0, 0) = 1.0;
  cut_lp.A.insert(1, 0) = 1.0;
  cut_lp.A.makeCompressed();
  cut_lp.b.resize(2);
  cut_lp.b << 10.0, 4.0;

  REQUIRE(state->begin_lp_transaction());
  auto cut = detail::solve_lp_relaxation(
      cut_lp, &root.primal.x, root.basis_hint.get(), options);
  REQUIRE(cut.primal.stats.success);
  CHECK(cut.primal.stats.objective == Approx(-4.0).margin(1e-9));
  CHECK(handle->getNumRow() == 2);
  REQUIRE(state->rollback_lp_transaction());
  CHECK(handle->getNumRow() == 1);
  CHECK(handle->getInfo().objective_function_value == Approx(-10.0).margin(1e-9));

  REQUIRE(state->begin_lp_transaction());
  cut = detail::solve_lp_relaxation(
      cut_lp, &root.primal.x, root.basis_hint.get(), options);
  REQUIRE(cut.primal.stats.success);
  REQUIRE(state->commit_lp_transaction());
  CHECK(handle->getNumRow() == 2);
}

// ─── 2-variable LP solved via dual simplex ────────────────────────────────
// min -3x - 2y  s.t.  x+y <= 4,  x >= 0,  y >= 0
// Optimal: x=4, y=0, obj=-12
TEST_CASE("DualSimplex: 2-variable LP", "[dual_simplex]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -3.0, -2.0;

  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 4.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  opts.max_iter = 500;

  auto res = solve_lp_with_basis(lp, opts);
  INFO("status=" << res.result.stats.status
                  << " iterations=" << res.result.stats.iterations);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(-12.0).margin(1e-5));
  CHECK(res.result.x[0] == Approx(4.0).margin(1e-5));
  CHECK(res.result.x[1] == Approx(0.0).margin(1e-5));
}

TEST_CASE("DualSimplex: publishes an audited primal ray for an unbounded LP",
          "[dual_simplex][unbounded][certificate]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -1.0, -1.0;
  lp.A.resize(0, 2);
  lp.b.resize(0);
  lp.Aeq.resize(1, 2);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = -1.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Zero(1);
  lp.vars.push_back(
      {VarType::Continuous, 0.0, std::numeric_limits<double>::infinity()});
  lp.vars.push_back(
      {VarType::Continuous, 0.0, std::numeric_limits<double>::infinity()});

  auto options = native_simplex_options();
  options.max_iter = 100;
  const auto result = solve_lp_with_basis(lp, options);
  INFO(result.result.stats.status);
  REQUIRE_FALSE(result.result.stats.success);
  REQUIRE(result.result.stats.status == "LP unbounded");
  REQUIRE(result.result.stats.has_unbounded_certificate);
  REQUIRE(result.result.primal_ray.size() == 2);
  CHECK((lp.Aeq * result.result.primal_ray).lpNorm<Eigen::Infinity>() <= 1e-10);
  CHECK(lp.c.dot(result.result.primal_ray) < -1e-8);
  CHECK(result.result.primal_ray.minCoeff() >= -1e-10);
}

TEST_CASE("DualSimplex: solves or certifies a row-free LP exactly",
          "[dual_simplex][unbounded][empty_rows]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -2.0, 3.0;
  lp.A.resize(0, 2);
  lp.b.resize(0);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0, 4.0});
  lp.vars.push_back({VarType::Continuous, 0.0, 5.0});

  auto options = native_simplex_options();
  auto bounded = solve_lp_with_basis(lp, options);
  REQUIRE(bounded.result.stats.success);
  CHECK(bounded.result.stats.objective == Approx(-8.0));
  CHECK(bounded.result.x[0] == Approx(4.0));
  CHECK(bounded.result.x[1] == Approx(0.0));

  lp.vars[0].ub = std::numeric_limits<double>::infinity();
  auto unbounded = solve_lp_with_basis(lp, options);
  REQUIRE_FALSE(unbounded.result.stats.success);
  CHECK(unbounded.result.stats.status == "LP unbounded");
  REQUIRE(unbounded.result.stats.has_unbounded_certificate);
  CHECK(unbounded.result.primal_ray[0] == Approx(1.0));
  CHECK(unbounded.result.primal_ray[1] == Approx(0.0));
}

TEST_CASE("DualSimplex: cost-shift crash certifies a cold dual start",
          "[dual_simplex][dual_crash]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Constant(1, -1.0);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 1.0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0,
                     std::numeric_limits<double>::infinity()});

  const StandardFormLP sf = build_standard_form_lp(lp);
  const SimplexOptions options = native_simplex_options();
  native_dual::Statistics statistics;
  native_dual::detail::State state;
  std::string failure;
  REQUIRE(native_dual::detail::initialize(
      state, sf, options, native_dual::detail::Phase::Two, nullptr,
      statistics, failure));

  CHECK_FALSE(native_dual::detail::normalize_nonbasic_moves(state, failure));
  REQUIRE(native_dual::detail::initialize_cost_shifted_dual_start(
      state, statistics, failure));
  const auto audit = native_dual::detail::audit(state, false, true, false);
  CHECK(audit.ok);
  CHECK(state.costs_shifted);
  CHECK(statistics.cost_shifts == 1);
  CHECK(statistics.dual_start_cost_shifts == 1);
  CHECK(state.original_cost[0] == Approx(1.0));
  CHECK(state.reduced_costs[0] == Approx(0.0).margin(1e-12));
}

TEST_CASE("DualSimplex: cold shift-start policy is bounded and overridable",
          "[dual_simplex][dual_crash][policy]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Constant(1, -1.0);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 1.0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0,
                     std::numeric_limits<double>::infinity()});

  const StandardFormLP sf = build_standard_form_lp(lp);
  auto options = native_simplex_options();
  native_dual::Statistics statistics;
  native_dual::detail::State state;
  std::string failure;
  REQUIRE(native_dual::detail::initialize(
      state, sf, options, native_dual::detail::Phase::Two, nullptr,
      statistics, failure));

  auto decision =
      native_dual::detail::decide_cost_shifted_dual_start(state, nullptr);
  CHECK(decision.try_shift_start);
  CHECK(decision.required_shift_lower_bound == 1);
  CHECK(decision.shift_threshold == 256);
  CHECK_FALSE(decision.adaptive_rejection);

  options.dual_shift_start_max_count = 1;
  options.dual_shift_start_max_fraction = 0.0;
  decision =
      native_dual::detail::decide_cost_shifted_dual_start(state, nullptr);
  CHECK_FALSE(decision.try_shift_start);
  CHECK(decision.required_shift_lower_bound == 1);
  CHECK(decision.shift_threshold == 1);
  CHECK(decision.adaptive_rejection);

  decision =
      native_dual::detail::decide_cost_shifted_dual_start(state, "on");
  CHECK(decision.try_shift_start);
  CHECK_FALSE(decision.adaptive_rejection);
  decision =
      native_dual::detail::decide_cost_shifted_dual_start(state, "off");
  CHECK_FALSE(decision.try_shift_start);
  CHECK_FALSE(decision.adaptive_rejection);
}

TEST_CASE("DualSimplex: dual Phase-I bounds are the translated HiGHS bounds",
          "[dual_simplex][dual_phase_one][contract]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -2.0, -3.0;
  lp.A.resize(2, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.insert(1, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 3.0, 10.0;
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0,
                     std::numeric_limits<double>::infinity()});
  lp.vars.push_back({VarType::Continuous, 0.0, 4.0});

  const StandardFormLP sf = build_standard_form_lp(lp);
  const Eigen::VectorXd anchor =
      native_dual::detail::make_dual_phase_one_anchor(sf);
  REQUIRE(anchor.size() == sf.A.cols());
  CHECK((sf.A * anchor - sf.b).lpNorm<Eigen::Infinity>() <= 1e-12);

  const auto bounds =
      native_dual::detail::make_dual_phase_one_bounds(sf, anchor);
  const int slack = sf.row_to_slack_col[0];
  REQUIRE(slack >= 0);
  CHECK(bounds.lower[0] == Approx(anchor[0]));
  CHECK(bounds.upper[0] == Approx(anchor[0] + 1.0));
  CHECK(bounds.enterable[0] == 1);
  CHECK(bounds.lower[1] == Approx(anchor[1]));
  CHECK(bounds.upper[1] == Approx(anchor[1]));
  CHECK(bounds.enterable[1] == 0);
  CHECK(bounds.lower[slack] == Approx(anchor[slack]));
  CHECK(bounds.upper[slack] == Approx(anchor[slack] + 1.0));
  CHECK(anchor[slack] == Approx(3.0));
}

TEST_CASE("DualSimplex: dual Phase-I anchor honors a scaled logical column",
          "[dual_simplex][dual_phase_one][contract]") {
  StandardFormLP sf;
  sf.A.resize(1, 2);
  sf.A.insert(0, 0) = 4.0;
  sf.A.insert(0, 1) = 2.0;
  sf.A.makeCompressed();
  sf.A_row = sf.A;
  sf.b = Eigen::VectorXd::Constant(1, 6.0);
  sf.c_max = Eigen::VectorXd::Zero(2);
  sf.var_ub = Eigen::VectorXd::Constant(
      2, std::numeric_limits<double>::infinity());
  sf.n_original = 1;
  sf.row_to_slack_col = {1};
  sf.row_to_surplus_col = {-1};
  sf.row_to_artificial_col = {-1};

  const Eigen::VectorXd anchor =
      native_dual::detail::make_dual_phase_one_anchor(sf);
  REQUIRE(anchor.size() == 2);
  CHECK(anchor[0] == Approx(0.0));
  CHECK(anchor[1] == Approx(3.0));
  CHECK((sf.A * anchor - sf.b).lpNorm<Eigen::Infinity>() <= 1e-12);
}

TEST_CASE("DualSimplex: dual Phase-I reconstruction uses anchor displacements",
          "[dual_simplex][dual_phase_one][contract]") {
  StandardFormLP sf;
  sf.A.resize(1, 2);
  sf.A.insert(0, 0) = 1.0;
  sf.A.insert(0, 1) = 0.1;
  sf.A.makeCompressed();
  sf.A_row = sf.A;
  sf.b = Eigen::VectorXd::Constant(1, 1e12 + 0.03);
  sf.c_max.resize(2);
  sf.c_max << 0.0, 1.0;
  sf.var_ub = Eigen::VectorXd::Constant(
      2, std::numeric_limits<double>::infinity());
  sf.n_original = 1;
  sf.row_to_slack_col = {1};
  sf.row_to_surplus_col = {-1};
  sf.row_to_artificial_col = {-1};

  mipsolvers::engine::SimplexBasis hint;
  hint.rows = 1;
  hint.cols = 2;
  hint.indices = {0};
  hint.at_upper = {0, 0};

  const SimplexOptions options = native_simplex_options();
  native_dual::Statistics statistics;
  native_dual::detail::State state;
  std::string failure;
  REQUIRE(native_dual::detail::initialize(
      state, sf, options, native_dual::detail::Phase::Two, &hint, statistics,
      failure));
  REQUIRE(native_dual::detail::initialize_dual_phase_one(state, failure));
  REQUIRE(state.move[1] == native_dual::detail::Move::Down);
  CHECK(state.x_basic[0] == Approx(-0.1).margin(1e-14));
  CHECK((sf.A * native_dual::detail::full_primal(state) - sf.b)
            .lpNorm<Eigen::Infinity>() <= options.feasibility_tol * sf.b[0]);
}

TEST_CASE("DualSimplex: dual Phase-I objective is the Native dual defect",
          "[dual_simplex][dual_phase_one][contract]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -2.0, -3.0;
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 3.0);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0,
                     std::numeric_limits<double>::infinity()});
  lp.vars.push_back({VarType::Continuous, 0.0, 4.0});

  const StandardFormLP sf = build_standard_form_lp(lp);
  const SimplexOptions options = native_simplex_options();
  native_dual::Statistics statistics;
  native_dual::detail::State state;
  std::string failure;
  REQUIRE(native_dual::detail::initialize(
      state, sf, options, native_dual::detail::Phase::Two, nullptr,
      statistics, failure));
  REQUIRE(native_dual::detail::initialize_dual_phase_one(state, failure));

  const auto audit = native_dual::detail::audit(state, false, true, false);
  INFO(failure);
  REQUIRE(audit.ok);
  const auto defect =
      native_dual::detail::original_dual_infeasibility_summary(state);
  CHECK(defect.count == 1);
  CHECK(defect.max == Approx(2.0));
  CHECK(defect.sum == Approx(2.0));
  CHECK(native_dual::detail::dual_phase_one_objective(state) ==
        Approx(defect.sum).epsilon(1e-12));
  CHECK(state.move[0] == native_dual::detail::Move::Down);
  CHECK(state.move[1] == native_dual::detail::Move::Fixed);
  CHECK((sf.A * native_dual::detail::full_primal(state) - sf.b)
            .lpNorm<Eigen::Infinity>() <= 1e-12);
}

TEST_CASE("DualSimplex: zero dual Phase-I defect transitions without a basis change",
          "[dual_simplex][dual_phase_one][contract]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 1.0, -2.0;
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 3.0);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0,
                     std::numeric_limits<double>::infinity()});
  lp.vars.push_back({VarType::Continuous, 0.0, 2.0});

  const StandardFormLP sf = build_standard_form_lp(lp);
  const SimplexOptions options = native_simplex_options();
  native_dual::Statistics statistics;
  native_dual::detail::State state;
  std::string failure;
  REQUIRE(native_dual::detail::initialize(
      state, sf, options, native_dual::detail::Phase::Two, nullptr,
      statistics, failure));
  REQUIRE(native_dual::detail::initialize_dual_phase_one(state, failure));
  CHECK(native_dual::detail::dual_phase_one_objective(state) ==
        Approx(0.0).margin(1e-14));
  CHECK(native_dual::detail::original_dual_infeasibility_summary(state).count ==
        0);

  const std::vector<int> basis_before = state.basis;
  native_dual::detail::DualInfeasibilitySummary transition;
  REQUIRE(native_dual::detail::transition_dual_phase_one_to_two(
      state, transition, failure));
  CHECK(state.phase == native_dual::detail::Phase::Two);
  CHECK(state.basis == basis_before);
  CHECK(transition.count == 0);
  CHECK(transition.max == Approx(0.0));
  const auto transitioned_audit =
      native_dual::detail::audit(state, false, true, false);
  INFO(transitioned_audit.failure);
  CHECK(transitioned_audit.ok);
  CHECK((sf.A * native_dual::detail::full_primal(state) - sf.b)
            .lpNorm<Eigen::Infinity>() <= 1e-12);
}

TEST_CASE("DualSimplex: cold solve bypasses dual Phase I with shifted start",
          "[dual_simplex][dual_start][telemetry]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -3.0, -2.0;
  lp.A.resize(2, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.insert(1, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 3.0, 10.0;
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0,
                     std::numeric_limits<double>::infinity()});
  lp.vars.push_back({VarType::Continuous, 0.0, 4.0});

  const StandardFormLP sf = build_standard_form_lp(lp);
  auto options = native_simplex_options();
  const native_dual::Result result = native_dual::solve(sf, options, nullptr);
  INFO(result.message);
  REQUIRE(result.status == native_dual::Status::Optimal);
  CHECK(sf.objective_const - result.max_objective ==
        Approx(solve_with_highs(lp)).margin(1e-9));
  CHECK(result.statistics.dual_phase_one_iterations == 0);
  CHECK(result.statistics.phase_transitions == 0);
  CHECK(result.statistics.dual_phase_one_initial_objective == Approx(0.0));
  CHECK(result.statistics.dual_phase_one_final_objective == Approx(0.0));
  CHECK(result.statistics.transition_dual_infeasibility_count == 0);
  CHECK(result.statistics.transition_max_dual_infeasibility <=
        options.optimality_tol);
  CHECK(result.statistics.dual_phase_one_terminal_reason == 0);
  CHECK(result.statistics.dual_start_cost_shifts == 1);
  CHECK(result.statistics.cost_shifts >=
        result.statistics.dual_start_cost_shifts);
  CHECK(result.statistics.dual_phase_one_time_sec >= 0.0);
  CHECK(result.statistics.dual_phase_two_time_sec >= 0.0);
  CHECK(result.statistics.dual_phase_two_iterations == 0);
  CHECK(result.statistics.primal_phase_one_iterations == 0);
  CHECK(result.statistics.primal_phase_two_iterations == 0);
  CHECK(result.statistics.primal_cleanup_iterations == 1);
  CHECK(result.statistics.cleanup_required == 1);
  CHECK(result.statistics.cleanup_avoided == 0);
  CHECK(result.statistics.dual_phase_one_iterations +
            result.statistics.dual_phase_two_iterations +
            result.statistics.primal_phase_one_iterations +
            result.statistics.primal_phase_two_iterations +
            result.statistics.primal_cleanup_iterations ==
        result.statistics.iterations);
}

TEST_CASE("IPM crossover basis can enter warm primal Phase I",
          "[dual_simplex][crossover][primal_phase_one]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -3.0, -2.0;
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 4.0);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  StandardFormLP sf = build_standard_form_lp(lp);
  mipsolvers::engine::SimplexBasis hint;
  hint.rows = 1;
  hint.cols = static_cast<int>(sf.A.cols());
  hint.indices = {1};
  hint.at_upper.assign(static_cast<std::size_t>(sf.A.cols()), 0);

  auto options = native_simplex_options();
  options.max_iter = 20;
  const auto strict = native_dual::solve(sf, options, hint);
  REQUIRE(strict.status == native_dual::Status::DualInfeasibleStart);

  options.allow_warm_primal_phase_one = true;
  const auto crossover = native_dual::solve(sf, options, hint);
  INFO("status=" << native_dual::status_name(crossover.status)
                  << " message=" << crossover.message);
  REQUIRE(crossover.status == native_dual::Status::Optimal);
  CHECK(crossover.statistics.iterations == 1);
  CHECK(crossover.max_objective == Approx(12.0).margin(1e-10));
}

// ─── 3-variable LP ────────────────────────────────────────────────────────
// min  x1 + x2 + x3
// s.t. x1 + x2 >= 2
//      x2 + x3 >= 2
//      all >= 0
// Optimal: x1=0, x2=2, x3=0, obj=2
TEST_CASE("IPM crash construction rejects an incrementally dependent column",
          "[dual_simplex][crossover][hfactor][hypersparse]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(3);
  lp.A.resize(2, 3);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 2.0;
  lp.A.insert(1, 2) = 1.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 13.0, 3.0;
  lp.Aeq.resize(0, 3);
  lp.beq.resize(0);
  for (int col = 0; col < 3; ++col) {
    lp.vars.push_back({VarType::Continuous, 0.0, 10.0});
  }

  StandardFormLP sf = build_standard_form_lp(lp);
  Eigen::VectorXd interior(3);
  interior << 5.0, 4.0, 3.0;
  mipsolvers::engine::SimplexBasis recovered;
  const detail::CrashBasisRecoveryStats stats =
      detail::recover_primal_activity_basis(
          sf, interior, nullptr, nullptr, recovered);

  REQUIRE(stats.rank_valid);
  REQUIRE(stats.passes_screen);
  CHECK(stats.attempted_columns == 3);
  CHECK(stats.stable_pivot_rejections == 1);
  CHECK(stats.structural_matches == 2);
  CHECK(stats.selected_swaps == 2);
  CHECK(stats.rank_repairs == 0);
  CHECK(stats.min_accepted_relative_pivot >= 0.1);

  HFactorBackend factor;
  REQUIRE(factor.factorize(
      sf.A, recovered.indices.data(), static_cast<int>(recovered.index_count())));
  CHECK(factor.rank_deficiency == 0);
}

TEST_CASE("DualSimplex: 3-variable LP with >= constraints", "[dual_simplex]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(3);
  lp.c << 1.0, 1.0, 1.0;

  // Convert x1+x2 >= 2 and x2+x3 >= 2 to -x <= -2
  Eigen::SparseMatrix<double> A(2, 3);
  A.insert(0, 0) = -1.0; A.insert(0, 1) = -1.0;
  A.insert(1, 1) = -1.0; A.insert(1, 2) = -1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << -2.0, -2.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  opts.max_iter = 500;

  auto res = solve_lp_with_basis(lp, opts);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(2.0).margin(1e-5));
}

// ─── Equality constrained LP ──────────────────────────────────────────────
// min  x + y   s.t.  x + y = 4,  x,y >= 0
// Optimal: obj=4, x+y=4
TEST_CASE("DualSimplex: equality constrained LP", "[dual_simplex]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 1.0, 1.0;

  lp.A.resize(0, 2);
  lp.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(1, 2);
  Aeq.insert(0, 0) = 1.0; Aeq.insert(0, 1) = 1.0;
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq.resize(1);
  lp.beq << 4.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  auto res = solve_lp_with_basis(lp, opts);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(4.0).margin(1e-5));
}

// ─── Basis round-trip: solve then warm-start ──────────────────────────────
TEST_CASE("DualSimplex: warm-start from previous basis", "[dual_simplex]") {
  // Same LP twice: second solve uses returned basis as hint.
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -1.0, -2.0;

  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 2.0;
  A.insert(1, 0) = 2.0; A.insert(1, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << 14.0, 14.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  auto first = solve_lp_with_basis(lp, opts);
  INFO("first status=" << first.result.stats.status
                        << " iterations=" << first.result.stats.iterations);
  REQUIRE(first.result.stats.success);

  // Re-solve with basis hint – should reach optimality in 0 pivots.
  auto second = solve_lp_with_basis(lp, opts, &first.basis);
  REQUIRE(second.result.stats.success);
  CHECK(second.result.stats.objective ==
        Approx(first.result.stats.objective).margin(1e-6));
}

TEST_CASE("DualSimplex: exact-breakpoint BFRT flips a complete prefix",
          "[dual_simplex][bfrt]") {
  // With x0 basic, x0=-2.5 is below its lower bound.  The three eligible
  // nonbasics have breakpoints 1, 2 and 3 and unit ranges.  A correct BFRT
  // flips x1 and x2 as complete earlier groups, then pivots x3 once.  A
  // one-at-a-time ratio test needs three basis exchanges on this model.
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(4);
  lp.c << 0.0, 1.0, 2.0, 3.0;
  lp.A.resize(0, 4);
  lp.b.resize(0);
  lp.Aeq.resize(1, 4);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = -1.0;
  lp.Aeq.insert(0, 2) = -1.0;
  lp.Aeq.insert(0, 3) = -1.0;
  lp.Aeq.makeCompressed();
  lp.beq.resize(1);
  lp.beq << -2.5;
  for (int j = 0; j < 4; ++j)
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});

  mipsolvers::engine::SimplexBasis hint;
  hint.rows = 1;
  hint.cols = 5;  // Four structural columns plus the equality artificial.
  hint.indices = {0};
  hint.at_upper.assign(5, 0);

  auto opts = native_simplex_options();
  opts.max_iter = 2;
  const auto solved = solve_lp_with_basis(lp, opts, &hint);
  INFO("status=" << solved.result.stats.status
                  << " iterations=" << solved.result.stats.iterations);
  REQUIRE(solved.result.stats.success);
  CHECK(solved.result.stats.iterations == 1);
  CHECK(solved.result.x[0] == Approx(0.0).margin(1e-10));
  CHECK(solved.result.x[1] == Approx(1.0).margin(1e-10));
  CHECK(solved.result.x[2] == Approx(1.0).margin(1e-10));
  CHECK(solved.result.x[3] == Approx(0.5).margin(1e-10));
  CHECK(solved.result.stats.objective == Approx(4.5).margin(1e-10));
}

TEST_CASE("DualSimplex: BFRT EXPAND accepts a near-exact capacity cover",
          "[dual_simplex][bfrt][contract]") {
  using namespace native_dual::detail;
  constexpr double alpha = 0.36289630446775362;

  StandardFormLP sf;
  sf.A.resize(1, 2);
  sf.A.insert(0, 0) = 1.0;
  sf.A.insert(0, 1) = alpha;
  sf.A.makeCompressed();

  State state;
  const SimplexOptions options = native_simplex_options();
  state.sf = &sf;
  state.options = &options;
  state.m = 1;
  state.n = 2;
  state.basis = {0};
  state.basic = {1, 0};
  state.move = {Move::Fixed, Move::Up};
  state.bounds.lower = Eigen::VectorXd::Zero(2);
  state.bounds.upper = Eigen::VectorXd::Ones(2);
  state.bounds.enterable = {0, 1};
  state.reduced_costs.resize(2);
  state.reduced_costs << 0.0, -alpha;

  Leaving leaving;
  leaving.row = 0;
  leaving.side = 1;
  leaving.violation = 0.36289630446775378;
  leaving.row_ep.dimension = 1;
  leaving.row_ep.index = {0};
  leaving.row_ep.value = {1.0};

  IndexedVector pivot_row;
  pivot_row.dimension = 2;
  pivot_row.index = {0, 1};
  pivot_row.value = {1.0, alpha};

  PivotTransaction transaction;
  std::string failure;
  REQUIRE(choose_entering_bfrt(state, leaving, pivot_row, transaction,
                               failure));
  INFO(failure);
  CHECK(transaction.positive_capacity < leaving.violation);
  CHECK(leaving.violation - transaction.positive_capacity < 1e-12);
  CHECK(transaction.entering.col == 1);
  CHECK(transaction.entering.alpha == Approx(alpha));
  CHECK(transaction.entering.theta == Approx(1.0));
}

TEST_CASE("DualSimplex: Phase-I BFRT certifies capacity by residual projection",
          "[dual_simplex][dual_phase_one][bfrt][contract]") {
  using namespace native_dual::detail;
  constexpr double alpha = 0.4;
  constexpr double shortage = 2e-11;

  StandardFormLP sf;
  sf.A.resize(1, 2);
  sf.A.insert(0, 0) = 1.0;
  sf.A.insert(0, 1) = alpha;
  sf.A.makeCompressed();
  sf.b = Eigen::VectorXd::Zero(1);

  const SimplexOptions options = native_simplex_options();
  State state;
  state.sf = &sf;
  state.options = &options;
  state.phase = Phase::DualOne;
  state.m = 1;
  state.n = 2;
  state.basis = {0};
  state.basic = {1, 0};
  state.move = {Move::Fixed, Move::Down};
  state.bounds.lower = Eigen::VectorXd::Zero(2);
  state.bounds.upper = Eigen::VectorXd::Ones(2);
  state.bounds.enterable = {0, 1};
  state.dual_phase_one_anchor = Eigen::VectorXd::Zero(2);
  state.x_basic = Eigen::VectorXd::Constant(1, -(alpha + shortage));
  state.reduced_costs.resize(2);
  state.reduced_costs << 0.0, alpha;

  Leaving leaving;
  leaving.row = 0;
  leaving.side = -1;
  leaving.violation = alpha + shortage;
  leaving.row_ep.dimension = 1;
  leaving.row_ep.index = {0};
  leaving.row_ep.value = {1.0};

  IndexedVector pivot_row;
  pivot_row.dimension = 2;
  pivot_row.index = {0, 1};
  pivot_row.value = {1.0, alpha};

  PivotTransaction transaction;
  std::string failure;
  REQUIRE(choose_entering_bfrt(state, leaving, pivot_row, transaction,
                               failure));
  INFO(failure);
  CHECK(transaction.stable_capacity == Approx(alpha));
  CHECK(transaction.entering.col == 1);
}

TEST_CASE("DualSimplex: stabilized warm solve rebuilds and cleans original cost",
          "[dual_simplex][stabilization][cleanup]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(4);
  lp.c << 0.0, 1.0, 2.0, 3.0;
  lp.A.resize(0, 4);
  lp.b.resize(0);
  lp.Aeq.resize(1, 4);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = -1.0;
  lp.Aeq.insert(0, 2) = -1.0;
  lp.Aeq.insert(0, 3) = -1.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Constant(1, -2.5);
  for (int j = 0; j < 4; ++j)
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});

  StandardFormLP sf = build_standard_form_lp(lp);
  mipsolvers::engine::SimplexBasis hint;
  hint.rows = 1;
  hint.cols = static_cast<int>(sf.A.cols());
  hint.indices = {0};
  hint.at_upper.assign(static_cast<std::size_t>(sf.A.cols()), 0);

  auto options = native_simplex_options();
  options.max_iter = 20;
  const auto result = native_dual::solve_phase2(sf, options, hint);
  INFO("status=" << native_dual::status_name(result.status)
                 << " message=" << result.message);
  REQUIRE(result.status == native_dual::Status::Optimal);
  CHECK(result.statistics.major_rebuilds >= 2);
  CHECK(result.statistics.max_cost_perturbation > 0.0);
  CHECK(result.statistics.devex_frameworks == 1);
  CHECK(result.statistics.dse_initialization_solves == 0);
  CHECK(result.statistics.harris_second_pass_candidates >= 1);
  CHECK(result.statistics.cleanup_passes == 1);
  CHECK(result.statistics.dual_phase_one_iterations == 0);
  CHECK(result.statistics.dual_phase_two_iterations == 1);
  CHECK(result.statistics.primal_phase_one_iterations == 0);
  CHECK(result.statistics.primal_phase_two_iterations == 0);
  CHECK(result.statistics.primal_cleanup_iterations == 0);
  CHECK(result.statistics.cleanup_required == 1);
  CHECK(result.statistics.cleanup_avoided == 0);
  CHECK(result.statistics.cleanup_time_sec >=
        result.statistics.primal_cleanup_time_sec);
  CHECK(result.statistics.dual_phase_one_iterations +
            result.statistics.dual_phase_two_iterations +
            result.statistics.primal_phase_one_iterations +
            result.statistics.primal_phase_two_iterations +
            result.statistics.primal_cleanup_iterations ==
        result.statistics.iterations);
  CHECK(result.max_objective == Approx(-4.5).margin(1e-10));
}

TEST_CASE("DualSimplex: batched DSE uses one existing INVERT",
          "[dual_simplex][stabilization][dse][factor_differential]") {
  StandardFormLP sf;
  Eigen::Matrix<double, 3, 6> dense;
  dense << 2.0, 1.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 3.0, 1.0, 0.0, 1.0, 0.0,
           1.0, 0.0, 4.0, 0.0, 0.0, 1.0;
  sf.A = dense.sparseView();
  sf.A.makeCompressed();
  std::vector<int> basis{0, 1, 2};
  mipsolvers::engine::native_dual::detail::BasisFactor factor(
      sf, {3, 4, 5});
  int repairs = 0;
  std::string failure;
  REQUIRE(factor.rebuild(basis, repairs, failure));
  const int rebuilds_before = factor.rebuild_count();

  const auto evidence = factor.compute_exact_edge_weights();
  INFO("residual=" << evidence.max_residual
                    << " limit=" << evidence.max_error_limit);
  REQUIRE(evidence.accepted);
  CHECK_FALSE(evidence.needs_rebuild);
  CHECK(factor.rebuild_count() == rebuilds_before);
  REQUIRE(evidence.weights.size() == 3);

  const Eigen::Matrix3d B = dense.leftCols<3>();
  const Eigen::Matrix3d inverse = B.inverse();
  for (int row = 0; row < 3; ++row) {
    CHECK(evidence.weights[static_cast<std::size_t>(row)] ==
          Approx(inverse.row(row).squaredNorm()).epsilon(2e-12));
  }
}

TEST_CASE("DualSimplex: cold pricing defaults to O(m+n) Devex",
          "[dual_simplex][performance][dse]") {
  using mipsolvers::engine::native_dual::Statistics;
  using mipsolvers::engine::native_dual::detail::BasisFactor;
  using mipsolvers::engine::native_dual::detail::State;

  StandardFormLP sf;
  Eigen::Matrix<double, 3, 6> dense;
  dense << 2.0, 1.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 3.0, 1.0, 0.0, 1.0, 0.0,
           1.0, 0.0, 4.0, 0.0, 0.0, 1.0;
  sf.A = dense.sparseView();
  sf.A.makeCompressed();

  auto initialize_weights = [&](bool exact) {
    auto options = native_simplex_options();
    options.exact_dse_initialization = exact;
    State state;
    state.sf = &sf;
    state.options = &options;
    state.m = 3;
    state.n = 6;
    state.basis = {0, 1, 2};
    state.factor = std::make_shared<BasisFactor>(sf, std::vector<int>{3, 4, 5});
    int repairs = 0;
    std::string failure;
    REQUIRE(state.factor->rebuild(state.basis, repairs, failure));
    Statistics statistics;
    REQUIRE(mipsolvers::engine::native_dual::detail::initialize_cold_edge_weights(
        state, statistics, failure));
    return statistics;
  };

  const Statistics scalable = initialize_weights(false);
  CHECK(scalable.devex_frameworks == 1);
  CHECK(scalable.dse_initialization_solves == 0);

  const Statistics exact = initialize_weights(true);
  CHECK(exact.devex_frameworks == 0);
  CHECK(exact.dse_initialization_solves == 3);
}

TEST_CASE("Standard form uses adaptive CSC and a shared-value row view",
          "[dual_simplex][performance][matrix_storage]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(3);
  lp.A.resize(2, 3);
  lp.A.insert(0, 0) = 2.0;
  lp.A.insert(1, 0) = -1.0;
  lp.A.insert(0, 2) = 4.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Ones(2);
  lp.Aeq.resize(0, 3);
  lp.beq.resize(0);
  for (int j = 0; j < 3; ++j) {
    lp.vars.push_back({VarType::Continuous, 0.0, 10.0});
  }

  StandardFormLP sf = build_standard_form_lp(lp);
  REQUIRE(sf.A.uses_32_bit_indices());
  REQUIRE(sf.A_row.shares_values_with(sf.A));
  CHECK(sf.A_row.uses_32_bit_row_offsets());

  const StandardFormIndex first_wide_dimension =
      static_cast<StandardFormIndex>((std::numeric_limits<int>::max)()) + 1;
  StandardColumnMatrix wide_rows(first_wide_dimension, 1);
  REQUIRE_FALSE(wide_rows.uses_32_bit_indices());
  CHECK(wide_rows.rows() == first_wide_dimension);
  CHECK(wide_rows.cols() == 1);

  const std::size_t full_row_copy_bytes =
      static_cast<std::size_t>(sf.A.rows() + 1) * sizeof(StandardFormIndex) +
      static_cast<std::size_t>(sf.A.nonZeros()) *
          (sizeof(StandardFormIndex) + sizeof(double));
  CHECK(sf.A_row.index_memory_bytes() < full_row_copy_bytes);

  StandardFormLP copied = sf;
  REQUIRE(copied.A.valuePtr() != sf.A.valuePtr());
  REQUIRE(copied.A_row.shares_values_with(copied.A));
  const int col = 0;
  const int row = static_cast<int>(
      StandardColumnMatrix::InnerIterator(copied.A, 0).row());
  const double source_value = sf.A.coeff(row, col);
  copied.A.valuePtr()[0] += 3.0;
  CHECK(copied.A_row.coeff(row, col) == Approx(source_value + 3.0));
  CHECK(sf.A_row.coeff(row, col) == Approx(source_value));

  SimplexOptions transient_options;
  transient_options.lp_kernel_backend = LpKernelBackend::HiGHS;
  const auto transient = solve_lp_from_sf(sf, transient_options);
  REQUIRE(transient.result.stats.success);
  CHECK(transient.form.A.rows() == 0);
  transient_options.retain_standard_form = true;
  const auto retained = solve_lp_from_sf(sf, transient_options);
  REQUIRE(retained.result.stats.success);
  CHECK(retained.form.A.rows() == sf.A.rows());
  CHECK(retained.form.A_row.shares_values_with(retained.form.A));
}

TEST_CASE("Standard form row view scales to an explicitly requested order",
          "[.lp_scale][matrix_storage]") {
  const char* requested = std::getenv("MIPSOLVERS_LP_SCALE_ROWS");
  if (requested == nullptr) SKIP("set MIPSOLVERS_LP_SCALE_ROWS to run");
  const long long parsed = std::strtoll(requested, nullptr, 10);
  REQUIRE(parsed > 0);
  REQUIRE(parsed <= (std::numeric_limits<int>::max)());
  const int dimension = static_cast<int>(parsed);

  const auto start = std::chrono::steady_clock::now();
  StandardColumnMatrix matrix(dimension, dimension);
  matrix.reserve(static_cast<StandardFormIndex>(dimension));
  for (int col = 0; col < dimension; ++col) {
    matrix.startVec(col);
    matrix.insertBackByOuterInner(col, col) = 1.0;
  }
  matrix.finalize();
  REQUIRE(matrix.isCompressed());
  REQUIRE(matrix.nonZeros() == dimension);

  StandardRowMatrix row_view;
  row_view = matrix;
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  INFO("dimension=" << dimension << " elapsed_sec=" << elapsed
                     << " row_index_bytes=" << row_view.index_memory_bytes());
  REQUIRE(row_view.shares_values_with(matrix));
  REQUIRE(row_view.uses_32_bit_row_offsets());
  CHECK(row_view.nonZeros() == dimension);
  CHECK(row_view.coeff(0, 0) == Approx(1.0));
  CHECK(row_view.coeff(dimension - 1, dimension - 1) == Approx(1.0));
  const std::size_t expected_max =
      (static_cast<std::size_t>(dimension) + 1) *
          sizeof(std::uint32_t) +
      2 * static_cast<std::size_t>(dimension) * sizeof(std::uint32_t);
  CHECK(row_view.index_memory_bytes() <= expected_max);
}

TEST_CASE("Cost journal stays sparse and deterministically densifies",
          "[dual_simplex][performance][cost_journal]") {
  using mipsolvers::engine::native_dual::detail::SparseCostJournal;

  SparseCostJournal sparse;
  sparse.reset(1024);
  sparse.add(17, 1.5);
  sparse.add(3, -2.0);
  sparse.add(17, 0.25);
  REQUIRE_FALSE(sparse.is_dense());
  CHECK(sparse.size() == 2);
  CHECK(sparse.at(17) == Approx(1.75));
  CHECK(sparse.at(3) == Approx(-2.0));
  CHECK(sparse.at(99) == Approx(0.0));
  CHECK(sparse.max_abs() == Approx(2.0));
  CHECK(sparse.memory_bytes() < 1024 * sizeof(double));
  std::vector<int> touched;
  sparse.for_each([&](int column, double) { touched.push_back(column); });
  REQUIRE(touched.size() == 2);
  CHECK(touched[0] == 17);
  CHECK(touched[1] == 3);
  sparse.add(17, -1.75);
  CHECK(sparse.size() == 1);
  sparse.set(3, 0.0);
  CHECK(sparse.empty());
  sparse.add(17, 2.5);
  CHECK(sparse.size() == 1);
  CHECK(sparse.at(17) == Approx(2.5));

  SparseCostJournal adaptive;
  adaptive.reset(64);
  for (int column = 0; column < 8; ++column) {
    adaptive.set(column, static_cast<double>(column + 1));
  }
  REQUIRE(adaptive.is_dense());
  CHECK(adaptive.size() == 8);
  CHECK(adaptive.at(7) == Approx(8.0));
  adaptive.clear();
  CHECK(adaptive.empty());
  CHECK(adaptive.max_abs() == Approx(0.0));
  adaptive.add(11, -4.0);
  CHECK(adaptive.at(11) == Approx(-4.0));
  adaptive.add(11, 4.0);
  CHECK(adaptive.empty());
}

TEST_CASE("Dense parallel PRICE agrees with the serial sparse product",
          "[dual_simplex][performance][parallel_price]") {
  using namespace mipsolvers::engine::native_dual::detail;
  constexpr int dimension = 4096;
  constexpr int entries_per_column = 16;
  StandardColumnMatrix A(dimension, dimension);
  A.reserve(static_cast<StandardFormIndex>(dimension) * entries_per_column);
  for (int column = 0; column < dimension; ++column) {
    A.startVec(column);
    for (int row = 0; row < entries_per_column; ++row) {
      A.insertBackByOuterInner(column, row) =
          0.25 + static_cast<double>((column + row) % 13) / 16.0;
    }
  }
  A.finalize();
  StandardRowMatrix A_row;
  A_row = A;

  IndexedVector y;
  y.dimension = dimension;
  y.index.reserve(dimension);
  y.value.reserve(dimension);
  for (int row = 0; row < dimension; ++row) {
    y.index.push_back(row);
    y.value.push_back(1.0 + static_cast<double>(row % 7) / 8.0);
  }

  IndexedVector serial;
  IndexedVector parallel;
  multiply_AT_indexed(A, A_row, y, serial, 1);
  multiply_AT_indexed(A, A_row, y, parallel, 4);
  REQUIRE(serial.index.size() == parallel.index.size());
  REQUIRE(serial.index.size() == dimension);

  if (const char* requested =
          std::getenv("MIPSOLVERS_LP_KERNEL_BENCH_REPEATS")) {
    const int repeats = std::max(1, std::atoi(requested));
    const auto serial_start = std::chrono::steady_clock::now();
    for (int repeat = 0; repeat < repeats; ++repeat) {
      multiply_AT_indexed(A, A_row, y, serial, 1);
    }
    const double serial_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - serial_start).count();
    const auto parallel_start = std::chrono::steady_clock::now();
    for (int repeat = 0; repeat < repeats; ++repeat) {
      multiply_AT_indexed(A, A_row, y, parallel, 4);
    }
    const double parallel_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - parallel_start).count();
    std::fprintf(stderr,
                 "[LP-KERNEL-BENCH] PRICE repeats=%d serial_sec=%.9f "
                 "parallel_sec=%.9f speedup=%.6f\n",
                 repeats, serial_sec, parallel_sec,
                 serial_sec / parallel_sec);
  }
  for (int column = 0; column < dimension; ++column) {
    CHECK(parallel.at(column) == serial.at(column));
  }
}

TEST_CASE("Parallel BFRT merge is independent of worker count",
          "[dual_simplex][performance][parallel_bfrt]") {
  using namespace mipsolvers::engine::native_dual::detail;
  constexpr int scan_count = 8192;
  constexpr int columns = scan_count + 1;
  StandardFormLP sf;
  sf.A.resize(1, columns);
  sf.A.makeCompressed();

  SimplexOptions options = native_simplex_options();
  State state;
  state.sf = &sf;
  state.options = &options;
  state.phase = Phase::Two;
  state.m = 1;
  state.n = columns;
  state.basis = {columns - 1};
  state.basic.assign(columns, 0);
  state.basic.back() = 1;
  state.move.assign(columns, Move::Up);
  state.move.back() = Move::Fixed;
  state.bounds.lower = Eigen::VectorXd::Zero(columns);
  state.bounds.upper = Eigen::VectorXd::Ones(columns);
  state.bounds.enterable.assign(columns, 1);
  state.reduced_costs = Eigen::VectorXd::Constant(columns, -1.0);
  state.reduced_costs[columns - 1] = 0.0;
  state.bfrt_error_coef.assign(columns, 0.0);

  Leaving leaving;
  leaving.row = 0;
  leaving.side = 1;
  leaving.violation = 0.5;
  leaving.row_ep.dimension = 1;
  leaving.row_ep.index = {0};
  leaving.row_ep.value = {1.0};
  IndexedVector pivot_row;
  pivot_row.dimension = columns;
  pivot_row.index.reserve(scan_count);
  pivot_row.value.reserve(scan_count);
  for (int column = 0; column < scan_count; ++column) {
    pivot_row.index.push_back(column);
    pivot_row.value.push_back(1.0);
  }

  PivotTransaction serial;
  PivotTransaction parallel;
  std::string failure;
  state.kernel_threads = 1;
  REQUIRE(choose_entering_bfrt(state, leaving, pivot_row, serial, failure));
  INFO(failure);
  state.kernel_threads = 4;
  REQUIRE(choose_entering_bfrt(state, leaving, pivot_row, parallel, failure));
  INFO(failure);

  if (const char* requested =
          std::getenv("MIPSOLVERS_LP_KERNEL_BENCH_REPEATS")) {
    const int repeats = std::max(1, std::atoi(requested));
    const auto measure = [&](int workers) {
      state.kernel_threads = workers;
      const auto start = std::chrono::steady_clock::now();
      for (int repeat = 0; repeat < repeats; ++repeat) {
        PivotTransaction measured;
        REQUIRE(choose_entering_bfrt(state, leaving, pivot_row, measured,
                                     failure));
      }
      return std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
    };
    const double serial_sec = measure(1);
    const double parallel_sec = measure(4);
    std::fprintf(stderr,
                 "[LP-KERNEL-BENCH] BFRT repeats=%d serial_sec=%.9f "
                 "parallel_sec=%.9f speedup=%.6f\n",
                 repeats, serial_sec, parallel_sec,
                 serial_sec / parallel_sec);
  }
  CHECK(parallel.entering.col == serial.entering.col);
  CHECK(parallel.entering.pivot == serial.entering.pivot);
  CHECK(parallel.entering.alpha == serial.entering.alpha);
  CHECK(parallel.entering.theta == serial.entering.theta);
  CHECK(parallel.positive_candidate_count == serial.positive_candidate_count);
  CHECK(parallel.certified_candidate_count ==
        serial.certified_candidate_count);
  CHECK(parallel.stable_candidate_count == serial.stable_candidate_count);
  CHECK(parallel.positive_capacity == serial.positive_capacity);
  CHECK(parallel.certified_capacity == serial.certified_capacity);
  CHECK(parallel.stable_capacity == serial.stable_capacity);
  CHECK(parallel.harris_second_pass_candidates ==
        serial.harris_second_pass_candidates);
  CHECK(parallel.flips.size() == serial.flips.size());
  CHECK(parallel.cost_shifts.size() == serial.cost_shifts.size());
}

TEST_CASE("Intermediate canonical residual audits are rate limited",
          "[dual_simplex][performance][audit]") {
  using namespace mipsolvers::engine::native_dual::detail;
  StandardFormLP sf;
  Eigen::Matrix<double, 2, 4> dense;
  dense << 2.0, 0.0, 1.0, 0.0,
           0.0, 3.0, 0.0, 1.0;
  sf.A = dense.sparseView();
  sf.A.makeCompressed();
  sf.A_row = sf.A;
  sf.b = Eigen::VectorXd::Ones(2);
  sf.c_max = Eigen::VectorXd::Zero(4);
  sf.var_ub = Eigen::VectorXd::Constant(
      4, std::numeric_limits<double>::infinity());

  auto options = native_simplex_options();
  options.intermediate_audit_interval = 32;
  State state;
  state.sf = &sf;
  state.options = &options;
  state.m = 2;
  state.n = 4;
  state.basis = {0, 1};
  state.bounds.lower = Eigen::VectorXd::Zero(4);
  state.bounds.upper = sf.var_ub;
  state.bounds.enterable.assign(4, 1);
  state.move.assign(4, Move::Up);
  state.cost = sf.c_max;
  state.original_cost = sf.c_max;
  state.factor = std::make_shared<BasisFactor>(sf, std::vector<int>{2, 3});
  int repairs = 0;
  std::string failure;
  REQUIRE(state.factor->rebuild(state.basis, repairs, failure));
  REQUIRE(reconstruct(state, failure));
  CHECK(state.canonical_residual_audits == 1);

  for (int rebuild = 2; rebuild < 32; ++rebuild) {
    REQUIRE(state.factor->rebuild(state.basis, repairs, failure));
  }
  REQUIRE(reconstruct(state, failure));
  CHECK(state.canonical_residual_audits == 1);
  REQUIRE(state.factor->rebuild(state.basis, repairs, failure));
  REQUIRE(reconstruct(state, failure));
  CHECK(state.canonical_residual_audits == 2);
  REQUIRE(reconstruct(state, failure, true, true, true));
  CHECK(state.canonical_residual_audits == 3);
}

TEST_CASE("Primal PSE recurrence matches an explicit basis exchange",
          "[dual_simplex][primal][pricing][pse]") {
  using namespace mipsolvers::engine::native_dual::detail;

  Eigen::Matrix3d B;
  B << 2.0, 1.0, 0.0,
       0.0, 1.0, 1.0,
       1.0, 0.0, 1.0;
  const Eigen::Vector3d aq(1.0, 3.0, 2.0);
  const Eigen::Vector3d aj(4.0, -1.0, 2.0);
  const int pivot_row = 1;
  const Eigen::Vector3d hq = B.fullPivLu().solve(aq);
  const Eigen::Vector3d hj = B.fullPivLu().solve(aj);
  const double pivot = hq[pivot_row];
  REQUIRE(std::abs(pivot) > 1e-12);

  Eigen::Matrix3d next_B = B;
  next_B.col(pivot_row) = aq;
  const Eigen::Vector3d next_hj = next_B.fullPivLu().solve(aj);
  const double lambda = hj[pivot_row] / pivot;
  const double mu = hq.dot(hj);
  const double updated = primal_pse_updated_weight(
      1.0 + hj.squaredNorm(), lambda, mu, hq.squaredNorm());
  CHECK(updated == Approx(1.0 + next_hj.squaredNorm()).epsilon(1e-12));

  const Eigen::Vector3d leaving_column = B.col(pivot_row);
  const Eigen::Vector3d next_leaving =
      next_B.fullPivLu().solve(leaving_column);
  CHECK(primal_pse_leaving_weight(hq.squaredNorm(), pivot) ==
        Approx(1.0 + next_leaving.squaredNorm()).epsilon(1e-12));
}

TEST_CASE("Primal PSE recurrence stays exact across consecutive exchanges",
          "[dual_simplex][primal][pricing][pse]") {
  using namespace mipsolvers::engine::native_dual::detail;

  Eigen::Matrix<double, 3, 6> A;
  A << 2.0, 1.0, 0.0, 1.0, 4.0, -2.0,
       0.0, 1.0, 1.0, 3.0, -1.0, 1.0,
       1.0, 0.0, 1.0, 2.0, 2.0, 3.0;
  std::vector<int> basis{0, 1, 2};
  std::vector<char> basic{1, 1, 1, 0, 0, 0};
  Eigen::Matrix3d B = A.leftCols<3>();
  std::vector<double> weight(6, 1.0);
  for (int col = 3; col < 6; ++col) {
    weight[static_cast<std::size_t>(col)] =
        1.0 + B.fullPivLu().solve(A.col(col)).squaredNorm();
  }

  const std::vector<int> entering_sequence{3, 4};
  const std::vector<int> leaving_rows{1, 0};
  for (std::size_t step = 0; step < entering_sequence.size(); ++step) {
    const int entering = entering_sequence[step];
    const int pivot_row = leaving_rows[step];
    const int leaving = basis[static_cast<std::size_t>(pivot_row)];
    const Eigen::Vector3d h = B.fullPivLu().solve(A.col(entering));
    REQUIRE(std::abs(h[pivot_row]) > 1e-12);
    const Eigen::Vector3d row_ep =
        B.transpose().fullPivLu().solve(Eigen::Vector3d::Unit(pivot_row));
    const Eigen::Vector3d z = B.transpose().fullPivLu().solve(h);
    const Eigen::RowVector<double, 6> rho = row_ep.transpose() * A;
    const Eigen::RowVector<double, 6> mu = z.transpose() * A;

    IndexedVector tableau_row;
    IndexedVector cross_products;
    tableau_row.dimension = 6;
    cross_products.dimension = 6;
    for (int col = 0; col < 6; ++col) {
      tableau_row.index.push_back(col);
      tableau_row.value.push_back(rho[col]);
      cross_products.index.push_back(col);
      cross_products.value.push_back(mu[col]);
    }
    std::vector<PrimalWeightChange> changes;
    REQUIRE(prepare_primal_pse_update(
        weight, entering, leaving, h[pivot_row], h.squaredNorm(), tableau_row,
        cross_products, changes));
    for (const PrimalWeightChange& change : changes) {
      weight[static_cast<std::size_t>(change.col)] = change.value;
    }

    B.col(pivot_row) = A.col(entering);
    basis[static_cast<std::size_t>(pivot_row)] = entering;
    basic[static_cast<std::size_t>(leaving)] = 0;
    basic[static_cast<std::size_t>(entering)] = 1;
    for (int col = 0; col < 6; ++col) {
      if (basic[static_cast<std::size_t>(col)]) continue;
      const double exact = 1.0 + B.fullPivLu().solve(A.col(col)).squaredNorm();
      CHECK(weight[static_cast<std::size_t>(col)] ==
            Approx(exact).epsilon(1e-11));
    }
  }
}

TEST_CASE("Primal Devex reference update matches direct projected edges",
          "[dual_simplex][primal][pricing][devex]") {
  using namespace mipsolvers::engine::native_dual::detail;

  // B is the first three columns, so the initial Devex reference set is
  // exactly columns 3 and 4. Exchange column 3 into basis position 1.
  Eigen::Matrix<double, 3, 5> A;
  A << 2.0, 1.0, 0.0, 1.0, 4.0,
       0.0, 1.0, 1.0, 3.0, -1.0,
       1.0, 0.0, 1.0, 2.0, 2.0;
  Eigen::Matrix3d B = A.leftCols<3>();
  std::vector<int> basis{0, 1, 2};
  std::vector<char> basic{1, 1, 1, 0, 0};
  PrimalDevexFramework framework;
  initialize_primal_devex(5, basic, framework);

  const int entering = 3;
  const int pivot_row = 1;
  const int leaving = basis[static_cast<std::size_t>(pivot_row)];
  const Eigen::Vector3d h = B.fullPivLu().solve(A.col(entering));
  IndexedVector direction;
  direction.dimension = 3;
  direction.index = {0, 1, 2};
  direction.value = {h[0], h[1], h[2]};
  const double pivot_weight = primal_devex_pivot_reference_weight(
      framework, basis, entering, direction);
  CHECK(pivot_weight == Approx(1.0));

  const Eigen::Vector3d row_ep =
      B.transpose().fullPivLu().solve(Eigen::Vector3d::Unit(pivot_row));
  const Eigen::RowVector<double, 5> rho = row_ep.transpose() * A;
  IndexedVector tableau_row;
  tableau_row.dimension = 5;
  tableau_row.index = {0, 1, 2, 3, 4};
  tableau_row.value = {rho[0], rho[1], rho[2], rho[3], rho[4]};
  std::vector<PrimalWeightChange> changes;
  REQUIRE(prepare_primal_devex_update(framework, entering, leaving,
                                      h[pivot_row], pivot_weight, tableau_row,
                                      changes));
  commit_primal_weight_changes(framework, changes);

  Eigen::Matrix3d next_B = B;
  next_B.col(pivot_row) = A.col(entering);
  const Eigen::Vector3d next_h4 = next_B.fullPivLu().solve(A.col(4));
  // Reference variables after the exchange are nonbasic column 4 and basic
  // entering column 3 at basis position 1.
  const double direct_col4_reference_norm =
      1.0 + next_h4[pivot_row] * next_h4[pivot_row];
  CHECK(framework.weight[4] ==
        Approx(direct_col4_reference_norm).epsilon(1e-12));

  const Eigen::Vector3d next_leaving = next_B.fullPivLu().solve(A.col(leaving));
  const double direct_leaving_reference_norm =
      next_leaving[pivot_row] * next_leaving[pivot_row];
  CHECK(framework.weight[static_cast<std::size_t>(leaving)] ==
        Approx(std::max(1.0, direct_leaving_reference_norm)).epsilon(1e-12));
  CHECK(framework.weight[static_cast<std::size_t>(entering)] == Approx(1.0));
}

TEST_CASE("Primal relative pivot is scale-relative",
          "[dual_simplex][primal][pricing][stability]") {
  using namespace mipsolvers::engine::native_dual::detail;
  IndexedVector direction;
  direction.dimension = 2;
  direction.index = {0, 1};
  direction.value = {1e12, 1.0};
  CHECK_FALSE(primal_relative_pivot_acceptable(1.0, direction));
  direction.value = {2.0, 1.0};
  CHECK(primal_relative_pivot_acceptable(1.0, direction));
  CHECK(primal_pivot_identity_acceptable(1.0, 1.0 + 1e-9));
  CHECK_FALSE(primal_pivot_identity_acceptable(1.0, 1.0 + 1e-6));
}

TEST_CASE("DualSimplex: reconstruction corrects canonical primal residual",
          "[dual_simplex][stabilization][refinement]") {
  using mipsolvers::engine::native_dual::detail::BasisFactor;
  using mipsolvers::engine::native_dual::detail::Move;
  using mipsolvers::engine::native_dual::detail::State;

  StandardFormLP sf;
  Eigen::Matrix<double, 2, 4> dense;
  dense << 1.0, 0.0, 1.0, 0.0,
           0.0, 1.0, 0.0, 1.0;
  sf.A = dense.sparseView();
  sf.A.makeCompressed();
  sf.b.resize(2);
  sf.b << 1.0, 2.0;
  sf.c_max = Eigen::VectorXd::Zero(4);
  sf.var_ub = Eigen::VectorXd::Constant(
      4, std::numeric_limits<double>::infinity());
  sf.n_original = 2;
  sf.row_to_artificial_col.assign(2, -1);
  sf.row_to_slack_col = {2, 3};

  auto options = native_simplex_options();
  options.feasibility_tol = 1e-8;
  State state;
  state.sf = &sf;
  state.options = &options;
  state.m = 2;
  state.n = 4;
  state.bounds.lower = Eigen::VectorXd::Zero(4);
  state.bounds.upper = Eigen::VectorXd::Constant(
      4, std::numeric_limits<double>::infinity());
  state.bounds.enterable.assign(4, 1);
  state.basis = {0, 1};
  state.move.assign(4, Move::Up);
  state.cost = sf.c_max;
  state.factor = std::make_shared<BasisFactor>(sf, std::vector<int>{2, 3});

  int repairs = 0;
  std::string failure;
  const bool rebuilt = state.factor->rebuild(state.basis, repairs, failure);
  INFO("factor failure=" << failure);
  REQUIRE(rebuilt);
  REQUIRE(mipsolvers::engine::native_dual::detail::rebuild_membership(
      state, failure));
  state.reduced_costs = Eigen::VectorXd::Zero(4);
  state.x_basic = sf.b;
  state.x_basic[0] += 1e-4;
  REQUIRE(
      mipsolvers::engine::native_dual::detail::correct_canonical_primal_residual(
          state, failure));
  INFO("failure=" << failure);
  CHECK(state.canonical_primal_corrections == 1);
  const Eigen::VectorXd x =
      mipsolvers::engine::native_dual::detail::full_primal(state);
  const double residual =
      mipsolvers::engine::native_dual::detail::equation_residual_inf(
          sf.A, x, sf.b);
  CHECK(residual <= options.feasibility_tol * sf.b.lpNorm<Eigen::Infinity>());
}

TEST_CASE("DualSimplex: repeated basis state taboos its prior outgoing edge",
          "[dual_simplex][stabilization][cycling][taboo]") {
  using mipsolvers::engine::native_dual::Statistics;
  using mipsolvers::engine::native_dual::detail::Move;
  using mipsolvers::engine::native_dual::detail::State;
  State state;
  state.m = 2;
  state.n = 4;
  state.basis = {0, 1};
  state.basic = {1, 1, 0, 0};
  state.move = {Move::Fixed, Move::Fixed, Move::Up, Move::Down};
  mipsolvers::engine::native_dual::detail::initialize_cycle_guard(state);

  mipsolvers::engine::native_dual::detail::record_cycle_departure(state, 0, 2);
  state.basis[0] = 2;
  state.basic = {0, 1, 1, 0};
  state.move = {Move::Up, Move::Fixed, Move::Fixed, Move::Down};
  // This test simulates a pivot by mutating basis/move directly, so it must
  // resync the incrementally maintained live signature the way the kernel's
  // pivot commit (or a major rebuild) would.
  mipsolvers::engine::native_dual::detail::resync_cycle_signature(state);
  Statistics statistics;
  CHECK_FALSE(mipsolvers::engine::native_dual::detail::record_cycle_arrival(
      state, statistics));

  mipsolvers::engine::native_dual::detail::record_cycle_departure(state, 2, 0);
  state.basis[0] = 0;
  state.basic = {1, 1, 0, 0};
  state.move = {Move::Fixed, Move::Fixed, Move::Up, Move::Down};
  mipsolvers::engine::native_dual::detail::resync_cycle_signature(state);
  REQUIRE(mipsolvers::engine::native_dual::detail::record_cycle_arrival(
      state, statistics));
  CHECK(statistics.cycles_detected == 1);
  CHECK(statistics.taboo_changes == 1);
  CHECK(mipsolvers::engine::native_dual::detail::is_taboo_change(state, 0, 2));
  CHECK_FALSE(
      mipsolvers::engine::native_dual::detail::is_taboo_change(state, 0, 3));
  mipsolvers::engine::native_dual::detail::add_taboo_row(state, 0,
                                                         statistics);
  CHECK(statistics.taboo_rows == 1);
  CHECK(mipsolvers::engine::native_dual::detail::is_taboo_row(state, 0));
  mipsolvers::engine::native_dual::detail::release_taboo_row(state, 0,
                                                             statistics);
  CHECK(statistics.taboo_row_releases == 1);
  CHECK_FALSE(mipsolvers::engine::native_dual::detail::is_taboo_row(state, 0));
}

TEST_CASE("DualSimplex: cycle history has a hard memory bound",
          "[dual_simplex][performance][cycling]") {
  using namespace mipsolvers::engine::native_dual;
  using namespace mipsolvers::engine::native_dual::detail;
  State state;
  state.m = 10000000;
  state.n = 1;
  state.basis = {0};
  state.basic = {1};
  state.move = {Move::Fixed};
  initialize_cycle_guard(state);
  Statistics statistics;
  bool repeated = false;
  for (std::size_t i = 0; i < kCycleHistoryCapacity + 32; ++i) {
    state.cycle_signature_live_a = static_cast<std::uint64_t>(i + 1);
    state.cycle_signature_live_b = ~static_cast<std::uint64_t>(i + 1);
    repeated = record_cycle_arrival(state, statistics) || repeated;
  }
  CHECK_FALSE(repeated);
  CHECK(state.cycle_history.size() <= kCycleHistoryCapacity);
  CHECK(state.cycle_history.storage_capacity() <=
        2 * kCycleHistoryCapacity);

  int expiry = -1;
  add_taboo_row(state, 0, statistics);
  REQUIRE(is_taboo_row(state, 0, &expiry));
  CHECK(expiry - state.pricing_epoch <= kMaxTabooLifetime);

  release_taboo_row(state, 0, statistics);
  for (std::size_t i = 0; i < kTabooCapacity + 32; ++i) {
    add_taboo_row(state, 0, statistics);
    release_taboo_row(state, 0, statistics);
  }
  CHECK(state.taboo_rows.empty());
  CHECK(state.taboo_rows_fifo.size() <= kTabooCapacity);
  CHECK(state.taboo_rows.storage_capacity() <= 2 * kTabooCapacity);

  state.m = std::numeric_limits<int>::max();
  add_taboo_row(state, 0, statistics);
  REQUIRE(is_taboo_row(state, 0, &expiry));
  CHECK(expiry - state.pricing_epoch == kMaxTabooLifetime);
}

TEST_CASE("DualSimplex: large LPModel warm-start stays sparse",
          "[dual_simplex][warm_start]") {
  constexpr int rows = 256;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Ones(rows);
  lp.A.resize(0, rows);
  lp.b.resize(0);
  lp.Aeq.resize(rows, rows);
  lp.beq = Eigen::VectorXd::Ones(rows);
  lp.vars.reserve(rows);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(rows);
  for (int i = 0; i < rows; ++i) {
    triplets.emplace_back(i, i, 1.0);
    lp.vars.push_back({VarType::Continuous, 0.0, 2.0});
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  auto opts = native_simplex_options();
  opts.max_iter = 1000;
  opts.escalation_max_level = 0;

  const auto first = solve_lp_with_basis(lp, opts);
  REQUIRE(first.result.stats.success);
  REQUIRE(first.basis.cached_sparse_basis);

  const auto second = solve_lp_with_basis(lp, opts, &first.basis);
  REQUIRE(second.result.stats.success);
  CHECK(second.solved_from_hint);
  CHECK(second.dual_reoptimized);
  CHECK(second.result.stats.iterations <= 1);
  CHECK(second.basis_inverse.size() == 0);
  CHECK(second.basis.cached_sparse_basis);
  CHECK(second.result.stats.objective == Approx(rows).margin(1e-8));
}

TEST_CASE("DualSimplex: sparse warm-start honors the explicit iteration limit",
          "[dual_simplex][warm_start]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows;

  LPModel root;
  root.sense = Sense::Minimize;
  root.c = Eigen::VectorXd::Zero(cols);
  root.A.resize(0, cols);
  root.b.resize(0);
  root.Aeq.resize(rows, cols);
  root.beq = Eigen::VectorXd::Ones(rows);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    triplets.emplace_back(row, 2 * row, 1.0);
    triplets.emplace_back(row, 2 * row + 1, 1.0);
    root.c[2 * row + 1] = 1.0;
    root.vars.push_back({VarType::Continuous, 0.0, 1.0});
    root.vars.push_back({VarType::Continuous, 0.0, 1.0});
  }
  root.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  root.Aeq.makeCompressed();

  auto opts = native_simplex_options();
  opts.max_iter = 160;
  opts.escalation_max_level = 0;

  const auto first = solve_lp_with_basis(root, opts);
  REQUIRE(first.result.stats.success);
  REQUIRE(first.basis.cached_dse_weights);
  REQUIRE(first.basis.cached_dse_basis);

  mipsolvers::engine::SimplexBasis portable_hint = first.basis;
  portable_hint.cached_sparse_basis.reset();

  LPModel child = root;
  for (int row = 0; row < rows; ++row) {
    child.vars[2 * row].ub = 0.4;
  }
  const auto second = solve_lp_with_basis(child, opts, &portable_hint);
  INFO("status=" << second.result.stats.status
                  << " iterations=" << second.result.stats.iterations);
  CHECK_FALSE(second.result.stats.success);
  CHECK(second.result.stats.status.starts_with(
      "Native dual simplex: dual simplex iteration limit"));
  CHECK(second.result.stats.iterations == opts.max_iter);
}

TEST_CASE("DualSimplex: warm reoptimization excludes outage-fixed columns",
          "[dual_simplex][warm_start][fixed_bounds][highs_differential]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Ones(rows);
  lp.vars.reserve(cols);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    triplets.emplace_back(row, row, 1.0);
    triplets.emplace_back(row, rows + row, 1.0);
    lp.c[rows + row] = 1.0;
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
  }
  for (int row = 0; row < rows; ++row) {
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  auto options = native_simplex_options();
  options.max_iter = 1000;
  options.escalation_max_level = 0;

  auto root = solve_lp_with_basis(lp, options);
  REQUIRE(root.result.stats.success);
  CHECK(root.result.stats.objective == Approx(0.0).margin(1e-9));

  for (int row = 0; row < rows; ++row) lp.vars[row].ub = 0.0;
  const double highs_objective = solve_with_highs(lp);
  auto outage = solve_lp_with_basis(lp, options, &root.basis);
  INFO("status=" << outage.result.stats.status
                  << " iterations=" << outage.result.stats.iterations);
  REQUIRE(outage.result.stats.success);
  CHECK(outage.solved_from_hint);
  CHECK(outage.dual_reoptimized);
  CHECK(outage.result.stats.objective == Approx(highs_objective).margin(1e-8));
  CHECK((lp.Aeq * outage.result.x - lp.beq).lpNorm<Eigen::Infinity>() <= 1e-9);
}

TEST_CASE("DualSimplex: warm reoptimization repairs a rank-deficient outage basis",
          "[dual_simplex][warm_start][rank_repair][highs_differential]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows;

  LPModel root;
  root.sense = Sense::Minimize;
  root.c = Eigen::VectorXd::Zero(cols);
  root.A.resize(0, cols);
  root.b.resize(0);
  root.Aeq.resize(rows, cols);
  root.beq = Eigen::VectorXd::Ones(rows);
  root.vars.reserve(cols);

  std::vector<Eigen::Triplet<double>> root_triplets;
  root_triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    root_triplets.emplace_back(row, row, 1.0);
    root_triplets.emplace_back(row, rows + row, 0.5);
    root.c[rows + row] = 1.0;
  }
  for (int col = 0; col < cols; ++col) {
    root.vars.push_back({VarType::Continuous, 0.0, 2.0});
  }
  root.Aeq.setFromTriplets(root_triplets.begin(), root_triplets.end());
  root.Aeq.makeCompressed();

  auto options = native_simplex_options();
  options.max_iter = 2000;
  options.escalation_max_level = 0;

  auto native = solve_lp_with_basis(root, options);
  REQUIRE(native.result.stats.success);
  CHECK(native.result.stats.objective == Approx(0.0).margin(1e-9));

  LPModel outage = root;
  std::vector<Eigen::Triplet<double>> outage_triplets;
  outage_triplets.reserve(cols + 2);
  outage_triplets.emplace_back(0, 0, 1.0);
  outage_triplets.emplace_back(1, 0, 1.0);
  outage_triplets.emplace_back(0, 1, 1.0);
  outage_triplets.emplace_back(1, 1, 1.0);
  for (int row = 2; row < rows; ++row) {
    outage_triplets.emplace_back(row, row, 1.0);
  }
  for (int row = 0; row < rows; ++row) {
    outage_triplets.emplace_back(row, rows + row, 0.5);
  }
  outage.Aeq.setZero();
  outage.Aeq.setFromTriplets(outage_triplets.begin(), outage_triplets.end());
  outage.Aeq.makeCompressed();

  const double highs_objective = solve_with_highs(outage);
  auto repaired = solve_lp_with_basis(outage, options, &native.basis);
  INFO("status=" << repaired.result.stats.status
                  << " iterations=" << repaired.result.stats.iterations);
  REQUIRE(repaired.result.stats.success);
  CHECK(repaired.solved_from_hint);
  CHECK(repaired.dual_reoptimized);
  CHECK(repaired.result.stats.objective == Approx(highs_objective).margin(1e-8));
  CHECK((outage.Aeq * repaired.result.x - outage.beq)
            .lpNorm<Eigen::Infinity>() <= 1e-9);
}

TEST_CASE("HFactor basis repair replaces only the column without a pivot",
          "[dual_simplex][rank_repair][hfactor]") {
  Eigen::SparseMatrix<double> A(2, 4);
  std::vector<Eigen::Triplet<double>> entries{
      {0, 0, 1.0}, {0, 1, 1.0}, {0, 2, 2.0}, {1, 3, 3.0}};
  A.setFromTriplets(entries.begin(), entries.end());
  A.makeCompressed();

  std::vector<int> basis{0, 1};
  const std::vector<int> logical{2, 3};
  std::vector<int> repaired;
  HFactorBackend factor;
  REQUIRE(factor.factorize_with_logicals(A, basis.data(), 2, logical,
                                         repaired));
  REQUIRE(factor.rank_deficiency == 1);
  REQUIRE(repaired.size() == basis.size());
  CHECK(std::inner_product(
            repaired.begin(), repaired.end(), basis.begin(), 0,
            std::plus<>(), std::not_equal_to<>()) == 1);

  const Eigen::Vector2d rhs(2.0, 6.0);
  Eigen::Vector2d solution;
  factor.ftran(rhs.data(), solution.data());
  CHECK((solution - Eigen::Vector2d(2.0, 2.0)).lpNorm<Eigen::Infinity>() ==
        Approx(0.0).margin(1e-12));
}

TEST_CASE("HFactor reuses immutable CSC storage across INVERTs",
          "[dual_simplex][performance][hfactor]") {
  Eigen::SparseMatrix<double> A(3, 6);
  std::vector<Eigen::Triplet<double>> entries{
      {0, 0, 2.0}, {2, 0, 1.0}, {0, 1, 1.0}, {1, 1, 3.0},
      {1, 2, 1.0}, {2, 2, 4.0}, {0, 3, 1.0}, {1, 4, 1.0},
      {2, 5, 1.0}};
  A.setFromTriplets(entries.begin(), entries.end());
  A.makeCompressed();
  std::vector<int> structural{0, 1, 2};
  std::vector<int> logical{3, 4, 5};

  HFactorBackend factor;
  REQUIRE(factor.factorize(A, structural.data(), 3));
  REQUIRE(factor.matrix_copy_count() == 0);
  REQUIRE(factor.factorize(A, logical.data(), 3));
  CHECK(factor.matrix_copy_count() == 0);

  Eigen::SparseMatrix<double> copied = A;
  copied.makeCompressed();
  REQUIRE(factor.factorize(copied, structural.data(), 3));
  CHECK(factor.matrix_copy_count() == 0);

  StandardColumnMatrix wide = A;
  HFactorBackend direct;
  REQUIRE(direct.factorize(wide, structural.data(), 3));
  REQUIRE(direct.factorize(wide, logical.data(), 3));
  CHECK(direct.matrix_copy_count() == 0);
}

TEST_CASE("Sparse basis inverse rows never use the dense solve API",
          "[dual_simplex][performance][basis_row]") {
  using mipsolvers::engine::native_dual::detail::BasisFactor;
  StandardFormLP sf;
  Eigen::Matrix<double, 3, 6> dense;
  dense << 2.0, 1.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 3.0, 1.0, 0.0, 1.0, 0.0,
           1.0, 0.0, 4.0, 0.0, 0.0, 1.0;
  sf.A = dense.sparseView();
  sf.A.makeCompressed();
  sf.A_row = sf.A;
  std::vector<int> basis{0, 1, 2};
  BasisFactor factor(sf, std::vector<int>{3, 4, 5});
  int repairs = 0;
  std::string failure;
  REQUIRE(factor.rebuild(basis, repairs, failure));
  const SparseFactorTelemetry before = factor.factor_telemetry();
  std::vector<std::pair<int, double>> row;
  REQUIRE(factor.basis_inverse_row_sparse_entries(1, row));
  REQUIRE_FALSE(row.empty());
  const SparseFactorTelemetry after = factor.factor_telemetry();
  CHECK(after.dense_solves == before.dense_solves);
  CHECK(after.indexed_solves == before.indexed_solves + 1);
  CHECK(after.matrix_copies == 0);
}

TEST_CASE("HFactor preserves caller basis positions and solve coordinates",
          "[dual_simplex][basis_order][hfactor]") {
  Eigen::SparseMatrix<double> A(3, 6);
  std::vector<Eigen::Triplet<double>> entries{
      {0, 0, 1.0}, {1, 1, 1.0}, {2, 2, 1.0},
      {0, 3, 2.0}, {1, 4, 3.0}, {2, 5, 4.0}};
  A.setFromTriplets(entries.begin(), entries.end());
  A.makeCompressed();

  const std::vector<int> basis{2, 0, 1};
  const std::vector<int> logical{3, 4, 5};
  std::vector<int> repaired;
  HFactorBackend factor;
  REQUIRE(factor.factorize_with_logicals(A, basis.data(), 3, logical,
                                         repaired));
  REQUIRE(factor.rank_deficiency == 0);
  CHECK(repaired == basis);

  const Eigen::Vector3d rhs(10.0, 20.0, 30.0);
  Eigen::Vector3d primal;
  factor.ftran(rhs.data(), primal.data());
  CHECK((primal - Eigen::Vector3d(30.0, 10.0, 20.0))
            .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(1e-12));

  const Eigen::Vector3d basic_cost(3.0, 5.0, 7.0);
  Eigen::Vector3d dual;
  factor.btran(basic_cost.data(), dual.data());
  CHECK((dual - Eigen::Vector3d(5.0, 7.0, 3.0))
            .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(1e-12));
}

TEST_CASE("HFactor indexed solves cover every nonzero result entry",
          "[dual_simplex][hfactor][hypersparse][factor_differential]") {
  Eigen::Matrix4d dense;
  dense << 4.0, 1.0, 0.0, 2.0,
           1.0, 5.0, 2.0, 0.0,
           0.0, 2.0, 6.0, 1.0,
           2.0, 0.0, 1.0, 7.0;
  Eigen::SparseMatrix<double> A = dense.sparseView();
  A.makeCompressed();
  const std::vector<int> basis{2, 0, 3, 1};
  Eigen::Matrix4d B;
  for (int position = 0; position < 4; ++position) {
    B.col(position) = dense.col(basis[static_cast<std::size_t>(position)]);
  }
  REQUIRE(B.fullPivLu().rank() == 4);

  HFactorBackend factor;
  REQUIRE(factor.factorize(A, basis.data(), 4));
  Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
  rhs[0] = 3.0;
  rhs[3] = -2.0;
  const std::vector<int> rhs_pattern{0, 3};
  Eigen::Vector4d ftran_result;
  Eigen::Vector4d btran_result;
  std::vector<int> ftran_pattern;
  std::vector<int> btran_pattern;
  factor.ftran(rhs.data(), ftran_result.data(), &rhs_pattern, &ftran_pattern);
  factor.btran(rhs.data(), btran_result.data(), &rhs_pattern, &btran_pattern);

  CHECK((ftran_result - B.fullPivLu().solve(rhs)).lpNorm<Eigen::Infinity>() ==
        Approx(0.0).margin(2e-11));
  CHECK((btran_result - B.transpose().fullPivLu().solve(rhs))
            .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
  for (int row = 0; row < 4; ++row) {
    if (ftran_result[row] != 0.0) {
      CHECK(std::count(ftran_pattern.begin(), ftran_pattern.end(), row) == 1);
    }
    if (btran_result[row] != 0.0) {
      CHECK(std::count(btran_pattern.begin(), btran_pattern.end(), row) == 1);
    }
  }
  for (int row : ftran_pattern) CHECK((row >= 0 && row < 4));
  for (int row : btran_pattern) CHECK((row >= 0 && row < 4));
}

TEST_CASE("HFactor fresh solves arbitrary non-diagonal bases",
          "[dual_simplex][basis_order][hfactor][factor_differential]") {
  Eigen::Matrix<double, 5, 10> dense;
  dense << 4.0, 1.0, 0.0, 2.0, 0.0, 1.0, 0.0, 2.0, -1.0, 3.0,
           1.0, 5.0, 2.0, 0.0, 0.0, 3.0, 1.0, 0.0,  2.0, 1.0,
           0.0, 2.0, 6.0, 1.0, 1.0, 0.0, 4.0, 1.0,  0.0, 2.0,
           2.0, 0.0, 1.0, 7.0, 2.0, 1.0, 0.0, 5.0,  1.0, 0.0,
           0.0, 0.0, 1.0, 2.0, 8.0, 2.0, 1.0, 0.0,  4.0, 3.0;
  Eigen::SparseMatrix<double> A = dense.sparseView();
  A.makeCompressed();
  const std::vector<std::vector<int>> bases{
      {0, 1, 2, 3, 4}, {3, 0, 4, 1, 2}, {5, 6, 7, 8, 9},
      {8, 2, 5, 0, 7}};
  const Eigen::Vector<double, 5> primal_rhs(0.5, -2.0, 3.5, 7.0, -1.25);
  const Eigen::Vector<double, 5> dual_rhs(-1.0, 4.0, 2.5, -3.0, 0.75);

  for (const std::vector<int>& basis : bases) {
    Eigen::Matrix<double, 5, 5> B;
    for (int position = 0; position < 5; ++position) {
      B.col(position) = dense.col(basis[static_cast<std::size_t>(position)]);
    }
    REQUIRE(B.fullPivLu().rank() == 5);
    HFactorBackend factor;
    REQUIRE(factor.factorize(A, basis.data(), 5));

    Eigen::Vector<double, 5> actual_primal;
    Eigen::Vector<double, 5> actual_dual;
    factor.ftran(primal_rhs.data(), actual_primal.data());
    factor.btran(dual_rhs.data(), actual_dual.data());
    CHECK((actual_primal - B.fullPivLu().solve(primal_rhs))
              .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
    CHECK((actual_dual - B.transpose().fullPivLu().solve(dual_rhs))
              .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
  }
}

TEST_CASE("HFactor product-form chain from a non-diagonal basis matches fresh LU",
          "[dual_simplex][factor_update][hfactor][factor_differential]") {
  Eigen::Matrix<double, 5, 10> dense;
  dense << 4.0, 1.0, 0.0, 2.0, 0.0, 1.0, 0.0, 2.0, -1.0, 3.0,
           1.0, 5.0, 2.0, 0.0, 0.0, 3.0, 1.0, 0.0,  2.0, 1.0,
           0.0, 2.0, 6.0, 1.0, 1.0, 0.0, 4.0, 1.0,  0.0, 2.0,
           2.0, 0.0, 1.0, 7.0, 2.0, 1.0, 0.0, 5.0,  1.0, 0.0,
           0.0, 0.0, 1.0, 2.0, 8.0, 2.0, 1.0, 0.0,  4.0, 3.0;
  Eigen::SparseMatrix<double> A = dense.sparseView();
  A.makeCompressed();
  std::vector<int> basis{3, 0, 4, 1, 2};
  HFactorBackend updated;
  REQUIRE(updated.factorize(A, basis.data(), 5));
  const std::vector<std::pair<int, int>> exchanges{
      {0, 5}, {1, 6}, {2, 7}, {3, 8}, {4, 9},
      {0, 3}, {1, 0}, {2, 4}, {3, 1}, {4, 2}};
  const Eigen::Vector<double, 5> primal_rhs(0.5, -2.0, 3.5, 7.0, -1.25);
  const Eigen::Vector<double, 5> dual_rhs(-1.0, 4.0, 2.5, -3.0, 0.75);
  HFactorBackend reused_fresh;

  auto matrix_for = [&](const std::vector<int>& index) {
    Eigen::Matrix<double, 5, 5> B;
    for (int position = 0; position < 5; ++position) {
      B.col(position) = dense.col(index[static_cast<std::size_t>(position)]);
    }
    return B;
  };
  for (const auto& [pivot_position, entering_col] : exchanges) {
    const auto before = matrix_for(basis);
    const Eigen::Vector<double, 5> aq =
        before.fullPivLu().solve(dense.col(entering_col));
    Eigen::Vector<double, 5> unit = Eigen::Vector<double, 5>::Zero();
    unit[pivot_position] = 1.0;
    const Eigen::Vector<double, 5> ep =
        before.transpose().fullPivLu().solve(unit);
    REQUIRE(std::abs(aq[pivot_position]) > 1e-12);
    Eigen::Vector<double, 5> captured_aq;
    Eigen::Vector<double, 5> captured_ep;
    updated.ftran_for_update(dense.col(entering_col).data(), captured_aq.data());
    updated.btran_for_update(unit.data(), captured_ep.data());
    REQUIRE(updated.update(pivot_position, entering_col, captured_aq.data(),
                           captured_ep.data()));
    basis[static_cast<std::size_t>(pivot_position)] = entering_col;

    const auto after = matrix_for(basis);
    REQUIRE(after.fullPivLu().rank() == 5);
    HFactorBackend fresh;
    REQUIRE(fresh.factorize(A, basis.data(), 5));
    REQUIRE(reused_fresh.factorize(A, basis.data(), 5));
    Eigen::Vector<double, 5> updated_primal, updated_dual;
    Eigen::Vector<double, 5> fresh_primal, fresh_dual;
    Eigen::Vector<double, 5> reused_primal, reused_dual;
    updated.ftran(primal_rhs.data(), updated_primal.data());
    updated.btran(dual_rhs.data(), updated_dual.data());
    fresh.ftran(primal_rhs.data(), fresh_primal.data());
    fresh.btran(dual_rhs.data(), fresh_dual.data());
    reused_fresh.ftran(primal_rhs.data(), reused_primal.data());
    reused_fresh.btran(dual_rhs.data(), reused_dual.data());
    CHECK((updated_primal - fresh_primal).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((updated_dual - fresh_dual).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((reused_primal - fresh_primal).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((reused_dual - fresh_dual).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((updated_primal - after.fullPivLu().solve(primal_rhs))
              .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
    CHECK((updated_dual - after.transpose().fullPivLu().solve(dual_rhs))
              .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
  }
}

TEST_CASE("HFactor FT update chain is algebraically identical to fresh bases",
          "[dual_simplex][factor_update][hfactor]") {
  Eigen::Matrix<double, 4, 8> dense;
  dense << 1, 0, 0, 0, 2, 1, 1, 1,
           0, 1, 0, 0, 1, 3, 1, 1,
           0, 0, 1, 0, 1, 1, 4, 1,
           0, 0, 0, 1, 1, 1, 1, 5;
  Eigen::SparseMatrix<double> A = dense.sparseView();
  A.makeCompressed();

  std::vector<int> basis{0, 1, 2, 3};
  HFactorBackend updated;
  REQUIRE(updated.factorize(A, basis.data(), 4));

  const std::vector<std::pair<int, int>> exchanges{
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
      {0, 0}, {1, 1}, {2, 2}, {3, 3}};
  const Eigen::Vector4d primal_rhs(0.5, -2.0, 3.5, 7.0);
  const Eigen::Vector4d dual_rhs(-1.0, 4.0, 2.5, -3.0);

  auto fresh_basis = [&]() {
    Eigen::Matrix4d B;
    for (int position = 0; position < 4; ++position) {
      B.col(position) = dense.col(basis[static_cast<std::size_t>(position)]);
    }
    return B;
  };

  for (const auto& [pivot_position, entering_col] : exchanges) {
    const Eigen::Matrix4d before = fresh_basis();
    const Eigen::Vector4d entering = dense.col(entering_col);
    const Eigen::Vector4d aq = before.fullPivLu().solve(entering);
    Eigen::Vector4d unit = Eigen::Vector4d::Zero();
    unit[pivot_position] = 1.0;
    const Eigen::Vector4d ep = before.transpose().fullPivLu().solve(unit);
    REQUIRE(std::abs(aq[pivot_position]) > 1e-12);
    Eigen::Vector4d captured_aq;
    Eigen::Vector4d captured_ep;
    updated.ftran_for_update(entering.data(), captured_aq.data());
    updated.btran_for_update(unit.data(), captured_ep.data());
    REQUIRE(updated.update(pivot_position, entering_col, captured_aq.data(),
                           captured_ep.data()));
    basis[static_cast<std::size_t>(pivot_position)] = entering_col;

    const Eigen::Matrix4d after = fresh_basis();
    REQUIRE(after.fullPivLu().rank() == 4);
    Eigen::Vector4d actual_primal;
    Eigen::Vector4d actual_dual;
    updated.ftran(primal_rhs.data(), actual_primal.data());
    updated.btran(dual_rhs.data(), actual_dual.data());
    const Eigen::Vector4d expected_primal =
        after.fullPivLu().solve(primal_rhs);
    const Eigen::Vector4d expected_dual =
        after.transpose().fullPivLu().solve(dual_rhs);
    CHECK((actual_primal - expected_primal).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((actual_dual - expected_dual).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
  }
}

TEST_CASE("DualSimplex: BFRT warm sequence agrees with HiGHS",
          "[dual_simplex][warm_start][bfrt][highs_differential]") {
  constexpr int side = 12;
  LPModel lp = make_degenerate_transport_lp(side);
  auto options = native_simplex_options();
  options.max_iter = 2000;
  options.escalation_max_level = 0;
  options.exact_dse_initialization = true;

  auto native = solve_lp_with_basis(lp, options);
  REQUIRE(native.result.stats.success);
  CHECK(native.result.stats.objective == Approx(solve_with_highs(lp)).margin(1e-8));

  auto check_dse_weights = [](const SimplexResult& result) {
    REQUIRE(result.basis.cached_sparse_basis);
    REQUIRE(result.basis.cached_dse_weights);
    REQUIRE(result.basis.cached_dse_weights->size() ==
            static_cast<std::size_t>(result.form.A.rows()));
    for (int row = 0; row < result.form.A.rows(); ++row) {
      Eigen::VectorXd unit = Eigen::VectorXd::Zero(result.form.A.rows());
      unit[row] = 1.0;
      const Eigen::VectorXd inverse_row =
          result.basis.cached_sparse_basis->btran(unit);
      REQUIRE(inverse_row.size() == result.form.A.rows());
      CHECK((*result.basis.cached_dse_weights)[static_cast<std::size_t>(row)] ==
            Approx(inverse_row.squaredNorm()).epsilon(2e-10).margin(2e-12));
    }
  };
  check_dse_weights(native);

  for (int step = 1; step <= 3; ++step) {
    int changed = 0;
    for (int col : native.basis.basis_indices()) {
      if (col < 0 || col >= side * side || native.result.x[col] <= 0.15) {
        continue;
      }
      auto& var = lp.vars[static_cast<size_t>(col)];
      const double next_ub = native.result.x[col] - 0.1;
      if (next_ub < var.ub - 1e-10) {
        var.ub = next_ub;
        ++changed;
        if (changed == 2) break;
      }
    }
    REQUIRE(changed > 0);
    const double highs_objective = solve_with_highs(lp);
    auto warm = solve_lp_with_basis(lp, options, &native.basis);
    INFO("step=" << step << " status=" << warm.result.stats.status
                 << " iterations=" << warm.result.stats.iterations);
    REQUIRE(warm.result.stats.success);
    CHECK(warm.solved_from_hint);
    CHECK(warm.dual_reoptimized);
    CHECK(warm.result.stats.iterations > 0);
    CHECK(warm.result.stats.objective == Approx(highs_objective).margin(1e-7));
    INFO("scaled_residual="
         << (warm.form.A * warm.x_std - warm.form.b)
                .lpNorm<Eigen::Infinity>());
    CHECK((warm.form.A * warm.x_std - warm.form.b)
              .lpNorm<Eigen::Infinity>() <= 1e-8);
    CHECK((lp.Aeq * warm.result.x - lp.beq).lpNorm<Eigen::Infinity>() <= 1e-8);
    native = std::move(warm);
    check_dse_weights(native);
  }
}

TEST_CASE("DualSimplex: cold Phase I honors the explicit pivot budget",
          "[dual_simplex][phase1]") {
  const EnvVarGuard shift_start("MIPSOLVERS_DUAL_SHIFT_START", "off");
  constexpr int rows = 256;
  constexpr int cols = 2 * rows + 1;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Ones(rows);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(2 * rows);
  for (int row = 0; row < rows; ++row) {
    // The crash selects 2*x first, but x=0.5 violates x<=0.4.  Phase I
    // therefore needs a bound flip followed by y entering for every row.
    triplets.emplace_back(row, 2 * row, 2.0);
    triplets.emplace_back(row, 2 * row + 1, 1.0);
    lp.vars.push_back({VarType::Continuous, 0.0, 0.4});
    lp.vars.push_back({VarType::Continuous, 0.0, 0.2});
  }
  // A lower-only improving zero column makes a dual-feasible endpoint
  // assignment impossible for the cold basis, so the exact start
  // classification must select primal Phase I without affecting its pivots.
  lp.c[cols - 1] = -1.0;
  lp.vars.push_back({VarType::Continuous, 0.0,
                     std::numeric_limits<double>::infinity()});
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  auto opts = native_simplex_options();
  opts.max_iter = 300;
  opts.escalation_max_level = 0;

  const auto solved = solve_lp_with_basis(lp, opts);
  INFO("status=" << solved.result.stats.status
                  << " iterations=" << solved.result.stats.iterations);
  CHECK_FALSE(solved.result.stats.success);
  CHECK(solved.result.stats.status ==
        "Native dual simplex: primal Phase I: primal simplex iteration limit");
  CHECK(solved.result.stats.iterations == opts.max_iter);
}

TEST_CASE("DualSimplex: crash basis skips fixed-width structural columns",
          "[dual_simplex][phase1][crash]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Ones(rows);
  lp.vars.reserve(cols);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    // The first, identity-like column is an attractive crash candidate but is
    // fixed at zero. The second column is the only one that can satisfy the
    // row and must be selected instead.
    triplets.emplace_back(row, row, -1.0);
    triplets.emplace_back(row, rows + row, 1.0);
    lp.vars.push_back({VarType::Continuous, 0.0, 0.0});
  }
  for (int row = 0; row < rows; ++row) {
    lp.vars.push_back({VarType::Continuous, 0.0, 2.0});
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  auto opts = native_simplex_options();
  opts.max_iter = 1000;
  opts.escalation_max_level = 0;

  const auto solved = solve_lp_with_basis(lp, opts);
  INFO("status=" << solved.result.stats.status
                  << " iterations=" << solved.result.stats.iterations);
  REQUIRE(solved.result.stats.success);
  CHECK(solved.result.stats.iterations <= 1);
  for (int row = 0; row < rows; ++row) {
    CHECK(solved.result.x[row] == Approx(0.0).margin(1e-12));
    CHECK(solved.result.x[rows + row] == Approx(1.0).margin(1e-9));
  }
}
