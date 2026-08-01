/// Regression tests for AML validation, linear export, and unsupported model
/// class rejection. Exported files are parsed and solved again by HiGHS so
/// these tests exercise semantics rather than checking strings only.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

#include "Highs.h"
#include "mipsolvers/aml/aml.hpp"

using Catch::Approx;
using namespace mipsolvers::aml;
namespace fs = std::filesystem;

namespace {

struct ExportPaths {
  fs::path lp;
  fs::path mps;

  ExportPaths() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const fs::path base = fs::temp_directory_path() /
                          ("mipsolvers_aml_export_" + std::to_string(stamp));
    lp = base;
    lp += ".lp";
    mps = base;
    mps += ".mps";
  }

  ~ExportPaths() {
    std::error_code error;
    fs::remove(lp, error);
    fs::remove(mps, error);
  }
};

struct SolvedExport {
  HighsLp lp;
  double objective{0.0};
};

SolvedExport read_and_solve(const fs::path& path) {
  Highs highs;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("log_to_console", false);
  REQUIRE(highs.readModel(path.string()) == HighsStatus::kOk);
  REQUIRE(highs.run() == HighsStatus::kOk);
  REQUIRE(highs.getModelStatus() == HighsModelStatus::kOptimal);
  return {highs.getLp(), highs.getInfo().objective_function_value};
}

}  // namespace

TEST_CASE("AML parameter validation rejects empty and non-finite storage",
          "[aml][validation]") {
  Model model("parameter_validation");
  auto& scalar = model.add_param_scalar("alpha");
  REQUIRE_THROWS_AS(model.check_missing_params(), std::domain_error);

  scalar.set_scalar(2.0);
  auto& table = model.add_param("demand", 1);
  REQUIRE_THROWS_AS(model.check_missing_params(), std::domain_error);

  table.set(Key::scalar("bus1"), std::numeric_limits<double>::quiet_NaN());
  REQUIRE_THROWS_AS(model.check_missing_params(), std::domain_error);
  table.set(Key::scalar("bus1"), 5.0);
  REQUIRE_NOTHROW(model.check_missing_params());
}

TEST_CASE("AML LP and MPS exports preserve MILP semantics and objective offset",
          "[aml][export]") {
  Model model("linear_export");
  auto& continuous_set = model.add_set("C", {"x"});
  auto& binary_set = model.add_set("B", {"y"});
  auto& integer_set = model.add_set("I", {"z"});
  auto& x = model.add_var("x", continuous_set, VarType::Continuous, 0.0, 10.0);
  auto& y = model.add_var("y", binary_set, VarType::Binary);
  auto& z = model.add_var("z", integer_set, VarType::Integer, -2.0, 3.0);

  model.add_constraint(x("x") + 2.0 * y("y") + z("z") <= 10.0, "capacity");
  model.add_constraint(LinearExpr::from_var(x("x").id()) - y("y") == 1.0,
                       "balance");
  LinearExpr objective = 2.0 * x("x") + 3.0 * y("y") - z("z");
  objective += LinearExpr::const_expr(7.0);
  model.minimize(objective);

  ExportPaths paths;
  model.write_lp(paths.lp.string());
  model.write_mps(paths.mps.string());

  const SolvedExport lp = read_and_solve(paths.lp);
  const SolvedExport mps = read_and_solve(paths.mps);
  CHECK(lp.lp.num_col_ == 3);
  CHECK(mps.lp.num_col_ == 3);
  REQUIRE(lp.lp.integrality_.size() == 3);
  REQUIRE(mps.lp.integrality_.size() == 3);
  CHECK(lp.lp.integrality_[1] == HighsVarType::kInteger);
  CHECK(lp.lp.integrality_[2] == HighsVarType::kInteger);
  CHECK(mps.lp.integrality_[1] == HighsVarType::kInteger);
  CHECK(mps.lp.integrality_[2] == HighsVarType::kInteger);
  CHECK(lp.lp.offset_ == Approx(7.0));
  CHECK(mps.lp.offset_ == Approx(7.0));
  CHECK(lp.objective == Approx(6.0));
  CHECK(mps.objective == Approx(6.0));

  const SolveResult direct = model.solve();
  REQUIRE(direct.is_optimal());
  CHECK(direct.objective_value == Approx(6.0));
}

TEST_CASE("AML rejects MIQP and MINLP instead of relaxing model structure",
          "[aml][validation]") {
  SECTION("MIQP") {
    Model model("miqp_rejected");
    auto& set = model.add_set("B", {"b"});
    auto& binary = model.add_var("b", set, VarType::Binary);
    model.minimize(QuadExpr::sq(binary("b").id(), 1.0));
    REQUIRE_THROWS_AS(model.solve(), std::invalid_argument);
    ExportPaths paths;
    REQUIRE_THROWS_AS(model.write_lp(paths.lp.string()), std::invalid_argument);
    REQUIRE_THROWS_AS(model.write_mps(paths.mps.string()), std::invalid_argument);
  }

  SECTION("MINLP") {
    Model model("minlp_rejected");
    auto& set = model.add_set("B", {"b"});
    auto& binary = model.add_var("b", set, VarType::Binary);
    model.minimize(model.nl_sq(model.nl_var(binary("b"))));
    REQUIRE_THROWS_AS(model.solve(), std::invalid_argument);
    ExportPaths paths;
    REQUIRE_THROWS_AS(model.write_lp(paths.lp.string()), std::invalid_argument);
    REQUIRE_THROWS_AS(model.write_mps(paths.mps.string()), std::invalid_argument);
  }
}
