/// @file test_scaling_regression.cpp
/// Phase 1 regression tests: verify that enabling residual/variable scaling
/// does not break any existing power flow test case, and that the scaled
/// residual norm is populated correctly.
///
/// These tests are tagged TIER=integration and run against MATPOWER-derived
/// cases loaded via the existing test infrastructure.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/power_flow/hybrid.hpp"
#include "hacdcpf/io/matpower_parser.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

/// Load a small test case from the matpower data directory.
hacdcpf::HybridPowerSystem load_case(const std::string& name) {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
  return hacdcpf::io::parse_matpower(path);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Basic regression: default options (scaling enabled) must converge.
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Phase1 scaling: case9 converges with default options", "[scaling][regression]") {
  hacdcpf::PowerFlowOptions opt;
  // Scaling is enabled by default in RobustNonlinearOptions.
  REQUIRE(opt.robust_nonlinear.enable_residual_scaling);
  REQUIRE(opt.robust_nonlinear.enable_variable_scaling);

  auto sys = load_case("case9.m");
  const auto result = hacdcpf::powerflow::solve_hybrid(sys, opt);

  REQUIRE(result.converged);
  REQUIRE(result.profiling.scaled_residual_norm >= 0.0);
  REQUIRE(result.profiling.raw_residual_norm >= 0.0);
  // Scaled residual should be populated (≥ 0) and finite.
  REQUIRE(std::isfinite(result.profiling.scaled_residual_norm));
}

TEST_CASE("Phase1 scaling: case14 converges with scaling enabled", "[scaling][regression]") {
  hacdcpf::PowerFlowOptions opt;
  auto sys = load_case("case14.m");
  const auto result = hacdcpf::powerflow::solve_hybrid(sys, opt);
  REQUIRE(result.converged);
  REQUIRE(std::isfinite(result.profiling.scaled_residual_norm));
  REQUIRE(std::isfinite(result.profiling.raw_residual_norm));
}

TEST_CASE("Phase1 scaling: case30 converges with scaling enabled", "[scaling][regression]") {
  hacdcpf::PowerFlowOptions opt;
  auto sys = load_case("case30.m");
  const auto result = hacdcpf::powerflow::solve_hybrid(sys, opt);
  REQUIRE(result.converged);
}

TEST_CASE("Phase1 scaling: case57 converges with scaling enabled", "[scaling][regression]") {
  hacdcpf::PowerFlowOptions opt;
  auto sys = load_case("case57.m");
  const auto result = hacdcpf::powerflow::solve_hybrid(sys, opt);
  REQUIRE(result.converged);
}

TEST_CASE("Phase1 scaling: case118 converges with scaling enabled", "[scaling][regression]") {
  hacdcpf::PowerFlowOptions opt;
  auto sys = load_case("case118.m");
  const auto result = hacdcpf::powerflow::solve_hybrid(sys, opt);
  REQUIRE(result.converged);
}

// ─────────────────────────────────────────────────────────────────────────────
// Condition number estimate is populated when enabled.
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Phase1 condition monitor: estimate is finite", "[scaling][condition]") {
  hacdcpf::PowerFlowOptions opt;
  opt.robust_nonlinear.enable_condition_monitor = true;
  auto sys = load_case("case14.m");
  const auto result = hacdcpf::powerflow::solve_hybrid(sys, opt);
  REQUIRE(result.converged);
  REQUIRE(result.profiling.condition_estimate > 0.0);
  REQUIRE(std::isfinite(result.profiling.condition_estimate));
}

// ─────────────────────────────────────────────────────────────────────────────
// Disabling scaling should still converge (fallback to original behaviour).
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Phase1 scaling: case14 converges with scaling disabled", "[scaling][regression]") {
  hacdcpf::PowerFlowOptions opt;
  opt.robust_nonlinear.enable_residual_scaling = false;
  opt.robust_nonlinear.enable_variable_scaling = false;
  opt.robust_nonlinear.enable_jacobian_row_col_equilibration = false;
  auto sys = load_case("case14.m");
  const auto result = hacdcpf::powerflow::solve_hybrid(sys, opt);
  REQUIRE(result.converged);
}
