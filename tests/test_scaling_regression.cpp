/// @file test_scaling_regression.cpp
/// Phase 1 regression tests: verify that enabling residual/variable scaling
/// does not break any existing power flow test case, and that the scaled
/// residual norm is populated correctly.
///
/// These tests are tagged TIER=integration and run against MATPOWER-derived
/// cases loaded via the existing test infrastructure.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>  // std::isfinite (GCC 15 no longer provides it transitively)

#include <Eigen/LU>
#include <Eigen/Sparse>

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/hybrid.hpp"
#include "hacdcpf/power_flow/nonlinear_scaling.hpp"
#include "hacdcpf/io/matpower_parser.hpp"

#ifndef HACDCPF_MATPOWER_DATA_DIR
#define HACDCPF_MATPOWER_DATA_DIR "../../external_data/matpower"
#endif

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

/// Load a small test case from the matpower data directory.
hacdcpf::HybridPowerSystem load_case(const std::string& name) {
  const std::string path = std::string(HACDCPF_MATPOWER_DATA_DIR) + "/" + name;
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

TEST_CASE("Jacobian equilibration preserves the physical Newton step",
          "[scaling][equilibration][unit]") {
  Eigen::Matrix3d dense;
  dense << 1.0e8, 2.0, 0.0,
           3.0, 4.0e-6, 5.0,
           0.0, 6.0, 7.0e3;
  const Eigen::SparseMatrix<double> jacobian = dense.sparseView();
  const Eigen::Vector3d rhs(2.0, -1.0, 3.0);

  const hacdcpf::powerflow::NonlinearScaling nominal(
      Eigen::Vector3d(0.5, 2.0, 0.25),
      Eigen::Vector3d(3.0, 0.2, 5.0));
  hacdcpf::powerflow::RobustNonlinearOptions options;
  const auto scaling = hacdcpf::powerflow::equilibrate_nonlinear_scaling(
      nominal, jacobian, options);

  const Eigen::Vector3d direct = dense.fullPivLu().solve(rhs);
  const Eigen::Matrix3d balanced =
      Eigen::Matrix3d(scaling.apply_jacobian_scaling(jacobian));
  const Eigen::Vector3d balanced_rhs = scaling.apply_residual_scaling(rhs);
  const Eigen::Vector3d recovered =
      scaling.unscale_step(balanced.fullPivLu().solve(balanced_rhs));

  REQUIRE((dense * recovered - rhs).lpNorm<Eigen::Infinity>() < 1e-9);
  REQUIRE((recovered - direct).lpNorm<Eigen::Infinity>() < 1e-9);
}

TEST_CASE("Low-voltage trial guard is explicit and can be disabled",
          "[scaling][line-search][options]") {
  hacdcpf::PowerFlowOptions options;
  REQUIRE_THAT(options.robust_nonlinear.min_trial_voltage_pu,
               WithinAbs(0.5, 1e-12));
  options.robust_nonlinear.min_trial_voltage_pu = 0.0;
  REQUIRE(options.robust_nonlinear.min_trial_voltage_pu == 0.0);
}
