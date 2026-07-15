#include <cmath>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/optimal_power_flow/reactive_power_opt.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../data"
#endif

TEST_CASE("RPO reports an honest local certificate and a physical loss ledger",
          "[rpo][opf][cross-validation]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");

  hacdcpf::opf::RPOOptions options;
  options.objective = hacdcpf::opf::RPOObjective::MinActiveLoss;
  options.max_nodes = 20;
  options.time_limit_sec = 20.0;
  options.max_ipm_iter = 400;

  const auto result = hacdcpf::opf::solve_rpo(system, options);

  INFO("RPO status=" << result.status);
  REQUIRE(result.converged);
  CHECK_FALSE(result.globally_certified);
  CHECK_FALSE(result.optimality_gap_available);
  CHECK(result.algorithm == "discrete_coordinate_search_with_ac_opf");
  CHECK(result.baseline_opf.converged);
  CHECK(result.optimized_opf.converged);
  CHECK(result.status.find("no adjustable discrete devices") != std::string::npos);
  CHECK(std::isfinite(result.total_loss_before_mw));
  CHECK(result.total_loss_before_mw > 0.0);
  CHECK(result.total_loss_before_mw < 50.0);
  CHECK(result.objective == result.total_loss_before_mw);
}

TEST_CASE("RPO exposes switchable shunt actions without a false global gap",
          "[rpo][shunt][discrete]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  hacdcpf::Shunt shunt;
  shunt.index = 901;
  shunt.name = "Bus 5 capacitor bank";
  shunt.bus = 5;
  shunt.in_service = true;
  shunt.switchable = true;
  shunt.n_steps = 2;
  shunt.current_step = 0;
  shunt.bs_per_step = 5.0;
  shunt.bs_mvar = 0.0;
  system.ac.shunts.push_back(shunt);

  hacdcpf::opf::RPOOptions options;
  options.objective = hacdcpf::opf::RPOObjective::MinVoltageDeviation;
  options.max_nodes = 20;
  options.time_limit_sec = 30.0;

  const auto result = hacdcpf::opf::solve_rpo(system, options);

  INFO("RPO status=" << result.status);
  REQUIRE(result.converged);
  REQUIRE(result.shunts.size() == 1);
  CHECK(result.shunts.front().shunt_index == 0);
  CHECK(result.shunts.front().step_after >= 0);
  CHECK(result.shunts.front().step_after <= 2);
  double baseline_voltage_objective = 0.0;
  for (const double vm : result.vm_before)
    baseline_voltage_objective += (vm - options.v_target) *
                                  (vm - options.v_target);
  CHECK(result.objective <= baseline_voltage_objective + 1e-10);
  CHECK_FALSE(result.qg_mvar_after.empty());
  CHECK_FALSE(result.globally_certified);
  CHECK_FALSE(result.optimality_gap_available);
  CHECK(result.gap == 1.0);
}

TEST_CASE("case300 RPO returns complete display vectors from a seeded solve",
          "[.performance][rpo][case300][warm-start][gui]") {
  const auto system = hacdcpf::io::build_case300_acdc();
  hacdcpf::opf::RPOOptions options;
  options.objective = hacdcpf::opf::RPOObjective::MinVoltageDeviation;
  options.max_nodes = 3;
  options.time_limit_sec = 30.0;
  options.max_ipm_iter = 400;
  options.ipm_tol = 1e-6;
  options.stationarity_tol = 1e-3;
  const auto result = hacdcpf::opf::solve_rpo(system, options);

  INFO("status=" << result.status
       << " nlp_solves=" << result.nlp_solves
       << " runtime=" << result.runtime_sec);
  REQUIRE(result.converged);
  CHECK(result.nlp_solves == 3);
  CHECK(result.vm_before.size() == system.ac.buses.size());
  CHECK(result.vm_after.size() == system.ac.buses.size());
  CHECK(result.qg_mvar_after.size() == system.ac.generators.size());
  CHECK(result.taps.size() == system.ac.transformers_2w.size());
}
