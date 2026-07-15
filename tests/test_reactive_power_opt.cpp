#include <cmath>
#include <string>

#include <catch2/catch_approx.hpp>
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
  options.max_tap_move = 2;
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

TEST_CASE("RPO control inventory explains adjustable and excluded inputs",
          "[rpo][controls][gui]") {
  hacdcpf::HybridPowerSystem system;
  auto make_transformer = [](int index, std::string name) {
    hacdcpf::Transformer2W transformer;
    transformer.index = index;
    transformer.name = std::move(name);
    transformer.hv_bus = 1;
    transformer.lv_bus = 2;
    transformer.tap_pos = 0;
    transformer.tap_min = -2;
    transformer.tap_max = 2;
    transformer.tap_neutral = 0;
    transformer.tap_step_percent = 1.25;
    return transformer;
  };
  system.ac.transformers_2w.push_back(make_transformer(10, "eligible"));
  auto fixed = make_transformer(11, "fixed");
  fixed.tap_min = fixed.tap_max = 0;
  system.ac.transformers_2w.push_back(fixed);
  auto invalid = make_transformer(12, "invalid-current");
  invalid.tap_pos = 3;
  system.ac.transformers_2w.push_back(invalid);
  auto offline = make_transformer(13, "offline");
  offline.in_service = false;
  system.ac.transformers_2w.push_back(offline);

  hacdcpf::Shunt switchable;
  switchable.index = 20;
  switchable.name = "capacitor-bank";
  switchable.bus = 2;
  switchable.switchable = true;
  switchable.current_step = 1;
  switchable.n_steps = 4;
  switchable.bs_per_step = 0.5;
  switchable.bs_mvar = 0.5;
  system.ac.shunts.push_back(switchable);
  auto fixed_shunt = switchable;
  fixed_shunt.index = 21;
  fixed_shunt.name = "fixed-shunt";
  fixed_shunt.switchable = false;
  system.ac.shunts.push_back(fixed_shunt);

  const auto inventory = hacdcpf::opf::inspect_rpo_controls(system);
  REQUIRE(inventory.taps.size() == 4);
  CHECK(inventory.taps[0].adjustable);
  CHECK(inventory.taps[0].selected_for_optimization);
  CHECK(inventory.taps[0].position_count == 5);
  CHECK(inventory.taps[0].optimization_position_count == 5);
  CHECK(inventory.taps[0].ratio_min == Catch::Approx(0.975));
  CHECK(inventory.taps[0].ratio_max == Catch::Approx(1.025));
  CHECK(inventory.taps[1].exclusion_reason == "fixed_or_invalid_tap_range");
  CHECK(inventory.taps[2].exclusion_reason == "current_tap_out_of_range");
  CHECK(inventory.taps[3].exclusion_reason == "not_in_service");
  REQUIRE(inventory.shunts.size() == 2);
  CHECK(inventory.shunts[0].adjustable);
  CHECK(inventory.shunts[0].position_count == 5);
  CHECK(inventory.shunts[1].exclusion_reason == "not_switchable");

  hacdcpf::opf::RPOOptions restricted;
  restricted.max_tap_move = 1;
  restricted.restrict_tap_indices = true;
  restricted.enabled_tap_indices = {0};
  const auto restricted_inventory =
      hacdcpf::opf::inspect_rpo_controls(system, restricted);
  CHECK(restricted_inventory.taps[0].selected_for_optimization);
  CHECK(restricted_inventory.taps[0].optimization_tap_min == -1);
  CHECK(restricted_inventory.taps[0].optimization_tap_max == 1);
  CHECK(restricted_inventory.taps[0].optimization_position_count == 3);
  CHECK_FALSE(restricted_inventory.taps[1].selected_for_optimization);
}
