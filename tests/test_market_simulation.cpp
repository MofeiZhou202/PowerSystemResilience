#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/market/market_simulation.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../data"
#endif

namespace {

hacdcpf::TimeSeriesData case9_day_profile() {
  hacdcpf::TimeSeriesData time_series;
  time_series.num_steps = 24;
  time_series.step_duration_hr = 1.0;
  hacdcpf::TimeSeriesProfile load;
  load.id = 0;
  load.name = "case9_day_load";
  load.values = {
      0.72, 0.68, 0.65, 0.64, 0.66, 0.72,
      0.80, 0.88, 0.94, 0.98, 1.00, 0.99,
      0.97, 0.96, 0.98, 1.02, 1.06, 1.10,
      1.08, 1.03, 0.96, 0.88, 0.81, 0.76};
  time_series.profiles.push_back(std::move(load));
  return time_series;
}

hacdcpf::market::MarketOptions market_options() {
  hacdcpf::market::MarketOptions options;
  options.energy_offer_segments = 12;
  options.upward_reserve_fraction = 0.05;
  options.value_of_lost_load_per_mwh = 10000.0;
  options.enable_network_constraints = true;
  options.run_ac_validation = true;
  options.uc_options.uc_solver = hacdcpf::UCSolverChoice::Native;
  options.ac_validation_options.max_iter = 100;
  options.ac_validation_options.tol = 1e-8;
  return options;
}

double sum(const std::vector<double>& values) {
  return std::accumulate(values.begin(), values.end(), 0.0);
}

}  // namespace

TEST_CASE("Case9 24-hour native market closes SCUC-SCED-LMP-ACPF-settlement",
          "[market][case9][integration]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  const auto original = system;
  const auto time_series = case9_day_profile();

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, market_options());

  INFO("market status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.feasible);
  REQUIRE(result.status == "converged");
  REQUIRE(result.commitment.feasible);
  REQUIRE(result.pricing.size() == 24);
  REQUIRE(result.num_pricing_converged == 24);
  REQUIRE(result.ac_validation.size() == 24);
  REQUIRE(result.num_ac_converged == 24);
  REQUIRE(result.num_ac_secure == 24);
  REQUIRE(result.offers.size() == 3);
  REQUIRE(result.generator_settlement.size() == 3);
  CHECK(result.performance.periods == 24);
  CHECK(result.performance.active_generators == 3);
  CHECK(result.performance.active_ac_branches == 9);
  CHECK(result.performance.estimated_scuc_variables == 2088);
  CHECK(result.performance.estimated_scuc_binary_variables == 72);
  CHECK(result.performance.scuc_mip_start_provided);
  CHECK(result.performance.scuc_structure_hint_provided);
  CHECK(result.performance.scuc_branching_priorities_provided);
  CHECK(result.performance.scuc_warm_start_generation_sec >= 0.0);
  CHECK(result.performance.estimated_sced_variables == 1872);
  CHECK(result.performance.estimated_lodf_dense_bytes == 2456);
  CHECK_FALSE(result.performance.scuc_solver_name.empty());
  CHECK_FALSE(result.performance.pricing_solver_name.empty());
  CHECK(result.performance.scuc_sec >= 0.0);
  CHECK(result.performance.base_sced_sec >= 0.0);
  CHECK(result.performance.total_sec > 0.0);

  for (const auto& period : result.pricing) {
    REQUIRE(period.converged);
    CHECK(period.generator_dispatch_mw.size() == system.ac.generators.size());
    CHECK(period.lmp_per_mwh.size() == system.ac.buses.size());
    CHECK(period.branch_flow_mw.size() == system.ac.branches.size());
    CHECK(period.exogenous_curtailment_mw.size() == system.ac.buses.size());
    CHECK(sum(period.load_shedding_mw) < 1e-6);
    CHECK(sum(period.upward_reserve_mw) ==
          Catch::Approx(period.reserve_requirement_mw).margin(1e-7));
    CHECK(std::all_of(period.lmp_per_mwh.begin(), period.lmp_per_mwh.end(),
                      [](double value) { return std::isfinite(value); }));
    CHECK(std::isfinite(period.upward_reserve_price_per_mwh));
  }

  for (const auto& validation : result.ac_validation) {
    REQUIRE(validation.converged);
    REQUIRE(validation.secure);
    CHECK(validation.status == "secure");
    CHECK(validation.residual < 1e-5);
    CHECK(validation.total_branch_loss_mw >= -1e-6);
    CHECK(validation.maximum_voltage_violation_pu <= 1e-5);
    CHECK(validation.maximum_branch_overload_mva <= 1e-4);
    CHECK(validation.maximum_generator_active_violation_mw <= 1e-5);
  }

  CHECK(result.settlement.cashflow_residual == Catch::Approx(0.0).margin(1e-6));
  CHECK(result.settlement.customer_total_payment ==
        Catch::Approx(result.settlement.resource_total_revenue +
                      result.settlement.congestion_rent).margin(1e-6));
  for (const auto& generator : result.generator_settlement) {
    CHECK(generator.uplift >= -1e-9);
    CHECK(generator.profit_after_uplift >= -1e-6);
  }

  // The market runner operates on snapshots and must not mutate the authored case.
  REQUIRE(system.ac.generators.size() == original.ac.generators.size());
  for (size_t g = 0; g < system.ac.generators.size(); ++g) {
    CHECK(system.ac.generators[g].pg_mw == original.ac.generators[g].pg_mw);
    CHECK(system.ac.generators[g].in_service == original.ac.generators[g].in_service);
  }
}

TEST_CASE("Base AC certification rejects converged voltage-limit violations",
          "[market][case9][ac-security]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  for (auto& bus : system.ac.buses) bus.vmax_pu = 0.95;
  auto time_series = case9_day_profile();
  time_series.num_steps = 1;
  time_series.profiles.front().values = {1.0};
  auto options = market_options();
  options.enable_network_constraints = false;
  options.run_ac_validation = true;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);

  REQUIRE(result.ac_validation.size() == 1);
  const auto& validation = result.ac_validation.front();
  REQUIRE(validation.converged);
  CHECK_FALSE(validation.secure);
  CHECK(validation.maximum_voltage_violation_pu > 1e-4);
  REQUIRE_FALSE(validation.violations.empty());
  CHECK(std::all_of(validation.violations.begin(), validation.violations.end(),
                    [](const auto& violation) {
                      return violation.category == "bus_voltage" &&
                          violation.component_type == "ac_bus" &&
                          violation.component_position >= 0 &&
                          violation.component_index > 0 &&
                          violation.violation > 0.0 &&
                          violation.unit == "pu";
                    }));
  CHECK(validation.status == "ac_security_limits_violated");
  CHECK(result.num_ac_converged == 1);
  CHECK(result.num_ac_secure == 0);
  CHECK_FALSE(result.feasible);
  CHECK(result.status == "ac_validation_failed");

  auto thermal_system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  for (auto& branch : thermal_system.ac.branches) branch.rate_a_mva = 1.0;
  const auto thermal_result = hacdcpf::market::run_day_ahead_market(
      thermal_system, time_series, options);
  REQUIRE(thermal_result.ac_validation.size() == 1);
  const auto& thermal_validation = thermal_result.ac_validation.front();
  REQUIRE(thermal_validation.converged);
  CHECK_FALSE(thermal_validation.secure);
  CHECK(thermal_validation.maximum_branch_overload_mva > 1.0);
  CHECK(thermal_validation.maximum_branch_loading_percent > 100.0);
  REQUIRE_FALSE(thermal_validation.violations.empty());
  CHECK(std::any_of(
      thermal_validation.violations.begin(),
      thermal_validation.violations.end(), [](const auto& violation) {
        return violation.category == "branch_thermal" &&
            violation.component_type == "ac_branch" &&
            violation.from_bus > 0 && violation.to_bus > 0 &&
            violation.actual > violation.upper_limit &&
            violation.violation > 1.0 && violation.unit == "MVA";
      }));
  CHECK_FALSE(thermal_result.feasible);
  CHECK(thermal_result.status == "ac_validation_failed");
}

TEST_CASE("SCUC exact network constraint generation matches the full thermal model",
          "[market][scuc][network-constraint-generation]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  for (auto& branch : system.ac.branches) branch.rate_a_mva = 60.0;
  auto time_series = case9_day_profile();
  time_series.num_steps = 1;
  time_series.profiles.front().values = {1.0};

  auto full_options = market_options();
  full_options.energy_offer_segments = 2;
  full_options.upward_reserve_fraction = 0.0;
  full_options.run_ac_validation = false;
  full_options.uc_options.uc_solver = hacdcpf::UCSolverChoice::HiGHS;
  full_options.enable_scuc_network_constraint_generation = false;
  const auto full = hacdcpf::market::run_day_ahead_market(
      system, time_series, full_options);
  REQUIRE(full.feasible);

  auto generated_options = full_options;
  generated_options.enable_scuc_network_constraint_generation = true;
  generated_options.scuc_network_constraint_generation_min_candidates = 0;
  generated_options.scuc_network_constraint_generation_max_iterations = 20;
  generated_options.structured_scuc_min_binary_variables = 0;
  const auto generated = hacdcpf::market::run_day_ahead_market(
      system, time_series, generated_options);

  INFO("generated SCUC status=" << generated.commitment.solver_status);
  REQUIRE(generated.feasible);
  CHECK(generated.commitment.total_cost ==
        Catch::Approx(full.commitment.total_cost).margin(1e-5));
  CHECK(generated.commitment.network_constraint_generation_run);
  CHECK(generated.commitment.network_constraint_generation_converged);
  CHECK(generated.commitment.network_constraint_generation_iterations >= 1);
  CHECK(generated.commitment.in_solve_network_constraint_generation_used);
  CHECK(generated.commitment.in_solve_network_constraint_callback_calls >= 1);
  CHECK(generated.commitment.in_solve_network_constraints_submitted > 0);
  CHECK(generated.commitment.cross_round_solver_state_reuse_enabled);
  CHECK(generated.commitment.search_tree_rebuilt ==
        (generated.commitment.network_constraint_generation_iterations > 1));
  if (generated.commitment.network_constraint_generation_iterations > 1) {
    CHECK(generated.commitment.cross_round_solver_state_reuse_used);
  }
  CHECK(generated.commitment.network_constraint_candidates == 9);
  CHECK(generated.commitment.network_constraints_activated > 0);
  CHECK(generated.commitment.network_constraints_activated <=
        generated.commitment.network_constraint_candidates);
  CHECK(generated.commitment.network_constraint_remaining_violations == 0);
  CHECK(generated.commitment.network_constraint_worst_violation_mw <= 1e-5);
  REQUIRE(generated.pricing.size() == 1);
  for (size_t l = 0; l < system.ac.branches.size(); ++l) {
    CHECK(std::abs(generated.pricing.front().branch_flow_mw[l]) <=
          system.ac.branches[l].rate_a_mva + 1e-5);
  }

  auto restarted_options = generated_options;
  restarted_options.enable_scuc_in_solve_network_constraint_generation = false;
  const auto restarted = hacdcpf::market::run_day_ahead_market(
      system, time_series, restarted_options);
  REQUIRE(restarted.feasible);
  CHECK(restarted.commitment.total_cost ==
        Catch::Approx(full.commitment.total_cost).margin(1e-5));
  CHECK_FALSE(restarted.commitment.in_solve_network_constraint_generation_used);
  CHECK(restarted.commitment.cross_round_solver_state_reuse_used);
  CHECK(restarted.commitment.cross_round_solver_state_reuse_rounds >= 1);
  CHECK(restarted.commitment.search_tree_rebuilt);
  CHECK((restarted.commitment.root_cuts_reused ||
         restarted.commitment.pseudocosts_reused));
  if (restarted.commitment.root_basis_reused) {
    CHECK(restarted.commitment.root_cuts_reused);
  }

  auto incomplete_options = generated_options;
  incomplete_options.enable_scuc_in_solve_network_constraint_generation = false;
  incomplete_options.scuc_network_constraint_generation_max_iterations = 1;
  incomplete_options.scuc_network_constraint_generation_max_new_per_iteration = 1;
  const auto incomplete = hacdcpf::market::run_day_ahead_market(
      system, time_series, incomplete_options);
  CHECK_FALSE(incomplete.feasible);
  CHECK_FALSE(incomplete.commitment.feasible);
  CHECK(incomplete.commitment.network_constraint_generation_run);
  CHECK_FALSE(incomplete.commitment.network_constraint_generation_converged);
  CHECK(incomplete.commitment.network_constraint_remaining_violations > 0);
  CHECK(incomplete.commitment.network_constraint_worst_violation_mw > 0.0);
  CHECK(incomplete.commitment.solver_status.find(
            "network constraint generation incomplete") != std::string::npos);
}

TEST_CASE("Cost-based offers preserve true cost separately from physical assets",
          "[market][offers]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  const double original_c1 = system.ac.generators.front().cost_c1;
  const auto offers = hacdcpf::market::make_cost_based_offers(system, 6);

  REQUIRE(offers.size() == system.ac.generators.size());
  REQUIRE(offers.front().energy_segments.size() == 6);
  CHECK(offers.front().generator_position == 0);
  CHECK(offers.front().generator_index == system.ac.generators.front().index);
  CHECK(std::is_sorted(
      offers.front().energy_segments.begin(), offers.front().energy_segments.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.price_per_mwh < rhs.price_per_mwh;
      }));
  CHECK(system.ac.generators.front().cost_c1 == original_c1);
}

TEST_CASE("Market SCUC objective uses the complete submitted energy commitment and reserve offer",
          "[market][scuc][offers]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  for (size_t g = 0; g < system.ac.generators.size(); ++g) {
    auto& generator = system.ac.generators[g];
    generator.cost_c0 = 10.0 + static_cast<double>(g);
    generator.startup_cost = 100.0 + 10.0 * static_cast<double>(g);
    generator.shutdown_cost = 40.0 + 5.0 * static_cast<double>(g);
  }
  auto time_series = case9_day_profile();
  time_series.num_steps = 2;
  time_series.profiles.front().values = {0.90, 1.00};

  hacdcpf::market::MarketParticipant portfolio;
  portfolio.participant_id = "portfolio";
  portfolio.generator_positions = {0, 1, 2};
  portfolio.behavior.type =
      hacdcpf::market::BehaviorPolicyType::MarkupAndWithholding;
  portfolio.behavior.energy_markup_fraction = 0.12;
  portfolio.behavior.commitment_markup_fraction = 0.08;
  portfolio.behavior.upward_reserve_price_per_mwh = 7.5;

  auto options = market_options();
  options.participants = {portfolio};
  options.upward_reserve_fraction = 0.08;
  options.run_ac_validation = false;
  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);

  INFO("complete-offer SCUC status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.feasible);
  const double settled_as_bid_cost = std::accumulate(
      result.generator_settlement.begin(), result.generator_settlement.end(),
      0.0, [](double total, const auto& row) {
        return total + row.as_bid_cost;
      });
  CHECK(result.commitment_cost ==
        Catch::Approx(settled_as_bid_cost).margin(1e-5));
  CHECK(sum(result.pricing.front().upward_reserve_mw) > 1e-6);
  CHECK(std::any_of(
      result.offers.begin(), result.offers.end(), [](const auto& offer) {
        return offer.shutdown_price > 0.0 &&
               offer.upward_reserve_price_per_mwh > 0.0;
      }));
}

TEST_CASE("Exogenous renewable surplus is curtailed explicitly",
          "[market][sced][curtailment]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  hacdcpf::StaticGenerator renewable;
  renewable.index = 1001;
  renewable.bus = 5;
  renewable.name = "must-take renewable";
  renewable.p_mw = 500.0;
  renewable.p_rated_mw = 500.0;
  renewable.pmax_mw = 500.0;
  renewable.scaling = 1.0;
  system.ac.static_generators.push_back(renewable);
  auto time_series = case9_day_profile();
  time_series.num_steps = 1;
  time_series.profiles.front().values = {1.0};
  auto options = market_options();
  options.upward_reserve_fraction = 0.0;
  options.exogenous_curtailment_penalty_per_mwh = 5.0;
  options.run_ac_validation = false;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);
  INFO("surplus market status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.feasible);
  REQUIRE(result.pricing.size() == 1);
  CHECK(sum(result.pricing.front().exogenous_curtailment_mw) > 100.0);
  CHECK(sum(result.pricing.front().load_shedding_mw) < 1e-7);
  CHECK(result.settlement.cashflow_residual == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("Load shedding settlement charges only served demand",
          "[market][settlement][load-shedding]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  for (auto& generator : system.ac.generators) {
    generator.pmin_mw = 0.0;
    generator.pmax_mw = 20.0;
    generator.pg_mw = std::min(generator.pg_mw, generator.pmax_mw);
  }
  auto time_series = case9_day_profile();
  time_series.num_steps = 1;
  time_series.profiles.front().values = {1.0};
  auto options = market_options();
  options.upward_reserve_fraction = 0.0;
  options.enable_network_constraints = false;
  options.run_ac_validation = false;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);
  INFO("load-shedding market status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.feasible);
  REQUIRE(result.pricing.size() == 1);
  const auto& period = result.pricing.front();
  REQUIRE(sum(period.load_shedding_mw) > 1.0);
  double expected_payment = 0.0;
  double gross_payment = 0.0;
  for (size_t b = 0; b < system.ac.buses.size(); ++b) {
    const double demand = std::max(0.0, system.ac.buses[b].pd_mw);
    expected_payment += period.lmp_per_mwh[b] *
        (demand - period.load_shedding_mw[b]);
    gross_payment += period.lmp_per_mwh[b] * demand;
  }
  CHECK(result.settlement.customer_energy_payment ==
        Catch::Approx(expected_payment).margin(1e-6));
  CHECK(std::abs(result.settlement.customer_energy_payment - gross_payment) >
        1.0);
  CHECK(result.settlement.cashflow_residual == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("Hybrid market clears remote DC load through VSC and DC branch",
          "[market][hybrid][pricing]") {
  auto system = hacdcpf::io::build_market_5bus_acdc_toy();
  system.vsc_converters[0].control_mode = hacdcpf::ConverterMode::VDC_Q;
  system.vsc_converters[1].in_service = false;
  system.dc.buses[0].bus_type = hacdcpf::DCBusType::DC_V;
  system.dc.buses[1].bus_type = hacdcpf::DCBusType::DC_P;
  hacdcpf::DCLoad load;
  load.index = 77;
  load.bus = system.dc.buses[1].index;
  load.p_mw = 20.0;
  load.scaling = 1.0;
  load.in_service = true;
  system.dc.loads = {load};

  hacdcpf::TimeSeriesData time_series;
  time_series.num_steps = 1;
  time_series.step_duration_hr = 1.0;
  auto options = market_options();
  options.upward_reserve_fraction = 0.0;
  options.run_ac_validation = false;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);
  INFO("status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.feasible);
  REQUIRE(result.pricing.size() == 1);
  const auto& period = result.pricing.front();
  REQUIRE(period.dc_lmp_per_mwh.size() == system.dc.buses.size());
  REQUIRE(period.dc_bus_voltage_pu.size() == system.dc.buses.size());
  REQUIRE(period.dc_branch_flow_mw.size() == system.dc.branches.size());
  REQUIRE(period.vsc_ac_injection_mw.size() == system.vsc_converters.size());
  CHECK(period.dc_load_shedding_mw[1] == Catch::Approx(0.0).margin(1e-7));
  CHECK(period.dc_branch_flow_mw[0] == Catch::Approx(20.0).margin(1e-5));
  CHECK(period.vsc_dc_injection_mw[0] == Catch::Approx(20.0).margin(1e-5));
  CHECK(period.vsc_ac_injection_mw[0] < -20.0);
  CHECK(period.vsc_loss_mw[0] > 0.0);
  CHECK(period.dc_bus_voltage_pu[0] > period.dc_bus_voltage_pu[1]);
  CHECK(std::isfinite(period.dc_lmp_per_mwh[0]));
  CHECK(std::isfinite(period.dc_lmp_per_mwh[1]));
  CHECK(result.model_scope.dc_network_modelled);
  CHECK(result.model_scope.dc_voltage_linearized);
  CHECK(result.model_scope.energy_prices_valid);
  CHECK(result.settlement.customer_dc_energy_payment > 0.0);
  CHECK(result.settlement.cashflow_residual == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("Hybrid market dispatch receives nonlinear AC-DC certification",
          "[market][hybrid][validation]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  system.dc.base_mva = system.base_mva;
  hacdcpf::DCBus dc1;
  dc1.index = 1;
  dc1.bus_type = hacdcpf::DCBusType::DC_V;
  dc1.vm_pu = 1.0;
  dc1.vmin_pu = 0.9;
  dc1.vmax_pu = 1.1;
  hacdcpf::DCBus dc2 = dc1;
  dc2.index = 2;
  dc2.bus_type = hacdcpf::DCBusType::DC_P;
  system.dc.buses = {dc1, dc2};
  hacdcpf::DCBranch dc_line;
  dc_line.index = 41;
  dc_line.from_bus = 1;
  dc_line.to_bus = 2;
  dc_line.r_pu = 0.01;
  dc_line.rate_a_mva = 50.0;
  system.dc.branches = {dc_line};
  hacdcpf::DCLoad dc_load;
  dc_load.index = 51;
  dc_load.bus = 2;
  dc_load.p_mw = 8.0;
  system.dc.loads = {dc_load};
  hacdcpf::VSCConverter vsc;
  vsc.index = 61;
  vsc.bus_ac = system.ac.buses.front().index;
  vsc.bus_dc = 1;
  vsc.control_mode = hacdcpf::ConverterMode::VDC_Q;
  vsc.v_dc_set_pu = 1.0;
  vsc.eta = 0.98;
  vsc.k_vdc = 10.0;
  vsc.pmin_mw = -30.0;
  vsc.pmax_mw = 30.0;
  vsc.qmin_mvar = -20.0;
  vsc.qmax_mvar = 20.0;
  system.vsc_converters = {vsc};

  hacdcpf::TimeSeriesData time_series;
  time_series.num_steps = 1;
  time_series.step_duration_hr = 1.0;
  auto options = market_options();
  options.upward_reserve_fraction = 0.0;
  options.ac_validation_voltage_tolerance_pu = 1e-4;
  options.ac_validation_thermal_tolerance_mva = 1e-3;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);
  INFO("status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.pricing.size() == 1);
  REQUIRE(result.ac_validation.size() == 1);
  const auto& validation = result.ac_validation.front();
  INFO("validation=" << validation.status
       << " residual=" << validation.residual
       << " dc_v=" << validation.maximum_dc_voltage_violation_pu
       << " dc_branch=" << validation.maximum_dc_branch_overload_mw);
  REQUIRE(validation.converged);
  CHECK(validation.secure);
  CHECK(result.feasible);
  CHECK(validation.converter_model_scope.validity.vsc_loss_modelled);
  CHECK(validation.maximum_dc_voltage_violation_pu <= 1e-4);
  CHECK(validation.maximum_dc_branch_overload_mw <= 1e-3);
  CHECK(result.ac_power_flow_results.front().vdc.size() == 2);
}

TEST_CASE("Hybrid market requires a voltage reference on every DC island",
          "[market][hybrid][reference]") {
  auto system = hacdcpf::io::build_market_5bus_acdc_toy();
  hacdcpf::DCBus unreferenced;
  unreferenced.index = 99;
  unreferenced.bus_type = hacdcpf::DCBusType::DC_P;
  unreferenced.vm_pu = 1.0;
  unreferenced.vmin_pu = 0.9;
  unreferenced.vmax_pu = 1.1;
  system.dc.buses.push_back(unreferenced);

  hacdcpf::TimeSeriesData time_series;
  time_series.num_steps = 1;
  time_series.step_duration_hr = 1.0;
  auto options = market_options();
  options.run_ac_validation = false;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);
  CHECK_FALSE(result.feasible);
  CHECK(result.status == "invalid_dc_reference");
  CHECK(std::any_of(result.warnings.begin(), result.warnings.end(),
                    [](const std::string& warning) {
                      return warning.find("DC island") != std::string::npos &&
                             warning.find("buses 99") != std::string::npos;
                    }));
}

TEST_CASE("Strategic policy marks up offers and withholds flexible capacity",
          "[market][behavior][offers]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  const auto original = system;

  hacdcpf::market::MarketParticipant strategic;
  strategic.participant_id = "strategic_firm";
  strategic.participant_name = "Strategic Firm";
  strategic.generator_positions = {2};
  strategic.behavior.type =
      hacdcpf::market::BehaviorPolicyType::MarkupAndWithholding;
  strategic.behavior.energy_markup_fraction = 0.25;
  strategic.behavior.capacity_withholding_fraction = 0.10;
  strategic.behavior.commitment_markup_fraction = 0.05;
  strategic.behavior.upward_reserve_price_per_mwh = 3.0;

  const auto competitive =
      hacdcpf::market::submit_participant_offers(system, {}, 6);
  const auto submission =
      hacdcpf::market::submit_participant_offers(system, {strategic}, 6);

  REQUIRE(submission.offers.size() == 3);
  REQUIRE(submission.actions.size() == 3);
  REQUIRE(submission.participants.size() == 3);
  const auto& offer = submission.offers[2];
  const auto& action = submission.actions[2];
  CHECK(offer.participant_id == "strategic_firm");
  CHECK(offer.physical_maximum_output_mw == Catch::Approx(270.0));
  CHECK(offer.offered_maximum_output_mw == Catch::Approx(244.0));
  CHECK(action.withheld_capacity_mw == Catch::Approx(26.0));
  CHECK(action.energy_markup_fraction == Catch::Approx(0.25));
  CHECK(offer.energy_segments.front().price_per_mwh >
        competitive.offers[2].energy_segments.front().price_per_mwh);
  CHECK(offer.startup_price == Catch::Approx(3150.0));
  CHECK(offer.upward_reserve_price_per_mwh == Catch::Approx(3.0));
  CHECK(system.ac.generators[2].pmax_mw == original.ac.generators[2].pmax_mw);
  CHECK(system.ac.generators[2].cost_c1 == original.ac.generators[2].cost_c1);
}

TEST_CASE("Strategic Case9 clearing preserves true cost and aggregates market power",
          "[market][behavior][settlement]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  auto time_series = case9_day_profile();
  time_series.num_steps = 6;
  time_series.profiles.front().values.resize(6);

  hacdcpf::market::MarketParticipant strategic;
  strategic.participant_id = "strategic_firm";
  strategic.participant_name = "Strategic Firm";
  strategic.generator_positions = {2};
  strategic.behavior.type =
      hacdcpf::market::BehaviorPolicyType::MarkupAndWithholding;
  strategic.behavior.energy_markup_fraction = 0.15;
  strategic.behavior.capacity_withholding_fraction = 0.10;
  strategic.behavior.upward_reserve_price_per_mwh = 2.0;

  auto options = market_options();
  options.run_ac_validation = false;
  options.participants = {strategic};
  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);

  INFO("strategic market status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.feasible);
  REQUIRE(result.participant_settlement.size() == 3);
  REQUIRE(result.behavior_actions.size() == 3);
  CHECK(result.market_power.total_withheld_capacity_mw == Catch::Approx(26.0));
  CHECK(result.market_power.output_hhi > 0.0);
  CHECK(result.market_power.output_hhi <= 10000.0 + 1e-6);
  CHECK(result.market_power.top3_output_share_percent ==
        Catch::Approx(100.0).margin(1e-6));
  CHECK(result.settlement.cashflow_residual == Catch::Approx(0.0).margin(1e-6));

  const auto participant = std::find_if(
      result.participant_settlement.begin(), result.participant_settlement.end(),
      [](const auto& row) { return row.participant_id == "strategic_firm"; });
  REQUIRE(participant != result.participant_settlement.end());
  CHECK(participant->withheld_capacity_mw == Catch::Approx(26.0));
  CHECK(participant->energy_mwh > 0.0);
  CHECK(participant->as_bid_cost > participant->true_cost);
  CHECK(participant->profit_after_uplift ==
        Catch::Approx(participant->market_revenue + participant->uplift -
                      participant->true_cost).margin(1e-6));

  for (const auto& period : result.pricing) {
    CHECK(period.generator_dispatch_mw[2] <= 244.0 + 1e-7);
    CHECK(period.generator_dispatch_mw[2] + period.upward_reserve_mw[2] <=
          244.0 + 1e-7);
  }
  CHECK(system.ac.generators[2].pmax_mw == Catch::Approx(270.0));
}

TEST_CASE("Duplicate participant ownership is rejected",
          "[market][behavior][validation]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  hacdcpf::market::MarketParticipant first;
  first.participant_id = "first";
  first.generator_positions = {0};
  hacdcpf::market::MarketParticipant second;
  second.participant_id = "second";
  second.generator_positions = {0};
  CHECK_THROWS_AS(
      hacdcpf::market::submit_participant_offers(system, {first, second}, 4),
      std::invalid_argument);
}

TEST_CASE("Case9 preventive SCED closes iterative DC N-1 security cuts",
          "[market][security][n-1]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  auto time_series = case9_day_profile();
  time_series.num_steps = 1;
  time_series.profiles.front().values = {1.10};

  auto options = market_options();
  options.run_ac_validation = false;
  options.enable_n1_security = true;
  options.n1_max_iterations = 8;
  options.n1_max_cuts_per_iteration = 100;
  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);

  INFO("security market status=" << result.status);
  INFO("initial violations=" << result.security.initial_violations);
  INFO("final violations=" << result.security.final_violations);
  INFO("cuts=" << result.security.cuts_added);
  INFO("initial worst overload="
       << result.security.initial_worst_overload_mw);
  for (const auto& warning : result.warnings) INFO(warning);
  for (const auto& warning : result.security.warnings) INFO(warning);
  REQUIRE(result.security.enabled);
  REQUIRE(result.security.lodf_available);
  REQUIRE(result.security.candidate_contingencies > 0);
  REQUIRE(result.security.dc_n1_secured);
  CHECK(result.security.final_violations == 0);
  if (result.security.initial_violations > 0) {
    CHECK(result.security.cuts_added > 0);
  } else {
    CHECK(result.security.cuts_added == 0);
  }
  CHECK(result.security.secured_pricing_objective >=
        result.security.baseline_pricing_objective - 1e-7);
  CHECK(result.security.preventive_redispatch_cost >= -1e-9);
  CHECK(result.settlement.cashflow_residual == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("Case9 final dispatch reports nonlinear AC contingency certification",
          "[market][security][ac-contingency]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  auto time_series = case9_day_profile();
  time_series.num_steps = 1;
  time_series.profiles.front().values = {1.0};

  auto options = market_options();
  options.run_ac_validation = false;
  options.run_ac_contingency_validation = true;
  options.max_ac_contingencies = 2;
  options.pricing_native_max_variables = 0;
  options.uc_options.uc_solver = hacdcpf::UCSolverChoice::HiGHS;
  options.structured_scuc_min_binary_variables = 0;
  options.scuc_mip_relative_gap = 0.01;
  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);

  INFO("AC contingency market status=" << result.status);
  for (const auto& warning : result.security.warnings) INFO(warning);
  REQUIRE(result.security.lodf_available);
  REQUIRE(result.security.ac_contingency_validation_run);
  REQUIRE(result.security.ac_checks.size() == 2);
  CHECK(result.performance.lodf_computed_columns == 2);
  CHECK(result.performance.estimated_lodf_sparse_bytes > 0);
  CHECK(result.performance.estimated_lodf_sparse_bytes <
        result.performance.estimated_lodf_dense_bytes);
  CHECK(result.performance.pricing_large_model_direct_highs_used);
  CHECK_FALSE(result.performance.pricing_solver_fallback_used);
  CHECK(result.performance.scuc_structured_branching_used);
  CHECK(result.performance.scuc_mip_gap_target_met);
  for (const auto& check : result.security.ac_checks) {
    CHECK(check.outage_branch_position >= 0);
    CHECK(std::isfinite(check.maximum_voltage_violation_pu));
    CHECK(std::isfinite(check.maximum_branch_overload_mva));
    CHECK_FALSE(check.status.empty());
  }
}

TEST_CASE("Case9 real-time fixed-commitment market closes two-settlement deviations",
          "[market][real-time][settlement]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  const auto original = system;
  auto day_ahead_series = case9_day_profile();
  day_ahead_series.num_steps = 3;
  day_ahead_series.profiles.front().values = {0.90, 1.00, 0.95};
  auto realized_series = day_ahead_series;
  realized_series.profiles.front().values = {0.96, 0.94, 1.04};

  auto day_ahead_options = market_options();
  day_ahead_options.run_ac_validation = false;
  const auto day_ahead = hacdcpf::market::run_day_ahead_market(
      system, day_ahead_series, day_ahead_options);
  REQUIRE(day_ahead.feasible);

  auto rejected_baseline = day_ahead;
  rejected_baseline.feasible = false;
  rejected_baseline.status = "ac_validation_failed";
  hacdcpf::market::RealTimeMarketOptions rejected_options;
  rejected_options.market_options = day_ahead_options;
  const auto rejected_real_time = hacdcpf::market::run_real_time_market(
      system, day_ahead_series, realized_series, rejected_baseline,
      rejected_options);
  CHECK_FALSE(rejected_real_time.feasible);
  CHECK(rejected_real_time.status == "invalid_day_ahead_baseline");
  CHECK(rejected_real_time.real_time_market.pricing.empty());

  hacdcpf::market::RealTimeMarketOptions real_time_options;
  real_time_options.market_options = day_ahead_options;
  const auto real_time = hacdcpf::market::run_real_time_market(
      system, day_ahead_series, realized_series, day_ahead,
      real_time_options);

  INFO("real-time status=" << real_time.status);
  for (const auto& warning : real_time.warnings) INFO(warning);
  REQUIRE(real_time.feasible);
  REQUIRE(real_time.real_time_market.pricing.size() == 3);
  REQUIRE(real_time.periods.size() == 3);
  REQUIRE(real_time.generator_deviation_settlement.size() == 3);
  REQUIRE(real_time.participant_deviation_settlement.size() == 3);
  CHECK(real_time.total_absolute_generator_deviation_mwh > 1e-6);
  CHECK(real_time.settlement.cashflow_residual ==
        Catch::Approx(0.0).margin(1e-6));
  CHECK(real_time.settlement.customer_two_settlement_payment ==
        Catch::Approx(
            real_time.settlement.customer_day_ahead_payment +
            real_time.settlement.customer_real_time_deviation_payment)
            .margin(1e-7));
  CHECK(real_time.settlement.resource_two_settlement_revenue ==
        Catch::Approx(
            real_time.settlement.resource_day_ahead_revenue +
            real_time.settlement.resource_real_time_deviation_revenue)
            .margin(1e-7));
  REQUIRE(real_time.real_time_market.commitment.gen_commit ==
          day_ahead.commitment.gen_commit);
  for (const auto& period : real_time.real_time_market.pricing) {
    CHECK(period.reserve_requirement_mw == Catch::Approx(0.0).margin(1e-9));
    CHECK(sum(period.upward_reserve_mw) == Catch::Approx(0.0).margin(1e-9));
  }
  for (const auto& generator : real_time.generator_deviation_settlement) {
    CHECK(generator.two_settlement_revenue ==
          Catch::Approx(generator.day_ahead_energy_revenue +
                        generator.day_ahead_reserve_revenue +
                        generator.day_ahead_uplift +
                        generator.real_time_deviation_revenue)
              .margin(1e-7));
    CHECK(generator.profit_after_two_settlement ==
          Catch::Approx(generator.two_settlement_revenue -
                        generator.actual_true_cost)
              .margin(1e-7));
  }
  for (size_t g = 0; g < system.ac.generators.size(); ++g) {
    CHECK(system.ac.generators[g].pg_mw == original.ac.generators[g].pg_mw);
  }
}

TEST_CASE("Hybrid real-time market settles DC load forecast deviations",
          "[market][hybrid][real-time][settlement]") {
  auto system = hacdcpf::io::build_market_5bus_acdc_toy();
  system.vsc_converters[0].control_mode = hacdcpf::ConverterMode::VDC_Q;
  system.vsc_converters[1].in_service = false;
  system.dc.buses[0].bus_type = hacdcpf::DCBusType::DC_V;
  system.dc.buses[1].bus_type = hacdcpf::DCBusType::DC_P;
  hacdcpf::DCLoad dc_load;
  dc_load.index = 177;
  dc_load.bus = system.dc.buses[1].index;
  dc_load.p_mw = 20.0;
  dc_load.profile_id = 77;
  system.dc.loads = {dc_load};

  hacdcpf::TimeSeriesData day_ahead_series;
  day_ahead_series.num_steps = 2;
  day_ahead_series.step_duration_hr = 1.0;
  day_ahead_series.profiles.push_back(
      hacdcpf::TimeSeriesProfile{77, "dc_load_forecast", {1.0, 1.0}});
  auto realized_series = day_ahead_series;
  realized_series.profiles.front().values = {1.2, 0.8};

  auto day_ahead_options = market_options();
  day_ahead_options.upward_reserve_fraction = 0.0;
  day_ahead_options.run_ac_validation = false;
  const auto day_ahead = hacdcpf::market::run_day_ahead_market(
      system, day_ahead_series, day_ahead_options);
  REQUIRE(day_ahead.feasible);

  hacdcpf::market::RealTimeMarketOptions real_time_options;
  real_time_options.market_options = day_ahead_options;
  const auto real_time = hacdcpf::market::run_real_time_market(
      system, day_ahead_series, realized_series, day_ahead,
      real_time_options);

  INFO("hybrid real-time status=" << real_time.status);
  for (const auto& warning : real_time.warnings) INFO(warning);
  REQUIRE(real_time.feasible);
  REQUIRE(real_time.periods.size() == 2);
  REQUIRE(real_time.real_time_market.pricing.size() == 2);
  CHECK(real_time.periods[0].demand_deviation_mw ==
        Catch::Approx(4.0).margin(1e-7));
  CHECK(real_time.periods[1].demand_deviation_mw ==
        Catch::Approx(-4.0).margin(1e-7));
  CHECK(std::abs(real_time.periods[0].customer_deviation_payment) > 1e-6);
  CHECK(std::abs(real_time.periods[1].customer_deviation_payment) > 1e-6);
  CHECK(real_time.real_time_market.pricing[0].dc_lmp_per_mwh.size() ==
        system.dc.buses.size());
  CHECK(real_time.settlement.cashflow_residual ==
        Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("DC storage co-optimizes charge discharge and cyclic SOC",
          "[market][hybrid][storage]") {
  auto system = hacdcpf::io::build_market_5bus_acdc_toy();
  system.vsc_converters[0].control_mode = hacdcpf::ConverterMode::VDC_Q;
  system.vsc_converters[1].in_service = false;
  for (auto& generator : system.ac.generators) {
    if (generator.bus <= 2) generator.cost_c2 = 0.5;
  }
  hacdcpf::DCLoad dc_load;
  dc_load.index = 201;
  dc_load.bus = system.dc.buses[1].index;
  dc_load.p_mw = 20.0;
  dc_load.profile_id = 91;
  system.dc.loads = {dc_load};

  hacdcpf::DCStorage storage;
  storage.index = 301;
  storage.bus = system.dc.buses[1].index;
  storage.name = "DC market battery";
  storage.pmin_mw = -15.0;
  storage.pmax_mw = 15.0;
  storage.p_rated_mw = 15.0;
  storage.e_rated_mwh = 40.0;
  storage.soc_init = 0.5;
  storage.soc_min = 0.1;
  storage.soc_max = 0.9;
  storage.eta_charge = 1.0;
  storage.eta_discharge = 1.0;
  storage.daily_cycle_limit = 1.0;
  system.dc.dc_storage = {storage};

  hacdcpf::TimeSeriesData time_series;
  time_series.num_steps = 3;
  time_series.step_duration_hr = 1.0;
  time_series.profiles.push_back(
      hacdcpf::TimeSeriesProfile{91, "dc_peak", {0.5, 1.5, 0.5}});
  auto options = market_options();
  options.upward_reserve_fraction = 0.0;
  options.run_ac_validation = false;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);
  INFO("storage market status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.feasible);
  REQUIRE(result.pricing.size() == 3);
  CHECK(result.model_scope.dc_storage_optimized);
  CHECK(result.model_scope.dc_storage_intertemporal_modelled);
  bool charged = false;
  bool discharged = false;
  for (const auto& period : result.pricing) {
    REQUIRE(period.dc_storage_dispatch_mw.size() == 1);
    REQUIRE(period.dc_storage_soc_mwh.size() == 1);
    charged = charged || period.dc_storage_dispatch_mw[0] < -1e-6;
    discharged = discharged || period.dc_storage_dispatch_mw[0] > 1e-6;
    CHECK(period.dc_storage_soc_mwh[0] >= 4.0 - 1e-7);
    CHECK(period.dc_storage_soc_mwh[0] <= 36.0 + 1e-7);
  }
  CHECK(charged);
  CHECK(discharged);
  CHECK(result.pricing.back().dc_storage_soc_mwh[0] ==
        Catch::Approx(20.0).margin(1e-7));
  REQUIRE(result.dc_storage_settlement.size() == 1);
  CHECK(result.dc_storage_settlement[0].charge_mwh > 1e-6);
  CHECK(result.dc_storage_settlement[0].discharge_mwh > 1e-6);
  CHECK(result.dc_storage_settlement[0].terminal_soc_mwh ==
        Catch::Approx(20.0).margin(1e-7));
  CHECK(result.settlement.cashflow_residual ==
        Catch::Approx(0.0).margin(1e-6));

  auto realized_series = time_series;
  realized_series.profiles.front().values = {1.5, 0.5, 0.5};
  hacdcpf::market::RealTimeMarketOptions real_time_options;
  real_time_options.market_options = options;
  const auto real_time = hacdcpf::market::run_real_time_market(
      system, time_series, realized_series, result, real_time_options);
  INFO("storage real-time status=" << real_time.status);
  for (const auto& warning : real_time.warnings) INFO(warning);
  REQUIRE(real_time.feasible);
  double expected_storage_deviation_revenue = 0.0;
  bool storage_dispatch_changed = false;
  for (size_t t = 0; t < real_time.periods.size(); ++t) {
    const auto& day_ahead_period = result.pricing[t];
    const auto& real_time_period = real_time.real_time_market.pricing[t];
    const double deviation = real_time_period.dc_storage_dispatch_mw[0] -
        day_ahead_period.dc_storage_dispatch_mw[0];
    storage_dispatch_changed = storage_dispatch_changed ||
        std::abs(deviation) > 1e-6;
    expected_storage_deviation_revenue += deviation *
        real_time_period.dc_lmp_per_mwh[1] * time_series.step_duration_hr;
  }
  CHECK(storage_dispatch_changed);
  const double reported_storage_deviation_revenue = std::accumulate(
      real_time.periods.begin(), real_time.periods.end(), 0.0,
      [](double total, const auto& period) {
        return total + period.dc_storage_deviation_revenue;
      });
  CHECK(reported_storage_deviation_revenue ==
        Catch::Approx(expected_storage_deviation_revenue).margin(1e-7));
  const double period_resource_deviation_revenue = std::accumulate(
      real_time.periods.begin(), real_time.periods.end(), 0.0,
      [](double total, const auto& period) {
        return total + period.resource_deviation_revenue;
      });
  CHECK(period_resource_deviation_revenue ==
        Catch::Approx(real_time.settlement.resource_real_time_deviation_revenue)
            .margin(1e-7));
  CHECK(real_time.settlement.cashflow_residual ==
        Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("Full-component N-1 enumerates AC DC converter generator and storage",
          "[market][hybrid][n-1][all-components]") {
  auto system = hacdcpf::io::build_market_5bus_acdc_toy();
  system.ac.loads.back().p_mw = 0.0;
  system.ac.loads.back().q_mvar = 0.0;

  hacdcpf::DCDCConverter dcdc;
  dcdc.index = 401;
  dcdc.bus_in = system.dc.buses[0].index;
  dcdc.bus_out = system.dc.buses[1].index;
  dcdc.control_mode = hacdcpf::DCDCControlMode::Power;
  dcdc.p_ref_mw = 0.0;
  dcdc.pmin_mw = -10.0;
  dcdc.pmax_mw = 10.0;
  dcdc.eta = 0.98;
  system.dc.dcdc_converters = {dcdc};

  hacdcpf::DCStorage storage;
  storage.index = 402;
  storage.bus = system.dc.buses[1].index;
  storage.pmin_mw = -5.0;
  storage.pmax_mw = 5.0;
  storage.e_rated_mwh = 10.0;
  storage.soc_init = 0.5;
  storage.soc_min = 0.1;
  storage.soc_max = 0.9;
  system.dc.dc_storage = {storage};

  hacdcpf::TimeSeriesData time_series;
  time_series.num_steps = 1;
  time_series.step_duration_hr = 1.0;
  auto options = market_options();
  options.upward_reserve_fraction = 0.0;
  options.run_ac_validation = false;
  options.enable_n1_security = true;
  options.n1_max_contingencies = 0;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);
  INFO("full N-1 status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.security.full_component_validation_run);
  CHECK(result.model_scope.hybrid_n1_modelled);
  CHECK(result.model_scope.full_component_n1_modelled);
  CHECK(result.model_scope.n1_recourse_policy.find("corrective") !=
        std::string::npos);
  const auto has_type = [&](const std::string& type) {
    return std::any_of(
        result.security.component_checks.begin(),
        result.security.component_checks.end(),
        [&](const auto& check) { return check.component_type == type; });
  };
  CHECK(has_type("ac_generator"));
  CHECK(has_type("ac_branch"));
  CHECK(has_type("dc_branch"));
  CHECK(has_type("vsc_converter"));
  CHECK(has_type("dcdc_converter"));
  CHECK(has_type("dc_storage"));
  CHECK(result.security.full_component_candidate_contingencies ==
        static_cast<int>(result.security.component_checks.size()));
  CHECK(result.performance.component_n1_parallel_requested);
  CHECK(result.performance.component_n1_parallel_workers >= 1);
  CHECK(result.performance.component_n1_parallel_effective ==
        (result.performance.component_n1_parallel_workers > 1));
  for (const auto& check : result.security.component_checks) {
    CHECK(check.component_position >= 0);
    CHECK(check.component_index >= 0);
    CHECK_FALSE(check.status.empty());
  }
}

TEST_CASE("Fixed-commitment SCED enforces initial and inter-period ramp deliverability",
          "[market][sced][ramp]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  for (auto& generator : system.ac.generators) {
    generator.ramp_up_mw_min = 3.0;
    generator.ramp_dn_mw_min = 3.0;
  }
  auto time_series = case9_day_profile();
  time_series.num_steps = 3;
  time_series.profiles.front().values = {0.90, 1.00, 0.92};
  auto options = market_options();
  options.upward_reserve_fraction = 0.05;
  options.run_ac_validation = false;

  const auto result = hacdcpf::market::run_day_ahead_market(
      system, time_series, options);
  INFO("ramp-constrained market status=" << result.status);
  for (const auto& warning : result.warnings) INFO(warning);
  REQUIRE(result.feasible);
  REQUIRE(result.pricing.size() == 3);
  const double ramp_mw = 3.0 * 60.0 * time_series.step_duration_hr;
  for (size_t g = 0; g < system.ac.generators.size(); ++g) {
    double previous = system.ac.generators[g].pg_mw;
    for (const auto& period : result.pricing) {
      const double dispatch = period.generator_dispatch_mw[g];
      const double reserve = period.upward_reserve_mw[g];
      CHECK(dispatch + reserve - previous <= ramp_mw + 1e-6);
      CHECK(previous - dispatch <= ramp_mw + 1e-6);
      previous = dispatch;
    }
  }
}

TEST_CASE("Case9 real-time reserve performance and imbalance charges close the ancillary ledger",
          "[market][real-time][ancillary-services]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  auto day_ahead_series = case9_day_profile();
  day_ahead_series.num_steps = 1;
  day_ahead_series.profiles.front().values = {1.00};
  auto realized_series = day_ahead_series;
  realized_series.profiles.front().values = {1.10};

  auto day_ahead_options = market_options();
  day_ahead_options.upward_reserve_fraction = 0.10;
  day_ahead_options.run_ac_validation = false;
  const auto day_ahead = hacdcpf::market::run_day_ahead_market(
      system, day_ahead_series, day_ahead_options);
  REQUIRE(day_ahead.feasible);

  hacdcpf::market::RealTimeMarketOptions real_time_options;
  real_time_options.market_options = day_ahead_options;
  auto& ancillary = real_time_options.ancillary_services;
  ancillary.enabled = true;
  ancillary.generator_imbalance_tolerance_fraction = 0.01;
  ancillary.load_imbalance_tolerance_fraction = 0.02;
  ancillary.generator_imbalance_penalty_per_mwh = 60.0;
  ancillary.load_imbalance_penalty_per_mwh = 80.0;
  ancillary.reserve_performance_payment_per_mwh = 12.0;
  ancillary.reserve_nonperformance_penalty_per_mwh = 120.0;
  ancillary.reserve_performance_factor_by_generator.assign(
      system.ac.generators.size(), 0.0);

  const auto real_time = hacdcpf::market::run_real_time_market(
      system, day_ahead_series, realized_series, day_ahead,
      real_time_options);

  INFO("ancillary real-time status=" << real_time.status);
  for (const auto& warning : real_time.warnings) INFO(warning);
  REQUIRE(real_time.feasible);
  REQUIRE(real_time.dispatch_instruction_market.feasible);
  REQUIRE(real_time.dispatch_instruction_market.pricing.size() == 1);
  REQUIRE(real_time.periods.size() == 1);
  const auto& period = real_time.periods.front();
  CHECK(period.reserve_activation_requirement_mw > 1e-6);
  CHECK(period.reserve_instruction_mw > 1e-6);
  CHECK(period.reserve_delivered_mw == Catch::Approx(0.0).margin(1e-9));
  CHECK(period.reserve_shortfall_mw ==
        Catch::Approx(period.reserve_instruction_mw).margin(1e-8));
  CHECK(period.load_penalized_imbalance_mwh > 1e-6);
  CHECK(period.load_imbalance_penalty > 1e-6);

  double instructed = 0.0;
  double shortfall = 0.0;
  double instruction_dispatch_mw = 0.0;
  double actual_dispatch_mw = 0.0;
  double replacement_dispatch_mw = 0.0;
  bool saw_failed_reserve_response = false;
  const auto& instruction_pricing =
      real_time.dispatch_instruction_market.pricing.front();
  const auto& actual_pricing = real_time.real_time_market.pricing.front();
  for (size_t g = 0; g < system.ac.generators.size(); ++g) {
    instruction_dispatch_mw += instruction_pricing.generator_dispatch_mw[g];
    actual_dispatch_mw += actual_pricing.generator_dispatch_mw[g];
    if (real_time.generator_deviation_settlement[g].instructed_reserve_mwh <=
        1e-9) {
      replacement_dispatch_mw += std::max(
          0.0, actual_pricing.generator_dispatch_mw[g] -
                   instruction_pricing.generator_dispatch_mw[g]);
    }
  }
  for (const auto& generator : real_time.generator_deviation_settlement) {
    instructed += generator.instructed_reserve_mwh;
    shortfall += generator.reserve_shortfall_mwh;
    CHECK(generator.reserve_performance_payment ==
          Catch::Approx(generator.delivered_reserve_mwh * 12.0).margin(1e-8));
    CHECK(generator.reserve_nonperformance_charge ==
          Catch::Approx(generator.reserve_shortfall_mwh * 120.0).margin(1e-8));
    if (generator.instructed_reserve_mwh > 1e-8) {
      saw_failed_reserve_response = true;
      CHECK(generator.actual_output_deviation_mwh ==
            Catch::Approx(-generator.reserve_shortfall_mwh).margin(1e-7));
      CHECK(generator.real_time_dispatch_instruction_mwh >
            generator.real_time_energy_mwh);
    }
    CHECK(generator.imbalance_charge ==
          Catch::Approx(generator.penalized_imbalance_mwh * 60.0).margin(1e-8));
    CHECK(generator.two_settlement_revenue ==
          Catch::Approx(generator.day_ahead_energy_revenue +
                        generator.day_ahead_reserve_revenue +
                        generator.day_ahead_uplift +
                        generator.real_time_deviation_revenue +
                        generator.reserve_performance_payment -
                        generator.reserve_nonperformance_charge -
                        generator.imbalance_charge)
              .margin(1e-7));
  }
  CHECK(instructed == Catch::Approx(period.reserve_instruction_mw).margin(1e-8));
  CHECK(shortfall == Catch::Approx(period.reserve_shortfall_mw).margin(1e-8));
  CHECK(saw_failed_reserve_response);
  CHECK(actual_dispatch_mw ==
        Catch::Approx(instruction_dispatch_mw).margin(1e-7));
  CHECK(replacement_dispatch_mw > 1e-6);
  CHECK(real_time.real_time_market.pricing.front().reserve_requirement_mw ==
        Catch::Approx(0.0).margin(1e-9));
  CHECK(real_time.settlement.customer_imbalance_penalty > 1e-6);
  CHECK(real_time.settlement.resource_reserve_nonperformance_charge > 1e-6);
  CHECK(real_time.settlement.resource_generator_imbalance_charge >= 0.0);
  CHECK(real_time.settlement.system_operator_ancillary_balance > 1e-6);
  CHECK(real_time.settlement.cashflow_residual ==
        Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("Case9 repeated participant game evaluates bounded local best responses",
          "[market][game][best-response]") {
  auto system = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
  auto day_ahead_series = case9_day_profile();
  day_ahead_series.num_steps = 1;
  day_ahead_series.profiles.front().values = {1.00};
  auto realized_series = day_ahead_series;
  realized_series.profiles.front().values = {1.05};

  hacdcpf::market::MarketParticipant strategic;
  strategic.participant_id = "strategic_firm";
  strategic.participant_name = "Strategic Firm";
  // A portfolio spanning all three generators is deliberately used here so a
  // one-step uniform markup has a deterministic profitable direction.  A
  // single marginal/non-marginal unit is not guaranteed to have one once SCUC
  // evaluates the complete piecewise offer curve.
  strategic.generator_positions = {0, 1, 2};
  strategic.behavior.type =
      hacdcpf::market::BehaviorPolicyType::MarkupAndWithholding;
  strategic.behavior.energy_markup_fraction = 0.10;
  strategic.behavior.capacity_withholding_fraction = 0.05;

  hacdcpf::market::RepeatedGameOptions options;
  options.max_rounds = 3;
  options.markup_step_fraction = 0.05;
  options.withholding_step_fraction = 0.05;
  options.maximum_markup_fraction = 0.30;
  options.maximum_withholding_fraction = 0.20;
  options.day_ahead_options = market_options();
  options.day_ahead_options.run_ac_validation = false;
  options.day_ahead_options.participants = {strategic};
  options.real_time_options.market_options = options.day_ahead_options;
  options.real_time_options.ancillary_services.enabled = true;
  options.real_time_options.ancillary_services
      .reserve_performance_factor_by_generator.assign(
          system.ac.generators.size(), 0.5);
  const auto game = hacdcpf::market::run_repeated_market_game(
      system, day_ahead_series, realized_series, options);

  INFO("game status=" << game.status);
  for (const auto& warning : game.warnings) INFO(warning);
  REQUIRE_FALSE(game.rounds.empty());
  REQUIRE(game.rounds.size() <= 3);
  REQUIRE(game.final_day_ahead.commitment.feasible);
  REQUIRE_FALSE(game.final_real_time.real_time_market.pricing.empty());
  bool saw_strategic = false;
  bool saw_change = false;
  for (const auto& round : game.rounds) {
    CHECK(round.day_ahead_feasible);
    CHECK(round.real_time_feasible);
    for (const auto& participant : round.participants) {
      CHECK(participant.best_response_profit + 1e-7 >= participant.profit);
      if (participant.participant_id != "strategic_firm") continue;
      saw_strategic = true;
      saw_change = saw_change || participant.strategy_changed;
      CHECK(participant.next_behavior.energy_markup_fraction <= 0.30 + 1e-12);
      CHECK(participant.next_behavior.capacity_withholding_fraction <=
            0.20 + 1e-12);
    }
  }
  CHECK(saw_strategic);
  CHECK(saw_change);
  REQUIRE_FALSE(game.final_real_time.participant_deviation_settlement.empty());
  const auto& final_round = game.rounds.back();
  for (const auto& settlement :
       game.final_real_time.participant_deviation_settlement) {
    const auto round_row = std::find_if(
        final_round.participants.begin(), final_round.participants.end(),
        [&](const auto& row) {
          return row.participant_id == settlement.participant_id;
        });
    REQUIRE(round_row != final_round.participants.end());
    CHECK(round_row->profit ==
          Catch::Approx(settlement.profit_after_two_settlement).margin(1e-7));
    CHECK(round_row->net_ancillary_adjustment ==
          Catch::Approx(settlement.net_ancillary_adjustment).margin(1e-7));
    CHECK(settlement.two_settlement_revenue ==
          Catch::Approx(settlement.day_ahead_market_revenue +
                        settlement.real_time_deviation_revenue +
                        settlement.net_ancillary_adjustment)
              .margin(1e-7));
  }

  auto limited_options = options;
  limited_options.max_rounds = 1;
  limited_options.real_time_options.ancillary_services.enabled = false;
  const auto limited_game = hacdcpf::market::run_repeated_market_game(
      system, day_ahead_series, realized_series, limited_options);
  REQUIRE(limited_game.rounds.size() == 1);
  CHECK_FALSE(limited_game.converged);
  CHECK(limited_game.status == "maximum_rounds_reached");
  const auto limited_strategic = std::find_if(
      limited_game.rounds.front().participants.begin(),
      limited_game.rounds.front().participants.end(),
      [](const auto& row) { return row.participant_id == "strategic_firm"; });
  REQUIRE(limited_strategic !=
          limited_game.rounds.front().participants.end());
  CHECK(limited_strategic->strategy_changed);
  CHECK(limited_strategic->best_response_improvement >
        limited_options.profit_improvement_tolerance);

  auto no_learning_options = limited_options;
  no_learning_options.day_ahead_options.participants.clear();
  no_learning_options.real_time_options.market_options.participants.clear();
  no_learning_options.include_cost_based_participants = false;
  const auto no_learning_game = hacdcpf::market::run_repeated_market_game(
      system, day_ahead_series, realized_series, no_learning_options);
  REQUIRE(no_learning_game.rounds.size() == 1);
  CHECK(no_learning_game.converged);
  CHECK(no_learning_game.status == "converged");
}
