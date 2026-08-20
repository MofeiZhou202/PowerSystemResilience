#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "hacdcpf/reliability/intelligent_cyber_physical.hpp"
#include "hacdcpf/reliability/online_protection.hpp"

using Catch::Approx;
using namespace hacdcpf;
using namespace hacdcpf::analysis;

TEST_CASE("intelligent detection integrates confusion and latency classes",
          "[reliability][intelligent][detection]") {
  IntelligentDetectionModel model;
  model.fault_class_ids = {"line", "bus"};
  model.confusion_matrix = {
      {0.99, 0.005, 0.005},
      {0.10, 0.80, 0.10},
      {0.20, 0.05, 0.75}};
  model.latency_by_true_class = {
      {{0.75, 0.1}, {0.25, 1.0}},
      {{1.0, 0.2}}};
  model.timely_detection_limit_s = 0.5;
  model.no_fault_decision_windows_per_year = 1000.0;
  const auto result = evaluate_intelligent_detection(model);
  CHECK(result.probabilities_normalized);
  CHECK(result.detection_probability[0] == Approx(0.9));
  CHECK(result.correct_classification_probability[0] == Approx(0.8));
  CHECK(result.expected_detection_latency_s[0] ==
        Approx(0.75 * 0.1 + 0.25 * 1.0));
  CHECK(result.timely_detection_probability[0] == Approx(0.9 * 0.75));
  CHECK(result.false_alarm_frequency_per_year == Approx(10.0));
}

TEST_CASE("intelligent isolation minimizes posterior risk with function gates",
          "[reliability][intelligent][isolation]") {
  IntelligentIsolationInput input;
  input.posterior_fault_probability = {0.7, 0.3};
  input.candidates = {
      {"narrow", {0}, 1.0, 1.0, 1.0, {0}},
      {"wide", {0, 1}, 5.0, 2.0, 1.0, {1}}};
  input.missed_isolation_cost = 100.0;
  input.isolated_load_cost = 1.0;
  input.information_function_available = {true, false};
  const auto result = select_minimum_risk_isolation(input);
  CHECK(result.selected_candidate_id == "narrow");
  CHECK(result.selected_expected_cost == Approx(31.0));
  CHECK_FALSE(result.candidates[1].executable);
}

TEST_CASE("risk constrained restoration uses manual fallback and rejects unsafe action",
          "[reliability][intelligent][restoration]") {
  RiskConstrainedRestorationInput input;
  input.actions = {
      {"unsafe-fast", 10.0, 0.0, 0.2, 1.0, 1.0, 60.0, {}},
      {"safe", 8.0, 1.0, 0.01, 0.9, 2.0, 120.0, {0}}};
  input.information_function_available = {false};
  input.maximum_unsafe_probability = 0.05;
  input.unserved_energy_cost_per_mwh = 10.0;
  input.decision_horizon_hr = 1.0;
  const auto result = select_risk_constrained_restoration(input);
  CHECK(result.selected_action_id == "safe");
  CHECK(result.used_manual_fallback);
  CHECK(result.selected_duration_s == Approx(120.0));
  REQUIRE(result.rejected_actions.size() == 1);
}

TEST_CASE("finite POMDP exact belief tree selects information gathering",
          "[reliability][intelligent][pomdp]") {
  FinitePOMDPModel model;
  model.state_ids = {"fault-a", "fault-b"};
  model.action_ids = {"inspect", "isolate-a", "isolate-b"};
  model.observation_ids = {"a", "b"};
  model.initial_belief = {0.5, 0.5};
  model.transition_probability = {
      {{1.0, 0.0}, {0.0, 1.0}},
      {{1.0, 0.0}, {0.0, 1.0}},
      {{1.0, 0.0}, {0.0, 1.0}}};
  model.observation_probability = {
      {{0.9, 0.1}, {0.1, 0.9}},
      {{0.5, 0.5}, {0.5, 0.5}},
      {{0.5, 0.5}, {0.5, 0.5}}};
  model.stage_cost = {{1.0, 0.0, 10.0}, {1.0, 10.0, 0.0}};
  model.terminal_cost = {0.0, 0.0};
  model.horizon_steps = 2;
  const auto result = solve_finite_pomdp(model);
  CHECK(result.exact_for_initial_belief);
  CHECK(result.first_action_id == "inspect");
  CHECK(result.expected_total_cost == Approx(2.0).margin(1e-12));
  CHECK(result.belief_nodes_evaluated > 1);
}

namespace {

ProtectionCyberReliabilityScenario make_chronological_scenario() {
  ProtectionCyberReliabilityScenario scenario;
  scenario.scenario_id = "scheduled-fault";
  scenario.initiating_frequency_per_year = 0.0;
  auto& event = scenario.event;
  event.primary.relay.relay_id = "primary";
  event.primary.relay.characteristic =
      ProtectionRelayCharacteristic::DefiniteTimeOvercurrent;
  event.primary.relay.pickup_current = 1.0;
  event.primary.relay.definite_time_delay_s = 0.1;
  event.primary.trajectory = {{0.0, 2.0}, {10.0, 2.0}};
  event.primary.relay_success_probability = 1.0;
  event.primary.breaker_success_probability = 1.0;
  event.backup = event.primary;
  event.backup.relay.relay_id = "backup";
  event.backup.relay.definite_time_delay_s = 0.5;
  event.coordination_margin_s = 0.3;
  event.uncleared_terminal_time_s = 1.0;
  return scenario;
}

}  // namespace

TEST_CASE("chronological reliability unions overlapping interruption intervals",
          "[reliability][intelligent][chronological]") {
  auto scenario = make_chronological_scenario();
  scenario.consequence.primary_clearing_shed_mw = 0.0;
  scenario.consequence.backup_clearing_shed_mw = 0.0;
  scenario.consequence.uncleared_shed_mw = 0.0;
  scenario.consequence.isolated_shed_mw = 10.0;
  scenario.consequence.isolation_failed_shed_mw = 10.0;
  scenario.consequence.restored_shed_mw = 0.0;
  scenario.consequence.restoration_failed_shed_mw = 0.0;
  scenario.consequence.automatic_restoration_hr = 2.0;
  scenario.consequence.manual_restoration_hr = 2.0;
  scenario.consequence.repair_hr = 2.0 + 0.1 / 3600.0;

  ChronologicalPhysicalFault fault;
  fault.scenario = scenario;
  fault.scheduled_occurrence_hours = {1.0, 2.0};
  ChronologicalCyberPhysicalOptions options;
  options.simulated_years = 1;
  options.hours_per_year = 10.0;
  options.random_seed = 7;
  const auto result = run_chronological_cyber_physical_reliability(
      {}, {}, {}, {fault}, options);

  CHECK(result.physical_faults_sampled == 2);
  CHECK(result.eens_mwh_yr == Approx(40.0).margin(1e-10));
  CHECK(result.lole_hr_yr == Approx(3.0).margin(1e-10));
  CHECK(result.lolf_occ_yr == Approx(1.0));
  CHECK(result.merged_interruption_intervals == 1);
  CHECK(result.outage_interval_union_modelled);
}

TEST_CASE("chronological cyber battery depletion changes later protection outcome",
          "[reliability][intelligent][chronological][power]") {
  ChronologicalCyberComponent center;
  center.service.id = "control-center";
  center.service.packet_delivery_probability = 1.0;
  center.service.supplied_by_bus_id = "ac:1";
  center.service.backup_energy_wh = 10.0;
  center.service.power_draw_w = 10.0;
  center.backup_recharge_power_w = 0.0;
  ProtectionCyberFunction control;
  control.name = "control";
  control.alternative_paths = {{{0}}};

  auto harmless = make_chronological_scenario();
  harmless.event.function_bindings.detection_function = 0;
  harmless.event.function_bindings.isolation_function = 0;
  harmless.event.function_bindings.restoration_function = 0;
  harmless.consequence.repair_hr = 0.0;
  ChronologicalPhysicalFault first;
  first.scenario = harmless;
  first.deenergized_bus_ids = {"ac:1"};
  first.information_power_outage_duration_hr = 3.0;
  first.scheduled_occurrence_hours = {0.0};

  auto consequential = harmless;
  consequential.consequence.uncleared_shed_mw = 10.0;
  consequential.consequence.isolation_failed_shed_mw = 10.0;
  consequential.consequence.restoration_failed_shed_mw = 10.0;
  consequential.consequence.manual_restoration_hr = 1.0;
  consequential.consequence.repair_hr = 1.0 + 1.0 / 3600.0;
  ChronologicalPhysicalFault second;
  second.scenario = consequential;
  second.scheduled_occurrence_hours = {2.0};

  ChronologicalCyberPhysicalOptions options;
  options.simulated_years = 1;
  options.hours_per_year = 10.0;
  options.random_seed = 11;
  const auto depleted = run_chronological_cyber_physical_reliability(
      {center}, {control}, {}, {first, second}, options);

  CHECK(depleted.cyber_power_coupling_modelled);
  CHECK(depleted.battery_energy_trajectory_modelled);
  CHECK(depleted.packet_delivery_sampled);
  CHECK(depleted.minimum_battery_energy_wh == Approx(0.0));
  CHECK(depleted.eens_mwh_yr ==
        Approx(10.0 * (1.0 + 1.0 / 3600.0)).margin(1e-9));
  CHECK(depleted.lole_hr_yr == Approx(1.0 + 1.0 / 3600.0).margin(1e-9));
}

TEST_CASE("online protection DAE derives and applies branch trip from trajectory",
          "[reliability][protection-frt][online-dae]") {
  HybridPowerSystem system;
  system.base_mva = 100.0;
  system.ac.base_mva = 100.0;
  system.ac.freq_hz = 50.0;
  ACBus source;
  source.index = 1;
  source.bus_type = BusType::SLACK;
  source.vm_pu = 1.02;
  ACBus fault_bus;
  fault_bus.index = 2;
  fault_bus.bus_type = BusType::PQ;
  fault_bus.vm_pu = 1.0;
  system.ac.buses = {source, fault_bus};
  ACBranch first;
  first.index = 101;
  first.from_bus = 1;
  first.to_bus = 2;
  first.r_pu = 0.02;
  first.x_pu = 0.1;
  ACBranch second = first;
  second.index = 102;
  system.ac.branches = {first, second};
  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.is_slack = true;
  slack.vg_pu = 1.02;
  slack.pg_mw = 20.0;
  slack.pmax_mw = 100.0;
  slack.qmin_mvar = -100.0;
  slack.qmax_mvar = 100.0;
  slack.xdpp_pu = 0.2;
  system.ac.generators = {slack};
  Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 10.0;
  load.q_mvar = 2.0;
  system.ac.loads = {load};

  OnlineProtectionDAEInput input;
  input.fault_ac_bus_id = 2;
  input.fault_phase = 0;
  input.fault_time_s = 0.05;
  input.fault_r_pu = 0.2;
  input.protected_ac_branch_index = 101;
  input.relay.relay_id = "online-50";
  input.relay.characteristic =
      ProtectionRelayCharacteristic::DefiniteTimeOvercurrent;
  input.relay.pickup_current = 0.2;
  input.relay.definite_time_delay_s = 0.02;
  input.breaker.mechanical_delay_s = 0.01;
  input.instrument_transformers.ct_ratio = 1.0;
  input.instrument_transformers.pt_ratio = 1.0;
  OnlineDERFRTMonitor monitor;
  monitor.stable_id = "der-at-bus-2";
  monitor.ac_bus_id = 2;
  monitor.settings = dynamics::make_default_ieee1547(
      dynamics::IEEE1547Category::CategoryII, 50.0);
  monitor.settings.enabled = true;
  input.der_monitors = {monitor};
  input.dynamic_options.t_end_s = 0.2;
  input.dynamic_options.dt_s = 0.01;
  input.dynamic_options.run_power_flow_initialization = false;
  input.dynamic_options.trim_dynamic_initial_conditions = false;
  input.dynamic_options.enforce_voltage_health_check = false;
  input.dynamic_options.singular_regularization_pu = 1e-7;

  auto invalid = input;
  invalid.breaker.trip_coil_delay_s = -0.01;
  CHECK_THROWS_AS(run_online_protection_dae(system, invalid),
                  std::invalid_argument);

  const auto result = run_online_protection_dae(system, input);
  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.online_network_dae_coupled);
  CHECK(result.relay_trajectory_from_dae);
  CHECK(result.relay.operated);
  CHECK(result.protection_clear_time_s > input.fault_time_s);
  CHECK(result.protection_action_applied);
  CHECK(result.der_frt_state_machine_consumed);
  REQUIRE(result.der_results.size() == 1);
  CHECK(result.discovery_run.success);
  CHECK(result.closed_loop_run.success);
  REQUIRE_FALSE(result.measured_relay_trajectory.empty());
  REQUIRE(result.discovery_der_trajectories.size() == 1);
  REQUIRE(result.closed_loop_der_trajectories.size() == 1);

  auto radial_system = system;
  radial_system.ac.branches.resize(1);
  const auto radial_result = run_online_protection_dae(radial_system, input);
  INFO(radial_result.message);
  REQUIRE(radial_result.success);
  REQUIRE(radial_result.deenergized_ac_bus_ids == std::vector<int>{2});
  CHECK(radial_result.dead_island_load_shedding_applied);
  CHECK(radial_result.closed_loop_run.success);

  OnlineProtectionCyberReliabilityScenario annual;
  annual.scenario_id = "online-primary-backup";
  annual.initiating_frequency_per_year = 1.0;
  annual.primary = input;
  annual.backup = input;
  annual.backup.protected_ac_branch_index = 102;
  annual.backup.relay.relay_id = "online-backup-50";
  annual.backup.relay.definite_time_delay_s = 0.05;
  annual.coordination_margin_s = 0.02;
  annual.uncleared_terminal_time_s = 0.2;
  ProtectionCyberComponent channel;
  channel.id = "shared-protection-channel";
  channel.intrinsic_availability = 0.8;
  ProtectionCyberFunction detection;
  detection.name = "fault-detection";
  detection.alternative_paths = {{{0}}};
  annual.information_components = {channel};
  annual.information_functions = {detection};
  annual.function_bindings.detection_function = 0;
  annual.consequence.primary_clearing_shed_mw = 10.0;
  annual.consequence.backup_clearing_shed_mw = 10.0;
  annual.consequence.uncleared_shed_mw = 10.0;
  annual.consequence.isolated_shed_mw = 2.0;
  annual.consequence.isolation_failed_shed_mw = 10.0;
  annual.consequence.restored_shed_mw = 0.0;
  annual.consequence.restoration_failed_shed_mw = 10.0;
  annual.consequence.automatic_restoration_hr = 0.01;
  annual.consequence.manual_restoration_hr = 1.0;
  annual.consequence.repair_hr = 4.0;
  const auto annual_result = compare_online_protection_cyber_reliability(
      system, {annual});
  REQUIRE(annual_result.dae_trajectories_consumed_by_event_tree);
  REQUIRE(annual_result.comparison.validity.online_network_dae_coupled);
  REQUIRE(annual_result.diagnostics.size() == 1);
  CHECK(annual_result.diagnostics[0].primary_event_tree_clear_time_s ==
        Approx(annual_result.diagnostics[0].primary.protection_clear_time_s)
            .margin(input.dynamic_options.dt_s + 1e-9));
  CHECK(annual_result.diagnostics[0].backup_event_tree_clear_time_s ==
        Approx(annual_result.diagnostics[0].backup.protection_clear_time_s)
            .margin(input.dynamic_options.dt_s + 1e-9));
  CHECK(annual_result.comparison.static_fmea_eens_mwh_yr == Approx(40.0));
  CHECK(annual_result.comparison.cyber_conditioned_eens_mwh_yr >=
        annual_result.comparison.protection_only_eens_mwh_yr);

  for (const bool grid_forming : {false, true}) {
    auto converter_system = system;
    VSCConverter converter;
    converter.index = grid_forming ? 902 : 901;
    converter.name = grid_forming ? "在线保护 GFM" : "在线保护 GFL";
    converter.bus_ac = 2;
    converter.bus_dc = 0;
    converter.in_service = true;
    converter.ac_grid_forming = grid_forming;
    converter.control_mode = ConverterMode::PQ_MODE;
    converter.p_set_mw = 0.2;
    converter.p_schedule_mw = 0.2;
    converter.q_set_mvar = 0.0;
    converter.p_rated_mw = 10.0;
    converter.pmax_mw = 10.0;
    converter.pmin_mw = -10.0;
    converter.i_ac_max_pu = 1.0;
    converter.v_ac_set_pu = 1.0;
    converter.gfm_internal_voltage_set_pu = 1.0;
    converter.gfm_virtual_x_pu = 0.1;
    converter.dynamic_model.standard = "NERC";
    converter.dynamic_model.model_name = grid_forming
        ? "GridFormingNortonDroop" : "REGC_REEC_GFL_Subset";
    converter_system.vsc_converters = {converter};

    const auto converter_result =
        run_online_protection_dae(converter_system, input);
    INFO(converter_result.message);
    REQUIRE(converter_result.success);
    REQUIRE(converter_result.converter_dynamic_feedback_consumed);
    const std::string expected_type = grid_forming
        ? "VSCGridForming" : "VSCGridFollowing";
    bool consumed_real_output = false;
    for (const auto& snapshot : converter_result.discovery_run.snapshots) {
      const auto found = std::find_if(
          snapshot.device_outputs.begin(), snapshot.device_outputs.end(),
          [&](const auto& output) {
            return output.type == expected_type &&
                output.component_index == converter.index &&
                !output.values.empty();
          });
      consumed_real_output = consumed_real_output ||
          found != snapshot.device_outputs.end();
    }
    CHECK(consumed_real_output);
  }
}
