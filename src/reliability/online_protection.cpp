#include "hacdcpf/reliability/online_protection.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"
#include "hacdcpf/dynamics/DynamicSolver.hpp"

namespace hacdcpf::analysis {
namespace {

dynamics::DynamicEvent make_fault_event(const OnlineProtectionDAEInput& input) {
  dynamics::DynamicEvent fault;
  fault.time_s = input.fault_time_s;
  fault.type = dynamics::DynamicEventType::FaultShunt;
  fault.bus = input.fault_ac_bus_id;
  fault.phase = input.fault_phase;
  fault.component_type = "AC";
  fault.label = "online protection discovery fault";
  fault.params["r_pu"] = input.fault_r_pu;
  fault.params["x_pu"] = input.fault_x_pu;
  return fault;
}

bool has_converter_dynamic_output(const dynamics::DynamicResults& results) {
  for (const auto& snapshot : results.snapshots) {
    for (const auto& device : snapshot.device_outputs) {
      std::string type = device.type;
      std::transform(type.begin(), type.end(), type.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (type.find("gfl") != std::string::npos ||
          type.find("gfm") != std::string::npos ||
          type.find("gridfollowing") != std::string::npos ||
          type.find("gridforming") != std::string::npos ||
          type.find("vsc") != std::string::npos)
        return true;
    }
  }
  return false;
}

std::vector<PrimaryProtectionPhasorPoint> relay_primary_trajectory(
    const dynamics::DynamicResults& results, int bus_position,
    const OnlineProtectionDAEInput& input) {
  std::vector<PrimaryProtectionPhasorPoint> trajectory;
  trajectory.reserve(results.snapshots.size());
  const std::complex<double> impedance(input.fault_r_pu, input.fault_x_pu);
  const std::complex<double> admittance = 1.0 / impedance;
  for (const auto& snapshot : results.snapshots) {
    const int node = 3 * bus_position + input.fault_phase;
    if (node < 0 || node >= snapshot.vac_abc.size())
      throw std::runtime_error("DAE snapshot AC phasor dimension is inconsistent");
    const std::complex<double> voltage = snapshot.vac_abc[node];
    const std::complex<double> current =
        snapshot.time_s + 1e-12 >= input.fault_time_s
        ? admittance * voltage
        : std::complex<double>{0.0, 0.0};
    trajectory.push_back({snapshot.time_s, voltage, current});
  }
  return trajectory;
}

std::vector<DERTrajectoryPoint> der_trajectory(
    const dynamics::DynamicResults& results, int bus_position) {
  std::vector<DERTrajectoryPoint> trajectory;
  trajectory.reserve(results.snapshots.size());
  for (const auto& snapshot : results.snapshots) {
    const int node = 3 * bus_position;
    if (node < 0 || node >= snapshot.vac_abc.size())
      throw std::runtime_error("DER monitor AC phasor dimension is inconsistent");
    trajectory.push_back(
        {snapshot.time_s, std::abs(snapshot.vac_abc[node]),
         std::arg(snapshot.vac_abc[node])});
  }
  return trajectory;
}

std::vector<ProtectionFRTDERInput> make_der_inputs(
    const std::vector<OnlineDERFRTMonitor>& monitors,
    const std::vector<std::vector<DERTrajectoryPoint>>& trajectories) {
  if (monitors.size() != trajectories.size())
    throw std::runtime_error("online DER monitor trajectory dimension mismatch");
  std::vector<ProtectionFRTDERInput> result;
  result.reserve(monitors.size());
  for (size_t i = 0; i < monitors.size(); ++i) {
    ProtectionFRTDERInput item;
    item.stable_id = monitors[i].stable_id;
    item.settings = monitors[i].settings;
    item.momentary_cessation = monitors[i].momentary_cessation;
    item.trajectory = trajectories[i];
    item.loss_of_generation_shed_mw = monitors[i].loss_of_generation_shed_mw;
    result.push_back(std::move(item));
  }
  return result;
}

std::vector<int> deenergized_buses_after_branch_trip(
    const HybridPowerSystem& system, int tripped_branch_index) {
  std::vector<int> energized;
  auto mark_source = [&](int bus) {
    if (std::find(energized.begin(), energized.end(), bus) == energized.end())
      energized.push_back(bus);
  };
  for (const auto& grid : system.ac.external_grids)
    if (grid.in_service) mark_source(grid.bus);
  for (const auto& generator : system.ac.generators)
    if (generator.in_service) mark_source(generator.bus);
  for (const auto& storage : system.ac.storage)
    if (storage.in_service && storage.grid_forming) mark_source(storage.bus);
  for (const auto& converter : system.vsc_converters)
    if (converter.in_service && converter.ac_grid_forming)
      mark_source(converter.bus_ac);
  if (energized.empty())
    throw std::invalid_argument(
        "在线保护拓扑在支路开断后找不到交流电压源");

  for (size_t cursor = 0; cursor < energized.size(); ++cursor) {
    const int bus = energized[cursor];
    for (const auto& branch : system.ac.branches) {
      if (!branch.in_service || branch.index == tripped_branch_index) continue;
      int next = 0;
      if (branch.from_bus == bus) next = branch.to_bus;
      else if (branch.to_bus == bus) next = branch.from_bus;
      if (next != 0 &&
          std::find(energized.begin(), energized.end(), next) == energized.end())
        energized.push_back(next);
    }
  }

  std::vector<int> deenergized;
  for (const auto& bus : system.ac.buses) {
    if (!bus.in_service) continue;
    if (std::find(energized.begin(), energized.end(), bus.index) == energized.end())
      deenergized.push_back(bus.index);
  }
  std::sort(deenergized.begin(), deenergized.end());
  return deenergized;
}

}  // namespace

OnlineProtectionDAEResult run_online_protection_dae(
    const HybridPowerSystem& system,
    const OnlineProtectionDAEInput& input) {
  if (input.fault_ac_bus_id == 0 || input.protected_ac_branch_index == 0 ||
      input.fault_phase < 0 || input.fault_phase > 2 ||
      !std::isfinite(input.fault_time_s) || input.fault_time_s < 0.0 ||
      !std::isfinite(input.fault_r_pu) || input.fault_r_pu < 0.0 ||
      !std::isfinite(input.fault_x_pu) ||
      std::hypot(input.fault_r_pu, input.fault_x_pu) <= 0.0)
    throw std::invalid_argument("online protection fault specification is invalid");
  const double breaker_delays[] = {
      input.breaker.channel_delay_s, input.breaker.trip_coil_delay_s,
      input.breaker.mechanical_delay_s, input.breaker.arc_delay_s};
  for (double delay_s : breaker_delays) {
    if (!std::isfinite(delay_s) || delay_s < 0.0)
      throw std::invalid_argument(
          "在线保护断路器动作链延时必须为有限非负数");
  }
  const auto branch = std::find_if(
      system.ac.branches.begin(), system.ac.branches.end(),
      [&](const auto& item) {
        return item.index == input.protected_ac_branch_index && item.in_service;
      });
  if (branch == system.ac.branches.end())
    throw std::invalid_argument(
        "protected AC branch stable index is absent or out of service");

  auto options = input.dynamic_options;
  options.solver_type = dynamics::DynamicSolverType::MassMatrixDae;
  options.enable_der_protection = true;
  options.record_every_step = true;
  options.record_device_outputs = true;
  if (!std::isfinite(options.t_end_s) || !std::isfinite(options.dt_s) ||
      options.t_end_s <= input.fault_time_s || options.dt_s <= 0.0)
    throw std::invalid_argument(
        "dynamic horizon must extend beyond the protection fault time");

  dynamics::DynamicModelBuilder builder;
  dynamics::DynamicSolver solver;
  auto discovery_system = builder.build(system, options);
  const int relay_bus_position =
      discovery_system.network.acBusPosition(input.fault_ac_bus_id);
  if (relay_bus_position < 0)
    throw std::invalid_argument("fault AC bus is absent after dynamic projection");
  discovery_system.events.push_back(make_fault_event(input));

  OnlineProtectionDAEResult result;
  result.discovery_run = solver.solve(discovery_system);
  if (!result.discovery_run.success) {
    result.message = "故障发现阶段 DAE 失败: " + result.discovery_run.message;
    return result;
  }
  const auto primary = relay_primary_trajectory(
      result.discovery_run, relay_bus_position, input);
  auto measured = simulate_instrument_transformers(
      primary, input.instrument_transformers);
  for (auto& point : measured)
    point.directional_current = point.current;
  result.measured_relay_trajectory = measured;
  result.relay = evaluate_protection_relay(measured, input.relay);
  result.online_network_dae_coupled = true;
  result.relay_trajectory_from_dae = true;
  result.converter_dynamic_feedback_consumed =
      has_converter_dynamic_output(result.discovery_run);
  for (const auto& monitor : input.der_monitors) {
    const int bus_position =
        discovery_system.network.acBusPosition(monitor.ac_bus_id);
    if (bus_position < 0)
      throw std::invalid_argument("DER monitor AC bus is absent");
    result.discovery_der_trajectories.push_back(
        der_trajectory(result.discovery_run, bus_position));
  }
  if (!result.relay.operated) {
    result.der_stable_ids.reserve(input.der_monitors.size());
    for (size_t i = 0; i < input.der_monitors.size(); ++i) {
      result.der_stable_ids.push_back(input.der_monitors[i].stable_id);
      result.der_results.push_back(classify_der_frt_trajectory(
          input.der_monitors[i].settings,
          input.der_monitors[i].momentary_cessation,
          result.discovery_der_trajectories[i], options.t_end_s));
    }
    result.der_frt_state_machine_consumed = !input.der_monitors.empty();
    result.success = true;
    result.message = "DAE 轨迹已评估，继电器未达到动作条件";
    return result;
  }
  // IEEE C37.112 causal clearing chain; derivation and unit contract are in
  // the Chinese reliability manual's online-protection chapter.
  result.protection_clear_time_s = result.relay.command_time_s +
      input.breaker.channel_delay_s + input.breaker.trip_coil_delay_s +
      input.breaker.mechanical_delay_s + input.breaker.arc_delay_s;
  if (!std::isfinite(result.protection_clear_time_s) ||
      result.protection_clear_time_s > options.t_end_s)
    throw std::runtime_error(
        "trajectory-derived protection clearing time exceeds dynamic horizon");

  auto closed_loop_system = builder.build(system, options);
  closed_loop_system.events.push_back(make_fault_event(input));
  dynamics::DynamicEvent trip;
  trip.time_s = result.protection_clear_time_s;
  trip.type = dynamics::DynamicEventType::ACBranchTrip;
  trip.component_index = input.protected_ac_branch_index;
  trip.component_type = "ACBranch";
  trip.label = "trajectory-derived protection branch trip";
  closed_loop_system.events.push_back(trip);
  dynamics::DynamicEvent clear;
  clear.time_s = result.protection_clear_time_s;
  clear.type = dynamics::DynamicEventType::ClearFault;
  clear.bus = input.fault_ac_bus_id;
  clear.component_type = "AC";
  clear.label = "trajectory-derived fault isolation";
  closed_loop_system.events.push_back(clear);
  result.deenergized_ac_bus_ids = deenergized_buses_after_branch_trip(
      system, input.protected_ac_branch_index);
  // Protection opens the island at the same causal instant as fault clearing.
  // Constant-power loads on a source-free island must be de-energized in the
  // DAE; their interruption energy is accounted by the annual consequence
  // windows rather than by an unsolvable algebraic load. See the Chinese
  // reliability manual, online DAE topology-feedback derivation.
  for (int bus_id : result.deenergized_ac_bus_ids) {
    dynamics::DynamicEvent shed;
    shed.time_s = result.protection_clear_time_s;
    shed.type = dynamics::DynamicEventType::ACLoadScale;
    shed.bus = bus_id;
    shed.value = 0.0;
    shed.params["scale"] = 0.0;
    shed.params["canonical_bus"] = static_cast<double>(bus_id);
    shed.component_type = "ACLoad";
    shed.label = "protection-deenergized island load shedding";
    closed_loop_system.events.push_back(std::move(shed));
  }
  result.closed_loop_run = solver.solve(closed_loop_system);
  if (!result.closed_loop_run.success) {
    result.message = "保护闭环 DAE 失败: " + result.closed_loop_run.message;
    return result;
  }

  bool trip_applied = false;
  bool clear_applied = false;
  for (const auto& record : result.closed_loop_run.applied_event_records) {
    trip_applied = trip_applied ||
        (record.type == "ACBranchTrip" &&
         record.component_index == input.protected_ac_branch_index);
    clear_applied = clear_applied ||
        (record.type == "ClearFault" && record.bus == input.fault_ac_bus_id);
  }
  result.protection_action_applied = trip_applied && clear_applied;
  result.dead_island_load_shedding_applied = true;
  for (int bus_id : result.deenergized_ac_bus_ids) {
    const bool bus_shed = std::any_of(
        result.closed_loop_run.applied_event_records.begin(),
        result.closed_loop_run.applied_event_records.end(),
        [&](const auto& record) {
          return record.type == "ACLoadScale" && record.bus == bus_id;
        });
    result.dead_island_load_shedding_applied =
        result.dead_island_load_shedding_applied && bus_shed;
  }
  if (!result.protection_action_applied)
    throw std::runtime_error(
        "closed-loop DAE did not apply both branch trip and fault clearing");

  for (const auto& monitor : input.der_monitors) {
    const int bus_position =
        closed_loop_system.network.acBusPosition(monitor.ac_bus_id);
    if (bus_position < 0)
      throw std::invalid_argument("DER monitor AC bus is absent");
    const auto trajectory =
        der_trajectory(result.closed_loop_run, bus_position);
    result.der_stable_ids.push_back(monitor.stable_id);
    result.closed_loop_der_trajectories.push_back(trajectory);
    result.der_results.push_back(classify_der_frt_trajectory(
        monitor.settings, monitor.momentary_cessation, trajectory,
        options.t_end_s));
  }
  result.der_frt_state_machine_consumed = !input.der_monitors.empty();
  result.converter_dynamic_feedback_consumed =
      result.converter_dynamic_feedback_consumed ||
      has_converter_dynamic_output(result.closed_loop_run);
  result.success = true;
  result.message = "在线 DAE、轨迹继电器、支路跳闸与 DER-FRT 闭环完成";
  return result;
}

OnlineProtectionCyberReliabilityResult
compare_online_protection_cyber_reliability(
    const HybridPowerSystem& system,
    const std::vector<OnlineProtectionCyberReliabilityScenario>& scenarios,
    double curtailment_threshold_mw) {
  if (scenarios.empty())
    throw std::invalid_argument(
        "online protection/cyber comparison requires at least one scenario");

  OnlineProtectionCyberReliabilityResult result;
  std::vector<ProtectionCyberReliabilityScenario> event_tree_scenarios;
  event_tree_scenarios.reserve(scenarios.size());
  result.diagnostics.reserve(scenarios.size());
  for (const auto& source : scenarios) {
    OnlineProtectionCyberScenarioDiagnostic diagnostic;
    diagnostic.scenario_id = source.scenario_id;
    try {
      diagnostic.primary = run_online_protection_dae(system, source.primary);
    } catch (const std::exception&) {
      throw;
    } catch (...) {
      throw std::runtime_error(
          "在线主保护 DAE 抛出非标准异常；动态系统含未满足运行契约的设备或状态");
    }
    try {
      diagnostic.backup = run_online_protection_dae(system, source.backup);
    } catch (const std::exception&) {
      throw;
    } catch (...) {
      throw std::runtime_error(
          "在线后备保护 DAE 抛出非标准异常；动态系统含未满足运行契约的设备或状态");
    }
    if (!diagnostic.primary.success || !diagnostic.backup.success)
      throw std::runtime_error(
          "在线主/后备 DAE 未产生可消费轨迹：主保护=" +
          diagnostic.primary.message + "；后备保护=" +
          diagnostic.backup.message);

    ProtectionCyberReliabilityScenario scenario;
    scenario.scenario_id = source.scenario_id;
    scenario.initiating_frequency_per_year =
        source.initiating_frequency_per_year;
    auto& event = scenario.event;
    event.primary.relay = source.primary.relay;
    event.primary.trajectory = diagnostic.primary.measured_relay_trajectory;
    event.primary.breaker = source.primary.breaker;
    event.primary.relay_success_probability =
        source.primary_relay_success_probability;
    event.primary.breaker_success_probability =
        source.primary_breaker_success_probability;
    event.backup.relay = source.backup.relay;
    event.backup.trajectory = diagnostic.backup.measured_relay_trajectory;
    event.backup.breaker = source.backup.breaker;
    event.backup.relay_success_probability =
        source.backup_relay_success_probability;
    event.backup.breaker_success_probability =
        source.backup_breaker_success_probability;
    event.coordination_margin_s = source.coordination_margin_s;
    event.uncleared_terminal_time_s = source.uncleared_terminal_time_s;
    event.information_components = source.information_components;
    event.information_functions = source.information_functions;
    event.information_environments = source.information_environments;
    event.function_bindings = source.function_bindings;

    const auto& primary_actual = diagnostic.primary.relay.operated
        ? diagnostic.primary.closed_loop_der_trajectories
        : diagnostic.primary.discovery_der_trajectories;
    const auto& backup_actual = diagnostic.backup.relay.operated
        ? diagnostic.backup.closed_loop_der_trajectories
        : diagnostic.backup.discovery_der_trajectories;
    event.ders = make_der_inputs(source.primary.der_monitors, primary_actual);
    event.backup_ders =
        make_der_inputs(source.backup.der_monitors, backup_actual);
    event.uncleared_ders = make_der_inputs(
        source.primary.der_monitors,
        diagnostic.primary.discovery_der_trajectories);
    if (event.ders.size() != event.backup_ders.size())
      throw std::invalid_argument(
          "online primary and backup DER monitor sets must have equal dimensions");
    scenario.consequence = source.consequence;

    ProtectionCyberClassGenerationResult generated;
    try {
      generated = generate_protection_cyber_classes(event);
    } catch (const std::exception&) {
      throw;
    } catch (...) {
      throw std::runtime_error(
          "在线 DAE 轨迹转换为保护信息事件类时抛出非标准异常");
    }
    diagnostic.primary_event_tree_clear_time_s =
        generated.coordination.clearing_times_s.at(0);
    diagnostic.backup_event_tree_clear_time_s =
        generated.coordination.clearing_times_s.at(1);
    const double tolerance = std::max(
        source.primary.dynamic_options.dt_s,
        source.backup.dynamic_options.dt_s) + 1e-9;
    if (diagnostic.primary.relay.operated &&
        std::abs(diagnostic.primary_event_tree_clear_time_s -
                 diagnostic.primary.protection_clear_time_s) > tolerance)
      throw std::runtime_error(
          "primary DAE and event-tree clearing times are inconsistent");
    if (diagnostic.backup.relay.operated &&
        std::abs(diagnostic.backup_event_tree_clear_time_s -
                 diagnostic.backup.protection_clear_time_s) > tolerance)
      throw std::runtime_error(
          "backup DAE and event-tree clearing times are inconsistent");

    event_tree_scenarios.push_back(std::move(scenario));
    result.diagnostics.push_back(std::move(diagnostic));
  }

  try {
    result.comparison = compare_protection_cyber_reliability(
        event_tree_scenarios, curtailment_threshold_mw);
  } catch (const std::exception&) {
    throw;
  } catch (...) {
    throw std::runtime_error(
        "在线保护信息事件类年度聚合时抛出非标准异常");
  }
  result.comparison.validity.online_network_dae_coupled = true;
  result.comparison.model_scope = result.model_scope;
  result.dae_trajectories_consumed_by_event_tree = true;
  return result;
}

}  // namespace hacdcpf::analysis
