#include <cmath>
#include <numeric>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/sioux_falls_ieee123_case.hpp"

using Catch::Approx;

TEST_CASE("Sioux Falls--IEEE 123 case preserves public networks and assumptions",
          "[ev_power_traffic][sioux_falls][ieee123][opendss]") {
  hacdcpf::io::SiouxFallsIEEE123Options options;
  options.external_data_root =
      std::string(HACDCPF_PROJECT_ROOT) + "/external_data";
  const auto data = hacdcpf::io::build_sioux_falls_ieee123_case(options);

  CHECK(data.problem.traffic.nodes.size() == 24);
  CHECK(data.problem.traffic.links.size() == 76);
  CHECK(data.problem.system.ac.buses.size() >= 120);
  CHECK(data.problem.system.ac.branches.size() >= 115);
  CHECK(data.station_mapping.size() == 8);
  CHECK(data.selected_od.size() == 12);
  CHECK(data.problem.demands.size() == 48);
  CHECK(data.problem.icv_demands.size() == 48);
  CHECK(data.problem.routes.size() == 72);
  CHECK(data.problem.system.ac.generators.size() == 5);
  REQUIRE(data.problem.system.three_phase_ac.has_value());
  CHECK(data.problem.system.three_phase_ac->buses.size() >= 120);
  CHECK(data.problem.system.dc.buses.size() == 4);
  CHECK(data.problem.system.dc.branches.size() == 3);
  CHECK(data.problem.system.vsc_converters.size() == 4);
  CHECK(data.phase_hybrid_model.opf.y_ac.rows() >= 250);
  CHECK(data.phase_hybrid_model.opf.g_dc.rows() == 4);
  CHECK(data.phase_hybrid_model.opf.converters.size() == 4);
  CHECK(data.phase_hybrid_model.vsc_component_indices.size() == 4);
  CHECK(data.phase_hybrid_model.opf.generators.size() >= 15);
  CHECK(data.phase_hybrid_model.generator_bindings.size() ==
        data.phase_hybrid_model.opf.generators.size());
  CHECK(data.phase_hybrid_model.ac_bus_ids.size() ==
        data.problem.system.three_phase_ac->buses.size());
  CHECK(data.base_load_multiplier.size() == 24);
  CHECK(data.power_model_scope.find("three-phase") != std::string::npos);
  CHECK(data.screening_power_model_scope.find("balanced") != std::string::npos);
  for (const auto& station : data.station_mapping) {
    CHECK(std::accumulate(station.phase_power_share.begin(),
                          station.phase_power_share.end(), 0.0) ==
          Approx(1.0));
    for (int phase = 0; phase < 3; ++phase) {
      if (station.phase_power_share[static_cast<std::size_t>(phase)] > 0.0) {
        CHECK(data.phase_hybrid_model.phase_node(station.three_phase_bus_index,
                                                  phase) >= 0);
      }
    }
  }

  double dynamic_vehicles = 0.0;
  for (const auto& demand : data.problem.demands) {
    dynamic_vehicles += demand.vehicles;
    REQUIRE(demand.candidate_route_indices.size() == 3);
  }
  for (const auto& demand : data.problem.icv_demands) {
    dynamic_vehicles += demand.vehicles;
    REQUIRE(demand.candidate_route_indices.size() == 3);
  }
  double selected_tntp_flow = 0.0;
  for (const auto& od : data.selected_od) selected_tntp_flow += od.tntp_flow;
  CHECK(dynamic_vehicles == Approx(selected_tntp_flow * options.od_demand_scale));

  hacdcpf::evpt::CTMForwardAssignment assignment;
  for (const auto& demand : data.problem.demands) {
    const double share = demand.vehicles /
                         static_cast<double>(demand.candidate_route_indices.size());
    for (int route : demand.candidate_route_indices) {
      assignment.ev_route_flow[demand.index][route] = share;
    }
  }
  for (const auto& demand : data.problem.icv_demands) {
    const double share = demand.vehicles /
                         static_cast<double>(demand.candidate_route_indices.size());
    for (int route : demand.candidate_route_indices) {
      assignment.icv_route_flow[demand.index][route] = share;
    }
  }
  hacdcpf::evpt::EVPowerTrafficOptions ev_options;
  ev_options.num_steps = options.num_steps;
  ev_options.time_step_hr = options.time_step_hr;
  hacdcpf::evpt::CTMOptions ctm_options;
  ctm_options.route_specific_cells = true;
  ctm_options.record_cell_history = false;
  ctm_options.dt_ctm_hr = options.time_step_hr /
      static_cast<double>(options.ctm_substeps_per_power_step);
  const auto forward = hacdcpf::evpt::simulate_ev_power_traffic_ctm_forward(
      data.problem, assignment, ev_options, ctm_options);
  REQUIRE(forward.valid);
  CHECK(forward.requested_vehicles == Approx(dynamic_vehicles));
  CHECK(forward.assigned_vehicles == Approx(dynamic_vehicles));
  CHECK(forward.unassigned_vehicles == Approx(0.0).margin(1e-8));
  CHECK(forward.simulation.admitted_vehicles ==
        Approx(forward.admitted_vehicles).margin(1e-8));
  CHECK(forward.simulation.step_link_results.size() ==
        static_cast<std::size_t>(options.num_steps *
                                 forward.simulation.steps_per_sim_step));
  CHECK(std::abs(forward.simulation.vehicle_conservation_error_vehicles) <
        1e-6);

  const auto recourse =
      hacdcpf::io::build_sioux_falls_ieee123_recourse(data, forward);
  CHECK(recourse.opf_cases.size() ==
        static_cast<std::size_t>(options.num_steps));
  CHECK(recourse.station_grid_load_kw.size() == data.station_mapping.size());
  CHECK(recourse.station_backlog_kwh.size() == data.station_mapping.size());
  REQUIRE(recourse.aggregate_backlog_kwh.size() ==
          static_cast<std::size_t>(options.num_steps));
  CHECK(recourse.total_arrivals_vehicles > 0.0);
  double admitted_ev_vehicles = 0.0;
  for (const auto& route : data.problem.routes) {
    if (route.charging_stops.empty()) continue;
    const auto it = forward.simulation.route_admitted_departures.find(route.index);
    if (it != forward.simulation.route_admitted_departures.end()) {
      admitted_ev_vehicles +=
          std::accumulate(it->second.begin(), it->second.end(), 0.0);
    }
  }
  CHECK(recourse.total_arrivals_vehicles <=
        Approx(admitted_ev_vehicles).margin(1e-7));
  double terminal_ev_vehicles = 0.0;
  for (const auto& route : data.problem.routes) {
    if (route.charging_stops.empty()) continue;
    const auto it =
        forward.simulation.route_terminal_arrivals.find(route.index);
    if (it != forward.simulation.route_terminal_arrivals.end()) {
      terminal_ev_vehicles +=
          std::accumulate(it->second.begin(), it->second.end(), 0.0);
    }
  }
  CHECK(terminal_ev_vehicles ==
        Approx(admitted_ev_vehicles).margin(1e-6));
  CHECK(recourse.total_arrivals_vehicles ==
        Approx(terminal_ev_vehicles).margin(1e-7));
  CHECK(recourse.requested_battery_energy_kwh ==
        Approx(recourse.delivered_battery_energy_kwh +
               recourse.charging_window_shortfall_kwh)
            .margin(1e-6));
  CHECK(recourse.delivered_battery_energy_kwh > 0.0);
  CHECK(recourse.maximum_station_backlog_kwh >= 0.0);
  CHECK(recourse.backlog_time_integral_kwh_hr >= 0.0);
  CHECK(recourse.station_saturated_intervals >= 0);
  CHECK(recourse.station_saturated_intervals <=
        options.num_steps * static_cast<int>(data.station_mapping.size()));
  double recomputed_maximum_backlog_kwh = 0.0;
  double recomputed_backlog_integral_kwh_hr = 0.0;
  for (const auto& mapping : data.station_mapping) {
    const auto& backlog = recourse.station_backlog_kwh.at(mapping.station_id);
    REQUIRE(backlog.size() == static_cast<std::size_t>(options.num_steps));
    for (int step = 0; step < options.num_steps; ++step) {
      const double value = backlog[static_cast<std::size_t>(step)];
      CHECK(value >= 0.0);
      recomputed_maximum_backlog_kwh =
          std::max(recomputed_maximum_backlog_kwh, value);
      recomputed_backlog_integral_kwh_hr += value * options.time_step_hr;
    }
  }
  CHECK(recourse.maximum_station_backlog_kwh ==
        Approx(recomputed_maximum_backlog_kwh));
  CHECK(recourse.backlog_time_integral_kwh_hr ==
        Approx(recomputed_backlog_integral_kwh_hr));
  CHECK(recourse.source_entry_shortfall_vehicles ==
        Approx(forward.source_entry_shortfall_vehicles));
  double peak_ev_mw = 0.0;
  for (int step = 0; step < options.num_steps; ++step) {
    double total_kw = 0.0;
    for (const auto& [station, profile] : recourse.station_grid_load_kw) {
      (void)station;
      total_kw += profile[static_cast<std::size_t>(step)];
    }
    peak_ev_mw = std::max(peak_ev_mw, total_kw / 1000.0);
  }
  CHECK(peak_ev_mw > 0.0);
  CHECK(peak_ev_mw <=
        Approx(options.station_power_kw * data.station_mapping.size() / 1000.0));

  auto causal_forward = forward;
  causal_forward.simulation.station_arrivals.clear();
  const int first_station = data.station_mapping.front().station_id;
  auto& substep_arrivals =
      causal_forward.simulation.station_arrivals[first_station];
  substep_arrivals.assign(
      static_cast<std::size_t>(options.num_steps *
                               forward.simulation.steps_per_sim_step),
      0.0);
  substep_arrivals.front() = 1.0;
  const auto causal_recourse =
      hacdcpf::io::build_sioux_falls_ieee123_recourse(data, causal_forward);
  REQUIRE(causal_recourse.station_arrivals_vehicles.at(first_station).size() ==
          static_cast<std::size_t>(options.num_steps));
  CHECK(causal_recourse.station_arrivals_vehicles.at(first_station)[0] ==
        Approx(0.0));
  CHECK(causal_recourse.station_arrivals_vehicles.at(first_station)[1] ==
        Approx(1.0));

  hacdcpf::io::SiouxFallsIEEE123RecourseOptions monotone_options;
  monotone_options.arrival_opens_until_horizon = true;
  auto early_forward = forward;
  early_forward.simulation.station_arrivals.clear();
  early_forward.simulation.station_arrivals[first_station].assign(
      static_cast<std::size_t>(options.num_steps *
                               forward.simulation.steps_per_sim_step),
      0.0);
  early_forward.simulation.station_arrivals[first_station][0] = 100.0;
  auto late_forward = early_forward;
  auto& late_arrivals =
      late_forward.simulation.station_arrivals[first_station];
  late_arrivals[0] = 0.0;
  late_arrivals[static_cast<std::size_t>(
      (options.num_steps - 2) * forward.simulation.steps_per_sim_step)] =
      100.0;
  const auto early_recourse =
      hacdcpf::io::build_sioux_falls_ieee123_recourse(
          data, early_forward, monotone_options);
  const auto late_recourse =
      hacdcpf::io::build_sioux_falls_ieee123_recourse(
          data, late_forward, monotone_options);
  CHECK(early_recourse.delivered_battery_energy_kwh >=
        late_recourse.delivered_battery_energy_kwh - 1e-9);
  CHECK(early_recourse.charging_window_shortfall_kwh <=
        late_recourse.charging_window_shortfall_kwh + 1e-9);
  CHECK(early_recourse.scheduling_scope.find("arrival-monotone") !=
        std::string::npos);
}

TEST_CASE("Sioux Falls--IEEE 123 stress controls preserve charging injection",
          "[ev_power_traffic][sioux_falls][ieee123][stress]") {
  hacdcpf::io::SiouxFallsIEEE123Options options;
  options.external_data_root =
      std::string(HACDCPF_PROJECT_ROOT) + "/external_data";
  options.base_load_scale = 1.20;
  options.vuf_limit = 0.015;
  options.converter_capacity_scale = 0.50;
  options.balanced_station_phase_allocation = true;
  const auto data = hacdcpf::io::build_sioux_falls_ieee123_case(options);

  CHECK(data.phase_hybrid_model.opf.vuf_max == Approx(options.vuf_limit));
  CHECK(data.phase_hybrid_model.opf.v_min_pu.minCoeff() >=
        options.phase_voltage_min_pu);
  CHECK(data.phase_hybrid_model.opf.v_max_pu.maxCoeff() <=
        options.phase_voltage_max_pu);
  REQUIRE(data.problem.system.vsc_converters.size() == 4);
  REQUIRE(data.phase_hybrid_model.opf.converters.size() == 4);
  for (const auto& converter : data.problem.system.vsc_converters) {
    CHECK(converter.pmax_mw == Approx(options.converter_capacity_scale));
    CHECK(converter.pmin_mw == Approx(-options.converter_capacity_scale));
    CHECK(converter.p_rated_mw == Approx(options.converter_capacity_scale));
    CHECK(converter.i_ac_max_pu ==
          Approx(1.2 * options.converter_capacity_scale));
  }
  for (const auto& converter : data.phase_hybrid_model.opf.converters) {
    CHECK(converter.s_max_pu ==
          Approx(options.converter_capacity_scale /
                 data.phase_hybrid_model.opf.base_mva));
    CHECK(converter.phase_current_max_pu ==
          Approx(1.2 * options.converter_capacity_scale));
  }

  hacdcpf::evpt::CTMForwardAssignment assignment;
  for (const auto& demand : data.problem.demands) {
    const double share = demand.vehicles /
                         static_cast<double>(demand.candidate_route_indices.size());
    for (int route : demand.candidate_route_indices) {
      assignment.ev_route_flow[demand.index][route] = share;
    }
  }
  for (const auto& demand : data.problem.icv_demands) {
    const double share = demand.vehicles /
                         static_cast<double>(demand.candidate_route_indices.size());
    for (int route : demand.candidate_route_indices) {
      assignment.icv_route_flow[demand.index][route] = share;
    }
  }
  hacdcpf::evpt::EVPowerTrafficOptions ev_options;
  ev_options.num_steps = options.num_steps;
  ev_options.time_step_hr = options.time_step_hr;
  hacdcpf::evpt::CTMOptions ctm_options;
  ctm_options.route_specific_cells = true;
  ctm_options.record_cell_history = false;
  ctm_options.dt_ctm_hr = options.time_step_hr /
      static_cast<double>(options.ctm_substeps_per_power_step);
  const auto forward = hacdcpf::evpt::simulate_ev_power_traffic_ctm_forward(
      data.problem, assignment, ev_options, ctm_options);
  REQUIRE(forward.valid);
  const auto recourse =
      hacdcpf::io::build_sioux_falls_ieee123_recourse(data, forward);

  for (int step = 0; step < options.num_steps; ++step) {
    const auto& problem = recourse.opf_cases[static_cast<std::size_t>(step)];
    const double multiplier = options.base_load_scale *
        data.base_load_multiplier[static_cast<std::size_t>(step)];
    const Eigen::VectorXd ev_increment =
        problem.p_load_pu - data.phase_hybrid_model.opf.p_load_pu * multiplier;
    double station_total_mw = 0.0;
    for (const auto& mapping : data.station_mapping) {
      const double station_mw =
          recourse.station_grid_load_kw.at(mapping.station_id)
              [static_cast<std::size_t>(step)] /
          1000.0;
      station_total_mw += station_mw;
      int available_phases = 0;
      for (int phase = 0; phase < 3; ++phase) {
        available_phases += data.phase_hybrid_model.phase_node(
                                mapping.three_phase_bus_index, phase) >= 0
            ? 1
            : 0;
      }
      REQUIRE(available_phases > 0);
      for (int phase = 0; phase < 3; ++phase) {
        const int node = data.phase_hybrid_model.phase_node(
            mapping.three_phase_bus_index, phase);
        if (node < 0) continue;
        CHECK(ev_increment[node] ==
              Approx(station_mw /
                     (available_phases * problem.base_mva))
                  .margin(1e-10));
      }
    }
    CHECK(ev_increment.sum() * problem.base_mva ==
          Approx(station_total_mw).margin(1e-9));
  }
}

TEST_CASE("Sioux Falls--IEEE 123 repeats its daily load profile for long horizons",
          "[ev_power_traffic][sioux_falls][ieee123]") {
  hacdcpf::io::SiouxFallsIEEE123Options options;
  options.external_data_root =
      std::string(HACDCPF_PROJECT_ROOT) + "/external_data";
  options.num_steps = 48;
  const auto data = hacdcpf::io::build_sioux_falls_ieee123_case(options);

  REQUIRE(data.base_load_multiplier.size() == 48);
  for (int step = 0; step < 24; ++step) {
    CHECK(data.base_load_multiplier[static_cast<std::size_t>(step)] ==
          Approx(data.base_load_multiplier[static_cast<std::size_t>(step + 24)]));
  }
}
