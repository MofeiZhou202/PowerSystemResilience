#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/ev_power_traffic/simulation.hpp"
#include "hacdcpf/optimal_power_flow/three_phase_hybrid_adapter.hpp"

namespace hacdcpf::io {

struct SiouxFallsIEEE123Options {
  std::filesystem::path external_data_root{"external_data"};
  int selected_od_pairs{12};
  int paths_per_od{3};
  int num_steps{24};
  double time_step_hr{0.25};
  int ctm_substeps_per_power_step{60};
  std::vector<int> departure_steps{4, 5, 6, 7};
  std::vector<double> departure_shares{0.15, 0.35, 0.35, 0.15};
  double od_demand_scale{0.10};
  double ev_penetration{0.20};
  double road_capacity_scale{0.10};
  double charging_energy_kwh_per_vehicle{18.0};
  double charging_power_kw_per_vehicle{50.0};
  int charging_dwell_steps{4};
  double station_power_kw{600.0};
  double base_load_scale{1.0};
  double phase_voltage_min_pu{0.95};
  double phase_voltage_max_pu{1.05};
  double vuf_limit{0.03};
  double converter_capacity_scale{1.0};
  bool balanced_station_phase_allocation{false};
};

struct SiouxFallsIEEE123StationMap {
  int station_id{0};
  int traffic_node{0};
  std::string opendss_bus_name;
  int power_bus_index{0};
  int three_phase_bus_index{0};
  std::array<double, 3> phase_power_share{1.0 / 3.0, 1.0 / 3.0,
                                          1.0 / 3.0};
};

struct SiouxFallsIEEE123ODPair {
  int origin_node{0};
  int destination_node{0};
  double tntp_flow{0.0};
  std::vector<int> ev_route_indices;
  std::vector<int> icv_route_indices;
};

struct SiouxFallsIEEE123Case {
  evpt::EVPowerTrafficProblem problem;
  opf::phase_hybrid::ThreePhaseHybridModel phase_hybrid_model;
  SiouxFallsIEEE123Options options;
  std::vector<SiouxFallsIEEE123StationMap> station_mapping;
  std::vector<SiouxFallsIEEE123ODPair> selected_od;
  std::vector<double> base_load_multiplier;
  std::vector<std::string> power_import_warnings;
  std::vector<std::string> power_import_skipped;
  std::string traffic_source;
  std::string power_source;
  std::string power_model_scope;
  std::string screening_power_model_scope;
};

struct SiouxFallsIEEE123RecourseOptions {
  double charging_efficiency{0.95};
  double charging_power_factor{0.98};
  bool scale_base_load_profile{true};
  /// When true, arrival opens service availability through the common study
  /// horizon. Earlier arrivals can wait and retain all actions available to a
  /// later cohort, matching arrival-monotone emergency-recovery recourse.
  bool arrival_opens_until_horizon{false};
};

struct SiouxFallsIEEE123Recourse {
  std::vector<opf::phase_hybrid::ThreePhaseHybridOPFCase> opf_cases;
  std::unordered_map<int, std::vector<double>> station_arrivals_vehicles;
  std::unordered_map<int, std::vector<double>> station_grid_load_kw;
  std::unordered_map<int, std::vector<double>> station_backlog_kwh;
  std::vector<double> aggregate_backlog_kwh;
  double total_arrivals_vehicles{0.0};
  double requested_battery_energy_kwh{0.0};
  double delivered_battery_energy_kwh{0.0};
  double charging_window_shortfall_kwh{0.0};
  double maximum_station_backlog_kwh{0.0};
  double backlog_time_integral_kwh_hr{0.0};
  int station_saturated_intervals{0};
  double source_entry_shortfall_vehicles{0.0};
  std::string scheduling_scope;
};

/// Build the reproducible Sioux Falls--IEEE 123 coupled operating case.
/// Public benchmark data are imported without numerical edits; all dynamic
/// demand, penetration, capacity, generator, and coupling assumptions are
/// recorded in the returned options and metadata.
SiouxFallsIEEE123Case build_sioux_falls_ieee123_case(
    const SiouxFallsIEEE123Options& options = {});

/// Convert CTM station arrivals into an earliest-deadline-first charging
/// schedule and inject its phase-specific P/Q demand into a fixed-topology
/// sequence of monolithic three-phase AC/DC hybrid OPF cases.
SiouxFallsIEEE123Recourse build_sioux_falls_ieee123_recourse(
    const SiouxFallsIEEE123Case& data,
    const evpt::CTMForwardResult& forward,
    const SiouxFallsIEEE123RecourseOptions& options = {});

}  // namespace hacdcpf::io
