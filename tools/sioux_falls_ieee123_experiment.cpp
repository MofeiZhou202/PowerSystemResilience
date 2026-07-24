#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "Highs.h"
#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/typhoon_resilience.hpp"
#include "hacdcpf/analysis/typhoon_traffic_impact.hpp"
#include "hacdcpf/io/sioux_falls_ieee123_case.hpp"
#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"
#include "hacdcpf/optimal_power_flow/three_phase_hybrid_relaxation.hpp"
#include "hacdcpf/power_flow/three_phase_hybrid.hpp"

namespace {

using nlohmann::json;

hacdcpf::evpt::CTMForwardAssignment equal_route_assignment(
    const hacdcpf::evpt::EVPowerTrafficProblem& problem) {
  hacdcpf::evpt::CTMForwardAssignment assignment;
  for (const auto& demand : problem.demands) {
    if (demand.candidate_route_indices.empty()) continue;
    const double share = demand.vehicles /
                         static_cast<double>(demand.candidate_route_indices.size());
    for (int route : demand.candidate_route_indices) {
      assignment.ev_route_flow[demand.index][route] = share;
    }
  }
  for (const auto& demand : problem.icv_demands) {
    if (demand.candidate_route_indices.empty()) continue;
    const double share = demand.vehicles /
                         static_cast<double>(demand.candidate_route_indices.size());
    for (int route : demand.candidate_route_indices) {
      assignment.icv_route_flow[demand.index][route] = share;
    }
  }
  return assignment;
}

double mean_arrival_step(
    const std::unordered_map<int, std::vector<double>>& station_arrivals) {
  double weighted_steps = 0.0;
  double vehicles = 0.0;
  for (const auto& [station, profile] : station_arrivals) {
    (void)station;
    for (std::size_t step = 0; step < profile.size(); ++step) {
      weighted_steps += static_cast<double>(step) * profile[step];
      vehicles += profile[step];
    }
  }
  return vehicles > 0.0 ? weighted_steps / vehicles : 0.0;
}

json opf_summary(
    const hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFResult& result,
    const hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFCase& problem) {
  double minimum_voltage_pu = std::numeric_limits<double>::infinity();
  double maximum_voltage_pu = 0.0;
  double minimum_lower_voltage_margin_pu =
      std::numeric_limits<double>::infinity();
  double minimum_upper_voltage_margin_pu =
      std::numeric_limits<double>::infinity();
  for (Eigen::Index i = 0; i < result.full_voltage.size(); ++i) {
    const auto voltage = result.full_voltage[i];
    const double magnitude = std::abs(voltage);
    minimum_voltage_pu = std::min(minimum_voltage_pu, magnitude);
    maximum_voltage_pu = std::max(maximum_voltage_pu, magnitude);
    if (i < problem.v_min_pu.size()) {
      minimum_lower_voltage_margin_pu = std::min(
          minimum_lower_voltage_margin_pu, magnitude - problem.v_min_pu[i]);
    }
    if (i < problem.v_max_pu.size()) {
      minimum_upper_voltage_margin_pu = std::min(
          minimum_upper_voltage_margin_pu, problem.v_max_pu[i] - magnitude);
    }
  }
  if (!std::isfinite(minimum_voltage_pu)) minimum_voltage_pu = 0.0;
  if (!std::isfinite(minimum_lower_voltage_margin_pu)) {
    minimum_lower_voltage_margin_pu = 0.0;
  }
  if (!std::isfinite(minimum_upper_voltage_margin_pu)) {
    minimum_upper_voltage_margin_pu = 0.0;
  }
  return {{"converged", result.converged},
          {"status", result.status},
          {"solver", result.solver},
          {"objective", result.objective},
          {"runtime_ms", result.runtime_ms},
          {"iterations", result.iterations},
          {"variables", result.variables},
          {"equalities", result.equalities},
          {"inequalities", result.inequalities},
          {"eliminated_phase_nodes", result.eliminated_phase_nodes},
          {"primal_residual", result.primal_residual},
          {"dual_residual", result.dual_residual},
          {"complementarity", result.complementarity},
          {"max_voltage_violation", result.max_voltage_violation},
          {"minimum_voltage_pu", minimum_voltage_pu},
          {"maximum_voltage_pu", maximum_voltage_pu},
          {"minimum_lower_voltage_margin_pu",
           minimum_lower_voltage_margin_pu},
          {"minimum_upper_voltage_margin_pu",
           minimum_upper_voltage_margin_pu},
          {"max_vuf", result.max_vuf},
          {"vuf_margin", problem.vuf_max - result.max_vuf},
          {"vuf_utilization",
           result.max_vuf / std::max(1e-12, problem.vuf_max)},
          {"max_converter_current_loading",
           result.max_converter_current_loading},
          {"max_converter_violation", result.max_converter_violation}};
}

json relaxation_summary(
    const hacdcpf::opf::phase_hybrid::ThreePhaseHybridRelaxationResult& result,
    double feasible_objective) {
  const double absolute_gap = result.dual_certificate_available
      ? std::max(0.0, feasible_objective - result.objective_lower_bound)
      : std::numeric_limits<double>::infinity();
  return {{"solved", result.solved},
          {"outer_relaxation_valid", result.outer_relaxation_valid},
          {"dual_certificate_available", result.dual_certificate_available},
          {"soc_outer_approximation_converged",
           result.soc_outer_approximation_converged},
          {"ac_passivity_cut_applied", result.ac_passivity_cut_applied},
          {"dc_passivity_cut_applied", result.dc_passivity_cut_applied},
          {"status", result.status},
          {"solver", result.solver},
          {"objective_lower_bound", result.objective_lower_bound},
          {"lp_primal_objective", result.lp_primal_objective},
          {"lp_primal_dual_gap", result.lp_primal_dual_gap},
          {"feasible_objective", feasible_objective},
          {"absolute_relaxation_gap", absolute_gap},
          {"relative_relaxation_gap",
           absolute_gap / std::max(1.0, std::abs(feasible_objective))},
          {"runtime_ms", result.runtime_ms},
          {"rounds", result.rounds},
          {"cuts_added", result.cuts_added},
          {"variables", result.variables},
          {"equalities", result.equalities},
          {"inequalities", result.inequalities},
          {"max_soc_violation", result.max_soc_violation},
          {"max_ac_lift_violation", result.max_ac_lift_violation},
          {"max_ac_psd_violation", result.max_ac_psd_violation},
          {"max_dc_lift_violation", result.max_dc_lift_violation},
          {"max_converter_apparent_power_violation",
           result.max_converter_apparent_power_violation},
          {"max_converter_current_violation",
           result.max_converter_current_violation},
          {"round_lower_bounds", result.round_lower_bounds},
          {"model_limitations", result.model_limitations}};
}

struct ExperimentOptions {
  bool show_help{false};
  bool solve_sequence{false};
  bool compact_sequence{false};
  bool batch{false};
  bool traffic_only{false};
  bool grid_admissibility{false};
  bool typhoon_road_impact{false};
  bool route_search{false};
  bool search_exhaustive{false};
  unsigned int typhoon_seed{2030};
  double typhoon_delta_p_hpa{70.0};
  double road_drainage_mm_hr{10.0};
  double road_flood_closure_mm{300.0};
  int search_scenarios{4};
  int search_threads{1};
  int search_max_nodes{5000};
  double search_relative_gap{0.0};
  double search_recovery_voll_per_mwh{10000.0};
  double search_source_shortfall_per_vehicle{100.0};
  hacdcpf::io::SiouxFallsIEEE123Options case_options;
};

void print_usage(const char* executable) {
  std::cout
      << "Usage: " << executable << " [options]\n\n"
      << "Study modes:\n"
      << "  --traffic-only                 Run CTM and charging recourse only\n"
      << "  --sequence                     Solve every power-system interval\n"
      << "  --sequence-summary             Solve the sequence and emit summaries\n"
      << "  --batch                        Run the paper road-capacity sweep\n"
      << "  --grid-admissibility           Verify interval charging by OPF and PF\n\n"
      << "Route-committed search:\n"
      << "  --route-search                 Run scenario-decomposed partial-loading search\n"
      << "  --search-exhaustive             Exhaustively verify small plan sets\n"
      << "  --search-scenarios VALUE       Consecutive seeds starting at --typhoon-seed\n"
      << "  --search-threads VALUE         Parallel scenario workers\n"
      << "  --search-max-nodes VALUE       Node-processing limit\n"
      << "  --search-relative-gap VALUE    Relative optimality-gap tolerance\n"
      << "  --search-recovery-voll VALUE   Recovery-loss weight [$/MWh]\n"
      << "  --search-source-shortfall VALUE  Source shortfall penalty [$/vehicle]\n\n"
      << "Hazard controls:\n"
      << "  --typhoon-road-impact          Apply one track to road degradation and power-fault generation\n"
      << "  --typhoon-seed VALUE\n"
      << "  --typhoon-delta-p-hpa VALUE\n"
      << "  --road-drainage-mm-hr VALUE\n"
      << "  --road-flood-closure-mm VALUE\n\n"
      << "Case controls:\n"
      << "  --road-capacity-scale VALUE\n"
      << "  --selected-od-pairs VALUE\n"
      << "  --paths-per-od VALUE\n"
      << "  --od-demand-scale VALUE\n"
      << "  --ev-penetration VALUE\n"
      << "  --station-power-kw VALUE\n"
      << "  --charging-dwell-steps VALUE\n"
      << "  --base-load-scale VALUE\n"
      << "  --phase-voltage-min-pu VALUE\n"
      << "  --phase-voltage-max-pu VALUE\n"
      << "  --vuf-limit VALUE\n"
      << "  --converter-capacity-scale VALUE\n"
      << "  --balanced-station-phases\n"
      << "  --num-steps VALUE\n"
      << "  --ctm-substeps-per-power-step VALUE\n"
      << "  -h, --help                     Show this message\n";
}

double parse_double(const std::string& value, const std::string& option) {
  std::size_t parsed = 0;
  const double result = std::stod(value, &parsed);
  if (parsed != value.size()) {
    throw std::invalid_argument("invalid value for " + option + ": " + value);
  }
  return result;
}

int parse_int(const std::string& value, const std::string& option) {
  std::size_t parsed = 0;
  const int result = std::stoi(value, &parsed);
  if (parsed != value.size()) {
    throw std::invalid_argument("invalid value for " + option + ": " + value);
  }
  return result;
}

ExperimentOptions parse_arguments(int argc, char** argv) {
  ExperimentOptions options;
  options.case_options.external_data_root =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) / "external_data";
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    auto require_value = [&](const std::string& option) -> std::string {
      if (++i >= argc) {
        throw std::invalid_argument("missing value for " + option);
      }
      return argv[i];
    };
    if (argument == "-h" || argument == "--help") {
      options.show_help = true;
    } else if (argument == "--sequence") {
      options.solve_sequence = true;
    } else if (argument == "--sequence-summary") {
      options.solve_sequence = true;
      options.compact_sequence = true;
    } else if (argument == "--batch") {
      options.batch = true;
      options.solve_sequence = true;
      options.compact_sequence = true;
    } else if (argument == "--traffic-only") {
      options.traffic_only = true;
    } else if (argument == "--grid-admissibility") {
      options.grid_admissibility = true;
    } else if (argument == "--route-search") {
      options.route_search = true;
    } else if (argument == "--search-exhaustive") {
      options.search_exhaustive = true;
    } else if (argument == "--search-scenarios") {
      options.search_scenarios =
          parse_int(require_value(argument), argument);
    } else if (argument == "--search-threads") {
      options.search_threads = parse_int(require_value(argument), argument);
    } else if (argument == "--search-max-nodes") {
      options.search_max_nodes = parse_int(require_value(argument), argument);
    } else if (argument == "--search-relative-gap") {
      options.search_relative_gap =
          parse_double(require_value(argument), argument);
    } else if (argument == "--search-recovery-voll") {
      options.search_recovery_voll_per_mwh =
          parse_double(require_value(argument), argument);
    } else if (argument == "--search-source-shortfall") {
      options.search_source_shortfall_per_vehicle =
          parse_double(require_value(argument), argument);
    } else if (argument == "--typhoon-road-impact") {
      options.typhoon_road_impact = true;
    } else if (argument == "--typhoon-seed") {
      options.typhoon_seed = static_cast<unsigned int>(
          parse_int(require_value(argument), argument));
    } else if (argument == "--typhoon-delta-p-hpa") {
      options.typhoon_delta_p_hpa =
          parse_double(require_value(argument), argument);
    } else if (argument == "--road-drainage-mm-hr") {
      options.road_drainage_mm_hr =
          parse_double(require_value(argument), argument);
    } else if (argument == "--road-flood-closure-mm") {
      options.road_flood_closure_mm =
          parse_double(require_value(argument), argument);
    } else if (argument == "--road-capacity-scale") {
      options.case_options.road_capacity_scale =
          parse_double(require_value(argument), argument);
    } else if (argument == "--selected-od-pairs") {
      options.case_options.selected_od_pairs =
          parse_int(require_value(argument), argument);
    } else if (argument == "--paths-per-od") {
      options.case_options.paths_per_od =
          parse_int(require_value(argument), argument);
    } else if (argument == "--od-demand-scale") {
      options.case_options.od_demand_scale =
          parse_double(require_value(argument), argument);
    } else if (argument == "--ev-penetration") {
      options.case_options.ev_penetration =
          parse_double(require_value(argument), argument);
    } else if (argument == "--station-power-kw") {
      options.case_options.station_power_kw =
          parse_double(require_value(argument), argument);
    } else if (argument == "--charging-dwell-steps") {
      options.case_options.charging_dwell_steps =
          parse_int(require_value(argument), argument);
    } else if (argument == "--base-load-scale") {
      options.case_options.base_load_scale =
          parse_double(require_value(argument), argument);
    } else if (argument == "--phase-voltage-min-pu") {
      options.case_options.phase_voltage_min_pu =
          parse_double(require_value(argument), argument);
    } else if (argument == "--phase-voltage-max-pu") {
      options.case_options.phase_voltage_max_pu =
          parse_double(require_value(argument), argument);
    } else if (argument == "--vuf-limit") {
      options.case_options.vuf_limit =
          parse_double(require_value(argument), argument);
    } else if (argument == "--converter-capacity-scale") {
      options.case_options.converter_capacity_scale =
          parse_double(require_value(argument), argument);
    } else if (argument == "--balanced-station-phases") {
      options.case_options.balanced_station_phase_allocation = true;
    } else if (argument == "--num-steps") {
      options.case_options.num_steps =
          parse_int(require_value(argument), argument);
    } else if (argument == "--ctm-substeps-per-power-step") {
      options.case_options.ctm_substeps_per_power_step =
          parse_int(require_value(argument), argument);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  return options;
}

double requested_vehicles(const auto& demands) {
  return std::accumulate(
      demands.begin(), demands.end(), 0.0,
      [](double total, const auto& demand) {
        return total + std::max(0.0, demand.vehicles);
      });
}

double route_flow_vehicles(
    const auto& demands,
    const std::unordered_map<int, std::vector<double>>& route_flow) {
  std::unordered_set<int> route_indices;
  for (const auto& demand : demands) {
    route_indices.insert(demand.candidate_route_indices.begin(),
                         demand.candidate_route_indices.end());
  }
  double total = 0.0;
  for (int route : route_indices) {
    const auto found = route_flow.find(route);
    if (found != route_flow.end()) {
      total += std::accumulate(found->second.begin(), found->second.end(), 0.0);
    }
  }
  return total;
}

double cumulative_arrival_time_hr(const std::vector<double>& arrivals,
                                  double required_vehicles,
                                  double dt_ctm_hr,
                                  double unavailable_time_hr) {
  if (required_vehicles <= 1e-9) return 0.0;
  double cumulative = 0.0;
  for (std::size_t step = 0; step < arrivals.size(); ++step) {
    cumulative += std::max(0.0, arrivals[step]);
    if (cumulative + 1e-9 >= required_vehicles) {
      return static_cast<double>(step + 1) * dt_ctm_hr;
    }
  }
  return unavailable_time_hr;
}

json resilience_summary(
    const hacdcpf::analysis::DistributionResilienceResult& result) {
  json shed_profile = json::array();
  json restoration_profile = json::array();
  for (const auto& step : result.steps) {
    shed_profile.push_back(step.shed_mw);
    restoration_profile.push_back(step.restoration_ratio);
  }
  const std::string model_scope =
      result.model ==
              hacdcpf::analysis::DistributionResilienceModel::HeuristicSequential
          ? "AC topology-and-capacity sequential restoration"
          : result.model_stats.model_scope;
  return {{"feasible", result.feasible},
          {"status", result.status},
          {"model_scope", model_scope},
          {"resilience_index", result.resilience_index},
          {"total_demand_mwh", result.total_demand_mwh},
          {"total_served_mwh", result.total_served_mwh},
          {"total_shed_mwh", result.total_shed_mwh},
          {"weighted_unserved_consequence", result.weighted_unserved_mwh},
          {"peak_shed_mw", result.peak_shed_mw},
          {"mobile_energy_delivered_mwh",
           result.mess_energy_delivered_mwh},
          {"shed_profile_mw", std::move(shed_profile)},
          {"restoration_profile", std::move(restoration_profile)}};
}

json run_typhoon_resilience_timing_comparison(
    const hacdcpf::io::SiouxFallsIEEE123Case& data,
    const hacdcpf::evpt::CTMForwardResult& free_flow,
    const hacdcpf::evpt::CTMForwardResult& ctm,
    const hacdcpf::analysis::TyphoonFaultSequenceResult& typhoon) {
  constexpr int kArrivalCohortsPerStation = 8;
  constexpr double kSupportEnergyMWhPerVehicle = 0.010;
  constexpr double kSupportPowerMWPerVehicle = 0.002;
  constexpr double kRestorationTimeStepHr = 1.0 / 12.0;
  const double horizon_hr =
      data.options.num_steps * data.options.time_step_hr;
  const double unavailable_time_hr =
      horizon_hr + data.options.time_step_hr;

  auto system = data.problem.system;
  system.mobile_storage.clear();
  hacdcpf::analysis::DistributionResilienceOptions free_flow_options;
  free_flow_options.model =
      hacdcpf::analysis::DistributionResilienceModel::HeuristicSequential;
  free_flow_options.horizon_hours =
      std::max(1, static_cast<int>(std::ceil(horizon_hr)));
  free_flow_options.time_step_hr = kRestorationTimeStepHr;
  free_flow_options.allow_reconfiguration = true;
  free_flow_options.allow_mess_dispatch = true;
  free_flow_options.default_fault_count = 0;
  free_flow_options.renewable_profile = typhoon.renewable_profile_multiplier;
  for (const auto& fault : typhoon.faults) {
    if (fault.branch_kind ==
        hacdcpf::analysis::ResilienceBranchKind::AC) {
      free_flow_options.faults.push_back(fault);
    }
  }
  auto ctm_options = free_flow_options;

  int next_storage_index = 9001;
  double represented_vehicles = 0.0;
  for (const auto& mapping : data.station_mapping) {
    const auto ff_it =
        free_flow.simulation.station_arrivals.find(mapping.station_id);
    if (ff_it == free_flow.simulation.station_arrivals.end()) continue;
    const auto ctm_it = ctm.simulation.station_arrivals.find(mapping.station_id);
    const std::vector<double> empty;
    const auto& ctm_arrivals = ctm_it == ctm.simulation.station_arrivals.end()
        ? empty
        : ctm_it->second;
    const double committed = std::accumulate(
        ff_it->second.begin(), ff_it->second.end(), 0.0);
    if (committed <= 1e-9) continue;
    const double cohort_vehicles =
        committed / static_cast<double>(kArrivalCohortsPerStation);
    represented_vehicles += committed;
    for (int cohort = 1; cohort <= kArrivalCohortsPerStation; ++cohort) {
      const double threshold = cohort_vehicles * cohort;
      hacdcpf::MobileStorage storage;
      storage.index = next_storage_index++;
      storage.name = "emergency-EV-cohort-station-" +
          std::to_string(mapping.station_id) + "-" +
          std::to_string(cohort);
      storage.bus = mapping.power_bus_index;
      storage.target_bus = mapping.power_bus_index;
      storage.in_service = true;
      storage.pmax_mw = cohort_vehicles * kSupportPowerMWPerVehicle;
      storage.p_rated_mw = storage.pmax_mw;
      storage.e_rated_mwh =
          cohort_vehicles * kSupportEnergyMWhPerVehicle;
      storage.e_mwh = storage.e_rated_mwh;
      storage.soc_init = 1.0;
      storage.soc_min = 0.0;
      storage.soc_max = 1.0;
      storage.eta_discharge = 0.95;
      storage.grid_forming = true;
      storage.max_travel_distance_km = 1e-6;
      system.mobile_storage.push_back(storage);
      free_flow_options.mobile_storage_available_from_hr[storage.index] =
          cumulative_arrival_time_hr(
              ff_it->second, threshold, free_flow.simulation.dt_ctm_hr,
              unavailable_time_hr);
      ctm_options.mobile_storage_available_from_hr[storage.index] =
          cumulative_arrival_time_hr(
              ctm_arrivals, threshold, ctm.simulation.dt_ctm_hr,
              unavailable_time_hr);
    }
  }

  const auto free_flow_result =
      hacdcpf::analysis::run_distribution_resilience_assessment(
          system, free_flow_options);
  const auto ctm_result =
      hacdcpf::analysis::run_distribution_resilience_assessment(
          system, ctm_options);
  return {{"recourse_model", "sequential feasible restoration baseline"},
          {"power_network_scope", "AC distribution restoration"},
          {"power_faults_applied", true},
          {"ac_fault_count", free_flow_options.faults.size()},
          {"arrival_cohorts_per_station", kArrivalCohortsPerStation},
          {"restoration_time_step_hr", kRestorationTimeStepHr},
          {"mobile_resource_count", system.mobile_storage.size()},
          {"represented_ev_vehicles", represented_vehicles},
          {"support_energy_mwh_per_vehicle",
           kSupportEnergyMWhPerVehicle},
          {"support_power_mw_per_vehicle", kSupportPowerMWPerVehicle},
          {"free_flow", resilience_summary(free_flow_result)},
          {"ctm", resilience_summary(ctm_result)},
          {"resilience_overestimation",
           free_flow_result.resilience_index - ctm_result.resilience_index},
          {"shed_energy_understatement_mwh",
           ctm_result.total_shed_mwh - free_flow_result.total_shed_mwh},
          {"model_limitation",
           "This feasible sequential restoration baseline applies the AC "
           "fault subset. The three-phase AC/DC OPF and its outer relaxation "
           "are evaluated separately; this field is not reported as their "
           "global optimum."}};
}

json run_experiment(
    const hacdcpf::io::SiouxFallsIEEE123Options& case_options,
    bool solve_sequence,
    bool compact_sequence,
    bool traffic_only,
    bool grid_admissibility,
    bool typhoon_road_impact,
    unsigned int typhoon_seed,
    double typhoon_delta_p_hpa,
    double road_drainage_mm_hr,
    double road_flood_closure_mm) {
  auto data =
      hacdcpf::io::build_sioux_falls_ieee123_case(case_options);

  hacdcpf::analysis::TyphoonFaultSequenceResult typhoon;
  hacdcpf::analysis::TyphoonTrafficImpactResult road_impact;
  if (typhoon_road_impact) {
    hacdcpf::analysis::TyphoonScenarioOptions hazard_options;
    hazard_options.horizon_hours = std::max(
        8, static_cast<int>(std::ceil(
               case_options.num_steps * case_options.time_step_hr)) + 2);
    hazard_options.time_step_hr = 1.0;
    hazard_options.seed = typhoon_seed;
    hazard_options.stochastic = false;
    hazard_options.initial_latitude = 22.30;
    hazard_options.initial_longitude = 113.55;
    hazard_options.initial_heading_deg = 0.0;
    hazard_options.initial_translation_speed_kmph = 18.0;
    hazard_options.initial_delta_p_hpa = typhoon_delta_p_hpa;
    hazard_options.initial_rmw_km = 40.0;
    hazard_options.fallback_case_center_latitude = 22.75;
    hazard_options.fallback_case_center_longitude = 113.55;
    typhoon = hacdcpf::analysis::generate_typhoon_fault_sequence(
        data.problem.system, hazard_options);

    hacdcpf::analysis::TyphoonTrafficImpactOptions road_options;
    road_options.num_steps = case_options.num_steps;
    road_options.time_step_hr = case_options.time_step_hr;
    road_options.coordinate_mode =
        hacdcpf::analysis::TyphoonTrafficCoordinateMode::AffineToStudyArea;
    road_options.study_center_latitude =
        hazard_options.fallback_case_center_latitude;
    road_options.study_center_longitude =
        hazard_options.fallback_case_center_longitude;
    road_options.drainage_rate_mm_hr = road_drainage_mm_hr;
    road_options.flood_closure_depth_mm = road_flood_closure_mm;
    road_impact = hacdcpf::analysis::apply_typhoon_traffic_impact(
        data.problem.traffic, typhoon.track, road_options);
    data.problem.traffic = road_impact.impacted_traffic;
  }

  hacdcpf::evpt::EVPowerTrafficOptions ev_options;
  ev_options.num_steps = case_options.num_steps;
  ev_options.time_step_hr = case_options.time_step_hr;
  hacdcpf::evpt::CTMOptions ctm_options;
  ctm_options.route_specific_cells = true;
  ctm_options.record_cell_history = false;
  ctm_options.dt_ctm_hr = case_options.time_step_hr /
      static_cast<double>(case_options.ctm_substeps_per_power_step);
  const auto assignment = equal_route_assignment(data.problem);
  auto free_flow_problem = data.problem;
  constexpr double kFreeFlowCapacityMultiplier = 100.0;
  for (auto& link : free_flow_problem.traffic.links) {
    link.capacity_veh_per_hr *= kFreeFlowCapacityMultiplier;
    link.jam_vehicles *= kFreeFlowCapacityMultiplier;
    for (double& capacity : link.capacity_profile_veh_per_hr) {
      capacity *= kFreeFlowCapacityMultiplier;
    }
  }
  const auto free_flow =
      hacdcpf::evpt::simulate_ev_power_traffic_ctm_forward(
          free_flow_problem, assignment, ev_options, ctm_options);
  if (!free_flow.valid || free_flow.source_entry_shortfall_vehicles > 1e-6) {
    throw std::runtime_error(
        "capacity-calibrated free-flow replay did not admit all traffic");
  }
  const auto forward = hacdcpf::evpt::simulate_ev_power_traffic_ctm_forward(
      data.problem, assignment, ev_options, ctm_options);
  if (!forward.valid) {
    throw std::runtime_error("CTM-forward failed: " + forward.status);
  }
  hacdcpf::io::SiouxFallsIEEE123RecourseOptions recourse_options;
  recourse_options.arrival_opens_until_horizon = true;
  const auto recourse = hacdcpf::io::build_sioux_falls_ieee123_recourse(
      data, forward, recourse_options);
  const auto free_flow_recourse =
      hacdcpf::io::build_sioux_falls_ieee123_recourse(
          data, free_flow, recourse_options);
  hacdcpf::io::SiouxFallsIEEE123RecourseOptions fixed_dwell_options;
  const auto fixed_dwell_recourse =
      hacdcpf::io::build_sioux_falls_ieee123_recourse(
          data, forward, fixed_dwell_options);
  const auto free_flow_fixed_dwell_recourse =
      hacdcpf::io::build_sioux_falls_ieee123_recourse(
          data, free_flow, fixed_dwell_options);

  std::vector<double> total_ev_mw(
      static_cast<std::size_t>(case_options.num_steps), 0.0);
  for (const auto& [station, profile] : recourse.station_grid_load_kw) {
    (void)station;
    for (std::size_t step = 0; step < profile.size(); ++step) {
      total_ev_mw[step] += profile[step] / 1000.0;
    }
  }
  const int peak_step = static_cast<int>(std::distance(
      total_ev_mw.begin(),
      std::max_element(total_ev_mw.begin(), total_ev_mw.end())));

  const double requested_ev = requested_vehicles(data.problem.demands);
  const double requested_icv = requested_vehicles(data.problem.icv_demands);
  const double admitted_ev = route_flow_vehicles(
      data.problem.demands, forward.simulation.route_admitted_departures);
  const double admitted_icv = route_flow_vehicles(
      data.problem.icv_demands, forward.simulation.route_admitted_departures);
  const double terminal_ev = route_flow_vehicles(
      data.problem.demands, forward.simulation.route_terminal_arrivals);
  const double terminal_icv = route_flow_vehicles(
      data.problem.icv_demands, forward.simulation.route_terminal_arrivals);
  const double committed_ev_energy_kwh =
      requested_ev * case_options.charging_energy_kwh_per_vehicle;
  const double ctm_unrestored_energy_kwh = std::max(
      0.0, committed_ev_energy_kwh - recourse.delivered_battery_energy_kwh);
  const double free_flow_unrestored_energy_kwh = std::max(
      0.0,
      committed_ev_energy_kwh - free_flow_recourse.delivered_battery_energy_kwh);
  const double fixed_dwell_ctm_unrestored_energy_kwh = std::max(
      0.0,
      committed_ev_energy_kwh -
          fixed_dwell_recourse.delivered_battery_energy_kwh);
  const double fixed_dwell_free_flow_unrestored_energy_kwh = std::max(
      0.0,
      committed_ev_energy_kwh -
          free_flow_fixed_dwell_recourse.delivered_battery_energy_kwh);

  json output;
  output["case"] = data.problem.system.name;
  output["parameters"] = {
      {"num_steps", case_options.num_steps},
      {"time_step_hr", case_options.time_step_hr},
      {"ctm_substeps_per_power_step",
       case_options.ctm_substeps_per_power_step},
      {"od_demand_scale", case_options.od_demand_scale},
      {"ev_penetration", case_options.ev_penetration},
      {"road_capacity_scale", case_options.road_capacity_scale},
      {"station_power_kw", case_options.station_power_kw},
      {"charging_dwell_steps", case_options.charging_dwell_steps},
      {"base_load_scale", case_options.base_load_scale},
      {"phase_voltage_min_pu", case_options.phase_voltage_min_pu},
      {"phase_voltage_max_pu", case_options.phase_voltage_max_pu},
      {"vuf_limit", case_options.vuf_limit},
      {"converter_capacity_scale", case_options.converter_capacity_scale},
      {"balanced_station_phase_allocation",
       case_options.balanced_station_phase_allocation},
      {"typhoon_road_impact", typhoon_road_impact}};
  if (typhoon_road_impact) {
    json faults = json::array();
    for (const auto& fault : typhoon.faults) {
      faults.push_back({
          {"branch_kind", hacdcpf::analysis::to_string(fault.branch_kind)},
          {"branch_index", fault.branch_index},
          {"outage_start_hr", fault.outage_start_hr},
          {"repair_duration_hr", fault.repair_duration_hr}});
    }
    output["typhoon"] = {
        {"seed", typhoon.seed},
        {"selected_category",
         hacdcpf::analysis::to_string(typhoon.selected_category)},
        {"track_maximum_wind_ms", typhoon.selected_track_max_vmax_ms},
        {"track_points", typhoon.track.size()},
        {"road_coordinate_mapping", "affine_to_coastal_study_area"},
        {"road_peak_wind_ms", road_impact.peak_wind_ms},
        {"road_peak_rainfall_mm_hr", road_impact.peak_rainfall_mm_hr},
        {"road_peak_surface_water_mm", road_impact.peak_surface_water_mm},
        {"road_minimum_speed_factor", road_impact.minimum_speed_factor},
        {"road_minimum_capacity_factor",
         road_impact.minimum_capacity_factor},
        {"road_closed_link_steps", road_impact.closed_link_steps},
        {"roads_closed_at_least_once",
         road_impact.links_closed_at_least_once},
        {"road_model_scope", road_impact.model_scope},
        {"power_fault_count", typhoon.faults.size()},
        {"power_faults", std::move(faults)},
        {"power_faults_applied_to_resilience_baseline", true},
        {"power_faults_applied_to_current_opf", false},
        {"power_fault_model_limitation",
         "The common-track AC fault subset is applied to the sequential "
         "restoration baseline; the current three-phase AC/DC OPF sequence "
         "remains the undamaged-grid traffic-timing comparison."}};
  }
  output["traffic"] = {
      {"requested_vehicles", forward.requested_vehicles},
      {"requested_ev_vehicles", requested_ev},
      {"requested_icv_vehicles", requested_icv},
      {"admitted_vehicles", forward.admitted_vehicles},
      {"admitted_ev_vehicles", admitted_ev},
      {"admitted_icv_vehicles", admitted_icv},
      {"source_entry_shortfall_vehicles",
       forward.source_entry_shortfall_vehicles},
      {"source_entry_shortfall_ev_vehicles",
       std::max(0.0, requested_ev - admitted_ev)},
      {"source_entry_shortfall_icv_vehicles",
       std::max(0.0, requested_icv - admitted_icv)},
      {"ctm_substeps", forward.simulation.step_link_results.size()},
      {"ctm_substeps_per_power_step",
       forward.simulation.steps_per_sim_step},
      {"terminal_arrivals_vehicles",
       forward.simulation.terminal_arrivals_vehicles},
      {"terminal_ev_vehicles", terminal_ev},
      {"terminal_icv_vehicles", terminal_icv},
      {"ev_not_at_destination_by_horizon_vehicles",
       std::max(0.0, requested_ev - terminal_ev)},
      {"ev_admitted_not_at_destination_by_horizon_vehicles",
       admitted_ev - terminal_ev},
      {"icv_admitted_not_at_destination_by_horizon_vehicles",
       admitted_icv - terminal_icv},
      {"final_network_occupancy_vehicles",
       forward.simulation.final_occupancy_vehicles},
      {"vehicle_conservation_error_vehicles",
       forward.simulation.vehicle_conservation_error_vehicles}};
  output["charging"] = {
      {"scheduling_scope", recourse.scheduling_scope},
      {"service_eligible_arrivals_vehicles",
       recourse.total_arrivals_vehicles},
      {"service_eligible_fraction_of_admitted_ev",
       admitted_ev > 0.0 ? recourse.total_arrivals_vehicles / admitted_ev : 0.0},
      {"requested_battery_energy_kwh",
       recourse.requested_battery_energy_kwh},
      {"committed_ev_energy_kwh", committed_ev_energy_kwh},
      {"delivered_battery_energy_kwh",
       recourse.delivered_battery_energy_kwh},
      {"charging_window_shortfall_kwh",
       recourse.charging_window_shortfall_kwh},
      {"unrestored_committed_energy_kwh", ctm_unrestored_energy_kwh},
      {"maximum_station_backlog_kwh",
       recourse.maximum_station_backlog_kwh},
      {"backlog_time_integral_kwh_hr",
       recourse.backlog_time_integral_kwh_hr},
      {"station_saturated_intervals",
       recourse.station_saturated_intervals},
      {"aggregate_backlog_profile_kwh", recourse.aggregate_backlog_kwh},
      {"peak_step", peak_step},
      {"peak_ev_load_mw", total_ev_mw[static_cast<std::size_t>(peak_step)]},
      {"grid_load_profile_mw", total_ev_mw}};
  output["fixed_dwell_applicability_case"] = {
      {"scheduling_scope", fixed_dwell_recourse.scheduling_scope},
      {"free_flow_shortfall_kwh",
       free_flow_fixed_dwell_recourse.charging_window_shortfall_kwh},
      {"ctm_shortfall_kwh",
       fixed_dwell_recourse.charging_window_shortfall_kwh},
      {"free_flow_unrestored_committed_energy_kwh",
       fixed_dwell_free_flow_unrestored_energy_kwh},
      {"ctm_unrestored_committed_energy_kwh",
       fixed_dwell_ctm_unrestored_energy_kwh},
      {"arrival_monotonicity_satisfied", false},
      {"interpretation",
       "A dwell deadline tied to arrival can make traffic spreading reduce "
       "station saturation; this case is outside the resilience-ordering "
       "theorem."}};
  double free_flow_peak_ev_load_mw = 0.0;
  for (int step = 0; step < case_options.num_steps; ++step) {
    double step_load_mw = 0.0;
    for (const auto& [station, profile] :
         free_flow_recourse.station_grid_load_kw) {
      (void)station;
      step_load_mw += profile[static_cast<std::size_t>(step)] / 1000.0;
    }
    free_flow_peak_ev_load_mw =
        std::max(free_flow_peak_ev_load_mw, step_load_mw);
  }
  const double free_flow_mean_arrival =
      mean_arrival_step(free_flow_recourse.station_arrivals_vehicles);
  const double ctm_mean_arrival =
      mean_arrival_step(recourse.station_arrivals_vehicles);
  output["free_flow_counterfactual"] = {
      {"road_capacity_multiplier", kFreeFlowCapacityMultiplier},
      {"calibration", "same CTM discretization with nonbinding road capacity"},
      {"arrivals_vehicles", free_flow_recourse.total_arrivals_vehicles},
      {"mean_arrival_step", free_flow_mean_arrival},
      {"requested_battery_energy_kwh",
       free_flow_recourse.requested_battery_energy_kwh},
      {"delivered_battery_energy_kwh",
       free_flow_recourse.delivered_battery_energy_kwh},
      {"charging_window_shortfall_kwh",
       free_flow_recourse.charging_window_shortfall_kwh},
      {"unrestored_committed_energy_kwh",
       free_flow_unrestored_energy_kwh},
      {"backlog_time_integral_kwh_hr",
       free_flow_recourse.backlog_time_integral_kwh_hr},
      {"peak_ev_load_mw", free_flow_peak_ev_load_mw}};
  output["resilience_ordering_check"] = {
      {"arrival_monotonicity_satisfied", true},
      {"free_flow_unrestored_committed_energy_kwh",
       free_flow_unrestored_energy_kwh},
      {"ctm_unrestored_committed_energy_kwh",
       ctm_unrestored_energy_kwh},
      {"congestion_induced_understatement_kwh",
       ctm_unrestored_energy_kwh - free_flow_unrestored_energy_kwh},
      {"free_flow_not_worse",
       free_flow_unrestored_energy_kwh <=
           ctm_unrestored_energy_kwh + 1e-3}};
  output["congestion_timing"] = {
      {"ctm_mean_arrival_step", ctm_mean_arrival},
      {"free_flow_mean_arrival_step", free_flow_mean_arrival},
      {"mean_arrival_delay_minutes",
       (ctm_mean_arrival - free_flow_mean_arrival) *
           case_options.time_step_hr * 60.0}};
  output["power_model"] = {
      {"formulation", "three-phase-unbalanced-ac-dc-hybrid-opf"},
      {"ac_phase_nodes", data.phase_hybrid_model.opf.y_ac.rows()},
      {"dc_nodes", data.phase_hybrid_model.opf.g_dc.rows()},
      {"phase_generators", data.phase_hybrid_model.opf.generators.size()},
      {"vscs", data.phase_hybrid_model.opf.converters.size()},
      {"phase_voltage_min_pu",
       data.phase_hybrid_model.opf.v_min_pu.minCoeff()},
      {"phase_voltage_max_pu",
       data.phase_hybrid_model.opf.v_max_pu.maxCoeff()},
      {"vuf_limit", data.phase_hybrid_model.opf.vuf_max},
      {"model_limitations", data.phase_hybrid_model.model_limitations}};
  if (typhoon_road_impact) {
    output["joint_resilience_timing"] =
        run_typhoon_resilience_timing_comparison(
            data, free_flow, forward, typhoon);
  }

  if (traffic_only) {
    output["mode"] = "ctm-forward-and-charging-recourse";
    return output;
  }

  hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFOptions opf_options;
  opf_options.variant =
      hacdcpf::opf::phase_hybrid::ModelVariant::GraphReduced;
  opf_options.backend =
      hacdcpf::opf::phase_hybrid::SolverBackend::Ipopt;
  opf_options.max_iterations = 500;
  opf_options.tolerance = 1e-6;
  opf_options.reduction_options.max_front = 16;
  opf_options.reduction_options.max_nnz_ratio = 12.0;

  auto baseline_case = [&](int step) {
    auto problem = data.phase_hybrid_model.opf;
    problem.name += "_baseline_step_" + std::to_string(step);
    const double load_multiplier = case_options.base_load_scale *
        data.base_load_multiplier[static_cast<std::size_t>(step)];
    problem.p_load_pu *= load_multiplier;
    problem.q_load_pu *= load_multiplier;
    return problem;
  };

  auto verified_feasible = [&](
      const hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFCase& problem) {
    const auto result =
        hacdcpf::opf::phase_hybrid::solve_three_phase_hybrid_opf(
            problem, opf_options);
    if (!result.converged) return false;
    const auto pf_case =
        hacdcpf::opf::phase_hybrid::make_three_phase_hybrid_pf_case(
            problem, result);
    hacdcpf::powerflow::ThreePhaseHybridPFOptions pf_options;
    pf_options.max_iterations = 100;
    pf_options.tolerance = 1e-8;
    const auto replay =
        hacdcpf::powerflow::solve_three_phase_hybrid_pf(pf_case, pf_options);
    return replay.converged && replay.residual <= 1e-7;
  };

  if (grid_admissibility) {
    const auto started = std::chrono::steady_clock::now();
    std::vector<double> load_factors(
        static_cast<std::size_t>(case_options.num_steps), 1.0);
    int verified_full_load_steps = 0;
    int baseline_infeasible_steps = 0;
    double scheduled_grid_energy_kwh = 0.0;
    double admitted_grid_energy_kwh = 0.0;
    double maximum_curtailed_power_mw = 0.0;
    double minimum_load_factor = 1.0;
    constexpr int kGridFactorBisectionIterations = 8;
    for (int step = 0; step < case_options.num_steps; ++step) {
      const auto base = baseline_case(step);
      const auto& full = recourse.opf_cases[static_cast<std::size_t>(step)];
      const double scheduled_mw = total_ev_mw[static_cast<std::size_t>(step)];
      scheduled_grid_energy_kwh +=
          scheduled_mw * 1000.0 * case_options.time_step_hr;
      if (scheduled_mw <= 1e-10) {
        ++verified_full_load_steps;
        continue;
      }
      if (!verified_feasible(base)) {
        load_factors[static_cast<std::size_t>(step)] = 0.0;
        minimum_load_factor = 0.0;
        ++baseline_infeasible_steps;
        maximum_curtailed_power_mw =
            std::max(maximum_curtailed_power_mw, scheduled_mw);
        continue;
      }
      if (verified_feasible(full)) {
        admitted_grid_energy_kwh +=
            scheduled_mw * 1000.0 * case_options.time_step_hr;
        ++verified_full_load_steps;
        continue;
      }

      double lower = 0.0;
      double upper = 1.0;
      for (int iteration = 0;
           iteration < kGridFactorBisectionIterations; ++iteration) {
        const double factor = 0.5 * (lower + upper);
        auto trial = base;
        trial.name = full.name + "_grid_factor_" + std::to_string(factor);
        trial.p_load_pu =
            base.p_load_pu + factor * (full.p_load_pu - base.p_load_pu);
        trial.q_load_pu =
            base.q_load_pu + factor * (full.q_load_pu - base.q_load_pu);
        if (verified_feasible(trial)) {
          lower = factor;
        } else {
          upper = factor;
        }
      }
      load_factors[static_cast<std::size_t>(step)] = lower;
      minimum_load_factor = std::min(minimum_load_factor, lower);
      admitted_grid_energy_kwh += lower * scheduled_mw * 1000.0 *
          case_options.time_step_hr;
      maximum_curtailed_power_mw = std::max(
          maximum_curtailed_power_mw, (1.0 - lower) * scheduled_mw);
    }
    const double curtailed_grid_energy_kwh =
        scheduled_grid_energy_kwh - admitted_grid_energy_kwh;
    const double runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    output["grid_admissibility"] = {
        {"policy", "uniform proportional station-load curtailment by interval"},
        {"verification", "nonlinear OPF plus independent three-phase power-flow replay"},
        {"bisection_iterations", kGridFactorBisectionIterations},
        {"verified_full_load_steps", verified_full_load_steps},
        {"baseline_infeasible_steps", baseline_infeasible_steps},
        {"minimum_load_factor", minimum_load_factor},
        {"maximum_curtailed_power_mw", maximum_curtailed_power_mw},
        {"scheduled_grid_energy_kwh", scheduled_grid_energy_kwh},
        {"admitted_grid_energy_kwh", admitted_grid_energy_kwh},
        {"grid_curtailed_energy_kwh", curtailed_grid_energy_kwh},
        {"battery_energy_shortfall_kwh",
         curtailed_grid_energy_kwh * recourse_options.charging_efficiency},
        {"load_factors", load_factors},
        {"runtime_ms", runtime_ms}};
  }

  if (solve_sequence) {
    const auto results =
        hacdcpf::opf::phase_hybrid::solve_three_phase_hybrid_opf_sequence(
            recourse.opf_cases, opf_options);
    output["mode"] = "three-phase-hybrid-opf-sequence";
    int converged_steps = 0;
    double total_runtime_ms = 0.0;
    double total_objective = 0.0;
    double maximum_vuf = 0.0;
    if (!compact_sequence) output["opf"] = json::array();
    for (std::size_t step = 0; step < results.size(); ++step) {
      converged_steps += results[step].converged ? 1 : 0;
      total_runtime_ms += results[step].runtime_ms;
      total_objective += results[step].objective;
      maximum_vuf = std::max(maximum_vuf, results[step].max_vuf);
      auto row = opf_summary(
          results[step], recourse.opf_cases[step]);
      row["step"] = step;
      row["ev_load_mw"] = total_ev_mw[step];
      if (!compact_sequence) output["opf"].push_back(row);
      if (static_cast<int>(step) == peak_step) {
        output["peak_step_opf"] = std::move(row);
      }
    }
    output["sequence_summary"] = {
        {"steps", results.size()},
        {"converged_steps", converged_steps},
        {"total_runtime_ms", total_runtime_ms},
        {"total_objective", total_objective},
        {"sum_of_period_objectives", total_objective},
        {"integrated_objective",
         total_objective * case_options.time_step_hr},
        {"maximum_vuf", maximum_vuf}};

    std::vector<hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFCase>
        baseline_cases;
    baseline_cases.reserve(static_cast<std::size_t>(case_options.num_steps));
    for (int step = 0; step < case_options.num_steps; ++step) {
      baseline_cases.push_back(baseline_case(step));
    }
    const auto baseline_results =
        hacdcpf::opf::phase_hybrid::solve_three_phase_hybrid_opf_sequence(
            baseline_cases, opf_options);
    int baseline_converged_steps = 0;
    double baseline_runtime_ms = 0.0;
    double baseline_total_objective = 0.0;
    double baseline_maximum_vuf = 0.0;
    for (const auto& result : baseline_results) {
      baseline_converged_steps += result.converged ? 1 : 0;
      baseline_runtime_ms += result.runtime_ms;
      baseline_total_objective += result.objective;
      baseline_maximum_vuf =
          std::max(baseline_maximum_vuf, result.max_vuf);
    }
    output["baseline_sequence_summary"] = {
        {"steps", baseline_results.size()},
        {"converged_steps", baseline_converged_steps},
        {"total_runtime_ms", baseline_runtime_ms},
        {"total_objective", baseline_total_objective},
        {"sum_of_period_objectives", baseline_total_objective},
        {"integrated_objective",
         baseline_total_objective * case_options.time_step_hr},
        {"maximum_vuf", baseline_maximum_vuf}};
    output["integrated_objective_increase"] =
        (total_objective - baseline_total_objective) *
        case_options.time_step_hr;
  } else {
    output["mode"] = "peak-step-three-phase-hybrid-opf-comparison";
  }

  auto no_ev = baseline_case(peak_step);
  no_ev.name += "_peak";
  const auto baseline =
      hacdcpf::opf::phase_hybrid::solve_three_phase_hybrid_opf(
          no_ev, opf_options);
  hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFResult with_ev;
  if (solve_sequence) {
    with_ev = hacdcpf::opf::phase_hybrid::solve_three_phase_hybrid_opf(
        recourse.opf_cases[static_cast<std::size_t>(peak_step)], opf_options);
  } else {
    with_ev = hacdcpf::opf::phase_hybrid::solve_three_phase_hybrid_opf(
        recourse.opf_cases[static_cast<std::size_t>(peak_step)], opf_options);
  }
  output["baseline_opf"] = opf_summary(baseline, no_ev);
  output["with_ev_opf"] = opf_summary(
      with_ev, recourse.opf_cases[static_cast<std::size_t>(peak_step)]);
  if (baseline.converged && with_ev.converged) {
    output["objective_increase"] = with_ev.objective - baseline.objective;
  }
  if (with_ev.converged) {
    hacdcpf::opf::phase_hybrid::ThreePhaseHybridRelaxationOptions
        relaxation_options;
    relaxation_options.max_outer_approximation_rounds = 0;
    relaxation_options.cost_tangent_points = 9;
    relaxation_options.cone_tolerance = 1e-6;
    const auto relaxation =
        hacdcpf::opf::phase_hybrid::solve_three_phase_hybrid_opf_relaxation(
            recourse.opf_cases[static_cast<std::size_t>(peak_step)],
            relaxation_options);
    output["peak_step_relaxation"] =
        relaxation_summary(relaxation, with_ev.objective);

    const auto pf_case =
        hacdcpf::opf::phase_hybrid::make_three_phase_hybrid_pf_case(
            recourse.opf_cases[static_cast<std::size_t>(peak_step)], with_ev);
    hacdcpf::powerflow::ThreePhaseHybridPFOptions pf_options;
    pf_options.max_iterations = 100;
    pf_options.tolerance = 1e-8;
    const auto replay =
        hacdcpf::powerflow::solve_three_phase_hybrid_pf(pf_case, pf_options);
    output["pf_replay"] = {
        {"converged", replay.converged},
        {"status", replay.status},
        {"iterations", replay.iterations},
        {"residual", replay.residual}};
  }
  return output;
}

struct RouteSearchPlanBaseline {
  int route_id{0};
  int station_id{0};
  double travel_cost{0.0};
  std::vector<double> station_arrivals;
};

struct RouteSearchScenario {
  hacdcpf::io::SiouxFallsIEEE123Case data;
  hacdcpf::analysis::TyphoonFaultSequenceResult typhoon;
  std::vector<std::vector<RouteSearchPlanBaseline>> plans;
  hacdcpf::evpt::CTMForwardResult resource_reference;
};

struct RouteSearchScenarioValue {
  double extra_travel_cost{0.0};
  double source_shortfall_cost{0.0};
  double recovery_cost{0.0};
  double raw_recovery_delta_mwh{0.0};
  std::vector<double> group_extra_travel_cost;
  double conservation_error_vehicles{0.0};
};

struct RouteSearchEvaluation {
  double value{std::numeric_limits<double>::infinity()};
  double baseline_travel_cost{0.0};
  double extra_travel_cost{0.0};
  double source_shortfall_cost{0.0};
  double recovery_cost{0.0};
  double minimum_raw_recovery_delta_mwh{
      std::numeric_limits<double>::infinity()};
  double maximum_conservation_error_vehicles{0.0};
  std::vector<double> group_extra_travel_cost;
};

struct RouteSearchNodeBound {
  double value{std::numeric_limits<double>::infinity()};
  std::vector<int> completion;
  RouteSearchEvaluation evaluation;
};

struct RouteSearchCounters {
  int highs_node_solves{0};
  int partial_ctm_calls{0};
  int full_ctm_calls{0};
  int node_recovery_pairs{0};
  int full_recovery_pairs{0};
  int resilience_assessment_calls{0};
  int precomputation_ctm_calls{0};
};

std::string route_assignment_key(const std::vector<int>& assignment) {
  std::string key;
  for (int plan : assignment) {
    if (!key.empty()) key.push_back('-');
    key += std::to_string(plan);
  }
  return key;
}

int route_search_group(
    const hacdcpf::io::SiouxFallsIEEE123Case& data,
    int origin, int destination) {
  for (int group = 0;
       group < static_cast<int>(data.selected_od.size()); ++group) {
    const auto& od = data.selected_od[static_cast<std::size_t>(group)];
    if (od.origin_node == origin && od.destination_node == destination) {
      return group;
    }
  }
  return -1;
}

hacdcpf::evpt::CTMForwardAssignment route_search_assignment(
    const hacdcpf::io::SiouxFallsIEEE123Case& data,
    const std::vector<int>& fixed_choice) {
  hacdcpf::evpt::CTMForwardAssignment assignment;
  for (const auto& demand : data.problem.demands) {
    const int group = route_search_group(
        data, demand.origin_node, demand.destination_node);
    if (group < 0 || group >= static_cast<int>(fixed_choice.size())) continue;
    const int choice = fixed_choice[static_cast<std::size_t>(group)];
    if (choice < 0) continue;
    const auto& routes =
        data.selected_od[static_cast<std::size_t>(group)].ev_route_indices;
    if (choice >= static_cast<int>(routes.size())) {
      throw std::runtime_error("route-search choice is outside the plan set");
    }
    assignment.ev_route_flow[demand.index][routes[static_cast<std::size_t>(choice)]] =
        demand.vehicles;
  }
  for (const auto& demand : data.problem.icv_demands) {
    if (demand.candidate_route_indices.empty()) continue;
    const double share = demand.vehicles /
        static_cast<double>(demand.candidate_route_indices.size());
    for (int route : demand.candidate_route_indices) {
      assignment.icv_route_flow[demand.index][route] = share;
    }
  }
  return assignment;
}

double route_search_group_travel_cost(
    const hacdcpf::io::SiouxFallsIEEE123Case& data,
    const hacdcpf::evpt::CTMForwardResult& result,
    int group, int choice) {
  const auto& od = data.selected_od.at(static_cast<std::size_t>(group));
  const int route_id = od.ev_route_indices.at(static_cast<std::size_t>(choice));
  double requested = 0.0;
  double weighted_departure_hr = 0.0;
  double value_of_time = -1.0;
  for (const auto& demand : data.problem.demands) {
    if (demand.origin_node != od.origin_node ||
        demand.destination_node != od.destination_node) {
      continue;
    }
    requested += demand.vehicles;
    weighted_departure_hr += demand.vehicles * demand.departure_step *
                             data.options.time_step_hr;
    if (value_of_time < 0.0) value_of_time = demand.value_of_time_per_hr;
    if (std::abs(value_of_time - demand.value_of_time_per_hr) > 1e-9) {
      throw std::runtime_error(
          "route-search group has heterogeneous values of travel time");
    }
  }
  if (requested <= 1e-12) return 0.0;
  const auto found = result.simulation.route_terminal_arrivals.find(route_id);
  double arrived = 0.0;
  double weighted_arrival_hr = 0.0;
  if (found != result.simulation.route_terminal_arrivals.end()) {
    for (std::size_t step = 0; step < found->second.size(); ++step) {
      const double vehicles = std::max(0.0, found->second[step]);
      arrived += vehicles;
      weighted_arrival_hr += vehicles * static_cast<double>(step + 1) *
                             result.simulation.dt_ctm_hr;
    }
  }
  const double unavailable_arrival_hr =
      data.options.num_steps * data.options.time_step_hr +
      data.options.time_step_hr;
  weighted_arrival_hr += std::max(0.0, requested - arrived) *
                         unavailable_arrival_hr;
  const double vehicle_hours =
      std::max(0.0, weighted_arrival_hr - weighted_departure_hr);
  return std::max(0.0, value_of_time) * vehicle_hours;
}

std::vector<double> earliest_arrival_envelope(
    const std::vector<RouteSearchPlanBaseline>& plans,
    const std::vector<int>& allowed) {
  if (allowed.empty()) return {};
  std::size_t steps = 0;
  for (int choice : allowed) {
    steps = std::max(
        steps, plans.at(static_cast<std::size_t>(choice)).station_arrivals.size());
  }
  std::vector<double> best_cumulative(steps, 0.0);
  for (int choice : allowed) {
    const auto& profile =
        plans.at(static_cast<std::size_t>(choice)).station_arrivals;
    double cumulative = 0.0;
    for (std::size_t step = 0; step < steps; ++step) {
      if (step < profile.size()) cumulative += std::max(0.0, profile[step]);
      best_cumulative[step] = std::max(best_cumulative[step], cumulative);
    }
  }
  std::vector<double> envelope(steps, 0.0);
  double previous = 0.0;
  for (std::size_t step = 0; step < steps; ++step) {
    envelope[step] = std::max(0.0, best_cumulative[step] - previous);
    previous = best_cumulative[step];
  }
  return envelope;
}

void add_arrival_profile(std::vector<double>& target,
                         const std::vector<double>& source) {
  if (target.size() < source.size()) target.resize(source.size(), 0.0);
  for (std::size_t step = 0; step < source.size(); ++step) {
    target[step] += source[step];
  }
}

RouteSearchScenario build_route_search_scenario(
    const hacdcpf::io::SiouxFallsIEEE123Options& case_options,
    unsigned int seed, bool apply_typhoon,
    double typhoon_delta_p_hpa, double road_drainage_mm_hr,
    double road_flood_closure_mm, RouteSearchCounters& counters) {
  RouteSearchScenario scenario;
  scenario.data = hacdcpf::io::build_sioux_falls_ieee123_case(case_options);
  if (apply_typhoon) {
    hacdcpf::analysis::TyphoonScenarioOptions hazard_options;
    hazard_options.horizon_hours = std::max(
        8, static_cast<int>(std::ceil(
               case_options.num_steps * case_options.time_step_hr)) + 2);
    hazard_options.time_step_hr = 1.0;
    hazard_options.seed = seed;
    hazard_options.stochastic = false;
    hazard_options.initial_latitude = 22.30;
    hazard_options.initial_longitude = 113.55;
    hazard_options.initial_heading_deg = 0.0;
    hazard_options.initial_translation_speed_kmph = 18.0;
    hazard_options.initial_delta_p_hpa = typhoon_delta_p_hpa;
    hazard_options.initial_rmw_km = 40.0;
    hazard_options.fallback_case_center_latitude = 22.75;
    hazard_options.fallback_case_center_longitude = 113.55;
    scenario.typhoon = hacdcpf::analysis::generate_typhoon_fault_sequence(
        scenario.data.problem.system, hazard_options);

    hacdcpf::analysis::TyphoonTrafficImpactOptions road_options;
    road_options.num_steps = case_options.num_steps;
    road_options.time_step_hr = case_options.time_step_hr;
    road_options.coordinate_mode =
        hacdcpf::analysis::TyphoonTrafficCoordinateMode::AffineToStudyArea;
    road_options.study_center_latitude =
        hazard_options.fallback_case_center_latitude;
    road_options.study_center_longitude =
        hazard_options.fallback_case_center_longitude;
    road_options.drainage_rate_mm_hr = road_drainage_mm_hr;
    road_options.flood_closure_depth_mm = road_flood_closure_mm;
    const auto impact = hacdcpf::analysis::apply_typhoon_traffic_impact(
        scenario.data.problem.traffic, scenario.typhoon.track, road_options);
    scenario.data.problem.traffic = impact.impacted_traffic;
  }

  hacdcpf::evpt::EVPowerTrafficOptions ev_options;
  ev_options.num_steps = case_options.num_steps;
  ev_options.time_step_hr = case_options.time_step_hr;
  hacdcpf::evpt::CTMOptions ctm_options;
  ctm_options.route_specific_cells = true;
  ctm_options.record_cell_history = false;
  ctm_options.dt_ctm_hr = case_options.time_step_hr /
      static_cast<double>(case_options.ctm_substeps_per_power_step);

  auto free_flow_problem = scenario.data.problem;
  constexpr double kNonbindingCapacityMultiplier = 100.0;
  for (auto& link : free_flow_problem.traffic.links) {
    link.capacity_veh_per_hr *= kNonbindingCapacityMultiplier;
    link.jam_vehicles *= kNonbindingCapacityMultiplier;
    for (double& capacity : link.capacity_profile_veh_per_hr) {
      capacity *= kNonbindingCapacityMultiplier;
    }
  }

  const int groups = static_cast<int>(scenario.data.selected_od.size());
  scenario.plans.resize(static_cast<std::size_t>(groups));
  for (int group = 0; group < groups; ++group) {
    const auto& od = scenario.data.selected_od[static_cast<std::size_t>(group)];
    const int plan_count = static_cast<int>(od.ev_route_indices.size());
    scenario.plans[static_cast<std::size_t>(group)].resize(
        static_cast<std::size_t>(plan_count));
    for (int choice = 0; choice < plan_count; ++choice) {
      std::vector<int> fixed(static_cast<std::size_t>(groups), -1);
      fixed[static_cast<std::size_t>(group)] = choice;
      auto assignment = route_search_assignment(scenario.data, fixed);
      assignment.icv_route_flow.clear();
      const auto result = hacdcpf::evpt::simulate_ev_power_traffic_ctm_forward(
          free_flow_problem, assignment, ev_options, ctm_options);
      ++counters.precomputation_ctm_calls;
      if (!result.valid || result.source_entry_shortfall_vehicles > 1e-6) {
        throw std::runtime_error(
            "route-search free-flow plan precomputation failed");
      }
      auto& baseline = scenario.plans[static_cast<std::size_t>(group)]
                                      [static_cast<std::size_t>(choice)];
      baseline.route_id = od.ev_route_indices[static_cast<std::size_t>(choice)];
      const auto route_it = std::find_if(
          scenario.data.problem.routes.begin(),
          scenario.data.problem.routes.end(),
          [&](const auto& route) { return route.index == baseline.route_id; });
      if (route_it == scenario.data.problem.routes.end() ||
          route_it->charging_stops.empty()) {
        throw std::runtime_error("route-search plan has no charging station");
      }
      baseline.station_id = route_it->charging_stops.front().station_id;
      baseline.travel_cost = route_search_group_travel_cost(
          scenario.data, result, group, choice);
      const auto profile =
          result.simulation.station_arrivals.find(baseline.station_id);
      if (profile != result.simulation.station_arrivals.end()) {
        baseline.station_arrivals = profile->second;
      }
      if (scenario.resource_reference.simulation.dt_ctm_hr <= 0.0) {
        scenario.resource_reference = result;
        for (auto& [station, arrivals] :
             scenario.resource_reference.simulation.station_arrivals) {
          (void)station;
          std::fill(arrivals.begin(), arrivals.end(), 0.0);
        }
      }
    }
  }

  for (int group = 0; group < groups; ++group) {
    const auto& plans = scenario.plans[static_cast<std::size_t>(group)];
    std::vector<int> all(plans.size());
    std::iota(all.begin(), all.end(), 0);
    const auto envelope = earliest_arrival_envelope(plans, all);
    const int station = plans.front().station_id;
    add_arrival_profile(
        scenario.resource_reference.simulation.station_arrivals[station],
        envelope);
  }
  scenario.resource_reference.valid = true;
  scenario.resource_reference.status = "route-search optimistic resource reference";
  return scenario;
}

std::pair<double, std::vector<int>> solve_route_node_with_highs(
    const std::vector<RouteSearchScenario>& scenarios,
    const std::vector<std::vector<int>>& allowed) {
  const int groups = static_cast<int>(allowed.size());
  std::vector<int> offset(static_cast<std::size_t>(groups + 1), 0);
  for (int group = 0; group < groups; ++group) {
    offset[static_cast<std::size_t>(group + 1)] =
        offset[static_cast<std::size_t>(group)] +
        static_cast<int>(scenarios.front().plans[static_cast<std::size_t>(group)].size());
  }
  const int columns = offset.back();
  std::vector<double> cost(static_cast<std::size_t>(columns), 0.0);
  std::vector<double> lower(static_cast<std::size_t>(columns), 0.0);
  std::vector<double> upper(static_cast<std::size_t>(columns), 0.0);
  std::vector<HighsInt> integrality(
      static_cast<std::size_t>(columns),
      static_cast<HighsInt>(HighsVarType::kInteger));
  std::vector<HighsInt> start(static_cast<std::size_t>(columns + 1), 0);
  std::vector<HighsInt> index(static_cast<std::size_t>(columns), 0);
  std::vector<double> value(static_cast<std::size_t>(columns), 1.0);
  for (int group = 0; group < groups; ++group) {
    std::set<int> permitted(allowed[static_cast<std::size_t>(group)].begin(),
                            allowed[static_cast<std::size_t>(group)].end());
    const int plans = offset[static_cast<std::size_t>(group + 1)] -
                      offset[static_cast<std::size_t>(group)];
    for (int choice = 0; choice < plans; ++choice) {
      const int column = offset[static_cast<std::size_t>(group)] + choice;
      for (const auto& scenario : scenarios) {
        cost[static_cast<std::size_t>(column)] +=
            scenario.plans[static_cast<std::size_t>(group)]
                          [static_cast<std::size_t>(choice)].travel_cost /
            static_cast<double>(scenarios.size());
      }
      upper[static_cast<std::size_t>(column)] = permitted.count(choice) ? 1.0 : 0.0;
      start[static_cast<std::size_t>(column)] = column;
      index[static_cast<std::size_t>(column)] = group;
    }
  }
  start[static_cast<std::size_t>(columns)] = columns;
  std::vector<double> row_lower(static_cast<std::size_t>(groups), 1.0);
  std::vector<double> row_upper(static_cast<std::size_t>(groups), 1.0);

  Highs writer;
  writer.setOptionValue("output_flag", false);
  writer.setOptionValue("log_to_console", false);
  const HighsStatus pass_status = writer.passModel(
      static_cast<HighsInt>(columns), static_cast<HighsInt>(groups),
      static_cast<HighsInt>(columns),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMinimize), 0.0, cost.data(),
      lower.data(), upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), integrality.data());
  if (pass_status == HighsStatus::kError) {
    throw std::runtime_error("HiGHS rejected the route-search node model");
  }

  static std::atomic<unsigned long long> solve_id{0};
  const auto id = ++solve_id;
  const auto base = std::filesystem::temp_directory_path() /
      ("sioux_route_highs_" + std::to_string(id));
  const auto model_path = std::filesystem::path(base.string() + ".mps");
  const auto solution_path = std::filesystem::path(base.string() + ".sol");
  const auto options_path = std::filesystem::path(base.string() + ".opt");
  const auto log_path = std::filesystem::path(base.string() + ".log");
  auto cleanup = [&]() {
    std::error_code error;
    std::filesystem::remove(model_path, error);
    std::filesystem::remove(solution_path, error);
    std::filesystem::remove(options_path, error);
    std::filesystem::remove(log_path, error);
  };
  if (writer.writeModel(model_path.string()) == HighsStatus::kError) {
    cleanup();
    throw std::runtime_error("HiGHS could not write the route-search node model");
  }
  {
    std::ofstream options_file(options_path);
    options_file << "mip_rel_gap = 1e-9\n";
    options_file << "threads = 1\n";
    options_file << "output_flag = false\n";
    options_file << "log_to_console = false\n";
  }
  const char* configured = std::getenv("HIGHS_EXECUTABLE");
  const std::string executable = configured != nullptr ? configured : "highs";
  const std::string command =
      "'" + executable + "' --model_file '" + model_path.string() +
      "' --solution_file '" + solution_path.string() +
      "' --options_file '" + options_path.string() + "' > '" +
      log_path.string() + "' 2>&1";
  const int return_code = std::system(command.c_str());
  std::vector<double> column_value;
  if (return_code == 0) {
    std::ifstream solution(solution_path);
    std::string line;
    bool in_columns = false;
    while (std::getline(solution, line)) {
      if (line.find("Columns") != std::string::npos ||
          line.find("columns") != std::string::npos) {
        in_columns = true;
        continue;
      }
      if (!in_columns) continue;
      if (line.find("Rows") != std::string::npos ||
          line.find("rows") != std::string::npos ||
          line.find("Dual") != std::string::npos ||
          line.find("dual") != std::string::npos) {
        break;
      }
      std::istringstream parser(line);
      std::vector<std::string> tokens;
      for (std::string token; parser >> token;) tokens.push_back(token);
      if (tokens.size() < 2) continue;
      try {
        column_value.push_back(std::stod(tokens.back()));
      } catch (const std::exception&) {
      }
    }
  }
  cleanup();
  if (column_value.size() < static_cast<std::size_t>(columns)) {
    throw std::runtime_error(
        "isolated HiGHS route-search node solve did not return a solution");
  }

  std::vector<int> completion(static_cast<std::size_t>(groups), -1);
  double objective = 0.0;
  for (int group = 0; group < groups; ++group) {
    for (int column = offset[static_cast<std::size_t>(group)];
         column < offset[static_cast<std::size_t>(group + 1)]; ++column) {
      if (column_value[static_cast<std::size_t>(column)] > 0.5) {
        completion[static_cast<std::size_t>(group)] =
            column - offset[static_cast<std::size_t>(group)];
        objective += cost[static_cast<std::size_t>(column)];
        break;
      }
    }
    if (completion[static_cast<std::size_t>(group)] < 0) {
      throw std::runtime_error("HiGHS route-search completion is incomplete");
    }
  }
  return {objective, completion};
}

RouteSearchScenarioValue evaluate_route_search_scenario(
    const RouteSearchScenario& scenario,
    const std::vector<int>& fixed_choice,
    const std::vector<std::vector<int>>& allowed,
    double recovery_voll_per_mwh,
    double source_shortfall_per_vehicle) {
  hacdcpf::evpt::EVPowerTrafficOptions ev_options;
  ev_options.num_steps = scenario.data.options.num_steps;
  ev_options.time_step_hr = scenario.data.options.time_step_hr;
  hacdcpf::evpt::CTMOptions ctm_options;
  ctm_options.route_specific_cells = true;
  ctm_options.record_cell_history = false;
  ctm_options.dt_ctm_hr = scenario.data.options.time_step_hr /
      static_cast<double>(scenario.data.options.ctm_substeps_per_power_step);
  const auto assignment = route_search_assignment(scenario.data, fixed_choice);
  const auto partial = hacdcpf::evpt::simulate_ev_power_traffic_ctm_forward(
      scenario.data.problem, assignment, ev_options, ctm_options);
  if (!partial.valid) {
    throw std::runtime_error("route-search partial CTM failed: " + partial.status);
  }

  RouteSearchScenarioValue value;
  value.group_extra_travel_cost.assign(fixed_choice.size(), 0.0);
  for (int group = 0; group < static_cast<int>(fixed_choice.size()); ++group) {
    const int choice = fixed_choice[static_cast<std::size_t>(group)];
    if (choice < 0) continue;
    const double actual = route_search_group_travel_cost(
        scenario.data, partial, group, choice);
    const double baseline =
        scenario.plans[static_cast<std::size_t>(group)]
                      [static_cast<std::size_t>(choice)].travel_cost;
    const double extra = std::max(0.0, actual - baseline);
    value.group_extra_travel_cost[static_cast<std::size_t>(group)] = extra;
    value.extra_travel_cost += extra;
  }
  value.source_shortfall_cost =
      source_shortfall_per_vehicle * partial.source_entry_shortfall_vehicles;
  value.conservation_error_vehicles =
      std::abs(partial.simulation.vehicle_conservation_error_vehicles);

  auto hybrid = partial;
  for (int group = 0; group < static_cast<int>(fixed_choice.size()); ++group) {
    if (fixed_choice[static_cast<std::size_t>(group)] >= 0) continue;
    const auto& plans = scenario.plans[static_cast<std::size_t>(group)];
    const auto envelope = earliest_arrival_envelope(
        plans, allowed[static_cast<std::size_t>(group)]);
    add_arrival_profile(
        hybrid.simulation.station_arrivals[plans.front().station_id], envelope);
  }
  const json comparison = run_typhoon_resilience_timing_comparison(
      scenario.data, scenario.resource_reference, hybrid, scenario.typhoon);
  const double reference_shed =
      comparison.at("free_flow").at("total_shed_mwh").get<double>();
  const double hybrid_shed =
      comparison.at("ctm").at("total_shed_mwh").get<double>();
  value.raw_recovery_delta_mwh = hybrid_shed - reference_shed;
  value.recovery_cost = recovery_voll_per_mwh *
      std::max(0.0, value.raw_recovery_delta_mwh);
  return value;
}

RouteSearchEvaluation aggregate_route_search_evaluation(
    const std::vector<RouteSearchScenario>& scenarios,
    const std::vector<int>& fixed_choice,
    const std::vector<std::vector<int>>& allowed,
    double baseline_travel_cost,
    double recovery_voll_per_mwh,
    double source_shortfall_per_vehicle,
    int threads) {
  std::vector<RouteSearchScenarioValue> values(scenarios.size());
  const int worker_count = std::max(1, threads);
  for (std::size_t begin = 0; begin < scenarios.size();
       begin += static_cast<std::size_t>(worker_count)) {
    const std::size_t end = std::min(
        scenarios.size(), begin + static_cast<std::size_t>(worker_count));
    std::vector<std::future<RouteSearchScenarioValue>> futures;
    for (std::size_t scenario = begin; scenario < end; ++scenario) {
      futures.push_back(std::async(
          std::launch::async,
          [&, scenario]() {
            return evaluate_route_search_scenario(
                scenarios[scenario], fixed_choice, allowed,
                recovery_voll_per_mwh, source_shortfall_per_vehicle);
          }));
    }
    for (std::size_t local = 0; local < futures.size(); ++local) {
      values[begin + local] = futures[local].get();
    }
  }

  RouteSearchEvaluation result;
  result.baseline_travel_cost = baseline_travel_cost;
  result.extra_travel_cost = 0.0;
  result.source_shortfall_cost = 0.0;
  result.recovery_cost = 0.0;
  result.group_extra_travel_cost.assign(fixed_choice.size(), 0.0);
  for (const auto& value : values) {
    const double probability = 1.0 / static_cast<double>(values.size());
    result.extra_travel_cost += probability * value.extra_travel_cost;
    result.source_shortfall_cost += probability * value.source_shortfall_cost;
    result.recovery_cost += probability * value.recovery_cost;
    result.minimum_raw_recovery_delta_mwh = std::min(
        result.minimum_raw_recovery_delta_mwh,
        value.raw_recovery_delta_mwh);
    result.maximum_conservation_error_vehicles = std::max(
        result.maximum_conservation_error_vehicles,
        value.conservation_error_vehicles);
    for (std::size_t group = 0;
         group < value.group_extra_travel_cost.size(); ++group) {
      result.group_extra_travel_cost[group] +=
          probability * value.group_extra_travel_cost[group];
    }
  }
  result.value = result.baseline_travel_cost + result.extra_travel_cost +
                 result.source_shortfall_cost + result.recovery_cost;
  return result;
}

double route_search_baseline_cost(
    const std::vector<RouteSearchScenario>& scenarios,
    const std::vector<int>& assignment) {
  double value = 0.0;
  for (const auto& scenario : scenarios) {
    for (int group = 0; group < static_cast<int>(assignment.size()); ++group) {
      value += scenario.plans[static_cast<std::size_t>(group)]
                             [static_cast<std::size_t>(assignment[static_cast<std::size_t>(group)])]
                                 .travel_cost /
               static_cast<double>(scenarios.size());
    }
  }
  return value;
}

RouteSearchNodeBound calculate_route_search_node_bound(
    const std::vector<RouteSearchScenario>& scenarios,
    const std::vector<std::vector<int>>& allowed,
    double recovery_voll_per_mwh,
    double source_shortfall_per_vehicle,
    int threads, RouteSearchCounters& counters) {
  auto [baseline, completion] =
      solve_route_node_with_highs(scenarios, allowed);
  ++counters.highs_node_solves;
  std::vector<int> fixed(allowed.size(), -1);
  for (std::size_t group = 0; group < allowed.size(); ++group) {
    if (allowed[group].size() == 1) fixed[group] = allowed[group].front();
  }
  RouteSearchNodeBound bound;
  bound.completion = std::move(completion);
  bound.evaluation = aggregate_route_search_evaluation(
      scenarios, fixed, allowed, baseline, recovery_voll_per_mwh,
      source_shortfall_per_vehicle, threads);
  bound.value = bound.evaluation.value;
  counters.partial_ctm_calls += static_cast<int>(scenarios.size());
  counters.node_recovery_pairs += static_cast<int>(scenarios.size());
  counters.resilience_assessment_calls +=
      2 * static_cast<int>(scenarios.size());
  return bound;
}

RouteSearchEvaluation evaluate_route_search_assignment(
    const std::vector<RouteSearchScenario>& scenarios,
    const std::vector<int>& assignment,
    double recovery_voll_per_mwh,
    double source_shortfall_per_vehicle,
    int threads, RouteSearchCounters& counters) {
  std::vector<std::vector<int>> singleton(assignment.size());
  for (std::size_t group = 0; group < assignment.size(); ++group) {
    singleton[group] = {assignment[group]};
  }
  const double baseline = route_search_baseline_cost(scenarios, assignment);
  auto result = aggregate_route_search_evaluation(
      scenarios, assignment, singleton, baseline, recovery_voll_per_mwh,
      source_shortfall_per_vehicle, threads);
  counters.full_ctm_calls += static_cast<int>(scenarios.size());
  counters.full_recovery_pairs += static_cast<int>(scenarios.size());
  counters.resilience_assessment_calls +=
      2 * static_cast<int>(scenarios.size());
  return result;
}

json run_route_committed_search(const ExperimentOptions& options) {
  if (options.search_scenarios <= 0 || options.search_threads <= 0 ||
      options.search_max_nodes <= 0 || options.search_relative_gap < 0.0 ||
      options.search_recovery_voll_per_mwh < 0.0 ||
      options.search_source_shortfall_per_vehicle < 0.0) {
    throw std::invalid_argument("route-search options are outside their range");
  }
  const auto total_started = std::chrono::steady_clock::now();
  RouteSearchCounters counters;
  std::vector<RouteSearchScenario> scenarios;
  scenarios.reserve(static_cast<std::size_t>(options.search_scenarios));
  const auto preprocessing_started = std::chrono::steady_clock::now();
  for (int scenario = 0; scenario < options.search_scenarios; ++scenario) {
    scenarios.push_back(build_route_search_scenario(
        options.case_options, options.typhoon_seed +
            static_cast<unsigned int>(scenario),
        options.typhoon_road_impact, options.typhoon_delta_p_hpa,
        options.road_drainage_mm_hr, options.road_flood_closure_mm, counters));
  }
  const double preprocessing_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    preprocessing_started).count();
  const int groups = static_cast<int>(scenarios.front().plans.size());
  std::vector<std::vector<int>> all(static_cast<std::size_t>(groups));
  unsigned long long combinations = 1;
  for (int group = 0; group < groups; ++group) {
    const int plans = static_cast<int>(
        scenarios.front().plans[static_cast<std::size_t>(group)].size());
    all[static_cast<std::size_t>(group)].resize(static_cast<std::size_t>(plans));
    std::iota(all[static_cast<std::size_t>(group)].begin(),
              all[static_cast<std::size_t>(group)].end(), 0);
    combinations *= static_cast<unsigned long long>(plans);
  }

  struct SearchNode {
    std::vector<std::vector<int>> allowed;
    RouteSearchNodeBound bound;
    unsigned long long combinations{0};
  };
  auto node_combinations = [](const auto& allowed) {
    unsigned long long count = 1;
    for (const auto& plans : allowed) {
      count *= static_cast<unsigned long long>(plans.size());
    }
    return count;
  };
  auto all_fixed = [](const auto& allowed) {
    return std::all_of(allowed.begin(), allowed.end(),
                       [](const auto& plans) { return plans.size() == 1; });
  };

  const auto search_started = std::chrono::steady_clock::now();
  SearchNode root;
  root.allowed = all;
  root.bound = calculate_route_search_node_bound(
      scenarios, root.allowed, options.search_recovery_voll_per_mwh,
      options.search_source_shortfall_per_vehicle, options.search_threads,
      counters);
  root.combinations = combinations;
  std::vector<SearchNode> active{std::move(root)};
  std::unordered_map<std::string, RouteSearchEvaluation> full_cache;
  double incumbent = std::numeric_limits<double>::infinity();
  std::vector<int> incumbent_assignment;
  int generated_nodes = 1;
  int processed_nodes = 0;
  int pruned_nodes = 0;
  unsigned long long pruned_combinations = 0;
  double maximum_bound_violation = 0.0;
  double minimum_child_strengthening =
      std::numeric_limits<double>::infinity();
  bool bound_audit_passed = true;
  bool stopped_by_gap = false;
  json convergence = json::array();

  auto global_lower_bound = [&]() {
    if (active.empty()) return incumbent;
    const double active_lower = std::min_element(
        active.begin(), active.end(),
        [](const SearchNode& lhs, const SearchNode& rhs) {
          return lhs.bound.value < rhs.bound.value;
        })->bound.value;
    return std::isfinite(incumbent)
        ? std::min(incumbent, active_lower)
        : active_lower;
  };
  auto relative_gap = [&](double lower_bound) {
    if (!std::isfinite(incumbent) || !std::isfinite(lower_bound)) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, incumbent - lower_bound) /
           std::max(1.0, std::abs(incumbent));
  };
  auto record_convergence = [&]() {
    const double lower = global_lower_bound();
    convergence.push_back({
        {"processed_nodes", processed_nodes},
        {"active_nodes", active.size()},
        {"lower_bound", lower},
        {"upper_bound", incumbent},
        {"relative_gap", relative_gap(lower)},
        {"full_plan_evaluations", full_cache.size()}});
  };

  while (!active.empty() && processed_nodes < options.search_max_nodes) {
    auto best = std::min_element(
        active.begin(), active.end(),
        [](const SearchNode& lhs, const SearchNode& rhs) {
          return lhs.bound.value < rhs.bound.value;
        });
    SearchNode node = std::move(*best);
    active.erase(best);
    ++processed_nodes;
    if (node.bound.value >= incumbent - 1e-8) {
      ++pruned_nodes;
      pruned_combinations += node.combinations;
      if (processed_nodes == 1 || processed_nodes % 10 == 0) record_convergence();
      continue;
    }

    const std::string candidate_key =
        route_assignment_key(node.bound.completion);
    auto cached = full_cache.find(candidate_key);
    if (cached == full_cache.end()) {
      RouteSearchEvaluation evaluation;
      if (all_fixed(node.allowed)) {
        evaluation = node.bound.evaluation;
      } else {
        evaluation = evaluate_route_search_assignment(
            scenarios, node.bound.completion,
            options.search_recovery_voll_per_mwh,
            options.search_source_shortfall_per_vehicle,
            options.search_threads, counters);
      }
      cached = full_cache.emplace(candidate_key, std::move(evaluation)).first;
    }
    const double violation = node.bound.value - cached->second.value;
    maximum_bound_violation = std::max(maximum_bound_violation, violation);
    if (violation > 1e-6 ||
        cached->second.minimum_raw_recovery_delta_mwh < -1e-8) {
      bound_audit_passed = false;
      break;
    }
    if (cached->second.value < incumbent) {
      incumbent = cached->second.value;
      incumbent_assignment = node.bound.completion;
    }

    if (all_fixed(node.allowed)) {
      if (std::abs(node.bound.value - cached->second.value) > 1e-6) {
        bound_audit_passed = false;
        break;
      }
      if (processed_nodes == 1 || processed_nodes % 10 == 0) record_convergence();
      continue;
    }

    int branch_group = -1;
    double branch_score = -1.0;
    for (int group = 0; group < groups; ++group) {
      if (node.allowed[static_cast<std::size_t>(group)].size() == 1) continue;
      double vehicles = 0.0;
      const auto& od = scenarios.front().data.selected_od[
          static_cast<std::size_t>(group)];
      for (const auto& demand : scenarios.front().data.problem.demands) {
        if (demand.origin_node == od.origin_node &&
            demand.destination_node == od.destination_node) {
          vehicles += demand.vehicles;
        }
      }
      const double score =
          (cached->second.group_extra_travel_cost[static_cast<std::size_t>(group)] +
           1e-6 * vehicles) *
          static_cast<double>(
              node.allowed[static_cast<std::size_t>(group)].size() - 1);
      if (score > branch_score) {
        branch_score = score;
        branch_group = group;
      }
    }
    if (branch_group < 0) {
      bound_audit_passed = false;
      break;
    }

    for (int choice : node.allowed[static_cast<std::size_t>(branch_group)]) {
      SearchNode child;
      child.allowed = node.allowed;
      child.allowed[static_cast<std::size_t>(branch_group)] = {choice};
      child.bound = calculate_route_search_node_bound(
          scenarios, child.allowed, options.search_recovery_voll_per_mwh,
          options.search_source_shortfall_per_vehicle, options.search_threads,
          counters);
      child.combinations = node_combinations(child.allowed);
      ++generated_nodes;
      minimum_child_strengthening = std::min(
          minimum_child_strengthening, child.bound.value - node.bound.value);
      if (child.bound.value + 1e-6 < node.bound.value) {
        bound_audit_passed = false;
        break;
      }
      if (child.bound.value >= incumbent - 1e-8) {
        ++pruned_nodes;
        pruned_combinations += child.combinations;
      } else {
        active.push_back(std::move(child));
      }
    }
    if (!bound_audit_passed) break;
    if (processed_nodes == 1 || processed_nodes % 10 == 0) record_convergence();
    const double lower = global_lower_bound();
    if (std::isfinite(incumbent) &&
        relative_gap(lower) <= options.search_relative_gap + 1e-12) {
      stopped_by_gap = !active.empty();
      break;
    }
  }
  record_convergence();
  const double search_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    search_started).count();
  const double total_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    total_started).count();
  const double final_lower_bound = global_lower_bound();
  const double final_relative_gap = relative_gap(final_lower_bound);
  const bool node_limit_reached =
      !active.empty() && processed_nodes >= options.search_max_nodes;
  const bool optimality_proven = bound_audit_passed && !node_limit_reached &&
      (active.empty() || stopped_by_gap) &&
      final_relative_gap <= options.search_relative_gap + 1e-12;
  unsigned long long bounded_remaining_combinations = 0;
  for (const auto& node : active) {
    bounded_remaining_combinations += node.combinations;
  }

  json exhaustive_validation;
  if (options.search_exhaustive) {
    if (combinations > 10000) {
      throw std::runtime_error(
          "route-search exhaustive validation is limited to 10,000 plans");
    }
    const auto exhaustive_started = std::chrono::steady_clock::now();
    RouteSearchCounters exhaustive_counters;
    double exhaustive_best = std::numeric_limits<double>::infinity();
    std::vector<int> exhaustive_assignment(static_cast<std::size_t>(groups), 0);
    std::vector<int> candidate(static_cast<std::size_t>(groups), 0);
    unsigned long long evaluated = 0;
    std::function<void(int)> enumerate = [&](int group) {
      if (group == groups) {
        const auto value = evaluate_route_search_assignment(
            scenarios, candidate, options.search_recovery_voll_per_mwh,
            options.search_source_shortfall_per_vehicle,
            options.search_threads, exhaustive_counters);
        ++evaluated;
        if (value.value < exhaustive_best) {
          exhaustive_best = value.value;
          exhaustive_assignment = candidate;
        }
        return;
      }
      for (int choice : all[static_cast<std::size_t>(group)]) {
        candidate[static_cast<std::size_t>(group)] = choice;
        enumerate(group + 1);
      }
    };
    enumerate(0);
    const double exhaustive_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - exhaustive_started).count();
    const bool matches = std::isfinite(incumbent) &&
        std::abs(exhaustive_best - incumbent) <= 1e-6;
    exhaustive_validation = {
        {"performed", true},
        {"evaluated_plans", evaluated},
        {"best_value", exhaustive_best},
        {"best_assignment", exhaustive_assignment},
        {"search_matches", matches},
        {"full_ctm_calls", exhaustive_counters.full_ctm_calls},
        {"recovery_pairs", exhaustive_counters.full_recovery_pairs},
        {"resilience_assessment_calls",
         exhaustive_counters.resilience_assessment_calls},
        {"runtime_seconds", exhaustive_seconds}};
    if (!matches) {
      throw std::runtime_error(
          "route-search result does not match exhaustive validation");
    }
  } else {
    exhaustive_validation = {{"performed", false}};
  }

  json seeds = json::array();
  for (int scenario = 0; scenario < options.search_scenarios; ++scenario) {
    seeds.push_back(options.typhoon_seed + static_cast<unsigned int>(scenario));
  }
  json assignment = json::array();
  for (int group = 0; group < groups; ++group) {
    const auto& od = scenarios.front().data.selected_od[
        static_cast<std::size_t>(group)];
    const int choice = incumbent_assignment.empty()
        ? -1
        : incumbent_assignment[static_cast<std::size_t>(group)];
    assignment.push_back({
        {"group", group},
        {"origin", od.origin_node},
        {"destination", od.destination_node},
        {"plan", choice},
        {"route_id", choice >= 0
             ? od.ev_route_indices[static_cast<std::size_t>(choice)]
             : -1}});
  }
  const auto incumbent_it =
      full_cache.find(route_assignment_key(incumbent_assignment));
  json incumbent_components;
  if (incumbent_it != full_cache.end()) {
    incumbent_components = {
        {"baseline_travel_cost", incumbent_it->second.baseline_travel_cost},
        {"extra_travel_cost", incumbent_it->second.extra_travel_cost},
        {"source_shortfall_cost", incumbent_it->second.source_shortfall_cost},
        {"recovery_cost", incumbent_it->second.recovery_cost},
        {"minimum_raw_recovery_delta_mwh",
         incumbent_it->second.minimum_raw_recovery_delta_mwh},
        {"maximum_conservation_error_vehicles",
         incumbent_it->second.maximum_conservation_error_vehicles}};
  }

  return {
      {"study", "Sioux Falls--IEEE 123 route-committed partial-loading search"},
      {"algorithm", "scenario-decomposed partial-loading search"},
      {"solver", "HiGHS isolated CLI node MILP"},
      {"hazard_applied", options.typhoon_road_impact},
      {"scenario_seeds", std::move(seeds)},
      {"scenario_threads", options.search_threads},
      {"route_groups", groups},
      {"plans_per_group", options.case_options.paths_per_od},
      {"plan_combinations", combinations},
      {"processed_nodes", processed_nodes},
      {"generated_nodes", generated_nodes},
      {"pruned_nodes", pruned_nodes},
      {"pruned_plan_combinations", pruned_combinations},
      {"bounded_remaining_combinations", bounded_remaining_combinations},
      {"distinct_full_plan_evaluations", full_cache.size()},
      {"highs_node_solves", counters.highs_node_solves},
      {"partial_ctm_calls", counters.partial_ctm_calls},
      {"full_ctm_calls", counters.full_ctm_calls},
      {"precomputation_ctm_calls", counters.precomputation_ctm_calls},
      {"node_recovery_pairs", counters.node_recovery_pairs},
      {"full_recovery_pairs", counters.full_recovery_pairs},
      {"resilience_assessment_calls", counters.resilience_assessment_calls},
      {"upper_bound", incumbent},
      {"lower_bound", final_lower_bound},
      {"relative_gap", final_relative_gap},
      {"optimality_proven", optimality_proven},
      {"benchmark_optimality_proven", optimality_proven},
      {"global_three_phase_acdc_optimality_proven", false},
      {"bound_audit_passed", bound_audit_passed},
      {"maximum_observed_bound_violation", maximum_bound_violation},
      {"minimum_child_bound_strengthening", minimum_child_strengthening},
      {"node_limit_reached", node_limit_reached},
      {"stopped_by_gap", stopped_by_gap},
      {"incumbent_assignment", std::move(assignment)},
      {"incumbent_components", std::move(incumbent_components)},
      {"convergence", std::move(convergence)},
      {"preprocessing_seconds", preprocessing_seconds},
      {"search_seconds", search_seconds},
      {"total_seconds", total_seconds},
      {"model_scope",
       "OD-level route commitments shared by four departure cohorts; fixed "
       "equal-route ICV background; fixed-turn route-specific CTM; "
       "arrival-monotone mobile-resource timing; AC sequential recovery "
       "comparison; three-phase AC/DC OPF is not solved inside this search"},
      {"validity",
       "Benchmark optimality is conditional on the fixed-turn CTM traffic-order "
       "property and monotonicity of the AC sequential recovery evaluator. It "
       "is reported only when the active-node lower bound closes the requested "
       "gap and the runtime bound audit passes; it is not global optimality for "
       "three-phase AC/DC restoration."},
      {"bound_audit_scope",
       options.search_exhaustive
           ? "All complete plans are enumerated and compared with the search."
           : "Generated-node monotonicity, evaluated completions, terminal "
             "nodes, and traffic conservation are checked at runtime; "
             "unevaluated complete plans are covered by the stated order "
             "assumption rather than exhaustive enumeration."},
      {"exhaustive_validation", std::move(exhaustive_validation)}};
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse_arguments(argc, argv);
    if (options.show_help) {
      print_usage(argv[0]);
      return 0;
    }
    if (options.route_search) {
      const json result = run_route_committed_search(options);
      std::cout << result.dump(2) << '\n';
      return result.value("optimality_proven", false) &&
                     result.value("bound_audit_passed", false)
                 ? 0
                 : 3;
    }
    if (!options.batch) {
      std::cout << run_experiment(
                       options.case_options, options.solve_sequence,
                       options.compact_sequence, options.traffic_only,
                       options.grid_admissibility,
                       options.typhoon_road_impact,
                       options.typhoon_seed,
                       options.typhoon_delta_p_hpa,
                       options.road_drainage_mm_hr,
                       options.road_flood_closure_mm)
                       .dump(2)
                << '\n';
      return 0;
    }

    json output;
    output["study"] =
        "Sioux Falls--IEEE 123 road-capacity sensitivity";
    output["power_model"] =
        options.traffic_only
            ? "not solved"
            : "three-phase-unbalanced-ac-dc-hybrid-opf";
    output["scenarios"] = json::array();
    for (double capacity_scale : {0.05, 0.08, 0.09, 0.20, 0.35}) {
      auto case_options = options.case_options;
      case_options.road_capacity_scale = capacity_scale;
      output["scenarios"].push_back(run_experiment(
          case_options, options.solve_sequence, true, options.traffic_only,
          options.grid_admissibility, options.typhoon_road_impact,
          options.typhoon_seed, options.typhoon_delta_p_hpa,
          options.road_drainage_mm_hr,
          options.road_flood_closure_mm));
    }
    std::cout << output.dump(2) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Sioux Falls--IEEE 123 experiment failed: "
              << error.what() << '\n';
    return 2;
  }
}
