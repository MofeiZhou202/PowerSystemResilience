#pragma once

#include <vector>

#include "hacdcpf/ev_power_traffic/simulation.hpp"

namespace hacdcpf::evpt {

struct CandidatePath {
  double free_flow_time_hr{0.0};
  std::vector<int> node_indices;
  std::vector<int> link_indices;
};

struct CandidateRouteGenerationOptions {
  int k_shortest_paths{3};
  bool add_no_stop_route{true};
  bool add_single_station_routes{true};
  bool add_two_station_routes{false};
  bool prune_by_soc{true};
  double default_charge_energy_kwh{20.0};
  int default_dwell_steps{2};
  double default_max_charge_kw{50.0};
};

/// Return at most k loopless paths, ordered by free-flow travel time.
std::vector<CandidatePath> k_shortest_loopless_paths(
    const TrafficGraph& graph,
    int origin_node,
    int destination_node,
    int k);

/// Populate candidate routes for demands whose candidate set is empty.
/// Charging stations are considered reachable when their bus ID equals a
/// traffic node ID.  Case builders with a non-identity mapping should attach
/// their charging stops explicitly after path generation.
EVPowerTrafficProblem generate_candidate_routes(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& options,
    const CandidateRouteGenerationOptions& generation_options = {});

}  // namespace hacdcpf::evpt
