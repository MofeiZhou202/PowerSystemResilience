#include "hacdcpf/ev_power_traffic/route_generation.hpp"

#include "evpt_internal.hpp"

namespace hacdcpf::evpt {

std::vector<CandidatePath> k_shortest_loopless_paths(
    const TrafficGraph& graph,
    int origin_node,
    int destination_node,
    int k) {
  std::vector<CandidatePath> result;
  for (const auto& path :
       yen_k_shortest_paths(graph, origin_node, destination_node, k)) {
    CandidatePath candidate;
    candidate.free_flow_time_hr = path.cost;
    candidate.node_indices = path.nodes;
    candidate.link_indices = path.links;
    result.push_back(std::move(candidate));
  }
  return result;
}

EVPowerTrafficProblem generate_candidate_routes(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& options,
    const CandidateRouteGenerationOptions& generation_options) {
  GenerateRoutesOptions internal;
  internal.K = generation_options.k_shortest_paths;
  internal.add_no_stop_route = generation_options.add_no_stop_route;
  internal.add_single_station_routes =
      generation_options.add_single_station_routes;
  internal.add_two_station_routes = generation_options.add_two_station_routes;
  internal.prune_by_soc = generation_options.prune_by_soc;
  internal.default_charge_energy_kwh =
      generation_options.default_charge_energy_kwh;
  internal.default_dwell_steps = generation_options.default_dwell_steps;
  internal.default_max_charge_kw =
      generation_options.default_max_charge_kw;
  return generate_candidate_routes_internal(problem, options, internal);
}

}  // namespace hacdcpf::evpt
