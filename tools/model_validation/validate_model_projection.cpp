#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "hacdcpf/graph/power_system_graph.hpp"
#include "hacdcpf/model/unit_conversion.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
using namespace hacdcpf;

namespace {

ACBus ac_bus(int index, BusType type, double kv = 10.0) {
  ACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.base_kv = kv;
  return bus;
}

DCBus dc_bus(int index, DCBusType type, double kv = 0.75) {
  DCBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.base_kv = kv;
  return bus;
}

json merge_case() {
  HybridPowerSystem system;
  system.base_mva = system.ac.base_mva = 100.0;
  system.ac.buses = {ac_bus(101, BusType::SLACK), ac_bus(205, BusType::PQ),
                     ac_bus(309, BusType::PQ)};
  ACBranch ideal;
  ideal.index = 7;
  ideal.from_bus = 101;
  ideal.to_bus = 205;
  ideal.ideal_connectivity = true;
  ACBranch line;
  line.index = 8;
  line.from_bus = 205;
  line.to_bus = 309;
  line.r_pu = 0.01;
  line.x_pu = 0.05;
  system.ac.branches = {ideal, line};
  Load l1;
  l1.index = 1;
  l1.bus = 101;
  l1.p_mw = 12.0;
  Load l2;
  l2.index = 2;
  l2.bus = 205;
  l2.p_mw = 8.0;
  system.ac.loads = {l1, l2};
  Generator g1;
  g1.index = 1;
  g1.bus = 101;
  g1.pg_mw = 30.0;
  g1.is_slack = true;
  Generator g2;
  g2.index = 2;
  g2.bus = 205;
  g2.pg_mw = 10.0;
  system.ac.generators = {g1, g2};

  const double load_before = total_load_p_mw(system);
  double generation_before = 0.0;
  for (const auto& generator : system.ac.generators) generation_before += generator.pg_mw;
  HybridPowerSystem merged = system;
  merge_zero_impedance_buses(merged);
  if (!merged.bus_merge_map) throw std::runtime_error("merge case did not produce a BusMergeMap");
  const BusMergeMap& map = *merged.bus_merge_map;
  const int rep_position = map.ext_to_int.at(101);
  std::vector<double> voltage(static_cast<std::size_t>(map.n_merged), 0.98);
  voltage[static_cast<std::size_t>(rep_position)] = 1.02;
  std::vector<double> generation(static_cast<std::size_t>(map.n_merged), 0.0);
  std::vector<double> demand(static_cast<std::size_t>(map.n_merged), 0.0);
  generation[static_cast<std::size_t>(rep_position)] = 40.0;
  demand[static_cast<std::size_t>(rep_position)] = 20.0;
  const auto recovered_voltage = unproject_bus_vector(voltage, map, BusVectorSemantics::Intensive);
  const auto recovered_generation =
      unproject_bus_vector(generation, map, BusVectorSemantics::ExtensiveGeneration);
  const auto recovered_demand =
      unproject_bus_vector(demand, map, BusVectorSemantics::ExtensiveDemand);
  const int count_after_first = static_cast<int>(merged.ac.buses.size());
  merge_zero_impedance_buses(merged);

  double generation_after = 0.0;
  for (const auto& generator : merged.ac.generators) generation_after += generator.pg_mw;
  json participation_demand;
  json participation_generation;
  for (int id : {101, 205}) {
    participation_demand[std::to_string(id)] = map.extensive_participation.at(id);
    participation_generation[std::to_string(id)] = map.generation_participation.at(id);
  }
  return {{"authored_bus_ids", {101, 205, 309}},
          {"authored_bus_count", 3}, {"canonical_bus_count", map.n_merged},
          {"groups", map.groups}, {"load_before_mw", load_before},
          {"load_after_mw", total_load_p_mw(merged)},
          {"generation_before_mw", generation_before},
          {"generation_after_mw", generation_after},
          {"demand_participation", participation_demand},
          {"generation_participation", participation_generation},
          {"recovered_voltage_pu", recovered_voltage},
          {"recovered_generation_mw", recovered_generation},
          {"recovered_demand_mw", recovered_demand},
          {"idempotent_bus_count", static_cast<int>(merged.ac.buses.size()) == count_after_first}};
}

json same_id_case() {
  HybridPowerSystem system;
  system.base_mva = system.ac.base_mva = system.dc.base_mva = 100.0;
  system.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  system.dc.buses = {dc_bus(1, DCBusType::DC_V), dc_bus(2, DCBusType::DC_P)};
  ACBranch ac;
  ac.index = 1;
  ac.from_bus = 1;
  ac.to_bus = 2;
  ac.r_pu = 0.01;
  ac.x_pu = 0.05;
  system.ac.branches = {ac};
  DCBranch dc;
  dc.index = 1;
  dc.from_bus = 1;
  dc.to_bus = 2;
  dc.r_pu = 0.02;
  system.dc.branches = {dc};
  const auto graph = graph::build_power_system_graph(system);
  return {{"node_count", graph.node_count()},
          {"ac_1_node", graph.ac_node_idx(1)}, {"dc_1_node", graph.dc_node_idx(1)},
          {"ac_2_node", graph.ac_node_idx(2)}, {"dc_2_node", graph.dc_node_idx(2)},
          {"node_domains", {static_cast<int>(graph.nodes[graph.ac_node_idx(1)].domain),
                            static_cast<int>(graph.nodes[graph.dc_node_idx(1)].domain)}}};
}

json dead_island_case() {
  HybridPowerSystem system;
  system.base_mva = system.ac.base_mva = system.dc.base_mva = 100.0;
  system.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  ACBranch ac;
  ac.index = 1;
  ac.from_bus = 1;
  ac.to_bus = 2;
  ac.r_pu = 0.01;
  ac.x_pu = 0.05;
  system.ac.branches = {ac};
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  system.ac.external_grids = {grid};
  system.dc.buses = {dc_bus(1, DCBusType::DC_V), dc_bus(2, DCBusType::DC_P),
                     dc_bus(5, DCBusType::DC_P)};
  DCBranch dc;
  dc.index = 1;
  dc.from_bus = 1;
  dc.to_bus = 2;
  dc.r_pu = 0.02;
  system.dc.branches = {dc};
  const auto projected = project_to_canonical_models(system);
  if (!projected.projection_certificate) throw std::runtime_error("dead-island certificate absent");
  const auto& cert = *projected.projection_certificate;
  return {{"authored_dc_count", 3}, {"survivor_dc_count", projected.dc.buses.size()},
          {"authored_dead_id", 5},
          {"dead_ids", cert.dc_dead_bus_indices},
          {"prestrip_to_survivor", cert.dc_prestrip_to_survivor},
          {"recovered_voltage_pu", unproject_dc_bus_vector({1.0, 0.99}, cert)}};
}

json unit_case() {
  HybridPowerSystem system;
  system.base_mva = system.ac.base_mva = 10.0;
  system.ac.buses = {ac_bus(1, BusType::SLACK, 12.47), ac_bus(2, BusType::PQ, 12.47)};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.length_km = 5.0;
  branch.n_parallel = 2;
  branch.r_ohm_per_km = 0.2;
  branch.x_ohm_per_km = 0.4;
  system.ac.branches = {branch};
  const int first = convert_actual_to_per_unit(system);
  const double r = system.ac.branches.front().r_pu;
  const double x = system.ac.branches.front().x_pu;
  const int second = convert_actual_to_per_unit(system);
  return {{"base_kv", 12.47}, {"base_mva", 10.0}, {"length_km", 5.0},
          {"parallel", 2}, {"r_ohm_per_km", 0.2}, {"x_ohm_per_km", 0.4},
          {"r_pu", r}, {"x_pu", x}, {"first_conversion_count", first},
          {"second_conversion_count", second},
          {"values_unchanged", r == system.ac.branches.front().r_pu &&
                               x == system.ac.branches.front().x_pu}};
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("usage: validate_model_projection OUTPUT.json");
    json output = {{"schema", "hysim-model-projection-xref-v1"},
                   {"merge", merge_case()}, {"same_numeric_acdc_ids", same_id_case()},
                   {"dc_dead_island", dead_island_case()}, {"unit_conversion", unit_case()}};
    const fs::path path = argv[1];
    if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("cannot open output file: " + path.string());
    stream << output.dump(2) << '\n';
    std::cout << path.string() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "model projection evidence generation failed: " << e.what() << '\n';
    return 2;
  }
}
