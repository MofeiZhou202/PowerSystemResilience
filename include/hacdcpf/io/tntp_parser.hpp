#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/ev_power_traffic/types.hpp"

namespace hacdcpf::io {

enum class TNTPTimeUnit { Minutes, Hours };
enum class TNTPLengthUnit { Kilometers, Miles };

struct TNTPLinkRecord {
  int from_node{0};
  int to_node{0};
  double capacity_veh_per_hr{0.0};
  double length{0.0};
  double free_flow_time{0.0};
  double b{0.0};
  double power{0.0};
  double speed{0.0};
  double toll{0.0};
  int link_type{0};
};

struct TNTPNodeRecord {
  int index{0};
  double x{0.0};
  double y{0.0};
};

struct TNTPNetworkData {
  int number_of_zones{0};
  int number_of_nodes{0};
  int first_thru_node{1};
  int declared_number_of_links{0};
  std::map<std::string, std::string> metadata;
  std::vector<TNTPLinkRecord> links;
};

struct TNTPTripTable {
  int number_of_zones{0};
  double declared_total_od_flow{0.0};
  std::map<std::string, std::string> metadata;
  std::map<int, std::map<int, double>> demand;

  double total_flow() const;
  double flow(int origin, int destination) const;
};

struct TNTPImportOptions {
  TNTPTimeUnit time_unit{TNTPTimeUnit::Minutes};
  TNTPLengthUnit length_unit{TNTPLengthUnit::Miles};
  double miles_to_km{1.609344};
  double jam_to_critical_occupancy_ratio{2.0};
  double drive_energy_kwh_per_veh_km{0.18};
  bool require_declared_counts{true};
};

TNTPNetworkData parse_tntp_network(const std::string& filepath);
std::vector<TNTPNodeRecord> parse_tntp_nodes(const std::string& filepath);
TNTPTripTable parse_tntp_trips(const std::string& filepath);

evpt::TrafficGraph make_traffic_graph(
    const TNTPNetworkData& network,
    const std::vector<TNTPNodeRecord>& nodes,
    const TNTPImportOptions& options = {});

}  // namespace hacdcpf::io
