#include "hacdcpf/io/tntp_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace hacdcpf::io {
namespace {

std::string trim(std::string value) {
  const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(),
              value.end());
  return value;
}

std::string uppercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::toupper(ch));
                 });
  return value;
}

std::ifstream open_file(const std::string& filepath) {
  std::ifstream stream(filepath);
  if (!stream) {
    throw std::runtime_error("TNTP parser: failed to open file: " + filepath);
  }
  return stream;
}

bool parse_metadata_line(const std::string& line,
                         std::map<std::string, std::string>& metadata) {
  const std::size_t left = line.find('<');
  const std::size_t right = line.find('>', left == std::string::npos ? 0 : left);
  if (left == std::string::npos || right == std::string::npos) return false;
  const std::string key = uppercase(trim(line.substr(left + 1, right - left - 1)));
  const std::string value = trim(line.substr(right + 1));
  metadata[key] = value;
  return true;
}

int metadata_int(const std::map<std::string, std::string>& metadata,
                 const std::string& key,
                 int fallback) {
  const auto it = metadata.find(key);
  if (it == metadata.end() || it->second.empty()) return fallback;
  try {
    return std::stoi(it->second);
  } catch (const std::exception&) {
    throw std::runtime_error("TNTP parser: invalid integer metadata <" + key + ">");
  }
}

double metadata_double(const std::map<std::string, std::string>& metadata,
                       const std::string& key,
                       double fallback) {
  const auto it = metadata.find(key);
  if (it == metadata.end() || it->second.empty()) return fallback;
  try {
    return std::stod(it->second);
  } catch (const std::exception&) {
    throw std::runtime_error("TNTP parser: invalid numeric metadata <" + key + ">");
  }
}

std::string strip_row_terminator(std::string line) {
  const std::size_t semicolon = line.find(';');
  if (semicolon != std::string::npos) line.erase(semicolon);
  std::replace(line.begin(), line.end(), '\t', ' ');
  return trim(line);
}

}  // namespace

double TNTPTripTable::total_flow() const {
  double total = 0.0;
  for (const auto& [origin, destinations] : demand) {
    (void)origin;
    for (const auto& [destination, value] : destinations) {
      (void)destination;
      total += value;
    }
  }
  return total;
}

double TNTPTripTable::flow(int origin, int destination) const {
  const auto origin_it = demand.find(origin);
  if (origin_it == demand.end()) return 0.0;
  const auto destination_it = origin_it->second.find(destination);
  return destination_it == origin_it->second.end() ? 0.0
                                                    : destination_it->second;
}

TNTPNetworkData parse_tntp_network(const std::string& filepath) {
  std::ifstream stream = open_file(filepath);
  TNTPNetworkData result;
  std::string line;
  std::set<std::pair<int, int>> directed_pairs;

  while (std::getline(stream, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '~') continue;
    if (parse_metadata_line(line, result.metadata)) continue;

    line = strip_row_terminator(line);
    if (line.empty() || !std::isdigit(static_cast<unsigned char>(line.front()))) {
      continue;
    }

    std::istringstream row(line);
    TNTPLinkRecord link;
    if (!(row >> link.from_node >> link.to_node >> link.capacity_veh_per_hr >>
          link.length >> link.free_flow_time >> link.b >> link.power >>
          link.speed >> link.toll >> link.link_type)) {
      throw std::runtime_error("TNTP parser: invalid network row: " + line);
    }
    if (link.from_node <= 0 || link.to_node <= 0 ||
        link.capacity_veh_per_hr < 0.0 || link.length < 0.0 ||
        link.free_flow_time < 0.0) {
      throw std::runtime_error("TNTP parser: invalid network values: " + line);
    }
    if (!directed_pairs.emplace(link.from_node, link.to_node).second) {
      throw std::runtime_error("TNTP parser: duplicate directed link " +
                               std::to_string(link.from_node) + "->" +
                               std::to_string(link.to_node));
    }
    result.links.push_back(link);
  }

  result.number_of_zones =
      metadata_int(result.metadata, "NUMBER OF ZONES", 0);
  result.number_of_nodes =
      metadata_int(result.metadata, "NUMBER OF NODES", 0);
  result.first_thru_node =
      metadata_int(result.metadata, "FIRST THRU NODE", 1);
  result.declared_number_of_links =
      metadata_int(result.metadata, "NUMBER OF LINKS", 0);
  if (result.links.empty()) {
    throw std::runtime_error("TNTP parser: network contains no links");
  }
  if (result.declared_number_of_links > 0 &&
      result.declared_number_of_links != static_cast<int>(result.links.size())) {
    throw std::runtime_error("TNTP parser: declared and parsed link counts differ");
  }
  return result;
}

std::vector<TNTPNodeRecord> parse_tntp_nodes(const std::string& filepath) {
  std::ifstream stream = open_file(filepath);
  std::vector<TNTPNodeRecord> result;
  std::set<int> ids;
  std::string line;

  while (std::getline(stream, line)) {
    line = strip_row_terminator(trim(line));
    if (line.empty() || line.front() == '~' ||
        !std::isdigit(static_cast<unsigned char>(line.front()))) {
      continue;
    }
    std::istringstream row(line);
    TNTPNodeRecord node;
    if (!(row >> node.index >> node.x >> node.y) || node.index <= 0) {
      throw std::runtime_error("TNTP parser: invalid node row: " + line);
    }
    if (!ids.insert(node.index).second) {
      throw std::runtime_error("TNTP parser: duplicate node " +
                               std::to_string(node.index));
    }
    result.push_back(node);
  }
  if (result.empty()) {
    throw std::runtime_error("TNTP parser: node file contains no nodes");
  }
  return result;
}

TNTPTripTable parse_tntp_trips(const std::string& filepath) {
  std::ifstream stream = open_file(filepath);
  TNTPTripTable result;
  std::string line;
  int current_origin = -1;
  const std::regex origin_pattern(R"(^\s*[Oo]rigin\s+([0-9]+))");
  const std::regex demand_pattern(
      R"(([0-9]+)\s*:\s*([-+]?(?:[0-9]*\.?[0-9]+)(?:[eE][-+]?[0-9]+)?))");

  while (std::getline(stream, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '~') continue;
    if (parse_metadata_line(line, result.metadata)) continue;

    std::smatch origin_match;
    if (std::regex_search(line, origin_match, origin_pattern)) {
      current_origin = std::stoi(origin_match[1].str());
      if (current_origin <= 0) {
        throw std::runtime_error("TNTP parser: invalid trip origin");
      }
      result.demand.try_emplace(current_origin);
      continue;
    }
    if (current_origin <= 0) continue;

    for (std::sregex_iterator it(line.begin(), line.end(), demand_pattern), end;
         it != end; ++it) {
      const int destination = std::stoi((*it)[1].str());
      const double value = std::stod((*it)[2].str());
      if (destination <= 0 || value < 0.0) {
        throw std::runtime_error("TNTP parser: invalid OD demand entry");
      }
      auto& destinations = result.demand[current_origin];
      if (!destinations.emplace(destination, value).second) {
        throw std::runtime_error("TNTP parser: duplicate OD demand entry");
      }
    }
  }

  result.number_of_zones =
      metadata_int(result.metadata, "NUMBER OF ZONES", 0);
  result.declared_total_od_flow =
      metadata_double(result.metadata, "TOTAL OD FLOW", 0.0);
  if (result.demand.empty()) {
    throw std::runtime_error("TNTP parser: trip table contains no origins");
  }
  if (result.number_of_zones > 0) {
    for (const auto& [origin, destinations] : result.demand) {
      if (origin > result.number_of_zones) {
        throw std::runtime_error("TNTP parser: trip origin exceeds declared zones");
      }
      for (const auto& [destination, value] : destinations) {
        (void)value;
        if (destination > result.number_of_zones) {
          throw std::runtime_error(
              "TNTP parser: trip destination exceeds declared zones");
        }
      }
    }
  }
  if (result.declared_total_od_flow > 0.0) {
    const double parsed_total = result.total_flow();
    const double tolerance =
        std::max(1e-6, 1e-8 * result.declared_total_od_flow);
    if (std::abs(parsed_total - result.declared_total_od_flow) > tolerance) {
      throw std::runtime_error(
          "TNTP parser: declared and parsed total OD flow differ");
    }
  }
  return result;
}

evpt::TrafficGraph make_traffic_graph(
    const TNTPNetworkData& network,
    const std::vector<TNTPNodeRecord>& nodes,
    const TNTPImportOptions& options) {
  if (options.miles_to_km <= 0.0 ||
      options.jam_to_critical_occupancy_ratio <= 1.0) {
    throw std::runtime_error("TNTP parser: invalid import options");
  }
  if (options.require_declared_counts && network.number_of_nodes > 0 &&
      network.number_of_nodes != static_cast<int>(nodes.size())) {
    throw std::runtime_error("TNTP parser: declared and parsed node counts differ");
  }

  evpt::TrafficGraph graph;
  std::set<int> node_ids;
  for (const auto& source : nodes) {
    node_ids.insert(source.index);
    evpt::TrafficNode node;
    node.index = source.index;
    node.name = "TNTP-" + std::to_string(source.index);
    node.x = source.x;
    node.y = source.y;
    graph.nodes.push_back(node);
  }

  int link_index = 1;
  for (const auto& source : network.links) {
    if (!node_ids.count(source.from_node) || !node_ids.count(source.to_node)) {
      throw std::runtime_error("TNTP parser: link endpoint is absent from node file");
    }
    evpt::TrafficLink link;
    link.index = link_index++;
    link.from_node = source.from_node;
    link.to_node = source.to_node;
    link.length_km = options.length_unit == TNTPLengthUnit::Miles
                         ? source.length * options.miles_to_km
                         : source.length;
    link.free_flow_time_hr = options.time_unit == TNTPTimeUnit::Minutes
                                 ? source.free_flow_time / 60.0
                                 : source.free_flow_time;
    link.capacity_veh_per_hr = source.capacity_veh_per_hr;
    link.alpha = source.b;
    link.beta = source.power;
    link.drive_energy_kwh_per_veh_km =
        options.drive_energy_kwh_per_veh_km;
    const double critical_occupancy =
        link.capacity_veh_per_hr * link.free_flow_time_hr;
    link.jam_vehicles = std::max(
        1.0, options.jam_to_critical_occupancy_ratio * critical_occupancy);
    graph.links.push_back(link);
  }
  return graph;
}

}  // namespace hacdcpf::io
