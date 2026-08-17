#include "runtime_api_v1.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"

namespace hacdcpf::server {
namespace {

using json = nlohmann::json;
using Clock = std::chrono::system_clock;

constexpr std::size_t kMaxSessions = 128;
constexpr std::size_t kMaxJobs = 4096;

std::string utc_now() {
  const auto now = Clock::now();
  const std::time_t value = Clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &value);
#else
  gmtime_r(&value, &tm);
#endif
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3)
      << std::setfill('0') << millis.count() << 'Z';
  return out.str();
}

std::string random_id(const char* prefix) {
  static thread_local std::mt19937_64 generator{std::random_device{}()};
  std::array<std::uint8_t, 16> bytes{};
  for (std::size_t i = 0; i < bytes.size(); i += 8) {
    const std::uint64_t value = generator();
    for (std::size_t j = 0; j < 8; ++j) {
      bytes[i + j] = static_cast<std::uint8_t>(value >> (j * 8));
    }
  }
  bytes[6] = static_cast<std::uint8_t>((bytes[6] & 0x0fU) | 0x40U);
  bytes[8] = static_cast<std::uint8_t>((bytes[8] & 0x3fU) | 0x80U);
  std::ostringstream out;
  out << prefix << '-';
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) out << '-';
    out << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(bytes[i]);
  }
  return out.str();
}

std::string model_etag(const std::string& session_id, std::uint64_t revision) {
  return "\"hysim-" + session_id + "-r" + std::to_string(revision) + "\"";
}

json parse_body(const httplib::Request& request) {
  if (request.body.empty()) return json::object();
  const json body = json::parse(request.body);
  if (!body.is_object()) {
    throw std::invalid_argument("request body must be a JSON object");
  }
  return body;
}

void set_json(httplib::Response& response, const json& body, int status = 200) {
  response.status = status;
  response.set_content(body.dump(), "application/json");
}

void set_error(httplib::Response& response, int status,
               const std::string& code, const std::string& message,
               const json& details = json::object()) {
  json body{{"schema", "hysim_error_v1"},
            {"error", code},
            {"message", message}};
  if (!details.empty()) body["details"] = details;
  set_json(response, body, status);
}

template <typename Function>
void guarded(httplib::Response& response, Function&& function) {
  try {
    function();
  } catch (const json::exception& error) {
    set_error(response, 400, "invalid_json", error.what());
  } catch (const std::invalid_argument& error) {
    set_error(response, 400, "invalid_request", error.what());
  } catch (const std::out_of_range& error) {
    set_error(response, 404, "chunk_not_found", error.what());
  } catch (const std::exception& error) {
    set_error(response, 500, "internal_error", error.what());
  }
}

json model_counts(const HybridPowerSystem& system) {
  return json{{"ac_buses", system.ac.buses.size()},
              {"ac_branches", system.ac.branches.size()},
              {"generators", system.ac.generators.size()},
              {"loads", system.ac.loads.size()},
              {"dc_buses", system.dc.buses.size()},
              {"dc_branches", system.dc.branches.size()},
              {"dc_loads", system.dc.loads.size()},
              {"vsc_converters", system.vsc_converters.size()},
              {"dcdc_converters", system.dc.dcdc_converters.size()},
              {"energy_routers", system.energy_routers.size()},
              {"three_phase_buses",
               system.three_phase_ac ? system.three_phase_ac->buses.size() : 0}};
}

int query_int(const httplib::Request& request, const char* name, int fallback,
              int minimum, int maximum) {
  if (!request.has_param(name)) return fallback;
  try {
    return std::clamp(std::stoi(request.get_param_value(name)), minimum,
                      maximum);
  } catch (...) {
    throw std::invalid_argument(std::string("query parameter ") + name +
                                " must be an integer");
  }
}

std::optional<double> query_double(const httplib::Request& request,
                                   const char* name) {
  if (!request.has_param(name)) return std::nullopt;
  try {
    const double value = std::stod(request.get_param_value(name));
    if (!std::isfinite(value)) throw std::invalid_argument("not finite");
    return value;
  } catch (...) {
    throw std::invalid_argument(std::string("query parameter ") + name +
                                " must be a finite number");
  }
}

std::string bus_key(const std::string& domain, int index) {
  return domain + ":" + std::to_string(index);
}

json bus_ref(const std::string& domain, int index) {
  return json{{"domain", domain}, {"index", index}};
}

json topology_lod2(const HybridPowerSystem& system) {
  json nodes = json::array();
  json edges = json::array();
  const std::size_t ac_count = system.ac.buses.size();
  const std::size_t dc_count = system.dc.buses.size();
  const std::size_t columns = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(std::sqrt(
             static_cast<double>(std::max(ac_count, dc_count))))));

  auto grid_position = [columns](std::size_t position, bool dc) {
    const double x = static_cast<double>(position % columns) +
                     (dc ? static_cast<double>(columns) + 2.0 : 0.0);
    const double y = static_cast<double>(position / columns);
    return std::pair<double, double>{x, y};
  };
  for (std::size_t position = 0; position < ac_count; ++position) {
    const auto& bus = system.ac.buses[position];
    const auto [grid_x, grid_y] = grid_position(position, false);
    const bool geographic = bus.latitude != 0.0 || bus.longitude != 0.0;
    nodes.push_back({{"ref", bus_ref("ac", bus.index)},
                     {"key", bus_key("ac", bus.index)},
                     {"name", bus.name},
                     {"domain", "ac"},
                     {"index", bus.index},
                     {"position", position},
                     {"area", bus.area},
                     {"zone", bus.zone},
                     {"base_kv", bus.base_kv},
                     {"in_service", bus.in_service},
                     {"x", geographic ? bus.longitude : grid_x},
                     {"y", geographic ? -bus.latitude : grid_y},
                     {"position_source", geographic ? "geographic" : "grid"}});
  }
  for (std::size_t position = 0; position < dc_count; ++position) {
    const auto& bus = system.dc.buses[position];
    const auto [grid_x, grid_y] = grid_position(position, true);
    const bool geographic = bus.latitude != 0.0 || bus.longitude != 0.0;
    nodes.push_back({{"ref", bus_ref("dc", bus.index)},
                     {"key", bus_key("dc", bus.index)},
                     {"name", bus.name},
                     {"domain", "dc"},
                     {"index", bus.index},
                     {"position", position},
                     {"area", bus.area},
                     {"zone", bus.zone},
                     {"base_kv", bus.base_kv},
                     {"in_service", bus.in_service},
                     {"x", geographic ? bus.longitude : grid_x},
                     {"y", geographic ? -bus.latitude : grid_y},
                     {"position_source", geographic ? "geographic" : "grid"}});
  }

  auto add_edge = [&edges](const char* domain, const char* resource, int index,
                           const std::string& name, int from, int to,
                           bool in_service, double rating) {
    edges.push_back(
        {{"ref", {{"domain", domain}, {"resource", resource}, {"index", index}}},
         {"key", std::string(domain) + ":" + resource + ":" +
                     std::to_string(index)},
         {"domain", domain},
         {"resource", resource},
         {"index", index},
         {"name", name},
         {"source", bus_ref(domain, from)},
         {"target", bus_ref(domain, to)},
         {"source_key", bus_key(domain, from)},
         {"target_key", bus_key(domain, to)},
         {"in_service", in_service},
         {"rating_mva", rating}});
  };
  for (const auto& branch : system.ac.branches) {
    add_edge("ac", "branch", branch.index, branch.name, branch.from_bus,
             branch.to_bus, branch.in_service, branch.rate_a_mva);
  }
  for (const auto& transformer : system.ac.transformers_2w) {
    add_edge("ac", "transformer_2w", transformer.index, transformer.name,
             transformer.hv_bus, transformer.lv_bus, transformer.in_service,
             transformer.sn_mva);
  }
  for (const auto& branch : system.dc.branches) {
    add_edge("dc", "branch", branch.index, branch.name, branch.from_bus,
             branch.to_bus, branch.in_service, branch.rate_a_mva);
  }
  for (const auto& converter : system.vsc_converters) {
    edges.push_back(
        {{"ref", {{"domain", "hybrid"}, {"resource", "vsc"},
                  {"index", converter.index}}},
         {"key", "hybrid:vsc:" + std::to_string(converter.index)},
         {"domain", "hybrid"},
         {"resource", "vsc"},
         {"index", converter.index},
         {"source", bus_ref("ac", converter.bus_ac)},
         {"target", bus_ref("dc", converter.bus_dc)},
         {"source_key", bus_key("ac", converter.bus_ac)},
         {"target_key", bus_key("dc", converter.bus_dc)},
         {"in_service", converter.in_service},
         {"rating_mva", converter.p_rated_mw}});
  }
  for (const auto& converter : system.dc.dcdc_converters) {
    add_edge("dc", "dcdc_converter", converter.index, converter.name,
             converter.bus_in, converter.bus_out, converter.in_service,
             converter.sn_mva);
  }
  return json{{"nodes", std::move(nodes)}, {"edges", std::move(edges)}};
}

json aggregate_topology(const json& full, int lod) {
  if (lod >= 2) return full;
  std::map<std::string, json> groups;
  std::unordered_map<std::string, std::string> node_group;
  for (const auto& node : full.at("nodes")) {
    const std::string domain = node.value("domain", "ac");
    const int area = node.value("area", 0);
    const int zone = node.value("zone", 0);
    const std::string key = lod == 0
                                ? domain
                                : domain + ":area:" + std::to_string(area) +
                                      ":zone:" + std::to_string(zone);
    node_group[node.at("key").get<std::string>()] = key;
    auto [found, inserted] = groups.emplace(
        key, json{{"key", key},
                  {"domain", domain},
                  {"area", lod == 0 ? json(nullptr) : json(area)},
                  {"zone", lod == 0 ? json(nullptr) : json(zone)},
                  {"member_count", 0},
                  {"x", 0.0},
                  {"y", 0.0},
                  {"in_service_count", 0}});
    auto& group = found->second;
    group["member_count"] = group.at("member_count").get<int>() + 1;
    group["x"] = group.at("x").get<double>() + node.value("x", 0.0);
    group["y"] = group.at("y").get<double>() + node.value("y", 0.0);
    if (node.value("in_service", true)) {
      group["in_service_count"] =
          group.at("in_service_count").get<int>() + 1;
    }
  }
  json nodes = json::array();
  for (auto& [key, group] : groups) {
    const double count = group.at("member_count").get<double>();
    group["x"] = group.at("x").get<double>() / std::max(1.0, count);
    group["y"] = group.at("y").get<double>() / std::max(1.0, count);
    group["name"] = lod == 0
                        ? (group.at("domain") == "ac" ? "AC network"
                                                       : "DC network")
                        : group.at("domain").get<std::string>() + " area " +
                              std::to_string(group.at("area").get<int>()) +
                              " zone " +
                              std::to_string(group.at("zone").get<int>());
    nodes.push_back(std::move(group));
  }
  std::map<std::string, json> grouped_edges;
  for (const auto& edge : full.at("edges")) {
    const auto source = node_group.find(edge.at("source_key").get<std::string>());
    const auto target = node_group.find(edge.at("target_key").get<std::string>());
    if (source == node_group.end() || target == node_group.end() ||
        source->second == target->second) {
      continue;
    }
    std::string a = source->second;
    std::string b = target->second;
    if (b < a) std::swap(a, b);
    const std::string key = a + "->" + b;
    auto [found, inserted] = grouped_edges.emplace(
        key, json{{"key", key},
                  {"source_key", a},
                  {"target_key", b},
                  {"member_count", 0},
                  {"in_service_count", 0}});
    auto& group = found->second;
    group["member_count"] = group.at("member_count").get<int>() + 1;
    if (edge.value("in_service", true)) {
      group["in_service_count"] =
          group.at("in_service_count").get<int>() + 1;
    }
  }
  json edges = json::array();
  for (auto& [key, edge] : grouped_edges) edges.push_back(std::move(edge));
  return json{{"nodes", std::move(nodes)}, {"edges", std::move(edges)}};
}

json topology_chunk(const HybridPowerSystem& system,
                    const httplib::Request& request) {
  const int lod = query_int(request, "lod", 2, 0, 2);
  json topology = aggregate_topology(topology_lod2(system), lod);
  const auto xmin = query_double(request, "xmin");
  const auto ymin = query_double(request, "ymin");
  const auto xmax = query_double(request, "xmax");
  const auto ymax = query_double(request, "ymax");
  const bool has_viewport = xmin && ymin && xmax && ymax;
  if ((xmin || ymin || xmax || ymax) && !has_viewport) {
    throw std::invalid_argument(
        "spatial chunk requires xmin, ymin, xmax, and ymax together");
  }
  json filtered_nodes = json::array();
  for (const auto& node : topology.at("nodes")) {
    const double x = node.value("x", 0.0);
    const double y = node.value("y", 0.0);
    if (!has_viewport || (x >= *xmin && x <= *xmax && y >= *ymin && y <= *ymax)) {
      filtered_nodes.push_back(node);
    }
  }
  const std::size_t total = filtered_nodes.size();
  const std::size_t offset = static_cast<std::size_t>(
      query_int(request, "offset", 0, 0, std::numeric_limits<int>::max()));
  const std::size_t limit = static_cast<std::size_t>(
      query_int(request, "limit", 10000, 1, 50000));
  json nodes = json::array();
  std::set<std::string> selected;
  const std::size_t end = std::min(total, offset + limit);
  for (std::size_t i = std::min(offset, total); i < end; ++i) {
    nodes.push_back(filtered_nodes[i]);
    selected.insert(filtered_nodes[i].at("key").get<std::string>());
  }
  json edges = json::array();
  for (const auto& edge : topology.at("edges")) {
    if (selected.count(edge.at("source_key").get<std::string>()) &&
        selected.count(edge.at("target_key").get<std::string>())) {
      edges.push_back(edge);
    }
  }
  return json{{"schema", "hysim_topology_chunk_v1"},
              {"lod", lod},
              {"coordinate_space", "model_layout"},
              {"total_nodes", total},
              {"returned_nodes", nodes.size()},
              {"offset", offset},
              {"limit", limit},
              {"next_offset", end < total ? json(end) : json(nullptr)},
              {"viewport", has_viewport
                               ? json{{"xmin", *xmin}, {"ymin", *ymin},
                                      {"xmax", *xmax}, {"ymax", *ymax}}
                               : json(nullptr)},
              {"nodes", std::move(nodes)},
              {"edges", std::move(edges)}};
}

json topology_subgraph(const HybridPowerSystem& system,
                       const httplib::Request& request) {
  const std::string domain =
      request.has_param("domain") ? request.get_param_value("domain") : "ac";
  if (domain != "ac" && domain != "dc") {
    throw std::invalid_argument("domain must be ac or dc");
  }
  if (!request.has_param("index")) {
    throw std::invalid_argument("index is required");
  }
  const int index = query_int(request, "index", 0,
                              std::numeric_limits<int>::min(),
                              std::numeric_limits<int>::max());
  const int depth = query_int(request, "depth", 2, 0, 8);
  const int max_nodes = query_int(request, "max_nodes", 500, 1, 5000);
  json full = topology_lod2(system);
  const std::string center = bus_key(domain, index);
  std::unordered_map<std::string, std::vector<std::string>> adjacency;
  for (const auto& node : full.at("nodes")) {
    adjacency[node.at("key").get<std::string>()];
  }
  if (!adjacency.count(center)) {
    throw std::invalid_argument("center bus does not exist");
  }
  for (const auto& edge : full.at("edges")) {
    const std::string a = edge.at("source_key").get<std::string>();
    const std::string b = edge.at("target_key").get<std::string>();
    adjacency[a].push_back(b);
    adjacency[b].push_back(a);
  }
  std::queue<std::pair<std::string, int>> queue;
  std::map<std::string, int> distance;
  queue.push({center, 0});
  distance[center] = 0;
  bool truncated = false;
  while (!queue.empty()) {
    const auto [key, hop] = queue.front();
    queue.pop();
    if (hop >= depth) continue;
    for (const auto& next : adjacency[key]) {
      if (distance.count(next)) continue;
      if (static_cast<int>(distance.size()) >= max_nodes) {
        truncated = true;
        continue;
      }
      distance[next] = hop + 1;
      queue.push({next, hop + 1});
    }
  }
  json nodes = json::array();
  for (auto node : full.at("nodes")) {
    const std::string key = node.at("key").get<std::string>();
    const auto found = distance.find(key);
    if (found == distance.end()) continue;
    node["hop"] = found->second;
    nodes.push_back(std::move(node));
  }
  json edges = json::array();
  for (const auto& edge : full.at("edges")) {
    if (distance.count(edge.at("source_key").get<std::string>()) &&
        distance.count(edge.at("target_key").get<std::string>())) {
      edges.push_back(edge);
    }
  }
  return json{{"schema", "hysim_subgraph_v1"},
              {"center", bus_ref(domain, index)},
              {"depth", depth},
              {"truncated", truncated},
              {"nodes", std::move(nodes)},
              {"edges", std::move(edges)}};
}

json converter_scope_json(const ConverterModelScope& scope) {
  const auto& value = scope.validity;
  return json{
      {"model_scope", scope.model_scope},
      {"validity_flags",
       {{"vsc_loss_modelled", value.vsc_loss_modelled},
        {"vsc_ac_conduction_loss_modelled",
         value.vsc_ac_conduction_loss_modelled},
        {"vsc_capacity_circle_enforced",
         value.vsc_capacity_circle_enforced},
        {"vsc_current_limits_enforced", value.vsc_current_limits_enforced},
        {"vsc_gfm_norton_modelled", value.vsc_gfm_norton_modelled},
        {"vsc_gfm_priority_limit_enforced",
         value.vsc_gfm_priority_limit_enforced},
        {"vsc_gfm_island_reference_modelled",
         value.vsc_gfm_island_reference_modelled},
        {"vsc_modulation_limits_enforced",
         value.vsc_modulation_limits_enforced},
        {"vsc_vdc_control_modelled", value.vsc_vdc_control_modelled},
        {"dc_multisource_coordination_modelled",
         value.dc_multisource_coordination_modelled},
         {"dcdc_loss_modelled", value.dcdc_loss_modelled},
         {"dcdc_duty_ratio_enforced", value.dcdc_duty_ratio_enforced},
         {"lcc_quasi_steady_modelled",
          value.lcc_quasi_steady_modelled},
         {"lcc_transformer_tap_control_modelled",
          value.lcc_transformer_tap_control_modelled},
         {"equation_closure_checked", value.equation_closure_checked}}}};
}

json lcc_transfers_json(const std::vector<LCCTransfer>& transfers) {
  json rows = json::array();
  for (const auto& transfer : transfers) {
    rows.push_back(
        {{"index", transfer.index},
         {"ac_bus", {{"domain", "ac"}, {"index", transfer.bus_ac}}},
         {"dc_bus", {{"domain", "dc"}, {"index", transfer.bus_dc}}},
         {"station_role", transfer.station_role},
         {"control_mode", transfer.control_mode},
         {"alpha_deg", transfer.alpha_deg},
         {"gamma_deg", transfer.gamma_deg},
         {"ud0_kv", transfer.ud0_kv},
         {"ud_kv", transfer.ud_kv},
         {"id_ka", transfer.id_ka},
         {"p_ac_mw", transfer.p_ac_mw},
         {"q_ac_mvar", transfer.q_ac_mvar},
         {"p_dc_mw", transfer.p_dc_mw},
         {"transformer_tap", transfer.transformer_tap},
         {"tap_target_angle_deg", transfer.tap_target_angle_deg},
         {"tap_control_iterations", transfer.tap_control_iterations},
         {"tap_control_active", transfer.tap_control_active},
         {"tap_control_converged", transfer.tap_control_converged},
         {"tap_at_limit", transfer.tap_at_limit},
         {"id_at_limit", transfer.id_at_limit},
         {"alpha_within_limits", transfer.alpha_within_limits},
         {"gamma_within_limits", transfer.gamma_within_limits}});
  }
  return rows;
}

void set_if_int(const json& body, const char* key, int& target) {
  if (body.contains(key) && body.at(key).is_number_integer()) {
    target = body.at(key).get<int>();
  }
}

void set_if_double(const json& body, const char* key, double& target) {
  if (body.contains(key) && body.at(key).is_number()) {
    target = body.at(key).get<double>();
  }
}

void set_if_bool(const json& body, const char* key, bool& target) {
  if (body.contains(key) && body.at(key).is_boolean()) {
    target = body.at(key).get<bool>();
  }
}

PowerFlowOptions power_flow_options(const json& request) {
  const json& body = request.contains("options") && request.at("options").is_object()
                         ? request.at("options")
                         : request;
  PowerFlowOptions options;
  set_if_int(body, "max_iter", options.max_iter);
  set_if_double(body, "tol", options.tol);
  set_if_int(body, "fdpf_max_iter", options.fdpf_max_iter);
  set_if_bool(body, "enable_pv_pq_conversion",
              options.enable_pv_pq_conversion);
  set_if_int(body, "pv_pq_max_outer_iterations",
             options.pv_pq_max_outer_iterations);
  set_if_bool(body, "enable_auto_swing_selection",
              options.enable_auto_swing_selection);
  set_if_bool(body, "enable_converter_mode_switching",
              options.enable_converter_mode_switching);
  set_if_bool(body, "enable_converter_coordination_check",
              options.enable_converter_coordination_check);
  set_if_bool(body, "enable_rigid_vdc_former",
              options.enable_rigid_vdc_former);
  set_if_bool(body, "enable_coupled_jacobian", options.enable_coupled_jacobian);
  set_if_bool(body, "enable_augmented_equations",
              options.enable_augmented_equations);
  set_if_bool(body, "enable_semi_smooth_newton",
              options.enable_semi_smooth_newton);
  set_if_bool(body, "enable_solver_profiling", options.enable_solver_profiling);
  set_if_bool(body, "enable_iteration_log", options.enable_iteration_log);
  set_if_int(body, "ac_eval_threads", options.ac_eval_threads);
  set_if_bool(body, "verbose", options.verbose);
  if (body.contains("robust_nonlinear") &&
      body.at("robust_nonlinear").is_object()) {
    const auto& robust = body.at("robust_nonlinear");
    set_if_bool(robust, "enable_klu_numeric_refactor",
                options.robust_nonlinear.enable_klu_numeric_refactor);
    set_if_double(
        robust, "refactor_backward_error_tolerance",
        options.robust_nonlinear.refactor_backward_error_tolerance);
    set_if_bool(robust, "enable_vsc_local_schur",
                options.robust_nonlinear.enable_vsc_local_schur);
    set_if_int(robust, "vsc_schur_min_network_dimension",
               options.robust_nonlinear.vsc_schur_min_network_dimension);
    set_if_double(robust, "vsc_schur_local_rcond_tolerance",
                  options.robust_nonlinear.vsc_schur_local_rcond_tolerance);
    set_if_double(
        robust, "vsc_schur_backward_error_tolerance",
        options.robust_nonlinear.vsc_schur_backward_error_tolerance);
    set_if_bool(robust, "enable_smooth_ncp",
                options.robust_nonlinear.enable_smooth_ncp);
    set_if_double(robust, "ncp_mu0", options.robust_nonlinear.ncp_mu0);
    set_if_double(robust, "ncp_mu_min", options.robust_nonlinear.ncp_mu_min);
    set_if_double(robust, "ncp_mu_factor",
                  options.robust_nonlinear.ncp_mu_factor);
    set_if_double(robust, "ncp_mu_factor_coarse",
                  options.robust_nonlinear.ncp_mu_factor_coarse);
    set_if_double(robust, "ncp_mu_phase_transition",
                  options.robust_nonlinear.ncp_mu_phase_transition);
  }
  options.max_iter = std::clamp(options.max_iter, 1, 100000);
  options.pv_pq_max_outer_iterations =
      std::clamp(options.pv_pq_max_outer_iterations, 1, 1000);
  options.tol = std::clamp(options.tol, 1.0e-14, 1.0);
  options.ac_eval_threads = std::clamp(options.ac_eval_threads, 1, 1024);
  powerflow::normalize_vsc_numerical_policy(options.robust_nonlinear);
  return options;
}

json serialize_power_flow(const HybridPowerSystem& system,
                          const PowerFlowResult& result,
                          const std::string& method,
                          const PowerFlowOptions& options) {
  json ac_buses = json::array();
  for (std::size_t i = 0; i < system.ac.buses.size(); ++i) {
    const auto& bus = system.ac.buses[i];
    ac_buses.push_back(
        {{"domain", "ac"},
         {"index", bus.index},
         {"position", i},
         {"name", bus.name},
         {"vm_pu", i < result.vm.size() ? json(result.vm[i]) : json(nullptr)},
         {"va_rad", i < result.va.size() ? json(result.va[i]) : json(nullptr)}});
  }
  json dc_buses = json::array();
  for (std::size_t i = 0; i < system.dc.buses.size(); ++i) {
    const auto& bus = system.dc.buses[i];
    dc_buses.push_back(
        {{"domain", "dc"},
         {"index", bus.index},
         {"position", i},
         {"name", bus.name},
         {"vdc_pu",
          i < result.vdc.size() ? json(result.vdc[i]) : json(nullptr)}});
  }
  json branches = json::array();
  for (std::size_t i = 0; i < system.ac.branches.size(); ++i) {
    const auto& branch = system.ac.branches[i];
    json row{{"domain", "ac"},
             {"index", branch.index},
             {"position", i},
             {"name", branch.name},
             {"from_bus", {{"domain", "ac"}, {"index", branch.from_bus}}},
             {"to_bus", {{"domain", "ac"}, {"index", branch.to_bus}}},
             {"rate_mva", branch.rate_a_mva}};
    if (i < result.branch_flows.size()) {
      const auto& flow = result.branch_flows[i];
      row["pf_mw"] = flow.pf_mw;
      row["qf_mvar"] = flow.qf_mvar;
      row["pt_mw"] = flow.pt_mw;
      row["qt_mvar"] = flow.qt_mvar;
      const double apparent = std::max(std::hypot(flow.pf_mw, flow.qf_mvar),
                                       std::hypot(flow.pt_mw, flow.qt_mvar));
      row["loading_pct"] = branch.rate_a_mva > 1.0e-12
                               ? 100.0 * apparent / branch.rate_a_mva
                               : 0.0;
    }
    branches.push_back(std::move(row));
  }
  json vsc = json::array();
  for (const auto& transfer : result.vsc_transfers) {
    vsc.push_back(
        {{"index", transfer.index},
         {"ac_bus", {{"domain", "ac"}, {"index", transfer.bus_ac}}},
         {"dc_bus", {{"domain", "dc"}, {"index", transfer.bus_dc}}},
         {"p_ac_mw", transfer.p_ac_mw},
         {"q_ac_mvar", transfer.q_ac_mvar},
         {"p_dc_mw", transfer.p_dc_mw},
         {"loss_mw", transfer.loss_mw},
         {"limit_ncp_enabled", transfer.limit_ncp_enabled},
         {"current_limit_active", transfer.current_limit_active},
         {"droop_saturated", transfer.droop_saturated},
         {"current_limit_priority", transfer.current_limit_priority},
         {"effective_mode", transfer.effective_mode},
         {"ac_current_pu", transfer.ac_current_pu},
         {"current_margin_pu", transfer.current_margin_pu},
         {"complementarity_residual", transfer.complementarity_residual}});
  }
  const json scope = converter_scope_json(result.converter_model_scope);
  json validity_flags = scope.at("validity_flags");
  validity_flags["generator_reactive_limits_enforced"] =
      result.reactive_limits.enforcement_requested;
  validity_flags["generator_reactive_limits_certified"] =
      result.reactive_limits.certified;
  const json reactive_limits{
      {"enforcement_requested",
       result.reactive_limits.enforcement_requested},
      {"certified", result.reactive_limits.certified},
      {"active_set_cycle_detected",
       result.reactive_limits.active_set_cycle_detected},
      {"outer_iteration_limit_reached",
       result.reactive_limits.outer_iteration_limit_reached},
      {"active_limited_buses",
       result.reactive_limits.active_limited_buses},
      {"max_violation_pu", result.reactive_limits.max_violation_pu},
      {"outer_iterations", result.profiling.pv_pq_outer_iterations},
      {"pv_to_pq_switches", result.profiling.pv_to_pq_switches},
      {"pq_to_pv_switches", result.profiling.pq_to_pv_switches},
      {"repeated_active_sets",
       result.profiling.pv_pq_repeated_active_sets},
      {"smooth_ncp_continuation_updates",
       result.profiling.smooth_ncp_continuation_updates},
      {"smooth_ncp_final_mu", result.profiling.smooth_ncp_final_mu}};
  const json linear_structure{
      {"vsc_schur_status", result.profiling.vsc_schur_status},
      {"vsc_schur_attempts", result.profiling.vsc_schur_attempts},
      {"vsc_schur_accepted", result.profiling.vsc_schur_accepted},
      {"vsc_schur_fallbacks", result.profiling.vsc_schur_fallbacks},
      {"local_regular_rejections",
       result.profiling.vsc_schur_local_regular_rejections},
      {"reduced_solve_rejections",
       result.profiling.vsc_schur_reduced_solve_rejections},
      {"full_backward_error_rejections",
       result.profiling.vsc_schur_full_backward_error_rejections},
      {"local_blocks", result.profiling.vsc_schur_local_blocks},
      {"full_dimension", result.profiling.vsc_schur_full_dimension},
      {"reduced_dimension", result.profiling.vsc_schur_reduced_dimension},
      {"full_structural_nnz",
       result.profiling.vsc_schur_full_structural_nnz},
      {"reduced_structural_nnz",
       result.profiling.vsc_schur_reduced_structural_nnz},
      {"reduced_factor_nonzeros",
       result.profiling.vsc_schur_reduced_factor_nonzeros},
      {"reduced_factor_work", result.profiling.vsc_schur_reduced_factor_work},
      {"full_lu_factor_nonzeros", result.profiling.full_lu_factor_nonzeros},
      {"full_lu_factor_work", result.profiling.full_lu_factor_work},
      {"minimum_local_rcond",
       result.profiling.vsc_schur_minimum_local_rcond},
      {"minimum_accepted_local_rcond",
       result.profiling.vsc_schur_minimum_accepted_local_rcond},
      {"max_reduced_backward_error",
       result.profiling.max_vsc_schur_reduced_backward_error},
      {"max_full_backward_error",
       result.profiling.max_vsc_schur_full_backward_error},
      {"semismooth_rate_samples",
       result.profiling.semismooth_rate_samples},
      {"semismooth_last_residual_ratio",
       result.profiling.semismooth_last_residual_ratio},
      {"semismooth_last_quadratic_ratio",
       result.profiling.semismooth_last_quadratic_ratio}};
  const auto& robust = options.robust_nonlinear;
  const json options_effective{
      {"max_iter", options.max_iter},
      {"tol", options.tol},
      {"enable_semi_smooth_newton", options.enable_semi_smooth_newton},
      {"robust_nonlinear",
       {{"enable_vsc_local_schur", robust.enable_vsc_local_schur},
        {"vsc_schur_min_network_dimension",
         robust.vsc_schur_min_network_dimension},
        {"vsc_schur_local_rcond_tolerance",
         robust.vsc_schur_local_rcond_tolerance},
        {"vsc_schur_backward_error_tolerance",
         robust.vsc_schur_backward_error_tolerance},
        {"enable_smooth_ncp", robust.enable_smooth_ncp},
        {"ncp_mu0", robust.ncp_mu0},
        {"ncp_mu_min", robust.ncp_mu_min},
        {"ncp_mu_factor", robust.ncp_mu_factor},
        {"ncp_mu_factor_coarse", robust.ncp_mu_factor_coarse},
        {"ncp_mu_phase_transition", robust.ncp_mu_phase_transition}}}};
  json lcc = lcc_transfers_json(result.lcc_transfers);
  return json{{"schema", "power_flow_result_v1"},
              {"method", method},
              {"method_actual", "hybrid_ac_dc_newton"},
              {"converged", result.converged},
              {"iterations", result.iterations},
              {"residual", result.residual},
              {"termination_reason", result.diagnostics.termination_reason},
              {"gfm_island_reference_vsc_indices",
               result.diagnostics.gfm_island_reference_vsc_indices},
              {"warnings", result.diagnostics.warnings},
              {"vm", result.vm},
              {"va", result.va},
              {"vdc", result.vdc},
              {"ac_bus_results", std::move(ac_buses)},
              {"dc_bus_results", std::move(dc_buses)},
               {"ac_branch_results", std::move(branches)},
               {"vsc_transfers", std::move(vsc)},
               {"lcc_transfers", std::move(lcc)},
              {"reactive_limits", reactive_limits},
              {"linear_structure", linear_structure},
              {"options_effective", options_effective},
               {"model_scope", scope.at("model_scope")},
              {"validity_flags", std::move(validity_flags)}};
}

json opf_audit_json(const opf::OpfAudit& audit) {
  return json{{"audited", audit.audited},
              {"feasible", audit.feasible()},
              {"max_power_balance_violation_mw",
               audit.max_power_balance_violation_mw},
              {"max_voltage_limit_violation_pu",
               audit.max_voltage_limit_violation_pu},
              {"max_branch_limit_violation_pu",
               audit.max_branch_limit_violation_pu},
              {"max_gen_limit_violation_mw", audit.max_gen_limit_violation_mw},
              {"objective_discrepancy_pct", audit.objective_discrepancy_pct},
              {"violations", audit.violations}};
}

json ac_opf_profiling_json(const opf::ACOPFResult& result) {
  const auto& p = result.profiling;
  return json{{"warm_start_used", p.warm_start_used},
              {"prepared_session_used", p.prepared_session_used},
              {"formulation_reused", p.formulation_reused},
              {"mapping_reused", p.mapping_reused},
              {"symbolic_reused", p.symbolic_reused},
              {"continuation_state_reused", p.continuation_state_reused},
              {"numeric_refactor_attempted", p.numeric_refactor_attempted},
              {"numeric_refactor_accepted", p.numeric_refactor_accepted},
              {"numeric_refactor_relative_drift",
               p.numeric_refactor_relative_drift},
              {"numeric_refactor_backward_error",
               p.numeric_refactor_backward_error},
              {"numeric_refactor_status", p.numeric_refactor_status},
              {"prepared_session_invalidation_reason",
               p.prepared_session_invalidation_reason},
              {"initial_primal_residual", p.initial_primal_residual},
              {"initial_dual_residual", p.initial_dual_residual},
              {"dc_phase_one_requested", p.dc_phase_one_requested},
              {"dc_phase_one_accepted", p.dc_phase_one_accepted},
              {"dc_phase_one_iterations", p.dc_phase_one_iterations},
              {"dc_phase_one_runtime_ms", p.dc_phase_one_runtime_ms},
              {"dc_phase_one_time_limit_ms", p.dc_phase_one_time_limit_ms},
              {"dc_phase_one_budget_exhausted",
               p.dc_phase_one_budget_exhausted},
              {"dc_phase_one_budget_overshoot_ms",
               p.dc_phase_one_budget_overshoot_ms},
              {"dc_phase_one_symbolic_analyze_calls",
               p.dc_phase_one_symbolic_analyze_calls},
              {"parity_formulation_builds", p.parity_formulation_builds},
              {"dc_phase_one_residual", p.dc_phase_one_residual},
              {"dc_phase_one_candidate_primal",
               p.dc_phase_one_candidate_primal},
              {"dc_phase_one_candidate_dual", p.dc_phase_one_candidate_dual},
              {"dc_phase_one_baseline_primal",
               p.dc_phase_one_baseline_primal},
              {"dc_phase_one_baseline_dual", p.dc_phase_one_baseline_dual},
              {"dc_phase_one_status", p.dc_phase_one_status},
              {"phase_one_initial_violation", p.phase_one_initial_violation},
              {"phase_one_constraint_violation",
               p.phase_one_constraint_violation},
              {"phase_one_dual_fit_residual", p.phase_one_dual_fit_residual},
              {"phase_one_primal_feasible", p.phase_one_primal_feasible},
              {"phase_one_in_handoff_corridor",
               p.phase_one_in_handoff_corridor},
              {"phase_one_dual_initialized", p.phase_one_dual_initialized},
              {"phase_one_handoff_primal_tolerance",
               p.phase_one_handoff_primal_tolerance},
              {"phase_one_perturbed_primal_residual",
               p.phase_one_perturbed_primal_residual},
              {"phase_one_centrality", p.phase_one_centrality},
              {"phase_one_barrier_mu", p.phase_one_barrier_mu},
              {"phase_one_budget_exhausted", p.phase_one_budget_exhausted},
              {"phase_one_iterations", p.phase_one_iterations},
              {"phase_one_factorizations", p.phase_one_factorizations},
              {"phase_one_backtracks", p.phase_one_backtracks},
              {"phase_one_structural_step_attempted",
               p.phase_one_structural_step_attempted},
              {"phase_one_structural_step_accepted",
               p.phase_one_structural_step_accepted},
              {"phase_one_structural_factorizations",
               p.phase_one_structural_factorizations},
              {"phase_one_structural_violation",
               p.phase_one_structural_violation},
              {"phase_one_structure", p.phase_one_structure},
              {"phase_one_runtime_ms", p.phase_one_runtime_ms},
              {"phase_one_termination", p.phase_one_termination},
              {"phase_one_linear_solver", p.phase_one_linear_solver},
              {"dispatch_dual_predictor_attempted",
               p.dispatch_dual_predictor_attempted},
              {"dispatch_dual_predictor_accepted",
               p.dispatch_dual_predictor_accepted},
              {"dispatch_dual_predictor_runtime_ms",
               p.dispatch_dual_predictor_runtime_ms},
              {"dispatch_dual_predictor_baseline_raw",
               p.dispatch_dual_predictor_baseline_raw},
              {"dispatch_dual_predictor_candidate_raw",
               p.dispatch_dual_predictor_candidate_raw},
              {"dispatch_dual_predictor_baseline_normalized",
               p.dispatch_dual_predictor_baseline_normalized},
              {"dispatch_dual_predictor_candidate_normalized",
               p.dispatch_dual_predictor_candidate_normalized},
              {"dispatch_dual_predictor_status",
               p.dispatch_dual_predictor_status},
              {"phase_two_start_accepted", p.phase_two_start_accepted},
              {"phase_two_start_rejection_reason",
               p.phase_two_start_rejection_reason},
              {"symbolic_analyze_calls", p.analyze_calls},
              {"factorization_calls", p.factorization_calls},
              {"linear_solve_calls", p.linear_solve_calls},
              {"accepted_steps", p.accepted_steps},
              {"rejected_steps", p.rejected_steps},
              {"backend_escalations", p.backend_escalations},
              {"scaling_rebuilds", p.scaling_rebuilds}};
}

json serialize_ac_opf(const HybridPowerSystem& system,
                      opf::ACOPFResult result,
                      const std::string& solver) {
  verify_opf_result(system, result);
  json buses = json::array();
  for (std::size_t i = 0; i < system.ac.buses.size(); ++i) {
    const auto& bus = system.ac.buses[i];
    json row{{"domain", "ac"},
             {"index", bus.index},
             {"position", i},
             {"name", bus.name}};
    if (i < result.vm.size()) row["vm_pu"] = result.vm[i];
    if (i < result.va.size()) row["va_rad"] = result.va[i];
    if (i < result.lmp_p.size()) row["lmp_p"] = result.lmp_p[i];
    if (i < result.lmp_q.size()) row["lmp_q"] = result.lmp_q[i];
    buses.push_back(std::move(row));
  }
  json generators = json::array();
  for (std::size_t i = 0; i < system.ac.generators.size(); ++i) {
    const auto& generator = system.ac.generators[i];
    json row{{"domain", "ac"},
             {"index", generator.index},
             {"position", i},
             {"bus", {{"domain", "ac"}, {"index", generator.bus}}},
             {"name", generator.name}};
    if (i < result.pg_mw.size()) row["pg_mw"] = result.pg_mw[i];
    if (i < result.qg_mvar.size()) row["qg_mvar"] = result.qg_mvar[i];
    generators.push_back(std::move(row));
  }
  const json scope = converter_scope_json(result.converter_model_scope);
  json lcc = lcc_transfers_json(result.lcc_transfers);
  return json{{"schema", "optimal_power_flow_result_v1"},
              {"solver", solver},
              {"network_model", "balanced_aggregate"},
              {"converged", result.converged},
              {"iterations", result.iterations},
              {"outer_iterations", result.outer_iterations},
              {"objective", result.objective},
              {"status", result.status},
              {"max_constraint_violation_pu",
               result.max_constraint_violation},
              {"max_stationarity", result.max_stationarity},
              {"solver_backend", result.profiling.linear_solver_backend},
              {"ipm_profiling", ac_opf_profiling_json(result)},
               {"infeasibility_hints", result.infeasibility_hints},
               {"model_limitations", result.model_limitations},
               {"audit", opf_audit_json(result.audit)},
              {"vm", result.vm},
              {"va", result.va},
              {"vdc", result.vdc},
              {"pg_mw", result.pg_mw},
              {"qg_mvar", result.qg_mvar},
               {"ac_bus_results", std::move(buses)},
               {"generator_dispatch", std::move(generators)},
               {"lcc_transfers", std::move(lcc)},
               {"model_scope", scope.at("model_scope")},
              {"validity_flags", scope.at("validity_flags")}};
}

json serialize_dc_opf(const HybridPowerSystem& system,
                      const opf::DCOPFResult& result) {
  json buses = json::array();
  for (std::size_t i = 0; i < system.ac.buses.size(); ++i) {
    const auto& bus = system.ac.buses[i];
    json row{{"domain", "ac"},
             {"index", bus.index},
             {"position", i},
             {"name", bus.name}};
    if (i < result.va.size()) row["va_rad"] = result.va[i];
    if (i < result.lmp.size()) row["lmp"] = result.lmp[i];
    buses.push_back(std::move(row));
  }
  json generators = json::array();
  for (std::size_t i = 0; i < system.ac.generators.size(); ++i) {
    const auto& generator = system.ac.generators[i];
    json row{{"domain", "ac"},
             {"index", generator.index},
             {"position", i},
             {"bus", {{"domain", "ac"}, {"index", generator.bus}}},
             {"name", generator.name}};
    if (i < result.pg_mw.size()) row["pg_mw"] = result.pg_mw[i];
    generators.push_back(std::move(row));
  }
  const json scope = converter_scope_json(result.converter_model_scope);
  return json{{"schema", "optimal_power_flow_result_v1"},
              {"solver", "dc"},
              {"network_model", "dc_approximation"},
              {"converged", result.converged},
              {"iterations", result.iterations},
              {"objective", result.objective},
              {"status", result.status},
              {"solver_backend", result.solver_name},
              {"solver_chain", result.solver_chain},
              {"objective_model", result.objective_model},
              {"structural_warm_start_requested",
               result.structural_warm_start_requested},
              {"structural_warm_start_built",
               result.structural_warm_start_built},
              {"structural_warm_start_used",
               result.structural_warm_start_used},
              {"structural_warm_start_components",
               result.structural_warm_start_components},
              {"structural_warm_start_factorizations",
               result.structural_warm_start_factorizations},
              {"structural_warm_start_residual",
               result.structural_warm_start_residual},
              {"solver_initial_primal_residual",
               result.solver_initial_primal_residual},
              {"structural_warm_start_status",
               result.structural_warm_start_status},
              {"phase_one_warm_start_only",
               result.phase_one_warm_start_only},
              {"phase_one_budget_exhausted",
               result.phase_one_budget_exhausted},
              {"phase_one_budget_overshoot_ms",
               result.phase_one_budget_overshoot_ms},
              {"native_qp_symbolic_analyze_calls",
               result.native_qp_symbolic_analyze_calls},
              {"phase_one_iterate_residual",
               result.phase_one_iterate_residual},
              {"runtime_sec", result.runtime_sec},
              {"branch_mu_valid", result.branch_mu_valid},
              {"total_load_shedding_mw", result.total_load_shedding_mw},
              {"va", result.va},
              {"pg_mw", result.pg_mw},
              {"pf_mw", result.pf_mw},
              {"lmp", result.lmp},
              {"ac_bus_results", std::move(buses)},
              {"generator_dispatch", std::move(generators)},
              {"model_scope", scope.at("model_scope")},
              {"validity_flags", scope.at("validity_flags")}};
}

opf::ACOPFOptions ac_opf_options(const json& request) {
  const json options = request.value("options", json::object());
  opf::ACOPFOptions value;
  const std::string solver = request.value("solver", std::string("parity"));
  if (solver == "parity") {
    value.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
    value.enable_primal_dual = true;
    value.use_parity_ipm = true;
  } else if (solver == "ipopt") {
    value.ac_solver_backend = opf::ACOPFSolverBackend::Ipopt;
  } else if (solver == "auto") {
    value.ac_solver_backend = opf::ACOPFSolverBackend::Auto;
    value.enable_primal_dual = true;
    value.use_parity_ipm = true;
  } else if (solver == "dispatch") {
    value.ac_solver_backend = opf::ACOPFSolverBackend::EconomicDispatch;
  } else {
    throw std::invalid_argument("unsupported AC OPF solver: " + solver);
  }
  set_if_int(options, "max_inner_iterations", value.max_inner_iterations);
  set_if_int(options, "max_outer_iterations", value.max_outer_iterations);
  set_if_int(options, "max_line_search_steps", value.max_line_search_steps);
  set_if_double(options, "feasibility_tol", value.feasibility_tol);
  set_if_double(options, "stationarity_tol", value.stationarity_tol);
  set_if_double(options, "barrier_mu0", value.barrier_mu0);
  set_if_double(options, "barrier_mu_reduction", value.barrier_mu_reduction);
  set_if_double(options, "regularization", value.regularization);
  set_if_int(options, "ac_eval_threads", value.ac_eval_threads);
  set_if_bool(options, "allow_fallback", value.allow_fallback);
  set_if_bool(options, "ac_pf_warm_start", value.ac_pf_warm_start);
  set_if_bool(options, "enable_phase_one", value.enable_phase_one);
  set_if_double(options, "phase_one_time_limit_ms",
                value.phase_one_time_limit_ms);
  set_if_int(options, "phase_one_max_iterations",
             value.phase_one_max_iterations);
  set_if_int(options, "phase_one_max_factorizations",
             value.phase_one_max_factorizations);
  set_if_int(options, "phase_one_max_backtracks",
             value.phase_one_max_backtracks);
  set_if_double(options, "phase_one_barrier_mu", value.phase_one_barrier_mu);
  set_if_double(options, "phase_one_admission_mu_factor",
                value.phase_one_admission_mu_factor);
  set_if_double(options, "phase_one_primal_mu_factor",
                value.phase_one_primal_mu_factor);
  set_if_double(options, "phase_one_centrality_tolerance",
                value.phase_one_centrality_tolerance);
  set_if_bool(options, "phase_one_dispatch_dual_predictor",
              value.phase_one_dispatch_dual_predictor);
  set_if_double(options, "phase_one_dispatch_dual_min_improvement",
                value.phase_one_dispatch_dual_min_improvement);
  set_if_bool(options, "ac_pf_dc_phase_one", value.ac_pf_dc_phase_one);
  set_if_int(options, "ac_pf_dc_phase_one_min_buses",
             value.ac_pf_dc_phase_one_min_buses);
  set_if_int(options, "ac_pf_dc_phase_one_max_iterations",
             value.ac_pf_dc_phase_one_max_iterations);
  set_if_double(options, "ac_pf_dc_phase_one_time_limit_ms",
                value.ac_pf_dc_phase_one_time_limit_ms);
  set_if_double(options, "ac_pf_dc_phase_one_tolerance",
                value.ac_pf_dc_phase_one_tolerance);
  set_if_double(options, "ac_pf_dc_phase_one_min_dual_improvement",
                value.ac_pf_dc_phase_one_min_dual_improvement);
  set_if_double(options, "ac_pf_dc_phase_one_baseline_dual_threshold",
                value.ac_pf_dc_phase_one_baseline_dual_threshold);
  set_if_bool(options, "verbose", value.verbose);
  const json constraints = request.value("constraints", json::object());
  set_if_bool(constraints, "branch_limits", value.enforce_branch_limits);
  set_if_bool(constraints, "converter_capacity",
              value.enforce_converter_capacity);
  set_if_bool(constraints, "converter_current",
              value.enforce_converter_current_limits);
  set_if_bool(constraints, "converter_modulation",
              value.enforce_converter_modulation_limits);
  value.max_inner_iterations = std::clamp(value.max_inner_iterations, 1, 100000);
  value.max_outer_iterations = std::clamp(value.max_outer_iterations, 1, 10000);
  value.feasibility_tol = std::clamp(value.feasibility_tol, 1.0e-12, 1.0);
  value.stationarity_tol = std::clamp(value.stationarity_tol, 1.0e-12, 1.0);
  value.phase_one_time_limit_ms =
      std::clamp(value.phase_one_time_limit_ms, -1.0, 3600000.0);
  value.phase_one_max_iterations =
      std::clamp(value.phase_one_max_iterations, 0, 10000);
  value.phase_one_max_factorizations =
      std::clamp(value.phase_one_max_factorizations, 0, 10000);
  value.phase_one_max_backtracks =
      std::clamp(value.phase_one_max_backtracks, 0, 100);
  value.phase_one_barrier_mu =
      std::clamp(value.phase_one_barrier_mu, 1.0e-12, 0.1);
  value.phase_one_admission_mu_factor =
      std::clamp(value.phase_one_admission_mu_factor, 0.0, 100.0);
  value.phase_one_primal_mu_factor =
      std::clamp(value.phase_one_primal_mu_factor, 0.0, 10.0);
  value.phase_one_centrality_tolerance =
      std::clamp(value.phase_one_centrality_tolerance, 0.0, 10.0);
  value.phase_one_dispatch_dual_min_improvement = std::clamp(
      value.phase_one_dispatch_dual_min_improvement, 0.0, 1.0);
  value.ac_pf_dc_phase_one_min_buses =
      std::clamp(value.ac_pf_dc_phase_one_min_buses, 0, 100000000);
  value.ac_pf_dc_phase_one_max_iterations =
      std::clamp(value.ac_pf_dc_phase_one_max_iterations, 1, 10000);
  value.ac_pf_dc_phase_one_time_limit_ms = std::clamp(
      value.ac_pf_dc_phase_one_time_limit_ms, -1.0, 3600000.0);
  value.ac_pf_dc_phase_one_tolerance =
      std::clamp(value.ac_pf_dc_phase_one_tolerance, 0.0, 1.0);
  value.ac_pf_dc_phase_one_min_dual_improvement = std::clamp(
      value.ac_pf_dc_phase_one_min_dual_improvement, 0.0, 1.0);
  value.ac_pf_dc_phase_one_baseline_dual_threshold = std::clamp(
      value.ac_pf_dc_phase_one_baseline_dual_threshold, 0.0, 1.0e12);
  return value;
}

opf::DCOPFOptions dc_opf_options(const json& request) {
  const json options = request.value("options", json::object());
  opf::DCOPFOptions value;
  set_if_int(options, "max_iterations", value.max_iterations);
  set_if_double(options, "feasibility_tol", value.feasibility_tol);
  set_if_int(options, "pwl_segments", value.pwl_segments);
  set_if_double(options, "branch_limit_margin", value.branch_limit_margin);
  set_if_bool(options, "load_shedding", value.load_shedding);
  set_if_double(options, "voll", value.voll);
  set_if_bool(options, "compute_lmp", value.compute_lmp);
  set_if_bool(options, "verbose", value.verbose);
  const json constraints = request.value("constraints", json::object());
  set_if_bool(constraints, "branch_limits", value.include_branch_limits);
  value.max_iterations = std::clamp(value.max_iterations, 1, 1000000);
  value.feasibility_tol = std::clamp(value.feasibility_tol, 1.0e-12, 1.0);
  value.pwl_segments = std::clamp(value.pwl_segments, 1, 1000);
  return value;
}

json execute_analysis(const HybridPowerSystem& system,
                      const std::string& analysis,
                      const json& request) {
  if (analysis == "power_flow") {
    const std::string method =
        request.value("method", std::string("ac_newton"));
    if (method != "ac_newton") {
      throw std::invalid_argument(
          "v1 power_flow currently supports method=ac_newton");
    }
    const auto options = power_flow_options(request);
    const auto result = solve_power_flow(system, options);
    return serialize_power_flow(system, result, method, options);
  }
  if (analysis == "optimal_power_flow") {
    const std::string network_model =
        request.value("network_model", std::string("balanced_aggregate"));
    if (network_model != "balanced_aggregate") {
      throw std::invalid_argument(
          "v1 optimal_power_flow currently supports network_model="
          "balanced_aggregate");
    }
    const std::string solver =
        request.value("solver", std::string("parity"));
    if (solver == "dc") {
      return serialize_dc_opf(
          system, ::hacdcpf::solve_dc_opf(system, dc_opf_options(request)));
    }
    return serialize_ac_opf(system,
                            solve_ac_opf(system, ac_opf_options(request)),
                            solver);
  }
  throw std::invalid_argument("unsupported v1 analysis: " + analysis);
}

struct ApiSession {
  mutable std::mutex mutex;
  std::string id;
  std::optional<HybridPowerSystem> system;
  std::uint64_t revision{0};
  std::string created_at;
  std::string updated_at;
  bool deleting{false};
};

json session_json(const ApiSession& session) {
  json body{{"schema", "hysim_session_v1"},
            {"session_id", session.id},
            {"model_revision", session.revision},
            {"etag", model_etag(session.id, session.revision)},
            {"has_model", session.system.has_value()},
            {"created_at", session.created_at},
            {"updated_at", session.updated_at}};
  if (session.system) {
    body["name"] = session.system->name;
    body["counts"] = model_counts(*session.system);
  }
  return body;
}

class SessionStore {
 public:
  std::shared_ptr<ApiSession> create() {
    auto session = std::make_shared<ApiSession>();
    session->id = random_id("ses");
    session->created_at = utc_now();
    session->updated_at = session->created_at;
    std::lock_guard<std::mutex> lock(mutex_);
    if (sessions_.size() >= kMaxSessions) {
      throw std::runtime_error("session capacity reached");
    }
    sessions_.emplace(session->id, session);
    return session;
  }

  std::shared_ptr<ApiSession> get(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = sessions_.find(id);
    return found == sessions_.end() ? nullptr : found->second;
  }

  bool erase(const std::string& id) {
    std::lock_guard<std::mutex> lock(mutex_);
    return sessions_.erase(id) > 0;
  }

  std::vector<std::shared_ptr<ApiSession>> list() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::shared_ptr<ApiSession>> result;
    result.reserve(sessions_.size());
    for (const auto& [id, session] : sessions_) {
      (void)id;
      result.push_back(session);
    }
    return result;
  }

 private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<ApiSession>> sessions_;
};

struct ApiJob {
  mutable std::mutex mutex;
  std::string id;
  std::string session_id;
  std::uint64_t model_revision{0};
  std::string analysis;
  json request;
  HybridPowerSystem system_snapshot;
  std::string state{"queued"};
  bool cancel_requested{false};
  std::uint64_t version{1};
  std::string created_at;
  std::string started_at;
  std::string finished_at;
  json result;
  json error;
};

bool terminal_state(const std::string& state) {
  return state == "succeeded" || state == "failed" || state == "cancelled";
}

json job_json(const ApiJob& job,
              std::optional<std::uint64_t> current_revision,
              bool include_result = true) {
  const bool stale = current_revision && *current_revision != job.model_revision;
  json body{{"schema", "hysim_job_v1"},
            {"job_id", job.id},
            {"session_id", job.session_id},
            {"model_revision", job.model_revision},
            {"analysis", job.analysis},
            {"state", job.state},
            {"cancel_requested", job.cancel_requested},
            {"created_at", job.created_at},
            {"started_at", job.started_at.empty() ? json(nullptr)
                                                   : json(job.started_at)},
            {"finished_at", job.finished_at.empty() ? json(nullptr)
                                                     : json(job.finished_at)},
            {"stale_against_current_model", stale}};
  if (current_revision) body["current_model_revision"] = *current_revision;
  if (!job.error.empty()) body["error"] = job.error;
  if (include_result && job.state == "succeeded") {
    body["result"] = job.result;
    std::string result_status = "completed";
    for (const char* key : {"converged", "success", "feasible", "accepted"}) {
      if (job.result.contains(key) && job.result.at(key).is_boolean() &&
          !job.result.at(key).get<bool>()) {
        result_status = "infeasible";
        break;
      }
    }
    body["result"]["_result_contract"] =
        json{{"schema", "hysim_result_v1"},
             {"analysis", job.analysis},
             {"request_id", job.id},
             {"session_id", job.session_id},
             {"model_revision", job.model_revision},
             {"current_model_revision",
              current_revision ? json(*current_revision) : json(nullptr)},
             {"stale", stale},
             {"status", stale ? "stale" : result_status},
             {"received_at", job.finished_at}};
  }
  return body;
}

std::set<int> query_indices(const httplib::Request& request) {
  std::set<int> values;
  if (!request.has_param("indices")) return values;
  std::istringstream input(request.get_param_value("indices"));
  std::string item;
  while (std::getline(input, item, ',')) {
    try {
      std::size_t consumed = 0;
      const int value = std::stoi(item, &consumed);
      if (consumed != item.size()) throw std::invalid_argument("trailing text");
      values.insert(value);
    } catch (...) {
      throw std::invalid_argument("indices must be a comma-separated integer list");
    }
  }
  return values;
}

json select_result_frame(const json& result, int step) {
  if (result.contains("frames") && result.at("frames").is_array()) {
    const auto& frames = result.at("frames");
    if (step < 0 || static_cast<std::size_t>(step) >= frames.size()) {
      throw std::out_of_range("result frame does not exist");
    }
    return frames.at(static_cast<std::size_t>(step));
  }
  if (step != 0) throw std::out_of_range("static result only has frame 0");
  return result;
}

json result_frame_chunk(const json& result, int step,
                        const httplib::Request& request,
                        const HybridPowerSystem* system) {
  const json frame = select_result_frame(result, step);
  const std::string domain =
      request.has_param("domain") ? request.get_param_value("domain") : "all";
  if (domain != "all" && domain != "ac" && domain != "dc") {
    throw std::invalid_argument("domain must be all, ac, or dc");
  }
  const std::set<int> indices = query_indices(request);
  const auto xmin = query_double(request, "xmin");
  const auto ymin = query_double(request, "ymin");
  const auto xmax = query_double(request, "xmax");
  const auto ymax = query_double(request, "ymax");
  const bool has_viewport = xmin && ymin && xmax && ymax;
  if ((xmin || ymin || xmax || ymax) && !has_viewport) {
    throw std::invalid_argument(
        "spatial frame chunk requires xmin, ymin, xmax, and ymax together");
  }
  if (has_viewport && !system) {
    throw std::invalid_argument(
        "viewport filtering is unavailable after the source model is deleted");
  }
  std::unordered_map<std::string, std::pair<double, double>> position;
  if (has_viewport) {
    const json topology = topology_lod2(*system);
    for (const auto& node : topology.at("nodes")) {
      position[node.at("key").get<std::string>()] =
          {node.value("x", 0.0), node.value("y", 0.0)};
    }
  }
  auto selected = [&](const json& row, const std::string& fallback_domain) {
    const std::string row_domain = row.value("domain", fallback_domain);
    const int index = row.value("index", std::numeric_limits<int>::min());
    if (domain != "all" && domain != row_domain) return false;
    if (!indices.empty() && !indices.count(index)) return false;
    if (has_viewport) {
      const auto found = position.find(bus_key(row_domain, index));
      if (found == position.end()) return false;
      if (found->second.first < *xmin || found->second.first > *xmax ||
          found->second.second < *ymin || found->second.second > *ymax) {
        return false;
      }
    }
    return true;
  };
  json all_nodes = json::array();
  for (const auto& [key, fallback_domain] :
       {std::pair<const char*, const char*>{"ac_bus_results", "ac"},
        {"dc_bus_results", "dc"}}) {
    if (!frame.contains(key) || !frame.at(key).is_array()) continue;
    for (const auto& row : frame.at(key)) {
      if (selected(row, fallback_domain)) all_nodes.push_back(row);
    }
  }
  const std::size_t total = all_nodes.size();
  const std::size_t offset = static_cast<std::size_t>(
      query_int(request, "offset", 0, 0, std::numeric_limits<int>::max()));
  const std::size_t limit = static_cast<std::size_t>(
      query_int(request, "limit", 5000, 1, 50000));
  const std::size_t end = std::min(total, offset + limit);
  json nodes = json::array();
  std::set<std::string> returned_bus_keys;
  for (std::size_t i = std::min(offset, total); i < end; ++i) {
    nodes.push_back(all_nodes[i]);
    returned_bus_keys.insert(bus_key(
        all_nodes[i].value("domain", "ac"),
        all_nodes[i].value("index", std::numeric_limits<int>::min())));
  }
  json branches = json::array();
  if (frame.contains("ac_branch_results") &&
      frame.at("ac_branch_results").is_array() &&
      (domain == "all" || domain == "ac")) {
    for (const auto& row : frame.at("ac_branch_results")) {
      const auto source = row.value("from_bus", json::object());
      const auto target = row.value("to_bus", json::object());
      const std::string a = bus_key(source.value("domain", "ac"),
                                    source.value("index", -1));
      const std::string b = bus_key(target.value("domain", "ac"),
                                    target.value("index", -1));
      if (returned_bus_keys.count(a) || returned_bus_keys.count(b)) {
        branches.push_back(row);
      }
    }
  }
  return json{{"schema", "hysim_result_frame_chunk_v1"},
              {"step", step},
              {"time", frame.value("time", json(nullptr))},
              {"lod", query_int(request, "lod", 2, 0, 3)},
              {"domain", domain},
              {"total_nodes", total},
              {"returned_nodes", nodes.size()},
              {"offset", offset},
              {"limit", limit},
              {"next_offset", end < total ? json(end) : json(nullptr)},
              {"nodes", std::move(nodes)},
              {"branches", std::move(branches)}};
}

json result_violations(const json& result, int step,
                       const httplib::Request& request) {
  const json frame = select_result_frame(result, step);
  const double vmin = query_double(request, "vmin").value_or(0.9);
  const double vmax = query_double(request, "vmax").value_or(1.1);
  const double loading_limit =
      query_double(request, "loading_limit_pct").value_or(100.0);
  const std::size_t limit = static_cast<std::size_t>(
      query_int(request, "limit", 200, 1, 5000));
  json items = json::array();
  auto add_bus_violations = [&](const char* key, const char* fallback_domain,
                                const char* voltage_key) {
    if (!frame.contains(key) || !frame.at(key).is_array()) return;
    for (const auto& row : frame.at(key)) {
      if (!row.contains(voltage_key) || !row.at(voltage_key).is_number()) continue;
      const double voltage = row.at(voltage_key).get<double>();
      if (voltage >= vmin && voltage <= vmax) continue;
      items.push_back(
          {{"kind", voltage < vmin ? "undervoltage" : "overvoltage"},
           {"severity", voltage < vmin ? vmin - voltage : voltage - vmax},
           {"ref", bus_ref(row.value("domain", fallback_domain),
                           row.value("index", -1))},
           {"value", voltage},
           {"unit", "pu"},
           {"limit", voltage < vmin ? vmin : vmax}});
    }
  };
  add_bus_violations("ac_bus_results", "ac", "vm_pu");
  add_bus_violations("dc_bus_results", "dc", "vdc_pu");
  if (frame.contains("ac_branch_results") &&
      frame.at("ac_branch_results").is_array()) {
    for (const auto& row : frame.at("ac_branch_results")) {
      const double loading = row.value("loading_pct", 0.0);
      if (loading <= loading_limit) continue;
      items.push_back(
          {{"kind", "overload"},
           {"severity", loading - loading_limit},
           {"ref", {{"domain", "ac"}, {"resource", "branch"},
                    {"index", row.value("index", -1)}}},
           {"value", loading},
           {"unit", "%"},
           {"limit", loading_limit}});
    }
  }
  std::sort(items.begin(), items.end(), [](const json& a, const json& b) {
    return a.value("severity", 0.0) > b.value("severity", 0.0);
  });
  const std::size_t total = items.size();
  if (items.size() > limit) items.erase(items.begin() + limit, items.end());
  return json{{"schema", "hysim_result_violation_chunk_v1"},
              {"step", step},
              {"total", total},
              {"returned", items.size()},
              {"items", std::move(items)}};
}

class JobManager {
 public:
  using Executor = std::function<json(const HybridPowerSystem&,
                                      const std::string&, const json&)>;

  JobManager(int worker_count, Executor executor)
      : executor_(std::move(executor)) {
    const int count = std::clamp(worker_count, 1, 32);
    workers_.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
      workers_.emplace_back([this] { worker_loop(); });
    }
  }

  ~JobManager() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      for (auto& job : queue_) {
        std::lock_guard<std::mutex> job_lock(job->mutex);
        if (job->state == "queued") {
          job->state = "cancelled";
          job->cancel_requested = true;
          job->finished_at = utc_now();
        }
      }
    }
    condition_.notify_all();
    for (auto& worker : workers_) {
      if (worker.joinable()) worker.join();
    }
  }

  std::shared_ptr<ApiJob> submit(const std::string& session_id,
                                 std::uint64_t revision,
                                 const std::string& analysis,
                                 json request,
                                 HybridPowerSystem snapshot) {
    auto job = std::make_shared<ApiJob>();
    job->id = random_id("job");
    job->session_id = session_id;
    job->model_revision = revision;
    job->analysis = analysis;
    job->request = std::move(request);
    job->system_snapshot = std::move(snapshot);
    job->created_at = utc_now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (jobs_.size() >= kMaxJobs) {
        throw std::runtime_error("job retention capacity reached");
      }
      jobs_.emplace(job->id, job);
      queue_.push_back(job);
    }
    condition_.notify_one();
    return job;
  }

  std::shared_ptr<ApiJob> get(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(id);
    return found == jobs_.end() ? nullptr : found->second;
  }

  std::vector<std::shared_ptr<ApiJob>> for_session(
      const std::string& session_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::shared_ptr<ApiJob>> result;
    for (const auto& [id, job] : jobs_) {
      (void)id;
      if (job->session_id == session_id) result.push_back(job);
    }
    return result;
  }

  bool has_active(const std::string& session_id) const {
    for (const auto& job : for_session(session_id)) {
      std::lock_guard<std::mutex> lock(job->mutex);
      if (!terminal_state(job->state)) return true;
    }
    return false;
  }

  bool erase_terminal(const std::string& id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(id);
    if (found == jobs_.end()) return false;
    std::lock_guard<std::mutex> job_lock(found->second->mutex);
    if (!terminal_state(found->second->state)) return false;
    jobs_.erase(found);
    return true;
  }

 private:
  void worker_loop() {
    while (true) {
      std::shared_ptr<ApiJob> job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
        if (stopping_) return;
        job = queue_.front();
        queue_.pop_front();
      }
      {
        std::lock_guard<std::mutex> lock(job->mutex);
        if (job->state != "queued") continue;
        job->state = "running";
        job->started_at = utc_now();
        ++job->version;
      }
      try {
        json result = executor_(job->system_snapshot, job->analysis, job->request);
        std::lock_guard<std::mutex> lock(job->mutex);
        job->finished_at = utc_now();
        if (job->cancel_requested) {
          job->state = "cancelled";
          job->result = json();
        } else {
          job->state = "succeeded";
          job->result = std::move(result);
        }
        job->system_snapshot = HybridPowerSystem{};
        ++job->version;
      } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(job->mutex);
        job->finished_at = utc_now();
        if (job->cancel_requested) {
          job->state = "cancelled";
        } else {
          job->state = "failed";
          job->error = json{{"code", "analysis_failed"},
                            {"message", error.what()}};
        }
        job->system_snapshot = HybridPowerSystem{};
        ++job->version;
      }
    }
  }

  Executor executor_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool stopping_{false};
  std::deque<std::shared_ptr<ApiJob>> queue_;
  std::unordered_map<std::string, std::shared_ptr<ApiJob>> jobs_;
  std::vector<std::thread> workers_;
};

void set_model_headers(httplib::Response& response,
                       const ApiSession& session) {
  response.set_header("ETag", model_etag(session.id, session.revision));
  response.set_header("X-HySim-Session-ID", session.id);
  response.set_header("X-HySim-Model-Revision",
                      std::to_string(session.revision));
}

bool check_if_none_match(const httplib::Request& request,
                         httplib::Response& response,
                         const ApiSession& session) {
  const std::string etag = model_etag(session.id, session.revision);
  if (request.has_header("If-None-Match") &&
      request.get_header_value("If-None-Match") == etag) {
    response.status = 304;
    set_model_headers(response, session);
    return true;
  }
  return false;
}

bool require_if_match(const httplib::Request& request,
                      httplib::Response& response,
                      const ApiSession& session) {
  const std::string expected = model_etag(session.id, session.revision);
  if (!request.has_header("If-Match")) {
    set_model_headers(response, session);
    set_error(response, 428, "precondition_required",
              "If-Match with the current model ETag is required",
              {{"current_etag", expected},
               {"current_model_revision", session.revision}});
    return false;
  }
  const std::string supplied = request.get_header_value("If-Match");
  if (supplied != expected) {
    set_model_headers(response, session);
    set_error(response, 412, "revision_conflict",
              "the supplied ETag does not match the current model revision",
              {{"supplied_etag", supplied},
               {"current_etag", expected},
               {"current_model_revision", session.revision}});
    return false;
  }
  return true;
}

}  // namespace

struct RuntimeApiV1::Impl {
  explicit Impl(CaseBuilder builder, int workers)
      : case_builder(std::move(builder)),
        jobs(workers, [](const HybridPowerSystem& system,
                         const std::string& analysis, const json& request) {
          return execute_analysis(system, analysis, request);
        }) {}

  HybridPowerSystem model_from(const json& body) const {
    if (body.contains("case") && body.at("case").is_string()) {
      return case_builder(body.at("case").get<std::string>());
    }
    if (body.contains("model") && body.at("model").is_object()) {
      return io::from_json(body.at("model").dump());
    }
    if (body.contains("json_string") && body.at("json_string").is_string()) {
      return io::from_json(body.at("json_string").get<std::string>());
    }
    throw std::invalid_argument(
        "model payload requires one of: case, model, json_string");
  }

  CaseBuilder case_builder;
  SessionStore sessions;
  JobManager jobs;
};

RuntimeApiV1::RuntimeApiV1(CaseBuilder case_builder, int worker_count)
    : impl_(std::make_unique<Impl>(std::move(case_builder), worker_count)) {}

RuntimeApiV1::~RuntimeApiV1() = default;

void RuntimeApiV1::register_routes(httplib::Server& server) {
  server.Get("/api/v1", [](const httplib::Request&,
                           httplib::Response& response) {
    set_json(response,
             {{"schema", "hysim_api_v1"},
              {"version", "1.0"},
              {"features",
               {"multi_session", "model_revision", "etag",
                "asynchronous_jobs", "topology_lod", "spatial_chunks",
                "time_chunks"}},
              {"analyses", {"power_flow", "optimal_power_flow"}},
              {"limits",
               {{"sessions", kMaxSessions}, {"retained_jobs", kMaxJobs}}}});
  });

  server.Post("/api/v1/sessions",
              [this](const httplib::Request& request,
                     httplib::Response& response) {
    guarded(response, [&] {
      const json body = parse_body(request);
      auto session = impl_->sessions.create();
      try {
        if (!body.empty()) {
          HybridPowerSystem system = impl_->model_from(body);
          std::lock_guard<std::mutex> lock(session->mutex);
          session->system = std::move(system);
          session->revision = 1;
          session->updated_at = utc_now();
        }
      } catch (...) {
        impl_->sessions.erase(session->id);
        throw;
      }
      std::lock_guard<std::mutex> lock(session->mutex);
      set_model_headers(response, *session);
      response.set_header("Location", "/api/v1/sessions/" + session->id);
      set_json(response, session_json(*session), 201);
    });
  });

  server.Get("/api/v1/sessions",
             [this](const httplib::Request&, httplib::Response& response) {
    guarded(response, [&] {
      json items = json::array();
      for (const auto& session : impl_->sessions.list()) {
        std::lock_guard<std::mutex> lock(session->mutex);
        if (!session->deleting) items.push_back(session_json(*session));
      }
      set_json(response,
               {{"schema", "hysim_session_collection_v1"},
                {"count", items.size()},
                {"sessions", std::move(items)}});
    });
  });

  server.Get(R"(/api/v1/sessions/([A-Za-z0-9_-]+))",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto session = impl_->sessions.get(request.matches[1].str());
      if (!session) {
        set_error(response, 404, "session_not_found", "session does not exist");
        return;
      }
      std::lock_guard<std::mutex> lock(session->mutex);
      if (check_if_none_match(request, response, *session)) return;
      set_model_headers(response, *session);
      set_json(response, session_json(*session));
    });
  });

  server.Get(R"(/api/v1/sessions/([A-Za-z0-9_-]+)/model)",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto session = impl_->sessions.get(request.matches[1].str());
      if (!session) {
        set_error(response, 404, "session_not_found", "session does not exist");
        return;
      }
      std::lock_guard<std::mutex> lock(session->mutex);
      if (!session->system) {
        set_error(response, 409, "model_not_loaded", "session has no model");
        return;
      }
      if (check_if_none_match(request, response, *session)) return;
      set_model_headers(response, *session);
      const json model = json::parse(io::to_json(*session->system, 2));
      set_json(response,
               {{"schema", "hysim_model_resource_v1"},
                {"session_id", session->id},
                {"model_revision", session->revision},
                {"model", model}});
    });
  });

  server.Get(R"(/api/v1/sessions/([A-Za-z0-9_-]+)/topology)",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto session = impl_->sessions.get(request.matches[1].str());
      if (!session) {
        set_error(response, 404, "session_not_found", "session does not exist");
        return;
      }
      std::lock_guard<std::mutex> lock(session->mutex);
      if (!session->system) {
        set_error(response, 409, "model_not_loaded", "session has no model");
        return;
      }
      if (check_if_none_match(request, response, *session)) return;
      set_model_headers(response, *session);
      json body = topology_chunk(*session->system, request);
      body["session_id"] = session->id;
      body["model_revision"] = session->revision;
      set_json(response, body);
    });
  });

  server.Get(R"(/api/v1/sessions/([A-Za-z0-9_-]+)/subgraph)",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto session = impl_->sessions.get(request.matches[1].str());
      if (!session) {
        set_error(response, 404, "session_not_found", "session does not exist");
        return;
      }
      std::lock_guard<std::mutex> lock(session->mutex);
      if (!session->system) {
        set_error(response, 409, "model_not_loaded", "session has no model");
        return;
      }
      if (check_if_none_match(request, response, *session)) return;
      set_model_headers(response, *session);
      json body = topology_subgraph(*session->system, request);
      body["session_id"] = session->id;
      body["model_revision"] = session->revision;
      set_json(response, body);
    });
  });

  server.Put(R"(/api/v1/sessions/([A-Za-z0-9_-]+)/model)",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto session = impl_->sessions.get(request.matches[1].str());
      if (!session) {
        set_error(response, 404, "session_not_found", "session does not exist");
        return;
      }
      const json body = parse_body(request);
      HybridPowerSystem replacement = impl_->model_from(body);
      std::lock_guard<std::mutex> lock(session->mutex);
      if (session->deleting) {
        set_error(response, 409, "session_deleting", "session is being deleted");
        return;
      }
      if (!require_if_match(request, response, *session)) return;
      session->system = std::move(replacement);
      ++session->revision;
      session->updated_at = utc_now();
      set_model_headers(response, *session);
      set_json(response, session_json(*session));
    });
  });

  server.Post(R"(/api/v1/sessions/([A-Za-z0-9_-]+)/jobs)",
              [this](const httplib::Request& request,
                     httplib::Response& response) {
    guarded(response, [&] {
      auto session = impl_->sessions.get(request.matches[1].str());
      if (!session) {
        set_error(response, 404, "session_not_found", "session does not exist");
        return;
      }
      const json body = parse_body(request);
      const std::string analysis = body.value("analysis", std::string());
      if (analysis != "power_flow" && analysis != "optimal_power_flow") {
        set_error(response, 400, "unsupported_analysis",
                  "analysis must be power_flow or optimal_power_flow");
        return;
      }
      const json analysis_request = body.value("request", json::object());
      if (!analysis_request.is_object()) {
        throw std::invalid_argument("job request must be a JSON object");
      }
      std::shared_ptr<ApiJob> job;
      {
        std::lock_guard<std::mutex> lock(session->mutex);
        if (session->deleting) {
          set_error(response, 409, "session_deleting", "session is being deleted");
          return;
        }
        if (!session->system) {
          set_error(response, 409, "model_not_loaded", "session has no model");
          return;
        }
        if (!require_if_match(request, response, *session)) return;
        job = impl_->jobs.submit(session->id, session->revision, analysis,
                                 analysis_request, *session->system);
        set_model_headers(response, *session);
      }
      std::lock_guard<std::mutex> lock(job->mutex);
      response.set_header("Location", "/api/v1/jobs/" + job->id);
      response.set_header("Retry-After", "1");
      response.set_header("Cache-Control", "no-store");
      set_json(response, job_json(*job, job->model_revision, false), 202);
    });
  });

  server.Get(R"(/api/v1/sessions/([A-Za-z0-9_-]+)/jobs)",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto session = impl_->sessions.get(request.matches[1].str());
      if (!session) {
        set_error(response, 404, "session_not_found", "session does not exist");
        return;
      }
      std::uint64_t revision = 0;
      {
        std::lock_guard<std::mutex> lock(session->mutex);
        revision = session->revision;
      }
      json items = json::array();
      for (const auto& job : impl_->jobs.for_session(session->id)) {
        std::lock_guard<std::mutex> lock(job->mutex);
        items.push_back(job_json(*job, revision, false));
      }
      response.set_header("Cache-Control", "no-store");
      set_json(response,
               {{"schema", "hysim_job_collection_v1"},
                {"session_id", session->id},
                {"count", items.size()},
                {"jobs", std::move(items)}});
    });
  });

  server.Delete(R"(/api/v1/sessions/([A-Za-z0-9_-]+))",
                [this](const httplib::Request& request,
                       httplib::Response& response) {
    guarded(response, [&] {
      auto session = impl_->sessions.get(request.matches[1].str());
      if (!session) {
        set_error(response, 404, "session_not_found", "session does not exist");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(session->mutex);
        if (!require_if_match(request, response, *session)) return;
        session->deleting = true;
      }
      if (impl_->jobs.has_active(session->id)) {
        std::lock_guard<std::mutex> lock(session->mutex);
        session->deleting = false;
        set_error(response, 409, "active_jobs",
                  "cancel or wait for active jobs before deleting the session");
        return;
      }
      impl_->sessions.erase(session->id);
      response.status = 204;
    });
  });

  server.Get(R"(/api/v1/jobs/([A-Za-z0-9_-]+))",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto job = impl_->jobs.get(request.matches[1].str());
      if (!job) {
        set_error(response, 404, "job_not_found", "job does not exist");
        return;
      }
      std::optional<std::uint64_t> revision;
      if (auto session = impl_->sessions.get(job->session_id)) {
        std::lock_guard<std::mutex> session_lock(session->mutex);
        revision = session->revision;
      }
      std::lock_guard<std::mutex> lock(job->mutex);
      response.set_header("Cache-Control", "no-store");
      set_json(response, job_json(*job, revision));
    });
  });

  server.Get(R"(/api/v1/jobs/([A-Za-z0-9_-]+)/frames/([0-9]+))",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto job = impl_->jobs.get(request.matches[1].str());
      if (!job) {
        set_error(response, 404, "job_not_found", "job does not exist");
        return;
      }
      json result;
      std::string session_id;
      std::uint64_t revision = 0;
      {
        std::lock_guard<std::mutex> lock(job->mutex);
        if (job->state != "succeeded") {
          set_error(response, 409, "result_not_ready",
                    "job must succeed before result frames are available");
          return;
        }
        result = job->result;
        session_id = job->session_id;
        revision = job->model_revision;
      }
      std::optional<HybridPowerSystem> system;
      if (request.has_param("xmin") || request.has_param("ymin") ||
          request.has_param("xmax") || request.has_param("ymax")) {
        if (auto session = impl_->sessions.get(session_id)) {
          std::lock_guard<std::mutex> lock(session->mutex);
          if (session->system && session->revision == revision) {
            system = *session->system;
          }
        }
      }
      const int step = std::stoi(request.matches[2].str());
      json body = result_frame_chunk(result, step, request,
                                     system ? &*system : nullptr);
      body["job_id"] = job->id;
      body["session_id"] = session_id;
      body["model_revision"] = revision;
      response.set_header("Cache-Control", "private, max-age=30");
      set_json(response, body);
    });
  });

  server.Get(R"(/api/v1/jobs/([A-Za-z0-9_-]+)/violations)",
             [this](const httplib::Request& request,
                    httplib::Response& response) {
    guarded(response, [&] {
      auto job = impl_->jobs.get(request.matches[1].str());
      if (!job) {
        set_error(response, 404, "job_not_found", "job does not exist");
        return;
      }
      json result;
      std::string session_id;
      std::uint64_t revision = 0;
      {
        std::lock_guard<std::mutex> lock(job->mutex);
        if (job->state != "succeeded") {
          set_error(response, 409, "result_not_ready",
                    "job must succeed before violations are available");
          return;
        }
        result = job->result;
        session_id = job->session_id;
        revision = job->model_revision;
      }
      const int step = query_int(request, "step", 0, 0,
                                 std::numeric_limits<int>::max());
      json body = result_violations(result, step, request);
      body["job_id"] = job->id;
      body["session_id"] = session_id;
      body["model_revision"] = revision;
      response.set_header("Cache-Control", "private, max-age=30");
      set_json(response, body);
    });
  });

  server.Post(R"(/api/v1/jobs/([A-Za-z0-9_-]+)/cancel)",
              [this](const httplib::Request& request,
                     httplib::Response& response) {
    guarded(response, [&] {
      auto job = impl_->jobs.get(request.matches[1].str());
      if (!job) {
        set_error(response, 404, "job_not_found", "job does not exist");
        return;
      }
      std::optional<std::uint64_t> revision;
      if (auto session = impl_->sessions.get(job->session_id)) {
        std::lock_guard<std::mutex> session_lock(session->mutex);
        revision = session->revision;
      }
      std::lock_guard<std::mutex> lock(job->mutex);
      if (!terminal_state(job->state)) {
        job->cancel_requested = true;
        if (job->state == "queued") {
          job->state = "cancelled";
          job->finished_at = utc_now();
        } else {
          job->state = "cancelling";
        }
        ++job->version;
      }
      response.set_header("Cache-Control", "no-store");
      set_json(response, job_json(*job, revision));
    });
  });

  server.Delete(R"(/api/v1/jobs/([A-Za-z0-9_-]+))",
                [this](const httplib::Request& request,
                       httplib::Response& response) {
    guarded(response, [&] {
      auto job = impl_->jobs.get(request.matches[1].str());
      if (!job) {
        set_error(response, 404, "job_not_found", "job does not exist");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(job->mutex);
        if (!terminal_state(job->state)) {
          set_error(response, 409, "job_not_terminal",
                    "only terminal jobs can be deleted");
          return;
        }
      }
      impl_->jobs.erase_terminal(job->id);
      response.status = 204;
    });
  });
}

}  // namespace hacdcpf::server
