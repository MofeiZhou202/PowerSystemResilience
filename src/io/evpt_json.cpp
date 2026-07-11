#include "hacdcpf/io/evpt_json.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace hacdcpf::io {

using json = nlohmann::json;
using namespace hacdcpf::evpt;

namespace {

// String-keyed JSON object from an unordered_map<int, T>.
template <typename T>
json int_map_to_json(const std::unordered_map<int, T>& m) {
  json out = json::object();
  for (const auto& [k, v] : m) out[std::to_string(k)] = v;
  return out;
}

json flow_map_to_json(
    const std::unordered_map<int, std::unordered_map<int, double>>& m) {
  json out = json::object();
  for (const auto& [d, rm] : m) out[std::to_string(d)] = int_map_to_json(rm);
  return out;
}

double num_or(const json& j, const char* key, double dflt) {
  if (!j.contains(key)) return dflt;
  const auto& v = j.at(key);
  if (v.is_number()) return v.get<double>();
  if (v.is_string()) {
    try {
      return std::stod(v.get<std::string>());
    } catch (...) {
    }
  }
  return dflt;
}

int int_or(const json& j, const char* key, int dflt) {
  return static_cast<int>(num_or(j, key, static_cast<double>(dflt)));
}

bool bool_or(const json& j, const char* key, bool dflt) {
  if (j.contains(key) && j.at(key).is_boolean()) return j.at(key).get<bool>();
  return dflt;
}

std::vector<double> vec_or(const json& j, const char* key) {
  std::vector<double> out;
  if (j.contains(key) && j.at(key).is_array()) {
    for (const auto& v : j.at(key)) {
      if (v.is_number()) out.push_back(v.get<double>());
    }
  }
  return out;
}

std::vector<int> ivec_or(const json& j, const char* key) {
  std::vector<int> out;
  if (j.contains(key) && j.at(key).is_array()) {
    for (const auto& v : j.at(key)) {
      if (v.is_number()) out.push_back(v.get<int>());
    }
  }
  return out;
}

std::vector<bool> bvec_or(const json& j, const char* key) {
  std::vector<bool> out;
  if (j.contains(key) && j.at(key).is_array()) {
    for (const auto& v : j.at(key)) {
      if (v.is_boolean()) out.push_back(v.get<bool>());
    }
  }
  return out;
}

// Per-link CTM occupancy/inflow/outflow time series, strided so that
// links × emitted-steps stays within the cell budget.
json ctm_link_series_to_json(const CTMSimulationResult& ctm,
                             int max_cells) {
  json out = json::object();
  const int T = static_cast<int>(ctm.step_link_results.size());
  const int L = T > 0 ? static_cast<int>(ctm.step_link_results.front().size())
                      : 0;
  out["dt_ctm_hr"] = ctm.dt_ctm_hr;
  out["steps_per_sim_step"] = ctm.steps_per_sim_step;
  out["n_ctm_steps"] = T;
  if (T <= 0 || L <= 0) {
    out["truncated"] = false;
    return out;
  }

  int stride = 1;
  if (max_cells > 0 && T * L > max_cells) {
    stride = (T * L + max_cells - 1) / max_cells;
  }
  out["step_stride"] = stride;
  out["truncated"] = stride > 1;

  std::vector<int> link_ids(static_cast<std::size_t>(L), 0);
  for (int li = 0; li < L; ++li) {
    link_ids[static_cast<std::size_t>(li)] =
        ctm.step_link_results.front()[static_cast<std::size_t>(li)].link_index;
  }
  out["link_ids"] = link_ids;

  json occ = json::array();
  json inflow = json::array();
  json outflow = json::array();
  json steps = json::array();
  for (int k = 0; k < T; k += stride) {
    steps.push_back(k);
    json row_o = json::array();
    json row_i = json::array();
    json row_f = json::array();
    for (int li = 0; li < L; ++li) {
      const auto& lr =
          ctm.step_link_results[static_cast<std::size_t>(k)]
                               [static_cast<std::size_t>(li)];
      row_o.push_back(lr.total_occupancy_veh);
      row_i.push_back(lr.total_inflow_veh);
      row_f.push_back(lr.total_outflow_veh);
    }
    occ.push_back(std::move(row_o));
    inflow.push_back(std::move(row_i));
    outflow.push_back(std::move(row_f));
  }
  out["ctm_steps"] = std::move(steps);
  out["occupancy_veh"] = std::move(occ);
  out["inflow_veh"] = std::move(inflow);
  out["outflow_veh"] = std::move(outflow);
  return out;
}

json sessions_summary_to_json(const std::vector<EVChargingSession>& sessions,
                              const std::vector<ChargingSessionResult>& results) {
  json arr = json::array();
  for (std::size_t i = 0; i < sessions.size(); ++i) {
    const auto& s = sessions[i];
    json js;
    js["index"] = s.index;
    js["station_id"] = s.station_id;
    js["arrival_step"] = s.arrival_step;
    js["departure_step"] = s.departure_step;
    js["vehicle_count"] = s.vehicle_count;
    js["energy_target_kwh"] = s.energy_target_kwh;
    js["v2g_capable"] = s.v2g_capable;
    if (i < results.size()) {
      js["unserved_energy_kwh"] = results[i].unserved_energy_kwh;
    }
    arr.push_back(std::move(js));
  }
  return arr;
}

}  // namespace

// ── Problem / scenario ─────────────────────────────────────────────────────

void evpt_problem_from_json(const json& j, EVPowerTrafficProblem& problem) {
  if (j.contains("traffic") && j.at("traffic").is_object()) {
    const auto& tj = j.at("traffic");
    problem.traffic.nodes.clear();
    problem.traffic.links.clear();
    if (tj.contains("nodes") && tj.at("nodes").is_array()) {
      for (const auto& nj : tj.at("nodes")) {
        TrafficNode n;
        n.index = int_or(nj, "index", 0);
        n.name = nj.value("name", std::string{});
        n.x = num_or(nj, "x", n.x);
        n.y = num_or(nj, "y", n.y);
        problem.traffic.nodes.push_back(std::move(n));
      }
    }
    if (tj.contains("links") && tj.at("links").is_array()) {
      for (const auto& lj : tj.at("links")) {
        TrafficLink l;
        l.index = int_or(lj, "index", 0);
        l.from_node = int_or(lj, "from_node", 0);
        l.to_node = int_or(lj, "to_node", 0);
        l.length_km = num_or(lj, "length_km", l.length_km);
        l.free_flow_time_hr = num_or(lj, "free_flow_time_hr", l.free_flow_time_hr);
        l.capacity_veh_per_hr =
            num_or(lj, "capacity_veh_per_hr", l.capacity_veh_per_hr);
        l.jam_vehicles = num_or(lj, "jam_vehicles", l.jam_vehicles);
        l.available = bool_or(lj, "available", l.available);
        l.capacity_profile_veh_per_hr =
            vec_or(lj, "capacity_profile_veh_per_hr");
        l.availability_profile = bvec_or(lj, "availability_profile");
        l.alpha = num_or(lj, "alpha", l.alpha);
        l.beta = num_or(lj, "beta", l.beta);
        l.drive_energy_kwh_per_veh_km =
            num_or(lj, "drive_energy_kwh_per_veh_km",
                   l.drive_energy_kwh_per_veh_km);
        problem.traffic.links.push_back(std::move(l));
      }
    }
  }

  if (j.contains("routes") && j.at("routes").is_array()) {
    problem.routes.clear();
    for (const auto& rj : j.at("routes")) {
      RouteAlternative r;
      r.index = int_or(rj, "index", 0);
      r.origin_node = int_or(rj, "origin_node", 0);
      r.destination_node = int_or(rj, "destination_node", 0);
      r.link_indices = ivec_or(rj, "link_indices");
      r.toll_cost = num_or(rj, "toll_cost", r.toll_cost);
      if (rj.contains("charging_stops") && rj.at("charging_stops").is_array()) {
        for (const auto& sj : rj.at("charging_stops")) {
          RouteChargingStop s;
          s.station_id = int_or(sj, "station_id", 0);
          s.requested_energy_kwh_per_vehicle =
              num_or(sj, "requested_energy_kwh_per_vehicle",
                     s.requested_energy_kwh_per_vehicle);
          s.requested_discharge_energy_kwh_per_vehicle =
              num_or(sj, "requested_discharge_energy_kwh_per_vehicle",
                     s.requested_discharge_energy_kwh_per_vehicle);
          s.dwell_steps = int_or(sj, "dwell_steps", s.dwell_steps);
          s.max_charge_kw_per_vehicle =
              num_or(sj, "max_charge_kw_per_vehicle", s.max_charge_kw_per_vehicle);
          s.v2g_capable = bool_or(sj, "v2g_capable", s.v2g_capable);
          s.max_discharge_kw_per_vehicle =
              num_or(sj, "max_discharge_kw_per_vehicle",
                     s.max_discharge_kw_per_vehicle);
          r.charging_stops.push_back(std::move(s));
        }
      }
      problem.routes.push_back(std::move(r));
    }
  }

  if (j.contains("demands") && j.at("demands").is_array()) {
    problem.demands.clear();
    for (const auto& dj : j.at("demands")) {
      EVDemand d;
      d.index = int_or(dj, "index", 0);
      d.origin_node = int_or(dj, "origin_node", 0);
      d.destination_node = int_or(dj, "destination_node", 0);
      d.departure_step = int_or(dj, "departure_step", 0);
      d.vehicles = num_or(dj, "vehicles", 0.0);
      d.candidate_route_indices = ivec_or(dj, "candidate_route_indices");
      d.initial_energy_kwh = num_or(dj, "initial_energy_kwh", d.initial_energy_kwh);
      d.energy_min_kwh = num_or(dj, "energy_min_kwh", d.energy_min_kwh);
      d.energy_max_kwh = num_or(dj, "energy_max_kwh", d.energy_max_kwh);
      d.reserve_energy_kwh = num_or(dj, "reserve_energy_kwh", d.reserve_energy_kwh);
      d.willingness_to_pay_per_vehicle =
          num_or(dj, "willingness_to_pay_per_vehicle",
                 d.willingness_to_pay_per_vehicle);
      d.departure_window_steps = ivec_or(dj, "departure_window_steps");
      problem.demands.push_back(std::move(d));
    }
  }

  if (j.contains("station_prices") && j.at("station_prices").is_array()) {
    problem.station_prices.clear();
    for (const auto& pj : j.at("station_prices")) {
      StationPriceProfile p;
      p.station_id = int_or(pj, "station_id", 0);
      p.price_per_kwh = vec_or(pj, "price_per_kwh");
      problem.station_prices.push_back(std::move(p));
    }
  }
}

json evpt_scenario_to_json(const EVPowerTrafficProblem& problem) {
  json out;
  out["$schema"] = "/xjtu/schemas/ev_traffic_scenario.schema.json";
  out["schema_version"] = "1.0";
  out["name"] = problem.system.name.empty() ? "EV power-traffic scenario"
                                             : problem.system.name;

  json nodes = json::array();
  for (const auto& n : problem.traffic.nodes) {
    json nj;
    nj["index"] = n.index;
    nj["name"] = n.name;
    if (std::isfinite(n.x)) nj["x"] = n.x;
    if (std::isfinite(n.y)) nj["y"] = n.y;
    nodes.push_back(std::move(nj));
  }
  json traffic;
  traffic["nodes"] = std::move(nodes);

  json links = json::array();
  for (const auto& l : problem.traffic.links) {
    json lj;
    lj["index"] = l.index;
    lj["from_node"] = l.from_node;
    lj["to_node"] = l.to_node;
    lj["length_km"] = l.length_km;
    lj["free_flow_time_hr"] = l.free_flow_time_hr;
    lj["capacity_veh_per_hr"] = l.capacity_veh_per_hr;
    lj["jam_vehicles"] = l.jam_vehicles;
    lj["available"] = l.available;
    lj["capacity_profile_veh_per_hr"] = l.capacity_profile_veh_per_hr;
    lj["availability_profile"] = l.availability_profile;
    lj["alpha"] = l.alpha;
    lj["beta"] = l.beta;
    lj["drive_energy_kwh_per_veh_km"] = l.drive_energy_kwh_per_veh_km;
    links.push_back(std::move(lj));
  }
  traffic["links"] = std::move(links);
  out["traffic"] = std::move(traffic);

  json routes = json::array();
  for (const auto& r : problem.routes) {
    json rj;
    rj["index"] = r.index;
    rj["origin_node"] = r.origin_node;
    rj["destination_node"] = r.destination_node;
    rj["link_indices"] = r.link_indices;
    rj["toll_cost"] = r.toll_cost;
    json stops = json::array();
    for (const auto& s : r.charging_stops) {
      stops.push_back({{"station_id", s.station_id},
                       {"requested_energy_kwh_per_vehicle",
                        s.requested_energy_kwh_per_vehicle},
                       {"requested_discharge_energy_kwh_per_vehicle",
                        s.requested_discharge_energy_kwh_per_vehicle},
                       {"dwell_steps", s.dwell_steps},
                       {"max_charge_kw_per_vehicle",
                        s.max_charge_kw_per_vehicle},
                       {"v2g_capable", s.v2g_capable},
                       {"max_discharge_kw_per_vehicle",
                        s.max_discharge_kw_per_vehicle}});
    }
    rj["charging_stops"] = std::move(stops);
    routes.push_back(std::move(rj));
  }
  out["routes"] = std::move(routes);

  json demands = json::array();
  for (const auto& d : problem.demands) {
    demands.push_back({{"index", d.index},
                       {"origin_node", d.origin_node},
                       {"destination_node", d.destination_node},
                       {"departure_step", d.departure_step},
                       {"vehicles", d.vehicles},
                       {"candidate_route_indices", d.candidate_route_indices},
                       {"initial_energy_kwh", d.initial_energy_kwh},
                       {"energy_min_kwh", d.energy_min_kwh},
                       {"energy_max_kwh", d.energy_max_kwh},
                       {"reserve_energy_kwh", d.reserve_energy_kwh},
                       {"willingness_to_pay_per_vehicle",
                        d.willingness_to_pay_per_vehicle},
                       {"departure_window_steps", d.departure_window_steps}});
  }
  out["demands"] = std::move(demands);

  json prices = json::array();
  for (const auto& p : problem.station_prices) {
    prices.push_back({{"station_id", p.station_id},
                      {"price_per_kwh", p.price_per_kwh}});
  }
  out["station_prices"] = std::move(prices);

  json stations = json::array();
  for (const auto& cs : problem.system.ac.charging_stations) {
    json sj;
    sj["station_id"] = cs.index;
    sj["name"] = cs.name;
    sj["bus"] = cs.bus;
    sj["max_power_kw"] = cs.max_power_kw;
    sj["in_service"] = cs.in_service;
    stations.push_back(std::move(sj));
  }
  out["charging_stations"] = std::move(stations);

  // Compact power-network echo used by the GUI's coupled topology view. This
  // is intentionally descriptive only; the imported traffic scenario never
  // replaces the power system held by the GUI session.
  json power;
  json buses = json::array();
  for (const auto& bus : problem.system.ac.buses) {
    buses.push_back({{"index", bus.index},
                     {"name", bus.name},
                     {"bus_type", static_cast<int>(bus.bus_type)},
                     {"pd_mw", bus.pd_mw},
                     {"base_kv", bus.base_kv},
                     {"in_service", bus.in_service}});
  }
  power["buses"] = std::move(buses);
  json branches = json::array();
  for (const auto& branch : problem.system.ac.branches) {
    branches.push_back({{"index", branch.index},
                        {"from_bus", branch.from_bus},
                        {"to_bus", branch.to_bus},
                        {"rate_a_mva", branch.rate_a_mva},
                        {"in_service", branch.in_service}});
  }
  power["branches"] = std::move(branches);
  json generators = json::array();
  for (const auto& gen : problem.system.ac.generators) {
    generators.push_back({{"index", gen.index},
                          {"name", gen.name},
                          {"bus", gen.bus},
                          {"pmin_mw", gen.pmin_mw},
                          {"pmax_mw", gen.pmax_mw},
                          {"cost_c1", gen.cost_c1},
                          {"is_slack", gen.is_slack},
                          {"in_service", gen.in_service}});
  }
  power["generators"] = std::move(generators);
  out["power_network"] = std::move(power);

  return out;
}

// ── Options ────────────────────────────────────────────────────────────────

EVPowerTrafficOptions evpt_options_from_json(const json& j) {
  EVPowerTrafficOptions o;
  o.num_steps = int_or(j, "num_steps", o.num_steps);
  o.time_step_hr = num_or(j, "time_step_hr", o.time_step_hr);
  o.auto_generate_routes = bool_or(j, "auto_generate_routes", o.auto_generate_routes);
  o.k_shortest_paths = int_or(j, "k_shortest_paths", o.k_shortest_paths);
  const std::string assignment =
      j.value("assignment_model", std::string{"deterministic_shortest_path"});
  if (assignment == "logit") {
    o.assignment_model = AssignmentModel::Logit;
  } else if (assignment == "capacity_aware_greedy") {
    o.assignment_model = AssignmentModel::CapacityAwareGreedy;
  } else if (assignment == "system_optimal_lp") {
    o.assignment_model = AssignmentModel::SystemOptimalLP;
  } else if (assignment == "system_optimal_milp") {
    o.assignment_model = AssignmentModel::SystemOptimalMILP;
  }
  o.logit_theta = num_or(j, "logit_theta", o.logit_theta);
  o.enforce_road_capacity = bool_or(j, "enforce_road_capacity", o.enforce_road_capacity);
  o.allow_unserved_travel_demand =
      bool_or(j, "allow_unserved_travel_demand", o.allow_unserved_travel_demand);
  o.allow_unserved_charging_energy =
      bool_or(j, "allow_unserved_charging_energy", o.allow_unserved_charging_energy);
  o.allow_v2g = bool_or(j, "allow_v2g", o.allow_v2g);
  o.value_of_time_per_hr = num_or(j, "value_of_time_per_hr", o.value_of_time_per_hr);
  o.queueing_weight = num_or(j, "queueing_weight", o.queueing_weight);
  o.station_energy_cost_weight =
      num_or(j, "station_energy_cost_weight", o.station_energy_cost_weight);
  o.default_station_price_per_kwh =
      num_or(j, "default_station_price_per_kwh", o.default_station_price_per_kwh);
  o.low_price_threshold_per_kwh =
      num_or(j, "low_price_threshold_per_kwh", o.low_price_threshold_per_kwh);
  o.high_price_threshold_per_kwh =
      num_or(j, "high_price_threshold_per_kwh", o.high_price_threshold_per_kwh);
  o.default_charging_efficiency =
      num_or(j, "default_charging_efficiency", o.default_charging_efficiency);
  o.default_route_stop_power_kw_per_vehicle =
      num_or(j, "default_route_stop_power_kw_per_vehicle",
             o.default_route_stop_power_kw_per_vehicle);
  o.default_station_power_kw =
      num_or(j, "default_station_power_kw", o.default_station_power_kw);
  o.system_optimal_unserved_trip_penalty =
      num_or(j, "system_optimal_unserved_trip_penalty",
             o.system_optimal_unserved_trip_penalty);
  o.system_optimal_mip_gap =
      num_or(j, "system_optimal_mip_gap", o.system_optimal_mip_gap);
  o.system_optimal_time_limit_sec =
      num_or(j, "system_optimal_time_limit_sec",
             o.system_optimal_time_limit_sec);
  o.system_optimal_max_nodes =
      int_or(j, "system_optimal_max_nodes", o.system_optimal_max_nodes);
  o.update_system_charging_stations =
      bool_or(j, "update_system_charging_stations",
              o.update_system_charging_stations);
  o.enforce_generation_capacity =
      bool_or(j, "enforce_generation_capacity", o.enforce_generation_capacity);
  o.external_grid_capacity_mw =
      num_or(j, "external_grid_capacity_mw", o.external_grid_capacity_mw);
  o.run_power_flow_validation =
      bool_or(j, "run_power_flow_validation", o.run_power_flow_validation);
  return o;
}

CTMOptions evpt_ctm_options_from_json(const json& j) {
  CTMOptions o;
  o.dt_ctm_hr = num_or(j, "dt_ctm_hr", o.dt_ctm_hr);
  o.n_cells_per_link = int_or(j, "n_cells_per_link", o.n_cells_per_link);
  o.backward_wave_speed_fallback_km_hr =
      num_or(j, "backward_wave_speed_fallback_km_hr",
             o.backward_wave_speed_fallback_km_hr);
  o.record_cell_history = bool_or(j, "record_cell_history", o.record_cell_history);
  o.route_specific_cells = bool_or(j, "route_specific_cells", o.route_specific_cells);
  o.enable_multiclass = bool_or(j, "enable_multiclass", o.enable_multiclass);
  o.ev_pce = num_or(j, "ev_pce", o.ev_pce);
  o.icv_pce = num_or(j, "icv_pce", o.icv_pce);
  o.phev_pce = num_or(j, "phev_pce", o.phev_pce);
  o.use_aggregate_link_variables =
      bool_or(j, "use_aggregate_link_variables", o.use_aggregate_link_variables);
  return o;
}

DUEOptions evpt_due_options_from_json(const json& j) {
  DUEOptions o;
  o.max_iterations = int_or(j, "max_iterations", o.max_iterations);
  o.convergence_tol = num_or(j, "convergence_tol", o.convergence_tol);
  o.msa_fixed_step = num_or(j, "msa_fixed_step", o.msa_fixed_step);
  o.compute_system_optimal_benchmark =
      bool_or(j, "compute_system_optimal_benchmark",
              o.compute_system_optimal_benchmark);
  o.logit_theta = num_or(j, "logit_theta", o.logit_theta);
  const std::string due_mode = j.value("due_mode", std::string{"fixed_departure"});
  if (due_mode == "full_endogenous") o.due_mode = DUEMode::FullEndogenous;
  o.max_departure_iterations =
      int_or(j, "max_departure_iterations", o.max_departure_iterations);
  o.departure_convergence_tol =
      num_or(j, "departure_convergence_tol", o.departure_convergence_tol);
  return o;
}

CTMJointWelfareOptions evpt_ctm_joint_options_from_json(const json& j) {
  CTMJointWelfareOptions o;
  o.max_iterations = int_or(j, "max_iterations", o.max_iterations);
  o.price_convergence_tol =
      num_or(j, "price_convergence_tol", o.price_convergence_tol);
  o.price_update_step = num_or(j, "price_update_step", o.price_update_step);
  o.use_dcopf_prices = bool_or(j, "use_dcopf_prices", o.use_dcopf_prices);
  if (j.contains("evpt_options") && j.at("evpt_options").is_object()) {
    o.evpt_opts = evpt_options_from_json(j.at("evpt_options"));
  }
  if (j.contains("ctm_options") && j.at("ctm_options").is_object()) {
    o.ctm_opts = evpt_ctm_options_from_json(j.at("ctm_options"));
  }
  if (j.contains("due_options") && j.at("due_options").is_object()) {
    o.due_opts = evpt_due_options_from_json(j.at("due_options"));
  }
  return o;
}

JointOptimizerOptions evpt_joint_optimizer_options_from_json(const json& j) {
  JointOptimizerOptions o;
  o.num_steps = int_or(j, "num_steps", o.num_steps);
  o.time_step_hr = num_or(j, "time_step_hr", o.time_step_hr);
  const std::string mode = j.value("mode", std::string{"social_welfare"});
  if (mode == "user_benefit") {
    o.mode = JointOptimizerMode::UserBenefitMax;
  } else if (mode == "integer_route_milp") {
    o.mode = JointOptimizerMode::IntegerRouteMILP;
  } else if (mode == "certified_dynamic_mpec_milp") {
    o.mode = JointOptimizerMode::CertifiedDynamicMPECMILP;
  } else if (mode == "certified_dynamic_user_benefit_mpec_milp") {
    o.mode = JointOptimizerMode::CertifiedDynamicUserBenefitMPECMILP;
  } else if (mode == "certified_ltm_pwl_milp") {
    o.mode = JointOptimizerMode::CertifiedFullJointLtmPwlMILP;
  } else if (mode == "certified_ltm_user_benefit_pwl_milp") {
    o.mode = JointOptimizerMode::CertifiedFullJointLtmUserBenefitPwlMILP;
  } else if (mode == "full_joint_nlp") {
    o.mode = JointOptimizerMode::FullJointSocialWelfareNLP;
  } else if (mode == "full_joint_user_benefit_nlp") {
    o.mode = JointOptimizerMode::FullJointUserBenefitNLP;
  } else {
    o.mode = JointOptimizerMode::SocialWelfareMax;
  }
  o.include_dcopf = bool_or(j, "include_dcopf", o.include_dcopf);
  o.allow_v2g = bool_or(j, "allow_v2g", o.allow_v2g);
  o.enforce_road_capacity =
      bool_or(j, "enforce_road_capacity", o.enforce_road_capacity);
  o.enforce_generation_capacity =
      bool_or(j, "enforce_generation_capacity", o.enforce_generation_capacity);
  o.unserved_trip_penalty =
      num_or(j, "unserved_trip_penalty", o.unserved_trip_penalty);
  o.power_slack_penalty =
      num_or(j, "power_slack_penalty", o.power_slack_penalty);
  o.outside_option_cost = num_or(j, "outside_option_cost", o.outside_option_cost);
  o.value_of_time_per_hr =
      num_or(j, "value_of_time_per_hr", o.value_of_time_per_hr);
  o.station_energy_cost_weight =
      num_or(j, "station_energy_cost_weight", o.station_energy_cost_weight);
  o.default_charging_efficiency =
      num_or(j, "default_charging_efficiency", o.default_charging_efficiency);
  o.default_route_stop_power_kw_per_vehicle =
      num_or(j, "default_route_stop_power_kw_per_vehicle",
             o.default_route_stop_power_kw_per_vehicle);
  o.default_station_power_kw =
      num_or(j, "default_station_power_kw", o.default_station_power_kw);
  o.default_station_price_per_kwh =
      num_or(j, "default_station_price_per_kwh", o.default_station_price_per_kwh);
  o.mip_gap = num_or(j, "mip_gap", o.mip_gap);
  o.time_limit_sec = num_or(j, "time_limit_sec", o.time_limit_sec);
  o.max_nodes = int_or(j, "max_nodes", o.max_nodes);
  o.use_ctm_travel_times = bool_or(j, "use_ctm_travel_times", o.use_ctm_travel_times);
  o.full_joint_prefer_ipopt =
      bool_or(j, "full_joint_prefer_ipopt", o.full_joint_prefer_ipopt);
  o.full_joint_equilibrium_penalty =
      num_or(j, "full_joint_equilibrium_penalty",
             o.full_joint_equilibrium_penalty);
  o.full_joint_ltm_pwl_segments =
      int_or(j, "full_joint_ltm_pwl_segments", o.full_joint_ltm_pwl_segments);
  o.full_joint_ltm_backward_wave_speed_fallback_km_hr =
      num_or(j, "full_joint_ltm_backward_wave_speed_fallback_km_hr",
             o.full_joint_ltm_backward_wave_speed_fallback_km_hr);
  if (j.contains("ctm_options") && j.at("ctm_options").is_object()) {
    o.ctm_opts = evpt_ctm_options_from_json(j.at("ctm_options"));
  }
  return o;
}

// ── Results ────────────────────────────────────────────────────────────────

json evpt_result_to_json(const EVPowerTrafficResult& r) {
  json out;
  out["feasible"] = r.feasible;
  out["status"] = r.status;
  out["warnings"] = r.warnings;
  out["total_demand_vehicles"] = r.total_demand_vehicles;
  out["served_demand_vehicles"] = r.served_demand_vehicles;
  out["unmet_demand_vehicles"] = r.unmet_demand_vehicles;
  out["total_requested_energy_kwh"] = r.total_requested_energy_kwh;
  out["total_delivered_energy_kwh"] = r.total_delivered_energy_kwh;
  out["total_v2g_energy_kwh"] = r.total_v2g_energy_kwh;
  out["total_unserved_energy_kwh"] = r.total_unserved_energy_kwh;
  out["total_travel_time_hr"] = r.total_travel_time_hr;
  out["mathematical_model_verified"] = r.mathematical_model_verified;
  out["mathematical_model_verification_status"] =
      r.mathematical_model_verification_status;

  json assigns = json::array();
  for (const auto& a : r.assignments) {
    assigns.push_back({{"demand_index", a.demand_index},
                       {"route_index", a.route_index},
                       {"departure_step", a.departure_step},
                       {"vehicles", a.vehicles},
                       {"generalized_cost", a.generalized_cost},
                       {"travel_time_hr", a.travel_time_hr},
                       {"served", a.served},
                       {"status", a.status}});
  }
  out["assignments"] = std::move(assigns);
  out["sessions"] = sessions_summary_to_json(r.sessions, r.session_results);
  return out;
}

json evpt_ctm_due_result_to_json(const CTMDUEResult& r,
                                 int max_link_step_cells) {
  json out;
  out["converged"] = r.converged;
  out["iterations"] = r.iterations;
  out["gap"] = r.gap;
  out["relative_gap"] = r.relative_gap;
  out["vi_gap"] = r.vi_gap;
  out["normalized_vi_gap"] = r.normalized_vi_gap;
  out["mathematical_model_verified"] = r.mathematical_model_verified;
  out["mathematical_model_verification_status"] =
      r.mathematical_model_verification_status;
  out["charging_dispatch_is_greedy"] = r.charging_dispatch_is_greedy;

  json hist = json::array();
  for (const auto& h : r.history) {
    hist.push_back({{"iteration", h.iteration},
                    {"gap", h.gap},
                    {"relative_gap", h.relative_gap}});
  }
  out["history"] = std::move(hist);

  out["flow_by_demand_route"] = flow_map_to_json(r.flow_by_demand_route);

  out["total_requested_energy_kwh"] = r.total_requested_energy_kwh;
  out["total_delivered_energy_kwh"] = r.total_delivered_energy_kwh;
  out["total_unserved_energy_kwh"] = r.total_unserved_energy_kwh;
  out["total_v2g_energy_kwh"] = r.total_v2g_energy_kwh;

  out["station_ev_load_kw"] = int_map_to_json(r.station_ev_load_kw);
  out["station_queue_veh"] = int_map_to_json(r.station_queue_veh);
  out["station_waiting_time_hr"] = int_map_to_json(r.station_waiting_time_hr);

  if (r.has_so_benchmark) {
    out["poa_tstt_ratio"] = r.poa_tstt_ratio;
    out["due_tstt_hr"] = r.due_tstt_hr;
    out["so_tstt_hr"] = r.so_tstt_hr;
  }

  out["route_travel_time"] = int_map_to_json(r.final_ctm.route_travel_time);
  out["ctm"] = ctm_link_series_to_json(r.final_ctm, max_link_step_cells);
  out["sessions"] = sessions_summary_to_json(r.sessions, r.session_results);
  return out;
}

json evpt_ctm_joint_result_to_json(const CTMJointWelfareResult& r,
                                   int max_link_step_cells) {
  json out;
  out["converged"] = r.converged;
  out["iterations"] = r.iterations;
  out["status"] = r.status;
  out["social_welfare"] = r.social_welfare;
  out["ev_gross_benefit"] = r.ev_gross_benefit;
  out["generation_cost"] = r.generation_cost;
  out["traffic_delay_cost"] = r.traffic_delay_cost;

  json hist = json::array();
  for (const auto& h : r.history) {
    hist.push_back({{"iteration", h.iteration},
                    {"social_welfare", h.social_welfare},
                    {"ev_gross_benefit", h.ev_gross_benefit},
                    {"generation_cost", h.generation_cost},
                    {"traffic_delay_cost", h.traffic_delay_cost},
                    {"price_change", h.price_change},
                    {"opf_converged", h.opf_converged},
                    {"due_converged", h.due_converged},
                    {"due_gap", h.due_gap}});
  }
  out["history"] = std::move(hist);

  out["station_prices_per_step"] = int_map_to_json(r.station_prices_per_step);

  json lmp = json::array();
  for (const auto& step_lmp : r.lmp_by_step) {
    lmp.push_back(int_map_to_json(step_lmp));
  }
  out["lmp_by_step"] = std::move(lmp);

  // Compact OPF summary; full DCOPFResult objects are intentionally omitted.
  json opf = json::array();
  for (const auto& o : r.opf_by_step) {
    opf.push_back({{"converged", o.converged}, {"objective", o.objective}});
  }
  out["opf_by_step"] = std::move(opf);

  out["ctm_due"] = evpt_ctm_due_result_to_json(r.final_ctm_due,
                                               max_link_step_cells);
  return out;
}

json evpt_joint_optimizer_result_to_json(const JointOptimizerResult& r) {
  json out;
  out["feasible"] = r.feasible;
  out["proven_optimal"] = r.proven_optimal;
  out["mathematical_model_verified"] = r.mathematical_model_verified;
  out["mathematical_model_verification_status"] =
      r.mathematical_model_verification_status;
  out["solver_backend"] = r.solver_backend;
  out["solver_status"] = r.solver_status;
  out["objective"] = r.objective;
  out["best_bound"] = r.best_bound;
  out["mip_gap"] = r.mip_gap;
  out["primal_max_violation"] = r.primal_max_violation;
  out["integrality_max_violation"] = r.integrality_max_violation;
  out["solve_time_sec"] = r.solve_time_sec;
  out["n_variables"] = r.n_variables;
  out["n_constraints"] = r.n_constraints;

  out["nonlinear_model"] = r.nonlinear_model;
  out["local_optimum_certificate"] = r.local_optimum_certificate;
  out["global_optimum_certificate"] = r.global_optimum_certificate;
  out["endogenous_congestion_enforced"] = r.endogenous_congestion_enforced;
  out["travel_times_are_endogenous"] = r.travel_times_are_endogenous;
  out["wardrop_complementarity_enforced"] = r.wardrop_complementarity_enforced;
  out["dcopf_coupling_enforced"] = r.dcopf_coupling_enforced;
  out["charging_v2g_enforced"] = r.charging_v2g_enforced;
  out["pwl_approximation_used"] = r.pwl_approximation_used;
  out["pwl_max_abs_error_bound"] = r.pwl_max_abs_error_bound;
  out["wardrop_gap"] = r.wardrop_gap;

  out["social_welfare"] = r.social_welfare;
  out["ev_benefit"] = r.ev_benefit;
  out["gen_cost"] = r.gen_cost;
  out["traffic_delay_cost"] = r.traffic_delay_cost;

  out["total_served_vehicles"] = r.total_served_vehicles;
  out["total_unserved_vehicles"] = r.total_unserved_vehicles;
  out["total_travel_time_hr"] = r.total_travel_time_hr;
  out["route_flow"] = flow_map_to_json(r.route_flow);

  out["total_requested_energy_kwh"] = r.total_requested_energy_kwh;
  out["total_delivered_energy_kwh"] = r.total_delivered_energy_kwh;
  out["total_v2g_energy_kwh"] = r.total_v2g_energy_kwh;
  out["total_unserved_energy_kwh"] = r.total_unserved_energy_kwh;

  json gen = json::array();
  for (const auto& step_gen : r.gen_dispatch_mw) {
    gen.push_back(int_map_to_json(step_gen));
  }
  out["gen_dispatch_mw"] = std::move(gen);

  // NOTE: lmp_by_step from Formulation D is a placeholder (dual extraction is
  // not implemented); it is intentionally NOT serialized so clients cannot
  // mistake empty maps for real prices.

  out["sessions"] = sessions_summary_to_json(r.sessions, r.session_results);
  out["warnings"] = r.warnings;
  return out;
}

}  // namespace hacdcpf::io
