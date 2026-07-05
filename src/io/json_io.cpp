#include "hacdcpf/io/json_io.hpp"

#include <algorithm>
#include <fstream>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/detail/internal_helpers.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

using json = nlohmann::json;

namespace hacdcpf::io {

struct PFDCBranchEstimate {
  double p_from_mw{0.0};
  double p_to_mw{0.0};
  double loss_mw{0.0};
  double loading_pct{0.0};
};

static double pf_ac_base_mva(const HybridPowerSystem& sys) {
  if (sys.base_mva > 1e-12) return sys.base_mva;
  if (sys.ac.base_mva > 1e-12) return sys.ac.base_mva;
  return 100.0;
}

static double pf_dc_base_mva(const HybridPowerSystem& sys) {
  if (sys.dc.base_mva > 1e-12) return sys.dc.base_mva;
  return pf_ac_base_mva(sys);
}

static double pf_ac_branch_rating_mva(const HybridPowerSystem& sys,
                                      const ACBranch& br) {
  if (br.rate_a_mva > 1e-12) return br.rate_a_mva;
  return pf_ac_base_mva(sys);
}

static double pf_dc_branch_rating_mva(const HybridPowerSystem& sys,
                                      const DCBranch& br) {
  if (br.s_max_mva > 1e-12) return br.s_max_mva;
  if (br.rate_a_mva > 1e-12) return br.rate_a_mva;
  return pf_dc_base_mva(sys);
}

static std::vector<PFDCBranchEstimate> pf_estimate_dc_branch_flows(
    const HybridPowerSystem& sys,
    const PowerFlowResult& result) {
  std::vector<PFDCBranchEstimate> estimates(sys.dc.branches.size());
  std::unordered_map<int, size_t> dc_bus_pos;
  dc_bus_pos.reserve(sys.dc.buses.size());
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    dc_bus_pos[sys.dc.buses[i].index] = i;
  }

  const double base_mva = pf_dc_base_mva(sys);
  for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
    const auto& br = sys.dc.branches[i];
    if (!br.in_service || br.r_pu <= 1e-12) continue;
    const auto from_it = dc_bus_pos.find(br.from_bus);
    const auto to_it = dc_bus_pos.find(br.to_bus);
    if (from_it == dc_bus_pos.end() || to_it == dc_bus_pos.end()) continue;

    const size_t from_pos = from_it->second;
    const size_t to_pos = to_it->second;
    const double v_from = (from_pos < result.vdc.size())
                              ? result.vdc[from_pos]
                              : sys.dc.buses[from_pos].vm_pu;
    const double v_to = (to_pos < result.vdc.size())
                            ? result.vdc[to_pos]
                            : sys.dc.buses[to_pos].vm_pu;
    const double i_pu = (v_from - v_to) / br.r_pu;
    const double p_from_mw = base_mva * v_from * i_pu;
    const double p_to_mw = -base_mva * v_to * i_pu;
    const double loss_mw = std::max(p_from_mw + p_to_mw, 0.0);
    double loading_pct = 0.0;
    const double rating = pf_dc_branch_rating_mva(sys, br);
    if (rating > 1e-12) {
      loading_pct =
          100.0 * std::max(std::abs(p_from_mw), std::abs(p_to_mw)) / rating;
    }
    estimates[i] = {p_from_mw, p_to_mw, loss_mw, loading_pct};
  }
  return estimates;
}

// ── Helper: safe get with default ──────────────────────────────────────
template <typename T>
static T jget(const json& j, const std::string& key, T def) {
  if (j.contains(key) && !j[key].is_null()) return j[key].get<T>();
  return def;
}

template <typename T>
static T jget_alias(const json& j,
                    const std::string& primary_key,
                    const std::string& alias_key,
                    T def) {
  if (j.contains(primary_key) && !j[primary_key].is_null()) {
    return j[primary_key].get<T>();
  }
  if (j.contains(alias_key) && !j[alias_key].is_null()) {
    return j[alias_key].get<T>();
  }
  return def;
}

static double normalize_loss_mw_from_json(const json& j) {
  if (j.contains("loss_mw") && !j["loss_mw"].is_null()) {
    return j["loss_mw"].get<double>();
  }
  if (j.contains("loss_kw") && !j["loss_kw"].is_null()) {
    return j["loss_kw"].get<double>() / 1000.0;
  }
  return 0.0;
}

static void reconcile_for_mtbf(double& forced_outage_rate,
                               double& mttr_hr,
                               double& mtbf_hr) {
  if (mtbf_hr <= 0.0 && mttr_hr > 0.0 && forced_outage_rate > 0.0 && forced_outage_rate < 1.0) {
    mtbf_hr = mttr_hr * (1.0 - forced_outage_rate) / forced_outage_rate;
  }
  if ((forced_outage_rate <= 0.0 || forced_outage_rate >= 1.0) && mtbf_hr > 0.0 && mttr_hr > 0.0) {
    forced_outage_rate = mttr_hr / (mtbf_hr + mttr_hr);
  }
}

static bool dynamic_model_component_empty(
    const DynamicModelComponentProfile& profile) {
  return profile.type.empty() && profile.model.empty() &&
         profile.standard.empty() && profile.parameter_set.empty() &&
         profile.parameters.empty();
}

static json dynamic_model_component_to_json(
    const DynamicModelComponentProfile& profile) {
  json j;
  if (!profile.type.empty()) j["type"] = profile.type;
  if (!profile.model.empty()) j["model"] = profile.model;
  if (!profile.standard.empty()) j["standard"] = profile.standard;
  if (!profile.parameter_set.empty()) {
    j["parameter_set"] = profile.parameter_set;
  }
  if (!profile.parameters.empty()) j["parameters"] = profile.parameters;
  return j;
}

static DynamicModelComponentProfile dynamic_model_component_from_json(
    const json& j) {
  DynamicModelComponentProfile profile;
  if (!j.is_object()) return profile;
  profile.type = jget<std::string>(j, "type", "");
  profile.model = jget_alias<std::string>(j, "model", "model_name", "");
  profile.standard = jget<std::string>(j, "standard", "");
  profile.parameter_set = jget<std::string>(j, "parameter_set", "");
  if (j.contains("parameters") && j["parameters"].is_object()) {
    for (const auto& [key, value] : j["parameters"].items()) {
      if (value.is_number()) {
        profile.parameters[key] = value.get<double>();
      }
    }
  }
  return profile;
}

static json dynamic_model_to_json(const DynamicModelProfile& profile) {
  json j;
  if (!profile.standard.empty()) j["standard"] = profile.standard;
  if (!profile.model_name.empty()) j["model_name"] = profile.model_name;
  if (!profile.parameter_set.empty()) {
    j["parameter_set"] = profile.parameter_set;
  }
  if (!profile.source_id.empty()) j["source_id"] = profile.source_id;
  if (!profile.notes.empty()) j["notes"] = profile.notes;
  if (!profile.components.empty()) {
    j["components"] = json::array();
    for (const auto& component : profile.components) {
      if (!dynamic_model_component_empty(component)) {
        j["components"].push_back(dynamic_model_component_to_json(component));
      }
    }
  }
  if (!profile.parameters.empty()) j["parameters"] = profile.parameters;
  return j;
}

static DynamicModelProfile dynamic_model_from_json(const json& j) {
  DynamicModelProfile profile;
  if (j.is_string()) {
    profile.model_name = j.get<std::string>();
    return profile;
  }
  if (!j.is_object()) return profile;
  profile.standard = jget<std::string>(j, "standard", "");
  profile.model_name = jget_alias<std::string>(j, "model_name", "model", "");
  profile.parameter_set = jget<std::string>(j, "parameter_set", "");
  profile.source_id = jget<std::string>(j, "source_id", "");
  profile.notes = jget<std::string>(j, "notes", "");
  if (j.contains("components") && j["components"].is_array()) {
    for (const auto& component : j["components"]) {
      auto parsed = dynamic_model_component_from_json(component);
      if (!dynamic_model_component_empty(parsed)) {
        profile.components.push_back(std::move(parsed));
      }
    }
  }
  if (j.contains("parameters") && j["parameters"].is_object()) {
    for (const auto& [key, value] : j["parameters"].items()) {
      if (value.is_number()) {
        profile.parameters[key] = value.get<double>();
      }
    }
  }
  return profile;
}

static void add_dynamic_model_if_present(json& j,
                                         const DynamicModelProfile& profile) {
  if (!profile.empty()) j["dynamic_model"] = dynamic_model_to_json(profile);
}

static void read_dynamic_model_if_present(const json& j,
                                          DynamicModelProfile& profile) {
  if (j.contains("dynamic_model") && !j["dynamic_model"].is_null()) {
    profile = dynamic_model_from_json(j["dynamic_model"]);
  }
}

// ═══════════════════════════════════════════════════════════════════════
// To JSON
// ═══════════════════════════════════════════════════════════════════════
static json ac_bus_to_json(const ACBus& b) {
  json j;
  j["index"] = b.index;
  j["bus_type"] = bus_type_str(b.bus_type);
  j["pd_mw"] = b.pd_mw;
  j["qd_mvar"] = b.qd_mvar;
  j["vm_pu"] = b.vm_pu;
  j["va_deg"] = b.va_deg;
  j["area"] = b.area;
  j["base_kv"] = b.base_kv;
  j["vmax_pu"] = b.vmax_pu;
  j["vmin_pu"] = b.vmin_pu;
  j["gs_mw"] = b.gs_mw;
  j["bs_mvar"] = b.bs_mvar;
  j["zone"] = b.zone;
  j["in_service"] = b.in_service;
  j["name"] = b.name;
  j["n_customers"] = b.n_customers;
  j["importance"] = b.importance;
  j["latitude"] = b.latitude;
  j["longitude"] = b.longitude;
  return j;
}

// Reads "bus_type" accepting both MATPOWER integer codes (1=PQ,2=PV,3=SLACK,4=ISOLATED)
// and string names, returning the canonical string for bus_type_from_str().
static std::string jget_ac_bus_type_str(const json& j) {
  if (!j.contains("bus_type") || j["bus_type"].is_null()) return "PQ";
  const auto& v = j["bus_type"];
  if (v.is_string()) return v.get<std::string>();
  if (v.is_number_integer()) {
    switch (v.get<int>()) {
      case 2: return "PV";
      case 3: return "SLACK";
      case 4: return "ISOLATED";
      default: return "PQ";
    }
  }
  return "PQ";
}

// Same for DC buses: MATPOWER codes 1=DC_P, 3=DC_V, 4=DC_ISOLATED;
// JPC codes 1=DC_P, 2=DC_V, 4=DC_ISOLATED.
static std::string jget_dc_bus_type_str(const json& j) {
  if (!j.contains("bus_type") || j["bus_type"].is_null()) return "DC_P";
  const auto& v = j["bus_type"];
  if (v.is_string()) return v.get<std::string>();
  if (v.is_number_integer()) {
    const int code = v.get<int>();
    // Accept both MATPOWER (3=DC_V) and JPC (2=DC_V) integer conventions;
    // 4 denotes an isolated/de-energized DC bus in both conventions.
    if (code == 4) return "DC_ISOLATED";
    return (code == 2 || code == 3) ? "DC_V" : "DC_P";
  }
  return "DC_P";
}

static ACBus ac_bus_from_json(const json& j) {
  ACBus b;
  b.index = j.at("index").get<int>();
  b.bus_type = bus_type_from_str(jget_ac_bus_type_str(j));
  b.pd_mw = jget(j, "pd_mw", 0.0);
  b.qd_mvar = jget(j, "qd_mvar", 0.0);
  b.vm_pu = jget(j, "vm_pu", 1.0);
  b.va_deg = jget(j, "va_deg", 0.0);
  b.area = jget(j, "area", 1);
  b.base_kv = jget(j, "base_kv", 110.0);
  b.vmax_pu = jget(j, "vmax_pu", 1.1);
  b.vmin_pu = jget(j, "vmin_pu", 0.9);
  b.gs_mw = jget(j, "gs_mw", 0.0);
  b.bs_mvar = jget(j, "bs_mvar", 0.0);
  b.zone = jget(j, "zone", 1);
  b.in_service = jget(j, "in_service", true);
  b.name = jget<std::string>(j, "name", "");
  b.n_customers = jget(j, "n_customers", 0);
  b.importance = jget(j, "importance", 1.0);
  b.latitude = jget_alias(j, "latitude", "lat", 0.0);
  b.longitude = jget_alias(j, "longitude", "lon", 0.0);
  return b;
}

static json ac_branch_to_json(const ACBranch& br) {
  json j;
  j["index"] = br.index;
  j["from_bus"] = br.from_bus;
  j["to_bus"] = br.to_bus;
  j["r_pu"] = br.r_pu;
  j["x_pu"] = br.x_pu;
  j["b_pu"] = br.b_pu;
  j["tap"] = br.tap;
  j["shift_deg"] = br.shift_deg;
  j["rate_a_mva"] = br.rate_a_mva;
  j["rate_b_mva"] = br.rate_b_mva;
  j["rate_c_mva"] = br.rate_c_mva;
  j["in_service"] = br.in_service;
  j["name"] = br.name;
  j["length_km"] = br.length_km;
  j["r0_pu"] = br.r0_pu;
  j["x0_pu"] = br.x0_pu;
  j["b0_pu"] = br.b0_pu;
  j["failure_rate"] = br.failure_rate;
  j["mttr_hr"] = br.mttr_hr;
  j["dynamic_rl"] = br.dynamic_rl;
  return j;
}

static ACBranch ac_branch_from_json(const json& j) {
  ACBranch br;
  br.index = j.at("index").get<int>();
  br.from_bus = j.at("from_bus").get<int>();
  br.to_bus = j.at("to_bus").get<int>();
  br.r_pu = jget(j, "r_pu", 0.0);
  br.x_pu = jget(j, "x_pu", 0.0);
  br.b_pu = jget(j, "b_pu", 0.0);
  br.tap = jget(j, "tap", 1.0);
  br.shift_deg = jget(j, "shift_deg", 0.0);
  br.rate_a_mva = jget(j, "rate_a_mva", 0.0);
  br.rate_b_mva = jget(j, "rate_b_mva", 0.0);
  br.rate_c_mva = jget(j, "rate_c_mva", 0.0);
  br.in_service = jget(j, "in_service", true);
  br.name = jget<std::string>(j, "name", "");
  br.length_km = jget(j, "length_km", 0.0);
  br.r0_pu = jget(j, "r0_pu", 0.0);
  br.x0_pu = jget(j, "x0_pu", 0.0);
  br.b0_pu = jget(j, "b0_pu", 0.0);
  br.failure_rate = jget(j, "failure_rate", 0.0);
  br.mttr_hr = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  br.dynamic_rl = jget(j, "dynamic_rl", false);
  return br;
}

static json dc_bus_to_json(const DCBus& b) {
  json j;
  j["index"] = b.index;
  j["bus_type"] = dc_bus_type_str(b.bus_type);
  j["vm_pu"] = b.vm_pu;
  j["vmax_pu"] = b.vmax_pu;
  j["vmin_pu"] = b.vmin_pu;
  j["base_kv"] = b.base_kv;
  j["pd_mw"] = b.pd_mw;
  j["in_service"] = b.in_service;
  j["name"] = b.name;
  j["latitude"] = b.latitude;
  j["longitude"] = b.longitude;
  return j;
}

static DCBus dc_bus_from_json(const json& j) {
  DCBus b;
  b.index = j.at("index").get<int>();
  b.bus_type = dc_bus_type_from_str(jget_dc_bus_type_str(j));
  b.vm_pu = jget(j, "vm_pu", 1.0);
  b.vmax_pu = jget(j, "vmax_pu", 1.1);
  b.vmin_pu = jget(j, "vmin_pu", 0.9);
  b.base_kv = jget(j, "base_kv", 0.0);
  b.pd_mw = jget(j, "pd_mw", 0.0);
  b.in_service = jget(j, "in_service", true);
  b.name = jget<std::string>(j, "name", "");
  b.latitude = jget_alias(j, "latitude", "lat", 0.0);
  b.longitude = jget_alias(j, "longitude", "lon", 0.0);
  return b;
}

static json dc_branch_to_json(const DCBranch& br) {
  json j;
  j["index"] = br.index;
  j["from_bus"] = br.from_bus;
  j["to_bus"] = br.to_bus;
  j["r_pu"] = br.r_pu;
  j["rate_a_mva"] = br.rate_a_mva;
  j["in_service"] = br.in_service;
  j["name"] = br.name;
  j["length_km"] = br.length_km;
  j["base_kv"] = br.base_kv;
  j["r_ohm_per_km"] = br.r_ohm_per_km;
  return j;
}

static DCBranch dc_branch_from_json(const json& j) {
  DCBranch br;
  br.index = j.at("index").get<int>();
  br.from_bus = j.at("from_bus").get<int>();
  br.to_bus = j.at("to_bus").get<int>();
  br.r_pu = jget(j, "r_pu", 0.0);
  br.rate_a_mva = jget(j, "rate_a_mva", 0.0);
  br.in_service = jget(j, "in_service", true);
  br.name = jget<std::string>(j, "name", "");
  br.length_km = jget(j, "length_km", 0.0);
  br.base_kv = jget(j, "base_kv", 0.0);
  br.r_ohm_per_km = jget(j, "r_ohm_per_km", 0.0);
  return br;
}

static json dc_load_to_json(const DCLoad& l) {
  json j;
  j["index"] = l.index;
  j["bus"] = l.bus;
  j["in_service"] = l.in_service;
  j["name"] = l.name;
  j["p_mw"] = l.p_mw;
  j["p_rated_mw"] = l.p_rated_mw;
  j["scaling"] = l.scaling;
  j["controllable"] = l.controllable;
  j["p_min_mw"] = l.p_min_mw;
  j["cost_mw"] = l.cost_mw;
  j["profile_id"] = l.profile_id;
  j["n_customers"] = l.n_customers;
  add_dynamic_model_if_present(j, l.dynamic_model);
  return j;
}

static DCLoad dc_load_from_json(const json& j) {
  DCLoad l;
  l.index = j.at("index").get<int>();
  l.bus = j.at("bus").get<int>();
  l.in_service = jget(j, "in_service", true);
  l.name = jget<std::string>(j, "name", "");
  l.p_mw = jget(j, "p_mw", 0.0);
  l.p_rated_mw = jget(j, "p_rated_mw", 0.0);
  l.scaling = jget(j, "scaling", 1.0);
  l.controllable = jget(j, "controllable", false);
  l.p_min_mw = jget(j, "p_min_mw", 0.0);
  l.cost_mw = jget(j, "cost_mw", 0.0);
  l.profile_id = jget(j, "profile_id", -1);
  l.n_customers = jget(j, "n_customers", 0);
  read_dynamic_model_if_present(j, l.dynamic_model);
  return l;
}

static json dc_static_generator_to_json(const StaticGeneratorDC& g) {
  json j;
  j["index"] = g.index;
  j["bus"] = g.bus;
  j["in_service"] = g.in_service;
  j["name"] = g.name;
  j["type"] = g.type;
  j["p_set_mw"] = g.p_set_mw;
  j["scaling"] = g.scaling;
  j["profile_id"] = g.profile_id;
  j["pmax_mw"] = g.pmax_mw;
  j["pmin_mw"] = g.pmin_mw;
  j["controllable"] = g.controllable;
  j["mtbf_hours"] = g.mtbf_hours;
  j["mttr_hours"] = g.mttr_hours;
  j["t_scheduled_hr"] = g.t_scheduled_hr;
  add_dynamic_model_if_present(j, g.dynamic_model);
  return j;
}

static StaticGeneratorDC dc_static_generator_from_json(const json& j) {
  StaticGeneratorDC g;
  g.index = j.at("index").get<int>();
  g.bus = j.at("bus").get<int>();
  g.in_service = jget(j, "in_service", true);
  g.name = jget<std::string>(j, "name", "");
  g.type = jget<std::string>(j, "type", "");
  g.p_set_mw = jget(j, "p_set_mw", 0.0);
  g.scaling = jget(j, "scaling", 1.0);
  g.profile_id = jget(j, "profile_id", -1);
  g.pmax_mw = jget(j, "pmax_mw", 0.0);
  g.pmin_mw = jget(j, "pmin_mw", 0.0);
  g.controllable = jget(j, "controllable", false);
  g.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  g.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  g.t_scheduled_hr = jget(j, "t_scheduled_hr", 0.0);
  read_dynamic_model_if_present(j, g.dynamic_model);
  return g;
}

static json pv_array_dc_to_json(const PVArrayDC& p) {
  json j;
  j["index"] = p.index;
  j["bus"] = p.bus;
  j["in_service"] = p.in_service;
  j["name"] = p.name;
  j["p_set_mw"] = p.p_set_mw;
  j["profile_id"] = p.profile_id;
  j["num_series"] = p.num_series;
  j["num_parallel"] = p.num_parallel;
  j["vmpp"] = p.vmpp;
  j["impp"] = p.impp;
  j["voc"] = p.voc;
  j["isc"] = p.isc;
  j["alpha_isc"] = p.alpha_isc;
  j["beta_voc"] = p.beta_voc;
  j["temperature"] = p.temperature;
  j["irradiance"] = p.irradiance;
  j["mtbf_hours"] = p.mtbf_hours;
  j["mttr_hours"] = p.mttr_hours;
  j["t_scheduled_hr"] = p.t_scheduled_hr;
  add_dynamic_model_if_present(j, p.dynamic_model);
  return j;
}

static PVArrayDC pv_array_dc_from_json(const json& j) {
  PVArrayDC p;
  p.index = j.at("index").get<int>();
  p.bus = j.at("bus").get<int>();
  p.in_service = jget(j, "in_service", true);
  p.name = jget<std::string>(j, "name", "");
  p.p_set_mw = jget(j, "p_set_mw", 0.0);
  p.profile_id = jget(j, "profile_id", -1);
  p.num_series = jget(j, "num_series", 0);
  p.num_parallel = jget(j, "num_parallel", 0);
  p.vmpp = jget(j, "vmpp", 0.0);
  p.impp = jget(j, "impp", 0.0);
  p.voc = jget(j, "voc", 0.0);
  p.isc = jget(j, "isc", 0.0);
  p.alpha_isc = jget(j, "alpha_isc", 0.0);
  p.beta_voc = jget(j, "beta_voc", 0.0);
  p.temperature = jget(j, "temperature", 25.0);
  p.irradiance = jget(j, "irradiance", 1000.0);
  p.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  p.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  p.t_scheduled_hr = jget(j, "t_scheduled_hr", 0.0);
  read_dynamic_model_if_present(j, p.dynamic_model);
  return p;
}

static json generator_to_json(const Generator& g) {
  json j;
  j["index"] = g.index;
  j["bus"] = g.bus;
  j["in_service"] = g.in_service;
  j["pg_mw"] = g.pg_mw;
  j["qg_mvar"] = g.qg_mvar;
  j["vg_pu"] = g.vg_pu;
  j["pmax_mw"] = g.pmax_mw;
  j["pmin_mw"] = g.pmin_mw;
  j["qmax_mvar"] = g.qmax_mvar;
  j["qmin_mvar"] = g.qmin_mvar;
  j["is_slack"] = g.is_slack;
  j["cost_c2"] = g.cost_c2;
  j["cost_c1"] = g.cost_c1;
  j["cost_c0"] = g.cost_c0;
  j["name"] = g.name;
  j["fuel_type"] = fuel_type_str(g.fuel_type);
  j["startup_cost"] = g.startup_cost;
  j["shutdown_cost"] = g.shutdown_cost;
  j["min_up_time_hr"] = g.min_up_time_hr;
  j["min_dn_time_hr"] = g.min_dn_time_hr;
  j["ramp_up_mw_min"] = g.ramp_up_mw_min;
  j["ramp_dn_mw_min"] = g.ramp_dn_mw_min;
  j["mbase_mva"] = g.mbase_mva;
  j["inertia_h"] = g.inertia_h;
  j["droop_r"] = g.droop_r;
  j["xd_pu"] = g.xd_pu;
  j["xdp_pu"] = g.xdp_pu;
  j["xdpp_pu"] = g.xdpp_pu;
  j["td0p_s"] = g.td0p_s;
  j["td0pp_s"] = g.td0pp_s;
  j["emission_factor_tco2_mwh"] = g.emission_factor_tco2_mwh;
  j["nox_factor_kg_mwh"] = g.nox_factor_kg_mwh;
  j["so2_factor_kg_mwh"] = g.so2_factor_kg_mwh;
  j["profile_id"] = g.profile_id;
  j["forced_outage_rate"] = g.forced_outage_rate;
  j["mttr_hr"] = g.mttr_hr;
  add_dynamic_model_if_present(j, g.dynamic_model);
  return j;
}

static Generator generator_from_json(const json& j) {
  Generator g;
  g.index = j.at("index").get<int>();
  g.bus = j.at("bus").get<int>();
  g.in_service = jget(j, "in_service", true);
  g.pg_mw = jget(j, "pg_mw", 0.0);
  g.qg_mvar = jget(j, "qg_mvar", 0.0);
  g.vg_pu = jget(j, "vg_pu", 1.0);
  g.pmax_mw = jget(j, "pmax_mw", 0.0);
  g.pmin_mw = jget(j, "pmin_mw", 0.0);
  g.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  g.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  g.is_slack = jget(j, "is_slack", false);
  g.cost_c2 = jget(j, "cost_c2", 0.0);
  g.cost_c1 = jget(j, "cost_c1", 0.0);
  g.cost_c0 = jget(j, "cost_c0", 0.0);
  g.name = jget<std::string>(j, "name", "");
  g.fuel_type = fuel_type_from_str(jget<std::string>(j, "fuel_type", "Unknown"));
  g.startup_cost = jget(j, "startup_cost", 0.0);
  g.shutdown_cost = jget(j, "shutdown_cost", 0.0);
  g.min_up_time_hr = jget(j, "min_up_time_hr", 0.0);
  g.min_dn_time_hr = jget(j, "min_dn_time_hr", 0.0);
  g.ramp_up_mw_min = jget(j, "ramp_up_mw_min", 0.0);
  g.ramp_dn_mw_min = jget(j, "ramp_dn_mw_min", 0.0);
  g.mbase_mva = jget(j, "mbase_mva", 0.0);
  g.inertia_h = jget(j, "inertia_h", 0.0);
  g.droop_r = jget(j, "droop_r", 0.05);
  g.xd_pu = jget(j, "xd_pu", 0.0);
  g.xdp_pu = jget(j, "xdp_pu", 0.0);
  g.xdpp_pu = jget(j, "xdpp_pu", 0.0);
  g.td0p_s = jget(j, "td0p_s", 0.0);
  g.td0pp_s = jget(j, "td0pp_s", 0.0);
  g.emission_factor_tco2_mwh = jget_alias(j, "emission_factor_tco2_mwh", "co2_emission_rate", 0.0);
  g.nox_factor_kg_mwh = jget(j, "nox_factor_kg_mwh", 0.0);
  g.so2_factor_kg_mwh = jget(j, "so2_factor_kg_mwh", 0.0);
  g.profile_id = jget(j, "profile_id", -1);
  g.forced_outage_rate = jget(j, "forced_outage_rate", 0.0);
  g.mttr_hr = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, g.dynamic_model);
  return g;
}

static json load_to_json(const Load& l) {
  json j;
  j["index"] = l.index;
  j["bus"] = l.bus;
  j["in_service"] = l.in_service;
  j["name"] = l.name;
  j["p_mw"] = l.p_mw;
  j["q_mvar"] = l.q_mvar;
  j["model"] = load_model_str(l.model);
  j["z_percent_p"] = l.z_percent_p;
  j["i_percent_p"] = l.i_percent_p;
  j["p_percent_p"] = l.p_percent_p;
  j["z_percent_q"] = l.z_percent_q;
  j["i_percent_q"] = l.i_percent_q;
  j["p_percent_q"] = l.p_percent_q;
  j["controllable"] = l.controllable;
  j["p_min_mw"] = l.p_min_mw;
  j["cost_mw"] = l.cost_mw;
  j["priority"] = load_priority_str(l.priority);
  j["n_customers"] = l.n_customers;
  j["profile_id"] = l.profile_id;
  j["sn_mva"] = l.sn_mva;
  j["motor_percent"] = l.motor_percent;
  j["x_sub_pu"] = l.x_sub_pu;
  j["r_sc_pu"] = l.r_sc_pu;
  j["sc_source_type"] = l.sc_source_type;
  j["sc_source_index"] = l.sc_source_index;
  j["motor_poles"] = l.motor_poles;
  j["motor_efficiency"] = l.motor_efficiency;
  add_dynamic_model_if_present(j, l.dynamic_model);
  return j;
}

static Load load_from_json(const json& j) {
  Load l;
  l.index = j.at("index").get<int>();
  l.bus = j.at("bus").get<int>();
  l.in_service = jget(j, "in_service", true);
  l.name = jget<std::string>(j, "name", "");
  l.p_mw = jget(j, "p_mw", 0.0);
  l.q_mvar = jget(j, "q_mvar", 0.0);
  l.model = load_model_from_str(jget<std::string>(j, "model", "ConstantPower"));
  l.z_percent_p = jget(j, "z_percent_p", 0.0);
  l.i_percent_p = jget(j, "i_percent_p", 0.0);
  l.p_percent_p = jget(j, "p_percent_p", 100.0);
  l.z_percent_q = jget(j, "z_percent_q", 0.0);
  l.i_percent_q = jget(j, "i_percent_q", 0.0);
  l.p_percent_q = jget(j, "p_percent_q", 100.0);
  l.controllable = jget(j, "controllable", false);
  l.p_min_mw = jget(j, "p_min_mw", 0.0);
  l.cost_mw = jget(j, "cost_mw", 0.0);
  l.priority = load_priority_from_str(jget<std::string>(j, "priority", "Medium"));
  l.n_customers = jget(j, "n_customers", 0);
  l.profile_id = jget(j, "profile_id", -1);
  l.sn_mva = jget(j, "sn_mva", 0.0);
  l.motor_percent = jget(j, "motor_percent", 0.0);
  l.x_sub_pu = jget(j, "x_sub_pu", 0.0);
  l.r_sc_pu = jget(j, "r_sc_pu", 0.0);
  l.sc_source_type = jget<std::string>(j, "sc_source_type", "");
  l.sc_source_index = jget(j, "sc_source_index", 0);
  l.motor_poles = jget(j, "motor_poles", 2);
  l.motor_efficiency = jget(j, "motor_efficiency", 0.95);
  read_dynamic_model_if_present(j, l.dynamic_model);
  return l;
}

static json shunt_to_json(const Shunt& s) {
  json j;
  j["index"] = s.index;
  j["bus"] = s.bus;
  j["in_service"] = s.in_service;
  j["name"] = s.name;
  j["gs_mw"] = s.gs_mw;
  j["bs_mvar"] = s.bs_mvar;
  j["switchable"] = s.switchable;
  j["n_steps"] = s.n_steps;
  j["current_step"] = s.current_step;
  j["bs_per_step"] = s.bs_per_step;
  return j;
}

static Shunt shunt_from_json(const json& j) {
  Shunt s;
  s.index = j.at("index").get<int>();
  s.bus = j.at("bus").get<int>();
  s.in_service = jget(j, "in_service", true);
  s.name = jget<std::string>(j, "name", "");
  s.gs_mw = jget(j, "gs_mw", 0.0);
  s.bs_mvar = jget(j, "bs_mvar", 0.0);
  s.switchable = jget(j, "switchable", false);
  s.n_steps = jget(j, "n_steps", 1);
  s.current_step = jget(j, "current_step", 1);
  s.bs_per_step = jget(j, "bs_per_step", 0.0);
  return s;
}

static json storage_to_json(const Storage& st) {
  json j;
  j["index"] = st.index;
  j["bus"] = st.bus;
  j["in_service"] = st.in_service;
  j["name"] = st.name;
  j["p_mw"] = st.p_mw;
  j["q_mvar"] = st.q_mvar;
  j["p_rated_mw"] = st.p_rated_mw;
  j["pmax_mw"] = st.pmax_mw;
  j["pmin_mw"] = st.pmin_mw;
  j["qmax_mvar"] = st.qmax_mvar;
  j["qmin_mvar"] = st.qmin_mvar;
  j["e_rated_mwh"] = st.e_rated_mwh;
  j["soc_init"] = st.soc_init;
  j["soc_min"] = st.soc_min;
  j["soc_max"] = st.soc_max;
  j["soc_carbon_intensity_tco2_mwh"] = st.soc_carbon_intensity_tco2_mwh;
  j["eta_charge"] = st.eta_charge;
  j["eta_discharge"] = st.eta_discharge;
  j["self_discharge_pct"] = st.self_discharge_pct;
  j["max_cycles"] = st.max_cycles;
  j["current_cycles"] = st.current_cycles;
  j["soh"] = st.soh;
  j["replacement_cost"] = st.replacement_cost;
  j["profile_id"] = st.profile_id;
  j["forced_outage_rate"] = st.forced_outage_rate;
  j["mttr_hr"] = st.mttr_hr;
  add_dynamic_model_if_present(j, st.dynamic_model);
  return j;
}

static Storage storage_from_json(const json& j) {
  Storage st;
  st.index = j.at("index").get<int>();
  st.bus = j.at("bus").get<int>();
  st.in_service = jget(j, "in_service", true);
  st.name = jget<std::string>(j, "name", "");
  st.p_mw = jget(j, "p_mw", 0.0);
  st.q_mvar = jget(j, "q_mvar", 0.0);
  st.p_rated_mw = jget(j, "p_rated_mw", 0.0);
  st.pmax_mw = jget(j, "pmax_mw", 0.0);
  st.pmin_mw = jget(j, "pmin_mw", 0.0);
  st.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  st.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  st.e_rated_mwh = jget(j, "e_rated_mwh", 0.0);
  st.soc_init = jget(j, "soc_init", 0.5);
  st.soc_min = jget(j, "soc_min", 0.1);
  st.soc_max = jget(j, "soc_max", 0.9);
  st.soc_carbon_intensity_tco2_mwh = jget(j, "soc_carbon_intensity_tco2_mwh", 0.0);
  st.eta_charge = jget(j, "eta_charge", 0.95);
  st.eta_discharge = jget(j, "eta_discharge", 0.95);
  st.self_discharge_pct = jget(j, "self_discharge_pct", 0.0);
  st.max_cycles = jget(j, "max_cycles", 5000);
  st.current_cycles = jget(j, "current_cycles", 0);
  st.soh = jget(j, "soh", 1.0);
  st.replacement_cost = jget(j, "replacement_cost", 0.0);
  st.profile_id = jget(j, "profile_id", -1);
  st.forced_outage_rate = jget(j, "forced_outage_rate", 0.0);
  st.mttr_hr = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, st.dynamic_model);
  return st;
}

// DC-side storage: active-power only (no reactive fields).
static json dc_storage_to_json(const DCStorage& st) {
  json j;
  j["index"] = st.index;
  j["bus"] = st.bus;
  j["in_service"] = st.in_service;
  j["name"] = st.name;
  j["type"] = st.type;
  j["p_mw"] = st.p_mw;
  j["p_rated_mw"] = st.p_rated_mw;
  j["pmax_mw"] = st.pmax_mw;
  j["pmin_mw"] = st.pmin_mw;
  j["e_rated_mwh"] = st.e_rated_mwh;
  j["soc_init"] = st.soc_init;
  j["soc_min"] = st.soc_min;
  j["soc_max"] = st.soc_max;
  j["soc_carbon_intensity_tco2_mwh"] = st.soc_carbon_intensity_tco2_mwh;
  j["eta_charge"] = st.eta_charge;
  j["eta_discharge"] = st.eta_discharge;
  j["self_discharge_pct"] = st.self_discharge_pct;
  j["max_cycles"] = st.max_cycles;
  j["current_cycles"] = st.current_cycles;
  j["soh"] = st.soh;
  j["replacement_cost"] = st.replacement_cost;
  j["profile_id"] = st.profile_id;
  j["controllable"] = st.controllable;
  j["forced_outage_rate"] = st.forced_outage_rate;
  j["mttr_hr"] = st.mttr_hr;
  add_dynamic_model_if_present(j, st.dynamic_model);
  return j;
}

static DCStorage dc_storage_from_json(const json& j) {
  DCStorage st;
  st.index = j.at("index").get<int>();
  st.bus = j.at("bus").get<int>();
  st.in_service = jget(j, "in_service", true);
  st.name = jget<std::string>(j, "name", "");
  st.type = jget<std::string>(j, "type", "");
  st.p_mw = jget(j, "p_mw", 0.0);
  st.p_rated_mw = jget(j, "p_rated_mw", 0.0);
  st.pmax_mw = jget(j, "pmax_mw", 0.0);
  st.pmin_mw = jget(j, "pmin_mw", 0.0);
  st.e_rated_mwh = jget(j, "e_rated_mwh", 0.0);
  st.soc_init = jget(j, "soc_init", 0.5);
  st.soc_min = jget(j, "soc_min", 0.1);
  st.soc_max = jget(j, "soc_max", 0.9);
  st.soc_carbon_intensity_tco2_mwh = jget(j, "soc_carbon_intensity_tco2_mwh", 0.0);
  st.eta_charge = jget(j, "eta_charge", 0.95);
  st.eta_discharge = jget(j, "eta_discharge", 0.95);
  st.self_discharge_pct = jget(j, "self_discharge_pct", 0.0);
  st.max_cycles = jget(j, "max_cycles", 5000);
  st.current_cycles = jget(j, "current_cycles", 0);
  st.soh = jget(j, "soh", 1.0);
  st.replacement_cost = jget(j, "replacement_cost", 0.0);
  st.profile_id = jget(j, "profile_id", -1);
  st.controllable = jget(j, "controllable", true);
  st.forced_outage_rate = jget(j, "forced_outage_rate", 0.0);
  st.mttr_hr = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, st.dynamic_model);
  return st;
}

static json renewable_gen_to_json(const RenewableGen& r) {
  json j;
  j["index"] = r.index;
  j["bus"] = r.bus;
  j["in_service"] = r.in_service;
  j["name"] = r.name;
  j["type"] = renewable_type_str(r.type);
  j["p_mw"] = r.p_mw;
  j["q_mvar"] = r.q_mvar;
  j["p_rated_mw"] = r.p_rated_mw;
  j["qmax_mvar"] = r.qmax_mvar;
  j["qmin_mvar"] = r.qmin_mvar;
  j["curtailable"] = r.curtailable;
  j["cost_curtail_mwh"] = r.cost_curtail_mwh;
  j["capacity_factor"] = r.capacity_factor;
  j["profile_id"] = r.profile_id;
  j["emission_offset_tco2_mwh"] = r.emission_offset_tco2_mwh;
  add_dynamic_model_if_present(j, r.dynamic_model);
  return j;
}

static RenewableGen renewable_gen_from_json(const json& j) {
  RenewableGen r;
  r.index = j.at("index").get<int>();
  r.bus = j.at("bus").get<int>();
  r.in_service = jget(j, "in_service", true);
  r.name = jget<std::string>(j, "name", "");
  r.type = renewable_type_from_str(jget<std::string>(j, "type", "Wind"));
  r.p_mw = jget(j, "p_mw", 0.0);
  r.q_mvar = jget(j, "q_mvar", 0.0);
  r.p_rated_mw = jget(j, "p_rated_mw", 0.0);
  r.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  r.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  r.curtailable = jget(j, "curtailable", true);
  r.cost_curtail_mwh = jget(j, "cost_curtail_mwh", 0.0);
  r.capacity_factor = jget(j, "capacity_factor", 0.3);
  r.profile_id = jget(j, "profile_id", -1);
  r.emission_offset_tco2_mwh = jget(j, "emission_offset_tco2_mwh", 0.0);
  read_dynamic_model_if_present(j, r.dynamic_model);
  return r;
}

static json vsc_to_json(const VSCConverter& c) {
  json j;
  j["index"] = c.index;
  j["bus_ac"] = c.bus_ac;
  j["bus_dc"] = c.bus_dc;
  j["in_service"] = c.in_service;
  j["control_mode"] = converter_mode_str(c.control_mode);
  j["p_set_mw"] = c.p_set_mw;
  j["p_is_hard_constraint"] = c.p_is_hard_constraint;
  j["p_schedule_mw"] = c.p_schedule_mw;
  j["p_initial_mw"] = c.p_initial_mw;
  j["q_set_mvar"] = c.q_set_mvar;
  j["v_dc_set_pu"] = c.v_dc_set_pu;
  j["v_ac_set_pu"] = c.v_ac_set_pu;
  j["v_ac_angle_set_deg"] = c.v_ac_angle_set_deg;
  j["eta"] = c.eta;
  j["loss_percent"] = c.loss_percent;
  j["loss_mw"] = c.loss_mw;
  j["k_vdc"] = c.k_vdc;
  j["pmax_mw"] = c.pmax_mw;
  j["pmin_mw"] = c.pmin_mw;
  j["qmax_mvar"] = c.qmax_mvar;
  j["qmin_mvar"] = c.qmin_mvar;
  j["p_rated_mw"] = c.p_rated_mw;
  j["r_conv_ac_pu"] = c.r_conv_ac_pu;
  j["r_sc_pu"] = c.r_sc_pu;
  j["x_sc_pu"] = c.x_sc_pu;
  j["r2_sc_pu"] = c.r2_sc_pu;
  j["x2_sc_pu"] = c.x2_sc_pu;
  j["i_max_pu"] = c.i_max_pu;
  j["i_ac_max_pu"] = c.i_ac_max_pu;
  j["i_dc_max_pu"] = c.i_dc_max_pu;
  j["k_m_modulation"] = c.k_m_modulation;
  j["m_min"] = c.m_min;
  j["m_max"] = c.m_max;
  j["name"] = c.name;
  j["forced_outage_rate"] = c.forced_outage_rate;
  j["mttr_hr"] = c.mttr_hr;
  j["grid_forming"] = c.grid_forming;
  j["ac_grid_forming"] = c.ac_grid_forming;
  j["allow_dual_side_grid_forming"] = c.allow_dual_side_grid_forming;
  j["has_energy_buffer"] = c.has_energy_buffer;
  j["coordination_group_id"] = c.coordination_group_id;
  j["is_master"] = c.is_master;
  j["participation_factor"] = c.participation_factor;
  add_dynamic_model_if_present(j, c.dynamic_model);
  return j;
}

static VSCConverter vsc_from_json(const json& j) {
  VSCConverter c;
  c.index = j.at("index").get<int>();
  c.bus_ac = j.at("bus_ac").get<int>();
  c.bus_dc = j.at("bus_dc").get<int>();
  c.in_service = jget(j, "in_service", true);
  c.control_mode = converter_mode_from_str(
      jget<std::string>(j, "control_mode", "PQ"));
  c.p_set_mw = jget(j, "p_set_mw", 0.0);
  c.p_is_hard_constraint = jget(j, "p_is_hard_constraint", false);
  c.p_schedule_mw = jget(j, "p_schedule_mw", 0.0);
  c.p_initial_mw = jget(j, "p_initial_mw", 0.0);
  c.q_set_mvar = jget(j, "q_set_mvar", 0.0);
  c.v_dc_set_pu = jget(j, "v_dc_set_pu", 1.0);
  c.v_ac_set_pu = jget(j, "v_ac_set_pu", 1.0);
  c.v_ac_angle_set_deg = jget(j, "v_ac_angle_set_deg", 0.0);
  c.eta = jget(j, "eta", 0.99);
  c.loss_percent = jget(j, "loss_percent", 0.0);
  c.loss_mw = normalize_loss_mw_from_json(j);
  c.k_vdc = jget(j, "k_vdc", 0.1);
  c.pmax_mw = jget(j, "pmax_mw", 0.0);
  c.pmin_mw = jget(j, "pmin_mw", 0.0);
  c.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  c.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  c.p_rated_mw = jget(j, "p_rated_mw", 0.0);
  c.r_conv_ac_pu = jget(j, "r_conv_ac_pu", 0.0);
  c.r_sc_pu = jget(j, "r_sc_pu", 0.0);
  c.x_sc_pu = jget(j, "x_sc_pu", 0.15);
  c.r2_sc_pu = jget(j, "r2_sc_pu", 0.0);
  c.x2_sc_pu = jget(j, "x2_sc_pu", 0.0);
  c.i_max_pu = jget(j, "i_max_pu", 1.0);
  c.i_ac_max_pu = jget(j, "i_ac_max_pu", 0.0);
  c.i_dc_max_pu = jget(j, "i_dc_max_pu", 0.0);
  c.k_m_modulation = jget(j, "k_m_modulation", 0.0);
  c.m_min = jget(j, "m_min", 0.0);
  c.m_max = jget(j, "m_max", 0.0);
  c.name = jget<std::string>(j, "name", "");
  c.forced_outage_rate = jget(j, "forced_outage_rate", 0.0);
  c.mttr_hr = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  c.grid_forming = jget(j, "grid_forming", false);
  c.ac_grid_forming = jget(j, "ac_grid_forming", false);
  c.allow_dual_side_grid_forming = jget(j, "allow_dual_side_grid_forming", false);
  c.has_energy_buffer = jget(j, "has_energy_buffer", false);
  c.coordination_group_id = jget<std::string>(j, "coordination_group_id", "");
  c.is_master = jget(j, "is_master", false);
  c.participation_factor = jget(j, "participation_factor", 0.0);
  read_dynamic_model_if_present(j, c.dynamic_model);
  return c;
}

static json static_generator_to_json(const StaticGenerator& g) {
  json j;
  j["index"] = g.index;
  j["bus"] = g.bus;
  j["in_service"] = g.in_service;
  j["name"] = g.name;
  j["sgen_type"] = sgen_type_str(g.sgen_type);
  j["p_mw"] = g.p_mw;
  j["q_mvar"] = g.q_mvar;
  j["p_rated_mw"] = g.p_rated_mw;
  j["sn_mva"] = g.sn_mva;
  j["pmax_mw"] = g.pmax_mw;
  j["pmin_mw"] = g.pmin_mw;
  j["qmax_mvar"] = g.qmax_mvar;
  j["qmin_mvar"] = g.qmin_mvar;
  j["scaling"] = g.scaling;
  j["controllable"] = g.controllable;
  j["v_ref_pu"] = g.v_ref_pu;
  j["k_p"] = g.k_p;
  j["k_q"] = g.k_q;
  j["k"] = g.k;
  j["rx"] = g.rx;
  j["co2_emission_rate"] = g.co2_emission_rate;
  j["mtbf_hours"] = g.mtbf_hours;
  j["mttr_hours"] = g.mttr_hours;
  add_dynamic_model_if_present(j, g.dynamic_model);
  return j;
}

static StaticGenerator static_generator_from_json(const json& j) {
  StaticGenerator g;
  g.index = j.at("index").get<int>();
  g.bus = j.at("bus").get<int>();
  g.in_service = jget(j, "in_service", true);
  g.name = jget<std::string>(j, "name", "");
  g.sgen_type = sgen_type_from_str(jget<std::string>(j, "sgen_type", "Other"));
  g.p_mw = jget(j, "p_mw", 0.0);
  g.q_mvar = jget(j, "q_mvar", 0.0);
  g.p_rated_mw = jget(j, "p_rated_mw", 0.0);
  g.sn_mva = jget(j, "sn_mva", 0.0);
  g.pmax_mw = jget(j, "pmax_mw", 0.0);
  g.pmin_mw = jget(j, "pmin_mw", 0.0);
  g.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  g.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  g.scaling = jget(j, "scaling", 1.0);
  g.controllable = jget(j, "controllable", false);
  g.v_ref_pu = jget(j, "v_ref_pu", 1.0);
  g.k_p = jget(j, "k_p", 0.0);
  g.k_q = jget(j, "k_q", 0.0);
  g.k = jget(j, "k", 1.0);
  g.rx = jget(j, "rx", 0.0);
  g.co2_emission_rate = jget_alias(j, "emission_factor_tco2_mwh", "co2_emission_rate", 0.0);
  g.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  g.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, g.dynamic_model);
  return g;
}

static json flexible_load_to_json(const FlexibleLoad& l) {
  json j;
  j["index"] = l.index;
  j["bus"] = l.bus;
  j["in_service"] = l.in_service;
  j["name"] = l.name;
  j["p_mw"] = l.p_mw;
  j["q_mvar"] = l.q_mvar;
  j["flex_up_mw"] = l.flex_up_mw;
  j["flex_down_mw"] = l.flex_down_mw;
  j["flex_duration_h"] = l.flex_duration_h;
  j["response_time_s"] = l.response_time_s;
  j["ramp_rate_mw_min"] = l.ramp_rate_mw_min;
  j["availability_pct"] = l.availability_pct;
  j["controllable"] = l.controllable;
  j["priority"] = load_priority_str(l.priority);
  j["control_area"] = l.control_area;
  return j;
}

static FlexibleLoad flexible_load_from_json(const json& j) {
  FlexibleLoad l;
  l.index = j.at("index").get<int>();
  l.bus = j.at("bus").get<int>();
  l.in_service = jget(j, "in_service", true);
  l.name = jget<std::string>(j, "name", "");
  l.p_mw = jget(j, "p_mw", 0.0);
  l.q_mvar = jget(j, "q_mvar", 0.0);
  l.flex_up_mw = jget(j, "flex_up_mw", 0.0);
  l.flex_down_mw = jget(j, "flex_down_mw", 0.0);
  l.flex_duration_h = jget(j, "flex_duration_h", 0.0);
  l.response_time_s = jget(j, "response_time_s", 0.0);
  l.ramp_rate_mw_min = jget(j, "ramp_rate_mw_min", 0.0);
  l.availability_pct = jget(j, "availability_pct", 100.0);
  l.controllable = jget(j, "controllable", true);
  l.priority = load_priority_from_str(jget<std::string>(j, "priority", "Medium"));
  l.control_area = jget<std::string>(j, "control_area", "");
  return l;
}

static json asymmetric_load_to_json(const AsymmetricLoad& l) {
  json j;
  j["index"] = l.index;
  j["bus"] = l.bus;
  j["in_service"] = l.in_service;
  j["name"] = l.name;
  j["connection"] = l.connection;
  j["grounded"] = l.grounded;
  j["pa_rated_mw"] = l.pa_rated_mw;
  j["qa_rated_mvar"] = l.qa_rated_mvar;
  j["pb_rated_mw"] = l.pb_rated_mw;
  j["qb_rated_mvar"] = l.qb_rated_mvar;
  j["pc_rated_mw"] = l.pc_rated_mw;
  j["qc_rated_mvar"] = l.qc_rated_mvar;
  j["pa_mw"] = l.pa_mw;
  j["qa_mvar"] = l.qa_mvar;
  j["pb_mw"] = l.pb_mw;
  j["qb_mvar"] = l.qb_mvar;
  j["pc_mw"] = l.pc_mw;
  j["qc_mvar"] = l.qc_mvar;
  j["scaling"] = l.scaling;
  j["const_z_percent"] = l.const_z_percent;
  j["const_i_percent"] = l.const_i_percent;
  j["const_p_percent"] = l.const_p_percent;
  j["controllable"] = l.controllable;
  j["priority"] = load_priority_str(l.priority);
  add_dynamic_model_if_present(j, l.dynamic_model);
  return j;
}

static AsymmetricLoad asymmetric_load_from_json(const json& j) {
  AsymmetricLoad l;
  l.index = j.at("index").get<int>();
  l.bus = j.at("bus").get<int>();
  l.in_service = jget(j, "in_service", true);
  l.name = jget<std::string>(j, "name", "");
  l.connection = jget<std::string>(j, "connection", "");
  l.grounded = jget(j, "grounded", true);
  l.pa_rated_mw = jget(j, "pa_rated_mw", 0.0);
  l.qa_rated_mvar = jget(j, "qa_rated_mvar", 0.0);
  l.pb_rated_mw = jget(j, "pb_rated_mw", 0.0);
  l.qb_rated_mvar = jget(j, "qb_rated_mvar", 0.0);
  l.pc_rated_mw = jget(j, "pc_rated_mw", 0.0);
  l.qc_rated_mvar = jget(j, "qc_rated_mvar", 0.0);
  l.pa_mw = jget(j, "pa_mw", 0.0);
  l.qa_mvar = jget(j, "qa_mvar", 0.0);
  l.pb_mw = jget(j, "pb_mw", 0.0);
  l.qb_mvar = jget(j, "qb_mvar", 0.0);
  l.pc_mw = jget(j, "pc_mw", 0.0);
  l.qc_mvar = jget(j, "qc_mvar", 0.0);
  l.scaling = jget(j, "scaling", 1.0);
  l.const_z_percent = jget(j, "const_z_percent", 0.0);
  l.const_i_percent = jget(j, "const_i_percent", 0.0);
  l.const_p_percent = jget(j, "const_p_percent", 100.0);
  l.controllable = jget(j, "controllable", false);
  l.priority = load_priority_from_str(jget<std::string>(j, "priority", "Medium"));
  read_dynamic_model_if_present(j, l.dynamic_model);
  return l;
}

static json pv_system_to_json(const PVSystem& p) {
  json j;
  j["index"] = p.index;
  j["bus"] = p.bus;
  j["in_service"] = p.in_service;
  j["name"] = p.name;
  j["p_mw"] = p.p_mw;
  j["q_mvar"] = p.q_mvar;
  j["sn_mva"] = p.sn_mva;
  j["pmax_mw"] = p.pmax_mw;
  j["pmin_mw"] = p.pmin_mw;
  j["qmax_mvar"] = p.qmax_mvar;
  j["qmin_mvar"] = p.qmin_mvar;
  j["control_mode"] = pv_control_str(p.control_mode);
  j["controllable"] = p.controllable;
  j["v_ac_set_pu"] = p.v_ac_set_pu;
  j["v_dc_set_pu"] = p.v_dc_set_pu;
  j["inverter_eff"] = p.inverter_eff;
  j["loss_percent"] = p.loss_percent;
  j["num_series"] = p.num_series;
  j["num_parallel"] = p.num_parallel;
  j["vmpp"] = p.vmpp;
  j["impp"] = p.impp;
  j["voc"] = p.voc;
  j["isc"] = p.isc;
  j["alpha_isc"] = p.alpha_isc;
  j["beta_voc"] = p.beta_voc;
  j["irradiance"] = p.irradiance;
  j["temperature"] = p.temperature;
  j["profile_id"] = p.profile_id;
  j["mtbf_panel_hours"] = p.mtbf_panel_hours;
  j["mttr_panel_hours"] = p.mttr_panel_hours;
  j["mtbf_inverter_hours"] = p.mtbf_inverter_hours;
  j["mttr_inverter_hours"] = p.mttr_inverter_hours;
  add_dynamic_model_if_present(j, p.dynamic_model);
  return j;
}

static PVSystem pv_system_from_json(const json& j) {
  PVSystem p;
  p.index = j.at("index").get<int>();
  p.bus = j.at("bus").get<int>();
  p.in_service = jget(j, "in_service", true);
  p.name = jget<std::string>(j, "name", "");
  p.p_mw = jget(j, "p_mw", 0.0);
  p.q_mvar = jget(j, "q_mvar", 0.0);
  p.sn_mva = jget(j, "sn_mva", 0.0);
  p.pmax_mw = jget(j, "pmax_mw", 0.0);
  p.pmin_mw = jget(j, "pmin_mw", 0.0);
  p.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  p.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  p.control_mode = pv_control_from_str(jget<std::string>(j, "control_mode", "MPPT"));
  p.controllable = jget(j, "controllable", false);
  p.v_ac_set_pu = jget(j, "v_ac_set_pu", 1.0);
  p.v_dc_set_pu = jget(j, "v_dc_set_pu", 1.0);
  p.inverter_eff = jget(j, "inverter_eff", 0.97);
  p.loss_percent = jget(j, "loss_percent", 0.0);
  p.num_series = jget(j, "num_series", 0);
  p.num_parallel = jget(j, "num_parallel", 0);
  p.vmpp = jget(j, "vmpp", 0.0);
  p.impp = jget(j, "impp", 0.0);
  p.voc = jget(j, "voc", 0.0);
  p.isc = jget(j, "isc", 0.0);
  p.alpha_isc = jget(j, "alpha_isc", 0.0);
  p.beta_voc = jget(j, "beta_voc", 0.0);
  p.irradiance = jget(j, "irradiance", 1000.0);
  p.temperature = jget(j, "temperature", 25.0);
  p.profile_id = jget(j, "profile_id", -1);
  p.mtbf_panel_hours = jget(j, "mtbf_panel_hours", 0.0);
  p.mttr_panel_hours = jget(j, "mttr_panel_hours", 0.0);
  p.mtbf_inverter_hours = jget(j, "mtbf_inverter_hours", 0.0);
  p.mttr_inverter_hours = jget(j, "mttr_inverter_hours", 0.0);
  read_dynamic_model_if_present(j, p.dynamic_model);
  return p;
}

static json external_grid_to_json(const ExternalGrid& e) {
  json j;
  j["index"] = e.index;
  j["name"] = e.name;
  j["bus"] = e.bus;
  j["in_service"] = e.in_service;
  j["vm_pu"] = e.vm_pu;
  j["va_deg"] = e.va_deg;
  j["s_sc_max_mva"] = e.s_sc_max_mva;
  j["s_sc_min_mva"] = e.s_sc_min_mva;
  j["rx_max"] = e.rx_max;
  j["rx_min"] = e.rx_min;
  j["r_pu"] = e.r_pu;
  j["x_pu"] = e.x_pu;
  j["r0_pu"] = e.r0_pu;
  j["x0_pu"] = e.x0_pu;
  j["vn_kv"] = e.vn_kv;
  j["controllable"] = e.controllable;
  j["emission_factor_tco2_mwh"] = e.emission_factor_tco2_mwh;
  j["cost_c2"] = e.cost_c2;
  j["cost_c1"] = e.cost_c1;
  j["cost_c0"] = e.cost_c0;
  if (e.price_profile_id >= 0)
    j["price_profile_id"] = e.price_profile_id;
  add_dynamic_model_if_present(j, e.dynamic_model);
  return j;
}

static ExternalGrid external_grid_from_json(const json& j) {
  ExternalGrid e;
  e.index = j.at("index").get<int>();
  e.name = jget<std::string>(j, "name", "");
  e.bus = j.at("bus").get<int>();
  e.in_service = jget(j, "in_service", true);
  e.vm_pu = jget(j, "vm_pu", 1.0);
  e.va_deg = jget(j, "va_deg", 0.0);
  e.s_sc_max_mva = jget(j, "s_sc_max_mva", 0.0);
  e.s_sc_min_mva = jget(j, "s_sc_min_mva", 0.0);
  e.rx_max = jget(j, "rx_max", 0.0);
  e.rx_min = jget(j, "rx_min", 0.0);
  e.r_pu = jget(j, "r_pu", 0.0);
  e.x_pu = jget(j, "x_pu", 0.0);
  e.r0_pu = jget(j, "r0_pu", 0.0);
  e.x0_pu = jget(j, "x0_pu", 0.0);
  e.vn_kv = jget(j, "vn_kv", 0.0);
  e.controllable = jget(j, "controllable", true);
  e.emission_factor_tco2_mwh =
      jget_alias(j, "emission_factor_tco2_mwh", "co2_emission_rate", 0.0);
  e.cost_c2 = jget(j, "cost_c2", 0.0);
  e.cost_c1 = jget(j, "cost_c1", 0.0);
  e.cost_c0 = jget(j, "cost_c0", 0.0);
  e.price_profile_id = jget(j, "price_profile_id", -1);
  read_dynamic_model_if_present(j, e.dynamic_model);
  return e;
}

static json transformer2w_to_json(const Transformer2W& t) {
  json j;
  j["index"] = t.index;
  j["name"] = t.name;
  j["std_type"] = t.std_type;
  j["hv_bus"] = t.hv_bus;
  j["lv_bus"] = t.lv_bus;
  j["in_service"] = t.in_service;
  j["sn_mva"] = t.sn_mva;
  j["vn_hv_kv"] = t.vn_hv_kv;
  j["vn_lv_kv"] = t.vn_lv_kv;
  j["vk_percent"] = t.vk_percent;
  j["vkr_percent"] = t.vkr_percent;
  j["pk_kw"] = t.pk_kw;
  j["pfe_kw"] = t.pfe_kw;
  j["i0_percent"] = t.i0_percent;
  j["tap_side"] = t.tap_side;
  j["tap_pos"] = t.tap_pos;
  j["tap_min"] = t.tap_min;
  j["tap_max"] = t.tap_max;
  j["tap_neutral"] = t.tap_neutral;
  j["tap_step_percent"] = t.tap_step_percent;
  j["shift_deg"] = t.shift_deg;
  j["vector_group"] = t.vector_group;
  j["z0_percent"] = t.z0_percent;
  j["x0_r0"] = t.x0_r0;
  j["mtbf_hours"] = t.mtbf_hours;
  j["mttr_hours"] = t.mttr_hours;
  j["source_branch_idx"] = t.source_branch_idx;
  return j;
}

static Transformer2W transformer2w_from_json(const json& j) {
  Transformer2W t;
  t.index = j.at("index").get<int>();
  t.name = jget<std::string>(j, "name", "");
  t.std_type = jget<std::string>(j, "std_type", "");
  t.hv_bus = j.at("hv_bus").get<int>();
  t.lv_bus = j.at("lv_bus").get<int>();
  t.in_service = jget(j, "in_service", true);
  t.sn_mva = jget(j, "sn_mva", 0.0);
  t.vn_hv_kv = jget(j, "vn_hv_kv", 0.0);
  t.vn_lv_kv = jget(j, "vn_lv_kv", 0.0);
  t.vk_percent = jget(j, "vk_percent", 0.0);
  t.vkr_percent = jget(j, "vkr_percent", 0.0);
  t.pk_kw = jget(j, "pk_kw", 0.0);
  t.pfe_kw = jget(j, "pfe_kw", 0.0);
  t.i0_percent = jget(j, "i0_percent", 0.0);
  t.tap_side = jget(j, "tap_side", 0);
  t.tap_pos = jget(j, "tap_pos", 0);
  t.tap_min = jget(j, "tap_min", 0);
  t.tap_max = jget(j, "tap_max", 0);
  t.tap_neutral = jget(j, "tap_neutral", 0);
  t.tap_step_percent = jget(j, "tap_step_percent", 0.0);
  t.shift_deg = jget(j, "shift_deg", 0.0);
  t.vector_group = jget<std::string>(j, "vector_group", "");
  t.z0_percent = jget(j, "z0_percent", 0.0);
  t.x0_r0 = jget(j, "x0_r0", 0.0);
  t.mtbf_hours = jget_alias(j, "mtbf_hours", "mtbf_hours", 0.0);
  t.mttr_hours = jget_alias(j, "mttr_hours", "mttr_hours", 0.0);
  t.source_branch_idx = jget(j, "source_branch_idx", 0);
  return t;
}

static bool branch_matches_transformer2w_metadata(const ACBranch& br,
                                                  const Transformer2W& tr,
                                                  double base_mva) {
  if (br.index <= 0 || tr.source_branch_idx > 0) return false;
  if (br.from_bus != tr.hv_bus || br.to_bus != tr.lv_bus) return false;
  const double branch_tap = std::abs(br.tap) < 1e-12 ? 1.0 : br.tap;
  const bool transformer_like_branch =
      std::abs(branch_tap - 1.0) > 1e-8 || std::abs(br.shift_deg) > 1e-8;
  if (!transformer_like_branch) return false;

  const double tr_tap = std::max(
      1e-6, 1.0 + (static_cast<double>(tr.tap_pos) -
                    static_cast<double>(tr.tap_neutral)) *
                       tr.tap_step_percent / 100.0);
  if (std::abs(branch_tap - tr_tap) > 1e-5) return false;
  if (std::abs(br.shift_deg - tr.shift_deg) > 1e-5) return false;
  if (tr.sn_mva <= 1e-9 || base_mva <= 1e-9) return true;

  const double scale = base_mva / tr.sn_mva;
  const double z_mag = std::max(0.0, tr.vk_percent / 100.0) * scale;
  const double r_pu = std::max(0.0, tr.vkr_percent / 100.0) * scale;
  const double x_pu = std::sqrt(std::max(0.0, z_mag * z_mag - r_pu * r_pu));
  const double tol = 1e-5;
  return std::abs(br.r_pu - r_pu) <= tol &&
         std::abs(std::abs(br.x_pu) - x_pu) <= tol;
}

static void repair_legacy_transformer2w_branch_links(HybridPowerSystem& sys) {
  const double base_mva = pf_ac_base_mva(sys);
  for (auto& tr : sys.ac.transformers_2w) {
    if (tr.source_branch_idx > 0) continue;
    int match = 0;
    int match_count = 0;
    for (const auto& br : sys.ac.branches) {
      if (!branch_matches_transformer2w_metadata(br, tr, base_mva)) continue;
      match = br.index;
      ++match_count;
    }
    if (match_count == 1) {
      tr.source_branch_idx = match;
    }
  }
}

static json transformer3w_to_json(const Transformer3W& t) {
  json j;
  j["index"] = t.index;
  j["name"] = t.name;
  j["std_type"] = t.std_type;
  j["hv_bus"] = t.hv_bus;
  j["mv_bus"] = t.mv_bus;
  j["lv_bus"] = t.lv_bus;
  j["in_service"] = t.in_service;
  j["sn_hv_mva"] = t.sn_hv_mva;
  j["sn_mv_mva"] = t.sn_mv_mva;
  j["sn_lv_mva"] = t.sn_lv_mva;
  j["vn_hv_kv"] = t.vn_hv_kv;
  j["vn_mv_kv"] = t.vn_mv_kv;
  j["vn_lv_kv"] = t.vn_lv_kv;
  j["vk_hv_mv_percent"] = t.vk_hv_mv_percent;
  j["vk_hv_lv_percent"] = t.vk_hv_lv_percent;
  j["vk_mv_lv_percent"] = t.vk_mv_lv_percent;
  j["vkr_hv_mv_percent"] = t.vkr_hv_mv_percent;
  j["vkr_hv_lv_percent"] = t.vkr_hv_lv_percent;
  j["vkr_mv_lv_percent"] = t.vkr_mv_lv_percent;
  j["pfe_kw"] = t.pfe_kw;
  j["i0_percent"] = t.i0_percent;
  j["tap_side"] = t.tap_side;
  j["tap_pos"] = t.tap_pos;
  j["tap_step_percent"] = t.tap_step_percent;
  j["shift_mv_deg"] = t.shift_mv_deg;
  j["shift_lv_deg"] = t.shift_lv_deg;
  j["mtbf_hours"] = t.mtbf_hours;
  j["mttr_hours"] = t.mttr_hours;
  return j;
}

static Transformer3W transformer3w_from_json(const json& j) {
  Transformer3W t;
  t.index = j.at("index").get<int>();
  t.name = jget<std::string>(j, "name", "");
  t.std_type = jget<std::string>(j, "std_type", "");
  t.hv_bus = j.at("hv_bus").get<int>();
  t.mv_bus = j.at("mv_bus").get<int>();
  t.lv_bus = j.at("lv_bus").get<int>();
  t.in_service = jget(j, "in_service", true);
  t.sn_hv_mva = jget(j, "sn_hv_mva", 0.0);
  t.sn_mv_mva = jget(j, "sn_mv_mva", 0.0);
  t.sn_lv_mva = jget(j, "sn_lv_mva", 0.0);
  t.vn_hv_kv = jget(j, "vn_hv_kv", 0.0);
  t.vn_mv_kv = jget(j, "vn_mv_kv", 0.0);
  t.vn_lv_kv = jget(j, "vn_lv_kv", 0.0);
  t.vk_hv_mv_percent = jget(j, "vk_hv_mv_percent", 0.0);
  t.vk_hv_lv_percent = jget(j, "vk_hv_lv_percent", 0.0);
  t.vk_mv_lv_percent = jget(j, "vk_mv_lv_percent", 0.0);
  t.vkr_hv_mv_percent = jget(j, "vkr_hv_mv_percent", 0.0);
  t.vkr_hv_lv_percent = jget(j, "vkr_hv_lv_percent", 0.0);
  t.vkr_mv_lv_percent = jget(j, "vkr_mv_lv_percent", 0.0);
  t.pfe_kw = jget(j, "pfe_kw", 0.0);
  t.i0_percent = jget(j, "i0_percent", 0.0);
  t.tap_side = jget(j, "tap_side", 0);
  t.tap_pos = jget(j, "tap_pos", 0);
  t.tap_step_percent = jget(j, "tap_step_percent", 0.0);
  t.shift_mv_deg = jget(j, "shift_mv_deg", 0.0);
  t.shift_lv_deg = jget(j, "shift_lv_deg", 0.0);
  t.mtbf_hours = jget_alias(j, "mtbf_hours", "mtbf_hours", 0.0);
  t.mttr_hours = jget_alias(j, "mttr_hours", "mttr_hours", 0.0);
  return t;
}

static json asynchronous_motor_to_json(const AsynchronousMotor& m) {
  json j;
  j["index"] = m.index;
  j["bus"] = m.bus;
  j["in_service"] = m.in_service;
  j["name"] = m.name;
  j["vn_kv"] = m.vn_kv;
  j["sn_mva"] = m.sn_mva;
  j["r_pu"] = m.r_pu;
  j["x_pu"] = m.x_pu;
  j["x_r"] = m.x_r;
  j["lrc"] = m.lrc;
  j["poles"] = m.poles;
  j["cos_phi"] = m.cos_phi;
  j["efficiency"] = m.efficiency;
  j["r0_pu"] = m.r0_pu;
  j["x0_pu"] = m.x0_pu;
  add_dynamic_model_if_present(j, m.dynamic_model);
  return j;
}

static AsynchronousMotor asynchronous_motor_from_json(const json& j) {
  AsynchronousMotor m;
  m.index = j.at("index").get<int>();
  m.bus = j.at("bus").get<int>();
  m.in_service = jget(j, "in_service", true);
  m.name = jget<std::string>(j, "name", "");
  m.vn_kv = jget(j, "vn_kv", 0.0);
  m.sn_mva = jget(j, "sn_mva", 0.0);
  m.r_pu = jget(j, "r_pu", 0.0);
  m.x_pu = jget(j, "x_pu", 0.0);
  m.x_r = jget(j, "x_r", 0.0);
  m.lrc = jget(j, "lrc", 0.0);
  m.poles = jget(j, "poles", 2);
  m.cos_phi = jget(j, "cos_phi", 0.85);
  m.efficiency = jget(j, "efficiency", 0.95);
  m.r0_pu = jget(j, "r0_pu", 0.0);
  m.x0_pu = jget(j, "x0_pu", 0.0);
  read_dynamic_model_if_present(j, m.dynamic_model);
  return m;
}

static json three_phase_bus_to_json(const ThreePhaseACBus& b) {
  json j;
  j["index"] = b.index;
  j["bus_type"] = bus_type_str(b.bus_type);
  j["name"] = b.name;
  j["base_kv"] = b.base_kv;
  j["in_service"] = b.in_service;
  j["vm_a_pu"] = b.vm_a_pu;
  j["va_a_deg"] = b.va_a_deg;
  j["vm_b_pu"] = b.vm_b_pu;
  j["va_b_deg"] = b.va_b_deg;
  j["vm_c_pu"] = b.vm_c_pu;
  j["va_c_deg"] = b.va_c_deg;
  j["pd_a_mw"] = b.pd_a_mw;
  j["qd_a_mvar"] = b.qd_a_mvar;
  j["pd_b_mw"] = b.pd_b_mw;
  j["qd_b_mvar"] = b.qd_b_mvar;
  j["pd_c_mw"] = b.pd_c_mw;
  j["qd_c_mvar"] = b.qd_c_mvar;
  j["gs_a_mw"] = b.gs_a_mw;
  j["bs_a_mvar"] = b.bs_a_mvar;
  j["gs_b_mw"] = b.gs_b_mw;
  j["bs_b_mvar"] = b.bs_b_mvar;
  j["gs_c_mw"] = b.gs_c_mw;
  j["bs_c_mvar"] = b.bs_c_mvar;
  j["vmin_pu"] = b.vmin_pu;
  j["vmax_pu"] = b.vmax_pu;
  j["area"] = b.area;
  j["zone"] = b.zone;
  return j;
}

static ThreePhaseACBus three_phase_bus_from_json(const json& j) {
  ThreePhaseACBus b;
  b.index = j.at("index").get<int>();
  b.bus_type = bus_type_from_str(jget_ac_bus_type_str(j));
  b.name = jget<std::string>(j, "name", "");
  b.base_kv = jget(j, "base_kv", 0.0);
  b.in_service = jget(j, "in_service", true);
  b.vm_a_pu = jget(j, "vm_a_pu", 1.0);
  b.va_a_deg = jget(j, "va_a_deg", 0.0);
  b.vm_b_pu = jget(j, "vm_b_pu", 1.0);
  b.va_b_deg = jget(j, "va_b_deg", -120.0);
  b.vm_c_pu = jget(j, "vm_c_pu", 1.0);
  b.va_c_deg = jget(j, "va_c_deg", 120.0);
  b.pd_a_mw = jget(j, "pd_a_mw", 0.0);
  b.qd_a_mvar = jget(j, "qd_a_mvar", 0.0);
  b.pd_b_mw = jget(j, "pd_b_mw", 0.0);
  b.qd_b_mvar = jget(j, "qd_b_mvar", 0.0);
  b.pd_c_mw = jget(j, "pd_c_mw", 0.0);
  b.qd_c_mvar = jget(j, "qd_c_mvar", 0.0);
  b.gs_a_mw = jget(j, "gs_a_mw", 0.0);
  b.bs_a_mvar = jget(j, "bs_a_mvar", 0.0);
  b.gs_b_mw = jget(j, "gs_b_mw", 0.0);
  b.bs_b_mvar = jget(j, "bs_b_mvar", 0.0);
  b.gs_c_mw = jget(j, "gs_c_mw", 0.0);
  b.bs_c_mvar = jget(j, "bs_c_mvar", 0.0);
  b.vmin_pu = jget(j, "vmin_pu", 0.9);
  b.vmax_pu = jget(j, "vmax_pu", 1.1);
  b.area = jget(j, "area", 1);
  b.zone = jget(j, "zone", 1);
  return b;
}

static json three_phase_line_to_json(const ThreePhaseACLine& l) {
  json j;
  j["index"] = l.index;
  j["from_bus"] = l.from_bus;
  j["to_bus"] = l.to_bus;
  j["name"] = l.name;
  j["in_service"] = l.in_service;
  j["length_km"] = l.length_km;
  j["parallel"] = l.parallel;
  j["r1_ohm_per_km"] = l.r1_ohm_per_km;
  j["x1_ohm_per_km"] = l.x1_ohm_per_km;
  j["c1_nf_per_km"] = l.c1_nf_per_km;
  j["r0_ohm_per_km"] = l.r0_ohm_per_km;
  j["x0_ohm_per_km"] = l.x0_ohm_per_km;
  j["c0_nf_per_km"] = l.c0_nf_per_km;
  j["r1_pu"] = l.r1_pu;
  j["x1_pu"] = l.x1_pu;
  j["b1_pu"] = l.b1_pu;
  j["r0_pu"] = l.r0_pu;
  j["x0_pu"] = l.x0_pu;
  j["b0_pu"] = l.b0_pu;
  j["max_i_ka"] = l.max_i_ka;
  j["rate_a_mva"] = l.rate_a_mva;
  j["failure_rate"] = l.failure_rate;
  j["mttr_hr"] = l.mttr_hr;
  return j;
}

static ThreePhaseACLine three_phase_line_from_json(const json& j) {
  ThreePhaseACLine l;
  l.index = j.at("index").get<int>();
  l.from_bus = j.at("from_bus").get<int>();
  l.to_bus = j.at("to_bus").get<int>();
  l.name = jget<std::string>(j, "name", "");
  l.in_service = jget(j, "in_service", true);
  l.length_km = jget(j, "length_km", 0.0);
  l.parallel = jget(j, "parallel", 1);
  l.r1_ohm_per_km = jget(j, "r1_ohm_per_km", 0.0);
  l.x1_ohm_per_km = jget(j, "x1_ohm_per_km", 0.0);
  l.c1_nf_per_km = jget(j, "c1_nf_per_km", 0.0);
  l.r0_ohm_per_km = jget(j, "r0_ohm_per_km", 0.0);
  l.x0_ohm_per_km = jget(j, "x0_ohm_per_km", 0.0);
  l.c0_nf_per_km = jget(j, "c0_nf_per_km", 0.0);
  l.r1_pu = jget(j, "r1_pu", 0.0);
  l.x1_pu = jget(j, "x1_pu", 0.0);
  l.b1_pu = jget(j, "b1_pu", 0.0);
  l.r0_pu = jget(j, "r0_pu", 0.0);
  l.x0_pu = jget(j, "x0_pu", 0.0);
  l.b0_pu = jget(j, "b0_pu", 0.0);
  l.max_i_ka = jget(j, "max_i_ka", 0.0);
  l.rate_a_mva = jget(j, "rate_a_mva", 0.0);
  l.failure_rate = jget(j, "failure_rate", 0.0);
  l.mttr_hr = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  return l;
}

static json three_phase_transformer_to_json(const ThreePhaseTransformer& t) {
  json j;
  j["index"] = t.index;
  j["name"] = t.name;
  j["hv_bus"] = t.hv_bus;
  j["lv_bus"] = t.lv_bus;
  j["in_service"] = t.in_service;
  j["sn_mva"] = t.sn_mva;
  j["vn_hv_kv"] = t.vn_hv_kv;
  j["vn_lv_kv"] = t.vn_lv_kv;
  j["vk_percent"] = t.vk_percent;
  j["vkr_percent"] = t.vkr_percent;
  j["pfe_kw"] = t.pfe_kw;
  j["i0_percent"] = t.i0_percent;
  j["vector_group"] = t.vector_group;
  j["vk0_percent"] = t.vk0_percent;
  j["vkr0_percent"] = t.vkr0_percent;
  j["mag0_percent"] = t.mag0_percent;
  j["mag0_rx"] = t.mag0_rx;
  j["si0_hv_partial"] = t.si0_hv_partial;
  j["tap_side"] = t.tap_side;
  j["tap_pos"] = t.tap_pos;
  j["tap_min"] = t.tap_min;
  j["tap_max"] = t.tap_max;
  j["tap_neutral"] = t.tap_neutral;
  j["tap_step_percent"] = t.tap_step_percent;
  j["shift_deg"] = t.shift_deg;
  j["mtbf_hr"] = t.mtbf_hr;
  j["mttr_hr"] = t.mttr_hr;
  return j;
}

static ThreePhaseTransformer three_phase_transformer_from_json(const json& j) {
  ThreePhaseTransformer t;
  t.index = j.at("index").get<int>();
  t.name = jget<std::string>(j, "name", "");
  t.hv_bus = j.at("hv_bus").get<int>();
  t.lv_bus = j.at("lv_bus").get<int>();
  t.in_service = jget(j, "in_service", true);
  t.sn_mva = jget(j, "sn_mva", 0.0);
  t.vn_hv_kv = jget(j, "vn_hv_kv", 0.0);
  t.vn_lv_kv = jget(j, "vn_lv_kv", 0.0);
  t.vk_percent = jget(j, "vk_percent", 0.0);
  t.vkr_percent = jget(j, "vkr_percent", 0.0);
  t.pfe_kw = jget(j, "pfe_kw", 0.0);
  t.i0_percent = jget(j, "i0_percent", 0.0);
  t.vector_group = jget<std::string>(j, "vector_group", "");
  t.vk0_percent = jget(j, "vk0_percent", 0.0);
  t.vkr0_percent = jget(j, "vkr0_percent", 0.0);
  t.mag0_percent = jget(j, "mag0_percent", 0.0);
  t.mag0_rx = jget(j, "mag0_rx", 0.0);
  t.si0_hv_partial = jget(j, "si0_hv_partial", 0.5);
  t.tap_side = jget(j, "tap_side", 0);
  t.tap_pos = jget(j, "tap_pos", 0);
  t.tap_min = jget(j, "tap_min", 0);
  t.tap_max = jget(j, "tap_max", 0);
  t.tap_neutral = jget(j, "tap_neutral", 0);
  t.tap_step_percent = jget(j, "tap_step_percent", 0.0);
  t.shift_deg = jget(j, "shift_deg", 0.0);
  t.mtbf_hr = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  t.mttr_hr = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  return t;
}

static json three_phase_load_to_json(const ThreePhaseLoad& l) {
  json j;
  j["index"] = l.index;
  j["bus"] = l.bus;
  j["name"] = l.name;
  j["in_service"] = l.in_service;
  j["connection"] = l.connection;
  j["grounded"] = l.grounded;
  j["p_a_mw"] = l.p_a_mw;
  j["q_a_mvar"] = l.q_a_mvar;
  j["p_b_mw"] = l.p_b_mw;
  j["q_b_mvar"] = l.q_b_mvar;
  j["p_c_mw"] = l.p_c_mw;
  j["q_c_mvar"] = l.q_c_mvar;
  j["const_z_percent"] = l.const_z_percent;
  j["const_i_percent"] = l.const_i_percent;
  j["const_p_percent"] = l.const_p_percent;
  j["motor_percent"] = l.motor_percent;
  j["lrc_pu"] = l.lrc_pu;
  j["x_r_ratio"] = l.x_r_ratio;
  add_dynamic_model_if_present(j, l.dynamic_model);
  return j;
}

static ThreePhaseLoad three_phase_load_from_json(const json& j) {
  ThreePhaseLoad l;
  l.index = j.at("index").get<int>();
  l.bus = j.at("bus").get<int>();
  l.name = jget<std::string>(j, "name", "");
  l.in_service = jget(j, "in_service", true);
  l.connection = jget<std::string>(j, "connection", "wye");
  l.grounded = jget(j, "grounded", true);
  l.p_a_mw = jget(j, "p_a_mw", 0.0);
  l.q_a_mvar = jget(j, "q_a_mvar", 0.0);
  l.p_b_mw = jget(j, "p_b_mw", 0.0);
  l.q_b_mvar = jget(j, "q_b_mvar", 0.0);
  l.p_c_mw = jget(j, "p_c_mw", 0.0);
  l.q_c_mvar = jget(j, "q_c_mvar", 0.0);
  l.const_z_percent = jget(j, "const_z_percent", 0.0);
  l.const_i_percent = jget(j, "const_i_percent", 0.0);
  l.const_p_percent = jget(j, "const_p_percent", 100.0);
  l.motor_percent = jget(j, "motor_percent", 0.0);
  l.lrc_pu = jget(j, "lrc_pu", 0.0);
  l.x_r_ratio = jget(j, "x_r_ratio", 0.0);
  read_dynamic_model_if_present(j, l.dynamic_model);
  return l;
}

static json three_phase_generator_to_json(const ThreePhaseGenerator& g) {
  json j;
  j["index"] = g.index;
  j["bus"] = g.bus;
  j["name"] = g.name;
  j["in_service"] = g.in_service;
  j["is_slack"] = g.is_slack;
  j["p_mw"] = g.p_mw;
  j["q_mvar"] = g.q_mvar;
  j["vm_pu"] = g.vm_pu;
  j["pmax_mw"] = g.pmax_mw;
  j["pmin_mw"] = g.pmin_mw;
  j["qmax_mvar"] = g.qmax_mvar;
  j["qmin_mvar"] = g.qmin_mvar;
  j["mbase_mva"] = g.mbase_mva;
  j["xd_pu"] = g.xd_pu;
  j["xdpp_pu"] = g.xdpp_pu;
  j["x2_pu"] = g.x2_pu;
  j["x0_pu"] = g.x0_pu;
  j["r0_pu"] = g.r0_pu;
  add_dynamic_model_if_present(j, g.dynamic_model);
  return j;
}

static ThreePhaseGenerator three_phase_generator_from_json(const json& j) {
  ThreePhaseGenerator g;
  g.index = j.at("index").get<int>();
  g.bus = j.at("bus").get<int>();
  g.name = jget<std::string>(j, "name", "");
  g.in_service = jget(j, "in_service", true);
  g.is_slack = jget(j, "is_slack", false);
  g.p_mw = jget(j, "p_mw", 0.0);
  g.q_mvar = jget(j, "q_mvar", 0.0);
  g.vm_pu = jget(j, "vm_pu", 1.0);
  g.pmax_mw = jget(j, "pmax_mw", 0.0);
  g.pmin_mw = jget(j, "pmin_mw", 0.0);
  g.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  g.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  g.mbase_mva = jget(j, "mbase_mva", 0.0);
  g.xd_pu = jget(j, "xd_pu", 0.0);
  g.xdpp_pu = jget(j, "xdpp_pu", 0.0);
  g.x2_pu = jget(j, "x2_pu", 0.0);
  g.x0_pu = jget(j, "x0_pu", 0.0);
  g.r0_pu = jget(j, "r0_pu", 0.0);
  read_dynamic_model_if_present(j, g.dynamic_model);
  return g;
}

static json three_phase_external_grid_to_json(const ThreePhaseExternalGrid& e) {
  json j;
  j["index"] = e.index;
  j["bus"] = e.bus;
  j["name"] = e.name;
  j["in_service"] = e.in_service;
  j["vm_pu"] = e.vm_pu;
  j["va_deg"] = e.va_deg;
  j["s_sc_max_mva"] = e.s_sc_max_mva;
  j["s_sc_min_mva"] = e.s_sc_min_mva;
  j["rx_max"] = e.rx_max;
  j["rx_min"] = e.rx_min;
  j["r1_pu"] = e.r1_pu;
  j["x1_pu"] = e.x1_pu;
  j["r2_pu"] = e.r2_pu;
  j["x2_pu"] = e.x2_pu;
  j["r0_pu"] = e.r0_pu;
  j["x0_pu"] = e.x0_pu;
  add_dynamic_model_if_present(j, e.dynamic_model);
  return j;
}

static ThreePhaseExternalGrid three_phase_external_grid_from_json(const json& j) {
  ThreePhaseExternalGrid e;
  e.index = j.at("index").get<int>();
  e.bus = j.at("bus").get<int>();
  e.name = jget<std::string>(j, "name", "");
  e.in_service = jget(j, "in_service", true);
  e.vm_pu = jget(j, "vm_pu", 1.0);
  e.va_deg = jget(j, "va_deg", 0.0);
  e.s_sc_max_mva = jget(j, "s_sc_max_mva", 0.0);
  e.s_sc_min_mva = jget(j, "s_sc_min_mva", 0.0);
  e.rx_max = jget(j, "rx_max", 0.0);
  e.rx_min = jget(j, "rx_min", 0.0);
  e.r1_pu = jget(j, "r1_pu", 0.0);
  e.x1_pu = jget(j, "x1_pu", 0.0);
  e.r2_pu = jget(j, "r2_pu", 0.0);
  e.x2_pu = jget(j, "x2_pu", 0.0);
  e.r0_pu = jget(j, "r0_pu", 0.0);
  e.x0_pu = jget(j, "x0_pu", 0.0);
  read_dynamic_model_if_present(j, e.dynamic_model);
  return e;
}

static json regulator_control_to_json(const RegulatorControl& c) {
  json j;
  j["index"] = c.index;
  j["name"] = c.name;
  j["transformer_index"] = c.transformer_index;
  j["transformer_name"] = c.transformer_name;
  j["winding"] = c.winding;
  j["tap_winding"] = c.tap_winding;
  j["monitored_bus"] = c.monitored_bus;
  j["monitored_node"] = c.monitored_node;
  j["vreg_volts"] = c.vreg_volts;
  j["band_volts"] = c.band_volts;
  j["ptratio"] = c.ptratio;
  j["remote_ptratio"] = c.remote_ptratio;
  j["ct_primary_amps"] = c.ct_primary_amps;
  j["r_volts"] = c.r_volts;
  j["x_volts"] = c.x_volts;
  j["max_tap_change"] = c.max_tap_change;
  j["reversible"] = c.reversible;
  j["enabled"] = c.enabled;
  return j;
}

static RegulatorControl regulator_control_from_json(const json& j) {
  RegulatorControl c;
  c.index = j.at("index").get<int>();
  c.name = jget<std::string>(j, "name", "");
  c.transformer_index = jget(j, "transformer_index", 0);
  c.transformer_name = jget<std::string>(j, "transformer_name", "");
  c.winding = jget(j, "winding", 0);
  c.tap_winding = jget(j, "tap_winding", 0);
  c.monitored_bus = jget(j, "monitored_bus", 0);
  c.monitored_node = jget(j, "monitored_node", 1);
  c.vreg_volts = jget(j, "vreg_volts", 0.0);
  c.band_volts = jget(j, "band_volts", 0.0);
  c.ptratio = jget(j, "ptratio", 0.0);
  c.remote_ptratio = jget(j, "remote_ptratio", 0.0);
  c.ct_primary_amps = jget(j, "ct_primary_amps", 0.0);
  c.r_volts = jget(j, "r_volts", 0.0);
  c.x_volts = jget(j, "x_volts", 0.0);
  c.max_tap_change = jget(j, "max_tap_change", 1);
  c.reversible = jget(j, "reversible", false);
  c.enabled = jget(j, "enabled", true);
  return c;
}

static json three_phase_regulator_control_to_json(
    const ThreePhaseRegulatorControl& c) {
  json j;
  j["index"] = c.index;
  j["name"] = c.name;
  j["transformer_index"] = c.transformer_index;
  j["transformer_name"] = c.transformer_name;
  j["winding"] = c.winding;
  j["tap_winding"] = c.tap_winding;
  j["monitored_bus"] = c.monitored_bus;
  j["monitored_node"] = c.monitored_node;
  j["vreg_volts"] = c.vreg_volts;
  j["band_volts"] = c.band_volts;
  j["ptratio"] = c.ptratio;
  j["remote_ptratio"] = c.remote_ptratio;
  j["ct_primary_amps"] = c.ct_primary_amps;
  j["r_volts"] = c.r_volts;
  j["x_volts"] = c.x_volts;
  j["max_tap_change"] = c.max_tap_change;
  j["reversible"] = c.reversible;
  j["enabled"] = c.enabled;
  return j;
}

static ThreePhaseRegulatorControl three_phase_regulator_control_from_json(
    const json& j) {
  ThreePhaseRegulatorControl c;
  c.index = j.at("index").get<int>();
  c.name = jget<std::string>(j, "name", "");
  c.transformer_index = jget(j, "transformer_index", 0);
  c.transformer_name = jget<std::string>(j, "transformer_name", "");
  c.winding = jget(j, "winding", 0);
  c.tap_winding = jget(j, "tap_winding", 0);
  c.monitored_bus = jget(j, "monitored_bus", 0);
  c.monitored_node = jget(j, "monitored_node", 1);
  c.vreg_volts = jget(j, "vreg_volts", 0.0);
  c.band_volts = jget(j, "band_volts", 0.0);
  c.ptratio = jget(j, "ptratio", 0.0);
  c.remote_ptratio = jget(j, "remote_ptratio", 0.0);
  c.ct_primary_amps = jget(j, "ct_primary_amps", 0.0);
  c.r_volts = jget(j, "r_volts", 0.0);
  c.x_volts = jget(j, "x_volts", 0.0);
  c.max_tap_change = jget(j, "max_tap_change", 1);
  c.reversible = jget(j, "reversible", false);
  c.enabled = jget(j, "enabled", true);
  return c;
}

static json three_phase_system_to_json(const ThreePhaseACSystem& sys) {
  json j;
  j["name"] = sys.name;
  j["base_mva"] = sys.base_mva;
  j["base_freq_hz"] = sys.base_freq_hz;
  j["buses"] = json::array();
  for (const auto& b : sys.buses) j["buses"].push_back(three_phase_bus_to_json(b));
  j["lines"] = json::array();
  for (const auto& l : sys.lines) j["lines"].push_back(three_phase_line_to_json(l));
  j["transformers"] = json::array();
  for (const auto& t : sys.transformers) j["transformers"].push_back(three_phase_transformer_to_json(t));
  j["loads"] = json::array();
  for (const auto& l : sys.loads) j["loads"].push_back(three_phase_load_to_json(l));
  j["generators"] = json::array();
  for (const auto& g : sys.generators) j["generators"].push_back(three_phase_generator_to_json(g));
  j["external_grids"] = json::array();
  for (const auto& e : sys.external_grids) j["external_grids"].push_back(three_phase_external_grid_to_json(e));
  j["regulator_controls"] = json::array();
  for (const auto& c : sys.regulator_controls) {
    j["regulator_controls"].push_back(three_phase_regulator_control_to_json(c));
  }
  return j;
}

static ThreePhaseACSystem three_phase_system_from_json(const json& j) {
  ThreePhaseACSystem sys;
  sys.name = jget<std::string>(j, "name", "Three-Phase AC System");
  sys.base_mva = jget(j, "base_mva", 100.0);
  sys.base_freq_hz = jget(j, "base_freq_hz", 50.0);
  if (j.contains("buses")) {
    for (const auto& bj : j["buses"]) sys.buses.push_back(three_phase_bus_from_json(bj));
  }
  if (j.contains("lines")) {
    for (const auto& lj : j["lines"]) sys.lines.push_back(three_phase_line_from_json(lj));
  }
  if (j.contains("transformers")) {
    for (const auto& tj : j["transformers"]) sys.transformers.push_back(three_phase_transformer_from_json(tj));
  }
  if (j.contains("loads")) {
    for (const auto& lj : j["loads"]) sys.loads.push_back(three_phase_load_from_json(lj));
  }
  if (j.contains("generators")) {
    for (const auto& gj : j["generators"]) sys.generators.push_back(three_phase_generator_from_json(gj));
  }
  if (j.contains("external_grids")) {
    for (const auto& ej : j["external_grids"]) sys.external_grids.push_back(three_phase_external_grid_from_json(ej));
  }
  if (j.contains("regulator_controls")) {
    for (const auto& cj : j["regulator_controls"]) sys.regulator_controls.push_back(three_phase_regulator_control_from_json(cj));
  }
  return sys;
}

static json switch_to_json(const Switch& s) {
  json j;
  j["index"] = s.index;
  j["name"] = s.name;
  j["bus_from"] = s.bus_from;
  j["bus_to"] = s.bus_to;
  j["in_service"] = s.in_service;
  j["switch_type"] = switch_type_str(s.switch_type);
  j["closed"] = s.closed;
  j["r_contact_ohm"] = s.r_contact_ohm;
  j["z_ohm"] = s.z_ohm;
  j["i_rated_ka"] = s.i_rated_ka;
  j["i_breaking_ka"] = s.i_breaking_ka;
  j["element_type"] = s.element_type;
  j["element_id"] = s.element_id;
  j["is_remote"] = s.is_remote;
  j["is_automated"] = s.is_automated;
  j["t_operation_s"] = s.t_operation_s;
  j["p_sw_fail"] = s.p_sw_fail;
  j["mtbf_hours"] = s.mtbf_hours;
  j["mttr_hours"] = s.mttr_hours;
  return j;
}

static Switch switch_from_json(const json& j) {
  Switch s;
  s.index = j.at("index").get<int>();
  s.name = jget<std::string>(j, "name", "");
  s.bus_from = j.at("bus_from").get<int>();
  s.bus_to = j.at("bus_to").get<int>();
  s.in_service = jget(j, "in_service", true);
  s.switch_type = switch_type_from_str(jget<std::string>(j, "switch_type", "CircuitBreaker"));
  s.closed = jget(j, "closed", true);
  s.r_contact_ohm = jget(j, "r_contact_ohm", 0.0);
  s.z_ohm = jget(j, "z_ohm", 0.0);
  s.i_rated_ka = jget(j, "i_rated_ka", 0.0);
  s.i_breaking_ka = jget(j, "i_breaking_ka", 0.0);
  s.element_type = jget(j, "element_type", 0);
  s.element_id = jget(j, "element_id", 0);
  s.is_remote = jget(j, "is_remote", false);
  s.is_automated = jget(j, "is_automated", false);
  s.t_operation_s = jget(j, "t_operation_s", 0.0);
  s.p_sw_fail = jget(j, "p_sw_fail", 0.0);
  s.mtbf_hours = jget_alias(j, "mtbf_hours", "mtbf_hours", 0.0);
  s.mttr_hours = jget_alias(j, "mttr_hours", "mttr_hours", 0.0);
  return s;
}

static json circuit_breaker_to_json(const CircuitBreaker& cb) {
  json j;
  j["index"] = cb.index;
  j["name"] = cb.name;
  j["bus_from"] = cb.bus_from;
  j["bus_to"] = cb.bus_to;
  j["in_service"] = cb.in_service;
  j["breaker_type"] = breaker_type_str(cb.breaker_type);
  j["closed"] = cb.closed;
  j["z_ohm"] = cb.z_ohm;
  j["rated_voltage_kv"] = cb.rated_voltage_kv;
  j["i_rated_ka"] = cb.i_rated_ka;
  j["i_breaking_ka"] = cb.i_breaking_ka;
  j["element_type"] = cb.element_type;
  j["element_id"] = cb.element_id;
  return j;
}

static CircuitBreaker circuit_breaker_from_json(const json& j) {
  CircuitBreaker cb;
  cb.index = j.at("index").get<int>();
  cb.name = jget<std::string>(j, "name", "");
  cb.bus_from = j.at("bus_from").get<int>();
  cb.bus_to = j.at("bus_to").get<int>();
  cb.in_service = jget(j, "in_service", true);
  cb.breaker_type = breaker_type_from_str(jget<std::string>(j, "breaker_type", "CB"));
  cb.closed = jget(j, "closed", true);
  cb.z_ohm = jget(j, "z_ohm", 0.0);
  cb.rated_voltage_kv = jget(j, "rated_voltage_kv", 0.0);
  cb.i_rated_ka = jget(j, "i_rated_ka", 0.0);
  cb.i_breaking_ka = jget(j, "i_breaking_ka", 0.0);
  cb.element_type = jget<std::string>(j, "element_type", "");
  cb.element_id = jget(j, "element_id", 0);
  return cb;
}

static json dc_circuit_breaker_to_json(const DCCircuitBreaker& cb) {
  json j;
  j["index"] = cb.index;
  j["name"] = cb.name;
  j["bus_from"] = cb.bus_from;
  j["bus_to"] = cb.bus_to;
  j["in_service"] = cb.in_service;
  j["breaker_type"] = breaker_type_str(cb.breaker_type);
  j["closed"] = cb.closed;
  j["r_ohm"] = cb.r_ohm;
  j["rated_voltage_kv"] = cb.rated_voltage_kv;
  j["i_rated_ka"] = cb.i_rated_ka;
  j["i_breaking_ka"] = cb.i_breaking_ka;
  j["element_type"] = cb.element_type;
  j["element_id"] = cb.element_id;
  return j;
}

static DCCircuitBreaker dc_circuit_breaker_from_json(const json& j) {
  DCCircuitBreaker cb;
  cb.index = j.at("index").get<int>();
  cb.name = jget<std::string>(j, "name", "");
  cb.bus_from = j.at("bus_from").get<int>();
  cb.bus_to = j.at("bus_to").get<int>();
  cb.in_service = jget(j, "in_service", true);
  cb.breaker_type = breaker_type_from_str(jget<std::string>(j, "breaker_type", "CB"));
  cb.closed = jget(j, "closed", true);
  cb.r_ohm = jget(j, "r_ohm", 0.0);
  cb.rated_voltage_kv = jget(j, "rated_voltage_kv", 0.0);
  cb.i_rated_ka = jget(j, "i_rated_ka", 0.0);
  cb.i_breaking_ka = jget(j, "i_breaking_ka", 0.0);
  cb.element_type = jget<std::string>(j, "element_type", "");
  cb.element_id = jget(j, "element_id", 0);
  return cb;
}

static json charging_station_to_json(const ChargingStation& c) {
  json j;
  j["index"] = c.index;
  j["name"] = c.name;
  j["bus"] = c.bus;
  j["location"] = c.location;
  j["in_service"] = c.in_service;
  j["n_fast"] = c.n_fast;
  j["n_slow"] = c.n_slow;
  j["num_chargers"] = c.num_chargers;
  j["p_fast_max_kw"] = c.p_fast_max_kw;
  j["p_slow_max_kw"] = c.p_slow_max_kw;
  j["max_power_kw"] = c.max_power_kw;
  j["simultaneity_factor"] = c.simultaneity_factor;
  j["power_factor"] = c.power_factor;
  j["utilization_rate"] = c.utilization_rate;
  j["p_total_kw"] = c.p_total_kw;
  j["q_total_kvar"] = c.q_total_kvar;
  j["mtbf_hours"] = c.mtbf_hours;
  j["mttr_hours"] = c.mttr_hours;
  return j;
}

static ChargingStation charging_station_from_json(const json& j) {
  ChargingStation c;
  c.index = j.at("index").get<int>();
  c.name = jget<std::string>(j, "name", "");
  c.bus = j.at("bus").get<int>();
  c.location = jget<std::string>(j, "location", "");
  c.in_service = jget(j, "in_service", true);
  c.n_fast = jget(j, "n_fast", 0);
  c.n_slow = jget(j, "n_slow", 0);
  c.num_chargers = jget(j, "num_chargers", 0);
  c.p_fast_max_kw = jget(j, "p_fast_max_kw", 0.0);
  c.p_slow_max_kw = jget(j, "p_slow_max_kw", 0.0);
  c.max_power_kw = jget(j, "max_power_kw", 0.0);
  c.simultaneity_factor = jget(j, "simultaneity_factor", 1.0);
  c.power_factor = jget(j, "power_factor", 0.95);
  c.utilization_rate = jget(j, "utilization_rate", 0.0);
  c.p_total_kw = jget(j, "p_total_kw", 0.0);
  c.q_total_kvar = jget(j, "q_total_kvar", 0.0);
  c.mtbf_hours = jget_alias(j, "mtbf_hours", "mtbf_hours", 0.0);
  c.mttr_hours = jget_alias(j, "mttr_hours", "mttr_hours", 0.0);
  return c;
}

static json charger_to_json(const Charger& c) {
  json j;
  j["index"] = c.index;
  j["name"] = c.name;
  j["station_id"] = c.station_id;
  j["charger_type"] = charger_type_str(c.charger_type);
  j["in_service"] = c.in_service;
  j["p_rated_kw"] = c.p_rated_kw;
  j["p_ch_max_kw"] = c.p_ch_max_kw;
  j["p_ch_min_kw"] = c.p_ch_min_kw;
  j["eta"] = c.eta;
  j["v2g_capable"] = c.v2g_capable;
  j["p_dis_max_kw"] = c.p_dis_max_kw;
  j["mtbf_hours"] = c.mtbf_hours;
  j["mttr_hours"] = c.mttr_hours;
  return j;
}

static Charger charger_from_json(const json& j) {
  Charger c;
  c.index = j.at("index").get<int>();
  c.name = jget<std::string>(j, "name", "");
  c.station_id = jget(j, "station_id", 0);
  c.charger_type = charger_type_from_str(jget<std::string>(j, "charger_type", "AC_L2"));
  c.in_service = jget(j, "in_service", true);
  c.p_rated_kw = jget(j, "p_rated_kw", 0.0);
  c.p_ch_max_kw = jget(j, "p_ch_max_kw", 0.0);
  c.p_ch_min_kw = jget(j, "p_ch_min_kw", 0.0);
  c.eta = jget(j, "eta", 0.95);
  c.v2g_capable = jget(j, "v2g_capable", false);
  c.p_dis_max_kw = jget(j, "p_dis_max_kw", 0.0);
  c.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  c.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  return c;
}

static json dcdc_to_json(const DCDCConverter& c) {
  json j;
  j["index"] = c.index;
  j["bus_in"] = c.bus_in;
  j["bus_out"] = c.bus_out;
  j["in_service"] = c.in_service;
  j["name"] = c.name;
  j["control_mode"] = dcdc_control_str(c.control_mode);
  j["p_ref_mw"] = c.p_ref_mw;
  j["v_ref_pu"] = c.v_ref_pu;
  j["sn_mva"] = c.sn_mva;
  j["vn_in_kv"] = c.vn_in_kv;
  j["vn_out_kv"] = c.vn_out_kv;
  j["eta"] = c.eta;
  j["r_eq_pu"] = c.r_eq_pu;
  j["pmax_mw"] = c.pmax_mw;
  j["pmin_mw"] = c.pmin_mw;
  j["k_droop"] = c.k_droop;
  j["topology"] = dcdc_topology_str(c.topology);
  j["d_min"] = c.d_min;
  j["d_max"] = c.d_max;
  j["n_ratio"] = c.n_ratio;
  j["mtbf_hours"] = c.mtbf_hours;
  j["mttr_hours"] = c.mttr_hours;
  add_dynamic_model_if_present(j, c.dynamic_model);
  return j;
}

static DCDCConverter dcdc_from_json(const json& j) {
  DCDCConverter c;
  c.index = j.at("index").get<int>();
  c.bus_in = j.at("bus_in").get<int>();
  c.bus_out = j.at("bus_out").get<int>();
  c.in_service = jget(j, "in_service", true);
  c.name = jget<std::string>(j, "name", "");
  c.control_mode = dcdc_control_from_str(jget<std::string>(j, "control_mode", "Voltage"));
  c.p_ref_mw = jget(j, "p_ref_mw", 0.0);
  c.v_ref_pu = jget(j, "v_ref_pu", 1.0);
  c.sn_mva = jget(j, "sn_mva", 0.0);
  c.vn_in_kv = jget(j, "vn_in_kv", 0.0);
  c.vn_out_kv = jget(j, "vn_out_kv", 0.0);
  c.eta = jget(j, "eta", 0.98);
  c.r_eq_pu = jget(j, "r_eq_pu", 0.0);
  c.pmax_mw = jget(j, "pmax_mw", 0.0);
  c.pmin_mw = jget(j, "pmin_mw", 0.0);
  c.k_droop = jget(j, "k_droop", 0.0);
  c.topology = dcdc_topology_from_str(jget<std::string>(j, "topology", "Generic"));
  c.d_min = jget(j, "d_min", 0.05);
  c.d_max = jget(j, "d_max", 0.95);
  c.n_ratio = jget(j, "n_ratio", 1.0);
  c.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  c.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, c.dynamic_model);
  return c;
}

static json er_port_to_json(const EnergyRouterPort& p) {
  json j;
  j["index"] = p.index;
  j["name"] = p.name;
  j["bus"] = p.bus;
  j["side"] = p.side;
  j["port_type"] = er_port_type_str(p.port_type);
  j["voltage_level_kv"] = p.voltage_level_kv;
  j["p_mw"] = p.p_mw;
  j["q_mvar"] = p.q_mvar;
  j["v_pu"] = p.v_pu;
  j["pmax_mw"] = p.pmax_mw;
  j["pmin_mw"] = p.pmin_mw;
  j["qmax_mvar"] = p.qmax_mvar;
  j["qmin_mvar"] = p.qmin_mvar;
  j["control_mode"] = er_control_str(p.control_mode);
  j["p_set_mw"] = p.p_set_mw;
  j["q_set_mvar"] = p.q_set_mvar;
  j["v_set_pu"] = p.v_set_pu;
  j["in_service"] = p.in_service;
  add_dynamic_model_if_present(j, p.dynamic_model);
  return j;
}

static EnergyRouterPort er_port_from_json(const json& j) {
  EnergyRouterPort p;
  p.index = j.at("index").get<int>();
  p.name = jget<std::string>(j, "name", "");
  p.bus = j.at("bus").get<int>();
  p.side = jget(j, "side", 0);
  p.port_type = er_port_type_from_str(jget<std::string>(j, "port_type", "AC"));
  p.voltage_level_kv = jget(j, "voltage_level_kv", 0.0);
  p.p_mw = jget(j, "p_mw", 0.0);
  p.q_mvar = jget(j, "q_mvar", 0.0);
  p.v_pu = jget(j, "v_pu", 1.0);
  p.pmax_mw = jget(j, "pmax_mw", 0.0);
  p.pmin_mw = jget(j, "pmin_mw", 0.0);
  p.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  p.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  p.control_mode = er_control_from_str(jget<std::string>(j, "control_mode", "PQ"));
  p.p_set_mw = jget(j, "p_set_mw", 0.0);
  p.q_set_mvar = jget(j, "q_set_mvar", 0.0);
  p.v_set_pu = jget(j, "v_set_pu", 1.0);
  p.in_service = jget(j, "in_service", true);
  read_dynamic_model_if_present(j, p.dynamic_model);
  return p;
}

static json energy_router_to_json(const EnergyRouter& r) {
  json j;
  j["index"] = r.index;
  j["name"] = r.name;
  j["in_service"] = r.in_service;
  j["router_type"] = r.router_type;
  j["num_ports"] = r.num_ports;
  j["ports"] = json::array();
  for (const auto& p : r.ports) j["ports"].push_back(er_port_to_json(p));
  j["p_rated_mw"] = r.p_rated_mw;
  j["vn_ac_kv"] = r.vn_ac_kv;
  j["vn_dc_kv"] = r.vn_dc_kv;
  j["loss_percent"] = r.loss_percent;
  j["pmax_mw"] = r.pmax_mw;
  j["pmin_mw"] = r.pmin_mw;
  j["qmax_mvar"] = r.qmax_mvar;
  j["qmin_mvar"] = r.qmin_mvar;
  j["mtbf_hours"] = r.mtbf_hours;
  j["mttr_hours"] = r.mttr_hours;
  add_dynamic_model_if_present(j, r.dynamic_model);
  return j;
}

static EnergyRouter energy_router_from_json(const json& j) {
  EnergyRouter r;
  r.index = j.at("index").get<int>();
  r.name = jget<std::string>(j, "name", "");
  r.in_service = jget(j, "in_service", true);
  r.router_type = jget<std::string>(j, "router_type", "");
  r.num_ports = jget(j, "num_ports", 0);
  if (j.contains("ports")) {
    for (const auto& pj : j["ports"]) r.ports.push_back(er_port_from_json(pj));
  }
  r.p_rated_mw = jget(j, "p_rated_mw", 0.0);
  r.vn_ac_kv = jget(j, "vn_ac_kv", 0.0);
  r.vn_dc_kv = jget(j, "vn_dc_kv", 0.0);
  r.loss_percent = jget(j, "loss_percent", 0.0);
  r.pmax_mw = jget(j, "pmax_mw", 0.0);
  r.pmin_mw = jget(j, "pmin_mw", 0.0);
  r.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  r.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  r.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  r.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, r.dynamic_model);
  return r;
}

static json mobile_storage_to_json(const MobileStorage& s) {
  json j;
  j["index"] = s.index;
  j["bus"] = s.bus;
  j["in_service"] = s.in_service;
  j["name"] = s.name;
  j["p_mw"] = s.p_mw;
  j["q_mvar"] = s.q_mvar;
  j["p_rated_mw"] = s.p_rated_mw;
  j["pmax_mw"] = s.pmax_mw;
  j["pmin_mw"] = s.pmin_mw;
  j["qmax_mvar"] = s.qmax_mvar;
  j["qmin_mvar"] = s.qmin_mvar;
  j["e_rated_mwh"] = s.e_rated_mwh;
  j["soc_init"] = s.soc_init;
  j["soc_min"] = s.soc_min;
  j["soc_max"] = s.soc_max;
  j["eta_charge"] = s.eta_charge;
  j["eta_discharge"] = s.eta_discharge;
  j["max_cycles"] = s.max_cycles;
  j["current_cycles"] = s.current_cycles;
  j["soh"] = s.soh;
  j["is_mobile"] = s.is_mobile;
  j["status"] = mobile_storage_status_str(s.status);
  j["current_location"] = s.current_location;
  j["target_bus"] = s.target_bus;
  j["e_consumption_mwh_km"] = s.e_consumption_mwh_km;
  j["max_travel_distance_km"] = s.max_travel_distance_km;
  j["mtbf_hours"] = s.mtbf_hours;
  j["mttr_hours"] = s.mttr_hours;
  add_dynamic_model_if_present(j, s.dynamic_model);
  return j;
}

static MobileStorage mobile_storage_from_json(const json& j) {
  MobileStorage s;
  s.index = j.at("index").get<int>();
  s.bus = j.at("bus").get<int>();
  s.in_service = jget(j, "in_service", true);
  s.name = jget<std::string>(j, "name", "");
  s.p_mw = jget(j, "p_mw", 0.0);
  s.q_mvar = jget(j, "q_mvar", 0.0);
  s.p_rated_mw = jget(j, "p_rated_mw", 0.0);
  s.pmax_mw = jget(j, "pmax_mw", 0.0);
  s.pmin_mw = jget(j, "pmin_mw", 0.0);
  s.qmax_mvar = jget(j, "qmax_mvar", 0.0);
  s.qmin_mvar = jget(j, "qmin_mvar", 0.0);
  s.e_rated_mwh = jget(j, "e_rated_mwh", 0.0);
  s.soc_init = jget(j, "soc_init", 0.5);
  s.soc_min = jget(j, "soc_min", 0.1);
  s.soc_max = jget(j, "soc_max", 0.9);
  s.eta_charge = jget(j, "eta_charge", 0.95);
  s.eta_discharge = jget(j, "eta_discharge", 0.95);
  s.max_cycles = jget(j, "max_cycles", 5000);
  s.current_cycles = jget(j, "current_cycles", 0);
  s.soh = jget(j, "soh", 1.0);
  s.is_mobile = jget(j, "is_mobile", true);
  s.status = mobile_storage_status_from_str(jget<std::string>(j, "status", "Stationary"));
  s.current_location = jget<std::string>(j, "current_location", "");
  s.target_bus = jget(j, "target_bus", 0);
  s.e_consumption_mwh_km = jget(j, "e_consumption_mwh_km", 0.0);
  s.max_travel_distance_km = jget(j, "max_travel_distance_km", 0.0);
  s.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  s.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, s.dynamic_model);
  return s;
}

static json vpp_to_json(const VirtualPowerPlant& v) {
  json j;
  j["index"] = v.index;
  j["name"] = v.name;
  j["description"] = v.description;
  j["pcc_bus"] = v.pcc_bus;
  j["in_service"] = v.in_service;
  j["n_pv_systems"] = v.n_pv_systems;
  j["n_wind_turbines"] = v.n_wind_turbines;
  j["n_battery_systems"] = v.n_battery_systems;
  j["n_ev_chargers"] = v.n_ev_chargers;
  j["n_controllable_loads"] = v.n_controllable_loads;
  j["p_generation_sum_mw"] = v.p_generation_sum_mw;
  j["e_storage_sum_mwh"] = v.e_storage_sum_mwh;
  j["p_load_controllable_mw"] = v.p_load_controllable_mw;
  j["p_output_mw"] = v.p_output_mw;
  j["q_output_mvar"] = v.q_output_mvar;
  j["pmax_mw"] = v.pmax_mw;
  j["pmin_mw"] = v.pmin_mw;
  j["ramp_up_max_mw_min"] = v.ramp_up_max_mw_min;
  j["ramp_down_max_mw_min"] = v.ramp_down_max_mw_min;
  j["mtbf_hours"] = v.mtbf_hours;
  j["mttr_hours"] = v.mttr_hours;
  add_dynamic_model_if_present(j, v.dynamic_model);
  return j;
}

static VirtualPowerPlant vpp_from_json(const json& j) {
  VirtualPowerPlant v;
  v.index = j.at("index").get<int>();
  v.name = jget<std::string>(j, "name", "");
  v.description = jget<std::string>(j, "description", "");
  v.pcc_bus = jget_alias(j, "pcc_bus", "aggregation_bus", 0);
  v.in_service = jget(j, "in_service", true);
  v.n_pv_systems = jget(j, "n_pv_systems", 0);
  v.n_wind_turbines = jget(j, "n_wind_turbines", 0);
  v.n_battery_systems = jget(j, "n_battery_systems", 0);
  v.n_ev_chargers = jget(j, "n_ev_chargers", 0);
  v.n_controllable_loads = jget(j, "n_controllable_loads", 0);
  v.p_generation_sum_mw = jget(j, "p_generation_sum_mw", 0.0);
  v.e_storage_sum_mwh = jget(j, "e_storage_sum_mwh", 0.0);
  v.p_load_controllable_mw = jget(j, "p_load_controllable_mw", 0.0);
  v.p_output_mw = jget(j, "p_output_mw", 0.0);
  v.q_output_mvar = jget(j, "q_output_mvar", 0.0);
  v.pmax_mw = jget(j, "pmax_mw", 0.0);
  v.pmin_mw = jget(j, "pmin_mw", 0.0);
  v.ramp_up_max_mw_min = jget(j, "ramp_up_max_mw_min", 0.0);
  v.ramp_down_max_mw_min = jget(j, "ramp_down_max_mw_min", 0.0);
  v.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  v.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, v.dynamic_model);
  return v;
}

static json microgrid_to_json(const Microgrid& m) {
  json j;
  j["index"] = m.index;
  j["name"] = m.name;
  j["description"] = m.description;
  j["in_service"] = m.in_service;
  j["pcc_bus"] = m.pcc_bus;
  j["internal_buses"] = m.internal_buses;
  j["operating_mode"] = microgrid_mode_str(m.operating_mode);
  j["islanding_capability"] = m.islanding_capability;
  j["auto_reconnection"] = m.auto_reconnection;
  j["p_exchange_max_mw"] = m.p_exchange_max_mw;
  j["p_exchange_min_mw"] = m.p_exchange_min_mw;
  j["p_import_max_mw"] = m.p_import_max_mw;
  j["p_export_max_mw"] = m.p_export_max_mw;
  j["p_exchange_mw"] = m.p_exchange_mw;
  j["total_generation_mw"] = m.total_generation_mw;
  j["total_storage_mwh"] = m.total_storage_mwh;
  j["total_load_mw"] = m.total_load_mw;
  j["capacity_mw"] = m.capacity_mw;
  j["peak_load_mw"] = m.peak_load_mw;
  j["f_set_hz"] = m.f_set_hz;
  j["v_set_pu"] = m.v_set_pu;
  j["k_droop"] = m.k_droop;
  j["area"] = m.area;
  j["mtbf_hours"] = m.mtbf_hours;
  j["mttr_hours"] = m.mttr_hours;
  add_dynamic_model_if_present(j, m.dynamic_model);
  return j;
}

static Microgrid microgrid_from_json(const json& j) {
  Microgrid m;
  m.index = j.at("index").get<int>();
  m.name = jget<std::string>(j, "name", "");
  m.description = jget<std::string>(j, "description", "");
  m.in_service = jget(j, "in_service", true);
  m.pcc_bus = jget_alias(j, "pcc_bus", "aggregation_bus", 0);
  if (j.contains("internal_buses")) m.internal_buses = j["internal_buses"].get<std::vector<int>>();
  m.operating_mode = microgrid_mode_from_str(jget<std::string>(j, "operating_mode", "GridConnected"));
  m.islanding_capability = jget(j, "islanding_capability", false);
  m.auto_reconnection = jget(j, "auto_reconnection", false);
  m.p_exchange_max_mw = jget(j, "p_exchange_max_mw", 0.0);
  m.p_exchange_min_mw = jget(j, "p_exchange_min_mw", 0.0);
  m.p_import_max_mw = jget(j, "p_import_max_mw", 0.0);
  m.p_export_max_mw = jget(j, "p_export_max_mw", 0.0);
  m.p_exchange_mw = jget(j, "p_exchange_mw", 0.0);
  m.total_generation_mw = jget(j, "total_generation_mw", 0.0);
  m.total_storage_mwh = jget(j, "total_storage_mwh", 0.0);
  m.total_load_mw = jget(j, "total_load_mw", 0.0);
  m.capacity_mw = jget(j, "capacity_mw", 0.0);
  m.peak_load_mw = jget(j, "peak_load_mw", 0.0);
  m.f_set_hz = jget(j, "f_set_hz", 50.0);
  m.v_set_pu = jget(j, "v_set_pu", 1.0);
  m.k_droop = jget(j, "k_droop", 0.0);
  m.area = jget(j, "area", 0);
  m.mtbf_hours = jget_alias(j, "mtbf_hr", "mtbf_hours", 0.0);
  m.mttr_hours = jget_alias(j, "mttr_hr", "mttr_hours", 0.0);
  read_dynamic_model_if_present(j, m.dynamic_model);
  return m;
}

// ═══════════════════════════════════════════════════════════════════════
// Top-level serialization
// ═══════════════════════════════════════════════════════════════════════
// ── Telemetry section serialization (§10) ────────────────────────────────────
namespace {

json time_base_to_json(const TimeBase& tb) {
  json j;
  j["epoch_utc"] = tb.epoch_utc;
  j["timezone"] = tb.timezone;
  j["resolution_s"] = tb.resolution_s;
  j["resample"] = to_string(tb.resample);
  return j;
}

TimeBase time_base_from_json(const json& j) {
  TimeBase tb;
  tb.epoch_utc = jget<std::string>(j, "epoch_utc", std::string());
  tb.timezone = jget<std::string>(j, "timezone", std::string("UTC"));
  tb.resolution_s = jget(j, "resolution_s", 1.0);
  tb.resample =
      resample_policy_from_string(jget<std::string>(j, "resample", std::string("hold")));
  return tb;
}

json telemetry_binding_to_json(const TelemetryBinding& b) {
  json j;
  j["component_ref"] = b.component_ref;
  j["measurement_type"] = to_string(b.measurement_type);
  j["phase"] = b.phase;
  j["unit"] = b.unit;
  j["sign_convention"] = to_string(b.sign_convention);
  j["reference_frame"] = to_string(b.reference_frame);
  j["source_system"] = b.source_system;
  j["tag"] = b.tag;
  j["sampling_interval_s"] = b.sampling_interval_s;
  j["deadband"] = b.deadband;
  j["scale"] = b.scale;
  j["cardinality"] = b.cardinality;
  j["constrains_state"] = b.constrains_state;
  j["quality"] = to_string(b.quality);
  j["timestamp"] = b.timestamp;
  return j;
}

TelemetryBinding telemetry_binding_from_json(const json& j) {
  TelemetryBinding b;
  b.component_ref = jget<std::string>(j, "component_ref", std::string());
  b.measurement_type = measurement_type_from_string(
      jget<std::string>(j, "measurement_type", std::string("voltage")));
  b.phase = jget<std::string>(j, "phase", std::string());
  b.unit = jget<std::string>(j, "unit", std::string());
  b.sign_convention = sign_convention_from_string(
      jget<std::string>(j, "sign_convention", std::string("load")));
  b.reference_frame = reference_frame_from_string(
      jget<std::string>(j, "reference_frame", std::string("phase_ground")));
  b.source_system = jget<std::string>(j, "source_system", std::string());
  b.tag = jget<std::string>(j, "tag", std::string());
  b.sampling_interval_s = jget(j, "sampling_interval_s", 1.0);
  b.deadband = jget(j, "deadband", 0.0);
  b.scale = jget(j, "scale", 1.0);
  b.cardinality = jget<std::string>(j, "cardinality", std::string("1:1"));
  b.constrains_state = jget<std::string>(j, "constrains_state", std::string());
  b.quality =
      quality_flag_from_string(jget<std::string>(j, "quality", std::string("good")));
  b.timestamp = jget<std::string>(j, "timestamp", std::string());
  return b;
}

json telemetry_sample_to_json(const TelemetrySample& s) {
  return json{{"timestamp", s.timestamp},
              {"value", s.value},
              {"quality", to_string(s.quality)}};
}

TelemetrySample telemetry_sample_from_json(const json& j) {
  TelemetrySample s;
  s.timestamp = jget<std::string>(j, "timestamp", std::string());
  s.value = jget(j, "value", 0.0);
  s.quality =
      quality_flag_from_string(jget<std::string>(j, "quality", std::string("good")));
  return s;
}

json telemetry_stream_to_json(const TelemetryStream& st) {
  json j;
  j["stream_id"] = st.stream_id;
  j["time_base"] = time_base_to_json(st.time_base);
  j["bindings"] = json::array();
  for (const auto& b : st.bindings)
    j["bindings"].push_back(telemetry_binding_to_json(b));
  j["samples"] = json::array();
  for (const auto& s : st.samples)
    j["samples"].push_back(telemetry_sample_to_json(s));
  return j;
}

TelemetryStream telemetry_stream_from_json(const json& j) {
  TelemetryStream st;
  st.stream_id = jget<std::string>(j, "stream_id", std::string());
  if (j.contains("time_base")) st.time_base = time_base_from_json(j.at("time_base"));
  if (j.contains("bindings"))
    for (const auto& b : j.at("bindings"))
      st.bindings.push_back(telemetry_binding_from_json(b));
  if (j.contains("samples"))
    for (const auto& s : j.at("samples"))
      st.samples.push_back(telemetry_sample_from_json(s));
  return st;
}

json state_seed_to_json(const StateSeed& s) {
  json j;
  j["timestamp"] = s.timestamp;
  j["component_ref"] = s.component_ref;
  j["quantity"] = s.quantity;
  j["value"] = s.value;
  j["provenance"] = s.provenance;
  return j;
}

StateSeed state_seed_from_json(const json& j) {
  StateSeed s;
  s.timestamp = jget<std::string>(j, "timestamp", std::string());
  s.component_ref = jget<std::string>(j, "component_ref", std::string());
  s.quantity = jget<std::string>(j, "quantity", std::string());
  s.value = jget(j, "value", 0.0);
  s.provenance = jget<std::string>(j, "provenance", std::string());
  return s;
}

json telemetry_section_to_json(const TelemetrySection& sec) {
  json j;
  j["streams"] = json::array();
  for (const auto& st : sec.streams)
    j["streams"].push_back(telemetry_stream_to_json(st));
  j["state_seeds"] = json::array();
  for (const auto& s : sec.state_seeds)
    j["state_seeds"].push_back(state_seed_to_json(s));
  return j;
}

TelemetrySection telemetry_section_from_json(const json& j) {
  TelemetrySection sec;
  if (j.contains("streams"))
    for (const auto& st : j.at("streams"))
      sec.streams.push_back(telemetry_stream_from_json(st));
  if (j.contains("state_seeds"))
    for (const auto& s : j.at("state_seeds"))
      sec.state_seeds.push_back(state_seed_from_json(s));
  return sec;
}

}  // namespace

std::string to_json(const HybridPowerSystem& sys, int indent) {
  json root;
  root["schema_version"] = kCurrentSchemaVersion;  // §3.3 on-disk contract stamp
  root["name"] = sys.name;
  root["base_mva"] = sys.base_mva;

  // AC system
  json ac;
  ac["name"] = sys.ac.name;
  ac["base_mva"] = sys.ac.base_mva;
  ac["freq_hz"] = sys.ac.freq_hz;

  ac["buses"] = json::array();
  for (const auto& b : sys.ac.buses) ac["buses"].push_back(ac_bus_to_json(b));

  ac["branches"] = json::array();
  for (const auto& br : sys.ac.branches) ac["branches"].push_back(ac_branch_to_json(br));

  ac["generators"] = json::array();
  for (const auto& g : sys.ac.generators) ac["generators"].push_back(generator_to_json(g));

  ac["loads"] = json::array();
  for (const auto& l : sys.ac.loads) ac["loads"].push_back(load_to_json(l));

  ac["shunts"] = json::array();
  for (const auto& s : sys.ac.shunts) ac["shunts"].push_back(shunt_to_json(s));

  ac["storage"] = json::array();
  for (const auto& s : sys.ac.storage) ac["storage"].push_back(storage_to_json(s));

  ac["renewable_gens"] = json::array();
  for (const auto& r : sys.ac.renewable_gens) ac["renewable_gens"].push_back(renewable_gen_to_json(r));

  ac["static_generators"] = json::array();
  for (const auto& g : sys.ac.static_generators) ac["static_generators"].push_back(static_generator_to_json(g));

  ac["flexible_loads"] = json::array();
  for (const auto& l : sys.ac.flexible_loads) ac["flexible_loads"].push_back(flexible_load_to_json(l));

  ac["asymmetric_loads"] = json::array();
  for (const auto& l : sys.ac.asymmetric_loads) ac["asymmetric_loads"].push_back(asymmetric_load_to_json(l));

  ac["pv_systems"] = json::array();
  for (const auto& p : sys.ac.pv_systems) ac["pv_systems"].push_back(pv_system_to_json(p));

  ac["external_grids"] = json::array();
  for (const auto& e : sys.ac.external_grids) ac["external_grids"].push_back(external_grid_to_json(e));

  ac["transformers_2w"] = json::array();
  for (const auto& t : sys.ac.transformers_2w) ac["transformers_2w"].push_back(transformer2w_to_json(t));

  ac["transformers_3w"] = json::array();
  for (const auto& t : sys.ac.transformers_3w) ac["transformers_3w"].push_back(transformer3w_to_json(t));

  ac["regulator_controls"] = json::array();
  for (const auto& c : sys.ac.regulator_controls) ac["regulator_controls"].push_back(regulator_control_to_json(c));

  ac["switches"] = json::array();
  for (const auto& s : sys.ac.switches) ac["switches"].push_back(switch_to_json(s));

  ac["circuit_breakers"] = json::array();
  for (const auto& cb : sys.ac.circuit_breakers) ac["circuit_breakers"].push_back(circuit_breaker_to_json(cb));

  ac["charging_stations"] = json::array();
  for (const auto& c : sys.ac.charging_stations) ac["charging_stations"].push_back(charging_station_to_json(c));

  ac["chargers"] = json::array();
  for (const auto& c : sys.ac.chargers) ac["chargers"].push_back(charger_to_json(c));

  ac["motors"] = json::array();
  for (const auto& m : sys.ac.motors) ac["motors"].push_back(asynchronous_motor_to_json(m));

  root["ac"] = ac;

  // DC system
  json dc;
  dc["name"] = sys.dc.name;
  dc["base_mva"] = sys.dc.base_mva;

  dc["buses"] = json::array();
  for (const auto& b : sys.dc.buses) dc["buses"].push_back(dc_bus_to_json(b));

  dc["branches"] = json::array();
  for (const auto& br : sys.dc.branches) dc["branches"].push_back(dc_branch_to_json(br));

  dc["loads"] = json::array();
  for (const auto& l : sys.dc.loads) dc["loads"].push_back(dc_load_to_json(l));

  dc["storage"] = json::array();
  for (const auto& s : sys.dc.storage) dc["storage"].push_back(storage_to_json(s));

  dc["dc_storage"] = json::array();
  for (const auto& s : sys.dc.dc_storage) dc["dc_storage"].push_back(dc_storage_to_json(s));

  dc["static_generators"] = json::array();
  for (const auto& g : sys.dc.static_generators) dc["static_generators"].push_back(static_generator_to_json(g));

  dc["dc_static_generators"] = json::array();
  for (const auto& g : sys.dc.dc_static_generators) dc["dc_static_generators"].push_back(dc_static_generator_to_json(g));

  dc["pv_arrays"] = json::array();
  for (const auto& p : sys.dc.pv_arrays) dc["pv_arrays"].push_back(pv_array_dc_to_json(p));

  dc["dcdc_converters"] = json::array();
  for (const auto& c : sys.dc.dcdc_converters) dc["dcdc_converters"].push_back(dcdc_to_json(c));

  dc["dc_circuit_breakers"] = json::array();
  for (const auto& cb : sys.dc.dc_circuit_breakers) dc["dc_circuit_breakers"].push_back(dc_circuit_breaker_to_json(cb));

  root["dc"] = dc;

  // Converters
  root["vsc_converters"] = json::array();
  for (const auto& c : sys.vsc_converters) root["vsc_converters"].push_back(vsc_to_json(c));

  root["energy_routers"] = json::array();
  for (const auto& r : sys.energy_routers) root["energy_routers"].push_back(energy_router_to_json(r));

  root["mobile_storage"] = json::array();
  for (const auto& s : sys.mobile_storage) root["mobile_storage"].push_back(mobile_storage_to_json(s));

  root["vpps"] = json::array();
  for (const auto& v : sys.vpps) root["vpps"].push_back(vpp_to_json(v));

  root["microgrids"] = json::array();
  for (const auto& m : sys.microgrids) root["microgrids"].push_back(microgrid_to_json(m));

  if (sys.three_phase_ac.has_value()) {
    root["three_phase_ac"] = three_phase_system_to_json(*sys.three_phase_ac);
  }

  if (sys.telemetry && !sys.telemetry->empty()) {
    root["telemetry"] = telemetry_section_to_json(*sys.telemetry);
  }

  return root.dump(indent);
}

void save_json(const HybridPowerSystem& sys, const std::string& path,
               int indent) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Cannot open file for writing: " + path);
  }
  ofs << to_json(sys, indent);
}

HybridPowerSystem from_json(const std::string& json_str) {
  json root = json::parse(json_str);
  HybridPowerSystem sys;
  sys.name = jget<std::string>(root, "name", "Hybrid AC/DC System");
  sys.base_mva = jget(root, "base_mva", 100.0);

  if (root.contains("ac")) {
    const auto& ac = root["ac"];
    sys.ac.name = jget<std::string>(ac, "name", "AC System");
    sys.ac.base_mva = jget(ac, "base_mva", 100.0);
    sys.ac.freq_hz = jget(ac, "freq_hz", 50.0);

    if (ac.contains("buses"))
      for (const auto& j : ac["buses"]) sys.ac.buses.push_back(ac_bus_from_json(j));
    if (ac.contains("branches"))
      for (const auto& j : ac["branches"]) sys.ac.branches.push_back(ac_branch_from_json(j));
    if (ac.contains("generators"))
      for (const auto& j : ac["generators"]) sys.ac.generators.push_back(generator_from_json(j));
    if (ac.contains("loads"))
      for (const auto& j : ac["loads"]) sys.ac.loads.push_back(load_from_json(j));
    if (ac.contains("shunts"))
      for (const auto& j : ac["shunts"]) sys.ac.shunts.push_back(shunt_from_json(j));
    if (ac.contains("storage"))
      for (const auto& j : ac["storage"]) sys.ac.storage.push_back(storage_from_json(j));
    if (ac.contains("renewable_gens"))
      for (const auto& j : ac["renewable_gens"]) sys.ac.renewable_gens.push_back(renewable_gen_from_json(j));
    if (ac.contains("static_generators"))
      for (const auto& j : ac["static_generators"]) sys.ac.static_generators.push_back(static_generator_from_json(j));
    if (ac.contains("flexible_loads"))
      for (const auto& j : ac["flexible_loads"]) sys.ac.flexible_loads.push_back(flexible_load_from_json(j));
    if (ac.contains("asymmetric_loads"))
      for (const auto& j : ac["asymmetric_loads"]) sys.ac.asymmetric_loads.push_back(asymmetric_load_from_json(j));
    if (ac.contains("pv_systems"))
      for (const auto& j : ac["pv_systems"]) sys.ac.pv_systems.push_back(pv_system_from_json(j));
    if (ac.contains("external_grids"))
      for (const auto& j : ac["external_grids"]) sys.ac.external_grids.push_back(external_grid_from_json(j));
    if (ac.contains("transformers_2w"))
      for (const auto& j : ac["transformers_2w"]) sys.ac.transformers_2w.push_back(transformer2w_from_json(j));
    if (ac.contains("transformers_3w"))
      for (const auto& j : ac["transformers_3w"]) sys.ac.transformers_3w.push_back(transformer3w_from_json(j));
    if (ac.contains("regulator_controls"))
      for (const auto& j : ac["regulator_controls"]) sys.ac.regulator_controls.push_back(regulator_control_from_json(j));
    if (ac.contains("switches"))
      for (const auto& j : ac["switches"]) sys.ac.switches.push_back(switch_from_json(j));
    if (ac.contains("circuit_breakers"))
      for (const auto& j : ac["circuit_breakers"]) sys.ac.circuit_breakers.push_back(circuit_breaker_from_json(j));
    if (ac.contains("charging_stations"))
      for (const auto& j : ac["charging_stations"]) sys.ac.charging_stations.push_back(charging_station_from_json(j));
    if (ac.contains("chargers"))
      for (const auto& j : ac["chargers"]) sys.ac.chargers.push_back(charger_from_json(j));
    if (ac.contains("motors"))
      for (const auto& j : ac["motors"]) sys.ac.motors.push_back(asynchronous_motor_from_json(j));
  }

  if (root.contains("dc")) {
    const auto& dc = root["dc"];
    sys.dc.name = jget<std::string>(dc, "name", "DC System");
    sys.dc.base_mva = jget(dc, "base_mva", 100.0);

    if (dc.contains("buses"))
      for (const auto& j : dc["buses"]) sys.dc.buses.push_back(dc_bus_from_json(j));
    if (dc.contains("branches"))
      for (const auto& j : dc["branches"]) sys.dc.branches.push_back(dc_branch_from_json(j));
    if (dc.contains("loads"))
      for (const auto& j : dc["loads"]) sys.dc.loads.push_back(dc_load_from_json(j));
    if (dc.contains("storage"))
      for (const auto& j : dc["storage"]) sys.dc.storage.push_back(storage_from_json(j));
    if (dc.contains("dc_storage"))
      for (const auto& j : dc["dc_storage"]) sys.dc.dc_storage.push_back(dc_storage_from_json(j));
    if (dc.contains("static_generators"))
      for (const auto& j : dc["static_generators"]) sys.dc.static_generators.push_back(static_generator_from_json(j));
    if (dc.contains("dc_static_generators"))
      for (const auto& j : dc["dc_static_generators"]) sys.dc.dc_static_generators.push_back(dc_static_generator_from_json(j));
    if (dc.contains("pv_arrays"))
      for (const auto& j : dc["pv_arrays"]) sys.dc.pv_arrays.push_back(pv_array_dc_from_json(j));
    if (dc.contains("dcdc_converters"))
      for (const auto& j : dc["dcdc_converters"]) sys.dc.dcdc_converters.push_back(dcdc_from_json(j));
    if (dc.contains("dc_circuit_breakers"))
      for (const auto& j : dc["dc_circuit_breakers"]) sys.dc.dc_circuit_breakers.push_back(dc_circuit_breaker_from_json(j));
  }

  if (root.contains("vsc_converters"))
    for (const auto& j : root["vsc_converters"]) sys.vsc_converters.push_back(vsc_from_json(j));

  if (root.contains("dcdc_converters"))
    for (const auto& j : root["dcdc_converters"]) sys.dc.dcdc_converters.push_back(dcdc_from_json(j));

  if (root.contains("energy_routers"))
    for (const auto& j : root["energy_routers"]) sys.energy_routers.push_back(energy_router_from_json(j));

  if (root.contains("mobile_storage"))
    for (const auto& j : root["mobile_storage"]) sys.mobile_storage.push_back(mobile_storage_from_json(j));

  if (root.contains("vpps"))
    for (const auto& j : root["vpps"]) sys.vpps.push_back(vpp_from_json(j));

  if (root.contains("microgrids"))
    for (const auto& j : root["microgrids"]) sys.microgrids.push_back(microgrid_from_json(j));

  if (root.contains("three_phase_ac")) {
    sys.three_phase_ac = three_phase_system_from_json(root["three_phase_ac"]);
  }

  if (root.contains("telemetry")) {
    sys.telemetry = telemetry_section_from_json(root.at("telemetry"));
  }

  repair_legacy_transformer2w_branch_links(sys);
  return sys;
}

HybridPowerSystem load_json(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("Cannot open file for reading: " + path);
  }
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
  return from_json(content);
}

// ── Schema versioning ────────────────────────────────────────────────────────

Result<bool> check_schema_version(const std::string& json_str) {
  try {
    const json j = json::parse(json_str);
    if (!j.contains("schema_version")) {
      // Legacy document without a stamp: treat as compatible (best-effort).
      return true;
    }
    const std::string ver = j.at("schema_version").get<std::string>();
    const int doc_major = std::stoi(ver);  // stoi stops at '.', e.g. "1.0" -> 1
    const int cur_major = std::stoi(std::string(kCurrentSchemaVersion));
    if (doc_major != cur_major) {
      return Error{ErrorCode::SchemaVersionMismatch,
                   "Incompatible schema version: document '" + ver +
                       "' vs supported '" +
                       std::string(kCurrentSchemaVersion) + "'",
                   {}};
    }
    return true;
  } catch (const std::exception& e) {
    return Error{ErrorCode::ParseError,
                 std::string("schema_version check failed: ") + e.what(), {}};
  }
}

// ── Exception-free safe variants ─────────────────────────────────────────────

Result<HybridPowerSystem> try_from_json(const std::string& json_str,
                                        ImportMode mode) {
  // Schema-version gate (§3.3): Strict rejects an incompatible major version;
  // Permissive proceeds best-effort so older/newer minor revisions still load.
  const auto compat = check_schema_version(json_str);
  if (!compat && mode == ImportMode::Strict) {
    return compat.error();
  }
  try {
    return from_json(json_str);
  } catch (const nlohmann::json::exception& e) {
    return Error{ErrorCode::ParseError,
                 std::string("JSON parse error: ") + e.what(), {}};
  } catch (const std::exception& e) {
    return Error{ErrorCode::ParseError,
                 std::string("Failed to parse JSON system: ") + e.what(), {}};
  }
}

Result<HybridPowerSystem> try_load_json(const std::string& path,
                                        ImportMode mode) {
  std::ifstream ifs(path);
  if (!ifs)
    return Error{ErrorCode::FileNotFound, "File not found: " + path, {}};
  try {
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
    return try_from_json(content, mode);
  } catch (const std::exception& e) {
    return Error{ErrorCode::ParseError,
                 std::string("Failed to load '") + path + "': " + e.what(), {}};
  }
}

std::string power_flow_result_to_json(const HybridPowerSystem& sys,
                                      const PowerFlowResult& result,
                                      int indent) {
  json root;
  root["converged"] = result.converged;
  root["iterations"] = result.iterations;
  root["residual"] = result.residual;
  root["warnings"] = result.diagnostics.warnings;
  root["termination_reason"] = result.diagnostics.termination_reason;

  const auto& coord = result.diagnostics.converter_coordination;
  json coord_json;
  coord_json["enabled"] = coord.enabled;
  coord_json["feasible"] = coord.feasible;
  coord_json["blocking_count"] = coord.blocking_count();
  coord_json["fatal_count"] = coord.fatal_count();
  coord_json["error_count"] = coord.error_count();
  coord_json["warning_count"] = coord.warning_count();
  coord_json["issues"] = json::array();
  for (const auto& issue : coord.issues) {
    coord_json["issues"].push_back({
        {"severity", powerflow::coordination_severity_str(issue.severity)},
        {"rule_id", issue.rule_id},
        {"component_type", issue.component_type},
        {"component_index", issue.component_index},
        {"island_index", issue.island_index},
        {"message", issue.message},
    });
  }
  coord_json["dc_islands"] = json::array();
  for (const auto& island : coord.dc_islands) {
    coord_json["dc_islands"].push_back({
        {"island_index", island.island_index},
        {"dc_buses", island.dc_buses},
        {"declared_v_buses", island.declared_v_buses},
        {"hard_vdc_sources", island.hard_vdc_sources},
        {"droop_sources", island.droop_sources},
        {"fixed_power_devices", island.fixed_power_devices},
        {"fixed_power_mw", island.fixed_power_mw},
        {"flexible_up_mw", island.flexible_up_mw},
        {"flexible_down_mw", island.flexible_down_mw},
    });
    auto& island_json = coord_json["dc_islands"].back();
    island_json["voltage_sources"] = json::array();
    for (const auto& source : island.voltage_sources) {
      island_json["voltage_sources"].push_back({
          {"component_type", source.component_type},
          {"component_index", source.component_index},
          {"bus", source.bus},
          {"v_set_pu", source.v_set_pu},
          {"has_v_set", source.has_v_set},
          {"droop", source.droop},
      });
    }
  }
  root["converter_coordination"] = std::move(coord_json);

  json ac_buses = json::array();
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const auto& bus = sys.ac.buses[i];
    ac_buses.push_back({
        {"index", bus.index},
        {"name", bus.name},
        {"vm_pu", (i < result.vm.size()) ? result.vm[i] : bus.vm_pu},
        {"va_deg", (i < result.va.size()) ? result.va[i] : bus.va_deg},
    });
  }
  root["ac_buses"] = std::move(ac_buses);

  json dc_buses = json::array();
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    const auto& bus = sys.dc.buses[i];
    dc_buses.push_back({
        {"index", bus.index},
        {"name", bus.name},
        {"vm_pu", (i < result.vdc.size()) ? result.vdc[i] : bus.vm_pu},
    });
  }
  root["dc_buses"] = std::move(dc_buses);

  json ac_flows = json::array();
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& br = sys.ac.branches[i];
    double pf_mw = 0.0;
    double qf_mvar = 0.0;
    double pt_mw = 0.0;
    double qt_mvar = 0.0;
    if (i < result.branch_flows.size()) {
      pf_mw = result.branch_flows[i].pf_mw;
      qf_mvar = result.branch_flows[i].qf_mvar;
      pt_mw = result.branch_flows[i].pt_mw;
      qt_mvar = result.branch_flows[i].qt_mvar;
    }
    const double sf = std::sqrt(pf_mw * pf_mw + qf_mvar * qf_mvar);
    const double st = std::sqrt(pt_mw * pt_mw + qt_mvar * qt_mvar);
    const double rating = pf_ac_branch_rating_mva(sys, br);
    const double loading_pct =
        (rating > 1e-12) ? (100.0 * std::max(sf, st) / rating) : 0.0;
    ac_flows.push_back({
        {"index", br.index},
        {"name", br.name},
        {"from_bus", br.from_bus},
        {"to_bus", br.to_bus},
        {"pf_mw", pf_mw},
        {"qf_mvar", qf_mvar},
        {"pt_mw", pt_mw},
        {"qt_mvar", qt_mvar},
        {"loss_mw", std::max(pf_mw + pt_mw, 0.0)},
        {"loading_pct", loading_pct},
    });
  }
  root["ac_branch_flows"] = std::move(ac_flows);

  const auto dc_estimates = pf_estimate_dc_branch_flows(sys, result);
  json dc_flows = json::array();
  for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
    const auto& br = sys.dc.branches[i];
    const PFDCBranchEstimate est =
        (i < dc_estimates.size()) ? dc_estimates[i] : PFDCBranchEstimate{};
    dc_flows.push_back({
        {"index", br.index},
        {"name", br.name},
        {"from_bus", br.from_bus},
        {"to_bus", br.to_bus},
        {"pf_mw", est.p_from_mw},
        {"pt_mw", est.p_to_mw},
        {"loss_mw", est.loss_mw},
        {"loading_pct", est.loading_pct},
    });
  }
  root["dc_branch_flows"] = std::move(dc_flows);

  json vsc_transfers = json::array();
  for (const auto& transfer : result.vsc_transfers) {
    vsc_transfers.push_back({
        {"index", transfer.index},
        {"bus_ac", transfer.bus_ac},
        {"bus_dc", transfer.bus_dc},
        {"p_ac_mw", transfer.p_ac_mw},
        {"q_ac_mvar", transfer.q_ac_mvar},
        {"p_dc_mw", transfer.p_dc_mw},
        {"loss_mw", std::max(transfer.loss_mw, 0.0)},
    });
  }
  root["vsc_transfers"] = std::move(vsc_transfers);

  json dcdc_transfers = json::array();
  for (const auto& transfer : result.dcdc_transfers) {
    dcdc_transfers.push_back({
        {"index", transfer.index},
        {"bus_in", transfer.bus_in},
        {"bus_out", transfer.bus_out},
        {"p_in_mw", transfer.p_in_mw},
        {"p_out_mw", transfer.p_out_mw},
        {"loss_mw", std::max(transfer.loss_mw, 0.0)},
    });
  }
  root["dcdc_transfers"] = std::move(dcdc_transfers);

  return root.dump(indent);
}

void save_power_flow_result_json(const HybridPowerSystem& sys,
                                 const PowerFlowResult& result,
                                 const std::string& path,
                                 int indent) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Cannot open file for writing: " + path);
  }
  ofs << power_flow_result_to_json(sys, result, indent);
}

// ═══════════════════════════════════════════════════════════════════════
// OPF Result JSON serialization / deserialization
// ═══════════════════════════════════════════════════════════════════════

std::string opf_result_to_json(const opf::ACOPFResult& result, int indent) {
  json j;
  j["converged"] = result.converged;
  j["iterations"] = result.iterations;
  j["outer_iterations"] = result.outer_iterations;
  j["objective"] = result.objective;
  j["max_constraint_violation"] = result.max_constraint_violation;
  j["max_stationarity"] = result.max_stationarity;
  j["status"] = result.status;

  // Solver path metadata
  switch (result.solver_path) {
    case opf::OPFSolverPath::NativeAC:  j["solver_path"] = "NativeAC"; break;
    case opf::OPFSolverPath::ParityIPM: j["solver_path"] = "ParityIPM"; break;
    default:                            j["solver_path"] = "Unknown"; break;
  }

  j["vm"] = result.vm;
  j["va"] = result.va;
  j["vdc"] = result.vdc;
  j["pg_mw"] = result.pg_mw;
  j["qg_mvar"] = result.qg_mvar;
  j["dpd_mw"] = result.dpd_mw;
  j["dqd_mvar"] = result.dqd_mvar;
  j["pac_mw"] = result.pac_mw;
  j["qac_mvar"] = result.qac_mvar;

  // Enhanced component dispatch vectors with identity metadata.
  // Each category is serialized as an object with "values" (the dispatch
  // array) and "map" (an array of {original_index, source_type} objects
  // that let consumers join results back to the original model tables).
  auto serialize_component_map = [](const std::vector<opf::ACOPFResult::ComponentRef>& map) {
    json arr = json::array();
    for (const auto& ref : map)
      arr.push_back({{"original_index", ref.original_index},
                     {"source_type", ref.source_type}});
    return arr;
  };

  {
    json ren;
    ren["pren_mw"]   = result.pren_mw;
    ren["qren_mvar"] = result.qren_mvar;
    ren["map"]        = serialize_component_map(result.ren_map);
    j["renewable_dispatch"] = ren;
  }
  {
    json stor;
    stor["pstor_mw"]   = result.pstor_mw;
    stor["qstor_mvar"] = result.qstor_mvar;
    stor["map"]         = serialize_component_map(result.stor_map);
    j["storage_dispatch"] = stor;
  }
  {
    json dcdc;
    dcdc["pdcdc_mw"] = result.pdcdc_mw;
    dcdc["map"]       = serialize_component_map(result.dcdc_map);
    j["dcdc_dispatch"] = dcdc;
  }
  {
    json flex;
    flex["pflex_mw"] = result.pflex_mw;
    flex["map"]       = serialize_component_map(result.flex_map);
    j["flex_dispatch"] = flex;
  }

  // Legacy bare arrays (kept for backward compatibility with existing readers)
  j["pren_mw"]   = result.pren_mw;
  j["qren_mvar"] = result.qren_mvar;
  j["pstor_mw"]  = result.pstor_mw;
  j["qstor_mvar"]= result.qstor_mvar;
  j["pdcdc_mw"]  = result.pdcdc_mw;
  j["pflex_mw"]  = result.pflex_mw;

  json prof;
  prof["linear_solver_backend"] = result.profiling.linear_solver_backend;
  prof["analyze_calls"] = result.profiling.analyze_calls;
  prof["factorization_calls"] = result.profiling.factorization_calls;
  prof["linear_solve_calls"] = result.profiling.linear_solve_calls;
  prof["total_iterations"] = result.profiling.total_iterations;
  prof["accepted_steps"] = result.profiling.accepted_steps;
  prof["rejected_steps"] = result.profiling.rejected_steps;
  prof["final_barrier_mu"] = result.profiling.final_barrier_mu;
  j["profiling"] = prof;

  return j.dump(indent);
}

std::string dc_opf_result_to_json(const opf::DCOPFResult& result, int indent) {
  json j;
  j["converged"]               = result.converged;
  j["iterations"]              = result.iterations;
  j["objective"]               = result.objective;
  j["status"]                  = result.status;
  j["solver_name"]             = result.solver_name;
  j["runtime_sec"]             = result.runtime_sec;
  j["total_load_shedding_mw"]  = result.total_load_shedding_mw;

  // Solver-path audit fields — the primary new data that callers care about.
  j["solver_chain"]   = result.solver_chain;
  j["objective_model"] = result.objective_model;

  j["lmp"]                = result.lmp;
  j["branch_mu_lower"]    = result.branch_mu_lower;
  j["branch_mu_upper"]    = result.branch_mu_upper;
  j["branch_mu_valid"]    = result.branch_mu_valid;
  j["load_shedding_mw"]   = result.load_shedding_mw;
  j["pg_mw"]              = result.pg_mw;
  j["pf_mw"]              = result.pf_mw;
  j["va"]                 = result.va;
  return j.dump(indent);
}

opf::DCOPFResult dc_opf_result_from_json(const std::string& json_str) {
  const json j = json::parse(json_str);
  opf::DCOPFResult r;

  r.converged = jget(j, "converged", false);
  r.iterations = jget(j, "iterations", 0);
  r.objective = jget(j, "objective", 0.0);
  r.status = jget<std::string>(j, "status", "");
  r.solver_name = jget<std::string>(j, "solver_name", "");
  r.runtime_sec = jget(j, "runtime_sec", 0.0);
  r.total_load_shedding_mw = jget(j, "total_load_shedding_mw", 0.0);
  r.branch_mu_valid = jget(j, "branch_mu_valid", false);
  r.objective_model = jget<std::string>(j, "objective_model", "");

  if (j.contains("solver_chain")) r.solver_chain = j["solver_chain"].get<std::vector<std::string>>();
  if (j.contains("lmp")) r.lmp = j["lmp"].get<std::vector<double>>();
  if (j.contains("branch_mu_lower")) r.branch_mu_lower = j["branch_mu_lower"].get<std::vector<double>>();
  if (j.contains("branch_mu_upper")) r.branch_mu_upper = j["branch_mu_upper"].get<std::vector<double>>();
  if (j.contains("load_shedding_mw")) r.load_shedding_mw = j["load_shedding_mw"].get<std::vector<double>>();
  if (j.contains("pg_mw")) r.pg_mw = j["pg_mw"].get<std::vector<double>>();
  if (j.contains("pf_mw")) r.pf_mw = j["pf_mw"].get<std::vector<double>>();
  if (j.contains("va")) r.va = j["va"].get<std::vector<double>>();
  return r;
}

void save_dc_opf_result_json(const opf::DCOPFResult& result,
                             const std::string& path, int indent) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Cannot open file for writing: " + path);
  }
  ofs << dc_opf_result_to_json(result, indent);
}

opf::DCOPFResult load_dc_opf_result_json(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("Cannot open file for reading: " + path);
  }
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  return dc_opf_result_from_json(buffer.str());
}

void save_opf_result_json(const opf::ACOPFResult& result,
                          const std::string& path, int indent) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Cannot open file for writing: " + path);
  }
  ofs << opf_result_to_json(result, indent);
}

opf::ACOPFResult opf_result_from_json(const std::string& json_str) {
  const json j = json::parse(json_str);
  opf::ACOPFResult r;

  r.converged = jget(j, "converged", false);
  r.iterations = jget(j, "iterations", 0);
  r.outer_iterations = jget(j, "outer_iterations", 0);
  r.objective = jget(j, "objective", 0.0);
  r.max_constraint_violation = jget(j, "max_constraint_violation", 0.0);
  r.max_stationarity = jget(j, "max_stationarity", 0.0);
  r.status = jget<std::string>(j, "status", "");

  // Solver path metadata
  if (j.contains("solver_path")) {
    const std::string sp = j["solver_path"].get<std::string>();
    if (sp == "NativeAC")       r.solver_path = opf::OPFSolverPath::NativeAC;
    else if (sp == "ParityIPM") r.solver_path = opf::OPFSolverPath::ParityIPM;
    else                        r.solver_path = opf::OPFSolverPath::Unknown;
  }

  if (j.contains("vm")) r.vm = j["vm"].get<std::vector<double>>();
  if (j.contains("va")) r.va = j["va"].get<std::vector<double>>();
  if (j.contains("vdc")) r.vdc = j["vdc"].get<std::vector<double>>();
  if (j.contains("pg_mw")) r.pg_mw = j["pg_mw"].get<std::vector<double>>();
  if (j.contains("qg_mvar")) r.qg_mvar = j["qg_mvar"].get<std::vector<double>>();
  if (j.contains("dpd_mw")) r.dpd_mw = j["dpd_mw"].get<std::vector<double>>();
  if (j.contains("dqd_mvar")) r.dqd_mvar = j["dqd_mvar"].get<std::vector<double>>();
  if (j.contains("pac_mw")) r.pac_mw = j["pac_mw"].get<std::vector<double>>();
  if (j.contains("qac_mvar")) r.qac_mvar = j["qac_mvar"].get<std::vector<double>>();

  // Enhanced component dispatch vectors
  if (j.contains("pren_mw")) r.pren_mw = j["pren_mw"].get<std::vector<double>>();
  if (j.contains("qren_mvar")) r.qren_mvar = j["qren_mvar"].get<std::vector<double>>();
  if (j.contains("pstor_mw")) r.pstor_mw = j["pstor_mw"].get<std::vector<double>>();
  if (j.contains("qstor_mvar")) r.qstor_mvar = j["qstor_mvar"].get<std::vector<double>>();
  if (j.contains("pdcdc_mw")) r.pdcdc_mw = j["pdcdc_mw"].get<std::vector<double>>();
  if (j.contains("pflex_mw")) r.pflex_mw = j["pflex_mw"].get<std::vector<double>>();

  // Structured dispatch objects (preferred path — supersedes bare arrays)
  auto deserialize_map = [](const json& arr) {
    std::vector<opf::ACOPFResult::ComponentRef> out;
    for (const auto& item : arr) {
      opf::ACOPFResult::ComponentRef ref;
      ref.original_index = item.value("original_index", 0);
      ref.source_type    = item.value("source_type", 0);
      out.push_back(ref);
    }
    return out;
  };
  if (j.contains("renewable_dispatch")) {
    const auto& rd = j["renewable_dispatch"];
    if (rd.contains("pren_mw"))   r.pren_mw   = rd["pren_mw"].get<std::vector<double>>();
    if (rd.contains("qren_mvar")) r.qren_mvar = rd["qren_mvar"].get<std::vector<double>>();
    if (rd.contains("map"))       r.ren_map   = deserialize_map(rd["map"]);
  }
  if (j.contains("storage_dispatch")) {
    const auto& sd = j["storage_dispatch"];
    if (sd.contains("pstor_mw"))   r.pstor_mw   = sd["pstor_mw"].get<std::vector<double>>();
    if (sd.contains("qstor_mvar")) r.qstor_mvar = sd["qstor_mvar"].get<std::vector<double>>();
    if (sd.contains("map"))        r.stor_map   = deserialize_map(sd["map"]);
  }
  if (j.contains("dcdc_dispatch")) {
    const auto& dd = j["dcdc_dispatch"];
    if (dd.contains("pdcdc_mw")) r.pdcdc_mw = dd["pdcdc_mw"].get<std::vector<double>>();
    if (dd.contains("map"))      r.dcdc_map = deserialize_map(dd["map"]);
  }
  if (j.contains("flex_dispatch")) {
    const auto& fd = j["flex_dispatch"];
    if (fd.contains("pflex_mw")) r.pflex_mw = fd["pflex_mw"].get<std::vector<double>>();
    if (fd.contains("map"))      r.flex_map = deserialize_map(fd["map"]);
  }

  if (j.contains("profiling")) {
    const auto& p = j["profiling"];
    r.profiling.linear_solver_backend = jget<std::string>(p, "linear_solver_backend", "");
    r.profiling.analyze_calls = jget(p, "analyze_calls", 0);
    r.profiling.factorization_calls = jget(p, "factorization_calls", 0);
    r.profiling.linear_solve_calls = jget(p, "linear_solve_calls", 0);
    r.profiling.total_iterations = jget(p, "total_iterations", 0);
    r.profiling.accepted_steps = jget(p, "accepted_steps", 0);
    r.profiling.rejected_steps = jget(p, "rejected_steps", 0);
    r.profiling.final_barrier_mu = jget(p, "final_barrier_mu", 0.0);
  }

  return r;
}

opf::ACOPFResult load_opf_result_json(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("Cannot open file for reading: " + path);
  }
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
  return opf_result_from_json(content);
}

// ═══════════════════════════════════════════════════════════════════════
// Carbon Analysis Result JSON serialization / deserialization
// ═══════════════════════════════════════════════════════════════════════

static json emissions_summary_to_json(const analysis::EmissionsSummary& s) {
  json j;
  j["total_generation_emissions_tco2"] = s.total_generation_emissions_tco2;
  j["total_load_emissions_tco2"]       = s.total_load_emissions_tco2;
  j["total_loss_emissions_tco2"]       = s.total_loss_emissions_tco2;
  j["balance_error_tco2"]             = s.balance_error_tco2;
  j["balance_error_pct"]              = s.balance_error_pct;
  return j;
}

static analysis::EmissionsSummary emissions_summary_from_json(const json& j) {
  analysis::EmissionsSummary s;
  s.total_generation_emissions_tco2 = jget(j, "total_generation_emissions_tco2", 0.0);
  s.total_load_emissions_tco2       = jget(j, "total_load_emissions_tco2", 0.0);
  s.total_loss_emissions_tco2       = jget(j, "total_loss_emissions_tco2", 0.0);
  s.balance_error_tco2             = jget(j, "balance_error_tco2", 0.0);
  s.balance_error_pct              = jget(j, "balance_error_pct", 0.0);
  return s;
}

std::string carbon_result_to_json(const analysis::CarbonAnalysisResult& result,
                                  int indent) {
  json j;
  j["tracing_verified"] = result.tracing_verified;
  j["matrix_solved"]    = result.matrix_solved;
  j["matrix_residual"]  = result.matrix_residual;

  auto load_results_to_json = [](const std::vector<analysis::LoadCarbonResult>& items) {
    json loads = json::array();
    for (const auto& lc : items) {
      json lj;
      lj["load_index"] = lc.load_index;
      lj["bus"] = lc.bus;
      lj["demand_mw"] = lc.demand_mw;
      lj["carbon_intensity_tco2_mwh"] = lc.carbon_intensity_tco2_mwh;
      lj["total_emissions_tco2"] = lc.total_emissions_tco2;
      json gen_supply;
      for (const auto& [sid, mw] : lc.generator_supply_mw)
        gen_supply[std::to_string(sid)] = mw;
      lj["generator_supply_mw"] = gen_supply;
      loads.push_back(lj);
    }
    return loads;
  };

  auto branch_results_to_json = [](const std::vector<analysis::BranchCarbonResult>& items) {
    json branches = json::array();
    for (const auto& bc : items) {
      json bj;
      bj["branch_index"] = bc.branch_index;
      bj["from_bus"] = bc.from_bus;
      bj["to_bus"] = bc.to_bus;
      bj["loss_mw"] = bc.loss_mw;
      bj["carbon_intensity_tco2_mwh"] = bc.carbon_intensity_tco2_mwh;
      bj["total_emissions_tco2"] = bc.total_emissions_tco2;
      json gen_loss;
      for (const auto& [sid, mw] : bc.generator_loss_mw)
        gen_loss[std::to_string(sid)] = mw;
      bj["generator_loss_mw"] = gen_loss;
      branches.push_back(bj);
    }
    return branches;
  };

  auto bus_results_to_json = [](const std::vector<analysis::BusCarbonResult>& items) {
    json buses = json::array();
    for (const auto& bc : items) {
      json bj;
      bj["bus_index"] = bc.bus_index;
      bj["carbon_intensity_tco2_mwh"] = bc.carbon_intensity_tco2_mwh;
      buses.push_back(bj);
    }
    return buses;
  };

  j["load_carbon"] = load_results_to_json(result.load_carbon);
  j["dc_load_carbon"] = load_results_to_json(result.dc_load_carbon);
  j["branch_carbon"] = branch_results_to_json(result.branch_carbon);
  j["dc_branch_carbon"] = branch_results_to_json(result.dc_branch_carbon);
  j["bus_carbon"] = bus_results_to_json(result.bus_carbon);
  j["dc_bus_carbon"] = bus_results_to_json(result.dc_bus_carbon);

  json vsc = json::array();
  for (const auto& vc : result.vsc_carbon) {
    json lj;
    lj["converter_index"] = vc.converter_index;
    lj["bus_ac"] = vc.bus_ac;
    lj["bus_dc"] = vc.bus_dc;
    lj["ac_to_dc"] = vc.ac_to_dc;
    lj["input_power_mw"] = vc.input_power_mw;
    lj["output_power_mw"] = vc.output_power_mw;
    lj["loss_mw"] = vc.loss_mw;
    lj["carbon_intensity_tco2_mwh"] = vc.carbon_intensity_tco2_mwh;
    lj["total_emissions_tco2"] = vc.total_emissions_tco2;
    json gen_loss;
    for (const auto& [sid, mw] : vc.generator_loss_mw)
      gen_loss[std::to_string(sid)] = mw;
    lj["generator_loss_mw"] = gen_loss;
    vsc.push_back(lj);
  }
  j["vsc_carbon"] = vsc;

  json dcdc = json::array();
  for (const auto& dc : result.dcdc_carbon) {
    json dj;
    dj["converter_index"] = dc.converter_index;
    dj["bus_in"] = dc.bus_in;
    dj["bus_out"] = dc.bus_out;
    dj["input_to_output"] = dc.input_to_output;
    dj["input_power_mw"] = dc.input_power_mw;
    dj["output_power_mw"] = dc.output_power_mw;
    dj["loss_mw"] = dc.loss_mw;
    dj["carbon_intensity_tco2_mwh"] = dc.carbon_intensity_tco2_mwh;
    dj["total_emissions_tco2"] = dc.total_emissions_tco2;
    json gen_loss;
    for (const auto& [sid, mw] : dc.generator_loss_mw)
      gen_loss[std::to_string(sid)] = mw;
    dj["generator_loss_mw"] = gen_loss;
    dcdc.push_back(dj);
  }
  j["dcdc_carbon"] = dcdc;

  json storage = json::array();
  for (const auto& sc : result.storage_carbon) {
    json sj;
    sj["storage_index"] = sc.storage_index;
    sj["bus"] = sc.bus;
    sj["is_dc"] = sc.is_dc;
    sj["p_mw"] = sc.p_mw;
    sj["soc"] = sc.soc;
    sj["stored_energy_mwh"] = sc.stored_energy_mwh;
    sj["soc_carbon_intensity_tco2_mwh"] = sc.soc_carbon_intensity_tco2_mwh;
    sj["carbon_intensity_tco2_mwh"] = sc.carbon_intensity_tco2_mwh;
    sj["total_emissions_tco2"] = sc.total_emissions_tco2;
    json supply;
    for (const auto& [sid, mw] : sc.source_supply_mw)
      supply[std::to_string(sid)] = mw;
    sj["source_supply_mw"] = supply;
    storage.push_back(sj);
  }
  j["storage_carbon"] = storage;

  json energy_router = json::array();
  for (const auto& rc : result.energy_router_carbon) {
    json rj;
    rj["router_index"] = rc.router_index;
    rj["input_power_mw"] = rc.input_power_mw;
    rj["output_power_mw"] = rc.output_power_mw;
    rj["loss_mw"] = rc.loss_mw;
    rj["active_input_ports"] = rc.active_input_ports;
    rj["active_output_ports"] = rc.active_output_ports;
    rj["carbon_intensity_tco2_mwh"] = rc.carbon_intensity_tco2_mwh;
    rj["total_emissions_tco2"] = rc.total_emissions_tco2;
    json loss_map;
    for (const auto& [sid, mw] : rc.source_loss_mw)
      loss_map[std::to_string(sid)] = mw;
    rj["source_loss_mw"] = loss_map;
    energy_router.push_back(rj);
  }
  j["energy_router_carbon"] = energy_router;

  // Generator loss allocation
  json gen_loss;
  for (const auto& [sid, mw] : result.generator_loss_allocation)
    gen_loss[std::to_string(sid)] = mw;
  j["generator_loss_allocation"] = gen_loss;

  j["tracing_summary"] = emissions_summary_to_json(result.tracing_summary);
  j["matrix_summary"]  = emissions_summary_to_json(result.matrix_summary);

  return j.dump(indent);
}

void save_carbon_result_json(const analysis::CarbonAnalysisResult& result,
                             const std::string& path, int indent) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Cannot open file for writing: " + path);
  }
  ofs << carbon_result_to_json(result, indent);
}

analysis::CarbonAnalysisResult carbon_result_from_json(const std::string& json_str) {
  const json j = json::parse(json_str);
  analysis::CarbonAnalysisResult r;

  r.tracing_verified = jget(j, "tracing_verified", false);
  r.matrix_solved    = jget(j, "matrix_solved", false);
  r.matrix_residual  = jget(j, "matrix_residual", 0.0);

  auto parse_load_results = [](const json& arr, std::vector<analysis::LoadCarbonResult>& out) {
    for (const auto& lj : arr) {
      analysis::LoadCarbonResult lc;
      lc.load_index = jget(lj, "load_index", 0);
      lc.bus = jget(lj, "bus", 0);
      lc.demand_mw = jget(lj, "demand_mw", 0.0);
      lc.carbon_intensity_tco2_mwh = jget(lj, "carbon_intensity_tco2_mwh", 0.0);
      lc.total_emissions_tco2 = jget(lj, "total_emissions_tco2", 0.0);
      if (lj.contains("generator_supply_mw")) {
        for (auto& [k, v] : lj["generator_supply_mw"].items())
          lc.generator_supply_mw[std::stoi(k)] = v.get<double>();
      }
      out.push_back(std::move(lc));
    }
  };

  auto parse_branch_results = [](const json& arr, std::vector<analysis::BranchCarbonResult>& out) {
    for (const auto& bj : arr) {
      analysis::BranchCarbonResult bc;
      bc.branch_index = jget(bj, "branch_index", 0);
      bc.from_bus = jget(bj, "from_bus", 0);
      bc.to_bus = jget(bj, "to_bus", 0);
      bc.loss_mw = jget(bj, "loss_mw", 0.0);
      bc.carbon_intensity_tco2_mwh = jget(bj, "carbon_intensity_tco2_mwh", 0.0);
      bc.total_emissions_tco2 = jget(bj, "total_emissions_tco2", 0.0);
      if (bj.contains("generator_loss_mw")) {
        for (auto& [k, v] : bj["generator_loss_mw"].items())
          bc.generator_loss_mw[std::stoi(k)] = v.get<double>();
      }
      out.push_back(std::move(bc));
    }
  };

  auto parse_bus_results = [](const json& arr, std::vector<analysis::BusCarbonResult>& out) {
    for (const auto& bj : arr) {
      analysis::BusCarbonResult bc;
      bc.bus_index = jget(bj, "bus_index", 0);
      bc.carbon_intensity_tco2_mwh = jget(bj, "carbon_intensity_tco2_mwh", 0.0);
      out.push_back(bc);
    }
  };

  if (j.contains("load_carbon")) {
    parse_load_results(j["load_carbon"], r.load_carbon);
  }
  if (j.contains("dc_load_carbon")) {
    parse_load_results(j["dc_load_carbon"], r.dc_load_carbon);
  }

  if (j.contains("branch_carbon")) {
    parse_branch_results(j["branch_carbon"], r.branch_carbon);
  }
  if (j.contains("dc_branch_carbon")) {
    parse_branch_results(j["dc_branch_carbon"], r.dc_branch_carbon);
  }

  if (j.contains("bus_carbon")) {
    parse_bus_results(j["bus_carbon"], r.bus_carbon);
  }
  if (j.contains("dc_bus_carbon")) {
    parse_bus_results(j["dc_bus_carbon"], r.dc_bus_carbon);
  }

  if (j.contains("vsc_carbon")) {
    for (const auto& vj : j["vsc_carbon"]) {
      analysis::VSCCarbonResult vc;
      vc.converter_index = jget(vj, "converter_index", 0);
      vc.bus_ac = jget(vj, "bus_ac", 0);
      vc.bus_dc = jget(vj, "bus_dc", 0);
      vc.ac_to_dc = jget(vj, "ac_to_dc", false);
      vc.input_power_mw = jget(vj, "input_power_mw", 0.0);
      vc.output_power_mw = jget(vj, "output_power_mw", 0.0);
      vc.loss_mw = jget(vj, "loss_mw", 0.0);
      vc.carbon_intensity_tco2_mwh = jget(vj, "carbon_intensity_tco2_mwh", 0.0);
      vc.total_emissions_tco2 = jget(vj, "total_emissions_tco2", 0.0);
      if (vj.contains("generator_loss_mw")) {
        for (auto& [k, v] : vj["generator_loss_mw"].items())
          vc.generator_loss_mw[std::stoi(k)] = v.get<double>();
      }
      r.vsc_carbon.push_back(std::move(vc));
    }
  }

  if (j.contains("dcdc_carbon")) {
    for (const auto& dj : j["dcdc_carbon"]) {
      analysis::DCDCCarbonResult dc;
      dc.converter_index = jget(dj, "converter_index", 0);
      dc.bus_in = jget(dj, "bus_in", 0);
      dc.bus_out = jget(dj, "bus_out", 0);
      dc.input_to_output = jget(dj, "input_to_output", false);
      dc.input_power_mw = jget(dj, "input_power_mw", 0.0);
      dc.output_power_mw = jget(dj, "output_power_mw", 0.0);
      dc.loss_mw = jget(dj, "loss_mw", 0.0);
      dc.carbon_intensity_tco2_mwh = jget(dj, "carbon_intensity_tco2_mwh", 0.0);
      dc.total_emissions_tco2 = jget(dj, "total_emissions_tco2", 0.0);
      if (dj.contains("generator_loss_mw")) {
        for (auto& [k, v] : dj["generator_loss_mw"].items())
          dc.generator_loss_mw[std::stoi(k)] = v.get<double>();
      }
      r.dcdc_carbon.push_back(std::move(dc));
    }
  }

  if (j.contains("storage_carbon")) {
    for (const auto& sj : j["storage_carbon"]) {
      analysis::StorageCarbonResult sc;
      sc.storage_index = jget(sj, "storage_index", 0);
      sc.bus = jget(sj, "bus", 0);
      sc.is_dc = jget(sj, "is_dc", false);
      sc.p_mw = jget(sj, "p_mw", 0.0);
      sc.soc = jget(sj, "soc", 0.0);
      sc.stored_energy_mwh = jget(sj, "stored_energy_mwh", 0.0);
      sc.soc_carbon_intensity_tco2_mwh =
          jget(sj, "soc_carbon_intensity_tco2_mwh", 0.0);
      sc.carbon_intensity_tco2_mwh = jget(sj, "carbon_intensity_tco2_mwh", 0.0);
      sc.total_emissions_tco2 = jget(sj, "total_emissions_tco2", 0.0);
      if (sj.contains("source_supply_mw")) {
        for (auto& [k, v] : sj["source_supply_mw"].items())
          sc.source_supply_mw[std::stoi(k)] = v.get<double>();
      }
      r.storage_carbon.push_back(std::move(sc));
    }
  }

  if (j.contains("energy_router_carbon")) {
    for (const auto& rj : j["energy_router_carbon"]) {
      analysis::EnergyRouterCarbonResult rc;
      rc.router_index = jget(rj, "router_index", 0);
      rc.input_power_mw = jget(rj, "input_power_mw", 0.0);
      rc.output_power_mw = jget(rj, "output_power_mw", 0.0);
      rc.loss_mw = jget(rj, "loss_mw", 0.0);
      rc.active_input_ports = jget(rj, "active_input_ports", 0);
      rc.active_output_ports = jget(rj, "active_output_ports", 0);
      rc.carbon_intensity_tco2_mwh = jget(rj, "carbon_intensity_tco2_mwh", 0.0);
      rc.total_emissions_tco2 = jget(rj, "total_emissions_tco2", 0.0);
      if (rj.contains("source_loss_mw")) {
        for (auto& [k, v] : rj["source_loss_mw"].items())
          rc.source_loss_mw[std::stoi(k)] = v.get<double>();
      }
      r.energy_router_carbon.push_back(std::move(rc));
    }
  }

  if (j.contains("generator_loss_allocation")) {
    for (auto& [k, v] : j["generator_loss_allocation"].items())
      r.generator_loss_allocation[std::stoi(k)] = v.get<double>();
  }

  if (j.contains("tracing_summary"))
    r.tracing_summary = emissions_summary_from_json(j["tracing_summary"]);
  if (j.contains("matrix_summary"))
    r.matrix_summary = emissions_summary_from_json(j["matrix_summary"]);

  return r;
}

analysis::CarbonAnalysisResult load_carbon_result_json(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("Cannot open file for reading: " + path);
  }
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
  return carbon_result_from_json(content);
}

// ═══════════════════════════════════════════════════════════════════════
// Time-Series Data JSON serialization / deserialization
// ═══════════════════════════════════════════════════════════════════════

std::string time_series_data_to_json(const TimeSeriesData& ts, int indent) {
  json root;
  root["num_steps"] = ts.num_steps;
  root["step_duration_hr"] = ts.step_duration_hr;
  root["profiles"] = json::array();
  for (const auto& p : ts.profiles) {
    json jp;
    jp["id"] = p.id;
    jp["name"] = p.name;
    jp["values"] = p.values;
    root["profiles"].push_back(jp);
  }
  return root.dump(indent);
}

TimeSeriesData time_series_data_from_json(const std::string& json_str) {
  json root = json::parse(json_str);
  TimeSeriesData ts;
  ts.num_steps = jget(root, "num_steps", 24);
  ts.step_duration_hr = jget(root, "step_duration_hr", 1.0);
  if (root.contains("profiles")) {
    for (const auto& jp : root["profiles"]) {
      TimeSeriesProfile p;
      p.id = jp.at("id").get<int>();
      p.name = jget<std::string>(jp, "name", "");
      p.values = jp.at("values").get<std::vector<double>>();
      ts.profiles.push_back(std::move(p));
    }
  }
  return ts;
}

// ═══════════════════════════════════════════════════════════════════════
// JPC matrix-case format serialization / deserialization
// ═══════════════════════════════════════════════════════════════════════

// Column indices for JPC matrices
namespace jpc_idx {
// Bus columns (idx_bus)
enum BusIdx {
  BUS_I = 0, BUS_TYPE, PD, QD, GS, BS, BUS_AREA, VM, VA, BASE_KV, ZONE,
  VMAX, VMIN, CARBON_AREA, CARBON_ZONE, LAM_P, LAM_Q, MU_VMAX, MU_VMIN,
  PER_CONSUMER, LATITUDE, LONGITUDE, N_BUS_COLS = 22
};

// Branch columns (idx_brch)
enum BranchIdx {
  F_BUS = 0, T_BUS, BR_R, BR_X, BR_B, RATE_A, RATE_B, RATE_C, TAP, SHIFT,
  BR_STATUS, ANGMIN, ANGMAX, DICTKEY, MAX_I, SN_MVA, PF, QF, PT, QT,
  MU_SF, MU_ST, MU_ANGMIN, MU_ANGMAX, LAMBDA, SW_TIME, RP_TIME, BR_TYPE,
  N_BRANCH_COLS = 28
};

// Generator columns (idx_gen)
enum GenIdx {
  GEN_BUS = 0, PG, QG, QMAX, QMIN, VG, MBASE, GEN_STATUS, PMAX, PMIN,
  PC1, PC2, QC1MIN, QC1MAX, QC2MIN, QC2MAX, RAMP_AGC, RAMP_10, RAMP_30,
  RAMP_Q, APF, MODEL, STARTUP, SHUTDOWN, NCOST, COST, CARBON_EMISSION,
  MU_PMAX, MU_PMIN, MU_QMAX, MU_QMIN, GEN_AREA,
  // COST1 and COST2 are appended after the standard 32 columns so that
  // existing JPC files (32 cols) parse correctly with c1/c0 defaulting to 0.
  COST1, COST2,
  N_GEN_COLS
};

// Load columns (idx_ld)
enum LoadIdx {
  LOAD_I = 0, LOAD_CND, LOAD_STATUS, LOAD_PD, LOAD_QD,
  LOADZ_PERCENT, LOADI_PERCENT, LOADP_PERCENT, N_LOAD_COLS = 8
};

// DC Bus columns (idx_dcbus)
enum DCBusIdx {
  DC_BUS_I = 0, DC_BUS_TYPE, DC_PD, DC_QD, DC_GS, DC_BS, DC_BUS_AREA,
  DC_VM, DC_VA, DC_BASE_KV, DC_ZONE, DC_VMAX, DC_VMIN,
  DC_CARBON_AREA, DC_CARBON_ZONE, DC_LAM_P, DC_LAM_Q, DC_MU_VMAX, DC_MU_VMIN,
  DC_PER_CONSUMER, DC_LATITUDE, N_DCBUS_COLS = 21
};

// Storage columns (idx_ess)
enum ESSIdx {
  ESS_BUS = 0, ESS_POWER_CAPACITY, ESS_ENERGY_CAPACITY, ESS_SOC_INIT,
  ESS_SOC_MIN, ESS_SOC_MAX, ESS_EFFICIENCY, ESS_STATUS, ESS_P_DISCHARGE,
  ESS_P_CHARGE, ESS_DISCHARGE_COST, ESS_CHARGE_COST, ESS_AREA, ESS_ZONE,
  ESS_PROFILE_ID, N_ESS_COLS = 15
};

// Converter columns (idx_conv)
enum ConvIdx {
  CONV_ACBUS = 0, CONV_DCBUS, CONV_INSERVICE, CONV_P_AC, CONV_Q_AC, CONV_P_DC,
  CONV_EFF, CONV_MODE, CONV_DROOP_KP, CONV_PMAX, CONV_PMIN, CONV_QMAX, CONV_QMIN,
  CONV_LOSS_A, CONV_LOSS_B, CONV_AREA, CONV_GRID_FORMING, CONV_CONTROL_STRATEGY,
  N_CONV_COLS = 18
};

// External grid columns (idx_ext_grid)
enum ExtGridIdx {
  EXT_GRID_BUS = 0, EXT_VM_PU, EXT_VA_DEG, EXT_PMAX, EXT_PMIN, EXT_QMAX, EXT_QMIN,
  EXT_STATUS, EXT_SC_MVA_3PH, EXT_SC_MVA_1PH, EXT_RX_RATIO, EXT_X0X1_RATIO, EXT_R0X0_RATIO,
  N_EXT_GRID_COLS = 13
};
}  // namespace jpc_idx

// Helper: Convert std::vector<double> to JSON array
static json row_to_json_array(const std::vector<double>& row) {
  return json(row);
}

// Helper: Bus type to JPC integer
static int bus_type_to_jpc(BusType bt) {
  switch (bt) {
    case BusType::PQ:       return 1;
    case BusType::PV:       return 2;
    case BusType::SLACK:    return 3;
    case BusType::ISOLATED: return 4;
    default:                return 1;  // PQ default
  }
}

// Helper: JPC integer to bus type
static BusType jpc_to_bus_type(int bt) {
  switch (bt) {
    case 1:  return BusType::PQ;
    case 2:  return BusType::PV;
    case 3:  return BusType::SLACK;
    case 4:  return BusType::ISOLATED;
    default: return BusType::PQ;
  }
}

// Helper: DC bus type to JPC integer
static int dc_bus_type_to_jpc(DCBusType bt) {
  switch (bt) {
    case DCBusType::DC_P:        return 1;
    case DCBusType::DC_V:        return 2;
    case DCBusType::DC_ISOLATED: return 4;
    default:                    return 1;  // DC_P default
  }
}

// Helper: JPC integer to DC bus type
static DCBusType jpc_to_dc_bus_type(int bt) {
  switch (bt) {
    case 1:  return DCBusType::DC_P;
    case 2:  return DCBusType::DC_V;
    case 4:  return DCBusType::DC_ISOLATED;
    default: return DCBusType::DC_P;
  }
}

// Helper: Converter mode to JPC integer
static int converter_mode_to_jpc(ConverterMode mode) {
  switch (mode) {
    case ConverterMode::PQ_MODE:  return 1;
    case ConverterMode::VDC_Q:    return 2;
    case ConverterMode::VDC_VAC:  return 3;
    default:                      return 1;
  }
}

// Helper: JPC integer to converter mode
static ConverterMode jpc_to_converter_mode(int mode) {
  switch (mode) {
    case 1:  return ConverterMode::PQ_MODE;
    case 2:  return ConverterMode::VDC_Q;
    case 3:  return ConverterMode::VDC_VAC;
    default: return ConverterMode::PQ_MODE;
  }
}

std::string to_jpc_json(const HybridPowerSystem& sys, int indent) {
  json root;
  
  // Metadata
  root["version"] = "2.0";
  root["baseMVA"] = sys.base_mva;
  root["success"] = true;
  root["iterationsAC"] = 0;
  root["iterationsDC"] = 0;
  
  // AC buses matrix
  json busAC = json::array();
  for (const auto& b : sys.ac.buses) {
    std::vector<double> row(jpc_idx::N_BUS_COLS, 0.0);
    row[jpc_idx::BUS_I] = b.index;
    row[jpc_idx::BUS_TYPE] = bus_type_to_jpc(b.bus_type);
    row[jpc_idx::PD] = b.pd_mw;
    row[jpc_idx::QD] = b.qd_mvar;
    row[jpc_idx::GS] = b.gs_mw;
    row[jpc_idx::BS] = b.bs_mvar;
    row[jpc_idx::BUS_AREA] = b.area;
    row[jpc_idx::VM] = b.vm_pu;
    row[jpc_idx::VA] = b.va_deg;
    row[jpc_idx::BASE_KV] = b.base_kv;
    row[jpc_idx::ZONE] = b.zone;
    row[jpc_idx::VMAX] = b.vmax_pu;
    row[jpc_idx::VMIN] = b.vmin_pu;
    row[jpc_idx::LATITUDE] = b.latitude;
    row[jpc_idx::LONGITUDE] = b.longitude;
    busAC.push_back(row_to_json_array(row));
  }
  root["busAC"] = busAC;
  
  // AC branches matrix
  json branchAC = json::array();
  for (const auto& br : sys.ac.branches) {
    std::vector<double> row(jpc_idx::N_BRANCH_COLS, 0.0);
    row[jpc_idx::F_BUS] = br.from_bus;
    row[jpc_idx::T_BUS] = br.to_bus;
    row[jpc_idx::BR_R] = br.r_pu;
    row[jpc_idx::BR_X] = br.x_pu;
    row[jpc_idx::BR_B] = br.b_pu;
    row[jpc_idx::RATE_A] = br.rate_a_mva;
    row[jpc_idx::RATE_B] = br.rate_b_mva;
    row[jpc_idx::RATE_C] = br.rate_c_mva;
    row[jpc_idx::TAP] = br.tap;
    row[jpc_idx::SHIFT] = br.shift_deg;
    row[jpc_idx::BR_STATUS] = br.in_service ? 1.0 : 0.0;
    row[jpc_idx::ANGMIN] = -360.0;
    row[jpc_idx::ANGMAX] = 360.0;
    row[jpc_idx::DICTKEY] = br.index;
    branchAC.push_back(row_to_json_array(row));
  }
  root["branchAC"] = branchAC;
  
  // Generators matrix
  json genAC = json::array();
  for (const auto& g : sys.ac.generators) {
    std::vector<double> row(jpc_idx::N_GEN_COLS, 0.0);
    row[jpc_idx::GEN_BUS] = g.bus;
    row[jpc_idx::PG] = g.pg_mw;
    row[jpc_idx::QG] = g.qg_mvar;
    row[jpc_idx::QMAX] = g.qmax_mvar;
    row[jpc_idx::QMIN] = g.qmin_mvar;
    row[jpc_idx::VG] = g.vg_pu;
    row[jpc_idx::MBASE] = sys.base_mva;
    row[jpc_idx::GEN_STATUS] = g.in_service ? 1.0 : 0.0;
    row[jpc_idx::PMAX] = g.pmax_mw;
    row[jpc_idx::PMIN] = g.pmin_mw;
    row[jpc_idx::MODEL] = 2.0;  // Polynomial model
    row[jpc_idx::NCOST] = 3.0;  // Quadratic cost
    row[jpc_idx::COST]  = g.cost_c2;
    row[jpc_idx::COST1] = g.cost_c1;
    row[jpc_idx::COST2] = g.cost_c0;
    row[jpc_idx::CARBON_EMISSION] = g.emission_factor_tco2_mwh;
    genAC.push_back(row_to_json_array(row));
  }
  root["genAC"] = genAC;
  
  // AC Loads matrix
  json loadAC = json::array();
  for (const auto& l : sys.ac.loads) {
    std::vector<double> row(jpc_idx::N_LOAD_COLS, 0.0);
    row[jpc_idx::LOAD_I] = l.bus;
    row[jpc_idx::LOAD_CND] = l.bus;  // Connected bus
    row[jpc_idx::LOAD_STATUS] = l.in_service ? 1.0 : 0.0;
    row[jpc_idx::LOAD_PD] = l.p_mw;
    row[jpc_idx::LOAD_QD] = l.q_mvar;
    row[jpc_idx::LOADZ_PERCENT] = 0.0;
    row[jpc_idx::LOADI_PERCENT] = 0.0;
    row[jpc_idx::LOADP_PERCENT] = 100.0;
    loadAC.push_back(row_to_json_array(row));
  }
  root["loadAC"] = loadAC;
  
  // DC buses matrix
  json busDC = json::array();
  for (const auto& b : sys.dc.buses) {
    std::vector<double> row(jpc_idx::N_DCBUS_COLS, 0.0);
    row[jpc_idx::DC_BUS_I] = b.index;
    row[jpc_idx::DC_BUS_TYPE] = dc_bus_type_to_jpc(b.bus_type);
    row[jpc_idx::DC_PD] = b.pd_mw;
    row[jpc_idx::DC_VM] = b.vm_pu;
    row[jpc_idx::DC_VMAX] = b.vmax_pu;
    row[jpc_idx::DC_VMIN] = b.vmin_pu;
    row[jpc_idx::DC_LATITUDE] = b.latitude;
    busDC.push_back(row_to_json_array(row));
  }
  root["busDC"] = busDC;
  
  // DC branches matrix
  json branchDC = json::array();
  for (const auto& br : sys.dc.branches) {
    std::vector<double> row(jpc_idx::N_BRANCH_COLS, 0.0);
    row[jpc_idx::F_BUS] = br.from_bus;
    row[jpc_idx::T_BUS] = br.to_bus;
    row[jpc_idx::BR_R] = br.r_pu;
    row[jpc_idx::RATE_A] = br.rate_a_mva;
    row[jpc_idx::BR_STATUS] = br.in_service ? 1.0 : 0.0;
    row[jpc_idx::DICTKEY] = br.index;
    branchDC.push_back(row_to_json_array(row));
  }
  root["branchDC"] = branchDC;
  
  // DC loads matrix
  json loadDC = json::array();
  for (const auto& l : sys.dc.loads) {
    std::vector<double> row(jpc_idx::N_LOAD_COLS, 0.0);
    row[jpc_idx::LOAD_I] = l.bus;
    row[jpc_idx::LOAD_CND] = l.bus;
    row[jpc_idx::LOAD_STATUS] = l.in_service ? 1.0 : 0.0;
    row[jpc_idx::LOAD_PD] = l.p_mw;
    row[jpc_idx::LOADP_PERCENT] = 100.0;
    loadDC.push_back(row_to_json_array(row));
  }
  root["loadDC"] = loadDC;
  
  // AC storage matrix
  json storage = json::array();
  for (const auto& s : sys.ac.storage) {
    std::vector<double> row(jpc_idx::N_ESS_COLS, 0.0);
    row[jpc_idx::ESS_BUS] = s.bus;
    row[jpc_idx::ESS_POWER_CAPACITY] = s.pmax_mw;
    row[jpc_idx::ESS_ENERGY_CAPACITY] = s.e_rated_mwh;
    row[jpc_idx::ESS_SOC_INIT] = s.soc_init;
    row[jpc_idx::ESS_SOC_MIN] = s.soc_min;
    row[jpc_idx::ESS_SOC_MAX] = s.soc_max;
    row[jpc_idx::ESS_EFFICIENCY] = s.eta_charge;
    row[jpc_idx::ESS_STATUS] = s.in_service ? 1.0 : 0.0;
    row[jpc_idx::ESS_P_DISCHARGE] = s.p_mw > 0 ? s.p_mw : 0.0;
    row[jpc_idx::ESS_P_CHARGE] = s.p_mw < 0 ? -s.p_mw : 0.0;
    storage.push_back(row_to_json_array(row));
  }
  root["storage"] = storage;
  
  // VSC converters matrix
  json converter = json::array();
  for (const auto& c : sys.vsc_converters) {
    std::vector<double> row(jpc_idx::N_CONV_COLS, 0.0);
    row[jpc_idx::CONV_ACBUS] = c.bus_ac;
    row[jpc_idx::CONV_DCBUS] = c.bus_dc;
    row[jpc_idx::CONV_INSERVICE] = c.in_service ? 1.0 : 0.0;
    row[jpc_idx::CONV_P_AC] = c.p_set_mw;
    row[jpc_idx::CONV_Q_AC] = c.q_set_mvar;
    row[jpc_idx::CONV_P_DC] = c.p_set_mw * c.eta;
    row[jpc_idx::CONV_EFF] = c.eta;
    row[jpc_idx::CONV_MODE] = converter_mode_to_jpc(c.control_mode);
    row[jpc_idx::CONV_DROOP_KP] = c.k_vdc;
    row[jpc_idx::CONV_PMAX] = c.pmax_mw;
    row[jpc_idx::CONV_PMIN] = c.pmin_mw;
    row[jpc_idx::CONV_QMAX] = c.qmax_mvar;
    row[jpc_idx::CONV_QMIN] = c.qmin_mvar;
    row[jpc_idx::CONV_LOSS_A]             = c.loss_mw;
    row[jpc_idx::CONV_LOSS_B]             = c.loss_percent;
    row[jpc_idx::CONV_AREA]               = 0.0;
    row[jpc_idx::CONV_GRID_FORMING]       = c.grid_forming ? 1.0 : 0.0;
    row[jpc_idx::CONV_CONTROL_STRATEGY]   = 0.0;
    converter.push_back(row_to_json_array(row));
  }
  root["converter"] = converter;

  // VSC supplemental data — fields not expressible in the fixed-column matrix.
  // Written as a parallel positional array: vscdata[i] corresponds to
  // converter[i].  On read these are merged back by position index.
  {
    json vscdata = json::array();
    for (const auto& c : sys.vsc_converters) {
      json obj;
      obj["index"]              = c.index;
      obj["name"]               = c.name;
      obj["forced_outage_rate"] = c.forced_outage_rate;
      obj["mttr_hr"]            = c.mttr_hr;
      obj["mtbf_hr"]            = c.mtbf_hr;
      obj["t_scheduled_hr"]     = c.t_scheduled_hr;
      obj["r_conv_ac_pu"]       = c.r_conv_ac_pu;
      obj["i_ac_max_pu"]        = c.i_ac_max_pu;
      obj["i_dc_max_pu"]        = c.i_dc_max_pu;
      obj["k_m_modulation"]     = c.k_m_modulation;
      obj["m_min"]              = c.m_min;
      obj["m_max"]              = c.m_max;
      obj["x_sc_pu"]            = c.x_sc_pu;
      obj["vn_ac_kv"]           = c.vn_ac_kv;
      obj["vn_dc_kv"]           = c.vn_dc_kv;
      obj["controllable"]       = c.controllable;
      vscdata.push_back(obj);
    }
    root["vscdata"] = vscdata;
  }

  // External grids matrix
  json ext_grid = json::array();
  for (const auto& e : sys.ac.external_grids) {
    std::vector<double> row(jpc_idx::N_EXT_GRID_COLS, 0.0);
    row[jpc_idx::EXT_GRID_BUS] = e.bus;
    row[jpc_idx::EXT_VM_PU] = e.vm_pu;
    row[jpc_idx::EXT_VA_DEG] = e.va_deg;
    row[jpc_idx::EXT_PMAX] = 1000.0;  // Default large value for slack
    row[jpc_idx::EXT_PMIN] = -1000.0;
    row[jpc_idx::EXT_QMAX] = 1000.0;
    row[jpc_idx::EXT_QMIN] = -1000.0;
    row[jpc_idx::EXT_STATUS] = e.in_service ? 1.0 : 0.0;
    row[jpc_idx::EXT_SC_MVA_3PH] = e.s_sc_max_mva;
    row[jpc_idx::EXT_SC_MVA_1PH] = e.s_sc_min_mva;
    row[jpc_idx::EXT_RX_RATIO] = e.rx_max;
    ext_grid.push_back(row_to_json_array(row));
  }
  root["ext_grid"] = ext_grid;
  
  // Rich component tables.  These are exported as structured records instead
  // of empty placeholders so JPC JSON export preserves model data even when a
  // downstream matrix-case readers only consume the core tables.
  root["genDC"] = json::array();
  for (const auto& g : sys.dc.dc_static_generators) {
    root["genDC"].push_back(dc_static_generator_to_json(g));
  }
  root["loadAC_flex"] = json::array();
  for (const auto& l : sys.ac.flexible_loads) {
    root["loadAC_flex"].push_back(flexible_load_to_json(l));
  }
  root["loadAC_asymm"] = json::array();
  for (const auto& l : sys.ac.asymmetric_loads) {
    root["loadAC_asymm"].push_back(asymmetric_load_to_json(l));
  }
  root["branch3ph"] = json::array();
  if (sys.three_phase_ac) {
    for (const auto& l : sys.three_phase_ac->lines) {
      root["branch3ph"].push_back(three_phase_line_to_json(l));
    }
  }
  root["sgenAC"] = json::array();
  for (const auto& g : sys.ac.static_generators) {
    root["sgenAC"].push_back(static_generator_to_json(g));
  }
  root["sgenDC"] = json::array();
  for (const auto& g : sys.dc.static_generators) {
    root["sgenDC"].push_back(static_generator_to_json(g));
  }
  root["storageetap"] = json::array();
  for (const auto& s : sys.ac.storage) {
    auto js = storage_to_json(s);
    js["domain"] = "AC";
    root["storageetap"].push_back(std::move(js));
  }
  for (const auto& s : sys.dc.storage) {
    auto js = storage_to_json(s);
    js["domain"] = "DC";
    root["storageetap"].push_back(std::move(js));
  }
  root["pv"] = json::array();
  for (const auto& p : sys.dc.pv_arrays) {
    root["pv"].push_back(pv_array_dc_to_json(p));
  }
  root["pv_acsystem"] = json::array();
  for (const auto& p : sys.ac.pv_systems) {
    root["pv_acsystem"].push_back(pv_system_to_json(p));
  }
  root["energyrouterCore"] = json::array();
  root["energyrouterConverter"] = json::array();
  for (const auto& er : sys.energy_routers) {
    root["energyrouterCore"].push_back(energy_router_to_json(er));
    for (const auto& p : er.ports) {
      json jp = er_port_to_json(p);
      jp["energy_router_id"] = er.index;
      root["energyrouterConverter"].push_back(std::move(jp));
    }
  }
  root["hvcb"] = json::array();
  for (const auto& cb : sys.ac.circuit_breakers) {
    root["hvcb"].push_back(circuit_breaker_to_json(cb));
  }
  root["microgrid"] = json::array();
  for (const auto& m : sys.microgrids) {
    root["microgrid"].push_back(microgrid_to_json(m));
  }
  // ── Additional rich tables not covered by the JPC matrix core ──────────────
  root["shuntAC"] = json::array();
  for (const auto& s : sys.ac.shunts)
    root["shuntAC"].push_back(shunt_to_json(s));
  root["renewableAC"] = json::array();
  for (const auto& r : sys.ac.renewable_gens)
    root["renewableAC"].push_back(renewable_gen_to_json(r));
  root["trafo2w"] = json::array();
  for (const auto& t : sys.ac.transformers_2w)
    root["trafo2w"].push_back(transformer2w_to_json(t));
  root["trafo3w"] = json::array();
  for (const auto& t : sys.ac.transformers_3w)
    root["trafo3w"].push_back(transformer3w_to_json(t));
  root["switch_ac"] = json::array();
  for (const auto& s : sys.ac.switches)
    root["switch_ac"].push_back(switch_to_json(s));
  root["ev_station"] = json::array();
  for (const auto& c : sys.ac.charging_stations)
    root["ev_station"].push_back(charging_station_to_json(c));
  root["charger"] = json::array();
  for (const auto& c : sys.ac.chargers)
    root["charger"].push_back(charger_to_json(c));
  root["motorAC"] = json::array();
  for (const auto& m : sys.ac.motors)
    root["motorAC"].push_back(asynchronous_motor_to_json(m));
  root["mobile_storage"] = json::array();
  for (const auto& s : sys.mobile_storage)
    root["mobile_storage"].push_back(mobile_storage_to_json(s));
  root["vpp"] = json::array();
  for (const auto& v : sys.vpps)
    root["vpp"].push_back(vpp_to_json(v));
  root["dcdcconv"] = json::array();
  for (const auto& c : sys.dc.dcdc_converters)
    root["dcdcconv"].push_back(dcdc_to_json(c));
  root["dccb"] = json::array();
  for (const auto& cb : sys.dc.dc_circuit_breakers)
    root["dccb"].push_back(dc_circuit_breaker_to_json(cb));
  root["bus_name_to_id"] = json::object();
  
  // Add bus names to ID mapping
  for (const auto& b : sys.ac.buses) {
    if (!b.name.empty()) {
      root["bus_name_to_id"][b.name] = b.index;
    }
  }
  
  return root.dump(indent);
}

void save_jpc_json(const HybridPowerSystem& sys, const std::string& path, int indent) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Cannot open file for writing: " + path);
  }
  ofs << to_jpc_json(sys, indent);
}

HybridPowerSystem from_jpc_json(const std::string& json_str) {
  const json root = json::parse(json_str);
  HybridPowerSystem sys;
  
  // Metadata
  sys.base_mva = jget(root, "baseMVA", 100.0);
  sys.ac.base_mva = sys.base_mva;
  sys.dc.base_mva = sys.base_mva;
  
  // AC buses
  if (root.contains("busAC")) {
    for (const auto& row : root["busAC"]) {
      ACBus b;
      auto r = row.get<std::vector<double>>();
      b.index = static_cast<int>(r[jpc_idx::BUS_I]);
      b.bus_type = jpc_to_bus_type(static_cast<int>(r[jpc_idx::BUS_TYPE]));
      b.pd_mw = r.size() > jpc_idx::PD ? r[jpc_idx::PD] : 0.0;
      b.qd_mvar = r.size() > jpc_idx::QD ? r[jpc_idx::QD] : 0.0;
      b.gs_mw = r.size() > jpc_idx::GS ? r[jpc_idx::GS] : 0.0;
      b.bs_mvar = r.size() > jpc_idx::BS ? r[jpc_idx::BS] : 0.0;
      b.area = r.size() > jpc_idx::BUS_AREA ? static_cast<int>(r[jpc_idx::BUS_AREA]) : 1;
      b.vm_pu = r.size() > jpc_idx::VM ? r[jpc_idx::VM] : 1.0;
      b.va_deg = r.size() > jpc_idx::VA ? r[jpc_idx::VA] : 0.0;
      b.base_kv = r.size() > jpc_idx::BASE_KV ? r[jpc_idx::BASE_KV] : 110.0;
      b.zone = r.size() > jpc_idx::ZONE ? static_cast<int>(r[jpc_idx::ZONE]) : 1;
      b.vmax_pu = r.size() > jpc_idx::VMAX ? r[jpc_idx::VMAX] : 1.1;
      b.vmin_pu = r.size() > jpc_idx::VMIN ? r[jpc_idx::VMIN] : 0.9;
      b.latitude = r.size() > jpc_idx::LATITUDE ? r[jpc_idx::LATITUDE] : 0.0;
      b.longitude = r.size() > jpc_idx::LONGITUDE ? r[jpc_idx::LONGITUDE] : 0.0;
      b.in_service = true;
      sys.ac.buses.push_back(b);
    }
  }
  
  // AC branches
  if (root.contains("branchAC")) {
    int idx = 0;
    for (const auto& row : root["branchAC"]) {
      ACBranch br;
      auto r = row.get<std::vector<double>>();
      // Prefer the stored DICTKEY index so that br.index survives an
      // export→import round-trip unchanged (non-sequential IDs preserved).
      br.index = (r.size() > static_cast<size_t>(jpc_idx::DICTKEY) && r[jpc_idx::DICTKEY] >= 0)
                     ? static_cast<int>(r[jpc_idx::DICTKEY]) : idx;
      ++idx;
      br.from_bus = static_cast<int>(r[jpc_idx::F_BUS]);
      br.to_bus = static_cast<int>(r[jpc_idx::T_BUS]);
      br.r_pu = r.size() > jpc_idx::BR_R ? r[jpc_idx::BR_R] : 0.0;
      br.x_pu = r.size() > jpc_idx::BR_X ? r[jpc_idx::BR_X] : 0.0;
      br.b_pu = r.size() > jpc_idx::BR_B ? r[jpc_idx::BR_B] : 0.0;
      br.rate_a_mva = r.size() > jpc_idx::RATE_A ? r[jpc_idx::RATE_A] : 0.0;
      br.rate_b_mva = r.size() > jpc_idx::RATE_B ? r[jpc_idx::RATE_B] : 0.0;
      br.rate_c_mva = r.size() > jpc_idx::RATE_C ? r[jpc_idx::RATE_C] : 0.0;
      br.tap = r.size() > jpc_idx::TAP ? r[jpc_idx::TAP] : 1.0;
      br.shift_deg = r.size() > jpc_idx::SHIFT ? r[jpc_idx::SHIFT] : 0.0;
      br.in_service = r.size() > jpc_idx::BR_STATUS ? r[jpc_idx::BR_STATUS] > 0.5 : true;
      sys.ac.branches.push_back(br);
    }
  }
  
  // Build set of slack bus IDs from already-imported AC buses (BUS_TYPE == 3 in JPC).
  // Used below to determine Generator::is_slack without hardcoding bus 1.
  std::unordered_set<int> slack_bus_ids;
  for (const auto& b : sys.ac.buses) {
    if (b.bus_type == BusType::SLACK) slack_bus_ids.insert(b.index);
  }

  // Generators
  if (root.contains("genAC")) {
    int idx = 0;
    for (const auto& row : root["genAC"]) {
      Generator g;
      auto r = row.get<std::vector<double>>();
      g.index = idx++;
      g.bus = static_cast<int>(r[jpc_idx::GEN_BUS]);
      g.pg_mw = r.size() > jpc_idx::PG ? r[jpc_idx::PG] : 0.0;
      g.qg_mvar = r.size() > jpc_idx::QG ? r[jpc_idx::QG] : 0.0;
      g.qmax_mvar = r.size() > jpc_idx::QMAX ? r[jpc_idx::QMAX] : 0.0;
      g.qmin_mvar = r.size() > jpc_idx::QMIN ? r[jpc_idx::QMIN] : 0.0;
      g.vg_pu = r.size() > jpc_idx::VG ? r[jpc_idx::VG] : 1.0;
      g.in_service = r.size() > jpc_idx::GEN_STATUS ? r[jpc_idx::GEN_STATUS] > 0.5 : true;
      g.pmax_mw = r.size() > jpc_idx::PMAX ? r[jpc_idx::PMAX] : 0.0;
      g.pmin_mw = r.size() > jpc_idx::PMIN ? r[jpc_idx::PMIN] : 0.0;
      g.cost_c2 = r.size() > jpc_idx::COST  ? r[jpc_idx::COST]  : 0.0;
      g.cost_c1 = r.size() > jpc_idx::COST1 ? r[jpc_idx::COST1] : 0.0;
      g.cost_c0 = r.size() > jpc_idx::COST2 ? r[jpc_idx::COST2] : 0.0;
      g.emission_factor_tco2_mwh = r.size() > jpc_idx::CARBON_EMISSION ? r[jpc_idx::CARBON_EMISSION] : 0.0;
      // Determine slack status from the bus BUS_TYPE column (JPC type 3 = REF/SLACK)
      // rather than hardcoding bus index 1, which breaks multi-area or renumbered cases.
      g.is_slack = slack_bus_ids.count(g.bus) > 0;
      sys.ac.generators.push_back(g);
    }
  }
  
  // AC Loads
  if (root.contains("loadAC")) {
    int idx = 0;
    for (const auto& row : root["loadAC"]) {
      Load l;
      auto r = row.get<std::vector<double>>();
      l.index = idx++;
      l.bus = static_cast<int>(r[jpc_idx::LOAD_I]);
      l.in_service = r.size() > jpc_idx::LOAD_STATUS ? r[jpc_idx::LOAD_STATUS] > 0.5 : true;
      l.p_mw = r.size() > jpc_idx::LOAD_PD ? r[jpc_idx::LOAD_PD] : 0.0;
      l.q_mvar = r.size() > jpc_idx::LOAD_QD ? r[jpc_idx::LOAD_QD] : 0.0;
      sys.ac.loads.push_back(l);
    }
  }
  
  // DC buses
  if (root.contains("busDC")) {
    for (const auto& row : root["busDC"]) {
      DCBus b;
      auto r = row.get<std::vector<double>>();
      b.index = static_cast<int>(r[jpc_idx::DC_BUS_I]);
      b.bus_type = jpc_to_dc_bus_type(static_cast<int>(r[jpc_idx::DC_BUS_TYPE]));
      b.pd_mw = r.size() > jpc_idx::DC_PD ? r[jpc_idx::DC_PD] : 0.0;
      b.vm_pu = r.size() > jpc_idx::DC_VM ? r[jpc_idx::DC_VM] : 1.0;
      b.vmax_pu = r.size() > jpc_idx::DC_VMAX ? r[jpc_idx::DC_VMAX] : 1.1;
      b.vmin_pu = r.size() > jpc_idx::DC_VMIN ? r[jpc_idx::DC_VMIN] : 0.9;
      b.latitude = r.size() > jpc_idx::DC_LATITUDE ? r[jpc_idx::DC_LATITUDE] : 0.0;
      b.in_service = true;
      sys.dc.buses.push_back(b);
    }
  }
  
  // DC branches
  if (root.contains("branchDC")) {
    int idx = 0;
    for (const auto& row : root["branchDC"]) {
      DCBranch br;
      auto r = row.get<std::vector<double>>();
      br.index = (r.size() > static_cast<size_t>(jpc_idx::DICTKEY) && r[jpc_idx::DICTKEY] >= 0)
                     ? static_cast<int>(r[jpc_idx::DICTKEY]) : idx;
      ++idx;
      br.from_bus = static_cast<int>(r[jpc_idx::F_BUS]);
      br.to_bus = static_cast<int>(r[jpc_idx::T_BUS]);
      br.r_pu = r.size() > jpc_idx::BR_R ? r[jpc_idx::BR_R] : 0.0;
      br.rate_a_mva = r.size() > jpc_idx::RATE_A ? r[jpc_idx::RATE_A] : 0.0;
      br.in_service = r.size() > jpc_idx::BR_STATUS ? r[jpc_idx::BR_STATUS] > 0.5 : true;
      sys.dc.branches.push_back(br);
    }
  }
  
  // DC loads
  if (root.contains("loadDC")) {
    int idx = 0;
    for (const auto& row : root["loadDC"]) {
      DCLoad l;
      auto r = row.get<std::vector<double>>();
      l.index = idx++;
      l.bus = static_cast<int>(r[jpc_idx::LOAD_I]);
      l.in_service = r.size() > jpc_idx::LOAD_STATUS ? r[jpc_idx::LOAD_STATUS] > 0.5 : true;
      l.p_mw = r.size() > jpc_idx::LOAD_PD ? r[jpc_idx::LOAD_PD] : 0.0;
      sys.dc.loads.push_back(l);
    }
  }
  
  // Storage
  if (root.contains("storage")) {
    int idx = 0;
    for (const auto& row : root["storage"]) {
      Storage s;
      auto r = row.get<std::vector<double>>();
      s.index = idx++;
      s.bus = static_cast<int>(r[jpc_idx::ESS_BUS]);
      s.pmax_mw = r.size() > jpc_idx::ESS_POWER_CAPACITY ? r[jpc_idx::ESS_POWER_CAPACITY] : 0.0;
      s.e_rated_mwh = r.size() > jpc_idx::ESS_ENERGY_CAPACITY ? r[jpc_idx::ESS_ENERGY_CAPACITY] : 0.0;
      s.soc_init = r.size() > jpc_idx::ESS_SOC_INIT ? r[jpc_idx::ESS_SOC_INIT] : 0.5;
      s.soc_min = r.size() > jpc_idx::ESS_SOC_MIN ? r[jpc_idx::ESS_SOC_MIN] : 0.1;
      s.soc_max = r.size() > jpc_idx::ESS_SOC_MAX ? r[jpc_idx::ESS_SOC_MAX] : 0.9;
      s.eta_charge = r.size() > jpc_idx::ESS_EFFICIENCY ? r[jpc_idx::ESS_EFFICIENCY] : 0.95;
      s.eta_discharge = s.eta_charge;
      s.in_service = r.size() > jpc_idx::ESS_STATUS ? r[jpc_idx::ESS_STATUS] > 0.5 : true;
      sys.ac.storage.push_back(s);
    }
  }
  
  // VSC converters
  if (root.contains("converter")) {
    int idx = 0;
    for (const auto& row : root["converter"]) {
      VSCConverter c;
      auto r = row.get<std::vector<double>>();
      c.index = idx++;
      c.bus_ac = static_cast<int>(r[jpc_idx::CONV_ACBUS]);
      c.bus_dc = static_cast<int>(r[jpc_idx::CONV_DCBUS]);
      c.in_service = r.size() > jpc_idx::CONV_INSERVICE ? r[jpc_idx::CONV_INSERVICE] > 0.5 : true;
      c.p_set_mw = r.size() > jpc_idx::CONV_P_AC ? r[jpc_idx::CONV_P_AC] : 0.0;
      c.q_set_mvar = r.size() > jpc_idx::CONV_Q_AC ? r[jpc_idx::CONV_Q_AC] : 0.0;
      c.eta = r.size() > jpc_idx::CONV_EFF ? r[jpc_idx::CONV_EFF] : 0.98;
      c.control_mode = r.size() > jpc_idx::CONV_MODE ? jpc_to_converter_mode(static_cast<int>(r[jpc_idx::CONV_MODE])) : ConverterMode::PQ_MODE;
      c.k_vdc = r.size() > jpc_idx::CONV_DROOP_KP ? r[jpc_idx::CONV_DROOP_KP] : 0.1;
      c.pmax_mw = r.size() > jpc_idx::CONV_PMAX ? r[jpc_idx::CONV_PMAX] : 100.0;
      c.pmin_mw = r.size() > jpc_idx::CONV_PMIN ? r[jpc_idx::CONV_PMIN] : -100.0;
      c.qmax_mvar = r.size() > jpc_idx::CONV_QMAX ? r[jpc_idx::CONV_QMAX] : 50.0;
      c.qmin_mvar = r.size() > jpc_idx::CONV_QMIN ? r[jpc_idx::CONV_QMIN] : -50.0;
      c.loss_mw   = r.size() > jpc_idx::CONV_LOSS_A ? r[jpc_idx::CONV_LOSS_A] : 0.0;
      c.loss_percent = r.size() > jpc_idx::CONV_LOSS_B ? r[jpc_idx::CONV_LOSS_B] : 0.0;
      c.grid_forming = r.size() > jpc_idx::CONV_GRID_FORMING && r[jpc_idx::CONV_GRID_FORMING] > 0.5;
      sys.vsc_converters.push_back(c);
    }
  }

  // VSC supplemental data — merge by position index into the converter list
  // loaded from the "converter" matrix above.
  if (root.contains("vscdata") && root["vscdata"].is_array()) {
    const auto& vd = root["vscdata"];
    for (size_t pos = 0; pos < vd.size() && pos < sys.vsc_converters.size(); ++pos) {
      const auto& j = vd[pos];
      auto& c = sys.vsc_converters[pos];
      c.index              = jget<int>(j, "index", c.index);
      c.name               = jget<std::string>(j, "name", c.name);
      c.forced_outage_rate = jget<double>(j, "forced_outage_rate", c.forced_outage_rate);
      c.mttr_hr            = jget<double>(j, "mttr_hr", c.mttr_hr);
      c.mtbf_hr            = jget<double>(j, "mtbf_hr", c.mtbf_hr);
      c.t_scheduled_hr     = jget<double>(j, "t_scheduled_hr", c.t_scheduled_hr);
      c.r_conv_ac_pu       = jget<double>(j, "r_conv_ac_pu", c.r_conv_ac_pu);
      c.i_ac_max_pu        = jget<double>(j, "i_ac_max_pu", c.i_ac_max_pu);
      c.i_dc_max_pu        = jget<double>(j, "i_dc_max_pu", c.i_dc_max_pu);
      c.k_m_modulation     = jget<double>(j, "k_m_modulation", c.k_m_modulation);
      c.m_min              = jget<double>(j, "m_min", c.m_min);
      c.m_max              = jget<double>(j, "m_max", c.m_max);
      c.x_sc_pu            = jget<double>(j, "x_sc_pu", c.x_sc_pu);
      c.vn_ac_kv           = jget<double>(j, "vn_ac_kv", c.vn_ac_kv);
      c.vn_dc_kv           = jget<double>(j, "vn_dc_kv", c.vn_dc_kv);
      c.controllable       = jget<bool>(j, "controllable", c.controllable);
    }
  }

  // External grids
  if (root.contains("ext_grid")) {
    int idx = 0;
    for (const auto& row : root["ext_grid"]) {
      ExternalGrid e;
      auto r = row.get<std::vector<double>>();
      e.index = idx++;
      e.bus = static_cast<int>(r[jpc_idx::EXT_GRID_BUS]);
      e.vm_pu = r.size() > jpc_idx::EXT_VM_PU ? r[jpc_idx::EXT_VM_PU] : 1.0;
      e.va_deg = r.size() > jpc_idx::EXT_VA_DEG ? r[jpc_idx::EXT_VA_DEG] : 0.0;
      // JPC pmax/pmin/qmax/qmin don't map directly to ExternalGrid struct
      e.in_service = r.size() > jpc_idx::EXT_STATUS ? r[jpc_idx::EXT_STATUS] > 0.5 : true;
      e.s_sc_max_mva = r.size() > jpc_idx::EXT_SC_MVA_3PH ? r[jpc_idx::EXT_SC_MVA_3PH] : 1000.0;
      e.s_sc_min_mva = r.size() > jpc_idx::EXT_SC_MVA_1PH ? r[jpc_idx::EXT_SC_MVA_1PH] : 1000.0;
      e.rx_max = r.size() > jpc_idx::EXT_RX_RATIO ? r[jpc_idx::EXT_RX_RATIO] : 0.1;
      sys.ac.external_grids.push_back(e);
    }
  }

  // ── Rich component tables (structured JSON, preserved across JPC round-trips) ──

  if (root.contains("genDC") && root["genDC"].is_array())
    for (const auto& j : root["genDC"])
      sys.dc.dc_static_generators.push_back(dc_static_generator_from_json(j));

  if (root.contains("loadAC_flex") && root["loadAC_flex"].is_array())
    for (const auto& j : root["loadAC_flex"])
      sys.ac.flexible_loads.push_back(flexible_load_from_json(j));

  if (root.contains("loadAC_asymm") && root["loadAC_asymm"].is_array())
    for (const auto& j : root["loadAC_asymm"])
      sys.ac.asymmetric_loads.push_back(asymmetric_load_from_json(j));

  if (root.contains("sgenAC") && root["sgenAC"].is_array())
    for (const auto& j : root["sgenAC"])
      sys.ac.static_generators.push_back(static_generator_from_json(j));

  if (root.contains("sgenDC") && root["sgenDC"].is_array())
    for (const auto& j : root["sgenDC"])
      sys.dc.static_generators.push_back(static_generator_from_json(j));

  // storageetap is the structured replacement for the JPC matrix "storage" table.
  // It carries richer fields (eta_charge, eta_discharge, soc_init, etc.) and a
  // "domain" tag that distinguishes AC from DC storage.
  // When storageetap is present and contains AC entries, prefer it over the
  // matrix-form "storage" import (which carries fewer fields) by clearing any
  // AC storage that was loaded from the matrix form first.
  if (root.contains("storageetap") && root["storageetap"].is_array()) {
    bool has_ac = false;
    for (const auto& j : root["storageetap"]) {
      if (jget<std::string>(j, "domain", "AC") != "DC") { has_ac = true; break; }
    }
    if (has_ac)
      sys.ac.storage.clear();  // discard matrix-form entries; prefer storageetap
    for (const auto& j : root["storageetap"]) {
      const std::string dom = jget<std::string>(j, "domain", "AC");
      if (dom == "DC")
        sys.dc.storage.push_back(storage_from_json(j));
      else
        sys.ac.storage.push_back(storage_from_json(j));
    }
  }

  if (root.contains("pv") && root["pv"].is_array())
    for (const auto& j : root["pv"])
      sys.dc.pv_arrays.push_back(pv_array_dc_from_json(j));

  if (root.contains("pv_acsystem") && root["pv_acsystem"].is_array())
    for (const auto& j : root["pv_acsystem"])
      sys.ac.pv_systems.push_back(pv_system_from_json(j));

  if (root.contains("hvcb") && root["hvcb"].is_array())
    for (const auto& j : root["hvcb"])
      sys.ac.circuit_breakers.push_back(circuit_breaker_from_json(j));

  if (root.contains("microgrid") && root["microgrid"].is_array())
    for (const auto& j : root["microgrid"])
      sys.microgrids.push_back(microgrid_from_json(j));

  // energyrouterCore already embeds ports as a nested "ports" array; the flat
  // energyrouterConverter table is a redundant export for external tooling only.
  if (root.contains("energyrouterCore") && root["energyrouterCore"].is_array())
    for (const auto& j : root["energyrouterCore"])
      sys.energy_routers.push_back(energy_router_from_json(j));

  // branch3ph: flat array of ThreePhaseACLine records exported from sys.three_phase_ac.
  if (root.contains("branch3ph") && root["branch3ph"].is_array() &&
      !root["branch3ph"].empty()) {
    if (!sys.three_phase_ac) sys.three_phase_ac = ThreePhaseACSystem{};
    for (const auto& j : root["branch3ph"])
      sys.three_phase_ac->lines.push_back(three_phase_line_from_json(j));
  }

  // ── Additional rich tables not covered by the JPC matrix core ──────────────
  if (root.contains("shuntAC") && root["shuntAC"].is_array())
    for (const auto& j : root["shuntAC"])
      sys.ac.shunts.push_back(shunt_from_json(j));

  if (root.contains("renewableAC") && root["renewableAC"].is_array())
    for (const auto& j : root["renewableAC"])
      sys.ac.renewable_gens.push_back(renewable_gen_from_json(j));

  if (root.contains("trafo2w") && root["trafo2w"].is_array())
    for (const auto& j : root["trafo2w"])
      sys.ac.transformers_2w.push_back(transformer2w_from_json(j));

  if (root.contains("trafo3w") && root["trafo3w"].is_array())
    for (const auto& j : root["trafo3w"])
      sys.ac.transformers_3w.push_back(transformer3w_from_json(j));

  if (root.contains("switch_ac") && root["switch_ac"].is_array())
    for (const auto& j : root["switch_ac"])
      sys.ac.switches.push_back(switch_from_json(j));

  if (root.contains("ev_station") && root["ev_station"].is_array())
    for (const auto& j : root["ev_station"])
      sys.ac.charging_stations.push_back(charging_station_from_json(j));

  if (root.contains("charger") && root["charger"].is_array())
    for (const auto& j : root["charger"])
      sys.ac.chargers.push_back(charger_from_json(j));

  if (root.contains("motorAC") && root["motorAC"].is_array())
    for (const auto& j : root["motorAC"])
      sys.ac.motors.push_back(asynchronous_motor_from_json(j));

  if (root.contains("mobile_storage") && root["mobile_storage"].is_array())
    for (const auto& j : root["mobile_storage"])
      sys.mobile_storage.push_back(mobile_storage_from_json(j));

  if (root.contains("vpp") && root["vpp"].is_array())
    for (const auto& j : root["vpp"])
      sys.vpps.push_back(vpp_from_json(j));

  if (root.contains("dcdcconv") && root["dcdcconv"].is_array())
    for (const auto& j : root["dcdcconv"])
      sys.dc.dcdc_converters.push_back(dcdc_from_json(j));

  if (root.contains("dccb") && root["dccb"].is_array())
    for (const auto& j : root["dccb"])
      sys.dc.dc_circuit_breakers.push_back(dc_circuit_breaker_from_json(j));

  repair_legacy_transformer2w_branch_links(sys);
  return sys;
}

HybridPowerSystem load_jpc_json(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("Cannot open file for reading: " + path);
  }
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
  return from_jpc_json(content);
}

}  // namespace hacdcpf::io
