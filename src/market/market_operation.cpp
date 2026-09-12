#include "hacdcpf/market/southern_market.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <thread>

namespace hacdcpf::market {
namespace {
using J = nlohmann::json;
const std::vector<std::string> factors = {"load_scale", "wind_scale", "solar_scale", "inflow_scale", "bid_scale", "line_limit_scale", "generator_bid_scale", "load_bid_scale"};
void require(bool condition, const std::string& message) {
  if (!condition) throw std::invalid_argument("market operation: " + message);
}
void keys(const J& j, const std::set<std::string>& allowed) {
  require(j.is_object(), "expected object");
  for (auto i = j.begin(); i != j.end(); ++i) require(allowed.count(i.key()), "unknown field " + i.key());
}
double number(const J& j, const char* name, double lo, double hi) {
  require(j.at(name).is_number(), std::string(name)+" must be numeric");
  const double n = j.at(name).get<double>();
  require(std::isfinite(n) && n >= lo && n <= hi, std::string(name)+" outside bounds"); return n;
}
double slot(const J& j, const char* field, int t) { return j.at(field).at(t).get<double>(); }
std::map<int, const J*> index(const J& rows) {
  std::map<int, const J*> result;
  for (const auto& r : rows) result.emplace(r.at("id").get<int>(), &r);
  return result;
}
J apply(const J& base, const J& carry, const J& change, double penalty) {
  J b = base;
  // Rules 2.3/2.4: authored overlays precede stochastic factors and 2.4.1.2
  // reconciliation. State variables remain owned by the daily recurrence.
  for (const auto& edit : change.value("boundary_overrides", J::array())) {
    const auto table = edit.at("table").get<std::string>(), field = edit.at("field").get<std::string>();
    auto& rows = b.at(table);
    auto it = std::find_if(rows.begin(), rows.end(), [&](const J& r) { return r.at("id") == edit.at("id"); });
    require(it != rows.end(), "unknown override ID");
    auto& value = it->at(field);
    if (value.is_array()) for (int t = edit.at("first_slot"); t <= edit.at("last_slot").get<int>(); ++t) value[t] = edit.at("value");
    else value = edit.at("value");
  }
  for (const auto& [table, fields] : std::map<std::string,std::vector<std::string>>{
    {"generators", {"initial_on", "initial_power_mw", "initial_state_minutes"}},
    {"storage", {"initial_mwh"}}, {"reservoirs", {"initial_level_m", "initial_release_m3_s", "release_history_m3_s"}},
    {"dc_links", {"initial_mw", "initial_adjustment"}}}) {
    const auto lookup = index(carry.at(table));
    for (auto& r : b[table]) for (const auto& field : fields) r[field] = lookup.at(r.at("id"))->at(field);
  }
  const int first = change.at("first_slot"), last = change.at("last_slot");
  auto scale = [&](J& r, const char* field, const char* factor) {
    for (int t = first; t <= last; ++t) r[field][t] = slot(r, field, t)*change.at(factor).get<double>();
  };
  for (auto& r : b["areas"]) scale(r, "load_mw", "load_scale");
  for (auto& r : b["buses"]) { scale(r, "load_mw", "load_scale"); scale(r, "q_load_mvar", "load_scale"); }
  for (auto& g : b["generators"]) {
    if (g["kind"] == "wind") scale(g, "forecast_mw", "wind_scale");
    if (g["kind"] == "solar") scale(g, "forecast_mw", "solar_scale");
    // Offers are day-level curves in the existing schema, so bid_scale is daily.
    for (auto& s : g["segments"]) s["price_per_mwh"] = s["price_per_mwh"].get<double>()*change.at("bid_scale").get<double>()*change.value("generator_bid_scale", 1.0);
  }
  for (auto& r : b["storage"]) for (const auto* f : {"discharge_price", "charge_price"}) r[f] = r[f].get<double>()*change.at("bid_scale").get<double>()*change.value("generator_bid_scale", 1.0);
  for (auto& r : b["controllable_loads"]) for (auto& v : r["compensation_per_mwh"]) v = v.get<double>()*change.at("bid_scale").get<double>()*change.value("load_bid_scale", 1.0);
  for (auto& r : b["reservoirs"]) scale(r, "inflow_m3_s", "inflow_scale");
  for (auto& r : b["branches"]) for (const auto* f : {"min_mw", "max_mw", "rate_mva"}) scale(r, f, "line_limit_scale");
  for (const auto& [table, field] : {std::pair{"generators", "generator_outages"}, std::pair{"branches", "branch_outages"}}) {
    for (auto& r : b[table]) if (std::find(change.at(field).begin(), change.at(field).end(), r.at("id")) != change.at(field).end())
      for (int t = first; t <= last; ++t) r["available"][t] = 0;
  }
  b["execution"]["ac_security"] = "schedule_only";
  b["execution"]["balance_policy"] = "diagnostic";
  b["execution"]["balance_penalty_per_mwh"] = penalty;
  return b;
}
J daily_window(const J& base, const J& carry, const J& change, const J& next, double penalty, J& provenance) {
  // Rules 2.6 T=98; executable selection convention: southern_execution_contract.md.
  J b = apply(base, carry, change, penalty);
  const J forecast = apply(base, carry, next, penalty);
  std::vector<double> load(96, 0.0);
  for (const auto& area : forecast.at("areas")) for (int t = 0; t < 96; ++t)
    load[t] += slot(area, "load_mw", t);
  const int valley = static_cast<int>(std::min_element(load.begin(), load.end())-load.begin());
  int peak = static_cast<int>(std::max_element(load.begin(), load.end())-load.begin());
  if (peak == valley) peak = 95; // Flat curve: distinct first/last representatives.
  std::vector<int> selected = {valley, peak}; std::sort(selected.begin(), selected.end());
  provenance = {{"selection", "total-dispatch-load-extrema:first-tie:flat-first-last"},
    {"points", J::array()}, {"scalar_scope", "D-day scalar parameters and generation/storage offers apply to all 98 points"}};
  const auto schema = southern_market_schema();
  for (int k = 0; k < 2; ++k) {
    const int t = selected[k], target = 96+k;
    const std::string kind = t == valley ? "valley" : "peak";
    // Preserve the authored duration/weight for each kind; do not invent gap energy.
    for (const auto& p : base.at("periods")) if (p.at("kind") == kind) b["periods"][target] = p;
    b["periods"][target]["start_minute"] = 1440+15*t;
    for (auto table = schema.at("properties").begin(); table != schema.at("properties").end(); ++table) {
      if (table.key() == "periods" || table.value().value("type", "") != "array" ||
          !table.value().at("items").contains("properties")) continue;
      const auto sources = index(forecast.at(table.key()));
      for (auto& row : b.at(table.key())) {
        const auto& source = *sources.at(row.at("id").get<int>());
        for (auto field = table.value().at("items").at("properties").begin(); field != table.value().at("items").at("properties").end(); ++field)
          if (field.value().value("type", "") == "array" && field.value().value("minItems", 0) == 98 &&
              field.value().value("maxItems", 0) == 98 && row.contains(field.key()))
            row[field.key()][target] = source.at(field.key()).at(t);
      }
    }
    provenance["points"].push_back({{"target_slot",target},{"source_slot",t},{"kind",kind},
      {"load_mw",load[t]},{"start_minute",1440+15*t},
      {"duration_hr",b["periods"][target]["duration_hr"]},{"weight_hr",b["periods"][target]["weight_hr"]}});
  }
  return b;
}
J carry_from(const J& input, const J& sced) {
  // Realized state recurrence: southern_execution_contract.md, rolling RATIONALE.
  J carry = J::object();
  for (const auto* table : {"generators", "storage", "reservoirs", "dc_links"}) {
    carry[table] = J::array(); const auto solved = index(sced.at(table));
    for (const auto& r : input.at(table)) {
      const J& s = *solved.at(r.at("id")); J out = {{"id", r.at("id")}};
      if (std::string(table) == "generators") {
        const int on = slot(s, "online", 95) > 0.5 ? 1 : 0;
        int count = 0;
        for (int t = 95; t >= 0 && (slot(s, "online", t) > 0.5 ? 1 : 0) == on; --t) ++count;
        out["initial_on"] = on;
        out["initial_power_mw"] = on ? std::clamp(slot(s, "power_mw", 95), 0.0, slot(r, "pmax_mw", 95)) : 0;
        out["initial_state_minutes"] = count*15 + (count == 96 && r.at("initial_on") == on ? r.at("initial_state_minutes").get<int>() : 0);
      } else if (std::string(table) == "storage") out["initial_mwh"] = std::clamp(slot(s, "energy_mwh", 95), 0.0, r.at("rated_mwh").get<double>());
      else if (std::string(table) == "reservoirs") {
        out["initial_level_m"] = slot(s, "level_m", 95);
        out["initial_release_m3_s"] = std::max(0.0, slot(s, "release_m3_s", 95));
        out["release_history_m3_s"] = J::array();
        for (int t = 0; t < 96; ++t) out["release_history_m3_s"].push_back(std::max(0.0, slot(s, "release_m3_s", t)));
      } else {
        out["initial_mw"] = std::max(0.0, slot(s, "power_mw", 95));
        out["initial_adjustment"] = slot(s, "up", 95) > 0.5 ? 1 : slot(s, "down", 95) > 0.5 ? -1 : 0;
      }
      carry[table].push_back(out);
    }
  }
  return carry;
}
J summarize(const J& result) {
  J day = {{"status", result.at("status")}, {"valid", result.value("schedule_feasible", false)},
    {"diagnostic_prices_valid", result.value("prices_valid", false)}, {"runtime_sec", result.at("runtime_sec")},
    {"validation_sec", result.value("validation_sec",J(nullptr))},
    {"stages", J::object()}, {"periods", J::array()}, {"nodes", J::array()}, {"lines", J::array()}};
  for (const auto* stage : {"scuc", "sced", "lmp"}) if (result.contains(stage)) {
    for (const auto* field : {"solver", "requested_solver", "requested_time_limit_sec", "requested_mip_gap", "requested_threads", "solver_status", "mip_gap", "max_residual", "objective", "optimality_proven", "solution_quality", "limit_reached", "variables", "binary_variables", "runtime_sec", "assembly_sec", "audit_sec", "nonzeros", "formulation", "compact_units", "compact_storage", "projected_commitment_units", "reconstructed_max_residual", "lp_algorithm", "reservoir_scaling", "primal_start", "model_size", "native_diagnostics"})
      day["stages"][stage][field] = result.at(stage).value(field, J(nullptr));
    for (const auto* field : {"solve_wall_sec", "solution_export_sec", "assembly_template", "solver_timing", "price_consistency", "gap_certificate"})
      day["stages"][stage][field] = result.at(stage).value(field, J(nullptr));
  }
  if (!day["valid"].get<bool>()) { day["error"] = result.value("error", std::string("No feasible diagnostic schedule")); return day; }
  const auto& b = result.at("effective_boundary"); const auto& s = result.at("sced");
  // Compact realized-day resource series, authored IDs and original units.
  // Keep D+1 representatives out of displayed execution history (Mixed IEEE118).
  day["resources"] = J::object();
  const std::map<std::string,std::vector<std::string>> resource_fields = {
    {"generators",{"power_mw","online","primary_reserve_mw"}}, {"storage",{"discharge_mw","charge_mw","energy_mwh"}},
    {"reservoirs",{"level_m","spill_m3_s","release_m3_s"}}, {"controllable_loads",{"reduction_mw"}}};
  for(const auto& [table,fields]:resource_fields) {
    day["resources"][table]=J::array();
    for(const auto& source:s.at(table)) {
      J row={{"id",source.at("id")},{"name",source.at("name")}};
      for(const auto& field:fields)row[field]=J(source.at(field).begin(),source.at(field).begin()+96);
      day["resources"][table].push_back(std::move(row));
    }
  }
  // Weekly Plan Results in southern_execution_contract.md: SCED 2.6.3.2--4.
  // Signed reserve contributions are constraint terms, not reserve awards.
  const auto generator_inputs = index(b.at("generators"));
  const auto bus_inputs = index(b.at("buses"));
  for (auto& g : day["resources"]["generators"]) {
    const auto& input = *generator_inputs.at(g.at("id"));
    g["kind"] = input.at("kind"); g["bus"] = input.at("bus");
    g["area"] = bus_inputs.at(input.at("bus"))->at("area");
    const bool renewable = input.at("kind") == "wind" || input.at("kind") == "solar" || input.at("kind") == "renewable";
    for (const auto* field : {"reserve_up_contribution_mw", "reserve_down_contribution_mw", "renewable_available_mw", "curtailment_mw", "utilization_percent"}) g[field] = J::array();
    for (int t = 0; t < 96; ++t) {
      const double p = slot(g,"power_mw",t), u = slot(g,"online",t);
      g["reserve_up_contribution_mw"].push_back((slot(input,"reserve_up_eligible",t) != 0 ? (slot(input,"pmax_mw",t)-slot(input,"regulation_up_mw",t))*u : 0)-p);
      g["reserve_down_contribution_mw"].push_back(p-(slot(input,"reserve_down_eligible",t) != 0 ? (slot(input,"pmin_mw",t)+slot(input,"regulation_down_mw",t))*u : 0));
      const double available = renewable ? std::min(slot(input,"pmax_mw",t),slot(input,"forecast_mw",t))*slot(input,"available",t)*(1-slot(input,"must_off",t)) : 0;
      g["renewable_available_mw"].push_back(renewable ? J(available) : J(nullptr));
      g["curtailment_mw"].push_back(renewable ? J(std::max(0.0,available-p)) : J(nullptr));
      g["utilization_percent"].push_back(available > 0 ? J(100*p/available) : J(nullptr));
    }
  }
  day["resources"]["areas"] = J::array();
  const auto storage_inputs = index(b.at("storage"));
  for (const auto& input : b.at("areas")) {
    J a = {{"id",input.at("id")},{"name",input.at("name")}};
    for (const auto* field : {"reserve_up_mw","reserve_down_mw","reserve_up_required_mw","reserve_down_required_mw","reserve_up_margin_mw","reserve_down_margin_mw"}) a[field] = J::array();
    for (int t = 0; t < 96; ++t) {
      double up = 0, down = 0;
      for (const auto& g : day["resources"]["generators"]) if (g.at("area") == input.at("id")) {
        up += slot(g,"reserve_up_contribution_mw",t); down += slot(g,"reserve_down_contribution_mw",t);
      }
      for (const auto& storage : day["resources"]["storage"]) if (bus_inputs.at(storage_inputs.at(storage.at("id"))->at("bus"))->at("area") == input.at("id")) {
        const double net = slot(storage,"discharge_mw",t)+slot(storage,"charge_mw",t);
        up -= net; down += net;
      }
      const double up_required = slot(input,"reserve_up_mw",t)+slot(input,"network_reserve_reduction_mw",t);
      const double down_required = slot(input,"reserve_down_mw",t)-slot(input,"load_side_down_reserve_mw",t);
      a["reserve_up_mw"].push_back(up); a["reserve_down_mw"].push_back(down);
      a["reserve_up_required_mw"].push_back(up_required); a["reserve_down_required_mw"].push_back(down_required);
      a["reserve_up_margin_mw"].push_back(up-up_required); a["reserve_down_margin_mw"].push_back(down-down_required);
    }
    day["resources"]["areas"].push_back(std::move(a));
  }
  day["constraint_families"] = s.value("constraint_families", J::object());
  day["binding_constraints"] = s.value("binding_constraints", J::array());
  day["binding_constraints_truncated"] = s.value("binding_constraints_truncated", false);
  const auto price = result.value("prices_valid", false) ? index(result.at("lmp").at("buses")) : std::map<int,const J*>{};
  const auto inputs = index(b.at("buses"));
  for (const auto& n : s.at("buses")) {
    J row = {{"id", n.at("id")}, {"name", n.at("name")}};
    for (const auto* f : {"deficit_mw", "surplus_mw", "node_imbalance_mw"}) row[f] = J(n.at(f).begin(), n.at(f).begin()+96);
    row["load_mw"] = J(inputs.at(n.at("id"))->at("load_mw").begin(), inputs.at(n.at("id"))->at("load_mw").begin()+96);
    row["delta_p_mw"] = J::array();
    for (int t = 0; t < 96; ++t) row["delta_p_mw"].push_back(slot(n, "deficit_mw", t)-slot(n, "surplus_mw", t));
    row["lmp_per_mwh"] = price.empty() ? J(nullptr) : J(price.at(n.at("id"))->at("lmp_per_mwh").begin(), price.at(n.at("id"))->at("lmp_per_mwh").begin()+96);
    day["nodes"].push_back(row);
  }
  const auto limits = index(b.at("branches"));
  for (const auto& l : s.at("branches")) {
    J row = {{"id", l.at("id")}, {"name", l.at("name")}};
    for (const auto* f : {"power_mw", "overload_mw"}) row[f] = J(l.at(f).begin(), l.at(f).begin()+96);
    row["delta_pij_mw"] = row["overload_mw"];
    const auto& authored = *limits.at(l.at("id"));
    for (const auto* f : {"from_bus", "to_bus"}) row[f] = authored.at(f);
    for (const auto* f : {"min_mw", "max_mw", "available"}) row[f] = J(authored.at(f).begin(), authored.at(f).begin()+96);
    row["loading_percent"] = J::array();
    for (int t = 0; t < 96; ++t) {
      const double p = slot(l,"power_mw",t), lo = slot(authored,"min_mw",t), hi = slot(authored,"max_mw",t);
      const double limit = p >= 0 ? hi : -lo;
      // Directional active-power utilization; not AC apparent-power loading.
      row["loading_percent"].push_back(slot(authored,"available",t) != 0 && lo <= 0 && hi >= 0 && limit > 0 ? J(100*std::abs(p)/limit) : J(nullptr));
    }
    day["lines"].push_back(row);
  }
  double energy = 0, surplus_energy = 0, overload_energy = 0;
  for (int t = 0; t < 96; ++t) {
    // MW * 0.25 h; network excess is summed across lines, not unserved energy.
    double deficit = 0, surplus = 0, overload = 0, maximum = 0, load = 0, capacity = 0;
    for (const auto& n : day["nodes"]) { deficit += slot(n, "deficit_mw", t); surplus += slot(n, "surplus_mw", t); }
    for (const auto& l : day["lines"]) { overload += slot(l, "overload_mw", t); maximum = std::max(maximum, slot(l, "overload_mw", t)); }
    for (const auto& n : b.at("buses")) load += slot(n, "load_mw", t);
    for (const auto& g : b.at("generators")) {
      double max = slot(g, "pmax_mw", t);
      if (g.at("kind") == "wind" || g.at("kind") == "solar" || g.at("kind") == "renewable") max = std::min(max, slot(g, "forecast_mw", t));
      capacity += max*slot(g, "available", t)*(1-slot(g, "must_off", t));
    }
    day["periods"].push_back({{"slot", t}, {"deficit_mw", deficit}, {"surplus_mw", surplus},
      {"overload_sum_mw", overload}, {"max_line_overload_mw", maximum}, {"load_mw", load}, {"available_generation_mw", capacity}});
    energy += deficit*0.25; surplus_energy += surplus*0.25; overload_energy += overload*0.25;
  }
  day["deficit_mwh"] = energy; day["surplus_mwh"] = surplus_energy; day["overload_mwh"] = overload_energy;
  day["state_end"] = carry_from(b, s);
  return day;
}
} // namespace

J make_market_operation(const J& boundary, const J& config) {
  keys(config, {"horizon", "start_date", "penalty_per_mwh", "explain", "explain_trigger", "recovery_pricing", "days", "reference_days", "terminal_forecast_source", "solver_options", "posthoc_ac_audit"});
  const auto trigger = config.value("explain_trigger", std::string("always"));
  require(trigger == "anomaly" || trigger == "always" || trigger == "manual", "invalid explain_trigger");
  const auto pricing = config.value("recovery_pricing", std::string("dispatch_only"));
  require(pricing == "dispatch_only" || pricing == "full", "invalid recovery_pricing");
  require(!config.contains("posthoc_ac_audit") || config.at("posthoc_ac_audit").is_boolean(), "posthoc_ac_audit must be boolean");
  validate_southern_market(boundary);
  // Preserve raw bus proportions until day-specific 2.4.1.2 reconciliation.
  auto base = boundary;
  if (config.contains("solver_options")) {
    keys(config.at("solver_options"),{"solver","time_limit_sec","mip_gap","threads","native_root_cuts"});
    base["execution"].update(config.at("solver_options"));
    validate_southern_market(base);
  }
  if (!base.contains("controllable_loads")) base["controllable_loads"] = J::array();
  require(config.at("start_date").is_string(), "start_date must be a string");
  const std::string date = config.at("start_date");
  require(date.size() == 10 && date[4] == '-' && date[7] == '-', "date must be YYYY-MM-DD");
  for (size_t i = 0; i < date.size(); ++i) if (i != 4 && i != 7) require(date[i] >= '0' && date[i] <= '9', "invalid date");
  using namespace std::chrono;
  const year_month_day start{year{std::stoi(date.substr(0,4))}, month{static_cast<unsigned>(std::stoi(date.substr(5,2)))}, day{static_cast<unsigned>(std::stoi(date.substr(8,2)))}};
  require(start.ok(), "invalid calendar date");
  require(config.at("horizon") == "day" || config.at("horizon") == "week" || config.at("horizon") == "month", "horizon must be day, week or month");
  int count = config.at("horizon") == "day" ? 1 : 7;
  if (config.at("horizon") == "month") {
    require(unsigned(start.day()) == 1, "month starts on day 1");
    count = unsigned(year_month_day_last{start.year(), month_day_last{start.month()}}.day());
  }
  number(config, "penalty_per_mwh", 1, 1e7);
  require(config.at("explain").is_boolean(), "explain must be boolean");
  require(config.at("days").is_array() && (config.at("days").empty() || config.at("days").size() == static_cast<size_t>(count) || config.at("days").size() == static_cast<size_t>(count+1)), "daily boundaries require calendar length plus one forecast day");
  for (const auto& g : base.at("generators")) {
    require(g.at("shutdown_curve_mw").empty(), "cross-day shutdown trajectory unsupported in rolling mode");
    for (const auto& curve : g.at("startup_curves_mw")) require(curve.empty(), "cross-day startup trajectory unsupported in rolling mode");
  }
  J normalized = config;
  normalized["explain_trigger"] = trigger;
  normalized["recovery_pricing"] = pricing;
  normalized["solver_options"] = {{"solver",base["execution"].value("solver",std::string("highs"))},
    {"threads",base["execution"].value("threads",0)}, {"time_limit_sec",base["execution"]["time_limit_sec"]}, {"mip_gap",base["execution"]["mip_gap"]},
    {"native_root_cuts",base["execution"].value("native_root_cuts",std::string("default"))}};
  normalized["terminal_forecast_source"] = config.at("days").size() == static_cast<size_t>(count+1) ? config.value("terminal_forecast_source", std::string("authored")) : "baseline_template";
  require(normalized["terminal_forecast_source"] == "authored" || normalized["terminal_forecast_source"] == "baseline_template", "invalid terminal forecast source");
  normalized["days"] = J::array();
  for (int d = 0; d <= count; ++d) {
    J change = {{"first_slot", 0}, {"last_slot", 95}, {"generator_outages", J::array()}, {"branch_outages", J::array()}};
    for (const auto& f : factors) change[f] = 1.0;
    if (static_cast<size_t>(d) < config.at("days").size()) {
      keys(config.at("days")[d], {"first_slot", "last_slot", "generator_outages", "branch_outages", "load_scale", "wind_scale", "solar_scale", "inflow_scale", "bid_scale", "line_limit_scale", "generator_bid_scale", "load_bid_scale", "boundary_overrides"});
      change.update(config.at("days")[d]);
    }
    for (const auto& f : factors) number(change, f.c_str(), 0, 10);
    require(change["first_slot"].is_number_integer() && change["last_slot"].is_number_integer(), "slots must be integers");
    number(change, "first_slot", 0, 95); number(change, "last_slot", 0, 95);
    require(change["first_slot"] <= change["last_slot"], "reversed slot interval");
    for (const auto& [table, field] : {std::pair{"generators", "generator_outages"}, std::pair{"branches", "branch_outages"}}) {
      const auto entities = index(base.at(table)); std::set<int> seen;
      require(change[field].is_array(), std::string(field)+" must be array");
      for (const auto& id : change[field]) { require(id.is_number_integer(), "outage ID must be integer"); require(entities.count(id.get<int>()) && seen.insert(id.get<int>()).second, "unknown or duplicate outage ID"); }
    }
    if (change.contains("boundary_overrides")) {
      const auto catalog = southern_market_boundary_catalog();
      std::map<std::pair<std::string,std::string>,J> permitted;
      for (const auto& group : catalog.at("items")) for (const auto& field : group.at("fields"))
        permitted[{field.at("table"),field.at("field")}] = field.at("schema");
      const auto& edits = change.at("boundary_overrides");
      require(edits.is_array() && edits.size() <= 10000, "boundary_overrides must be an array of at most 10000 edits");
      std::set<std::tuple<std::string,int,std::string,int>> occupied;
      for (const auto& e : edits) {
        keys(e,{"table","id","field","first_slot","last_slot","value","reason"});
        require(e.at("table").is_string() && e.at("field").is_string() && e.at("id").is_number_integer(), "typed stable override identity required");
        number(e,"id",0,1000000);
        const std::string table = e.at("table"), field = e.at("field");
        require(permitted.count({table,field}), "field is not an admitted operating boundary: " + table + "." + field);
        require(index(base.at(table)).count(e.at("id").get<int>()), "unknown boundary override ID");
        require(e.at("reason").is_string() && !e.at("reason").get_ref<const std::string&>().empty() && e.at("reason").get_ref<const std::string&>().size() <= 2048, "override reason required (max 2048 bytes)");
        require(e.at("first_slot").is_number_integer() && e.at("last_slot").is_number_integer(), "override slots must be integers");
        number(e,"first_slot",0,97); number(e,"last_slot",0,97);
        require(e.at("first_slot") <= e.at("last_slot"), "reversed override interval");
        const auto schema = permitted.at({table,field}); const bool series = schema.at("type") == "array";
        require(d < count || series, "forecast-only terminal day accepts time-series overrides only; scalar parameters belong to realized day windows");
        require(!series || e.at("last_slot").get<int>() <= 95, "rolling series edits address 0..95; edit the following forecast day for representative points");
        require(series || (e.at("first_slot") == 0 && e.at("last_slot") == 97), "scalar boundary applies to the entire day window (0..97)");
        const auto scalar = series ? schema.at("items") : schema;
        if (scalar.at("type") == "string") require(e.at("value").is_string(), "override value must be text");
        else { require(scalar.at("type") != "integer" || e.at("value").is_number_integer(), "override value must be integer"); number(e,"value",scalar.at("minimum"),scalar.at("maximum")); }
        for (int t = e.at("first_slot"); t <= e.at("last_slot").get<int>(); ++t)
          require(occupied.emplace(table,e.at("id").get<int>(),field,t).second, "overlapping boundary overrides");
      }
    }
    normalized["days"].push_back(change);
  }
  if (config.contains("reference_days")) {
    J reference = config; reference.erase("reference_days"); reference["days"] = config.at("reference_days");
    normalized["reference_days"] = make_market_operation(base, reference).at("config").at("days");
  }
  J carry = J::object();
  for (const auto* table : {"generators", "storage", "reservoirs", "dc_links"}) {
    carry[table] = J::array();
    for (const auto& row : base.at(table)) {
      J state = {{"id", row.at("id")}};
      for (const auto* field : {"initial_on", "initial_power_mw", "initial_state_minutes", "initial_mwh", "initial_level_m", "initial_release_m3_s", "release_history_m3_s", "initial_mw", "initial_adjustment"})
        if (row.contains(field)) state[field] = row.at(field);
      carry[table].push_back(state);
    }
  }
  for (const auto& change : normalized.at("days")) if (change.contains("boundary_overrides") && !change.at("boundary_overrides").empty())
    validate_southern_market(apply(base,carry,change,config.at("penalty_per_mwh")));
  return {{"status", "ready"}, {"base", base}, {"config", normalized}, {"carry", carry}, {"days", J::array()},
    {"total_days", count}, {"forecast_days", count+1}, {"completed_days", 0}, {"diagnostic", true},
    {"model_scope", "rolling-daily-98-slot-diagnostic:next-day-extrema:realized-first-96:state-carry:daily-authored-SOC-target"},
    {"limitations", {"逐日滚动，不是全周/月联合最优；次日统调负荷峰谷时刻同步提取所有98点时序字段。", "末日预测来源：" + normalized.at("terminal_forecast_source").get<std::string>() + "；baseline_template 表示显式基准补齐，非外部预测。", "单值物理参数、发电/储能报价及日终目标属于当天日窗，尚不支持尾部两点独立单值报价或参数。代表点间隙不积分未知水量与能量。", "缺额/富余为研究罚变量；电价受罚价影响；未进行交流安全认证。", "容量证据未扣除全部水量、爬坡与网络限制，不能单独判断充分供电。", "配对恢复只改变当日因素，固定次日预测及当日日初状态；差值为条件敏感性，不代表唯一原因。", "限时可行解的正松弛不证明异常不可避免；失败不按零异常统计。"}}};
}

J preview_market_operation_boundary(const J& boundary, const J& config, int day) {
  const auto job = make_market_operation(boundary,config);
  require(day >= 0 && day <= job.at("total_days").get<int>(), "preview day outside forecast calendar");
  J provenance = nullptr;
  const auto& days = job.at("config").at("days");
  const auto input = day == job.at("total_days").get<int>() ? apply(job.at("base"),job.at("carry"),days[day],config.at("penalty_per_mwh")) :
    daily_window(job.at("base"),job.at("carry"),days[day],days[day+1],config.at("penalty_per_mwh"),provenance);
  if (!provenance.is_null()) provenance["source_day"] = day+1;
  return {{"authored",input},{"effective",validate_southern_market(input)},
    {"day",day},{"lookahead",provenance},{"forecast_only",day == job.at("total_days").get<int>()},
    {"state_scope","Boundary preview uses authored initial state; daily solving replaces it with chronological carry. Forecast-only preview uses only first 96 points."}};
}

static void recover_day(const J& job, J& day, int d, const std::string& pricing) {
      require(pricing == "dispatch_only" || pricing == "full", "invalid recovery pricing");
      const auto& change = job.at("config").at("days").at(d);
      const auto& next = job.at("config").at("days").at(d+1);
      const double penalty = job.at("config").at("penalty_per_mwh");
      day["counterfactuals"] = J::array();
      auto interventions = factors;
      interventions.push_back("generator_outages"); interventions.push_back("branch_outages");
      if (change.contains("boundary_overrides")) interventions.push_back("boundary_overrides");
      std::vector<std::tuple<std::string,J,J>> experiments;
      for (const auto& factor : interventions) {
        const bool outage = factor == "generator_outages" || factor == "branch_outages" || factor == "boundary_overrides";
        const J reference = job.at("config").contains("reference_days") ? job.at("config").at("reference_days")[d].value(factor,outage ? J::array() : J(1.0)) : outage ? J::array() : J(1.0);
        const J sampled = change.value(factor, outage ? J::array() : J(1.0));
        if (sampled != reference) experiments.emplace_back(factor,reference,sampled);
      }
      // Independent potential outcomes share the same pre-intervention state;
      // never parallelize chronological days. Bounded Gurobi environments,
      // Amdahl/memory budget: docs/modules/market/performance.md, Paired Recovery.
      const auto& execution = job.at("base").at("execution");
      const int requested_threads = execution.value("threads",0);
      const unsigned cores = std::thread::hardware_concurrency();
      const int per_solve_threads = requested_threads == 0 ? 2 : requested_threads;
      const bool parallel = execution.value("solver",std::string("highs")) == "gurobi" &&
        experiments.size()>1 && job.at("base").at("buses").size()<=118 &&
        job.at("base").at("generators").size()<=128 && job.at("base").at("storage").size()<=16 &&
        job.at("base").at("reservoirs").size()<=24 &&
        cores>=4 && per_solve_threads<=static_cast<int>(cores/2);
      // Worker count: min(6, experiments, cores/2). RATIONALE: measured 2-worker
      // efficiency 96.5% (performance.md, 恢复并发扩展与定价并发双解), so the 2-worker
      // cap is the binding constraint; per-day waves of 6 match the 6 intervention
      // factors. cores>=4 keeps the small-machine fallback at 2; the model-size caps
      // above bound each concurrent MILP/JSON footprint (measured peak RSS 2.28 GiB
      // at 2 workers, ~+0.3-0.5 GiB per additional worker) in place of a portable
      // physical-memory probe. Each solve keeps per_solve_threads, so peak solver
      // threads are workers*per_solve_threads <= cores.
      const size_t workers = parallel ? std::min({static_cast<size_t>(6),experiments.size(),
        static_cast<size_t>(cores/2)}) : 1;
      const auto recovery_start = std::chrono::steady_clock::now();
      day["recovery_execution"] = {{"workers",workers},{"experiments",experiments.size()},
        {"requested_solver_threads",requested_threads},{"resolved_solver_threads",parallel ? per_solve_threads : requested_threads},
        {"worker_cap",6},{"logical_cpus",cores},
        {"worker_admission",parallel ? "min(6, experiments, logical_cpus/2); size caps <=118 buses/<=128 generators/<=16 storage/<=24 reservoirs"
                                     : "sequential: non-gurobi solver, single experiment, size cap or thread budget not met"},
        {"scope","independent paired interventions; chronological days remain sequential"}};
      const J baseline_totals = {{"deficit_mwh",day.at("deficit_mwh")},
        {"surplus_mwh",day.at("surplus_mwh")},{"overload_mwh",day.at("overload_mwh")}};
      const auto evaluate = [&](size_t index) {
        const auto& [factor,reference,sampled] = experiments[index];
        J restored = change; restored[factor] = reference;
        J proof = {{"factor", factor}, {"valid", false}, {"reference_value", reference}, {"sampled_value", sampled},
          {"pricing_scope",pricing},{"prices_valid",false}};
        try {
          J restored_provenance;
          const auto boundary_start = std::chrono::steady_clock::now();
          auto restored_input = daily_window(job.at("base"),day.at("state_start"),restored,next,penalty,restored_provenance);
          proof["boundary_sec"] = std::chrono::duration<double>(std::chrono::steady_clock::now()-boundary_start).count();
          if(parallel) restored_input["execution"]["threads"] = per_solve_threads;
          auto other = summarize(pricing == "full" ? run_southern_day_ahead_market(restored_input)
                                                     : run_southern_dispatch_recovery(restored_input));
          proof["pricing_scope"] = pricing;
          proof["prices_valid"] = other.at("diagnostic_prices_valid");
          proof["runtime_sec"] = other.at("runtime_sec");
          proof["validation_sec"] = other.at("validation_sec");
          proof["stages"] = other.at("stages");
          proof["status"] = other["status"]; proof["valid"] = other["valid"];
          if (other["valid"].get<bool>()) {
            for (const auto* field : {"deficit_mwh", "surplus_mwh", "overload_mwh"}) proof[std::string("reduction_")+field] = baseline_totals.at(field).get<double>()-other[field].get<double>();
            proof["periods"] = other["periods"];
          }
        } catch (const std::invalid_argument& e) { proof["error"] = e.what(); }
        catch (const std::exception& e) { proof["error"] = e.what(); }
        return proof;
      };
      // Waves of `workers` async experiments; results are collected in factor
      // order within each wave and waves run sequentially, so the emitted
      // counterfactual order is identical to the fully sequential loop.
      for(size_t first=0;first<experiments.size();first+=workers) {
        const size_t wave=std::min(workers,experiments.size()-first);
        if(wave==1){day["counterfactuals"].push_back(evaluate(first));continue;}
        std::vector<std::future<J>> pending;
        pending.reserve(wave-1);
        for(size_t k=1;k<wave;++k)pending.push_back(std::async(std::launch::async,evaluate,first+k));
        day["counterfactuals"].push_back(evaluate(first));
        for(auto& f:pending)day["counterfactuals"].push_back(f.get());
      }
      day["recovery_execution"]["wall_sec"] = std::chrono::duration<double>(std::chrono::steady_clock::now()-recovery_start).count();
      const bool all_valid = std::all_of(day.at("counterfactuals").begin(), day.at("counterfactuals").end(),
        [](const J& proof) { return proof.at("valid").get<bool>(); });
      day["cause_analysis"]["status"] = experiments.empty() ? "no_interventions" : all_valid ? "completed" : "partial_failure";
      day["cause_analysis"]["executed_experiments"] = experiments.size();
      day["cause_analysis"]["pricing_scope"] = pricing;
}

J explain_market_operation_day(const J& previous, int d, const std::string& pricing) {
  require(d >= 0 && d < previous.at("completed_days").get<int>(), "explanation day must be completed");
  require(previous.at("status") != "stale" && previous.at("status") != "cancelled", "job cannot be explained");
  J job = previous;
  auto& day = job["days"][d];
  require(day.at("valid").get<bool>(), "explanation requires valid SCED");
  const J original_execution = day.value("recovery_execution", J(nullptr));
  day["cause_analysis"]["requested_by"] = "manual";
  recover_day(job, day, d, pricing);
  day["cause_analysis"]["manual_wall_sec"] = day.at("recovery_execution").at("wall_sec");
  day["manual_recovery_execution"] = day.at("recovery_execution");
  if (original_execution.is_null()) day.erase("recovery_execution");
  else day["recovery_execution"] = original_execution;
  return job;
}

J step_market_operation(const J& previous) {
  const auto day_start = std::chrono::steady_clock::now();
  J job = previous;
  require(job["status"] == "ready" || job["status"] == "running", "job cannot advance");
  const int d = job.at("completed_days"); const auto change = job.at("config").at("days")[d];
  try {
    const double penalty = job["config"]["penalty_per_mwh"];
    J provenance;
    const auto& next = job.at("config").at("days").at(d+1);
    const auto boundary_start = std::chrono::steady_clock::now();
    const J input = daily_window(job.at("base"), job.at("carry"), change, next, penalty, provenance);
    const double boundary_sec = std::chrono::duration<double>(std::chrono::steady_clock::now()-boundary_start).count();
    provenance["source_day"] = d+1;
    const J result = run_southern_day_ahead_market(input);
    const auto summary_start = std::chrono::steady_clock::now();
    J day = summarize(result); day["day"] = d; day["boundary"] = change; day["lookahead"] = provenance;
    const auto analysis_start = std::chrono::steady_clock::now();
    if (day["valid"].get<bool>()) day["analysis"] = analyze_southern_market_result(result, job.at("config").value("posthoc_ac_audit", false));
    const double analysis_sec = std::chrono::duration<double>(std::chrono::steady_clock::now()-analysis_start).count();
    day["state_start"] = job.at("carry"); day["counterfactuals"] = J::array();
    const auto trigger = job.at("config").value("explain_trigger", std::string("always"));
    // Same realized-day MW threshold as the diagnostic UI; performance.md.
    const bool anomaly = std::any_of(day.at("periods").begin(), day.at("periods").end(), [](const J& p) {
      return p.at("deficit_mw").get<double>() > 1e-6 || p.at("surplus_mw").get<double>() > 1e-6 ||
             p.at("overload_sum_mw").get<double>() > 1e-6;
    });
    const bool requested = job.at("config").at("explain").get<bool>() && trigger != "manual";
    day["cause_analysis"] = {{"status", !day.at("valid").get<bool>() ? "unavailable" : !requested ? "not_requested" : "not_triggered"},
      {"trigger",trigger},{"requested_by","automatic"},{"anomaly_detected",day.at("valid").get<bool>() ? J(anomaly) : J(nullptr)},
      {"threshold_mw",1e-6},{"executed_experiments",0}};
    if (day.at("valid").get<bool>() && requested && (trigger == "always" || anomaly))
      recover_day(job, day, d, job.at("config").value("recovery_pricing",std::string("dispatch_only")));
    day["execution_timing"] = {{"main_sec",result.at("runtime_sec")},
      {"boundary_sec",boundary_sec},{"validation_sec",result.at("validation_sec")},
      {"summary_sec",std::chrono::duration<double>(analysis_start-summary_start).count()},{"analysis_sec",analysis_sec},
      {"recovery_wall_sec",day.contains("recovery_execution") ? day.at("recovery_execution").at("wall_sec") : J(0)},
      {"day_wall_sec",std::chrono::duration<double>(std::chrono::steady_clock::now()-day_start).count()}};
    job["days"].push_back(day);
    if (!day["valid"].get<bool>()) { job["status"] = "failed"; return job; }
    job["carry"] = day["state_end"];
    job["completed_days"] = d+1;
    job["status"] = d+1 == job["total_days"].get<int>() ? "completed" : "running";
  } catch (const std::invalid_argument& e) { job["status"] = "failed"; job["error"] = e.what(); }
  catch (const std::exception& e) { job["status"] = "failed"; job["error"] = e.what(); }
  return job;
}
} // namespace hacdcpf::market
