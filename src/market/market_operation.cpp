#include "hacdcpf/market/southern_market.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace hacdcpf::market {
namespace {
using J = nlohmann::json;
const std::vector<std::string> factors = {"load_scale", "wind_scale", "solar_scale", "inflow_scale", "bid_scale", "line_limit_scale"};
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
    for (auto& s : g["segments"]) s["price_per_mwh"] = s["price_per_mwh"].get<double>()*change.at("bid_scale").get<double>();
  }
  for (auto& r : b["storage"]) for (const auto* f : {"discharge_price", "charge_price"}) r[f] = r[f].get<double>()*change.at("bid_scale").get<double>();
  for (auto& r : b["controllable_loads"]) for (auto& v : r["compensation_per_mwh"]) v = v.get<double>()*change.at("bid_scale").get<double>();
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
J carry_from(const J& input, const J& sced) {
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
    {"stages", J::object()}, {"periods", J::array()}, {"nodes", J::array()}, {"lines", J::array()}};
  for (const auto* stage : {"scuc", "sced", "lmp"}) if (result.contains(stage)) {
    for (const auto* field : {"solver_status", "mip_gap", "max_residual", "objective", "optimality_proven"})
      day["stages"][stage][field] = result.at(stage).value(field, J(nullptr));
  }
  if (!day["valid"].get<bool>()) { day["error"] = result.value("error", std::string("No feasible diagnostic schedule")); return day; }
  const auto& b = result.at("effective_boundary"); const auto& s = result.at("sced");
  const auto price = result.value("prices_valid", false) ? index(result.at("lmp").at("buses")) : std::map<int,const J*>{};
  for (const auto& n : s.at("buses")) {
    J row = {{"id", n.at("id")}, {"name", n.at("name")}};
    for (const auto* f : {"deficit_mw", "surplus_mw", "node_imbalance_mw"}) row[f] = J(n.at(f).begin(), n.at(f).begin()+96);
    row["lmp_per_mwh"] = price.empty() ? J(nullptr) : J(price.at(n.at("id"))->at("lmp_per_mwh").begin(), price.at(n.at("id"))->at("lmp_per_mwh").begin()+96);
    day["nodes"].push_back(row);
  }
  const auto limits = index(b.at("branches"));
  for (const auto& l : s.at("branches")) {
    J row = {{"id", l.at("id")}, {"name", l.at("name")}};
    for (const auto* f : {"power_mw", "overload_mw"}) row[f] = J(l.at(f).begin(), l.at(f).begin()+96);
    const auto& authored = *limits.at(l.at("id"));
    for (const auto* f : {"from_bus", "to_bus"}) row[f] = authored.at(f);
    for (const auto* f : {"min_mw", "max_mw", "available"}) row[f] = J(authored.at(f).begin(), authored.at(f).begin()+96);
    day["lines"].push_back(row);
  }
  double energy = 0, surplus_energy = 0, overload_energy = 0;
  for (int t = 0; t < 96; ++t) {
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
  keys(config, {"horizon", "start_date", "penalty_per_mwh", "explain", "days"});
  const auto base = validate_southern_market(boundary);
  require(config.at("start_date").is_string(), "start_date must be a string");
  const std::string date = config.at("start_date");
  require(date.size() == 10 && date[4] == '-' && date[7] == '-', "date must be YYYY-MM-DD");
  for (size_t i = 0; i < date.size(); ++i) if (i != 4 && i != 7) require(date[i] >= '0' && date[i] <= '9', "invalid date");
  using namespace std::chrono;
  const year_month_day start{year{std::stoi(date.substr(0,4))}, month{static_cast<unsigned>(std::stoi(date.substr(5,2)))}, day{static_cast<unsigned>(std::stoi(date.substr(8,2)))}};
  require(start.ok(), "invalid calendar date");
  require(config.at("horizon") == "week" || config.at("horizon") == "month", "horizon must be week or month");
  int count = 7;
  if (config.at("horizon") == "month") {
    require(unsigned(start.day()) == 1, "month starts on day 1");
    count = unsigned(year_month_day_last{start.year(), month_day_last{start.month()}}.day());
  }
  number(config, "penalty_per_mwh", 1, 1e7);
  require(config.at("explain").is_boolean(), "explain must be boolean");
  require(config.at("days").is_array() && (config.at("days").empty() || config.at("days").size() == static_cast<size_t>(count)), "daily boundaries must match calendar length");
  for (const auto& g : base.at("generators")) {
    require(g.at("shutdown_curve_mw").empty(), "cross-day shutdown trajectory unsupported in rolling mode");
    for (const auto& curve : g.at("startup_curves_mw")) require(curve.empty(), "cross-day startup trajectory unsupported in rolling mode");
  }
  J normalized = config;
  normalized["days"] = J::array();
  for (int d = 0; d < count; ++d) {
    J change = {{"first_slot", 0}, {"last_slot", 95}, {"generator_outages", J::array()}, {"branch_outages", J::array()}};
    for (const auto& f : factors) change[f] = 1.0;
    if (!config.at("days").empty()) {
      keys(config.at("days")[d], {"first_slot", "last_slot", "generator_outages", "branch_outages", "load_scale", "wind_scale", "solar_scale", "inflow_scale", "bid_scale", "line_limit_scale"});
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
    normalized["days"].push_back(change);
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
  return {{"status", "ready"}, {"base", base}, {"config", normalized}, {"carry", carry}, {"days", J::array()},
    {"total_days", count}, {"completed_days", 0}, {"diagnostic", true},
    {"model_scope", "rolling-daily-98-slot-diagnostic:realized-first-96:state-carry:daily-authored-SOC-target"},
    {"limitations", {"逐日滚动，不是全周/月联合最优；模板每日复用，指定日/时段覆盖。", "缺额/富余为研究罚变量；电价受罚价影响；未进行交流安全认证。", "容量证据未扣除全部水量、爬坡与网络限制，不能单独判断充分供电。", "配对恢复只改变当日因素，固定当日日初状态；差值为条件敏感性，不代表唯一原因。"}}};
}

J step_market_operation(const J& previous) {
  J job = previous;
  require(job["status"] == "ready" || job["status"] == "running", "job cannot advance");
  const int d = job.at("completed_days"); const auto change = job.at("config").at("days")[d];
  try {
    const double penalty = job["config"]["penalty_per_mwh"];
    const J input = apply(job.at("base"), job.at("carry"), change, penalty);
    const J result = run_southern_day_ahead_market(input);
    J day = summarize(result); day["day"] = d; day["boundary"] = change;
    day["state_start"] = job.at("carry"); day["counterfactuals"] = J::array();
    if (day["valid"].get<bool>() && job["config"]["explain"].get<bool>()) {
      auto interventions = factors;
      interventions.push_back("generator_outages"); interventions.push_back("branch_outages");
      for (const auto& factor : interventions) {
        const bool outage = factor == "generator_outages" || factor == "branch_outages";
        if (outage ? change.at(factor).empty() : change.at(factor) == 1.0) continue;
        J restored = change; restored[factor] = outage ? J::array() : J(1.0);
        J proof = {{"factor", factor}, {"valid", false}};
        try {
          auto other = summarize(run_southern_day_ahead_market(apply(job.at("base"), job.at("carry"), restored, penalty)));
          proof["status"] = other["status"]; proof["valid"] = other["valid"];
          if (other["valid"].get<bool>()) {
            for (const auto* field : {"deficit_mwh", "surplus_mwh", "overload_mwh"}) proof[std::string("reduction_")+field] = day[field].get<double>()-other[field].get<double>();
            proof["periods"] = other["periods"];
          }
        } catch (const std::exception& e) { proof["error"] = e.what(); }
        day["counterfactuals"].push_back(proof);
      }
    }
    job["days"].push_back(day);
    if (!day["valid"].get<bool>()) { job["status"] = "failed"; return job; }
    job["carry"] = day["state_end"];
    job["completed_days"] = d+1;
    job["status"] = d+1 == job["total_days"].get<int>() ? "completed" : "running";
  } catch (const std::exception& e) { job["status"] = "failed"; job["error"] = e.what(); }
  return job;
}
} // namespace hacdcpf::market
