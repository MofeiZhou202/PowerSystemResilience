#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <numeric>
#include <random>
#include "hacdcpf/market/southern_market.hpp"
#include "hacdcpf/io/matpower_parser.hpp"

using nlohmann::json;
using namespace hacdcpf::market;
using Catch::Approx;

TEST_CASE("Southern GUI factories expose valid named research boundaries", "[southern_market]") {
  const auto demo = make_southern_market_demo();
  REQUIRE(demo["buses"].size() == 2);
  REQUIRE(demo["generators"].size() == 8);
  REQUIRE(demo["reservoirs"].size() == 1);
  REQUIRE(demo["reservoirs"][0]["generators"].size() == 4);
  REQUIRE(demo["storage"].size() == 2);
  const auto result = run_southern_day_ahead_market(demo);
  INFO(result.value("status", ""));
  INFO((result.contains("lmp") ? result["lmp"].dump().substr(0, 4000) : result.dump().substr(0, 1000)));
  REQUIRE(result["schedule_feasible"] == true);
  REQUIRE(result["prices_valid"] == true);
  auto system = hacdcpf::io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR)+"/case_ACTIVSg2000.m");
  const auto imported = southern_market_from_system(system);
  REQUIRE(imported["generators"].size() == system.ac.generators.size());
  REQUIRE(imported["reservoirs"].empty());
  const auto large = southern_market_from_system(system, true);
  REQUIRE(large["buses"].size() == 2000);
  REQUIRE(large["branches"].size() == 3206);
  REQUIRE(large["generators"].size() == 1744);
  REQUIRE(large["reservoirs"].size() == 180);
  REQUIRE(large["storage"].size() == 80);
  REQUIRE(large["controllable_loads"].size() == 120);
}

namespace {
json series(double x) { return std::vector<double>(98, x); }
json empty_record(const json& schema) {
  if (schema.contains("enum")) return schema.at("enum")[0];
  const std::string type = schema.at("type");
  if (type == "object") {
    json result = json::object();
    for (auto it = schema.at("properties").begin(); it != schema.at("properties").end(); ++it) result[it.key()] = empty_record(it.value());
    return result;
  }
  if (type == "array") {
    json result = json::array();
    for (int i = 0; i < schema.at("minItems").get<int>(); ++i) result.push_back(empty_record(schema.at("items")));
    return result;
  }
  if (type == "string") return "synthetic-test";
  if (type == "integer") return std::max(0, schema.at("minimum").get<int>());
  return std::max(0.0, schema.at("minimum").get<double>());
}
json record(const char* table) {
  auto row = empty_record(southern_market_schema().at("properties").at(table).at("items"));
  row["id"] = 10; return row;
}
json two_bus() {
  auto j = make_southern_market_example();
  j["execution"]["ac_security"] = "schedule_only";
  j["buses"][0]["load_mw"] = series(0);
  auto node = j["buses"][0]; node["id"] = 2; node["load_mw"] = series(100); j["buses"].push_back(node);
  auto g = j["generators"][0]; g["id"] = 2; g["bus"] = 2; g["segments"][0]["price_per_mwh"] = 300;
  g["initial_power_mw"] = 0; j["generators"].push_back(g);
  auto line = record("branches"); line["id"] = 10; line["from_bus"] = 1; line["to_bus"] = 2;
  line["available"] = std::vector<int>(98, 1); line["x_pu"] = 0.1; line["tap"] = 1;
  line["min_mw"] = series(-50); line["max_mw"] = series(50); line["rate_mva"] = series(60);
  j["branches"].push_back(line); return j;
}
double first_power(const json& result, int position = 0) {
  return result.at("sced").at("generators")[position].at("power_mw")[0];
}
void require_schedule(const json& result) {
  INFO(result.value("error", ""));
  INFO(result.at("status"));
  if (result.contains("scuc")) INFO(result.at("scuc").at("solver_status"));
  REQUIRE(result.at("schedule_feasible") == true);
  REQUIRE(result.at("prices_valid") == true);
}
}

TEST_CASE("Southern explicit 98-point boundary and analytic dispatch", "[southern_market]") {
  const auto boundary = make_southern_market_example();
  const auto result = run_southern_day_ahead_market(boundary);
  INFO(result.dump().substr(0, 8000));
  REQUIRE(result.at("status") == "converged");
  REQUIRE(result.at("feasible") == true);
  REQUIRE(result.at("scuc").at("max_residual").get<double>() <= 1e-6);
  REQUIRE(result.at("sced").at("day_energy_bid_cost").get<double>() == Approx(480000).margin(1e-6));
  REQUIRE(result.at("sced").at("objective").get<double>() == Approx(490000).margin(1e-6));
  REQUIRE(result.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>() == Approx(200).margin(1e-6));
  auto changed = boundary;
  changed["areas"][0]["load_mw"] = std::vector<double>(98, 101);
  const auto scenario = run_southern_day_ahead_market(changed);
  const auto comparison = compare_southern_market_results(result, scenario);
  REQUIRE(comparison.at("delta_day_energy_bid_cost").get<double>() == Approx(4800).margin(1e-6));
  REQUIRE(scenario.at("effective_boundary").at("buses")[0].at("load_mw")[0] == 101);
}

TEST_CASE("Southern hand oracle: interruptible load chooses by compensation", "[southern_market][hand_oracle]") {
  auto j = make_southern_market_example();
  j["execution"]["ac_security"] = "schedule_only";
  auto d = record("controllable_loads");
  d["id"] = 7; d["bus"] = 1; d["available"] = std::vector<int>(98, 1);
  d["max_reduction_mw"] = series(20); d["compensation_per_mwh"] = series(50);
  d["max_day_reduction_mwh"] = 480;
  j["controllable_loads"].push_back(d);
  j["generators"][0]["segments"][0]["price_per_mwh"] = 100;
  const auto result = run_southern_day_ahead_market(j);
  REQUIRE(result.at("schedule_feasible") == true);
  REQUIRE(result.at("sced").at("day_load_reduction_mwh").get<double>() == Approx(480).margin(1e-6));
  REQUIRE(result.at("sced").at("day_generation_mwh").get<double>() == Approx(1920).margin(1e-6));
  REQUIRE(result.at("sced").at("objective_terms").at("demand_response").get<double>() == Approx(24500).margin(1e-6));
  REQUIRE(result.at("sced").at("objective_terms").at("energy").get<double>() == Approx(196000).margin(1e-6));

  j["controllable_loads"][0]["compensation_per_mwh"] = series(150);
  const auto no_reduction = run_southern_day_ahead_market(j);
  REQUIRE(no_reduction.at("sced").at("day_load_reduction_mwh").get<double>() == Approx(0).margin(1e-6));
  REQUIRE(no_reduction.at("sced").at("day_generation_mwh").get<double>() == Approx(2400).margin(1e-6));
}

TEST_CASE("Southern hand oracle: two units conserve one reservoir water", "[southern_market][hand_oracle]") {
  auto j = make_southern_market_example();
  j["execution"]["ac_security"] = "schedule_only";
  j["generators"][0]["kind"] = "hydro";
  auto g = j["generators"][0]; g["id"] = 2; g["pmax_mw"] = series(100); g["segments"][0]["quantity_mw"] = 100;
  j["generators"].push_back(g);
  auto h = record("reservoirs"); h.erase("generator"); h["generators"] = {1, 2}; h["upstream"] = -1;
  h["water_m3_mwh"] = 3600; h["area_m2"] = 90000; h["initial_level_m"] = 100;
  h["physical_max_m"] = 200; h["max_level_m"] = series(200); h["inflow_m3_s"] = series(0);
  h["release_max_m3_s"] = series(1000); h["release_ramp_m3_s"] = series(1000);
  h["initial_release_m3_s"] = 0; h["max_mwh"] = 2400;
  j["reservoirs"].push_back(h);
  const auto result = run_southern_day_ahead_market(j);
  REQUIRE(result.at("schedule_feasible") == true);
  REQUIRE(result.at("sced").at("day_generation_mwh").get<double>() == Approx(2400).margin(1e-6));
  REQUIRE(result.at("sced").at("reservoirs")[0].at("release_m3_s")[0].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(result.at("sced").at("reservoirs")[0].at("release_m3_s")[95].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(result.at("sced").at("reservoirs")[0].at("level_m")[95].get<double>() == Approx(4).margin(1e-6));
}

TEST_CASE("Southern internal scenario sweep preserves weighted result semantics", "[southern_market][scenario]") {
  constexpr int scenarios = 20;
  std::mt19937 rng(20260905);
  std::normal_distribution<double> noise(0.0, 0.04);
  double probability_sum = 0.0;
  std::vector<double> load_factor, generation;
  for (int s = 0; s < scenarios; ++s) {
    auto j = make_southern_market_example();
    j["execution"]["ac_security"] = "schedule_only";
    const double factor = std::clamp(1.0 + noise(rng), 0.85, 1.15);
    const double probability = 1.0 / scenarios;
    probability_sum += probability;
    load_factor.push_back(factor);
    j["areas"][0]["load_mw"] = series(100 * factor);
    j["buses"][0]["load_mw"] = series(100 * factor);
    auto result = run_southern_day_ahead_market(j);
    REQUIRE(result.at("schedule_feasible") == true);
    REQUIRE(result.at("sced").at("max_residual").get<double>() <= 1e-6);
    REQUIRE(result.at("effective_boundary").at("areas")[0].at("load_mw")[0].get<double>() == Approx(100 * factor).margin(1e-6));
    generation.push_back(result.at("sced").at("day_generation_mwh").get<double>());
  }
  REQUIRE(probability_sum == Approx(1.0).margin(1e-12));
  REQUIRE(std::inner_product(load_factor.begin(), load_factor.end(), generation.begin(), 0.0) > 0.0);
  for (size_t i = 1; i < generation.size(); ++i) {
    if (load_factor[i] > load_factor[0] + 0.02) REQUIRE(generation[i] > generation[0]);
    if (load_factor[i] < load_factor[0] - 0.02) REQUIRE(generation[i] < generation[0]);
  }
}

TEST_CASE("Southern joint input scenarios emit overload price and renewable statistics", "[southern_market][scenario][statistics]") {
  constexpr int scenarios = 12;
  std::mt19937 rng(20260905); std::normal_distribution<double> eps(0.0, 0.06);
  int valid = 0, overload_events = 0; double price_sum = 0, renewable_util_sum = 0;
  for (int s = 0; s < scenarios; ++s) {
    auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
    const double load = std::clamp(1.0 + eps(rng), 0.85, 1.15);
    const double wind = std::clamp(1.0 + eps(rng), 0.0, 1.2);
    const double solar = std::clamp(1.0 + eps(rng), 0.0, 1.2);
    j["areas"][0]["load_mw"] = series(100 * load); j["buses"][0]["load_mw"] = series(100 * load);
    auto renewable = j["generators"][0]; renewable["id"] = 2; renewable["kind"] = "wind"; renewable["bid_mode"] = "quantity";
    renewable["pmax_mw"] = series(80); renewable["segments"][0]["quantity_mw"] = 80;
    renewable["forecast_mw"] = series(40 * wind); renewable["max_curtailment_mw"] = series(80);
    renewable["price_setting"] = std::vector<int>(98, 0); renewable["initial_power_mw"] = 0; j["generators"].push_back(renewable);
    auto solar_unit = renewable; solar_unit["id"] = 3; solar_unit["kind"] = "solar"; solar_unit["forecast_mw"] = series(30 * solar); j["generators"].push_back(solar_unit);
    auto d = record("controllable_loads"); d["id"] = 8; d["bus"] = 1; d["available"] = std::vector<int>(98, 1);
    d["max_reduction_mw"] = series(10); d["compensation_per_mwh"] = series(70 + 20 * load); d["max_day_reduction_mwh"] = 240; j["controllable_loads"].push_back(d);
    const auto result = run_southern_day_ahead_market(j);
    if (!result.value("schedule_feasible", false)) continue;
    ++valid; REQUIRE(result.at("sced").at("max_residual").get<double>() <= 1e-6);
    price_sum += result.at("lmp").at("buses")[0].at("lmp_per_mwh")[0].get<double>();
    double forecast = 0, dispatched = 0;
    for (const auto& g : result.at("effective_boundary").at("generators")) if (g.at("kind") == "wind" || g.at("kind") == "solar") forecast += g.at("forecast_mw")[0].get<double>();
    for (const auto& g : result.at("sced").at("generators")) if (g.at("kind") == "wind" || g.at("kind") == "solar") dispatched += g.at("power_mw")[0].get<double>();
    if (forecast > 1e-9) renewable_util_sum += dispatched / forecast;
    for (const auto& line : result.at("sced").at("branches")) if (line.at("slack_plus_mw")[0].get<double>() > 1e-6 || line.at("slack_minus_mw")[0].get<double>() > 1e-6) ++overload_events;
  }
  REQUIRE(valid == scenarios); REQUIRE(price_sum / valid >= 0); REQUIRE(renewable_util_sum / valid >= 0);
  REQUIRE(overload_events >= 0);
}

TEST_CASE("Southern boundaries reject silent field and identity errors", "[southern_market]") {
  auto input = make_southern_market_example();
  SECTION("unknown field") { input["ignored"] = 1; }
  SECTION("missing field") { input.erase("periods"); }
  SECTION("wrong horizon") { input["periods"].erase(97); }
  SECTION("duplicate generator") { input["generators"].push_back(input["generators"][0]); }
  SECTION("invalid bus reference") { input["generators"][0]["bus"] = 555; }
  SECTION("maintenance and must on") { input["generators"][0]["available"][0] = 0; input["generators"][0]["must_on"][0] = 1; }
  SECTION("cannot reconcile zero forecast") { input["buses"][0]["load_mw"][0] = 0; }
  SECTION("mislabelled representative point") { input["periods"][97]["kind"] = "valley"; }
  REQUIRE_THROWS_AS(validate_southern_market(input), std::invalid_argument);
}

TEST_CASE("Southern congestion prices and maintenance are boundary sensitive", "[southern_market]") {
  auto j = two_bus();
  auto result = run_southern_day_ahead_market(j); require_schedule(result);
  REQUIRE(first_power(result) == Approx(50).margin(1e-6));
  REQUIRE(first_power(result, 1) == Approx(50).margin(1e-6));
  REQUIRE(result["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>() == Approx(200).margin(1e-6));
  REQUIRE(result["lmp"]["buses"][1]["lmp_per_mwh"][0].get<double>() == Approx(300).margin(1e-6));
  SECTION("line outage changes topology") {
    j["branches"][0]["available"][0] = 0;
    result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(0).margin(1e-6));
    REQUIRE(first_power(result, 1) == Approx(100).margin(1e-6));
  }
  SECTION("generator maintenance cannot be bypassed") {
    j["generators"][0]["available"][0] = 0;
    result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(0).margin(1e-6));
  }
  SECTION("section binds separately from branch rating") {
    auto section = record("sections"); section["min_mw"] = series(-30); section["max_mw"] = series(30);
    section["members"] = {{{"branch", 10}, {"coefficient", 1}}}; j["sections"].push_back(section);
    result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(30).margin(1e-6));
  }
  SECTION("LMP matches load perturbation at the congested node") {
    j["areas"][0]["load_mw"][0] = 100.001;
    const auto next = run_southern_day_ahead_market(j); require_schedule(next);
    const double marginal = (next["sced"]["objective"].get<double>()-result["sced"]["objective"].get<double>())/0.00025;
    REQUIRE(marginal == Approx(300).margin(1e-4));
  }
}

TEST_CASE("Southern three-state starts, initial obligation, and trajectory", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  auto& g = j["generators"][0]; g["initial_on"] = 0; g["initial_power_mw"] = 0;
  g["startup_cost"] = {10, 20, 30};
  SECTION("hot") { g["initial_state_minutes"] = 100; }
  SECTION("warm inclusive threshold") { g["initial_state_minutes"] = 240; }
  SECTION("cold inclusive threshold") { g["initial_state_minutes"] = 720; }
  SECTION("residual downtime prevents initial start") {
    g["initial_state_minutes"] = 10; g["min_down_minutes"] = 60;
    const auto result = run_southern_day_ahead_market(j);
    REQUIRE(result["status"] == "scuc_failed"); return;
  }
  SECTION("authored startup trajectory") {
    for (auto& curve : g["startup_curves_mw"]) curve = {20, 40};
    j["areas"][0]["load_mw"][0] = 20; j["areas"][0]["load_mw"][1] = 40;
  }
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  const int expected = g["initial_state_minutes"].get<int>() < 240 ? 10 : g["initial_state_minutes"].get<int>() < 720 ? 20 : 30;
  REQUIRE(result["scuc"]["objective_terms"]["startup"].get<double>() == Approx(expected).margin(1e-6));
}

TEST_CASE("Southern provincial reserves, group policy, regulation and nonmarket plans", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  auto second = j["generators"][0]; second["id"] = 2; second["segments"][0]["price_per_mwh"] = 300;
  second["initial_power_mw"] = 0; j["generators"].push_back(second);
  double expected = 100;
  SECTION("positive reserve shortage") {
    j["areas"][0]["reserve_up_mw"] = series(301);
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed"); return;
  }
  SECTION("downward reserve shortage") {
    j["areas"][0]["reserve_down_mw"] = series(101);
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed"); return;
  }
  SECTION("primary frequency sum min") {
    j["areas"][0]["primary_mw"] = series(20);
    j["generators"][0]["primary_fraction"] = series(0.05);
    j["generators"][1]["primary_fraction"] = series(0.05);
  }
  SECTION("nonmarket nuclear schedule") {
    auto e = record("external_schedules"); e["bus"] = 1; e["kind"] = "nuclear"; e["power_mw"] = series(30);
    j["external_schedules"].push_back(e); expected = 70;
  }
  SECTION("regulation preclearing reduces SCED range") {
    j["generators"][0]["regulation_up_mw"] = series(120);
    j["generators"][1]["must_on"] = std::vector<int>(98, 1); expected = 80;
  }
  SECTION("group energy maximum") {
    auto group = record("groups"); group["generators"] = {1}; group["max_online"] = series(1);
    group["max_mw"] = series(200); group["max_mwh"] = 1200; j["groups"].push_back(group);
    const auto result = run_southern_day_ahead_market(j); require_schedule(result);
    double energy = 0; for (int t = 0; t < 96; ++t) energy += result["sced"]["generators"][0]["power_mw"][t].get<double>()*0.25;
    REQUIRE(energy == Approx(1200).margin(1e-6)); return;
  }
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  REQUIRE(first_power(result) == Approx(expected).margin(1e-6));
}

TEST_CASE("Southern storage signs, terminal energy and efficiency weighted cycle", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  auto s = record("storage"); s["bus"] = 1; s["available"] = std::vector<int>(98, 1);
  s["discharge_max_mw"] = 40; s["charge_max_mw"] = 40; s["rated_mwh"] = 100;
  s["roundtrip_efficiency"] = 0.81; s["initial_mwh"] = 50; s["terminal_mwh"] = 59;
  s["max_mwh"] = series(100); s["max_cycles"] = 2; s["price_setting"] = std::vector<int>(98, 1);
  s["discharge_price"] = 10000; s["charge_price"] = 0;
  j["storage"].push_back(s);
  SECTION("efficiency and independent terminal target") {}
  SECTION("zero cycles rejects unequal terminal target") {
    j["storage"][0]["max_cycles"] = 0;
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed"); return;
  }
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  const auto& storage = result["sced"]["storage"][0];
  double charged = 0;
  for (int t = 0; t < 96; ++t) {
    REQUIRE(storage["charge_mw"][t].get<double>() <= 1e-6);
    charged -= storage["charge_mw"][t].get<double>()*0.25;
    const double before = t ? storage["energy_mwh"][t-1].get<double>() : 50;
    const double expected = before-storage["charge_mw"][t].get<double>()*0.9*0.25-storage["discharge_mw"][t].get<double>()/0.9*0.25;
    REQUIRE(storage["energy_mwh"][t].get<double>() == Approx(expected).margin(1e-6));
  }
  REQUIRE(charged == Approx(10).margin(1e-6));
  REQUIRE(storage["energy_mwh"][95].get<double>() == Approx(59).margin(1e-6));
  for (int hour = 0; hour < 24; ++hour) {
    double dis = 0, ch = 0;
    for (int t = 4*hour; t < 4*hour+4; ++t) { dis += storage["discharge_mw"][t].get<double>(); ch -= storage["charge_mw"][t].get<double>(); }
    REQUIRE(std::min(dis, ch) <= 1e-6);
  }
}

TEST_CASE("Southern hydrology uses SI conservation and D-day energy policy", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  j["generators"][0]["kind"] = "hydro";
  auto h = record("reservoirs"); h["generator"] = 1; h["upstream"] = -1;
  h["water_m3_mwh"] = 3600; h["area_m2"] = 90000; h["initial_level_m"] = 100;
  h["physical_max_m"] = 200; h["max_level_m"] = series(200); h["inflow_m3_s"] = series(90);
  h["release_max_m3_s"] = series(1000); h["release_ramp_m3_s"] = series(1000);
  h["initial_release_m3_s"] = 100; h["max_mwh"] = 2500;
  j["reservoirs"].push_back(h);
  SECTION("water conservation") {}
  SECTION("hydro daily energy cap") {
    j["reservoirs"][0]["max_mwh"] = 2399;
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed"); return;
  }
  SECTION("cascade upstream delay includes supplied history") {
    j["reservoirs"][0]["release_history_m3_s"] = {50};
    auto g = j["generators"][0]; g["id"] = 2; g["must_off"] = std::vector<int>(98, 1);
    g["initial_on"] = 0; g["initial_power_mw"] = 0; j["generators"].push_back(g);
    auto downstream = h; downstream["id"] = 11; downstream["generator"] = 2; downstream["upstream"] = 10;
    downstream["lag_slots"] = 1; downstream["inflow_m3_s"] = series(0); downstream["initial_release_m3_s"] = 0;
    downstream["max_level_m"] = series(300); downstream["physical_max_m"] = 300;
    j["reservoirs"].push_back(downstream);
    const auto result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(result["sced"]["reservoirs"][1]["level_m"][0].get<double>() == Approx(100.5).margin(1e-6));
    REQUIRE(result["sced"]["reservoirs"][1]["level_m"][1].get<double>() == Approx(101.5).margin(1e-6)); return;
  }
  const auto result = run_southern_day_ahead_market(j); require_schedule(result);
  REQUIRE(result["sced"]["reservoirs"][0]["level_m"][0].get<double>() == Approx(99.9).margin(1e-6));
  REQUIRE(result["sced"]["reservoirs"][0]["level_m"][95].get<double>() == Approx(90.4).margin(1e-6));
}

TEST_CASE("Southern HVDC loss, hub balance and adjustment reversal", "[southern_market]") {
  auto j = two_bus(); j["branches"] = json::array();
  auto dc = record("dc_links"); dc["from_bus"] = 1; dc["to_bus"] = 2; dc["from_hub"] = -1; dc["to_hub"] = -1;
  dc["available"] = std::vector<int>(98, 1); dc["max_mw"] = series(200); dc["loss_fraction"] = 0.1;
  dc["initial_mw"] = 100; dc["ramp_up_mw"] = series(200); dc["ramp_down_mw"] = series(200);
  j["dc_links"].push_back(dc);
  SECTION("constant line losses") {
    auto result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(100/0.9).margin(1e-6));
    REQUIRE(first_power(result, 1) == Approx(0).margin(1e-6));
  }
  SECTION("multiterminal hub") {
    auto hub = record("dc_hubs"); j["dc_hubs"].push_back(hub);
    j["dc_links"][0]["to_bus"] = -1; j["dc_links"][0]["to_hub"] = 10;
    dc["id"] = 11; dc["from_bus"] = -1; dc["from_hub"] = 10;
    j["dc_links"].push_back(dc);
    auto result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(100/0.81).margin(1e-6));
  }
  SECTION("forced consecutive opposite adjustments are infeasible") {
    j["dc_links"][0]["min_mw"][0] = 120; j["dc_links"][0]["max_mw"][0] = 120;
    j["dc_links"][0]["min_mw"][1] = 100; j["dc_links"][0]["max_mw"][1] = 100;
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed");
  }
}

TEST_CASE("Southern renewable categories and zero start limit", "[southern_market]") {
  auto j = make_southern_market_example(); j["execution"]["ac_security"] = "schedule_only";
  SECTION("quantity renewable deviation") {
    auto g = j["generators"][0]; g["id"] = 2; g["kind"] = "renewable"; g["bid_mode"] = "quantity";
    g["forecast_mw"] = series(120); g["max_curtailment_mw"] = series(120);
    g["price_setting"] = std::vector<int>(98, 0); j["generators"].push_back(g);
    const auto r = run_southern_day_ahead_market(j); require_schedule(r);
    REQUIRE(first_power(r, 1) == Approx(100).margin(1e-6));
    REQUIRE(r["sced"]["generators"][1]["renewable_deviation_mw"][0].get<double>() == Approx(20).margin(1e-6));
  }
  SECTION("zero starts forbids activation") {
    j["generators"][0]["initial_on"] = 0; j["generators"][0]["initial_power_mw"] = 0;
    j["generators"][0]["max_starts"] = 0;
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed");
  }
}

TEST_CASE("Southern AC feedback repeats the clearing chain and never prices failed security", "[southern_market]") {
  auto j = two_bus(); j["execution"]["ac_security"] = "required";
  j["branches"][0]["rate_mva"] = series(35);
  SECTION("thermal sensitivity cuts reduce transfer") {
    const auto result = run_southern_day_ahead_market(j);
    INFO(result.at("status"));
    REQUIRE(result["security_iterations"].size() > 1);
    REQUIRE(result["feasible"] == true);
    REQUIRE(first_power(result) < 35);
    REQUIRE(result["security_iterations"].back()["secure"] == true);
  }
  SECTION("iteration cap returns failure with no prices") {
    j["execution"]["security_iterations"] = 1;
    const auto result = run_southern_day_ahead_market(j);
    REQUIRE(result["status"] == "ac_security_failed");
    REQUIRE(result["feasible"] == false);
    REQUIRE(result["prices_valid"] == false);
    REQUIRE_FALSE(result.contains("lmp"));
  }
}

TEST_CASE("Southern priority trade mapping and strict versus penalized energy", "[southern_market]") {
  auto j = two_bus();
  auto trade = record("trades"); trade["gateway_kind"] = "ac_branch"; trade["gateway"] = 10;
  trade["max_mw"] = series(30); trade["adjusted_min_mwh"] = 800; trade["original_min_mwh"] = 800;
  trade["max_mwh"] = 2000; j["trades"].push_back(trade);
  SECTION("hard priority infeasible") {
    REQUIRE(run_southern_day_ahead_market(j)["status"] == "scuc_failed");
  }
  SECTION("explicit slack interpretation") {
    j["execution"]["priority_policy"] = "penalized_shortfall";
    const auto result = run_southern_day_ahead_market(j); require_schedule(result);
    REQUIRE(first_power(result) == Approx(30).margin(1e-6));
    REQUIRE(result["sced"]["trades"][0]["priority_shortfall_mwh"].get<double>() == Approx(80).margin(1e-6));
  }
}
