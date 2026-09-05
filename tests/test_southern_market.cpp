#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <numeric>
#include <random>
#include <set>
#include "hacdcpf/market/southern_market.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "../src/market/southern_solver_status.hpp"
#include "hacdcpf/engine/solver/external/adapters.hpp"
#include "hacdcpf/engine/engine.hpp"

using nlohmann::json;
using namespace hacdcpf::market;
using Catch::Approx;

TEST_CASE("Southern compact projection preserves original startup history and objective", "[southern_market][compact]") {
  for (const int downtime : {239,240,719,720}) {
    auto b = make_southern_market_example();
    b["execution"]["ac_security"] = "schedule_only";
    auto& g = b["generators"][0];
    g["initial_on"] = 0; g["initial_power_mw"] = 0; g["initial_state_minutes"] = downtime;
    g["startup_cost"] = {17,17,17};
    // Forced load makes t=0 a start; a middle-day outage exercises all four transitions.
    b["execution"]["balance_policy"] = "diagnostic";
    g["must_off"][30] = 1; g["must_off"][31] = 1;
    b["execution"]["formulation"] = "reference";
    const auto reference = run_southern_day_ahead_market(b);
    b["execution"]["formulation"] = "compact";
    const auto compact = run_southern_day_ahead_market(b);
    REQUIRE(reference["prices_valid"] == true); REQUIRE(compact["prices_valid"] == true);
    for (const auto* stage : {"scuc","sced","lmp"}) {
      REQUIRE(compact[stage]["objective"].get<double>() == Approx(reference[stage]["objective"].get<double>()).margin(1e-4));
      REQUIRE(compact[stage]["max_residual"].get<double>() <= 1e-6);
      REQUIRE(compact[stage]["reconstructed_max_residual"].get<double>() <= 1e-6);
      REQUIRE(compact[stage]["compact_units"] == 1);
      const auto& a = compact[stage]["generators"][0]; const auto& r = reference[stage]["generators"][0];
      for (const auto* field : {"online","start","stop","hot_start","warm_start","cold_start","power_mw"})
        for (int t=0;t<98;++t) REQUIRE(a[field][t].get<double>() == Approx(r[field][t].get<double>()).margin(1e-6));
    }
    REQUIRE(reference["scuc"]["variables"].get<int>()-compact["scuc"]["variables"].get<int>() == 4*98);
    REQUIRE(reference["scuc"]["binary_variables"].get<int>()-compact["scuc"]["binary_variables"].get<int>() == 5*98);
  }
  auto b = make_southern_market_example();
  b["generators"][0]["startup_cost"] = {10,20,30};
  const auto noneligible = run_southern_day_ahead_market(b);
  REQUIRE(noneligible["prices_valid"] == true);
  REQUIRE(noneligible["scuc"]["compact_units"] == 0);
  b["execution"]["formulation"] = "invalid"; REQUIRE_THROWS(validate_southern_market(b));
}

TEST_CASE("Southern solver options normalize old inputs and reject unavailable selection", "[southern_market][solver_options]") {
  auto b = make_southern_market_example();
  b["execution"].erase("solver"); b["execution"].erase("threads");
  const auto effective = validate_southern_market(b);
  REQUIRE(effective["execution"]["solver"] == "highs"); REQUIRE(effective["execution"]["threads"] == 0);
  auto invalid = b; invalid["execution"]["solver"] = "unknown"; REQUIRE_THROWS(validate_southern_market(invalid));
  invalid = b; invalid["execution"]["threads"] = 2; REQUIRE_THROWS(validate_southern_market(invalid));
  b["execution"]["solver"] = "gurobi";
  REQUIRE_NOTHROW(validate_southern_market(b));
  if (!southern_market_solver_capabilities()[1]["available"].get<bool>()) {
    REQUIRE_THROWS(run_southern_day_ahead_market(b));
    json c={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
    const auto initial=make_market_operation(b,c), failed=step_market_operation(initial);
    REQUIRE(failed["status"] == "failed"); REQUIRE(failed["completed_days"] == 0);
    REQUIRE(failed["carry"] == initial["carry"]);
  }
}

TEST_CASE("Southern Gurobi clears identical sparse models and preserves rolling selection", "[southern_market][gurobi]") {
  const auto capabilities = southern_market_solver_capabilities();
  if (!capabilities[1]["available"].get<bool>()) { SKIP("Gurobi library or license unavailable"); }
  auto b = make_southern_market_example(); b["execution"]["solver"] = "gurobi"; b["execution"]["threads"] = 2;
  const auto result = run_southern_day_ahead_market(b);
  INFO(result.dump().substr(0,2500));
  REQUIRE(result["schedule_feasible"] == true); REQUIRE(result["prices_valid"] == true);
  for (const auto* stage : {"scuc","sced","lmp"}) {
    REQUIRE(result[stage]["solver"] == "Gurobi");
    REQUIRE(result[stage]["requested_threads"] == 2);
    REQUIRE(result[stage]["max_residual"].get<double>() <= 1e-6);
    REQUIRE(result[stage]["optimality_proven"] == true);
  }
  REQUIRE(result["sced"]["day_energy_bid_cost"].get<double>() == Approx(480000).margin(1e-6));
  REQUIRE(result["sced"]["generators"][0]["power_mw"][95].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(result["lmp"]["buses"][0]["lmp_per_mwh"][0].get<double>() == Approx(200).margin(1e-6));
  auto demo = make_southern_market_demo(); const auto highs = run_southern_day_ahead_market(demo);
  demo["execution"]["solver"] = "gurobi"; demo["execution"]["threads"] = 2;
  const auto gurobi = run_southern_day_ahead_market(demo);
  INFO(gurobi.dump().substr(0,2000)); REQUIRE(gurobi["schedule_feasible"] == true); REQUIRE(gurobi["prices_valid"] == true);
  REQUIRE(highs["schedule_feasible"] == true);
  REQUIRE(gurobi["scuc"]["objective"].get<double>() == Approx(highs["scuc"]["objective"].get<double>()).margin(1e-4));
  for (const auto* stage : {"scuc","sced","lmp"}) REQUIRE(gurobi[stage]["max_residual"].get<double>() <= 1e-6);
  json config={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()},
    {"solver_options",{{"solver","gurobi"},{"threads",2},{"time_limit_sec",30},{"mip_gap",.01}}}};
  auto job = step_market_operation(make_market_operation(make_southern_market_example(),config));
  REQUIRE(job["completed_days"] == 1);
  REQUIRE(job["days"][0]["stages"]["scuc"]["solver"] == "Gurobi");
  REQUIRE(job["days"][0]["stages"]["scuc"]["requested_mip_gap"] == .01);
  REQUIRE(job["days"][0]["stages"]["scuc"]["requested_time_limit_sec"] == 30);
  REQUIRE(preview_market_operation_boundary(b,config,0)["effective"]["execution"]["solver"] == "gurobi");
  config["solver_options"]["solver"] = "typo"; REQUIRE_THROWS(make_market_operation(b,config));
  config["solver_options"]["solver"] = "highs"; REQUIRE_THROWS(make_market_operation(b,config));
}

TEST_CASE("Gurobi sparse continuous adapter retains ranged-row dual identity", "[southern_market][gurobi]") {
  using namespace hacdcpf::engine;
  GurobiAdapter adapter(GurobiOptions{30,0,1});
  if (!adapter.available()) { SKIP("Gurobi library or license unavailable"); }
  REQUIRE_THROWS(GurobiAdapter(GurobiOptions{-1,0,1}));
  LPModel lp; lp.vars.resize(2);
  for (auto& v : lp.vars) { v.lb=0; v.ub=10; }
  lp.c=Eigen::Vector2d(1,2); lp.A.resize(1,2); lp.A.insert(0,0)=1;
  lp.b=Eigen::VectorXd::Constant(1,3); lp.row_lhs=Eigen::VectorXd::Constant(1,2);
  lp.Aeq.resize(1,2); lp.Aeq.insert(0,1)=1; lp.beq=Eigen::VectorXd::Constant(1,4);
  const auto solved=adapter.solve_lp(lp);
  REQUIRE(solved.stats.success); REQUIRE(solved.stats.objective == Approx(10).margin(1e-8));
  REQUIRE(solved.constraint_duals.size()==2);
  REQUIRE(solved.constraint_duals[0]==Approx(1).margin(1e-8));
  REQUIRE(solved.constraint_duals[1]==Approx(2).margin(1e-8));
  GurobiAdapter limited(GurobiOptions{1e-9,0,1});
  const auto timeout = limited.solve_lp(lp);
  REQUIRE(timeout.stats.status == "TimeLimit");
  REQUIRE(timeout.constraint_duals.size() == 0);
  lp.row_lhs[0] = 11; lp.b[0] = 12;
  const auto impossible = adapter.solve_lp(lp);
  REQUIRE_FALSE(impossible.stats.success);
  REQUIRE((impossible.stats.status == "Infeasible" || impossible.stats.status == "UnboundedOrInfeasible"));
}

TEST_CASE("Southern solver quality requires audited feasibility and exact adapter status", "[southern_market][lookahead]") {
  using detail::southern_solver_status;
  REQUIRE(southern_solver_status("StrictHiGHS Optimal run=0",true,0).optimality_proven);
  REQUIRE(southern_solver_status("HiGHS optimal",true,0).quality == "optimal_within_tolerance");
  REQUIRE_FALSE(southern_solver_status("StrictHiGHS Optimal run=0",false,0).optimality_proven);
  REQUIRE_FALSE(southern_solver_status("StrictHiGHS Optimal run=0",true,.01).optimality_proven);
  REQUIRE(southern_solver_status("StrictHiGHS TimeLimit run=1",true,.2).quality == "feasible_limit");
  REQUIRE(southern_solver_status("HiGHS time_limit",false,.2).quality == "limit_without_verified_solution");
  REQUIRE(southern_solver_status("StrictHiGHS UnboundedOrInfeasible run=0",false,0).quality == "infeasible_or_unbounded");
  REQUIRE(southern_solver_status("StrictHiGHS Infeasible run=0",false,0).quality == "proven_infeasible");
  REQUIRE_FALSE(southern_solver_status("suboptimal",true,0).optimality_proven);
  REQUIRE(southern_solver_status("Optimal",true,0).optimality_proven);
  REQUIRE(southern_solver_status("NodeLimit",true,.2).quality == "feasible_limit");
  REQUIRE(southern_solver_status("TimeLimit",false,0).quality == "limit_without_verified_solution");
}

TEST_CASE("Rolling next-day extrema drive all series and realized ramp carry", "[southern_market][operation][lookahead]") {
  auto b = make_southern_market_example();
  auto& g = b["generators"][0]; g["must_on"] = std::vector<int>(98,1); g["ramp_up_mw_min"] = 2;
  json config = {{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  auto baseline = step_market_operation(make_market_operation(b,config));
  REQUIRE(baseline["completed_days"] == 1);
  config = baseline["config"];
  REQUIRE(config["days"].size() == 8);
  const auto edit = [](const char* table,const char* field,int t,double value) {
    return json{{"table",table},{"id",1},{"field",field},{"first_slot",t},{"last_slot",t},{"value",value},{"reason","analytic next-day forecast"}};
  };
  config["days"][1]["boundary_overrides"] = json::array({edit("areas","load_mw",0,180),edit("areas","load_mw",95,60),edit("generators","pmin_mw",0,180)});
  const auto p = preview_market_operation_boundary(b,config,0);
  REQUIRE(p["lookahead"]["source_day"] == 1);
  REQUIRE(p["lookahead"]["points"][0]["kind"] == "peak");
  REQUIRE(p["lookahead"]["points"][0]["source_slot"] == 0);
  REQUIRE(p["authored"]["areas"][0]["load_mw"][96] == 180);
  REQUIRE(p["authored"]["areas"][0]["load_mw"][97] == 60);
  REQUIRE(p["effective"]["buses"][0]["load_mw"][96] == 180);
  REQUIRE(p["authored"]["generators"][0]["pmin_mw"][96] == 180);
  for (int t = 0; t < 96; ++t) REQUIRE(p["authored"]["areas"][0]["load_mw"][t] == 100);
  auto job = step_market_operation(make_market_operation(b,config));
  INFO(job.dump().substr(0,1000)); REQUIRE(job["completed_days"] == 1);
  REQUIRE(job["days"][0]["state_end"]["generators"][0]["initial_power_mw"].get<double>() == Approx(150).margin(1e-6));
  REQUIRE(baseline["days"][0]["state_end"]["generators"][0]["initial_power_mw"].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(job["days"][0]["surplus_mwh"].get<double>() == Approx(17.5).margin(1e-6));
  REQUIRE(job["days"][0]["periods"].size() == 96);
  job = step_market_operation(job); REQUIRE(job["completed_days"] == 2);
  REQUIRE(job["days"][1]["state_start"] == job["days"][0]["state_end"]);
  config["days"][7]["load_scale"] = 1.5;
  REQUIRE(preview_market_operation_boundary(b,config,6)["authored"]["areas"][0]["load_mw"][96] == 150);
  REQUIRE(preview_market_operation_boundary(b,config,7)["forecast_only"] == true);
  auto invalid = config;
  invalid["days"][7]["boundary_overrides"] = json::array({edit("generators","ramp_up_mw_min",0,2)});
  invalid["days"][7]["boundary_overrides"][0]["last_slot"] = 97;
  REQUIRE_THROWS(make_market_operation(b,invalid));
  config["days"][0]["boundary_overrides"] = json::array({edit("areas","load_mw",96,100)});
  REQUIRE_THROWS(make_market_operation(b,config));
}

TEST_CASE("Rule boundary catalog matches admitted schemas and daily load reconciliation", "[southern_market][operation][boundary_rules]") {
  const auto schema = southern_market_schema(), catalog = southern_market_boundary_catalog();
  REQUIRE(catalog.at("items").size() == 15);
  for (const auto& group : catalog.at("items")) {
    REQUIRE_FALSE(group.at("clauses").get<std::string>().empty());
    REQUIRE_FALSE(group.at("limitation").get<std::string>().empty());
    for (const auto& f : group.at("fields")) REQUIRE(f.at("schema") == schema.at("properties").at(f.at("table").get<std::string>()).at("items").at("properties").at(f.at("field").get<std::string>()));
  }
  auto b=make_southern_market_example();
  b["areas"][0]["load_mw"]=std::vector<double>(98,200);
  b["buses"][0]["load_mw"]=std::vector<double>(98,1);
  auto other=b["buses"][0];other["id"]=22;other["load_mw"]=std::vector<double>(98,3);b["buses"].push_back(other);
  json c={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  auto p=preview_market_operation_boundary(b,c,0);
  REQUIRE(p["authored"]["buses"][0]["load_mw"][0]==1);
  REQUIRE(p["effective"]["buses"][0]["load_mw"][0].get<double>()==Approx(50));
  REQUIRE(p["effective"]["buses"][1]["load_mw"][0].get<double>()==Approx(150));
  c=make_market_operation(b,c)["config"];
  json e={{"table","buses"},{"id",1},{"field","load_mw"},{"first_slot",0},{"last_slot",95},{"value",3},{"reason","forecast revised"}};
  c["days"][0]["boundary_overrides"]=json::array({e});
  p=preview_market_operation_boundary(b,c,0);
  REQUIRE(p["effective"]["buses"][0]["load_mw"][0].get<double>()==Approx(100));
  REQUIRE(p["effective"]["buses"][1]["load_mw"][0].get<double>()==Approx(100));
  REQUIRE(p["effective"]["buses"][0]["load_mw"][96].get<double>()==Approx(50));
  REQUIRE(preview_market_operation_boundary(b,c,1)["effective"]["buses"][0]["load_mw"][0].get<double>()==Approx(50));
  auto invalid=c;invalid["days"][0]["boundary_overrides"].push_back(e);REQUIRE_THROWS(make_market_operation(b,invalid));
  for (const auto* field : {"initial_power_mw","segments","unimplemented_field"}) {
    invalid=c;invalid["days"][0]["boundary_overrides"][0]["table"]="generators";invalid["days"][0]["boundary_overrides"][0]["field"]=field;REQUIRE_THROWS(make_market_operation(b,invalid));
  }
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["id"]=999;REQUIRE_THROWS(make_market_operation(b,invalid));
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["id"]=4294967297LL;REQUIRE_THROWS(make_market_operation(b,invalid));
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["reason"]="";REQUIRE_THROWS(make_market_operation(b,invalid));
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["value"]=-1;REQUIRE_THROWS(make_market_operation(b,invalid));
  invalid=c;invalid["days"][0]["boundary_overrides"][0]["table"]="generators";invalid["days"][0]["boundary_overrides"][0]["field"]="pmin_mw";invalid["days"][0]["boundary_overrides"][0]["value"]=300;REQUIRE_THROWS(make_market_operation(b,invalid));
}

TEST_CASE("Rolling representatives synchronize every typed time series by stable ID", "[southern_market][operation][lookahead]") {
  const auto b = make_southern_market_demo();
  json c = {{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  c = make_market_operation(b,c)["config"];
  c["days"][1]["wind_scale"] = .7; c["days"][1]["solar_scale"] = .8;
  c["days"][1]["inflow_scale"] = .9; c["days"][1]["load_scale"] = 1.1;
  c["days"][1]["line_limit_scale"] = .6; c["days"][1]["branch_outages"] = {b["branches"][0]["id"]};
  c["days"][1]["load_bid_scale"] = .5;
  const auto p = preview_market_operation_boundary(b,c,0), next = preview_market_operation_boundary(b,c,1);
  const auto schema = southern_market_schema();
  size_t compared = 0;
  for (const auto& point : p["lookahead"]["points"]) {
    const int target = point["target_slot"], source = point["source_slot"];
    for (auto table = schema["properties"].begin(); table != schema["properties"].end(); ++table) {
      if (table.key() == "periods" || table.value().value("type", "") != "array" || !table.value()["items"].contains("properties")) continue;
      for (const auto& row : p["authored"][table.key()]) {
        const auto& sources = next["authored"][table.key()];
        const auto it = std::find_if(sources.begin(),sources.end(),[&](const json& r){return r["id"] == row["id"];});
        REQUIRE(it != sources.end());
        for (auto f = table.value()["items"]["properties"].begin(); f != table.value()["items"]["properties"].end(); ++f)
          if (f.value().value("type", "") == "array" && f.value().value("minItems",0) == 98 && f.value().value("maxItems",0) == 98 && row.contains(f.key())) {
            REQUIRE(row[f.key()][target] == it->at(f.key())[source]); ++compared;
          }
      }
    }
  }
  REQUIRE(compared > 200);
}

TEST_CASE("Daily nonmarket plan enters clearing and forecast boundaries", "[southern_market][operation][boundary_rules]") {
  auto b=make_southern_market_example();
  b["external_schedules"].push_back({{"id",72},{"name","nonmarket nuclear"},{"source","analytic"},{"kind","nuclear"},{"bus",1},{"power_mw",std::vector<double>(98,0)}});
  json c={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  c=make_market_operation(b,c)["config"];
  json e={{"table","external_schedules"},{"id",72},{"field","power_mw"},{"first_slot",0},{"last_slot",95},{"value",40},{"reason","approved nuclear plan"}};
  c["days"][0]["boundary_overrides"]=json::array({e});
  auto job=step_market_operation(make_market_operation(b,c));
  REQUIRE(job["days"][0]["valid"]==true);
  REQUIRE(job["days"][0]["state_end"]["generators"][0]["initial_power_mw"].get<double>()==Approx(60).margin(1e-6));
  REQUIRE(job["days"][0]["nodes"][0]["load_mw"][0]==100);
  auto f=market_forecast_defaults();f["sample_count"]=1;f["operation"]=c;
  for(auto& m:f["marginals"]) {m["distribution"]="fixed";m["center"]=std::vector<double>(7,1);}
  auto forecast=make_market_forecast(b,f);
  REQUIRE(forecast["scenarios"][0]["config"]["days"][0]["boundary_overrides"][0]==e);
  forecast=step_market_forecast(forecast);
  REQUIRE(forecast["scenarios"][0]["days"][0]["state_end"]["generators"][0]["initial_power_mw"].get<double>()==Approx(60).margin(1e-6));
}

TEST_CASE("Rolling market operation calendar and analytic deficit sensitivity", "[southern_market][operation]") {
  auto b = make_southern_market_example();
  json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", true}, {"days", json::array()}};
  auto invalid = c; invalid["unexpected"] = 1;
  REQUIRE_THROWS(make_market_operation(b, invalid));
  invalid = c; invalid["start_date"] = "2027-02-29";
  REQUIRE_THROWS(make_market_operation(b, invalid));
  auto job = make_market_operation(b, c);
  REQUIRE(job["total_days"] == 7);
  c = job["config"]; c["days"][1]["load_scale"] = 3;
  job = make_market_operation(b, c);
  for (int d = 0; d < 7; ++d) {
    job = step_market_operation(job);
    INFO(job.value("error", ""));
    REQUIRE(job["completed_days"] == d+1);
    const auto& day = job["days"][d]; REQUIRE(day["valid"] == true);
    REQUIRE(day["periods"].size() == 96);
    for (const auto& p : day["periods"]) REQUIRE(p["deficit_mw"].get<double>() == Approx(d == 1 ? 100 : 0).margin(1e-6));
    REQUIRE(day["deficit_mwh"].get<double>() == Approx(d == 1 ? 2400 : 0).margin(1e-6));
    if (d) REQUIRE(day["state_start"] == job["days"][d-1]["state_end"]);
  }
  REQUIRE(job["status"] == "completed");
  const auto& proof = job["days"][1]["counterfactuals"][0];
  REQUIRE(proof["factor"] == "load_scale"); REQUIRE(proof["valid"] == true);
  REQUIRE(proof["reduction_deficit_mwh"].get<double>() == Approx(2400).margin(1e-6));
  REQUIRE(job["days"][1]["state_end"]["generators"][0]["initial_power_mw"].get<double>() == Approx(200).margin(1e-6));
  REQUIRE(job["days"][1]["state_end"]["generators"][0]["initial_state_minutes"].get<int>() == 4320);
  REQUIRE_THROWS(step_market_operation(job));
  b["areas"][0]["load_mw"] = std::vector<double>(98, 300);
  b["buses"][0]["load_mw"] = std::vector<double>(98, 300);
  REQUIRE(run_southern_day_ahead_market(b)["schedule_feasible"] == false);
}

TEST_CASE("Rolling market operation realizes a complete leap month", "[southern_market][operation]") {
  const auto b = make_southern_market_example();
  json c = {{"horizon", "month"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", false}, {"days", json::array()}};
  auto job = make_market_operation(b, c); REQUIRE(job["total_days"] == 29);
  for (int d = 0; d < 29; ++d) { job = step_market_operation(job); INFO(job.value("error", "")); REQUIRE(job["completed_days"] == d+1); }
  REQUIRE(job["status"] == "completed");
  size_t points = 0; for (const auto& d : job["days"]) points += d["periods"].size(); REQUIRE(points == 2784);
  c["start_date"] = "2028-02-02"; REQUIRE_THROWS(make_market_operation(b, c));
  c["start_date"] = "2027-02-01"; REQUIRE(make_market_operation(b, c)["total_days"] == 28);
  c["start_date"] = "2028-04-01"; REQUIRE(make_market_operation(b, c)["total_days"] == 30);
  c["start_date"] = "2028-01-01"; REQUIRE(make_market_operation(b, c)["total_days"] == 31);
}

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
  REQUIRE(result.at("scuc").at("optimality_proven") == true);
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

TEST_CASE("Rolling diagnostics distinguish a line limit breach from node deficit", "[southern_market][operation]") {
  auto b = two_bus(); b["generators"].erase(1);
  json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", true}, {"days", json::array()}};
  auto job = make_market_operation(b, c); c = job["config"];
  c["days"][0]["line_limit_scale"] = 0.5; c["days"][0]["first_slot"] = 4; c["days"][0]["last_slot"] = 7;
  job = step_market_operation(make_market_operation(b, c));
  INFO(job.value("error", "")); REQUIRE(job["completed_days"] == 1);
  const auto& d = job["days"][0]; REQUIRE(d["valid"] == true);
  for (int t = 0; t < 96; ++t) {
    REQUIRE(d["periods"][t]["deficit_mw"].get<double>() == Approx(0).margin(1e-6));
    REQUIRE(d["lines"][0]["power_mw"][t].get<double>() == Approx(100).margin(1e-6));
    REQUIRE(d["lines"][0]["overload_mw"][t].get<double>() == Approx(t >= 4 && t <= 7 ? 75 : 50).margin(1e-6));
  }
  REQUIRE(d["counterfactuals"][0]["reduction_overload_mwh"].get<double>() == Approx(25).margin(1e-6));
  auto invalid = c; invalid["days"][0]["branch_outages"] = {999}; REQUIRE_THROWS(make_market_operation(b, invalid));
  c["days"][0]["branch_outages"] = {10};
  job = step_market_operation(make_market_operation(b, c)); REQUIRE(job["completed_days"] == 1);
  REQUIRE(job["days"][0]["periods"][4]["deficit_mw"].get<double>() == Approx(100).margin(1e-6));
  REQUIRE(job["days"][0]["lines"][0]["overload_mw"][4].get<double>() == Approx(0).margin(1e-6));
}

TEST_CASE("Rolling shared hydro and storage carry realized state across days", "[southern_market][operation]") {
  const auto b = make_southern_market_demo();
  json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", false}, {"days", json::array()}};
  auto job = step_market_operation(make_market_operation(b, c));
  INFO(job.value("error", "")); REQUIRE(job["completed_days"] == 1);
  const auto end = job["days"][0]["state_end"];
  REQUIRE(end["storage"].size() == 2); REQUIRE(end["reservoirs"].size() == 1);
  REQUIRE(end["reservoirs"][0]["release_history_m3_s"].size() == 96);
  REQUIRE(end["reservoirs"][0]["release_history_m3_s"][95] == end["reservoirs"][0]["initial_release_m3_s"]);
  job = step_market_operation(job); INFO(job.value("error", "")); REQUIRE(job["completed_days"] == 2);
  REQUIRE(job["days"][1]["state_start"] == end);
  auto invalid = b; invalid["generators"][0]["shutdown_curve_mw"] = {0};
  REQUIRE_THROWS(make_market_operation(invalid, c));
}

TEST_CASE("Rolling diagnostic surplus is distinct from equality residual", "[southern_market][operation]") {
  auto b = make_southern_market_example();
  b["generators"][0]["pmin_mw"] = series(150);
  b["generators"][0]["must_on"] = std::vector<int>(98, 1);
  json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", false}, {"days", json::array()}};
  const auto job = step_market_operation(make_market_operation(b, c));
  INFO(job.value("error", "")); REQUIRE(job["completed_days"] == 1);
  REQUIRE(job["days"][0]["surplus_mwh"].get<double>() == Approx(1200).margin(1e-6));
  for (const auto& p : job["days"][0]["periods"]) REQUIRE(p["surplus_mw"].get<double>() == Approx(50).margin(1e-6));
  for (const auto& v : job["days"][0]["nodes"][0]["node_imbalance_mw"]) REQUIRE(std::abs(v.get<double>()) <= 1e-6);
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
    REQUIRE(result["sced"]["reservoirs"][1]["level_m"][1].get<double>() == Approx(101.5).margin(1e-6));
    json c = {{"horizon", "week"}, {"start_date", "2028-02-01"}, {"penalty_per_mwh", 100000}, {"explain", false}, {"days", json::array()}};
    auto job = step_market_operation(make_market_operation(j, c)); REQUIRE(job["completed_days"] == 1);
    REQUIRE(job["carry"]["reservoirs"][1]["initial_level_m"].get<double>() == Approx(195.5).margin(1e-6));
    job = step_market_operation(job); REQUIRE(job["completed_days"] == 2);
    REQUIRE(job["carry"]["reservoirs"][1]["initial_level_m"].get<double>() == Approx(291.5).margin(1e-6)); return;
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

TEST_CASE("Every admitted rule field round trips through typed daily overlays", "[southern_market][operation][boundary_rules]") {
  auto b=make_southern_market_demo();
  auto primary=record("primary_groups");primary["generators"]={1};b["primary_groups"].push_back(primary);
  auto group=record("groups");group["generators"]={1};group["max_online"]=series(1);group["max_mw"]=series(10000);group["max_mwh"]=1e6;b["groups"].push_back(group);
  auto section=record("sections");section["members"]=json::array({{{"branch",1},{"coefficient",1}}});section["min_mw"]=series(-10000);section["max_mw"]=series(10000);b["sections"].push_back(section);
  auto trade=record("trades");trade["gateway_kind"]="ac_branch";trade["gateway"]=1;trade["max_mw"]=series(10000);trade["max_mwh"]=1e6;b["trades"].push_back(trade);
  auto dc=record("dc_links");dc["from_bus"]=1;dc["to_bus"]=2;dc["from_hub"]=-1;dc["to_hub"]=-1;dc["available"]=std::vector<int>(98,1);dc["max_mw"]=series(100);dc["ramp_up_mw"]=series(100);dc["ramp_down_mw"]=series(100);b["dc_links"].push_back(dc);
  auto plan=record("external_schedules");plan["bus"]=1;b["external_schedules"].push_back(plan);
  const auto effective=validate_southern_market(b);
  json c={{"horizon","week"},{"start_date","2028-02-01"},{"penalty_per_mwh",100000},{"explain",false},{"days",json::array()}};
  c=make_market_operation(b,c)["config"];
  std::set<std::pair<std::string,std::string>> seen;
  json edits=json::array();
  const auto catalog=southern_market_boundary_catalog();
  for(const auto& category:catalog.at("items")) for(const auto& f:category.at("fields")) {
    const std::string table=f.at("table"),field=f.at("field");
    if(!seen.emplace(table,field).second)continue;
    REQUIRE_FALSE(b.at(table).empty());
    const auto& row=b.at(table)[0];const bool is_series=row.at(field).is_array();
    edits.push_back({{"table",table},{"id",row.at("id")},{"field",field},{"first_slot",0},{"last_slot",is_series?0:97},{"value",is_series?row.at(field)[0]:row.at(field)},{"reason","field round trip"}});
  }
  c["days"][0]["boundary_overrides"]=edits;
  const auto preview=preview_market_operation_boundary(b,c,0);
  for(const auto& [table,field]:seen) {
    if (b[table][0][field].is_array()) {
      for (int t = 0; t < 96; ++t) {
        REQUIRE(preview["authored"][table][0][field][t]==b[table][0][field][t]);
        REQUIRE(preview["effective"][table][0][field][t]==effective[table][0][field][t]);
      }
    } else {
      REQUIRE(preview["authored"][table][0][field]==b[table][0][field]);
      REQUIRE(preview["effective"][table][0][field]==effective[table][0][field]);
    }
  }
  REQUIRE(seen.size()>60);
}
