#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "hacdcpf/market/southern_market.hpp"
#include <cmath>
#include <numeric>

using namespace hacdcpf::market;
using J = nlohmann::json;
using Catch::Approx;

namespace {
J fixed_config(int count = 2) {
  auto c = market_forecast_defaults(); c["sample_count"] = count; c["operation"]["explain"] = false;
  for (auto& m : c["marginals"]) m["distribution"] = "fixed";
  return c;
}
J finish(J job) {
  while (job["status"] == "ready" || job["status"] == "running") job = step_market_forecast(job);
  return job;
}
}

TEST_CASE("Forecast sampler is reproducible, bounded and validates correlation", "[market_forecast]") {
  const auto b = make_southern_market_example(); auto c = market_forecast_defaults(); c["sample_count"] = 16;
  auto first = make_market_forecast(b, c); REQUIRE(first == make_market_forecast(b, c));
  c["seed"] = 1; REQUIRE(first["scenarios"] != make_market_forecast(b, c)["scenarios"]);
  c["correlation"][0][1] = 0.5; REQUIRE_THROWS(make_market_forecast(b, c));
  c["correlation"][1][0] = 0.5; REQUIRE_NOTHROW(make_market_forecast(b, c));
  c["correlation"][0][2] = c["correlation"][2][0] = .9;
  c["correlation"][1][2] = c["correlation"][2][1] = -.9; REQUIRE_THROWS(make_market_forecast(b, c));
  c = fixed_config(); c["unexpected"] = 1; REQUIRE_THROWS(make_market_forecast(b, c));
  c = fixed_config(); c["marginals"][0]["center"] = {1}; REQUIRE_THROWS(make_market_forecast(b, c));
  c = fixed_config(); c["operation"]["horizon"] = "month"; REQUIRE_THROWS(make_market_forecast(b, c));
  c = fixed_config(); c["mode"] = "interval"; REQUIRE_THROWS(make_market_forecast(b, c));
  c["temporal_rho"] = 0; REQUIRE_NOTHROW(make_market_forecast(b, c));
}

TEST_CASE("Forecast eighth day is sampled and enters seventh-day representatives only", "[market_forecast][lookahead]") {
  auto c = fixed_config(1); c["marginals"][0]["upper"] = 2; c["marginals"][0]["center"][7] = 1.5;
  const auto b = make_southern_market_example();
  const auto job = make_market_forecast(b,c);
  REQUIRE(job["total_days"] == 7); REQUIRE(job["forecast_days"] == 8);
  const auto config = job["scenarios"][0]["config"];
  REQUIRE(config["days"].size() == 8); REQUIRE(config["reference_days"].size() == 8);
  REQUIRE(config["days"][7]["load_scale"] == 1.5);
  const auto p = preview_market_operation_boundary(b,config,6);
  REQUIRE(p["lookahead"]["source_day"] == 7);
  REQUIRE(p["effective"]["buses"][0]["load_mw"][96] == 150);
  REQUIRE(p["effective"]["buses"][0]["load_mw"][0] == 100);
  const auto completed = finish(job);
  REQUIRE(completed["completed_days"] == 7);
  REQUIRE(completed["statistics"]["periods"].size() == 672);
  REQUIRE(completed["statistics"]["complete_scenarios_with_limit"] == 0);
  REQUIRE(completed["statistics"]["complete_scenarios_unproven"] == 0);
  auto limited = completed;
  limited["scenarios"][0]["days"][0]["stages"]["scuc"]["limit_reached"] = true;
  limited["scenarios"][0]["days"][0]["stages"]["scuc"]["optimality_proven"] = false;
  REQUIRE(market_forecast_statistics(limited)["complete_scenarios_with_limit"] == 1);
  REQUIRE(market_forecast_statistics(limited)["complete_scenarios_unproven"] == 1);
  auto invalid = c; invalid["marginals"][0]["center"][7] = 11;
  REQUIRE_THROWS(make_market_forecast(b,invalid));
}

TEST_CASE("Forecast uniform copula has expected moments and singular correlation", "[market_forecast]") {
  const auto b = make_southern_market_example(); auto c = market_forecast_defaults();
  c["sample_count"] = 512; c["temporal_rho"] = 0;
  c["correlation"][0][1] = c["correlation"][1][0] = 1;
  for (auto& m : c["marginals"]) { m["lower"] = 0; m["upper"] = 2; }
  double sum = 0, sumsq = 0; int n = 0;
  for (int seed = 0; seed < 8; ++seed) {
    c["seed"] = seed; const auto job = make_market_forecast(b, c);
    for (const auto& s : job["scenarios"]) {
      const auto& d = s["config"]["days"][0]; const double v = d["load_scale"];
      REQUIRE(v >= 0); REQUIRE(v <= 2);
      REQUIRE(v == Approx(d["wind_scale"].get<double>()).margin(1e-10));
      sum += v; sumsq += v*v; ++n;
    }
  }
  REQUIRE(n == 4096); REQUIRE(sum/n == Approx(1).margin(.02));
  REQUIRE(sumsq/n-std::pow(sum/n, 2) == Approx(1.0/3).margin(.02));
  SUCCEED("4096 draws: mean=" + std::to_string(sum/n) + " variance=" + std::to_string(sumsq/n-std::pow(sum/n,2)));
}

TEST_CASE("Forecast AR1 preserves specified daily dependence", "[market_forecast]") {
  auto c = fixed_config(512); c["temporal_rho"] = .6;
  auto& m = c["marginals"][0]; m["distribution"] = "clipped_normal"; m["center"] = std::vector<double>(7,5); m["lower"] = 0; m["upper"] = 10; m["sigma"] = .2;
  const auto job = make_market_forecast(make_southern_market_example(),c);
  double xy=0, xx=0, yy=0;
  for (const auto& s : job["scenarios"]) {
    const double x = (s["config"]["days"][0]["load_scale"].get<double>()-5)/.2;
    const double y = (s["config"]["days"][1]["load_scale"].get<double>()-5)/.2;
    xy+=x*y; xx+=x*x; yy+=y*y;
  }
  REQUIRE(xy/std::sqrt(xx*yy) == Approx(.6).margin(.1));
  SUCCEED("AR1 empirical correlation=" + std::to_string(xy/std::sqrt(xx*yy)));
}

TEST_CASE("Forecast line Delta Pij statistics preserve stable identities", "[market_forecast]") {
  auto b = make_southern_market_example(); b["buses"][0]["load_mw"] = std::vector<double>(98,0);
  auto node=b["buses"][0]; node["id"]=2; node["load_mw"]=std::vector<double>(98,100); b["buses"].push_back(node);
  auto line=make_southern_market_demo()["branches"][0]; line["min_mw"]=std::vector<double>(98,-50); line["max_mw"]=std::vector<double>(98,50); b["branches"].push_back(line);
  auto c=fixed_config(); c["operation"]["explain"]=true;
  auto job=make_market_forecast(b,c);
  for(auto& d:job["scenarios"][1]["config"]["days"]) d["line_limit_scale"] = .5;
  job=finish(job);
  REQUIRE(job["statistics"]["lines"][0]["id"]==1);
  REQUIRE(job["statistics"]["week_delta_pij_peak_mw"]["mean"].get<double>()==Approx(62.5).margin(1e-6));
  REQUIRE(job["statistics"]["week_delta_pij_peak_mw"]["event_probability_valid_only"].get<double>()==Approx(1));
  REQUIRE(job["scenarios"][1]["days"][0]["lines"][0]["delta_pij_mw"][0].get<double>()==Approx(75).margin(1e-6));
  REQUIRE(job["scenarios"][1]["days"][0]["counterfactuals"][0]["reduction_overload_mwh"].get<double>()==Approx(600).margin(1e-6));
}

TEST_CASE("Forecast interval strata and alternate marginals preserve their meanings", "[market_forecast]") {
  const auto b = make_southern_market_example(); auto c = fixed_config(16);
  c["mode"] = "interval"; c["temporal_rho"] = 0;
  c["marginals"][0]["distribution"] = "interval"; c["marginals"][0]["lower"] = 0; c["marginals"][0]["upper"] = 2;
  auto job = make_market_forecast(b, c); std::vector<int> bins(16, 0);
  for (const auto& s : job["scenarios"]) ++bins.at(static_cast<int>(s["config"]["days"][0]["load_scale"].get<double>()*8));
  for (int v : bins) REQUIRE(v == 1);
  const auto stats = market_forecast_statistics(job);
  REQUIRE(stats["week_delta_p_peak_mw"]["event_probability_valid_only"].is_null());
  REQUIRE(stats["week_delta_p_peak_mw"]["probability_bounds"].is_null());
  c = fixed_config(32); c["marginals"][0]["distribution"] = "triangular";
  c["marginals"][1]["distribution"] = "clipped_normal"; c["marginals"][1]["sigma"] = 10;
  job = make_market_forecast(b, c); int endpoints = 0;
  for (const auto& s : job["scenarios"]) {
    const double v = s["config"]["days"][0]["load_scale"], w = s["config"]["days"][0]["wind_scale"];
    REQUIRE(v >= .9); REQUIRE(v <= 1.1); REQUIRE(w >= .9); REQUIRE(w <= 1.1);
    if (w == .9 || w == 1.1) ++endpoints;
  }
  REQUIRE(endpoints > 20);
}

TEST_CASE("Forecast weekly Delta P statistics match the two-path hand oracle", "[market_forecast]") {
  auto c = fixed_config(); c["operation"]["explain"] = true;
  c["marginals"][0]["center"] = std::vector<double>(7, 2); c["marginals"][0]["lower"] = 1; c["marginals"][0]["upper"] = 3;
  auto job = make_market_forecast(make_southern_market_example(), c);
  for (int s = 0; s < 2; ++s) for (auto& d : job["scenarios"][s]["config"]["days"]) d["load_scale"] = s ? 3 : 1;
  job = finish(job); REQUIRE(job["completed_days"] == 14);
  const auto& stats = job["statistics"]; REQUIRE(stats["complete_scenarios"] == 2);
  REQUIRE(stats["week_delta_p_peak_mw"]["event_probability_valid_only"].get<double>() == Approx(.5));
  REQUIRE(stats["week_deficit_mwh"]["mean"].get<double>() == Approx(8400).margin(1e-6));
  REQUIRE(stats["periods"].size() == 672);
  REQUIRE(stats["periods"][0]["delta_p_abs_sum_mw"]["mean"].get<double>() == Approx(50).margin(1e-6));
  REQUIRE(stats["nodes"][0]["id"] == 1);
  REQUIRE(stats["nodes"][0]["peak_mw"]["sample_event_fraction"].get<double>() == Approx(.5));
  const auto& cause = job["scenarios"][1]["days"][0]["counterfactuals"][0];
  REQUIRE(cause["reference_value"] == 2); REQUIRE(cause["valid"] == true);
  REQUIRE(cause["reduction_deficit_mwh"].get<double>() == Approx(2400).margin(1e-6));
  REQUIRE(job["scenarios"][1]["days"][0]["nodes"][0]["delta_p_mw"][0].get<double>() == Approx(100).margin(1e-6));
  auto failed = job["scenarios"][0]; failed["status"] = "failed"; failed["days"] = J::array(); job["scenarios"].push_back(failed);
  auto partial = market_forecast_statistics(job);
  REQUIRE(partial["week_delta_p_peak_mw"]["unknown_count"] == 1);
  REQUIRE(partial["week_delta_p_peak_mw"]["probability_bounds"][0].get<double>() == Approx(1.0/3));
  REQUIRE(partial["week_delta_p_peak_mw"]["probability_bounds"][1].get<double>() == Approx(2.0/3));
  REQUIRE(partial["week_deficit_mwh"]["mean"].get<double>() == Approx(8400).margin(1e-6));
  job["config"]["mode"] = "interval"; partial = market_forecast_statistics(job);
  REQUIRE(partial["week_delta_p_peak_mw"]["event_probability_valid_only"].is_null());
  REQUIRE(partial["week_delta_p_peak_mw"]["sample_event_fraction"].get<double>() == Approx(.5));
}

TEST_CASE("Forecast failures remain unknown and generation and demand bids are independent", "[market_forecast]") {
  auto b = make_southern_market_example(); auto c = fixed_config();
  b["areas"][0]["reserve_up_mw"] = std::vector<double>(98, 400);
  auto job = finish(make_market_forecast(b, c));
  REQUIRE(job["finished_scenarios"] == 2); REQUIRE(job["completed_days"] == 0);
  REQUIRE(job["statistics"]["complete_scenarios"] == 0);
  REQUIRE(job["statistics"]["week_delta_p_peak_mw"]["mean"].is_null());
  REQUIRE(job["statistics"]["week_delta_p_peak_mw"]["probability_bounds"] == J({0,1}));
  b = make_southern_market_example(); b["generators"][0]["must_on"] = std::vector<int>(98,1);
  auto conflicting = make_market_forecast(b,c);
  conflicting["scenarios"][0]["config"]["days"][0]["generator_outages"] = {1};
  conflicting = finish(conflicting);
  REQUIRE(conflicting["scenarios"][0]["status"] == "failed");
  REQUIRE(conflicting["scenarios"][1]["status"] == "completed");
  REQUIRE(conflicting["statistics"]["failed_scenarios"] == 1);
  b = make_southern_market_example();
  auto demand = make_southern_market_demo()["controllable_loads"][0]; demand["bus"] = 1; b["controllable_loads"].push_back(demand);
  auto operation = make_market_operation(b, c["operation"]); operation["config"]["days"][0]["generator_bid_scale"] = .1;
  operation = step_market_operation(operation); REQUIRE(operation["completed_days"] == 1);
  REQUIRE(operation["days"][0]["stages"]["sced"]["objective"].get<double>() == Approx(49000).margin(1e-6));
  operation = make_market_operation(b, c["operation"]); operation["config"]["days"][0]["load_bid_scale"] = .1;
  const auto preview = preview_market_operation_boundary(b,operation["config"],0);
  REQUIRE(preview["authored"]["controllable_loads"][0]["compensation_per_mwh"][0] == 9);
  REQUIRE(preview["authored"]["controllable_loads"][0]["compensation_per_mwh"][96] == 90);
  operation = step_market_operation(operation); REQUIRE(operation["completed_days"] == 1);
  // 24h at D compensation 9, 0.5h at D+1 compensation 90; thermal 80 MW at 200.
  REQUIRE(operation["days"][0]["stages"]["sced"]["objective"].get<double>() == Approx(24*(80*200+20*9)+.5*(80*200+20*90)).margin(1e-6));
}
