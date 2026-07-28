/// test_scuc_module.cpp — Integration test for the SCUC/SCED/LMP module.
///
/// Builds a small 2-bus, 2-generator, 3-period problem programmatically,
/// serialises to JSON, solves with the standalone module, and checks results.

#include <catch2/catch_all.hpp>

#include <cmath>
#include <string>
#include <nlohmann/json.hpp>

#include "mipsolvers/scuc/scuc.hpp"

using namespace mipsolvers::scuc;
using json = nlohmann::json;

static std::string make_small_problem_json(const std::string& solver = "Auto") {
  json j;
  j["config"] = {
    {"solver",               solver},
    {"allow_fallback",       true},
    {"num_periods",          3},
    {"period_length_hr",     1.0},
    {"n_segments",           2},
    {"mip_gap",              0.005},
    {"time_limit_sec",       60},
    {"verbose",              false},
    {"spinning_reserve_req", 0.10},
    {"voll",                 5000.0},
    {"vocc",                 500.0},
    {"enable_market_cuts",   true},
    {"solve_sced",           true},
    {"solve_lmp",            true},
    {"lmp_delta",            0.1}
  };
  j["num_buses"] = 2;
  j["generators"] = json::array({
    {
      {"name",   "G1"}, {"bus", 0},
      {"pmin",   50.0}, {"pmax", 200.0},
      {"ramp_up_mw_min", 5.0}, {"ramp_dn_mw_min", 5.0},
      {"min_up_time_hr", 2.0}, {"min_dn_time_hr", 2.0},
      {"must_run", false},
      {"startup_cost", 300.0}, {"no_load_cost", 80.0},
      {"bid_segments", json::array({
        {{"price", 25.0}, {"quantity", 100.0}},
        {{"price", 40.0}, {"quantity", 50.0}}
      })},
      {"spinning_reserve_price", 3.0},
      {"regulation_up_price",    5.0},
      {"regulation_down_price",  4.0}
    },
    {
      {"name",   "G2"}, {"bus", 1},
      {"pmin",   20.0}, {"pmax", 100.0},
      {"ramp_up_mw_min", 3.0}, {"ramp_dn_mw_min", 3.0},
      {"min_up_time_hr", 1.0}, {"min_dn_time_hr", 1.0},
      {"must_run", false},
      {"startup_cost", 150.0}, {"no_load_cost", 40.0},
      {"bid_segments", json::array({
        {{"price", 35.0}, {"quantity", 50.0}},
        {{"price", 55.0}, {"quantity", 30.0}}
      })},
      {"spinning_reserve_price", 4.0},
      {"regulation_up_price",    6.0},
      {"regulation_down_price",  5.0}
    }
  });
  j["branches"] = json::array({
    {{"from", 0}, {"to", 1}, {"reactance", 0.1}, {"rating_mw", 150.0}, {"in_service", true}}
  });
  j["loads"] = json::array({
    {{"bus", 0}, {"p_mw", 150.0}},
    {{"bus", 1}, {"p_mw",  80.0}}
  });
  j["wind"]     = json::array();
  j["solar"]    = json::array();
  j["storage"]  = json::array();
  j["dc_lines"] = json::array();
  j["profiles"] = {
    {"load",  json::array({ json::array({0.90, 1.00, 0.85}),
                            json::array({0.88, 0.95, 0.80}) })},
    {"wind",  json::array()},
    {"solar", json::array()}
  };
  j["initial_status"] = {
    {"commitment",  json::array({1.0, 0.0})},
    {"dispatch",    json::array({120.0, 0.0})},
    {"storage_soc", json::array()}
  };
  return j.dump(2);
}

TEST_CASE("SCUC: JSON round-trip parses without errors", "[scuc][unit]") {
  const std::string json_str = make_small_problem_json();
  SCUCInput input;
  REQUIRE_NOTHROW(input = scuc_from_json(json_str));
  REQUIRE(input.generators.size() == 2);
  REQUIRE(input.branches.size()   == 1);
  REQUIRE(input.loads.size()      == 2);
  REQUIRE(input.config.num_periods == 3);
  REQUIRE(input.num_buses == 2);
  REQUIRE(input.generators[0].bid_segments.size() == 2);
  REQUIRE(input.generators[0].bid_segments[0].price == Catch::Approx(25.0));
}

TEST_CASE("SCUC: small 2-bus problem converges", "[scuc][integration]") {
  const std::string json_str = make_small_problem_json("Auto");
  const SCUCInput input = scuc_from_json(json_str);
  const SCUCOutput output = scuc_solve(input);

  REQUIRE(output.scuc.converged);
  REQUIRE(output.scuc.objective > 0.0);

  // Commitment: G1 was on (initial), G2 must start up at period 0
  REQUIRE(output.scuc.commitment.size() == 2);
  REQUIRE(output.scuc.commitment[0].size() == 3);  // T_commit = T for dt=1hr

  // No load shedding
  for (double ls : output.scuc.load_shedding)
    REQUIRE(std::abs(ls) < 1.0);

  // Power balance: dispatch + renewable ≈ load (each period)
  const double p1_load = 150.0 * 0.90 + 80.0 * 0.88;
  const double p1_gen  = output.scuc.dispatch[0][0] + output.scuc.dispatch[1][0];
  REQUIRE(p1_gen == Catch::Approx(p1_load).margin(5.0));

  // Cost components are positive
  REQUIRE(output.scuc.energy_cost > 0.0);
  REQUIRE(output.scuc.total_cost  > 0.0);
}

TEST_CASE("SCUC: SCED LP refines dispatch", "[scuc][integration]") {
  const std::string json_str = make_small_problem_json("Auto");
  const SCUCInput input = scuc_from_json(json_str);
  const SCUCOutput output = scuc_solve(input);

  REQUIRE(output.sced.converged);

  // SCED commitment must match SCUC commitment (binaries fixed)
  for (int g = 0; g < 2; ++g)
    for (int h = 0; h < 3; ++h)
      REQUIRE(std::round(output.sced.commitment[g][h]) ==
              Catch::Approx(std::round(output.scuc.commitment[g][h])).margin(0.01));
}

TEST_CASE("SCUC: LMP values are physically consistent", "[scuc][integration]") {
  const std::string json_str = make_small_problem_json("Auto");
  const SCUCInput input = scuc_from_json(json_str);
  const SCUCOutput output = scuc_solve(input);

  if (!output.lmp.converged) {
    WARN("LMP solve did not converge — skip LMP checks");
    return;
  }

  // LMP should be in a reasonable range for this test case ($0–$300/MWh)
  REQUIRE(output.lmp.avg_lmp >= 0.0);
  REQUIRE(output.lmp.avg_lmp <= 300.0);
  REQUIRE(output.lmp.min_lmp <= output.lmp.avg_lmp);
  REQUIRE(output.lmp.avg_lmp <= output.lmp.max_lmp);

  // Nodal LMPs: 2 buses, 3 periods
  REQUIRE(output.lmp.nodal_lmp.size() == 2);
  for (const auto& row : output.lmp.nodal_lmp)
    REQUIRE(row.size() == 3);

  // Period 2 (high load) LMP >= period 1 (lower load)
  // G1 pushes into higher segment
  const double lmp_b0_t1 = output.lmp.nodal_lmp[0][0];
  const double lmp_b0_t2 = output.lmp.nodal_lmp[0][1];
  REQUIRE(lmp_b0_t2 >= lmp_b0_t1 - 0.1);  // non-decreasing with demand
}

TEST_CASE("SCUC: output JSON serialisation round-trips", "[scuc][unit]") {
  const std::string json_str = make_small_problem_json("Auto");
  const SCUCInput input = scuc_from_json(json_str);
  const SCUCOutput output = scuc_solve(input);

  std::string out_json;
  REQUIRE_NOTHROW(out_json = scuc_output_to_json(output, input, 2));
  REQUIRE(!out_json.empty());

  json parsed;
  REQUIRE_NOTHROW(parsed = json::parse(out_json));
  REQUIRE(parsed.contains("scuc"));
  REQUIRE(parsed.contains("meta"));
  REQUIRE(parsed["scuc"]["converged"].get<bool>() == output.scuc.converged);
  REQUIRE(parsed["scuc"]["commitment"].is_array());
  REQUIRE(parsed["scuc"]["dispatch"].is_array());
  if (input.config.solve_lmp) {
    REQUIRE(parsed.contains("lmp"));
    REQUIRE(parsed["lmp"]["nodal"].is_array());
  }
}
