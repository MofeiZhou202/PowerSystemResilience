// test_evpt_gui_io.cpp
// ──────────────────────────────────────────────────────────────────────────
// GUI-integration layer for ev_power_traffic: demo scenario builders +
// JSON serialisation (io/evpt_json).  Exercises exactly what the
// /api/session/run_ev_traffic endpoint does: build a demo problem, parse
// options from JSON, run a formulation, serialize the result.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include "hacdcpf/io/evpt_demo_cases.hpp"
#include "hacdcpf/io/evpt_json.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;
using json = nlohmann::json;
using Approx = Catch::Approx;

TEST_CASE("EVPT demo cases: build and scenario echo", "[evpt][gui][demo]") {
  const auto names = io::evpt_demo_case_names();
  REQUIRE(names.size() == 2);

  for (const auto& name : names) {
    const auto prob = io::build_evpt_demo_case(name);
    CHECK(!prob.traffic.nodes.empty());
    CHECK(!prob.traffic.links.empty());
    CHECK(!prob.routes.empty());
    CHECK(!prob.demands.empty());
    CHECK(!prob.system.ac.charging_stations.empty());

    // Every route link exists in the graph, and every stop's station exists.
    for (const auto& r : prob.routes) {
      for (int li : r.link_indices) {
        bool found = false;
        for (const auto& l : prob.traffic.links) {
          if (l.index == li) { found = true; break; }
        }
        CHECK(found);
      }
      for (const auto& s : r.charging_stops) {
        bool found = false;
        for (const auto& cs : prob.system.ac.charging_stations) {
          if (cs.index == s.station_id) { found = true; break; }
        }
        CHECK(found);
      }
    }

    // Route links form a connected O→D chain.
    for (const auto& r : prob.routes) {
      int cur = r.origin_node;
      for (int li : r.link_indices) {
        const evpt::TrafficLink* lnk = nullptr;
        for (const auto& l : prob.traffic.links) {
          if (l.index == li) { lnk = &l; break; }
        }
        REQUIRE(lnk != nullptr);
        REQUIRE(lnk->from_node == cur);
        cur = lnk->to_node;
      }
      CHECK(cur == r.destination_node);
    }

    // Scenario echo contains nodes with coordinates and station→bus mapping.
    const json sc = io::evpt_scenario_to_json(prob);
    REQUIRE(sc.contains("nodes"));
    REQUIRE(!sc["nodes"].empty());
    CHECK(sc["nodes"].front().contains("x"));
    REQUIRE(sc.contains("charging_stations"));
    CHECK(sc["charging_stations"].front().contains("bus"));
  }
}

TEST_CASE("EVPT JSON: problem round trip", "[evpt][gui][json]") {
  const auto prob = io::build_evpt_demo_small();
  const json sc = io::evpt_scenario_to_json(prob);

  // Re-parse the echoed scenario into a fresh problem: graph shape survives.
  EVPowerTrafficProblem back;
  json pj;
  pj["traffic"] = {{"nodes", sc["nodes"]}, {"links", sc["links"]}};
  pj["routes"] = sc["routes"];
  pj["demands"] = sc["demands"];
  io::evpt_problem_from_json(pj, back);

  REQUIRE(back.traffic.nodes.size() == prob.traffic.nodes.size());
  REQUIRE(back.traffic.links.size() == prob.traffic.links.size());
  REQUIRE(back.routes.size() == prob.routes.size());
  REQUIRE(back.demands.size() == prob.demands.size());
  CHECK(back.traffic.links.front().capacity_veh_per_hr ==
        Approx(prob.traffic.links.front().capacity_veh_per_hr));
  CHECK(back.demands.front().vehicles == Approx(prob.demands.front().vehicles));
}

TEST_CASE("EVPT JSON: mode B (CTM-DUE) run and serialize", "[evpt][gui][ctm_due]") {
  auto prob = io::build_evpt_demo_small();

  const json oj = {{"num_steps", 6},
                   {"time_step_hr", 1.0},
                   {"allow_v2g", true}};
  auto ev_opts = io::evpt_options_from_json(oj);
  auto ctm_opts = io::evpt_ctm_options_from_json(json::object());
  auto due_opts = io::evpt_due_options_from_json(json{{"max_iterations", 20}});

  const auto res =
      simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);
  const json out = io::evpt_ctm_due_result_to_json(res);

  CHECK(out.at("iterations").get<int>() >= 1);
  CHECK(out.at("total_delivered_energy_kwh").get<double>() > 0.0);
  REQUIRE(out.contains("ctm"));
  CHECK(out["ctm"].at("dt_ctm_hr").get<double>() > 0.0);
  CHECK(out["ctm"].at("steps_per_sim_step").get<int>() >= 1);
  CHECK(!out["ctm"].at("occupancy_veh").empty());
  REQUIRE(out.contains("station_ev_load_kw"));
  CHECK(!out["station_ev_load_kw"].empty());
  // Demo has a V2G-capable stop and a price spread → arbitrage export.
  CHECK(out.at("total_v2g_energy_kwh").get<double>() > 0.0);
}

TEST_CASE("EVPT JSON: mode C (joint welfare) run and serialize",
          "[evpt][gui][ctm_joint]") {
  auto prob = io::build_evpt_demo_small();

  json oj;
  oj["max_iterations"] = 5;
  oj["price_update_step"] = 0.6;
  oj["use_dcopf_prices"] = true;
  oj["evpt_options"] = {{"num_steps", 6}, {"time_step_hr", 1.0}};
  oj["due_options"] = {{"max_iterations", 15}, {"convergence_tol", 5e-3}};
  auto opts = io::evpt_ctm_joint_options_from_json(oj);
  opts.dcopf_opts.compute_lmp = true;
  opts.dcopf_opts.load_shedding = true;

  const auto res = simulate_ev_power_traffic_ctm_joint(prob, opts);
  const json out = io::evpt_ctm_joint_result_to_json(res);

  CHECK(out.at("iterations").get<int>() >= 1);
  REQUIRE(out.contains("history"));
  CHECK(!out["history"].empty());
  REQUIRE(out.contains("lmp_by_step"));
  CHECK(!out["lmp_by_step"].empty());
  REQUIRE(out.contains("station_prices_per_step"));
  CHECK(!out["station_prices_per_step"].empty());
  REQUIRE(out.contains("ctm_due"));
}

TEST_CASE("EVPT JSON: mode D (joint optimizer) run and serialize",
          "[evpt][gui][joint_opt]") {
  auto prob = io::build_evpt_demo_small();

  const json oj = {{"num_steps", 6},   {"time_step_hr", 1.0},
                   {"mode", "social_welfare"}, {"include_dcopf", true},
                   {"allow_v2g", true}};
  auto opts = io::evpt_joint_optimizer_options_from_json(oj);

  const auto res = solve_joint_optimizer(prob, opts);
  const json out = io::evpt_joint_optimizer_result_to_json(res);

  CHECK(out.at("feasible").get<bool>());
  CHECK(out.contains("solver_backend"));
  CHECK(out.contains("social_welfare"));
  REQUIRE(out.contains("route_flow"));
  CHECK(!out["route_flow"].empty());
  // Formulation D LMPs are placeholders — must NOT be serialized.
  CHECK(!out.contains("lmp_by_step"));
}
