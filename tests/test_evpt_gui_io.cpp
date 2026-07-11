// test_evpt_gui_io.cpp
// ──────────────────────────────────────────────────────────────────────────
// GUI-integration layer for ev_power_traffic: demo scenario builders +
// JSON serialisation (io/evpt_json).  Exercises exactly what the
// /api/session/run_ev_traffic endpoint does: build a demo problem, parse
// options from JSON, run a formulation, serialize the result.

#include <algorithm>

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
  REQUIRE(names.size() == 3);

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
    CHECK(sc.value("schema_version", "") == "1.0");
    REQUIRE(sc.contains("traffic"));
    REQUIRE(!sc["traffic"]["nodes"].empty());
    CHECK(sc["traffic"]["nodes"].front().contains("x"));
    REQUIRE(sc.contains("charging_stations"));
    CHECK(sc["charging_stations"].front().contains("bus"));
  }
}

TEST_CASE("EVPT comprehensive demo exercises joint traffic-power workflow",
          "[evpt][gui][comprehensive]") {
  auto prob = io::build_evpt_demo_comprehensive();
  REQUIRE(prob.traffic.nodes.size() == 25);
  REQUIRE(prob.traffic.links.size() == 40);
  REQUIRE(prob.routes.size() == 7);
  REQUIRE(prob.demands.size() == 4);
  REQUIRE(prob.system.ac.charging_stations.size() == 3);
  REQUIRE(prob.station_prices.size() == 3);

  int dynamic_links = 0;
  for (const auto& link : prob.traffic.links) {
    if (!link.capacity_profile_veh_per_hr.empty()) ++dynamic_links;
  }
  CHECK(dynamic_links == 2);
  CHECK(std::any_of(prob.routes.begin(), prob.routes.end(), [](const auto& route) {
    return std::any_of(route.charging_stops.begin(), route.charging_stops.end(),
                       [](const auto& stop) { return stop.v2g_capable; });
  }));

  CTMJointWelfareOptions options;
  options.max_iterations = 6;
  options.price_convergence_tol = 5e-3;
  options.price_update_step = 0.6;
  options.use_dcopf_prices = true;
  options.evpt_opts.num_steps = 8;
  options.evpt_opts.time_step_hr = 1.0;
  options.evpt_opts.allow_v2g = true;
  options.due_opts.max_iterations = 20;
  options.due_opts.convergence_tol = 5e-3;
  options.dcopf_opts.compute_lmp = true;
  options.dcopf_opts.load_shedding = true;

  const auto result = simulate_ev_power_traffic_ctm_joint(prob, options);
  CHECK(result.iterations >= 1);
  CHECK(!result.history.empty());
  CHECK(result.lmp_by_step.size() == 8);
  CHECK(result.opf_by_step.size() == 8);
  CHECK(result.final_ctm_due.flow_by_demand_route.size() == 4);
  CHECK(result.final_ctm_due.station_ev_load_kw.size() == 3);
  CHECK(result.final_ctm_due.total_delivered_energy_kwh > 0.0);

  const json scenario = io::evpt_scenario_to_json(prob);
  REQUIRE(scenario.contains("power_network"));
  CHECK(scenario["power_network"]["buses"].size() ==
        prob.system.ac.buses.size());
  CHECK(scenario["power_network"]["branches"].size() ==
        prob.system.ac.branches.size());
}

TEST_CASE("EVPT JSON: problem round trip", "[evpt][gui][json]") {
  const auto prob = io::build_evpt_demo_small();
  const json sc = io::evpt_scenario_to_json(prob);

  // Re-parse the echoed scenario into a fresh problem: graph shape survives.
  EVPowerTrafficProblem back;
  json pj;
  pj["traffic"] = sc["traffic"];
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
  CHECK(back.demands.front().willingness_to_pay_per_vehicle ==
        Approx(prob.demands.front().willingness_to_pay_per_vehicle));
  CHECK(back.routes.back().charging_stops.front().max_discharge_kw_per_vehicle ==
        Approx(prob.routes.back().charging_stops.front().max_discharge_kw_per_vehicle));

  json full = sc;
  EVPowerTrafficProblem with_prices;
  io::evpt_problem_from_json(full, with_prices);
  REQUIRE(with_prices.station_prices.size() == prob.station_prices.size());
  CHECK(with_prices.station_prices.front().price_per_kwh ==
        prob.station_prices.front().price_per_kwh);
}

TEST_CASE("EVPT JSON: GUI advanced options are mapped", "[evpt][gui][options]") {
  const json model = {{"assignment_model", "system_optimal_milp"},
                      {"allow_unserved_travel_demand", true},
                      {"default_station_power_kw", 250.0},
                      {"system_optimal_max_nodes", 99}};
  const auto ev = io::evpt_options_from_json(model);
  CHECK(ev.assignment_model == AssignmentModel::SystemOptimalMILP);
  CHECK(ev.allow_unserved_travel_demand);
  CHECK(ev.default_station_power_kw == Approx(250.0));
  CHECK(ev.system_optimal_max_nodes == 99);

  const auto due = io::evpt_due_options_from_json(
      json{{"due_mode", "full_endogenous"},
           {"max_departure_iterations", 12},
           {"departure_convergence_tol", 2e-4}});
  CHECK(due.due_mode == DUEMode::FullEndogenous);
  CHECK(due.max_departure_iterations == 12);
  CHECK(due.departure_convergence_tol == Approx(2e-4));

  const auto joint = io::evpt_joint_optimizer_options_from_json(
      json{{"mode", "certified_ltm_user_benefit_pwl_milp"},
           {"max_nodes", 123},
           {"unserved_trip_penalty", 456.0},
           {"full_joint_ltm_pwl_segments", 9}});
  CHECK(joint.mode ==
        JointOptimizerMode::CertifiedFullJointLtmUserBenefitPwlMILP);
  CHECK(joint.max_nodes == 123);
  CHECK(joint.unserved_trip_penalty == Approx(456.0));
  CHECK(joint.full_joint_ltm_pwl_segments == 9);
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
