// test_evpt_fidelity_bound.cpp
// ──────────────────────────────────────────────────────────────────────────
// Numerical validation of the TSN↔CTM fidelity-bound predictions
// (docs/latex/paper/ev_power_traffic_joint_opt/BOUND_SKETCH.md).
//
// We use the module's CTM replay as the fidelity oracle and check the two
// core predictions of the bound sketch:
//
//   Prediction A (Prop. 2, §2 of the sketch): in an UNCONGESTED case the
//   experienced (CTM) route travel time equals the free-flow route travel
//   time, so Δτ = 0 and the free-flow abstraction is exact.
//
//   Prediction B (§1, Lemma 2): in a CONGESTED case the experienced route
//   travel time EXCEEDS free-flow (Δτ > 0), the saturated set S is non-empty,
//   and Δτ increases monotonically with loading (more demand ⇒ more delay).
//
// The "free-flow route time" reference is Σ_link free_flow_time_hr over the
// route links — exactly the τ^ff of the 2022 TSN arc endpoints.
//
// This is a numerical probe, not a pass/fail unit test of the module; the
// CHECKs encode the bound's qualitative predictions so a regression would
// surface. Run the printout with:
//   ./tests/test_evpt_fidelity_bound "[fidelity_analysis]"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <vector>

#include "hacdcpf/ev_power_traffic/ev_power_traffic_simulation.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;
using Approx = Catch::Approx;

namespace {

// Minimal power system: one slack bus + generator + one station at the
// destination bus so the route's charging stop resolves.
HybridPowerSystem make_min_system() {
  HybridPowerSystem sys;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.in_service = true;
  sys.ac.buses.push_back(b1);

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.is_slack = true;
  g.pmax_mw = 100.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  sys.ac.generators.push_back(g);

  ChargingStation stn;
  stn.index = 101;
  stn.bus = 1;
  stn.in_service = true;
  stn.max_power_kw = 1.0e6;
  stn.power_factor = 1.0;
  sys.ac.charging_stations.push_back(stn);

  Charger ch;
  ch.index = 1;
  ch.station_id = 101;
  ch.in_service = true;
  ch.p_ch_max_kw = 1.0e6;
  sys.ac.chargers.push_back(ch);

  return sys;
}

// Two-link corridor with a downstream bottleneck. Each 2-km link has a
// 0.10-h free-flow time. The entry link admits up to 20 veh/h, while the
// downstream link admits 5 veh/h, so sustained inflow above 5 veh/h forms
// an internal queue instead of being explained solely by source rejection.
//
// SUSTAINED-INFLOW ORACLE (option 1): rather than one OD lump (which the
// injector meters to ~recv and drops the rest), we spread the demand across a
// window of `n_dep` consecutive sim departure steps, each carrying an equal
// share. The per-step arrival rate is  (total / n_dep) / dt_sim  [veh/h]; when
// this exceeds the link's capacity flow q_cap the entry cell stays full and
// occupancy climbs onto the congested FD branch — a genuine standing queue,
// not a transient pulse. Total vehicles is held ≤ what the window can admit so
// the injector's remainder-drop does not silently truncate the study.
EVPowerTrafficProblem make_corridor_problem(double veh_per_dep_step,
                                            int n_dep_steps) {
  EVPowerTrafficProblem prob;
  prob.system = make_min_system();

  TrafficGraph g;
  g.nodes = {{1, "origin"}, {2, "bottleneck"}, {3, "dest"}};

  TrafficLink l;
  l.index = 11;
  l.from_node = 1;
  l.to_node = 2;
  l.length_km = 2.0;
  l.free_flow_time_hr = 0.10;      // τ^ff for this link
  l.capacity_veh_per_hr = 20.0;    // q_cap: capacity/critical flow
  l.jam_vehicles = 80.0;           // congested FD branch headroom
  g.links.push_back(l);
  l.index = 12;
  l.from_node = 2;
  l.to_node = 3;
  l.capacity_veh_per_hr = 5.0;
  g.links.push_back(l);
  prob.traffic = g;

  RouteAlternative r;
  r.index = 1;
  r.origin_node = 1;
  r.destination_node = 3;
  r.link_indices = {11, 12};
  RouteChargingStop stop;
  stop.station_id = 101;
  stop.requested_energy_kwh_per_vehicle = 5.0;
  stop.dwell_steps = 1;
  stop.max_charge_kw_per_vehicle = 50.0;
  r.charging_stops.push_back(stop);
  prob.routes.push_back(r);

  // One demand per departure step, same OD/route, each `veh_per_dep_step`.
  for (int k = 0; k < n_dep_steps; ++k) {
    EVDemand d;
    d.index = 1 + k;
    d.origin_node = 1;
    d.destination_node = 3;
    d.departure_step = k;
    d.vehicles = veh_per_dep_step;
    d.candidate_route_indices = {1};
    prob.demands.push_back(d);
  }

  return prob;
}

// τ^ff for the single route = Σ free_flow_time_hr over route links.
double free_flow_route_time(const EVPowerTrafficProblem& prob) {
  double tau = 0.0;
  const auto& route = prob.routes.front();
  for (int li : route.link_indices) {
    for (const auto& link : prob.traffic.links) {
      if (link.index == li) { tau += link.free_flow_time_hr; break; }
    }
  }
  return tau;
}

constexpr double kFreeFlowTau = 0.20;
constexpr double kOracleToleranceHr = 0.01;

// Run the CTM replay under a sustained inflow and return the EXPERIENCED link
// travel time (τ^CTM) — the fidelity oracle.
//
// veh_per_dep_step: vehicles released each sim step; n_dep: number of such
// steps. Per-step arrival rate = veh_per_dep_step / dt_sim [veh/h]; above the
// 10 veh/h bottleneck capacity the queue builds and τ^CTM rises above τ^ff.
double ctm_route_time(double veh_per_dep_step, int n_dep = 30) {
  auto prob = make_corridor_problem(veh_per_dep_step, n_dep);

  EVPowerTrafficOptions ev;
  ev.num_steps = 120;          // long horizon so the queue forms and drains
  ev.time_step_hr = 0.05;

  CTMOptions cm;
  cm.dt_ctm_hr = 0.0;          // auto CFL
  cm.n_cells_per_link = 0;     // auto M_a (multiple cells → queue can stand)
  cm.route_specific_cells = true;

  DUEOptions du;
  du.max_iterations = 1;       // single forward pass = replay of the route
  du.convergence_tol = 1e-3;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev, cm, du);

  const auto& departures = res.final_ctm.route_admitted_departures.at(1);
  const auto& arrivals = res.final_ctm.route_terminal_arrivals.at(1);
  std::size_t departure = 0;
  std::size_t arrival = 0;
  double departure_remaining = departures.empty() ? 0.0 : departures.front();
  double arrival_remaining = arrivals.empty() ? 0.0 : arrivals.front();
  double matched = 0.0;
  double vehicle_hours = 0.0;
  while (departure < departures.size() && arrival < arrivals.size()) {
    if (departure_remaining <= 1e-10) {
      ++departure;
      if (departure < departures.size()) {
        departure_remaining = departures[departure];
      }
      continue;
    }
    if (arrival_remaining <= 1e-10) {
      ++arrival;
      if (arrival < arrivals.size()) arrival_remaining = arrivals[arrival];
      continue;
    }
    if (arrival < departure) {
      ++arrival;
      if (arrival < arrivals.size()) arrival_remaining = arrivals[arrival];
      continue;
    }
    const double flow = std::min(departure_remaining, arrival_remaining);
    vehicle_hours += flow * static_cast<double>(arrival - departure) *
                     res.final_ctm.dt_ctm_hr;
    matched += flow;
    departure_remaining -= flow;
    arrival_remaining -= flow;
  }
  return matched > 1e-9 ? vehicle_hours / matched : -1.0;
}

}  // namespace

// Per-step arrival rate that saturates the 20 veh/h link:
//   rate [veh/h] = veh_per_dep_step / dt_sim,  dt_sim = 0.05 h.
// So veh_per_dep_step = 0.05 -> 1 veh/h (sub-capacity, uncongested);
//    veh_per_dep_step = 1.0 -> 20 veh/h (4x bottleneck capacity).

// ──────────────────────────────────────────────────────────────────────────
// Prediction A: sub-capacity sustained inflow ⇒ τ^CTM ≈ τ^ff (Δτ ≈ 0).
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("Fidelity bound: sub-capacity inflow is free-flow-exact",
          "[fidelity][exactness]") {
  const double tau_ctm = ctm_route_time(0.05, 30);  // 1 veh/h < 5 cap
  REQUIRE(tau_ctm > 0.0);
  CHECK(tau_ctm <= kFreeFlowTau + kOracleToleranceHr);
}

// ──────────────────────────────────────────────────────────────────────────
// Prediction B: over-capacity sustained inflow ⇒ Δτ > 0, monotone in load.
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("Fidelity bound: over-capacity inflow induces positive, monotone Δτ",
          "[fidelity][congestion]") {
  const double calibrated_free_flow = ctm_route_time(0.05, 30);
  // Per-step vehicle shares -> arrival rates {1,5,10,15,20} veh/h.
  const std::vector<double> shares = {0.05, 0.25, 0.5, 0.75, 1.0};
  std::vector<double> deltas;
  for (double s : shares) {
    const double tau_ctm = ctm_route_time(s, 8);
    REQUIRE(tau_ctm > 0.0);
    deltas.push_back(tau_ctm - calibrated_free_flow);
  }
  for (double d : deltas) CHECK(d >= -kOracleToleranceHr);
  CHECK(deltas.back() > 1e-3);                       // heaviest load congests (S≠∅)
  for (std::size_t i = 1; i < deltas.size(); ++i)
    CHECK(deltas[i] >= deltas[i - 1] - 1e-6);        // monotone non-decreasing
}

// ──────────────────────────────────────────────────────────────────────────
// Printout for the paper: τ^ff, τ^CTM, Δτ across the loading sweep.
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("Fidelity bound: numerical sweep printout",
          "[fidelity_analysis]") {
  using std::printf;
  constexpr int n_dep_steps = 30;
  auto prob = make_corridor_problem(1.0, n_dep_steps);
  const double tau_ff = free_flow_route_time(prob);

  printf("\n==================================================================\n");
  printf(" TSN<->CTM fidelity oracle: single corridor, tau_ff = %.4f h\n", tau_ff);
  printf(" entry cap = 20 veh/h, bottleneck cap = 5 veh/h\n");
  printf("==================================================================\n");
  printf(" veh/step  rate(veh/h) tau_ff   tau_CTM   Delta_tau  S?  regime\n");
  printf(" --------  ----------- ------   -------   ---------  --  ----------\n");

  const double calibrated_free_flow = ctm_route_time(0.05, n_dep_steps);
  for (double n : {0.05, 0.25, 0.5, 0.75, 1.0}) {
    const double tau_ctm = ctm_route_time(n);
    const double delta = tau_ctm - calibrated_free_flow;
    const bool saturated = delta > 1e-4;
    printf("   %5.2f       %5.1f    %.4f   %.4f    %+.4f   %-3s %s\n",
           n, n / 0.05, tau_ff, tau_ctm, delta,
           saturated ? "yes" : "no",
           saturated ? "congested (correct)" : "free-flow (exact)");
  }
  printf("\n  Prediction A: small demand -> Delta_tau ~ 0  (free-flow exact)\n");
  printf("  Prediction B: large demand -> Delta_tau > 0, monotone (S != empty)\n");
  printf("==================================================================\n\n");
}
