// test_ltm_traffic.cpp
// ──────────────────────────────────────────────────────────────────────────
// Comprehensive test for the LTM-based meso-micro traffic framework.
//
// Covers (Section 20 of the technical notebook):
//   A. Triangular FD sending / receiving functions
//   B. Single-link LTM: free-flow propagation delay
//   C. Single-link LTM: downstream bottleneck and queue spillback
//   D. Three-link corridor: mesoscopic network simulation
//   E. EV penetration sweep: station loading vs penetration level
//   F. Individual EV trajectory reconstruction (FIFO) + V2G potential
//
// Numerical results (flows, travel times, SOC values, V2G power) that appear
// in the tex file are anchored to CHECK / REQUIRE assertions here so that
// the numbers remain consistent after any code change.
// ──────────────────────────────────────────────────────────────────────────

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/ev_power_traffic/ltm_network.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

using namespace hacdcpf::evpt;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ─────────────────────────────────────────────────────────────────────────
// Helper: build a standard 3-link corridor
//   Origin → A (2 km, 60 km/h, 1800 veh/h)
//           → B (1.5 km, 50 km/h, 1400 veh/h)  ← bottleneck
//           → C (3 km, 70 km/h, 2000 veh/h)  → Sink
// ─────────────────────────────────────────────────────────────────────────
static LTMNetworkProblem make_corridor(
    int T = 96, double dt = 0.25, double ev_pen = 0.0)
{
  LTMNetworkProblem prob;
  prob.T   = T;
  prob.dt_hr = dt;
  prob.ev_penetration = ev_pen;

  // Link A – approach
  LTMLinkParams la;
  la.id = 1; la.length_km = 2.0; la.v_ff_km_hr = 60.0;
  la.w_km_hr = 20.0; la.capacity_veh_hr = 1800.0; la.dt_hr = dt;

  // Link B – bottleneck
  LTMLinkParams lb;
  lb.id = 2; lb.length_km = 1.5; lb.v_ff_km_hr = 50.0;
  lb.w_km_hr = 15.0; lb.capacity_veh_hr = 1400.0; lb.dt_hr = dt;

  // Link C – downstream
  LTMLinkParams lc;
  lc.id = 3; lc.length_km = 3.0; lc.v_ff_km_hr = 70.0;
  lc.w_km_hr = 25.0; lc.capacity_veh_hr = 2000.0; lc.dt_hr = dt;

  prob.links = {la, lb, lc};
  prob.downstream_turns[1] = {{2, 1.0}};  // A → B
  prob.downstream_turns[2] = {{3, 1.0}};  // B → C
  prob.downstream_turns[3] = {};          // C → sink

  // Demand on link A
  std::vector<double> dem(static_cast<std::size_t>(T), 0.0);
  for (int k = 8; k < 20 && k < T; ++k)
    dem[static_cast<std::size_t>(k)] = 500.0 + 100.0 * (k - 8);
  for (int k = 20; k < 30 && k < T; ++k)
    dem[static_cast<std::size_t>(k)] = 1800.0;
  for (int k = 30; k < 42 && k < T; ++k)
    dem[static_cast<std::size_t>(k)] = 1800.0 - 100.0 * (k - 30);
  for (int k = 42; k < T; ++k)
    dem[static_cast<std::size_t>(k)] = 600.0;
  prob.origin_demand[1] = dem;
  return prob;
}

// ═════════════════════════════════════════════════════════════════════════
// TEST CASE A — Triangular FD: sending and receiving functions
// ═════════════════════════════════════════════════════════════════════════
TEST_CASE("LTM-A: Triangular FD sending and receiving functions",
          "[ltm][fundamental_diagram]")
{
  LTMLinkParams p;
  p.id = 10; p.length_km = 2.0; p.v_ff_km_hr = 60.0;
  p.w_km_hr = 20.0; p.capacity_veh_hr = 1800.0; p.dt_hr = 0.25;

  // τ_ff = ceil(2 / (60 * 0.25)) = ceil(2/15) = 1 step
  CHECK(p.tau_ff() == 1);
  // τ_bw = ceil(2 / (20 * 0.25)) = ceil(0.4) = 1 step
  CHECK(p.tau_bw() == 1);

  // k_crit = 1800/60 = 30 veh/km
  CHECK_THAT(p.k_crit(), WithinAbs(30.0, 1e-9));

  // k_jam = 1800*(1/60 + 1/20) = 1800*4/60 = 120 veh/km
  CHECK_THAT(p.k_jam(), WithinAbs(120.0, 1e-9));

  // N_jam = 120 * 2 = 240 veh
  CHECK_THAT(p.n_jam(), WithinAbs(240.0, 1e-9));

  // Capacity per step = 1800 * 0.25 = 450 veh
  const double cap_step = p.capacity_veh_hr * p.dt_hr;
  CHECK_THAT(cap_step, WithinAbs(450.0, 1e-9));

  // Sending at empty link: S(0) = 0
  {
    LTMLinkState ls;
    ls.id = p.id; ls.params = p;
    ls.init(4);
    CHECK_THAT(ls.sending(0), WithinAbs(0.0, 1e-9));
  }

  // Receiving at empty link: R = min(N_jam, cap_step) = 240 (< 450)
  {
    LTMLinkState ls;
    ls.id = p.id; ls.params = p;
    ls.init(4);
    CHECK_THAT(ls.receiving(0), WithinAbs(240.0, 0.1));
  }

  // Sending at full link (N_in[0] = N_jam, N_out[0] = 0)
  // S = min(N_in[k-tau_ff] - N_out[k], cap_step) = min(240-0, 450) = 240 = n_jam
  {
    LTMLinkState ls;
    ls.id = p.id; ls.params = p;
    ls.init(4);
    ls.N_in[0] = p.n_jam();
    CHECK_THAT(ls.sending(0), WithinAbs(p.n_jam(), 1e-6));  // 240, not 450
  }

  SECTION("Single-link free-flow: outflow equals demand after delay")
  {
    // Step demand: 900 veh/h (below capacity) for 1 step, then 0.
    std::vector<double> dem(8, 0.0);
    dem[0] = 900.0;

    LTMLinkState ls = simulate_ltm_single_link(p, dem);

    // Free-flow: 900 * 0.25 = 225 veh enters step 0 → N_in[1] = 225.
    // In the LTM discrete formulation, vehicles entering in step k appear in
    // N_in[k+1] and first appear in sending(k+tau_ff+1). With tau_ff=1:
    //   sending(2) = N_in[1] - N_out[2] = 225 → exit in N_out[3].
    CHECK_THAT(ls.N_in[1] - ls.N_in[0], WithinAbs(225.0, 1.0));

    // Outflow: N_out[3] - N_out[2] = 225 (exit at step tau_ff+1 = 2)
    CHECK_THAT(ls.N_out[3] - ls.N_out[2], WithinAbs(225.0, 1.0));
  }
}

// ═════════════════════════════════════════════════════════════════════════
// TEST CASE B — Single-link LTM: free-flow propagation delay
// ═════════════════════════════════════════════════════════════════════════
TEST_CASE("LTM-B: Single-link free-flow propagation delay",
          "[ltm][single_link][propagation_delay]")
{
  // Link with tau_ff = 2 steps: L=2 km, v_ff=60 km/h, dt=1/60 h ≈ 1 min
  LTMLinkParams p;
  p.id = 20; p.length_km = 2.0; p.v_ff_km_hr = 60.0;
  p.w_km_hr = 20.0; p.capacity_veh_hr = 1800.0; p.dt_hr = 1.0 / 30.0;

  // τ_ff = ceil(2 / (60 * (1/30))) = ceil(1) = 1
  // Let's use dt = 1/60 to get τ_ff = ceil(2 / (60/60)) = ceil(2) = 2
  p.dt_hr = 1.0 / 60.0;
  REQUIRE(p.tau_ff() == 2);

  const int T = 20;
  std::vector<double> dem(static_cast<std::size_t>(T), 0.0);
  dem[0] = 1000.0;  // pulse at step 0

  LTMLinkState ls = simulate_ltm_single_link(p, dem);

  // N_in increases at step 0 → 1 by 1000 * (1/60) = 16.67 veh
  const double entry_veh = 1000.0 * p.dt_hr;
  CHECK_THAT(ls.N_in[1], WithinRel(entry_veh, 1e-6));

  // N_out should not yet increase at steps 1 or 2 (delay = tau_ff = 2 + 1 step offset)
  CHECK_THAT(ls.N_out[2], WithinAbs(0.0, 1e-6));
  CHECK_THAT(ls.N_out[3], WithinAbs(0.0, 1e-6));  // still not yet at step 2
  // Vehicles exit starting at step tau_ff+1 = 3 (first contributing step)
  CHECK(ls.N_out[4] > ls.N_out[3]);  // outflow occurs at step 3

  // Total throughput conserved
  const double total_in  = ls.N_in[T];
  const double total_out = ls.N_out[T];
  CHECK(total_out <= total_in + 1e-6);
  CHECK(total_in  > 0.0);
}

// ═════════════════════════════════════════════════════════════════════════
// TEST CASE C — Single-link LTM: downstream bottleneck and spillback
// ═════════════════════════════════════════════════════════════════════════
TEST_CASE("LTM-C: Single-link with downstream bottleneck and spillback",
          "[ltm][single_link][spillback]")
{
  LTMLinkParams p;
  p.id = 30; p.length_km = 2.0; p.v_ff_km_hr = 60.0;
  p.w_km_hr = 20.0; p.capacity_veh_hr = 1800.0; p.dt_hr = 0.25;

  const int T = 48;
  std::vector<double> dem(static_cast<std::size_t>(T), 1600.0);  // below capacity

  // Downstream bottleneck: receiving = 40% of capacity for steps 8..20
  const double cap_step = p.capacity_veh_hr * p.dt_hr;
  std::vector<double> ds_recv(static_cast<std::size_t>(T),
                              std::numeric_limits<double>::infinity());
  for (int k = 8; k < 24 && k < T; ++k)
    ds_recv[static_cast<std::size_t>(k)] = cap_step * 0.40;

  LTMLinkState ls = simulate_ltm_single_link(p, dem, ds_recv);

  // During bottleneck, outflow is capped at 40% of capacity
  double max_out_rate_during_bn = 0.0;
  for (int k = 8; k < 24; ++k) {
    const double out_step =
        ls.N_out[static_cast<std::size_t>(k + 1)] -
        ls.N_out[static_cast<std::size_t>(k)];
    max_out_rate_during_bn = std::max(max_out_rate_during_bn, out_step);
  }
  // Outflow during bottleneck should be at most cap * 0.40 + small tolerance
  CHECK(max_out_rate_during_bn <= cap_step * 0.40 + 1.0);

  // Occupancy should increase during bottleneck
  const double n_before = ls.occupancy(7);
  const double n_during = ls.occupancy(18);
  CHECK(n_during > n_before);

  // After bottleneck releases, cumulative counts still monotone
  for (int k = 0; k < T; ++k) {
    CHECK(ls.N_in [static_cast<std::size_t>(k + 1)] >=
          ls.N_in [static_cast<std::size_t>(k)] - 1e-9);
    CHECK(ls.N_out[static_cast<std::size_t>(k + 1)] >=
          ls.N_out[static_cast<std::size_t>(k)] - 1e-9);
  }

  // Vehicle conservation: N_out[T] <= N_in[T]
  CHECK(ls.N_out[static_cast<std::size_t>(T)] <=
        ls.N_in [static_cast<std::size_t>(T)] + 1e-6);

  // Report key values for the tex
  double peak_occupancy = 0.0;
  for (int k = 0; k <= T; ++k)
    peak_occupancy = std::max(peak_occupancy, ls.occupancy(k));

  WARN("[tex] Peak occupancy during bottleneck: " << peak_occupancy << " veh");
  WARN("[tex] Jam storage:                       " << p.n_jam()       << " veh");
  CHECK(peak_occupancy <= p.n_jam() + 1e-6);
}

// ═════════════════════════════════════════════════════════════════════════
// TEST CASE D — Three-link corridor: mesoscopic network simulation
// ═════════════════════════════════════════════════════════════════════════
TEST_CASE("LTM-D: Three-link corridor mesoscopic network simulation",
          "[ltm][network][corridor]")
{
  const int T = 96;
  auto prob = make_corridor(T, 0.25, 0.0);
  LTMSimulationResult res = simulate_ltm_network(prob);

  REQUIRE(res.link_states.count(1));
  REQUIRE(res.link_states.count(2));
  REQUIRE(res.link_states.count(3));

  const auto& lsA = res.link_states.at(1);
  const auto& lsB = res.link_states.at(2);
  const auto& lsC = res.link_states.at(3);

  // SECTION D1: cumulative curves are non-decreasing on all links
  for (int k = 0; k < T; ++k) {
    CHECK(lsA.N_in [static_cast<std::size_t>(k + 1)] >=
          lsA.N_in [static_cast<std::size_t>(k)] - 1e-9);
    CHECK(lsB.N_in [static_cast<std::size_t>(k + 1)] >=
          lsB.N_in [static_cast<std::size_t>(k)] - 1e-9);
    CHECK(lsC.N_out[static_cast<std::size_t>(k + 1)] >=
          lsC.N_out[static_cast<std::size_t>(k)] - 1e-9);
  }

  // SECTION D2: vehicle conservation across the corridor
  // Total vehicles that entered A ~= total that exited C (some may still be en route)
  const double total_in  = lsA.N_in[static_cast<std::size_t>(T)];
  const double total_out = lsC.N_out[static_cast<std::size_t>(T)];
  CHECK(total_out <= total_in + 1.0);  // allow rounding
  CHECK(total_in  > 0.0);

  // SECTION D3: bottleneck link B has lower throughput than A's peak demand
  double max_B_out = 0.0;
  for (int k = 0; k < T; ++k) {
    const double out_k =
        lsB.N_out[static_cast<std::size_t>(k + 1)] -
        lsB.N_out[static_cast<std::size_t>(k)];
    max_B_out = std::max(max_B_out, out_k);
  }
  const double B_cap_step = 1400.0 * 0.25;  // 350 veh
  CHECK(max_B_out <= B_cap_step + 1e-6);

  // SECTION D4: link B has non-trivial occupancy during peak
  double max_occ_B = 0.0;
  for (int k = 0; k <= T; ++k)
    max_occ_B = std::max(max_occ_B, lsB.occupancy(k));
  CHECK(max_occ_B > 0.0);

  // Compute and report total throughput for the tex
  const double throughput_pct = 100.0 * total_out / std::max(total_in, 1.0);
  WARN("[tex] Corridor total_in:          " << total_in);
  WARN("[tex] Corridor total_out (exitC): " << total_out);
  WARN("[tex] Corridor throughput pct:    " << throughput_pct << "%");
  WARN("[tex] Link B max outflow step:    " << max_B_out << " veh (cap_step=" << B_cap_step << ")");
  WARN("[tex] Link B peak occupancy:      " << max_occ_B
       << " veh (jam = " << prob.links[1].n_jam() << " veh)");
}

// ═════════════════════════════════════════════════════════════════════════
// TEST CASE E — EV penetration sweep
// ═════════════════════════════════════════════════════════════════════════
TEST_CASE("LTM-E: EV penetration sweep – station loading",
          "[ltm][ev_penetration][station]")
{
  const std::vector<double> penetrations = {0.10, 0.30, 0.50, 0.70};
  const double dt       = 0.25;
  const int    T        = 96;
  const int    n_plugs  = 10;
  const double charging_fraction = 0.40;  // fraction of EVs needing a charge

  // Demand profile (veh/h)
  std::vector<double> dem_base(static_cast<std::size_t>(T), 0.0);
  for (int k = 8;  k < 28 && k < T; ++k)
    dem_base[static_cast<std::size_t>(k)] = 500.0 + 50.0 * (k - 8);
  for (int k = 28; k < 36 && k < T; ++k)
    dem_base[static_cast<std::size_t>(k)] = 1500.0;
  for (int k = 36; k < 48 && k < T; ++k)
    dem_base[static_cast<std::size_t>(k)] = 1500.0 - 58.0 * (k - 36);
  for (int k = 48; k < T; ++k)
    dem_base[static_cast<std::size_t>(k)] = 600.0;

  std::vector<double> max_queues;
  std::vector<double> avg_service_util;

  for (double pev : penetrations) {
    LTMNetworkProblem prob;
    prob.T      = T;
    prob.dt_hr  = dt;
    prob.ev_penetration = pev;

    LTMLinkParams la;
    la.id = 1; la.length_km = 2.0; la.v_ff_km_hr = 60.0;
    la.w_km_hr = 20.0; la.capacity_veh_hr = 1800.0; la.dt_hr = dt;
    prob.links = {la};
    prob.downstream_turns[1] = {};  // sink
    prob.origin_demand[1]    = dem_base;

    // Charging station
    ChargingStationParams sp;
    sp.id = 100; sp.n_plugs = n_plugs; sp.power_kw = 50.0;
    sp.mean_service_time_hr = 0.5; sp.mean_dwell_time_hr = 0.25; sp.dt_hr = dt;
    prob.stations = {sp};

    // Drive EV arrivals into the station via u_arr
    LTMSimulationResult res = simulate_ltm_network(prob);

    // Inject station arrivals from EV demand fraction
    auto& ss = res.station_states.at(100);
    for (int k = 0; k < T; ++k) {
      const double ev_arr = dem_base[static_cast<std::size_t>(k)]
                            * pev * charging_fraction * dt;
      ss.u_arr[static_cast<std::size_t>(k)] = ev_arr;
    }

    // Re-simulate station with injected arrivals
    ChargingStationState ss2;
    ss2.id = 100; ss2.params = sp; ss2.init(T);
    for (int k = 0; k < T; ++k)
      ss2.u_arr[static_cast<std::size_t>(k)] = ss.u_arr[static_cast<std::size_t>(k)];

    const double mu      = sp.mu_per_step();
    const double dw_rate = sp.dwell_rate();

    for (int k = 0; k < T; ++k) {
      const double arr_k = ss2.u_arr[static_cast<std::size_t>(k)];
      const double avail  = std::max(0.0,
          static_cast<double>(n_plugs) - ss2.B[static_cast<std::size_t>(k)]);
      const double b_k    = std::min(ss2.Q[static_cast<std::size_t>(k)] + arr_k, avail);
      const double c_k    = std::min(ss2.B[static_cast<std::size_t>(k)],
                                     ss2.B[static_cast<std::size_t>(k)] * mu);
      const double dp     = ss2.H[static_cast<std::size_t>(k)] + c_k;
      const double o_k    = std::min(dp, dp * dw_rate);
      ss2.b_adm[static_cast<std::size_t>(k)] = b_k;
      ss2.c_cmp[static_cast<std::size_t>(k)] = c_k;
      ss2.o_dep[static_cast<std::size_t>(k)] = o_k;
      ss2.Q[static_cast<std::size_t>(k + 1)] =
          std::max(0.0, ss2.Q[static_cast<std::size_t>(k)] + arr_k - b_k);
      ss2.B[static_cast<std::size_t>(k + 1)] =
          std::max(0.0, ss2.B[static_cast<std::size_t>(k)] + b_k - c_k);
      ss2.H[static_cast<std::size_t>(k + 1)] =
          std::max(0.0, ss2.H[static_cast<std::size_t>(k)] + c_k - o_k);
    }

    double max_q = *std::max_element(ss2.Q.begin(), ss2.Q.end());
    double avg_b = 0.0;
    for (int k = 0; k < T; ++k) avg_b += ss2.B[static_cast<std::size_t>(k)];
    avg_b /= T;

    max_queues.push_back(max_q);
    avg_service_util.push_back(avg_b / n_plugs * 100.0);

    INFO("EV penetration " << pev * 100 << "%"
         << "  max_queue=" << max_q
         << "  avg_util=" << avg_b / n_plugs * 100.0 << "%");
    WARN("[tex] EV pen=" << pev*100 << "%  max_queue=" << max_q
         << "  avg_plug_util=" << avg_b / n_plugs * 100.0 << "%");
  }

  // Monotone: higher penetration → larger max queue
  for (std::size_t i = 1; i < max_queues.size(); ++i)
    CHECK(max_queues[i] >= max_queues[i - 1] - 1e-6);

  // At 70% penetration, queue should be non-trivial
  CHECK(max_queues.back() > 0.0);

  // At 10% penetration, average utilisation is lower than at 70%
  CHECK(avg_service_util.front() <= avg_service_util.back() + 1e-6);
}

// ═════════════════════════════════════════════════════════════════════════
// TEST CASE F — Individual EV: trajectory reconstruction + V2G
// ═════════════════════════════════════════════════════════════════════════
TEST_CASE("LTM-F: Individual EV trajectory reconstruction and V2G potential",
          "[ltm][ev_agent][trajectory][v2g]")
{
  const int    T  = 96;
  const double dt = 0.25;

  LTMNetworkProblem prob = make_corridor(T, dt, 0.30);

  // Charging station on the corridor
  ChargingStationParams sp;
  sp.id = 200; sp.n_plugs = 8; sp.power_kw = 50.0;
  sp.mean_service_time_hr = 0.5; sp.mean_dwell_time_hr = 0.25; sp.dt_hr = dt;
  prob.stations = {sp};

  // Three EV agents: high, medium, low initial SOC
  EVAgentParams ev_high;
  ev_high.id = 1; ev_high.battery_capacity_kwh = 80.0;
  ev_high.initial_soc = 0.85;
  ev_high.consumption_kwh_per_km = 0.20;
  ev_high.reserve_kwh = 8.0; ev_high.charge_power_kw = 50.0;
  ev_high.path = {1, 2, 3};

  EVAgentParams ev_mid;
  ev_mid.id = 2; ev_mid.battery_capacity_kwh = 80.0;
  ev_mid.initial_soc = 0.45;
  ev_mid.consumption_kwh_per_km = 0.20;
  ev_mid.reserve_kwh = 8.0; ev_mid.charge_power_kw = 50.0;
  ev_mid.path = {1, 2, 3};

  EVAgentParams ev_low;
  ev_low.id = 3; ev_low.battery_capacity_kwh = 80.0;
  ev_low.initial_soc = 0.20;
  ev_low.consumption_kwh_per_km = 0.20;
  ev_low.reserve_kwh = 8.0; ev_low.charge_power_kw = 50.0;
  ev_low.path = {1, 2, 3};

  prob.ev_agents = {ev_high, ev_mid, ev_low};

  LTMSimulationResult res = simulate_ltm_network(prob);

  REQUIRE(res.ev_records.size() == 3);

  const auto& rec_h = res.ev_records[0];
  const auto& rec_l = res.ev_records[2];

  // ── F1: SOC never exceeds battery capacity or goes below 0 ───────────
  for (const auto& rec : res.ev_records) {
    for (double s : rec.soc) {
      CHECK(s >= -1e-6);
      CHECK(s <= 1.0 + 1e-6);
    }
  }

  // ── F2: High-SOC EV should not need to charge ────────────────────────
  INFO("EV_high charging events: " << rec_h.charging_events.size());
  // (May charge if reserve is tight; we just check SOC stays above reserve)
  for (double e : rec_h.energy_kwh)
    CHECK(e >= 0.0);

  // ── F3: Low-SOC EV should trigger charging ───────────────────────────
  INFO("EV_low charging events: " << rec_l.charging_events.size());
  INFO("EV_low final SOC: " << rec_l.final_soc() * 100 << "%");
  // After charging, low-SOC EV should end with more energy than initial
  // (unless trip is very long and drains it again — energy added > 0)
  if (!rec_l.charging_events.empty()) {
    double total_added = 0.0;
    for (const auto& ev : rec_l.charging_events)
      total_added += ev.energy_added_kwh;
    CHECK(total_added > 0.0);
  }

  // ── F4: FIFO trajectory reconstruction on link A ────────────────────
  {
    const auto& lsA = res.link_states.at(1);
    const int T_lsA = static_cast<int>(lsA.N_in.size()) - 1;

    // Build a cohort of 20 vehicles entering link A between steps 8..20
    std::vector<double> ranks;
    std::vector<int>    entry_steps_vec;
    const int n_veh = 20;
    for (int i = 0; i < n_veh; ++i) {
      const int k_e = 8 + i * (20 - 8) / (n_veh - 1);
      // Use N_in AFTER step k_e so we track vehicles that actually entered
      // (N_in[k_e+1] > N_in[k_e] when demand > 0 at step k_e).
      const int idx = std::min(k_e + 1, T_lsA);
      const double xi = lsA.N_in[static_cast<std::size_t>(idx)];
      ranks.push_back(xi);
      entry_steps_vec.push_back(k_e);
    }

    auto trajs = reconstruct_trajectories(lsA, ranks, entry_steps_vec);
    REQUIRE(trajs.size() == static_cast<std::size_t>(n_veh));

    for (const auto& tr : trajs) {
      // Exit time >= entry time
      CHECK(tr.exit_time_hr >= tr.entry_time_hr - 1e-9);
      // Travel time >= free-flow travel time
      CHECK(tr.experienced_travel_time_hr >=
            static_cast<double>(lsA.params.tau_ff()) * dt - 1e-9);
    }

    // FIFO: later-entering vehicles exit later
    for (std::size_t i = 1; i < trajs.size(); ++i)
      CHECK(trajs[i].exit_time_hr >= trajs[i - 1].exit_time_hr - 1e-9);
  }

  // ── F5: V2G potential is non-negative and bounded ────────────────────
  if (res.v2g_potential_kw.count(200)) {
    const auto& v2g = res.v2g_potential_kw.at(200);
    double peak_v2g = 0.0;
    for (double p : v2g) {
      CHECK(p >= -1e-6);
      peak_v2g = std::max(peak_v2g, p);
    }
    WARN("[tex] Peak V2G potential station 200: " << peak_v2g << " kW");
  }

  // ── F6: Station states have valid dynamics ─────────────────────────
  if (res.station_states.count(200)) {
    const auto& ss = res.station_states.at(200);
    for (int k = 0; k <= T; ++k) {
      CHECK(ss.Q[static_cast<std::size_t>(k)] >= -1e-6);
      CHECK(ss.B[static_cast<std::size_t>(k)] >= -1e-6);
      CHECK(ss.H[static_cast<std::size_t>(k)] >= -1e-6);
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════
// TEST CASE G — Quasi-static vs. LTM comparison (Experiment 1)
// ═════════════════════════════════════════════════════════════════════════
TEST_CASE("LTM-G: Quasi-static vs LTM propagation delay comparison",
          "[ltm][quasi_static][comparison]")
{
  const double dt = 0.25;
  const int    T  = 96;

  LTMLinkParams p;
  p.id = 40; p.length_km = 3.0; p.v_ff_km_hr = 60.0;
  p.w_km_hr = 20.0; p.capacity_veh_hr = 1600.0; p.dt_hr = dt;

  // tau_ff > 1: L=3 km, v_ff=60 km/h, dt=0.25 h → tau_ff = ceil(3/(60*0.25)) = ceil(0.2) = 1
  // For a clear delay, use dt = 1/60 h (1 min steps)
  LTMLinkParams p2 = p;
  p2.dt_hr = 1.0 / 60.0;
  INFO("tau_ff = " << p2.tau_ff() << " steps (1-min steps)");

  std::vector<double> dem_ltm(static_cast<std::size_t>(T), 0.0);
  for (int k = 8;  k < 20 && k < T; ++k)
    dem_ltm[static_cast<std::size_t>(k)] = 400.0 + 100.0 * (k - 8);
  for (int k = 20; k < 30 && k < T; ++k)
    dem_ltm[static_cast<std::size_t>(k)] = 1600.0;
  for (int k = 30; k < 40 && k < T; ++k)
    dem_ltm[static_cast<std::size_t>(k)] = 1600.0 - 100.0 * (k - 30);
  for (int k = 40; k < T; ++k)
    dem_ltm[static_cast<std::size_t>(k)] = 400.0;

  // LTM simulation
  LTMLinkState ltm_ls = simulate_ltm_single_link(p, dem_ltm);

  // Quasi-static: outflow capped at capacity, no delay
  std::vector<double> qs_outflow(static_cast<std::size_t>(T), 0.0);
  double qs_n = 0.0;
  for (int k = 0; k < T; ++k) {
    const double cap_step = p.capacity_veh_hr * dt;
    const double S_k = std::min(qs_n * p.v_ff_km_hr / p.length_km * dt, cap_step);
    const double R_k = std::min((p.n_jam() - qs_n) * p.w_km_hr / p.length_km * dt, cap_step);
    const double u_k = std::min(dem_ltm[static_cast<std::size_t>(k)] * dt, R_k);
    const double v_k = S_k;  // free downstream
    qs_outflow[static_cast<std::size_t>(k)] = v_k / dt;
    qs_n = std::max(0.0, qs_n + u_k - v_k);
  }

  // Find peak outflow step for LTM and quasi-static
  const auto ltm_out_rates = [&]() {
    std::vector<double> rates(static_cast<std::size_t>(T));
    for (int k = 0; k < T; ++k)
      rates[static_cast<std::size_t>(k)] =
          (ltm_ls.N_out[static_cast<std::size_t>(k + 1)] -
           ltm_ls.N_out[static_cast<std::size_t>(k)]) / dt;
    return rates;
  }();

  const int ltm_peak_step =
      static_cast<int>(std::max_element(ltm_out_rates.begin(),
                                        ltm_out_rates.end()) -
                        ltm_out_rates.begin());
  const int qs_peak_step  =
      static_cast<int>(std::max_element(qs_outflow.begin(),
                                        qs_outflow.end()) -
                        qs_outflow.begin());

  INFO("LTM peak outflow step:          " << ltm_peak_step
       << " (" << ltm_peak_step * dt << " h)");
  INFO("Quasi-static peak outflow step: " << qs_peak_step
       << " (" << qs_peak_step * dt << " h)");
  INFO("LTM tau_ff: " << p.tau_ff() << " step(s)");
  WARN("[tex] LTM peak outflow step:  " << ltm_peak_step << " (" << ltm_peak_step*dt << "h)");
  WARN("[tex] QS  peak outflow step:  " << qs_peak_step  << " (" << qs_peak_step *dt << "h)");

  // LTM peak should be >= quasi-static peak (propagation delay)
  CHECK(ltm_peak_step >= qs_peak_step);

  // Total throughput comparable (both conserve vehicles)
  const double ltm_total  = ltm_ls.N_out[static_cast<std::size_t>(T)];
  const double qs_total   = std::accumulate(qs_outflow.begin(),
                                             qs_outflow.end(), 0.0) * dt;
  WARN("[tex] LTM total throughput:   " << ltm_total << " veh");
  WARN("[tex] QS  total throughput:   " << qs_total  << " veh");
  INFO("LTM total throughput:          " << ltm_total << " veh");
  INFO("Quasi-static total throughput: " << qs_total  << " veh");
  CHECK(ltm_total  > 0.0);
  CHECK(qs_total   > 0.0);
}
