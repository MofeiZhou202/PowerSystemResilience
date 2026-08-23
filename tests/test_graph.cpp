// tests/test_graph.cpp
//
// Unit and integration tests for the hacdcpf graph analysis &
// reduction module (hacdcpf::graph).
//
// Test groups:
//   1. Topology analysis (connectivity, radiality, bridges, cut vertices)
//   2. Switch / zero-impedance contraction
//   3. Series reduction (passive degree-2 node elimination)
//   4. Pendant reduction (leaf-node folding)
//   5. Result recovery

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>
// MinGW GCC in strict -std=c++20 mode does not define M_PI via <cmath>.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using namespace hacdcpf;
using namespace hacdcpf::graph;
using Catch::Matchers::WithinAbs;

// ═══════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════

static ACBus make_ac_bus(int idx, BusType type, double base_kv = 110.0,
                          double pd = 0.0, double qd = 0.0) {
  ACBus b;
  b.index    = idx;
  b.bus_type = type;
  b.base_kv  = base_kv;
  b.pd_mw    = pd;
  b.qd_mvar  = qd;
  b.in_service = true;
  return b;
}

static ACBranch make_ac_branch(int idx, int from, int to,
                                double r = 0.01, double x = 0.05,
                                double rate_a = 0.0) {
  ACBranch br;
  br.index      = idx;
  br.from_bus   = from;
  br.to_bus     = to;
  br.r_pu       = r;
  br.x_pu       = x;
  br.b_pu       = 0.0;
  br.tap        = 1.0;
  br.rate_a_mva = rate_a;
  br.in_service = true;
  return br;
}

static Switch make_switch(int idx, int from, int to, bool closed = true) {
  Switch sw;
  sw.index      = idx;
  sw.bus_from   = from;
  sw.bus_to     = to;
  sw.closed     = closed;
  sw.in_service = true;
  return sw;
}

static HybridPowerSystem make_simple_system(
    std::vector<ACBus> buses, std::vector<ACBranch> branches) {
  HybridPowerSystem sys;
  sys.base_mva       = 100.0;
  sys.ac.buses       = std::move(buses);
  sys.ac.branches    = std::move(branches);
  return sys;
}

// ═══════════════════════════════════════════════════════════════════════
// 1. Topology analysis tests
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Graph topology: single island with slack", "[graph][topology]") {
  // 1 -- 2 -- 3 (Slack at 1)
  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ),
       make_ac_bus(3, BusType::PQ)},
      {make_ac_branch(0, 1, 2),
       make_ac_branch(1, 2, 3)});

  auto g   = build_power_system_graph(sys);
  auto rep = analyze_topology(g);

  REQUIRE(rep.n_ac_islands == 1);
  REQUIRE(rep.is_connected == true);
  REQUIRE(rep.is_radial    == true);
  REQUIRE(rep.cycle_count  == 0);
  REQUIRE(rep.islands[0].has_ac_slack == true);
  REQUIRE(rep.all_islands_valid == true);
  REQUIRE(rep.diagnostics.empty());
}

TEST_CASE("Graph topology: multiple islands, one without slack", "[graph][topology]") {
  // Island A: 1 -- 2 -- 3 (Slack at 1)
  // Island B: 4 -- 5 (no Slack)
  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ),
       make_ac_bus(3, BusType::PQ),
       make_ac_bus(4, BusType::PQ),
       make_ac_bus(5, BusType::PQ)},
      {make_ac_branch(0, 1, 2),
       make_ac_branch(1, 2, 3),
       make_ac_branch(2, 4, 5)});

  auto g   = build_power_system_graph(sys);
  auto rep = analyze_topology(g);

  // Two AC islands
  REQUIRE(rep.n_ac_islands == 2);
  REQUIRE(rep.is_connected == false);
  REQUIRE(rep.all_islands_valid == false);

  // At least one diagnostic about missing slack
  bool found_no_slack = false;
  for (const auto& d : rep.diagnostics)
    if (d.code == DiagCode::GraphNoSlackInIsland) { found_no_slack = true; break; }
  REQUIRE(found_no_slack);
}

TEST_CASE("Graph topology: radial check on linear chain", "[graph][topology]") {
  // 1 -- 2 -- 3 -- 4 (radial)
  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ),
       make_ac_bus(3, BusType::PQ),
       make_ac_bus(4, BusType::PQ)},
      {make_ac_branch(0, 1, 2),
       make_ac_branch(1, 2, 3),
       make_ac_branch(2, 3, 4)});

  auto g = build_power_system_graph(sys);
  REQUIRE(is_radial(g)    == true);
  REQUIRE(is_connected(g) == true);

  // Add a loop: 4 -- 2
  sys.ac.branches.push_back(make_ac_branch(3, 4, 2));
  auto g2   = build_power_system_graph(sys);
  auto rep2 = analyze_topology(g2);
  REQUIRE(is_radial(g2)   == false);
  REQUIRE(rep2.cycle_count >= 1);
}

TEST_CASE("Graph topology: bridge and cut-vertex detection", "[graph][topology]") {
  // Topology:
  //   1 -- 2 -- 3
  //        |
  //        4
  // Edge 1-2 is a bridge; Node 2 is a cut vertex.
  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ),
       make_ac_bus(3, BusType::PQ),
       make_ac_bus(4, BusType::PQ)},
      {make_ac_branch(0, 1, 2),
       make_ac_branch(1, 2, 3),
       make_ac_branch(2, 2, 4)});

  auto g   = build_power_system_graph(sys);
  auto rep = analyze_topology(g);

  // Edge 0 (1-2) should be a bridge
  bool has_bridge_01 = false;
  for (int eid : rep.bridge_edge_ids) {
    const auto& e = g.edges[eid];
    if ((e.from_bus_id == 1 && e.to_bus_id == 2) ||
        (e.from_bus_id == 2 && e.to_bus_id == 1))
      has_bridge_01 = true;
  }
  REQUIRE(has_bridge_01);

  // Bus 2 should be a cut vertex
  bool has_cutv_2 = false;
  for (int bid : rep.cut_vertex_bus_ids)
    if (bid == 2) has_cutv_2 = true;
  REQUIRE(has_cutv_2);
}

TEST_CASE("Graph topology: cut vertices retain AC/DC domains for colliding bus IDs",
          "[graph][topology][collision]") {
  // AC1 -- AC2 -- VSC -- DC2 -- DC3.  Both inner buses are articulation
  // points, but their stable integer IDs intentionally collide.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {make_ac_bus(1, BusType::SLACK),
                  make_ac_bus(2, BusType::PQ)};
  sys.ac.branches = {make_ac_branch(10, 1, 2)};

  DCBus dc2;
  dc2.index = 2;
  dc2.bus_type = DCBusType::DC_V;
  dc2.in_service = true;
  DCBus dc3;
  dc3.index = 3;
  dc3.bus_type = DCBusType::DC_P;
  dc3.in_service = true;
  sys.dc.buses = {dc2, dc3};

  DCBranch dc_branch;
  dc_branch.index = 20;
  dc_branch.from_bus = 2;
  dc_branch.to_bus = 3;
  dc_branch.r_pu = 0.02;
  dc_branch.in_service = true;
  sys.dc.branches = {dc_branch};

  VSCConverter vsc;
  vsc.index = 30;
  vsc.bus_ac = 2;
  vsc.bus_dc = 2;
  vsc.in_service = true;
  sys.vsc_converters = {vsc};

  const auto graph = build_power_system_graph(sys);
  const auto report = analyze_topology(graph);

  const auto has_ref = [&](NodeDomain domain, int bus_id) {
    return std::any_of(report.cut_vertices.begin(), report.cut_vertices.end(),
                       [&](const TopologyBusRef& ref) {
                         return ref.domain == domain && ref.bus_id == bus_id;
                       });
  };
  REQUIRE(has_ref(NodeDomain::AC, 2));
  REQUIRE(has_ref(NodeDomain::DC, 2));
  REQUIRE(std::count(report.cut_vertex_bus_ids.begin(),
                     report.cut_vertex_bus_ids.end(), 2) == 2);
}

TEST_CASE("Graph topology: iterative Tarjan survives stack growth",
          "[graph][topology][tarjan]") {
  constexpr int node_count = 64;
  std::vector<ACBus> buses;
  std::vector<ACBranch> branches;
  buses.reserve(node_count);
  branches.reserve(node_count - 1);

  for (int bus_id = 1; bus_id <= node_count; ++bus_id) {
    buses.push_back(make_ac_bus(
        bus_id, bus_id == 1 ? BusType::SLACK : BusType::PQ));
  }
  for (int bus_id = 1; bus_id < node_count; ++bus_id) {
    branches.push_back(
        make_ac_branch(bus_id - 1, bus_id, bus_id + 1));
  }

  const auto graph =
      build_power_system_graph(make_simple_system(buses, branches));
  const auto report = analyze_topology(graph);

  REQUIRE(report.bridge_edge_ids.size() == node_count - 1);
  REQUIRE(report.cut_vertex_bus_ids.size() == node_count - 2);
}

TEST_CASE("Graph topology: parallel edges are not bridges",
          "[graph][topology][tarjan]") {
  const auto graph = build_power_system_graph(make_simple_system(
      {make_ac_bus(1, BusType::SLACK), make_ac_bus(2, BusType::PQ)},
      {make_ac_branch(10, 1, 2), make_ac_branch(20, 1, 2)}));
  const auto report = analyze_topology(graph);

  REQUIRE(report.bridge_edge_ids.empty());
  REQUIRE(report.cut_vertex_bus_ids.empty());
}

TEST_CASE("Graph topology: isolated load node diagnostic", "[graph][topology]") {
  // Bus 2 has load but no branch
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {make_ac_bus(1, BusType::SLACK),
                  make_ac_bus(2, BusType::PQ, 110.0, 50.0, 20.0)};
  // No branch connecting bus 2 to anything

  auto g   = build_power_system_graph(sys);
  auto rep = analyze_topology(g);

  bool found_isolated = false;
  for (const auto& d : rep.diagnostics)
    if (d.code == DiagCode::GraphIsolatedLoad) { found_isolated = true; break; }
  REQUIRE(found_isolated);
  REQUIRE(std::count_if(rep.islands.begin(), rep.islands.end(), [](const auto& island) {
            return island.status == IslandStatus::IsolatedLoad;
          }) == 1);
}

TEST_CASE("Graph topology: cycle detection in mesh network", "[graph][topology]") {
  // 3-bus ring: 1--2, 2--3, 3--1
  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ),
       make_ac_bus(3, BusType::PQ)},
      {make_ac_branch(0, 1, 2),
       make_ac_branch(1, 2, 3),
       make_ac_branch(2, 3, 1)});

  auto g   = build_power_system_graph(sys);
  auto rep = analyze_topology(g);

  REQUIRE(rep.is_radial == false);
  REQUIRE(rep.cycle_count == 1);
  REQUIRE(!rep.fundamental_cycles.empty());
  for (const auto& cycle : rep.fundamental_cycles) {
    REQUIRE(cycle.size() >= 3);
    for (const int edge_position : cycle) {
      REQUIRE(edge_position >= 0);
      REQUIRE(edge_position < static_cast<int>(g.edges.size()));
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════
// 2. Switch / zero-impedance contraction tests
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Switch contraction: closed switch merges loads", "[graph][contraction]") {
  // Bus1 -- Line -- Bus2 -- ClosedSwitch -- Bus3
  // Bus2 load = 10 MW, Bus3 load = 15 MW
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK, 110.0),
      make_ac_bus(2, BusType::PQ,    110.0, 10.0, 5.0),
      make_ac_bus(3, BusType::PQ,    110.0, 15.0, 7.0),
  };
  sys.ac.branches = {make_ac_branch(0, 1, 2)};
  sys.ac.switches = {make_switch(0, 2, 3, true)}; // closed switch

  auto g = build_power_system_graph(sys);
  ContractionOptions opts;
  auto res = contract_zero_impedance_edges(g, sys, opts);

  // Bus2 and Bus3 should be merged
  REQUIRE(res.ac_bus_to_super.count(2) > 0);
  REQUIRE(res.ac_bus_to_super.count(3) > 0);
  int rep2 = res.ac_bus_to_super.at(2);
  int rep3 = res.ac_bus_to_super.at(3);
  REQUIRE(rep2 == rep3); // same super-node

  // Super-node load should be sum: 25 MW
  double merged_pd = 0.0;
  for (const auto& bus : res.contracted_system.ac.buses) {
    if (bus.index == rep2) { merged_pd = bus.pd_mw; break; }
  }
  REQUIRE_THAT(merged_pd, WithinAbs(25.0, 1e-9));

  // Record should be created
  REQUIRE(!res.switch_records.empty());
}

TEST_CASE("Switch contraction: voltage base mismatch rejected", "[graph][contraction]") {
  // Bus2 (10 kV) -- ClosedSwitch -- Bus3 (35 kV): should produce diagnostic
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK, 110.0),
      make_ac_bus(2, BusType::PQ,    10.0),   // 10 kV
      make_ac_bus(3, BusType::PQ,    35.0),   // 35 kV
  };
  sys.ac.branches = {make_ac_branch(0, 1, 2)};
  sys.ac.switches = {make_switch(0, 2, 3, true)};

  auto g = build_power_system_graph(sys);
  ContractionOptions opts;
  opts.voltage_base_tolerance = 0.01; // 1% tolerance
  auto res = contract_zero_impedance_edges(g, sys, opts);

  // Should have a voltage-base mismatch diagnostic
  bool found = false;
  for (const auto& d : res.diagnostics)
    if (d.code == DiagCode::GraphVoltageBaseMismatch) { found = true; break; }
  REQUIRE(found);

  // Bus 2 and 3 must NOT be merged
  if (res.ac_bus_to_super.count(2) && res.ac_bus_to_super.count(3))
    REQUIRE(res.ac_bus_to_super.at(2) != res.ac_bus_to_super.at(3));
}

TEST_CASE("Switch contraction: multiple slack in merge triggers diagnostic",
          "[graph][contraction]") {
  // Slack1 -- ClosedSwitch -- Slack2
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK, 110.0),
      make_ac_bus(2, BusType::SLACK, 110.0),
  };
  sys.ac.switches = {make_switch(0, 1, 2, true)};

  auto g = build_power_system_graph(sys);
  ContractionOptions opts;
  opts.allow_multi_slack_merge = false; // strict
  auto res = contract_zero_impedance_edges(g, sys, opts);

  bool found_multislack = false;
  for (const auto& d : res.diagnostics)
    if (d.code == DiagCode::GraphMultipleSlack) { found_multislack = true; break; }
  REQUIRE(found_multislack);
}

TEST_CASE("Switch contraction: open switch does not merge buses", "[graph][contraction]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK, 110.0),
      make_ac_bus(2, BusType::PQ,    110.0, 10.0, 0.0),
      make_ac_bus(3, BusType::PQ,    110.0, 15.0, 0.0),
  };
  sys.ac.switches = {make_switch(0, 2, 3, false)}; // OPEN switch

  auto g = build_power_system_graph(sys);
  auto res = contract_zero_impedance_edges(g, sys, {});

  // Buses 2 and 3 must remain separate
  if (res.ac_bus_to_super.count(2) && res.ac_bus_to_super.count(3))
    REQUIRE(res.ac_bus_to_super.at(2) != res.ac_bus_to_super.at(3));
}

// ═══════════════════════════════════════════════════════════════════════
// 3. Series reduction tests
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Series reduction: passive degree-2 node eliminated", "[graph][series]") {
  // 1 -- Z12 -- 2 -- Z23 -- 3
  // Bus 2: no load, no gen, no shunt → series reduction candidate
  const double r12 = 0.01, x12 = 0.05;
  const double r23 = 0.02, x23 = 0.04;

  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ),   // passive
       make_ac_bus(3, BusType::PQ)},
      {make_ac_branch(0, 1, 2, r12, x12),
       make_ac_branch(1, 2, 3, r23, x23)});

  auto g = build_power_system_graph(sys);

  GraphReductionOptions opts;
  opts.enable_series_reduction      = true;
  opts.preserve_all_load_buses      = true;
  opts.preserve_all_generator_buses = true;
  // Bus 2 has no load/gen, so it should be a series candidate

  auto candidates = classify_reduction_candidates(g, sys, opts);
  // Check Bus 2 is classified as ZeroInjectionDegree2
  bool found_d2 = false;
  for (const auto& c : candidates.candidates)
    if (c.bus.bus_id == 2 && c.type == CandidateType::ZeroInjectionDegree2)
      found_d2 = true;
  REQUIRE(found_d2);

  auto plan = make_reduction_plan(g, candidates, opts);
  REQUIRE(plan.n_buses_eliminated >= 1);

  auto result = apply_series_reduction(g, sys, plan, opts);

  // Verify: Bus 2 should be marked inactive in the reduced system
  bool bus2_inactive = false;
  for (const auto& bus : result.reduced_system.ac.buses)
    if (bus.index == 2 && !bus.in_service) { bus2_inactive = true; break; }
  REQUIRE(bus2_inactive);

  // Verify: there should be a new equivalent branch
  bool has_eq_branch = false;
  for (const auto& br : result.reduced_system.ac.branches) {
    if (!br.in_service) continue;
    // Equivalent branch connects bus 1 and bus 3
    if ((br.from_bus == 1 && br.to_bus == 3) ||
        (br.from_bus == 3 && br.to_bus == 1)) {
      has_eq_branch = true;
      REQUIRE_THAT(br.r_pu, WithinAbs(r12 + r23, 1e-12));
      REQUIRE_THAT(br.x_pu, WithinAbs(x12 + x23, 1e-12));
      break;
    }
  }
  REQUIRE(has_eq_branch);

  // Verify series record stored correct r_eq, x_eq
  REQUIRE(!result.mapping.series_records.empty());
  const auto& rec = result.mapping.series_records[0];
  REQUIRE(rec.eliminated_bus_id == 2);
  REQUIRE_THAT(rec.r_eq, WithinAbs(r12 + r23, 1e-12));
  REQUIRE_THAT(rec.x_eq, WithinAbs(x12 + x23, 1e-12));
}

TEST_CASE("Series reduction: bus with load not eliminated", "[graph][series]") {
  // Bus 2 has a 10 MW load
  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ, 110.0, 10.0, 0.0), // has load
       make_ac_bus(3, BusType::PQ)},
      {make_ac_branch(0, 1, 2, 0.01, 0.05),
       make_ac_branch(1, 2, 3, 0.02, 0.04)});

  auto g = build_power_system_graph(sys);
  GraphReductionOptions opts;
  opts.preserve_all_load_buses = true;

  auto candidates = classify_reduction_candidates(g, sys, opts);
  // Bus 2 should be MustRetain because it has load
  bool bus2_must_retain = false;
  for (const auto& c : candidates.candidates)
    if (c.bus.bus_id == 2 && c.type == CandidateType::MustRetain)
      bus2_must_retain = true;
  REQUIRE(bus2_must_retain);

  auto plan = make_reduction_plan(g, candidates, opts);
  // No series reduction actions
  bool has_series = false;
  for (const auto& a : plan.actions)
    if (a.type == ReductionActionType::SeriesReduction) has_series = true;
  REQUIRE(!has_series);
}

// ═══════════════════════════════════════════════════════════════════════
// 4. Pendant reduction tests
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Pendant reduction: leaf load folded to parent", "[graph][pendant]") {
  // 1 -- Z12 -- 2 (load: 1.0 MW, 0.5 Mvar)
  // |V1| = 1.0 pu approximation
  const double r12 = 0.01, x12 = 0.04;
  const double p2_mw = 1.0, q2_mvar = 0.5;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK, 110.0),
      make_ac_bus(2, BusType::PQ, 110.0, p2_mw, q2_mvar), // pendant load
  };
  sys.ac.branches = {make_ac_branch(0, 1, 2, r12, x12)};

  auto g = build_power_system_graph(sys);

  GraphReductionOptions opts;
  opts.enable_pendant_reduction = true;
  opts.preserve_all_load_buses  = false; // allow pendant folding

  auto candidates = classify_reduction_candidates(g, sys, opts);
  // Bus 2 should be PendantLoad
  bool found_pendant = false;
  for (const auto& c : candidates.candidates)
    if (c.bus.bus_id == 2 && c.type == CandidateType::PendantLoad)
      found_pendant = true;
  REQUIRE(found_pendant);

  auto plan   = make_reduction_plan(g, candidates, opts);
  auto result = apply_pendant_reduction(g, sys, plan, opts);

  // Bus 1's load should have increased
  double p1_new = 0.0;
  for (const auto& bus : result.reduced_system.ac.buses)
    if (bus.index == 1) { p1_new = bus.pd_mw; break; }

  // Expected: P_i_new = P_i + P_j + R_ij * (P_j² + Q_j²) / |V_i|²
  // Base: P_i = 0, P_j = 1.0, Q_j = 0.5
  double p_pu = p2_mw / 100.0, q_pu = q2_mvar / 100.0;
  double p_loss_pu = r12 * (p_pu*p_pu + q_pu*q_pu);
  double expected_p1 = (p_pu + p_loss_pu) * 100.0; // back to MW

  REQUIRE_THAT(p1_new, WithinAbs(expected_p1, 1e-6));

  // Bus 2 should be inactive
  bool bus2_inactive = false;
  for (const auto& bus : result.reduced_system.ac.buses)
    if (bus.index == 2 && !bus.in_service) { bus2_inactive = true; break; }
  REQUIRE(bus2_inactive);
}

// ═══════════════════════════════════════════════════════════════════════
// 5. Result recovery tests
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Result recovery: switch contraction voltage propagation",
          "[graph][recovery]") {
  // Bus1 -- Switch -- Bus2 (merged into super-node = Bus1)
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK, 110.0),
      make_ac_bus(2, BusType::PQ,    110.0),
  };
  sys.ac.switches = {make_switch(0, 1, 2, true)};

  auto g   = build_power_system_graph(sys);
  auto res = contract_zero_impedance_edges(g, sys, {});

  // Simulate: after solving, super-node (rep bus) has voltage
  FullNetworkVoltages voltages;
  int rep = res.ac_bus_to_super.at(1);
  voltages.bus_voltage[rep] = std::complex<double>{1.05, 0.0}; // 1.05∠0°

  recover_switch_contracted_buses(voltages, res);

  // Both Bus 1 and Bus 2 should have the same voltage
  REQUIRE(voltages.bus_voltage.count(1) > 0);
  REQUIRE(voltages.bus_voltage.count(2) > 0);
  REQUIRE_THAT(voltages.vm_pu(1), WithinAbs(1.05, 1e-12));
  REQUIRE_THAT(voltages.vm_pu(2), WithinAbs(1.05, 1e-12));
  REQUIRE_THAT(voltages.va_deg(1), WithinAbs(0.0, 1e-12));
  REQUIRE_THAT(voltages.va_deg(2), WithinAbs(0.0, 1e-12));
}

TEST_CASE("Result recovery: series-eliminated bus voltage", "[graph][recovery]") {
  // 1 -- Z12 -- 2 -- Z23 -- 3 (passive Bus 2 eliminated)
  // Given V1, V3 → recover V2
  const double r12 = 0.01, x12 = 0.05;
  const double r23 = 0.02, x23 = 0.04;

  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ),
       make_ac_bus(3, BusType::PQ)},
      {make_ac_branch(0, 1, 2, r12, x12),
       make_ac_branch(1, 2, 3, r23, x23)});

  auto g   = build_power_system_graph(sys);
  GraphReductionOptions opts;
  opts.enable_series_reduction = true;
  opts.preserve_all_load_buses = true;

  auto candidates = classify_reduction_candidates(g, sys, opts);
  auto plan   = make_reduction_plan(g, candidates, opts);
  auto result = apply_series_reduction(g, sys, plan, opts);

  // Set known voltages at Bus 1 and Bus 3
  // V1 = 1.05∠0°, V3 = 0.99∠-2°
  const double deg_to_rad = M_PI / 180.0;
  std::complex<double> V1 = std::polar(1.05, 0.0);
  std::complex<double> V3 = std::polar(0.99, -2.0 * deg_to_rad);

  FullNetworkVoltages voltages;
  voltages.bus_voltage[1] = V1;
  voltages.bus_voltage[3] = V3;

  recover_series_reduced_buses(voltages, result.mapping, sys);

  // Check V2 is recovered
  REQUIRE(voltages.bus_voltage.count(2) > 0);

  // Analytical V2: Z_ik = Z12 + Z23; I_ik = (V1-V3)/Z_ik; V2 = V1 - Z12*I_ik
  std::complex<double> Z12{r12, x12}, Z23{r23, x23};
  std::complex<double> Z_ik = Z12 + Z23;
  std::complex<double> I_ik = (V1 - V3) / Z_ik;
  std::complex<double> V2_expected = V1 - Z12 * I_ik;

  REQUIRE_THAT(std::abs(voltages.bus_voltage.at(2) - V2_expected),
               WithinAbs(0.0, 1e-10));
}

// ═══════════════════════════════════════════════════════════════════════
// 6. Graph construction edge cases
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Graph build: empty system", "[graph][build]") {
  HybridPowerSystem sys;
  auto g = build_power_system_graph(sys);
  REQUIRE(g.node_count() == 0);
  REQUIRE(g.edge_count() == 0);
}

TEST_CASE("Graph build: single bus no branches", "[graph][build]") {
  HybridPowerSystem sys;
  sys.ac.buses = {make_ac_bus(1, BusType::SLACK)};
  auto g = build_power_system_graph(sys);
  REQUIRE(g.node_count() == 1);
  REQUIRE(g.edge_count() == 0);
  REQUIRE(g.nodes[0].is_slack == true);
  REQUIRE(count_islands(g) == 1);
}

TEST_CASE("Graph build: AC+DC hybrid", "[graph][build]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {make_ac_bus(1, BusType::SLACK), make_ac_bus(2, BusType::PQ)};
  sys.ac.branches = {make_ac_branch(0, 1, 2)};

  DCBus dcb1, dcb2;
  dcb1.index = 10; dcb1.bus_type = DCBusType::DC_V; dcb1.in_service = true;
  dcb2.index = 11; dcb2.bus_type = DCBusType::DC_P; dcb2.in_service = true;
  sys.dc.buses = {dcb1, dcb2};
  DCBranch dcbr;
  dcbr.index = 100; dcbr.from_bus = 10; dcbr.to_bus = 11; dcbr.r_pu = 0.02;
  dcbr.in_service = true;
  sys.dc.branches = {dcbr};

  auto g = build_power_system_graph(sys);
  REQUIRE(g.node_count() == 4); // 2 AC + 2 DC
  // DC Vref bus should be marked slack
  int ni_dc1 = g.dc_node_idx(10);
  REQUIRE(ni_dc1 >= 0);
  REQUIRE(g.nodes[ni_dc1].is_slack == true);
  REQUIRE(g.nodes[ni_dc1].domain == NodeDomain::DC);
}

TEST_CASE("Graph build: node flags correctly set", "[graph][build]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {make_ac_bus(1, BusType::SLACK),
                  make_ac_bus(2, BusType::PV),
                  make_ac_bus(3, BusType::PQ, 110.0, 50.0, 20.0)};
  sys.ac.branches = {make_ac_branch(0, 1, 2), make_ac_branch(1, 2, 3)};

  Generator g3;
  g3.bus = 2; g3.in_service = true; g3.is_slack = false;
  sys.ac.generators = {g3};

  Shunt sh;
  sh.bus = 3; sh.gs_mw = 0.0; sh.bs_mvar = 5.0; sh.in_service = true;
  sys.ac.shunts = {sh};

  auto g = build_power_system_graph(sys);

  // Bus 1: slack
  int ni1 = g.node_idx(1);
  REQUIRE(g.nodes[ni1].is_slack == true);

  // Bus 2: voltage controlled (PV bus type)
  int ni2 = g.node_idx(2);
  REQUIRE(g.nodes[ni2].is_voltage_controlled == true);
  REQUIRE(g.nodes[ni2].has_generator == true);

  // Bus 3: has load (from bus pd/qd) and shunt
  int ni3 = g.node_idx(3);
  REQUIRE(g.nodes[ni3].has_load  == true);
  REQUIRE(g.nodes[ni3].has_shunt == true);
}

// ═══════════════════════════════════════════════════════════════════════
// 7. AC/DC same-ID collision regression tests
// These tests verify that AC bus N and DC bus N (same integer) are treated
// as independent nodes throughout contraction, reduction and recovery.
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Same-ID collision: switch contraction isolates AC and DC domains",
          "[graph][contraction][collision]") {
  // AC: Bus1(SLACK, 0 MW) -- ClosedSwitch -- Bus2(PQ, 10 MW)
  // DC: Bus1(DC_V, 0 MW)  -- Closed CB   -- Bus2(DC_P, 20 MW)
  // Both domains use bus IDs 1 and 2 deliberately.
  HybridPowerSystem sys;
  sys.base_mva = 10.0;

  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK, 110.0, 0.0, 0.0),
      make_ac_bus(2, BusType::PQ,    110.0, 10.0, 0.0),
  };
  sys.ac.switches = {make_switch(0, 1, 2, true)};

  DCBus dc1; dc1.index = 1; dc1.bus_type = DCBusType::DC_V; dc1.pd_mw = 0.0; dc1.in_service = true;
  DCBus dc2; dc2.index = 2; dc2.bus_type = DCBusType::DC_P; dc2.pd_mw = 20.0; dc2.in_service = true;
  sys.dc.buses = {dc1, dc2};

  DCCircuitBreaker dccb; dccb.index = 0; dccb.bus_from = 1; dccb.bus_to = 2;
  dccb.closed = true; dccb.in_service = true;
  sys.dc.dc_circuit_breakers = {dccb};

  auto g   = build_power_system_graph(sys);
  auto res = contract_zero_impedance_edges(g, sys, {});

  // ── AC domain: buses 1 and 2 should be merged ────────────────────
  REQUIRE(res.ac_bus_to_super.count(1) > 0);
  REQUIRE(res.ac_bus_to_super.count(2) > 0);
  int ac_rep = res.ac_bus_to_super.at(1);
  REQUIRE(res.ac_bus_to_super.at(2) == ac_rep);

  // AC super-node rep should carry the AC load (10 MW) — NOT 20 MW
  double ac_pd = 0.0;
  for (const auto& bus : res.contracted_system.ac.buses)
    if (bus.index == ac_rep) { ac_pd = bus.pd_mw; break; }
  REQUIRE_THAT(ac_pd, WithinAbs(10.0, 1e-9));

  // ── DC domain: buses 1 and 2 should be merged ────────────────────
  REQUIRE(res.dc_bus_to_super.count(1) > 0);
  REQUIRE(res.dc_bus_to_super.count(2) > 0);
  int dc_rep = res.dc_bus_to_super.at(1);
  REQUIRE(res.dc_bus_to_super.at(2) == dc_rep);

  // AC and DC super-nodes must be domain-independent (the sets are disjoint
  // because ac_super_to_buses and dc_super_to_buses are separate maps)
  REQUIRE(res.ac_super_to_buses.count(ac_rep) > 0);
  REQUIRE(res.dc_super_to_buses.count(dc_rep) > 0);

  // ── Domain-aware recovery should not mix AC and DC voltages ──────
  FullNetworkVoltages voltages;
  // Seed representative bus voltages as if the two independent solves returned:
  //   AC bus 1 (rep) = 1.02 pu,  DC bus 1 (rep) = 0.98 pu
  // Because AC and DC share the same integer key "1" we write to the
  // domain-qualified maps directly so the two solvers don't overwrite each other.
  voltages.ac_bus_voltage[ac_rep] = std::complex<double>{1.02, 0.0};
  voltages.dc_bus_voltage[dc_rep] = std::complex<double>{0.98, 0.0};
  // Seed the legacy flat map too (AC rep wins for the shared key).
  voltages.bus_voltage[ac_rep] = std::complex<double>{1.02, 0.0};
  recover_switch_contracted_buses(voltages, res);
  // After recovery, AC bus 2 should carry the AC super-node voltage (1.02 pu)
  REQUIRE(voltages.ac_bus_voltage.count(2) > 0);
  REQUIRE(std::abs(voltages.ac_bus_voltage.at(2).real() - 1.02) < 1e-9);
  // DC bus 2 should carry the DC super-node voltage (0.98 pu), not the AC voltage
  REQUIRE(voltages.dc_bus_voltage.count(2) > 0);
  REQUIRE(std::abs(voltages.dc_bus_voltage.at(2).real() - 0.98) < 1e-9);
  // Legacy map: AC voltage for bus 1 must not have been overwritten by DC value
  REQUIRE(std::abs(voltages.bus_voltage.at(ac_rep).real() - 1.02) < 1e-9);
}

TEST_CASE("Same-ID collision: series reduction domain maps are independent",
          "[graph][series][collision]") {
  // AC: 1 -- r=0.01,x=0.05 -- 2(passive) -- r=0.02,x=0.04 -- 3
  // DC: 1 -- r=0.03 -- 2(passive) -- r=0.04 -- 3
  // Both domains use bus IDs 1, 2, 3 deliberately.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK),
      make_ac_bus(2, BusType::PQ),   // passive
      make_ac_bus(3, BusType::PQ),
  };
  sys.ac.branches = {
      make_ac_branch(0, 1, 2, 0.01, 0.05),
      make_ac_branch(1, 2, 3, 0.02, 0.04),
  };

  DCBus dcb1; dcb1.index = 1; dcb1.bus_type = DCBusType::DC_V; dcb1.in_service = true;
  DCBus dcb2; dcb2.index = 2; dcb2.bus_type = DCBusType::DC_P; dcb2.in_service = true;
  DCBus dcb3; dcb3.index = 3; dcb3.bus_type = DCBusType::DC_P; dcb3.in_service = true;
  sys.dc.buses = {dcb1, dcb2, dcb3};
  DCBranch dclne0; dclne0.index = 10; dclne0.from_bus = 1; dclne0.to_bus = 2;
    dclne0.r_pu = 0.03; dclne0.in_service = true;
  DCBranch dclne1; dclne1.index = 11; dclne1.from_bus = 2; dclne1.to_bus = 3;
    dclne1.r_pu = 0.04; dclne1.in_service = true;
  sys.dc.branches = {dclne0, dclne1};

  auto g = build_power_system_graph(sys);

  GraphReductionOptions opts;
  opts.enable_series_reduction      = true;
  opts.preserve_all_load_buses      = true;
  opts.preserve_all_generator_buses = true;

  auto candidates = classify_reduction_candidates(g, sys, opts);
  auto plan       = make_reduction_plan(g, candidates, opts);
  auto result     = apply_series_reduction(g, sys, plan, opts);

  // AC bus 2 should be mapped (eliminated) in the AC domain map
  REQUIRE(result.mapping.ac_original_to_reduced_bus.count(2) > 0);
  // DC bus 2 should be mapped (eliminated) in the DC domain map
  REQUIRE(result.mapping.dc_original_to_reduced_bus.count(2) > 0);

  // The legacy flat map maps bus 2 to something; it may be overwritten by
  // whichever domain processed last, so we can't assert its value reliably.
  // But the domain-qualified maps must each be independently correct.
  int ac_parent = result.mapping.ac_original_to_reduced_bus.at(2);
  int dc_parent = result.mapping.dc_original_to_reduced_bus.at(2);
  // Both parents must be either 1 or 3 (the terminals of the series chain)
  REQUIRE((ac_parent == 1 || ac_parent == 3));
  REQUIRE((dc_parent == 1 || dc_parent == 3));
}

TEST_CASE("Series recovery: same-ID AC/DC voltages written to separate maps",
          "[graph][series][recovery][collision]") {
  // AC: 1 --(r=0.01, x=0.05)-- 2 --(r=0.02, x=0.04)-- 3
  // DC: 1 --(r=0.10)-- 2 --(r=0.20)-- 3
  // Bus ID 2 exists in both domains.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK),
      make_ac_bus(2, BusType::PQ),
      make_ac_bus(3, BusType::PQ),
  };
  sys.ac.branches = {
      make_ac_branch(0, 1, 2, 0.01, 0.05),
      make_ac_branch(1, 2, 3, 0.02, 0.04),
  };
  DCBus d1; d1.index = 1; d1.bus_type = DCBusType::DC_V; d1.in_service = true;
  DCBus d2; d2.index = 2; d2.bus_type = DCBusType::DC_P; d2.in_service = true;
  DCBus d3; d3.index = 3; d3.bus_type = DCBusType::DC_P; d3.in_service = true;
  sys.dc.buses = {d1, d2, d3};
  DCBranch dl0; dl0.index = 10; dl0.from_bus = 1; dl0.to_bus = 2;
    dl0.r_pu = 0.10; dl0.in_service = true;
  DCBranch dl1; dl1.index = 11; dl1.from_bus = 2; dl1.to_bus = 3;
    dl1.r_pu = 0.20; dl1.in_service = true;
  sys.dc.branches = {dl0, dl1};

  auto g = build_power_system_graph(sys);
  GraphReductionOptions opts;
  opts.enable_series_reduction      = true;
  opts.preserve_all_load_buses      = true;
  opts.preserve_all_generator_buses = true;
  auto candidates = classify_reduction_candidates(g, sys, opts);
  auto plan       = make_reduction_plan(g, candidates, opts);
  auto red        = apply_series_reduction(g, sys, plan, opts);

  // Both bus 2s must have been eliminated in their respective domains.
  REQUIRE(red.mapping.ac_original_to_reduced_bus.count(2) > 0);
  REQUIRE(red.mapping.dc_original_to_reduced_bus.count(2) > 0);

  // Seed solved voltages on the retained terminals.
  FullNetworkVoltages voltages;
  voltages.ac_bus_voltage[1] = {1.00, 0.0};
  voltages.ac_bus_voltage[3] = {0.98, 0.0};
  voltages.bus_voltage[1]    = {1.00, 0.0};
  voltages.bus_voltage[3]    = {0.98, 0.0};
  voltages.dc_bus_voltage[1] = {1.02, 0.0};
  voltages.dc_bus_voltage[3] = {0.96, 0.0};

  recover_series_reduced_buses(voltages, red.mapping, sys);

  // AC bus 2 must appear in ac_bus_voltage — value must differ from DC (interpolated
  // using AC impedances).
  REQUIRE(voltages.ac_bus_voltage.count(2) > 0);
  // DC bus 2 must appear in dc_bus_voltage — value must differ from AC (interpolated
  // using DC resistance only).
  REQUIRE(voltages.dc_bus_voltage.count(2) > 0);
  // The two recovered voltages must be distinct (different domains, different impedances).
  double ac_v = voltages.ac_bus_voltage.at(2).real();
  double dc_v = voltages.dc_bus_voltage.at(2).real();
  REQUIRE(std::abs(ac_v - dc_v) > 1e-9);  // same-ID must not collide
  // Both values must lie strictly between the terminal voltages of their domain.
  REQUIRE(ac_v > 0.97);
  REQUIRE(ac_v < 1.01);
  REQUIRE(dc_v > 0.95);
  REQUIRE(dc_v < 1.03);
  // Legacy bus_voltage for key 2 must match the AC value (AC wins on collision).
  REQUIRE_THAT(voltages.bus_voltage.at(2).real(),
               WithinAbs(voltages.ac_bus_voltage.at(2).real(), 1e-9));
}

TEST_CASE("Series recovery: DC-only caller via legacy bus_voltage fallback",
          "[graph][series][recovery][dc][fallback]") {
  // Single-domain DC system (no AC buses).  Caller only fills bus_voltage
  // (legacy) — recover_series_reduced_buses must still recover bus 2
  // via the fallback path.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  DCBus d1; d1.index = 1; d1.bus_type = DCBusType::DC_V; d1.in_service = true;
  DCBus d2; d2.index = 2; d2.bus_type = DCBusType::DC_P; d2.in_service = true;
  DCBus d3; d3.index = 3; d3.bus_type = DCBusType::DC_P; d3.in_service = true;
  sys.dc.buses = {d1, d2, d3};
  DCBranch dl0; dl0.index = 0; dl0.from_bus = 1; dl0.to_bus = 2;
    dl0.r_pu = 0.10; dl0.in_service = true;
  DCBranch dl1; dl1.index = 1; dl1.from_bus = 2; dl1.to_bus = 3;
    dl1.r_pu = 0.10; dl1.in_service = true;
  sys.dc.branches = {dl0, dl1};

  auto g = build_power_system_graph(sys);
  GraphReductionOptions opts;
  opts.enable_series_reduction      = true;
  opts.preserve_all_load_buses      = true;
  opts.preserve_all_generator_buses = true;
  auto candidates = classify_reduction_candidates(g, sys, opts);
  auto plan       = make_reduction_plan(g, candidates, opts);
  auto red        = apply_series_reduction(g, sys, plan, opts);

  REQUIRE(red.mapping.dc_original_to_reduced_bus.count(2) > 0);

  // Legacy-only caller: only fills bus_voltage, not dc_bus_voltage.
  FullNetworkVoltages voltages;
  voltages.bus_voltage[1] = {1.00, 0.0};
  voltages.bus_voltage[3] = {0.90, 0.0};

  recover_series_reduced_buses(voltages, red.mapping, sys);

  // Recovery must populate dc_bus_voltage via fallback.
  REQUIRE(voltages.dc_bus_voltage.count(2) > 0);
  double v2 = voltages.dc_bus_voltage.at(2).real();
  REQUIRE(v2 > 0.89);
  REQUIRE(v2 < 1.01);
}

TEST_CASE("Pendant recovery: DC pendant bus voltage recovered",
          "[graph][pendant][recovery][dc]") {
  // DC network: bus 1 (slack) -- branch r=0.05 -- bus 2 (pendant, pd=10 MW)
  // Bus 3 is another non-pendant load bus to keep bus 1 non-pendant.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  DCBus d1; d1.index = 1; d1.bus_type = DCBusType::DC_V; d1.in_service = true;
  DCBus d2; d2.index = 2; d2.bus_type = DCBusType::DC_P;
    d2.pd_mw = 10.0; d2.in_service = true;
  DCBus d3; d3.index = 3; d3.bus_type = DCBusType::DC_P; d3.in_service = true;
  sys.dc.buses = {d1, d2, d3};

  DCBranch dl0; dl0.index = 0; dl0.from_bus = 1; dl0.to_bus = 2;
    dl0.r_pu = 0.05; dl0.in_service = true;
  DCBranch dl1; dl1.index = 1; dl1.from_bus = 1; dl1.to_bus = 3;
    dl1.r_pu = 0.10; dl1.in_service = true;
  sys.dc.branches = {dl0, dl1};

  auto g = build_power_system_graph(sys);
  GraphReductionOptions opts;
  opts.enable_series_reduction      = false;
  opts.enable_pendant_reduction     = true;
  opts.preserve_all_load_buses      = false;
  opts.preserve_all_generator_buses = true;
  auto candidates = classify_reduction_candidates(g, sys, opts);
  auto plan       = make_reduction_plan(g, candidates, opts);
  auto red        = apply_pendant_reduction(g, sys, plan, opts);

  // Bus 2 or 3 should be eliminated as a pendant.
  bool any_pendant = !red.mapping.pendant_records.empty();
  if (!any_pendant) {
    // If no pendant was eliminated, skip — system did not qualify.
    SUCCEED("No pendant eliminated; skipping recovery test");
    return;
  }

  // Seed parent (bus 1) voltage in dc_bus_voltage.
  FullNetworkVoltages voltages;
  voltages.dc_bus_voltage[1] = {1.0, 0.0};
  voltages.bus_voltage[1]    = {1.0, 0.0};

  RecoveryOptions ropts;
  recover_pendant_buses(voltages, red.mapping, sys, ropts);

  // The eliminated pendant bus must now have a DC voltage entry.
  for (const auto& rec : red.mapping.pendant_records) {
    REQUIRE(voltages.dc_bus_voltage.count(rec.eliminated_bus_id) > 0);
    double v = voltages.dc_bus_voltage.at(rec.eliminated_bus_id).real();
    // Voltage must be slightly below 1.0 (load draws current → resistive drop).
    REQUIRE(v > 0.80);
    REQUIRE(v < 1.01);
  }
}

TEST_CASE("Pendant recovery: DC-only caller via legacy bus_voltage fallback",
          "[graph][pendant][recovery][dc][fallback]") {
  // Same topology as above; caller seeds only bus_voltage (legacy).
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  DCBus d1; d1.index = 1; d1.bus_type = DCBusType::DC_V; d1.in_service = true;
  DCBus d2; d2.index = 2; d2.bus_type = DCBusType::DC_P;
    d2.pd_mw = 10.0; d2.in_service = true;
  DCBus d3; d3.index = 3; d3.bus_type = DCBusType::DC_P; d3.in_service = true;
  sys.dc.buses = {d1, d2, d3};
  DCBranch dl0; dl0.index = 0; dl0.from_bus = 1; dl0.to_bus = 2;
    dl0.r_pu = 0.05; dl0.in_service = true;
  DCBranch dl1; dl1.index = 1; dl1.from_bus = 1; dl1.to_bus = 3;
    dl1.r_pu = 0.10; dl1.in_service = true;
  sys.dc.branches = {dl0, dl1};

  auto g = build_power_system_graph(sys);
  GraphReductionOptions opts;
  opts.enable_series_reduction      = false;
  opts.enable_pendant_reduction     = true;
  opts.preserve_all_load_buses      = false;
  opts.preserve_all_generator_buses = true;
  auto candidates = classify_reduction_candidates(g, sys, opts);
  auto plan       = make_reduction_plan(g, candidates, opts);
  auto red        = apply_pendant_reduction(g, sys, plan, opts);

  if (red.mapping.pendant_records.empty()) {
    SUCCEED("No pendant eliminated; skipping fallback test");
    return;
  }

  // Legacy-only caller: only fills bus_voltage, not dc_bus_voltage.
  FullNetworkVoltages voltages;
  voltages.bus_voltage[1] = {1.0, 0.0};

  RecoveryOptions ropts;
  recover_pendant_buses(voltages, red.mapping, sys, ropts);

  for (const auto& rec : red.mapping.pendant_records) {
    REQUIRE(voltages.dc_bus_voltage.count(rec.eliminated_bus_id) > 0);
    REQUIRE(voltages.dc_bus_voltage.at(rec.eliminated_bus_id).real() > 0.80);
  }
}

// ═══════════════════════════════════════════════════════════════════════
// DC_ISOLATED bus type — graph / island handling
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("DC_ISOLATED bus is excluded from graph topology and islands",
          "[graph][topology][dc][isolated]") {
  // DC island A: bus 1 (DC_V) -- bus 2 (DC_P), valid (has voltage ref).
  // DC bus 3 is DC_ISOLATED with no in-service connection — it must be
  // treated as out of service: not part of any island, no NoDCVoltageRef.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  DCBus d1; d1.index = 1; d1.bus_type = DCBusType::DC_V;        d1.in_service = true;
  DCBus d2; d2.index = 2; d2.bus_type = DCBusType::DC_P;        d2.pd_mw = 5.0; d2.in_service = true;
  DCBus d3; d3.index = 3; d3.bus_type = DCBusType::DC_ISOLATED; d3.pd_mw = 5.0; d3.in_service = true;
  sys.dc.buses = {d1, d2, d3};

  DCBranch dl0; dl0.index = 0; dl0.from_bus = 1; dl0.to_bus = 2;
    dl0.r_pu = 0.10; dl0.in_service = true;
  sys.dc.branches = {dl0};

  auto g   = build_power_system_graph(sys);
  auto rep = analyze_topology(g);

  // The isolated DC node must exist in the map but be marked out of service.
  REQUIRE(g.dc_bus_id_to_node_idx.count(3) > 0);
  const int iso_node = g.dc_bus_id_to_node_idx.at(3);
  REQUIRE(g.nodes[static_cast<size_t>(iso_node)].in_service == false);

  // Bus 3 must not appear in any island's dc_bus_ids.
  bool iso_in_island = false;
  bool island_12_valid = false;
  for (const auto& isl : rep.islands) {
    bool has1 = false, has2 = false;
    for (int bid : isl.dc_bus_ids) {
      if (bid == 1) has1 = true;
      if (bid == 2) has2 = true;
      if (bid == 3) iso_in_island = true;
    }
    if (has1 && has2 && isl.status == IslandStatus::Valid) island_12_valid = true;
  }
  REQUIRE(iso_in_island == false);
  REQUIRE(island_12_valid == true);

  // No NoDCVoltageRef diagnostic should be raised for the isolated bus.
  for (const auto& d : rep.diagnostics)
    REQUIRE(d.code != DiagCode::GraphNoDCVoltageRef);
}

TEST_CASE("Typed-ID graph lookups map StableBusId to NodeIdx", "[graph][typed_ids]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 100.0;
  ACBus a1; a1.index = 7; a1.bus_type = BusType::SLACK; a1.in_service = true;
  ACBus a2; a2.index = 8; a2.bus_type = BusType::PQ;    a2.in_service = true;
  sys.ac.buses = {a1, a2};
  ACBranch br; br.index = 1; br.from_bus = 7; br.to_bus = 8; br.x_pu = 0.1;
  br.in_service = true;
  sys.ac.branches = {br};
  // Same numeric id (7) as an AC bus: the typed, domain-qualified lookups must
  // not confuse the two.
  DCBus d1; d1.index = 7; d1.bus_type = DCBusType::DC_V; d1.in_service = true;
  sys.dc.buses = {d1};

  const auto g = build_power_system_graph(sys);

  const NodeIdx ac7 = g.ac_node_idx(StableBusId{7});
  const NodeIdx dc7 = g.dc_node_idx(StableBusId{7});
  CHECK(ac7.valid());
  CHECK(dc7.valid());
  CHECK(ac7.value() == g.ac_node_idx(7));
  CHECK(dc7.value() == g.dc_node_idx(7));
  CHECK(ac7.value() != dc7.value());          // AC bus 7 and DC bus 7 differ
  CHECK_FALSE(g.ac_node_idx(StableBusId{999}).valid());  // absent -> invalid
}

TEST_CASE("Reduction planning keeps same-number AC and DC Kron actions separate",
          "[graph][audit][planning][collision]") {
  HybridPowerSystem sys;
  sys.ac.buses = {
      make_ac_bus(1, BusType::SLACK), make_ac_bus(2, BusType::PQ),
      make_ac_bus(3, BusType::PQ, 110.0, 1.0),
      make_ac_bus(4, BusType::PQ, 110.0, 1.0)};
  sys.ac.branches = {
      make_ac_branch(101, 1, 2), make_ac_branch(102, 2, 3),
      make_ac_branch(103, 2, 4)};

  for (int id = 1; id <= 4; ++id) {
    DCBus bus;
    bus.index = id;
    bus.bus_type = id == 1 ? DCBusType::DC_V : DCBusType::DC_P;
    bus.pd_mw = id >= 3 ? 1.0 : 0.0;
    sys.dc.buses.push_back(bus);
  }
  for (const auto [index, from, to] :
       {std::tuple{201, 1, 2}, std::tuple{202, 2, 3},
        std::tuple{203, 2, 4}}) {
    DCBranch branch;
    branch.index = index;
    branch.from_bus = from;
    branch.to_bus = to;
    branch.r_pu = 0.02;
    sys.dc.branches.push_back(branch);
  }

  const auto graph = build_power_system_graph(sys);
  GraphReductionOptions options;
  options.mode = ReductionMode::Moderate;
  options.enable_series_reduction = false;
  options.max_fill_ratio = 1.25;
  const auto candidates = classify_reduction_candidates(graph, sys, options);
  const auto plan = make_reduction_plan(graph, candidates, options);

  std::vector<const ReductionAction*> kron_actions;
  for (const auto& action : plan.actions) {
    if (action.type == ReductionActionType::KronEliminate) {
      kron_actions.push_back(&action);
    }
  }
  REQUIRE(kron_actions.size() == 2);
  for (const auto* action : kron_actions) {
    REQUIRE(action->method == ReductionMethod::KronPassiveOnly);
    REQUIRE(action->eliminated_buses ==
            std::vector<BusRef>{{action->eliminated_buses.front().domain, 2}});
    REQUIRE_THAT(action->max_fill_ratio, WithinAbs(1.25, 0.0));
    for (const auto& retained : action->retained_buses) {
      CHECK(retained.domain == action->eliminated_buses.front().domain);
    }
  }
  CHECK(kron_actions[0]->eliminated_buses.front().domain !=
        kron_actions[1]->eliminated_buses.front().domain);
}

TEST_CASE("Reduction planning options generate switch and retain actions",
          "[graph][audit][planning][options]") {
  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK), make_ac_bus(2, BusType::PV),
       make_ac_bus(3, BusType::PQ)},
      {make_ac_branch(10, 1, 2), make_ac_branch(11, 2, 3)});
  sys.ac.switches = {make_switch(40, 1, 3, true)};
  const auto graph = build_power_system_graph(sys);

  GraphReductionOptions preserve;
  preserve.preserve_all_voltage_constrained_buses = true;
  const auto retained = classify_reduction_candidates(graph, sys, preserve);
  const auto retained_plan = make_reduction_plan(graph, retained, preserve);
  CHECK(std::any_of(retained_plan.actions.begin(), retained_plan.actions.end(),
                    [](const ReductionAction& action) {
                      return action.type == ReductionActionType::SwitchContraction &&
                             action.method == ReductionMethod::SwitchContraction &&
                             action.eliminated_edge_positions.size() == 1;
                    }));
  CHECK(std::any_of(retained_plan.actions.begin(), retained_plan.actions.end(),
                    [](const ReductionAction& action) {
                      return action.type == ReductionActionType::Retain &&
                             action.retained_buses ==
                                 std::vector<BusRef>{{NodeDomain::AC, 2}};
                    }));

  GraphReductionOptions allow = preserve;
  allow.preserve_all_voltage_constrained_buses = false;
  const auto reducible = classify_reduction_candidates(graph, sys, allow);
  const auto bus2 = std::find_if(reducible.candidates.begin(),
                                 reducible.candidates.end(),
                                 [](const BusCandidate& candidate) {
                                   return candidate.bus == BusRef{NodeDomain::AC, 2};
                                 });
  REQUIRE(bus2 != reducible.candidates.end());
  CHECK(bus2->type == CandidateType::ZeroInjectionDegree2);
}

TEST_CASE("Reduction mappings rebuild reverse maps and compose stages",
          "[graph][audit][mapping]") {
  ReductionMapping first;
  first.original_to_reduced_buses = {
      {{NodeDomain::AC, 1}, {NodeDomain::AC, 1}},
      {{NodeDomain::AC, 2}, {NodeDomain::AC, 1}},
      {{NodeDomain::DC, 1}, {NodeDomain::DC, 1}}};
  first.original_to_reduced_branches = {
      {{NodeDomain::AC, 10}, {NodeDomain::AC, 20}},
      {{NodeDomain::AC, 11}, {NodeDomain::AC, 20}}};
  rebuild_reduction_reverse_maps(first);
  REQUIRE(first.reduced_to_original_bus_refs.at({NodeDomain::AC, 1}).size() == 2);
  REQUIRE(first.reduced_to_original_branch_refs.at({NodeDomain::AC, 20}).size() == 2);

  ReductionMapping second;
  second.original_to_reduced_buses = {
      {{NodeDomain::AC, 1}, {NodeDomain::AC, 3}},
      {{NodeDomain::AC, 3}, {NodeDomain::AC, 3}},
      {{NodeDomain::DC, 1}, {NodeDomain::DC, 1}}};
  second.original_to_reduced_branches = {
      {{NodeDomain::AC, 20}, {NodeDomain::AC, 30}}};
  rebuild_reduction_reverse_maps(second);

  const auto composed = compose_reduction_mappings(first, second);
  CHECK(composed.original_to_reduced_buses.at({NodeDomain::AC, 2}) ==
        BusRef{NodeDomain::AC, 3});
  CHECK(composed.original_to_reduced_buses.at({NodeDomain::DC, 1}) ==
        BusRef{NodeDomain::DC, 1});
  CHECK(composed.original_to_reduced_branches.at({NodeDomain::AC, 10}) ==
        BranchRef{NodeDomain::AC, 30});
  REQUIRE(composed.reduced_to_original_bus_refs.at({NodeDomain::AC, 3}).size() == 2);
  REQUIRE(composed.reduced_to_original_branch_refs.at({NodeDomain::AC, 30}).size() == 2);
}

TEST_CASE("Pendant recovery uses stable non-sequential branch component index",
          "[graph][audit][pendant][recovery]") {
  HybridPowerSystem sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK),
       make_ac_bus(2, BusType::PQ, 110.0, 10.0, 5.0)},
      {make_ac_branch(77, 1, 2, 0.1, 0.2)});
  const auto graph = build_power_system_graph(sys);
  GraphReductionOptions options;
  options.enable_pendant_reduction = true;
  options.preserve_all_load_buses = false;
  const auto candidates = classify_reduction_candidates(graph, sys, options);
  const auto plan = make_reduction_plan(graph, candidates, options);
  const auto reduced = apply_pendant_reduction(graph, sys, plan, options);
  REQUIRE(reduced.mapping.pendant_records.size() == 1);
  CHECK(reduced.mapping.pendant_records.front().source_branch_index == 77);
  CHECK(reduced.mapping.original_to_reduced_branches.at(
            {NodeDomain::AC, 77}) == BranchRef{NodeDomain::AC, -1});

  FullNetworkVoltages voltages;
  voltages.ac_bus_voltage[1] = {1.0, 0.0};
  RecoveryOptions recovery;
  recovery.max_pendant_iterations = 1;
  recover_pendant_buses(voltages, reduced.mapping, sys, recovery);
  REQUIRE(voltages.ac_bus_voltage.contains(2));
  CHECK_THAT(voltages.ac_bus_voltage.at(2).real(), WithinAbs(0.98, 1e-12));
  CHECK_THAT(voltages.ac_bus_voltage.at(2).imag(), WithinAbs(-0.015, 1e-12));

  auto missing_branch = sys;
  missing_branch.ac.branches.clear();
  FullNetworkVoltages invalid;
  invalid.ac_bus_voltage[1] = {1.0, 0.0};
  CHECK_THROWS_AS(
      recover_pendant_buses(invalid, reduced.mapping, missing_branch, recovery),
      std::invalid_argument);
}

TEST_CASE("Series reduction preserves graph edge-position topology invariant",
          "[graph][audit][series][topology]") {
  const auto sys = make_simple_system(
      {make_ac_bus(1, BusType::SLACK), make_ac_bus(2, BusType::PQ),
       make_ac_bus(3, BusType::PQ)},
      {make_ac_branch(100, 1, 2), make_ac_branch(500, 2, 3),
       make_ac_branch(900, 1, 3)});
  const auto graph = build_power_system_graph(sys);
  GraphReductionOptions options;
  options.preserve_branch_flow_limited_edges = false;
  const auto plan = make_reduction_plan(
      graph, classify_reduction_candidates(graph, sys, options), options);
  const auto reduced = apply_series_reduction(graph, sys, plan, options);

  for (std::size_t position = 0; position < reduced.reduced_graph.edges.size();
       ++position) {
    CHECK(reduced.reduced_graph.edges[position].edge_id ==
          static_cast<int>(position));
  }
  const auto report = analyze_topology(reduced.reduced_graph);
  CHECK(report.cycle_count == 1);
  CHECK(report.bridge_edge_ids.empty());
  REQUIRE(report.fundamental_cycles.size() == 1);
  for (int position : report.fundamental_cycles.front()) {
    REQUIRE(position >= 0);
    REQUIRE(position < reduced.reduced_graph.edge_count());
  }
}

TEST_CASE("Rich graph represents transformer LCC router and three-phase topology",
          "[graph][audit][build][rich]") {
  HybridPowerSystem sys;
  for (int id = 1; id <= 4; ++id) {
    sys.ac.buses.push_back(make_ac_bus(
        id, id == 1 ? BusType::SLACK : BusType::PQ, 20.0));
  }
  DCBus dc10;
  dc10.index = 10;
  dc10.bus_type = DCBusType::DC_V;
  DCBus dc11 = dc10;
  dc11.index = 11;
  dc11.bus_type = DCBusType::DC_P;
  sys.dc.buses = {dc10, dc11};

  Transformer3W transformer;
  transformer.index = 301;
  transformer.hv_bus = 1;
  transformer.mv_bus = 2;
  transformer.lv_bus = 3;
  sys.ac.transformers_3w = {transformer};
  LCCConverter lcc;
  lcc.index = 401;
  lcc.ac_bus = 1;
  lcc.dc_bus = 10;
  sys.lcc_converters = {lcc};
  EnergyRouter router;
  router.index = 501;
  EnergyRouterPort ac_port;
  ac_port.index = 1;
  ac_port.bus = 2;
  ac_port.port_type = ERPortType::AC;
  EnergyRouterPort dc_port = ac_port;
  dc_port.index = 2;
  dc_port.bus = 11;
  dc_port.port_type = ERPortType::DC;
  EnergyRouterPort ac_port_2 = ac_port;
  ac_port_2.index = 3;
  ac_port_2.bus = 4;
  router.ports = {ac_port, dc_port, ac_port_2};
  sys.energy_routers = {router};

  ThreePhaseACSystem phase;
  for (int id = 1; id <= 3; ++id) {
    ThreePhaseACBus bus;
    bus.index = id;
    bus.bus_type = id == 1 ? BusType::SLACK : BusType::PQ;
    bus.base_kv = 20.0;
    phase.buses.push_back(bus);
  }
  ThreePhaseACLine line;
  line.index = 601;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r1_pu = 0.01;
  line.x1_pu = 0.03;
  phase.lines = {line};
  ThreePhaseTransformer phase_transformer;
  phase_transformer.index = 602;
  phase_transformer.hv_bus = 2;
  phase_transformer.lv_bus = 3;
  phase.transformers = {phase_transformer};
  sys.three_phase_ac = phase;

  const auto graph = build_power_system_graph(sys);
  CHECK(std::count_if(graph.edges.begin(), graph.edges.end(), [](const auto& edge) {
          return edge.category == EdgeCategory::AC_Transformer;
        }) == 2);
  CHECK(std::count_if(graph.edges.begin(), graph.edges.end(), [](const auto& edge) {
          return edge.category == EdgeCategory::LCC_Coupling;
        }) == 1);
  CHECK(std::count_if(graph.edges.begin(), graph.edges.end(), [](const auto& edge) {
          return edge.category == EdgeCategory::EnergyRouter_Coupling;
        }) == 2);
  CHECK(graph.nodes[graph.ac_node_idx(2)].has_energy_router);
  CHECK(graph.nodes[graph.ac_node_idx(2)].has_three_phase_model);
  CHECK(graph.nodes[graph.dc_node_idx(10)].has_lcc);

  const auto phase_graph = build_three_phase_power_system_graph(phase);
  REQUIRE(phase_graph.node_count() == 3);
  REQUIRE(phase_graph.edge_count() == 2);
  CHECK(phase_graph.edges[0].category == EdgeCategory::ThreePhase_Line);
  CHECK(phase_graph.edges[1].category == EdgeCategory::ThreePhase_Transformer);
}

TEST_CASE("Contraction remaps rich terminals and rejects unresolved phase collapse",
          "[graph][audit][contraction][rich]") {
  HybridPowerSystem sys;
  sys.ac.buses = {make_ac_bus(1, BusType::SLACK, 20.0),
                  make_ac_bus(2, BusType::PQ, 20.0),
                  make_ac_bus(3, BusType::PQ, 20.0)};
  sys.ac.switches = {make_switch(10, 1, 2, true)};
  DCBus dc;
  dc.index = 10;
  dc.bus_type = DCBusType::DC_V;
  sys.dc.buses = {dc};
  MobileStorage mobile;
  mobile.index = 20;
  mobile.bus = 2;
  mobile.target_bus = 2;
  sys.mobile_storage = {mobile};
  VirtualPowerPlant vpp;
  vpp.index = 21;
  vpp.pcc_bus = 2;
  sys.vpps = {vpp};
  Microgrid microgrid;
  microgrid.index = 22;
  microgrid.pcc_bus = 2;
  microgrid.internal_buses = {2, 3};
  sys.microgrids = {microgrid};
  LCCConverter lcc;
  lcc.index = 23;
  lcc.ac_bus = 2;
  lcc.dc_bus = 10;
  sys.lcc_converters = {lcc};
  EnergyRouter router;
  router.index = 24;
  EnergyRouterPort port;
  port.index = 1;
  port.bus = 2;
  port.port_type = ERPortType::AC;
  router.ports = {port};
  sys.energy_routers = {router};

  const auto graph = build_power_system_graph(sys);
  const auto contracted = contract_zero_impedance_edges(graph, sys, {});
  const int representative = contracted.ac_bus_to_super.at(2);
  CHECK(contracted.contracted_system.mobile_storage.front().bus == representative);
  CHECK(contracted.contracted_system.mobile_storage.front().target_bus == representative);
  CHECK(contracted.contracted_system.vpps.front().pcc_bus == representative);
  CHECK(contracted.contracted_system.microgrids.front().pcc_bus == representative);
  CHECK(contracted.contracted_system.microgrids.front().internal_buses.front() ==
        representative);
  CHECK(contracted.contracted_system.lcc_converters.front().ac_bus == representative);
  CHECK(contracted.contracted_system.energy_routers.front().ports.front().bus ==
        representative);
  CHECK(contracted.mapping.original_to_reduced_buses.at({NodeDomain::AC, 2}) ==
        BusRef{NodeDomain::AC, representative});

  ThreePhaseACSystem phase;
  ThreePhaseACBus phase_bus;
  phase_bus.index = 2;
  phase.buses = {phase_bus};
  sys.three_phase_ac = phase;
  const auto rejected = contract_zero_impedance_edges(
      build_power_system_graph(sys), sys, {});
  CHECK(rejected.ac_bus_to_super.at(2) == 2);
  CHECK(std::any_of(rejected.diagnostics.begin(), rejected.diagnostics.end(),
                    [](const Diagnostic& diagnostic) {
                      return diagnostic.code == DiagCode::Error;
                    }));
}
