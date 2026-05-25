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
  REQUIRE(res.bus_to_super.count(2) > 0);
  REQUIRE(res.bus_to_super.count(3) > 0);
  int rep2 = res.bus_to_super.at(2);
  int rep3 = res.bus_to_super.at(3);
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
  if (res.bus_to_super.count(2) && res.bus_to_super.count(3))
    REQUIRE(res.bus_to_super.at(2) != res.bus_to_super.at(3));
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
  if (res.bus_to_super.count(2) && res.bus_to_super.count(3))
    REQUIRE(res.bus_to_super.at(2) != res.bus_to_super.at(3));
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
    if (c.bus_id == 2 && c.type == CandidateType::ZeroInjectionDegree2)
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
    if (c.bus_id == 2 && c.type == CandidateType::MustRetain)
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
    if (c.bus_id == 2 && c.type == CandidateType::PendantLoad)
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
  int rep = res.bus_to_super.at(1);
  voltages.bus_voltage[rep] = std::complex<double>{1.05, 0.0}; // 1.05∠0°

  recover_switch_contracted_buses(voltages, res.bus_to_super, res.super_to_buses);

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
  int ni_dc1 = g.node_idx(10);
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
