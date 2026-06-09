// tests/test_graph_roundtrip.cpp
//
// Round-trip validation for the network-level graph aggregation
// (hacdcpf::graph).  The strategy is: solve a power flow / OPF on the FULL
// network, aggregate the network with the graph module, solve again on the
// REDUCED network, recover the eliminated quantities, and assert that the
// physical solution is preserved (bus voltages, branch flows, total losses,
// and OPF objective).
//
// Aggregation classes and their expected fidelity:
//   - Switch / breaker contraction : EXACT  (V_i = V_K across zero-Z merges)
//   - Series reduction (no charging): EXACT  (Z_ik = Z_ij + Z_jk)
//   - Pendant / Kron reduction      : APPROX (flat-start / linear model)
//
// Test groups (this file grows iteratively):
//   A. Switch / circuit-breaker contraction round-trip
//
// All comparisons are made on quantities that are invariant to bus-level
// projection, so they isolate the *network-level* aggregation under test.

#include <cmath>
#include <unordered_map>
#include <vector>
#include <algorithm>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using namespace hacdcpf;
using namespace hacdcpf::graph;
using Catch::Matchers::WithinAbs;

namespace {

// ═══════════════════════════════════════════════════════════════════════
// Model construction helpers
// ═══════════════════════════════════════════════════════════════════════

ACBus mk_bus(int idx, BusType type, double base_kv = 11.0,
             double pd = 0.0, double qd = 0.0) {
  ACBus b;
  b.index      = idx;
  b.bus_type   = type;
  b.base_kv    = base_kv;
  b.pd_mw      = pd;
  b.qd_mvar    = qd;
  b.vm_pu      = 1.0;
  b.va_deg     = 0.0;
  b.vmin_pu    = 0.90;
  b.vmax_pu    = 1.10;
  b.in_service = true;
  return b;
}

ACBranch mk_branch(int idx, int from, int to,
                   double r = 0.01, double x = 0.03, double b = 0.0,
                   double rate_a = 0.0) {
  ACBranch br;
  br.index      = idx;
  br.from_bus   = from;
  br.to_bus     = to;
  br.r_pu       = r;
  br.x_pu       = x;
  br.b_pu       = b;
  br.tap        = 1.0;
  br.shift_deg  = 0.0;
  br.rate_a_mva = rate_a;
  br.in_service = true;
  return br;
}

Switch mk_switch(int idx, int from, int to, bool closed = true) {
  Switch sw;
  sw.index      = idx;
  sw.bus_from   = from;
  sw.bus_to     = to;
  sw.closed     = closed;
  sw.in_service = true;
  return sw;
}

CircuitBreaker mk_cb(int idx, int from, int to, bool closed = true) {
  CircuitBreaker cb;
  cb.index      = idx;
  cb.bus_from   = from;
  cb.bus_to     = to;
  cb.closed     = closed;
  cb.in_service = true;
  return cb;
}

Generator mk_slack_gen(int idx, int bus, double vg = 1.0) {
  Generator g;
  g.index      = idx;
  g.bus        = bus;
  g.in_service = true;
  g.is_slack   = true;
  g.vg_pu      = vg;
  g.pg_mw      = 0.0;
  g.pmax_mw    = 1e4;
  g.pmin_mw    = -1e4;
  g.qmax_mvar  = 1e4;
  g.qmin_mvar  = -1e4;
  return g;
}

// ═══════════════════════════════════════════════════════════════════════
// Solve helpers
// ═══════════════════════════════════════════════════════════════════════

struct SolvedPF {
  bool converged{false};
  std::unordered_map<int, double> vm;   ///< bus_id -> |V| pu
  std::unordered_map<int, double> va;   ///< bus_id -> angle (raw result units)
  double total_loss_mw{0.0};            ///< Σ branch (pf + pt)
};

// solve_power_flow projects + un-projects internally, so pf.vm[i] corresponds
// to sys.ac.buses[i].index.  We build a bus_id-keyed map for robust comparison
// regardless of the internal projection ordering.
SolvedPF solve_pf(const HybridPowerSystem& sys) {
  PowerFlowOptions opt;
  PowerFlowResult pf = solve_power_flow(sys, opt);
  SolvedPF s;
  s.converged = pf.converged;
  for (size_t i = 0; i < sys.ac.buses.size() && i < pf.vm.size(); ++i) {
    if (!sys.ac.buses[i].in_service) continue;
    s.vm[sys.ac.buses[i].index] = pf.vm[i];
    if (i < pf.va.size()) s.va[sys.ac.buses[i].index] = pf.va[i];
  }
  for (const auto& bf : pf.branch_flows)
    s.total_loss_mw += bf.pf_mw + bf.pt_mw;
  return s;
}

// Compare voltage magnitudes at a set of (retained) bus IDs.
void require_vm_match(const SolvedPF& full, const SolvedPF& reduced,
                      const std::vector<int>& bus_ids, double tol) {
  for (int bid : bus_ids) {
    REQUIRE(full.vm.count(bid) > 0);
    REQUIRE(reduced.vm.count(bid) > 0);
    INFO("bus " << bid << ": full=" << full.vm.at(bid)
                << " reduced=" << reduced.vm.at(bid));
    REQUIRE_THAT(reduced.vm.at(bid), WithinAbs(full.vm.at(bid), tol));
  }
}

// Seed a FullNetworkVoltages structure (complex pu) from a solved PF.
// PowerFlowResult.va is in radians, so V = |V| · e^{jθ}.
FullNetworkVoltages to_voltages(const SolvedPF& s) {
  FullNetworkVoltages v;
  for (const auto& [bid, vm] : s.vm) {
    const double va = s.va.count(bid) ? s.va.at(bid) : 0.0;
    v.bus_voltage[bid]    = std::polar(vm, va);
    v.ac_bus_voltage[bid] = std::polar(vm, va);
  }
  return v;
}

// Run the full series-reduction pipeline (classify → plan → apply).
SeriesReductionResult reduce_series(const HybridPowerSystem& sys) {
  auto g = build_power_system_graph(sys);
  GraphReductionOptions opt;
  opt.enable_series_reduction            = true;
  opt.preserve_all_load_buses            = true;
  opt.preserve_all_generator_buses       = true;
  opt.preserve_branch_flow_limited_edges = false;  // rate_a=0 branches anyway
  auto cand = classify_reduction_candidates(g, sys, opt);
  auto plan = make_reduction_plan(g, cand, opt);
  return apply_series_reduction(g, sys, plan, opt);
}

// Count in-service AC branches whose index is in `ids`.
int count_in_service_with_index(const HybridPowerSystem& sys,
                                const std::vector<int>& ids) {
  int n = 0;
  for (const auto& br : sys.ac.branches)
    for (int id : ids)
      if (br.index == id && br.in_service) ++n;
  return n;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// A. Switch / circuit-breaker contraction round-trip
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Round-trip: non-1-based AC bus indices converge after projection",
          "[graph][roundtrip][reindex]") {
  // A standalone sub-network whose bus IDs do not start at 1 (e.g. a contracted
  // island that preserves original IDs).  The projection must reindex it to a
  // 1-based-contiguous canonical form so the positional Y-bus builder keeps its
  // branches.  Regression for the "non-1-based PF" bug.
  HybridPowerSystem s;
  s.base_mva = 10.0;
  s.ac.buses = {mk_bus(11, BusType::SLACK, 11.0),
                mk_bus(12, BusType::PQ, 11.0, 2.2, 0.6),
                mk_bus(13, BusType::PQ, 11.0, 1.0, 0.3)};
  s.ac.branches   = {mk_branch(200, 11, 12, 0.02, 0.05),
                     mk_branch(201, 12, 13, 0.02, 0.05)};
  s.ac.generators = {mk_slack_gen(0, 11)};
  PowerFlowResult pf = solve_power_flow(s, {});
  REQUIRE(pf.converged);
  // Voltages drop monotonically down the feeder.
  REQUIRE(pf.vm.size() == 3);
  REQUIRE(pf.vm[0] > pf.vm[1]);
  REQUIRE(pf.vm[1] > pf.vm[2]);
}

TEST_CASE("DIAG: GraphEdge::comp_index is populated from model .index",
          "[graph][roundtrip][diag]") {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {mk_bus(1, BusType::SLACK), mk_bus(2, BusType::PQ),
                  mk_bus(3, BusType::PQ, 11.0, 2.0, 0.8)};
  sys.ac.branches   = {mk_branch(101, 1, 2), mk_branch(202, 2, 3)};
  sys.ac.generators = {mk_slack_gen(0, 1)};
  auto g = build_power_system_graph(sys);
  REQUIRE(g.edges.size() == 2);
  std::vector<int> ci{g.edges[0].comp_index, g.edges[1].comp_index};
  std::sort(ci.begin(), ci.end());
  INFO("comp_index values: " << ci[0] << ", " << ci[1]);
  CHECK(ci[0] == 101);
  CHECK(ci[1] == 202);
}

TEST_CASE("Round-trip: closed AC switch contraction preserves PF solution",
          "[graph][roundtrip][contraction]") {
  // 1(slack) --line-- 2(load) --closed switch-- 3(load)
  // Switch contraction merges buses 2 and 3 into one super-node.
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0, 1.0, 0.4),
      mk_bus(3, BusType::PQ,    11.0, 1.5, 0.6),
  };
  sys.ac.branches   = {mk_branch(0, 1, 2, 0.02, 0.05)};
  sys.ac.switches   = {mk_switch(0, 2, 3, /*closed=*/true)};
  sys.ac.generators = {mk_slack_gen(0, 1)};

  const SolvedPF full = solve_pf(sys);
  REQUIRE(full.converged);

  // Network-level aggregation: contract zero-impedance / closed switches.
  auto g   = build_power_system_graph(sys);
  auto res = contract_zero_impedance_edges(g, sys, ContractionOptions{});

  // The contracted system must still be solvable.
  const SolvedPF reduced = solve_pf(res.contracted_system);
  REQUIRE(reduced.converged);

  // Retained bus 1 and the super-node representative must match the full solve.
  REQUIRE(res.ac_bus_to_super.count(2) > 0);
  const int rep = res.ac_bus_to_super.at(2);
  require_vm_match(full, reduced, {1, rep}, 1e-6);

  // Recover the merged buses and check every original bus matches the full PF.
  FullNetworkVoltages volt;
  for (const auto& [bid, vm] : reduced.vm) {
    volt.bus_voltage[bid]    = std::polar(vm, 0.0);
    volt.ac_bus_voltage[bid] = std::polar(vm, 0.0);
  }
  recover_switch_contracted_buses(volt, res);
  for (int bid : {1, 2, 3}) {
    REQUIRE(volt.ac_bus_voltage.count(bid) > 0);
    REQUIRE_THAT(std::abs(volt.ac_bus_voltage.at(bid)),
                 WithinAbs(full.vm.at(bid), 1e-6));
  }

  // Total losses are invariant under zero-impedance merging.
  REQUIRE_THAT(reduced.total_loss_mw, WithinAbs(full.total_loss_mw, 1e-6));
}

TEST_CASE("Round-trip: closed AC circuit-breaker contraction preserves PF",
          "[graph][roundtrip][contraction]") {
  // 1(slack) --line-- 2 --closed CB-- 3(load)
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0, 0.8, 0.3),
      mk_bus(3, BusType::PQ,    11.0, 2.0, 0.7),
  };
  sys.ac.branches        = {mk_branch(0, 1, 2, 0.03, 0.06)};
  sys.ac.circuit_breakers = {mk_cb(0, 2, 3, /*closed=*/true)};
  sys.ac.generators      = {mk_slack_gen(0, 1)};

  const SolvedPF full = solve_pf(sys);
  REQUIRE(full.converged);

  auto g   = build_power_system_graph(sys);
  auto res = contract_zero_impedance_edges(g, sys, ContractionOptions{});

  const SolvedPF reduced = solve_pf(res.contracted_system);
  REQUIRE(reduced.converged);

  REQUIRE(res.ac_bus_to_super.count(3) > 0);
  const int rep = res.ac_bus_to_super.at(3);
  require_vm_match(full, reduced, {1, rep}, 1e-6);
  REQUIRE_THAT(reduced.total_loss_mw, WithinAbs(full.total_loss_mw, 1e-6));
}

TEST_CASE("Round-trip: open switch leaves the network unchanged",
          "[graph][roundtrip][contraction]") {
  // 1(slack) --line-- 2(load) --line-- 3(load), with an OPEN tie switch 2-3'.
  // The open switch must not merge anything: reduced solve == full solve.
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0, 1.0, 0.3),
      mk_bus(3, BusType::PQ,    11.0, 1.2, 0.4),
  };
  sys.ac.branches   = {mk_branch(0, 1, 2, 0.02, 0.05),
                       mk_branch(1, 2, 3, 0.02, 0.05)};
  sys.ac.switches   = {mk_switch(0, 1, 3, /*closed=*/false)};  // open tie
  sys.ac.generators = {mk_slack_gen(0, 1)};

  const SolvedPF full = solve_pf(sys);
  REQUIRE(full.converged);

  auto g   = build_power_system_graph(sys);
  auto res = contract_zero_impedance_edges(g, sys, ContractionOptions{});

  const SolvedPF reduced = solve_pf(res.contracted_system);
  REQUIRE(reduced.converged);

  // No merge happened — all three buses survive and match.
  require_vm_match(full, reduced, {1, 2, 3}, 1e-6);
  REQUIRE_THAT(reduced.total_loss_mw, WithinAbs(full.total_loss_mw, 1e-6));
}

// ═══════════════════════════════════════════════════════════════════════
// B. Series reduction round-trip (passive degree-2 elimination)
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Round-trip: series reduction (0-based indices) preserves PF",
          "[graph][roundtrip][series]") {
  // 1(slack) --Z12-- 2(passive) --Z23-- 3(load)
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0),            // passive degree-2
      mk_bus(3, BusType::PQ,    11.0, 2.0, 0.8),  // load
  };
  sys.ac.branches   = {mk_branch(0, 1, 2, 0.01, 0.05),
                       mk_branch(1, 2, 3, 0.02, 0.04)};
  sys.ac.generators = {mk_slack_gen(0, 1)};

  const SolvedPF full = solve_pf(sys);
  REQUIRE(full.converged);

  auto red = reduce_series(sys);
  // Bus 2 must have been eliminated.
  REQUIRE(red.mapping.ac_original_to_reduced_bus.count(2) > 0);

  // Structural: the two original branches (index 0 and 1) must be disabled.
  CHECK(count_in_service_with_index(red.reduced_system, {0, 1}) == 0);

  const SolvedPF reduced = solve_pf(red.reduced_system);
  REQUIRE(reduced.converged);
  require_vm_match(full, reduced, {1, 3}, 1e-6);
  REQUIRE_THAT(reduced.total_loss_mw, WithinAbs(full.total_loss_mw, 1e-6));

  // Recover the eliminated bus 2 and compare to the full solve.
  FullNetworkVoltages volt = to_voltages(reduced);
  recover_series_reduced_buses(volt, red.mapping, sys);
  REQUIRE(volt.ac_bus_voltage.count(2) > 0);
  REQUIRE_THAT(std::abs(volt.ac_bus_voltage.at(2)),
               WithinAbs(full.vm.at(2), 1e-6));
}

TEST_CASE("Round-trip: series reduction with non-sequential branch indices",
          "[graph][roundtrip][series]") {
  // Same chain, but branch .index values are MATPOWER-style (not array
  // positions).  This probes whether the reducer disables branches by their
  // component index rather than by the graph's positional edge id.
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0),
      mk_bus(3, BusType::PQ,    11.0, 2.0, 0.8),
  };
  sys.ac.branches   = {mk_branch(101, 1, 2, 0.01, 0.05),
                       mk_branch(202, 2, 3, 0.02, 0.04)};
  sys.ac.generators = {mk_slack_gen(0, 1)};

  const SolvedPF full = solve_pf(sys);
  REQUIRE(full.converged);

  auto red = reduce_series(sys);
  REQUIRE(red.mapping.ac_original_to_reduced_bus.count(2) > 0);

  // Structural: the two original branches (index 101 and 202) must be disabled.
  CHECK(count_in_service_with_index(red.reduced_system, {101, 202}) == 0);

  const SolvedPF reduced = solve_pf(red.reduced_system);
  REQUIRE(reduced.converged);
  require_vm_match(full, reduced, {1, 3}, 1e-6);
  REQUIRE_THAT(reduced.total_loss_mw, WithinAbs(full.total_loss_mw, 1e-6));

  FullNetworkVoltages volt = to_voltages(reduced);
  recover_series_reduced_buses(volt, red.mapping, sys);
  REQUIRE(volt.ac_bus_voltage.count(2) > 0);
  REQUIRE_THAT(std::abs(volt.ac_bus_voltage.at(2)),
               WithinAbs(full.vm.at(2), 1e-6));
}

TEST_CASE("Round-trip: series reduction on a 5-bus passive chain",
          "[graph][roundtrip][series]") {
  // 1(slack) -- 2 -- 3 -- 4 -- 5(load), buses 2,3,4 passive.
  // A single pass eliminates the non-adjacent passive nodes (2 and 4).
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0),
      mk_bus(3, BusType::PQ,    11.0),
      mk_bus(4, BusType::PQ,    11.0),
      mk_bus(5, BusType::PQ,    11.0, 1.5, 0.6),
  };
  sys.ac.branches   = {mk_branch(0, 1, 2, 0.01, 0.03),
                       mk_branch(1, 2, 3, 0.01, 0.03),
                       mk_branch(2, 3, 4, 0.01, 0.03),
                       mk_branch(3, 4, 5, 0.01, 0.03)};
  sys.ac.generators = {mk_slack_gen(0, 1)};

  const SolvedPF full = solve_pf(sys);
  REQUIRE(full.converged);

  auto red = reduce_series(sys);
  REQUIRE(red.mapping.series_records.size() >= 1);

  const SolvedPF reduced = solve_pf(red.reduced_system);
  REQUIRE(reduced.converged);
  // Retained terminals (slack and load) must match exactly.
  require_vm_match(full, reduced, {1, 5}, 1e-6);
  REQUIRE_THAT(reduced.total_loss_mw, WithinAbs(full.total_loss_mw, 1e-6));

  // Every eliminated bus must recover to its full-solve voltage.
  FullNetworkVoltages volt = to_voltages(reduced);
  recover_series_reduced_buses(volt, red.mapping, sys);
  for (const auto& rec : red.mapping.series_records) {
    const int elim = rec.eliminated_bus_id;
    REQUIRE(volt.ac_bus_voltage.count(elim) > 0);
    INFO("eliminated bus " << elim);
    REQUIRE_THAT(std::abs(volt.ac_bus_voltage.at(elim)),
                 WithinAbs(full.vm.at(elim), 1e-5));
  }
}

// ═══════════════════════════════════════════════════════════════════════
// C. Meshed (looped) network round-trip
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Round-trip: series reduction on a meshed network (with a chord)",
          "[graph][roundtrip][series][meshed]") {
  // Ring 1-2-3-4-1 plus chord 2-4 (meshed), with a passive junction bus 5
  // spliced into the 2–3 leg: 2 --Z-- 5(passive) --Z-- 3.
  // Non-sequential branch indices exercise the index-robust reducer.
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0, 1.0, 0.4),
      mk_bus(3, BusType::PQ,    11.0, 0.8, 0.3),
      mk_bus(4, BusType::PQ,    11.0, 0.6, 0.2),
      mk_bus(5, BusType::PQ,    11.0),            // passive junction
  };
  sys.ac.branches = {
      mk_branch(10, 1, 2, 0.02, 0.06),
      mk_branch(20, 2, 5, 0.01, 0.03),   // 2–5
      mk_branch(30, 5, 3, 0.01, 0.03),   // 5–3  (bus 5 is degree-2 passive)
      mk_branch(40, 3, 4, 0.02, 0.05),
      mk_branch(50, 4, 1, 0.02, 0.05),   // ring closure
      mk_branch(60, 2, 4, 0.03, 0.07),   // chord → meshed
  };
  sys.ac.generators = {mk_slack_gen(0, 1)};

  const SolvedPF full = solve_pf(sys);
  REQUIRE(full.converged);

  auto red = reduce_series(sys);
  REQUIRE(red.mapping.ac_original_to_reduced_bus.count(5) > 0);
  CHECK(count_in_service_with_index(red.reduced_system, {20, 30}) == 0);

  const SolvedPF reduced = solve_pf(red.reduced_system);
  REQUIRE(reduced.converged);
  require_vm_match(full, reduced, {1, 2, 3, 4}, 1e-6);
  REQUIRE_THAT(reduced.total_loss_mw, WithinAbs(full.total_loss_mw, 1e-6));

  FullNetworkVoltages volt = to_voltages(reduced);
  recover_series_reduced_buses(volt, red.mapping, sys);
  REQUIRE(volt.ac_bus_voltage.count(5) > 0);
  REQUIRE_THAT(std::abs(volt.ac_bus_voltage.at(5)),
               WithinAbs(full.vm.at(5), 1e-5));
}

// ═══════════════════════════════════════════════════════════════════════
// D. Islanded multi-component round-trip
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Round-trip: switch contraction across two independent islands",
          "[graph][roundtrip][contraction][islands]") {
  // Island A: 1(slack) --line-- 2 --closed switch-- 3(load)
  // Island B: 11(slack) --line-- 12 --closed CB-- 13(load)
  // The two islands share no buses or branches.
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1,  BusType::SLACK, 11.0),
      mk_bus(2,  BusType::PQ,    11.0, 0.7, 0.2),
      mk_bus(3,  BusType::PQ,    11.0, 1.1, 0.5),
      mk_bus(11, BusType::SLACK, 11.0),
      mk_bus(12, BusType::PQ,    11.0, 0.9, 0.3),
      mk_bus(13, BusType::PQ,    11.0, 1.3, 0.6),
  };
  sys.ac.branches = {mk_branch(100, 1, 2, 0.02, 0.05),
                     mk_branch(200, 11, 12, 0.02, 0.05)};
  sys.ac.switches  = {mk_switch(0, 2, 3, true)};
  sys.ac.circuit_breakers = {mk_cb(0, 12, 13, true)};
  sys.ac.generators = {mk_slack_gen(0, 1), mk_slack_gen(1, 11)};

  // ── Structural: contraction must keep the two islands separate ──────────
  auto g0 = build_power_system_graph(sys);
  REQUIRE(analyze_topology(g0).n_ac_islands == 2);

  auto res = contract_zero_impedance_edges(g0, sys, ContractionOptions{});

  // Still two islands after contraction.
  auto g1 = build_power_system_graph(res.contracted_system);
  REQUIRE(analyze_topology(g1).n_ac_islands == 2);
  // Merges happen within each island, never across islands.
  REQUIRE(res.ac_bus_to_super.at(2)  == res.ac_bus_to_super.at(3));
  REQUIRE(res.ac_bus_to_super.at(12) == res.ac_bus_to_super.at(13));
  REQUIRE(res.ac_bus_to_super.at(2)  != res.ac_bus_to_super.at(12));
  // Loads aggregate per island; both slacks survive.
  REQUIRE(res.contracted_system.ac.buses.size() == 4);
  int n_slack = 0;
  for (const auto& b : res.contracted_system.ac.buses)
    if (b.bus_type == BusType::SLACK) ++n_slack;
  REQUIRE(n_slack == 2);

  // ── PF exactness per island.  solve_power_flow targets a single island, so
  //    each island is round-tripped as a standalone system (a combined
  //    multi-island solve is the domain of solve_power_flow_islanded).  Island
  //    B uses non-1-based bus IDs, exercising the projection reindex. ─────────
  auto round_trip_island = [](HybridPowerSystem isl,
                              int slack_bus, int merged_a, int merged_b) {
    const SolvedPF full = solve_pf(isl);
    REQUIRE(full.converged);
    auto gg  = build_power_system_graph(isl);
    auto cr  = contract_zero_impedance_edges(gg, isl, ContractionOptions{});
    const SolvedPF reduced = solve_pf(cr.contracted_system);
    REQUIRE(reduced.converged);
    REQUIRE(cr.ac_bus_to_super.count(merged_b) > 0);
    const int rep = cr.ac_bus_to_super.at(merged_b);
    require_vm_match(full, reduced, {slack_bus, rep}, 1e-6);
    FullNetworkVoltages volt;
    for (const auto& [bid, vm] : reduced.vm) {
      volt.bus_voltage[bid]    = std::polar(vm, 0.0);
      volt.ac_bus_voltage[bid] = std::polar(vm, 0.0);
    }
    recover_switch_contracted_buses(volt, cr);
    for (int bid : {slack_bus, merged_a, merged_b}) {
      REQUIRE(volt.ac_bus_voltage.count(bid) > 0);
      REQUIRE_THAT(std::abs(volt.ac_bus_voltage.at(bid)),
                   WithinAbs(full.vm.at(bid), 1e-6));
    }
  };

  HybridPowerSystem islA;
  islA.base_mva = 10.0;
  islA.ac.buses = {mk_bus(1, BusType::SLACK, 11.0),
                   mk_bus(2, BusType::PQ, 11.0, 0.7, 0.2),
                   mk_bus(3, BusType::PQ, 11.0, 1.1, 0.5)};
  islA.ac.branches   = {mk_branch(100, 1, 2, 0.02, 0.05)};
  islA.ac.switches   = {mk_switch(0, 2, 3, true)};
  islA.ac.generators = {mk_slack_gen(0, 1)};
  round_trip_island(islA, 1, 2, 3);

  HybridPowerSystem islB;
  islB.base_mva = 10.0;
  islB.ac.buses = {mk_bus(11, BusType::SLACK, 11.0),
                   mk_bus(12, BusType::PQ, 11.0, 0.9, 0.3),
                   mk_bus(13, BusType::PQ, 11.0, 1.3, 0.6)};
  islB.ac.branches         = {mk_branch(200, 11, 12, 0.02, 0.05)};
  islB.ac.circuit_breakers = {mk_cb(0, 12, 13, true)};
  islB.ac.generators       = {mk_slack_gen(0, 11)};
  round_trip_island(islB, 11, 12, 13);
}

// ═══════════════════════════════════════════════════════════════════════
// G. DC OPF round-trip (objective + dispatch preserved under reduction)
// ═══════════════════════════════════════════════════════════════════════

namespace {

Generator mk_cost_gen(int idx, int bus, double pmax, double c1) {
  Generator g;
  g.index      = idx;
  g.bus        = bus;
  g.in_service = true;
  g.is_slack   = (idx == 0);
  g.vg_pu      = 1.0;
  g.pg_mw      = 0.0;
  g.pmax_mw    = pmax;
  g.pmin_mw    = 0.0;
  g.qmax_mvar  = 1e3;
  g.qmin_mvar  = -1e3;
  g.cost_c1    = c1;   // linear $/MWh
  g.cost_c2    = 0.0;
  return g;
}

}  // namespace

TEST_CASE("Round-trip: DC OPF objective preserved under series reduction",
          "[graph][roundtrip][series][opf]") {
  // 1(cheap gen, slack) --Z-- 2(passive) --Z-- 3(load) --Z-- 4(expensive gen)
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 110.0),
      mk_bus(2, BusType::PQ,    110.0),              // passive
      mk_bus(3, BusType::PQ,    110.0, 80.0, 0.0),   // load
      mk_bus(4, BusType::PV,    110.0),
  };
  sys.ac.branches = {mk_branch(7, 1, 2, 0.005, 0.03),
                     mk_branch(8, 2, 3, 0.005, 0.03),
                     mk_branch(9, 3, 4, 0.005, 0.03)};
  sys.ac.generators = {mk_cost_gen(0, 1, 200.0, 10.0),
                       mk_cost_gen(1, 4, 200.0, 30.0)};

  opf::DCOPFOptions oo;
  oo.verbose = false;
  oo.compute_lmp = false;
  oo.include_branch_limits = false;

  auto full_opf = hacdcpf::solve_dc_opf(sys, oo);
  REQUIRE(full_opf.converged);

  auto red = reduce_series(sys);
  REQUIRE(red.mapping.ac_original_to_reduced_bus.count(2) > 0);
  CHECK(count_in_service_with_index(red.reduced_system, {7, 8}) == 0);

  auto red_opf = hacdcpf::solve_dc_opf(red.reduced_system, oo);
  REQUIRE(red_opf.converged);

  INFO("full obj=" << full_opf.objective << " reduced obj=" << red_opf.objective);
  REQUIRE_THAT(red_opf.objective, WithinAbs(full_opf.objective, 1e-3));
}

