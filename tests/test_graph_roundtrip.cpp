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

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
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
  double residual{0.0};
  std::string termination_reason;
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
  s.residual = pf.residual;
  s.termination_reason = pf.diagnostics.termination_reason;
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

TEST_CASE("Round-trip: PF and AC OPF results are unprojected after pure AC reindexing",
          "[graph][roundtrip][reindex][opf]") {
  HybridPowerSystem s;
  s.base_mva = 10.0;
  s.ac.buses = {mk_bus(11, BusType::SLACK, 11.0),
                mk_bus(12, BusType::PQ, 11.0, 2.2, 0.6),
                mk_bus(13, BusType::PQ, 11.0, 1.0, 0.3)};
  s.ac.branches = {mk_branch(200, 11, 12, 0.02, 0.05),
                   mk_branch(201, 12, 13, 0.02, 0.05)};
  auto gen = mk_slack_gen(0, 11);
  gen.pmin_mw = 0.0;
  gen.pmax_mw = 20.0;
  gen.qmin_mvar = -20.0;
  gen.qmax_mvar = 20.0;
  gen.cost_c1 = 1.0;
  s.ac.generators = {gen};

  PowerFlowResult pf = solve_power_flow(s, {});
  REQUIRE(pf.converged);
  REQUIRE(pf.vm.size() == s.ac.buses.size());
  REQUIRE(pf.va.size() == s.ac.buses.size());

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 80;
  opt.max_outer_iterations = 1;
  const opf::ACOPFResult opf_result = solve_ac_opf(s, opt);
  REQUIRE(opf_result.vm.size() == s.ac.buses.size());
  REQUIRE(opf_result.va.size() == s.ac.buses.size());
  if (!opf_result.dpd_mw.empty()) {
    REQUIRE(opf_result.dpd_mw.size() == s.ac.buses.size());
  }
  if (!opf_result.dqd_mvar.empty()) {
    REQUIRE(opf_result.dqd_mvar.size() == s.ac.buses.size());
  }
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

// ═══════════════════════════════════════════════════════════════════════
// E. Hybrid AC/DC + VSC + DC circuit-breaker contraction
// ═══════════════════════════════════════════════════════════════════════

namespace {

DCBus mk_dc_bus(int idx, DCBusType type, double pd = 0.0, double base_kv = 0.4) {
  DCBus b;
  b.index      = idx;
  b.bus_type   = type;
  b.pd_mw      = pd;
  b.base_kv    = base_kv;
  b.vm_pu      = 1.0;
  b.in_service = true;
  return b;
}

DCBranch mk_dc_branch(int idx, int from, int to, double r) {
  DCBranch br;
  br.index      = idx;
  br.from_bus   = from;
  br.to_bus     = to;
  br.r_pu       = r;
  br.in_service = true;
  return br;
}

DCCircuitBreaker mk_dc_cb(int idx, int from, int to, bool closed = true) {
  DCCircuitBreaker cb;
  cb.index      = idx;
  cb.bus_from   = from;
  cb.bus_to     = to;
  cb.closed     = closed;
  cb.in_service = true;
  return cb;
}

}  // namespace

TEST_CASE("Round-trip: hybrid AC/DC with VSC and a closed DC breaker",
          "[graph][roundtrip][hybrid][contraction]") {
  // AC: 1(slack) --line-- 2 --line-- 3, VSC couples AC bus 3 to DC bus 101.
  // DC: 101(DC_V) --closed DC CB-- 104 --line-- 102(load).
  // The closed DC breaker (101-104) is a zero-impedance DC_Switch edge that
  // contraction merges; the VSC's bus_dc reference must follow the merge.
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0, 0.5, 0.2),
      mk_bus(3, BusType::PQ,    11.0, 0.4, 0.1),
  };
  sys.ac.branches   = {mk_branch(10, 1, 2, 0.02, 0.05),
                       mk_branch(11, 2, 3, 0.02, 0.05)};
  sys.ac.generators = {mk_slack_gen(0, 1)};

  sys.dc.base_mva = 10.0;
  sys.dc.buses = {
      mk_dc_bus(101, DCBusType::DC_V, 0.0),
      mk_dc_bus(102, DCBusType::DC_P, 0.3),
      mk_dc_bus(104, DCBusType::DC_P, 0.0),   // DC CB output node
  };
  sys.dc.branches = {mk_dc_branch(1, 104, 102, 0.01)};
  sys.dc.dc_circuit_breakers = {mk_dc_cb(1, 101, 104, /*closed=*/true)};

  {
    VSCConverter vsc;
    vsc.index        = 1;
    vsc.bus_ac       = 3;
    vsc.bus_dc       = 101;
    vsc.in_service   = true;
    vsc.control_mode = ConverterMode::PQ_MODE;
    vsc.p_set_mw     = 0.3;
    vsc.q_set_mvar   = 0.0;
    vsc.eta          = 0.98;
    vsc.pmax_mw      = 2.0;
    vsc.pmin_mw      = -2.0;
    sys.vsc_converters = {vsc};
  }

  // ── Contract: the closed DC breaker merges DC buses 101 and 104 ─────────
  auto g   = build_power_system_graph(sys);
  auto res = contract_zero_impedance_edges(g, sys, ContractionOptions{});

  REQUIRE(res.dc_bus_to_super.count(101) > 0);
  REQUIRE(res.dc_bus_to_super.count(104) > 0);
  const int dc_rep = res.dc_bus_to_super.at(101);
  REQUIRE(res.dc_bus_to_super.at(104) == dc_rep);
  // DC_V must win representative selection (voltage reference is preserved).
  REQUIRE(dc_rep == 101);

  // The VSC's DC terminal must now point at the surviving super-node.
  REQUIRE(res.contracted_system.vsc_converters.size() == 1);
  CHECK(res.contracted_system.vsc_converters[0].bus_dc == dc_rep);
  CHECK(res.contracted_system.vsc_converters[0].bus_ac == 3);   // AC side intact

  // The AC side is untouched (no AC zero-impedance elements).
  REQUIRE(res.contracted_system.ac.buses.size() == 3);

  // ── PF round-trip on the AC subsystem (VSC modelled as its scheduled
  //    injection).  The hybrid solver path is exercised by other suites;
  //    here we confirm the contracted hybrid system is still solvable and the
  //    AC voltages are unchanged by the DC-side contraction. ────────────────
  const SolvedPF full    = solve_pf(sys);
  const SolvedPF reduced = solve_pf(res.contracted_system);
  INFO("full PF: residual=" << full.residual << " reason=" << full.termination_reason);
  INFO("reduced PF: residual=" << reduced.residual << " reason=" << reduced.termination_reason);
  REQUIRE(full.converged);
  REQUIRE(reduced.converged);
  require_vm_match(full, reduced, {1, 2, 3}, 1e-6);
}

TEST_CASE("Round-trip: open DC breaker is not contracted in a hybrid system",
          "[graph][roundtrip][hybrid][contraction]") {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {mk_bus(1, BusType::SLACK, 11.0),
                  mk_bus(2, BusType::PQ, 11.0, 0.5, 0.2)};
  sys.ac.branches   = {mk_branch(10, 1, 2, 0.02, 0.05)};
  sys.ac.generators = {mk_slack_gen(0, 1)};

  sys.dc.base_mva = 10.0;
  sys.dc.buses = {mk_dc_bus(101, DCBusType::DC_V, 0.0),
                  mk_dc_bus(104, DCBusType::DC_P, 0.0)};
  sys.dc.dc_circuit_breakers = {mk_dc_cb(1, 101, 104, /*closed=*/false)};

  auto g   = build_power_system_graph(sys);
  auto res = contract_zero_impedance_edges(g, sys, ContractionOptions{});

  // Open breaker: DC buses 101 and 104 must NOT be merged.
  if (res.dc_bus_to_super.count(101) && res.dc_bus_to_super.count(104))
    REQUIRE(res.dc_bus_to_super.at(101) != res.dc_bus_to_super.at(104));
}

// ═══════════════════════════════════════════════════════════════════════
// F. Real distribution feeder (case33bw) series-reduction round-trip
// ═══════════════════════════════════════════════════════════════════════

namespace {

// Split AC branch at array position `pos` (a→b) into a→N→b through a new
// zero-injection bus N, halving the series impedance on each segment so the
// electrical path is unchanged.  Series reduction must then collapse N back
// and reproduce the original feeder's power flow exactly.
void split_branch_through_passive_bus(HybridPowerSystem& sys, int pos,
                                      int new_bus_id, int new_idx_a,
                                      int new_idx_b) {
  ACBranch& br = sys.ac.branches[static_cast<size_t>(pos)];
  const int a = br.from_bus, b = br.to_bus;
  const double r = br.r_pu, x = br.x_pu, bb = br.b_pu;
  const double base_kv = sys.ac.buses.empty() ? 12.66 : sys.ac.buses.front().base_kv;

  ACBus nb = mk_bus(new_bus_id, BusType::PQ, base_kv, 0.0, 0.0);  // passive
  sys.ac.buses.push_back(nb);

  ACBranch seg1 = mk_branch(new_idx_a, a, new_bus_id, r * 0.5, x * 0.5, bb * 0.5);
  ACBranch seg2 = mk_branch(new_idx_b, new_bus_id, b, r * 0.5, x * 0.5, bb * 0.5);
  // Remove the original branch, add the two segments.
  sys.ac.branches.erase(sys.ac.branches.begin() + pos);
  sys.ac.branches.push_back(seg1);
  sys.ac.branches.push_back(seg2);
}

}  // namespace

TEST_CASE("Round-trip: case33bw series reduction reproduces the feeder PF",
          "[graph][roundtrip][series][case33bw]") {
  const std::string path =
      std::string(HACDCPF_MATPOWER_DATA_DIR) + "/case33bw.m";
  HybridPowerSystem base = io::parse_matpower(path);
  REQUIRE(!base.ac.buses.empty());

  const SolvedPF full = solve_pf(base);
  REQUIRE(full.converged);

  // Build a "split" feeder: insert a passive bus midway along three branches,
  // using non-sequential indices to exercise the index-robust reducer.
  HybridPowerSystem split = base;
  int next_bus = 0;
  for (const auto& b : split.ac.buses) next_bus = std::max(next_bus, b.index);
  ++next_bus;
  int next_idx = 0;
  for (const auto& br : split.ac.branches) next_idx = std::max(next_idx, br.index);
  next_idx += 100;  // deliberately non-sequential
  for (int pos : {0, 5, 10}) {
    split_branch_through_passive_bus(split, pos, next_bus, next_idx, next_idx + 1);
    ++next_bus;
    next_idx += 2;
  }

  // The split feeder is electrically identical to the original.
  const SolvedPF split_full = solve_pf(split);
  REQUIRE(split_full.converged);

  // Series-reduce the passive split nodes back out.
  auto red = reduce_series(split);
  REQUIRE(red.mapping.series_records.size() >= 1);
  const SolvedPF reduced = solve_pf(red.reduced_system);
  REQUIRE(reduced.converged);

  // The reduced feeder must reproduce the ORIGINAL case33bw voltages at every
  // original bus.
  std::vector<int> orig_ids;
  for (const auto& b : base.ac.buses) orig_ids.push_back(b.index);
  require_vm_match(full, reduced, orig_ids, 1e-6);
  REQUIRE_THAT(reduced.total_loss_mw, WithinAbs(full.total_loss_mw, 1e-4));

  // Every eliminated passive bus recovers to its split-feeder voltage.
  FullNetworkVoltages volt = to_voltages(reduced);
  recover_series_reduced_buses(volt, red.mapping, split);
  for (const auto& rec : red.mapping.series_records) {
    const int elim = rec.eliminated_bus_id;
    REQUIRE(volt.ac_bus_voltage.count(elim) > 0);
    INFO("eliminated bus " << elim);
    REQUIRE_THAT(std::abs(volt.ac_bus_voltage.at(elim)),
                 WithinAbs(split_full.vm.at(elim), 1e-4));
  }
}

// ═══════════════════════════════════════════════════════════════════════
// H. Pendant (leaf-load) folding round-trip (approximate)
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Round-trip: pendant load folding conserves total demand",
          "[graph][roundtrip][pendant]") {
  // 1(slack) --Z-- 2(load) --Z-- 3(leaf load).  Bus 3 is folded into bus 2.
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.buses = {
      mk_bus(1, BusType::SLACK, 11.0),
      mk_bus(2, BusType::PQ,    11.0, 0.5, 0.2),
      mk_bus(3, BusType::PQ,    11.0, 0.3, 0.1),   // pendant leaf
  };
  sys.ac.branches   = {mk_branch(40, 1, 2, 0.01, 0.02),
                       mk_branch(41, 2, 3, 0.01, 0.02)};
  sys.ac.generators = {mk_slack_gen(0, 1)};

  auto g = build_power_system_graph(sys);
  GraphReductionOptions opt;
  opt.enable_pendant_reduction           = true;
  opt.preserve_all_load_buses            = false;  // allow folding
  opt.preserve_all_generator_buses       = true;
  opt.preserve_branch_flow_limited_edges = false;
  auto cand = classify_reduction_candidates(g, sys, opt);
  auto plan = make_reduction_plan(g, cand, opt);
  auto red  = apply_pendant_reduction(g, sys, plan, opt);

  // Bus 3 eliminated; the connecting branch (index 41) disabled by comp_index.
  REQUIRE(red.mapping.pendant_records.size() == 1);
  bool bus3_inactive = false;
  for (const auto& b : red.reduced_system.ac.buses)
    if (b.index == 3 && !b.in_service) bus3_inactive = true;
  REQUIRE(bus3_inactive);
  CHECK(count_in_service_with_index(red.reduced_system, {41}) == 0);

  // Parent bus 2 absorbs at least the leaf's demand (plus approximate I²R loss).
  double pd2 = 0.0;
  for (const auto& b : red.reduced_system.ac.buses)
    if (b.index == 2) pd2 = b.pd_mw;
  REQUIRE(pd2 >= 0.5 + 0.3 - 1e-9);   // original 0.5 + folded 0.3 (+ losses)

  // The reduced (now 2-bus) feeder still solves.
  const SolvedPF reduced = solve_pf(red.reduced_system);
  REQUIRE(reduced.converged);
}

// ═══════════════════════════════════════════════════════════════════════
// I. Kron (Schur complement) elimination round-trip
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Round-trip: Kron elimination of a passive interior node",
          "[graph][roundtrip][kron]") {
  // 4-bus star: passive hub (index 1, 0-based) joins buses 0, 2, 3.
  // Eliminate the hub and verify the boundary solution is preserved and the
  // hub voltage is recovered (V_beta = -Ybb^-1 Yba V_alpha).
  using cplx = std::complex<double>;
  auto y = [](double x) { return cplx{0.0, -1.0 / x}; };  // pure reactance line
  const cplx y01 = y(0.10), y12 = y(0.12), y13 = y(0.08);

  Eigen::MatrixXcd Y(4, 4);
  Y.setZero();
  // hub = node 1
  Y(0, 0) += y01;  Y(0, 1) -= y01;  Y(1, 0) -= y01;  Y(1, 1) += y01;
  Y(1, 1) += y12;  Y(1, 2) -= y12;  Y(2, 1) -= y12;  Y(2, 2) += y12;
  Y(1, 1) += y13;  Y(1, 3) -= y13;  Y(3, 1) -= y13;  Y(3, 3) += y13;

  Eigen::SparseMatrix<cplx> Ys = Y.sparseView();

  std::vector<int> retained   = {0, 2, 3};
  std::vector<int> eliminated = {1};
  auto kr = apply_kron_reduction(Ys, retained, eliminated, 10.0);
  REQUIRE(kr.kron_data.valid);

  // Pick boundary voltages; the hub is passive (zero injection).  Its true
  // voltage is determined by the boundary voltages, so set it from the
  // analytical passive-node relation before forming the full-network currents.
  Eigen::VectorXcd V_alpha(3);
  V_alpha << cplx{1.00, 0.0}, cplx{0.99, -0.01}, cplx{0.98, 0.02};

  // Recover the hub voltage from the reduced data.
  Eigen::VectorXcd V_beta = recover_eliminated_voltages(kr.kron_data, V_alpha);
  REQUIRE(V_beta.size() == 1);

  // Analytical hub voltage for a passive node: V_hub = Σ y_k V_k / Σ y_k.
  const cplx v_hub_expected =
      (y01 * V_alpha(0) + y12 * V_alpha(1) + y13 * V_alpha(2)) /
      (y01 + y12 + y13);
  REQUIRE_THAT(std::abs(V_beta(0) - v_hub_expected), WithinAbs(0.0, 1e-9));

  // Assemble the full voltage vector with the recovered hub voltage, so the
  // hub row of (Y·V) is zero (passive) and the boundary currents of the full
  // and reduced networks coincide.
  Eigen::VectorXcd V_full(4);
  V_full << V_alpha(0), V_beta(0), V_alpha(1), V_alpha(2);
  Eigen::VectorXcd I_full = Y * V_full;
  REQUIRE(std::abs(I_full(1)) < 1e-9);   // hub injection is ~0 (passive)

  Eigen::VectorXcd I_alpha_expected(3);
  I_alpha_expected << I_full(0), I_full(2), I_full(3);
  Eigen::VectorXcd I_alpha_reduced = kr.Y_reduced * V_alpha;
  REQUIRE((I_alpha_reduced - I_alpha_expected).cwiseAbs().maxCoeff() < 1e-9);
}
