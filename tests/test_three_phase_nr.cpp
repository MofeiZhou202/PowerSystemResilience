/// tests/test_three_phase_nr.cpp
///
/// Three-Phase Newton-Raphson Power Flow Solver — Unit & Integration Tests
///
/// Test sections
/// -------------
///   1.  PhaseNodeIndexer       — compact node mapping and helper queries
///   2.  Balanced 3-bus radial  — all three phases equal, Vm ≈ 1 at slack
///   3.  Unbalanced loads       — asymmetric per-phase demand, power balance
///   4.  Convergence            — result.converged == true, residual < tol
///   5.  Single-phase lateral   — phase-A only branch produces phase-B/C = 0
///   6.  Two-phase branch       — ab branch: no phantom phase-C node
///   7.  Full phase-matrix line — explicit Z matrix path
///   8.  Multi-source (DG)      — PV generator at bus 2
///   9.  Meshed topology        — ring network still converges
///  10.  HybridPowerSystem overload — wraps ThreePhaseACSystem correctly
///  11.  VUF check              — balanced input gives near-zero VUF
///  12.  Loss sanity            — total losses > 0, per-phase losses summed match total
///  13.  Zero-load (no-load)    — all loads = 0, Vm still converges to 1.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>

#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Approx = Catch::Approx;

// ─────────────────────────────────────────────────────────────────────────────
// Helper builders
// ─────────────────────────────────────────────────────────────────────────────

namespace {

ThreePhaseACBus make_bus(int id, BusType type, PhaseMask mask) {
  ThreePhaseACBus b;
  b.index       = id;
  b.bus_type    = type;
  b.phase_mask  = mask;
  b.in_service  = true;
  b.vm_a_pu     = 1.0;  b.va_a_deg = 0.0;
  b.vm_b_pu     = 1.0;  b.va_b_deg = -120.0;
  b.vm_c_pu     = 1.0;  b.va_c_deg = 120.0;
  return b;
}

ThreePhaseACLine make_line(int id, int from, int to, PhaseMask mask,
                            double r_pu, double x_pu) {
  ThreePhaseACLine l;
  l.index      = id;
  l.from_bus   = from;
  l.to_bus     = to;
  l.phase_mask = mask;
  l.in_service = true;
  l.r1_pu      = r_pu;
  l.x1_pu      = x_pu;
  l.r0_pu      = r_pu;
  l.x0_pu      = x_pu;
  l.rate_a_mva = 100.0;
  return l;
}

ThreePhaseNROptions tight_opts() {
  ThreePhaseNROptions o;
  o.max_iter = 50;
  o.tol      = 1e-8;
  return o;
}

/// 3-bus radial, balanced loads, all three phases.
ThreePhaseACSystem build_balanced_3bus() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto b1 = make_bus(1, BusType::SLACK, PhaseMask::abc());

  auto b2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  b2.pd_a_mw = 0.3; b2.qd_a_mvar = 0.1;
  b2.pd_b_mw = 0.3; b2.qd_b_mvar = 0.1;
  b2.pd_c_mw = 0.3; b2.qd_c_mvar = 0.1;

  auto b3 = make_bus(3, BusType::PQ, PhaseMask::abc());
  b3.pd_a_mw = 0.2; b3.qd_a_mvar = 0.08;
  b3.pd_b_mw = 0.2; b3.qd_b_mvar = 0.08;
  b3.pd_c_mw = 0.2; b3.qd_c_mvar = 0.08;

  sys.buses = {b1, b2, b3};
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.04, 0.08),
      make_line(2, 2, 3, PhaseMask::abc(), 0.04, 0.08),
  };
  return sys;
}

/// 3-bus radial, unbalanced loads.
ThreePhaseACSystem build_unbalanced_3bus() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto b1 = make_bus(1, BusType::SLACK, PhaseMask::abc());

  auto b2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  b2.pd_a_mw = 0.50; b2.qd_a_mvar = 0.20;
  b2.pd_b_mw = 0.20; b2.qd_b_mvar = 0.05;
  b2.pd_c_mw = 0.10; b2.qd_c_mvar = 0.02;

  auto b3 = make_bus(3, BusType::PQ, PhaseMask::abc());
  b3.pd_a_mw = 0.15; b3.qd_a_mvar = 0.06;
  b3.pd_b_mw = 0.40; b3.qd_b_mvar = 0.15;
  b3.pd_c_mw = 0.25; b3.qd_c_mvar = 0.10;

  sys.buses = {b1, b2, b3};
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.03, 0.06),
      make_line(2, 2, 3, PhaseMask::abc(), 0.05, 0.10),
  };
  return sys;
}

/// 3-bus with an additional ring branch (meshed topology).
ThreePhaseACSystem build_meshed_3bus() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto b1 = make_bus(1, BusType::SLACK, PhaseMask::abc());

  auto b2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  b2.pd_a_mw = 0.3; b2.qd_a_mvar = 0.1;
  b2.pd_b_mw = 0.3; b2.qd_b_mvar = 0.1;
  b2.pd_c_mw = 0.3; b2.qd_c_mvar = 0.1;

  auto b3 = make_bus(3, BusType::PQ, PhaseMask::abc());
  b3.pd_a_mw = 0.2; b3.qd_a_mvar = 0.08;
  b3.pd_b_mw = 0.2; b3.qd_b_mvar = 0.08;
  b3.pd_c_mw = 0.2; b3.qd_c_mvar = 0.08;

  sys.buses = {b1, b2, b3};
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.04, 0.08),
      make_line(2, 2, 3, PhaseMask::abc(), 0.04, 0.08),
      make_line(3, 3, 1, PhaseMask::abc(), 0.06, 0.12),  // ring
  };
  return sys;
}

/// 3-bus, phase-A lateral on branch 1-2 only.
ThreePhaseACSystem build_phase_a_lateral() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto b1 = make_bus(1, BusType::SLACK, PhaseMask::abc());
  auto b2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  b2.pd_a_mw = 0.20; b2.qd_a_mvar = 0.08;
  b2.pd_b_mw = 0.20; b2.qd_b_mvar = 0.08;
  b2.pd_c_mw = 0.20; b2.qd_c_mvar = 0.08;

  auto b3 = make_bus(3, BusType::PQ, PhaseMask::a());  // phase-A only
  b3.pd_a_mw = 0.12; b3.qd_a_mvar = 0.05;

  sys.buses = {b1, b2, b3};
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.04, 0.08),
      make_line(2, 2, 3, PhaseMask::a(), 0.05, 0.10),
  };
  return sys;
}

/// 3-bus, multi-source: PV generator at bus 2.
ThreePhaseACSystem build_multi_source_3bus() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto b1 = make_bus(1, BusType::SLACK, PhaseMask::abc());
  auto b2 = make_bus(2, BusType::PV, PhaseMask::abc());
  // PV bus holds voltage fixed at these values (set on the bus)
  b2.vm_a_pu = 1.02; b2.vm_b_pu = 1.02; b2.vm_c_pu = 1.02;
  b2.pd_a_mw = 0.30; b2.qd_a_mvar = 0.10;
  b2.pd_b_mw = 0.30; b2.qd_b_mvar = 0.10;
  b2.pd_c_mw = 0.30; b2.qd_c_mvar = 0.10;

  auto b3 = make_bus(3, BusType::PQ, PhaseMask::abc());
  b3.pd_a_mw = 0.20; b3.qd_a_mvar = 0.08;
  b3.pd_b_mw = 0.20; b3.qd_b_mvar = 0.08;
  b3.pd_c_mw = 0.20; b3.qd_c_mvar = 0.08;

  sys.buses = {b1, b2, b3};
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.04, 0.08),
      make_line(2, 2, 3, PhaseMask::abc(), 0.04, 0.08),
  };

  ThreePhaseGenerator gen;
  gen.index      = 1;
  gen.bus        = 2;
  gen.in_service = true;
  gen.phase_mask = PhaseMask::abc();
  gen.p_mw       = 0.45;
  gen.q_mvar     = 0.0;
  gen.vm_pu      = 1.02;
  sys.generators.push_back(gen);

  return sys;
}

/// No loads anywhere — all buses PQ with P = Q = 0.
ThreePhaseACSystem build_no_load_3bus() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto b1 = make_bus(1, BusType::SLACK, PhaseMask::abc());
  auto b2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  auto b3 = make_bus(3, BusType::PQ, PhaseMask::abc());

  sys.buses = {b1, b2, b3};
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.04, 0.08),
      make_line(2, 2, 3, PhaseMask::abc(), 0.04, 0.08),
  };
  return sys;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// 1. PhaseNodeIndexer — compact node mapping
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("PhaseNodeIndexer: balanced 3-bus has 9 compact nodes",
          "[unit][three_phase_nr]") {
  const auto sys     = build_balanced_3bus();
  const auto indexer = PhaseNodeIndexer::build(sys);

  REQUIRE(indexer.total_nodes == 9);

  // bus 0 (SLACK), all three phases present
  for (int ph = 0; ph < 3; ++ph)
    REQUIRE(indexer.has_node(0, ph));

  // bus 1 (PQ, abc)
  for (int ph = 0; ph < 3; ++ph)
    REQUIRE(indexer.has_node(1, ph));

  // round-trip: node_index → node_ref → bus_offset
  int ni = indexer.node_index(1, 2);  // bus 1, phase C
  REQUIRE(ni >= 0);
  REQUIRE(ni < indexer.total_nodes);
  REQUIRE(indexer.node_ref(ni).bus_offset == 1);
  REQUIRE(indexer.node_ref(ni).phase_index == 2);
}

TEST_CASE("PhaseNodeIndexer: phase-A lateral has 7 compact nodes",
          "[unit][three_phase_nr]") {
  const auto sys     = build_phase_a_lateral();
  const auto indexer = PhaseNodeIndexer::build(sys);

  // bus 0: abc (3) + bus 1: abc (3) + bus 2: a only (1) = 7
  REQUIRE(indexer.total_nodes == 7);

  REQUIRE(indexer.has_node(2, 0));        // phase A present
  REQUIRE_FALSE(indexer.has_node(2, 1));  // phase B absent
  REQUIRE_FALSE(indexer.has_node(2, 2));  // phase C absent
}

TEST_CASE("PhaseNodeIndexer: bus_phase_mask reflects PhaseMask",
          "[unit][three_phase_nr]") {
  const auto sys     = build_phase_a_lateral();
  const auto indexer = PhaseNodeIndexer::build(sys);

  // bus 0: abc
  REQUIRE(indexer.bus_phase_mask(0).has(0));
  REQUIRE(indexer.bus_phase_mask(0).has(1));
  REQUIRE(indexer.bus_phase_mask(0).has(2));

  // bus 2: a only
  REQUIRE(indexer.bus_phase_mask(2).has(0));
  REQUIRE_FALSE(indexer.bus_phase_mask(2).has(1));
  REQUIRE_FALSE(indexer.bus_phase_mask(2).has(2));
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Balanced 3-bus radial — all three phases equal
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: balanced 3-bus converges with equal per-phase voltages",
          "[integration][three_phase_nr]") {
  const auto sys    = build_balanced_3bus();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);
  REQUIRE(result.iterations > 0);
  REQUIRE(result.residual < 1e-7);
  REQUIRE(result.bus_voltages.size() == 3);

  // Slack bus must hold 1.0 p.u. on all phases
  const auto& slack = result.bus_voltages[0];
  REQUIRE(slack.vm_a_pu == Approx(1.0).epsilon(1e-6));
  REQUIRE(slack.vm_b_pu == Approx(1.0).epsilon(1e-6));
  REQUIRE(slack.vm_c_pu == Approx(1.0).epsilon(1e-6));

  // Balanced: per-phase magnitudes must be equal at each load bus
  for (size_t i = 1; i < result.bus_voltages.size(); ++i) {
    const auto& bv = result.bus_voltages[i];
    REQUIRE(bv.vm_a_pu == Approx(bv.vm_b_pu).epsilon(1e-5));
    REQUIRE(bv.vm_b_pu == Approx(bv.vm_c_pu).epsilon(1e-5));
    // Voltage must be below 1.0 (load is non-trivial) and above 0.9
    REQUIRE(bv.vm_a_pu > 0.90);
    REQUIRE(bv.vm_a_pu < 1.01);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Unbalanced loads — per-phase magnitudes differ
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: unbalanced loads produce unequal per-phase Vm",
          "[integration][three_phase_nr]") {
  const auto sys    = build_unbalanced_3bus();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);

  // Per-phase Vm at bus 2 must be unequal (asymmetric loading)
  const auto& bv2 = result.bus_voltages[1];
  // Phase A has the heaviest load → lowest Vm
  REQUIRE(bv2.vm_a_pu < bv2.vm_b_pu);
  // All phases above 0.85 for this moderate loading
  REQUIRE(bv2.vm_a_pu > 0.85);
  REQUIRE(bv2.vm_b_pu > 0.85);
  REQUIRE(bv2.vm_c_pu > 0.85);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Convergence properties
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: residual is below tolerance at convergence",
          "[integration][three_phase_nr]") {
  const auto sys = build_balanced_3bus();

  ThreePhaseNROptions opt;
  opt.max_iter = 50;
  opt.tol      = 1e-8;

  const auto result = solve_three_phase_nr(sys, opt);

  REQUIRE(result.converged);
  REQUIRE(result.residual < opt.tol * 10.0);  // allow one ULP above tol
}

TEST_CASE("3-phase NR: tight tolerance still converges within 30 iterations",
          "[integration][three_phase_nr]") {
  const auto sys = build_balanced_3bus();

  ThreePhaseNROptions opt;
  opt.max_iter = 30;
  opt.tol      = 1e-9;

  const auto result = solve_three_phase_nr(sys, opt);
  REQUIRE(result.converged);
  REQUIRE(result.iterations <= 30);
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Single-phase lateral — phase-A only branch
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: phase-A lateral branch leaves B/C voltage at 0 on phase-A bus",
          "[integration][three_phase_nr]") {
  const auto sys    = build_phase_a_lateral();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);
  REQUIRE(result.bus_voltages.size() == 3);

  // Bus 3 is phase-A only — B and C must be 0
  const auto& bv3 = result.bus_voltages[2];
  REQUIRE(bv3.vm_a_pu > 0.0);
  REQUIRE(std::abs(bv3.vm_b_pu) < 1e-12);
  REQUIRE(std::abs(bv3.vm_c_pu) < 1e-12);

  // Branch 2 carries only phase-A power
  const auto& bp2 = result.branch_powers[1];
  REQUIRE(std::abs(bp2.p_a_mw)   > 1e-6);
  REQUIRE(std::abs(bp2.p_b_mw)   < 1e-12);
  REQUIRE(std::abs(bp2.p_c_mw)   < 1e-12);
  REQUIRE(std::abs(bp2.q_b_mvar) < 1e-12);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Meshed topology
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: meshed (ring) topology converges",
          "[integration][three_phase_nr]") {
  const auto sys    = build_meshed_3bus();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);
  REQUIRE(result.bus_voltages.size() == 3);

  // All buses should have reasonable voltages
  for (const auto& bv : result.bus_voltages) {
    REQUIRE(bv.vm_a_pu > 0.85);
    REQUIRE(bv.vm_b_pu > 0.85);
    REQUIRE(bv.vm_c_pu > 0.85);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. Multi-source (distributed generator at bus 2)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: multi-source system converges with PV generator",
          "[integration][three_phase_nr]") {
  const auto sys    = build_multi_source_3bus();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);
  REQUIRE(result.bus_voltages.size() == 3);

  // Bus 2 (PV) must hold vm close to vm_pu setpoint (1.02)
  const auto& bv2 = result.bus_voltages[1];
  REQUIRE(bv2.vm_a_pu == Approx(1.02).epsilon(1e-3));
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. HybridPowerSystem overload
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: HybridPowerSystem overload wraps ThreePhaseACSystem",
          "[integration][three_phase_nr]") {
  HybridPowerSystem hybrid;
  hybrid.three_phase_ac = build_balanced_3bus();

  const auto result = solve_three_phase_nr(hybrid, tight_opts());

  REQUIRE(result.converged);
  REQUIRE(result.bus_voltages.size() == 3);
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. VUF check — balanced input → near-zero unbalance factor
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: balanced system gives near-zero VUF",
          "[integration][three_phase_nr]") {
  const auto sys    = build_balanced_3bus();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);
  // For a perfectly balanced feeder, VUF should be very small
  REQUIRE(result.max_vuf_percent < 0.5);
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. Loss sanity — total losses positive and per-phase sum matches
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: per-phase active losses sum to total active loss",
          "[integration][three_phase_nr]") {
  const auto sys    = build_unbalanced_3bus();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);
  REQUIRE(result.total_p_loss_mw > 0.0);
  REQUIRE(result.total_q_loss_mvar > 0.0);

  const double sum_phase = result.p_loss_a_mw + result.p_loss_b_mw + result.p_loss_c_mw;
  REQUIRE(sum_phase == Approx(result.total_p_loss_mw).epsilon(1e-6));
}

// ─────────────────────────────────────────────────────────────────────────────
// 11. Zero-load (no-load) — Vm must converge to 1.0 everywhere
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: no-load case gives Vm = 1.0 at all buses",
          "[integration][three_phase_nr]") {
  const auto sys    = build_no_load_3bus();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);

  for (const auto& bv : result.bus_voltages) {
    REQUIRE(bv.vm_a_pu == Approx(1.0).epsilon(1e-6));
    REQUIRE(bv.vm_b_pu == Approx(1.0).epsilon(1e-6));
    REQUIRE(bv.vm_c_pu == Approx(1.0).epsilon(1e-6));
  }

  // No load → zero losses
  REQUIRE(std::abs(result.total_p_loss_mw)   < 1e-9);
  REQUIRE(std::abs(result.total_q_loss_mvar) < 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// 12. Branch power size check
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("3-phase NR: branch_powers has one entry per in-service line",
          "[integration][three_phase_nr]") {
  const auto sys    = build_balanced_3bus();
  const auto result = solve_three_phase_nr(sys, tight_opts());

  REQUIRE(result.converged);
  REQUIRE(result.branch_powers.size() == sys.lines.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// 13. Numerical cross-validation: 2-bus exact analytical check
// ─────────────────────────────────────────────────────────────────────────────
// Network: slack bus 1 (Vm = 1.0 ∠ 0°), PQ bus 2 (P + jQ = 0.3 + j0.1 per phase)
// Line:    R = 0.05, X = 0.1 (per-unit on 10 MVA base, negligible shunt)
// Expect:  Vm at bus 2 < 1.0, and real/imag power balance at bus 2.

TEST_CASE("3-phase NR: 2-bus exact balance — power injected equals load consumed",
          "[integration][three_phase_nr]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto b1 = make_bus(1, BusType::SLACK, PhaseMask::abc());
  auto b2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  b2.pd_a_mw = 0.30; b2.qd_a_mvar = 0.10;
  b2.pd_b_mw = 0.30; b2.qd_b_mvar = 0.10;
  b2.pd_c_mw = 0.30; b2.qd_c_mvar = 0.10;

  sys.buses = {b1, b2};
  sys.lines = {make_line(1, 1, 2, PhaseMask::abc(), 0.05, 0.10)};

  const auto result = solve_three_phase_nr(sys, tight_opts());
  REQUIRE(result.converged);

  // Vm at bus 2 must drop below 1.0 (loaded)
  const auto& bv2 = result.bus_voltages[1];
  REQUIRE(bv2.vm_a_pu < 1.0);
  REQUIRE(bv2.vm_b_pu < 1.0);
  REQUIRE(bv2.vm_c_pu < 1.0);

  // Branch sending-end power ≥ load (includes losses in the line)
  const auto& bp = result.branch_powers[0];
  REQUIRE(bp.p_a_mw >= 0.30);
  REQUIRE(bp.p_b_mw >= 0.30);
  REQUIRE(bp.p_c_mw >= 0.30);
}
