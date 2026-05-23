/// tests/test_three_phase_pf.cpp
///
/// Three-phase power flow tests.
///
/// Architecture note
/// -----------------
/// The library implements three-phase support through a *projection* layer:
/// when a HybridPowerSystem carries a ThreePhaseACSystem in its
/// three_phase_ac field, project_three_phase_if_needed() (network_utils.cpp)
/// automatically converts the three-phase buses, lines, transformers, and
/// loads into positive-sequence single-phase equivalents before the NR
/// solver runs.  This means solve_power_flow() is the entry point for all
/// three-phase cases, and the returned vm/va vectors correspond to the
/// projected positive-sequence network.
///
/// Test sections
/// -------------
///   1.  Data model           — build / query ThreePhaseACSystem fields
///   2.  Projection           — three_phase_ac → single-phase equivalent
///   3.  Balanced radial      — 3-bus balanced, expect symmetric Vm/Va
///   4.  Unbalanced loads     — asymmetric load distribution, power balance
///   5.  Meshed topology      — 3-bus with ring branch, solver still converges
///   6.  Multi-source         — DG at bus 3, slack-only at bus 1
///   7.  Transformer          — Δ-Yg 3-bus with two-winding transformer
///   8.  Distribution PF      — solve_distribution_power_flow stub behaviour
///   9.  JSON round-trip      — serialize / deserialize ThreePhaseACSystem

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <numeric>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"
#include "hacdcpf/io/json_io.hpp"

using namespace hacdcpf;
using namespace hacdcpf::io;
using Catch::Approx;

// ──────────────────────────────────────────────────────────────────────────────
// Helper: balanced 3-bus 3-phase radial network.
//
// Topology:   slack(1) ─ L1 ─ bus2 ─ L2 ─ bus3
// base_mva = 10, base_kv = 11 kV
// Line parameters (positive-sequence, in p.u. on 10 MVA / 11 kV base):
//   L1: r1=0.10  x1=0.20   L2: r1=0.08  x1=0.15
// Loads (balanced across all three phases):
//   bus2: 0.5+0.3j MW/phase  →  1.5+0.9j MW total
//   bus3: 0.8+0.4j MW/phase  →  2.4+1.2j MW total
// ──────────────────────────────────────────────────────────────────────────────
static ThreePhaseACSystem build_balanced_3bus() {
    ThreePhaseACSystem sys;
    sys.base_mva = 10.0;
    sys.name = "balanced_3bus";

    // bus 1 – slack
    ThreePhaseACBus b1;
    b1.index = 1; b1.bus_type = BusType::SLACK;
    b1.vm_a_pu = 1.0; b1.va_a_deg = 0.0;
    b1.vm_b_pu = 1.0; b1.va_b_deg = -120.0;
    b1.vm_c_pu = 1.0; b1.va_c_deg = 120.0;
    b1.base_kv = 11.0; b1.in_service = true;

    // bus 2 – load bus
    ThreePhaseACBus b2;
    b2.index = 2; b2.bus_type = BusType::PQ;
    b2.vm_a_pu = 1.0; b2.va_a_deg = 0.0;
    b2.vm_b_pu = 1.0; b2.va_b_deg = -120.0;
    b2.vm_c_pu = 1.0; b2.va_c_deg = 120.0;
    b2.pd_a_mw = 0.5; b2.qd_a_mvar = 0.3;
    b2.pd_b_mw = 0.5; b2.qd_b_mvar = 0.3;
    b2.pd_c_mw = 0.5; b2.qd_c_mvar = 0.3;
    b2.base_kv = 11.0; b2.in_service = true;

    // bus 3 – load bus
    ThreePhaseACBus b3;
    b3.index = 3; b3.bus_type = BusType::PQ;
    b3.vm_a_pu = 1.0; b3.va_a_deg = 0.0;
    b3.vm_b_pu = 1.0; b3.va_b_deg = -120.0;
    b3.vm_c_pu = 1.0; b3.va_c_deg = 120.0;
    b3.pd_a_mw = 0.8; b3.qd_a_mvar = 0.4;
    b3.pd_b_mw = 0.8; b3.qd_b_mvar = 0.4;
    b3.pd_c_mw = 0.8; b3.qd_c_mvar = 0.4;
    b3.base_kv = 11.0; b3.in_service = true;

    sys.buses = {b1, b2, b3};

    ThreePhaseACLine l1, l2;
    l1.index = 1; l1.from_bus = 1; l1.to_bus = 2;
    l1.r1_pu = 0.10; l1.x1_pu = 0.20;
    l1.r0_pu = 0.10; l1.x0_pu = 0.20;
    l1.in_service = true;

    l2.index = 2; l2.from_bus = 2; l2.to_bus = 3;
    l2.r1_pu = 0.08; l2.x1_pu = 0.15;
    l2.r0_pu = 0.08; l2.x0_pu = 0.15;
    l2.in_service = true;

    sys.lines = {l1, l2};

    ThreePhaseGenerator g;
    g.index = 1; g.bus = 1;
    g.is_slack = true; g.vm_pu = 1.0;
    g.pmax_mw = 100.0; g.in_service = true;
    sys.generators = {g};

    return sys;
}

// Wrap the ThreePhaseACSystem in a HybridPowerSystem ready for solving.
static HybridPowerSystem as_hybrid(ThreePhaseACSystem tp) {
    HybridPowerSystem sys;
    sys.base_mva = tp.base_mva;
    sys.three_phase_ac = std::move(tp);
    return sys;
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 1 – Data model
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("ThreePhaseACSystem: bus fields round-trip through struct", "[three_phase]") {
    ThreePhaseACBus bus;
    bus.index = 5;
    bus.bus_type = BusType::PQ;
    bus.vm_a_pu = 1.02; bus.va_a_deg = 0.0;
    bus.vm_b_pu = 0.99; bus.va_b_deg = -120.5;
    bus.vm_c_pu = 1.01; bus.va_c_deg = 119.8;
    bus.pd_a_mw = 0.3; bus.qd_a_mvar = 0.1;
    bus.pd_b_mw = 0.4; bus.qd_b_mvar = 0.15;
    bus.pd_c_mw = 0.2; bus.qd_c_mvar = 0.08;
    bus.base_kv = 11.0;
    bus.in_service = true;

    CHECK(bus.index == 5);
    CHECK(bus.bus_type == BusType::PQ);
    CHECK(bus.vm_a_pu == Approx(1.02));
    CHECK(bus.va_b_deg == Approx(-120.5));
    CHECK(bus.pd_a_mw + bus.pd_b_mw + bus.pd_c_mw == Approx(0.9));
    CHECK(bus.base_kv == Approx(11.0));
}

TEST_CASE("ThreePhaseACSystem: line fields round-trip through struct", "[three_phase]") {
    ThreePhaseACLine line;
    line.index = 2; line.from_bus = 3; line.to_bus = 7;
    line.r1_pu = 0.05; line.x1_pu = 0.12;
    line.r0_pu = 0.15; line.x0_pu = 0.35;
    line.length_km = 2.5; line.in_service = true;

    CHECK(line.from_bus == 3);
    CHECK(line.to_bus == 7);
    CHECK(line.r1_pu == Approx(0.05));
    CHECK(line.x0_pu == Approx(0.35));
    CHECK(line.length_km == Approx(2.5));
}

TEST_CASE("ThreePhaseACSystem: load fields round-trip through struct", "[three_phase]") {
    ThreePhaseLoad load;
    load.index = 1; load.bus = 2;
    load.p_a_mw = 0.3; load.q_a_mvar = 0.1;
    load.p_b_mw = 0.5; load.q_b_mvar = 0.2;
    load.p_c_mw = 0.2; load.q_c_mvar = 0.08;
    load.connection = "wye"; load.grounded = true;
    load.const_p_percent = 100.0;

    CHECK(load.p_a_mw + load.p_b_mw + load.p_c_mw == Approx(1.0));
    CHECK(load.connection == "wye");
    CHECK(load.grounded);
}

TEST_CASE("ThreePhaseACSystem: generator fields round-trip through struct", "[three_phase]") {
    ThreePhaseGenerator gen;
    gen.index = 1; gen.bus = 1;
    gen.is_slack = true; gen.vm_pu = 1.02;
    gen.pmax_mw = 50.0; gen.qmax_mvar = 25.0; gen.qmin_mvar = -25.0;

    CHECK(gen.is_slack);
    CHECK(gen.vm_pu == Approx(1.02));
    CHECK(gen.pmax_mw == Approx(50.0));
}

TEST_CASE("ThreePhaseACSystem: system assembly and counts", "[three_phase]") {
    auto sys = build_balanced_3bus();

    CHECK(sys.buses.size() == 3);
    CHECK(sys.lines.size() == 2);
    CHECK(sys.generators.size() == 1);
    CHECK(sys.loads.empty());           // loads in bus pd fields, not loads table
    CHECK(sys.transformers.empty());
    CHECK(sys.name == "balanced_3bus");
    CHECK(sys.base_mva == Approx(10.0));
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 2 – Projection: ThreePhaseACSystem → HybridPowerSystem
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Three-phase projection: balanced 3-bus is converted and solves", "[three_phase]") {
    auto sys = as_hybrid(build_balanced_3bus());

    // Before solve the ac subsystem is empty; projection fills it
    CHECK(sys.three_phase_ac.has_value());
    CHECK(sys.ac.buses.empty());   // not yet projected

    auto r = solve_power_flow(sys);
    REQUIRE(r.converged);
    CHECK(r.vm.size() == 3);       // 3 projected buses
}

TEST_CASE("Three-phase projection: bus pd sums are correct", "[three_phase]") {
    // bus2: 0.5*3=1.5 MW total, bus3: 0.8*3=2.4 MW total
    auto tp = build_balanced_3bus();

    double pd2 = tp.buses[1].pd_a_mw + tp.buses[1].pd_b_mw + tp.buses[1].pd_c_mw;
    double pd3 = tp.buses[2].pd_a_mw + tp.buses[2].pd_b_mw + tp.buses[2].pd_c_mw;

    CHECK(pd2 == Approx(1.5));
    CHECK(pd3 == Approx(2.4));
}

TEST_CASE("Three-phase projection: projected single-phase equivalent converges", "[three_phase]") {
    HybridPowerSystem sys = as_hybrid(build_balanced_3bus());
    auto r = solve_power_flow(sys);

    REQUIRE(r.converged);
    // Slack bus (index 1) voltage magnitude must stay at 1.0 p.u.
    CHECK(r.vm[0] == Approx(1.0).margin(1e-6));
    // Load buses should have slightly lower voltage due to line drop
    CHECK(r.vm[1] < 1.0);
    CHECK(r.vm[2] < r.vm[1]);   // further from slack → more voltage drop
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 3 – Balanced radial feeder
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Three-phase balanced: voltage magnitudes are in [0.8, 1.1]", "[three_phase]") {
    // Heavy loading (3.9 MW on a 10 MVA base with large-impedance lines) means
    // the bus-3 voltage can fall below 0.9 pu — the solver should still converge.
    auto r = solve_power_flow(as_hybrid(build_balanced_3bus()));
    REQUIRE(r.converged);
    for (double vm : r.vm) {
        CHECK(vm > 0.8);
        CHECK(vm < 1.1);
    }
}

TEST_CASE("Three-phase balanced: angles are negative or zero down the feeder", "[three_phase]") {
    auto r = solve_power_flow(as_hybrid(build_balanced_3bus()));
    REQUIRE(r.converged);
    // slack va = 0, load buses have negative angle (lagging)
    CHECK(r.va[0] == Approx(0.0).margin(1e-6));
    CHECK(r.va[1] <= 0.0);
    CHECK(r.va[2] <= r.va[1]);
}

TEST_CASE("Three-phase balanced: iterations are reasonable (< 15)", "[three_phase]") {
    auto r = solve_power_flow(as_hybrid(build_balanced_3bus()));
    REQUIRE(r.converged);
    CHECK(r.iterations < 15);
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 4 – Unbalanced loads
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Three-phase unbalanced: asymmetric loads, still converges", "[three_phase]") {
    auto tp = build_balanced_3bus();
    // Make loads asymmetric on bus 2
    tp.buses[1].pd_a_mw = 0.3; tp.buses[1].qd_a_mvar = 0.15;
    tp.buses[1].pd_b_mw = 0.2; tp.buses[1].qd_b_mvar = 0.10;
    tp.buses[1].pd_c_mw = 0.1; tp.buses[1].qd_c_mvar = 0.05;
    // Make loads asymmetric on bus 3
    tp.buses[2].pd_a_mw = 0.4; tp.buses[2].qd_a_mvar = 0.20;
    tp.buses[2].pd_b_mw = 0.3; tp.buses[2].qd_b_mvar = 0.15;
    tp.buses[2].pd_c_mw = 0.2; tp.buses[2].qd_c_mvar = 0.10;

    auto r = solve_power_flow(as_hybrid(std::move(tp)));
    REQUIRE(r.converged);
}

TEST_CASE("Three-phase unbalanced: total load equals balanced version with same sum", "[three_phase]") {
    // Build a system where the sum of phases equals the balanced case
    // → projected single-phase solve should give the same result
    auto tp_bal = build_balanced_3bus();   // bus2: 1.5 MW, bus3: 2.4 MW
    auto r_bal = solve_power_flow(as_hybrid(tp_bal));

    auto tp_unbal = build_balanced_3bus();
    // Unbalanced but with the same per-bus totals
    tp_unbal.buses[1].pd_a_mw = 1.0; tp_unbal.buses[1].pd_b_mw = 0.3;
    tp_unbal.buses[1].pd_c_mw = 0.2;  // 1.0+0.3+0.2 = 1.5 MW
    tp_unbal.buses[1].qd_a_mvar = 0.6; tp_unbal.buses[1].qd_b_mvar = 0.2;
    tp_unbal.buses[1].qd_c_mvar = 0.1;  // 0.9 MVAr total
    tp_unbal.buses[2].pd_a_mw = 1.5; tp_unbal.buses[2].pd_b_mw = 0.6;
    tp_unbal.buses[2].pd_c_mw = 0.3;  // 2.4 MW total
    tp_unbal.buses[2].qd_a_mvar = 0.7; tp_unbal.buses[2].qd_b_mvar = 0.3;
    tp_unbal.buses[2].qd_c_mvar = 0.2; // 1.2 MVAr total

    auto r_unbal = solve_power_flow(as_hybrid(std::move(tp_unbal)));

    REQUIRE(r_bal.converged);
    REQUIRE(r_unbal.converged);
    // Projection sums phases → identical single-phase problem → identical result
    CHECK(r_bal.vm[1] == Approx(r_unbal.vm[1]).margin(1e-6));
    CHECK(r_bal.vm[2] == Approx(r_unbal.vm[2]).margin(1e-6));
}

TEST_CASE("Three-phase unbalanced: zero load on one phase, still converges", "[three_phase]") {
    auto tp = build_balanced_3bus();
    // Bus 3 phase C has no load
    tp.buses[2].pd_c_mw = 0.0; tp.buses[2].qd_c_mvar = 0.0;
    auto r = solve_power_flow(as_hybrid(std::move(tp)));
    REQUIRE(r.converged);
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 5 – Meshed topology
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Three-phase meshed: 3-bus ring (1-2-3-1) converges", "[three_phase]") {
    auto tp = build_balanced_3bus();
    // Add a back-tie 1→3 to create a ring
    ThreePhaseACLine l3;
    l3.index = 3; l3.from_bus = 1; l3.to_bus = 3;
    l3.r1_pu = 0.12; l3.x1_pu = 0.25;
    l3.r0_pu = 0.12; l3.x0_pu = 0.25;
    l3.in_service = true;
    tp.lines.push_back(l3);

    auto r = solve_power_flow(as_hybrid(std::move(tp)));
    REQUIRE(r.converged);
    CHECK(r.vm.size() == 3);
}

TEST_CASE("Three-phase meshed: ring topology improves voltage profile", "[three_phase]") {
    auto tp_radial = build_balanced_3bus();
    auto r_radial = solve_power_flow(as_hybrid(tp_radial));

    auto tp_ring = build_balanced_3bus();
    ThreePhaseACLine l3;
    l3.index = 3; l3.from_bus = 1; l3.to_bus = 3;
    l3.r1_pu = 0.12; l3.x1_pu = 0.25;
    l3.r0_pu = 0.12; l3.x0_pu = 0.25; l3.in_service = true;
    tp_ring.lines.push_back(l3);
    auto r_ring = solve_power_flow(as_hybrid(std::move(tp_ring)));

    REQUIRE(r_radial.converged);
    REQUIRE(r_ring.converged);
    // Ring provides parallel path → voltage at bus 3 should be higher (less drop)
    CHECK(r_ring.vm[2] > r_radial.vm[2]);
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 6 – Multi-source (distributed generation)
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Three-phase multi-source: DG at bus 3 reduces slack injection", "[three_phase]") {
    // Radial: slack only
    auto r_radial = solve_power_flow(as_hybrid(build_balanced_3bus()));
    REQUIRE(r_radial.converged);

    // Add a non-slack generator at bus 3 (PV bus)
    auto tp_dg = build_balanced_3bus();
    ThreePhaseGenerator dg;
    dg.index = 2; dg.bus = 3;
    dg.is_slack = false;
    dg.p_mw = 1.2;    // supplies half of bus-3 load
    dg.q_mvar = 0.0;
    dg.vm_pu = 1.0;
    dg.pmax_mw = 5.0; dg.pmin_mw = 0.0;
    dg.qmax_mvar = 2.0; dg.qmin_mvar = -2.0;
    dg.in_service = true;
    tp_dg.generators.push_back(dg);

    auto r_dg = solve_power_flow(as_hybrid(std::move(tp_dg)));
    REQUIRE(r_dg.converged);
    // With local DG, line 1→2→3 carries less power → less voltage drop
    CHECK(r_dg.vm[2] >= r_radial.vm[2]);
}

TEST_CASE("Three-phase multi-source: DG at every load bus converges", "[three_phase]") {
    auto tp = build_balanced_3bus();
    ThreePhaseGenerator dg2, dg3;
    dg2.index = 2; dg2.bus = 2; dg2.is_slack = false;
    dg2.p_mw = 0.5; dg2.q_mvar = 0.0;
    dg2.pmax_mw = 2.0; dg2.in_service = true;

    dg3.index = 3; dg3.bus = 3; dg3.is_slack = false;
    dg3.p_mw = 1.0; dg3.q_mvar = 0.0;
    dg3.pmax_mw = 3.0; dg3.in_service = true;

    tp.generators.push_back(dg2);
    tp.generators.push_back(dg3);

    auto r = solve_power_flow(as_hybrid(std::move(tp)));
    REQUIRE(r.converged);
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 7 – Transformer in three-phase system
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Three-phase transformer: 3-bus with transformer between buses 2-3 converges",
          "[three_phase]") {
    ThreePhaseACSystem tp;
    tp.base_mva = 10.0; tp.name = "3bus_trafo";

    ThreePhaseACBus b1, b2, b3;
    b1.index = 1; b1.bus_type = BusType::SLACK;
    b1.vm_a_pu = 1.0; b1.va_a_deg = 0.0;
    b1.vm_b_pu = 1.0; b1.va_b_deg = -120.0;
    b1.vm_c_pu = 1.0; b1.va_c_deg = 120.0;
    b1.base_kv = 11.0; b1.in_service = true;

    b2.index = 2; b2.bus_type = BusType::PQ;
    b2.pd_a_mw = 0.3; b2.qd_a_mvar = 0.1;
    b2.pd_b_mw = 0.3; b2.qd_b_mvar = 0.1;
    b2.pd_c_mw = 0.3; b2.qd_c_mvar = 0.1;
    b2.base_kv = 11.0; b2.in_service = true;

    b3.index = 3; b3.bus_type = BusType::PQ;
    b3.pd_a_mw = 0.5; b3.qd_a_mvar = 0.2;
    b3.pd_b_mw = 0.5; b3.qd_b_mvar = 0.2;
    b3.pd_c_mw = 0.5; b3.qd_c_mvar = 0.2;
    b3.base_kv = 0.4; b3.in_service = true;   // LV side of transformer

    tp.buses = {b1, b2, b3};

    // Line: 1→2
    ThreePhaseACLine l1;
    l1.index = 1; l1.from_bus = 1; l1.to_bus = 2;
    l1.r1_pu = 0.10; l1.x1_pu = 0.20;
    l1.r0_pu = 0.10; l1.x0_pu = 0.20;
    l1.in_service = true;
    tp.lines = {l1};

    // Transformer: 2→3 (11 kV → 0.4 kV, Dy connection)
    ThreePhaseTransformer tr;
    tr.index = 1; tr.hv_bus = 2; tr.lv_bus = 3;
    tr.sn_mva = 2.0; tr.vn_hv_kv = 11.0; tr.vn_lv_kv = 0.4;
    tr.vk_percent = 4.0; tr.vkr_percent = 1.2;
    tr.tap_pos = 0; tr.tap_neutral = 0; tr.tap_step_percent = 2.5;
    tr.in_service = true;
    tp.transformers = {tr};

    // Slack generator at bus 1
    ThreePhaseGenerator g;
    g.index = 1; g.bus = 1; g.is_slack = true;
    g.vm_pu = 1.0; g.pmax_mw = 100.0; g.in_service = true;
    tp.generators = {g};

    auto r = solve_power_flow(as_hybrid(std::move(tp)));
    REQUIRE(r.converged);
    CHECK(r.vm.size() == 3);
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 8 – Distribution PF stub
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("solve_distribution_power_flow: stub returns not_implemented", "[three_phase]") {
    HybridPowerSystem sys;
    sys.base_mva = 10.0;
    // Minimal single-bus system
    ACBus b; b.index = 1; b.bus_type = BusType::SLACK;
    b.vm_pu = 1.0; b.va_deg = 0.0; b.in_service = true;
    sys.ac.buses = {b};

    analysis::DistributionPFOptions opt;
    opt.max_iterations = 50;
    opt.convergence_tolerance = 1e-6;

    auto r = analysis::solve_distribution_power_flow(sys, opt);
    // Implementation is a stub — it should not crash, regardless of converged state
    CHECK(r.status == "not_implemented");
    CHECK_FALSE(r.converged);
}

TEST_CASE("solve_distribution_power_flow: default options compile and run", "[three_phase]") {
    HybridPowerSystem sys;
    auto r = analysis::solve_distribution_power_flow(sys);
    // Should not throw; stub returns immediately
    CHECK(r.iterations == 0);
}

// ══════════════════════════════════════════════════════════════════════════════
// Section 9 – JSON round-trip
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Three-phase JSON round-trip: bus count and topology preserved", "[three_phase]") {
    HybridPowerSystem orig = as_hybrid(build_balanced_3bus());

    std::string json_str = to_json(orig);
    REQUIRE_FALSE(json_str.empty());

    HybridPowerSystem restored = from_json(json_str);

    REQUIRE(restored.three_phase_ac.has_value());
    const auto& tp = *restored.three_phase_ac;
    CHECK(tp.buses.size() == 3);
    CHECK(tp.lines.size() == 2);
    CHECK(tp.generators.size() == 1);
    CHECK(tp.name == "balanced_3bus");
}

TEST_CASE("Three-phase JSON round-trip: bus voltage setpoints preserved", "[three_phase]") {
    HybridPowerSystem orig = as_hybrid(build_balanced_3bus());
    std::string json_str = to_json(orig);
    HybridPowerSystem restored = from_json(json_str);

    const auto& tp = *restored.three_phase_ac;
    CHECK(tp.buses[0].bus_type == BusType::SLACK);
    CHECK(tp.buses[0].vm_a_pu == Approx(1.0));
    CHECK(tp.buses[0].va_b_deg == Approx(-120.0));
    CHECK(tp.buses[0].va_c_deg == Approx(120.0));
}

TEST_CASE("Three-phase JSON round-trip: load values preserved", "[three_phase]") {
    HybridPowerSystem orig = as_hybrid(build_balanced_3bus());
    std::string json_str = to_json(orig);
    HybridPowerSystem restored = from_json(json_str);

    const auto& tp = *restored.three_phase_ac;
    CHECK(tp.buses[1].pd_a_mw == Approx(0.5));
    CHECK(tp.buses[1].qd_a_mvar == Approx(0.3));
    CHECK(tp.buses[2].pd_a_mw == Approx(0.8));
    CHECK(tp.buses[2].qd_c_mvar == Approx(0.4));
}

TEST_CASE("Three-phase JSON round-trip: line parameters preserved", "[three_phase]") {
    HybridPowerSystem orig = as_hybrid(build_balanced_3bus());
    std::string json_str = to_json(orig);
    HybridPowerSystem restored = from_json(json_str);

    const auto& tp = *restored.three_phase_ac;
    CHECK(tp.lines[0].r1_pu == Approx(0.10));
    CHECK(tp.lines[0].x1_pu == Approx(0.20));
    CHECK(tp.lines[1].r1_pu == Approx(0.08));
    CHECK(tp.lines[1].x1_pu == Approx(0.15));
}

TEST_CASE("Three-phase JSON round-trip: restored system solves identically", "[three_phase]") {
    HybridPowerSystem orig = as_hybrid(build_balanced_3bus());
    auto r_orig = solve_power_flow(orig);

    std::string json_str = to_json(orig);
    HybridPowerSystem restored = from_json(json_str);
    auto r_rest = solve_power_flow(restored);

    REQUIRE(r_orig.converged);
    REQUIRE(r_rest.converged);
    REQUIRE(r_orig.vm.size() == r_rest.vm.size());
    for (size_t i = 0; i < r_orig.vm.size(); ++i) {
        CHECK(r_orig.vm[i] == Approx(r_rest.vm[i]).margin(1e-8));
        CHECK(r_orig.va[i] == Approx(r_rest.va[i]).margin(1e-8));
    }
}
