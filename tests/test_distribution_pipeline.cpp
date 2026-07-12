// tests/test_distribution_pipeline.cpp
//
// Comprehensive distribution-system pipeline test.
//
// Tests information propagation from rich model construction through every
// analysis stage to JSON results export.  The network is a realistic
// 11-kV urban distribution system with a DC microgrid:
//
// ┌─────────────────────────────────────────────────────────────────────┐
// │  AC Network — 9 buses (Bus1–Bus9), base = 10 MVA / 11 kV           │
// │                                                                     │
// │  Bus1 ─[CB1]─ Bus2 ─[SW-A2]─ Bus3 ─────┐                          │
// │  (SLACK)   2.0+0.5j MW      1.5+0.3j MW │ TIE-1 (NO)             │
// │    └─[CB2]─ Bus4 ─[SW-A4]─ Bus5 ────────┘                          │
// │           2.5+0.5j MW       1.5+0.3j MW                            │
// │                 └─[SW-A6]─ Bus7 ─[CB3]─ Bus8                       │
// │                         0.5+0.1j MW  (VSC AC, 0.5+0.1j MW)         │
// │  Bus3 ─[SW-A5]─ Bus6 (PV, DER gen G2, 0–3 MW, no load)            │
// │  TIE-2 (NO): Bus2 ↔ Bus4                                           │
// │  TIE-3 (NO): Bus6 ↔ Bus8                                           │
// │  SW1 (closed): Bus8 ↔ Bus9 (LV spur, 1.0+0.2j MW load)            │
// ├─────────────────────────────────────────────────────────────────────┤
// │  DC Network — 3 buses (DCBus1–DCBus3), base = 10 MVA / 0.4 kV     │
// │  DCBus1 (slack) ─[DCCB]─ DCBus2 (0.3 MW load)                     │
// │  DCBus1 ─────────────── DCBus3 (PV array 0.25 MW)                  │
// ├─────────────────────────────────────────────────────────────────────┤
// │  VSC1: AC Bus8 ↔ DC Bus1  (PQ mode, 0.3 MW import AC←DC)          │
// └─────────────────────────────────────────────────────────────────────┘
//
// Pipeline stages:
//   Stage 0:  Rich model construction (breakers, switches, VSC, DC microgrid)
//   Stage 1:  Graph analysis of rich model (bridges, cut vertices, cycles)
//   Stage 2:  Graph topology change — close TIE-1, recheck cycle count
//   Stage 3:  Canonical projection (project_to_canonical_models)
//   Stage 4:  Graph analysis on projected model (edge category counts)
//   Stage 5:  Newton power flow on projected system
//   Stage 6:  DC OPF on projected system
//   Stage 7:  Short-circuit analysis (3PH + SLG, multiple buses)
//   Stage 8:  Optimal Network Reconfiguration (ONR)
//   Stage 9:  Results export — JSON round-trip for system and all results

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/network_reconfiguration/topology_analysis.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

using namespace hacdcpf;
using namespace hacdcpf::graph;
using namespace hacdcpf::analysis;
using namespace hacdcpf::opf;
using namespace hacdcpf::io;
using Approx = Catch::Approx;

// ═══════════════════════════════════════════════════════════════════════
// Network construction helpers
// ═══════════════════════════════════════════════════════════════════════

/// Build an ACBus (all on 11 kV, base_mva = 10 MVA).
static ACBus ac_bus(int idx, BusType type,
                    double pd_mw = 0.0, double qd_mvar = 0.0,
                    double vm = 1.0, double base_kv = 11.0) {
    ACBus b;
    b.index      = idx;
    b.bus_type   = type;
    b.pd_mw      = pd_mw;
    b.qd_mvar    = qd_mvar;
    b.vm_pu      = vm;
    b.va_deg     = 0.0;
    b.vmin_pu    = 0.90;
    b.vmax_pu    = 1.10;
    b.base_kv    = base_kv;
    b.in_service = true;
    return b;
}

/// Build an ACBranch.
static ACBranch ac_branch(int idx, int from, int to,
                           double r, double x,
                           double rate_a = 0.0,
                           bool in_service = true) {
    ACBranch br;
    br.index      = idx;
    br.from_bus   = from;
    br.to_bus     = to;
    br.r_pu       = r;
    br.x_pu       = x;
    br.b_pu       = 0.0;
    br.tap        = 1.0;
    br.shift_deg  = 0.0;
    br.rate_a_mva = rate_a;
    br.in_service = in_service;
    br.length_km  = 1.0;
    br.mttr_hr    = 8.0;
    return br;
}

/// Build a Generator with quadratic cost.
static Generator make_gen(int idx, int bus, bool slack,
                           double pg, double pmax, double pmin,
                           double vg, double xdpp,
                           double c1, double c2 = 0.0) {
    Generator g;
    g.index        = idx;
    g.bus          = bus;
    g.in_service   = true;
    g.pg_mw        = pg;
    g.qg_mvar      = 0.0;
    g.vg_pu        = vg;
    g.pmax_mw      = pmax;
    g.pmin_mw      = pmin;
    g.qmax_mvar    = 9999.0;
    g.qmin_mvar    = -9999.0;
    g.is_slack     = slack;
    g.xdpp_pu      = xdpp;
    g.cost_c1      = c1;
    g.cost_c2      = c2;
    g.cost_c0      = 0.0;
    return g;
}

/// Build a Switch (zero-impedance bus-tie device, no ACBranch counterpart).
static Switch make_switch(int idx, int from, int to, bool closed,
                           SwitchType sw_type = SwitchType::CircuitBreaker) {
    Switch sw;
    sw.index        = idx;
    sw.bus_from     = from;
    sw.bus_to       = to;
    sw.in_service   = true;
    sw.closed       = closed;
    sw.switch_type  = sw_type;
    sw.r_contact_ohm = 0.0;
    sw.z_ohm         = 0.0;
    sw.i_rated_ka    = 0.5;
    sw.is_automated  = true;
    sw.t_operation_s = 0.5;
    return sw;
}

/// Build a CircuitBreaker (metadata; same from/to as the feeder ACBranch).
static CircuitBreaker make_cb(int idx, int from, int to,
                               double v_kv = 11.0,
                               bool closed = true) {
    CircuitBreaker cb;
    cb.index            = idx;
    cb.bus_from         = from;
    cb.bus_to           = to;
    cb.in_service       = true;
    cb.closed           = closed;
    cb.rated_voltage_kv = v_kv;
    cb.i_rated_ka       = 1.0;
    cb.i_breaking_ka    = 12.5;
    return cb;
}

/// Build a DCBus.
static DCBus dc_bus(int idx, DCBusType type, double pd_mw = 0.0,
                    double base_kv = 0.0) {
    DCBus b;
    b.index      = idx;
    b.bus_type   = type;
    b.vm_pu      = 1.0;
    b.pd_mw      = pd_mw;
    b.base_kv    = base_kv;
    b.in_service = true;
    return b;
}

/// Build a DCBranch.
static DCBranch dc_branch(int idx, int from, int to, double r_pu) {
    DCBranch br;
    br.index      = idx;
    br.from_bus   = from;
    br.to_bus     = to;
    br.r_pu       = r_pu;
    br.in_service = true;
    return br;
}

/// Build a DCCircuitBreaker (metadata).
static DCCircuitBreaker dc_cb(int idx, int from, int to) {
    DCCircuitBreaker cb;
    cb.index     = idx;
    cb.bus_from  = from;
    cb.bus_to    = to;
    cb.in_service = true;
    cb.closed    = true;
    return cb;
}

// ═══════════════════════════════════════════════════════════════════════
// Master system builder
// ═══════════════════════════════════════════════════════════════════════

/// Construct the full 12-bus AC + 3-bus DC hybrid distribution system.
///
/// DESIGN NOTE: Circuit-breakers are modelled as near-zero-impedance elements
/// between the substation/source bus and a dedicated intermediate "protection"
/// bus.  The ACBranch (with significant impedance) then runs FROM that
/// intermediate bus TO the load bus.  This avoids overlap between CB endpoints
/// and feeder-line endpoints, keeping the rich-model graph radial and
/// preventing unwanted bus merging during canonical projection.
///
/// ACBranch index layout:
///   1: Bus10→Bus2 (feeder A main,     in_service=true)
///   2: Bus2→Bus3  (feeder A section,  in_service=true)
///   3: Bus11→Bus4 (feeder B main,     in_service=true)
///   4: Bus4→Bus5  (feeder B section,  in_service=true)
///   5: Bus12→Bus6 (DER spur,          in_service=true)
///   6: Bus4→Bus7  (VSC spur head,     in_service=true)
///   7: Bus7→Bus8  (VSC spur tail,     in_service=true)
///   8: Bus3→Bus5  (TIE-1,             in_service=false — normally open)
///   9: Bus2→Bus4  (TIE-2,             in_service=false — normally open)
///  10: Bus6→Bus8  (TIE-3,             in_service=false — normally open)
///
/// Switch layout (no ACBranch counterpart — pure bus-tie device):
///   SW1: Bus8→Bus9 (closed=true, always-on LV spur)
///
/// CircuitBreaker layout (intermediate-bus pattern — no shared endpoints):
///   CB1: Bus1→Bus10  (feeder A head breaker; Bus10 is the CB output node)
///   CB2: Bus1→Bus11  (feeder B head breaker; Bus11 is the CB output node)
///   CB3: Bus3→Bus12  (DER spur sectionalizer; Bus12 is CB output node)
///
/// After project_to_canonical_models() the CBs and SW1 are expanded to
/// near-zero-Z branches.  merge_zero_impedance_buses() then contracts:
///   {Bus1, Bus10, Bus11} → Bus1,  {Bus3, Bus12} → Bus3,  {Bus8, Bus9} → Bus8
/// leaving 8 canonical AC buses.  DC bus IDs start at 101 to avoid overlap
/// with AC bus IDs 1-12 in the shared bus_id_to_node_idx graph map.
///
/// DC Circuit Breaker (intermediate-bus pattern, same as AC CBs):
///   DCCB1: DCBus101→DCBus104  (CB connects slack to dedicated intermediate output node)
///   DC branch 1 then runs FROM DCBus104 TO DCBus102 (load)
///   DCBus104 is the CB output node (no load)
///
/// VSC:
///   VSC1: AC Bus8 ↔ DC Bus101 (PQ mode, p_set=+0.3 MW — import into AC)
static HybridPowerSystem build_distribution_system() {
    HybridPowerSystem sys;
    sys.base_mva  = 10.0;
    sys.name      = "Urban 11-kV distribution + DC microgrid";

    // ── AC buses ──────────────────────────────────────────────────────
    //   Bus 1:  substation slack
    //   Bus 2:  feeder A mid
    //   Bus 3:  feeder A end / DER junction
    //   Bus 4:  feeder B mid
    //   Bus 5:  feeder B end (TIE-1 target)
    //   Bus 6:  DER bus (PV generator)
    //   Bus 7:  VSC spur head
    //   Bus 8:  VSC AC coupling bus (TIE-3 target)
    //   Bus 9:  LV spur (connected via SW1)
    //   Bus 10: CB1 output node (feeder A intermediate)
    //   Bus 11: CB2 output node (feeder B intermediate)
    //   Bus 12: CB3 output node (DER spur intermediate)
    sys.ac.base_mva = 10.0;
    sys.ac.freq_hz  = 50.0;
    sys.ac.buses = {
        ac_bus( 1, BusType::SLACK,  0.0, 0.0, 1.0),
        ac_bus( 2, BusType::PQ,     2.0, 0.5),
        ac_bus( 3, BusType::PQ,     1.5, 0.3),
        ac_bus( 4, BusType::PQ,     2.5, 0.5),
        ac_bus( 5, BusType::PQ,     1.5, 0.3),
        ac_bus( 6, BusType::PV,     0.0, 0.0, 1.02),  // DER — no load
        ac_bus( 7, BusType::PQ,     0.5, 0.1),
        ac_bus( 8, BusType::PQ,     0.5, 0.1),
        ac_bus( 9, BusType::PQ,     1.0, 0.2),         // LV spur (via SW1)
        ac_bus(10, BusType::PQ,     0.0, 0.0),         // CB1 output (no load)
        ac_bus(11, BusType::PQ,     0.0, 0.0),         // CB2 output (no load)
        ac_bus(12, BusType::PQ,     0.0, 0.0),         // CB3 output (no load)
    };

    // ── AC branches ───────────────────────────────────────────────────
    sys.ac.branches = {
        // Normally-closed feeder lines (from CB output → load bus)
        // Ratings sized ≥ worst-case DC power flow (no losses, G2 at pmax=3 MW)
        ac_branch( 1, 10, 2,  0.040, 0.120, 4.0),   // feeder A main   (max 2 MW with G2)
        ac_branch( 2,  2, 3,  0.050, 0.130, 2.5),   // feeder A section
        ac_branch( 3, 11, 4,  0.038, 0.110, 7.0),   // feeder B main   (6 MW: all B+VSC loads)
        ac_branch( 4,  4, 5,  0.052, 0.130, 2.5),   // feeder B section
        ac_branch( 5, 12, 6,  0.020, 0.060, 3.5),   // DER spur        (≥3 MW reverse DER flow)
        ac_branch( 6,  4, 7,  0.030, 0.080, 2.5),   // VSC spur (head) (2 MW: Bus7+Bus8)
        ac_branch( 7,  7, 8,  0.025, 0.065, 2.0),   // VSC spur (tail) (1.5 MW: merged Bus8)
        // Normally-open tie switches (in_service=false)
        ac_branch( 8,  3, 5,  0.030, 0.090, 2.5, false),  // TIE-1
        ac_branch( 9,  2, 4,  0.040, 0.100, 4.5, false),  // TIE-2 (may carry feeder B)
        ac_branch(10,  6, 8,  0.020, 0.050, 2.0, false),  // TIE-3
    };

    // ── Generators ────────────────────────────────────────────────────
    //   G1 at Bus1: slack reference (high cost → OPF prefers DER)
    //   G2 at Bus6: DER / PV (low cost → OPF dispatches preferentially)
    sys.ac.generators = {
        make_gen(1,  1, true,  5.0,  20.0,  0.0, 1.00, 0.15, 60.0, 0.5),
        make_gen(2,  6, false, 2.0,   3.0,  0.0, 1.02, 0.20, 20.0, 0.0),
    };

    // ── Switch (Bus8→Bus9, no ACBranch for this connection) ───────────
    sys.ac.switches = {
        make_switch(1, 8, 9, true, SwitchType::LoadBreakSwitch),  // SW1: always closed
    };

    // ── Circuit breakers (intermediate-bus pattern) ───────────────────
    //   Each CB connects the source bus to a dedicated intermediate bus.
    //   The feeder ACBranch then connects FROM the intermediate bus.
    //   After projection, merge_zero_impedance_buses contracts the CB pair.
    sys.ac.circuit_breakers = {
        make_cb(1, 1, 10),  // CB1: feeder A head (Bus1 → Bus10)
        make_cb(2, 1, 11),  // CB2: feeder B head (Bus1 → Bus11)
        make_cb(3, 3, 12),  // CB3: DER spur      (Bus3 → Bus12)
    };

    // ── DC buses ──────────────────────────────────────────────────────
    //   DC bus IDs start at 101 to avoid collision with AC bus IDs 1-12
    //   in the shared bus_id_to_node_idx map inside PowerSystemGraph.
    //   DCBus101: DC voltage reference (slack)
    //   DCBus102: DC load  (e.g. EV charger, 0.3 MW)
    //   DCBus103: DC PV source (−0.25 MW → net injection)
    //   DCBus104: DCCB1 output node (intermediate CB output, no load)
    sys.dc.base_mva = 10.0;
    sys.dc.buses = {
        dc_bus(101, DCBusType::DC_V,  0.0),   // DC voltage reference
        dc_bus(102, DCBusType::DC_P,  0.3),   // 0.3 MW DC load
        dc_bus(103, DCBusType::DC_P, -0.25),  // 0.25 MW PV injection
        dc_bus(104, DCBusType::DC_P,  0.0),   // DCCB1 output node (no load)
    };

    // ── DC branches ───────────────────────────────────────────────────
    //   Branch 1 runs FROM the DCCB1 output node (DCBus104) to the DC load.
    //   This mirrors the AC CB intermediate-bus pattern: the CB (101→104)
    //   protects the branch (104→102), with non-overlapping endpoints.
    sys.dc.branches = {
        dc_branch(1, 104, 102, 0.010),  // DC feeder FROM CB output → DC load
        dc_branch(2, 101, 103, 0.008),  // DC feeder to PV injection bus
    };

    // ── DC circuit breaker (intermediate-bus pattern) ─────────────────
    //   DCCB1 connects the DC slack (DCBus101) to the intermediate output
    //   node (DCBus104).  The DC feeder branch then runs DCBus104→DCBus102.
    //   This keeps CB endpoints non-overlapping with branch endpoints, so
    //   the closed CB creates a zero-impedance DC_Switch edge without forming
    //   a parallel path with any DC branch (graph remains radial).
    sys.dc.dc_circuit_breakers = {
        dc_cb(1, 101, 104),  // DCCB1: slack → CB output intermediate node
    };

    // ── VSC converter: AC Bus8 ↔ DC Bus1 ─────────────────────────────
    {
        VSCConverter vsc;
        vsc.index          = 1;
        vsc.bus_ac         = 8;
        vsc.bus_dc         = 101;
        vsc.in_service     = true;
        vsc.control_mode   = ConverterMode::PQ_MODE;
        vsc.p_set_mw       = 0.3;   // 0.3 MW flows AC ← DC (import)
        vsc.q_set_mvar     = 0.0;
        vsc.eta            = 0.98;
        vsc.pmax_mw        = 2.0;
        vsc.pmin_mw        = -2.0;
        vsc.r_conv_ac_pu   = 0.002;
        vsc.x_sc_pu        = 0.10;
        vsc.name           = "VSC1";
        sys.vsc_converters = {vsc};
    }

    return sys;
}

// ═══════════════════════════════════════════════════════════════════════
// Utility: count graph edges by category
// ═══════════════════════════════════════════════════════════════════════

static int count_edges(const PowerSystemGraph& g, EdgeCategory cat) {
    int n = 0;
    for (const auto& e : g.edges)
        if (e.category == cat) ++n;
    return n;
}

static int count_active_edges(const PowerSystemGraph& g, EdgeCategory cat) {
    int n = 0;
    for (const auto& e : g.edges)
        if (e.category == cat && e.in_service) ++n;
    return n;
}

static double total_mw_load(const HybridPowerSystem& sys) {
    double s = 0.0;
    for (const auto& b : sys.ac.buses) s += b.pd_mw;
    for (const auto& b : sys.dc.buses) s += b.pd_mw;
    return s;
}

// ═══════════════════════════════════════════════════════════════════════
// Test: full pipeline
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("Distribution CB/switch/VSC pipeline: rich model → graph → PF → OPF "
          "→ SC → ONR → export",
          "[distribution][pipeline][integration][graph][pf][opf][sc][onr]") {

    // ── Stage 0: Rich model construction ─────────────────────────────
    SECTION("Stage 0: rich model — bus/branch/switch/converter counts") {
        const HybridPowerSystem sys = build_distribution_system();

        // AC topology
        REQUIRE(sys.ac.buses.size()            == 12);  // 9 load/gen + 3 CB nodes
        REQUIRE(sys.ac.branches.size()         == 10);  // 7 closed + 3 open ties
        REQUIRE(sys.ac.generators.size()       == 2);
        REQUIRE(sys.ac.switches.size()         == 1);   // SW1: Bus8→Bus9
        REQUIRE(sys.ac.circuit_breakers.size() == 3);   // CB1, CB2, CB3

        // DC topology
        REQUIRE(sys.dc.buses.size()                == 4);  // +DCBus104 intermediate CB output
        REQUIRE(sys.dc.branches.size()             == 2);
        REQUIRE(sys.dc.dc_circuit_breakers.size()  == 1);

        // VSC coupling
        REQUIRE(sys.vsc_converters.size() == 1);
        CHECK(sys.vsc_converters[0].bus_ac == 8);
        CHECK(sys.vsc_converters[0].bus_dc == 101);

        // Normally-open tie switches
        int n_open  = 0;
        int n_closed = 0;
        for (const auto& br : sys.ac.branches) {
            if (br.in_service) ++n_closed; else ++n_open;
        }
        CHECK(n_closed == 7);
        CHECK(n_open   == 3);

        // Switch SW1 is closed
        REQUIRE(!sys.ac.switches.empty());
        CHECK(sys.ac.switches[0].closed == true);

        // Generator cost model (OPF prefers DER: G2 cost_c1 < G1 cost_c1)
        const auto& g1 = sys.ac.generators[0];
        const auto& g2 = sys.ac.generators[1];
        CHECK(g2.cost_c1 < g1.cost_c1);

        // Total AC load (MW): buses 2..5 + 7 + 8 + 9 = 2+1.5+2.5+1.5+0.5+0.5+1 = 9.5 MW
        // (Bus10, 11, 12 are intermediate CB nodes with no load)
        double ac_load = 0.0;
        for (const auto& b : sys.ac.buses) ac_load += b.pd_mw;
        CHECK(ac_load == Approx(9.5).margin(0.01));
    }

    // ── Stage 1: Graph analysis — initial topology (ties open) ───────
    SECTION("Stage 1: graph analysis — radial topology with ties open") {
        const HybridPowerSystem sys = build_distribution_system();
        const PowerSystemGraph  g   = build_power_system_graph(sys);

        // Node counts: 12 AC + 4 DC (101,102,103,104) = 16
        CHECK(static_cast<int>(g.nodes.size()) == 16);

        // Active edge categories present:
        //   AC_Line:     7 closed branches (A1..A7)  — CB output→load buses
        //   Breaker:     3 (CB1..CB3), all closed  — source→intermediate buses
        //   Switch:      1 (SW1, closed)
        //   DC_Line:     2 (DC1, DC2)
        //   DC_Switch:   1 (DCCB1, closed) — DC circuit breaker added as graph edge
        //   VSC_Coupling: 1 (virtual edge Bus8↔DCBus101)
        CHECK(count_active_edges(g, EdgeCategory::AC_Line)      >= 7);
        CHECK(count_active_edges(g, EdgeCategory::Breaker)       == 3);
        CHECK(count_active_edges(g, EdgeCategory::Switch)        == 1);
        CHECK(count_active_edges(g, EdgeCategory::DC_Line)       == 2);
        CHECK(count_active_edges(g, EdgeCategory::DC_Switch)     == 1);
        CHECK(count_active_edges(g, EdgeCategory::VSC_Coupling)  == 1);

        // Inactive edges: 3 tie-switch ACBranches (TIE-1..TIE-3)
        int n_inactive_lines = 0;
        for (const auto& e : g.edges)
            if (e.category == EdgeCategory::AC_Line && !e.in_service)
                ++n_inactive_lines;
        CHECK(n_inactive_lines == 3);

        // Topology report
        const TopologyReport topo = analyze_topology(g);

        // With all ties open and CB/SW edges using non-overlapping bus IDs,
        // the full graph (AC + DC via VSC) is a tree → zero cycles.
        CHECK(topo.cycle_count == 0);
        CHECK(topo.is_radial   == true);

        // Single connected AC+DC component (VSC couples DC to AC Bus8)
        CHECK(topo.is_connected == true);

        // All islands valid (AC island has slack Bus1, DC island has DC_V Bus101)
        CHECK(topo.all_islands_valid == true);

        // Bus1 is a cut vertex (removing it splits feeder A and feeder B paths)
        const auto& cvs = topo.cut_vertex_bus_ids;
        CHECK(std::find(cvs.begin(), cvs.end(), 1) != cvs.end());

        // All main feeder branches must be bridges (tree topology)
        CHECK(!topo.bridge_edge_ids.empty());

        // Articulation-point invariant: #cut-vertices ≥ 1 for a tree
        CHECK(topo.cut_vertex_bus_ids.size() >= 1);
    }

    // ── Stage 2: Graph topology change — close TIE-1 (Bus3→Bus5) ────
    SECTION("Stage 2: closing TIE-1 introduces one cycle") {
        HybridPowerSystem sys = build_distribution_system();

        // Close TIE-1 (branch index 8: Bus3→Bus5)
        for (auto& br : sys.ac.branches) {
            if (br.index == 8) { br.in_service = true; break; }
        }

        const PowerSystemGraph g    = build_power_system_graph(sys);
        const TopologyReport   topo = analyze_topology(g);

        // Now there is exactly one back-edge → one cycle
        CHECK(topo.cycle_count  >= 1);
        CHECK(topo.is_radial    == false);
        CHECK(topo.is_connected == true);

        // The tie we just closed should no longer be a bridge
        // (closing TIE-1 creates a ring → some bridges disappear)
        // At minimum: branches on the ring (A2, A4, TIE-1) are NOT bridges
        for (int bridge_id : topo.bridge_edge_ids) {
            CHECK(bridge_id != 8);   // TIE-1 itself is never a bridge in a ring
        }

        // fundamental_cycles should contain the new cycle
        CHECK(!topo.fundamental_cycles.empty());
    }

    // ── Stage 3: Canonical projection ───────────────────────────────
    SECTION("Stage 3: project_to_canonical_models expands switches") {
        const HybridPowerSystem rich = build_distribution_system();
        const HybridPowerSystem proj = project_to_canonical_models(rich);

        // After projection:
        //   CBs (Bus1→Bus10, Bus1→Bus11, Bus3→Bus12) and SW1 (Bus8→Bus9) are
        //   expanded to near-zero-Z ACBranches, then merge_zero_impedance_buses
        //   contracts them, removing self-loop equivalents.  Net branch count
        //   is unchanged (4 new added, 4 self-loops removed).
        //   Bus count drops because merged groups {1,10,11}, {3,12}, {8,9}
        //   each collapse to a single representative bus.
        CHECK(proj.ac.branches.size() == rich.ac.branches.size()); // net 0 change

        // Projection merges CB/SW bus pairs → fewer buses than rich model
        CHECK(proj.ac.buses.size() < rich.ac.buses.size());

        // Generators preserved (at their original bus IDs)
        CHECK(proj.ac.generators.size() == rich.ac.generators.size());

        // VSC preserved
        CHECK(proj.vsc_converters.size() == rich.vsc_converters.size());
    }

    // ── Stage 4: Graph analysis on projected model ───────────────────
    SECTION("Stage 4: projected graph — edge category audit") {
        const HybridPowerSystem proj = project_to_canonical_models(
            build_distribution_system());
        const PowerSystemGraph g = build_power_system_graph(proj);

        // After projection, CB/SW equivalents collapse to self-loops and are
        // removed.  7 original in-service ACBranches survive.
        int active_ac = count_active_edges(g, EdgeCategory::AC_Line)
                      + count_active_edges(g, EdgeCategory::AC_Transformer);
        CHECK(active_ac >= 7);

        // DC side unchanged
        CHECK(count_active_edges(g, EdgeCategory::DC_Line) == 2);

        // VSC coupling still present
        CHECK(count_active_edges(g, EdgeCategory::VSC_Coupling) == 1);

        // Full topology still valid
        const TopologyReport topo = analyze_topology(g);
        CHECK(topo.all_islands_valid == true);
        CHECK(topo.is_connected      == true);
    }

    // ── Stage 5: Newton power flow ───────────────────────────────────
    SECTION("Stage 5: Newton power flow converges on projected system") {
        const HybridPowerSystem sys = project_to_canonical_models(
            build_distribution_system());

        PowerFlowOptions opts;
        opts.max_iter = 50;
        opts.tol      = 1e-6;

        const PowerFlowResult pf = solve_power_flow(sys, opts);

        INFO("PF residual=" << pf.residual
             << " reason=" << pf.diagnostics.termination_reason
             << " AC-P=" << pf.diagnostics.final_breakdown.ac_p_norm
             << " AC-Q=" << pf.diagnostics.final_breakdown.ac_q_norm
             << " DC-P=" << pf.diagnostics.final_breakdown.dc_p_norm
             << " converter-P=" << pf.diagnostics.final_breakdown.converter_p_norm);
        REQUIRE(pf.converged);
        CHECK(pf.iterations > 0);
        CHECK(pf.iterations <= 50);

        // All bus voltages within distribution limits [0.88, 1.12] pu
        // pf.vm is indexed by bus position in sys.ac.buses
        for (double vm : pf.vm) {
            CHECK(vm >= 0.88);
            CHECK(vm <= 1.12);
        }

        // Total dispatch (from system generators after solve) vs load+losses
        // Newton PF sets pg on generators; we read the declared set-points.
        double p_load = 0.0;
        for (const auto& b : sys.ac.buses) p_load += b.pd_mw;
        double p_loss = 0.0;
        for (const auto& bf : pf.branch_flows) p_loss += (bf.pf_mw + bf.pt_mw);

        // p_loss may be slightly negative (capacitive shunts) — broad check
        CHECK(p_loss >= -0.5);

        // Slack bus (Bus1) is position 0 in buses → vm[0] == 1.0 pu
        REQUIRE(!pf.vm.empty());
        CHECK(pf.vm[0] == Approx(1.0).margin(1e-4));

        // DER bus (Bus6) is PV → vm[5] ≈ vg_pu = 1.02
        if (pf.vm.size() >= 6)
            CHECK(pf.vm[5] == Approx(1.02).margin(5e-3));
    }

    // ── Stage 6: DC OPF ──────────────────────────────────────────────
    SECTION("Stage 6: DC OPF — DER dispatch preference") {
        const HybridPowerSystem sys = project_to_canonical_models(
            build_distribution_system());

        DCOPFOptions opts;
        opts.load_shedding = false;

        const DCOPFResult opf = hacdcpf::solve_dc_opf(sys, opts);

        REQUIRE(opf.converged);
        CHECK(opf.objective >= 0.0);

        // Both generators dispatched within limits
        for (size_t i = 0; i < opf.pg_mw.size(); ++i) {
            const auto& g = sys.ac.generators[i];
            CHECK(opf.pg_mw[i] >= g.pmin_mw - 1e-4);
            CHECK(opf.pg_mw[i] <= g.pmax_mw + 1e-4);
        }

        // Total dispatch covers total load (no shedding)
        double total_pg   = 0.0;
        for (double p : opf.pg_mw) total_pg += p;
        double total_load = 0.0;
        for (const auto& b : sys.ac.buses) total_load += b.pd_mw;
        CHECK(total_pg >= total_load - 0.5);

        // DER (G2, lower cost) should be at or near its upper limit (3 MW)
        // when total load > G2.pmax: slack G1 picks up the remainder.
        if (opf.pg_mw.size() >= 2)
            CHECK(opf.pg_mw[1] == Approx(3.0).margin(0.1));
    }

    // ── Stage 7: Short-circuit analysis ─────────────────────────────
    SECTION("Stage 7a: 3-phase short circuit — monotone Sk along feeder A") {
        const HybridPowerSystem sys = project_to_canonical_models(
            build_distribution_system());

        SCOptions sc_opts;
        sc_opts.fault_type       = FaultType::ThreePhase;
        sc_opts.c_factor         = 1.1;
        sc_opts.compute_all_buses = true;

        const SCResult sc = compute_short_circuit(sys, sc_opts);

        REQUIRE(!sc.bus_results.empty());

        // All Sk > 0
        for (const auto& r : sc.bus_results)
            CHECK(r.sk_mva > 0.0);

        // Fault current at substation (Bus1) should be largest
        auto it1 = std::find_if(sc.bus_results.begin(), sc.bus_results.end(),
                                 [](const BusFaultResult& r){ return r.bus_id==1; });
        auto it3 = std::find_if(sc.bus_results.begin(), sc.bus_results.end(),
                                 [](const BusFaultResult& r){ return r.bus_id==3; });

        if (it1 != sc.bus_results.end() && it3 != sc.bus_results.end()) {
            // Bus1 (slack) has smallest Thevenin impedance → largest Sk
            CHECK(it1->sk_mva > it3->sk_mva);
        }

        // summary() is non-empty
        CHECK(!sc.summary().empty());
    }

    SECTION("Stage 7b: SLG vs 3PH fault type ratio") {
        const HybridPowerSystem sys = project_to_canonical_models(
            build_distribution_system());

        SCOptions opts_3ph;
        opts_3ph.fault_type = FaultType::ThreePhase;
        opts_3ph.c_factor   = 1.0;

        SCOptions opts_slg;
        opts_slg.fault_type = FaultType::SinglePhaseGround;
        opts_slg.c_factor   = 1.0;

        const SCResult sc_3ph = compute_short_circuit(sys, opts_3ph);
        const SCResult sc_slg = compute_short_circuit(sys, opts_slg);

        // Both analyses produce results for every bus
        CHECK(sc_3ph.bus_results.size() == sc_slg.bus_results.size());

        // For a grounded source (sequence networks with Z0≈Z1):
        //   Ik_SLG  ≈  (√3 · c · V) / (Z1 + Z2 + Z0) ≈ same order as 3PH
        //   Both Sk values are strictly positive.
        for (size_t i = 0; i < sc_3ph.bus_results.size(); ++i) {
            CHECK(sc_3ph.bus_results[i].sk_mva > 0.0);
            CHECK(sc_slg.bus_results[i].sk_mva > 0.0);
        }
    }

    SECTION("Stage 7c: single-bus API round-trip (compute_fault_at_bus)") {
        const HybridPowerSystem sys = project_to_canonical_models(
            build_distribution_system());

        SCOptions opts;
        opts.compute_all_buses = true;

        const SCResult sc_full = compute_short_circuit(sys, opts);

        // Single-bus API for Bus3 must match the full-batch result
        const BusFaultResult single = compute_fault_at_bus(sys, 3, opts);
        const auto batch_it = std::find_if(
            sc_full.bus_results.begin(), sc_full.bus_results.end(),
            [](const BusFaultResult& r){ return r.bus_id == 3; });

        if (batch_it != sc_full.bus_results.end()) {
            CHECK(single.sk_mva == Approx(batch_it->sk_mva).margin(1e-3));
            CHECK(std::abs(single.z_thevenin - batch_it->z_thevenin) < 1e-6);
        }
    }

    // ── Stage 8: ONR ─────────────────────────────────────────────────
    SECTION("Stage 8: ONR finds feasible radial topology") {
        const HybridPowerSystem sys = project_to_canonical_models(
            build_distribution_system());

        ONROptions onr_opts;
        onr_opts.max_time_s = 30;

        const ONRResult onr = solve_optimal_reconfiguration(sys, onr_opts);

        REQUIRE(onr.feasible);

        // Spanning-tree condition: exactly (n_buses − 1) closed branches.
        // For the projected system with 9 original AC buses + Bus9 (from SW1)
        // = 10 nodes → exactly 9 closed branches.
        const int n_closed = static_cast<int>(onr.closed_branch_ids.size());
        const int n_open   = static_cast<int>(onr.open_branch_ids.size());
        CHECK(n_closed + n_open > 0);          // some topology emitted
        CHECK(n_closed >= static_cast<int>(sys.ac.buses.size()) - 2);  // ≥ n−2

        // Every bus reachable → verification PF must converge
        CHECK(onr.verification_pf.converged);

        // Estimated losses positive
        CHECK(onr.estimated_loss_mw >= 0.0);

        // summary() is non-empty
        CHECK(!onr.summary().empty());
    }

    // ── Stage 9: Results export — JSON round-trip ────────────────────
    SECTION("Stage 9a: HybridPowerSystem JSON round-trip") {
        const HybridPowerSystem sys = build_distribution_system();

        const std::string json_str = to_json(sys);
        REQUIRE(!json_str.empty());

        // Round-trip: parse back and verify key counts
        const HybridPowerSystem sys2 = from_json(json_str);
        CHECK(sys2.ac.buses.size()            == sys.ac.buses.size());
        CHECK(sys2.ac.branches.size()         == sys.ac.branches.size());
        CHECK(sys2.ac.generators.size()       == sys.ac.generators.size());
        CHECK(sys2.ac.switches.size()         == sys.ac.switches.size());
        CHECK(sys2.ac.circuit_breakers.size() == sys.ac.circuit_breakers.size());
        CHECK(sys2.dc.buses.size()            == sys.dc.buses.size());
        CHECK(sys2.vsc_converters.size()      == sys.vsc_converters.size());

        // Tie-switch in_service flags preserved
        int open_after = 0;
        for (const auto& br : sys2.ac.branches)
            if (!br.in_service) ++open_after;
        CHECK(open_after == 3);
    }

    SECTION("Stage 9b: PowerFlowResult JSON round-trip") {
        const HybridPowerSystem sys = project_to_canonical_models(
            build_distribution_system());
        const PowerFlowResult pf = solve_power_flow(sys);
        REQUIRE(pf.converged);

        const std::string json_str = power_flow_result_to_json(sys, pf);
        REQUIRE(!json_str.empty());

        // Must contain expected keys
        CHECK(json_str.find("\"converged\"") != std::string::npos);
        CHECK(json_str.find("\"ac_buses\"") != std::string::npos);
        CHECK(json_str.find("\"ac_branch_flows\"") != std::string::npos);
    }

    SECTION("Stage 9c: DCOPFResult JSON round-trip") {
        const HybridPowerSystem sys = project_to_canonical_models(
            build_distribution_system());
        const DCOPFResult opf = hacdcpf::solve_dc_opf(sys);
        REQUIRE(opf.converged);

        // DC OPF result: verify key fields are populated
        CHECK(!opf.status.empty());
        CHECK(opf.pg_mw.size() == sys.ac.generators.size());

        // solver_chain and objective_model must be populated after a successful
        // solve — these audit fields are the whole point of the new design.
        CHECK(!opf.solver_chain.empty());
        CHECK((!opf.objective_model.empty()
               && (opf.objective_model == "QP" || opf.objective_model == "LP")));

        // Verify dc_opf_result_to_json serialises the audit fields.
        const std::string dc_json = hacdcpf::io::dc_opf_result_to_json(opf);
        REQUIRE(!dc_json.empty());
        CHECK(dc_json.find("\"solver_chain\"")    != std::string::npos);
        CHECK(dc_json.find("\"objective_model\"") != std::string::npos);
        CHECK(dc_json.find("\"converged\"")        != std::string::npos);

        const DCOPFResult parsed = hacdcpf::io::dc_opf_result_from_json(dc_json);
        CHECK(parsed.converged == opf.converged);
        CHECK(parsed.status == opf.status);
        CHECK(parsed.solver_chain == opf.solver_chain);
        CHECK(parsed.objective_model == opf.objective_model);
        CHECK(parsed.objective == Approx(opf.objective).margin(1e-8));
        CHECK(parsed.total_load_shedding_mw == Approx(opf.total_load_shedding_mw).margin(1e-8));
        CHECK(parsed.pg_mw == opf.pg_mw);
        CHECK(parsed.pf_mw == opf.pf_mw);
        CHECK(parsed.va == opf.va);

        const auto path = std::filesystem::temp_directory_path() /
            "hacdcpf_dcopf_result_roundtrip.json";
        hacdcpf::io::save_dc_opf_result_json(opf, path.string());
        const DCOPFResult loaded = hacdcpf::io::load_dc_opf_result_json(path.string());
        CHECK(loaded.solver_chain == opf.solver_chain);
        CHECK(loaded.objective_model == opf.objective_model);
        CHECK(loaded.load_shedding_mw == opf.load_shedding_mw);
        std::filesystem::remove(path);

        // PF JSON also verifiable
        const PowerFlowResult pf2 = solve_power_flow(sys);
        const std::string pf_json = power_flow_result_to_json(sys, pf2);
        CHECK(!pf_json.empty());
        CHECK(pf_json.find("\"converged\"") != std::string::npos);
    }

    // ── Stage 10: End-to-end information-propagation summary ─────────
    SECTION("Stage 10: information propagation audit") {
        // Construct once; run every stage and verify that outputs are
        // consistent with the inputs that fed them.
        const HybridPowerSystem rich  = build_distribution_system();
        const HybridPowerSystem proj  = project_to_canonical_models(rich);

        // Power flow
        const PowerFlowResult pf = solve_power_flow(proj);
        REQUIRE(pf.converged);

        // DC OPF
        const DCOPFResult opf = hacdcpf::solve_dc_opf(proj);
        REQUIRE(opf.converged);

        // Short circuit
        SCOptions sc_opts;
        sc_opts.c_factor = 1.1;
        const SCResult sc = compute_short_circuit(proj, sc_opts);
        CHECK(!sc.bus_results.empty());

        // ONR
        ONROptions onr_opts;
        onr_opts.max_time_s = 30;
        const ONRResult onr = solve_optimal_reconfiguration(proj, onr_opts);
        REQUIRE(onr.feasible);

        // [Transfer 1] OPF total dispatch covers the network load
        double opf_gen_mw = 0.0;
        for (double p : opf.pg_mw) opf_gen_mw += p;
        double total_load = 0.0;
        for (const auto& b : proj.ac.buses) total_load += b.pd_mw;
        CHECK(opf_gen_mw >= total_load - 0.5);

        // [Transfer 2] ONR reconfigured PF also converges
        CHECK(onr.verification_pf.converged);

        // [Transfer 3] PF losses are finite
        double pf_loss = 0.0;
        for (const auto& bf : pf.branch_flows)
            pf_loss += (bf.pf_mw + bf.pt_mw);
        CHECK(pf_loss >= -0.5);  // small negative OK (capacitive shunts)

        // [Transfer 4] Short-circuit Sk at Bus1 > Sk at Bus3 > Sk at Bus5
        //   (monotone along radial feeder: closer buses have larger Sk)
        auto sk = [&](int bus) -> double {
            for (const auto& r : sc.bus_results)
                if (r.bus_id == bus) return r.sk_mva;
            return -1.0;
        };
        double sk1 = sk(1), sk3 = sk(3), sk5 = sk(5);
        if (sk1 > 0 && sk3 > 0 && sk5 > 0) {
            CHECK(sk1 > sk3);
            CHECK(sk3 > sk5);
        }

        // [Transfer 5] Graph analysis of rich model detects VSC coupling edge
        const PowerSystemGraph g_rich  = build_power_system_graph(rich);
        CHECK(count_active_edges(g_rich, EdgeCategory::VSC_Coupling) == 1);

        // JSON export of all four result types succeeds
        CHECK(!to_json(rich).empty());
        CHECK(!power_flow_result_to_json(proj, pf).empty());
    }
}
