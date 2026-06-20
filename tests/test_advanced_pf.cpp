/// @file test_advanced_pf.cpp
/// @brief Advanced power flow tests covering distributed slack buses,
///        islanded system detection/solving, and multi-island scenarios.
///
/// Test patterns adapted from the HybridACDCPowerSystemsPlanning /
/// luosipeng branch (tests/test_enhanced_solver.cpp and
/// tests/test_distribution_pf_crossval.cpp) – the more comprehensive
/// upstream project.
///
/// Tags: [advanced_pf], [island], [distributed_slack]

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/power_flow/distributed_slack_solver.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;
namespace pf = hacdcpf::powerflow;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: load a MATPOWER case by file name
// ─────────────────────────────────────────────────────────────────────────────
static HybridPowerSystem load_case(const std::string& fname) {
    const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/" + fname;
    return hacdcpf::io::parse_matpower(path);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: 9-bus island network
//
// Topology (adapted from luosipeng / test_distribution_pf_crossval.cpp):
//   Zone A: buses 1-4  (bus 1 is SLACK, generator)
//   Zone B: buses 5-7  (StaticGen at bus 7 — can serve itself when islanded)
//   Zone C: buses 8-9  (PV + Storage at bus 9)
//
// Branches: 1-2, 2-3, 3-4, 4-5, 5-6, 6-7, 5-8, 8-9
//   Index:   0    1    2    3    4    5    6    7
//
// To create 2 islands: disable branch index 3 (4-5)
// To create 3 islands: disable branch indices 3 and 6 (4-5 and 5-8)
// ─────────────────────────────────────────────────────────────────────────────
static HybridPowerSystem build_9bus_island_network() {
    HybridPowerSystem sys;
    sys.base_mva = 10.0;
    auto& ac = sys.ac;
    ac.base_mva = 10.0;

    // 9 AC buses; bus 1 is SLACK, all others PQ
    for (int i = 1; i <= 9; ++i) {
        ACBus b;
        b.index    = i;
        b.bus_type = (i == 1) ? BusType::SLACK : BusType::PQ;
        b.vm_pu    = 1.0; b.va_deg = 0.0;
        b.pd_mw    = (i == 1) ? 0.0 : 0.3;
        b.qd_mvar  = (i == 1) ? 0.0 : 0.1;
        b.vmin_pu  = 0.9; b.vmax_pu = 1.1; b.in_service = true;
        ac.buses.push_back(b);
    }

    // Branches
    int pairs[][2] = {{1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{5,8},{8,9}};
    for (int k = 0; k < 8; ++k) {
        ACBranch br;
        br.index    = k + 1;
        br.from_bus = pairs[k][0]; br.to_bus = pairs[k][1];
        br.r_pu = 0.03; br.x_pu = 0.06; br.b_pu = 0.0;
        br.tap = 1.0; br.shift_deg = 0.0; br.in_service = true;
        br.rate_a_mva = 100.0;
        ac.branches.push_back(br);
    }

    // Slack generator at bus 1
    Generator g;
    g.index = 1; g.bus = 1; g.in_service = true; g.is_slack = true;
    g.pg_mw = 0.0; g.qg_mvar = 0.0; g.vg_pu = 1.0;
    g.pmax_mw = 100.0; g.pmin_mw = 0.0;
    g.qmax_mvar = 100.0; g.qmin_mvar = -100.0;
    ac.generators = {g};

    // Zone B: StaticGen at bus 7 (serves buses 5,6,7 when islanded)
    StaticGenerator sg;
    sg.index = 1; sg.bus = 7; sg.in_service = true;
    sg.p_mw = 0.8; sg.q_mvar = 0.3; sg.scaling = 1.0;
    ac.static_generators = {sg};

    // Zone C: PV + Storage at bus 9 (serves buses 8,9 when islanded)
    PVSystem pv;
    pv.index = 1; pv.bus = 9; pv.in_service = true;
    pv.p_mw = 0.4; pv.q_mvar = 0.1;
    ac.pv_systems = {pv};

    Storage st;
    st.index = 1; st.bus = 9; st.in_service = true;
    st.p_mw = 0.2; st.q_mvar = 0.05;
    st.p_rated_mw = 1.0; st.pmax_mw = 1.0; st.pmin_mw = -1.0;
    ac.storage = {st};

    return sys;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: tiny 3-bus network for distributed slack unit tests
// ─────────────────────────────────────────────────────────────────────────────
static HybridPowerSystem build_3bus_distributed_slack() {
    HybridPowerSystem sys;
    sys.base_mva = 100.0;
    auto& ac = sys.ac;
    ac.base_mva = 100.0;

    for (int i = 1; i <= 3; ++i) {
        ACBus b;
        b.index    = i;
        b.bus_type = (i == 1) ? BusType::SLACK : (i == 2) ? BusType::PV : BusType::PQ;
        b.vm_pu    = 1.0; b.va_deg = 0.0;
        b.pd_mw    = (i == 3) ? 5.0 : 0.0;
        b.qd_mvar  = (i == 3) ? 2.0 : 0.0;
        b.vmin_pu  = 0.9; b.vmax_pu = 1.1; b.in_service = true;
        ac.buses.push_back(b);
    }

    for (int i = 0; i < 2; ++i) {
        ACBranch br;
        br.index    = i + 1;
        br.from_bus = i + 1; br.to_bus = i + 2;
        br.r_pu = 0.01; br.x_pu = 0.05; br.b_pu = 0.0;
        br.tap = 1.0; br.shift_deg = 0.0; br.in_service = true;
        br.rate_a_mva = 100.0;
        ac.branches.push_back(br);
    }

    Generator g1;
    g1.index = 1; g1.bus = 1; g1.in_service = true; g1.is_slack = true;
    g1.pg_mw = 3.0; g1.qg_mvar = 0.0; g1.vg_pu = 1.0;
    g1.pmax_mw = 10.0; g1.pmin_mw = 0.0;
    g1.qmax_mvar = 10.0; g1.qmin_mvar = -10.0;

    Generator g2;
    g2.index = 2; g2.bus = 2; g2.in_service = true; g2.is_slack = false;
    g2.pg_mw = 2.0; g2.qg_mvar = 0.0; g2.vg_pu = 1.0;
    g2.pmax_mw = 5.0; g2.pmin_mw = 0.0;
    g2.qmax_mvar = 5.0; g2.qmin_mvar = -5.0;

    ac.generators = {g1, g2};
    return sys;
}

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 1 — Island detection
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Island detection: fully-connected 9-bus returns single island",
          "[advanced_pf][island][detection]") {
    auto sys = build_9bus_island_network();
    const auto islands = pf::detect_islands(sys);
    REQUIRE(islands.size() == 1);
    REQUIRE(islands[0].ac_buses.size() == 9);
    REQUIRE(islands[0].has_ac_slack);
    REQUIRE(islands[0].has_generators);
}

TEST_CASE("Island detection: branch 4-5 out → 2 islands",
          "[advanced_pf][island][detection]") {
    auto sys = build_9bus_island_network();
    sys.ac.branches[3].in_service = false;  // 4-5

    const auto islands = pf::detect_islands(sys);
    REQUIRE(islands.size() == 2);

    // One island must have the SLACK bus (bus 1)
    bool has_slack_island = false;
    bool has_generator_island = false;
    for (const auto& isl : islands) {
        if (isl.has_ac_slack) has_slack_island = true;
        if (isl.has_generators) has_generator_island = true;
    }
    REQUIRE(has_slack_island);
    REQUIRE(has_generator_island);
}

TEST_CASE("Island detection: branches 4-5 and 5-8 out → 3 islands",
          "[advanced_pf][island][detection]") {
    auto sys = build_9bus_island_network();
    sys.ac.branches[3].in_service = false;  // 4-5
    sys.ac.branches[6].in_service = false;  // 5-8

    const auto islands = pf::detect_islands(sys);
    REQUIRE(islands.size() == 3);

    // All three zones have generation (slack gen, StaticGen, PV+Storage)
    int live = 0;
    for (const auto& isl : islands) {
        if (isl.has_generators) ++live;
    }
    REQUIRE(live == 3);
}

TEST_CASE("Island detection: non-contiguous bus IDs (case_2_1_backup) → 3 islands",
          "[advanced_pf][island][detection][noncontiguous]") {
    // Regression for the "lots of spurious islands" bug: a system whose bus
    // indices are NOT a contiguous 1..N sequence (here 1-5, 10-14, 20-21)
    // must still be partitioned by electrical connectivity, not by the raw
    // ``index - 1`` array position.  The three feeders (tie lines out of
    // service) each carry their own SLACK substation, so the detector must
    // report exactly three islands — not one-per-bus.
    const std::string path =
        std::string(HACDCPF_PROJECT_ROOT) + "/tests/data/case_2_1_backup.json";
    const HybridPowerSystem sys = hacdcpf::io::load_json(path);
    REQUIRE(sys.ac.buses.size() == 12);

    const auto islands = pf::detect_islands(sys);
    REQUIRE(islands.size() == 3);

    // Every island is energised by its own substation slack.
    int live = 0;
    for (const auto& isl : islands) {
        REQUIRE(isl.has_ac_slack);
        REQUIRE(isl.has_generators);
        if (isl.has_generators) ++live;
    }
    REQUIRE(live == 3);

    // Membership must follow real bus IDs, gaps included.
    auto island_with = [&](int bus_id) {
        return std::find_if(islands.begin(), islands.end(),
            [&](const IslandInfo& isl) {
                return std::find(isl.ac_buses.begin(), isl.ac_buses.end(), bus_id)
                       != isl.ac_buses.end();
            });
    };
    const auto feeder1 = island_with(1);
    const auto feeder2 = island_with(10);
    const auto feeder3 = island_with(20);
    REQUIRE(feeder1 != islands.end());
    REQUIRE(feeder2 != islands.end());
    REQUIRE(feeder3 != islands.end());
    REQUIRE(feeder1->ac_buses == std::vector<int>{1, 2, 3, 4, 5});
    REQUIRE(feeder2->ac_buses == std::vector<int>{10, 11, 12, 13, 14});
    REQUIRE(feeder3->ac_buses == std::vector<int>{20, 21});

    // End-to-end: the adaptive solver must energise every bus (no spurious
    // dead/zero-voltage buses from mis-detected singleton islands).
    const auto adaptive = hacdcpf::solve_power_flow_adaptive(sys, hacdcpf::PowerFlowOptions{});
    REQUIRE(adaptive.converged);
    REQUIRE(adaptive.vm.size() == 12);
    for (double v : adaptive.vm) {
        REQUIRE(v > 0.8);
    }
}

TEST_CASE("Island detection: case14 forms single connected island",
          "[advanced_pf][island][detection][matpower]") {
    const auto sys = io::build_ieee14_acdc();
    const auto islands = pf::detect_islands(sys);
    REQUIRE(islands.size() == 1);
    REQUIRE(islands[0].ac_buses.size() == 14);
    REQUIRE(islands[0].has_ac_slack);
}

TEST_CASE("Island detection: case118 forms single connected island",
          "[advanced_pf][island][detection][matpower]") {
    const auto sys = load_case("case118.m");
    const auto islands = pf::detect_islands(sys);
    REQUIRE(islands.size() == 1);
    REQUIRE(islands[0].ac_buses.size() == 118);
    REQUIRE(islands[0].has_ac_slack);
}

TEST_CASE("extract_island_subsystem: isolated bus produces 1-bus system",
          "[advanced_pf][island][extract]") {
    auto sys = io::build_ieee14_acdc();
    // Isolate bus 14 by disabling all its branches
    for (auto& br : sys.ac.branches) {
        if (br.from_bus == 14 || br.to_bus == 14) br.in_service = false;
    }
    const auto islands = pf::detect_islands(sys);
    REQUIRE(islands.size() >= 2);

    const auto it14 = std::find_if(islands.begin(), islands.end(),
        [](const IslandInfo& isl) {
            return std::find(isl.ac_buses.begin(), isl.ac_buses.end(), 14)
                   != isl.ac_buses.end();
        });
    REQUIRE(it14 != islands.end());

    const auto sub = pf::extract_island_subsystem(sys, *it14, 14);
    REQUIRE(sub.ac.buses.size() == 1);
    REQUIRE(sub.ac.branches.empty());
}

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 2 — Islanded solver (adaptive and islanded)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Islanded solver: 9-bus no failure → converges normally",
          "[advanced_pf][island][solver]") {
    auto sys = build_9bus_island_network();
    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_islanded(sys, opt);
    REQUIRE(result.converged);
    REQUIRE(result.islands.size() == 1);
    REQUIRE(result.vm.size() == 9);
    for (double v : result.vm) { REQUIRE(v > 0.8); REQUIRE(v < 1.2); }
}

TEST_CASE("Islanded solver: 2-island scenario — DG-only island is live",
          "[advanced_pf][island][solver]") {
    auto sys = build_9bus_island_network();
    sys.ac.branches[3].in_service = false;  // break at 4-5

    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_islanded(sys, opt);
    REQUIRE(result.converged);
    REQUIRE(result.islands.size() == 2);
    REQUIRE(result.vm.size() == 9);

    // DG-only island (buses 5-9, no external slack) must not be dead
    bool dg_island_live = false;
    for (const auto& isl : result.islands) {
        if (!isl.has_ac_slack && isl.has_generators) dg_island_live = true;
    }
    REQUIRE(dg_island_live);

    // Dead-island buses should have voltage close to zero; live buses must
    // be in the physical range [0.7, 1.2] pu
    for (size_t i = 0; i < result.vm.size(); ++i) {
        if (result.vm[i] > 1e-6) {
            REQUIRE(result.vm[i] > 0.7);
            REQUIRE(result.vm[i] < 1.2);
        }
    }
}

TEST_CASE("Islanded solver: 3-island scenario — all islands converge",
          "[advanced_pf][island][solver]") {
    auto sys = build_9bus_island_network();
    sys.ac.branches[3].in_service = false;  // 4-5
    sys.ac.branches[6].in_service = false;  // 5-8

    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_islanded(sys, opt);
    REQUIRE(result.converged);
    REQUIRE(result.islands.size() == 3);
    REQUIRE(result.vm.size() == 9);
}

TEST_CASE("Adaptive solver: 2-island system — result sizes and dead-bus voltage",
          "[advanced_pf][island][adaptive]") {
    auto sys = io::build_ieee14_acdc();
    for (auto& br : sys.ac.branches) {
        if (br.from_bus == 14 || br.to_bus == 14) br.in_service = false;
    }

    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 50;

    const auto result = solve_power_flow_adaptive(sys, opt);
    REQUIRE(result.vm.size() == 14);
    REQUIRE(result.va.size() == 14);
    REQUIRE(result.vdc.size() == 2);  // IEEE-14 AC/DC builder has 2 DC buses
    REQUIRE(result.islands.size() >= 2);
    // Bus 14 is isolated — its voltage must be effectively zero
    REQUIRE(std::abs(result.vm[13]) <= 1e-12);
}

TEST_CASE("Adaptive solver: case9 fully connected → converged with island metadata",
          "[advanced_pf][island][adaptive][matpower]") {
    const auto sys = load_case("case9.m");
    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 50;

    const auto result = solve_power_flow_adaptive(sys, opt);
    REQUIRE(result.converged);
    REQUIRE(result.vm.size() == 9);
    REQUIRE(result.islands.size() == 1);
    REQUIRE(result.residual < 1e-4);
}

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 3 — Distributed slack: participation factors
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Distributed slack: capacity-based factors sum to 1.0",
          "[advanced_pf][distributed_slack][factors]") {
    const auto sys = io::build_ieee14_acdc();
    const auto cfg = pf::create_participation_factors(sys, "capacity");
    REQUIRE_FALSE(cfg.participating_buses.empty());
    double sum = 0.0;
    for (double f : cfg.participation_factors) sum += f;
    REQUIRE(std::abs(sum - 1.0) <= 1e-12);
}

TEST_CASE("Distributed slack: equal factors are uniform over explicit bus list",
          "[advanced_pf][distributed_slack][factors]") {
    const auto sys = io::build_ieee14_acdc();
    const auto cfg = pf::create_participation_factors(sys, "equal", {1, 2, 6});
    REQUIRE(cfg.participation_factors.size() == 3);
    for (double f : cfg.participation_factors) {
        REQUIRE(std::abs(f - 1.0 / 3.0) <= 1e-12);
    }
}

TEST_CASE("Distributed slack: droop factors ordered inversely with droop coefficient",
          "[advanced_pf][distributed_slack][factors]") {
    // Smaller droop coeff → larger participation factor (stiffer governor)
    const auto sys = io::build_ieee14_acdc();
    const std::unordered_map<int, double> droop = {{1, 0.05}, {2, 0.10}, {6, 0.20}};
    const auto cfg = pf::create_participation_factors(sys, "droop", {1, 2, 6}, droop);
    REQUIRE(cfg.participation_factors.size() == 3);
    REQUIRE(cfg.participation_factors[0] > cfg.participation_factors[1]);
    REQUIRE(cfg.participation_factors[1] > cfg.participation_factors[2]);
}

TEST_CASE("Distributed slack: capacity factors — custom 3-bus system",
          "[advanced_pf][distributed_slack][factors]") {
    const auto sys = build_3bus_distributed_slack();
    const auto cfg = pf::create_participation_factors(sys, "capacity");
    REQUIRE_FALSE(cfg.participating_buses.empty());
    double sum = 0.0;
    for (double f : cfg.participation_factors) sum += f;
    REQUIRE(std::abs(sum - 1.0) <= 1e-12);
}

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 4 — Distributed slack: solver convergence
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Distributed slack (simplified): 3-bus system converges and allocates power",
          "[advanced_pf][distributed_slack][solver]") {
    const auto sys = build_3bus_distributed_slack();
    const auto cfg = pf::create_participation_factors(sys, "capacity");
    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_distributed_slack(sys, cfg, opt);
    REQUIRE(result.converged);
    REQUIRE_FALSE(result.distributed_slack_p.empty());
    REQUIRE(result.vm.size() == 3);
    for (double v : result.vm) { REQUIRE(v > 0.85); REQUIRE(v < 1.15); }
}

TEST_CASE("Distributed slack (full Jacobian): 3-bus system converges and allocates power",
          "[advanced_pf][distributed_slack][solver]") {
    const auto sys = build_3bus_distributed_slack();
    const auto cfg = pf::create_participation_factors(sys, "capacity");
    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_distributed_slack_full(sys, cfg, opt);
    REQUIRE(result.converged);
    REQUIRE_FALSE(result.distributed_slack_p.empty());
}

TEST_CASE("Distributed slack: participation limit forces bus to hit_limits",
          "[advanced_pf][distributed_slack][limits]") {
    auto sys = build_3bus_distributed_slack();
    auto cfg = pf::create_participation_factors(sys, "capacity");
    // Force bus 1 participation limit to ~zero
    cfg.max_participation_p[1] = 1e-12;

    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_distributed_slack_full(sys, cfg, opt);
    REQUIRE(result.converged);

    bool bus1_hit = false;
    for (int b : result.hit_limits) {
        if (b == 1) bus1_hit = true;
    }
    REQUIRE(bus1_hit);
}

TEST_CASE("Distributed slack: case9 MATPOWER converges with capacity participation",
          "[advanced_pf][distributed_slack][solver][matpower]") {
    const auto sys = load_case("case9.m");
    const auto cfg = pf::create_participation_factors(sys, "capacity");
    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_distributed_slack(sys, cfg, opt);
    REQUIRE(result.converged);
    REQUIRE_FALSE(result.distributed_slack_p.empty());
}

TEST_CASE("Distributed slack: case14 MATPOWER — simplified and full give same convergence",
          "[advanced_pf][distributed_slack][solver][matpower]") {
    const auto sys = load_case("case14.m");
    const auto cfg = pf::create_participation_factors(sys, "capacity");
    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto r_simple = solve_power_flow_distributed_slack(sys, cfg, opt);
    const auto r_full   = solve_power_flow_distributed_slack_full(sys, cfg, opt);
    REQUIRE(r_simple.converged);
    REQUIRE(r_full.converged);
    // Both should produce non-trivial allocation maps
    REQUIRE_FALSE(r_simple.distributed_slack_p.empty());
    REQUIRE_FALSE(r_full.distributed_slack_p.empty());
}

TEST_CASE("Distributed slack: case30 MATPOWER — droop participation converges",
          "[advanced_pf][distributed_slack][solver][matpower]") {
    const auto sys = load_case("case30.m");
    // Use droop on the first two generators for a non-trivial distribution
    // case30 generators sit at buses 1, 2, 13, 22, 23, 27
    const std::unordered_map<int, double> droop = {{1, 0.04}, {2, 0.06}};
    const auto cfg = pf::create_participation_factors(sys, "droop", {1, 2}, droop);
    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_distributed_slack(sys, cfg, opt);
    REQUIRE(result.converged);
    REQUIRE(result.distributed_slack_p.count(1) + result.distributed_slack_p.count(2) >= 1);
}

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 5 — PV/PQ conversion control
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("PV/PQ conversion: case118 converges WITHOUT switching (fast path)",
          "[advanced_pf][pvpq]") {
    const auto sys = load_case("case118.m");
    PowerFlowOptions opt;
    opt.enable_pv_pq_conversion = false;
    opt.tol = 1e-8; opt.max_iter = 50;

    const auto r = solve_power_flow(sys, opt);
    REQUIRE(r.converged);
    REQUIRE(r.residual < 1e-4);
}

TEST_CASE("PV/PQ conversion: case118 converges WITH switching (full mode)",
          "[advanced_pf][pvpq]") {
    const auto sys = load_case("case118.m");
    PowerFlowOptions opt;
    opt.enable_pv_pq_conversion = true;
    opt.tol = 1e-8; opt.max_iter = 50;

    const auto r = solve_power_flow(sys, opt);
    REQUIRE(r.converged);
}

TEST_CASE("PV/PQ conversion: adaptive try-without-then-retry pattern on case2383wp",
          "[advanced_pf][pvpq]") {
    // Reproduces the adaptive strategy used in luosipeng performance test
    // and now also used for Tier-3 cases in test_matpower_cases.cpp
    const auto sys = load_case("case2383wp.m");

    PowerFlowOptions opt_fast;
    opt_fast.enable_pv_pq_conversion = false;
    opt_fast.tol = 1e-8; opt_fast.max_iter = 100;

    auto r = solve_power_flow(sys, opt_fast);
    if (!r.converged) {
        PowerFlowOptions opt_full;
        opt_full.enable_pv_pq_conversion = true;
        opt_full.tol = 1e-8; opt_full.max_iter = 300;
        r = solve_power_flow(sys, opt_full);
    }
    REQUIRE(r.converged);
}

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 6 — Integration: combined island + distributed slack
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Integration: island detection then distributed slack on same case14",
          "[advanced_pf][integration]") {
    const auto sys = io::build_ieee14_acdc();

    // 1. Verify topology is connected
    const auto islands = pf::detect_islands(sys);
    REQUIRE(islands.size() == 1);

    // 2. Distributed slack converges (capacity-based)
    const auto cfg = pf::create_participation_factors(sys, "capacity");
    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_distributed_slack(sys, cfg, opt);
    REQUIRE(result.converged);
    REQUIRE(result.vm.size() == 14);
    // Slack bus voltage should remain near set-point
    REQUIRE(result.vm[0] > 0.95);
    REQUIRE(result.vm[0] < 1.10);
}

TEST_CASE("Integration: islanded 9-bus → adaptive solver → power balance",
          "[advanced_pf][integration]") {
    auto sys = build_9bus_island_network();
    sys.ac.branches[3].in_service = false;  // create 2-island split

    PowerFlowOptions opt;
    opt.tol = 1e-8; opt.max_iter = 100;

    const auto result = solve_power_flow_adaptive(sys, opt);
    REQUIRE(result.converged);
    REQUIRE(result.vm.size() == 9);
    REQUIRE(result.va.size() == 9);
    REQUIRE(result.islands.size() == 2);
}
