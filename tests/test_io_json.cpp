/// @file test_io_json.cpp
/// @brief Comprehensive I/O tests: JSON round-trip, MATPOWER parsing,
///        exception-free try_ variants, OPF result serialisation, and
///        schema-version compatibility.
///
/// Tags: [io], [json], [matpower], [roundtrip], [safe_io]

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/validation/validate_system.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

namespace fs = std::filesystem;
using namespace hacdcpf;
using namespace hacdcpf::io;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::ContainsSubstring;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static std::string data_path(const std::string& name) {
    return std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
}

static HybridPowerSystem load_mp(const std::string& name) {
    return parse_matpower(data_path(name));
}

/// Build a minimal 2-bus AC system for JSON tests.
static HybridPowerSystem make_2bus() {
    HybridPowerSystem sys;
    sys.name     = "json_test_2bus";
    sys.base_mva = 100.0;

    ACBus b1; b1.index=1; b1.bus_type=BusType::SLACK; b1.vm_pu=1.04;
    b1.vmin_pu=0.9; b1.vmax_pu=1.1; b1.in_service=true;
    ACBus b2; b2.index=2; b2.bus_type=BusType::PQ; b2.vm_pu=1.0;
    b2.pd_mw=40.0; b2.qd_mvar=15.0; b2.vmin_pu=0.9; b2.vmax_pu=1.1; b2.in_service=true;
    sys.ac.buses = {b1, b2};

    ACBranch br; br.from_bus=1; br.to_bus=2; br.r_pu=0.01; br.x_pu=0.04;
    br.b_pu=0.0; br.tap=1.0; br.in_service=true;
    sys.ac.branches = {br};

    Generator g; g.bus=1; g.is_slack=true; g.pmax_mw=200.0; g.pmin_mw=0.0;
    g.qmax_mvar=100.0; g.qmin_mvar=-100.0; g.vg_pu=1.04; g.in_service=true;
    sys.ac.generators = {g};

    return sys;
}

// ═════════════════════════════════════════════════════════════════════════════
// JSON round-trip: simple 2-bus system
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("JSON round-trip: 2-bus system serialises and deserialises", "[io][json][roundtrip]") {
    auto orig = make_2bus();
    const std::string json_str = to_json(orig);
    REQUIRE_FALSE(json_str.empty());

    auto restored = from_json(json_str);

    CHECK(restored.name == orig.name);
    CHECK_THAT(restored.base_mva, WithinAbs(orig.base_mva, 1e-9));
    CHECK(restored.ac.buses.size()      == orig.ac.buses.size());
    CHECK(restored.ac.branches.size()   == orig.ac.branches.size());
    CHECK(restored.ac.generators.size() == orig.ac.generators.size());

    // Bus-level fidelity
    CHECK(restored.ac.buses[0].index    == orig.ac.buses[0].index);
    CHECK(restored.ac.buses[0].bus_type == orig.ac.buses[0].bus_type);
    CHECK_THAT(restored.ac.buses[0].vm_pu, WithinAbs(orig.ac.buses[0].vm_pu, 1e-9));
    CHECK_THAT(restored.ac.buses[1].pd_mw, WithinAbs(40.0, 1e-9));
    CHECK_THAT(restored.ac.buses[1].qd_mvar, WithinAbs(15.0, 1e-9));

    // Branch fidelity
    CHECK(restored.ac.branches[0].from_bus == orig.ac.branches[0].from_bus);
    CHECK(restored.ac.branches[0].to_bus   == orig.ac.branches[0].to_bus);
    CHECK_THAT(restored.ac.branches[0].r_pu, WithinAbs(0.01, 1e-9));
    CHECK_THAT(restored.ac.branches[0].x_pu, WithinAbs(0.04, 1e-9));
}

TEST_CASE("JSON round-trip: IEEE-14 AC/DC case preserves topology", "[io][json][roundtrip]") {
    auto orig = build_ieee14_acdc();
    const std::string json_str = to_json(orig);
    auto restored = from_json(json_str);

    CHECK(restored.ac.buses.size()        == orig.ac.buses.size());
    CHECK(restored.ac.branches.size()     == orig.ac.branches.size());
    CHECK(restored.ac.generators.size()   == orig.ac.generators.size());
    CHECK(restored.dc.buses.size()        == orig.dc.buses.size());
    CHECK(restored.vsc_converters.size()  == orig.vsc_converters.size());
}

TEST_CASE("JSON round-trip: case118 MATPOWER preserves bus count", "[io][json][roundtrip][matpower]") {
    auto orig = load_mp("case118.m");
    const std::string json_str = to_json(orig);
    auto restored = from_json(json_str);

    CHECK(restored.ac.buses.size()      == orig.ac.buses.size());
    CHECK(restored.ac.branches.size()   == orig.ac.branches.size());
    CHECK(restored.ac.generators.size() == orig.ac.generators.size());
    // Numeric fidelity on first bus
    CHECK_THAT(restored.ac.buses[0].pd_mw,
               WithinAbs(orig.ac.buses[0].pd_mw, 1e-6));
}

// ═════════════════════════════════════════════════════════════════════════════
// JSON indent variants
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("JSON to_json: indent=0 produces compact output", "[io][json]") {
    auto sys = make_2bus();
    auto compact  = to_json(sys, 0);
    auto indented = to_json(sys, 4);
    // compact must be shorter (no whitespace padding)
    CHECK(compact.size() < indented.size());
    // Both must round-trip
    auto r1 = from_json(compact);
    auto r2 = from_json(indented);
    CHECK(r1.ac.buses.size() == 2);
    CHECK(r2.ac.buses.size() == 2);
}

// ═════════════════════════════════════════════════════════════════════════════
// Exception-free try_ variants
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("try_from_json: valid JSON returns success Result", "[io][json][safe_io]") {
    auto sys = make_2bus();
    const std::string json_str = to_json(sys);

    auto result = try_from_json(json_str);
    REQUIRE(static_cast<bool>(result));
    CHECK(result->ac.buses.size() == 2);
    CHECK_THAT(result->base_mva, WithinAbs(100.0, 1e-9));
}

TEST_CASE("try_from_json: malformed JSON returns Error", "[io][json][safe_io]") {
    auto result = try_from_json("{ this is not json }");
    CHECK_FALSE(static_cast<bool>(result));
    CHECK(result.error().code == ErrorCode::ParseError);
    CHECK_FALSE(result.error().message.empty());
}

TEST_CASE("try_from_json: empty string returns Error", "[io][json][safe_io]") {
    auto result = try_from_json("");
    CHECK_FALSE(static_cast<bool>(result));
    CHECK(result.error().code == ErrorCode::ParseError);
}

TEST_CASE("try_load_json: non-existent file returns FileNotFound", "[io][json][safe_io]") {
    auto result = try_load_json("/tmp/hacdcpf_definitely_does_not_exist_12345.json");
    CHECK_FALSE(static_cast<bool>(result));
    CHECK(result.error().code == ErrorCode::FileNotFound);
}

TEST_CASE("try_load_json: valid file returns success Result", "[io][json][safe_io]") {
    // Write the 2-bus system to a temp file, then reload it.
    auto sys = make_2bus();
    const std::string path = "/tmp/hacdcpf_test_2bus_io.json";
    save_json(sys, path);

    auto result = try_load_json(path);
    REQUIRE(static_cast<bool>(result));
    CHECK(result->ac.buses.size() == 2);
    CHECK(result->name == "json_test_2bus");

    std::filesystem::remove(path);
}

TEST_CASE("try_from_json: ImportMode::Permissive still parses valid JSON", "[io][json][safe_io]") {
    auto sys = make_2bus();
    const std::string json_str = to_json(sys);
    auto result = try_from_json(json_str, ImportMode::Permissive);
    REQUIRE(static_cast<bool>(result));
    CHECK(result->ac.buses.size() == 2);
}

// ═════════════════════════════════════════════════════════════════════════════
// save_json / load_json (throwing variants)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("save_json/load_json: file round-trip for IEEE-24 system", "[io][json][file]") {
    auto orig = build_ieee24_3area_acdc();
    const std::string path = "/tmp/hacdcpf_test_ieee24.json";

    REQUIRE_NOTHROW(save_json(orig, path));
    HybridPowerSystem restored;
    REQUIRE_NOTHROW(restored = load_json(path));

    CHECK(restored.ac.buses.size()       == orig.ac.buses.size());
    CHECK(restored.ac.branches.size()    == orig.ac.branches.size());
    CHECK(restored.dc.buses.size()       == orig.dc.buses.size());
    CHECK(restored.vsc_converters.size() == orig.vsc_converters.size());

    std::filesystem::remove(path);
}

TEST_CASE("load_json: throws on missing file", "[io][json][file]") {
    CHECK_THROWS(load_json("/tmp/hacdcpf_missing_file_xyz.json"));
}

// ═════════════════════════════════════════════════════════════════════════════
// OPF result round-trip
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("OPF result JSON round-trip: converged flag and objective", "[io][json][opf_result]") {
    opf::ACOPFResult r;
    r.converged       = true;
    r.objective       = 12345.678;
    r.iterations      = 42;
    r.status          = "Solve_To_Optimal_Or_Localopt";
    r.vm              = {1.02, 0.99, 1.01};
    r.pg_mw           = {80.0, 50.0};

    const std::string json_str = opf_result_to_json(r);
    REQUIRE_FALSE(json_str.empty());

    auto r2 = opf_result_from_json(json_str);
    CHECK(r2.converged == true);
    CHECK_THAT(r2.objective, WithinAbs(12345.678, 1e-6));
    CHECK(r2.iterations == 42);
    CHECK(r2.vm.size() == 3);
    CHECK_THAT(r2.vm[0], WithinAbs(1.02, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// Power flow result serialisation
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("power_flow_result_to_json: converged 2-bus result serialises", "[io][json][pf_result]") {
    auto sys = make_2bus();
    PowerFlowOptions opt;
    auto result = solve_power_flow(sys, opt);
    REQUIRE(result.converged);

    const std::string json_str = power_flow_result_to_json(sys, result);
    REQUIRE_FALSE(json_str.empty());
    // Should contain key fields
    CHECK_THAT(json_str, ContainsSubstring("converged"));
    CHECK_THAT(json_str, ContainsSubstring("iterations"));
}

// ═════════════════════════════════════════════════════════════════════════════
// MATPOWER parser: parsing of core IEEE cases
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("MATPOWER parser: case5 parses to correct topology", "[io][matpower]") {
    auto sys = load_mp("case5.m");
    CHECK(sys.ac.buses.size()      == 5);
    CHECK(sys.ac.branches.size()   >= 4);
    CHECK(sys.ac.generators.size() >= 1);
    CHECK(sys.base_mva             > 0.0);
    // All bus indices must be positive
    for (const auto& b : sys.ac.buses) CHECK(b.index > 0);
}

TEST_CASE("MATPOWER parser: case9 parses and has valid slack bus", "[io][matpower]") {
    auto sys = load_mp("case9.m");
    CHECK(sys.ac.buses.size() == 9);
    bool found_slack = false;
    for (const auto& b : sys.ac.buses)
        if (b.bus_type == BusType::SLACK) found_slack = true;
    CHECK(found_slack);
}

TEST_CASE("MATPOWER parser: case14 parses with correct bus count", "[io][matpower]") {
    auto sys = load_mp("case14.m");
    CHECK(sys.ac.buses.size() == 14);
    CHECK(sys.ac.branches.size() >= 14);
}

TEST_CASE("MATPOWER parser: case30 parses with loads on multiple buses", "[io][matpower]") {
    auto sys = load_mp("case30.m");
    CHECK(sys.ac.buses.size() == 30);
    double total_load_p = 0.0;
    for (const auto& b : sys.ac.buses) total_load_p += b.pd_mw;
    CHECK(total_load_p > 1.0);
}

TEST_CASE("MATPOWER parser: case57 parses correctly", "[io][matpower]") {
    auto sys = load_mp("case57.m");
    CHECK(sys.ac.buses.size()    == 57);
    CHECK(sys.ac.branches.size() >= 57);
    CHECK(sys.base_mva > 0.0);
}

TEST_CASE("MATPOWER parser: case118 parses with all generator fields", "[io][matpower]") {
    auto sys = load_mp("case118.m");
    CHECK(sys.ac.buses.size()      == 118);
    CHECK(sys.ac.generators.size() >= 54);
    for (const auto& g : sys.ac.generators) {
        CHECK(g.pmax_mw >= g.pmin_mw);
        CHECK(g.bus > 0);
    }
}

TEST_CASE("MATPOWER parser: case300 parses without throwing", "[io][matpower]") {
    HybridPowerSystem sys;
    REQUIRE_NOTHROW(sys = load_mp("case300.m"));
    CHECK(sys.ac.buses.size() == 300);
    CHECK(sys.base_mva > 0.0);
}

TEST_CASE("MATPOWER parser: case33bw distribution case parses", "[io][matpower]") {
    auto sys = load_mp("case33bw.m");
    CHECK(sys.ac.buses.size() == 33);
    CHECK(sys.base_mva > 0.0);
}

TEST_CASE("MATPOWER parser: case39 New England parses", "[io][matpower]") {
    auto sys = load_mp("case39.m");
    CHECK(sys.ac.buses.size() == 39);
    CHECK(sys.ac.generators.size() >= 10);
}

TEST_CASE("MATPOWER parser: case_ieee30 parses", "[io][matpower]") {
    auto sys = load_mp("case_ieee30.m");
    CHECK(sys.ac.buses.size() == 30);
}

TEST_CASE("MATPOWER parser: case24_ieee_rts parses", "[io][matpower]") {
    auto sys = load_mp("case24_ieee_rts.m");
    CHECK(sys.ac.buses.size() == 24);
    CHECK(sys.ac.generators.size() >= 12);
}

// ═════════════════════════════════════════════════════════════════════════════
// MATPOWER parser: branch and generator field validity
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("MATPOWER parser: case14 branch endpoints reference valid buses", "[io][matpower]") {
    auto sys = load_mp("case14.m");
    std::unordered_set<int> bus_ids;
    for (const auto& b : sys.ac.buses) bus_ids.insert(b.index);

    for (const auto& br : sys.ac.branches) {
        CHECK(bus_ids.count(br.from_bus) == 1);
        CHECK(bus_ids.count(br.to_bus)   == 1);
    }
}

TEST_CASE("MATPOWER parser: case9 generator dispatch within limits", "[io][matpower]") {
    auto sys = load_mp("case9.m");
    for (const auto& g : sys.ac.generators) {
        if (!g.in_service) continue;
        CHECK(g.pmax_mw >= g.pmin_mw);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// MATPOWER: large cases parse without exception
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("MATPOWER parser: case1354pegase parses", "[io][matpower][large]") {
    HybridPowerSystem sys;
    REQUIRE_NOTHROW(sys = load_mp("case1354pegase.m"));
    CHECK(sys.ac.buses.size() >= 1354);
}

TEST_CASE("MATPOWER parser: case2383wp parses", "[io][matpower][large]") {
    HybridPowerSystem sys;
    REQUIRE_NOTHROW(sys = load_mp("case2383wp.m"));
    CHECK(sys.ac.buses.size() >= 2383);
}

// ═════════════════════════════════════════════════════════════════════════════
// MATPOWER: solve power flow on parsed cases
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("MATPOWER parse + power flow: case5 converges", "[io][matpower][pf]") {
    auto sys = load_mp("case5.m");
    auto result = solve_power_flow(sys);
    REQUIRE(result.converged);
    for (const auto& vm : result.vm) {
        CHECK(vm >= 0.5);
        CHECK(vm <= 1.5);
    }
}

TEST_CASE("MATPOWER parse + power flow: case9 converges", "[io][matpower][pf]") {
    auto sys = load_mp("case9.m");
    auto result = solve_power_flow(sys);
    REQUIRE(result.converged);
}

TEST_CASE("MATPOWER parse + power flow: case14 converges", "[io][matpower][pf]") {
    auto sys = load_mp("case14.m");
    auto result = solve_power_flow(sys);
    REQUIRE(result.converged);
}

TEST_CASE("MATPOWER parse + JSON round-trip + power flow: case14", "[io][matpower][json][pf][roundtrip]") {
    // Parse → serialise to JSON → deserialise → solve.
    auto orig    = load_mp("case14.m");
    auto json_str = to_json(orig);
    auto restored = from_json(json_str);

    auto r_orig     = solve_power_flow(orig);
    auto r_restored = solve_power_flow(restored);

    REQUIRE(r_orig.converged);
    REQUIRE(r_restored.converged);

    // Voltage magnitudes must agree to within numerical noise.
    REQUIRE(r_orig.vm.size() == r_restored.vm.size());
    for (size_t i = 0; i < r_orig.vm.size(); ++i)
        CHECK_THAT(r_restored.vm[i], WithinAbs(r_orig.vm[i], 1e-6));
}

// ═════════════════════════════════════════════════════════════════════════════
// JSON: loaded case3/case4 formal SOC piecewise test data
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("JSON load: case3_case4 formal SOC piecewise file is valid JSON", "[io][json][testdata]") {
    const std::string path = data_path("case3_case4_formal_soc_piecewise.json");
    if (!fs::exists(path)) SKIP("Test data file not present");

    // This file is an economics/piecewise-cost JSON, not a HybridPowerSystem.
    // Verify that nlohmann can parse it without throwing.
    std::ifstream ifs(path);
    REQUIRE(ifs.good());
    nlohmann::json j;
    REQUIRE_NOTHROW(j = nlohmann::json::parse(ifs));
    CHECK(!j.empty());  // file is non-empty valid JSON
}

// ═════════════════════════════════════════════════════════════════════════════
// JPC JSON: full rich-component round-trip
// Verifies that all 12 extended component types added in Round 3 survive the
// to_jpc_json → from_jpc_json cycle with their collection sizes intact.
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("JPC JSON round-trip: all 12 rich component types preserved",
          "[io][json][jpc][round-trip][integration]") {
    using namespace hacdcpf::io;

    // ── Base AC skeleton (minimal 2-bus) ────────────────────────────────────
    HybridPowerSystem sys;
    sys.name     = "jpc_richcomponent_rt";
    sys.base_mva = 100.0;

    ACBus b1; b1.index=1; b1.bus_type=BusType::SLACK; b1.vm_pu=1.0; b1.in_service=true;
    ACBus b2; b2.index=2; b2.bus_type=BusType::PQ;   b2.vm_pu=1.0; b2.in_service=true;
    sys.ac.buses = {b1, b2};

    ACBranch br; br.from_bus=1; br.to_bus=2; br.r_pu=0.01; br.x_pu=0.04; br.in_service=true;
    sys.ac.branches = {br};

    Generator g; g.bus=1; g.is_slack=true; g.pmax_mw=100.0; g.pmin_mw=0.0; g.in_service=true;
    sys.ac.generators = {g};

    // ── One DC bus (needed by DCDCConverter / DCCircuitBreaker) ─────────────
    DCBus db; db.index=10; db.in_service=true;
    sys.dc.buses = {db};

    // ── 1. ShuntAC ───────────────────────────────────────────────────────────
    Shunt sh; sh.bus=2; sh.gs_mw=0.0; sh.bs_mvar=5.0; sh.in_service=true;
    sys.ac.shunts = {sh};

    // ── 2. RenewableGen ──────────────────────────────────────────────────────
    RenewableGen rg; rg.bus=2; rg.p_mw=10.0; rg.in_service=true;
    sys.ac.renewable_gens = {rg};

    // ── 3. Transformer2W ─────────────────────────────────────────────────────
    Transformer2W t2; t2.hv_bus=1; t2.lv_bus=2; t2.sn_mva=50.0; t2.in_service=true;
    sys.ac.transformers_2w = {t2};

    // ── 4. Transformer3W ─────────────────────────────────────────────────────
    Transformer3W t3; t3.hv_bus=1; t3.mv_bus=2; t3.lv_bus=2; t3.sn_hv_mva=30.0; t3.in_service=true;
    sys.ac.transformers_3w = {t3};

    // ── 5. Switch ────────────────────────────────────────────────────────────
    Switch sw; sw.bus_from=1; sw.bus_to=2; sw.closed=true; sw.in_service=true;
    sys.ac.switches = {sw};

    // ── 6. EVChargingStation ─────────────────────────────────────────────────
    ChargingStation cs; cs.bus=2; cs.in_service=true;
    sys.ac.charging_stations = {cs};

    // ── 7. Charger ───────────────────────────────────────────────────────────
    Charger ch; ch.p_ch_max_kw=50.0; ch.in_service=true;
    sys.ac.chargers = {ch};

    // ── 8. AsynchronousMotor ─────────────────────────────────────────────────
    AsynchronousMotor mo; mo.bus=2; mo.sn_mva=5.0; mo.in_service=true;
    sys.ac.motors = {mo};

    // ── 9. MobileStorage ─────────────────────────────────────────────────────
    MobileStorage ms; ms.bus=2; ms.e_rated_mwh=10.0; ms.in_service=true;
    sys.mobile_storage = {ms};

    // ── 10. VirtualPowerPlant ────────────────────────────────────────────────
    VirtualPowerPlant vpp; vpp.pcc_bus=1; vpp.pmax_mw=20.0; vpp.in_service=true;
    sys.vpps = {vpp};

    // ── 11. DCDCConverter ────────────────────────────────────────────────────
    DCDCConverter dcdc; dcdc.bus_in=10; dcdc.bus_out=10; dcdc.in_service=true;
    sys.dc.dcdc_converters = {dcdc};

    // ── 12. DCCircuitBreaker ─────────────────────────────────────────────────
    DCCircuitBreaker dccb; dccb.bus_from=10; dccb.bus_to=10; dccb.in_service=true;
    sys.dc.dc_circuit_breakers = {dccb};

    // ── Serialize → deserialize ──────────────────────────────────────────────
    const std::string jpc_str = to_jpc_json(sys);
    REQUIRE_FALSE(jpc_str.empty());

    HybridPowerSystem rt = from_jpc_json(jpc_str);

    // ── Verify all 12 collection sizes survive the round-trip ────────────────
    CHECK(rt.ac.shunts.size()            == sys.ac.shunts.size());
    CHECK(rt.ac.renewable_gens.size()    == sys.ac.renewable_gens.size());
    CHECK(rt.ac.transformers_2w.size()   == sys.ac.transformers_2w.size());
    CHECK(rt.ac.transformers_3w.size()   == sys.ac.transformers_3w.size());
    CHECK(rt.ac.switches.size()          == sys.ac.switches.size());
    CHECK(rt.ac.charging_stations.size() == sys.ac.charging_stations.size());
    CHECK(rt.ac.chargers.size()          == sys.ac.chargers.size());
    CHECK(rt.ac.motors.size()            == sys.ac.motors.size());
    CHECK(rt.mobile_storage.size()       == sys.mobile_storage.size());
    CHECK(rt.vpps.size()                 == sys.vpps.size());
    CHECK(rt.dc.dcdc_converters.size()   == sys.dc.dcdc_converters.size());
    CHECK(rt.dc.dc_circuit_breakers.size() == sys.dc.dc_circuit_breakers.size());

    // ── Core topology also preserved ─────────────────────────────────────────
    CHECK(rt.ac.buses.size()      == sys.ac.buses.size());
    CHECK(rt.ac.branches.size()   == sys.ac.branches.size());
    CHECK(rt.ac.generators.size() == sys.ac.generators.size());
}

// ═════════════════════════════════════════════════════════════════════════════
// JPC JSON: field-level round-trip correctness (Round 7 regression guard)
// Checks that branch indices (DICTKEY), generator is_slack, and storage
// fields survive the to_jpc_json → from_jpc_json cycle without loss.
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("JPC JSON round-trip: branch index and generator slack preserved",
          "[io][json][jpc][round-trip][fields][integration]") {
    using namespace hacdcpf::io;
    using Catch::Matchers::WithinAbs;

    HybridPowerSystem sys;
    sys.base_mva = 100.0;

    // Bus 5 is SLACK (non-default index, non-1)
    ACBus b5; b5.index=5; b5.bus_type=BusType::SLACK; b5.vm_pu=1.0; b5.in_service=true;
    ACBus b7; b7.index=7; b7.bus_type=BusType::PQ;   b7.vm_pu=1.0; b7.in_service=true;
    sys.ac.buses = {b5, b7};

    // Branch with non-zero, non-sequential index
    ACBranch br; br.index=42; br.from_bus=5; br.to_bus=7;
    br.r_pu=0.02; br.x_pu=0.05; br.in_service=true;
    sys.ac.branches = {br};

    // Generator on bus 5 (SLACK)
    Generator g; g.bus=5; g.is_slack=true; g.pmax_mw=200.0; g.in_service=true;
    sys.ac.generators = {g};

    // Storage with non-default fields
    Storage st; st.index=0; st.bus=7; st.pmax_mw=10.0; st.e_rated_mwh=20.0;
    st.soc_init=0.6; st.soc_min=0.1; st.soc_max=0.95;
    st.eta_charge=0.93; st.eta_discharge=0.91; st.in_service=true;
    sys.ac.storage = {st};

    const std::string jpc_str = to_jpc_json(sys);
    HybridPowerSystem rt = from_jpc_json(jpc_str);

    // Branch index must survive via DICTKEY column
    REQUIRE(rt.ac.branches.size() == 1);
    CHECK(rt.ac.branches[0].index == 42);

    // Generator is_slack must be derived from bus type, not hardcoded bus 1
    REQUIRE(rt.ac.generators.size() == 1);
    CHECK(rt.ac.generators[0].is_slack == true);

    // Storage fields must survive via storageetap (structured JSON)
    REQUIRE(rt.ac.storage.size() == 1);
    CHECK_THAT(rt.ac.storage[0].e_rated_mwh,     WithinAbs(20.0, 1e-9));
    CHECK_THAT(rt.ac.storage[0].soc_init,         WithinAbs(0.6,  1e-9));
    CHECK_THAT(rt.ac.storage[0].eta_charge,       WithinAbs(0.93, 1e-9));
    CHECK_THAT(rt.ac.storage[0].eta_discharge,    WithinAbs(0.91, 1e-9));
}
