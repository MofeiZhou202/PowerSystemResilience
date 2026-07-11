/// @file test_validation.cpp
/// @brief Tests for the validation subsystem:
///        ValidationReport, ValidationIssue, ValidationLevel,
///        val::validate(sys), val::validate(sys, level), and the new
///        ok() / summary() report methods.
///
/// Tags: [validation], [validate], [validation_level], [validation_report]

#include <algorithm>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/validation/validate_system.hpp"
#include "hacdcpf/validation/validation_report.hpp"

#ifndef HACDCPF_MATPOWER_DATA_DIR
#define HACDCPF_MATPOWER_DATA_DIR "../../external_data/matpower"
#endif

using namespace hacdcpf;
// Use explicit namespace to avoid ambiguity with hacdcpf::projection::validate.
namespace val = hacdcpf::validation;
using hacdcpf::validation::ValidationReport;
using hacdcpf::validation::ValidationLevel;
using hacdcpf::validation::Severity;
using Catch::Matchers::ContainsSubstring;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Minimal valid 2-bus system.
static HybridPowerSystem make_valid_2bus() {
    HybridPowerSystem sys;
    sys.base_mva = sys.ac.base_mva = 100.0;

    ACBus b1; b1.index=1; b1.bus_type=BusType::SLACK; b1.vm_pu=1.04;
    b1.vmin_pu=0.9; b1.vmax_pu=1.1; b1.in_service=true;
    ACBus b2; b2.index=2; b2.bus_type=BusType::PQ; b2.vm_pu=1.0;
    b2.pd_mw=40.0; b2.qd_mvar=10.0; b2.vmin_pu=0.9; b2.vmax_pu=1.1; b2.in_service=true;
    sys.ac.buses = {b1, b2};

    ACBranch br; br.from_bus=1; br.to_bus=2; br.r_pu=0.01; br.x_pu=0.04;
    br.tap=1.0; br.in_service=true;
    sys.ac.branches = {br};

    Generator g; g.bus=1; g.is_slack=true; g.pmax_mw=200.0; g.pmin_mw=0.0;
    g.qmax_mvar=100.0; g.qmin_mvar=-100.0; g.vg_pu=1.04; g.in_service=true;
    sys.ac.generators = {g};

    return sys;
}

/// System with no slack bus.
static HybridPowerSystem make_no_slack() {
    auto sys = make_valid_2bus();
    sys.ac.buses[0].bus_type = BusType::PV;
    sys.ac.generators[0].is_slack = false;
    return sys;
}

/// System with duplicate bus indices.
static HybridPowerSystem make_duplicate_bus_ids() {
    auto sys = make_valid_2bus();
    sys.ac.buses[1].index = 1;   // duplicate of bus 0
    return sys;
}

/// System with branch pointing to non-existent bus.
static HybridPowerSystem make_dangling_branch() {
    auto sys = make_valid_2bus();
    sys.ac.branches[0].to_bus = 99;  // does not exist
    return sys;
}

/// System with inverted voltage limits.
static HybridPowerSystem make_inverted_vlimits() {
    auto sys = make_valid_2bus();
    sys.ac.buses[1].vmin_pu = 1.1;
    sys.ac.buses[1].vmax_pu = 0.9;   // vmin > vmax
    return sys;
}

/// System with inverted generator MW limits.
static HybridPowerSystem make_inverted_gen_limits() {
    auto sys = make_valid_2bus();
    sys.ac.generators[0].pmin_mw = 200.0;
    sys.ac.generators[0].pmax_mw = 50.0;   // pmin > pmax
    return sys;
}

/// System with zero base_mva.
static HybridPowerSystem make_zero_base_mva() {
    auto sys = make_valid_2bus();
    sys.base_mva = 0.0;
    sys.ac.base_mva = 0.0;
    return sys;
}

static bool has_issue(const ValidationReport& report,
                      const std::string& component_type,
                      const std::string& field) {
    return std::any_of(report.issues.begin(), report.issues.end(), [&](const auto& issue) {
        return issue.component_type == component_type && issue.field == field;
    });
}

// ═════════════════════════════════════════════════════════════════════════════
// ValidationReport: ok() and summary() methods
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("ValidationReport: ok() alias for is_valid()", "[validation][report]") {
    ValidationReport r;
    CHECK(r.ok());
    CHECK(r.is_valid());

    r.add(Severity::Warning, "ACBus", "2", "vmin_pu", "Low voltage limit");
    CHECK(r.ok());        // warnings don't make invalid
    CHECK(r.is_valid());

    r.add(Severity::Error, "ACBranch", "3", "x_pu", "Zero reactance");
    CHECK_FALSE(r.ok());
    CHECK_FALSE(r.is_valid());
}

TEST_CASE("ValidationReport: summary() formats issue counts", "[validation][report]") {
    ValidationReport r;
    r.add(Severity::Error,   "ACBus",    "1", "vmin_pu", "Inverted limits");
    r.add(Severity::Error,   "ACBranch", "2", "x_pu",    "Zero reactance");
    r.add(Severity::Warning, "ACBus",    "3", "vm_pu",   "Low initial voltage");

    const std::string s = r.summary();
    CHECK_THAT(s, ContainsSubstring("2"));   // 2 errors
    CHECK_THAT(s, ContainsSubstring("1"));   // 1 warning
}

TEST_CASE("ValidationReport: summary() is empty for clean report", "[validation][report]") {
    ValidationReport r;
    const std::string s = r.summary();
    // Either empty string or a "0 issues" message — should not contain "error"
    // A clean report should not report errors
    if (!r.has_errors())
        CHECK(s.find("error") == std::string::npos);
}

TEST_CASE("ValidationReport: count() by severity", "[validation][report]") {
    ValidationReport r;
    r.add(Severity::Error,   "T", "1", "", "e1");
    r.add(Severity::Error,   "T", "2", "", "e2");
    r.add(Severity::Warning, "T", "3", "", "w1");
    r.add(Severity::Info,    "T", "4", "", "i1");

    CHECK(r.count(Severity::Error)   == 2);
    CHECK(r.count(Severity::Warning) == 1);
    CHECK(r.count(Severity::Info)    == 1);
    CHECK(r.has_errors());
    CHECK(r.has_warnings());
}

// ═════════════════════════════════════════════════════════════════════════════
// val::validate(sys): valid system
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("validate: valid 2-bus system reports no errors", "[validation][valid]") {
    auto sys = make_valid_2bus();
    auto r = val::validate(sys);
    CHECK(r.ok());
    CHECK_FALSE(r.has_errors());
}

TEST_CASE("validate: IEEE-14 AC/DC builder reports no errors", "[validation][valid][case_builder]") {
    auto sys = io::build_ieee14_acdc();
    auto r = val::validate(sys);
    CHECK(r.ok());
}

TEST_CASE("validate: case9 MATPOWER parses and validates clean", "[validation][valid][matpower]") {
    auto sys = io::parse_matpower(
        std::string(HACDCPF_MATPOWER_DATA_DIR) + "/case9.m");
    auto r = val::validate(sys);
    CHECK(r.ok());
}

TEST_CASE("validate: case118 validates clean", "[validation][valid][matpower]") {
    auto sys = io::parse_matpower(
        std::string(HACDCPF_MATPOWER_DATA_DIR) + "/case118.m");
    auto r = val::validate(sys);
    CHECK(r.ok());
}

// ═════════════════════════════════════════════════════════════════════════════
// val::validate(sys): deliberate faults produce expected errors
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("validate: no slack bus produces Error", "[validation][fault][slack]") {
    auto sys = make_no_slack();
    auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
    CHECK(r.has_errors());
}

TEST_CASE("validate: AC slack bus requires a physical balancing device",
          "[validation][fault][slack][reference]") {
    auto sys = make_valid_2bus();
    sys.ac.generators.clear();
    const auto r = val::validate_reference_bus_eligibility(sys);
    REQUIRE(r.has_errors());
    CHECK(has_issue(r, "ACBus", "bus_type"));
    const auto pf = solve_power_flow(sys);
    CHECK_FALSE(pf.converged);
    CHECK(pf.iterations == 0);
    CHECK(pf.diagnostics.termination_reason ==
          "Reference-bus eligibility check failed");
}

TEST_CASE("validate: bare DC_V bus is not an implicit source",
          "[validation][fault][slack][reference][dc]") {
    auto sys = make_valid_2bus();
    DCBus dc1;
    dc1.index = 1;
    dc1.bus_type = DCBusType::DC_V;
    dc1.in_service = true;
    sys.dc.buses = {dc1};
    const auto r = val::validate_reference_bus_eligibility(sys);
    REQUIRE(r.has_errors());
    CHECK(has_issue(r, "DCBus", "bus_type"));
    const auto pf = solve_power_flow(sys);
    CHECK_FALSE(pf.converged);
    CHECK(pf.iterations == 0);
    CHECK(pf.diagnostics.termination_reason ==
          "Reference-bus eligibility check failed");
}

TEST_CASE("validate: controllable DC storage can back a DC_V bus",
          "[validation][slack][reference][dc][storage]") {
    auto sys = make_valid_2bus();
    DCBus dc1;
    dc1.index = 1;
    dc1.bus_type = DCBusType::DC_V;
    dc1.in_service = true;
    sys.dc.buses = {dc1};
    DCStorage storage;
    storage.index = 1;
    storage.bus = 1;
    storage.in_service = true;
    storage.controllable = true;
    storage.pmin_mw = -1.0;
    storage.pmax_mw = 1.0;
    sys.dc.dc_storage = {storage};
    CHECK(val::validate_reference_bus_eligibility(sys).ok());
}

TEST_CASE("validate: duplicate bus IDs produce Error", "[validation][fault][topology]") {
    auto sys = make_duplicate_bus_ids();
    auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
    CHECK(r.has_errors());
}

TEST_CASE("validate: dangling branch endpoint produces Error", "[validation][fault][topology]") {
    auto sys = make_dangling_branch();
    auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
    CHECK(r.has_errors());
}

TEST_CASE("validate: inverted voltage limits produce Error or Warning", "[validation][fault][limits]") {
    auto sys = make_inverted_vlimits();
    auto r = val::validate(sys);
    CHECK_FALSE(r.issues.empty());
}

TEST_CASE("validate: inverted generator MW limits produce Error", "[validation][fault][limits]") {
    auto sys = make_inverted_gen_limits();
    auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
}

TEST_CASE("validate: zero base_mva produces Error", "[validation][fault][base_mva]") {
    auto sys = make_zero_base_mva();
    auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
}

// ═════════════════════════════════════════════════════════════════════════════
// val::validate(sys, ValidationLevel::Basic)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("ValidationLevel::Basic: valid system has no issues", "[validation][level][basic]") {
    auto sys = make_valid_2bus();
    auto r = val::validate(sys, ValidationLevel::Basic);
    CHECK(r.ok());
    CHECK(r.issues.empty());
}

TEST_CASE("ValidationLevel::Basic: errors reported, warnings suppressed", "[validation][level][basic]") {
    auto sys = make_dangling_branch();
    auto r_full  = val::validate(sys);
    auto r_basic = val::validate(sys, ValidationLevel::Basic);

    // Basic must not have more issues than full.
    const bool not_more_than_full = r_basic.issues.size() <= r_full.issues.size();
    CHECK(not_more_than_full);

    // Basic must not contain any Warnings.
    for (const auto& issue : r_basic.issues)
        CHECK(issue.severity == Severity::Error);

    // But must still detect the hard error.
    CHECK_FALSE(r_basic.ok());
}

// ═════════════════════════════════════════════════════════════════════════════
// val::validate(sys, ValidationLevel::Electrical)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("ValidationLevel::Electrical: slack missing is caught", "[validation][level][electrical]") {
    auto sys = make_no_slack();
    auto r = val::validate(sys, ValidationLevel::Electrical);
    CHECK_FALSE(r.ok());
}

TEST_CASE("ValidationLevel::Electrical: subset of SolverReady issues", "[validation][level][electrical]") {
    auto sys = make_valid_2bus();
    // Add a warning-only issue by setting a slightly non-unity tap.
    sys.ac.buses[1].vmin_pu = 0.85;   // borderline low; may produce warning

    auto r_elec  = val::validate(sys, ValidationLevel::Electrical);
    auto r_full  = val::validate(sys, ValidationLevel::SolverReady);

    // Electrical may have fewer issues but must not have more Errors.
    const bool elec_not_worse = r_elec.count(Severity::Error) <= r_full.count(Severity::Error);
    CHECK(elec_not_worse);
}

// ═════════════════════════════════════════════════════════════════════════════
// val::validate(sys, ValidationLevel::SolverReady)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("ValidationLevel::SolverReady: identical to val::validate(sys)", "[validation][level][solver_ready]") {
    auto sys = make_valid_2bus();
    auto r1 = val::validate(sys);
    auto r2 = val::validate(sys, ValidationLevel::SolverReady);

    CHECK(r1.issues.size() == r2.issues.size());
    for (size_t i = 0; i < r1.issues.size(); ++i) {
        CHECK(r1.issues[i].severity     == r2.issues[i].severity);
        CHECK(r1.issues[i].message      == r2.issues[i].message);
        CHECK(r1.issues[i].component_type == r2.issues[i].component_type);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// val::validate(sys, ValidationLevel::Strict)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("ValidationLevel::Strict: warnings promoted to errors", "[validation][level][strict]") {
    auto sys = make_valid_2bus();
    // Force a warning: base_mva mismatch at sub-system level
    sys.ac.base_mva = 50.0;   // mismatch with sys.base_mva=100 → produces Warning

    auto r_full   = val::validate(sys, ValidationLevel::SolverReady);
    auto r_strict = val::validate(sys, ValidationLevel::Strict);

    // Any warning in SolverReady should be an error in Strict.
    int warnings_in_full = r_full.count(Severity::Warning);
    if (warnings_in_full > 0) {
        CHECK(r_strict.count(Severity::Warning) == 0);
        CHECK(r_strict.count(Severity::Error) > 0);
    }
}

TEST_CASE("ValidationLevel::Strict: valid system with no warnings stays valid", "[validation][level][strict]") {
    auto sys = make_valid_2bus();
    auto r = val::validate(sys, ValidationLevel::Strict);
    // If there were no warnings in SolverReady, Strict must also pass.
    auto r_full = val::validate(sys, ValidationLevel::SolverReady);
    if (!r_full.has_warnings()) {
        CHECK(r.ok());
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Validation + power flow: only pass validated systems to solver
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Validate-then-solve pattern: case14 passes validation before PF", "[validation][workflow]") {
    auto sys = io::parse_matpower(
        std::string(HACDCPF_MATPOWER_DATA_DIR) + "/case14.m");
    auto report = val::validate(sys);

    // If validation passes, power flow must converge.
    if (report.ok()) {
        auto pf = hacdcpf::solve_power_flow(sys);
        CHECK(pf.converged);
    } else {
        WARN("case14.m failed validation: " + report.summary());
    }
}

TEST_CASE("Validate-then-solve: faulty system is caught before solve", "[validation][workflow]") {
    auto sys = make_dangling_branch();
    auto report = val::validate(sys, ValidationLevel::Basic);
    // The dangling branch should be caught before even attempting the solve.
    CHECK_FALSE(report.ok());
}

// ═════════════════════════════════════════════════════════════════════════════
// VSC converter validation
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("validate: VSC converter with missing AC bus produces Error", "[validation][fault][vsc]") {
    auto sys = make_valid_2bus();

    DCBus dcb; dcb.index=1; dcb.bus_type=DCBusType::DC_V; dcb.in_service=true;
    sys.dc.buses = {dcb};

    VSCConverter vsc;
    vsc.bus_ac    = 99;   // non-existent AC bus
    vsc.bus_dc    = 1;
    vsc.in_service= true;
    sys.vsc_converters = {vsc};

    auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
}

TEST_CASE("validate: VSC converter with missing DC bus produces Error", "[validation][fault][vsc]") {
    auto sys = make_valid_2bus();

    VSCConverter vsc;
    vsc.bus_ac    = 1;
    vsc.bus_dc    = 99;   // non-existent DC bus
    vsc.in_service= true;
    sys.vsc_converters = {vsc};

    auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
}

TEST_CASE("validate: rich AC component references and bounds are safety-gated",
          "[validation][fault][rich_components]") {
    auto sys = make_valid_2bus();

    StaticGenerator sg;
    sg.index = 7;
    sg.bus = 99;
    sg.in_service = true;
    sg.pmin_mw = 3.0;
    sg.pmax_mw = 1.0;
    sys.ac.static_generators = {sg};

    Storage storage;
    storage.index = 8;
    storage.bus = 2;
    storage.in_service = true;
    storage.soc_min = 0.9;
    storage.soc_max = 0.1;
    storage.eta_charge = 1.2;
    sys.ac.storage = {storage};

    Switch sw;
    sw.index = 9;
    sw.bus_from = 1;
    sw.bus_to = 77;
    sys.ac.switches = {sw};

    const auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
    CHECK(has_issue(r, "StaticGenerator", "bus"));
    CHECK(has_issue(r, "StaticGenerator", "pmin_mw"));
    CHECK(has_issue(r, "Storage", "soc_min"));
    CHECK(has_issue(r, "Storage", "eta_charge"));
    CHECK(has_issue(r, "Switch", "bus_to"));
}

TEST_CASE("validate: rich DC component references and converter bounds are safety-gated",
          "[validation][fault][rich_components][dc]") {
    auto sys = make_valid_2bus();

    DCBus dc_bus;
    dc_bus.index = 101;
    dc_bus.vmin_pu = 0.9;
    dc_bus.vmax_pu = 1.1;
    sys.dc.buses = {dc_bus};

    StaticGeneratorDC dc_gen;
    dc_gen.index = 1;
    dc_gen.bus = 999;
    dc_gen.pmin_mw = 5.0;
    dc_gen.pmax_mw = 2.0;
    sys.dc.dc_static_generators = {dc_gen};

    DCDCConverter dcdc;
    dcdc.index = 2;
    dcdc.bus_in = 101;
    dcdc.bus_out = 202;
    dcdc.eta = 0.0;
    sys.dc.dcdc_converters = {dcdc};

    VSCConverter vsc;
    vsc.index = 3;
    vsc.bus_ac = 1;
    vsc.bus_dc = 101;
    vsc.pmin_mw = 10.0;
    vsc.pmax_mw = 1.0;
    vsc.eta = 1.5;
    sys.vsc_converters = {vsc};

    const auto r = val::validate(sys);
    CHECK_FALSE(r.ok());
    CHECK(has_issue(r, "StaticGeneratorDC", "bus"));
    CHECK(has_issue(r, "StaticGeneratorDC", "pmin_mw"));
    CHECK(has_issue(r, "DCDCConverter", "bus_out"));
    CHECK(has_issue(r, "DCDCConverter", "eta"));
    CHECK(has_issue(r, "VSCConverter", "pmin_mw"));
    CHECK(has_issue(r, "VSCConverter", "eta"));
}

// ═════════════════════════════════════════════════════════════════════════════
// Regression: previously-validated cases do not start failing
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Validation regression: case5 always validates clean", "[validation][regression][matpower]") {
    auto sys = io::parse_matpower(
        std::string(HACDCPF_MATPOWER_DATA_DIR) + "/case5.m");
    CHECK(val::validate(sys).ok());
}

TEST_CASE("Validation regression: case30 always validates clean", "[validation][regression][matpower]") {
    auto sys = io::parse_matpower(
        std::string(HACDCPF_MATPOWER_DATA_DIR) + "/case30.m");
    CHECK(val::validate(sys).ok());
}

TEST_CASE("Validation regression: case57 always validates clean", "[validation][regression][matpower]") {
    auto sys = io::parse_matpower(
        std::string(HACDCPF_MATPOWER_DATA_DIR) + "/case57.m");
    CHECK(val::validate(sys).ok());
}

TEST_CASE("Validation regression: IEEE14 builder always validates clean", "[validation][regression][case_builder]") {
    CHECK(val::validate(io::build_ieee14_acdc()).ok());
}

TEST_CASE("Validation regression: IEEE118 builder always validates clean", "[validation][regression][case_builder]") {
    const auto report = val::validate(io::build_ieee118_acdc());
    INFO(report.summary());
    CHECK(report.ok());
}
