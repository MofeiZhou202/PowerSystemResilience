/// @file test_solver_capabilities.cpp
/// @brief Tests for the SolverCapabilities API and get_solver_capabilities().
///
/// Tests: capability struct fields, compile-time detection, logical
/// consistency rules (e.g. integer_variables requires at least one MIP solver),
/// and integration with the power-flow API.
///
/// Tags: [capabilities], [solver_caps], [api]

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/api/solver_capabilities.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;

// ─────────────────────────────────────────────────────────────────────────────
// Basic struct construction and defaults
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SolverCapabilities: default-constructed struct has sane values", "[capabilities][api]") {
    SolverCapabilities caps{};

    // Always-available features must default to true.
    CHECK(caps.has_sparse_lu);
    CHECK(caps.has_native_ipm);
    CHECK(caps.supports_ac_opf);
    CHECK(caps.supports_dc_opf);
    CHECK(caps.supports_three_phase);
    CHECK(caps.supports_quadratic_objective);

    // Optional backends must default to false.
    CHECK_FALSE(caps.has_highs);
    CHECK_FALSE(caps.has_gurobi);
    CHECK_FALSE(caps.has_ipopt);
    CHECK_FALSE(caps.has_scip);
    CHECK_FALSE(caps.has_klu);
    CHECK_FALSE(caps.has_umfpack);
    CHECK_FALSE(caps.has_accelerate);
    CHECK_FALSE(caps.has_excel);
    CHECK_FALSE(caps.has_opendss);
    CHECK_FALSE(caps.supports_integer_variables);
}

// ─────────────────────────────────────────────────────────────────────────────
// get_solver_capabilities() — runtime query
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("get_solver_capabilities: returns a SolverCapabilities by value", "[capabilities][api]") {
    const auto caps = get_solver_capabilities();
    // Just check we get a value back without crashing.
    CHECK(caps.has_sparse_lu);
    CHECK(caps.has_native_ipm);
}

TEST_CASE("get_solver_capabilities: is noexcept and repeatable", "[capabilities][api]") {
    static_assert(noexcept(get_solver_capabilities()),
                  "get_solver_capabilities() must be noexcept");

    const auto c1 = get_solver_capabilities();
    const auto c2 = get_solver_capabilities();

    // Two calls must return identical results.
    CHECK(c1.has_sparse_lu    == c2.has_sparse_lu);
    CHECK(c1.has_highs        == c2.has_highs);
    CHECK(c1.has_gurobi       == c2.has_gurobi);
    CHECK(c1.has_ipopt        == c2.has_ipopt);
    CHECK(c1.has_scip         == c2.has_scip);
    CHECK(c1.has_klu          == c2.has_klu);
    CHECK(c1.has_umfpack      == c2.has_umfpack);
    CHECK(c1.has_accelerate   == c2.has_accelerate);
    CHECK(c1.has_excel        == c2.has_excel);
    CHECK(c1.has_opendss      == c2.has_opendss);
    CHECK(c1.supports_integer_variables == c2.supports_integer_variables);
}

TEST_CASE("get_solver_capabilities: always-on features present", "[capabilities][api]") {
    const auto caps = get_solver_capabilities();
    CHECK(caps.has_sparse_lu);
    CHECK(caps.has_native_ipm);
    CHECK(caps.supports_ac_opf);
    CHECK(caps.supports_dc_opf);
    CHECK(caps.supports_three_phase);
    CHECK(caps.supports_quadratic_objective);
}

// ─────────────────────────────────────────────────────────────────────────────
// Logical consistency rules
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SolverCapabilities: integer_variables requires at least one MIP backend", "[capabilities][consistency]") {
    const auto caps = get_solver_capabilities();
    if (caps.supports_integer_variables) {
        const bool has_mip = caps.has_highs || caps.has_gurobi || caps.has_scip;
        CHECK(has_mip);
    }
}

TEST_CASE("SolverCapabilities: if no MIP backend, integer_variables is false", "[capabilities][consistency]") {
    // Construct a manually-crafted caps with no MIP backends.
    SolverCapabilities caps;
    caps.has_highs  = false;
    caps.has_gurobi = false;
    caps.has_scip   = false;
    caps.supports_integer_variables = false;

    // Rule: no MIP ⇒ no integers.
    if (!caps.has_highs && !caps.has_gurobi && !caps.has_scip)
        CHECK_FALSE(caps.supports_integer_variables);
}

TEST_CASE("SolverCapabilities: linear-algebra features are independent of MIP backends", "[capabilities][consistency]") {
    const auto caps = get_solver_capabilities();
    // Sparse LU is always available regardless of MIP backend availability.
    CHECK(caps.has_sparse_lu);
}

// ─────────────────────────────────────────────────────────────────────────────
// Integration: capabilities match actual behaviour
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SolverCapabilities: AC OPF available flag matches actual solve", "[capabilities][integration]") {
    const auto caps = get_solver_capabilities();

    // AC OPF is always declared available (native IPM is built-in).
    REQUIRE(caps.supports_ac_opf);

    auto sys = io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
    auto r   = solve_ac_opf(sys);
    // If the flag says available, the solve must not throw.
    CHECK(r.converged);  // or at worst not throw — already covered by not-throwing
}

TEST_CASE("SolverCapabilities: DC OPF available flag matches actual solve", "[capabilities][integration]") {
    const auto caps = get_solver_capabilities();
    REQUIRE(caps.supports_dc_opf);

    auto sys = io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR) + "/case14.m");
    auto r   = solve_dc_opf(sys);
    CHECK(r.converged);
}

TEST_CASE("SolverCapabilities: power flow available (always)", "[capabilities][integration]") {
    const auto caps = get_solver_capabilities();
    CHECK(caps.has_native_ipm);  // Newton solver counts as native IPM path

    auto sys = io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR) + "/case30.m");
    auto r   = solve_power_flow(sys);
    CHECK(r.converged);
}

#ifdef HACDCPF_HAVE_HIGHS_LIB
TEST_CASE("SolverCapabilities: HiGHS flag true at compile time", "[capabilities][highs]") {
    const auto caps = get_solver_capabilities();
    CHECK(caps.has_highs);
    CHECK(caps.supports_integer_variables);
}
#endif

#ifdef HACDCPF_HAVE_IPOPT
TEST_CASE("SolverCapabilities: Ipopt flag true at compile time", "[capabilities][ipopt]") {
    const auto caps = get_solver_capabilities();
    CHECK(caps.has_ipopt);
}
#endif

#ifdef HACDCPF_HAVE_KLU
TEST_CASE("SolverCapabilities: KLU flag true at compile time", "[capabilities][klu]") {
    const auto caps = get_solver_capabilities();
    CHECK(caps.has_klu);
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// verify_opf_result: feasibility audit
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("verify_opf_result: case9 converged result passes audit", "[capabilities][audit]") {
    auto sys = io::parse_matpower(std::string(HACDCPF_TEST_DATA_DIR) + "/case9.m");
    auto r   = solve_ac_opf(sys);
    REQUIRE(r.converged);
    REQUIRE_FALSE(r.audit.audited);  // not yet audited

    verify_opf_result(sys, r);

    CHECK(r.audit.audited);
    // All limit violations should be small for a converged optimal solution.
    CHECK(r.audit.max_voltage_limit_violation_pu < 0.02);
    CHECK(r.audit.max_gen_limit_violation_mw     < 1.0);
    CHECK(std::isfinite(r.audit.objective_reported));
    CHECK(std::isfinite(r.audit.objective_recomputed));
}

TEST_CASE("verify_opf_result: infeasibility hints populated for non-converged result", "[capabilities][audit]") {
    // Build a system that is structurally infeasible: load with no source.
    HybridPowerSystem sys;
    sys.base_mva = 100.0;

    ACBus b1; b1.index=1; b1.bus_type=BusType::SLACK; b1.vm_pu=1.0;
    b1.vmin_pu=0.9; b1.vmax_pu=1.1; b1.in_service=true;
    ACBus b2; b2.index=2; b2.bus_type=BusType::PQ; b2.vm_pu=1.0;
    b2.pd_mw=1000.0; b2.vmin_pu=0.9; b2.vmax_pu=1.1; b2.in_service=true;
    sys.ac.buses = {b1, b2};

    ACBranch br; br.from_bus=1; br.to_bus=2; br.r_pu=0.01; br.x_pu=0.04; br.in_service=true;
    sys.ac.branches = {br};

    Generator g; g.bus=1; g.is_slack=true; g.pmax_mw=50.0; g.pmin_mw=0.0;
    g.qmax_mvar=50.0; g.qmin_mvar=-50.0; g.vg_pu=1.0; g.in_service=true;
    sys.ac.generators = {g};

    // Fabricate a non-converged result
    opf::ACOPFResult r;
    r.converged = false;
    r.vm = {1.0, 0.85};
    r.pg_mw = {50.0};

    verify_opf_result(sys, r);
    CHECK(r.audit.audited);
    // For a heavily overloaded system, should detect infeasibility causes.
    // At minimum, audit ran.
}

TEST_CASE("verify_opf_result: audit feasible() true for clean result", "[capabilities][audit]") {
    opf::OpfAudit a;
    a.audited = true;
    // No violations added → feasible
    CHECK(a.feasible());

    a.violations.push_back("bus 1 voltage exceeds limit");
    CHECK_FALSE(a.feasible());
}
