/// @file test_unit_conversion.cpp
/// @brief Tests for convert_actual_to_per_unit — the engineering "actual value"
///        (ohm/km, kV) to per-unit bridge for AC and DC branches.
///
/// Tags: [unit_conversion], [actual_values], [model]

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/model/unit_conversion.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

using namespace hacdcpf;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ─────────────────────────────────────────────────────────────────────────────
// AC branch: ohm/km → per-unit
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("convert_actual_to_per_unit: AC branch ohm/km maps to pu",
          "[unit_conversion][actual_values]") {
    HybridPowerSystem sys;
    sys.base_mva = sys.ac.base_mva = 100.0;

    ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0; b1.in_service = true;
    ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ;    b2.base_kv = 110.0; b2.in_service = true;
    sys.ac.buses = {b1, b2};

    // z_base = 110^2 / 100 = 121 ohm.
    ACBranch br;
    br.index = 1; br.from_bus = 1; br.to_bus = 2;
    br.r_pu = 0.0; br.x_pu = 0.0; br.b_pu = 0.0;
    br.r_ohm_per_km = 0.1;   // total 1.0 ohm over 10 km
    br.x_ohm_per_km = 0.4;   // total 4.0 ohm over 10 km
    br.b_us_per_km  = 3.0;   // total 3e-5 S over 10 km
    br.length_km    = 10.0;
    br.n_parallel   = 1;
    sys.ac.branches = {br};

    const int n = convert_actual_to_per_unit(sys);
    REQUIRE(n == 1);

    const double z_base = 121.0;
    CHECK_THAT(sys.ac.branches[0].r_pu, WithinRel(1.0 / z_base, 1e-9));
    CHECK_THAT(sys.ac.branches[0].x_pu, WithinRel(4.0 / z_base, 1e-9));
    CHECK_THAT(sys.ac.branches[0].b_pu, WithinRel(3.0e-5 * z_base, 1e-9));
}

// ─────────────────────────────────────────────────────────────────────────────
// AC branch: parallel circuits halve the series impedance
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("convert_actual_to_per_unit: parallel circuits divide impedance",
          "[unit_conversion][actual_values]") {
    HybridPowerSystem sys;
    sys.base_mva = sys.ac.base_mva = 100.0;

    ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0;
    ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ;    b2.base_kv = 110.0;
    sys.ac.buses = {b1, b2};

    ACBranch br;
    br.index = 1; br.from_bus = 1; br.to_bus = 2;
    br.r_ohm_per_km = 0.1; br.x_ohm_per_km = 0.4; br.length_km = 10.0;
    br.n_parallel = 2;  // two identical circuits → half the impedance
    sys.ac.branches = {br};

    convert_actual_to_per_unit(sys);
    const double z_base = 121.0;
    CHECK_THAT(sys.ac.branches[0].r_pu, WithinRel(0.5 / z_base, 1e-9));
    CHECK_THAT(sys.ac.branches[0].x_pu, WithinRel(2.0 / z_base, 1e-9));
}

// ─────────────────────────────────────────────────────────────────────────────
// DC branch: ohm/km → per-unit
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("convert_actual_to_per_unit: DC branch ohm/km maps to pu",
          "[unit_conversion][actual_values][dc]") {
    HybridPowerSystem sys;
    sys.base_mva = sys.dc.base_mva = 100.0;

    DCBus d1; d1.index = 1; d1.bus_type = DCBusType::DC_V; d1.base_kv = 100.0; d1.in_service = true;
    DCBus d2; d2.index = 2; d2.bus_type = DCBusType::DC_P; d2.base_kv = 100.0; d2.in_service = true;
    sys.dc.buses = {d1, d2};

    // z_base = 100^2 / 100 = 100 ohm.
    DCBranch br;
    br.index = 1; br.from_bus = 1; br.to_bus = 2;
    br.r_pu = 0.0; br.r_ohm_per_km = 0.05; br.length_km = 4.0;  // total 0.2 ohm
    sys.dc.branches = {br};

    const int n = convert_actual_to_per_unit(sys);
    REQUIRE(n == 1);
    CHECK_THAT(sys.dc.branches[0].r_pu, WithinRel(0.2 / 100.0, 1e-9));
}

// ─────────────────────────────────────────────────────────────────────────────
// Non-destructive: a branch already given in per-unit is left untouched
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("convert_actual_to_per_unit: existing per-unit data is preserved",
          "[unit_conversion][actual_values]") {
    HybridPowerSystem sys;
    sys.base_mva = sys.ac.base_mva = 100.0;

    ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0;
    ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ;    b2.base_kv = 110.0;
    sys.ac.buses = {b1, b2};

    ACBranch br;
    br.index = 1; br.from_bus = 1; br.to_bus = 2;
    br.r_pu = 0.01; br.x_pu = 0.05;          // already per-unit
    br.r_ohm_per_km = 0.1; br.x_ohm_per_km = 0.4; br.length_km = 10.0;  // also has actual
    sys.ac.branches = {br};

    const int n = convert_actual_to_per_unit(sys);
    CHECK(n == 0);  // nothing converted
    CHECK_THAT(sys.ac.branches[0].r_pu, WithinAbs(0.01, 1e-12));
    CHECK_THAT(sys.ac.branches[0].x_pu, WithinAbs(0.05, 1e-12));

    // Idempotent: a second call is also a no-op.
    CHECK(convert_actual_to_per_unit(sys) == 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// End-to-end: an actual-value system solves identically to its per-unit twin
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("Power flow: actual-value branch matches equivalent per-unit branch",
          "[unit_conversion][actual_values][power_flow]") {
    auto make_2bus = [](bool actual) {
        HybridPowerSystem sys;
        sys.base_mva = sys.ac.base_mva = 100.0;

        ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0;
        b1.vm_pu = 1.0; b1.vmin_pu = 0.9; b1.vmax_pu = 1.1; b1.in_service = true;
        ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ; b2.base_kv = 110.0;
        b2.vm_pu = 1.0; b2.pd_mw = 20.0; b2.qd_mvar = 5.0;
        b2.vmin_pu = 0.9; b2.vmax_pu = 1.1; b2.in_service = true;
        sys.ac.buses = {b1, b2};

        ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2; br.in_service = true;
        if (actual) {
            // z_base = 121 ohm; choose actual values equal to the pu twin below.
            br.r_ohm_per_km = 0.1; br.x_ohm_per_km = 0.4; br.length_km = 10.0;
        } else {
            br.r_pu = 1.0 / 121.0;   // 1.0 ohm / 121
            br.x_pu = 4.0 / 121.0;   // 4.0 ohm / 121
        }
        sys.ac.branches = {br};
        return sys;
    };

    PowerFlowOptions opt;
    auto res_pu     = solve_power_flow(make_2bus(false), opt);
    auto res_actual = solve_power_flow(make_2bus(true), opt);

    REQUIRE(res_pu.converged);
    REQUIRE(res_actual.converged);
    REQUIRE(res_pu.vm.size() == res_actual.vm.size());
    for (size_t i = 0; i < res_pu.vm.size(); ++i) {
        CHECK_THAT(res_actual.vm[i], WithinAbs(res_pu.vm[i], 1e-7));
        CHECK_THAT(res_actual.va[i], WithinAbs(res_pu.va[i], 1e-7));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ETAP-style example case builder: actual values all the way to a solved PF
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("build_actual_value_demo_acdc: actual-value case converts and solves",
          "[unit_conversion][actual_values][case_builder]") {
    auto sys = hacdcpf::io::build_actual_value_demo_acdc();

    // As authored, the AC cables and DC link carry only actual values.
    REQUIRE(sys.ac.branches.size() == 3);
    for (const auto& br : sys.ac.branches) {
        CHECK(br.r_pu == 0.0);
        CHECK(br.x_pu == 0.0);
        CHECK(br.r_ohm_per_km > 0.0);
        CHECK(br.length_km > 0.0);
    }
    REQUIRE(sys.dc.branches.size() == 1);
    CHECK(sys.dc.branches[0].r_pu == 0.0);
    CHECK(sys.dc.branches[0].r_ohm_per_km > 0.0);

    // The conversion fills every branch (3 AC + 1 DC).
    auto copy = sys;
    const int n = convert_actual_to_per_unit(copy);
    CHECK(n == 4);
    for (const auto& br : copy.ac.branches) {
        CHECK(br.r_pu > 0.0);
        CHECK(br.x_pu > 0.0);
    }
    CHECK(copy.dc.branches[0].r_pu > 0.0);

    // End to end: the as-authored actual-value system solves (projection runs
    // convert_actual_to_per_unit internally).
    PowerFlowOptions opt;
    auto res = solve_power_flow(sys, opt);
    CHECK(res.converged);
}

// ─────────────────────────────────────────────────────────────────────────
// Transformer2W nameplate (vk%) → per-unit on projection (already actual-value)
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("Transformer2W nameplate vk% maps to per-unit ACBranch on projection",
          "[unit_conversion][actual_values][transformer]") {
    HybridPowerSystem sys;
    sys.base_mva = sys.ac.base_mva = 100.0;

    ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0; b1.in_service = true;
    ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ;    b2.base_kv = 11.0;  b2.in_service = true;
    sys.ac.buses = {b1, b2};

    // Nameplate (actual) transformer data only — no per-unit impedance.
    Transformer2W tr;
    tr.index = 1; tr.hv_bus = 1; tr.lv_bus = 2;
    tr.sn_mva = 25.0; tr.vk_percent = 6.0; tr.vkr_percent = 0.9;
    tr.vn_hv_kv = 110.0; tr.vn_lv_kv = 11.0; tr.in_service = true; tr.name = "T1";
    sys.ac.transformers_2w = {tr};

    const auto proj = project_to_canonical_models(sys);

    const ACBranch* eq = nullptr;
    for (const auto& br : proj.ac.branches)
        if (br.from_bus == 1 && br.to_bus == 2) { eq = &br; break; }
    REQUIRE(eq != nullptr);

    // scale = base/sn = 4; z = vk% * scale; r = vkr% * scale; x = sqrt(z^2 - r^2).
    const double scale = 100.0 / 25.0;
    const double z = 0.06 * scale;
    const double r = 0.009 * scale;
    const double x = std::sqrt(z * z - r * r);
    CHECK_THAT(eq->r_pu, WithinRel(r, 1e-9));
    CHECK_THAT(eq->x_pu, WithinRel(x, 1e-9));
}
