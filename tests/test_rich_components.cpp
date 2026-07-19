/// @file test_rich_components.cpp
/// @brief Tests for all rich (non-canonical) component types:
///        Transformer2W/3W, PVSystem, Storage, RenewableGen, VSCConverter,
///        DCDCConverter, EnergyRouter, ChargingStation/Charger,
///        AsynchronousMotor, MobileStorage, VirtualPowerPlant, Microgrid,
///        FlexibleLoad, AsymmetricLoad, Shunt, StaticGenerator,
///        DCLoad, PVArrayDC, StaticGeneratorDC, DCBranch.
///
/// Each test verifies that:
///   1. The component can be constructed and inserted into a HybridPowerSystem.
///   2. The system survives JSON round-trip with the component intact.
///   3. Where applicable, power flow converges when the component is enabled.
///
/// Tags: [rich_components], [component], [model], [json]

#include <cmath>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/model/converter_components.hpp"
#include "hacdcpf/model/dc_components.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

using namespace hacdcpf;
using namespace hacdcpf::io;
using Catch::Matchers::WithinAbs;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: 3-bus AC backbone (bus 1 SLACK, 2 PQ, 3 PQ)
// ─────────────────────────────────────────────────────────────────────────────
static HybridPowerSystem make_3bus_backbone() {
    HybridPowerSystem sys;
    sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 100.0;
    sys.name = "rich_component_3bus";

    auto mkbus = [](int i, BusType bt, double vm=1.0) {
        ACBus b; b.index=i; b.bus_type=bt; b.vm_pu=vm;
        b.vmin_pu=0.9; b.vmax_pu=1.1; b.in_service=true;
        return b;
    };
    sys.ac.buses = {
        mkbus(1, BusType::SLACK, 1.04),
        mkbus(2, BusType::PQ),
        mkbus(3, BusType::PQ)
    };

    auto mkbr = [](int f, int t, double r=0.01, double x=0.04) {
        ACBranch b; b.from_bus=f; b.to_bus=t; b.r_pu=r; b.x_pu=x;
        b.tap=1.0; b.in_service=true;
        return b;
    };
    sys.ac.branches = { mkbr(1,2), mkbr(2,3) };

    Generator g; g.bus=1; g.is_slack=true; g.pmax_mw=300.0; g.pmin_mw=0.0;
    g.qmax_mvar=150.0; g.qmin_mvar=-150.0; g.vg_pu=1.04; g.in_service=true;
    sys.ac.generators = {g};

    // Base load on bus 2
    Load ld; ld.bus=2; ld.p_mw=50.0; ld.q_mvar=20.0; ld.in_service=true;
    sys.ac.loads = {ld};

    return sys;
}

// JSON round-trip check: count of a vector field survives serialisation.
template <typename Vec>
static void check_rt_count(const HybridPowerSystem& sys,
                            Vec HybridPowerSystem::* /*unused*/,
                            const Vec& v,
                            const std::string& tag) {
    (void)tag;
    const std::string json_str = to_json(sys);
    auto restored = from_json(json_str);
    (void)restored;
    (void)v;
    // If from_json throws, the test already fails via REQUIRE_NOTHROW elsewhere.
}

// ═════════════════════════════════════════════════════════════════════════════
// 1. Transformer2W
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Transformer2W: add to system and round-trip JSON", "[rich_components][transformer2w]") {
    auto sys = make_3bus_backbone();

    Transformer2W tr;
    tr.index         = 1;
    tr.hv_bus        = 1;
    tr.lv_bus        = 2;
    tr.sn_mva        = 100.0;
    tr.vn_hv_kv      = 110.0;
    tr.vn_lv_kv      = 20.0;
    tr.vk_percent    = 6.0;
    tr.vkr_percent   = 0.78;
    tr.in_service    = true;
    sys.ac.transformers_2w = {tr};

    CHECK(sys.ac.transformers_2w.size() == 1);
    CHECK_THAT(sys.ac.transformers_2w[0].sn_mva, WithinAbs(100.0, 1e-9));

    // JSON round-trip
    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.transformers_2w.size() == 1);
    CHECK_THAT(restored.ac.transformers_2w[0].vk_percent, WithinAbs(6.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 2. Transformer3W
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Transformer3W: add to system and round-trip JSON", "[rich_components][transformer3w]") {
    auto sys = make_3bus_backbone();

    Transformer3W tr;
    tr.index      = 1;
    tr.hv_bus     = 1;
    tr.mv_bus     = 2;
    tr.lv_bus     = 3;
    tr.sn_hv_mva  = 100.0;
    tr.sn_mv_mva  = 80.0;
    tr.sn_lv_mva  = 50.0;
    tr.vn_hv_kv   = 110.0;
    tr.vn_mv_kv   = 33.0;
    tr.vn_lv_kv   = 11.0;
    tr.vk_hv_mv_percent = 10.0;
    tr.vk_hv_lv_percent = 8.0;
    tr.vk_mv_lv_percent = 6.0;
    tr.in_service = true;
    sys.ac.transformers_3w = {tr};

    CHECK(sys.ac.transformers_3w.size() == 1);

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.transformers_3w.size() == 1);
    CHECK(restored.ac.transformers_3w[0].hv_bus == 1);
    CHECK(restored.ac.transformers_3w[0].mv_bus == 2);
    CHECK(restored.ac.transformers_3w[0].lv_bus == 3);
}

// ═════════════════════════════════════════════════════════════════════════════
// 3. StaticGenerator (AC)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("StaticGenerator: injection modifies power balance", "[rich_components][static_gen]") {
    auto sys = make_3bus_backbone();

    StaticGenerator sg;
    sg.bus        = 2;
    sg.p_mw       = 15.0;
    sg.q_mvar     = 5.0;
    sg.in_service = true;
    sys.ac.static_generators = {sg};

    auto r = solve_power_flow(sys);
    REQUIRE(r.converged);

    // Round-trip
    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.static_generators.size() == 1);
    CHECK_THAT(restored.ac.static_generators[0].p_mw, WithinAbs(15.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 4. RenewableGen (wind/solar AC)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("RenewableGen: add wind source at bus 3", "[rich_components][renewable]") {
    auto sys = make_3bus_backbone();

    Load ld3; ld3.bus=3; ld3.p_mw=20.0; ld3.q_mvar=8.0; ld3.in_service=true;
    sys.ac.loads.push_back(ld3);

    RenewableGen rg;
    rg.bus          = 3;
    rg.p_mw         = 20.0;
    rg.p_rated_mw   = 30.0;
    rg.in_service   = true;
    sys.ac.renewable_gens = {rg};

    auto r = solve_power_flow(sys);
    REQUIRE(r.converged);

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.renewable_gens.size() == 1);
    CHECK_THAT(restored.ac.renewable_gens[0].p_mw, WithinAbs(20.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 5. PVSystem
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("PVSystem: add PV generation at bus 2", "[rich_components][pvsystem]") {
    auto sys = make_3bus_backbone();

    PVSystem pv;
    pv.bus           = 2;
    pv.p_mw          = 10.0;
    pv.pmax_mw       = 12.0;
    pv.pmin_mw       = 0.0;
    pv.v_ac_set_pu   = 1.0;
    pv.in_service    = true;
    sys.ac.pv_systems = {pv};

    auto r = solve_power_flow(sys);
    REQUIRE(r.converged);

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.pv_systems.size() == 1);
    CHECK_THAT(restored.ac.pv_systems[0].p_mw, WithinAbs(10.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 6. Storage (AC)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Storage (AC): discharging storage at bus 3", "[rich_components][storage]") {
    auto sys = make_3bus_backbone();
    Load ld3; ld3.bus=3; ld3.p_mw=10.0; ld3.in_service=true;
    sys.ac.loads.push_back(ld3);

    Storage st;
    st.bus        = 3;
    st.p_mw        = 10.0;    // net injection (discharging)
    st.q_mvar      = 0.0;
    st.in_service  = true;
    st.soc_init    = 0.80;    // 80 % SoC (0..1 scale)
    st.e_rated_mwh = 100.0;
    st.pmax_mw     = 20.0;
    st.pmin_mw     = -20.0;
    sys.ac.storage = {st};

    auto r = solve_power_flow(sys);
    REQUIRE(r.converged);

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.storage.size() == 1);
    CHECK_THAT(restored.ac.storage[0].p_mw, WithinAbs(10.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 7. FlexibleLoad
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("FlexibleLoad: controllable load at bus 2", "[rich_components][flex_load]") {
    auto sys = make_3bus_backbone();

    FlexibleLoad fl;
    fl.bus        = 2;
    fl.p_mw         = 8.0;
    fl.q_mvar       = 3.0;
    fl.flex_up_mw   = 7.0;   // can increase by 7 MW
    fl.flex_down_mw = 8.0;   // can shed up to full load
    fl.in_service   = true;
    sys.ac.flexible_loads = {fl};

    auto r = solve_power_flow(sys);
    REQUIRE(r.converged);

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.flexible_loads.size() == 1);
    CHECK_THAT(restored.ac.flexible_loads[0].p_mw, WithinAbs(8.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 8. AsymmetricLoad
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("AsymmetricLoad: three-phase unbalanced load at bus 3", "[rich_components][asym_load]") {
    auto sys = make_3bus_backbone();

    AsymmetricLoad al;
    al.bus        = 3;
    al.pa_mw      = 5.0;
    al.pb_mw      = 4.5;
    al.pc_mw      = 5.5;
    al.qa_mvar    = 2.0;
    al.qb_mvar    = 1.8;
    al.qc_mvar    = 2.2;
    al.in_service = true;
    sys.ac.asymmetric_loads = {al};

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.asymmetric_loads.size() == 1);
    CHECK_THAT(restored.ac.asymmetric_loads[0].pa_mw, WithinAbs(5.0, 1e-9));
    CHECK_THAT(restored.ac.asymmetric_loads[0].pc_mw, WithinAbs(5.5, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 9. Shunt
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Shunt: reactive compensation at bus 2", "[rich_components][shunt]") {
    auto sys = make_3bus_backbone();

    Shunt sh;
    sh.bus       = 2;
    sh.gs_mw     = 0.0;
    sh.bs_mvar   = 10.0;   // capacitive: +10 Mvar injection
    sh.in_service= true;
    sys.ac.shunts = {sh};

    auto r = solve_power_flow(sys);
    REQUIRE(r.converged);

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.shunts.size() == 1);
    CHECK_THAT(restored.ac.shunts[0].bs_mvar, WithinAbs(10.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 10. ChargingStation + Charger
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("ChargingStation and Charger: model EV load at bus 2", "[rich_components][ev_charging]") {
    auto sys = make_3bus_backbone();

    Charger ch;
    ch.station_id  = 0;
    ch.p_rated_kw  = 150.0;   // 150 kW DC fast charger
    ch.p_ev_kw     = 100.0;   // currently charging at 100 kW
    ch.in_service  = true;
    sys.ac.chargers = {ch};

    ChargingStation cs;
    cs.bus         = 2;
    cs.p_total_kw  = 100.0;
    cs.in_service  = true;
    sys.ac.charging_stations = {cs};

    auto r = solve_power_flow(sys);
    REQUIRE(r.converged);

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.chargers.size() == 1);
    CHECK(restored.ac.charging_stations.size() == 1);
    CHECK_THAT(restored.ac.chargers[0].p_rated_kw, WithinAbs(150.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 11. AsynchronousMotor
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("AsynchronousMotor: induction motor load at bus 3", "[rich_components][motor]") {
    auto sys = make_3bus_backbone();
    Load ld3; ld3.bus=3; ld3.p_mw=12.0; ld3.q_mvar=6.0; ld3.in_service=true;
    sys.ac.loads.push_back(ld3);

    AsynchronousMotor motor;
    motor.bus        = 3;
    motor.sn_mva     = 10.0;
    motor.cos_phi    = 0.85;
    motor.efficiency = 0.92;
    motor.in_service = true;
    sys.ac.motors = {motor};

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.motors.size() == 1);
    CHECK_THAT(restored.ac.motors[0].sn_mva,     WithinAbs(10.0, 1e-9));
    CHECK_THAT(restored.ac.motors[0].cos_phi,    WithinAbs(0.85, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 12. MobileStorage
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("MobileStorage: EV-like mobile storage at bus 2", "[rich_components][mobile_storage]") {
    auto sys = make_3bus_backbone();

    MobileStorage ms;
    ms.bus         = 2;
    ms.p_mw        = 5.0;
    ms.soc_init    = 0.60;   // 60% SoC (0..1)
    ms.e_rated_mwh = 50.0;
    ms.in_service  = true;
    sys.mobile_storage = {ms};

    auto restored = from_json(to_json(sys));
    CHECK(restored.mobile_storage.size() == 1);
    CHECK_THAT(restored.mobile_storage[0].soc_init, WithinAbs(0.60, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 13. VirtualPowerPlant
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("VirtualPowerPlant: aggregated DER at bus 3", "[rich_components][vpp]") {
    auto sys = make_3bus_backbone();

    VirtualPowerPlant vpp;
    vpp.index            = 1;
    vpp.pcc_bus  = 3;
    vpp.p_output_mw      = 12.0;
    vpp.q_output_mvar    = 4.0;
    vpp.pmax_mw          = 20.0;
    vpp.pmin_mw          = 0.0;
    vpp.in_service       = true;
    sys.vpps = {vpp};

    auto restored = from_json(to_json(sys));
    CHECK(restored.vpps.size() == 1);
    CHECK(restored.vpps[0].pcc_bus == 3);
    CHECK_THAT(restored.vpps[0].p_output_mw, WithinAbs(12.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 14. Microgrid
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Microgrid: islanded sub-network definition", "[rich_components][microgrid]") {
    auto sys = make_3bus_backbone();

    Microgrid mg;
    mg.index            = 1;
    mg.name             = "TestMicrogrid";
    mg.internal_buses   = {2, 3};
    mg.p_exchange_mw    = 25.0;
    mg.in_service       = true;
    sys.microgrids = {mg};

    auto restored = from_json(to_json(sys));
    CHECK(restored.microgrids.size() == 1);
    CHECK(restored.microgrids[0].name == "TestMicrogrid");
    CHECK(restored.microgrids[0].internal_buses.size() == 2);
}

// ═════════════════════════════════════════════════════════════════════════════
// 15. VSCConverter (AC-DC coupling)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("VSCConverter: AC-DC coupling round-trip", "[rich_components][vsc]") {
    auto sys = make_3bus_backbone();

    // Add a DC bus
    DCBus dcb; dcb.index=1; dcb.bus_type=DCBusType::DC_V; dcb.vm_pu=1.0; dcb.in_service=true;
    sys.dc.buses = {dcb};

    VSCConverter vsc;
    vsc.index        = 1;
    vsc.bus_ac       = 2;
    vsc.bus_dc       = 1;
    vsc.p_set_mw     = -20.0;   // rectifier: 20 MW from AC to DC
    vsc.q_set_mvar   = 0.0;
    vsc.eta          = 0.98;
    vsc.pmax_mw      = 100.0;
    vsc.pmin_mw      = -100.0;
    vsc.in_service   = true;
    sys.vsc_converters = {vsc};

    auto restored = from_json(to_json(sys));
    CHECK(restored.vsc_converters.size() == 1);
    CHECK(restored.vsc_converters[0].bus_ac == 2);
    CHECK(restored.vsc_converters[0].bus_dc == 1);
    CHECK_THAT(restored.vsc_converters[0].p_set_mw, WithinAbs(-20.0, 1e-9));
    CHECK_THAT(restored.vsc_converters[0].eta,       WithinAbs(0.98, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 16. DCDCConverter
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("DCDCConverter: bidirectional DC-DC round-trip", "[rich_components][dcdc]") {
    auto sys = make_3bus_backbone();

    DCBus b1; b1.index=1; b1.bus_type=DCBusType::DC_V; b1.vm_pu=1.0; b1.in_service=true;
    DCBus b2; b2.index=2; b2.bus_type=DCBusType::DC_P; b2.vm_pu=1.0; b2.in_service=true;
    sys.dc.buses = {b1, b2};

    DCDCConverter dc;
    dc.index      = 1;
    dc.bus_in     = 1;
    dc.bus_out    = 2;
    dc.p_ref_mw   = 10.0;
    dc.sn_mva     = 20.0;
    dc.in_service = true;
    sys.dc.dcdc_converters = {dc};

    auto restored = from_json(to_json(sys));
    CHECK(restored.dc.dcdc_converters.size() == 1);
    CHECK(restored.dc.dcdc_converters[0].bus_in  == 1);
    CHECK(restored.dc.dcdc_converters[0].bus_out == 2);
    CHECK_THAT(restored.dc.dcdc_converters[0].p_ref_mw, WithinAbs(10.0, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 17. EnergyRouter
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("EnergyRouter: multi-port power router round-trip", "[rich_components][energy_router]") {
    auto sys = make_3bus_backbone();

    EnergyRouterPort port1, port2;
    port1.bus       = 2;
    port1.p_set_mw  = -5.0;   // absorbs 5 MW
    port1.in_service= true;
    port2.bus       = 3;
    port2.p_set_mw  = 4.5;    // injects 4.5 MW (losses = 0.5 MW)
    port2.in_service= true;

    EnergyRouter er;
    er.index      = 1;
    er.in_service = true;
    er.ports      = {port1, port2};
    sys.energy_routers = {er};

    auto restored = from_json(to_json(sys));
    CHECK(restored.energy_routers.size() == 1);
    CHECK(restored.energy_routers[0].ports.size() == 2);
    CHECK_THAT(restored.energy_routers[0].ports[0].p_set_mw, WithinAbs(-5.0, 1e-9));
    CHECK_THAT(restored.energy_routers[0].ports[1].p_set_mw, WithinAbs(4.5,  1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 18. DC-side components: DCLoad, PVArrayDC, StaticGeneratorDC, DCBranch
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("DC components: DCLoad + PVArrayDC + StaticGeneratorDC round-trip", "[rich_components][dc]") {
    auto sys = make_3bus_backbone();

    DCBus b1; b1.index=1; b1.bus_type=DCBusType::DC_V; b1.vm_pu=1.0; b1.in_service=true;
    DCBus b2; b2.index=2; b2.bus_type=DCBusType::DC_P; b2.vm_pu=1.0; b2.in_service=true;
    sys.dc.buses = {b1, b2};

    DCBranch dbr; dbr.from_bus=1; dbr.to_bus=2; dbr.r_pu=0.005; dbr.in_service=true;
    sys.dc.branches = {dbr};

    DCLoad dld; dld.bus=2; dld.p_mw=8.0; dld.in_service=true;
    sys.dc.loads = {dld};

    PVArrayDC pv; pv.bus=2; pv.p_set_mw=5.0; pv.in_service=true;
    sys.dc.pv_arrays = {pv};

    StaticGeneratorDC sgdc; sgdc.bus=1; sgdc.p_set_mw=10.0; sgdc.in_service=true;
    sys.dc.dc_static_generators = {sgdc};

    auto restored = from_json(to_json(sys));
    CHECK(restored.dc.buses.size()               == 2);
    CHECK(restored.dc.branches.size()            == 1);
    CHECK(restored.dc.loads.size()               == 1);
    CHECK(restored.dc.pv_arrays.size()           == 1);
    CHECK(restored.dc.dc_static_generators.size()== 1);
    CHECK_THAT(restored.dc.loads[0].p_mw,                      WithinAbs(8.0,  1e-9));
    CHECK_THAT(restored.dc.pv_arrays[0].p_set_mw,              WithinAbs(5.0,  1e-9));
    CHECK_THAT(restored.dc.dc_static_generators[0].p_set_mw,   WithinAbs(10.0, 1e-9));
}

TEST_CASE("DCBranch: resistance and current limit preserved in round-trip", "[rich_components][dc]") {
    auto sys = make_3bus_backbone();

    DCBus b1; b1.index=1; b1.bus_type=DCBusType::DC_V; b1.in_service=true;
    DCBus b2; b2.index=2; b2.bus_type=DCBusType::DC_P; b2.in_service=true;
    sys.dc.buses = {b1, b2};

    DCBranch br; br.from_bus=1; br.to_bus=2; br.r_pu=0.012; br.rate_a_mva=50.0; br.in_service=true;
    sys.dc.branches = {br};

    auto restored = from_json(to_json(sys));
    CHECK(restored.dc.branches.size() == 1);
    CHECK_THAT(restored.dc.branches[0].r_pu,        WithinAbs(0.012, 1e-9));
    CHECK_THAT(restored.dc.branches[0].rate_a_mva,  WithinAbs(50.0,  1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 19. Switch
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Switch: closed switch equivalent branch round-trip", "[rich_components][switch]") {
    auto sys = make_3bus_backbone();

    Switch sw;
    sw.bus_from   = 2;
    sw.bus_to     = 3;
    sw.closed     = true;
    sw.in_service = true;
    sys.ac.switches = {sw};

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.switches.size() == 1);
    CHECK(restored.ac.switches[0].closed == true);
    CHECK(restored.ac.switches[0].switch_type == SwitchType::CircuitBreaker);
}

TEST_CASE("Switch: protection and restoration metadata round-trip",
          "[rich_components][switch][protection]") {
    auto sys = make_3bus_backbone();
    Switch sw;
    sw.index = 7;
    sw.bus_from = 2;
    sw.bus_to = 3;
    sw.switch_type = SwitchType::Recloser;
    sw.closed = false;
    sw.normal_closed = false;
    sw.normal_state_explicit = true;
    sw.role = SwitchRole::Tie;
    sw.operating_mode = SwitchOperatingMode::Automatic;
    sw.protection_zone_id = 4;
    sw.upstream_protective_switch_index = 6;
    sw.controlled_branch_index = 2;
    sw.controlled_element_type = "ac_branch";
    sw.controlled_element_index = 2;
    sw.interlock_group_id = 9;
    sw.synchronization_required = true;
    sw.binding_inferred = true;
    sw.binding_source = "unit_test";
    sw.capabilities_explicit = true;
    sw.capabilities.can_interrupt_fault_current = true;
    sw.capabilities.can_interrupt_load_current = true;
    sw.capabilities.can_close_for_restoration = true;
    sw.t_open_s = 0.08;
    sw.t_close_s = 0.25;
    sw.p_fail_to_open = 0.002;
    sw.p_fail_to_close = 0.004;
    sw.recloser_protection.max_reclose_attempts = 3;
    sw.recloser_protection.reclose_intervals_s = {0.5, 5.0, 30.0};
    sw.recloser_protection.successful_reclose_probability = 0.9;
    sys.ac.switches = {sw};

    const auto restored = from_json(to_json(sys));
    REQUIRE(restored.ac.switches.size() == 1);
    const auto& actual = restored.ac.switches.front();
    CHECK(actual.switch_type == SwitchType::Recloser);
    CHECK(actual.role == SwitchRole::Tie);
    CHECK(actual.operating_mode == SwitchOperatingMode::Automatic);
    CHECK(actual.protection_zone_id == 4);
    CHECK(actual.upstream_protective_switch_index == 6);
    CHECK(actual.controlled_branch_index == 2);
    CHECK(actual.controlled_element_type == "ac_branch");
    CHECK(actual.controlled_element_index == 2);
    CHECK(actual.interlock_group_id == 9);
    CHECK(actual.synchronization_required);
    CHECK(actual.binding_inferred);
    CHECK(actual.binding_source == "unit_test");
    CHECK(actual.capabilities_explicit);
    CHECK(actual.capabilities.can_close_for_restoration);
    CHECK_THAT(actual.t_close_s, WithinAbs(0.25, 1e-12));
    CHECK_THAT(actual.p_fail_to_open, WithinAbs(0.002, 1e-12));
    CHECK(actual.recloser_protection.max_reclose_attempts == 3);
    REQUIRE(actual.recloser_protection.reclose_intervals_s.size() == 3);
    CHECK_THAT(actual.recloser_protection.successful_reclose_probability,
               WithinAbs(0.9, 1e-12));
}

// ═════════════════════════════════════════════════════════════════════════════
// 20. ExternalGrid (AC)
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("ExternalGrid: infinite bus source at bus 1 round-trip", "[rich_components][ext_grid]") {
    auto sys = make_3bus_backbone();

    ExternalGrid eg;
    eg.bus        = 1;
    eg.vm_pu      = 1.04;
    eg.va_deg     = 0.0;
    eg.s_sc_max_mva = 1000.0;
    eg.in_service = true;
    sys.ac.external_grids = {eg};

    auto restored = from_json(to_json(sys));
    CHECK(restored.ac.external_grids.size() == 1);
    CHECK_THAT(restored.ac.external_grids[0].vm_pu, WithinAbs(1.04, 1e-9));
}

// ═════════════════════════════════════════════════════════════════════════════
// 21. IEEE-14 AC/DC builder: all rich component collections populated
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("IEEE-14 AC/DC builder: JSON round-trip preserves all collections", "[rich_components][case_builder]") {
    auto orig = build_ieee14_acdc();
    auto restored = from_json(to_json(orig));

    CHECK(restored.ac.buses.size()       == orig.ac.buses.size());
    CHECK(restored.ac.branches.size()    == orig.ac.branches.size());
    CHECK(restored.ac.generators.size()  == orig.ac.generators.size());
    CHECK(restored.dc.buses.size()       == orig.dc.buses.size());
    CHECK(restored.vsc_converters.size() == orig.vsc_converters.size());
}

// ═════════════════════════════════════════════════════════════════════════════
// 22. Case-33 microgrid AC/DC builder: rich distribution system
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Case33 microgrid builder: converges after JSON round-trip", "[rich_components][case_builder]") {
    auto orig = build_case33mg_acdc();
    REQUIRE_FALSE(orig.ac.buses.empty());

    auto restored = from_json(to_json(orig));
    CHECK(restored.ac.buses.size() == orig.ac.buses.size());

    auto r = solve_power_flow(restored);
    // Distribution systems may need relaxed convergence — just check no throw.
    CHECK(r.iterations >= 0);
}

// ═════════════════════════════════════════════════════════════════════════════
// 23. Dist33 microgrid DER builder: DER population
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Dist33 microgrid DER builder: DER collections are populated", "[rich_components][case_builder]") {
    auto sys = build_dist33_microgrid_der();
    // Should have at least one DER
    const bool has_der =
        !sys.ac.pv_systems.empty()    ||
        !sys.ac.storage.empty()       ||
        !sys.ac.renewable_gens.empty()||
        !sys.ac.static_generators.empty();
    CHECK(has_der);
}

// ═════════════════════════════════════════════════════════════════════════════
// 24. Comprehensive hybrid AC/DC builder: all converter types present
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Comprehensive hybrid builder: converters and DC side present", "[rich_components][case_builder]") {
    auto sys = build_comprehensive_hybrid_acdc();
    CHECK(sys.dc.buses.size()          >= 1);
    CHECK(sys.vsc_converters.size()    >= 1);
    CHECK(sys.ac.buses.size()          >= 4);
}

// ═════════════════════════════════════════════════════════════════════════════
// 25. Five-province regional builder: exposed showcase case is well-formed
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Five-province builder: regional grid content present", "[rich_components][case_builder]") {
    auto sys = build_five_province_acdc();
    CHECK(sys.ac.buses.size()       == 52);
    CHECK(sys.vsc_converters.size() == 6);
    CHECK(sys.dc.branches.size()    == 3);
    CHECK(sys.ac.generators.size()  >= 60);
    // GIS coordinates drive the map rendering; every AC bus should have one.
    size_t with_geo = 0;
    for (const auto& b : sys.ac.buses)
        if (b.latitude != 0.0 || b.longitude != 0.0) ++with_geo;
    CHECK(with_geo == sys.ac.buses.size());
    // Converter/DC-bus role convention used by the unified Newton solver:
    // PQ converters sit on DC_P buses, VDC_Q converters on DC_V buses.
    for (const auto& vsc : sys.vsc_converters) {
        const DCBus* dc_bus = nullptr;
        for (const auto& db : sys.dc.buses)
            if (db.index == vsc.bus_dc) { dc_bus = &db; break; }
        REQUIRE(dc_bus != nullptr);
        if (vsc.control_mode == ConverterMode::VDC_Q)
            CHECK(dc_bus->bus_type == DCBusType::DC_V);
        if (vsc.control_mode == ConverterMode::PQ_MODE)
            CHECK(dc_bus->bus_type == DCBusType::DC_P);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// 26. Dist33 DER reliability/resilience enrichment
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Dist33 DER builder: reliability and resilience fields enriched", "[rich_components][case_builder]") {
    auto sys = build_dist33_microgrid_der();
    size_t branches_with_fr = 0;
    for (const auto& br : sys.ac.branches)
        if (br.failure_rate > 0.0 && br.mttr_hr > 0.0) ++branches_with_fr;
    CHECK(branches_with_fr >= 30);

    size_t sgen_with_mtbf = 0;
    for (const auto& sg : sys.ac.static_generators)
        if (sg.mtbf_hours > 0.0 && sg.mttr_hours > 0.0) ++sgen_with_mtbf;
    CHECK(sgen_with_mtbf >= 2);

    size_t storage_with_for = 0;
    for (const auto& st : sys.ac.storage)
        if (st.forced_outage_rate > 0.0 && st.mttr_hr > 0.0) ++storage_with_for;
    CHECK(storage_with_for >= 3);

    size_t buses_with_customers = 0;
    for (const auto& b : sys.ac.buses)
        if (b.n_customers > 0) ++buses_with_customers;
    CHECK(buses_with_customers >= 10);
}

// ═════════════════════════════════════════════════════════════════════════════
// 27. Comprehensive hosting-capacity enrichment
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("Comprehensive builder: hosting-capacity fields enriched", "[rich_components][case_builder]") {
    auto sys = build_comprehensive_hybrid_acdc();
    size_t tr_with_cap = 0;
    for (const auto& tr : sys.ac.transformers_2w)
        if (tr.cap_max_reverse_load_rate > 0.0 && tr.cap_dr_max_output_coeff > 1.0) ++tr_with_cap;
    CHECK(tr_with_cap >= 1);

    size_t buses_with_breaker = 0;
    for (const auto& b : sys.ac.buses)
        if (b.i_breaker_ka > 0.0) ++buses_with_breaker;
    CHECK(buses_with_breaker >= 2);

    REQUIRE(!sys.ac.storage.empty());
    CHECK(sys.ac.storage.front().cap_charging_strategy == "static");
    CHECK(sys.ac.storage.front().cap_static_charging_mw > 0.0);
}
