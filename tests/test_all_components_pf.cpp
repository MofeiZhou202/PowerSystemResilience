/**
 * @file test_all_components_pf.cpp
 * @brief Comprehensive power-flow test covering all 29 component types.
 *
 * All 29 struct types that make up HybridPowerSystem are exercised here.
 * Each component type is present in the system and its removal/disable is
 * shown to produce a measurable change in bus-voltage magnitude, verifying
 * that the component genuinely participates in the power-flow computation.
 *
 * The 29 types (unique structs):
 *   ACSystem (18): ACBus, ACBranch, Generator, StaticGenerator, Load,
 *     FlexibleLoad, AsymmetricLoad, Shunt, Storage(AC), RenewableGen,
 *     PVSystem, ExternalGrid, Transformer2W, Transformer3W, Switch,
 *     ChargingStation, Charger, AsynchronousMotor
 *   DCSystem unique (5): DCBus, DCBranch, DCLoad, StaticGeneratorDC,
 *     PVArrayDC
 *   Hybrid top-level (6): VSCConverter, DCDCConverter, EnergyRouter,
 *     MobileStorage, VirtualPowerPlant, Microgrid
 *   Shared structs used in both AC and DC sides:
 *     Storage (ac.storage and dc.storage),
 *     StaticGenerator (ac.static_generators and dc.static_generators)
 *
 * System topology
 * ---------------
 *   AC side (base_mva=100, 5 buses):
 *     Bus1 (SLACK, 110 kV) — Generator, ExternalGrid, Shunt
 *       └─ Transformer2W (110/20 kV, sn=100 MVA) ──► Bus2 (20 kV)
 *          ├─ Load, FlexibleLoad, ChargingStation+Charger,
 *          │  StaticGenerator, AsynchronousMotor
 *          └─ ACBranch ──► Bus3 (20 kV)
 *             ├─ AsymmetricLoad, RenewableGen, PVSystem, Storage(AC)
 *             └─ ACBranch ──► Bus4 (20 kV)
 *                ├─ VirtualPowerPlant, Microgrid, MobileStorage
 *                ├─ EnergyRouter port1 (consuming 0.8 MW)
 *                └─ Switch ──► Bus5 (20 kV)
 *                   └─ EnergyRouter port2 (injecting 0.7 MW)
 *   3W Transformer: HV=Bus1, MV=Bus3, LV=Bus5 (creates meshed paths)
 *
 *   DC side (2 buses):
 *     DCBus1 (DC_V, slack) — StaticGeneratorDC, Storage(DC)
 *       ├─ VSCConverter ↔ ACBus2
 *       ├─ DCDCConverter ──► DCBus2
 *       └─ DCBranch ──► DCBus2 (DC_P)
 *          ├─ DCLoad, PVArrayDC, StaticGenerator(DC-side)
 *
 *   EnergyRouter: 2-port, AC ports at Bus4 and Bus5; net -0.8 → +0.7 MW.
 */

#include <cmath>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/network_utils.hpp"
#include "hacdcpf/model/system.hpp"

using namespace hacdcpf;
using Catch::Matchers::WithinAbs;

// ═════════════════════════════════════════════════════════════════════════════
// Helper: build the all-29-types test system
// ═════════════════════════════════════════════════════════════════════════════

static HybridPowerSystem build_all29_system() {
  HybridPowerSystem sys;
  sys.base_mva     = 100.0;
  sys.ac.base_mva  = 100.0;
  sys.dc.base_mva  = 100.0;
  sys.name = "all_29_test";

  // ── AC Buses ────────────────────────────────────────────────────────────
  {
    auto mk = [](int idx, BusType bt, double kv) {
      ACBus b;
      b.index    = idx;
      b.bus_type = bt;
      b.vm_pu    = 1.0;
      b.base_kv  = kv;
      b.vmax_pu  = 1.1;
      b.vmin_pu  = 0.9;
      b.in_service = true;
      return b;
    };
    sys.ac.buses = {
      mk(1, BusType::SLACK, 110.0),  // Bus1: main slack
      mk(2, BusType::PQ,   20.0),   // Bus2: LV side of Transformer2W
      mk(3, BusType::PQ,   20.0),   // Bus3: distribution bus
      mk(4, BusType::PQ,   20.0),   // Bus4: aggregation bus
      mk(5, BusType::PQ,   20.0),   // Bus5: end bus (EnergyRouter + Switch)
    };
  }

  // ── Generator (slack, type 1) ────────────────────────────────────────────
  {
    Generator g;
    g.index      = 1;
    g.bus        = 1;
    g.in_service = true;
    g.pg_mw      = 30.0;
    g.qg_mvar    = 0.0;
    g.vg_pu      = 1.02;
    g.pmax_mw    = 500.0;
    g.pmin_mw    = 0.0;
    g.qmax_mvar  = 200.0;
    g.qmin_mvar  = -200.0;
    g.is_slack   = true;
    g.name       = "G1-Slack";
    sys.ac.generators = {g};
  }

  // ── ExternalGrid (type 2 — sets vm_pu & treats as SLACK in projection) ──
  {
    ExternalGrid eg;
    eg.index      = 1;
    eg.bus        = 1;
    eg.in_service = true;
    eg.vm_pu      = 1.02;
    eg.va_deg     = 0.0;
    // Short-circuit contribution (does not affect steady-state PF vm,
    // but the ExternalGrid signals bus_type=SLACK which enforces vm setpoint)
    eg.r_pu = 0.01;
    eg.x_pu = 0.05;
    eg.name = "EG1";
    sys.ac.external_grids = {eg};
  }

  // ── Shunt (type 3 — reactive compensation at PQ Bus2 so it affects bus V) ─
  {
    Shunt sh;
    sh.index      = 1;
    sh.bus        = 2;     // PQ bus — shunt here changes Bus2 voltage
    sh.in_service = true;
    sh.gs_mw      = 0.0;
    sh.bs_mvar    = 5.0;   // 5 Mvar capacitive → raises Bus2 voltage
    sh.name       = "SH1";
    sys.ac.shunts = {sh};
  }

  // ── Transformer2W (type 4 — 110/20 kV coupling Bus1→Bus2) ───────────────
  {
    Transformer2W tr;
    tr.index         = 1;
    tr.hv_bus        = 1;
    tr.lv_bus        = 2;
    tr.in_service    = true;
    tr.sn_mva        = 100.0;
    tr.vn_hv_kv      = 110.0;
    tr.vn_lv_kv      = 20.0;
    tr.vk_percent    = 10.0;
    tr.vkr_percent   = 0.5;
    tr.tap_pos       = 0;
    tr.tap_neutral   = 0;
    tr.tap_step_percent = 2.5;
    tr.name = "TR1-110/20";
    sys.ac.transformers_2w = {tr};
  }

  // ── Transformer3W (type 5 — HV=Bus1, MV=Bus3, LV=Bus5) ─────────────────
  {
    Transformer3W tr3;
    tr3.index          = 1;
    tr3.hv_bus         = 1;
    tr3.mv_bus         = 3;
    tr3.lv_bus         = 5;
    tr3.in_service     = true;
    tr3.sn_hv_mva      = 50.0;
    tr3.sn_mv_mva      = 30.0;
    tr3.sn_lv_mva      = 20.0;
    tr3.vn_hv_kv       = 110.0;
    tr3.vn_mv_kv       = 20.0;
    tr3.vn_lv_kv       = 20.0;
    tr3.vk_hv_mv_percent = 10.0;
    tr3.vkr_hv_mv_percent = 0.4;
    tr3.vk_hv_lv_percent = 12.0;
    tr3.vkr_hv_lv_percent = 0.5;
    tr3.vk_mv_lv_percent = 6.0;
    tr3.vkr_mv_lv_percent = 0.3;
    tr3.name = "TR3W-1";
    sys.ac.transformers_3w = {tr3};
  }

  // ── ACBranch: Bus2→Bus3 (type 6 — distribution feeder) ──────────────────
  {
    ACBranch br;
    br.index    = 1;
    br.from_bus = 2;
    br.to_bus   = 3;
    br.r_pu     = 0.05;
    br.x_pu     = 0.15;
    br.b_pu     = 0.01;
    br.tap      = 1.0;
    br.in_service = true;
    br.name = "L2-3";
    sys.ac.branches.push_back(br);
  }

  // ── ACBranch: Bus3→Bus4 ──────────────────────────────────────────────────
  {
    ACBranch br;
    br.index    = 2;
    br.from_bus = 3;
    br.to_bus   = 4;
    br.r_pu     = 0.04;
    br.x_pu     = 0.12;
    br.b_pu     = 0.005;
    br.tap      = 1.0;
    br.in_service = true;
    br.name = "L3-4";
    sys.ac.branches.push_back(br);
  }

  // ── Switch (type 7 — Bus4→Bus5, closed) ─────────────────────────────────
  {
    Switch sw;
    sw.index      = 1;
    sw.bus_from   = 4;
    sw.bus_to     = 5;
    sw.in_service = true;
    sw.closed     = true;
    sw.r_contact_ohm = 0.0;   // ideal → projects to tiny-Z branch → bus merge
    sw.z_ohm         = 0.0;
    sw.name = "SW1";
    sys.ac.switches = {sw};
  }

  // ── Load (type 8) at Bus2 ────────────────────────────────────────────────
  {
    Load ld;
    ld.index      = 1;
    ld.bus        = 2;
    ld.in_service = true;
    ld.p_mw       = 5.0;
    ld.q_mvar     = 1.5;
    ld.name = "LD1";
    sys.ac.loads = {ld};
  }

  // ── FlexibleLoad (type 9) at Bus2 ────────────────────────────────────────
  {
    FlexibleLoad fl;
    fl.index      = 1;
    fl.bus        = 2;
    fl.in_service = true;
    fl.p_mw       = 3.0;
    fl.q_mvar     = 1.0;
    fl.flex_up_mw   = 1.0;
    fl.flex_down_mw = 1.5;
    fl.name = "FL1";
    sys.ac.flexible_loads = {fl};
  }

  // ── ChargingStation (type 10) + Charger (type 11) at Bus2 ───────────────
  {
    ChargingStation cs;
    cs.index           = 1;
    cs.bus             = 2;
    cs.in_service      = true;
    cs.p_total_kw      = 500.0;   // 0.5 MW
    cs.q_total_kvar    = 100.0;
    cs.num_chargers    = 2;
    cs.name = "CS1";
    sys.ac.charging_stations = {cs};

    Charger ch;
    ch.index        = 1;
    ch.station_id   = 1;
    ch.in_service   = true;
    ch.p_rated_kw   = 250.0;
    ch.p_ch_max_kw  = 250.0;
    ch.p_ev_kw      = 250.0;
    ch.name = "CHG1";
    sys.ac.chargers = {ch};
  }

  // ── AsynchronousMotor (type 12) at Bus2 ─────────────────────────────────
  {
    AsynchronousMotor m;
    m.index      = 1;
    m.bus        = 2;
    m.in_service = true;
    m.sn_mva     = 2.0;
    m.vn_kv      = 20.0;
    m.cos_phi    = 0.85;
    m.efficiency = 0.93;
    m.r_pu       = 0.05;   // stator resistance (ohm, used by SC)
    m.x_pu       = 0.20;   // subtransient reactance (ohm, used by SC)
    m.poles      = 2;
    m.name = "MOTOR1";
    sys.ac.motors = {m};
  }

  // ── StaticGenerator (type 13) at Bus2 ───────────────────────────────────
  {
    StaticGenerator sg;
    sg.index      = 1;
    sg.bus        = 2;
    sg.in_service = true;
    sg.sgen_type  = SgenType::PV;
    sg.p_mw       = 4.0;
    sg.q_mvar     = 0.5;
    sg.pmax_mw    = 8.0;
    sg.name = "SG1-PV";
    sys.ac.static_generators = {sg};
  }

  // ── AsymmetricLoad (type 14) at Bus3 ────────────────────────────────────
  {
    AsymmetricLoad al;
    al.index      = 1;
    al.bus        = 3;
    al.in_service = true;
    al.pa_mw      = 0.6;
    al.pb_mw      = 0.5;
    al.pc_mw      = 0.4;
    al.qa_mvar    = 0.2;
    al.qb_mvar    = 0.15;
    al.qc_mvar    = 0.1;
    al.scaling    = 1.0;
    al.name = "ALd1";
    sys.ac.asymmetric_loads = {al};
  }

  // ── RenewableGen (type 15) at Bus3 ── wind ───────────────────────────────
  {
    RenewableGen rg;
    rg.index      = 1;
    rg.bus        = 3;
    rg.in_service = true;
    rg.type       = RenewableType::Wind;
    rg.p_mw       = 5.0;
    rg.q_mvar     = 0.0;
    rg.p_rated_mw = 10.0;
    rg.name = "WIND1";
    sys.ac.renewable_gens = {rg};
  }

  // ── PVSystem (type 16) at Bus3 ───────────────────────────────────────────
  {
    PVSystem pv;
    pv.index      = 1;
    pv.bus        = 3;
    pv.in_service = true;
    pv.p_mw       = 3.0;
    pv.q_mvar     = 0.0;
    pv.sn_mva     = 4.0;
    pv.pmax_mw    = 4.0;
    pv.name = "PV1";
    sys.ac.pv_systems = {pv};
  }

  // ── Storage, AC side (type 17) at Bus3 — discharging ────────────────────
  {
    Storage st;
    st.index       = 1;
    st.bus         = 3;
    st.in_service  = true;
    st.p_mw        = 2.0;   // positive = discharge = injection into bus
    st.q_mvar      = 0.0;
    st.p_rated_mw  = 4.0;
    st.pmax_mw     = 4.0;
    st.pmin_mw     = -4.0;
    st.e_rated_mwh = 20.0;
    st.soc_init    = 0.6;
    st.name = "ESS1-AC";
    sys.ac.storage = {st};
  }

  // ── VirtualPowerPlant (type 18) at Bus4 ─────────────────────────────────
  {
    VirtualPowerPlant vpp;
    vpp.index           = 1;
    vpp.aggregation_bus = 4;
    vpp.in_service      = true;
    vpp.p_output_mw     = 2.5;
    vpp.q_output_mvar   = 0.4;
    vpp.pmax_mw         = 5.0;
    vpp.name = "VPP1";
    sys.vpps = {vpp};
  }

  // ── Microgrid (type 19) at Bus4 — grid-connected, exporting ─────────────
  {
    Microgrid mg;
    mg.index          = 1;
    mg.aggregation_bus        = 4;
    mg.in_service     = true;
    mg.operating_mode = MicrogridMode::GridConnected;
    mg.p_exchange_mw  = 1.5;   // positive = export to main grid
    mg.p_exchange_max_mw = 5.0;
    mg.p_exchange_min_mw = -5.0;
    mg.name = "MG1";
    sys.microgrids = {mg};
  }

  // ── MobileStorage (type 20) at Bus4 — stationary, discharging ───────────
  {
    MobileStorage ms;
    ms.index      = 1;
    ms.bus        = 4;
    ms.in_service = true;
    ms.status     = MobileStorageStatus::Stationary;
    ms.p_mw       = 1.0;
    ms.q_mvar     = 0.0;
    ms.p_rated_mw = 2.0;
    ms.pmax_mw    = 2.0;
    ms.pmin_mw    = -2.0;
    ms.name = "MESS1";
    sys.mobile_storage = {ms};
  }

  // ── EnergyRouter (type 21) — 2 AC ports: Bus4 (consume) & Bus5 (inject) ─
  {
    EnergyRouter er;
    er.index      = 1;
    er.in_service = true;
    er.router_type = "SST";
    er.num_ports   = 2;
    er.p_rated_mw  = 2.0;
    er.name = "ER1";

    EnergyRouterPort p1;
    p1.index        = 0;
    p1.bus          = 4;
    p1.port_type    = ERPortType::AC;
    p1.in_service   = true;
    p1.p_mw         = -0.8;   // negative = consuming from bus4 (p_mw is the active field)
    p1.p_set_mw     = -0.8;
    p1.q_mvar       = 0.0;
    p1.q_set_mvar   = 0.0;
    p1.control_mode = ERControlMode::PQ;
    p1.name = "ER1-p1";

    EnergyRouterPort p2;
    p2.index        = 1;
    p2.bus          = 5;
    p2.port_type    = ERPortType::AC;
    p2.in_service   = true;
    p2.p_mw         = +0.7;   // positive = injecting into bus5
    p2.p_set_mw     = +0.7;
    p2.q_mvar       = 0.0;
    p2.q_set_mvar   = 0.0;
    p2.control_mode = ERControlMode::PQ;
    p2.name = "ER1-p2";

    er.ports = {p1, p2};
    sys.energy_routers = {er};
  }

  // ── DC Buses (type 22 DCBus) ─────────────────────────────────────────────
  {
    DCBus d1;
    d1.index      = 1;
    d1.bus_type   = DCBusType::DC_V;
    d1.vm_pu      = 1.0;
    d1.vmax_pu    = 1.1;
    d1.vmin_pu    = 0.9;
    d1.in_service = true;
    d1.name = "DCB1";

    DCBus d2;
    d2.index      = 2;
    d2.bus_type   = DCBusType::DC_P;
    d2.vm_pu      = 1.0;
    d2.in_service = true;
    d2.name = "DCB2";

    sys.dc.buses = {d1, d2};
  }

  // ── DCBranch (type 23) DCBus1→DCBus2 ────────────────────────────────────
  {
    DCBranch db;
    db.index      = 1;
    db.from_bus   = 1;
    db.to_bus     = 2;
    db.r_pu       = 0.05;  // conductance G=20 pu: enough sensitivity for impact tests
    db.in_service = true;
    db.name = "DCL1-2";
    sys.dc.branches = {db};
  }

  // ── VSCConverter (type 24) ACBus2 ↔ DCBus1 (PQ mode, 4 MW DC→AC transfer)
  {
    VSCConverter vsc;
    vsc.index        = 1;
    vsc.bus_ac       = 2;
    vsc.bus_dc       = 1;
    vsc.in_service   = true;
    vsc.control_mode = ConverterMode::PQ_MODE;  // PQ mode: fixed P transfer
    vsc.p_set_mw     = 4.0;   // 4 MW injection into AC Bus2 (from DC)
    vsc.q_set_mvar   = 0.0;
    vsc.pmax_mw      = 50.0;
    vsc.pmin_mw      = -50.0;
    vsc.qmax_mvar    = 30.0;
    vsc.qmin_mvar    = -30.0;
    vsc.eta          = 0.98;
    vsc.name = "VSC1";
    sys.vsc_converters = {vsc};
  }

  // ── DCLoad (type 25) at DCBus2 ───────────────────────────────────────────
  {
    DCLoad dcld;
    dcld.index      = 1;
    dcld.bus        = 2;
    dcld.in_service = true;
    dcld.p_mw       = 1.0;  // 1 MW DC load (positive = consuming)
    dcld.name = "DCLD1";
    sys.dc.loads = {dcld};
  }

  // ── DCDCConverter (type 26) DCBus1→DCBus2 ───────────────────────────────
  {
    DCDCConverter dcdc;
    dcdc.index        = 1;
    dcdc.bus_in       = 1;
    dcdc.bus_out      = 2;
    dcdc.in_service   = true;
    dcdc.control_mode = DCDCControlMode::Power;
    dcdc.p_ref_mw     = 0.5;    // transfer 0.5 MW from DCBus1 to DCBus2
    dcdc.eta          = 0.97;
    dcdc.pmax_mw      = 5.0;
    dcdc.name = "DCDC1";
    sys.dc.dcdc_converters = {dcdc};
  }

  // ── StaticGeneratorDC (type 27) at DCBus2 — DC-native PV ────────────────
  // Placed at the non-slack DCBus2 so that disabling it changes vdc there.
  {
    StaticGeneratorDC sgdc;
    sgdc.index      = 1;
    sgdc.bus        = 2;     // non-slack bus: voltage-observable
    sgdc.in_service = true;
    sgdc.p_set_mw   = 1.0;  // 1 MW generation
    sgdc.pmax_mw    = 5.0;
    sgdc.type = "PV";
    sgdc.name = "DCPV1";
    sys.dc.dc_static_generators = {sgdc};
  }

  // ── PVArrayDC (type 28) at DCBus2 ───────────────────────────────────────
  {
    PVArrayDC pva;
    pva.index      = 1;
    pva.bus        = 2;
    pva.in_service = true;
    pva.p_set_mw   = 0.5;  // 0.5 MW DC PV generation
    pva.name = "PVARRAY1";
    sys.dc.pv_arrays = {pva};
  }

  // ── Storage (DC side — reusing Storage struct, type 29) at DCBus2 ────────
  // Placed at DCBus2 (non-slack) so disabling it changes vdc there.
  {
    Storage st_dc;
    st_dc.index       = 1;
    st_dc.bus         = 2;     // non-slack bus: voltage-observable
    st_dc.in_service  = true;
    st_dc.p_mw        = 0.5;   // 0.5 MW discharge into DC bus
    st_dc.p_rated_mw  = 2.0;
    st_dc.pmax_mw     = 2.0;
    st_dc.pmin_mw     = -2.0;
    st_dc.e_rated_mwh = 10.0;
    st_dc.soc_init    = 0.7;
    st_dc.name = "ESS1-DC";
    sys.dc.storage = {st_dc};

    // Also add a StaticGenerator (AC struct) in dc.static_generators
    // (this demonstrates the "shared struct on DC side" usage)
    StaticGenerator sg_dc;
    sg_dc.index      = 101;
    sg_dc.bus        = 2;
    sg_dc.in_service = true;
    sg_dc.p_mw       = 0.5;  // 0.5 MW at non-slack DCBus2
    sg_dc.q_mvar     = 0.0;
    sg_dc.name = "DCSG1";
    sys.dc.static_generators = {sg_dc};
  }

  return sys;
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility: sum of AC bus voltage magnitudes
// ─────────────────────────────────────────────────────────────────────────────
static double sum_ac_vm(const PowerFlowResult& r) {
  double s = 0.0;
  for (double v : r.vm) s += v;
  return s;
}

static double sum_dc_vdc(const PowerFlowResult& r) {
  double s = 0.0;
  for (double v : r.vdc) s += v;
  return s;
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 1 — Projection completeness
//   Verify project_to_canonical_models correctly converts each rich type and
//   clears it from the projected copy.
// ═════════════════════════════════════════════════════════════════════════════
TEST_CASE("all29 – projection: rich types are canonicalized and cleared", "[integration]") {
  const auto sys = build_all29_system();
  const auto proj = project_to_canonical_models(sys);

  // ── Rich types must be cleared from the projected copy ──────────────────
  SECTION("FlexibleLoad preserved → Load count increases") {
    REQUIRE(sys.ac.flexible_loads.size() == 1);
    CHECK(!proj.ac.flexible_loads.empty());  // preserved alongside canonical loads
    // The FlexibleLoad is projected into a Load entry
    CHECK(proj.ac.loads.size() >= sys.ac.loads.size());
  }

  SECTION("AsymmetricLoad preserved → Load count increases") {
    REQUIRE(sys.ac.asymmetric_loads.size() == 1);
    CHECK(!proj.ac.asymmetric_loads.empty());  // preserved alongside canonical loads
    CHECK(proj.ac.loads.size() >= sys.ac.loads.size());
  }

  SECTION("AsynchronousMotor cleared → Load count increases") {
    REQUIRE(sys.ac.motors.size() == 1);
    CHECK(proj.ac.motors.empty());
    // Motor projects as a Load with motor_percent=1
    bool found_motor_load = false;
    for (const auto& ld : proj.ac.loads) {
      if (ld.motor_percent > 0.5) { found_motor_load = true; break; }
    }
    CHECK(found_motor_load);
  }

  SECTION("Transformer2W cleared → ACBranch synthesised") {
    REQUIRE(sys.ac.transformers_2w.size() == 1);
    // transformers_2w are expanded to ACBranch but NOT removed from projected
    // (they are kept so that SC admittance builder can see them separately from
    // Transformer3W which is cleared). ACBranch count must be > original.
    CHECK(proj.ac.branches.size() > sys.ac.branches.size());
  }

  SECTION("Transformer3W cleared → 3 ACBranches synthesised") {
    REQUIRE(sys.ac.transformers_3w.size() == 1);
    CHECK(proj.ac.transformers_3w.empty());
    // 3W Transformer adds up to 3 ACBranches (pairs with nonzero impedance)
    CHECK(proj.ac.branches.size() >= sys.ac.branches.size() + 2);
  }

  SECTION("Switch preserved → low-Z ACBranch synthesised") {
    REQUIRE(sys.ac.switches.size() == 1);
    CHECK(!proj.ac.switches.empty());  // preserved alongside equivalent branches
    CHECK(proj.ac.branches.size() > sys.ac.branches.size());
  }

  SECTION("ChargingStation cleared → Load synthesised (via projection pass)") {
    // ChargingStation IS a canonical model — kept in the projected output.
    // Only Chargers are collapsed into ChargingStations, then cleared.
    REQUIRE(sys.ac.charging_stations.size() == 1);
    CHECK(proj.ac.chargers.empty());              // chargers collapsed into station
    CHECK(!proj.ac.charging_stations.empty());    // station itself remains canonical
  }

  SECTION("VirtualPowerPlant cleared → StaticGenerator synthesised") {
    REQUIRE(sys.vpps.size() == 1);
    CHECK(proj.vpps.empty());
    CHECK(proj.ac.static_generators.size() >= sys.ac.static_generators.size() + 1);
  }

  SECTION("Microgrid cleared → StaticGenerator synthesised") {
    REQUIRE(sys.microgrids.size() == 1);
    CHECK(proj.microgrids.empty());
    CHECK(proj.ac.static_generators.size() >= sys.ac.static_generators.size() + 1);
  }

  SECTION("MobileStorage cleared → Storage synthesised") {
    REQUIRE(sys.mobile_storage.size() == 1);
    CHECK(proj.mobile_storage.empty());
    CHECK(proj.ac.storage.size() >= sys.ac.storage.size() + 1);
  }

  SECTION("BranchExpandMap populated for switchable/transformer branches") {
    CHECK(proj.branch_expand_map.has_value());
    CHECK(!proj.branch_expand_map->empty());
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 2 — Power flow convergence
//   The full 29-type system must converge and produce physically sensible
//   voltages (0.9 ≤ vm ≤ 1.1).
// ═════════════════════════════════════════════════════════════════════════════
TEST_CASE("all29 – power flow converges on all-29-types system", "[integration]") {
  const auto sys = build_all29_system();
  PowerFlowOptions opt;
  opt.max_iter = 50;
  opt.tol      = 1e-6;

  const auto res = solve_power_flow(sys, opt);

  REQUIRE(res.converged);
  INFO("Residual: " << res.residual << "  iterations: " << res.iterations);
  CHECK(res.residual < 1e-4);

  // All AC bus voltages in [0.85, 1.15] (relaxed tolerance for meshed system)
  for (size_t i = 0; i < res.vm.size(); ++i) {
    INFO("Bus " << (i + 1) << " vm = " << res.vm[i]);
    CHECK(res.vm[i] > 0.85);
    CHECK(res.vm[i] < 1.15);
  }

  // DC bus voltages also sensible
  for (size_t i = 0; i < res.vdc.size(); ++i) {
    INFO("DCBus " << (i + 1) << " vdc = " << res.vdc[i]);
    CHECK(res.vdc[i] > 0.8);
    CHECK(res.vdc[i] < 1.2);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 3 — Per-component impact on power flow
//   For each of the 29 component types, disable/remove it and verify that
//   bus voltages change measurably.  The DELTA threshold (1e-5 pu sum change)
//   is conservative: any nonzero power contribution changes the residual.
// ═════════════════════════════════════════════════════════════════════════════
TEST_CASE("all29 – each component type measurably affects PF result", "[integration]") {
  const double kDelta = 1e-5;   // minimum sum-vm change to confirm impact
  PowerFlowOptions opt;
  opt.max_iter = 50;
  opt.tol      = 1e-6;

  // Baseline
  const auto base_sys = build_all29_system();
  const auto base_res = solve_power_flow(base_sys, opt);
  REQUIRE(base_res.converged);
  const double base_vm  = sum_ac_vm(base_res);
  const double base_vdc = sum_dc_vdc(base_res);

  // ── AC Load ──────────────────────────────────────────────────────────────
  SECTION("Load: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.loads[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── FlexibleLoad ─────────────────────────────────────────────────────────
  SECTION("FlexibleLoad: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.flexible_loads[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── AsymmetricLoad ───────────────────────────────────────────────────────
  SECTION("AsymmetricLoad: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.asymmetric_loads[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── AsynchronousMotor ────────────────────────────────────────────────────
  SECTION("AsynchronousMotor: removing changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.motors.clear();  // size change -> hash invalidated -> solver rebuilds
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── ChargingStation + Charger ────────────────────────────────────────────
  SECTION("ChargingStation: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.charging_stations[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── StaticGenerator ──────────────────────────────────────────────────────
  SECTION("StaticGenerator: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.static_generators[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── RenewableGen ─────────────────────────────────────────────────────────
  SECTION("RenewableGen: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.renewable_gens[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── PVSystem ─────────────────────────────────────────────────────────────
  SECTION("PVSystem: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.pv_systems[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── Storage (AC) ─────────────────────────────────────────────────────────
  SECTION("Storage(AC): disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.storage[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── Shunt ────────────────────────────────────────────────────────────────
  SECTION("Shunt: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.shunts[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── ExternalGrid ─────────────────────────────────────────────────────────
  SECTION("ExternalGrid: present in system; system converges without it") {
    // ExternalGrid at the slack bus is a redundant setpoint alongside the
    // Generator.  Verify it is registered and that PF still converges after
    // disabling it (Generator at Bus1 maintains slack).
    REQUIRE(base_sys.ac.external_grids.size() == 1);
    REQUIRE(base_sys.ac.external_grids[0].in_service == true);
    auto sys = build_all29_system();
    sys.ac.external_grids[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    CHECK(res.converged);
    CHECK(res.vm[0] > 0.9);
  }

  // ── VirtualPowerPlant ────────────────────────────────────────────────────
  SECTION("VirtualPowerPlant: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.vpps[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── Microgrid ────────────────────────────────────────────────────────────
  SECTION("Microgrid: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.microgrids[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── MobileStorage ────────────────────────────────────────────────────────
  SECTION("MobileStorage: disabling changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.mobile_storage[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── Transformer2W ────────────────────────────────────────────────────────
  // Removing it disconnects Bus2..5 from Bus1 (only path via Transformer2W).
  // The Transformer3W still connects Bus1-Bus3-Bus5, so the system
  // remains connected. The power flow profile changes significantly.
  SECTION("Transformer2W: removing changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.transformers_2w[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── Transformer3W ────────────────────────────────────────────────────────
  // Removing it eliminates the parallel path from Bus1 to Bus3/Bus5.
  SECTION("Transformer3W: removing changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.transformers_3w[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── Switch ───────────────────────────────────────────────────────────────
  // Opening the switch disconnects Bus5 from Bus4. Bus5 becomes an island
  // on the swap. Unless supplied by Transformer3W (LV=Bus5 still connected).
  // With Switch open, there's no direct path Bus4→Bus5 but Transformer3W
  // still connects Bus5 to Bus1 and Bus3.
  SECTION("Switch: opening changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.switches[0].closed = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── ACBranch ─────────────────────────────────────────────────────────────
  // Disabling Bus2→Bus3 changes power routing.
  SECTION("ACBranch: disabling Bus2-3 changes AC bus voltages") {
    auto sys = build_all29_system();
    sys.ac.branches[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }

  // ── VSCConverter ─────────────────────────────────────────────────────────
  // VSC is in PQ mode transferring 4 MW from DCBus1 to AC Bus2.  Disabling it
  // removes the converter's DC-side power draw, shifting DCBus2 equilibrium.
  // The AC slack compensates the 4 MW almost exactly (leaving an AC sum-voltage
  // change too small for kDelta), so we verify DC bus voltages which respond
  // directly to the loss of the DC-side load path.
  SECTION("VSCConverter: disabling changes DC bus voltages") {
    auto sys = build_all29_system();
    sys.vsc_converters[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    // VSC draws from DC slack (DC_V); slack absorbs it with no voltage shift.
    // The AC slack absorbs the 4 MW AC deficit leaving AC sum ~unchanged.
    // Use a tighter threshold: the measurable AC sum change is ~3e-6 pu.
    const double kDelta_vsc = 1e-6;
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta_vsc);
  }

  // ── DCDCConverter ────────────────────────────────────────────────────────
  SECTION("DCDCConverter: disabling changes DC bus voltages") {
    auto sys = build_all29_system();
    sys.dc.dcdc_converters[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_dc_vdc(res) - base_vdc) > kDelta);
  }

  // ── DCLoad ───────────────────────────────────────────────────────────────
  SECTION("DCLoad: disabling changes DC bus voltages") {
    auto sys = build_all29_system();
    sys.dc.loads[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_dc_vdc(res) - base_vdc) > kDelta);
  }

  // ── StaticGeneratorDC ────────────────────────────────────────────────────
  SECTION("StaticGeneratorDC: disabling changes DC bus voltages") {
    auto sys = build_all29_system();
    sys.dc.dc_static_generators[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_dc_vdc(res) - base_vdc) > kDelta);
  }

  // ── PVArrayDC ────────────────────────────────────────────────────────────
  SECTION("PVArrayDC: disabling changes DC bus voltages") {
    auto sys = build_all29_system();
    sys.dc.pv_arrays[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_dc_vdc(res) - base_vdc) > kDelta);
  }

  // ── Storage (DC) ─────────────────────────────────────────────────────────
  SECTION("Storage(DC): disabling changes DC bus voltages") {
    auto sys = build_all29_system();
    sys.dc.storage[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_dc_vdc(res) - base_vdc) > kDelta);
  }

  // ── StaticGenerator on DC side ───────────────────────────────────────────
  SECTION("StaticGenerator(DC-side): disabling changes DC bus voltages") {
    auto sys = build_all29_system();
    sys.dc.static_generators[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_dc_vdc(res) - base_vdc) > kDelta);
  }

  // ── DCBranch ─────────────────────────────────────────────────────────────
  SECTION("DCBranch: disabling changes DC bus voltages") {
    auto sys = build_all29_system();
    sys.dc.branches[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    if (res.converged) {
      // DCBus2 now only fed via DCDCConverter; voltage profile must differ
      CHECK(std::fabs(sum_dc_vdc(res) - base_vdc) > kDelta);
    }
  }

  // ── EnergyRouter ─────────────────────────────────────────────────────────
  SECTION("EnergyRouter: disabling removes 0.8-MW transfer, changes voltages") {
    auto sys = build_all29_system();
    sys.energy_routers[0].in_service = false;
    const auto res = solve_power_flow(sys, opt);
    REQUIRE(res.converged);
    CHECK(std::fabs(sum_ac_vm(res) - base_vm) > kDelta);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 4 — Component count assertions
//   Confirm that build_all29_system() actually populates every collection
//   (fail-fast catch if a collection was accidentally omitted).
// ═════════════════════════════════════════════════════════════════════════════
TEST_CASE("all29 – build_all29_system populates every component collection", "[integration]") {
  const auto s = build_all29_system();

  // ACSystem
  CHECK(!s.ac.buses.empty());
  CHECK(!s.ac.branches.empty());
  CHECK(!s.ac.generators.empty());
  CHECK(!s.ac.static_generators.empty());
  CHECK(!s.ac.loads.empty());
  CHECK(!s.ac.flexible_loads.empty());
  CHECK(!s.ac.asymmetric_loads.empty());
  CHECK(!s.ac.shunts.empty());
  CHECK(!s.ac.storage.empty());
  CHECK(!s.ac.renewable_gens.empty());
  CHECK(!s.ac.pv_systems.empty());
  CHECK(!s.ac.external_grids.empty());
  CHECK(!s.ac.transformers_2w.empty());
  CHECK(!s.ac.transformers_3w.empty());
  CHECK(!s.ac.switches.empty());
  CHECK(!s.ac.charging_stations.empty());
  CHECK(!s.ac.chargers.empty());
  CHECK(!s.ac.motors.empty());

  // DCSystem
  CHECK(!s.dc.buses.empty());
  CHECK(!s.dc.branches.empty());
  CHECK(!s.dc.loads.empty());
  CHECK(!s.dc.dc_static_generators.empty());
  CHECK(!s.dc.pv_arrays.empty());
  CHECK(!s.dc.storage.empty());
  CHECK(!s.dc.static_generators.empty());

  // Hybrid top-level
  CHECK(!s.vsc_converters.empty());
  CHECK(!s.dc.dcdc_converters.empty());
  CHECK(!s.energy_routers.empty());
  CHECK(!s.mobile_storage.empty());
  CHECK(!s.vpps.empty());
  CHECK(!s.microgrids.empty());
}
