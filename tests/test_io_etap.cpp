/// @file test_io_etap.cpp
/// @brief End-to-end ETAP Excel round-trip ("round-circle") test.
///
/// Builds a representative hybrid AC/DC system, exports it to an ETAP-schema
/// Excel workbook, re-imports it, exports again and re-imports.  Verifies:
///   * read-after-write fidelity   (original  vs  first reload)
///   * idempotent round-trip       (first reload  vs  second reload)
/// using Excel as both input and output medium.
///
/// Tags: [io], [etap], [excel], [roundtrip]

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/io/etap_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

namespace fs = std::filesystem;
using namespace hacdcpf;
using namespace hacdcpf::io;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kTol = 1e-6;

HybridPowerSystem make_reference_system() {
  HybridPowerSystem sys;
  sys.name = "ETAP Round-Trip Reference";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  sys.dc.base_mva = 100.0;

  // ── AC buses ──────────────────────────────────────────────────────────────
  auto bus = [](int idx, const char* nm, BusType t, double kv) {
    ACBus b;
    b.index = idx;
    b.name = nm;
    b.bus_type = t;
    b.base_kv = kv;
    b.vm_pu = 1.0;
    b.va_deg = 0.0;
    b.vmax_pu = 1.1;
    b.vmin_pu = 0.9;
    b.area = 1;
    b.zone = 1;
    return b;
  };
  sys.ac.buses.push_back(bus(1, "B1", BusType::SLACK, 110.0));
  sys.ac.buses.push_back(bus(2, "B2", BusType::PQ, 110.0));
  sys.ac.buses.push_back(bus(3, "B3", BusType::PV, 110.0));
  sys.ac.buses.push_back(bus(4, "B4", BusType::PQ, 20.0));
  sys.ac.buses[1].pd_mw = 12.0;
  sys.ac.buses[1].qd_mvar = 4.0;
  sys.ac.buses[1].bs_mvar = 3.0;

  // ── AC branches ───────────────────────────────────────────────────────────
  auto line = [](int idx, const char* nm, int f, int t, double r, double x,
                 double b) {
    ACBranch br;
    br.index = idx;
    br.name = nm;
    br.from_bus = f;
    br.to_bus = t;
    br.r_pu = r;
    br.x_pu = x;
    br.b_pu = b;
    br.tap = 1.0;
    br.rate_a_mva = 100.0;
    br.length_km = 5.0;
    return br;
  };
  sys.ac.branches.push_back(line(1, "L1", 1, 2, 0.01, 0.06, 0.02));
  sys.ac.branches.push_back(line(2, "L2", 2, 3, 0.015, 0.07, 0.03));

  // ── Transformer (110 / 20 kV) ─────────────────────────────────────────────
  Transformer2W t;
  t.index = 1;
  t.name = "T1";
  t.hv_bus = 3;
  t.lv_bus = 4;
  t.vn_hv_kv = 110.0;
  t.vn_lv_kv = 20.0;
  t.sn_mva = 40.0;
  t.vk_percent = 10.5;
  t.vkr_percent = 0.5;
  t.shift_deg = 0.0;
  t.pk_kw = 150.0;
  sys.ac.transformers_2w.push_back(t);

  // ── External grid (utility / swing) ───────────────────────────────────────
  ExternalGrid g;
  g.index = 1;
  g.name = "U1";
  g.bus = 1;
  g.vn_kv = 110.0;
  g.vm_pu = 1.02;
  g.va_deg = 0.0;
  g.r_pu = 0.001;
  g.x_pu = 0.01;
  sys.ac.external_grids.push_back(g);

  // ── Synchronous generator ─────────────────────────────────────────────────
  Generator gen;
  gen.index = 1;
  gen.name = "G1";
  gen.bus = 3;
  gen.vn_kv = 110.0;
  gen.pg_mw = 25.0;
  gen.qg_mvar = 5.0;
  gen.pmax_mw = 50.0;
  gen.pmin_mw = 0.0;
  gen.qmax_mvar = 30.0;
  gen.qmin_mvar = -30.0;
  gen.vg_pu = 1.01;
  gen.mbase_mva = 60.0;
  gen.cos_phi = 0.9;
  sys.ac.generators.push_back(gen);

  // ── PV array ──────────────────────────────────────────────────────────────
  PVSystem pv;
  pv.index = 1;
  pv.name = "PV1";
  pv.bus = 2;
  pv.p_mw = 3.0;
  pv.q_mvar = 0.5;
  pv.sn_mva = 4.0;
  pv.pmax_mw = 4.0;
  pv.qmax_mvar = 2.0;
  pv.qmin_mvar = -2.0;
  sys.ac.pv_systems.push_back(pv);

  // ── Lumped loads ──────────────────────────────────────────────────────────
  auto load = [](int idx, const char* nm, int b, double p, double q) {
    Load l;
    l.index = idx;
    l.name = nm;
    l.bus = b;
    l.p_mw = p;
    l.q_mvar = q;
    l.scaling = 1.0;
    return l;
  };
  sys.ac.loads.push_back(load(1, "LD1", 2, 12.0, 4.0));
  sys.ac.loads.push_back(load(2, "LD2", 4, 6.0, 2.0));

  // ── Capacitor (shunt) ─────────────────────────────────────────────────────
  Shunt sh;
  sh.index = 1;
  sh.name = "C1";
  sh.bus = 2;
  sh.gs_mw = 0.0;
  sh.bs_mvar = 5.0;
  sys.ac.shunts.push_back(sh);

  // ── High-voltage circuit breaker ──────────────────────────────────────────
  CircuitBreaker cb;
  cb.index = 1;
  cb.name = "CB1";
  cb.bus_from = 1;
  cb.bus_to = 2;
  cb.closed = true;
  cb.rated_voltage_kv = 110.0;
  sys.ac.circuit_breakers.push_back(cb);

  // ── Induction motor ───────────────────────────────────────────────────────
  AsynchronousMotor m;
  m.index = 1;
  m.name = "M1";
  m.bus = 4;
  m.vn_kv = 20.0;
  m.sn_mva = 2.5;
  m.r_pu = 0.02;
  m.x_pu = 0.15;
  sys.ac.motors.push_back(m);

  // ── DC buses (750 V) ──────────────────────────────────────────────────────
  auto dcbus = [](int idx, const char* nm, DCBusType t, double kv) {
    DCBus b;
    b.index = idx;
    b.name = nm;
    b.bus_type = t;
    b.base_kv = kv;
    b.vm_pu = 1.0;
    b.vmax_pu = 1.05;
    b.vmin_pu = 0.95;
    return b;
  };
  sys.dc.buses.push_back(dcbus(1, "D1", DCBusType::DC_V, 0.75));
  sys.dc.buses.push_back(dcbus(2, "D2", DCBusType::DC_P, 0.75));
  sys.dc.buses[1].pd_mw = 0.2;

  // ── DC branch ─────────────────────────────────────────────────────────────
  DCBranch dbr;
  dbr.index = 1;
  dbr.name = "DL1";
  dbr.from_bus = 1;
  dbr.to_bus = 2;
  dbr.r_pu = 0.01;
  dbr.rate_a_mva = 1.0;
  dbr.length_km = 0.5;
  sys.dc.branches.push_back(dbr);

  // ── DC load ───────────────────────────────────────────────────────────────
  DCLoad dl;
  dl.index = 1;
  dl.name = "DLD1";
  dl.bus = 2;
  dl.p_mw = 0.15;  // 150 kW
  dl.scaling = 1.0;
  sys.dc.loads.push_back(dl);

  // ── DC/DC converter ───────────────────────────────────────────────────────
  DCDCConverter dc;
  dc.index = 1;
  dc.name = "DCDC1";
  dc.bus_in = 1;
  dc.bus_out = 2;
  dc.p_ref_mw = 0.1;  // 100 kW
  dc.eta = 0.98;
  dc.vn_in_kv = 0.75;
  dc.vn_out_kv = 0.4;
  dc.pmax_mw = 0.5;
  dc.pmin_mw = -0.5;
  sys.dc.dcdc_converters.push_back(dc);

  // ── DC circuit breaker ────────────────────────────────────────────────────
  DCCircuitBreaker dccb;
  dccb.index = 1;
  dccb.name = "DCCB1";
  dccb.bus_from = 1;
  dccb.bus_to = 2;
  dccb.closed = true;
  dccb.rated_voltage_kv = 0.75;
  sys.dc.dc_circuit_breakers.push_back(dccb);

  // ── Battery (storage on DC bus) ───────────────────────────────────────────
  Storage bat;
  bat.index = 1;
  bat.name = "BAT1";
  bat.bus = 1;
  bat.p_mw = 0.0;
  bat.pmax_mw = 0.5;
  bat.pmin_mw = -0.5;
  bat.e_rated_mwh = 1.0;
  bat.soc_init = 0.6;
  bat.soc_min = 0.1;
  bat.soc_max = 0.9;
  bat.eta_charge = 0.95;
  bat.eta_discharge = 0.95;
  sys.dc.storage.push_back(bat);

  // ── Converters (AC <-> DC) ────────────────────────────────────────────────
  VSCConverter inv;
  inv.index = 1;
  inv.name = "INV1";
  inv.type = "INVERTER";
  inv.bus_ac = 4;
  inv.bus_dc = 1;
  inv.control_mode = ConverterMode::VDC_Q;
  inv.p_set_mw = 0.3;
  inv.q_set_mvar = 0.1;
  inv.v_dc_set_pu = 1.0;
  inv.v_ac_set_pu = 1.0;
  inv.pmax_mw = 1.0;
  inv.pmin_mw = -1.0;
  inv.qmax_mvar = 0.5;
  inv.qmin_mvar = -0.5;
  inv.eta = 0.99;
  inv.vn_ac_kv = 20.0;
  inv.vn_dc_kv = 0.75;
  sys.vsc_converters.push_back(inv);

  VSCConverter chg;
  chg.index = 2;
  chg.name = "CHG1";
  chg.type = "CHARGER";
  chg.bus_ac = 2;
  chg.bus_dc = 2;
  chg.control_mode = ConverterMode::PQ_MODE;
  chg.p_set_mw = 0.2;
  chg.q_set_mvar = 0.0;
  chg.pmax_mw = 0.5;
  chg.pmin_mw = 0.0;
  chg.eta = 0.97;
  chg.vn_ac_kv = 110.0;
  chg.vn_dc_kv = 0.75;
  sys.vsc_converters.push_back(chg);

  return sys;
}

// ── Field-level comparison of the ETAP-mappable subset ──────────────────────
void compare_systems(const HybridPowerSystem& a, const HybridPowerSystem& b,
                     double tol) {
  CHECK(a.name == b.name);
  CHECK_THAT(a.base_mva, WithinAbs(b.base_mva, tol));
  CHECK_THAT(a.ac.freq_hz, WithinAbs(b.ac.freq_hz, tol));

  // AC buses
  REQUIRE(a.ac.buses.size() == b.ac.buses.size());
  for (size_t i = 0; i < a.ac.buses.size(); ++i) {
    const auto& x = a.ac.buses[i];
    const auto& y = b.ac.buses[i];
    CHECK(x.name == y.name);
    CHECK(x.bus_type == y.bus_type);
    CHECK_THAT(x.base_kv, WithinAbs(y.base_kv, tol));
    CHECK_THAT(x.vmax_pu, WithinAbs(y.vmax_pu, tol));
    CHECK_THAT(x.vmin_pu, WithinAbs(y.vmin_pu, tol));
    CHECK_THAT(x.pd_mw, WithinAbs(y.pd_mw, tol));
    CHECK_THAT(x.qd_mvar, WithinAbs(y.qd_mvar, tol));
    CHECK_THAT(x.bs_mvar, WithinAbs(y.bs_mvar, tol));
    CHECK(x.in_service == y.in_service);
  }

  // AC branches
  REQUIRE(a.ac.branches.size() == b.ac.branches.size());
  for (size_t i = 0; i < a.ac.branches.size(); ++i) {
    const auto& x = a.ac.branches[i];
    const auto& y = b.ac.branches[i];
    CHECK(x.name == y.name);
    CHECK(x.from_bus == y.from_bus);
    CHECK(x.to_bus == y.to_bus);
    CHECK_THAT(x.r_pu, WithinAbs(y.r_pu, tol));
    CHECK_THAT(x.x_pu, WithinAbs(y.x_pu, tol));
    CHECK_THAT(x.b_pu, WithinAbs(y.b_pu, tol));
    CHECK_THAT(x.rate_a_mva, WithinAbs(y.rate_a_mva, tol));
  }

  // Transformers
  REQUIRE(a.ac.transformers_2w.size() == b.ac.transformers_2w.size());
  for (size_t i = 0; i < a.ac.transformers_2w.size(); ++i) {
    const auto& x = a.ac.transformers_2w[i];
    const auto& y = b.ac.transformers_2w[i];
    CHECK(x.name == y.name);
    CHECK(x.hv_bus == y.hv_bus);
    CHECK(x.lv_bus == y.lv_bus);
    CHECK_THAT(x.vn_hv_kv, WithinAbs(y.vn_hv_kv, tol));
    CHECK_THAT(x.vn_lv_kv, WithinAbs(y.vn_lv_kv, tol));
    CHECK_THAT(x.sn_mva, WithinAbs(y.sn_mva, tol));
    CHECK_THAT(x.vk_percent, WithinAbs(y.vk_percent, tol));
    CHECK_THAT(x.vkr_percent, WithinAbs(y.vkr_percent, tol));
  }

  // External grids
  REQUIRE(a.ac.external_grids.size() == b.ac.external_grids.size());
  for (size_t i = 0; i < a.ac.external_grids.size(); ++i) {
    const auto& x = a.ac.external_grids[i];
    const auto& y = b.ac.external_grids[i];
    CHECK(x.name == y.name);
    CHECK(x.bus == y.bus);
    CHECK_THAT(x.vm_pu, WithinAbs(y.vm_pu, tol));
    CHECK_THAT(x.r_pu, WithinAbs(y.r_pu, tol));
    CHECK_THAT(x.x_pu, WithinAbs(y.x_pu, tol));
  }

  // Generators
  REQUIRE(a.ac.generators.size() == b.ac.generators.size());
  for (size_t i = 0; i < a.ac.generators.size(); ++i) {
    const auto& x = a.ac.generators[i];
    const auto& y = b.ac.generators[i];
    CHECK(x.name == y.name);
    CHECK(x.bus == y.bus);
    CHECK_THAT(x.pg_mw, WithinAbs(y.pg_mw, tol));
    CHECK_THAT(x.pmax_mw, WithinAbs(y.pmax_mw, tol));
    CHECK_THAT(x.qmax_mvar, WithinAbs(y.qmax_mvar, tol));
    CHECK_THAT(x.qmin_mvar, WithinAbs(y.qmin_mvar, tol));
  }

  // PV systems
  REQUIRE(a.ac.pv_systems.size() == b.ac.pv_systems.size());
  for (size_t i = 0; i < a.ac.pv_systems.size(); ++i) {
    CHECK(a.ac.pv_systems[i].name == b.ac.pv_systems[i].name);
    CHECK(a.ac.pv_systems[i].bus == b.ac.pv_systems[i].bus);
    CHECK_THAT(a.ac.pv_systems[i].p_mw, WithinAbs(b.ac.pv_systems[i].p_mw, tol));
  }

  // Loads
  REQUIRE(a.ac.loads.size() == b.ac.loads.size());
  for (size_t i = 0; i < a.ac.loads.size(); ++i) {
    CHECK(a.ac.loads[i].name == b.ac.loads[i].name);
    CHECK(a.ac.loads[i].bus == b.ac.loads[i].bus);
    CHECK_THAT(a.ac.loads[i].p_mw, WithinAbs(b.ac.loads[i].p_mw, tol));
    CHECK_THAT(a.ac.loads[i].q_mvar, WithinAbs(b.ac.loads[i].q_mvar, tol));
  }

  // Shunts
  REQUIRE(a.ac.shunts.size() == b.ac.shunts.size());
  for (size_t i = 0; i < a.ac.shunts.size(); ++i) {
    CHECK(a.ac.shunts[i].name == b.ac.shunts[i].name);
    CHECK(a.ac.shunts[i].bus == b.ac.shunts[i].bus);
    CHECK_THAT(a.ac.shunts[i].bs_mvar, WithinAbs(b.ac.shunts[i].bs_mvar, tol));
  }

  // Circuit breakers
  REQUIRE(a.ac.circuit_breakers.size() == b.ac.circuit_breakers.size());
  for (size_t i = 0; i < a.ac.circuit_breakers.size(); ++i) {
    CHECK(a.ac.circuit_breakers[i].name == b.ac.circuit_breakers[i].name);
    CHECK(a.ac.circuit_breakers[i].bus_from == b.ac.circuit_breakers[i].bus_from);
    CHECK(a.ac.circuit_breakers[i].bus_to == b.ac.circuit_breakers[i].bus_to);
    CHECK(a.ac.circuit_breakers[i].closed == b.ac.circuit_breakers[i].closed);
  }

  // Motors
  REQUIRE(a.ac.motors.size() == b.ac.motors.size());
  for (size_t i = 0; i < a.ac.motors.size(); ++i) {
    CHECK(a.ac.motors[i].name == b.ac.motors[i].name);
    CHECK(a.ac.motors[i].bus == b.ac.motors[i].bus);
    CHECK_THAT(a.ac.motors[i].x_pu, WithinAbs(b.ac.motors[i].x_pu, tol));
  }

  // DC buses
  REQUIRE(a.dc.buses.size() == b.dc.buses.size());
  for (size_t i = 0; i < a.dc.buses.size(); ++i) {
    const auto& x = a.dc.buses[i];
    const auto& y = b.dc.buses[i];
    CHECK(x.name == y.name);
    CHECK(x.bus_type == y.bus_type);
    CHECK_THAT(x.base_kv, WithinAbs(y.base_kv, tol));
    CHECK_THAT(x.pd_mw, WithinAbs(y.pd_mw, tol));
  }

  // DC branches
  REQUIRE(a.dc.branches.size() == b.dc.branches.size());
  for (size_t i = 0; i < a.dc.branches.size(); ++i) {
    const auto& x = a.dc.branches[i];
    const auto& y = b.dc.branches[i];
    CHECK(x.name == y.name);
    CHECK(x.from_bus == y.from_bus);
    CHECK(x.to_bus == y.to_bus);
    CHECK_THAT(x.r_pu, WithinAbs(y.r_pu, tol));
  }

  // DC loads
  REQUIRE(a.dc.loads.size() == b.dc.loads.size());
  for (size_t i = 0; i < a.dc.loads.size(); ++i) {
    CHECK(a.dc.loads[i].name == b.dc.loads[i].name);
    CHECK(a.dc.loads[i].bus == b.dc.loads[i].bus);
    CHECK_THAT(a.dc.loads[i].p_mw, WithinAbs(b.dc.loads[i].p_mw, tol));
  }

  // DC/DC converters
  REQUIRE(a.dc.dcdc_converters.size() == b.dc.dcdc_converters.size());
  for (size_t i = 0; i < a.dc.dcdc_converters.size(); ++i) {
    const auto& x = a.dc.dcdc_converters[i];
    const auto& y = b.dc.dcdc_converters[i];
    CHECK(x.name == y.name);
    CHECK(x.bus_in == y.bus_in);
    CHECK(x.bus_out == y.bus_out);
    CHECK_THAT(x.p_ref_mw, WithinAbs(y.p_ref_mw, tol));
    CHECK_THAT(x.eta, WithinAbs(y.eta, tol));
  }

  // DC circuit breakers
  REQUIRE(a.dc.dc_circuit_breakers.size() == b.dc.dc_circuit_breakers.size());
  for (size_t i = 0; i < a.dc.dc_circuit_breakers.size(); ++i) {
    CHECK(a.dc.dc_circuit_breakers[i].name == b.dc.dc_circuit_breakers[i].name);
    CHECK(a.dc.dc_circuit_breakers[i].closed == b.dc.dc_circuit_breakers[i].closed);
  }

  // Battery / storage
  REQUIRE(a.dc.storage.size() == b.dc.storage.size());
  for (size_t i = 0; i < a.dc.storage.size(); ++i) {
    const auto& x = a.dc.storage[i];
    const auto& y = b.dc.storage[i];
    CHECK(x.name == y.name);
    CHECK(x.bus == y.bus);
    CHECK_THAT(x.e_rated_mwh, WithinAbs(y.e_rated_mwh, tol));
    CHECK_THAT(x.soc_init, WithinAbs(y.soc_init, tol));
  }

  // VSC converters
  REQUIRE(a.vsc_converters.size() == b.vsc_converters.size());
  for (size_t i = 0; i < a.vsc_converters.size(); ++i) {
    const auto& x = a.vsc_converters[i];
    const auto& y = b.vsc_converters[i];
    CHECK(x.name == y.name);
    CHECK(x.type == y.type);
    CHECK(x.bus_ac == y.bus_ac);
    CHECK(x.bus_dc == y.bus_dc);
    CHECK(x.control_mode == y.control_mode);
    CHECK_THAT(x.p_set_mw, WithinAbs(y.p_set_mw, tol));
    CHECK_THAT(x.eta, WithinAbs(y.eta, tol));
  }
}

}  // namespace

TEST_CASE("ETAP Excel round-circle (excel in / excel out)", "[io][etap][excel][roundtrip]") {
  const HybridPowerSystem sys = make_reference_system();

  const fs::path dir = fs::temp_directory_path();
  const std::string path_a = (dir / "hacdcpf_etap_roundtrip_A.xlsx").string();
  const std::string path_b = (dir / "hacdcpf_etap_roundtrip_B.xlsx").string();

  // First cycle: model -> Excel A -> model1.
  EtapIoReport save_rep;
  REQUIRE_NOTHROW(save_etap(sys, path_a, save_rep));
  REQUIRE(fs::exists(path_a));

  EtapIoReport load_rep;
  HybridPowerSystem sys1;
  REQUIRE_NOTHROW(sys1 = load_etap(path_a, load_rep));

  SECTION("element counts survive the first export/import") {
    CHECK(sys1.ac.buses.size() == sys.ac.buses.size());
    CHECK(sys1.ac.branches.size() == sys.ac.branches.size());
    CHECK(sys1.ac.transformers_2w.size() == sys.ac.transformers_2w.size());
    CHECK(sys1.ac.external_grids.size() == sys.ac.external_grids.size());
    CHECK(sys1.ac.generators.size() == sys.ac.generators.size());
    CHECK(sys1.ac.pv_systems.size() == sys.ac.pv_systems.size());
    CHECK(sys1.ac.loads.size() == sys.ac.loads.size());
    CHECK(sys1.ac.shunts.size() == sys.ac.shunts.size());
    CHECK(sys1.ac.circuit_breakers.size() == sys.ac.circuit_breakers.size());
    CHECK(sys1.ac.motors.size() == sys.ac.motors.size());
    CHECK(sys1.dc.buses.size() == sys.dc.buses.size());
    CHECK(sys1.dc.branches.size() == sys.dc.branches.size());
    CHECK(sys1.dc.loads.size() == sys.dc.loads.size());
    CHECK(sys1.dc.dcdc_converters.size() == sys.dc.dcdc_converters.size());
    CHECK(sys1.dc.dc_circuit_breakers.size() == sys.dc.dc_circuit_breakers.size());
    CHECK(sys1.dc.storage.size() == sys.dc.storage.size());
    CHECK(sys1.vsc_converters.size() == sys.vsc_converters.size());
    CHECK(load_rep.count("BUS") == 4);
    CHECK(load_rep.count("DCBUS") == 2);
    CHECK(load_rep.count("INVERTER") == 1);
    CHECK(load_rep.count("CHARGER") == 1);
  }

  SECTION("read-after-write fidelity (original vs first reload)") {
    compare_systems(sys, sys1, kTol);
  }

  SECTION("idempotent round-trip (first reload vs second reload)") {
    REQUIRE_NOTHROW(save_etap(sys1, path_b));
    REQUIRE(fs::exists(path_b));
    HybridPowerSystem sys2;
    REQUIRE_NOTHROW(sys2 = load_etap(path_b));
    compare_systems(sys1, sys2, kTol);
  }

  // Best-effort cleanup of temp artifacts.
  std::error_code ec;
  fs::remove(path_a, ec);
  fs::remove(path_b, ec);
}

TEST_CASE("ETAP real-export ingestion (etap-main toolkit schema)",
          "[io][etap][excel][ingest]") {
  const std::string path =
      std::string(HACDCPF_TEST_DATA_DIR) + "/etap_sample.xlsx";
  if (!fs::exists(path)) {
    WARN("ETAP sample workbook missing at " << path << "; skipping");
    return;
  }

  // Permissive import tolerates the toolkit's placeholder rows (elements whose
  // bus attributes were left blank) and records them as warnings.
  EtapIoReport rep;
  HybridPowerSystem sys;
  REQUIRE_NOTHROW(sys = load_etap(path, EtapImportMode::Permissive, rep));

  SECTION("element counts match the ETAP project") {
    CHECK(sys.ac.buses.size() == 11);
    CHECK(sys.ac.branches.size() == 6);  // 5 lines + 1 cable
    CHECK(sys.ac.transformers_2w.size() == 4);
    CHECK(sys.ac.external_grids.size() == 3);
    CHECK(sys.ac.loads.size() == 5);
  }

  SECTION("transformer T1: ETAP names + units (kVA->MVA, %Z, %R)") {
    REQUIRE(sys.ac.transformers_2w.size() >= 1);
    const auto& t = sys.ac.transformers_2w.front();
    CHECK(t.name == "T1");
    CHECK_THAT(t.vn_hv_kv, WithinAbs(220.0, 1e-6));
    CHECK_THAT(t.vn_lv_kv, WithinAbs(110.0, 1e-6));
    CHECK_THAT(t.sn_mva, WithinAbs(120.0, 1e-6));  // ZBaseMVA 120000 kVA
    CHECK_THAT(t.vk_percent, WithinAbs(12.5, 1e-6));   // AnsiPosZ
    CHECK_THAT(t.vkr_percent, WithinAbs(0.277999997, 1e-6));  // PosR
  }

  SECTION("utility pins its bus to SLACK (derived, no Type column)") {
    const ACBus* bus3 = nullptr;
    for (const auto& b : sys.ac.buses)
      if (b.name == "Bus3") bus3 = &b;
    REQUIRE(bus3 != nullptr);
    CHECK(bus3->bus_type == BusType::SLACK);
  }

  SECTION("lumped load rated MVA + PF(%) -> P/Q (MW/Mvar)") {
    const Load* lump1 = nullptr;
    for (const auto& l : sys.ac.loads)
      if (l.name == "Lump1") lump1 = &l;
    REQUIRE(lump1 != nullptr);
    CHECK_THAT(lump1->p_mw, WithinAbs(340.0, 1e-3));    // 400 * 0.85
    CHECK_THAT(lump1->q_mvar, WithinAbs(210.713, 2e-2));  // 400 * sin(acos(0.85))
  }

  SECTION("placeholder rows surface as warnings") {
    CHECK_FALSE(rep.warnings.empty());
  }

  SECTION("strict mode rejects unresolved references") {
    EtapIoReport strict_rep;
    CHECK_THROWS(load_etap(path, EtapImportMode::Strict, strict_rep));
  }
}
