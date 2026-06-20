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
#include <fstream>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/io/etap_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"

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
  sys.ac.branches[0].failure_rate = 0.02;
  sys.ac.branches[0].mttr_hr = 8.0;
  sys.ac.branches[0].n_parallel = 2;

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
  t.mtbf_hours = 87600.0;
  t.mttr_hours = 24.0;
  t.z0_percent = 9.5;
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
  g.s_sc_max_mva = 2500.0;
  g.s_sc_min_mva = 2000.0;
  g.rx_max = 10.0;
  g.rx_min = 8.0;
  g.r0_pu = 0.0015;
  g.x0_pu = 0.015;
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
  gen.cost_c2 = 0.01;
  gen.cost_c1 = 25.0;
  gen.cost_c0 = 100.0;
  gen.forced_outage_rate = 0.03;
  gen.mttr_hr = 12.0;
  gen.xdpp_pu = 0.18;
  gen.xdp_pu = 0.25;
  gen.xd_pu = 1.8;
  gen.ra_pu = 0.005;
  gen.r0_pu = 0.01;
  gen.x0_pu = 0.05;
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

  // ── Wind / renewable generator ────────────────────────────────────────────
  RenewableGen wind;
  wind.index = 1;
  wind.name = "WT1";
  wind.bus = 3;
  wind.type = RenewableType::Wind;
  wind.p_mw = 8.0;
  wind.q_mvar = 1.0;
  wind.p_rated_mw = 10.0;
  wind.qmax_mvar = 3.0;
  wind.qmin_mvar = -3.0;
  wind.curtailable = true;
  wind.capacity_factor = 0.35;
  wind.cost_curtail_mwh = 5.0;
  sys.ac.renewable_gens.push_back(wind);

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
  sys.ac.loads[0].model = LoadModel::ZIP;
  sys.ac.loads[0].z_percent_p = 20.0;
  sys.ac.loads[0].i_percent_p = 30.0;
  sys.ac.loads[0].p_percent_p = 50.0;
  sys.ac.loads[0].z_percent_q = 10.0;
  sys.ac.loads[0].i_percent_q = 40.0;
  sys.ac.loads[0].p_percent_q = 50.0;
  sys.ac.loads[0].priority = LoadPriority::High;

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
  cb.i_rated_ka = 2.0;
  cb.i_breaking_ka = 40.0;
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
    CHECK_THAT(x.failure_rate, WithinAbs(y.failure_rate, tol));
    CHECK_THAT(x.mttr_hr, WithinAbs(y.mttr_hr, tol));
    CHECK(x.n_parallel == y.n_parallel);
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
    CHECK_THAT(x.mtbf_hours, WithinAbs(y.mtbf_hours, tol));
    CHECK_THAT(x.mttr_hours, WithinAbs(y.mttr_hours, tol));
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
    CHECK_THAT(x.cost_c2, WithinAbs(y.cost_c2, tol));
    CHECK_THAT(x.cost_c1, WithinAbs(y.cost_c1, tol));
    CHECK_THAT(x.cost_c0, WithinAbs(y.cost_c0, tol));
    CHECK_THAT(x.forced_outage_rate, WithinAbs(y.forced_outage_rate, tol));
    CHECK_THAT(x.mttr_hr, WithinAbs(y.mttr_hr, tol));
  }

  // PV systems
  REQUIRE(a.ac.pv_systems.size() == b.ac.pv_systems.size());
  for (size_t i = 0; i < a.ac.pv_systems.size(); ++i) {
    CHECK(a.ac.pv_systems[i].name == b.ac.pv_systems[i].name);
    CHECK(a.ac.pv_systems[i].bus == b.ac.pv_systems[i].bus);
    CHECK_THAT(a.ac.pv_systems[i].p_mw, WithinAbs(b.ac.pv_systems[i].p_mw, tol));
  }

  // Renewable (wind) generators
  REQUIRE(a.ac.renewable_gens.size() == b.ac.renewable_gens.size());
  for (size_t i = 0; i < a.ac.renewable_gens.size(); ++i) {
    const auto& x = a.ac.renewable_gens[i];
    const auto& y = b.ac.renewable_gens[i];
    CHECK(x.name == y.name);
    CHECK(x.bus == y.bus);
    CHECK(x.type == y.type);
    CHECK_THAT(x.p_mw, WithinAbs(y.p_mw, tol));
    CHECK_THAT(x.p_rated_mw, WithinAbs(y.p_rated_mw, tol));
    CHECK(x.curtailable == y.curtailable);
    CHECK_THAT(x.capacity_factor, WithinAbs(y.capacity_factor, tol));
    CHECK_THAT(x.cost_curtail_mwh, WithinAbs(y.cost_curtail_mwh, tol));
  }

  // Loads
  REQUIRE(a.ac.loads.size() == b.ac.loads.size());
  for (size_t i = 0; i < a.ac.loads.size(); ++i) {
    CHECK(a.ac.loads[i].name == b.ac.loads[i].name);
    CHECK(a.ac.loads[i].bus == b.ac.loads[i].bus);
    CHECK_THAT(a.ac.loads[i].p_mw, WithinAbs(b.ac.loads[i].p_mw, tol));
    CHECK_THAT(a.ac.loads[i].q_mvar, WithinAbs(b.ac.loads[i].q_mvar, tol));
    CHECK(a.ac.loads[i].model == b.ac.loads[i].model);
    CHECK_THAT(a.ac.loads[i].z_percent_p, WithinAbs(b.ac.loads[i].z_percent_p, tol));
    CHECK_THAT(a.ac.loads[i].i_percent_p, WithinAbs(b.ac.loads[i].i_percent_p, tol));
    CHECK_THAT(a.ac.loads[i].p_percent_p, WithinAbs(b.ac.loads[i].p_percent_p, tol));
    CHECK_THAT(a.ac.loads[i].z_percent_q, WithinAbs(b.ac.loads[i].z_percent_q, tol));
    CHECK_THAT(a.ac.loads[i].i_percent_q, WithinAbs(b.ac.loads[i].i_percent_q, tol));
    CHECK_THAT(a.ac.loads[i].p_percent_q, WithinAbs(b.ac.loads[i].p_percent_q, tol));
    CHECK(a.ac.loads[i].priority == b.ac.loads[i].priority);
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
    CHECK(sys1.ac.renewable_gens.size() == sys.ac.renewable_gens.size());
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

TEST_CASE("ETAP I/O preserves power-flow solvability (case14)",
          "[io][etap][excel][pf][roundtrip]") {
  const std::string mp = std::string(HACDCPF_TEST_DATA_DIR) + "/case14.m";
  if (!fs::exists(mp)) {
    WARN("case14.m not found at " << mp << "; skipping");
    return;
  }

  HybridPowerSystem orig;
  REQUIRE_NOTHROW(orig = parse_matpower(mp));

  const fs::path dir = fs::temp_directory_path();
  const std::string path = (dir / "hacdcpf_etap_case14.xlsx").string();

  REQUIRE_NOTHROW(save_etap(orig, path));
  HybridPowerSystem restored;
  REQUIRE_NOTHROW(restored = load_etap(path));

  // Topology is preserved.
  REQUIRE(restored.ac.buses.size() == orig.ac.buses.size());
  REQUIRE(restored.ac.branches.size() == orig.ac.branches.size());

  // Power flow on the ETAP round-trip must converge to the same voltages.
  const PowerFlowResult r_orig = solve_power_flow(orig);
  const PowerFlowResult r_restored = solve_power_flow(restored);
  REQUIRE(r_orig.converged);
  REQUIRE(r_restored.converged);
  REQUIRE(r_orig.vm.size() == r_restored.vm.size());
  for (size_t i = 0; i < r_orig.vm.size(); ++i) {
    CHECK_THAT(r_restored.vm[i], WithinAbs(r_orig.vm[i], 1e-6));
  }

  std::error_code ec;
  fs::remove(path, ec);
}

TEST_CASE("ETAP ingested sample is power-flow solvable without crashing",
          "[io][etap][excel][pf][ingest]") {
  const std::string path =
      std::string(HACDCPF_TEST_DATA_DIR) + "/etap_sample.xlsx";
  if (!fs::exists(path)) {
    WARN("ETAP sample workbook missing at " << path << "; skipping");
    return;
  }
  HybridPowerSystem sys;
  REQUIRE_NOTHROW(sys = load_etap(path));  // Permissive by default

  // The exception-free solver must handle the toolkit's partial demo case
  // gracefully — returning a value or a structured error, never throwing.
  REQUIRE_NOTHROW([&] {
    const auto result = safe_solve_power_flow(sys);
    (void)result;
  }());
}

TEST_CASE("ETAP fidelity check reports a lossless round-trip",
          "[io][etap][excel][fidelity]") {
  const HybridPowerSystem sys = make_reference_system();
  const EtapFidelityReport fr = etap_fidelity_check(sys);
  for (const auto& m : fr.mismatches) WARN(m);
  CHECK(fr.fields_checked > 0);
  CHECK(fr.fields_mismatched == 0);
  CHECK(fr.lossless);
}

TEST_CASE("ETAP native XML import (Feeder.xml)", "[io][etap][xml][ingest]") {
  const std::string path =
      std::string(HACDCPF_TEST_DATA_DIR) + "/etap_feeder.xml";
  if (!fs::exists(path)) {
    WARN("ETAP XML fixture missing at " << path << "; skipping");
    return;
  }
  EtapIoReport rep;
  HybridPowerSystem sys;
  REQUIRE_NOTHROW(sys = load_etap_xml(path, EtapImportMode::Permissive, rep));

  // Element counts must match the raw <COMPONENTS> tag inventory of Feeder.xml.
  CHECK(sys.ac.buses.size() == 43);
  CHECK(sys.ac.branches.size() == 17);        // 16 XLINE + 1 CABLE
  CHECK(sys.ac.transformers_2w.size() == 16);
  CHECK(sys.ac.external_grids.size() == 3);
  CHECK(sys.ac.generators.size() == 1);
  CHECK(sys.ac.pv_systems.size() == 10);
  CHECK(sys.ac.loads.size() == 4);
  CHECK(sys.dc.buses.size() == 7);
  CHECK(sys.dc.loads.size() == 3);
  CHECK(sys.vsc_converters.size() == 10);     // 7 INVERTER + 3 CHARGER
  CHECK(sys.dc.storage.size() == 3);          // BATTERY

  // Utilities pin their AC bus to SLACK via bus-type derivation.
  bool any_slack = false;
  for (const auto& b : sys.ac.buses)
    if (b.bus_type == BusType::SLACK) any_slack = true;
  CHECK(any_slack);
}

// ───────────────────────────────────────────────────────────────────────────
// 3-winding transformer with NO standard tap changer.
//
// ETAP's XFORM3W schema carries the three winding ratings (MVA), nominal
// voltages and the three pairwise short-circuit impedances, but intentionally
// carries no tap-changer state.  A 3-winding transformer modelled without a
// standard tap changer (empty std_type; all tap_* at their neutral defaults)
// must therefore survive an ETAP round-trip losslessly and must never acquire a
// spurious off-nominal tap.  These cases also exercise the native ETAP XML
// reader's newly added XFORM3W support.
// ───────────────────────────────────────────────────────────────────────────
namespace {

HybridPowerSystem make_three_winding_system() {
  HybridPowerSystem sys;
  sys.name = "3W No Tap Changer";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  sys.dc.base_mva = 100.0;

  auto bus = [](int idx, const char* nm, BusType t, double kv) {
    ACBus b;
    b.index = idx;
    b.name = nm;
    b.bus_type = t;
    b.base_kv = kv;
    b.vm_pu = 1.0;
    b.vmax_pu = 1.1;
    b.vmin_pu = 0.9;
    b.area = 1;
    b.zone = 1;
    return b;
  };
  sys.ac.buses.push_back(bus(1, "HVB", BusType::SLACK, 220.0));
  sys.ac.buses.push_back(bus(2, "MVB", BusType::PQ, 66.0));
  sys.ac.buses.push_back(bus(3, "LVB", BusType::PQ, 11.0));

  ExternalGrid g;
  g.index = 1;
  g.name = "U1";
  g.bus = 1;
  g.vn_kv = 220.0;
  g.vm_pu = 1.0;
  sys.ac.external_grids.push_back(g);

  // 3-winding transformer — deliberately without a standard tap changer.
  Transformer3W t;
  t.index = 1;
  t.name = "T3W";
  t.hv_bus = 1;
  t.mv_bus = 2;
  t.lv_bus = 3;
  t.vn_hv_kv = 220.0;
  t.vn_mv_kv = 66.0;
  t.vn_lv_kv = 11.0;
  t.sn_hv_mva = 100.0;
  t.sn_mv_mva = 60.0;
  t.sn_lv_mva = 40.0;
  t.vk_hv_mv_percent = 12.0;
  t.vk_hv_lv_percent = 18.0;
  t.vk_mv_lv_percent = 7.0;
  t.vkr_hv_mv_percent = 0.6;
  t.vkr_hv_lv_percent = 0.9;
  t.vkr_mv_lv_percent = 0.35;
  t.in_service = true;
  t.std_type.clear();        // no standard (library) tap definition
  t.tap_side = 0;            // no regulated winding
  t.tap_pos = 0;             // sitting on the neutral position
  t.tap_step_percent = 0.0;  // no per-step voltage adjustment
  sys.ac.transformers_3w.push_back(t);

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
  sys.ac.loads.push_back(load(1, "LD_MV", 2, 30.0, 10.0));
  sys.ac.loads.push_back(load(2, "LD_LV", 3, 15.0, 5.0));
  return sys;
}

}  // namespace

TEST_CASE("ETAP 3-winding transformer without standard tap changer round-trips",
          "[io][etap][excel][xform3w][roundtrip]") {
  const HybridPowerSystem sys = make_three_winding_system();
  REQUIRE(sys.ac.transformers_3w.size() == 1);

  const fs::path dir = fs::temp_directory_path();
  const std::string path = (dir / "hacdcpf_etap_xform3w.xlsx").string();

  REQUIRE_NOTHROW(save_etap(sys, path));
  HybridPowerSystem back;
  REQUIRE_NOTHROW(back = load_etap(path));
  std::error_code ec;
  fs::remove(path, ec);

  REQUIRE(back.ac.transformers_3w.size() == 1);
  const auto& a = sys.ac.transformers_3w[0];
  const auto& b = back.ac.transformers_3w[0];

  SECTION("winding topology, ratings and impedances are preserved") {
    CHECK(b.name == a.name);
    CHECK(b.hv_bus == a.hv_bus);
    CHECK(b.mv_bus == a.mv_bus);
    CHECK(b.lv_bus == a.lv_bus);
    CHECK_THAT(b.vn_hv_kv, WithinAbs(a.vn_hv_kv, kTol));
    CHECK_THAT(b.vn_mv_kv, WithinAbs(a.vn_mv_kv, kTol));
    CHECK_THAT(b.vn_lv_kv, WithinAbs(a.vn_lv_kv, kTol));
    CHECK_THAT(b.sn_hv_mva, WithinAbs(a.sn_hv_mva, kTol));
    CHECK_THAT(b.sn_mv_mva, WithinAbs(a.sn_mv_mva, kTol));
    CHECK_THAT(b.sn_lv_mva, WithinAbs(a.sn_lv_mva, kTol));
    CHECK_THAT(b.vk_hv_mv_percent, WithinAbs(a.vk_hv_mv_percent, kTol));
    CHECK_THAT(b.vk_hv_lv_percent, WithinAbs(a.vk_hv_lv_percent, kTol));
    CHECK_THAT(b.vk_mv_lv_percent, WithinAbs(a.vk_mv_lv_percent, kTol));
    CHECK(b.in_service == a.in_service);

    // The HV/MV/LV winding terminals still point at the right named buses.
    REQUIRE(back.ac.buses.size() == 3);
    CHECK(back.ac.buses.at(static_cast<size_t>(b.hv_bus - 1)).name == "HVB");
    CHECK(back.ac.buses.at(static_cast<size_t>(b.mv_bus - 1)).name == "MVB");
    CHECK(back.ac.buses.at(static_cast<size_t>(b.lv_bus - 1)).name == "LVB");
  }

  SECTION("no standard tap changer is introduced by the round-trip") {
    CHECK(b.std_type.empty());
    CHECK(b.tap_side == 0);
    CHECK(b.tap_pos == 0);
    CHECK_THAT(b.tap_step_percent, WithinAbs(0.0, kTol));
  }
}

TEST_CASE("ETAP native XML imports a 3-winding transformer (no tap changer)",
          "[io][etap][xml][xform3w]") {
  // Minimal native-ETAP XML using raw toolkit attribute names: PrimkV/SeckV/
  // TerkV, PrimkVA/SeckVA/TerkVA (kVA) and the PS/PT/ST pairwise %Z values.
  // No tap-changer attributes are present.
  const std::string xml = R"(<?xml version="1.0"?>
<PROJECT>
 <COMPONENTS>
  <BUS ID="HVB" NominalkV="220" InService="true"/>
  <BUS ID="MVB" NominalkV="66" InService="true"/>
  <BUS ID="LVB" NominalkV="11" InService="true"/>
  <UTIL ID="U1" Bus="HVB" KV="220" OpVMag="100"/>
  <XFORM3W ID="T3W" HVBus="HVB" MVBus="MVB" LVBus="LVB" PrimkV="220" SeckV="66" TerkV="11" PrimkVA="100000" SeckVA="60000" TerkVA="40000" PSPosZ="12" PTPosZ="18" STPosZ="7" InService="true"/>
 </COMPONENTS>
</PROJECT>)";

  const fs::path path = fs::temp_directory_path() / "hacdcpf_etap_xform3w.xml";
  {
    std::ofstream ofs(path);
    ofs << xml;
  }

  EtapIoReport rep;
  HybridPowerSystem sys;
  REQUIRE_NOTHROW(sys = load_etap_xml(path.string(), EtapImportMode::Permissive, rep));
  std::error_code ec;
  fs::remove(path, ec);

  REQUIRE(sys.ac.buses.size() == 3);
  REQUIRE(sys.ac.transformers_3w.size() == 1);
  const auto& t = sys.ac.transformers_3w[0];
  CHECK(t.name == "T3W");
  CHECK(sys.ac.buses.at(static_cast<size_t>(t.hv_bus - 1)).name == "HVB");
  CHECK(sys.ac.buses.at(static_cast<size_t>(t.mv_bus - 1)).name == "MVB");
  CHECK(sys.ac.buses.at(static_cast<size_t>(t.lv_bus - 1)).name == "LVB");
  CHECK_THAT(t.vn_hv_kv, WithinAbs(220.0, kTol));
  CHECK_THAT(t.vn_mv_kv, WithinAbs(66.0, kTol));
  CHECK_THAT(t.vn_lv_kv, WithinAbs(11.0, kTol));
  CHECK_THAT(t.sn_hv_mva, WithinAbs(100.0, kTol));  // 100000 kVA -> 100 MVA
  CHECK_THAT(t.sn_mv_mva, WithinAbs(60.0, kTol));
  CHECK_THAT(t.sn_lv_mva, WithinAbs(40.0, kTol));
  CHECK_THAT(t.vk_hv_mv_percent, WithinAbs(12.0, kTol));  // PSPosZ
  CHECK_THAT(t.vk_hv_lv_percent, WithinAbs(18.0, kTol));  // PTPosZ
  CHECK_THAT(t.vk_mv_lv_percent, WithinAbs(7.0, kTol));   // STPosZ

  // No standard tap changer present in the XML -> neutral defaults.
  CHECK(t.std_type.empty());
  CHECK(t.tap_pos == 0);
  CHECK_THAT(t.tap_step_percent, WithinAbs(0.0, kTol));
}

TEST_CASE("ETAP 3-winding transformer WITH a tap changer round-trips",
          "[io][etap][excel][xform3w][tap][roundtrip]") {
  HybridPowerSystem sys = make_three_winding_system();
  // Give the 3-winding transformer an off-nominal regulating tap + phase shift.
  auto& t0 = sys.ac.transformers_3w[0];
  t0.tap_side = 1;
  t0.tap_pos = 2;
  t0.tap_step_percent = 1.25;
  t0.shift_mv_deg = 30.0;
  t0.shift_lv_deg = -30.0;

  SECTION("Excel round-trip preserves tap position, step and phase shift") {
    const std::string path =
        (fs::temp_directory_path() / "hacdcpf_etap_3w_tap.xlsx").string();
    REQUIRE_NOTHROW(save_etap(sys, path));
    HybridPowerSystem back;
    REQUIRE_NOTHROW(back = load_etap(path));
    std::error_code ec;
    fs::remove(path, ec);

    REQUIRE(back.ac.transformers_3w.size() == 1);
    const auto& b = back.ac.transformers_3w[0];
    CHECK(b.tap_side == 1);
    CHECK(b.tap_pos == 2);
    CHECK_THAT(b.tap_step_percent, WithinAbs(1.25, kTol));
    CHECK_THAT(b.shift_mv_deg, WithinAbs(30.0, kTol));
    CHECK_THAT(b.shift_lv_deg, WithinAbs(-30.0, kTol));
  }

  SECTION("fidelity check stays lossless with a tapped 3-winding transformer") {
    const EtapFidelityReport fr = etap_fidelity_check(sys);
    for (const auto& m : fr.mismatches) WARN(m);
    CHECK(fr.fields_mismatched == 0);
    CHECK(fr.lossless);
  }
}

TEST_CASE("ETAP 3-winding transformer solves power flow (star-equivalent)",
          "[io][etap][xform3w][pf]") {
  const HybridPowerSystem sys = make_three_winding_system();
  // The 3-winding transformer is expanded into three equivalent AC branches
  // during projection; the resulting network must solve.
  const PowerFlowResult r = solve_power_flow(sys);
  CHECK(r.converged);
  for (double vm : r.vm) CHECK((vm > 0.7 && vm < 1.2));
}

TEST_CASE("ETAP native XML import is power-flow solvable",
          "[io][etap][xml][pf]") {
  // A minimal but electrically complete ETAP project: a utility source, a line,
  // a 2-winding transformer, and a lumped load.
  const std::string xml = R"(<?xml version="1.0"?>
<PROJECT>
 <COMPONENTS>
  <BUS ID="B1" NominalkV="110" InService="true"/>
  <BUS ID="B2" NominalkV="110" InService="true"/>
  <BUS ID="B3" NominalkV="20" InService="true"/>
  <UTIL ID="U1" Bus="B1" KV="110" OpVMag="100" PosR="0.1" PosX="1.0"/>
  <XLINE ID="L1" FromBus="B1" ToBus="B2" RPos="1.21" XPos="7.26"/>
  <XFORM2W ID="T1" FromBus="B2" ToBus="B3" PrimkV="110" SeckV="20" AnsiMVA="40000" AnsiPosZ="10.5" AnsiPosXR="20"/>
  <LUMPEDLOAD ID="LD1" Bus="B3" MVA="10" PF="90"/>
 </COMPONENTS>
</PROJECT>)";
  const fs::path path = fs::temp_directory_path() / "hacdcpf_etap_pf.xml";
  {
    std::ofstream ofs(path);
    ofs << xml;
  }
  EtapIoReport rep;
  HybridPowerSystem sys;
  REQUIRE_NOTHROW(sys = load_etap_xml(path.string(), EtapImportMode::Permissive, rep));
  std::error_code ec;
  fs::remove(path, ec);

  REQUIRE(sys.ac.buses.size() == 3);
  REQUIRE(sys.ac.external_grids.size() == 1);
  REQUIRE(sys.ac.transformers_2w.size() == 1);
  REQUIRE(sys.ac.loads.size() == 1);

  // The imported system must be solvable without throwing, and converge.
  const PowerFlowResult r = solve_power_flow(sys);
  CHECK(r.converged);
}

TEST_CASE("3-winding OLTC scales only the regulated winding's branches",
          "[io][etap][xform3w][tap][projection]") {
  HybridPowerSystem sys = make_three_winding_system();
  auto& t = sys.ac.transformers_3w[0];
  t.tap_side = 0;             // HV winding regulated
  t.tap_pos = 2;
  t.tap_step_percent = 1.25;  // ratio = 1 + 2 * 1.25% = 1.025

  // Projection expands the 3-winding transformer into three equivalent AC
  // branches (a delta of pairwise impedances).  The HV winding's OLTC must
  // scale only the two branches incident to the HV terminal (HV-MV, HV-LV),
  // leaving the opposite MV-LV branch at unity ratio.
  const HybridPowerSystem proj = project_to_canonical_models(sys);
  double tap_hv_mv = -1.0, tap_hv_lv = -1.0, tap_mv_lv = -1.0;
  for (const auto& br : proj.ac.branches) {
    if (br.name.find("_HV_MV_eq") != std::string::npos) tap_hv_mv = br.tap;
    else if (br.name.find("_HV_LV_eq") != std::string::npos) tap_hv_lv = br.tap;
    else if (br.name.find("_MV_LV_eq") != std::string::npos) tap_mv_lv = br.tap;
  }
  CHECK_THAT(tap_hv_mv, WithinAbs(1.025, kTol));
  CHECK_THAT(tap_hv_lv, WithinAbs(1.025, kTol));
  CHECK_THAT(tap_mv_lv, WithinAbs(1.0, kTol));

  // The tapped network still solves.
  CHECK(solve_power_flow(sys).converged);
}

TEST_CASE("ETAP short-circuit data round-trips (Excel + native XML)",
          "[io][etap][shortcircuit]") {
  SECTION("Excel preserves source/generator/breaker/transformer SC fields") {
    const HybridPowerSystem sys = make_reference_system();
    const std::string path =
        (fs::temp_directory_path() / "hacdcpf_etap_sc.xlsx").string();
    REQUIRE_NOTHROW(save_etap(sys, path));
    HybridPowerSystem back;
    REQUIRE_NOTHROW(back = load_etap(path));
    std::error_code ec;
    fs::remove(path, ec);

    REQUIRE(!back.ac.external_grids.empty());
    const auto& g = back.ac.external_grids[0];
    CHECK_THAT(g.s_sc_max_mva, WithinAbs(2500.0, kTol));
    CHECK_THAT(g.s_sc_min_mva, WithinAbs(2000.0, kTol));
    CHECK_THAT(g.rx_max, WithinAbs(10.0, kTol));
    CHECK_THAT(g.x0_pu, WithinAbs(0.015, kTol));

    REQUIRE(!back.ac.generators.empty());
    const auto& gen = back.ac.generators[0];
    CHECK_THAT(gen.xdpp_pu, WithinAbs(0.18, kTol));
    CHECK_THAT(gen.xdp_pu, WithinAbs(0.25, kTol));
    CHECK_THAT(gen.x0_pu, WithinAbs(0.05, kTol));

    REQUIRE(!back.ac.circuit_breakers.empty());
    const auto& cb = back.ac.circuit_breakers[0];
    CHECK_THAT(cb.i_rated_ka, WithinAbs(2.0, kTol));
    CHECK_THAT(cb.i_breaking_ka, WithinAbs(40.0, kTol));

    REQUIRE(!back.ac.transformers_2w.empty());
    CHECK_THAT(back.ac.transformers_2w[0].z0_percent, WithinAbs(9.5, kTol));
  }

  SECTION("native ETAP XML maps ZeroR/ZeroX and breaker Rated SC attributes") {
    const std::string xml = R"(<?xml version="1.0"?>
<PROJECT>
 <COMPONENTS>
  <BUS ID="B1" NominalkV="110" InService="true"/>
  <BUS ID="B2" NominalkV="110" InService="true"/>
  <UTIL ID="U1" Bus="B1" KV="110" OpVMag="100" PosR="0.1" PosX="1.0" ZeroR="0.12" ZeroX="1.2"/>
  <HVCB ID="CB1" FromBus="B1" ToBus="B2" Closed="true" MaxkV="123" Rated="40"/>
 </COMPONENTS>
</PROJECT>)";
    const fs::path path = fs::temp_directory_path() / "hacdcpf_etap_sc.xml";
    {
      std::ofstream ofs(path);
      ofs << xml;
    }
    EtapIoReport rep;
    HybridPowerSystem sys;
    REQUIRE_NOTHROW(sys = load_etap_xml(path.string(), EtapImportMode::Permissive, rep));
    std::error_code ec;
    fs::remove(path, ec);

    REQUIRE(sys.ac.external_grids.size() == 1);
    CHECK_THAT(sys.ac.external_grids[0].r0_pu, WithinAbs(0.12, kTol));
    CHECK_THAT(sys.ac.external_grids[0].x0_pu, WithinAbs(1.2, kTol));
    REQUIRE(sys.ac.circuit_breakers.size() == 1);
    CHECK_THAT(sys.ac.circuit_breakers[0].i_breaking_ka, WithinAbs(40.0, kTol));
  }
}

TEST_CASE("ETAP utility short-circuit MVA drives the fault current",
          "[io][etap][shortcircuit][fault]") {
  // An external grid specified only by its 3-phase short-circuit MVA + R/X (the
  // common ETAP utility case, with no explicit r_pu/x_pu/ikq) must still
  // contribute a finite source impedance — and a stronger grid (higher SC MVA)
  // must yield a larger fault current.
  auto build = [](double s_sc_mva) {
    HybridPowerSystem sys;
    sys.base_mva = 100.0;
    sys.ac.base_mva = 100.0;
    sys.ac.freq_hz = 50.0;
    ACBus b1;
    b1.index = 1; b1.name = "B1"; b1.bus_type = BusType::SLACK;
    b1.base_kv = 110.0; b1.vm_pu = 1.0;
    ACBus b2;
    b2.index = 2; b2.name = "B2"; b2.bus_type = BusType::PQ;
    b2.base_kv = 110.0; b2.vm_pu = 1.0;
    sys.ac.buses = {b1, b2};
    ACBranch br;
    br.index = 1; br.name = "L1"; br.from_bus = 1; br.to_bus = 2;
    br.r_pu = 0.01; br.x_pu = 0.05; br.rate_a_mva = 100.0;
    sys.ac.branches = {br};
    ExternalGrid g;
    g.index = 1; g.name = "U1"; g.bus = 1; g.vn_kv = 110.0; g.vm_pu = 1.0;
    g.s_sc_max_mva = s_sc_mva;  // no r_pu/x_pu/ikq -> derived from SC MVA
    g.rx_max = 0.1;
    sys.ac.external_grids = {g};
    return sys;
  };

  using namespace hacdcpf::analysis;
  SCDetailedOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.calc_type = SCCalcType::Max;

  auto ik_at_bus1 = [&](const SCDetailedResult& r) -> double {
    for (const auto& b : r.bus_results)
      if (b.bus_id == 1) return b.ikss_ka;
    return -1.0;
  };

  const SCDetailedResult weak = run_short_circuit_detailed(build(500.0), 1, opt);
  const SCDetailedResult strong = run_short_circuit_detailed(build(5000.0), 1, opt);
  REQUIRE(weak.solved);
  REQUIRE(strong.solved);

  const double ik_weak = ik_at_bus1(weak);
  const double ik_strong = ik_at_bus1(strong);
  CHECK(ik_weak > 0.0);
  CHECK(ik_strong > ik_weak);  // a stronger source (more SC MVA) faults harder
}

TEST_CASE("ETAP native XML imports ETAP load-flow result voltages",
          "[io][etap][xml][crossval]") {
  const std::string path =
      std::string(HACDCPF_TEST_DATA_DIR) + "/etap_feeder.xml";
  if (!fs::exists(path)) {
    WARN("ETAP XML fixture missing at " << path << "; skipping");
    return;
  }
  EtapIoReport rep;
  HybridPowerSystem sys;
  REQUIRE_NOTHROW(sys = load_etap_xml(path, EtapImportMode::Permissive, rep));

  // ETAP's load-flow study result (per-bus OpVMag/OpVAng) is imported as the bus
  // voltage state — e.g. Bus18 sits at ~0.9914 pu / -29.8 deg in the ETAP study,
  // so importing the project also imports ETAP's computed operating point.
  const ACBus* b18 = nullptr;
  for (const auto& b : sys.ac.buses)
    if (b.name == "Bus18") b18 = &b;
  REQUIRE(b18 != nullptr);
  CHECK_THAT(b18->vm_pu, WithinAbs(0.991366, 1e-4));
  CHECK_THAT(b18->va_deg, WithinAbs(-29.8026, 1e-2));
}

TEST_CASE("ETAP operating-point round-trip cross-validates with our power flow",
          "[io][etap][excel][crossval][pf]") {
  const std::string mp = std::string(HACDCPF_TEST_DATA_DIR) + "/case14.m";
  if (!fs::exists(mp)) {
    WARN("case14.m not found at " << mp << "; skipping");
    return;
  }
  HybridPowerSystem orig;
  REQUIRE_NOTHROW(orig = parse_matpower(mp));

  // Round-trip the system through the ETAP schema, then cross-validate our power
  // flow on the original vs the ETAP-reloaded model.  Both solves go through the
  // same projection (so the bus ordering is consistent), and the ETAP-reloaded
  // operating point must reproduce the original to solver tolerance.
  const std::string path =
      (fs::temp_directory_path() / "hacdcpf_etap_crossval.xlsx").string();
  REQUIRE_NOTHROW(save_etap(orig, path));
  HybridPowerSystem back;
  REQUIRE_NOTHROW(back = load_etap(path));
  std::error_code ec;
  fs::remove(path, ec);

  const PowerFlowResult r_orig = solve_power_flow(orig);
  const PowerFlowResult r_back = solve_power_flow(back);
  REQUIRE(r_orig.converged);
  REQUIRE(r_back.converged);
  REQUIRE(r_orig.vm.size() == r_back.vm.size());
  double max_dvm = 0.0;
  for (size_t i = 0; i < r_orig.vm.size(); ++i)
    max_dvm = std::max(max_dvm, std::abs(r_orig.vm[i] - r_back.vm[i]));
  INFO("max |Vm(ours) - Vm(ETAP round-trip)| = " << max_dvm << " pu");
  CHECK(max_dvm < 1e-6);
}
