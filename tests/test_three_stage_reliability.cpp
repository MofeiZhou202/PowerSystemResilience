/**
 * @file test_three_stage_reliability.cpp
 * @brief Integration tests for the three-stage native C++ MILP reliability evaluator.
 *
 * The three-stage fault-recovery reliability evaluator is implemented natively
 * in C++ (src/reliability/three_stage_reliability.cpp).  It uses the local
 * MIPSolvers/HiGHS stack and requires no external Julia or Gurobi runtime.
 *
 * Test layout:
 *   TC-1  Symbol linkage (unconditional) — verifies the test data files are
 *         present and that the evaluator returns a structured error (not a
 *         crash) when given empty input JSON.
 *
 *   TC-2  test_1_no_sop  — 5-bus / 5-line pure-AC radial network, no VSC or
 *         SOP.  Represents a minimal ring-main unit (RMU) feeder with two
 *         external grid connections.
 *
 *   TC-3  case33mg_acdc  — modified IEEE 33-bus distribution network with one
 *         AC/DC VSC link and one microgrid.  This is the primary test system
 *         for verifying hybrid network handling.
 *
 *   TC-4  case33bw_acdc  — modified IEEE 33-bus (Baran–Wu variant) with an
 *         added DC sub-grid and SOP device.  Exercises the documented hybrid
 *         connectivity fallback and zero-dispatch SOP reporting.
 *
 * Sanity ranges used in TC-2 / TC-3 / TC-4:
 *   SAIFI    ≥ 0                    (frequency index is non-negative)
 *   SAIDI    ≥ 0                    (duration index is non-negative)
 *   EENS     ≥ 0                    (energy index is non-negative)
 *   worst_line ∈ [1, nl_ac+nl_dc]   (refers to a valid branch)
 *   nodal vectors length == nd      (one entry per load bus)
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "hacdcpf/analysis/three_stage_reliability.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"

namespace fs = std::filesystem;

using hacdcpf::analysis::run_three_stage_reliability;
using hacdcpf::analysis::ThreeStageReliabilityOptions;
using hacdcpf::analysis::ThreeStageReliabilityResult;

// ─── helpers ─────────────────────────────────────────────────────────────────

namespace {

fs::path repo_root() {
#ifdef HACDCPF_PROJECT_ROOT
  return fs::path(HACDCPF_PROJECT_ROOT);
#else
  return fs::current_path();
#endif
}

fs::path reliability_data(const char* name) {
  return repo_root() / "tests" / "data" / "reliability" / name;
}

/// Run with default native options.
ThreeStageReliabilityResult run_case(const char* json_name) {
  ThreeStageReliabilityOptions opts;
  return run_three_stage_reliability(reliability_data(json_name), opts);
}

/// Common post-run assertions shared by all integration test cases.
void check_result_shape(const ThreeStageReliabilityResult& r,
                        bool expect_exact_ok = true) {
  INFO("error: " << r.error);
  if (expect_exact_ok) {
    REQUIRE(r.ok);
  } else {
    CHECK_FALSE(r.ok);
    CHECK(!r.error.empty());
  }

  // Indices must be finite non-negative.
  CHECK(r.saifi       >= 0.0);
  CHECK(r.saidi_min   >= 0.0);
  CHECK(r.eens_kwh_yr >= 0.0);
  CHECK(r.eens_cost   >= 0.0);

  // worst_line refers to a valid branch index.
  CHECK(r.worst_line >= 1);
  CHECK(r.worst_line <= r.nl_ac + r.nl_dc);

  // Nodal vector lengths match nd.
  CHECK(static_cast<int>(r.nodal_eens_kwh_yr.size()) == r.nd);
  CHECK(static_cast<int>(r.nodal_cif.size())         == r.nd);
  CHECK(static_cast<int>(r.nodal_cid_min.size())     == r.nd);

  // All nodal entries non-negative.
  for (double v : r.nodal_eens_kwh_yr) CHECK(v >= 0.0);
  for (double v : r.nodal_cif)         CHECK(v >= 0.0);
  for (double v : r.nodal_cid_min)     CHECK(v >= 0.0);

  // Per-fault details have been parsed.
  CHECK(!r.faults.empty());
  for (const auto& f : r.faults) {
    CHECK(!f.stage1_status.empty());
    CHECK(!f.stage2_status.empty());
    CHECK(!f.stage3_status.empty());
    CHECK(f.stage1_mip_gap >= 0.0);
    CHECK(f.stage2_mip_gap >= 0.0);
    CHECK(f.stage3_mip_gap >= 0.0);
    CHECK(f.pls_stage1 >= 0.0);
    CHECK(f.pls_stage2 >= 0.0);
    CHECK(f.pls_stage3 >= 0.0);
    CHECK(f.pls_total  >= 0.0);
    // pls_total is the sum of the three stages.
    CHECK(f.pls_total  == Catch::Approx(f.pls_stage1 + f.pls_stage2 + f.pls_stage3)
                                        .epsilon(1e-4));
  }
}

}  // namespace

// ─── TC-1: Symbol linkage (unconditional) ────────────────────────────────────

TEST_CASE("Three-stage reliability — symbol linkage and project tree",
          "[reliability][three_stage][smoke]") {
  // Verify test data is present.
  REQUIRE(fs::exists(reliability_data("test_1_no_sop.json")));
  REQUIRE(fs::exists(reliability_data("case33mg_acdc.json")));
  REQUIRE(fs::exists(reliability_data("case33bw_acdc.json")));

  // Verify symbol linkage: calling with empty JSON must return an error, not throw.
  ThreeStageReliabilityOptions opts;
  auto r = run_three_stage_reliability_from_string("", opts);
  CHECK(!r.ok);
  CHECK(!r.error.empty());
}

// ─── TC-2: test_1_no_sop — minimal 5-bus feeder ──────────────────────────────

TEST_CASE("Three-stage reliability — test_1_no_sop (5-bus pure AC)",
          "[reliability][three_stage][integration][milp]") {
  auto r = run_case("test_1_no_sop.json");

  // Network shape: 5 buses (all AC), 5 lines, no DC/VSC/SOP.
  CHECK(r.nb     == 5);
  CHECK(r.nb_ac  == 5);
  CHECK(r.nb_dc  == 0);
  CHECK(r.nl_ac  == 5);
  CHECK(r.nl_dc  == 0);
  CHECK(r.nl_sop == 0);
  CHECK(r.nl_vsc == 0);

  // With typical λ = 0.1 occ/yr per line on a 5-line feeder,
  // SAIFI should be well below 1 interruption/customer/year.
  CHECK(r.saifi < 1.0);

  CHECK(r.model_scope == "ac-lindistflow-milp");
  CHECK(r.model_limitations.find("ACBranch and DCBranch") != std::string::npos);
  CHECK(r.model_limitations.find("VSC") != std::string::npos);
  CHECK(r.validity.branch_flow_enforced);
  CHECK(r.validity.voltage_constraints_enforced);
  CHECK(r.validity.radial_topology_enforced);
  CHECK(r.validity.restoration_milp_solved);

  check_result_shape(r);

  std::printf("[TC-2] test_1_no_sop: SAIFI=%.4f, SAIDI=%.2f min/yr, EENS=%.1f kWh/yr\n",
               r.saifi, r.saidi_min, r.eens_kwh_yr);
}

// ─── TC-3: case33mg_acdc — IEEE 33-bus with VSC + microgrid ──────────────────

TEST_CASE("Three-stage reliability — case33mg_acdc (33-bus AC/DC + microgrid)",
          "[reliability][three_stage][integration][milp]") {
  auto r = run_case("case33mg_acdc.json");

  // Hybrid network: at least 33 AC buses, some DC buses, VSC links.
  // (The case carries no microgrid records despite its name.)
  CHECK(r.nb_ac  >= 33);
  CHECK(r.nl_vsc >= 1);

  CHECK(r.model_scope.find("dc-lindistflow") != std::string::npos);
  CHECK(r.model_limitations.find("ACBranch and DCBranch") != std::string::npos);
  CHECK(r.validity.dc_power_flow_enforced);
  CHECK_FALSE(r.validity.sop_dispatch_optimised);

  check_result_shape(r, false);

  std::printf("[TC-3] case33mg_acdc: nb=%d (ac=%d dc=%d), nl=%d (ac=%d dc=%d), "
               "vsc=%d, sop=%d, mg=%d\n",
               r.nb, r.nb_ac, r.nb_dc, r.nl, r.nl_ac, r.nl_dc,
               r.nl_vsc, r.nl_sop, r.nmg);
  std::printf("[TC-3] SAIFI=%.4f, SAIDI=%.2f min/yr, EENS=%.1f kWh/yr\n",
               r.saifi, r.saidi_min, r.eens_kwh_yr);
}

// ─── TC-3b: opt-in converter / switch / breaker fault set ────────────────────

TEST_CASE("Three-stage reliability — converter fault set extends contingencies",
          "[reliability][three_stage][faults]") {
  const auto path = reliability_data("case33mg_acdc.json");
  REQUIRE(fs::exists(path));

  ThreeStageReliabilityOptions base;
  auto r_base = run_three_stage_reliability(path, base);

  ThreeStageReliabilityOptions with_conv;
  with_conv.include_converter_faults = true;  // case33mg_acdc has >=1 VSC
  auto r_conv = run_three_stage_reliability(path, with_conv);

  // Enabling converter faults adds VSC/DC-DC contingencies to the fault set.
  // (Whether they shed depends on DC-side redundancy; case33mg has a self-
  // sufficient DC subnetwork, so EENS may be unchanged here — the synthetic
  // case below proves the shedding path.)
  CHECK(r_conv.faults.size() > r_base.faults.size());
  CHECK(r_conv.eens_kwh_yr >= r_base.eens_kwh_yr - 1e-6);
}

TEST_CASE("Three-stage reliability — VSC fault islands a VSC-fed DC load",
          "[reliability][three_stage][faults]") {
  // AC slack + generator; a DC bus (DC_P, not a source) whose only supply path
  // is the single VSC.  A VSC outage therefore islands the DC load.
  hacdcpf::HybridPowerSystem sys;
  hacdcpf::ACBus b1; b1.index = 1; b1.bus_type = hacdcpf::BusType::SLACK; b1.in_service = true;
  sys.ac.buses = {b1};
  hacdcpf::Generator g; g.index = 1; g.bus = 1; g.in_service = true;
  g.pmax_mw = 20.0; g.is_slack = true;
  sys.ac.generators = {g};
  hacdcpf::DCBus d; d.index = 101; d.in_service = true;  // DC_P default -> not a source
  sys.dc.buses = {d};
  hacdcpf::DCLoad dl; dl.index = 1; dl.bus = 101; dl.in_service = true; dl.p_mw = 5.0;
  sys.dc.loads = {dl};
  hacdcpf::VSCConverter v; v.index = 1; v.bus_ac = 1; v.bus_dc = 101; v.in_service = true;
  v.pmax_mw = 10.0; v.pmin_mw = -10.0; v.p_rated_mw = 10.0;
  sys.vsc_converters = {v};

  const std::string js = hacdcpf::io::to_json(sys, 2);

  ThreeStageReliabilityOptions base;
  auto r_base = hacdcpf::analysis::run_three_stage_reliability_from_string(js, base);

  ThreeStageReliabilityOptions conv;
  conv.include_converter_faults = true;
  auto r_conv = hacdcpf::analysis::run_three_stage_reliability_from_string(js, conv);

  // Baseline (branch-only) has no faults here; the opt-in VSC fault islands the
  // DC bus and sheds its 5 MW, so it strictly raises EENS.
  CHECK(r_conv.faults.size() > r_base.faults.size());
  CHECK(r_conv.eens_kwh_yr > r_base.eens_kwh_yr + 1e-6);
  bool sheds = false;
  for (const auto& f : r_conv.faults)
    if (f.pls_total > 1e-6) { sheds = true; break; }
  CHECK(sheds);
}

TEST_CASE("Three-stage reliability — DC power flow captures a DC line-limit shed",
          "[reliability][three_stage][dcpf]") {
  // AC slack + generator feed a DC bus via a VSC; two parallel DC branches
  // (each rated 3 MVA) carry a 5 MW DC load at the far DC bus.  A single DC-
  // branch outage leaves the survivor to carry all 5 MW — a thermal overload the
  // aggregate capacity fallback cannot see, but the DC power flow does.
  hacdcpf::HybridPowerSystem sys;
  hacdcpf::ACBus b1; b1.index = 1; b1.bus_type = hacdcpf::BusType::SLACK; b1.in_service = true;
  sys.ac.buses = {b1};
  hacdcpf::Generator g; g.index = 1; g.bus = 1; g.in_service = true;
  g.pmax_mw = 20.0; g.is_slack = true;
  sys.ac.generators = {g};
  hacdcpf::DCBus d101; d101.index = 101; d101.in_service = true;
  hacdcpf::DCBus d102; d102.index = 102; d102.in_service = true;
  sys.dc.buses = {d101, d102};
  hacdcpf::DCLoad dl; dl.index = 1; dl.bus = 102; dl.in_service = true; dl.p_mw = 5.0;
  sys.dc.loads = {dl};
  hacdcpf::DCBranch br1; br1.index = 1; br1.from_bus = 101; br1.to_bus = 102;
  br1.in_service = true; br1.r_pu = 0.01; br1.rate_a_mva = 3.0;
  hacdcpf::DCBranch br2; br2.index = 2; br2.from_bus = 101; br2.to_bus = 102;
  br2.in_service = true; br2.r_pu = 0.01; br2.rate_a_mva = 3.0;
  sys.dc.branches = {br1, br2};
  hacdcpf::VSCConverter v; v.index = 1; v.bus_ac = 1; v.bus_dc = 101; v.in_service = true;
  v.pmax_mw = 10.0; v.pmin_mw = -10.0; v.p_rated_mw = 10.0;
  sys.vsc_converters = {v};

  const std::string js = hacdcpf::io::to_json(sys, 2);

  ThreeStageReliabilityOptions cap;  // force the legacy capacity-only fallback
  cap.include_dc_power_flow = false;
  auto r_cap = hacdcpf::analysis::run_three_stage_reliability_from_string(js, cap);

  ThreeStageReliabilityOptions pf;
  pf.include_dc_power_flow = true;
  auto r_pf = hacdcpf::analysis::run_three_stage_reliability_from_string(js, pf);

  // The capacity fallback sees aggregate capacity (VSC 10 MW) >= 5 MW demand, so
  // a single DC-branch outage sheds nothing.  The DC power flow enforces the
  // 3 MVA branch limit -> ~2 MW shed per stage.
  CHECK_FALSE(r_cap.validity.dc_power_flow_enforced);
  CHECK(r_cap.eens_kwh_yr == Catch::Approx(0.0).margin(1.0));

  CHECK(r_pf.validity.dc_power_flow_enforced);
  CHECK(r_pf.model_scope.find("dc-lindistflow") != std::string::npos);
  CHECK(r_pf.eens_kwh_yr > r_cap.eens_kwh_yr + 1.0);
  bool overload_shed = false;
  for (const auto& f : r_pf.faults)
    if (f.pls_stage1 == Catch::Approx(2000.0).margin(300.0)) { overload_shed = true; break; }
  CHECK(overload_shed);
}

// ─── TC-4: case33bw_acdc — IEEE 33-bus (BW) with SOP ────────────────────────

TEST_CASE("Three-stage reliability — case33bw_acdc (33-bus BW + SOP)",
          "[reliability][three_stage][integration][milp]") {
  auto r = run_case("case33bw_acdc.json");

  // Must have at least one SOP device.
  CHECK(r.nl_sop >= 1);
  CHECK(r.model_scope.find("dc-lindistflow") != std::string::npos);
  CHECK(r.validity.dc_power_flow_enforced);
  CHECK_FALSE(r.validity.sop_dispatch_optimised);

  // SOP configuration must be populated.
  CHECK(!r.sop_config.empty());
  for (const auto& sop : r.sop_config) {
    CHECK(sop.pmax_kw > 0.0);
    CHECK(sop.efficiency > 0.0);
    CHECK(sop.efficiency <= 1.0);
  }

  check_result_shape(r, false);

  // All three stage-1 SOP power vectors must have the right length.
  for (const auto& f : r.faults) {
    CHECK(static_cast<int>(f.psop1.size()) == r.nl_sop);
    CHECK(static_cast<int>(f.psop2.size()) == r.nl_sop);
    CHECK(static_cast<int>(f.psop3.size()) == r.nl_sop);
  }

  std::printf("[TC-4] case33bw_acdc: nl_sop=%d, SAIFI=%.4f, "
               "SAIDI=%.2f min/yr, EENS=%.1f kWh/yr\n",
               r.nl_sop, r.saifi, r.saidi_min, r.eens_kwh_yr);
}

// ─── TC-5: error handling — nonexistent case file ────────────────────────────

TEST_CASE("Three-stage reliability — graceful error on missing case file",
          "[reliability][three_stage][error]") {
  ThreeStageReliabilityOptions opts;
  auto r = run_three_stage_reliability(
      reliability_data("nonexistent_case_12345.json"), opts);

  CHECK(!r.ok);
  CHECK(!r.error.empty());
}

TEST_CASE("Three-stage reliability — AC bus load is not counted twice",
          "[reliability][three_stage][regression]") {
  const char* json = R"json({
    "name":"bus_load_once", "base_mva":10.0,
    "ac":{
      "base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],
      "external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":1.0,"pmax_mw":1.0,"pmin_mw":0.0,"qmax_mvar":1.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions opts;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(r.ok);
  REQUIRE(r.faults.size() == 1);

  // The single branch fault isolates bus2 for the whole event (F7: the fault is
  // repaired only at tau_RP, so Stage 3 keeps it open).  bus2's 1 MW = 1000 kW is
  // therefore shed in the repair window, counted exactly ONCE (2000 would signal
  // the bus-load double count this test guards against).
  CHECK(r.faults.front().pls_stage3 == Catch::Approx(1000.0).margin(1e-3));
}

TEST_CASE("Three-stage reliability — source capacity is finite, not infinite slack",
          "[reliability][three_stage][regression]") {
  // Two PARALLEL branches feed bus2 so that faulting one still leaves bus2
  // connected to the finite 1 MW source through the other.  This isolates the
  // *capacity* limit (gen 1 MW < load 3 MW ⇒ 2 MW shed) from the F7 topology
  // effect: with the healthy parallel branch carrying, bus2 stays energized in
  // every stage, and the finite source can only serve 1 MW ⇒ pls == 2000 kW.
  const char* json = R"json({
    "name":"finite_source", "base_mva":10.0,
    "ac":{
      "base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":3.0,"qd_mvar":0.0,"n_customers":3}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":2,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":1.0,"pmax_mw":1.0,"pmin_mw":0.0,"qmax_mvar":1.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions opts;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(r.ok);
  REQUIRE(r.faults.size() == 2);  // one fault per parallel branch

  // Faulting either parallel branch leaves bus2 fed by the surviving branch;
  // the finite 1 MW source serves 1 MW of the 3 MW load ⇒ 2 MW (2000 kW) shed
  // during the repair window.
  CHECK(r.faults.front().pls_stage3 == Catch::Approx(2000.0).margin(1e-3));
}

TEST_CASE("Three-stage reliability — standalone AC switch is a Stage 2 candidate edge",
          "[reliability][three_stage][regression]") {
  const char* json = R"json({
    "name":"standalone_switch_stage2", "base_mva":10.0,
    "ac":{
      "base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.5,"qd_mvar":0.0,"n_customers":1},
        {"index":3,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.5,"qd_mvar":0.0,"n_customers":1}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":2,"from_bus":2,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":2.0,"pmax_mw":2.0,"pmin_mw":0.0,"qmax_mvar":2.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],
      "switches":[{"index":1,"bus_from":1,"bus_to":3,"closed":false,"in_service":true}]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions opts;
  opts.max_switch_operations = 1;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(r.ok);
  REQUIRE(r.faults.size() == 2);

  const auto& faulted_source_edge = r.faults.front();
  CHECK(faulted_source_edge.pls_stage1 > 900.0);
  CHECK(faulted_source_edge.pls_stage2 == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("Three-stage reliability — unavailable tie (fail-to-close) loses Stage 2 restoration",
          "[reliability][three_stage][switch_fail]") {
  const char* json = R"json({
    "name":"tie_fail_to_close", "base_mva":10.0,
    "ac":{
      "base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.5,"qd_mvar":0.0,"n_customers":1},
        {"index":3,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.5,"qd_mvar":0.0,"n_customers":1}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":2,"from_bus":2,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":2.0,"pmax_mw":2.0,"pmin_mw":0.0,"qmax_mvar":2.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],
      "switches":[{"index":1,"bus_from":1,"bus_to":3,"closed":false,"in_service":true}]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  // Baseline: the tie may close -> Stage 2 fully restores the isolated feeder.
  ThreeStageReliabilityOptions opts;
  opts.max_switch_operations = 1;
  auto base = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(base.ok);
  REQUIRE(base.faults.size() == 2);

  // Fail-to-close: tie switch index 1 is unavailable -> Stage-2 restoration lost.
  ThreeStageReliabilityOptions opts_fail = opts;
  opts_fail.unavailable_tie_switch_ids = {1};
  auto failed = run_three_stage_reliability_from_string(json, opts_fail);
  REQUIRE(failed.ok);
  REQUIRE(failed.faults.size() == 2);

  const auto& base_fault = base.faults.front();
  const auto& failed_fault = failed.faults.front();
  CHECK(base_fault.pls_stage2 == Catch::Approx(0.0).margin(1e-6));
  CHECK(failed_fault.pls_stage2 > base_fault.pls_stage2 + 1.0);
  CHECK(failed.eens_kwh_yr > base.eens_kwh_yr);
}

TEST_CASE("Three-stage reliability — longer MTTR raises EENS (per-component repair duration)",
          "[reliability][three_stage][duration]") {
  auto run_with_mttr = [](double mttr) {
    // Capacity-limited source (gen 0.5 MW < load 1.0 MW) so Stage-3 (post-repair)
    // shed is non-zero; the Stage-3 duration then scales with the branch MTTR.
    std::string json = std::string(R"json({
      "name":"mttr_test","base_mva":10.0,
      "ac":{"base_mva":10.0,
        "buses":[
          {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
          {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1}
        ],
        "branches":[
          {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":)json")
      + std::to_string(mttr) + R"json(}
        ],
        "loads":[],"external_grids":[],
        "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":0.5,"pmax_mw":0.5,"pmin_mw":0.0,"qmax_mvar":0.5,"qmin_mvar":0.0}],
        "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
      },
      "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
      "vsc_converters":[],"dcdc_converters":[]
    })json";
    ThreeStageReliabilityOptions opts;
    return run_three_stage_reliability_from_string(json, opts);
  };

  auto r1 = run_with_mttr(1.0);
  auto r20 = run_with_mttr(20.0);
  REQUIRE(r1.ok);
  REQUIRE(r20.ok);
  CHECK(r1.eens_kwh_yr > 0.0);
  // ~20x MTTR on the dominant Stage-3 repair window -> several-fold more EENS.
  CHECK(r20.eens_kwh_yr > 5.0 * r1.eens_kwh_yr);
}

// ─── F7 regression guard: topology-isolated load IS charged the repair window ──
// A radial feeder with ONE branch and NO tie.  Ample generation (10 MW) feeds a
// 1 MW load through the only branch.  When that branch faults the load is
// DISCONNECTED for the whole repair time (F7 fix: Stage 3 keeps the fault open
// and holds the reconfiguration through [τ_TP, τ_RP]).  The stage durations sum
// exactly to MTTR (τ_iso + τ_sw + τ_rep = 1/60 + 1/60 + (MTTR − 1/30) = MTTR), so
// EENS = λ·L·MTTR = 1000·MTTR kWh/yr and scales linearly with MTTR.
//
// Before the fix, Stage 3 restored the faulted branch, giving pls_stage3 ≈ 0 and
// an EENS pinned at λ·L·(τ_iso+τ_sw) ≈ 33.3 kWh/yr regardless of MTTR.  This test
// now fails if that regression returns.
TEST_CASE("Three-stage reliability — F7: topology-isolated load is charged the repair window",
          "[reliability][three_stage][f7]") {
  auto run_with_mttr = [](double mttr) {
    std::string json = std::string(R"json({
      "name":"f7_radial_no_tie","base_mva":10.0,
      "ac":{"base_mva":10.0,
        "buses":[
          {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
          {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1}
        ],
        "branches":[
          {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":)json")
      + std::to_string(mttr) + R"json(}
        ],
        "loads":[],"external_grids":[],
        "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":10.0,"pmax_mw":10.0,"pmin_mw":0.0,"qmax_mvar":10.0,"qmin_mvar":0.0}],
        "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
      },
      "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
      "vsc_converters":[],"dcdc_converters":[]
    })json";
    ThreeStageReliabilityOptions opts;
    return run_three_stage_reliability_from_string(json, opts);
  };

  // Sweep MTTR over three orders of magnitude.
  const double mttrs[] = {1.0, 10.0, 100.0, 1000.0};
  std::printf("\n[F7] radial 1-branch feeder, 1 MW load, 10 MW gen, no tie, lambda=1/yr\n");
  std::printf("[F7] %8s | %10s %10s %10s | %12s | %14s\n",
              "MTTR(h)", "shed1(kW)", "shed2(kW)", "shed3(kW)", "EENS(kWh/yr)",
              "lam*L*MTTR");
  double eens_first = -1.0, eens_last = -1.0;
  for (double mttr : mttrs) {
    auto r = run_with_mttr(mttr);
    REQUIRE(r.ok);
    REQUIRE(r.faults.size() == 1);
    const auto& f = r.faults.front();
    const double textbook = 1.0 * 1000.0 * mttr;  // lambda * load(kW) * MTTR(h)
    std::printf("[F7] %8.1f | %10.3f %10.3f %10.3f | %12.3f | %14.0f\n",
                mttr, f.pls_stage1, f.pls_stage2, f.pls_stage3, r.eens_kwh_yr, textbook);
    if (eens_first < 0.0) eens_first = r.eens_kwh_yr;
    eens_last = r.eens_kwh_yr;

    // The isolated 1 MW load is now shed in every stage, including the long
    // repair window (Stage 3), so pls_stage3 == the full load.
    CHECK(f.pls_stage3 == Catch::Approx(1000.0).margin(1e-3));
    // EENS equals the textbook radial load-point index lambda*L*MTTR exactly,
    // because the three stage durations sum to MTTR.
    CHECK(r.eens_kwh_yr == Catch::Approx(textbook).epsilon(0.02));
  }

  // EENS now scales linearly with MTTR: a 1000x repair window ⇒ ~1000x EENS.
  std::printf("[F7] EENS(MTTR=1h)=%.3f  EENS(MTTR=1000h)=%.3f  ratio=%.4f "
              "(expected ~1000)\n",
              eens_first, eens_last, eens_last / std::max(1e-9, eens_first));
  CHECK(eens_last == Catch::Approx(1000.0 * eens_first).epsilon(0.02));
  CHECK(eens_first == Catch::Approx(1000.0 * 1.0).epsilon(0.05));  // MTTR=1h ⇒ ~1000 kWh/yr
}

TEST_CASE("Three-stage reliability — opt-in generator + transformer faults extend the contingency set",
          "[reliability][three_stage][fault_set]") {
  // bus3 is reachable only through transformer1 (hv=1, lv=3); branch1 feeds bus2.
  // gen1 at bus1 is the only source.
  const char* json = R"json({
    "name":"gen_tf_faults","base_mva":10.0,
    "ac":{"base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1},
        {"index":3,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.5,"qd_mvar":0.0,"n_customers":1}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "transformers_2w":[
        {"index":1,"hv_bus":1,"lv_bus":3,"sn_mva":10.0,"in_service":true,"mtbf_hours":8760.0,"mttr_hours":5.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":3.0,"pmax_mw":3.0,"pmin_mw":0.0,"qmax_mvar":3.0,"qmin_mvar":0.0,"forced_outage_rate":0.02,"mttr_hr":10.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions base;
  auto r_base = run_three_stage_reliability_from_string(json, base);
  REQUIRE(r_base.ok);
  const size_t n_base = r_base.faults.size();

  ThreeStageReliabilityOptions gen_on = base;
  gen_on.include_generator_faults = true;
  auto r_gen = run_three_stage_reliability_from_string(json, gen_on);
  REQUIRE(r_gen.ok);
  CHECK(r_gen.faults.size() == n_base + 1);            // +1 generator outage
  CHECK(r_gen.eens_kwh_yr > r_base.eens_kwh_yr);       // loss-of-only-source adds EENS

  ThreeStageReliabilityOptions tf_on = base;
  tf_on.include_transformer_faults = true;
  auto r_tf = run_three_stage_reliability_from_string(json, tf_on);
  REQUIRE(r_tf.ok);
  CHECK(r_tf.faults.size() == n_base + 1);             // +1 transformer outage

  ThreeStageReliabilityOptions both = base;
  both.include_generator_faults = true;
  both.include_transformer_faults = true;
  auto r_both = run_three_stage_reliability_from_string(json, both);
  REQUIRE(r_both.ok);
  CHECK(r_both.faults.size() == n_base + 2);
}

TEST_CASE("nsq Monte Carlo on a real hybrid case file routes through the hybrid LP",
          "[reliability][mc][hybrid][integration]") {
  const fs::path path = reliability_data("case33mg_acdc.json");
  if (!fs::exists(path)) {
    WARN("case33mg_acdc.json not found; skipping real-file MC test");
    return;
  }
  std::ifstream in(path);
  const std::string text((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());
  REQUIRE_FALSE(text.empty());

  hacdcpf::HybridPowerSystem sys = hacdcpf::io::from_json(text);
  REQUIRE_FALSE(sys.vsc_converters.empty());  // confirms a hybrid AC/DC case

  hacdcpf::analysis::ReliabilityOptions opt;
  opt.max_iterations = 300;
  opt.seed = 7;
  opt.compute_tail_risk = false;
  auto r = hacdcpf::analysis::run_nonsequential_mc(sys, opt);

  // The hybrid case must now be evaluated with the hybrid network LP so DC load
  // curtailment is part of the reported EENS/LOLE.
  CHECK(r.model_scope == "hybrid-acdc-network-lp");
  CHECK(r.validity.dc_load_curtailment_included);
  CHECK(r.eens_mwh_yr >= 0.0);
  CHECK(r.iterations_used > 0);
}

TEST_CASE("Three-stage reliability — healthy closed loop must shed to stay radial",
          "[reliability][three_stage][regression]") {
  // Triangle (buses 1-2-3, branches b1,b2,b3) fed by a 3 MW source at bus 1, plus
  // a radial spur b4 (3-4) to load bus 4.  The z >= y_from+y_to-1 constraint means
  // a healthy closed line cannot be opened for free — a loop can only be broken by
  // de-energizing (shedding) a bus.  Two behaviours are checked together:
  //   • Fault on the spur b4 (line_id 4): bus4 is isolated for the whole repair
  //     window (F7) AND the still-healthy, still-closed triangle must shed one of
  //     its load buses to become radial ⇒ pls_stage3 ≈ 2000 kW.
  //   • Fault on a triangle edge (line_id 1): the fault itself breaks the loop, so
  //     the remaining network is radial and the 3 MW source serves all load ⇒
  //     pls_stage3 ≈ 0.  (Under the old Stage-3-restores-fault bug this fault also
  //     recreated the closed triangle and wrongly forced a shed.)
  const char* json = R"json({
    "name":"closed_loop_shed_for_radiality", "base_mva":10.0,
    "ac":{
      "base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1},
        {"index":3,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1},
        {"index":4,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":2,"from_bus":2,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":3,"from_bus":1,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":4,"from_bus":3,"to_bus":4,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":3.0,"pmax_mw":3.0,"pmin_mw":0.0,"qmax_mvar":3.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions opts;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(r.ok);
  REQUIRE(r.faults.size() == 4);

  const hacdcpf::analysis::ThreeStageFaultDetail* spur = nullptr;  // fault on b4 (3-4), line_id 4
  const hacdcpf::analysis::ThreeStageFaultDetail* tri = nullptr;   // fault on a triangle edge, line_id 1
  for (const auto& f : r.faults) {
    CHECK(f.stage3_status == "success");
    if (f.line_id == 4) spur = &f;
    if (f.line_id == 1) tri = &f;
  }
  REQUIRE(spur != nullptr);
  REQUIRE(tri != nullptr);

  // Spur fault: isolated spur load (1000 kW) + one triangle load shed to break the
  // healthy closed loop (1000 kW) ⇒ ≈ 2000 kW.
  CHECK(spur->pls_stage3 >= 1999.0);
  // Triangle-edge fault breaks the loop itself ⇒ radial, fully served ⇒ ≈ 0.
  CHECK(tri->pls_stage3 == Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("Three-stage reliability — VSC transfer limits AC source support for DC load",
          "[reliability][three_stage][regression]") {
  const char* json = R"json({
    "name":"vsc_capacity_limited_dc_load", "base_mva":10.0,
    "ac":{
      "base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":2.0,"pmax_mw":2.0,"pmin_mw":0.0,"qmax_mvar":2.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
    },
    "dc":{
      "buses":[{"index":101,"bus_type":1,"vm_pu":1.0,"in_service":true,"pd_mw":1.0,"is_load":true,"n_customers":1}],
      "branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]
    },
    "vsc_converters":[{"index":1,"bus_ac":1,"bus_dc":101,"in_service":true,"eta":1.0,"p_rated_mw":0.6,"pmin_mw":-0.6,"pmax_mw":0.6}],
    "dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions opts;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE_FALSE(r.ok);
  REQUIRE(!r.error.empty());
  CHECK(r.model_scope.find("dc-lindistflow") != std::string::npos);
  CHECK(r.validity.dc_power_flow_enforced);
  REQUIRE(r.faults.size() == 1);

  CHECK(r.faults.front().pls_stage3 == Catch::Approx(400.0).margin(1e-3));
}
