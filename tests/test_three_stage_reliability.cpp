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

/// Run with default options; inherit stdio so CI logs show Julia output.
ThreeStageReliabilityResult run_case(const char* json_name) {
  ThreeStageReliabilityOptions opts;
  opts.inherit_stdio = true;
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
  opts.inherit_stdio = false;
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
  CHECK(r.model_limitations.find("branch-only") != std::string::npos);
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

  CHECK(r.model_scope.find("dc-connectivity-fallback") != std::string::npos);
  CHECK(r.model_limitations.find("branch-only") != std::string::npos);
  CHECK_FALSE(r.validity.dc_power_flow_enforced);
  CHECK_FALSE(r.validity.sop_dispatch_optimised);

  check_result_shape(r, false);

  std::printf("[TC-3] case33mg_acdc: nb=%d (ac=%d dc=%d), nl=%d (ac=%d dc=%d), "
               "vsc=%d, sop=%d, mg=%d\n",
               r.nb, r.nb_ac, r.nb_dc, r.nl, r.nl_ac, r.nl_dc,
               r.nl_vsc, r.nl_sop, r.nmg);
  std::printf("[TC-3] SAIFI=%.4f, SAIDI=%.2f min/yr, EENS=%.1f kWh/yr\n",
               r.saifi, r.saidi_min, r.eens_kwh_yr);
}

// ─── TC-4: case33bw_acdc — IEEE 33-bus (BW) with SOP ────────────────────────

TEST_CASE("Three-stage reliability — case33bw_acdc (33-bus BW + SOP)",
          "[reliability][three_stage][integration][milp]") {
  auto r = run_case("case33bw_acdc.json");

  // Must have at least one SOP device.
  CHECK(r.nl_sop >= 1);
  CHECK(r.model_scope.find("dc-connectivity-fallback") != std::string::npos);
  CHECK_FALSE(r.validity.dc_power_flow_enforced);
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
  opts.inherit_stdio = false;
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
  opts.inherit_stdio = false;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(r.ok);
  REQUIRE(r.faults.size() == 1);

  CHECK(r.faults.front().pls_stage3 == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("Three-stage reliability — source capacity is finite, not infinite slack",
          "[reliability][three_stage][regression]") {
  const char* json = R"json({
    "name":"finite_source", "base_mva":10.0,
    "ac":{
      "base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":3.0,"qd_mvar":0.0,"n_customers":3}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":1.0,"pmax_mw":1.0,"pmin_mw":0.0,"qmax_mvar":1.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions opts;
  opts.inherit_stdio = false;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(r.ok);
  REQUIRE(r.faults.size() == 1);

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
  opts.inherit_stdio = false;
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
  opts.inherit_stdio = false;
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
    opts.inherit_stdio = false;
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
  base.inherit_stdio = false;
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

TEST_CASE("Three-stage reliability — Stage 1/3 cannot open healthy closed lines for free",
          "[reliability][three_stage][regression]") {
  const char* json = R"json({
    "name":"closed_loop_no_free_open", "base_mva":10.0,
    "ac":{
      "base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1},
        {"index":3,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":1}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":2,"from_bus":2,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":3,"from_bus":1,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":2.0,"pmax_mw":2.0,"pmin_mw":0.0,"qmax_mvar":2.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions opts;
  opts.inherit_stdio = false;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(r.ok);
  REQUIRE(!r.faults.empty());

  for (const auto& fault : r.faults) {
    CHECK(fault.stage3_status == "success");
    CHECK(fault.pls_stage3 >= 999.0);
  }
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
  opts.inherit_stdio = false;
  auto r = run_three_stage_reliability_from_string(json, opts);
  REQUIRE_FALSE(r.ok);
  REQUIRE(!r.error.empty());
  CHECK(r.model_scope.find("dc-connectivity-fallback") != std::string::npos);
  CHECK_FALSE(r.validity.dc_power_flow_enforced);
  REQUIRE(r.faults.size() == 1);

  CHECK(r.faults.front().pls_stage3 == Catch::Approx(400.0).margin(1e-3));
}
