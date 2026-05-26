/**
 * @file test_three_stage_reliability.cpp
 * @brief Integration tests for the three-stage native C++ MILP reliability evaluator.
 *
 * The three-stage fault-recovery reliability evaluator is implemented natively
 * in C++ (src/reliability/three_stage_reliability.cpp).  It uses the local
 * MIPSolvers B&C engine and requires no external Julia or Gurobi runtime.
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
 *         added DC sub-grid and SOP device.  Exercises the SOP power flow
 *         terms in all three MILP stages.
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
#include <string>

#include "hacdcpf/analysis/three_stage_reliability.hpp"

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
void check_result_shape(const ThreeStageReliabilityResult& r) {
  INFO("error: " << r.error);
  REQUIRE(r.ok);

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

  check_result_shape(r);

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

  // SOP configuration must be populated.
  CHECK(!r.sop_config.empty());
  for (const auto& sop : r.sop_config) {
    CHECK(sop.pmax_kw > 0.0);
    CHECK(sop.efficiency > 0.0);
    CHECK(sop.efficiency <= 1.0);
  }

  check_result_shape(r);

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
