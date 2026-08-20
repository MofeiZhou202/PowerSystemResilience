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
 *         added DC sub-grid and SOP device. Exercises the coupled AC/DC
 *         restoration model and signed VSC dispatch reporting.
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

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/analysis/three_stage_reliability.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"

namespace fs = std::filesystem;

using hacdcpf::analysis::run_three_stage_reliability;
using hacdcpf::analysis::run_three_stage_reliability_from_string;
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

hacdcpf::HybridPowerSystem make_two_bus_islanding_case(double load_mw = 0.4,
                                                        double mttr_hr = 1.0) {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  hacdcpf::ACBus upstream;
  upstream.index = 1;
  upstream.bus_type = hacdcpf::BusType::SLACK;
  upstream.base_kv = 10.0;
  upstream.vmin_pu = 0.9;
  upstream.vmax_pu = 1.1;
  hacdcpf::ACBus island = upstream;
  island.index = 2;
  island.bus_type = hacdcpf::BusType::PQ;
  island.pd_mw = load_mw;
  island.qd_mvar = load_mw * 0.2;
  island.n_customers = 10;
  sys.ac.buses = {upstream, island};

  hacdcpf::ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.001;
  branch.x_pu = 0.001;
  branch.rate_a_mva = 5.0;
  branch.failure_rate = 1.0;
  branch.mttr_hr = mttr_hr;
  sys.ac.branches = {branch};

  hacdcpf::ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.s_sc_max_mva = 10.0;
  sys.ac.external_grids = {grid};
  return sys;
}

/// Common post-run assertions shared by all integration test cases.
void check_result_shape(const ThreeStageReliabilityResult& r,
                        bool expect_exact_ok = true) {
  std::ostringstream diagnostics;
  if (!r.ok) {
    diagnostics << "fault_count=" << r.faults.size() << '\n';
    for (const auto& fault : r.faults) {
      diagnostics << "fault " << fault.component_type << ':'
                  << fault.component_index << " status=" << fault.status
                  << " stages=[" << fault.stage1_status << ", "
                  << fault.stage2_status << ", " << fault.stage3_status << "]"
                  << " solver_statuses=[" << fault.stage1_solver_status << ", "
                  << fault.stage2_solver_status << ", "
                  << fault.stage3_solver_status << "]"
                  << " gaps=[" << fault.stage1_mip_gap << ", "
                  << fault.stage2_mip_gap << ", " << fault.stage3_mip_gap
                  << "] solver_reported_gaps=["
                  << fault.stage1_solver_reported_mip_gap << ", "
                  << fault.stage2_solver_reported_mip_gap << ", "
                  << fault.stage3_solver_reported_mip_gap
                  << "]\n";
    }
  }
  INFO("error: " << r.error << '\n' << diagnostics.str());
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
    CHECK(!f.stage1_solver_status.empty());
    CHECK(!f.stage2_solver_status.empty());
    CHECK(!f.stage3_solver_status.empty());
    CHECK(f.stage1_mip_gap >= 0.0);
    CHECK(f.stage2_mip_gap >= 0.0);
    CHECK(f.stage3_mip_gap >= 0.0);
    CHECK(f.stage1_solver_reported_mip_gap >= 0.0);
    CHECK(f.stage2_solver_reported_mip_gap >= 0.0);
    CHECK(f.stage3_solver_reported_mip_gap >= 0.0);
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

TEST_CASE("Three-stage reliability - non-grid-forming 0.4 MW DER cannot anchor an outage island",
          "[reliability][three_stage][islanding][der]") {
  auto baseline = make_two_bus_islanding_case();
  const auto without_der = run_three_stage_reliability_from_string(
      hacdcpf::io::to_json(baseline, 2), {});

  hacdcpf::StaticGenerator der;
  der.index = 1;
  der.bus = 2;
  der.p_mw = 0.4;
  der.pmax_mw = 0.4;
  der.qmax_mvar = 0.4;
  der.scaling = 1.0;
  der.grid_forming = false;
  der.anti_islanding = true;
  baseline.ac.static_generators = {der};
  const auto with_der = run_three_stage_reliability_from_string(
      hacdcpf::io::to_json(baseline, 2), {});

  REQUIRE(without_der.ok);
  REQUIRE(with_der.ok);
  REQUIRE(with_der.faults.size() == 1);
  CHECK(with_der.faults[0].pls_stage1 == Catch::Approx(400.0).margin(1e-3));
  CHECK(with_der.faults[0].pls_stage2 == Catch::Approx(400.0).margin(1e-3));
  CHECK(with_der.faults[0].pls_stage3 == Catch::Approx(400.0).margin(1e-3));
  CHECK(with_der.eens_kwh_yr == Catch::Approx(without_der.eens_kwh_yr).margin(1e-6));
}

TEST_CASE("Three-stage reliability - storage follows sequential stage energy and forming rules",
          "[reliability][three_stage][islanding][storage]") {
  auto make_storage_case = [](bool grid_forming, double energy_mwh) {
    auto sys = make_two_bus_islanding_case(0.4, 1.0);
    hacdcpf::Storage storage;
    storage.index = 1;
    storage.bus = 2;
    storage.p_rated_mw = 0.4;
    storage.pmax_mw = 0.4;
    storage.qmax_mvar = 0.4;
    storage.e_rated_mwh = energy_mwh;
    storage.soc_init = 1.0;
    storage.soc_min = 0.0;
    storage.eta_discharge = 1.0;
    storage.grid_forming = grid_forming;
    storage.anti_islanding = true;
    sys.ac.storage = {storage};
    return run_three_stage_reliability_from_string(hacdcpf::io::to_json(sys, 2), {});
  };

  const auto following = make_storage_case(false, 1.0);
  REQUIRE(following.ok);
  CHECK(following.faults[0].pls_stage3 == Catch::Approx(400.0).margin(1e-3));
  CHECK(following.faults[0].storage_energy_used_stage1_mwh ==
        Catch::Approx(0.0).margin(1e-9));

  const auto small_forming = make_storage_case(true, 0.1);
  REQUIRE(small_forming.ok);
  REQUIRE(small_forming.faults.size() == 1);
  const auto& fault = small_forming.faults[0];
  INFO("raw stage3=" << fault.raw_pls_stage3
       << " n0 stage3=" << fault.n0_pls_stage3
       << " incremental stage3=" << fault.pls_stage3);
  CHECK(fault.pls_stage1 == Catch::Approx(0.0).margin(1e-3));
  CHECK(fault.pls_stage2 == Catch::Approx(0.0).margin(1e-3));
  CHECK(fault.pls_stage3 > 300.0);
  CHECK(fault.pls_stage3 < 400.0);
  CHECK(fault.tau_iso_hr == Catch::Approx(1.0 / 60.0));
  CHECK(fault.tau_sw_hr == Catch::Approx(1.0 / 60.0));
  CHECK(fault.tau_rep_hr == Catch::Approx(1.0 - 1.0 / 30.0));
  CHECK(fault.storage_energy_initial_mwh == Catch::Approx(0.1));
  CHECK(fault.storage_energy_used_stage1_mwh ==
        Catch::Approx(0.4 / 60.0).margin(1e-8));
  CHECK(fault.storage_energy_used_stage2_mwh ==
        Catch::Approx(0.4 / 60.0).margin(1e-8));
  CHECK(fault.storage_energy_remaining_mwh == Catch::Approx(0.0).margin(1e-8));

  const auto large_forming = make_storage_case(true, 1.0);
  REQUIRE(large_forming.ok);
  CHECK(large_forming.faults[0].pls_stage3 == Catch::Approx(0.0).margin(1e-3));
  CHECK(large_forming.eens_kwh_yr < small_forming.eens_kwh_yr);
}

TEST_CASE("Three-stage reliability - grid-following PV needs a grid-forming island anchor",
          "[reliability][three_stage][islanding][pv]") {
  const auto run = [](bool include_storage, bool storage_grid_forming) {
    auto sys = make_two_bus_islanding_case(0.8, 1.0);
    hacdcpf::PVSystem pv;
    pv.index = 1;
    pv.bus = 2;
    pv.p_mw = 0.4;
    pv.pmax_mw = 0.4;
    pv.qmax_mvar = 0.4;
    pv.grid_forming = false;
    pv.anti_islanding = true;
    sys.ac.pv_systems = {pv};
    if (include_storage) {
      hacdcpf::Storage storage;
      storage.index = 1;
      storage.bus = 2;
      storage.p_rated_mw = 0.4;
      storage.pmax_mw = 0.4;
      storage.qmax_mvar = 0.4;
      storage.e_rated_mwh = 2.0;
      storage.soc_init = 1.0;
      storage.soc_min = 0.0;
      storage.eta_discharge = 1.0;
      storage.grid_forming = storage_grid_forming;
      sys.ac.storage = {storage};
    }
    return run_three_stage_reliability_from_string(hacdcpf::io::to_json(sys, 2), {});
  };

  const auto pv_only = run(false, false);
  const auto following_storage = run(true, false);
  const auto forming_storage = run(true, true);
  REQUIRE(pv_only.ok);
  REQUIRE(following_storage.ok);
  REQUIRE(forming_storage.ok);
  CHECK(pv_only.faults[0].pls_stage3 == Catch::Approx(800.0).margin(1e-3));
  CHECK(following_storage.faults[0].pls_stage3 == Catch::Approx(800.0).margin(1e-3));
  CHECK(forming_storage.faults[0].pls_stage3 == Catch::Approx(0.0).margin(1e-3));
}

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
  CHECK(r.model_limitations.find("默认 N-1 故障集包含交流、直流支路") != std::string::npos);
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
  CHECK(r.model_limitations.find("默认 N-1 故障集包含交流、直流支路") != std::string::npos);
  CHECK(r.validity.dc_power_flow_enforced);
  CHECK(r.validity.sop_dispatch_optimised);
  CHECK(r.validity.branch_flow_enforced);
  CHECK(r.validity.voltage_constraints_enforced);
  CHECK(r.validity.radial_topology_enforced);
  CHECK(r.validity.restoration_milp_solved);

  check_result_shape(r);

  // Regression for a near-zero load-shed optimum: HiGHS 1.14 reports a
  // relative gap of 1 although its model status is explicitly Optimal.  Keep
  // the backend diagnostic, but normalize the certified gap from the stronger
  // termination certificate instead of downgrading the stage to approximate.
  const auto zero_shed_fault = std::find_if(
      r.faults.begin(), r.faults.end(), [](const auto& fault) {
        return fault.component_type == "ac_branch" && fault.component_index == 2;
      });
  REQUIRE(zero_shed_fault != r.faults.end());
  CHECK(zero_shed_fault->stage2_solver_status == "StrictHiGHS Optimal run=0");
  CHECK(zero_shed_fault->stage3_solver_status == "StrictHiGHS Optimal run=0");
  CHECK(zero_shed_fault->stage2_solver_reported_mip_gap ==
        Catch::Approx(1.0).margin(1e-12));
  CHECK(zero_shed_fault->stage3_solver_reported_mip_gap ==
        Catch::Approx(1.0).margin(1e-12));
  CHECK(zero_shed_fault->stage2_mip_gap == Catch::Approx(0.0).margin(1e-12));
  CHECK(zero_shed_fault->stage3_mip_gap == Catch::Approx(0.0).margin(1e-12));
  CHECK(zero_shed_fault->stage2_status == "success");
  CHECK(zero_shed_fault->stage3_status == "success");

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

TEST_CASE("Three-stage reliability — fault-level parallel execution matches serial",
          "[reliability][three_stage][parallel]") {
  const auto path = reliability_data("test_1_no_sop.json");
  ThreeStageReliabilityOptions serial_options;
  serial_options.enable_parallel = false;
  serial_options.revalidate_stage3_plan = true;
  const auto serial = run_three_stage_reliability(path, serial_options);

  ThreeStageReliabilityOptions parallel_options;
  parallel_options.enable_parallel = true;
  parallel_options.parallel_threads = 2;
  const auto parallel = run_three_stage_reliability(path, parallel_options);

  REQUIRE(serial.ok);
  REQUIRE(parallel.ok);
  CHECK_FALSE(serial.parallel_execution.effective);
  CHECK(parallel.parallel_execution.effective);
  CHECK(parallel.parallel_execution.resolved_workers == 2);
  REQUIRE(parallel.faults.size() == serial.faults.size());
  CHECK(parallel.eens_kwh_yr == Catch::Approx(serial.eens_kwh_yr));
  CHECK(parallel.saifi == Catch::Approx(serial.saifi));
  CHECK(parallel.saidi_min == Catch::Approx(serial.saidi_min));
  for (size_t i = 0; i < serial.faults.size(); ++i) {
    CHECK(parallel.faults[i].line_id == serial.faults[i].line_id);
    CHECK(parallel.faults[i].pls_stage1 ==
          Catch::Approx(serial.faults[i].pls_stage1));
    CHECK(parallel.faults[i].pls_stage2 ==
          Catch::Approx(serial.faults[i].pls_stage2));
    CHECK(parallel.faults[i].pls_stage3 ==
          Catch::Approx(serial.faults[i].pls_stage3));
  }
}

// ─── TC-4: case33bw_acdc — IEEE 33-bus (BW) with SOP ────────────────────────

TEST_CASE("Three-stage reliability — case33bw_acdc (33-bus BW + SOP)",
          "[reliability][three_stage][integration][milp]") {
  auto r = run_case("case33bw_acdc.json");

  // Must have at least one SOP device.
  CHECK(r.nl_sop >= 1);
  CHECK(r.model_scope.find("dc-lindistflow") != std::string::npos);
  CHECK(r.validity.dc_power_flow_enforced);
  CHECK(r.validity.sop_dispatch_optimised);

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

  // The 2 MW deficit already exists in the healthy N-0 state. It remains
  // visible in the raw audit fields but is not charged to either branch fault.
  CHECK(r.faults.front().raw_pls_stage3 == Catch::Approx(2000.0).margin(1e-3));
  CHECK(r.faults.front().n0_pls_stage3 == Catch::Approx(3000.0).margin(1e-3));
  CHECK(r.faults.front().pls_stage3 == Catch::Approx(0.0).margin(1e-3));
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

TEST_CASE("Three-stage reliability — trip, isolate, then restore sequence is audited",
          "[reliability][three_stage][switch_sequence]") {
  const char* json = R"json({
    "name":"typed_switch_restoration", "base_mva":10.0,
    "ac":{"base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":3,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.5,"qd_mvar":0.0,"n_customers":10},
        {"index":4,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1}
      ],
      "branches":[
        {"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":2,"from_bus":2,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":3,"from_bus":4,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":0.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":2.0,"pmax_mw":2.0,"pmin_mw":0.0,"qmax_mvar":2.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],
      "switches":[
        {"index":10,"name":"CB-FEEDER","bus_from":1,"bus_to":2,"closed":true,"normal_closed":true,"in_service":true,"switch_type":"CircuitBreaker","role":"Protection","t_open_s":0.08,"capabilities":{"can_interrupt_fault_current":true,"can_interrupt_load_current":true,"can_close_for_restoration":true,"requires_deenergized_operation":false,"allows_source_parallel":false}},
        {"index":13,"name":"DS-FEEDER","bus_from":2,"bus_to":3,"closed":true,"normal_closed":true,"in_service":true,"switch_type":"Disconnector","role":"Isolation","controlled_element_type":"ac_branch","controlled_element_index":1,"controlled_branch_index":1,"upstream_protective_switch_index":10,"t_open_s":1.0,"capabilities":{"can_interrupt_fault_current":false,"can_interrupt_load_current":false,"can_close_for_restoration":false,"requires_deenergized_operation":true,"allows_source_parallel":false}},
        {"index":11,"name":"FUSE-NO","bus_from":1,"bus_to":4,"closed":false,"normal_closed":false,"in_service":true,"switch_type":"Fuse","role":"Protection","capabilities":{"can_interrupt_fault_current":true,"can_interrupt_load_current":false,"can_close_for_restoration":false,"requires_deenergized_operation":false,"allows_source_parallel":false}},
        {"index":12,"name":"TIE-NO","bus_from":1,"bus_to":3,"closed":false,"normal_closed":false,"in_service":true,"switch_type":"LoadBreakSwitch","role":"Tie","t_close_s":2.5,"capabilities":{"can_interrupt_fault_current":false,"can_interrupt_load_current":true,"can_close_for_restoration":true,"requires_deenergized_operation":false,"allows_source_parallel":false}}
      ]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions opts;
  opts.max_switch_operations = 1;
  const auto result = run_three_stage_reliability_from_string(json, opts);
  REQUIRE(result.ok);
  REQUIRE(result.faults.size() == 3);
  const auto& source_fault = result.faults.front();
  CHECK(source_fault.pls_stage2 == Catch::Approx(0.0).margin(1e-6));
  CHECK(source_fault.switching_sequence_valid);
  CHECK(source_fault.protection_interlock_valid);
  CHECK(source_fault.restoration_milp_admitted);
  CHECK(source_fault.stage3_switch_plan_held);
  CHECK(source_fault.fault_isolation_explicit);
  REQUIRE(source_fault.switching_sequence.size() == 3);
  const auto& trip = source_fault.switching_sequence[0];
  CHECK(trip.sequence_order == 1);
  CHECK(trip.switch_index == 10);
  CHECK(trip.action == "trip");
  CHECK(trip.validated);
  const auto& isolate = source_fault.switching_sequence[1];
  CHECK(isolate.sequence_order == 2);
  CHECK(isolate.switch_index == 13);
  CHECK(isolate.action == "open");
  CHECK(isolate.validated);
  const auto& close = source_fault.switching_sequence[2];
  CHECK(close.sequence_order == 3);
  CHECK(close.switch_index == 12);
  CHECK(close.action == "close");
  CHECK(close.validated);
  CHECK(close.operation_time_s == Catch::Approx(2.5));

  std::string blocked_json(json);
  const std::string protection_marker = "\"role\":\"Protection\",";
  const auto protection_pos = blocked_json.find(protection_marker);
  REQUIRE(protection_pos != std::string::npos);
  blocked_json.insert(protection_pos, "\"locked_closed\":true,");
  const auto blocked = run_three_stage_reliability_from_string(blocked_json, opts);
  CHECK_FALSE(blocked.ok);
  REQUIRE_FALSE(blocked.faults.empty());
  const auto& blocked_fault = blocked.faults.front();
  CHECK(blocked_fault.stage2_status == "failed (protection interlock)");
  CHECK(blocked_fault.stage3_status == "failed (protection interlock)");
  CHECK_FALSE(blocked_fault.protection_interlock_valid);
  CHECK_FALSE(blocked_fault.restoration_milp_admitted);
  CHECK_FALSE(blocked_fault.stage3_switch_plan_held);
  CHECK_FALSE(blocked_fault.switching_sequence_valid);
  CHECK(blocked_fault.pls_stage2 == Catch::Approx(blocked_fault.pls_stage1));
  CHECK(std::none_of(blocked_fault.switching_sequence.begin(),
                     blocked_fault.switching_sequence.end(),
                     [](const auto& action) { return action.action == "close"; }));
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
  REQUIRE(n_base == 1);
  // Transformer faults are opt-in, but the healthy transformer must still be
  // present as a topology edge. A fault on branch 1 must not strand bus 3.
  CHECK(r_base.faults[0].pls_stage3 ==
        Catch::Approx(1000.0).margin(1e-3));

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

TEST_CASE("Three-stage reliability — islanding microgrid restores a feeder island",
          "[reliability][three_stage][microgrid]") {
  const auto run = [](bool islanding) {
    std::string json = std::string(R"json({
      "name":"microgrid_islanding","base_mva":10.0,
      "ac":{"base_mva":10.0,
        "buses":[
          {"index":1,"bus_type":1,"base_kv":10.0,"vmin_pu":0.9,"vmax_pu":1.1,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
          {"index":2,"bus_type":3,"base_kv":10.0,"vmin_pu":0.9,"vmax_pu":1.1,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":10}
        ],
        "branches":[{"index":1,"from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":5.0,"in_service":true,"failure_rate":1.0,"mttr_hr":4.0}],
        "loads":[],"external_grids":[],
        "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":2.0,"pmax_mw":2.0,"pmin_mw":0.0,"qmax_mvar":2.0,"qmin_mvar":-2.0}],
        "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[]
      },
      "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
      "microgrids":[{"index":1,"pcc_bus":2,"in_service":true,"islanding_capability":)json")
        + (islanding ? "true" : "false") + R"json(,"capacity_mw":1.0,"mtbf_hours":20000.0,"mttr_hours":8.0}],
      "vsc_converters":[],"dcdc_converters":[]
    })json";
    return run_three_stage_reliability_from_string(json, {});
  };

  const auto without_islanding = run(false);
  const auto with_islanding = run(true);
  REQUIRE(without_islanding.ok);
  REQUIRE(with_islanding.ok);
  REQUIRE(without_islanding.faults.size() == 1);
  REQUIRE(with_islanding.faults.size() == 1);
  CHECK(without_islanding.faults[0].pls_stage3 >= 999.0);
  CHECK(with_islanding.faults[0].pls_stage3 ==
        Catch::Approx(0.0).margin(1e-3));
  CHECK(with_islanding.eens_kwh_yr < without_islanding.eens_kwh_yr);
}

TEST_CASE("Three-stage reliability — DER storage and microgrid faults are enumerated",
          "[reliability][three_stage][microgrid][fault_set]") {
  const char* json = R"json({
    "name":"der_microgrid_faults","base_mva":10.0,
    "ac":{"base_mva":10.0,
      "buses":[{"index":1,"bus_type":3,"base_kv":10.0,"vmin_pu":0.9,"vmax_pu":1.1,"in_service":true,"pd_mw":0.5,"qd_mvar":0.0,"n_customers":10}],
      "branches":[],"loads":[],"external_grids":[],"generators":[],
      "static_generators":[{"index":1,"bus":1,"in_service":true,"p_mw":0.5,"pmax_mw":0.5,"qmax_mvar":0.5,"scaling":1.0,"grid_forming":true,"mtbf_hours":3000.0,"mttr_hours":24.0}],
      "renewable_gens":[{"index":1,"bus":1,"in_service":true,"p_mw":0.3,"p_rated_mw":0.3,"capacity_factor":1.0,"mtbf_hours":4000.0,"mttr_hours":48.0}],
      "pv_systems":[{"index":1,"bus":1,"in_service":true,"p_mw":0.3,"pmax_mw":0.3,"mtbf_hours":8000.0,"mttr_hours":12.0}],
      "storage":[{"index":1,"bus":1,"in_service":true,"p_mw":0.0,"pmax_mw":0.3,"p_rated_mw":0.3,"forced_outage_rate":0.015,"mttr_hr":24.0}],
      "switches":[]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "microgrids":[{"index":1,"pcc_bus":1,"internal_buses":[1],"in_service":true,"islanding_capability":true,"capacity_mw":1.2,"mtbf_hours":20000.0,"mttr_hours":8.0}],
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  ThreeStageReliabilityOptions options;
  options.include_generator_faults = true;
  const auto result = run_three_stage_reliability_from_string(json, options);
  REQUIRE(result.ok);
  REQUIRE(result.faults.size() == 5);

  std::vector<std::string> types;
  for (const auto& fault : result.faults) types.push_back(fault.component_type);
  for (const char* expected : {"static_generator", "renewable_gen",
                               "ac_pv_system", "storage", "microgrid"}) {
    CHECK(std::find(types.begin(), types.end(), expected) != types.end());
  }
  const auto microgrid_fault = std::find_if(
      result.faults.begin(), result.faults.end(), [](const auto& fault) {
        return fault.component_type == "microgrid";
      });
  REQUIRE(microgrid_fault != result.faults.end());
  CHECK(microgrid_fault->pls_stage3 >= 499.0);
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

  // The healthy loop already requires 1000 kW of raw radiality shedding. The
  // spur fault adds only the isolated 1000 kW spur load to that N-0 baseline.
  CHECK(spur->raw_pls_stage3 >= 1999.0);
  CHECK(spur->n0_pls_stage3 >= 999.0);
  CHECK(spur->pls_stage3 == Catch::Approx(1000.0).margin(1e-3));
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
  REQUIRE(r.ok);
  REQUIRE(r.error.empty());
  CHECK(r.model_scope.find("dc-lindistflow") != std::string::npos);
  CHECK(r.validity.dc_power_flow_enforced);
  REQUIRE(r.faults.size() == 1);

  CHECK(r.faults.front().raw_pls_stage3 == Catch::Approx(400.0).margin(1e-3));
  CHECK(r.faults.front().n0_pls_stage3 == Catch::Approx(400.0).margin(1e-3));
  CHECK(r.faults.front().pls_stage3 == Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("Three-stage reliability — coupled VSC applies efficiency once in both directions",
          "[reliability][three_stage][hybrid][vsc]") {
  SECTION("AC to DC transfer") {
    auto sys = make_two_bus_islanding_case(0.0, 1.0);
    sys.ac.buses[1].pd_mw = 0.0;
    sys.ac.buses[1].qd_mvar = 0.0;
    sys.ac.external_grids.clear();
    hacdcpf::Generator generator;
    generator.index = 1; generator.bus = 1; generator.in_service = true;
    generator.pmax_mw = 5.0; generator.qmax_mvar = 5.0; generator.is_slack = true;
    sys.ac.generators = {generator};
    hacdcpf::DCBus dc_bus;
    dc_bus.index = 101; dc_bus.in_service = true;
    sys.dc.buses = {dc_bus};
    hacdcpf::DCLoad load;
    load.index = 1; load.bus = 101; load.in_service = true; load.p_mw = 1.0;
    sys.dc.loads = {load};
    hacdcpf::VSCConverter converter;
    converter.index = 1; converter.bus_ac = 1; converter.bus_dc = 101;
    converter.in_service = true; converter.pmax_mw = 1.0;
    converter.pmin_mw = -1.0; converter.p_rated_mw = 1.0; converter.eta = 0.9;
    sys.vsc_converters = {converter};

    const auto result = run_three_stage_reliability_from_string(
        hacdcpf::io::to_json(sys, 2), {});
    REQUIRE(result.ok);
    REQUIRE(result.faults.size() == 1);
    CHECK(result.faults.front().raw_pls_stage3 ==
          Catch::Approx(100.0).margin(1e-3));
    REQUIRE(result.faults.front().psop3.size() == 1);
    CHECK(result.faults.front().psop3.front() ==
          Catch::Approx(1000.0).margin(1e-3));
  }

  SECTION("DC to AC transfer") {
    auto sys = make_two_bus_islanding_case(0.9, 1.0);
    sys.ac.external_grids.clear();
    hacdcpf::Generator generator;
    generator.index = 1; generator.bus = 1; generator.in_service = true;
    generator.pmax_mw = 5.0; generator.qmax_mvar = 5.0; generator.is_slack = true;
    sys.ac.generators = {generator};
    hacdcpf::DCBus dc_bus;
    dc_bus.index = 101; dc_bus.bus_type = hacdcpf::DCBusType::DC_V;
    dc_bus.in_service = true;
    sys.dc.buses = {dc_bus};
    hacdcpf::StaticGeneratorDC dc_generator;
    dc_generator.index = 1; dc_generator.bus = 101; dc_generator.in_service = true;
    dc_generator.controllable = true; dc_generator.pmax_mw = 2.0;
    dc_generator.p_set_mw = 2.0;
    sys.dc.dc_static_generators = {dc_generator};
    hacdcpf::VSCConverter converter;
    converter.index = 1; converter.bus_ac = 2; converter.bus_dc = 101;
    converter.in_service = true; converter.pmax_mw = 2.0;
    converter.pmin_mw = -2.0; converter.p_rated_mw = 2.0;
    converter.qmin_mvar = -1.0; converter.qmax_mvar = 1.0;
    converter.eta = 0.9; converter.ac_grid_forming = true;
    sys.vsc_converters = {converter};

    const auto result = run_three_stage_reliability_from_string(
        hacdcpf::io::to_json(sys, 2), {});
    REQUIRE(result.ok);
    REQUIRE(result.faults.size() == 1);
    CHECK(result.faults.front().raw_pls_stage3 ==
          Catch::Approx(0.0).margin(1e-3));
    REQUIRE(result.faults.front().psop3.size() == 1);
    CHECK(result.faults.front().psop3.front() ==
          Catch::Approx(-1000.0).margin(1e-3));
  }
}

TEST_CASE("Three-stage reliability — DC-DC efficiency and outage are physical",
          "[reliability][three_stage][hybrid][dcdc]") {
  auto sys = make_two_bus_islanding_case(0.0, 1.0);
  sys.ac.buses[1].pd_mw = 0.0;
  sys.ac.buses[1].qd_mvar = 0.0;
  hacdcpf::DCBus d1; d1.index = 101; d1.in_service = true;
  hacdcpf::DCBus d2; d2.index = 102; d2.in_service = true;
  sys.dc.buses = {d1, d2};
  hacdcpf::DCLoad load;
  load.index = 1; load.bus = 102; load.in_service = true; load.p_mw = 1.0;
  sys.dc.loads = {load};
  hacdcpf::VSCConverter vsc;
  vsc.index = 1; vsc.bus_ac = 1; vsc.bus_dc = 101; vsc.in_service = true;
  vsc.pmax_mw = 2.0; vsc.pmin_mw = -2.0; vsc.p_rated_mw = 2.0; vsc.eta = 1.0;
  sys.vsc_converters = {vsc};
  hacdcpf::DCDCConverter dcdc;
  dcdc.index = 1; dcdc.bus_in = 101; dcdc.bus_out = 102; dcdc.in_service = true;
  dcdc.pmax_mw = 1.0; dcdc.pmin_mw = -1.0; dcdc.sn_mva = 1.0;
  dcdc.eta = 0.8; dcdc.mtbf_hours = 8760.0; dcdc.mttr_hours = 2.0;
  sys.dc.dcdc_converters = {dcdc};

  ThreeStageReliabilityOptions options;
  options.include_converter_faults = true;
  const auto result = run_three_stage_reliability_from_string(
      hacdcpf::io::to_json(sys, 2), options);
  REQUIRE(result.ok);
  const auto fault = std::find_if(result.faults.begin(), result.faults.end(),
                                  [](const auto& row) {
                                    return row.component_type == "dcdc_converter";
                                  });
  REQUIRE(fault != result.faults.end());
  CHECK(fault->raw_pls_stage3 == Catch::Approx(1000.0).margin(1e-3));
  const auto branch_fault = std::find_if(result.faults.begin(), result.faults.end(),
                                         [](const auto& row) {
                                           return row.component_type == "ac_branch";
                                         });
  REQUIRE(branch_fault != result.faults.end());
  CHECK(branch_fault->raw_pls_stage3 == Catch::Approx(200.0).margin(1e-3));
}

TEST_CASE("Three-stage reliability — DC storage energy carries across all stages",
          "[reliability][three_stage][hybrid][storage]") {
  auto sys = make_two_bus_islanding_case(0.0, 2.0);
  sys.ac.buses[1].pd_mw = 0.0;
  sys.ac.buses[1].qd_mvar = 0.0;
  hacdcpf::DCBus bus;
  bus.index = 101; bus.bus_type = hacdcpf::DCBusType::DC_V; bus.in_service = true;
  sys.dc.buses = {bus};
  hacdcpf::DCLoad load;
  load.index = 1; load.bus = 101; load.in_service = true; load.p_mw = 1.0;
  sys.dc.loads = {load};
  hacdcpf::Storage storage;
  storage.index = 1; storage.bus = 101; storage.in_service = true;
  storage.pmax_mw = 1.0; storage.p_rated_mw = 1.0;
  storage.e_rated_mwh = 1.25; storage.soc_init = 1.0; storage.soc_min = 0.0;
  storage.eta_discharge = 0.8;
  sys.dc.storage = {storage};

  const auto result = run_three_stage_reliability_from_string(
      hacdcpf::io::to_json(sys, 2), {});
  REQUIRE(result.ok);
  REQUIRE(result.faults.size() == 1);
  const auto& fault = result.faults.front();
  CHECK(fault.storage_energy_initial_mwh == Catch::Approx(1.0).margin(1e-9));
  CHECK(fault.storage_energy_used_stage1_mwh +
        fault.storage_energy_used_stage2_mwh +
        fault.storage_energy_used_stage3_mwh == Catch::Approx(1.0).margin(1e-6));
  CHECK(fault.storage_energy_remaining_mwh == Catch::Approx(0.0).margin(1e-6));
  CHECK(fault.raw_pls_stage3 > 400.0);
}

TEST_CASE("Three-stage reliability — all DC DER fault families are enumerated",
          "[reliability][three_stage][hybrid][fault_set]") {
  hacdcpf::HybridPowerSystem sys;
  hacdcpf::DCBus bus;
  bus.index = 101; bus.bus_type = hacdcpf::DCBusType::DC_V; bus.in_service = true;
  bus.pd_mw = 0.7; sys.dc.buses = {bus};
  hacdcpf::StaticGeneratorDC g1;
  g1.index = 1; g1.bus = 101; g1.in_service = true; g1.controllable = true;
  g1.pmax_mw = 0.2; g1.p_set_mw = 0.2; g1.mtbf_hours = 8760.0; g1.mttr_hours = 2.0;
  sys.dc.dc_static_generators = {g1};
  hacdcpf::StaticGenerator g2;
  g2.index = 2; g2.bus = 101; g2.in_service = true; g2.controllable = true;
  g2.pmax_mw = 0.2; g2.p_mw = 0.2; g2.scaling = 1.0;
  g2.mtbf_hours = 8760.0; g2.mttr_hours = 2.0;
  sys.dc.static_generators = {g2};
  hacdcpf::PVArrayDC pv;
  pv.index = 3; pv.bus = 101; pv.in_service = true; pv.p_set_mw = 0.2;
  pv.mtbf_hours = 8760.0; pv.mttr_hours = 2.0;
  sys.dc.pv_arrays = {pv};
  hacdcpf::Storage storage;
  storage.index = 4; storage.bus = 101; storage.in_service = true;
  storage.pmax_mw = 0.2; storage.p_rated_mw = 0.2;
  storage.e_rated_mwh = 1.0; storage.soc_init = 1.0; storage.soc_min = 0.0;
  storage.forced_outage_rate = 0.01; storage.mttr_hr = 2.0;
  sys.dc.storage = {storage};

  ThreeStageReliabilityOptions options;
  options.include_generator_faults = true;
  const auto result = run_three_stage_reliability_from_string(
      hacdcpf::io::to_json(sys, 2), options);
  REQUIRE(result.ok);
  std::vector<std::string> types;
  for (const auto& fault : result.faults) types.push_back(fault.component_type);
  for (const char* expected : {"dc_static_generator", "dc_static_generator_ac",
                               "dc_pv_array", "dc_storage"})
  {
    const auto fault = std::find_if(result.faults.begin(), result.faults.end(),
                                    [&](const auto& row) {
                                      return row.component_type == expected;
                                    });
    REQUIRE(fault != result.faults.end());
    CHECK(fault->raw_pls_stage3 > 90.0);
  }
}

TEST_CASE("Three-stage reliability — mobile storage status, energy and outage are physical",
          "[reliability][three_stage][der][mobile_storage]") {
  const auto run = [](hacdcpf::MobileStorageStatus status) {
    auto sys = make_two_bus_islanding_case(0.0, 1.0);
    sys.ac.buses.front().pd_mw = 0.4;
    sys.ac.buses.front().qd_mvar = 0.08;
    sys.ac.external_grids.front().s_sc_max_mva = 0.1;

    hacdcpf::MobileStorage storage;
    storage.index = 7;
    storage.bus = 1;
    storage.in_service = true;
    storage.status = status;
    storage.p_rated_mw = 0.3;
    storage.pmax_mw = 0.3;
    storage.qmax_mvar = 0.1;
    storage.e_rated_mwh = 1.0;
    storage.soc_init = 1.0;
    storage.soc_min = 0.0;
    storage.eta_discharge = 1.0;
    storage.grid_forming = true;
    storage.mtbf_hours = 8760.0;
    storage.mttr_hours = 2.0;
    sys.mobile_storage = {storage};

    ThreeStageReliabilityOptions options;
    options.include_generator_faults = true;
    options.enable_parallel = false;
    return run_three_stage_reliability_from_string(
        hacdcpf::io::to_json(sys, 2), options);
  };

  const auto deployed = run(hacdcpf::MobileStorageStatus::Deployed);
  REQUIRE(deployed.ok);
  const auto mobile_fault = std::find_if(
      deployed.faults.begin(), deployed.faults.end(), [](const auto& row) {
        return row.component_type == "mobile_storage";
      });
  REQUIRE(mobile_fault != deployed.faults.end());
  CHECK(mobile_fault->pls_stage3 == Catch::Approx(300.0).margin(1e-3));
  const auto branch_fault = std::find_if(
      deployed.faults.begin(), deployed.faults.end(), [](const auto& row) {
        return row.component_type == "ac_branch";
      });
  REQUIRE(branch_fault != deployed.faults.end());
  CHECK(branch_fault->raw_pls_stage3 == Catch::Approx(0.0).margin(1e-3));
  CHECK(branch_fault->storage_energy_used_stage1_mwh +
            branch_fault->storage_energy_used_stage2_mwh +
            branch_fault->storage_energy_used_stage3_mwh ==
        Catch::Approx(0.3).margin(1e-6));

  const auto in_transit = run(hacdcpf::MobileStorageStatus::InTransit);
  REQUIRE(in_transit.ok);
  CHECK(std::none_of(in_transit.faults.begin(), in_transit.faults.end(),
                     [](const auto& row) {
                       return row.component_type == "mobile_storage";
                     }));
  REQUIRE_FALSE(in_transit.faults.empty());
  CHECK(in_transit.faults.front().raw_pls_stage3 ==
        Catch::Approx(300.0).margin(1e-3));
}

TEST_CASE("Three-stage reliability — VPP is an outage-aware PCC boundary dispatch",
          "[reliability][three_stage][der][vpp]") {
  auto sys = make_two_bus_islanding_case(0.0, 1.0);
  sys.ac.buses.front().pd_mw = 0.4;
  sys.ac.buses.front().qd_mvar = 0.08;
  sys.ac.external_grids.front().s_sc_max_mva = 0.1;
  hacdcpf::VirtualPowerPlant vpp;
  vpp.index = 9;
  vpp.pcc_bus = 1;
  vpp.in_service = true;
  vpp.p_generation_sum_mw = 0.3;
  vpp.pmax_mw = 0.3;
  vpp.q_output_mvar = 0.08;
  vpp.mtbf_hours = 8760.0;
  vpp.mttr_hours = 2.0;
  sys.vpps = {vpp};

  ThreeStageReliabilityOptions options;
  options.include_generator_faults = true;
  options.enable_parallel = false;
  const auto result = run_three_stage_reliability_from_string(
      hacdcpf::io::to_json(sys, 2), options);
  REQUIRE(result.ok);
  const auto fault = std::find_if(result.faults.begin(), result.faults.end(),
                                  [](const auto& row) {
                                    return row.component_type ==
                                           "virtual_power_plant";
                                  });
  REQUIRE(fault != result.faults.end());
  CHECK(fault->pls_stage3 == Catch::Approx(300.0).margin(1e-3));
}

TEST_CASE("Three-stage reliability — equal AC and DC bus IDs remain isolated",
          "[reliability][three_stage][hybrid][domain_map]") {
  auto sys = make_two_bus_islanding_case(0.0, 1.0);
  hacdcpf::DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.bus_type = hacdcpf::DCBusType::DC_V;
  dc_bus.in_service = true;
  sys.dc.buses = {dc_bus};
  hacdcpf::DCLoad dc_load;
  dc_load.index = 1;
  dc_load.bus = 1;
  dc_load.in_service = true;
  dc_load.p_mw = 0.5;
  sys.dc.loads = {dc_load};

  ThreeStageReliabilityOptions options;
  options.enable_parallel = false;
  const auto result = run_three_stage_reliability_from_string(
      hacdcpf::io::to_json(sys, 2), options);
  REQUIRE(result.ok);
  REQUIRE(result.faults.size() == 1);
  CHECK(result.faults.front().raw_pls_stage3 ==
        Catch::Approx(500.0).margin(1e-3));
}

TEST_CASE("Three-stage reliability — built-in Dist33 DER uses the coupled model",
          "[reliability][three_stage][builtin][dist33_der]") {
  const auto sys = hacdcpf::io::build_dist33_microgrid_der();
  ThreeStageReliabilityOptions options;
  options.enable_parallel = false;
  const auto result = run_three_stage_reliability_from_string(
      hacdcpf::io::to_json(sys, 2), options);
  INFO("error: " << result.error);
  REQUIRE(result.ok);
  CHECK(result.model_scope ==
        "coupled-acdc-lindistflow-restoration-milp");
  CHECK(result.validity.dc_power_flow_enforced);
  CHECK(result.validity.sop_dispatch_optimised);
  CHECK(result.validity.branch_flow_enforced);
  CHECK(result.validity.apparent_power_polygon_enforced);
  CHECK(result.validity.voltage_constraints_enforced);
  CHECK(result.validity.radial_topology_enforced);
  CHECK(result.apparent_power_polygon_sides == 16);
  CHECK(result.maximum_ac_branch_apparent_power_ratio <= 1.0 + 1e-7);
  REQUIRE_FALSE(result.faults.empty());
  for (const auto& fault : result.faults)
    CHECK(fault.psop3.size() == sys.vsc_converters.size());
}

TEST_CASE("Three-stage reliability rejects an invalid apparent-power polygon",
          "[reliability][three_stage][thermal_limit][validation]") {
  auto sys = make_two_bus_islanding_case();
  ThreeStageReliabilityOptions options;
  options.enable_parallel = false;
  options.apparent_power_polygon_sides = 5;
  CHECK_THROWS_AS(run_three_stage_reliability_from_string(
                      hacdcpf::io::to_json(sys, 2), options),
                  std::invalid_argument);
}

TEST_CASE("Three-stage reliability — custom protection conditions sustained scenarios",
          "[reliability][three_stage][protection_configuration]") {
  const char* json = R"json({
    "name":"protection_conditioning", "base_mva":10.0,
    "ac":{"base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":10}
      ],
      "branches":[
        {"index":1,"name":"ProtectedLine","from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":2.0,"pmax_mw":2.0,"pmin_mw":0.0,"qmax_mvar":2.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[],
      "circuit_breakers":[{"index":10,"name":"PrimaryCB","bus_from":1,"bus_to":2,"closed":true,"in_service":true}]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  hacdcpf::analysis::ThreeStageReliabilityOptions baseline_options;
  baseline_options.enable_parallel = false;
  const auto baseline =
      hacdcpf::analysis::run_three_stage_reliability_from_string(
          json, baseline_options);
  REQUIRE(baseline.ok);
  REQUIRE(baseline.faults.size() == 1);

  hacdcpf::analysis::ProtectionConfiguration protection;
  protection.protection_id = "P-main";
  protection.protective_device_id = "ac_circuit_breaker:10";
  protection.protected_component_id = "ac_branch:1";
  protection.zone_component_ids = {"ac_branch:1"};
  protection.automatic_reclose = true;
  protection.successful_reclose_probability = 0.8;
  protection.primary_clearing_time_s = 0.12;

  auto configured_options = baseline_options;
  configured_options.reliability_configuration.protection = {protection};
  const auto configured =
      hacdcpf::analysis::run_three_stage_reliability_from_string(
          json, configured_options);
  REQUIRE(configured.ok);
  REQUIRE(configured.protection_configuration_applied);
  REQUIRE(configured.protection_rows_applied == 1);
  REQUIRE(configured.faults.size() == 1);
  const auto& scenario = configured.faults.front();
  CHECK(scenario.protection_scenario == "primary_cleared");
  CHECK(scenario.scenario_probability == Catch::Approx(0.2).margin(1e-12));
  CHECK(scenario.failure_rate == Catch::Approx(0.2).margin(1e-12));
  CHECK(scenario.initiating_failure_rate == Catch::Approx(1.0));
  CHECK(scenario.clearing_time_s == Catch::Approx(0.12));
  CHECK(scenario.tau_iso_hr == Catch::Approx(0.12 / 3600.0).margin(1e-12));
  CHECK(configured.transient_reclose_frequency_per_year ==
        Catch::Approx(0.8).margin(1e-12));
  CHECK(configured.sustained_fault_frequency_per_year ==
        Catch::Approx(0.2).margin(1e-12));
  CHECK(configured.eens_kwh_yr ==
        Catch::Approx(0.2 * baseline.eens_kwh_yr).margin(1e-6));
}

TEST_CASE("Three-stage reliability — backup protection expands the outage zone",
          "[reliability][three_stage][protection_configuration]") {
  const char* json = R"json({
    "name":"backup_zone", "base_mva":10.0,
    "ac":{"base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":10},
        {"index":3,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":10}
      ],
      "branches":[
        {"index":1,"name":"ProtectedLine","from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0},
        {"index":2,"name":"BackupZoneLine","from_bus":1,"to_bus":3,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":0.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":3.0,"pmax_mw":3.0,"pmin_mw":0.0,"qmax_mvar":3.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[],
      "circuit_breakers":[
        {"index":10,"name":"PrimaryCB","bus_from":1,"bus_to":2,"closed":true,"in_service":true},
        {"index":11,"name":"BackupCB","bus_from":1,"bus_to":3,"closed":true,"in_service":true}
      ]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  hacdcpf::analysis::ProtectionConfiguration protection;
  protection.protection_id = "P-zone";
  protection.protective_device_id = "ac_circuit_breaker:10";
  protection.protected_component_id = "ac_branch:1";
  protection.backup_device_id = "ac_circuit_breaker:11";
  protection.zone_component_ids = {"ac_branch:1", "ac_branch:2"};
  protection.fail_to_trip_probability = 0.5;
  protection.primary_clearing_time_s = 0.1;
  protection.backup_clearing_time_s = 0.5;

  hacdcpf::analysis::ThreeStageReliabilityOptions options;
  options.enable_parallel = false;
  options.reliability_configuration.protection = {protection};
  const auto result =
      hacdcpf::analysis::run_three_stage_reliability_from_string(json, options);
  REQUIRE(result.ok);
  REQUIRE(result.faults.size() == 3); // two scenarios for line 1, line 2 unchanged
  const hacdcpf::analysis::ThreeStageFaultDetail* primary = nullptr;
  const hacdcpf::analysis::ThreeStageFaultDetail* backup = nullptr;
  for (const auto& fault : result.faults) {
    if (fault.protection_scenario == "primary_cleared") primary = &fault;
    if (fault.protection_scenario == "backup_cleared") backup = &fault;
  }
  REQUIRE(primary != nullptr);
  REQUIRE(backup != nullptr);
  CHECK(primary->failure_rate == Catch::Approx(0.5).margin(1e-12));
  CHECK(backup->failure_rate == Catch::Approx(0.5).margin(1e-12));
  CHECK(primary->clearing_time_s == Catch::Approx(0.1));
  CHECK(backup->clearing_time_s == Catch::Approx(0.5));
  CHECK(backup->backup_device_id == "ac_circuit_breaker:11");
  CHECK(backup->pls_stage3 > primary->pls_stage3 + 900.0);
  CHECK(result.initiating_fault_frequency_per_year ==
        Catch::Approx(result.sustained_fault_frequency_per_year).margin(1e-12));
}

// Hidden diagnostic for the theory-guided cost prediction: two sustained
// protection scenarios should remain within the same order as one scenario.
TEST_CASE("Protection scenario runtime probe", "[.protection_runtime_probe]") {
  const char* json = R"json({
    "name":"protection_runtime_probe", "base_mva":10.0,
    "ac":{"base_mva":10.0,
      "buses":[
        {"index":1,"bus_type":1,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":0.0,"qd_mvar":0.0,"n_customers":1},
        {"index":2,"bus_type":3,"base_kv":10.0,"vm_pu":1.0,"vmin_pu":0.95,"vmax_pu":1.05,"in_service":true,"pd_mw":1.0,"qd_mvar":0.0,"n_customers":10}
      ],
      "branches":[
        {"index":1,"name":"ProtectedLine","from_bus":1,"to_bus":2,"r_pu":0.001,"x_pu":0.001,"rate_a_mva":10.0,"in_service":true,"failure_rate":1.0,"mttr_hr":1.0}
      ],
      "loads":[],"external_grids":[],
      "generators":[{"index":1,"bus":1,"in_service":true,"pg_mw":2.0,"pmax_mw":2.0,"pmin_mw":0.0,"qmax_mvar":2.0,"qmin_mvar":0.0}],
      "static_generators":[],"renewable_gens":[],"pv_systems":[],"storage":[],"switches":[],
      "circuit_breakers":[{"index":10,"name":"PrimaryCB","bus_from":1,"bus_to":2,"closed":true,"in_service":true}]
    },
    "dc":{"buses":[],"branches":[],"loads":[],"storage":[],"static_generators":[],"dc_static_generators":[],"pv_arrays":[],"dc_circuit_breakers":[]},
    "vsc_converters":[],"dcdc_converters":[]
  })json";

  hacdcpf::analysis::ThreeStageReliabilityOptions baseline;
  baseline.enable_parallel = false;
  auto primary = baseline;
  hacdcpf::analysis::ProtectionConfiguration row;
  row.protection_id = "P-main";
  row.protective_device_id = "ac_circuit_breaker:10";
  row.protected_component_id = "ac_branch:1";
  row.zone_component_ids = {"ac_branch:1"};
  row.primary_clearing_time_s = 0.1;
  row.backup_clearing_time_s = 0.5;
  primary.reliability_configuration.protection = {row};
  auto primary_backup = primary;
  primary_backup.reliability_configuration.protection[0]
      .fail_to_trip_probability = 0.5;

  const auto measure = [&](const auto& options) {
    std::vector<double> seconds;
    std::size_t scenarios = 0;
    for (int repeat = 0; repeat < 7; ++repeat) {
      const auto started = std::chrono::steady_clock::now();
      const auto result =
          hacdcpf::analysis::run_three_stage_reliability_from_string(
              json, options);
      const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count();
      REQUIRE(result.ok);
      scenarios = result.faults.size();
      if (repeat >= 2) seconds.push_back(elapsed);
    }
    std::sort(seconds.begin(), seconds.end());
    return std::pair{seconds[seconds.size() / 2], scenarios};
  };

  const auto baseline_result = measure(baseline);
  const auto primary_result = measure(primary);
  const auto backup_result = measure(primary_backup);
  std::cout << "PROTECTION_RUNTIME baseline=" << baseline_result.first
            << " primary=" << primary_result.first
            << " primary_backup=" << backup_result.first
            << " primary_to_baseline="
            << primary_result.first / baseline_result.first
            << " backup_to_primary="
            << backup_result.first / primary_result.first
            << " scenarios=" << baseline_result.second << ','
            << primary_result.second << ',' << backup_result.second << '\n';
}
