// test_crossmodule_integration.cpp
//
// Comprehensive cross-module pipeline integration test.
//
// Results are explicitly transferred between modules; each transfer point
// has a dedicated cross-check that detects high-level contradictions.
//
// ── Pipeline structure ────────────────────────────────────────────────────────
//
//  PIPELINE 1 — IEEE 14-bus transmission: OPF → PF → Carbon
//
//    Stage 0 (reference):  Base Newton PF on case14.m.
//    Stage 1 (AC OPF):     Optimal dispatch {Pg, Qg, Vm} from AC OPF.
//    Stage 2 (Newton PF):  OPF dispatch applied to system → Newton PF.
//    Stage 3 (Carbon):     Stage-2 PF result fed into carbon analysis.
//
//    Transfer cross-checks:
//      P1-A1  Base PF converges (sanity reference).
//      P1-A2  AC OPF converges; objective ≥ 0.
//      P1-A3  DC OPF converges; objective ≥ 0.
//      P1-A4  DC OPF cost ≤ AC OPF cost × 1.05 (relaxation property).
//      P1-A5  AC OPF dispatch within [Pmin, Pmax] for each generator.
//      P1-A6  [Stage 1→2] Newton PF converges with OPF dispatch
//             (OPF feasibility cross-check: if OPF found a feasible point,
//              the full nonlinear PF equations must also converge there).
//      P1-A7  [Stage 1→2] Per-bus voltage consistency: |Vm_PF − Vm_OPF|
//             < 0.05 pu  (OPF operating point ≈ Newton PF solution).
//      P1-B1  [Stage 2→3] Carbon tracing balance error < 1 % on pipeline PF.
//      P1-B2  [Stage 2→3] Carbon matrix  balance error < 1 % on pipeline PF.
//
//  PIPELINE 2 — IEEE 33-bus BW distribution: ONR → DC OPF → PF → Carbon
//
//    Stage 0 (reference):  Base Newton PF + DC OPF on original topology.
//    Stage 1 (ONR):        Optimal radial topology {open/closed branch IDs}.
//    Stage 2 (DC OPF):     ONR topology applied → DC OPF dispatch {Pg}.
//    Stage 3 (Newton PF):  DC OPF dispatch applied to reconfigured system
//                          → Newton PF operating point.
//    Stage 4 (Carbon):     Stage-3 PF result fed into carbon analysis.
//
//    Transfer cross-checks:
//      P2-C1  Base PF converges (reference).
//      P2-C2  [Stage 0→1] Base DC OPF converges (reference cost).
//      P2-C3  [Stage 1] ONR finds feasible radial spanning-tree topology.
//      P2-C4  [Stage 1→2] DC OPF converges on reconfigured topology.
//      P2-C5  [Stage 1→2] DC OPF cost on reconfigured topology ≤ base DC
//             OPF cost × 1.05 (ONR lowers dispatch cost via loss reduction).
//      P2-C6  [Stage 2] DC OPF dispatch within [Pmin, Pmax].
//      P2-C7  [Stage 2→3] Newton PF converges with OPF dispatch on
//             reconfigured topology  (OPF feasibility after ONR).
//      P2-C8  [Stage 2→3] Pipeline PF losses ≤ base PF losses × 1.10
//             (ONR loss improvement preserved through OPF dispatch).
//      P2-D1  [Stage 3→4] Carbon tracing balance error < 1 % on pipeline PF.
//      P2-D2  [Stage 3→4] Carbon intensity at every load bus ≤ emission
//             factor × 1.01  (single-source network: no amplification).
//
// ── High-level contradictions detected ───────────────────────────────────────
//   - OPF feasibility (P1-A6 / P2-C7): OPF converges but Newton PF diverges.
//   - OPF–PF voltage mismatch (P1-A7):  OPF operating point ≠ Newton solution.
//   - OPF relaxation breach (P1-A4):    DC OPF cost > AC OPF cost.
//   - Generator limit breach (P1-A5 / P2-C6): dispatch outside [Pmin,Pmax].
//   - ONR cost regression (P2-C5):      reconfiguration increases DC OPF cost.
//   - ONR loss regression (P2-C8):      reconfigured losses > base losses.
//   - Carbon tracer error (P1-B1/B2, P2-D1): generation ≠ load+loss.
//   - Carbon source purity (P2-D2):     single-source intensity amplified.

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/network_reconfiguration/topology_analysis.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

// ── Namespaces ────────────────────────────────────────────────────────────────
// Only pull in the top-level namespace and io; use opf:: and analysis:: prefixes
// to avoid the ambiguity between hacdcpf::solve_dc_opf (api wrapper) and
// hacdcpf::opf::solve_dc_opf (native) that arises when both are imported.
using namespace hacdcpf;
using namespace hacdcpf::io;

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Sum of active branch losses from a converged PowerFlowResult.
static double total_active_loss_mw(const PowerFlowResult& pf) {
  double loss = 0.0;
  for (const auto& bf : pf.branch_flows)
    loss += bf.pf_mw + bf.pt_mw;
  return loss;
}

/// Transfer: apply ONR open/closed topology to a copy of a system.
/// Uses the stable ACBranch::index identifiers from the ONRResult.
static HybridPowerSystem apply_onr_topology(const HybridPowerSystem& sys,
                                             const analysis::ONRResult& onr) {
  HybridPowerSystem result = sys;
  for (auto& br : result.ac.branches) {
    const bool is_open =
        std::find(onr.open_branch_ids.begin(), onr.open_branch_ids.end(),
                  br.index) != onr.open_branch_ids.end();
    br.in_service = !is_open;
  }
  return result;
}

/// Transfer: apply AC OPF dispatch {Pg, Qg, Vg} to a copy of a system.
/// Setting Vg on PV buses ensures Newton PF starts from the OPF operating point.
static HybridPowerSystem apply_acopf_dispatch(const HybridPowerSystem& sys,
                                               const opf::ACOPFResult& opf_res) {
  HybridPowerSystem result = sys;
  const size_t ng = result.ac.generators.size();
  const size_t nb = result.ac.buses.size();
  for (size_t gi = 0; gi < ng && gi < opf_res.pg_mw.size(); ++gi) {
    result.ac.generators[gi].pg_mw = opf_res.pg_mw[gi];
    if (gi < opf_res.qg_mvar.size())
      result.ac.generators[gi].qg_mvar = opf_res.qg_mvar[gi];
    const int  bus_id = result.ac.generators[gi].bus;
    const auto bi     = static_cast<size_t>(bus_id - 1);
    if (bi < nb && bi < opf_res.vm.size())
      result.ac.generators[gi].vg_pu = opf_res.vm[bi];
  }
  return result;
}

/// Transfer: apply DC OPF dispatch {Pg} to a copy of a system.
static HybridPowerSystem apply_dcopf_dispatch(const HybridPowerSystem& sys,
                                               const opf::DCOPFResult& opf_res) {
  HybridPowerSystem result = sys;
  const size_t ng = result.ac.generators.size();
  for (size_t gi = 0; gi < ng && gi < opf_res.pg_mw.size(); ++gi)
    result.ac.generators[gi].pg_mw = opf_res.pg_mw[gi];
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// PIPELINE 1 — IEEE 14-bus: OPF → PF → Carbon
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("Pipeline 1: OPF → PF → Carbon (IEEE 14-bus)",
          "[integration][crossval][crossmodule][pipeline]") {
  // Load case14.m (has generator cost coefficients for meaningful OPF).
  const std::string case14_path =
      std::string(HACDCPF_TEST_DATA_DIR) + "/case14.m";
  HybridPowerSystem sys_base;
  try {
    sys_base = io::parse_matpower(case14_path);
  } catch (...) {
    WARN("case14.m not found — skipping Pipeline-1 test");
    return;
  }

  REQUIRE_FALSE(sys_base.ac.generators.empty());
  REQUIRE_FALSE(sys_base.ac.buses.empty());

  const size_t ng = sys_base.ac.generators.size();
  const size_t nb = sys_base.ac.buses.size();

  // Emission factors: loosely fuel-type-based (coal/gas/CCGT/oil/steam).
  static constexpr double kEF[] = {0.90, 0.50, 0.40, 0.60, 0.70};
  HybridPowerSystem sys_carbon = sys_base;
  for (size_t gi = 0; gi < ng; ++gi)
    sys_carbon.ac.generators[gi].emission_factor_tco2_mwh = kEF[gi % 5];

  PowerFlowOptions pf_opt;
  pf_opt.max_iter = 100;
  pf_opt.tol      = 1e-8;

  // ── Stage 0: Reference base PF ────────────────────────────────────────────
  const PowerFlowResult pf_base = solve_power_flow(sys_base, pf_opt);
  INFO("P1-A1  Base PF residual: " << pf_base.residual);
  REQUIRE(pf_base.converged);  // P1-A1

  const double loss_base_mw = total_active_loss_mw(pf_base);
  INFO("Reference base PF total active loss: " << loss_base_mw << " MW");

  // ── Stage 1: AC OPF → optimal dispatch {Pg, Qg, Vm} ─────────────────────
  opf::ACOPFOptions acopf_opt;
  acopf_opt.max_inner_iterations = 100;
  acopf_opt.max_outer_iterations = 10;
  acopf_opt.feasibility_tol      = 1e-6;
  acopf_opt.use_parity_ipm       = false;
  acopf_opt.verbose              = false;

  opf::DCOPFOptions dcopf_opt;
  dcopf_opt.feasibility_tol       = 1e-6;
  dcopf_opt.include_branch_limits = true;
  dcopf_opt.solver                = opf::DCOPFSolverBackend::NativeQP;
  dcopf_opt.verbose               = false;

  const opf::ACOPFResult acopf = opf::solve_ac_opf(sys_base, acopf_opt);
  const opf::DCOPFResult dcopf = opf::solve_dc_opf(sys_base, dcopf_opt);

  INFO("P1-A2  AC OPF  converged=" << acopf.converged
                                   << "  obj=" << acopf.objective);
  INFO("P1-A3  DC OPF  converged=" << dcopf.converged
                                   << "  obj=" << dcopf.objective);

  REQUIRE(acopf.converged);              // P1-A2
  CHECK(acopf.objective >= 0.0);
  REQUIRE(dcopf.converged);              // P1-A3
  CHECK(dcopf.objective >= 0.0);

  // P1-A4: DC OPF is a relaxation of AC OPF → DC cost ≤ AC cost.
  INFO("P1-A4  DC OPF obj=" << dcopf.objective
                             << "  AC OPF obj=" << acopf.objective);
  CHECK(dcopf.objective <= acopf.objective * 1.05 + 1.0);  // P1-A4

  // P1-A5: AC OPF dispatch within [Pmin, Pmax] for each generator.
  for (size_t gi = 0; gi < ng && gi < acopf.pg_mw.size(); ++gi) {
    if (!sys_base.ac.generators[gi].in_service) continue;
    const double pg   = acopf.pg_mw[gi];
    const double pmin = sys_base.ac.generators[gi].pmin_mw;
    const double pmax = sys_base.ac.generators[gi].pmax_mw;
    INFO("P1-A5  Gen " << gi << "  Pg=" << pg
                       << " MW  [" << pmin << ", " << pmax << "]");
    CHECK(pg >= pmin - 0.1);  // P1-A5 (0.1 MW numerical tolerance)
    CHECK(pg <= pmax + 0.1);
  }

  // ── Stage 2: Transfer OPF dispatch → Newton PF ────────────────────────────
  // P1-A6: Newton PF must converge at the OPF operating point.
  // P1-A7: Per-bus voltages must match OPF voltages within 0.05 pu.
  if (!acopf.pg_mw.empty()) {
    const HybridPowerSystem sys_opf_dispatched =
        apply_acopf_dispatch(sys_carbon, acopf);
    const PowerFlowResult pf_opf = solve_power_flow(sys_opf_dispatched, pf_opt);

    INFO("P1-A6  PF(OPF dispatch) residual: " << pf_opf.residual);
    REQUIRE(pf_opf.converged);  // P1-A6: OPF feasibility via Newton PF

    // P1-A7: OPF voltage consistency — operating point from OPF must match
    // the Newton PF solution.  A discrepancy > 0.05 pu indicates the OPF
    // solver found a point that does not satisfy the full AC power flow.
    for (size_t bi = 0; bi < nb &&
                        bi < pf_opf.vm.size() &&
                        bi < acopf.vm.size(); ++bi) {
      const double diff = std::abs(pf_opf.vm[bi] - acopf.vm[bi]);
      INFO("P1-A7  Bus " << bi
                         << "  Vm_PF=" << pf_opf.vm[bi]
                         << "  Vm_OPF=" << acopf.vm[bi]
                         << "  diff=" << diff << " pu");
      CHECK(diff < 0.05);  // P1-A7
    }

    INFO("Pipeline-1 PF loss (OPF dispatch): "
         << total_active_loss_mw(pf_opf) << " MW"
         << "  (base: " << loss_base_mw << " MW)");

    // ── Stage 3: Transfer PF result → Carbon Analysis ─────────────────────
    // P1-B1/B2: Carbon balance on the pipeline endpoint (OPF dispatch, Newton PF).
    analysis::CarbonAnalysisOptions ca_opt;
    ca_opt.verbose = false;

    const auto ca = analysis::compute_carbon_analysis(
        sys_opf_dispatched, pf_opf, ca_opt);

    INFO("P1-B1  Pipeline carbon tracing balance error: "
         << ca.tracing_summary.balance_error_pct << " %");
    CHECK(ca.tracing_summary.balance_error_pct < 1.0);  // P1-B1

    if (ca.matrix_solved) {
      INFO("P1-B2  Pipeline carbon matrix balance error: "
           << ca.matrix_summary.balance_error_pct << " %");
      CHECK(ca.matrix_summary.balance_error_pct < 1.0);  // P1-B2
    }

    // Informational: total generation emissions at OPF dispatch vs. base.
    const auto ca_base = analysis::compute_carbon_analysis(
        sys_carbon, pf_base, ca_opt);
    INFO("Pipeline-1 total gen emissions (OPF dispatch): "
         << ca.tracing_summary.total_generation_emissions_tco2 << " tCO2");
    INFO("Pipeline-1 total gen emissions (base dispatch): "
         << ca_base.tracing_summary.total_generation_emissions_tco2 << " tCO2");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// PIPELINE 2 — IEEE 33-bus BW: ONR → DC OPF → PF → Carbon
// ─────────────────────────────────────────────────────────────────────────────
TEST_CASE("Pipeline 2: ONR → DC OPF → PF → Carbon (IEEE 33-bus BW)",
          "[integration][crossval][crossmodule][pipeline]") {
  // ── Setup ──────────────────────────────────────────────────────────────────
  const HybridPowerSystem sys_hybrid = build_case33bw_acdc();
  const HybridPowerSystem sys_ac     = build_ac_only_version(sys_hybrid);
  REQUIRE_FALSE(sys_ac.ac.buses.empty());
  REQUIRE_FALSE(sys_ac.ac.branches.empty());

  const size_t ng = sys_ac.ac.generators.size();

  // case33bw has gencost [2, 0, 0, 3, 0, 20, 0] → c1 = $20/MWh.
  // Emission factor: 0.6 tCO₂/MWh (representative grid average).
  HybridPowerSystem sys_costed = sys_ac;
  for (auto& g : sys_costed.ac.generators)
    if (g.in_service)
      g.emission_factor_tco2_mwh = 0.6;

  PowerFlowOptions pf_opt;
  pf_opt.max_iter = 200;
  pf_opt.tol      = 1e-8;

  opf::DCOPFOptions dcopf_opt;
  dcopf_opt.feasibility_tol       = 1e-6;
  dcopf_opt.include_branch_limits = false;  // distribution lines have no MVA ratings
  dcopf_opt.solver                = opf::DCOPFSolverBackend::NativeQP;
  dcopf_opt.verbose               = false;

  // ── Stage 0: Reference base PF and DC OPF on original topology ───────────
  const PowerFlowResult pf_base = solve_power_flow(sys_costed, pf_opt);
  INFO("P2-C1  Base PF residual: " << pf_base.residual);
  REQUIRE(pf_base.converged);  // P2-C1

  const double loss_base_mw = total_active_loss_mw(pf_base);
  INFO("Reference base PF total active loss: " << loss_base_mw << " MW");

  const opf::DCOPFResult dcopf_base = opf::solve_dc_opf(sys_costed, dcopf_opt);
  INFO("P2-C2  Base DC OPF  converged=" << dcopf_base.converged
                                        << "  obj=" << dcopf_base.objective);
  REQUIRE(dcopf_base.converged);  // P2-C2
  CHECK(dcopf_base.objective >= 0.0);

  // ── Stage 1: ONR → reconfigured radial topology ───────────────────────────
  analysis::ONROptions onr_opt;
  onr_opt.v_min_pu   = 0.90;
  onr_opt.v_max_pu   = 1.10;
  onr_opt.mip_gap    = 0.02;
  onr_opt.max_time_s = 120;
  onr_opt.verbose    = false;

  const analysis::ONRResult onr =
      analysis::solve_optimal_reconfiguration(sys_costed.ac, onr_opt);
  INFO(onr.summary());

  REQUIRE(onr.feasible);  // P2-C3 — ONR must find a feasible topology

  // Spanning-tree structural check.
  const HybridPowerSystem sys_reconfig = apply_onr_topology(sys_costed, onr);
  REQUIRE(analysis::is_radial(sys_reconfig.ac));
  REQUIRE(static_cast<int>(onr.closed_branch_ids.size()) ==
          static_cast<int>(sys_costed.ac.buses.size()) - 1);

  // ── Stage 2: Transfer ONR topology → DC OPF on reconfigured system ────────
  // P2-C4: DC OPF must converge on the reconfigured topology.
  // P2-C5: DC OPF cost on reconfigured topology ≤ base DC OPF cost.
  //        ONR reduces active losses → less generation → lower dispatch cost.
  const opf::DCOPFResult dcopf_reconfig =
      opf::solve_dc_opf(sys_reconfig, dcopf_opt);

  INFO("P2-C4  Reconfigured DC OPF  converged=" << dcopf_reconfig.converged
                                                 << "  obj=" << dcopf_reconfig.objective);
  REQUIRE(dcopf_reconfig.converged);  // P2-C4

  INFO("P2-C5  DC OPF cost: base=" << dcopf_base.objective
                                    << "  reconfig=" << dcopf_reconfig.objective);
  CHECK(dcopf_reconfig.objective <= dcopf_base.objective * 1.05 + 1.0);  // P2-C5

  // P2-C6: DC OPF dispatch within [Pmin, Pmax].
  for (size_t gi = 0; gi < ng && gi < dcopf_reconfig.pg_mw.size(); ++gi) {
    if (!sys_reconfig.ac.generators[gi].in_service) continue;
    const double pg   = dcopf_reconfig.pg_mw[gi];
    const double pmin = sys_reconfig.ac.generators[gi].pmin_mw;
    const double pmax = sys_reconfig.ac.generators[gi].pmax_mw;
    INFO("P2-C6  Gen " << gi << "  Pg=" << pg
                       << " MW  [" << pmin << ", " << pmax << "]");
    CHECK(pg >= pmin - 0.1);  // P2-C6
    CHECK(pg <= pmax + 0.1);
  }

  // ── Stage 3: Transfer DC OPF dispatch → Newton PF on reconfigured system ──
  // P2-C7: Newton PF must converge (OPF feasibility on reconfigured topology).
  // P2-C8: Pipeline losses ≤ base losses × 1.10 (ONR improvement preserved).
  const HybridPowerSystem sys_pipeline =
      apply_dcopf_dispatch(sys_reconfig, dcopf_reconfig);
  const PowerFlowResult pf_pipeline = solve_power_flow(sys_pipeline, pf_opt);

  INFO("P2-C7  Pipeline PF residual: " << pf_pipeline.residual);
  REQUIRE(pf_pipeline.converged);  // P2-C7

  const double loss_pipeline_mw = total_active_loss_mw(pf_pipeline);
  INFO("P2-C8  Loss: base=" << loss_base_mw
                             << " MW  pipeline=" << loss_pipeline_mw << " MW");
  CHECK(loss_pipeline_mw <= loss_base_mw * 1.10);  // P2-C8

  // ── Stage 4: Transfer PF result → Carbon Analysis ─────────────────────────
  // P2-D1: Carbon tracing balance error on pipeline PF < 1 %.
  // P2-D2: Carbon intensity at every load bus ≤ emission_factor × 1.01.
  //        A single-source network cannot amplify carbon intensity above the
  //        source emission rate; violation indicates a tracer bug.
  analysis::CarbonAnalysisOptions ca_opt;
  ca_opt.verbose = false;

  const auto ca = analysis::compute_carbon_analysis(
      sys_pipeline, pf_pipeline, ca_opt);

  INFO("P2-D1  Pipeline carbon tracing balance error: "
       << ca.tracing_summary.balance_error_pct << " %");
  CHECK(ca.tracing_summary.balance_error_pct < 1.0);  // P2-D1

  static constexpr double kEF_source = 0.6;  // tCO₂/MWh set on the slack generator
  for (const auto& lc : ca.load_carbon) {
    INFO("P2-D2  Load bus " << lc.bus
                            << "  carbon_intensity=" << lc.carbon_intensity_tco2_mwh
                            << " tCO2/MWh  (source=" << kEF_source << ")");
    CHECK(lc.carbon_intensity_tco2_mwh <= kEF_source * 1.01);  // P2-D2
  }

  // Informational: pipeline endpoint summary.
  const auto ca_base = analysis::compute_carbon_analysis(
      sys_costed, pf_base, ca_opt);
  INFO("Pipeline-2 total gen emissions (base):     "
       << ca_base.tracing_summary.total_generation_emissions_tco2 << " tCO2");
  INFO("Pipeline-2 total gen emissions (pipeline): "
       << ca.tracing_summary.total_generation_emissions_tco2 << " tCO2");
  INFO("Pipeline-2 DC OPF cost reduction: "
       << (dcopf_base.objective - dcopf_reconfig.objective)
       << " $/h  ("
       << 100.0 * (dcopf_base.objective - dcopf_reconfig.objective) /
              (dcopf_base.objective + 1e-9)
       << " %)");
}
