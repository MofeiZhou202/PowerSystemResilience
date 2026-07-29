/// tests/test_bpa_dsp_compare.cpp
/// ================================
/// End-to-end integration comparison against the DSP reference solutions in
/// data/dsp (dat card -> import -> unified Newton power flow -> DSP SOL):
///
///   * 39.dat / IEEE90.dat   : pure-AC cases; all-bus |dV| / |d_delta|
///                             against 39NEW.SOL / IEEE90NEW.SOL.
///   * 2DC.dat / cigre.dat   : two-terminal LCC HVDC cases (default import =
///                             native LccQuasiSteady model).  Bus voltages
///                             against 2DCNEW.SOL / cigreNEW.SOL and the LCC
///                             station operating point against the DSP .pf
///                             report values (hard-coded below from
///                             2DC.pf / cigre.pf, ">>>常规直流换流器" and
///                             ">>>直流线路潮流" sections).
///
/// DSP reference operating point (identical for 2DC and cigre):
///   rectifier : alpha = 15.00 deg, Vdc = 500 kV, Id = 3000 A, P_ac = -1500 MW
///   inverter  : gamma = 17.00 deg, Vdc = 470 kV, Id = 3000 A, P_ac = +1410 MW
///   DC line loss = 90 MW (10 ohm * (3 kA)^2).
///
/// R-CARD TAP-CONTROL NOTE:
/// DSP holds alpha = AlphaN = 15 deg and gamma = GamaN = 17 deg by moving the
/// existing converter-transformer taps within each R-card range. The unified
/// power-flow outer loop models the same action; it does not add a second
/// transformer. The checks below keep three reactive-power quantities
/// separate: valve-side LCC consumption, transformer leakage consumption,
/// and their sum at the primary AC terminal.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/bpa_io.hpp"

using Catch::Matchers::WithinAbs;

namespace {

constexpr double kDeg = 180.0 / 3.14159265358979323846;

std::string dsp_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/dsp/" + name;
}

struct SolRow {
  std::string name;   // raw 8-column name (GBK bytes for IEEE90 — unused there)
  double base_kv{0.0};
  double vm_pu{0.0};
  double va_deg{0.0};
  int seq{0};         // DSP internal sequence number
};

/// Parse a DSP .SOL bus-voltage listing:
///   BPASOLV1.0 / >>>Bus Voltage header, then rows of
///   <name:8> <base kV> <vm pu> <va deg> <internal seq>, ended by EOD.
std::vector<SolRow> parse_sol(const std::string& path) {
  std::ifstream in(path);
  REQUIRE(in.good());
  std::vector<SolRow> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind("EOD", 0) == 0) break;
    if (line.size() < 30 || line[0] == '>' || line.rfind("BPASOLV", 0) == 0) {
      continue;
    }
    SolRow row;
    row.name = line.substr(0, 8);
    while (!row.name.empty() && row.name.back() == ' ') row.name.pop_back();
    std::istringstream rest(line.substr(8));
    if (!(rest >> row.base_kv >> row.vm_pu >> row.va_deg >> row.seq)) continue;
    rows.push_back(row);
  }
  return rows;
}

/// Compare a solved power flow against a SOL reference, matching buses by
/// name (ASCII SOL files).  Buses absent from the SOL (e.g. the valve-side
/// LCC stub buses, which DSP does not list) are skipped; every SOL row must
/// be matched.  Returns (max |dV| pu, max |d_delta| deg).
std::pair<double, double> compare_vs_sol_by_name(
    const hacdcpf::HybridPowerSystem& sys,
    const hacdcpf::PowerFlowResult& pf,
    const std::vector<SolRow>& sol) {
  double max_dv = 0.0, max_da = 0.0;
  size_t matched = 0;
  for (const auto& row : sol) {
    const hacdcpf::ACBus* bus = nullptr;
    for (const auto& b : sys.ac.buses) {
      if (b.name == row.name) {
        bus = &b;
        break;
      }
    }
    REQUIRE(bus != nullptr);
    const size_t i = static_cast<size_t>(bus->index) - 1;
    REQUIRE(i < pf.vm.size());
    max_dv = std::max(max_dv, std::abs(pf.vm[i] - row.vm_pu));
    max_da = std::max(max_da, std::abs(pf.va[i] * kDeg - row.va_deg));
    ++matched;
  }
  REQUIRE(matched == sol.size());
  return {max_dv, max_da};
}

}  // namespace

TEST_CASE("DSP compare: 39-bus PF matches 39NEW.SOL", "[bpa][dsp]") {
  auto res = hacdcpf::io::parse_bpa_dat(dsp_path("39.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto pf = hacdcpf::solve_power_flow(res.system);
  REQUIRE(pf.converged);

  const auto sol = parse_sol(dsp_path("39NEW.SOL"));
  REQUIRE(sol.size() == res.system.ac.buses.size());
  const auto [max_dv, max_da] = compare_vs_sol_by_name(res.system, pf, sol);
  REQUIRE(max_dv <= 2e-3);
  REQUIRE(max_da <= 0.5);
}

TEST_CASE("DSP compare: IEEE90 PF matches IEEE90NEW.SOL", "[bpa][dsp]") {
  auto res = hacdcpf::io::parse_bpa_dat(dsp_path("IEEE90.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto pf = hacdcpf::solve_power_flow(res.system);
  REQUIRE(pf.converged);

  // IEEE90NEW.SOL carries GBK-encoded Chinese names; match by the DSP
  // internal sequence number, which follows the bus card order and therefore
  // equals the imported ACBus.index.
  const auto sol = parse_sol(dsp_path("IEEE90NEW.SOL"));
  REQUIRE(sol.size() == res.system.ac.buses.size());
  double max_dv = 0.0, max_da = 0.0;
  for (const auto& row : sol) {
    const size_t i = static_cast<size_t>(row.seq) - 1;
    REQUIRE(i < pf.vm.size());
    REQUIRE(res.system.ac.buses[i].index == row.seq);
    max_dv = std::max(max_dv, std::abs(pf.vm[i] - row.vm_pu));
    max_da = std::max(max_da, std::abs(pf.va[i] * kDeg - row.va_deg));
  }
  // Same precedent as test_bpa_io: DSP applies its own shunt/Q-limit
  // corrections the importer does not replicate (~2.6e-3 pu residual).
  REQUIRE(max_dv <= 5e-3);
  REQUIRE(max_da <= 0.5);
}

namespace {

/// Shared checks for the two LCC HVDC cases (2DC, cigre).  Both run the same
/// schedule (Psch = 1500 MW, Vdr = 500 kV, AlphaN = 15, GamaN = 17, R = 10
/// ohm) and DSP reports the same operating point for both.
void check_lcc_case(const std::string& dat,
                    const std::string& sol_file,
                    double v_tol_pu,
                    double a_tol_deg,
                    double rect_tap_pu,
                    double inv_tap_pu,
                    double transformer_q_loss_mvar) {
  auto res = hacdcpf::io::parse_bpa_dat(dsp_path(dat));  // default: LccQuasiSteady
  REQUIRE_FALSE(res.report.has_errors());
  REQUIRE(res.system.lcc_converters.size() == 2);
  REQUIRE(res.system.vsc_converters.empty());

  const auto pf = hacdcpf::solve_power_flow(res.system);
  INFO("termination: " + pf.diagnostics.termination_reason);
  for (const auto& w : pf.diagnostics.warnings) INFO("warning: " + w);
  REQUIRE(pf.converged);

  // AC bus voltages vs DSP SOL (valve-side stub buses are not listed by DSP
  // and are skipped). The tolerances cover the remaining importer/model
  // simplifications outside LCC tap control.
  const auto sol = parse_sol(dsp_path(sol_file));
  const auto [max_dv, max_da] = compare_vs_sol_by_name(res.system, pf, sol);
  REQUIRE(max_dv <= v_tol_pu);
  REQUIRE(max_da <= a_tol_deg);

  // ── LCC station results vs the DSP .pf report ───────────────────────────
  REQUIRE(pf.lcc_transfers.size() == 2);
  const hacdcpf::LCCTransfer* rect = nullptr;
  const hacdcpf::LCCTransfer* inv = nullptr;
  for (const auto& tr : pf.lcc_transfers) {
    if (tr.station_role == static_cast<int>(hacdcpf::LCCStationRole::Rectifier)) rect = &tr;
    if (tr.station_role == static_cast<int>(hacdcpf::LCCStationRole::Inverter)) inv = &tr;
  }
  REQUIRE(rect != nullptr);
  REQUIRE(inv != nullptr);

  const hacdcpf::LCCConverter* rect_model = nullptr;
  const hacdcpf::LCCConverter* inv_model = nullptr;
  for (const auto& model : res.system.lcc_converters) {
    if (model.station_role == hacdcpf::LCCStationRole::Rectifier) {
      rect_model = &model;
    }
    if (model.station_role == hacdcpf::LCCStationRole::Inverter) {
      inv_model = &model;
    }
  }
  REQUIRE(rect_model != nullptr);
  REQUIRE(inv_model != nullptr);
  REQUIRE(rect_model->tap_control_modelled);
  REQUIRE(inv_model->tap_control_modelled);

  // DC-side operating point: exact match with DSP (see header note).
  REQUIRE_THAT(rect->p_ac_mw, WithinAbs(-1500.0, 1.0));   // DSP: -1500 MW
  REQUIRE_THAT(inv->p_ac_mw, WithinAbs(1410.0, 1.0));     // DSP: +1410 MW
  REQUIRE_THAT(rect->id_ka, WithinAbs(3.0, 0.01));        // DSP: 3000 A
  REQUIRE_THAT(inv->id_ka, WithinAbs(3.0, 0.01));
  REQUIRE_THAT(rect->ud_kv, WithinAbs(500.0, 1.0));       // DSP: 500 kV
  REQUIRE_THAT(inv->ud_kv, WithinAbs(470.0, 1.0));        // DSP: 470 kV
  const double loss_mw = rect->p_dc_mw + inv->p_dc_mw;    // R*Id^2
  REQUIRE_THAT(loss_mw, WithinAbs(90.0, 1.0));            // DSP: 90 MW

  // R-card control moves the existing T branches until the LD normal angles
  // are recovered. Valve-side Q and tap remain calibrated bounded
  // approximations rather than bit-for-bit DSP reproductions.
  REQUIRE_THAT(rect->alpha_deg, WithinAbs(15.0, 0.1));
  REQUIRE_THAT(inv->gamma_deg, WithinAbs(17.0, 0.1));
  REQUIRE_THAT(rect->q_ac_mvar, WithinAbs(-526.54, 1.5));
  REQUIRE_THAT(inv->q_ac_mvar, WithinAbs(-530.90, 1.5));
  REQUIRE_THAT(rect->transformer_tap, WithinAbs(rect_tap_pu, 2e-3));
  REQUIRE_THAT(inv->transformer_tap, WithinAbs(inv_tap_pu, 2e-3));
  REQUIRE(rect->tap_control_active);
  REQUIRE(inv->tap_control_active);
  REQUIRE(rect->tap_control_converged);
  REQUIRE(inv->tap_control_converged);
  REQUIRE(rect->tap_control_iterations > 0);
  REQUIRE(inv->tap_control_iterations > 0);
  REQUIRE_FALSE(rect->tap_at_limit);
  REQUIRE_FALSE(inv->tap_at_limit);

  // Rated current remains the active current order at the calibrated point.
  REQUIRE(inv->id_at_limit);
  REQUIRE(rect->alpha_within_limits);
  REQUIRE(inv->gamma_within_limits);

  // Valve-side Q above excludes converter-transformer leakage. The authored
  // T branch must independently reproduce DSP's roughly 258 Mvar Q loss.
  const auto check_transformer_q_loss = [&](const hacdcpf::LCCConverter& model) {
    const auto branch = std::find_if(
        res.system.ac.branches.begin(), res.system.ac.branches.end(),
        [&](const auto& item) {
          return item.index == model.converter_transformer_branch;
        });
    REQUIRE(branch != res.system.ac.branches.end());
    const auto position =
        static_cast<size_t>(branch - res.system.ac.branches.begin());
    REQUIRE(position < pf.branch_flows.size());
    const auto& flow = pf.branch_flows[position];
    REQUIRE_THAT(flow.qf_mvar + flow.qt_mvar,
                 WithinAbs(transformer_q_loss_mvar, 2.0));
  };
  check_transformer_q_loss(*rect_model);
  check_transformer_q_loss(*inv_model);
}

}  // namespace

TEST_CASE("DSP compare: 2DC LCC link vs DSP solution", "[bpa][dsp][lcc]") {
  // The broad bus tolerance also covers non-LCC shunt and Q-control details.
  check_lcc_case("2DC.dat", "2DCNEW.SOL", 1e-2, 1.0,
                 549.25 / 500.0, 574.48 / 500.0, 258.15);
}

TEST_CASE("DSP compare: cigre LCC link vs DSP solution", "[bpa][dsp][lcc]") {
  // DSP: transformer taps 547.97/525 and 571.25/525, with about 258.11 Mvar
  // leakage consumption in each converter-transformer branch.
  check_lcc_case("cigre.dat", "cigreNEW.SOL", 5e-3, 1.0,
                 547.97 / 525.0, 571.25 / 525.0, 258.11);
}

TEST_CASE("DSP compare: vsc2 VSC-HVDC link vs DSP solution", "[bpa][dsp][vsc]") {
  // data/dsp/vsc2.dat is a synthetic two-terminal VSC-HVDC case (BZ/BZ+/LZ
  // cards, layout cross-checked against a production grid dat).  The DSP
  // reference (data/dsp/vsc2NEW.SOL, bus voltages; data/dsp/vsc2.pf,
  // ">>>柔性直流换流器" / ">>>直流线路潮流" sections) was produced by
  // DSP 2.1.47 Pwrflow (PQ -> NR, 6 iterations).
  //
  // DSP reference operating point (hard-coded from vsc2.pf):
  //   VSCA (BZ+ no flag -> mode 2, constant P/Q): P_ac = +1470.00 MW
  //     (inverter injecting; the card carries -1470 in the BPA load
  //     convention), Q_ac = -50.00 Mvar (50 absorbed), Udc = 299.9753 kV;
  //   VSCB (BZ+ flag 1 -> mode 1, constant Udc): Udc = 300.0000 kV exactly,
  //     P_ac = -1491.69 MW (rectifier), Q_ac = +50.00 Mvar (50 injected);
  //   station losses 10.363 / 10.442 MW (|P|*Rc, Rc = 0.007 pu);
  //   DC line (R = 0.01 ohm/pole, 2 poles): loss 0.12 MW, I = 4.935 kA.
  auto res = hacdcpf::io::parse_bpa_dat(dsp_path("vsc2.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  REQUIRE(res.system.vsc_converters.size() == 2);
  REQUIRE(res.system.lcc_converters.empty());

  const auto pf = hacdcpf::solve_power_flow(res.system);
  INFO("termination: " + pf.diagnostics.termination_reason);
  for (const auto& w : pf.diagnostics.warnings) INFO("warning: " + w);
  REQUIRE(pf.converged);

  // AC bus voltages vs the DSP SOL.  The AC-terminal injections are the
  // controlled quantities and match DSP almost exactly (see below), so the
  // AC network solution should too; the residual gap comes from the ~0.9 MW
  // (0.06%) transfer difference at VSCB.
  const auto sol = parse_sol(dsp_path("vsc2NEW.SOL"));
  const auto [max_dv, max_da] = compare_vs_sol_by_name(res.system, pf, sol);
  // Measured: max |dV| = 2.4e-4 pu (VSCB), max |d_delta| = 0.021 deg.
  REQUIRE(max_dv <= 1e-3);
  REQUIRE(max_da <= 0.1);

  // Converter operating point vs the DSP .pf report.  Station A holds its
  // card setpoints exactly; station B forms Udc (very stiff droop, see the
  // k_vdc note in make_vsc_station) and balances the link.
  REQUIRE(pf.vsc_transfers.size() == 2);
  const auto& ta = pf.vsc_transfers[0];  // VSCA, inverter (constant P/Q)
  const auto& tb = pf.vsc_transfers[1];  // VSCB, rectifier (constant Udc)
  REQUIRE_THAT(ta.p_ac_mw, WithinAbs(1470.0, 0.5));     // DSP: +1470.00
  REQUIRE_THAT(ta.q_ac_mvar, WithinAbs(-50.0, 0.5));    // DSP: -50.00
  REQUIRE_THAT(ta.p_dc_mw, WithinAbs(-1480.36, 0.5));   // DSP: 1480.36 drawn
  REQUIRE_THAT(ta.loss_mw, WithinAbs(10.363, 0.2));     // DSP: 10.363
  REQUIRE_THAT(tb.p_ac_mw, WithinAbs(-1491.69, 1.5));   // DSP: -1491.69
  REQUIRE_THAT(tb.q_ac_mvar, WithinAbs(50.0, 0.5));     // DSP: +50.00
  REQUIRE_THAT(tb.p_dc_mw, WithinAbs(1481.25, 1.5));    // DSP: 1481.25
  REQUIRE_THAT(tb.loss_mw, WithinAbs(10.442, 0.2));     // DSP: 10.442

  // DC voltage: DSP holds 300.0000 kV at B; HySim's stiff droop lands
  // within 0.03 kV.  Station A sits one I*R drop below (DSP 299.9753).
  const double udc_b = pf.vdc[static_cast<size_t>(
                                res.system.vsc_converters[1].bus_dc) - 1] *
                       300.0;
  const double udc_a = pf.vdc[static_cast<size_t>(
                                res.system.vsc_converters[0].bus_dc) - 1] *
                       300.0;
  REQUIRE_THAT(udc_b, WithinAbs(300.0, 0.05));
  REQUIRE_THAT(udc_a, WithinAbs(299.9753, 0.05));
  // DC current from the line transfer: 1480.4 MW at ~300 kV = 4.935 kA.
  REQUIRE_THAT(tb.p_dc_mw / udc_b, WithinAbs(4.935, 0.01));
}
