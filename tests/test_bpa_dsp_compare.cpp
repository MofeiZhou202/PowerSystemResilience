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
/// FIXED-TAP / CURRENT-LIMIT NOTE (declared model limitation):
/// DSP holds alpha = AlphaN = 15 deg and gamma = GamaN = 17 deg by moving the
/// converter-transformer taps (2DC.pf shows the ratios adjusted to
/// 217/549.25 and 217/574.48, lowering the valve-side voltage ~10-15%).  Our
/// model keeps the tap at its nominal ratio, so U_d0 = (3*sqrt(2)/pi)*n_b*E
/// is ~10-15% HIGHER than DSP's tapped value.  The solved operating point is
/// then the physically consistent one for the declared controls:
///   * the rectifier holds the scheduled DC power exactly (P_ac = -1500 MW);
///   * the inverter's CEA characteristic at the nominal-tap U_d0 would demand
///     far more than the rated current, so the rated-current limit (current
///     order, Id = 3 kA — exactly the LD-card schedule Psch/Udr) binds, and
///     the DC voltage settles at the scheduled 500/470 kV: all DC-side
///     quantities (P, Id, Vdc, line loss) match DSP exactly;
///   * the back-calculated alpha / gamma depart from 15/17 deg (the degrees
///     of freedom a tap changer would absorb); gamma stays ABOVE gamma_min,
///     so commutation margin is preserved;
///   * the converter-station reactive draw differs from DSP (Q = P*tan(phi)
///     with cos(phi) = U_d/U_d0 and a higher U_d0), which shows up as a
///     modest voltage deviation at the converter buses (2DC; cigre's AC
///     system is stiff enough that the deviation stays ~2.6e-4 pu).
/// The tolerance choices below document the measured fixed-tap deviations
/// instead of hiding them.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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
                    double a_tol_deg) {
  auto res = hacdcpf::io::parse_bpa_dat(dsp_path(dat));  // default: LccQuasiSteady
  REQUIRE_FALSE(res.report.has_errors());
  REQUIRE(res.system.lcc_converters.size() == 2);
  REQUIRE(res.system.vsc_converters.empty());

  const auto pf = hacdcpf::solve_power_flow(res.system);
  INFO("termination: " + pf.diagnostics.termination_reason);
  for (const auto& w : pf.diagnostics.warnings) INFO("warning: " + w);
  REQUIRE(pf.converged);

  // AC bus voltages vs DSP SOL (valve-side stub buses are not listed by DSP
  // and are skipped).  See the header note for the fixed-tap deviation
  // mechanism behind the 2DC voltage tolerance.
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

  // DC-side operating point: exact match with DSP (see header note).
  REQUIRE_THAT(rect->p_ac_mw, WithinAbs(-1500.0, 1.0));   // DSP: -1500 MW
  REQUIRE_THAT(inv->p_ac_mw, WithinAbs(1410.0, 1.0));     // DSP: +1410 MW
  REQUIRE_THAT(rect->id_ka, WithinAbs(3.0, 0.01));        // DSP: 3000 A
  REQUIRE_THAT(inv->id_ka, WithinAbs(3.0, 0.01));
  REQUIRE_THAT(rect->ud_kv, WithinAbs(500.0, 1.0));       // DSP: 500 kV
  REQUIRE_THAT(inv->ud_kv, WithinAbs(470.0, 1.0));        // DSP: 470 kV
  const double loss_mw = rect->p_dc_mw + inv->p_dc_mw;    // R*Id^2
  REQUIRE_THAT(loss_mw, WithinAbs(90.0, 1.0));            // DSP: 90 MW

  // Both stations consume reactive power (Q = P*tan(phi), cos(phi)=U_d/U_d0).
  REQUIRE(rect->q_ac_mvar < 0.0);
  REQUIRE(inv->q_ac_mvar < 0.0);

  // The inverter runs at its rated current: the current limit binds (the
  // nominal-tap U_d0 would drive the CEA characteristic far above 3 kA), so
  // the CEA setpoint becomes a lower bound and the physical gamma floats.
  REQUIRE(inv->id_at_limit);
  // Back-calculated angles depart from AlphaN/GamaN (tap-absorbed degrees of
  // freedom): alpha = 28.5 deg (2DC) / 23.9 deg (cigre), gamma = 33.1 / 29.1
  // deg — always above the 17 deg CEA minimum, i.e. commutation-safe.
  REQUIRE(rect->alpha_deg > 15.0);
  REQUIRE(rect->alpha_deg < 45.0);
  REQUIRE(inv->gamma_deg >= 17.0 - 1e-6);
  REQUIRE(inv->gamma_deg < 45.0);
  // Firing/extinction angles remain inside the physical windows.
  REQUIRE(rect->alpha_within_limits);
  REQUIRE(inv->gamma_within_limits);
}

}  // namespace

TEST_CASE("DSP compare: 2DC LCC link vs DSP solution", "[bpa][dsp][lcc]") {
  // Measured: max |dV| = 7.9e-3 pu (converter buses SOUTH/STATIONS/NORTH, the
  // fixed-tap reactive-draw gap — see header), max |d_delta| = 0.15 deg.
  check_lcc_case("2DC.dat", "2DCNEW.SOL", 1e-2, 1.0);
}

TEST_CASE("DSP compare: cigre LCC link vs DSP solution", "[bpa][dsp][lcc]") {
  // Measured: max |dV| = 2.6e-4 pu, max |d_delta| < 1e-3 deg (stiff 525 kV
  // system; the fixed-tap Q gap barely moves the bus voltages).
  check_lcc_case("cigre.dat", "cigreNEW.SOL", 5e-3, 1.0);
}
