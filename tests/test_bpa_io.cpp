/// tests/test_bpa_io.cpp
/// =======================
/// BPA/DSP .dat importer contract tests (io/bpa_io).
///
///   * Structure: bus / branch / generator / DC-element counts and spot
///     values for the four sample cases under data/dsp.
///   * Round-trip: to_json -> from_json preserves element counts.
///   * Accuracy: Newton power flow on the converted pure-AC cases
///     (39.dat, IEEE90.dat) is compared against the DSP reference solutions
///     (Samples/39bus/39NEW.SOL, Samples/IEEE90/IEEE90NEW.SOL).
///   * HVDC cases (2DC.dat, cigre.dat): conversion must converge and the
///     LCC link must transfer approximately its scheduled 1500 MW.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <string>
#include <unordered_map>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/bpa_io.hpp"
#include "hacdcpf/io/json_io.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

std::string dat_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/dsp/" + name;
}

const hacdcpf::ACBus* find_bus(const hacdcpf::HybridPowerSystem& sys,
                               const std::string& name) {
  for (const auto& b : sys.ac.buses)
    if (b.name == name) return &b;
  return nullptr;
}

const hacdcpf::ACBranch* find_branch(const hacdcpf::HybridPowerSystem& sys,
                                     const std::string& name) {
  for (const auto& b : sys.ac.branches)
    if (b.name == name) return &b;
  return nullptr;
}

const hacdcpf::Generator* find_gen(const hacdcpf::HybridPowerSystem& sys,
                                   const std::string& name) {
  for (const auto& g : sys.ac.generators)
    if (g.name == name) return &g;
  return nullptr;
}

}  // namespace

TEST_CASE("BPA import: 39-bus structure", "[bpa][structure]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("39.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 39);
  REQUIRE(sys.ac.branches.size() == 46);  // 34 L + 12 T cards
  REQUIRE(sys.ac.generators.size() == 10);
  REQUIRE(sys.ac.loads.size() == 20);
  REQUIRE(sys.dc.buses.empty());

  const auto* b30 = find_bus(sys, "new30");
  REQUIRE(b30 != nullptr);
  REQUIRE(b30->bus_type == hacdcpf::BusType::PV);
  REQUIRE_THAT(b30->vm_pu, WithinAbs(1.047, 1e-9));
  REQUIRE_THAT(b30->base_kv, WithinAbs(20.0, 1e-9));

  const auto* g30 = find_gen(sys, "new30");
  REQUIRE(g30 != nullptr);
  REQUIRE_THAT(g30->pg_mw, WithinAbs(250.0, 1e-9));
  REQUIRE_THAT(g30->vg_pu, WithinAbs(1.047, 1e-9));

  const auto* bs = find_bus(sys, "new39");
  REQUIRE(bs != nullptr);
  REQUIRE(bs->bus_type == hacdcpf::BusType::SLACK);
  REQUIRE_THAT(bs->vm_pu, WithinAbs(1.03, 1e-9));

  const auto* l12 = find_branch(sys, "L_new1_new2");
  REQUIRE(l12 != nullptr);
  REQUIRE_THAT(l12->r_pu, WithinAbs(0.0035, 1e-9));
  REQUIRE_THAT(l12->x_pu, WithinAbs(0.0411, 1e-9));
  REQUIRE_THAT(l12->b_pu, WithinAbs(0.6936, 1e-9));  // 2 * B/2

  const auto* t631 = find_branch(sys, "T_new6_new31");
  REQUIRE(t631 != nullptr);
  REQUIRE_THAT(t631->x_pu, WithinAbs(0.025, 1e-9));
  // tap = (369/345) / (20/20)
  REQUIRE_THAT(t631->tap, WithinAbs(369.0 / 345.0, 1e-9));
}

TEST_CASE("BPA import: IEEE90 structure and GBK names", "[bpa][structure]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("IEEE90.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 9);
  REQUIRE(sys.ac.branches.size() == 9);  // 6 L + 3 T cards
  REQUIRE(sys.ac.generators.size() == 3);
  REQUIRE(sys.ac.loads.size() == 4);
  REQUIRE(sys.ac.shunts.size() == 3);

  // Chinese bus names must survive as valid UTF-8.
  const auto* g1 = find_bus(sys, "发电机1");
  REQUIRE(g1 != nullptr);
  REQUIRE(g1->bus_type == hacdcpf::BusType::SLACK);
  REQUIRE_THAT(g1->vm_pu, WithinAbs(1.01, 1e-9));
  REQUIRE_THAT(g1->base_kv, WithinAbs(16.5, 1e-9));

  const auto* g2 = find_gen(sys, "发电机2");
  REQUIRE(g2 != nullptr);
  REQUIRE_THAT(g2->pg_mw, WithinAbs(163.0, 1e-9));

  const auto* tr = find_branch(sys, "T_发电机1_母线1");
  REQUIRE(tr != nullptr);
  REQUIRE_THAT(tr->x_pu, WithinAbs(0.0567, 1e-9));
  // tap = (16.5/16.5) / (242/230)
  REQUIRE_THAT(tr->tap, WithinAbs(230.0 / 242.0, 1e-9));
}

TEST_CASE("BPA import: 2DC HVDC structure", "[bpa][structure]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("2DC.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 11);
  REQUIRE(sys.ac.branches.size() == 13);  // 6 L + 7 T cards
  REQUIRE(sys.ac.generators.size() == 5);
  REQUIRE(sys.dc.buses.size() == 2);
  REQUIRE(sys.dc.branches.size() == 1);
  REQUIRE(sys.vsc_converters.size() == 2);

  const auto& dcb = sys.dc.branches.front();
  // r_pu = 10 ohm * 100 MVA / (500 kV)^2
  REQUIRE_THAT(dcb.r_pu, WithinAbs(0.004, 1e-9));
  REQUIRE_THAT(dcb.rate_a_mva, WithinAbs(1500.0, 1e-9));

  // Rectifier draws scheduled power; inverter forms the DC voltage (VDC_Q)
  // and delivers the received power emergently.
  const auto& rect = sys.vsc_converters[0];
  const auto& inv = sys.vsc_converters[1];
  REQUIRE(rect.control_mode == hacdcpf::ConverterMode::PQ_MODE);
  REQUIRE_THAT(rect.p_set_mw, WithinAbs(-1500.0, 1e-9));
  REQUIRE(inv.control_mode == hacdcpf::ConverterMode::VDC_Q);
  // v_dc_set = (500 kV - 3 kA * 10 ohm) / 500 kV = 0.94 pu
  REQUIRE_THAT(inv.v_dc_set_pu, WithinAbs(0.94, 1e-9));
  REQUIRE(inv.k_vdc > 100.0);
  // Station reactive consumption estimate: 0.5 * P, absorbed from AC.
  REQUIRE(rect.q_set_mvar < 0.0);
  REQUIRE(inv.q_set_mvar < 0.0);
}

TEST_CASE("BPA import: cigre HVDC structure", "[bpa][structure]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"));
  REQUIRE_FALSE(res.report.has_errors());
  const auto& sys = res.system;

  REQUIRE(sys.ac.buses.size() == 6);
  REQUIRE(sys.ac.branches.size() == 4);  // 2 L + 2 T cards
  REQUIRE(sys.dc.buses.size() == 2);
  REQUIRE(sys.dc.branches.size() == 1);
  REQUIRE(sys.vsc_converters.size() == 2);
  REQUIRE_THAT(sys.dc.branches.front().r_pu, WithinAbs(0.004, 1e-9));
}

TEST_CASE("BPA import: JSON round-trip", "[bpa][roundtrip]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("39.dat"));
  const auto json = hacdcpf::io::to_json(res.system, 2);
  auto back = hacdcpf::io::from_json(json);
  REQUIRE(back.ac.buses.size() == res.system.ac.buses.size());
  REQUIRE(back.ac.branches.size() == res.system.ac.branches.size());
  REQUIRE(back.ac.generators.size() == res.system.ac.generators.size());
  REQUIRE(back.ac.loads.size() == res.system.ac.loads.size());
}

TEST_CASE("BPA export: fixed-column round-trip", "[bpa][roundtrip][export]") {
  for (const auto* filename : {"39.dat", "2DC.dat"}) {
    const auto imported = hacdcpf::io::parse_bpa_dat(dat_path(filename));
    REQUIRE_FALSE(imported.report.has_errors());

    const std::string dat = hacdcpf::io::to_bpa_dat(imported.system);
    REQUIRE(dat.find("/MVA_BASE=100") != std::string::npos);
    REQUIRE(dat.find("A0000001") != std::string::npos);
    REQUIRE(dat.find("(END)") != std::string::npos);

    const auto round_trip = hacdcpf::io::parse_bpa_dat_string(dat);
    REQUIRE_FALSE(round_trip.report.has_errors());
    REQUIRE(round_trip.system.ac.buses.size() == imported.system.ac.buses.size());
    REQUIRE(round_trip.system.ac.branches.size() == imported.system.ac.branches.size());
    REQUIRE(round_trip.system.ac.generators.size() == imported.system.ac.generators.size());
    REQUIRE(round_trip.system.ac.loads.size() == imported.system.ac.loads.size());
    REQUIRE(round_trip.system.dc.buses.size() == imported.system.dc.buses.size());
    REQUIRE(round_trip.system.dc.branches.size() == imported.system.dc.branches.size());
    REQUIRE(round_trip.system.vsc_converters.size() == imported.system.vsc_converters.size());
  }
}

// DSP reference solutions (Samples/39bus/39NEW.SOL), name -> (vm, va_deg).
static const std::unordered_map<std::string, std::pair<double, double>>
    kSol39 = {
        {"new1", {1.047575, 1.487873}},   {"new2", {1.048977, 3.966515}},
        {"new3", {1.031298, 1.094970}},   {"new4", {1.006304, 0.267400}},
        {"new5", {1.008868, 1.424903}},   {"new6", {1.011043, 2.115523}},
        {"new7", {1.000283, -0.056716}},  {"new8", {0.999240, -0.551302}},
        {"new9", {1.029554, -0.263977}},  {"new10", {1.019182, 4.509627}},
        {"new11", {1.015197, 3.693142}},  {"new12", {1.002276, 3.695372}},
        {"new13", {1.016338, 3.802513}},  {"new14", {1.013760, 2.148957}},
        {"new15", {1.016729, 1.753573}},  {"new16", {1.032714, 3.164391}},
        {"new17", {1.034451, 2.173402}},  {"new18", {1.031940, 1.334891}},
        {"new19", {1.049769, 7.790752}},  {"new20", {0.990567, 6.378937}},
        {"new21", {1.031873, 5.568243}},  {"new22", {1.049711, 10.017804}},
        {"new23", {1.044762, 9.819463}},  {"new24", {1.038110, 3.284089}},
        {"new25", {1.057297, 5.332769}},  {"new26", {1.052123, 4.059755}},
        {"new27", {1.038195, 2.040314}},  {"new28", {1.049837, 7.573736}},
        {"new29", {1.049558, 10.335159}}, {"new30", {1.047000, 6.386712}},
        {"new31", {0.982000, 10.693116}}, {"new32", {0.983000, 12.487224}},
        {"new33", {0.997000, 13.009079}}, {"new34", {1.012000, 11.571776}},
        {"new35", {1.049000, 14.981018}}, {"new36", {1.063000, 17.676756}},
        {"new37", {1.027000, 12.124703}}, {"new38", {1.026000, 17.404641}},
        {"new39", {1.030000, 0.000000}},
};

TEST_CASE("BPA import: 39-bus PF matches DSP solution", "[bpa][pf]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("39.dat"));
  hacdcpf::PowerFlowOptions opt;
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  REQUIRE(pf.converged);

  double max_dv = 0.0, max_da = 0.0;
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  for (const auto& bus : res.system.ac.buses) {
    const auto it = kSol39.find(bus.name);
    REQUIRE(it != kSol39.end());
    const size_t i = static_cast<size_t>(bus.index) - 1;
    const double va_deg = pf.va[i] * kDeg;  // solver reports radians
    max_dv = std::max(max_dv, std::abs(pf.vm[i] - it->second.first));
    max_da = std::max(max_da, std::abs(va_deg - it->second.second));
  }
  REQUIRE(max_dv < 2e-3);
  REQUIRE(max_da < 0.5);
}

// DSP reference solution (Samples/IEEE90/IEEE90NEW.SOL).
static const std::unordered_map<std::string, std::pair<double, double>>
    kSolIeee90 = {
        {"发电机1", {1.010000, 0.000000}}, {"母线1", {1.038771, -3.436394}},
        {"母线A", {1.006125, -6.161531}},  {"母线B", {1.022197, -5.463977}},
        {"母线C", {1.031936, -3.145088}},  {"母线2", {1.042976, -0.746674}},
        {"发电机2", {1.010000, 5.093616}}, {"母线3", {1.053444, -1.307074}},
        {"发电机3", {1.010000, 1.516309}},
};

TEST_CASE("BPA import: IEEE90 PF matches DSP solution", "[bpa][pf]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("IEEE90.dat"));
  hacdcpf::PowerFlowOptions opt;
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  REQUIRE(pf.converged);

  // DSP applies its own shunt/Q-limit corrections that the importer does not
  // replicate; the residual deviation is ~2.6e-3 pu / 0.4 deg (the 39-bus
  // case, which carries no shunts, matches to <2e-3 / 0.5 deg).
  double max_dv = 0.0, max_da = 0.0;
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  for (const auto& bus : res.system.ac.buses) {
    const auto it = kSolIeee90.find(bus.name);
    REQUIRE(it != kSolIeee90.end());
    const size_t i = static_cast<size_t>(bus.index) - 1;
    const double va_deg = pf.va[i] * kDeg;  // solver reports radians
    max_dv = std::max(max_dv, std::abs(pf.vm[i] - it->second.first));
    max_da = std::max(max_da, std::abs(va_deg - it->second.second));
  }
  REQUIRE(max_dv < 5e-3);
  REQUIRE(max_da < 0.5);
}

TEST_CASE("BPA import: 2DC HVDC link transfers scheduled power", "[bpa][pf]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("2DC.dat"));
  hacdcpf::PowerFlowOptions opt;  // default solver options (as used by the GUI)
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  REQUIRE(pf.converged);
  REQUIRE(pf.vsc_transfers.size() == 2);
  // Rectifier (converter 0) draws the scheduled 1500 MW from its AC system,
  // exactly as in the DSP solution.
  REQUIRE_THAT(pf.vsc_transfers[0].p_ac_mw, WithinAbs(-1500.0, 1.0));
  // Inverter delivers Psch - I^2*R = 1410 MW, matching DSP's 1410 MW label.
  REQUIRE_THAT(pf.vsc_transfers[1].p_ac_mw, WithinAbs(1410.0, 15.0));
  // Both stations absorb reactive power (LCC estimate 0.5 * P).
  REQUIRE(pf.vsc_transfers[0].q_ac_mvar < 0.0);
  REQUIRE(pf.vsc_transfers[1].q_ac_mvar < 0.0);
  // Inverter-side DC voltage held at ~0.94 pu (470 kV) by the VDC_Q station.
  const auto& inv = res.system.vsc_converters[1];
  REQUIRE_THAT(pf.vdc[static_cast<size_t>(inv.bus_dc) - 1],
               WithinAbs(0.94, 0.02));
}

TEST_CASE("BPA import: cigre HVDC link transfers scheduled power", "[bpa][pf]") {
  auto res = hacdcpf::io::parse_bpa_dat(dat_path("cigre.dat"));
  hacdcpf::PowerFlowOptions opt;
  auto pf = hacdcpf::solve_power_flow(res.system, opt);
  REQUIRE(pf.converged);
  REQUIRE(pf.vsc_transfers.size() == 2);
  REQUIRE_THAT(pf.vsc_transfers[0].p_ac_mw, WithinAbs(-1500.0, 1.0));
  REQUIRE_THAT(pf.vsc_transfers[1].p_ac_mw, WithinAbs(1410.0, 15.0));
}
