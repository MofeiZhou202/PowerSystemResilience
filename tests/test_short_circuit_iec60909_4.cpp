// IEC TR 60909-4:2021 §6.2 regression test — 33 kV/6 kV medium-voltage system
// with asynchronous motors (Figure 9 of the technical report).
//
// Reference values from §6.2 (complex calculation with absolute quantities),
// three-phase fault at the 6 kV busbar F (bus 4), tmin = 0.1 s:
//   I"k = 19.52 kA   ip = 48.82 kA   Ib = 17.04 kA   Ik = 14.74 kA
//   partial currents: I"k(T1,T2) = 14.74 kA, I"kM1 = 2.54 kA, I"kM2 = 2.24 kA
//
// The case data (tests/data/short_circuit_iec60909_4.json) rounds some inputs
// to four significant digits. Motor r_pu/x_pu are the published impedances
// converted from ohms to each motor's V_N^2/S_N nameplate base. A 0.5% band is
// therefore used instead of an exact match.
//
// Regression coverage for three fixes:
//   1. Transformer2W nameplate impedance is converted onto the LV bus voltage
//      base (33/6.3 kV transformer on a 6 kV bus → ×(6.3/6)² on the canonical
//      branch impedance); without it the fault impedance is ~10% too small.
//   2. Peak current uses IEC 60909-0 formula (59) per-contribution summation
//      (κ_net·I_net + Σκ_M·I_M) instead of a single meshed κ on the total.
//   3. Motor breaking-current factors use μ(I"kM/IrM) and q(PrM/p) with p =
//      pole pairs, so M1 (5 MW, p = 2) gets q = 0.68 and M2 (3×1 MW) q = 0.57.
//   4. Motor nameplate-base pu impedance is converted once to the system base;
//      treating the same numeric values as ohms or system-base pu is rejected.

#include <fstream>
#include <sstream>
#include <string>

#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/system.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT ".."
#endif

using namespace hacdcpf;
using namespace hacdcpf::analysis;

namespace {

HybridPowerSystem load_iec_case() {
  const std::string path = std::string(HACDCPF_PROJECT_ROOT) +
                           "/tests/data/short_circuit_iec60909_4.json";
  std::ifstream f(path);
  REQUIRE(f.good());
  std::stringstream ss;
  ss << f.rdbuf();
  return io::from_json(ss.str());
}

const SCDetailedBusResult& fault_row(const SCDetailedResult& res, int bus_id) {
  for (const auto& row : res.bus_results) {
    if (row.bus_id == bus_id) return row;
  }
  throw std::runtime_error("fault bus row not found");
}

}  // namespace

TEST_CASE("IEC 60909-4 §6.2: 33/6 kV system with asynchronous motors",
          "[short_circuit][iec60909]") {
  const auto sys = load_iec_case();

  SCDetailedOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.calc_type = SCCalcType::Max;
  opt.breaking_time_s = 0.1;  // tmin = 0.1 s as in §6.2

  const auto res = run_short_circuit_detailed(sys, 4, opt);
  REQUIRE(res.solved);
  const auto& row = fault_row(res, 4);

  // IEC TR 60909-4:2021 §6.2 reference values (±0.5% for rounded inputs).
  CHECK(row.ikss_ka == Catch::Approx(19.52).epsilon(0.005));
  CHECK(row.ip_ka == Catch::Approx(48.82).epsilon(0.005));
  CHECK(row.ib_ka == Catch::Approx(17.04).epsilon(0.005));
  CHECK(row.ik_ka == Catch::Approx(14.74).epsilon(0.005));

  // Partial currents: network (transformers) and asynchronous motors.
  CHECK(row.ikss_no_motor_ka == Catch::Approx(14.74).epsilon(0.005));
  CHECK(row.ikss_motor_contrib_ka == Catch::Approx(4.78).epsilon(0.005));
}
