// Short Circuit Analysis — Cross-Validation Tests
//
// Validates the Z-bus implementation against analytical Thévenin impedances on
// small radial networks, then runs self-consistency checks on IEEE 14-bus and
// IEEE 33-bus BW.
//
// Tests:
//   1. 2-bus: infinite bus + series impedance → Z_Thev = Z_line  (analytical)
//   2. 3-bus radial: Z_Thev(bus2)=Z12, Z_Thev(bus3)=Z12+Z23     (analytical)
//   3. IEEE 14-bus: positivity, passivity, single-bus API consistency
//   4. IEEE 33-bus BW: monotonic Sk, single-bus API round-trip
//   5. Fault-type ratios: SLG ≥ 3PH for grounded source
//   6. IEC 60909 c-factor: larger c_factor ↑ Sk

#include <cmath>
#include <complex>

#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/model/system.hpp"
#include <catch2/catch_test_macros.hpp>

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Cx = std::complex<double>;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static HybridPowerSystem make_sys(std::vector<ACBus>    buses,
                                  std::vector<ACBranch> branches,
                                  double base_mva = 100.0,
                                  double xdpp     = 0.20) {
  HybridPowerSystem sys;
  sys.base_mva  = base_mva;
  sys.ac.base_mva = base_mva;
  sys.ac.buses    = std::move(buses);
  sys.ac.branches = std::move(branches);

  int idx = 1;
  for (const auto& b : sys.ac.buses) {
    if (b.bus_type == BusType::SLACK || b.bus_type == BusType::PV) {
      Generator g;
      g.index = idx++; g.bus = b.index; g.in_service = true;
      g.pg_mw = 0.0; g.vg_pu = 1.0;
      g.pmax_mw = 9999.0; g.pmin_mw = 0.0;
      g.qmax_mvar = 9999.0; g.qmin_mvar = -9999.0;
      g.xdpp_pu = xdpp;
      sys.ac.generators.push_back(g);
    }
  }
  return sys;
}

static ACBus make_bus(int id, BusType t, double kv = 100.0) {
  ACBus b;
  b.index = id; b.bus_type = t;
  b.vm_pu = 1.0; b.va_deg = 0.0;
  b.vmin_pu = 0.9; b.vmax_pu = 1.1;
  b.pd_mw = 0.0; b.qd_mvar = 0.0;
  b.base_kv = kv; b.in_service = true;
  return b;
}

static ACBranch make_branch(int id, int f, int t, double r, double x) {
  ACBranch br;
  br.index = id; br.from_bus = f; br.to_bus = t;
  br.r_pu = r; br.x_pu = x; br.b_pu = 0.0;
  br.tap = 1.0; br.shift_deg = 0.0; br.in_service = true;
  br.rate_a_mva = 9999.0;
  return br;
}

// ---------------------------------------------------------------------------
// Test 1 — 2-bus: analytical Thévenin
// ---------------------------------------------------------------------------
TEST_CASE("SC: 2-bus analytical Thévenin", "[short_circuit][analytical]") {
  // Infinite bus (xdpp → 0) + single branch Z = 0.20+0.40j pu
  // Expected Z_Thev(bus2) ≈ Z_line
  const Cx Z_line(0.20, 0.40);
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)},
      {make_branch(1, 1, 2, Z_line.real(), Z_line.imag())},
      100.0,
      0.001  // near-infinite source
  );

  SCOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor   = 1.0;
  auto res = compute_short_circuit(sys, opt);

  REQUIRE(res.bus_results.size() == 2);

  Cx Z_computed = res.bus_results[1].z_thevenin;
  INFO("Z_Thev(bus2) computed = " << Z_computed.real() << "+" << Z_computed.imag() << "j pu");
  INFO("Z_Thev(bus2) analytic = " << Z_line.real() << "+" << Z_line.imag() << "j pu");

  CHECK(std::abs(Z_computed - Z_line) < 0.01);               // < 1% tolerance
  CHECK(std::abs(res.bus_results[1].i_fault_pu
                 - Cx(1.0,0.0)/Z_line) < 0.05);              // fault current
  CHECK(res.bus_results[1].sk_mva > 0.0);
  CHECK(std::isfinite(res.bus_results[1].sk_mva));
}

// ---------------------------------------------------------------------------
// Test 2 — 3-bus radial: exact series-impedance rule
// ---------------------------------------------------------------------------
TEST_CASE("SC: 3-bus radial analytical Thévenin", "[short_circuit][analytical]") {
  //   Bus 1 (slack, xdpp=0.001)
  //   Branch 1-2: Z12 = 0.20+0.40j
  //   Branch 2-3: Z23 = 0.30+0.60j
  //
  // Expected: Z_kk(bus2) = Z12,  Z_kk(bus3) = Z12+Z23
  // (Z-bus derivation: with bus 1 at ground, Y = [y12+y23,-y23;-y23,y23]
  //  → Z-bus = [Z12, Z12; Z12, Z12+Z23] )
  const Cx Z12(0.20, 0.40);
  const Cx Z23(0.30, 0.60);

  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ), make_bus(3, BusType::PQ)},
      {make_branch(1, 1, 2, Z12.real(), Z12.imag()),
       make_branch(2, 2, 3, Z23.real(), Z23.imag())},
      100.0,
      0.001
  );

  SCOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor   = 1.0;
  auto res = compute_short_circuit(sys, opt);

  REQUIRE(res.bus_results.size() == 3);

  const Cx Z2 = res.bus_results[1].z_thevenin;
  const Cx Z3 = res.bus_results[2].z_thevenin;

  INFO("Z_kk(bus2): " << Z2.real() << "+" << Z2.imag() << "j  (analytic: " << Z12.real() << "+" << Z12.imag() << "j)");
  INFO("Z_kk(bus3): " << Z3.real() << "+" << Z3.imag() << "j  (analytic: " << (Z12+Z23).real() << "+" << (Z12+Z23).imag() << "j)");

  CHECK(std::abs(Z2 - Z12)        < 0.01);
  CHECK(std::abs(Z3 - (Z12+Z23)) < 0.01);

  // Monotonicity: further from source → lower Sk
  double sk1 = res.bus_results[0].sk_mva;
  double sk2 = res.bus_results[1].sk_mva;
  double sk3 = res.bus_results[2].sk_mva;
  INFO("Sk: bus1=" << sk1 << "  bus2=" << sk2 << "  bus3=" << sk3 << " MVA");
  CHECK(sk1 >= sk2);
  CHECK(sk2 >= sk3);
  for (const auto& r : res.bus_results) {
    CHECK(r.sk_mva > 0.0);
    CHECK(std::isfinite(r.sk_mva));
  }
}

// ---------------------------------------------------------------------------
// Test 3 — IEEE 14-bus: self-consistency
// ---------------------------------------------------------------------------
TEST_CASE("SC: IEEE 14-bus self-consistency", "[short_circuit][ieee14]") {
  auto sys = io::build_ieee14_acdc();
  const int n = static_cast<int>(sys.ac.buses.size());

  SCOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor   = 1.1;
  auto res = compute_short_circuit(sys, opt);

  REQUIRE(static_cast<int>(res.bus_results.size()) == n);

  // All Sk positive and finite
  for (const auto& r : res.bus_results) {
    INFO("bus " << r.bus_id << ": Sk=" << r.sk_mva << " MVA  Z_kk=(" << r.z_thevenin.real() << "+" << r.z_thevenin.imag() << "j)");
    CHECK(r.sk_mva > 0.0);
    CHECK(std::isfinite(r.sk_mva));
    CHECK(r.ikpp_ka > 0.0);
    CHECK(std::isfinite(r.ikpp_ka));
    // Passive network: real part of Z_kk ≥ 0
    CHECK(r.z_thevenin.real() >= 0.0);
  }

  // Bus 1 (slack / strong source) should contribute a large Sk
  double sk_max = 0.0;
  for (const auto& r : res.bus_results) sk_max = std::max(sk_max, r.sk_mva);
  double sk_bus1 = res.bus_results[0].sk_mva;
  INFO("Sk_max=" << sk_max << "  Sk(bus1)=" << sk_bus1);
  CHECK(sk_bus1 >= 0.5 * sk_max);

  // Summary string is non-empty
  CHECK(!res.summary().empty());
}

// ---------------------------------------------------------------------------
// Test 4 — single-bus API round-trip (IEEE 14-bus bus 9)
// ---------------------------------------------------------------------------
TEST_CASE("SC: compute_fault_at_bus matches full solve", "[short_circuit][api]") {
  const Cx Z12(0.20, 0.40);
  const Cx Z23(0.30, 0.60);
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ), make_bus(3, BusType::PQ)},
      {make_branch(1, 1, 2, Z12.real(), Z12.imag()),
       make_branch(2, 2, 3, Z23.real(), Z23.imag())},
      100.0, 0.001);

  SCOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor   = 1.0;

  auto full   = compute_short_circuit(sys, opt);
  auto single = compute_fault_at_bus(sys, 3, opt);

  REQUIRE(single.bus_id == 3);
  // Results must match exactly (same code path)
  CHECK(std::abs(single.z_thevenin  - full.bus_results[2].z_thevenin)  < 1e-12);
  CHECK(std::abs(single.i_fault_pu  - full.bus_results[2].i_fault_pu)  < 1e-12);
  CHECK(std::abs(single.sk_mva      - full.bus_results[2].sk_mva)      < 1e-6);
  CHECK(std::abs(single.ikpp_ka     - full.bus_results[2].ikpp_ka)     < 1e-9);
}

// ---------------------------------------------------------------------------
// Test 5 — Fault-type ratios (SLG vs 3PH on 2-bus grounded source)
// ---------------------------------------------------------------------------
TEST_CASE("SC: SLG >= 3PH for strongly grounded system", "[short_circuit][fault_types]") {
  // For a solidly grounded system Z0 = Z1, SLG fault current:
  //   I_SLG = 3c / (Z1+Z2+Z0) = 3c / (3*Z1) = c / Z1 = I_3PH
  // When Z0 < Z1 (as in well-grounded networks), I_SLG > I_3PH.
  // With the simplified model (Z0=Z1 approximation), SLG ≈ 3PH.
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)},
      {make_branch(1, 1, 2, 0.10, 0.30)},
      100.0, 0.001);

  SCOptions opt3;
  opt3.fault_type = FaultType::ThreePhase;
  opt3.c_factor   = 1.0;

  SCOptions optSLG;
  optSLG.fault_type = FaultType::SinglePhaseGround;
  optSLG.c_factor   = 1.0;

  auto res3   = compute_short_circuit(sys, opt3);
  auto resSLG = compute_short_circuit(sys, optSLG);

  double I3   = std::abs(res3.bus_results[1].i_fault_pu);
  double ISLG = std::abs(resSLG.bus_results[1].i_fault_pu);

  INFO("I_3PH=" << I3 << " pu  I_SLG=" << ISLG << " pu");
  // With Z0≈Z1 approximation: SLG ≈ 3PH (within 1%)
  CHECK(ISLG >= I3 * 0.99);
  // Both are positive and finite
  CHECK(I3 > 0.0);
  CHECK(std::isfinite(I3));
  CHECK(ISLG > 0.0);
  CHECK(std::isfinite(ISLG));
}

// ---------------------------------------------------------------------------
// Test 6 — IEC 60909 c-factor: larger c → higher Sk
// ---------------------------------------------------------------------------
TEST_CASE("SC: IEC c-factor scales fault levels", "[short_circuit][iec60909]") {
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)},
      {make_branch(1, 1, 2, 0.05, 0.20)},
      100.0, 0.05);

  SCOptions opt_low, opt_high;
  opt_low.fault_type  = FaultType::ThreePhase;
  opt_low.c_factor    = 1.0;
  opt_high.fault_type = FaultType::ThreePhase;
  opt_high.c_factor   = 1.1;

  auto res_low  = compute_short_circuit(sys, opt_low);
  auto res_high = compute_short_circuit(sys, opt_high);

  double sk_low  = res_low.bus_results[1].sk_mva;
  double sk_high = res_high.bus_results[1].sk_mva;

  INFO("Sk(c=1.0)=" << sk_low << " MVA  Sk(c=1.1)=" << sk_high << " MVA");
  // Sk ∝ c² → high/low ratio ≈ 1.21
  CHECK(sk_high > sk_low);
  CHECK(sk_high / sk_low > 1.15);
  CHECK(sk_high / sk_low < 1.30);
}

// ---------------------------------------------------------------------------
// Test 7 — IEEE 33-bus BW: monotonic Sk along main feeder
// ---------------------------------------------------------------------------
TEST_CASE("SC: IEEE 33-bus BW monotonic fault levels", "[short_circuit][ieee33bw]") {
  auto sys_full = io::build_case33bw_acdc();
  auto sys = io::build_ac_only_version(sys_full);
  const int n = static_cast<int>(sys.ac.buses.size());

  SCOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor   = 1.1;
  auto res = compute_short_circuit(sys, opt);

  REQUIRE(static_cast<int>(res.bus_results.size()) == n);

  // All Sk positive and finite
  for (const auto& r : res.bus_results) {
    CHECK(r.sk_mva > 0.0);
    CHECK(std::isfinite(r.sk_mva));
    CHECK(r.ikpp_ka > 0.0);
    CHECK(r.z_thevenin.real() >= 0.0);
  }

  // Bus 1 (substation, strongest source) has maximum Sk
  double sk_max = 0.0;
  for (const auto& r : res.bus_results) sk_max = std::max(sk_max, r.sk_mva);
  double sk_bus1 = res.bus_results[0].sk_mva;
  INFO("Sk_max=" << sk_max << "  Sk(bus1)=" << sk_bus1 << "  n=" << n);
  CHECK(sk_bus1 == sk_max);   // bus 1 is the substation source (infinite bus)

  // Sk at end-of-feeder (bus 18, last bus on main feeder) << Sk at bus 1
  // Bus index 17 (0-based) = bus_id 18
  double sk_end = res.bus_results[17].sk_mva;
  INFO("Sk(bus18)=" << sk_end << " MVA  (end of main feeder)");
  CHECK(sk_end < sk_bus1 * 0.5);   // at least halved at end of feeder

  // single-bus API round-trip for bus 18
  auto single = compute_fault_at_bus(sys, 18, opt);
  CHECK(single.bus_id == 18);
  CHECK(std::abs(single.sk_mva - res.bus_results[17].sk_mva) < 1e-4);
}
