// Harmonic Power Flow (hybrid AC/DC) — Numerical Verification Tests
// ==================================================================
// Validates hacdcpf::harmonics::solve_harmonic_power_flow against closed-form
// analytical solutions on small networks, then runs self-consistency checks.
//
// The frequency-domain model is linear at each order, so every harmonic bus
// voltage equals  I_inj * Z(bus->ground).  For a radial path this is simply the
// sum of the series branch impedances plus the source internal impedance, which
// makes the expected voltages exactly computable.
//
//   z_src,AC(h) = r + j*h*x''           (default x'' = 0.2 pu on a slack node)
//   z_line(h)   = r + j*h*x
//   z_src,DC    = dc_source_impedance_pu (default 0.01 pu, resistive)
//   z_branch,DC = r                      (DC branches carry no inductance)
//
// Tests:
//   1. AC 2-bus: V2(h) = I*(z_src+z_line), V1(h) = I*z_src, THD = |Vh|/|V1|
//   2. AC impedance frequency scaling (lossless): |V(7)|/|V(5)| = 7/5
//   3. AC linearity / superposition: 2*I -> 2*V
//   4. AC radial feeder: monotone harmonic voltage, exact per-bus values
//   5. DC 2-bus ripple: V2(r) = I*(z_src,DC + r_branch)
//   6. NIC hybrid coupling: I_ac1 = conj(S)/conj(V), AC & DC ripple, P-scaling
//   7. Multi-harmonic THD formula
//   8. Default spectra, summary(), branch-flow output sanity

#include <cmath>
#include <complex>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/analysis/harmonics_power_flow.hpp"

using namespace hacdcpf;
using namespace hacdcpf::harmonics;
using Cx = std::complex<double>;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ---------------------------------------------------------------------------
// Builders
// ---------------------------------------------------------------------------
static ACBus ac_bus(int id, BusType t, double kv = 10.0) {
  ACBus b;
  b.index = id;
  b.bus_type = t;
  b.vm_pu = 1.0;
  b.va_deg = 0.0;
  b.base_kv = kv;
  b.in_service = true;
  return b;
}

static ACBranch ac_line(int id, int f, int t, double r, double x, double b = 0.0) {
  ACBranch br;
  br.index = id;
  br.from_bus = f;
  br.to_bus = t;
  br.r_pu = r;
  br.x_pu = x;
  br.b_pu = b;
  br.tap = 1.0;
  br.in_service = true;
  return br;
}

static DCBus dc_bus(int id, DCBusType t, double kv = 1.0) {
  DCBus b;
  b.index = id;
  b.bus_type = t;
  b.vm_pu = 1.0;
  b.base_kv = kv;
  b.in_service = true;
  return b;
}

static DCBranch dc_line(int id, int f, int t, double r) {
  DCBranch br;
  br.index = id;
  br.from_bus = f;
  br.to_bus = t;
  br.r_pu = r;
  br.in_service = true;
  return br;
}

// Look up the harmonic voltage phasor at a given bus and order.
static Cx vbus(const HPFResult& r, int bus, int order, bool is_dc = false) {
  const auto& vec = is_dc ? r.dc_bus_results : r.ac_bus_results;
  for (const auto& br : vec) {
    if (br.bus == bus) {
      auto it = br.v_by_order.find(order);
      if (it != br.v_by_order.end()) return it->second;
    }
  }
  return Cx(0.0, 0.0);
}

static double thd_at(const HPFResult& r, int bus, bool is_dc = false) {
  const auto& vec = is_dc ? r.dc_bus_results : r.ac_bus_results;
  for (const auto& br : vec)
    if (br.bus == bus) return br.thd_pct;
  return -1.0;
}

// ===========================================================================
// 1. AC 2-bus analytical
// ===========================================================================
TEST_CASE("HPF AC 2-bus analytical voltage and THD", "[harmonics][ac]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.01, 0.1)};

  HarmonicCurrentSource src;
  src.bus = 2;
  src.is_dc = false;
  src.i_base_pu = 1.0;          // 1.0 pu reference current
  src.spectrum = {{5, 100.0, 0.0}};  // inject 1.0 pu at the 5th

  HarmonicStudyInputs in;
  in.sources = {src};

  HPFOptions opt;
  opt.run_base_power_flow = false;   // operating point from stored voltages
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {};

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.ac_order_solved.at(5));

  // z_src(5) = j*5*0.2 = j1.0 ; z_line(5) = 0.01 + j*0.5
  // V2(5) = I*(z_src + z_line) = 0.01 + j1.5 ; V1(5) = I*z_src = j1.0
  const Cx v2 = vbus(r, 2, 5);
  const Cx v1 = vbus(r, 1, 5);
  CHECK_THAT(v2.real(), WithinAbs(0.01, 1e-5));
  CHECK_THAT(v2.imag(), WithinAbs(1.5, 1e-5));
  CHECK_THAT(v1.real(), WithinAbs(0.0, 1e-5));
  CHECK_THAT(v1.imag(), WithinAbs(1.0, 1e-5));

  // THD at bus 2 = |V2(5)| / |V2(1)| with |V2(1)| = 1.0
  const double expected_thd = std::abs(v2) * 100.0;  // since |V_fund| = 1
  CHECK_THAT(thd_at(r, 2), WithinRel(expected_thd, 1e-6));
  CHECK_THAT(thd_at(r, 2), WithinAbs(150.003, 1e-2));
}

// ===========================================================================
// 2. Impedance frequency scaling (lossless)
// ===========================================================================
TEST_CASE("HPF AC impedance scales linearly with harmonic order", "[harmonics][ac]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};  // lossless

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}, {7, 100.0, 0.0}};

  HarmonicStudyInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.dc_orders = {};

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);

  // Lossless: V2(h) = j*h*(x'' + x_line)*I, so |V2(7)|/|V2(5)| = 7/5.
  const double m5 = std::abs(vbus(r, 2, 5));
  const double m7 = std::abs(vbus(r, 2, 7));
  CHECK_THAT(m7 / m5, WithinRel(7.0 / 5.0, 1e-6));
  // Absolute: x''=0.2, x_line=0.1 -> |V2(5)| = 5*0.3 = 1.5
  CHECK_THAT(m5, WithinAbs(1.5, 1e-5));
  CHECK_THAT(m7, WithinAbs(2.1, 1e-5));
}

// ===========================================================================
// 3. Linearity / superposition
// ===========================================================================
TEST_CASE("HPF is linear in the injected current", "[harmonics][ac]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.01, 0.1)};

  auto run = [&](double ibase) {
    HarmonicCurrentSource src;
    src.bus = 2;
    src.i_base_pu = ibase;
    src.spectrum = {{5, 100.0, 0.0}};
    HarmonicStudyInputs in{.sources = {src}, .nics = {}};
    HPFOptions opt;
    opt.run_base_power_flow = false;
    opt.include_load_impedance = false;
    opt.ac_orders = {5};
    opt.dc_orders = {};
    return solve_harmonic_power_flow(sys, in, opt);
  };

  const Cx v1 = vbus(run(1.0), 2, 5);
  const Cx v2 = vbus(run(2.0), 2, 5);
  CHECK_THAT(v2.real(), WithinRel(2.0 * v1.real(), 1e-9));
  CHECK_THAT(v2.imag(), WithinRel(2.0 * v1.imag(), 1e-9));
}

// ===========================================================================
// 4. Radial feeder: monotone harmonic voltage + exact per-bus values
// ===========================================================================
TEST_CASE("HPF radial feeder accumulates harmonic voltage to the source bus",
          "[harmonics][ac]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ),
                  ac_bus(3, BusType::PQ), ac_bus(4, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1), ac_line(2, 2, 3, 0.0, 0.1),
                     ac_line(3, 3, 4, 0.0, 0.1)};

  HarmonicCurrentSource src;
  src.bus = 4;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};

  HarmonicStudyInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {};

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);

  // z_src(5)=j1.0, each branch z(5)=j0.5. Path-to-ground impedance:
  //   bus1: j1.0, bus2: j1.5, bus3: j2.0, bus4: j2.5
  CHECK_THAT(std::abs(vbus(r, 1, 5)), WithinAbs(1.0, 1e-5));
  CHECK_THAT(std::abs(vbus(r, 2, 5)), WithinAbs(1.5, 1e-5));
  CHECK_THAT(std::abs(vbus(r, 3, 5)), WithinAbs(2.0, 1e-5));
  CHECK_THAT(std::abs(vbus(r, 4, 5)), WithinAbs(2.5, 1e-5));

  // THD increases monotonically toward the harmonic source.
  CHECK(thd_at(r, 1) < thd_at(r, 2));
  CHECK(thd_at(r, 2) < thd_at(r, 3));
  CHECK(thd_at(r, 3) < thd_at(r, 4));
  CHECK(r.max_ac_thd_bus == 4);
}

// ===========================================================================
// 5. DC 2-bus ripple analytical
// ===========================================================================
TEST_CASE("HPF DC 2-bus ripple analytical voltage", "[harmonics][dc]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.dc.base_mva = 100.0;
  sys.dc.buses = {dc_bus(1, DCBusType::DC_V), dc_bus(2, DCBusType::DC_P)};
  sys.dc.branches = {dc_line(1, 1, 2, 0.05)};

  HarmonicCurrentSource src;
  src.bus = 2;
  src.is_dc = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{6, 100.0, 0.0}};

  HarmonicStudyInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.ac_orders = {};
  opt.dc_orders = {6};
  opt.dc_source_impedance_pu = 0.01;

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.dc_order_solved.at(6));

  // z_src,DC = 0.01, branch r = 0.05 -> V2(6) = I*(0.06) = 0.06 ; V1(6) = 0.01
  CHECK_THAT(std::abs(vbus(r, 2, 6, true)), WithinAbs(0.06, 1e-6));
  CHECK_THAT(std::abs(vbus(r, 1, 6, true)), WithinAbs(0.01, 1e-6));
  // DC THD at bus 2 = 0.06 / 1.0 = 6 %
  CHECK_THAT(thd_at(r, 2, true), WithinAbs(6.0, 1e-3));
}

// ===========================================================================
// 6. NIC hybrid AC/DC coupling
// ===========================================================================
TEST_CASE("HPF NIC bridges AC and DC through a single operating point",
          "[harmonics][nic][hybrid]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.dc.base_mva = 100.0;
  // AC: slack(1) -- line -- VSC AC port(2)
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};
  // DC: DC_V reference(10) -- line -- VSC DC port(11)
  sys.dc.buses = {dc_bus(10, DCBusType::DC_V), dc_bus(11, DCBusType::DC_P)};
  sys.dc.branches = {dc_line(1, 10, 11, 0.05)};

  VSCConverter v;
  v.index = 0;
  v.bus_ac = 2;
  v.bus_dc = 11;
  v.control_mode = ConverterMode::PQ_MODE;
  v.in_service = true;
  sys.vsc_converters = {v};

  auto run = [&](double p_mw) {
    HarmonicNIC nic;
    nic.vsc_index = 0;
    nic.ac_port = PortBehavior::GridFollowing;
    nic.dc_port = PortBehavior::GridFollowing;  // current-injecting DC ripple
    nic.ac_spectrum = {{5, 100.0, 0.0}};
    nic.dc_spectrum = {{6, 100.0, 0.0}};
    nic.s_ac_p_mw = p_mw;       // through-power operating point
    nic.s_ac_q_mvar = 0.0;
    nic.p_dc_mw = -p_mw;        // absorbed on DC side
    HarmonicStudyInputs in{.sources = {}, .nics = {nic}};
    HPFOptions opt;
    opt.run_base_power_flow = false;
    opt.include_load_impedance = false;
    opt.ac_orders = {5};
    opt.dc_orders = {6};
    opt.dc_source_impedance_pu = 0.01;
    opt.auto_nic_from_vscs = true;  // must NOT double-count the explicit NIC
    return solve_harmonic_power_flow(sys, in, opt);
  };

  // P = 100 MW = 1.0 pu, V_ac1 = 1.0 -> |I_ac1| = 1.0 -> AC 5th injection = 1.0
  HPFResult r = run(100.0);
  REQUIRE(r.ok);
  // AC port harmonic voltage: V2(5) = 1.0*(j1.0 + j0.5) = j1.5
  CHECK_THAT(std::abs(vbus(r, 2, 5)), WithinAbs(1.5, 1e-5));
  // DC port ripple: |I_dc0| = |P_dc|/Vdc0 = 1.0 -> V11(6)=1.0*(0.01+0.05)=0.06
  CHECK_THAT(std::abs(vbus(r, 11, 6, true)), WithinAbs(0.06, 1e-5));

  // Doubling the through-power doubles both the AC harmonic and the DC ripple
  // (the converter injection magnitudes scale with the operating-point current).
  HPFResult r2 = run(200.0);
  REQUIRE(r2.ok);
  CHECK_THAT(std::abs(vbus(r2, 2, 5)), WithinRel(2.0 * std::abs(vbus(r, 2, 5)), 1e-6));
  CHECK_THAT(std::abs(vbus(r2, 11, 6, true)),
             WithinRel(2.0 * std::abs(vbus(r, 11, 6, true)), 1e-6));
}

// ===========================================================================
// 7. Multi-harmonic THD formula
// ===========================================================================
TEST_CASE("HPF multi-harmonic THD matches RSS definition", "[harmonics][thd]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}, {7, 50.0, 0.0}};  // 1.0 pu @5th, 0.5 pu @7th

  HarmonicStudyInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.dc_orders = {};

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);

  // |V2(5)| = 1.0*5*0.3 = 1.5 ; |V2(7)| = 0.5*7*0.3 = 1.05
  const double v5 = std::abs(vbus(r, 2, 5));
  const double v7 = std::abs(vbus(r, 2, 7));
  CHECK_THAT(v5, WithinAbs(1.5, 1e-5));
  CHECK_THAT(v7, WithinAbs(1.05, 1e-5));
  const double thd_expected = std::sqrt(v5 * v5 + v7 * v7) / 1.0 * 100.0;
  CHECK_THAT(thd_at(r, 2), WithinRel(thd_expected, 1e-6));
}

// ===========================================================================
// 8. Default spectra, summary(), branch-flow output sanity
// ===========================================================================
TEST_CASE("HPF default spectra and result metadata", "[harmonics][meta]") {
  const auto ac = default_six_pulse_ac_spectrum();
  REQUIRE(ac.size() >= 4);
  CHECK(ac.front().order == 5);
  CHECK_THAT(ac.front().mag_percent, WithinAbs(20.0, 1e-9));

  const auto dc = default_dc_ripple_spectrum();
  REQUIRE(dc.size() >= 1);
  CHECK(dc.front().order == 6);
  CHECK_THAT(dc.front().mag_percent, WithinAbs(4.5, 1e-9));

  // Branch-flow output is populated and the summary string is non-empty.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.01, 0.1)};

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};
  HarmonicStudyInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {};
  opt.compute_branch_flows = true;

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.ac_branch_flows.size() == 1);
  // Radial branch carries the full injected current at the 5th (1.0 pu).
  CHECK_THAT(r.ac_branch_flows[0].i_by_order.at(5), WithinAbs(1.0, 1e-5));
  CHECK_FALSE(r.summary().empty());
}

// ===========================================================================
// Three-phase (abc-domain) harmonic power flow
// ===========================================================================
static ThreePhaseACBus tp_bus(int id, BusType t, double kv = 10.0) {
  ThreePhaseACBus b;
  b.index = id;
  b.bus_type = t;
  b.phase_mask = PhaseMask::abc();
  b.vm_a_pu = 1.0; b.va_a_deg = 0.0;
  b.vm_b_pu = 1.0; b.va_b_deg = -120.0;
  b.vm_c_pu = 1.0; b.va_c_deg = 120.0;
  b.base_kv = kv;
  b.in_service = true;
  return b;
}

static ThreePhaseACLine tp_line(int id, int f, int t, double x1, double x0) {
  ThreePhaseACLine ln;
  ln.index = id;
  ln.from_bus = f;
  ln.to_bus = t;
  ln.phase_mask = PhaseMask::abc();
  ln.r1_pu = 0.0; ln.x1_pu = x1; ln.b1_pu = 0.0;
  ln.r0_pu = 0.0; ln.x0_pu = x0; ln.b0_pu = 0.0;
  ln.in_service = true;
  return ln;
}

static Cx vph(const HPF3phResult& r, int bus, int order, int phase) {
  for (const auto& b : r.bus_results) {
    if (b.bus != bus) continue;
    const auto& m = (phase == 0) ? b.v_by_order_a
                  : (phase == 1) ? b.v_by_order_b : b.v_by_order_c;
    auto it = m.find(order);
    if (it != m.end()) return it->second;
  }
  return Cx(0.0, 0.0);
}

static double angdiff_deg(Cx x, Cx y) {
  double d = (std::arg(x) - std::arg(y)) * 180.0 / M_PI;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

TEST_CASE("HPF 3-phase harmonic sequence classification", "[harmonics][3ph]") {
  CHECK(harmonic_sequence_of_order(1) == 1);   // positive
  CHECK(harmonic_sequence_of_order(7) == 1);   // positive
  CHECK(harmonic_sequence_of_order(13) == 1);  // positive
  CHECK(harmonic_sequence_of_order(5) == 2);   // negative
  CHECK(harmonic_sequence_of_order(11) == 2);  // negative
  CHECK(harmonic_sequence_of_order(3) == 0);   // zero
  CHECK(harmonic_sequence_of_order(9) == 0);   // zero
}

TEST_CASE("HPF 3-phase positive-sequence harmonic is balanced (+/-120)",
          "[harmonics][3ph]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};  // x1=0.1, x0=0.3 (lossless)

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{7, 100.0, 0.0}};  // 7th: positive sequence

  ThreePhaseHarmonicInputs in;
  in.sources = {src};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {7};

  HPF3phResult r = solve_harmonic_power_flow_3ph(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.ac_order_solved.at(7));

  // Positive sequence sees z1: |V2| = I*(x''*7 + x1*7) = 1*(1.4 + 0.7) = 2.1
  const Cx va = vph(r, 2, 7, 0), vb = vph(r, 2, 7, 1), vc = vph(r, 2, 7, 2);
  CHECK_THAT(std::abs(va), WithinAbs(2.1, 1e-4));
  CHECK_THAT(std::abs(vb), WithinAbs(2.1, 1e-4));
  CHECK_THAT(std::abs(vc), WithinAbs(2.1, 1e-4));
  // Positive sequence: B lags A by 120, C leads A by 120.
  CHECK_THAT(angdiff_deg(vb, va), WithinAbs(-120.0, 1e-3));
  CHECK_THAT(angdiff_deg(vc, va), WithinAbs(120.0, 1e-3));
}

TEST_CASE("HPF 3-phase negative-sequence harmonic reverses rotation",
          "[harmonics][3ph]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};  // 5th: negative sequence

  ThreePhaseHarmonicInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};

  HPF3phResult r = solve_harmonic_power_flow_3ph(sys, in, opt);
  REQUIRE(r.ok);

  // Negative sequence also sees z1: |V2| = 1*(1.0 + 0.5) = 1.5
  const Cx va = vph(r, 2, 5, 0), vb = vph(r, 2, 5, 1), vc = vph(r, 2, 5, 2);
  CHECK_THAT(std::abs(va), WithinAbs(1.5, 1e-4));
  CHECK_THAT(std::abs(vb), WithinAbs(1.5, 1e-4));
  CHECK_THAT(std::abs(vc), WithinAbs(1.5, 1e-4));
  // Negative sequence: B leads A by 120, C lags A by 120 (reversed).
  CHECK_THAT(angdiff_deg(vb, va), WithinAbs(120.0, 1e-3));
  CHECK_THAT(angdiff_deg(vc, va), WithinAbs(-120.0, 1e-3));
}

TEST_CASE("HPF 3-phase zero-sequence (triplen) harmonic is in-phase and uses z0",
          "[harmonics][3ph]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};  // x0=0.3 != x1=0.1

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{3, 100.0, 0.0}};  // 3rd: zero sequence

  ThreePhaseHarmonicInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {3};

  HPF3phResult r = solve_harmonic_power_flow_3ph(sys, in, opt);
  REQUIRE(r.ok);

  // Zero sequence sees z0: |V2| = I*(x''*3 + x0*3) = 1*(0.6 + 0.9) = 1.5
  const Cx va = vph(r, 2, 3, 0), vb = vph(r, 2, 3, 1), vc = vph(r, 2, 3, 2);
  CHECK_THAT(std::abs(va), WithinAbs(1.5, 1e-4));
  // All three phases identical (equal magnitude AND angle).
  CHECK_THAT(angdiff_deg(vb, va), WithinAbs(0.0, 1e-3));
  CHECK_THAT(angdiff_deg(vc, va), WithinAbs(0.0, 1e-3));
  CHECK_THAT(std::abs(vb), WithinAbs(1.5, 1e-4));
  CHECK_THAT(std::abs(vc), WithinAbs(1.5, 1e-4));

  // Cross-check: a POSITIVE-sequence harmonic at the same order/current would
  // see z1 (= 0.1) instead of z0, giving a different magnitude. This confirms
  // the zero-sequence path genuinely routed through x0.
  CHECK(std::abs(va) > 1.0 * (3 * 0.2 + 3 * 0.1) - 1e-6);  // 1.5 > 0.9 (z1 result)
}

TEST_CASE("HPF 3-phase converter (NIC) balanced injection from operating point",
          "[harmonics][3ph][nic]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};

  ThreePhaseHarmonicNIC nic;
  nic.bus_ac = 2;
  nic.s_ac_p_mw = 100.0;   // 1.0 pu total; per-phase S = 1/3 pu
  nic.s_ac_q_mvar = 0.0;
  nic.ac_spectrum = {{5, 100.0, 0.0}};  // |I_a1| = 1/3 -> 5th injection = 1/3

  ThreePhaseHarmonicInputs in{.sources = {}, .nics = {nic}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};

  HPF3phResult r = solve_harmonic_power_flow_3ph(sys, in, opt);
  REQUIRE(r.ok);

  // |I_a1| = (1/3)/|V_a1| = 1/3 ; 5th injection = 1/3 ; z1(5) = j1.5
  // |V2_a(5)| = (1/3) * 1.5 = 0.5
  const Cx va = vph(r, 2, 5, 0), vb = vph(r, 2, 5, 1), vc = vph(r, 2, 5, 2);
  CHECK_THAT(std::abs(va), WithinAbs(0.5, 1e-4));
  CHECK_THAT(std::abs(vb), WithinAbs(0.5, 1e-4));
  CHECK_THAT(std::abs(vc), WithinAbs(0.5, 1e-4));
  // 5th is negative sequence.
  CHECK_THAT(angdiff_deg(vb, va), WithinAbs(120.0, 1e-3));

  // Balanced source -> equal per-phase THD.
  double ta = 0, tb = 0, tc = 0;
  for (const auto& b : r.bus_results)
    if (b.bus == 2) { ta = b.thd_a_pct; tb = b.thd_b_pct; tc = b.thd_c_pct; }
  CHECK_THAT(ta, WithinRel(tb, 1e-6));
  CHECK_THAT(tb, WithinRel(tc, 1e-6));
  CHECK(ta > 0.0);
  CHECK_FALSE(r.summary().empty());
}

// ===========================================================================
// Unbalanced three-phase source (per-phase magnitudes, decoupled line z0=z1)
// ===========================================================================
TEST_CASE("HPF 3-phase unbalanced source yields per-phase distortion",
          "[harmonics][3ph][unbalanced]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.1)};  // z0 = z1 -> phases decouple

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = false;            // per-phase magnitudes, no sequence rotation
  src.i_base_pu_a = 1.0;
  src.i_base_pu_b = 0.5;
  src.i_base_pu_c = 0.0;
  src.spectrum = {{5, 100.0, 0.0}};

  ThreePhaseHarmonicInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};

  HPF3phResult r = solve_harmonic_power_flow_3ph(sys, in, opt);
  REQUIRE(r.ok);

  // Decoupled phases: V_phi(5) = I_phi * (x''*5 + x1*5) = I_phi * 1.5
  CHECK_THAT(std::abs(vph(r, 2, 5, 0)), WithinAbs(1.5, 1e-4));   // a: 1.0 * 1.5
  CHECK_THAT(std::abs(vph(r, 2, 5, 1)), WithinAbs(0.75, 1e-4));  // b: 0.5 * 1.5
  CHECK_THAT(std::abs(vph(r, 2, 5, 2)), WithinAbs(0.0, 1e-6));   // c: 0

  double ta = 0, tb = 0, tc = 0;
  for (const auto& b : r.bus_results)
    if (b.bus == 2) { ta = b.thd_a_pct; tb = b.thd_b_pct; tc = b.thd_c_pct; }
  CHECK(ta > tb);
  CHECK(tb > tc);
  CHECK_THAT(tc, WithinAbs(0.0, 1e-6));
}

// ===========================================================================
// Standards compliance: IEEE 519-2014 / GB-T 14549-1993 limit checks
// ===========================================================================
TEST_CASE("Harmonic voltage limits per standard and voltage level",
          "[harmonics][limits]") {
  using HS = HarmonicStandard;
  // IEEE 519-2014 Table 1: (individual, THD)
  CHECK(harmonic_voltage_limits(HS::IEEE519_2014, 0.4, 5)   == std::pair(5.0, 8.0));
  CHECK(harmonic_voltage_limits(HS::IEEE519_2014, 10.0, 5)  == std::pair(3.0, 5.0));
  CHECK(harmonic_voltage_limits(HS::IEEE519_2014, 120.0, 5) == std::pair(1.5, 2.5));
  CHECK(harmonic_voltage_limits(HS::IEEE519_2014, 230.0, 5) == std::pair(1.0, 1.5));
  // GB/T 14549-1993: odd vs even individual limits differ.
  CHECK(harmonic_voltage_limits(HS::GBT14549_1993, 0.38, 5) == std::pair(4.0, 5.0));
  CHECK(harmonic_voltage_limits(HS::GBT14549_1993, 0.38, 2) == std::pair(2.0, 5.0));
  CHECK(harmonic_voltage_limits(HS::GBT14549_1993, 10.0, 5) == std::pair(3.2, 4.0));
  CHECK(harmonic_voltage_limits(HS::GBT14549_1993, 110.0, 5)== std::pair(1.6, 2.0));
}

// Build a one-bus HybridPowerSystem result by hand for controlled limit checks.
static HybridPowerSystem one_ac_bus_sys(int bus_id, double base_kv) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus b = ac_bus(bus_id, BusType::PQ, base_kv);
  sys.ac.buses = {b};
  return sys;
}

static HPFResult one_bus_result(int bus_id, std::map<int, Cx> spectrum,
                                double v_fund = 1.0) {
  HPFResult r;
  r.ok = true;
  HarmonicBusResult br;
  br.bus = bus_id;
  br.is_dc = false;
  br.v_fund_pu = v_fund;
  br.v_by_order = std::move(spectrum);
  // THD = RSS(h>1) / v_fund
  double acc = 0.0;
  for (const auto& [o, v] : br.v_by_order)
    if (o != 1) acc += std::norm(v);
  br.thd_pct = std::sqrt(acc) / v_fund * 100.0;
  r.ac_bus_results = {br};
  return r;
}

TEST_CASE("Harmonic compliance flags individual and THD violations",
          "[harmonics][limits]") {
  using HS = HarmonicStandard;

  // 10 kV bus, single 5th at 2% -> IHD 2% < 3%, THD 2% < 5% -> compliant.
  {
    auto sys = one_ac_bus_sys(7, 10.0);
    auto r = one_bus_result(7, {{1, Cx(1, 0)}, {5, Cx(0.02, 0)}});
    auto rep = check_harmonic_limits(r, sys, HS::IEEE519_2014);
    CHECK(rep.all_compliant);
    CHECK(rep.n_violations == 0);
    CHECK(rep.checks.size() == 1);
    CHECK(rep.checks[0].thd_ok);
    CHECK(rep.checks[0].ihd_ok);
  }

  // 10 kV bus, single 5th at 4% -> IHD 4% > 3% (individual violation) but
  // THD 4% < 5% (THD ok).  Overall non-compliant.
  {
    auto sys = one_ac_bus_sys(7, 10.0);
    auto r = one_bus_result(7, {{1, Cx(1, 0)}, {5, Cx(0.04, 0)}});
    auto rep = check_harmonic_limits(r, sys, HS::IEEE519_2014);
    CHECK_FALSE(rep.all_compliant);
    CHECK(rep.n_violations == 1);
    CHECK(rep.checks[0].thd_ok);
    CHECK_FALSE(rep.checks[0].ihd_ok);
    CHECK(rep.checks[0].worst_ihd_order == 5);
    CHECK_THAT(rep.checks[0].worst_ihd_pct, WithinAbs(4.0, 1e-6));
  }

  // Standard differentiator: 3rd harmonic at 3.1% on a 10 kV bus.
  //   IEEE 519 individual limit = 3.0% -> violation.
  //   GB/T 14549 odd limit      = 3.2% -> compliant.
  {
    auto sys = one_ac_bus_sys(7, 10.0);
    auto r = one_bus_result(7, {{1, Cx(1, 0)}, {3, Cx(0.031, 0)}});
    auto ieee = check_harmonic_limits(r, sys, HS::IEEE519_2014);
    auto gbt = check_harmonic_limits(r, sys, HS::GBT14549_1993);
    CHECK_FALSE(ieee.all_compliant);
    CHECK(gbt.all_compliant);
  }
}

TEST_CASE("Harmonic compliance end-to-end from the solver", "[harmonics][limits]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};  // 10 kV
  sys.ac.branches = {ac_line(1, 1, 2, 0.01, 0.1)};

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};  // huge 5th -> THD ~ 150 %
  HarmonicStudyInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {};

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);
  auto rep = check_harmonic_limits(r, sys, HarmonicStandard::IEEE519_2014);
  CHECK_FALSE(rep.all_compliant);
  CHECK(rep.n_violations >= 1);       // bus 2 (and bus 1) grossly exceed limits
  CHECK(rep.worst_ratio > 1.0);
  CHECK_FALSE(rep.summary().empty());
}


