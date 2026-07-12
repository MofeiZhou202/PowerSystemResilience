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
//   z_branch,DC = r + j*ripple_order*x   (when configured in dc_ripple_model)
//
// Tests:
//   1. AC 2-bus: V2(h) = I*(z_src+z_line), V1(h) = I*z_src, THD = |Vh|/|V1|
//   2. AC impedance frequency scaling (lossless): |V(7)|/|V(5)| = 7/5
//   3. AC linearity / superposition: 2*I -> 2*V
//   4. AC radial feeder: monotone harmonic voltage, exact per-bus values
//   5. DC 2-bus ripple: V2(r) = I*(z_src,DC + r_branch)
//   6. DC ripple branch inductance (order-dependent) analytical check
//   7. DC ripple bus shunt capacitance attenuation analytical check
//   8. NIC hybrid coupling: I_ac1 = conj(S)/conj(V), AC & DC ripple, P-scaling
//   9. Multi-harmonic THD formula
//  10. Default spectra, summary(), branch-flow output sanity

#include <cmath>
#include <complex>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/analysis/harmonics_power_flow.hpp"
#include "hacdcpf/io/case_builders.hpp"

using namespace hacdcpf;
using namespace hacdcpf::harmonics;
using Cx = std::complex<double>;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
}

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

TEST_CASE("HPF DC ripple branch inductance is order-dependent", "[harmonics][dc]") {
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
  opt.dc_ripple_model.branch_x_pu[1] = 0.02;

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.dc_order_solved.at(6));

  // Z_tot(6) = z_src + z_branch = 0.01 + (0.05 + j*6*0.02) = 0.06 + j0.12.
  Cx v2 = vbus(r, 2, 6, true);
  CHECK_THAT(v2.real(), WithinAbs(0.06, 1e-6));
  CHECK_THAT(v2.imag(), WithinAbs(0.12, 1e-6));
  CHECK_THAT(std::abs(v2), WithinAbs(std::sqrt(0.06 * 0.06 + 0.12 * 0.12), 1e-6));
}

TEST_CASE("HPF DC ripple shunt capacitance attenuates bus ripple", "[harmonics][dc]") {
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

  HPFOptions opt_no_cap;
  opt_no_cap.run_base_power_flow = false;
  opt_no_cap.ac_orders = {};
  opt_no_cap.dc_orders = {6};
  opt_no_cap.dc_source_impedance_pu = 0.01;
  HPFResult r_no_cap = solve_harmonic_power_flow(sys, in, opt_no_cap);
  REQUIRE(r_no_cap.ok);

  HPFOptions opt_cap = opt_no_cap;
  opt_cap.dc_ripple_model.bus_b_pu[2] = 0.2;
  HPFResult r_cap = solve_harmonic_power_flow(sys, in, opt_cap);
  REQUIRE(r_cap.ok);

  // Two-node closed form with current source injected at bus 2:
  // V2 = I * Y11 / (Y11*Y22 - Y12*Y21).
  const Cx zsrc(0.01, 0.0);
  const Cx zbr(0.05, 0.0);
  const Cx ysrc = Cx(1.0, 0.0) / zsrc;
  const Cx ybr = Cx(1.0, 0.0) / zbr;
  const Cx ycap(0.0, 6.0 * 0.2);
  const Cx y11 = ysrc + ybr;
  const Cx y22 = ybr + ycap;
  const Cx y12 = -ybr;
  const Cx det = y11 * y22 - y12 * y12;
  const Cx v2_expected = y11 / det;

  CHECK_THAT(std::abs(vbus(r_cap, 2, 6, true)), WithinRel(std::abs(v2_expected), 1e-6));
  CHECK(std::abs(vbus(r_cap, 2, 6, true)) < std::abs(vbus(r_no_cap, 2, 6, true)));
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
  double d = (std::arg(x) - std::arg(y)) * 180.0 / kPi;
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

// ===========================================================================
// Coupled three-phase AC + DC harmonic power flow (full NIC bridge)
// ===========================================================================
// Shared 2-bus AC (slack -- line -- VSC AC port) + 2-bus DC (DC_V -- line -- VSC
// DC port) test bed.  z0 = z1 on the AC line so phases decouple for clean
// per-phase analytics; z_dc(port->ground) = 0.01 + 0.05 = 0.06.
namespace {
struct HybridBed {
  ThreePhaseACSystem ac;
  DCSystem dc;
};
HybridBed make_hybrid_bed() {
  HybridBed b;
  b.ac.base_mva = 100.0;
  b.ac.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  b.ac.lines = {tp_line(1, 1, 2, 0.1, 0.1)};
  b.dc.base_mva = 100.0;
  b.dc.buses = {dc_bus(10, DCBusType::DC_V), dc_bus(11, DCBusType::DC_P)};
  b.dc.branches = {dc_line(1, 10, 11, 0.05)};
  return b;
}
Cx vac_hyb(const HPFHybrid3phResult& r, int bus, int order, int phase) {
  for (const auto& b : r.ac_bus_results) {
    if (b.bus != bus) continue;
    const auto& m = (phase == 0) ? b.v_by_order_a
                  : (phase == 1) ? b.v_by_order_b : b.v_by_order_c;
    auto it = m.find(order);
    if (it != m.end()) return it->second;
  }
  return Cx(0, 0);
}
Cx vdc_hyb(const HPFHybrid3phResult& r, int bus, int order) {
  for (const auto& b : r.dc_bus_results)
    if (b.bus == bus) { auto it = b.v_by_order.find(order); if (it != b.v_by_order.end()) return it->second; }
  return Cx(0, 0);
}
}  // namespace

TEST_CASE("HPF hybrid coupled solve reduces to decoupled when gains are zero",
          "[harmonics][hybrid][coupling]") {
  auto bed = make_hybrid_bed();
  ThreePhaseHybridNIC nic;
  nic.bus_ac = 2;
  nic.bus_dc = 11;
  nic.dc_port = PortBehavior::GridFollowing;  // bus 10 (DC_V) provides the ground
  nic.s_ac_p_mw = 100.0;       // |I_a1| = 1/3
  nic.p_dc_mw = -100.0;        // |I_dc0| = 1
  nic.ac_spectrum = {{5, 100.0, 0.0}};
  nic.dc_spectrum = {{6, 100.0, 0.0}};
  nic.k_ad = {0.0, 0.0};
  nic.k_da = {0.0, 0.0};

  ThreePhaseHybridInputs in;
  in.nics = {nic};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {6};
  opt.dc_source_impedance_pu = 0.01;

  HPFHybrid3phResult r = solve_harmonic_power_flow_3ph_hybrid(bed.ac, bed.dc, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.combined_solved);
  // AC 5th: (1/3) * (x''*5 + x1*5) = (1/3) * 1.5 = 0.5
  CHECK_THAT(std::abs(vac_hyb(r, 2, 5, 0)), WithinAbs(0.5, 1e-4));
  // DC 6th: 1.0 * (0.01 + 0.05) = 0.06
  CHECK_THAT(std::abs(vdc_hyb(r, 11, 6)), WithinAbs(0.06, 1e-4));
  CHECK_FALSE(r.summary().empty());
}

TEST_CASE("HPF hybrid AC->DC coupling: AC harmonic drives DC ripple via k_da",
          "[harmonics][hybrid][coupling]") {
  auto bed = make_hybrid_bed();
  ThreePhaseHybridNIC nic;
  nic.bus_ac = 2;
  nic.bus_dc = 11;
  nic.dc_port = PortBehavior::GridFollowing;
  nic.s_ac_p_mw = 0.0;   // no NIC self-injection
  nic.p_dc_mw = 0.0;
  nic.k_ad = {0.0, 0.0};
  nic.k_da = {2.0, 0.0};  // DC current per AC voltage

  // External AC source drives a known 5th-harmonic port voltage.
  ThreePhaseHarmonicSource ac_src;
  ac_src.bus = 2;
  ac_src.balanced = true;
  ac_src.i_base_pu = 1.0;
  ac_src.spectrum = {{5, 100.0, 0.0}};

  ThreePhaseHybridInputs in;
  in.ac_sources = {ac_src};
  in.nics = {nic};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {6};
  opt.dc_source_impedance_pu = 0.01;

  HPFHybrid3phResult r = solve_harmonic_power_flow_3ph_hybrid(bed.ac, bed.dc, in, opt);
  REQUIRE(r.ok);
  // AC port 5th: 1.0 * 1.5 = 1.5 (independent of DC since k_ad = 0)
  const Cx vac5 = vac_hyb(r, 2, 5, 0);
  CHECK_THAT(std::abs(vac5), WithinAbs(1.5, 1e-4));
  // DC 6th = z_dc * (k_da * V_ac,a(5)) = 0.06 * 2.0 * 1.5 = 0.18
  CHECK_THAT(std::abs(vdc_hyb(r, 11, 6)), WithinAbs(0.18, 1e-3));

  // With k_da = 0 the DC ripple vanishes.
  in.nics[0].k_da = {0.0, 0.0};
  HPFHybrid3phResult r0 = solve_harmonic_power_flow_3ph_hybrid(bed.ac, bed.dc, in, opt);
  CHECK(std::abs(vdc_hyb(r0, 11, 6)) < 1e-6);
}

TEST_CASE("HPF hybrid DC->AC coupling: DC ripple drives AC 5th & 7th via k_ad",
          "[harmonics][hybrid][coupling]") {
  auto bed = make_hybrid_bed();
  ThreePhaseHybridNIC nic;
  nic.bus_ac = 2;
  nic.bus_dc = 11;
  nic.dc_port = PortBehavior::GridFollowing;
  nic.s_ac_p_mw = 0.0;
  nic.p_dc_mw = 0.0;
  nic.k_ad = {0.5, 0.0};  // AC current per DC voltage
  nic.k_da = {0.0, 0.0};

  // External DC ripple source drives a known 6th-order DC voltage.
  HarmonicCurrentSource dc_src;
  dc_src.bus = 11;
  dc_src.is_dc = true;
  dc_src.i_base_pu = 1.0;
  dc_src.spectrum = {{6, 100.0, 0.0}};

  ThreePhaseHybridInputs in;
  in.dc_sources = {dc_src};
  in.nics = {nic};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};   // both couple to DC 6 (|5-6|=1, |7-6|=1)
  opt.dc_orders = {6};
  opt.dc_source_impedance_pu = 0.01;

  HPFHybrid3phResult r = solve_harmonic_power_flow_3ph_hybrid(bed.ac, bed.dc, in, opt);
  REQUIRE(r.ok);
  // DC 6th = 1.0 * 0.06 = 0.06 (independent of AC since k_da = 0)
  CHECK_THAT(std::abs(vdc_hyb(r, 11, 6)), WithinAbs(0.06, 1e-4));
  // AC 5th injection = k_ad * V_dc(6) = 0.5 * 0.06 = 0.03 ; z_ac(5) = 1.5
  //   |V_ac,a(5)| = 0.03 * 1.5 = 0.045
  CHECK_THAT(std::abs(vac_hyb(r, 2, 5, 0)), WithinAbs(0.045, 1e-4));
  // AC 7th injection = 0.03 ; z_ac(7) = (x''+x1)*7 = 0.3*7 = 2.1
  //   |V_ac,a(7)| = 0.03 * 2.1 = 0.063
  CHECK_THAT(std::abs(vac_hyb(r, 2, 7, 0)), WithinAbs(0.063, 1e-4));

  // With k_ad = 0 the AC harmonics vanish (no DC->AC path).
  in.nics[0].k_ad = {0.0, 0.0};
  HPFHybrid3phResult r0 = solve_harmonic_power_flow_3ph_hybrid(bed.ac, bed.dc, in, opt);
  CHECK(std::abs(vac_hyb(r0, 2, 5, 0)) < 1e-6);
  CHECK(std::abs(vac_hyb(r0, 2, 7, 0)) < 1e-6);
}

// ===========================================================================
// Skin effect: frequency-dependent series resistance R(h)
// ===========================================================================
TEST_CASE("HPF skin effect scales series resistance with sqrt(h)", "[harmonics][skin]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.1, 0.1)};  // r1 = 0.1, x1 = 0.1

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};
  HarmonicStudyInputs in{.sources = {src}, .nics = {}};

  auto run = [&](SkinEffectModel m, double k) {
    HPFOptions opt;
    opt.run_base_power_flow = false;
    opt.include_load_impedance = false;
    opt.ac_orders = {5};
    opt.dc_orders = {};
    opt.skin_effect = m;
    opt.skin_coefficient = k;
    return solve_harmonic_power_flow(sys, in, opt);
  };

  // V2(5) = I*(z_src + z_line) = (R_line) + j*(5*0.2 + 5*0.1) = R_line + j1.5
  // Re(V2) is exactly the line resistance at order 5; Im(V2) = 1.5 always.
  const HPFResult r_none = run(SkinEffectModel::None, 0.0);
  const HPFResult r_sqrt = run(SkinEffectModel::SqrtOrder, 0.0);
  const Cx v_none = vbus(r_none, 2, 5);
  const Cx v_sqrt = vbus(r_sqrt, 2, 5);
  const Cx v_prop = vbus(run(SkinEffectModel::ProportionalSqrt, 1.0), 2, 5);

  CHECK_THAT(v_none.real(), WithinAbs(0.1, 1e-5));
  CHECK_THAT(v_none.imag(), WithinAbs(1.5, 1e-5));
  // SqrtOrder: R(5) = 0.1 * sqrt(5)
  CHECK_THAT(v_sqrt.real(), WithinAbs(0.1 * std::sqrt(5.0), 1e-5));
  CHECK_THAT(v_sqrt.imag(), WithinAbs(1.5, 1e-5));
  REQUIRE(r_sqrt.ac_branch_flows.size() == 1);
  REQUIRE(r_sqrt.ac_branch_flows[0].i_by_order.count(5) == 1);
  CHECK_THAT(r_sqrt.ac_branch_flows[0].i_by_order.at(5), WithinAbs(1.0, 1e-9));
  // ProportionalSqrt (k=1): R(5) = 0.1 * (1 + sqrt(5))
  CHECK_THAT(v_prop.real(), WithinAbs(0.1 * (1.0 + std::sqrt(5.0)), 1e-5));
}

// ===========================================================================
// Transformer vector groups: zero-sequence blocking and ±30° shift
// ===========================================================================
static ThreePhaseTransformer tp_xfmr(int id, int hv, int lv, const std::string& vg) {
  ThreePhaseTransformer t;
  t.index = id;
  t.hv_bus = hv;
  t.lv_bus = lv;
  t.in_service = true;
  t.hv_phase_mask = PhaseMask::abc();
  t.lv_phase_mask = PhaseMask::abc();
  t.sn_mva = 100.0;
  t.vk_percent = 10.0;
  t.vkr_percent = 1.0;
  t.vector_group = vg;
  return t;
}

TEST_CASE("HPF transformer delta winding blocks triplen (zero-seq) harmonics",
          "[harmonics][3ph][transformer]") {
  auto build = [&](const std::string& vg) {
    ThreePhaseACSystem sys;
    sys.base_mva = 100.0;
    sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
    sys.transformers = {tp_xfmr(1, 1, 2, vg)};
    return sys;
  };
  auto run = [&](const std::string& vg, int order) {
    ThreePhaseACSystem sys = build(vg);
    ThreePhaseHarmonicSource src;
    src.bus = 2;
    src.balanced = true;
    src.i_base_pu = 1.0;
    src.spectrum = {{order, 100.0, 0.0}};
    ThreePhaseHarmonicInputs in{.sources = {src}, .nics = {}};
    HPFOptions opt;
    opt.run_base_power_flow = false;
    opt.include_load_impedance = false;
    opt.ac_orders = {order};
    return solve_harmonic_power_flow_3ph(sys, in, opt);
  };

  // 3rd harmonic = zero sequence.
  const double v3_ygyg = std::abs(vph(run("YNyn0", 3), 2, 3, 0));
  const double v3_ynd  = std::abs(vph(run("YNd11", 3), 2, 3, 0));
  // Yg-Yg passes the 3rd (finite, O(1)); Yg-D blocks it (near-open -> huge).
  CHECK(v3_ygyg < 10.0);
  CHECK(v3_ynd > 1.0e6 * v3_ygyg);

  // 7th harmonic = positive sequence: passes through BOTH connections similarly.
  const double v7_ygyg = std::abs(vph(run("YNyn0", 7), 2, 7, 0));
  const double v7_ynd  = std::abs(vph(run("YNd11", 7), 2, 7, 0));
  CHECK(v7_ygyg < 10.0);
  CHECK(v7_ynd < 10.0);
  CHECK_THAT(v7_ynd, WithinRel(v7_ygyg, 0.5));  // same order of magnitude
}

TEST_CASE("HPF transformer vector group introduces a 30-degree phase shift",
          "[harmonics][3ph][transformer]") {
  auto run = [&](const std::string& vg) {
    ThreePhaseACSystem sys;
    sys.base_mva = 100.0;
    sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
    sys.transformers = {tp_xfmr(1, 1, 2, vg)};
    ThreePhaseHarmonicSource src;
    src.bus = 2;
    src.balanced = true;
    src.i_base_pu = 1.0;
    src.spectrum = {{7, 100.0, 0.0}};  // positive sequence
    ThreePhaseHarmonicInputs in{.sources = {src}, .nics = {}};
    HPFOptions opt;
    opt.run_base_power_flow = false;
    opt.include_load_impedance = false;
    opt.ac_orders = {7};
    return solve_harmonic_power_flow_3ph(sys, in, opt);
  };

  // Voltage transferred to the HV (wye) side for clock 1 vs clock 11.
  const Cx v1_c1 = vph(run("YNd1"), 1, 7, 0);
  const Cx v1_c11 = vph(run("YNd11"), 1, 7, 0);
  // Same magnitude, phase differs by 2*30 = 60 degrees (YIII vs YIII^T).
  CHECK_THAT(std::abs(v1_c11), WithinRel(std::abs(v1_c1), 1e-6));
  CHECK_THAT(std::abs(angdiff_deg(v1_c11, v1_c1)), WithinAbs(60.0, 1e-2));
}

// ===========================================================================
// Newton-Raphson HPF with nonlinear (voltage-dependent) resources
// ===========================================================================
static Cx vnewton(const HPFNewtonResult& r, int bus, int order) {
  for (const auto& b : r.ac_bus_results)
    if (b.bus == bus) { auto it = b.v_by_order.find(order); if (it != b.v_by_order.end()) return it->second; }
  return Cx(0, 0);
}

TEST_CASE("Newton HPF reduces to the linear solve with no nonlinear resource",
          "[harmonics][newton]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};
  HarmonicStudyInputs lin{.sources = {src}, .nics = {}};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {};

  // Linear reference.
  HPFResult lin_res = solve_harmonic_power_flow(sys, lin, opt);
  // Newton with no nonlinear resources.
  HPFNewtonResult nr = solve_harmonic_power_flow_newton(sys, {}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.converged);
  CHECK(nr.iterations.at(5) <= 1);          // already at the solution
  const Cx vn = vnewton(nr, 2, 5);
  const Cx vl = vbus(lin_res, 2, 5);
  CHECK_THAT(vn.real(), WithinAbs(vl.real(), 1e-9));
  CHECK_THAT(vn.imag(), WithinAbs(vl.imag(), 1e-9));
  CHECK_THAT(vn.imag(), WithinAbs(1.5, 1e-6));  // V2(5) = 1.0 * j1.5
}

TEST_CASE("Newton HPF converges to the analytic nonlinear fixed point",
          "[harmonics][newton]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};  // lossless

  // Nonlinear resource at bus 2:  I = i_src - g2 * V^2  with i_src = 1, g2 = 0.1.
  HarmonicNonlinearSource nl;
  nl.bus = 2;
  nl.i_src[5] = Cx(1.0, 0.0);
  nl.g2[5] = Cx(0.1, 0.0);

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {};
  opt.newton_tol = 1e-12;

  HPFNewtonResult nr = solve_harmonic_power_flow_newton(sys, {nl}, {}, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.converged);
  CHECK(nr.iterations.at(5) >= 1);     // genuine iteration
  CHECK(nr.iterations.at(5) < 10);
  CHECK(nr.final_residual.at(5) < 1e-10);

  // Reduced equation at bus 2:  g2*V^2 + A*V - i_src = 0,
  //   A = 1/Z_th,  Z_th = z_src(5) + z_line(5) = j1.0 + j0.5 = j1.5.
  const Cx A = Cx(1.0, 0.0) / Cx(0.0, 1.5);
  const Cx V2 = vnewton(nr, 2, 5);
  const Cx residual = Cx(0.1, 0.0) * V2 * V2 + A * V2 - Cx(1.0, 0.0);
  CHECK_THAT(std::abs(residual), WithinAbs(0.0, 1e-6));

  // The nonlinearity genuinely changes the answer vs the linear case (V=j1.5).
  CHECK(std::abs(std::abs(V2) - 1.5) > 0.1);

  // Cross-check against the closed-form quadratic root nearest the linear seed.
  const Cx disc = std::sqrt(A * A + Cx(0.4, 0.0));  // A^2 + 4*g2*i_src
  const Cx root1 = (-A + disc) / Cx(0.2, 0.0);
  const Cx root2 = (-A - disc) / Cx(0.2, 0.0);
  const Cx seed = Cx(1.0, 0.0) / A;  // linear solution
  const Cx expected = (std::abs(root1 - seed) < std::abs(root2 - seed)) ? root1 : root2;
  CHECK_THAT(V2.real(), WithinAbs(expected.real(), 1e-6));
  CHECK_THAT(V2.imag(), WithinAbs(expected.imag(), 1e-6));
  CHECK_FALSE(nr.summary().empty());
}

// ===========================================================================
// Three-phase Newton-Raphson HPF (per-phase nonlinear resources)
// ===========================================================================
TEST_CASE("Newton 3-phase reduces to the linear 3-phase solve", "[harmonics][newton][3ph]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{7, 100.0, 0.0}};
  ThreePhaseHarmonicInputs lin{.sources = {src}, .nics = {}};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {7};

  HPF3phResult linres = solve_harmonic_power_flow_3ph(sys, lin, opt);
  HPF3phResult nr = solve_harmonic_power_flow_3ph_newton(sys, {}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.newton_converged);
  CHECK(nr.newton_iterations.at(7) <= 1);
  for (int ph = 0; ph < 3; ++ph) {
    const Cx vn = vph(nr, 2, 7, ph);
    const Cx vl = vph(linres, 2, 7, ph);
    CHECK_THAT(vn.real(), WithinAbs(vl.real(), 1e-9));
    CHECK_THAT(vn.imag(), WithinAbs(vl.imag(), 1e-9));
  }
}

TEST_CASE("Newton 3-phase converges to per-phase analytic nonlinear roots",
          "[harmonics][newton][3ph]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.1)};  // z0 = z1 -> phases decouple

  // Per-phase nonlinear resource:  I_phi = i_src_phi - 0.1 * V_phi^2
  ThreePhaseNonlinearSource nl;
  nl.bus = 2;
  nl.i_src[5] = {Cx(1.0, 0.0), Cx(0.5, 0.0), Cx(0.0, 0.0)};
  nl.g2[5] = {Cx(0.1, 0.0), Cx(0.1, 0.0), Cx(0.1, 0.0)};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.newton_tol = 1e-12;

  HPF3phResult nr = solve_harmonic_power_flow_3ph_newton(sys, {nl}, {}, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.newton_converged);
  CHECK(nr.newton_residual.at(5) < 1e-10);

  // Each phase is an independent quadratic g2*V^2 + A*V - i_src = 0,
  //   A = 1/Z_th = 1/(j1.5) (z_src(5)=j1.0, z_line(5)=j0.5; decoupled line).
  const Cx A = Cx(1.0, 0.0) / Cx(0.0, 1.5);
  const Cx g2 = Cx(0.1, 0.0);
  auto qroot = [&](Cx c) {
    const Cx disc = std::sqrt(A * A + 4.0 * g2 * c);
    const Cx r1 = (-A + disc) / (2.0 * g2), r2 = (-A - disc) / (2.0 * g2);
    const Cx seed = (std::abs(c) > 1e-15) ? c / A : Cx(0.0, 0.0);
    return (std::abs(r1 - seed) < std::abs(r2 - seed)) ? r1 : r2;
  };

  const Cx ea = qroot(Cx(1.0, 0.0)), eb = qroot(Cx(0.5, 0.0));
  const Cx va = vph(nr, 2, 5, 0), vb = vph(nr, 2, 5, 1), vc = vph(nr, 2, 5, 2);
  CHECK_THAT(va.real(), WithinAbs(ea.real(), 1e-6));
  CHECK_THAT(va.imag(), WithinAbs(ea.imag(), 1e-6));
  CHECK_THAT(vb.real(), WithinAbs(eb.real(), 1e-6));
  CHECK_THAT(vb.imag(), WithinAbs(eb.imag(), 1e-6));
  CHECK_THAT(std::abs(vc), WithinAbs(0.0, 1e-9));  // zero excitation -> zero
  // Phase a is more heavily loaded -> larger distortion than phase b.
  CHECK(std::abs(va) > std::abs(vb));
}

// ===========================================================================
// Real/imaginary (2N) Newton: non-holomorphic constant-power harmonic loads
// ===========================================================================
TEST_CASE("Newton real-form reduces to the linear solve without CP loads",
          "[harmonics][newton][real]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};
  HarmonicStudyInputs lin{.sources = {src}, .nics = {}};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {};

  HPFResult linres = solve_harmonic_power_flow(sys, lin, opt);
  HPFNewtonResult nr = solve_harmonic_power_flow_newton_real(sys, {}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.converged);
  CHECK(nr.iterations.at(5) <= 1);
  const Cx vn = vnewton(nr, 2, 5), vl = vbus(linres, 2, 5);
  CHECK_THAT(vn.real(), WithinAbs(vl.real(), 1e-9));
  CHECK_THAT(vn.imag(), WithinAbs(vl.imag(), 1e-9));
}

TEST_CASE("Newton real-form converges to the constant-power analytic fixed point",
          "[harmonics][newton][real]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};  // lossless -> A = 1/(j1.5)

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};  // I_src = 1.0
  HarmonicStudyInputs lin{.sources = {src}, .nics = {}};

  ConstantPowerHarmonicLoad cp;     // draws S = 0.2 at the 5th
  cp.bus = 2;
  cp.s_set[5] = Cx(0.2, 0.0);

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {};
  opt.newton_tol = 1e-12;

  HPFNewtonResult nr = solve_harmonic_power_flow_newton_real(sys, {cp}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.converged);
  CHECK(nr.iterations.at(5) >= 1);
  CHECK(nr.final_residual.at(5) < 1e-10);

  // Analytic fixed point: |A|^2 m^2 + (2 Re(A S) - |I_src|^2) m + |S|^2 = 0,
  //   m = |V|^2 ; then V = conj((A m + conj(S)) / I_src).
  const Cx A = Cx(1.0, 0.0) / Cx(0.0, 1.5);
  const Cx S(0.2, 0.0), Isrc(1.0, 0.0);
  const double aa = std::norm(A);
  const double bb = 2.0 * std::real(A * S) - std::norm(Isrc);
  const double cc = std::norm(S);
  const double disc = std::sqrt(bb * bb - 4.0 * aa * cc);
  const double m1 = (-bb + disc) / (2.0 * aa), m2 = (-bb - disc) / (2.0 * aa);
  const double mlin = std::norm(Isrc / A);
  const double m = (std::abs(m1 - mlin) < std::abs(m2 - mlin)) ? m1 : m2;
  const Cx V2exp = std::conj((A * m + std::conj(S)) / Isrc);

  const Cx V2 = vnewton(nr, 2, 5);
  CHECK_THAT(V2.real(), WithinAbs(V2exp.real(), 1e-6));
  CHECK_THAT(V2.imag(), WithinAbs(V2exp.imag(), 1e-6));
  // Drawing real power makes a real component appear (linear answer was pure j1.5).
  CHECK(std::abs(V2.real()) > 0.1);
  CHECK_FALSE(nr.summary().empty());
}

// ===========================================================================
// Stacked all-orders Newton: cross-order (frequency-mixing) nonlinear resources
// ===========================================================================
TEST_CASE("Newton coupled reduces to per-order linear solves without mixing",
          "[harmonics][newton][crossorder]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}, {7, 100.0, 0.0}};
  HarmonicStudyInputs lin{.sources = {src}, .nics = {}};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.dc_orders = {};

  HPFResult linres = solve_harmonic_power_flow(sys, lin, opt);
  HPFNewtonResult nr = solve_harmonic_power_flow_newton_coupled(sys, {}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.converged);
  CHECK(nr.iterations.at(0) <= 1);
  for (int h : {5, 7}) {
    const Cx vn = vnewton(nr, 2, h), vl = vbus(linres, 2, h);
    CHECK_THAT(vn.real(), WithinAbs(vl.real(), 1e-9));
    CHECK_THAT(vn.imag(), WithinAbs(vl.imag(), 1e-9));
  }
}

TEST_CASE("Newton coupled solves one-way frequency mixing (7th^2 -> 5th)",
          "[harmonics][newton][crossorder]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};

  CrossOrderNonlinearSource r;
  r.bus = 2;
  r.i_src[5] = Cx(1.0, 0.0);
  r.i_src[7] = Cx(1.0, 0.0);
  r.mixing.push_back({5, 7, 7, Cx(0.05, 0.0)});  // I_5 -= 0.05 * V_7 * V_7

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.dc_orders = {};
  opt.newton_tol = 1e-12;

  HPFNewtonResult nr = solve_harmonic_power_flow_newton_coupled(sys, {r}, {}, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.converged);
  CHECK(nr.final_residual.at(0) < 1e-10);

  // Z_th(h) = j*h*0.3.  V7 = i_src7 * Zth7 (unaffected) ; V5 = (i_src5 - k V7^2) Zth5.
  const Cx Zth5(0.0, 1.5), Zth7(0.0, 2.1);
  const Cx V7exp = Cx(1.0, 0.0) * Zth7;
  const Cx V5exp = (Cx(1.0, 0.0) - Cx(0.05, 0.0) * V7exp * V7exp) * Zth5;
  const Cx V5 = vnewton(nr, 2, 5), V7 = vnewton(nr, 2, 7);
  CHECK_THAT(V7.real(), WithinAbs(V7exp.real(), 1e-6));
  CHECK_THAT(V7.imag(), WithinAbs(V7exp.imag(), 1e-6));
  CHECK_THAT(V5.real(), WithinAbs(V5exp.real(), 1e-6));
  CHECK_THAT(V5.imag(), WithinAbs(V5exp.imag(), 1e-6));
  // Cross-order coupling shifted the 5th away from its decoupled value j1.5.
  CHECK(std::abs(V5 - Cx(0.0, 1.5)) > 0.1);
}

TEST_CASE("Newton coupled solves mutual frequency mixing (V5*V7 in both)",
          "[harmonics][newton][crossorder]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};

  CrossOrderNonlinearSource r;
  r.bus = 2;
  r.i_src[5] = Cx(1.0, 0.0);
  r.i_src[7] = Cx(1.0, 0.0);
  const Cx k(0.05, 0.0);
  r.mixing.push_back({5, 5, 7, k});  // I_5 -= k V_5 V_7
  r.mixing.push_back({7, 5, 7, k});  // I_7 -= k V_5 V_7

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.dc_orders = {};
  opt.newton_tol = 1e-12;

  HPFNewtonResult nr = solve_harmonic_power_flow_newton_coupled(sys, {r}, {}, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.converged);

  // Plug back into the reduced equations:  A_h V_h - i_src + k V_5 V_7 = 0.
  const Cx A5 = Cx(1.0, 0.0) / Cx(0.0, 1.5), A7 = Cx(1.0, 0.0) / Cx(0.0, 2.1);
  const Cx V5 = vnewton(nr, 2, 5), V7 = vnewton(nr, 2, 7);
  const Cx r5 = A5 * V5 - Cx(1.0, 0.0) + k * V5 * V7;
  const Cx r7 = A7 * V7 - Cx(1.0, 0.0) + k * V5 * V7;
  CHECK_THAT(std::abs(r5), WithinAbs(0.0, 1e-6));
  CHECK_THAT(std::abs(r7), WithinAbs(0.0, 1e-6));
  // Both orders shifted from their decoupled values by the mutual coupling.
  CHECK(std::abs(V5 - Cx(0.0, 1.5)) > 0.05);
  CHECK(std::abs(V7 - Cx(0.0, 2.1)) > 0.05);
}

// ===========================================================================
// Frequency scan / resonance analysis
// ===========================================================================
static int freq_idx(const std::vector<double>& freqs, double target) {
  for (size_t i = 0; i < freqs.size(); ++i)
    if (std::abs(freqs[i] - target) < 1e-6) return static_cast<int>(i);
  return -1;
}

TEST_CASE("Frequency scan reproduces the analytic driving-point impedance",
          "[harmonics][scan]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};

  FrequencyScanOptions sopt;
  sopt.f_start = 1.0;
  sopt.f_end = 15.0;
  sopt.f_step = 1.0;
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;

  FrequencyScanResult r = frequency_scan(sys, sopt, opt);
  REQUIRE(r.ok);
  const int i5 = freq_idx(r.freqs, 5.0), i7 = freq_idx(r.freqs, 7.0);
  REQUIRE(i5 >= 0);
  REQUIRE(i7 >= 0);
  // Z_dp(bus2) = z_line + z_src = j*h*(0.1 + 0.2) = j*h*0.3 -> |Z(5)| = 1.5
  CHECK_THAT(r.z_mag.at(2)[i5], WithinAbs(1.5, 1e-3));
  // Z_dp(bus1) ~ z_src = j*h*0.2 -> |Z(5)| = 1.0
  CHECK_THAT(r.z_mag.at(1)[i5], WithinAbs(1.0, 1e-3));
  // Impedance scales linearly with frequency: |Z(7)|/|Z(5)| = 7/5.
  CHECK_THAT(r.z_mag.at(2)[i7] / r.z_mag.at(2)[i5], WithinRel(7.0 / 5.0, 1e-4));
}

TEST_CASE("Frequency scan detects a parallel resonance from a shunt capacitor",
          "[harmonics][scan][resonance]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus b1 = ac_bus(1, BusType::SLACK), b2 = ac_bus(2, BusType::PQ);
  b2.bs_mvar = 6.8;  // shunt capacitor -> b_cap = 0.068 pu
  sys.ac.buses = {b1, b2};
  sys.ac.branches = {ac_line(1, 1, 2, 0.01, 0.1)};  // small R -> finite peak

  FrequencyScanOptions sopt;
  sopt.f_start = 1.0;
  sopt.f_end = 15.0;
  sopt.f_step = 0.05;
  sopt.buses = {2};
  sopt.resonance_min_pu = 2.0;  // ignore small ripples
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;

  FrequencyScanResult r = frequency_scan(sys, sopt, opt);
  REQUIRE(r.ok);
  // Expected parallel resonance: h_res = 1/sqrt(X_tot * b_cap) = 1/sqrt(0.3*0.068) = 7.0
  const HarmonicResonance* peak = nullptr;
  for (const auto& res : r.resonances)
    if (res.parallel && (!peak || res.z_mag > peak->z_mag)) peak = &res;
  REQUIRE(peak != nullptr);
  CHECK_THAT(peak->freq_order, WithinAbs(7.0, 0.3));
  // The resonant impedance dwarfs a non-resonant point (e.g. h = 3).
  const int i3 = freq_idx(r.freqs, 3.0);
  REQUIRE(i3 >= 0);
  CHECK(peak->z_mag > 10.0 * r.z_mag.at(2)[i3]);
}

TEST_CASE("Per-sequence frequency scan separates positive and zero sequence",
          "[harmonics][scan][3ph]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};  // x1 = 0.1, x0 = 0.3

  FrequencyScanOptions sopt;
  sopt.f_start = 1.0;
  sopt.f_end = 13.0;
  sopt.f_step = 1.0;
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;

  SequenceScanResult r = sequence_frequency_scan(sys, 2, sopt, opt);
  REQUIRE(r.ok);
  const int i5 = freq_idx(r.freqs, 5.0);
  REQUIRE(i5 >= 0);
  // Z1_dp = z_src + z1_line = j*h*(0.2 + 0.1) -> |Z1(5)| = 1.5  (= Z2)
  // Z0_dp = z_src + z0_line = j*h*(0.2 + 0.3) -> |Z0(5)| = 2.5
  CHECK_THAT(r.z1_mag[i5], WithinAbs(1.5, 1e-3));
  CHECK_THAT(r.z2_mag[i5], WithinAbs(1.5, 1e-3));
  CHECK_THAT(r.z0_mag[i5], WithinAbs(2.5, 1e-3));
  CHECK_FALSE(r.summary().empty());
}

// ===========================================================================
// Three-phase Newton variants: non-holomorphic + cross-order
// ===========================================================================
// Balanced sequence rotation factor for phase (0/1/2) at a given sequence.
static Cx seq_rot(int seq, int phase) {
  const Cx a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Cx a2 = std::polar(1.0, 4.0 * kPi / 3.0);
  if (phase == 0) return Cx(1, 0);
  if (seq == 1) return (phase == 1) ? a2 : a;   // positive: b=a², c=a
  if (seq == 2) return (phase == 1) ? a : a2;   // negative: b=a, c=a²
  return Cx(1, 0);                              // zero
}

TEST_CASE("Newton 3-phase real-form reduces to linear without CP loads",
          "[harmonics][newton][3ph][real]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};
  ThreePhaseHarmonicInputs lin{.sources = {src}, .nics = {}};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};

  HPF3phResult linres = solve_harmonic_power_flow_3ph(sys, lin, opt);
  HPF3phResult nr = solve_harmonic_power_flow_3ph_newton_real(sys, {}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.newton_converged);
  CHECK(nr.newton_iterations.at(5) <= 1);
  for (int ph = 0; ph < 3; ++ph) {
    const Cx vn = vph(nr, 2, 5, ph), vl = vph(linres, 2, 5, ph);
    CHECK_THAT(vn.real(), WithinAbs(vl.real(), 1e-9));
    CHECK_THAT(vn.imag(), WithinAbs(vl.imag(), 1e-9));
  }
}

TEST_CASE("Newton 3-phase real-form converges to per-phase constant-power roots",
          "[harmonics][newton][3ph][real]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.1)};  // z0 = z1 -> phases decouple

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}};
  ThreePhaseHarmonicInputs lin{.sources = {src}, .nics = {}};

  ThreePhaseConstantPowerLoad cp;
  cp.bus = 2;
  cp.s_set[5] = {Cx(0.2, 0.0), Cx(0.1, 0.0), Cx(0.0, 0.0)};  // per-phase power

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.newton_tol = 1e-12;

  HPF3phResult nr = solve_harmonic_power_flow_3ph_newton_real(sys, {cp}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.newton_converged);

  const Cx A = Cx(1.0, 0.0) / Cx(0.0, 1.5);  // Z_th(5) = j1.5 per phase
  auto cp_root = [&](Cx S, Cx Isrc) {
    if (std::abs(Isrc) < 1e-15) return Cx(0.0, 0.0);
    double aa = std::norm(A), bb = 2.0 * std::real(A * S) - std::norm(Isrc), cc = std::norm(S);
    double disc = std::sqrt(std::max(bb * bb - 4.0 * aa * cc, 0.0));
    double m1 = (-bb + disc) / (2.0 * aa), m2 = (-bb - disc) / (2.0 * aa);
    double mlin = std::norm(Isrc / A);
    double m = (std::abs(m1 - mlin) < std::abs(m2 - mlin)) ? m1 : m2;
    return std::conj((A * m + std::conj(S)) / Isrc);
  };
  const double s_ph[3] = {0.2, 0.1, 0.0};
  for (int ph = 0; ph < 3; ++ph) {
    const Cx Isrc = seq_rot(2, ph);  // 5th = negative sequence
    const Cx exp = cp_root(Cx(s_ph[ph], 0.0), Isrc);
    const Cx vn = vph(nr, 2, 5, ph);
    CHECK_THAT(vn.real(), WithinAbs(exp.real(), 1e-6));
    CHECK_THAT(vn.imag(), WithinAbs(exp.imag(), 1e-6));
  }
}

TEST_CASE("Newton 3-phase coupled reduces to linear without mixing",
          "[harmonics][newton][3ph][crossorder]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}, {7, 100.0, 0.0}};
  ThreePhaseHarmonicInputs lin{.sources = {src}, .nics = {}};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};

  HPF3phResult linres = solve_harmonic_power_flow_3ph(sys, lin, opt);
  HPF3phResult nr = solve_harmonic_power_flow_3ph_newton_coupled(sys, {}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.newton_converged);
  for (int h : {5, 7})
    for (int ph = 0; ph < 3; ++ph) {
      const Cx vn = vph(nr, 2, h, ph), vl = vph(linres, 2, h, ph);
      CHECK_THAT(vn.real(), WithinAbs(vl.real(), 1e-9));
      CHECK_THAT(vn.imag(), WithinAbs(vl.imag(), 1e-9));
    }
}

TEST_CASE("Newton 3-phase coupled solves per-phase frequency mixing (7th^2 -> 5th)",
          "[harmonics][newton][3ph][crossorder]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.1)};  // z0 = z1 -> phases decouple

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = true;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 100.0, 0.0}, {7, 100.0, 0.0}};
  ThreePhaseHarmonicInputs lin{.sources = {src}, .nics = {}};

  ThreePhaseCrossOrderSource r;
  r.bus = 2;
  const Cx k(0.05, 0.0);
  r.mixing.push_back({5, 7, 7, -1, k});  // all phases: I_5 -= k V_7^2

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.newton_tol = 1e-12;

  HPF3phResult nr = solve_harmonic_power_flow_3ph_newton_coupled(sys, {r}, lin, opt);
  REQUIRE(nr.ok);
  REQUIRE(nr.newton_converged);

  const Cx Zth5(0.0, 1.5), Zth7(0.0, 2.1);
  for (int ph = 0; ph < 3; ++ph) {
    const Cx Isrc5 = seq_rot(2, ph), Isrc7 = seq_rot(1, ph);  // 5th neg, 7th pos
    const Cx V7exp = Isrc7 * Zth7;
    const Cx V5exp = (Isrc5 - k * V7exp * V7exp) * Zth5;
    const Cx V5 = vph(nr, 2, 5, ph), V7 = vph(nr, 2, 7, ph);
    CHECK_THAT(V7.real(), WithinAbs(V7exp.real(), 1e-6));
    CHECK_THAT(V7.imag(), WithinAbs(V7exp.imag(), 1e-6));
    CHECK_THAT(V5.real(), WithinAbs(V5exp.real(), 1e-6));
    CHECK_THAT(V5.imag(), WithinAbs(V5exp.imag(), 1e-6));
  }
}

// ===========================================================================
// Harmonic metrics: losses, K-factor, current THD / TDD
// ===========================================================================
TEST_CASE("Harmonic metrics compute K-factor, THD/TDD and losses", "[harmonics][metrics]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus b1 = ac_bus(1, BusType::SLACK);
  ACBus b2 = ac_bus(2, BusType::PQ);
  b2.vm_pu = 0.9;  // fundamental drop -> branch carries I_1 = 1.0 pu (ys = 10)
  sys.ac.buses = {b1, b2};
  sys.ac.branches = {ac_line(1, 1, 2, 0.1, 0.0)};  // purely resistive, ys = 10 at all orders

  HarmonicCurrentSource src;
  src.bus = 2;
  src.i_base_pu = 1.0;
  src.spectrum = {{5, 20.0, 0.0}, {7, 10.0, 0.0}};  // I_5 = 0.2, I_7 = 0.1
  HarmonicStudyInputs in{.sources = {src}, .nics = {}};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.dc_orders = {};

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);

  HarmonicMetricsOptions mopt;
  mopt.i_demand_pu = 2.0;
  HarmonicMetricsResult m = harmonic_metrics(sys, r, mopt, opt);
  REQUIRE(m.ac_branches.size() == 1);
  const auto& bm = m.ac_branches[0];

  // I_1 = 1.0, I_5 = 0.2, I_7 = 0.1
  CHECK_THAT(bm.i_fund_pu, WithinAbs(1.0, 1e-3));
  // K = (1·1 + 25·0.04 + 49·0.01)/(1+0.04+0.01) = 2.49/1.05 = 2.3714
  CHECK_THAT(bm.k_factor, WithinAbs(2.3714, 1e-2));
  // THD_I = sqrt(0.05)/1 = 22.36 %
  CHECK_THAT(bm.thd_i_pct, WithinAbs(22.36, 0.1));
  // TDD (I_L = 2.0) = sqrt(0.05)/2 = 11.18 %
  CHECK_THAT(bm.tdd_pct, WithinAbs(11.18, 0.1));
  // Loss = (1 + 0.04 + 0.01)·R, R = 0.1 -> 0.105 ; harmonic part = 0.05·0.1 = 0.005
  CHECK_THAT(bm.p_loss_pu, WithinAbs(0.105, 2e-3));
  CHECK_THAT(bm.p_loss_harmonic_pu, WithinAbs(0.005, 2e-3));
  CHECK(bm.p_loss_harmonic_pu < bm.p_loss_pu);
  CHECK_THAT(m.total_loss_pu, WithinAbs(0.105, 2e-3));
  CHECK(m.max_k_factor > 2.0);
  CHECK_FALSE(m.summary().empty());

  // Skin effect raises the harmonic I²R losses (current-based K-factor unchanged).
  HPFOptions opt_skin = opt;
  opt_skin.skin_effect = SkinEffectModel::SqrtOrder;
  HPFResult r2 = solve_harmonic_power_flow(sys, in, opt_skin);
  HarmonicMetricsResult m2 = harmonic_metrics(sys, r2, mopt, opt_skin);
  CHECK(m2.ac_branches[0].p_loss_harmonic_pu > bm.p_loss_harmonic_pu);
  CHECK_THAT(m2.ac_branches[0].k_factor, WithinAbs(bm.k_factor, 1e-2));
}

// ===========================================================================
// Hybrid AC + DC Newton with bilinear NIC cross-domain coupling
// ===========================================================================
namespace {
HybridPowerSystem make_hybrid_newton_bed() {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.dc.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};
  sys.dc.buses = {dc_bus(10, DCBusType::DC_V), dc_bus(11, DCBusType::DC_P)};
  sys.dc.branches = {dc_line(1, 10, 11, 0.05)};
  return sys;
}
Cx vac_hn(const HPFHybridNewtonResult& r, int bus, int order) {
  for (const auto& b : r.ac_bus_results)
    if (b.bus == bus) { auto it = b.v_by_order.find(order); if (it != b.v_by_order.end()) return it->second; }
  return Cx(0, 0);
}
Cx vdc_hn(const HPFHybridNewtonResult& r, int bus, int order) {
  for (const auto& b : r.dc_bus_results)
    if (b.bus == bus) { auto it = b.v_by_order.find(order); if (it != b.v_by_order.end()) return it->second; }
  return Cx(0, 0);
}
}  // namespace

TEST_CASE("Hybrid AC/DC Newton reduces to independent linear solves",
          "[harmonics][newton][hybrid]") {
  HybridPowerSystem sys = make_hybrid_newton_bed();
  HybridNewtonInputs in;
  HarmonicCurrentSource acs;
  acs.bus = 2; acs.i_base_pu = 1.0; acs.spectrum = {{5, 100.0, 0.0}};
  HarmonicCurrentSource dcs;
  dcs.bus = 11; dcs.is_dc = true; dcs.i_base_pu = 1.0; dcs.spectrum = {{6, 100.0, 0.0}};
  in.sources = {acs, dcs};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5};
  opt.dc_orders = {6};
  opt.dc_source_impedance_pu = 0.01;

  HPFHybridNewtonResult r = solve_harmonic_power_flow_hybrid_newton(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.converged);
  CHECK(r.iterations <= 1);
  CHECK_THAT(std::abs(vac_hn(r, 2, 5)), WithinAbs(1.5, 1e-4));   // j1.5
  CHECK_THAT(std::abs(vdc_hn(r, 11, 6)), WithinAbs(0.06, 1e-4));  // 1.0 * 0.06
}

TEST_CASE("Hybrid AC/DC Newton: bilinear converter mixing drives DC ripple",
          "[harmonics][newton][hybrid]") {
  HybridPowerSystem sys = make_hybrid_newton_bed();
  HybridNewtonInputs in;
  HarmonicCurrentSource acs;
  acs.bus = 2; acs.i_base_pu = 1.0; acs.spectrum = {{5, 100.0, 0.0}, {7, 100.0, 0.0}};
  in.sources = {acs};
  // I_dc(6,bus11) -= 0.1 * V_ac(5,bus2) * V_ac(7,bus2)  (converter mixes 5th & 7th)
  HybridBilinearTerm t;
  t.out_is_dc = true; t.out_order = 6; t.out_bus = 11;
  t.a_is_dc = false; t.a_order = 5; t.a_bus = 2;
  t.b_is_dc = false; t.b_order = 7; t.b_bus = 2;
  t.coeff = Cx(0.1, 0.0);
  in.bilinear = {t};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.dc_orders = {6};
  opt.dc_source_impedance_pu = 0.01;
  opt.newton_tol = 1e-12;

  HPFHybridNewtonResult r = solve_harmonic_power_flow_hybrid_newton(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.converged);
  // AC unaffected (no DC->AC feedback): V_ac5 = j1.5, V_ac7 = j2.1
  const Cx V5 = vac_hn(r, 2, 5), V7 = vac_hn(r, 2, 7);
  CHECK_THAT(std::abs(V5), WithinAbs(1.5, 1e-4));
  CHECK_THAT(std::abs(V7), WithinAbs(2.1, 1e-4));
  // V_dc(6) = -coeff * V5 * V7 * Z_dc = -0.1*(j1.5)(j2.1)*0.06 = 0.0189 (real)
  const Cx Vdc6 = vdc_hn(r, 11, 6);
  CHECK_THAT(Vdc6.real(), WithinAbs(0.0189, 1e-5));
  CHECK_THAT(Vdc6.imag(), WithinAbs(0.0, 1e-5));

  // With coeff = 0 the DC ripple vanishes.
  in.bilinear[0].coeff = Cx(0.0, 0.0);
  HPFHybridNewtonResult r0 = solve_harmonic_power_flow_hybrid_newton(sys, in, opt);
  CHECK(std::abs(vdc_hn(r0, 11, 6)) < 1e-6);
}

TEST_CASE("Hybrid AC/DC Newton: mutual cross-domain coupling residual",
          "[harmonics][newton][hybrid]") {
  HybridPowerSystem sys = make_hybrid_newton_bed();
  HybridNewtonInputs in;
  HarmonicCurrentSource acs;
  acs.bus = 2; acs.i_base_pu = 1.0; acs.spectrum = {{5, 100.0, 0.0}, {7, 100.0, 0.0}};
  in.sources = {acs};
  const Cx g(0.1, 0.0), g2(0.3, 0.0);
  HybridBilinearTerm t1;  // I_dc(6) -= g * V_ac5 * V_ac7
  t1.out_is_dc = true; t1.out_order = 6; t1.out_bus = 11;
  t1.a_is_dc = false; t1.a_order = 5; t1.a_bus = 2;
  t1.b_is_dc = false; t1.b_order = 7; t1.b_bus = 2;
  t1.coeff = g;
  HybridBilinearTerm t2;  // I_ac(5) -= g2 * V_ac7 * V_dc6  (DC ripple modulates AC 5th)
  t2.out_is_dc = false; t2.out_order = 5; t2.out_bus = 2;
  t2.a_is_dc = false; t2.a_order = 7; t2.a_bus = 2;
  t2.b_is_dc = true; t2.b_order = 6; t2.b_bus = 11;
  t2.coeff = g2;
  in.bilinear = {t1, t2};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = {5, 7};
  opt.dc_orders = {6};
  opt.dc_source_impedance_pu = 0.01;
  opt.newton_tol = 1e-12;

  HPFHybridNewtonResult r = solve_harmonic_power_flow_hybrid_newton(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.converged);

  // Reduced equations:  A5 V5 - I5 + g2 V7 Vdc6 = 0 ; A7 V7 - I7 = 0 ;
  //                     Adc Vdc6 + g V5 V7 = 0.
  const Cx A5 = Cx(1.0, 0.0) / Cx(0.0, 1.5), A7 = Cx(1.0, 0.0) / Cx(0.0, 2.1);
  const Cx Adc = Cx(1.0, 0.0) / Cx(0.06, 0.0);
  const Cx V5 = vac_hn(r, 2, 5), V7 = vac_hn(r, 2, 7), Vdc6 = vdc_hn(r, 11, 6);
  CHECK_THAT(std::abs(A5 * V5 - Cx(1.0, 0.0) + g2 * V7 * Vdc6), WithinAbs(0.0, 1e-6));
  CHECK_THAT(std::abs(A7 * V7 - Cx(1.0, 0.0)), WithinAbs(0.0, 1e-6));
  CHECK_THAT(std::abs(Adc * Vdc6 + g * V5 * V7), WithinAbs(0.0, 1e-6));
  CHECK_FALSE(r.summary().empty());
}

// ===========================================================================
// Coverage backfill: paths that were previously untested
// ===========================================================================

// Helper: look up a DC branch-flow current magnitude at a given order.
static double dc_branch_i(const HPFResult& r, int from, int to, int order) {
  for (const auto& bf : r.dc_branch_flows)
    if (bf.from_bus == from && bf.to_bus == to) {
      auto it = bf.i_by_order.find(order);
      if (it != bf.i_by_order.end()) return it->second;
    }
  return -1.0;
}

// Pins the DC branch-flow ripple-inductance fix: the reported branch current must
// use the SAME order-dependent impedance Z(r)=r+j*r*x_pu as the network solve.
// In a radial DC feeder all the injected ripple current returns through the single
// branch, so KCL fixes |I_branch(6)| = |I_inj(6)| = 1.0 EXACTLY — regardless of
// the branch reactance.  The pre-fix (resistance-only) formula gives 2.6 here.
TEST_CASE("HPF DC branch flow respects ripple inductance (radial KCL)",
          "[harmonics][dc][branchflow]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.dc.base_mva = 100.0;
  sys.dc.buses = {dc_bus(1, DCBusType::DC_V), dc_bus(2, DCBusType::DC_P)};
  sys.dc.branches = {dc_line(1, 1, 2, 0.05)};

  HarmonicCurrentSource src;
  src.bus = 2; src.is_dc = true; src.i_base_pu = 1.0;
  src.spectrum = {{6, 100.0, 0.0}};

  HarmonicStudyInputs in{.sources = {src}, .nics = {}};
  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.ac_orders = {};
  opt.dc_orders = {6};
  opt.dc_source_impedance_pu = 0.01;
  opt.dc_ripple_model.branch_x_pu[1] = 0.02;   // Z(6) = 0.05 + j*6*0.02
  opt.compute_branch_flows = true;

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);
  REQUIRE(r.ok);
  REQUIRE(r.dc_order_solved.at(6));
  // Radial branch carries the full injected ripple current: exactly 1.0 pu.
  CHECK_THAT(dc_branch_i(r, 1, 2, 6), WithinAbs(1.0, 1e-6));
}

// Exercises the three-phase load-as-shunt-impedance path (build_3ph_ac_ybus
// stamp_load), which no prior 3-phase test enabled.  For a balanced positive-
// sequence excitation the phase-domain solution is a pure positive-sequence set,
// so the A-phase voltage matches a two-node positive-sequence hand solution, and
// adding the load shunt strictly attenuates the bus harmonic voltage.
TEST_CASE("HPF 3-phase load impedance attenuates harmonic voltage (analytic)",
          "[harmonics][3ph][load]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 100.0;
  sys.buses = {tp_bus(1, BusType::SLACK), tp_bus(2, BusType::PQ)};
  sys.lines = {tp_line(1, 1, 2, 0.1, 0.3)};   // x1 = 0.1 (lossless)

  ThreePhaseLoad ld;
  ld.index = 1; ld.bus = 2; ld.in_service = true;
  ld.phase_mask = PhaseMask::abc();
  ld.p_a_mw = 20.0; ld.p_b_mw = 20.0; ld.p_c_mw = 20.0;  // 0.2 pu/phase, q = 0
  sys.loads = {ld};

  ThreePhaseHarmonicSource src;
  src.bus = 2; src.balanced = true; src.i_base_pu = 1.0;
  src.spectrum = {{7, 100.0, 0.0}};            // 7th: positive sequence
  ThreePhaseHarmonicInputs in;
  in.sources = {src};

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.ac_orders = {7};

  opt.include_load_impedance = false;
  HPF3phResult r_off = solve_harmonic_power_flow_3ph(sys, in, opt);
  REQUIRE(r_off.ok);
  REQUIRE(r_off.ac_order_solved.at(7));

  opt.include_load_impedance = true;
  HPF3phResult r_on = solve_harmonic_power_flow_3ph(sys, in, opt);
  REQUIRE(r_on.ok);
  REQUIRE(r_on.ac_order_solved.at(7));

  // Two-node positive-sequence closed form with the load shunt at bus 2:
  //   y_src = 1/(j*7*0.2), y_line = 1/(j*7*0.1), y_load = 0.2 (q = 0).
  const Cx I(1.0, 0.0);
  const Cx ysrc  = I / Cx(0.0, 7.0 * 0.2);
  const Cx yline = I / Cx(0.0, 7.0 * 0.1);
  const Cx yload(0.2, 0.0);
  const Cx Y11 = ysrc + yline, Y22 = yline + yload, Y12 = -yline;
  const Cx det = Y11 * Y22 - Y12 * Y12;
  const Cx v2_expected = I * Y11 / det;

  // Balanced across phases (positive-sequence: B lags, C leads by 120 deg).
  const Cx va = vph(r_on, 2, 7, 0), vb = vph(r_on, 2, 7, 1), vc = vph(r_on, 2, 7, 2);
  CHECK_THAT(std::abs(vb), WithinRel(std::abs(va), 1e-6));
  CHECK_THAT(std::abs(vc), WithinRel(std::abs(va), 1e-6));
  CHECK_THAT(angdiff_deg(vb, va), WithinAbs(-120.0, 1e-3));
  CHECK_THAT(angdiff_deg(vc, va), WithinAbs(120.0, 1e-3));
  // A-phase magnitude matches the positive-sequence hand solution.
  CHECK_THAT(std::abs(va), WithinRel(std::abs(v2_expected), 1e-4));
  // The load shunt provides an extra path to ground -> strictly lower voltage.
  CHECK(std::abs(vph(r_on, 2, 7, 0)) < std::abs(vph(r_off, 2, 7, 0)));
}

// Drives the NIC operating point from a REAL base power flow (the vsc_transfers
// extraction path), which every other NIC test bypasses with run_base_power_flow
// = false and hand-fed setpoints.  A converged base PF supplies the converter's
// P_dc, so the lossless guess must NOT be used and the auto-NIC must inject
// characteristic harmonics that raise the AC voltage THD above zero.
TEST_CASE("HPF NIC operating point derived from a converged base power flow",
          "[harmonics][nic][basepf]") {
  HybridPowerSystem sys = io::build_ieee14_acdc();
  REQUIRE_FALSE(sys.vsc_converters.empty());   // auto-NIC needs converters

  HPFOptions opt;                  // defaults: run_base_power_flow + auto_nic_from_vscs
  HPFResult r = solve_harmonic_power_flow(sys, opt);
  REQUIRE(r.ok);
  REQUIRE(r.base_pf_converged);
  REQUIRE_FALSE(r.ac_bus_results.empty());
  // Base PF provided the converter transfer, so no lossless fallback was taken.
  CHECK(r.message.find("lossless") == std::string::npos);
  // Converters inject characteristic harmonics -> non-zero AC voltage distortion.
  CHECK(r.max_ac_thd_pct > 0.0);
}

// Exercises the single-phase NIC Norton output admittance (y_out_ac) stamping,
// which the existing NIC test leaves at zero.  A pure conductance g = 0.5 at the
// AC port turns the radial node into a loaded node; the closed-form two-node
// solution gives |V_ac(5)| = 1.2 pu (vs 1.5 with an ideal current source).
TEST_CASE("HPF NIC Norton output admittance loads the AC port",
          "[harmonics][nic][yout]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.dc.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};   // x = 0.1
  sys.dc.buses = {dc_bus(10, DCBusType::DC_V), dc_bus(11, DCBusType::DC_P)};
  sys.dc.branches = {dc_line(1, 10, 11, 0.05)};

  VSCConverter v;
  v.index = 0; v.bus_ac = 2; v.bus_dc = 11;
  v.control_mode = ConverterMode::PQ_MODE; v.in_service = true;
  sys.vsc_converters = {v};

  auto run = [&](Cx y_out) {
    HarmonicNIC nic;
    nic.vsc_index = 0;
    nic.ac_port = PortBehavior::GridFollowing;
    nic.dc_port = PortBehavior::GridFollowing;
    nic.ac_spectrum = {{5, 100.0, 0.0}};
    nic.s_ac_p_mw = 100.0;            // -> |I_ac1| = 1.0 at V_ac1 = 1.0
    nic.s_ac_q_mvar = 0.0;
    nic.p_dc_mw = -100.0;
    nic.y_out_ac = y_out;
    HarmonicStudyInputs in{.sources = {}, .nics = {nic}};
    HPFOptions opt;
    opt.run_base_power_flow = false;
    opt.include_load_impedance = false;
    opt.ac_orders = {5};
    opt.dc_orders = {};
    opt.auto_nic_from_vscs = false;  // only the explicit NIC
    return solve_harmonic_power_flow(sys, in, opt);
  };

  HPFResult r_ideal = run(Cx(0.0, 0.0));
  REQUIRE(r_ideal.ok);
  CHECK_THAT(std::abs(vbus(r_ideal, 2, 5)), WithinAbs(1.5, 1e-5));  // radial sum

  HPFResult r_yout = run(Cx(0.5, 0.0));  // g = 0.5 (b = 0 -> no order scaling)
  REQUIRE(r_yout.ok);
  CHECK_THAT(std::abs(vbus(r_yout, 2, 5)), WithinAbs(1.2, 1e-5));
  CHECK(std::abs(vbus(r_yout, 2, 5)) < std::abs(vbus(r_ideal, 2, 5)));
}

// THD is referenced to the fundamental magnitude, so a depressed fundamental
// voltage inflates THD proportionally.  With run_base_power_flow = false the
// stored bus voltage IS the fundamental reference, so halving it doubles THD.
TEST_CASE("HPF THD scales inversely with a depressed fundamental voltage",
          "[harmonics][thd]") {
  auto run = [&](double v2_fund) {
    HybridPowerSystem sys;
    sys.base_mva = 100.0;
    sys.ac.base_mva = 100.0;
    sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ)};
    sys.ac.buses[1].vm_pu = v2_fund;   // stored fundamental at bus 2
    sys.ac.branches = {ac_line(1, 1, 2, 0.0, 0.1)};  // x = 0.1
    HarmonicCurrentSource src;
    src.bus = 2; src.i_base_pu = 1.0; src.spectrum = {{5, 100.0, 0.0}};
    HarmonicStudyInputs in{.sources = {src}, .nics = {}};
    HPFOptions opt;
    opt.run_base_power_flow = false;
    opt.include_load_impedance = false;
    opt.ac_orders = {5};
    opt.dc_orders = {};
    return solve_harmonic_power_flow(sys, in, opt);
  };

  // |V5| = I*(z_src + z_line) = 1.0*(j1.0 + j0.5) = 1.5 pu, independent of V_fund.
  HPFResult r1 = run(1.0);
  REQUIRE(r1.ok);
  CHECK_THAT(thd_at(r1, 2), WithinAbs(150.0, 1e-3));   // 1.5 / 1.0
  HPFResult rhalf = run(0.5);
  REQUIRE(rhalf.ok);
  CHECK_THAT(thd_at(rhalf, 2), WithinAbs(300.0, 1e-3)); // 1.5 / 0.5
  CHECK(std::isfinite(thd_at(rhalf, 2)));
}

TEST_CASE("HPF projects rich topology and broadcasts merged-bus observables",
          "[harmonics][projection][attribution]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {ac_bus(10, BusType::SLACK), ac_bus(20, BusType::PQ),
                  ac_bus(30, BusType::PQ)};
  sys.ac.branches = {ac_line(1, 10, 20, 0.01, 0.1)};
  CircuitBreaker breaker;
  breaker.index = 34;
  breaker.bus_from = 20;
  breaker.bus_to = 30;
  breaker.closed = true;
  breaker.z_ohm = 0.0;
  sys.ac.circuit_breakers.push_back(breaker);

  HarmonicCurrentSource source;
  source.bus = 30;
  source.i_base_pu = 1.0;
  source.spectrum = {{5, 100.0, 0.0}};
  HarmonicStudyInputs inputs{.sources = {source}, .nics = {}};
  HPFOptions options;
  options.run_base_power_flow = false;
  options.include_load_impedance = false;
  options.ac_orders = {5};
  options.dc_orders = {};

  const auto result = solve_harmonic_power_flow(sys, inputs, options);
  REQUIRE(result.ok);
  REQUIRE(result.ac_bus_results.size() == 3);
  CHECK_THAT(std::abs(vbus(result, 20, 5)), WithinAbs(std::abs(vbus(result, 30, 5)), 1e-12));
  CHECK(std::abs(vbus(result, 30, 5)) > 0.0);
}








