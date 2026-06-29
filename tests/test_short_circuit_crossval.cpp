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
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/model/system.hpp"
#include "hacdcpf/model/network_utils.hpp"
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Cx = std::complex<double>;
using json = nlohmann::json;

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

static std::string read_text_file(const std::string& path) {
  std::ifstream in(path);
  REQUIRE(in.good());
  return std::string(std::istreambuf_iterator<char>(in),
                     std::istreambuf_iterator<char>());
}

static double check_value(const json& j, const std::string& key, double fallback = 0.0) {
  return j.contains(key) && j[key].is_number() ? j[key].get<double>() : fallback;
}

static double check_abs_tolerance(const json& meta) {
  if (!meta.contains("tolerance")) return 1e-6;
  return meta["tolerance"].value("absolute_ka", 1e-6);
}

static double check_rel_tolerance(const json& meta) {
  if (!meta.contains("tolerance")) return 1e-6;
  return meta["tolerance"].value("relative", 1e-6);
}

static const SCDetailedBusResult& fault_row(const SCDetailedResult& r) {
  for (const auto& row : r.bus_results)
    if (row.bus_id == r.fault_bus_id) return row;
  FAIL("Fault bus row not found");
  return r.bus_results.front();
}

static const SCDetailedBranchResult* branch_row(const SCDetailedResult& r,
                                                int branch_index) {
  for (const auto& row : r.branch_results)
    if (row.branch_index == branch_index) return &row;
  return nullptr;
}

static double sc_result_field(const SCDetailedBusResult& row,
                              const std::string& field) {
  if (field == "ikss_ka") return row.ikss_ka;
  if (field == "ikss_1_ka") return row.ikss_1_ka;
  if (field == "ikss_2_ka") return row.ikss_2_ka;
  if (field == "ikss_gen_contrib_ka") return row.ikss_gen_contrib_ka;
  if (field == "ikss_motor_contrib_ka") return row.ikss_motor_contrib_ka;
  if (field == "ikss_load_contrib_ka") return row.ikss_load_contrib_ka;
  if (field == "ikss_sgen_contrib_ka") return row.ikss_sgen_contrib_ka;
  if (field == "ikss_extgrid_contrib_ka") return row.ikss_extgrid_contrib_ka;
  if (field == "ikss_converter_contrib_ka") return row.ikss_converter_contrib_ka;
  if (field == "ikss_no_motor_ka") return row.ikss_no_motor_ka;
  if (field == "ip_ka") return row.ip_ka;
  if (field == "ib_ka") return row.ib_ka;
  if (field == "ik_ka") return row.ik_ka;
  if (field == "ith_ka") return row.ith_ka;
  if (field == "v_remaining_pu") return row.v_remaining_pu;
  FAIL("Unknown short-circuit result field: " << field);
  return 0.0;
}

static int count_branch_origins(const HybridPowerSystem& projected,
                                BranchOriginType type) {
  if (!projected.branch_expand_map) return 0;
  int count = 0;
  for (const auto& entry : projected.branch_expand_map->entries) {
    if (entry.origin_type == type) ++count;
  }
  return count;
}

static FaultType fault_type_from_name(const std::string& name) {
  if (name == "SinglePhaseGround") return FaultType::SinglePhaseGround;
  if (name == "TwoPhase") return FaultType::TwoPhase;
  if (name == "TwoPhaseGround") return FaultType::TwoPhaseGround;
  return FaultType::ThreePhase;
}

static SCCalcType calc_type_from_name(const std::string& name) {
  return name == "Min" ? SCCalcType::Min : SCCalcType::Max;
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
// Test 4b — non-contiguous bus IDs resolve to the correct physical bus
//           (regression: canonical projection renumbers buses to 1..N, so the
//           original external id must be translated through BusMergeMap rather
//           than looked up verbatim against the renumbered index map)
// ---------------------------------------------------------------------------
TEST_CASE("SC: non-contiguous bus IDs resolve to the correct bus",
          "[short_circuit][api][noncontiguous]") {
  // Radial feeder with gaps in the numbering: 1 (slack) — 5 — 10.
  const Cx Z15(0.20, 0.40);
  const Cx Z5_10(0.30, 0.60);
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(5, BusType::PQ), make_bus(10, BusType::PQ)},
      {make_branch(1, 1, 5,  Z15.real(),   Z15.imag()),
       make_branch(2, 5, 10, Z5_10.real(), Z5_10.imag())},
      100.0, 0.001);

  SCOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor   = 1.0;

  // 1) All-buses solve reports the ORIGINAL external ids, in order.
  auto full = compute_short_circuit(sys, opt);
  REQUIRE(full.bus_results.size() == 3);
  CHECK(full.bus_results[0].bus_id == 1);
  CHECK(full.bus_results[1].bus_id == 5);
  CHECK(full.bus_results[2].bus_id == 10);

  // Thévenin impedance grows with distance from the slack ⇒ Sk decreases.
  CHECK(std::abs(full.bus_results[0].z_thevenin) < std::abs(full.bus_results[1].z_thevenin));
  CHECK(std::abs(full.bus_results[1].z_thevenin) < std::abs(full.bus_results[2].z_thevenin));
  CHECK(full.bus_results[0].sk_mva > full.bus_results[1].sk_mva);
  CHECK(full.bus_results[1].sk_mva > full.bus_results[2].sk_mva);

  // 2) Single-bus API faults the CORRECT physical bus for each external id.
  for (int ext : {1, 5, 10}) {
    const BusFaultResult* expected = &full.bus_results[0];
    for (const auto& br : full.bus_results)
      if (br.bus_id == ext) expected = &br;
    auto single = compute_fault_at_bus(sys, ext, opt);
    CHECK(single.bus_id == ext);
    CHECK(std::abs(single.z_thevenin - expected->z_thevenin) < 1e-12);
    CHECK(std::abs(single.sk_mva     - expected->sk_mva)     < 1e-6);
  }

  // 3) Detailed API accepts the original ids and reports them back.
  SCDetailedOptions dopt;
  dopt.fault_type = FaultType::ThreePhase;
  auto det = run_short_circuit_detailed(sys, 10, dopt);
  CHECK(det.solved);
  CHECK(det.fault_bus_id == 10);
  const SCDetailedBusResult* fault_row = nullptr;
  for (const auto& br : det.bus_results)
    if (br.bus_id == 10) fault_row = &br;
  REQUIRE(fault_row != nullptr);
  CHECK(fault_row->ikss_ka > 0.0);

  // 4) A non-existent external id is rejected, not silently computed.
  CHECK_THROWS_AS(run_short_circuit_detailed(sys, 999, dopt), std::invalid_argument);
  CHECK_THROWS_AS(compute_fault_at_bus(sys, 999, opt), std::invalid_argument);
}

TEST_CASE("SC detailed: downstream fault reports upstream source contribution and voltages",
          "[short_circuit][detailed][regression]") {
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ), make_bus(3, BusType::PQ)},
      {make_branch(1, 1, 2, 0.02, 0.20),
       make_branch(2, 2, 3, 0.03, 0.30)},
      100.0,
      0.001);
  sys.ac.generators.clear();

  ExternalGrid eg;
  eg.index = 1;
  eg.bus = 1;
  eg.in_service = true;
  eg.s_sc_max_mva = 5000.0;
  eg.rx_max = 0.1;
  sys.ac.external_grids.push_back(eg);

  SCDetailedOptions dopt;
  dopt.fault_type = FaultType::ThreePhase;
  dopt.calc_type = SCCalcType::Max;
  auto det = run_short_circuit_detailed(sys, 3, dopt);
  REQUIRE(det.solved);
  REQUIRE(det.bus_results.size() == 3);

  const SCDetailedBusResult* bus1 = nullptr;
  const SCDetailedBusResult* bus2 = nullptr;
  const SCDetailedBusResult* fault_row = nullptr;
  for (const auto& br : det.bus_results) {
    if (br.bus_id == 1) bus1 = &br;
    if (br.bus_id == 2) bus2 = &br;
    if (br.bus_id == 3) fault_row = &br;
  }
  REQUIRE(bus1 != nullptr);
  REQUIRE(bus2 != nullptr);
  REQUIRE(fault_row != nullptr);

  CHECK(fault_row->ikss_ka > 0.0);
  CHECK(fault_row->ikss_extgrid_contrib_ka > 0.0);
  CHECK(fault_row->v_remaining_pu == 0.0);
  CHECK(bus1->v_remaining_pu > 0.0);
  CHECK(bus2->v_remaining_pu > 0.0);
  CHECK(std::isfinite(bus1->v_remaining_pu));
  CHECK(std::isfinite(bus2->v_remaining_pu));
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
  // Sk and Ik'' scale linearly with IEC voltage factor c.
  CHECK(sk_high > sk_low);
  CHECK(sk_high / sk_low > 1.09);
  CHECK(sk_high / sk_low < 1.11);
  CHECK(res_low.bus_results[1].ikpp_ka > 1.0);
}

TEST_CASE("SC detailed: explicit c-factor scales selected-bus current",
          "[short_circuit][iec60909][detailed]") {
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)},
      {make_branch(1, 1, 2, 0.05, 0.20)},
      100.0, 0.05);
  sys.ac.generators.clear();
  ExternalGrid eg;
  eg.index = 1;
  eg.bus = 1;
  eg.r_pu = 0.001;
  eg.x_pu = 0.01;
  sys.ac.external_grids = {eg};

  SCDetailedOptions lo, hi;
  lo.fault_type = FaultType::ThreePhase;
  hi.fault_type = FaultType::ThreePhase;
  lo.c_factor = 1.0;
  hi.c_factor = 1.1;

  const auto r_lo = run_short_circuit_detailed(sys, 2, lo);
  const auto r_hi = run_short_circuit_detailed(sys, 2, hi);
  REQUIRE(r_lo.solved);
  REQUIRE(r_hi.solved);

  auto own_ik = [](const SCDetailedResult& r) {
    for (const auto& b : r.bus_results)
      if (b.bus_id == r.fault_bus_id) return b.ikss_ka;
    return 0.0;
  };
  const double i_lo = own_ik(r_lo);
  const double i_hi = own_ik(r_hi);
  CHECK(i_lo > 0.0);
  CHECK(i_hi / i_lo > 1.09);
  CHECK(i_hi / i_lo < 1.11);
}

TEST_CASE("SC detailed: VSC AC grid-forming flag selects voltage-source model",
          "[short_circuit][converter][iec60909]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::PQ, 20.0)};
  sys.ac.generators.clear();

  DCBus dc;
  dc.index = 1;
  dc.bus_type = DCBusType::DC_V;
  dc.base_kv = 320.0;
  dc.in_service = true;
  sys.dc.buses = {dc};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 1;
  vsc.bus_dc = 1;
  vsc.in_service = true;
  vsc.p_rated_mw = 20.0;
  vsc.r_sc_pu = 0.0;
  vsc.x_sc_pu = 0.2;
  vsc.i_max_pu = 1.2;
  vsc.grid_forming = true;     // DC-side forming only.
  vsc.ac_grid_forming = false;
  sys.vsc_converters = {vsc};

  SCDetailedOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor = 1.0;

  const auto dc_forming_only = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(dc_forming_only.solved);
  const auto current_limited = dc_forming_only.bus_results.front().ikss_converter_contrib_ka;
  const double expected_current_limited = 1.2 * 20.0 / (std::sqrt(3.0) * 20.0);
  CHECK(std::abs(current_limited - expected_current_limited) < 1e-9);

  sys.vsc_converters.front().ac_grid_forming = true;
  const auto ac_forming = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(ac_forming.solved);
  const auto voltage_source = ac_forming.bus_results.front().ikss_converter_contrib_ka;

  CHECK(voltage_source > current_limited * 3.0);
  CHECK(std::abs(ac_forming.bus_results.front().ikss_ka - voltage_source) < voltage_source * 1e-9);
}

TEST_CASE("SC detailed: minimum external-grid data lowers fault current",
          "[short_circuit][iec60909][external_grid]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 110.0)};
  ExternalGrid eg;
  eg.index = 1;
  eg.bus = 1;
  eg.vn_kv = 110.0;
  eg.s_sc_max_mva = 5000.0;
  eg.s_sc_min_mva = 500.0;
  eg.rx_max = 0.1;
  eg.rx_min = 0.1;
  sys.ac.external_grids = {eg};

  SCDetailedOptions max_opt, min_opt;
  max_opt.fault_type = FaultType::ThreePhase;
  min_opt.fault_type = FaultType::ThreePhase;
  max_opt.calc_type = SCCalcType::Max;
  min_opt.calc_type = SCCalcType::Min;

  const auto r_max = run_short_circuit_detailed(sys, 1, max_opt);
  const auto r_min = run_short_circuit_detailed(sys, 1, min_opt);
  REQUIRE(r_max.solved);
  REQUIRE(r_min.solved);

  const double i_max = r_max.bus_results.front().ikss_ka;
  const double i_min = r_min.bus_results.front().ikss_ka;
  CHECK(i_max > 0.0);
  CHECK(i_min > 0.0);
  CHECK(i_min < i_max * 0.2);
}

TEST_CASE("SC detailed: transformer correction factor is applied in canonical space",
          "[short_circuit][iec60909][transformer]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::SLACK, 110.0),
      make_bus(2, BusType::PQ, 20.0),
  };

  ExternalGrid eg;
  eg.index = 1;
  eg.bus = 1;
  eg.r_pu = 0.001;
  eg.x_pu = 0.01;
  sys.ac.external_grids = {eg};
  sys.ac.generators.clear();

  Transformer2W tr;
  tr.index = 1;
  tr.hv_bus = 1;
  tr.lv_bus = 2;
  tr.sn_mva = 100.0;
  tr.vn_hv_kv = 110.0;
  tr.vn_lv_kv = 20.0;
  tr.vk_percent = 10.0;
  tr.vkr_percent = 0.0;
  sys.ac.transformers_2w = {tr};

  SCDetailedOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.calc_type = SCCalcType::Max;
  opt.c_factor = 1.1;

  const auto r = run_short_circuit_detailed(sys, 2, opt);
  REQUIRE(r.solved);
  const auto& row = r.bus_results.back();

  const double kt = 0.95 * 1.1 / (1.0 + 0.6 * 0.10);
  const Cx expected_z = Cx(0.001, 0.01) + Cx(0.0, 0.10) * kt;
  const double expected_ka =
      (1.1 / std::abs(expected_z)) * (100.0 / (std::sqrt(3.0) * 20.0));

  CHECK(row.bus_id == 2);
  CHECK(std::abs(row.ikss_ka - expected_ka) < expected_ka * 0.02);
}

TEST_CASE("SC projection: transformer zero-sequence percent respects x0/r0 ratio",
          "[short_circuit][iec60909][transformer][zero_sequence]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::SLACK, 110.0),
      make_bus(2, BusType::PQ, 20.0),
  };

  Transformer2W tr;
  tr.index = 1;
  tr.hv_bus = 1;
  tr.lv_bus = 2;
  tr.sn_mva = 50.0;
  tr.vn_hv_kv = 110.0;
  tr.vn_lv_kv = 20.0;
  tr.vk_percent = 10.0;
  tr.vkr_percent = 1.0;
  tr.z0_percent = 8.0;
  tr.x0_r0 = 4.0;
  sys.ac.transformers_2w = {tr};

  const auto proj = project_to_canonical_models(sys);
  REQUIRE(!proj.ac.branches.empty());
  const auto& br = proj.ac.branches.back();

  const double z0 = 0.08 * (100.0 / 50.0);
  const double expected_r0 = z0 / std::sqrt(1.0 + 4.0 * 4.0);
  const double expected_x0 = expected_r0 * 4.0;

  CHECK(std::abs(br.r0_pu - expected_r0) < 1e-12);
  CHECK(std::abs(br.x0_pu - expected_x0) < 1e-12);
}

TEST_CASE("SC detailed: projected rich motors remain motor contributions and obey threshold",
          "[short_circuit][iec60909][motor]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0)};
  sys.ac.generators.clear();

  ExternalGrid eg;
  eg.index = 1;
  eg.bus = 1;
  eg.r_pu = 0.01;
  eg.x_pu = 0.10;
  sys.ac.external_grids = {eg};

  AsynchronousMotor small;
  small.index = 1;
  small.bus = 1;
  small.vn_kv = 20.0;
  small.sn_mva = 0.04;
  small.cos_phi = 0.9;
  small.efficiency = 0.9;
  small.r_pu = 0.5;
  small.x_pu = 2.0;

  AsynchronousMotor large = small;
  large.index = 2;
  large.sn_mva = 1.0;
  large.r_pu = 5.0;
  large.x_pu = 20.0;
  large.poles = 4;

  SCDetailedOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor = 1.0;

  auto base = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(base.solved);
  const double i_base = base.bus_results.front().ikss_ka;

  sys.ac.motors = {small};
  auto with_small = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(with_small.solved);
  CHECK(with_small.bus_results.front().ikss_motor_contrib_ka == 0.0);
  CHECK(std::abs(with_small.bus_results.front().ikss_ka - i_base) < 1e-9);

  sys.ac.motors = {large};
  auto with_large = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(with_large.solved);
  const auto& row = with_large.bus_results.front();
  CHECK(row.ikss_motor_contrib_ka > 0.0);
  CHECK(row.ikss_load_contrib_ka == 0.0);
  CHECK(row.ikss_ka > i_base);
  CHECK(row.ib_ka > 0.0);
  CHECK(std::isfinite(row.ib_ka));
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

TEST_CASE("SC: classical hand examples match embedded expected values",
          "[short_circuit][classical_examples]") {
#ifdef HACDCPF_PROJECT_ROOT
  const std::string root = HACDCPF_PROJECT_ROOT;
#else
  const std::string root = ".";
#endif
  const std::vector<std::string> files = {
      "sc_hand_01_two_bus_source_line.json",
      "sc_hand_02_three_bus_radial_feeder.json",
      "sc_hand_03_slg_low_zero_sequence.json",
      "sc_hand_04_external_grid_max_min.json",
      "sc_hand_05_transformer_correction.json",
      "sc_hand_06_converter_grid_following.json",
      "sc_hand_07_converter_ac_grid_forming.json",
  };

  for (const auto& file : files) {
    const std::string path = root + "/external_data/classical_examples/" + file;
    INFO("case=" << file);
    const std::string text = read_text_file(path);
    const json doc = json::parse(text);
    const auto sys = hacdcpf::io::from_json(text);
    REQUIRE(doc.contains("expected_short_circuit"));
    const json& meta = doc["expected_short_circuit"];
    REQUIRE(meta.contains("checks"));
      const double abs_tol = check_abs_tolerance(meta);
    const double rel_tol = check_rel_tolerance(meta);

    for (const auto& check : meta["checks"]) {
      INFO("case=" << file << " fault_bus=" << check.value("fault_bus_id", -1)
                   << " fault_type=" << check["options"].value("fault_type", "ThreePhase"));
      SCDetailedOptions opt;
      const auto& opts = check["options"];
      opt.fault_type = fault_type_from_name(opts.value("fault_type", "ThreePhase"));
      opt.calc_type = calc_type_from_name(opts.value("calc_type", "Max"));
      opt.c_factor = opts.value("c_factor", 0.0);
      opt.compute_branch_flows = false;
      opt.compute_voltage_drops = true;
      opt.compute_ith = false;

      const int fault_bus = check.at("fault_bus_id").get<int>();
      const auto result = run_short_circuit_detailed(sys, fault_bus, opt);
      REQUIRE(result.solved);
      const auto& row = fault_row(result);

      const double expected_ik = check.at("ikss_ka").get<double>();
      INFO("actual_ikss_ka=" << row.ikss_ka << " expected_ikss_ka=" << expected_ik);
      CHECK(std::abs(row.ikss_ka - expected_ik)
            <= std::max(abs_tol, rel_tol * std::max(1.0, std::abs(expected_ik))));

      if (check.contains("ikss_converter_contrib_ka")) {
        const double expected = check.at("ikss_converter_contrib_ka").get<double>();
        CHECK(std::abs(row.ikss_converter_contrib_ka - expected)
              <= std::max(abs_tol, rel_tol * std::max(1.0, std::abs(expected))));
      }
      if (check.contains("sk_mva")) {
        const double expected = check.at("sk_mva").get<double>();
        const double bus_kv = [&]() {
          for (const auto& b : sys.ac.buses)
            if (b.index == fault_bus) return b.base_kv;
          return 1.0;
        }();
        const double sk = std::sqrt(3.0) * bus_kv * row.ikss_ka;
        CHECK(std::abs(sk - expected)
              <= std::max(1e-4, rel_tol * std::max(1.0, std::abs(expected))));
      }
    }
  }
}

TEST_CASE("SC: practical classical examples satisfy stress invariants",
          "[short_circuit][classical_examples][practical]") {
#ifdef HACDCPF_PROJECT_ROOT
  const std::string root = HACDCPF_PROJECT_ROOT;
#else
  const std::string root = ".";
#endif
  const std::vector<std::string> files = {
      "sc_practical_01_industrial_plant_110_20kv.json",
      "sc_practical_02_urban_meshed_10kv_feeder.json",
      "sc_practical_03_hybrid_acdc_inverter_microgrid.json",
      "sc_practical_04_ground_fault_low_z0_transformer_feeder.json",
  };

  for (const auto& file : files) {
    const std::string path = root + "/external_data/classical_examples/" + file;
    INFO("case=" << file);
    const std::string text = read_text_file(path);
    const json doc = json::parse(text);
    const auto sys = hacdcpf::io::from_json(text);
    REQUIRE(doc.contains("expected_short_circuit"));
    const json& meta = doc["expected_short_circuit"];
    REQUIRE(meta.contains("practical_checks"));

    const auto projected = project_to_canonical_models(sys);
    if (meta.contains("canonical_checks")) {
      const auto& cc = meta["canonical_checks"];
      if (cc.contains("min_projected_branch_count")) {
        CHECK(projected.ac.branches.size() >=
              cc.at("min_projected_branch_count").get<size_t>());
      }
      if (cc.contains("min_transformer_origin_branches")) {
        CHECK(count_branch_origins(projected, BranchOriginType::Transformer2W) >=
              cc.at("min_transformer_origin_branches").get<int>());
      }
      if (cc.contains("min_projected_loads_from_motors")) {
        int motor_loads = 0;
        for (const auto& ld : projected.ac.loads) {
          if (ld.sc_source_type == "AsynchronousMotor") ++motor_loads;
        }
        CHECK(motor_loads >= cc.at("min_projected_loads_from_motors").get<int>());
      }
      if (cc.value("rich_motors_removed_after_projection", false)) {
        CHECK(projected.ac.motors.empty());
      }
      if (cc.contains("min_vsc_converters")) {
        CHECK(projected.vsc_converters.size() >=
              cc.at("min_vsc_converters").get<size_t>());
      }
    }

    std::unordered_map<std::string, SCDetailedBusResult> by_check_id;

    for (const auto& check : meta["practical_checks"]) {
      const std::string check_id = check.value("id", "");
      INFO("case=" << file << " check=" << check_id
                   << " fault_bus=" << check.value("fault_bus_id", -1));
      SCDetailedOptions opt;
      const auto& opts = check["options"];
      opt.fault_type = fault_type_from_name(opts.value("fault_type", "ThreePhase"));
      opt.calc_type = calc_type_from_name(opts.value("calc_type", "Max"));
      opt.c_factor = opts.value("c_factor", 0.0);
      opt.compute_branch_flows = opts.value("compute_branch_flows", false);
      opt.compute_voltage_drops = true;
      opt.compute_ith = false;

      const int fault_bus = check.at("fault_bus_id").get<int>();
      const auto result = run_short_circuit_detailed(sys, fault_bus, opt);
      REQUIRE(result.solved);
      const auto& row = fault_row(result);
      CHECK(row.ikss_ka > 0.0);
      CHECK(std::isfinite(row.ikss_ka));
      CHECK(row.ip_ka >= row.ikss_1_ka);

      if (check.contains("ikss_min_ka")) {
        CHECK(row.ikss_ka >= check.at("ikss_min_ka").get<double>());
      }
      if (check.contains("ikss_max_ka")) {
        CHECK(row.ikss_ka <= check.at("ikss_max_ka").get<double>());
      }
      if (check.contains("contribution_min_ka")) {
        for (auto it = check["contribution_min_ka"].begin();
             it != check["contribution_min_ka"].end(); ++it) {
          const double actual = sc_result_field(row, it.key());
          CHECK(actual >= it.value().get<double>());
        }
      }
      if (check.contains("branch_current_min_ka")) {
        REQUIRE(opt.compute_branch_flows);
        for (const auto& bc : check["branch_current_min_ka"]) {
          const int branch_index = bc.at("branch_index").get<int>();
          const auto* br = branch_row(result, branch_index);
          REQUIRE(br != nullptr);
          CHECK(br->i_branch_ka >= bc.at("min_ka").get<double>());
        }
      }
      if (check.contains("compare")) {
        const auto& cmp = check["compare"];
        const std::string ref_id = cmp.at("check_id").get<std::string>();
        REQUIRE(by_check_id.count(ref_id) == 1);
        const std::string field = cmp.value("field", "ikss_ka");
        const double actual = sc_result_field(row, field);
        const double ref = sc_result_field(by_check_id.at(ref_id), field);
        const std::string relation = cmp.value("relation", "");
        if (relation == "less_than") {
          CHECK(actual < ref * cmp.value("ratio_max", 1.0));
        } else if (relation == "greater_than") {
          CHECK(actual > ref * cmp.value("ratio_min", 1.0));
        } else {
          FAIL("Unknown practical comparison relation: " << relation);
        }
      }
      if (!check_id.empty()) {
        by_check_id[check_id] = row;
      }
    }
  }
}
