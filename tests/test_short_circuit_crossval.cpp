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

#include <algorithm>
#include <cmath>
#include <complex>
#include <fstream>
#include <limits>
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
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Cx = std::complex<double>;
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static ACBus make_bus(int id, BusType t, double kv = 100.0);
static ACBranch make_branch(int id, int f, int t, double r, double x);
static const SCDetailedBusResult& fault_row(const SCDetailedResult& r);

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
      g.mbase_mva = base_mva;
      g.vn_kv = b.base_kv;
      g.sc_lambda_max = 1.0;
      g.sc_lambda_min = 1.0;
      sys.ac.generators.push_back(g);
    }
  }
  return sys;
}

TEST_CASE("SC detailed rejects invalid numeric options",
          "[short_circuit][validation]") {
  const auto sys = make_sys({make_bus(1, BusType::SLACK)}, {});
  SCDetailedOptions opt;
  opt.base_frequency_hz = std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS_AS(run_short_circuit_detailed(sys, 1, opt), std::invalid_argument);
  opt = {};
  opt.fault_impedance_pu = -0.01;
  CHECK_THROWS_AS(run_short_circuit_detailed(sys, 1, opt), std::invalid_argument);
  opt = {};
  opt.inverse_rhs_batch_size = 0;
  CHECK_THROWS_AS(run_short_circuit_detailed(sys, 1, opt), std::invalid_argument);
}

TEST_CASE("SC detailed reports singular required sequence factors",
          "[short_circuit][validation][singular]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::PQ, 20.0)};
  const auto result = run_short_circuit_detailed(sys, 1);
  CHECK_FALSE(result.solved);
  CHECK(result.status == "numerical_failure");
  CHECK_FALSE(result.numerical_quality.factorization_valid);
  CHECK_FALSE(result.message.empty());
}

TEST_CASE("SC detailed reconstructs SLG and LL phase currents",
          "[short_circuit][sequence][phase-current]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0)};
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.r_pu = 0.01;
  grid.x_pu = 0.10;
  grid.r0_pu = 0.02;
  grid.x0_pu = 0.20;
  sys.ac.external_grids = {grid};
  SCDetailedOptions opt;
  opt.c_factor = 1.0;
  opt.compute_branch_flows = false;

  opt.fault_type = FaultType::SinglePhaseGround;
  const auto slg = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(slg.solved);
  const auto& slg_row = fault_row(slg);
  CHECK(slg_row.i_phase_a_ka > 0.0);
  CHECK(std::abs(slg_row.i_phase_b_ka) < 1e-10);
  CHECK(std::abs(slg_row.i_phase_c_ka) < 1e-10);
  CHECK(std::abs(slg_row.i_ground_ka - slg_row.i_phase_a_ka) < 1e-9);

  opt.fault_type = FaultType::TwoPhase;
  const auto ll = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(ll.solved);
  const auto& ll_row = fault_row(ll);
  CHECK(std::abs(ll_row.i_phase_a_ka) < 1e-10);
  CHECK(std::abs(ll_row.i_phase_b_ka - ll_row.i_phase_c_ka) < 1e-9);
  CHECK(std::abs(ll_row.i_ground_ka) < 1e-10);

  const Cx z1(0.01, 0.10);
  const Cx z2 = z1;
  const Cx z0(0.02, 0.20);
  const Cx zf(0.05, 0.0);
  const Cx a(-0.5, std::sqrt(3.0) / 2.0);
  const Cx a2 = a * a;
  const double i_base_ka = 100.0 / (std::sqrt(3.0) * 20.0);

  opt.fault_impedance_pu = zf.real();
  const auto ll_resistive = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(ll_resistive.solved);
  const auto& ll_resistive_row = fault_row(ll_resistive);
  const Cx ll_i1 = 1.0 / (z1 + z2 + zf);
  const double ll_phase_ka = std::abs((a2 - a) * ll_i1) * i_base_ka;
  CHECK(ll_resistive_row.i_phase_b_ka == Catch::Approx(ll_phase_ka).margin(1e-9));
  CHECK(ll_resistive_row.i_phase_c_ka == Catch::Approx(ll_phase_ka).margin(1e-9));
  CHECK(ll_resistive_row.ikss_ka == Catch::Approx(ll_phase_ka).margin(1e-9));

  opt.fault_type = FaultType::TwoPhaseGround;
  const auto llg = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(llg.solved);
  const auto& llg_row = fault_row(llg);
  const Cx z0f = z0 + 3.0 * zf;
  const Cx llg_i1 = 1.0 / (z1 + z2 * z0f / (z2 + z0f));
  const Cx llg_i2 = -llg_i1 * z0f / (z2 + z0f);
  const Cx llg_i0 = -llg_i1 * z2 / (z2 + z0f);
  const double llg_b_ka = std::abs(llg_i0 + a2 * llg_i1 + a * llg_i2) * i_base_ka;
  const double llg_c_ka = std::abs(llg_i0 + a * llg_i1 + a2 * llg_i2) * i_base_ka;
  CHECK(llg_row.i_phase_a_ka == Catch::Approx(0.0).margin(1e-9));
  CHECK(llg_row.i_phase_b_ka == Catch::Approx(llg_b_ka).margin(1e-9));
  CHECK(llg_row.i_phase_c_ka == Catch::Approx(llg_c_ka).margin(1e-9));
  CHECK(llg_row.i_ground_ka ==
        Catch::Approx(3.0 * std::abs(llg_i0) * i_base_ka).margin(1e-9));
  CHECK(llg_row.ikss_ka == Catch::Approx(std::max(llg_b_ka, llg_c_ka)).margin(1e-9));
}

TEST_CASE("SC detailed branch output carries authored transformer identity",
          "[short_circuit][projection][identity]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 110.0),
                  make_bus(2, BusType::PQ, 20.0)};
  ExternalGrid grid;
  grid.index = 1; grid.bus = 1; grid.r_pu = 0.01; grid.x_pu = 0.1;
  grid.r0_pu = 0.01; grid.x0_pu = 0.1;
  sys.ac.external_grids = {grid};
  Transformer2W transformer;
  transformer.index = 77;
  transformer.hv_bus = 1; transformer.lv_bus = 2;
  transformer.sn_mva = 100.0;
  transformer.vn_hv_kv = 110.0; transformer.vn_lv_kv = 20.0;
  transformer.vk_percent = 10.0; transformer.vkr_percent = 1.0;
  transformer.z0_percent = 10.0; transformer.x0_r0 = 10.0;
  sys.ac.transformers_2w = {transformer};
  const auto result = run_short_circuit_detailed(sys, 2);
  REQUIRE(result.solved);
  REQUIRE_FALSE(result.branch_results.empty());
  const auto& branch = result.branch_results.front();
  CHECK(branch.domain == "AC");
  CHECK(branch.component_kind == "Transformer2W");
  CHECK(branch.component_index == 77);
  CHECK(branch.attribution_complete);
}

TEST_CASE("SC detailed preserves contracted switch identity without fake current",
          "[short_circuit][projection][identity][switch]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0),
                  make_bus(2, BusType::PQ, 20.0),
                  make_bus(3, BusType::PQ, 20.0)};
  sys.ac.branches = {make_branch(10, 2, 3, 0.01, 0.1)};
  ExternalGrid grid;
  grid.index = 1; grid.bus = 1; grid.r_pu = 0.01; grid.x_pu = 0.1;
  grid.r0_pu = 0.01; grid.x0_pu = 0.1;
  sys.ac.external_grids = {grid};
  Switch sw;
  sw.index = 88; sw.bus_from = 1; sw.bus_to = 2;
  sw.closed = true; sw.in_service = true;
  sw.r_contact_ohm = 0.0; sw.z_ohm = 0.0;
  sys.ac.switches = {sw};
  const auto result = run_short_circuit_detailed(sys, 3);
  REQUIRE(result.solved);
  const auto found = std::find_if(
      result.branch_results.begin(), result.branch_results.end(),
      [](const auto& row) {
        return row.component_kind == "Switch" && row.component_index == 88;
      });
  REQUIRE(found != result.branch_results.end());
  CHECK_FALSE(found->electrical_value_available);
  CHECK_FALSE(found->message.empty());
}

TEST_CASE("SC detailed assembles Transformer3W canonical equivalent exactly once",
          "[short_circuit][projection][transformer3w]") {
  HybridPowerSystem rich;
  rich.base_mva = rich.ac.base_mva = 100.0;
  rich.ac.buses = {make_bus(1, BusType::SLACK, 110.0),
                   make_bus(2, BusType::PQ, 33.0),
                   make_bus(3, BusType::PQ, 11.0)};
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.r_pu = grid.r0_pu = 0.01;
  grid.x_pu = grid.x0_pu = 0.10;
  rich.ac.external_grids = {grid};

  Transformer3W transformer;
  transformer.index = 91;
  transformer.hv_bus = 1;
  transformer.mv_bus = 2;
  transformer.lv_bus = 3;
  transformer.sn_hv_mva = 100.0;
  transformer.sn_mv_mva = 60.0;
  transformer.sn_lv_mva = 40.0;
  transformer.vn_hv_kv = 110.0;
  transformer.vn_mv_kv = 33.0;
  transformer.vn_lv_kv = 11.0;
  transformer.vk_hv_mv_percent = 10.0;
  transformer.vk_hv_lv_percent = 12.0;
  transformer.vk_mv_lv_percent = 8.0;
  transformer.vkr_hv_mv_percent = 1.0;
  transformer.vkr_hv_lv_percent = 1.2;
  transformer.vkr_mv_lv_percent = 0.8;
  rich.ac.transformers_3w = {transformer};

  SCDetailedOptions options;
  options.c_factor = 1.0;
  options.apply_iec_transformer_correction = false;
  options.compute_branch_flows = false;
  options.compute_voltage_drops = false;
  const auto rich_result = run_short_circuit_detailed(rich, 3, options);
  REQUIRE(rich_result.solved);

  auto canonical = projection::RichToCanonicalOperator::apply(rich).canonical;
  canonical.ac.transformers_3w.clear();
  const auto canonical_result = run_short_circuit_detailed(canonical, 3, options);
  REQUIRE(canonical_result.solved);
  CHECK(fault_row(rich_result).ikss_ka ==
        Catch::Approx(fault_row(canonical_result).ikss_ka).margin(1e-10));
  CHECK(fault_row(rich_result).ip_ka ==
        Catch::Approx(fault_row(canonical_result).ip_ka).margin(1e-10));
}

static ACBus make_bus(int id, BusType t, double kv) {
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

static const SCDetailedBusResult& bus_row(const SCDetailedResult& r, int bus_id) {
  for (const auto& row : r.bus_results)
    if (row.bus_id == bus_id) return row;
  FAIL("Bus row not found");
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
    CHECK(r.z_thevenin.real() >= -1e-12);
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

TEST_CASE("SC broadcasts ideal-merge results to every rich bus",
          "[short_circuit][projection][attribution]") {
  auto sys = make_sys(
      {make_bus(10, BusType::SLACK), make_bus(20, BusType::PQ),
       make_bus(30, BusType::PQ)},
      {make_branch(1, 10, 20, 0.02, 0.20)}, 100.0, 0.001);
  CircuitBreaker breaker;
  breaker.index = 34;
  breaker.bus_from = 20;
  breaker.bus_to = 30;
  breaker.closed = true;
  breaker.z_ohm = 0.0;
  sys.ac.circuit_breakers.push_back(breaker);

  SCOptions options;
  options.fault_type = FaultType::ThreePhase;
  options.c_factor = 1.0;
  const auto result = compute_short_circuit(sys, options);
  REQUIRE(result.bus_results.size() == 3);
  CHECK(result.bus_results[0].bus_id == 10);
  CHECK(result.bus_results[1].bus_id == 20);
  CHECK(result.bus_results[2].bus_id == 30);
  CHECK(std::abs(result.bus_results[1].z_thevenin -
                 result.bus_results[2].z_thevenin) < 1e-12);

  SCDetailedOptions detailed_options;
  detailed_options.fault_type = FaultType::ThreePhase;
  detailed_options.compute_branch_flows = false;
  const auto detailed =
      run_short_circuit_detailed(sys, 30, detailed_options);
  REQUIRE(detailed.solved);
  REQUIRE(detailed.bus_results.size() == 3);
  CHECK(detailed.bus_results[1].bus_id == 20);
  CHECK(detailed.bus_results[2].bus_id == 30);
  CHECK(std::abs(detailed.bus_results[1].ikss_ka -
                 detailed.bus_results[2].ikss_ka) < 1e-12);
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

TEST_CASE("SC detailed voltage-profile mode skips non-fault inverse diagonals",
          "[short_circuit][detailed][selected_inverse]") {
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ),
       make_bus(3, BusType::PQ)},
      {make_branch(1, 1, 2, 0.02, 0.20),
       make_branch(2, 2, 3, 0.03, 0.30)},
      100.0, 0.001);

  SCDetailedOptions full_options;
  full_options.compute_branch_flows = false;
  const auto full = run_short_circuit_detailed(sys, 3, full_options);

  auto profile_options = full_options;
  profile_options.compute_nonfault_currents = false;
  const auto profile = run_short_circuit_detailed(sys, 3, profile_options);

  REQUIRE(full.solved);
  REQUIRE(profile.solved);
  REQUIRE(profile.bus_results.size() == full.bus_results.size());
  for (size_t i = 0; i < full.bus_results.size(); ++i) {
    CHECK(profile.bus_results[i].v_remaining_pu ==
          Catch::Approx(full.bus_results[i].v_remaining_pu).margin(1e-12));
    if (profile.bus_results[i].bus_id == 3) {
      CHECK(profile.bus_results[i].ikss_ka ==
            Catch::Approx(full.bus_results[i].ikss_ka).margin(1e-12));
      CHECK(profile.bus_results[i].ip_ka ==
            Catch::Approx(full.bus_results[i].ip_ka).margin(1e-12));
      CHECK(profile.bus_results[i].ib_ka ==
            Catch::Approx(full.bus_results[i].ib_ka).margin(1e-12));
      CHECK(profile.bus_results[i].ik_ka ==
            Catch::Approx(full.bus_results[i].ik_ka).margin(1e-12));
      CHECK(profile.bus_results[i].ith_ka ==
            Catch::Approx(full.bus_results[i].ith_ka).margin(1e-12));
    } else {
      CHECK(profile.bus_results[i].ikss_ka == 0.0);
      CHECK(profile.bus_results[i].ip_ka == 0.0);
      CHECK(profile.bus_results[i].ib_ka == 0.0);
      CHECK(profile.bus_results[i].ik_ka == 0.0);
      CHECK(profile.bus_results[i].ith_ka == 0.0);
    }
  }
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
  REQUIRE(dc_forming_only.converter_contributions.size() == 1);
  CHECK(dc_forming_only.converter_contributions.front().model ==
        "dc_side_forming_current_limited_source");
  CHECK(dc_forming_only.converter_contributions.front().dc_grid_forming);
  CHECK_FALSE(dc_forming_only.converter_contributions.front().ac_grid_forming);
  const auto current_limited = dc_forming_only.bus_results.front().ikss_converter_contrib_ka;
  const double expected_current_limited = 1.2 * 20.0 / (std::sqrt(3.0) * 20.0);
  CHECK(std::abs(current_limited - expected_current_limited) < 1e-9);
  CHECK(dc_forming_only.bus_results.front().ip_ka ==
        Catch::Approx(std::sqrt(2.0) * current_limited).epsilon(1e-12));

  sys.vsc_converters.front().ac_grid_forming = true;
  const auto ac_forming = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(ac_forming.solved);
  REQUIRE(ac_forming.converter_contributions.size() == 1);
  CHECK(ac_forming.converter_contributions.front().model ==
        "ac_grid_forming_voltage_source");
  CHECK(ac_forming.converter_contributions.front().ac_grid_forming);
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

TEST_CASE("SC overview includes an external-grid source without generators",
          "[short_circuit][overview][external_grid]") {
  auto sys = make_sys(
      {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)},
      {make_branch(1, 1, 2, 0.02, 0.20)}, 100.0, 0.001);
  sys.ac.generators.clear();

  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.in_service = true;
  grid.s_sc_max_mva = 5000.0;
  grid.rx_max = 0.1;
  sys.ac.external_grids = {grid};

  const auto result = compute_short_circuit(sys);
  REQUIRE(result.bus_results.size() == 2);
  CHECK(result.bus_results[0].ikpp_ka > 0.0);
  CHECK(result.bus_results[1].ikpp_ka > 0.0);
}

TEST_CASE("SC detailed: method B uses voltage-level peak-factor caps",
          "[short_circuit][iec60909][peak]") {
  // IEC 60909-0:2016, method B: the meshed correction is capped at 2.0 in
  // HV/MV systems and at 1.8 only in LV systems.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0)};
  ExternalGrid eg;
  eg.index = 1;
  eg.bus = 1;
  eg.r_pu = 1e-9;
  eg.x_pu = 0.1;
  sys.ac.external_grids = {eg};
  sys.ac.generators.clear();

  SCDetailedOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor = 1.0;
  opt.kappa_method = SCKappaMethod::B;
  opt.topology = SCTopology::Meshed;
  opt.compute_ith = false;

  const auto result = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(result.solved);
  const auto& row = fault_row(result);
  CHECK(row.ip_ka == Catch::Approx(2.0 * std::sqrt(2.0) * row.ikss_ka)
                         .epsilon(1e-6));

  sys.ac.buses.front().base_kv = 0.4;
  const auto lv_result = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(lv_result.solved);
  const auto& lv_row = fault_row(lv_result);
  CHECK(lv_row.ip_ka ==
        Catch::Approx(1.8 * std::sqrt(2.0) * lv_row.ikss_ka).epsilon(1e-6));
}

TEST_CASE("SC detailed: transferred current peak keeps the method B cap",
          "[short_circuit][iec60909][peak]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0),
                  make_bus(2, BusType::PQ, 20.0)};
  ExternalGrid eg;
  eg.index = 1;
  eg.bus = 1;
  eg.r_pu = 1e-9;
  eg.x_pu = 0.1;
  sys.ac.external_grids = {eg};
  sys.ac.generators.clear();
  sys.ac.branches = {make_branch(1, 1, 2, 0.01, 0.05)};

  SCDetailedOptions opt;
  opt.fault_type = FaultType::ThreePhase;
  opt.c_factor = 1.0;
  opt.kappa_method = SCKappaMethod::B;
  opt.topology = SCTopology::Meshed;
  opt.compute_ith = false;

  const auto result = run_short_circuit_detailed(sys, 1, opt);
  REQUIRE(result.solved);
  const auto& row2 = bus_row(result, 2);
  // The fault voltage level is 20 kV, so the method-B upper cap is 2.0.
  CHECK(row2.ip_ka == Catch::Approx(2.0 * std::sqrt(2.0) * row2.ikss_1_ka)
                          .epsilon(1e-12));
}

TEST_CASE("SC detailed: method B Auto classifies multiple source paths",
          "[short_circuit][iec60909][peak][topology]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0),
                  make_bus(2, BusType::PV, 20.0),
                  make_bus(3, BusType::PQ, 20.0)};
  sys.ac.branches = {make_branch(1, 1, 3, 0.05, 0.10),
                     make_branch(2, 2, 3, 0.05, 0.10)};
  ExternalGrid first;
  first.index = 1; first.bus = 1; first.r_pu = 0.05; first.x_pu = 0.10;
  ExternalGrid second = first;
  second.index = 2; second.bus = 2;
  sys.ac.external_grids = {first, second};

  SCDetailedOptions options;
  options.c_factor = 1.0;
  options.kappa_method = SCKappaMethod::B;
  options.compute_ith = false;
  options.compute_branch_flows = false;
  options.compute_nonfault_currents = false;
  options.topology = SCTopology::Auto;
  const auto auto_meshed = run_short_circuit_detailed(sys, 3, options);
  options.topology = SCTopology::Meshed;
  const auto explicit_meshed = run_short_circuit_detailed(sys, 3, options);
  REQUIRE(auto_meshed.solved);
  REQUIRE(explicit_meshed.solved);
  CHECK(fault_row(auto_meshed).ip_ka ==
        Catch::Approx(fault_row(explicit_meshed).ip_ka).epsilon(1e-12));

  sys.ac.branches.front().r_pu = 0.001;
  sys.ac.external_grids.front().r_pu = 0.001;
  options.topology = SCTopology::Auto;
  const auto auto_low_rx = run_short_circuit_detailed(sys, 3, options);
  options.topology = SCTopology::Radial;
  const auto explicit_radial = run_short_circuit_detailed(sys, 3, options);
  REQUIRE(auto_low_rx.solved);
  REQUIRE(explicit_radial.solved);
  CHECK(fault_row(auto_low_rx).ip_ka ==
        Catch::Approx(fault_row(explicit_radial).ip_ka).epsilon(1e-12));
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

TEST_CASE("SC detailed: zigzag neutral uses authored zero-sequence test impedance",
          "[short_circuit][iec60909][transformer][zero_sequence][zigzag]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::PQ, 110.0),
                  make_bus(2, BusType::SLACK, 20.0)};
  ExternalGrid grid;
  grid.index = 1; grid.bus = 2; grid.r_pu = 0.01; grid.x_pu = 0.10;
  grid.r0_pu = 0.02; grid.x0_pu = 0.20;
  sys.ac.external_grids = {grid};

  Transformer2W transformer;
  transformer.index = 1; transformer.hv_bus = 1; transformer.lv_bus = 2;
  transformer.sn_mva = 100.0;
  transformer.vn_hv_kv = 110.0; transformer.vn_lv_kv = 20.0;
  transformer.vk_percent = 10.0; transformer.vkr_percent = 1.0;
  transformer.z0_percent = 8.0; transformer.x0_r0 = 4.0;
  transformer.vector_group = "YNd";
  sys.ac.transformers_2w = {transformer};

  SCDetailedOptions options;
  options.fault_type = FaultType::SinglePhaseGround;
  options.c_factor = 1.0;
  options.apply_iec_transformer_correction = false;
  options.compute_branch_flows = false;
  options.compute_ith = false;
  const auto grounded_wye = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(grounded_wye.solved);

  sys.ac.transformers_2w.front().vector_group = "ZNd";
  const auto grounded_zigzag = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(grounded_zigzag.solved);
  CHECK(fault_row(grounded_zigzag).ikss_ka ==
        Catch::Approx(fault_row(grounded_wye).ikss_ka).epsilon(1e-12));
  CHECK(fault_row(grounded_zigzag).i_ground_ka ==
        Catch::Approx(fault_row(grounded_wye).i_ground_ka).epsilon(1e-12));
}

TEST_CASE("SC detailed: three-winding zigzag ports preserve zero-sequence equivalent",
          "[short_circuit][iec60909][transformer][three_winding][zigzag]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 110.0),
                  make_bus(2, BusType::PQ, 20.0),
                  make_bus(3, BusType::PQ, 10.0)};
  ExternalGrid grid;
  grid.index = 1; grid.bus = 1; grid.r_pu = 0.01; grid.x_pu = 0.10;
  grid.r0_pu = 0.02; grid.x0_pu = 0.20;
  sys.ac.external_grids = {grid};

  Transformer3W transformer;
  transformer.index = 1;
  transformer.hv_bus = 1; transformer.mv_bus = 2; transformer.lv_bus = 3;
  transformer.sn_hv_mva = 100.0;
  transformer.sn_mv_mva = 60.0; transformer.sn_lv_mva = 40.0;
  transformer.vn_hv_kv = 110.0;
  transformer.vn_mv_kv = 20.0; transformer.vn_lv_kv = 10.0;
  transformer.vk_hv_mv_percent = 10.0;
  transformer.vk_hv_lv_percent = 12.0;
  transformer.vk_mv_lv_percent = 8.0;
  transformer.vkr_hv_mv_percent = 1.0;
  transformer.vkr_hv_lv_percent = 1.2;
  transformer.vkr_mv_lv_percent = 0.8;
  transformer.vk0_hv_mv_percent = 10.0;
  transformer.vk0_hv_lv_percent = 12.0;
  transformer.vk0_mv_lv_percent = 8.0;
  transformer.vkr0_hv_mv_percent = 1.0;
  transformer.vkr0_hv_lv_percent = 1.2;
  transformer.vkr0_mv_lv_percent = 0.8;
  transformer.vector_group = "YNynyn";
  sys.ac.transformers_3w = {transformer};

  SCDetailedOptions options;
  options.fault_type = FaultType::SinglePhaseGround;
  options.c_factor = 1.0;
  options.apply_iec_transformer_correction = false;
  options.compute_branch_flows = false;
  options.compute_ith = false;
  const auto grounded_wye = run_short_circuit_detailed(sys, 3, options);
  REQUIRE(grounded_wye.solved);

  sys.ac.transformers_3w.front().vector_group = "ZNznyn";
  const auto grounded_zigzag = run_short_circuit_detailed(sys, 3, options);
  REQUIRE(grounded_zigzag.solved);
  CHECK(fault_row(grounded_zigzag).ikss_ka ==
        Catch::Approx(fault_row(grounded_wye).ikss_ka).epsilon(1e-12));
  CHECK(fault_row(grounded_zigzag).i_ground_ka ==
        Catch::Approx(fault_row(grounded_wye).i_ground_ka).epsilon(1e-12));
}

TEST_CASE("SC detailed: zero-sequence line capacitance closes an ungrounded fault path",
          "[short_circuit][iec60909][zero_sequence][capacitance]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0),
                  make_bus(2, BusType::PQ, 20.0)};
  Generator generator;
  generator.index = 1; generator.bus = 1; generator.mbase_mva = 100.0;
  generator.vn_kv = 20.0; generator.xdpp_pu = 0.10;
  generator.sc_lambda_max = 1.0; generator.sc_lambda_min = 1.0;
  sys.ac.generators = {generator};
  auto line = make_branch(1, 1, 2, 0.01, 0.10);
  line.r0_pu = 0.03; line.x0_pu = 0.30; line.b0_pu = 0.0;
  sys.ac.branches = {line};

  SCDetailedOptions options;
  options.fault_type = FaultType::SinglePhaseGround;
  options.c_factor = 1.0;
  options.compute_branch_flows = false;
  options.compute_ith = false;
  const auto open = run_short_circuit_detailed(sys, 2, options);
  REQUIRE(open.solved);
  CHECK(open.status == "solved_zero_sequence_open");
  CHECK(fault_row(open).ikss_ka == Catch::Approx(0.0).margin(1e-12));

  sys.ac.branches.front().b0_pu = 0.04;
  const auto capacitive = run_short_circuit_detailed(sys, 2, options);
  INFO("status=" << capacitive.status << " message=" << capacitive.message);
  REQUIRE(capacitive.solved);
  CHECK(capacitive.status == "solved");
  CHECK(fault_row(capacitive).ikss_ka > 0.0);
  CHECK(std::isfinite(fault_row(capacitive).ikss_ka));
}

TEST_CASE("SC detailed: LV transformer taps cross-validate with OpenDSS fault study",
          "[short_circuit][crossval][opendss][transformer][tap]") {
#ifdef HACDCPF_PROJECT_ROOT
  const std::string root = HACDCPF_PROJECT_ROOT;
#else
  const std::string root = ".";
#endif
  const json reference = json::parse(read_text_file(
      root + "/external_data/short_circuit_validation/opendss_transformer_taps.json"));

  for (const auto& expected : reference.at("cases")) {
    const double tap = expected.at("lv_tap_pu").get<double>();
    INFO("OpenDSS LV tap=" << tap);

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
    eg.r_pu = 0.00995037190209989;
    eg.x_pu = 0.0995037190209989;
    eg.r0_pu = eg.r_pu;
    eg.x0_pu = eg.x_pu;
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
    tr.vkr_percent = 1.0;
    tr.z0_percent = 10.0;
    tr.x0_r0 = std::sqrt(99.0);
    tr.tap_side = 1;
    tr.tap_neutral = 0;
    tr.tap_pos = static_cast<int>(std::lround((tap - 1.0) / 0.05));
    tr.tap_step_percent = 5.0;
    sys.ac.transformers_2w = {tr};

    SCDetailedOptions opt;
    opt.fault_type = FaultType::ThreePhase;
    opt.c_factor = 1.0;
    opt.kappa_method = SCKappaMethod::A;
    opt.topology = SCTopology::Radial;
    opt.apply_iec_transformer_correction = false;
    opt.compute_branch_flows = false;
    opt.compute_ith = false;

    const auto three_phase = run_short_circuit_detailed(sys, 2, opt);
    REQUIRE(three_phase.solved);
    const auto& row3 = fault_row(three_phase);
    CHECK(row3.ikss_ka == Catch::Approx(expected.at("ik3_ka").get<double>()).epsilon(2e-6));
    const double transformer_x = std::sqrt(0.10 * 0.10 - 0.01 * 0.01);
    const double participating_rx = std::min(
        eg.r_pu / eg.x_pu, 0.01 / transformer_x);
    const double method_a_ip = std::sqrt(2.0) *
        calculate_kappa_basic_sc(participating_rx) * row3.ikss_ka;
    CHECK(row3.ip_ka == Catch::Approx(method_a_ip).epsilon(2e-12));

    // Z0 = Z1 in this grounded OpenDSS fixture, so the SLG and 3-phase RMS
    // currents must agree. This specifically exercises LV-tap Z0 referral.
    opt.fault_type = FaultType::SinglePhaseGround;
    const auto ground = run_short_circuit_detailed(sys, 2, opt);
    REQUIRE(ground.solved);
    CHECK(fault_row(ground).ikss_ka == Catch::Approx(row3.ikss_ka).epsilon(2e-6));
  }
}

TEST_CASE("SC detailed method A uses the minimum participating branch R over X",
          "[short_circuit][iec60909][peak][method-a]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0),
                  make_bus(2, BusType::PQ, 20.0),
                  make_bus(3, BusType::PQ, 20.0)};
  ExternalGrid grid;
  grid.index = 1; grid.bus = 1; grid.r_pu = 0.05; grid.x_pu = 0.10;
  sys.ac.external_grids = {grid};
  sys.ac.branches = {make_branch(1, 1, 2, 0.04, 0.10),
                     make_branch(2, 2, 3, 0.005, 0.10)};

  SCDetailedOptions options;
  options.c_factor = 1.0;
  options.kappa_method = SCKappaMethod::A;
  options.topology = SCTopology::Radial;
  options.compute_branch_flows = false;
  options.compute_ith = false;
  const auto result = run_short_circuit_detailed(sys, 3, options);
  REQUIRE(result.solved);
  const auto& row = fault_row(result);
  const double expected_kappa = calculate_kappa_basic_sc(0.005 / 0.10);
  CHECK(row.ip_ka ==
        Catch::Approx(std::sqrt(2.0) * expected_kappa * row.ikss_ka)
            .epsilon(1e-12));

  const Cx equivalent(0.05 + 0.04 + 0.005, 0.10 + 0.10 + 0.10);
  const double wrong_equivalent_kappa = calculate_kappa_basic_sc(
      std::abs(equivalent.real() / equivalent.imag()));
  CHECK(std::abs(expected_kappa - wrong_equivalent_kappa) > 0.20);
}

TEST_CASE("SC steady current classifies IEC near and far generator faults",
          "[short_circuit][iec60909][steady][lambda]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0)};
  Generator generator;
  generator.index = 17; generator.bus = 1; generator.mbase_mva = 100.0;
  generator.vn_kv = 20.0; generator.xdpp_pu = 0.10;
  generator.sc_lambda_max = 2.20; generator.sc_lambda_min = 1.40;
  sys.ac.generators = {generator};

  SCDetailedOptions options;
  options.c_factor = 1.0;
  options.compute_branch_flows = false;
  options.compute_ith = false;
  auto maximum = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(maximum.solved);
  const double rated_ka = 100.0 / (std::sqrt(3.0) * 20.0);
  CHECK(fault_row(maximum).ik_ka ==
        Catch::Approx(2.20 * rated_ka).epsilon(1e-12));

  options.calc_type = SCCalcType::Min;
  auto minimum = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(minimum.solved);
  CHECK(fault_row(minimum).ik_ka ==
        Catch::Approx(1.40 * rated_ka).epsilon(1e-12));

  sys.ac.generators.front().sc_lambda_min = 0.0;
  const auto missing = run_short_circuit_detailed(sys, 1, options);
  CHECK_FALSE(missing.solved);
  CHECK(missing.status == "invalid_generator_data");

  sys.ac.generators.front().sc_terminal_fed_static_excitation = true;
  sys.ac.generators.front().sc_lambda_min = 1.40;
  const auto static_excitation = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(static_excitation.solved);
  CHECK(fault_row(static_excitation).ik_ka ==
        Catch::Approx(1.40 * rated_ka).epsilon(1e-12));

  options.calc_type = SCCalcType::Max;
  const auto static_excitation_max = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(static_excitation_max.solved);
  CHECK(fault_row(static_excitation_max).ik_ka ==
        Catch::Approx(1.40 * rated_ka).epsilon(1e-12));

  sys.ac.generators.front().sc_terminal_fed_static_excitation = false;
  sys.ac.generators.front().xdpp_pu = 1.0;
  options.calc_type = SCCalcType::Min;
  const auto far_fault = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(far_fault.solved);
  CHECK(fault_row(far_fault).ik_ka ==
        Catch::Approx(fault_row(far_fault).ikss_ka).epsilon(1e-12));
}

TEST_CASE("IEC Annex A thermal factors match analytic values and limits",
          "[short_circuit][iec60909][thermal][annex-a]") {
  const auto kappa_one = calculate_thermal_factors_sc(1.0, 10.0, 10.0, 50.0, 1.0);
  CHECK(kappa_one.m == 0.0);
  CHECK(kappa_one.n == 1.0);

  const auto kappa_two = calculate_thermal_factors_sc(2.0, 10.0, 10.0, 50.0, 1.0);
  CHECK(kappa_two.m == 2.0);
  CHECK(kappa_two.n == 1.0);

  const auto analytic = calculate_thermal_factors_sc(1.8, 10.0, 5.0, 50.0, 1.0);
  CHECK(analytic.m == Catch::Approx(0.04481420117724551).margin(1e-14));
  CHECK(analytic.n == Catch::Approx(0.5285589523006868).margin(1e-14));
  CHECK(10.0 * std::sqrt(analytic.m + analytic.n) ==
        Catch::Approx(7.572140737452866).margin(1e-13));

  const auto zero_steady = calculate_thermal_factors_sc(1.8, 10.0, 0.0, 50.0, 1.0);
  CHECK(std::isfinite(zero_steady.n));
  CHECK(zero_steady.n > 0.0);
  CHECK(zero_steady.n < 1.0);
}

TEST_CASE("IEC Annex A uses the peak factor produced by method C",
          "[short_circuit][iec60909][thermal][method-c]") {
  auto sys = make_sys({make_bus(1, BusType::SLACK, 20.0)}, {}, 100.0, 0.20);
  auto& generator = sys.ac.generators.front();
  generator.ra_pu = 0.20;
  generator.sc_lambda_max = 1.0;

  SCDetailedOptions options;
  options.c_factor = 1.0;
  options.kappa_method = SCKappaMethod::C;
  options.compute_branch_flows = false;
  options.compute_nonfault_currents = false;
  const auto result = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(result.solved);
  const auto& row = fault_row(result);

  const double effective_kappa =
      row.ip_ka / (std::sqrt(2.0) * row.ikss_ka);
  const auto expected = calculate_thermal_factors_sc(
      effective_kappa, row.ikss_ka, row.ik_ka,
      options.base_frequency_hz, options.ith_duration_s);
  const auto wrong_power_frequency = calculate_thermal_factors_sc(
      calculate_kappa_basic_sc(generator.ra_pu / generator.xdpp_pu),
      row.ikss_ka, row.ik_ka,
      options.base_frequency_hz, options.ith_duration_s);

  CHECK(row.thermal_m == Catch::Approx(expected.m).margin(1e-14));
  CHECK(row.thermal_n == Catch::Approx(expected.n).margin(1e-14));
  CHECK(row.ith_ka ==
        Catch::Approx(row.ikss_ka * std::sqrt(expected.m + expected.n))
            .margin(1e-13));
  CHECK(std::abs(row.thermal_m - wrong_power_frequency.m) > 1e-3);
}

TEST_CASE("SC steady current uses breaking current for IEC multiple-fed near faults",
          "[short_circuit][iec60909][steady][multiple-fed]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK, 20.0)};
  Generator generator;
  generator.index = 1; generator.bus = 1; generator.mbase_mva = 100.0;
  generator.vn_kv = 20.0; generator.xdpp_pu = 0.10;
  sys.ac.generators = {generator};
  ExternalGrid grid;
  grid.index = 1; grid.bus = 1; grid.r_pu = 0.01; grid.x_pu = 0.10;
  sys.ac.external_grids = {grid};

  SCDetailedOptions options;
  options.c_factor = 1.0;
  options.compute_branch_flows = false;
  options.compute_ith = false;
  const auto result = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(result.solved);
  const auto& row = fault_row(result);
  CHECK(row.ib_ka > 0.0);
  CHECK(row.ik_ka == Catch::Approx(row.ib_ka).epsilon(1e-12));

  AsynchronousMotor motor;
  motor.index = 1; motor.bus = 1; motor.vn_kv = 20.0;
  motor.sn_mva = 10.0; motor.cos_phi = 0.9; motor.efficiency = 0.95;
  motor.r_pu = 0.05; motor.x_pu = 0.20; motor.poles = 1;
  sys.ac.motors = {motor};
  const auto with_motor = run_short_circuit_detailed(sys, 1, options);
  REQUIRE(with_motor.solved);
  const auto& motor_row = fault_row(with_motor);
  CHECK(motor_row.ikss_motor_contrib_ka > 0.0);
  CHECK(motor_row.ik_ka > 0.0);
  CHECK(motor_row.ib_ka > motor_row.ik_ka);
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
    CHECK(r.z_thevenin.real() >= -1e-12);
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
          "[short_circuit][short_circuit_example]") {
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
    const std::string path = root + "/external_data/short_circuit_example/" + file;
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

TEST_CASE("SC: short_circuit_case bus4 matches classical reference",
          "[short_circuit][regression]") {
#ifdef HACDCPF_PROJECT_ROOT
  const std::string root = HACDCPF_PROJECT_ROOT;
#else
  const std::string root = ".";
#endif
  const std::string text =
      read_text_file(root + "/tests/short_circuit/short_circuit_case.json");
  const auto sys = hacdcpf::io::from_json(text);

  SCDetailedOptions opt;
  opt.compute_branch_flows = false;
  opt.compute_voltage_drops = false;
  opt.compute_ith = false;

  const auto result = run_short_circuit_detailed(sys, 4, opt);
  REQUIRE(result.solved);
  const auto& row = fault_row(result);

  INFO("ikss_ka=" << row.ikss_ka << " ip_ka=" << row.ip_ka
       << " ib_ka=" << row.ib_ka << " ik_ka=" << row.ik_ka
       << " motor=" << row.ikss_motor_contrib_ka
      << " no_motor=" << row.ikss_no_motor_ka
       << " extgrid=" << row.ikss_extgrid_contrib_ka);
  CHECK(std::abs(row.ikss_ka - 19.52) <= 0.05);
  CHECK(std::abs(row.ip_ka - 48.82) <= 0.10);
  CHECK(std::abs(row.ib_ka - 17.04) <= 0.05);
  CHECK(std::abs(row.ik_ka - 14.74) <= 0.05);
}

TEST_CASE("SC: practical classical examples satisfy stress invariants",
          "[short_circuit][short_circuit_example][practical]") {
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
    const std::string path = root + "/external_data/short_circuit_example/" + file;
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
