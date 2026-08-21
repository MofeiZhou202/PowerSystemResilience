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

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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

namespace {

HybridPowerSystem make_iec_60909_4_comprehensive_case() {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 34.0;
  const std::array<double, 13> kv = {
      380.0, 110.0, 110.0, 110.0, 110.0, 10.0, 10.0,
      30.0, 20.0, 10.0, 10.0, 10.0, 30.0};
  for (size_t i = 0; i < kv.size(); ++i) {
    ACBus bus;
    bus.index = static_cast<int>(i) + 1;
    bus.base_kv = kv[i];
    bus.bus_type = i == 0 ? BusType::SLACK : BusType::PQ;
    sys.ac.buses.push_back(bus);
  }

  const auto add_grid = [&](int index, int bus, double ssc, double rx,
                            double x0x, double r0x0) {
    ExternalGrid grid;
    grid.index = index;
    grid.bus = bus;
    grid.s_sc_max_mva = ssc;
    grid.s_sc_min_mva = ssc / 10.0;
    grid.rx_max = grid.rx_min = rx;
    const double z1 = 1.1 * sys.base_mva / ssc;
    const double x1 = z1 / std::sqrt(1.0 + rx * rx);
    grid.x0_pu = x0x * x1;
    grid.r0_pu = r0x0 * grid.x0_pu;
    sys.ac.external_grids.push_back(grid);
  };
  add_grid(1, 1, 38.0 * 380.0 * std::sqrt(3.0), 0.1, 3.0, 0.15);
  add_grid(2, 5, 16.0 * 110.0 * std::sqrt(3.0), 0.1, 3.3, 0.2);

  const auto add_line = [&](int index, int from, int to, double length,
                            double r1, double x1, double r0, double x0) {
    ACBranch line;
    line.index = index;
    line.from_bus = from;
    line.to_bus = to;
    line.length_km = length;
    const double zbase = std::pow(kv[static_cast<size_t>(from - 1)], 2.0) /
                         sys.base_mva;
    line.r_pu = r1 * length / zbase;
    line.x_pu = x1 * length / zbase;
    line.r0_pu = r0 * length / zbase;
    line.x0_pu = x0 * length / zbase;
    line.sc_r_reference_temperature_c = 20.0;
    line.sc_end_temperature_c = 80.0;
    line.sc_alpha_per_c = 0.004;
    sys.ac.branches.push_back(line);
  };
  add_line(1, 2, 3, 20.0, 0.120, 0.390, 0.320, 1.260);
  add_line(2, 3, 4, 10.0, 0.120, 0.390, 0.320, 1.260);
  add_line(3, 2, 5, 5.0, 0.120, 0.390, 0.520, 1.860);
  add_line(4, 2, 5, 5.0, 0.120, 0.390, 0.520, 1.860);
  add_line(5, 5, 3, 10.0, 0.096, 0.388, 0.220, 1.100);
  add_line(6, 5, 4, 15.0, 0.120, 0.386, 0.220, 1.100);
  add_line(7, 6, 7, 1.0, 0.082, 0.086, 0.082, 0.086);

  Transformer2W t1;
  t1.index = 1; t1.hv_bus = 4; t1.lv_bus = 9; t1.sn_mva = 150.0;
  t1.vn_hv_kv = 115.0; t1.vn_lv_kv = 21.0;
  t1.vk_percent = 16.0; t1.vkr_percent = 0.5;
  t1.z0_percent = 15.2;
  t1.x0_r0 = std::sqrt(15.2 * 15.2 - 0.5 * 0.5) / 0.5;
  t1.mag0_percent = 100.0; t1.mag0_rx = 0.0; t1.si0_hv_partial = 0.5;
  t1.xn_ohm = 22.0; t1.vector_group = "YNd";
  t1.power_station_unit = true; t1.oltc = true; t1.pt_percent = 12.0;
  sys.ac.transformers_2w.push_back(t1);

  Transformer2W t2;
  t2.index = 2; t2.hv_bus = 3; t2.lv_bus = 10; t2.sn_mva = 100.0;
  t2.vn_hv_kv = 120.0; t2.vn_lv_kv = 10.5;
  t2.vk_percent = 12.0; t2.vkr_percent = 0.5;
  t2.z0_percent = 12.0;
  t2.x0_r0 = std::sqrt(12.0 * 12.0 - 0.5 * 0.5) / 0.5;
  t2.mag0_percent = 100.0; t2.vector_group = "Yd";
  t2.power_station_unit = true; t2.oltc = false;
  sys.ac.transformers_2w.push_back(t2);

  const auto add_3w = [&](int index, int hv, int mv, int lv,
                          double sh, double sm, double sl,
                          double vh, double vm, double vl,
                          double vkhm, double vkml, double vkhl,
                          double vkrhm, double vkrml, double vkrhl,
                          double vk0hm, double vk0ml, double vk0hl,
                          double vkr0hm, double vkr0ml, double vkr0hl,
                          const std::string& group) {
    Transformer3W transformer;
    transformer.index = index;
    transformer.hv_bus = hv; transformer.mv_bus = mv; transformer.lv_bus = lv;
    transformer.sn_hv_mva = sh; transformer.sn_mv_mva = sm;
    transformer.sn_lv_mva = sl;
    transformer.vn_hv_kv = vh; transformer.vn_mv_kv = vm;
    transformer.vn_lv_kv = vl;
    transformer.vk_hv_mv_percent = vkhm;
    transformer.vk_mv_lv_percent = vkml;
    transformer.vk_hv_lv_percent = vkhl;
    transformer.vkr_hv_mv_percent = vkrhm;
    transformer.vkr_mv_lv_percent = vkrml;
    transformer.vkr_hv_lv_percent = vkrhl;
    transformer.vk0_hv_mv_percent = vk0hm;
    transformer.vk0_mv_lv_percent = vk0ml;
    transformer.vk0_hv_lv_percent = vk0hl;
    transformer.vkr0_hv_mv_percent = vkr0hm;
    transformer.vkr0_mv_lv_percent = vkr0ml;
    transformer.vkr0_hv_lv_percent = vkr0hl;
    transformer.vector_group = group;
    sys.ac.transformers_3w.push_back(transformer);
  };
  add_3w(1, 1, 2, 13, 350, 350, 50, 400, 120, 30,
         21, 7, 10, .26, .16, .16,
         44.1, 6.299627, 6.299627, .26, .03714286, .03714286, "YNyd");
  add_3w(2, 1, 2, 8, 350, 350, 50, 400, 120, 30,
         21, 7, 10, .26, .16, .16,
         44.1, 6.299627, 6.299627, .26, .03714286, .03714286, "Yynd");
  add_3w(3, 5, 6, 11, 31.5, 31.5, 31.5, 115, 10.5, 10.5,
         12, 12, 12, .5, .5, .5,
         12, 12, 12, .5, .5, .5, "Yyd");
  add_3w(4, 5, 6, 12, 31.5, 31.5, 31.5, 115, 10.5, 10.5,
         12, 12, 12, .5, .5, .5,
         12, 12, 12, .5, .5, .5, "Yynd");

  const auto add_generator = [&](int index, int bus, double sn, double vn,
                                 double xdss, double rdss_ohm,
                                 double cos_phi, double pg_percent,
                                 int station_transformer) {
    Generator generator;
    generator.index = index; generator.bus = bus;
    generator.mbase_mva = sn; generator.vn_kv = vn;
    generator.xdpp_pu = xdss; generator.xd_pu = xdss;
    generator.ra_pu = rdss_ohm / (vn * vn / sn);
    generator.cos_phi = cos_phi; generator.pg_percent = pg_percent;
    generator.power_station_transformer_index = station_transformer;
    sys.ac.generators.push_back(generator);
  };
  add_generator(1, 9, 150, 21, .14, .002, .85, 0.0, 1);
  add_generator(2, 10, 100, 10.5, .16, .005, .90, 7.5, 2);
  add_generator(3, 6, 10, 10.5, .10, .018, .80, 0.0, 0);

  const auto add_motor = [&](int index, double mechanical_mw, double cos_phi,
                             double efficiency, double lrc) {
    AsynchronousMotor motor;
    motor.index = index; motor.bus = 7; motor.vn_kv = 10.0;
    motor.cos_phi = cos_phi; motor.efficiency = efficiency;
    motor.sn_mva = mechanical_mw / (cos_phi * efficiency);
    const double z = 1.0 / lrc;
    motor.r_pu = z / std::sqrt(1.0 + 10.0 * 10.0);
    motor.x_pu = 10.0 * motor.r_pu;
    motor.x_r = 10.0; motor.lrc = lrc;
    motor.poles = 1;
    sys.ac.motors.push_back(motor);
  };
  add_motor(1, 5.0, .88, .975, 5.0);
  add_motor(2, 2.0, .89, .968, 5.2);
  add_motor(3, 2.0, .89, .968, 5.2);
  return sys;
}

void check_relative_errors(const std::vector<SCDetailedResult>& actual,
                           const std::array<double, 13>& expected,
                           bool peak,
                           double threshold) {
  for (size_t i = 0; i < expected.size(); ++i) {
    REQUIRE(actual[i].solved);
    const auto& row = fault_row(actual[i], static_cast<int>(i) + 1);
    const double value = peak ? row.ip_ka : row.ikss_ka;
    const double absolute = std::abs(value - expected[i]);
    const double relative = absolute / std::max(1e-9, std::abs(expected[i]));
    INFO("bus=" << i + 1 << " actual=" << value
         << " expected=" << expected[i] << " relative=" << relative);
    // pandapower 2.14.10 regularizes every generator zero-sequence port with
    // 1/(1000+j1000) pu (pd2ppc_zero.py::_add_gen_sc_impedance_zero). Values
    // below 1 A are therefore classified as regularization markers rather
    // than compared as physical fault levels. HySim must retain the exact
    // open-circuit result to its independently fixed 1e-6 kA absolute gate.
    if (std::abs(expected[i]) < 1e-3) {
      CHECK(std::abs(value) <= 1e-6);
      CHECK(std::abs(expected[i]) <= 1e-3);
    } else {
      CHECK(relative <= threshold);
    }
  }
}

template <size_t N, typename ValueAt>
void check_prefix_relative_errors(const std::vector<SCDetailedResult>& actual,
                                  const std::array<double, N>& expected,
                                  ValueAt value_at,
                                  double threshold) {
  REQUIRE(actual.size() >= N);
  for (size_t i = 0; i < N; ++i) {
    REQUIRE(actual[i].solved);
    const auto& row = fault_row(actual[i], static_cast<int>(i) + 1);
    const double value = value_at(row);
    const double relative = std::abs(value - expected[i]) /
                            std::max(1e-9, std::abs(expected[i]));
    INFO("bus=" << i + 1 << " actual=" << value
         << " expected=" << expected[i] << " relative=" << relative);
    CHECK(relative <= threshold);
  }
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

TEST_CASE("IEC TR 60909-4 comprehensive 13-bus network",
          "[short_circuit][iec60909][comprehensive]") {
  const auto sys = make_iec_60909_4_comprehensive_case();
  std::vector<int> buses(13);
  for (int i = 0; i < 13; ++i) buses[static_cast<size_t>(i)] = i + 1;

  const std::array<double, 13> three_phase_max = {
      40.6447259811, 31.7830591047, 19.6729558157, 16.2276556883,
      33.1894126354, 37.5628811566, 25.5894647188, 13.5777716428,
      52.4438061352, 80.5720479721, 17.6156665149, 17.6156665149,
      13.5777716428};
  const std::array<double, 13> three_phase_ip = {
      100.5676605747, 80.6079433370, 45.8111162558, 36.8427167202,
      83.4033103072, 98.1433638007, 51.6899411400, 36.9226760533,
      136.2801473221, 210.3158972170, 46.7402433296, 46.7402433296,
      36.9226760533};
  const std::array<double, 13> three_phase_min = {
      5.0501073652, 12.2914673305, 10.3292496199, 9.4708367964,
      11.8603822938, 28.3052212000, 18.6148299209, 10.9005098927,
      44.5097690261, 67.9577865704, 14.8799200986, 14.8799200986,
      10.9005098927};
  const std::array<double, 13> two_phase_max = {
      35.1993652295, 27.5249365946, 17.0372795039, 14.0535620700,
      28.7428744789, 32.5304093210, 22.1611265157, 11.7586951695,
      45.4176683842, 69.7774403788, 15.2556147065, 15.2556147065,
      11.7586951695};
  const std::array<double, 13> single_phase_max = {
      24.6526144498, 15.9722156015, 10.4105643084, 9.0497970920,
      17.0451996437, 25.4854578112, 19.0489620695, 0.0,
      0.0000673609, 0.0001347218, 0.0, 0.0, 0.0};
  const std::array<double, 10> three_phase_breaking = {
      40.6450, 31.5700, 19.3880, 16.0170, 32.7950,
      34.0280, 23.2120, 13.5780, 42.3867, 68.4172};
  const std::array<double, 10> two_phase_ip = {
      87.0941, 69.8085, 39.6736, 31.9067, 72.2294,
      84.9946, 44.7648, 31.9760, 118.0221, 182.1389};
  const std::array<double, 5> single_phase_ip_grounded = {
      60.9982, 40.5086, 24.2424, 20.5464, 42.8337};
  const std::array<double, 8> no_motor_ikss = {
      40.6347, 31.6635, 19.6231, 16.1956,
      32.9971, 34.3559, 22.2762, 13.5726};
  const std::array<double, 8> no_motor_ip = {
      100.5427, 80.3509, 45.7157, 36.7855,
      82.9406, 90.6143, 43.3826, 36.9103};

  SCDetailedOptions options;
  options.compute_branch_flows = false;
  options.compute_voltage_drops = false;
  options.compute_nonfault_currents = false;
  options.compute_ith = false;

  options.fault_type = FaultType::ThreePhase;
  options.calc_type = SCCalcType::Max;
  const auto max3 = run_short_circuit_detailed_batch(sys, buses, options);
  check_relative_errors(max3, three_phase_max, false, 0.005);
  check_relative_errors(max3, three_phase_ip, true, 0.02);
  check_prefix_relative_errors(
      max3, three_phase_breaking,
      [](const SCDetailedBusResult& row) { return row.ib_ka; }, 0.005);

  options.calc_type = SCCalcType::Min;
  const auto min3 = run_short_circuit_detailed_batch(sys, buses, options);
  check_relative_errors(min3, three_phase_min, false, 0.005);

  options.fault_type = FaultType::TwoPhase;
  options.calc_type = SCCalcType::Max;
  const auto max2 = run_short_circuit_detailed_batch(sys, buses, options);
  check_relative_errors(max2, two_phase_max, false, 0.005);
  check_prefix_relative_errors(
      max2, two_phase_ip,
      [](const SCDetailedBusResult& row) { return row.ip_ka; }, 0.02);

  options.fault_type = FaultType::SinglePhaseGround;
  const auto max1 = run_short_circuit_detailed_batch(sys, buses, options);
  check_relative_errors(max1, single_phase_max, false, 0.005);
  check_prefix_relative_errors(
      max1, single_phase_ip_grounded,
      [](const SCDetailedBusResult& row) { return row.ip_ka; }, 0.02);

  auto no_motor = sys;
  no_motor.ac.motors.clear();
  options.fault_type = FaultType::ThreePhase;
  const auto max3_no_motor = run_short_circuit_detailed_batch(no_motor, buses, options);
  check_prefix_relative_errors(
      max3_no_motor, no_motor_ikss,
      [](const SCDetailedBusResult& row) { return row.ikss_ka; }, 0.005);
  check_prefix_relative_errors(
      max3_no_motor, no_motor_ip,
      [](const SCDetailedBusResult& row) { return row.ip_ka; }, 0.02);
}
