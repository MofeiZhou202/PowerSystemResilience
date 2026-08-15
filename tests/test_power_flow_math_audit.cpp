#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/power_flow/ncp_functions.hpp"
#include "hacdcpf/power_flow/residual_evaluator.hpp"
#include "hacdcpf/power_flow/solver_factory.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/distributed_slack_solver.hpp"
#include "hacdcpf/power_flow/adaptive_solver.hpp"
#include "hacdcpf/power_flow/dc_solver.hpp"
#include "hacdcpf/power_flow/ac_kernel_impl.hpp"
#include "hacdcpf/power_flow/fdpf_solver.hpp"
#include "hacdcpf/power_flow/helm_solver.hpp"
#include "hacdcpf/power_flow/lcc_model.hpp"
#include "hacdcpf/power_flow/lm_trust_region.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/nonmonotone_linesearch.hpp"
#include "hacdcpf/power_flow/jacobian_check.hpp"
#include "hacdcpf/power_flow/assembly/pf_injection_assembly.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/assembly/ybus_builder.hpp"

namespace {

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Catch::Approx;

constexpr double kPi = 3.14159265358979323846;

double wrapped_angle_error_deg(double actual, double expected) {
  double error = std::fmod(actual - expected, 360.0);
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;
  return std::abs(error);
}

ThreePhaseACBus make_three_phase_bus(int index, BusType type) {
  ThreePhaseACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.phase_mask = PhaseMask::abc();
  bus.base_kv = 12.47;
  bus.vm_a_pu = 1.0;
  bus.va_a_deg = 0.0;
  bus.vm_b_pu = 1.0;
  bus.va_b_deg = -120.0;
  bus.vm_c_pu = 1.0;
  bus.va_c_deg = 120.0;
  bus.in_service = true;
  return bus;
}

ThreePhaseTransformer make_transformer(const std::string& vector_group) {
  ThreePhaseTransformer transformer;
  transformer.index = 1;
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.hv_phase_mask = PhaseMask::abc();
  transformer.lv_phase_mask = PhaseMask::abc();
  transformer.vector_group = vector_group;
  transformer.sn_mva = 10.0;
  transformer.vn_hv_kv = 12.47;
  transformer.vn_lv_kv = 12.47;
  transformer.vk_percent = 8.0;
  transformer.vkr_percent = 0.5;
  transformer.in_service = true;
  return transformer;
}

}  // namespace

#ifdef HACDCPF_HAVE_OPENDSS
namespace {

class TemporaryDSSFixture {
 public:
  TemporaryDSSFixture(const std::string& name, const std::string& contents) {
    directory_ = std::filesystem::temp_directory_path() /
                 ("hacdcpf_power_flow_math_audit_" + name);
    std::filesystem::create_directories(directory_);
    master_ = directory_ / "Master.dss";
    std::ofstream output(master_);
    if (!output) {
      throw std::runtime_error("failed to create temporary OpenDSS fixture");
    }
    output << contents;
  }

  ~TemporaryDSSFixture() {
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
  }

  const std::filesystem::path& master() const { return master_; }

 private:
  std::filesystem::path directory_;
  std::filesystem::path master_;
};

}  // namespace
#endif

TEST_CASE("Audit A2: fixed-point Norton source uses the KCL source sign",
          "[power_flow][math_audit][A2]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  auto bus = make_three_phase_bus(1, BusType::PQ);
  bus.pd_a_mw = 0.10;
  bus.pd_b_mw = 0.10;
  bus.pd_c_mw = 0.10;
  sys.buses = {bus};

  ThreePhaseExternalGrid source;
  source.index = 1;
  source.bus = 1;
  source.phase_mask = PhaseMask::abc();
  source.vm_pu = 1.0;
  source.va_deg = 0.0;
  source.r1_pu = 0.02;
  source.x1_pu = 0.08;
  source.r2_pu = 0.02;
  source.x2_pu = 0.08;
  source.r0_pu = 0.06;
  source.x0_pu = 0.24;
  source.in_service = true;
  sys.external_grids = {source};

  ThreePhaseFixedPointOptions options;
  options.max_iter = 300;
  options.tol = 1e-10;
  const auto compact = solve_three_phase_compact_pf(sys, options);
  const auto fixed = solve_three_phase_fixed_point(sys, options);
  REQUIRE(compact.converged);
  REQUIRE(fixed.converged);
  REQUIRE(compact.bus_voltages.size() == 1);
  REQUIRE(fixed.bus_voltages.size() == 1);

  const auto& cv = compact.bus_voltages.front();
  const auto& fv = fixed.bus_voltages.front();
  CHECK(cv.vm_a_pu > 0.95);
  CHECK(wrapped_angle_error_deg(cv.va_a_deg, 0.0) < 2.0);
  CHECK(wrapped_angle_error_deg(cv.va_b_deg, -120.0) < 2.0);
  CHECK(wrapped_angle_error_deg(cv.va_c_deg, 120.0) < 2.0);
  CHECK(fv.vm_a_pu == Approx(cv.vm_a_pu).margin(1e-7));
  CHECK(wrapped_angle_error_deg(fv.va_a_deg, cv.va_a_deg) < 1e-5);
}

TEST_CASE("Audit A3: phase-shifting linearized DC flow satisfies nodal balance",
          "[power_flow][math_audit][A3]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 1.0;
  slack.in_service = true;
  ACBus load;
  load.index = 2;
  load.bus_type = BusType::PQ;
  load.pd_mw = 20.0;
  load.in_service = true;
  sys.ac.buses = {slack, load};

  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.x_pu = 0.10;
  branch.tap = 1.0;
  branch.shift_deg = 10.0;
  branch.in_service = true;
  sys.ac.branches = {branch};

  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.in_service = true;
  sys.ac.generators = {generator};

  const auto result = solve_ac_dc_power_flow(sys);
  REQUIRE(result.success);
  REQUIRE(result.pf_mw.size() == 1);
  CHECK(result.pf_mw.front() == Approx(20.0).margin(1e-10));
}

TEST_CASE("Audit A4/B15: BFS fail-closes on disconnected and meshed networks",
          "[power_flow][math_audit][A4][B15]") {
  ACSystem ac;
  ac.base_mva = 100.0;
  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.in_service = true;
  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.in_service = true;
  ACBus b3 = b2;
  b3.index = 3;
  b3.pd_mw = 5.0;
  ac.buses = {b1, b2, b3};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.01;
  branch.x_pu = 0.05;
  branch.in_service = true;
  ac.branches = {branch};

  const auto disconnected = solve_distribution_pf(ac);
  CHECK_FALSE(disconnected.converged);
  CHECK(disconnected.specialized_path_fail_close_reason ==
        "bfs_requires_one_connected_component");
  CHECK(std::isinf(disconnected.residual));

  ACBranch b23 = branch;
  b23.index = 2;
  b23.from_bus = 2;
  b23.to_bus = 3;
  ACBranch b31 = branch;
  b31.index = 3;
  b31.from_bus = 3;
  b31.to_bus = 1;
  ac.branches = {branch, b23, b31};
  const auto meshed = solve_distribution_pf(ac);
  CHECK_FALSE(meshed.converged);
  CHECK(meshed.specialized_path_fail_close_reason ==
        "bfs_requires_radial_topology");
}

TEST_CASE("Audit B15: BFS uses additive scaled loads and voltage-dependent shunts",
          "[power_flow][math_audit][B15]") {
  ACSystem ac;
  ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 1.0;
  slack.in_service = true;
  ACBus load_bus;
  load_bus.index = 2;
  load_bus.bus_type = BusType::PQ;
  load_bus.pd_mw = 1.0;
  load_bus.bs_mvar = 4.0;
  load_bus.in_service = true;
  ac.buses = {slack, load_bus};

  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.01;
  branch.x_pu = 0.04;
  branch.in_service = true;
  ac.branches = {branch};

  Load component_load;
  component_load.index = 1;
  component_load.bus = 2;
  component_load.p_mw = 2.0;
  component_load.q_mvar = 1.0;
  component_load.scaling = 0.5;
  component_load.in_service = true;
  ac.loads = {component_load};

  DPFOptions options;
  options.tol = 1e-12;
  options.max_iter = 200;
  const auto result = solve_distribution_pf(ac, options);
  REQUIRE(result.converged);
  REQUIRE(result.vm_pu.size() == 2);

  const double vm2 = result.vm_pu[1] * result.vm_pu[1];
  const double expected_p_demand = 1.0 + 2.0 * 0.5;
  const double expected_q_demand = 1.0 * 0.5 - 4.0 * vm2;
  CHECK(result.p_branch_mw[0] - result.total_p_loss_mw ==
        Approx(expected_p_demand).margin(2e-8));
  CHECK(result.q_branch_mvar[0] - result.total_q_loss_mvar ==
        Approx(expected_q_demand).margin(2e-8));
}

TEST_CASE("Audit B15: BFS line charging satisfies receiving-end pi-model KCL",
          "[power_flow][math_audit][B15]") {
  ACSystem ac;
  ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 1.0;
  slack.in_service = true;
  ACBus pq;
  pq.index = 2;
  pq.bus_type = BusType::PQ;
  pq.vm_pu = 1.0;
  pq.in_service = true;
  ac.buses = {slack, pq};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.01;
  branch.x_pu = 0.10;
  branch.b_pu = 0.20;
  branch.in_service = true;
  ac.branches = {branch};

  DPFOptions options;
  options.tol = 1e-13;
  options.max_iter = 300;
  const auto result = solve_distribution_pf(ac, options);
  REQUIRE(result.converged);
  const std::complex<double> v_from =
      std::polar(result.vm_pu[0], result.va_deg[0] * kPi / 180.0);
  const std::complex<double> v_to =
      std::polar(result.vm_pu[1], result.va_deg[1] * kPi / 180.0);
  const std::complex<double> z(branch.r_pu, branch.x_pu);
  const std::complex<double> y_half(0.0, branch.b_pu / 2.0);
  const std::complex<double> current_to = -(v_from - v_to) / z + y_half * v_to;
  CHECK(std::abs(current_to) < 2e-11);
  CHECK(std::abs(result.q_branch_mvar[0]) > 1.0);
}

TEST_CASE("Audit B15: three-phase BFS fail-closes on unsupported phase topology",
          "[power_flow][math_audit][B15]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  auto slack = make_three_phase_bus(1, BusType::SLACK);
  auto load = make_three_phase_bus(2, BusType::PQ);
  load.phase_mask = PhaseMask::a();
  sys.buses = {slack, load};
  ThreePhaseACLine line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.phase_mask = PhaseMask::a();
  line.r1_pu = 0.01;
  line.x1_pu = 0.05;
  line.in_service = true;
  sys.lines = {line};

  const auto result = solve_three_phase_distribution_pf(sys);
  CHECK_FALSE(result.converged);
  CHECK(std::isinf(result.residual));
  CHECK(result.primary_solver_failed_reason ==
        "bfs_requires_full_abc_phase_masks_use_three_phase_nr");
}

#ifdef HACDCPF_HAVE_OPENDSS
TEST_CASE("Audit A5: OpenDSS import establishes voltage bases itself",
          "[power_flow][math_audit][A5][opendss]") {
  TemporaryDSSFixture fixture(
      "a5_voltage_bases",
      "Clear\n"
      "New Circuit.a5 phases=3 bus1=source.1.2.3 basekv=12.47 pu=1.0\n"
      "New Line.l1 phases=3 bus1=source.1.2.3 bus2=load.1.2.3 "
      "r1=0.1 x1=0.2 r0=0.3 x0=0.6 length=1 units=km\n"
      "New Load.ld phases=3 bus1=load.1.2.3 conn=wye kv=7.2 "
      "kw=300 kvar=90\n"
      "Set ControlMode=Off\n"
      "Solve\n");

  const auto system =
      load_three_phase_system_from_opendss(fixture.master(), 10.0);
  REQUIRE(system.buses.size() == 2);
  REQUIRE(system.lines.size() == 1);
  CHECK(system.buses[0].base_kv == Approx(12.47).margin(1e-10));
  CHECK(system.buses[1].base_kv == Approx(12.47).margin(1e-10));
  const double expected_r1_pu = 0.1 / (12.47 * 12.47 / 10.0);
  CHECK(system.lines.front().r1_pu == Approx(expected_r1_pu).margin(1e-10));
}

TEST_CASE("Audit A6: OpenDSS neutral node cannot overwrite phase A",
          "[power_flow][math_audit][A6][opendss]") {
  TemporaryDSSFixture fixture(
      "a6_neutral_node",
      "Clear\n"
      "New Circuit.a6 phases=1 bus1=source.1 basekv=7.2 pu=1.0\n"
      "New Line.l1 phases=2 bus1=source.1.0 bus2=load.1.4 "
      "rmatrix=[0.1 | 0.01 0.1] xmatrix=[0.2 | 0.02 0.2] "
      "cmatrix=[0 | 0 0] length=1 units=km\n"
      "New Load.ld phases=1 bus1=load.1.4 conn=wye kv=7.2 "
      "kw=100 kvar=30\n"
      "Set VoltageBases=[7.2]\n"
      "CalcVoltageBases\n"
      "Set ControlMode=Off\n"
      "Solve\n");

  ThreePhaseACSystem system;
  system.base_mva = 1.0;
  auto source = make_three_phase_bus(1, BusType::SLACK);
  source.name = "source";
  source.phase_mask = PhaseMask::a();
  source.base_kv = 7.2;
  auto load = make_three_phase_bus(2, BusType::PQ);
  load.name = "load";
  load.phase_mask = PhaseMask::a();
  load.base_kv = 7.2;
  system.buses = {source, load};
  ThreePhaseACLine native_line;
  native_line.index = 1;
  native_line.from_bus = 1;
  native_line.to_bus = 2;
  native_line.phase_mask = PhaseMask::a();
  native_line.r1_pu = 0.1 / (7.2 * 7.2);
  native_line.x1_pu = 0.2 / (7.2 * 7.2);
  native_line.r0_pu = native_line.r1_pu;
  native_line.x0_pu = native_line.x1_pu;
  native_line.in_service = true;
  system.lines = {native_line};
  RunPFPhaseOptions options;
  options.algorithm = PhaseDomainSolverAlgorithm::OpenDSS;
  options.dss_file_path = fixture.master();
  options.max_iter = 100;
  options.tol = 1e-9;
  const auto result = runpf_phase(system, options);
  REQUIRE(result.success);
  const auto load_bus = std::find_if(
      result.bus_abc.begin(), result.bus_abc.end(),
      [](const PhaseDomainBusRow& bus) {
        return bus.bus_id != 0 && bus.bus_type != BusType::SLACK;
      });
  REQUIRE(load_bus != result.bus_abc.end());
  CHECK(load_bus->has_phase[0]);
  CHECK(load_bus->vm_pu[0] > 0.95);
  CHECK_FALSE(load_bus->has_phase[1]);
  CHECK_FALSE(load_bus->has_phase[2]);
}

TEST_CASE("Audit B16: OpenDSS floating neutral and delta capacitor retain topology",
          "[power_flow][math_audit][B16][opendss]") {
  TemporaryDSSFixture fixture(
      "b16_neutral_capacitor",
      "Clear\n"
      "New Circuit.b16 phases=3 bus1=source.1.2.3 basekv=12.47 pu=1.0\n"
      "New Line.l1 phases=3 bus1=source.1.2.3 bus2=loadbus.1.2.3 "
      "r1=0.1 x1=0.2 r0=0.3 x0=0.6 length=1 units=km\n"
      "New Load.floating phases=3 bus1=loadbus.1.2.3 conn=wye kv=7.2 "
      "kw=300 kvar=90 rneut=-1 xneut=0\n"
      "New Capacitor.delta_cap phases=3 bus1=loadbus.1.2.3 conn=delta "
      "kv=12.47 kvar=900\n"
      "Set VoltageBases=[12.47]\n"
      "CalcVoltageBases\n"
      "Set ControlMode=Off\n"
      "Solve\n");

  const auto system = load_three_phase_system_from_opendss(fixture.master(), 10.0);
  const auto floating = std::find_if(
      system.loads.begin(), system.loads.end(),
      [](const ThreePhaseLoad& load) { return load.name == "floating"; });
  REQUIRE(floating != system.loads.end());
  CHECK_FALSE(floating->grounded);

  const auto capacitor = std::find_if(
      system.loads.begin(), system.loads.end(),
      [](const ThreePhaseLoad& load) { return load.name == "capacitor.delta_cap"; });
  REQUIRE(capacitor != system.loads.end());
  CHECK(capacitor->connection == "delta");
  CHECK(capacitor->const_z_percent == Approx(100.0));
  CHECK(capacitor->q_a_mvar == Approx(-0.3).margin(1e-10));
  CHECK(capacitor->q_b_mvar == Approx(-0.3).margin(1e-10));
  CHECK(capacitor->q_c_mvar == Approx(-0.3).margin(1e-10));
  const auto load_bus = std::find_if(
      system.buses.begin(), system.buses.end(),
      [](const ThreePhaseACBus& bus) { return bus.name == "loadbus"; });
  REQUIRE(load_bus != system.buses.end());
  CHECK(load_bus->bs_a_mvar == Approx(0.0).margin(1e-12));
  CHECK(load_bus->bs_b_mvar == Approx(0.0).margin(1e-12));
  CHECK(load_bus->bs_c_mvar == Approx(0.0).margin(1e-12));
}

TEST_CASE("Audit B16: OpenDSS transformer bases and signed star legs are preserved",
          "[power_flow][math_audit][B16][opendss]") {
  TemporaryDSSFixture fixture(
      "b16_transformer_bases",
      "Clear\n"
      "New Circuit.b16xf phases=3 bus1=source.1.2.3 basekv=12.47 pu=1.0\n"
      "New Transformer.t3 phases=3 windings=3 "
      "buses=[source.1.2.3,mv.1.2.3,lv.1.2.3] "
      "conns=[wye,wye,wye] kvs=[12.47,4.16,0.48] "
      "kvas=[10000,10000,5000] %rs=[0.2,0.2,0.2] "
      "xhl=4 xht=4 xlt=10\n"
      "New Transformer.t2 phases=3 windings=2 "
      "buses=[source.1.2.3,aux.1.2.3] conns=[wye,wye] "
      "kvs=[12.47,4.16] kvas=[10000,5000] %rs=[0.2,0.2] xhl=6\n"
      "Set VoltageBases=[12.47,4.16,0.48]\n"
      "CalcVoltageBases\n"
      "Set ControlMode=Off\n"
      "Solve\n");

  const auto system = load_three_phase_system_from_opendss(fixture.master(), 10.0);
  for (const std::string name : {"t3_hv", "t3_mv", "t3_lv"}) {
    const auto leg = std::find_if(
        system.transformers.begin(), system.transformers.end(),
        [&](const ThreePhaseTransformer& transformer) {
          return transformer.name == name;
        });
    REQUIRE(leg != system.transformers.end());
    CHECK(leg->sn_mva == Approx(10.0).margin(1e-12));
    CHECK(leg->use_signed_series_impedance);
  }
  const auto hv_leg = std::find_if(
      system.transformers.begin(), system.transformers.end(),
      [](const ThreePhaseTransformer& transformer) {
        return transformer.name == "t3_hv";
      });
  REQUIRE(hv_leg != system.transformers.end());
  CHECK(hv_leg->signed_series_x_percent == Approx(-1.0).margin(1e-10));

  const auto two_winding = std::find_if(
      system.transformers.begin(), system.transformers.end(),
      [](const ThreePhaseTransformer& transformer) {
        return transformer.name == "t2";
      });
  REQUIRE(two_winding != system.transformers.end());
  CHECK(two_winding->sn_mva == Approx(10.0).margin(1e-12));
}

TEST_CASE("Audit B16: three-winding RegControl fails closed instead of disappearing",
          "[power_flow][math_audit][B16][opendss]") {
  TemporaryDSSFixture fixture(
      "b16_three_winding_regcontrol",
      "Clear\n"
      "New Circuit.b16reg phases=3 bus1=source.1.2.3 basekv=12.47 pu=1.0\n"
      "New Transformer.t3 phases=3 windings=3 "
      "buses=[source.1.2.3,mv.1.2.3,lv.1.2.3] "
      "conns=[wye,wye,wye] kvs=[12.47,4.16,0.48] "
      "kvas=[10000,10000,5000] %rs=[0.2,0.2,0.2] "
      "xhl=6 xht=8 xlt=10\n"
      "New RegControl.r3 transformer=t3 winding=2 vreg=120 band=2 ptratio=34.67\n"
      "Set VoltageBases=[12.47,4.16,0.48]\n"
      "CalcVoltageBases\n"
      "Set ControlMode=Off\n"
      "Solve\n");

  CHECK_THROWS_WITH(
      load_three_phase_system_from_opendss(fixture.master(), 10.0),
      Catch::Matchers::ContainsSubstring("dynamic three-winding tap controls are not supported"));
}

TEST_CASE("Audit C22/C23: DSS inline comments cannot override dual fixed taps",
          "[power_flow][math_audit][C22][C23][opendss]") {
  TemporaryDSSFixture fixture(
      "c22_c23_fixed_taps",
      "Clear\n"
      "New Circuit.c22c23 phases=3 bus1=source.1.2.3 basekv=12.47 pu=1.0\n"
      "New Transformer.tfix phases=3 windings=2 "
      "buses=[source.1.2.3,load.1.2.3] conns=[wye,wye] "
      "kvs=[12.47,4.16] kvas=[10000,10000] %rs=[0.2,0.2] xhl=6\n"
      "~ wdg=1 tap=1.05 mintap=1.05 maxtap=1.05 numtaps=0\n"
      "~ wdg=2 tap=0.98 mintap=0.98 maxtap=0.98 numtaps=0 "
      "! tap=1.25 must remain a comment\n"
      "New Load.ld bus1=load.1.2.3 phases=3 conn=wye kv=4.16 kw=300 kvar=100\n"
      "Set VoltageBases=[12.47,4.16]\n"
      "CalcVoltageBases\n"
      "Solve\n");

  const auto system = load_three_phase_system_from_opendss(fixture.master(), 10.0);
  REQUIRE(system.transformers.size() == 1);
  const auto& transformer = system.transformers.front();
  CHECK(transformer.tap_side == 0);
  CHECK(transformer.tap_step_percent == Approx(0.0).margin(1e-12));
  CHECK(transformer.fixed_tap_pu == Approx(1.05 / 0.98).margin(1e-12));
}

TEST_CASE("Audit D12: OpenDSS unsupported assets fail closed and leadlag is retained",
          "[power_flow][math_audit][D12][opendss]") {
  SECTION("enabled Generator is rejected instead of disappearing") {
    TemporaryDSSFixture fixture(
        "d12_unsupported_generator",
        "Clear\n"
        "New Circuit.d12gen phases=3 bus1=source.1.2.3 basekv=12.47 pu=1.0\n"
        "New Generator.gen phases=3 bus1=source.1.2.3 kv=12.47 kw=100 kvar=20\n"
        "Set VoltageBases=[12.47]\n"
        "CalcVoltageBases\n"
        "Solve\n");
    CHECK_THROWS_WITH(
        load_three_phase_system_from_opendss(fixture.master(), 10.0),
        Catch::Matchers::ContainsSubstring("Generator.gen"));
  }

  SECTION("three-winding delta leg uses authored leadlag clock") {
    TemporaryDSSFixture fixture(
        "d12_three_winding_leadlag",
        "Clear\n"
        "New Circuit.d12lead phases=3 bus1=source.1.2.3 basekv=12.47 pu=1.0\n"
        "New Transformer.t3 phases=3 windings=3 "
        "buses=[source.1.2.3,mv.1.2.3,lv.1.2.3] "
        "conns=[wye,delta,wye] kvs=[12.47,4.16,0.48] "
        "kvas=[10000,10000,10000] %rs=[0.2,0.2,0.2] "
        "xhl=6 xht=8 xlt=10 leadlag=lead\n"
        "Set VoltageBases=[12.47,4.16,0.48]\n"
        "CalcVoltageBases\n"
        "Solve\n");
    const auto system =
        load_three_phase_system_from_opendss(fixture.master(), 10.0);
    const auto delta_leg = std::find_if(
        system.transformers.begin(), system.transformers.end(),
        [](const ThreePhaseTransformer& transformer) {
          return transformer.name == "t3_mv";
        });
    REQUIRE(delta_leg != system.transformers.end());
    CHECK(delta_leg->vector_group.find("11") != std::string::npos);
  }
}
#endif

TEST_CASE("Audit B18: DC factory solves AC B-theta and does not fabricate voltage magnitudes",
          "[power_flow][math_audit][B18]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus load;
  load.index = 2;
  load.bus_type = BusType::PQ;
  load.pd_mw = 20.0;
  load.in_service = true;
  sys.ac.buses = {slack, load};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.x_pu = 0.1;
  branch.tap = 1.0;
  branch.in_service = true;
  sys.ac.branches = {branch};

  const auto problem = build_power_flow_problem(sys);
  auto solver = PowerFlowSolverFactory::create(PowerFlowMethod::DC);
  const auto result = solver->solve(problem);
  REQUIRE(result.converged);
  REQUIRE(result.va.size() == 2);
  CHECK(result.va[1] == Approx(-0.02).margin(1e-12));
  CHECK(result.vm.empty());
  CHECK(result.vdc.empty());
  CHECK(result.residual < 1e-12);
  CHECK(result.converter_model_scope.model_scope == "ac-only-linearized-dc");

  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.in_service = true;
  sys.dc.buses = {dc_bus};
  const auto hybrid_problem = build_power_flow_problem(sys);
  const auto rejected = solver->solve(hybrid_problem);
  CHECK_FALSE(rejected.converged);
  CHECK(rejected.vm.empty());
  CHECK(rejected.converter_model_scope.model_scope ==
        "ac-only-linearized-dc:not-applicable-to-hybrid");
}

TEST_CASE("Audit B1: ZIP voltage derivatives match centered finite differences",
          "[power_flow][math_audit][B1]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus load;
  load.index = 2;
  load.bus_type = BusType::PQ;
  load.pd_mw = 40.0;
  load.qd_mvar = 20.0;
  load.in_service = true;
  sys.ac.buses = {slack, load};
  ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.02;
  line.x_pu = 0.15;
  line.in_service = true;
  sys.ac.branches = {line};

  auto data = hacdcpf::powerflow::make_solver_data(sys);
  data.zip_pw[0] = 0.2;
  data.zip_pw[1] = 0.3;
  data.zip_pw[2] = 0.5;
  data.zip_qw[0] = 0.1;
  data.zip_qw[1] = 0.6;
  data.zip_qw[2] = 0.3;

  hacdcpf::powerflow::JacobianContext context;
  context.n = 2;
  context.np = 1;
  context.nq = 1;
  context.nvar = 2;
  context.non_slack = {1};
  context.pq = {1};
  context.p_row = {-1, 0};
  context.q_row = {-1, 1};
  context.va_col = {-1, 0};
  context.vm_col = {-1, 1};
  auto pattern = hacdcpf::powerflow::build_jacobian_pattern(data, context);

  Eigen::VectorXd vm(2);
  Eigen::VectorXd va(2);
  vm << 1.0, 0.93;
  va << 0.0, -0.04;
  const Eigen::VectorXd vdc;
  Eigen::VectorXd pcalc(2), qcalc(2), p_spec(2), q_spec(2);
  Eigen::VectorXd pdc_linear, pdc_calc, pdc_spec;
  Eigen::VectorXd mismatch(2);
  hacdcpf::powerflow::evaluate_residual_and_jacobian(
      data, context, data.ac_buses, data.converters, data.pg, data.qg,
      vm, va, vdc, pcalc, qcalc, p_spec, q_spec, pdc_linear, pdc_calc,
      pdc_spec, mismatch, pattern, 1);

  auto residual_at = [&](double load_vm) {
    Eigen::VectorXd trial_vm = vm;
    trial_vm[1] = load_vm;
    Eigen::VectorXd pc(2), qc(2), ps(2), qs(2), pl, pdc, pds, mm(2);
    hacdcpf::powerflow::evaluate_residual_only(
        data, context, data.ac_buses, data.converters, data.pg, data.qg,
        trial_vm, va, vdc, pc, qc, ps, qs, pl, pdc, pds, mm, pattern, 1);
    return mm;
  };
  constexpr double step = 1e-6;
  const Eigen::VectorXd fd =
      (residual_at(vm[1] + step) - residual_at(vm[1] - step)) /
      (2.0 * step);
  CHECK(pattern.matrix.coeff(0, 1) == Approx(-fd[0]).margin(2e-8));
  CHECK(pattern.matrix.coeff(1, 1) == Approx(-fd[1]).margin(2e-8));
}

TEST_CASE("Audit C1: PV-to-PQ switching does not hide a sub-hysteresis Q violation",
          "[power_flow][math_audit][C1]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus pv;
  pv.index = 2;
  pv.bus_type = BusType::PV;
  pv.pd_mw = 45.0;
  pv.qd_mvar = 20.0;
  pv.in_service = true;
  sys.ac.buses = {slack, pv};
  ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.01;
  line.x_pu = 0.15;
  line.in_service = true;
  sys.ac.branches = {line};
  Generator reference;
  reference.index = 1;
  reference.bus = 1;
  reference.is_slack = true;
  reference.qmin_mvar = -500.0;
  reference.qmax_mvar = 500.0;
  reference.in_service = true;
  Generator controlled;
  controlled.index = 2;
  controlled.bus = 2;
  controlled.pg_mw = 45.0;
  controlled.vg_pu = 1.04;
  controlled.qmin_mvar = -500.0;
  controlled.qmax_mvar = 500.0;
  controlled.in_service = true;
  sys.ac.generators = {reference, controlled};

  PowerFlowOptions unconstrained_options;
  unconstrained_options.enable_pv_pq_conversion = false;
  const auto unconstrained = solve_power_flow(sys, unconstrained_options);
  REQUIRE(unconstrained.converged);
  const auto unconstrained_data =
      hacdcpf::powerflow::make_solver_data(sys);
  Eigen::VectorXcd voltage(2);
  for (int index = 0; index < 2; ++index) {
    voltage[index] = std::polar(unconstrained.vm[static_cast<size_t>(index)],
                               unconstrained.va[static_cast<size_t>(index)]);
  }
  const Eigen::VectorXcd current = unconstrained_data.ybus * voltage;
  const double qg_required_mvar =
      (voltage[1] * std::conj(current[1])).imag() * sys.base_mva +
      pv.qd_mvar;
  sys.ac.generators[1].qmax_mvar = qg_required_mvar - 0.5;

  PowerFlowOptions limited_options;
  limited_options.enable_solver_profiling = true;
  const auto limited = solve_power_flow(sys, limited_options);
  REQUIRE(limited.converged);
  CHECK(qg_required_mvar - sys.ac.generators[1].qmax_mvar ==
        Approx(0.5).margin(1e-10));
  CHECK((qg_required_mvar - sys.ac.generators[1].qmax_mvar) /
            sys.base_mva <
        limited_options.pv_q_hysteresis_pu);
  CHECK(limited.profiling.pv_to_pq_switches >= 1);
  CHECK(limited.profiling.pq_to_pv_switches == 0);
  CHECK(limited.profiling.pv_pq_repeated_active_sets == 0);
  CHECK(limited.reactive_limits.enforcement_requested);
  CHECK(limited.reactive_limits.certified);
  CHECK_FALSE(limited.reactive_limits.active_set_cycle_detected);
  CHECK_FALSE(limited.reactive_limits.outer_iteration_limit_reached);
  CHECK(limited.reactive_limits.active_limited_buses == 1);
  CHECK(limited.reactive_limits.max_violation_pu <= 1e-10);
  CHECK(limited.vm[1] < controlled.vg_pu);

  PowerFlowOptions bounded_options = limited_options;
  bounded_options.pv_pq_max_outer_iterations = 1;
  const auto bounded = solve_power_flow(sys, bounded_options);
  CHECK(bounded.reactive_limits.enforcement_requested);
  CHECK_FALSE(bounded.reactive_limits.certified);
  CHECK(bounded.reactive_limits.outer_iteration_limit_reached);
  CHECK(bounded.profiling.pv_pq_outer_iterations == 1);
  CHECK(bounded.profiling.pv_to_pq_switches >= 1);

  PowerFlowOptions smooth_options;
  smooth_options.max_iter = 100;
  smooth_options.enable_pv_pq_conversion = false;
  smooth_options.enable_semi_smooth_newton = true;
  smooth_options.enable_solver_profiling = true;
  smooth_options.robust_nonlinear.enable_smooth_ncp = true;
  smooth_options.robust_nonlinear.ncp_mu0 = 1e-3;
  smooth_options.robust_nonlinear.ncp_mu_min = 1e-10;
  const auto smooth = solve_power_flow(sys, smooth_options);
  REQUIRE(smooth.converged);
  CHECK(smooth.profiling.smooth_ncp_continuation_updates >= 1);
  CHECK(smooth.profiling.smooth_ncp_final_mu == Approx(1e-10));
  CHECK(smooth.reactive_limits.enforcement_requested);
  CHECK(smooth.reactive_limits.certified);
  CHECK(smooth.reactive_limits.max_violation_pu <= 1e-10);
}

TEST_CASE("Audit C1b: disabled Q-limit enforcement is explicit and uncertified",
          "[power_flow][math_audit][C1]") {
  HybridPowerSystem system;
  system.base_mva = 100.0;
  system.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus load;
  load.index = 2;
  load.bus_type = BusType::PQ;
  load.pd_mw = 25.0;
  load.qd_mvar = 8.0;
  load.in_service = true;
  system.ac.buses = {slack, load};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.01;
  branch.x_pu = 0.10;
  branch.in_service = true;
  system.ac.branches = {branch};
  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.in_service = true;
  system.ac.generators = {generator};
  PowerFlowOptions options;
  options.enable_pv_pq_conversion = false;
  const auto result = solve_power_flow(system, options);
  REQUIRE(result.converged);
  CHECK_FALSE(result.reactive_limits.enforcement_requested);
  CHECK_FALSE(result.reactive_limits.certified);
  CHECK(result.profiling.pv_to_pq_switches == 0);
  CHECK(result.profiling.pq_to_pv_switches == 0);
}

TEST_CASE("Audit C2: DC-DC droop is negative feedback and its Jacobian matches FD",
          "[power_flow][math_audit][C2]") {
  DCDCConverter converter;
  converter.bus_in = 1;
  converter.bus_out = 2;
  converter.control_mode = DCDCControlMode::Droop;
  converter.p_ref_mw = 10.0;
  converter.v_ref_pu = 1.0;
  converter.k_droop = 2.0;
  converter.eta = 1.0;
  converter.in_service = true;
  Eigen::VectorXd vdc(2);
  vdc << 1.0, 1.01;
  const auto transfer =
      hacdcpf::powerflow::dcdc_power_transfer(converter, vdc, 100.0);
  CHECK(transfer.p_out_ref_pu == Approx(0.08).margin(1e-12));
  CHECK(transfer.dpout_ref_dvdc_out == Approx(-2.0).margin(1e-12));

  constexpr double h = 1e-6;
  Eigen::VectorXd plus = vdc;
  Eigen::VectorXd minus = vdc;
  plus[1] += h;
  minus[1] -= h;
  const double fd =
      (hacdcpf::powerflow::dcdc_power_transfer(converter, plus, 100.0)
           .p_out_ref_pu -
       hacdcpf::powerflow::dcdc_power_transfer(converter, minus, 100.0)
           .p_out_ref_pu) /
      (2.0 * h);
  CHECK(transfer.dpout_ref_dvdc_out == Approx(fd).margin(1e-9));
}

TEST_CASE("Audit C3: out-of-service AC bus demand is zero in both load paths",
          "[power_flow][math_audit][C3]") {
  hacdcpf::powerflow::SolverData data;
  data.base_mva = 100.0;
  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::ISOLATED;
  bus.pd_mw = 25.0;
  bus.qd_mvar = 8.0;
  bus.in_service = false;
  data.ac_buses = {bus};
  data.pg = Eigen::VectorXd::Zero(1);
  data.qg = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd vm = Eigen::VectorXd::Ones(1);
  Eigen::VectorXd va = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd vdc;
  Eigen::VectorXd p;
  Eigen::VectorXd q;
  hacdcpf::powerflow::assemble_ac_injections(data, vm, va, vdc, p, q);
  CHECK(p[0] == Approx(0.0));
  CHECK(q[0] == Approx(0.0));

  Load disabled_component;
  disabled_component.index = 1;
  disabled_component.bus = 1;
  disabled_component.p_mw = 5.0;
  disabled_component.in_service = false;
  data.loads = {disabled_component};
  hacdcpf::powerflow::aggregate_load_demand(data);
  hacdcpf::powerflow::assemble_ac_injections(data, vm, va, vdc, p, q);
  CHECK(p[0] == Approx(0.0));
  CHECK(q[0] == Approx(0.0));
}

TEST_CASE("Audit C4/D5: converter clamp derivatives and efficiency are physical",
          "[power_flow][math_audit][C4][D5]") {
  VSCConverter converter;
  converter.eta = 0.98;
  converter.loss_percent = 2.0;
  converter.loss_mw = 0.1;
  converter.r_conv_ac_pu = 0.05;
  converter.bus_ac = 1;
  converter.in_service = true;

  constexpr double p = 0.4;
  constexpr double vdc = 0.05;
  const auto jacobian = hacdcpf::powerflow::converter_loss_jacobian(
      converter, p, vdc, 100.0, LossModelType::CurrentBased);
  constexpr double h = 1e-6;
  const double fd_vdc =
      (hacdcpf::powerflow::converter_loss(
           converter, p, vdc + h, 100.0, LossModelType::CurrentBased) -
       hacdcpf::powerflow::converter_loss(
           converter, p, vdc - h, 100.0, LossModelType::CurrentBased)) /
      (2.0 * h);
  CHECK(jacobian.second == Approx(0.0).margin(1e-14));
  CHECK(jacobian.second == Approx(fd_vdc).margin(1e-10));

  Eigen::VectorXd vm(1);
  vm << 0.05;
  const auto vm_jacobian =
      hacdcpf::powerflow::converter_dc_jacobian_vm_ac(
          converter, vm, p, 100.0);
  CHECK(vm_jacobian.dpdc_dvm_ac == Approx(0.0).margin(1e-14));

  converter.eta = 1.2;
  CHECK(hacdcpf::powerflow::converter_loss(
            converter, p, 1.0, 100.0, LossModelType::Linear) ==
        Approx(0.0).margin(1e-14));
}

TEST_CASE("Audit C5: LCC DC-voltage derivatives vanish below the 1 kV floor",
          "[power_flow][math_audit][C5]") {
  hacdcpf::powerflow::SolverData data;
  data.base_mva = 100.0;
  ACBus ac_bus;
  ac_bus.index = 1;
  ac_bus.in_service = true;
  data.ac_buses = {ac_bus};
  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.base_kv = 100.0;
  dc_bus.in_service = true;
  data.dc_buses = {dc_bus};
  LCCConverter converter;
  converter.ac_bus = 1;
  converter.dc_bus = 1;
  converter.station_role = LCCStationRole::Rectifier;
  converter.control_mode = LCCControlMode::ConstantCurrent;
  converter.vn_ac_kv = 100.0;
  converter.i_set_ka = 0.5;
  converter.x_comm_ohm = 1.0;
  converter.in_service = true;
  Eigen::VectorXd vm = Eigen::VectorXd::Ones(1);
  Eigen::VectorXd vdc(1);
  vdc << 0.005;

  const double analytic = hacdcpf::powerflow::lcc_dc_jacobian_vdc(
      data, converter, vm, vdc);
  constexpr double h = 1e-6;
  Eigen::VectorXd plus = vdc;
  Eigen::VectorXd minus = vdc;
  plus[0] += h;
  minus[0] -= h;
  const double fd =
      (hacdcpf::powerflow::lcc_dc_injection(data, converter, vm, plus) -
       hacdcpf::powerflow::lcc_dc_injection(data, converter, vm, minus)) /
      (2.0 * h);
  const auto full = hacdcpf::powerflow::lcc_ac_dc_jacobian(
      data, converter, vm, vdc);
  CHECK(analytic == Approx(0.0).margin(1e-14));
  CHECK(analytic == Approx(fd).margin(1e-10));
  CHECK(full.dpdc_dvdc == Approx(0.0).margin(1e-14));
  CHECK(full.dpac_dvdc == Approx(0.0).margin(1e-14));
  CHECK(full.dqac_dvdc == Approx(0.0).margin(1e-14));
}

TEST_CASE("Audit C11/D11: sequence shunts use b0 and parallel circuits",
          "[power_flow][math_audit][C11][D11]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.buses = {
      make_three_phase_bus(1, BusType::SLACK),
      make_three_phase_bus(2, BusType::PQ),
  };
  ThreePhaseACLine line;
  line.index = 11;
  line.from_bus = 1;
  line.to_bus = 2;
  line.phase_mask = PhaseMask::abc();
  line.r1_pu = 0.01;
  line.x1_pu = 0.08;
  line.r0_pu = line.r1_pu;
  line.x0_pu = line.x1_pu;
  line.b1_pu = 0.06;
  line.b0_pu = 0.12;
  line.parallel = 2;
  sys.lines = {line};

  const auto compact = build_compact_pf_data(sys, true);
  std::complex<double> y_ab{0.0, 0.0};
  for (const auto& entry : compact.ybus_entries) {
    if (entry.row == 0 && entry.col == 1) y_ab += entry.value;
  }
  const double expected_off_diagonal_b =
      static_cast<double>(line.parallel) * (line.b0_pu - line.b1_pu) / 6.0;
  CHECK(y_ab.real() == Approx(0.0).margin(1e-12));
  CHECK(y_ab.imag() == Approx(expected_off_diagonal_b).margin(1e-12));

  sys.lines[0].r0_pu = 0.0;
  sys.lines[0].x0_pu = 0.0;
  sys.buses[1].pd_a_mw = 0.05;
  sys.buses[1].pd_b_mw = 0.05;
  sys.buses[1].pd_c_mw = 0.05;
  const auto result = solve_three_phase_nr(sys);
  REQUIRE(result.converged);
  REQUIRE_FALSE(result.model_limitations.empty());
  CHECK(result.model_limitations.front().find("z1 was used") != std::string::npos);
}

TEST_CASE("Audit C12/D15: unsafe branch parameters fail closed",
          "[power_flow][math_audit][C12][D15]") {
  hacdcpf::powerflow::SolverData data;
  data.base_mva = 100.0;
  ACBus ac1;
  ac1.index = 1;
  ac1.in_service = true;
  ACBus ac2 = ac1;
  ac2.index = 2;
  data.ac_buses = {ac1, ac2};
  ACBranch ac_branch;
  ac_branch.index = 41;
  ac_branch.from_bus = 1;
  ac_branch.to_bus = 2;
  ac_branch.tap = 1.0;
  ac_branch.in_service = true;
  data.ac_branches = {ac_branch};
  CHECK_THROWS_WITH(
      hacdcpf::powerflow::build_admittance_matrix(data),
      Catch::Matchers::ContainsSubstring("AC branch 41"));

  data.ac_branches[0].x_pu = 1e-15;
  CHECK_THROWS_WITH(
      hacdcpf::powerflow::build_admittance_matrix(data),
      Catch::Matchers::ContainsSubstring("numerically unsafe"));
  data.ac_branches[0].x_pu = 0.1;
  data.ac_branches[0].tap = 0.0;
  CHECK_THROWS_WITH(
      hacdcpf::powerflow::build_admittance_matrix(data),
      Catch::Matchers::ContainsSubstring("tap"));

  DCBus dc1;
  dc1.index = 1;
  dc1.in_service = true;
  DCBus dc2 = dc1;
  dc2.index = 2;
  data.dc_buses = {dc1, dc2};
  DCBranch dc_branch;
  dc_branch.index = 52;
  dc_branch.from_bus = 1;
  dc_branch.to_bus = 2;
  dc_branch.r_pu = 0.0;
  dc_branch.in_service = true;
  data.dc_branches = {dc_branch};
  CHECK_THROWS_WITH(
      hacdcpf::powerflow::build_dc_conductance(data),
      Catch::Matchers::ContainsSubstring("DC branch 52"));

  ThreePhaseACSystem phase_sys;
  phase_sys.base_mva = 10.0;
  phase_sys.buses = {
      make_three_phase_bus(1, BusType::SLACK),
      make_three_phase_bus(2, BusType::PQ),
  };
  ThreePhaseACLine zero_line;
  zero_line.index = 63;
  zero_line.from_bus = 1;
  zero_line.to_bus = 2;
  zero_line.phase_mask = PhaseMask::abc();
  zero_line.in_service = true;
  phase_sys.lines = {zero_line};
  CHECK_THROWS_WITH(
      build_compact_pf_data(phase_sys, true),
      Catch::Matchers::ContainsSubstring("line 63"));
}

TEST_CASE("Audit C13: three-phase totals include transformer terminal losses",
          "[power_flow][math_audit][C13]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  auto slack = make_three_phase_bus(1, BusType::SLACK);
  auto load = make_three_phase_bus(2, BusType::PQ);
  load.pd_a_mw = 0.8;
  load.pd_b_mw = 0.6;
  load.pd_c_mw = 0.7;
  load.qd_a_mvar = 0.25;
  load.qd_b_mvar = 0.20;
  load.qd_c_mvar = 0.22;
  sys.buses = {slack, load};
  auto transformer = make_transformer("YNyn0");
  transformer.vkr_percent = 1.0;
  transformer.pfe_kw = 12.0;
  transformer.i0_percent = 0.5;
  sys.transformers = {transformer};

  const auto result = solve_three_phase_nr(sys);
  REQUIRE(result.converged);
  REQUIRE(result.transformer_terminal_observations.size() == 1);
  CHECK(result.total_p_loss_mw > 0.0);
  CHECK(result.total_q_loss_mvar > 0.0);

  const auto& observation = result.transformer_terminal_observations.front();
  std::complex<double> terminal_sum_mva{0.0, 0.0};
  const double vbase_ln_volts = 12.47e3 / std::sqrt(3.0);
  for (int phase = 0; phase < 3; ++phase) {
    const std::complex<double> vh{
        observation.hv_voltage_pu.real[static_cast<size_t>(phase)],
        observation.hv_voltage_pu.imag[static_cast<size_t>(phase)]};
    const std::complex<double> ih{
        observation.hv_current_amps.real[static_cast<size_t>(phase)],
        observation.hv_current_amps.imag[static_cast<size_t>(phase)]};
    const std::complex<double> vl{
        observation.lv_voltage_pu.real[static_cast<size_t>(phase)],
        observation.lv_voltage_pu.imag[static_cast<size_t>(phase)]};
    const std::complex<double> il{
        observation.lv_current_amps.real[static_cast<size_t>(phase)],
        observation.lv_current_amps.imag[static_cast<size_t>(phase)]};
    terminal_sum_mva +=
        vbase_ln_volts * (vh * std::conj(ih) + vl * std::conj(il)) / 1e6;
  }
  CHECK(result.total_p_loss_mw == Approx(terminal_sum_mva.real()).margin(1e-9));
  CHECK(result.total_q_loss_mvar == Approx(terminal_sum_mva.imag()).margin(1e-9));
  CHECK(result.total_p_loss_mw ==
        Approx(result.p_loss_a_mw + result.p_loss_b_mw + result.p_loss_c_mw)
            .margin(1e-12));
}

TEST_CASE("Audit C14: fixed-point paths reject PV and enforce vmin validity",
          "[power_flow][math_audit][C14]") {
  ThreePhaseACSystem pv_sys;
  pv_sys.base_mva = 10.0;
  pv_sys.buses = {
      make_three_phase_bus(1, BusType::SLACK),
      make_three_phase_bus(2, BusType::PV),
  };
  ThreePhaseACLine line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r1_pu = line.r0_pu = 0.01;
  line.x1_pu = line.x0_pu = 0.08;
  pv_sys.lines = {line};
  CHECK_THROWS_WITH(
      solve_three_phase_compact_pf(pv_sys),
      Catch::Matchers::ContainsSubstring("does not implement PV voltage control"));
  CHECK_THROWS_WITH(
      solve_three_phase_fixed_point(pv_sys),
      Catch::Matchers::ContainsSubstring("does not implement PV voltage control"));

  ThreePhaseACSystem low_voltage_sys;
  low_voltage_sys.base_mva = 10.0;
  auto low_slack = make_three_phase_bus(1, BusType::SLACK);
  low_slack.vm_a_pu = low_slack.vm_b_pu = low_slack.vm_c_pu = 0.90;
  low_voltage_sys.buses = {low_slack};
  ThreePhaseFixedPointOptions options;
  options.vmin_pu = 0.95;
  const auto rejected = solve_three_phase_compact_pf(low_voltage_sys, options);
  CHECK_FALSE(rejected.converged);
  CHECK(rejected.primary_solver_failed_reason ==
        "solved voltage is below configured vmin_pu");
  CHECK_FALSE(rejected.model_limitations.empty());
}

TEST_CASE("Audit C15: impedance-grounded constant-Z calibration satisfies nominal KCL",
          "[power_flow][math_audit][C15]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.buses = {make_three_phase_bus(1, BusType::SLACK)};
  ThreePhaseLoad load;
  load.index = 15;
  load.bus = 1;
  load.phase_mask = PhaseMask::abc();
  load.connection = "wye";
  load.grounded = true;
  load.r_neut_ohm = 20.0;
  load.x_neut_ohm = 5.0;
  load.p_a_mw = 0.30; load.q_a_mvar = 0.12;
  load.p_b_mw = 0.10; load.q_b_mvar = 0.04;
  load.p_c_mw = 0.22; load.q_c_mvar = 0.08;
  load.const_z_percent = 100.0;
  load.const_i_percent = 0.0;
  load.const_p_percent = 0.0;
  load.vmin_pu = 0.1;
  load.vmax_pu = 2.0;
  load.in_service = true;
  sys.loads = {load};

  const auto data = build_compact_pf_data(sys, true);
  Eigen::Matrix3cd ybus = Eigen::Matrix3cd::Zero();
  for (const auto& entry : data.ybus_entries) {
    ybus(entry.row, entry.col) += entry.value;
  }
  Eigen::Vector3cd voltage;
  voltage << std::complex<double>(1.0, 0.0),
      std::polar(1.0, -2.0 * kPi / 3.0),
      std::polar(1.0, 2.0 * kPi / 3.0);
  const Eigen::Vector3cd current = ybus * voltage;
  const std::array<double, 3> expected_p = {
      load.p_a_mw, load.p_b_mw, load.p_c_mw};
  const std::array<double, 3> expected_q = {
      load.q_a_mvar, load.q_b_mvar, load.q_c_mvar};
  for (int phase = 0; phase < 3; ++phase) {
    const std::complex<double> power =
        voltage[phase] * std::conj(current[phase]) * (sys.base_mva / 3.0);
    CHECK(power.real() == Approx(expected_p[static_cast<size_t>(phase)]).margin(1e-11));
    CHECK(power.imag() == Approx(expected_q[static_cast<size_t>(phase)]).margin(1e-11));
  }
}

TEST_CASE("Audit D15: negative scaling cannot reverse fixed injections",
          "[power_flow][math_audit][D15]") {
  hacdcpf::powerflow::SolverData data;
  data.base_mva = 100.0;
  ACBus bus;
  bus.index = 1;
  bus.in_service = true;
  data.ac_buses = {bus};
  StaticGenerator generator;
  generator.bus = 1;
  generator.p_mw = 10.0;
  generator.q_mvar = 3.0;
  generator.scaling = -1.0;
  data.static_generators = {generator};
  hacdcpf::powerflow::aggregate_generation(data);
  CHECK(data.pg[0] == Approx(0.0));
  CHECK(data.qg[0] == Approx(0.0));

  Load ac_load;
  ac_load.bus = 1;
  ac_load.p_mw = 5.0;
  ac_load.q_mvar = 2.0;
  ac_load.scaling = -2.0;
  data.loads = {ac_load};
  hacdcpf::powerflow::aggregate_load_demand(data);
  CHECK(data.pd_pu[0] == Approx(0.0));
  CHECK(data.qd_pu[0] == Approx(0.0));

  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.in_service = true;
  data.dc_buses = {dc_bus};
  data.dc_static_generators = {generator};
  Eigen::VectorXd vm = Eigen::VectorXd::Ones(1);
  Eigen::VectorXd va = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd vdc = Eigen::VectorXd::Ones(1);
  Eigen::VectorXd pdc = Eigen::VectorXd::Zero(1);
  hacdcpf::powerflow::assemble_dc_injections(data, vm, va, vdc, pdc);
  CHECK(pdc[0] == Approx(0.0));
}

TEST_CASE("Audit C21: Jacobian checker uses combined absolute-relative tolerance",
          "[power_flow][math_audit][C21]") {
  Eigen::VectorXd x0(1);
  x0 << 0.0;
  const hacdcpf::ResidualFn residual = [](const Eigen::VectorXd& x) {
    Eigen::VectorXd value(1);
    value[0] = 1e-10 * x[0];
    return value;
  };
  const hacdcpf::DenseJacFn near_zero = [](const Eigen::VectorXd&) {
    return Eigen::MatrixXd::Zero(1, 1);
  };
  const auto accepted =
      hacdcpf::check_jacobian_dense(residual, near_zero, x0, 1e-6, 1e-9, 1e-5);
  CHECK(accepted.passed);
  CHECK(accepted.max_absolute_error < accepted.absolute_tolerance);

  const hacdcpf::DenseJacFn wrong = [](const Eigen::VectorXd&) {
    return Eigen::MatrixXd::Constant(1, 1, 1e-3);
  };
  const auto rejected =
      hacdcpf::check_jacobian_dense(residual, wrong, x0, 1e-6, 1e-9, 1e-5);
  CHECK_FALSE(rejected.passed);
  CHECK(rejected.max_normalized_error > 1.0);
}

TEST_CASE("Audit C18: adaptive solver consumes bus reactive-limit overrides",
          "[power_flow][math_audit][C18]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 1.0;
  slack.in_service = true;
  ACBus pv;
  pv.index = 2;
  pv.bus_type = BusType::PV;
  pv.vm_pu = 1.05;
  pv.pd_mw = 50.0;
  pv.qd_mvar = 35.0;
  pv.in_service = true;
  sys.ac.buses = {slack, pv};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.02;
  branch.x_pu = 0.20;
  branch.tap = 1.0;
  branch.in_service = true;
  sys.ac.branches = {branch};
  Generator slack_generator;
  slack_generator.index = 1;
  slack_generator.bus = 1;
  slack_generator.is_slack = true;
  slack_generator.qmin_mvar = -200.0;
  slack_generator.qmax_mvar = 200.0;
  slack_generator.in_service = true;
  Generator pv_generator;
  pv_generator.index = 2;
  pv_generator.bus = 2;
  pv_generator.pg_mw = 50.0;
  pv_generator.vg_pu = 1.05;
  pv_generator.qmin_mvar = -200.0;
  pv_generator.qmax_mvar = 200.0;
  pv_generator.in_service = true;
  sys.ac.generators = {slack_generator, pv_generator};

  hacdcpf::powerflow::AdaptiveSolver solver;
  PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 80;
  const auto unrestricted = solver.solve(sys, options, {});
  const auto limited = solver.solve(
      sys, options, {{2, ReactiveLimit{.qmin = 0.0, .qmax = 0.0}}});
  REQUIRE(unrestricted.converged);
  REQUIRE(limited.converged);
  REQUIRE(unrestricted.vm.size() == 2);
  REQUIRE(limited.vm.size() == 2);
  CHECK(unrestricted.vm[1] == Approx(1.05).margin(1e-8));
  CHECK(std::abs(limited.vm[1] - unrestricted.vm[1]) > 1e-3);
  CHECK_THROWS_WITH(
      solver.solve(sys, options, {{99, ReactiveLimit{}}}),
      Catch::Matchers::ContainsSubstring("without an in-service generator"));
}

TEST_CASE("Audit A7: transformer current observations use the physical ampere base",
          "[power_flow][math_audit][A7]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  auto slack = make_three_phase_bus(1, BusType::SLACK);
  auto load = make_three_phase_bus(2, BusType::PQ);
  load.pd_a_mw = 1.0;
  load.pd_b_mw = 1.0;
  load.pd_c_mw = 1.0;
  sys.buses = {slack, load};
  sys.transformers = {make_transformer("YNyn0")};

  ThreePhaseNROptions options;
  options.max_iter = 80;
  options.tol = 1e-10;
  const auto result = solve_three_phase_nr(sys, options);
  REQUIRE(result.converged);
  REQUIRE(result.transformer_terminal_observations.size() == 1);
  REQUIRE(result.bus_voltages.size() == 2);
  const auto& observation = result.transformer_terminal_observations.front();
  const double measured = std::hypot(
      observation.lv_current_amps.real[0],
      observation.lv_current_amps.imag[0]);
  const double v_ln_volts =
      result.bus_voltages[1].vm_a_pu * 12.47e3 / std::sqrt(3.0);
  const double expected = 1.0e6 / v_ln_volts;
  CHECK(measured == Approx(expected).epsilon(2e-6));
}

TEST_CASE("Audit A8: ungrounded wye blocks common-mode transformer current",
          "[power_flow][math_audit][A8]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  auto hv = make_three_phase_bus(1, BusType::SLACK);
  auto lv = make_three_phase_bus(2, BusType::PQ);
  sys.buses = {hv, lv};
  sys.transformers = {make_transformer("Yy0")};

  const auto entries = build_full_ybus_phase_entries(sys, true);
  std::vector<std::complex<double>> voltage(6, {0.0, 0.0});
  voltage[0] = {1.0, 0.0};
  voltage[1] = {1.0, 0.0};
  voltage[2] = {1.0, 0.0};
  std::vector<std::complex<double>> current(6, {0.0, 0.0});
  for (const auto& entry : entries) {
    current[static_cast<size_t>(entry.row)] +=
        entry.value * voltage[static_cast<size_t>(entry.col)];
  }
  double max_current = 0.0;
  for (const auto& value : current) {
    max_current = std::max(max_current, std::abs(value));
  }
  CHECK(max_current < 1e-10);
}

TEST_CASE("Audit B4: median NCP contains all three PV/PQ physical branches",
          "[power_flow][math_audit][B4]") {
  using hacdcpf::powerflow::pv_pq_ncp;
  constexpr double qmin = -0.2;
  constexpr double qmax = 0.3;
  constexpr double vset = 1.0;

  CHECK(pv_pq_ncp(0.0, qmin, qmax, vset, vset) == Approx(0.0).margin(1e-14));
  CHECK(pv_pq_ncp(qmax, qmin, qmax, 0.95, vset) == Approx(0.0).margin(1e-14));
  CHECK(pv_pq_ncp(qmin, qmin, qmax, 1.05, vset) == Approx(0.0).margin(1e-14));
  CHECK(std::abs(pv_pq_ncp(qmax, qmin, qmax, 1.05, vset)) > 1e-3);
  CHECK(std::abs(pv_pq_ncp(qmin, qmin, qmax, 0.95, vset)) > 1e-3);
}

TEST_CASE("Audit B4b: CHKS-smoothed PV/PQ median has a consistent Jacobian",
          "[power_flow][math_audit][B4]") {
  using hacdcpf::powerflow::pv_pq_ncp;
  using hacdcpf::powerflow::pv_pq_smooth_ncp;
  using hacdcpf::powerflow::pv_pq_smooth_ncp_jacobian;
  constexpr double qmin = -0.2;
  constexpr double qmax = 0.3;
  constexpr double qg = 0.295;
  constexpr double vm = 0.997;
  constexpr double vset = 1.0;
  constexpr double mu = 1e-3;
  constexpr double step = 1e-7;

  const auto [dq, dv] =
      pv_pq_smooth_ncp_jacobian(qg, qmin, qmax, vm, vset, mu);
  const double fd_q =
      (pv_pq_smooth_ncp(qg + step, qmin, qmax, vm, vset, mu) -
       pv_pq_smooth_ncp(qg - step, qmin, qmax, vm, vset, mu)) /
      (2.0 * step);
  const double fd_v =
      (pv_pq_smooth_ncp(qg, qmin, qmax, vm + step, vset, mu) -
       pv_pq_smooth_ncp(qg, qmin, qmax, vm - step, vset, mu)) /
      (2.0 * step);
  CHECK(dq == Approx(fd_q).margin(2e-8));
  CHECK(dv == Approx(fd_v).margin(2e-8));
  CHECK(pv_pq_smooth_ncp(qg, qmin, qmax, vm, vset, 1e-10) ==
        Approx(pv_pq_ncp(qg, qmin, qmax, vm, vset)).margin(2e-9));
}

TEST_CASE("Audit A9/B17: distributed slack uses unified ZIP component injections",
          "[power_flow][math_audit][A9][B17]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 1.0;
  slack.in_service = true;
  ACBus pq;
  pq.index = 2;
  pq.bus_type = BusType::PQ;
  pq.vm_pu = 1.0;
  pq.in_service = true;
  sys.ac.buses = {slack, pq};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.02;
  branch.x_pu = 0.08;
  branch.tap = 1.0;
  branch.in_service = true;
  sys.ac.branches = {branch};
  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.in_service = true;
  sys.ac.generators = {generator};
  Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 5.0;
  load.q_mvar = 1.0;
  load.model = LoadModel::ZIP;
  load.i_percent_p = 100.0;
  load.p_percent_p = 0.0;
  load.i_percent_q = 100.0;
  load.p_percent_q = 0.0;
  sys.ac.loads = {load};

  DistributedSlack cfg;
  cfg.participating_buses = {2};
  cfg.participation_factors = {1.0};
  cfg.reference_bus = 1;
  PowerFlowOptions options;
  options.tol = 1e-10;
  const auto result = solve_power_flow_distributed_slack(sys, cfg, options);
  REQUIRE(result.converged);
  REQUIRE(result.distributed_slack_p.count(2) == 1);

  const auto data = hacdcpf::powerflow::make_solver_data(sys);
  Eigen::VectorXcd voltage(2);
  for (int i = 0; i < 2; ++i) {
    voltage[i] = std::polar(result.vm[static_cast<size_t>(i)],
                            result.va[static_cast<size_t>(i)]);
  }
  const Eigen::VectorXcd current = data.ybus * voltage;
  const double slack_network_injection =
      (voltage[0] * std::conj(current[0])).real();
  CHECK(result.distributed_slack_p.at(2) ==
        Approx(slack_network_injection).margin(2e-9));
  CHECK(result.distributed_slack_p.at(2) > 0.049);
}

TEST_CASE("Audit B17: iterative distributed slack re-solves and honors the reference bus",
          "[power_flow][math_audit][B17]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  for (int id = 1; id <= 3; ++id) {
    ACBus bus;
    bus.index = id;
    bus.bus_type = id == 1 ? BusType::SLACK
                           : (id == 2 ? BusType::PV : BusType::PQ);
    bus.vm_pu = 1.0;
    bus.in_service = true;
    if (id == 3) {
      bus.pd_mw = 10.0;
      bus.qd_mvar = 3.0;
    }
    sys.ac.buses.push_back(bus);
  }
  for (int from : {1, 2}) {
    ACBranch branch;
    branch.index = from;
    branch.from_bus = from;
    branch.to_bus = 3;
    branch.r_pu = 0.02;
    branch.x_pu = 0.08;
    branch.tap = 1.0;
    branch.in_service = true;
    sys.ac.branches.push_back(branch);
  }
  for (int bus : {1, 2}) {
    Generator generator;
    generator.index = bus;
    generator.bus = bus;
    generator.pg_mw = 0.0;
    generator.pmin_mw = 0.0;
    generator.pmax_mw = 20.0;
    generator.qmin_mvar = -20.0;
    generator.qmax_mvar = 20.0;
    generator.vg_pu = 1.0;
    generator.in_service = true;
    generator.is_slack = bus == 1;
    sys.ac.generators.push_back(generator);
  }

  DistributedSlack cfg;
  cfg.participating_buses = {1, 2};
  cfg.participation_factors = {0.4, 0.6};
  cfg.reference_bus = 2;
  PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 100;
  const auto result = solve_power_flow_distributed_slack_full(sys, cfg, options);
  REQUIRE(result.converged);
  REQUIRE(result.vm.size() == 3);
  REQUIRE(result.va.size() == 3);
  CHECK(result.reference_bus_used == 2);
  CHECK(result.va[1] == Approx(0.0).margin(2e-10));
  CHECK(result.distributed_slack_p_unit == "p.u. on system base_mva");
  CHECK(result.model_scope ==
        "iterative-newton-bounded-distributed-slack-redispatch");
  CHECK(std::abs(result.unallocated_slack_pu) < 1e-10);

  const auto data = hacdcpf::powerflow::make_solver_data(sys);
  Eigen::VectorXcd voltage(3);
  for (int bus = 0; bus < 3; ++bus) {
    voltage[bus] = std::polar(result.vm[static_cast<size_t>(bus)],
                              result.va[static_cast<size_t>(bus)]);
  }
  const Eigen::VectorXcd current = data.ybus * voltage;
  const double p_bus1 = (voltage[0] * std::conj(current[0])).real();
  const double p_bus2 = (voltage[1] * std::conj(current[1])).real();
  CHECK(p_bus1 == Approx(result.distributed_slack_p.at(1)).margin(2e-9));
  CHECK(p_bus2 == Approx(result.distributed_slack_p.at(2)).margin(2e-9));
  CHECK(result.distributed_slack_p.at(1) +
            result.distributed_slack_p.at(2) >
        0.10);
}

TEST_CASE("Audit B17: distributed slack fails when pmin-pmax cannot absorb mismatch",
          "[power_flow][math_audit][B17]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus pv;
  pv.index = 2;
  pv.bus_type = BusType::PV;
  pv.in_service = true;
  ACBus load;
  load.index = 3;
  load.bus_type = BusType::PQ;
  load.pd_mw = 10.0;
  load.qd_mvar = 2.0;
  load.in_service = true;
  sys.ac.buses = {slack, pv, load};
  for (int from : {1, 2}) {
    ACBranch branch;
    branch.index = from;
    branch.from_bus = from;
    branch.to_bus = 3;
    branch.r_pu = 0.01;
    branch.x_pu = 0.05;
    branch.tap = 1.0;
    branch.in_service = true;
    sys.ac.branches.push_back(branch);
    Generator generator;
    generator.index = from;
    generator.bus = from;
    generator.pmin_mw = 0.0;
    generator.pmax_mw = 3.0;
    generator.qmin_mvar = -10.0;
    generator.qmax_mvar = 10.0;
    generator.in_service = true;
    generator.is_slack = from == 1;
    sys.ac.generators.push_back(generator);
  }
  DistributedSlack cfg;
  cfg.participating_buses = {1, 2};
  cfg.participation_factors = {0.5, 0.5};
  cfg.reference_bus = 1;
  PowerFlowOptions options;
  options.tol = 1e-10;
  const auto result = solve_power_flow_distributed_slack_full(sys, cfg, options);
  CHECK_FALSE(result.converged);
  CHECK(result.unallocated_slack_pu > 0.03);
  CHECK(result.diagnostics.termination_reason ==
        "Distributed slack generator limits leave unallocated active power");
  CHECK(result.hit_limits.size() == 2);
}

TEST_CASE("Audit B17: distributed slack propagates global ZIP options",
          "[power_flow][math_audit][B17]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus load;
  load.index = 2;
  load.bus_type = BusType::PQ;
  load.pd_mw = 50.0;
  load.qd_mvar = 20.0;
  load.in_service = true;
  sys.ac.buses = {slack, load};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.04;
  branch.x_pu = 0.12;
  branch.tap = 1.0;
  branch.in_service = true;
  sys.ac.branches = {branch};
  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.pg_mw = 50.0;
  generator.pmin_mw = 0.0;
  generator.pmax_mw = 100.0;
  generator.in_service = true;
  generator.is_slack = true;
  sys.ac.generators = {generator};
  DistributedSlack cfg;
  cfg.participating_buses = {1};
  cfg.participation_factors = {1.0};
  cfg.reference_bus = 1;

  PowerFlowOptions constant_power;
  constant_power.tol = 1e-10;
  const auto pq_result =
      solve_power_flow_distributed_slack(sys, cfg, constant_power);
  REQUIRE(pq_result.converged);

  PowerFlowOptions constant_impedance = constant_power;
  constant_impedance.zip_pw[0] = 0.0;
  constant_impedance.zip_pw[2] = 1.0;
  constant_impedance.zip_qw[0] = 0.0;
  constant_impedance.zip_qw[2] = 1.0;
  const auto z_result =
      solve_power_flow_distributed_slack(sys, cfg, constant_impedance);
  REQUIRE(z_result.converged);
  CHECK(z_result.vm[1] > pq_result.vm[1] + 1e-3);
}

TEST_CASE("Audit B8: Mode 6 is a DC voltage-forming source",
          "[power_flow][math_audit][B8]") {
  HybridPowerSystem sys;
  ACBus ac;
  ac.index = 1;
  ac.bus_type = BusType::SLACK;
  ac.in_service = true;
  sys.ac.buses = {ac};
  DCBus dc;
  dc.index = 1;
  dc.bus_type = DCBusType::DC_P;
  dc.pd_mw = 1.0;
  dc.in_service = true;
  sys.dc.buses = {dc};
  VSCConverter conv;
  conv.index = 1;
  conv.bus_ac = 1;
  conv.bus_dc = 1;
  conv.control_mode = ConverterMode::DC_V_DROOP_AC_V;
  conv.k_vdc = 10.0;
  conv.v_dc_set_pu = 1.0;
  conv.in_service = true;
  sys.vsc_converters = {conv};

  const auto report =
      hacdcpf::powerflow::evaluate_converter_coordination(sys, true);
  bool missing_reference = false;
  for (const auto& issue : report.issues) {
    missing_reference = missing_reference || issue.rule_id == "DCISLAND-01";
  }
  CHECK_FALSE(missing_reference);
  REQUIRE(report.dc_islands.size() == 1);
  CHECK(report.dc_islands.front().droop_sources == 1);
}

TEST_CASE("Audit B9: island graph retains pure DC, LCC and ER connectivity",
          "[power_flow][math_audit][B9]") {
  SECTION("pure DC generation is retained as an energized island") {
    HybridPowerSystem sys;
    DCBus bus;
    bus.index = 7;
    bus.bus_type = DCBusType::DC_V;
    bus.in_service = true;
    sys.dc.buses = {bus};
    StaticGeneratorDC source;
    source.index = 3;
    source.bus = 7;
    source.p_set_mw = 1.0;
    source.in_service = true;
    sys.dc.dc_static_generators = {source};

    const auto islands = hacdcpf::powerflow::detect_islands(sys);
    REQUIRE(islands.size() == 1);
    CHECK(islands.front().ac_buses.empty());
    CHECK(islands.front().dc_buses == std::vector<int>{7});
    CHECK(islands.front().has_generators);
  }

  SECTION("two AC areas linked only through LCC terminals form one island") {
    HybridPowerSystem sys;
    for (int id : {11, 22}) {
      ACBus bus;
      bus.index = id;
      bus.bus_type = id == 11 ? BusType::SLACK : BusType::PQ;
      bus.in_service = true;
      sys.ac.buses.push_back(bus);
    }
    DCBus dc;
    dc.index = 5;
    dc.bus_type = DCBusType::DC_V;
    dc.in_service = true;
    sys.dc.buses = {dc};
    LCCConverter rect;
    rect.index = 1;
    rect.ac_bus = 11;
    rect.dc_bus = 5;
    rect.in_service = true;
    LCCConverter inv = rect;
    inv.index = 2;
    inv.ac_bus = 22;
    inv.station_role = LCCStationRole::Inverter;
    sys.lcc_converters = {rect, inv};
    Generator generator;
    generator.index = 1;
    generator.bus = 11;
    generator.is_slack = true;
    generator.in_service = true;
    sys.ac.generators = {generator};

    const auto islands = hacdcpf::powerflow::detect_islands(sys);
    REQUIRE(islands.size() == 1);
    CHECK(islands.front().ac_buses == std::vector<int>{11, 22});
    CHECK(islands.front().dc_buses == std::vector<int>{5});
    const auto sub = hacdcpf::powerflow::extract_island_subsystem(
        sys, islands.front(), 11);
    CHECK(sub.lcc_converters.size() == 2);
  }

  SECTION("energy-router AC/DC ports survive subsystem extraction") {
    HybridPowerSystem sys;
    ACBus ac;
    ac.index = 9;
    ac.bus_type = BusType::SLACK;
    ac.in_service = true;
    sys.ac.buses = {ac};
    DCBus dc;
    dc.index = 4;
    dc.bus_type = DCBusType::DC_V;
    dc.in_service = true;
    sys.dc.buses = {dc};
    EnergyRouter router;
    router.index = 8;
    router.in_service = true;
    EnergyRouterPort ac_port;
    ac_port.index = 1;
    ac_port.bus = 9;
    ac_port.port_type = ERPortType::AC;
    EnergyRouterPort dc_port;
    dc_port.index = 2;
    dc_port.bus = 4;
    dc_port.port_type = ERPortType::DC;
    router.ports = {ac_port, dc_port};
    router.num_ports = 2;
    sys.energy_routers = {router};
    Generator generator;
    generator.index = 1;
    generator.bus = 9;
    generator.is_slack = true;
    generator.in_service = true;
    sys.ac.generators = {generator};

    const auto islands = hacdcpf::powerflow::detect_islands(sys);
    REQUIRE(islands.size() == 1);
    const auto sub = hacdcpf::powerflow::extract_island_subsystem(
        sys, islands.front(), 9);
    REQUIRE(sub.energy_routers.size() == 1);
    REQUIRE(sub.energy_routers.front().ports.size() == 2);
    CHECK(sub.energy_routers.front().ports[0].bus == 1);
    CHECK(sub.energy_routers.front().ports[1].bus == 1);
    CHECK(sub.energy_routers.front().ports[0].port_type == ERPortType::AC);
    CHECK(sub.energy_routers.front().ports[1].port_type == ERPortType::DC);
  }
}

TEST_CASE("Audit B10: AC-only approximations reject hybrid assets without hiding AC loads",
          "[power_flow][math_audit][B10]") {
  HybridPowerSystem ac_only;
  ac_only.base_mva = 100.0;
  ac_only.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus load_bus;
  load_bus.index = 2;
  load_bus.bus_type = BusType::PQ;
  load_bus.in_service = true;
  ac_only.ac.buses = {slack, load_bus};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.x_pu = 0.1;
  branch.tap = 1.0;
  branch.in_service = true;
  ac_only.ac.branches = {branch};
  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.in_service = true;
  ac_only.ac.generators = {generator};
  Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 20.0;
  load.in_service = true;
  ac_only.ac.loads = {load};

  const auto linear = solve_ac_dc_power_flow(ac_only);
  REQUIRE(linear.success);
  REQUIRE(linear.pf_mw.size() == 1);
  CHECK(linear.pf_mw.front() == Approx(20.0).margin(1e-10));
  CHECK(linear.model_scope == "ac-only-linearized-dc");

  HybridPowerSystem hybrid = ac_only;
  DCBus dc;
  dc.index = 1;
  dc.bus_type = DCBusType::DC_V;
  dc.in_service = true;
  hybrid.dc.buses = {dc};
  const auto rejected_linear = solve_ac_dc_power_flow(hybrid);
  CHECK_FALSE(rejected_linear.success);
  CHECK(rejected_linear.model_scope ==
        "ac-only-linearized-dc:not-applicable-to-hybrid");
  CHECK_FALSE(rejected_linear.model_limitations.empty());

  const auto rejected_fdpf = solve_power_flow_fdpf(hybrid);
  CHECK_FALSE(rejected_fdpf.converged);
  CHECK(rejected_fdpf.converter_model_scope.model_scope ==
        "ac-only-fdpf:not-applicable-to-hybrid");
  CHECK_FALSE(rejected_fdpf.diagnostics.termination_reason.empty());
}

TEST_CASE("Audit B11: residual evaluator matches multi-reference equation sets",
          "[power_flow][math_audit][B11]") {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  for (int id : {1, 2}) {
    ACBus bus;
    bus.index = id;
    bus.bus_type = BusType::SLACK;
    bus.vm_pu = 1.0;
    bus.in_service = true;
    sys.ac.buses.push_back(bus);
  }
  for (int id = 1; id <= 4; ++id) {
    DCBus bus;
    bus.index = id;
    bus.bus_type = (id == 1 || id == 3) ? DCBusType::DC_V
                                        : DCBusType::DC_P;
    bus.vm_pu = 1.0;
    bus.in_service = true;
    sys.dc.buses.push_back(bus);
  }
  DCBranch first;
  first.index = 1;
  first.from_bus = 1;
  first.to_bus = 2;
  first.r_pu = 0.1;
  first.in_service = true;
  DCBranch second = first;
  second.index = 2;
  second.from_bus = 3;
  second.to_bus = 4;
  sys.dc.branches = {first, second};

  const auto data = hacdcpf::powerflow::make_solver_data(sys);
  const Eigen::VectorXd vm = Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd va = Eigen::VectorXd::Zero(2);
  const Eigen::VectorXd vdc = Eigen::VectorXd::Ones(4);
  const auto residual = hacdcpf::powerflow::evaluate_power_flow_residual(
      data, vm, va, vdc);
  CHECK(residual.ac.size() == 0);
  CHECK(residual.dc.size() == 2);
  CHECK(residual.full.size() == 2);
}

TEST_CASE("Audit D1: AC kernel is bitwise reproducible across thread counts",
          "[power_flow][math_audit][D1]") {
  hacdcpf::powerflow::JacobianPattern pattern;
  pattern.matrix.resize(4, 4);
  for (int i = 0; i < 4; ++i) pattern.matrix.insert(i, i) = 0.0;
  pattern.matrix.makeCompressed();
  pattern.ac_entries.reserve(2048);
  for (int k = 0; k < 2048; ++k) {
    hacdcpf::powerflow::JacobianPattern::ACEntry entry;
    entry.i = k % 2;
    entry.j = 1 - entry.i;
    entry.p_va_nz = 0;
    entry.q_va_nz = 1;
    entry.p_vm_nz = 2;
    entry.q_vm_nz = 3;
    entry.g = 1e-4 * static_cast<double>((k % 17) - 8);
    entry.b = -2e-4 * static_cast<double>((k % 13) + 1);
    pattern.ac_entries.push_back(entry);
  }
  Eigen::VectorXd vm(2);
  Eigen::VectorXd va(2);
  vm << 0.97, 1.03;
  va << -0.17, 0.23;

  auto evaluate = [&](int threads) {
    Eigen::VectorXd p = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
    std::array<double, 4> jac{};
    hacdcpf::powerflow::evaluate_ac_kernel_parallel(
        pattern, vm, va, threads, true, jac.data(), p, q);
    std::array<std::uint64_t, 8> bits{};
    bits[0] = std::bit_cast<std::uint64_t>(p[0]);
    bits[1] = std::bit_cast<std::uint64_t>(p[1]);
    bits[2] = std::bit_cast<std::uint64_t>(q[0]);
    bits[3] = std::bit_cast<std::uint64_t>(q[1]);
    for (int i = 0; i < 4; ++i) {
      bits[static_cast<size_t>(i + 4)] =
          std::bit_cast<std::uint64_t>(jac[static_cast<size_t>(i)]);
    }
    return bits;
  };

  const auto serial = evaluate(1);
  const auto two_threads = evaluate(2);
  const auto auto_threads = evaluate(0);
  CHECK(serial == two_threads);
  CHECK(serial == auto_threads);
}

TEST_CASE("Audit D7: B-double-prime includes component shunts",
          "[power_flow][math_audit][D7]") {
  hacdcpf::powerflow::SolverData data;
  data.base_mva = 100.0;
  ACBus bus;
  bus.index = 1;
  bus.in_service = true;
  data.ac_buses = {bus};

  Shunt fixed;
  fixed.index = 1;
  fixed.bus = 1;
  fixed.bs_mvar = 20.0;
  fixed.in_service = true;
  Shunt switched = fixed;
  switched.index = 2;
  switched.switchable = true;
  switched.n_steps = 5;
  switched.current_step = 3;
  switched.bs_per_step = 5.0;
  switched.bs_mvar = 999.0;
  data.shunts = {fixed, switched};

  const auto bpp = hacdcpf::powerflow::build_susceptance_matrix(
      data, hacdcpf::powerflow::make_bpp_flags());
  CHECK(bpp.coeff(0, 0) == Approx(-0.35).margin(1e-14));
}

TEST_CASE("Audit D9: LCC angle diagnostics and power contract are independent",
          "[power_flow][math_audit][D9]") {
  LCCConverter converter;
  converter.station_role = LCCStationRole::Rectifier;
  converter.control_mode = LCCControlMode::ConstantPower;
  converter.p_set_mw = 0.0;
  CHECK_FALSE(hacdcpf::powerflow::lcc_operating_point(
                  converter, 100.0, 100.0).valid);

  converter.p_set_mw = 10.0;
  converter.v_drop_v = 50000.0;
  const auto point = hacdcpf::powerflow::lcc_operating_point(
      converter, 100.0, 100.0);
  REQUIRE(point.valid);
  CHECK(point.alpha_beyond_range);
  CHECK_FALSE(point.gamma_beyond_range);
}

TEST_CASE("Audit D10: HELM rejects isolated buses instead of treating them as PV",
          "[power_flow][math_audit][D10]") {
  hacdcpf::powerflow::SolverData data;
  data.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.in_service = true;
  ACBus isolated = slack;
  isolated.index = 2;
  isolated.bus_type = BusType::ISOLATED;
  data.ac_buses = {slack, isolated};
  data.ybus.resize(2, 2);
  data.pg = Eigen::VectorXd::Zero(2);
  data.qg = Eigen::VectorXd::Zero(2);
  data.pd_pu = Eigen::VectorXd::Zero(2);
  data.qd_pu = Eigen::VectorXd::Zero(2);

  hacdcpf::powerflow::HelmSolver solver;
  const auto result = solver.solve(data, PowerFlowOptions{});
  CHECK_FALSE(result.converged);
  CHECK(std::isinf(result.residual));
  CHECK(result.diagnostics.termination_reason ==
        "HELM does not support in-model isolated AC buses");
}

TEST_CASE("Audit D8: standalone DC Newton includes voltage-dependent injections",
          "[power_flow][math_audit][D8]") {
  hacdcpf::powerflow::SolverData data;
  data.base_mva = 100.0;
  DCBus reference;
  reference.index = 1;
  reference.bus_type = DCBusType::DC_V;
  reference.vm_pu = 1.0;
  reference.in_service = true;
  DCBus controlled = reference;
  controlled.index = 2;
  controlled.bus_type = DCBusType::DC_P;
  data.dc_buses = {reference, controlled};
  DCBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.1;
  branch.in_service = true;
  data.dc_branches = {branch};
  data.gdc = hacdcpf::powerflow::build_dc_conductance(data);

  DCDCConverter converter;
  converter.index = 1;
  converter.bus_in = 1;
  converter.bus_out = 2;
  converter.control_mode = DCDCControlMode::Droop;
  converter.p_ref_mw = 10.0;
  converter.v_ref_pu = 1.0;
  converter.k_droop = 2.0;
  converter.eta = 1.0;
  converter.in_service = true;
  data.dcdc_converters = {converter};

  PowerFlowOptions options;
  options.max_iter = 1;
  options.tol = 1e-14;
  hacdcpf::powerflow::DCSolver solver;
  const auto result = solver.solve(data, options);
  REQUIRE(result.vdc.size() == 2);
  // At V2=1: mismatch=0.1 and d(calc-spec)/dV2=10-(-2)=12.
  CHECK(result.vdc[1] == Approx(1.0 + 0.1 / 12.0).margin(1e-9));
}

TEST_CASE("Audit D11: three-phase PV generator voltage setpoint is enforced",
          "[power_flow][math_audit][D11]") {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.buses = {
      make_three_phase_bus(1, BusType::SLACK),
      make_three_phase_bus(2, BusType::PV),
  };
  ThreePhaseACLine line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r1_pu = line.r0_pu = 0.01;
  line.x1_pu = line.x0_pu = 0.08;
  line.in_service = true;
  sys.lines = {line};
  ThreePhaseGenerator generator;
  generator.index = 1;
  generator.bus = 2;
  generator.phase_mask = PhaseMask::abc();
  generator.p_mw = 0.3;
  generator.vm_pu = 1.04;
  generator.in_service = true;
  sys.generators = {generator};

  const auto result = solve_three_phase_nr(sys);
  REQUIRE(result.converged);
  REQUIRE(result.bus_voltages.size() == 2);
  CHECK(result.bus_voltages[1].vm_a_pu == Approx(1.04).margin(1e-10));
  CHECK(result.bus_voltages[1].vm_b_pu == Approx(1.04).margin(1e-10));
  CHECK(result.bus_voltages[1].vm_c_pu == Approx(1.04).margin(1e-10));
}

TEST_CASE("Audit D3: GLL Armijo and PTC-SER gamma have their stated mathematics",
          "[power_flow][math_audit][D3]") {
  hacdcpf::powerflow::NonmonotoneLineSearch gll(2, 0.1, 0.5);
  gll.push_merit(4.0);
  gll.push_merit(1.0);
  const auto accepted = gll.search(
      1.0, -2.0, 1, [](double) { return 3.0; });
  CHECK(accepted.accepted);
  CHECK(accepted.phi_reference == Approx(4.0));
  CHECK(accepted.alpha == Approx(1.0));

  hacdcpf::powerflow::NonmonotoneLineSearch monotone(1, 0.1, 0.5);
  monotone.push_merit(1.0);
  const auto rejected = monotone.search(
      1.0, -2.0, 3, [](double) { return 1.0; });
  CHECK_FALSE(rejected.accepted);

  hacdcpf::powerflow::PtcSerController square_root(1.0, 1e-4, 100.0, 0.5);
  hacdcpf::powerflow::PtcSerController linear(1.0, 1e-4, 100.0, 1.0);
  square_root.update(4.0, 1.0);
  linear.update(4.0, 1.0);
  CHECK(square_root.dt() == Approx(2.0));
  CHECK(linear.dt() == Approx(4.0));
}

TEST_CASE("Audit D14: Newton workspace reuse cannot contaminate later solves",
          "[power_flow][math_audit][D14]") {
  auto make_case = [](double load_mw, bool add_third_bus) {
    HybridPowerSystem sys;
    sys.base_mva = 100.0;
    sys.ac.base_mva = 100.0;

    ACBus slack;
    slack.index = 1;
    slack.bus_type = BusType::SLACK;
    slack.vm_pu = 1.0;
    slack.in_service = true;
    ACBus load;
    load.index = 2;
    load.bus_type = BusType::PQ;
    load.pd_mw = load_mw;
    load.qd_mvar = 0.35 * load_mw;
    load.in_service = true;
    sys.ac.buses = {slack, load};

    ACBranch line;
    line.index = 1;
    line.from_bus = 1;
    line.to_bus = 2;
    line.r_pu = 0.02;
    line.x_pu = 0.12;
    line.in_service = true;
    sys.ac.branches = {line};

    if (add_third_bus) {
      ACBus extra = load;
      extra.index = 3;
      extra.pd_mw = 0.4 * load_mw;
      extra.qd_mvar = 0.1 * load_mw;
      sys.ac.buses.push_back(extra);
      ACBranch second = line;
      second.index = 2;
      second.from_bus = 2;
      second.to_bus = 3;
      sys.ac.branches.push_back(second);
    }

    Generator generator;
    generator.index = 1;
    generator.bus = 1;
    generator.is_slack = true;
    generator.in_service = true;
    sys.ac.generators = {generator};
    return build_power_flow_problem(sys);
  };

  const auto primary = make_case(35.0, false);
  const auto different_layout = make_case(20.0, true);
  hacdcpf::powerflow::NewtonSolver solver;
  const auto first = solver.solve(primary.network, primary.options);
  const auto intervening =
      solver.solve(different_layout.network, different_layout.options);
  const auto after_resize = solver.solve(primary.network, primary.options);
  const auto consecutive = solver.solve(primary.network, primary.options);

  REQUIRE(first.converged);
  REQUIRE(intervening.converged);
  REQUIRE(after_resize.converged);
  REQUIRE(consecutive.converged);

  auto vector_bits = [](const std::vector<double>& values) {
    std::vector<std::uint64_t> bits;
    bits.reserve(values.size());
    for (const double value : values) {
      bits.push_back(std::bit_cast<std::uint64_t>(value));
    }
    return bits;
  };
  const auto first_vm = vector_bits(first.vm);
  const auto first_va = vector_bits(first.va);
  CHECK(vector_bits(after_resize.vm) == first_vm);
  CHECK(vector_bits(after_resize.va) == first_va);
  CHECK(vector_bits(consecutive.vm) == first_vm);
  CHECK(vector_bits(consecutive.va) == first_va);
  CHECK(std::bit_cast<std::uint64_t>(after_resize.residual) ==
        std::bit_cast<std::uint64_t>(first.residual));
  CHECK(std::bit_cast<std::uint64_t>(consecutive.residual) ==
        std::bit_cast<std::uint64_t>(first.residual));
  CHECK(after_resize.iterations == first.iterations);
  CHECK(consecutive.iterations == first.iterations);
}
