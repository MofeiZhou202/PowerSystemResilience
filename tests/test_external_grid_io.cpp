#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/external_grid_io.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

hacdcpf::ACBus make_bus(int index,
                        hacdcpf::BusType type,
                        double base_kv,
                        double pd_mw = 0.0,
                        double qd_mvar = 0.0) {
  hacdcpf::ACBus bus;
  bus.index = index;
  bus.name = "Bus" + std::to_string(index);
  bus.bus_type = type;
  bus.base_kv = base_kv;
  bus.pd_mw = pd_mw;
  bus.qd_mvar = qd_mvar;
  bus.vm_pu = 1.0;
  bus.in_service = true;
  return bus;
}

hacdcpf::HybridPowerSystem make_external_io_case() {
  hacdcpf::HybridPowerSystem sys;
  sys.name = "external_io_roundtrip";
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  sys.ac.freq_hz = 50.0;
  sys.ac.buses = {
      make_bus(1, hacdcpf::BusType::SLACK, 12.47),
      make_bus(2, hacdcpf::BusType::PQ, 12.47),
      make_bus(3, hacdcpf::BusType::PQ, 12.47),
  };

  hacdcpf::ACBranch br1;
  br1.index = 1;
  br1.name = "L12";
  br1.from_bus = 1;
  br1.to_bus = 2;
  br1.r_pu = 0.010;
  br1.x_pu = 0.030;
  br1.r0_pu = 0.030;
  br1.x0_pu = 0.090;
  br1.tap = 1.0;
  br1.length_km = 1.0;
  br1.in_service = true;

  hacdcpf::ACBranch br2;
  br2.index = 2;
  br2.name = "L23";
  br2.from_bus = 2;
  br2.to_bus = 3;
  br2.r_pu = 0.012;
  br2.x_pu = 0.028;
  br2.tap = 1.0;
  br2.length_km = 1.0;
  br2.in_service = true;
  sys.ac.branches = {br1, br2};

  hacdcpf::Generator gen;
  gen.index = 1;
  gen.name = "Slack";
  gen.bus = 1;
  gen.is_slack = true;
  gen.in_service = true;
  gen.vg_pu = 1.0;
  gen.pmax_mw = 100.0;
  gen.qmax_mvar = 100.0;
  gen.qmin_mvar = -100.0;
  sys.ac.generators.push_back(gen);

  hacdcpf::Load load;
  load.index = 1;
  load.name = "Load3";
  load.bus = 3;
  load.in_service = true;
  load.p_mw = 1.2;
  load.q_mvar = 0.45;
  sys.ac.loads.push_back(load);

  hacdcpf::StaticGenerator sgen;
  sgen.index = 1;
  sgen.name = "PV2";
  sgen.bus = 2;
  sgen.in_service = true;
  sgen.p_mw = 0.25;
  sgen.q_mvar = 0.04;
  sys.ac.static_generators.push_back(sgen);
  return sys;
}

std::filesystem::path temp_file(const std::string& name) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("hacdcpf_" + name + "_" + std::to_string(stamp));
}

}  // namespace

TEST_CASE("OpenDSS text import builds a solvable AC snapshot",
          "[io][external][opendss][import]") {
  const std::string dss = R"(
Clear
New Circuit.Tiny bus1=source.1.2.3 basekv=12.47 pu=1.0 angle=0 phases=3
New Line.L12 bus1=source.1.2.3 bus2=loadbus.1.2.3 phases=3 r1=0.20 x1=0.40 r0=0.60 x0=1.20 length=1.5 units=km
New Load.LoadBus bus1=loadbus.1.2.3 phases=3 kv=12.47 kw=1200 kvar=450 model=1
New Generator.PV bus1=loadbus.1.2.3 phases=3 kv=12.47 kw=250 kvar=40 model=1
Solve
)";

  hacdcpf::io::OpenDSSImportOptions options;
  options.default_base_mva = 10.0;
  const auto report = hacdcpf::io::from_opendss_with_report(dss, options);

  REQUIRE(report.system.ac.buses.size() == 2);
  REQUIRE(report.system.ac.branches.size() == 1);
  REQUIRE(report.system.ac.loads.size() == 1);
  REQUIRE(report.system.ac.static_generators.size() == 1);
  CHECK(report.system.ac.buses.front().bus_type == hacdcpf::BusType::SLACK);
  CHECK_THAT(report.system.ac.branches.front().r_pu,
             WithinAbs((0.20 * 1.5) / (12.47 * 12.47 / 10.0), 1e-10));
  CHECK_THAT(report.system.ac.loads.front().p_mw, WithinAbs(1.2, 1e-12));
  CHECK_THAT(report.system.ac.static_generators.front().p_mw,
             WithinAbs(0.25, 1e-12));

  const auto pf = hacdcpf::solve_power_flow(report.system);
  CHECK(pf.converged);
}

TEST_CASE("OpenDSS text export round-trips through the text importer",
          "[io][external][opendss][roundtrip]") {
  const auto sys = make_external_io_case();
  const auto text = hacdcpf::io::to_opendss(sys);

  CHECK_THAT(text, ContainsSubstring("New Circuit."));
  CHECK_THAT(text, ContainsSubstring("New Line.L12"));
  CHECK_THAT(text, ContainsSubstring("New Load."));
  CHECK_THAT(text, ContainsSubstring("New Generator.PV2"));

  hacdcpf::io::OpenDSSImportOptions options;
  options.default_base_mva = 10.0;
  const auto restored = hacdcpf::io::from_opendss(text, options);
  CHECK(restored.ac.buses.size() == sys.ac.buses.size());
  CHECK(restored.ac.branches.size() == sys.ac.branches.size());
  CHECK(restored.ac.loads.size() == 1);
  CHECK(restored.ac.static_generators.size() == 1);
  CHECK_THAT(restored.ac.loads.front().p_mw, WithinAbs(1.2, 1e-9));
}

TEST_CASE("OpenDSS transformer text maps to branch nameplate metadata",
          "[io][external][opendss][transformer]") {
  const std::string dss = R"(
Clear
New Circuit.Tr bus1=source.1.2.3 basekv=35 pu=1.0 phases=3
New Transformer.T1 phases=3 windings=2 buses=[source.1.2.3,loadbus.1.2.3] kvs=[35,10] kvas=[10000,10000] %r=0.5 xhl=5.0 conns=[wye,wye]
New Load.LoadLV bus1=loadbus.1.2.3 phases=3 kv=10 kw=1000 kvar=300
)";

  hacdcpf::io::OpenDSSImportOptions options;
  options.default_base_mva = 10.0;
  const auto sys = hacdcpf::io::from_opendss(dss, options);
  REQUIRE(sys.ac.buses.size() == 2);
  REQUIRE(sys.ac.branches.size() == 1);
  const auto& br = sys.ac.branches.front();
  CHECK_THAT(br.r_pu, WithinAbs(0.005, 1e-12));
  CHECK_THAT(br.x_pu, WithinAbs(0.05, 1e-12));
  CHECK_THAT(br.vn_hv_kv, WithinAbs(35.0, 1e-12));
  CHECK_THAT(br.vn_lv_kv, WithinAbs(10.0, 1e-12));
  CHECK_THAT(br.sn_mva, WithinAbs(10.0, 1e-12));
}

TEST_CASE("GridLAB-D text import parses generated load and line objects",
          "[io][external][gridlabd][import]") {
  const std::string glm = R"(
module powerflow;
object meter {
  name source;
  phases ABCN;
  nominal_voltage 7200;
  bustype SWING;
};
object load {
  name loadbus;
  phases ABCN;
  nominal_voltage 7200;
  constant_power_A 400000+150000j;
  constant_power_B 400000+150000j;
  constant_power_C 400000+150000j;
};
object line_configuration {
  name cfg_l12;
  z11 0.4+0.8j Ohm/mile;
  z12 0.1+0.2j Ohm/mile;
  z13 0.1+0.2j Ohm/mile;
  z22 0.4+0.8j Ohm/mile;
  z33 0.4+0.8j Ohm/mile;
};
object overhead_line {
  name l12;
  phases ABC;
  from source;
  to loadbus;
  length 1 mile;
  configuration cfg_l12;
};
)";

  hacdcpf::io::GridLABDImportOptions options;
  options.default_base_mva = 10.0;
  const auto report = hacdcpf::io::from_gridlabd_with_report(glm, options);

  REQUIRE(report.system.ac.buses.size() == 2);
  REQUIRE(report.system.ac.loads.size() == 1);
  REQUIRE(report.system.ac.branches.size() == 1);
  CHECK(report.system.ac.buses.front().bus_type == hacdcpf::BusType::SLACK);
  CHECK_THAT(report.system.ac.loads.front().p_mw, WithinAbs(1.2, 1e-12));
  CHECK_THAT(report.system.ac.loads.front().q_mvar, WithinAbs(0.45, 1e-12));
  CHECK_THAT(report.system.ac.branches.front().length_km,
             WithinAbs(1.609344, 1e-12));
  CHECK_THAT(report.system.ac.branches.front().r_ohm_per_km,
             WithinAbs((0.4 - 0.1) / 1.609344, 1e-12));
}

TEST_CASE("GridLAB-D text export round-trips through the GLM importer",
          "[io][external][gridlabd][roundtrip]") {
  auto sys = make_external_io_case();
  sys.ac.static_generators.clear();

  hacdcpf::io::GridLABDExportOptions export_options;
  export_options.model_name = "external_io";
  export_options.include_load_recorders = false;
  export_options.include_branch_recorders = false;
  const auto text = hacdcpf::io::to_gridlabd(sys, export_options);

  CHECK_THAT(text, ContainsSubstring("object line_configuration"));
  CHECK_THAT(text, ContainsSubstring("constant_power_A"));

  hacdcpf::io::GridLABDImportOptions import_options;
  import_options.default_base_mva = 10.0;
  const auto restored = hacdcpf::io::from_gridlabd(text, import_options);
  CHECK(restored.ac.buses.size() == sys.ac.buses.size());
  CHECK(restored.ac.branches.size() == sys.ac.branches.size());
  REQUIRE(restored.ac.loads.size() == 1);
  CHECK_THAT(restored.ac.loads.front().p_mw, WithinAbs(1.2, 1e-9));
}

TEST_CASE("GridLAB-D transformer text maps to branch nameplate metadata",
          "[io][external][gridlabd][transformer]") {
  const std::string glm = R"(
object meter {
  name source;
  nominal_voltage 20207.2594216369;
  bustype SWING;
};
object load {
  name loadbus;
  nominal_voltage 5773.50269189626;
  constant_power_A 333333.333333333+100000j;
  constant_power_B 333333.333333333+100000j;
  constant_power_C 333333.333333333+100000j;
};
object transformer_configuration {
  name cfg_t1;
  connect_type WYE_WYE;
  power_rating 10000;
  primary_voltage 35000;
  secondary_voltage 10000;
  resistance 0.005;
  reactance 0.05;
};
object transformer {
  name t1;
  from source;
  to loadbus;
  configuration cfg_t1;
};
)";

  hacdcpf::io::GridLABDImportOptions options;
  options.default_base_mva = 10.0;
  const auto sys = hacdcpf::io::from_gridlabd(glm, options);
  REQUIRE(sys.ac.buses.size() == 2);
  REQUIRE(sys.ac.branches.size() == 1);
  CHECK_THAT(sys.ac.branches.front().r_pu, WithinAbs(0.005, 1e-12));
  CHECK_THAT(sys.ac.branches.front().x_pu, WithinAbs(0.05, 1e-12));
  CHECK_THAT(sys.ac.branches.front().vn_hv_kv, WithinAbs(35.0, 1e-12));
  CHECK_THAT(sys.ac.branches.front().vn_lv_kv, WithinAbs(10.0, 1e-12));
}

TEST_CASE("External grid IO safe load variants report missing files",
          "[io][external][safe_io]") {
  const auto missing = temp_file("missing_external_format");
  std::filesystem::remove(missing);
  const auto dss = hacdcpf::io::try_load_opendss(missing);
  const auto glm = hacdcpf::io::try_load_gridlabd(missing);
  REQUIRE_FALSE(dss);
  REQUIRE_FALSE(glm);
  CHECK(dss.error().code == hacdcpf::ErrorCode::FileNotFound);
  CHECK(glm.error().code == hacdcpf::ErrorCode::FileNotFound);
}

TEST_CASE("External grid IO save/load APIs preserve the JSON-style workflow",
          "[io][external][files]") {
  const auto sys = make_external_io_case();
  const auto dss_path = temp_file("external_io_roundtrip.dss");
  const auto glm_path = temp_file("external_io_roundtrip.glm");

  hacdcpf::io::save_opendss(sys, dss_path);
  hacdcpf::io::GridLABDExportOptions glm_options;
  glm_options.include_load_recorders = false;
  glm_options.include_branch_recorders = false;
  hacdcpf::io::save_gridlabd(sys, glm_path, glm_options);

  const auto dss_loaded = hacdcpf::io::load_opendss(dss_path);
  const auto glm_loaded = hacdcpf::io::load_gridlabd(glm_path);
  CHECK(dss_loaded.ac.buses.size() == sys.ac.buses.size());
  CHECK(glm_loaded.ac.buses.size() == sys.ac.buses.size());

  std::filesystem::remove(dss_path);
  std::filesystem::remove(glm_path);
}
