#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/external_grid_io.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

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

TEST_CASE("OpenDSS file loader expands Redirect libraries and richer devices",
          "[io][external][opendss][include]") {
  const auto dir = temp_file("opendss_redirect_case_dir");
  std::filesystem::create_directories(dir);
  const auto master = dir / "Master.dss";
  {
    std::ofstream(dir / "LineCodes.dss")
        << "New LineCode.LC1 nphases=3 r1=0.4 x1=0.8 r0=1.2 x0=2.4 "
           "c1=10 c0=4 units=mi normamps=300\n";
    std::ofstream(dir / "Network.dss")
        << "New Line.L12 bus1=source.1.2.3 bus2=loadbus.1.2.3 phases=3 "
           "linecode=LC1 length=5280 units=ft\n"
        << "New Capacitor.C1 bus1=loadbus.1.2.3 kv=12.47 kvar=600\n"
        << "New Load.L1 bus1=loadbus.1.2.3 kv=12.47 kw=1000 pf=0.8\n";
    std::ofstream(master)
        << "Clear\n"
        << "New Circuit.Include bus1=source.1.2.3 basekv=12.47 pu=1.0\n"
        << "Redirect LineCodes.dss\n"
        << "Redirect Network.dss\n";
  }

  hacdcpf::io::OpenDSSImportOptions options;
  options.default_base_mva = 10.0;
  const auto report = hacdcpf::io::load_opendss_with_report(master, options);

  REQUIRE(report.system.ac.buses.size() == 2);
  REQUIRE(report.system.ac.branches.size() == 1);
  REQUIRE(report.system.ac.shunts.size() == 1);
  REQUIRE(report.system.ac.loads.size() == 1);
  const double zbase = 12.47 * 12.47 / 10.0;
  CHECK_THAT(report.system.ac.branches.front().r_pu,
             WithinAbs(0.4 / zbase, 1e-10));
  CHECK_THAT(report.system.ac.branches.front().x_pu,
             WithinAbs(0.8 / zbase, 1e-10));
  CHECK_THAT(report.system.ac.shunts.front().bs_mvar,
             WithinAbs(0.6, 1e-12));
  CHECK_THAT(report.system.ac.loads.front().p_mw, WithinAbs(1.0, 1e-12));
  CHECK_THAT(report.system.ac.loads.front().q_mvar, WithinAbs(0.75, 1e-12));

  std::filesystem::remove_all(dir);
}

TEST_CASE("OpenDSS text export round-trips through the text importer",
          "[io][external][opendss][roundtrip]") {
  const auto sys = make_external_io_case();
  const auto text = hacdcpf::io::to_opendss(sys);

  CHECK_THAT(text, ContainsSubstring("New Circuit."));
  CHECK_THAT(text, ContainsSubstring("New Line.L12"));
  CHECK_THAT(text, ContainsSubstring("New Load."));
  CHECK_THAT(text, ContainsSubstring("New Generator.PV2"));
  CHECK_THAT(text, !ContainsSubstring("Set baseMVA="));
  CHECK_THAT(text, ContainsSubstring("Set VoltageBases=[12.47]"));
  CHECK_THAT(text, ContainsSubstring("CalcVoltageBases"));

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

TEST_CASE("OpenDSS ckt5 parenthesized transformer keeps the active feeder connected",
          "[io][external][opendss][ckt5][regression]") {
  const auto master =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) /
      "data/test_cases/electricdss-code-r4166-trunk-Distrib-EPRITestCircuits/"
      "ckt5/Master_ckt5.dss";

  hacdcpf::io::OpenDSSImportOptions options;
  options.mode = hacdcpf::io::ImportMode::Permissive;
  const auto report = hacdcpf::io::load_opendss_with_report(master, options);
  const auto& sys = report.system;

  REQUIRE(sys.ac.buses.size() == 3010);
  REQUIRE(sys.ac.branches.size() == 2949);
  REQUIRE(sys.ac.switches.size() == 73);
  REQUIRE(sys.ac.loads.size() == 1379);

  const auto active_bus_count = std::count_if(
      sys.ac.buses.begin(), sys.ac.buses.end(),
      [](const hacdcpf::ACBus& bus) { return bus.in_service; });
  CHECK(active_bus_count == 2998);
  CHECK(std::none_of(sys.ac.buses.begin(), sys.ac.buses.end(),
                     [](const hacdcpf::ACBus& bus) {
                       return !bus.name.empty() &&
                              (bus.name.front() == '(' || bus.name.back() == ')');
                     }));

  const auto islands = hacdcpf::powerflow::detect_islands(sys);
  REQUIRE(islands.size() == 1);
  CHECK(islands.front().has_ac_slack);
  CHECK(islands.front().has_generators);
  CHECK(islands.front().ac_buses.size() == active_bus_count);

  const auto projected = hacdcpf::project_to_canonical_models(sys);
  REQUIRE(projected.bus_merge_map.has_value());
  const std::vector<double> projected_voltage(projected.ac.buses.size(), 1.0);
  const auto restored_voltage = hacdcpf::unproject_bus_vector(
      projected_voltage, *projected.bus_merge_map,
      hacdcpf::BusVectorSemantics::Intensive);
  REQUIRE(restored_voltage.size() == sys.ac.buses.size());
  for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) {
    CHECK_THAT(restored_voltage[i],
               WithinAbs(sys.ac.buses[i].in_service ? 1.0 : 0.0, 1e-12));
  }
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

TEST_CASE("GridLAB-D capacitor objects import as shunt susceptance",
          "[io][external][gridlabd][capacitor]") {
  const std::string glm = R"(
object meter {
  name source;
  nominal_voltage 7200;
  bustype SWING;
};
object capacitor {
  name cap1;
  parent source;
  nominal_voltage 7200;
  capacitor_A 0.15 MVAr;
  capacitor_B 0.15 MVAr;
  capacitor_C 0.15 MVAr;
  switchA CLOSED;
  switchB CLOSED;
  switchC CLOSED;
};
)";

  hacdcpf::io::GridLABDImportOptions options;
  options.default_base_mva = 10.0;
  const auto sys = hacdcpf::io::from_gridlabd(glm, options);
  REQUIRE(sys.ac.shunts.size() == 1);
  CHECK(sys.ac.shunts.front().bus == 1);
  CHECK_THAT(sys.ac.shunts.front().bs_mvar, WithinAbs(0.45, 1e-12));
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

TEST_CASE("GridLAB-D no-unit line lengths default to feet",
          "[io][external][gridlabd][units]") {
  const std::string glm = R"(
object meter {
  name source;
  nominal_voltage 7200;
  bustype SWING;
};
object load {
  name loadbus;
  nominal_voltage 7200;
  constant_power_A 100000+30000j;
};
object line_configuration {
  name cfg_l12;
  z11 0.4+0.8j Ohm/mile;
  z12 0.1+0.2j Ohm/mile;
};
object overhead_line {
  name l12;
  from source;
  to loadbus;
  length 5280;
  configuration cfg_l12;
};
)";

  hacdcpf::io::GridLABDImportOptions options;
  options.default_base_mva = 10.0;
  const auto sys = hacdcpf::io::from_gridlabd(glm, options);

  REQUIRE(sys.ac.branches.size() == 1);
  CHECK_THAT(sys.ac.branches.front().length_km,
             WithinAbs(1.609344, 1e-9));
  CHECK_THAT(sys.ac.branches.front().r_pu,
             WithinAbs(0.30 / (12.470765814495916 * 12.470765814495916 / 10.0),
                       1e-10));
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

TEST_CASE("GridLAB-D transformer complex impedance preserves declared bus bases",
          "[io][external][gridlabd][transformer][regression]") {
  const std::string glm = R"(
object meter {
  name source;
  nominal_voltage 7200;
  bustype SWING;
};
object node {
  name loadbus;
  nominal_voltage 240;
};
object transformer_configuration {
  name cfg_t1;
  power_rating 500;
  primary_voltage 13200;
  secondary_voltage 480;
  impedance 0.0075+.075j;
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
  const auto report = hacdcpf::io::from_gridlabd_with_report(glm, options);

  REQUIRE(report.system.ac.buses.size() == 2);
  REQUIRE(report.system.ac.branches.size() == 1);
  const auto& branch = report.system.ac.branches.front();
  CHECK_THAT(branch.r_pu, WithinAbs(0.15, 1e-12));
  CHECK_THAT(branch.x_pu, WithinAbs(1.5, 1e-12));
  CHECK_THAT(report.system.ac.buses[0].base_kv,
             WithinAbs(7200.0 * std::sqrt(3.0) / 1000.0, 1e-12));
  CHECK_THAT(report.system.ac.buses[1].base_kv,
             WithinAbs(240.0 * std::sqrt(3.0) / 1000.0, 1e-12));
}

TEST_CASE("GridLAB-D line-to-line constant powers enter balanced load aggregate",
          "[io][external][gridlabd][load][delta][regression]") {
  const std::string glm = R"(
object meter {
  name source;
  nominal_voltage 7200;
  bustype SWING;
};
object load {
  name delta_load;
  parent source;
  nominal_voltage 7200;
  constant_power_A 100000+10000j;
  constant_power_AB 200000+20000j;
  constant_power_BC 300000+30000j;
  constant_power_CA 400000+40000j;
};
)";

  const auto report = hacdcpf::io::from_gridlabd_with_report(glm);
  REQUIRE(report.system.ac.loads.size() == 1);
  CHECK_THAT(report.system.ac.loads.front().p_mw, WithinAbs(1.0, 1e-12));
  CHECK_THAT(report.system.ac.loads.front().q_mvar, WithinAbs(0.1, 1e-12));
  REQUIRE_FALSE(report.warnings.empty());
  CHECK_THAT(report.warnings.back(), ContainsSubstring("constant_power_AB/BC/CA"));
}

TEST_CASE("LVNT GridLAB-D cases import nonzero transformers and converge",
          "[io][external][gridlabd][lvnt][regression]") {
  const auto root = std::filesystem::path(HACDCPF_PROJECT_ROOT) /
                    "data/test_cases/LVNT Model/LVNT Model";
  const std::vector<std::string> cases = {
      "Network_Model.glm", "Network_Model_Case1.glm", "Network_Model_Case2.glm"};

  for (const auto& filename : cases) {
    DYNAMIC_SECTION(filename) {
      const auto path = root / filename;
      if (!std::filesystem::exists(path)) SKIP("LVNT fixture is not available");

      hacdcpf::io::GridLABDImportOptions import_options;
      import_options.default_base_mva = 100.0;
      const auto report =
          hacdcpf::io::load_gridlabd_with_report(path, import_options);
      REQUIRE(report.system.ac.buses.size() == 1420);

      std::size_t transformer_count = 0;
      std::size_t zero_impedance_transformers = 0;
      for (const auto& branch : report.system.ac.branches) {
        if (branch.sn_mva <= 0.0 && branch.vn_hv_kv <= 0.0 &&
            branch.vn_lv_kv <= 0.0) {
          continue;
        }
        ++transformer_count;
        if (std::hypot(branch.r_pu, branch.x_pu) <= 1e-12) {
          ++zero_impedance_transformers;
        }
      }
      CHECK(transformer_count == 70);
      CHECK(zero_impedance_transformers == 0);

      hacdcpf::PowerFlowOptions pf_options;
      pf_options.max_iter = 100;
      pf_options.tol = 1e-7;
      const auto pf = hacdcpf::solve_power_flow(report.system, pf_options);
      CHECK(pf.converged);
    }
  }
}

TEST_CASE("GridLAB-D topology links import as rich switches for projection",
          "[io][external][gridlabd][switch]") {
  const std::string glm = R"(
object meter {
  name source;
  nominal_voltage 7200;
  bustype SWING;
};
object node {
  name tap;
  parent source;
  nominal_voltage 7200;
};
object load {
  name loadbus;
  nominal_voltage 7200;
  constant_power_A 100000+30000j;
  constant_power_B 100000+30000j;
  constant_power_C 100000+30000j;
};
object switch {
  name sw_tap_load;
  from tap;
  to loadbus;
  status CLOSED;
};
object fuse {
  name open_tie;
  from source;
  to loadbus;
  status OPEN;
};
)";

  hacdcpf::io::GridLABDImportOptions options;
  options.default_base_mva = 10.0;
  const auto sys = hacdcpf::io::from_gridlabd(glm, options);

  REQUIRE(sys.ac.buses.size() == 3);
  CHECK(sys.ac.branches.empty());
  REQUIRE(sys.ac.switches.size() == 3);
  CHECK(sys.ac.switches[0].name == "parent_link_tap");
  CHECK(sys.ac.switches[0].closed);
  CHECK(sys.ac.switches[1].name == "sw_tap_load");
  CHECK(sys.ac.switches[1].closed);
  CHECK(sys.ac.switches[2].name == "open_tie");
  CHECK_FALSE(sys.ac.switches[2].closed);

  const auto projected =
      hacdcpf::project_to_canonical_models(sys, /*strip_dead=*/false);
  REQUIRE(projected.ac.buses.size() == 1);
  CHECK(projected.ac.branches.empty());
  REQUIRE(projected.ac.loads.size() == 1);
  CHECK(projected.ac.loads.front().bus == 1);
}

TEST_CASE("Canonical projection keeps physical short lines when GridLAB-D switches exist",
          "[io][external][gridlabd][projection][regression]") {
  hacdcpf::HybridPowerSystem sys;
  sys.name = "short_line_with_switch";
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  sys.ac.buses = {
      make_bus(1, hacdcpf::BusType::SLACK, 12.47),
      make_bus(2, hacdcpf::BusType::PQ, 12.47),
      make_bus(3, hacdcpf::BusType::PQ, 12.47),
  };

  hacdcpf::ACBranch short_line;
  short_line.index = 1;
  short_line.name = "real_short_line";
  short_line.from_bus = 1;
  short_line.to_bus = 2;
  short_line.r_pu = hacdcpf::kBusMergeZThreshold * 0.5;
  short_line.x_pu = hacdcpf::kBusMergeZThreshold * 0.5;
  short_line.b_pu = 0.0;
  short_line.tap = 1.0;
  short_line.in_service = true;
  sys.ac.branches = {short_line};

  hacdcpf::Switch sw;
  sw.index = 1;
  sw.name = "closed_switch";
  sw.bus_from = 2;
  sw.bus_to = 3;
  sw.in_service = true;
  sw.closed = true;
  sys.ac.switches = {sw};

  const auto projected =
      hacdcpf::project_to_canonical_models(sys, /*strip_dead=*/false);

  REQUIRE(projected.ac.buses.size() == 2);
  REQUIRE(projected.ac.branches.size() == 1);
  const auto& kept = projected.ac.branches.front();
  CHECK(kept.name == "real_short_line");
  CHECK(kept.from_bus != kept.to_bus);
  CHECK_THAT(kept.r_pu, WithinAbs(short_line.r_pu, 1e-15));
  CHECK_THAT(kept.x_pu, WithinAbs(short_line.x_pu, 1e-15));

  REQUIRE(projected.bus_merge_map.has_value());
  bool merged_switch_endpoints = false;
  bool kept_short_line_endpoint = false;
  for (const auto& group : projected.bus_merge_map->groups) {
    const std::unordered_set<int> members(group.begin(), group.end());
    if (members.count(2) && members.count(3)) merged_switch_endpoints = true;
    if (members.count(1) && !members.count(2) && !members.count(3)) {
      kept_short_line_endpoint = true;
    }
  }
  CHECK(merged_switch_endpoints);
  CHECK(kept_short_line_endpoint);
}

TEST_CASE("GridLAB-D GC taxonomy feeder projection preserves imported line objects",
          "[io][external][gridlabd][projection][taxonomy]") {
  const auto path = std::filesystem::path(HACDCPF_TEST_DATA_DIR) /
                    "test_cases/gridlab-d-code-r5643-Taxonomy_Feeders/GC-12.47-1.glm";
  if (!std::filesystem::exists(path)) {
    SKIP("GridLAB-D taxonomy feeder fixture is not available");
  }

  hacdcpf::io::GridLABDImportOptions options;
  options.mode = hacdcpf::io::ImportMode::Permissive;
  options.default_base_mva = 10.0;
  const auto sys = hacdcpf::io::load_gridlabd(path, options);

  REQUIRE(sys.ac.branches.size() >= 18);
  REQUIRE(sys.ac.switches.size() >= 5);

  const auto projected =
      hacdcpf::project_to_canonical_models(sys, /*strip_dead=*/false);

  std::unordered_set<std::string> projected_branch_names;
  for (const auto& br : projected.ac.branches) {
    projected_branch_names.insert(br.name);
  }

  CHECK(projected_branch_names.count("GC-12-47-1_ul_13") == 1);
  CHECK(projected_branch_names.count("GC-12-47-1_ul_14") == 1);
  CHECK(projected_branch_names.count("GC-12-47-1_ul_18") == 1);
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
