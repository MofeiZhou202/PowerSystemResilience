#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#ifndef _WIN32
#include <unistd.h>
#endif

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/gridlabd_bridge.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace {

namespace fs = std::filesystem;

hacdcpf::ACBus bus(int index,
                   hacdcpf::BusType type = hacdcpf::BusType::PQ,
                   double base_kv = 12.47,
                   double pd_mw = 0.0,
                   double qd_mvar = 0.0) {
  hacdcpf::ACBus b;
  b.index = index;
  b.bus_type = type;
  b.base_kv = base_kv;
  b.pd_mw = pd_mw;
  b.qd_mvar = qd_mvar;
  b.vm_pu = 1.0;
  b.va_deg = 0.0;
  b.in_service = true;
  b.name = "Bus" + std::to_string(index);
  return b;
}

hacdcpf::ACBranch branch(int index,
                         int from,
                         int to,
                         double r_pu,
                         double x_pu) {
  hacdcpf::ACBranch br;
  br.index = index;
  br.from_bus = from;
  br.to_bus = to;
  br.r_pu = r_pu;
  br.x_pu = x_pu;
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.shift_deg = 0.0;
  br.in_service = true;
  br.name = "Line" + std::to_string(index);
  return br;
}

hacdcpf::HybridPowerSystem two_bus_component_case() {
  hacdcpf::HybridPowerSystem sys;
  sys.name = "gridlabd_two_bus_component";
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  sys.ac.freq_hz = 50.0;
  sys.ac.buses = {
      bus(1, hacdcpf::BusType::SLACK, 12.47),
      bus(2, hacdcpf::BusType::PQ, 12.47, 1.2, 0.45),
  };
  sys.ac.branches = {branch(1, 1, 2, 0.012, 0.032)};
  hacdcpf::Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.0;
  sys.ac.generators.push_back(g);
  return sys;
}

hacdcpf::HybridPowerSystem three_bus_radial_case() {
  auto sys = two_bus_component_case();
  sys.name = "gridlabd_three_bus_radial";
  sys.ac.buses.push_back(bus(3, hacdcpf::BusType::PQ, 12.47, 0.8, 0.25));
  sys.ac.branches.push_back(branch(2, 2, 3, 0.010, 0.025));
  return sys;
}

hacdcpf::HybridPowerSystem rich_projection_case() {
  hacdcpf::HybridPowerSystem sys;
  sys.name = "gridlabd_rich_projection";
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  sys.ac.buses = {
      bus(1, hacdcpf::BusType::SLACK, 12.47),
      bus(2, hacdcpf::BusType::PQ, 12.47),
      bus(3, hacdcpf::BusType::PQ, 12.47, 0.7, 0.2),
  };
  hacdcpf::Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  sys.ac.generators.push_back(g);

  hacdcpf::Transformer2W tr;
  tr.index = 1;
  tr.name = "T1";
  tr.hv_bus = 1;
  tr.lv_bus = 2;
  tr.sn_mva = 10.0;
  tr.vn_hv_kv = 12.47;
  tr.vn_lv_kv = 12.47;
  tr.vk_percent = 5.0;
  tr.vkr_percent = 0.5;
  tr.in_service = true;
  sys.ac.transformers_2w.push_back(tr);

  hacdcpf::Switch sw;
  sw.index = 1;
  sw.name = "SW23";
  sw.bus_from = 2;
  sw.bus_to = 3;
  sw.closed = false;
  sw.in_service = true;
  sys.ac.switches.push_back(sw);

  return sys;
}

hacdcpf::HybridPowerSystem transformer_stepdown_case(double tap = 1.0) {
  hacdcpf::HybridPowerSystem sys;
  sys.name = "gridlabd_transformer_stepdown";
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  sys.ac.freq_hz = 50.0;
  sys.ac.buses = {
      bus(1, hacdcpf::BusType::SLACK, 35.0),
      bus(2, hacdcpf::BusType::PQ, 10.0, 1.0, 0.30),
  };

  hacdcpf::ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.004;
  br.x_pu = 0.045;
  br.tap = tap;
  br.rate_a_mva = 10.0;
  br.in_service = true;
  br.name = "T35_10";
  sys.ac.branches = {br};

  hacdcpf::Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.0;
  sys.ac.generators.push_back(g);
  return sys;
}

hacdcpf::HybridPowerSystem static_generator_case() {
  auto sys = three_bus_radial_case();
  sys.name = "gridlabd_static_generator";

  hacdcpf::StaticGenerator sg;
  sg.index = 1;
  sg.bus = 3;
  sg.in_service = true;
  sg.p_mw = 0.25;
  sg.q_mvar = 0.05;
  sg.scaling = 1.0;
  sg.name = "PV_as_PQ_injection";
  sys.ac.static_generators.push_back(sg);
  return sys;
}

hacdcpf::HybridPowerSystem pv_bus_voltage_regulation_case() {
  auto sys = three_bus_radial_case();
  sys.name = "gridlabd_pv_bus_diagnostic";
  sys.ac.buses[2].bus_type = hacdcpf::BusType::PV;
  sys.ac.buses[2].pd_mw = 0.30;
  sys.ac.buses[2].qd_mvar = 0.10;
  sys.ac.buses[2].vm_pu = 1.04;

  hacdcpf::Generator g;
  g.index = 2;
  g.bus = 3;
  g.is_slack = false;
  g.in_service = true;
  g.pg_mw = 1.0;
  g.qg_mvar = 0.0;
  g.vg_pu = 1.04;
  g.qmax_mvar = 2.0;
  g.qmin_mvar = -2.0;
  sys.ac.generators.push_back(g);
  return sys;
}

fs::path temp_dir(const std::string& name) {
  long pid = 0;
#ifndef _WIN32
  pid = static_cast<long>(::getpid());
#endif
  const auto dir = fs::temp_directory_path() / ("hacdcpf_" + name + "_" +
                                                std::to_string(pid));
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

std::string read_file(const fs::path& path) {
  std::ifstream in(path);
  std::string text((std::istreambuf_iterator<char>(in)),
                   std::istreambuf_iterator<char>());
  return text;
}

void require_export_shape(const hacdcpf::io::GridLABDExportedSnapshot& exported,
                          std::size_t min_buses,
                          std::size_t min_exported_branches,
                          bool require_line_configuration = true) {
  REQUIRE(fs::exists(exported.glm_path));
  CHECK(exported.bus_mappings.size() >= min_buses);
  const auto exported_branch_count = std::count_if(
      exported.branch_mappings.begin(), exported.branch_mappings.end(),
      [](const hacdcpf::io::GridLABDBranchMapping& mapping) {
        return mapping.exported;
      });
  CHECK(static_cast<std::size_t>(exported_branch_count) >= min_exported_branches);
  const std::string glm = read_file(exported.glm_path);
  CHECK(glm.find("module powerflow") != std::string::npos);
  CHECK(glm.find("module tape") != std::string::npos);
  CHECK(glm.find("object recorder") != std::string::npos);
  if (require_line_configuration) {
    CHECK(glm.find("z11") != std::string::npos);
  }
}

std::size_t failed_item_count(const hacdcpf::io::GridLABDComparisonReport& report) {
  return static_cast<std::size_t>(std::count_if(
      report.items.begin(), report.items.end(),
      [](const hacdcpf::io::GridLABDComparisonItem& item) {
        return !item.passed;
      }));
}

void print_report_diagnostics(const hacdcpf::io::GridLABDComparisonReport& report,
                              std::size_t max_items = 12) {
  std::size_t printed = 0;
  for (const auto& item : report.items) {
    if (item.passed) continue;
    if (printed < max_items) {
      UNSCOPED_INFO(item.kind << " " << item.key << " hacdcpf="
                             << item.hacdcpf_value << " gridlabd="
                             << item.gridlabd_value << " diff="
                             << item.difference << " tol="
                             << item.tolerance);
    }
    ++printed;
  }
  if (printed > max_items) {
    UNSCOPED_INFO("additional failed comparison items suppressed: "
                  << (printed - max_items));
  }
  for (const auto& skipped : report.skipped) {
    UNSCOPED_INFO("skipped: " << skipped);
  }
  std::size_t printed_warnings = 0;
  std::size_t pv_generator_warnings = 0;
  constexpr std::size_t kMaxWarnings = 12;
  for (const auto& warning : report.warnings) {
    if (warning.find("PV voltage regulation is not represented") !=
        std::string::npos) {
      ++pv_generator_warnings;
      continue;
    }
    if (printed_warnings < kMaxWarnings) {
      UNSCOPED_INFO("warning: " << warning);
    }
    ++printed_warnings;
  }
  if (pv_generator_warnings > 0) {
    UNSCOPED_INFO("warning: " << pv_generator_warnings
                              << " non-slack generators exported as negative "
                                 "constant-power loads; PV voltage regulation "
                                 "is not represented in this GridLAB-D snapshot.");
  }
  if (printed_warnings > kMaxWarnings) {
    UNSCOPED_INFO("additional warnings suppressed: "
                  << (printed_warnings - kMaxWarnings));
  }
}

hacdcpf::io::GridLABDComparisonReport run_optional_external_compare(
    hacdcpf::HybridPowerSystem sys,
    const std::string& label,
    const hacdcpf::io::GridLABDComparisonOptions& input_options = {}) {
  const auto gridlabd = hacdcpf::io::discover_gridlabd();
  if (!gridlabd.available) {
    SKIP("GridLAB-D executable not available for " + label);
  }

  auto pf = hacdcpf::solve_power_flow(sys);
  REQUIRE(pf.converged);

  hacdcpf::io::GridLABDComparisonOptions options = input_options;
  options.export_options.model_name = "compare_" + label;
  options.vm_tolerance_pu = 2e-2;
  options.va_tolerance_deg = 2.0;
  options.branch_p_tolerance_mw = 0.10;
  options.branch_q_tolerance_mvar = 0.10;
  options.require_gridlabd = true;

  const auto report = hacdcpf::io::compare_gridlabd_snapshot(sys, options);
  INFO("GridLAB-D command: " << report.gridlabd_result.command);
  if (!report.passed) {
    print_report_diagnostics(report);
  }
  CHECK(report.gridlabd_run_attempted);
  CHECK(report.gridlabd_run_success);
  CHECK(report.passed);
  return report;
}

}  // namespace

TEST_CASE("GridLAB-D bridge discovers optional executable without requiring it",
          "[gridlabd][bridge]") {
  const auto discovered = hacdcpf::io::discover_gridlabd();
  if (!discovered.available) {
    CHECK(!discovered.diagnostics.empty());
  } else {
    CHECK(fs::exists(discovered.path));
  }
}

TEST_CASE("GridLAB-D bridge exports a two-bus component snapshot",
          "[gridlabd][bridge][export]") {
  const auto sys = two_bus_component_case();
  auto pf = hacdcpf::solve_power_flow(sys);
  REQUIRE(pf.converged);

  hacdcpf::io::GridLABDExportOptions options;
  options.model_name = "two_bus_component";
  const auto exported = hacdcpf::io::export_gridlabd_snapshot(
      sys, temp_dir("gridlabd_two_bus"), options);
  require_export_shape(exported, 2, 1);
  CHECK(exported.load_mappings.size() == 1);
  CHECK(exported.branch_mappings.front().origin_type == "ACBranch");
}

TEST_CASE("GridLAB-D bridge exports rich-to-canonical provenance",
          "[gridlabd][bridge][projection]") {
  const auto sys = rich_projection_case();
  const auto projected = hacdcpf::project_to_canonical_models(sys);
  REQUIRE(projected.branch_expand_map.has_value());
  CHECK(std::any_of(projected.branch_expand_map->entries.begin(),
                    projected.branch_expand_map->entries.end(),
                    [](const hacdcpf::BranchExpandEntry& entry) {
                      return entry.origin_type == hacdcpf::BranchOriginType::Switch;
                    }));

  hacdcpf::io::GridLABDExportOptions options;
  options.model_name = "rich_projection";
  const auto exported = hacdcpf::io::export_gridlabd_snapshot(
      sys, temp_dir("gridlabd_rich"), options);
  CHECK(exported.bus_mappings.size() >= 2);
  REQUIRE(!exported.branch_mappings.empty());
  CHECK(std::any_of(exported.branch_mappings.begin(),
                    exported.branch_mappings.end(),
                    [](const hacdcpf::io::GridLABDBranchMapping& mapping) {
                      return mapping.origin_type == "Transformer2W";
                    }));
}

TEST_CASE("GridLAB-D bridge exports transformer and parallel-equivalent metadata",
          "[gridlabd][bridge][export][transformer]") {
  hacdcpf::io::GridLABDExportOptions options;
  options.model_name = "transformer_stepdown";
  const auto exported = hacdcpf::io::export_gridlabd_snapshot(
      transformer_stepdown_case(), temp_dir("gridlabd_transformer"), options);
  require_export_shape(exported, 2, 1, false);
  REQUIRE(exported.branch_mappings.size() == 1);
  CHECK(exported.branch_mappings.front().exported_as_transformer);
  const std::string glm = read_file(exported.glm_path);
  CHECK(glm.find("object transformer_configuration") != std::string::npos);
  CHECK(glm.find("primary_voltage") != std::string::npos);
  CHECK(glm.find("secondary_voltage") != std::string::npos);
}

TEST_CASE("GridLAB-D bridge exports built-in feeder scale cases",
          "[gridlabd][bridge][export][case33]") {
  const auto case33 = hacdcpf::io::build_ac_only_version(
      hacdcpf::io::build_case33bw_acdc());
  hacdcpf::io::GridLABDExportOptions options;
  options.model_name = "case33bw_ac_scope";
  const auto exported = hacdcpf::io::export_gridlabd_snapshot(
      case33, temp_dir("gridlabd_case33"), options);
  require_export_shape(exported, 30, 30);

  const auto case69 = hacdcpf::io::build_ac_only_version(
      hacdcpf::io::build_case69_acdc());
  options.model_name = "case69_ac_scope";
  const auto exported69 = hacdcpf::io::export_gridlabd_snapshot(
      case69, temp_dir("gridlabd_case69"), options);
  require_export_shape(exported69, 60, 60);
}

TEST_CASE("GridLAB-D external comparison: two-bus and three-bus snapshots",
          "[gridlabd][compare][external]") {
  run_optional_external_compare(two_bus_component_case(), "two_bus");
  run_optional_external_compare(three_bus_radial_case(), "three_bus");
}

TEST_CASE("GridLAB-D external comparison: component ladder",
          "[gridlabd][compare][external][component]") {
  run_optional_external_compare(static_generator_case(), "static_generator");

  hacdcpf::io::GridLABDComparisonOptions transformer_options;
  transformer_options.compare_transformer_branch_flows = false;
  const auto transformer_report = run_optional_external_compare(
      transformer_stepdown_case(), "transformer_stepdown", transformer_options);
  CHECK(std::any_of(transformer_report.exported_snapshot.branch_mappings.begin(),
                    transformer_report.exported_snapshot.branch_mappings.end(),
                    [](const hacdcpf::io::GridLABDBranchMapping& mapping) {
                      return mapping.exported_as_transformer;
                    }));
}

TEST_CASE("GridLAB-D external diagnostic: PV bus voltage regulation is not exact yet",
          "[gridlabd][compare][external][diagnostic]") {
  const auto gridlabd = hacdcpf::io::discover_gridlabd();
  if (!gridlabd.available) {
    SKIP("GridLAB-D executable not available for PV-bus diagnostic");
  }

  hacdcpf::io::GridLABDComparisonOptions options;
  options.export_options.model_name = "compare_pv_bus_diagnostic";
  options.require_gridlabd = true;
  const auto report =
      hacdcpf::io::compare_gridlabd_snapshot(pv_bus_voltage_regulation_case(),
                                             options);
  INFO("GridLAB-D command: " << report.gridlabd_result.command);
  CHECK(report.gridlabd_run_attempted);
  CHECK(report.gridlabd_run_success);
  CHECK_FALSE(report.warnings.empty());
  CHECK(std::any_of(report.warnings.begin(), report.warnings.end(),
                    [](const std::string& warning) {
                      return warning.find("PV voltage regulation") !=
                             std::string::npos;
                    }));
  CHECK(failed_item_count(report) > 0);
}

TEST_CASE("GridLAB-D external comparison: distribution feeder ladder",
          "[gridlabd][compare][external][feeder]") {
  run_optional_external_compare(
      hacdcpf::io::build_ac_only_version(hacdcpf::io::build_case33bw_acdc()),
      "case33bw_ac");

  if (std::getenv("HACDCPF_GRIDLABD_RUN_LARGE") == nullptr) {
    SKIP("Set HACDCPF_GRIDLABD_RUN_LARGE=1 to run medium/large GridLAB-D feeder comparisons");
  }
  run_optional_external_compare(
      hacdcpf::io::build_ac_only_version(hacdcpf::io::build_case69_acdc()),
      "case69_ac");
}

TEST_CASE("GridLAB-D external diagnostic: large PV-heavy transmission case runs",
          "[gridlabd][compare][external][diagnostic][large]") {
  if (std::getenv("HACDCPF_GRIDLABD_RUN_LARGE") == nullptr) {
    SKIP("Set HACDCPF_GRIDLABD_RUN_LARGE=1 to run the case300 GridLAB-D diagnostic");
  }
  const auto gridlabd = hacdcpf::io::discover_gridlabd();
  if (!gridlabd.available) {
    SKIP("GridLAB-D executable not available for case300 diagnostic");
  }

  hacdcpf::io::GridLABDComparisonOptions options;
  options.export_options.model_name = "diagnostic_case300_ac";
  options.compare_bus_voltages = false;
  options.compare_branch_flows = false;
  options.require_gridlabd = true;

  const auto report = hacdcpf::io::compare_gridlabd_snapshot(
      hacdcpf::io::build_ac_only_version(hacdcpf::io::build_case300_acdc()),
      options);
  INFO("GridLAB-D command: " << report.gridlabd_result.command);
  print_report_diagnostics(report, 4);
  CHECK(report.hacdcpf_power_flow_converged);
  CHECK(report.gridlabd_run_attempted);
  CHECK(report.gridlabd_run_success);
  CHECK(report.gridlabd_result.bus_voltages.size() >= 250);
  CHECK(std::any_of(report.warnings.begin(), report.warnings.end(),
                    [](const std::string& warning) {
                      return warning.find("PV voltage regulation") !=
                             std::string::npos;
                    }));
}
