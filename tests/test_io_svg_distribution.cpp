#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <string>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/svg_distribution_io.hpp"

namespace {

std::filesystem::path data_file(const std::string& name) {
  const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "data" /
                    std::filesystem::path(
                        std::u8string(name.begin(), name.end()));
  if (!std::filesystem::is_regular_file(path)) {
    SKIP("optional distribution SVG fixture is unavailable: " + name);
  }
  return path;
}

void check_calculable(const hacdcpf::io::SvgDistributionImportResult& imported,
                      std::size_t expected_lines,
                      std::size_t expected_transformers) {
  for (const auto& warning : imported.warnings) INFO(warning);
  CHECK_FALSE(imported.report.has_errors());
  CHECK(imported.report.unit_assertion == hacdcpf::io::UnitAssertion::BestEffort);
  CHECK(imported.source_line_objects == expected_lines);
  CHECK(imported.source_transformer_objects == expected_transformers);
  CHECK(imported.system.ac.transformers_2w.size() == expected_transformers);
  CHECK(imported.system.ac.loads.size() == expected_transformers);
  CHECK(imported.system.ac.branches.size() >= expected_lines);
  CHECK_FALSE(imported.system.ac.switches.empty());
  CHECK_FALSE(imported.system.ac.external_grids.empty());
  CHECK(imported.synthetic_external_grids ==
        imported.system.ac.external_grids.size());
  CHECK(imported.synthetic_external_grids == 1);
  CHECK(imported.isolated_components > 0);
  CHECK(imported.isolated_buses > 0);
  CHECK(imported.recovered_nodes == imported.system.ac.buses.size());
  CHECK(imported.unresolved_objects == 0);
  CHECK(imported.parameter_completion_applied);
  CHECK(imported.parameter_completion.candidates ==
        static_cast<int>(imported.system.ac.branches.size()));
  CHECK(imported.parameter_completion.fields_changed > 0);
  CHECK_FALSE(imported.parameter_completion.references.empty());
  CHECK(std::any_of(imported.parameter_completion.references.begin(),
                    imported.parameter_completion.references.end(),
                    [](const auto& reference) {
                      return reference.standard.find("GB/T") !=
                             std::string::npos;
                    }));
  CHECK(std::all_of(imported.system.ac.branches.begin(),
                    imported.system.ac.branches.end(), [](const auto& branch) {
                      return branch.parameter_source ==
                                 "design_handbook_gbt3956_distribution_standards" &&
                             branch.cross_section_inferred &&
                             branch.cross_section_mm2 > 0.0 &&
                             branch.rate_a_mva == branch.rate_b_mva &&
                             branch.rate_a_mva == branch.rate_c_mva;
                    }));
  CHECK(std::all_of(imported.system.ac.external_grids.begin(),
                    imported.system.ac.external_grids.end(),
                    [](const auto& grid) {
                      return grid.s_sc_max_mva > grid.s_sc_min_mva &&
                             grid.r_pu > 0.0 && grid.x_pu > 0.0 &&
                             grid.r0_pu > grid.r_pu &&
                             grid.x0_pu > grid.x_pu;
                    }));
  CHECK(std::all_of(imported.system.ac.transformers_2w.begin(),
                    imported.system.ac.transformers_2w.end(),
                    [](const auto& transformer) {
                      return transformer.pk_kw > 0.0 &&
                             transformer.vk_percent >=
                                 transformer.vkr_percent;
                    }));

  hacdcpf::PowerFlowOptions options;
  options.max_iter = 200;
  options.tol = 1e-8;
  options.enable_converter_coordination_check = false;
  const auto power_flow = hacdcpf::solve_power_flow(imported.system, options);
  INFO(power_flow.residual);
  INFO(power_flow.iterations);
  REQUIRE(power_flow.converged);
  REQUIRE(power_flow.vm.size() == imported.system.ac.buses.size());
  std::size_t energized = 0;
  for (std::size_t i = 0; i < imported.system.ac.buses.size(); ++i) {
    if (imported.system.ac.buses[i].bus_type == hacdcpf::BusType::ISOLATED) {
      CHECK(power_flow.vm[i] == 0.0);
    } else {
      CHECK(power_flow.vm[i] > 0.85);
      ++energized;
    }
  }
  CHECK(energized > 1);
  CHECK(std::any_of(imported.system.ac.loads.begin(),
                    imported.system.ac.loads.end(), [&](const auto& load) {
                      const auto bus = std::find_if(
                          imported.system.ac.buses.begin(),
                          imported.system.ac.buses.end(), [&](const auto& item) {
                            return item.index == load.bus;
                          });
                      return bus != imported.system.ac.buses.end() &&
                             bus->bus_type != hacdcpf::BusType::ISOLATED;
                    }));
}

}  // namespace

TEST_CASE("SVG file API preserves Unicode paths without optional feeder fixtures",
          "[io][svg][distribution][unicode]") {
  struct TemporaryDirectory {
    std::filesystem::path path;
    ~TemporaryDirectory() {
      std::error_code ignored;
      std::filesystem::remove_all(path, ignored);
    }
  } directory{std::filesystem::temp_directory_path() /
      ("hysim-svg-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()))};
  REQUIRE(std::filesystem::create_directory(directory.path));
  const auto file = directory.path / std::filesystem::path(u8"研究馈线-Δ.svg");
  hacdcpf::HybridPowerSystem system;
  hacdcpf::ACBus bus;
  bus.index = 1;
  bus.base_kv = 10.0;
  system.ac.buses.push_back(bus);
  const auto exported = hacdcpf::io::save_svg_distribution(system, file);
  CHECK(exported.exported_buses == 1);
  const auto loaded = hacdcpf::io::load_svg_distribution(file);
  CHECK(loaded.system.name == "研究馈线-Δ");
  CHECK_FALSE(loaded.report.has_errors());
  CHECK(loaded.system.ac.buses.size() == 1);
  const auto missing = directory.path / std::filesystem::path(u8"不存在-Δ.svg");
  const auto missing_result = hacdcpf::io::load_svg_distribution(missing);
  REQUIRE(missing_result.report.has_errors());
  REQUIRE_FALSE(missing_result.report.records.empty());
  const auto missing_utf8 = missing.u8string();
  CHECK(missing_result.report.records.front().source_locator ==
        std::string(missing_utf8.begin(), missing_utf8.end()));
}

TEST_CASE("IEC-CGE overhead feeder SVG imports and solves",
          "[io][svg][distribution][power-flow]") {
  const auto path = data_file("张庄C503线-架空-丽水市.svg");
  const auto imported = hacdcpf::io::load_svg_distribution(path);
  CHECK(imported.system.name == "张庄C503线-架空-丽水市");
  check_calculable(imported, 30, 11);
  CHECK(std::all_of(imported.system.ac.branches.begin(),
                    imported.system.ac.branches.end(), [](const auto& branch) {
                      return branch.line_type == "overhead" &&
                             branch.parameters_inferred;
                    }));
}

TEST_CASE("IEC-CGE mixed cable and overhead feeder SVG imports and solves",
          "[io][svg][distribution][power-flow]") {
  const auto path = data_file("新区B259线-混合.svg");
  const auto imported = hacdcpf::io::load_svg_distribution(path);
  CHECK(imported.system.name == "新区B259线-混合");
  check_calculable(imported, 69, 14);
  CHECK(std::any_of(imported.system.ac.branches.begin(),
                    imported.system.ac.branches.end(), [](const auto& branch) {
                      return branch.line_type == "cable";
                    }));
  CHECK(std::any_of(imported.system.ac.branches.begin(),
                    imported.system.ac.branches.end(), [](const auto& branch) {
                      return branch.line_type == "overhead";
                    }));
  CHECK(std::any_of(imported.system.ac.transformers_2w.begin(),
                    imported.system.ac.transformers_2w.end(), [](const auto& t) {
                      return t.sn_mva > 0.2;
                    }));
}

TEST_CASE("Plain decorative SVG is rejected as a calculable feeder",
          "[io][svg][distribution][negative]") {
  const auto imported = hacdcpf::io::from_svg_distribution(
      R"svg(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0 0L1 1"/></svg>)svg");
  CHECK(imported.report.has_errors());
  CHECK(imported.system.ac.buses.empty());
}

TEST_CASE("SVG source synthesis can be disabled without implying supply",
          "[io][svg][distribution][islands]") {
  hacdcpf::io::SvgDistributionImportOptions options;
  options.auto_add_external_grids = false;
  const auto imported = hacdcpf::io::load_svg_distribution(
      data_file("张庄C503线-架空-丽水市.svg"),
      hacdcpf::io::ImportMode::Permissive, options);
  REQUIRE_FALSE(imported.report.has_errors());
  CHECK(imported.system.ac.external_grids.empty());
  CHECK(imported.synthetic_external_grids == 0);
  CHECK(imported.isolated_buses == imported.system.ac.buses.size());
  CHECK(std::all_of(imported.system.ac.buses.begin(),
                    imported.system.ac.buses.end(), [](const auto& bus) {
                      return bus.bus_type == hacdcpf::BusType::ISOLATED;
                    }));
}

TEST_CASE("SVG standards-aware completion remains explicitly optional",
          "[io][svg][distribution][parameter-completion]") {
  hacdcpf::io::SvgDistributionImportOptions options;
  options.auto_complete_parameters = false;
  options.overhead_r_ohm_per_km = 0.777;
  const auto imported = hacdcpf::io::load_svg_distribution(
      data_file("张庄C503线-架空-丽水市.svg"),
      hacdcpf::io::ImportMode::Permissive, options);
  REQUIRE_FALSE(imported.report.has_errors());
  CHECK_FALSE(imported.parameter_completion_applied);
  CHECK(imported.parameter_completion.suggestions.empty());
  REQUIRE_FALSE(imported.system.ac.branches.empty());
  CHECK(std::all_of(imported.system.ac.branches.begin(),
                    imported.system.ac.branches.end(), [&](const auto& branch) {
                      return branch.parameter_source == "svg_geometry_estimate" &&
                             branch.parameters_inferred &&
                             branch.r_ohm_per_km ==
                                 options.overhead_r_ohm_per_km;
                    }));
}

TEST_CASE("HACDCPF distribution SVG export is structurally re-importable",
          "[io][svg][distribution][roundtrip]") {
  const auto source = hacdcpf::io::load_svg_distribution(
      data_file("张庄C503线-架空-丽水市.svg"));
  REQUIRE_FALSE(source.report.has_errors());

  const auto exported = hacdcpf::io::to_svg_distribution(source.system);
  CHECK(exported.svg.find("xmlns:cge=") != std::string::npos);
  CHECK(exported.svg.find("xmlns:hacdcpf=") != std::string::npos);
  CHECK(exported.svg.find("<hacdcpf:model") != std::string::npos);
  CHECK(exported.exported_buses == source.system.ac.buses.size());
  CHECK(exported.exported_branches == source.system.ac.branches.size());
  CHECK(exported.exported_switches == source.system.ac.switches.size());
  CHECK(exported.exported_transformers ==
        source.system.ac.transformers_2w.size());
  CHECK(exported.exported_external_grids ==
        source.system.ac.external_grids.size());
  CHECK(exported.embedded_loads == source.system.ac.loads.size());
  CHECK(exported.omitted_loads == 0);

  const auto restored =
      hacdcpf::io::from_svg_distribution(exported.svg);
  for (const auto& warning : restored.warnings) INFO(warning);
  REQUIRE_FALSE(restored.report.has_errors());
  CHECK(restored.embedded_parameter_objects > 0);
  CHECK(restored.synthetic_external_grids == 0);
  CHECK(restored.restored_external_grids == 1);
  CHECK(restored.system.ac.buses.size() == source.system.ac.buses.size());
  CHECK(restored.system.ac.branches.size() == source.system.ac.branches.size());
  CHECK(restored.system.ac.switches.size() == source.system.ac.switches.size());
  CHECK(restored.system.ac.transformers_2w.size() ==
        source.system.ac.transformers_2w.size());
  CHECK(restored.system.ac.loads.size() == source.system.ac.loads.size());

  const auto sorted_indices = [](const auto& components) {
    std::vector<int> indices;
    for (const auto& component : components) indices.push_back(component.index);
    std::sort(indices.begin(), indices.end());
    return indices;
  };
  CHECK(sorted_indices(restored.system.ac.buses) ==
        sorted_indices(source.system.ac.buses));
  CHECK(sorted_indices(restored.system.ac.branches) ==
        sorted_indices(source.system.ac.branches));
  REQUIRE_FALSE(source.system.ac.branches.empty());
  REQUIRE_FALSE(restored.system.ac.branches.empty());
  CHECK(std::abs(restored.system.ac.branches.front().length_km -
                 source.system.ac.branches.front().length_km) < 1e-10);

  hacdcpf::PowerFlowOptions options;
  options.enable_converter_coordination_check = false;
  const auto power_flow = hacdcpf::solve_power_flow(restored.system, options);
  INFO(power_flow.residual);
  REQUIRE(power_flow.converged);
}
