#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <string>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/svg_distribution_io.hpp"

namespace {

std::filesystem::path data_file(const std::string& name) {
  return std::filesystem::path(__FILE__).parent_path().parent_path() / "data" /
         name;
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

TEST_CASE("IEC-CGE overhead feeder SVG imports and solves",
          "[io][svg][distribution][power-flow]") {
  const auto path = data_file("张庄C503线-架空-丽水市.svg");
  REQUIRE(std::filesystem::is_regular_file(path));
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
  REQUIRE(std::filesystem::is_regular_file(path));
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
