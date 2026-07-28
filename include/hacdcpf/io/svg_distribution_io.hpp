#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "hacdcpf/io/import_report.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

/// @file svg_distribution_io.hpp
/// Import the IEC-CGE annotated distribution single-line SVG dialect emitted by
/// Chinese utility drawing systems. This is XML syntax, but it is not CIM/RDF:
/// equipment identity is carried by cge:psr_ref and connectivity is recovered
/// from SVG geometry.

namespace hacdcpf::io {

struct SvgDistributionImportOptions {
  std::string model_name{"SVG distribution feeder"};
  double base_mva{10.0};
  double nominal_mv_kv{10.0};
  double nominal_lv_kv{0.4};

  /// Coordinate tolerance used when joining line vertices and device terminals.
  double connection_tolerance{1.0};
  /// Diagram distance to physical length conversion used when no asset length
  /// is present. The supplied examples use roughly 1 SVG unit per metre.
  double km_per_svg_unit{0.001};

  double overhead_r_ohm_per_km{0.641};
  double overhead_x_ohm_per_km{0.35};
  double cable_r_ohm_per_km{0.387};
  double cable_x_ohm_per_km{0.08};
  double default_line_rate_mva{6.0};

  double default_transformer_sn_mva{0.2};
  double default_transformer_vk_percent{4.0};
  double default_transformer_vkr_percent{1.0};
  double transformer_load_factor{0.4};
  double power_factor{0.9};

  /// Add a synthetic source at every substation symbol. If the drawing has no
  /// substation symbol, add one source to its largest connected component.
  /// Other disconnected components remain explicitly isolated.
  bool auto_add_external_grids{true};
};

struct SvgDistributionImportResult {
  HybridPowerSystem system;
  ImportReport report;
  std::vector<std::string> warnings;
  std::size_t source_line_objects{0};
  std::size_t source_switch_objects{0};
  std::size_t source_transformer_objects{0};
  std::size_t recovered_nodes{0};
  std::size_t unresolved_objects{0};
  std::size_t synthetic_external_grids{0};
  std::size_t isolated_components{0};
  std::size_t isolated_buses{0};
};

SvgDistributionImportResult from_svg_distribution(
    const std::string& svg,
    ImportMode mode = ImportMode::Permissive,
    const SvgDistributionImportOptions& opts = {});

SvgDistributionImportResult load_svg_distribution(
    const std::filesystem::path& path,
    ImportMode mode = ImportMode::Permissive,
    const SvgDistributionImportOptions& opts = {});

}  // namespace hacdcpf::io
