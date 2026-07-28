#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "hacdcpf/io/import_report.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/model/standard_parameter_library.hpp"

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
  double default_mv_cross_section_mm2{300.0};

  double default_transformer_sn_mva{0.2};
  double default_transformer_vk_percent{4.0};
  double default_transformer_vkr_percent{1.0};
  double transformer_load_factor{0.4};
  double power_factor{0.9};

  /// Apply the shared standards-aware design-handbook completion workflow to
  /// fields explicitly marked as SVG estimates. Authored/manufacturer values
  /// are outside the replaceable provenance set.
  bool auto_complete_parameters{true};

  /// Synthetic source strength remains an engineering assumption. The
  /// impedance conversion follows GB/T 15544.1 / IEC 60909.
  double synthetic_source_s_sc_max_mva{100.0};
  double synthetic_source_s_sc_min_mva{50.0};
  double synthetic_source_rx{0.1};
  double synthetic_source_zero_sequence_multiplier{3.0};

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
  std::size_t embedded_parameter_objects{0};
  std::size_t restored_external_grids{0};
  bool parameter_completion_applied{false};
  DesignHandbookCompletionReport parameter_completion;
};

struct SvgDistributionExportOptions {
  std::string title;
  double horizontal_spacing{180.0};
  double vertical_spacing{90.0};
  double margin{60.0};
  bool include_hacdcpf_parameters{true};
};

struct SvgDistributionExportResult {
  std::string svg;
  std::vector<std::string> warnings;
  std::size_t exported_buses{0};
  std::size_t exported_branches{0};
  std::size_t exported_switches{0};
  std::size_t exported_transformers{0};
  std::size_t exported_external_grids{0};
  std::size_t embedded_loads{0};
  std::size_t omitted_loads{0};
  std::size_t omitted_non_ac_components{0};
};

SvgDistributionImportResult from_svg_distribution(
    const std::string& svg,
    ImportMode mode = ImportMode::Permissive,
    const SvgDistributionImportOptions& opts = {});

SvgDistributionImportResult load_svg_distribution(
    const std::filesystem::path& path,
    ImportMode mode = ImportMode::Permissive,
    const SvgDistributionImportOptions& opts = {});

SvgDistributionExportResult to_svg_distribution(
    const HybridPowerSystem& system,
    const SvgDistributionExportOptions& opts = {});

SvgDistributionExportResult save_svg_distribution(
    const HybridPowerSystem& system,
    const std::filesystem::path& path,
    const SvgDistributionExportOptions& opts = {});

}  // namespace hacdcpf::io
