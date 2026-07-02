#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "hacdcpf/io/gridlabd_bridge.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

/// Shared import options for text-based distribution-network formats.
///
/// The converters intentionally cover the balanced AC network subset first:
/// buses/sources, series lines, two-winding transformers represented as AC
/// branches with nameplate metadata, constant-power loads, and PQ/static
/// generation. Unsupported controls and dynamic-only objects are reported in
/// the import report instead of silently changing the network.
struct ExternalGridImportOptions {
  ImportMode mode{ImportMode::Strict};
  double default_base_mva{100.0};
  double default_base_kv_ll{12.47};
  double default_frequency_hz{50.0};
  bool convert_actual_to_per_unit{true};
  bool create_slack_generator{true};
};

struct OpenDSSExportOptions {
  bool project_to_canonical{true};
  bool include_metadata_comments{true};
  bool include_solve_command{true};
  double minimum_base_kv{0.12};
  double nominal_frequency_hz{50.0};
  std::string circuit_name{"hacdcpf_export"};
};

using OpenDSSImportOptions = ExternalGridImportOptions;
using GridLABDImportOptions = ExternalGridImportOptions;

struct ExternalGridImportReport {
  HybridPowerSystem system;
  std::vector<std::string> warnings;
  std::vector<std::string> skipped;
};

// ── GridLAB-D GLM text conversion ────────────────────────────────────────────

std::string to_gridlabd(const HybridPowerSystem& sys,
                        const GridLABDExportOptions& options = {});

void save_gridlabd(const HybridPowerSystem& sys,
                   const std::filesystem::path& path,
                   const GridLABDExportOptions& options = {});

ExternalGridImportReport from_gridlabd_with_report(
    const std::string& glm_text,
    const GridLABDImportOptions& options = {});

HybridPowerSystem from_gridlabd(const std::string& glm_text,
                                const GridLABDImportOptions& options = {});

ExternalGridImportReport load_gridlabd_with_report(
    const std::filesystem::path& path,
    const GridLABDImportOptions& options = {});

HybridPowerSystem load_gridlabd(const std::filesystem::path& path,
                                const GridLABDImportOptions& options = {});

Result<HybridPowerSystem> try_from_gridlabd(
    const std::string& glm_text,
    const GridLABDImportOptions& options = {});

Result<HybridPowerSystem> try_load_gridlabd(
    const std::filesystem::path& path,
    const GridLABDImportOptions& options = {});

// ── OpenDSS DSS text conversion ──────────────────────────────────────────────

std::string to_opendss(const HybridPowerSystem& sys,
                       const OpenDSSExportOptions& options = {});

void save_opendss(const HybridPowerSystem& sys,
                  const std::filesystem::path& path,
                  const OpenDSSExportOptions& options = {});

ExternalGridImportReport from_opendss_with_report(
    const std::string& dss_text,
    const OpenDSSImportOptions& options = {});

HybridPowerSystem from_opendss(const std::string& dss_text,
                               const OpenDSSImportOptions& options = {});

ExternalGridImportReport load_opendss_with_report(
    const std::filesystem::path& path,
    const OpenDSSImportOptions& options = {});

HybridPowerSystem load_opendss(const std::filesystem::path& path,
                               const OpenDSSImportOptions& options = {});

Result<HybridPowerSystem> try_from_opendss(
    const std::string& dss_text,
    const OpenDSSImportOptions& options = {});

Result<HybridPowerSystem> try_load_opendss(
    const std::filesystem::path& path,
    const OpenDSSImportOptions& options = {});

}  // namespace hacdcpf::io
