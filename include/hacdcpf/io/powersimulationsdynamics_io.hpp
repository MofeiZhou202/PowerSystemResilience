#pragma once

#include <filesystem>
#include <string>

#include "hacdcpf/model/hybrid_power_system.hpp"

/// @file powersimulationsdynamics_io.hpp
/// Lightweight PowerSimulationsDynamics.jl / PowerSystems.jl interchange.
///
/// The exported JSON uses the same neutral `hacdcpf_psd_snapshot.v1` envelope as
/// `tools/psd_validation/export_psd_snapshot.jl`.  It is intentionally a
/// dynamic-profile and benchmark-manifest IO surface, not a Julia runtime
/// dependency or a claim that PSD can represent every hybrid AC/DC rich object.

namespace hacdcpf::io {

struct PowerSimulationsDynamicsExportOptions {
  std::string model_name{"hacdcpf_psd_snapshot"};
  bool include_static_components{true};
  bool include_dynamic_profiles{true};
  bool include_conversion_notes{true};
};

/// Export a HACDCPF system to the neutral PSD/PowerSystems snapshot JSON shape.
std::string to_powersimulationsdynamics_json(
    const HybridPowerSystem& sys,
    const PowerSimulationsDynamicsExportOptions& opts = {},
    int indent = 2);

/// Save a HACDCPF → PSD/PowerSystems snapshot JSON file.
void save_powersimulationsdynamics_json(
    const HybridPowerSystem& sys,
    const std::filesystem::path& path,
    const PowerSimulationsDynamicsExportOptions& opts = {},
    int indent = 2);

}  // namespace hacdcpf::io
