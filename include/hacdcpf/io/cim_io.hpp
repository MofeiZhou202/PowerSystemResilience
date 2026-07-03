#pragma once

#include <filesystem>
#include <string>

#include "hacdcpf/io/import_report.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

/// @file cim_io.hpp
/// Bounded IEC 61970 CIM / CGMES 3.0 (EQ + SSH) import/export
/// (docs/digital_twin_data_io_architecture.md §5, docs/cim_cgmes3_crosswalk.md).
///
/// Scope (honest ceiling): topology (TopologicalNode + BaseVoltage), lines
/// (ACLineSegment), synchronous machines (SynchronousMachine), and loads
/// (EnergyConsumer), wired by Terminals.  Transformers, DC, converters, TP/SV
/// profiles, and dynamics are OUT of scope and reported as structural loss.
///
/// Binding level is Rich (§3.2).  The RDF/XML reader is self-contained and
/// XXE-hardened: DOCTYPE/ENTITY declarations are rejected, not expanded.

namespace hacdcpf::io {

/// Options for CGMES export.
struct CimExportOptions {
  bool include_ssh{true};          ///< Emit the SSH operating point / setpoints.
  std::string model_name{"hacdcpf"};
};

/// Result of a CGMES import: the reconstructed system plus a uniform report.
struct CimImportResult {
  HybridPowerSystem system;
  ImportReport report;
};

/// Export the bounded rich model to CGMES 3.0 EQ(+SSH) RDF/XML.
std::string to_cim(const HybridPowerSystem& sys,
                   const CimExportOptions& opts = {});

/// Export to a CGMES RDF/XML file.
void save_cim(const HybridPowerSystem& sys, const std::filesystem::path& path,
              const CimExportOptions& opts = {});

/// Import a bounded CGMES 3.0 EQ(+SSH) RDF/XML document.
CimImportResult from_cim(const std::string& xml,
                         ImportMode mode = ImportMode::Permissive);

/// Import a bounded CGMES RDF/XML file.
CimImportResult load_cim(const std::filesystem::path& path,
                         ImportMode mode = ImportMode::Permissive);

}  // namespace hacdcpf::io
