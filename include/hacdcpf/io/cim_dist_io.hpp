#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "hacdcpf/io/import_report.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

/// @file cim_dist_io.hpp
/// Import / export for the Chinese distribution-grid CIM/RDF dialect used by
/// State Grid / CSG low-voltage 配变台区 (transformer-district) exports.
///
/// This is a DIFFERENT schema from the bounded CGMES 3.0 profile handled by
/// cim_io.hpp: the namespace is `http://iec.ch/TC57/2003/CIM-schema-cim10#`
/// (+ `http://www.chinapower.cn/Rfs/2006/Rdf-Cim#`), topology is expressed with
/// `ConnectivityNode` + `Terminal`, and the core classes are `PowerTransformer`
/// / `TransformerWinding` / `ACLineSegment` / `Breaker` / `Disconnector` /
/// `BusbarSection` / `LVBuilding` / `Meter` — all of which cim_io explicitly
/// puts out of scope.
///
/// Mapping (honest ceiling):
///   - ConnectivityNode (referenced by ≥1 Terminal)  → ACBus
///   - PowerTransformer + primary/secondary Winding   → Transformer2W
///   - Missing source model                           → one synthetic
///     ExternalGrid per physical feeder component
///   - ACLineSegment                                  → ACBranch (impedance
///     estimated from Conductor.length + crossSectionArea + a cable table)
///   - Breaker / Disconnector                         → Switch (closed =
///     !normalOpen; collapsed later by switch_contraction)
///   - LVBuilding (+ aggregated Meters)               → Load (per-building
///     demand estimated by splitting the transformer capacity across buildings
///     by meter count; Meter count → n_customers)
///   - Transformer with no LVBuilding                  → one estimated Load at
///     the low-voltage bus
///   - EnergyConsumer / SynchronousMachine and common subclasses → explicit
///     Load / Generator when a connected Terminal is present
///   - ConductingEquipment.phases                      → executable
///     ThreePhaseACSystem phase masks and per-phase load/generation values
///
/// Loads carry no nameplate kW in the source, so demand is ESTIMATED (recorded
/// as Coerced/BestEffort in the ImportReport). The RDF/XML reader is the same
/// self-contained, XXE-hardened parser used by cim_io.

namespace hacdcpf::io {

/// Tunables for the estimated-demand and estimated-impedance heuristics.
struct CimDistImportOptions {
  /// Fraction of the transformer's rated capacity assumed as coincident peak
  /// apparent load, split across LVBuildings by meter count.
  double load_factor{0.4};
  /// Power factor used to split the estimated apparent load into P and Q.
  double power_factor{0.9};
  /// Short-circuit level (MVA) of the synthetic external grid at the HV bus.
  double grid_s_sc_mva{100.0};
  /// ACLineSegments shorter than this (metres) are cable heads / joints with
  /// near-zero impedance; they are modelled as closed switches (merged by
  /// switch contraction) instead of branches, so the Y-bus stays non-singular.
  double min_segment_length_m{1.0};
  /// Default two-winding transformer impedance when the source omits it.
  double default_vk_percent{4.0};
  double default_vkr_percent{1.0};
  double default_i0_percent{0.5};
  /// Type-aware conductor fallbacks used only when both Equipment.model and
  /// Conductor.crossSectionArea are absent. These represent feeder conductors,
  /// not customer service drops.
  double default_lv_overhead_cross_section_mm2{35.0};
  double default_lv_cable_cross_section_mm2{50.0};
  double default_mv_cross_section_mm2{300.0};
  /// System power base (MVA). Kept near the transformer-district scale (≈1 MVA)
  /// so 0.4 kV feeder per-unit impedances stay O(0.01–1) and Newton power flow
  /// stays well conditioned (a 100 MVA base pushes them to 5–50 pu).
  double base_mva{1.0};
  /// Add a synthetic ExternalGrid when the imported CIM has no explicit
  /// source model. A Circuit.SourceBreaker is preferred. Transformer-area
  /// exports with isolated HV buses get one source per transformer; a remaining
  /// source-less MV network gets one source per connected component.
  bool auto_add_external_grid{true};
};

/// Options for distribution-CIM export.
struct CimDistExportOptions {
  std::string model_name{"hacdcpf"};
  /// Emit one generic <cim:Meter> per customer under each LVBuilding.
  bool emit_meters{true};
};

/// Result of a distribution-CIM import: system + uniform report + a flat list of
/// human-readable warning strings (mirrors the ETAP importer's rep.warnings, so
/// the GUI can surface them in the import dialog).
struct CimDistImportResult {
  HybridPowerSystem system;
  ImportReport report;
  std::vector<std::string> warnings;
  /// Source-document facts used by the GUI to distinguish absent CIM data
  /// from an importer failure or a synthesized equivalent.
  std::size_t source_load_objects{0};
  std::size_t source_generator_objects{0};
  std::size_t inferred_load_objects{0};
  bool has_explicit_phase_data{false};
  bool is_unbalanced{false};
};

/// Import a distribution-CIM RDF/XML document.
CimDistImportResult from_cim_dist(const std::string& xml,
                                  ImportMode mode = ImportMode::Permissive,
                                  const CimDistImportOptions& opts = {});

/// Import and stitch multiple distribution-CIM RDF/XML documents. Objects are
/// joined by rdf:ID/rdf:about, so references may cross file boundaries; repeated
/// declarations are de-duplicated and complementary properties are merged.
CimDistImportResult from_cim_dist(
    const std::vector<std::string>& xml_documents,
    ImportMode mode = ImportMode::Permissive,
    const CimDistImportOptions& opts = {});

/// Import a distribution-CIM RDF/XML file.
CimDistImportResult load_cim_dist(const std::filesystem::path& path,
                                  ImportMode mode = ImportMode::Permissive,
                                  const CimDistImportOptions& opts = {});

/// Load and stitch multiple distribution-CIM RDF/XML files.
CimDistImportResult load_cim_dist(
    const std::vector<std::filesystem::path>& paths,
    ImportMode mode = ImportMode::Permissive,
    const CimDistImportOptions& opts = {});

/// Export the current system to the distribution-CIM RDF/XML dialect.
std::string to_cim_dist(const HybridPowerSystem& sys,
                        const CimDistExportOptions& opts = {});

/// Export to a distribution-CIM RDF/XML file.
void save_cim_dist(const HybridPowerSystem& sys,
                   const std::filesystem::path& path,
                   const CimDistExportOptions& opts = {});

}  // namespace hacdcpf::io
