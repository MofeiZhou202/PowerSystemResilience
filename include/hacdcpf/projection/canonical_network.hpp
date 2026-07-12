#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hacdcpf {

/// Projection policy used when contracting switch/circuit-breaker terminals.
/// ExactIdeal contracts only devices whose authored impedance is exactly zero.
/// ThresholdApproximate also contracts non-zero devices below the configured
/// numerical threshold and records them as approximate merge edges.
enum class ProjectionMode {
  ExactIdeal,
  ThresholdApproximate,
};

enum class MergeSemantics {
  ExactIdeal,
  ThresholdApproximate,
};

struct MergeRecord {
  int branch_index{0};
  int bus_from{0};
  int bus_to{0};
  double r_pu{0.0};
  double x_pu{0.0};
  MergeSemantics semantics{MergeSemantics::ExactIdeal};
};

/// Machine-readable evidence emitted by one projection pass.
struct ProjectionCertificate {
  ProjectionMode mode{ProjectionMode::ThresholdApproximate};
  double impedance_threshold{1e-4};
  int n_authored_dc_buses{0};
  std::vector<MergeRecord> merge_records;
  std::vector<std::string> diagnostics;

  [[nodiscard]] int exact_merge_count() const noexcept {
    int n = 0;
    for (const auto& record : merge_records)
      if (record.semantics == MergeSemantics::ExactIdeal) ++n;
    return n;
  }

  [[nodiscard]] int approximate_merge_count() const noexcept {
    int n = 0;
    for (const auto& record : merge_records)
      if (record.semantics == MergeSemantics::ThresholdApproximate) ++n;
    return n;
  }

  [[nodiscard]] bool exact() const noexcept {
    return approximate_merge_count() == 0;
  }
};

// ─────────────────────────────────────────────────────────────────────
// Branch expansion provenance map
//
// Populated by project_to_canonical_models() to record which ACBranch
// entries were synthesised from a rich element (Transformer2W/3W or
// Switch).  Callers can use this to map solver branch-flow results back
// to their original component type and index.
// ─────────────────────────────────────────────────────────────────────
enum class BranchOriginType { Transformer2W, Transformer3W, Switch, CircuitBreaker };

struct BranchExpandEntry {
  int    branch_index{0};     ///< ACBranch::index of the equivalent branch
  BranchOriginType origin_type{BranchOriginType::Transformer2W};
  int    origin_index{0};     ///< index of the original element (tr.index / sw.index)
  int    pair_number{0};      ///< Transformer3W only: 0=HV-MV, 1=HV-LV, 2=MV-LV
  // Original (pre-merge) endpoint bus indices of the device that produced this
  // branch.  Populated for Switch/CircuitBreaker origins so callers can map a
  // collapsed zero-impedance device back to a two-terminal cut and compute its
  // flow as the net injection across that cut.  -1 when not applicable.
  int    bus_from{-1};
  int    bus_to{-1};
  bool   closed{true};        ///< device closed state at projection time
};

struct BranchExpandMap {
  std::vector<BranchExpandEntry> entries;
  bool empty() const { return entries.empty(); }
};

// Describes how zero-impedance bus groups were merged during
// project_to_canonical_models().  Each equivalence class of buses
// connected by zero-impedance branches collapses to a single
// representative bus in the projected (internal) system.
//
// Terminology:
//   external  – original bus index (1-based ACBus::index)
//   internal  – bus position in the merged/projected buses vector (0-based)
struct BusMergeMap {
  // external bus index → internal position (0-based)
  std::unordered_map<int, int> ext_to_int;

  // external bus index → original bus position before merging (0-based).
  // Needed to unproject vectors when external indices are not contiguous.
  std::unordered_map<int, int> ext_to_orig_pos;

  // internal position (0-based) → external bus index (representative)
  std::vector<int> int_to_ext;

  // For each representative external bus, the list of all original bus
  // indices that were merged into it (including itself).
  std::vector<std::vector<int>> groups;

  // Original number of AC buses (before merging).
  int n_original{0};

  // Number of buses after merging.
  int n_merged{0};

  // External bus indices of dead-island buses stripped during projection.
  // These buses have no generation path and were removed from the
  // projected system.  unproject_bus_vector returns 0.0 for them.
  std::unordered_set<int> dead_bus_indices;

  // Original branch count before projection — used to unproject branch
  // flow results back to original branch indexing.
  int n_original_branches{0};

  // Maps original branch position (0-based) → projected branch position
  // (0-based).  Dead-island branches get -1.
  std::unordered_map<int, int> branch_orig_to_proj;

  // Original-bus participation used to recover extensive quantities. Values
  // sum to one within each surviving merge group.
  std::unordered_map<int, double> extensive_participation;

  /// Projection policy and the physical edges that induced each contraction.
  ProjectionMode projection_mode{ProjectionMode::ThresholdApproximate};
  double impedance_threshold{1e-4};
  std::vector<MergeRecord> merge_records;

  bool has_merges() const { return n_merged > 0 && n_merged < n_original; }

  // Returns true if any dead-island buses were stripped.
  bool has_dead_buses() const { return !dead_bus_indices.empty(); }

  // Check if a bus (by external 1-based index) is a dead island bus.
  bool is_dead_bus(int ext_bus) const { return dead_bus_indices.count(ext_bus) > 0; }
};

// ───────────────────────────────────────────────────────────────────
// Projection provenance mapping
//
// Records which canonical component was synthesised from which rich
// source component during project_to_canonical_models().  Used for
// result attribution, carbon tracing, and user-facing diagnostics.
// ───────────────────────────────────────────────────────────────────
struct ComponentMapping {
  std::string source_type;          ///< e.g. "PVSystem", "Motor", "Transformer2W"
  std::string source_id;            ///< original component index or name
  std::string canonical_type;       ///< e.g. "StaticGenerator", "ACBranch", "BusMerge"
  std::string canonical_id;         ///< canonical component index or representative bus
  double participation_factor{1.0}; ///< fraction of source output attributed here
};

/// Optional report returned alongside a projected system.
/// Populated when the caller requests diagnostic detail.
struct ProjectionReport {
  std::vector<ComponentMapping> mappings;
  std::vector<std::string>      diagnostics;  ///< human-readable notes

  bool empty() const noexcept {
    return mappings.empty() && diagnostics.empty();
  }

  /// Number of mappings for a given source type.
  int count_source(const std::string& type) const noexcept {
    int n = 0;
    for (const auto& m : mappings)
      if (m.source_type == type) ++n;
    return n;
  }
};

enum class ObservableKind {
  ACBusVoltage,
  DCBusVoltage,
  ACBranchTerminalFlow,
  SwitchTerminalFlow,
  ConverterTransfer,
  AggregatedDeviceInjection,
  NodalDual,
  ServiceLoss,
};

enum class RecoveryClass {
  Strong,
  Approximate,
  AuditOnly,
  Unsupported,
};

struct ObservableAttribution {
  ObservableKind observable{ObservableKind::ACBusVoltage};
  RecoveryClass recovery{RecoveryClass::Unsupported};
  int canonical_entities{0};
  int attributed_entities{0};
  std::string reason;

  [[nodiscard]] bool total() const noexcept {
    return recovery != RecoveryClass::Unsupported &&
           attributed_entities == canonical_entities;
  }
};

}  // namespace hacdcpf
