#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hacdcpf {

// ─────────────────────────────────────────────────────────────────────
// Branch expansion provenance map
//
// Populated by project_to_canonical_models() to record which ACBranch
// entries were synthesised from a rich element (Transformer2W/3W or
// Switch).  Callers can use this to map solver branch-flow results back
// to their original component type and index.
// ─────────────────────────────────────────────────────────────────────
enum class BranchOriginType { Transformer2W, Transformer3W, Switch };

struct BranchExpandEntry {
  int    branch_index{0};     ///< ACBranch::index of the equivalent branch
  BranchOriginType origin_type{BranchOriginType::Transformer2W};
  int    origin_index{0};     ///< index of the original element (tr.index / sw.index)
  int    pair_number{0};      ///< Transformer3W only: 0=HV-MV, 1=HV-LV, 2=MV-LV
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

  bool has_merges() const { return n_merged > 0 && n_merged < n_original; }

  // Returns true if any dead-island buses were stripped.
  bool has_dead_buses() const { return !dead_bus_indices.empty(); }

  // Check if a bus (by external 1-based index) is a dead island bus.
  bool is_dead_bus(int ext_bus) const { return dead_bus_indices.count(ext_bus) > 0; }

  // Build identity (no-op) mapping for n buses with external indices
  // taken from a bus vector.  Used when no merging occurs.
  static BusMergeMap identity(int n) {
    BusMergeMap m;
    m.n_original = n;
    m.n_merged = n;
    return m;
  }
};

}  // namespace hacdcpf
