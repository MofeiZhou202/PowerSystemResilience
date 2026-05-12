#pragma once

#include <string>
#include <unordered_map>
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

  bool has_merges() const { return n_merged > 0 && n_merged < n_original; }

  // Build identity (no-op) mapping for n buses with external indices
  // taken from a bus vector.  Used when no merging occurs.
  static BusMergeMap identity(int n) {
    BusMergeMap m;
    m.n_original = n;
    m.n_merged = n;
    const int count = n > 0 ? n : 0;
    m.int_to_ext.reserve(static_cast<size_t>(count));
    m.groups.reserve(static_cast<size_t>(count));
    for (int i = 0; i < n; ++i) {
      const int ext = i + 1;
      m.ext_to_int[ext] = i;
      m.ext_to_orig_pos[ext] = i;
      m.int_to_ext.push_back(ext);
      m.groups.push_back({ext});
    }
    return m;
  }
};

}  // namespace hacdcpf
