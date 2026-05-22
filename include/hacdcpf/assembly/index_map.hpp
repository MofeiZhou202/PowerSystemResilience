#pragma once

/// assembly/index_map.hpp
/// =======================
/// Bus and branch index mapping types used during solver assembly.
/// Maps between external (user-facing) indices and internal (solver)
/// contiguous 0-based positions.

#include <unordered_map>
#include <vector>

#include "hacdcpf/projection/canonical_network.hpp"

namespace hacdcpf::powerflow {

/// Maps between external bus indices (1-based) and internal positions (0-based).
struct BusIndexMap {
  /// ext_to_pos[ext_bus_index] = internal 0-based position.
  std::unordered_map<int, int> ext_to_pos;

  /// pos_to_ext[internal_position] = external 1-based bus index.
  std::vector<int> pos_to_ext;

  int size() const { return static_cast<int>(pos_to_ext.size()); }

  /// Build from a contiguous range of external bus indices.
  static BusIndexMap from_external_indices(const std::vector<int>& ext_indices);
};

/// Maps between external branch indices and internal 0-based positions.
struct BranchIndexMap {
  std::unordered_map<int, int> ext_to_pos;
  std::vector<int> pos_to_ext;

  int size() const { return static_cast<int>(pos_to_ext.size()); }
};

/// Combined index maps for a projected system.
struct SystemIndexMap {
  BusIndexMap    ac_bus_map;
  BusIndexMap    dc_bus_map;
  BranchIndexMap ac_branch_map;
  BranchIndexMap dc_branch_map;

  /// If a bus merge map is present, maps merged internal position back to
  /// the canonical external index.
  const BusMergeMap* merge_map{nullptr};  ///< non-owning; may be nullptr
};

}  // namespace hacdcpf::powerflow
