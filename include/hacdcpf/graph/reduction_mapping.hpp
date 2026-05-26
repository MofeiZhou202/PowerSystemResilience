#pragma once

/// graph/reduction_mapping.hpp
/// ============================
/// Mapping structures that record how original nodes/branches are
/// transformed during every reduction step, enabling full result recovery.

#include <string>
#include <unordered_map>
#include <vector>

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// Per-reduction-type records
// ═══════════════════════════════════════════════════════════════════════

struct SwitchContractionRecord {
  std::vector<int> original_bus_ids;  ///< All buses merged into super-node
  int              super_bus_id{0};   ///< Canonical representative bus ID
  std::string      reason;            ///< e.g. "closed switch SW_9"
};

struct SeriesReductionRecord {
  int eliminated_bus_id{0};
  int from_bus_id{0};       ///< Bus i in i--j--k
  int to_bus_id{0};         ///< Bus k in i--j--k
  int new_branch_id{0};     ///< New merged branch ID
  std::vector<int> original_branch_ids;
  double r_eq{0.0};         ///< Equivalent resistance [pu]
  double x_eq{0.0};         ///< Equivalent reactance [pu]
  double b_eq{0.0};         ///< Equivalent susceptance [pu]
};

struct PendantReductionRecord {
  int    eliminated_bus_id{0};
  int    parent_bus_id{0};
  int    branch_id{0};
  double p_load_absorbed{0.0}; ///< P transferred to parent [pu]
  double q_load_absorbed{0.0}; ///< Q transferred to parent [pu]
};

struct KronReductionRecord {
  std::vector<int> eliminated_buses;
  std::vector<int> retained_buses;
};

// ═══════════════════════════════════════════════════════════════════════
// Aggregated mapping
// ═══════════════════════════════════════════════════════════════════════

struct ReductionMapping {
  /// original bus_id → reduced bus_id (after all reductions)
  /// Legacy: safe only when AC and DC bus IDs never overlap.
  std::unordered_map<int, int> original_to_reduced_bus;
  /// reduced bus_id → list of original bus_ids it represents (legacy)
  std::unordered_map<int, std::vector<int>> reduced_to_original_buses;

  /// Domain-qualified bus maps — always correct in hybrid systems where AC
  /// and DC buses may share the same numeric ID.
  std::unordered_map<int, int>              ac_original_to_reduced_bus;
  std::unordered_map<int, int>              dc_original_to_reduced_bus;
  std::unordered_map<int, std::vector<int>> ac_reduced_to_original_buses;
  std::unordered_map<int, std::vector<int>> dc_reduced_to_original_buses;

  /// original branch_id → reduced branch_id (-1 if eliminated)
  std::unordered_map<int, int> original_to_reduced_branch;
  /// reduced branch_id → list of original branch_ids it represents
  std::unordered_map<int, std::vector<int>> reduced_to_original_branches;

  std::vector<SwitchContractionRecord> switch_records;
  std::vector<SeriesReductionRecord>   series_records;
  std::vector<PendantReductionRecord>  pendant_records;
  std::vector<KronReductionRecord>     kron_records;
};

}  // namespace hacdcpf::graph
