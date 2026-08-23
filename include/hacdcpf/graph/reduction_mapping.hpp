#pragma once

/// graph/reduction_mapping.hpp
/// ============================
/// Mapping structures that record how original nodes/branches are
/// transformed during every reduction step, enabling full result recovery.

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/graph/power_system_graph.hpp"

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// Per-reduction-type records
// ═══════════════════════════════════════════════════════════════════════

struct SwitchContractionRecord {
  NodeDomain       domain{NodeDomain::AC};
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
  NodeDomain domain{NodeDomain::AC}; ///< Domain of the eliminated bus (AC or DC)
};

struct PendantReductionRecord {
  int    eliminated_bus_id{0};
  int    parent_bus_id{0};
  int    source_branch_index{-1}; ///< Stable ACBranch/DCBranch .index
  double p_load_absorbed{0.0}; ///< P transferred to parent [MW]
  double q_load_absorbed{0.0}; ///< Q transferred to parent [MVAr]
  NodeDomain domain{NodeDomain::AC}; ///< Domain of the eliminated bus (AC or DC)
};

struct KronReductionRecord {
  std::vector<BusRef> eliminated_buses;
  std::vector<BusRef> retained_buses;
};

// ═══════════════════════════════════════════════════════════════════════
// Aggregated mapping
// ═══════════════════════════════════════════════════════════════════════

struct ReductionMapping {
  /// Authoritative domain-qualified certificate. Every original identity has
  /// exactly one current representative; reverse maps are derived from it.
  std::unordered_map<BusRef, BusRef, BusRefHash> original_to_reduced_buses;
  std::unordered_map<BusRef, std::vector<BusRef>, BusRefHash>
      reduced_to_original_bus_refs;

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

  /// Authoritative stable component mapping for ordinary AC/DC branches.
  /// An invalid target means the source branch was eliminated without a
  /// replacement, as in pendant folding.
  std::unordered_map<BranchRef, BranchRef, BranchRefHash>
      original_to_reduced_branches;
  std::unordered_map<BranchRef, std::vector<BranchRef>, BranchRefHash>
      reduced_to_original_branch_refs;

  /// original branch_id → reduced branch_id (-1 if eliminated)
  std::unordered_map<int, int> original_to_reduced_branch;
  /// reduced branch_id → list of original branch_ids it represents
  std::unordered_map<int, std::vector<int>> reduced_to_original_branches;

  std::vector<SwitchContractionRecord> switch_records;
  std::vector<SeriesReductionRecord>   series_records;
  std::vector<PendantReductionRecord>  pendant_records;
  std::vector<KronReductionRecord>     kron_records;
};

ReductionMapping make_identity_reduction_mapping(
    const PowerSystemGraph& graph);

void set_bus_reduction(ReductionMapping& mapping,
                       BusRef original, BusRef reduced);
void set_branch_reduction(ReductionMapping& mapping,
                          BranchRef original, BranchRef reduced);
void rebuild_reduction_reverse_maps(ReductionMapping& mapping);

/// Compose original->intermediate and intermediate->final certificates.
/// Operation records are concatenated in forward execution order.
ReductionMapping compose_reduction_mappings(
    const ReductionMapping& first,
    const ReductionMapping& second);

}  // namespace hacdcpf::graph
