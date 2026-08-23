/// Domain-qualified, composable certificates for graph reduction.

#include "hacdcpf/graph/reduction_mapping.hpp"

#include <algorithm>

namespace hacdcpf::graph {
namespace {

template <typename T>
void sort_unique(std::vector<T>& values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
}

void append_records(ReductionMapping& target,
                    const ReductionMapping& source) {
  target.switch_records.insert(target.switch_records.end(),
                               source.switch_records.begin(),
                               source.switch_records.end());
  target.series_records.insert(target.series_records.end(),
                               source.series_records.begin(),
                               source.series_records.end());
  target.pendant_records.insert(target.pendant_records.end(),
                                source.pendant_records.begin(),
                                source.pendant_records.end());
  target.kron_records.insert(target.kron_records.end(),
                             source.kron_records.begin(),
                             source.kron_records.end());
}

}  // namespace

ReductionMapping make_identity_reduction_mapping(
    const PowerSystemGraph& graph) {
  ReductionMapping mapping;
  for (const auto& node : graph.nodes) {
    const BusRef ref{node.domain, node.bus_id};
    mapping.original_to_reduced_buses.emplace(ref, ref);
  }
  for (const auto& edge : graph.edges) {
    NodeDomain domain;
    if (edge.category == EdgeCategory::AC_Line) {
      domain = NodeDomain::AC;
    } else if (edge.category == EdgeCategory::DC_Line) {
      domain = NodeDomain::DC;
    } else {
      continue;
    }
    const BranchRef ref{domain, edge.comp_index};
    mapping.original_to_reduced_branches.emplace(ref, ref);
  }
  rebuild_reduction_reverse_maps(mapping);
  return mapping;
}

void set_bus_reduction(ReductionMapping& mapping,
                       BusRef original, BusRef reduced) {
  mapping.original_to_reduced_buses[original] = reduced;
  rebuild_reduction_reverse_maps(mapping);
}

void set_branch_reduction(ReductionMapping& mapping,
                          BranchRef original, BranchRef reduced) {
  mapping.original_to_reduced_branches[original] = reduced;
  rebuild_reduction_reverse_maps(mapping);
}

void rebuild_reduction_reverse_maps(ReductionMapping& mapping) {
  mapping.reduced_to_original_bus_refs.clear();
  mapping.ac_original_to_reduced_bus.clear();
  mapping.dc_original_to_reduced_bus.clear();
  mapping.ac_reduced_to_original_buses.clear();
  mapping.dc_reduced_to_original_buses.clear();
  mapping.original_to_reduced_bus.clear();
  mapping.reduced_to_original_buses.clear();

  for (const auto& [original, reduced] : mapping.original_to_reduced_buses) {
    auto& forward = original.domain == NodeDomain::AC
                        ? mapping.ac_original_to_reduced_bus
                        : mapping.dc_original_to_reduced_bus;
    forward[original.bus_id] = reduced.bus_id;
    if (reduced.valid()) {
      mapping.reduced_to_original_bus_refs[reduced].push_back(original);
    }
  }
  for (auto& [reduced, originals] : mapping.reduced_to_original_bus_refs) {
    sort_unique(originals);
    auto& reverse = reduced.domain == NodeDomain::AC
                        ? mapping.ac_reduced_to_original_buses
                        : mapping.dc_reduced_to_original_buses;
    auto& ids = reverse[reduced.bus_id];
    for (const auto& original : originals) ids.push_back(original.bus_id);
    sort_unique(ids);
  }

  // Legacy compatibility is AC-preferred when numeric IDs overlap.
  for (const auto& [original, reduced] : mapping.ac_original_to_reduced_bus) {
    mapping.original_to_reduced_bus[original] = reduced;
  }
  for (const auto& [original, reduced] : mapping.dc_original_to_reduced_bus) {
    mapping.original_to_reduced_bus.try_emplace(original, reduced);
  }
  for (const auto& [reduced, originals] : mapping.ac_reduced_to_original_buses) {
    mapping.reduced_to_original_buses[reduced] = originals;
  }
  for (const auto& [reduced, originals] : mapping.dc_reduced_to_original_buses) {
    mapping.reduced_to_original_buses.try_emplace(reduced, originals);
  }

  mapping.reduced_to_original_branch_refs.clear();
  mapping.original_to_reduced_branch.clear();
  mapping.reduced_to_original_branches.clear();
  for (const auto& [original, reduced] : mapping.original_to_reduced_branches) {
    if (reduced.valid()) {
      mapping.reduced_to_original_branch_refs[reduced].push_back(original);
    }
  }
  for (auto& [reduced, originals] : mapping.reduced_to_original_branch_refs) {
    std::sort(originals.begin(), originals.end(),
              [](const BranchRef& lhs, const BranchRef& rhs) {
                if (lhs.domain != rhs.domain) return lhs.domain < rhs.domain;
                return lhs.component_index < rhs.component_index;
              });
    originals.erase(std::unique(originals.begin(), originals.end()),
                    originals.end());
  }
  auto add_legacy_branch_domain = [&](NodeDomain domain) {
    for (const auto& [original, reduced] : mapping.original_to_reduced_branches) {
      if (original.domain != domain) continue;
      if (domain == NodeDomain::DC &&
          mapping.original_to_reduced_branch.contains(original.component_index)) {
        continue;
      }
      mapping.original_to_reduced_branch[original.component_index] =
          reduced.valid() ? reduced.component_index : -1;
    }
    for (const auto& [reduced, originals] :
         mapping.reduced_to_original_branch_refs) {
      if (reduced.domain != domain) continue;
      if (domain == NodeDomain::DC &&
          mapping.reduced_to_original_branches.contains(
              reduced.component_index)) {
        continue;
      }
      auto& ids = mapping.reduced_to_original_branches[
          reduced.component_index];
      for (const auto& original : originals) {
        ids.push_back(original.component_index);
      }
      sort_unique(ids);
    }
  };
  add_legacy_branch_domain(NodeDomain::AC);
  add_legacy_branch_domain(NodeDomain::DC);
}

ReductionMapping compose_reduction_mappings(
    const ReductionMapping& first,
    const ReductionMapping& second) {
  if (first.original_to_reduced_buses.empty() &&
      first.original_to_reduced_branches.empty()) {
    return second;
  }
  if (second.original_to_reduced_buses.empty() &&
      second.original_to_reduced_branches.empty()) {
    return first;
  }

  ReductionMapping composed;
  for (const auto& [original, intermediate] :
       first.original_to_reduced_buses) {
    const auto next = second.original_to_reduced_buses.find(intermediate);
    composed.original_to_reduced_buses[original] =
        next == second.original_to_reduced_buses.end()
            ? intermediate
            : next->second;
  }
  for (const auto& [original, intermediate] :
       first.original_to_reduced_branches) {
    if (!intermediate.valid()) {
      composed.original_to_reduced_branches[original] = intermediate;
      continue;
    }
    const auto next = second.original_to_reduced_branches.find(intermediate);
    composed.original_to_reduced_branches[original] =
        next == second.original_to_reduced_branches.end()
            ? intermediate
            : next->second;
  }

  append_records(composed, first);
  append_records(composed, second);
  rebuild_reduction_reverse_maps(composed);
  return composed;
}

}  // namespace hacdcpf::graph
