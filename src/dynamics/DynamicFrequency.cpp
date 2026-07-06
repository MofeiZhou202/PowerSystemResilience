#include "hacdcpf/dynamics/DynamicFrequency.hpp"

#include <algorithm>
#include <numeric>
#include <vector>

#include "hacdcpf/dynamics/DynamicSystem.hpp"

namespace hacdcpf::dynamics {
namespace {

// Minimal union-find over AC bus positions for connected-component (island)
// detection on the live network.
class UnionFind {
 public:
  explicit UnionFind(int n) : parent_(n), rank_(n, 0) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  int find(int a) {
    while (parent_[a] != a) {
      parent_[a] = parent_[parent_[a]];  // path halving
      a = parent_[a];
    }
    return a;
  }

  void unite(int a, int b) {
    int ra = find(a);
    int rb = find(b);
    if (ra == rb) return;
    if (rank_[ra] < rank_[rb]) std::swap(ra, rb);
    parent_[rb] = ra;
    if (rank_[ra] == rank_[rb]) ++rank_[ra];
  }

 private:
  std::vector<int> parent_;
  std::vector<int> rank_;
};

// Accumulators for a single connected component while it is being built.
struct IslandAccumulator {
  int n_ac_buses{0};
  bool has_anchor{false};
  bool has_source{false};
  double sum_hs{0.0};      // Sum of H*S               [MW*s]
  double sum_hs_omega{0.0};  // Sum of H*S*omega        [MW*s]
};

}  // namespace

DynamicFrequencyReport computeFrequencyReport(const DynamicSystem& sys) {
  DynamicFrequencyReport report;
  const double nominal =
      sys.network.frequency_hz > 0.0 ? sys.network.frequency_hz : 50.0;
  report.nominal_frequency_hz = nominal;
  report.system_coi_frequency_hz = nominal;

  const int n_bus = static_cast<int>(sys.network.ac_bus_ids.size());
  if (n_bus <= 0) return report;

  // 1) Partition the live AC network into connected components (islands).
  UnionFind uf(n_bus);
  for (const auto& branch : sys.network.ac_branches) {
    if (!branch.in_service) continue;
    if (branch.from_pos < 0 || branch.from_pos >= n_bus) continue;
    if (branch.to_pos < 0 || branch.to_pos >= n_bus) continue;
    uf.unite(branch.from_pos, branch.to_pos);
  }

  // Compress component roots to contiguous island indices [0, n_islands).
  std::vector<int> root_to_island(n_bus, -1);
  std::vector<IslandAccumulator> accs;
  std::vector<int> bus_island(n_bus, -1);
  for (int bus = 0; bus < n_bus; ++bus) {
    const int root = uf.find(bus);
    if (root_to_island[root] < 0) {
      root_to_island[root] = static_cast<int>(accs.size());
      accs.emplace_back();
    }
    const int island = root_to_island[root];
    bus_island[bus] = island;
    ++accs[static_cast<std::size_t>(island)].n_ac_buses;
  }

  // 2) Bucket each generation-capable device into its terminal-bus island and
  //    accumulate the inertia-weighted speed sums.
  for (const auto& device : sys.devices) {
    const FrequencyParticipation fp =
        device->frequencyParticipation(sys.x, sys.y);
    if (fp.ac_bus_pos < 0 || fp.ac_bus_pos >= n_bus) continue;
    const int island = bus_island[fp.ac_bus_pos];
    if (island < 0) continue;
    IslandAccumulator& acc = accs[static_cast<std::size_t>(island)];
    acc.has_source |= fp.is_source;
    acc.has_anchor |= fp.is_anchor;
    if (fp.contributes_coi && fp.inertia_h > 0.0 && fp.base_mva > 0.0) {
      const double weight = fp.inertia_h * fp.base_mva;
      acc.sum_hs += weight;
      acc.sum_hs_omega += weight * fp.speed_pu;
    }
  }

  // 3) Emit energized islands (those carrying at least one source) and build the
  //    inertia-weighted system COI across all islands.
  double sys_sum_hs = 0.0;
  double sys_sum_hs_omega = 0.0;
  int next_island_id = 0;
  for (const auto& acc : accs) {
    sys_sum_hs += acc.sum_hs;
    sys_sum_hs_omega += acc.sum_hs_omega;
    if (!acc.has_source) continue;  // dead / passive component: no frequency

    DynamicIslandFrequency island;
    island.island_id = next_island_id++;
    island.n_ac_buses = acc.n_ac_buses;
    island.has_anchor = acc.has_anchor;
    island.has_source = acc.has_source;
    island.total_inertia_mws = acc.sum_hs;
    island.coi_frequency_hz =
        acc.sum_hs > 0.0 ? nominal * (acc.sum_hs_omega / acc.sum_hs) : nominal;
    report.islands.push_back(island);

    if (acc.has_source && !acc.has_anchor) {
      report.has_anchorless_source_island = true;
    }
  }

  report.system_coi_frequency_hz =
      sys_sum_hs > 0.0 ? nominal * (sys_sum_hs_omega / sys_sum_hs) : nominal;
  return report;
}

}  // namespace hacdcpf::dynamics
