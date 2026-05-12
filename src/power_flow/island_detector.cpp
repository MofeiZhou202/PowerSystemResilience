#include "hacdcpf/power_flow/island_detector.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hacdcpf::powerflow {

std::vector<IslandInfo> detect_islands(const HybridPowerSystem& sys) {
  const int nac = static_cast<int>(sys.ac.buses.size());
  const int ndc = static_cast<int>(sys.dc.buses.size());
  const int n_total = nac + ndc;

  std::vector<IslandInfo> islands;
  if (n_total == 0) {
    return islands;
  }

  std::vector<std::vector<int>> adj(static_cast<size_t>(n_total));
  auto add_edge = [&](int u, int v) {
    if (u < 0 || v < 0 || u >= n_total || v >= n_total || u == v) {
      return;
    }
    adj[static_cast<size_t>(u)].push_back(v);
    adj[static_cast<size_t>(v)].push_back(u);
  };

  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) {
      continue;
    }
    const int u = br.from_bus - 1;
    const int v = br.to_bus - 1;
    if (u >= 0 && v >= 0 && u < nac && v < nac) {
      add_edge(u, v);
    }
  }

  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) {
      continue;
    }
    const int u = nac + br.from_bus - 1;
    const int v = nac + br.to_bus - 1;
    if (u >= nac && v >= nac && u < n_total && v < n_total) {
      add_edge(u, v);
    }
  }

  for (const auto& conv : sys.vsc_converters) {
    if (!conv.in_service) {
      continue;
    }
    const int u = conv.bus_ac - 1;
    const int v = nac + conv.bus_dc - 1;
    if (u >= 0 && u < nac && v >= nac && v < n_total) {
      add_edge(u, v);
    }
  }

  std::vector<char> visited(static_cast<size_t>(n_total), 0);
  std::vector<int> stack;
  for (int start = 0; start < n_total; ++start) {
    if (visited[static_cast<size_t>(start)] != 0) {
      continue;
    }

    std::vector<int> component;
    stack.clear();
    stack.push_back(start);

    while (!stack.empty()) {
      const int node = stack.back();
      stack.pop_back();
      if (visited[static_cast<size_t>(node)] != 0) {
        continue;
      }
      visited[static_cast<size_t>(node)] = 1;
      component.push_back(node);
      for (int nb : adj[static_cast<size_t>(node)]) {
        if (visited[static_cast<size_t>(nb)] == 0) {
          stack.push_back(nb);
        }
      }
    }

    std::vector<int> ac_buses;
    std::vector<int> dc_buses;
    ac_buses.reserve(component.size());
    dc_buses.reserve(component.size());
    for (int node : component) {
      if (node < nac) {
        ac_buses.push_back(node + 1);
      } else {
        dc_buses.push_back(node - nac + 1);
      }
    }

    if (ac_buses.empty()) {
      continue;
    }

    std::sort(ac_buses.begin(), ac_buses.end());
    std::sort(dc_buses.begin(), dc_buses.end());
    std::unordered_set<int> ac_set(ac_buses.begin(), ac_buses.end());
    std::unordered_set<int> dc_set(dc_buses.begin(), dc_buses.end());

    std::vector<int> converters;
    converters.reserve(sys.vsc_converters.size());
    for (int i = 0; i < static_cast<int>(sys.vsc_converters.size()); ++i) {
      const auto& conv = sys.vsc_converters[static_cast<size_t>(i)];
      if (!conv.in_service) {
        continue;
      }
      if (ac_set.count(conv.bus_ac) != 0 || dc_set.count(conv.bus_dc) != 0) {
        converters.push_back(i + 1);
      }
    }

    bool has_generators = false;
    for (int bus : ac_buses) {
      const auto& b = sys.ac.buses[static_cast<size_t>(bus - 1)];
      if (b.bus_type == BusType::SLACK || b.bus_type == BusType::PV) {
        has_generators = true;
        break;
      }
    }
    if (!has_generators) {
      for (const auto& g : sys.ac.generators) {
        if (!g.in_service || ac_set.count(g.bus) == 0) {
          continue;
        }
        if (g.pg_mw > 1e-9 || g.is_slack) {
          has_generators = true;
          break;
        }
      }
    }
    if (!has_generators) {
      for (int conv_idx : converters) {
        const auto& conv = sys.vsc_converters[static_cast<size_t>(conv_idx - 1)];
        if (std::abs(conv.p_set_mw) > 1e-9) {
          has_generators = true;
          break;
        }
      }
    }
    // Check DG sources: static generators, renewable gens, PV systems, storage.
    if (!has_generators) {
      for (const auto& sg : sys.ac.static_generators) {
        if (!sg.in_service || ac_set.count(sg.bus) == 0) continue;
        if (sg.p_mw * sg.scaling > 1e-9) { has_generators = true; break; }
      }
    }
    if (!has_generators) {
      for (const auto& rg : sys.ac.renewable_gens) {
        if (!rg.in_service || ac_set.count(rg.bus) == 0) continue;
        if (rg.p_mw > 1e-9) { has_generators = true; break; }
      }
    }
    if (!has_generators) {
      for (const auto& pv : sys.ac.pv_systems) {
        if (!pv.in_service || ac_set.count(pv.bus) == 0) continue;
        if (pv.p_mw > 1e-9) { has_generators = true; break; }
      }
    }
    if (!has_generators) {
      for (const auto& st : sys.ac.storage) {
        if (!st.in_service || ac_set.count(st.bus) == 0) continue;
        if (std::abs(st.p_mw) > 1e-9) { has_generators = true; break; }
      }
    }
    if (!has_generators) {
      for (const auto& vpp : sys.vpps) {
        if (!vpp.in_service || ac_set.count(vpp.aggregation_bus) == 0) continue;
        if (vpp.p_output_mw > 1e-9) { has_generators = true; break; }
      }
    }

    bool has_ac_slack = false;
    int ac_slack_bus = 0;
    for (int bus : ac_buses) {
      if (sys.ac.buses[static_cast<size_t>(bus - 1)].bus_type == BusType::SLACK) {
        has_ac_slack = true;
        ac_slack_bus = bus;
        break;
      }
    }

    bool has_dc_slack = !dc_buses.empty();
    int dc_slack_bus = 0;
    if (!dc_buses.empty()) {
      dc_slack_bus = dc_buses.front();
      for (int bus : dc_buses) {
        if (sys.dc.buses[static_cast<size_t>(bus - 1)].bus_type == DCBusType::DC_V) {
          dc_slack_bus = bus;
          break;
        }
      }
    }

    IslandInfo island;
    island.id = static_cast<int>(islands.size()) + 1;
    island.has_ac_slack = has_ac_slack;
    island.has_dc_slack = has_dc_slack;
    island.ac_slack_bus = ac_slack_bus;
    island.dc_slack_bus = dc_slack_bus;
    island.has_generators = has_generators;
    island.ac_buses = std::move(ac_buses);
    island.dc_buses = std::move(dc_buses);
    island.converters = std::move(converters);
    islands.push_back(std::move(island));
  }

  return islands;
}

HybridPowerSystem extract_island_subsystem(const HybridPowerSystem& sys,
                                           const IslandInfo& island,
                                           int slack_bus_override) {
  HybridPowerSystem sub;
  sub.base_mva = sys.base_mva;
  sub.name = sys.name + " island " + std::to_string(island.id);
  sub.ac.base_mva = sys.ac.base_mva;
  sub.ac.name = sys.ac.name + " island " + std::to_string(island.id);
  sub.dc.base_mva = sys.dc.base_mva;
  sub.dc.name = sys.dc.name + " island " + std::to_string(island.id);

  std::vector<int> ac_sorted = island.ac_buses;
  std::vector<int> dc_sorted = island.dc_buses;
  std::sort(ac_sorted.begin(), ac_sorted.end());
  std::sort(dc_sorted.begin(), dc_sorted.end());

  std::unordered_map<int, int> ac_map;
  std::unordered_map<int, int> dc_map;
  ac_map.reserve(ac_sorted.size());
  dc_map.reserve(dc_sorted.size());

  sub.ac.buses.reserve(ac_sorted.size());
  sub.ac.branches.reserve(sys.ac.branches.size());
  sub.ac.generators.reserve(sys.ac.generators.size());
  sub.ac.loads.reserve(sys.ac.loads.size());
  sub.ac.static_generators.reserve(sys.ac.static_generators.size());
  sub.ac.renewable_gens.reserve(sys.ac.renewable_gens.size());
  sub.ac.pv_systems.reserve(sys.ac.pv_systems.size());
  sub.ac.storage.reserve(sys.ac.storage.size());
  sub.ac.shunts.reserve(sys.ac.shunts.size());
  sub.ac.charging_stations.reserve(sys.ac.charging_stations.size());
  sub.ac.external_grids.reserve(sys.ac.external_grids.size());
  sub.dc.buses.reserve(dc_sorted.size());
  sub.dc.branches.reserve(sys.dc.branches.size());
  sub.vsc_converters.reserve(sys.vsc_converters.size());

  for (int i = 0; i < static_cast<int>(ac_sorted.size()); ++i) {
    const int orig = ac_sorted[static_cast<size_t>(i)];
    if (orig < 1 || orig > static_cast<int>(sys.ac.buses.size())) {
      continue;
    }
    ac_map.emplace(orig, i + 1);
    ACBus bus = sys.ac.buses[static_cast<size_t>(orig - 1)];
    bus.index = i + 1;
    if (orig == slack_bus_override) {
      bus.bus_type = BusType::SLACK;
      bus.va_deg = 0.0;
    }
    sub.ac.buses.push_back(std::move(bus));
  }

  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) {
      continue;
    }
    const auto itf = ac_map.find(br.from_bus);
    const auto itt = ac_map.find(br.to_bus);
    if (itf == ac_map.end() || itt == ac_map.end()) {
      continue;
    }
    ACBranch copy = br;
    copy.index = static_cast<int>(sub.ac.branches.size()) + 1;
    copy.from_bus = itf->second;
    copy.to_bus = itt->second;
    sub.ac.branches.push_back(std::move(copy));
  }

  for (const auto& gen : sys.ac.generators) {
    if (!gen.in_service) {
      continue;
    }
    const auto it = ac_map.find(gen.bus);
    if (it == ac_map.end()) {
      continue;
    }
    Generator copy = gen;
    copy.index = static_cast<int>(sub.ac.generators.size()) + 1;
    copy.bus = it->second;
    sub.ac.generators.push_back(std::move(copy));
  }

  for (int i = 0; i < static_cast<int>(dc_sorted.size()); ++i) {
    const int orig = dc_sorted[static_cast<size_t>(i)];
    if (orig < 1 || orig > static_cast<int>(sys.dc.buses.size())) {
      continue;
    }
    dc_map.emplace(orig, i + 1);
    DCBus bus = sys.dc.buses[static_cast<size_t>(orig - 1)];
    bus.index = i + 1;
    sub.dc.buses.push_back(std::move(bus));
  }

  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) {
      continue;
    }
    const auto itf = dc_map.find(br.from_bus);
    const auto itt = dc_map.find(br.to_bus);
    if (itf == dc_map.end() || itt == dc_map.end()) {
      continue;
    }
    DCBranch copy = br;
    copy.index = static_cast<int>(sub.dc.branches.size()) + 1;
    copy.from_bus = itf->second;
    copy.to_bus = itt->second;
    sub.dc.branches.push_back(std::move(copy));
  }

  for (const auto& conv : sys.vsc_converters) {
    if (!conv.in_service) {
      continue;
    }
    const auto it_ac = ac_map.find(conv.bus_ac);
    const auto it_dc = dc_map.find(conv.bus_dc);
    if (it_ac == ac_map.end() || it_dc == dc_map.end()) {
      continue;
    }
    VSCConverter copy = conv;
    copy.index = static_cast<int>(sub.vsc_converters.size()) + 1;
    copy.bus_ac = it_ac->second;
    copy.bus_dc = it_dc->second;
    sub.vsc_converters.push_back(std::move(copy));
  }

  // Copy all AC component tables with bus remapping.
  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    const auto it = ac_map.find(ld.bus);
    if (it == ac_map.end()) continue;
    Load copy = ld;
    copy.index = static_cast<int>(sub.ac.loads.size()) + 1;
    copy.bus = it->second;
    sub.ac.loads.push_back(std::move(copy));
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    const auto it = ac_map.find(sg.bus);
    if (it == ac_map.end()) continue;
    StaticGenerator copy = sg;
    copy.index = static_cast<int>(sub.ac.static_generators.size()) + 1;
    copy.bus = it->second;
    sub.ac.static_generators.push_back(std::move(copy));
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    const auto it = ac_map.find(rg.bus);
    if (it == ac_map.end()) continue;
    RenewableGen copy = rg;
    copy.index = static_cast<int>(sub.ac.renewable_gens.size()) + 1;
    copy.bus = it->second;
    sub.ac.renewable_gens.push_back(std::move(copy));
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    const auto it = ac_map.find(pv.bus);
    if (it == ac_map.end()) continue;
    PVSystem copy = pv;
    copy.index = static_cast<int>(sub.ac.pv_systems.size()) + 1;
    copy.bus = it->second;
    sub.ac.pv_systems.push_back(std::move(copy));
  }
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    const auto it = ac_map.find(st.bus);
    if (it == ac_map.end()) continue;
    Storage copy = st;
    copy.index = static_cast<int>(sub.ac.storage.size()) + 1;
    copy.bus = it->second;
    sub.ac.storage.push_back(std::move(copy));
  }
  for (const auto& sh : sys.ac.shunts) {
    if (!sh.in_service) continue;
    const auto it = ac_map.find(sh.bus);
    if (it == ac_map.end()) continue;
    Shunt copy = sh;
    copy.index = static_cast<int>(sub.ac.shunts.size()) + 1;
    copy.bus = it->second;
    sub.ac.shunts.push_back(std::move(copy));
  }
  for (const auto& cs : sys.ac.charging_stations) {
    if (!cs.in_service) continue;
    const auto it = ac_map.find(cs.bus);
    if (it == ac_map.end()) continue;
    ChargingStation copy = cs;
    copy.index = static_cast<int>(sub.ac.charging_stations.size()) + 1;
    copy.bus = it->second;
    sub.ac.charging_stations.push_back(std::move(copy));
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const auto it = ac_map.find(eg.bus);
    if (it == ac_map.end()) continue;
    ExternalGrid copy = eg;
    copy.index = static_cast<int>(sub.ac.external_grids.size()) + 1;
    copy.bus = it->second;
    sub.ac.external_grids.push_back(std::move(copy));
  }

  // Copy DC component tables with bus remapping.
  for (const auto& st : sys.dc.storage) {
    if (!st.in_service) continue;
    const auto it = dc_map.find(st.bus);
    if (it == dc_map.end()) continue;
    Storage copy = st;
    copy.index = static_cast<int>(sub.dc.storage.size()) + 1;
    copy.bus = it->second;
    sub.dc.storage.push_back(std::move(copy));
  }
  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    const auto it = dc_map.find(sg.bus);
    if (it == dc_map.end()) continue;
    StaticGenerator copy = sg;
    copy.index = static_cast<int>(sub.dc.static_generators.size()) + 1;
    copy.bus = it->second;
    sub.dc.static_generators.push_back(std::move(copy));
  }

  // Copy VPP/Microgrid/MobileStorage with bus remapping.
  for (const auto& vpp : sys.vpps) {
    if (!vpp.in_service) continue;
    const auto it = ac_map.find(vpp.aggregation_bus);
    if (it == ac_map.end()) continue;
    VirtualPowerPlant copy = vpp;
    copy.index = static_cast<int>(sub.vpps.size()) + 1;
    copy.aggregation_bus = it->second;
    sub.vpps.push_back(std::move(copy));
  }
  for (const auto& mg : sys.microgrids) {
    if (!mg.in_service) continue;
    const auto it = ac_map.find(mg.aggregation_bus);
    if (it == ac_map.end()) continue;
    Microgrid copy = mg;
    copy.index = static_cast<int>(sub.microgrids.size()) + 1;
    copy.aggregation_bus = it->second;
    sub.microgrids.push_back(std::move(copy));
  }
  for (const auto& ms : sys.mobile_storage) {
    if (!ms.in_service) continue;
    const auto it = ac_map.find(ms.bus);
    if (it == ac_map.end()) continue;
    MobileStorage copy = ms;
    copy.index = static_cast<int>(sub.mobile_storage.size()) + 1;
    copy.bus = it->second;
    sub.mobile_storage.push_back(std::move(copy));
  }

  bool has_slack = false;
  for (const auto& bus : sub.ac.buses) {
    if (bus.bus_type == BusType::SLACK) {
      has_slack = true;
      break;
    }
  }
  if (!has_slack && !sub.ac.buses.empty()) {
    sub.ac.buses.front().bus_type = BusType::SLACK;
    sub.ac.buses.front().va_deg = 0.0;
  }

  return sub;
}

}  // namespace hacdcpf::powerflow
