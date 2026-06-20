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

  // Bus IDs are NOT guaranteed to be a contiguous 1..N sequence (e.g. multiple
  // feeders numbered 1-5, 10-14, 20-21).  Map each real bus index to its array
  // position so connectivity follows the actual topology rather than the raw
  // ``index - 1`` offset.  ``ac_node`` / ``dc_node`` return the graph node for
  // a given bus ID (or -1 when the ID is unknown); ``add_edge`` ignores -1.
  std::unordered_map<int, int> ac_id_to_pos;
  std::unordered_map<int, int> dc_id_to_pos;
  ac_id_to_pos.reserve(static_cast<size_t>(nac));
  dc_id_to_pos.reserve(static_cast<size_t>(ndc));
  for (int i = 0; i < nac; ++i) {
    ac_id_to_pos[sys.ac.buses[static_cast<size_t>(i)].index] = i;
  }
  for (int i = 0; i < ndc; ++i) {
    dc_id_to_pos[sys.dc.buses[static_cast<size_t>(i)].index] = i;
  }
  auto ac_node = [&](int bus_id) -> int {
    const auto it = ac_id_to_pos.find(bus_id);
    return (it == ac_id_to_pos.end()) ? -1 : it->second;
  };
  auto dc_node = [&](int bus_id) -> int {
    const auto it = dc_id_to_pos.find(bus_id);
    return (it == dc_id_to_pos.end()) ? -1 : (nac + it->second);
  };

  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) {
      continue;
    }
    add_edge(ac_node(br.from_bus), ac_node(br.to_bus));
  }

  // Closed AC switches connect their two buses (zero/near-zero impedance link).
  // Without this, a system whose connectivity is dominated by switches/CBs
  // (e.g. ring distribution networks with circuit breakers between feeders)
  // would be reported as many disjoint islands, causing the GUI to dispatch
  // the adaptive solver and leave most buses with vm = 0.
  for (const auto& sw : sys.ac.switches) {
    if (!sw.in_service || !sw.closed) continue;
    add_edge(ac_node(sw.bus_from), ac_node(sw.bus_to));
  }

  // Closed AC circuit breakers also act as zero-impedance connections.
  for (const auto& cb : sys.ac.circuit_breakers) {
    if (!cb.in_service || !cb.closed) continue;
    add_edge(ac_node(cb.bus_from), ac_node(cb.bus_to));
  }

  // 2W and 3W transformers connect AC buses (in-service ones contribute
  // electrical paths just like branches).  Without these edges, a network
  // whose AC connectivity goes through transformers would be fragmented
  // into many spurious singleton islands.
  for (const auto& tr : sys.ac.transformers_2w) {
    if (!tr.in_service) continue;
    add_edge(ac_node(tr.hv_bus), ac_node(tr.lv_bus));
  }
  for (const auto& tr : sys.ac.transformers_3w) {
    if (!tr.in_service) continue;
    const int h = ac_node(tr.hv_bus);
    const int m = ac_node(tr.mv_bus);
    const int l = ac_node(tr.lv_bus);
    add_edge(h, m);
    add_edge(h, l);
    add_edge(m, l);
  }

  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) {
      continue;
    }
    add_edge(dc_node(br.from_bus), dc_node(br.to_bus));
  }

  // Closed DC circuit breakers connect their two DC buses.
  for (const auto& cb : sys.dc.dc_circuit_breakers) {
    if (!cb.in_service || !cb.closed) continue;
    add_edge(dc_node(cb.bus_from), dc_node(cb.bus_to));
  }

  for (const auto& conv : sys.vsc_converters) {
    if (!conv.in_service) {
      continue;
    }
    add_edge(ac_node(conv.bus_ac), dc_node(conv.bus_dc));
  }

  for (const auto& dcdc : sys.dc.dcdc_converters) {
    if (!dcdc.in_service) {
      continue;
    }
    add_edge(dc_node(dcdc.bus_in), dc_node(dcdc.bus_out));
  }

  // Energy Routers: before expansion, their ports bridge AC buses
  // that would otherwise be in separate islands.  Chain all in-service
  // port buses of the same ER so the BFS recognises the connection.
  for (const auto& er : sys.energy_routers) {
    if (!er.in_service) {
      continue;
    }
    int prev = -1;
    for (const auto& port : er.ports) {
      if (!port.in_service) {
        continue;
      }
      const int u = ac_node(port.bus);
      if (u < 0) {
        continue;
      }
      if (prev >= 0) {
        add_edge(prev, u);
      }
      prev = u;
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
        ac_buses.push_back(sys.ac.buses[static_cast<size_t>(node)].index);
      } else {
        dc_buses.push_back(sys.dc.buses[static_cast<size_t>(node - nac)].index);
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
    // NOTE: We do NOT use ``bus_type == SLACK`` (or PV) as a proxy for a
    // real injection.  A SLACK-typed bus without an actual source element
    // attached is a data artifact (commonly seen after a circuit breaker
    // disconnects a feeder, leaving its terminal bus stranded as a SLACK
    // node with no backing generator/ext-grid).  Treating such buses as
    // sources would make the adaptive solver believe a sourceless island
    // is alive — and produce physically meaningless voltages instead of
    // zeros.  Below we look only at concrete in-service injections.
    for (const auto& g : sys.ac.generators) {
      if (!g.in_service || ac_set.count(g.bus) == 0) {
        continue;
      }
      if (g.pg_mw > 1e-9 || g.is_slack) {
        has_generators = true;
        break;
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
        if (!vpp.in_service || ac_set.count(vpp.pcc_bus) == 0) continue;
        if (vpp.p_output_mw > 1e-9) { has_generators = true; break; }
      }
    }
    // External grids act as an unlimited slack source — treat any island
    // hosting an in-service ExternalGrid as solvable, even if the bus
    // type has not yet been promoted to BusType::SLACK (promotion happens
    // later in solver_data.cpp).
    if (!has_generators) {
      for (const auto& eg : sys.ac.external_grids) {
        if (!eg.in_service || ac_set.count(eg.bus) == 0) continue;
        has_generators = true;
        break;
      }
    }

    bool has_ac_slack = false;
    int ac_slack_bus = 0;
    for (int bus : ac_buses) {
      const auto it = ac_id_to_pos.find(bus);
      if (it != ac_id_to_pos.end() &&
          sys.ac.buses[static_cast<size_t>(it->second)].bus_type == BusType::SLACK) {
        has_ac_slack = true;
        ac_slack_bus = bus;
        break;
      }
    }
    // External grids implicitly provide a slack reference at their bus.
    if (!has_ac_slack) {
      for (const auto& eg : sys.ac.external_grids) {
        if (!eg.in_service || ac_set.count(eg.bus) == 0) continue;
        has_ac_slack = true;
        ac_slack_bus = eg.bus;
        break;
      }
    }

    bool has_dc_slack = !dc_buses.empty();
    int dc_slack_bus = 0;
    if (!dc_buses.empty()) {
      dc_slack_bus = dc_buses.front();
      // Prefer a DC_V reference; otherwise anchor on the first non-isolated
      // bus.  DC_ISOLATED buses are de-energized and are only used as a
      // fallback anchor when the island contains nothing else.
      int first_non_isolated = -1;
      for (int bus : dc_buses) {
        const auto it = dc_id_to_pos.find(bus);
        if (it == dc_id_to_pos.end()) {
          continue;
        }
        const auto bt = sys.dc.buses[static_cast<size_t>(it->second)].bus_type;
        if (bt == DCBusType::DC_V) {
          dc_slack_bus = bus;
          first_non_isolated = bus;
          break;
        }
        if (first_non_isolated < 0 && bt != DCBusType::DC_ISOLATED) {
          first_non_isolated = bus;
        }
      }
      if (first_non_isolated >= 0) dc_slack_bus = first_non_isolated;
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

  // ``island.ac_buses`` / ``dc_buses`` hold real bus indices, which need not be
  // a contiguous 1..N sequence.  Map each real index to its array position so
  // the original bus records can be located regardless of numbering gaps.
  std::unordered_map<int, int> ac_id_to_pos;
  std::unordered_map<int, int> dc_id_to_pos;
  ac_id_to_pos.reserve(sys.ac.buses.size());
  dc_id_to_pos.reserve(sys.dc.buses.size());
  for (int i = 0; i < static_cast<int>(sys.ac.buses.size()); ++i) {
    ac_id_to_pos[sys.ac.buses[static_cast<size_t>(i)].index] = i;
  }
  for (int i = 0; i < static_cast<int>(sys.dc.buses.size()); ++i) {
    dc_id_to_pos[sys.dc.buses[static_cast<size_t>(i)].index] = i;
  }

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
    const auto pit = ac_id_to_pos.find(orig);
    if (pit == ac_id_to_pos.end()) {
      continue;
    }
    ac_map.emplace(orig, i + 1);
    ACBus bus = sys.ac.buses[static_cast<size_t>(pit->second)];
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

  // AC switches and circuit breakers act as connectivity (when closed).
  // Without copying them into the sub-system the canonical projection
  // run inside the adaptive solver will not be able to splice them
  // into branches, leaving most buses electrically disconnected and
  // producing a singular Jacobian.
  for (const auto& sw : sys.ac.switches) {
    const auto itf = ac_map.find(sw.bus_from);
    const auto itt = ac_map.find(sw.bus_to);
    if (itf == ac_map.end() || itt == ac_map.end()) continue;
    Switch copy = sw;
    copy.index = static_cast<int>(sub.ac.switches.size()) + 1;
    copy.bus_from = itf->second;
    copy.bus_to = itt->second;
    sub.ac.switches.push_back(std::move(copy));
  }
  for (const auto& cb : sys.ac.circuit_breakers) {
    const auto itf = ac_map.find(cb.bus_from);
    const auto itt = ac_map.find(cb.bus_to);
    if (itf == ac_map.end() || itt == ac_map.end()) continue;
    CircuitBreaker copy = cb;
    copy.index = static_cast<int>(sub.ac.circuit_breakers.size()) + 1;
    copy.bus_from = itf->second;
    copy.bus_to = itt->second;
    sub.ac.circuit_breakers.push_back(std::move(copy));
  }

  // 2W / 3W transformers carry their own bus references and contribute
  // to branch admittance via canonical projection, so they must be
  // copied as well.
  for (const auto& tr : sys.ac.transformers_2w) {
    if (!tr.in_service) continue;
    const auto itf = ac_map.find(tr.hv_bus);
    const auto itt = ac_map.find(tr.lv_bus);
    if (itf == ac_map.end() || itt == ac_map.end()) continue;
    Transformer2W copy = tr;
    copy.index = static_cast<int>(sub.ac.transformers_2w.size()) + 1;
    copy.hv_bus = itf->second;
    copy.lv_bus = itt->second;
    sub.ac.transformers_2w.push_back(std::move(copy));
  }
  for (const auto& tr : sys.ac.transformers_3w) {
    if (!tr.in_service) continue;
    const auto ith = ac_map.find(tr.hv_bus);
    const auto itm = ac_map.find(tr.mv_bus);
    const auto itl = ac_map.find(tr.lv_bus);
    if (ith == ac_map.end() || itm == ac_map.end() || itl == ac_map.end()) continue;
    Transformer3W copy = tr;
    copy.index = static_cast<int>(sub.ac.transformers_3w.size()) + 1;
    copy.hv_bus = ith->second;
    copy.mv_bus = itm->second;
    copy.lv_bus = itl->second;
    sub.ac.transformers_3w.push_back(std::move(copy));
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
    const auto pit = dc_id_to_pos.find(orig);
    if (pit == dc_id_to_pos.end()) {
      continue;
    }
    dc_map.emplace(orig, i + 1);
    DCBus bus = sys.dc.buses[static_cast<size_t>(pit->second)];
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
  for (const auto& fl : sys.ac.flexible_loads) {
    if (!fl.in_service) continue;
    const auto it = ac_map.find(fl.bus);
    if (it == ac_map.end()) continue;
    FlexibleLoad copy = fl;
    copy.index = static_cast<int>(sub.ac.flexible_loads.size()) + 1;
    copy.bus = it->second;
    sub.ac.flexible_loads.push_back(std::move(copy));
  }
  for (const auto& al : sys.ac.asymmetric_loads) {
    if (!al.in_service) continue;
    const auto it = ac_map.find(al.bus);
    if (it == ac_map.end()) continue;
    AsymmetricLoad copy = al;
    copy.index = static_cast<int>(sub.ac.asymmetric_loads.size()) + 1;
    copy.bus = it->second;
    sub.ac.asymmetric_loads.push_back(std::move(copy));
  }
  for (const auto& m : sys.ac.motors) {
    if (!m.in_service) continue;
    const auto it = ac_map.find(m.bus);
    if (it == ac_map.end()) continue;
    AsynchronousMotor copy = m;
    copy.index = static_cast<int>(sub.ac.motors.size()) + 1;
    copy.bus = it->second;
    sub.ac.motors.push_back(std::move(copy));
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
  for (const auto& ld : sys.dc.loads) {
    if (!ld.in_service) continue;
    const auto it = dc_map.find(ld.bus);
    if (it == dc_map.end()) continue;
    DCLoad copy = ld;
    copy.index = static_cast<int>(sub.dc.loads.size()) + 1;
    copy.bus = it->second;
    sub.dc.loads.push_back(std::move(copy));
  }
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    const auto it = dc_map.find(sg.bus);
    if (it == dc_map.end()) continue;
    StaticGeneratorDC copy = sg;
    copy.index = static_cast<int>(sub.dc.dc_static_generators.size()) + 1;
    copy.bus = it->second;
    sub.dc.dc_static_generators.push_back(std::move(copy));
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    const auto it = dc_map.find(pv.bus);
    if (it == dc_map.end()) continue;
    PVArrayDC copy = pv;
    copy.index = static_cast<int>(sub.dc.pv_arrays.size()) + 1;
    copy.bus = it->second;
    sub.dc.pv_arrays.push_back(std::move(copy));
  }
  for (const auto& cb : sys.dc.dc_circuit_breakers) {
    const auto itf = dc_map.find(cb.bus_from);
    const auto itt = dc_map.find(cb.bus_to);
    if (itf == dc_map.end() || itt == dc_map.end()) continue;
    DCCircuitBreaker copy = cb;
    copy.index = static_cast<int>(sub.dc.dc_circuit_breakers.size()) + 1;
    copy.bus_from = itf->second;
    copy.bus_to = itt->second;
    sub.dc.dc_circuit_breakers.push_back(std::move(copy));
  }

  // DCDC converters: copy when both buses are in this island.
  for (const auto& dc : sys.dc.dcdc_converters) {
    if (!dc.in_service) continue;
    const auto iti = dc_map.find(dc.bus_in);
    const auto ito = dc_map.find(dc.bus_out);
    if (iti == dc_map.end() || ito == dc_map.end()) continue;
    DCDCConverter copy = dc;
    copy.index = static_cast<int>(sub.dc.dcdc_converters.size()) + 1;
    copy.bus_in = iti->second;
    copy.bus_out = ito->second;
    sub.dc.dcdc_converters.push_back(std::move(copy));
  }

  // Copy VPP/Microgrid/MobileStorage with bus remapping.
  for (const auto& vpp : sys.vpps) {
    if (!vpp.in_service) continue;
    const auto it = ac_map.find(vpp.pcc_bus);
    if (it == ac_map.end()) continue;
    VirtualPowerPlant copy = vpp;
    copy.index = static_cast<int>(sub.vpps.size()) + 1;
    copy.pcc_bus = it->second;
    sub.vpps.push_back(std::move(copy));
  }
  for (const auto& mg : sys.microgrids) {
    if (!mg.in_service) continue;
    const auto it = ac_map.find(mg.pcc_bus);
    if (it == ac_map.end()) continue;
    Microgrid copy = mg;
    copy.index = static_cast<int>(sub.microgrids.size()) + 1;
    copy.pcc_bus = it->second;
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
