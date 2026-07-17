// @ts-check

import process from 'node:process';

function arg(name, fallback) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length
    ? process.argv[index + 1]
    : fallback;
}

async function request(base, endpoint, body = {}) {
  const response = await fetch(base + endpoint, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await response.json();
  if (!response.ok || data.error) throw new Error(data.error || response.statusText);
  return data;
}

function positive(value) {
  return Math.max(0, Number(value) || 0);
}

async function main() {
  const base = arg('base-url', 'http://127.0.0.1:18085');
  const opfBody = branchLimits => ({
    solver: 'dc',
    constraints: { branch_limits: branchLimits },
    options: { load_shedding: true, voll: 1_000_000 },
  });
  const [topology, exported] = await Promise.all([
    request(base, '/api/session/topology'),
    request(base, '/api/session/export_json'),
  ]);
  // Analyses share one guarded server session and must run sequentially.
  const limitedOpf = await request(base, '/api/session/opf', opfBody(true));
  const unlimitedOpf = await request(base, '/api/session/opf', opfBody(false));
  const system = JSON.parse(exported.json_string);
  const islands = topology.islands || [];
  const islandByBus = new Map();
  islands.forEach(island => (island.ac_bus_ids || []).forEach(bus =>
    islandByBus.set(Number(bus), Number(island.island_id))));

  const loadByBus = new Map();
  const loadRowsByBus = new Map();
  const addLoad = (bus, mw, name, kind) => {
    const id = Number(bus);
    const value = positive(mw);
    if (!Number.isFinite(id) || value <= 0) return;
    loadByBus.set(id, (loadByBus.get(id) || 0) + value);
    if (!loadRowsByBus.has(id)) loadRowsByBus.set(id, []);
    loadRowsByBus.get(id).push({ name, kind, p_mw: value });
  };
  for (const bus of system.ac?.buses || []) {
    if (bus.in_service !== false) addLoad(bus.index, bus.pd_mw, bus.name || `Bus ${bus.index}`, 'bus');
  }
  for (const load of system.ac?.loads || []) {
    if (load.in_service !== false) {
      addLoad(load.bus, positive(load.p_mw) * (Number(load.scaling) || 1),
        load.name || `Load ${load.index}`, 'load');
    }
  }

  const boundariesByIsland = new Map();
  const addBoundary = (islandId, row) => {
    if (!boundariesByIsland.has(islandId)) boundariesByIsland.set(islandId, []);
    boundariesByIsland.get(islandId).push(row);
  };
  for (const sw of system.ac?.switches || []) {
    const fromIsland = islandByBus.get(Number(sw.bus_from));
    const toIsland = islandByBus.get(Number(sw.bus_to));
    if (fromIsland == null || toIsland == null || fromIsland === toIsland) continue;
    const row = {
      index: sw.index,
      name: sw.name,
      switch_type: sw.switch_type,
      role: sw.role,
      closed: sw.closed !== false,
      normal_closed: sw.normal_closed !== false,
      from_bus: sw.bus_from,
      to_bus: sw.bus_to,
      from_island: fromIsland,
      to_island: toIsland,
    };
    addBoundary(fromIsland, row);
    addBoundary(toIsland, row);
  }

  const sourceBuses = (system.ac?.external_grids || [])
    .filter(source => source.in_service !== false)
    .map(source => Number(source.bus));
  const slackBuses = (system.ac?.buses || [])
    .filter(bus => bus.in_service !== false && Number(bus.bus_type) === 1)
    .map(bus => Number(bus.index));
  const mainIsland = [...sourceBuses, ...slackBuses]
    .map(bus => islandByBus.get(bus))
    .find(id => id != null) ?? 0;

  const islandRows = islands.map(island => {
    const buses = (island.ac_bus_ids || []).map(Number);
    const loadMw = buses.reduce((sum, bus) => sum + (loadByBus.get(bus) || 0), 0);
    return {
      island_id: island.island_id,
      status: island.status,
      n_buses: buses.length,
      buses,
      load_mw: Number(loadMw.toFixed(6)),
      loads: buses.flatMap(bus => (loadRowsByBus.get(bus) || []).map(load => ({ bus, ...load }))),
      boundary_switches: boundariesByIsland.get(Number(island.island_id)) || [],
    };
  });
  const disconnected = islandRows.filter(row => Number(row.island_id) !== Number(mainIsland));
  const affected = disconnected.filter(row => row.load_mw > 0);
  const disconnectedLoadMw = affected.reduce((sum, row) => sum + row.load_mw, 0);
  const limitedShedMw = positive(limitedOpf.total_load_shedding_mw);
  const unlimitedShedMw = positive(unlimitedOpf.total_load_shedding_mw);
  const unlimitedFlowByIndex = new Map(
    (unlimitedOpf.dc_opf_branch_dispatch || []).map(row => [Number(row.index), row]));
  const limitingBranches = (limitedOpf.dc_opf_branch_dispatch || [])
    .map(row => {
      const rating = positive(row.rate_mva);
      const limitedFlow = Math.abs(Number(row.pf_mw) || 0);
      const unlimited = unlimitedFlowByIndex.get(Number(row.index));
      const requiredFlow = Math.abs(Number(unlimited?.pf_mw) || 0);
      return {
        index: row.index,
        name: row.name,
        from_bus: row.from_bus,
        to_bus: row.to_bus,
        rate_mva: rating,
        limited_flow_mw: limitedFlow,
        required_flow_without_limits_mw: requiredFlow,
        limited_utilization: rating > 0 ? limitedFlow / rating : null,
        required_to_rating_ratio: rating > 0 ? requiredFlow / rating : null,
      };
    })
    .filter(row => row.rate_mva > 0 &&
      (row.limited_utilization >= 0.995 || row.required_to_rating_ratio > 1.0))
    .sort((a, b) => (b.required_to_rating_ratio || 0) - (a.required_to_rating_ratio || 0));
  const typeCounts = {};
  for (const sw of system.ac?.switches || []) {
    const key = `${sw.switch_type || 'Unknown'}:${sw.closed === false ? 'open' : 'closed'}`;
    typeCounts[key] = (typeCounts[key] || 0) + 1;
  }

  console.log(JSON.stringify({
    model: exported.name,
    main_island: mainIsland,
    source_buses: [...new Set([...sourceBuses, ...slackBuses])],
    topology: {
      connected: topology.is_connected,
      ac_islands: topology.n_ac_islands,
      buses: topology.n_buses,
      active_edges: topology.n_branches,
    },
    switch_state_counts: typeCounts,
    total_load_mw: Number([...loadByBus.values()].reduce((a, b) => a + b, 0).toFixed(6)),
    disconnected_load_mw: Number(disconnectedLoadMw.toFixed(6)),
    n0_supply_audit: {
      branch_limits_converged: limitedOpf.converged === true,
      without_branch_limits_converged: unlimitedOpf.converged === true,
      total_shed_with_branch_limits_mw: Number(limitedShedMw.toFixed(6)),
      topological_island_shed_mw: Number(disconnectedLoadMw.toFixed(6)),
      source_or_analysis_model_shed_mw: Number(
        Math.max(0, unlimitedShedMw - disconnectedLoadMw).toFixed(6)),
      branch_limit_shed_mw: Number(Math.max(0, limitedShedMw - unlimitedShedMw).toFixed(6)),
      raw_normal_state_eens_mwh_per_year: Number((limitedShedMw * 8760).toFixed(3)),
      limiting_branches: limitingBranches,
    },
    load_bearing_islands: affected,
    empty_or_terminal_islands: disconnected
      .filter(row => row.load_mw <= 0)
      .map(row => ({
        island_id: row.island_id,
        buses: row.buses,
        boundary_switches: row.boundary_switches,
      })),
  }, null, 2));
}

main().catch(error => {
  console.error(error.message || error);
  process.exitCode = 1;
});
