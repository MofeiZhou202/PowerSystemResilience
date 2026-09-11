/** Bounded read-only busbar sheets. Geometry/complexity: docs/planning/gui_one_line_redesign.md § Scale-first. */
'use strict';
(function (global) {
  const core = global.HySimCore = global.HySimCore || {};
  const LIMITS = Object.freeze({ buses: 80, edges: 160, taps: 12, page: 20 });
  const keyOf = (domain, index) => `${domain}:${index}`;
  const id = value => ['number', 'string'].includes(typeof value) && String(value).trim() !== '' &&
    Number.isSafeInteger(Number(value)) ? Number(value) : null;
  const items = value => Array.isArray(value) ? value : [];

  function buildGraph(system) {
    const nodes = new Map(), edges = [], incident = new Map(), limitations = new Set();
    for (const domain of ['ac', 'dc']) {
      for (const bus of items(system?.[domain]?.buses)) {
        const index = id(bus.index);
        if (index === null) throw new Error(`${domain.toUpperCase()} bus has no valid stable index`);
        const key = keyOf(domain, index);
        if (nodes.has(key)) throw new Error(`Duplicate bus identity: ${key}`);
        nodes.set(key, { key, domain, index, bus, devices: {} });
        incident.set(key, []);
      }
    }
    const seen = new Set();
    // Endpoints follow src/io/json_io.cpp. No vector positions become public IDs.
    function add(collection, kind, record, ends, suffix = '', aliases = []) {
      const index = id(record.index);
      if (index === null) { limitations.add(`${collection}: missing stable link index`); return; }
      const key = `${collection}:${index}${suffix}`;
      if (seen.has(key)) throw new Error(`Duplicate link identity: ${key}`);
      seen.add(key);
      if (ends.some(([domain, bus]) => id(bus) === null || !nodes.has(keyOf(domain, id(bus))))) {
        limitations.add(`${collection}: invalid bus references omitted`); return;
      }
      const [a, b] = ends.map(([domain, bus]) => keyOf(domain, id(bus)));
      const edge = { key, collection, kind, index, a, b, aliases,
        name: String(record.name || ''), open: record.closed === false,
        inService: record.in_service !== false };
      edges.push(edge);
      incident.get(a).push(edge);
      if (b !== a) incident.get(b).push(edge);
    }
    const acBranches = items(system?.ac?.branches);
    const branchIds = new Set(acBranches.map(b => id(b.index)));
    const aliases = new Map();
    const transformerIds = new Set();
    for (const transformer of items(system?.ac?.transformers_2w)) {
      const stableId = id(transformer.index);
      if (stableId === null) { limitations.add('ac.transformers_2w: missing stable index'); continue; }
      if (transformerIds.has(stableId)) throw new Error(`Duplicate link identity: ac.transformers_2w:${stableId}`);
      transformerIds.add(stableId);
      const source = id(transformer.source_branch_idx);
      if (source !== null && source >= 0 && branchIds.has(source)) {
        if (!aliases.has(source)) aliases.set(source, []);
        aliases.get(source).push({ collection: 'ac.transformers_2w', index: transformer.index });
      } else add('ac.transformers_2w', 'transformer', transformer,
        [['ac', transformer.hv_bus], ['ac', transformer.lv_bus]]);
    }
    for (const branch of acBranches) add('ac.branches', aliases.has(id(branch.index)) ||
      branch.branch_kind === 'transformer' ? 'transformer' : 'line', branch,
    [['ac', branch.from_bus], ['ac', branch.to_bus]], '', aliases.get(id(branch.index)) || []);
    for (const branch of items(system?.dc?.branches)) add('dc.branches', 'dcline', branch,
      [['dc', branch.from_bus], ['dc', branch.to_bus]]);
    for (const [domain, collection, kind] of [['ac', 'switches', 'switch'],
      ['ac', 'circuit_breakers', 'breaker'], ['dc', 'dc_circuit_breakers', 'breaker']]) {
      for (const record of items(system?.[domain]?.[collection])) add(`${domain}.${collection}`, kind,
        record, [[domain, record.bus_from], [domain, record.bus_to]]);
    }
    for (const [collection, kind] of [['vsc_converters', 'vsc'], ['lcc_converters', 'lcc']]) {
      for (const record of items(system?.[collection])) add(collection, kind, record,
        [['ac', record.bus_ac], ['dc', record.bus_dc]]);
    }
    for (const [collection, records] of [['dc.dcdc_converters', system?.dc?.dcdc_converters],
      ['dcdc_converters', system?.dcdc_converters]]) {
      for (const record of items(records)) add(collection, 'dcdc', record,
        [['dc', record.bus_in], ['dc', record.bus_out]]);
    }
    for (const record of items(system?.ac?.transformers_3w)) {
      limitations.add('三绕组变压器按端口关联显示；连线不是两台独立变压器');
      for (const field of ['mv_bus', 'lv_bus']) add('ac.transformers_3w', 'transformer3w', record,
        [['ac', record.hv_bus], ['ac', record[field]]], `:${field}`);
    }
    for (const record of items(system?.energy_routers)) {
      limitations.add('能量路由器按端口关联显示；不表示内部导通或电气等值');
      const ports = items(record.ports);
      const endpoint = port => [String(port.port_type || '').toLowerCase(), port.bus];
      for (let i = 1; i < ports.length; i++) add('energy_routers', 'router', record,
        [endpoint(ports[0]), endpoint(ports[i])], `:port-${ports[i].index}`);
    }
    // Devices are summarized, not materialized as thousands of SVG symbols.
    for (const domain of ['ac', 'dc']) {
      for (const [collection, records] of Object.entries(system?.[domain] || {})) {
        if (['buses', 'branches', 'dcdc_converters'].includes(collection)) continue;
        for (const record of items(records)) {
          const node = nodes.get(keyOf(domain, id(record.bus)));
          if (node) node.devices[collection] = (node.devices[collection] || 0) + 1;
        }
      }
    }
    return { nodes, edges, incident, limitations: [...limitations] };
  }

  function extract(graph, center, options = {}) {
    const centerKey = keyOf(center.domain, id(center.index));
    if (!graph.nodes.has(centerKey)) throw new Error(`Bus not found: ${centerKey}`);
    const limit = Math.max(1, Math.min(LIMITS.buses, Math.floor(Number(options.limit) || 20)));
    const hops = Math.max(1, Math.min(4, Math.floor(Number(options.hops) || 2)));
    const hopOf = new Map([[centerKey, 0]]), queue = [centerKey];
    let truncated = false;
    // BFS with a cursor avoids quadratic queue.shift(); open devices remain
    // structural adjacency, never an energization calculation.
    for (let cursor = 0; cursor < queue.length; cursor++) {
      const key = queue[cursor], hop = hopOf.get(key);
      if (hop >= hops) continue;
      for (const edge of graph.incident.get(key)) {
        const next = edge.a === key ? edge.b : edge.a;
        if (hopOf.has(next)) continue;
        if (queue.length >= limit) { truncated = true; continue; }
        hopOf.set(next, hop + 1); queue.push(next);
      }
    }
    const internal = [], boundary = [], rendered = [], hidden = [];
    const degree = new Map(queue.map(key => [key, 0]));
    for (const edge of graph.edges) {
      const a = hopOf.has(edge.a), b = hopOf.has(edge.b);
      if (a && b) {
        internal.push(edge);
        if (edge.a !== edge.b && rendered.length < LIMITS.edges &&
            degree.get(edge.a) < LIMITS.taps && degree.get(edge.b) < LIMITS.taps) {
          rendered.push(edge);
          degree.set(edge.a, degree.get(edge.a) + 1);
          degree.set(edge.b, degree.get(edge.b) + 1);
        } else hidden.push(edge);
      } else if (a || b) boundary.push(edge);
    }
    return { graph, centerKey, hopOf, nodes: queue.map(key => graph.nodes.get(key)),
      internal, boundary, rendered, hidden, degree, hops, limit, truncated };
  }

  function layout(model) {
    // Disjoint rows: 160px pitch reserves label, bus, departure and boundary
    // bands. 20px tap pitch; lanes lie strictly beyond the bus column. See
    // gui_one_line_redesign.md § Scale-first implementation rationale.
    const maxDegree = Math.max(1, ...model.degree.values());
    const barWidth = Math.max(260, (maxDegree + 1) * 20);
    const left = 24, pitch = 160, laneStart = left + barWidth + 36;
    const positions = new Map(model.nodes.map((node, i) => [node.key,
      { x: left, y: i * pitch + 60, width: barWidth, height: 136, top: i * pitch + 12 }]));
    const slots = new Map(model.nodes.map(node => [node.key, 0]));
    const paths = model.rendered.map((edge, i) => {
      const a = positions.get(edge.a), b = positions.get(edge.b);
      const tap = key => {
        const position = positions.get(key), slot = slots.get(key) + 1;
        slots.set(key, slot);
        return { x: position.x + slot * barWidth / (model.degree.get(key) + 1),
          departureY: position.y + 10 + slot * 4 };
      };
      const at = tap(edge.a), bt = tap(edge.b), lane = laneStart + i * 8;
      return { edge, ax: at.x, bx: bt.x, ay: a.y, by: b.y,
        d: `M ${at.x} ${a.y} V ${at.departureY} H ${lane} V ${bt.departureY} H ${bt.x} V ${b.y}` };
    });
    return { positions, paths, width: laneStart + model.rendered.length * 8 + 24,
      height: Math.max(pitch, model.nodes.length * pitch), barWidth };
  }

  core.LocalBusDiagram = Object.freeze({ buildGraph, extract, layout, keyOf, LIMITS });
})(typeof window === 'undefined' ? globalThis : window);
