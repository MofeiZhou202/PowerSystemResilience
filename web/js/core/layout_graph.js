/** Semantic graph projection for hybrid AC/DC single-line layout. */
'use strict';

(function initLayoutGraph(global) {
  const core = global.HySimCore = global.HySimCore || {};
  const BUS_TYPES = new Set(['ac_bus', 'dc_bus']);
  const AC_LINKS = new Set(['ac_branch', 'transformer_2w', 'transformer_3w', 'switch_comp', 'circuit_breaker']);
  const DC_LINKS = new Set(['dc_branch', 'dc_circuit_breaker']);
  const COUPLERS = new Set(['vsc_converter', 'dcdc_converter', 'energy_router']);

  function componentSize(type) {
    if (BUS_TYPES.has(type)) return { width: 126, height: 70 };
    if (type === 'energy_router') return { width: 150, height: 130 };
    if (type === 'transformer_3w') return { width: 128, height: 128 };
    if (AC_LINKS.has(type) || DC_LINKS.has(type) || COUPLERS.has(type)) return { width: 120, height: 112 };
    return { width: 116, height: 116 };
  }

  function nodeId(id) { return `component-${id}`; }

  function isFixed(component) {
    const params = component.params || {};
    return component.layoutFixed === true || params.layout_fixed === true ||
      params.position_locked === true || params.fixed_position === true;
  }

  function connectionIndex(components, connections) {
    const byId = new Map(components.map(component => [component.id, component]));
    const neighbors = new Map(components.map(component => [component.id, new Set()]));
    const busNeighbors = new Map(components.map(component => [component.id, new Set()]));
    (connections || []).forEach(connection => {
      const from = byId.get(connection.from?.compId);
      const to = byId.get(connection.to?.compId);
      if (!from || !to) return;
      neighbors.get(from.id).add(to.id);
      neighbors.get(to.id).add(from.id);
      if (BUS_TYPES.has(from.type)) busNeighbors.get(to.id).add(from.id);
      if (BUS_TYPES.has(to.type)) busNeighbors.get(from.id).add(to.id);
    });
    return { byId, neighbors, busNeighbors };
  }

  function componentDomain(component, index) {
    if (component.type === 'ac_bus') return 'ac';
    if (component.type === 'dc_bus') return 'dc';
    if (COUPLERS.has(component.type)) return 'coupling';
    const domains = new Set();
    (index.busNeighbors.get(component.id) || []).forEach(busId => {
      const bus = index.byId.get(busId);
      if (bus?.type === 'ac_bus') domains.add('ac');
      if (bus?.type === 'dc_bus') domains.add('dc');
    });
    if (domains.size === 1) return [...domains][0];
    if (domains.size > 1) return 'coupling';
    if (DC_LINKS.has(component.type) || String(component.type).startsWith('dc_')) return 'dc';
    return 'ac';
  }

  function buildBusAdjacency(components, index, domain) {
    const buses = components.filter(component => component.type === `${domain}_bus`);
    const busSet = new Set(buses.map(bus => bus.id));
    const adjacency = new Map(buses.map(bus => [bus.id, new Set()]));
    const validLinks = domain === 'ac' ? AC_LINKS : DC_LINKS;
    components.forEach(component => {
      if (!validLinks.has(component.type)) return;
      const linked = [...(index.busNeighbors.get(component.id) || [])].filter(id => busSet.has(id));
      // Multi-terminal equipment remains a star centered on the equipment for
      // ELK; this projection only connects its buses for feeder discovery.
      for (let i = 1; i < linked.length; i += 1) {
        adjacency.get(linked[0]).add(linked[i]);
        adjacency.get(linked[i]).add(linked[0]);
      }
    });
    return { buses, busSet, adjacency };
  }

  function findRoots(domainGraph, components, index) {
    const roots = [];
    const rootSet = new Set();
    domainGraph.buses.forEach(bus => {
      const type = String(bus.params?.bus_type || '').toUpperCase();
      if (type === 'SLACK' || type === 'REF') { roots.push(bus.id); rootSet.add(bus.id); }
    });
    components.forEach(component => {
      if (component.type !== 'external_grid') return;
      (index.busNeighbors.get(component.id) || []).forEach(busId => {
        if (domainGraph.busSet.has(busId) && !rootSet.has(busId)) {
          roots.push(busId); rootSet.add(busId);
        }
      });
    });
    return roots;
  }

  function discoverFeeders(domain, components, index) {
    const graph = buildBusAdjacency(components, index, domain);
    const roots = findRoots(graph, components, index);
    const visited = new Set();
    const feeders = [];
    const feederByBus = new Map();

    const seedComponent = start => {
      if (visited.has(start)) return;
      let root = start;
      if (!roots.includes(start)) {
        const component = [];
        const queue = [start];
        const seen = new Set([start]);
        while (queue.length) {
          const current = queue.shift();
          component.push(current);
          (graph.adjacency.get(current) || []).forEach(next => {
            if (!seen.has(next)) { seen.add(next); queue.push(next); }
          });
        }
        root = component.reduce((best, id) =>
          (graph.adjacency.get(id)?.size || 0) > (graph.adjacency.get(best)?.size || 0) ? id : best, component[0]);
      }
      visited.add(root);
      const first = [...(graph.adjacency.get(root) || [])];
      if (!first.length) first.push(root);
      first.forEach((child, branchIndex) => {
        if (child !== root && visited.has(child)) return;
        const id = `${domain}-feeder-${root}-${branchIndex + 1}`;
        const busIds = [];
        const queue = [child];
        if (child === root) visited.add(root);
        else visited.add(child);
        while (queue.length) {
          const current = queue.shift();
          busIds.push(current);
          feederByBus.set(current, id);
          (graph.adjacency.get(current) || []).forEach(next => {
            if (next === root || visited.has(next)) return;
            visited.add(next); queue.push(next);
          });
        }
        feeders.push({ id, domain, root_bus_id: root, bus_ids: busIds, component_ids: [] });
      });
    };

    roots.forEach(seedComponent);
    graph.buses.forEach(bus => seedComponent(bus.id));
    const feederById = new Map(feeders.map(feeder => [feeder.id, feeder]));
    components.forEach(component => {
      const linkedFeeders = new Set([...(index.busNeighbors.get(component.id) || [])]
        .map(busId => feederByBus.get(busId)).filter(Boolean));
      if (linkedFeeders.size !== 1) return;
      feederById.get([...linkedFeeders][0])?.component_ids.push(component.id);
    });
    return { feeders, feederByBus };
  }

  function build(componentsInput = [], connectionsInput = [], options = {}) {
    const components = componentsInput.map(component => ({
      id: component.id,
      type: component.type,
      x: Number(component.x) || 0,
      y: Number(component.y) || 0,
      rotation: Number(component.rotation) || 0,
      params: component.params || {},
      layoutFixed: component.layoutFixed === true,
    }));
    const connections = connectionsInput.map((connection, index) => ({
      id: connection.id ?? index,
      from: { compId: connection.from?.compId, portId: connection.from?.portId },
      to: { compId: connection.to?.compId, portId: connection.to?.portId },
    }));
    const index = connectionIndex(components, connections);
    const ac = discoverFeeders('ac', components, index);
    const dc = discoverFeeders('dc', components, index);
    const domains = { ac: 0, dc: 0, coupling: 0 };
    const layoutDomains = { ac: [], dc: [], coupling: [] };
    const metadata = {};
    const fixedNodeIds = [];
    components.forEach(component => {
      const domain = componentDomain(component, index);
      const size = componentSize(component.type);
      if (BUS_TYPES.has(component.type) && Number(options.busbarHalfMax) > 0) {
        // Conservative rotated footprint: a later auto-span stays inside the
        // ELK node. Same bound as canvas.js::layoutFootprint; see scale-first
        // rationale in docs/planning/gui_one_line_redesign.md.
        const half = Number(options.busbarHalfMax) + 18;
        const angle = component.rotation * Math.PI / 180;
        const cos = Math.abs(Math.cos(angle)), sin = Math.abs(Math.sin(angle));
        size.width = 2 * (half * cos + 34 * sin) + 28;
        size.height = 2 * (half * sin + 34 * cos) + 28;
      }
      const fixed = isFixed(component);
      const busTerminalCount = index.busNeighbors.get(component.id)?.size || 0;
      const skeleton = BUS_TYPES.has(component.type) || COUPLERS.has(component.type) || busTerminalCount > 2;
      const linkedFeeders = new Set([...(index.busNeighbors.get(component.id) || [])]
        .map(busId => ac.feederByBus.get(busId) || dc.feederByBus.get(busId)).filter(Boolean));
      const feederId = ac.feederByBus.get(component.id) || dc.feederByBus.get(component.id) ||
        (linkedFeeders.size === 1 ? [...linkedFeeders][0] : null);
      metadata[nodeId(component.id)] = {
        component_id: component.id,
        type: component.type,
        domain,
        feeder_id: feederId,
        fixed,
        original: { x: component.x, y: component.y },
        ...size,
      };
      if (fixed) fixedNodeIds.push(component.id);
      domains[domain] += 1;
      if (skeleton) {
        layoutDomains[domain].push({
          id: nodeId(component.id),
          ...size,
          x: component.x - size.width / 2,
          y: component.y - size.height / 2,
          layoutOptions: fixed ? { 'org.eclipse.elk.position': `(${component.x},${component.y})` } : {},
        });
      }
    });

    const direction = options.direction === 'LR' ? 'RIGHT' : 'DOWN';
    const domainChildren = [
      ['ac', 'AC domain'], ['dc', 'DC domain'], ['coupling', 'Converters and routers'],
    ].filter(([domain]) => layoutDomains[domain].length).map(([domain, label]) => ({
      id: `domain-${domain}`,
      labels: [{ text: label }],
      children: layoutDomains[domain],
      layoutOptions: {
        'elk.algorithm': 'layered',
        'elk.direction': direction,
        'elk.edgeRouting': 'ORTHOGONAL',
        'elk.spacing.nodeNode': '80',
        'elk.layered.spacing.nodeNodeBetweenLayers': '120',
        'elk.padding': '[top=70,left=50,bottom=50,right=50]',
      },
    }));
    const skeletonIds = new Set(Object.values(layoutDomains).flat().map(node => node.id));
    const edges = [];
    components.forEach(component => {
      const linked = [...(index.busNeighbors.get(component.id) || [])];
      if (skeletonIds.has(nodeId(component.id)) && !BUS_TYPES.has(component.type)) {
        linked.forEach((busId, terminal) => edges.push({
          id: `hyper-${component.id}-${terminal}`,
          sources: [nodeId(component.id)], targets: [nodeId(busId)],
        }));
      } else if ((AC_LINKS.has(component.type) || DC_LINKS.has(component.type)) && linked.length >= 2) {
        for (let terminal = 1; terminal < linked.length; terminal += 1) {
          edges.push({
            id: `branch-${component.id}-${terminal}`,
            sources: [nodeId(linked[0])], targets: [nodeId(linked[terminal])],
          });
        }
      }
    });
    const hyperedges = components.filter(component =>
      component.type === 'energy_router' || (index.busNeighbors.get(component.id)?.size || 0) > 2).map(component => ({
        component_id: component.id,
        type: component.type,
        terminals: [...(index.busNeighbors.get(component.id) || [])],
      }));
    const graph = {
      id: 'hysim-layout-root',
      children: domainChildren,
      edges,
      layoutOptions: {
        'elk.algorithm': 'layered',
        'elk.direction': 'RIGHT',
        'elk.edgeRouting': 'ORTHOGONAL',
        'elk.hierarchyHandling': 'INCLUDE_CHILDREN',
        'elk.separateConnectedComponents': 'true',
        'elk.spacing.componentComponent': '180',
        'elk.spacing.nodeNode': '100',
        'elk.layered.considerModelOrder.strategy': options.incremental ? 'NODES_AND_EDGES' : 'NONE',
        'elk.layered.nodePlacement.strategy': options.incremental ? 'INTERACTIVE' : 'NETWORK_SIMPLEX',
      },
    };
    return {
      schema: 'hysim_layout_graph_v1',
      graph,
      metadata,
      domains,
      hyperedges,
      feeders: [...ac.feeders, ...dc.feeders],
      fixed_node_ids: fixedNodeIds,
      component_count: components.length,
      connection_count: connections.length,
      layout_edge_count: edges.length,
    };
  }

  core.LayoutGraph = Object.freeze({
    schema: 'hysim_layout_graph_v1',
    build,
  });
})(window);
