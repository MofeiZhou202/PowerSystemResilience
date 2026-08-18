/**
 * network_overview.js - WebGL2 full-network overview for large systems.
 * Keeps one stable {domain,index} identity across overview, tables and SVG.
 */
'use strict';

const NetworkOverview = (() => {
  // Viewport-driven fetching: after pan/zoom settles for this long, the
  // viewport is re-evaluated for an LOD switch or a topology_window fetch.
  const WINDOW_DEBOUNCE_MS = 250;
  // Each fetch requests the viewport grown by this fraction on every side, so
  // small pans stay inside the already-fetched window (prefetch margin).
  const WINDOW_PREFETCH_MARGIN = 0.4;
  // Auto-LOD thresholds on the estimated number of visible (geo-referenced)
  // nodes: at most this many for per-bus LOD2, then aggregated LOD1, else LOD0.
  const AUTO_LOD2_MAX_VISIBLE = 2500;
  const AUTO_LOD1_MAX_VISIBLE = 30000;
  // Minimum share of buses with non-default coordinates required before the
  // backend topology_window service drives rendering (mirrors the geographic
  // layout rule in layoutGraph).
  const GEO_COVERAGE_MIN = 0.8;

  const state = {
    active: false,
    system: null,
    full: null,
    graph: null,
    lod: 2,
    // Auto LOD (zoom-driven) is only used in windowed mode; the local
    // fallback keeps the historical manual behavior.
    lodAuto: false,
    selected: null,
    centerX: 0,
    centerY: 0,
    scale: 1,
    dragging: false,
    moved: false,
    start: null,
    gl: null,
    program: null,
    lineBuffer: null,
    transformerBuffer: null,
    nodeBuffer: null,
    lineCount: 0,
    transformerCount: 0,
    nodeCount: 0,
    // SoA render store: preallocated interleaved [x, y, r, g, b, a] vertex
    // data plus parallel identity arrays. Capacity grows x2 so LOD rebuilds
    // and selection updates avoid per-change allocation.
    soa: {
      nodeData: null,
      nodeDomains: [],
      nodeIndices: [],
      nodeKeys: [],
      nodeSlotByKey: new Map(),
      lineData: null,
      lineEndpoints: null,
      // Per-drawn-edge bookkeeping for result recoloring: vertex float offset
      // in lineData, stable identity, and the structural base color to restore
      // when no loading value applies. aggKey (order-independent group pair
      // key) identifies collapsed LOD0/1 edges in the result maps.
      lineEdgeInfo: [],
      transformerData: null,
    },
    colors: null,
    selectedSlot: -1,
    dirtyMin: -1,
    dirtyMax: -1,
    lineDirtyMin: -1,
    lineDirtyMax: -1,
    grid: null,
    // Viewport-driven backend topology_window mode.
    fetchWindow: null,   // injected async (body) => topology_window_v1 | null
    // Injected async (body) => result_window_v1 | {empty:true} (409: no cached
    // PF) | null (transport/other error). Result coloring only exists in
    // windowed mode; the local fallback path is unaffected.
    fetchResultWindow: null,
    result: {
      seq: 0,            // bumped per result request; stale replies dropped
      active: false,     // result coloring currently applied
      // Map<node.key, vm> — per-bus `${domain}:${index}` at LOD2, backend
      // group key at LOD0/1 (value = worst band deviation, else vm_avg).
      nodeVm: null,
      // Map<edge id, loading_pct> — `${domain}:${index}` at LOD2, the
      // order-independent group-pair aggEdgeKey at LOD0/1.
      branchLoading: null,
      meta: null,        // result_meta verbatim (source/method/converged/...)
      limitations: [],
    },
    windowed: false,
    fullBounds: null,    // world-space bounds of the full network
    window: {
      timer: 0,
      seq: 0,            // bumped per request; stale responses are dropped
      bbox: null,        // last fetched expanded world-space bbox
      lod: -1,
      limitations: [],
      coverage: null,
    },
    resizeObserver: null,
  };

  let root;
  let canvas;
  let status;
  let lodSelect;
  let legend;

  const keyOf = (domain, index) => `${domain}:${index}`;
  const finite = value => Number.isFinite(Number(value));

  function buildFullGraph(system) {
    const nodes = [];
    const edges = [];
    const addNodes = (items, domain) => (Array.isArray(items) ? items : []).forEach((bus, position) => {
      const index = Number(bus.index ?? position + 1);
      nodes.push({
        key: keyOf(domain, index), domain, index, position,
        name: bus.name || `${domain.toUpperCase()} ${index}`,
        area: Number(bus.area || 0), zone: Number(bus.zone || 0),
        latitude: Number(bus.latitude || 0), longitude: Number(bus.longitude || 0),
        inService: bus.in_service !== false,
      });
    });
    addNodes(system?.ac?.buses, 'ac');
    addNodes(system?.dc?.buses, 'dc');

    const addEdges = (items, domain, fromField, toField, kind) =>
      (Array.isArray(items) ? items : []).forEach((item, position) => {
        const from = Number(item[fromField]);
        const to = Number(item[toField]);
        if (!finite(from) || !finite(to)) return;
        const edgeKind = typeof kind === 'function' ? kind(item) : kind;
        edges.push({
          key: `${domain}:${edgeKind}:${item.index ?? position + 1}`,
          source: keyOf(domain, from), target: keyOf(domain, to),
          domain, kind: edgeKind, count: 1, inService: item.in_service !== false,
        });
      });
    const acBranches = Array.isArray(system?.ac?.branches) ? system.ac.branches : [];
    const acBranchIndices = new Set(acBranches.map(item => Number(item.index)).filter(finite));
    const linkedTransformerBranches = new Set(
      (Array.isArray(system?.ac?.transformers_2w) ? system.ac.transformers_2w : [])
        .map(item => Number(item.source_branch_idx))
        .filter(index => index > 0 && acBranchIndices.has(index)));
    const acBranchKind = item => {
      const explicit = String(item?.branch_kind || '').trim().toLowerCase();
      if (explicit === 'transformer') return 'transformer';
      if (explicit === 'line') return 'branch';
      const transformerLike =
        linkedTransformerBranches.has(Number(item?.index)) ||
        String(item?.name || '').startsWith('T_') ||
        Number(item?.sn_mva) > 0 || Number(item?.vn_hv_kv) > 0 ||
        Number(item?.vn_lv_kv) > 0 ||
        Math.abs(Number(item?.tap ?? 1) - 1) > 1e-6 ||
        Math.abs(Number(item?.shift_deg ?? 0)) > 1e-6;
      return transformerLike ? 'transformer' : 'branch';
    };
    addEdges(acBranches, 'ac', 'from_bus', 'to_bus', acBranchKind);
    // A Transformer2W with source_branch_idx is metadata for the same canonical
    // branch, not a second physical edge.
    addEdges(
      (Array.isArray(system?.ac?.transformers_2w) ? system.ac.transformers_2w : [])
        .filter(item => !linkedTransformerBranches.has(Number(item.source_branch_idx))),
      'ac', 'hv_bus', 'lv_bus', 'transformer');
    addEdges(system?.ac?.switches, 'ac', 'bus_from', 'bus_to', 'switch');
    addEdges(system?.ac?.circuit_breakers, 'ac', 'bus_from', 'bus_to', 'breaker');
    addEdges(system?.dc?.branches, 'dc', 'from_bus', 'to_bus', 'branch');
    addEdges(system?.dc?.dcdc_converters || system?.dcdc_converters,
      'dc', 'bus_in', 'bus_out', 'dcdc');
    (Array.isArray(system?.vsc_converters) ? system.vsc_converters : []).forEach((item, position) => {
      const ac = Number(item.bus_ac);
      const dc = Number(item.bus_dc);
      if (!finite(ac) || !finite(dc)) return;
      edges.push({
        key: `hybrid:vsc:${item.index ?? position + 1}`,
        source: keyOf('ac', ac), target: keyOf('dc', dc),
        domain: 'hybrid', kind: 'vsc', count: 1, inService: item.in_service !== false,
      });
    });
    const nodeKeys = new Set(nodes.map(node => node.key));
    return { nodes, edges: edges.filter(edge => nodeKeys.has(edge.source) && nodeKeys.has(edge.target)) };
  }

  function layoutGraph(graph) {
    const byKey = new Map(graph.nodes.map(node => [node.key, node]));
    const geographic = graph.nodes.length > 1 &&
      graph.nodes.filter(node => node.latitude !== 0 || node.longitude !== 0).length >= graph.nodes.length * 0.8;
    if (geographic) {
      graph.nodes.forEach(node => {
        node.x = node.longitude * 100;
        node.y = -node.latitude * 100;
      });
      return graph;
    }

    const adjacency = new Map(graph.nodes.map(node => [node.key, []]));
    graph.edges.forEach(edge => {
      adjacency.get(edge.source)?.push(edge.target);
      adjacency.get(edge.target)?.push(edge.source);
    });
    const unseen = new Set(graph.nodes.map(node => node.key));
    let componentOffsetY = 0;
    while (unseen.size) {
      let seed = unseen.values().next().value;
      const component = [];
      const discover = [seed];
      unseen.delete(seed);
      for (let i = 0; i < discover.length; i += 1) {
        const key = discover[i];
        component.push(key);
        (adjacency.get(key) || []).forEach(next => {
          if (unseen.delete(next)) discover.push(next);
        });
      }
      seed = component.reduce((best, key) =>
        (adjacency.get(key)?.length || 0) > (adjacency.get(best)?.length || 0) ? key : best, component[0]);
      const depth = new Map([[seed, 0]]);
      const queue = [seed];
      for (let i = 0; i < queue.length; i += 1) {
        const key = queue[i];
        (adjacency.get(key) || []).forEach(next => {
          if (!depth.has(next)) {
            depth.set(next, depth.get(key) + 1);
            queue.push(next);
          }
        });
      }
      const layers = new Map();
      component.forEach(key => {
        const d = depth.get(key) || 0;
        if (!layers.has(d)) layers.set(d, []);
        layers.get(d).push(key);
      });
      let componentHeight = 0;
      layers.forEach(keys => { componentHeight = Math.max(componentHeight, keys.length * 30); });
      const maxDepth = Math.max(...layers.keys());
      const layerSpacing = Math.max(64, componentHeight * 1.35 / Math.max(1, maxDepth));
      layers.forEach((keys, d) => {
        keys.sort((a, b) => (byKey.get(a)?.index || 0) - (byKey.get(b)?.index || 0));
        const y0 = componentOffsetY + (componentHeight - (keys.length - 1) * 30) / 2;
        keys.forEach((key, i) => {
          const node = byKey.get(key);
          node.x = d * layerSpacing;
          node.y = y0 + i * 30;
        });
      });
      componentOffsetY += componentHeight + 80;
    }
    return graph;
  }

  function aggregate(graph, lod) {
    if (lod >= 2) return graph;
    const groups = new Map();
    const memberGroup = new Map();
    graph.nodes.forEach(node => {
      const key = lod === 0 ? node.domain : `${node.domain}:area:${node.area}:zone:${node.zone}`;
      memberGroup.set(node.key, key);
      if (!groups.has(key)) groups.set(key, {
        key, domain: node.domain, area: node.area, zone: node.zone,
        name: lod === 0 ? `${node.domain.toUpperCase()} 网络` :
          `${node.domain.toUpperCase()} 区域 ${node.area}/${node.zone}`,
        x: 0, y: 0, count: 0, inService: true,
      });
      const group = groups.get(key);
      group.x += node.x;
      group.y += node.y;
      group.count += 1;
    });
    groups.forEach(group => { group.x /= group.count; group.y /= group.count; });
    const edgeGroups = new Map();
    graph.edges.forEach(edge => {
      let source = memberGroup.get(edge.source);
      let target = memberGroup.get(edge.target);
      if (!source || !target || source === target) return;
      if (target < source) [source, target] = [target, source];
      const key = `${source}->${target}`;
      if (!edgeGroups.has(key)) edgeGroups.set(key, {
        key, source, target, domain: edge.domain, kind: 'aggregate', count: 0, inService: true,
      });
      edgeGroups.get(key).count += 1;
    });
    return { nodes: [...groups.values()], edges: [...edgeGroups.values()] };
  }

  function shader(type, source) {
    const gl = state.gl;
    const value = gl.createShader(type);
    gl.shaderSource(value, source);
    gl.compileShader(value);
    if (!gl.getShaderParameter(value, gl.COMPILE_STATUS)) {
      throw new Error(gl.getShaderInfoLog(value) || 'WebGL shader compilation failed');
    }
    return value;
  }

  function initGl() {
    // Preserve the latest frame so screenshots and pixel-based regression checks
    // can inspect the operational overview after browser compositing.
    state.gl = canvas.getContext('webgl2', {
      antialias: true,
      alpha: false,
      preserveDrawingBuffer: true,
    });
    if (!state.gl) return false;
    const gl = state.gl;
    const vertex = shader(gl.VERTEX_SHADER, `#version 300 es
      in vec2 a_position;
      in vec4 a_color;
      uniform vec2 u_center;
      uniform vec2 u_viewport;
      uniform float u_scale;
      uniform float u_point_size;
      out vec4 v_color;
      void main() {
        vec2 pixel = (a_position - u_center) * u_scale;
        gl_Position = vec4(pixel.x / (u_viewport.x * 0.5), -pixel.y / (u_viewport.y * 0.5), 0.0, 1.0);
        gl_PointSize = u_point_size;
        v_color = a_color;
      }`);
    const fragment = shader(gl.FRAGMENT_SHADER, `#version 300 es
      precision mediump float;
      in vec4 v_color;
      uniform bool u_round;
      out vec4 out_color;
      void main() {
        if (u_round && distance(gl_PointCoord, vec2(0.5)) > 0.5) discard;
        out_color = v_color;
      }`);
    state.program = gl.createProgram();
    gl.attachShader(state.program, vertex);
    gl.attachShader(state.program, fragment);
    gl.linkProgram(state.program);
    if (!gl.getProgramParameter(state.program, gl.LINK_STATUS)) return false;
    state.lineBuffer = gl.createBuffer();
    state.transformerBuffer = gl.createBuffer();
    state.nodeBuffer = gl.createBuffer();
    return true;
  }

  function cssColor(name, fallback) {
    const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
    const probe = document.createElement('span');
    probe.style.color = value;
    probe.style.display = 'none';
    document.body.appendChild(probe);
    const rgb = getComputedStyle(probe).color.match(/[\d.]+/g)?.map(Number) || [128, 128, 128, 1];
    probe.remove();
    return [rgb[0] / 255, rgb[1] / 255, rgb[2] / 255, rgb.length > 3 ? rgb[3] : 1];
  }

  function palette() {
    return {
      ac: cssColor('--accent', '#61afef'),
      dc: cssColor('--accent2', '#56b6c2'),
      hybrid: cssColor('--purple', '#c678dd'),
      edge: cssColor('--ink3', '#657086'),
      transformer: cssColor('--purple', '#c678dd'),
      selected: cssColor('--yellow', '#e5c07b'),
      background: cssColor('--canvas-bg', '#10141d'),
    };
  }

  function ensureCapacity(current, needed, Type) {
    if (current && current.length >= needed) return current;
    let capacity = current ? current.length : 256;
    while (capacity < needed) capacity *= 2;
    const next = new Type(capacity);
    if (current) next.set(current);
    return next;
  }

  // Structural rebuild (new system or LOD switch): fills the preallocated
  // interleaved vertex arrays from the current graph, uploads the used prefix
  // of each GL buffer, and rebuilds the uniform grid. Selection and other
  // color-only changes never reach this path; they use markNodeDirty instead.
  function rebuildBuffers() {
    if (!state.gl || !state.graph) return;
    const gl = state.gl;
    const colors = palette();
    colors.edge[3] = 0.22;
    colors.hybrid[3] = 0.7;
    colors.transformer[3] = 0.85;
    state.colors = colors;

    const nodes = state.graph.nodes;
    const nodeCount = nodes.length;
    const nodeData = ensureCapacity(state.soa.nodeData, nodeCount * 6, Float32Array);
    const slotByKey = state.soa.nodeSlotByKey;
    slotByKey.clear();
    state.soa.nodeDomains.length = nodeCount;
    state.soa.nodeIndices.length = nodeCount;
    state.soa.nodeKeys.length = nodeCount;
    state.selectedSlot = -1;
    for (let i = 0; i < nodeCount; i += 1) {
      const node = nodes[i];
      const offset = i * 6;
      const selected = state.selected && node.domain === state.selected.domain &&
        Number(node.index) === Number(state.selected.index);
      state.soa.nodeDomains[i] = node.domain;
      state.soa.nodeIndices[i] = node.index;
      state.soa.nodeKeys[i] = node.key;
      slotByKey.set(node.key, i);
      if (selected) state.selectedSlot = i;
      // Result coloring (when active) is the base color; selection wins.
      const color = selected ? colors.selected : nodeBaseColor(i);
      nodeData[offset] = node.x;
      nodeData[offset + 1] = node.y;
      nodeData[offset + 2] = color[0];
      nodeData[offset + 3] = color[1];
      nodeData[offset + 4] = color[2];
      nodeData[offset + 5] = color[3];
    }

    // Line endpoints reference node slots, so edge identity survives
    // color-only updates; only structural rebuilds rewrite these arrays.
    const edges = state.graph.edges;
    const lineData = ensureCapacity(state.soa.lineData, edges.length * 12, Float32Array);
    const lineEndpoints = ensureCapacity(state.soa.lineEndpoints, edges.length * 2, Int32Array);
    const transformerData = ensureCapacity(state.soa.transformerData, edges.length * 6, Float32Array);
    let lineFloats = 0;
    let linePairs = 0;
    let transformerFloats = 0;
    const lineEdgeInfo = state.soa.lineEdgeInfo;
    lineEdgeInfo.length = 0;
    edges.forEach(edge => {
      const source = slotByKey.get(edge.source);
      const target = slotByKey.get(edge.target);
      if (source == null || target == null) return;
      const baseColor = edge.kind === 'transformer'
        ? colors.transformer
        : (edge.domain === 'hybrid' ? colors.hybrid : colors.edge);
      const info = {
        offset: lineFloats,
        domain: edge.domain,
        index: edge.index != null ? Number(edge.index) : null,
        aggKey: edge.kind === 'aggregate'
          ? aggEdgeKey(String(edge.source), String(edge.target))
          : null,
        base: baseColor,
      };
      lineEdgeInfo.push(info);
      const color = edgeBaseColor(info);
      [source, target].forEach(slot => {
        const from = slot * 6;
        lineData[lineFloats] = nodeData[from];
        lineData[lineFloats + 1] = nodeData[from + 1];
        lineData[lineFloats + 2] = color[0];
        lineData[lineFloats + 3] = color[1];
        lineData[lineFloats + 4] = color[2];
        lineData[lineFloats + 5] = color[3];
        lineFloats += 6;
      });
      lineEndpoints[linePairs] = source;
      lineEndpoints[linePairs + 1] = target;
      linePairs += 2;
      if (edge.kind === 'transformer') {
        transformerData[transformerFloats] = (nodeData[source * 6] + nodeData[target * 6]) / 2;
        transformerData[transformerFloats + 1] = (nodeData[source * 6 + 1] + nodeData[target * 6 + 1]) / 2;
        transformerData[transformerFloats + 2] = colors.transformer[0];
        transformerData[transformerFloats + 3] = colors.transformer[1];
        transformerData[transformerFloats + 4] = colors.transformer[2];
        transformerData[transformerFloats + 5] = colors.transformer[3];
        transformerFloats += 6;
      }
    });

    state.soa.nodeData = nodeData;
    state.soa.lineData = lineData;
    state.soa.lineEndpoints = lineEndpoints;
    state.soa.transformerData = transformerData;
    state.lineCount = lineFloats / 6;
    state.transformerCount = transformerFloats / 6;
    state.nodeCount = nodeCount;
    state.dirtyMin = -1;
    state.dirtyMax = -1;
    state.lineDirtyMin = -1;
    state.lineDirtyMax = -1;

    // DYNAMIC_DRAW: node colors are patched in place via bufferSubData on
    // selection changes; bufferData reallocates only on structural rebuilds.
    gl.bindBuffer(gl.ARRAY_BUFFER, state.lineBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, lineData.subarray(0, lineFloats), gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, state.transformerBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, transformerData.subarray(0, transformerFloats), gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, state.nodeBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, nodeData.subarray(0, nodeCount * 6), gl.DYNAMIC_DRAW);

    buildGrid();
  }

  // Uniform grid over node positions, rebuilt together with the buffers.
  // Cell size starts at 4x the average node spacing estimated from the
  // bounding-box area, then each axis is capped at 256 cells so a single
  // distant outlier cannot shrink cells below usefulness.
  function buildGrid() {
    const data = state.soa.nodeData;
    const count = state.nodeCount;
    if (!data || !count) {
      state.grid = null;
      return;
    }
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (let i = 0; i < count; i += 1) {
      const x = data[i * 6];
      const y = data[i * 6 + 1];
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
    const width = Math.max(maxX - minX, 1e-6);
    const height = Math.max(maxY - minY, 1e-6);
    const spacing = Math.sqrt((width * height) / count);
    const cols = Math.min(256, Math.max(1, Math.ceil(width / (spacing * 4))));
    const rows = Math.min(256, Math.max(1, Math.ceil(height / (spacing * 4))));
    const cellW = width / cols;
    const cellH = height / rows;
    const cells = new Map();
    for (let i = 0; i < count; i += 1) {
      const cx = Math.min(cols - 1, Math.floor((data[i * 6] - minX) / cellW));
      const cy = Math.min(rows - 1, Math.floor((data[i * 6 + 1] - minY) / cellH));
      const key = cy * cols + cx;
      let bucket = cells.get(key);
      if (!bucket) {
        bucket = [];
        cells.set(key, bucket);
      }
      bucket.push(i);
    }
    state.grid = { minX, minY, cellW, cellH, cols, rows, cells };
  }

  // Color-only update path: rewrites the rgba slots of the affected nodes and
  // uploads the merged dirty range [dirtyMin, dirtyMax] with bufferSubData.
  function markNodeDirty(slot, color) {
    const data = state.soa.nodeData;
    const offset = slot * 6 + 2;
    data[offset] = color[0];
    data[offset + 1] = color[1];
    data[offset + 2] = color[2];
    data[offset + 3] = color[3];
    if (state.dirtyMin < 0 || slot < state.dirtyMin) state.dirtyMin = slot;
    if (slot > state.dirtyMax) state.dirtyMax = slot;
  }

  function writeNodeColorIfChanged(slot, color) {
    const data = state.soa.nodeData;
    const offset = slot * 6 + 2;
    if (data[offset] === color[0] && data[offset + 1] === color[1] &&
        data[offset + 2] === color[2] && data[offset + 3] === color[3]) return;
    markNodeDirty(slot, color);
  }

  // Same dirty-range mechanism for the line buffer, tracked in float offsets
  // (vertex-aligned, 6 floats per vertex, 2 vertices per edge).
  function markLineDirty(floatOffset) {
    if (state.lineDirtyMin < 0 || floatOffset < state.lineDirtyMin) state.lineDirtyMin = floatOffset;
    if (floatOffset > state.lineDirtyMax) state.lineDirtyMax = floatOffset;
  }

  function writeEdgeColorIfChanged(floatOffset, color) {
    const data = state.soa.lineData;
    if (!data) return;
    for (let v = 0; v < 2; v += 1) {
      const base = floatOffset + v * 6;
      if (data[base + 2] === color[0] && data[base + 3] === color[1] &&
          data[base + 4] === color[2] && data[base + 5] === color[3]) continue;
      data[base + 2] = color[0];
      data[base + 3] = color[1];
      data[base + 4] = color[2];
      data[base + 5] = color[3];
      markLineDirty(base);
    }
  }

  function flushDirtyRanges() {
    const gl = state.gl;
    if (state.dirtyMin >= 0 && state.dirtyMax >= state.dirtyMin) {
      const start = state.dirtyMin * 6;
      const end = (state.dirtyMax + 1) * 6;
      gl.bindBuffer(gl.ARRAY_BUFFER, state.nodeBuffer);
      gl.bufferSubData(gl.ARRAY_BUFFER, start * 4, state.soa.nodeData.subarray(start, end));
      state.dirtyMin = -1;
      state.dirtyMax = -1;
    }
    if (state.lineDirtyMin >= 0 && state.lineDirtyMax >= state.lineDirtyMin) {
      const start = state.lineDirtyMin;
      const end = state.lineDirtyMax + 6;
      gl.bindBuffer(gl.ARRAY_BUFFER, state.lineBuffer);
      gl.bufferSubData(gl.ARRAY_BUFFER, start * 4, state.soa.lineData.subarray(start, end));
      state.lineDirtyMin = -1;
      state.lineDirtyMax = -1;
    }
  }

  // Result color scales — the single source of truth shared by the WebGL
  // coloring and the legend overlay. Voltage band colors mirror the SVG
  // result overlay (canvas.js showPowerFlowResults): <0.95 pu red #e06c75,
  // >1.05 pu orange #d19a66, in-band green #98c379 for AC and cyan #56b6c2
  // for DC. The loading ramp mirrors canvas.js loadingColor(): green →
  // yellow at 50% → red at 100%, clamped at 150%.
  const VM_BAND_LOW = 0.95;
  const VM_BAND_HIGH = 1.05;
  const VM_COLOR_LOW = [0.878, 0.424, 0.459, 1];    // #e06c75
  const VM_COLOR_HIGH = [0.820, 0.604, 0.400, 1];   // #d19a66
  const VM_COLOR_IN_AC = [0.596, 0.765, 0.475, 1];  // #98c379
  const VM_COLOR_IN_DC = [0.337, 0.714, 0.761, 1];  // #56b6c2
  const LOADING_CLAMP_PCT = 150;
  const LOADING_RAMP = [
    { pct: 0, rgb: [76, 175, 80] },     // green
    { pct: 50, rgb: [255, 235, 59] },   // yellow
    { pct: 100, rgb: [244, 67, 54] },   // red (held up to the clamp)
  ];

  function voltageColor(vm, domain) {
    if (vm < VM_BAND_LOW) return VM_COLOR_LOW;
    if (vm > VM_BAND_HIGH) return VM_COLOR_HIGH;
    return domain === 'dc' ? VM_COLOR_IN_DC : VM_COLOR_IN_AC;
  }

  // Alpha 0.9 so loaded lines stand out over the 0.22-alpha structural edge
  // color.
  function loadingColorFloat(pct) {
    const p = Math.max(0, Math.min(Number(pct) || 0, LOADING_CLAMP_PCT));
    const [a, b] = p <= LOADING_RAMP[1].pct
      ? [LOADING_RAMP[0], LOADING_RAMP[1]]
      : [LOADING_RAMP[1], LOADING_RAMP[2]];
    const t = Math.min((p - a.pct) / (b.pct - a.pct), 1);
    return [
      (a.rgb[0] + (b.rgb[0] - a.rgb[0]) * t) / 255,
      (a.rgb[1] + (b.rgb[1] - a.rgb[1]) * t) / 255,
      (a.rgb[2] + (b.rgb[2] - a.rgb[2]) * t) / 255,
      0.9,
    ];
  }

  // Order-independent key for an aggregate edge between two group keys; the
  // backend orders group pairs by first-encounter order, which differs
  // between the topology and result windows, so both sides normalize.
  const aggEdgeKey = (a, b) => (a < b ? `${a}->${b}` : `${b}->${a}`);

  const rgbCss = rgb =>
    `#${rgb.map(v => Math.max(0, Math.min(255, Math.round(v))).toString(16).padStart(2, '0')).join('')}`;
  const floatCss = color => rgbCss(color.slice(0, 3).map(v => v * 255));

  // Result-color legend overlay: shown only while result coloring is active.
  // Created lazily with inline styles so no stylesheet/HTML change is needed;
  // all colors come from the shared scales above.
  function ensureLegend() {
    if (legend || !root) return legend;
    legend = document.createElement('div');
    legend.setAttribute('aria-label', '结果着色图例');
    const s = legend.style;
    s.position = 'absolute';
    s.right = '10px';
    s.bottom = '10px';
    s.zIndex = '2';
    s.padding = '6px 8px';
    s.borderRadius = '4px';
    s.fontSize = '11px';
    s.lineHeight = '1.6';
    s.background = 'rgba(16, 20, 29, 0.88)';
    s.border = '1px solid rgba(101, 112, 134, 0.6)';
    s.color = '#c8d0e0';
    s.pointerEvents = 'none';
    s.display = 'none';
    const swatch = color =>
      `<span style="display:inline-block;width:10px;height:10px;border-radius:2px;` +
      `background:${floatCss(color)};margin-right:4px;vertical-align:-1px;"></span>`;
    const stops = LOADING_RAMP.map(stop =>
      `${rgbCss(stop.rgb)} ${(stop.pct / LOADING_CLAMP_PCT) * 100}%`).join(', ');
    legend.innerHTML =
      '<div style="font-weight:600;">电压 (pu)</div>' +
      `<div>${swatch(VM_COLOR_LOW)}&lt;${VM_BAND_LOW}　${swatch(VM_COLOR_HIGH)}&gt;${VM_BAND_HIGH}</div>` +
      `<div>${swatch(VM_COLOR_IN_AC)}AC ${VM_BAND_LOW}–${VM_BAND_HIGH}　` +
      `${swatch(VM_COLOR_IN_DC)}DC ${VM_BAND_LOW}–${VM_BAND_HIGH}</div>` +
      '<div style="font-weight:600;margin-top:2px;">负载率</div>' +
      `<div style="width:140px;height:8px;border-radius:2px;background:linear-gradient(90deg, ${stops});"></div>` +
      `<div style="display:flex;justify-content:space-between;width:140px;">` +
      `<span>0%</span><span>${LOADING_CLAMP_PCT}%+</span></div>`;
    root.appendChild(legend);
    return legend;
  }

  function updateLegend() {
    if (!ensureLegend()) return;
    legend.style.display = state.result.active ? 'block' : 'none';
  }

  // Base color of a node slot: result voltage color when result coloring is
  // active and the slot's identity (per-bus key at LOD2, group key at LOD0/1)
  // has a vm value, otherwise the structural domain color.
  function nodeBaseColor(slot) {
    const domain = state.soa.nodeDomains[slot];
    const vm = state.result.active && state.result.nodeVm
      ? state.result.nodeVm.get(state.soa.nodeKeys[slot])
      : undefined;
    if (vm != null && Number.isFinite(vm)) return voltageColor(vm, domain);
    return state.colors[domain] || state.colors.ac;
  }

  // Base color of a drawn edge: loading ramp when result coloring has a value
  // for this stable edge identity (per-branch key at LOD2, group-pair aggKey
  // at LOD0/1), otherwise the structural color recorded at rebuild time.
  function edgeBaseColor(info) {
    if (state.result.active && state.result.branchLoading) {
      const loading = info.aggKey != null
        ? state.result.branchLoading.get(info.aggKey)
        : (info.index != null
          ? state.result.branchLoading.get(`${info.domain}:${info.index}`)
          : undefined);
      if (loading != null && Number.isFinite(loading)) return loadingColorFloat(loading);
    }
    return info.base;
  }

  // Bulk pass after a result_window payload arrives or is cleared. The
  // selected slot keeps its selection color (restored to the result color on
  // deselection via nodeBaseColor in applySelection).
  function applyResultColors() {
    if (!state.soa.nodeData || !state.colors) return;
    for (let i = 0; i < state.nodeCount; i += 1) {
      if (i === state.selectedSlot) continue;
      writeNodeColorIfChanged(i, nodeBaseColor(i));
    }
    state.soa.lineEdgeInfo.forEach(info => {
      writeEdgeColorIfChanged(info.offset, edgeBaseColor(info));
    });
    flushDirtyRanges();
  }

  function applySelection() {
    const colors = state.colors;
    if (!colors || !state.soa.nodeData) return;
    const previous = state.selectedSlot;
    const next = state.selected
      ? state.soa.nodeSlotByKey.get(keyOf(state.selected.domain, state.selected.index))
      : undefined;
    if (previous >= 0 && previous !== next) {
      markNodeDirty(previous, nodeBaseColor(previous));
    }
    if (next != null && next !== previous) markNodeDirty(next, colors.selected);
    state.selectedSlot = next != null ? next : -1;
    flushDirtyRanges();
  }

  function boundsOfNodes(nodes) {
    if (!nodes?.length) return null;
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    nodes.forEach(node => {
      if (node.x < minX) minX = node.x;
      if (node.x > maxX) maxX = node.x;
      if (node.y < minY) minY = node.y;
      if (node.y > maxY) maxY = node.y;
    });
    return { minX, minY, maxX, maxY };
  }

  // Share of buses carrying real (non-default 0,0) coordinates; the backend
  // window service can only return geo-referenced buses, so below the
  // threshold the overview stays on the local full-system path.
  function geoCoverage(nodes) {
    if (!nodes?.length) return 0;
    const withCoords = nodes.filter(node => node.latitude !== 0 || node.longitude !== 0).length;
    return withCoords / nodes.length;
  }

  // Zoom-driven LOD from the estimated number of visible nodes, using the
  // full-network density over its bounding box. Deterministic and cheap; the
  // manual LOD select overrides it (state.lodAuto = false).
  function autoLod() {
    if (!state.fullBounds || !state.full?.nodes.length || !canvas) return state.lod;
    const spanX = Math.max(state.fullBounds.maxX - state.fullBounds.minX, 1e-6);
    const spanY = Math.max(state.fullBounds.maxY - state.fullBounds.minY, 1e-6);
    const density = state.full.nodes.length / (spanX * spanY);
    const rect = canvas.getBoundingClientRect();
    const visible = density *
      (rect.width / Math.max(state.scale, 1e-6)) * (rect.height / Math.max(state.scale, 1e-6));
    if (visible <= AUTO_LOD2_MAX_VISIBLE) return 2;
    if (visible <= AUTO_LOD1_MAX_VISIBLE) return 1;
    return 0;
  }

  function viewportWorldBounds() {
    const rect = canvas.getBoundingClientRect();
    const halfW = rect.width / 2 / Math.max(state.scale, 1e-6);
    const halfH = rect.height / 2 / Math.max(state.scale, 1e-6);
    return {
      minX: state.centerX - halfW, maxX: state.centerX + halfW,
      minY: state.centerY - halfH, maxY: state.centerY + halfH,
    };
  }

  function scheduleViewportUpdate(delay = WINDOW_DEBOUNCE_MS) {
    if (state.window.timer) clearTimeout(state.window.timer);
    state.window.timer = setTimeout(() => {
      state.window.timer = 0;
      onViewportSettled();
    }, delay);
  }

  function onViewportSettled() {
    if (!state.active || !state.windowed) return;
    if (state.lodAuto) {
      const next = autoLod();
      if (next !== state.lod) {
        state.lod = next;
        state.window.bbox = null;
      }
    }
    fetchWindowIfNeeded();
  }

  // Viewport grown by the prefetch margin, in world coordinates.
  function expandedViewportBounds() {
    const view = viewportWorldBounds();
    const marginX = (view.maxX - view.minX) * WINDOW_PREFETCH_MARGIN;
    const marginY = (view.maxY - view.minY) * WINDOW_PREFETCH_MARGIN;
    return {
      minX: view.minX - marginX, maxX: view.maxX + marginX,
      minY: view.minY - marginY, maxY: view.maxY + marginY,
    };
  }

  // World y grows downward (y = -latitude * 100), x = longitude * 100.
  function windowBody(expanded) {
    return {
      min_x: expanded.minX / 100,
      max_x: expanded.maxX / 100,
      min_y: -expanded.maxY / 100,
      max_y: -expanded.minY / 100,
    };
  }

  // Fetch the current viewport (grown by the prefetch margin) unless it is
  // already covered by the previously fetched window at the same LOD. The
  // response sequence guard drops replies that arrive after a newer request.
  function fetchWindowIfNeeded() {
    if (!state.windowed || typeof state.fetchWindow !== 'function') return;
    const view = viewportWorldBounds();
    const cached = state.window.bbox;
    if (cached && state.window.lod === state.lod &&
        cached.minX <= view.minX && cached.maxX >= view.maxX &&
        cached.minY <= view.minY && cached.maxY >= view.maxY) {
      updateStatus();
      return;
    }
    const expanded = expandedViewportBounds();
    const body = { ...windowBody(expanded), lod: state.lod };
    const seq = ++state.window.seq;
    Promise.resolve(state.fetchWindow(body)).then(payload => {
      if (seq !== state.window.seq || !state.active || !state.windowed) return;
      if (!payload || payload.error || !Array.isArray(payload.nodes)) {
        if (!state.graph) {
          // First fetch failed: degrade to the local full-system path rather
          // than showing an empty canvas.
          state.windowed = false;
          state.lodAuto = false;
          applyLocalLod(state.full.nodes.length > 12000 ? 1 : 2);
        } else {
          updateStatus('拓扑窗口数据不可用，保留当前视图');
        }
        return;
      }
      state.window.bbox = expanded;
      state.window.lod = state.lod;
      state.window.limitations = Array.isArray(payload.model_limitations)
        ? payload.model_limitations.map(String) : [];
      state.window.coverage = payload.coordinate_coverage || null;
      state.graph = adaptWindowGraph(payload);
      rebuildBuffers();
      draw();
      updateStatus();
    }).catch(() => {
      if (seq === state.window.seq) updateStatus('拓扑窗口数据不可用，保留当前视图');
    });
    // Result coloring follows the same window: same bbox, same LOD (the
    // backend aggregates group voltage statistics and max branch loading).
    fetchResultWindow({ ...windowBody(expanded), lod: state.lod });
  }

  // result_window fetch with its own sequence guard so it never invalidates a
  // pending topology fetch. The injected fetcher normalizes transport: payload
  // on success, {empty:true} on 409 no_cached_power_flow, null otherwise.
  function fetchResultWindow(body) {
    if (!state.windowed || typeof state.fetchResultWindow !== 'function') return;
    const seq = ++state.result.seq;
    Promise.resolve(state.fetchResultWindow(body)).then(outcome => {
      if (seq !== state.result.seq || !state.active || !state.windowed) return;
      if (!outcome) return;  // transport/other error: keep current coloring
      if (outcome.empty) {
        clearResultColors();
        return;
      }
      if (outcome.error || !Array.isArray(outcome.nodes)) return;
      state.result.nodeVm = new Map();
      state.result.branchLoading = new Map();
      outcome.nodes.forEach(node => {
        if (node.group != null) {
          // Aggregated group node (LOD0/1): color by the worst band
          // deviation among members when any member leaves the
          // [VM_BAND_LOW, VM_BAND_HIGH] band, otherwise by the group mean;
          // the backend declares the statistics semantics in
          // model_limitations, surfaced in the status line.
          const vmin = Number(node.vm_min);
          const vmax = Number(node.vm_max);
          const vavg = Number(node.vm_avg);
          const lowDev = Number.isFinite(vmin) ? VM_BAND_LOW - vmin : 0;
          const highDev = Number.isFinite(vmax) ? vmax - VM_BAND_HIGH : 0;
          const vm = Math.max(lowDev, highDev) > 0
            ? (lowDev >= highDev ? vmin : vmax)
            : vavg;
          if (Number.isFinite(vm)) state.result.nodeVm.set(String(node.group), vm);
          return;
        }
        const vm = Number(node.vm_pu);
        if (Number.isFinite(vm)) {
          state.result.nodeVm.set(
            keyOf(String(node.domain || '').toLowerCase(), Number(node.index)), vm);
        }
      });
      (Array.isArray(outcome.branches) ? outcome.branches : []).forEach(branch => {
        const loading = Number(branch.loading_pct);
        if (!Number.isFinite(loading)) return;
        if (branch.kind === 'aggregate' && branch.source != null && branch.target != null) {
          state.result.branchLoading.set(
            aggEdgeKey(String(branch.source), String(branch.target)), loading);
          return;
        }
        state.result.branchLoading.set(
          `${String(branch.domain || '').toLowerCase()}:${Number(branch.index)}`, loading);
      });
      state.result.meta = outcome.result_meta || null;
      state.result.limitations = Array.isArray(outcome.model_limitations)
        ? outcome.model_limitations.map(String) : [];
      state.result.active = true;
      applyResultColors();
      draw();
      updateStatus();
    }).catch(() => {});
  }

  function clearResultColors() {
    const had = state.result.active;
    state.result.active = false;
    state.result.nodeVm = null;
    state.result.branchLoading = null;
    state.result.meta = null;
    state.result.limitations = [];
    if (had) {
      applyResultColors();
      draw();
    }
    updateStatus();
  }

  // Exported hook for "a new PF result is cached on the backend": refetch the
  // result layer for the current window. No-op outside windowed mode.
  function refreshResults() {
    if (!state.active || !state.windowed ||
        typeof state.fetchResultWindow !== 'function') return false;
    fetchResultWindow({ ...windowBody(expandedViewportBounds()), lod: state.lod });
    return true;
  }

  // topology_window_v1 -> internal graph shape. Backend coordinates are WGS84
  // degrees; the overview world uses x = longitude * 100, y = -latitude * 100
  // (same scaling as the geographic branch of layoutGraph). Node identity
  // stays {domain, index} at LOD2 and the backend group key at LOD0/1.
  function adaptWindowGraph(payload) {
    const lod = Number(payload.lod);
    const nodes = [];
    const edges = [];
    (Array.isArray(payload.nodes) ? payload.nodes : []).forEach(node => {
      const domain = String(node.domain || '').toLowerCase();
      const x = Number(node.x) * 100;
      const y = -Number(node.y) * 100;
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      if (lod === 2) {
        const index = Number(node.index);
        nodes.push({
          key: keyOf(domain, index), domain, index, x, y,
          name: `${domain.toUpperCase()} ${index}`,
          inService: node.in_service !== false,
        });
      } else {
        nodes.push({
          key: String(node.key), domain, index: undefined, x, y,
          name: String(node.name || node.key),
          count: Number(node.count) || 0, inService: true,
        });
      }
    });
    (Array.isArray(payload.edges) ? payload.edges : []).forEach(edge => {
      if (lod === 2) {
        const fromDomain = String(edge.from_domain || '').toLowerCase();
        const toDomain = String(edge.to_domain || '').toLowerCase();
        edges.push({
          source: keyOf(fromDomain, Number(edge.from)),
          target: keyOf(toDomain, Number(edge.to)),
          domain: fromDomain !== toDomain ? 'hybrid' : fromDomain,
          kind: edge.category === 'AC_Transformer' ? 'transformer' : 'branch',
          index: Number(edge.index),
        });
      } else {
        edges.push({
          source: String(edge.source), target: String(edge.target),
          domain: String(edge.domain || '').toLowerCase(), kind: 'aggregate',
        });
      }
    });
    return { nodes, edges };
  }

  // Local (non-windowed) LOD application: the historical full-system path.
  function applyLocalLod(lod) {
    state.lod = Math.max(0, Math.min(2, lod));
    state.graph = aggregate(state.full, state.lod);
    rebuildBuffers();
    fit();
    updateStatus();
  }

  function resize() {
    if (!canvas || !state.gl) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width * dpr));
    const height = Math.max(1, Math.round(rect.height * dpr));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    draw();
  }

  function drawBuffer(buffer, mode, count, pointSize, round) {
    const gl = state.gl;
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    const position = gl.getAttribLocation(state.program, 'a_position');
    const color = gl.getAttribLocation(state.program, 'a_color');
    gl.enableVertexAttribArray(position);
    gl.vertexAttribPointer(position, 2, gl.FLOAT, false, 24, 0);
    gl.enableVertexAttribArray(color);
    gl.vertexAttribPointer(color, 4, gl.FLOAT, false, 24, 8);
    gl.uniform1f(gl.getUniformLocation(state.program, 'u_point_size'), pointSize);
    gl.uniform1i(gl.getUniformLocation(state.program, 'u_round'), round ? 1 : 0);
    gl.drawArrays(mode, 0, count);
  }

  function draw() {
    if (!state.active || !state.gl || !state.graph) return;
    // Full drawArrays for every primitive. Vertex data is uploaded once per
    // structural rebuild, so the per-frame cost of off-screen points/lines is
    // small next to pan/zoom; at the targeted system sizes (tens of thousands
    // of buses) culling is not the bottleneck — picking is, which the uniform
    // grid in hitTest covers. If far larger systems need viewport culling,
    // bucket line/node vertices by grid cell at rebuild time and draw only
    // the ranges of intersecting cells.
    const gl = state.gl;
    const bg = palette().background;
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clearColor(bg[0], bg[1], bg[2], 1);
    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.useProgram(state.program);
    gl.uniform2f(gl.getUniformLocation(state.program, 'u_center'), state.centerX, state.centerY);
    gl.uniform2f(gl.getUniformLocation(state.program, 'u_viewport'), canvas.width, canvas.height);
    gl.uniform1f(gl.getUniformLocation(state.program, 'u_scale'), state.scale * (window.devicePixelRatio || 1));
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    drawBuffer(state.lineBuffer, gl.LINES, state.lineCount, 1, false);
    const transformerSize = state.nodeCount > 2000 ? 5 : 9;
    drawBuffer(
      state.transformerBuffer, gl.POINTS, state.transformerCount,
      transformerSize, false);
    const pointSize = state.lod < 2 ? 13 : (state.nodeCount > 2000 ? 3 : 7);
    drawBuffer(state.nodeBuffer, gl.POINTS, state.nodeCount, pointSize, true);
  }

  function fit() {
    if (!canvas) return;
    // In windowed mode the current graph only holds the fetched window, so
    // fit always targets the full-network bounds; the local path sees the
    // same bounds as before (full graph at LOD2, centroids otherwise).
    const bounds = state.fullBounds || boundsOfNodes(state.graph?.nodes);
    if (!bounds) return;
    state.centerX = (bounds.minX + bounds.maxX) / 2;
    state.centerY = (bounds.minY + bounds.maxY) / 2;
    const rect = canvas.getBoundingClientRect();
    state.scale = Math.max(0.02, Math.min((rect.width - 60) / Math.max(1, bounds.maxX - bounds.minX),
      (rect.height - 60) / Math.max(1, bounds.maxY - bounds.minY)));
    draw();
    scheduleViewportUpdate(0);
  }

  // Manual LOD override ('auto' re-enables zoom-driven selection). In windowed
  // mode an LOD change invalidates the fetched window and refetches; the local
  // path re-aggregates as before.
  function setLod(value) {
    if (String(value) === 'auto') {
      if (!state.windowed) {
        applyLocalLod(state.full?.nodes.length > 12000 ? 1 : 2);
        if (lodSelect) lodSelect.value = String(state.lod);
        return;
      }
      state.lodAuto = true;
      state.lod = autoLod();
      state.window.bbox = null;
      scheduleViewportUpdate(0);
      return;
    }
    state.lodAuto = false;
    state.lod = Math.max(0, Math.min(2, Number(value) || 0));
    if (state.windowed) {
      state.window.bbox = null;
      scheduleViewportUpdate(0);
      return;
    }
    state.graph = aggregate(state.full, state.lod);
    rebuildBuffers();
    fit();
    updateStatus();
  }

  function updateStatus(extra) {
    updateLegend();
    if (!status) return;
    if (!state.graph && !extra) return;
    const selected = state.selected ? ` · 已选 ${state.selected.domain.toUpperCase()} ${state.selected.index}` : '';
    // Result honesty first (stale/unconverged), then backend-declared
    // limitations (topology window, then result window), surfaced verbatim.
    const meta = state.result.active ? state.result.meta : null;
    let warning = '';
    if (meta && meta.result_matches_current_system === false) {
      warning = '结果滞后于当前模型，请重跑潮流';
    } else if (meta && meta.converged === false) {
      warning = '潮流结果未收敛';
    } else if (state.window.limitations.length) {
      warning = state.window.limitations[0];
    } else if (state.result.active && state.result.limitations.length) {
      warning = state.result.limitations[0];
    }
    const limitation = warning ? ` · ⚠ ${warning}` : '';
    const autoTag = state.lodAuto && state.windowed ? '自动 ' : '';
    status.textContent = extra ||
      `${autoTag}LOD${state.lod} · ${(state.graph?.nodes.length || 0).toLocaleString()} 节点 · ${(state.graph?.edges.length || 0).toLocaleString()} 连边${selected}${limitation}`;
  }

  function worldAt(event) {
    const rect = canvas.getBoundingClientRect();
    return {
      x: state.centerX + (event.clientX - rect.left - rect.width / 2) / state.scale,
      y: state.centerY + (event.clientY - rect.top - rect.height / 2) / state.scale,
    };
  }

  function hitTest(event) {
    // Precise picking stays LOD2-only; coarser LODs show aggregated groups.
    if (state.lod !== 2) return null;
    const data = state.soa.nodeData;
    if (!data || !state.nodeCount) return null;
    const point = worldAt(event);
    const maxDistance = 14 / Math.max(state.scale, 0.0001);
    let best = -1;
    let bestD2 = maxDistance * maxDistance;
    const consider = slot => {
      const dx = data[slot * 6] - point.x;
      const dy = data[slot * 6 + 1] - point.y;
      const d2 = dx * dx + dy * dy;
      if (d2 < bestD2) {
        bestD2 = d2;
        best = slot;
      }
    };
    const grid = state.grid;
    if (grid) {
      // Only scan cells intersecting the pick circle's bounding box.
      const clampX = x => Math.max(0, Math.min(grid.cols - 1, Math.floor((x - grid.minX) / grid.cellW)));
      const clampY = y => Math.max(0, Math.min(grid.rows - 1, Math.floor((y - grid.minY) / grid.cellH)));
      const cx0 = clampX(point.x - maxDistance);
      const cx1 = clampX(point.x + maxDistance);
      const cy0 = clampY(point.y - maxDistance);
      const cy1 = clampY(point.y + maxDistance);
      for (let cy = cy0; cy <= cy1; cy += 1) {
        for (let cx = cx0; cx <= cx1; cx += 1) {
          const bucket = grid.cells.get(cy * grid.cols + cx);
          if (bucket) bucket.forEach(consider);
        }
      }
    } else {
      for (let i = 0; i < state.nodeCount; i += 1) consider(i);
    }
    if (best < 0) return null;
    return { domain: state.soa.nodeDomains[best], index: state.soa.nodeIndices[best] };
  }

  function selectNode(node, openLocal) {
    if (!node || node.index == null) return;
    state.selected = { domain: node.domain, index: Number(node.index) };
    applySelection();
    draw();
    updateStatus();
    window.dispatchEvent(new CustomEvent('hysim:network-selection', {
      detail: { ref: { ...state.selected }, openLocal: !!openLocal },
    }));
  }

  function bindEvents() {
    canvas.addEventListener('pointerdown', event => {
      state.dragging = true;
      state.moved = false;
      state.start = { clientX: event.clientX, clientY: event.clientY,
        centerX: state.centerX, centerY: state.centerY };
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener('pointermove', event => {
      if (!state.dragging || !state.start) return;
      const dx = event.clientX - state.start.clientX;
      const dy = event.clientY - state.start.clientY;
      if (Math.abs(dx) + Math.abs(dy) > 3) state.moved = true;
      state.centerX = state.start.centerX - dx / state.scale;
      state.centerY = state.start.centerY - dy / state.scale;
      draw();
      scheduleViewportUpdate();
    });
    canvas.addEventListener('pointerup', event => {
      if (!state.moved) selectNode(hitTest(event), false);
      state.dragging = false;
      state.start = null;
    });
    canvas.addEventListener('dblclick', event => selectNode(hitTest(event), true));
    canvas.addEventListener('wheel', event => {
      event.preventDefault();
      const before = worldAt(event);
      state.scale = Math.max(0.01, Math.min(80, state.scale * Math.exp(-event.deltaY * 0.0015)));
      const after = worldAt(event);
      state.centerX += before.x - after.x;
      state.centerY += before.y - after.y;
      draw();
      scheduleViewportUpdate();
    }, { passive: false });
    document.getElementById('btnOverviewFit')?.addEventListener('click', fit);
    document.getElementById('btnOverviewLocal')?.addEventListener('click', () => {
      if (!state.selected) { updateStatus('请先在 LOD2 中选择母线'); return; }
      window.dispatchEvent(new CustomEvent('hysim:network-selection', {
        detail: { ref: { ...state.selected }, openLocal: true },
      }));
    });
    document.getElementById('btnOverviewTable')?.addEventListener('click', () => {
      document.body.classList.add('network-overview-table');
      window.dispatchEvent(new CustomEvent('hysim:overview-table'));
    });
    lodSelect?.addEventListener('change', event => setLod(event.target.value));
  }

  function init() {
    root = document.getElementById('networkOverview');
    canvas = document.getElementById('networkOverviewCanvas');
    status = document.getElementById('networkOverviewStatus');
    lodSelect = document.getElementById('networkOverviewLod');
    if (!root || !canvas) return false;
    if (!initGl()) {
      root.dataset.webgl = 'unavailable';
      return false;
    }
    bindEvents();
    state.resizeObserver = new ResizeObserver(resize);
    state.resizeObserver.observe(canvas);
    return true;
  }

  function show(system, options = {}) {
    if (!root && !init()) return false;
    if (!state.gl) return false;
    state.system = system;
    state.fetchWindow = typeof options.fetchTopologyWindow === 'function'
      ? options.fetchTopologyWindow : null;
    state.fetchResultWindow = typeof options.fetchResultWindow === 'function'
      ? options.fetchResultWindow : null;
    state.full = layoutGraph(buildFullGraph(system || {}));
    state.fullBounds = boundsOfNodes(state.full.nodes);
    // Windowed mode requires a backend fetcher and enough geo-referenced
    // buses; anything else stays on the local full-system rendering path.
    state.windowed = !!(state.fetchWindow && state.fullBounds &&
      geoCoverage(state.full.nodes) >= GEO_COVERAGE_MIN);
    // Drop the previous system's rendered graph; a first-fetch failure in
    // windowed mode detects the empty graph and degrades to the local path.
    state.graph = null;
    state.result.seq += 1;
    state.result.active = false;
    state.result.nodeVm = null;
    state.result.branchLoading = null;
    state.result.meta = null;
    state.result.limitations = [];
    state.window.bbox = null;
    state.window.lod = -1;
    state.window.limitations = [];
    state.window.coverage = null;
    state.window.seq += 1;  // invalidate in-flight fetches for the old system
    if (state.window.timer) {
      clearTimeout(state.window.timer);
      state.window.timer = 0;
    }
    updateLegend();  // result state reset above: hide a stale legend
    state.active = true;
    document.body.classList.add('network-overview-active');
    document.body.classList.remove('network-overview-table');
    root.hidden = false;
    resize();
    if (state.windowed) {
      state.lodAuto = true;
      if (lodSelect) lodSelect.value = 'auto';
      fit();               // full-network bounds; establishes view + scale
      state.lod = autoLod();
      if (state.window.timer) {
        clearTimeout(state.window.timer);
        state.window.timer = 0;
      }
      onViewportSettled(); // first fetch for the fitted viewport, no debounce
    } else {
      state.lodAuto = false;
      const initialLod = state.full.nodes.length > 12000 ? 1 : 2;
      if (lodSelect) lodSelect.value = String(initialLod);
      applyLocalLod(initialLod);
    }
    return true;
  }

  function hide() {
    state.active = false;
    state.window.seq += 1;  // drop any in-flight window response
    state.result.seq += 1;
    if (state.window.timer) {
      clearTimeout(state.window.timer);
      state.window.timer = 0;
    }
    document.body.classList.remove('network-overview-active', 'network-overview-table');
    if (legend) legend.style.display = 'none';
    if (root) root.hidden = true;
  }

  function selectRef(ref) {
    if (!ref || !state.active) return false;
    state.selected = { domain: String(ref.domain), index: Number(ref.index) };
    if (state.lod !== 2 && lodSelect) {
      lodSelect.value = '2';
      setLod(2);
    } else {
      applySelection();
      draw();
      updateStatus();
    }
    return true;
  }

  function stats() {
    const edgeKinds = {};
    (state.full?.edges || []).forEach(edge => {
      edgeKinds[edge.kind] = (edgeKinds[edge.kind] || 0) + 1;
    });
    return {
      schema: 'hysim_network_overview_v1', active: state.active, lod: state.lod,
      lod_auto: state.lodAuto,
      windowed: state.windowed,
      nodes: state.graph?.nodes.length || 0, edges: state.graph?.edges.length || 0,
      edge_kinds: edgeKinds,
      transformer_edges: edgeKinds.transformer || 0,
      webgl2: !!state.gl, selected: state.selected ? { ...state.selected } : null,
      coordinate_coverage: state.window.coverage,
      model_limitations: [...state.window.limitations],
      result_active: state.result.active,
      result_meta: state.result.meta ? { ...state.result.meta } : null,
      result_nodes: state.result.nodeVm ? state.result.nodeVm.size : 0,
      result_branches: state.result.branchLoading ? state.result.branchLoading.size : 0,
      result_limitations: [...state.result.limitations],
      canvas_pixels: canvas ? canvas.width * canvas.height : 0,
    };
  }

  return { init, show, hide, fit, setLod, selectRef, refreshResults, stats, get active() { return state.active; } };
})();
