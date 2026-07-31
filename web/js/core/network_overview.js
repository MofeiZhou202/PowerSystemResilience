/**
 * network_overview.js - WebGL2 full-network overview for large systems.
 * Keeps one stable {domain,index} identity across overview, tables and SVG.
 */
'use strict';

const NetworkOverview = (() => {
  const state = {
    active: false,
    system: null,
    full: null,
    graph: null,
    lod: 2,
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
    resizeObserver: null,
  };

  let root;
  let canvas;
  let status;
  let lodSelect;

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

  function pushVertex(target, node, color) {
    target.push(node.x, node.y, color[0], color[1], color[2], color[3]);
  }

  function rebuildBuffers() {
    if (!state.gl || !state.graph) return;
    const gl = state.gl;
    const colors = palette();
    colors.edge[3] = 0.22;
    colors.hybrid[3] = 0.7;
    colors.transformer[3] = 0.85;
    const byKey = new Map(state.graph.nodes.map(node => [node.key, node]));
    const lines = [];
    const transformerPoints = [];
    state.graph.edges.forEach(edge => {
      const a = byKey.get(edge.source);
      const b = byKey.get(edge.target);
      if (!a || !b) return;
      const color = edge.kind === 'transformer'
        ? colors.transformer
        : (edge.domain === 'hybrid' ? colors.hybrid : colors.edge);
      pushVertex(lines, a, color);
      pushVertex(lines, b, color);
      if (edge.kind === 'transformer') {
        pushVertex(transformerPoints, {
          x: (a.x + b.x) / 2,
          y: (a.y + b.y) / 2,
        }, colors.transformer);
      }
    });
    const points = [];
    state.graph.nodes.forEach(node => {
      const selected = state.selected && node.domain === state.selected.domain &&
        Number(node.index) === Number(state.selected.index);
      pushVertex(points, node, selected ? colors.selected : (colors[node.domain] || colors.ac));
    });
    gl.bindBuffer(gl.ARRAY_BUFFER, state.lineBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(lines), gl.STATIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, state.transformerBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(transformerPoints), gl.STATIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, state.nodeBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(points), gl.STATIC_DRAW);
    state.lineCount = lines.length / 6;
    state.transformerCount = transformerPoints.length / 6;
    state.nodeCount = points.length / 6;
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
    if (!state.graph?.nodes.length || !canvas) return;
    const xs = state.graph.nodes.map(node => node.x);
    const ys = state.graph.nodes.map(node => node.y);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    state.centerX = (minX + maxX) / 2;
    state.centerY = (minY + maxY) / 2;
    const rect = canvas.getBoundingClientRect();
    state.scale = Math.max(0.02, Math.min((rect.width - 60) / Math.max(1, maxX - minX),
      (rect.height - 60) / Math.max(1, maxY - minY)));
    draw();
  }

  function setLod(value) {
    state.lod = Math.max(0, Math.min(2, Number(value) || 0));
    state.graph = aggregate(state.full, state.lod);
    rebuildBuffers();
    fit();
    updateStatus();
  }

  function updateStatus(extra) {
    if (!status || !state.graph) return;
    const selected = state.selected ? ` · 已选 ${state.selected.domain.toUpperCase()} ${state.selected.index}` : '';
    status.textContent = extra ||
      `LOD${state.lod} · ${state.graph.nodes.length.toLocaleString()} 节点 · ${state.graph.edges.length.toLocaleString()} 连边${selected}`;
  }

  function worldAt(event) {
    const rect = canvas.getBoundingClientRect();
    return {
      x: state.centerX + (event.clientX - rect.left - rect.width / 2) / state.scale,
      y: state.centerY + (event.clientY - rect.top - rect.height / 2) / state.scale,
    };
  }

  function hitTest(event) {
    if (state.lod !== 2) return null;
    const point = worldAt(event);
    const maxDistance = 14 / Math.max(state.scale, 0.0001);
    let best = null;
    let bestD2 = maxDistance * maxDistance;
    state.graph.nodes.forEach(node => {
      const dx = node.x - point.x;
      const dy = node.y - point.y;
      const d2 = dx * dx + dy * dy;
      if (d2 < bestD2) { bestD2 = d2; best = node; }
    });
    return best;
  }

  function selectNode(node, openLocal) {
    if (!node || node.index == null) return;
    state.selected = { domain: node.domain, index: Number(node.index) };
    rebuildBuffers();
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

  function show(system) {
    if (!root && !init()) return false;
    if (!state.gl) return false;
    state.system = system;
    state.full = layoutGraph(buildFullGraph(system || {}));
    state.active = true;
    document.body.classList.add('network-overview-active');
    document.body.classList.remove('network-overview-table');
    root.hidden = false;
    const initialLod = state.full.nodes.length > 12000 ? 1 : 2;
    if (lodSelect) lodSelect.value = String(initialLod);
    setLod(initialLod);
    resize();
    return true;
  }

  function hide() {
    state.active = false;
    document.body.classList.remove('network-overview-active', 'network-overview-table');
    if (root) root.hidden = true;
  }

  function selectRef(ref) {
    if (!ref || !state.active) return false;
    state.selected = { domain: String(ref.domain), index: Number(ref.index) };
    if (state.lod !== 2 && lodSelect) {
      lodSelect.value = '2';
      setLod(2);
    } else {
      rebuildBuffers();
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
      nodes: state.graph?.nodes.length || 0, edges: state.graph?.edges.length || 0,
      edge_kinds: edgeKinds,
      transformer_edges: edgeKinds.transformer || 0,
      webgl2: !!state.gl, selected: state.selected ? { ...state.selected } : null,
      canvas_pixels: canvas ? canvas.width * canvas.height : 0,
    };
  }

  return { init, show, hide, fit, setLod, selectRef, stats, get active() { return state.active; } };
})();
