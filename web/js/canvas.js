/**
 * canvas.js — SVG canvas for power system single-line diagram editing.
 * Handles: component placement, drag-and-drop, pan/zoom, connections, selection.
 */
'use strict';

const Canvas = (() => {
  // Helper: parse numeric value, preserving 0 and other falsy numbers.
  // Returns fallback only when value is undefined, null, empty, or NaN.
  function numOr(v, fallback) {
    if (v == null || v === '') return fallback;
    const n = Number(v);
    return Number.isFinite(n) ? n : fallback;
  }

  function cloneDynamicModel(profile) {
    if (!profile || typeof profile !== 'object' || Array.isArray(profile)) return undefined;
    try {
      return JSON.parse(JSON.stringify(profile));
    } catch (_) {
      return undefined;
    }
  }

  function addDynamicModel(row, params) {
    const profile = cloneDynamicModel(params?.dynamic_model);
    if (profile && Object.keys(profile).length > 0) row.dynamic_model = profile;
    return row;
  }

  // ========== Display Unit Helpers ==========
  function getPowerUnit() {
    const el = document.getElementById('pfDisplayUnit');
    return el ? el.value : 'MW';
  }
  function pScale() { const u = getPowerUnit(); return u === 'kW' ? 1e3 : u === 'W' ? 1e6 : 1; }
  function pUnit()  { const u = getPowerUnit(); return u === 'kW' ? 'kW' : u === 'W' ? 'W' : 'MW'; }
  function pConv(mw) { return mw * pScale(); }
  function pFmt(mw, d = 1) { return (mw * pScale()).toFixed(d); }

  // ========== State ==========
  const state = {
    components: [],       // {id, type, x, y, params, el}
    connections: [],      // {id, from:{compId, portId}, to:{compId, portId}, el}
    nextId: 1,
    selectedId: null,
    selectedConnectionId: null,  // selected connection line
    selectedIds: new Set(),  // multi-selection (box select)
    mode: 'select',       // 'select' | 'connect' | 'place'
    placeType: null,      // component type being placed
    zoom: 1,
    panX: 0, panY: 0,
    isPanning: false,
    panStart: null,
    isDragging: false,
    dragTarget: null,
    dragOffset: null,
    connectStart: null,   // {compId, portId, x, y}
    tempLine: null,
    isBoxSelecting: false,
    boxSelectStart: null,  // {x, y} in SVG coords
    boxSelectRect: null,   // SVG rect element
    baseMva: 100,          // system base MVA (preserved from loaded system)
    connectionStyle: 'avoid', // 'straight' | 'orthogonal' | 'avoid' (doc §10.2/§10.3)
    alignSnap: true,       // snap to neighbour x/y while dragging (doc §18 Phase 4)
  };

  let _preservedModelBlocks = {};

  function cloneJsonBlock(value) {
    if (!value || typeof value !== 'object') return undefined;
    try {
      return JSON.parse(JSON.stringify(value));
    } catch (_) {
      return undefined;
    }
  }

  // Cached routing context (obstacle boxes + grid + parallel-offset groups) used
  // by the §10.3 auto-avoidance router.  Rebuilt by buildRouteContext() before a
  // full re-route; null means "route cheaply" (plain orthogonal) so live drag
  // stays responsive.
  let _routeCtx = null;

  // Last auto-layout statistics ({buses, rings}) for the status bar.
  let _layoutStats = null;

  let svg, componentsLayer, connectionsLayer, resultsLayer, tempLayer;
  let viewBox = { x: -200, y: -100, w: 1200, h: 700 };

  // Viewport culling (virtualized rendering) for large systems: when the
  // component count exceeds this threshold, component glyphs whose anchor falls
  // outside the visible viewBox (plus a margin) are display:none'd so the
  // browser skips their layout/paint while panning and zooming.  Connection
  // wires are left intact (cheap single elements).  Below the threshold every
  // component is rendered, so small cases are unaffected.
  const CULL_THRESHOLD = 1500;
  let _cullActive = false;
  let _cullPending = false;

  // ========== Init ==========
  function init() {
    svg = document.getElementById('canvas');
    componentsLayer = document.getElementById('componentsLayer');
    connectionsLayer = document.getElementById('connectionsLayer');
    resultsLayer = document.getElementById('resultsLayer');
    tempLayer = document.getElementById('tempLayer');

    // Set initial viewbox
    updateViewBox();

    // Event listeners
    svg.addEventListener('mousedown', onMouseDown);
    svg.addEventListener('mousemove', onMouseMove);
    svg.addEventListener('mouseup', onMouseUp);
    svg.addEventListener('wheel', onWheel, { passive: false });
    svg.addEventListener('dblclick', onDblClick);

    // Keyboard
    document.addEventListener('keydown', onKeyDown);

    // Restore the persisted connection style (doc §10.2/§10.3).
    try {
      const cs = localStorage.getItem('connectionStyle');
      if (['straight', 'orthogonal', 'avoid'].includes(cs)) state.connectionStyle = cs;
    } catch (e) { /* ignore */ }
  }

  // ========== View ==========
  function updateViewBox() {
    svg.setAttribute('viewBox', `${viewBox.x} ${viewBox.y} ${viewBox.w} ${viewBox.h}`);
    scheduleViewportCulling();
  }

  // Hide component glyphs outside the visible viewBox (+margin) for large
  // systems.  O(n) over components, coalesced to one run per animation frame so
  // rapid pan/zoom stays smooth.  Reversible: dropping below the threshold (or a
  // fit-all view) re-shows everything.
  function updateViewportCulling() {
    const comps = state.components;
    if (comps.length < CULL_THRESHOLD) {
      if (_cullActive) {
        comps.forEach(c => { if (c.el && c.el.style.display === 'none') c.el.style.display = ''; });
        _cullActive = false;
      }
      return;
    }
    _cullActive = true;
    const mx = viewBox.w * 0.2, my = viewBox.h * 0.2;
    const x0 = viewBox.x - mx, x1 = viewBox.x + viewBox.w + mx;
    const y0 = viewBox.y - my, y1 = viewBox.y + viewBox.h + my;
    for (let i = 0; i < comps.length; i++) {
      const c = comps[i];
      if (!c.el) continue;
      const visible = c.x >= x0 && c.x <= x1 && c.y >= y0 && c.y <= y1;
      const want = visible ? '' : 'none';
      if (c.el.style.display !== want) c.el.style.display = want;
    }
  }

  function scheduleViewportCulling() {
    if (_cullPending) return;
    _cullPending = true;
    requestAnimationFrame(() => { _cullPending = false; updateViewportCulling(); });
  }

  function screenToSvg(clientX, clientY) {
    const pt = svg.createSVGPoint();
    pt.x = clientX;
    pt.y = clientY;
    const ctm = svg.getScreenCTM();
    if (ctm) {
      return pt.matrixTransform(ctm.inverse());
    }
    // Fallback if getScreenCTM unavailable
    const rect = svg.getBoundingClientRect();
    const x = viewBox.x + (clientX - rect.left) / rect.width * viewBox.w;
    const y = viewBox.y + (clientY - rect.top) / rect.height * viewBox.h;
    return { x, y };
  }

  function snapToGrid(val, grid = 20) {
    return Math.round(val / grid) * grid;
  }

  // ========== Component Management ==========
  function addComponent(type, x, y, params = null, rotation = 0) {
    const id = state.nextId++;
    const p = params || { ...COMP.defaults[type] };
    if (!p.name || p.name === COMP.defaults[type]?.name) {
      p.name = (COMP.defaults[type]?.name || type) + ' ' + id;
    }

    const comp = { id, type, x: snapToGrid(x), y: snapToGrid(y), rotation: rotation || 0, params: p, el: null };
    state.components.push(comp);
    renderComponent(comp);
    updateInfo();
    return comp;
  }

  function removeComponent(id) {
    const idx = state.components.findIndex(c => c.id === id);
    if (idx < 0) return;
    const comp = state.components[idx];
    // Remove associated connections
    state.connections = state.connections.filter(conn => {
      if (conn.from.compId === id || conn.to.compId === id) {
        conn.el?.remove();
        return false;
      }
      return true;
    });
    comp.el?.remove();
    state.components.splice(idx, 1);
    if (state.selectedId === id) {
      state.selectedId = null;
      if (typeof App !== 'undefined') App.onSelectionChanged(null);
    }
    // Topology changed → clear stale PF results and visualization overlays
    clearResults();
    updateInfo();
  }

  function removeConnection(connId) {
    const idx = state.connections.findIndex(c => c.id === connId);
    if (idx < 0) return;
    state.connections[idx].el?.remove();
    state.connections.splice(idx, 1);
    if (state.selectedConnectionId === connId) {
      state.selectedConnectionId = null;
    }
    // Topology changed → clear stale PF results and visualization overlays
    clearResults();
    updateInfo();
  }

  function selectConnection(connId) {
    // Deselect previous connection
    if (state.selectedConnectionId) {
      const prev = state.connections.find(c => c.id === state.selectedConnectionId);
      if (prev?.el) prev.el.classList.remove('conn-selected');
    }
    // Deselect component if any
    if (connId !== null && state.selectedId !== null) {
      selectComponent(null);
    }
    state.selectedConnectionId = connId;
    if (connId !== null) {
      const conn = state.connections.find(c => c.id === connId);
      if (conn?.el) conn.el.classList.add('conn-selected');
    }
  }

  function getComponent(id) {
    return state.components.find(c => c.id === id);
  }

  function selectComponent(id) {
    // Clear multi-selection when single-selecting
    clearMultiSelection();
    // Clear selected connection
    selectConnection(null);
    // Deselect previous
    if (state.selectedId !== null) {
      const prev = getComponent(state.selectedId);
      if (prev?.el) prev.el.classList.remove('selected');
    }
    state.selectedId = id;
    if (id !== null) {
      const comp = getComponent(id);
      if (comp?.el) comp.el.classList.add('selected');
      state.selectedIds.add(id);
    }
    if (typeof App !== 'undefined') App.onSelectionChanged(id);
  }

  function selectMultiple(ids) {
    clearMultiSelection();
    state.selectedId = null;
    ids.forEach(id => {
      state.selectedIds.add(id);
      const comp = getComponent(id);
      if (comp?.el) comp.el.classList.add('selected');
    });
    // Set selectedId to first for property panel
    if (ids.length > 0) {
      state.selectedId = ids[0];
    }
    if (typeof App !== 'undefined') App.onSelectionChanged(state.selectedId, ids.length);
  }

  function clearMultiSelection() {
    state.selectedIds.forEach(id => {
      const comp = getComponent(id);
      if (comp?.el) comp.el.classList.remove('selected');
    });
    state.selectedIds.clear();
  }

  function removeSelected() {
    const ids = [...state.selectedIds];
    if (ids.length === 0 && state.selectedId !== null) ids.push(state.selectedId);
    if (ids.length === 0) return;
    ids.forEach(id => removeComponent(id));
    state.selectedIds.clear();
    state.selectedId = null;
    if (typeof App !== 'undefined') {
      App.onSelectionChanged(null);
      App.onTopologyChanged();
    }
    updateInfo();
  }

  // ========== Rendering ==========
  function renderComponent(comp) {
    const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
    g.classList.add('component', `comp-${comp.type}`);
    g.setAttribute('transform', `translate(${comp.x}, ${comp.y}) rotate(${comp.rotation || 0})`);
    g.dataset.compId = comp.id;

    // Symbol
    const symbolFn = COMP.symbols[comp.type];
    if (symbolFn) {
      g.innerHTML = symbolFn(comp.params);
    }

    // Invisible hit area
    const hit = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    hit.classList.add('comp-outline');
    hit.setAttribute('x', '-45');
    hit.setAttribute('y', '-35');
    hit.setAttribute('width', '90');
    hit.setAttribute('height', '80');
    hit.setAttribute('fill', 'transparent');
    hit.setAttribute('stroke', 'transparent');
    hit.setAttribute('stroke-width', '1');
    hit.setAttribute('rx', '4');
    g.insertBefore(hit, g.firstChild);

    // Ports
    const portDefs = COMP.ports[comp.type] || [];
    portDefs.forEach(pd => {
      const pg = document.createElementNS('http://www.w3.org/2000/svg', 'g');
      pg.classList.add('port');
      pg.dataset.portId = pd.id;
      pg.dataset.compId = comp.id;
      const c = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
      c.setAttribute('cx', pd.x);
      c.setAttribute('cy', pd.y);
      c.setAttribute('r', '4');
      pg.appendChild(c);
      g.appendChild(pg);
    });

    componentsLayer.appendChild(g);
    comp.el = g;
  }

  function rerenderComponent(comp) {
    comp.el?.remove();
    renderComponent(comp);
    if (comp.id === state.selectedId) {
      comp.el.classList.add('selected');
    }
    // Re-render connections
    state.connections.forEach(conn => {
      if (conn.from.compId === comp.id || conn.to.compId === comp.id) {
        rerenderConnection(conn);
      }
    });
  }

  function clearSolvedGeneratorDisplays() {
    state.components.forEach(comp => {
      if (comp.type !== 'generator' || !comp.params) return;
      delete comp.params._result_pg_mw;
      delete comp.params._result_qg_mvar;
      delete comp.params._result_p_unit;
    });
  }

  function formatSolvedPowerForComponent(mw) {
    const n = Number(mw);
    if (!Number.isFinite(n)) return null;
    return pFmt(n, Math.abs(n) >= 10 ? 1 : 2);
  }

  function resultRowsByIndexOrOrder(rows, comps) {
    const out = [];
    const list = Array.isArray(rows) ? rows : [];
    const used = new Set();
    comps.forEach((comp, order) => {
      const idx = Number(comp.params?.index);
      let row = null;
      if (Number.isFinite(idx)) {
        row = list.find((r, i) => !used.has(i) && Number(r.index) === idx) || null;
      }
      if (!row && Number.isFinite(idx)) {
        row = list.find((r, i) => !used.has(i) && Number(r.canvas_index) === idx) || null;
      }
      if (!row) {
        row = list.find((r, i) => !used.has(i) && Number(r.position) === order) || null;
      }
      if (!row) {
        row = list.find((r, i) => !used.has(i)) || null;
      }
      if (row) used.add(list.indexOf(row));
      out.push(row);
    });
    return out;
  }

  function applySolvedGeneratorDisplays(result) {
    clearSolvedGeneratorDisplays();
    const genComps = state.components.filter(comp => comp.type === 'generator');
    if (!genComps.length || !result) return;
    const rows = resultRowsByIndexOrOrder(result.geo_gen || result.generator_dispatch || [], genComps);
    genComps.forEach((comp, i) => {
      const row = rows[i];
      const pgText = formatSolvedPowerForComponent(row?.pg_mw);
      if (pgText === null) return;
      comp.params._result_pg_mw = pgText;
      const qg = Number(row?.qg_mvar);
      if (Number.isFinite(qg)) comp.params._result_qg_mvar = qg;
      comp.params._result_p_unit = pUnit();
    });
  }

  function refreshSolvedGeneratorComponents() {
    state.components
      .filter(comp => comp.type === 'generator')
      .forEach(comp => rerenderComponent(comp));
  }

  function normalizePortId(compId, portId) {
    const comp = getComponent(compId);
    if (!comp) return portId;
    const ports = COMP.ports[comp.type] || [];
    if (!ports.length) return portId;
    const requested = String(portId || '');
    if (ports.some(p => p.id === requested)) return requested;
    if (ports.length === 1) return ports[0].id;

    // Legacy saved layouts may carry generic side ports (left/right/top/bottom)
    // for symbols that now expose semantic ports (ac/dc, hv/lv, pcc, ...).
    const pickByCoord = (axis, dir) => {
      let best = ports[0];
      ports.forEach(p => {
        if (dir < 0 ? p[axis] < best[axis] : p[axis] > best[axis]) best = p;
      });
      return best.id;
    };
    if (requested === 'left') return pickByCoord('x', -1);
    if (requested === 'right') return pickByCoord('x', 1);
    if (requested === 'top') return pickByCoord('y', -1);
    if (requested === 'bottom') return pickByCoord('y', 1);
    return ports[0].id;
  }

  function getPortWorldPos(compId, portId) {
    const comp = getComponent(compId);
    if (!comp) return null;
    const actualPortId = normalizePortId(compId, portId);
    const portDef = (COMP.ports[comp.type] || []).find(p => p.id === actualPortId);
    if (!portDef) return null;
    // Apply rotation to port position
    const rad = (comp.rotation || 0) * Math.PI / 180;
    const rx = portDef.x * Math.cos(rad) - portDef.y * Math.sin(rad);
    const ry = portDef.x * Math.sin(rad) + portDef.y * Math.cos(rad);
    return { x: comp.x + rx, y: comp.y + ry };
  }

  // ========== Connection routing (doc §10.2 orthogonal / §10.3 avoidance) ====
  // A connection's geometry is a polyline (array of {x,y} world points).  The
  // visible/​hit elements are <path>; flow-overlay code reads geometry via
  // getConnGeom() instead of the DOM so straight/orthogonal/avoid all work.

  // Infer the natural exit direction of a port from its (rotated) offset sign.
  function portDirection(compId, portId) {
    const comp = getComponent(compId);
    if (!comp) return null;
    const actualPortId = normalizePortId(compId, portId);
    const pd = (COMP.ports[comp.type] || []).find(p => p.id === actualPortId);
    if (!pd) return null;
    const rad = (comp.rotation || 0) * Math.PI / 180;
    const rx = pd.x * Math.cos(rad) - pd.y * Math.sin(rad);
    const ry = pd.x * Math.sin(rad) + pd.y * Math.cos(rad);
    if (Math.abs(rx) >= Math.abs(ry)) return rx >= 0 ? 'right' : 'left';
    return ry >= 0 ? 'down' : 'up';
  }

  function makeStraightPoints(p1, p2) {
    return [{ x: p1.x, y: p1.y }, { x: p2.x, y: p2.y }];
  }

  // §10.2 orthogonal (Manhattan) polyline.  Generalises the doc snippet to a
  // points array and adds short port-exit stubs when the port directions are
  // known so wires leave a symbol cleanly before turning.
  function makeOrthogonalPoints(p1, p2, d1, d2, offset = 0) {
    const dx = Math.abs(p2.x - p1.x), dy = Math.abs(p2.y - p1.y);
    if (dx < 1 && dy < 1) return [{ x: p1.x, y: p1.y }, { x: p2.x, y: p2.y }];
    const stub = 18;
    const horiz = (d) => d === 'left' || d === 'right';
    const vert = (d) => d === 'up' || d === 'down';
    const s1 = d1 ? { x: p1.x + (d1 === 'right' ? stub : d1 === 'left' ? -stub : 0),
                      y: p1.y + (d1 === 'down' ? stub : d1 === 'up' ? -stub : 0) } : null;
    const s2 = d2 ? { x: p2.x + (d2 === 'right' ? stub : d2 === 'left' ? -stub : 0),
                      y: p2.y + (d2 === 'down' ? stub : d2 === 'up' ? -stub : 0) } : null;
    const a = s1 || p1, b = s2 || p2;
    const ad = Math.abs(b.x - a.x), bd = Math.abs(b.y - a.y);
    const pts = [{ x: p1.x, y: p1.y }];
    if (s1) pts.push({ x: s1.x, y: s1.y });
    // Choose the bend axis: follow the dominant port orientation, else the
    // longer span (horizontal-first when wide, vertical-first when tall).
    let horizFirst;
    if (d1 && horiz(d1)) horizFirst = true;
    else if (d1 && vert(d1)) horizFirst = false;
    else horizFirst = ad >= bd;
    if (horizFirst) {
      const midX = (a.x + b.x) / 2 + offset; // offset spreads parallel wires
      pts.push({ x: midX, y: a.y }, { x: midX, y: b.y });
    } else {
      const midY = (a.y + b.y) / 2 + offset;
      pts.push({ x: a.x, y: midY }, { x: b.x, y: midY });
    }
    if (s2) pts.push({ x: s2.x, y: s2.y });
    pts.push({ x: p2.x, y: p2.y });
    return simplifyPoints(pts);
  }

  // Drop collinear / duplicate vertices so paths stay minimal.
  function simplifyPoints(pts) {
    if (!pts || pts.length <= 2) return pts;
    const out = [pts[0]];
    for (let i = 1; i < pts.length - 1; i++) {
      const a = out[out.length - 1], b = pts[i], c = pts[i + 1];
      if ((Math.abs(a.x - b.x) < 0.5 && Math.abs(b.x - c.x) < 0.5) ||
          (Math.abs(a.y - b.y) < 0.5 && Math.abs(b.y - c.y) < 0.5)) continue; // collinear
      if (Math.abs(a.x - b.x) < 0.5 && Math.abs(a.y - b.y) < 0.5) continue;   // duplicate
      out.push(b);
    }
    out.push(pts[pts.length - 1]);
    return out;
  }

  function pointsToPath(points) {
    if (!points || points.length === 0) return '';
    return points.map((p, i) => `${i === 0 ? 'M' : 'L'} ${p.x} ${p.y}`).join(' ');
  }

  // Pick the polyline for a connection according to the active style.  `cheap`
  // forces the fast orthogonal route (used during live drag) instead of A*.
  function routeConnection(conn, opts = {}) {
    const p1 = getPortWorldPos(conn.from.compId, conn.from.portId);
    const p2 = getPortWorldPos(conn.to.compId, conn.to.portId);
    if (!p1 || !p2) return null;
    const style = state.connectionStyle || 'orthogonal';
    if (style === 'straight') return makeStraightPoints(p1, p2);
    const d1 = portDirection(conn.from.compId, conn.from.portId);
    const d2 = portDirection(conn.to.compId, conn.to.portId);
    if (style === 'avoid' && !opts.cheap && _routeCtx) {
      const routed = routeAvoid(conn, p1, p2, d1, d2, _routeCtx);
      if (routed) return routed;
    }
    const offset = _routeCtx ? (_routeCtx.offsetOf.get(conn.id) || 0) : 0;
    return makeOrthogonalPoints(p1, p2, d1, d2, offset);
  }

  // Geometry accessor for overlay code: returns the polyline plus its
  // arc-length midpoint and unit tangent (oriented from→to), and — when a
  // component id is supplied — which endpoint is the component vs. the bus.
  // Falls back to a straight 2-point polyline if conn.geom is missing/stale so
  // overlays never break (doc §10.1 safety net).
  function getConnGeom(conn, opts = {}) {
    let pts = conn.geom && conn.geom.points;
    if (!pts || pts.length < 2) {
      const a = getPortWorldPos(conn.from.compId, conn.from.portId);
      const b = getPortWorldPos(conn.to.compId, conn.to.portId);
      if (!a || !b) return null;
      pts = [a, b];
    }
    let total = 0;
    const segs = [];
    for (let i = 1; i < pts.length; i++) {
      const ddx = pts[i].x - pts[i - 1].x, ddy = pts[i].y - pts[i - 1].y;
      const len = Math.hypot(ddx, ddy);
      segs.push({ len, dx: ddx, dy: ddy, x0: pts[i - 1].x, y0: pts[i - 1].y });
      total += len;
    }
    let half = total / 2, mx = pts[0].x, my = pts[0].y, tdx = 1, tdy = 0;
    for (let i = 0; i < segs.length; i++) {
      const s = segs[i];
      if (half <= s.len || i === segs.length - 1) {
        const t = s.len ? half / s.len : 0;
        mx = s.x0 + s.dx * t; my = s.y0 + s.dy * t;
        const l = s.len || 1; tdx = s.dx / l; tdy = s.dy / l;
        break;
      }
      half -= s.len;
    }
    const fromPt = pts[0], toPt = pts[pts.length - 1];
    let compEnd = fromPt, busEnd = toPt, compIsFrom = true;
    if (opts.fromCompId != null) {
      compIsFrom = conn.from.compId === opts.fromCompId;
      compEnd = compIsFrom ? fromPt : toPt;
      busEnd = compIsFrom ? toPt : fromPt;
    }
    return { points: pts, fromPt, toPt, compEnd, busEnd, compIsFrom, mx, my, tdx, tdy };
  }

  // Arrow placement at the polyline midpoint, pointing toward `headPt` (which
  // must be one of g.fromPt / g.toPt / g.compEnd / g.busEnd — these alias the
  // same objects).  Returns the midpoint, rotation angle and a perpendicular
  // label offset, matching the old straight-line overlay maths.
  function flowArrow(g, headPt) {
    let tx = g.tdx, ty = g.tdy;
    if (headPt !== g.toPt) { tx = -tx; ty = -ty; }
    return {
      mx: g.mx, my: g.my,
      angle: Math.atan2(ty, tx) * 180 / Math.PI,
      offX: -ty * 14, offY: tx * 14,
    };
  }

  // ---- §10.3 auto-avoidance routing ------------------------------------------
  // Axis-aligned obstacle box for a component (buses are short & wide, devices
  // are taller).  Inflated by `pad` so wires keep clear of the symbol.
  const FLOW_ARROW_EPS_MW = 1e-9;

  function normalizedPowerPct(absPower, minPower, powerRange) {
    const p = Number(absPower);
    return Number.isFinite(p) ? 100 * (p - minPower) / powerRange : 0;
  }

  function addFlowLabel(x, y, powerMW, color) {
    const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
    label.classList.add('viz-overlay', 'flow-label');
    label.setAttribute('x', x);
    label.setAttribute('y', y);
    label.setAttribute('fill', color);
    label.textContent = `${pFmt(Math.abs(numOr(powerMW, 0)))} ${pUnit()}`;
    resultsLayer.appendChild(label);
    return label;
  }

  function addFlowArrow(fa, absPowerMW, color) {
    const absPower = Math.abs(numOr(absPowerMW, 0));
    if (absPower <= FLOW_ARROW_EPS_MW) return null;
    const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
    arrow.classList.add('viz-overlay', 'flow-arrow-dot');
    const sz = Math.max(5, Math.min(10, 5 + absPower / 100));
    arrow.setAttribute('points', `${-sz},-${sz/2} ${sz},0 ${-sz},${sz/2}`);
    arrow.setAttribute('transform', `translate(${fa.mx},${fa.my}) rotate(${fa.angle})`);
    arrow.setAttribute('fill', color);
    arrow.setAttribute('opacity', '0.85');
    resultsLayer.appendChild(arrow);
    return arrow;
  }

  function addFlowMarker(g, headPt, powerMW, color) {
    const fa = flowArrow(g, headPt);
    const absPower = Math.abs(numOr(powerMW, 0));
    addFlowArrow(fa, absPower, color);
    addFlowLabel(fa.mx + fa.offX, fa.my + fa.offY, absPower, color);
    return fa;
  }

  function componentObstacleBox(comp, pad = 10) {
    const isBus = comp.type === 'ac_bus' || comp.type === 'dc_bus';
    const hw = isBus ? 44 : 45;
    const top = isBus ? 12 : 35;
    const h = isBus ? 24 : 80;
    return { id: comp.id, x: comp.x - hw - pad, y: comp.y - top - pad,
             w: 2 * hw + 2 * pad, h: h + 2 * pad };
  }

  function rectsOverlap(a, b) {
    return a.x < b.x + b.width && a.x + a.width > b.x &&
           a.y < b.y + b.height && a.y + a.height > b.y;
  }

  // Build (and cache) the routing context: obstacle boxes, a coarse blocked-cell
  // grid for A*, the per-box cell ranges (so a wire may exit its own endpoints),
  // and parallel-offset amounts for connections that share an endpoint pair.
  function buildRouteContext() {
    const comps = state.components;
    if (!comps.length) { _routeCtx = null; return null; }
    // Per-connection A* does not scale to very large diagrams; above this many
    // connections we skip the grid so routing falls back to fast orthogonal.
    if (state.connections.length > 800) { _routeCtx = null; return null; }
    const boxes = comps.map(c => componentObstacleBox(c));
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    boxes.forEach(b => {
      minX = Math.min(minX, b.x); minY = Math.min(minY, b.y);
      maxX = Math.max(maxX, b.x + b.w); maxY = Math.max(maxY, b.y + b.h);
    });
    const margin = 100; minX -= margin; minY -= margin; maxX += margin; maxY += margin;
    const cell = 20;
    let cols = Math.ceil((maxX - minX) / cell);
    let rows = Math.ceil((maxY - minY) / cell);
    // Cap total cells so a huge canvas can't produce a pathological grid.
    const MAX_CELLS = 250000;
    if (cols * rows > MAX_CELLS) { _routeCtx = null; return null; }
    cols = Math.max(1, cols); rows = Math.max(1, rows);
    const blocked = new Uint8Array(cols * rows);
    const boxCells = new Map();
    boxes.forEach(b => {
      const c0 = Math.max(0, Math.floor((b.x - minX) / cell));
      const c1 = Math.min(cols - 1, Math.floor((b.x + b.w - minX) / cell));
      const r0 = Math.max(0, Math.floor((b.y - minY) / cell));
      const r1 = Math.min(rows - 1, Math.floor((b.y + b.h - minY) / cell));
      boxCells.set(b.id, { c0, c1, r0, r1 });
      for (let r = r0; r <= r1; r++)
        for (let c = c0; c <= c1; c++) blocked[r * cols + c] = 1;
    });
    // Parallel offsets for multi-edges between the same component pair.
    const groups = new Map();
    state.connections.forEach(cn => {
      const a = cn.from.compId, b = cn.to.compId;
      const key = a < b ? a + '_' + b : b + '_' + a;
      if (!groups.has(key)) groups.set(key, []);
      groups.get(key).push(cn.id);
    });
    const offsetOf = new Map();
    groups.forEach(ids => {
      const n = ids.length;
      ids.forEach((id, i) => offsetOf.set(id, (i - (n - 1) / 2) * 9));
    });
    _routeCtx = { minX, minY, cell, cols, rows, blocked, boxCells, offsetOf };
    return _routeCtx;
  }

  // 4-connected A* (Manhattan) with a turn penalty.  Returns a simplified world
  // polyline avoiding obstacle cells, or null (caller falls back to orthogonal).
  function routeAvoid(conn, p1, p2, d1, d2, ctx) {
    if (!ctx) return null;
    const { minX, minY, cell, cols, rows, blocked } = ctx;
    const clamp = (v, hi) => Math.min(hi, Math.max(0, v));
    const toCol = (x) => clamp(Math.floor((x - minX) / cell), cols - 1);
    const toRow = (y) => clamp(Math.floor((y - minY) / cell), rows - 1);
    const fromR = ctx.boxCells.get(conn.from.compId);
    const toR = ctx.boxCells.get(conn.to.compId);
    const inR = (c, r, R) => R && c >= R.c0 && c <= R.c1 && r >= R.r0 && r <= R.r1;
    const idx = (c, r) => r * cols + c;
    const free = (c, r) => {
      if (c < 0 || r < 0 || c >= cols || r >= rows) return false;
      if (!blocked[idx(c, r)]) return true;
      return inR(c, r, fromR) || inR(c, r, toR); // may exit own endpoints
    };
    const sCol = toCol(p1.x), sRow = toRow(p1.y);
    const gCol = toCol(p2.x), gRow = toRow(p2.y);
    const H = (c, r) => Math.abs(c - gCol) + Math.abs(r - gRow);
    const gScore = new Map(), came = new Map(), dirMap = new Map();
    const sIdx = idx(sCol, sRow);
    gScore.set(sIdx, 0);
    const open = [{ f: H(sCol, sRow), c: sCol, r: sRow }];
    const neigh = [[1, 0], [-1, 0], [0, 1], [0, -1]];
    const MAX = 8000;
    let expansions = 0, foundIdx = -1;
    while (open.length && expansions < MAX) {
      let bi = 0;
      for (let i = 1; i < open.length; i++) if (open[i].f < open[bi].f) bi = i;
      const cur = open.splice(bi, 1)[0];
      expansions++;
      const ci = idx(cur.c, cur.r);
      if (cur.c === gCol && cur.r === gRow) { foundIdx = ci; break; }
      const cg = gScore.get(ci);
      const pdir = dirMap.get(ci);
      for (const [dx, dy] of neigh) {
        const nc = cur.c + dx, nr = cur.r + dy;
        if (!free(nc, nr)) continue;
        const ni = idx(nc, nr);
        const ndir = dx === 1 ? 0 : dx === -1 ? 2 : dy === 1 ? 1 : 3;
        let step = 1;
        if (pdir !== undefined && pdir !== ndir) step += 3; // discourage turns
        const ng = cg + step;
        if (ng < (gScore.has(ni) ? gScore.get(ni) : Infinity)) {
          gScore.set(ni, ng); came.set(ni, ci); dirMap.set(ni, ndir);
          open.push({ f: ng + H(nc, nr), c: nc, r: nr });
        }
      }
    }
    if (foundIdx < 0) return null;
    const cells = [];
    let ci = foundIdx;
    while (ci !== undefined) { cells.push(ci); ci = came.get(ci); }
    cells.reverse();
    const cx = (c) => minX + c * cell + cell / 2;
    const cy = (r) => minY + r * cell + cell / 2;
    const pts = [{ x: p1.x, y: p1.y }];
    cells.forEach(i => pts.push({ x: cx(i % cols), y: cy(Math.floor(i / cols)) }));
    pts.push({ x: p2.x, y: p2.y });
    return simplifyPoints(pts);
  }

  // Rebuild routing + re-render every connection (the 整理连线 button).  Drops
  // any user waypoints so the auto-router fully takes over.
  function rerouteConnections() {
    _routeCtx = state.connectionStyle === 'avoid' ? buildRouteContext() : null;
    state.connections.forEach(conn => {
      if (conn.geom) conn.geom.userPoints = null;
      rerenderConnection(conn);
    });
    if (_vizMode !== 'off' && _lastPfResult) applyVisualizationOverlay();
  }

  function setConnectionStyle(style) {
    if (!['straight', 'orthogonal', 'avoid'].includes(style)) style = 'orthogonal';
    state.connectionStyle = style;
    try { localStorage.setItem('connectionStyle', style); } catch (e) { /* ignore */ }
    rerouteConnections();
  }

  function setAlignSnap(on) { state.alignSnap = !!on; }

  // ---- Phase 4: drag alignment snapping (doc §18) ----------------------------
  function clearAlignGuides() {
    if (tempLayer) tempLayer.querySelectorAll('.align-guide').forEach(e => e.remove());
  }

  function drawAlignGuide(x, y) {
    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.classList.add('align-guide');
    if (x !== null) {
      line.setAttribute('x1', x); line.setAttribute('x2', x);
      line.setAttribute('y1', viewBox.y); line.setAttribute('y2', viewBox.y + viewBox.h);
    } else {
      line.setAttribute('y1', y); line.setAttribute('y2', y);
      line.setAttribute('x1', viewBox.x); line.setAttribute('x2', viewBox.x + viewBox.w);
    }
    tempLayer.appendChild(line);
  }

  // Snap a dragged component's x/y onto the nearest neighbour within threshold
  // and show alignment guides.  Mutates comp.x/comp.y in place.
  function applyDragSnap(comp) {
    clearAlignGuides();
    const TH = 8;
    let sx = null, sy = null;
    for (const o of state.components) {
      if (o.id === comp.id) continue;
      if (sx === null && Math.abs(o.x - comp.x) <= TH) { comp.x = o.x; sx = o.x; }
      if (sy === null && Math.abs(o.y - comp.y) <= TH) { comp.y = o.y; sy = o.y; }
      if (sx !== null && sy !== null) break;
    }
    if (sx !== null) drawAlignGuide(sx, null);
    if (sy !== null) drawAlignGuide(null, sy);
  }

  // ---- Phase 4: result-label collision avoidance (doc §18) -------------------
  function deOverlapLabels(container) {
    const labels = [...container.querySelectorAll('.flow-label, .heatmap-label')];
    if (labels.length < 2) return;
    const items = [];
    labels.forEach(l => { try { items.push({ l, b: l.getBBox() }); } catch (e) { /* not rendered */ } });
    for (let pass = 0; pass < 3; pass++) {
      let moved = false;
      for (let i = 0; i < items.length; i++) {
        for (let j = i + 1; j < items.length; j++) {
          if (rectsOverlap(items[i].b, items[j].b)) {
            const t = items[j].l;
            const shift = (items[i].b.y <= items[j].b.y ? 1 : -1) * 7;
            t.setAttribute('y', parseFloat(t.getAttribute('y') || 0) + shift);
            try { items[j].b = t.getBBox(); } catch (e) { /* ignore */ }
            moved = true;
          }
        }
      }
      if (!moved) break;
    }
  }

  // ========== Connections ==========
  // Electrical domain of a component: 'converter' bridges AC and DC and may
  // connect to either side; 'dc' for DC buses/devices; 'ac' otherwise.
  function componentDomain(comp) {
    if (!comp) return null;
    const t = comp.type;
    if (t === 'vsc_converter' || t === 'dcdc_converter' || t === 'energy_router') return 'converter';
    if (t.startsWith('dc_')) return 'dc';
    return 'ac';
  }

  // An AC component must not wire DIRECTLY to a DC component — power must pass
  // through a converter (VSC / DC-DC). Returns true when the connection is
  // illegal (opposite pure domains, neither side a converter).
  function isCrossDomainConnection(fromCompId, toCompId) {
    const a = componentDomain(getComponent(fromCompId));
    const b = componentDomain(getComponent(toCompId));
    if (!a || !b) return false;
    if (a === 'converter' || b === 'converter') return false;
    return a !== b;  // 'ac' vs 'dc'
  }

  function addConnection(fromCompId, fromPortId, toCompId, toPortId) {
    fromPortId = normalizePortId(fromCompId, fromPortId);
    toPortId = normalizePortId(toCompId, toPortId);

    // Check if connection already exists
    const exists = state.connections.some(c =>
      (c.from.compId === fromCompId && c.from.portId === fromPortId &&
       c.to.compId === toCompId && c.to.portId === toPortId) ||
      (c.from.compId === toCompId && c.from.portId === toPortId &&
       c.to.compId === fromCompId && c.to.portId === fromPortId)
    );
    if (exists) return null;

    const id = 'conn_' + state.nextId++;
    const conn = {
      id,
      from: { compId: fromCompId, portId: fromPortId },
      to: { compId: toCompId, portId: toPortId },
      el: null
    };
    state.connections.push(conn);
    renderConnection(conn);
    updateInfo();
    return conn;
  }

  function renderConnection(conn, opts = {}) {
    // Route the wire (straight / orthogonal / avoid) and cache its geometry so
    // the flow-overlay code can read endpoints/midpoint without touching the DOM.
    let points = conn.geom && conn.geom.userPoints
      ? conn.geom.userPoints                       // user-edited waypoints win
      : routeConnection(conn, opts);
    if (!points || points.length < 2) return;
    conn.geom = {
      points,
      p1: points[0],
      p2: points[points.length - 1],
      userPoints: conn.geom && conn.geom.userPoints || null,
    };
    const d = pointsToPath(points);

    const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
    g.classList.add('connection');
    g.dataset.connId = conn.id;

    // Wide invisible hit area for easier clicking
    const hit = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    hit.setAttribute('d', d);
    hit.setAttribute('fill', 'none');
    hit.setAttribute('stroke', 'transparent');
    hit.setAttribute('stroke-width', '12');
    hit.style.cursor = 'pointer';
    g.appendChild(hit);

    const line = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    line.setAttribute('d', d);
    line.setAttribute('fill', 'none');
    line.setAttribute('stroke', '#666');
    line.setAttribute('stroke-width', '2');
    line.classList.add('conn-line');
    g.appendChild(line);

    connectionsLayer.appendChild(g);
    conn.el = g;
    if (conn.id === state.selectedConnectionId) g.classList.add('conn-selected');
  }

  function rerenderConnection(conn, opts = {}) {
    conn.el?.remove();
    renderConnection(conn, opts);
  }

  // ========== Event: Mouse ==========
  function onMouseDown(e) {
    const pt = screenToSvg(e.clientX, e.clientY);

    // Check if clicking on a port (for connection mode or starting connection)
    const portEl = e.target.closest('.port');
    if (portEl && (state.mode === 'connect' || state.mode === 'select')) {
      const compId = parseInt(portEl.dataset.compId);
      const portId = portEl.dataset.portId;
      const pos = getPortWorldPos(compId, portId);
      state.connectStart = { compId, portId, x: pos.x, y: pos.y };
      // Create temp line
      state.tempLine = document.createElementNS('http://www.w3.org/2000/svg', 'line');
      state.tempLine.setAttribute('x1', pos.x);
      state.tempLine.setAttribute('y1', pos.y);
      state.tempLine.setAttribute('x2', pos.x);
      state.tempLine.setAttribute('y2', pos.y);
      state.tempLine.setAttribute('stroke', '#61afef');
      state.tempLine.setAttribute('stroke-width', '2');
      state.tempLine.setAttribute('stroke-dasharray', '6 3');
      tempLayer.appendChild(state.tempLine);
      e.preventDefault();
      return;
    }

    // Check if clicking on a component
    const compEl = e.target.closest('.component');
    if (compEl && state.mode === 'select') {
      const compId = parseInt(compEl.dataset.compId);
      selectComponent(compId);
      // Start dragging
      const comp = getComponent(compId);
      if (comp) {
        state.isDragging = true;
        state.dragTarget = comp;
        state.dragOffset = { x: pt.x - comp.x, y: pt.y - comp.y };
      }
      e.preventDefault();
      return;
    }

    // Place mode: place a component
    if (state.mode === 'place' && state.placeType) {
      const comp = addComponent(state.placeType, pt.x, pt.y);
      selectComponent(comp.id);
      // Stay in place mode for multiple placement
      e.preventDefault();
      return;
    }

    // Click on empty area: deselect
    if (state.mode === 'select' && !compEl && !portEl) {
      // Check if clicking on a connection line
      const connEl = e.target.closest('.connection');
      if (connEl) {
        const connId = connEl.dataset.connId;
        selectConnection(connId);
        e.preventDefault();
        return;
      }
      selectComponent(null);
      selectConnection(null);
    }

    // Box selection: left click on empty area (not Ctrl, not middle button)
    if (e.button === 0 && !e.ctrlKey && !compEl && !portEl && state.mode === 'select') {
      state.isBoxSelecting = true;
      state.boxSelectStart = pt;
      state.boxSelectRect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
      state.boxSelectRect.setAttribute('x', pt.x);
      state.boxSelectRect.setAttribute('y', pt.y);
      state.boxSelectRect.setAttribute('width', 0);
      state.boxSelectRect.setAttribute('height', 0);
      state.boxSelectRect.setAttribute('fill', 'rgba(97,175,239,0.12)');
      state.boxSelectRect.setAttribute('stroke', '#61afef');
      state.boxSelectRect.setAttribute('stroke-width', '1');
      state.boxSelectRect.setAttribute('stroke-dasharray', '6 3');
      tempLayer.appendChild(state.boxSelectRect);
      e.preventDefault();
      return;
    }

    // Pan: middle mouse button or Ctrl+left
    if (e.button === 1 || (e.button === 0 && e.ctrlKey)) {
      state.isPanning = true;
      state.panStart = { x: e.clientX, y: e.clientY, vbx: viewBox.x, vby: viewBox.y };
      svg.style.cursor = 'grabbing';
      e.preventDefault();
    }
  }

  function onMouseMove(e) {
    const pt = screenToSvg(e.clientX, e.clientY);

    // Dragging component
    if (state.isDragging && state.dragTarget) {
      const comp = state.dragTarget;
      comp.x = snapToGrid(pt.x - state.dragOffset.x);
      comp.y = snapToGrid(pt.y - state.dragOffset.y);
      // Alignment snapping to nearby components (doc §18 Phase 4) — shows guides
      if (state.alignSnap) applyDragSnap(comp);
      comp.el.setAttribute('transform', `translate(${comp.x}, ${comp.y}) rotate(${comp.rotation || 0})`);
      // Update connections — cheap (orthogonal) routing keeps the drag smooth;
      // a full avoid-route runs once on mouse-up.
      state.connections.forEach(conn => {
        if (conn.from.compId === comp.id || conn.to.compId === comp.id) {
          rerenderConnection(conn, { cheap: true });
        }
      });
      // Update result overlays (voltage text + visualization)
      updateResultsOnDrag(comp.id);
      return;
    }

    // Box selection
    if (state.isBoxSelecting && state.boxSelectRect && state.boxSelectStart) {
      const x = Math.min(pt.x, state.boxSelectStart.x);
      const y = Math.min(pt.y, state.boxSelectStart.y);
      const w = Math.abs(pt.x - state.boxSelectStart.x);
      const h = Math.abs(pt.y - state.boxSelectStart.y);
      state.boxSelectRect.setAttribute('x', x);
      state.boxSelectRect.setAttribute('y', y);
      state.boxSelectRect.setAttribute('width', w);
      state.boxSelectRect.setAttribute('height', h);
      return;
    }

    // Drawing connection
    if (state.connectStart && state.tempLine) {
      state.tempLine.setAttribute('x2', pt.x);
      state.tempLine.setAttribute('y2', pt.y);
      return;
    }

    // Panning
    if (state.isPanning && state.panStart) {
      const rect = svg.getBoundingClientRect();
      const dx = (e.clientX - state.panStart.x) / rect.width * viewBox.w;
      const dy = (e.clientY - state.panStart.y) / rect.height * viewBox.h;
      viewBox.x = state.panStart.vbx - dx;
      viewBox.y = state.panStart.vby - dy;
      updateViewBox();
    }
  }

  function onMouseUp(e) {
    // Complete box selection
    if (state.isBoxSelecting && state.boxSelectRect && state.boxSelectStart) {
      const pt = screenToSvg(e.clientX, e.clientY);
      const x1 = Math.min(pt.x, state.boxSelectStart.x);
      const y1 = Math.min(pt.y, state.boxSelectStart.y);
      const x2 = Math.max(pt.x, state.boxSelectStart.x);
      const y2 = Math.max(pt.y, state.boxSelectStart.y);
      const w = x2 - x1;
      const h = y2 - y1;

      state.boxSelectRect.remove();
      state.boxSelectRect = null;
      state.isBoxSelecting = false;
      state.boxSelectStart = null;

      // Only select if the box is large enough (avoid accidental clicks)
      if (w > 10 || h > 10) {
        const ids = [];
        state.components.forEach(comp => {
          if (comp.x >= x1 && comp.x <= x2 && comp.y >= y1 && comp.y <= y2) {
            ids.push(comp.id);
          }
        });
        if (ids.length > 0) {
          selectMultiple(ids);
          updateInfo();
        }
      }
      return;
    }

    // Complete connection
    if (state.connectStart && state.tempLine) {
      const portEl = e.target.closest('.port');
      if (portEl) {
        const toCompId = parseInt(portEl.dataset.compId);
        const toPortId = portEl.dataset.portId;
        if (toCompId !== state.connectStart.compId) {
          if (isCrossDomainConnection(state.connectStart.compId, toCompId)) {
            // AC↔DC must go through a converter — block the wire and warn.
            const msg = 'AC 元件不能直接连接 DC 元件，请通过换流器 (VSC / DC-DC) 连接';
            if (typeof App !== 'undefined' && App.log) App.log(msg, 'warn');
            if (typeof App !== 'undefined' && App.setStatus) App.setStatus('非法连接：AC↔DC', 'error');
          } else {
            addConnection(state.connectStart.compId, state.connectStart.portId,
                          toCompId, toPortId);
            if (typeof App !== 'undefined') App.onTopologyChanged();
          }
        }
      }
      state.tempLine.remove();
      state.tempLine = null;
      state.connectStart = null;
    }

    // Stop dragging
    if (state.isDragging) {
      const dragged = state.dragTarget;
      state.isDragging = false;
      state.dragTarget = null;
      state.dragOffset = null;
      clearAlignGuides();
      // Full (avoid-aware) re-route of the dragged component's wires now that
      // the drag is over — live drag used the cheap orthogonal route.
      if (dragged && state.connectionStyle === 'avoid') {
        buildRouteContext();
        state.connections.forEach(conn => {
          if (conn.from.compId === dragged.id || conn.to.compId === dragged.id) {
            rerenderConnection(conn);
          }
        });
      }
      // Final refresh of visualization overlay after drag ends
      if (_vizMode !== 'off' && _lastPfResult) {
        applyVisualizationOverlay();
      }
    }

    // Stop panning
    if (state.isPanning) {
      state.isPanning = false;
      state.panStart = null;
      svg.style.cursor = '';
    }
  }

  function onWheel(e) {
    e.preventDefault();
    const factor = e.deltaY > 0 ? 1.1 : 0.9;
    const pt = screenToSvg(e.clientX, e.clientY);

    const newW = viewBox.w * factor;
    const newH = viewBox.h * factor;
    // Limit zoom: min width 200 (max zoom in), max width 30000 (max zoom out)
    if (newW < 200 || newW > 30000) return;

    viewBox.x = pt.x - (pt.x - viewBox.x) * factor;
    viewBox.y = pt.y - (pt.y - viewBox.y) * factor;
    viewBox.w = newW;
    viewBox.h = newH;
    state.zoom = 1200 / viewBox.w;
    updateViewBox();
  }

  function onDblClick(e) {
    const compEl = e.target.closest('.component');
    if (compEl) {
      const compId = parseInt(compEl.dataset.compId);
      selectComponent(compId);
      // Switch to properties tab
      if (typeof App !== 'undefined') App.switchTab('properties');
    }
  }

  function onKeyDown(e) {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA') return;

    if (e.key === 'Delete' || e.key === 'Backspace') {
      if (state.selectedConnectionId) {
        removeConnection(state.selectedConnectionId);
        if (typeof App !== 'undefined') App.onTopologyChanged();
      } else if (state.selectedIds.size > 1) {
        removeSelected();
      } else if (state.selectedId !== null) {
        removeComponent(state.selectedId);
        if (typeof App !== 'undefined') App.onTopologyChanged();
      }
    } else if (e.key === 'Escape') {
      setMode('select');
      selectComponent(null);
    } else if (e.key === 'a' && (e.ctrlKey || e.metaKey)) {
      // Ctrl+A: select all
      e.preventDefault();
      const allIds = state.components.map(c => c.id);
      if (allIds.length > 0) selectMultiple(allIds);
    } else if (e.key === 'v' || e.key === 'V') {
      setMode('select');
    } else if (e.key === 'c' || e.key === 'C') {
      setMode('connect');
    } else if (e.key === 'r' || e.key === 'R') {
      rotateSelected(e.shiftKey ? -90 : 90);
    }
  }

  // ========== Rotation ==========
  function rotateSelected(angleDeg) {
    if (state.selectedId === null) return;
    const comp = getComponent(state.selectedId);
    if (!comp) return;
    comp.rotation = ((comp.rotation || 0) + angleDeg + 360) % 360;
    rerenderComponent(comp);
  }

  // ========== Mode ==========
  function setMode(mode, placeType = null) {
    state.mode = mode;
    state.placeType = placeType;

    // Update toolbar buttons
    document.querySelectorAll('.canvas-tool').forEach(b => b.classList.remove('active'));
    if (mode === 'select') document.getElementById('btnSelect')?.classList.add('active');
    if (mode === 'connect') document.getElementById('btnConnect')?.classList.add('active');

    // Update cursor
    if (mode === 'place') svg.style.cursor = 'crosshair';
    else if (mode === 'connect') svg.style.cursor = 'crosshair';
    else svg.style.cursor = '';
  }

  // ========== Zoom Controls ==========
  function zoomIn() {
    const cx = viewBox.x + viewBox.w / 2;
    const cy = viewBox.y + viewBox.h / 2;
    viewBox.w *= 0.8;
    viewBox.h *= 0.8;
    viewBox.x = cx - viewBox.w / 2;
    viewBox.y = cy - viewBox.h / 2;
    state.zoom = 1200 / viewBox.w;
    updateViewBox();
  }

  function zoomOut() {
    const cx = viewBox.x + viewBox.w / 2;
    const cy = viewBox.y + viewBox.h / 2;
    viewBox.w *= 1.25;
    viewBox.h *= 1.25;
    viewBox.x = cx - viewBox.w / 2;
    viewBox.y = cy - viewBox.h / 2;
    state.zoom = 1200 / viewBox.w;
    updateViewBox();
  }

  function zoomFit() {
    if (state.components.length === 0) {
      viewBox = { x: -200, y: -100, w: 1200, h: 700 };
    } else {
      let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      state.components.forEach(c => {
        minX = Math.min(minX, c.x - 60);
        minY = Math.min(minY, c.y - 60);
        maxX = Math.max(maxX, c.x + 60);
        maxY = Math.max(maxY, c.y + 60);
      });
      const w = Math.max(maxX - minX + 100, 400);
      const h = Math.max(maxY - minY + 100, 300);
      viewBox = { x: minX - 50, y: minY - 50, w, h };
    }
    state.zoom = 1200 / viewBox.w;
    updateViewBox();
  }

  // ========== Auto-layout geometry helpers ==========
  const BUS_TYPES = new Set(['ac_bus', 'dc_bus']);
  const INLINE_LAYOUT_TYPES = new Set([
    'ac_branch', 'dc_branch', 'transformer_2w', 'transformer_3w',
    'switch_comp', 'circuit_breaker', 'vsc_converter', 'dcdc_converter',
    'energy_router'
  ]);

  function isBusType(type) {
    return BUS_TYPES.has(type);
  }

  function isInlineLayoutType(type) {
    return INLINE_LAYOUT_TYPES.has(type);
  }

  function applyComponentTransform(comp) {
    if (comp?.el) {
      comp.el.setAttribute('transform', `translate(${comp.x}, ${comp.y}) rotate(${comp.rotation || 0})`);
    }
  }

  function layoutFootprint(comp, pad = 14) {
    const isBus = isBusType(comp.type);
    const isWide = comp.type === 'ac_branch' || comp.type === 'dc_branch' ||
                   comp.type === 'switch_comp' || comp.type === 'circuit_breaker';
    const hw = isBus ? 58 : (isWide ? 62 : 52);
    const top = isBus ? 30 : 50;
    const bottom = isBus ? 34 : 62;
    return {
      id: comp.id,
      x: comp.x - hw - pad,
      y: comp.y - top - pad,
      w: 2 * hw + 2 * pad,
      h: top + bottom + 2 * pad,
    };
  }

  function layoutRectOverlap(a, b, clearance = 0) {
    return a.x < b.x + b.w + clearance && a.x + a.w + clearance > b.x &&
           a.y < b.y + b.h + clearance && a.y + a.h + clearance > b.y;
  }

  function countLayoutOverlaps(rects, clearance = 0) {
    let count = 0;
    for (let i = 0; i < rects.length; i++)
      for (let j = i + 1; j < rects.length; j++)
        if (layoutRectOverlap(rects[i], rects[j], clearance)) count++;
    return count;
  }

  function rankFreeGridPosition(comp, targetX, targetY, occupied, opts = {}) {
    const grid = opts.grid || 20;
    const maxRing = opts.maxRing || 10;
    const minDist = opts.minDist || 0;
    const clearance = opts.clearance || 6;
    const anchor = opts.anchor || null;
    const baseX = snapToGrid(targetX, grid);
    const baseY = snapToGrid(targetY, grid);
    let best = null;
    const testAt = (x, y, ring) => {
      if (anchor && Math.hypot(x - anchor.x, y - anchor.y) < minDist) return;
      const oldX = comp.x, oldY = comp.y;
      comp.x = x; comp.y = y;
      const box = layoutFootprint(comp);
      comp.x = oldX; comp.y = oldY;
      let overlaps = 0;
      for (const o of occupied) if (layoutRectOverlap(box, o, clearance)) overlaps++;
      const dist = Math.hypot(x - targetX, y - targetY);
      const score = overlaps * 100000 + dist + ring * 2;
      if (!best || score < best.score) best = { x, y, score, overlaps };
    };

    testAt(baseX, baseY, 0);
    for (let ring = 1; ring <= maxRing; ring++) {
      for (let dx = -ring; dx <= ring; dx++) {
        testAt(baseX + dx * grid, baseY - ring * grid, ring);
        testAt(baseX + dx * grid, baseY + ring * grid, ring);
      }
      for (let dy = -ring + 1; dy <= ring - 1; dy++) {
        testAt(baseX - ring * grid, baseY + dy * grid, ring);
        testAt(baseX + ring * grid, baseY + dy * grid, ring);
      }
      if (best && best.overlaps === 0) break;
    }
    return best || { x: baseX, y: baseY, overlaps: 0 };
  }

  function placeWithOccupancy(comp, targetX, targetY, occupied, opts = {}) {
    const pos = rankFreeGridPosition(comp, targetX, targetY, occupied, opts);
    comp.x = pos.x;
    comp.y = pos.y;
    occupied.push(layoutFootprint(comp));
    return pos;
  }

  function layoutCollisionCount(comp, occupied, clearance = 2) {
    const box = layoutFootprint(comp);
    let count = 0;
    for (const other of occupied) {
      if (other.id !== comp.id && layoutRectOverlap(box, other, clearance)) count++;
    }
    return count;
  }

  function refineLayoutCollisions(movable, allComps, opts = {}) {
    if (allComps.length > (opts.maxComponents || 1200)) return { moved: 0, skipped: true };
    const clearance = opts.clearance || 8;
    const passes = opts.passes || 4;
    const maxRing = opts.maxRing || 32;
    let moved = 0;
    for (let pass = 0; pass < passes; pass++) {
      let changed = false;
      for (const comp of movable) {
        const occupied = allComps.filter(c => c.id !== comp.id).map(c => layoutFootprint(c));
        const before = layoutCollisionCount(comp, occupied, clearance);
        if (before === 0) continue;
        const oldX = comp.x, oldY = comp.y;
        const pos = rankFreeGridPosition(comp, comp.x, comp.y, occupied, {
          grid: 20,
          maxRing,
          clearance,
        });
        comp.x = pos.x;
        comp.y = pos.y;
        const after = layoutCollisionCount(comp, occupied, clearance);
        if (after > before) {
          comp.x = oldX;
          comp.y = oldY;
          continue;
        }
        if (Math.abs(comp.x - oldX) > 0.5 || Math.abs(comp.y - oldY) > 0.5) {
          applyComponentTransform(comp);
          changed = true;
          moved++;
        }
      }
      if (!changed) break;
    }
    return { moved, skipped: false };
  }

  function connectionBusNeighbors(compId, busIdSet, busNeighborsByComp) {
    return [...new Set(busNeighborsByComp.get(compId) || [])].filter(id => busIdSet.has(id));
  }

  function layoutAttachedDevice(comp, bus, slot, dir, nodeGap, levelGap) {
    const rowGap = Math.max(74, Math.min(100, Math.round(levelGap * 0.36)));
    const colGap = Math.max(84, Math.min(118, Math.round(nodeGap * 0.45)));
    const baseRadial = Math.max(112, Math.min(145, Math.round(levelGap * 0.48)));
    const sideRadial = Math.max(120, Math.min(155, Math.round(nodeGap * 0.62)));
    switch (slot.dir) {
      case 'up': {
        const col = slot.n % 5, row = Math.floor(slot.n / 5);
        return { x: bus.x + (col - 2) * colGap, y: bus.y - baseRadial - row * rowGap };
      }
      case 'left':
        return { x: bus.x - sideRadial - Math.floor(slot.n / 4) * colGap,
                 y: bus.y + 24 + (slot.n % 4 - 1.5) * rowGap };
      case 'right':
        return { x: bus.x + sideRadial + Math.floor(slot.n / 4) * colGap,
                 y: bus.y + 24 + (slot.n % 4 - 1.5) * rowGap };
      case 'dl': {
        const col = slot.n % 3, row = Math.floor(slot.n / 3);
        return { x: bus.x - sideRadial - (2 - col) * colGap * 0.72,
                 y: bus.y + baseRadial + row * rowGap };
      }
      case 'dr': {
        const col = slot.n % 3, row = Math.floor(slot.n / 3);
        return { x: bus.x + sideRadial + col * colGap * 0.72,
                 y: bus.y + baseRadial + row * rowGap };
      }
      default: {
        const col = slot.n % 5, row = Math.floor(slot.n / 5);
        return { x: bus.x + (col - 2) * colGap,
                 y: bus.y + baseRadial + row * rowGap };
      }
    }
  }

  function resolveInlinePorts(comp, linked, busParent) {
    if (linked.length < 2) return null;
    const a = linked[0], b = linked[1];
    const aParentB = busParent.get(a.id) === b.id;
    const bParentA = busParent.get(b.id) === a.id;
    const parentFirst = bParentA;
    const first = parentFirst ? b : a;
    const second = parentFirst ? a : b;
    return { first, second };
  }

  // ========== Auto Layout ==========
  function autoLayout(options = {}) {
    const buses = state.components.filter(c => c.type === 'ac_bus' || c.type === 'dc_bus');
    const branches = state.components.filter(c => isInlineLayoutType(c.type));
    const devices = state.components.filter(c => !isBusType(c.type) && !isInlineLayoutType(c.type));

    if (buses.length === 0) { zoomFit(); return; }

    // Build adjacency between buses via branch-type components.
    const busIdSet = new Set(buses.map(b => b.id));

    // ---- Connection indices (built ONCE, O(connections)) -----------------
    // Large systems (>10k buses) used to be O(branches × connections) because
    // every branch re-scanned the full connection list three separate times
    // (adjacency, ring detection, branch placement) and every device scanned it
    // again. Precompute, per non-bus component, the bus ids it touches, so each
    // later pass is O(n) over its own list instead of O(n × connections).
    const busNeighborsByComp = new Map(); // non-bus compId -> [busCompId,...]
    state.connections.forEach(c => {
      const a = c.from.compId, b = c.to.compId;
      const aBus = busIdSet.has(a), bBus = busIdSet.has(b);
      if (aBus !== bBus) { // exactly one endpoint is a bus
        const busId = aBus ? a : b;
        const compId = aBus ? b : a;
        let arr = busNeighborsByComp.get(compId);
        if (!arr) { arr = []; busNeighborsByComp.set(compId, arr); }
        arr.push(busId);
      }
    });
    const busesOfBranch = (brId) => connectionBusNeighbors(brId, busIdSet, busNeighborsByComp);
    const busLoadById = new Map(buses.map(b => [b.id, 0]));
    state.components.forEach(comp => {
      if (isBusType(comp.type)) return;
      const linked = connectionBusNeighbors(comp.id, busIdSet, busNeighborsByComp);
      if (!linked.length) return;
      const weight = isInlineLayoutType(comp.type) && linked.length >= 2 ? 0.45 : 1;
      linked.forEach(id => busLoadById.set(id, (busLoadById.get(id) || 0) + weight));
    });
    let maxLocalLoad = 0;
    busLoadById.forEach(v => { if (v > maxLocalLoad) maxLocalLoad = v; });

    const adj = new Map(); // busCompId -> Set<busCompId>
    buses.forEach(b => adj.set(b.id, new Set()));
    branches.forEach(br => {
      const linked = busesOfBranch(br.id);
      for (let i = 0; i < linked.length; i++)
        for (let j = i + 1; j < linked.length; j++) {
          adj.get(linked[i]).add(linked[j]);
          adj.get(linked[j]).add(linked[i]);
        }
    });

    // BFS layering from a root bus (prefer SLACK / external-grid-connected bus)
    const busLayer = new Map();
    let root = buses[0].id;
    // Buses backed by an external grid (precomputed once, O(devices)).
    const egBusIds = new Set();
    devices.forEach(d => {
      if (d.type === 'external_grid')
        (busNeighborsByComp.get(d.id) || []).forEach(bid => egBusIds.add(bid));
    });
    // Prefer a SLACK bus, else an external-grid-backed bus, then the highest
    // degree bus, as the layout root.
    let foundRoot = false;
    for (const b of buses) {
      if (b.params.bus_type === 'SLACK' || b.params.bus_type === 'Slack') {
        root = b.id;
        foundRoot = true;
        break;
      }
      if (!foundRoot && egBusIds.has(b.id)) {
        root = b.id;
        foundRoot = true;
      }
    }
    if (!foundRoot) {
      let bestDegree = -1;
      for (const b of buses) {
        const d = adj.get(b.id)?.size || 0;
        if (d > bestDegree) { bestDegree = d; root = b.id; }
      }
    }

    // Multi-root BFS: build a spanning tree (parent/children) per connected
    // component so the layout can centre each parent over its sub-tree.  The
    // primary root is the SLACK / external-grid bus chosen above; every other
    // disconnected bus seeds its own tree.
    const busParent = new Map();
    const visited = new Set();
    const roots = [];
    const bfsFrom = (r) => {
      roots.push(r);
      const queue = [r];
      busLayer.set(r, 0);
      busParent.set(r, null);
      visited.add(r);
      while (queue.length > 0) {
        const cur = queue.shift();
        const layer = busLayer.get(cur);
        for (const nb of adj.get(cur) || []) {
          if (!visited.has(nb)) {
            visited.add(nb);
            busLayer.set(nb, layer + 1);
            busParent.set(nb, cur);
            queue.push(nb);
          }
        }
      }
    };
    bfsFrom(root);
    buses.forEach(b => { if (!visited.has(b.id)) bfsFrom(b.id); });

    // Children (spanning-tree edges only)
    const children = new Map();
    buses.forEach(b => children.set(b.id, []));
    buses.forEach(b => {
      const p = busParent.get(b.id);
      if (p != null && children.has(p)) children.get(p).push(b.id);
    });

    // Sub-tree leaf width (post-order): a leaf spans 1 column; a parent spans
    // the sum of its children's widths.  Iterative post-order avoids deep
    // recursion blowing the stack on large radial feeders.
    const subWidth = new Map();
    roots.forEach(r => {
      const order = [];
      const stack = [r];
      while (stack.length) {
        const id = stack.pop();
        order.push(id);
        (children.get(id) || []).forEach(c => stack.push(c));
      }
      for (let i = order.length - 1; i >= 0; i--) {
        const id = order[i];
        const ch = children.get(id) || [];
        if (ch.length === 0) subWidth.set(id, 1);
        else subWidth.set(id, ch.reduce((s, c) => s + (subWidth.get(c) || 1), 0));
      }
    });

    // ---- Abstract tree positioning (depth + leaf coordinate) -------------
    // Assigns each bus a leaf coordinate (_pos); a parent is centred over its
    // children.  Iterative DFS (explicit stack) so deep feeders with thousands
    // of buses cannot overflow the call stack.  Re-runnable so the crossing
    // sweeps below can reorder siblings and recompute coordinates.
    // Multi-feeder grouping (doc §18): an extra gap is inserted between the
    // primary root's direct sub-trees so distinct feeders read as separate.
    const feederGap = 1;
    const placeSubtree = (rootId, startLeaf) => {
      const stack = [{ id: rootId, left: startLeaf, phase: 0 }];
      while (stack.length) {
        const f = stack[stack.length - 1];
        const ch = children.get(f.id) || [];
        if (ch.length === 0) { const bus = getComponent(f.id); if (bus) bus._pos = f.left; stack.pop(); continue; }
        if (f.phase === 0) {
          f.phase = 1;
          const gap = (busParent.get(f.id) == null && ch.length > 1) ? feederGap : 0;
          let cursor = f.left;
          const lefts = [];
          ch.forEach(c => { lefts.push(cursor); cursor += (subWidth.get(c) || 1) + gap; });
          for (let k = ch.length - 1; k >= 0; k--) stack.push({ id: ch[k], left: lefts[k], phase: 0 });
        } else {
          const bus = getComponent(f.id);
          const first = getComponent(ch[0]);
          const last = getComponent(ch[ch.length - 1]);
          if (bus && first && last) bus._pos = (first._pos + last._pos) / 2;
          stack.pop();
        }
      }
    };
    const assignLeafPos = () => {
      let leaf = 0;
      roots.forEach(r => { placeSubtree(r, leaf); leaf += (subWidth.get(r) || 1) + 2; });
    };
    assignLeafPos();

    // ---- Ring / loop detection (doc §18): edges not in the spanning tree mark
    // a cycle.  Their branch components are flagged so 'avoid' routing can give
    // them extra clearance; ringCount is surfaced for the status bar.
    let ringCount = 0;
    const ringBusPairs = new Set();
    buses.forEach(b => {
      (adj.get(b.id) || []).forEach(nb => {
        if (nb <= b.id) return; // count each undirected edge once
        const isTree = busParent.get(b.id) === nb || busParent.get(nb) === b.id;
        if (!isTree) { ringCount++; ringBusPairs.add(b.id + '_' + nb); }
      });
    });
    branches.forEach(br => {
      const linked = busesOfBranch(br.id);
      br._isRing = linked.length >= 2 &&
        ringBusPairs.has(Math.min(linked[0], linked[1]) + '_' + Math.max(linked[0], linked[1]));
    });
    _layoutStats = { buses: buses.length, rings: ringCount };

    // ---- Crossing reduction: barycenter sibling ordering -----------------
    // A few sweeps reorder each parent's children toward the average position
    // of their cross-links, reducing wire crossings in meshed feeders.  It is a
    // no-op for pure radial trees (already crossing-free) so it never worsens
    // the common case.
    for (let sweep = 0; sweep < 4; sweep++) {
      let changed = false;
      buses.forEach(b => {
        const ch = children.get(b.id);
        if (!ch || ch.length < 2) return;
        const bary = new Map();
        ch.forEach(cid => {
          let sum = 0, cnt = 0;
          (adj.get(cid) || []).forEach(nb => {
            const nc = getComponent(nb);
            if (nc && nc._pos !== undefined && nb !== b.id) { sum += nc._pos; cnt++; }
          });
          bary.set(cid, cnt ? sum / cnt : (getComponent(cid)._pos || 0));
        });
        const before = ch.join(',');
        ch.sort((x, y) => bary.get(x) - bary.get(y));
        if (ch.join(',') !== before) changed = true;
      });
      assignLeafPos();
      if (!changed) break;
    }

    // ---- Crossing-count refinement (doc §18) -----------------------------
    // For small/medium graphs, run extra alternating sweeps and keep whichever
    // sibling ordering yields the fewest edge crossings.  Skipped for large
    // graphs where the O(E²) count would be too costly (keeps thousands-of-node
    // layouts fast).
    const countCrossings = () => {
      // Group adjacency edges by the (parent) layer of their lower endpoint.
      const byLayer = new Map();
      buses.forEach(u => {
        (adj.get(u.id) || []).forEach(vId => {
          if (vId <= u.id) return;
          const lu = busLayer.get(u.id), lv = busLayer.get(vId);
          if (lu === undefined || lv === undefined) return;
          const L = Math.min(lu, lv);
          if (!byLayer.has(L)) byLayer.set(L, []);
          const top = lu <= lv ? u.id : vId, bot = lu <= lv ? vId : u.id;
          byLayer.get(L).push([getComponent(top)._pos || 0, getComponent(bot)._pos || 0]);
        });
      });
      let cross = 0;
      byLayer.forEach(edges => {
        for (let i = 0; i < edges.length; i++)
          for (let j = i + 1; j < edges.length; j++) {
            const a = edges[i], b = edges[j];
            if ((a[0] - b[0]) * (a[1] - b[1]) < 0) cross++;
          }
      });
      return cross;
    };
    if (buses.length <= 250) {
      let best = countCrossings();
      let bestOrder = new Map([...children].map(([k, v]) => [k, v.slice()]));
      let bestPos = new Map(buses.map(b => [b.id, b._pos]));
      for (let sweep = 0; sweep < 6 && best > 0; sweep++) {
        // Median heuristic on the alternate parity for ordering diversity.
        buses.forEach(b => {
          const ch = children.get(b.id);
          if (!ch || ch.length < 2) return;
          const med = new Map();
          ch.forEach(cid => {
            const ps = [];
            (adj.get(cid) || []).forEach(nb => {
              const nc = getComponent(nb);
              if (nc && nc._pos !== undefined && nb !== b.id) ps.push(nc._pos);
            });
            ps.sort((p, q) => p - q);
            med.set(cid, ps.length ? ps[Math.floor(ps.length / 2)] : (getComponent(cid)._pos || 0));
          });
          ch.sort((x, y) => med.get(x) - med.get(y));
        });
        assignLeafPos();
        const c = countCrossings();
        if (c < best) {
          best = c;
          bestOrder = new Map([...children].map(([k, v]) => [k, v.slice()]));
          bestPos = new Map(buses.map(b => [b.id, b._pos]));
        }
      }
      // Restore the best ordering found.
      bestOrder.forEach((v, k) => { if (children.has(k)) children.set(k, v); });
      buses.forEach(b => { b._pos = bestPos.get(b.id); });
      if (_layoutStats) _layoutStats.crossings = best;
    }

    // ---- Map abstract (leaf, depth) → screen (x, y) by direction ---------
    //   TB (default) top→bottom, LR left→right, RADIAL concentric, COMPACT = TB
    //   with tighter gaps, BUSBAR = stacked horizontal busbars (depth→y) joined
    //   by vertical lines/cables/transformers (pos→x, wider node gap).
    const dir = (options && options.direction) || 'TB';
    const densityExtra = Math.min(150, Math.max(0, Math.ceil(maxLocalLoad - 3)) * 18);
    const meshExtra = Math.min(100, ringCount * 8);
    const baseLevelGap = dir === 'COMPACT' ? 180 : dir === 'BUSBAR' ? 220 : dir === 'RADIAL' ? 250 : 250;
    const baseNodeGap = dir === 'COMPACT' ? 155 : dir === 'BUSBAR' ? 270 : 220;
    const levelGap = (options && options.levelGap) ||
      (baseLevelGap + Math.min(90, densityExtra * 0.55) + Math.min(50, meshExtra * 0.35));
    const nodeGap  = (options && options.nodeGap) ||
      (baseNodeGap + densityExtra + meshExtra);
    if (_layoutStats) {
      _layoutStats.maxLocalDevices = Number(maxLocalLoad.toFixed(1));
      _layoutStats.nodeGap = Math.round(nodeGap);
      _layoutStats.levelGap = Math.round(levelGap);
    }
    const totalLeaves = buses.reduce((m, b) => Math.max(m, b._pos || 0), 0);
    buses.forEach(b => {
      const depth = busLayer.get(b.id) || 0;
      const pos = b._pos || 0;
      if (dir === 'LR') {
        b.x = depth * levelGap;
        b.y = pos * nodeGap;
      } else if (dir === 'RADIAL') {
        const ang = (pos / (totalLeaves + 1)) * 2 * Math.PI;
        const rad = (depth + 1) * levelGap * 0.9;
        b.x = Math.cos(ang) * rad;
        b.y = Math.sin(ang) * rad;
      } else {
        b.x = pos * nodeGap;
        b.y = depth * levelGap;
      }
    });

    // Centre the whole drawing around the origin (both axes).  Uses a running
    // min/max loop rather than Math.min(...spread) — spreading a thousands-long
    // array overflows the JS argument limit (RangeError) on large systems.
    if (buses.length > 0) {
      let mnX = Infinity, mnY = Infinity, mxX = -Infinity, mxY = -Infinity;
      buses.forEach(b => {
        if (b.x < mnX) mnX = b.x; if (b.x > mxX) mxX = b.x;
        if (b.y < mnY) mnY = b.y; if (b.y > mxY) mxY = b.y;
      });
      const midX = (mnX + mxX) / 2, midY = (mnY + mxY) / 2;
      buses.forEach(b => { b.x -= midX; b.y -= midY; });
    }
    const spacingX = nodeGap, spacingY = levelGap;

    // Layer groups retained for orphan placement below.
    const layers = new Map();
    buses.forEach(b => {
      const L = busLayer.get(b.id);
      if (!layers.has(L)) layers.set(L, []);
      layers.get(L).push(b);
    });
    const sortedLayers = [...layers.keys()].sort((a, b) => a - b);

    // Snap buses to grid
    buses.forEach(bus => {
      if (dir === 'BUSBAR') bus.rotation = 0; // busbars are drawn horizontal
      bus.x = snapToGrid(bus.x);
      bus.y = snapToGrid(bus.y);
      bus.el.setAttribute('transform', `translate(${bus.x}, ${bus.y}) rotate(${bus.rotation || 0})`);
    });

    // Busbar widening: in BUSBAR mode stretch each horizontal bus bar so it spans
    // the horizontal extent of the feeders that tap it (its spanning-tree
    // children), giving a substation single-line look.  Other directions restore
    // the native ±40 bar so switching layouts stays clean.  The bar is the first
    // <line> in the bus glyph; editing x1/x2 leaves the centred label untouched.
    const BAR_HALF = 40, BAR_MARGIN = 26, BAR_HALF_MAX = nodeGap * 3;
    buses.forEach(bus => {
      const line = bus.el.querySelector('line');
      if (!line) return;
      if (dir === 'BUSBAR') {
        let minDx = -BAR_HALF, maxDx = BAR_HALF;
        (children.get(bus.id) || []).forEach(cid => {
          const c = getComponent(cid);
          if (c) { const dx = c.x - bus.x; if (dx < minDx) minDx = dx; if (dx > maxDx) maxDx = dx; }
        });
        const x1 = Math.max(minDx - BAR_MARGIN, -BAR_HALF_MAX);
        const x2 = Math.min(maxDx + BAR_MARGIN, BAR_HALF_MAX);
        line.setAttribute('x1', x1.toFixed(1));
        line.setAttribute('x2', x2.toFixed(1));
      } else {
        line.setAttribute('x1', String(-BAR_HALF));
        line.setAttribute('x2', String(BAR_HALF));
      }
    });

    // Collision-aware placement.  Buses are fixed first; all branch-like and
    // attached components reserve approximate label-aware rectangles after they
    // are placed, so later components can find a nearby free grid cell.
    const occupied = buses.map(b => layoutFootprint(b, dir === 'BUSBAR' ? 24 : 16));
    const pairSlots = new Map();
    const inlineOffset = {
      ac_branch: 42, dc_branch: 42, switch_comp: 38, circuit_breaker: 38,
      transformer_2w: 60, transformer_3w: 72, vsc_converter: 64,
      dcdc_converter: 64, energy_router: 80,
    };
    const isLineType = (t) => t === 'ac_branch' || t === 'dc_branch' ||
                            t === 'switch_comp' || t === 'circuit_breaker';
    branches.forEach(br => {
      const linked = busesOfBranch(br.id).map(getComponent).filter(Boolean);
      if (linked.length >= 2) {
        const xs = linked.map(b => b.x), ys = linked.map(b => b.y);
        let targetX = xs.reduce((s, x) => s + x, 0) / xs.length;
        let targetY = ys.reduce((s, y) => s + y, 0) / ys.length;

        if (linked.length === 2) {
          const a = linked[0], b = linked[1];
          const key = a.id < b.id ? a.id + '_' + b.id : b.id + '_' + a.id;
          const n = pairSlots.get(key) || 0;
          pairSlots.set(key, n + 1);
          const dx = b.x - a.x, dy = b.y - a.y;
          const len = Math.hypot(dx, dy) || 1;
          const laneSign = (n % 2 === 0) ? 1 : -1;
          const lane = Math.ceil((n + 1) / 2);
          const meshLane = br._isRing ? 1.55 : 1;
          const off = (inlineOffset[br.type] || 48) * lane * meshLane;
          if (dir === 'BUSBAR') {
            targetX += laneSign * off;
          } else {
            targetX += (-dy / len) * off * laneSign;
            targetY += (dx / len) * off * laneSign;
          }
        } else {
          const spread = Math.max(70, Math.min(140, linked.length * 22));
          targetY += spread;
        }

        const resolved = resolveInlinePorts(br, linked, busParent);
        if (resolved) {
          const dx = resolved.second.x - resolved.first.x;
          const dy = resolved.second.y - resolved.first.y;
          if (isLineType(br.type)) br.rotation = Math.abs(dy) > Math.abs(dx) ? 90 : 0;
          else if (br.type === 'transformer_2w' || br.type === 'transformer_3w') br.rotation = 0;
          else br.rotation = Math.abs(dy) > Math.abs(dx) ? 90 : 0;
        }
        if (dir === 'BUSBAR' && isLineType(br.type)) br.rotation = 90;

        placeWithOccupancy(br, targetX, targetY, occupied, {
          maxRing: br._isRing ? 16 : 11,
          clearance: br._isRing ? 10 : 8,
        });
      } else if (linked.length === 1) {
        const bus = linked[0];
        const side = br.type.startsWith('dc') ? 1 : -1;
        placeWithOccupancy(br, bus.x + side * 130, bus.y, occupied, {
          anchor: bus, minDist: 96, maxRing: 10, clearance: 8,
        });
      } else {
        const maxLayer = sortedLayers.length > 0 ? sortedLayers[sortedLayers.length - 1] + 1 : 0;
        const i = occupied.length;
        placeWithOccupancy(br, (i % 6) * 130, maxLayer * spacingY + 120 + Math.floor(i / 6) * 120, occupied);
      }
      applyComponentTransform(br);
    });

    // Place device components around their connected bus by semantic zone.  Each
    // candidate slot then goes through the occupancy search, so dense buses grow
    // into clean rows instead of covering labels and neighboring feeders.
    const dirSlots = new Map(); // `${busId}|${dir}` -> count already placed
    const slotOf = (busId, zone) => {
      const key = busId + '|' + zone;
      const n = dirSlots.get(key) || 0;
      dirSlots.set(key, n + 1);
      return { dir: zone, n };
    };
    const placement = {
      external_grid:    'up',
      generator:        'left',
      static_generator: 'right',
      pv_system:        'right',
      dc_pv_array:      'right',
      renewable_gen:    'right',
      vpp:              'right',
      storage:          'dr',
      dc_storage:       'dr',
      mobile_storage:   'dr',
      microgrid:        'dr',
      shunt:            'dl',
    };
    let orphanIdx = 0;
    devices.forEach(comp => {
      const busId = connectionBusNeighbors(comp.id, busIdSet, busNeighborsByComp)[0];
      if (busId !== undefined) {
        const bus = getComponent(busId);
        if (bus) {
          const zone = placement[comp.type] || 'down';
          const target = layoutAttachedDevice(comp, bus, slotOf(busId, zone), dir, nodeGap, levelGap);
          placeWithOccupancy(comp, target.x, target.y, occupied, {
            anchor: bus,
            minDist: 96,
            maxRing: Math.max(8, Math.min(18, 8 + Math.ceil((busLoadById.get(busId) || 0) / 2))),
            clearance: 8,
          });
        }
      } else {
        // No connection, place in a separate area beneath the bus layers.
        const maxLayer = sortedLayers.length > 0 ? sortedLayers[sortedLayers.length - 1] + 1 : 0;
        placeWithOccupancy(comp,
          (orphanIdx % 5) * 130,
          maxLayer * spacingY + 140 + Math.floor(orphanIdx / 5) * 120,
          occupied,
          { maxRing: 8, clearance: 8 });
        orphanIdx++;
      }
      applyComponentTransform(comp);
    });

    const movable = branches.concat(devices);
    const refine = refineLayoutCollisions(movable, state.components, {
      clearance: 8,
      passes: 5,
      maxRing: 36,
    });
    const finalRects = state.components.map(c => layoutFootprint(c));
    if (_layoutStats) {
      _layoutStats.overlaps = countLayoutOverlaps(finalRects, 2);
      _layoutStats.refinedMoves = refine.moved;
      _layoutStats.refineSkipped = refine.skipped;
    }

    // Re-render connections — rebuild the avoidance context first so wires route
    // around the freshly-placed components.
    _routeCtx = state.connectionStyle === 'avoid' ? buildRouteContext() : null;
    state.connections.forEach(rerenderConnection);
    zoomFit();
  }

  // ---- Phase 4: local re-layout of the current selection (doc §18) -----------
  // Re-runs the full topology layout but writes positions only for the selected
  // buses (and their attached devices/branches), keeping the rest of the diagram
  // anchored.  Falls back to a full autoLayout when nothing useful is selected.
  function autoLayoutSelection(options = {}) {
    const sel = new Set(state.selectedIds && state.selectedIds.size
      ? state.selectedIds
      : (state.selectedId != null ? [state.selectedId] : []));
    if (sel.size === 0) { autoLayout(options); return; }

    // Snapshot every component position, run a global layout, then revert all
    // non-selected components so only the selection is re-flowed.  Simple and
    // robust; the selection keeps the global structure but other nodes hold.
    const saved = new Map(state.components.map(c => [c.id, { x: c.x, y: c.y, rot: c.rotation || 0 }]));
    // Expand selection to include devices/branches attached to selected buses.
    state.connections.forEach(cn => {
      if (sel.has(cn.from.compId)) sel.add(cn.to.compId);
      if (sel.has(cn.to.compId)) sel.add(cn.from.compId);
    });
    autoLayout(options);
    state.components.forEach(c => {
      if (!sel.has(c.id)) {
        const s = saved.get(c.id);
        if (s) { c.x = s.x; c.y = s.y; c.rotation = s.rot;
          c.el.setAttribute('transform', `translate(${c.x}, ${c.y}) rotate(${c.rotation})`); }
      }
    });
    _routeCtx = state.connectionStyle === 'avoid' ? buildRouteContext() : null;
    state.connections.forEach(rerenderConnection);
    if (_vizMode !== 'off' && _lastPfResult) applyVisualizationOverlay();
    updateInfo();
  }

  // ========== Build JSON System ==========
  // Assign 1-based bus indices to every bus component of the given type.
  // An imported (or explicitly edited) bus keeps its own params.index when it
  // is a positive integer and not already taken; the remaining buses receive
  // the lowest free contiguous indices.  Drawn buses (no params.index) thus
  // behave exactly as before (1..N), while imported NON-contiguous ids
  // (e.g. 1-5, 10-14, 20-21) are preserved end-to-end so the canvas label, the
  // backend bus id, the analysis results and the fault-bus input all agree.
  // buildSystemJson() and getCompBusMap() MUST use this same mapping.
  function assignBusIndices(busType) {
    const map = {};
    const used = new Set();
    const pending = [];
    state.components.forEach(comp => {
      if (comp.type !== busType) return;
      const raw = comp.params ? comp.params.index : undefined;
      const idx = parseInt(raw, 10);
      if (Number.isInteger(idx) && idx > 0 && !used.has(idx)) {
        used.add(idx);
        map[comp.id] = idx;
      } else {
        pending.push(comp.id);
      }
    });
    let next = 1;
    pending.forEach(id => {
      while (used.has(next)) next++;
      used.add(next);
      map[id] = next;
    });
    return map;
  }

  // Re-derive each device's connected-bus parameters (from_bus/to_bus, hv_bus/
  // lv_bus, bus, bus_ac/bus_dc) from the LIVE wiring on the canvas. Called after
  // the user rewires connections so the property panel and exports reflect the
  // new topology instead of the stale values captured at import time. Uses the
  // same bus-index assignment as buildSystemJson()/getCompBusMap() for consistency.
  function syncConnectivity() {
    const acMap = assignBusIndices('ac_bus');
    const dcMap = assignBusIndices('dc_bus');
    const busesOf = (compId) => {
      const out = [];
      for (const conn of state.connections) {
        let other = null;
        if (conn.from.compId === compId) other = conn.to.compId;
        else if (conn.to.compId === compId) other = conn.from.compId;
        if (other === null) continue;
        if (acMap[other] != null) out.push({ domain: 'ac', index: acMap[other] });
        else if (dcMap[other] != null) out.push({ domain: 'dc', index: dcMap[other] });
      }
      return out;
    };
    // Resolve the bus wired to a specific named port (for same-domain ports that
    // busesOf() cannot disambiguate, e.g. a DC/DC's 'in'/'out').
    const busByPort = (compId, portId) => {
      for (const conn of state.connections) {
        let other = null;
        if (conn.from.compId === compId && conn.from.portId === portId) other = conn.to.compId;
        else if (conn.to.compId === compId && conn.to.portId === portId) other = conn.from.compId;
        if (other === null) continue;
        if (acMap[other] != null) return { domain: 'AC', index: acMap[other] };
        if (dcMap[other] != null) return { domain: 'DC', index: dcMap[other] };
      }
      return null;
    };
    state.components.forEach(comp => {
      const p = comp.params;
      if (!p || comp.type === 'ac_bus' || comp.type === 'dc_bus') return;
      const buses = busesOf(comp.id);
      if ('bus_in' in p || 'bus_out' in p) {
        const bi = busByPort(comp.id, 'in');
        const bo = busByPort(comp.id, 'out');
        if ('bus_in' in p && bi != null) p.bus_in = bi.index;
        if ('bus_out' in p && bo != null) p.bus_out = bo.index;
      } else if (comp.type === 'energy_router') {
        const erPortDefs = COMP.ports.energy_router || [];
        erPortDefs.forEach((def, idx) => {
          const pi = idx + 1;
          const b = busByPort(comp.id, def.id);
          if (!b) return;
          p['port' + pi + '_bus'] = b.index;
          p['port' + pi + '_type'] = b.domain;
        });
      } else if ('bus_ac' in p || 'bus_dc' in p) {
        const ac = buses.find(b => b.domain === 'ac');
        const dc = buses.find(b => b.domain === 'dc');
        if ('bus_ac' in p && ac) p.bus_ac = ac.index;
        if ('bus_dc' in p && dc) p.bus_dc = dc.index;
      } else if ('hv_bus' in p || 'lv_bus' in p) {
        if (buses.length >= 1 && 'hv_bus' in p) p.hv_bus = buses[0].index;
        if (buses.length >= 2 && 'lv_bus' in p) p.lv_bus = buses[1].index;
      } else if ('from_bus' in p || 'to_bus' in p) {
        if (buses.length >= 1 && 'from_bus' in p) p.from_bus = buses[0].index;
        if (buses.length >= 2 && 'to_bus' in p) p.to_bus = buses[1].index;
      } else if ('bus' in p) {
        if (buses.length >= 1) p.bus = buses[0].index;
      }
    });
  }

	  function buildSystemJson() {
	    const sys = {
      name: 'Canvas System',
      base_mva: state.baseMva || 100,
      ac: { buses: [], branches: [], generators: [], loads: [],
            static_generators: [], storage: [], renewable_gens: [],
            pv_systems: [], external_grids: [], transformers_2w: [],
            switches: [], circuit_breakers: [], motors: [],
            flexible_loads: [], asymmetric_loads: [], shunts: [],
            transformers_3w: [], chargers: [], charging_stations: [] },
      dc: { buses: [], branches: [], loads: [], dc_storage: [], static_generators: [], pv_arrays: [], dc_circuit_breakers: [] },
      vsc_converters: [],
      dcdc_converters: [],
      energy_routers: [],
      mobile_storage: [],
      vpps: [],
	      microgrids: []
	    };
	    if (_preservedModelBlocks.three_phase_ac) {
	      sys.three_phase_ac = cloneJsonBlock(_preservedModelBlocks.three_phase_ac);
	    }

    // Assign bus indices (1-based, matching MATPOWER/C++ convention).
    // Imported buses keep their original (possibly non-contiguous) index.
    const acBusIndexMap = assignBusIndices('ac_bus');
    const dcBusIndexMap = assignBusIndices('dc_bus');
    const compBusMap = {}; // compId -> bus index (1-based, both AC and DC)
    const compBusDomainMap = {}; // compId -> 'AC' | 'DC'

    // First pass: create buses
    state.components.forEach(comp => {
      if (comp.type === 'ac_bus') {
        const p = comp.params;
        const idx = acBusIndexMap[comp.id];
        compBusMap[comp.id] = idx;
        compBusDomainMap[comp.id] = 'AC';
        sys.ac.buses.push({
          index: idx,
          bus_type: p.bus_type || 'PQ',
          base_kv: numOr(p.base_kv, 110),
          vm_pu: numOr(p.vm_pu, 1.0),
          va_deg: numOr(p.va_deg, 0),
          pd_mw: numOr(p.pd_mw, 0),
          qd_mvar: numOr(p.qd_mvar, 0),
          vmin_pu: numOr(p.vmin_pu, 0.9),
          vmax_pu: numOr(p.vmax_pu, 1.1),
          gs_mw: numOr(p.gs_mw, 0),
          bs_mvar: numOr(p.bs_mvar, 0),
          in_service: p.in_service !== false,
          area: parseInt(p.area) || 1,
          zone: parseInt(p.zone) || 1,
          n_customers: numOr(p.n_customers, 0),
          importance: numOr(p.importance, 0),
        });
      } else if (comp.type === 'dc_bus') {
        const p = comp.params;
        const idx = dcBusIndexMap[comp.id];
        compBusMap[comp.id] = idx;
        compBusDomainMap[comp.id] = 'DC';
        sys.dc.buses.push({
          index: idx,
          bus_type: p.bus_type || 'DC_P',
          base_kv: numOr(p.base_kv, 320),
          vm_pu: numOr(p.vm_pu, 1.0),
          vmax_pu: numOr(p.vmax_pu, 1.1),
          vmin_pu: numOr(p.vmin_pu, 0.9),
          pd_mw: numOr(p.pd_mw, 0),
          in_service: p.in_service !== false,
        });
      }
    });

    // Resolve bus index from a device component. Find connected bus via connections.
    // When no wired bus is found, fall back to `fallback` — callers pass the device's
    // own stored `bus` param so a device whose wire was lost (e.g. generators added
    // programmatically to the JSON, never hand-wired on the canvas) keeps its intended
    // bus instead of silently collapsing onto bus 1.
    function findBusIndex(compId, fallback = 1) {
      for (const conn of state.connections) {
        if (conn.from.compId === compId) {
          const idx = compBusMap[conn.to.compId];
          if (idx !== undefined) return idx;
        }
        if (conn.to.compId === compId) {
          const idx = compBusMap[conn.from.compId];
          if (idx !== undefined) return idx;
        }
      }
      return fallback;  // fallback: caller's stored bus, else bus 1 (1-based)
    }

    // Find two connected bus indices (for branches, transformers)
    function findTwoBusIndices(compId) {
      const indices = [];
      for (const conn of state.connections) {
        let busCompId = null;
        if (conn.from.compId === compId) busCompId = conn.to.compId;
        if (conn.to.compId === compId) busCompId = conn.from.compId;
        if (busCompId !== null && compBusMap[busCompId] !== undefined) {
          indices.push(compBusMap[busCompId]);
        }
      }
      return indices.length >= 2 ? [indices[0], indices[1]] : [indices[0] || 1, indices[1] || 1];
    }

    // Resolve the bus index wired to a SPECIFIC named port of a device. Needed
    // for devices whose ports cannot be disambiguated by domain alone — e.g. a
    // DC/DC converter whose 'in' and 'out' ports are BOTH on the DC side, or a
    // VSC's 'ac'/'dc' ports. Returns 0 when the port is unwired.
    function findBusIndexByPort(compId, portId) {
      for (const conn of state.connections) {
        if (conn.from.compId === compId && conn.from.portId === portId) {
          const idx = compBusMap[conn.to.compId];
          if (idx !== undefined) return idx;
        }
        if (conn.to.compId === compId && conn.to.portId === portId) {
          const idx = compBusMap[conn.from.compId];
          if (idx !== undefined) return idx;
        }
      }
      return 0;
    }

    function findBusByPort(compId, portId) {
      for (const conn of state.connections) {
        let busCompId = null;
        if (conn.from.compId === compId && conn.from.portId === portId) {
          busCompId = conn.to.compId;
        } else if (conn.to.compId === compId && conn.to.portId === portId) {
          busCompId = conn.from.compId;
        }
        if (busCompId === null) continue;
        const idx = compBusMap[busCompId];
        if (idx !== undefined) {
          return { index: idx, domain: compBusDomainMap[busCompId] || 'AC' };
        }
      }
      return null;
    }

    // Second pass: create devices
    let genIdx = 0, brIdx = 0, vscIdx = 0, loadIdx = 0, trafoIdx = 0, egIdx = 0,
        storIdx = 0, pvIdx = 0, renIdx = 0, sgenIdx = 0, dcLoadIdx = 0, dcBrIdx = 0,
        swIdx = 0, cbIdx = 0, motorIdx = 0, dcPvIdx = 0, dcStorIdx = 0;
    state.components.forEach(comp => {
      const p = comp.params;
      switch (comp.type) {
        case 'generator': {
          const busIdx = findBusIndex(comp.id, numOr(p.bus, 1));
          sys.ac.generators.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : genIdx,
            name: p.name || `Gen ${genIdx}`,
            bus: busIdx,
            pg_mw: numOr(p.pg_mw, 0),
            qg_mvar: numOr(p.qg_mvar, 0),
            vg_pu: numOr(p.vg_pu, 1.0),
            pmax_mw: numOr(p.pmax_mw, 200),
            pmin_mw: numOr(p.pmin_mw, 0),
            qmax_mvar: numOr(p.qmax_mvar, 100),
            qmin_mvar: numOr(p.qmin_mvar, -100),
            mbase_mva: numOr(p.mbase_mva, 100),
            is_slack: p.is_slack === true || p.is_slack === 'true',
            in_service: p.in_service !== false,
            cost_c2: numOr(p.cost_c2, 0),
            cost_c1: numOr(p.cost_c1, 20),
            cost_c0: numOr(p.cost_c0, 0),
            emission_factor_tco2_mwh: numOr(p.emission_factor_tco2_mwh, 0),
            startup_cost: numOr(p.startup_cost, 0),
            shutdown_cost: numOr(p.shutdown_cost, 0),
            ramp_up_mw_min: numOr(p.ramp_up_mw_min, 0),
            ramp_dn_mw_min: numOr(p.ramp_dn_mw_min, 0),
          }, p));
          genIdx++;
          // Update bus type to PV or SLACK
          const bus = sys.ac.buses.find(b => b.index === busIdx);
          if (bus) {
            if (p.is_slack === true || p.is_slack === 'true') bus.bus_type = 'SLACK';
            else if (bus.bus_type === 'PQ') bus.bus_type = 'PV';
          }
          break;
        }
        case 'load': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.loads.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : loadIdx,
            name: p.name || `Load ${Number.isFinite(Number(p.index)) ? Number(p.index) : loadIdx}`,
            bus: busIdx,
            p_mw: numOr(p.p_mw, 0),
            q_mvar: numOr(p.q_mvar, 0),
            scaling: numOr(p.scaling, 1.0),
            model: p.model || 'ConstantPower',
            z_percent_p: numOr(p.z_percent_p, 0),
            i_percent_p: numOr(p.i_percent_p, 0),
            p_percent_p: numOr(p.p_percent_p, 100),
            z_percent_q: numOr(p.z_percent_q, 0),
            i_percent_q: numOr(p.i_percent_q, 0),
            p_percent_q: numOr(p.p_percent_q, 100),
            controllable: p.controllable === true || p.controllable === 'true',
            p_min_mw: numOr(p.p_min_mw, 0),
            cost_mw: numOr(p.cost_mw, 0),
            priority: p.priority || 'Medium',
            n_customers: numOr(p.n_customers, 0),
            profile_id: numOr(p.profile_id, -1),
            in_service: p.in_service !== false,
          }, p));
          loadIdx++;
          break;
        }
        case 'ac_branch': {
          const [from, to] = findTwoBusIndices(comp.id);
          sys.ac.branches.push({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : brIdx,
            name: p.name || `Line ${Number.isFinite(Number(p.index)) ? Number(p.index) : brIdx}`,
            from_bus: from, to_bus: to,
            r_pu: numOr(p.r_pu, 0.01),
            x_pu: numOr(p.x_pu, 0.1),
            b_pu: numOr(p.b_pu, 0.02),
            rate_a_mva: numOr(p.rate_a_mva, 100),
            rate_b_mva: numOr(p.rate_b_mva, 0),
            rate_c_mva: numOr(p.rate_c_mva, 0),
            length_km: numOr(p.length_km, 0),
            tap: numOr(p.tap, 1.0),
            shift_deg: numOr(p.shift_deg, 0),
            in_service: p.in_service !== false,
            n_parallel: parseInt(p.n_parallel) || 1,
            failure_rate: numOr(p.failure_rate, 0),
            mttr_hr: numOr(p.mttr_hr, 0),
          });
          brIdx++;
          break;
        }
        case 'transformer_2w': {
          const [hv, lv] = findTwoBusIndices(comp.id);
          // If this transformer was created from a MATPOWER branch (tap≠1),
          // export it back as an ac.branches entry so the solver sees it
          // with the correct branch-model parameters.
          if (p._from_branch) {
            sys.ac.branches.push({
              index: Number.isFinite(Number(p.index)) ? Number(p.index) : brIdx,
              name: p.name || `Line ${Number.isFinite(Number(p.index)) ? Number(p.index) : brIdx}`,
              from_bus: hv, to_bus: lv,
              r_pu: numOr(p.r_pu, 0.01),
              x_pu: numOr(p.x_pu, 0.1),
              b_pu: numOr(p.b_pu, 0),
              rate_a_mva: numOr(p.rate_a_mva, 100),
              rate_b_mva: numOr(p.rate_b_mva, 0),
              rate_c_mva: numOr(p.rate_c_mva, 0),
              length_km: 0,
              tap: numOr(p.tap, 1.0),
              shift_deg: numOr(p.shift_deg, 0),
              in_service: p.in_service !== false,
              n_parallel: 1,
            });
            brIdx++;
          } else {
            const trafoIndex = Number.isFinite(Number(p.index)) ? Number(p.index) : trafoIdx;
            const trafo = {
              index: trafoIndex, hv_bus: hv, lv_bus: lv,
              sn_mva: numOr(p.sn_mva, 100),
              vn_hv_kv: numOr(p.vn_hv_kv, 220),
              vn_lv_kv: numOr(p.vn_lv_kv, 110),
              vk_percent: numOr(p.vk_percent, 12),
              vkr_percent: numOr(p.vkr_percent, 0.5),
              pfe_kw: numOr(p.pfe_kw, 30),
              i0_percent: numOr(p.i0_percent, 0.1),
              shift_deg: numOr(p.shift_deg, 0),
              tap_side: numOr(p.tap_side, 0),
              tap_pos: numOr(p.tap_pos, 0),
              tap_min: numOr(p.tap_min, -8),
              tap_max: numOr(p.tap_max, 8),
              tap_neutral: numOr(p.tap_neutral, 0),
              tap_step_percent: numOr(p.tap_step_percent, 1.25),
              vector_group: p.vector_group || '',
              in_service: p.in_service !== false,
            };
            if (Number(p.source_branch_idx) > 0) {
              trafo.source_branch_idx = Number(p.source_branch_idx);
            }
            sys.ac.transformers_2w.push(trafo);
            trafoIdx++;
          }
          break;
        }
        case 'external_grid': {
          const busIdx = findBusIndex(comp.id);
          const eg = {
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : egIdx,
            name: p.name || `Grid ${egIdx}`,
            bus: busIdx,
            vm_pu: numOr(p.vm_pu, 1.05),
            va_deg: numOr(p.va_deg, 0),
            s_sc_max_mva: numOr(p.s_sc_max_mva, 10000),
            s_sc_min_mva: numOr(p.s_sc_min_mva, 8000),
            rx_max: numOr(p.rx_max, 0.1),
            rx_min: numOr(p.rx_min, 0.1),
            r_pu: numOr(p.r_pu, 0),
            x_pu: numOr(p.x_pu, 0),
            r0_pu: numOr(p.r0_pu, 0),
            x0_pu: numOr(p.x0_pu, 0),
            vn_kv: numOr(p.vn_kv, 0),
            emission_factor_tco2_mwh: numOr(p.emission_factor_tco2_mwh, 0),
            controllable: p.controllable === true || p.controllable === 'true',
            in_service: p.in_service !== false,
            cost_c2: numOr(p.cost_c2, 0),
            cost_c1: numOr(p.cost_c1, 0),
            cost_c0: numOr(p.cost_c0, 0),
            price_profile_id: numOr(p.price_profile_id, -1),
          };
          if (Array.isArray(p.emission_factor_profile_tco2_mwh)) {
            eg.emission_factor_profile_tco2_mwh =
              p.emission_factor_profile_tco2_mwh.map(Number).filter(Number.isFinite);
          }
          sys.ac.external_grids.push(addDynamicModel(eg, p));
          egIdx++;
          // Set bus as SLACK
          const bus = sys.ac.buses.find(b => b.index === busIdx);
          if (bus) bus.bus_type = 'SLACK';
          break;
        }
        case 'storage': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.storage.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : storIdx,
            name: p.name || `ESS ${Number.isFinite(Number(p.index)) ? Number(p.index) : storIdx}`,
            bus: busIdx,
            p_mw: numOr(p.p_mw, 0),
            q_mvar: numOr(p.q_mvar, 0),
            p_rated_mw: numOr(p.p_rated_mw, 10),
            e_rated_mwh: numOr(p.e_rated_mwh, 40),
            soc_init: numOr(p.soc_init, 0.5),
            soc_min: numOr(p.soc_min, 0.1),
            soc_max: numOr(p.soc_max, 0.9),
            eta_charge: numOr(p.eta_charge, 0.95),
            eta_discharge: numOr(p.eta_discharge, 0.95),
            pmax_mw: numOr(p.pmax_mw, 10),
            pmin_mw: numOr(p.pmin_mw, -10),
            qmax_mvar: numOr(p.qmax_mvar, 0),
            qmin_mvar: numOr(p.qmin_mvar, 0),
            self_discharge_pct: numOr(p.self_discharge_pct, 0),
            profile_id: numOr(p.profile_id, -1),
            in_service: p.in_service !== false,
          }, p));
          storIdx++;
          break;
        }
        case 'dc_storage': {
          const busIdx = findBusIndex(comp.id);
          sys.dc.dc_storage.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : dcStorIdx,
            name: p.name || `DC ESS ${Number.isFinite(Number(p.index)) ? Number(p.index) : dcStorIdx}`,
            bus: busIdx,
            p_mw: numOr(p.p_mw, 0),
            p_rated_mw: numOr(p.p_rated_mw, 10),
            e_rated_mwh: numOr(p.e_rated_mwh, 40),
            soc_init: numOr(p.soc_init, 0.5),
            soc_min: numOr(p.soc_min, 0.1),
            soc_max: numOr(p.soc_max, 0.9),
            eta_charge: numOr(p.eta_charge, 0.95),
            eta_discharge: numOr(p.eta_discharge, 0.95),
            pmax_mw: numOr(p.pmax_mw, 10),
            pmin_mw: numOr(p.pmin_mw, -10),
            self_discharge_pct: numOr(p.self_discharge_pct, 0),
            profile_id: numOr(p.profile_id, -1),
            in_service: p.in_service !== false,
          }, p));
          dcStorIdx++;
          break;
        }
        case 'pv_system': {
          const busIdx = findBusIndex(comp.id);
          const pvMode = p.control_mode || 'MPPT';
          const pvControllable = pvMode === 'Curtailed' ? true
            : (p.controllable === true || p.controllable === 'true');
          sys.ac.pv_systems.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : pvIdx,
            name: p.name || `PV ${Number.isFinite(Number(p.index)) ? Number(p.index) : pvIdx}`,
            bus: busIdx,
            p_mw: numOr(p.p_mw, 5),
            q_mvar: numOr(p.q_mvar, 0),
            sn_mva: numOr(p.sn_mva, 6),
            pmax_mw: numOr(p.pmax_mw, 0),
            pmin_mw: numOr(p.pmin_mw, 0),
            qmax_mvar: numOr(p.qmax_mvar, 0),
            qmin_mvar: numOr(p.qmin_mvar, 0),
            control_mode: pvMode,
            controllable: pvControllable,
            v_ac_set_pu: numOr(p.v_ac_set_pu, 1.0),
            v_dc_set_pu: numOr(p.v_dc_set_pu, 1.0),
            inverter_eff: numOr(p.inverter_eff, 0.97),
            loss_percent: numOr(p.loss_percent, 0),
            num_series: parseInt(p.num_series) || 0,
            num_parallel: parseInt(p.num_parallel) || 0,
            vmpp: numOr(p.vmpp, 0),
            impp: numOr(p.impp, 0),
            voc: numOr(p.voc, 0),
            isc: numOr(p.isc, 0),
            alpha_isc: numOr(p.alpha_isc, 0),
            beta_voc: numOr(p.beta_voc, 0),
            irradiance: numOr(p.irradiance, 1000),
            temperature: numOr(p.temperature, 25),
            profile_id: numOr(p.profile_id, -1),
            in_service: p.in_service !== false,
          }, p));
          pvIdx++;
          break;
        }
        case 'renewable_gen': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.renewable_gens.push({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : renIdx,
            name: p.name || `${p.type || 'Renewable'} ${Number.isFinite(Number(p.index)) ? Number(p.index) : renIdx}`,
            bus: busIdx, type: p.type || 'Wind',
            p_mw: numOr(p.p_mw, 20),
            q_mvar: numOr(p.q_mvar, 0),
            p_rated_mw: numOr(p.p_rated_mw, 30),
            qmax_mvar: numOr(p.qmax_mvar, 0),
            qmin_mvar: numOr(p.qmin_mvar, 0),
            curtailable: p.curtailable !== false,
            cost_curtail_mwh: numOr(p.cost_curtail_mwh, 0),
            capacity_factor: numOr(p.capacity_factor, 0.3),
            profile_id: numOr(p.profile_id, -1),
            emission_offset_tco2_mwh: numOr(p.emission_offset_tco2_mwh, 0),
            in_service: p.in_service !== false,
          });
          renIdx++;
          break;
        }
        case 'static_generator': {
          const busIdx = findBusIndex(comp.id);
          const isDcStaticGen = state.connections.some(conn => {
            const otherId = conn.from.compId === comp.id ? conn.to.compId
              : (conn.to.compId === comp.id ? conn.from.compId : null);
            return otherId != null && compBusDomainMap[otherId] === 'DC';
          });
          const row = {
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : sgenIdx,
            name: p.name || `SGen ${sgenIdx}`,
            bus: busIdx,
            p_mw: numOr(p.p_mw, 5),
            q_mvar: numOr(p.q_mvar, 0),
            sgen_type: p.sgen_type || 'PV',
            p_rated_mw: numOr(p.p_rated_mw, 0),
            sn_mva: numOr(p.sn_mva, 0),
            pmax_mw: numOr(p.pmax_mw, 0),
            pmin_mw: numOr(p.pmin_mw, 0),
            qmax_mvar: numOr(p.qmax_mvar, 0),
            qmin_mvar: numOr(p.qmin_mvar, 0),
            scaling: numOr(p.scaling, 1.0),
            controllable: p.controllable === true || p.controllable === 'true',
            v_ref_pu: numOr(p.v_ref_pu, 0),
            co2_emission_rate: numOr(p.emission_factor_tco2_mwh ?? p.co2_emission_rate, 0),
            in_service: p.in_service !== false,
          };
          if (isDcStaticGen) sys.dc.static_generators.push(addDynamicModel(row, p));
          else sys.ac.static_generators.push(addDynamicModel(row, p));
          sgenIdx++;
          break;
        }
        case 'vsc_converter': {
          sys.vsc_converters.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : vscIdx,
            name: p.name || `VSC ${Number.isFinite(Number(p.index)) ? Number(p.index) : vscIdx}`,
            // Resolve from the wired 'ac'/'dc' ports (authoritative); typed value is fallback.
            bus_ac: findBusIndexByPort(comp.id, 'ac') || parseInt(p.bus_ac) || 0,
            bus_dc: findBusIndexByPort(comp.id, 'dc') || parseInt(p.bus_dc) || 0,
            control_mode: p.control_mode || 'PQ_MODE',
            type: p.type || 'two_level',
            p_set_mw: numOr(p.p_set_mw, 0),
            q_set_mvar: numOr(p.q_set_mvar, 0),
            pmax_mw: numOr(p.pmax_mw, 200),
            pmin_mw: numOr(p.pmin_mw, -200),
            qmax_mvar: numOr(p.qmax_mvar, 100),
            qmin_mvar: numOr(p.qmin_mvar, -100),
            eta: numOr(p.eta, 0.98),
            loss_percent: numOr(p.loss_percent, 0),
            loss_mw: numOr(p.loss_mw, 0),
            v_dc_set_pu: numOr(p.v_dc_set_pu, 1.0),
            v_ac_set_pu: numOr(p.v_ac_set_pu, 1.0),
            v_ac_angle_set_deg: numOr(p.v_ac_angle_set_deg, 0),
            k_vdc: numOr(p.k_vdc, 0.1),
            p_rated_mw: numOr(p.p_rated_mw, 0),
            r_conv_ac_pu: numOr(p.r_conv_ac_pu, 0),
            r_sc_pu: numOr(p.r_sc_pu, 0),
            x_sc_pu: numOr(p.x_sc_pu, 0.15),
            r2_sc_pu: numOr(p.r2_sc_pu, 0),
            x2_sc_pu: numOr(p.x2_sc_pu, 0),
            i_max_pu: numOr(p.i_max_pu, 1.0),
            i_ac_max_pu: numOr(p.i_ac_max_pu, 0),
            i_dc_max_pu: numOr(p.i_dc_max_pu, 0),
            k_m_modulation: numOr(p.k_m_modulation, 0),
            m_min: numOr(p.m_min, 0),
            m_max: numOr(p.m_max, 0),
            vn_ac_kv: numOr(p.vn_ac_kv, 0),
            vn_dc_kv: numOr(p.vn_dc_kv, 0),
            grid_forming: p.grid_forming === true || p.grid_forming === 'true',
            ac_grid_forming: p.ac_grid_forming === true || p.ac_grid_forming === 'true',
            allow_dual_side_grid_forming: p.allow_dual_side_grid_forming === true || p.allow_dual_side_grid_forming === 'true',
            has_energy_buffer: p.has_energy_buffer === true || p.has_energy_buffer === 'true',
            in_service: p.in_service !== false,
          }, p));
          vscIdx++;
          break;
        }
        case 'dc_load': {
          const busIdx = findBusIndex(comp.id);
          sys.dc.loads.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : dcLoadIdx,
            name: p.name || `DC Load ${Number.isFinite(Number(p.index)) ? Number(p.index) : dcLoadIdx}`,
            bus: busIdx,
            p_mw: numOr(p.p_mw, 0),
            scaling: numOr(p.scaling, 1.0),
            controllable: p.controllable === true || p.controllable === 'true',
            p_min_mw: numOr(p.p_min_mw, 0),
            cost_mw: numOr(p.cost_mw, 0),
            profile_id: numOr(p.profile_id, -1),
            in_service: p.in_service !== false,
          }, p));
          dcLoadIdx++;
          break;
        }
        case 'dc_branch': {
          const [from, to] = findTwoBusIndices(comp.id);
          sys.dc.branches.push({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : dcBrIdx,
            name: p.name || `DC Line ${Number.isFinite(Number(p.index)) ? Number(p.index) : dcBrIdx}`,
            from_bus: from, to_bus: to,
            r_pu: numOr(p.r_pu, 0.01),
            rate_a_mva: numOr(p.rate_a_mva, 200),
            length_km: numOr(p.length_km, 100),
            in_service: p.in_service !== false,
          });
          dcBrIdx++;
          break;
        }
        case 'dc_pv_array': {
          const busIdx = findBusIndex(comp.id);
          sys.dc.pv_arrays.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : dcPvIdx,
            name: p.name || `DC PV ${Number.isFinite(Number(p.index)) ? Number(p.index) : dcPvIdx}`,
            bus: busIdx,
            p_set_mw: numOr(p.p_set_mw, 5),
            irradiance: numOr(p.irradiance, 1000),
            temperature: numOr(p.temperature, 25),
            num_series: parseInt(p.num_series) || 0,
            num_parallel: parseInt(p.num_parallel) || 0,
            vmpp: numOr(p.vmpp, 0),
            impp: numOr(p.impp, 0),
            voc: numOr(p.voc, 0),
            isc: numOr(p.isc, 0),
            alpha_isc: numOr(p.alpha_isc, 0),
            beta_voc: numOr(p.beta_voc, 0),
            profile_id: numOr(p.profile_id, -1),
            in_service: p.in_service !== false,
          }, p));
          dcPvIdx++;
          break;
        }
        // switch, circuit_breaker, motor — add similarly
        case 'switch_comp': {
          const [from, to] = findTwoBusIndices(comp.id);
          sys.ac.switches.push({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : swIdx,
            bus_from: from, bus_to: to,
            switch_type: p.switch_type || '',
            closed: p.closed !== false,
            r_contact_ohm: numOr(p.r_contact_ohm, 0),
            z_ohm: numOr(p.z_ohm, 0),
            i_rated_ka: numOr(p.i_rated_ka, 0),
            i_breaking_ka: numOr(p.i_breaking_ka, 0),
            in_service: p.in_service !== false,
          });
          swIdx++;
          break;
        }
        case 'circuit_breaker': {
          const busConns = [];
          for (const conn of state.connections) {
            let otherCompId = null;
            if (conn.from.compId === comp.id) otherCompId = conn.to.compId;
            else if (conn.to.compId === comp.id) otherCompId = conn.from.compId;
            if (otherCompId === null) continue;
            const other = getComponent(otherCompId);
            if (other && (other.type === 'ac_bus' || other.type === 'dc_bus')) {
              busConns.push({ comp: other, type: other.type });
            }
          }
          const isDcBreaker = busConns.length > 0 && busConns.every(b => b.type === 'dc_bus');
          const [from, to] = findTwoBusIndices(comp.id);
          const idxValue = Number.isFinite(Number(p.index)) ? Number(p.index) : cbIdx;
          if (isDcBreaker) {
            sys.dc.dc_circuit_breakers.push({
              index: idxValue,
              bus_from: from, bus_to: to,
              breaker_type: p.breaker_type || '',
              closed: p.closed !== false,
              r_ohm: numOr(p.r_ohm ?? p.z_ohm, 0),
              rated_voltage_kv: numOr(p.rated_voltage_kv, 0),
              i_rated_ka: numOr(p.i_rated_ka ?? p.rated_current_ka, 0),
              i_breaking_ka: numOr(p.i_breaking_ka, 0),
              in_service: p.in_service !== false,
            });
          } else {
            sys.ac.circuit_breakers.push({
              index: idxValue,
              bus_from: from, bus_to: to,
              breaker_type: p.breaker_type || '',
              closed: p.closed !== false,
              z_ohm: numOr(p.z_ohm, 0),
              rated_voltage_kv: numOr(p.rated_voltage_kv, 0),
              i_rated_ka: numOr(p.i_rated_ka ?? p.rated_current_ka, 0),
              i_breaking_ka: numOr(p.i_breaking_ka, 0),
              rated_current_ka: numOr(p.rated_current_ka, 2.0),
              in_service: p.in_service !== false,
            });
          }
          cbIdx++;
          break;
        }
        case 'motor': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.motors.push({
            index: motorIdx++, bus: busIdx,
            vn_kv: numOr(p.vn_kv, 6.3),
            sn_mva: numOr(p.sn_mva, 5),
            r_pu: numOr(p.r_pu, 0.02),
            x_pu: numOr(p.x_pu, 0.15),
            x_r: numOr(p.x_r, 0),
            lrc: numOr(p.lrc, 0),
            poles: numOr(p.poles, 0),
            cos_phi: numOr(p.cos_phi, 0.85),
            efficiency: numOr(p.efficiency, 0.94),
            r0_pu: numOr(p.r0_pu, 0),
            x0_pu: numOr(p.x0_pu, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'flexible_load': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.flexible_loads.push({
            index: sys.ac.flexible_loads.length, bus: busIdx,
            p_mw: numOr(p.p_mw, 0),
            q_mvar: numOr(p.q_mvar, 0),
            flex_up_mw: numOr(p.flex_up_mw, 0),
            flex_down_mw: numOr(p.flex_down_mw, 0),
            flex_duration_h: numOr(p.flex_duration_h, 0),
            response_time_s: numOr(p.response_time_s, 0),
            ramp_rate_mw_min: numOr(p.ramp_rate_mw_min, 0),
            availability_pct: numOr(p.availability_pct, 100),
            controllable: p.controllable !== false,
            priority: p.priority || 'Medium',
            control_area: p.control_area || '',
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'asymmetric_load': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.asymmetric_loads.push(addDynamicModel({
            index: sys.ac.asymmetric_loads.length, bus: busIdx,
            connection: p.connection || 'wye',
            grounded: p.grounded !== false,
            pa_mw: numOr(p.pa_mw, 0),
            qa_mvar: numOr(p.qa_mvar, 0),
            pb_mw: numOr(p.pb_mw, 0),
            qb_mvar: numOr(p.qb_mvar, 0),
            pc_mw: numOr(p.pc_mw, 0),
            qc_mvar: numOr(p.qc_mvar, 0),
            scaling: numOr(p.scaling, 1.0),
            const_z_percent: numOr(p.const_z_percent, 0),
            const_i_percent: numOr(p.const_i_percent, 0),
            const_p_percent: numOr(p.const_p_percent, 100),
            controllable: p.controllable === true || p.controllable === 'true',
            priority: p.priority || 'Medium',
            in_service: p.in_service !== false,
          }, p));
          break;
        }
        case 'shunt': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.shunts.push({
            index: sys.ac.shunts.length, bus: busIdx,
            gs_mw: numOr(p.gs_mw, 0),
            bs_mvar: numOr(p.bs_mvar, 0),
            switchable: p.switchable === true,
            n_steps: parseInt(p.n_steps) || 1,
            current_step: parseInt(p.current_step) || 1,
            bs_per_step: numOr(p.bs_per_step, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'transformer_3w': {
          const ports3w = [];
          for (const conn of state.connections) {
            let busCompId = null;
            if (conn.from.compId === comp.id) busCompId = conn.to.compId;
            if (conn.to.compId === comp.id) busCompId = conn.from.compId;
            if (busCompId !== null && compBusMap[busCompId] !== undefined) ports3w.push(compBusMap[busCompId]);
          }
          sys.ac.transformers_3w.push({
            index: sys.ac.transformers_3w.length,
            hv_bus: ports3w[0] || 0, mv_bus: ports3w[1] || 0, lv_bus: ports3w[2] || 0,
            sn_hv_mva: numOr(p.sn_hv_mva, 100),
            sn_mv_mva: numOr(p.sn_mv_mva, 50),
            sn_lv_mva: numOr(p.sn_lv_mva, 25),
            vn_hv_kv: numOr(p.vn_hv_kv, 220),
            vn_mv_kv: numOr(p.vn_mv_kv, 110),
            vn_lv_kv: numOr(p.vn_lv_kv, 35),
            vk_hv_mv_percent: numOr(p.vk_hv_mv_percent, 12),
            vk_hv_lv_percent: numOr(p.vk_hv_lv_percent, 12),
            vk_mv_lv_percent: numOr(p.vk_mv_lv_percent, 10),
            vkr_hv_mv_percent: numOr(p.vkr_hv_mv_percent, 0.5),
            vkr_hv_lv_percent: numOr(p.vkr_hv_lv_percent, 0.5),
            vkr_mv_lv_percent: numOr(p.vkr_mv_lv_percent, 0.4),
            pfe_kw: numOr(p.pfe_kw, 30),
            i0_percent: numOr(p.i0_percent, 0.1),
            tap_side: numOr(p.tap_side, 0),
            tap_pos: numOr(p.tap_pos, 0),
            tap_step_percent: numOr(p.tap_step_percent, 0),
            shift_mv_deg: numOr(p.shift_mv_deg, 0),
            shift_lv_deg: numOr(p.shift_lv_deg, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'charger': {
          sys.ac.chargers.push({
            index: sys.ac.chargers.length,
            station_id: parseInt(p.station_id) || 0,
            charger_type: p.charger_type || 'AC_L2',
            p_rated_kw: numOr(p.p_rated_kw, 0),
            p_ch_max_kw: numOr(p.p_ch_max_kw, 0),
            p_ch_min_kw: numOr(p.p_ch_min_kw, 0),
            eta: numOr(p.eta, 0.95),
            v2g_capable: p.v2g_capable === true,
            p_dis_max_kw: numOr(p.p_dis_max_kw, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'charging_station': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.charging_stations.push({
            index: sys.ac.charging_stations.length, bus: busIdx,
            location: p.location || '',
            n_fast: parseInt(p.n_fast) || 0,
            n_slow: parseInt(p.n_slow) || 0,
            num_chargers: parseInt(p.num_chargers) || 0,
            p_fast_max_kw: numOr(p.p_fast_max_kw, 0),
            p_slow_max_kw: numOr(p.p_slow_max_kw, 0),
            max_power_kw: numOr(p.max_power_kw, 0),
            simultaneity_factor: numOr(p.simultaneity_factor, 1.0),
            power_factor: numOr(p.power_factor, 0.95),
            utilization_rate: numOr(p.utilization_rate, 0),
            p_total_kw: numOr(p.p_total_kw, 0),
            q_total_kvar: numOr(p.q_total_kvar, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'mobile_storage': {
          const busIdx = findBusIndex(comp.id);
          sys.mobile_storage.push({
            index: sys.mobile_storage.length, bus: busIdx,
            p_mw: numOr(p.p_mw, 0),
            q_mvar: numOr(p.q_mvar, 0),
            p_rated_mw: numOr(p.p_rated_mw, 0),
            e_rated_mwh: numOr(p.e_rated_mwh, 0),
            pmax_mw: numOr(p.pmax_mw, 0),
            pmin_mw: numOr(p.pmin_mw, 0),
            qmax_mvar: numOr(p.qmax_mvar, 0),
            qmin_mvar: numOr(p.qmin_mvar, 0),
            soc_init: numOr(p.soc_init, 0.5),
            soc_min: numOr(p.soc_min, 0.1),
            soc_max: numOr(p.soc_max, 0.9),
            eta_charge: numOr(p.eta_charge, 0.95),
            eta_discharge: numOr(p.eta_discharge, 0.95),
            is_mobile: true,
            status: p.status || 'Stationary',
            target_bus: parseInt(p.target_bus) || 0,
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'dcdc_converter': {
          sys.dcdc_converters.push(addDynamicModel({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : sys.dcdc_converters.length,
            name: p.name || `DCDC ${Number.isFinite(Number(p.index)) ? Number(p.index) : sys.dcdc_converters.length}`,
            // Resolve from the wired 'in'/'out' ports (authoritative); the typed
            // property is only a fallback when the port is unwired.
            bus_in: findBusIndexByPort(comp.id, 'in') || parseInt(p.bus_in) || 0,
            bus_out: findBusIndexByPort(comp.id, 'out') || parseInt(p.bus_out) || 0,
            control_mode: p.control_mode || 'Voltage',
            p_ref_mw: numOr(p.p_ref_mw, 0),
            v_ref_pu: numOr(p.v_ref_pu, 1.0),
            sn_mva: numOr(p.sn_mva, 0),
            vn_in_kv: numOr(p.vn_in_kv, 0),
            vn_out_kv: numOr(p.vn_out_kv, 0),
            eta: numOr(p.eta, 0.98),
            r_eq_pu: numOr(p.r_eq_pu, 0),
            pmax_mw: numOr(p.pmax_mw, 0),
            pmin_mw: numOr(p.pmin_mw, 0),
            k_droop: numOr(p.k_droop, 0),
            topology: p.topology || 'Generic',
            d_min: numOr(p.d_min, 0.05),
            d_max: numOr(p.d_max, 0.95),
            n_ratio: numOr(p.n_ratio, 1.0),
            in_service: p.in_service !== false,
          }, p));
          break;
        }
        case 'energy_router': {
          // Build ports array from per-port parameters
          const erPorts = [];
          const erPortDefs = COMP.ports.energy_router || [];
          for (let pi = 1; pi <= 4; pi++) {
            const portDef = erPortDefs[pi - 1];
            const wired = portDef ? findBusByPort(comp.id, portDef.id) : null;
            const typedPortType = String(p['port' + pi + '_type'] || '').toUpperCase();
            const portType = wired?.domain || (typedPortType === 'DC' ? 'DC' : 'AC');
            const pBus = wired?.index || parseInt(p['port' + pi + '_bus']) || 0;
            if (pBus === 0) continue;  // skip ports with no bus assigned
            const voltageLevel = portType === 'DC'
              ? numOr(p.vn_dc_kv, numOr(p.vn_ac_kv, 0))
              : numOr(p.vn_ac_kv, 0);
            erPorts.push({
              index: Number.isFinite(Number(p['port' + pi + '_index']))
                ? Number(p['port' + pi + '_index'])
                : pi,
              name: p['port' + pi + '_name'] || (p.name || 'ER') + '_P' + pi,
              bus: pBus,
              port_type: portType,
              side: parseInt(p['port' + pi + '_side']) || 0,
              control_mode: p['port' + pi + '_control_mode'] || 'PQ',
              p_set_mw: numOr(p['port' + pi + '_p_set_mw'], 0),
              q_set_mvar: numOr(p['port' + pi + '_q_set_mvar'], 0),
              v_set_pu: numOr(p['port' + pi + '_v_set_pu'], 1.0),
              eta: numOr(p['port' + pi + '_eta'], 0.98),
              voltage_level_kv: voltageLevel,
              pmax_mw: numOr(p['port' + pi + '_pmax_mw'], numOr(p.pmax_mw, 0)),
              pmin_mw: numOr(p['port' + pi + '_pmin_mw'], numOr(p.pmin_mw, 0)),
              qmax_mvar: numOr(p['port' + pi + '_qmax_mvar'], numOr(p.qmax_mvar, 0)),
              qmin_mvar: numOr(p['port' + pi + '_qmin_mvar'], numOr(p.qmin_mvar, 0)),
              in_service: p['port' + pi + '_in_service'] !== false,
            });
          }
          sys.energy_routers.push({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : sys.energy_routers.length,
            name: p.name || 'ERouter',
            router_type: p.router_type || 'hybrid',
            num_ports: erPorts.length,
            ports: erPorts,
            p_rated_mw: numOr(p.p_rated_mw, 0),
            vn_ac_kv: numOr(p.vn_ac_kv, 0),
            vn_dc_kv: numOr(p.vn_dc_kv, 0),
            loss_percent: numOr(p.loss_percent, 0),
            pmax_mw: numOr(p.pmax_mw, 0),
            pmin_mw: numOr(p.pmin_mw, 0),
            qmax_mvar: numOr(p.qmax_mvar, 0),
            qmin_mvar: numOr(p.qmin_mvar, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'vpp': {
          const busIdx = findBusIndex(comp.id);
          // Coerce aggregated ID lists into integer arrays (UI may store as comma-separated strings)
          const toIdList = (v) => {
            if (Array.isArray(v)) return v.map(x => parseInt(x)).filter(n => Number.isFinite(n));
            if (typeof v === 'string' && v.trim()) return v.split(/[,\s]+/).map(s => parseInt(s)).filter(n => Number.isFinite(n));
            return [];
          };
          sys.vpps.push({
            index: sys.vpps.length,
            name: p.name || '',
            description: p.description || '',
            pcc_bus: busIdx || parseInt(p.pcc_bus) || parseInt(p.aggregation_bus) || 0,
            aggregated_gen_ids: toIdList(p.aggregated_gen_ids),
            aggregated_storage_ids: toIdList(p.aggregated_storage_ids),
            aggregated_load_ids: toIdList(p.aggregated_load_ids),
            n_pv_systems: parseInt(p.n_pv_systems) || 0,
            n_wind_turbines: parseInt(p.n_wind_turbines) || 0,
            n_battery_systems: parseInt(p.n_battery_systems) || 0,
            n_ev_chargers: parseInt(p.n_ev_chargers) || 0,
            n_controllable_loads: parseInt(p.n_controllable_loads) || 0,
            p_generation_sum_mw: numOr(p.p_generation_sum_mw, 0),
            e_storage_sum_mwh: numOr(p.e_storage_sum_mwh, 0),
            p_load_controllable_mw: numOr(p.p_load_controllable_mw, 0),
            p_output_mw: numOr(p.p_output_mw, 0),
            q_output_mvar: numOr(p.q_output_mvar, 0),
            pmax_mw: numOr(p.pmax_mw, 0),
            pmin_mw: numOr(p.pmin_mw, 0),
            ramp_up_max_mw_min: numOr(p.ramp_up_max_mw_min, 0),
            ramp_down_max_mw_min: numOr(p.ramp_down_max_mw_min, 0),
            mtbf_hours: numOr(p.mtbf_hours, 0),
            mttr_hours: numOr(p.mttr_hours, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'microgrid': {
          const busIdx = findBusIndex(comp.id);
          sys.microgrids.push({
            index: Number.isFinite(Number(p.index)) ? Number(p.index) : sys.microgrids.length,
            name: p.name || 'MicroGrid',
            description: p.description || '',
            pcc_bus: busIdx || parseInt(p.pcc_bus) || 0,
            operating_mode: p.operating_mode || 'GridConnected',
            islanding_capability: p.islanding_capability === true,
            auto_reconnection: p.auto_reconnection === true,
            p_exchange_max_mw: numOr(p.p_exchange_max_mw, 0),
            p_exchange_min_mw: numOr(p.p_exchange_min_mw, 0),
            p_import_max_mw: numOr(p.p_import_max_mw, 0),
            p_export_max_mw: numOr(p.p_export_max_mw, 0),
            p_exchange_mw: numOr(p.p_exchange_mw, 0),
            total_generation_mw: numOr(p.total_generation_mw, 0),
            total_storage_mwh: numOr(p.total_storage_mwh, 0),
            total_load_mw: numOr(p.total_load_mw, 0),
            capacity_mw: numOr(p.capacity_mw, 0),
            peak_load_mw: numOr(p.peak_load_mw, 0),
            f_set_hz: numOr(p.f_set_hz, 50),
            v_set_pu: numOr(p.v_set_pu, 1.0),
            k_droop: numOr(p.k_droop, 0),
            area: parseInt(p.area) || 0,
            in_service: p.in_service !== false,
          });
          break;
        }
      }
    });

    // When load components exist, zero out bus-level pd/qd to avoid double-counting.
    // The PF solver uses load objects when they exist, ignoring bus pd_mw.
    if (sys.ac.loads.length > 0) {
      sys.ac.buses.forEach(bus => { bus.pd_mw = 0; bus.qd_mvar = 0; });
    }

    // ----- Canvas layout (positions/rotation/connections/viewport) -----
    // Persist GUI placement so that round-tripping through export/import
    // does not destroy the user's layout.  Prefer stable model indices for
    // non-contiguous ids (e.g. load#1001), and fall back to per-type ordinals
    // when an index key would be ambiguous.  The backend ignores unknown fields.
    {
      const compKey = {};
      const typeCounter = {};
      const typeTotals = {};
      state.components.forEach(comp => {
        typeTotals[comp.type] = (typeTotals[comp.type] || 0) + 1;
      });
      state.components.forEach(comp => {
        const t = comp.type;
        typeCounter[t] = (typeCounter[t] || 0) + 1;
        const ordinal = typeCounter[t];
        const idx = Number(comp.params?.index);
        const canUseIndexKey = Number.isFinite(idx) && (idx === ordinal || idx > typeTotals[t] || idx < 1);
        compKey[comp.id] = t + '#' + (canUseIndexKey ? idx : ordinal);
      });
      sys._canvas = {
        version: 2,
        themeMode: (typeof document !== 'undefined' &&
                    document.documentElement.getAttribute('data-theme')) || 'dark',
        connectionStyle: state.connectionStyle || 'orthogonal',
        viewBox: { x: viewBox.x, y: viewBox.y, w: viewBox.w, h: viewBox.h },
        components: state.components.map(c => ({
          key: compKey[c.id],
          type: c.type,
          x: c.x,
          y: c.y,
          rotation: c.rotation || 0,
        })),
        connections: state.connections
          .map(cn => ({
            from: { key: compKey[cn.from.compId], port: cn.from.portId },
            to:   { key: compKey[cn.to.compId],   port: cn.to.portId   },
            // Persist only user-edited waypoints; auto-routed wires recompute.
            waypoints: cn.geom && cn.geom.userPoints
              ? cn.geom.userPoints.map(p => ({ x: p.x, y: p.y })) : null,
          }))
          .filter(cn => cn.from.key && cn.to.key),
      };
    }

    return sys;
  }

  // ========== Apply Canvas Layout (positions/connections/viewport) ==========
  function applyCanvasLayout(layout) {
    if (!layout || !Array.isArray(layout.components)) return;

    // Restore the saved light/dark theme so a shared file keeps its appearance
    // (doc §12).  Mirrors app.js setThemeMode (attribute + persisted choice).
    if (layout.themeMode === 'light' || layout.themeMode === 'dark') {
      try {
        document.documentElement.setAttribute('data-theme', layout.themeMode);
        localStorage.setItem('themeMode', layout.themeMode);
      } catch (e) { /* ignore */ }
    }

    // Restore the saved connection style (doc §10.2/§10.3); unknown/absent keeps
    // the current default so older files load fine.
    if (['straight', 'orthogonal', 'avoid'].includes(layout.connectionStyle)) {
      state.connectionStyle = layout.connectionStyle;
      try { localStorage.setItem('connectionStyle', layout.connectionStyle); } catch (e) { /* ignore */ }
    }

    // Rebuild key -> compId by walking the just-loaded components in the
    // same per-type order used when the layout was saved.  Some historical
    // cases use model indices in layout keys (e.g. load#1001) instead of
    // per-type ordinals, so keep an index-key fallback as well.
    const keyToCompId = {};
    const indexKeyToCompId = {};
    const typeCounter = {};
    state.components.forEach(comp => {
      const t = comp.type;
      typeCounter[t] = (typeCounter[t] || 0) + 1;
      keyToCompId[t + '#' + typeCounter[t]] = comp.id;
      const idx = Number(comp.params?.index);
      if (Number.isFinite(idx)) {
        indexKeyToCompId[t + '#' + idx] = comp.id;
      }
    });
    const resolveLayoutCompId = key => keyToCompId[key] ?? indexKeyToCompId[key];

    // Restore position and rotation.
    layout.components.forEach(item => {
      if (!item || !item.key) return;
      const cid = resolveLayoutCompId(item.key);
      if (cid === undefined) return;
      const comp = getComponent(cid);
      if (!comp) return;
      comp.x = numOr(item.x, comp.x);
      comp.y = numOr(item.y, comp.y);
      comp.rotation = numOr(item.rotation, 0);
      comp.el.setAttribute(
        'transform',
        'translate(' + comp.x + ', ' + comp.y + ') rotate(' + comp.rotation + ')'
      );
    });

    // Replace auto-generated connections with the saved port-level set.
    if (Array.isArray(layout.connections)) {
      [...state.connections].forEach(cn => removeConnection(cn.id));
      layout.connections.forEach(cn => {
        if (!cn || !cn.from || !cn.to) return;
        const fid = resolveLayoutCompId(cn.from.key);
        const tid = resolveLayoutCompId(cn.to.key);
        if (fid === undefined || tid === undefined) return;
        const conn = addConnection(fid, cn.from.port, tid, cn.to.port);
        if (conn && Array.isArray(cn.waypoints) && cn.waypoints.length >= 2) {
          conn.geom = conn.geom || {};
          conn.geom.userPoints = cn.waypoints.map(p => ({ x: p.x, y: p.y }));
        }
      });
    }

    // Restore the viewport (pan + zoom) so the diagram appears identically.
    if (layout.viewBox) {
      viewBox.x = numOr(layout.viewBox.x, viewBox.x);
      viewBox.y = numOr(layout.viewBox.y, viewBox.y);
      viewBox.w = numOr(layout.viewBox.w, viewBox.w);
      viewBox.h = numOr(layout.viewBox.h, viewBox.h);
      state.zoom = 1200 / viewBox.w;
      updateViewBox();
    }

    // Re-render all connections so endpoints follow the new positions (build the
    // avoidance context first when that style is active).
    _routeCtx = state.connectionStyle === 'avoid' ? buildRouteContext() : null;
    state.connections.forEach(rerenderConnection);
  }

  // ========== Load from JSON System ==========
	  function loadFromSystemJson(jsonSys) {
	    // Clear canvas
	    clearAll();
	    _preservedModelBlocks = {};
	    if (jsonSys?.three_phase_ac) {
	      _preservedModelBlocks.three_phase_ac = cloneJsonBlock(jsonSys.three_phase_ac);
	    }

    // Preserve system base MVA (critical for per-unit calculations)
    state.baseMva = jsonSys.base_mva || 100;

    const busCompMap = {}; // busIndex -> compId (for AC)
    const dcBusCompMap = {};  // for DC

    // Create AC buses
    if (jsonSys.ac?.buses) {
      jsonSys.ac.buses.forEach((bus, i) => {
        const comp = addComponent('ac_bus',
          100 + (i % 6) * 200,
          100 + Math.floor(i / 6) * 220,
          {
            ...COMP.defaults.ac_bus,
            index: bus.index,
            name: `Bus ${bus.index}`,
            bus_type: bus.bus_type || 'PQ',
            base_kv: bus.base_kv || 110,
            vm_pu: bus.vm_pu || 1.0,
            va_deg: bus.va_deg || 0,
            pd_mw: bus.pd_mw || 0,
            qd_mvar: bus.qd_mvar || 0,
            vmin_pu: bus.vmin_pu || 0.9,
            vmax_pu: bus.vmax_pu || 1.1,
            gs_mw: bus.gs_mw || 0,
            bs_mvar: bus.bs_mvar || 0,
            n_customers: bus.n_customers || 0,
            importance: bus.importance || 0,
            in_service: bus.in_service !== false,
            area: bus.area || 1,
            zone: bus.zone || 1,
          }
        );
        busCompMap[bus.index] = comp.id;
      });
    }

    // Create DC buses
    if (jsonSys.dc?.buses) {
      jsonSys.dc.buses.forEach((bus, i) => {
        const comp = addComponent('dc_bus',
          100 + (i % 4) * 200,
          100 + ((jsonSys.ac?.buses?.length || 0) > 0 ? Math.ceil((jsonSys.ac.buses.length) / 6) : 0) * 220 + 200 + Math.floor(i / 4) * 180,
          {
            ...COMP.defaults.dc_bus,
            index: bus.index,
            name: `DC Bus ${bus.index}`,
            bus_type: bus.bus_type || 'DC_P',
            base_kv: bus.base_kv || 320,
            vm_pu: bus.vm_pu || 1.0,
            vmax_pu: bus.vmax_pu || 1.1,
            vmin_pu: bus.vmin_pu || 0.9,
            pd_mw: bus.pd_mw || 0,
            in_service: bus.in_service !== false,
          }
        );
        dcBusCompMap[bus.index] = comp.id;
      });
    }

    // Helper: add device and connect to bus
    function addDeviceAtBus(type, busIdx, params, busMap, yOffset = 80) {
      const busCompId = busMap[busIdx];
      if (busCompId === undefined) return null;
      const busComp = getComponent(busCompId);
      if (!busComp) return null;

      // Count existing devices on this bus to offset horizontally
      const existingDevices = state.connections.filter(c =>
        c.from.compId === busCompId || c.to.compId === busCompId
      ).length;

      const offsetX = (existingDevices % 5 - 2) * 80;
      const comp = addComponent(type, busComp.x + offsetX, busComp.y + yOffset, params);
      addConnection(comp.id, COMP.ports[type]?.[0]?.id || 'top',
                    busCompId, 'bottom');
      return comp;
    }

    // Generators
    jsonSys.ac?.generators?.forEach(gen => {
      addDeviceAtBus('generator', gen.bus, {
        ...COMP.defaults.generator,
        index: gen.index,
        name: gen.name || `Gen ${gen.index !== undefined ? gen.index : ''}`,
        bus: gen.bus,
        pg_mw: gen.pg_mw, qg_mvar: gen.qg_mvar,
        vg_pu: gen.vg_pu || 1.0,
        pmax_mw: gen.pmax_mw, pmin_mw: gen.pmin_mw,
        qmax_mvar: gen.qmax_mvar, qmin_mvar: gen.qmin_mvar,
        mbase_mva: gen.mbase_mva,
        is_slack: gen.is_slack || false,
        in_service: gen.in_service !== false,
        cost_c2: gen.cost_c2, cost_c1: gen.cost_c1, cost_c0: gen.cost_c0,
        emission_factor_tco2_mwh: gen.emission_factor_tco2_mwh || gen.co2_emission_rate || 0,
        startup_cost: gen.startup_cost, shutdown_cost: gen.shutdown_cost,
        ramp_up_mw_min: gen.ramp_up_mw_min, ramp_dn_mw_min: gen.ramp_dn_mw_min,
        dynamic_model: cloneDynamicModel(gen.dynamic_model) || COMP.defaults.generator.dynamic_model,
      }, busCompMap);
    });

    // Loads
    jsonSys.ac?.loads?.forEach(load => {
      addDeviceAtBus('load', load.bus, {
        ...COMP.defaults.load,
        index: load.index,
        name: load.name || `Load ${load.index !== undefined ? load.index : ''}`,
        bus: load.bus,
        p_mw: load.p_mw, q_mvar: load.q_mvar,
        scaling: load.scaling || 1.0,
        model: load.model || 'ConstantPower',
        z_percent_p: load.z_percent_p, i_percent_p: load.i_percent_p,
        p_percent_p: load.p_percent_p,
        z_percent_q: load.z_percent_q, i_percent_q: load.i_percent_q,
        p_percent_q: load.p_percent_q,
        controllable: load.controllable || false,
        p_min_mw: load.p_min_mw, cost_mw: load.cost_mw,
        priority: load.priority || 'Medium',
        n_customers: load.n_customers,
        profile_id: load.profile_id,
        dynamic_model: cloneDynamicModel(load.dynamic_model) || COMP.defaults.load.dynamic_model,
        in_service: load.in_service !== false,
      }, busCompMap);
    });

    // If no explicit loads, create load components from buses with non-zero pd_mw
    if (!jsonSys.ac?.loads?.length && jsonSys.ac?.buses) {
      jsonSys.ac.buses.forEach(bus => {
        if ((bus.pd_mw && bus.pd_mw !== 0) || (bus.qd_mvar && bus.qd_mvar !== 0)) {
          addDeviceAtBus('load', bus.index, {
            ...COMP.defaults.load,
            index: bus.index,
            name: `Load ${bus.index !== undefined ? bus.index : ''}`,
            bus: bus.index,
            p_mw: bus.pd_mw || 0,
            q_mvar: bus.qd_mvar || 0,
            scaling: 1.0,
            dynamic_model: COMP.defaults.load.dynamic_model,
          }, busCompMap);
        }
      });
    }

    // External grids
    jsonSys.ac?.external_grids?.forEach(eg => {
      addDeviceAtBus('external_grid', eg.bus, {
        ...COMP.defaults.external_grid,
        index: eg.index,
        name: eg.name || 'Grid',
        vm_pu: eg.vm_pu, va_deg: eg.va_deg,
        s_sc_max_mva: eg.s_sc_max_mva,
        s_sc_min_mva: eg.s_sc_min_mva,
        rx_max: eg.rx_max, rx_min: eg.rx_min,
        r_pu: eg.r_pu, x_pu: eg.x_pu,
        r0_pu: eg.r0_pu, x0_pu: eg.x0_pu,
        vn_kv: eg.vn_kv,
        emission_factor_tco2_mwh: eg.emission_factor_tco2_mwh || eg.co2_emission_rate || 0,
        emission_factor_profile_tco2_mwh: Array.isArray(eg.emission_factor_profile_tco2_mwh)
          ? eg.emission_factor_profile_tco2_mwh
          : undefined,
        controllable: eg.controllable || false,
        in_service: eg.in_service !== false,
        cost_c2: eg.cost_c2 || 0,
        cost_c1: eg.cost_c1 || 0,
        cost_c0: eg.cost_c0 || 0,
        price_profile_id: eg.price_profile_id != null ? eg.price_profile_id : -1,
        dynamic_model: cloneDynamicModel(eg.dynamic_model) || COMP.defaults.external_grid.dynamic_model,
      }, busCompMap, -80);
    });

    // Storage
    jsonSys.ac?.storage?.forEach(s => {
      addDeviceAtBus('storage', s.bus, {
        ...COMP.defaults.storage,
        index: s.index,
        name: s.name || `ESS ${s.index !== undefined ? s.index : ''}`,
        bus: s.bus,
        p_mw: s.p_mw, q_mvar: s.q_mvar,
        p_rated_mw: s.p_rated_mw, e_rated_mwh: s.e_rated_mwh,
        soc_init: s.soc_init, soc_min: s.soc_min, soc_max: s.soc_max,
        eta_charge: s.eta_charge, eta_discharge: s.eta_discharge,
        pmax_mw: s.pmax_mw, pmin_mw: s.pmin_mw,
        qmax_mvar: s.qmax_mvar, qmin_mvar: s.qmin_mvar,
        self_discharge_pct: s.self_discharge_pct,
        profile_id: s.profile_id,
        dynamic_model: cloneDynamicModel(s.dynamic_model) || COMP.defaults.storage.dynamic_model,
        in_service: s.in_service !== false,
      }, busCompMap);
    });

    // PV Systems
    jsonSys.ac?.pv_systems?.forEach(pv => {
      addDeviceAtBus('pv_system', pv.bus, {
        ...COMP.defaults.pv_system,
        index: pv.index,
        name: pv.name || `PV ${pv.index !== undefined ? pv.index : ''}`,
        bus: pv.bus,
        p_mw: pv.p_mw, q_mvar: pv.q_mvar,
        sn_mva: pv.sn_mva,
        pmax_mw: pv.pmax_mw, pmin_mw: pv.pmin_mw,
        qmax_mvar: pv.qmax_mvar, qmin_mvar: pv.qmin_mvar,
        control_mode: pv.control_mode,
        controllable: pv.controllable,
        v_ac_set_pu: pv.v_ac_set_pu, v_dc_set_pu: pv.v_dc_set_pu,
        inverter_eff: pv.inverter_eff, loss_percent: pv.loss_percent,
        num_series: pv.num_series, num_parallel: pv.num_parallel,
        vmpp: pv.vmpp, impp: pv.impp, voc: pv.voc, isc: pv.isc,
        alpha_isc: pv.alpha_isc, beta_voc: pv.beta_voc,
        irradiance: pv.irradiance, temperature: pv.temperature,
        profile_id: pv.profile_id,
        dynamic_model: cloneDynamicModel(pv.dynamic_model) || COMP.defaults.pv_system.dynamic_model,
        in_service: pv.in_service !== false,
      }, busCompMap);
    });

    // Renewable gens
    jsonSys.ac?.renewable_gens?.forEach(rg => {
      addDeviceAtBus('renewable_gen', rg.bus, {
        ...COMP.defaults.renewable_gen,
        index: rg.index,
        name: rg.name || rg.type || 'Wind',
        type: rg.type,
        p_mw: rg.p_mw, q_mvar: rg.q_mvar,
        p_rated_mw: rg.p_rated_mw,
        qmax_mvar: rg.qmax_mvar, qmin_mvar: rg.qmin_mvar,
        curtailable: rg.curtailable,
        cost_curtail_mwh: rg.cost_curtail_mwh,
        capacity_factor: rg.capacity_factor,
        profile_id: rg.profile_id,
        emission_offset_tco2_mwh: rg.emission_offset_tco2_mwh,
        in_service: rg.in_service !== false,
      }, busCompMap);
    });

    // Branches (lines)
    jsonSys.ac?.branches?.forEach(br => {
      const fromCompId = busCompMap[br.from_bus];
      const toCompId = busCompMap[br.to_bus];
      if (fromCompId === undefined || toCompId === undefined) return;
      const fromComp = getComponent(fromCompId);
      const toComp = getComponent(toCompId);
      if (!fromComp || !toComp) return;

      const mx = (fromComp.x + toComp.x) / 2;
      const my = (fromComp.y + toComp.y) / 2;

      // MATPOWER branches with tap≠1 or shift≠0 are transformers — render
      // them with the transformer_2w symbol while keeping branch parameters
      // so that buildSystemJson() still exports them as ac.branches.
      const tapVal = br.tap || 1.0;
      const shiftVal = br.shift_deg || 0;
      const isTrafo = (Math.abs(tapVal - 1.0) > 1e-6) || (Math.abs(shiftVal) > 1e-6);

      if (isTrafo) {
        const comp = addComponent('transformer_2w', mx + 60, my, {
          ...COMP.defaults.transformer_2w,
          index: br.index,
          name: br.name || `Trafo ${br.index !== undefined ? br.index : ''}`,
          hv_bus: br.from_bus, lv_bus: br.to_bus,
          // Store branch-model parameters so roundtrip is consistent
          _from_branch: true,
          r_pu: br.r_pu, x_pu: br.x_pu, b_pu: br.b_pu,
          rate_a_mva: br.rate_a_mva, tap: tapVal, shift_deg: shiftVal,
          sn_mva: br.rate_a_mva || 100,
          vk_percent: (br.x_pu || 0.1) * 100,
          vkr_percent: (br.r_pu || 0.01) * 100,
          in_service: br.in_service !== false,
        });
        addConnection(comp.id, 'hv', fromCompId, 'bottom');
        addConnection(comp.id, 'lv', toCompId, 'top');
      } else {
        const comp = addComponent('ac_branch', mx, my - 40, {
          ...COMP.defaults.ac_branch,
          index: br.index,
          name: br.name || `Line ${br.index !== undefined ? br.index : ''}`,
          from_bus: br.from_bus, to_bus: br.to_bus,
          r_pu: br.r_pu, x_pu: br.x_pu, b_pu: br.b_pu,
          rate_a_mva: br.rate_a_mva,
          rate_b_mva: br.rate_b_mva,
          rate_c_mva: br.rate_c_mva,
          length_km: br.length_km,
          tap: tapVal, shift_deg: shiftVal,
          n_parallel: br.n_parallel,
          failure_rate: br.failure_rate, mttr_hr: br.mttr_hr,
          in_service: br.in_service !== false,
        });
        addConnection(comp.id, 'left', fromCompId, 'right');
        addConnection(comp.id, 'right', toCompId, 'left');
      }
    });

    // Transformers — skip entries extracted from branches (source_branch_idx > 0)
    // to avoid duplication; those are already represented as ac_branch components.
    jsonSys.ac?.transformers_2w?.forEach(tr => {
      if (tr.source_branch_idx > 0) return; // already in ac.branches
      const hvCompId = busCompMap[tr.hv_bus];
      const lvCompId = busCompMap[tr.lv_bus];
      if (hvCompId === undefined || lvCompId === undefined) return;
      const hvComp = getComponent(hvCompId);
      const lvComp = getComponent(lvCompId);
      if (!hvComp || !lvComp) return;

      const mx = (hvComp.x + lvComp.x) / 2;
      const my = (hvComp.y + lvComp.y) / 2;
      const comp = addComponent('transformer_2w', mx + 60, my, {
        ...COMP.defaults.transformer_2w,
        index: tr.index,
        name: 'Trafo',
        hv_bus: tr.hv_bus, lv_bus: tr.lv_bus,
        sn_mva: tr.sn_mva,
        vn_hv_kv: tr.vn_hv_kv, vn_lv_kv: tr.vn_lv_kv,
        vk_percent: tr.vk_percent,
        vkr_percent: tr.vkr_percent,
        pfe_kw: tr.pfe_kw,
        i0_percent: tr.i0_percent,
        shift_deg: tr.shift_deg,
        tap_side: tr.tap_side,
        tap_pos: tr.tap_pos,
        tap_min: tr.tap_min,
        tap_max: tr.tap_max,
        tap_neutral: tr.tap_neutral,
        tap_step_percent: tr.tap_step_percent,
        vector_group: tr.vector_group || '',
        source_branch_idx: tr.source_branch_idx,
        in_service: tr.in_service !== false,
      });
      addConnection(comp.id, 'hv', hvCompId, 'bottom');
      addConnection(comp.id, 'lv', lvCompId, 'top');
    });

    // VSC converters
    jsonSys.vsc_converters?.forEach(vsc => {
      const acCompId = busCompMap[vsc.bus_ac];
      const dcCompId = dcBusCompMap[vsc.bus_dc];
      const acComp = acCompId !== undefined ? getComponent(acCompId) : null;
      const dcComp = dcCompId !== undefined ? getComponent(dcCompId) : null;

      const x = acComp ? acComp.x + 100 : (dcComp ? dcComp.x - 100 : 400);
      const y = acComp ? acComp.y : (dcComp ? dcComp.y : 300);
      const comp = addComponent('vsc_converter', x, y, {
        ...COMP.defaults.vsc_converter,
        index: vsc.index,
        name: vsc.name || `VSC ${vsc.index !== undefined ? vsc.index : ''}`,
        bus_ac: vsc.bus_ac, bus_dc: vsc.bus_dc,
        p_set_mw: vsc.p_set_mw, q_set_mvar: vsc.q_set_mvar,
        // Normalize control_mode: C++ uses "PQ"/"VDC_Q"/"VDC_VAC" but UI uses "PQ_MODE"
        control_mode: (vsc.control_mode === 'PQ' ? 'PQ_MODE' : vsc.control_mode) || 'PQ_MODE',
        eta: vsc.eta,
        loss_percent: vsc.loss_percent,
        loss_mw: vsc.loss_mw,
        v_dc_set_pu: vsc.v_dc_set_pu,
        v_ac_set_pu: vsc.v_ac_set_pu,
        v_ac_angle_set_deg: vsc.v_ac_angle_set_deg ?? 0,
        k_vdc: vsc.k_vdc,
        pmax_mw: vsc.pmax_mw, pmin_mw: vsc.pmin_mw,
        qmax_mvar: vsc.qmax_mvar, qmin_mvar: vsc.qmin_mvar,
        p_rated_mw: vsc.p_rated_mw,
        r_conv_ac_pu: vsc.r_conv_ac_pu ?? 0,
        r_sc_pu: vsc.r_sc_pu ?? 0,
        x_sc_pu: vsc.x_sc_pu ?? 0.15,
        r2_sc_pu: vsc.r2_sc_pu ?? 0,
        x2_sc_pu: vsc.x2_sc_pu ?? 0,
        i_max_pu: vsc.i_max_pu ?? 1.0,
        i_ac_max_pu: vsc.i_ac_max_pu ?? 0,
        i_dc_max_pu: vsc.i_dc_max_pu ?? 0,
        k_m_modulation: vsc.k_m_modulation ?? 0,
        m_min: vsc.m_min ?? 0,
        m_max: vsc.m_max ?? 0,
        vn_ac_kv: vsc.vn_ac_kv ?? 0,
        vn_dc_kv: vsc.vn_dc_kv ?? 0,
        grid_forming: vsc.grid_forming === true,
        ac_grid_forming: vsc.ac_grid_forming === true,
        allow_dual_side_grid_forming: vsc.allow_dual_side_grid_forming === true,
        has_energy_buffer: vsc.has_energy_buffer === true,
        dynamic_model: cloneDynamicModel(vsc.dynamic_model) || COMP.defaults.vsc_converter.dynamic_model,
        in_service: vsc.in_service !== false,
      });
      if (acCompId !== undefined) addConnection(comp.id, 'ac', acCompId, 'right');
      if (dcCompId !== undefined) addConnection(comp.id, 'dc', dcCompId, 'left');
    });

    // DC branches
    jsonSys.dc?.branches?.forEach(br => {
      const fromCompId = dcBusCompMap[br.from_bus];
      const toCompId = dcBusCompMap[br.to_bus];
      if (fromCompId === undefined || toCompId === undefined) return;
      const fromComp = getComponent(fromCompId);
      const toComp = getComponent(toCompId);
      if (!fromComp || !toComp) return;
      const mx = (fromComp.x + toComp.x) / 2;
      const my = (fromComp.y + toComp.y) / 2;
      const comp = addComponent('dc_branch', mx, my - 40, {
        ...COMP.defaults.dc_branch,
        index: br.index,
        name: br.name || `DC Line ${br.index !== undefined ? br.index : ''}`,
        from_bus: br.from_bus,
        to_bus: br.to_bus,
        r_pu: br.r_pu, rate_a_mva: br.rate_a_mva,
        length_km: br.length_km,
        in_service: br.in_service !== false,
      });
      addConnection(comp.id, 'left', fromCompId, 'right');
      addConnection(comp.id, 'right', toCompId, 'left');
    });

    // DC loads
    jsonSys.dc?.loads?.forEach(ld => {
      addDeviceAtBus('dc_load', ld.bus, {
        ...COMP.defaults.dc_load,
        index: ld.index,
        name: ld.name || `DC Load ${ld.index !== undefined ? ld.index : ''}`,
        bus: ld.bus,
        p_mw: ld.p_mw, scaling: ld.scaling,
        controllable: ld.controllable || false,
        p_min_mw: ld.p_min_mw,
        cost_mw: ld.cost_mw,
        profile_id: ld.profile_id,
        dynamic_model: cloneDynamicModel(ld.dynamic_model) || COMP.defaults.dc_load.dynamic_model,
        in_service: ld.in_service !== false,
      }, dcBusCompMap);
    });

    // DC static generators
    jsonSys.dc?.static_generators?.forEach(sg => {
      addDeviceAtBus('static_generator', sg.bus, {
        ...COMP.defaults.static_generator,
        index: sg.index,
        name: sg.name || `DC SGen ${sg.index !== undefined ? sg.index : ''}`,
        p_mw: sg.p_mw, q_mvar: sg.q_mvar,
        sgen_type: sg.sgen_type || 'PV',
        controllable: sg.controllable || false,
        scaling: sg.scaling,
        p_rated_mw: sg.p_rated_mw, sn_mva: sg.sn_mva,
        pmax_mw: sg.pmax_mw, pmin_mw: sg.pmin_mw,
        qmax_mvar: sg.qmax_mvar, qmin_mvar: sg.qmin_mvar,
        v_ref_pu: sg.v_ref_pu,
        dynamic_model: cloneDynamicModel(sg.dynamic_model) || COMP.defaults.static_generator.dynamic_model,
        in_service: sg.in_service !== false,
      }, dcBusCompMap);
    });

    // DC storage — both the dedicated dc_storage vector and the legacy
    // dc.storage (AC Storage reused on DC) are imported as dc_storage components
    // so they render on the DC bus and round-trip back to sys.dc.dc_storage.
    const importDcStorage = (s) => {
      addDeviceAtBus('dc_storage', s.bus, {
        ...COMP.defaults.dc_storage,
        index: s.index,
        name: s.name || 'DC ESS',
        p_mw: s.p_mw,
        p_rated_mw: s.p_rated_mw, e_rated_mwh: s.e_rated_mwh,
        soc_init: s.soc_init, soc_min: s.soc_min, soc_max: s.soc_max,
        eta_charge: s.eta_charge, eta_discharge: s.eta_discharge,
        pmax_mw: s.pmax_mw, pmin_mw: s.pmin_mw,
        self_discharge_pct: s.self_discharge_pct,
        profile_id: s.profile_id,
        dynamic_model: cloneDynamicModel(s.dynamic_model) || COMP.defaults.dc_storage.dynamic_model,
        in_service: s.in_service !== false,
      }, dcBusCompMap);
    };
    jsonSys.dc?.dc_storage?.forEach(importDcStorage);
    jsonSys.dc?.storage?.forEach(importDcStorage);

    // DC PV arrays
    jsonSys.dc?.pv_arrays?.forEach(pv => {
      addDeviceAtBus('dc_pv_array', pv.bus, {
        ...COMP.defaults.dc_pv_array,
        index: pv.index,
        name: pv.name || `DC PV ${pv.index !== undefined ? pv.index : ''}`,
        p_set_mw: pv.p_set_mw,
        irradiance: pv.irradiance,
        temperature: pv.temperature,
        num_series: pv.num_series, num_parallel: pv.num_parallel,
        vmpp: pv.vmpp, impp: pv.impp, voc: pv.voc, isc: pv.isc,
        alpha_isc: pv.alpha_isc, beta_voc: pv.beta_voc,
        profile_id: pv.profile_id,
        dynamic_model: cloneDynamicModel(pv.dynamic_model) || COMP.defaults.dc_pv_array.dynamic_model,
        in_service: pv.in_service !== false,
      }, dcBusCompMap);
    });

    // AC static generators
    jsonSys.ac?.static_generators?.forEach(sg => {
      addDeviceAtBus('static_generator', sg.bus, {
        ...COMP.defaults.static_generator,
        index: sg.index,
        name: sg.name || 'SGen',
        p_mw: sg.p_mw, q_mvar: sg.q_mvar,
        sgen_type: sg.sgen_type || 'PV',
        p_rated_mw: sg.p_rated_mw, sn_mva: sg.sn_mva,
        pmax_mw: sg.pmax_mw, pmin_mw: sg.pmin_mw,
        qmax_mvar: sg.qmax_mvar, qmin_mvar: sg.qmin_mvar,
        scaling: sg.scaling,
        controllable: sg.controllable || false,
        v_ref_pu: sg.v_ref_pu,
        emission_factor_tco2_mwh: sg.emission_factor_tco2_mwh || sg.co2_emission_rate || 0,
        dynamic_model: cloneDynamicModel(sg.dynamic_model) || COMP.defaults.static_generator.dynamic_model,
        in_service: sg.in_service !== false,
      }, busCompMap);
    });

    // Switches
    jsonSys.ac?.switches?.forEach(sw => {
      const fromCompId = busCompMap[sw.bus_from];
      const toCompId = busCompMap[sw.bus_to];
      if (fromCompId === undefined || toCompId === undefined) return;
      const fromComp = getComponent(fromCompId);
      const toComp = getComponent(toCompId);
      if (!fromComp || !toComp) return;
      const mx = (fromComp.x + toComp.x) / 2;
      const my = (fromComp.y + toComp.y) / 2;
      const comp = addComponent('switch_comp', mx, my, {
        ...COMP.defaults.switch_comp,
        index: sw.index,
        switch_type: sw.switch_type || '',
        closed: sw.closed !== false,
        r_contact_ohm: sw.r_contact_ohm,
        z_ohm: sw.z_ohm,
        i_rated_ka: sw.i_rated_ka,
        i_breaking_ka: sw.i_breaking_ka,
        in_service: sw.in_service !== false,
      });
      addConnection(comp.id, 'left', fromCompId, 'right');
      addConnection(comp.id, 'right', toCompId, 'left');
    });

    // Circuit breakers
    jsonSys.ac?.circuit_breakers?.forEach(cb => {
      const fromCompId = busCompMap[cb.bus_from];
      const toCompId = busCompMap[cb.bus_to];
      if (fromCompId === undefined || toCompId === undefined) return;
      const fromComp = getComponent(fromCompId);
      const toComp = getComponent(toCompId);
      if (!fromComp || !toComp) return;
      const mx = (fromComp.x + toComp.x) / 2;
      const my = (fromComp.y + toComp.y) / 2;
      const comp = addComponent('circuit_breaker', mx, my, {
        ...COMP.defaults.circuit_breaker,
        index: cb.index,
        breaker_type: cb.breaker_type || '',
        closed: cb.closed !== false,
        z_ohm: cb.z_ohm,
        rated_voltage_kv: cb.rated_voltage_kv,
        i_rated_ka: cb.i_rated_ka,
        i_breaking_ka: cb.i_breaking_ka,
        rated_current_ka: cb.rated_current_ka,
        in_service: cb.in_service !== false,
      });
      addConnection(comp.id, 'left', fromCompId, 'right');
      addConnection(comp.id, 'right', toCompId, 'left');
    });

    // DC circuit breakers use the same drawn breaker component but connect to
    // DC buses; buildSystemJson classifies them back into dc.dc_circuit_breakers.
    jsonSys.dc?.dc_circuit_breakers?.forEach(cb => {
      const fromCompId = dcBusCompMap[cb.bus_from];
      const toCompId = dcBusCompMap[cb.bus_to];
      if (fromCompId === undefined || toCompId === undefined) return;
      const fromComp = getComponent(fromCompId);
      const toComp = getComponent(toCompId);
      if (!fromComp || !toComp) return;
      const mx = (fromComp.x + toComp.x) / 2;
      const my = (fromComp.y + toComp.y) / 2;
      const comp = addComponent('circuit_breaker', mx, my, {
        ...COMP.defaults.circuit_breaker,
        index: cb.index,
        breaker_type: cb.breaker_type || '',
        closed: cb.closed !== false,
        r_ohm: cb.r_ohm,
        z_ohm: cb.r_ohm,
        rated_voltage_kv: cb.rated_voltage_kv,
        i_rated_ka: cb.i_rated_ka,
        i_breaking_ka: cb.i_breaking_ka,
        rated_current_ka: cb.rated_current_ka ?? cb.i_rated_ka,
        in_service: cb.in_service !== false,
      });
      addConnection(comp.id, 'left', fromCompId, 'right');
      addConnection(comp.id, 'right', toCompId, 'left');
    });

    // Motors
    jsonSys.ac?.motors?.forEach(m => {
      addDeviceAtBus('motor', m.bus, {
        ...COMP.defaults.motor,
        name: 'Motor',
        vn_kv: m.vn_kv, sn_mva: m.sn_mva,
        r_pu: m.r_pu, x_pu: m.x_pu,
        x_r: m.x_r, lrc: m.lrc, poles: m.poles,
        cos_phi: m.cos_phi, efficiency: m.efficiency,
        r0_pu: m.r0_pu, x0_pu: m.x0_pu,
        in_service: m.in_service !== false,
      }, busCompMap);
    });

    // Flexible loads
    jsonSys.ac?.flexible_loads?.forEach(fl => {
      addDeviceAtBus('flexible_load', fl.bus, {
        ...COMP.defaults.flexible_load,
        name: 'Flex Load',
        p_mw: fl.p_mw, q_mvar: fl.q_mvar,
        flex_up_mw: fl.flex_up_mw, flex_down_mw: fl.flex_down_mw,
        flex_duration_h: fl.flex_duration_h,
        response_time_s: fl.response_time_s,
        ramp_rate_mw_min: fl.ramp_rate_mw_min,
        availability_pct: fl.availability_pct,
        controllable: fl.controllable,
        priority: fl.priority || 'Medium',
        control_area: fl.control_area || '',
        in_service: fl.in_service !== false,
      }, busCompMap);
    });

    // Asymmetric loads
    jsonSys.ac?.asymmetric_loads?.forEach(al => {
      addDeviceAtBus('asymmetric_load', al.bus, {
        ...COMP.defaults.asymmetric_load,
        name: 'Asym Load',
        connection: al.connection || 'wye',
        grounded: al.grounded,
        pa_mw: al.pa_mw, qa_mvar: al.qa_mvar,
        pb_mw: al.pb_mw, qb_mvar: al.qb_mvar,
        pc_mw: al.pc_mw, qc_mvar: al.qc_mvar,
        scaling: al.scaling,
        const_z_percent: al.const_z_percent,
        const_i_percent: al.const_i_percent,
        const_p_percent: al.const_p_percent,
        controllable: al.controllable || false,
        priority: al.priority || 'Medium',
        dynamic_model: cloneDynamicModel(al.dynamic_model) || COMP.defaults.asymmetric_load.dynamic_model,
        in_service: al.in_service !== false,
      }, busCompMap);
    });

    // Shunts
    jsonSys.ac?.shunts?.forEach(sh => {
      addDeviceAtBus('shunt', sh.bus, {
        ...COMP.defaults.shunt,
        name: 'Shunt',
        gs_mw: sh.gs_mw, bs_mvar: sh.bs_mvar,
        switchable: sh.switchable || false,
        n_steps: sh.n_steps,
        current_step: sh.current_step,
        bs_per_step: sh.bs_per_step,
        in_service: sh.in_service !== false,
      }, busCompMap);
    });

    // 3W Transformers
    jsonSys.ac?.transformers_3w?.forEach(tr => {
      const hvCompId = busCompMap[tr.hv_bus];
      const mvCompId = busCompMap[tr.mv_bus];
      const lvCompId = busCompMap[tr.lv_bus];
      const hvComp = hvCompId !== undefined ? getComponent(hvCompId) : null;
      const x = hvComp ? hvComp.x + 60 : 400;
      const y = hvComp ? hvComp.y : 300;
      const comp = addComponent('transformer_3w', x, y, {
        ...COMP.defaults.transformer_3w,
        name: 'Trafo3W',
        sn_hv_mva: tr.sn_hv_mva, vn_hv_kv: tr.vn_hv_kv,
        sn_mv_mva: tr.sn_mv_mva, vn_mv_kv: tr.vn_mv_kv,
        sn_lv_mva: tr.sn_lv_mva, vn_lv_kv: tr.vn_lv_kv,
        vk_hv_mv_percent: tr.vk_hv_mv_percent,
        vk_hv_lv_percent: tr.vk_hv_lv_percent,
        vk_mv_lv_percent: tr.vk_mv_lv_percent,
        vkr_hv_mv_percent: tr.vkr_hv_mv_percent,
        vkr_hv_lv_percent: tr.vkr_hv_lv_percent,
        vkr_mv_lv_percent: tr.vkr_mv_lv_percent,
        pfe_kw: tr.pfe_kw, i0_percent: tr.i0_percent,
        tap_side: tr.tap_side,
        tap_pos: tr.tap_pos,
        tap_step_percent: tr.tap_step_percent,
        shift_mv_deg: tr.shift_mv_deg,
        shift_lv_deg: tr.shift_lv_deg,
        in_service: tr.in_service !== false,
      });
      if (hvCompId !== undefined) addConnection(comp.id, 'hv', hvCompId, 'bottom');
      if (mvCompId !== undefined) addConnection(comp.id, 'mv', mvCompId, 'top');
      if (lvCompId !== undefined) addConnection(comp.id, 'lv', lvCompId, 'top');
    });

    // Chargers
    jsonSys.ac?.chargers?.forEach(ch => {
      const comp = addComponent('charger', 400, 400, {
        ...COMP.defaults.charger,
        name: 'Charger',
        station_id: ch.station_id, charger_type: ch.charger_type,
        p_rated_kw: ch.p_rated_kw,
        p_ch_max_kw: ch.p_ch_max_kw,
        p_ch_min_kw: ch.p_ch_min_kw,
        eta: ch.eta,
        v2g_capable: ch.v2g_capable || false,
        p_dis_max_kw: ch.p_dis_max_kw,
        in_service: ch.in_service !== false,
      });
    });

    // Charging stations
    jsonSys.ac?.charging_stations?.forEach(cs => {
      addDeviceAtBus('charging_station', cs.bus, {
        ...COMP.defaults.charging_station,
        name: 'EV Station',
        location: cs.location || '',
        n_fast: cs.n_fast, n_slow: cs.n_slow,
        num_chargers: cs.num_chargers,
        p_fast_max_kw: cs.p_fast_max_kw,
        p_slow_max_kw: cs.p_slow_max_kw,
        max_power_kw: cs.max_power_kw,
        simultaneity_factor: cs.simultaneity_factor,
        power_factor: cs.power_factor,
        utilization_rate: cs.utilization_rate,
        p_total_kw: cs.p_total_kw,
        q_total_kvar: cs.q_total_kvar,
        in_service: cs.in_service !== false,
      }, busCompMap);
    });

    // DCDC converters — this repo's io::to_json nests them under sys.dc
    // (the ported frontend originally read them top-level), so accept both.
    const dcdcList = (jsonSys.dcdc_converters && jsonSys.dcdc_converters.length)
      ? jsonSys.dcdc_converters
      : ((jsonSys.dc && jsonSys.dc.dcdc_converters) || []);
    dcdcList.forEach(dc => {
      const inCompId = dcBusCompMap[dc.bus_in];
      const outCompId = dcBusCompMap[dc.bus_out];
      const inComp = inCompId !== undefined ? getComponent(inCompId) : null;
      const outComp = outCompId !== undefined ? getComponent(outCompId) : null;
      const x = inComp ? (outComp ? (inComp.x + outComp.x) / 2 : inComp.x + 100) : 400;
      const y = inComp ? inComp.y : (outComp ? outComp.y : 400);
      const comp = addComponent('dcdc_converter', x, y, {
        ...COMP.defaults.dcdc_converter,
        index: dc.index,
        name: dc.name || `DCDC ${dc.index !== undefined ? dc.index : ''}`,
        bus_in: dc.bus_in, bus_out: dc.bus_out,
        control_mode: dc.control_mode || 'Voltage',
        p_ref_mw: dc.p_ref_mw, v_ref_pu: dc.v_ref_pu,
        sn_mva: dc.sn_mva,
        vn_in_kv: dc.vn_in_kv, vn_out_kv: dc.vn_out_kv,
        eta: dc.eta,
        r_eq_pu: dc.r_eq_pu,
        pmax_mw: dc.pmax_mw, pmin_mw: dc.pmin_mw,
        k_droop: dc.k_droop,
        topology: dc.topology || 'Generic',
        d_min: dc.d_min ?? 0.05,
        d_max: dc.d_max ?? 0.95,
        n_ratio: dc.n_ratio ?? 1.0,
        dynamic_model: cloneDynamicModel(dc.dynamic_model) || COMP.defaults.dcdc_converter.dynamic_model,
        in_service: dc.in_service !== false,
      });
      if (inCompId !== undefined) addConnection(comp.id, 'in', inCompId, 'right');
      if (outCompId !== undefined) addConnection(comp.id, 'out', outCompId, 'left');
    });

    // Energy routers
    jsonSys.energy_routers?.forEach(er => {
      // Extract per-port parameters from the ports array
      const portParams = {};
      if (er.ports && er.ports.length > 0) {
        er.ports.forEach((pt, idx) => {
          const pi = idx + 1;
          portParams['port' + pi + '_index'] = pt.index ?? pi;
          portParams['port' + pi + '_name'] = pt.name || `${er.name || 'ER'}_P${pt.index ?? pi}`;
          portParams['port' + pi + '_bus'] = pt.bus || 0;
          portParams['port' + pi + '_type'] = String(pt.port_type || 'AC').toUpperCase() === 'DC' ? 'DC' : 'AC';
          portParams['port' + pi + '_side'] = pt.side || 0;
          portParams['port' + pi + '_control_mode'] = pt.control_mode || 'PQ';
          portParams['port' + pi + '_p_set_mw'] = pt.p_set_mw || 0;
          portParams['port' + pi + '_q_set_mvar'] = pt.q_set_mvar || 0;
          portParams['port' + pi + '_v_set_pu'] = pt.v_set_pu || 1.0;
          portParams['port' + pi + '_eta'] = pt.eta || 0.98;
          portParams['port' + pi + '_pmax_mw'] = pt.pmax_mw || 0;
          portParams['port' + pi + '_pmin_mw'] = pt.pmin_mw || 0;
          portParams['port' + pi + '_qmax_mvar'] = pt.qmax_mvar || 0;
          portParams['port' + pi + '_qmin_mvar'] = pt.qmin_mvar || 0;
          portParams['port' + pi + '_voltage_level_kv'] = pt.voltage_level_kv || 0;
          portParams['port' + pi + '_in_service'] = pt.in_service !== false;
        });
      }
      const comp = addComponent('energy_router', 500, 400, {
        ...COMP.defaults.energy_router,
        index: er.index,
        name: er.name || 'ERouter',
        router_type: er.router_type, num_ports: er.num_ports,
        p_rated_mw: er.p_rated_mw,
        vn_ac_kv: er.vn_ac_kv, vn_dc_kv: er.vn_dc_kv,
        loss_percent: er.loss_percent,
        pmax_mw: er.pmax_mw, pmin_mw: er.pmin_mw,
        qmax_mvar: er.qmax_mvar, qmin_mvar: er.qmin_mvar,
        in_service: er.in_service !== false,
        ...portParams,
      });

      // 自动为每个端口与对应母线添加连接线
      if (er.ports && er.ports.length > 0) {
        const erPortDefs = COMP.ports.energy_router || [];
        er.ports.forEach((pt, idx) => {
          const busIdx = pt.bus;
          if (!busIdx) return;
          const portType = String(pt.port_type || 'AC').toUpperCase() === 'DC' ? 'DC' : 'AC';
          const busCompId = portType === 'DC' ? dcBusCompMap[busIdx] : busCompMap[busIdx];
          if (busCompId === undefined) return;
          // 端口ID与COMP.ports.energy_router顺序一一对应
          const erPortId = erPortDefs[idx] ? erPortDefs[idx].id : 'ac_left';
          // 优先连ac_bus的left/right/top/bottom
          const busPortId = 'left';
          addConnection(comp.id, erPortId, busCompId, busPortId);
        });
      }
    });

    // Mobile storage
    jsonSys.mobile_storage?.forEach(ms => {
      addDeviceAtBus('mobile_storage', ms.bus, {
        ...COMP.defaults.mobile_storage,
        index: ms.index,
        name: ms.name || 'Mobile ESS',
        bus: ms.bus,
        p_mw: ms.p_mw, q_mvar: ms.q_mvar,
        p_rated_mw: ms.p_rated_mw, e_rated_mwh: ms.e_rated_mwh,
        pmax_mw: ms.pmax_mw, pmin_mw: ms.pmin_mw,
        qmax_mvar: ms.qmax_mvar, qmin_mvar: ms.qmin_mvar,
        soc_init: ms.soc_init, soc_min: ms.soc_min, soc_max: ms.soc_max,
        eta_charge: ms.eta_charge, eta_discharge: ms.eta_discharge,
        is_mobile: ms.is_mobile,
        status: ms.status || 'Stationary',
        target_bus: ms.target_bus,
        in_service: ms.in_service !== false,
      }, busCompMap);
    });

    // VPPs
    jsonSys.vpps?.forEach(v => {
      const pccBus = (v.pcc_bus !== undefined ? v.pcc_bus : v.aggregation_bus) || 0;
      addDeviceAtBus('vpp', pccBus, {
        ...COMP.defaults.vpp,
        index: v.index,
        name: v.name || 'VPP',
        description: v.description || '',
        pcc_bus: pccBus,
        aggregated_gen_ids: v.aggregated_gen_ids || [],
        aggregated_storage_ids: v.aggregated_storage_ids || [],
        aggregated_load_ids: v.aggregated_load_ids || [],
        n_pv_systems: v.n_pv_systems,
        n_wind_turbines: v.n_wind_turbines,
        n_battery_systems: v.n_battery_systems,
        n_ev_chargers: v.n_ev_chargers,
        n_controllable_loads: v.n_controllable_loads,
        p_generation_sum_mw: v.p_generation_sum_mw,
        e_storage_sum_mwh: v.e_storage_sum_mwh,
        p_load_controllable_mw: v.p_load_controllable_mw,
        p_output_mw: v.p_output_mw,
        q_output_mvar: v.q_output_mvar,
        pmax_mw: v.pmax_mw, pmin_mw: v.pmin_mw,
        ramp_up_max_mw_min: v.ramp_up_max_mw_min,
        ramp_down_max_mw_min: v.ramp_down_max_mw_min,
        in_service: v.in_service !== false,
      }, busCompMap);
    });

    // Microgrids
    jsonSys.microgrids?.forEach(mg => {
      addDeviceAtBus('microgrid', mg.pcc_bus, {
        ...COMP.defaults.microgrid,
        index: mg.index,
        name: mg.name || 'MicroGrid',
        description: mg.description || '',
        pcc_bus: mg.pcc_bus,
        operating_mode: mg.operating_mode,
        islanding_capability: mg.islanding_capability || false,
        auto_reconnection: mg.auto_reconnection || false,
        p_exchange_max_mw: mg.p_exchange_max_mw,
        p_exchange_min_mw: mg.p_exchange_min_mw,
        p_import_max_mw: mg.p_import_max_mw,
        p_export_max_mw: mg.p_export_max_mw,
        p_exchange_mw: mg.p_exchange_mw,
        total_generation_mw: mg.total_generation_mw,
        total_storage_mwh: mg.total_storage_mwh,
        total_load_mw: mg.total_load_mw,
        capacity_mw: mg.capacity_mw,
        peak_load_mw: mg.peak_load_mw,
        f_set_hz: mg.f_set_hz,
        v_set_pu: mg.v_set_pu,
        k_droop: mg.k_droop,
        area: mg.area,
        in_service: mg.in_service !== false,
      }, busCompMap);
    });

    // If the JSON carries a saved canvas layout, restore positions /
    // connections / viewport exactly.  Otherwise fall back to autoLayout()
    // so legacy files still produce a usable picture.
    if (jsonSys && jsonSys._canvas) {
      applyCanvasLayout(jsonSys._canvas);
    } else {
      autoLayout();
    }
    if (typeof App !== 'undefined') App.onTopologyChanged();
  }

  // ========== Results Overlay ==========
  function showPowerFlowResults(result) {
    resultsLayer.innerHTML = '';
    if (!result) return;

    // Store last PF result for visualization mode changes
    _lastPfResult = result;
    applySolvedGeneratorDisplays(result);
    refreshSolvedGeneratorComponents();

    // Overlay voltage values on buses (tagged with data-comp-id for drag tracking)
    let dcIdx = 0;
    state.components.forEach(comp => {
      if (comp.type === 'ac_bus') {
        const idx = compToBusIndex(comp.id);
        if (idx !== null && result.vm && result.vm[idx] !== undefined) {
          const vm = result.vm[idx];
          const va = result.va ? result.va[idx] : 0;
          const color = vm < 0.95 ? '#e06c75' : vm > 1.05 ? '#d19a66' : '#98c379';
          const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          t.classList.add('result-voltage');
          t.setAttribute('x', comp.x);
          t.setAttribute('y', comp.y - 24);
          t.setAttribute('fill', color);
          t.setAttribute('data-comp-id', comp.id);
          t.textContent = `${vm.toFixed(4)}∠${(va * 180 / Math.PI).toFixed(2)}°`;
          resultsLayer.appendChild(t);
        }
      } else if (comp.type === 'dc_bus') {
        if (result.vdc && result.vdc[dcIdx] !== undefined) {
          const vdc = result.vdc[dcIdx];
          const color = vdc < 0.95 ? '#e06c75' : vdc > 1.05 ? '#d19a66' : '#56b6c2';
          const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          t.classList.add('result-voltage');
          t.setAttribute('x', comp.x);
          t.setAttribute('y', comp.y - 24);
          t.setAttribute('fill', color);
          t.setAttribute('data-comp-id', comp.id);
          t.textContent = `DC ${vdc.toFixed(4)} pu`;
          resultsLayer.appendChild(t);
        }
        dcIdx++;
      }
    });

    // Apply visualization overlay
    applyVisualizationOverlay();
  }

  let _lastPfResult = null;
  let _vizMode = 'off';

  // Color mapping for heatmap: loading_pct → color
  function loadingColor(pct) {
    // 0% → green(76,175,80), 50% → yellow(255,235,59), 100% → red(244,67,54)
    const p = Math.max(0, Math.min(pct, 150)) / 100; // clamp to [0, 1.5]
    let r, g, b;
    if (p <= 0.5) {
      const t = p * 2; // 0→1
      r = Math.round(76 + (255 - 76) * t);
      g = Math.round(175 + (235 - 175) * t);
      b = Math.round(80 + (59 - 80) * t);
    } else {
      const t = Math.min((p - 0.5) * 2, 1); // 0→1
      r = Math.round(255 + (244 - 255) * t);
      g = Math.round(235 + (67 - 235) * t);
      b = Math.round(59 + (54 - 59) * t);
    }
    return `rgb(${r},${g},${b})`;
  }

  function setVisualizationMode(mode) {
    _vizMode = mode;
    applyVisualizationOverlay();
  }

  function applyVisualizationOverlay() {
    if (_lastPfResult) {
      applySolvedGeneratorDisplays(_lastPfResult);
      refreshSolvedGeneratorComponents();
    }
    // Remove existing visualization elements (keep voltage text)
    resultsLayer.querySelectorAll('.viz-overlay').forEach(el => el.remove());
    // Remove previous radial gradient defs
    const svgEl = resultsLayer.ownerSVGElement || document.querySelector('#canvas');
    let defsEl = svgEl.querySelector('defs#vizGradDefs');
    if (defsEl) defsEl.remove();
    // Reset connection line stroke if we changed them
    state.connections.forEach(conn => {
      if (conn.el) {
        const line = conn.el.querySelector('.conn-line');
        if (line) {
          line.setAttribute('stroke', '#666');
          line.setAttribute('stroke-width', '2');
          line.style.animation = '';
        }
      }
    });
    // Hide legend
    const legend = document.getElementById('vizLegend');
    if (legend) legend.style.display = 'none';

    if (_vizMode === 'off' || !_lastPfResult) return;

    const showFlow = (_vizMode === 'flow' || _vizMode === 'both');
    const showHeat = (_vizMode === 'heatmap' || _vizMode === 'both');

    // Show legend for heatmap
    if (showHeat && legend) legend.style.display = 'block';

    // Build branch flow data lookup: branchIndex → {pf_mw, loading_pct, from, to}
    const branchData = [];
    if (_lastPfResult.geo_ac_branches) {
      _lastPfResult.geo_ac_branches.forEach(br => {
        branchData.push({
          from: br.from, to: br.to,
          pf_mw: numOr(br.pf_mw, 0),
          pt_mw: numOr(br.pt_mw, 0),
          loading_pct: numOr(br.loading_pct, 0),
          rate_mva: numOr(br.rate_mva, 0),
        });
      });
    } else if (_lastPfResult.branch_abs) {
      // Fallback: use branch_abs (no direction info)
      _lastPfResult.branch_abs.forEach((absP, i) => {
        branchData.push({ from: 0, to: 0, pf_mw: absP, pt_mw: 0, loading_pct: 0, rate_mva: 0 });
      });
    }

    // Maps for lookup — match branch <-> connection via bus indices
    const busMap = getCompBusMap();
    // Invert maps: compId → busIndex
    const compToBus = {};
    for (const [idx, cid] of Object.entries(busMap.ac)) compToBus[cid] = parseInt(idx);
    for (const [idx, cid] of Object.entries(busMap.dc)) compToBus[cid] = parseInt(idx);

    const supplyTypes = new Set(['generator', 'pv_system', 'renewable_gen', 'static_generator', 'external_grid', 'dc_pv_array', 'vpp', 'microgrid']);
    const demandTypes = new Set(['load', 'dc_load', 'flexible_load', 'asymmetric_load', 'motor', 'charging_station', 'charger']);
    const bidirTypes = new Set(['storage', 'dc_storage', 'mobile_storage']);

    function getComponentBusConnections(comp, allowedBusTypes) {
      const busConns = [];
      const allowed = allowedBusTypes || new Set(['ac_bus', 'dc_bus']);
      for (const conn of state.connections) {
        let busCompId = null, portOfComp = null;
        if (conn.from.compId === comp.id) {
          busCompId = conn.to.compId;
          portOfComp = conn.from.portId;
        } else if (conn.to.compId === comp.id) {
          busCompId = conn.from.compId;
          portOfComp = conn.to.portId;
        }
        if (busCompId === null) continue;
        const busComp = getComponent(busCompId);
        if (!busComp || !allowed.has(busComp.type)) continue;
        busConns.push({
          conn,
          busCompId,
          busIdx: compToBus[busCompId],
          busType: busComp.type,
          portOfComp,
        });
      }
      return busConns;
    }

    function firstBusConnection(comp) {
      const conns = getComponentBusConnections(comp);
      return conns.length ? conns[0] : null;
    }

    function firstNonEmptyArray(...values) {
      for (const value of values) {
        if (Array.isArray(value) && value.length > 0) return value;
      }
      return [];
    }

    const dcStorageComps = state.components.filter(comp => comp.type === 'dc_storage');
    const solvedDcStorageRows = resultRowsByIndexOrOrder(_lastPfResult.dc_storage_results || [], dcStorageComps);
    const solvedDcStorageById = {};
    dcStorageComps.forEach((comp, i) => {
      if (solvedDcStorageRows[i]) solvedDcStorageById[comp.id] = solvedDcStorageRows[i];
    });
    const solvedDcStoragePowerMW = (comp) => {
      const row = solvedDcStorageById[comp.id];
      if (!row) return null;
      const value = Number(row.p_mw);
      return Number.isFinite(value) ? value : null;
    };

    function readOperatingPowerMW(comp, busCompId) {
      const p = comp.params || {};
      if (p.in_service === false || p.in_service === 'false') return 0;
      if (comp.type === 'generator') {
        const genData = (_lastPfResult.geo_gen || []);
        const compIndex = Number(p.index);
        const busIdx = compToBus[busCompId];
        let gd = Number.isFinite(compIndex)
          ? genData.find(g => Number(g.index) === compIndex)
          : null;
        if (!gd && busIdx !== undefined) gd = genData.find(g => g.bus === busIdx);
        return gd ? numOr(gd.pg_mw, 0) : numOr(p.pg_mw, 0);
      }
      if (comp.type === 'asymmetric_load') {
        return numOr(p.pa_mw, 0) + numOr(p.pb_mw, 0) + numOr(p.pc_mw, 0);
      }
      if (comp.type === 'dc_storage') {
        const solved = solvedDcStoragePowerMW(comp);
        if (solved !== null) return solved;
      }
      const currentPowerFields = {
        vpp: ['p_output_mw', 'p_mw'],
        microgrid: ['p_exchange_mw'],
        charging_station: ['p_total_kw'],
        charger: ['p_ch_kw'],
        dc_storage: ['p_mw'],
      };
      const fields = currentPowerFields[comp.type] || ['p_mw', 'p_set_mw'];
      for (const field of fields) {
        if (p[field] === undefined || p[field] === null || p[field] === '') continue;
        const value = Number(p[field]);
        if (!Number.isFinite(value)) continue;
        return field.endsWith('_kw') ? value / 1000 : value;
      }
      return 0;
    }

    // Positive value means active power leaving the bus into the attached element.
    function componentOutflowFromBusMW(comp, busCompId) {
      const powerMW = readOperatingPowerMW(comp, busCompId);
      if (supplyTypes.has(comp.type)) return -powerMW;
      if (demandTypes.has(comp.type)) return powerMW;
      if (bidirTypes.has(comp.type)) return -powerMW;
      return 0;
    }

    const knownBusOutflowMW = {};
    function addKnownBusOutflow(busCompId, powerMW) {
      if (busCompId === undefined || busCompId === null) return;
      knownBusOutflowMW[busCompId] = numOr(knownBusOutflowMW[busCompId], 0) + numOr(powerMW, 0);
    }

    state.components.forEach(comp => {
      if (!supplyTypes.has(comp.type) && !demandTypes.has(comp.type) && !bidirTypes.has(comp.type)) return;
      if (comp.type === 'external_grid') return;
      const bc = firstBusConnection(comp);
      if (!bc) return;
      addKnownBusOutflow(bc.busCompId, componentOutflowFromBusMW(comp, bc.busCompId));
    });

    (_lastPfResult.geo_ac_branches || []).forEach(br => {
      addKnownBusOutflow(busMap.ac[br.from], numOr(br.pf_mw, 0));
      addKnownBusOutflow(busMap.ac[br.to], numOr(br.pt_mw, 0));
    });
    firstNonEmptyArray(_lastPfResult.geo_dc_branches, _lastPfResult.dc_branch_flows).forEach(br => {
      const from = br.from ?? br.from_bus;
      const to = br.to ?? br.to_bus;
      addKnownBusOutflow(busMap.dc[from], numOr(br.pf_mw, 0));
      addKnownBusOutflow(busMap.dc[to], numOr(br.pt_mw, 0));
    });
    const solvedVscTransfers = firstNonEmptyArray(_lastPfResult.geo_vsc, _lastPfResult.vsc_transfers);
    solvedVscTransfers.forEach(vsc => {
      addKnownBusOutflow(busMap.ac[vsc.bus_ac], -numOr(vsc.p_ac_mw, 0));
      addKnownBusOutflow(busMap.dc[vsc.bus_dc], -numOr(vsc.p_dc_mw, 0));
    });
    const solvedDcdcTransfers = firstNonEmptyArray(_lastPfResult.geo_dcdc, _lastPfResult.dcdc_transfers);
    solvedDcdcTransfers.forEach(dcdc => {
      addKnownBusOutflow(busMap.dc[dcdc.bus_in], numOr(dcdc.p_in_mw, 0));
      addKnownBusOutflow(busMap.dc[dcdc.bus_out], -numOr(dcdc.p_out_mw, 0));
    });
    (_lastPfResult.geo_trafo3w || []).forEach(tf => {
      addKnownBusOutflow(busMap.ac[tf.hv_bus], numOr(tf.p_hv_mw, 0));
      addKnownBusOutflow(busMap.ac[tf.mv_bus], numOr(tf.p_mv_mw, 0));
      addKnownBusOutflow(busMap.ac[tf.lv_bus], numOr(tf.p_lv_mw, 0));
    });
    (_lastPfResult.geo_er || []).forEach(er => {
      (er.ports || []).forEach(pt => {
        const busCompId = pt.is_ac ? busMap.ac[pt.bus] : busMap.dc[pt.bus];
        addKnownBusOutflow(busCompId, -numOr(pt.p_mw, 0));
      });
    });

    const unresolvedTrafo2wByBus = {};
    state.components.forEach(comp => {
      if (comp.type !== 'transformer_2w' || comp.params?._from_branch) return;
      getComponentBusConnections(comp, new Set(['ac_bus'])).forEach(bc => {
        unresolvedTrafo2wByBus[bc.busCompId] = (unresolvedTrafo2wByBus[bc.busCompId] || 0) + 1;
      });
    });

    // Build branchKey → branchData index lookup
    function estimateTrafo2wSidePowerMW(busCompId) {
      const nUnknown = unresolvedTrafo2wByBus[busCompId] || 0;
      if (nUnknown <= 0) return 0;
      return -numOr(knownBusOutflowMW[busCompId], 0) / nUnknown;
    }

    function complex(re, im = 0) {
      return { re, im };
    }
    function cAdd(a, b) {
      return complex(a.re + b.re, a.im + b.im);
    }
    function cSub(a, b) {
      return complex(a.re - b.re, a.im - b.im);
    }
    function cMul(a, b) {
      return complex(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
    }
    function cDiv(a, b) {
      const den = b.re * b.re + b.im * b.im;
      if (den <= 0) return complex(0, 0);
      return complex((a.re * b.re + a.im * b.im) / den, (a.im * b.re - a.re * b.im) / den);
    }
    function cConj(a) {
      return complex(a.re, -a.im);
    }
    function cNeg(a) {
      return complex(-a.re, -a.im);
    }

    function voltagePhasor(busCompId) {
      const pos = compToBusIndex(busCompId);
      if (pos === null || !_lastPfResult.vm || !_lastPfResult.va) return null;
      const vm = Number(_lastPfResult.vm[pos]);
      const va = Number(_lastPfResult.va[pos]);
      if (!Number.isFinite(vm) || !Number.isFinite(va)) return null;
      return complex(vm * Math.cos(va), vm * Math.sin(va));
    }

    function computeTrafo2wSidesFromVoltage(comp, busConns) {
      if (!busConns || busConns.length < 2) return null;
      const p = comp.params || {};
      const hvBus = Number(p.hv_bus);
      const lvBus = Number(p.lv_bus);
      const hvSide = busConns.find(bc => bc.portOfComp === 'hv')
        || busConns.find(bc => Number.isFinite(hvBus) && bc.busIdx === hvBus)
        || busConns[0];
      const lvSide = busConns.find(bc => bc.portOfComp === 'lv')
        || busConns.find(bc => Number.isFinite(lvBus) && bc.busIdx === lvBus)
        || busConns.find(bc => bc !== hvSide);
      if (!hvSide || !lvSide || hvSide === lvSide) return null;

      const vh = voltagePhasor(hvSide.busCompId);
      const vl = voltagePhasor(lvSide.busCompId);
      if (!vh || !vl) return null;

      const baseMva = numOr(state.baseMva, 100);
      const snMva = numOr(p.sn_mva, 0);
      if (baseMva <= 1e-9 || snMva <= 1e-9) return null;
      const scale = baseMva / snMva;
      const zMag = Math.max(0, numOr(p.vk_percent, 0) / 100) * scale;
      let rPu = Math.max(0, numOr(p.vkr_percent, 0) / 100) * scale;
      let xPu = Math.sqrt(Math.max(0, zMag * zMag - rPu * rPu));
      if (rPu === 0 && xPu === 0) xPu = 1e-4;

      const rawTap = Math.max(1e-6, 1 + (numOr(p.tap_pos, 0) - numOr(p.tap_neutral, 0)) * numOr(p.tap_step_percent, 0) / 100);
      const tapSide = Number(p.tap_side);
      const impedanceScale = tapSide === 1 ? rawTap * rawTap : 1;
      rPu *= impedanceScale;
      xPu *= impedanceScale;
      const z = complex(rPu, xPu);
      const zDen = z.re * z.re + z.im * z.im;
      if (zDen <= 0) return null;
      const ys = cDiv(complex(1, 0), z);
      const ytt = ys; // Transformer2W equivalent branch has b_pu = 0.
      const tapMag = tapSide === 1 ? 1 / rawTap : rawTap;
      const shift = numOr(p.shift_deg, 0) * Math.PI / 180;
      const tap = complex(tapMag * Math.cos(shift), tapMag * Math.sin(shift));
      const tapAbs2 = tap.re * tap.re + tap.im * tap.im;
      if (tapAbs2 <= 0) return null;

      const yff = complex(ytt.re / tapAbs2, ytt.im / tapAbs2);
      const yft = cNeg(cDiv(ys, cConj(tap)));
      const ytf = cNeg(cDiv(ys, tap));
      const ih = cAdd(cMul(yff, vh), cMul(yft, vl));
      const il = cAdd(cMul(ytf, vh), cMul(ytt, vl));
      const sh = cMul(vh, cConj(ih));
      const sl = cMul(vl, cConj(il));

      return busConns.slice(0, 2).map(bc => {
        if (bc === hvSide) return { ...bc, p_mw: sh.re * baseMva };
        if (bc === lvSide) return { ...bc, p_mw: sl.re * baseMva };
        return { ...bc, p_mw: estimateTrafo2wSidePowerMW(bc.busCompId) };
      });
    }

    const estimatedTrafo2wFlows = [];
    state.components.forEach(comp => {
      if (comp.type !== 'transformer_2w' || comp.params?._from_branch) return;
      const busConns = getComponentBusConnections(comp, new Set(['ac_bus']));
      if (busConns.length < 2) return;
      const solvedSides = computeTrafo2wSidesFromVoltage(comp, busConns);
      const sides = solvedSides || busConns.slice(0, 2).map(bc => ({
        ...bc,
        p_mw: estimateTrafo2wSidePowerMW(bc.busCompId),
      }));
      if (!solvedSides && sides.length === 2) {
        const p0 = numOr(sides[0].p_mw, 0);
        const p1 = numOr(sides[1].p_mw, 0);
        if (Math.abs(p0) > FLOW_ARROW_EPS_MW && Math.abs(p1) <= FLOW_ARROW_EPS_MW) sides[1].p_mw = -p0;
        else if (Math.abs(p1) > FLOW_ARROW_EPS_MW && Math.abs(p0) <= FLOW_ARROW_EPS_MW) sides[0].p_mw = -p1;
      }
      const absPower = Math.max(...sides.map(s => Math.abs(numOr(s.p_mw, 0))), 0);
      const rateMva = numOr(comp.params?.sn_mva ?? comp.params?.rate_a_mva, 0);
      estimatedTrafo2wFlows.push({
        comp,
        sides,
        absPower,
        rate_mva: rateMva,
        loading_pct: rateMva > 0 ? 100 * absPower / rateMva : 0,
      });
    });

    const expandedSwitchFlows = [];
    const switchComps = state.components.filter(comp => comp.type === 'switch_comp');
    resultRowsByIndexOrOrder(_lastPfResult.ac_switch_flows, switchComps)
      .forEach((flow, i) => { if (flow) expandedSwitchFlows.push({ comp: switchComps[i], flow }); });
    const breakerComps = state.components.filter(comp => comp.type === 'circuit_breaker');
    const acBreakerComps = [];
    const dcBreakerComps = [];
    breakerComps.forEach(comp => {
      const busConns = getComponentBusConnections(comp);
      const isDc = busConns.length > 0 && busConns.every(bc => bc.busType === 'dc_bus');
      (isDc ? dcBreakerComps : acBreakerComps).push(comp);
    });
    resultRowsByIndexOrOrder(_lastPfResult.ac_circuit_breaker_flows, acBreakerComps)
      .forEach((flow, i) => { if (flow) expandedSwitchFlows.push({ comp: acBreakerComps[i], flow }); });
    resultRowsByIndexOrOrder(_lastPfResult.dc_circuit_breaker_flows, dcBreakerComps)
      .forEach((flow, i) => { if (flow) expandedSwitchFlows.push({ comp: dcBreakerComps[i], flow }); });

    const branchByKey = {};
    branchData.forEach((bd, i) => {
      if (bd.from && bd.to) {
        branchByKey[`${bd.from}-${bd.to}`] = i;
        branchByKey[`${bd.to}-${bd.from}`] = i; // bidirectional lookup
      }
    });

    // Compute power range for relative color normalization (include DC branches)
    const dcBranchDataAll = firstNonEmptyArray(_lastPfResult.geo_dc_branches, _lastPfResult.dc_branch_flows);
    const powers = branchData.map(bd => Math.abs(bd.pf_mw))
      .concat(dcBranchDataAll.map(bd => Math.abs(numOr(bd.pf_mw, 0))))
      .concat(estimatedTrafo2wFlows.map(tf => Math.abs(numOr(tf.absPower, 0))))
      .concat(expandedSwitchFlows.map(sw => Math.max(Math.abs(numOr(sw.flow?.pf_mw, 0)), Math.abs(numOr(sw.flow?.pt_mw, 0)))))
      .filter(v => v > 0.01);
    // Running min/max (avoid Math.min(...spread) — overflows on thousands of branches).
    let maxPower = 1, minPower = 0;
    if (powers.length) {
      maxPower = -Infinity; minPower = Infinity;
      for (const v of powers) { if (v > maxPower) maxPower = v; if (v < minPower) minPower = v; }
    }
    const powerRange = maxPower - minPower || 1;
    // Check loading availability: per-branch coloring will use loading% when rate_mva > 0,
    // and power normalization otherwise. Legend reflects the dominant mode.
    const branchesWithRate = branchData.filter(bd => bd.rate_mva > 0).length +
      dcBranchDataAll.filter(bd => numOr(bd.rate_mva, 0) > 0).length;
    const totalBranches = branchData.length + dcBranchDataAll.length;
    const allHaveLoading = totalBranches > 0 && branchesWithRate === totalBranches;
    const labeledFlowConnections = new Set();

    // For each connection that links a branch/transformer to buses, find the branch data
    // We need: connection → branch comp → two bus connections → bus indices → branch data
    let branchIdx = 0;
    const branchCompTypes = new Set(['ac_branch', 'transformer_2w']);

    // Update legend text based on coloring mode
    if (showHeat && legend) {
      const legendMin = legend.querySelector('.legend-min');
      const legendMax = legend.querySelector('.legend-max');
      if (allHaveLoading) {
        if (legendMin) legendMin.textContent = '0%';
        if (legendMax) legendMax.textContent = '100% 负载率';
      } else {
        if (legendMin) legendMin.textContent = `${pFmt(minPower, 0)} ${pUnit()}`;
        if (legendMax) legendMax.textContent = `${pFmt(maxPower, 0)} ${pUnit()}`;
      }
    }

    // Create SVG <defs> for radial gradients (one per branch)
    if (showHeat) {
      defsEl = document.createElementNS('http://www.w3.org/2000/svg', 'defs');
      defsEl.id = 'vizGradDefs';
      svgEl.insertBefore(defsEl, svgEl.firstChild);
    }

    // Collect heatmap data for radial gradient circles
    const heatItems = [];

    state.components.forEach(comp => {
      if (!branchCompTypes.has(comp.type)) return;
      // For transformer_2w with _from_branch, and ac_branch, they map to sys.ac.branches
      // For transformer_2w without _from_branch, they're in sys.ac.transformers_2w (skip)
      if (comp.type === 'transformer_2w' && !comp.params._from_branch) return;

      const bIdx = branchIdx++;
      if (bIdx >= branchData.length) return;
      const bd = branchData[bIdx];

      // Find the two connected buses via connections
      let fromBusCompId = null, toBusCompId = null;
      for (const conn of state.connections) {
        let otherCompId = null;
        if (conn.from.compId === comp.id) otherCompId = conn.to.compId;
        if (conn.to.compId === comp.id) otherCompId = conn.from.compId;
        if (otherCompId !== null) {
          const otherComp = getComponent(otherCompId);
          if (otherComp && (otherComp.type === 'ac_bus' || otherComp.type === 'dc_bus')) {
            if (fromBusCompId === null) fromBusCompId = otherCompId;
            else toBusCompId = otherCompId;
          }
        }
      }

      if (!fromBusCompId || !toBusCompId) return;

      const fromComp = getComponent(fromBusCompId);
      const toComp = getComponent(toBusCompId);
      if (!fromComp || !toComp) return;

      // Determine flow direction: pf_mw > 0 means flow from→to
      const pf_mw = bd.pf_mw;
      const pt_mw = bd.pt_mw;
      const absPower = Math.max(Math.abs(pf_mw), Math.abs(pt_mw));
      const loading = bd.loading_pct;

      // Compute normalized percentage for color mapping
      let colorPct;
      if (bd.rate_mva > 0) {
        colorPct = loading; // use actual loading % when this branch has valid rate
      } else {
        // Relative normalization based on power flow range
        colorPct = normalizedPowerPct(absPower, minPower, powerRange);
      }

      // Determine endpoints: the branch component is between its two buses
      // Draw the viz on the connections between this branch comp and its buses
      const connPairs = [];
      for (const conn of state.connections) {
        let busCompId = null, portOfBranch = null;
        if (conn.from.compId === comp.id) { busCompId = conn.to.compId; portOfBranch = conn.from.portId; }
        if (conn.to.compId === comp.id) { busCompId = conn.from.compId; portOfBranch = conn.to.portId; }
        if (busCompId === fromBusCompId || busCompId === toBusCompId) {
          connPairs.push({ conn, busCompId, portOfBranch });
        }
      }

      // Determine which bus is "from" (matching branchData.from)
      const fromBusIdx = compToBus[fromBusCompId];
      const toBusIdx = compToBus[toBusCompId];
      const isForward = (fromBusIdx === bd.from); // true if fromBusCompId matches bd.from

      // Collect heatmap data for radial glow rendering
      if (showHeat) {
        heatItems.push({ comp, colorPct, absPower, maxPower, bd, hasLoading: bd.rate_mva > 0, loading });
      }

      // Add flow direction arrows
      if (showFlow) {
        // Draw animated arrow along each connection segment
        connPairs.forEach(({ conn, busCompId }) => {
          if (!conn.el) return;
          // Pass fromCompId so g.busEnd / g.compEnd are oriented to THIS branch
          // regardless of how the user drew the wire (bus→branch or branch→bus).
          const g = getConnGeom(conn, { fromCompId: comp.id });
          if (!g) return;
          labeledFlowConnections.add(conn.id);

          // Determine arrow direction for this segment
          // If pf_mw > 0: flow goes from_bus → comp → to_bus
          // If pf_mw < 0: flow goes to_bus → comp → from_bus
          let flowsTowardBus;
          if (pf_mw >= 0) {
            // Power flows from→to; toBusCompId is the destination
            flowsTowardBus = (busCompId === toBusCompId) === isForward;
          } else {
            flowsTowardBus = (busCompId === fromBusCompId) === isForward;
          }

          // Determine which power value (Pf or Pt) to show on this side
          // from-bus side shows |Pf|, to-bus side shows |Pt|
          const isFromSide = isForward ? (busCompId === fromBusCompId) : (busCompId === toBusCompId);
          const segPower = isFromSide ? Math.abs(pf_mw) : Math.abs(pt_mw);

          // Aim the arrow at the bus end when power flows toward the bus, else at
          // the branch end — using the orientation-correct endpoints so the head
          // is right whichever way the connection was drawn (doc §10.2).
          addFlowMarker(g, flowsTowardBus ? g.busEnd : g.compEnd, segPower, showHeat ? loadingColor(colorPct) : '#1976D2');
        });
      }

      // Add loading/power label for heatmap mode
      if (showHeat && absPower > 0.01) {
        const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        label.classList.add('viz-overlay', 'heatmap-label');
        label.setAttribute('x', comp.x);
        label.setAttribute('y', comp.y + (comp.type === 'transformer_2w' ? 55 : (showFlow ? 40 : 30)));
        label.setAttribute('fill', loadingColor(colorPct));
        if (bd.rate_mva > 0) {
          label.textContent = `${loading.toFixed(1)}%`;
        } else {
          label.textContent = `${pFmt(absPower)} ${pUnit()}`;
        }
        resultsLayer.appendChild(label);
      }
    });

    // ── 3W Transformer arrows & heatmap ──
    // Each 3W transformer has 3 winding connections (HV, MV, LV).
    // Match connections to windings via bus indices from geo_trafo3w.
    // Estimated 2W transformer flows: use local bus KCL because
    // sys.ac.transformers_2w are not exposed in geo_ac_branches.
    estimatedTrafo2wFlows.forEach(tf => {
      const colorPct = tf.rate_mva > 0
        ? tf.loading_pct
        : normalizedPowerPct(tf.absPower, minPower, powerRange);

      if (showHeat) {
        heatItems.push({
          comp: tf.comp,
          colorPct,
          absPower: tf.absPower,
          maxPower,
          bd: { rate_mva: tf.rate_mva },
          hasLoading: tf.rate_mva > 0,
          loading: tf.loading_pct,
        });

        if (tf.absPower > 0.01) {
          const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          label.classList.add('viz-overlay', 'heatmap-label');
          label.setAttribute('x', tf.comp.x);
          label.setAttribute('y', tf.comp.y + 55);
          label.setAttribute('fill', loadingColor(colorPct));
          label.textContent = tf.rate_mva > 0
            ? `${tf.loading_pct.toFixed(1)}%`
            : `${pFmt(tf.absPower)} ${pUnit()}`;
          resultsLayer.appendChild(label);
        }
      }

      if (showFlow) {
        tf.sides.forEach(side => {
          if (!side.conn || !side.conn.el) return;
          const g = getConnGeom(side.conn, { fromCompId: tf.comp.id });
          if (!g) return;
          labeledFlowConnections.add(side.conn.id);
          const powerMW = numOr(side.p_mw, 0);
          const absPower = Math.abs(powerMW);
          const color = showHeat ? loadingColor(colorPct) : '#1976D2';
          addFlowMarker(g, powerMW > 0 ? g.compEnd : g.busEnd, absPower, color);
        });
      }
    });

    expandedSwitchFlows.forEach(sw => {
      const comp = sw.comp;
      const isDc = getComponentBusConnections(comp).some(bc => bc.busType === 'dc_bus');
      const busConns = getComponentBusConnections(comp, new Set([isDc ? 'dc_bus' : 'ac_bus']));
      if (busConns.length < 2) return;
      const pf_mw = numOr(sw.flow?.pf_mw, 0);
      const pt_mw = numOr(sw.flow?.pt_mw, 0);
      const absPower = Math.max(Math.abs(pf_mw), Math.abs(pt_mw));
      const rateMva = numOr(sw.flow?.rate_mva, 0);
      const loadingPct = numOr(sw.flow?.loading_pct, 0);
      const colorPct = rateMva > 0 && loadingPct > 0
        ? loadingPct
        : normalizedPowerPct(absPower, minPower, powerRange);
      const color = showHeat ? loadingColor(colorPct) : '#1976D2';

      if (showHeat && absPower > 0.01) {
        heatItems.push({
          comp,
          colorPct,
          absPower,
          maxPower,
          bd: { rate_mva: rateMva },
          hasLoading: rateMva > 0,
          loading: loadingPct,
        });
      }

      if (showFlow) {
        const fromBus = Number(sw.flow?.from ?? sw.flow?.from_bus);
        const toBus = Number(sw.flow?.to ?? sw.flow?.to_bus);
        let fromBc = busConns.find(bc => Number(bc.busIdx) === fromBus);
        let toBc = busConns.find(bc => Number(bc.busIdx) === toBus);
        if (!fromBc || !toBc) {
          [fromBc, toBc] = busConns.slice(0, 2);
        }
        [
          { bc: fromBc, isFromSide: true },
          { bc: toBc, isFromSide: false },
        ].forEach(({ bc, isFromSide }) => {
          if (!bc) return;
          if (!bc.conn || !bc.conn.el) return;
          const g = getConnGeom(bc.conn, { fromCompId: comp.id });
          if (!g) return;
          labeledFlowConnections.add(bc.conn.id);
          const sidePower = isFromSide ? pf_mw : pt_mw;
          addFlowMarker(g, sidePower >= 0 ? g.compEnd : g.busEnd, Math.abs(sidePower), color);
        });
      }
    });

    const trafo3wData = _lastPfResult.geo_trafo3w || [];
    if (trafo3wData.length > 0) {
      // Build lookup: sequential trafo3w index → geo_trafo3w entry
      const trafo3wBySeqIdx = {};
      trafo3wData.forEach((tf, i) => { trafo3wBySeqIdx[i] = tf; });

      let trafo3wSeqIdx = 0;
      state.components.forEach(comp => {
        if (comp.type !== 'transformer_3w') return;
        const seqIdx = trafo3wSeqIdx++;
        const tf = trafo3wBySeqIdx[seqIdx];
        if (!tf) return;

        // Find the 3 bus connections
        const busConns = [];
        for (const c of state.connections) {
          let otherCompId = null;
          if (c.from.compId === comp.id) otherCompId = c.to.compId;
          else if (c.to.compId === comp.id) otherCompId = c.from.compId;
          if (otherCompId !== null) {
            const otherComp = getComponent(otherCompId);
            if (otherComp && (otherComp.type === 'ac_bus' || otherComp.type === 'dc_bus')) {
              busConns.push({ conn: c, busCompId: otherCompId, busIdx: compToBus[otherCompId] });
            }
          }
        }

        // Map each connection to a winding (HV/MV/LV) by matching bus index
        const windingMap = { hv: null, mv: null, lv: null };
        busConns.forEach(bc => {
          if (bc.busIdx === tf.hv_bus) windingMap.hv = bc;
          else if (bc.busIdx === tf.mv_bus) windingMap.mv = bc;
          else if (bc.busIdx === tf.lv_bus) windingMap.lv = bc;
        });

        const windingPowers = [
          { key: 'hv', p: tf.p_hv_mw, q: tf.q_hv_mvar, bc: windingMap.hv },
          { key: 'mv', p: tf.p_mv_mw, q: tf.q_mv_mvar, bc: windingMap.mv },
          { key: 'lv', p: tf.p_lv_mw, q: tf.q_lv_mvar, bc: windingMap.lv },
        ];

        // Heatmap for 3W transformer
        if (showHeat) {
          const smax = Math.max(
            Math.hypot(tf.p_hv_mw, tf.q_hv_mvar),
            Math.hypot(tf.p_mv_mw, tf.q_mv_mvar),
            Math.hypot(tf.p_lv_mw, tf.q_lv_mvar)
          );
          let colorPct;
          if (tf.rate_mva > 0) {
            colorPct = tf.loading_pct;
          } else {
            colorPct = normalizedPowerPct(smax, minPower, powerRange);
          }
          heatItems.push({ comp, colorPct, absPower: smax, maxPower, bd: { rate_mva: tf.rate_mva }, hasLoading: tf.rate_mva > 0, loading: tf.loading_pct });

          // Loading/power label for heatmap mode
          if (smax > 0.01) {
            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.classList.add('viz-overlay', 'heatmap-label');
            label.setAttribute('x', comp.x);
            label.setAttribute('y', comp.y + 55);
            label.setAttribute('fill', loadingColor(colorPct));
            if (tf.rate_mva > 0) {
              label.textContent = `${tf.loading_pct.toFixed(1)}%`;
            } else {
              label.textContent = `${pFmt(smax)} ${pUnit()}`;
            }
            resultsLayer.appendChild(label);
          }
        }

        // Arrows along each winding connection
        if (showFlow) {
          windingPowers.forEach(({ key, p, q, bc }) => {
            if (!bc || !bc.conn || !bc.conn.el) return;
            labeledFlowConnections.add(bc.conn.id);
            const powerMW = numOr(p, 0);
            const absPower = Math.abs(powerMW);

            const g = getConnGeom(bc.conn, { fromCompId: comp.id });
            if (!g) return;

            // p > 0 means power entering transformer from this bus → arrow bus→comp
            // p < 0 means power leaving transformer to this bus → arrow comp→bus
            const fa = flowArrow(g, p > 0 ? g.compEnd : g.busEnd);
            const mx = fa.mx, my = fa.my, angle = fa.angle;

            let colorPct;
            if (tf.rate_mva > 0) colorPct = tf.loading_pct;
            else colorPct = normalizedPowerPct(absPower, minPower, powerRange);

            const color = showHeat ? loadingColor(colorPct) : '#1976D2';
            addFlowArrow(fa, absPower, color);
            addFlowLabel(mx + fa.offX, my + fa.offY, absPower, color);
          });
        }
      });
    }

    // ── Non-branch component power flow arrows & labels ──
    // Show arrows on connections for generators, loads, PV, storage, etc.
    if (showFlow) {
      // Build set of dead-island bus indices so we skip arrows for components on de-energized buses
      const deadBusSet = new Set();
      (_lastPfResult.geo_buses || []).forEach(gb => { if (gb.bus_type === 'DEAD') deadBusSet.add(gb.id); });

      const skipTypes = new Set(['ac_bus', 'dc_bus', 'ac_branch', 'dc_branch', 'transformer_2w', 'transformer_3w', 'switch_comp', 'circuit_breaker', 'shunt', 'vsc_converter', 'dcdc_converter', 'energy_router']);

      // Compute bus net injection from branch flows for external_grid power estimation
      const busInject = {};
      ((_lastPfResult.geo_ac_branches || [])
        .concat(firstNonEmptyArray(_lastPfResult.geo_dc_branches, _lastPfResult.dc_branch_flows)))
        .forEach(br => {
        if (br.pf_mw !== undefined) {
          const from = br.from ?? br.from_bus;
          const to = br.to ?? br.to_bus;
          busInject[from] = numOr(busInject[from], 0) - numOr(br.pf_mw, 0);
          busInject[to] = numOr(busInject[to], 0) - numOr(br.pt_mw, 0);
        }
      });

      const solvedVscTransfersForOverlay = firstNonEmptyArray(_lastPfResult.geo_vsc, _lastPfResult.vsc_transfers);
      const solvedDcdcTransfersForOverlay = firstNonEmptyArray(_lastPfResult.geo_dcdc, _lastPfResult.dcdc_transfers);

      // VSC transfer power at AC-side buses
      solvedVscTransfersForOverlay.forEach(vsc => {
        if (vsc.p_ac_mw !== undefined) {
          busInject[vsc.bus_ac] = numOr(busInject[vsc.bus_ac], 0) - numOr(vsc.p_ac_mw, 0);
        }
      });

      // 3W transformer winding flows contribute to bus injection
      (_lastPfResult.geo_trafo3w || []).forEach(tf => {
        busInject[tf.hv_bus] = numOr(busInject[tf.hv_bus], 0) - numOr(tf.p_hv_mw, 0);
        busInject[tf.mv_bus] = numOr(busInject[tf.mv_bus], 0) - numOr(tf.p_mv_mw, 0);
        busInject[tf.lv_bus] = numOr(busInject[tf.lv_bus], 0) - numOr(tf.p_lv_mw, 0);
      });

      state.components.forEach(comp => {
        if (skipTypes.has(comp.type)) return;
        if (!supplyTypes.has(comp.type) && !demandTypes.has(comp.type) && !bidirTypes.has(comp.type)) return;
        const p = comp.params || {};

        // Find a connection from this component to a bus
        let conn = null, busCompId = null;
        for (const c of state.connections) {
          let otherCompId = null;
          if (c.from.compId === comp.id) otherCompId = c.to.compId;
          else if (c.to.compId === comp.id) otherCompId = c.from.compId;
          if (otherCompId !== null) {
            const otherComp = getComponent(otherCompId);
            if (otherComp && (otherComp.type === 'ac_bus' || otherComp.type === 'dc_bus')) {
              conn = c;
              busCompId = otherCompId;
              break;
            }
          }
        }
        if (!conn || !conn.el || !busCompId) return;

        // Skip components on dead-island buses (no power flow)
        const attachedBusIdx = compToBus[busCompId];
        if (attachedBusIdx !== undefined && deadBusSet.has(attachedBusIdx)) return;

        let powerMW = 0;
        if (comp.type === 'external_grid') {
          // Estimate slack power from bus net injection
          const busIdx = compToBus[busCompId];
          if (busIdx !== undefined && busInject[busIdx] !== undefined) {
            powerMW = busInject[busIdx]; // net injection at bus (gen positive)
            // Add back loads at this bus
            const loadAtBus = (_lastPfResult.geo_buses || []).find(gb => gb.id === busIdx);
            if (loadAtBus) powerMW += numOr(loadAtBus.pd_mw, 0);
          }
        }
        else {
          // Use the same solved operating-power resolver as bus KCL and glyph
          // updates. This keeps generator terminal wires on PF and OPF aligned
          // with geo_gen instead of falling back to zero/nonexistent p_mw fields.
          powerMW = readOperatingPowerMW(comp, busCompId);
        }

        if (Math.abs(powerMW) < 0.001) return;

        // Determine direction
        let isSupply;
        if (comp.type === 'storage' || comp.type === 'dc_storage' || comp.type === 'mobile_storage' || comp.type === 'microgrid' || comp.type === 'vpp') isSupply = powerMW >= 0;
        else if (supplyTypes.has(comp.type)) isSupply = true;
        else if (demandTypes.has(comp.type)) isSupply = false;
        else isSupply = powerMW > 0; // storage: positive = discharge

        const g = getConnGeom(conn, { fromCompId: comp.id });
        if (!g) return;
        labeledFlowConnections.add(conn.id);

        // Arrow direction: supply → component→bus, demand → bus→component
        const fa = flowArrow(g, isSupply ? g.busEnd : g.compEnd);
        const mx = fa.mx, my = fa.my, angle = fa.angle;

        // Color by power magnitude (same scale as branch arrows)
        const absPower = Math.abs(powerMW);
        const colorPct = normalizedPowerPct(absPower, minPower, powerRange);
        const arrowColor = showHeat ? loadingColor(colorPct) : loadingColor(colorPct);

        addFlowArrow(fa, absPower, arrowColor);
        addFlowLabel(mx + fa.offX, my + fa.offY, absPower, arrowColor);
      });
    }

    // ── DC branch flow arrows ──
    if (showFlow || showHeat) {
      const dcBranchData = firstNonEmptyArray(_lastPfResult.geo_dc_branches, _lastPfResult.dc_branch_flows);
      let dcBrIdx = 0;
      state.components.forEach(comp => {
        if (comp.type !== 'dc_branch') return;
        const bd = dcBranchData[dcBrIdx++];
        if (!bd) return;
        const pf_mw_dc = numOr(bd.pf_mw, 0);
        const pt_mw_dc = numOr(bd.pt_mw, 0);
        const absPower = Math.max(Math.abs(pf_mw_dc), Math.abs(pt_mw_dc));

        // Find two connected DC buses
        let fromBusCompId = null, toBusCompId = null;
        for (const c of state.connections) {
          let otherCompId = null;
          if (c.from.compId === comp.id) otherCompId = c.to.compId;
          if (c.to.compId === comp.id) otherCompId = c.from.compId;
          if (otherCompId !== null) {
            const otherComp = getComponent(otherCompId);
            if (otherComp && otherComp.type === 'dc_bus') {
              if (!fromBusCompId) fromBusCompId = otherCompId;
              else toBusCompId = otherCompId;
            }
          }
        }
        if (!fromBusCompId || !toBusCompId) return;

        const fromBusIdx = compToBus[fromBusCompId];
        const toBusIdx = compToBus[toBusCompId];
        const isForward = (fromBusIdx === bd.from);
        const rateMva = numOr(bd.rate_mva, 0);
        const loadingPct = numOr(bd.loading_pct, 0);
        const colorPct = (rateMva > 0 && loadingPct > 0)
          ? loadingPct
          : normalizedPowerPct(absPower, minPower, powerRange);

        // Heatmap glow for DC branch
        if (showHeat) {
          heatItems.push({ comp, colorPct, absPower, maxPower, bd: { rate_mva: rateMva }, hasLoading: rateMva > 0, loading: loadingPct });
        }

        if (showFlow) {
          const connPairs = [];
          for (const c of state.connections) {
            let busCompId = null;
            if (c.from.compId === comp.id) busCompId = c.to.compId;
            if (c.to.compId === comp.id) busCompId = c.from.compId;
            if (busCompId === fromBusCompId || busCompId === toBusCompId) {
              connPairs.push({ conn: c, busCompId });
            }
          }

          connPairs.forEach(({ conn, busCompId }) => {
            if (!conn.el) return;
            // Orient endpoints to THIS branch so the arrow head is correct no
            // matter how the wire was drawn (bus→branch or branch→bus).
            const g = getConnGeom(conn, { fromCompId: comp.id });
            if (!g) return;
            labeledFlowConnections.add(conn.id);
            let flowsTowardBus;
            if (pf_mw_dc >= 0) flowsTowardBus = (busCompId === toBusCompId) === isForward;
            else flowsTowardBus = (busCompId === fromBusCompId) === isForward;

            // Determine which power value (Pf or Pt) to show on this side
            const isFromSide = isForward ? (busCompId === fromBusCompId) : (busCompId === toBusCompId);
            const segPower = isFromSide ? Math.abs(pf_mw_dc) : Math.abs(pt_mw_dc);

            addFlowMarker(g, flowsTowardBus ? g.busEnd : g.compEnd, segPower, loadingColor(colorPct));
          });
        }

        // Heatmap label for the DC branch (mirrors the AC branch label so DC
        // lines show a number in 热力图 / 方向+热力图 modes, not just a glow).
        if (showHeat && absPower > 0.01) {
          const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          label.classList.add('viz-overlay', 'heatmap-label');
          label.setAttribute('x', comp.x);
          label.setAttribute('y', comp.y + (showFlow ? 40 : 30));
          label.setAttribute('fill', loadingColor(colorPct));
          label.textContent = (rateMva > 0 && loadingPct > 0)
            ? `${loadingPct.toFixed(1)}%`
            : `${pFmt(absPower)} ${pUnit()}`;
          resultsLayer.appendChild(label);
        }
      });
    }

    // ── VSC converter flow arrows ──
    if (showFlow || showHeat) {
      const vscData = firstNonEmptyArray(_lastPfResult.geo_vsc, _lastPfResult.vsc_transfers);
      let vscIdx = 0;
      state.components.forEach(comp => {
        if (comp.type !== 'vsc_converter') return;
        const compIndex = Number(comp.params?.index);
        let vd = Number.isFinite(compIndex)
          ? vscData.find(v => Number(v.index) === compIndex)
          : null;
        if (!vd) vd = vscData[vscIdx];
        vscIdx++;
        if (!vd) return;

        // Find ALL bus connections (AC and DC)
        const busConns = [];
        for (const c of state.connections) {
          let otherCompId = null;
          if (c.from.compId === comp.id) otherCompId = c.to.compId;
          else if (c.to.compId === comp.id) otherCompId = c.from.compId;
          if (otherCompId !== null) {
            const otherComp = getComponent(otherCompId);
            if (otherComp && (otherComp.type === 'ac_bus' || otherComp.type === 'dc_bus')) {
              busConns.push({ conn: c, busCompId: otherCompId, busIdx: compToBus[otherCompId], busType: otherComp.type });
            }
          }
        }

        // Draw arrows on each connection: AC side and DC side
        const sides = [
          { p: vd.p_ac_mw, q: vd.q_ac_mvar, matchBus: vd.bus_ac, sideLabel: 'AC', expectedBusType: 'ac_bus' },
          { p: vd.p_dc_mw, q: 0, matchBus: vd.bus_dc, sideLabel: 'DC', expectedBusType: 'dc_bus' },
        ];

        sides.forEach(({ p, q, matchBus, sideLabel, expectedBusType }) => {
          const powerMW = numOr(p, 0);
          const absPower = Math.abs(powerMW);
          // Match by both bus index AND bus type to avoid AC/DC index collision
          const bc = busConns.find(bc => bc.busIdx === matchBus && bc.busType === expectedBusType);
          if (!bc || !bc.conn || !bc.conn.el) return;
          labeledFlowConnections.add(bc.conn.id);

          const g = getConnGeom(bc.conn, { fromCompId: comp.id });
          if (!g) return;

          // Bus-injection-positive: p > 0 means inject into bus → arrow comp→bus
          //                         p < 0 means draw from bus   → arrow bus→comp
          const fa = flowArrow(g, powerMW > 0 ? g.busEnd : g.compEnd);
          const mx = fa.mx, my = fa.my, angle = fa.angle;
          const colorPct = normalizedPowerPct(absPower, minPower, powerRange);

          if (showFlow) {
            const color = loadingColor(colorPct);
            addFlowArrow(fa, absPower, color);
            addFlowLabel(mx + fa.offX, my + fa.offY, absPower, color);
          }
        });

        // Heatmap glow for VSC
        if (showHeat) {
          const sMax = Math.max(Math.abs(numOr(vd.p_ac_mw, 0)), Math.abs(numOr(vd.p_dc_mw, 0)));
          const colorPct = normalizedPowerPct(sMax, minPower, powerRange);
          heatItems.push({ comp, colorPct, absPower: sMax, maxPower, bd: { rate_mva: 0 }, hasLoading: false, loading: 0 });

          // Power number label (mirrors AC/DC branches so the VSC shows a value
          // in 热力图 / 方向+热力图 modes, not just a glow).
          if (sMax > 0.01) {
            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.classList.add('viz-overlay', 'heatmap-label');
            label.setAttribute('x', comp.x);
            label.setAttribute('y', comp.y + 55);
            label.setAttribute('fill', loadingColor(colorPct));
            label.textContent = `${pFmt(sMax)} ${pUnit()}`;
            resultsLayer.appendChild(label);
          }
        }

        // Loss label for VSC
        if (showFlow && Math.abs(vd.loss_mw) > 0.001) {
          const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          label.classList.add('viz-overlay', 'flow-label');
          label.setAttribute('x', comp.x);
          label.setAttribute('y', comp.y + 40);
          label.setAttribute('fill', '#e06c75');
          label.textContent = `Loss: ${pFmt(Math.abs(vd.loss_mw), 2)} ${pUnit()}`;
          resultsLayer.appendChild(label);
        }
      });
    }

    // ── DCDC converter flow arrows ──
    if (showFlow || showHeat) {
      const dcdcData = firstNonEmptyArray(_lastPfResult.geo_dcdc, _lastPfResult.dcdc_transfers);
      let dcdcIdx = 0;
      state.components.forEach(comp => {
        if (comp.type !== 'dcdc_converter') return;
        const compIndex = Number(comp.params?.index);
        let dd = Number.isFinite(compIndex)
          ? dcdcData.find(d => Number(d.index) === compIndex)
          : null;
        if (!dd) dd = dcdcData[dcdcIdx];
        dcdcIdx++;
        if (!dd) return;

        const busConns = [];
        for (const c of state.connections) {
          let otherCompId = null;
          if (c.from.compId === comp.id) otherCompId = c.to.compId;
          else if (c.to.compId === comp.id) otherCompId = c.from.compId;
          if (otherCompId !== null) {
            const otherComp = getComponent(otherCompId);
            if (otherComp && otherComp.type === 'dc_bus') {
              busConns.push({ conn: c, busCompId: otherCompId, busIdx: compToBus[otherCompId] });
            }
          }
        }

        const sides = [
          { p: dd.p_in_mw, matchBus: dd.bus_in, label: 'in' },
          { p: dd.p_out_mw, matchBus: dd.bus_out, label: 'out' },
        ];

        sides.forEach(({ p, matchBus, label }) => {
          const powerMW = numOr(p, 0);
          const absPower = Math.abs(powerMW);
          const bc = busConns.find(bc => bc.busIdx === matchBus);
          if (!bc || !bc.conn || !bc.conn.el) return;
          labeledFlowConnections.add(bc.conn.id);

          const g = getConnGeom(bc.conn, { fromCompId: comp.id });
          if (!g) return;

          // p_in > 0: power drawn from bus_in → arrow bus→comp
          // p_out > 0: power delivered to bus_out → arrow comp→bus
          let headPt;
          if (label === 'in') headPt = powerMW > 0 ? g.compEnd : g.busEnd;
          else headPt = powerMW > 0 ? g.busEnd : g.compEnd;
          const fa = flowArrow(g, headPt);
          const mx = fa.mx, my = fa.my, angle = fa.angle;
          const colorPct = normalizedPowerPct(absPower, minPower, powerRange);

          if (showFlow) {
            const color = loadingColor(colorPct);
            addFlowArrow(fa, absPower, color);
            addFlowLabel(mx + fa.offX, my + fa.offY, absPower, color);
          }
        });

        // Heatmap glow for DCDC
        if (showHeat) {
          const sMax = Math.max(Math.abs(numOr(dd.p_in_mw, 0)), Math.abs(numOr(dd.p_out_mw, 0)));
          const colorPct = normalizedPowerPct(sMax, minPower, powerRange);
          heatItems.push({ comp, colorPct, absPower: sMax, maxPower, bd: { rate_mva: 0 }, hasLoading: false, loading: 0 });

          // Power number label (mirrors AC/DC branches so the DC/DC converter
          // shows a value in 热力图 / 方向+热力图 modes, not just a glow).
          if (sMax > 0.01) {
            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.classList.add('viz-overlay', 'heatmap-label');
            label.setAttribute('x', comp.x);
            label.setAttribute('y', comp.y + 55);
            label.setAttribute('fill', loadingColor(colorPct));
            label.textContent = `${pFmt(sMax)} ${pUnit()}`;
            resultsLayer.appendChild(label);
          }
        }

        // Loss label for DCDC
        if (showFlow && Math.abs(dd.loss_mw) > 0.001) {
          const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          label.classList.add('viz-overlay', 'flow-label');
          label.setAttribute('x', comp.x);
          label.setAttribute('y', comp.y + 40);
          label.setAttribute('fill', '#e06c75');
          label.textContent = `Loss: ${pFmt(Math.abs(dd.loss_mw), 2)} ${pUnit()}`;
          resultsLayer.appendChild(label);
        }
      });
    }

    // ── Energy Router flow arrows + heatmap ──
    if (showFlow || showHeat) {
      const erData = _lastPfResult.geo_er || [];
      // Build router_index → geo_er entry map
      const erByIdx = {};
      erData.forEach(erd => { erByIdx[erd.router_index] = erd; });

      let erCompIdx = 0;
      state.components.forEach(comp => {
        if (comp.type !== 'energy_router') return;
        // Prefer the stable router index from component params. Fallback to
        // sequential order for older canvases that do not carry the index.
        const erIdx = erCompIdx++;
        const paramIdx = Number(comp.params && comp.params.index);
        const erd = Number.isFinite(paramIdx) && erByIdx[paramIdx]
          ? erByIdx[paramIdx]
          : erData[erIdx];
        if (!erd || !erd.ports) return;

        // Find ALL bus connections from this ER
        const busConns = [];
        for (const c of state.connections) {
          let otherCompId = null;
          if (c.from.compId === comp.id) otherCompId = c.to.compId;
          else if (c.to.compId === comp.id) otherCompId = c.from.compId;
          if (otherCompId !== null) {
            const otherComp = getComponent(otherCompId);
            if (otherComp && (otherComp.type === 'ac_bus' || otherComp.type === 'dc_bus')) {
              busConns.push({ conn: c, busCompId: otherCompId, busIdx: compToBus[otherCompId], busType: otherComp.type });
            }
          }
        }

        // Draw arrows on each port connection
        erd.ports.forEach(pt => {
          const powerMW = numOr(pt.p_mw, 0);
          const absPower = Math.abs(powerMW);
          const expectedBusType = pt.is_ac ? 'ac_bus' : 'dc_bus';
          const bc = busConns.find(bc => bc.busIdx === pt.bus && bc.busType === expectedBusType);
          if (!bc || !bc.conn || !bc.conn.el) return;
          labeledFlowConnections.add(bc.conn.id);

          const g = getConnGeom(bc.conn, { fromCompId: comp.id });
          if (!g) return;

          // Bus-injection positive: p > 0 → inject into bus → arrow comp→bus
          const fa = flowArrow(g, powerMW > 0 ? g.busEnd : g.compEnd);
          const mx = fa.mx, my = fa.my, angle = fa.angle;
          const colorPct = normalizedPowerPct(absPower, minPower, powerRange);

          if (showFlow) {
            const color = loadingColor(colorPct);
            addFlowArrow(fa, absPower, color);
            addFlowLabel(mx + fa.offX, my + fa.offY, absPower, color);
          }
        });

        // Heatmap glow for Energy Router
        if (showHeat) {
          const sMax = Math.max(...erd.ports.map(pt => Math.abs(numOr(pt.p_mw, 0))), 0);
          const colorPct = normalizedPowerPct(sMax, minPower, powerRange);
          heatItems.push({ comp, colorPct, absPower: sMax, maxPower, bd: { rate_mva: 0 }, hasLoading: false, loading: 0 });

          // Power number label (mirrors AC/DC branches so the energy router
          // shows a value in 热力图 / 方向+热力图 modes, not just a glow).
          if (sMax > 0.01) {
            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.classList.add('viz-overlay', 'heatmap-label');
            label.setAttribute('x', comp.x);
            label.setAttribute('y', comp.y + 58);
            label.setAttribute('fill', loadingColor(colorPct));
            label.textContent = `${pFmt(sMax)} ${pUnit()}`;
            resultsLayer.appendChild(label);
          }
        }

        // Loss label for Energy Router
        if (showFlow && erd.loss_mw > 0.001) {
          const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          label.classList.add('viz-overlay', 'flow-label');
          label.setAttribute('x', comp.x);
          label.setAttribute('y', comp.y + 40);
          label.setAttribute('fill', '#e06c75');
          label.textContent = `Loss: ${pFmt(erd.loss_mw, 2)} ${pUnit()}`;
          resultsLayer.appendChild(label);
        }
      });
    }

    // ── Radial gradient heatmap: render glowing circles behind components ──
    // Inserted BEFORE other overlay elements so they sit underneath arrows/labels
    if (showHeat && heatItems.length > 0) {
      // Render from lowest intensity to highest so stronger glows paint on top
      heatItems.sort((a, b) => a.colorPct - b.colorPct);
      heatItems.forEach((hi, idx) => {
        const { comp, colorPct, absPower, maxPower } = hi;
        const color = loadingColor(colorPct);
        // Parse the rgb components for gradient stops
        const m = color.match(/rgb\((\d+),(\d+),(\d+)\)/);
        const [cr, cg, cb] = m ? [m[1], m[2], m[3]] : ['255', '152', '0'];

        // Create a unique radial gradient for this branch
        const gradId = `heatGrad_${idx}`;
        const grad = document.createElementNS('http://www.w3.org/2000/svg', 'radialGradient');
        grad.id = gradId;
        grad.setAttribute('cx', '50%');
        grad.setAttribute('cy', '50%');
        grad.setAttribute('r', '50%');
        // Inner stop: full color, high opacity
        const stop0 = document.createElementNS('http://www.w3.org/2000/svg', 'stop');
        stop0.setAttribute('offset', '0%');
        stop0.setAttribute('stop-color', `rgb(${cr},${cg},${cb})`);
        stop0.setAttribute('stop-opacity', '0.7');
        grad.appendChild(stop0);
        // Middle stop: color fades
        const stop1 = document.createElementNS('http://www.w3.org/2000/svg', 'stop');
        stop1.setAttribute('offset', '40%');
        stop1.setAttribute('stop-color', `rgb(${cr},${cg},${cb})`);
        stop1.setAttribute('stop-opacity', '0.35');
        grad.appendChild(stop1);
        // Outer stop: transparent
        const stop2 = document.createElementNS('http://www.w3.org/2000/svg', 'stop');
        stop2.setAttribute('offset', '100%');
        stop2.setAttribute('stop-color', `rgb(${cr},${cg},${cb})`);
        stop2.setAttribute('stop-opacity', '0');
        grad.appendChild(stop2);
        defsEl.appendChild(grad);

        // Glow radius proportional to power magnitude (min 30, max 120 SVG units)
        const radius = Math.max(30, Math.min(120, 30 + 90 * (absPower / maxPower)));
        const circ = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        circ.classList.add('viz-overlay', 'viz-heatmap-glow');
        circ.setAttribute('cx', comp.x);
        circ.setAttribute('cy', comp.y);
        circ.setAttribute('r', String(radius));
        circ.setAttribute('fill', `url(#${gradId})`);
        circ.setAttribute('stroke', 'none');
        circ.style.pointerEvents = 'none';
        // Insert glow circles at the beginning of resultsLayer so they're behind text/arrows
        resultsLayer.insertBefore(circ, resultsLayer.firstChild);
      });
    }

    // Phase 4 (doc §18): nudge overlapping flow / heatmap labels apart.  Skipped
    // for very large diagrams where the O(n²) pass would be too costly.
    // Every visible connection should carry a flow label in flow mode. If no
    // solver-backed value was matched above, show explicit zero.
    if (showFlow) {
      state.connections.forEach(conn => {
        if (!conn.el || labeledFlowConnections.has(conn.id)) return;
        const g = getConnGeom(conn);
        if (!g) return;
        addFlowLabel(g.mx - g.tdy * 14, g.my + g.tdx * 14, 0, '#777');
        labeledFlowConnections.add(conn.id);
      });
    }

    if (resultsLayer.querySelectorAll('.flow-label, .heatmap-label').length <= 400) {
      deOverlapLabels(resultsLayer);
    }
  }

  // Update result overlay positions when a component is dragged
  let _vizRafPending = false;
  function updateResultsOnDrag(compId) {
    const comp = getComponent(compId);
    if (!comp) return;
    // Move voltage text labels that belong to this component
    resultsLayer.querySelectorAll(`.result-voltage[data-comp-id="${compId}"]`).forEach(t => {
      t.setAttribute('x', comp.x);
      t.setAttribute('y', comp.y - 24);
    });
    // Throttled refresh of visualization overlay (heatmap + flow arrows)
    if (_vizMode !== 'off' && _lastPfResult && !_vizRafPending) {
      _vizRafPending = true;
      requestAnimationFrame(() => {
        _vizRafPending = false;
        applyVisualizationOverlay();
      });
    }
  }

  // ========== Topology Reconfiguration Results Overlay ==========
  // Visualise reconfiguration results on the diagram: highlight each branch's
  // connection polyline green (closed) or red-dashed (open).  Follows the routed
  // path (doc §10.2) by drawing a <path> from the cached connection geometry.
  function showTopologyReconfigResults(data) {
    resultsLayer.querySelectorAll('.topo-reconfig-overlay').forEach(el => el.remove());
    if (!data.branch_details || !Array.isArray(data.branch_details)) return;

    state.connections.forEach(conn => {
      const fromComp = getComponent(conn.from.compId);
      const toComp = getComponent(conn.to.compId);
      if (!fromComp || !toComp) return;
      // Only consider ac_branch and transformer_2w for now
      let branchComp = null;
      if (fromComp.type === 'ac_branch' || fromComp.type === 'transformer_2w') branchComp = fromComp;
      else if (toComp.type === 'ac_branch' || toComp.type === 'transformer_2w') branchComp = toComp;
      if (!branchComp) return;

      // Find branch status by matching name or params
      const branchName = branchComp.params?.name || '';
      let branchIdx = null;
      const m = branchName.match(/(Line|Trafo)\s*(\d+)/);
      if (m) branchIdx = parseInt(m[2]);
      let branchDetail = null;
      if (branchIdx !== null) branchDetail = data.branch_details.find(b => b.id == branchIdx);
      if (!branchDetail && branchComp.params?.from_bus !== undefined && branchComp.params?.to_bus !== undefined) {
        branchDetail = data.branch_details.find(b =>
          b.from_bus == branchComp.params.from_bus && b.to_bus == branchComp.params.to_bus);
      }
      if (!branchDetail) return;

      // Overlay highlight following the connection's routed polyline
      const g = getConnGeom(conn);
      if (!g) return;
      const overlay = document.createElementNS('http://www.w3.org/2000/svg', 'path');
      overlay.setAttribute('d', pointsToPath(g.points));
      overlay.setAttribute('fill', 'none');
      overlay.setAttribute('stroke-width', '7');
      overlay.setAttribute('stroke-linecap', 'round');
      overlay.setAttribute('stroke-linejoin', 'round');
      overlay.setAttribute('opacity', '0.45');
      overlay.classList.add('topo-reconfig-overlay');
      if (branchDetail.closed) {
        overlay.setAttribute('stroke', '#27ae60'); // green for closed
      } else {
        overlay.setAttribute('stroke', '#e74c3c'); // red for open
        overlay.setAttribute('stroke-dasharray', '18 10');
      }
      resultsLayer.appendChild(overlay);
    });
  }

  function clearResults() {
    resultsLayer.innerHTML = '';
    _lastPfResult = null;
    clearSolvedGeneratorDisplays();
    refreshSolvedGeneratorComponents();
    // Remove heatmap gradient defs
    const svgEl = resultsLayer.ownerSVGElement || document.querySelector('#canvas');
    const defsEl = svgEl.querySelector('defs#vizGradDefs');
    if (defsEl) defsEl.remove();
  }

  function carbonColor(value, maxValue) {
    const t = Math.max(0, Math.min(1, value / Math.max(maxValue, 1e-9)));
    const r = Math.round(46 + (224 - 46) * t);
    const g = Math.round(204 + (78 - 204) * t);
    const b = Math.round(113 + (71 - 113) * t);
    return `rgb(${r},${g},${b})`;
  }

  function showCarbonPotentialResults(data) {
    if (!resultsLayer || !data) return;
    resultsLayer.querySelectorAll('.carbon-potential-overlay').forEach(el => el.remove());

    const busMap = getCompBusMap();
    const rows = [
      ...(data.bus_carbon || []).map(b => ({ ...b, is_dc: false })),
      ...(data.dc_bus_carbon || []).map(b => ({ ...b, is_dc: true })),
    ];
    const maxIntensity = rows.reduce(
      (mx, b) => Math.max(mx, Number(b.carbon_intensity_tco2_mwh || 0)),
      0,
    );

    rows.forEach(row => {
      const busIndex = Number(row.bus_index);
      const compId = row.is_dc ? busMap.dc[busIndex] : busMap.ac[busIndex];
      const comp = getComponent(compId);
      if (!comp) return;
      const intensity = Number(row.carbon_intensity_tco2_mwh || 0);
      const color = carbonColor(intensity, maxIntensity);

      const ring = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
      ring.classList.add('carbon-potential-overlay');
      ring.setAttribute('cx', comp.x);
      ring.setAttribute('cy', comp.y);
      ring.setAttribute('r', row.is_dc ? '22' : '20');
      ring.setAttribute('fill', 'none');
      ring.setAttribute('stroke', color);
      ring.setAttribute('stroke-width', '5');
      ring.setAttribute('opacity', '0.9');
      ring.setAttribute('data-comp-id', comp.id);
      ring.style.pointerEvents = 'none';
      resultsLayer.appendChild(ring);

      const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
      label.classList.add('carbon-potential-overlay', 'result-voltage');
      label.setAttribute('x', comp.x);
      label.setAttribute('y', comp.y + 34);
      label.setAttribute('fill', color);
      label.setAttribute('data-comp-id', comp.id);
      label.textContent = `${(intensity * 1000).toFixed(1)} kg/MWh`;
      resultsLayer.appendChild(label);
    });
  }

  function clearCarbonPotentialResults() {
    if (!resultsLayer) return;
    resultsLayer.querySelectorAll('.carbon-potential-overlay').forEach(el => el.remove());
  }

  function showReliabilityImpactResults(data) {
    if (!resultsLayer || !data) return;
    resultsLayer.innerHTML = '';
    const SVGNS = 'http://www.w3.org/2000/svg';
    const maps = getCompBusMap();
    const compsById = new Map(state.components.map(c => [c.id, c]));
    const fallbackMap = {
      generator: 'gen', Generator: 'gen',
      ac_branch: 'branch', ACBranch: 'branch',
      dc_branch: 'dcBranch', DCBranch: 'dcBranch',
      vsc_converter: 'vsc', VSCConverter: 'vsc',
      static_generator: 'sgen', StaticGen: 'sgen',
      renewable_gen: 'renGen', RenewableGen: 'renGen',
      storage: 'storage', ACStorage: 'storage',
      transformer_2w: 'trafo', Transformer2W: 'trafo',
      transformer_3w: 'trafo3w', Transformer3W: 'trafo3w',
      dcdc_converter: 'dcdcConverter', DCDCConverter: 'dcdcConverter',
      dc_circuit_breaker: 'dcCb', DCCircuitBreaker: 'dcCb',
      dc_storage: 'dcStorage', DCStorage: 'dcStorage',
      dc_pv_array: 'dcPv', DCPVArray: 'dcPv',
      ac_switch: 'sw', ACSwitch: 'sw',
      ac_circuit_breaker: 'cb', ACCircuitBreaker: 'cb',
      ac_pv_system: 'pv', ACPVSystem: 'pv',
      dc_static_generator_ac: 'dcSgen', DCStaticGenAC: 'dcSgen'
    };
    const compIdFor = (row) => {
      const rawDirect = row?.canvas_comp_id ?? row?.comp_id;
      if (rawDirect !== null && rawDirect !== undefined && rawDirect !== '') {
        const direct = Number(rawDirect);
        if (Number.isInteger(direct) && compsById.has(direct)) return direct;
      }
      const bucket = row?.canvas_type || fallbackMap[row?.component_type] ||
        fallbackMap[row?.canonical_component_type];
      const modelIdx = Number(row?.canvas_index);
      if (bucket && maps[bucket] && Number.isFinite(modelIdx) &&
          maps[bucket][modelIdx] != null) {
        return maps[bucket][modelIdx];
      }
      const pos = Number(row?.component_index ?? row?.index);
      if (bucket && maps.byPosition && maps.byPosition[bucket] &&
          Number.isFinite(pos) && maps.byPosition[bucket][pos] != null) {
        return maps.byPosition[bucket][pos];
      }
      if (bucket && maps[bucket] && Number.isFinite(pos) && maps[bucket][pos] != null) {
        return maps[bucket][pos];
      }
      const primaryBus = Number(row?.primary_bus);
      if (Number.isFinite(primaryBus) && primaryBus > 0) {
        if (row?.component_domain === 'DC' && maps.dc[primaryBus] != null) return maps.dc[primaryBus];
        if (maps.ac[primaryBus] != null) return maps.ac[primaryBus];
        if (maps.dc[primaryBus] != null) return maps.dc[primaryBus];
      }
      return null;
    };
    const scoreOf = (row) => {
      const basis = data?._weak_basis || 'auto';
      const firstNumber = (...values) => {
        for (const value of values) {
          const n = Number(value);
          if (Number.isFinite(n)) return n;
        }
        return null;
      };
      if (basis === 'lole') {
        return firstNumber(row?.lole_contribution_hr_yr, row?.lole_contribution) ?? 0;
      }
      if (basis === 'frequency') {
        return firstNumber(row?.lolf_contribution_occ_yr, row?.lolf_contribution,
          row?.frequency_per_year, row?.failure_rate, row?.joint_frequency_per_year) ?? 0;
      }
      if (basis === 'conditional') {
        return firstNumber(row?.conditional_down_given_loss, row?.loss_weighted_risk,
          row?.importance) ?? 0;
      }
      if (basis === 'stage_shed') {
        const shedMw = firstNumber(row?.shed_mw, row?.total_shed_mw);
        return firstNumber(row?.pls_total, row?.shed_kw, shedMw != null ? shedMw * 1000 : null) ?? 0;
      }
      return firstNumber(row?.loss_weighted_risk, row?.associated_eens_mwh_yr,
        row?.eens_contribution_mwh_yr, row?.eens_contribution, row?.importance) ?? 0;
    };
    const sourceRows = Array.isArray(data.critical_components) && data.critical_components.length
      ? data.critical_components
      : (Array.isArray(data.contingencies) && data.contingencies.length
          ? data.contingencies
          : (Array.isArray(data.faults) ? data.faults : []));
    const riskRows = sourceRows
      .filter(row => row && scoreOf(row) > 0)
      .slice(0, 8)
      .map((row, i) => ({ row, index: i, score: scoreOf(row), comp: compsById.get(compIdFor(row)) }))
      .filter(item => item.comp);

    const nodal = Array.isArray(data.nodal_eens_mwh_yr) ? data.nodal_eens_mwh_yr : [];
    const maxEens = Math.max(0, ...nodal.map(v => Number(v) || 0));
    const nodalItems = [];
    if (maxEens > 0) {
      nodal.forEach((value, i) => {
        const eens = Number(value) || 0;
        if (eens <= 0) return;
        const busId = i + 1;
        const compId = maps.ac[busId] ?? maps.dc[busId];
        const comp = compsById.get(compId);
        if (comp) nodalItems.push({ busId, eens, comp });
      });
      nodalItems.sort((a, b) => b.eens - a.eens);
      nodalItems.forEach(item => {
        const radius = 8 + 28 * Math.sqrt(item.eens / maxEens);
        const dot = document.createElementNS(SVGNS, 'circle');
        dot.setAttribute('cx', item.comp.x);
        dot.setAttribute('cy', item.comp.y);
        dot.setAttribute('r', radius.toFixed(1));
        dot.setAttribute('fill', '#ff7a59');
        dot.setAttribute('opacity', '0.22');
        dot.setAttribute('data-comp-id', item.comp.id);
        dot.style.pointerEvents = 'none';
        dot.classList.add('reliability-impact-overlay');
        resultsLayer.insertBefore(dot, resultsLayer.firstChild);
      });
    }

    riskRows.forEach(({ row, index, score, comp }) => {
      const radius = Math.max(22, Math.min(62, 22 + Math.sqrt(Math.abs(score)) * 30));
      const ring = document.createElementNS(SVGNS, 'circle');
      ring.setAttribute('cx', comp.x);
      ring.setAttribute('cy', comp.y);
      ring.setAttribute('r', radius);
      ring.setAttribute('fill', 'none');
      ring.setAttribute('stroke', index < 3 ? '#ff6b6b' : '#f5c542');
      ring.setAttribute('stroke-width', index < 3 ? '4' : '2.5');
      ring.setAttribute('opacity', index < 3 ? '0.78' : '0.55');
      ring.setAttribute('data-comp-id', comp.id);
      ring.style.pointerEvents = 'none';
      ring.classList.add('reliability-impact-overlay');
      resultsLayer.appendChild(ring);

      const label = document.createElementNS(SVGNS, 'text');
      label.setAttribute('x', comp.x + radius + 5);
      label.setAttribute('y', comp.y - radius * 0.55);
      label.setAttribute('fill', '#ffdf80');
      label.setAttribute('font-size', '12');
      label.setAttribute('font-weight', '700');
      label.setAttribute('data-comp-id', comp.id);
      label.style.pointerEvents = 'none';
      label.classList.add('reliability-impact-overlay');
      label.textContent = `R${index + 1}`;
      resultsLayer.appendChild(label);
    });

    riskRows.slice(0, 3).forEach(({ comp }, i) => {
      nodalItems.slice(0, 4).forEach((item, j) => {
        if (item.comp.id === comp.id) return;
        const line = document.createElementNS(SVGNS, 'line');
        line.setAttribute('x1', comp.x);
        line.setAttribute('y1', comp.y);
        line.setAttribute('x2', item.comp.x);
        line.setAttribute('y2', item.comp.y);
        line.setAttribute('stroke', i === 0 ? '#ff6b6b' : '#f5c542');
        line.setAttribute('stroke-width', Math.max(1.2, 3.4 - j * 0.55).toFixed(1));
        line.setAttribute('stroke-dasharray', '8 7');
        line.setAttribute('opacity', (0.36 - j * 0.05).toFixed(2));
        line.style.pointerEvents = 'none';
        line.classList.add('reliability-impact-overlay');
        resultsLayer.insertBefore(line, resultsLayer.firstChild);
      });
    });

    if (riskRows.length || nodalItems.length) {
      const legend = document.createElementNS(SVGNS, 'g');
      legend.classList.add('reliability-impact-overlay');
      legend.style.pointerEvents = 'none';
      const x = viewBox.x + 18;
      const y = viewBox.y + 22;
      const box = document.createElementNS(SVGNS, 'rect');
      box.setAttribute('x', x);
      box.setAttribute('y', y);
      box.setAttribute('width', '230');
      box.setAttribute('height', '58');
      box.setAttribute('rx', '6');
      box.setAttribute('fill', 'rgba(22, 25, 31, 0.82)');
      box.setAttribute('stroke', '#3a3f4b');
      legend.appendChild(box);
      const labels = [
        { color: '#ff6b6b', text: 'R1-R3 最高风险元件' },
        { color: '#ff7a59', text: '负荷点 EENS 影响' },
        { color: '#f5c542', text: '虚线：传播/关联影响' }
      ];
      labels.forEach((entry, i) => {
        const cy = y + 17 + i * 16;
        const swatch = document.createElementNS(SVGNS, 'circle');
        swatch.setAttribute('cx', x + 12);
        swatch.setAttribute('cy', cy - 4);
        swatch.setAttribute('r', '4');
        swatch.setAttribute('fill', entry.color);
        legend.appendChild(swatch);
        const text = document.createElementNS(SVGNS, 'text');
        text.setAttribute('x', x + 24);
        text.setAttribute('y', cy);
        text.setAttribute('fill', '#dcdfe4');
        text.setAttribute('font-size', '11');
        text.textContent = entry.text;
        legend.appendChild(text);
      });
      resultsLayer.appendChild(legend);
    }
  }

  function compToBusIndex(compId) {
    // Reconstruct bus index from component position in list
    let idx = 0;
    for (const comp of state.components) {
      if (comp.type === 'ac_bus') {
        if (comp.id === compId) return idx;
        idx++;
      }
    }
    return null;
  }

  // ========== Clear ==========
  function clearAll() {
    state.components.forEach(c => c.el?.remove());
    state.connections.forEach(c => c.el?.remove());
    state.components = [];
    state.connections = [];
    state.selectedId = null;
    state.nextId = 1;
    // Note: baseMva is NOT reset here; loadFromSystemJson sets it before clearAll returns
    resultsLayer.innerHTML = '';
    updateInfo();
  }

  function updateInfo() {
    const el = document.getElementById('canvasInfo');
    if (el) {
      const selCount = state.selectedIds.size;
      const selText = selCount > 1 ? ` | 已选 ${selCount}` : '';
      el.textContent = `${state.components.length} 元件 | ${state.connections.length} 连接${selText}`;
    }
  }

  // ========== Pan-to-Component & Highlight ==========
  function panToComponent(id) {
    const comp = getComponent(id);
    if (!comp) return;

    // Center viewBox on the component, keep current zoom scale
    viewBox.x = comp.x - viewBox.w / 2;
    viewBox.y = comp.y - viewBox.h / 2;
    updateViewBox();

    // Select the component
    selectComponent(id);

    // Trigger highlight animation
    if (comp.el) {
      comp.el.classList.remove('highlighted');
      // Force reflow to restart animation
      void comp.el.offsetWidth;
      comp.el.classList.add('highlighted');
      comp.el.addEventListener('animationend', () => {
        comp.el.classList.remove('highlighted');
      }, { once: true });
    }
  }

  /**
   * Build a mapping from bus index (1-based) to canvas component ID.
   * Returns { ac: { busIndex -> compId }, dc: { busIndex -> compId },
   *           branch: { branchIndex -> compId }, gen: { genIndex -> compId } }
   */
  function getCompBusMap() {
	    const maps = { ac: {}, dc: {}, branch: {}, gen: {}, load: {}, trafo: {},
	      extGrid: {}, storage: {}, pv: {}, renGen: {}, sgen: {}, dcSgen: {}, sw: {}, cb: {}, dcCb: {},
	      motor: {}, dcLoad: {}, dcBranch: {}, vsc: {}, shunt: {}, trafo3w: {},
	      flexLoad: {}, asymLoad: {}, charger: {}, chargingStation: {},
	      mobileStorage: {}, dcdcConverter: {}, energyRouter: {}, vpp: {}, microgrid: {}, dcPv: {}, dcStorage: {},
	      byPosition: {} };

    const acBusIndexMap = assignBusIndices('ac_bus');
    const dcBusIndexMap = assignBusIndices('dc_bus');
    state.components.forEach(comp => {
      if (comp.type === 'ac_bus') maps.ac[acBusIndexMap[comp.id]] = comp.id;
      else if (comp.type === 'dc_bus') maps.dc[dcBusIndexMap[comp.id]] = comp.id;
    });

	    const putIndexed = (bucketName, comp, fallback) => {
	      const bucket = maps[bucketName];
	      const idx = Number(comp.params?.index);
	      bucket[Number.isFinite(idx) ? idx : fallback] = comp.id;
	      if (!maps.byPosition[bucketName]) maps.byPosition[bucketName] = {};
	      maps.byPosition[bucketName][fallback] = comp.id;
	    };

    // Must match buildSystemJson iteration order for index consistency
    const idx = { br: 0, gen: 0, load: 0, trafo: 0, eg: 0, stor: 0, pv: 0,
      ren: 0, sgen: 0, dcSgen: 0, sw: 0, cb: 0, dcCb: 0, motor: 0, dcLoad: 0, dcBr: 0, vsc: 0,
      shunt: 0, trafo3w: 0, flex: 0, asym: 0, charger: 0, cs: 0, ms: 0,
      dcdc: 0, er: 0, vpp: 0, mg: 0, dcpv: 0, dcStor: 0 };
    state.components.forEach(comp => {
      const p = comp.params;
      switch (comp.type) {
	        case 'ac_branch': putIndexed('branch', comp, idx.br++); break;
	        case 'generator': putIndexed('gen', comp, idx.gen++); break;
	        case 'load': putIndexed('load', comp, idx.load++); break;
	        case 'transformer_2w':
	          if (p._from_branch) putIndexed('branch', comp, idx.br++);
	          else putIndexed('trafo', comp, idx.trafo++);
	          break;
	        case 'external_grid': putIndexed('extGrid', comp, idx.eg++); break;
	        case 'storage': putIndexed('storage', comp, idx.stor++); break;
	        case 'pv_system': putIndexed('pv', comp, idx.pv++); break;
	        case 'dc_pv_array': putIndexed('dcPv', comp, idx.dcpv++); break;
	        case 'renewable_gen': putIndexed('renGen', comp, idx.ren++); break;
        case 'static_generator': {
          const isDcSgen = state.connections.some(conn => {
            const otherId = conn.from.compId === comp.id ? conn.to.compId
              : (conn.to.compId === comp.id ? conn.from.compId : null);
            return otherId != null && getComponent(otherId)?.type === 'dc_bus';
          });
	          if (isDcSgen) putIndexed('dcSgen', comp, idx.dcSgen++);
	          else putIndexed('sgen', comp, idx.sgen++);
	          break;
	        }
	        case 'switch_comp': putIndexed('sw', comp, idx.sw++); break;
        case 'circuit_breaker': {
          const isDcCb = state.connections.some(conn => {
            const otherId = conn.from.compId === comp.id ? conn.to.compId
              : (conn.to.compId === comp.id ? conn.from.compId : null);
            return otherId != null && getComponent(otherId)?.type === 'dc_bus';
          });
	          if (isDcCb) putIndexed('dcCb', comp, idx.dcCb++);
	          else putIndexed('cb', comp, idx.cb++);
	          break;
	        }
	        case 'motor': putIndexed('motor', comp, idx.motor++); break;
	        case 'dc_load': putIndexed('dcLoad', comp, idx.dcLoad++); break;
	        case 'dc_storage': putIndexed('dcStorage', comp, idx.dcStor++); break;
	        case 'dc_branch': putIndexed('dcBranch', comp, idx.dcBr++); break;
	        case 'vsc_converter': putIndexed('vsc', comp, idx.vsc++); break;
	        case 'shunt': putIndexed('shunt', comp, idx.shunt++); break;
	        case 'transformer_3w': putIndexed('trafo3w', comp, idx.trafo3w++); break;
	        case 'flexible_load': putIndexed('flexLoad', comp, idx.flex++); break;
	        case 'asymmetric_load': putIndexed('asymLoad', comp, idx.asym++); break;
	        case 'charger': putIndexed('charger', comp, idx.charger++); break;
	        case 'charging_station': putIndexed('chargingStation', comp, idx.cs++); break;
	        case 'mobile_storage': putIndexed('mobileStorage', comp, idx.ms++); break;
	        case 'dcdc_converter': putIndexed('dcdcConverter', comp, idx.dcdc++); break;
	        case 'energy_router': putIndexed('energyRouter', comp, idx.er++); break;
	        case 'vpp': putIndexed('vpp', comp, idx.vpp++); break;
	        case 'microgrid': putIndexed('microgrid', comp, idx.mg++); break;
      }
    });

    return maps;
  }

  // ========== Topology Analysis overlay ==========
  // Draw island halos (color per electrical island), cut-vertex warning rings,
  // and bridge-edge highlights onto the results layer.  Keyed by model bus id
  // so it matches the result tables in app.js.
  function showTopologyResults(data, opts) {
    opts = opts || {};
    const showIslands = opts.showIslands !== false;
    const showBridges = opts.showBridges !== false;
    const showCutVertices = opts.showCutVertices !== false;
    const islandColor = typeof opts.islandColor === 'function' ? opts.islandColor : () => '#61afef';

    resultsLayer.innerHTML = '';
    if (!data) return;

    const SVGNS = 'http://www.w3.org/2000/svg';
    const busMap = getCompBusMap(); // busModelId -> compId, per domain
    const acComp = (id) => (busMap.ac[id] != null ? getComponent(busMap.ac[id]) : null);
    const dcComp = (id) => (busMap.dc[id] != null ? getComponent(busMap.dc[id]) : null);

    // model bus id -> island id, domain-separated
    const acIsl = {}, dcIsl = {};
    (data.islands || []).forEach(isl => {
      (isl.ac_bus_ids || []).forEach(id => { acIsl[id] = isl.island_id; });
      (isl.dc_bus_ids || []).forEach(id => { dcIsl[id] = isl.island_id; });
    });
    // Cut vertices, domain-separated.  AC and DC buses can share the same
    // integer id, so a flat id list conflates e.g. AC bus 2 with DC bus 2 and
    // would draw a false ring on the wrong-domain twin.  Prefer the new
    // domain-tagged `cut_vertices` array; fall back to the legacy flat list
    // (applied to both domains, preserving old behaviour) when absent.
    let acCutSet, dcCutSet;
    if (Array.isArray(data.cut_vertices)) {
      acCutSet = new Set();
      dcCutSet = new Set();
      data.cut_vertices.forEach(cv => {
        (cv.domain === 'DC' ? dcCutSet : acCutSet).add(cv.bus);
      });
    } else {
      const flat = new Set(data.cut_vertex_bus_ids || []);
      acCutSet = flat;
      dcCutSet = flat;
    }

    const drawBusOverlay = (comp, islandId, isCut) => {
      if (!comp) return;
      if (showIslands && islandId !== undefined && islandId !== null && islandId >= 0) {
        const halo = document.createElementNS(SVGNS, 'circle');
        halo.setAttribute('class', 'topo-island-halo');
        halo.setAttribute('cx', comp.x);
        halo.setAttribute('cy', comp.y);
        halo.setAttribute('r', 18);
        halo.setAttribute('fill', islandColor(islandId));
        halo.setAttribute('fill-opacity', '0.28');
        halo.setAttribute('stroke', islandColor(islandId));
        halo.setAttribute('stroke-opacity', '0.9');
        halo.setAttribute('stroke-width', '2');
        resultsLayer.appendChild(halo);
      }
      if (showCutVertices && isCut) {
        const ring = document.createElementNS(SVGNS, 'circle');
        ring.setAttribute('class', 'topo-cut-vertex-ring');
        ring.setAttribute('cx', comp.x);
        ring.setAttribute('cy', comp.y);
        ring.setAttribute('r', 23);
        resultsLayer.appendChild(ring);
      }
    };

    Object.keys(acIsl).forEach(id => drawBusOverlay(acComp(+id), acIsl[id], acCutSet.has(+id)));
    Object.keys(dcIsl).forEach(id => drawBusOverlay(dcComp(+id), dcIsl[id], dcCutSet.has(+id)));
    // Cut vertices not covered by an island map (rare) still get a ring.
    if (showCutVertices) {
      acCutSet.forEach(id => {
        if (acIsl[id] === undefined) drawBusOverlay(acComp(id), undefined, true);
      });
      dcCutSet.forEach(id => {
        if (dcIsl[id] === undefined) drawBusOverlay(dcComp(id), undefined, true);
      });
    }

    // Bridge edges: dashed line between the two endpoint buses.  A bridge may
    // straddle the AC/DC boundary (a VSC/converter coupling), so each endpoint
    // is resolved in its OWN domain — not a single per-edge domain, which would
    // look up the DC endpoint in the AC component map and miss (or hit a
    // same-id AC bus).  Prefer explicit per-endpoint from_domain/to_domain from
    // the server; otherwise derive from the edge category.
    if (showBridges) {
      const DC_BOTH = { DC_Line: 1, DC_Switch: 1, DCDC_Coupling: 1 };
      (data.bridges || []).forEach(b => {
        let fromDc, toDc;
        if (b.from_domain || b.to_domain) {
          fromDc = b.from_domain === 'DC';
          toDc = b.to_domain === 'DC';
        } else if (b.category === 'VSC_Coupling') {
          // server emits from_bus = AC bus, to_bus = DC bus
          fromDc = false;
          toDc = true;
        } else if (DC_BOTH[b.category]) {
          fromDc = true;
          toDc = true;
        } else {
          fromDc = b.domain === 'DC';
          toDc = b.domain === 'DC';
        }
        const fromComp = fromDc ? dcComp(b.from_bus) : acComp(b.from_bus);
        const toComp = toDc ? dcComp(b.to_bus) : acComp(b.to_bus);
        if (!fromComp || !toComp) return;
        const line = document.createElementNS(SVGNS, 'line');
        line.setAttribute('class', 'topo-bridge-edge');
        line.setAttribute('x1', fromComp.x);
        line.setAttribute('y1', fromComp.y);
        line.setAttribute('x2', toComp.x);
        line.setAttribute('y2', toComp.y);
        resultsLayer.appendChild(line);
      });
    }
  }

  // ========== Network Reduction overlay ==========
  // Visualize the graph reduction "before/after": every bus that collapses
  // (merged / series-eliminated / pendant-folded) is drawn faded with a dashed
  // connector to its representative (surviving) bus; the representative gets a
  // colored "super-node" ring.  Keyed by canvas position (pos / rep_pos) which
  // the backend computes to match this canvas's bus ordering.
  function showNetworkReduction(data, colorFn) {
    resultsLayer.innerHTML = '';
    if (!data) return;
    const SVGNS = 'http://www.w3.org/2000/svg';
    const color = typeof colorFn === 'function' ? colorFn : () => '#56b6c2';

    const busMap = getCompBusMap(); // busModelId -> compId, per domain
    const compOf = (dc, busId) => {
      const id = dc ? busMap.dc[busId] : busMap.ac[busId];
      return id != null ? getComponent(id) : null;
    };

    // Count members per representative so single-bus "retained" groups stay plain.
    const groupSize = {};
    const tally = (arr) => (arr || []).forEach(r => {
      const k = (r.domain === 'DC' ? 'd' : 'a') + r.rep_bus_id;
      groupSize[k] = (groupSize[k] || 0) + 1;
    });
    tally(data.ac_bus_reduction);
    tally(data.dc_bus_reduction);

    const drawDomain = (arr, dc) => {
      (arr || []).forEach(r => {
        const comp = compOf(dc, r.bus_id);
        if (!comp) return;
        const repComp = compOf(dc, r.rep_bus_id);
        const key = (dc ? 'd' : 'a') + r.rep_bus_id;
        const inGroup = (groupSize[key] || 0) > 1;
        const eliminated = r.status !== 'retained';
        const col = color(r.rep_bus_id);

        if (eliminated && repComp && repComp !== comp) {
          // Dashed connector from the collapsed bus to its representative.
          const line = document.createElementNS(SVGNS, 'line');
          line.setAttribute('class', 'net-reduce-connector');
          line.setAttribute('x1', comp.x); line.setAttribute('y1', comp.y);
          line.setAttribute('x2', repComp.x); line.setAttribute('y2', repComp.y);
          line.setAttribute('stroke', col);
          resultsLayer.appendChild(line);
        }

        if (eliminated) {
          // Faded marker on the bus being removed.
          const m = document.createElementNS(SVGNS, 'circle');
          m.setAttribute('class', 'net-reduce-eliminated');
          m.setAttribute('cx', comp.x); m.setAttribute('cy', comp.y);
          m.setAttribute('r', 13);
          m.setAttribute('fill', col);
          resultsLayer.appendChild(m);
          const x = document.createElementNS(SVGNS, 'text');
          x.setAttribute('class', 'net-reduce-x');
          x.setAttribute('x', comp.x); x.setAttribute('y', comp.y);
          x.setAttribute('text-anchor', 'middle');
          x.setAttribute('dominant-baseline', 'central');
          const tag = r.status === 'merged' ? '⊝' : (r.status === 'pendant_eliminated' ? '↘' : '×');
          x.textContent = tag;
          resultsLayer.appendChild(x);
        } else if (inGroup) {
          // Surviving representative that absorbs others → super-node ring.
          const ring = document.createElementNS(SVGNS, 'circle');
          ring.setAttribute('class', 'net-reduce-super');
          ring.setAttribute('cx', comp.x); ring.setAttribute('cy', comp.y);
          ring.setAttribute('r', 22);
          ring.setAttribute('stroke', col);
          resultsLayer.appendChild(ring);
        }
      });
    };
    drawDomain(data.ac_bus_reduction, false);
    drawDomain(data.dc_bus_reduction, true);
  }

  // Pan/select the canvas to a bus by its model id (used by result-table clicks).
  function panToBusId(busId) {
    const busMap = getCompBusMap();
    const compId = busMap.ac[busId] != null ? busMap.ac[busId] : busMap.dc[busId];
    if (compId != null) panToComponent(compId);
  }

  // ========== Public API ==========
  return {
    init,
    addComponent,
    addConnection,
    removeComponent,
    removeConnection,
    removeSelected,
    getComponent,
    selectComponent,
    selectMultiple,
    clearMultiSelection,
    rerenderComponent,
    setMode,
    zoomIn,
    zoomOut,
    zoomFit,
    autoLayout,
    autoLayoutSelection,
    setConnectionStyle,
    rerouteConnections,
    setAlignSnap,
    get layoutStats() { return _layoutStats; },
    rotateSelected,
    buildSystemJson,
    syncConnectivity,
    loadFromSystemJson,
	    showPowerFlowResults,
	    showCarbonPotentialResults,
	    showReliabilityImpactResults,
	    clearCarbonPotentialResults,
    showTopologyResults,
    showNetworkReduction,
    showTopologyReconfigResults,
    clearResults,
    setVisualizationMode,
    refreshVisualization: applyVisualizationOverlay,
    clearAll,
    panToComponent,
    panToBusId,
    getCompBusMap,
    get state() { return state; },
  };
})();
