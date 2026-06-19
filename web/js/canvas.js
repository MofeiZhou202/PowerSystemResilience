  // ========== Topology Reconfiguration Results Overlay ==========
  /**
   * Visualize topology reconfiguration results on the main SVG diagram.
   * Highlights open/closed branches after reconfiguration.
   * @param {Object} data - Topology reconfiguration result data
   */
  function showTopologyReconfigResults(data) {
    // Clear previous overlays
    resultsLayer.querySelectorAll('.topo-reconfig-overlay').forEach(el => el.remove());

    // Highlight open branches (red), closed branches (green)
    if (data.branch_details && Array.isArray(data.branch_details)) {
      state.connections.forEach(conn => {
        // Find the branch component this connection belongs to
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
        // Try to extract branch index from name (e.g., 'Line 5' or 'Trafo 2')
        let branchIdx = null;
        const m = branchName.match(/(Line|Trafo)\s*(\d+)/);
        if (m) branchIdx = parseInt(m[2]);
        // Fallback: try params.from_bus/to_bus
        let branchDetail = null;
        if (branchIdx !== null) {
          branchDetail = data.branch_details.find(b => b.id == branchIdx);
        }
        if (!branchDetail && branchComp.params?.from_bus !== undefined && branchComp.params?.to_bus !== undefined) {
          branchDetail = data.branch_details.find(b => b.from_bus == branchComp.params.from_bus && b.to_bus == branchComp.params.to_bus);
        }
        if (!branchDetail) return;

        // Overlay highlight on the connection line
        const overlayLine = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        const p1 = getPortWorldPos(conn.from.compId, conn.from.portId);
        const p2 = getPortWorldPos(conn.to.compId, conn.to.portId);
        if (!p1 || !p2) return;
        overlayLine.setAttribute('x1', p1.x);
        overlayLine.setAttribute('y1', p1.y);
        overlayLine.setAttribute('x2', p2.x);
        overlayLine.setAttribute('y2', p2.y);
        overlayLine.setAttribute('stroke-width', '7');
        overlayLine.setAttribute('stroke-linecap', 'round');
        overlayLine.setAttribute('opacity', '0.45');
        overlayLine.classList.add('topo-reconfig-overlay');
        if (branchDetail.closed) {
          overlayLine.setAttribute('stroke', '#27ae60'); // green for closed
        } else {
          overlayLine.setAttribute('stroke', '#e74c3c'); // red for open
          overlayLine.setAttribute('stroke-dasharray', '18 10');
        }
        resultsLayer.appendChild(overlayLine);
      });
    }
  }

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
      showTopologyReconfigResults,
    connectStart: null,   // {compId, portId, x, y}
    tempLine: null,
    isBoxSelecting: false,
    boxSelectStart: null,  // {x, y} in SVG coords
    boxSelectRect: null,   // SVG rect element
    baseMva: 100,          // system base MVA (preserved from loaded system)
  };

  let svg, componentsLayer, connectionsLayer, resultsLayer, tempLayer;
  let viewBox = { x: -200, y: -100, w: 1200, h: 700 };

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
  }

  // ========== View ==========
  function updateViewBox() {
    svg.setAttribute('viewBox', `${viewBox.x} ${viewBox.y} ${viewBox.w} ${viewBox.h}`);
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

  function getPortWorldPos(compId, portId) {
    const comp = getComponent(compId);
    if (!comp) return null;
    const portDef = (COMP.ports[comp.type] || []).find(p => p.id === portId);
    if (!portDef) return null;
    // Apply rotation to port position
    const rad = (comp.rotation || 0) * Math.PI / 180;
    const rx = portDef.x * Math.cos(rad) - portDef.y * Math.sin(rad);
    const ry = portDef.x * Math.sin(rad) + portDef.y * Math.cos(rad);
    return { x: comp.x + rx, y: comp.y + ry };
  }

  // ========== Connections ==========
  function addConnection(fromCompId, fromPortId, toCompId, toPortId) {
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

  function renderConnection(conn) {
    const p1 = getPortWorldPos(conn.from.compId, conn.from.portId);
    const p2 = getPortWorldPos(conn.to.compId, conn.to.portId);
    if (!p1 || !p2) return;

    const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
    g.classList.add('connection');
    g.dataset.connId = conn.id;

    // Wide invisible hit area for easier clicking
    const hitLine = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    hitLine.setAttribute('x1', p1.x);
    hitLine.setAttribute('y1', p1.y);
    hitLine.setAttribute('x2', p2.x);
    hitLine.setAttribute('y2', p2.y);
    hitLine.setAttribute('stroke', 'transparent');
    hitLine.setAttribute('stroke-width', '10');
    hitLine.style.cursor = 'pointer';
    g.appendChild(hitLine);

    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.setAttribute('x1', p1.x);
    line.setAttribute('y1', p1.y);
    line.setAttribute('x2', p2.x);
    line.setAttribute('y2', p2.y);
    line.setAttribute('stroke', '#666');
    line.setAttribute('stroke-width', '2');
    line.classList.add('conn-line');
    g.appendChild(line);

    connectionsLayer.appendChild(g);
    conn.el = g;
  }

  function rerenderConnection(conn) {
    conn.el?.remove();
    renderConnection(conn);
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
      comp.el.setAttribute('transform', `translate(${comp.x}, ${comp.y}) rotate(${comp.rotation || 0})`);
      // Update connections
      state.connections.forEach(conn => {
        if (conn.from.compId === comp.id || conn.to.compId === comp.id) {
          rerenderConnection(conn);
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
          addConnection(state.connectStart.compId, state.connectStart.portId,
                        toCompId, toPortId);
          if (typeof App !== 'undefined') App.onTopologyChanged();
        }
      }
      state.tempLine.remove();
      state.tempLine = null;
      state.connectStart = null;
    }

    // Stop dragging
    if (state.isDragging) {
      state.isDragging = false;
      state.dragTarget = null;
      state.dragOffset = null;
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

  // ========== Auto Layout ==========
  function autoLayout() {
    const buses = state.components.filter(c => c.type === 'ac_bus' || c.type === 'dc_bus');
    const branchTypes = new Set(['ac_branch', 'transformer_2w', 'transformer_3w', 'dc_branch']);
    const branches = state.components.filter(c => branchTypes.has(c.type));
    const devices = state.components.filter(c => c.type !== 'ac_bus' && c.type !== 'dc_bus' && !branchTypes.has(c.type));

    if (buses.length === 0) { zoomFit(); return; }

    // Build adjacency between buses via branch-type components
    const busIdSet = new Set(buses.map(b => b.id));
    const adj = new Map(); // busCompId -> Set<busCompId>
    buses.forEach(b => adj.set(b.id, new Set()));

    branches.forEach(br => {
      const linked = [];
      state.connections.forEach(c => {
        const other = c.from.compId === br.id ? c.to.compId
                    : c.to.compId === br.id ? c.from.compId : null;
        if (other !== null && busIdSet.has(other)) linked.push(other);
      });
      for (let i = 0; i < linked.length; i++)
        for (let j = i + 1; j < linked.length; j++) {
          adj.get(linked[i]).add(linked[j]);
          adj.get(linked[j]).add(linked[i]);
        }
    });

    // BFS layering from a root bus (prefer SLACK / external-grid-connected bus)
    const busLayer = new Map();
    let root = buses[0].id;
    // Prefer bus connected to external_grid or SLACK
    for (const b of buses) {
      if (b.params.bus_type === 'SLACK' || b.params.bus_type === 'Slack') { root = b.id; break; }
      const hasEG = state.connections.some(c => {
        const otherId = c.from.compId === b.id ? c.to.compId
                      : c.to.compId === b.id ? c.from.compId : null;
        if (otherId === null) return false;
        const o = getComponent(otherId);
        return o && o.type === 'external_grid';
      });
      if (hasEG) { root = b.id; break; }
    }

    // BFS
    const queue = [root];
    busLayer.set(root, 0);
    while (queue.length > 0) {
      const cur = queue.shift();
      const layer = busLayer.get(cur);
      for (const nb of adj.get(cur) || []) {
        if (!busLayer.has(nb)) {
          busLayer.set(nb, layer + 1);
          queue.push(nb);
        }
      }
    }
    // Handle disconnected buses
    buses.forEach(b => { if (!busLayer.has(b.id)) busLayer.set(b.id, 0); });

    // Group buses by layer
    const layers = new Map(); // layer -> [busComp]
    buses.forEach(b => {
      const L = busLayer.get(b.id);
      if (!layers.has(L)) layers.set(L, []);
      layers.get(L).push(b);
    });

    // Position buses: layers top-to-bottom, buses left-to-right within layer
    const spacingX = 200, spacingY = 240;
    const sortedLayers = [...layers.keys()].sort((a, b) => a - b);
    sortedLayers.forEach(L => {
      const layerBuses = layers.get(L);
      const offsetX = -(layerBuses.length - 1) * spacingX / 2;
      layerBuses.forEach((bus, i) => {
        bus.x = offsetX + i * spacingX;
        bus.y = L * spacingY;
      });
    });

    // Force-directed refinement (only X within same layer) — 30 iterations
    for (let iter = 0; iter < 30; iter++) {
      buses.forEach(bus => {
        let fx = 0;
        const neighbors = adj.get(bus.id) || new Set();
        neighbors.forEach(nbId => {
          const nb = getComponent(nbId);
          if (!nb) return;
          const dx = nb.x - bus.x;
          fx += dx * 0.1; // attraction
        });
        // Repulsion from same-layer buses
        const layer = busLayer.get(bus.id);
        (layers.get(layer) || []).forEach(other => {
          if (other.id === bus.id) return;
          const dx = bus.x - other.x;
          const dist = Math.abs(dx) || 1;
          if (dist < spacingX) fx += Math.sign(dx) * spacingX / dist * 2;
        });
        bus.x += fx * 0.3;
      });
    }

    // Snap buses to grid
    buses.forEach(bus => {
      bus.x = snapToGrid(bus.x);
      bus.y = snapToGrid(bus.y);
      bus.el.setAttribute('transform', `translate(${bus.x}, ${bus.y}) rotate(${bus.rotation || 0})`);
    });

    // Place branch-type components at midpoint of their connected buses
    branches.forEach(br => {
      const linked = [];
      state.connections.forEach(c => {
        const other = c.from.compId === br.id ? c.to.compId
                    : c.to.compId === br.id ? c.from.compId : null;
        if (other !== null && busIdSet.has(other)) linked.push(getComponent(other));
      });
      if (linked.length >= 2 && linked[0] && linked[1]) {
        br.x = snapToGrid((linked[0].x + linked[1].x) / 2);
        br.y = snapToGrid((linked[0].y + linked[1].y) / 2 - 30);
      } else if (linked.length === 1 && linked[0]) {
        br.x = linked[0].x + 60;
        br.y = linked[0].y;
      }
      br.el.setAttribute('transform', `translate(${br.x}, ${br.y}) rotate(${br.rotation || 0})`);
    });

    // Place device components around their connected buses
    const busDeviceCount = new Map(); // busCompId -> count of devices placed
    let orphanIdx = 0;
    devices.forEach(comp => {
      const conn = state.connections.find(c =>
        c.from.compId === comp.id || c.to.compId === comp.id
      );
      if (conn) {
        const busId = conn.from.compId === comp.id ? conn.to.compId : conn.from.compId;
        const bus = getComponent(busId);
        if (bus) {
          const n = busDeviceCount.get(busId) || 0;
          busDeviceCount.set(busId, n + 1);
          // Spread devices around the bus: above for grids, below for others
          if (comp.type === 'external_grid') {
            comp.x = bus.x + (n % 3 - 1) * 70;
            comp.y = bus.y - 100;
          } else {
            const col = n % 4;
            const row = Math.floor(n / 4);
            comp.x = bus.x + (col - 1.5) * 80;
            comp.y = bus.y + 100 + row * 80;
          }
        }
      } else {
        // No connection, place in a separate area
        const maxLayer = sortedLayers.length > 0 ? sortedLayers[sortedLayers.length - 1] + 1 : 0;
        comp.x = (orphanIdx % 5) * 100;
        comp.y = maxLayer * spacingY + 100 + Math.floor(orphanIdx / 5) * 100;
        orphanIdx++;
      }
      comp.el.setAttribute('transform', `translate(${comp.x}, ${comp.y}) rotate(${comp.rotation || 0})`);
    });

    // Re-render connections
    state.connections.forEach(rerenderConnection);
    zoomFit();
  }

  // ========== Build JSON System ==========
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
      dc: { buses: [], branches: [], loads: [], static_generators: [], pv_arrays: [] },
      vsc_converters: [],
      dcdc_converters: [],
      energy_routers: [],
      mobile_storage: [],
      vpps: [],
      microgrids: []
    };

    // Assign bus indices (1-based, matching MATPOWER/C++ convention)
    let acBusIdx = 1, dcBusIdx = 1;
    const compBusMap = {}; // compId -> bus index (1-based)

    // First pass: create buses
    state.components.forEach(comp => {
      if (comp.type === 'ac_bus') {
        const p = comp.params;
        const idx = acBusIdx++;
        compBusMap[comp.id] = idx;
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
        const idx = dcBusIdx++;
        compBusMap[comp.id] = idx;
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
    function findBusIndex(compId) {
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
      return 1;  // fallback: bus 1 (1-based)
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

    // Second pass: create devices
    let genIdx = 0, brIdx = 0, vscIdx = 0, loadIdx = 0, trafoIdx = 0, egIdx = 0,
        storIdx = 0, pvIdx = 0, renIdx = 0, sgenIdx = 0, dcLoadIdx = 0, dcBrIdx = 0,
        swIdx = 0, cbIdx = 0, motorIdx = 0, dcPvIdx = 0;
    state.components.forEach(comp => {
      const p = comp.params;
      switch (comp.type) {
        case 'generator': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.generators.push({
            index: genIdx++, bus: busIdx,
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
            startup_cost: numOr(p.startup_cost, 0),
            shutdown_cost: numOr(p.shutdown_cost, 0),
            ramp_up_mw_min: numOr(p.ramp_up_mw_min, 0),
            ramp_dn_mw_min: numOr(p.ramp_dn_mw_min, 0),
          });
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
          sys.ac.loads.push({
            index: loadIdx++, bus: busIdx,
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
          });
          break;
        }
        case 'ac_branch': {
          const [from, to] = findTwoBusIndices(comp.id);
          sys.ac.branches.push({
            index: brIdx++, from_bus: from, to_bus: to,
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
          });
          break;
        }
        case 'transformer_2w': {
          const [hv, lv] = findTwoBusIndices(comp.id);
          // If this transformer was created from a MATPOWER branch (tap≠1),
          // export it back as an ac.branches entry so the solver sees it
          // with the correct branch-model parameters.
          if (p._from_branch) {
            sys.ac.branches.push({
              index: brIdx++, from_bus: hv, to_bus: lv,
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
          } else {
            sys.ac.transformers_2w.push({
              index: trafoIdx++, hv_bus: hv, lv_bus: lv,
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
            });
          }
          break;
        }
        case 'external_grid': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.external_grids.push({
            index: egIdx++, bus: busIdx,
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
            controllable: p.controllable === true || p.controllable === 'true',
            in_service: p.in_service !== false,
          });
          // Set bus as SLACK
          const bus = sys.ac.buses.find(b => b.index === busIdx);
          if (bus) bus.bus_type = 'SLACK';
          break;
        }
        case 'storage': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.storage.push({
            index: storIdx++, bus: busIdx,
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
          });
          break;
        }
        case 'pv_system': {
          const busIdx = findBusIndex(comp.id);
          const pvMode = p.control_mode || 'MPPT';
          const pvControllable = pvMode === 'Curtailed' ? true
            : (p.controllable === true || p.controllable === 'true');
          sys.ac.pv_systems.push({
            index: pvIdx++, bus: busIdx,
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
          });
          break;
        }
        case 'renewable_gen': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.renewable_gens.push({
            index: renIdx++, bus: busIdx, type: p.type || 'Wind',
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
          break;
        }
        case 'static_generator': {
          const busIdx = findBusIndex(comp.id);
          sys.ac.static_generators.push({
            index: sgenIdx++, bus: busIdx,
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
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'vsc_converter': {
          sys.vsc_converters.push({
            index: vscIdx++,
            bus_ac: parseInt(p.bus_ac) || 0,
            bus_dc: parseInt(p.bus_dc) || 0,
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
            k_vdc: numOr(p.k_vdc, 0.1),
            p_rated_mw: numOr(p.p_rated_mw, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'dc_load': {
          const busIdx = findBusIndex(comp.id);
          sys.dc.loads.push({
            index: dcLoadIdx++, bus: busIdx,
            p_mw: numOr(p.p_mw, 0),
            scaling: numOr(p.scaling, 1.0),
            controllable: p.controllable === true || p.controllable === 'true',
            p_min_mw: numOr(p.p_min_mw, 0),
            cost_mw: numOr(p.cost_mw, 0),
            profile_id: numOr(p.profile_id, -1),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'dc_branch': {
          const [from, to] = findTwoBusIndices(comp.id);
          sys.dc.branches.push({
            index: dcBrIdx++, from_bus: from, to_bus: to,
            r_pu: numOr(p.r_pu, 0.01),
            rate_a_mva: numOr(p.rate_a_mva, 200),
            length_km: numOr(p.length_km, 100),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'dc_pv_array': {
          const busIdx = findBusIndex(comp.id);
          sys.dc.pv_arrays.push({
            index: dcPvIdx++, bus: busIdx,
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
          });
          break;
        }
        // switch, circuit_breaker, motor — add similarly
        case 'switch_comp': {
          const [from, to] = findTwoBusIndices(comp.id);
          sys.ac.switches.push({
            index: swIdx++, bus_from: from, bus_to: to,
            switch_type: p.switch_type || '',
            closed: p.closed !== false,
            r_contact_ohm: numOr(p.r_contact_ohm, 0),
            z_ohm: numOr(p.z_ohm, 0),
            i_rated_ka: numOr(p.i_rated_ka, 0),
            i_breaking_ka: numOr(p.i_breaking_ka, 0),
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'circuit_breaker': {
          const [from, to] = findTwoBusIndices(comp.id);
          sys.ac.circuit_breakers.push({
            index: cbIdx++, bus_from: from, bus_to: to,
            breaker_type: p.breaker_type || '',
            closed: p.closed !== false,
            z_ohm: numOr(p.z_ohm, 0),
            rated_voltage_kv: numOr(p.rated_voltage_kv, 0),
            i_rated_ka: numOr(p.i_rated_ka, 0),
            i_breaking_ka: numOr(p.i_breaking_ka, 0),
            rated_current_ka: numOr(p.rated_current_ka, 2.0),
            in_service: p.in_service !== false,
          });
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
          sys.ac.asymmetric_loads.push({
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
          });
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
          sys.dcdc_converters.push({
            index: sys.dcdc_converters.length,
            bus_in: parseInt(p.bus_in) || 0,
            bus_out: parseInt(p.bus_out) || 0,
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
            in_service: p.in_service !== false,
          });
          break;
        }
        case 'energy_router': {
          // Build ports array from per-port parameters
          const erPorts = [];
          for (let pi = 1; pi <= 4; pi++) {
            const pBus = parseInt(p['port' + pi + '_bus']) || 0;
            if (pBus === 0) continue;  // skip ports with no bus assigned
            erPorts.push({
              index: pi,
              name: (p.name || 'ER') + '_P' + pi,
              bus: pBus,
              port_type: 'AC',
              side: parseInt(p['port' + pi + '_side']) || 0,
              control_mode: p['port' + pi + '_control_mode'] || 'PQ',
              p_set_mw: numOr(p['port' + pi + '_p_set_mw'], 0),
              q_set_mvar: numOr(p['port' + pi + '_q_set_mvar'], 0),
              v_set_pu: numOr(p['port' + pi + '_v_set_pu'], 1.0),
              eta: numOr(p['port' + pi + '_eta'], 0.98),
              voltage_level_kv: numOr(p.vn_ac_kv, 0),
              pmax_mw: numOr(p.pmax_mw, 0),
              pmin_mw: numOr(p.pmin_mw, 0),
              qmax_mvar: numOr(p.qmax_mvar, 0),
              qmin_mvar: numOr(p.qmin_mvar, 0),
              in_service: true,
            });
          }
          sys.energy_routers.push({
            index: sys.energy_routers.length,
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
            index: sys.microgrids.length,
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
    // does not destroy the user's layout.  The block is keyed by
    // "<type>#<n>" with n = per-type sequence number in state.components
    // iteration order, which matches the order loadFromSystemJson will
    // recreate components in.  The backend ignores unknown fields.
    {
      const compKey = {};
      const typeCounter = {};
      state.components.forEach(comp => {
        const t = comp.type;
        typeCounter[t] = (typeCounter[t] || 0) + 1;
        compKey[comp.id] = t + '#' + typeCounter[t];
      });
      sys._canvas = {
        version: 1,
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
          }))
          .filter(cn => cn.from.key && cn.to.key),
      };
    }

    return sys;
  }

  // ========== Apply Canvas Layout (positions/connections/viewport) ==========
  function applyCanvasLayout(layout) {
    if (!layout || !Array.isArray(layout.components)) return;

    // Rebuild key -> compId by walking the just-loaded components in the
    // same per-type order used when the layout was saved.
    const keyToCompId = {};
    const typeCounter = {};
    state.components.forEach(comp => {
      const t = comp.type;
      typeCounter[t] = (typeCounter[t] || 0) + 1;
      keyToCompId[t + '#' + typeCounter[t]] = comp.id;
    });

    // Restore position and rotation.
    layout.components.forEach(item => {
      if (!item || !item.key) return;
      const cid = keyToCompId[item.key];
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
        const fid = keyToCompId[cn.from.key];
        const tid = keyToCompId[cn.to.key];
        if (fid === undefined || tid === undefined) return;
        addConnection(fid, cn.from.port, tid, cn.to.port);
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

    // Re-render all connections so endpoints follow the new positions.
    state.connections.forEach(rerenderConnection);
  }

  // ========== Load from JSON System ==========
  function loadFromSystemJson(jsonSys) {
    // Clear canvas
    clearAll();

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
        name: `Gen ${gen.index !== undefined ? gen.index : ''}`,
        pg_mw: gen.pg_mw, qg_mvar: gen.qg_mvar,
        vg_pu: gen.vg_pu || 1.0,
        pmax_mw: gen.pmax_mw, pmin_mw: gen.pmin_mw,
        qmax_mvar: gen.qmax_mvar, qmin_mvar: gen.qmin_mvar,
        mbase_mva: gen.mbase_mva,
        is_slack: gen.is_slack || false,
        in_service: gen.in_service !== false,
        cost_c2: gen.cost_c2, cost_c1: gen.cost_c1, cost_c0: gen.cost_c0,
        startup_cost: gen.startup_cost, shutdown_cost: gen.shutdown_cost,
        ramp_up_mw_min: gen.ramp_up_mw_min, ramp_dn_mw_min: gen.ramp_dn_mw_min,
      }, busCompMap);
    });

    // Loads
    jsonSys.ac?.loads?.forEach(load => {
      addDeviceAtBus('load', load.bus, {
        ...COMP.defaults.load,
        name: `Load`,
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
        in_service: load.in_service !== false,
      }, busCompMap);
    });

    // If no explicit loads, create load components from buses with non-zero pd_mw
    if (!jsonSys.ac?.loads?.length && jsonSys.ac?.buses) {
      jsonSys.ac.buses.forEach(bus => {
        if ((bus.pd_mw && bus.pd_mw !== 0) || (bus.qd_mvar && bus.qd_mvar !== 0)) {
          addDeviceAtBus('load', bus.index, {
            ...COMP.defaults.load,
            name: `Load`,
            p_mw: bus.pd_mw || 0,
            q_mvar: bus.qd_mvar || 0,
            scaling: 1.0,
          }, busCompMap);
        }
      });
    }

    // External grids
    jsonSys.ac?.external_grids?.forEach(eg => {
      addDeviceAtBus('external_grid', eg.bus, {
        ...COMP.defaults.external_grid,
        name: 'Grid',
        vm_pu: eg.vm_pu, va_deg: eg.va_deg,
        s_sc_max_mva: eg.s_sc_max_mva,
        s_sc_min_mva: eg.s_sc_min_mva,
        rx_max: eg.rx_max, rx_min: eg.rx_min,
        r_pu: eg.r_pu, x_pu: eg.x_pu,
        r0_pu: eg.r0_pu, x0_pu: eg.x0_pu,
        vn_kv: eg.vn_kv,
        controllable: eg.controllable || false,
        in_service: eg.in_service !== false,
      }, busCompMap, -80);
    });

    // Storage
    jsonSys.ac?.storage?.forEach(s => {
      addDeviceAtBus('storage', s.bus, {
        ...COMP.defaults.storage,
        name: 'ESS',
        p_mw: s.p_mw, q_mvar: s.q_mvar,
        p_rated_mw: s.p_rated_mw, e_rated_mwh: s.e_rated_mwh,
        soc_init: s.soc_init, soc_min: s.soc_min, soc_max: s.soc_max,
        eta_charge: s.eta_charge, eta_discharge: s.eta_discharge,
        pmax_mw: s.pmax_mw, pmin_mw: s.pmin_mw,
        qmax_mvar: s.qmax_mvar, qmin_mvar: s.qmin_mvar,
        self_discharge_pct: s.self_discharge_pct,
        profile_id: s.profile_id,
        in_service: s.in_service !== false,
      }, busCompMap);
    });

    // PV Systems
    jsonSys.ac?.pv_systems?.forEach(pv => {
      addDeviceAtBus('pv_system', pv.bus, {
        ...COMP.defaults.pv_system,
        name: 'PV',
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
        in_service: pv.in_service !== false,
      }, busCompMap);
    });

    // Renewable gens
    jsonSys.ac?.renewable_gens?.forEach(rg => {
      addDeviceAtBus('renewable_gen', rg.bus, {
        ...COMP.defaults.renewable_gen,
        name: rg.type || 'Wind',
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
          name: `Trafo ${br.index !== undefined ? br.index : ''}`,
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
          name: `Line ${br.index !== undefined ? br.index : ''}`,
          from_bus: br.from_bus, to_bus: br.to_bus,
          r_pu: br.r_pu, x_pu: br.x_pu, b_pu: br.b_pu,
          rate_a_mva: br.rate_a_mva,
          rate_b_mva: br.rate_b_mva,
          rate_c_mva: br.rate_c_mva,
          length_km: br.length_km,
          tap: tapVal, shift_deg: shiftVal,
          n_parallel: br.n_parallel,
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
        name: `VSC ${vsc.index !== undefined ? vsc.index : ''}`,
        bus_ac: vsc.bus_ac, bus_dc: vsc.bus_dc,
        p_set_mw: vsc.p_set_mw, q_set_mvar: vsc.q_set_mvar,
        // Normalize control_mode: C++ uses "PQ"/"VDC_Q"/"VDC_VAC" but UI uses "PQ_MODE"
        control_mode: (vsc.control_mode === 'PQ' ? 'PQ_MODE' : vsc.control_mode) || 'PQ_MODE',
        eta: vsc.eta,
        loss_percent: vsc.loss_percent,
        loss_mw: vsc.loss_mw,
        v_dc_set_pu: vsc.v_dc_set_pu,
        v_ac_set_pu: vsc.v_ac_set_pu,
        k_vdc: vsc.k_vdc,
        pmax_mw: vsc.pmax_mw, pmin_mw: vsc.pmin_mw,
        qmax_mvar: vsc.qmax_mvar, qmin_mvar: vsc.qmin_mvar,
        p_rated_mw: vsc.p_rated_mw,
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
        name: `DC Line ${br.index !== undefined ? br.index : ''}`,
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
        name: 'DC Load',
        p_mw: ld.p_mw, scaling: ld.scaling,
        controllable: ld.controllable || false,
        p_min_mw: ld.p_min_mw,
        cost_mw: ld.cost_mw,
        profile_id: ld.profile_id,
        in_service: ld.in_service !== false,
      }, dcBusCompMap);
    });

    // DC static generators
    jsonSys.dc?.static_generators?.forEach(sg => {
      addDeviceAtBus('static_generator', sg.bus, {
        ...COMP.defaults.static_generator,
        name: 'DC SGen',
        p_mw: sg.p_mw, q_mvar: sg.q_mvar,
        sgen_type: sg.sgen_type || 'PV',
        controllable: sg.controllable || false,
        scaling: sg.scaling,
        p_rated_mw: sg.p_rated_mw, sn_mva: sg.sn_mva,
        pmax_mw: sg.pmax_mw, pmin_mw: sg.pmin_mw,
        qmax_mvar: sg.qmax_mvar, qmin_mvar: sg.qmin_mvar,
        v_ref_pu: sg.v_ref_pu,
        in_service: sg.in_service !== false,
      }, dcBusCompMap);
    });

    // DC storage
    jsonSys.dc?.storage?.forEach(s => {
      addDeviceAtBus('storage', s.bus, {
        ...COMP.defaults.storage,
        name: 'DC ESS',
        p_mw: s.p_mw, q_mvar: s.q_mvar,
        p_rated_mw: s.p_rated_mw, e_rated_mwh: s.e_rated_mwh,
        soc_init: s.soc_init, soc_min: s.soc_min, soc_max: s.soc_max,
        eta_charge: s.eta_charge, eta_discharge: s.eta_discharge,
        pmax_mw: s.pmax_mw, pmin_mw: s.pmin_mw,
        qmax_mvar: s.qmax_mvar, qmin_mvar: s.qmin_mvar,
        self_discharge_pct: s.self_discharge_pct,
        profile_id: s.profile_id,
        in_service: s.in_service !== false,
      }, dcBusCompMap);
    });

    // DC PV arrays
    jsonSys.dc?.pv_arrays?.forEach(pv => {
      addDeviceAtBus('dc_pv_array', pv.bus, {
        ...COMP.defaults.dc_pv_array,
        name: 'DC PV',
        p_set_mw: pv.p_set_mw,
        irradiance: pv.irradiance,
        temperature: pv.temperature,
        num_series: pv.num_series, num_parallel: pv.num_parallel,
        vmpp: pv.vmpp, impp: pv.impp, voc: pv.voc, isc: pv.isc,
        alpha_isc: pv.alpha_isc, beta_voc: pv.beta_voc,
        profile_id: pv.profile_id,
        in_service: pv.in_service !== false,
      }, dcBusCompMap);
    });

    // AC static generators
    jsonSys.ac?.static_generators?.forEach(sg => {
      addDeviceAtBus('static_generator', sg.bus, {
        ...COMP.defaults.static_generator,
        name: 'SGen',
        p_mw: sg.p_mw, q_mvar: sg.q_mvar,
        sgen_type: sg.sgen_type || 'PV',
        p_rated_mw: sg.p_rated_mw, sn_mva: sg.sn_mva,
        pmax_mw: sg.pmax_mw, pmin_mw: sg.pmin_mw,
        qmax_mvar: sg.qmax_mvar, qmin_mvar: sg.qmin_mvar,
        scaling: sg.scaling,
        controllable: sg.controllable || false,
        v_ref_pu: sg.v_ref_pu,
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

    // DCDC converters
    jsonSys.dcdc_converters?.forEach(dc => {
      const inCompId = dcBusCompMap[dc.bus_in];
      const outCompId = dcBusCompMap[dc.bus_out];
      const inComp = inCompId !== undefined ? getComponent(inCompId) : null;
      const outComp = outCompId !== undefined ? getComponent(outCompId) : null;
      const x = inComp ? (outComp ? (inComp.x + outComp.x) / 2 : inComp.x + 100) : 400;
      const y = inComp ? inComp.y : (outComp ? outComp.y : 400);
      const comp = addComponent('dcdc_converter', x, y, {
        ...COMP.defaults.dcdc_converter,
        name: `DCDC ${dc.index !== undefined ? dc.index : ''}`,
        bus_in: dc.bus_in, bus_out: dc.bus_out,
        control_mode: dc.control_mode || 'Voltage',
        p_ref_mw: dc.p_ref_mw, v_ref_pu: dc.v_ref_pu,
        sn_mva: dc.sn_mva,
        vn_in_kv: dc.vn_in_kv, vn_out_kv: dc.vn_out_kv,
        eta: dc.eta,
        r_eq_pu: dc.r_eq_pu,
        pmax_mw: dc.pmax_mw, pmin_mw: dc.pmin_mw,
        k_droop: dc.k_droop,
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
          portParams['port' + pi + '_bus'] = pt.bus || 0;
          portParams['port' + pi + '_side'] = pt.side || 0;
          portParams['port' + pi + '_control_mode'] = pt.control_mode || 'PQ';
          portParams['port' + pi + '_p_set_mw'] = pt.p_set_mw || 0;
          portParams['port' + pi + '_q_set_mvar'] = pt.q_set_mvar || 0;
          portParams['port' + pi + '_v_set_pu'] = pt.v_set_pu || 1.0;
          portParams['port' + pi + '_eta'] = pt.eta || 0.98;
        });
      }
      const comp = addComponent('energy_router', 500, 400, {
        ...COMP.defaults.energy_router,
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
          const busCompId = busCompMap[busIdx];
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
        name: 'Mobile ESS',
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
    // Remove existing visualization elements (keep voltage text)
    resultsLayer.querySelectorAll('.viz-overlay').forEach(el => el.remove());
    // Remove previous radial gradient defs
    const svgEl = resultsLayer.ownerSVGElement || document.querySelector('#canvas');
    let defsEl = svgEl.querySelector('defs#vizGradDefs');
    if (defsEl) defsEl.remove();
    // Reset connection line stroke if we changed them
    state.connections.forEach(conn => {
      if (conn.el) {
        const line = conn.el.querySelector('line');
        if (line) {
          line.setAttribute('stroke', '#666');
          line.setAttribute('stroke-width', '2');
          line.classList.remove('flow-arrow-line');
          line.style.animation = '';
          line.removeAttribute('marker-mid');
          line.removeAttribute('marker-end');
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
          pf_mw: br.pf_mw || 0,
          pt_mw: br.pt_mw || 0,
          loading_pct: br.loading_pct || 0,
          rate_mva: br.rate_mva || 0,
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

    // Build branchKey → branchData index lookup
    const branchByKey = {};
    branchData.forEach((bd, i) => {
      if (bd.from && bd.to) {
        branchByKey[`${bd.from}-${bd.to}`] = i;
        branchByKey[`${bd.to}-${bd.from}`] = i; // bidirectional lookup
      }
    });

    // Compute power range for relative color normalization (include DC branches)
    const dcBranchDataAll = _lastPfResult.geo_dc_branches || [];
    const powers = branchData.map(bd => Math.abs(bd.pf_mw))
      .concat(dcBranchDataAll.map(bd => Math.abs(bd.pf_mw || 0)))
      .filter(v => v > 0.01);
    const maxPower = powers.length ? Math.max(...powers) : 1;
    const minPower = powers.length ? Math.min(...powers) : 0;
    const powerRange = maxPower - minPower || 1;
    // Check loading availability: per-branch coloring will use loading% when rate_mva > 0,
    // and power normalization otherwise. Legend reflects the dominant mode.
    const branchesWithRate = branchData.filter(bd => bd.rate_mva > 0).length +
      dcBranchDataAll.filter(bd => (bd.rate_mva || 0) > 0).length;
    const totalBranches = branchData.length + dcBranchDataAll.length;
    const allHaveLoading = totalBranches > 0 && branchesWithRate === totalBranches;

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
        colorPct = 100 * (absPower - minPower) / powerRange;
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
      if (showFlow && absPower > 0.01) {
        // Draw animated arrow along each connection segment
        connPairs.forEach(({ conn, busCompId }) => {
          if (!conn.el) return;
          const line = conn.el.querySelector('line');
          if (!line) return;

          let x1 = parseFloat(line.getAttribute('x1'));
          let y1 = parseFloat(line.getAttribute('y1'));
          let x2 = parseFloat(line.getAttribute('x2'));
          let y2 = parseFloat(line.getAttribute('y2'));

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

          // (x1,y1) is the branch component end, (x2,y2) is the bus end
          // for the connection. But we need to check the actual conn direction.
          // conn.from is one end, conn.to is the other
          if (!flowsTowardBus) {
            [x1, y1, x2, y2] = [x2, y2, x1, y1]; // reverse arrow
          }

          const mx = (x1 + x2) / 2;
          const my = (y1 + y2) / 2;
          const angle = Math.atan2(y2 - y1, x2 - x1) * 180 / Math.PI;

          // Arrow triangle
          const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
          arrow.classList.add('viz-overlay', 'flow-arrow-dot');
          const sz = Math.max(5, Math.min(10, 5 + segPower / 100));
          arrow.setAttribute('points', `${-sz},-${sz/2} ${sz},0 ${-sz},${sz/2}`);
          arrow.setAttribute('transform', `translate(${mx},${my}) rotate(${angle})`);
          arrow.setAttribute('fill', showHeat ? loadingColor(colorPct) : '#1976D2');
          arrow.setAttribute('opacity', '0.85');
          resultsLayer.appendChild(arrow);

          // Power label next to the arrow (perpendicular offset)
          const dx = x2 - x1, dy = y2 - y1;
          const nl = Math.hypot(dx, dy) || 1;
          const offX = -dy / nl * 14, offY = dx / nl * 14;
          const segLabel = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          segLabel.classList.add('viz-overlay', 'flow-label');
          segLabel.setAttribute('x', mx + offX);
          segLabel.setAttribute('y', my + offY);
          segLabel.setAttribute('fill', showHeat ? loadingColor(colorPct) : '#1976D2');
          segLabel.textContent = `${pFmt(segPower)} ${pUnit()}`;
          resultsLayer.appendChild(segLabel);
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
            colorPct = 100 * (smax - minPower) / powerRange;
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
            const absPower = Math.abs(p);
            if (absPower < 0.01) return;

            const line = bc.conn.el.querySelector('line');
            if (!line) return;

            let x1 = parseFloat(line.getAttribute('x1'));
            let y1 = parseFloat(line.getAttribute('y1'));
            let x2 = parseFloat(line.getAttribute('x2'));
            let y2 = parseFloat(line.getAttribute('y2'));

            // Determine which end is the component vs the bus
            const fromIsComp = bc.conn.from.compId === comp.id;
            const cx = fromIsComp ? x1 : x2, cy = fromIsComp ? y1 : y2;
            const bx = fromIsComp ? x2 : x1, by = fromIsComp ? y2 : y1;

            // p > 0 means power entering transformer from this bus → arrow from bus to comp
            // p < 0 means power leaving transformer to this bus → arrow from comp to bus
            let ax1, ay1, ax2, ay2;
            if (p > 0) { ax1 = bx; ay1 = by; ax2 = cx; ay2 = cy; }
            else { ax1 = cx; ay1 = cy; ax2 = bx; ay2 = by; }

            const mx = (ax1 + ax2) / 2;
            const my = (ay1 + ay2) / 2;
            const angle = Math.atan2(ay2 - ay1, ax2 - ax1) * 180 / Math.PI;

            let colorPct;
            if (tf.rate_mva > 0) colorPct = tf.loading_pct;
            else colorPct = 100 * (absPower - minPower) / powerRange;

            const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            arrow.classList.add('viz-overlay', 'flow-arrow-dot');
            const sz = Math.max(5, Math.min(10, 5 + absPower / 100));
            arrow.setAttribute('points', `${-sz},-${sz/2} ${sz},0 ${-sz},${sz/2}`);
            arrow.setAttribute('transform', `translate(${mx},${my}) rotate(${angle})`);
            arrow.setAttribute('fill', showHeat ? loadingColor(colorPct) : '#1976D2');
            arrow.setAttribute('opacity', '0.85');
            resultsLayer.appendChild(arrow);

            // Winding power label
            const dx = ax2 - ax1, dy = ay2 - ay1;
            const nl = Math.hypot(dx, dy) || 1;
            const offX = -dy / nl * 14, offY = dx / nl * 14;
            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.classList.add('viz-overlay', 'flow-label');
            label.setAttribute('x', mx + offX);
            label.setAttribute('y', my + offY);
            label.textContent = `${pFmt(absPower)} ${pUnit()}`;
            resultsLayer.appendChild(label);
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

      const supplyTypes = new Set(['generator', 'pv_system', 'renewable_gen', 'static_generator', 'external_grid', 'dc_pv_array', 'vpp']);
      const demandTypes = new Set(['load', 'dc_load', 'flexible_load', 'asymmetric_load', 'motor', 'charging_station', 'charger']);
      const bidirTypes = new Set(['storage', 'mobile_storage']);
      const skipTypes = new Set(['ac_bus', 'dc_bus', 'ac_branch', 'dc_branch', 'transformer_2w', 'transformer_3w', 'switch_comp', 'circuit_breaker', 'shunt', 'microgrid', 'vsc_converter', 'dcdc_converter', 'energy_router']);

      // Compute bus net injection from branch flows for external_grid power estimation
      const busInject = {};
      ((_lastPfResult.geo_ac_branches || []).concat(_lastPfResult.geo_dc_branches || [])).forEach(br => {
        if (br.pf_mw !== undefined) {
          busInject[br.from] = (busInject[br.from] || 0) - br.pf_mw;
          busInject[br.to] = (busInject[br.to] || 0) - (br.pt_mw || 0);
        }
      });

      // VSC transfer power at AC-side buses
      (_lastPfResult.geo_vsc || []).forEach(vsc => {
        if (vsc.p_ac_mw !== undefined) {
          busInject[vsc.bus_ac] = (busInject[vsc.bus_ac] || 0) - vsc.p_ac_mw;
        }
      });

      // 3W transformer winding flows contribute to bus injection
      (_lastPfResult.geo_trafo3w || []).forEach(tf => {
        busInject[tf.hv_bus] = (busInject[tf.hv_bus] || 0) - tf.p_hv_mw;
        busInject[tf.mv_bus] = (busInject[tf.mv_bus] || 0) - tf.p_mv_mw;
        busInject[tf.lv_bus] = (busInject[tf.lv_bus] || 0) - tf.p_lv_mw;
      });

      state.components.forEach(comp => {
        if (skipTypes.has(comp.type)) return;
        if (!supplyTypes.has(comp.type) && !demandTypes.has(comp.type) && !bidirTypes.has(comp.type)) return;

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

        const p = comp.params;
        let powerMW = 0;
        if (comp.type === 'generator') {
          // Use actual PF result from geo_gen if available
          const genData = (_lastPfResult.geo_gen || []);
          const busIdx = compToBus[busCompId];
          const gd = genData.find(g => g.bus === busIdx);
          powerMW = gd ? gd.pg_mw : (p.pg_mw || 0);
        }
        else if (comp.type === 'asymmetric_load') powerMW = (p.pa_mw||0) + (p.pb_mw||0) + (p.pc_mw||0);
        else if (comp.type === 'external_grid') {
          // Estimate slack power from bus net injection
          const busIdx = compToBus[busCompId];
          if (busIdx !== undefined && busInject[busIdx] !== undefined) {
            powerMW = busInject[busIdx]; // net injection at bus (gen positive)
            // Add back loads at this bus
            const loadAtBus = (_lastPfResult.geo_buses || []).find(gb => gb.id === busIdx);
            if (loadAtBus) powerMW += (loadAtBus.pd_mw || 0);
          }
        }
        else powerMW = p.p_mw || p.p_set_mw || p.p_rated_mw || 0;

        if (Math.abs(powerMW) < 0.001) return;

        // Determine direction
        let isSupply;
        if (supplyTypes.has(comp.type)) isSupply = true;
        else if (demandTypes.has(comp.type)) isSupply = false;
        else isSupply = powerMW > 0; // storage: positive = discharge

        const line = conn.el.querySelector('line');
        if (!line) return;

        let x1 = parseFloat(line.getAttribute('x1'));
        let y1 = parseFloat(line.getAttribute('y1'));
        let x2 = parseFloat(line.getAttribute('x2'));
        let y2 = parseFloat(line.getAttribute('y2'));

        // Determine which end is the component and which is the bus
        const fromIsComp = conn.from.compId === comp.id;
        const cx = fromIsComp ? x1 : x2, cy = fromIsComp ? y1 : y2;
        const bx = fromIsComp ? x2 : x1, by = fromIsComp ? y2 : y1;

        // Arrow direction: supply → component→bus, demand → bus→component
        let ax1, ay1, ax2, ay2;
        if (isSupply) { ax1 = cx; ay1 = cy; ax2 = bx; ay2 = by; }
        else { ax1 = bx; ay1 = by; ax2 = cx; ay2 = cy; }

        const mx = (ax1 + ax2) / 2;
        const my = (ay1 + ay2) / 2;
        const angle = Math.atan2(ay2 - ay1, ax2 - ax1) * 180 / Math.PI;

        // Color by power magnitude (same scale as branch arrows)
        const absPower = Math.abs(powerMW);
        const colorPct = 100 * (absPower - minPower) / powerRange;
        const arrowColor = showHeat ? loadingColor(colorPct) : loadingColor(colorPct);

        // Arrow triangle
        const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
        arrow.classList.add('viz-overlay', 'flow-arrow-dot');
        const sz = Math.max(5, Math.min(10, 5 + absPower / 100));
        arrow.setAttribute('points', `${-sz},-${sz/2} ${sz},0 ${-sz},${sz/2}`);
        arrow.setAttribute('transform', `translate(${mx},${my}) rotate(${angle})`);
        arrow.setAttribute('fill', arrowColor);
        arrow.setAttribute('opacity', '0.85');
        resultsLayer.appendChild(arrow);

        // Power label at offset from midpoint along the normal
        const dx = ax2 - ax1, dy = ay2 - ay1;
        const nl = Math.hypot(dx, dy) || 1;
        const offX = -dy / nl * 14, offY = dx / nl * 14;
        const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        label.classList.add('viz-overlay', 'flow-label');
        label.setAttribute('x', mx + offX);
        label.setAttribute('y', my + offY);
        label.setAttribute('fill', arrowColor);
        label.textContent = `${pFmt(absPower)} ${pUnit()}`;
        resultsLayer.appendChild(label);
      });
    }

    // ── DC branch flow arrows ──
    if (showFlow || showHeat) {
      const dcBranchData = _lastPfResult.geo_dc_branches || [];
      let dcBrIdx = 0;
      state.components.forEach(comp => {
        if (comp.type !== 'dc_branch') return;
        const bd = dcBranchData[dcBrIdx++];
        if (!bd) return;
        const pf_mw_dc = bd.pf_mw || 0;
        const pt_mw_dc = bd.pt_mw || 0;
        const absPower = Math.max(Math.abs(pf_mw_dc), Math.abs(pt_mw_dc));
        if (absPower < 0.01) return;

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
        const colorPct = (bd.rate_mva > 0 && bd.loading_pct > 0)
          ? bd.loading_pct
          : 100 * (absPower - minPower) / powerRange;

        // Heatmap glow for DC branch
        if (showHeat) {
          heatItems.push({ comp, colorPct, absPower, maxPower, bd: { rate_mva: bd.rate_mva || 0 }, hasLoading: (bd.rate_mva || 0) > 0, loading: bd.loading_pct || 0 });
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
            const line = conn.el.querySelector('line');
            if (!line) return;
            let x1 = parseFloat(line.getAttribute('x1'));
            let y1 = parseFloat(line.getAttribute('y1'));
            let x2 = parseFloat(line.getAttribute('x2'));
            let y2 = parseFloat(line.getAttribute('y2'));
            let flowsTowardBus;
            if (pf_mw_dc >= 0) flowsTowardBus = (busCompId === toBusCompId) === isForward;
            else flowsTowardBus = (busCompId === fromBusCompId) === isForward;

            // Determine which power value (Pf or Pt) to show on this side
            const isFromSide = isForward ? (busCompId === fromBusCompId) : (busCompId === toBusCompId);
            const segPower = isFromSide ? Math.abs(pf_mw_dc) : Math.abs(pt_mw_dc);

            if (!flowsTowardBus) [x1, y1, x2, y2] = [x2, y2, x1, y1];
            const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
            const angle = Math.atan2(y2 - y1, x2 - x1) * 180 / Math.PI;
            const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            arrow.classList.add('viz-overlay', 'flow-arrow-dot');
            const sz = Math.max(5, Math.min(10, 5 + segPower / 100));
            arrow.setAttribute('points', `${-sz},-${sz/2} ${sz},0 ${-sz},${sz/2}`);
            arrow.setAttribute('transform', `translate(${mx},${my}) rotate(${angle})`);
            arrow.setAttribute('fill', loadingColor(colorPct));
            arrow.setAttribute('opacity', '0.85');
            resultsLayer.appendChild(arrow);

            // Power label next to the arrow (perpendicular offset)
            const dx = x2 - x1, dy = y2 - y1;
            const nl = Math.hypot(dx, dy) || 1;
            const offX = -dy / nl * 14, offY = dx / nl * 14;
            const segLabel = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            segLabel.classList.add('viz-overlay', 'flow-label');
            segLabel.setAttribute('x', mx + offX);
            segLabel.setAttribute('y', my + offY);
            segLabel.setAttribute('fill', loadingColor(colorPct));
            segLabel.textContent = `${pFmt(segPower)} ${pUnit()}`;
            resultsLayer.appendChild(segLabel);
          });
        }
      });
    }

    // ── VSC converter flow arrows ──
    if (showFlow || showHeat) {
      const vscData = _lastPfResult.geo_vsc || [];
      let vscIdx = 0;
      state.components.forEach(comp => {
        if (comp.type !== 'vsc_converter') return;
        const vd = vscData[vscIdx++];
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
          const absPower = Math.abs(p);
          if (absPower < 0.01) return;
          // Match by both bus index AND bus type to avoid AC/DC index collision
          const bc = busConns.find(bc => bc.busIdx === matchBus && bc.busType === expectedBusType);
          if (!bc || !bc.conn || !bc.conn.el) return;

          const line = bc.conn.el.querySelector('line');
          if (!line) return;
          let x1 = parseFloat(line.getAttribute('x1'));
          let y1 = parseFloat(line.getAttribute('y1'));
          let x2 = parseFloat(line.getAttribute('x2'));
          let y2 = parseFloat(line.getAttribute('y2'));

          const fromIsComp = bc.conn.from.compId === comp.id;
          const cx = fromIsComp ? x1 : x2, cy = fromIsComp ? y1 : y2;
          const bx = fromIsComp ? x2 : x1, by = fromIsComp ? y2 : y1;

          // Bus-injection-positive: p > 0 means inject into bus → arrow comp→bus
          //                         p < 0 means draw from bus   → arrow bus→comp
          let ax1, ay1, ax2, ay2;
          if (p > 0) { ax1 = cx; ay1 = cy; ax2 = bx; ay2 = by; }
          else { ax1 = bx; ay1 = by; ax2 = cx; ay2 = cy; }

          const mx = (ax1 + ax2) / 2, my = (ay1 + ay2) / 2;
          const angle = Math.atan2(ay2 - ay1, ax2 - ax1) * 180 / Math.PI;
          const colorPct = 100 * (absPower - minPower) / powerRange;

          if (showFlow) {
            const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            arrow.classList.add('viz-overlay', 'flow-arrow-dot');
            const sz = Math.max(5, Math.min(10, 5 + absPower / 100));
            arrow.setAttribute('points', `${-sz},-${sz/2} ${sz},0 ${-sz},${sz/2}`);
            arrow.setAttribute('transform', `translate(${mx},${my}) rotate(${angle})`);
            arrow.setAttribute('fill', loadingColor(colorPct));
            arrow.setAttribute('opacity', '0.85');
            resultsLayer.appendChild(arrow);

            const dx = ax2 - ax1, dy = ay2 - ay1;
            const nl = Math.hypot(dx, dy) || 1;
            const offX = -dy / nl * 14, offY = dx / nl * 14;
            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.classList.add('viz-overlay', 'flow-label');
            label.setAttribute('x', mx + offX);
            label.setAttribute('y', my + offY);
            label.textContent = `${pFmt(absPower)} ${pUnit()}`;
            resultsLayer.appendChild(label);
          }
        });

        // Heatmap glow for VSC
        if (showHeat) {
          const sMax = Math.max(Math.abs(vd.p_ac_mw), Math.abs(vd.p_dc_mw));
          const colorPct = 100 * (sMax - minPower) / powerRange;
          heatItems.push({ comp, colorPct, absPower: sMax, maxPower, bd: { rate_mva: 0 }, hasLoading: false, loading: 0 });
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
      const dcdcData = _lastPfResult.geo_dcdc || [];
      let dcdcIdx = 0;
      state.components.forEach(comp => {
        if (comp.type !== 'dcdc_converter') return;
        const dd = dcdcData[dcdcIdx++];
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
          const absPower = Math.abs(p);
          if (absPower < 0.01) return;
          const bc = busConns.find(bc => bc.busIdx === matchBus);
          if (!bc || !bc.conn || !bc.conn.el) return;

          const line = bc.conn.el.querySelector('line');
          if (!line) return;
          let x1 = parseFloat(line.getAttribute('x1'));
          let y1 = parseFloat(line.getAttribute('y1'));
          let x2 = parseFloat(line.getAttribute('x2'));
          let y2 = parseFloat(line.getAttribute('y2'));

          const fromIsComp = bc.conn.from.compId === comp.id;
          const cx = fromIsComp ? x1 : x2, cy = fromIsComp ? y1 : y2;
          const bx = fromIsComp ? x2 : x1, by = fromIsComp ? y2 : y1;

          // p_in > 0: power drawn from bus_in → arrow bus→comp
          // p_out > 0: power delivered to bus_out → arrow comp→bus
          let ax1, ay1, ax2, ay2;
          if (label === 'in') {
            if (p > 0) { ax1 = bx; ay1 = by; ax2 = cx; ay2 = cy; }
            else { ax1 = cx; ay1 = cy; ax2 = bx; ay2 = by; }
          } else {
            if (p > 0) { ax1 = cx; ay1 = cy; ax2 = bx; ay2 = by; }
            else { ax1 = bx; ay1 = by; ax2 = cx; ay2 = cy; }
          }

          const mx = (ax1 + ax2) / 2, my = (ay1 + ay2) / 2;
          const angle = Math.atan2(ay2 - ay1, ax2 - ax1) * 180 / Math.PI;
          const colorPct = 100 * (absPower - minPower) / powerRange;

          if (showFlow) {
            const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            arrow.classList.add('viz-overlay', 'flow-arrow-dot');
            const sz = Math.max(5, Math.min(10, 5 + absPower / 100));
            arrow.setAttribute('points', `${-sz},-${sz/2} ${sz},0 ${-sz},${sz/2}`);
            arrow.setAttribute('transform', `translate(${mx},${my}) rotate(${angle})`);
            arrow.setAttribute('fill', loadingColor(colorPct));
            arrow.setAttribute('opacity', '0.85');
            resultsLayer.appendChild(arrow);

            const dx = ax2 - ax1, dy = ay2 - ay1;
            const nl = Math.hypot(dx, dy) || 1;
            const offX = -dy / nl * 14, offY = dx / nl * 14;
            const label2 = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label2.classList.add('viz-overlay', 'flow-label');
            label2.setAttribute('x', mx + offX);
            label2.setAttribute('y', my + offY);
            label2.textContent = `${pFmt(absPower)} ${pUnit()}`;
            resultsLayer.appendChild(label2);
          }
        });

        // Heatmap glow for DCDC
        if (showHeat) {
          const sMax = Math.max(Math.abs(dd.p_in_mw), Math.abs(dd.p_out_mw));
          const colorPct = 100 * (sMax - minPower) / powerRange;
          heatItems.push({ comp, colorPct, absPower: sMax, maxPower, bd: { rate_mva: 0 }, hasLoading: false, loading: 0 });
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
        // Determine ER index: match by sequential order among ER components
        const erIdx = erCompIdx++;
        const erd = erData[erIdx];
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
          const absPower = Math.abs(pt.p_mw);
          if (absPower < 0.01) return;
          const expectedBusType = pt.is_ac ? 'ac_bus' : 'dc_bus';
          const bc = busConns.find(bc => bc.busIdx === pt.bus && bc.busType === expectedBusType);
          if (!bc || !bc.conn || !bc.conn.el) return;

          const line = bc.conn.el.querySelector('line');
          if (!line) return;
          let x1 = parseFloat(line.getAttribute('x1'));
          let y1 = parseFloat(line.getAttribute('y1'));
          let x2 = parseFloat(line.getAttribute('x2'));
          let y2 = parseFloat(line.getAttribute('y2'));

          const fromIsComp = bc.conn.from.compId === comp.id;
          const cx = fromIsComp ? x1 : x2, cy = fromIsComp ? y1 : y2;
          const bx = fromIsComp ? x2 : x1, by = fromIsComp ? y2 : y1;

          // Bus-injection positive: p > 0 → inject into bus → arrow comp→bus
          let ax1, ay1, ax2, ay2;
          if (pt.p_mw > 0) { ax1 = cx; ay1 = cy; ax2 = bx; ay2 = by; }
          else { ax1 = bx; ay1 = by; ax2 = cx; ay2 = cy; }

          const mx = (ax1 + ax2) / 2, my = (ay1 + ay2) / 2;
          const angle = Math.atan2(ay2 - ay1, ax2 - ax1) * 180 / Math.PI;
          const colorPct = 100 * (absPower - minPower) / powerRange;

          if (showFlow) {
            const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            arrow.classList.add('viz-overlay', 'flow-arrow-dot');
            const sz = Math.max(5, Math.min(10, 5 + absPower / 100));
            arrow.setAttribute('points', `${-sz},-${sz/2} ${sz},0 ${-sz},${sz/2}`);
            arrow.setAttribute('transform', `translate(${mx},${my}) rotate(${angle})`);
            arrow.setAttribute('fill', loadingColor(colorPct));
            arrow.setAttribute('opacity', '0.85');
            resultsLayer.appendChild(arrow);

            const dx = ax2 - ax1, dy = ay2 - ay1;
            const nl = Math.hypot(dx, dy) || 1;
            const offX = -dy / nl * 14, offY = dx / nl * 14;
            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.classList.add('viz-overlay', 'flow-label');
            label.setAttribute('x', mx + offX);
            label.setAttribute('y', my + offY);
            label.setAttribute('fill', loadingColor(colorPct));
            label.textContent = `${pFmt(absPower)} ${pUnit()}`;
            resultsLayer.appendChild(label);
          }
        });

        // Heatmap glow for Energy Router
        if (showHeat) {
          const sMax = Math.max(...erd.ports.map(pt => Math.abs(pt.p_mw)), 0);
          const colorPct = 100 * (sMax - minPower) / powerRange;
          heatItems.push({ comp, colorPct, absPower: sMax, maxPower, bd: { rate_mva: 0 }, hasLoading: false, loading: 0 });
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

  function clearResults() {
    resultsLayer.innerHTML = '';
    _lastPfResult = null;
    // Remove heatmap gradient defs
    const svgEl = resultsLayer.ownerSVGElement || document.querySelector('#canvas');
    const defsEl = svgEl.querySelector('defs#vizGradDefs');
    if (defsEl) defsEl.remove();
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
      extGrid: {}, storage: {}, pv: {}, renGen: {}, sgen: {}, sw: {}, cb: {},
      motor: {}, dcLoad: {}, dcBranch: {}, vsc: {}, shunt: {}, trafo3w: {},
      flexLoad: {}, asymLoad: {}, charger: {}, chargingStation: {},
      mobileStorage: {}, dcdcConverter: {}, energyRouter: {}, vpp: {}, microgrid: {} };

    let acBusIdx = 1, dcBusIdx = 1;
    state.components.forEach(comp => {
      if (comp.type === 'ac_bus') maps.ac[acBusIdx++] = comp.id;
      else if (comp.type === 'dc_bus') maps.dc[dcBusIdx++] = comp.id;
    });

    // Must match buildSystemJson iteration order for index consistency
    const idx = { br: 0, gen: 0, load: 0, trafo: 0, eg: 0, stor: 0, pv: 0,
      ren: 0, sgen: 0, sw: 0, cb: 0, motor: 0, dcLoad: 0, dcBr: 0, vsc: 0,
      shunt: 0, trafo3w: 0, flex: 0, asym: 0, charger: 0, cs: 0, ms: 0,
      dcdc: 0, er: 0, vpp: 0, mg: 0 };
    state.components.forEach(comp => {
      const p = comp.params;
      switch (comp.type) {
        case 'ac_branch': maps.branch[idx.br++] = comp.id; break;
        case 'generator': maps.gen[idx.gen++] = comp.id; break;
        case 'load': maps.load[idx.load++] = comp.id; break;
        case 'transformer_2w':
          if (p._from_branch) maps.branch[idx.br++] = comp.id;
          else maps.trafo[idx.trafo++] = comp.id;
          break;
        case 'external_grid': maps.extGrid[idx.eg++] = comp.id; break;
        case 'storage': maps.storage[idx.stor++] = comp.id; break;
        case 'pv_system': maps.pv[idx.pv++] = comp.id; break;
        case 'renewable_gen': maps.renGen[idx.ren++] = comp.id; break;
        case 'static_generator': maps.sgen[idx.sgen++] = comp.id; break;
        case 'switch_comp': maps.sw[idx.sw++] = comp.id; break;
        case 'circuit_breaker': maps.cb[idx.cb++] = comp.id; break;
        case 'motor': maps.motor[idx.motor++] = comp.id; break;
        case 'dc_load': maps.dcLoad[idx.dcLoad++] = comp.id; break;
        case 'dc_branch': maps.dcBranch[idx.dcBr++] = comp.id; break;
        case 'vsc_converter': maps.vsc[idx.vsc++] = comp.id; break;
        case 'shunt': maps.shunt[idx.shunt++] = comp.id; break;
        case 'transformer_3w': maps.trafo3w[idx.trafo3w++] = comp.id; break;
        case 'flexible_load': maps.flexLoad[idx.flex++] = comp.id; break;
        case 'asymmetric_load': maps.asymLoad[idx.asym++] = comp.id; break;
        case 'charger': maps.charger[idx.charger++] = comp.id; break;
        case 'charging_station': maps.chargingStation[idx.cs++] = comp.id; break;
        case 'mobile_storage': maps.mobileStorage[idx.ms++] = comp.id; break;
        case 'dcdc_converter': maps.dcdcConverter[idx.dcdc++] = comp.id; break;
        case 'energy_router': maps.energyRouter[idx.er++] = comp.id; break;
        case 'vpp': maps.vpp[idx.vpp++] = comp.id; break;
        case 'microgrid': maps.microgrid[idx.mg++] = comp.id; break;
      }
    });

    return maps;
  }

  // ========== Public API ==========
  return {
    init,
    addComponent,
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
    rotateSelected,
    buildSystemJson,
    loadFromSystemJson,
    showPowerFlowResults,
    clearResults,
    setVisualizationMode,
    refreshVisualization: applyVisualizationOverlay,
    clearAll,
    panToComponent,
    getCompBusMap,
    get state() { return state; },
  };
})();
