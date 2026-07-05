/**
 * app.js — Main application: API communication, UI wiring, topology tables, results display.
 */
'use strict';

const API_BASE = window.location.origin;  // Same origin as the C++ server

// Bus-terminal fields that are resolved from the canvas wiring (connected ports)
// rather than typed by hand. The property inspector renders these read-only so
// users wire ports (e.g. a DC/DC's 'in'/'out') instead of guessing internal IDs.
const WIRING_DERIVED_BUS_FIELDS = new Set([
  'bus_in', 'bus_out', 'bus_ac', 'bus_dc', 'from_bus', 'to_bus', 'hv_bus', 'lv_bus',
  'port1_bus', 'port2_bus', 'port3_bus', 'port4_bus',
]);

const App = (() => {
  // ========== Canvas Dirty Tracking ==========
  // When a built-in case or JSON is loaded, the backend already has the correct
  // system. We mark the canvas "clean". Any edits on the canvas set it "dirty",
  // and only dirty canvases need a roundtrip sync before analysis.
  let _canvasDirty = false;

  // ========== Console Logging ==========
  function log(msg, level = 'info') {
    const el = document.getElementById('consoleLog');
    if (!el) return;
    const time = new Date().toLocaleTimeString('zh-CN', { hour12: false });
    const div = document.createElement('div');
    div.className = `log-${level}`;
    div.innerHTML = `<span class="log-time">[${time}]</span>${escapeHtml(msg)}`;
    el.appendChild(div);
    el.scrollTop = el.scrollHeight;
  }

  function escapeHtml(str) {
    const d = document.createElement('div');
    d.textContent = str;
    return d.innerHTML;
  }

  // ========== Display Unit Helpers ==========
  // Reads the pfDisplayUnit selector (MW / kW / W) and converts power values.
  // All backend values are in MW/MVar; these helpers scale for display.
  function getPowerUnit() {
    const el = document.getElementById('pfDisplayUnit');
    return el ? el.value : 'MW';
  }
  function pScale() { const u = getPowerUnit(); return u === 'kW' ? 1e3 : u === 'W' ? 1e6 : 1; }
  function pUnit()  { const u = getPowerUnit(); return u === 'kW' ? 'kW' : u === 'W' ? 'W' : 'MW'; }
  function qUnit()  { const u = getPowerUnit(); return u === 'kW' ? 'kVar' : u === 'W' ? 'Var' : 'MVar'; }
  function pConv(mw) { return mw * pScale(); }
  function pFmt(mw, d = 2) { return (mw * pScale()).toFixed(d); }

  // Cache for re-rendering on unit change without re-running solver
  let _lastPfData = null;
  let _lastOpfData = null;
  let _lastTspfData = null;
  let _lastIntegratedEnergyData = null;
  let _lastAnnualData = null;
  let _lastCarbonData = null;
  let _lastDynamicCarbonData = null;
  let _lastTransientData = null;
  let _lastReliabilityData = null;
  let _lastResilienceData = null;
  let _lastScenarioGenerationData = null;
  let _lastTopoAnalysisData = null;
  let _lastNetReductionData = null;
  let _lastScenarioBaseSystemJson = null;
  let _resilienceComparisonRuns = [];
  let _scenarioCurveEntries = [];
  let _importedGeneratedScenario = null;
  let _lastImportedGeneratedScenarioKey = '';
  let _generatedScenarioTimeSeriesActive = false;
  let _transientEvents = [];
  let _analysisQueue = Promise.resolve();
  let _activeLoadPromise = null;

  const ANALYSIS_BUSY_ERROR = 'Another analysis is already running';

  function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  async function waitForBackendIdle(timeoutMs = 120000) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
      const status = await apiGet('/api/session/status', { quiet: true });
      if (!status || status.busy !== true) return true;
      await sleep(250);
    }
    log('后端仍有分析任务在运行，请稍后重试', 'warn');
    return false;
  }

  async function withAnalysisQueue(work) {
    const previous = _analysisQueue.catch(() => {});
    let release;
    _analysisQueue = new Promise(resolve => { release = resolve; });
    await previous;
    try {
      return await work();
    } finally {
      release();
    }
  }

  function trackActiveLoad(work) {
    const loadPromise = Promise.resolve().then(work);
    _activeLoadPromise = loadPromise;
    return loadPromise.finally(() => {
      if (_activeLoadPromise === loadPromise) _activeLoadPromise = null;
    });
  }

  function invalidateAnalysisResults(reason = '') {
    _lastPfData = null;
    _lastOpfData = null;
    _lastTspfData = null;
    _lastIntegratedEnergyData = null;
    _lastCarbonData = null;
    _lastDynamicCarbonData = null;
    _lastTransientData = null;
    if (typeof Canvas !== 'undefined' && Canvas.clearResults) Canvas.clearResults();
    if (reason) log(reason, 'info');
  }

  // ========== Per-module result group switching ==========
  // Each calc display function calls setActiveResultGroup(name). CSS in
  // style.css uses #resultsContent[data-active-group=...] to show only the
  // matching .result-group block, so different functional modules don't
  // visually mix their results.
  function setActiveResultGroup(name) {
    const rc = document.getElementById('resultsContent');
    const active = name || '';
    if (rc) rc.dataset.activeGroup = active;
    document.body?.setAttribute('data-active-result-group', active);
    const panel = document.getElementById('rightPanel');
    if (panel) {
      if (active === 'transient' || active === 'modelIO') {
        const target = window.innerWidth <= 900 ? window.innerWidth : Math.round(Math.min(window.innerWidth * 0.66, 1180));
        const minUseful = window.innerWidth <= 900 ? window.innerWidth : Math.min(target, Math.max(560, window.innerWidth - 420));
        if (!panel.style.width || panel.offsetWidth < minUseful) {
          panel.style.width = `${minUseful}px`;
          panel.dataset.autoTransientWidth = '1';
        }
      } else if (panel.dataset.autoTransientWidth === '1') {
        panel.style.width = '';
        delete panel.dataset.autoTransientWidth;
      }
    }
  }

  function showModelIoStatus(title, rows = [], options = {}) {
    const el = document.getElementById('modelIoResults');
    if (!el) return;
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('modelIO');
    const subtitle = options.subtitle || '统一 IO 操作记录';
    const rowHtml = rows.length
      ? rows.map(([label, value]) => `<tr><td>${escapeHtml(label)}</td><td>${escapeHtml(String(value ?? ''))}</td></tr>`).join('')
      : '<tr><td colspan="2">暂无明细。</td></tr>';
    const warnings = Array.isArray(options.warnings) && options.warnings.length
      ? `<div class="transient-section-head"><h5>导入告警</h5><span>${options.warnings.length} 条</span></div>
         <div class="transient-mini-grid">${options.warnings.map(w => `<div class="transient-mini-card"><strong>Warning</strong><span>${escapeHtml(w)}</span></div>`).join('')}</div>`
      : '';
    el.innerHTML = `
      <div class="transient-section-head"><h5>${escapeHtml(title)}</h5><span>${escapeHtml(subtitle)}</span></div>
      <div class="transient-table-scroll">
        <table><tbody>${rowHtml}</tbody></table>
      </div>
      ${warnings}`;
    switchTab('results');
  }

  // Build an inline attribute that makes a result row pan the canvas to a bus
  // (by model bus id). Returns '' when the bus has no canvas component, so only
  // mappable rows become clickable — mirroring the power-flow tables. Branch /
  // converter rows pass one endpoint bus, since the canvas does not retain model
  // branch ids (their result indices are non-positional model ids).
  function busClickAttr(busId, busMap) {
    const id = parseInt(busId, 10);
    if (!Number.isInteger(id)) return '';
    const m = busMap || (typeof Canvas !== 'undefined' && Canvas.getCompBusMap ? Canvas.getCompBusMap() : null);
    if (!m) return '';
    const compId = (m.ac && m.ac[id] != null) ? m.ac[id]
                 : (m.dc && m.dc[id] != null) ? m.dc[id] : undefined;
    return compId != null
      ? ` class="topo-clickable" data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
  }

  function compClickAttr(compId) {
    const id = parseInt(compId, 10);
    return Number.isInteger(id)
      ? ` class="topo-clickable" data-comp-id="${id}" onclick="Canvas.panToComponent(${id})"` : '';
  }

  function resultTypeKey(type) {
    return String(type || '')
      .replace(/([a-z0-9])([A-Z])/g, '$1_$2')
      .replace(/[-\s/]+/g, '_')
      .replace(/__+/g, '_')
      .toLowerCase();
  }

  function resultCanvasBucket(type, maps) {
    if (!type || !maps) return undefined;
    const raw = String(type);
    if (maps[raw]) return raw;
    const key = resultTypeKey(raw);
    const aliases = {
      ac: 'ac',
      ac_bus: 'ac',
      bus_ac: 'ac',
      dc: 'dc',
      dc_bus: 'dc',
      bus_dc: 'dc',
      branch: 'branch',
      ac_branch: 'branch',
      line: 'branch',
      ac_line: 'branch',
      transformer: 'trafo',
      transformer_2w: 'trafo',
      transformer2_w: 'trafo',
      transformer2w: 'trafo',
      trafo: 'trafo',
      transformer_3w: 'trafo3w',
      transformer3_w: 'trafo3w',
      transformer3w: 'trafo3w',
      trafo3w: 'trafo3w',
      generator: 'gen',
      synchronous_machine: 'gen',
      three_phase_generator: 'gen',
      gen: 'gen',
      ac_load: 'load',
      three_phase_load: 'load',
      load: 'load',
      external_grid: 'extGrid',
      three_phase_external_grid: 'extGrid',
      ext_grid: 'extGrid',
      extgrid: 'extGrid',
      storage: 'storage',
      ac_storage: 'storage',
      grid_forming_storage: 'storage',
      pv_system: 'pv',
      ac_pv_system: 'pv',
      pv: 'pv',
      renewable_gen: 'renGen',
      renewable_generator: 'renGen',
      ren_gen: 'renGen',
      static_generator: 'sgen',
      static_gen: 'sgen',
      sgen: 'sgen',
      dc_static_generator: 'dcSgen',
      dc_static_gen: 'dcSgen',
      dc_static_generator_ac: 'dcSgen',
      dc_sgen: 'dcSgen',
      switch: 'sw',
      switch_comp: 'sw',
      ac_switch: 'sw',
      sw: 'sw',
      circuit_breaker: 'cb',
      ac_circuit_breaker: 'cb',
      breaker: 'cb',
      cb: 'cb',
      dc_circuit_breaker: 'dcCb',
      dc_breaker: 'dcCb',
      dc_cb: 'dcCb',
      motor: 'motor',
      dc_branch: 'dcBranch',
      dc_line: 'dcBranch',
      dc_load: 'dcLoad',
      dc_storage: 'dcStorage',
      dc_pv_array: 'dcPv',
      dc_pv: 'dcPv',
      vsc: 'vsc',
      vsc_converter: 'vsc',
      vsc_grid_forming: 'vsc',
      vsc_grid_following: 'vsc',
      dcdc: 'dcdcConverter',
      dc_dc: 'dcdcConverter',
      dcdc_converter: 'dcdcConverter',
      dc_dc_converter: 'dcdcConverter',
      energy_router: 'energyRouter',
      er: 'energyRouter',
      shunt: 'shunt',
      flexible_load: 'flexLoad',
      flex_load: 'flexLoad',
      asymmetric_load: 'asymLoad',
      asym_load: 'asymLoad',
      charger: 'charger',
      charging_station: 'chargingStation',
      mobile_storage: 'mobileStorage',
      vpp: 'vpp',
      virtual_power_plant: 'vpp',
      microgrid: 'microgrid',
    };
    const bucket = aliases[key] || aliases[raw];
    return bucket && maps[bucket] ? bucket : undefined;
  }

  function validCanvasCompId(compId) {
    const id = Number(compId);
    if (!Number.isInteger(id)) return undefined;
    if (typeof Canvas !== 'undefined' && Canvas.getComponent && !Canvas.getComponent(id)) return undefined;
    return id;
  }

  function rowCanvasCompId(row, maps, options = {}) {
    if (!row || !maps) return validCanvasCompId(options.fallbackCompId);
    const directKeys = ['canvas_comp_id', 'comp_id', 'canvasComponentId', 'canvas_id'];
    for (const key of directKeys) {
      const hit = validCanvasCompId(row[key]);
      if (hit !== undefined) return hit;
    }

    const typeCandidates = [
      row.canvas_type,
      row.component_type,
      row.canonical_component_type,
      row.type,
      row.bucket,
      options.canvasType,
    ].filter(v => v !== undefined && v !== null && String(v) !== '');
    let bucket = undefined;
    for (const t of typeCandidates) {
      bucket = resultCanvasBucket(t, maps);
      if (bucket) break;
    }

    if (bucket) {
      const indexKeys = ['canvas_index', 'index', 'id', 'router_index', 'component_index'];
      for (const key of indexKeys) {
        const idx = Number(row[key]);
        if (Number.isFinite(idx) && maps[bucket] && maps[bucket][idx] != null) {
          return validCanvasCompId(maps[bucket][idx]);
        }
      }
      const positionKeys = ['position', 'component_position', 'canvas_position', 'row_position'];
      for (const key of positionKeys) {
        const pos = Number(row[key]);
        if (Number.isFinite(pos) && maps.byPosition && maps.byPosition[bucket] &&
            maps.byPosition[bucket][pos] != null) {
          return validCanvasCompId(maps.byPosition[bucket][pos]);
        }
      }
      for (const key of positionKeys) {
        const pos = Number(row[key]);
        if (Number.isFinite(pos) && maps[bucket] && maps[bucket][pos] != null) {
          return validCanvasCompId(maps[bucket][pos]);
        }
      }
    }

    const domainText = `${row.domain || row.component_domain || ''} ${typeCandidates.join(' ')} ${bucket || ''}`.toLowerCase();
    const preferDc = /\bdc\b/.test(domainText) || ['dc', 'dcBranch', 'dcLoad', 'dcStorage', 'dcPv', 'dcCb', 'dcSgen', 'dcdcConverter'].includes(bucket);
    const preferAc = /\bac\b/.test(domainText) || ['ac', 'branch', 'load', 'gen', 'extGrid', 'storage', 'pv', 'renGen', 'sgen', 'sw', 'cb', 'motor', 'shunt', 'trafo', 'trafo3w', 'flexLoad', 'asymLoad', 'charger', 'chargingStation', 'mobileStorage', 'vpp', 'microgrid'].includes(bucket);
    const busKeys = preferDc && !preferAc
      ? ['bus_dc', 'dc_bus', 'bus_in', 'bus_out', 'from_bus', 'to_bus', 'from', 'to', 'primary_bus', 'bus', 'index']
      : preferAc && !preferDc
      ? ['bus_ac', 'ac_bus', 'bus', 'from_bus', 'to_bus', 'from', 'to', 'hv_bus', 'mv_bus', 'lv_bus', 'primary_bus', 'index']
      : ['bus_ac', 'ac_bus', 'bus_dc', 'dc_bus', 'bus', 'from_bus', 'to_bus', 'from', 'to', 'bus_in', 'bus_out', 'hv_bus', 'mv_bus', 'lv_bus', 'primary_bus', 'index'];
    const tryBusKeys = (mapObj, keys) => {
      if (!mapObj) return undefined;
      for (const key of keys) {
        const bus = Number(row[key]);
        if (Number.isFinite(bus) && mapObj[bus] != null) return validCanvasCompId(mapObj[bus]);
      }
      return undefined;
    };
    const primary = preferDc && !preferAc ? maps.dc : maps.ac;
    const secondary = preferDc && !preferAc ? maps.ac : maps.dc;
    return tryBusKeys(primary, busKeys) ?? tryBusKeys(secondary, busKeys) ?? validCanvasCompId(options.fallbackCompId);
  }

  function canvasRowAttr(row, maps, options = {}) {
    const compId = rowCanvasCompId(row, maps, options);
    if (compId === undefined) return '';
    const cls = options.className ? ` class="${escapeHtml(options.className)}"` : '';
    return `${cls} data-comp-id="${compId}"`;
  }

  function positiveDemandMw(row) {
    return Math.abs(numberOr(row?.display_demand_mw ?? row?.demand_mw, 0));
  }

  function carbonLoadLabel(row) {
    if (row?.display_load != null && String(row.display_load).trim() !== '') {
      return String(row.display_load);
    }
    const idx = Number(row?.load_index);
    if (Number.isFinite(idx) && idx >= 0) return String(row.load_index);
    const bus = row?.bus ?? '—';
    return `${row?.is_dc ? 'DC Bus Load @' : 'Bus Load @'}${bus}`;
  }

  function numberOr(value, fallback = 0) {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  }

  function parsePositiveIntText(raw) {
    return String(raw || '').trim()
      ? String(raw || '').split(/[,，\s]+/).map(v => parseInt(v, 10)).filter(v => Number.isFinite(v) && v >= 0)
      : [];
  }

  function parseFaultInputIds(kind) {
    const id = kind === 'DC' ? 'resDcFaultLocations' : 'resAcFaultLocations';
    return parsePositiveIntText(document.getElementById(id)?.value || '');
  }

  function setFaultInputIds(kind, ids) {
    const id = kind === 'DC' ? 'resDcFaultLocations' : 'resAcFaultLocations';
    const el = document.getElementById(id);
    if (el) el.value = Array.from(new Set((ids || []).map(v => parseInt(v, 10)).filter(v => Number.isFinite(v) && v >= 0))).join(',');
  }

  function mappedCompId(mapObj, item, rowIndex) {
    if (!mapObj) return undefined;
    const explicit = Number(item?.index);
    if (Number.isFinite(explicit)) return mapObj[explicit];
    if (mapObj[rowIndex] != null) return mapObj[rowIndex];
    return undefined;
  }

  function modelRowForComp(items, mapObj, compId) {
    const cid = Number(compId);
    const arr = Array.isArray(items) ? items : [];
    for (let i = 0; i < arr.length; i += 1) {
      const item = arr[i] || {};
      if (Number(mappedCompId(mapObj, item, i)) === cid) return { item, index: i, key: item.index ?? i };
    }
    return { item: null, index: -1, key: null };
  }

  function componentCurveTargets() {
    const sys = Canvas.buildSystemJson ? Canvas.buildSystemJson() : { ac: {}, dc: {} };
    const m = Canvas.getCompBusMap ? Canvas.getCompBusMap() : {};
    const rows = [];
    const add = (kind, type, item, compId, idx) => {
      if (compId == null) return;
      const name = item?.name || `${kind}${idx + 1}`;
      const bus = item?.bus ?? item?.pcc_bus ?? item?.aggregation_bus ?? '';
      const rating = positivePower(item?.p_rated_mw, item?.p_mw, item?.p_set_mw, item?.pg_mw);
      rows.push({ kind, type, item, compId, idx, name, bus, rating });
    };
    (sys.ac?.loads || []).forEach((x, i) => add('负荷', 'load', x, mappedCompId(m.load, x, i), i));
    (sys.dc?.loads || []).forEach((x, i) => add('DC负荷', 'dc_load', x, mappedCompId(m.dcLoad, x, i), i));
    (sys.ac?.renewable_gens || []).forEach((x, i) => {
      if (x?.in_service === false || positivePower(x?.p_rated_mw, x?.p_mw) <= 1e-9) return;
      add(String(x.type || '').toLowerCase().includes('wind') ? '风电' : '新能源', 'renewable_gen', x, mappedCompId(m.renGen, x, i), i);
    });
    (sys.ac?.pv_systems || []).forEach((x, i) => {
      if (x?.in_service === false || positivePower(x?.pmax_mw, x?.p_mw, x?.p_rated_mw) <= 1e-9) return;
      add('光伏', 'pv_system', x, mappedCompId(m.pv, x, i), i);
    });
    (sys.dc?.pv_arrays || []).forEach((x, i) => {
      if (x?.in_service === false || positivePower(x?.p_set_mw, x?.p_rated_mw) <= 1e-9) return;
      add('DC光伏', 'dc_pv_array', x, mappedCompId(m.dcPv, x, i), i);
    });
    return rows;
  }

  function renderComponentCurveTargets(context) {
    const divId = context === 'scenario' ? 'scenarioComponentCurveTargets'
      : (context === 'resilience' ? 'resilienceComponentCurveTargets' : 'tspfComponentCurveTargets');
    const div = document.getElementById(divId);
    if (!div) return;
    const rows = componentCurveTargets();
    if (!rows.length) {
      div.innerHTML = '<p class="empty-hint">当前拓扑中没有可展示曲线的负荷/新能源元件。</p>';
      return;
    }
    div.innerHTML = rows.map(r => `<button type="button" class="component-curve-target" data-comp-id="${r.compId}" data-curve-context="${context}">
        ${escapeHtml(r.kind)} #${escapeHtml(String(r.item?.index ?? r.idx + 1))} ${escapeHtml(r.name)}${r.bus !== '' ? ` / Bus ${escapeHtml(String(r.bus))}` : ''}
      </button>`).join('');
    div.querySelectorAll('.component-curve-target').forEach(btn => {
      btn.addEventListener('click', (ev) => {
        ev.preventDefault();
        ev.stopPropagation();
        const cid = parseInt(btn.dataset.compId, 10);
        const curveContext = btn.dataset.curveContext || context;
        if (Number.isInteger(cid)) {
          if (Canvas.panToComponent) Canvas.panToComponent(cid);
          renderSelectedComponentCurve(cid, curveContext);
        }
      });
    });
  }

  function scenarioCurveEntryOptions(data) {
    const entries = [];
    (data?.regular?.clusters || []).forEach((c, i) => entries.push({ family: 'regular', label: `常规 #${c.cluster_id ?? i + 1} ${c.representative_id || ''}`, cluster: c, representative: c.representative }));
    (data?.reliability?.contingencies || []).forEach((g, gi) => (g.clusters || []).forEach((c, ci) => entries.push({ family: 'reliability', label: `可靠性 ${g.contingency?.display_name || g.contingency?.id || gi + 1} / 簇${c.cluster_id ?? ci + 1}`, cluster: c, representative: c.representative })));
    (data?.resilience?.intensities || []).forEach((g, gi) => (g.clusters || []).forEach((c, ci) => entries.push({ family: 'resilience', label: `弹性 ${g.intensity || gi + 1} / 簇${c.cluster_id ?? ci + 1} ${c.representative_id || ''}`, cluster: c, representative: c.representative })));
    return entries.filter(e => e.representative);
  }

  function selectedScenarioCurveEntry() {
    const family = document.getElementById('scenarioCurveFamily')?.value || '';
    const index = parseInt(document.getElementById('scenarioCurveRepresentative')?.value || '0', 10);
    const filtered = _scenarioCurveEntries.filter(e => !family || e.family === family);
    return filtered[Math.max(0, Math.min(Number.isFinite(index) ? index : 0, filtered.length - 1))] || _scenarioCurveEntries[0] || null;
  }

  function representativeForScenarioCurve(data) {
    if (!_scenarioCurveEntries.length) _scenarioCurveEntries = scenarioCurveEntryOptions(data);
    return selectedScenarioCurveEntry()?.representative || null;
  }

  function renderScenarioCurveSelectors(data) {
    _scenarioCurveEntries = scenarioCurveEntryOptions(data);
    const familySel = document.getElementById('scenarioCurveFamily');
    const repSel = document.getElementById('scenarioCurveRepresentative');
    if (!familySel || !repSel) return;
    const families = [
      ['regular', '常规场景'],
      ['reliability', '可靠性 N-1'],
      ['resilience', '弹性台风'],
    ].filter(([key]) => _scenarioCurveEntries.some(e => e.family === key));
    familySel.innerHTML = families.map(([key, label]) => `<option value="${key}">${label}</option>`).join('');
    const refreshReps = () => {
      const filtered = _scenarioCurveEntries.filter(e => e.family === familySel.value);
      repSel.innerHTML = filtered.map((e, i) => `<option value="${i}">${escapeHtml(e.label)}</option>`).join('');
      const chart = document.getElementById('scenarioSelectedComponentCurve');
      if (chart) chart.innerHTML = '<p class="empty-hint">请选择元件查看该聚类代表下的曲线。</p>';
    };
    familySel.onchange = refreshReps;
    repSel.onchange = () => {
      const chart = document.getElementById('scenarioSelectedComponentCurve');
      if (chart) chart.innerHTML = '<p class="empty-hint">已切换聚类代表，请点击元件重新绘制曲线。</p>';
    };
    refreshReps();
  }

  function scenarioCandidateProfileValues(candidate, profileName) {
    const profiles = candidate?.time_series?.profiles || [];
    const profile = profiles.find(p => p.name === profileName);
    return Array.isArray(profile?.values) ? profile.values.map(v => Number(v || 0)) : [];
  }

  function scenarioCandidateComponentLoadCurve(candidate, type, row, comp, isDc) {
    const source = candidate?.component_load_profiles;
    const profiles = Array.isArray(source?.profiles) ? source.profiles : [];
    const map = Array.isArray(source?.load_profile_map) ? source.load_profile_map : [];
    if (!profiles.length || !map.length) return null;
    const item = row?.item || {};
    const itemBus = Number(item.bus ?? comp?.params?.bus);
    const itemIndex = Number(item.index);
    const itemPosition = Number(row?.index);
    const desiredLoadKind = isDc ? 'DC_LOAD' : 'AC_LOAD';
    const desiredBusKind = isDc ? 'DC_BUS' : 'AC_BUS';
    const matches = (m) => {
      const kind = String(m?.kind || desiredLoadKind).toUpperCase();
      if (kind !== desiredLoadKind && kind !== desiredBusKind) return false;
      const pos = Number(m.load_position ?? m.position);
      const idx = Number(m.load_index);
      const bus = Number(m.bus);
      if (Number.isFinite(pos) && Number.isFinite(itemPosition) && pos === itemPosition) return true;
      if (Number.isFinite(idx) && Number.isFinite(itemIndex) && idx === itemIndex) return true;
      // Scenario generation may emit AC_BUS/DC_BUS rows when the source case used
      // bus pd_mw fallback, while imported/generated cases may materialize those
      // bus loads into explicit loads.  Matching by bus keeps both views aligned.
      if (Number.isFinite(bus) && Number.isFinite(itemBus) && bus === itemBus) return true;
      return false;
    };
    const binding = map.find(matches);
    if (!binding) return null;
    const profileId = Number(binding.profile_id);
    const profile = profiles.find(p => Number(p?.id) === profileId || String(p?.id) === String(binding.profile_id));
    const values = Array.isArray(profile?.values) ? profile.values.map(v => Number(v || 0)) : [];
    if (!values.length) return null;
    const baseMw = isDc
      ? positivePower(item.p_mw, item.p_rated_mw, comp?.params?.p_mw, comp?.params?.p_rated_mw) * numberOr(item.scaling ?? comp?.params?.scaling, 1)
      : numberOr(item.p_mw ?? comp?.params?.p_mw, 0) * numberOr(item.scaling ?? comp?.params?.scaling, 1);
    if (!(baseMw > 1e-9)) return null;
    return {
      y: values.map(v => Math.max(0, v) * baseMw),
      profileId,
      source: source?.source || 'backend_per_load_site',
      kind: String(binding.kind || desiredLoadKind).toUpperCase(),
    };
  }

  function positivePower(...values) {
    for (const value of values) {
      const n = Number(value);
      if (Number.isFinite(n) && n > 1e-9) return n;
    }
    return 0;
  }

  function variedPositiveField(items, key) {
    const vals = (items || []).map(x => Number(x?.[key])).filter(v => Number.isFinite(v) && v > 1e-9);
    if (vals.length < 2) return false;
    return Math.max(...vals) - Math.min(...vals) > 1e-6;
  }

  function renewableCurveCapacity(item, params = {}, peers = []) {
    // In this UI users often edit either p_mw or p_rated_mw as the “capacity”.
    // Prefer the field that actually varies across same-type renewable units;
    // otherwise fall back to the first positive value. This avoids all wind units
    // plotting identically when one capacity field remains at its default.
    if (variedPositiveField(peers, 'p_rated_mw')) return positivePower(item?.p_rated_mw, params?.p_rated_mw, item?.p_mw, params?.p_mw);
    if (variedPositiveField(peers, 'p_mw')) return positivePower(item?.p_mw, params?.p_mw, item?.p_rated_mw, params?.p_rated_mw);
    return positivePower(item?.p_rated_mw, params?.p_rated_mw, item?.p_mw, params?.p_mw);
  }

  function scaledCurve(base, numerator, denominator) {
    const den = Number(denominator);
    const ratio = den > 1e-9 ? Number(numerator || 0) / den : 0;
    return (base || []).map(v => Number(v || 0) * ratio);
  }

  function capacityScaledShape(base, capacity) {
    const arr = (base || []).map(v => Math.max(0, Number(v || 0)));
    const cap = Number(capacity || 0);
    const mx = arr.length ? Math.max(...arr) : 0;
    if (!(cap > 1e-9) || !(mx > 1e-9)) return arr;
    return arr.map(v => v / mx * cap);
  }

  function curveRangePad(vals) {
    const arr = (vals || []).flat().map(Number).filter(Number.isFinite);
    if (!arr.length) return null;
    const mn = Math.min(...arr), mx = Math.max(...arr);
    const pad = Math.max((mx - mn) * 0.15, Math.max(Math.abs(mx), 1) * 0.03);
    return [Math.min(0, mn - pad), mx + pad];
  }

  function resolveSelectedComponentCurve(compId, context) {
    const comp = Canvas.getComponent ? Canvas.getComponent(Number(compId)) : null;
    if (!comp) return { error: '未找到画布元件' };
    const sys = Canvas.buildSystemJson ? Canvas.buildSystemJson() : { ac: {}, dc: {} };
    const maps = Canvas.getCompBusMap ? Canvas.getCompBusMap() : {};
    const type = comp.type;
    const label = comp.params?.name || `${type} ${compId}`;
    const make = (y, title, note = '', name = label) => {
      const arr = (y || []).map(v => Number(v || 0));
      return arr.length ? { title, note, traces: [{ x: arr.map((_, i) => i), y: arr, mode: 'lines+markers', name, line: { width: 2 } }] } : { error: '当前结果没有该元件可用曲线' };
    };

    if (context === 'resilience') {
      const d = _lastResilienceData || {};
      const hrs = Array.isArray(d.hours) ? d.hours : [];
      const makeXY = (traces, title, note = '') => traces.length ? { title, note, traces } : { error: '当前弹性评估结果没有该元件可用曲线' };
      const priorityNames = ['关键', '高', '中', '低'];
      const findBusSeries = (kind, busId) => {
        const demand = [], served = [], shed = [];
        let tier = null, importance = null;
        hrs.forEach((_, t) => {
          const kinds = d.bus_supply_kind?.[t] || [];
          const indices = d.bus_supply_index?.[t] || [];
          let pos = -1;
          for (let i = 0; i < indices.length; i += 1) {
            if (String(kinds[i] || 'AC').toUpperCase() === kind && Number(indices[i]) === Number(busId)) { pos = i; break; }
          }
          demand.push(pos >= 0 ? Number(d.bus_supply_demand_mw?.[t]?.[pos] || 0) : 0);
          served.push(pos >= 0 ? Number(d.bus_supply_served_mw?.[t]?.[pos] || 0) : 0);
          shed.push(pos >= 0 ? Number(d.bus_supply_shed_mw?.[t]?.[pos] || 0) : 0);
          if (pos >= 0 && tier === null) tier = Number(d.bus_supply_priority_tier?.[t]?.[pos]);
          if (pos >= 0 && importance === null) importance = Number(d.bus_supply_importance?.[t]?.[pos]);
        });
        return { demand, served, shed, tier, importance, source: 'canonical_bus' };
      };
      const findRichLoadSeries = (isDc, row, comp) => {
        const richLoads = Array.isArray(d.rich_loads) ? d.rich_loads : [];
        if (!richLoads.length || !Array.isArray(d.rich_load_demand_mw)) return null;
        const desired = isDc ? 'DC_LOAD' : 'AC_LOAD';
        const desiredBus = isDc ? 'DC_BUS' : 'AC_BUS';
        const item = row?.item || {};
        const pos = Number(row?.index);
        const idx = Number(item.index);
        const bus = Number(item.bus ?? comp?.params?.bus);
        const matches = (r) => {
          const kind = String(r?.kind || '').toUpperCase();
          if (kind !== desired && kind !== desiredBus) return false;
          if (Number.isFinite(Number(r.position)) && Number.isFinite(pos) && Number(r.position) === pos) return true;
          if (Number.isFinite(Number(r.index)) && Number.isFinite(idx) && Number(r.index) === idx) return true;
          if (Number.isFinite(Number(r.bus)) && Number.isFinite(bus) && Number(r.bus) === bus) return true;
          return false;
        };
        const richPos = richLoads.findIndex(matches);
        if (richPos < 0) return null;
        const pick = (rows) => hrs.map((_, t) => Number(rows?.[t]?.[richPos] || 0));
        return {
          demand: pick(d.rich_load_demand_mw),
          served: pick(d.rich_load_served_mw),
          shed: pick(d.rich_load_shed_mw),
          tier: Number(richLoads[richPos]?.priority_tier),
          importance: Number(richLoads[richPos]?.importance),
          priority: richLoads[richPos]?.priority,
          source: 'rich_load_back_projection',
          rich: richLoads[richPos],
        };
      };
      if (type === 'load' || type === 'dc_load') {
        const isDc = type === 'dc_load';
        const row = modelRowForComp(isDc ? (sys.dc?.loads || []) : (sys.ac?.loads || []), isDc ? maps.dcLoad : maps.load, compId);
        const bus = row.item?.bus ?? comp.params?.bus;
        const s = findRichLoadSeries(isDc, row, comp) || findBusSeries(isDc ? 'DC' : 'AC', bus);
        const pr = s.priority || (Number.isInteger(s.tier) ? priorityNames[Math.max(0, Math.min(3, s.tier))] : '—');
        const note = s.source === 'rich_load_back_projection'
          ? `Rich 负荷回映射：canonical bus 供电/切负荷按“优先级优先、同级按实时需求比例”分摊到该元件；优先级 ${pr}。`
          : `未找到 rich 负荷回映射，展示 canonical bus 聚合结果；负荷优先级：${pr}${Number.isFinite(s.importance) ? `（重要度 ${s.importance.toFixed(1)}）` : ''}`;
        return makeXY([
          { x: hrs, y: s.demand, mode: 'lines+markers', name: '负荷需求', line: { color: '#61afef', width: 2 } },
          { x: hrs, y: s.served, mode: 'lines+markers', name: '实际供电', line: { color: '#98c379', width: 2 } },
          { x: hrs, y: s.shed, mode: 'lines', name: '切负荷', line: { color: '#e06c75', dash: 'dot' } },
        ], `弹性负荷曲线：${label}`, note);
      }
      if (type === 'renewable_gen') {
        const row = modelRowForComp(sys.ac?.renewable_gens || [], maps.renGen, compId);
        const isWind = String(row.item?.type || comp.params?.type || '').toLowerCase().includes('wind');
        const peers = (sys.ac?.renewable_gens || []).filter(x => String(x.type || '').toLowerCase().includes('wind') === isWind);
        const cap = renewableCurveCapacity(row.item, comp.params, peers);
        const isSolar = /solar|pv/i.test(String(row.item?.type || comp.params?.type || row.item?.name || ''));
        const sourceMult = isWind ? d.wind_multipliers : (isSolar ? d.pv_multipliers : d.res_multipliers);
        const mult = Array.isArray(sourceMult) ? sourceMult.map(Number) : (Array.isArray(d.res_multipliers) ? d.res_multipliers.map(Number) : hrs.map(() => 1));
        return makeXY([{ x: hrs, y: mult.map(v => Math.max(0, Number(v || 0)) * cap), mode: 'lines+markers', name: isWind ? '风电出力' : (isSolar ? '光伏出力' : '新能源出力'), line: { color: isSolar ? '#f59e0b' : '#16a34a', width: 2 } }], `弹性${isWind ? '风电' : (isSolar ? '光伏' : '新能源')}曲线：${label}`, '按弹性评估对应类型可用率和该元件容量估算 48h 出力。');
      }
      if (type === 'pv_system' || type === 'dc_pv_array') {
        const isDc = type === 'dc_pv_array';
        const row = modelRowForComp(isDc ? (sys.dc?.pv_arrays || []) : (sys.ac?.pv_systems || []), isDc ? maps.dcPv : maps.pv, compId);
        const cap = isDc ? positivePower(row.item?.p_set_mw, comp.params?.p_set_mw) : positivePower(row.item?.p_mw, row.item?.p_rated_mw, comp.params?.p_mw, comp.params?.p_rated_mw);
        const mult = Array.isArray(d.pv_multipliers) ? d.pv_multipliers.map(Number) : (Array.isArray(d.res_multipliers) ? d.res_multipliers.map(Number) : hrs.map(() => 1));
        return makeXY([{ x: hrs, y: mult.map(v => Math.max(0, Number(v || 0)) * cap), mode: 'lines+markers', name: '光伏出力', line: { color: '#f59e0b', width: 2 } }], `弹性光伏曲线：${label}`, '按弹性评估光伏可用率和该元件容量估算 48h 出力。');
      }
      return { error: '该元件暂不支持弹性曲线展示' };
    }

    if (context === 'timeSeries') {
      const d = _lastTspfData || {};
      if (type === 'load') {
        const row = modelRowForComp(sys.ac?.loads || [], maps.load, compId);
        const y = d.load_demand?.[row.index];
        if (y) return make(y, `AC负荷曲线：${label}`);
        const denom = (sys.ac?.loads || []).reduce((a, x) => a + numberOr(x.p_mw, 0), 0);
        return make(scaledCurve(d.total_load || [], row.item?.p_mw || comp.params?.p_mw || 0, denom), `AC负荷估算曲线：${label}`, '后端未返回精确 AC 负荷曲线，按额定负荷占比由系统总负荷估算。');
      }
      if (type === 'dc_load') {
        const row = modelRowForComp(sys.dc?.loads || [], maps.dcLoad, compId);
        const y = d.dc_load_demand?.[row.index];
        const denom = (sys.dc?.loads || []).reduce((a, x) => a + numberOr(x.p_mw, 0), 0);
        return y ? make(y, `DC负荷曲线：${label}`) : make(scaledCurve(d.total_load || [], row.item?.p_mw || comp.params?.p_mw || 0, denom), `DC负荷估算曲线：${label}`, '按 DC 负荷额定占比估算。');
      }
      if (type === 'renewable_gen') {
        const row = modelRowForComp(sys.ac?.renewable_gens || [], maps.renGen, compId);
        const isWind = String(row.item?.type || comp.params?.type || '').toLowerCase().includes('wind');
        const peers = (sys.ac?.renewable_gens || []).filter(x => String(x.type || '').toLowerCase().includes('wind') === isWind);
        const cap = renewableCurveCapacity(row.item, comp.params, peers);
        const titleName = isWind ? '风电出力曲线' : '新能源出力曲线';
        const rows = Array.isArray(d.renewable_dispatch) ? d.renewable_dispatch : [];
        const raw = rows[row.index] || rows[row.key] || rows.find((_, i) => i === row.index || i === row.key) || [];
        const profileName = isWind ? 'scenario_wind_scale' : (/solar|pv/i.test(String(row.item?.type || comp.params?.type || row.item?.name || '')) ? 'scenario_pv_scale' : 'scenario_renewable_scale');
        const tsProfile = extractGeneratedScenarioProfileValues(getImportedGeneratedScenarioTimeSeries('regular'), profileName);
        const fallback = tsProfile.length ? tsProfile.map(v => Math.max(0, Number(v || 0)) * cap) : [];
        const y = raw.length ? capacityScaledShape(raw, cap) : fallback;
        return make(y, `${titleName}：${label}`, raw.length ? '按该元件容量将出力曲线归一化到 MW，确保不同容量风电峰值不同。' : '后端结果未返回该新能源调度曲线，按导入场景时序和该元件容量估算。');
      }
      if (type === 'pv_system') {
        const row = modelRowForComp(sys.ac?.pv_systems || [], maps.pv, compId);
        return make(d.ac_pv_dispatch?.[row.index], `光伏出力曲线：${label}`);
      }
      if (type === 'dc_pv_array') {
        const row = modelRowForComp(sys.dc?.pv_arrays || [], maps.dcPv, compId);
        return make(d.dc_pv_dispatch?.[row.index], `DC光伏出力曲线：${label}`);
      }
      if (type === 'generator') {
        const row = modelRowForComp(sys.ac?.generators || [], maps.gen, compId);
        return make(d.gen_dispatch?.[row.index], `发电机出力曲线：${label}`);
      }
      return { error: '该元件暂不支持曲线展示' };
    }

    const candidate = representativeForScenarioCurve(_lastScenarioGenerationData);
    if (!candidate) return { error: '暂无代表场景曲线，请先运行场景生成' };
    const load = scenarioCandidateProfileValues(candidate, 'total_load_mw');
    const pv = scenarioCandidateProfileValues(candidate, 'pv_mw');
    const wind = scenarioCandidateProfileValues(candidate, 'wind_mw');
    const ren = scenarioCandidateProfileValues(candidate, 'total_renewable_mw');
    if (type === 'load' || type === 'dc_load') {
      const isDc = type === 'dc_load';
      const row = modelRowForComp(isDc ? (sys.dc?.loads || []) : (sys.ac?.loads || []), isDc ? maps.dcLoad : maps.load, compId);
      const componentCurve = scenarioCandidateComponentLoadCurve(candidate, type, row, comp, isDc);
      if (componentCurve) {
        return make(componentCurve.y, `场景${isDc ? 'DC' : 'AC'}负荷曲线：${label}`,
          `使用后端逐负荷场景 profile #${componentCurve.profileId}（${componentCurve.kind}），与导出 JSON 和弹性评估绑定一致。`);
      }
      const total = (sys.ac?.loads || []).reduce((a, x) => a + numberOr(x.p_mw, 0) * numberOr(x.scaling, 1), 0) + (sys.dc?.loads || []).reduce((a, x) => a + numberOr(x.p_mw, 0) * numberOr(x.scaling, 1), 0);
      const base = scaledCurve(load, numberOr(row.item?.p_mw ?? comp.params?.p_mw, 0) * numberOr(row.item?.scaling ?? comp.params?.scaling, 1), total);
      return make(base, `场景${isDc ? 'DC' : 'AC'}负荷估算曲线：${label}`, '后端未返回/未匹配逐负荷 profile，按额定负荷占比由代表场景总负荷估算。');
    }
    if (type === 'pv_system' || type === 'dc_pv_array') {
      const isDc = type === 'dc_pv_array';
      const row = modelRowForComp(isDc ? (sys.dc?.pv_arrays || []) : (sys.ac?.pv_systems || []), isDc ? maps.dcPv : maps.pv, compId);
      const total = (sys.ac?.pv_systems || []).reduce((a, x) => a + numberOr(x.p_mw ?? x.p_rated_mw, 0), 0) + (sys.dc?.pv_arrays || []).reduce((a, x) => a + numberOr(x.p_set_mw, 0), 0);
      const cap = isDc ? numberOr(row.item?.p_set_mw ?? comp.params?.p_set_mw, 0) : numberOr(row.item?.p_mw ?? row.item?.p_rated_mw ?? comp.params?.p_mw, 0);
      return make(scaledCurve(pv, cap, total), `场景${isDc ? 'DC' : ''}光伏估算曲线：${label}`, '按光伏容量占比由代表场景光伏总出力估算。');
    }
    if (type === 'renewable_gen') {
      const row = modelRowForComp(sys.ac?.renewable_gens || [], maps.renGen, compId);
      const isWind = String(row.item?.type || comp.params?.type || '').toLowerCase().includes('wind');
      const isSolar = /solar|pv/i.test(String(row.item?.type || comp.params?.type || row.item?.name || ''));
      const source = isWind ? wind : (isSolar ? pv : ren);
      const sameType = (sys.ac?.renewable_gens || []).filter(x => {
        const t = String(x.type || x.name || '').toLowerCase();
        return isWind ? t.includes('wind') : (isSolar ? /solar|pv/.test(t) : (!t.includes('wind') && !/solar|pv/.test(t)));
      });
      const cap = renewableCurveCapacity(row.item, comp.params, sameType);
      const kindName = isWind ? '风电' : (isSolar ? '光伏' : '新能源');
      return make(capacityScaledShape(source, cap), `场景${kindName}估算曲线：${label}`, `按该${kindName}元件容量将代表场景曲线归一化到 MW。`);
    }
    return { error: '该元件暂不支持场景曲线展示' };
  }

  function baseDeterministicProfile(length, seed) {
    // Scenario generation only returns system-level load. For per-load display,
    // add a deterministic component-specific shape: different phase, amplitude,
    // low-frequency drift and short-term jitter. This keeps curves reproducible
    // while making individual loads visibly different.
    let x = (Number(seed) || 1) * 1103515245 + 12345;
    const s = Number(seed) || 1;
    const phase = (s * 5) % 24;
    const dailyAmp = 0.22 + ((s * 37) % 11) / 100;      // 0.22–0.32
    const driftAmp = 0.08 + ((s * 19) % 7) / 100;       // 0.08–0.14
    const jitterAmp = 0.18 + ((s * 13) % 7) / 100;      // 0.18–0.24
    let smoothNoise = 0;
    return Array.from({ length }, (_, i) => {
      x = (x * 1664525 + 1013904223) >>> 0;
      const raw = (x / 4294967295) - 0.5;
      smoothNoise = 0.72 * smoothNoise + 0.28 * raw;
      const daily = Math.sin((2 * Math.PI * ((i + phase) % 24)) / 24 - Math.PI / 2);
      const drift = Math.sin((2 * Math.PI * (i + s * 3)) / Math.max(24, Math.min(length || 48, 168)));
      return Math.max(0.10, 1 + dailyAmp * daily + driftAmp * drift + jitterAmp * smoothNoise);
    });
  }

  function renderSelectedComponentCurve(compId, context) {
    const chartId = context === 'scenario' ? 'scenarioSelectedComponentCurve'
      : (context === 'resilience' ? 'resilienceSelectedComponentCurve' : 'tspfSelectedComponentCurve');
    const chart = document.getElementById(chartId);
    if (!chart) return;
    const info = resolveSelectedComponentCurve(compId, context);
    if (info.error) {
      chart.innerHTML = `<p class="empty-hint">${escapeHtml(info.error)}</p>`;
      return;
    }
    const range = curveRangePad(info.traces.map(t => t.y));
    const whiteChart = context === 'resilience';
    const axisTheme = whiteChart ? { gridcolor: '#d1d5db', linecolor: '#111827', tickcolor: '#111827', zerolinecolor: '#9ca3af' } : { gridcolor: '#3e4451' };
    const layout = {
      paper_bgcolor: whiteChart ? '#ffffff' : 'rgba(0,0,0,0)',
      plot_bgcolor: whiteChart ? '#ffffff' : 'rgba(0,0,0,0)',
      font: { color: whiteChart ? '#111827' : '#abb2bf', size: 11 }, margin: { l: 55, r: 15, t: 42, b: 40 },
      xaxis: { ...axisTheme, title: context === 'resilience' ? '小时' : '时间步' },
      yaxis: { ...axisTheme, title: 'MW', ...(range ? { range } : {}) },
      title: info.title,
      annotations: [],
    };
    if (window.Plotly) Plotly.newPlot(chart, info.traces, layout, { responsive: true });
    else chart.innerHTML = '<p class="empty-hint">Plotly 未加载，无法展示曲线。</p>';
  }

  function listFaultableBranches() {
    const sys = Canvas.buildSystemJson ? Canvas.buildSystemJson() : { ac: {}, dc: {} };
    const m = Canvas.getCompBusMap ? Canvas.getCompBusMap() : {};
    const normalize = (kind, br, i) => {
      const idx = Number(br.index);
      const compId = kind === 'DC' ? m.dcBranch?.[idx] : m.branch?.[idx];
      const name = br.name || `${kind} branch ${idx}`;
      const label = `${kind} #${idx} | ${name} | ${br.from_bus ?? '-'} → ${br.to_bus ?? '-'}`;
      return { kind, index: idx, name, from: br.from_bus, to: br.to_bus, compId, label };
    };
    return {
      AC: (sys.ac?.branches || []).map((br, i) => normalize('AC', br, i)).filter(x => Number.isFinite(x.index)),
      DC: (sys.dc?.branches || []).map((br, i) => normalize('DC', br, i)).filter(x => Number.isFinite(x.index)),
    };
  }

  function validateFaultBranchIds() {
    const branches = listFaultableBranches();
    const acSet = new Set((branches.AC || []).map(x => x.index));
    const dcSet = new Set((branches.DC || []).map(x => x.index));
    const badAc = parseFaultInputIds('AC').filter(id => !acSet.has(id));
    const badDc = parseFaultInputIds('DC').filter(id => !dcSet.has(id));
    if (badAc.length || badDc.length) {
      const msg = `故障支路不存在：${badAc.length ? `AC ${badAc.join(',')}` : ''}${badAc.length && badDc.length ? '；' : ''}${badDc.length ? `DC ${badDc.join(',')}` : ''}。请检查拓扑支路表中的 ID 后重新输入。`;
      log(msg, 'warn');
      setStatus('故障支路 ID 无效', 'error');
      return false;
    }
    return true;
  }

  function markResilienceFaultBranches() {
    if (!Canvas.state?.components) return;
    Canvas.state.components.forEach(c => c.el?.classList?.remove('fault-highlighted'));
    const maps = Canvas.getCompBusMap ? Canvas.getCompBusMap() : {};
    const mark = (kind, id) => {
      const compId = kind === 'DC' ? maps.dcBranch?.[id] : maps.branch?.[id];
      const comp = compId != null && Canvas.getComponent ? Canvas.getComponent(compId) : null;
      if (comp?.el) comp.el.classList.add('fault-highlighted');
    };
    parseFaultInputIds('AC').forEach(id => mark('AC', id));
    parseFaultInputIds('DC').forEach(id => mark('DC', id));
  }

  // Extract the first bus id from a component name like "F3-Line-20-21" → 20.
  function busIdFromComponentName(name) {
    if (!name) return null;
    const m = String(name).match(/(\d+)\D+(\d+)\s*$/);
    return m ? parseInt(m[1], 10) : null;
  }

  // Trigger a JSON file download in the browser.
  function downloadJsonFile(filename, obj, options = {}) {
    try {
      let text;
      try {
        text = JSON.stringify(obj, null, options.compact ? 0 : 2);
      } catch (prettyErr) {
        if (options.compact) throw prettyErr;
        text = JSON.stringify(obj);
      }
      const blob = new Blob([text], { type: 'application/json;charset=utf-8' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url; a.download = filename;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      setTimeout(() => URL.revokeObjectURL(url), 1000);
      log(`已导出: ${filename}`, 'success');
      return true;
    } catch (e) {
      log(`导出失败: ${e.message || e}`, 'error');
      return false;
    }
  }

  function tsTagForFilename() {
    const d = new Date();
    const pad = (n) => String(n).padStart(2, '0');
    return `${d.getFullYear()}${pad(d.getMonth()+1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
  }

  function buildResultsExportBundle() {
    const results = {};
    const add = (key, value) => {
      if (value !== null && value !== undefined) results[key] = value;
    };
    add('power_flow', _lastPfData);
    add('optimal_power_flow', _lastOpfData);
    add('time_series_power_flow', _lastTspfData);
    add('campus_integrated_energy', _lastIntegratedEnergyData);
    add('annual_production_simulation', _lastAnnualData);
    add('carbon_flow', _lastCarbonData);
    add('dynamic_carbon_flow', _lastDynamicCarbonData);
    add('transient_simulation', _lastTransientData);
    add('reliability', _lastReliabilityData);
    add('resilience', _lastResilienceData);
    add('scenario_generation', _lastScenarioGenerationData);
    add('topology_analysis', _lastTopoAnalysisData);
    add('network_reduction', _lastNetReductionData);

    return {
      schema: 'hacdcpf.gui.results.bundle.v1',
      exported_at: new Date().toISOString(),
      active_result_group: document.getElementById('resultsContent')?.dataset.activeGroup || '',
      display_unit: getPowerUnit(),
      result_count: Object.keys(results).length,
      results,
    };
  }

  function exportAllCachedResults() {
    const bundle = buildResultsExportBundle();
    if (!bundle.result_count) {
      log('暂无可导出的分析结果，请先运行至少一个计算', 'warn');
      return false;
    }
    return downloadJsonFile(`gui_results_bundle_${tsTagForFilename()}.json`, bundle);
  }

  // ========== API Client ==========
  async function parseJsonResponse(res) {
    const text = await res.text();
    if (!text.trim()) return {};
    try {
      return JSON.parse(text);
    } catch (e) {
      return { error: text || res.statusText || e.message };
    }
  }

  async function apiPost(path, body = {}, options = {}) {
    const url = `${API_BASE}${path}`;
    if (!options.quiet) log(`POST ${path}`, 'info');
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const data = await parseJsonResponse(res);
      if (!res.ok) {
        if (!options.quiet) log(`Error: ${data.error || res.statusText}`, 'error');
        return null;
      }
      return data;
    } catch (e) {
      if (!options.quiet) log(`Network error: ${e.message}`, 'error');
      return null;
    }
  }

  // Like apiPost but surfaces the backend error message instead of swallowing
  // it.  Returns { ok, data, error } so callers can show a specific reason
  // (e.g. an unknown fault bus id rejected by the server).
  async function apiPostResult(path, body = {}, options = {}) {
    const url = `${API_BASE}${path}`;
    if (!options.quiet) log(`POST ${path}`, 'info');
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const data = await parseJsonResponse(res);
      if (!res.ok) {
        const error = (data && (data.error || data.message)) || res.statusText;
        if (!options.quiet) log(`Error: ${error}`, 'error');
        return { ok: false, data, error };
      }
      return { ok: true, data, error: null };
    } catch (e) {
      if (!options.quiet) log(`Network error: ${e.message}`, 'error');
      return { ok: false, data: null, error: e.message };
    }
  }

  async function apiGet(path, options = {}) {
    const url = `${API_BASE}${path}`;
    try {
      const res = await fetch(url);
      const data = await parseJsonResponse(res);
      if (!res.ok) {
        if (!options.quiet) log(`Error: ${data.error || res.statusText}`, 'error');
        return null;
      }
      return data;
    } catch (e) {
      if (!options.quiet) log(`Network error: ${e.message}`, 'error');
      return null;
    }
  }

  function firstNonEmptyArray(...values) {
    for (const value of values) {
      if (Array.isArray(value) && value.length > 0) return value;
    }
    return [];
  }

  function firstFiniteNumber(...values) {
    for (const value of values) {
      if (value === null || value === undefined || value === '') continue;
      const n = Number(value);
      if (Number.isFinite(n)) return n;
    }
    return 0;
  }

  function normalizePowerFlowResult(data) {
    if (!data || typeof data !== 'object') return data;
    data.geo_ac_branches = firstNonEmptyArray(data.geo_ac_branches, data.branch_flows);
    data.geo_dc_branches = firstNonEmptyArray(data.geo_dc_branches, data.dc_branch_flows);
    data.dc_branch_flows = firstNonEmptyArray(data.dc_branch_flows, data.geo_dc_branches);
    data.vsc_transfers = firstNonEmptyArray(data.vsc_transfers, data.geo_vsc);
    data.geo_vsc = firstNonEmptyArray(data.geo_vsc, data.vsc_transfers);
    data.dcdc_transfers = firstNonEmptyArray(data.dcdc_transfers, data.geo_dcdc);
    data.geo_dcdc = firstNonEmptyArray(data.geo_dcdc, data.dcdc_transfers);
    data.ac_switch_flows = Array.isArray(data.ac_switch_flows) ? data.ac_switch_flows : [];
    data.ac_circuit_breaker_flows = Array.isArray(data.ac_circuit_breaker_flows) ? data.ac_circuit_breaker_flows : [];
    data.dc_circuit_breaker_flows = Array.isArray(data.dc_circuit_breaker_flows) ? data.dc_circuit_breaker_flows : [];
    data.geo_er = Array.isArray(data.geo_er) ? data.geo_er : [];
    data.geo_gen = firstNonEmptyArray(data.geo_gen, data.generator_dispatch);
    if (!data.geo_gen.length && Array.isArray(data.pg_mw) && data.pg_mw.length) {
      const qg = Array.isArray(data.qg_mvar) ? data.qg_mvar : [];
      data.geo_gen = data.pg_mw.map((p, i) => ({
        position: i,
        index: i,
        canvas_type: 'gen',
        canvas_index: i,
        pg_mw: p,
        qg_mvar: qg[i],
      }));
    }
    data.geo_trafo3w = Array.isArray(data.geo_trafo3w) ? data.geo_trafo3w : [];
    data.component_results = Array.isArray(data.component_results) ? data.component_results : [];
    data.dc_storage_results = Array.isArray(data.dc_storage_results) ? data.dc_storage_results : [];
    return data;
  }

  // ========== Status ==========
  function setStatus(text, type = '') {
    const badge = document.getElementById('statusBadge');
    if (badge) {
      badge.textContent = text;
      badge.className = 'badge' + (type ? ' ' + type : '');
    }
  }

  function nextPaint() {
    return new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
  }

  async function showFormatConversionStatus(text) {
    setStatus(text, 'busy');
    // Give the browser a chance to repaint the top-right badge before JSON
    // parsing or workbook upload/download work starts, so large files don't look frozen.
    await nextPaint();
  }

  // ========== Load Built-in Cases ==========
  function fillSelectOptions(selectId, values, placeholder) {
    const select = document.getElementById(selectId);
    if (!select) return;
    select.innerHTML = `<option value="">${escapeHtml(placeholder)}</option>`;
    (values || []).forEach(value => {
      const opt = document.createElement('option');
      opt.value = value;
      opt.textContent = value;
      select.appendChild(opt);
    });
  }

  async function loadCaseList() {
    const data = await apiGet('/api/cases');
    if (!data) return;
    fillSelectOptions('caseSelect', data.cases, '-- 加载算例 --');
    fillSelectOptions('ioCaseSelect', data.cases, '-- 选择算例 --');
    log(`已加载 ${(data.cases || []).length} 个内置算例`, 'success');
  }

  async function loadMatpowerFileList() {
    const data = await apiGet('/api/matpower_files');
    if (!data) return;
    fillSelectOptions('matpowerSelect', data.files, '-- MATPOWER文件 --');
    fillSelectOptions('ioMatpowerSelect', data.files, '-- MATPOWER文件 --');
    log(`已加载 ${(data.files || []).length} 个MATPOWER文件`, 'success');
  }

  async function loadMatpowerCase(filename) {
    if (!filename) return;
    return trackActiveLoad(async () => {
      setStatus('加载MATPOWER...', 'busy');
      const data = await apiPost('/api/session/load_matpower', { filename });
      if (data) {
        log(`已加载MATPOWER文件: ${filename}`, 'success');
        if (data._raw_json) {
          try {
            const sys = JSON.parse(data._raw_json);
            Canvas.loadFromSystemJson(sys);
            _canvasDirty = false;  // backend already has the correct system
            updateResilienceSwitchDefault();
            invalidateAnalysisResults();
            log(`画布已更新: ${data.counts ? JSON.stringify(data.counts) : ''}`, 'info');
          } catch (e) {
            log(`JSON解析失败: ${e.message}`, 'error');
          }
        }
        setStatus('就绪');

        // Auto-run power flow after import
        log('自动运行潮流计算...', 'info');
        await runPowerFlow();
        showModelIoStatus('MATPOWER 导入完成', [
          ['文件', filename],
          ['AC母线', data.counts?.ac_buses ?? data.ac_buses ?? ''],
          ['AC支路', data.counts?.ac_branches ?? data.ac_branches ?? ''],
          ['发电机', data.counts?.generators ?? data.generators ?? ''],
        ], { subtitle: '已同步到画布并自动运行潮流' });
      } else {
        setStatus('加载失败', 'error');
      }
      return data;
    });
  }

  async function loadBuiltinCase(caseName) {
    if (!caseName) return;
    setStatus('加载中...', 'busy');
    const data = await apiPost('/api/session/load_builtin', { case: caseName });
    if (data) {
      log(`已加载算例: ${caseName}`, 'success');
      // Parse and display on canvas
      if (data._raw_json) {
        try {
          const sys = JSON.parse(data._raw_json);
          Canvas.loadFromSystemJson(sys);
          _canvasDirty = false;  // backend already has the correct system
          updateResilienceSwitchDefault();
          invalidateAnalysisResults('算例已变更，旧潮流和碳流结果已失效');
          log(`画布已更新: ${data.counts ? JSON.stringify(data.counts) : ''}`, 'info');
        } catch (e) {
          log(`JSON解析失败: ${e.message}`, 'error');
        }
      }
      setStatus('就绪');
      showModelIoStatus('内置算例加载完成', [
        ['算例', caseName],
        ['AC母线', data.counts?.ac_buses ?? data.ac_buses ?? ''],
        ['AC支路', data.counts?.ac_branches ?? data.ac_branches ?? ''],
        ['DC母线', data.counts?.dc_buses ?? data.dc_buses ?? ''],
        ['VSC', data.counts?.vsc_converters ?? data.vsc_converters ?? ''],
      ], { subtitle: '当前后端会话与画布已更新' });
    } else {
      setStatus('加载失败', 'error');
    }
  }

  // Render a freshly-loaded backend system onto the canvas (shared by the ETAP
  // import paths); logs any permissive-import warnings.
  function applyLoadedSystem(data, label) {
    if (data._raw_json) {
      try {
        const sys = JSON.parse(data._raw_json);
	        Canvas.loadFromSystemJson(sys);
	        _canvasDirty = false;  // backend already has the correct system
	        if ((data._has_three_phase_ac || sys.three_phase_ac) && document.getElementById('pfMethod')) {
	          document.getElementById('pfMethod').value = 'three_phase';
	        }
	        updateResilienceSwitchDefault();
	        invalidateAnalysisResults('系统已变更，旧潮流和碳流结果已失效');
        log(`画布已更新: ${data.counts ? JSON.stringify(data.counts) : ''}`, 'info');
      } catch (e) {
        log(`JSON解析失败: ${e.message}`, 'error');
      }
    }
    const warns = data._etap_warnings;
    if (Array.isArray(warns) && warns.length) {
      log(`${label}：${warns.length} 条导入告警（宽松模式）`, 'warn');
    }
  }

  async function loadGridlabd(file) {
    if (!file) return;
    setStatus('导入GridLAB-D...', 'busy');
    try {
      const glm = await file.text();
      const data = await apiPost('/api/session/load_gridlabd', { glm_string: glm });
      if (!data) { setStatus('加载失败', 'error'); return; }
      log(`已导入GridLAB-D GLM: ${file.name}`, 'success');
      applyLoadedSystem(data, 'GridLAB-D GLM');
      showModelIoStatus('GridLAB-D 导入完成', [
        ['文件', file.name],
        ['AC母线', data.counts?.ac_buses ?? data.ac_buses ?? ''],
        ['AC支路', data.counts?.ac_branches ?? data.ac_branches ?? ''],
        ['负荷', data.counts?.loads ?? data.loads ?? ''],
      ], { subtitle: 'GLM 文本转换到当前系统', warnings: data._io_warnings || data._gridlabd_warnings || [] });
      setStatus('就绪');
    } catch (e) {
      log(`导入GridLAB-D失败: ${e.message}`, 'error');
      setStatus('加载失败', 'error');
    }
  }

  async function loadOpendss(file) {
    if (!file) return;
    setStatus('导入OpenDSS...', 'busy');
    try {
      const dss = await file.text();
	      const data = await apiPost('/api/session/load_opendss', { dss_string: dss, filename: file.name });
      if (!data) { setStatus('加载失败', 'error'); return; }
      log(`已导入OpenDSS DSS: ${file.name}`, 'success');
      applyLoadedSystem(data, 'OpenDSS DSS');
	      showModelIoStatus('OpenDSS 导入完成', [
	        ['文件', file.name],
	        ['导入路径', data._opendss_import_mode || ''],
	        ['AC母线', data.counts?.ac_buses ?? data.ac_buses ?? ''],
	        ['AC支路', data.counts?.ac_branches ?? data.ac_branches ?? ''],
	        ['三相母线', data.counts?.tp_buses ?? data.tp_buses ?? ''],
	        ['三相线路', data.counts?.tp_lines ?? data.tp_lines ?? ''],
	        ['负荷', data.counts?.loads ?? data.loads ?? ''],
	      ], { subtitle: data._has_three_phase_ac ? 'DSS 已作为三相 abc 模型导入，并生成等值单线图' : 'DSS 文本转换到当前系统', warnings: data._io_warnings || data._opendss_warnings || [] });
      setStatus('就绪');
    } catch (e) {
      log(`导入OpenDSS失败: ${e.message}`, 'error');
      setStatus('加载失败', 'error');
    }
  }

  function currentCaseHasSwitches() {
    try {
      const sys = Canvas.buildSystemJson ? Canvas.buildSystemJson() : null;
      return Boolean(
        (sys?.ac?.switches || []).length ||
        (sys?.ac?.circuit_breakers || []).length ||
        (sys?.dc?.dc_circuit_breakers || []).length
      );
    } catch (_) {
      return false;
    }
  }

  function updateResilienceSwitchDefault(force = false) {
    const cb = document.getElementById('resConsiderSwitches');
    if (!cb) return;
    if (!force && cb.dataset.userTouched === '1') return;
    cb.checked = currentCaseHasSwitches();
  }

  // Import an ETAP-schema .xlsx workbook (raw binary upload -> load_etap).
  async function loadEtapXlsx(file) {
    if (!file) return;
    setStatus('导入ETAP工作簿...', 'busy');
    try {
      const buf = await file.arrayBuffer();
      const res = await fetch(`${API_BASE}/api/session/load_etap_xlsx`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/octet-stream' },
        body: buf,
      });
      const data = await res.json();
      if (!res.ok || data.error) {
        log(`导入ETAP工作簿失败: ${data.error || res.statusText}`, 'error');
        setStatus('加载失败', 'error');
        return;
      }
      log(`已导入ETAP工作簿: ${file.name}`, 'success');
      applyLoadedSystem(data, 'ETAP工作簿');
      showModelIoStatus('ETAP 工作簿导入完成', [
        ['文件', file.name],
        ['AC母线', data.counts?.ac_buses ?? data.ac_buses ?? ''],
        ['AC支路', data.counts?.ac_branches ?? data.ac_branches ?? ''],
        ['VSC', data.counts?.vsc_converters ?? data.vsc_converters ?? ''],
      ], { subtitle: 'ETAP xlsx 转换到当前系统', warnings: data._etap_warnings || [] });
      setStatus('就绪');
    } catch (e) {
      log(`导入ETAP工作簿失败: ${e.message}`, 'error');
      setStatus('加载失败', 'error');
    }
  }

  // Import a native ETAP project .xml (text upload -> load_etap_xml).
  async function loadEtapXml(file) {
    if (!file) return;
    setStatus('导入ETAP工程...', 'busy');
    try {
      const xml = await file.text();
      const data = await apiPost('/api/session/load_etap_xml', { xml_string: xml });
      if (!data) { setStatus('加载失败', 'error'); return; }
      log(`已导入ETAP工程: ${file.name}`, 'success');
      applyLoadedSystem(data, 'ETAP工程');
      showModelIoStatus('ETAP XML 导入完成', [
        ['文件', file.name],
        ['AC母线', data.counts?.ac_buses ?? data.ac_buses ?? ''],
        ['AC支路', data.counts?.ac_branches ?? data.ac_branches ?? ''],
        ['VSC', data.counts?.vsc_converters ?? data.vsc_converters ?? ''],
      ], { subtitle: 'ETAP XML 转换到当前系统', warnings: data._etap_warnings || [] });
      setStatus('就绪');
    } catch (e) {
      log(`导入ETAP工程失败: ${e.message}`, 'error');
      setStatus('加载失败', 'error');
    }
  }

  async function createNewSystem() {
    setStatus('创建中...', 'busy');
    const data = await apiPost('/api/session/new_empty');
    if (data) {
      Canvas.clearAll();
      _canvasDirty = false;  // backend already has the empty system
      updateResilienceSwitchDefault(true);
      invalidateAnalysisResults('系统已清空，旧潮流和碳流结果已失效');
      log('已创建空白系统', 'success');
      showModelIoStatus('已创建空白系统', [
        ['系统名', 'New System'],
        ['Base MVA', '100'],
      ], { subtitle: '当前会话已重置' });
      setStatus('就绪');
    } else {
      setStatus('创建失败', 'error');
    }
  }

  // ========== Export / Import ==========
  async function exportJson() {
    // Build system from canvas
    const sys = Canvas.buildSystemJson();
    const jsonStr = JSON.stringify(sys, null, 2);
    const blob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'power_system.json';
    a.click();
    URL.revokeObjectURL(url);
    log('已导出系统JSON', 'success');
    showModelIoStatus('JSON 导出完成', [
      ['文件', 'power_system.json'],
      ['大小', `${jsonStr.length} bytes`],
    ], { subtitle: '从当前画布生成' });
  }

  function downloadTextFile(filename, text, mimeType = 'text/plain;charset=utf-8') {
    const blob = new Blob([text], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  // Export the current system as an ETAP-schema .xlsx workbook (one sheet per
  // ETAP element class).  The workbook is generated server-side by save_etap();
  // here we push the latest canvas to the backend (if edited), then stream the
  // binary response to a browser download.
  async function exportEtap() {
    setStatus('导出ETAP中...', 'busy');
    try {
      // Make sure the backend session reflects any unsaved canvas edits.  A
      // non-forced sync is a no-op when the canvas is unchanged, preserving the
      // full-fidelity system originally loaded into the backend.
      const ok = await syncToBackend();
      if (!ok) { setStatus('导出失败', 'error'); return; }
      // Send an explicit (empty JSON) body so the request always carries a
      // Content-Length; a body-less POST can stall some HTTP servers.
      const resp = await fetch('/api/session/export_etap', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: '{}',
      });
      if (!resp.ok) {
        let msg = 'HTTP ' + resp.status;
        try { const j = await resp.json(); if (j && j.error) msg = j.error; } catch (_) {}
        throw new Error(msg);
      }
      const blob = await resp.blob();
      let filename = 'system.xlsx';
      const cd = resp.headers.get('Content-Disposition');
      if (cd) {
        const m = /filename="?([^"]+)"?/.exec(cd);
        if (m && m[1]) filename = m[1];
      }
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      a.click();
      URL.revokeObjectURL(url);
      log(`已导出ETAP工作簿: ${filename}`, 'success');
      showModelIoStatus('ETAP 工作簿导出完成', [
        ['文件', filename],
        ['格式', 'xlsx'],
      ], { subtitle: '从后端当前系统生成' });
      setStatus('就绪');
    } catch (err) {
      log(`导出ETAP失败: ${err.message}`, 'error');
      setStatus('导出失败', 'error');
    }
  }

  // Export the current system as a native ETAP project .xml (PDE document) that
  // ETAP can re-import. The XML is generated server-side by save_etap_xml().
  async function exportEtapXml() {
    setStatus('导出ETAP XML中...', 'busy');
    try {
      const ok = await syncToBackend();
      if (!ok) { setStatus('导出失败', 'error'); return; }
      const data = await apiPost('/api/session/export_etap_xml', {});
      if (!data || data.error) throw new Error((data && data.error) || '导出失败');
      const blob = new Blob([data.xml_string], { type: 'application/xml' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `${data.name || 'system'}.xml`;
      a.click();
      URL.revokeObjectURL(url);
      log(`已导出ETAP XML: ${a.download}`, 'success');
      showModelIoStatus('ETAP XML 导出完成', [
        ['文件', a.download],
        ['大小', `${(data.xml_string || '').length} bytes`],
      ], { subtitle: '从后端当前系统生成' });
      setStatus('就绪');
    } catch (err) {
      log(`导出ETAP XML失败: ${err.message}`, 'error');
      setStatus('导出失败', 'error');
    }
  }

  async function exportMatpower() {
    setStatus('导出MATPOWER中...', 'busy');
    try {
      const ok = await syncToBackend();
      if (!ok) { setStatus('导出失败', 'error'); return; }
      const data = await apiPost('/api/session/export_matpower', {});
      if (!data || data.error) throw new Error((data && data.error) || '导出失败');
      const text = data.matpower_string || '';
      const filename = `${data.name || 'system'}.m`;
      downloadTextFile(filename, text, 'text/x-matlab;charset=utf-8');
      log(`已导出MATPOWER: ${filename}`, 'success');
      showModelIoStatus('MATPOWER 导出完成', [
        ['文件', filename],
        ['格式', 'MATPOWER case v2 (.m)'],
        ['大小', `${text.length} bytes`],
      ], { subtitle: '导出当前系统的 AC steady-state 子集', warnings: data.warnings || [] });
      setStatus('就绪');
    } catch (err) {
      log(`导出MATPOWER失败: ${err.message}`, 'error');
      setStatus('导出失败', 'error');
    }
  }

  async function exportExternalGrid(format) {
    const isGridlabd = format === 'gridlabd';
    const label = isGridlabd ? 'GridLAB-D GLM' : 'OpenDSS DSS';
    const endpoint = isGridlabd ? '/api/session/export_gridlabd' : '/api/session/export_opendss';
    const ext = isGridlabd ? 'glm' : 'dss';
    const mime = isGridlabd ? 'text/x-gridlabd;charset=utf-8' : 'text/x-opendss;charset=utf-8';
    setStatus(`导出${label}中...`, 'busy');
    try {
      const ok = await syncToBackend();
      if (!ok) { setStatus('导出失败', 'error'); return; }
      const data = await apiPost(endpoint, {});
      if (!data || data.error) throw new Error((data && data.error) || '导出失败');
      const text = isGridlabd ? data.glm_string : data.dss_string;
      const filename = `${data.name || 'system'}.${ext}`;
      downloadTextFile(filename, text || '', mime);
      log(`已导出${label}: ${filename}`, 'success');
      showModelIoStatus(`${label} 导出完成`, [
        ['文件', filename],
        ['格式', ext],
        ['大小', `${(text || '').length} bytes`],
      ], { subtitle: '外部仿真工具文本导出', warnings: data.warnings || [] });
      setStatus('就绪');
    } catch (err) {
      log(`导出${label}失败: ${err.message}`, 'error');
      setStatus('导出失败', 'error');
    }
  }

  function importJson(file) {
    const reader = new FileReader();
    reader.onload = async (e) => {
      try {
        const sys = JSON.parse(e.target.result);
        // Send to backend
        const data = await apiPost('/api/session/load_json_string', {
          json_string: JSON.stringify(sys)
        });
        if (data) {
          let loadedName = sys?.name || '';
          if (data._raw_json) {
            // The backend strips unknown fields from the system JSON, so the
            // _canvas layout block (added by Canvas.buildSystemJson) is lost
            // on the round-trip.  Re-attach it from the user's original
            // upload so that loadFromSystemJson can restore positions /
            // connections / viewport exactly instead of running autoLayout.
            const raw = JSON.parse(data._raw_json);
            if (sys && sys._canvas && !raw._canvas) {
              raw._canvas = sys._canvas;
            }
            loadedName = raw?.name || loadedName;
            Canvas.loadFromSystemJson(raw);
          } else {
            Canvas.loadFromSystemJson(sys);
          }
          _canvasDirty = false;  // backend already has the imported system
          updateResilienceSwitchDefault();
          invalidateAnalysisResults('系统已导入，旧潮流和碳流结果已失效');
          log('已导入系统JSON', 'success');
          showModelIoStatus('JSON 导入完成', [
            ['文件', file.name],
            ['系统名', loadedName],
          ], { subtitle: 'JSON 已同步到后端并恢复画布' });
        }
      } catch (err) {
        log(`导入失败: ${err.message}`, 'error');
      }
    };
    reader.readAsText(file);
  }

  // ========== Sync Canvas to Backend ==========
  async function syncToBackend(force = false) {
    // If not forced and the canvas hasn't been modified since the last backend load,
    // skip the roundtrip — the backend already has the correct system.
    if (!force && !_canvasDirty) {
      console.log('[syncToBackend] skipped — canvas not dirty');
      return true;
    }
    const sys = Canvas.buildSystemJson();
    const jsonStr = JSON.stringify(sys);
    if (window.__HACDCPF_DEBUG_SYNC) {
      console.log('[syncToBackend] canvas JSON length:', jsonStr.length);
      console.log('[syncToBackend] DCDC converters:', JSON.stringify(sys.dcdc_converters, null, 2));
      console.log('[syncToBackend] VSC converters:', JSON.stringify(sys.vsc_converters, null, 2));
      console.log('[syncToBackend] DC buses:', JSON.stringify(sys.dc?.buses, null, 2));
    }
    window.__lastSyncJson = jsonStr;  // for debugging in console
    const data = await apiPost('/api/session/load_json_string', { json_string: jsonStr });
    if (!data) {
      log('同步到后端失败', 'error');
      return false;
    }
    _canvasDirty = false;
    invalidateAnalysisResults('网络已同步，旧潮流和碳流结果已失效');
    return true;
  }

  function emissionFactorFromInput(row, defaultValue = 0) {
    if (!row || typeof row !== 'object') return defaultValue;
    const value = row.emission_factor_tco2_mwh ?? row.co2_emission_rate ?? row.emission_factor;
    const num = Number(value);
    return Number.isFinite(num) ? num : defaultValue;
  }

  function emissionFactorProfileFromInput(row, scale = 1.0) {
    if (!row || typeof row !== 'object') return null;
    const values = row.emission_factor_profile_tco2_mwh ??
      row.emission_factor_tco2_mwh_profile ??
      row.co2_emission_rate_profile ??
      row.emission_factor_profile;
    if (!Array.isArray(values)) return null;
    const profile = values
      .map(v => Number(v) * scale)
      .filter(v => Number.isFinite(v));
    return profile.length ? profile : null;
  }

  function carbonFactorScale(unit) {
    const u = String(unit || 'kg/MWh').toLowerCase();
    if (u.includes('kg') && u.includes('mwh')) return 0.001;
    if (u.includes('kg') && u.includes('kwh')) return 1.0;
    return 1.0;
  }

  function carbonFactorDisplay(value) {
    return carbonIntensityDisplay(value);
  }

  function carbonFactorInputValue(value) {
    const num = Number(value);
    return Number.isFinite(num) ? num / 1000 : 0;
  }

  function carbonIntensityUnit() {
    return 'kg/MWh';
  }

  function carbonIntensityDisplay(value) {
    // Backend stores tCO2/MWh; the UI displays kg/MWh.
    const num = Number(value);
    return Number.isFinite(num) ? num * 1000 : 0;
  }

  function setCarbonHint(message, level = 'info') {
    const hint = document.getElementById('carbonFlowHint');
    if (!hint) return;
    hint.textContent = message;
    hint.className = `sub-hint sub-hint-${level}`;
  }

  function carbonPotentialColor(value, maxValue) {
    const t = Math.max(0, Math.min(1, Number(value) / Math.max(Number(maxValue) || 0, 1e-9)));
    if (t <= 0.5) {
      const k = t / 0.5;
      const r = Math.round(46 + (241 - 46) * k);
      const g = Math.round(204 + (196 - 204) * k);
      const b = Math.round(113 + (15 - 113) * k);
      return `rgb(${r},${g},${b})`;
    }
    const k = (t - 0.5) / 0.5;
    const r = Math.round(241 + (231 - 241) * k);
    const g = Math.round(196 + (76 - 196) * k);
    const b = Math.round(15 + (60 - 15) * k);
    return `rgb(${r},${g},${b})`;
  }

  function carbonPlotTheme(title = '') {
    return {
      title,
      paper_bgcolor: 'rgba(0,0,0,0)',
      plot_bgcolor: 'rgba(0,0,0,0)',
      font: { color: '#dcdfe4', size: 11 },
      xaxis: { gridcolor: '#3e4451' },
      yaxis: { gridcolor: '#3e4451' },
      margin: { l: 55, r: 20, t: 38, b: 45 },
      legend: { orientation: 'h', y: -0.25 },
    };
  }

  function sourceColor(sourceId, alpha = 0.78) {
    const palette = [
      [97, 175, 239],
      [152, 195, 121],
      [229, 192, 123],
      [224, 108, 117],
      [198, 120, 221],
      [86, 182, 194],
      [209, 154, 102],
      [171, 178, 191],
    ];
    const c = palette[Math.abs(Number(sourceId) || 0) % palette.length];
    return `rgba(${c[0]},${c[1]},${c[2]},${alpha})`;
  }

  function sourceTypeLabel(type) {
    const labels = {
      generator: '发电机',
      external_grid: '外部电网',
      static_generator: '静态电源',
      renewable_generator: '可再生电源',
      pv_system: '光伏',
      dc_static_generator: 'DC静态电源',
      dc_generator: 'DC电源',
      dc_pv_array: 'DC光伏',
      storage_discharge: '储能放电',
      dc_storage_discharge: 'DC储能放电',
    };
    return labels[type] || type || '源';
  }

  function carbonSourceMeta(data) {
    const map = new Map();
    const rows = Array.isArray(data?.carbon_sources) ? data.carbon_sources : [];
    rows.forEach((row, i) => {
      const id = Number(row.source_id ?? i);
      if (!Number.isFinite(id)) return;
      map.set(id, {
        id,
        label: row.label || `Source ${id}`,
        type: row.source_type || 'source',
        bus: row.bus,
        isDc: row.is_dc === true,
        pMw: numberOr(row.p_mw, 0),
        ef: numberOr(row.emission_factor_tco2_mwh, 0),
      });
    });
    if (!map.size && Array.isArray(data?.sankey_labels)) {
      const maxSource = Math.max(-1, ...(data.sankey_sources || []).map(Number).filter(Number.isFinite));
      for (let i = 0; i <= maxSource; ++i) {
        map.set(i, { id: i, label: data.sankey_labels[i] || `Source ${i}`, type: 'source', ef: 0 });
      }
    }
    return map;
  }

  function contributionEntries(mapLike) {
    if (!mapLike || typeof mapLike !== 'object') return [];
    return Object.entries(mapLike)
      .map(([sid, mw]) => [Number(sid), numberOr(mw, 0)])
      .filter(([sid, mw]) => Number.isFinite(sid) && Number.isFinite(mw) && Math.abs(mw) > 1e-9);
  }

  function renderCarbonSankey(data) {
    const section = document.getElementById('staticCarbonSankeySection');
    const div = document.getElementById('carbonSankeyFlowChart');
    if (!section || !div) return;
    section.style.display = '';
    if (typeof Plotly === 'undefined') {
      div.innerHTML = '<p class="empty-hint">Plotly 未加载，无法展示 Sankey 图。</p>';
      return;
    }

    const metric = document.getElementById('carbonSankeyMetric')?.value || 'power';
    const isCarbon = metric === 'carbon';
    const sourceMap = carbonSourceMeta(data);
    const labels = [];
    const colors = [];
    const nodeIndex = new Map();
    const src = [];
    const tgt = [];
    const val = [];
    const linkColor = [];
    const custom = [];

    const addNode = (key, label, color) => {
      if (nodeIndex.has(key)) return nodeIndex.get(key);
      const idx = labels.length;
      nodeIndex.set(key, idx);
      labels.push(label);
      colors.push(color);
      return idx;
    };
    const addSourceNode = (sid) => {
      const meta = sourceMap.get(Number(sid)) || { id: sid, label: `Source ${sid}`, type: 'source', ef: 0 };
      const label = `${meta.label} (${sourceTypeLabel(meta.type)})`;
      return addNode(`source:${sid}`, label, sourceColor(sid, 0.95));
    };
    const linkValue = (sid, mw) => {
      const ef = sourceMap.get(Number(sid))?.ef || 0;
      return isCarbon ? Math.max(0, mw * ef) : Math.max(0, mw);
    };
    const addFlow = (sid, targetKey, targetLabel, mw, targetColor) => {
      const powerMw = Math.max(0, numberOr(mw, 0));
      if (powerMw <= 1e-9) return;
      const value = linkValue(sid, powerMw);
      const threshold = isCarbon ? 1e-7 : 1e-5;
      if (value <= threshold) return;
      const ef = sourceMap.get(Number(sid))?.ef || 0;
      src.push(addSourceNode(sid));
      tgt.push(addNode(targetKey, targetLabel, targetColor));
      val.push(value);
      linkColor.push(sourceColor(sid, 0.38));
      custom.push([powerMw, powerMw * ef, carbonIntensityDisplay(ef)]);
    };

    const addLoadRows = (rows, isDc) => {
      (rows || []).forEach(row => {
        const prefix = isDc ? 'DC负荷' : 'AC负荷';
        const targetKey = `${isDc ? 'dc' : 'ac'}-load:${row.load_index}:${row.bus}:${carbonLoadLabel(row)}`;
        const targetLabel = `${prefix} ${carbonLoadLabel(row)} @ Bus ${row.bus}`;
        contributionEntries(row.generator_supply_mw).forEach(([sid, mw]) => {
          addFlow(sid, targetKey, targetLabel, mw, isDc ? 'rgba(86,182,194,0.95)' : 'rgba(152,195,121,0.95)');
        });
      });
    };
    addLoadRows(data.load_carbon || [], false);
    addLoadRows(data.dc_load_carbon || [], true);

    (data.storage_carbon || []).forEach(row => {
      if (numberOr(row.p_mw, 0) >= 0 && !Object.keys(row.source_supply_mw || {}).length) return;
      const targetKey = `storage-charge:${row.is_dc ? 'dc' : 'ac'}:${row.storage_index}:${row.bus}`;
      const targetLabel = `${row.is_dc ? 'DC' : 'AC'}储能充电 ${row.storage_index} @ Bus ${row.bus}`;
      contributionEntries(row.source_supply_mw).forEach(([sid, mw]) => {
        addFlow(sid, targetKey, targetLabel, mw, 'rgba(198,120,221,0.95)');
      });
    });

    const addLossCategory = (label, key, rows, mapField, color) => {
      const sum = new Map();
      (rows || []).forEach(row => {
        contributionEntries(row[mapField]).forEach(([sid, mw]) => {
          sum.set(sid, (sum.get(sid) || 0) + mw);
        });
      });
      sum.forEach((mw, sid) => addFlow(sid, key, label, mw, color));
    };
    addLossCategory('AC支路损耗', 'loss:ac-branch', data.branch_carbon, 'generator_loss_mw', 'rgba(224,108,117,0.95)');
    addLossCategory('DC支路损耗', 'loss:dc-branch', data.dc_branch_carbon, 'generator_loss_mw', 'rgba(224,108,117,0.86)');
    addLossCategory('VSC换流损耗', 'loss:vsc', data.vsc_carbon, 'generator_loss_mw', 'rgba(209,154,102,0.95)');
    addLossCategory('DC/DC损耗', 'loss:dcdc', data.dcdc_carbon, 'generator_loss_mw', 'rgba(209,154,102,0.86)');
    addLossCategory('能量路由器损耗', 'loss:energy-router', data.energy_router_carbon, 'source_loss_mw', 'rgba(209,154,102,0.76)');

    if (!src.length && Array.isArray(data.sankey_sources) && Array.isArray(data.sankey_targets)) {
      (data.sankey_labels || []).forEach((label, i) => addNode(`legacy:${i}`, label, i < sourceMap.size ? sourceColor(i, 0.95) : 'rgba(152,195,121,0.95)'));
      (data.sankey_values || []).forEach((mw, i) => {
        const powerMw = numberOr(mw, 0);
        if (powerMw <= 1e-9) return;
        const s = Number(data.sankey_sources[i]);
        const t = Number(data.sankey_targets[i]);
        const value = isCarbon ? 0 : powerMw;
        if (value <= 0) return;
        src.push(s); tgt.push(t); val.push(value);
        linkColor.push(sourceColor(s, 0.38));
        custom.push([powerMw, 0, 0]);
      });
    }

    if (!src.length) {
      div.innerHTML = '<p class="empty-hint">暂无可用于 Sankey 的源荷追踪数据。</p>';
      return;
    }

    const unit = isCarbon ? 'tCO2' : 'MW';
    Plotly.newPlot(div, [{
      type: 'sankey',
      arrangement: 'snap',
      node: {
        pad: 16,
        thickness: 16,
        line: { color: 'rgba(255,255,255,0.22)', width: 0.5 },
        label: labels,
        color: colors,
      },
      link: {
        source: src,
        target: tgt,
        value: val,
        color: linkColor,
        customdata: custom,
        hovertemplate:
          `%{source.label} → %{target.label}<br>` +
          `显示值: %{value:.4f} ${unit}<br>` +
          `电力: %{customdata[0]:.4f} MW<br>` +
          `碳排: %{customdata[1]:.6f} tCO2<br>` +
          `源碳因子: %{customdata[2]:.2f} ${carbonIntensityUnit()}<extra></extra>`,
      },
    }], {
      ...carbonPlotTheme(isCarbon ? '源-荷碳流追踪' : '源-荷电力追踪'),
      margin: { l: 12, r: 12, t: 34, b: 12 },
      font: { color: '#dcdfe4', size: 11 },
    }, { responsive: true, displaylogo: false });
  }

  function componentBusIndex(compId) {
    const busMap = Canvas.getCompBusMap().ac || {};
    for (const conn of Canvas.state.connections || []) {
      let otherId = null;
      if (conn.from.compId === compId) otherId = conn.to.compId;
      if (conn.to.compId === compId) otherId = conn.from.compId;
      if (otherId === null) continue;
      for (const [bus, id] of Object.entries(busMap)) {
        if (Number(id) === otherId) return Number(bus);
      }
    }
    return null;
  }

  function normalizeCarbonFactorRows(input, key) {
    const rows = input?.[key];
    if (Array.isArray(rows)) return rows;
    if (rows && typeof rows === 'object') {
      return Object.entries(rows).map(([index, value]) => (
        value && typeof value === 'object'
          ? { index: Number(index), ...value }
          : { index: Number(index), emission_factor: value }
      ));
    }
    return [];
  }

  function carbonIndexBase(input) {
    if (Number(input?.index_base) === 1) return 1;
    const numbering = String(input?.numbering || '').toLowerCase();
    return numbering.includes('one') || numbering.includes('1') ? 1 : 0;
  }

  function applyRowsToComponents(rows, type, scale, indexBase = 0) {
    const components = Canvas.state.components.filter(c => c.type === type);
    let updated = 0;
    rows.forEach(row => {
      if (!row || typeof row !== 'object') return;
      const ef = emissionFactorFromInput(row, NaN) * scale;
      if (!Number.isFinite(ef)) return;
      const profile = emissionFactorProfileFromInput(row, scale);
      components.forEach((comp, pos) => {
        const p = comp.params || {};
        const rowIndex = Number(row.index);
        const normalizedIndex = Number.isFinite(rowIndex) ? rowIndex - indexBase : NaN;
        const componentIndex = Number(p.index);
        const rowBus = Number(row.bus);
        const bus = componentBusIndex(comp.id);
        const hasIndex = Number.isFinite(normalizedIndex);
        const hasName = Boolean(row.name);
        const hasBus = Number.isFinite(rowBus);
        const indexMatches = hasIndex && (
          normalizedIndex === pos ||
          (Number.isFinite(componentIndex) && normalizedIndex === componentIndex) ||
          (Number.isFinite(componentIndex) && rowIndex === componentIndex)
        );
        const nameMatches = hasName && p.name && String(row.name) === String(p.name);
        const busMatches = hasBus && bus === rowBus;
        if (!indexMatches && !nameMatches && !busMatches) return;
        p.emission_factor_tco2_mwh = ef;
        if (profile) p.emission_factor_profile_tco2_mwh = profile;
        else delete p.emission_factor_profile_tco2_mwh;
        if (type === 'static_generator') p.co2_emission_rate = ef;
        updated++;
      });
    });
    return updated;
  }

  function collectCarbonFactorsFromCanvas() {
    const factors = { generators: [], static_generators: [], external_grids: [] };
    const maps = Canvas.getCompBusMap();
    Canvas.state.components.forEach((comp) => {
      const p = comp.params || {};
      if (comp.type === 'generator') {
        const entries = Object.entries(maps.gen || {});
        const pos = entries.findIndex(([, id]) => Number(id) === comp.id);
        factors.generators.push({
          index: pos >= 0 ? pos : factors.generators.length,
          bus: componentBusIndex(comp.id) || Number(p.bus) || 0,
          name: p.name || '',
          emission_factor_tco2_mwh: Number(p.emission_factor_tco2_mwh || p.co2_emission_rate || 0),
        });
      } else if (comp.type === 'static_generator') {
        const entries = Object.entries(maps.sgen || {});
        const pos = entries.findIndex(([, id]) => Number(id) === comp.id);
        factors.static_generators.push({
          index: pos >= 0 ? pos : factors.static_generators.length,
          bus: componentBusIndex(comp.id) || Number(p.bus) || 0,
          name: p.name || '',
          emission_factor_tco2_mwh: Number(p.emission_factor_tco2_mwh || p.co2_emission_rate || 0),
        });
      } else if (comp.type === 'external_grid') {
        const entries = Object.entries(maps.extGrid || {});
        const pos = entries.findIndex(([, id]) => Number(id) === comp.id);
        factors.external_grids.push({
          index: pos >= 0 ? pos : factors.external_grids.length,
          bus: componentBusIndex(comp.id) || Number(p.bus) || 0,
          name: p.name || '',
          emission_factor_tco2_mwh: Number(p.emission_factor_tco2_mwh || p.co2_emission_rate || 0),
        });
        if (Array.isArray(p.emission_factor_profile_tco2_mwh)) {
          factors.external_grids[factors.external_grids.length - 1].emission_factor_profile_tco2_mwh =
            p.emission_factor_profile_tco2_mwh.map(Number).filter(Number.isFinite);
        }
      }
    });
    return factors;
  }

  async function syncCarbonFactorsOnly() {
    const factors = collectCarbonFactorsFromCanvas();
    const data = await apiPost('/api/session/update_carbon_factors', factors);
    if (!data) return false;
    log(`碳排放因子已同步：发电机 ${data.updated_generators || 0}，外部电网 ${data.updated_external_grids || 0}`, 'success');
    return true;
  }

  async function importCarbonFactorsJson(file) {
    try {
      const wasDirty = _canvasDirty;
      const text = await file.text();
      const input = JSON.parse(text);
      const scale = carbonFactorScale(input.unit || input.units);
      const indexBase = carbonIndexBase(input);
      const genRows = normalizeCarbonFactorRows(input, 'generators');
      const sgenRows = normalizeCarbonFactorRows(input, 'static_generators');
      const gridRows = normalizeCarbonFactorRows(input, 'external_grids');
      const nGen = applyRowsToComponents(genRows, 'generator', scale, indexBase);
      const nSgen = applyRowsToComponents(sgenRows, 'static_generator', scale, indexBase);
      const nGrid = applyRowsToComponents(gridRows, 'external_grid', scale, indexBase);
      if (nGen + nSgen + nGrid === 0) {
        log('碳排放因子 JSON 未匹配到当前画布元件，请检查 index、bus 或 name', 'warn');
        return;
      }
      _canvasDirty = wasDirty;
      updateTopologyTables();
      if (Canvas.state.selectedId !== null) onSelectionChanged(Canvas.state.selectedId);
      await syncCarbonFactorsOnly();
      log(`已导入碳排放因子 JSON：发电机 ${nGen}，静态电源 ${nSgen}，外部电网 ${nGrid}`, 'success');
    } catch (e) {
      log(`导入碳排放因子失败：${e.message || e}`, 'error');
    }
  }

  function showCarbonResults(data, dynamic = false) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('carbonFlow');
    document.getElementById('carbonSummaryTitle').textContent = '静态碳流分析结果 — 概览';
    document.getElementById('staticCarbonSankeySection').style.display = '';
    document.getElementById('staticCarbonBusSection').style.display = '';
    document.getElementById('staticCarbonLoadSection').style.display = '';
    document.getElementById('staticCarbonBranchSection').style.display = '';

    const summary = data.matrix_summary || data.tracing_summary || {};
    const balance = Number(summary.balance_error_pct || 0);
    const balanceTco2 = Number(summary.balance_error_tco2 || (
      (summary.total_generation_emissions_tco2 || 0) -
      (summary.total_load_emissions_tco2 || 0) -
      (summary.total_loss_emissions_tco2 || 0)
    ));
    document.getElementById('carbonSummary').innerHTML = `
      <div class="result-item"><span class="result-label">来源</span>
        <span class="result-value">${escapeHtml(data.pf_source || (dynamic ? 'time_series_pf' : 'last_pf'))}</span></div>
      <div class="result-item"><span class="result-label">矩阵法求解</span>
        <span class="result-value ${data.matrix_solved === false ? 'result-failed' : 'result-converged'}">${data.matrix_solved === false ? '否' : '是'}</span></div>
      <div class="result-item"><span class="result-label">源端碳排(tCO2)</span>
        <span class="result-value">${Number(summary.total_generation_emissions_tco2 || 0).toFixed(4)}</span></div>
      <div class="result-item"><span class="result-label">负荷碳排(tCO2)</span>
        <span class="result-value">${Number(summary.total_load_emissions_tco2 || 0).toFixed(4)}</span></div>
      <div class="result-item"><span class="result-label">损耗碳排(tCO2)</span>
        <span class="result-value">${Number(summary.total_loss_emissions_tco2 || 0).toFixed(4)}</span></div>
      <div class="result-item"><span class="result-label">守恒误差</span>
        <span class="result-value ${Math.abs(balance) < 1e-3 ? 'result-converged' : 'result-failed'}">${balanceTco2.toExponential(3)} tCO2 / ${balance.toExponential(3)}%</span></div>
    `;

    const busRows = [
      ...(data.bus_carbon || []).map(b => ({ ...b, is_dc: false })),
      ...(data.dc_bus_carbon || []).map(b => ({ ...b, is_dc: true })),
    ];
    renderCarbonSankey(data);
    renderCarbonPotentialChart(busRows);
    Canvas.clearCarbonPotentialResults?.();

    // Make result rows pan/select the related bus component on the canvas
    // (mirroring the power-flow tables). DC-aware: DC rows prefer the DC bus map.
    const cbMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;
    const cbClick = (busId, isDc) => {
      const id = parseInt(busId, 10);
      if (!Number.isInteger(id) || !cbMap) return '';
      const prim = isDc ? cbMap.dc : cbMap.ac;
      const alt = isDc ? cbMap.ac : cbMap.dc;
      const compId = (prim && prim[id] != null) ? prim[id]
                   : (alt && alt[id] != null) ? alt[id] : undefined;
      return compId != null
        ? ` class="topo-clickable" data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
    };

    let busHtml = `<table><thead><tr><th>类型</th><th>Bus</th><th>碳势/碳强度(${carbonIntensityUnit()})</th><th>有效消纳(MW)</th><th>状态</th></tr></thead><tbody>`;
    busRows.forEach(b => {
      const potentialValid = b.carbon_potential_valid !== false;
      const hasSink = Number(b.sink_power_mw || 0) > 1e-9;
      const status = !potentialValid
        ? '碳势病态，图表排除'
        : (hasSink ? '有本地消纳' : '无本地消纳/电源或过境节点');
      const statusClass = potentialValid ? 'result-converged' : 'result-failed';
      busHtml += `<tr${cbClick(b.bus_index, b.is_dc)}><td>${b.is_dc ? 'DC' : 'AC'}</td><td>${b.bus_index}</td><td>${carbonIntensityDisplay(b.carbon_intensity_tco2_mwh).toFixed(3)}</td><td>${Number(b.sink_power_mw || 0).toFixed(6)}</td><td class="${statusClass}">${status}</td></tr>`;
    });
    busHtml += busRows.length ? '</tbody></table>' : '<tr><td colspan="5">暂无节点碳强度结果</td></tr></tbody></table>';
    document.getElementById('carbonBusResults').innerHTML = busHtml;

    const loadRows = [
      ...(data.load_carbon || []).map(l => ({ ...l, is_dc: false })),
      ...(data.dc_load_carbon || []).map(l => ({ ...l, is_dc: true })),
    ];
    let loadHtml = `<table><thead><tr><th>类型</th><th>Load</th><th>Bus</th><th>P(MW)</th><th>碳强度(${carbonIntensityUnit()})</th><th>碳排(tCO2)</th></tr></thead><tbody>`;
    loadRows.forEach(l => {
      const demand = positiveDemandMw(l);
      const emissions = Math.max(0, Number(l.total_emissions_tco2 || 0));
      loadHtml += `<tr${cbClick(l.bus, l.is_dc)}><td>${l.is_dc ? 'DC' : 'AC'}</td><td>${escapeHtml(carbonLoadLabel(l))}</td><td>${l.bus}</td><td>${demand.toFixed(4)}</td><td>${carbonIntensityDisplay(l.carbon_intensity_tco2_mwh).toFixed(3)}</td><td>${emissions.toFixed(6)}</td></tr>`;
    });
    loadHtml += loadRows.length ? '</tbody></table>' : '<tr><td colspan="6">暂无负荷碳排放结果</td></tr></tbody></table>';
    document.getElementById('carbonLoadResults').innerHTML = loadHtml;

    const branchRows = [
      ...(data.branch_carbon || []).map(b => ({ ...b, is_dc: false })),
      ...(data.dc_branch_carbon || []).map(b => ({ ...b, is_dc: true })),
    ];
    let branchHtml = `<table><thead><tr><th>类型</th><th>Branch</th><th>From</th><th>To</th><th>Loss(MW)</th><th>碳强度(${carbonIntensityUnit()})</th><th>碳排(tCO2)</th></tr></thead><tbody>`;
    branchRows.forEach(b => {
      branchHtml += `<tr${cbClick(b.from_bus, b.is_dc)}><td>${b.is_dc ? 'DC' : 'AC'}</td><td>${b.branch_index}</td><td>${b.from_bus}</td><td>${b.to_bus}</td><td>${Math.max(0, Number(b.loss_mw || 0)).toFixed(6)}</td><td>${carbonIntensityDisplay(b.carbon_intensity_tco2_mwh).toFixed(3)}</td><td>${Math.max(0, Number(b.total_emissions_tco2 || 0)).toFixed(6)}</td></tr>`;
    });
    branchHtml += branchRows.length ? '</tbody></table>' : '<tr><td colspan="7">暂无支路损耗碳排放结果</td></tr></tbody></table>';
    document.getElementById('carbonBranchResults').innerHTML = branchHtml;

    const storageSec = document.getElementById('carbonStorageSection');
    const storageRows = data.storage_carbon || data.terminal_storage_states || [];
    if (storageRows.length) {
      storageSec.style.display = '';
      let html = `<table><thead><tr><th>类型</th><th>Storage</th><th>Bus</th><th>P(MW)</th><th>SOC</th><th>库存碳强度(${carbonIntensityUnit()})</th><th>本时段碳排(tCO2)</th></tr></thead><tbody>`;
      storageRows.forEach(s => {
        const row = {
          ...s,
          canvas_type: s.canvas_type || (s.is_dc ? 'dc_storage' : 'storage'),
          canvas_index: s.canvas_index ?? s.storage_index,
        };
        const attr = compClickAttr(rowCanvasCompId(row, cbMap));
        const pText = Number.isFinite(Number(s.p_mw)) ? Number(s.p_mw).toFixed(4) : '—';
        html += `<tr${attr}><td>${s.is_dc ? 'DC' : 'AC'}</td><td>${s.storage_index}</td><td>${s.bus}</td><td>${pText}</td><td>${Number(s.soc || 0).toFixed(4)}</td><td>${carbonIntensityDisplay(s.soc_carbon_intensity_tco2_mwh).toFixed(3)}</td><td>${Math.max(0, Number(s.total_emissions_tco2 || 0)).toFixed(6)}</td></tr>`;
      });
      html += '</tbody></table>';
      document.getElementById('carbonStorageResults').innerHTML = html;
    } else {
      storageSec.style.display = 'none';
      document.getElementById('carbonStorageResults').innerHTML = '';
    }

    document.getElementById('dynamicCarbonSection').style.display = 'none';
    document.getElementById('dynamicCarbonResults').innerHTML = '';
    switchTab('results');
  }

  function renderCarbonPotentialChart(busRows) {
    const div = document.getElementById('carbonPotentialChart');
    if (!div) return;
    const validRows = (busRows || []).filter(b =>
      b.carbon_potential_valid !== false &&
      Number.isFinite(Number(b.carbon_intensity_tco2_mwh)));
    if (!validRows.length || typeof Plotly === 'undefined') {
      div.innerHTML = '<p class="empty-hint">暂无节点碳势图数据</p>';
      return;
    }
    const rows = [...validRows].sort((a, b) =>
      Number(b.carbon_intensity_tco2_mwh || 0) - Number(a.carbon_intensity_tco2_mwh || 0));
    const values = rows.map(b => carbonIntensityDisplay(b.carbon_intensity_tco2_mwh));
    const maxValue = values.reduce((mx, v) => Math.max(mx, v), 0);
    Plotly.newPlot(div, [{
      x: rows.map(b => `${b.is_dc ? 'DC' : 'AC'}-${b.bus_index}`),
      y: values,
      type: 'bar',
      marker: {
        color: values.map(v => carbonPotentialColor(v, maxValue)),
      },
      hovertemplate: `%{x}<br>碳势 %{y:.3f} ${carbonIntensityUnit()}<extra></extra>`,
    }], {
      title: '节点碳势图',
      margin: { l: 55, r: 35, t: 35, b: 70 },
      xaxis: { title: '节点', tickangle: -45 },
      yaxis: { title: carbonIntensityUnit(), rangemode: 'tozero' },
      paper_bgcolor: 'rgba(0,0,0,0)',
      plot_bgcolor: 'rgba(0,0,0,0)',
      font: { color: '#dcdfe4' },
    }, { responsive: true, displaylogo: false });
  }

  async function runCarbonFlow() {
    if (!_lastPfData) {
      const msg = '请先运行潮流计算，并确保潮流收敛后再进行静态碳流分析。';
      setCarbonHint(msg, 'warn');
      log(msg, 'warn');
      setStatus('需先运行潮流', 'error');
      return;
    }
    if (!_lastPfData.converged) {
      const msg = '最近一次潮流计算未收敛，无法进行静态碳流分析。';
      setCarbonHint(msg, 'warn');
      log(msg, 'warn');
      setStatus('潮流未收敛', 'error');
      return;
    }
    if (_canvasDirty) {
      const msg = '当前网络已修改，最近一次潮流结果已失效。请重新运行潮流计算。';
      setCarbonHint(msg, 'warn');
      log(msg, 'warn');
      setStatus('需重跑潮流', 'error');
      return;
    }
    setStatus('碳流分析中...', 'busy');
    await syncCarbonFactorsOnly();
    const data = await apiPost('/api/session/run_carbon', {});
    if (!data) {
      setStatus('碳流分析失败', 'error');
      return;
    }
    _lastCarbonData = data;
    showCarbonResults(data);
    const summary = data.matrix_summary || data.tracing_summary || {};
    log(`碳流分析完成：负荷 ${Number(summary.total_load_emissions_tco2 || 0).toFixed(4)} tCO2，损耗 ${Number(summary.total_loss_emissions_tco2 || 0).toFixed(4)} tCO2`, 'success');
    setCarbonHint('静态碳流已基于最近一次收敛潮流结果完成。', 'ok');
    setStatus('碳流分析完成');
  }

  async function runDynamicCarbonFlow() {
    if (!_lastTspfData) {
      const msg = '请先在“时序潮流”模块运行时序潮流或时序OPF，再进行动态碳流分析。';
      setCarbonHint(msg, 'warn');
      log(msg, 'warn');
      setStatus('需先运行时序潮流', 'error');
      return;
    }
    if (Number(_lastTspfData.num_converged || 0) <= 0) {
      const msg = '最近一次时序潮流没有收敛时段，无法进行动态碳流分析。';
      setCarbonHint(msg, 'warn');
      log(msg, 'warn');
      setStatus('时序潮流未收敛', 'error');
      return;
    }
    if (_canvasDirty) {
      const msg = '当前网络已修改，最近一次时序潮流结果已失效。请重新运行时序潮流/OPF。';
      setCarbonHint(msg, 'warn');
      log(msg, 'warn');
      setStatus('需重跑时序潮流', 'error');
      return;
    }
    setStatus('动态碳流分析中...', 'busy');
    await syncCarbonFactorsOnly();
    const data = await apiPost('/api/session/run_dynamic_carbon', {
      use_last_tspf: true,
    });
    if (!data) {
      setStatus('动态碳流分析失败', 'error');
      return;
    }
    _lastDynamicCarbonData = data;
    showDynamicCarbonResults(data);
    log(`动态碳流分析完成：${data.num_pf_converged}/${data.num_steps} 时段收敛，总负荷碳排 ${Number(data.total_load_emissions_tco2 || 0).toFixed(4)} tCO2`, 'success');
    setCarbonHint(`动态碳流已基于最近一次${data.run_opf ? '时序OPF' : '时序潮流'}结果完成。`, 'ok');
    setStatus('动态碳流分析完成');
  }

  function buildDynamicBusRoleMap() {
    const roleMap = new Map();
    const add = (isDc, bus, role) => {
      const key = `${isDc ? 'dc' : 'ac'}:${Number(bus)}`;
      if (!Number.isFinite(Number(bus))) return;
      if (!roleMap.has(key)) roleMap.set(key, new Set());
      roleMap.get(key).add(role);
    };

    try {
      const sys = Canvas.buildSystemJson();
      (sys.ac?.generators || []).forEach(g => add(false, g.bus, '电源'));
      (sys.ac?.static_generators || []).forEach(g => add(false, g.bus, '电源'));
      (sys.ac?.renewable_gens || []).forEach(g => add(false, g.bus, '电源'));
      (sys.ac?.pv_systems || []).forEach(g => add(false, g.bus, '电源'));
      (sys.ac?.external_grids || []).forEach(g => add(false, g.bus, '外部电网'));
      (sys.dc?.static_generators || []).forEach(g => add(true, g.bus, '电源'));
      (sys.dc?.pv_arrays || []).forEach(g => add(true, g.bus, '电源'));

      (sys.ac?.loads || []).forEach(l => add(false, l.bus, '负荷'));
      (sys.ac?.flexible_loads || []).forEach(l => add(false, l.bus, '负荷'));
      (sys.ac?.asymmetric_loads || []).forEach(l => add(false, l.bus, '负荷'));
      (sys.ac?.chargers || []).forEach(l => add(false, l.bus, '负荷'));
      (sys.ac?.charging_stations || []).forEach(l => add(false, l.bus, '负荷'));
      (sys.dc?.loads || []).forEach(l => add(true, l.bus, '负荷'));

      (sys.ac?.storage || []).forEach(s => add(false, s.bus, '储能'));
      (sys.dc?.dc_storage || []).forEach(s => add(true, s.bus, '储能'));
      (sys.dc?.storage || []).forEach(s => add(true, s.bus, '储能'));
      (sys.mobile_storage || []).forEach(s => add(false, s.bus, '移动储能'));
    } catch (_) {
      // Canvas data is only used for labeling. Dynamic result rendering can continue without it.
    }
    return roleMap;
  }

  function dynamicBusLabel(row, roleMap = buildDynamicBusRoleMap()) {
    const isDc = !!row?.is_dc;
    const bus = Number(row?.bus_index);
    const roles = [...(roleMap.get(`${isDc ? 'dc' : 'ac'}:${bus}`) || new Set())];
    if (!Number(row?.energy_mwh || 0)) roles.push('零消纳');
    return `${isDc ? 'DC' : 'AC'}-${bus}${roles.length ? ` ${roles.join('/')}` : ' 节点'}`;
  }

  function numericSeries(values, scale = 1) {
    return (values || []).map(v => {
      const num = Number(v);
      return Number.isFinite(num) ? num * scale : null;
    });
  }

  function dynamicBalanceSummary(data) {
    const gen = Number(data.total_generation_emissions_tco2 || 0);
    const load = Number(data.total_load_emissions_tco2 || 0);
    const loss = Number(data.total_loss_emissions_tco2 || 0);
    const charge = Number(data.total_storage_charge_emissions_tco2 || 0);
    const discharge = Number(data.total_storage_discharge_emissions_tco2 || 0);
    const basic = gen - load - loss;
    const storageAware = gen - load - loss - charge + discharge;
    return {
      gen,
      load,
      loss,
      charge,
      discharge,
      basic,
      storageDelta: charge - discharge,
      storageAware,
      basicPct: Math.abs(basic) / Math.max(Math.abs(gen), 1e-12) * 100,
      storageAwarePct: Math.abs(storageAware) / Math.max(Math.abs(gen), 1e-12) * 100,
    };
  }

  function storageDispatchSourceLabel(data) {
    if (data.run_opf) return `OPF (${Number(data.num_opf_converged || 0)}/${Number(data.num_steps || 0)} 时段收敛)`;
    if (data.skip_uc === false && data.uc_feasible === true) return `UC/SCUC (${data.uc_solver_name || 'solver'})`;
    return '固定出力/简化策略（非OPF优化）';
  }

  function renderDynamicCarbonCharts(data) {
    const root = document.getElementById('dynamicCarbonCharts');
    if (!root) return;
    if (typeof Plotly === 'undefined') {
      root.innerHTML = '<p class="empty-hint">Plotly 未加载，无法展示动态图表。</p>';
      return;
    }

    const steps = data.step_results || [];
    const hrs = steps.map(s => Number(s.step));
    const storageCharge = Number(data.total_storage_charge_emissions_tco2 || 0);
    const loadTotal = Number(data.total_load_emissions_tco2 || 0);
    const loadOnly = Math.max(loadTotal - storageCharge, 0);
    const pieValues = [
      Math.max(Number(data.total_generation_emissions_tco2 || 0), 0),
      loadOnly,
      storageCharge,
      Math.max(Number(data.total_loss_emissions_tco2 || 0), 0),
    ];

    Plotly.newPlot('dynamicCarbonMixChart', [{
      type: 'pie',
      labels: ['源端供给', '负荷消纳', '储能充电', '网络损耗'],
      values: pieValues,
      hole: 0.42,
      marker: { colors: ['#4c78a8', '#59a14f', '#f2cf5b', '#e15759'] },
      textinfo: 'label+percent',
      hovertemplate: '%{label}<br>%{value:.4f} tCO2<extra></extra>',
    }], {
      ...carbonPlotTheme('源荷储损耗碳排统计'),
      showlegend: true,
      margin: { l: 10, r: 10, t: 38, b: 10 },
    }, { responsive: true, displaylogo: false });

    const chargeSeries = numericSeries(steps.map(s => s.total_storage_charge_emissions_tco2));
    const dischargeSeries = numericSeries(steps.map(s => s.total_storage_discharge_emissions_tco2));
    Plotly.newPlot('dynamicCarbonTrendChart', [
      {
        x: hrs,
        y: numericSeries(steps.map(s => s.total_generation_emissions_tco2)),
        mode: 'lines+markers',
        name: '源端',
        line: { color: '#4c78a8', width: 2 },
      },
      {
        x: hrs,
        y: numericSeries(steps.map(s => Math.max(Number(s.total_load_emissions_tco2 || 0) - Number(s.total_storage_charge_emissions_tco2 || 0), 0))),
        mode: 'lines+markers',
        name: '负荷',
        line: { color: '#59a14f', width: 2 },
      },
      {
        x: hrs,
        y: chargeSeries,
        mode: 'lines+markers',
        name: '储能充电',
        line: { color: '#f2cf5b', width: 2 },
      },
      {
        x: hrs,
        y: dischargeSeries,
        mode: 'lines+markers',
        name: '储能放电',
        line: { color: '#9c755f', width: 2, dash: 'dot' },
      },
      {
        x: hrs,
        y: numericSeries(steps.map(s => s.total_loss_emissions_tco2)),
        mode: 'lines+markers',
        name: '损耗',
        line: { color: '#e15759', width: 2 },
      },
    ], {
      ...carbonPlotTheme('动态碳排趋势'),
      xaxis: { gridcolor: '#3e4451', title: '时段' },
      yaxis: { gridcolor: '#3e4451', title: 'tCO2', rangemode: 'tozero' },
    }, { responsive: true, displaylogo: false });

    renderDynamicBusSelector(data);
  }

  function renderDynamicBusSelector(data) {
    const selector = document.getElementById('dynamicCarbonBusSelector');
    const addBtn = document.getElementById('btnAddDynamicCarbonBus');
    const selectedDiv = document.getElementById('dynamicCarbonSelectedBuses');
    if (!selector || !addBtn || !selectedDiv) return;

    const roleMap = buildDynamicBusRoleMap();
    const rows = data.bus_stats || [];
    if (!rows.length || !(data.hourly_bus_intensity_tco2_mwh || []).length) {
      selector.innerHTML = '';
      selectedDiv.innerHTML = '<span class="empty-hint">暂无节点碳势时序数据。</span>';
      return;
    }

    selector.innerHTML = rows.map((row, idx) =>
      `<option value="${idx}">${escapeHtml(dynamicBusLabel(row, roleMap))}</option>`).join('');

    const selected = new Set();
    const redraw = () => {
      const traces = [...selected].map(idx => {
        const row = rows[idx];
        const y = (data.hourly_bus_intensity_tco2_mwh || []).map(hourRow =>
          carbonIntensityDisplay(Array.isArray(hourRow) ? hourRow[idx] : NaN));
        return {
          x: y.map((_, t) => t),
          y,
          mode: 'lines+markers',
          name: dynamicBusLabel(row, roleMap),
          line: { width: 2 },
        };
      });

      if (!traces.length) {
        Plotly.newPlot('dynamicCarbonBusTrendChart', [], {
          ...carbonPlotTheme('节点碳势时段变化'),
          xaxis: { gridcolor: '#3e4451', title: '时段' },
          yaxis: { gridcolor: '#3e4451', title: carbonIntensityUnit(), rangemode: 'tozero' },
          annotations: [{
            text: '请选择节点后点击添加',
            xref: 'paper',
            yref: 'paper',
            x: 0.5,
            y: 0.5,
            showarrow: false,
            font: { color: '#abb2bf' },
          }],
        }, { responsive: true, displaylogo: false });
      } else {
        Plotly.newPlot('dynamicCarbonBusTrendChart', traces, {
          ...carbonPlotTheme('节点碳势时段变化'),
          xaxis: { gridcolor: '#3e4451', title: '时段' },
          yaxis: { gridcolor: '#3e4451', title: carbonIntensityUnit(), rangemode: 'tozero' },
        }, { responsive: true, displaylogo: false });
      }

      selectedDiv.innerHTML = [...selected].map(idx =>
        `<button type="button" class="dynamic-carbon-chip" data-bus-idx="${idx}">${escapeHtml(dynamicBusLabel(rows[idx], roleMap))} ×</button>`
      ).join('');
      selectedDiv.querySelectorAll('[data-bus-idx]').forEach(btn => {
        btn.addEventListener('click', () => {
          selected.delete(Number(btn.dataset.busIdx));
          redraw();
        });
      });
    };

    addBtn.onclick = () => {
      const idx = Number(selector.value);
      if (Number.isInteger(idx) && idx >= 0 && idx < rows.length) {
        selected.add(idx);
        redraw();
      }
    };

    const defaultIdx = rows.findIndex(row => Number(row.energy_mwh || 0) > 1e-9);
    if (rows.length) selected.add(defaultIdx >= 0 ? defaultIdx : 0);
    redraw();
  }

  function showDynamicCarbonResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('carbonFlow');
    switchTab('results');
    document.getElementById('carbonSummaryTitle').textContent = '动态碳流分析结果 — 概览';
    document.getElementById('staticCarbonSankeySection').style.display = 'none';
    document.getElementById('carbonSankeyFlowChart').innerHTML = '';
    document.getElementById('staticCarbonBusSection').style.display = 'none';
    document.getElementById('staticCarbonLoadSection').style.display = 'none';
    document.getElementById('staticCarbonBranchSection').style.display = 'none';

    const balance = dynamicBalanceSummary(data);
    const totalHours = Number(data.num_steps || 0) * Number(data.step_duration_hr || 1);
    const sourceLabel = data.result_source === 'last_tspf'
      ? (data.run_opf ? '最近一次时序OPF结果' : '最近一次时序潮流结果')
      : '动态碳流内部重新计算结果';
    const profileMsg = Number(data.external_grid_carbon_profiles || 0) > 0
      ? `${Number(data.external_grid_carbon_profile_applications || 0)} 次应用`
      : '未使用分时外部电网碳因子';

    document.getElementById('carbonSummary').innerHTML = `
      <div class="result-item"><span class="result-label">数据来源</span>
        <span class="result-value">${escapeHtml(sourceLabel)}</span></div>
      <div class="result-item"><span class="result-label">时段/步长</span>
        <span class="result-value">${Number(data.num_steps || 0)} × ${Number(data.step_duration_hr || 1).toFixed(2)} h = ${totalHours.toFixed(2)} h</span></div>
      <div class="result-item"><span class="result-label">潮流收敛</span>
        <span class="result-value ${Number(data.num_pf_converged || 0) === Number(data.num_steps || 0) ? 'result-converged' : 'result-failed'}">${Number(data.num_pf_converged || 0)}/${Number(data.num_steps || 0)}</span></div>
      <div class="result-item"><span class="result-label">OPF收敛</span>
        <span class="result-value">${Number(data.num_opf_converged || 0)}/${Number(data.num_steps || 0)}</span></div>
      <div class="result-item"><span class="result-label">储能调度来源</span>
        <span class="result-value">${escapeHtml(storageDispatchSourceLabel(data))}</span></div>
      <div class="result-item"><span class="result-label">外部电网分时因子</span>
        <span class="result-value">${escapeHtml(profileMsg)}</span></div>
      <div class="result-item"><span class="result-label">累计碳平衡残差</span>
        <span class="result-value ${Math.abs(balance.basicPct) < 1e-3 ? 'result-converged' : 'result-failed'}">${balance.basic.toExponential(3)} t / ${balance.basicPct.toExponential(3)}%</span></div>
      <div class="result-item"><span class="result-label">储能库存净变化</span>
        <span class="result-value">${balance.storageDelta.toFixed(6)} t</span></div>
    `;
    document.getElementById('carbonBusResults').innerHTML = '';
    document.getElementById('carbonLoadResults').innerHTML = '';
    document.getElementById('carbonBranchResults').innerHTML = '';
    document.getElementById('carbonStorageSection').style.display = 'none';
    document.getElementById('carbonStorageResults').innerHTML = '';

    const sec = document.getElementById('dynamicCarbonSection');
    sec.style.display = '';
    const charts = document.getElementById('dynamicCarbonCharts');
    if (charts) {
      charts.innerHTML = `
        <div class="dynamic-carbon-grid">
          <div id="dynamicCarbonMixChart" class="dynamic-carbon-chart"></div>
          <div id="dynamicCarbonTrendChart" class="dynamic-carbon-chart"></div>
        </div>
        <div class="dynamic-carbon-controls">
          <select id="dynamicCarbonBusSelector" class="dynamic-carbon-select"></select>
          <button type="button" id="btnAddDynamicCarbonBus" class="btn btn-sm">添加节点</button>
        </div>
        <div id="dynamicCarbonSelectedBuses" class="dynamic-carbon-selected"></div>
        <div id="dynamicCarbonBusTrendChart" class="dynamic-carbon-chart dynamic-carbon-wide"></div>
      `;
    }
    let html = '<table><thead><tr><th>时段</th><th>潮流收敛</th><th>OPF收敛</th><th>源端(t)</th><th>负荷(t)</th><th>储能充电(t)</th><th>储能放电(t)</th><th>损耗(t)</th><th>碳平衡残差(t)</th></tr></thead><tbody>';
    (data.step_results || []).forEach(s => {
      const stepBalance = Number(s.balance_error_tco2 ?? (
        Number(s.total_generation_emissions_tco2 || 0) -
        Number(s.total_load_emissions_tco2 || 0) -
        Number(s.total_loss_emissions_tco2 || 0)
      ));
      html += `<tr><td>${s.step}</td><td>${s.pf_converged ? '是' : '否'}</td><td>${s.opf_converged == null ? '—' : (s.opf_converged ? '是' : '否')}</td><td>${Number(s.total_generation_emissions_tco2 || 0).toFixed(6)}</td><td>${Number(s.total_load_emissions_tco2 || 0).toFixed(6)}</td><td>${Number(s.total_storage_charge_emissions_tco2 || 0).toFixed(6)}</td><td>${Number(s.total_storage_discharge_emissions_tco2 || 0).toFixed(6)}</td><td>${Number(s.total_loss_emissions_tco2 || 0).toFixed(6)}</td><td>${stepBalance.toExponential(3)}</td></tr>`;
    });
    html += '</tbody></table>';
    document.getElementById('dynamicCarbonResults').innerHTML = html;
    renderDynamicCarbonCharts(data);
  }

  // ========== Power Flow ==========
  function pfNumber(id, fallback) {
    const el = document.getElementById(id);
    if (!el) return fallback;
    const n = Number(el.value);
    return Number.isFinite(n) ? n : fallback;
  }

  function pfInteger(id, fallback) {
    const n = pfNumber(id, fallback);
    return Number.isFinite(n) ? Math.trunc(n) : fallback;
  }

  function pfBool(id, fallback) {
    const el = document.getElementById(id);
    return el ? !!el.checked : fallback;
  }

  function readPowerFlowOptions() {
    return {
      max_iter: pfInteger('pfMaxIter', 100),
      tol: pfNumber('pfTol', 1e-8),
      fdpf_max_iter: pfInteger('pfFdpfMaxIter', 1000),
      verbose: pfBool('pfVerbose', false),
      enable_pv_pq_conversion: pfBool('pfPvPqConversion', true),
      enable_auto_swing_selection: pfBool('pfAutoSwing', true),
      enable_converter_mode_switching: pfBool('pfConverterModeSwitch', true),
      enable_converter_coordination_check: pfBool('pfCoordCheck', true),
      enable_rigid_vdc_former: pfBool('pfRigidVdcFormer', false),
      loss_model: document.getElementById('pfLossModel')?.value || 'linear',
      pv_q_hysteresis_pu: pfNumber('pfPvQHysteresis', 0.01),
      pv_recover_vm_tol_pu: pfNumber('pfPvRecoverVmTol', 0.01),
      converter_vdc_switch_high_pu: pfNumber('pfVdcSwitchHigh', 0.03),
      converter_vdc_switch_low_pu: pfNumber('pfVdcSwitchLow', 0.01),
      mode_hysteresis_iters: pfInteger('pfModeHysteresis', 2),
      max_delta_va_rad: pfNumber('pfMaxDeltaVa', 1.5),
      max_delta_vm_pu: pfNumber('pfMaxDeltaVm', 0.5),
      max_delta_vdc_pu: pfNumber('pfMaxDeltaVdc', 0.5),
      max_line_search_steps: pfInteger('pfMaxLineSearch', 10),
      max_regularization_steps: pfInteger('pfMaxRegularization', 8),
      regularization_lambda0: pfNumber('pfRegularizationLambda0', 1e-6),
      regularization_growth: pfNumber('pfRegularizationGrowth', 10),
      enable_coupled_jacobian: pfBool('pfCoupledJacobian', false),
      enable_augmented_equations: pfBool('pfAugmentedEquations', false),
      enable_semi_smooth_newton: pfBool('pfSemiSmoothNewton', false),
      globalization: document.getElementById('pfGlobalization')?.value || 'line_search',
      trust_region_radius0: pfNumber('pfTrustRadius0', 1),
      trust_region_max: pfNumber('pfTrustRadiusMax', 10),
      ptc_delta0: pfNumber('pfPtcDelta0', 1),
      ptc_growth: pfNumber('pfPtcGrowth', 2),
      zip_pw: [
        pfNumber('pfZipPwP', 1),
        pfNumber('pfZipPwI', 0),
        pfNumber('pfZipPwZ', 0),
      ],
      zip_qw: [
        pfNumber('pfZipQwP', 1),
        pfNumber('pfZipQwI', 0),
        pfNumber('pfZipQwZ', 0),
      ],
      ac_eval_threads: pfInteger('pfAcEvalThreads', 0),
      enable_solver_profiling: pfBool('pfSolverProfiling', false),
      enable_iteration_log: pfBool('pfIterationLog', false),
      robust_nonlinear: {
        enable_residual_scaling: pfBool('pfRobustScaling', true),
        enable_variable_scaling: pfBool('pfRobustScaling', true),
        enable_jacobian_row_col_equilibration: pfBool('pfRobustEquilibration', true),
        enable_condition_monitor: pfBool('pfRobustCondition', true),
        enable_nonmonotone_linesearch: pfBool('pfRobustNonmonotone', true),
        enable_auto_fallback_scheduling: pfBool('pfRobustAutoFallback', false),
        enable_homotopy: pfBool('pfRobustHomotopy', true),
        enable_newton_krylov_fallback: pfBool('pfRobustKrylov', false),
        min_vm_pu: pfNumber('pfRobustMinVm', 1e-8),
      },
    };
  }

  async function runPowerFlow() {
    return withAnalysisQueue(async () => {
      setStatus('潮流计算中...', 'busy');
      const method = document.getElementById('pfMethod').value;

      // Sync canvas to backend first
      if (!await syncToBackend()) {
        setStatus('同步失败', 'error');
        return null;
      }

      const pfOptions = readPowerFlowOptions();
      if (!await waitForBackendIdle()) {
        setStatus('后端繁忙', 'error');
        return null;
      }
      const data = await apiPost('/api/session/pf', {
        method: method,
        options: pfOptions
      });

      if (data) {
        data.options_requested = pfOptions;
        const pfData = normalizePowerFlowResult(data);
        const converged = pfData.converged;
        const islandInfo = pfData.islands_detected
          ? ` [检测到${pfData.islands_detected}个岛, ${pfData.solvable_islands}个可解]`
          : '';
        if (converged) {
          log(`潮流计算收敛 [${pfData.method_actual || method}]: 迭代${pfData.iterations}次, 残差=${Number(pfData.residual).toExponential(4)}${islandInfo}`, 'success');
          setStatus('潮流收敛', '');
        } else {
          log(`潮流计算未收敛 [${pfData.method_actual || method}]: 迭代${pfData.iterations}次${islandInfo}`, 'warn');
          setStatus('未收敛', 'error');
        }

        // Display results.  Always replace the previous GUI cache, including after
        // a failed run, so a later successful run is never masked by stale arrays.
        _lastPfData = pfData;
        try {
          showPowerFlowResultsTables(pfData);
        } catch (err) {
          log(`潮流结果表刷新失败: ${err.message || err}`, 'error');
        }
        try {
          Canvas.showPowerFlowResults(pfData);
        } catch (err) {
          log(`潮流画布叠加刷新失败，结果表已更新: ${err.message || err}`, 'warn');
        }
        switchTab('results');
      } else {
        setStatus('计算失败', 'error');
      }
      return data;
    });
  }

  // ========== Optimal Power Flow ==========
  // Reads the OPF sub-toolbar (solver + 4 constraint families) and posts to the
  // unified /api/session/opf endpoint, which routes to the parity-IPM hybrid
  // AC/DC formulation (default), the AC OPF, or the DC OPF. The constraint
  // checkboxes gate branch thermal limits, converter capacity circles,
  // converter AC/DC current limits, and converter modulation-ratio limits, all
  // of which are honoured by the parity manual-KKT formulation.
  async function runOpf() {
    if (_activeLoadPromise) {
      setStatus('等待算例加载完成...', 'busy');
      await _activeLoadPromise.catch(() => null);
    }
    return withAnalysisQueue(async () => {
      setStatus('最优潮流计算中...', 'busy');
      const solver = document.getElementById('opfSolver')?.value || 'parity';
      const checkConsistency = !!(document.getElementById('opfCheckConsistency')?.checked);
      const constraints = {
        branch_limits:        !!(document.getElementById('opfBranchLimits')?.checked),
        converter_capacity:   !!(document.getElementById('opfConvCapacity')?.checked),
        converter_current:    !!(document.getElementById('opfConvCurrent')?.checked),
        converter_modulation: !!(document.getElementById('opfConvModulation')?.checked),
      };

      // Sync canvas to backend first so the OPF runs against the current edits.
      if (!await syncToBackend()) {
        setStatus('同步失败', 'error');
        return null;
      }

      if (!await waitForBackendIdle()) {
        setStatus('后端繁忙', 'error');
        return null;
      }
      let resp = await apiPostResult('/api/session/opf',
        { solver, constraints, check_consistency: checkConsistency });
      if (!resp.ok && resp.error === ANALYSIS_BUSY_ERROR) {
        setStatus('等待当前分析完成...', 'busy');
        if (await waitForBackendIdle()) {
          resp = await apiPostResult('/api/session/opf',
            { solver, constraints, check_consistency: checkConsistency },
            { quiet: true });
        }
      }
      const data = resp.ok ? resp.data : null;
      if (data) {
        data._constraints = constraints;
        const backendTag = data.solver_backend ? ` · ${data.solver_backend}` : '';
        if (data.converged) {
          log(`最优潮流收敛 [${solver}${backendTag}]: 迭代${data.iterations || 0}次, 目标=${Number(data.objective || 0).toFixed(4)}`, 'success');
          setStatus('最优潮流收敛', '');
        } else {
          log(`最优潮流未收敛 [${solver}${backendTag}]: ${data.status || ''}`, 'warn');
          setStatus('未收敛', 'error');
        }
        // Surface the OPF↔PF consistency verdict in the activity log too.
        if (data.consistency && data.consistency.ran) {
          const c = data.consistency;
          if (!c.pf_converged) {
            log('一致性校验: OPF后潮流未收敛，无法校验', 'warn');
          } else if (c.consistent) {
            log(`一致性校验通过: max|ΔVm|=${Number(c.max_dvm_pu).toExponential(2)}pu, max|ΔVa|=${Number(c.max_dva_deg).toExponential(2)}°`, 'success');
          } else {
            log(`一致性校验存在偏差: max|ΔVm|=${Number(c.max_dvm_pu).toExponential(2)}pu(Bus ${c.max_dvm_bus}), max|ΔVa|=${Number(c.max_dva_deg).toExponential(2)}°(Bus ${c.max_dva_bus})`, 'warn');
          }
        }
        _lastOpfData = data;
        // Overlay the optimal voltages on the canvas (AC vm/va + DC vdc), reusing
        // the power-flow voltage overlay. The OPF endpoint now also emits the
        // post-OPF terminal flows, so wire them into the same canonical fields the
        // PF overlay/tables already consume.
        if (data.post_pf) {
          const pf = data.post_pf;
          if (Array.isArray(pf.vm) && pf.vm.length) data.vm = pf.vm;
          if (Array.isArray(pf.va) && pf.va.length) data.va = pf.va;
          if (Array.isArray(pf.vdc) && pf.vdc.length) data.vdc = pf.vdc;
          [
            'geo_buses',
            'geo_ac_branches',
            'geo_dc_branches',
            'geo_vsc',
            'geo_dcdc',
            'geo_trafo3w',
            'geo_gen',
            'geo_er',
            'component_results',
            'dc_storage_results',
            'ac_switch_flows',
            'ac_circuit_breaker_flows',
            'dc_circuit_breaker_flows',
          ].forEach(key => {
            if (Array.isArray(pf[key])) data[key] = pf[key];
          });
          if (pf.power_balance_diagnostics && typeof pf.power_balance_diagnostics === 'object') {
            data.power_balance_diagnostics = pf.power_balance_diagnostics;
          }
          if (Array.isArray(pf.branch_flows)) {
            data.branch_flows = pf.branch_flows;
            if (!Array.isArray(pf.geo_ac_branches) || !pf.geo_ac_branches.length) data.geo_ac_branches = pf.branch_flows.map((b, i) => ({
              index: b.index ?? i,
              from: b.from_bus ?? b.from,
              to: b.to_bus ?? b.to,
              from_bus: b.from_bus ?? b.from,
              to_bus: b.to_bus ?? b.to,
              pf_mw: b.pf_mw, pt_mw: b.pt_mw,
              qf_mvar: b.qf_mvar, qt_mvar: b.qt_mvar,
              loss_mw: b.loss_mw ?? ((Number(b.pf_mw) || 0) + (Number(b.pt_mw) || 0)),
              loading_pct: b.loading_pct || 0,
              rate_mva: b.rate_mva || 0,
            }));
          }
          if (Array.isArray(pf.dc_branch_flows)) {
            data.dc_branch_flows = pf.dc_branch_flows;
            if (!Array.isArray(pf.geo_dc_branches) || !pf.geo_dc_branches.length) data.geo_dc_branches = pf.dc_branch_flows.map((b, i) => ({
              index: b.index ?? i,
              from: b.from_bus ?? b.from,
              to: b.to_bus ?? b.to,
              from_bus: b.from_bus ?? b.from,
              to_bus: b.to_bus ?? b.to,
              pf_mw: b.pf_mw, pt_mw: b.pt_mw,
              loss_mw: b.loss_mw ?? ((Number(b.pf_mw) || 0) + (Number(b.pt_mw) || 0)),
              loading_pct: b.loading_pct || 0,
              rate_mva: b.rate_mva || 0,
            }));
          }
          if (Array.isArray(pf.vsc_transfers)) {
            data.vsc_transfers = pf.vsc_transfers;
            if (!Array.isArray(pf.geo_vsc) || !pf.geo_vsc.length) data.geo_vsc = pf.vsc_transfers;
          }
          if (Array.isArray(pf.dcdc_transfers)) {
            data.dcdc_transfers = pf.dcdc_transfers;
            if (!Array.isArray(pf.geo_dcdc) || !pf.geo_dcdc.length) data.geo_dcdc = pf.dcdc_transfers;
          }
        }
        normalizePowerFlowResult(data);
        if (Canvas.showPowerFlowResults) Canvas.showPowerFlowResults(data);
        showOpfResults(data);
        switchTab('results');
      } else {
        if (resp.error) log(`最优潮流计算失败: ${resp.error}`, 'error');
        setStatus('计算失败', 'error');
      }
      return data;
    });
  }

  // Render the optimal-power-flow result tables: summary, the enforced
  // constraint scope returned by the server, generator / converter dispatch,
  // and AC/DC bus voltages with locational marginal prices.
  function showOpfResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('opf');

    // The shared summary banner is owned by the power-flow view; clear it so the
    // OPF view shows only its own self-contained summary block.
    const sharedSummary = document.getElementById('resultsSummary');
    if (sharedSummary) sharedSummary.innerHTML = '';

    const fmt = (x, d = 4) => (x == null || Number.isNaN(Number(x))) ? '-' : Number(x).toFixed(d);
    const req = data._constraints || {};
    // Bus/component map so OPF result rows pan to the matching canvas component
    // when clicked, exactly like the power-flow and carbon-flow result tables.
	    const busMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap)
	      ? Canvas.getCompBusMap() : { ac: {}, dc: {}, gen: {}, vsc: {} };
	    const panAttr = (compId) => compId !== undefined
	      ? ` class="topo-clickable" data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
	    const attrForRow = (row, fallbackCompId) => panAttr(rowCanvasCompId(row, busMap) ?? fallbackCompId);

    // Summary
    const sumDiv = document.getElementById('opfSummary');
    if (sumDiv) {
      sumDiv.innerHTML = `
        <div class="result-item"><span class="result-label">求解器</span>
          <span class="result-value">${escapeHtml(data.solver || '')}</span></div>
        <div class="result-item"><span class="result-label">实际后端</span>
          <span class="result-value">${escapeHtml(data.solver_backend || '-')}</span></div>
        <div class="result-item"><span class="result-label">收敛</span>
          <span class="result-value ${data.converged ? 'result-converged' : 'result-failed'}">
            ${data.converged ? '✓ 是' : '✗ 否'}</span></div>
        <div class="result-item"><span class="result-label">迭代次数</span>
          <span class="result-value">${data.iterations || 0}</span></div>
        <div class="result-item"><span class="result-label">目标值</span>
          <span class="result-value">${fmt(data.objective)}</span></div>
        <div class="result-item"><span class="result-label">状态</span>
          <span class="result-value">${escapeHtml(data.status || '')}</span></div>
      `;
    }

    // Enforced constraint scope (server echoes what was actually applied).
    const scopeDiv = document.getElementById('opfScope');
    const scope = data.scope || {};
    if (scopeDiv) {
      const yn = (v) => v ? '<span style="color:#15803d">启用</span>' : '<span style="color:#94a3b8">关闭</span>';
      const branchOn = scope.branch_limits != null ? scope.branch_limits : req.branch_limits;
      scopeDiv.innerHTML = `
        <table><thead><tr><th>约束族</th><th>状态</th></tr></thead><tbody>
          <tr><td>换流器模型范围</td><td>${escapeHtml(scope.model_scope || '-')}</td></tr>
          <tr><td>支路热稳定限值</td><td>${yn(branchOn)}</td></tr>
          <tr><td>换流器容量圆 (P²+Q²≤S²)</td><td>${yn(scope.capacity)}</td></tr>
          <tr><td>换流器电流限值 (i_ac/i_dc)</td><td>${yn(scope.current)}</td></tr>
          <tr><td>换流器调制限值 (m_min/m_max)</td><td>${yn(scope.modulation)}</td></tr>
          <tr><td>DC/DC占空比限值 (d_min/d_max)</td><td>${yn(scope.dcdc_duty)}</td></tr>
          <tr><td>直流电压控制</td><td>${escapeHtml(scope.vdc_control || '-')}</td></tr>
        </tbody></table>`;
    }

    // Generator dispatch
	    const genDiv = document.getElementById('opfGenResults');
	    if (genDiv) {
	      const pg = data.pg_mw || [];
	      const qg = data.qg_mvar || [];
	      const genRows = Array.isArray(data.geo_gen) && data.geo_gen.length
	        ? data.geo_gen
	        : Array.isArray(data.generator_dispatch) && data.generator_dispatch.length
	        ? data.generator_dispatch
	        : pg.map((p, i) => {
	            const item = SYS?.ac?.generators?.[i] || {};
	            return { position: i, index: item.index ?? i, name: item.name, bus: item.bus,
	              pg_mw: p, qg_mvar: qg[i], canvas_type: 'gen', canvas_index: item.index ?? i };
	          });
	      if (genRows.length) {
	        let html = '<table><thead><tr><th>发电机</th><th>Bus</th><th>Pg(MW)</th><th>Qg(MVar)</th></tr></thead><tbody>';
	        genRows.forEach((g, i) => {
	          const p = g.pg_mw ?? pg[i];
	          const q = g.qg_mvar ?? qg[i];
	          const label = g.name || (g.index != null ? `#${g.index}` : `pos ${g.position ?? i}`);
	          html += `<tr${attrForRow(g)}><td>${escapeHtml(label)}</td><td>${g.bus ?? '-'}</td><td>${fmt(p)}</td><td>${fmt(q)}</td></tr>`;
	        });
	        html += '</tbody></table>';
	        genDiv.innerHTML = html;
      } else {
        genDiv.innerHTML = '<p class="empty-hint">无发电机出力数据</p>';
      }
    }

    // Converter dispatch — prefer the post-OPF PF transfers so this table
    // matches the canvas flow overlay, including converter losses/signs.
    const convSec = document.getElementById('opfConvSection');
    const convDiv = document.getElementById('opfConvResults');
	    const pac = data.pac_mw || [];
	    const qac = data.qac_mvar || [];
	    const pdcdc = data.pdcdc_mw || [];
	    const vscRows = Array.isArray(data.vsc_dispatch) ? data.vsc_dispatch : [];
	    const dcdcRows = Array.isArray(data.dcdc_dispatch) ? data.dcdc_dispatch : [];
	    const postVsc = data.post_pf?.vsc_transfers || data.vsc_transfers || [];
	    const postDcdc = data.post_pf?.dcdc_transfers || data.dcdc_transfers || [];
	    if (convSec && convDiv && (pac.length || postVsc.length || vscRows.length || pdcdc.length || postDcdc.length || dcdcRows.length)) {
	      convSec.style.display = '';
	      let html = '';
	      if (pac.length || postVsc.length || vscRows.length) {
	        const n = Math.max(pac.length, postVsc.length, vscRows.length);
	        html += '<table><thead><tr><th>VSC</th><th>AC Bus</th><th>DC Bus</th><th>Pac(MW)</th><th>Qac(MVar)</th><th>Pdc(MW)</th><th>Loss(MW)</th></tr></thead><tbody>';
	        for (let i = 0; i < n; i++) {
	          const row = vscRows[i] || {};
	          const v = postVsc[i] || {};
	          const merged = { ...row, ...v, canvas_type: row.canvas_type || 'vsc', canvas_index: row.canvas_index ?? v.index ?? row.index };
	          const label = row.name || (merged.index != null ? `#${merged.index}` : `pos ${i}`);
	          html += `<tr${attrForRow(merged)}><td>${escapeHtml(label)}</td><td>${merged.bus_ac ?? '-'}</td><td>${merged.bus_dc ?? '-'}</td>` +
	                  `<td>${fmt(v.p_ac_mw ?? row.pac_mw ?? pac[i])}</td><td>${fmt(v.q_ac_mvar ?? row.qac_mvar ?? qac[i])}</td>` +
	                  `<td>${fmt(v.p_dc_mw)}</td><td>${fmt(v.loss_mw)}</td></tr>`;
	        }
	        html += '</tbody></table>';
	      }
	      if (pdcdc.length || postDcdc.length || dcdcRows.length) {
	        const n = Math.max(pdcdc.length, postDcdc.length, dcdcRows.length);
	        html += '<table style="margin-top:8px"><thead><tr><th>DC/DC</th><th>Bus In</th><th>Bus Out</th><th>OPF P(MW)</th><th>Pin(MW)</th><th>Pout(MW)</th><th>Loss(MW)</th></tr></thead><tbody>';
	        for (let i = 0; i < n; i++) {
	          const row = dcdcRows[i] || {};
	          const d = postDcdc[i] || {};
	          const merged = { ...row, ...d, canvas_type: row.canvas_type || 'dcdcConverter', canvas_index: row.canvas_index ?? d.index ?? row.index };
	          const label = row.name || (merged.index != null ? `#${merged.index}` : `pos ${i}`);
	          html += `<tr${attrForRow(merged)}><td>${escapeHtml(label)}</td><td>${merged.bus_in ?? '-'}</td><td>${merged.bus_out ?? '-'}</td>` +
	                  `<td>${fmt(row.pdcdc_mw ?? pdcdc[i])}</td><td>${fmt(d.p_in_mw)}</td><td>${fmt(d.p_out_mw)}</td><td>${fmt(d.loss_mw)}</td></tr>`;
	        }
	        html += '</tbody></table>';
	      }
      convDiv.innerHTML = html || '<p class="empty-hint">无换流器出力数据</p>';
    } else if (convSec) {
      convSec.style.display = 'none';
    }

    // DC bus voltages — only for hybrid AC/DC cases.
	    const dcSec = document.getElementById('opfDcBusSection');
	    const dcDiv = document.getElementById('opfDcBusResults');
	    const vdc = data.vdc || [];
	    const dcRows = Array.isArray(data.dc_bus_results) && data.dc_bus_results.length
	      ? data.dc_bus_results
	      : vdc.map((v, i) => {
	          const busId = SYS?.dc?.buses?.[i]?.index;
	          return { position: i, index: busId ?? i, vdc_pu: v, canvas_type: 'dc', canvas_index: busId ?? i };
	        });
	    if (dcSec && dcDiv && dcRows.length) {
	      dcSec.style.display = '';
	      let html = '<table><thead><tr><th>DC Bus</th><th>Vdc(pu)</th></tr></thead><tbody>';
	      dcRows.forEach((row, i) => {
	        const busId = row.index ?? row.canvas_index;
	        html += `<tr${attrForRow(row)}><td>${busId ?? `pos ${row.position ?? i}`}</td><td>${fmt(row.vdc_pu ?? vdc[i], 6)}</td></tr>`;
	      });
	      html += '</tbody></table>';
	      dcDiv.innerHTML = html;
    } else if (dcSec) {
      dcSec.style.display = 'none';
    }

    renderAllPowerFlowComponentStatus(data, busMap, {
      sectionId: 'opfAllComponentsSection',
      resultsId: 'opfAllComponentsResults',
    });

	    // AC bus voltages and locational marginal prices.
	    const busDiv = document.getElementById('opfBusResults');
	    const vm = data.vm || [];
	    const acRows = Array.isArray(data.ac_bus_results) && data.ac_bus_results.length
	      ? data.ac_bus_results
	      : vm.map((v, i) => {
	          const busId = SYS?.ac?.buses?.[i]?.index;
	          return { position: i, index: busId ?? i, vm_pu: v, va_rad: data.va?.[i],
	            lmp_p: data.lmp_p?.[i] ?? data.lmp?.[i], lmp_q: data.lmp_q?.[i],
	            canvas_type: 'ac', canvas_index: busId ?? i };
	        });
	    if (busDiv && acRows.length) {
	      const va = data.va || [];
	      const lmpP = data.lmp_p || [];
	      const lmpQ = data.lmp_q || [];
	      const hasLmp = acRows.some(r => r.lmp_p != null) || lmpP.length > 0 || Array.isArray(data.lmp);
	      const hasVm = acRows.some(r => r.vm_pu != null) || vm.length > 0;
	      let html = `<table><thead><tr><th>Bus</th>${hasVm ? '<th>Vm(pu)</th>' : ''}<th>Va(°)</th>${hasLmp ? '<th>LMP-P</th><th>LMP-Q</th>' : ''}</tr></thead><tbody>`;
	      acRows.forEach((row, i) => {
	        const busId = row.index ?? row.canvas_index;
	        const v = row.vm_pu ?? vm[i];
	        const ang = row.va_rad != null ? (row.va_rad * 180 / Math.PI).toFixed(4)
	          : (va[i] != null ? (va[i] * 180 / Math.PI).toFixed(4) : '0');
	        const color = v < 0.95 ? 'color:#e06c75' : v > 1.05 ? 'color:#d19a66' : '';
	        html += `<tr${attrForRow(row)}><td>${busId ?? `pos ${row.position ?? i}`}</td>${hasVm ? `<td style="${color}">${fmt(v, 6)}</td>` : ''}<td>${ang}</td>${hasLmp ? `<td>${fmt(row.lmp_p ?? lmpP[i] ?? data.lmp?.[i])}</td><td>${fmt(row.lmp_q ?? lmpQ[i])}</td>` : ''}</tr>`;
	      });
	      html += '</tbody></table>';
	      busDiv.innerHTML = html;
    } else if (busDiv) {
      busDiv.innerHTML = '<p class="empty-hint">无AC节点电压数据</p>';
    }

    // ── OPF ↔ PF consistency audit (建模/参数一致性) ──
    const ccSec = document.getElementById('opfConsistencySection');
    const ccDiv = document.getElementById('opfConsistencyResults');
    const cc = data.consistency;
    if (ccSec && ccDiv && cc && cc.ran) {
      ccSec.style.display = '';
      const sci = (x) => (x == null || Number.isNaN(Number(x))) ? '-' : Number(x).toExponential(2);
      const verdict = !cc.pf_converged
        ? '<span style="color:#e06c75">✗ 潮流未收敛</span>'
        : (cc.consistent ? '<span style="color:#15803d">✓ 一致</span>'
                         : '<span style="color:#d19a66">⚠ 存在偏差</span>');
      const devRow = (label, val, tol, unit, bus) => {
        const over = (tol != null && Number.isFinite(Number(val)) && Number(val) > Number(tol));
        const c = over ? 'color:#e06c75' : '';
        const busTxt = (bus != null && bus >= 0) ? `Bus ${bus}` : '-';
        return `<tr><td>${label}</td><td style="${c}">${sci(val)}${unit}</td><td>${tol != null ? ('≤ ' + tol + unit) : '-'}</td><td>${busTxt}</td></tr>`;
      };
      let html = `<div class="result-item"><span class="result-label">校验结论</span><span class="result-value">${verdict}</span></div>`;
      if (cc.pf_converged) {
        html += '<table style="margin-top:6px"><thead><tr><th>指标</th><th>最大偏差</th><th>容差</th><th>最严节点</th></tr></thead><tbody>';
        html += devRow('|ΔVm| 电压幅值', cc.max_dvm_pu, cc.tol_vm_pu, ' pu', cc.max_dvm_bus);
        html += devRow('|ΔVa| 电压相角', cc.max_dva_deg, cc.tol_va_deg, '°', cc.max_dva_bus);
        if (cc.max_dvdc_bus != null && cc.max_dvdc_bus >= 0)
          html += devRow('|ΔVdc| 直流电压', cc.max_dvdc_pu, cc.tol_vdc_pu, ' pu', cc.max_dvdc_bus);
        if (cc.max_dpf_mw != null) {
          const brTxt = (cc.max_dpf_branch != null && cc.max_dpf_branch >= 0) ? ('Branch ' + cc.max_dpf_branch) : '-';
          html += `<tr><td>|ΔPf| 支路有功流(最大)</td><td>${sci(cc.max_dpf_mw)} MW</td><td>-</td><td>${brTxt}</td></tr>`;
          html += `<tr><td>|ΔQf| 支路无功流(最大)</td><td>${sci(cc.max_dqf_mvar)} MVar</td><td>-</td><td>-</td></tr>`;
          html += `<tr><td>发电/损耗一致性</td><td>${sci(cc.branch_loss_mismatch_mw)} MW</td><td>-</td><td>OPF损耗 ${sci(cc.branch_loss_opf_mw)} / 潮流 ${sci(cc.branch_loss_pf_mw)} MW</td></tr>`;
          if (cc.converter_loss_pf_mw != null && Number(cc.converter_loss_pf_mw) !== 0)
            html += `<tr><td>换流器损耗 (潮流)</td><td>${sci(cc.converter_loss_pf_mw)} MW</td><td>-</td><td>-</td></tr>`;
        }
        html += `<tr><td>平均 |ΔVm| / |ΔVa|</td><td>${sci(cc.mean_dvm_pu)} pu / ${sci(cc.mean_dva_deg)}°</td><td>-</td><td>-</td></tr>`;
        html += `<tr><td>潮流迭代 / 残差</td><td>${cc.pf_iterations || 0} 次 / ${sci(cc.pf_residual)}</td><td>-</td><td>-</td></tr>`;
        html += '</tbody></table>';
        html += '<p class="empty-hint" style="margin-top:6px">在 OPF 调度点固定发电出力与电压设定后独立求解潮流。若 OPF 与潮流共享一致的网络建模/参数，OPF 解即为潮流不动点，电压/相角偏差应接近 0；偏差越大说明两者建模或参数越不一致。</p>';
      } else {
        html += `<p class="empty-hint" style="margin-top:6px">${escapeHtml(cc.note || 'OPF后潮流未收敛，无法完成一致性校验。')}</p>`;
      }
      ccDiv.innerHTML = html;
    } else if (ccSec) {
      ccSec.style.display = 'none';
    }

	    // ── Post-OPF power flow (潮流) at the OPF dispatch ──
	    const postPfSec = document.getElementById('opfPostPfSection');
	    const postPfDiv = document.getElementById('opfPostPfResults');
	    const postPf = data.post_pf;
	    const dcOpfBranches = Array.isArray(data.dc_opf_branch_dispatch) ? data.dc_opf_branch_dispatch : [];
	    if (postPfSec && postPfDiv && postPf && Array.isArray(postPf.branch_flows) && postPf.branch_flows.length) {
	      postPfSec.style.display = '';
	      let html = '<table><thead><tr><th>支路</th><th>P_from(MW)</th><th>Q_from(MVar)</th><th>P_to(MW)</th><th>负载率(%)</th></tr></thead><tbody>';
	      postPf.branch_flows.forEach(b => {
	        const ld = b.loading_pct || 0;
	        const color = ld > 100 ? 'color:#e06c75' : ld > 80 ? 'color:#d19a66' : '';
	        const row = { ...b, canvas_type: b.canvas_type || 'branch', canvas_index: b.canvas_index ?? b.index };
	        const label = b.name || (b.index != null ? `#${b.index} ${b.from_bus}→${b.to_bus}` : `${b.from_bus}→${b.to_bus}`);
	        html += `<tr${attrForRow(row)}><td>${escapeHtml(label)}</td><td>${fmt(b.pf_mw)}</td><td>${fmt(b.qf_mvar)}</td><td>${fmt(b.pt_mw)}</td><td style="${color}">${fmt(ld, 1)}</td></tr>`;
	      });
	      html += '</tbody></table>';
	      html += '<p class="empty-hint" style="margin-top:6px">提示：使用画布上方的「可视化」下拉(潮流/热力图)可在 OPF 解上叠加支路潮流与负载率热力图。</p>';
	      postPfDiv.innerHTML = html;
	    } else if (postPfSec && postPfDiv && dcOpfBranches.length) {
	      postPfSec.style.display = '';
	      let html = '<table><thead><tr><th>支路</th><th>P_from(MW)</th><th>Rate(MVA)</th></tr></thead><tbody>';
	      dcOpfBranches.forEach((b, i) => {
	        const label = b.name || (b.index != null ? `#${b.index} ${b.from_bus}→${b.to_bus}` : `pos ${b.position ?? i}`);
	        html += `<tr${attrForRow(b)}><td>${escapeHtml(label)}</td><td>${fmt(b.pf_mw)}</td><td>${fmt(b.rate_mva)}</td></tr>`;
	      });
	      html += '</tbody></table>';
	      postPfDiv.innerHTML = html;
	    } else if (postPfSec) {
	      postPfSec.style.display = 'none';
	    }

    // ── Post-OPF carbon flow (碳流) at the OPF dispatch ──
    const postCbSec = document.getElementById('opfPostCarbonSection');
    const postCbDiv = document.getElementById('opfPostCarbonResults');
    const carbon = data.post_carbon;
    if (postCbSec && postCbDiv && carbon) {
      postCbSec.style.display = '';
      const summary = carbon.tracing_summary || carbon.matrix_summary || {};
      const totalGen = summary.total_generation_emissions_tco2 ?? null;
      const totalLoad = summary.total_load_emissions_tco2 ?? null;
      const buses = Array.isArray(carbon.bus_carbon) ? carbon.bus_carbon.slice() : [];
      // Top carbon-intensity nodes (descending). Show the actual computed values
      // (only guard against NaN/negative); some datasets use unrealistic emission
      // factors, which the table then surfaces transparently.
      const topNodes = buses
        .filter(b => Number.isFinite(b.carbon_intensity_tco2_mwh) && b.carbon_intensity_tco2_mwh >= 0)
        .sort((a, b) => b.carbon_intensity_tco2_mwh - a.carbon_intensity_tco2_mwh)
        .slice(0, 10);
      let html = '<div class="result-summary-grid" style="margin-bottom:8px">';
      if (totalGen != null) html += `<div class="result-item"><span class="result-label">发电排放(tCO₂)</span><span class="result-value">${fmt(totalGen, 4)}</span></div>`;
      if (totalLoad != null) html += `<div class="result-item"><span class="result-label">负荷排放(tCO₂)</span><span class="result-value">${fmt(totalLoad, 4)}</span></div>`;
      html += `<div class="result-item"><span class="result-label">碳流求解</span><span class="result-value">${carbon.matrix_solved ? '✓' : '—'}</span></div>`;
      html += `<div class="result-item"><span class="result-label">来源</span><span class="result-value">OPF调度</span></div></div>`;
      if (topNodes.length) {
        const busMapAc = busMap.ac || {};
        html += '<table><thead><tr><th>节点</th><th>碳势(tCO₂/MWh)</th></tr></thead><tbody>';
        topNodes.forEach(b => {
          const attr = panAttr(busMapAc[b.bus_index]);
          html += `<tr${attr}><td>${b.bus_index ?? '-'}</td><td>${fmt(b.carbon_intensity_tco2_mwh, 4)}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      postCbDiv.innerHTML = html;
    } else if (postCbSec) {
      postCbSec.style.display = 'none';
    }
  }

  // ========== Short Circuit ==========
  function showScDialog() {
    document.getElementById('scDialog').style.display = 'flex';
  }

  function hideScDialog() {
    document.getElementById('scDialog').style.display = 'none';
  }

  function scReadNumber(id, fallback) {
    const el = document.getElementById(id);
    const value = Number(el?.value);
    return Number.isFinite(value) ? value : fallback;
  }

  function scReadChecked(id, fallback = false) {
    const el = document.getElementById(id);
    return el ? !!el.checked : fallback;
  }

  function syncScToolbarToHidden() {
    const pairs = [
      ['scDomainSelect', 'scDomain'],
      ['faultTypeSelect', 'scFaultType'],
      ['scCalcTypeSelect', 'scCalcType'],
      ['voltageCorrectionFactor', 'scCFactor'],
      ['scKappaMethodSelect', 'scKappaMethod'],
      ['scTopologySelect', 'scTopology'],
    ];
    pairs.forEach(([srcId, dstId]) => {
      const src = document.getElementById(srcId);
      const dst = document.getElementById(dstId);
      if (src && dst) dst.value = src.value;
    });
  }

  function getShortCircuitOptions({ detailed = false } = {}) {
    syncScToolbarToHidden();
    return {
      fault_type: document.getElementById('scFaultType')?.value || 'ThreePhase',
      calc_type: document.getElementById('scCalcType')?.value || 'Max',
      c_factor: scReadNumber('scCFactor', 0),
      kappa_method: document.getElementById('scKappaMethod')?.value || 'B',
      topology: document.getElementById('scTopology')?.value || 'Meshed',
      fault_impedance_pu: scReadNumber('scFaultImpedance', 0),
      breaking_time_s: scReadNumber('scBreakingTime', 0.05),
      ith_duration_s: scReadNumber('scIthDuration', 1.0),
      base_frequency_hz: scReadNumber('scBaseFrequency', 50),
      default_xdpp: scReadNumber('scDefaultXdpp', 0.2),
      compute_branch_flows: detailed && scReadChecked('scComputeBranchFlows', true),
      compute_voltage_drops: detailed && scReadChecked('scComputeVoltageDrops', true),
      compute_ith: scReadChecked('scComputeIth', true),
    };
  }

  function getDcShortCircuitOptions() {
    return {
      fault_resistance_pu: scReadNumber('scFaultImpedance', 0),
      source_voltage_pu: scReadNumber('dcScSourceVoltage', 0),
      consider_dc_breakers: scReadChecked('dcScConsiderBreakers', true),
      dc_breakers_control_branches: scReadChecked('dcScBreakerControlsBranch', true),
      add_unassigned_closed_breaker_edges: scReadChecked('dcScAddBreakerEdges', true),
    };
  }

  async function runShortCircuit() {
    hideScDialog();
    setStatus('短路计算中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const faultBusRaw = (document.getElementById('scFaultBus').value || '').trim();
    const domain = (document.getElementById('scDomain')?.value || 'AC').toUpperCase();
    if (domain === 'DC') {
      const dcBusIds = Array.isArray(SYS?.dc?.buses) ? SYS.dc.buses.map(b => b.index) : [];
      const faultBusIds = faultBusRaw === ''
        ? dcBusIds
        : faultBusRaw.split(/[,\s]+/).map(x => parseInt(x, 10)).filter(Number.isInteger);
      if (!faultBusIds.length) {
        setStatus('没有可计算的DC故障母线。', 'error');
        return;
      }
      const resp = await apiPostResult('/api/session/dc_sc', {
        fault_bus_ids: faultBusIds,
        options: getDcShortCircuitOptions(),
      });
      if (!resp.ok) {
        setStatus(resp.error || 'DC短路计算失败', 'error');
        return;
      }
      log(`DC短路计算完成: ${faultBusIds.length} 个母线结果`, 'success');
      setStatus('DC短路完成');
      showDcShortCircuitResults(resp.data, faultBusIds[0]);
      switchTab('results');
      return;
    }

    const overviewOptions = getShortCircuitOptions({ detailed: false });

    // Blank fault bus → short circuit at every bus (overview mode).
    if (faultBusRaw === '') {
      const data = await apiPost('/api/session/sc', {
        options: {
          ...overviewOptions,
          compute_all_buses: true,
        }
      });
      if (data) {
        log(`短路计算完成: ${data.bus_results?.length || 0} 个母线结果`, 'success');
        setStatus('短路完成 (全部母线)');
        showShortCircuitResults(data);
        switchTab('results');
      } else {
        setStatus('计算失败', 'error');
      }
      return;
    }

    // A specific fault location was set → run a validated single-bus fault.
    // The server resolves the id against the real bus indices and returns
    // HTTP 400 when the id does not exist, so a non-existent bus can no
    // longer "still calculate".
    const faultBus = parseInt(faultBusRaw, 10);
    if (!Number.isInteger(faultBus)) {
      setStatus(`故障母线 ID “${faultBusRaw}” 无效，请输入整数母线编号。`, 'error');
      return;
    }

    const resp = await apiPostResult('/api/session/sc_detailed', {
      fault_bus_ids: [faultBus],
      ...getShortCircuitOptions({ detailed: true }),
    });

    if (!resp.ok) {
      const raw = resp.error || '';
      const friendly = /not found/i.test(raw)
        ? `故障母线 ID ${faultBus} 不存在，请输入系统中真实存在的母线编号（与结果表 / 画布中显示的 Bus 编号一致）。`
        : (raw || '计算失败');
      setStatus(friendly, 'error');
      return;
    }

    log(`短路计算完成: 故障母线 ${faultBus}`, 'success');
    setStatus(`短路完成 (故障母线 ${faultBus})`);
    showShortCircuitDetailedResults(resp.data, faultBus);
    switchTab('results');
  }

  // ========== Harmonic Power Flow (hybrid AC/DC) ==========
  function parseOrderList(raw, fallback) {
    const list = (raw || '').split(/[,\s]+/)
      .map(s => parseInt(s, 10))
      .filter(n => Number.isInteger(n) && n >= 0);
    return list.length ? list : fallback;
  }

  // Read the shared HPFOptions block from the toolbar.
  function hpfReadOptions() {
    const acOrders = parseOrderList(document.getElementById('hpfAcOrders')?.value,
                                    [5, 7, 11, 13, 17, 19, 23, 25]);
    const dcOrders = parseOrderList(document.getElementById('hpfDcOrders')?.value,
                                    [2, 6, 12, 18, 24]);
    const xpp = parseFloat(document.getElementById('hpfSourceXpp')?.value || '0.2');
    return {
      ac_orders: acOrders,
      dc_orders: dcOrders,
      include_load_impedance: document.getElementById('hpfLoadImpedance')?.checked ?? true,
      auto_nic_from_vscs: document.getElementById('hpfAutoNic')?.checked ?? true,
      default_source_xpp_pu: Number.isFinite(xpp) ? xpp : 0.2,
      skin_effect: document.getElementById('hpfSkin')?.value || 'none',
      standard: document.getElementById('hpfStandard')?.value || '',
      run_base_power_flow: true,
    };
  }

  // Show only the listed harmonics result sections, hide the rest.
  function hpfShowSections(ids) {
    const all = ['hpfAcSection', 'hpfDcSection', 'hpfSpectrumSection', 'hpfBranchSection',
                 'hpfFreqScanSection', 'hpf3phSection', 'hpfMetricsSection', 'hpfNewtonSection'];
    all.forEach(id => {
      const el = document.getElementById(id);
      if (el) el.style.display = ids.includes(id) ? 'block' : 'none';
    });
  }

  // Toggle toolbar controls so only those relevant to the active mode are shown.
  function hpfUpdateModeControls() {
    const mode = document.getElementById('hpfMode')?.value || 'penetration';
    document.querySelectorAll('.hpf-ctl').forEach(el => {
      const modes = (el.getAttribute('data-modes') || '').split(/\s+/);
      el.style.display = modes.includes(mode) ? '' : 'none';
    });
  }

  const transientEventLabels = {
    ACBranchTrip: 'AC支路跳闸',
    ACBranchClose: 'AC支路合闸',
    DCBranchTrip: 'DC支路跳闸',
    DCBranchClose: 'DC支路合闸',
    ACLoadScale: 'AC负荷阶跃',
    DCLoadScale: 'DC负荷阶跃',
    GeneratorTrip: '发电机跳闸',
    VSCTrip: 'VSC跳闸',
    DCDCTrip: 'DC/DC跳闸',
    StoragePowerStep: 'AC储能P阶跃',
    DCStoragePowerStep: 'DC储能P阶跃',
    FaultShunt: '母线故障',
    ClearFault: '清除故障',
  };

  const transientEditableDurationTypes = new Set([
    'FaultShunt',
    'ACLoadScale',
    'DCLoadScale',
    'StoragePowerStep',
    'DCStoragePowerStep',
    'Custom',
  ]);

  function transientEventLabel(type) {
    return transientEventLabels[type] || type || '扰动';
  }

  function transientEventUsesBus(eventType) {
    return eventType === 'FaultShunt' || eventType === 'ClearFault' ||
      eventType === 'ACLoadScale' || eventType === 'DCLoadScale';
  }

  function transientEventDefaultDomain(eventType) {
    if (eventType === 'DCLoadScale' || eventType === 'DCBranchTrip' ||
        eventType === 'DCBranchClose' || eventType === 'DCStoragePowerStep') {
      return 'DC';
    }
    return 'AC';
  }

  function transientDefaultEventParams(type, value, duration) {
    const params = {};
    if (type === 'FaultShunt') {
      if (Number(value) > 0) params.g_pu = Number(value);
      if (Number(duration) > 0) params.duration_s = Number(duration);
    } else if (type === 'ACLoadScale' || type === 'DCLoadScale') {
      params.scale = Math.max(0, Number(value) || 0);
      if (Number(duration) > 0) params.duration_s = Number(duration);
    } else if (type === 'StoragePowerStep' || type === 'DCStoragePowerStep') {
      params.p_ref_mw = Number(value) || 0;
      if (Number(duration) > 0) params.duration_s = Number(duration);
    }
    return params;
  }

  function transientReadParamsInput() {
    const raw = String(document.getElementById('trEventParams')?.value || '').trim();
    if (!raw) return {};
    try {
      const parsed = JSON.parse(raw);
      if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) {
        throw new Error('params must be an object');
      }
      const params = {};
      Object.entries(parsed).forEach(([key, value]) => {
        const num = Number(value);
        if (Number.isFinite(num)) params[key] = num;
      });
      return params;
    } catch (err) {
      throw new Error(`扰动参数 JSON 无效: ${err.message || err}`);
    }
  }

  function transientReadOptionalNumber(id) {
    const raw = String(document.getElementById(id)?.value || '').trim();
    if (!raw) return null;
    const value = Number(raw);
    return Number.isFinite(value) ? value : null;
  }

  function transientNormalizeEvent(raw, fallbackIndex = 0) {
    const type = raw?.type || '';
    if (!type) return null;
    const value = numberOr(raw.value, 0.0);
    const duration = Math.max(0, numberOr(raw.duration_s, 0.0));
    const params = {
      ...transientDefaultEventParams(type, value, duration),
      ...(raw.params || {}),
    };
    const event = {
      type,
      time_s: Math.max(0, numberOr(raw.time_s, 0.2)),
      component_index: Math.max(0, parseInt(raw.component_index ?? raw.target ?? 0, 10) || 0),
      bus: Math.max(0, parseInt(raw.bus ?? 0, 10) || 0),
      value,
      duration_s: duration,
      component_type: raw.component_type || transientEventDefaultDomain(type),
      label: String(raw.label || '').trim(),
      params,
    };
    if (transientEventUsesBus(type)) {
      event.bus = event.bus || event.component_index;
      event.component_index = event.bus;
    }
    if (!event.label) {
      const target = transientEventUsesBus(type)
        ? `${event.component_type || 'AC'} bus ${event.bus || '*'}`
        : (event.component_index ? `#${event.component_index}` : '全部');
      event.label = `${transientEventLabel(type)} ${target}`;
    }
    event._id = raw._id || `tr_evt_${Date.now()}_${fallbackIndex}_${Math.random().toString(36).slice(2, 7)}`;
    return event;
  }

  function transientEventFromControls() {
    const type = document.getElementById('trEventType')?.value || '';
    if (!type) return null;
    const domainEl = document.getElementById('trEventDomain');
    const domain = domainEl?.value || transientEventDefaultDomain(type);
    const target = parseInt(document.getElementById('trEventTarget')?.value, 10) || 0;
    const value = numberOr(document.getElementById('trEventValue')?.value, type.includes('LoadScale') ? 1.10 : 0.0);
    const duration = numberOr(document.getElementById('trEventDuration')?.value, type === 'FaultShunt' ? 0.08 : 0.0);
    const params = transientReadParamsInput();
    if (type === 'FaultShunt') {
      const rPu = transientReadOptionalNumber('trFaultRPu');
      const xPu = transientReadOptionalNumber('trFaultXPu');
      if (rPu != null) params.r_pu = Math.max(0, rPu);
      if (xPu != null) params.x_pu = xPu;
    }
    const event = transientNormalizeEvent({
      type,
      time_s: numberOr(document.getElementById('trEventTime')?.value, 0.2),
      component_index: target,
      bus: transientEventUsesBus(type) ? target : 0,
      value,
      duration_s: duration,
      component_type: domain,
      params,
      label: document.getElementById('trEventLabel')?.value || '',
    }, _transientEvents.length);
    return event;
  }

  function transientFormatEvent(event) {
    if (!event) return '';
    const type = transientEventLabel(event.type);
    const target = transientEventUsesBus(event.type)
      ? `${event.component_type || 'AC'} bus ${event.bus || event.component_index || '*'}`
      : (event.component_index ? `#${event.component_index}` : '全部');
    const pieces = [`${Number(event.time_s || 0).toFixed(3)}s`, type, target];
    if (event.type === 'FaultShunt') {
      const p = event.params || {};
      if (p.r_pu != null || p.x_pu != null) {
        pieces.push(`z=${Number(p.r_pu || 0).toExponential(1)}+j${Number(p.x_pu || 0).toExponential(1)}pu`);
      } else {
        pieces.push(`g=${Number(p.g_pu ?? event.value ?? 0).toFixed(3)}pu`);
      }
      if (Number(event.duration_s) > 0) pieces.push(`${Number(event.duration_s).toFixed(3)}s`);
    } else if (event.type.includes('LoadScale')) {
      pieces.push(`x${Number(event.params?.scale ?? event.value ?? 0).toFixed(3)}`);
      if (Number(event.duration_s) > 0) pieces.push(`${Number(event.duration_s).toFixed(3)}s`);
    } else if (event.type.includes('StoragePowerStep')) {
      pieces.push(`${Number(event.params?.p_ref_mw ?? event.value ?? 0).toFixed(3)}MW`);
      if (Number(event.duration_s) > 0) pieces.push(`${Number(event.duration_s).toFixed(3)}s`);
    }
    return pieces.join(' · ');
  }

  function transientSerializableEvents() {
    return _transientEvents
      .map((event, i) => transientNormalizeEvent(event, i))
      .filter(Boolean)
      .sort((a, b) => (a.time_s - b.time_s) || String(a.label).localeCompare(String(b.label)))
      .map(({ _id, ...event }) => event);
  }

  function renderTransientEventSchedule() {
    const box = document.getElementById('trEventSchedule');
    if (!box) return;
    if (!_transientEvents.length) {
      box.innerHTML = '<span class="transient-event-empty">未设置扰动序列</span>';
      return;
    }
    const events = _transientEvents
      .map((event, i) => ({ event, i }))
      .sort((a, b) => (a.event.time_s - b.event.time_s) || a.i - b.i);
    box.innerHTML = events.map(({ event, i }) =>
      `<span class="transient-event-chip" title="${escapeHtml(event.label || '')}">
        ${escapeHtml(transientFormatEvent(event))}
        <button type="button" data-tr-event-remove="${i}" aria-label="删除扰动">×</button>
      </span>`
    ).join('');
    box.querySelectorAll('[data-tr-event-remove]').forEach(btn => {
      btn.addEventListener('click', () => {
        const idx = parseInt(btn.dataset.trEventRemove, 10);
        if (Number.isInteger(idx)) {
          _transientEvents.splice(idx, 1);
          renderTransientEventSchedule();
        }
      });
    });
  }

  function addTransientEventFromControls() {
    let event = null;
    try {
      event = transientEventFromControls();
    } catch (err) {
      setStatus(err.message || String(err), 'error');
      return;
    }
    if (!event) {
      setStatus('请选择一个暂态扰动类型', 'warn');
      return;
    }
    _transientEvents.push(event);
    renderTransientEventSchedule();
    setStatus(`已加入扰动: ${transientFormatEvent(event)}`);
  }

  function clearTransientEvents() {
    _transientEvents = [];
    renderTransientEventSchedule();
    setStatus('已清空暂态扰动序列');
  }

  function updateTransientEventControls() {
    const type = document.getElementById('trEventType')?.value || '';
    const domainEl = document.getElementById('trEventDomain');
    const durationEl = document.getElementById('trEventDuration');
    const valueEl = document.getElementById('trEventValue');
    const valueWrap = document.getElementById('trEventValueWrap');
    const valueLabel = document.getElementById('trEventValueLabel');
    const paramsEl = document.getElementById('trEventParams');
    const faultParamEls = document.querySelectorAll('.tr-fault-param');
    if (domainEl) {
      domainEl.disabled = !(type === 'FaultShunt' || type === 'ClearFault' ||
        type === 'ACLoadScale' || type === 'DCLoadScale');
      domainEl.value = transientEventDefaultDomain(type);
    }
    if (durationEl) {
      const editable = transientEditableDurationTypes.has(type);
      durationEl.disabled = !editable;
      durationEl.title = editable
        ? '事件持续时间。母线故障会自动清除；负荷/储能阶跃会随事件一起记录，当前求解器中阶跃保持到后续事件修改。'
        : '该事件为瞬时切换/跳闸，持续时间不参与求解。';
    }
    if (valueEl) {
      valueEl.disabled = false;
      if (type === 'FaultShunt') {
        if (valueLabel) valueLabel.textContent = '导纳g(pu)';
        valueEl.title = '母线故障并联电导 g(pu)，越大表示故障越强；如果填写 R/X，则后端优先使用 1/(R+jX)';
        if (valueWrap) valueWrap.title = valueEl.title;
      } else if (type.includes('LoadScale')) {
        if (valueLabel) valueLabel.textContent = '负荷倍率';
        valueEl.title = '负荷倍率，例如 1.10 表示增加 10%，0.80 表示降低 20%';
        if (valueWrap) valueWrap.title = valueEl.title;
      } else if (type.includes('StoragePowerStep')) {
        if (valueLabel) valueLabel.textContent = 'P目标(MW)';
        valueEl.title = '储能有功功率目标，正值为放电注入，负值为充电吸收';
        if (valueWrap) valueWrap.title = valueEl.title;
      } else {
        if (valueLabel) valueLabel.textContent = '无需数值';
        valueEl.disabled = true;
        valueEl.title = '跳闸、合闸、清故障等事件只需要对象和时间，不读取该数值';
        if (valueWrap) valueWrap.title = valueEl.title;
      }
    }
    faultParamEls.forEach(el => {
      el.style.display = type === 'FaultShunt' ? 'flex' : 'none';
    });
    if (paramsEl) {
      if (type === 'FaultShunt') {
        paramsEl.placeholder = '{"r_pu":0.001,"x_pu":0.002}';
        paramsEl.title = '可选高级参数：r_pu/x_pu 等值故障阻抗，或 g_pu/b_pu 等值故障导纳，也可写 duration_s';
      } else if (type.includes('LoadScale')) {
        paramsEl.placeholder = '{"scale":1.10,"duration_s":0.20}';
        paramsEl.title = '可选：负荷倍率和持续时间。当前求解器保持阶跃，后续可用反向事件恢复。';
      } else if (type.includes('StoragePowerStep')) {
        paramsEl.placeholder = '{"p_ref_mw":-2.0,"duration_s":0.20}';
        paramsEl.title = '可选：储能有功目标和持续时间。当前求解器保持阶跃，后续可用新目标恢复。';
      } else {
        paramsEl.placeholder = '{}';
        paramsEl.title = '可选事件参数，只保留数值字段';
      }
    }
  }

  function transientReadOptions() {
    const events = transientSerializableEvents();
    return {
      solver_type: document.getElementById('trSolver')?.value || 'heun',
      t_end_s: Math.max(0.001, numberOr(document.getElementById('trEnd')?.value, 1.0)),
      dt_s: Math.max(0.0001, numberOr(document.getElementById('trDt')?.value, 0.01)),
      use_adaptive_step: !!document.getElementById('trAdaptive')?.checked,
      rel_tol: Math.max(1e-10, numberOr(document.getElementById('trRelTol')?.value, 1e-6)),
      abs_tol: Math.max(1e-12, numberOr(document.getElementById('trAbsTol')?.value, 1e-8)),
      max_step_halving: Math.max(0, parseInt(document.getElementById('trMaxHalving')?.value, 10) || 12),
      min_accepted_step_s: 1e-7,
      enforce_voltage_health_check: true,
      allow_low_voltage_during_active_fault: true,
      voltage_collapse_min_ac_pu: Math.max(0, numberOr(document.getElementById('trMinVac')?.value, 0.05)),
      voltage_collapse_min_dc_pu: Math.max(0, numberOr(document.getElementById('trMinVac')?.value, 0.05)),
      voltage_blowup_max_ac_pu: 2.5,
      voltage_blowup_max_dc_pu: 2.5,
      dynamic_dc_link: !!document.getElementById('trDynamicDcLink')?.checked,
      dc_link_capacitance_s: Math.max(0.001, numberOr(document.getElementById('trDcLinkC')?.value, 0.10)),
      dc_link_coupling_conductance_pu: Math.max(0.0, numberOr(document.getElementById('trDcLinkG')?.value, 20.0)),
      run_power_flow_initialization: document.getElementById('trPowerFlowInit')?.checked !== false,
      trim_dynamic_initial_conditions: true,
      algebraic_network_max_iters: Math.max(1, parseInt(document.getElementById('trAlgMaxIter')?.value, 10) || 20),
      algebraic_network_tol: Math.max(1e-12, numberOr(document.getElementById('trAlgTol')?.value, 1e-10)),
      power_flow_options: {
        max_iter: Math.max(5, parseInt(document.getElementById('trPfMaxIter')?.value, 10) || 80),
        tol: Math.max(1e-12, numberOr(document.getElementById('trPfTol')?.value, 1e-8)),
        enable_converter_coordination_check: true,
      },
      record_every_step: true,
      record_device_outputs: document.getElementById('trRecordDeviceOutputs')?.checked !== false,
      events,
    };
  }

  async function runTransientSimulation() {
    setStatus('暂态仿真中...', 'busy');
    if (!await syncToBackend()) { setStatus('同步失败', 'error'); return; }
    const result = await apiPostResult('/api/session/run_transient', transientReadOptions());
    const data = result.data;
    if (result.ok && data && !data.error) {
      _lastTransientData = data;
      showTransientResults(data);
      switchTab('results');
      if (data.success === false) {
        const msg = data.message || '暂态仿真失败';
        log(`暂态仿真失败: ${msg}`, 'error');
        setStatus(msg, 'error');
      } else {
        setStatus('暂态仿真完成');
      }
    } else {
      const msg = result.error || data?.error || '暂态仿真失败';
      log(`暂态仿真失败: ${msg}`, 'error');
      setStatus(msg, 'error');
    }
  }

  function updateTransientPfControls() {
    const enabled = document.getElementById('trPowerFlowInit')?.checked !== false;
    ['trPfMaxIter', 'trPfTol'].forEach(id => {
      const el = document.getElementById(id);
      if (el) el.disabled = !enabled;
    });
  }

  function refreshTransientObserverSelection() {
    if (_lastTransientData) showTransientResults(_lastTransientData);
  }

  function transientDeviceByType(data, pred) {
    return (data?.device_series || []).filter(d => pred(String(d.type || '')));
  }

  function transientTraditionalGenerators(data) {
    return (data?.device_series || []).filter(dev => {
      const type = String(dev.type || '');
      const source = String(dev.source_type || '');
      return type === 'SynchronousMachine' || type === 'ExternalGrid' ||
        type === 'ThreePhaseGenerator' || type === 'ThreePhaseExternalGrid' ||
        source === 'generator' || source === 'external_grid' ||
        source === 'three_phase_generator' || source === 'three_phase_external_grid';
    });
  }

  function transientConverters(data) {
    return (data?.device_series || []).filter(dev => {
      const type = String(dev.type || '');
      return type.includes('GridFollowing') || type.includes('GridForming') ||
        type === 'DCDCConverter' || String(dev.canvas_type || '') === 'vsc';
    });
  }

  function transientMetricTrace(dev, metric, name, color) {
    const y = dev?.values?.[metric];
    if (!Array.isArray(y) || !y.length) return null;
    return {
      x: _lastTransientData?.time_s || [],
      y,
      mode: 'lines',
      name,
      line: color ? { color } : undefined,
      hovertemplate: `${escapeHtml(name)}<br>t=%{x:.4f}s<br>%{y:.4f}<extra></extra>`,
    };
  }

  function transientPlotLayout(title, yTitle = '') {
    return {
      title: { text: title, font: { size: 14 }, x: 0.02, xanchor: 'left' },
      margin: { l: 64, r: 24, t: 44, b: 52 },
      xaxis: { title: 's', automargin: true },
      yaxis: { title: yTitle, automargin: true },
      paper_bgcolor: 'rgba(0,0,0,0)',
      plot_bgcolor: 'rgba(0,0,0,0)',
      font: { color: getComputedStyle(document.documentElement).getPropertyValue('--ink').trim() || '#dcdfe4' },
    };
  }

  function transientPlotLayoutNoAxes(title) {
    const layout = transientPlotLayout(title, '');
    delete layout.xaxis;
    delete layout.yaxis;
    return layout;
  }

  function transientBusVoltageTrace(data, domain, busId, color) {
    const ids = domain === 'DC' ? (data.dc_bus_ids || []) : (data.ac_bus_ids || []);
    const matrix = domain === 'DC' ? (data.dc_voltage_matrix || []) : (data.ac_voltage_matrix || []);
    const pos = ids.findIndex(id => Number(id) === Number(busId));
    if (pos < 0 || !Array.isArray(matrix[pos])) return null;
    return {
      x: data.time_s || [],
      y: matrix[pos],
      mode: 'lines',
      name: `${domain} bus ${busId}`,
      line: color ? { color } : undefined,
      hovertemplate: `${domain} bus ${busId}<br>t=%{x:.4f}s<br>V=%{y:.4f} pu<extra></extra>`,
    };
  }

  function transientObserverInputBuses() {
    const read = id => parsePositiveIntText(document.getElementById(id)?.value || '')
      .filter(v => Number.isFinite(v) && v > 0);
    return {
      AC: read('trObserverAcBuses'),
      DC: read('trObserverDcBuses'),
    };
  }

  function transientObserverRows(data) {
    const rows = [];
    const seen = new Set();
    const add = row => {
      const key = `${row.kind}:${row.domain || ''}:${row.bus || ''}:${row.compKey || ''}`;
      if (seen.has(key)) return;
      seen.add(key);
      rows.push(row);
    };
    const userObservers = transientObserverInputBuses();
    userObservers.AC.forEach(bus => add({
      kind: 'bus',
      name: `AC bus ${bus}`,
      domain: 'AC',
      bus,
      reason: '用户指定',
    }));
    userObservers.DC.forEach(bus => add({
      kind: 'bus',
      name: `DC bus ${bus}`,
      domain: 'DC',
      bus,
      reason: '用户指定',
    }));
    (data?.scheduled_events || []).forEach(event => {
      const type = String(event.type || '');
      if (type === 'FaultShunt' || type === 'ClearFault' || type.includes('LoadScale')) {
        const domain = event.component_type === 'DC' || type === 'DCLoadScale' ? 'DC' : 'AC';
        const bus = Number(event.bus || 0);
        if (bus > 0) {
          add({
            kind: 'bus',
            name: `${domain} bus ${bus}`,
            domain,
            bus,
            reason: transientEventLabel(type),
          });
        }
      }
    });
    const devices = [
      ...transientTraditionalGenerators(data).slice(0, 4),
      ...transientConverters(data).slice(0, 4),
    ];
    devices.forEach(dev => add({
      kind: 'device',
      name: dev.name || `${dev.type || 'device'} ${dev.component_index ?? ''}`,
      domain: dev.component_domain || (String(dev.canvas_type || '').startsWith('dc') ? 'DC' : 'AC'),
      bus: Number(dev.bus || 0),
      compKey: `${dev.type}:${dev.component_index}:${dev.name}`,
      dev,
      reason: dev.type || '',
    }));
    if (!rows.length) {
      (data?.ac_bus_ids || []).slice(0, 4).forEach(bus => add({
        kind: 'bus',
        name: `AC bus ${bus}`,
        domain: 'AC',
        bus,
        reason: '电压观测',
      }));
    }
    return rows.slice(0, 10);
  }

  function transientModelProfileRows(data) {
    const rows = [];
    const seen = new Set();
    (data?.device_series || []).forEach(dev => {
      const profiles = Array.isArray(dev.model_profiles) && dev.model_profiles.length
        ? dev.model_profiles
        : [{
            standard: dev.model_standard || '',
            model_name: dev.model_name || dev.type || '',
            parameter_set: dev.parameter_set || '',
            source_id: '',
            notes: '',
            components: [],
            parameters: {},
          }];
      profiles.forEach(profile => {
        const key = [
          dev.type || '',
          dev.component_index ?? '',
          profile.standard || '',
          profile.model_name || profile.model || '',
          profile.parameter_set || '',
        ].join('|');
        if (seen.has(key)) return;
        seen.add(key);
        rows.push({ dev, profile });
      });
    });
    return rows;
  }

  function transientModelProfileSummary(data) {
    const rows = transientModelProfileRows(data);
    const byStandard = new Map();
    rows.forEach(({ profile }) => {
      const std = profile.standard || 'Native';
      byStandard.set(std, (byStandard.get(std) || 0) + 1);
    });
    return {
      rows,
      byStandard: Array.from(byStandard.entries()).sort((a, b) => b[1] - a[1]),
      total: rows.length,
    };
  }

  function transientRenderModelCompatibility(data) {
    const summary = transientModelProfileSummary(data);
    let html = '<div class="transient-section-head"><h5>动态模型 / 标准兼容性</h5><span>来自统一 JSON dynamic_model 与标准映射</span></div>';
    html += '<div class="transient-model-summary">';
    if (summary.byStandard.length) {
      summary.byStandard.forEach(([std, count]) => {
        html += `<span><strong>${escapeHtml(std)}</strong>${count}</span>`;
      });
    } else {
      html += '<span><strong>Native</strong>0</span>';
    }
    html += '</div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>设备</th><th>标准</th><th>模型</th><th>参数集</th><th>组件控制块</th><th>参数数</th></tr></thead><tbody>';
    if (summary.rows.length) {
      summary.rows.slice(0, 48).forEach(({ dev, profile }) => {
        const comps = Array.isArray(profile.components) ? profile.components : [];
        const compText = comps.length
          ? comps.map(c => `${c.type || ''}:${c.model || c.model_name || ''}`.replace(/^:/, '')).filter(Boolean).join(', ')
          : (profile.profile || '');
        const paramCount = Object.keys(profile.parameters || {}).length +
          comps.reduce((sum, c) => sum + Object.keys(c.parameters || {}).length, 0);
        html += `<tr>
          <td>${escapeHtml(dev.name || `${dev.type || 'device'} ${dev.component_index ?? ''}`)}</td>
          <td>${escapeHtml(profile.standard || dev.model_standard || 'Native')}</td>
          <td>${escapeHtml(profile.model_name || profile.model || dev.model_name || dev.type || '')}</td>
          <td>${escapeHtml(profile.parameter_set || dev.parameter_set || '')}</td>
          <td>${escapeHtml(compText || '—')}</td>
          <td>${paramCount}</td>
        </tr>`;
      });
    } else {
      html += '<tr><td colspan="6">暂无动态模型配置。可在元件属性中填写 dynamic_model JSON。</td></tr>';
    }
    html += '</tbody></table></div>';
    return html;
  }

  function transientCompatibilityRatio(value, total) {
    const n = Number(value);
    const t = Number(total);
    if (!Number.isFinite(n) || !Number.isFinite(t) || t <= 0) return '—';
    return `${((n / t) * 100).toFixed(1)}%`;
  }

  function modelIoCountBy(rows, key) {
    const out = {};
    (rows || []).forEach(row => {
      const value = row?.[key] || '未分类';
      out[value] = (out[value] || 0) + 1;
    });
    return out;
  }

  function modelIoRangeText(row) {
    const min = row?.expected_min ?? row?.min;
    const max = row?.expected_max ?? row?.max;
    const unit = row?.units ? ` ${row.units}` : '';
    if (min != null && max != null) {
      return `${row.min_inclusive === false ? '(' : '['}${min}, ${max}${row.max_inclusive === false ? ')' : ']'}${unit}`;
    }
    if (min != null) return `${row.min_inclusive === false ? '>' : '≥'} ${min}${unit}`;
    if (max != null) return `${row.max_inclusive === false ? '<' : '≤'} ${max}${unit}`;
    return row?.required ? '必填' : '—';
  }

  function modelIoSeverityClass(severity) {
    const s = String(severity || '').toLowerCase();
    if (s === 'error') return 'model-io-severity-error';
    if (s === 'warning') return 'model-io-severity-warning';
    return 'model-io-severity-info';
  }

  function modelIoDimensionLabel(dimension) {
    const labels = {
      AssetIdentity: '资产身份',
      TopologyConnectivity: '拓扑连通',
      ElectricalParameters: '电气参数',
      DynamicBehavior: '动态模型',
      TelemetryObservability: '遥测观测',
      StateSynchronization: '状态同步',
      ScenarioEvents: '场景事件',
      ReliabilityLifecycle: '可靠性生命周期',
      StandardsInteroperability: '标准互操作',
      NumericalValidation: '数值验证',
      ProvenanceGovernance: '来源治理',
    };
    return labels[dimension] || dimension || '未分类';
  }

  function modelIoRatioPercent(value, digits = 0) {
    const pct = Math.max(0, Math.min(1, Number(value || 0))) * 100;
    return `${pct.toFixed(digits)}%`;
  }

  function modelIoStatusBadge(value, passText = '通过', failText = '未通过') {
    const ok = value === true || value === 'true' || value === 'pass' || value === 'passed';
    const cls = ok ? 'model-io-badge-pass' : 'model-io-badge-fail';
    return `<span class="model-io-badge ${cls}">${escapeHtml(ok ? passText : failText)}</span>`;
  }

  function modelIoPolicyCounts(mappings, field) {
    const counts = {};
    (mappings || []).forEach(row => {
      const key = row?.[field] || 'Unknown';
      counts[key] = (counts[key] || 0) + 1;
    });
    return counts;
  }

  function modelIoPolicyCountsText(counts) {
    const order = ['Exact', 'Equivalent', 'Projected', 'Aggregated', 'BoundaryInjection', 'InternalOnly', 'DiagnosticOnly', 'Unsupported', 'Unknown'];
    const parts = order
      .filter(key => Number(counts?.[key] || 0) > 0)
      .map(key => `${key}:${counts[key]}`);
    return parts.length ? parts.join(' / ') : '—';
  }

  function drawModelIoCharts(data) {
    if (typeof Plotly === 'undefined') return;
    const summary = data?.summary || {};
    const total = Number(summary.total_instances || 0);
    const gridRepresented = Number(summary.gridlabd_represented || 0);
    const dssRepresented = Number(summary.opendss_represented || 0);
    const audit = data?.parameter_audit || {};
    const auditSummary = audit.summary || {};
    const rules = Array.isArray(data?.parameter_rules) ? data.parameter_rules : [];
    const findings = Array.isArray(audit.findings) ? audit.findings : [];
    const twin = data?.digital_twin_readiness || {};
    const twinSummary = twin.summary || {};
    const twinDimensions = Array.isArray(twin.dimensions) ? twin.dimensions : [];
    const cfg = { responsive: true, displaylogo: false, modeBarButtonsToRemove: ['select2d', 'lasso2d'] };

    const twinGaugeChart = document.getElementById('modelIoTwinGaugeChart');
    if (twinGaugeChart) {
      const readiness = Math.max(0, Math.min(1, Number(twinSummary.readiness_ratio || 0))) * 100;
      Plotly.react(twinGaugeChart, [{
        type: 'indicator',
        mode: 'gauge+number',
        value: readiness,
        number: { suffix: '%', font: { size: 34 } },
        title: { text: `${twinSummary.maturity_label || '未评估'}` },
        gauge: {
          axis: { range: [0, 100], tickwidth: 1 },
          bar: { color: '#56b6c2' },
          bgcolor: 'rgba(255,255,255,0.04)',
          borderwidth: 1,
          bordercolor: '#4b5563',
          steps: [
            { range: [0, 35], color: 'rgba(224,108,117,0.25)' },
            { range: [35, 55], color: 'rgba(209,154,102,0.24)' },
            { range: [55, 72], color: 'rgba(229,192,123,0.24)' },
            { range: [72, 88], color: 'rgba(152,195,121,0.22)' },
            { range: [88, 100], color: 'rgba(86,182,194,0.26)' },
          ],
          threshold: { line: { color: '#e06c75', width: 3 }, thickness: 0.75, value: 88 },
        },
      }], {
        ...transientPlotLayoutNoAxes('数字孪生成熟度'),
        margin: { l: 28, r: 28, t: 48, b: 28 },
      }, cfg);
    }

    const twinDimensionChart = document.getElementById('modelIoTwinDimensionChart');
    if (twinDimensionChart) {
      if (twinDimensions.length) {
        const labels = twinDimensions.map(row => modelIoDimensionLabel(row.dimension));
        const values = twinDimensions.map(row => {
          const max = Number(row.max_score || 0);
          return max > 0 ? 100 * Number(row.score || 0) / max : 0;
        });
        Plotly.react(twinDimensionChart, [{
          x: values,
          y: labels,
          type: 'bar',
          orientation: 'h',
          marker: {
            color: values.map(v => v >= 88 ? '#56b6c2' : v >= 72 ? '#98c379' : v >= 55 ? '#e5c07b' : v >= 35 ? '#d19a66' : '#e06c75'),
          },
          text: values.map(v => `${v.toFixed(0)}%`),
          textposition: 'auto',
        }], {
          ...transientPlotLayout('孪生维度就绪度', '%'),
          xaxis: { range: [0, 100], title: '', automargin: true },
          yaxis: { automargin: true },
          margin: { l: 112, r: 18, t: 42, b: 34 },
        }, cfg);
      } else {
        twinDimensionChart.innerHTML = '<p class="empty-hint">当前尚无数字孪生维度评分。</p>';
      }
    }

    const coverageChart = document.getElementById('modelIoCoverageChart');
    if (coverageChart) {
      Plotly.react(coverageChart, [{
        x: ['GridLAB-D', 'OpenDSS'],
        y: [gridRepresented, dssRepresented],
        name: '可表示',
        type: 'bar',
        marker: { color: '#56b6c2' },
      }, {
        x: ['GridLAB-D', 'OpenDSS'],
        y: [Math.max(total - gridRepresented, 0), Math.max(total - dssRepresented, 0)],
        name: '需投影/暂不支持',
        type: 'bar',
        marker: { color: '#d19a66' },
      }], {
        ...transientPlotLayout('外部格式覆盖', '元件数'),
        xaxis: { title: '', automargin: true },
        barmode: 'stack',
      }, cfg);
    }

    const severityChart = document.getElementById('modelIoSeverityChart');
    if (severityChart) {
      const severity = {
        Error: Number(auditSummary.errors || 0),
        Warning: Number(auditSummary.warnings || 0),
        Info: Number(auditSummary.info || 0),
      };
      if (Object.values(severity).some(v => v > 0)) {
        Plotly.react(severityChart, [{
          labels: Object.keys(severity),
          values: Object.values(severity),
          type: 'pie',
          hole: 0.48,
          marker: { colors: ['#e06c75', '#d19a66', '#61afef'] },
          textinfo: 'label+value',
        }], {
          ...transientPlotLayoutNoAxes('参数健康等级'),
          showlegend: false,
        }, cfg);
      } else {
        severityChart.innerHTML = '<p class="empty-hint">当前无参数诊断项。</p>';
      }
    }

    const categoryChart = document.getElementById('modelIoCategoryChart');
    if (categoryChart) {
      const byCategory = auditSummary.by_category || modelIoCountBy(findings, 'category');
      const categoryValues = Object.values(byCategory).map(Number);
      if (categoryValues.some(v => v > 0)) {
        Plotly.react(categoryChart, [{
          x: Object.keys(byCategory),
          y: categoryValues,
          type: 'bar',
          marker: { color: ['#61afef', '#98c379', '#c678dd', '#e06c75', '#e5c07b'] },
        }], {
          ...transientPlotLayout('问题所属参数域', '发现数'),
          xaxis: { title: '', automargin: true },
        }, cfg);
      } else {
        categoryChart.innerHTML = '<p class="empty-hint">当前无越界参数域。</p>';
      }
    }

    const standardsChart = document.getElementById('modelIoStandardsChart');
    if (standardsChart) {
      const byStandard = modelIoCountBy(rules, 'standard_family');
      Plotly.react(standardsChart, [{
        labels: Object.keys(byStandard),
        values: Object.values(byStandard),
        type: 'pie',
        hole: 0.38,
        textinfo: 'label+value',
      }], {
        ...transientPlotLayoutNoAxes('规则标准来源'),
        showlegend: false,
      }, cfg);
    }
  }

  function renderModelCompatibilityReport(data, options = {}) {
    const targetId = options.targetId || 'modelIoResults';
    const resultGroup = options.resultGroup || 'modelIO';
    const title = options.title || '模型兼容性检查';
    const subtitle = options.subtitle || '统一 IO registry / 当前画布系统';
    const summary = data?.summary || {};
    const coverage = Array.isArray(data?.coverage) ? data.coverage : [];
    const mappings = Array.isArray(data?.mappings) ? data.mappings : [];
    const diagnostics = data?.diagnostics || {};
    const parameterRules = Array.isArray(data?.parameter_rules) ? data.parameter_rules : [];
    const parameterAudit = data?.parameter_audit || {};
    const auditSummary = parameterAudit.summary || {};
    const findings = Array.isArray(parameterAudit.findings) ? parameterAudit.findings : [];
    const twinCriteria = Array.isArray(data?.digital_twin_criteria) ? data.digital_twin_criteria : [];
    const twin = data?.digital_twin_readiness || {};
    const twinSummary = twin.summary || {};
    const twinFindings = Array.isArray(twin.findings) ? twin.findings : [];
    const twinGates = Array.isArray(twin.gates) ? twin.gates : [];
    const roundTripRows = Array.isArray(twin.round_trip) ? twin.round_trip : [];
    const twinDimensions = Array.isArray(twin.dimensions) ? twin.dimensions : [];
    const total = Number(summary.total_instances || 0);
    const gridRepresented = Number(summary.gridlabd_represented || 0);
    const openDssRepresented = Number(summary.opendss_represented || 0);
    const gridUnrepresented = Number(summary.gridlabd_unrepresented || 0);
    const openDssUnrepresented = Number(summary.opendss_unrepresented || 0);
    const errors = Number(auditSummary.errors || 0);
    const warnings = Number(auditSummary.warnings || 0);
    const infos = Number(auditSummary.info || 0);
    const twinReadiness = Number(twinSummary.readiness_ratio || 0);
    const twinErrors = Number(twinSummary.errors || 0);
    const twinWarnings = Number(twinSummary.warnings || 0);

    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup(resultGroup);

    let html = `<div class="transient-section-head"><h5>${escapeHtml(title)}</h5><span>${escapeHtml(subtitle)}</span></div>`;
    html += '<div class="transient-kpi-grid">';
    [
      ['当前元件实例', total, '个'],
      ['GridLAB-D覆盖', transientCompatibilityRatio(gridRepresented, total), `${gridRepresented}/${total}`],
      ['OpenDSS覆盖', transientCompatibilityRatio(openDssRepresented, total), `${openDssRepresented}/${total}`],
      ['注册元件类型', mappings.length, '类'],
      ['当前覆盖类型', coverage.length, '类'],
      ['参数规则', parameterRules.length, '条'],
      ['参数硬错误', errors, `警告 ${warnings} / 提示 ${infos}`],
      ['已检查参数', auditSummary.checked_parameters || 0, `元件 ${auditSummary.component_instances_checked || 0}`],
      ['孪生就绪度', modelIoRatioPercent(twinReadiness), twinSummary.maturity_label || '未评估'],
      ['孪生成熟度', `L${twinSummary.maturity_level ?? 0}`, `缺口 ${twinErrors}/${twinWarnings}`],
    ].forEach(([label, value, unit]) => {
      html += `<div class="transient-kpi"><div class="transient-kpi-label">${escapeHtml(label)}</div><div class="transient-kpi-value">${escapeHtml(String(value))}</div><div class="transient-kpi-label">${escapeHtml(String(unit || ''))}</div></div>`;
    });
    html += '</div>';

    html += '<div class="model-io-dashboard-grid">';
    html += '<div id="modelIoTwinGaugeChart" class="model-io-chart"></div>';
    html += '<div id="modelIoTwinDimensionChart" class="model-io-chart model-io-chart-tall"></div>';
    html += '<div id="modelIoCoverageChart" class="model-io-chart"></div>';
    html += '<div id="modelIoSeverityChart" class="model-io-chart"></div>';
    html += '<div id="modelIoCategoryChart" class="model-io-chart"></div>';
    html += '<div id="modelIoStandardsChart" class="model-io-chart"></div>';
    html += '</div>';

    html += '<div class="transient-section-head"><h5>成熟度门槛与 IO 证据</h5><span>弱链门槛 / round-trip conformance</span></div>';
    html += '<div class="model-io-split-grid">';
    html += '<div class="model-io-panel"><div class="model-io-panel-title">Fidelity / Integration Gates</div>';
    html += '<div class="transient-table-scroll model-io-compact-scroll"><table><thead><tr><th>门槛</th><th>轴</th><th>等级</th><th>状态</th><th>证据</th></tr></thead><tbody>';
    if (twinGates.length) {
      twinGates.forEach(g => {
        html += `<tr>
          <td>${escapeHtml(g.gate_id || '')}</td>
          <td>${escapeHtml(g.axis || '')}</td>
          <td>${escapeHtml(String(g.level ?? ''))}</td>
          <td>${modelIoStatusBadge(g.passed)}</td>
          <td>${escapeHtml(g.evidence || g.title || '')}</td>
        </tr>`;
      });
    } else {
      html += '<tr><td colspan="5">暂无门槛证据；请先加载或同步系统。</td></tr>';
    }
    html += '</tbody></table></div></div>';

    html += '<div class="model-io-panel"><div class="model-io-panel-title">Round-trip / Adapter Evidence</div>';
    html += '<div class="transient-table-scroll model-io-compact-scroll"><table><thead><tr><th>适配器</th><th>层级</th><th>一致性</th><th>保真</th><th>字段</th></tr></thead><tbody>';
    if (roundTripRows.length) {
      roundTripRows.forEach(rt => {
        const checked = Number(rt.fields_checked || 0);
        const mismatched = Number(rt.fields_mismatched || 0);
        html += `<tr>
          <td>${escapeHtml(rt.adapter || '')}</td>
          <td>${escapeHtml(rt.level || '')}</td>
          <td>${modelIoStatusBadge(rt.passed)}</td>
          <td>${modelIoStatusBadge(rt.lossless, '无损', '有投影损失')}</td>
          <td>${escapeHtml(`${checked - mismatched}/${checked} matched`)}</td>
        </tr>`;
      });
    } else {
      html += '<tr><td colspan="5">暂无存档 round-trip 证据。</td></tr>';
    }
    html += '</tbody></table></div></div>';
    html += '</div>';

    const adapterRows = [
      ['Internal JSON', 'json_policy', `${total}/${total}`, 'rich binding'],
      ['Canonical Model', 'canonical_policy', `${mappings.length} registered`, 'canonical projection'],
      ['GridLAB-D', 'gridlabd_policy', `${gridRepresented}/${gridRepresented + gridUnrepresented || total}`, 'external snapshot'],
      ['OpenDSS', 'opendss_policy', `${openDssRepresented}/${openDssRepresented + openDssUnrepresented || total}`, 'external snapshot'],
    ];
    html += '<div class="transient-section-head"><h5>适配器能力矩阵</h5><span>registry policy / 当前实例覆盖</span></div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>格式</th><th>绑定</th><th>Registry策略分布</th><th>当前覆盖</th></tr></thead><tbody>';
    adapterRows.forEach(([label, field, current, binding]) => {
      html += `<tr>
        <td>${escapeHtml(label)}</td>
        <td>${escapeHtml(binding)}</td>
        <td>${escapeHtml(modelIoPolicyCountsText(modelIoPolicyCounts(mappings, field)))}</td>
        <td>${escapeHtml(current)}</td>
      </tr>`;
    });
    html += '</tbody></table></div>';

    const telemetryDims = new Set(['TelemetryObservability', 'StateSynchronization', 'ScenarioEvents', 'ReliabilityLifecycle', 'ProvenanceGovernance', 'NumericalValidation']);
    const telemetryRows = twinDimensions.filter(row => telemetryDims.has(row.dimension));
    html += '<div class="transient-section-head"><h5>遥测 / 校准 / 验证准备</h5><span>I1 / I2 相关维度</span></div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>维度</th><th>得分</th><th>发现数</th><th>主要缺口</th></tr></thead><tbody>';
    if (telemetryRows.length) {
      telemetryRows.forEach(dim => {
        const maxScore = Number(dim.max_score || 0);
        const pct = maxScore > 0 ? `${(100 * Number(dim.score || 0) / maxScore).toFixed(0)}%` : '—';
        const gaps = twinFindings
          .filter(f => f.dimension === dim.dimension)
          .slice(0, 2)
          .map(f => `${f.criterion_id || ''} ${f.message || f.title || ''}`.trim())
          .filter(Boolean)
          .map(escapeHtml)
          .join('<br>');
        html += `<tr>
          <td>${escapeHtml(modelIoDimensionLabel(dim.dimension))}</td>
          <td>${escapeHtml(pct)}</td>
          <td>${escapeHtml(String(dim.findings ?? 0))}</td>
          <td>${gaps || '—'}</td>
        </tr>`;
      });
    } else {
      html += '<tr><td colspan="4">当前尚无 I1/I2 相关维度证据。</td></tr>';
    }
    html += '</tbody></table></div>';

    const diagSummaryRows = [
      ['GridLAB-D', Array.isArray(diagnostics.gridlabd) ? diagnostics.gridlabd.length : 0],
      ['OpenDSS', Array.isArray(diagnostics.opendss) ? diagnostics.opendss.length : 0],
    ];
    html += '<div class="transient-section-head"><h5>导入诊断契约</h5><span>records / summary / binding level / unit assertion</span></div>';
    html += '<div class="model-io-contract-grid">';
    html += `<div class="model-io-contract-item"><strong>records</strong><span>${escapeHtml(diagSummaryRows.map(([k, n]) => `${k}:${n}`).join(' / '))}</span></div>`;
    html += `<div class="model-io-contract-item"><strong>summary</strong><span>${escapeHtml(`参数错误 ${errors} / 警告 ${warnings} / 提示 ${infos}`)}</span></div>`;
    html += `<div class="model-io-contract-item"><strong>binding level</strong><span>${escapeHtml(`F${twinSummary.fidelity_level ?? 0} / I${twinSummary.integration_level ?? 0}`)}</span></div>`;
    html += `<div class="model-io-contract-item"><strong>unit assertion</strong><span>${escapeHtml(`${parameterRules.filter(r => r.units).length}/${parameterRules.length} rules with units`)}</span></div>`;
    html += '</div>';

    html += '<div class="transient-section-head"><h5>数字孪生就绪诊断</h5><span>identity / topology / parameters / dynamics / telemetry / validation / governance</span></div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>等级</th><th>维度</th><th>准则</th><th>得分</th><th>标准依据</th><th>证据</th><th>改进方向</th></tr></thead><tbody>';
    if (twinFindings.length) {
      twinFindings.forEach(row => {
        const maxScore = Number(row.max_score || 0);
        const scorePct = maxScore > 0 ? `${(100 * Number(row.score || 0) / maxScore).toFixed(0)}%` : '—';
        const basis = `${row.standard_family || ''}${row.standard_profile ? ` / ${row.standard_profile}` : ''}`.replace(/^ \/ /, '');
        html += `<tr>
          <td><span class="model-io-severity ${modelIoSeverityClass(row.severity)}">${escapeHtml(row.severity || '')}</span></td>
          <td>${escapeHtml(modelIoDimensionLabel(row.dimension))}</td>
          <td>${escapeHtml(row.criterion_id || '')}<br>${escapeHtml(row.title || '')}</td>
          <td>${escapeHtml(scorePct)}</td>
          <td>${escapeHtml(basis || 'Native')}</td>
          <td>${escapeHtml(row.evidence || '')}</td>
          <td>${escapeHtml(row.message || '')}</td>
        </tr>`;
      });
    } else {
      html += '<tr><td colspan="7">当前尚未加载系统；请先加载算例或同步画布。</td></tr>';
    }
    html += '</tbody></table></div>';

    html += '<div class="transient-section-head"><h5>当前系统元件覆盖</h5><span>JSON / Canonical / GridLAB-D / OpenDSS</span></div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>元件</th><th>路径</th><th>数量</th><th>JSON</th><th>Canonical</th><th>GridLAB-D</th><th>OpenDSS</th><th>验证范围</th><th>标准模型</th></tr></thead><tbody>';
    if (coverage.length) {
      coverage.forEach(row => {
        const profiles = Array.isArray(row.standard_profiles) ? row.standard_profiles : [];
        const profileText = profiles.map(p => `${p.family || ''}:${p.model_name || p.profile || ''}`.replace(/^:/, '')).filter(Boolean).join(', ');
        html += `<tr>
          <td>${escapeHtml(row.component_type || '')}</td>
          <td>${escapeHtml(row.collection_path || '')}</td>
          <td>${escapeHtml(String(row.count ?? 0))}</td>
          <td>${escapeHtml(row.json_policy || '')}</td>
          <td>${escapeHtml(row.canonical_policy || '')}</td>
          <td>${escapeHtml(row.gridlabd_policy || '')}</td>
          <td>${escapeHtml(row.opendss_policy || '')}</td>
          <td>${escapeHtml(row.verification_scope || '')}</td>
          <td>${escapeHtml(profileText || '—')}</td>
        </tr>`;
      });
    } else {
      html += '<tr><td colspan="9">当前后端尚未加载系统；请先加载算例或同步画布。</td></tr>';
    }
    html += '</tbody></table></div>';

    const diagGroups = [
      ['GridLAB-D', Array.isArray(diagnostics.gridlabd) ? diagnostics.gridlabd : []],
      ['OpenDSS', Array.isArray(diagnostics.opendss) ? diagnostics.opendss : []],
    ];
    html += '<div class="transient-section-head"><h5>外部工具诊断</h5><span>Unsupported / Internal-only / Diagnostic-only 会列出</span></div>';
    html += '<div class="transient-mini-grid">';
    diagGroups.forEach(([name, rows]) => {
      html += `<div class="transient-mini-card"><strong>${escapeHtml(name)}</strong><span>${rows.length ? rows.map(escapeHtml).join('<br>') : '当前系统内元件均有可表示策略。'}</span></div>`;
    });
    html += '</div>';

    html += '<div class="transient-section-head"><h5>参数健康诊断</h5><span>static / dynamic / transient / failure / reliability</span></div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>等级</th><th>元件</th><th>位置</th><th>参数域</th><th>参数</th><th>当前值</th><th>建议范围</th><th>标准依据</th><th>说明</th></tr></thead><tbody>';
    if (findings.length) {
      findings.slice(0, 300).forEach(row => {
        const component = row.component_name || `${row.component_type || ''} ${row.component_index ?? row.component_position ?? ''}`;
        const valueText = row.value == null ? '—' : String(row.value);
        const basis = `${row.standard_family || ''}${row.standard_profile ? ` / ${row.standard_profile}` : ''}`.replace(/^ \/ /, '');
        html += `<tr>
          <td><span class="model-io-severity ${modelIoSeverityClass(row.severity)}">${escapeHtml(row.severity || '')}</span></td>
          <td>${escapeHtml(component)}</td>
          <td>${escapeHtml(row.collection_path || '')}${row.component_position != null ? ` #${escapeHtml(String(row.component_position))}` : ''}</td>
          <td>${escapeHtml(row.category || '')}</td>
          <td>${escapeHtml(row.parameter_path || '')}</td>
          <td>${escapeHtml(valueText)}</td>
          <td>${escapeHtml(modelIoRangeText(row))}</td>
          <td>${escapeHtml(basis || 'Native')}</td>
          <td>${escapeHtml(row.message || '')}</td>
        </tr>`;
      });
      if (findings.length > 300) {
        html += `<tr><td colspan="9">仅显示前 300 条；总计 ${escapeHtml(String(findings.length))} 条。</td></tr>`;
      }
    } else {
      html += '<tr><td colspan="9">当前已检查参数没有发现越界或缺失项。</td></tr>';
    }
    html += '</tbody></table></div>';

    const profileRows = [];
    mappings.forEach(mapping => {
      (Array.isArray(mapping.standard_profiles) ? mapping.standard_profiles : []).forEach(profile => {
        profileRows.push({ mapping, profile });
      });
    });
    html += '<div class="transient-section-head"><h5>标准模型注册表</h5><span>行业标准与本模块 rich/canonical 映射方向</span></div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>元件类型</th><th>标准族</th><th>Profile</th><th>模型名</th><th>策略</th><th>验证</th><th>说明</th></tr></thead><tbody>';
    if (profileRows.length) {
      profileRows.forEach(({ mapping, profile }) => {
        html += `<tr>
          <td>${escapeHtml(mapping.component_type || '')}</td>
          <td>${escapeHtml(profile.family || '')}</td>
          <td>${escapeHtml(profile.profile || '')}</td>
          <td>${escapeHtml(profile.model_name || '')}</td>
          <td>${escapeHtml(profile.policy || '')}</td>
          <td>${escapeHtml(profile.verification_scope || '')}</td>
          <td>${escapeHtml(profile.notes || '')}</td>
        </tr>`;
      });
    } else {
      html += '<tr><td colspan="7">尚未注册行业标准 profile。</td></tr>';
    }
    html += '</tbody></table></div>';

    const ruleRows = parameterRules.slice(0, 360);
    html += '<div class="transient-section-head"><h5>参数规则库</h5><span>可用于导入校验、参数估计和标准兼容审查</span></div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>元件</th><th>路径</th><th>参数域</th><th>参数</th><th>范围</th><th>必填</th><th>标准族</th><th>Profile</th><th>等级</th></tr></thead><tbody>';
    if (ruleRows.length) {
      ruleRows.forEach(rule => {
        html += `<tr>
          <td>${escapeHtml(rule.component_type || '')}</td>
          <td>${escapeHtml(rule.collection_path || '')}</td>
          <td>${escapeHtml(rule.category || '')}</td>
          <td>${escapeHtml(rule.parameter_path || '')}</td>
          <td>${escapeHtml(modelIoRangeText(rule))}</td>
          <td>${rule.required ? '是' : '否'}</td>
          <td>${escapeHtml(rule.standard_family || '')}</td>
          <td>${escapeHtml(rule.standard_profile || '')}</td>
          <td>${escapeHtml(rule.range_severity || rule.missing_severity || '')}</td>
        </tr>`;
      });
      if (parameterRules.length > ruleRows.length) {
        html += `<tr><td colspan="9">仅显示前 ${ruleRows.length} 条；总计 ${escapeHtml(String(parameterRules.length))} 条。</td></tr>`;
      }
    } else {
      html += '<tr><td colspan="9">尚未注册参数规则。</td></tr>';
    }
    html += '</tbody></table></div>';

    html += '<div class="transient-section-head"><h5>数字孪生准则库</h5><span>Data IO 从文件交换走向运行孪生的统一验收口径</span></div>';
    html += '<div class="transient-table-scroll"><table><thead><tr><th>准则</th><th>维度</th><th>权重</th><th>失败等级</th><th>标准/Profile</th><th>说明</th></tr></thead><tbody>';
    if (twinCriteria.length) {
      twinCriteria.forEach(row => {
        const basis = `${row.standard_family || ''}${row.standard_profile ? ` / ${row.standard_profile}` : ''}`.replace(/^ \/ /, '');
        html += `<tr>
          <td>${escapeHtml(row.criterion_id || '')}<br>${escapeHtml(row.title || '')}</td>
          <td>${escapeHtml(modelIoDimensionLabel(row.dimension))}</td>
          <td>${escapeHtml(String(row.weight ?? ''))}</td>
          <td>${escapeHtml(row.severity_if_failed || '')}</td>
          <td>${escapeHtml(basis || 'Native')}</td>
          <td>${escapeHtml(row.description || '')}</td>
        </tr>`;
      });
    } else {
      html += '<tr><td colspan="6">尚未注册数字孪生准则。</td></tr>';
    }
    html += '</tbody></table></div>';

    const el = document.getElementById(targetId);
    if (el) {
      el.innerHTML = html;
      drawModelIoCharts(data);
      requestAnimationFrame(() => drawModelIoCharts(data));
      setTimeout(() => drawModelIoCharts(data), 250);
    }
  }

  function renderTransientCompatibilityReport(data) {
    renderModelCompatibilityReport(data, {
      targetId: 'modelIoResults',
      resultGroup: 'modelIO',
      title: '暂态模型兼容性检查',
      subtitle: '从暂态仿真入口触发，报告集中到模型IO',
    });
  }

  async function runModelCompatibility(options = {}) {
    setStatus('模型兼容性检查中...', 'busy');
    if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
    const data = await apiGet('/api/io/model_compatibility');
    if (data && !data.error) {
      switchTab('results');
      renderModelCompatibilityReport(data, options);
      setStatus('模型兼容性检查完成');
    } else {
      const msg = data?.error || '模型兼容性检查失败';
      log(msg, 'error');
      setStatus(msg, 'error');
    }
  }

  async function runTransientCompatibility() {
    await runModelCompatibility({
      targetId: 'modelIoResults',
      resultGroup: 'modelIO',
      title: '暂态模型兼容性检查',
      subtitle: '动态模型 / rich-canonical / GridLAB-D / OpenDSS 兼容性',
    });
  }

  function drawTransientDashboard(data) {
    if (typeof Plotly === 'undefined') return;
    const tab = document.getElementById('tabResults');
    if (tab && !tab.classList.contains('active')) {
      return;
    }
    const t = data.time_s || [];
    const cfg = {
      responsive: true,
      displaylogo: false,
      modeBarButtonsToRemove: ['select2d', 'lasso2d'],
    };
    const voltageChart = document.getElementById('trVoltageChart');
    if (voltageChart) {
      Plotly.react(voltageChart, [
        { x: t, y: data.max_ac_voltage_pu || [], mode: 'lines', name: 'AC max', line: { color: '#61afef' } },
        { x: t, y: data.min_ac_voltage_pu || [], mode: 'lines', name: 'AC min', line: { color: '#e06c75' } },
        { x: t, y: data.max_dc_voltage_pu || [], mode: 'lines', name: 'DC max', line: { color: '#98c379', dash: 'dot' } },
        { x: t, y: data.min_dc_voltage_pu || [], mode: 'lines', name: 'DC min', line: { color: '#d19a66', dash: 'dot' } },
      ], transientPlotLayout('电压包络', 'p.u.'), cfg);
    }
    const acHeat = document.getElementById('trAcHeatmap');
    if (acHeat && Array.isArray(data.ac_voltage_matrix) && data.ac_voltage_matrix.length) {
      Plotly.react(acHeat, [{
        z: data.ac_voltage_matrix,
        x: t,
        y: data.ac_voltage_matrix.map((_, i) => `n${i + 1}`),
        type: 'heatmap',
        colorscale: 'Viridis',
        colorbar: { title: 'p.u.' },
      }], transientPlotLayout('三相节点电压热图', ''), cfg);
    }
    const freqChart = document.getElementById('trFreqChart');
    const gfl = transientDeviceByType(data, type => type.includes('GridFollowing'));
    const gfm = transientDeviceByType(data, type => type.includes('GridForming'));
    const gens = transientTraditionalGenerators(data);
    const converters = transientConverters(data);
    const freqTraces = [];
    gfl.slice(0, 4).forEach((dev, i) => {
      const tr = transientMetricTrace(dev, 'pll_frequency_hz', `${dev.name} PLL`, ['#61afef', '#56b6c2', '#c678dd', '#e5c07b'][i % 4]);
      if (tr) freqTraces.push(tr);
    });
    gfm.slice(0, 4).forEach((dev, i) => {
      const tr = transientMetricTrace(dev, 'frequency_hz', `${dev.name} GFM`, ['#98c379', '#d19a66', '#e06c75', '#7c3aed'][i % 4]);
      if (tr) freqTraces.push(tr);
    });
    if (freqChart) {
      if (freqTraces.length) Plotly.react(freqChart, freqTraces, transientPlotLayout('GFL/GFM 频率轨迹', 'Hz'), cfg);
      else freqChart.innerHTML = '<p class="empty-hint">无 GFL/GFM 频率轨迹</p>';
    }
    const powerChart = document.getElementById('trPowerChart');
    const pTraces = [];
    converters.slice(0, 6).forEach((dev, i) => {
      const tr = transientMetricTrace(dev, 'p_mw', `${dev.name} P`, ['#61afef', '#98c379', '#d19a66', '#e06c75', '#c678dd', '#56b6c2'][i % 6]);
      if (tr) pTraces.push(tr);
    });
    if (powerChart) {
      if (pTraces.length) Plotly.react(powerChart, pTraces, transientPlotLayout('逆变器功率轨迹', 'MW'), cfg);
      else powerChart.innerHTML = '<p class="empty-hint">无逆变器功率轨迹</p>';
    }
    const currentChart = document.getElementById('trCurrentChart');
    const iTraces = [];
    gfl.slice(0, 4).forEach((dev, i) => {
      const tr = transientMetricTrace(dev, 'i_mag_pu', `${dev.name} |I|`, ['#61afef', '#56b6c2', '#c678dd', '#e5c07b'][i % 4]);
      if (tr) iTraces.push(tr);
    });
    gfm.slice(0, 4).forEach((dev, i) => {
      const tr = transientMetricTrace(dev, 'i_rms_pu', `${dev.name} Irms`, ['#98c379', '#d19a66', '#e06c75', '#7c3aed'][i % 4]);
      if (tr) iTraces.push(tr);
    });
    if (currentChart) {
      if (iTraces.length) Plotly.react(currentChart, iTraces, transientPlotLayout('逆变器电流轨迹', 'p.u.'), cfg);
      else currentChart.innerHTML = '<p class="empty-hint">无逆变器电流轨迹</p>';
    }
    const dcLinkChart = document.getElementById('trDcLinkChart');
    if (dcLinkChart) {
      const dcLinkTraces = converters.slice(0, 6).map((dev, i) =>
        transientMetricTrace(dev, 'vdc_link_pu', `${dev.name} Vdc`, ['#56b6c2', '#98c379', '#d19a66', '#c678dd', '#61afef', '#e06c75'][i % 6])
      ).filter(Boolean);
      if (dcLinkTraces.length) Plotly.react(dcLinkChart, dcLinkTraces, transientPlotLayout('动态 DC 链电压轨迹', 'p.u.'), cfg);
      else dcLinkChart.innerHTML = '<p class="empty-hint">未启用动态 DC 链</p>';
    }
    const genFreqChart = document.getElementById('trGenFreqChart');
    if (genFreqChart) {
      const traces = gens.slice(0, 6).map((dev, i) =>
        transientMetricTrace(dev, 'frequency_hz', `${dev.name} f`, ['#e06c75', '#d19a66', '#98c379', '#61afef', '#c678dd', '#56b6c2'][i % 6])
      ).filter(Boolean);
      if (traces.length) Plotly.react(genFreqChart, traces, transientPlotLayout('传统同步机/外部电网频率轨迹', 'Hz'), cfg);
      else genFreqChart.innerHTML = '<p class="empty-hint">无传统同步机/外部电网频率轨迹</p>';
    }
    const genPowerChart = document.getElementById('trGenPowerChart');
    if (genPowerChart) {
      const traces = [];
      gens.slice(0, 4).forEach((dev, i) => {
        const p = transientMetricTrace(dev, 'p_mw', `${dev.name} Pe`, ['#61afef', '#98c379', '#e06c75', '#c678dd'][i % 4]);
        const pm = transientMetricTrace(dev, 'p_mech_mw', `${dev.name} Pm`, ['#61afef', '#98c379', '#e06c75', '#c678dd'][i % 4]);
        if (p) traces.push(p);
        if (pm) {
          pm.line = { ...(pm.line || {}), dash: 'dot' };
          traces.push(pm);
        }
      });
      if (traces.length) Plotly.react(genPowerChart, traces, transientPlotLayout('传统机组功率轨迹', 'MW'), cfg);
      else genPowerChart.innerHTML = '<p class="empty-hint">无传统机组功率轨迹</p>';
    }
    const observerChart = document.getElementById('trObserverChart');
    if (observerChart) {
      const palette = ['#61afef', '#e06c75', '#98c379', '#d19a66', '#c678dd', '#56b6c2', '#e5c07b', '#7c3aed'];
      const traces = [];
      transientObserverRows(data).forEach((row, i) => {
        let tr = null;
        if (row.kind === 'bus') {
          tr = transientBusVoltageTrace(data, row.domain || 'AC', row.bus, palette[i % palette.length]);
        } else if (row.dev) {
          tr = transientMetricTrace(row.dev, 'v_pos_pu', `${row.name} V`, palette[i % palette.length]) ||
            transientMetricTrace(row.dev, 'vdc_pu', `${row.name} Vdc`, palette[i % palette.length]);
        }
        if (tr) traces.push(tr);
      });
      if (traces.length) Plotly.react(observerChart, traces, transientPlotLayout('本地观测点电压', 'p.u.'), cfg);
      else observerChart.innerHTML = '<p class="empty-hint">暂无可用本地观测轨迹</p>';
    }
    requestAnimationFrame(() => {
      document.querySelectorAll('#transientResults .js-plotly-plot').forEach(el => {
        try { Plotly.Plots.resize(el); } catch (_) {}
      });
    });
  }

  function showTransientResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('transient');
    const nf = (v, d = 3) => Number.isFinite(Number(v)) ? Number(v).toFixed(d) : '—';
    const final = data.final || {};
    const init = data.initialization || {};
    const gflCount = transientDeviceByType(data, type => type.includes('GridFollowing')).length;
    const gfmCount = transientDeviceByType(data, type => type.includes('GridForming')).length;
    const genCount = transientTraditionalGenerators(data).length;
    const opts = data.options || {};
    const dynamicDcLinkOn = opts.dynamic_dc_link === true;
    const recordDeviceOutputsOn = opts.record_device_outputs !== false;
    const initStatus = init.power_flow_requested
      ? (init.power_flow_converged ? 'PF收敛' : (init.fallback_voltage_setpoints ? 'PF未收敛/回退' : 'PF未收敛'))
      : '设定值初始化';
    let html = '<div class="transient-kpi-grid">';
    [
      ['初始点', initStatus, 't=0'],
      ['步数', data.steps ?? 0, ''],
      ['拒绝步', data.rejected_steps ?? 0, '次'],
      ['最大误差', nf(data.max_local_error_norm, 2), 'scaled'],
      ['最小步长', Number.isFinite(Number(data.min_accepted_step_s)) && Number(data.min_accepted_step_s) > 0 ? Number(data.min_accepted_step_s).toExponential(1) : '—', 's'],
      ['Newton', data.newton_iterations ?? 0, 'iter'],
      ['GFL', gflCount, '台'],
      ['GFM', gfmCount, '台'],
      ['传统机组', genCount, '台'],
      ['启用动态 DC 链', dynamicDcLinkOn ? '开' : '关', ''],
      ['动态设备', recordDeviceOutputsOn ? (data.device_series?.length ?? 0) : '未记录', recordDeviceOutputsOn ? '台' : ''],
      ['扰动数', Array.isArray(data.scheduled_events) ? data.scheduled_events.length : 0, '个'],
      ['AC最低电压', nf(final.min_ac_voltage_pu), 'p.u.'],
      ['DC最高电压', nf(final.max_dc_voltage_pu), 'p.u.'],
    ].forEach(([label, value, unit]) => {
      html += `<div class="transient-kpi"><div class="transient-kpi-label">${escapeHtml(label)}</div><div class="transient-kpi-value">${escapeHtml(String(value))}</div><div class="transient-kpi-label">${escapeHtml(unit)}</div></div>`;
    });
    html += '</div>';
    if (Array.isArray(data.warnings) && data.warnings.length) {
      html += `<div style="color:var(--orange);font-size:12px;margin:6px 0;">${data.warnings.map(escapeHtml).join('<br>')}</div>`;
    }
    const initWarnings = Array.isArray(init.warnings) && init.warnings.length
      ? `<div class="transient-init-warnings">${init.warnings.map(escapeHtml).join('<br>')}</div>` : '';
    const residualDiagnostics = Array.isArray(init.dynamic_residual_diagnostics)
      ? init.dynamic_residual_diagnostics.slice(0, 6) : [];
    html += '<div class="transient-init-panel">';
    html += `<div><span>事件前初始点</span><strong>${escapeHtml(initStatus)}</strong></div>`;
    html += `<div><span>PF迭代</span><strong>${escapeHtml(String(init.iterations ?? 0))}</strong></div>`;
    html += `<div><span>PF残差</span><strong>${Number.isFinite(Number(init.residual)) ? Number(init.residual).toExponential(2) : '—'}</strong></div>`;
    html += `<div><span>动态平衡</span><strong>${init.dynamic_trim_converged ? 'trim收敛' : 'trim未收敛'} / ${escapeHtml(String(init.dynamic_trim_iterations ?? 0))}次</strong></div>`;
    html += `<div><span>快速状态残差</span><strong>${Number.isFinite(Number(init.dynamic_fast_dxdt_inf_norm)) ? Number(init.dynamic_fast_dxdt_inf_norm).toExponential(2) : '—'}</strong></div>`;
    html += `<div><span>AC电压范围</span><strong>${nf(init.min_ac_voltage_pu)} - ${nf(init.max_ac_voltage_pu)} pu</strong></div>`;
    html += `<div><span>DC电压范围</span><strong>${nf(init.min_dc_voltage_pu)} - ${nf(init.max_dc_voltage_pu)} pu</strong></div>`;
    if (residualDiagnostics.length) {
      html += '<div class="transient-residual-diagnostics"><span>残差来源</span>';
      html += residualDiagnostics.map(d => {
        const value = Number(d.residual);
        const residual = Number.isFinite(value) ? value.toExponential(2) : '—';
        const name = d.device_name || d.device_type || 'Device';
        const type = d.device_type ? ` / ${d.device_type}` : '';
        const comp = Number.isFinite(Number(d.component_index)) ? `#${d.component_index}` : '';
        const state = Number.isFinite(Number(d.state_index)) ? `x[${d.state_index}]` : 'x';
        return `<strong>${escapeHtml(name)}${escapeHtml(comp)}${escapeHtml(type)} · ${escapeHtml(state)} = ${escapeHtml(residual)}</strong>`;
      }).join('');
      html += '</div>';
    }
    html += initWarnings;
    html += '</div>';
    const scheduledEvents = Array.isArray(data.scheduled_events) ? data.scheduled_events : [];
    const appliedEvents = Array.isArray(data.applied_events) ? data.applied_events : [];
    html += '<div class="transient-section-head"><h5>扰动序列</h5><span>计划 / 实际触发</span></div>';
    html += '<div class="transient-mini-grid">';
    if (scheduledEvents.length) {
      scheduledEvents.forEach((event, i) => {
        html += `<div class="transient-mini-card"><strong>${escapeHtml(event.label || transientEventLabel(event.type) || `Event ${i + 1}`)}</strong><span>${escapeHtml(transientFormatEvent(event))}</span></div>`;
      });
    } else {
      html += '<div class="transient-mini-card"><strong>无扰动</strong><span>本次为事件前平衡点保持/小扰动检查。</span></div>';
    }
    html += `<div class="transient-mini-card"><strong>已触发</strong><span>${appliedEvents.length ? appliedEvents.map(escapeHtml).join('<br>') : '无'}</span></div>`;
    html += '</div>';
    const observerRows = transientObserverRows(data);
    const maps = typeof Canvas !== 'undefined' && Canvas.getCompBusMap ? Canvas.getCompBusMap() : null;
    html += '<div class="transient-section-head"><h5>本地观测点</h5><span>点击卡片定位画布元件</span></div>';
    html += '<div class="transient-mini-grid">';
    if (observerRows.length) {
      observerRows.forEach(row => {
        const compId = row.dev ? rowCanvasCompId(row.dev, maps)
          : (row.domain === 'DC' ? maps?.dc?.[row.bus] : maps?.ac?.[row.bus]);
        const cid = parseInt(compId, 10);
        const clickable = Number.isInteger(cid);
        const attr = clickable
          ? ` class="transient-mini-card topo-clickable" data-comp-id="${cid}" onclick="Canvas.panToComponent(${cid})"`
          : ' class="transient-mini-card"';
        html += `<div${attr}><strong>${escapeHtml(row.name || 'Observer')}</strong><span>${escapeHtml(row.reason || '')}${row.bus ? `<br>${escapeHtml(row.domain || 'AC')} bus ${escapeHtml(String(row.bus))}` : ''}</span></div>`;
      });
    } else {
      html += '<div class="transient-mini-card"><strong>暂无观测点</strong><span>加载含母线或动态设备的算例后自动生成。</span></div>';
    }
    html += '</div>';
    html += transientRenderModelCompatibility(data);
    html += '<div class="transient-section-head"><h5>暂态轨迹</h5><span>传统机组 / GFL-GFM / 逆变器 / DC链</span></div>';
    html += '<div class="transient-dashboard-grid">';
    html += '<div id="trVoltageChart" class="transient-chart transient-chart-wide"></div>';
    html += '<div id="trAcHeatmap" class="transient-chart transient-chart-wide"></div>';
    html += '<div id="trObserverChart" class="transient-chart transient-chart-wide"></div>';
    html += '<div id="trGenFreqChart" class="transient-chart"></div>';
    html += '<div id="trGenPowerChart" class="transient-chart"></div>';
    html += '<div id="trFreqChart" class="transient-chart"></div>';
    html += '<div id="trPowerChart" class="transient-chart"></div>';
    html += '<div id="trCurrentChart" class="transient-chart"></div>';
    html += '<div id="trDcLinkChart" class="transient-chart"></div>';
    html += '</div>';
    html += '<h5>动态设备</h5><table><thead><tr><th>设备</th><th>类型</th><th>母线</th><th>P(MW)</th><th>Q(Mvar)</th><th>频率/PLL(Hz)</th><th>|I|(pu)</th></tr></thead><tbody>';
    (data.device_series || []).slice(0, 80).forEach(dev => {
      const vals = dev.values || {};
      const last = arr => Array.isArray(arr) && arr.length ? arr[arr.length - 1] : undefined;
      const freq = last(vals.frequency_hz) ?? last(vals.pll_frequency_hz);
      const imag = last(vals.i_mag_pu) ?? last(vals.i_rms_pu);
      const compId = rowCanvasCompId(dev, maps);
      const attr = compClickAttr(compId);
      html += `<tr${attr}><td>${escapeHtml(dev.name || '')}</td><td>${escapeHtml(dev.type || '')}</td><td>${escapeHtml(String(dev.bus ?? ''))}</td><td>${nf(last(vals.p_mw), 3)}</td><td>${nf(last(vals.q_mvar), 3)}</td><td>${nf(freq, 3)}</td><td>${nf(imag, 4)}</td></tr>`;
    });
    html += '</tbody></table>';
    const el = document.getElementById('transientResults');
    if (el) el.innerHTML = html;
    drawTransientDashboard(data);
  }

  // Dispatch the harmonics run by the selected analysis mode.
  async function runHarmonics() {
    const mode = document.getElementById('hpfMode')?.value || 'penetration';
    setStatus('谐波分析计算中...', 'busy');
    if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
    try {
      if (mode === 'freqscan') return await runHarmonicsFreqScan();
      if (mode === 'threephase') return await runHarmonics3ph();
      if (mode === 'metrics') return await runHarmonicsMetrics();
      if (mode === 'newton') return await runHarmonicsNewton();
      return await runHarmonicsPenetration();
    } catch (e) {
      setStatus((e && e.message) || '谐波分析失败', 'error');
    }
  }

  async function runHarmonicsPenetration() {
    const data = await apiPost('/api/session/harmonics', { options: hpfReadOptions() });
    if (data && data.ok) {
      log(`谐波潮流完成: 最大 AC THD ${(data.max_ac_thd_pct || 0).toFixed(2)}% @ 母线 ${data.max_ac_thd_bus}`, 'success');
      setStatus('谐波潮流完成');
      showHarmonicsResults(data);
      switchTab('results');
    } else {
      setStatus((data && (data.message || data.error)) || '谐波潮流失败', 'error');
    }
  }

  async function runHarmonicsFreqScan() {
    const seq = document.getElementById('hpfFsSequence')?.checked ?? false;
    const buses = parseOrderList(document.getElementById('hpfFsBuses')?.value, []);
    const scan = {
      f_start: parseFloat(document.getElementById('hpfFsStart')?.value || '1'),
      f_end: parseFloat(document.getElementById('hpfFsEnd')?.value || '25'),
      f_step: parseFloat(document.getElementById('hpfFsStep')?.value || '0.1'),
      buses, sequence: seq,
    };
    if (seq && buses.length) scan.bus = buses[0];
    const data = await apiPost('/api/session/harmonics_freqscan',
                               { options: hpfReadOptions(), scan });
    if (data && data.ok) {
      log(`频率扫描完成: ${(data.resonances || []).length} 处谐振`, 'success');
      setStatus('频率扫描完成');
      showHarmonicsFreqScan(data);
      switchTab('results');
    } else {
      setStatus((data && (data.message || data.error)) || '频率扫描失败', 'error');
    }
  }

  async function runHarmonics3ph() {
    const data = await apiPost('/api/session/harmonics_3ph', { options: hpfReadOptions() });
    if (data && data.ok) {
      log(`三相谐波完成: 最大 THD ${(data.max_thd_pct || 0).toFixed(2)}% @ 母线 ${data.max_thd_bus}`, 'success');
      setStatus('三相谐波完成');
      showHarmonics3ph(data);
      switchTab('results');
    } else {
      setStatus((data && (data.message || data.error)) || '三相谐波失败 (本算例可能无三相模型)', 'error');
    }
  }

  async function runHarmonicsMetrics() {
    const iL = parseFloat(document.getElementById('hpfIDemand')?.value || '0');
    const data = await apiPost('/api/session/harmonics_metrics',
                               { options: hpfReadOptions(), i_demand_pu: Number.isFinite(iL) ? iL : 0 });
    if (data && data.ok) {
      log(`谐波指标完成: 最大 K=${(data.max_k_factor || 1).toFixed(2)}, 谐波损耗占比 ${((data.harmonic_loss_fraction || 0) * 100).toFixed(2)}%`, 'success');
      setStatus('谐波指标完成');
      showHarmonicsMetrics(data);
      switchTab('results');
    } else {
      setStatus((data && (data.message || data.error)) || '谐波指标失败', 'error');
    }
  }

  async function runHarmonicsNewton() {
    const nmode = document.getElementById('hpfNewtonMode')?.value || 'constant_power';
    let resources = [];
    const raw = (document.getElementById('hpfNewtonResources')?.value || '').trim();
    if (raw) {
      try { resources = JSON.parse(raw); }
      catch (e) { setStatus('牛顿资源 JSON 解析失败: ' + e.message, 'error'); return; }
    }
    const data = await apiPost('/api/session/harmonics_newton',
                               { options: hpfReadOptions(), mode: nmode, resources });
    if (data && data.ok) {
      log(`牛顿谐波完成: ${data.converged ? '收敛' : '未收敛'} (${data.max_iterations_used} 次)`,
          data.converged ? 'success' : 'warn');
      setStatus('牛顿谐波完成');
      showHarmonicsNewton(data);
      switchTab('results');
    } else {
      setStatus((data && (data.message || data.error)) || '牛顿谐波失败', 'error');
    }
  }

  // ========== Topology Reconfiguration ==========
  async function runTopologyReconfig() {
    setStatus('拓扑重构中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const data = await apiPost('/api/session/run_reconfig', {
      options: {
        v_min_pu: 0.95,
        v_max_pu: 1.05,
        mip_gap: 0.01,
        max_time_s: 60,
        enable_pf: (document.getElementById('rcEnablePF')||{}).value !== '0',
        enable_voltage: (document.getElementById('rcEnableVoltage')||{}).checked !== false,
        enable_thermal: (document.getElementById('rcEnableThermal')||{}).checked !== false,
        split_domain_trees: (document.getElementById('rcSplitTrees')||{}).checked === true,
        allow_dc_mesh: (document.getElementById('rcDcMesh')||{}).checked === true,
        loss_aware: (document.getElementById('rcLossAware')||{}).checked !== false,
        line_failures: ((document.getElementById('rcFaults')||{}).value||'').split(',').map(x=>parseInt(x.trim())).filter(x=>x>0),
        fault_dc: ((document.getElementById('rcFaultsDc')||{}).value||'').split(',').map(x=>parseInt(x.trim())).filter(x=>x>0),
        fault_vsc: ((document.getElementById('rcFaultsVsc')||{}).value||'').split(',').map(x=>parseInt(x.trim())).filter(x=>x>0),
        lambda_shed: parseFloat((document.getElementById('rcLambdaShed')||{}).value) || 1e4,
        lambda_island: parseFloat((document.getElementById('rcLambdaIsland')||{}).value) || 1e5,
        solver: (document.getElementById('rcSolver')||{}).value || 'auto',
        lambda_loss: parseFloat((document.getElementById('rcLambdaLoss')||{}).value) || ({loss:50,switch:5,restore:10})[(document.getElementById('rcObjective')||{}).value||'loss'],
        lambda_switch: parseFloat((document.getElementById('rcLambdaSwitch')||{}).value) || ({loss:1,switch:20,restore:1})[(document.getElementById('rcObjective')||{}).value||'loss'],
        lambda_shed: parseFloat((document.getElementById('rcLambdaShed')||{}).value) || 1e4,
        max_switch_ops: parseInt((document.getElementById('rcMaxSwOps')||{}).value)||0,
        verbose: false,
      }
    });

    if (data) {
      if (data.feasible !== false) {
        const baseLoss = data.base_loss_mw?.toFixed(4) || '?';
        const reconLoss = data.reconfig_loss_mw?.toFixed(4) || '?';
        const redPct = data.loss_reduction_pct?.toFixed(1) || '0';
        log(`拓扑重构完成: 重构前损耗=${baseLoss}MW, 重构后损耗=${reconLoss}MW, 降低${redPct}%, ` +
            `辐射状=${data.reconfig_is_radial ? '是' : '否'}, 连通=${data.reconfig_is_connected ? '是' : '否'}`, 'success');
        const ops = data.switch_operations || [];
        const ot = data.obj_terms || {};
        log(`目标分解: 损耗=${(ot.loss||0).toFixed(3)} 开关=${(ot.switching||0).toFixed(0)} 切负荷=${(ot.shed||0).toFixed(1)} 孤岛=${(ot.island||0).toFixed(0)} | 校验 PF=${data.reconfig_pf_converged?'✓':'✗'} OPF=${data.opf_converged?'✓':'✗'}(obj=${(data.opf_objective||0).toFixed(1)})`, 'info');
        const af = data.applied_faults || {};
        const nf = (af.ac||[]).length + (af.dc||[]).length + (af.vsc||[]).length;
        if (nf) log(`已施加故障 AC=[${(af.ac||[]).join(',')}] DC=[${(af.dc||[]).join(',')}] VSC=[${(af.vsc||[]).join(',')}]`, 'info');
        if (ops.length) {
          const desc = ops.map(o => `${o.kind === 'circuit_breaker' ? '断路器' : (o.kind === 'switch' ? '开关' : '支路')}#${o.index}:${o.close ? '合' : '分'}`).join(', ');
          log(`联络开关/断路器操作 (${ops.length}): ${desc}`, 'info');
        }
        setStatus('拓扑重构完成');
      } else {
        log('拓扑重构: 未找到可行解', 'warn');
        setStatus('不可行', 'error');
      }
      showTopologyResults(data);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
  }

  // ========== Topology Analysis (graph structure) ==========
  // Palette for island coloring — shared between the legend, the result
  // tables, and the canvas overlay so the same island is the same color.
  const TOPO_ISLAND_COLORS = [
    '#61afef', '#98c379', '#e5c07b', '#c678dd', '#56b6c2',
    '#d19a66', '#e06c75', '#7f9f7f', '#b294bb', '#de935f',
  ];
  function topoIslandColor(islandId) {
    if (islandId === undefined || islandId === null || islandId < 0) return '#888';
    return TOPO_ISLAND_COLORS[islandId % TOPO_ISLAND_COLORS.length];
  }
  function topoOverlayOptions() {
    return {
      showIslands: document.getElementById('topoShowIslands')?.checked !== false,
      showBridges: document.getElementById('topoShowBridges')?.checked !== false,
      showCutVertices: document.getElementById('topoShowCutVertices')?.checked !== false,
      islandColor: topoIslandColor,
    };
  }

  async function runTopologyAnalysis() {
    setStatus('拓扑分析中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const data = await apiPost('/api/session/topology', {});

    if (data && !data.error) {
      _lastTopoAnalysisData = data;
      const nIsl = (data.n_ac_islands || 0) + (data.n_dc_islands || 0);
      const radial = data.is_radial ? '辐射状' : `含${data.cycle_count || 0}个环`;
      const conn = data.is_connected ? '连通' : '不连通';
      const valid = data.all_islands_valid ? '全部有效' : '存在无效孤岛';
      log(`拓扑分析完成: ${conn}, ${radial}, 孤岛${nIsl}个(${valid}), ` +
          `桥支路${(data.bridges || []).length}条, 割点${(data.cut_vertex_bus_ids || []).length}个`,
          data.all_islands_valid ? 'success' : 'warn');
      setStatus('拓扑分析完成');
      showTopologyAnalysisResults(data);
      Canvas.showTopologyResults(data, topoOverlayOptions());
      setActiveResultGroup('topologyAnalysis');
      switchTab('results');
    } else {
      log(`拓扑分析失败: ${data?.error || '未知错误'}`, 'error');
      setStatus('分析失败', 'error');
    }
  }

  function showTopologyAnalysisResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('topologyAnalysis');
    // Shares its result-group with 网络化简; show the structural-analysis panel
    // and hide the reduction panel.
    const ts = document.getElementById('topoAnalysisSection');
    const ns = document.getElementById('netReductionSection');
    if (ts) ts.style.display = 'block';
    if (ns) ns.style.display = 'none';

    const esc = (s) => String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));

    // ── Summary ──
    const summary = document.getElementById('topoAnalysisSummary');
    if (summary) {
      summary.innerHTML =
        `<span>母线: <b>${data.n_buses ?? '?'}</b></span>` +
        `<span>支路: <b>${data.n_branches ?? '?'}</b></span>` +
        `<span>连通性: <b>${data.is_connected ? '连通' : '不连通'}</b></span>` +
        `<span>辐射性: <b>${data.is_radial ? '辐射状' : '含环网'}</b></span>` +
        `<span>环路数: <b>${data.cycle_count ?? 0}</b></span>` +
        `<span>AC孤岛: <b>${data.n_ac_islands ?? 0}</b></span>` +
        `<span>DC孤岛: <b>${data.n_dc_islands ?? 0}</b></span>` +
        `<span>孤岛状态: <b>${data.all_islands_valid ? '全部有效' : '存在无效'}</b></span>`;
    }

    // ── Island legend (color chips, matches canvas) ──
    const legend = document.getElementById('topoAnalysisIslandLegend');
    if (legend) {
      const islands = data.islands || [];
      legend.innerHTML = islands.map(isl => {
        const invalid = isl.status !== 'Valid';
        return `<span class="legend-item${invalid ? ' invalid' : ''}">` +
          `<span class="legend-swatch" style="background:${topoIslandColor(isl.island_id)}"></span>` +
          `孤岛 ${isl.island_id} (${isl.domain}, ${isl.n_buses}母线)</span>`;
      }).join('') || '<span class="empty-hint">无孤岛</span>';
    }

    // ── Islands table ──
    const islStatusLabel = {
      Valid: '✅ 有效', NoSlack: '⚠️ 无平衡节点', NoDCVoltageRef: '⚠️ 无DC电压参考',
      IsolatedLoad: '⚠️ 孤立负荷', Empty: '空',
    };
    const islEl = document.getElementById('topoAnalysisIslands');
    if (islEl) {
      const rows = (data.islands || []).map(isl => {
        const buses = [...(isl.ac_bus_ids || []), ...(isl.dc_bus_ids || [])];
        const busList = buses.slice(0, 12).join(', ') + (buses.length > 12 ? ` … (+${buses.length - 12})` : '');
        return `<tr><td><span class="legend-swatch" style="background:${topoIslandColor(isl.island_id)}"></span> ${isl.island_id}</td>` +
          `<td>${isl.domain}</td><td>${isl.n_buses}</td>` +
          `<td>${islStatusLabel[isl.status] || esc(isl.status)}</td>` +
          `<td>${esc(busList)}</td></tr>`;
      }).join('');
      islEl.innerHTML = rows
        ? `<table class="topo-table"><thead><tr><th>孤岛</th><th>域</th><th>母线数</th><th>状态</th><th>母线 ID</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<p class="empty-hint">无孤岛</p>';
    }

    // ── Bridges table ──
    const brEl = document.getElementById('topoAnalysisBridges');
    if (brEl) {
      const rows = (data.bridges || []).map(b => {
        // Per-endpoint domain when the bridge straddles AC/DC (e.g. a VSC),
        // else the single domain.
        const fd = b.from_domain || b.domain || '';
        const td = b.to_domain || b.domain || '';
        const domLabel = fd && td && fd !== td ? `${fd}→${td}` : (fd || td || '');
        return `<tr><td class="topo-clickable" data-bus="${b.from_bus}">${b.from_bus}</td>` +
          `<td class="topo-clickable" data-bus="${b.to_bus}">${b.to_bus}</td>` +
          `<td>${esc(b.category || '')}</td><td>${esc(domLabel)}</td></tr>`;
      }).join('');
      brEl.innerHTML = rows
        ? `<table class="topo-table"><thead><tr><th>起始母线</th><th>终止母线</th><th>类型</th><th>域</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<p class="empty-hint">无桥支路（无单点故障支路）</p>';
    }

    // ── Cut vertices table ──
    const cvEl = document.getElementById('topoAnalysisCutVertices');
    if (cvEl) {
      // Prefer the domain-tagged list so AC bus N and DC bus N are shown
      // distinctly (a flat id list would render "母线 N" twice with no way to
      // tell them apart); fall back to the legacy flat list.
      const cvs = Array.isArray(data.cut_vertices)
        ? data.cut_vertices
        : (data.cut_vertex_bus_ids || []).map(id => ({ bus: id, domain: '' }));
      cvEl.innerHTML = cvs.length
        ? '<div class="topo-cutvertex-chips">' + cvs.map(cv =>
            `<span class="legend-item topo-clickable" data-bus="${cv.bus}">` +
            `${cv.domain ? esc(cv.domain) + '母线' : '母线 '} ${cv.bus}</span>`).join(' ') + '</div>'
        : '<p class="empty-hint">无割点</p>';
    }

    // ── Diagnostics table ──
    const dgEl = document.getElementById('topoAnalysisDiagnostics');
    if (dgEl) {
      const rows = (data.diagnostics || []).map(d => {
        const buses = (d.related_buses || []).join(', ');
        return `<tr><td>${esc(d.message || '')}</td><td>${esc(buses)}</td></tr>`;
      }).join('');
      dgEl.innerHTML = rows
        ? `<table class="topo-table"><thead><tr><th>诊断</th><th>相关母线</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<p class="empty-hint">无诊断信息（拓扑健康）</p>';
    }

    // Click-to-pan: any element carrying data-bus pans the canvas to that bus.
    document.querySelectorAll('#topoAnalysisBridges [data-bus], #topoAnalysisCutVertices [data-bus]').forEach(el => {
      el.addEventListener('click', () => {
        const busId = parseInt(el.dataset.bus, 10);
        if (Number.isInteger(busId) && Canvas.panToBusId) Canvas.panToBusId(busId);
      });
    });
  }

  // ========== Network Reduction (graph reduction — before/after view) ==========
  function netReductionOptions() {
    return {
      enable_switch_contraction: document.getElementById('redEnableSwitch')?.checked !== false,
      enable_series_reduction:   document.getElementById('redEnableSeries')?.checked !== false,
      enable_pendant_reduction:  document.getElementById('redEnablePendant')?.checked === true,
      // Off by default: zero-impedance lines (e.g. a DC line left at r=0) must
      // not merge their two buses unless the user explicitly asks for it.
      contract_zero_impedance_lines: document.getElementById('redEnableZeroZLines')?.checked === true,
    };
  }

  async function runNetworkReduction() {
    setStatus('网络化简中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const data = await apiPost('/api/session/network_reduction', netReductionOptions());

    if (data && !data.error) {
      _lastNetReductionData = data;
      const b = data.before || {}, a = data.after || {};
      const pct = (data.reduction_pct_buses || 0).toFixed(1);
      log(`网络化简完成: 母线 ${b.n_buses ?? '?'}→${a.n_buses ?? '?'} (-${data.n_buses_eliminated ?? 0}, ${pct}%), ` +
          `支路 ${b.n_branches ?? '?'}→${a.n_branches ?? '?'}; ` +
          `开关合并 ${(data.switch_groups || []).length} 组, 串联 ${(data.series_records || []).length}, ` +
          `悬挂 ${(data.pendant_records || []).length}`, 'success');
      setStatus('网络化简完成');
      showNetworkReductionResults(data);
      Canvas.showNetworkReduction(data, netReductionColors);
      setActiveResultGroup('topologyAnalysis');
      switchTab('results');
    } else {
      log(`网络化简失败: ${data?.error || '未知错误'}`, 'error');
      setStatus('化简失败', 'error');
    }
  }

  // Stable color per representative (surviving) bus, so a collapsed group and
  // its representative share a tint on the canvas and in the tables.
  const NET_REDUCTION_COLORS = [
    '#56b6c2', '#98c379', '#e5c07b', '#c678dd', '#61afef',
    '#d19a66', '#e06c75', '#7f9f7f', '#b294bb', '#de935f',
  ];
  function netReductionColors(repBusId) {
    if (repBusId === undefined || repBusId === null || repBusId < 0) return '#888';
    return NET_REDUCTION_COLORS[Math.abs(repBusId) % NET_REDUCTION_COLORS.length];
  }

  function showNetworkReductionResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('topologyAnalysis');
    // This module shares its result-group with 拓扑分析; show the reduction
    // panel and hide the structural-analysis panel (and vice versa).
    const ts = document.getElementById('topoAnalysisSection');
    const ns = document.getElementById('netReductionSection');
    if (ts) ts.style.display = 'none';
    if (ns) ns.style.display = 'block';

    const esc = (s) => String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
    const b = data.before || {}, a = data.after || {};
    const st = data.stages || {};

    // ── Summary ──
    const summary = document.getElementById('redSummary');
    if (summary) {
      const pct = (data.reduction_pct_buses || 0).toFixed(1);
      summary.innerHTML =
        `<span>母线: <b>${b.n_buses ?? '?'} → ${a.n_buses ?? '?'}</b></span>` +
        `<span>支路: <b>${b.n_branches ?? '?'} → ${a.n_branches ?? '?'}</b></span>` +
        `<span>消去母线: <b>${data.n_buses_eliminated ?? 0}</b> (${pct}%)</span>` +
        `<span>消去支路: <b>${data.n_branches_eliminated ?? 0}</b></span>` +
        `<span>Kron 可消去节点: <b>${st.kron_identify?.n_candidates ?? 0}</b></span>`;
    }

    // ── Before/after stage breakdown ──
    const baEl = document.getElementById('redBeforeAfter');
    if (baEl) {
      const row = (label, enabled, detail) =>
        `<tr><td>${label}</td><td>${enabled ? '✅ 启用' : '— 关闭'}</td><td>${detail}</td></tr>`;
      baEl.innerHTML =
        '<table class="topo-table"><thead><tr><th>化简阶段</th><th>状态</th><th>结果</th></tr></thead><tbody>' +
        row('开关合并 (零阻抗/闭合开关)', st.switch_contraction?.enabled,
            `${st.switch_contraction?.n_groups ?? 0} 个超级节点，合并 ${st.switch_contraction?.n_buses_merged ?? 0} 母线 · 精确`) +
        row('串联化简 (二度无注入节点)', st.series_reduction?.enabled,
            `消去 ${st.series_reduction?.n_eliminated ?? 0} 母线 · 条件精确`) +
        row('悬挂折叠 (叶子负荷节点·近似)', st.pendant_reduction?.enabled,
            `消去 ${st.pendant_reduction?.n_eliminated ?? 0} 母线`) +
        row('Kron 消去 (被动内部节点·仅识别)', st.kron_identify?.enabled,
            `识别 ${st.kron_identify?.n_candidates ?? 0} 个候选节点（本视图不折叠）`) +
        '</tbody></table>';
    }

    // ── Switch contraction groups ──
    const sgEl = document.getElementById('redSwitchGroups');
    if (sgEl) {
      const rows = (data.switch_groups || []).map(g => {
        const buses = (g.bus_ids || []);
        const list = buses.slice(0, 14).join(', ') + (buses.length > 14 ? ` … (+${buses.length - 14})` : '');
        return `<tr><td><span class="legend-swatch" style="background:${netReductionColors(g.super_bus_id)}"></span> ` +
          `<span class="topo-clickable" data-bus="${g.super_bus_id}">${g.super_bus_id}</span></td>` +
          `<td>${g.domain || ''}</td><td>${buses.length}</td><td>${esc(list)}</td></tr>`;
      }).join('');
      sgEl.innerHTML = rows
        ? `<table class="topo-table"><thead><tr><th>超级节点</th><th>域</th><th>母线数</th><th>合并母线</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<p class="empty-hint">无开关合并组</p>';
    }

    // ── Series reduction records ──
    const srEl = document.getElementById('redSeriesRecords');
    if (srEl) {
      const rows = (data.series_records || []).map(r =>
        `<tr><td class="topo-clickable" data-bus="${r.eliminated_bus_id}">${r.eliminated_bus_id}</td>` +
        `<td class="topo-clickable" data-bus="${r.from_bus_id}">${r.from_bus_id}</td>` +
        `<td class="topo-clickable" data-bus="${r.to_bus_id}">${r.to_bus_id}</td>` +
        `<td>${r.domain || ''}</td><td>${(r.r_eq ?? 0).toFixed(5)}</td><td>${(r.x_eq ?? 0).toFixed(5)}</td>` +
        `<td>${esc(r.fidelity || 'exact')}</td></tr>`).join('');
      srEl.innerHTML = rows
        ? `<table class="topo-table"><thead><tr><th>消去母线</th><th>端点 i</th><th>端点 k</th><th>域</th><th>R_eq(pu)</th><th>X_eq(pu)</th><th>保真度</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<p class="empty-hint">无串联化简</p>';
    }

    // ── Pendant reduction records ──
    const prEl = document.getElementById('redPendantRecords');
    if (prEl) {
      const rows = (data.pendant_records || []).map(r =>
        `<tr><td class="topo-clickable" data-bus="${r.eliminated_bus_id}">${r.eliminated_bus_id}</td>` +
        `<td class="topo-clickable" data-bus="${r.parent_bus_id}">${r.parent_bus_id}</td>` +
        `<td>${r.domain || ''}</td><td>${esc(r.fidelity || 'approximate')}</td></tr>`).join('');
      prEl.innerHTML = rows
        ? `<table class="topo-table"><thead><tr><th>消去母线</th><th>父母线</th><th>域</th><th>保真度</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<p class="empty-hint">无悬挂折叠（未启用或无候选）</p>';
    }

    // ── Diagnostics ──
    const dgEl = document.getElementById('redDiagnostics');
    if (dgEl) {
      const rows = (data.diagnostics || []).map(d => {
        const buses = (d.related_buses || []).join(', ');
        return `<tr><td>${esc(d.message || '')}</td><td>${esc(buses)}</td></tr>`;
      }).join('');
      dgEl.innerHTML = rows
        ? `<table class="topo-table"><thead><tr><th>诊断</th><th>相关母线</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<p class="empty-hint">无诊断信息</p>';
    }

    // Click-to-pan on any bus id.
    document.querySelectorAll('#netReductionSection [data-bus]').forEach(el => {
      el.addEventListener('click', () => {
        const busId = parseInt(el.dataset.bus, 10);
        if (Number.isInteger(busId) && Canvas.panToBusId) Canvas.panToBusId(busId);
      });
    });
  }

  // ========== Bearing Capability Assessment (DL/T 2041-2019) ==========
  function showBcDialog() {
    document.getElementById('bcDialog').style.display = 'flex';
  }

  function hideBcDialog() {
    document.getElementById('bcDialog').style.display = 'none';
  }

  async function runBearingCapacity() {
    hideBcDialog();
    setStatus('承载力评估中 (潮流+短路+校核)...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const kr = parseFloat(document.getElementById('bcKr').value) || 0.8;
    const delta_UH = parseFloat(document.getElementById('bcDeltaUH').value) || 7.0;
    const delta_UL = parseFloat(document.getElementById('bcDeltaUL').value) || 7.0;
    const thd_limit = parseFloat(document.getElementById('bcThdLimit').value) || 5.0;

    const data = await apiPost('/api/session/run_bearing_capacity', {
      kr, delta_UH_pct: delta_UH, delta_UL_pct: delta_UL, thd_limit_pct: thd_limit
    });

    if (data && data.converged) {
      log(`承载力评估完成: ${data.bus_results?.length || 0} 母线, ${data.branch_results?.length || 0} 支路`, 'success');
      setStatus('承载力评估完成');
      showBearingCapResults(data);
      switchTab('results');
    } else if (data && data.error) {
      log(`承载力评估失败: ${data.error}`, 'error');
      setStatus('评估失败', 'error');
    } else {
      setStatus('评估失败', 'error');
    }
  }

  function gradeHtml(grade) {
    const labels = { green: '🟢 绿色', yellow: '🟡 黄色', red: '🔴 红色' };
    return `<span class="grade-${grade}">${labels[grade] || grade}</span>`;
  }

  function showBearingCapResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    document.getElementById('bearingCapSection').style.display = 'block';
    setActiveResultGroup('hosting');

    // Summary
    const summary = document.getElementById('bearingCapSummary');
    const worstGrade = data.area_results?.reduce((w, a) => {
      if (a.grade === 'red') return 'red';
      if (a.grade === 'yellow' && w !== 'red') return 'yellow';
      return w;
    }, 'green') || 'green';
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">潮流收敛</span>
        <span class="result-value result-converged">✓ 收敛 (${data.pf_iterations} 次迭代)</span></div>
      <div class="result-item"><span class="result-label">裕度系数 k<sub>r</sub></span>
        <span class="result-value">${data.kr}</span></div>
      <div class="result-item"><span class="result-label">电压偏差限值 (GB/T 12325)</span>
        <span class="result-value">+${data.delta_UH_limit}% / -${data.delta_UL_limit}%</span></div>
      <div class="result-item"><span class="result-label">谐波限值 THD (GB/T 14549)</span>
        <span class="result-value">${data.thd_limit_pct || 5.0}%</span></div>
      <div class="result-item"><span class="result-label">向220kV+反送电</span>
        <span class="result-value ${data.any_reverse_220kv ? 'result-failed' : 'result-converged'}">
          ${data.any_reverse_220kv ? '✗ 存在 → 直接判红' : '✓ 无'}</span></div>
      <div class="result-item"><span class="result-label">综合评估等级</span>
        <span class="result-value">${gradeHtml(worstGrade)}</span></div>
    `;
    document.getElementById('resultsSummary').innerHTML = summary.innerHTML;

    // Area results
    const areaDiv = document.getElementById('bearingAreaResults');
    if (data.area_results && data.area_results.length > 0) {
      let html = '<table><thead><tr><th>区域</th><th>评估等级</th></tr></thead><tbody>';
      data.area_results.forEach(a => {
        html += `<tr><td>Area ${a.area}</td><td>${gradeHtml(a.grade)}</td></tr>`;
      });
      html += '</tbody></table>';
      areaDiv.innerHTML = html;
    }

    // Branch/Transformer thermal stability table (DL/T 2041 §5.1)
    const brDiv = document.getElementById('bearingBranchResults');
    if (data.branch_results && data.branch_results.length > 0) {
      const busMap = Canvas.getCompBusMap();
      let html = '<table><thead><tr>' +
        '<th>评估线路/变压器</th><th>类型</th><th>From</th><th>To</th>' +
        '<th>S<sub>e</sub>(MVA)</th><th>P(MW)</th><th>负载率(%)</th>' +
        '<th>反向</th><th>λ(%)</th><th>P<sub>m</sub>(MW)</th><th>等级</th>' +
        '</tr></thead><tbody>';
      data.branch_results.forEach(br => {
        const tg = br.thermal_grade || 'green';
        const cls = tg === 'red' ? 'grade-red' : (tg === 'yellow' ? 'grade-yellow' :
                    (tg === 'no_rating' ? '' : 'grade-green'));
        const compId = Number.isFinite(Number(br.index)) ? busMap.branch?.[Number(br.index)] : undefined;
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        const typeLabel = br.equip_type === 'transformer' ? '变压器' : '线路';
        const seStr = br.has_rating ? br.se_mva.toFixed(1) : '<span style="color:#999">未设</span>';
        const lamStr = br.has_rating ? br.lambda_pct.toFixed(1) : '—';
        const pmStr = br.has_rating ? br.pm_mw.toFixed(1) : '—';
        const loadStr = br.has_rating ? br.loading_pct.toFixed(1) : '—';
        const gradeStr = tg === 'no_rating' ? '<span style="color:#999">未设额定</span>' : gradeHtml(tg);
        html += `<tr${attr}>
          <td>${br.name}</td><td>${typeLabel}</td>
          <td>${br.from_bus}</td><td>${br.to_bus}</td>
          <td>${seStr}</td><td>${br.pf_mw.toFixed(2)}</td>
          <td>${loadStr}</td>
          <td>${br.reverse_flow ? '⚠️ 反向' : '—'}</td>
          <td class="${cls}">${lamStr}</td>
          <td>${pmStr}</td>
          <td>${gradeStr}</td></tr>`;
      });
      html += '</tbody></table>';
      brDiv.innerHTML = html;
    } else {
      brDiv.innerHTML = '<p style="color:#888">无支路数据</p>';
    }

    // Bus results — SC check, voltage deviation check, harmonic check
    const busDiv = document.getElementById('bearingBusResults');
    if (data.bus_results && data.bus_results.length > 0) {
      const busMap = Canvas.getCompBusMap();
      let html = '<table><thead><tr>' +
        '<th>Bus</th><th>kV</th><th>V<sub>m</sub>(pu)</th>' +
        '<th>Ik"(kA)</th><th>I<sub>m</sub>(kA)</th><th>短路校核</th>' +
        '<th>δU<sub>H</sub>(%)</th><th>δU<sub>L</sub>(%)</th><th>电压校核</th>' +
        '<th>谐波校核</th><th>DG(MW)</th><th>等级</th>' +
        '</tr></thead><tbody>';
      data.bus_results.forEach(br => {
        const compId = busMap.ac?.[br.bus_id];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        const scCls = br.sc_pass ? 'grade-green' : 'grade-red';
        const vCls = br.voltage_pass ? 'grade-green' : 'grade-red';
        const harmStatus = br.harmonic_status || 'no_data';
        const harmCls = harmStatus === 'no_data' ? '' : (br.harmonic_pass ? 'grade-green' : 'grade-red');
        const harmLabel = harmStatus === 'no_data' ? '<span style="color:#999">无数据</span>' :
                          (br.harmonic_pass ? '✓' : '✗');
        html += `<tr${attr}>
          <td>${br.bus_id}</td><td>${br.base_kv}</td>
          <td>${br.vm_pu.toFixed(4)}</td>
          <td>${br.ikss_ka.toFixed(3)}</td>
          <td>${br.i_breaker_ka > 0 ? br.i_breaker_ka.toFixed(2) : '<span style="color:#999">未设</span>'}</td>
          <td class="${scCls}">${br.sc_pass ? '✓ 通过' : '✗ 超限'}</td>
          <td>${br.delta_u_h_pct.toFixed(2)}</td>
          <td>${br.delta_u_l_pct.toFixed(2)}</td>
          <td class="${vCls}">${br.voltage_pass ? '✓ 通过' : '✗ 超限'}</td>
          <td class="${harmCls}">${harmLabel}</td>
          <td>${(br.dg_mw || 0).toFixed(2)}</td>
          <td>${gradeHtml(br.grade)}</td></tr>`;
      });
      html += '</tbody></table>';
      busDiv.innerHTML = html;
    }

    // Comprehensive summary table (DL/T 2041 Appendix C)
    const sumDiv = document.getElementById('bearingSummaryTable');
    if (data.summary_table && data.summary_table.length > 0) {
      let html = '<table><thead><tr>' +
        '<th>评估对象</th><th>类型</th><th>热稳定 λ(%)</th>' +
        '<th>短路校核</th><th>电压校核</th><th>谐波校核</th>' +
        '<th>评估等级</th><th>P<sub>m</sub>(MW)</th>' +
        '</tr></thead><tbody>';
      data.summary_table.forEach(row => {
        const typeLabel = row.type === 'transformer' ? '变压器' :
                          (row.type === 'line' ? '线路' : (row.type === 'bus' ? '母线' : row.type));
        const lamStr = row.has_rating ? row.lambda_pct.toFixed(1) : '—';
        const scStr = row.sc_pass ? '✓' : '✗';
        const vStr = row.voltage_pass ? '✓' : '✗';
        const harmStr = (row.harmonic_status === 'no_data') ? '无数据' :
                        (row.harmonic_pass ? '✓' : '✗');
        const pmStr = row.pm_mw > 0 ? row.pm_mw.toFixed(1) : '—';
        html += `<tr>
          <td>${row.name}</td><td>${typeLabel}</td>
          <td>${lamStr}</td>
          <td>${scStr}</td><td>${vStr}</td><td>${harmStr}</td>
          <td>${gradeHtml(row.grade)}</td>
          <td>${pmStr}</td></tr>`;
      });
      html += '</tbody></table>';
      html += '<p style="color:#888;font-size:0.85em;margin-top:4px">注：下级电网评估等级应不高于上级电网的评估等级。</p>';
      sumDiv.innerHTML = html;
    }
  }

  // ========== Time-Series Power Flow ==========
  function showTspfDialog() {
    document.getElementById('tspfDialog').style.display = 'flex';
  }

  function hideTspfDialog() {
    document.getElementById('tspfDialog').style.display = 'none';
  }

  function parseCsvProfiles(text) {
    const lines = text.trim().split('\n').map(l => l.split(',').map(s => s.trim()));
    if (lines.length < 2) return null;
    const headers = lines[0];
    const numProfiles = headers.length;
    const profiles = headers.map((name, i) => ({ id: i, name: name, values: [] }));
    for (let r = 1; r < lines.length; r++) {
      for (let c = 0; c < numProfiles; c++) {
        profiles[c].values.push(parseFloat(lines[r][c]) || 0);
      }
    }
    return { num_steps: lines.length - 1, step_duration_hr: 1.0, profiles };
  }

  // ===== Scenario generation: outer-scope helpers (import / apply scenario time-series) =====
  function isUsableGeneratedScenarioTimeSeries(ts) {
    if (!ts || !Array.isArray(ts.profiles) || ts.profiles.length === 0) return false;
    const numSteps = Number(ts.num_steps || ts.profiles[0]?.values?.length || 0);
    return numSteps > 0 && ts.profiles.some(p => Array.isArray(p?.values) && p.values.length > 0);
  }

  function getImportedGeneratedScenarioCase(targetFamily = null) {
    const caseJson = _importedGeneratedScenario?.case || null;
    if (!caseJson) return null;
    const family = caseJson?._generated_scenario?.family || _importedGeneratedScenario?.family || '';
    if (targetFamily && family && family !== 'unknown' && family !== targetFamily) return null;
    return caseJson;
  }

  function getImportedGeneratedScenarioTimeSeries(targetFamily = null) {
    const caseJson = getImportedGeneratedScenarioCase(targetFamily);
    const ts = caseJson?._time_series;
    return isUsableGeneratedScenarioTimeSeries(ts) ? ts : null;
  }

  function hasUsableGeneratedScenarioTimeSeries(targetFamily = null) {
    return !!getImportedGeneratedScenarioTimeSeries(targetFamily);
  }

  function extractGeneratedScenarioProfileValues(ts, profileIdOrName) {
    if (profileIdOrName === undefined || profileIdOrName === null || profileIdOrName === '') return [];
    const profiles = Array.isArray(ts?.profiles) ? ts.profiles : [];
    const matched = profiles.find(profile => profile?.id === profileIdOrName || String(profile?.id) === String(profileIdOrName) || profile?.name === profileIdOrName);
    return Array.isArray(matched?.values) ? matched.values.map(v => Number(v || 0)) : [];
  }

  async function applyGeneratedScenarioTimeSeries(caseJson) {
    const ts = caseJson?._time_series;
    if (!ts || !Array.isArray(ts.profiles) || ts.profiles.length === 0) return false;
    const body = {
      num_steps: ts.num_steps || ts.profiles[0]?.values?.length || 24,
      step_duration_hr: ts.step_duration_hr || 1.0,
      profiles: ts.profiles,
      load_profile_map: ts.binding?.load_profile_map || [],
      assign_all_loads_to: Number.isInteger(ts.binding?.assign_all_loads_to) ? ts.binding.assign_all_loads_to : -1,
      assign_all_pv_to: Number.isInteger(ts.binding?.assign_all_pv_to) ? ts.binding.assign_all_pv_to : -1,
    };
    const r = await fetch('/api/session/set_ts_config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || 'set_ts_config failed');
    _generatedScenarioTimeSeriesActive = true;
    const simHr = document.getElementById('simulationHours');
    if (simHr) simHr.value = String(j.num_steps || body.num_steps);
    log(`生成场景时序已恢复：${j.num_steps}步，profiles=${j.num_profiles}`, 'success');
    return true;
  }

  async function resetGeneratedScenarioTimeSeriesIfActive(numSteps = null) {
    if (!_generatedScenarioTimeSeriesActive) return false;
    const simHours = Number.isFinite(numSteps) && numSteps > 0
      ? numSteps
      : (parseInt(document.getElementById('simulationHours')?.value, 10) || 24);
    const r = await fetch('/api/session/set_ts_config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ num_steps: simHours, step_duration_hr: 1.0 }),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || 'set_ts_config reset failed');
    _generatedScenarioTimeSeriesActive = false;
    log(`已切回默认时序配置：${j.num_steps}步，profiles=${j.num_profiles}`, 'info');
    return true;
  }

  async function runTimeSeriesPF() {
    setStatus('时序潮流计算中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    // If "使用场景时序" is enabled, bind imported regular-scenario profiles first.
    if (document.getElementById('regUseScenarioTimeSeries')?.checked && hasUsableGeneratedScenarioTimeSeries('regular')) {
      try { await applyGeneratedScenarioTimeSeries(getImportedGeneratedScenarioCase('regular')); }
      catch (e) { log(`应用生成场景时序失败：${e.message || e}`, 'warn'); }
    }

    // Read parameters from the inline time-series sub-toolbar.
    // 仿真时间(小时) = num_steps × step_duration_hr (here step_duration_hr = 1).
    const simHours = parseInt(document.getElementById('simulationHours')?.value, 10);
    const numSteps = (Number.isFinite(simHours) && simHours > 0) ? simHours : 24;
    const skipUC = document.getElementById('tspfSkipUC')?.checked ?? true;
    const runOPF = document.getElementById('tspfRunOPF')?.checked ?? false;
    // Constraint set + MILP solver selection (时序生产模拟 controls).
    const ucSolver = document.getElementById('tspfUcSolver')?.value || 'auto';
    const enableNet = document.getElementById('tspfNetworkConstraints')?.checked ?? false;
    const enableDcNet = document.getElementById('tspfDcNetworkConstraints')?.checked ?? false;
    const reservePct = parseFloat(document.getElementById('tspfReservePct')?.value);
    const reserveFraction = (Number.isFinite(reservePct) && reservePct > 0) ? reservePct / 100.0 : 0.0;

    const data = await apiPost('/api/session/run_ts_pf', {
      num_steps: numSteps,
      skip_uc: skipUC,
      run_opf: runOPF,
      uc_solver: ucSolver,
      enable_network_constraints: enableNet,
      enable_dc_network_constraints: enableDcNet,
      reserve_fraction: reserveFraction,
      objective: document.getElementById('tspfObjective')?.value || 'cost',
      enable_external_grid: document.getElementById('tspfExternalGrid')?.checked ?? false,
      enable_demand_response: document.getElementById('tspfDemandResponse')?.checked ?? false,
      dr_shiftable: document.getElementById('tspfDrShiftable')?.checked ?? false,
      dr_penalty: parseFloat(document.getElementById('tspfDrPenalty')?.value) || 0,
      enable_dispatchable_pv: document.getElementById('tspfDispatchablePv')?.checked ?? false,
      pv_curtail_penalty: parseFloat(document.getElementById('tspfPvCurtailPenalty')?.value) || 0,
      enable_microgrid: document.getElementById('tspfMicrogrid')?.checked ?? false,
      microgrid_island_penalty: parseFloat(document.getElementById('tspfMgIslandPenalty')?.value) || 0,
      enable_dc_branch_flows: document.getElementById('tspfDcBranchFlows')?.checked ?? false,
      enable_storage_degradation: document.getElementById('tspfStorageDegradation')?.checked ?? false,
      enable_vpp: document.getElementById('tspfVpp')?.checked ?? false,
      enable_energy_router: document.getElementById('tspfEnergyRouter')?.checked ?? false,
      enable_mobile_storage: document.getElementById('tspfMobileStorage')?.checked ?? false,
      mobile_storage_corelocate: document.getElementById('tspfMobileCorelocate')?.checked ?? false,
      cyclic_soc: document.getElementById('tspfCyclicSoc')?.checked ?? false,
      parallel_daily: document.getElementById('tspfParallel')?.checked ?? false,
      parallel_threads: parseInt(document.getElementById('tspfThreads')?.value, 10) || 0,
    });

    if (data) {
      log(`时序潮流完成: ${data.num_converged}/${data.num_steps}步收敛, 总发电成本=$${(data.total_generation_cost || 0).toFixed(0)}`, 'success');
      setStatus('时序潮流完成');
      _lastTspfData = data;
      showTimeSeriesResults(data);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
  }

  // ── Annual parallel production simulation (split year into independent days) ──
  async function runAnnualSim() {
    setStatus('年度并行生产模拟计算中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const resolution = document.getElementById('annResolution')?.value || '6h';
    const dailyMode = document.getElementById('annDailyMode')?.value || 'scuc';
    const parallel = document.getElementById('annParallel')?.checked ?? true;
    const threads = parseInt(document.getElementById('annThreads')?.value, 10);
    const cyclicSoc = document.getElementById('annCyclicSoc')?.checked ?? true;
    // Reuse the shared solver / constraint controls from the time-series toolbar.
    const ucSolver = document.getElementById('tspfUcSolver')?.value || 'auto';
    const enableNet = document.getElementById('tspfNetworkConstraints')?.checked ?? false;
    const enableDcNet = document.getElementById('tspfDcNetworkConstraints')?.checked ?? false;
    const reservePct = parseFloat(document.getElementById('tspfReservePct')?.value);
    const reserveFraction = (Number.isFinite(reservePct) && reservePct > 0) ? reservePct / 100.0 : 0.0;

    // Lightweight progress indicator: tick elapsed time while the (blocking)
    // annual solve runs, since the backend call is synchronous.
    const t0 = performance.now();
    let _tick = 0;
    const progressTimer = setInterval(() => {
      _tick += 1;
      setStatus(`年度并行生产模拟计算中... ${_tick}s`, 'busy');
    }, 1000);

    let data;
    try {
      data = await apiPost('/api/session/run_annual_sim', {
        resolution,
        daily_mode: dailyMode,
        parallel_daily: parallel,
        parallel_threads: Number.isFinite(threads) ? threads : 0,
        daily_cyclic_soc: cyclicSoc,
        run_opf: dailyMode !== 'sced',
        uc_solver: ucSolver,
        enable_network_constraints: enableNet,
        enable_dc_network_constraints: enableDcNet,
        reserve_fraction: reserveFraction,
        objective: document.getElementById('tspfObjective')?.value || 'cost',
        enable_external_grid: document.getElementById('tspfExternalGrid')?.checked ?? false,
        enable_demand_response: document.getElementById('tspfDemandResponse')?.checked ?? false,
        dr_shiftable: document.getElementById('tspfDrShiftable')?.checked ?? false,
        dr_penalty: parseFloat(document.getElementById('tspfDrPenalty')?.value) || 0,
        enable_dispatchable_pv: document.getElementById('tspfDispatchablePv')?.checked ?? false,
        pv_curtail_penalty: parseFloat(document.getElementById('tspfPvCurtailPenalty')?.value) || 0,
        enable_microgrid: document.getElementById('tspfMicrogrid')?.checked ?? false,
        microgrid_island_penalty: parseFloat(document.getElementById('tspfMgIslandPenalty')?.value) || 0,
        enable_dc_branch_flows: document.getElementById('tspfDcBranchFlows')?.checked ?? false,
        enable_storage_degradation: document.getElementById('tspfStorageDegradation')?.checked ?? false,
        enable_vpp: document.getElementById('tspfVpp')?.checked ?? false,
        enable_energy_router: document.getElementById('tspfEnergyRouter')?.checked ?? false,
        enable_mobile_storage: document.getElementById('tspfMobileStorage')?.checked ?? false,
        mobile_storage_corelocate: document.getElementById('tspfMobileCorelocate')?.checked ?? false,
      });
    } finally {
      clearInterval(progressTimer);
    }

    if (data) {
      data._wallSeconds = (performance.now() - t0) / 1000.0;
      data._dailyMode = dailyMode;   // remember the per-day mode for drill-down faithfulness
      data._cyclicSoc = cyclicSoc;
      log(`年度仿真完成: ${data.solver_name || ''}, 目标=$${Number(data.total_cost || 0).toFixed(0)}, 用时${data._wallSeconds.toFixed(1)}s`, 'success');
      setStatus('年度并行仿真完成');
      _lastAnnualData = data;
      showAnnualSimResults(data);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
  }

  // Shared rich-model / solver options read from the time-series toolbar, reused
  // by the single-horizon run and the annual day drill-down.
  function tspfRichOptions() {
    const reservePct = parseFloat(document.getElementById('tspfReservePct')?.value);
    return {
      uc_solver: document.getElementById('tspfUcSolver')?.value || 'auto',
      enable_network_constraints: document.getElementById('tspfNetworkConstraints')?.checked ?? false,
      enable_dc_network_constraints: document.getElementById('tspfDcNetworkConstraints')?.checked ?? false,
      reserve_fraction: (Number.isFinite(reservePct) && reservePct > 0) ? reservePct / 100.0 : 0.0,
      objective: document.getElementById('tspfObjective')?.value || 'cost',
      enable_external_grid: document.getElementById('tspfExternalGrid')?.checked ?? false,
      enable_demand_response: document.getElementById('tspfDemandResponse')?.checked ?? false,
      dr_shiftable: document.getElementById('tspfDrShiftable')?.checked ?? false,
      dr_penalty: parseFloat(document.getElementById('tspfDrPenalty')?.value) || 0,
      enable_dispatchable_pv: document.getElementById('tspfDispatchablePv')?.checked ?? false,
      pv_curtail_penalty: parseFloat(document.getElementById('tspfPvCurtailPenalty')?.value) || 0,
      enable_microgrid: document.getElementById('tspfMicrogrid')?.checked ?? false,
      microgrid_island_penalty: parseFloat(document.getElementById('tspfMgIslandPenalty')?.value) || 0,
      enable_dc_branch_flows: document.getElementById('tspfDcBranchFlows')?.checked ?? false,
      enable_storage_degradation: document.getElementById('tspfStorageDegradation')?.checked ?? false,
      enable_vpp: document.getElementById('tspfVpp')?.checked ?? false,
      enable_energy_router: document.getElementById('tspfEnergyRouter')?.checked ?? false,
      enable_mobile_storage: document.getElementById('tspfMobileStorage')?.checked ?? false,
      mobile_storage_corelocate: document.getElementById('tspfMobileCorelocate')?.checked ?? false,
    };
  }

  // Drill into a single day of the last annual run: re-solve that day's window
  // with the rich per-step time-series solver and show it in the (rich) 时序潮流
  // 结果 dashboard, keeping the annual dashboard in place for context.
  async function runAnnualDayDetail() {
    const meta = _lastAnnualData;
    if (!meta) { log('请先运行年度仿真，再查看某日详情', 'warn'); return; }
    const stepHr = meta.step_duration_hr || 1;
    const stepsPerDay = Math.max(1, Math.round(24 / stepHr));
    const numDays = Math.max(1, Math.round((meta.num_steps || stepsPerDay) / stepsPerDay));
    let day = parseInt(document.getElementById('annualSimDaySel')?.value, 10);
    if (!Number.isFinite(day) || day < 1) day = 1;
    if (day > numDays) day = numDays;

    setStatus(`第 ${day} 日逐步详情重解中...`, 'busy');
    if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }

    const payload = Object.assign({
      num_steps: stepsPerDay,
      step_duration_hr: stepHr,
      day_index: day,
      skip_uc: false,
      run_opf: true,
      // Faithful drill-down: re-solve the day with the SAME per-day mode and
      // cyclic-SOC the annual run used, so it matches that day of the annual run.
      daily_mode: meta._dailyMode || 'scuc',
      cyclic_soc: meta._cyclicSoc != null ? meta._cyclicSoc : true,
    }, tspfRichOptions());

    const data = await apiPost('/api/session/run_ts_pf', payload);
    if (data) {
      log(`第 ${day} 日详情完成: ${data.num_converged}/${data.num_steps} 步收敛`, 'success');
      setStatus(`第 ${day} 日详情完成`);
      _lastTspfData = data;
      showTimeSeriesResults(data, { keepAnnual: true });
      const tspfSec = document.getElementById('tspfResultsSection');
      if (tspfSec && tspfSec.scrollIntoView) { try { tspfSec.scrollIntoView({ behavior: 'smooth', block: 'start' }); } catch (e) {} }
    } else {
      setStatus('计算失败', 'error');
    }
  }

  function showAnnualSimResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('timeSeries');

    const section = document.getElementById('annualSimResultsSection');
    if (section) section.style.display = 'block';
    // Hide the (stale) single-horizon time-series dashboard so the annual
    // dashboard is shown unambiguously when an annual run completes.
    const tspfSec = document.getElementById('tspfResultsSection');
    if (tspfSec) tspfSec.style.display = 'none';
    // Day drill-down: size the selector to the number of simulated days.
    const stepsPerDay = Math.max(1, Math.round(24 / (data.step_duration_hr || 1)));
    const numDays = Math.max(1, Math.round((data.num_steps || stepsPerDay) / stepsPerDay));
    const daySel = document.getElementById('annualSimDaySel');
    if (daySel) {
      daySel.max = String(numDays);
      if (!daySel.value || +daySel.value < 1 || +daySel.value > numDays) daySel.value = '1';
    }
    // Representative-day quick presets (peak-load / min-load / peak-net-load /
    // per-season), auto-detected by the backend — one click drills into that day.
    const presetBox = document.getElementById('annualSimDayPresets');
    if (presetBox) {
      const reps = Array.isArray(data.representative_days) ? data.representative_days : [];
      presetBox.innerHTML = '<span style="font-weight:600;">代表日：</span>';
      if (!reps.length) {
        presetBox.insertAdjacentHTML('beforeend', '<span class="sub-hint">（无）</span>');
      } else {
        reps.forEach((r) => {
          const btn = document.createElement('button');
          btn.className = 'toolbar-btn';
          btn.textContent = `${r.label} (第${r.day}日)`;
          btn.title = `定位并以富时序图表重解第 ${r.day} 日`;
          btn.addEventListener('click', () => {
            const sel = document.getElementById('annualSimDaySel');
            if (sel) sel.value = String(r.day);
            runAnnualDayDetail();
          });
          presetBox.appendChild(btn);
        });
      }
    }

    data.cost_formula = data.cost_formula || data.objective_formula || {};
    const fmt = (x, n = 0) => Number(x || 0).toLocaleString('en-US', { maximumFractionDigits: n });
    const monthlyCostValue = (m) => firstFiniteNumber(
      m?.total_cost, m?.operating_cost, m?.total_operating_cost,
      m?.generation_cost, m?.total_generation_cost,
      m?.cost, m?.production_cost, m?.objective_value);
    const costFormulaTitle = data.cost_formula.title || '年度运行成本';
    const costFormulaText = data.cost_formula.formula_text || data.cost_formula.description || '';
    const costFormulaSource = data.cost_formula.dispatch_basis || data.cost_formula.source || data.solver_name || '';
    const feasClass = data.feasible ? 'result-converged' : 'result-failed';
    const annParExec = data.parallel_execution || {};
    const annWorkers = Number(data.parallel_workers || 1);
    const annParLabel = data.parallel_daily_effective
      ? `按日并行 (${annWorkers}线程)`
      : (data.parallel_daily ? '安全串行' : '串行');
    const annParDetail = annParExec.work_items != null
      ? ` · ${annParExec.work_items}项 · HW ${annParExec.hardware_threads || '—'}`
        + (annParExec.guard_reason ? ` · ${escapeHtml(annParExec.guard_reason)}` : '')
      : '';
    const summary = document.getElementById('annualSimSummary');
    if (summary) {
      summary.innerHTML = `
        <div class="result-item"><span class="result-label">求解模型/调度</span>
          <span class="result-value">${data.solver_name || '—'}</span></div>
        <div class="result-item"><span class="result-label">可行</span>
          <span class="result-value ${feasClass}">${data.feasible ? '是' : '否'}</span></div>
        <div class="result-item"><span class="result-label">时间步数</span>
          <span class="result-value">${data.num_steps} (${data.step_duration_hr}h)</span></div>
        <div class="result-item"><span class="result-label">并行</span>
          <span class="result-value">${annParLabel}${annParDetail}</span></div>
        <div class="result-item"><span class="result-label">${data.objective_label || '年总运行成本 ($)'}</span>
          <span class="result-value">$${fmt(data.objective_value != null ? data.objective_value : data.total_cost)}</span></div>
        <div class="result-item"><span class="result-label">成本口径</span>
          <span class="result-value" title="${escapeHtml(costFormulaText)}">${escapeHtml(costFormulaTitle)}</span></div>
        <div class="result-item"><span class="result-label">成本来源</span>
          <span class="result-value">${escapeHtml(costFormulaSource || '—')}</span></div>
        <div class="result-item"><span class="result-label">墙钟用时</span>
          <span class="result-value">${(data._wallSeconds || 0).toFixed(1)} s</span></div>
        <div class="result-item"><span class="result-label">总发电量</span>
          <span class="result-value">${fmt(data.total_gen_mwh)} MWh</span></div>
        <div class="result-item"><span class="result-label">总负荷</span>
          <span class="result-value">${fmt(data.total_load_mwh)} MWh</span></div>
        <div class="result-item"><span class="result-label">新能源发电</span>
          <span class="result-value">${fmt(data.total_renewable_mwh)} MWh</span></div>
        <div class="result-item"><span class="result-label">弃电量</span>
          <span class="result-value">${fmt(data.total_curtailment_mwh)} MWh</span></div>
        <div class="result-item"><span class="result-label">网损</span>
          <span class="result-value">${fmt(data.total_loss_mwh)} MWh</span></div>
        <div class="result-item"><span class="result-label">失负荷(ENS)</span>
          <span class="result-value">${fmt(data.total_ens_mwh)} MWh</span></div>
        <div class="result-item"><span class="result-label">潮流收敛</span>
          <span class="result-value">${data.num_pf_converged}/${data.num_steps}</span></div>
      `;
    }

    const theme = {
      paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)',
      font: { color: '#abb2bf', size: 11 },
      xaxis: { gridcolor: '#3e4451' }, yaxis: { gridcolor: '#3e4451' },
      margin: { l: 55, r: 15, t: 40, b: 40 }, legend: { orientation: 'h', y: -0.25 },
    };

    // Annual timeline (gen / load / renewable / curtailment / storage / loss).
    const tl = document.getElementById('annualSimTimelineChart');
    if (tl && window.Plotly && Array.isArray(data.timeline_hours)) {
      const h = data.timeline_hours;
      Plotly.react(tl, [
        { x: h, y: data.timeline_load || [], name: '负荷', type: 'scatter', mode: 'lines', line: { color: '#e74c3c' } },
        { x: h, y: data.timeline_gen || [], name: '发电', type: 'scatter', mode: 'lines', line: { color: '#0b6e4f' } },
        { x: h, y: data.timeline_ren || [], name: '新能源', type: 'scatter', mode: 'lines', line: { color: '#2980b9' } },
        { x: h, y: data.timeline_curt || [], name: '弃电', type: 'scatter', mode: 'lines', line: { color: '#f39c12' } },
        { x: h, y: data.timeline_ess || [], name: '储能(+放/−充)', type: 'scatter', mode: 'lines', line: { color: '#8e44ad', dash: 'dot' } },
        { x: h, y: data.timeline_loss || [], name: '网损', type: 'scatter', mode: 'lines', line: { color: '#7f8c8d', dash: 'dot' }, yaxis: 'y2' },
      ], Object.assign({
        title: '全年逐时段功率 (MW)',
        xaxis: Object.assign({ title: '小时' }, theme.xaxis),
        yaxis2: { title: '网损 (MW)', overlaying: 'y', side: 'right', gridcolor: 'rgba(0,0,0,0)', titlefont: { color: '#7f8c8d' }, tickfont: { color: '#7f8c8d' } },
      }, theme), { responsive: true, displayModeBar: false });
    }

    // Annual bus-voltage envelope (min / mean / max across buses per snapshot).
    const vc = document.getElementById('annualSimVoltChart');
    if (vc && window.Plotly && Array.isArray(data.snapshot_vm) && data.snapshot_vm.length) {
      const sh = data.snapshot_hours || data.snapshot_vm.map((_, i) => i);
      const vmin = [], vmean = [], vmax = [];
      data.snapshot_vm.forEach(row => {
        if (!Array.isArray(row) || !row.length) { vmin.push(null); vmean.push(null); vmax.push(null); return; }
        let mn = Infinity, mx = -Infinity, sum = 0;
        row.forEach(v => { if (v < mn) mn = v; if (v > mx) mx = v; sum += v; });
        vmin.push(mn); vmax.push(mx); vmean.push(sum / row.length);
      });
      Plotly.react(vc, [
        { x: sh, y: vmax, name: '最高电压', type: 'scatter', mode: 'lines', line: { color: '#e74c3c', width: 1 } },
        { x: sh, y: vmean, name: '平均电压', type: 'scatter', mode: 'lines', line: { color: '#2980b9', width: 2 } },
        { x: sh, y: vmin, name: '最低电压', type: 'scatter', mode: 'lines', line: { color: '#f39c12', width: 1 }, fill: 'tonexty', fillcolor: 'rgba(41,128,185,0.08)' },
      ], Object.assign({ title: '全年母线电压区间 (p.u., 抽样快照)', xaxis: Object.assign({ title: '小时' }, theme.xaxis), yaxis: Object.assign({ title: 'p.u.' }, theme.yaxis) }, theme), { responsive: true, displayModeBar: false });
    }

    // Monthly cost + energy bars.
    const ms = data.monthly_summaries || [];
    const mc = document.getElementById('annualSimMonthlyChart');
    if (mc && window.Plotly && ms.length) {
      const months = ms.map((m, i) => ['1月','2月','3月','4月','5月','6月','7月','8月','9月','10月','11月','12月'][m.block_id != null ? m.block_id : i] || `块${i}`);
      Plotly.react(mc, [
        { x: months, y: ms.map(monthlyCostValue), name: '月运行成本($)', type: 'bar', marker: { color: '#8e44ad' } },
      ], Object.assign({
        title: `月度成本 — ${costFormulaTitle}`,
        xaxis: theme.xaxis,
        yaxis: Object.assign({ title: '$' }, theme.yaxis),
        annotations: costFormulaText ? [{
          text: escapeHtml(costFormulaText),
          xref: 'paper', yref: 'paper', x: 0, y: -0.22, showarrow: false,
          align: 'left', font: { size: 10, color: '#8a909c' },
        }] : [],
      }, theme), { responsive: true });
    } else if (mc) {
      mc.innerHTML = '<p class="empty-hint">暂无月度成本数据</p>';
    }

    // Monthly table.
    const mt = document.getElementById('annualSimMonthlyTable');
    if (mt && ms.length) {
      let html = `<div class="cost-formula-box"><strong>${escapeHtml(costFormulaTitle)}</strong>${costFormulaText ? `<span>${escapeHtml(costFormulaText)}</span>` : ''}</div>`;
      html += '<table class="tbl"><thead><tr><th>月</th><th>发电(MWh)</th><th>负荷(MWh)</th><th>新能源(MWh)</th><th>弃电(MWh)</th><th>网损(MWh)</th><th>成本($)</th><th>PF收敛</th></tr></thead><tbody>';
      ms.forEach((m, i) => {
        const mn = ['1月','2月','3月','4月','5月','6月','7月','8月','9月','10月','11月','12月'][m.block_id != null ? m.block_id : i] || `块${i}`;
        html += `<tr><td>${mn}</td><td>${fmt(m.total_gen_mwh)}</td><td>${fmt(m.total_load_mwh)}</td><td>${fmt(m.total_renewable_mwh)}</td><td>${fmt(m.total_curtailment_mwh)}</td><td>${fmt(m.total_loss_mwh)}</td><td>${fmt(monthlyCostValue(m))}</td><td>${m.num_pf_converged}/${m.num_steps}</td></tr>`;
      });
      html += '</tbody></table>';
      mt.innerHTML = html;
    }

    // Per-generator annual statistics.
    const gt = document.getElementById('annualSimGenStats');
    if (gt) {
      const gss = data.gen_stats || [];
      if (gss.length) {
        let html = '<table class="tbl"><thead><tr><th>机组</th><th>发电量(MWh)</th><th>容量因子</th><th>启动次数</th><th>在线小时</th></tr></thead><tbody>';
        gss.forEach(g => {
          html += `<tr><td>${g.name || '—'}</td><td>${fmt(g.total_energy_mwh)}</td><td>${(Number(g.capacity_factor || 0) * 100).toFixed(1)}%</td><td>${fmt(g.total_startups)}</td><td>${fmt(g.total_hours_online)}</td></tr>`;
        });
        html += '</tbody></table>';
        gt.innerHTML = html;
      } else gt.innerHTML = '<p class="empty-hint">无机组统计</p>';
    }

    // Per-storage annual statistics.
    const st = document.getElementById('annualSimStorageStats');
    if (st) {
      const sss = data.storage_stats || [];
      if (sss.length) {
        let html = '<table class="tbl"><thead><tr><th>储能</th><th>充电(MWh)</th><th>放电(MWh)</th><th>等效循环</th></tr></thead><tbody>';
        sss.forEach(s => {
          html += `<tr><td>${s.name || '—'}</td><td>${fmt(s.total_charge_mwh)}</td><td>${fmt(s.total_discharge_mwh)}</td><td>${Number(s.cycles || 0).toFixed(1)}</td></tr>`;
        });
        html += '</tbody></table>';
        st.innerHTML = html;
      } else st.innerHTML = '<p class="empty-hint">无储能统计</p>';
    }

    // Per-renewable annual statistics.
    const rt = document.getElementById('annualSimRenStats');
    if (rt) {
      const rss = data.renewable_stats || [];
      if (rss.length) {
        let html = '<table class="tbl"><thead><tr><th>新能源</th><th>发电量(MWh)</th><th>弃电量(MWh)</th><th>容量因子</th><th>弃电率</th></tr></thead><tbody>';
        rss.forEach(r => {
          html += `<tr><td>${r.name || '—'}</td><td>${fmt(r.total_energy_mwh)}</td><td>${fmt(r.total_curtailed_mwh)}</td><td>${(Number(r.capacity_factor || 0) * 100).toFixed(1)}%</td><td>${(Number(r.curtailment_rate || 0) * 100).toFixed(1)}%</td></tr>`;
        });
        html += '</tbody></table>';
        rt.innerHTML = html;
      } else rt.innerHTML = '<p class="empty-hint">无新能源统计</p>';
    }

    // Bring the annual dashboard into view (it sits below the time-series one).
    if (section && section.scrollIntoView) {
      try { section.scrollIntoView({ behavior: 'smooth', block: 'start' }); } catch (e) { section.scrollIntoView(); }
    }
  }

  function showTimeSeriesResults(data, opts) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('timeSeries');
    renderComponentCurveTargets('timeSeries');

    const section = document.getElementById('tspfResultsSection');
    section.style.display = 'block';
    // Unless this is an annual day drill-down (keepAnnual), hide the annual
    // dashboard so a plain time-series run shows only its own results.
    if (!(opts && opts.keepAnnual)) {
      const annSec = document.getElementById('annualSimResultsSection');
      if (annSec) annSec.style.display = 'none';
    }

    // Summary KPIs
    const summary = document.getElementById('tspfSummary');
    const ucStatus = data.uc_feasible === false ? '失败' : (data.uc_feasible === true ? '成功' : '—');
    const ucClass = data.uc_feasible === false ? 'result-failed' : 'result-converged';
    // Active-constraint set summary (from the backend echo).
    const c = data.constraints || {};
    const cParts = [];
    if (c.unit_commitment) cParts.push('机组组合');
    if (c.economic_dispatch_opf) cParts.push('经济调度OPF');
    cParts.push(c.network_constraints ? '网络约束(DC潮流)' : '单母线平衡');
    if (c.dc_network_constraints) cParts.push('DC网络耦合');
    if (c.reserve_fraction > 0) cParts.push('旋转备用' + (c.reserve_fraction * 100).toFixed(0) + '%');
    const constraintSummary = cParts.join('、') || '—';
    const objLabel = data.objective_label || '目标函数值';
    const objVal = (data.objective_value != null) ? data.objective_value : data.total_generation_cost;
    const solverTag = data.uc_solver_name || data.uc_solver_requested || '';
    const parExec = data.parallel_execution || {};
    const parWorkers = Number(data.parallel_workers || 1);
    const parLabel = data.parallel_daily_effective
      ? `按日并行 (${parWorkers}线程)`
      : (data.parallel_mode === 'parallel-daily/serial-guarded' ? '安全串行' : '串行');
    const parDetail = parExec.work_items != null
      ? ` · ${parExec.work_items}项 · HW ${parExec.hardware_threads || '—'}`
        + (parExec.guard_reason ? ` · ${escapeHtml(parExec.guard_reason)}` : '')
      : '';
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">时间步数</span>
        <span class="result-value">${data.num_steps}</span></div>
      <div class="result-item"><span class="result-label">潮流收敛</span>
        <span class="result-value ${data.num_converged === data.num_steps ? 'result-converged' : 'result-failed'}">
          ${data.num_converged}/${data.num_steps}</span></div>
      <div class="result-item"><span class="result-label">OPF收敛</span>
        <span class="result-value">${data.num_opf_converged}/${data.num_steps}</span></div>
      <div class="result-item"><span class="result-label">机组组合</span>
        <span class="result-value ${ucClass}">${ucStatus}${data.uc_solver_name ? ' (' + data.uc_solver_name + ')' : ''}</span></div>
      <div class="result-item"><span class="result-label">求解器</span>
        <span class="result-value">${solverTag || '—'}</span></div>
      <div class="result-item"><span class="result-label">并行</span>
        <span class="result-value">${parLabel}${parDetail}</span></div>
      <div class="result-item"><span class="result-label">${objLabel}</span>
        <span class="result-value">$${Number(objVal || 0).toFixed(0)}</span></div>
      <div class="result-item"><span class="result-label">约束集</span>
        <span class="result-value" style="font-size:0.82em;">${constraintSummary}</span></div>
      <div class="result-item"><span class="result-label">总网损</span>
        <span class="result-value">${((data.losses_mw || []).reduce((a, b) => a + b, 0) * (data.step_duration_hr || 1)).toFixed(2)} MWh</span></div>
    `;

    const hrs = Array.from({ length: data.num_steps }, (_, i) => i);
    const pal = ['#0b6e4f', '#2c8c99', '#b5651d', '#8e44ad', '#2980b9', '#e74c3c', '#27ae60', '#f39c12'];
    const plotTheme = {
      paper_bgcolor: 'rgba(0,0,0,0)',
      plot_bgcolor: 'rgba(0,0,0,0)',
      font: { color: '#abb2bf', size: 11 },
      xaxis: { gridcolor: '#3e4451', title: '时间步' },
      yaxis: { gridcolor: '#3e4451' },
      margin: { l: 55, r: 15, t: 40, b: 40 },
      legend: { orientation: 'h', y: -0.25 },
    };

    // 1. Generation dispatch stacked bar chart
    const genDiv = document.getElementById('tspfGenChart');
    const dtraces = (data.gen_dispatch || []).map((d, gi) => ({
      x: hrs, y: d, type: 'bar',
      name: (data.gen_names || [])[gi] || 'Gen ' + gi,
      marker: { color: pal[gi % pal.length] },
    }));
    (data.renewable_dispatch || []).forEach((rd, ri) => dtraces.push({
      x: hrs, y: rd, type: 'bar',
      name: (data.ren_names || [])[ri] || 'Ren ' + ri,
      marker: { color: '#27ae60' },
    }));
    if (dtraces.length > 0) {
      Plotly.newPlot(genDiv, dtraces, {
        ...plotTheme, title: '发电出力 (MW)', barmode: 'stack',
        yaxis: { ...plotTheme.yaxis, title: 'MW' },
      }, { responsive: true });
    } else {
      genDiv.innerHTML = '<p class="empty-hint">无发电出力数据</p>';
    }

    // 2. Mean voltage time series (with min/max band)
    const voltDiv = document.getElementById('tspfVoltChart');
    if (data.vm_mean && data.vm_mean.length > 0) {
      const allV = [...data.vm_mean, ...(data.vm_min||[]), ...(data.vm_max||[])].filter(v=>v>0);
      const vMin = Math.min(...allV), vMax = Math.max(...allV);
      const vPad = Math.max((vMax - vMin) * 0.15, 0.005);
      const traces = [];
      if (data.vm_min && data.vm_max) {
        traces.push({
          x: hrs, y: data.vm_max, mode: 'lines', line: { color: 'rgba(97,175,239,0.3)', width: 0 },
          name: '最大电压', showlegend: false,
        });
        traces.push({
          x: hrs, y: data.vm_min, mode: 'lines', line: { color: 'rgba(97,175,239,0.3)', width: 0 },
          fill: 'tonexty', fillcolor: 'rgba(97,175,239,0.13)',
          name: '电压范围 (min–max)',
        });
      }
      traces.push({
        x: hrs, y: data.vm_mean,
        mode: 'lines+markers',
        line: { color: '#61afef', width: 2 },
        name: '平均电压',
      });
      Plotly.newPlot(voltDiv, traces, {
        ...plotTheme, title: '各时步母线电压 (p.u.)',
        yaxis: { ...plotTheme.yaxis, title: 'p.u.', range: [vMin - vPad, vMax + vPad] },
      }, { responsive: true });
    } else {
      voltDiv.innerHTML = '<p class="empty-hint">无电压数据</p>';
    }

    // 3. Losses time series
    const lossDiv = document.getElementById('tspfLossChart');
    if (data.losses_mw && data.losses_mw.length > 0) {
      const lMin = Math.min(...data.losses_mw), lMax = Math.max(...data.losses_mw);
      const lPad = Math.max((lMax - lMin) * 0.15, lMax * 0.05 || 0.001);
      Plotly.newPlot(lossDiv, [{
        x: hrs, y: data.losses_mw,
        mode: 'lines+markers',
        line: { color: '#e06c75', width: 2 },
        name: '系统损耗',
      }], {
        ...plotTheme, title: '系统损耗 (MW)',
        yaxis: { ...plotTheme.yaxis, title: 'MW', range: [lMin - lPad, lMax + lPad] },
      }, { responsive: true });
    } else {
      lossDiv.innerHTML = '';
    }

    // 4. ESS SOC
    const essDiv = document.getElementById('tspfESSChart');
    if (data.ess_soc && data.ess_soc.length > 0) {
      const essTraces = data.ess_soc.map((soc, si) => ({
        x: hrs, y: soc, mode: 'lines+markers',
        name: (data.ess_names || [])[si] || 'ESS ' + si,
        line: { width: 2 },
      }));
      Plotly.newPlot(essDiv, essTraces, {
        ...plotTheme, title: '储能SOC',
        yaxis: { ...plotTheme.yaxis, title: 'SOC', range: [0, 1] },
      }, { responsive: true });
    } else {
      essDiv.innerHTML = '';
    }

    // 4b. AC ESS dispatch (charge/discharge)
    const essDispDiv = document.getElementById('tspfESSDispatchChart');
    if (data.ess_dispatch && data.ess_dispatch.length > 0) {
      const essDispTraces = data.ess_dispatch.map((d, si) => ({
        x: hrs, y: d, type: 'bar',
        name: (data.ess_names || [])[si] || 'ESS ' + si,
      }));
      const allVals = data.ess_dispatch.flat();
      const eMin = Math.min(...allVals, 0), eMax = Math.max(...allVals, 0);
      const ePad = Math.max((eMax - eMin) * 0.15, 0.1);
      Plotly.newPlot(essDispDiv, essDispTraces, {
        ...plotTheme, title: '储能充放电调度 (MW, 正=放电 负=充电)', barmode: 'relative',
        yaxis: { ...plotTheme.yaxis, title: 'MW', range: [eMin - ePad, eMax + ePad] },
      }, { responsive: true });
    } else {
      essDispDiv.innerHTML = '';
    }

    // 5. Total load demand
    const loadDiv = document.getElementById('tspfLoadChart');
    if (data.total_load && data.total_load.length > 0) {
      const dMin = Math.min(...data.total_load), dMax = Math.max(...data.total_load);
      const dPad = Math.max((dMax - dMin) * 0.15, dMax * 0.05 || 0.001);
      Plotly.newPlot(loadDiv, [{
        x: hrs, y: data.total_load,
        mode: 'lines+markers',
        line: { color: '#2c8c99', width: 2 },
        name: '总负荷',
      }], {
        ...plotTheme, title: '负荷需求曲线 (MW)',
        yaxis: { ...plotTheme.yaxis, title: 'MW', range: [dMin - dPad, dMax + dPad] },
      }, { responsive: true });
    } else {
      loadDiv.innerHTML = '';
    }

    // 6. Voltage heatmap (per-bus per-timestep)
    const heatDiv = document.getElementById('tspfVoltHeatmap');
    if (data.vm_matrix && data.vm_matrix.length > 0 && data.bus_labels) {
      Plotly.newPlot(heatDiv, [{
        z: data.vm_matrix,
        x: hrs,
        y: data.bus_labels,
        type: 'heatmap',
        colorscale: [[0,'#e74c3c'],[0.3,'#f39c12'],[0.5,'#27ae60'],[0.7,'#f39c12'],[1,'#e74c3c']],
        zmin: 0.9, zmax: 1.1,
        colorbar: { title: 'V (p.u.)' },
      }], {
        ...plotTheme, title: '母线电压热力图 (p.u.)',
        yaxis: { ...plotTheme.yaxis, title: '', automargin: true },
        margin: { l: 100, r: 15, t: 40, b: 40 },
      }, { responsive: true });
    } else {
      heatDiv.innerHTML = '';
    }

    // 7. PV dispatch (AC PV systems + DC PV arrays)
    const pvDiv = document.getElementById('tspfPVChart');
    const pvTraces = [];
    // AC PV dispatch from backend (computed from solar profile)
    (data.ac_pv_dispatch || []).forEach((d, ki) => pvTraces.push({
      x: hrs, y: d, type: 'bar',
      name: (data.pv_names || [])[ki] || 'PV ' + ki,
      marker: { color: '#e67e22' },
    }));
    // DC PV dispatch from backend
    (data.dc_pv_dispatch || []).forEach((d, ki) => pvTraces.push({
      x: hrs, y: d, type: 'bar',
      name: (data.dc_pv_names || [])[ki] || 'DC_PV ' + ki,
      marker: { color: '#f1c40f' },
    }));
    if (pvTraces.length > 0) {
      Plotly.newPlot(pvDiv, pvTraces, {
        ...plotTheme, title: '光伏出力 (MW)', barmode: 'stack',
        yaxis: { ...plotTheme.yaxis, title: 'MW' },
      }, { responsive: true });
    } else {
      pvDiv.innerHTML = '';
    }

    // 8. DC-side components (DC ESS dispatch)
    const dcDiv = document.getElementById('tspfDCChart');
    const dcTraces = [];
    (data.dc_ess_dispatch || []).forEach((d, ki) => dcTraces.push({
      x: hrs, y: d, mode: 'lines+markers',
      name: (data.dc_ess_names || [])[ki] || 'DC_ESS ' + ki,
      line: { width: 2 },
    }));
    if (dcTraces.length > 0) {
      Plotly.newPlot(dcDiv, dcTraces, {
        ...plotTheme, title: 'DC储能调度 (MW)',
        yaxis: { ...plotTheme.yaxis, title: 'MW' },
      }, { responsive: true });
    } else {
      dcDiv.innerHTML = '';
    }
  }

  // ========== Results Display ==========
  function componentTypeLabel(type) {
    if (typeof COMP !== 'undefined' && COMP.categories) {
      for (const items of Object.values(COMP.categories)) {
        const item = (items || []).find(x => x.type === type);
        if (item && item.label) return item.label;
      }
    }
    return type || '';
  }

  function reliabilityComponentTypeLabel(type) {
    const labels = {
	      generator: '发电机',
	      Generator: '发电机',
	      ac_branch: 'AC线路',
	      ACBranch: 'AC线路',
	      dc_branch: 'DC线路',
	      DCBranch: 'DC线路',
	      vsc_converter: 'VSC换流器',
	      VSCConverter: 'VSC换流器',
	      static_generator: '静态电源',
	      StaticGen: '静态电源',
	      renewable_gen: '新能源电源',
	      RenewableGen: '新能源电源',
	      storage: '储能',
	      ACStorage: '储能',
	      transformer_2w: '双绕组变压器',
	      Transformer2W: '双绕组变压器',
	      transformer_3w: '三绕组变压器',
	      Transformer3W: '三绕组变压器',
	      dcdc_converter: 'DC/DC变换器',
	      DCDCConverter: 'DC/DC变换器',
	      dc_circuit_breaker: 'DC断路器',
	      DCCircuitBreaker: 'DC断路器',
	      dc_storage: 'DC储能',
	      DCStorage: 'DC储能',
	      dc_pv_array: 'DC光伏',
	      DCPVArray: 'DC光伏',
	      dc_static_generator: 'DC静态电源',
	      DCStaticGen: 'DC静态电源',
	      ac_switch: 'AC开关',
	      ACSwitch: 'AC开关',
	      ac_circuit_breaker: 'AC断路器',
	      ACCircuitBreaker: 'AC断路器',
	      ac_pv_system: 'AC光伏',
	      ACPVSystem: 'AC光伏',
	      dc_static_generator_ac: 'DC分布式电源',
	      DCStaticGenAC: 'DC分布式电源',
	    };
    return labels[type] || componentTypeLabel(type) || type || '—';
  }

  // Readable labels for failure-mode taxonomy tokens (activation / cause /
  // consequence) emitted by the failure-mode FMEA engine.
  function reliabilityFmLabel(value) {
    const labels = {
      // activation
      passive: '时基/被动', active_on_demand: '按需动作',
      // cause
      physical: '物理', cyber_control: '网络控制', communication: '通信',
      measurement: '测量', protection_logic: '保护逻辑',
      human_operation: '人为操作', scheduled: '计划检修',
      // consequence
      forced_outage: '强制停运', derating: '降容', stuck_open: '卡断开',
      stuck_closed: '卡闭合', fail_to_open: '拒动(分闸)', fail_to_close: '拒动(合闸)',
      fail_to_trip: '拒动(跳闸)', nuisance_trip: '误动跳闸',
      control_unavailable: '控制不可用', setpoint_frozen: '设定值冻结',
      measurement_bias: '测量偏差', communication_loss: '通信中断',
      grid_forming_unavailable: '构网能力丧失', protection_zone_trip: '保护区扩大',
    };
    if (value === null || value === undefined || value === '') return '—';
    return labels[value] || value;
  }

  function reliabilityContingencyCompId(c, maps) {
    return rowCanvasCompId(c, maps);
  }

  function renderAllPowerFlowComponentStatus(data, busMap, options = {}) {
    data = normalizePowerFlowResult(data);
    const section = document.getElementById(options.sectionId || 'pfAllComponentsSection');
    const div = document.getElementById(options.resultsId || 'pfAllComponentsResults');
    if (!section || !div) return;

    const components = (typeof Canvas !== 'undefined' && Canvas.state && Array.isArray(Canvas.state.components))
      ? Canvas.state.components : [];
    if (!components.length) {
      section.style.display = 'none';
      div.innerHTML = '';
      return;
    }
    section.style.display = '';

    const maps = busMap || (Canvas.getCompBusMap ? Canvas.getCompBusMap() : {});
    const toNum = (v) => {
      const n = Number(v);
      return Number.isFinite(n) ? n : null;
    };
    const fmt = (v, d = 3) => {
      const n = toNum(v);
      return n === null ? '-' : n.toFixed(d);
    };
    const fmtP = (v, d = 3) => {
      const n = toNum(v);
      return n === null ? '-' : `${pFmt(n, d)} ${pUnit()}`;
    };
    const fmtQ = (v, d = 3) => {
      const n = toNum(v);
      return n === null ? '-' : `${pFmt(n, d)} ${qUnit()}`;
    };
    const pct = (v, d = 1) => {
      const n = toNum(v);
      return n === null ? '-' : `${n.toFixed(d)}%`;
    };
    const lineHtml = (items) => items
      .filter(x => x !== undefined && x !== null && String(x) !== '')
      .map(x => escapeHtml(String(x)))
      .join('<br>');
    const compName = (comp) => {
      const p = comp.params || {};
      return p.name || (COMP.defaults && COMP.defaults[comp.type] && COMP.defaults[comp.type].name) || comp.type || '';
    };

    const compToBus = {};
    Object.entries(maps.ac || {}).forEach(([idx, cid]) => { compToBus[Number(cid)] = { domain: 'ac', index: Number(idx) }; });
    Object.entries(maps.dc || {}).forEach(([idx, cid]) => { compToBus[Number(cid)] = { domain: 'dc', index: Number(idx) }; });

    const resultConnectedBuses = (compId) => {
      const out = [];
      for (const conn of Canvas.state.connections || []) {
        let otherId = null, portId = '';
        if (Number(conn.from.compId) === Number(compId)) {
          otherId = conn.to.compId;
          portId = conn.from.portId || '';
        } else if (Number(conn.to.compId) === Number(compId)) {
          otherId = conn.from.compId;
          portId = conn.to.portId || '';
        }
        if (otherId === null) continue;
        const b = compToBus[Number(otherId)];
        if (b) out.push({ ...b, portId, compId: otherId });
      }
      return out;
    };

    const backendRows = Array.isArray(data.component_results) ? data.component_results : [];
    if (backendRows.length) {
      const domainOf = (comp) => {
        if (comp.type === 'ac_bus' || comp.type === 'external_grid' || comp.type === 'generator' ||
            comp.type === 'pv_system' || comp.type === 'renewable_gen' || comp.type === 'load' ||
            comp.type === 'flexible_load' || comp.type === 'asymmetric_load' || comp.type === 'shunt' ||
            comp.type === 'motor' || comp.type === 'charger' || comp.type === 'charging_station' ||
            comp.type === 'vpp' || comp.type === 'microgrid') return 'AC';
        if (comp.type === 'dc_bus' || comp.type === 'dc_branch' || comp.type === 'dc_load' ||
            comp.type === 'dc_storage' || comp.type === 'dc_pv_array' || comp.type === 'dcdc_converter') return 'DC';
        if (comp.type === 'vsc_converter' || comp.type === 'energy_router') return 'ACDC';
        const buses = resultConnectedBuses(comp.id);
        const hasAc = buses.some(b => b.domain === 'ac');
        const hasDc = buses.some(b => b.domain === 'dc');
        if (hasAc && hasDc) return 'ACDC';
        if (hasDc) return 'DC';
        return 'AC';
      };
      const rowTypeOf = (comp) => (comp.type === 'transformer_2w' && (comp.params || {})._from_branch)
        ? 'ac_branch'
        : comp.type;
      const orderByTypeDomain = {};
      components.forEach(comp => {
        const key = `${rowTypeOf(comp)}|${domainOf(comp)}`;
        if (!orderByTypeDomain[key]) orderByTypeDomain[key] = [];
        orderByTypeDomain[key].push(comp.id);
      });
      const positionOf = (comp) => {
        const key = `${rowTypeOf(comp)}|${domainOf(comp)}`;
        return (orderByTypeDomain[key] || []).findIndex(id => Number(id) === Number(comp.id));
      };
      const usedRows = new Set();
      const rowDomainMatches = (rowDomain, compDomain) => {
        const rd = String(rowDomain || '').toUpperCase();
        if (!rd || rd === compDomain) return true;
        return compDomain === 'ACDC' && (rd === 'AC' || rd === 'DC');
      };
      const takeBackendRow = (comp) => {
        const t = rowTypeOf(comp);
        const d = domainOf(comp);
        const pos = positionOf(comp);
        const idx = Number((comp.params || {}).index);
        const candidates = backendRows
          .map((row, rowIndex) => ({ row, rowIndex }))
          .filter(x => !usedRows.has(x.rowIndex) &&
            String(x.row.canvas_type || '') === t &&
            rowDomainMatches(x.row.domain, d));
        let hit = null;
        if (Number.isFinite(idx)) {
          hit = candidates.find(x => Number(x.row.index) === idx) || null;
        }
        if (!hit && pos >= 0) {
          hit = candidates.find(x => Number(x.row.position) === pos) || null;
        }
        if (!hit) hit = candidates[0] || null;
        if (hit) usedRows.add(hit.rowIndex);
        return hit ? hit.row : null;
      };
      const fmtMetric = (m) => {
        if (!m) return '';
        const label = m.label || '';
        if (m.quantity === 'text') return `${label}=${m.value ?? '-'}`;
        const n = toNum(m.value);
        if (n === null) return `${label}=-`;
        const precision = Number.isFinite(Number(m.precision)) ? Number(m.precision) : 3;
        const quantity = String(m.quantity || '').toLowerCase();
        const unit = String(m.unit || '');
        if (quantity === 'p') return `${label}=${pFmt(n, precision)} ${pUnit()}`;
        if (quantity === 'q') return `${label}=${pFmt(n, precision)} ${qUnit()}`;
        const suffix = unit ? ` ${unit}` : '';
        return `${label}=${n.toFixed(precision)}${suffix}`;
      };
      let html = '<table><thead><tr><th>ID</th><th>类型</th><th>名称</th><th>状态</th><th>连接</th><th>潮流/电压/功率</th><th>备注</th></tr></thead><tbody>';
      components.forEach(comp => {
        const row = takeBackendRow(comp);
        const detail = row && Array.isArray(row.metrics) ? row.metrics.map(fmtMetric) : [];
        const notes = row && Array.isArray(row.notes) ? row.notes : [];
        const buses = resultConnectedBuses(comp.id);
        const fallbackConnection = buses.length
          ? buses.map(b => `${b.portId ? b.portId + ':' : ''}${b.domain === 'dc' ? 'DC' : 'AC'} Bus ${b.index}`).join('; ')
          : '-';
        html += `<tr data-comp-id="${comp.id}">`;
        html += `<td>${comp.id}</td>`;
        html += `<td>${escapeHtml(row?.type_label || componentTypeLabel(comp.type))}</td>`;
        html += `<td>${escapeHtml(row?.name || compName(comp))}</td>`;
        html += `<td>${escapeHtml(row?.status || (data.converged ? '无分项结果' : '未收敛'))}</td>`;
        html += `<td>${escapeHtml(row?.connection || fallbackConnection)}</td>`;
        html += `<td>${lineHtml(detail.length ? detail : ['-'])}</td>`;
        html += `<td>${lineHtml(row ? notes : ['缺少潮流后元件行'])}</td>`;
        html += '</tr>';
      });
      html += '</tbody></table>';
      div.innerHTML = html;
      return;
    }

    const orders = {};
    const addOrder = (key, comp) => {
      if (!orders[key]) orders[key] = [];
      orders[key].push(comp.id);
    };
    components.forEach(comp => {
      const p = comp.params || {};
      switch (comp.type) {
        case 'ac_bus': addOrder('ac', comp); break;
        case 'dc_bus': addOrder('dc', comp); break;
        case 'ac_branch': addOrder('branch', comp); break;
        case 'transformer_2w':
          if (p._from_branch) addOrder('branch', comp);
          else addOrder('trafo', comp);
          break;
        case 'generator': addOrder('gen', comp); break;
        case 'load': addOrder('load', comp); break;
        case 'external_grid': addOrder('extGrid', comp); break;
        case 'storage': addOrder('storage', comp); break;
        case 'pv_system': addOrder('pv', comp); break;
        case 'renewable_gen': addOrder('renGen', comp); break;
        case 'static_generator': addOrder('sgen', comp); break;
        case 'switch_comp': addOrder('sw', comp); break;
        case 'circuit_breaker': addOrder('cb', comp); break;
        case 'motor': addOrder('motor', comp); break;
        case 'dc_branch': addOrder('dcBranch', comp); break;
        case 'dc_load': addOrder('dcLoad', comp); break;
        case 'dc_pv_array': addOrder('dcPv', comp); break;
        case 'vsc_converter': addOrder('vsc', comp); break;
        case 'dcdc_converter': addOrder('dcdcConverter', comp); break;
        case 'energy_router': addOrder('energyRouter', comp); break;
        case 'shunt': addOrder('shunt', comp); break;
        case 'transformer_3w': addOrder('trafo3w', comp); break;
        case 'flexible_load': addOrder('flexLoad', comp); break;
        case 'asymmetric_load': addOrder('asymLoad', comp); break;
        case 'charger': addOrder('charger', comp); break;
        case 'charging_station': addOrder('chargingStation', comp); break;
        case 'mobile_storage': addOrder('mobileStorage', comp); break;
        case 'vpp': addOrder('vpp', comp); break;
        case 'microgrid': addOrder('microgrid', comp); break;
        default: addOrder(comp.type, comp); break;
      }
    });

    const posOf = (key, compId) => {
      const arr = orders[key] || [];
      return arr.findIndex(id => Number(id) === Number(compId));
    };
    const mapKeyOf = (key, compId) => {
      const obj = maps[key] || {};
      for (const [idx, cid] of Object.entries(obj)) {
        if (Number(cid) === Number(compId)) {
          const n = Number(idx);
          return Number.isFinite(n) ? n : idx;
        }
      }
      return null;
    };
    const rowByIndexOrPos = (arr, key, compId) => {
      const rows = Array.isArray(arr) ? arr : [];
      const k = mapKeyOf(key, compId);
      if (k !== null) {
        const hit = rows.find(r =>
          Number(r.index) === Number(k) ||
          Number(r.id) === Number(k) ||
          Number(r.router_index) === Number(k)
        );
        if (hit) return hit;
      }
      const pos = posOf(key, compId);
      return pos >= 0 && pos < rows.length ? rows[pos] : null;
    };
    const componentMetric = (row, label) => {
      const metric = Array.isArray(row?.metrics)
        ? row.metrics.find(m => m.label === label)
        : null;
      if (!metric || String(metric.quantity || '').toLowerCase() === 'text') return null;
      const value = Number(metric.value);
      return Number.isFinite(value) ? value : null;
    };

    const geoBus = (domain, idx) => (data.geo_buses || []).find(b =>
      String(b.type || '').toLowerCase() === (domain === 'dc' ? 'dc' : 'ac') &&
      Number(b.id) === Number(idx)
    );
    const busFlowBalance = { ac: {}, dc: {} };
    const addBal = (domain, bus, p, q = 0) => {
      const idx = Number(bus);
      if (!Number.isFinite(idx)) return;
      if (!busFlowBalance[domain][idx]) busFlowBalance[domain][idx] = { p: 0, q: 0 };
      busFlowBalance[domain][idx].p += Number(p) || 0;
      busFlowBalance[domain][idx].q += Number(q) || 0;
    };
    (data.geo_ac_branches || []).forEach(br => {
      addBal('ac', br.from, br.pf_mw, br.qf_mvar);
      addBal('ac', br.to, br.pt_mw, br.qt_mvar);
    });
    firstNonEmptyArray(data.geo_dc_branches, data.dc_branch_flows).forEach(br => {
      addBal('dc', br.from ?? br.from_bus, br.pf_mw, 0);
      addBal('dc', br.to ?? br.to_bus, br.pt_mw, 0);
    });
    (data.geo_trafo3w || []).forEach(tf => {
      addBal('ac', tf.hv_bus, tf.p_hv_mw, tf.q_hv_mvar);
      addBal('ac', tf.mv_bus, tf.p_mv_mw, tf.q_mv_mvar);
      addBal('ac', tf.lv_bus, tf.p_lv_mw, tf.q_lv_mvar);
    });
    firstNonEmptyArray(data.vsc_transfers, data.geo_vsc).forEach(v => {
      addBal('ac', v.bus_ac, -(v.p_ac_mw || 0), -(v.q_ac_mvar || 0));
      addBal('dc', v.bus_dc, -(v.p_dc_mw || 0), 0);
    });
    firstNonEmptyArray(data.dcdc_transfers, data.geo_dcdc).forEach(d => {
      addBal('dc', d.bus_in, d.p_in_mw, 0);
      addBal('dc', d.bus_out, -(d.p_out_mw || 0), 0);
    });

    const componentInjection = (comp) => {
      const p = comp.params || {};
      const scale = toNum(p.scaling) ?? 1;
      switch (comp.type) {
        case 'load':
        case 'flexible_load':
          return { p: -((toNum(p.p_mw) ?? 0) * scale), q: -((toNum(p.q_mvar) ?? 0) * scale) };
        case 'asymmetric_load':
          return {
            p: -(((toNum(p.pa_mw) ?? 0) + (toNum(p.pb_mw) ?? 0) + (toNum(p.pc_mw) ?? 0)) * scale),
            q: -(((toNum(p.qa_mvar) ?? 0) + (toNum(p.qb_mvar) ?? 0) + (toNum(p.qc_mvar) ?? 0)) * scale),
          };
        case 'dc_load':
          return { p: -((toNum(p.p_mw) ?? 0) * scale), q: 0 };
        case 'dc_storage': {
          const row = rowByIndexOrPos(firstNonEmptyArray(data.dc_storage_results, (data.component_results || []).filter(r => r.canvas_type === 'dc_storage')), 'dcStorage', comp.id);
          return { p: toNum(row?.p_mw) ?? componentMetric(row, 'P计算') ?? toNum(p.p_mw) ?? 0, q: 0 };
        }
        case 'storage':
        case 'mobile_storage':
          return { p: toNum(p.p_mw) ?? 0, q: toNum(p.q_mvar) ?? 0 };
        case 'pv_system':
        case 'renewable_gen':
        case 'static_generator':
          return { p: (toNum(p.p_mw) ?? 0) * scale, q: (toNum(p.q_mvar) ?? 0) * scale };
        case 'dc_pv_array':
          return { p: toNum(p.p_set_mw) ?? 0, q: 0 };
        case 'shunt':
          return { p: toNum(p.gs_mw) ?? 0, q: toNum(p.bs_mvar) ?? 0 };
        case 'charging_station': {
          const pKw = toNum(p.p_total_kw);
          const qKvar = toNum(p.q_total_kvar);
          const pMw = pKw !== null ? pKw / 1000 : ((toNum(p.max_power_kw) ?? 0) * (toNum(p.utilization_rate) ?? 0) * (toNum(p.simultaneity_factor) ?? 1) / 1000);
          return { p: -pMw, q: -(qKvar !== null ? qKvar / 1000 : 0) };
        }
        case 'vpp':
          return { p: toNum(p.p_output_mw) ?? 0, q: toNum(p.q_output_mvar) ?? 0 };
        case 'microgrid':
          return { p: -(toNum(p.p_exchange_mw) ?? 0), q: 0 };
        default:
          return { p: 0, q: 0 };
      }
    };
    const componentsAtBus = (domain, busIdx, typeFilter = null) => components.filter(c => {
      if (typeFilter && !typeFilter.has(c.type)) return false;
      const buses = resultConnectedBuses(c.id);
      return buses.some(b => b.domain === domain && Number(b.index) === Number(busIdx));
    });
    const solvedExternalGridInjection = (comp) => {
      const buses = resultConnectedBuses(comp.id);
      const ac = buses.find(b => b.domain === 'ac') || ((comp.params || {}).bus ? { domain: 'ac', index: (comp.params || {}).bus } : null);
      if (!ac) return null;
      const idx = Number(ac.index);
      const bal = busFlowBalance.ac[idx] || { p: 0, q: 0 };
      let nonGridP = 0, nonGridQ = 0;
      componentsAtBus('ac', idx).forEach(c => {
        if (c.id === comp.id || c.type === 'external_grid' || (c.params || {}).in_service === false) return;
        const inj = componentInjection(c);
        nonGridP += inj.p;
        nonGridQ += inj.q;
      });
      return { p: bal.p - nonGridP, q: bal.q - nonGridQ, bus: idx };
    };
    const voltageText = (domain, idx) => {
      const gb = geoBus(domain, idx);
      if (domain === 'dc') {
        const v = gb ? gb.vm_pu : (data.vdc || [])[posOf('dc', maps.dc ? maps.dc[idx] : null)];
        return toNum(v) === null ? '' : `Vdc=${fmt(v, 6)} pu`;
      }
      const acCompId = maps.ac ? maps.ac[idx] : null;
      const pos = posOf('ac', acCompId);
      const vm = gb ? gb.vm_pu : (data.vm || [])[pos];
      const va = gb ? gb.va_rad : (data.va || [])[pos];
      const parts = [];
      if (toNum(vm) !== null) parts.push(`Vm=${fmt(vm, 6)} pu`);
      if (toNum(va) !== null) parts.push(`Va=${fmt(Number(va) * 180 / Math.PI, 4)} deg`);
      return parts.join(', ');
    };
    const busLabel = (domain, idx) => `${domain === 'dc' ? 'DC' : 'AC'} Bus ${idx}`;

    const attachedObjects = (compId) => {
      const out = [];
      for (const conn of Canvas.state.connections || []) {
        let otherId = null;
        if (Number(conn.from.compId) === Number(compId)) otherId = conn.to.compId;
        else if (Number(conn.to.compId) === Number(compId)) otherId = conn.from.compId;
        if (otherId === null) continue;
        const other = components.find(c => Number(c.id) === Number(otherId));
        if (other) out.push(`${componentTypeLabel(other.type)}#${other.id}`);
      }
      return out;
    };
    const connectionText = (comp) => {
      const directBus = compToBus[Number(comp.id)];
      if (directBus) {
        const attached = attachedObjects(comp.id).filter(x => !x.includes('母线'));
        const suffix = attached.length
          ? `；连接${attached.length}个元件: ${attached.slice(0, 5).join(', ')}${attached.length > 5 ? '...' : ''}`
          : '';
        return `${busLabel(directBus.domain, directBus.index)}${suffix}`;
      }
      const buses = resultConnectedBuses(comp.id);
      if (buses.length) {
        return buses.map(b => `${b.portId ? b.portId + ':' : ''}${busLabel(b.domain, b.index)}`).join('; ');
      }
      const p = comp.params || {};
      switch (comp.type) {
        case 'vsc_converter': return `AC Bus ${p.bus_ac || '-'}; DC Bus ${p.bus_dc || '-'}`;
        case 'dcdc_converter': return `DC Bus in ${p.bus_in || '-'}; DC Bus out ${p.bus_out || '-'}`;
        case 'dc_branch': return `DC Bus ${p.from_bus || '-'} -> ${p.to_bus || '-'}`;
        case 'ac_branch': return `AC Bus ${p.from_bus || '-'} -> ${p.to_bus || '-'}`;
        case 'transformer_2w': return `HV ${p.hv_bus || '-'}; LV ${p.lv_bus || '-'}`;
        case 'transformer_3w': return `HV ${p.hv_bus || '-'}; MV ${p.mv_bus || '-'}; LV ${p.lv_bus || '-'}`;
        case 'switch_comp':
        case 'circuit_breaker': return `Bus ${p.from_bus || p.bus_from || '-'} -> ${p.to_bus || p.bus_to || '-'}`;
        case 'dc_load':
        case 'dc_pv_array': return `DC Bus ${p.bus || '-'}`;
        default:
          if (p.bus !== undefined) return `Bus ${p.bus || '-'}`;
          if (p.pcc_bus !== undefined) return `PCC Bus ${p.pcc_bus || '-'}`;
          return '-';
      }
    };

    const describe = (comp) => {
      const p = comp.params || {};
      const note = [];
      let solved = false;
      let detail = [];

      switch (comp.type) {
        case 'ac_bus': {
          const bus = compToBus[Number(comp.id)];
          const gb = bus ? geoBus('ac', bus.index) : null;
          const pos = posOf('ac', comp.id);
          const vm = gb ? gb.vm_pu : (data.vm || [])[pos];
          const va = gb ? gb.va_rad : (data.va || [])[pos];
          solved = toNum(vm) !== null;
          detail = [
            `类型=${gb?.bus_type || p.bus_type || '-'}`,
            `Vm=${fmt(vm, 6)} pu`,
            `Va=${toNum(va) === null ? '-' : fmt(Number(va) * 180 / Math.PI, 4)} deg`,
            `Pd=${fmtP(gb?.pd_mw ?? p.pd_mw ?? 0, 3)}`,
            `Qd=${fmtQ(gb?.qd_mvar ?? p.qd_mvar ?? 0, 3)}`,
          ];
          break;
        }
        case 'dc_bus': {
          const bus = compToBus[Number(comp.id)];
          const gb = bus ? geoBus('dc', bus.index) : null;
          const pos = posOf('dc', comp.id);
          const vdc = gb ? gb.vm_pu : (data.vdc || [])[pos];
          solved = toNum(vdc) !== null;
          detail = [
            `类型=${gb?.bus_type || p.bus_type || '-'}`,
            `Vdc=${fmt(vdc, 6)} pu`,
            `Pd=${fmtP(gb?.pd_mw ?? p.pd_mw ?? 0, 3)}`,
          ];
          break;
        }
        case 'generator': {
          const g = rowByIndexOrPos(data.geo_gen, 'gen', comp.id);
          solved = !!g || data.converged;
          detail = [
            `Pg=${fmtP(g?.pg_mw ?? p.pg_mw, 3)}`,
            `Qg=${fmtQ(g?.qg_mvar ?? p.qg_mvar, 3)}`,
            `Vg=${fmt(g?.vg_pu ?? p.vg_pu, 4)} pu`,
            `Slack=${(g?.is_slack ?? p.is_slack) ? '是' : '否'}`,
          ];
          if (!g) note.push('本次潮流按发电机给定注入参与计算');
          break;
        }
        case 'external_grid': {
          const buses = resultConnectedBuses(comp.id);
          const ac = buses.find(b => b.domain === 'ac') || (p.bus ? { domain: 'ac', index: p.bus } : null);
          const eg = solvedExternalGridInjection(comp);
          detail = [
            `P平衡=${fmtP(eg?.p, 3)}`,
            `Q平衡=${fmtQ(eg?.q, 3)}`,
            ac ? voltageText('ac', ac.index) : '',
            `V控制=${fmt(p.vm_pu, 4)} pu`,
            `角度=${fmt(p.va_deg, 3)} deg`,
          ];
          solved = !!eg || !!(ac && voltageText('ac', ac.index));
          note.push('P/Q为根据潮流后母线功率平衡反算的外部电网结果');
          break;
        }
        case 'load':
        case 'flexible_load': {
          const scale = toNum(p.scaling) ?? 1;
          detail = [
            `P=${fmtP((toNum(p.p_mw) ?? 0) * scale, 3)}`,
            `Q=${fmtQ((toNum(p.q_mvar) ?? 0) * scale, 3)}`,
            comp.type === 'flexible_load' ? `上调=${fmtP(p.flex_up_mw, 3)}` : '',
            comp.type === 'flexible_load' ? `下调=${fmtP(p.flex_down_mw, 3)}` : '',
          ];
          solved = data.converged;
          note.push('本次潮流计算采用的负荷消耗');
          break;
        }
        case 'asymmetric_load': {
          const scale = toNum(p.scaling) ?? 1;
          const pa = (toNum(p.pa_mw) ?? 0) * scale;
          const pb = (toNum(p.pb_mw) ?? 0) * scale;
          const pc = (toNum(p.pc_mw) ?? 0) * scale;
          detail = [`Pa=${fmtP(pa, 3)}`, `Pb=${fmtP(pb, 3)}`, `Pc=${fmtP(pc, 3)}`, `Psum=${fmtP(pa + pb + pc, 3)}`];
          solved = data.converged;
          note.push('本次潮流计算采用的相别负荷消耗');
          break;
        }
        case 'dc_storage':
        case 'storage':
        case 'mobile_storage': {
          const dcRow = comp.type === 'dc_storage'
            ? rowByIndexOrPos(firstNonEmptyArray(data.dc_storage_results, (data.component_results || []).filter(r => r.canvas_type === 'dc_storage')), 'dcStorage', comp.id)
            : null;
          const pSolved = comp.type === 'dc_storage'
            ? (toNum(dcRow?.p_mw) ?? componentMetric(dcRow, 'P计算') ?? toNum(p.p_mw))
            : toNum(p.p_mw);
          const pBalanceShare = comp.type === 'dc_storage'
            ? (toNum(dcRow?.p_balance_share_mw) ?? componentMetric(dcRow, '平衡分摊'))
            : null;
          detail = [
            `P计算=${fmtP(pSolved, 3)}`,
            pBalanceShare !== null && Math.abs(pBalanceShare) > 1e-6 ? `平衡分摊=${fmtP(pBalanceShare, 3)}` : '',
            `Q计算=${fmtQ(p.q_mvar, 3)}`,
            `P额定=${fmtP(p.p_rated_mw, 3)}`,
            `E额定=${fmt(p.e_rated_mwh, 3)} MWh`,
            `SOC=${fmt(dcRow?.soc ?? p.soc_init, 4)}`,
            comp.type === 'mobile_storage' ? `移动状态=${p.status || '-'}` : '',
            comp.type === 'mobile_storage' ? `目标母线=${p.target_bus || '-'}` : '',
          ];
          solved = data.converged;
          note.push(pBalanceShare !== null && Math.abs(pBalanceShare) > 1e-6
            ? '本次潮流计算采用的储能注入；正值放电，负值充电；DC_V平衡功率已投影到该储能'
            : '本次潮流计算采用的储能注入；正值放电，负值充电');
          break;
        }
        case 'pv_system':
        case 'renewable_gen':
        case 'static_generator': {
          detail = [
            `P=${fmtP(p.p_mw, 3)}`,
            `Q=${fmtQ(p.q_mvar, 3)}`,
            `额定=${fmtP(p.p_rated_mw ?? p.sn_mva, 3)}`,
            p.control_mode ? `控制=${p.control_mode}` : '',
            p.sgen_type ? `类型=${p.sgen_type}` : '',
          ];
          solved = data.converged;
          note.push('本次潮流计算采用的电源注入');
          break;
        }
        case 'dc_load': {
          const scale = toNum(p.scaling) ?? 1;
          detail = [`P=${fmtP((toNum(p.p_mw) ?? 0) * scale, 3)}`];
          solved = data.converged;
          note.push('本次潮流计算采用的DC负荷消耗');
          break;
        }
        case 'dc_pv_array': {
          detail = [`P计算=${fmtP(p.p_set_mw, 3)}`, `辐照度=${fmt(p.irradiance, 1)} W/m2`, `温度=${fmt(p.temperature, 1)} degC`];
          solved = data.converged;
          note.push('本次潮流计算采用的DC光伏注入');
          break;
        }
        case 'ac_branch':
        case 'transformer_2w': {
          const br = rowByIndexOrPos(data.geo_ac_branches, 'branch', comp.id);
          if (br) {
            solved = true;
            detail = [
              `${br.from ?? p.from_bus ?? p.hv_bus ?? '-'} -> ${br.to ?? p.to_bus ?? p.lv_bus ?? '-'}`,
              `Pf=${fmtP(br.pf_mw, 3)}`,
              `Pt=${fmtP(br.pt_mw, 3)}`,
              `Qf=${fmtQ(br.qf_mvar, 3)}`,
              `Qt=${fmtQ(br.qt_mvar, 3)}`,
              `Loss=${fmtP(br.loss_mw ?? ((br.pf_mw || 0) + (br.pt_mw || 0)), 3)}`,
              `Loading=${pct(br.loading_pct, 1)}`,
            ];
          } else if (comp.type === 'transformer_2w') {
            detail = [`HV=${p.hv_bus || '-'}`, `LV=${p.lv_bus || '-'}`, `Sn=${fmt(p.sn_mva, 3)} MVA`, `Tap=${fmt(p.tap_pos, 3)}`];
            note.push('本次潮流无该双绕组变压器分项返回');
          } else {
            detail = [`${p.from_bus || '-'} -> ${p.to_bus || '-'}`, `R=${fmt(p.r_pu, 6)} pu`, `X=${fmt(p.x_pu, 6)} pu`, `Rate=${fmt(p.rate_a_mva, 3)} MVA`];
            note.push('本次潮流无该支路分项返回');
          }
          break;
        }
        case 'dc_branch': {
          const br = rowByIndexOrPos(firstNonEmptyArray(data.geo_dc_branches, data.dc_branch_flows), 'dcBranch', comp.id);
          solved = !!br;
          detail = [
            `${br?.from ?? br?.from_bus ?? p.from_bus ?? '-'} -> ${br?.to ?? br?.to_bus ?? p.to_bus ?? '-'}`,
            `Pf=${fmtP(br?.pf_mw, 3)}`,
            `Pt=${fmtP(br?.pt_mw, 3)}`,
            `Loss=${fmtP(br?.loss_mw ?? ((br?.pf_mw || 0) + (br?.pt_mw || 0)), 3)}`,
            br?.loading_pct !== undefined ? `Loading=${pct(br.loading_pct, 1)}` : '',
          ];
          if (!br) note.push('本次潮流无该DC支路分项返回');
          break;
        }
        case 'vsc_converter': {
          const v = rowByIndexOrPos(firstNonEmptyArray(data.vsc_transfers, data.geo_vsc), 'vsc', comp.id);
          solved = !!v;
          detail = [
            `模式=${p.control_mode || '-'}`,
            `Pac=${fmtP(v?.p_ac_mw, 3)}`,
            `Qac=${fmtQ(v?.q_ac_mvar, 3)}`,
            `Pdc=${fmtP(v?.p_dc_mw, 3)}`,
            `Loss=${fmtP(v?.loss_mw, 3)}`,
            !v ? `P计算=${fmtP(p.p_set_mw, 3)}` : '',
            !v ? `Q计算=${fmtQ(p.q_set_mvar, 3)}` : '',
          ];
          if (!v) note.push('本次潮流无该VSC分项返回');
          break;
        }
        case 'dcdc_converter': {
          const d = rowByIndexOrPos(firstNonEmptyArray(data.dcdc_transfers, data.geo_dcdc), 'dcdcConverter', comp.id);
          solved = !!d;
          detail = [
            `模式=${p.control_mode || '-'}`,
            `Pin=${fmtP(d?.p_in_mw, 3)}`,
            `Pout=${fmtP(d?.p_out_mw, 3)}`,
            `Loss=${fmtP(d?.loss_mw, 3)}`,
            !d ? `P计算=${fmtP(p.p_ref_mw, 3)}` : '',
            `Vref=${fmt(p.v_ref_pu, 4)} pu`,
          ];
          if (!d) note.push('本次潮流无该DC/DC分项返回');
          break;
        }
        case 'transformer_3w': {
          const tf = rowByIndexOrPos(data.geo_trafo3w, 'trafo3w', comp.id);
          solved = !!tf;
          detail = tf ? [
            `HV=${tf.hv_bus} P=${fmtP(tf.p_hv_mw, 3)}`,
            `MV=${tf.mv_bus} P=${fmtP(tf.p_mv_mw, 3)}`,
            `LV=${tf.lv_bus} P=${fmtP(tf.p_lv_mw, 3)}`,
            `Loss=${fmtP(tf.loss_mw, 3)}`,
            `Loading=${pct(tf.loading_pct, 1)}`,
          ] : [`HV=${p.hv_bus || '-'}`, `MV=${p.mv_bus || '-'}`, `LV=${p.lv_bus || '-'}`, `SnHV=${fmt(p.sn_hv_mva, 3)} MVA`];
          if (!tf) note.push('本次潮流无该三绕组变压器分项返回');
          break;
        }
        case 'switch_comp':
        case 'circuit_breaker': {
          const buses = resultConnectedBuses(comp.id);
          const isDc = buses.some(b => b.domain === 'dc');
          const rows = comp.type === 'switch_comp'
            ? data.ac_switch_flows
            : (isDc ? data.dc_circuit_breaker_flows : data.ac_circuit_breaker_flows);
          const key = comp.type === 'switch_comp' ? 'sw' : 'cb';
          const fl = rowByIndexOrPos(rows, key, comp.id);
          solved = !!fl || data.converged;
          detail = [
            `状态=${p.closed !== false ? '合闸' : '分闸'}`,
            `From=${fl?.from ?? fl?.from_bus ?? p.from_bus ?? p.bus_from ?? '-'}`,
            `To=${fl?.to ?? fl?.to_bus ?? p.to_bus ?? p.bus_to ?? '-'}`,
            `Pf=${fmtP(fl?.pf_mw, 3)}`,
            `Pt=${fmtP(fl?.pt_mw, 3)}`,
          ];
          if (!isDc) {
            detail.push(`Qf=${fmtQ(fl?.qf_mvar, 3)}`);
            detail.push(`Qt=${fmtQ(fl?.qt_mvar, 3)}`);
          }
          if (fl?.loading_pct !== undefined) detail.push(`Loading=${pct(fl.loading_pct, 1)}`);
          if (comp.type === 'circuit_breaker') detail.push(`额定电流=${fmt(p.rated_current_ka ?? p.i_rated_ka, 3)} kA`);
          if (!fl) note.push('本次潮流无该开关/断路器分项返回');
          break;
        }
        case 'shunt': {
          detail = [`Gs=${fmtP(p.gs_mw, 3)}`, `Bs=${fmtQ(p.bs_mvar, 3)}`, `投切=${p.switchable ? '可投切' : '固定'}`, `步=${p.current_step || 1}/${p.n_steps || 1}`];
          solved = data.converged;
          note.push('本次潮流计算采用的并联补偿注入');
          break;
        }
        case 'motor': {
          detail = [`Sn=${fmt(p.sn_mva, 3)} MVA`, `cosPhi=${fmt(p.cos_phi, 3)}`, `效率=${fmt(p.efficiency, 4)}`];
          solved = data.converged;
          note.push('本次潮流计算采用的电动机负荷模型');
          break;
        }
        case 'charger': {
          detail = [`类型=${p.charger_type || '-'}`, `额定=${fmt(p.p_rated_kw, 3)} kW`, `最大充电=${fmt(p.p_ch_max_kw, 3)} kW`, `V2G=${p.v2g_capable ? '是' : '否'}`];
          solved = data.converged;
          note.push('本次潮流计算采用的充电桩模型');
          break;
        }
        case 'charging_station': {
          detail = [`快充=${p.n_fast || 0}`, `慢充=${p.n_slow || 0}`, `最大=${fmt(p.max_power_kw, 3)} kW`, `同时率=${fmt(p.simultaneity_factor, 3)}`];
          solved = data.converged;
          note.push('本次潮流计算采用的充电站负荷模型');
          break;
        }
        case 'energy_router': {
          const er = rowByIndexOrPos(data.geo_er, 'energyRouter', comp.id);
          solved = !!er;
          if (er && Array.isArray(er.ports)) {
            detail = er.ports.map(pt =>
              `Port ${pt.port_index}@${pt.is_ac ? 'AC' : 'DC'} Bus ${pt.bus}: ` +
              `P=${fmtP(pt.p_mw, 3)}, Q=${fmtQ(pt.q_mvar, 3)}, V=${fmt(pt.v_pu, 4)} pu`
            );
            if (toNum(er.internal_dcdc_pin_mw) !== null || toNum(er.internal_dcdc_pout_mw) !== null) {
              detail.push(`内部DC/DC: Pin=${fmtP(er.internal_dcdc_pin_mw, 3)}, Pout=${fmtP(er.internal_dcdc_pout_mw, 3)}`);
            }
            if (toNum(er.vsc_loss_mw) !== null || toNum(er.internal_dcdc_loss_mw) !== null) {
              detail.push(`端口VSC损耗=${fmtP(er.vsc_loss_mw, 3)}, 内部DC/DC损耗=${fmtP(er.internal_dcdc_loss_mw, 3)}`);
            }
            detail.push(`Loss=${fmtP(er.loss_mw, 3)}`);
            note.push('能量路由器结果由展开后的端口VSC与内部DC/DC潮流按索引回填');
          } else {
            detail = [`类型=${p.router_type || '-'}`, `端口=${p.num_ports || 0}`, `额定=${fmtP(p.p_rated_mw, 3)}`];
            note.push('本次潮流无该能量路由器端口分项返回');
          }
          break;
        }
        case 'vpp': {
          detail = [`P输出=${fmtP(p.p_output_mw, 3)}`, `Q输出=${fmtQ(p.q_output_mvar, 3)}`, `上调=${fmtP(p.p_regulation_up_mw, 3)}`, `下调=${fmtP(p.p_regulation_down_mw, 3)}`];
          solved = data.converged;
          note.push('本次潮流计算采用的虚拟电厂聚合注入');
          break;
        }
        case 'microgrid': {
          detail = [`模式=${p.operating_mode || '-'}`, `交换P=${fmtP(p.p_exchange_mw, 3)}`, `容量=${fmtP(p.capacity_mw, 3)}`, `Vset=${fmt(p.v_set_pu, 4)} pu`];
          solved = data.converged;
          note.push('本次潮流计算采用的微电网交换功率');
          break;
        }
        default:
          detail = Object.entries(p).slice(0, 6).map(([k, v]) => `${k}=${v}`);
          note.push('本次潮流结果中的元件参数摘要');
      }

      return { solved, detail, note };
    };

    const globalBlocked = data.converter_coordination && data.converter_coordination.enabled && data.converter_coordination.feasible === false;
    const statusFor = (comp, desc) => {
      const p = comp.params || {};
      if (p.in_service === false) return '停运';
      if (comp.type === 'switch_comp' || comp.type === 'circuit_breaker') return p.closed !== false ? '合闸' : '分闸';
      if (data.converged && desc.solved) return '已求解';
      if (desc.solved) return '有返回值/未收敛';
      if (data.converged) return '已参与计算';
      return globalBlocked ? '校核阻断' : '未收敛';
    };

    let html = '<table><thead><tr><th>ID</th><th>类型</th><th>名称</th><th>状态</th><th>连接</th><th>潮流/电压/功率</th><th>备注</th></tr></thead><tbody>';
    components.forEach(comp => {
      const desc = describe(comp);
      const status = statusFor(comp, desc);
      html += `<tr data-comp-id="${comp.id}">`;
      html += `<td>${comp.id}</td><td>${escapeHtml(componentTypeLabel(comp.type))}</td><td>${escapeHtml(compName(comp))}</td>`;
      html += `<td>${escapeHtml(status)}</td><td>${escapeHtml(connectionText(comp))}</td>`;
      html += `<td>${lineHtml(desc.detail)}</td><td>${lineHtml(desc.note)}</td></tr>`;
    });
    html += '</tbody></table>';
    div.innerHTML = html;
  }

  function showThreePhasePowerFlowResults(data, busMap) {
    const section = document.getElementById('pfThreePhaseSection');
    const div = document.getElementById('pfThreePhaseResults');
    if (!section || !div) return;
    const rows = Array.isArray(data.tp_bus_results) ? data.tp_bus_results : [];
    if (!rows.length) {
      section.style.display = 'none';
      div.innerHTML = '';
      return;
    }
    section.style.display = '';
    const fmt = v => Number.isFinite(Number(v)) ? Number(v).toFixed(6) : '-';
    const afmt = v => Number.isFinite(Number(v)) ? Number(v).toFixed(3) : '-';
    const comp = data.opendss_reference?.comparison || null;
    let html = '';
    if (data.opendss_reference?.ran && comp) {
      const ok = !!comp.within_gui_tolerance;
      html += `<p class="empty-hint" style="color:${ok ? '#15803d' : '#b45309'}">OpenDSS对比：${ok ? '通过' : '存在偏差'}；点数 ${comp.count || 0}，max|ΔVm|=${Number(comp.max_vm_error_pu || 0).toExponential(3)} pu，max|Δθ|=${Number(comp.max_angle_error_deg || 0).toFixed(4)}°。</p>`;
    } else if (data.opendss_reference && data.opendss_reference.error) {
      html += `<p class="empty-hint" style="color:#b45309">OpenDSS参考未运行：${escapeHtml(data.opendss_reference.error)}</p>`;
    } else {
      html += '<p class="empty-hint">当前会话没有可解析的 OpenDSS master 路径，仅显示本模块三相结果。</p>';
    }
    html += '<table><thead><tr><th>Bus</th><th>名称</th><th>相</th><th>Va(pu)</th><th>∠A(°)</th><th>Vb(pu)</th><th>∠B(°)</th><th>Vc(pu)</th><th>∠C(°)</th></tr></thead><tbody>';
    rows.forEach(r => {
      const busId = Number(r.bus_id ?? r.bus);
      const compId = Number.isFinite(busId) ? busMap.ac?.[busId] : undefined;
      const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
      html += `<tr${attr}><td>${Number.isFinite(busId) ? busId : ''}</td><td>${escapeHtml(r.name || '')}</td><td>${escapeHtml(r.phases || '')}</td><td>${fmt(r.vm_a_pu)}</td><td>${afmt(r.va_a_deg)}</td><td>${fmt(r.vm_b_pu)}</td><td>${afmt(r.va_b_deg)}</td><td>${fmt(r.vm_c_pu)}</td><td>${afmt(r.va_c_deg)}</td></tr>`;
    });
    html += '</tbody></table>';
    div.innerHTML = html;
  }

  function showPowerFlowBalanceDiagnostics(data, busMap) {
	    const section = document.getElementById('pfBalanceSection');
	    const div = document.getElementById('pfBalanceResults');
	    if (!section || !div) return;

	    if (data.three_phase || data.method === 'three_phase' || Array.isArray(data.tp_bus_results)) {
	      section.style.display = '';
	      div.innerHTML = '<p class="empty-hint">三相不对称潮流使用 abc 域节点方程；这里不显示单相等值节点功率平衡诊断，请查看“三相节点电压 / OpenDSS对比”。</p>';
	      return;
	    }

    const sys = (typeof Canvas !== 'undefined' && Canvas.buildSystemJson)
      ? Canvas.buildSystemJson() : null;
    if (!sys) {
      section.style.display = 'none';
      div.innerHTML = '';
      return;
    }

    const num = (v, d = 0) => {
      const x = Number(v);
      return Number.isFinite(x) ? x : d;
    };
    const isOn = (x) => !(x && (x.in_service === false || x.in_service === 'false'));
    if (!data.converged) {
      const coord = data.converter_coordination;
      const blocked = coord && coord.enabled && coord.feasible === false;
      const blockingCount = coord ? (coord.blocking_count ?? ((coord.fatal_count || 0) + (coord.error_count || 0))) : 0;
      section.style.display = '';
      div.innerHTML = `<p class="empty-hint" style="color:#b91c1c">潮流未收敛${blocked ? `，协调校核阻断 ${blockingCount || 0} 项` : ''}，节点功率平衡诊断不作为正常潮流结果显示。</p>`;
      return;
    }

    const backendDiag = data.power_balance_diagnostics;
    if (backendDiag && Array.isArray(backendDiag.ordinary) && Array.isArray(backendDiag.sources)) {
      const tolKw = num(backendDiag.tolerance_kw, 1.0);
      const rowKw = r => num(r.residual_kw ?? r.source_kw ?? r.kw ?? (num(r.mw ?? r.residual_mw ?? r.source_mw, 0) * 1000), 0);
      const rowMw = r => num(r.mw ?? r.residual_mw ?? r.source_mw, rowKw(r) / 1000);
      const rowKind = r => String(r.kind || r.domain || '').toUpperCase() === 'DC' ? 'DC' : 'AC';
      const detailsHtml = r => {
        const details = Array.isArray(r.details) ? r.details : [];
        return details
          .map(x => `${escapeHtml(x.label || '')}: ${pFmt(num(x.mw ?? x.value, 0), 3)} ${pUnit()}`)
          .join('<br>');
      };
      const makeRow = (r, source = false) => {
        const kind = rowKind(r);
        const id = Number(r.id ?? r.bus);
        const compId = kind === 'AC' ? busMap.ac?.[id] : busMap.dc?.[id];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        const type = source && (r.isImplicit || r.is_implicit)
          ? `${r.type || r.bus_type || ''} / 隐式DC平衡`
          : (r.type || r.bus_type || '');
        return `<tr${attr}><td>${kind}</td><td>${Number.isFinite(id) ? id : ''}</td><td>${escapeHtml(type)}</td><td>${pFmt(rowMw(r), 3)}</td><td>${escapeHtml(r.name || '')}</td><td>${detailsHtml(r)}</td></tr>`;
      };
      const bad = backendDiag.ordinary
        .filter(r => Math.abs(rowKw(r)) > tolKw)
        .sort((a, b) => Math.abs(rowKw(b)) - Math.abs(rowKw(a)));
      const sources = backendDiag.sources
        .filter(r => Math.abs(rowKw(r)) > tolKw)
        .sort((a, b) => Math.abs(rowKw(b)) - Math.abs(rowKw(a)));
      section.style.display = '';
      const status = bad.length
        ? `<p class="empty-hint" style="color:#b45309">发现 ${bad.length} 个非平衡节点超过 ${tolKw} kW，请检查连接或设备功率。</p>`
        : `<p class="empty-hint">普通节点有功平衡通过：未发现超过 ${tolKw} kW 的非平衡节点。</p>`;
      const sourceHint = sources.length
        ? `<p class="empty-hint">Slack/平衡源承担的功率本来可以非零，它不是普通节点KCL残差；“隐式DC平衡”表示该DC岛没有DC_V节点，求解器自动选该母线作为参考。</p>`
        : '';
      const imbalanceRows = bad.map(r => makeRow(r, false)).join('');
      const sourceRows = sources.map(r => makeRow(r, true)).join('');
      const imbalanceTable = imbalanceRows
        ? `<table><thead><tr><th>域</th><th>Bus</th><th>类型</th><th>普通节点KCL残差(${pUnit()})</th><th>名称</th><th>主要构成</th></tr></thead><tbody>${imbalanceRows}</tbody></table>`
        : '';
      const sourceTable = sourceRows
        ? `<table><thead><tr><th>域</th><th>Bus</th><th>类型</th><th>平衡源承担功率(${pUnit()})</th><th>名称</th><th>主要构成</th></tr></thead><tbody>${sourceRows}</tbody></table>`
        : '';
	      div.innerHTML = status + sourceHint + imbalanceTable + sourceTable;
	      return;
	    }

    const ac = new Map();
    const dc = new Map();
    const add = (map, key, value, label) => {
      if (key === undefined || key === null || key === 0) return;
      const id = Number(key);
      const row = map.get(id) || { mw: 0, items: [] };
      const mw = num(value, 0);
      row.mw += mw;
      if (Math.abs(mw) > 1e-9 && label) row.items.push({ mw, label });
      map.set(id, row);
    };
    const busName = (kind, id) => {
      const arr = kind === 'AC' ? (sys.ac?.buses || []) : (sys.dc?.buses || []);
      const b = arr.find(x => Number(x.index) === Number(id));
      return b ? (b.name || '') : '';
    };
    const busType = (kind, id) => {
      const arr = kind === 'AC' ? (sys.ac?.buses || []) : (sys.dc?.buses || []);
      const b = arr.find(x => Number(x.index) === Number(id));
      return b ? (b.bus_type || '') : '';
    };
    const rowMetric = (row, label) => {
      const metric = Array.isArray(row?.metrics)
        ? row.metrics.find(m => m.label === label)
        : null;
      if (!metric || String(metric.quantity || '').toLowerCase() === 'text') return null;
      const value = Number(metric.value);
      return Number.isFinite(value) ? value : null;
    };
    const solvedBusNet = (canvasType) => {
      const out = new Map();
      (data.component_results || []).forEach(row => {
        if (row?.canvas_type !== canvasType) return;
        const p = rowMetric(row, 'P净注入');
        const bus = Number(row.index);
        if (p !== null && Number.isFinite(bus)) out.set(bus, p);
      });
      return out;
    };
    const solvedAcNet = solvedBusNet('ac_bus');
    const solvedDcNet = solvedBusNet('dc_bus');

    (data.geo_ac_branches || []).forEach(br => {
      add(ac, br.from, br.pf_mw, `AC支路 ${br.from}->${br.to} Pf`);
      add(ac, br.to, br.pt_mw, `AC支路 ${br.from}->${br.to} Pt`);
    });
    const isMatchedBranchFlow = row => {
      const source = String(row?.source || '');
      return source === 'matched_ac_branch' || source === 'matched_dc_branch';
    };
    (data.ac_switch_flows || []).forEach(sw => {
      if (isMatchedBranchFlow(sw)) return;
      const from = sw.from ?? sw.from_bus;
      const to = sw.to ?? sw.to_bus;
      add(ac, from, sw.pf_mw, `AC开关 ${from}->${to} Pf`);
      add(ac, to, sw.pt_mw, `AC开关 ${from}->${to} Pt`);
    });
    (data.ac_circuit_breaker_flows || []).forEach(cb => {
      if (isMatchedBranchFlow(cb)) return;
      const from = cb.from ?? cb.from_bus;
      const to = cb.to ?? cb.to_bus;
      add(ac, from, cb.pf_mw, `AC断路器 ${from}->${to} Pf`);
      add(ac, to, cb.pt_mw, `AC断路器 ${from}->${to} Pt`);
    });
    const dcBranches = firstNonEmptyArray(data.geo_dc_branches, data.dc_branch_flows);
    dcBranches.forEach(br => {
      const from = br.from ?? br.from_bus;
      const to = br.to ?? br.to_bus;
      add(dc, from, br.pf_mw, `DC支路 ${from}->${to} Pf`);
      add(dc, to, br.pt_mw, `DC支路 ${from}->${to} Pt`);
    });
    (data.dc_circuit_breaker_flows || []).forEach(cb => {
      if (isMatchedBranchFlow(cb)) return;
      const from = cb.from ?? cb.from_bus;
      const to = cb.to ?? cb.to_bus;
      add(dc, from, cb.pf_mw, `DC断路器 ${from}->${to} Pf`);
      add(dc, to, cb.pt_mw, `DC断路器 ${from}->${to} Pt`);
    });
    const vscRows = firstNonEmptyArray(data.geo_vsc, data.vsc_transfers);
    vscRows.forEach(v => {
      if (!solvedAcNet.size) add(ac, v.bus_ac, -num(v.p_ac_mw), `VSC ${v.index ?? ''} AC侧`);
      if (!solvedDcNet.size) add(dc, v.bus_dc, -num(v.p_dc_mw), `VSC ${v.index ?? ''} DC侧`);
    });
    const dcdcRows = firstNonEmptyArray(data.geo_dcdc, data.dcdc_transfers);
    dcdcRows.forEach(d => {
      if (solvedDcNet.size) return;
      add(dc, d.bus_in, num(d.p_in_mw), `DCDC ${d.index ?? ''} 输入侧`);
      add(dc, d.bus_out, -num(d.p_out_mw), `DCDC ${d.index ?? ''} 输出侧`);
    });
    (data.geo_trafo3w || []).forEach(tf => {
      add(ac, tf.hv_bus, tf.p_hv_mw, `三绕组变压器 ${tf.index ?? ''} HV`);
      add(ac, tf.mv_bus, tf.p_mv_mw, `三绕组变压器 ${tf.index ?? ''} MV`);
      add(ac, tf.lv_bus, tf.p_lv_mw, `三绕组变压器 ${tf.index ?? ''} LV`);
    });

    function complex(re, im = 0) { return { re, im }; }
    function cAdd(a, b) { return complex(a.re + b.re, a.im + b.im); }
    function cMul(a, b) { return complex(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re); }
    function cDiv(a, b) {
      const den = b.re * b.re + b.im * b.im;
      return den > 0 ? complex((a.re * b.re + a.im * b.im) / den, (a.im * b.re - a.re * b.im) / den) : complex(0, 0);
	  }

    function cConj(a) { return complex(a.re, -a.im); }
    function cNeg(a) { return complex(-a.re, -a.im); }
    const acVoltagePosByBus = new Map();
    (data.component_results || []).forEach(row => {
      if (row?.canvas_type !== 'ac_bus') return;
      const bus = Number(row.index);
      const pos = Number(row.position);
      if (Number.isFinite(bus) && Number.isInteger(pos) && pos >= 0) {
        acVoltagePosByBus.set(bus, pos);
      }
    });
    if (!acVoltagePosByBus.size) {
      (sys.ac?.buses || []).forEach((bus, pos) => {
        const id = Number(bus?.index);
        if (Number.isFinite(id)) acVoltagePosByBus.set(id, pos);
      });
    }
    function vphasor(busId) {
      const idx = acVoltagePosByBus.get(Number(busId));
      if (!Number.isInteger(idx) || idx < 0) return null;
      const vm = Number(data.vm?.[idx]);
      const va = Number(data.va?.[idx]);
      if (!Number.isFinite(vm) || !Number.isFinite(va)) return null;
      return complex(vm * Math.cos(va), vm * Math.sin(va));
    }
    const acBranchBackedTrafos = [];
    (sys.ac?.branches || []).forEach(br => {
      if (!isOn(br)) return;
      const tap = num(br.tap, 1);
      const shift = num(br.shift_deg, 0);
      if (Math.abs(tap - 1) <= 1e-8 && Math.abs(shift) <= 1e-8) return;
      acBranchBackedTrafos.push({
        from: Number(br.from_bus),
        to: Number(br.to_bus),
        tap,
        shift,
        rPu: num(br.r_pu),
        xPu: Math.abs(num(br.x_pu)),
      });
    });
    const isBranchBackedTrafo = (tf) => {
      const sourceIdx = Number(tf?.source_branch_idx);
      if (Number.isFinite(sourceIdx) && sourceIdx > 0) return true;
      const hv = Number(tf?.hv_bus);
      const lv = Number(tf?.lv_bus);
      const tap = Math.max(1e-6, 1 + (num(tf?.tap_pos) - num(tf?.tap_neutral)) * num(tf?.tap_step_percent) / 100);
      const shift = num(tf?.shift_deg, 0);
      const baseMva = num(sys.base_mva ?? sys.ac?.base_mva, 100);
      const snMva = num(tf?.sn_mva);
      const scale = snMva > 1e-9 ? baseMva / snMva : 0;
      const zMag = Math.max(0, num(tf?.vk_percent) / 100) * scale;
      const rPu = Math.max(0, num(tf?.vkr_percent) / 100) * scale;
      const xPu = Math.sqrt(Math.max(0, zMag * zMag - rPu * rPu));
      return acBranchBackedTrafos.some(br => {
        if (br.from !== hv || br.to !== lv) return false;
        if (Math.abs(br.tap - tap) > 1e-5 || Math.abs(br.shift - shift) > 1e-5) return false;
        if (snMva <= 1e-9 || baseMva <= 1e-9) return true;
        return Math.abs(br.rPu - rPu) <= 1e-5 && Math.abs(br.xPu - xPu) <= 1e-5;
      });
    };
    (sys.ac?.transformers_2w || []).forEach(tf => {
      if (!isOn(tf) || isBranchBackedTrafo(tf)) return;
      const vh = vphasor(tf.hv_bus);
      const vl = vphasor(tf.lv_bus);
      const baseMva = num(sys.base_mva ?? sys.ac?.base_mva, 100);
      const snMva = num(tf.sn_mva);
      if (!vh || !vl || baseMva <= 1e-9 || snMva <= 1e-9) return;
      const scale = baseMva / snMva;
      const zMag = Math.max(0, num(tf.vk_percent) / 100) * scale;
      let rPu = Math.max(0, num(tf.vkr_percent) / 100) * scale;
      let xPu = Math.sqrt(Math.max(0, zMag * zMag - rPu * rPu));
      if (rPu === 0 && xPu === 0) xPu = 1e-4;
      const rawTap = Math.max(1e-6, 1 + (num(tf.tap_pos) - num(tf.tap_neutral)) * num(tf.tap_step_percent) / 100);
      if (Number(tf.tap_side) === 1) {
        rPu *= rawTap * rawTap;
        xPu *= rawTap * rawTap;
      }
      const ys = cDiv(complex(1, 0), complex(rPu, xPu));
      const tapMag = Number(tf.tap_side) === 1 ? 1 / rawTap : rawTap;
      const shift = num(tf.shift_deg) * Math.PI / 180;
      const tap = complex(tapMag * Math.cos(shift), tapMag * Math.sin(shift));
      const tapAbs2 = tap.re * tap.re + tap.im * tap.im;
      if (tapAbs2 <= 0) return;
      const yff = complex(ys.re / tapAbs2, ys.im / tapAbs2);
      const yft = cNeg(cDiv(ys, cConj(tap)));
      const ytf = cNeg(cDiv(ys, tap));
      const ih = cAdd(cMul(yff, vh), cMul(yft, vl));
      const il = cAdd(cMul(ytf, vh), cMul(ys, vl));
      add(ac, tf.hv_bus, cMul(vh, cConj(ih)).re * baseMva, `二绕组变压器 ${tf.index ?? ''} HV`);
      add(ac, tf.lv_bus, cMul(vl, cConj(il)).re * baseMva, `二绕组变压器 ${tf.index ?? ''} LV`);
    });

    if (solvedAcNet.size) {
      solvedAcNet.forEach((p, bus) => add(ac, bus, -p, `求解净注入 AC Bus ${bus}`));
    } else {
      (sys.ac?.buses || []).forEach(x => {
        if (isOn(x) && Math.abs(num(x.pd_mw)) > 1e-9) {
          add(ac, x.index, num(x.pd_mw), `母线负荷 ${x.index}`);
        }
      });
      (sys.ac?.loads || []).forEach(x => { if (isOn(x)) add(ac, x.bus, num(x.p_mw) * num(x.scaling, 1), `负荷 ${x.index ?? ''}`); });
      (sys.ac?.charging_stations || []).forEach(x => { if (isOn(x)) add(ac, x.bus, num(x.p_total_kw) / 1000, `充电站 ${x.index ?? ''}`); });
      (sys.ac?.chargers || []).forEach(x => { if (isOn(x)) add(ac, x.bus, num(x.p_ch_kw) / 1000, `充电桩 ${x.index ?? ''}`); });
      const solvedGenByIndex = new Map();
      const solvedGenByBusQueue = new Map();
      (data.geo_gen || []).forEach(g => {
        const idx = Number(g.index);
        if (Number.isFinite(idx)) solvedGenByIndex.set(idx, g);
        const bus = Number(g.bus);
        if (Number.isFinite(bus)) {
          const arr = solvedGenByBusQueue.get(bus) || [];
          arr.push(g);
          solvedGenByBusQueue.set(bus, arr);
        }
      });
      (sys.ac?.generators || []).forEach(x => {
        if (!isOn(x)) return;
        const idx = Number(x.index);
        let g = Number.isFinite(idx) ? solvedGenByIndex.get(idx) : null;
        if (!g) {
          const q = solvedGenByBusQueue.get(Number(x.bus)) || [];
          g = q.shift() || null;
        }
        add(ac, x.bus, -num(g?.pg_mw ?? x.pg_mw ?? x.p_mw), `发电机 ${x.index ?? ''}`);
      });
      (sys.ac?.pv_systems || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.p_mw), `光伏 ${x.index ?? ''}`); });
      (sys.ac?.renewable_gens || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.p_mw), `新能源 ${x.index ?? ''}`); });
      (sys.ac?.static_generators || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.p_mw ?? x.p_set_mw), `静态电源 ${x.index ?? ''}`); });
      (sys.ac?.storage || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.p_mw), `储能 ${x.index ?? ''}`); });
      (sys.vpps || []).forEach(x => { if (isOn(x)) add(ac, x.pcc_bus ?? x.bus, -num(x.p_output_mw ?? x.p_mw), `虚拟电厂 ${x.index ?? ''}`); });
      (sys.microgrids || []).forEach(x => { if (isOn(x) && x.operating_mode !== 'Islanded') add(ac, x.pcc_bus, -num(x.p_exchange_mw), `微网 ${x.index ?? ''}`); });
    }
    if (solvedDcNet.size) {
      solvedDcNet.forEach((p, bus) => add(dc, bus, -p, `求解净注入 DC Bus ${bus}`));
    } else {
      (sys.dc?.loads || []).forEach(x => { if (isOn(x)) add(dc, x.bus, num(x.p_mw) * num(x.scaling, 1), `DC负荷 ${x.index ?? ''}`); });
      (sys.dc?.buses || []).forEach(x => {
        if (isOn(x) && Math.abs(num(x.pd_mw)) > 1e-9) {
          add(dc, x.index, num(x.pd_mw), `DC母线负荷 ${x.index}`);
        }
      });
      (sys.dc?.pv_arrays || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw ?? x.p_set_mw), `DC光伏 ${x.index ?? ''}`); });
      (sys.dc?.static_generators || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw ?? x.p_set_mw), `DC静态电源 ${x.index ?? ''}`); });
      (sys.dc?.dc_static_generators || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw ?? x.p_set_mw), `DC静态电源 ${x.index ?? ''}`); });
      (sys.dc?.dc_storage || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw), `DC储能 ${x.index ?? ''}`); });
      (sys.dc?.storage || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw), `DC储能 ${x.index ?? ''}`); });
    }

    function dcSlackSet() {
      const buses = (sys.dc?.buses || []).filter(isOn);
      const byId = new Map(buses.map(b => [Number(b.index), b]));
      const adj = new Map(buses.map(b => [Number(b.index), []]));
      (sys.dc?.branches || []).forEach(br => {
        if (!isOn(br)) return;
        const f = Number(br.from_bus), t = Number(br.to_bus);
        if (!adj.has(f) || !adj.has(t)) return;
        adj.get(f).push(t);
        adj.get(t).push(f);
      });
      firstNonEmptyArray(sys.dc?.dcdc_converters, sys.dcdc_converters).forEach(d => {
        if (!isOn(d)) return;
        const f = Number(d.bus_in), t = Number(d.bus_out);
        if (!adj.has(f) || !adj.has(t)) return;
        adj.get(f).push(t);
        adj.get(t).push(f);
      });
      const visited = new Set();
      const slacks = new Set();
      const implicit = new Set();
      for (const start of adj.keys()) {
        if (visited.has(start)) continue;
        const q = [start];
        const comp = [];
        visited.add(start);
        for (let head = 0; head < q.length; head++) {
          const u = q[head];
          comp.push(u);
          (adj.get(u) || []).forEach(v => {
            if (!visited.has(v)) {
              visited.add(v);
              q.push(v);
            }
          });
        }
        let chosen = comp.find(id => String(byId.get(id)?.bus_type || '').toUpperCase() === 'DC_V');
        if (chosen === undefined) {
          chosen = comp.find(id => String(byId.get(id)?.bus_type || '').toUpperCase() !== 'DC_ISOLATED');
          if (chosen === undefined) chosen = comp[0];
          implicit.add(chosen);
        }
        if (chosen !== undefined) slacks.add(chosen);
      }
      return { slacks, implicit };
    }

    const acSlack = new Set((sys.ac?.external_grids || []).filter(isOn).map(x => Number(x.bus)));
    (sys.ac?.buses || []).forEach(b => {
      if (String(b.bus_type || '').toUpperCase() === 'SLACK') acSlack.add(Number(b.index));
    });
    (sys.vsc_converters || []).forEach(v => {
      if (!isOn(v)) return;
      const mode = String(v.control_mode || '').toUpperCase();
      if (v.ac_grid_forming || mode === 'AC_GRID_FORMING' || mode === 'AC_GFM') {
        acSlack.add(Number(v.bus_ac));
      }
    });
    const dcSlack = dcSlackSet();
    const sourceAc = new Map();
    const sourceDc = new Map();
    const addSource = (map, key, value, label) => {
      if (key === undefined || key === null || key === 0) return;
      const id = Number(key);
      const mw = num(value, 0);
      const row = map.get(id) || { mw: 0, items: [] };
      row.mw += mw;
      if (Math.abs(mw) > 1e-9 && label) row.items.push({ mw, label });
      map.set(id, row);
    };
    (data.geo_gen || []).forEach(g => {
      const bus = Number(g.bus);
      const isSlackGen = g.is_slack || acSlack.has(bus);
      if (isSlackGen) addSource(sourceAc, bus, g.pg_mw, `Slack发电机 ${g.index ?? ''}`);
    });
    (data.component_results || []).forEach(row => {
      if (row?.canvas_type === 'external_grid') {
        const p = rowMetric(row, 'P平衡');
        const eg = (sys.ac?.external_grids || []).find(x => Number(x.index) === Number(row.index));
        if (p !== null) addSource(sourceAc, eg?.bus, p, `外部电网 ${row.index ?? ''}`);
      } else if (row?.canvas_type === 'dc_bus') {
        const p = rowMetric(row, 'DC平衡源');
        if (p !== null) addSource(sourceDc, row.index, p, `DC_V平衡源 ${row.index ?? ''}`);
      }
    });
    const vscByIndex = new Map((data.vsc_transfers || []).map(v => [Number(v.index), v]));
    (sys.vsc_converters || []).forEach(v => {
      if (!isOn(v)) return;
      const mode = String(v.control_mode || '').toUpperCase();
      if (!(v.ac_grid_forming || mode === 'AC_GRID_FORMING' || mode === 'AC_GFM')) return;
      const solved = vscByIndex.get(Number(v.index));
      addSource(sourceAc, solved?.bus_ac ?? v.bus_ac, solved?.p_ac_mw ?? v.p_set_mw, `AC构网VSC ${v.index ?? ''}`);
    });
    const tolKw = 1.0;
    const bad = [];
    const sources = [];
    const collect = (kind, map, buses, slackSet, implicitSet = new Set(), sourceMap = new Map()) => {
      (buses || []).forEach(b => {
        if (!isOn(b)) return;
        const id = Number(b.index);
        const rec = map.get(id) || { mw: 0, items: [] };
        const residualKw = rec.mw * 1000;
        const isSlack = slackSet.has(id);
        const isImplicit = implicitSet.has(id);
        if (!isSlack && Math.abs(residualKw) > tolKw) {
          bad.push({
            kind, id, kw: residualKw, isSlack, isImplicit,
            type: b.bus_type || '',
            name: busName(kind, id),
            details: rec.items.sort((a, b) => Math.abs(b.mw) - Math.abs(a.mw)).slice(0, 4),
          });
        }
        if (!isSlack) return;
        const explicitSource = sourceMap.get(id);
        const source = explicitSource || rec;
        const sourceKw = source.mw * 1000;
        if (Math.abs(sourceKw) <= tolKw) return;
        sources.push({
          kind, id, kw: sourceKw, isSlack, isImplicit,
          type: b.bus_type || '',
          name: busName(kind, id),
          details: source.items.sort((a, b) => Math.abs(b.mw) - Math.abs(a.mw)).slice(0, 4),
        });
      });
    };
    collect('AC', ac, sys.ac?.buses || [], acSlack, new Set(), sourceAc);
    collect('DC', dc, sys.dc?.buses || [], dcSlack.slacks, dcSlack.implicit, sourceDc);

    section.style.display = '';
    const status = bad.length
      ? `<p class="empty-hint" style="color:#b45309">发现 ${bad.length} 个非平衡节点超过 ${tolKw} kW，请检查连接或设备功率。</p>`
      : `<p class="empty-hint">普通节点有功平衡通过：未发现超过 ${tolKw} kW 的非平衡节点。</p>`;
    const sourceHint = sources.length
      ? `<p class="empty-hint">Slack/平衡源承担的功率本来可以非零，它不是普通节点KCL残差；“隐式DC平衡”表示该DC岛没有DC_V节点，求解器自动选该母线作为参考。</p>`
      : '';
    const makeRow = (r) => {
      const compId = r.kind === 'AC' ? busMap.ac?.[r.id] : busMap.dc?.[r.id];
      const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
      const detail = r.details.map(x => `${x.label}: ${pFmt(x.mw, 3)} ${pUnit()}`).join('<br>');
      return `<tr${attr}><td>${r.kind}</td><td>${r.id}</td><td>${escapeHtml(r.type)}</td><td>${pFmt(r.kw / 1000, 3)}</td><td>${escapeHtml(r.name)}</td><td>${detail}</td></tr>`;
    };
    const imbalanceRows = bad
      .sort((a, b) => Math.abs(b.kw) - Math.abs(a.kw))
      .map(makeRow)
      .join('');
    const sourceRows = sources
      .sort((a, b) => Math.abs(b.kw) - Math.abs(a.kw))
      .map(r => makeRow({ ...r, type: r.isImplicit ? `${r.type || ''} / 隐式DC平衡` : r.type }))
      .join('');
    const imbalanceTable = imbalanceRows
      ? `<table><thead><tr><th>域</th><th>Bus</th><th>类型</th><th>普通节点KCL残差(${pUnit()})</th><th>名称</th><th>主要构成</th></tr></thead><tbody>${imbalanceRows}</tbody></table>`
      : '';
    const sourceTable = sourceRows
      ? `<table><thead><tr><th>域</th><th>Bus</th><th>类型</th><th>平衡源承担功率(${pUnit()})</th><th>名称</th><th>主要构成</th></tr></thead><tbody>${sourceRows}</tbody></table>`
      : '';
    div.innerHTML = status + sourceHint + imbalanceTable + sourceTable;
  }

  function showPowerFlowResultsTables(data) {
    data = normalizePowerFlowResult(data);
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('powerFlow');

    // Summary
    const summary = document.getElementById('resultsSummary');
    const opt = data.options_effective || data.options_requested || {};
    const optNumber = (value, fallback = 0) => {
      const n = Number(value);
      return Number.isFinite(n) ? n : fallback;
    };
    const boolLabel = (value) => value ? '开' : '关';
    const lossLabel = opt.loss_model === 'current_based' ? '电流相关' : '线性';
    const globLabel = opt.globalization === 'trust_region'
      ? '信赖域'
      : opt.globalization === 'pseudo_transient'
        ? '伪暂态'
        : '线搜索';
    const zipText = (arr) => Array.isArray(arr)
      ? arr.map(v => optNumber(v, 0).toFixed(2)).join('/')
      : '';
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">方法</span>
        <span class="result-value">${data.method_actual || data.method || ''}</span></div>
      <div class="result-item"><span class="result-label">收敛</span>
        <span class="result-value ${data.converged ? 'result-converged' : 'result-failed'}">
          ${data.converged ? '✓ 是' : '✗ 否'}</span></div>
      <div class="result-item"><span class="result-label">迭代次数</span>
        <span class="result-value">${data.iterations || 0}</span></div>
      <div class="result-item"><span class="result-label">最大残差</span>
        <span class="result-value">${Number(data.residual || 0).toExponential(4)}</span></div>
      <div class="result-item"><span class="result-label">求解参数</span>
        <span class="result-value">tol=${optNumber(opt.tol, 0).toExponential(1)}, max=${opt.max_iter ?? '-'}</span></div>
      <div class="result-item"><span class="result-label">损耗/全局化</span>
        <span class="result-value">${lossLabel} / ${globLabel}</span></div>
      <div class="result-item"><span class="result-label">模型开关</span>
        <span class="result-value">PV/PQ=${boolLabel(opt.enable_pv_pq_conversion !== false)}, VSC=${boolLabel(opt.enable_converter_mode_switching !== false)}, 校核=${boolLabel(opt.enable_converter_coordination_check === true)}</span></div>
      <div class="result-item"><span class="result-label">ZIP(P/Q)</span>
        <span class="result-value">${zipText(opt.zip_pw) || '-'} / ${zipText(opt.zip_qw) || '-'}</span></div>
    `;

    let busMap = {};
    try {
      busMap = Canvas.getCompBusMap ? Canvas.getCompBusMap() : {};
    } catch (err) {
      console.warn('Failed to build canvas bus map for power-flow results:', err);
      busMap = {};
    }
    try {
      showPowerFlowBalanceDiagnostics(data, busMap);
    } catch (err) {
      console.warn('Failed to render power-flow balance diagnostics:', err);
    }
    try {
      showThreePhasePowerFlowResults(data, busMap);
    } catch (err) {
      console.warn('Failed to render three-phase power-flow results:', err);
    }
    const resultRowAttr = (row, fallbackCompId) => canvasRowAttr(row, busMap, { fallbackCompId });

    const coordSec = document.getElementById('pfCoordSection');
    const coordDiv = document.getElementById('pfCoordResults');
    const coord = data.converter_coordination;
    if (coordSec && coordDiv && coord && coord.enabled) {
      coordSec.style.display = '';
      const issues = Array.isArray(coord.issues) ? coord.issues : [];
      const islands = Array.isArray(coord.dc_islands) ? coord.dc_islands : [];
      const status = coord.feasible ? '通过' : '不可行';
      const blockingCount = coord.blocking_count ?? ((coord.fatal_count || 0) + (coord.error_count || 0));
      let html = `<div style="margin-bottom:8px;color:${coord.feasible ? '#15803d' : '#b91c1c'};"><strong>状态：</strong>${status}；阻断项: ${blockingCount || 0}；Fatal: ${coord.fatal_count || 0}；Error: ${coord.error_count || 0}；Warning: ${coord.warning_count || 0}</div>`;
      if (issues.length) {
        html += '<table><thead><tr><th>等级</th><th>规则</th><th>对象</th><th>岛</th><th>说明</th></tr></thead><tbody>';
        issues.forEach(issue => {
          const sev = issue.severity || '';
          const color = sev === 'fatal' || sev === 'error' ? '#b91c1c' : (sev === 'warning' ? '#b45309' : '#475569');
          const comp = `${issue.component_type || ''}${issue.component_index != null && issue.component_index >= 0 ? '#' + issue.component_index : ''}`;
          html += `<tr><td style="color:${color};font-weight:600">${escapeHtml(sev)}</td><td>${escapeHtml(issue.rule_id || '')}</td><td>${escapeHtml(comp)}</td><td>${issue.island_index ?? '-'}</td><td>${escapeHtml(issue.message || '')}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      if (islands.length) {
        html += '<table style="margin-top:8px"><thead><tr><th>DC岛</th><th>DC母线</th><th>声明DC_V母线</th><th>电压源</th><th>硬Vdc源</th><th>下垂源</th><th>固定功率设备</th><th>固定净注入(MW)</th><th>上调(MW)</th><th>下调(MW)</th></tr></thead><tbody>';
        islands.forEach(isle => {
          const sources = Array.isArray(isle.voltage_sources) ? isle.voltage_sources : [];
          const sourceText = sources.map(src => {
            const mode = src.droop ? 'droop' : 'rigid';
            const vset = src.has_v_set ? `@${Number(src.v_set_pu || 0).toFixed(4)}pu` : '';
            return `${src.component_type || ''}#${src.component_index ?? '-'}:${mode}${vset}`;
          }).join('; ');
          html += `<tr><td>${isle.island_index ?? '-'}</td><td>${escapeHtml((isle.dc_buses || []).join(','))}</td><td>${escapeHtml((isle.declared_v_buses || []).join(','))}</td><td>${escapeHtml(sourceText)}</td><td>${isle.hard_vdc_sources || 0}</td><td>${isle.droop_sources || 0}</td><td>${isle.fixed_power_devices || 0}</td><td>${Number(isle.fixed_power_mw || 0).toFixed(4)}</td><td>${Number(isle.flexible_up_mw || 0).toFixed(4)}</td><td>${Number(isle.flexible_down_mw || 0).toFixed(4)}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      coordDiv.innerHTML = html;
    } else if (coordSec && coordDiv) {
      coordSec.style.display = 'none';
      coordDiv.innerHTML = '';
    }

    try {
      renderAllPowerFlowComponentStatus(data, busMap);
    } catch (err) {
      console.warn('Failed to render all-component power-flow results:', err);
      const section = document.getElementById('pfAllComponentsSection');
      const div = document.getElementById('pfAllComponentsResults');
      if (section) section.style.display = '';
      if (div) div.innerHTML = '<p class="empty-hint">全部元件计算结果渲染失败；其余潮流结果已继续显示。</p>';
    }

    const pfBusIdByPosition = (domain, i) => {
      const type = domain === 'dc' ? 'dc_bus' : 'ac_bus';
      const row = (data.component_results || []).find(r =>
        r.canvas_type === type && Number(r.position) === Number(i));
      if (row && row.index !== undefined && row.index !== null) return Number(row.index);
      const geoType = domain === 'dc' ? 'DC' : 'AC';
      const geo = (data.geo_buses || []).filter(b => b.type === geoType);
      if (geo[i] && geo[i].id !== undefined && geo[i].id !== null) return Number(geo[i].id);
      return null;
    };

    // AC Bus voltage table
    const busDiv = document.getElementById('pfBusResults');
    if (data.vm && data.vm.length > 0) {
      let html = '<table><thead><tr><th>Bus</th><th>Vm(pu)</th><th>Va(°)</th></tr></thead><tbody>';
      data.vm.forEach((vm, i) => {
        const busId = pfBusIdByPosition('ac', i);
        const busLabel = busId ?? `pos ${i}`;
        const va = data.va ? (data.va[i] * 180 / Math.PI).toFixed(4) : '0';
        const color = vm < 0.95 ? 'color:#e06c75' : vm > 1.05 ? 'color:#d19a66' : '';
        const resultRow = (data.component_results || []).find(r =>
          r.canvas_type === 'ac_bus' && (Number(r.position) === Number(i) || Number(r.index) === Number(busId))) ||
          { canvas_type: 'ac_bus', index: busId, position: i };
        const attr = resultRowAttr(resultRow, busId != null && busMap.ac ? busMap.ac[busId] : undefined);
        html += `<tr${attr}><td>${busLabel}</td><td style="${color}">${vm.toFixed(6)}</td><td>${va}</td></tr>`;
      });
      html += '</tbody></table>';
      busDiv.innerHTML = html;
    } else {
      busDiv.innerHTML = '<p class="empty-hint">无AC节点电压数据</p>';
    }

    // DC Bus voltage table
    const dcBusSec = document.getElementById('pfDcBusSection');
    const dcBusDiv = document.getElementById('pfDcBusResults');
    if (data.vdc && data.vdc.length > 0) {
      dcBusSec.style.display = '';
      const dcBusRows = (data.component_results || []).filter(r => r.canvas_type === 'dc_bus');
      const dcMetric = (busId, label) => {
        const row = dcBusRows.find(r => Number(r.index) === Number(busId));
        const metric = row && Array.isArray(row.metrics)
          ? row.metrics.find(m => m.label === label)
          : null;
        const n = metric ? Number(metric.value) : NaN;
        return Number.isFinite(n) ? n : null;
      };
      let html = `<table><thead><tr><th>DC Bus</th><th>Vdc(pu)</th><th>P净注入(${pUnit()})</th></tr></thead><tbody>`;
      data.vdc.forEach((vdc, i) => {
        const busId = pfBusIdByPosition('dc', i);
        const busLabel = busId ?? `pos ${i}`;
        const color = vdc < 0.95 ? 'color:#e06c75' : vdc > 1.05 ? 'color:#d19a66' : '';
        const resultRow = dcBusRows.find(r => Number(r.position) === Number(i) || Number(r.index) === Number(busId)) ||
          { canvas_type: 'dc_bus', index: busId, position: i };
        const attr = resultRowAttr(resultRow, busId != null && busMap.dc ? busMap.dc[busId] : undefined);
        const pNet = busId != null ? dcMetric(busId, 'P净注入') : null;
        html += `<tr${attr}><td>${busLabel}</td><td style="${color}">${vdc.toFixed(6)}</td><td>${pNet === null ? '-' : pFmt(pNet, 4)}</td></tr>`;
      });
      html += '</tbody></table>';
      dcBusDiv.innerHTML = html;
    } else {
      dcBusSec.style.display = 'none';
      dcBusDiv.innerHTML = '';
    }

    // Generator output table (from geo_gen solved data)
    const genSec = document.getElementById('pfGenSection');
    const genDiv = document.getElementById('pfGenResults');
    if (data.geo_gen && data.geo_gen.length > 0) {
      genSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>Bus</th><th>Name</th><th>Pg(${pUnit()})</th><th>Qg(${qUnit()})</th><th>Vg(pu)</th><th>Slack</th></tr></thead><tbody>`;
      data.geo_gen.forEach((g, i) => {
        const attr = resultRowAttr({ ...g, canvas_type: g.canvas_type || 'generator', canvas_index: g.canvas_index ?? g.index, position: g.position ?? i },
          busMap.gen ? (busMap.gen[g.index] ?? busMap.gen[i]) : undefined);
        html += `<tr${attr}><td>${g.index ?? i}</td><td>${g.bus}</td><td>${g.name || ''}</td>`;
        html += `<td>${pFmt(g.pg_mw, 4)}</td><td>${pFmt(g.qg_mvar, 4)}</td>`;
        html += `<td>${(g.vg_pu || 1).toFixed(4)}</td><td>${g.is_slack ? '✓' : ''}</td></tr>`;
      });
      html += '</tbody></table>';
      genDiv.innerHTML = html;
    } else {
      genSec.style.display = 'none';
      genDiv.innerHTML = '';
    }

    // AC Branch flow table (prefer geo_ac_branches with full Pf/Pt/Q/Loss data)
    const brDiv = document.getElementById('pfBranchResults');
    if (data.geo_ac_branches && data.geo_ac_branches.length > 0) {
      let html = `<table><thead><tr><th>#</th><th>From</th><th>To</th><th>Pf(${pUnit()})</th><th>Pt(${pUnit()})</th><th>Qf(${qUnit()})</th><th>Qt(${qUnit()})</th><th>Loss(${pUnit()})</th><th>Loading%</th></tr></thead><tbody>`;
      data.geo_ac_branches.forEach((br, i) => {
        const fallbackCompId = (busMap.branch && busMap.branch[br.index] !== undefined)
          ? busMap.branch[br.index]
          : (busMap.ac ? busMap.ac[br.from] : undefined);
        const attr = resultRowAttr({ ...br, canvas_type: br.canvas_type || 'ac_branch', canvas_index: br.canvas_index ?? br.index, position: br.position ?? i, from_bus: br.from, to_bus: br.to }, fallbackCompId);
        const loss = (br.loss_mw != null) ? br.loss_mw : ((br.pf_mw || 0) + (br.pt_mw || 0));
        const ldg = br.loading_pct != null ? br.loading_pct.toFixed(1) + '%' : '-';
        const ldgStyle = (br.loading_pct || 0) > 100 ? ' style="color:#e06c75;font-weight:bold"' : '';
        html += `<tr${attr}><td>${i}</td><td>${br.from}</td><td>${br.to}</td>`;
        html += `<td>${pFmt(br.pf_mw || 0, 4)}</td><td>${pFmt(br.pt_mw || 0, 4)}</td>`;
        html += `<td>${pFmt(br.qf_mvar || 0, 4)}</td><td>${pFmt(br.qt_mvar || 0, 4)}</td>`;
        html += `<td>${pFmt(loss, 4)}</td><td${ldgStyle}>${ldg}</td></tr>`;
      });
      html += '</tbody></table>';
      brDiv.innerHTML = html;
    } else if (data.branch_abs && data.branch_abs.length > 0) {
      // Fallback: old branch_abs (|P| only)
      let html = `<table><thead><tr><th>Branch</th><th>|P|(${pUnit()})</th></tr></thead><tbody>`;
      data.branch_abs.forEach((p, i) => {
        const attr = resultRowAttr({ canvas_type: 'ac_branch', index: i, position: i }, busMap.branch ? busMap.branch[i] : undefined);
        html += `<tr${attr}><td>${i}</td><td>${pFmt(p, 4)}</td></tr>`;
      });
      html += '</tbody></table>';
      brDiv.innerHTML = html;
    } else {
      brDiv.innerHTML = '<p class="empty-hint">无AC支路功率数据</p>';
    }

    // DC Branch flow table (prefer geo_dc_branches, fallback to dc_branch_flows)
    const dcBrSec = document.getElementById('pfDcBranchSection');
    const dcBrDiv = document.getElementById('pfDcBranchResults');
    const dcBrData = firstNonEmptyArray(data.geo_dc_branches, data.dc_branch_flows);
    if (dcBrData.length > 0) {
      dcBrSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>From</th><th>To</th><th>Pf(${pUnit()})</th><th>Pt(${pUnit()})</th><th>Loss(${pUnit()})</th></tr></thead><tbody>`;
      dcBrData.forEach((br, i) => {
        const fallbackCompId = busMap.dcBranch
          ? (busMap.dcBranch[br.index] ?? busMap.dcBranch[i])
          : undefined;
        const attr = resultRowAttr({ ...br, canvas_type: br.canvas_type || 'dc_branch', canvas_index: br.canvas_index ?? br.index, position: br.position ?? i }, fallbackCompId);
        const from = br.from ?? br.from_bus ?? '-';
        const to = br.to ?? br.to_bus ?? '-';
        const pf = br.pf_mw || 0;
        const pt = br.pt_mw || 0;
        const loss = br.loss_mw != null ? br.loss_mw : (pf + pt);
        html += `<tr${attr}><td>${i}</td><td>${from}</td><td>${to}</td><td>${pFmt(pf, 4)}</td><td>${pFmt(pt, 4)}</td><td>${pFmt(loss, 4)}</td></tr>`;
      });
      html += '</tbody></table>';
      dcBrDiv.innerHTML = html;
    } else {
      dcBrSec.style.display = 'none';
      dcBrDiv.innerHTML = '';
    }

    // 3W Transformer flow table
    const t3wSec = document.getElementById('pfTrafo3wSection');
    const t3wDiv = document.getElementById('pfTrafo3wResults');
    if (data.geo_trafo3w && data.geo_trafo3w.length > 0) {
      t3wSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>HV</th><th>MV</th><th>LV</th><th>P_hv(${pUnit()})</th><th>P_mv(${pUnit()})</th><th>P_lv(${pUnit()})</th><th>Loss(${pUnit()})</th><th>Loading%</th></tr></thead><tbody>`;
      data.geo_trafo3w.forEach((tf, i) => {
        const attr = resultRowAttr({ ...tf, canvas_type: tf.canvas_type || 'transformer_3w', canvas_index: tf.canvas_index ?? tf.index, position: tf.position ?? i });
        const ldg = tf.loading_pct != null ? tf.loading_pct.toFixed(1) + '%' : '-';
        const ldgStyle = (tf.loading_pct || 0) > 100 ? ' style="color:#e06c75;font-weight:bold"' : '';
        html += `<tr${attr}><td>${tf.index ?? i}</td><td>${tf.hv_bus}</td><td>${tf.mv_bus}</td><td>${tf.lv_bus}</td>`;
        html += `<td>${pFmt(tf.p_hv_mw || 0, 3)}</td><td>${pFmt(tf.p_mv_mw || 0, 3)}</td><td>${pFmt(tf.p_lv_mw || 0, 3)}</td>`;
        html += `<td>${pFmt(tf.loss_mw || 0, 3)}</td><td${ldgStyle}>${ldg}</td></tr>`;
      });
      html += '</tbody></table>';
      t3wDiv.innerHTML = html;
    } else {
      t3wSec.style.display = 'none';
      t3wDiv.innerHTML = '';
    }

    // DCDC converter transfers table
    const dcdcSec = document.getElementById('pfDcdcSection');
    const dcdcDiv = document.getElementById('pfDcdcResults');
    const dcdcRows = firstNonEmptyArray(data.dcdc_transfers, data.geo_dcdc);
    if (dcdcRows.length > 0) {
      dcdcSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>Bus In</th><th>Bus Out</th><th>P_in(${pUnit()})</th><th>P_out(${pUnit()})</th><th>Loss(${pUnit()})</th></tr></thead><tbody>`;
      dcdcRows.forEach((d, i) => {
        const attr = resultRowAttr({ ...d, canvas_type: d.canvas_type || 'dcdc_converter', canvas_index: d.canvas_index ?? d.index, position: d.position ?? i });
        html += `<tr${attr}><td>${d.index ?? i}</td><td>${d.bus_in}</td><td>${d.bus_out}</td>`;
        html += `<td>${pFmt(d.p_in_mw || 0, 3)}</td><td>${pFmt(d.p_out_mw || 0, 3)}</td><td>${pFmt(d.loss_mw || 0, 3)}</td></tr>`;
      });
      html += '</tbody></table>';
      dcdcDiv.innerHTML = html;
    } else {
      dcdcSec.style.display = 'none';
      dcdcDiv.innerHTML = '';
    }

    // VSC converter transfers table
    const vscSec = document.getElementById('pfVscSection');
    const vscDiv = document.getElementById('pfVscResults');
    const vscRows = firstNonEmptyArray(data.vsc_transfers, data.geo_vsc);
    if (vscRows.length > 0) {
      vscSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>AC Bus</th><th>DC Bus</th><th>Pac(${pUnit()})</th><th>Qac(${qUnit()})</th><th>Pdc(${pUnit()})</th><th>Loss(${pUnit()})</th></tr></thead><tbody>`;
      vscRows.forEach((v, i) => {
        const attr = resultRowAttr({ ...v, canvas_type: v.canvas_type || 'vsc_converter', canvas_index: v.canvas_index ?? v.index, position: v.position ?? i },
          busMap.vsc ? (busMap.vsc[v.index] ?? busMap.vsc[i]) : undefined);
        html += `<tr${attr}><td>${v.index ?? i}</td><td>${v.bus_ac}</td><td>${v.bus_dc}</td>
                 <td>${v.p_ac_mw != null ? pFmt(v.p_ac_mw, 3) : '0'}</td><td>${v.q_ac_mvar != null ? pFmt(v.q_ac_mvar, 3) : '0'}</td>
                 <td>${v.p_dc_mw != null ? pFmt(v.p_dc_mw, 3) : '0'}</td><td>${v.loss_mw != null ? pFmt(v.loss_mw, 3) : '0'}</td></tr>`;
      });
      html += '</tbody></table>';
      vscDiv.innerHTML = html;
    } else {
      vscSec.style.display = 'none';
      vscDiv.innerHTML = '';
    }

    // Clear SC and topo results
    document.getElementById('scResults').innerHTML = '';
    document.getElementById('topoResults').innerHTML = '';
  }

  function scOpt(data, key, fallback = '') {
    const opts = data?.options || {};
    return opts[key] !== undefined ? opts[key] : (data?.[key] !== undefined ? data[key] : fallback);
  }

  function scFmtOpt(value, digits = 3) {
    const n = Number(value);
    return Number.isFinite(n) ? n.toFixed(digits) : 'N/A';
  }

  function scOptionsSummaryHtml(data, detailed = false) {
    return `
      <div class="result-item"><span class="result-label">计算类型</span>
        <span class="result-value">${scOpt(data, 'calc_type', 'Max')}</span></div>
      <div class="result-item"><span class="result-label">c / κ / 拓扑</span>
        <span class="result-value">${scFmtOpt(scOpt(data, 'c_factor', 0), 3)} / ${scOpt(data, 'kappa_method', 'B')} / ${scOpt(data, 'topology', 'Meshed')}</span></div>
      <div class="result-item"><span class="result-label">Zf / x''d</span>
        <span class="result-value">${scFmtOpt(scOpt(data, 'fault_impedance_pu', 0), 4)} / ${scFmtOpt(scOpt(data, 'default_xdpp', 0.2), 3)} pu</span></div>
      <div class="result-item"><span class="result-label">tb / Tk / f</span>
        <span class="result-value">${scFmtOpt(scOpt(data, 'breaking_time_s', 0.05), 3)}s / ${scFmtOpt(scOpt(data, 'ith_duration_s', 1), 3)}s / ${scFmtOpt(scOpt(data, 'base_frequency_hz', 50), 1)}Hz</span></div>
      ${detailed ? `<div class="result-item"><span class="result-label">输出</span>
        <span class="result-value">支路:${scOpt(data, 'compute_branch_flows', true) ? '开' : '关'} 电压:${scOpt(data, 'compute_voltage_drops', true) ? '开' : '关'} Ith:${scOpt(data, 'compute_ith', true) ? '开' : '关'}</span></div>` : ''}
    `;
  }

  function showShortCircuitResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('shortCircuit');

    const summary = document.getElementById('resultsSummary');
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">故障类型</span>
        <span class="result-value">${data.fault_type || 'ThreePhase'}</span></div>
      <div class="result-item"><span class="result-label">计算母线数</span>
        <span class="result-value">${data.bus_results?.length || 0}</span></div>
      ${scOptionsSummaryHtml(data, false)}
    `;

    const scDiv = document.getElementById('scResults');
    if (data.bus_results && data.bus_results.length > 0) {
      const busMap = Canvas.getCompBusMap();
      let html = '<table><thead><tr><th>Bus</th><th>Sk(MVA)</th><th>Ik"(kA)</th></tr></thead><tbody>';
      data.bus_results.forEach(br => {
        const compId = busMap.ac[br.bus_id];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        html += `<tr${attr}><td>${br.bus_id}</td><td>${br.sk_mva?.toFixed(2) || 'N/A'}</td>
                 <td>${br.ikpp_ka?.toFixed(4) || 'N/A'}</td></tr>`;
      });
      html += '</tbody></table>';
      scDiv.innerHTML = html;
    }
  }

  // Render a single-bus (validated) fault from /api/session/sc_detailed.
  // data = { fault_type, results:[ { fault_bus_id, solved, bus_results:[...] } ] }
  function showShortCircuitDetailedResults(data, faultBus) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('shortCircuit');

    const fmt = (x, d = 3) =>
      (x === undefined || x === null || isNaN(x)) ? 'N/A' : Number(x).toFixed(d);

    const results = data.results || [];
    const r = results.find(x => x.fault_bus_id === faultBus) || results[0];
    const summary = document.getElementById('resultsSummary');
    const scDiv = document.getElementById('scResults');

    if (!r) {
      summary.innerHTML =
        '<div class="result-item"><span class="result-label">短路</span>' +
        '<span class="result-value result-failed">无结果</span></div>';
      scDiv.innerHTML = '';
      return;
    }

    const busRows = r.bus_results || [];
    const fb = busRows.find(b => b.bus_id === r.fault_bus_id) || {};

    summary.innerHTML = `
      <div class="result-item"><span class="result-label">故障类型</span>
        <span class="result-value">${data.fault_type || 'ThreePhase'}</span></div>
      <div class="result-item"><span class="result-label">故障母线</span>
        <span class="result-value">Bus ${r.fault_bus_id}</span></div>
      <div class="result-item"><span class="result-label">Ik" 初始 (kA)</span>
        <span class="result-value">${fmt(fb.ikss_ka, 3)}</span></div>
      <div class="result-item"><span class="result-label">ip 峰值 (kA)</span>
        <span class="result-value">${fmt(fb.ip_ka, 3)}</span></div>
      <div class="result-item"><span class="result-label">ib 开断 (kA)</span>
        <span class="result-value">${fmt(fb.ib_ka, 3)}</span></div>
      <div class="result-item"><span class="result-label">ik 稳态 (kA)</span>
        <span class="result-value">${fmt(fb.ik_ka, 3)}</span></div>
      <div class="result-item"><span class="result-label">Ith 热效 (kA)</span>
        <span class="result-value">${fmt(fb.ith_ka, 3)}</span></div>
      ${scOptionsSummaryHtml(data, true)}
    `;

    const busMap = Canvas.getCompBusMap();
    const contribs = [
      ['发电机', fb.ikss_gen_contrib_ka],
      ['电动机', fb.ikss_motor_contrib_ka],
      ['外部电网', fb.ikss_extgrid_contrib_ka],
      ['换流器', fb.ikss_converter_contrib_ka],
      ['静止发电机', fb.ikss_sgen_contrib_ka],
      ['负荷', fb.ikss_load_contrib_ka],
    ].filter(([, v]) => v !== undefined && v !== null);

    let html = '';
    if (contribs.length) {
      html += '<h4 style="margin:8px 0 4px">故障源贡献 (kA)</h4>';
      html += '<table><thead><tr><th>来源</th><th>Ik" (kA)</th></tr></thead><tbody>';
      contribs.forEach(([lbl, v]) => {
        html += `<tr><td>${lbl}</td><td>${fmt(v, 4)}</td></tr>`;
      });
      html += '</tbody></table>';
    }

    if (r.converter_contributions && r.converter_contributions.length) {
      html += '<h4 style="margin:12px 0 4px">换流器短路模型</h4>';
      html += '<table><thead><tr><th>ID</th><th>Bus</th><th>模型</th><th>Ir限值(pu)</th><th>贡献(kA)</th></tr></thead><tbody>';
      r.converter_contributions.forEach(c => {
        html += `<tr><td>${c.converter_index ?? ''}</td><td>${c.bus_id ?? ''}</td>` +
                `<td>${c.model || ''}</td><td>${fmt(c.i_limit_pu, 3)}</td>` +
                `<td>${fmt(c.contribution_ka, 4)}</td></tr>`;
      });
      html += '</tbody></table>';
    }

    html += '<h4 style="margin:12px 0 4px">故障期间各母线剩余电压 (p.u.)</h4>';
    html += '<table><thead><tr><th>Bus</th><th>V<sub>remain</sub> (p.u.)</th></tr></thead><tbody>';
    busRows.forEach(b => {
      const compId = busMap.ac[b.bus_id];
      const clickable = compId !== undefined
        ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
      const isFault = b.bus_id === r.fault_bus_id;
      const style = isFault ? ' style="font-weight:700;background:var(--primary-soft)"' : '';
      html += `<tr${clickable}${style}><td>${b.bus_id}${isFault ? ' (故障点)' : ''}</td>` +
              `<td>${fmt(b.v_remaining_pu, 4)}</td></tr>`;
    });
    html += '</tbody></table>';
    scDiv.innerHTML = html;
  }

  function showDcShortCircuitResults(data, primaryFaultBus) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('shortCircuit');

    const fmt = (x, d = 3) =>
      (x === undefined || x === null || isNaN(x)) ? 'N/A' : Number(x).toFixed(d);
    const results = data.results || [];
    const r = results.find(x => x.fault_bus_id === primaryFaultBus) || results[0] || {};
    const solved = results.filter(x => x.solved).length;
    const summary = document.getElementById('resultsSummary');
    const scDiv = document.getElementById('scResults');

    summary.innerHTML = `
      <div class="result-item"><span class="result-label">短路域</span>
        <span class="result-value">DC</span></div>
      <div class="result-item"><span class="result-label">计算母线数</span>
        <span class="result-value">${results.length} (${solved} solved)</span></div>
      <div class="result-item"><span class="result-label">首个故障母线</span>
        <span class="result-value">DC Bus ${r.fault_bus_id ?? 'N/A'}</span></div>
      <div class="result-item"><span class="result-label">If (kA)</span>
        <span class="result-value">${fmt(r.i_fault_ka, 4)}</span></div>
      <div class="result-item"><span class="result-label">Rth / Vpre</span>
        <span class="result-value">${fmt(r.r_thevenin_pu, 5)} pu / ${fmt(r.v_prefault_pu, 4)} pu</span></div>
      <div class="result-item"><span class="result-label">DCCB拓扑</span>
        <span class="result-value">${data.options?.consider_dc_breakers ? '开' : '关'}</span></div>
    `;

    let html = '<h4 style="margin:8px 0 4px">DC母线故障电流</h4>';
    html += '<table><thead><tr><th>DC Bus</th><th>Solved</th><th>If(kA)</th><th>If(pu)</th><th>Rth(pu)</th><th>消息</th></tr></thead><tbody>';
    results.forEach(row => {
      html += `<tr><td>${row.fault_bus_id}</td><td>${row.solved ? 'yes' : 'no'}</td>` +
              `<td>${fmt(row.i_fault_ka, 4)}</td><td>${fmt(row.i_fault_pu, 4)}</td>` +
              `<td>${fmt(row.r_thevenin_pu, 5)}</td><td>${row.message || ''}</td></tr>`;
    });
    html += '</tbody></table>';

    const duties = r.breaker_duties || [];
    if (duties.length) {
      html += '<h4 style="margin:12px 0 4px">DCCB开断能力校核</h4>';
      html += '<table><thead><tr><th>DCCB</th><th>端点</th><th>模型</th><th>Duty(kA)</th><th>Breaking(kA)</th><th>状态</th></tr></thead><tbody>';
      duties.forEach(d => {
        const ok = d.breaking_rating_ok;
        html += `<tr><td>${d.name || d.breaker_index}</td><td>${d.from_bus}-${d.to_bus}</td>` +
                `<td>${d.model || ''}</td><td>${fmt(d.i_duty_ka, 4)}</td>` +
                `<td>${fmt(d.i_breaking_ka, 4)}</td>` +
                `<td class="${ok ? 'result-ok' : 'result-failed'}">${ok ? 'OK' : '超限'}</td></tr>`;
      });
      html += '</tbody></table>';
    }
    scDiv.innerHTML = html;
  }

  // Render hybrid AC/DC harmonic power-flow results (/api/session/harmonics).
  function showHarmonicsResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('harmonics');
    hpfShowSections(['hpfAcSection', 'hpfSpectrumSection', 'hpfBranchSection']);

    const fmt = (x, d = 2) =>
      (x === undefined || x === null || isNaN(x)) ? 'N/A' : Number(x).toFixed(d);

    const summary = document.getElementById('resultsSummary');
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">状态</span>
        <span class="result-value ${data.ok ? 'result-converged' : 'result-failed'}">
          ${data.ok ? '完成' : '失败'}</span></div>
      <div class="result-item"><span class="result-label">基波潮流</span>
        <span class="result-value">${data.base_pf_converged ? '已收敛' : '未收敛/沿用存储电压'}</span></div>
      <div class="result-item"><span class="result-label">AC 谐波次数</span>
        <span class="result-value">${(data.ac_orders || []).join(', ') || '—'}</span></div>
      <div class="result-item"><span class="result-label">DC 纹波次数</span>
        <span class="result-value">${(data.dc_orders || []).join(', ') || '—'}</span></div>
      <div class="result-item"><span class="result-label">最大 AC 电压 THD</span>
        <span class="result-value">${fmt(data.max_ac_thd_pct)}% @ Bus ${data.max_ac_thd_bus}</span></div>
      ${(data.dc_bus_results && data.dc_bus_results.length) ? `
      <div class="result-item"><span class="result-label">最大 DC 纹波 THD</span>
        <span class="result-value">${fmt(data.max_dc_thd_pct)}% @ Bus ${data.max_dc_thd_bus}</span></div>` : ''}
      ${data.compliance ? `
      <div class="result-item"><span class="result-label">畸变限值 (${data.compliance.standard})</span>
        <span class="result-value ${data.compliance.all_compliant ? 'result-converged' : 'result-failed'}">
          ${data.compliance.all_compliant ? '全部合格' : (data.compliance.n_violations + ' 处越限')}</span></div>` : ''}
    `;

    // Per-bus compliance lookup for row highlighting.
    const compByBus = {};
    if (data.compliance && Array.isArray(data.compliance.checks)) {
      data.compliance.checks.forEach(c => { compByBus[c.bus] = c; });
    }
    const hasComp = Object.keys(compByBus).length > 0;

    const busMap = Canvas.getCompBusMap();
    // Build a compact spectrum string (top contributing orders) for a bus row.
    const specStr = (harmonics, fundOrder) => {
      return (harmonics || [])
        .filter(h => h.order !== fundOrder && h.mag_pu > 1e-6)
        .sort((a, b) => b.mag_pu - a.mag_pu)
        .slice(0, 3)
        .map(h => `h${h.order}:${(h.mag_pu * 100).toFixed(1)}%`)
        .join('  ') || '—';
    };

    const acDiv = document.getElementById('hpfAcResults');
    const acRows = (data.ac_bus_results || []).slice()
      .sort((a, b) => b.thd_pct - a.thd_pct);
    if (acRows.length) {
      let html = '<table><thead><tr><th>Bus</th><th>V<sub>1</sub>(p.u.)</th>' +
                 '<th>THD<sub>V</sub>(%)</th><th>主要谐波</th>' +
                 (hasComp ? '<th>限值校核</th>' : '') + '</tr></thead><tbody>';
      acRows.forEach(b => {
        const compId = busMap.ac ? busMap.ac[b.bus] : undefined;
        const attr = compId !== undefined
          ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        const hot = b.thd_pct >= 5.0 ? ' style="color:var(--red);font-weight:600"' : '';
        let compCell = '';
        if (hasComp) {
          const c = compByBus[b.bus];
          if (c) {
            const ok = c.compliant;
            const label = ok ? '合格'
              : (!c.thd_ok ? `THD>${fmt(c.thd_limit_pct, 1)}%` : `h${c.worst_ihd_order}>${fmt(c.ihd_limit_pct, 1)}%`);
            compCell = `<td style="color:${ok ? 'var(--green)' : 'var(--red)'};font-weight:600">${label}</td>`;
          } else {
            compCell = '<td>—</td>';
          }
        }
        html += `<tr${attr}><td>${b.bus}</td><td>${fmt(b.v_fund_pu, 4)}</td>` +
                `<td${hot}>${fmt(b.thd_pct)}</td><td>${specStr(b.harmonics, 1)}</td>${compCell}</tr>`;
      });
      html += '</tbody></table>';
      acDiv.innerHTML = html;
    } else {
      acDiv.innerHTML = '<p class="muted">无 AC 母线结果</p>';
    }

    const dcSection = document.getElementById('hpfDcSection');
    const dcDiv = document.getElementById('hpfDcResults');
    const dcRows = (data.dc_bus_results || []).slice()
      .sort((a, b) => b.thd_pct - a.thd_pct);
    if (dcRows.length) {
      dcSection.style.display = 'block';
      let html = '<table><thead><tr><th>Bus</th><th>V<sub>0</sub>(p.u.)</th>' +
                 '<th>THD<sub>V</sub>(%)</th><th>主要纹波</th></tr></thead><tbody>';
      dcRows.forEach(b => {
        const compId = busMap.dc ? busMap.dc[b.bus] : undefined;
        const attr = compId !== undefined
          ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        html += `<tr${attr}><td>${b.bus}</td><td>${fmt(b.v_fund_pu, 4)}</td>` +
                `<td>${fmt(b.thd_pct)}</td><td>${specStr(b.harmonics, 0)}</td></tr>`;
      });
      html += '</tbody></table>';
      dcDiv.innerHTML = html;
    } else {
      dcSection.style.display = 'none';
      dcDiv.innerHTML = '';
    }

    // Voltage spectrum bar chart (top distorted buses) + branch current flows.
    renderSpectrumChart('hpfSpectrumChart',
      (data.ac_bus_results || []).concat(data.dc_bus_results || []),
      '母线电压谐波频谱 (|V_h| / |V_1| %)');
    renderHpfBranchTable(data);
  }

  // Grouped bar chart of per-order voltage distortion (|V_h|/|V_1|·100) for the
  // most-distorted buses.  `rows` are HarmonicBusResult-like objects.
  function renderSpectrumChart(divId, rows, title) {
    const div = document.getElementById(divId);
    if (!div) return;
    if (typeof Plotly === 'undefined' || !(rows || []).length) {
      div.innerHTML = '<p class="muted">无频谱数据</p>';
      return;
    }
    const top = rows.slice()
      .sort((a, b) => (b.thd_pct || 0) - (a.thd_pct || 0))
      .slice(0, 6);
    const orderSet = new Set();
    top.forEach(b => (b.harmonics || []).forEach(h => {
      if (h.order > 1 && h.mag_pu > 1e-9) orderSet.add(h.order);
    }));
    const orders = [...orderSet].sort((a, b) => a - b);
    if (!orders.length) { div.innerHTML = '<p class="muted">无显著谐波分量</p>'; return; }
    const traces = top.map(b => {
      const v1 = b.v_fund_pu || 1.0;
      const byOrder = {};
      (b.harmonics || []).forEach(h => { byOrder[h.order] = h.mag_pu; });
      return {
        x: orders.map(o => 'h' + o),
        y: orders.map(o => (byOrder[o] || 0) / (v1 || 1) * 100),
        name: `${b.is_dc ? 'DC' : 'AC'}-${b.bus}`,
        type: 'bar',
      };
    });
    Plotly.newPlot(div, traces, {
      ...carbonPlotTheme(title),
      barmode: 'group',
      xaxis: { title: '谐波次数', gridcolor: '#3e4451' },
      yaxis: { title: '幅值 (% of V₁)', gridcolor: '#3e4451', rangemode: 'tozero' },
    }, { responsive: true, displaylogo: false });
  }

  function renderHpfBranchTable(data) {
    const div = document.getElementById('hpfBranchResults');
    if (!div) return;
    const flows = (data.ac_branch_flows || []).concat(data.dc_branch_flows || []);
    const rows = flows.filter(f => (f.thd_i_pct || 0) > 1e-6)
      .sort((a, b) => (b.thd_i_pct || 0) - (a.thd_i_pct || 0)).slice(0, 30);
    if (!rows.length) { div.innerHTML = '<p class="muted">无支路谐波电流数据</p>'; return; }
    const fmt = (x, d = 3) => (x == null || isNaN(x)) ? '—' : Number(x).toFixed(d);
    const topSpec = (hs, fund) => (hs || [])
      .filter(h => h.order !== fund && (h.i_pu || 0) > 1e-9)
      .sort((a, b) => b.i_pu - a.i_pu).slice(0, 3)
      .map(h => `h${h.order}:${fmt(h.i_pu, 4)}`).join('  ') || '—';
    let html = '<table><thead><tr><th>支路</th><th>类型</th><th>THD<sub>I</sub>(%)</th>'
             + '<th>主要分量 (pu)</th></tr></thead><tbody>';
    rows.forEach(f => {
      html += `<tr><td>${f.from_bus}→${f.to_bus}</td><td>${f.is_dc ? 'DC' : 'AC'}</td>`
            + `<td>${fmt(f.thd_i_pct, 2)}</td><td>${topSpec(f.harmonics, f.is_dc ? 0 : 1)}</td></tr>`;
    });
    div.innerHTML = html + '</tbody></table>';
  }

  // ── Frequency scan / resonance ──
  function showHarmonicsFreqScan(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('harmonics');
    hpfShowSections(['hpfFreqScanSection']);
    const fmt = (x, d = 3) => (x == null || isNaN(x)) ? 'N/A' : Number(x).toFixed(d);

    const reson = data.resonances || [];
    const parallelN = reson.filter(r => r.parallel).length;
    document.getElementById('resultsSummary').innerHTML = `
      <div class="result-item"><span class="result-label">模式</span>
        <span class="result-value">${data.sequence ? '序分量扫描' : '驱动点阻抗扫描'}</span></div>
      <div class="result-item"><span class="result-label">频点数</span>
        <span class="result-value">${(data.freqs || []).length}</span></div>
      <div class="result-item"><span class="result-label">谐振点</span>
        <span class="result-value">${reson.length} (并联 ${parallelN} / 串联 ${reson.length - parallelN})</span></div>`;

    const chart = document.getElementById('hpfFreqScanChart');
    if (typeof Plotly !== 'undefined' && (data.freqs || []).length) {
      let traces = [];
      if (data.sequence) {
        traces = [
          { x: data.freqs, y: data.z1_mag, name: '正序 Z₁', type: 'scatter', mode: 'lines' },
          { x: data.freqs, y: data.z2_mag, name: '负序 Z₂', type: 'scatter', mode: 'lines' },
          { x: data.freqs, y: data.z0_mag, name: '零序 Z₀', type: 'scatter', mode: 'lines' },
        ];
      } else {
        traces = (data.buses || []).map(b => ({
          x: data.freqs, y: b.z_mag, name: 'Bus ' + b.bus, type: 'scatter', mode: 'lines',
        }));
      }
      // Resonance markers.
      if (reson.length) {
        traces.push({
          x: reson.map(r => r.freq_order), y: reson.map(r => r.z_mag),
          name: '谐振', mode: 'markers',
          marker: { size: 9, symbol: reson.map(r => r.parallel ? 'triangle-up' : 'triangle-down'),
                    color: reson.map(r => r.parallel ? '#e06c75' : '#61afef') },
        });
      }
      Plotly.newPlot(chart, traces, {
        ...carbonPlotTheme('驱动点阻抗 |Z(f)|'),
        xaxis: { title: '谐波次数 (f / f₁)', gridcolor: '#3e4451' },
        yaxis: { title: '|Z| (p.u.)', gridcolor: '#3e4451', rangemode: 'tozero' },
      }, { responsive: true, displaylogo: false });
    } else {
      chart.innerHTML = '<p class="muted">无扫描数据</p>';
    }

    const tbl = document.getElementById('hpfResonanceResults');
    if (reson.length) {
      let html = '<table><thead><tr><th>母线</th><th>次数</th><th>|Z|(p.u.)</th>'
               + '<th>类型</th>' + (data.sequence ? '<th>序</th>' : '') + '</tr></thead><tbody>';
      reson.slice().sort((a, b) => b.z_mag - a.z_mag).forEach(r => {
        const seqName = ['零序', '正序', '负序'][r.sequence] || '—';
        html += `<tr><td>${r.bus}</td><td>${fmt(r.freq_order, 2)}</td><td>${fmt(r.z_mag, 3)}</td>`
              + `<td style="color:${r.parallel ? 'var(--red)' : 'var(--blue,#61afef)'}">`
              + `${r.parallel ? '并联(峰)' : '串联(谷)'}</td>`
              + (data.sequence ? `<td>${seqName}</td>` : '') + '</tr>';
      });
      tbl.innerHTML = html + '</tbody></table>';
    } else {
      tbl.innerHTML = '<p class="muted">未检出谐振点</p>';
    }
  }

  // ── Three-phase (abc) harmonic distortion ──
  function showHarmonics3ph(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('harmonics');
    hpfShowSections(['hpf3phSection']);
    const fmt = (x, d = 2) => (x == null || isNaN(x)) ? 'N/A' : Number(x).toFixed(d);

    document.getElementById('resultsSummary').innerHTML = `
      <div class="result-item"><span class="result-label">状态</span>
        <span class="result-value ${data.ok ? 'result-converged' : 'result-failed'}">${data.ok ? '完成' : '失败'}</span></div>
      <div class="result-item"><span class="result-label">AC 谐波次数</span>
        <span class="result-value">${(data.ac_orders || []).join(', ') || '—'}</span></div>
      <div class="result-item"><span class="result-label">最大相 THD</span>
        <span class="result-value">${fmt(data.max_thd_pct)}% @ Bus ${data.max_thd_bus}</span></div>`;

    const rows = (data.bus_results || []).slice().sort((a, b) =>
      Math.max(b.thd_a_pct, b.thd_b_pct, b.thd_c_pct) - Math.max(a.thd_a_pct, a.thd_b_pct, a.thd_c_pct));
    const div = document.getElementById('hpf3phResults');
    if (rows.length) {
      let html = '<table><thead><tr><th>Bus</th><th>THD<sub>A</sub>(%)</th>'
               + '<th>THD<sub>B</sub>(%)</th><th>THD<sub>C</sub>(%)</th></tr></thead><tbody>';
      rows.forEach(b => {
        const hot = v => v >= 5.0 ? ' style="color:var(--red);font-weight:600"' : '';
        html += `<tr><td>${b.bus}</td><td${hot(b.thd_a_pct)}>${fmt(b.thd_a_pct)}</td>`
              + `<td${hot(b.thd_b_pct)}>${fmt(b.thd_b_pct)}</td>`
              + `<td${hot(b.thd_c_pct)}>${fmt(b.thd_c_pct)}</td></tr>`;
      });
      div.innerHTML = html + '</tbody></table>';
    } else {
      div.innerHTML = '<p class="muted">无三相母线结果</p>';
    }

    const chart = document.getElementById('hpf3phChart');
    if (typeof Plotly !== 'undefined' && rows.length) {
      const top = rows.slice(0, 8);
      const mk = (key, name) => ({
        x: top.map(b => 'Bus ' + b.bus), y: top.map(b => b[key]), name, type: 'bar',
      });
      Plotly.newPlot(chart, [mk('thd_a_pct', 'A 相'), mk('thd_b_pct', 'B 相'), mk('thd_c_pct', 'C 相')], {
        ...carbonPlotTheme('各相电压 THD'),
        barmode: 'group',
        xaxis: { title: '母线', gridcolor: '#3e4451' },
        yaxis: { title: 'THD (%)', gridcolor: '#3e4451', rangemode: 'tozero' },
      }, { responsive: true, displaylogo: false });
    } else {
      chart.innerHTML = '';
    }
  }

  // ── Harmonic metrics: K-factor, TDD, losses ──
  function showHarmonicsMetrics(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('harmonics');
    hpfShowSections(['hpfMetricsSection']);
    const fmt = (x, d = 3) => (x == null || isNaN(x)) ? 'N/A' : Number(x).toFixed(d);

    document.getElementById('resultsSummary').innerHTML = `
      <div class="result-item"><span class="result-label">最大 K 系数</span>
        <span class="result-value">${fmt(data.max_k_factor, 2)} @ 支路 ${data.max_k_factor_branch}</span></div>
      <div class="result-item"><span class="result-label">最大电流 THD</span>
        <span class="result-value">${fmt(data.max_thd_i_pct, 2)}%</span></div>
      <div class="result-item"><span class="result-label">最大 TDD</span>
        <span class="result-value">${fmt(data.max_tdd_pct, 2)}%</span></div>
      <div class="result-item"><span class="result-label">谐波损耗占比</span>
        <span class="result-value">${fmt((data.harmonic_loss_fraction || 0) * 100, 3)}%</span></div>
      <div class="result-item"><span class="result-label">总损耗 (p.u.)</span>
        <span class="result-value">${fmt(data.total_loss_pu, 5)}</span></div>`;

    const rows = (data.branches || []).slice().sort((a, b) => b.k_factor - a.k_factor);
    const div = document.getElementById('hpfMetricsResults');
    if (rows.length) {
      let html = '<table><thead><tr><th>支路</th><th>K 系数</th><th>THD<sub>I</sub>(%)</th>'
               + '<th>TDD(%)</th><th>I<sub>rms</sub>(pu)</th><th>谐波损耗(pu)</th></tr></thead><tbody>';
      rows.slice(0, 40).forEach(b => {
        const hotK = b.k_factor >= 4 ? ' style="color:var(--red);font-weight:600"' : '';
        html += `<tr><td>${b.from_bus}→${b.to_bus}</td><td${hotK}>${fmt(b.k_factor, 2)}</td>`
              + `<td>${fmt(b.thd_i_pct, 2)}</td><td>${fmt(b.tdd_pct, 2)}</td>`
              + `<td>${fmt(b.i_rms_pu, 4)}</td><td>${fmt(b.p_loss_harmonic_pu, 6)}</td></tr>`;
      });
      div.innerHTML = html + '</tbody></table>';
    } else {
      div.innerHTML = '<p class="muted">无支路指标</p>';
    }

    const chart = document.getElementById('hpfMetricsChart');
    if (typeof Plotly !== 'undefined' && rows.length) {
      const top = rows.slice(0, 12);
      Plotly.newPlot(chart, [{
        x: top.map(b => `${b.from_bus}→${b.to_bus}`), y: top.map(b => b.k_factor),
        type: 'bar', name: 'K 系数', marker: { color: '#e5c07b' },
      }], {
        ...carbonPlotTheme('支路 K 系数'),
        xaxis: { title: '支路', tickangle: -40, gridcolor: '#3e4451' },
        yaxis: { title: 'K 系数', gridcolor: '#3e4451', rangemode: 'tozero' },
      }, { responsive: true, displaylogo: false });
    } else {
      chart.innerHTML = '';
    }
  }

  // ── Newton (nonlinear) harmonic power flow ──
  function showHarmonicsNewton(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('harmonics');
    hpfShowSections(['hpfNewtonSection', 'hpfSpectrumSection']);
    const fmt = (x, d = 2) => (x == null || isNaN(x)) ? 'N/A' : Number(x).toFixed(d);

    document.getElementById('resultsSummary').innerHTML = `
      <div class="result-item"><span class="result-label">类型</span>
        <span class="result-value">${data.mode === 'holomorphic' ? '二次电压相关' : '恒功率谐波负荷'}</span></div>
      <div class="result-item"><span class="result-label">收敛</span>
        <span class="result-value ${data.converged ? 'result-converged' : 'result-failed'}">
          ${data.converged ? '是' : '否'} (${data.max_iterations_used} 次)</span></div>
      <div class="result-item"><span class="result-label">最大 AC THD</span>
        <span class="result-value">${fmt(data.max_ac_thd_pct)}% @ Bus ${data.max_ac_thd_bus}</span></div>`;

    const div = document.getElementById('hpfNewtonResults');
    const iters = data.iterations || {}, resid = data.final_residual || {};
    const orders = Object.keys(iters).sort((a, b) => a - b);
    let html = '';
    if (orders.length) {
      html += '<table><thead><tr><th>次数</th><th>牛顿迭代</th><th>残差 ‖r‖∞</th></tr></thead><tbody>';
      orders.forEach(o => {
        html += `<tr><td>h${o}</td><td>${iters[o]}</td><td>${(resid[o] ?? 0).toExponential(2)}</td></tr>`;
      });
      html += '</tbody></table>';
    }
    const rows = (data.ac_bus_results || []).slice().sort((a, b) => b.thd_pct - a.thd_pct).slice(0, 20);
    if (rows.length) {
      html += '<table style="margin-top:8px"><thead><tr><th>Bus</th><th>V<sub>1</sub>(p.u.)</th>'
            + '<th>THD<sub>V</sub>(%)</th></tr></thead><tbody>';
      rows.forEach(b => {
        html += `<tr><td>${b.bus}</td><td>${fmt(b.v_fund_pu, 4)}</td><td>${fmt(b.thd_pct)}</td></tr>`;
      });
      html += '</tbody></table>';
    }
    div.innerHTML = html || '<p class="muted">无结果</p>';
    renderSpectrumChart('hpfSpectrumChart', data.ac_bus_results || [], '母线电压谐波频谱');
  }

  function showTopologyResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('topology');

    const summary = document.getElementById('resultsSummary');
    const totalSwOn  = (data.sw_on_per_step  || []).reduce((a,b) => a+b, 0);
    const totalSwOff = (data.sw_off_per_step || []).reduce((a,b) => a+b, 0);
    const totalCurt  = (data.curt_per_step   || []).reduce((a,b) => a+b, 0);

    // Loss comparison display
    const baseLoss = data.base_loss_mw ?? 0;
    const reconLoss = data.reconfig_loss_mw ?? 0;
    const lossRed = data.loss_reduction_mw ?? 0;
    const lossRedPct = data.loss_reduction_pct ?? 0;
    const lossCls = lossRed > 0.001 ? 'result-converged' : (lossRed < -0.001 ? 'result-failed' : '');

    summary.innerHTML = `
      <div class="result-item"><span class="result-label">可行</span>
        <span class="result-value ${data.feasible !== false ? 'result-converged' : 'result-failed'}">
          ${data.feasible !== false ? '✓ 是' : '✗ 否'}</span></div>
      <div class="result-item"><span class="result-label">求解器</span>
        <span class="result-value">${data.solver_name || 'N/A'}</span></div>
      <div class="result-item"><span class="result-label">MILP目标值</span>
        <span class="result-value">${data.milp_objective?.toFixed(4) || 'N/A'}</span></div>
      <div class="result-item"><span class="result-label">重构前损耗(MW)</span>
        <span class="result-value">${data.base_pf_converged ? baseLoss.toFixed(4) : '潮流未收敛'}</span></div>
      <div class="result-item"><span class="result-label">重构后损耗(MW)</span>
        <span class="result-value">${data.reconfig_pf_converged ? reconLoss.toFixed(4) : '潮流未收敛'}</span></div>
      <div class="result-item"><span class="result-label">损耗降低</span>
        <span class="result-value ${lossCls}">
          ${lossRed.toFixed(4)} MW (${lossRedPct.toFixed(1)}%)</span></div>
      <div class="result-item"><span class="result-label">时步数</span>
        <span class="result-value">${data.num_steps || 'N/A'}</span></div>
      <div class="result-item"><span class="result-label">开关操作</span>
        <span class="result-value">闭合 ${totalSwOn} / 断开 ${totalSwOff}</span></div>
      <div class="result-item"><span class="result-label">弃风弃光(MW)</span>
        <span class="result-value">${totalCurt.toFixed(2)}</span></div>
    `;

    const topoDiv = document.getElementById('topoResults');
    let html = '';

    // --- Visualize topology reconfiguration results on main diagram ---
    if (typeof Canvas !== 'undefined' && Canvas.showTopologyReconfigResults) {
      Canvas.showTopologyReconfigResults(data);
    }
    const busMap = Canvas.getCompBusMap();
    const makeBrLinks = (ids) => ids.map(id => {
      const compId = busMap.branch ? busMap.branch[id] : undefined;
      return compId !== undefined
        ? `<span class="clickable-branch" onclick="Canvas.panToComponent(${compId})">${id}</span>`
        : `${id}`;
    });

    // Topology verification section
    html += `<h4 style="margin:8px 0 6px;">拓扑验证</h4>`;
    html += `<table><thead><tr><th></th><th>重构前</th><th>重构后</th></tr></thead><tbody>`;
    html += `<tr><td>辐射状</td>
      <td class="${data.base_is_radial ? 'grade-green' : 'grade-red'}">${data.base_is_radial ? '✓ 是' : '✗ 否'}</td>
      <td class="${data.reconfig_is_radial ? 'grade-green' : 'grade-red'}">${data.reconfig_is_radial ? '✓ 是' : '✗ 否'}</td></tr>`;
    html += `<tr><td>连通</td>
      <td class="${data.base_is_connected ? 'grade-green' : 'grade-red'}">${data.base_is_connected ? '✓ 是' : '✗ 否'}</td>
      <td class="${data.reconfig_is_connected ? 'grade-green' : 'grade-red'}">${data.reconfig_is_connected ? '✓ 是' : '✗ 否'}</td></tr>`;
    html += `<tr><td>孤岛数</td><td>${data.base_islands ?? '—'}</td><td>${data.reconfig_islands ?? '—'}</td></tr>`;
    html += `<tr><td>闭合支路数</td><td>—</td><td>${data.reconfig_closed_count ?? '—'} / ${data.num_buses ? data.num_buses - 1 : '—'} (n-1)</td></tr>`;
    html += `<tr><td>潮流收敛</td>
      <td>${data.base_pf_converged ? '✓ 收敛' : '✗'}</td>
      <td>${data.reconfig_pf_converged ? '✓ 收敛' : '✗'}</td></tr>`;
    html += `<tr><td>有功损耗(MW)</td>
      <td>${data.base_pf_converged ? baseLoss.toFixed(4) : '—'}</td>
      <td>${data.reconfig_pf_converged ? reconLoss.toFixed(4) : '—'}</td></tr>`;
    html += `<tr><td><strong>损耗变化</strong></td><td colspan="2" class="${lossCls}"><strong>`;
    if (data.base_pf_converged && data.reconfig_pf_converged) {
      html += lossRed > 0 ? `↓ 降低 ${lossRed.toFixed(4)} MW (${lossRedPct.toFixed(1)}%)` :
              lossRed < 0 ? `↑ 增加 ${(-lossRed).toFixed(4)} MW (${(-lossRedPct).toFixed(1)}%)` :
              '无变化';
    } else { html += '无法比较'; }
    html += `</strong></td></tr></tbody></table>`;

    if (data.open_branch_ids && data.open_branch_ids.length > 0) {
      html += `<p><strong>断开支路 (最终):</strong> [${makeBrLinks(data.open_branch_ids).join(', ')}]</p>`;
    }
    if (data.closed_branch_ids && data.closed_branch_ids.length > 0) {
      html += `<p><strong>闭合支路 (最终):</strong> [${makeBrLinks(data.closed_branch_ids).join(', ')}]</p>`;
    }

    // Detailed branch status table — show which lines are open/closed with from/to bus
    if (data.branch_details && data.branch_details.length > 0) {
      const openBranches = data.branch_details.filter(b => !b.closed);
      const closedBranches = data.branch_details.filter(b => b.closed);
      html += `<h4 style="margin:12px 0 6px;">重构后支路状态明细</h4>`;
      // Open (disconnected) branches table
      if (openBranches.length > 0) {
        html += `<p><strong>断开支路 (${openBranches.length}条):</strong></p>`;
        html += `<table class="result-table"><thead><tr><th>支路ID</th><th>起始母线</th><th>终止母线</th><th>状态</th></tr></thead><tbody>`;
        for (const b of openBranches) {
          const compId = busMap.branch ? busMap.branch[b.id] : undefined;
          const idCell = compId !== undefined
            ? `<span class="clickable-branch" onclick="Canvas.panToComponent(${compId})">${b.id}</span>`
            : `${b.id}`;
          html += `<tr><td>${idCell}</td><td>${b.from_bus}</td><td>${b.to_bus}</td><td style="color:#e74c3c;font-weight:bold;">断开</td></tr>`;
        }
        html += `</tbody></table>`;
      }
      // Closed (connected) branches table
      if (closedBranches.length > 0) {
        html += `<details style="margin-top:8px;"><summary><strong>闭合支路 (${closedBranches.length}条) ▸ 点击展开</strong></summary>`;
        html += `<table class="result-table"><thead><tr><th>支路ID</th><th>起始母线</th><th>终止母线</th><th>状态</th></tr></thead><tbody>`;
        for (const b of closedBranches) {
          const compId = busMap.branch ? busMap.branch[b.id] : undefined;
          const idCell = compId !== undefined
            ? `<span class="clickable-branch" onclick="Canvas.panToComponent(${compId})">${b.id}</span>`
            : `${b.id}`;
          html += `<tr><td>${idCell}</td><td>${b.from_bus}</td><td>${b.to_bus}</td><td style="color:#27ae60;">闭合</td></tr>`;
        }
        html += `</tbody></table></details>`;
      }
    }
    if (data.verification_pf) {
      const pf = data.verification_pf;
      html += `<p><strong>验证潮流:</strong> ${pf.converged ? '收敛' : '未收敛'},
               迭代${pf.iterations}次, 残差=${Number(pf.residual || 0).toExponential(4)}</p>`;
    }
    // Per-step topology changes table
    if (data.steps_topology && data.steps_topology.length > 0) {
      html += `<h4 style="margin:12px 0 6px;">各时步拓扑变化</h4>`;
      html += `<table class="result-table"><thead><tr><th>时步</th><th>断开支路数</th><th>闭合支路数</th><th>闭合操作</th><th>断开操作</th></tr></thead><tbody>`;
      const swOn  = data.sw_on_per_step  || [];
      const swOff = data.sw_off_per_step || [];
      for (let t = 0; t < data.steps_topology.length; t++) {
        const st = data.steps_topology[t];
        html += `<tr><td>${t}</td><td>${st.open.length}</td><td>${st.closed.length}</td><td>${swOn[t]||0}</td><td>${swOff[t]||0}</td></tr>`;
      }
      html += `</tbody></table>`;
    }
    // Per-step curtailment and switching chart placeholders
    if (data.sw_on_per_step && data.num_steps > 1) {
      html += `<div id="rcChartSwitch" style="width:100%;height:220px;margin-top:12px;"></div>`;
      html += `<div id="rcChartCurt" style="width:100%;height:220px;margin-top:8px;"></div>`;
    }
    if (data.num_ess > 0) {
      html += `<div id="rcChartESS" style="width:100%;height:220px;margin-top:8px;"></div>`;
    }
    topoDiv.innerHTML = html || '<p class="empty-hint">无拓扑变化数据</p>';

    // Render Plotly charts if available
    if (typeof Plotly !== 'undefined' && data.sw_on_per_step && data.num_steps > 1) {
      const T = data.num_steps;
      const hrs = Array.from({length:T}, (_,i) => i);
      const swDiv = document.getElementById('rcChartSwitch');
      if (swDiv) {
        Plotly.newPlot(swDiv, [
          {x:hrs, y:data.sw_on_per_step,  type:'bar', name:'闭合操作', marker:{color:'#27ae60'}},
          {x:hrs, y:data.sw_off_per_step, type:'bar', name:'断开操作', marker:{color:'#e74c3c'}},
        ], {title:'各时步开关操作', barmode:'group', xaxis:{title:'时步'}, yaxis:{title:'次数'},
            margin:{l:50,r:15,t:35,b:40}, font:{size:11}}, {responsive:true});
      }
      const curtDiv = document.getElementById('rcChartCurt');
      if (curtDiv && data.curt_per_step) {
        Plotly.newPlot(curtDiv, [
          {x:hrs, y:data.curt_per_step, mode:'lines+markers', name:'弃风弃光(MW)', line:{color:'#b5651d',width:2}},
        ], {title:'各时步弃电量', xaxis:{title:'时步'}, yaxis:{title:'MW'},
            margin:{l:50,r:15,t:35,b:40}, font:{size:11}}, {responsive:true});
      }
      const essDiv = document.getElementById('rcChartESS');
      if (essDiv && data.ess_soc_by_step && data.num_ess > 0) {
        const traces = Array.from({length:data.num_ess}, (_, si) => ({
          x: hrs,
          y: data.ess_soc_by_step.map(row => (row && row[si]) || 0),
          mode: 'lines+markers',
          name: `储能 ${si+1} SOC`
        }));
        Plotly.newPlot(essDiv, traces, {title:'储能SOC变化', xaxis:{title:'时步'},
            yaxis:{title:'SOC', range:[0,1]}, margin:{l:50,r:15,t:35,b:40}, font:{size:11}},
            {responsive:true});
      }
    }
  }

  // ========== Topology Tables ==========
  // Only these solvers can be selected directly on hybrid AC/DC cases.
  // The others are AC-only (pure_ac/fdpf/three_phase), DC-only (dc), or a
  // one-shot linearization (hybrid_linearized), so their hybrid answers are not
  // directly comparable with the coupled Newton family.
  const HYBRID_PF_METHODS = new Set(['ac_newton', 'adaptive', 'islanded', 'distributed_slack']);
  function updatePfMethodAvailability(sys) {
    const sel = document.getElementById('pfMethod');
    if (!sel) return;
    const dc = (sys && sys.dc) ? sys.dc : {};
    const hasDc = (Array.isArray(dc.buses) && dc.buses.length > 0) ||
      (sys && Array.isArray(sys.vsc_converters) && sys.vsc_converters.length > 0) ||
      (sys && Array.isArray(sys.dcdc_converters) && sys.dcdc_converters.length > 0);
    let selectedGotDisabled = false;
    Array.from(sel.options).forEach(opt => {
      const restrict = hasDc && !HYBRID_PF_METHODS.has(opt.value);
      opt.disabled = restrict;
      opt.title = restrict ? '该算法不适合作为混合交直流算例的直接GUI潮流方法' : '';
      if (restrict && opt.selected) selectedGotDisabled = true;
    });
    if (selectedGotDisabled) sel.value = 'ac_newton';
  }

  function onTopologyChanged() {
    _canvasDirty = true;
    // Rewiring changes which buses a device connects to — re-derive those
    // connection params so the property panel and exports stay in sync.
    if (Canvas.syncConnectivity) Canvas.syncConnectivity();
    updateTopologyTables();
    // Reflect the updated connection info in the open property panel immediately.
    const sel = Canvas.state.selectedId;
    if (sel !== null && sel !== undefined) onSelectionChanged(sel);
  }

  function updateTopologyTables() {
    const sys = Canvas.buildSystemJson();
    const m = Canvas.getCompBusMap();
    updatePfMethodAvailability(sys);
    markResilienceFaultBranches();
    renderComponentCurveTargets('timeSeries');
    renderComponentCurveTargets('scenario');
    renderComponentCurveTargets('resilience');

    // Helper: populate a table and toggle its section visibility
    function fillTable(bodyId, sectionId, items, mapObj, rowFn) {
      const body = document.querySelector(bodyId + ' tbody');
      if (!body) return;
      body.innerHTML = '';
      if (sectionId) {
        const sec = document.getElementById(sectionId);
        if (sec) sec.style.display = items.length ? '' : 'none';
      }
      items.forEach(item => {
        const tr = document.createElement('tr');
        const compId = mapObj ? mapObj[item.index] : undefined;
        if (compId !== undefined) {
          tr.dataset.compId = compId;
          tr.addEventListener('click', () => Canvas.panToComponent(compId));
        }
        tr.innerHTML = rowFn(item);
        body.appendChild(tr);
      });
    }

    // Aggregate load per bus from load components
    const busLoad = {};
    sys.ac.loads.forEach(ld => {
      if (!busLoad[ld.bus]) busLoad[ld.bus] = { p: 0, q: 0 };
      busLoad[ld.bus].p += (ld.p_mw || 0) * (ld.scaling || 1);
      busLoad[ld.bus].q += (ld.q_mvar || 0) * (ld.scaling || 1);
    });

    // Bus table (always shown)
    fillTable('#busTableInner', null, sys.ac.buses, m.ac, bus => {
      const ld = busLoad[bus.index] || { p: bus.pd_mw, q: bus.qd_mvar };
      return `<td>${bus.index}</td><td>${bus.bus_type}</td>
        <td>${bus.vm_pu.toFixed(4)}</td><td>${bus.va_deg.toFixed(2)}</td>
        <td>${ld.p.toFixed(2)}</td><td>${ld.q.toFixed(2)}</td>
        <td>${bus.base_kv}</td>`;
    });

    // Branch table (always shown, includes _from_branch transformers)
    fillTable('#branchTableInner', null, sys.ac.branches, m.branch, br =>
      `<td>${br.index ?? ''}</td><td>${escapeHtml(br.name || '')}</td><td>${br.from_bus}</td><td>${br.to_bus}</td>
        <td>${Number(br.r_pu || 0).toFixed(6)}</td><td>${Number(br.x_pu || 0).toFixed(6)}</td>
        <td>${Number(br.b_pu || 0).toFixed(6)}</td><td>${br.rate_a_mva}</td>
        <td>${(br.tap || 1).toFixed(4)}</td><td>${(br.shift_deg || 0).toFixed(2)}</td>`);

    // Generator table (always shown)
    fillTable('#genTableInner', null, sys.ac.generators, m.gen, gen =>
      `<td>${gen.bus}</td><td>${gen.pg_mw}</td>
        <td>${gen.qg_mvar}</td><td>${gen.vg_pu}</td>
        <td>${gen.pmax_mw}</td><td>${gen.pmin_mw}</td>
        <td>${carbonFactorDisplay(gen.emission_factor_tco2_mwh || gen.co2_emission_rate || 0).toFixed(1)}</td>
        <td>${gen.is_slack ? '✓' : ''}</td>`);

    // Load table
    fillTable('#loadTableInner', 'loadSection', sys.ac.loads, m.load, ld =>
      `<td>${ld.bus}</td><td>${ld.p_mw}</td><td>${ld.q_mvar}</td><td>${ld.scaling}</td>`);

    // Transformer table (non-from_branch only)
    fillTable('#trafoTableInner', 'trafoSection', sys.ac.transformers_2w, m.trafo, t =>
      `<td>${t.hv_bus}</td><td>${t.lv_bus}</td><td>${t.sn_mva}</td>
        <td>${t.vk_percent}</td><td>${t.shift_deg}</td>`);

    // External Grid
    fillTable('#extGridTableInner', 'extGridSection', sys.ac.external_grids, m.extGrid, eg =>
      `<td>${eg.bus}</td><td>${eg.vm_pu}</td><td>${eg.va_deg}</td><td>${eg.s_sc_max_mva}</td>
        <td>${carbonFactorDisplay(eg.emission_factor_tco2_mwh || eg.co2_emission_rate || 0).toFixed(1)}</td>`);

    // Storage
    fillTable('#storageTableInner', 'storageSection', sys.ac.storage, m.storage, s =>
      `<td>${s.bus}</td><td>${s.p_rated_mw}</td><td>${s.e_rated_mwh}</td><td>${s.soc_init}</td>`);

    // PV System
    fillTable('#pvTableInner', 'pvSection', sys.ac.pv_systems, m.pv, pv =>
      `<td>${pv.bus}</td><td>${pv.p_mw}</td><td>${pv.q_mvar}</td><td>${pv.sn_mva}</td>`);

    // Renewable Gen
    fillTable('#renGenTableInner', 'renGenSection', sys.ac.renewable_gens, m.renGen, rg =>
      `<td>${rg.bus}</td><td>${rg.type}</td><td>${rg.p_mw}</td><td>${rg.p_rated_mw}</td><td>${rg.capacity_factor}</td>`);

    // Static Generator
    fillTable('#sgenTableInner', 'sgenSection', sys.ac.static_generators, m.sgen, sg =>
      `<td>${sg.bus}</td><td>${sg.p_mw}</td><td>${sg.q_mvar}</td><td>${sg.sgen_type}</td>`);

    // Shunt
    fillTable('#shuntTableInner', 'shuntSection', sys.ac.shunts, m.shunt, sh =>
      `<td>${sh.bus}</td><td>${sh.gs_mw}</td><td>${sh.bs_mvar}</td><td>${sh.switchable ? '✓' : ''}</td>`);

    // Switch
    fillTable('#switchTableInner', 'switchSection', sys.ac.switches, m.sw, sw =>
      `<td>${sw.bus_from}</td><td>${sw.bus_to}</td><td>${sw.closed ? '✓' : '✗'}</td>`);

    // Circuit Breaker
    fillTable('#cbTableInner', 'cbSection', sys.ac.circuit_breakers, m.cb, cb =>
      `<td>${cb.bus_from}</td><td>${cb.bus_to}</td><td>${cb.closed ? '✓' : '✗'}</td><td>${cb.rated_current_ka}</td>`);

    // Motor
    fillTable('#motorTableInner', 'motorSection', sys.ac.motors, m.motor, mt =>
      `<td>${mt.bus}</td><td>${mt.sn_mva}</td><td>${mt.cos_phi}</td><td>${mt.efficiency}</td>`);

    // Transformer 3W
    fillTable('#trafo3wTableInner', 'trafo3wSection', sys.ac.transformers_3w, m.trafo3w, t =>
      `<td>${t.hv_bus}</td><td>${t.mv_bus}</td><td>${t.lv_bus}</td><td>${t.sn_hv_mva}</td>`);

    // Flexible Load
    fillTable('#flexLoadTableInner', 'flexLoadSection', sys.ac.flexible_loads, m.flexLoad, fl =>
      `<td>${fl.bus}</td><td>${fl.p_mw}</td><td>${fl.flex_up_mw}</td><td>${fl.flex_down_mw}</td>`);

    // Asymmetric Load
    fillTable('#asymLoadTableInner', 'asymLoadSection', sys.ac.asymmetric_loads, m.asymLoad, al =>
      `<td>${al.bus}</td><td>${al.pa_mw}</td><td>${al.pb_mw}</td><td>${al.pc_mw}</td>`);

    // DC Bus
    fillTable('#dcBusTableInner', 'dcBusSection', sys.dc.buses, m.dc, db =>
      `<td>${db.index}</td><td>${db.bus_type || ''}</td><td>${(db.vm_pu || 1).toFixed(4)}</td><td>${db.base_kv || ''}</td>`);

    // DC Branch
    fillTable('#dcBranchTableInner', 'dcBranchSection', sys.dc.branches, m.dcBranch, db =>
      `<td>${db.index ?? ''}</td><td>${escapeHtml(db.name || '')}</td><td>${db.from_bus}</td><td>${db.to_bus}</td><td>${db.r_pu}</td><td>${db.rate_a_mva}</td>`);

    // DC Load
    fillTable('#dcLoadTableInner', 'dcLoadSection', sys.dc.loads, m.dcLoad, dl =>
      `<td>${dl.bus}</td><td>${dl.p_mw}</td><td>${dl.scaling}</td>`);

    // DC Storage
    fillTable('#dcStorageTableInner', 'dcStorageSection', sys.dc.dc_storage || [], m.dcStorage, s =>
      `<td>${s.bus}</td><td>${s.p_mw}</td><td>${s.p_rated_mw}</td><td>${s.e_rated_mwh}</td><td>${s.soc_init}</td>`);

    // DC PV Array
    fillTable('#dcPvTableInner', 'dcPvSection', sys.dc.pv_arrays || [], m.dcPv, pv =>
      `<td>${pv.bus}</td><td>${pv.p_set_mw ?? ''}</td><td>${pv.irradiance ?? ''}</td><td>${pv.temperature ?? ''}</td>`);

    // VSC Converter
    fillTable('#vscTableInner', 'vscSection', sys.vsc_converters, m.vsc, v =>
      `<td>${v.bus_ac}</td><td>${v.bus_dc}</td><td>${v.p_set_mw}</td><td>${v.control_mode}</td>`);

    // Charger
    fillTable('#chargerTableInner', 'chargerSection', sys.ac.chargers, m.charger, ch =>
      `<td>${ch.charger_type}</td><td>${ch.p_rated_kw}</td><td>${ch.eta}</td><td>${ch.v2g_capable ? '✓' : ''}</td>`);

    // Charging Station
    fillTable('#csTableInner', 'csSection', sys.ac.charging_stations, m.chargingStation, cs =>
      `<td>${cs.bus}</td><td>${cs.n_fast}</td><td>${cs.n_slow}</td><td>${cs.max_power_kw}</td>`);

    // Mobile Storage
    fillTable('#msTableInner', 'msSection', sys.mobile_storage, m.mobileStorage, ms =>
      `<td>${ms.bus}</td><td>${ms.p_rated_mw}</td><td>${ms.e_rated_mwh}</td><td>${ms.soc_init}</td>`);

    // DC-DC Converter
    fillTable('#dcdcTableInner', 'dcdcSection', sys.dcdc_converters, m.dcdcConverter, dc =>
      `<td>${dc.bus_in}</td><td>${dc.bus_out}</td><td>${dc.control_mode}</td><td>${dc.p_ref_mw}</td>`);

    // Energy Router
    fillTable('#erTableInner', 'erSection', sys.energy_routers, m.energyRouter, er =>
      `<td>${er.router_type}</td><td>${er.num_ports}</td><td>${er.p_rated_mw}</td>`);

    // VPP
    fillTable('#vppTableInner', 'vppSection', sys.vpps, m.vpp, v =>
      `<td>${v.pcc_bus ?? v.aggregation_bus ?? 0}</td><td>${v.n_pv_systems}</td><td>${v.n_wind_turbines}</td><td>${v.n_battery_systems}</td><td>${v.p_output_mw}</td>`);

    // Microgrid
    fillTable('#mgTableInner', 'mgSection', sys.microgrids, m.microgrid, mg =>
      `<td>${mg.pcc_bus}</td><td>${mg.operating_mode}</td><td>${mg.p_exchange_mw}</td><td>${mg.capacity_mw}</td>`);
  }

  function dynamicModelTextValue(value) {
    if (!value || typeof value !== 'object' || Array.isArray(value)) {
      return '';
    }
    return JSON.stringify(value, null, 2);
  }

  // ========== Property Editor ==========
  function onSelectionChanged(compId) {
    const propEmpty = document.getElementById('propEmpty');
    const propEditor = document.getElementById('propEditor');

    if (compId === null) {
      propEmpty.style.display = 'block';
      propEditor.style.display = 'none';
      return;
    }

    const comp = Canvas.getComponent(compId);
    if (!comp) return;

    propEmpty.style.display = 'none';
    propEditor.style.display = 'block';

    document.getElementById('propTitle').textContent =
      `${COMP.defaults[comp.type]?.name || comp.type} — ID:${comp.id}`;

    const fieldsDiv = document.getElementById('propFields');
    fieldsDiv.innerHTML = '';

    const defaults = COMP.defaults[comp.type] || {};
    // Optional section dividers: when a field key matches, a header row is
    // inserted before it to group the OPF constraint-limit fields visually.
    const sectionHeaders = {
      generator:      { dynamic_model: '暂态模型参数' },
      load:           { dynamic_model: '暂态模型参数' },
      external_grid:  { dynamic_model: '暂态模型参数' },
      storage:        { dynamic_model: '暂态模型参数' },
      pv_system:      { dynamic_model: '暂态模型参数' },
      static_generator: { dynamic_model: '暂态模型参数' },
      vsc_converter:  { r_conv_ac_pu: '约束限值 (OPF)', grid_forming: '构网与协调', dynamic_model: '暂态模型参数' },
      dc_load:        { dynamic_model: '暂态模型参数' },
      dc_storage:     { dynamic_model: '暂态模型参数' },
      dc_pv_array:    { dynamic_model: '暂态模型参数' },
      asymmetric_load: { dynamic_model: '暂态模型参数' },
      dcdc_converter: { topology: '占空比约束 (OPF)' },
    };
    const secMap = sectionHeaders[comp.type] || null;
    Object.keys(defaults).forEach(key => {
      if (secMap && secMap[key]) {
        const hd = document.createElement('div');
        hd.className = 'prop-section-header';
        hd.textContent = secMap[key];
        fieldsDiv.appendChild(hd);
      }
      const rawVal = comp.params[key] !== undefined ? comp.params[key] : defaults[key];
      const val = key === 'emission_factor_tco2_mwh'
        ? carbonFactorDisplay(rawVal)
        : rawVal;
      const label = COMP.fieldLabels[key] || key;

      const div = document.createElement('div');
      div.className = 'prop-field';

      const lbl = document.createElement('label');
      lbl.textContent = label;
      div.appendChild(lbl);

      const isJsonArrayField = Array.isArray(val) || /^_ies_.*profile.*values$/.test(key);
      if (key === 'dynamic_model' || isJsonArrayField) {
        const txt = document.createElement('textarea');
        txt.dataset.field = key;
        txt.className = 'prop-json-textarea';
        txt.spellcheck = false;
        txt.value = key === 'dynamic_model'
          ? dynamicModelTextValue(val)
          : JSON.stringify(Array.isArray(val) ? val : [], null, 2);
        txt.placeholder = key === 'dynamic_model'
          ? '{"standard":"PSS/E","model_name":"GENROU","parameters":{}}'
          : '[8, 10, 9, 7]';
        txt.title = key === 'dynamic_model'
          ? '统一动态模型 JSON：standard/model_name/parameter_set/parameters/components。会随系统 JSON 同步并用于暂态模型构建。'
          : '导入或手工编辑的时序数组，长度应与园区综合能源仿真步数一致。';
        div.classList.add('prop-field-wide');
        div.appendChild(txt);
      } else if (typeof val === 'boolean') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        sel.innerHTML = `<option value="true" ${val ? 'selected' : ''}>是</option>
                         <option value="false" ${!val ? 'selected' : ''}>否</option>`;
        div.appendChild(sel);
      } else if (key === 'bus_type') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        // DC buses have their own type set (DC_P / DC_V); AC buses use PQ/PV/SLACK/ISOLATED.
        const busTypeOptions = comp.type === 'dc_bus'
          ? ['DC_P', 'DC_V']
          : ['PQ', 'PV', 'SLACK', 'ISOLATED'];
        busTypeOptions.forEach(t => {
          sel.innerHTML += `<option value="${t}" ${val === t ? 'selected' : ''}>${t}</option>`;
        });
        div.appendChild(sel);
      } else if (key === 'fuel_type') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        ['Thermal', 'Nuclear', 'Hydro', 'Gas', 'Oil', 'Coal', 'Biomass', 'Other'].forEach(t => {
          sel.innerHTML += `<option value="${t}" ${val === t ? 'selected' : ''}>${t}</option>`;
        });
        div.appendChild(sel);
      } else if (key === 'model') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        ['ConstantPower', 'ConstantImpedance', 'ZIP'].forEach(t => {
          sel.innerHTML += `<option value="${t}" ${val === t ? 'selected' : ''}>${t}</option>`;
        });
        div.appendChild(sel);
      } else if (key === 'topology') {
        // DC/DC power-stage topology selects the duty-ratio feasibility model.
        const sel = document.createElement('select');
        sel.dataset.field = key;
        [
          {v:'Generic',   l:'Generic (无占空比约束)'},
          {v:'Buck',      l:'Buck (降压)'},
          {v:'Boost',     l:'Boost (升压)'},
          {v:'BuckBoost', l:'Buck-Boost (升降压)'},
          {v:'Isolated',  l:'Isolated (隔离/DAB)'},
        ].forEach(o => {
          sel.innerHTML += `<option value="${o.v}" ${val === o.v ? 'selected' : ''}>${o.l}</option>`;
        });
        div.appendChild(sel);
      } else if (/^port\d+_type$/.test(key)) {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        ['AC', 'DC'].forEach(t => {
          sel.innerHTML += `<option value="${t}" ${val === t ? 'selected' : ''}>${t}</option>`;
        });
        div.appendChild(sel);
      } else if (/^port\d+_control_mode$/.test(key)) {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        [
          {v:'VF',    l:'VF'},
          {v:'PQ',    l:'PQ'},
          {v:'Droop', l:'Droop'},
        ].forEach(o => {
          sel.innerHTML += `<option value="${o.v}" ${val === o.v ? 'selected' : ''}>${o.l}</option>`;
        });
        div.appendChild(sel);
      } else if (key === 'control_mode') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        let modeOptions;
        if (comp.type === 'pv_system') {
          modeOptions = [
            {v:'MPPT',   l:'MPPT (最大功率追踪)'},
            {v:'PQ',     l:'PQ (恒功率)'},
            {v:'VQ',     l:'VQ (电压无功)'},
            {v:'Curtailed', l:'Curtailed (自适应削减)'},
          ];
        } else if (comp.type === 'dcdc_converter') {
          modeOptions = [
            {v:'Voltage', l:'Voltage'},
            {v:'Power',   l:'Power'},
            {v:'Droop',   l:'Droop'},
          ];
        } else {
          modeOptions = [
            {v:'PQ_MODE',         l:'PQ_MODE (模式3 · AC定P+定Q)'},
            {v:'AC_PV',           l:'AC_PV (模式2 · AC定P+定电压)'},
            {v:'VDC_Q',           l:'VDC_Q (模式4/7 · 定/下垂Udc+定Q)'},
            {v:'VDC_VAC',         l:'VDC_VAC (模式5 · 定Udc+定电压)'},
            {v:'DC_V_DROOP_AC_V', l:'DC_V_DROOP_AC_V (模式6 · 下垂Udc+定电压)'},
            {v:'AC_GRID_FORMING', l:'AC_GRID_FORMING (模式1 · AC构网 δs+Vs)'},
          ];
        }
        modeOptions.forEach(o => {
          sel.innerHTML += `<option value="${o.v}" ${val === o.v ? 'selected' : ''}>${o.l}</option>`;
        });
        div.appendChild(sel);
      } else {
        const inp = document.createElement('input');
        inp.dataset.field = key;
        inp.type = typeof val === 'number' ? 'number' : 'text';
        inp.step = 'any';
        inp.value = val;
        // Bus terminals are resolved from the canvas wiring (connected ports),
        // not typed by hand. Show them read-only so the user wires ports — e.g.
        // a DC/DC's 'in'/'out' — instead of guessing internal bus IDs.
        if (WIRING_DERIVED_BUS_FIELDS.has(key)) {
          inp.readOnly = true;
          inp.title = '由接线自动确定（请通过连线修改端口）';
          inp.style.opacity = '0.65';
          inp.style.cursor = 'not-allowed';
        }
        div.appendChild(inp);
      }

      fieldsDiv.appendChild(div);
    });
  }

  function applyProperties() {
    const compId = Canvas.state.selectedId;
    if (compId === null) return;
    const comp = Canvas.getComponent(compId);
    if (!comp) return;

    // Track which fields changed for per-km ↔ per-unit sync
    const oldParams = Object.assign({}, comp.params);
    const fields = document.querySelectorAll('#propFields [data-field]');
    const changedKeys = new Set();
    const updates = [];
    let parseError = null;
    fields.forEach(el => {
      if (parseError) return;
      const key = el.dataset.field;
      let val = el.value;
      // Type conversion
      if (key === 'dynamic_model') {
        const raw = String(val || '').trim();
        if (!raw) {
          val = {};
        } else {
          try {
            val = JSON.parse(raw);
            if (!val || typeof val !== 'object' || Array.isArray(val)) {
              throw new Error('dynamic_model must be a JSON object');
            }
          } catch (err) {
            parseError = err;
            return;
          }
        }
      } else if (/^_ies_.*profile.*values$/.test(key)) {
        const raw = String(val || '').trim();
        if (!raw) {
          val = [];
        } else {
          try {
            const parsed = JSON.parse(raw);
            if (!Array.isArray(parsed)) throw new Error('profile values must be a JSON array');
            val = parsed.map(Number).filter(Number.isFinite);
          } catch (err) {
            parseError = err;
            return;
          }
        }
      } else if (val === 'true') val = true;
      else if (val === 'false') val = false;
      else if (el.type === 'number' && val !== '') val = parseFloat(val);
      if (key === 'emission_factor_tco2_mwh') val = carbonFactorInputValue(val);
      updates.push({ key, val });
    });
    if (parseError) {
      const msg = `JSON 字段无效: ${parseError.message || parseError}`;
      setStatus(msg, 'error');
      log(msg, 'error');
      return;
    }
    updates.forEach(({ key, val }) => {
      const oldVal = comp.params[key];
      const sameNumber = typeof oldVal === 'number' && typeof val === 'number' &&
        Math.abs(oldVal - val) < 1e-12;
      const sameJson = (key === 'dynamic_model' || /^_ies_.*profile.*values$/.test(key)) &&
        JSON.stringify(oldVal || {}) === JSON.stringify(val || {});
      if (!sameNumber && !sameJson && oldVal !== val) changedKeys.add(key);
      comp.params[key] = val;
    });

    // Bidirectional sync for ac_branch: per-km ↔ per-unit parameters
    if (comp.type === 'ac_branch') {
      syncBranchImpedance(comp, changedKeys, oldParams);
    }

    // Re-render component with new params
    Canvas.rerenderComponent(comp);
    // Only mark dirty if something actually changed
    const onlyCarbonFactorChanged =
      changedKeys.size === 1 && changedKeys.has('emission_factor_tco2_mwh') &&
      (comp.type === 'generator' || comp.type === 'external_grid' || comp.type === 'static_generator');
    if (changedKeys.size > 0 && !onlyCarbonFactorChanged) {
      _canvasDirty = true;
    }
    updateTopologyTables();
    if (onlyCarbonFactorChanged) {
      syncCarbonFactorsOnly();
    }
    // Refresh property panel to show synced values
    if (changedKeys.size > 0 && comp.type === 'ac_branch') {
      onSelectionChanged(compId);
    }
    if (changedKeys.size > 0) {
      log(`已更新 ${comp.params.name} 的属性 (${[...changedKeys].join(', ')})`, 'info');
    } else {
      log(`${comp.params.name} 属性未变化`, 'info');
    }
  }

  /**
   * Synchronize per-km and per-unit impedance parameters for ac_branch.
   *
   * Conversion formulas (using from_bus base_kv and system base_mva):
   *   z_base = base_kv² / base_mva
   *   r_pu = r_ohm_per_km * length_km / z_base
   *   x_pu = x_ohm_per_km * length_km / z_base
   *   b_pu = b_us_per_km  * length_km * z_base * 1e-6
   *
   * Direction of sync depends on which parameter was modified:
   *   - Changed per-km or length → recalculate per-unit
   *   - Changed per-unit → recalculate per-km
   */
  function syncBranchImpedance(comp, changedKeys, oldParams) {
    const p = comp.params;
    const perKmChanged = changedKeys.has('r_ohm_per_km') || changedKeys.has('x_ohm_per_km')
                       || changedKeys.has('b_us_per_km') || changedKeys.has('c_nf_per_km');
    const puChanged = changedKeys.has('r_pu') || changedKeys.has('x_pu') || changedKeys.has('b_pu');
    const lengthChanged = changedKeys.has('length_km');

    if (!perKmChanged && !puChanged && !lengthChanged) return;

    // Find from_bus base_kv
    const fromBus = p.from_bus || 0;
    let base_kv = 110;  // default
    const base_mva = 100;  // system default
    if (fromBus > 0) {
      const busComp = Canvas.state.components.find(
        c => c.type === 'ac_bus' && c.params.index === fromBus
      );
      if (busComp && busComp.params.base_kv > 0) base_kv = busComp.params.base_kv;
    }
    const z_base = (base_kv * base_kv) / base_mva;
    const length = p.length_km || 0;

    if (perKmChanged || lengthChanged) {
      // Per-km → per-unit
      if (length > 0 && z_base > 0) {
        // If c_nf_per_km changed, derive b_us_per_km from it
        if (changedKeys.has('c_nf_per_km') && p.c_nf_per_km > 0) {
          const freq = 50;  // Hz
          p.b_us_per_km = 2 * Math.PI * freq * p.c_nf_per_km * 1e-3;  // nF→μS
        }
        if (p.r_ohm_per_km >= 0) p.r_pu = p.r_ohm_per_km * length / z_base;
        if (p.x_ohm_per_km >= 0) p.x_pu = p.x_ohm_per_km * length / z_base;
        if (p.b_us_per_km >= 0)  p.b_pu = p.b_us_per_km * length * z_base * 1e-6;
      }
    } else if (puChanged) {
      // Per-unit → per-km
      if (length > 0 && z_base > 0) {
        if (changedKeys.has('r_pu')) p.r_ohm_per_km = p.r_pu * z_base / length;
        if (changedKeys.has('x_pu')) p.x_ohm_per_km = p.x_pu * z_base / length;
        if (changedKeys.has('b_pu')) p.b_us_per_km  = p.b_pu / (length * z_base * 1e-6);
      }
    }
  }

  // ========== Tab Switching ==========
  function switchTab(tabName) {
    document.querySelectorAll('.panel-tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
    document.querySelector(`.panel-tab[data-tab="${tabName}"]`)?.classList.add('active');
    document.getElementById('tab' + tabName.charAt(0).toUpperCase() + tabName.slice(1))?.classList.add('active');
    if (tabName === 'results' && document.getElementById('resultsContent')?.dataset.activeGroup === 'transient' && _lastTransientData) {
      requestAnimationFrame(() => drawTransientDashboard(_lastTransientData));
    }
  }

  // ========== Component Library ==========
  function initComponentLibrary() {
    Object.entries(COMP.categories).forEach(([catId, items]) => {
      const grid = document.getElementById(catId);
      if (!grid) return;
      items.forEach(item => {
        const div = document.createElement('div');
        div.className = 'lib-item';
        div.dataset.compType = item.type;

        // Create a small SVG icon
        const iconSvg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        iconSvg.setAttribute('viewBox', '-30 -30 60 60');
        iconSvg.setAttribute('width', '36');
        iconSvg.setAttribute('height', '36');
        const symbolFn = COMP.symbols[item.type];
        if (symbolFn) {
          const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
          g.setAttribute('transform', 'scale(0.6)');
          g.innerHTML = symbolFn(COMP.defaults[item.type] || {});
          // Remove labels from icon
          g.querySelectorAll('.comp-label, .comp-value').forEach(t => t.remove());
          iconSvg.appendChild(g);
        }
        div.appendChild(iconSvg);

        const span = document.createElement('span');
        span.textContent = item.label;
        div.appendChild(span);

        // Click to enter place mode
        div.addEventListener('click', () => {
          document.querySelectorAll('.lib-item').forEach(i => i.classList.remove('selected'));
          div.classList.add('selected');
          Canvas.setMode('place', item.type);
          log(`选择放置: ${item.label}`, 'info');
        });

        grid.appendChild(div);
      });
    });
  }

  // ========== Toolbar UI (3-tier layout) ==========
  function showCaseLoadModal() {
    const m = document.getElementById('caseLoadModal');
    if (m) m.style.display = 'flex';
  }
  function hideCaseLoadModal() {
    const m = document.getElementById('caseLoadModal');
    if (m) m.style.display = 'none';
  }
  function setActiveCanvasTool(toolName) {
    // Tools that live in bar 1 and act as exclusive canvas modes.
    const ids = ['btnSelect', 'btnConnect', 'btnBoxSelect'];
    ids.forEach(id => {
      const el = document.getElementById(id);
      if (el) el.classList.toggle('active', id === toolName);
    });
  }
  function setActiveModule(moduleName) {
    const prev = document.querySelector('.module-btn.active');
    const changed = !prev || prev.dataset.module !== moduleName;
    document.querySelectorAll('.module-btn').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.module === moduleName);
    });
    renderSubToolbar(moduleName);
    // Also switch the right-panel result view to the matching module's
    // results group (if any), so the 结果 tab only shows the active
    // module's outputs / placeholder. CSS in style.css drives visibility.
    setActiveResultGroup(moduleName);
    // The shared summary banner is written by power-flow / short-circuit /
    // harmonics. Clear it when actually switching modules so a previous
    // module's summary does not linger over the new module's result view (each
    // module repopulates it, or uses its own in-group summary, when it runs).
    if (changed) {
      const shared = document.getElementById('resultsSummary');
      if (shared) shared.innerHTML = '';
    }
  }
  function renderSubToolbar(moduleName) {
    const bar = document.getElementById('subToolbar');
    if (!bar) return;
    let anyVisible = false;
    bar.querySelectorAll('.sub-section').forEach(sec => {
      const match = sec.dataset.sub === moduleName;
      sec.hidden = !match;
      if (match) anyVisible = true;
    });
    bar.classList.toggle('hidden', !anyVisible);
    if (moduleName === 'resilience') markResilienceFaultBranches();
  }

  function iesNumber(id, fallback) {
    const v = Number(document.getElementById(id)?.value);
    return Number.isFinite(v) ? v : fallback;
  }

  function iesNum(value, fallback = 0) {
    if (value === null || value === undefined || value === '') return fallback;
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  }

  function iesParam(params, key, fallback = 0) {
    return iesNum(params?.[key], fallback);
  }

  function iesActiveCanvasComponents() {
    if (typeof Canvas === 'undefined' || !Array.isArray(Canvas.state?.components)) return [];
    return Canvas.state.components.filter(comp =>
      String(comp.type || '').startsWith('ies_') && comp.params?.in_service !== false);
  }

  function iesComponentsOf(components, type) {
    return components.filter(comp => comp.type === type);
  }

  function iesSum(components, type, key) {
    return iesComponentsOf(components, type)
      .reduce((sum, comp) => sum + iesParam(comp.params, key, 0), 0);
  }

  function iesWeightedAverage(components, valueKey, weightKey, fallback) {
    let wsum = 0;
    let vsum = 0;
    components.forEach(comp => {
      const value = iesParam(comp.params, valueKey, NaN);
      if (!Number.isFinite(value)) return;
      const rawWeight = iesParam(comp.params, weightKey, 0);
      const weight = rawWeight > 0 ? rawWeight : 1;
      wsum += weight;
      vsum += value * weight;
    });
    return wsum > 0 ? vsum / wsum : fallback;
  }

  function iesLegacyElectricLoadMw() {
    if (typeof Canvas === 'undefined' || !Array.isArray(Canvas.state?.components)) return 0;
    return Canvas.state.components.reduce((sum, comp) => {
      const p = comp.params || {};
      if (p.in_service === false) return sum;
      if (comp.type === 'load') return sum + iesParam(p, 'p_mw', 0) * iesParam(p, 'scaling', 1);
      if (comp.type === 'dc_load') return sum + iesParam(p, 'p_mw', 0) * iesParam(p, 'scaling', 1);
      if (comp.type === 'charging_station') {
        const directMw = iesParam(p, 'p_total_kw', 0) / 1000;
        const maxMw = iesParam(p, 'max_power_kw', 0) * iesParam(p, 'utilization_rate', 0.3) / 1000;
        return sum + (directMw > 0 ? directMw : maxMw);
      }
      return sum;
    }, 0);
  }

  function iesLegacySolarRatedMw() {
    if (typeof Canvas === 'undefined' || !Array.isArray(Canvas.state?.components)) return 0;
    return Canvas.state.components.reduce((sum, comp) => {
      const p = comp.params || {};
      if (p.in_service === false) return sum;
      if (comp.type === 'pv_system') return sum + (iesParam(p, 'p_rated_mw', 0) || iesParam(p, 'p_mw', 0));
      if (comp.type === 'dc_pv_array') return sum + iesParam(p, 'p_set_mw', 0);
      if (comp.type === 'static_generator' && String(p.sgen_type || '').toLowerCase().includes('pv')) {
        return sum + (iesParam(p, 'p_rated_mw', 0) || iesParam(p, 'p_mw', 0));
      }
      return sum;
    }, 0);
  }

  function iesLegacyWindRatedMw() {
    if (typeof Canvas === 'undefined' || !Array.isArray(Canvas.state?.components)) return 0;
    return Canvas.state.components.reduce((sum, comp) => {
      const p = comp.params || {};
      if (p.in_service === false || comp.type !== 'renewable_gen') return sum;
      const kind = String(p.type || p.name || '').toLowerCase();
      return kind.includes('wind') || kind.includes('风') ? sum + iesParam(p, 'p_rated_mw', iesParam(p, 'p_mw', 0)) : sum;
    }, 0);
  }

  function iesLegacyElectricStorage() {
    const out = { count: 0, energy: 0, initial: 0, charge: 0, discharge: 0 };
    if (typeof Canvas === 'undefined' || !Array.isArray(Canvas.state?.components)) return out;
    Canvas.state.components.forEach(comp => {
      if (comp.type !== 'storage' && comp.type !== 'dc_storage') return;
      const p = comp.params || {};
      if (p.in_service === false) return;
      const e = iesParam(p, 'e_rated_mwh', 0);
      out.count += 1;
      out.energy += e;
      out.initial += e * iesParam(p, 'soc_init', 0.5);
      out.charge += Math.max(iesParam(p, 'pmax_mw', 0), iesParam(p, 'p_rated_mw', 0));
      out.discharge += Math.max(Math.abs(iesParam(p, 'pmin_mw', 0)), iesParam(p, 'p_rated_mw', 0));
    });
    return out;
  }

  function iesApplyStoragePayload(payload, rows, prefix) {
    payload[`${prefix}_storage_capacity_mwh`] = rows.reduce((s, c) => s + iesParam(c.params, 'capacity_mwh', 0), 0);
    payload[`${prefix}_storage_initial_mwh`] = rows.reduce((s, c) => s + iesParam(c.params, 'initial_mwh', 0), 0);
    payload[`${prefix}_storage_charge_max_mw`] = rows.reduce((s, c) => s + iesParam(c.params, 'charge_max_mw', 0), 0);
    payload[`${prefix}_storage_discharge_max_mw`] = rows.reduce((s, c) => s + iesParam(c.params, 'discharge_max_mw', 0), 0);
  }

  const IES_PROFILE_KEYS = [
    'electric_load_mw', 'heat_load_mw', 'hydrogen_load_mw', 'fuel_load_mw',
    'transport_demand_km', 'solar_available_mw', 'wind_available_mw',
    'grid_buy_price', 'grid_sell_price', 'grid_carbon_tco2_mwh',
  ];

  const IES_PROFILE_ALIASES = {
    electric_load_mw: ['electric_load_mw', 'electric_load', 'electricity_load', 'load_electric', 'power_load', '电负荷', '电力负荷', '用电负荷'],
    heat_load_mw: ['heat_load_mw', 'thermal_load_mw', 'heat_load', 'thermal_load', '热负荷', '供热负荷'],
    hydrogen_load_mw: ['hydrogen_load_mw', 'h2_load_mw', 'hydrogen_load', 'h2_load', '氢负荷', '氢气负荷'],
    fuel_load_mw: ['fuel_load_mw', 'gas_load_mw', 'fuel_load', 'gas_load', '燃料负荷', '燃气负荷'],
    transport_demand_km: ['transport_demand_km', 'transport_km', 'traffic_demand_km', '交通需求', '出行需求'],
    solar_available_mw: ['solar_available_mw', 'pv_available_mw', 'solar_mw', 'pv_mw', 'solar', 'pv', '光伏', '光伏可用', '光伏出力'],
    wind_available_mw: ['wind_available_mw', 'wind_mw', 'wind', '风电', '风电可用', '风电出力'],
    grid_buy_price: ['grid_buy_price', 'buy_price', 'purchase_price', 'import_price', '购电价格', '购电价', '电价'],
    grid_sell_price: ['grid_sell_price', 'sell_price', 'export_price', '售电价格', '售电价', '上网电价'],
    grid_carbon_tco2_mwh: ['grid_carbon_tco2_mwh', 'grid_carbon', 'carbon_factor', 'co2_factor', '电网碳因子', '碳因子'],
  };

  const IES_PROFILE_ALIAS_MAP = (() => {
    const out = {};
    Object.entries(IES_PROFILE_ALIASES).forEach(([key, aliases]) => {
      aliases.concat(key).forEach(alias => { out[iesProfileNameKey(alias)] = key; });
    });
    return out;
  })();

  function iesProfileNameKey(raw) {
    return String(raw || '')
      .trim()
      .toLowerCase()
      .replace(/[：:;,，、|\/\\(){}<>_\-\s]/g, '')
      .replace(/[\[\]]/g, '')
      .replace(/（|）/g, '');
  }

  function iesCanonicalProfileKey(raw) {
    const exact = String(raw || '').trim();
    if (IES_PROFILE_KEYS.includes(exact)) return exact;
    return IES_PROFILE_ALIAS_MAP[iesProfileNameKey(exact)] || null;
  }

  function iesNumericArray(values) {
    if (!Array.isArray(values)) return [];
    return values.map(Number).filter(Number.isFinite);
  }

  function iesAverage(values, fallback = 0) {
    const arr = iesNumericArray(values);
    if (!arr.length) return fallback;
    return arr.reduce((sum, v) => sum + v, 0) / arr.length;
  }

  function iesMax(values, fallback = 0) {
    const arr = iesNumericArray(values);
    return arr.length ? Math.max(...arr) : fallback;
  }

  function iesProfileArray(params, key = '_ies_profile_values') {
    const raw = params?.[key];
    if (Array.isArray(raw)) return iesNumericArray(raw);
    if (typeof raw === 'string' && raw.trim()) {
      try { return iesNumericArray(JSON.parse(raw)); } catch (_) { return []; }
    }
    return [];
  }

  function iesProfileSeriesValue(values, idx, fallback = 0) {
    if (!values.length) return fallback;
    const raw = values[Math.min(idx, values.length - 1)];
    return Number.isFinite(raw) ? raw : fallback;
  }

  function iesAggregateComponentProfiles(rows, fallbackFn, profileKey = '_ies_profile_values') {
    const series = rows.map(comp => iesProfileArray(comp.params, profileKey));
    const n = series.reduce((maxLen, arr) => Math.max(maxLen, arr.length), 0);
    if (n <= 0) return null;
    return Array.from({ length: n }, (_, t) =>
      rows.reduce((sum, comp, i) => {
        const fallback = Number(fallbackFn(comp)) || 0;
        return sum + iesProfileSeriesValue(series[i], t, fallback);
      }, 0));
  }

  function iesWeightedAverageProfile(rows, valueKey, weightKey, profileKey, fallback) {
    const series = rows.map(comp => iesProfileArray(comp.params, profileKey));
    const n = series.reduce((maxLen, arr) => Math.max(maxLen, arr.length), 0);
    if (n <= 0) return null;
    return Array.from({ length: n }, (_, t) => {
      let weighted = 0;
      let weightSum = 0;
      rows.forEach((comp, i) => {
        const weightRaw = iesParam(comp.params, weightKey, 0);
        const weight = weightRaw > 0 ? weightRaw : 1;
        const base = iesParam(comp.params, valueKey, fallback);
        const value = iesProfileSeriesValue(series[i], t, base);
        weighted += value * weight;
        weightSum += weight;
      });
      return weightSum > 0 ? weighted / weightSum : fallback;
    });
  }

  function iesCanvasProfileLength(components) {
    const profileKeys = [
      '_ies_profile_values',
      '_ies_buy_price_profile_values',
      '_ies_sell_price_profile_values',
      '_ies_carbon_profile_values',
    ];
    return components.reduce((maxLen, comp) => {
      profileKeys.forEach(key => { maxLen = Math.max(maxLen, iesProfileArray(comp.params, key).length); });
      return maxLen;
    }, 0);
  }

  function iesCanvasProfileStepDuration(components, fallback = 1.0) {
    for (const comp of components) {
      const step = iesParam(comp.params, '_ies_profile_step_duration_hr', NaN);
      if (Number.isFinite(step) && step > 0) return step;
    }
    return fallback;
  }

  function iesSetProfile(comp, name, values, stepDuration, scalarKey, scalarMode = 'average') {
    const arr = iesNumericArray(values);
    if (!comp?.params || !arr.length) return false;
    comp.params._ies_profile_name = name || comp.params.name || '';
    comp.params._ies_profile_values = arr;
    comp.params._ies_profile_step_duration_hr = stepDuration || 1.0;
    if (scalarKey) {
      const scalar = scalarMode === 'max' ? iesMax(arr, iesParam(comp.params, scalarKey, 0)) : iesAverage(arr, iesParam(comp.params, scalarKey, 0));
      comp.params[scalarKey] = scalar;
    }
    return true;
  }

  function iesSetGridProfile(comp, key, values, stepDuration) {
    const arr = iesNumericArray(values);
    if (!comp?.params || !arr.length) return false;
    const profileField = key === 'grid_buy_price' ? '_ies_buy_price_profile_values'
      : key === 'grid_sell_price' ? '_ies_sell_price_profile_values'
      : '_ies_carbon_profile_values';
    const scalarField = key === 'grid_buy_price' ? 'buy_price_per_mwh'
      : key === 'grid_sell_price' ? 'sell_price_per_mwh'
      : 'carbon_tco2_mwh';
    comp.params[profileField] = arr;
    comp.params._ies_profile_step_duration_hr = stepDuration || 1.0;
    comp.params[scalarField] = iesAverage(arr, iesParam(comp.params, scalarField, 0));
    return true;
  }

  function iesDistributeProfile(rows, profile, weightFn, scalarKey, scalarMode = 'average') {
    if (!rows.length || !profile?.values?.length) return 0;
    const weights = rows.map(comp => Math.max(0, Number(weightFn(comp)) || 0));
    const total = weights.reduce((sum, v) => sum + v, 0);
    let applied = 0;
    rows.forEach((comp, i) => {
      const share = total > 1e-12 ? weights[i] / total : 1 / rows.length;
      const values = profile.values.map(v => v * share);
      if (iesSetProfile(comp, profile.name, values, profile.step_duration_hr, scalarKey, scalarMode)) applied += 1;
    });
    return applied;
  }

  function iesApplyNamedProfileToComponent(comp, profile) {
    if (!comp || !profile?.values?.length) return false;
    switch (comp.type) {
      case 'ies_electric_load':
      case 'ies_heat_load':
      case 'ies_hydrogen_load':
      case 'ies_fuel_load':
        return iesSetProfile(comp, profile.name, profile.values, profile.step_duration_hr, 'demand_mw');
      case 'ies_transport':
        return iesSetProfile(comp, profile.name, profile.values, profile.step_duration_hr, 'demand_km_per_h');
      case 'ies_solar':
      case 'ies_wind':
        return iesSetProfile(comp, profile.name, profile.values, profile.step_duration_hr, 'rated_mw', 'max');
      default:
        return false;
    }
  }

  function iesReadBrowserFileText(file) {
    return new Promise((resolve, reject) => {
      const fr = new FileReader();
      fr.onload = () => resolve(String(fr.result || ''));
      fr.onerror = () => reject(fr.error || new Error('file read failed'));
      fr.readAsText(file, 'utf-8');
    });
  }

  function iesPushImportedProfile(profiles, name, values, key, stepDuration) {
    const arr = iesNumericArray(values);
    if (!arr.length) return;
    const canonical = key || iesCanonicalProfileKey(name);
    profiles.push({
      name: String(name || canonical || `profile_${profiles.length + 1}`),
      key: canonical,
      values: arr,
      step_duration_hr: stepDuration || 1.0,
    });
  }

  function parseIntegratedEnergyJsonProfiles(text) {
    const j = JSON.parse(text);
    const stepDuration = Number(j.step_duration_hr || j.time_series?.step_duration_hr || j._time_series?.step_duration_hr || 1.0) || 1.0;
    const profiles = [];
    IES_PROFILE_KEYS.forEach(key => iesPushImportedProfile(profiles, key, j[key], key, stepDuration));
    const consumeContainer = container => {
      if (!container) return;
      if (Array.isArray(container)) {
        container.forEach((p, i) => {
          const values = p?.values ?? p?.p_mw_values ?? p?.data ?? p?.profile;
          const name = p?.name ?? p?.field ?? p?.key ?? p?.id ?? `profile_${i + 1}`;
          const key = p?.field || p?.key || iesCanonicalProfileKey(name);
          iesPushImportedProfile(profiles, name, values, key, stepDuration);
        });
      } else if (typeof container === 'object') {
        Object.entries(container).forEach(([name, values]) => {
          iesPushImportedProfile(profiles, name, values, iesCanonicalProfileKey(name), stepDuration);
        });
      }
    };
    consumeContainer(j.profiles);
    consumeContainer(j.load_profiles);
    consumeContainer(j.time_series?.profiles);
    consumeContainer(j._time_series?.profiles);
    const detectedSteps = profiles.reduce((maxLen, p) => Math.max(maxLen, p.values.length), 0);
    const numSteps = Math.round(Number(j.num_steps || j.time_series?.num_steps || j._time_series?.num_steps || detectedSteps));
    if (!profiles.length || !Number.isInteger(numSteps) || numSteps <= 0) {
      throw new Error('未找到有效的综合能源时序数组');
    }
    const bad = profiles.filter(p => p.values.length !== numSteps);
    if (bad.length) {
      throw new Error(`${bad.length} 条时序长度与 num_steps=${numSteps} 不一致`);
    }
    return { num_steps: numSteps, step_duration_hr: stepDuration, profiles };
  }

  function iesSplitDelimitedLine(line, delimiter) {
    const out = [];
    let cur = '';
    let quoted = false;
    for (let i = 0; i < line.length; i++) {
      const ch = line[i];
      if (ch === '"') {
        if (quoted && line[i + 1] === '"') { cur += '"'; i++; }
        else quoted = !quoted;
      } else if (ch === delimiter && !quoted) {
        out.push(cur.trim().replace(/^"|"$/g, ''));
        cur = '';
      } else {
        cur += ch;
      }
    }
    out.push(cur.trim().replace(/^"|"$/g, ''));
    return out;
  }

  function parseIntegratedEnergyCsvProfiles(text) {
    const lines = text.split(/\r?\n/).map(line => line.trim()).filter(Boolean);
    if (lines.length < 2) throw new Error('CSV 至少需要表头和一行数据');
    const first = lines[0];
    const delimiter = first.includes('\t') ? '\t'
      : ((first.match(/;/g) || []).length > (first.match(/,/g) || []).length ? ';' : ',');
    const rows = lines.map(line => iesSplitDelimitedLine(line, delimiter));
    const headers = rows[0];
    const skipHeaders = new Set(['time', 'timestamp', 'datetime', 'date', 'hour', 'step', 't', '时间', '时刻', '步']);
    const profiles = [];
    headers.forEach((header, c) => {
      if (!header || skipHeaders.has(iesProfileNameKey(header))) return;
      const values = rows.slice(1).map(row => {
        const n = Number.parseFloat(row[c]);
        return Number.isFinite(n) ? n : 0;
      });
      if (values.length && values.some(v => Number.isFinite(v))) {
        iesPushImportedProfile(profiles, header, values, iesCanonicalProfileKey(header), 1.0);
      }
    });
    if (!profiles.length) throw new Error('CSV 未识别到任何数值时序列');
    return { num_steps: profiles[0].values.length, step_duration_hr: 1.0, profiles };
  }

  function parseIntegratedEnergyProfiles(fileName, text) {
    return /\.json$/i.test(fileName)
      ? parseIntegratedEnergyJsonProfiles(text)
      : parseIntegratedEnergyCsvProfiles(text);
  }

  function applyIntegratedEnergyImportedProfiles(bundle, fileName) {
    const components = iesActiveCanvasComponents();
    if (!components.length) {
      log('综合能源时序导入：画布中没有投运的综合能源元件，请先添加或生成示例画布', 'warn');
      return;
    }

    const profilesByKey = new Map();
    bundle.profiles.forEach(profile => {
      const key = profile.key || iesCanonicalProfileKey(profile.name);
      if (key) profilesByKey.set(key, { ...profile, key, step_duration_hr: bundle.step_duration_hr });
    });

    let applied = 0;
    const appliedNotes = [];
    const distribute = (key, type, scalarKey, weightFn, scalarMode = 'average') => {
      const profile = profilesByKey.get(key);
      const rows = iesComponentsOf(components, type);
      if (!profile || !rows.length) return;
      const count = iesDistributeProfile(rows, profile, weightFn, scalarKey, scalarMode);
      if (count > 0) {
        applied += count;
        appliedNotes.push(`${key}→${count}`);
      }
    };

    distribute('electric_load_mw', 'ies_electric_load', 'demand_mw', comp => iesParam(comp.params, 'demand_mw', 0));
    distribute('heat_load_mw', 'ies_heat_load', 'demand_mw', comp => iesParam(comp.params, 'demand_mw', 0));
    distribute('hydrogen_load_mw', 'ies_hydrogen_load', 'demand_mw', comp => iesParam(comp.params, 'demand_mw', 0));
    distribute('fuel_load_mw', 'ies_fuel_load', 'demand_mw', comp => iesParam(comp.params, 'demand_mw', 0));
    distribute('transport_demand_km', 'ies_transport', 'demand_km_per_h', comp => iesParam(comp.params, 'demand_km_per_h', 0));
    distribute('solar_available_mw', 'ies_solar', 'rated_mw', comp => iesParam(comp.params, 'rated_mw', 0), 'max');
    distribute('wind_available_mw', 'ies_wind', 'rated_mw', comp => iesParam(comp.params, 'rated_mw', 0), 'max');

    ['grid_buy_price', 'grid_sell_price', 'grid_carbon_tco2_mwh'].forEach(key => {
      const profile = profilesByKey.get(key);
      const rows = iesComponentsOf(components, 'ies_grid');
      if (!profile || !rows.length) return;
      let count = 0;
      rows.forEach(comp => { if (iesSetGridProfile(comp, key, profile.values, bundle.step_duration_hr)) count += 1; });
      if (count > 0) {
        applied += count;
        appliedNotes.push(`${key}→${count}`);
      }
    });

    bundle.profiles.forEach(profile => {
      const key = profile.key || iesCanonicalProfileKey(profile.name);
      if (key && IES_PROFILE_KEYS.includes(key)) return;
      const targetName = iesProfileNameKey(profile.name);
      if (!targetName) return;
      components.forEach(comp => {
        if (iesProfileNameKey(comp.params?.name) !== targetName) return;
        if (iesApplyNamedProfileToComponent(comp, { ...profile, step_duration_hr: bundle.step_duration_hr })) {
          applied += 1;
          appliedNotes.push(`${profile.name}→${comp.params?.name || comp.type}`);
        }
      });
    });

    if (applied <= 0) {
      log(`综合能源时序导入：${fileName} 已解析，但未匹配到画布元件或标准字段`, 'warn');
      return;
    }

    const hours = document.getElementById('iesHours');
    if (hours) hours.value = String(bundle.num_steps);
    _canvasDirty = true;
    if (Canvas.state?.selectedId !== null && Canvas.state?.selectedId !== undefined) {
      onSelectionChanged(Canvas.state.selectedId);
    }
    const shown = appliedNotes.slice(0, 6).join('，');
    const more = appliedNotes.length > 6 ? `，等 ${appliedNotes.length} 项` : '';
    log(`综合能源时序导入完成：${fileName}，${bundle.num_steps}步，已绑定 ${applied} 个元件/字段（${shown}${more}）`, 'success');
  }

  async function handleIntegratedEnergyProfilesImport(file) {
    try {
      const text = await iesReadBrowserFileText(file);
      const bundle = parseIntegratedEnergyProfiles(file.name, text);
      applyIntegratedEnergyImportedProfiles(bundle, file.name);
    } catch (err) {
      log(`综合能源时序导入失败：${err.message || err}`, 'error');
      setStatus('综合能源时序导入失败', 'error');
    }
  }

  function applyIntegratedEnergyCanvasPayload(payload) {
    const useCanvas = document.getElementById('iesUseCanvas')?.checked !== false;
    const components = iesActiveCanvasComponents();
    payload.canvas_model_used = false;
    payload.canvas_integrated_energy_component_count = components.length;
    if (!useCanvas) return payload;

    const typed = type => iesComponentsOf(components, type);
    const has = type => typed(type).length > 0;
    let usedCanvas = components.length > 0;
    const profileSteps = iesCanvasProfileLength(components);
    if (profileSteps > 0) {
      payload.num_steps = profileSteps;
      payload.step_duration_hr = iesCanvasProfileStepDuration(components, payload.step_duration_hr || 1.0);
    }

    const grids = typed('ies_grid');
    if (grids.length) {
      payload.import_limit_mw = grids.reduce((s, c) => s + iesParam(c.params, 'import_limit_mw', 0), 0);
      payload.export_limit_mw = grids.reduce((s, c) => s + iesParam(c.params, 'export_limit_mw', 0), 0);
      payload.grid_buy_price = iesWeightedAverage(grids, 'buy_price_per_mwh', 'import_limit_mw', 90);
      payload.grid_sell_price = iesWeightedAverage(grids, 'sell_price_per_mwh', 'export_limit_mw', 35);
      payload.grid_carbon_tco2_mwh = iesWeightedAverage(grids, 'carbon_tco2_mwh', 'import_limit_mw', 0.58);
      payload.grid_buy_price =
        iesWeightedAverageProfile(grids, 'buy_price_per_mwh', 'import_limit_mw', '_ies_buy_price_profile_values', payload.grid_buy_price) ||
        payload.grid_buy_price;
      payload.grid_sell_price =
        iesWeightedAverageProfile(grids, 'sell_price_per_mwh', 'export_limit_mw', '_ies_sell_price_profile_values', payload.grid_sell_price) ||
        payload.grid_sell_price;
      payload.grid_carbon_tco2_mwh =
        iesWeightedAverageProfile(grids, 'carbon_tco2_mwh', 'import_limit_mw', '_ies_carbon_profile_values', payload.grid_carbon_tco2_mwh) ||
        payload.grid_carbon_tco2_mwh;
    }

    if (has('ies_electric_load')) {
      const rows = typed('ies_electric_load');
      payload.electric_load_mw =
        iesAggregateComponentProfiles(rows, comp => iesParam(comp.params, 'demand_mw', 0)) ||
        iesSum(components, 'ies_electric_load', 'demand_mw');
    }
    else {
      const legacyLoad = iesLegacyElectricLoadMw();
      if (legacyLoad > 0) { payload.electric_load_mw = legacyLoad; usedCanvas = true; }
    }
    if (has('ies_heat_load')) {
      const rows = typed('ies_heat_load');
      payload.heat_load_mw =
        iesAggregateComponentProfiles(rows, comp => iesParam(comp.params, 'demand_mw', 0)) ||
        iesSum(components, 'ies_heat_load', 'demand_mw');
    }
    if (has('ies_hydrogen_load')) {
      const rows = typed('ies_hydrogen_load');
      payload.hydrogen_load_mw =
        iesAggregateComponentProfiles(rows, comp => iesParam(comp.params, 'demand_mw', 0)) ||
        iesSum(components, 'ies_hydrogen_load', 'demand_mw');
    }
    if (has('ies_fuel_load')) {
      const rows = typed('ies_fuel_load');
      payload.fuel_load_mw =
        iesAggregateComponentProfiles(rows, comp => iesParam(comp.params, 'demand_mw', 0)) ||
        iesSum(components, 'ies_fuel_load', 'demand_mw');
    }

    const transport = typed('ies_transport');
    if (transport.length) {
      payload.transport_demand_km =
        iesAggregateComponentProfiles(transport, comp => iesParam(comp.params, 'demand_km_per_h', 0)) ||
        transport.reduce((s, c) => s + iesParam(c.params, 'demand_km_per_h', 0), 0);
      payload.ev_ratio = iesWeightedAverage(transport, 'ev_ratio', 'demand_km_per_h', payload.ev_ratio);
      payload.hv_ratio = iesWeightedAverage(transport, 'hv_ratio', 'demand_km_per_h', payload.hv_ratio);
      payload.icv_ratio = iesWeightedAverage(transport, 'icv_ratio', 'demand_km_per_h', payload.icv_ratio);
      const ratioTotal = payload.ev_ratio + payload.hv_ratio + payload.icv_ratio;
      if (ratioTotal > 1e-9) {
        payload.ev_ratio /= ratioTotal;
        payload.hv_ratio /= ratioTotal;
        payload.icv_ratio /= ratioTotal;
      }
      payload.alpha_ev_mwh_per_km = iesWeightedAverage(transport, 'alpha_ev_mwh_per_km', 'demand_km_per_h', 0.00018);
      payload.alpha_hv_mwh_per_km = iesWeightedAverage(transport, 'alpha_hv_mwh_per_km', 'demand_km_per_h', 0.00060);
      payload.alpha_icv_mwh_per_km = iesWeightedAverage(transport, 'alpha_icv_mwh_per_km', 'demand_km_per_h', 0.00075);
    }

    if (has('ies_solar')) {
      const rows = typed('ies_solar');
      payload.solar_rated_mw = rows.reduce((s, c) => s + iesParam(c.params, 'rated_mw', 0), 0);
      payload.solar_om_cost_per_mwh = iesWeightedAverage(rows, 'om_cost_per_mwh', 'rated_mw', 2);
      payload.solar_available_mw =
        iesAggregateComponentProfiles(rows, comp =>
          iesParam(comp.params, 'rated_mw', 0) * iesParam(comp.params, 'availability_scale', 1)) ||
        payload.solar_available_mw;
    } else {
      const solar = iesLegacySolarRatedMw();
      if (solar > 0) { payload.solar_rated_mw = solar; usedCanvas = true; }
    }
    if (has('ies_wind')) {
      const rows = typed('ies_wind');
      payload.wind_rated_mw = rows.reduce((s, c) => s + iesParam(c.params, 'rated_mw', 0), 0);
      payload.wind_om_cost_per_mwh = iesWeightedAverage(rows, 'om_cost_per_mwh', 'rated_mw', 3);
      payload.wind_available_mw =
        iesAggregateComponentProfiles(rows, comp =>
          iesParam(comp.params, 'rated_mw', 0) * iesParam(comp.params, 'availability_scale', 1)) ||
        payload.wind_available_mw;
    } else {
      const wind = iesLegacyWindRatedMw();
      if (wind > 0) { payload.wind_rated_mw = wind; usedCanvas = true; }
    }

    const chp = typed('ies_chp');
    if (chp.length) {
      payload.chp_power_max_mw = chp.reduce((s, c) => s + iesParam(c.params, 'power_max_mw', 0), 0);
      payload.chp_heat_max_mw = chp.reduce((s, c) => s + iesParam(c.params, 'heat_max_mw', 0), 0);
      payload.eta_chp_elec = iesWeightedAverage(chp, 'eta_elec', 'power_max_mw', 0.35);
      payload.eta_chp_heat = iesWeightedAverage(chp, 'eta_heat', 'heat_max_mw', 0.45);
      payload.eta_chp_total = iesWeightedAverage(chp, 'eta_total', 'power_max_mw', 0.82);
      payload.chp_om_cost_per_mwh = iesWeightedAverage(chp, 'om_cost_per_mwh', 'power_max_mw', 5);
    }

    const heatPumps = typed('ies_heat_pump');
    if (heatPumps.length) {
      payload.heat_pump_power_max_mw = heatPumps.reduce((s, c) => s + iesParam(c.params, 'power_max_mw', 0), 0);
      payload.cop_heatpump = iesWeightedAverage(heatPumps, 'cop', 'power_max_mw', 3.2);
      payload.heat_pump_om_cost_per_mwh = iesWeightedAverage(heatPumps, 'om_cost_per_mwh', 'power_max_mw', 1);
    }

    const electrolyzers = typed('ies_electrolyzer');
    if (electrolyzers.length) {
      payload.electrolyzer_power_max_mw = electrolyzers.reduce((s, c) => s + iesParam(c.params, 'power_max_mw', 0), 0);
      payload.eta_electrolysis = iesWeightedAverage(electrolyzers, 'eta', 'power_max_mw', 0.65);
      payload.electrolyzer_om_cost_per_mwh = iesWeightedAverage(electrolyzers, 'om_cost_per_mwh', 'power_max_mw', 2);
    }

    const fuelCells = typed('ies_fuel_cell');
    if (fuelCells.length) {
      payload.fuel_cell_power_max_mw = fuelCells.reduce((s, c) => s + iesParam(c.params, 'power_max_mw', 0), 0);
      payload.eta_fuelcell = iesWeightedAverage(fuelCells, 'eta', 'power_max_mw', 0.52);
      payload.fuel_cell_om_cost_per_mwh = iesWeightedAverage(fuelCells, 'om_cost_per_mwh', 'power_max_mw', 4);
    }

    const eStores = typed('ies_electric_storage');
    if (eStores.length) {
      iesApplyStoragePayload(payload, eStores, 'electric');
      payload.eta_storage_charge = iesWeightedAverage(eStores, 'eta_charge', 'capacity_mwh', 0.95);
      payload.eta_storage_discharge = iesWeightedAverage(eStores, 'eta_discharge', 'capacity_mwh', 0.95);
      payload.electric_storage_retention = iesWeightedAverage(eStores, 'retention', 'capacity_mwh', 0.999);
      payload.storage_throughput_cost_per_mwh = iesWeightedAverage(eStores, 'throughput_cost_per_mwh', 'capacity_mwh', 1);
    } else {
      const legacyStorage = iesLegacyElectricStorage();
      if (legacyStorage.count > 0) {
        payload.electric_storage_capacity_mwh = legacyStorage.energy;
        payload.electric_storage_initial_mwh = legacyStorage.initial;
        payload.electric_storage_charge_max_mw = legacyStorage.charge;
        payload.electric_storage_discharge_max_mw = legacyStorage.discharge;
        usedCanvas = true;
      }
    }

    const tStores = typed('ies_thermal_storage');
    if (tStores.length) {
      iesApplyStoragePayload(payload, tStores, 'thermal');
      payload.thermal_storage_retention = iesWeightedAverage(tStores, 'retention', 'capacity_mwh', 0.995);
    }

    const hStores = typed('ies_hydrogen_storage');
    if (hStores.length) {
      const layerOf = comp => String(comp.params?.storage_layer || 'daily').toLowerCase();
      const daily = hStores.filter(comp => !['weekly', 'seasonal'].includes(layerOf(comp)));
      const weekly = hStores.filter(comp => layerOf(comp) === 'weekly');
      const seasonal = hStores.filter(comp => layerOf(comp) === 'seasonal');
      if (daily.length) iesApplyStoragePayload(payload, daily, 'hydrogen');
      if (weekly.length) iesApplyStoragePayload(payload, weekly, 'weekly_hydrogen');
      if (seasonal.length) iesApplyStoragePayload(payload, seasonal, 'seasonal_hydrogen');
      payload.hydrogen_storage_retention = iesWeightedAverage(daily.length ? daily : hStores, 'retention', 'capacity_mwh', 0.999);
      payload.weekly_hydrogen_storage_retention = iesWeightedAverage(weekly, 'retention', 'capacity_mwh', 0.9995);
      payload.seasonal_hydrogen_storage_retention = iesWeightedAverage(seasonal, 'retention', 'capacity_mwh', 0.9998);
      payload.hydrogen_storage_throughput_cost_per_mwh =
        iesWeightedAverage(hStores, 'throughput_cost_per_mwh', 'capacity_mwh', 1.5);
    }

    const ccus = typed('ies_ccus');
    if (ccus.length) {
      payload.ccus_capture_fraction = iesWeightedAverage(ccus, 'capture_fraction', 'max_tco2_per_h', payload.ccus_capture_fraction);
      payload.ccus_max_tco2_per_h = ccus.reduce((s, c) => s + iesParam(c.params, 'max_tco2_per_h', 0), 0);
      payload.ccus_power_mwh_per_tco2 = iesWeightedAverage(ccus, 'power_mwh_per_tco2', 'max_tco2_per_h', 0.12);
      payload.ccus_cost_per_tco2 = iesWeightedAverage(ccus, 'cost_per_tco2', 'max_tco2_per_h', 35);
    }

    const fuels = typed('ies_fuel_supply');
    if (fuels.length) {
      payload.fuel_purchase_limit_mw = fuels.reduce((s, c) => s + iesParam(c.params, 'purchase_limit_mw', 0), 0);
      payload.fuel_cost_per_mwh = iesWeightedAverage(fuels, 'cost_per_mwh', 'purchase_limit_mw', 38);
      payload.fuel_carbon_tco2_mwh = iesWeightedAverage(fuels, 'carbon_tco2_mwh', 'purchase_limit_mw', 0.27);
    }

    payload.canvas_model_used = usedCanvas;
    payload.canvas_component_summary = components.reduce((acc, comp) => {
      acc[comp.type] = (acc[comp.type] || 0) + 1;
      return acc;
    }, {});
    return payload;
  }

  function collectIntegratedEnergyPayload() {
    const evRatio = iesNumber('iesEvRatio', 0.45);
    const hvRatio = iesNumber('iesHvRatio', 0.20);
    const payload = {
      num_steps: Math.max(1, Math.round(iesNumber('iesHours', 24))),
      step_duration_hr: 1.0,
      solver: document.getElementById('iesSolver')?.value || 'auto',
      objective: document.getElementById('iesObjective')?.value || 'cost',
      import_limit_mw: iesNumber('iesImportLimit', 16),
      export_limit_mw: iesNumber('iesExportLimit', 5),
      electric_load_mw: iesNumber('iesElectricLoad', 8),
      heat_load_mw: iesNumber('iesHeatLoad', 4),
      hydrogen_load_mw: iesNumber('iesHydrogenLoad', 0.2),
      solar_rated_mw: iesNumber('iesSolarRated', 8),
      wind_rated_mw: iesNumber('iesWindRated', 4),
      chp_power_max_mw: iesNumber('iesChpPower', 4),
      heat_pump_power_max_mw: iesNumber('iesHeatPump', 3),
      electrolyzer_power_max_mw: iesNumber('iesElectrolyzer', 3),
      fuel_cell_power_max_mw: iesNumber('iesFuelCell', 2),
      electric_storage_capacity_mwh: iesNumber('iesElectricStorage', 10),
      hydrogen_storage_capacity_mwh: iesNumber('iesHydrogenStorage', 8),
      ev_ratio: evRatio,
      hv_ratio: hvRatio,
      icv_ratio: Math.max(0, 1 - evRatio - hvRatio),
      enable_carbon_budget: document.getElementById('iesEnableCarbonBudget')?.checked || false,
      co2_budget_tco2: iesNumber('iesCarbonBudget', 70),
      ccus_capture_fraction: iesNumber('iesCaptureFraction', 0.85),
      enforce_terminal_storage_cyclic: document.getElementById('iesCyclicStorage')?.checked !== false,
      enable_hydrogen_storage_layers: true,
    };
    return applyIntegratedEnergyCanvasPayload(payload);
  }

  function seedIntegratedEnergyCanvas() {
    if (typeof Canvas === 'undefined' || !Canvas.addComponent) return;
    const existing = iesActiveCanvasComponents().length;
    if (existing > 0 && !window.confirm('当前画布已有综合能源元件，是否继续追加示例？')) return;

    const add = (type, x, y, params = {}) =>
      Canvas.addComponent(type, x, y, { ...COMP.defaults[type], ...params });
    const link = (a, ap, b, bp) => {
      try { Canvas.addConnection(a.id, ap, b.id, bp); } catch (_) { /* keep sample creation best-effort */ }
    };

    const eBus = add('ies_electric_bus', 120, 140, { name: '园区电母线' });
    const hBus = add('ies_heat_bus', 120, 320, { name: '园区热母线' });
    const h2Bus = add('ies_hydrogen_bus', 520, 140, { name: '园区氢母线' });
    const fuelBus = add('ies_fuel_bus', 520, 320, { name: '燃料母线' });

    const grid = add('ies_grid', -100, 140);
    const solar = add('ies_solar', 120, -40);
    const wind = add('ies_wind', 300, -40);
    const eLoad = add('ies_electric_load', 120, 235);
    const heatLoad = add('ies_heat_load', 120, 415);
    const h2Load = add('ies_hydrogen_load', 700, 140);
    const transport = add('ies_transport', 705, 250);
    const fuel = add('ies_fuel_supply', 705, 320);
    const chp = add('ies_chp', 360, 320);
    const heatPump = add('ies_heat_pump', 300, 220);
    const electrolyzer = add('ies_electrolyzer', 340, 100);
    const fuelCell = add('ies_fuel_cell', 520, 40);
    const bess = add('ies_electric_storage', -100, 40);
    const tes = add('ies_thermal_storage', -100, 320);
    const h2Store = add('ies_hydrogen_storage', 520, 235);
    const ccus = add('ies_ccus', 360, 420);

    link(grid, 'electric', eBus, 'left');
    link(solar, 'electric', eBus, 'top');
    link(wind, 'electric', eBus, 'top');
    link(eLoad, 'electric', eBus, 'bottom');
    link(bess, 'electric', eBus, 'left');
    link(heatLoad, 'heat', hBus, 'bottom');
    link(tes, 'heat', hBus, 'left');
    link(h2Load, 'hydrogen', h2Bus, 'right');
    link(h2Store, 'hydrogen', h2Bus, 'bottom');
    link(fuel, 'fuel', fuelBus, 'right');
    link(chp, 'fuel', fuelBus, 'left');
    link(chp, 'electric', eBus, 'right');
    link(chp, 'heat', hBus, 'right');
    link(heatPump, 'electric', eBus, 'right');
    link(heatPump, 'heat', hBus, 'right');
    link(electrolyzer, 'electric', eBus, 'right');
    link(electrolyzer, 'hydrogen', h2Bus, 'left');
    link(fuelCell, 'hydrogen', h2Bus, 'left');
    link(fuelCell, 'electric', eBus, 'right');
    link(transport, 'electric', eBus, 'right');
    link(transport, 'hydrogen', h2Bus, 'right');
    link(transport, 'fuel', fuelBus, 'right');
    link(ccus, 'electric', eBus, 'right');

    if (Canvas.zoomFit) Canvas.zoomFit();
    _canvasDirty = true;
    updateTopologyTables();
    log('已添加园区综合能源示例画布，可直接选择元件修改容量、效率和成本参数。', 'success');
  }

  function iesFmt(value, digits = 2) {
    const n = Number(value);
    return Number.isFinite(n) ? n.toFixed(digits) : '—';
  }

  function iesTimeAxis(data) {
    const n = Number(data?.num_steps) || Math.max(0, (data?.p_grid_import_mw || []).length);
    const dt = Number(data?.step_duration_hr) || 1;
    return Array.from({ length: n }, (_, i) => i * dt);
  }

  function renderIntegratedEnergySummary(data) {
    const el = document.getElementById('iesSummary');
    if (!el) return;
    const s = data.summary || {};
    const cards = [
      ['总成本', iesFmt(s.total_cost, 0)],
      ['购电量', `${iesFmt(s.total_grid_import_mwh, 1)} MWh`],
      ['新能源利用率', `${iesFmt(100 * Number(s.renewable_utilization || 0), 1)}%`],
      ['弃能量', `${iesFmt(s.total_curtailment_mwh, 1)} MWh`],
      ['碳排放', `${iesFmt(s.total_carbon_residual_tco2, 2)} tCO2`],
      ['CO2捕集', `${iesFmt(s.total_co2_captured_tco2, 2)} tCO2`],
      ['热负荷', `${iesFmt(s.total_heat_load_mwh, 1)} MWh`],
      ['交通需求', `${iesFmt(s.total_transport_km, 0)} km`],
    ];
    const sourceText = data.canvas_model_used
      ? `；来源：画布元件 (${Number(data.canvas_integrated_energy_component_count || 0)} 个)`
      : '；来源：参数面板';
    el.innerHTML = `
      <div class="ies-kpi-grid">
        ${cards.map(([label, value]) => `<div class="ies-kpi-card"><span>${escapeHtml(label)}</span><strong>${escapeHtml(value)}</strong></div>`).join('')}
      </div>
      <div class="sub-hint">状态：${escapeHtml(data.status || '')}；求解器：${escapeHtml(data.solver_name || 'auto')}；目标：${escapeHtml(data.objective_mode || 'cost')}；耗时 ${iesFmt(data.solve_time_sec, 3)} s${escapeHtml(sourceText)}</div>`;
  }

  function iesCarrierColor(carrier, alpha = 0.45) {
    const colors = {
      electricity: `rgba(97,175,239,${alpha})`,
      heat: `rgba(224,108,117,${alpha})`,
      hydrogen: `rgba(86,182,194,${alpha})`,
      fuel: `rgba(229,192,123,${alpha})`,
    };
    return colors[carrier] || `rgba(152,195,121,${alpha})`;
  }

  function renderIntegratedEnergySankey(data) {
    const div = document.getElementById('iesSankeyChart');
    if (!div) return;
    if (typeof Plotly === 'undefined') {
      div.innerHTML = '<p class="empty-hint">Plotly 未加载，无法展示 Sankey 图。</p>';
      return;
    }
    const nodes = data.sankey?.nodes || {};
    const links = data.sankey?.links || {};
    const labels = nodes.label || [];
    const values = links.value || [];
    if (!labels.length || !values.length) {
      div.innerHTML = '<p class="empty-hint">暂无可显示的多能流。</p>';
      return;
    }
    const carriers = links.carrier || [];
    Plotly.newPlot(div, [{
      type: 'sankey',
      arrangement: 'snap',
      node: {
        pad: 14,
        thickness: 16,
        line: { color: 'rgba(255,255,255,0.25)', width: 0.5 },
        label: labels,
        color: nodes.color || labels.map(() => 'rgba(97,175,239,0.95)'),
      },
      link: {
        source: links.source || [],
        target: links.target || [],
        value: values,
        color: carriers.map(c => iesCarrierColor(c, 0.38)),
        customdata: carriers,
        hovertemplate: '%{source.label} → %{target.label}<br>%{value:.2f} MWh<br>%{customdata}<extra></extra>',
      },
    }], {
      margin: { l: 10, r: 10, t: 10, b: 10 },
      paper_bgcolor: 'rgba(0,0,0,0)',
      font: { color: '#dcdfe4', size: 11 },
    }, { responsive: true });
  }

  function renderIntegratedEnergyCharts(data) {
    if (typeof Plotly === 'undefined') return;
    const x = iesTimeAxis(data);
    const profiles = data.input_profiles || {};
    const dispatchDiv = document.getElementById('iesDispatchChart');
    if (dispatchDiv) {
      Plotly.newPlot(dispatchDiv, [
        { x, y: profiles.electric_load_mw || [], mode: 'lines', name: '电负荷', line: { color: '#e5c07b', width: 2 } },
        { x, y: data.p_grid_import_mw || [], mode: 'lines', name: '购电', line: { color: '#61afef' } },
        { x, y: data.p_solar_mw || [], mode: 'lines', name: '光伏', stackgroup: 'gen', line: { color: '#f5c542' } },
        { x, y: data.p_wind_mw || [], mode: 'lines', name: '风电', stackgroup: 'gen', line: { color: '#98c379' } },
        { x, y: data.p_chp_mw || [], mode: 'lines', name: 'CHP电', stackgroup: 'gen', line: { color: '#e06c75' } },
        { x, y: data.p_fuelcell_mw || [], mode: 'lines', name: '燃料电池', stackgroup: 'gen', line: { color: '#56b6c2' } },
        { x, y: data.p_grid_export_mw || [], mode: 'lines', name: '售电', line: { color: '#c678dd', dash: 'dot' } },
      ], {
        margin: { l: 55, r: 18, t: 20, b: 42 },
        xaxis: { title: 'Hour' },
        yaxis: { title: 'MW' },
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        font: { color: '#dcdfe4' },
        legend: { orientation: 'h', y: -0.22 },
      }, { responsive: true });
    }
    const storageDiv = document.getElementById('iesStorageChart');
    if (storageDiv) {
      const xs = Array.from({ length: (data.e_storage_mwh || []).length }, (_, i) => i);
      Plotly.newPlot(storageDiv, [
        { x: xs, y: data.e_storage_mwh || [], mode: 'lines', name: '电储能 MWh', line: { color: '#61afef' } },
        { x: xs, y: data.q_storage_mwh || [], mode: 'lines', name: '热储能 MWh', line: { color: '#e06c75' } },
        { x: xs, y: data.h_storage_mwh || [], mode: 'lines', name: '日内氢储 MWh', line: { color: '#56b6c2' } },
        { x: xs, y: data.h_weekly_storage_mwh || [], mode: 'lines', name: '周间氢储 MWh', line: { color: '#c678dd' } },
      ], {
        margin: { l: 55, r: 18, t: 20, b: 42 },
        xaxis: { title: 'State Step' },
        yaxis: { title: 'MWh' },
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        font: { color: '#dcdfe4' },
        legend: { orientation: 'h', y: -0.22 },
      }, { responsive: true });
    }
    const carbonDiv = document.getElementById('iesCarbonChart');
    if (carbonDiv) {
      Plotly.newPlot(carbonDiv, [
        { x, y: data.emissions_tco2 || [], type: 'bar', name: '总排放', marker: { color: '#e06c75' } },
        { x, y: data.co2_captured_tco2 || [], type: 'bar', name: '捕集', marker: { color: '#56b6c2' } },
        { x, y: data.carbon_residual_tco2 || [], mode: 'lines+markers', name: '残余碳', line: { color: '#e5c07b' } },
      ], {
        barmode: 'group',
        margin: { l: 55, r: 18, t: 20, b: 42 },
        xaxis: { title: 'Hour' },
        yaxis: { title: 'tCO2' },
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        font: { color: '#dcdfe4' },
        legend: { orientation: 'h', y: -0.22 },
      }, { responsive: true });
    }
  }

  function renderIntegratedEnergyTable(data) {
    const el = document.getElementById('iesResultsTable');
    if (!el) return;
    const profiles = data.input_profiles || {};
    const n = Math.min(Number(data.num_steps) || 0, 48);
    let html = '<table><thead><tr><th>t</th><th>电负荷</th><th>购电</th><th>售电</th><th>光伏</th><th>风电</th><th>CHP</th><th>热泵</th><th>电解槽</th><th>CO2残余</th></tr></thead><tbody>';
    for (let i = 0; i < n; ++i) {
      html += `<tr><td>${i}</td><td>${iesFmt((profiles.electric_load_mw || [])[i])}</td><td>${iesFmt((data.p_grid_import_mw || [])[i])}</td><td>${iesFmt((data.p_grid_export_mw || [])[i])}</td><td>${iesFmt((data.p_solar_mw || [])[i])}</td><td>${iesFmt((data.p_wind_mw || [])[i])}</td><td>${iesFmt((data.p_chp_mw || [])[i])}</td><td>${iesFmt((data.p_heatpump_mw || [])[i])}</td><td>${iesFmt((data.p_electrolysis_h2_mw || [])[i])}</td><td>${iesFmt((data.carbon_residual_tco2 || [])[i], 3)}</td></tr>`;
    }
    html += '</tbody></table>';
    if ((Number(data.num_steps) || 0) > n) {
      html += `<p class="empty-hint">仅显示前 ${n} 个时段，完整结果请导出 JSON。</p>`;
    }
    el.innerHTML = html;
  }

  function renderIntegratedEnergyResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('integratedEnergy');
    renderIntegratedEnergySummary(data);
    renderIntegratedEnergySankey(data);
    renderIntegratedEnergyCharts(data);
    renderIntegratedEnergyTable(data);
    switchTab('results');
  }

  async function runIntegratedEnergy() {
    setStatus('园区综合能源仿真中...', 'busy');
    const payload = collectIntegratedEnergyPayload();
    const data = await apiPost('/api/session/run_campus_ies', payload);
    if (data && !data.error) {
      _lastIntegratedEnergyData = data;
      renderIntegratedEnergyResults(data);
      setStatus(data.feasible ? '园区综合能源仿真完成' : '园区综合能源仿真未找到可行解',
                data.feasible ? 'success' : 'warn');
    } else {
      setStatus('园区综合能源仿真失败', 'error');
    }
  }

  // ========== Init ==========
  // ---- Theme (light / dark background) ----
  // Keeps the dark palette as the default so existing users see no change until
  // they toggle.  The choice persists in localStorage across reloads.
  function setThemeMode(mode) {
    const finalMode = mode === 'light' ? 'light' : 'dark';
    document.documentElement.setAttribute('data-theme', finalMode);
    try { localStorage.setItem('themeMode', finalMode); } catch (e) { /* ignore */ }
    const btn = document.getElementById('btnToggleTheme');
    if (btn) btn.title = finalMode === 'light' ? '切换到深色背景' : '切换到浅色背景';
  }

  function toggleThemeMode() {
    const current = document.documentElement.getAttribute('data-theme') || 'dark';
    setThemeMode(current === 'light' ? 'dark' : 'light');
  }

  function initThemeMode() {
    let saved = 'dark';
    try { saved = localStorage.getItem('themeMode') || 'dark'; } catch (e) { /* ignore */ }
    setThemeMode(saved);
    document.getElementById('btnToggleTheme')?.addEventListener('click', toggleThemeMode);
  }

  // Current auto-layout direction chosen in the toolbar (TB/LR/RADIAL/COMPACT).
  function layoutDirection() {
    return document.getElementById('layoutDirSelect')?.value || 'TB';
  }

  function init() {
    // Apply the saved light/dark theme before anything renders.
    initThemeMode();

    // Initialize canvas
    Canvas.init();

    // Initialize component library
    initComponentLibrary();

    // Load case list
    loadCaseList();
    loadMatpowerFileList();

    // ---- Toolbar Events ----
    // Bar 1: 加载算例 -> open case-load modal
    document.getElementById('btnLoadCase').addEventListener('click', showCaseLoadModal);
    document.getElementById('resAcFaultLocations')?.addEventListener('input', markResilienceFaultBranches);
    document.getElementById('resDcFaultLocations')?.addEventListener('input', markResilienceFaultBranches);

    // Case-load modal: three import paths
    document.getElementById('btnImportLibraryCase')?.addEventListener('click', () => {
      const caseName = document.getElementById('caseSelect').value;
      if (!caseName) {
        log('请先在下拉框中选择一个内置算例', 'warn');
        return;
      }
      hideCaseLoadModal();
      loadBuiltinCase(caseName);
    });
    document.getElementById('btnImportMatpowerCase')?.addEventListener('click', () => {
      const filename = document.getElementById('matpowerSelect').value;
      if (!filename) {
        log('请先在下拉框中选择一个 MATPOWER 文件', 'warn');
        return;
      }
      hideCaseLoadModal();
      loadMatpowerCase(filename);
    });
    document.getElementById('btnImportJsonCase')?.addEventListener('click', () => {
      document.getElementById('fileImportJson').click();
    });
    // ETAP workbook (.xlsx) import from the case-load modal.
    document.getElementById('btnImportEtapXlsxCase')?.addEventListener('click', () => {
      document.getElementById('fileImportEtapXlsx')?.click();
    });
    document.getElementById('fileImportEtapXlsx')?.addEventListener('change', (e) => {
      const f = e.target.files[0];
      e.target.value = '';
      if (f) { hideCaseLoadModal(); loadEtapXlsx(f); }
    });
    // Native ETAP project (.xml) import from the case-load modal.
    document.getElementById('btnImportEtapXmlCase')?.addEventListener('click', () => {
      document.getElementById('fileImportEtapXml')?.click();
    });
    document.getElementById('fileImportEtapXml')?.addEventListener('change', (e) => {
      const f = e.target.files[0];
      e.target.value = '';
      if (f) { hideCaseLoadModal(); loadEtapXml(f); }
    });
    document.getElementById('btnCaseModalClose')?.addEventListener('click', hideCaseLoadModal);
    document.querySelector('#caseLoadModal .modal-backdrop')?.addEventListener('click', hideCaseLoadModal);

    document.getElementById('btnNewSystem').addEventListener('click', createNewSystem);
    // Legacy buttons may have been replaced; guard with optional chaining.
    document.getElementById('btnLoadMatpower')?.addEventListener('click', () => {
      const filename = document.getElementById('matpowerSelect').value;
      if (filename) loadMatpowerCase(filename);
    });
    document.getElementById('btnExportJson').addEventListener('click', exportJson);
    document.getElementById('btnExportEtap')?.addEventListener('click', exportEtap);
    document.getElementById('btnExportEtapXml')?.addEventListener('click', exportEtapXml);
    document.getElementById('btnImportJson').addEventListener('click', () => {
      document.getElementById('fileImportJson').click();
    });
    document.getElementById('fileImportJson').addEventListener('change', (e) => {
      if (e.target.files[0]) importJson(e.target.files[0]);
      e.target.value = '';
    });

    // Bar 3: unified model IO module
    document.getElementById('btnIoLoadBuiltin')?.addEventListener('click', () => {
      const caseName = document.getElementById('ioCaseSelect')?.value || '';
      if (!caseName) {
        log('请先在模型IO中选择一个内置算例', 'warn');
        return;
      }
      loadBuiltinCase(caseName);
    });
    document.getElementById('btnIoLoadMatpower')?.addEventListener('click', () => {
      const filename = document.getElementById('ioMatpowerSelect')?.value || '';
      if (!filename) {
        log('请先在模型IO中选择一个 MATPOWER 文件', 'warn');
        return;
      }
      loadMatpowerCase(filename);
    });
    document.getElementById('btnIoImportJson')?.addEventListener('click', () => {
      document.getElementById('fileImportJson')?.click();
    });
    document.getElementById('btnIoImportEtapXlsx')?.addEventListener('click', () => {
      document.getElementById('fileImportEtapXlsx')?.click();
    });
    document.getElementById('btnIoImportEtapXml')?.addEventListener('click', () => {
      document.getElementById('fileImportEtapXml')?.click();
    });
    document.getElementById('btnIoImportGridlabd')?.addEventListener('click', () => {
      document.getElementById('fileImportGridlabd')?.click();
    });
    document.getElementById('fileImportGridlabd')?.addEventListener('change', (e) => {
      const f = e.target.files[0];
      e.target.value = '';
      if (f) loadGridlabd(f);
    });
    document.getElementById('btnIoImportOpendss')?.addEventListener('click', () => {
      document.getElementById('fileImportOpendss')?.click();
    });
    document.getElementById('fileImportOpendss')?.addEventListener('change', (e) => {
      const f = e.target.files[0];
      e.target.value = '';
      if (f) loadOpendss(f);
    });
    document.getElementById('btnIoNewSystem')?.addEventListener('click', createNewSystem);
    document.getElementById('btnIoExportJson')?.addEventListener('click', exportJson);
    document.getElementById('btnIoExportMatpower')?.addEventListener('click', exportMatpower);
    document.getElementById('btnIoExportEtap')?.addEventListener('click', exportEtap);
    document.getElementById('btnIoExportEtapXml')?.addEventListener('click', exportEtapXml);
    document.getElementById('btnIoExportGridlabd')?.addEventListener('click', () => exportExternalGrid('gridlabd'));
    document.getElementById('btnIoExportOpendss')?.addEventListener('click', () => exportExternalGrid('opendss'));
    document.getElementById('btnIoModelCompatibility')?.addEventListener('click', () => runModelCompatibility({
      targetId: 'modelIoResults',
      resultGroup: 'modelIO',
      title: '模型兼容性检查',
      subtitle: 'JSON / Canonical / GridLAB-D / OpenDSS / 标准模型映射',
    }));
    document.getElementById('btnIoSyncBackend')?.addEventListener('click', async () => {
      setStatus('同步画布中...', 'busy');
      const ok = await syncToBackend(true);
      if (ok) {
        showModelIoStatus('画布同步完成', [
          ['同步方向', 'Canvas -> Backend session'],
          ['状态', '成功'],
        ], { subtitle: '后续导出和兼容性检查将使用当前画布系统' });
        setStatus('同步完成');
      } else {
        setStatus('同步失败', 'error');
      }
    });

    // Calculation buttons (Bar 3 "运行..." buttons reuse original IDs where possible)
    document.getElementById('btnExportAllResults')?.addEventListener('click', exportAllCachedResults);
    document.getElementById('btnPowerFlow').addEventListener('click', runPowerFlow);
    document.getElementById('btnPfAdvanced')?.addEventListener('click', () => {
      const panel = document.getElementById('pfAdvancedPanel');
      const btn = document.getElementById('btnPfAdvanced');
      if (!panel || !btn) return;
      const nextOpen = panel.hasAttribute('hidden');
      panel.toggleAttribute('hidden', !nextOpen);
      btn.classList.toggle('active', nextOpen);
      btn.setAttribute('aria-expanded', nextOpen ? 'true' : 'false');
    });
    document.getElementById('btnRunOpf')?.addEventListener('click', runOpf);
    document.getElementById('btnCarbonFlow')?.addEventListener('click', runCarbonFlow);
    document.getElementById('carbonSankeyMetric')?.addEventListener('change', () => {
      if (_lastCarbonData) renderCarbonSankey(_lastCarbonData);
    });
    document.getElementById('btnImportCarbonFactors')?.addEventListener('click', () => {
      document.getElementById('fileImportCarbonFactors')?.click();
    });
    document.getElementById('fileImportCarbonFactors')?.addEventListener('change', async (e) => {
      const f = e.target.files[0];
      e.target.value = '';
      if (f) await importCarbonFactorsJson(f);
    });
    // Legacy header buttons removed — guard:
    document.getElementById('btnShortCircuit')?.addEventListener('click', showScDialog);
    document.getElementById('btnTopology')?.addEventListener('click', runTopologyReconfig);
    document.getElementById('btnTimeSeries')?.addEventListener('click', showTspfDialog);

    // Bar 3: shortCircuit sub-toolbar
    document.getElementById('btnFaultLocation')?.addEventListener('click', showScDialog);
    document.getElementById('scDomainSelect')?.addEventListener('change', (e) => {
      const f = document.getElementById('scDomain');
      if (f) f.value = e.target.value;
    });
    document.getElementById('faultTypeSelect')?.addEventListener('change', (e) => {
      const f = document.getElementById('scFaultType');
      if (f) f.value = e.target.value;
    });
    document.getElementById('scCalcTypeSelect')?.addEventListener('change', (e) => {
      const f = document.getElementById('scCalcType');
      if (f) f.value = e.target.value;
    });
    document.getElementById('voltageCorrectionFactor')?.addEventListener('change', (e) => {
      const f = document.getElementById('scCFactor');
      if (f) f.value = e.target.value;
    });
    document.getElementById('scKappaMethodSelect')?.addEventListener('change', (e) => {
      const f = document.getElementById('scKappaMethod');
      if (f) f.value = e.target.value;
    });
    document.getElementById('scTopologySelect')?.addEventListener('change', (e) => {
      const f = document.getElementById('scTopology');
      if (f) f.value = e.target.value;
    });
    document.getElementById('btnRunShortCircuit')?.addEventListener('click', () => {
      syncScToolbarToHidden();
      runShortCircuit();
    });

    // Bar 3: harmonic power flow
    document.getElementById('btnRunHarmonics')?.addEventListener('click', runHarmonics);
    document.getElementById('hpfMode')?.addEventListener('change', hpfUpdateModeControls);
    hpfUpdateModeControls();

    // Bar 3: transient phasor dynamics
    document.getElementById('btnRunTransient')?.addEventListener('click', runTransientSimulation);
    document.getElementById('btnTransientCompatibility')?.addEventListener('click', runTransientCompatibility);
    document.getElementById('trPowerFlowInit')?.addEventListener('change', updateTransientPfControls);
    document.getElementById('trEventType')?.addEventListener('change', updateTransientEventControls);
    document.getElementById('trObserverAcBuses')?.addEventListener('change', refreshTransientObserverSelection);
    document.getElementById('trObserverDcBuses')?.addEventListener('change', refreshTransientObserverSelection);
    document.getElementById('btnAddTransientEvent')?.addEventListener('click', addTransientEventFromControls);
    document.getElementById('btnClearTransientEvents')?.addEventListener('click', clearTransientEvents);
    updateTransientPfControls();
    updateTransientEventControls();
    renderTransientEventSchedule();
    document.getElementById('btnExportTransient')?.addEventListener('click', () => {
      if (!_lastTransientData) {
        log('暂无暂态仿真结果可导出，请先运行暂态仿真', 'warn');
        return;
      }
      downloadJsonFile(`transient_results_${tsTagForFilename()}.json`, _lastTransientData);
    });

    // Bar 3: topology
    document.getElementById('btnRunTopology')?.addEventListener('click', runTopologyReconfig);

    // Bar 3: topology analysis (graph structure)
    document.getElementById('btnRunTopologyAnalysis')?.addEventListener('click', runTopologyAnalysis);
    document.getElementById('btnExportTopologyAnalysis')?.addEventListener('click', () => {
      if (!_lastTopoAnalysisData) {
        setStatus('请先运行拓扑分析', 'warn');
        return;
      }
      downloadJsonFile('topology_analysis.json', _lastTopoAnalysisData);
    });
    ['topoShowIslands', 'topoShowBridges', 'topoShowCutVertices'].forEach(id => {
      document.getElementById(id)?.addEventListener('change', () => {
        if (_lastTopoAnalysisData) Canvas.showTopologyResults(_lastTopoAnalysisData, topoOverlayOptions());
      });
    });

    // Bar 3: network reduction (graph reduction — before/after view)
    document.getElementById('btnRunNetworkReduction')?.addEventListener('click', runNetworkReduction);
    document.getElementById('btnExportNetworkReduction')?.addEventListener('click', () => {
      if (!_lastNetReductionData) {
        setStatus('请先运行网络化简', 'warn');
        return;
      }
      downloadJsonFile('network_reduction.json', _lastNetReductionData);
    });

    // Bar 3: hosting capacity
    document.getElementById('btnRunBearingCap')?.addEventListener('click', showBcDialog);
    // Legacy bearing-capacity launcher (header button removed)
    document.getElementById('btnBearingCap')?.addEventListener('click', showBcDialog);

    // Bar 3: time-series — run directly with inline params (skip UC / OPF).
    document.getElementById('btnRunTimeSeriesPF')?.addEventListener('click', runTimeSeriesPF);
    document.getElementById('btnSeedIntegratedEnergyCanvas')?.addEventListener('click', seedIntegratedEnergyCanvas);
    document.getElementById('btnImportIntegratedEnergyProfiles')?.addEventListener('click', () => {
      document.getElementById('fileImportIntegratedEnergyProfiles')?.click();
    });
    document.getElementById('fileImportIntegratedEnergyProfiles')?.addEventListener('change', async (e) => {
      const file = e.target.files && e.target.files[0];
      e.target.value = '';
      if (file) await handleIntegratedEnergyProfilesImport(file);
    });
    document.getElementById('btnRunIntegratedEnergy')?.addEventListener('click', runIntegratedEnergy);
    document.getElementById('btnExportIntegratedEnergy')?.addEventListener('click', () => {
      if (!_lastIntegratedEnergyData) {
        log('请先运行园区综合能源仿真', 'warn');
        return;
      }
      downloadJsonFile(`campus_integrated_energy_${tsTagForFilename()}.json`, _lastIntegratedEnergyData);
    });
    document.getElementById('btnToggleAnnualPanel')?.addEventListener('click', () => {
      const panel = document.getElementById('annualSimControls');
      const btn = document.getElementById('btnToggleAnnualPanel');
      if (!panel || !btn) return;
      const show = panel.hasAttribute('hidden');
      if (show) panel.removeAttribute('hidden'); else panel.setAttribute('hidden', '');
      panel.closest('.ts-annual-group')?.classList.toggle('ts-annual-expanded', show);
      btn.setAttribute('aria-expanded', show ? 'true' : 'false');
      btn.innerHTML = show ? '收起<br>年度设置 ▾' : '展开<br>年度设置 ▸';
    });
    document.getElementById('btnRunAnnualSim')?.addEventListener('click', runAnnualSim);
    document.getElementById('btnExportAnnualSim')?.addEventListener('click', () => {
      if (!_lastAnnualData) { log('请先运行年度并行生产模拟', 'warn'); return; }
      downloadJsonFile(`annual_production_sim_${tsTagForFilename()}.json`, _lastAnnualData);
    });
    document.getElementById('btnAnnualDayDetail')?.addEventListener('click', runAnnualDayDetail);
    document.getElementById('btnDynamicCarbonFlow')?.addEventListener('click', runDynamicCarbonFlow);

    // Bar 3: PF result export — write _lastPfData to a JSON file.
    document.getElementById('btnExportPfResults')?.addEventListener('click', () => {
      if (!_lastPfData) {
        log('暂无潮流计算结果可导出，请先运行潮流计算', 'warn');
        return;
      }
      downloadJsonFile(`pf_results_${tsTagForFilename()}.json`, _lastPfData);
    });

    // Bar 3: time-series result export. Bundles the full per-step bus
    // voltages (magnitude + angle) and per-branch flows (Pf, Pt, Qf, Qt)
    // alongside the original TSPF response so downstream tools have
    // everything in one file.
    document.getElementById('btnExportTspfResults')?.addEventListener('click', () => {
      if (!_lastTspfData) {
        log('暂无时序潮流结果可导出，请先运行时序潮流计算', 'warn');
        return;
      }
      const d = _lastTspfData;
      const bundle = {
        meta: {
          exported_at: new Date().toISOString(),
          num_steps: d.num_steps,
          num_converged: d.num_converged,
          total_generation_cost: d.total_generation_cost,
        },
        bus_voltages: {
          bus_labels: d.bus_labels || [],
          vm_matrix:  d.vm_matrix  || [],     // [bus][t]  pu
          va_matrix:  d.va_matrix  || [],     // [bus][t]  rad
        },
        branch_flows: {
          branch_labels:  d.branch_labels  || [],
          branch_from_bus: d.branch_from_bus || [],
          branch_to_bus:   d.branch_to_bus   || [],
          pf_mw:    d.branch_pf_mw  || [],    // [branch][t]  MW (from-bus injection)
          pt_mw:    d.branch_pt_mw  || [],    // [branch][t]  MW (to-bus injection)
          qf_mvar:  d.branch_qf_mvar || [],   // [branch][t]  MVAr
          qt_mvar:  d.branch_qt_mvar || [],
        },
        per_step_summary: {
          vm_mean:   d.vm_mean   || [],
          vm_min:    d.vm_min    || [],
          vm_max:    d.vm_max    || [],
          losses_mw: d.losses_mw || [],
          total_load_mw: d.total_load || [],
        },
        raw: d,   // keep entire response for completeness
      };
      downloadJsonFile(`tspf_results_${tsTagForFilename()}.json`, bundle);
      const nbus = (d.bus_labels || []).length;
      const nbr  = (d.branch_labels || []).length;
      log(`时序潮流结果已导出：${d.num_steps} 步 × ${nbus} 母线电压/相角 × ${nbr} 分支 Pf/Pt/Qf/Qt`, 'success');
    });

	    function updateReliabilityControlState() {
	      const physicalModel = document.getElementById('relPhysicalModel')?.value || 'auto';
	      const methodEl = document.getElementById('relMethod');
	      const maxIterEl = document.getElementById('relMaxIter');
	      const hintEl = document.getElementById('relAnalysisHint');
	      const useThreeStage = physicalModel === 'restoration_milp';
	      if (methodEl) methodEl.disabled = useThreeStage;
	      if (maxIterEl) maxIterEl.disabled = useThreeStage;
	      if (hintEl) {
	        hintEl.textContent = useThreeStage
	          ? '三阶段恢复 MILP 将作为独立可靠性评估运行'
	          : '后果模型由所选统计方法自动匹配';
	      }
	    }

	    // Bar 3: Reliability — run + export. Backend live.
	    async function runReliability() {
	      setStatus('可靠性分析中...', 'busy');
	      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
	      const selectedMethod = (document.getElementById('relMethod')?.value) || 'nsq';
	      const physicalModel = (document.getElementById('relPhysicalModel')?.value) || 'auto';
	      const method = physicalModel === 'restoration_milp' ? 'three_stage' : selectedMethod;
	      const maxIter = parseInt(document.getElementById('relMaxIter')?.value) || 2000;
	      const metricFocus = (document.getElementById('relMetricFocus')?.value) || 'all';
	      const weakBasis = (document.getElementById('relWeakBasis')?.value) || 'auto';
	      const relParallel = document.getElementById('relParallel')?.checked ?? true;
	      const relParallelThreads = parseInt(document.getElementById('relParallelThreads')?.value, 10);
	      const opts = {
	        method,
	        physical_model: physicalModel,
	        data_policy: (document.getElementById('relDataPolicy')?.value) || 'missing_only',
	        reliability_template: (document.getElementById('relTemplate')?.value) || 'none',
	        load: { scale_factor: 1.0, hours_per_year: 8736 },
	        execution: {
	          parallel: relParallel,
	          parallel_threads: Number.isFinite(relParallelThreads) ? relParallelThreads : 0,
	        },
	        monte_carlo: {
	          max_iterations: method === 'seq' ? Math.max(50, Math.round(maxIter / 10)) : maxIter,
	          cov_threshold: 0.05,
	          compute_tail_risk: false,
	          parallel: relParallel,
	          parallel_threads: Number.isFinite(relParallelThreads) ? relParallelThreads : 0,
	        },
	        restoration: {
	          enable_switch_reconfiguration: true,
	          max_switch_actions: 2,
	          parallel: relParallel,
	          parallel_threads: Number.isFinite(relParallelThreads) ? relParallelThreads : 0,
	          include_converter_faults: !!document.getElementById('relTsFaults')?.checked,
	          include_switch_faults: !!document.getElementById('relTsFaults')?.checked,
	          include_generator_faults: !!document.getElementById('relTsFaults')?.checked,
	          include_transformer_faults: !!document.getElementById('relTsFaults')?.checked,
	          include_dc_power_flow: !!document.getElementById('relTsDcPf')?.checked,
	        },
	        failure_mode_scope: {
	          include_passive: !!document.getElementById('relFmPassive')?.checked,
	          include_active_on_demand: !!document.getElementById('relFmActive')?.checked,
	          include_physical: !!document.getElementById('relFmPhysical')?.checked,
	          include_cyber_control: !!document.getElementById('relFmCyber')?.checked,
	          include_communication: !!document.getElementById('relFmCyber')?.checked,
	          include_measurement: !!document.getElementById('relFmCyber')?.checked,
	          include_protection_logic: !!document.getElementById('relFmProtection')?.checked,
	          include_human_operation: true,
	          include_scheduled: false,
	          only_in_service: true,
	        },
	        max_order: (document.getElementById('relFmN2')?.checked ? 2 : 1),
	        reporting: { metric_focus: metricFocus, weak_basis: weakBasis },
	      };
	      const data = await apiPost('/api/session/run_reliability', opts);
	      if (data && !data.error) {
	        data._metric_focus = metricFocus;
	        data._weak_basis = weakBasis;
	        _lastReliabilityData = Object.assign({ _method: method }, data);
	        showReliabilityResults(data, method);
        switchTab('results');
        setStatus('可靠性分析完成');
      } else {
        setStatus('计算失败', 'error');
	      }
	    }

	    function relMetricValue(data, key) {
	      const metrics = data && data.metrics ? data.metrics : null;
	      if (metrics && Object.prototype.hasOwnProperty.call(metrics, key)) return metrics[key];
	      return data ? data[key] : undefined;
	    }

	    function relMetricHtml(value, digits = 2, scale = 1) {
	      if (value && typeof value === 'object' && value.available === false) {
	        return `<span title="${escapeHtml(value.reason || 'not available')}">—</span>`;
	      }
	      const raw = value && typeof value === 'object' && Object.prototype.hasOwnProperty.call(value, 'value')
	        ? value.value : value;
	      if (raw === null || raw === undefined) return '—';
	      const n = Number(raw);
	      if (Number.isFinite(n)) return (n * scale).toFixed(digits);
	      return escapeHtml(String(raw));
	    }

	    function renderReliabilityPanel(title, inner) {
	      return `<div style="border:1px solid var(--border,#3a3f4b);border-radius:6px;padding:8px 10px;margin:10px 0;">` +
	             `<h5 style="margin:0 0 6px;">${escapeHtml(title)}</h5>${inner}</div>`;
	    }

		    function relNumber(value) {
		      if (value && typeof value === 'object' && value.available === false) return null;
		      if (value && typeof value === 'object' && Object.prototype.hasOwnProperty.call(value, 'value')) value = value.value;
		      const n = Number(value);
		      return Number.isFinite(n) ? n : null;
		    }

		    function relSelectedMetricFocus(data) {
		      return data?._metric_focus || document.getElementById('relMetricFocus')?.value || 'all';
		    }

		    function relSelectedWeakBasis(data, method) {
		      const selected = data?._weak_basis || document.getElementById('relWeakBasis')?.value || 'auto';
		      if (selected && selected !== 'auto') return selected;
		      if (method === 'three_stage') return 'eens';
		      if (method === 'nsq' || method === 'seq') return 'eens';
		      if (method === 'fd') return 'frequency';
		      return 'eens';
		    }

		    function relWeakBasisMeta(basis) {
		      const metas = {
		        eens: { label: 'EENS 贡献', unit: 'MWh/yr', digits: 3, axis: 'MWh/yr',
		          help: '按期望未供电量排序，适合找能量风险最大的元件或失效模式。' },
		        lole: { label: 'LOLE 贡献', unit: 'h/yr', digits: 3, axis: 'h/yr',
		          help: '按失负荷持续时间贡献排序，适合找停电影响时间最长的薄弱环节。' },
		        frequency: { label: '故障频率', unit: 'occ/yr', digits: 4, axis: 'occ/yr',
		          help: '按导致失负荷的年发生频率排序，适合关注频繁动作或频繁停运模式。' },
		        conditional: { label: '条件风险', unit: '', digits: 4, axis: '',
		          help: '按蒙特卡洛 P(元件停运 | 系统失负荷) 或风险占比排序，适合解释抽样状态中的关联风险。' },
		        stage_shed: { label: '阶段切负荷', unit: 'kW', digits: 1, axis: 'kW',
		          help: '按三阶段恢复中的阶段切负荷总量排序，适合诊断隔离、重构和修复窗口的故障传播影响。' },
		      };
		      return metas[basis] || metas.eens;
		    }

		    function relMetricFocusLabel(focus) {
		      const labels = {
		        all: '全部指标',
		        eens: 'EENS/EDNS',
		        lole: 'LOLE/LOLP',
		        customer: 'SAIFI/SAIDI',
		        cost: '成本',
		      };
		      return labels[focus] || focus || '全部指标';
		    }

		    function relMetricField(row, basis) {
		      const firstNumber = (...values) => {
		        for (const value of values) {
		          const n = relNumber(value);
		          if (n !== null) return n;
		        }
		        return 0;
		      };
		      if (basis === 'lole') {
		        return firstNumber(row?.lole_contribution_hr_yr, row?.lole_contribution);
		      }
		      if (basis === 'frequency') {
		        return firstNumber(row?.lolf_contribution_occ_yr, row?.lolf_contribution,
		          row?.frequency_per_year, row?.failure_rate, row?.joint_frequency_per_year);
		      }
		      if (basis === 'conditional') {
		        return firstNumber(row?.conditional_down_given_loss, row?.loss_weighted_risk,
		          row?.importance);
		      }
		      if (basis === 'stage_shed') {
		        return firstNumber(row?.pls_total, row?.shed_kw,
		          relNumber(row?.shed_mw ?? row?.total_shed_mw) !== null
		            ? relNumber(row?.shed_mw ?? row?.total_shed_mw) * 1000.0
		            : null);
		      }
		      return firstNumber(row?.associated_eens_mwh_yr, row?.eens_contribution_mwh_yr,
		        row?.eens_contribution);
		    }

		    function relCandidateRows(data) {
		      if (Array.isArray(data?.critical_components) && data.critical_components.length) return data.critical_components;
		      if (Array.isArray(data?.contingencies) && data.contingencies.length) return data.contingencies;
		      if (Array.isArray(data?.faults) && data.faults.length) return data.faults;
		      return [];
		    }

		    function relRankRows(rows, basis) {
		      return (rows || []).slice()
		        .map(row => ({ row, score: relMetricField(row, basis) }))
		        .filter(item => Number.isFinite(item.score) && item.score > 0)
		        .sort((a, b) => b.score - a.score)
		        .map(item => item.row);
		    }

		    function relFocusMetricKeys(focus) {
		      const groups = {
		        all: ['eens_mwh_yr', 'edns_mw', 'lole_hr_yr', 'lolf_occ_yr', 'lolp', 'plc', 'saifi', 'saidi', 'asai', 'eens_cost'],
		        eens: ['eens_mwh_yr', 'eens_kwh_yr', 'edns_mw'],
		        lole: ['lole_hr_yr', 'lolf_occ_yr', 'lolp', 'lold_hr', 'plc'],
		        customer: ['saifi', 'saidi', 'saidi_min', 'caidi', 'asai'],
		        cost: ['eens_cost', 'eens_mwh_yr'],
		      };
		      return new Set(groups[focus] || groups.all);
		    }

		    function renderReliabilityKpiTiles(data, method) {
		      const focus = relSelectedMetricFocus(data);
		      const allowed = relFocusMetricKeys(focus);
		      const allItems = [
		        ['EENS', 'eens_mwh_yr', 'MWh/yr', 1],
		        ['EDNS', 'edns_mw', 'MW', 3],
		        ['LOLE', 'lole_hr_yr', 'h/yr', 2],
		        ['LOLF', 'lolf_occ_yr', 'occ/yr', 3],
		        ['LOLP', 'lolp', '', 6],
		        ['SAIFI', 'saifi', 'int/cust/yr', 3],
		        ['SAIDI', 'saidi', 'h/cust/yr', 3],
		        ['ASAI', 'asai', '', 6],
		        ['PLC', 'plc', '%', 3, 100],
		        ['EENS成本', 'eens_cost', '', 1],
		      ];
		      const items = allItems
		        .filter(([, key]) => allowed.has(key))
		        .map(([label, key, unit, digits, scale]) => [label, relMetricValue(data, key), unit, digits, scale])
		        .filter(x => relNumber(x[1]) !== null);
		      if (!items.length) return '';
		      let html = '<div class="reliability-kpi-grid">';
	      items.forEach(([label, value, unit, digits, scale]) => {
	        html += `<div class="reliability-kpi-card">` +
	                `<div class="reliability-kpi-label">${escapeHtml(label)}</div>` +
	                `<div class="reliability-kpi-value">${relMetricHtml(value, digits, scale || 1)}</div>` +
	                `<div class="reliability-kpi-unit">${escapeHtml(unit)}</div></div>`;
	      });
	      html += '</div>';
	      return html;
	    }

		    function renderReliabilityDashboardShell(data, method) {
		      const hasRisk = (Array.isArray(data.critical_components) && data.critical_components.length) ||
		                      (Array.isArray(data.contingencies) && data.contingencies.length) ||
		                      (Array.isArray(data.faults) && data.faults.length);
	      const hasNodal = Array.isArray(data.nodal_eens_mwh_yr) && data.nodal_eens_mwh_yr.some(v => Number(v) > 0);
	      const hasCoverage = !!data.failure_mode_coverage;
	      const hasConv = (method === 'nsq' || method === 'seq') && Array.isArray(data.eens_history) && data.eens_history.length;
	      if (!hasRisk && !hasNodal && !hasCoverage && !hasConv) return '';
	      let html = '<div class="reliability-dashboard-grid">';
	      if (hasRisk) html += '<div id="relRiskChart" class="reliability-chart reliability-chart-wide"></div>';
	      if (hasNodal) html += '<div id="relNodalChart" class="reliability-chart"></div>';
	      if (hasCoverage) html += '<div id="relFailureTaxonomyChart" class="reliability-chart"></div>';
	      if (hasConv) html += '<div id="relConvChart" class="reliability-chart"></div>';
	      html += '</div>';
	      return renderReliabilityPanel('可靠性风险仪表盘', html);
	    }

	    function relPlotLayout(title, xTitle, yTitle) {
	      return {
	        title: { text: title, font: { size: 13 }, x: 0.02, xanchor: 'left' },
	        margin: { l: 58, r: 18, t: 38, b: 70 },
	        xaxis: { title: xTitle || '', tickangle: -30, automargin: true },
	        yaxis: { title: yTitle || '', automargin: true },
	        paper_bgcolor: 'rgba(0,0,0,0)',
	        plot_bgcolor: 'rgba(0,0,0,0)',
	        font: { color: '#dcdfe4' }
	      };
	    }

		    function drawReliabilityDashboard(data, method) {
		      if (typeof Plotly === 'undefined') return;
		      const relMaps = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;
		      const basis = relSelectedWeakBasis(data, method);
		      const basisMeta = relWeakBasisMeta(basis);
		      const riskRows = relRankRows(relCandidateRows(data), basis).slice(0, 12).map(r => ({
		        name: r.display_name || r.component_name || r.mode_id || `Line ${r.line_id ?? '—'}`,
		        y: relMetricField(r, basis),
		        pct: Number(r.loss_weighted_risk ?? r.importance ?? 0) || 0,
		        compId: reliabilityContingencyCompId(r, relMaps),
		      }));
		      const riskChart = document.getElementById('relRiskChart');
		      if (riskChart && riskRows.length) {
		        Plotly.newPlot(riskChart, [{
	          x: riskRows.map(r => r.y).reverse(),
	          y: riskRows.map(r => r.name).reverse(),
	          type: 'bar',
	          orientation: 'h',
		          marker: { color: riskRows.map((_, i) => i < 3 ? '#ff6b6b' : '#61afef').reverse() },
		          customdata: riskRows.map(r => [r.pct, basisMeta.label, basisMeta.unit]).reverse(),
		          hovertemplate: '%{y}<br>%{customdata[1]}=%{x:.3f} %{customdata[2]}<br>风险占比=%{customdata[0]:.3f}<extra></extra>'
		        }], {
		          ...relPlotLayout(`Top 风险元件/失效模式 (${basisMeta.label})`, basisMeta.axis, ''),
		          margin: { l: 92, r: 18, t: 38, b: 48 },
	          yaxis: { automargin: true }
	        }, { responsive: true });
	        if (riskChart.on) {
	          riskChart.on('plotly_click', ev => {
	            const idx = Number(ev?.points?.[0]?.pointIndex);
	            const compId = Number.isInteger(idx) ? riskRows[riskRows.length - 1 - idx]?.compId : undefined;
	            if (compId != null && typeof Canvas !== 'undefined' && Canvas.panToComponent) {
	              Canvas.panToComponent(compId);
	            }
	          });
	        }
	      }
	      const nodal = (data.nodal_eens_mwh_yr || [])
	        .map((v, i) => {
	          const bus = i + 1;
	          const compId = relMaps ? (relMaps.ac?.[bus] ?? relMaps.dc?.[bus]) : undefined;
	          return { bus, value: Number(v) || 0, compId };
	        })
	        .filter(r => r.value > 0)
	        .sort((a, b) => b.value - a.value)
	        .slice(0, 20);
	      const nodalChart = document.getElementById('relNodalChart');
	      if (nodalChart && nodal.length) {
	        Plotly.newPlot(nodalChart, [{
	          x: nodal.map(r => r.value).reverse(),
	          y: nodal.map(r => `Bus ${r.bus}`).reverse(),
	          type: 'bar',
	          orientation: 'h',
	          marker: { color: '#ff9f43' },
	          hovertemplate: '%{y}<br>节点EENS=%{x:.3f} MWh/yr<extra></extra>'
	        }], {
	          ...relPlotLayout('负荷点影响', 'MWh/yr', ''),
	          margin: { l: 74, r: 18, t: 38, b: 48 },
	          yaxis: { automargin: true }
	        }, { responsive: true });
	        if (nodalChart.on) {
	          nodalChart.on('plotly_click', ev => {
	            const idx = Number(ev?.points?.[0]?.pointIndex);
	            const compId = Number.isInteger(idx) ? nodal[nodal.length - 1 - idx]?.compId : undefined;
	            if (compId != null && typeof Canvas !== 'undefined' && Canvas.panToComponent) {
	              Canvas.panToComponent(compId);
	            }
	          });
	        }
	      }
	      const cov = data.failure_mode_coverage;
	      if (document.getElementById('relFailureTaxonomyChart') && cov) {
	        Plotly.newPlot('relFailureTaxonomyChart', [{
	          labels: ['被动时基', '主动按需', '物理设备', '网络/控制', '保护逻辑', '不支持'],
	          values: [cov.modes_passive, cov.modes_active, cov.modes_physical, cov.modes_cyber_control, cov.modes_protection_logic, cov.modes_unsupported],
	          type: 'pie',
	          hole: 0.45,
	          marker: { colors: ['#61afef', '#f5c542', '#98c379', '#c678dd', '#e06c75', '#5c6370'] }
	        }], { ...relPlotLayout('失效模式分类', '', ''), margin: { l: 10, r: 10, t: 38, b: 10 } }, { responsive: true });
	      }
	      if (document.getElementById('relConvChart') && Array.isArray(data.eens_history) && data.eens_history.length) {
	        const x = data.eens_history.map((_, i) => i + 1);
	        Plotly.newPlot('relConvChart', [{ x, y: data.eens_history, mode: 'lines', name: 'EENS', line: { color: '#61afef' } }],
	          relPlotLayout('EENS 收敛过程', '样本批次', 'MWh/yr'), { responsive: true });
	      }
	    }

      function relScopeBadge(text, ok) {
	      const bg = (ok === true) ? '#2e7d32' : (ok === false) ? '#b03a3a' : '#4a4f59';
	      return `<span style="display:inline-block;padding:2px 8px;margin:2px 4px 2px 0;border-radius:10px;font-size:11px;background:${bg};color:#fff;">${escapeHtml(text)}</span>`;
	    }
	    function renderRelScopeHtml(data) {
	      if (!data || !data.model_scope) return '';
	      let h = '';
	      h += `<b>有效分析：</b>${relScopeBadge(data.model_scope)}`;
	      if (data.physical_model) h += ` <b>物理模型：</b>${relScopeBadge(data.physical_model)}`;
	      if (data.data_policy) h += ` <b>数据策略：</b>${relScopeBadge(data.data_policy)}`;
	      if (data.parallel_mode) {
	        const pe = data.parallel_execution || {};
	        let ptxt = `${data.parallel_effective ? '有效' : '串行'} ${data.parallel_workers || 1}线程`;
	        if (pe.work_items != null) ptxt += ` / ${pe.work_items}项`;
	        if (pe.guard_reason) ptxt += ` / ${pe.guard_reason}`;
	        h += ` <b>并行：</b>${relScopeBadge(ptxt, !!data.parallel_effective)}`;
	      }
	      const v = data.validity || {};
	      const vkeys = Object.keys(v);
      if (vkeys.length) {
        h += '<br><b>有效性：</b>';
        if ('dc_load_curtailment_included' in v) {
          h += relScopeBadge('DC负荷' + (v.dc_load_curtailment_included ? '已计入' : '未计入'), v.dc_load_curtailment_included);
          h += relScopeBadge('VSC直流潮流' + (v.vsc_dc_power_flow_modelled ? '已建模' : '未建模'), v.vsc_dc_power_flow_modelled);
          h += relScopeBadge('AC电压/无功' + (v.ac_voltage_reactive_feasibility_certified ? '已认证' : '未认证'), v.ac_voltage_reactive_feasibility_certified);
        } else {
          vkeys.forEach(k => { if (typeof v[k] === 'boolean') h += relScopeBadge(k.replace(/_/g, ' '), v[k]); });
        }
      }
      if (v.generation_adequacy_only) h += '<br>' + relScopeBadge('仅发电充裕度 — 无网络/DC/用户指标', false);
	      const dq = data.data_quality;
	      if (dq) {
	        h += `<br><b>数据质量：</b>${dq.components_with_reliability_data}/${dq.components_total} 含用例数据，${dq.components_defaulted} 默认值`;
	        if (dq.missing_required_data && dq.missing_required_data.length) h += ' ' + relScopeBadge(dq.missing_required_data.length + ' 缺失', false);
	      }
	      if (data.model_limitations) h += `<div style="color:var(--muted,#8a909c);font-size:11px;margin-top:6px;">${escapeHtml(data.model_limitations)}</div>`;
	      return renderReliabilityPanel('有效分析方法', h);
	    }

		    function renderReliabilityMetricsHtml(data, method) {
		      const focus = relSelectedMetricFocus(data);
		      const allowed = relFocusMetricKeys(focus);
		      const rows = [
		        ['EENS (MWh/yr)', 'eens_mwh_yr', 2, 1, 'eens'],
		        ['EENS (kWh/yr)', 'eens_kwh_yr', 1, 1, 'eens'],
		        ['EDNS (MW)', 'edns_mw', 3, 1, 'eens'],
		        ['LOLE (h/yr)', 'lole_hr_yr', 2, 1, 'lole'],
		        ['LOLF (occ/yr)', 'lolf_occ_yr', 3, 1, 'lole'],
		        ['LOLP', 'lolp', 6, 1, 'lole'],
		        ['LOLD (h/occ)', 'lold_hr', 2, 1, 'lole'],
		        ['PLC (%)', 'plc', 3, 100, 'lole'],
		        ['SAIFI', 'saifi', 4, 1, 'customer'],
		        ['SAIDI (h/yr)', 'saidi', 4, 1, 'customer'],
		        ['SAIDI (min/yr)', 'saidi_min', 2, 1, 'customer'],
		        ['CAIDI', 'caidi', 4, 1, 'customer'],
		        ['ASAI', 'asai', 6, 1, 'customer'],
		        ['EENS 成本', 'eens_cost', 1, 1, 'cost'],
		      ];
		      let html = '<table><thead><tr><th>指标</th><th>数值</th></tr></thead><tbody>';
		      rows.forEach(([label, key, digits, scale]) => {
		        if (focus !== 'all' && !allowed.has(key)) return;
		        const value = relMetricValue(data, key);
		        if (value === undefined && !(data.metrics && Object.prototype.hasOwnProperty.call(data.metrics, key))) return;
		        html += `<tr><td>${escapeHtml(label)}</td><td class="result-value">${relMetricHtml(value, digits, scale || 1)}</td></tr>`;
	      });
		      if (method === 'nsq' || method === 'seq') {
		        html += `<tr><td>收敛</td><td>${data.converged ? '是' : '否'}</td></tr>`;
		        html += `<tr><td>迭代/年数</td><td>${data.iterations_used ?? '—'}</td></tr>`;
		        html += `<tr><td>最终 CoV</td><td>${relMetricHtml(data.final_cov, 4)}</td></tr>`;
		      }
		      if (data.parallel_mode || data.parallel_workers != null) {
		        const pEff = data.parallel_effective ? '有效' : (data.parallel ? '未触发' : '关闭');
		        const pWorkers = Number(data.parallel_workers || 1);
		        const pe = data.parallel_execution || {};
		        const pExtra = pe.work_items != null
		          ? ` · ${escapeHtml(String(pe.work_items))}项 · HW ${escapeHtml(String(pe.hardware_threads || '—'))}`
		            + (pe.actual_parallel_evaluations != null ? ` · 并行评估 ${escapeHtml(String(pe.actual_parallel_evaluations))}` : '')
		            + (pe.guard_reason ? ` · ${escapeHtml(pe.guard_reason)}` : '')
		          : '';
		        html += `<tr><td>并行执行</td><td>${escapeHtml(pEff)} · ${escapeHtml(String(pWorkers))} 线程 · ${escapeHtml(data.parallel_mode || 'serial')}${pExtra}</td></tr>`;
		      }
		      if (data.n_contingencies != null) {
		        html += `<tr><td>枚举项</td><td>${data.n_contingencies}${data.n_with_loss != null ? ` (${data.n_with_loss} 含失负荷)` : ''}</td></tr>`;
		      }
		      html += '</tbody></table>';
		      if (data.metric_semantics?.description) {
		        html += `<div style="color:var(--muted,#8a909c);font-size:11px;margin-top:6px;">${escapeHtml(data.metric_semantics.description)}</div>`;
		      }
		      return renderReliabilityPanel('指标', html);
		    }

	    function renderFailureModeCoverageHtml(data) {
	      const cov = data.failure_mode_coverage;
	      if (!cov) return '';
	      const rows = [
	        ['元件数', cov.components_total],
	        ['模式总数', cov.modes_total],
	        ['启用/停用/不支持', `${cov.modes_enabled}/${cov.modes_disabled}/${cov.modes_unsupported}`],
	        ['被动/主动', `${cov.modes_passive}/${cov.modes_active}`],
	        ['物理/网络控制/保护', `${cov.modes_physical}/${cov.modes_cyber_control}/${cov.modes_protection_logic}`],
	        ['用例/模板/缺失数据', `${cov.modes_case_data}/${cov.modes_template_or_default}/${cov.modes_missing_data}`],
	      ];
	      let html = '<table><tbody>';
	      rows.forEach(([k, v]) => { html += `<tr><td>${escapeHtml(k)}</td><td>${escapeHtml(String(v ?? '—'))}</td></tr>`; });
	      html += '</tbody></table>';
	      return renderReliabilityPanel('失效模式覆盖', html);
	    }

		    function renderFailureModeLegendHtml(data) {
		      const html = '<div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:8px;font-size:12px;">' +
		        '<div><b>时基失效</b><br><span style="color:var(--muted,#8a909c);">用年故障率 lambda 和修复时间 r；不可用度 U=lambda*r/(H+lambda*r)，确定性贡献为 lambda × 持续时间 × 后果。</span></div>' +
		        '<div><b>按需动作失效</b><br><span style="color:var(--muted,#8a909c);">只在开断、保护或控制动作被请求时暴露；等效年频率为 demand frequency × probability per demand。</span></div>' +
		        '<div><b>设备物理</b><br><span style="color:var(--muted,#8a909c);">硬件或一次设备故障，通常改变拓扑、容量、连通性或设备可用性。</span></div>' +
		        '<div><b>网络控制</b><br><span style="color:var(--muted,#8a909c);">通信、测量、控制器或设定值异常，代表 cyber/control 层失效；恢复时间可独立于物理 MTTR。</span></div>' +
		        '<div><b>保护逻辑</b><br><span style="color:var(--muted,#8a909c);">保护拒动、误动或保护区扩大，影响隔离范围和故障传播路径。</span></div>' +
		        '<div><b>N-2 共因</b><br><span style="color:var(--muted,#8a909c);">当前按独立重叠 U_i × U_j 近似联合不可用度；用于筛查二阶薄弱组合。</span></div>' +
		        '</div>';
		      return renderReliabilityPanel('故障模式计算方法', html);
		    }

	    function renderReliabilityComponentModelHtml(data) {
	      const dq = data.data_quality;
	      const cov = data.failure_mode_coverage;
	      let html = '';
	      if (dq) {
	        html += '<table><tbody>';
	        html += `<tr><td>可靠性元件</td><td>${dq.components_total ?? 0}</td></tr>`;
	        html += `<tr><td>用例含参</td><td>${dq.components_with_reliability_data ?? 0}</td></tr>`;
	        html += `<tr><td>模板/默认补全</td><td>${dq.components_defaulted ?? 0}</td></tr>`;
	        html += `<tr><td>缺失必需数据</td><td>${(dq.missing_required_data || []).length}</td></tr>`;
	        html += '</tbody></table>';
	      }
	      if (cov) {
	        html += `<div style="color:var(--muted,#8a909c);font-size:11px;margin-top:6px;">组件已展开为 ${cov.modes_total} 个失效模式，其中 ${cov.modes_active} 个主动模式、${cov.modes_passive} 个被动模式。</div>`;
	      }
	      return html ? renderReliabilityPanel('元件建模', html) : '';
	    }

		    function renderReliabilitySelectionHtml(data, method) {
		      const focus = relSelectedMetricFocus(data);
		      const basis = relSelectedWeakBasis(data, method);
		      const meta = relWeakBasisMeta(basis);
		      let html = `<b>指标聚焦：</b>${escapeHtml(relMetricFocusLabel(focus))} `;
		      html += `<b style="margin-left:10px;">薄弱环节排序：</b>${escapeHtml(meta.label)}`;
		      html += `<div style="color:var(--muted,#8a909c);font-size:11px;margin-top:6px;">${escapeHtml(meta.help)}</div>`;
		      return renderReliabilityPanel('统计指标与薄弱环节', html);
		    }

		    function renderReliabilityRiskBasisHtml(method, basis) {
		      const meta = relWeakBasisMeta(basis || 'eens');
		      let prefix = `<div style="color:var(--muted,#8a909c);font-size:11px;margin:4px 0 8px;">当前按 ${escapeHtml(meta.label)} 排序。${escapeHtml(meta.help)} `;
		      if (method === 'nsq' || method === 'seq') {
		        return prefix +
		          '蒙特卡洛薄弱元件反映抽样共停运状态中的关联风险；FMEA 按单一故障或失效模式的频率×后果排序，因此同一算例排序可以不同。</div>';
		      }
		      if (method === 'fmea' || method === 'failure_mode_fmea') {
		        return prefix + 'FMEA 为确定性枚举，薄弱环节来自单一故障或失效模式的频率加权后果。</div>';
		      }
		      if (method === 'three_stage') {
		        return prefix + '三阶段恢复为确定性频率加权评估，阶段1/2/3分别对应隔离、重构和修复窗口。</div>';
		      }
		      return prefix + '</div>';
		    }

		    function showReliabilityResults(data, method) {
		      document.getElementById('resultsEmpty').style.display = 'none';
		      document.getElementById('resultsContent').style.display = 'block';
		      setActiveResultGroup('reliability');
		      const nf = (v, d = 2) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(d) : '—';
		      const methodLabel = { nsq: '非序贯蒙特卡洛', seq: '序贯蒙特卡洛', fmea: 'FMEA (N-1)', failure_mode_fmea: '失效模式 FMEA', fd: '频率-持续时间', three_stage: '三阶段恢复重构' }[method] || method;
		      const weakBasis = relSelectedWeakBasis(data, method);
		      const weakMeta = relWeakBasisMeta(weakBasis);
		      const weakRows = relRankRows(relCandidateRows(data), weakBasis);
		      let html = `<div style="margin-bottom:8px;"><b>方法：</b>${escapeHtml(methodLabel)}</div>`;
		      html += renderReliabilityKpiTiles(data, method);
		      html += renderReliabilityDashboardShell(data, method);
		      html += renderReliabilityComponentModelHtml(data);
		      html += renderFailureModeCoverageHtml(data);
		      html += renderFailureModeLegendHtml(data);
		      html += renderReliabilitySelectionHtml(data, method);
		      html += renderRelScopeHtml(data);
		      html += renderReliabilityMetricsHtml(data, method);
		      if (weakRows.length) {
		        html += `<h4 style="margin:10px 0 4px;">薄弱环节 (按 ${escapeHtml(weakMeta.label)})</h4>`;
		        html += renderReliabilityRiskBasisHtml(method, weakBasis);
		        html += `<table><thead><tr><th>#</th><th>元件/模式</th><th>类型</th><th>${escapeHtml(weakMeta.label)}</th><th>风险占比</th><th>关联EENS</th><th>LOLE</th></tr></thead><tbody>`;
		        const ccBusMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;
		        weakRows.slice(0, 15).forEach((c, i) => {
		          const compId = reliabilityContingencyCompId(c, ccBusMap);
		          const clk = compClickAttr(compId);
		          const fallbackName = c.line_id != null
		            ? `故障 ${c.line_id} (${c.from_bus ?? '—'} -> ${c.to_bus ?? '—'})`
		            : `${c.component_type || ''}[${c.index ?? c.component_index ?? ''}]`;
		          const name = c.display_name || c.component_name || c.mode_id || fallbackName;
		          const type = c.display_type || reliabilityComponentTypeLabel(c.component_type || c.canonical_component_type);
		          html += `<tr${clk}><td>${i + 1}</td><td>${escapeHtml(name)}</td><td>${escapeHtml(type)}</td><td>${nf(relMetricField(c, weakBasis), weakMeta.digits)} ${escapeHtml(weakMeta.unit)}</td><td>${nf(c.loss_weighted_risk ?? c.importance, 4)}</td><td>${nf(c.associated_eens_mwh_yr ?? c.eens_contribution, 3)}</td><td>${nf(c.lole_contribution_hr_yr ?? c.lole_contribution, 3)}</td></tr>`;
		        });
		        html += '</tbody></table>';
		      } else if (relCandidateRows(data).length) {
		        html += renderReliabilityPanel('薄弱环节', `<div style="color:var(--muted,#8a909c);font-size:12px;">当前排序依据“${escapeHtml(weakMeta.label)}”在该方法结果中没有可用的正贡献值；请切换薄弱环节排序依据或查看下方原始结果表。</div>`);
		      }
		      // FMEA top contingencies
		      if ((method === 'fmea' || method === 'failure_mode_fmea') && Array.isArray(data.contingencies) && data.contingencies.length) {
		        const contingencyRows = relRankRows(data.contingencies, weakBasis);
		        html += `<h4 style="margin:10px 0 4px;">关键故障/失效模式 (按 ${escapeHtml(weakMeta.label)})</h4>`;
		        html += renderReliabilityRiskBasisHtml(method, weakBasis);
		        if (contingencyRows.length) {
		          html += '<table><thead><tr><th>元件/模式</th><th>类型</th><th>激活</th><th>原因</th><th>后果</th><th>状态</th><th>EENS贡献</th><th>切负荷</th></tr></thead><tbody>';
		          const relBusMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;
		          contingencyRows.slice(0, 15).forEach(c => {
		            const compId = reliabilityContingencyCompId(c, relBusMap);
		            const clk = compClickAttr(compId);
		            const name = c.display_name || c.mode_id || c.component_name || '—';
		            const type = c.display_type || reliabilityComponentTypeLabel(c.component_type);
		            const supported = c.supported === false ? (c.unsupported_reason || '不支持') : (c.data_source || '已评估');
		            html += `<tr${clk}><td>${escapeHtml(name)}</td><td>${escapeHtml(type)}</td><td>${escapeHtml(reliabilityFmLabel(c.activation))}</td><td>${escapeHtml(reliabilityFmLabel(c.cause))}</td><td>${escapeHtml(reliabilityFmLabel(c.consequence))}</td><td>${escapeHtml(supported)}</td><td>${nf(c.eens_contribution, 2)}</td><td>${nf(c.shed_mw ?? c.total_shed_mw, 2)}</td></tr>`;
		          });
		          html += '</tbody></table>';
		        } else {
		          html += `<div style="color:var(--muted,#8a909c);font-size:12px;">该 FMEA 结果没有“${escapeHtml(weakMeta.label)}”排序值。</div>`;
		        }
		      }
	      // Multi-mode (N-2) co-failures (failure-mode FMEA with max_order>=2)
	      if (Array.isArray(data.co_contingencies) && data.co_contingencies.length) {
	        html += '<h4 style="margin:10px 0 4px;">共因失效 (N-2 联合停运，按 EENS 贡献)</h4>';
	        html += '<table><thead><tr><th>模式 A</th><th>模式 B</th><th>联合不可用度</th><th>切负荷(MW)</th><th>EENS贡献</th></tr></thead><tbody>';
	        data.co_contingencies.slice(0, 15).forEach(co => {
	          const a = co.mode_a || {}, b = co.mode_b || {};
	          const na2 = (a.display_name || a.mode_id || '—') + ' · ' + reliabilityFmLabel(a.consequence);
	          const nb2 = (b.display_name || b.mode_id || '—') + ' · ' + reliabilityFmLabel(b.consequence);
	          const uij = (co.joint_unavailability != null) ? Number(co.joint_unavailability).toExponential(2) : '—';
	          html += `<tr><td>${escapeHtml(na2)}</td><td>${escapeHtml(nb2)}</td><td>${uij}</td><td>${nf(co.total_shed_mw, 2)}</td><td>${nf(co.eens_contribution, 3)}</td></tr>`;
	        });
	        html += '</tbody></table>';
	      }
	      // Three-stage restoration: per-fault load shed by stage
	      if (method === 'three_stage' && Array.isArray(data.faults) && data.faults.length) {
	        const faultRows = relRankRows(data.faults, weakBasis);
	        html += '<h4 style="margin:10px 0 4px;">三阶段故障传播与恢复</h4><table><thead><tr><th>元件</th><th>类型</th><th>状态</th><th>阶段1</th><th>阶段2</th><th>阶段3</th><th>合计(kW)</th><th>EENS</th><th>LOLE</th></tr></thead><tbody>';
	        const tsMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;
	        faultRows.slice(0, 15).forEach(f => {
	          const compId = reliabilityContingencyCompId(f, tsMap);
	          const clk = compClickAttr(compId);
	          const name = f.display_name || f.component_name || `故障 ${f.line_id} (${f.from_bus ?? '—'} -> ${f.to_bus ?? '—'})`;
	          const type = f.display_type || reliabilityComponentTypeLabel(f.component_type);
	          html += `<tr${clk}><td>${escapeHtml(name)}</td><td>${escapeHtml(type)}</td><td>${escapeHtml(f.status || '')}</td><td>${nf(f.pls_stage1, 1)}</td><td>${nf(f.pls_stage2, 1)}</td><td>${nf(f.pls_stage3, 1)}</td><td>${nf(f.pls_total, 1)}</td><td>${nf(f.eens_contribution_mwh_yr ?? f.eens_contribution, 3)}</td><td>${nf(f.lole_contribution_hr_yr ?? f.lole_contribution, 3)}</td></tr>`;
	        });
        html += '</tbody></table>';
      }
	      document.getElementById('reliabilityResults').innerHTML = html;
	      drawReliabilityDashboard(data, method);
	      if (typeof Canvas !== 'undefined' && Canvas.showReliabilityImpactResults) {
	        Canvas.showReliabilityImpactResults(data);
	      }
	    }

	    document.getElementById('relPhysicalModel')?.addEventListener('change', updateReliabilityControlState);
	    updateReliabilityControlState();
	    document.getElementById('btnRunReliability')?.addEventListener('click', runReliability);
    document.getElementById('btnExportReliabilityResults')?.addEventListener('click', () => {
      if (!_lastReliabilityData) {
        log('暂无可靠性分析结果可导出，请先运行可靠性分析', 'warn');
        return;
      }
      downloadJsonFile(`reliability_results_${tsTagForFilename()}.json`, _lastReliabilityData);
    });

    // ---- Carbon emission analysis ----
    // Backend reuses the cached PF result if one exists, otherwise runs a fresh
    // AC Newton power flow. Request body is empty; the server operates on the
    // currently loaded system (synced from the canvas below).
    async function runCarbonAnalysis() {
      setStatus('碳排放分析中...', 'busy');
      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
      const data = await apiPost('/api/session/run_carbon', {});
      if (data && !data.error) {
        _lastCarbonData = data;
        showCarbonResults(data);
        switchTab('results');
        setStatus('碳排放分析完成');
      } else {
        setStatus('碳排放分析失败', 'error');
      }
    }

    function showCarbonResults(data) {
      document.getElementById('resultsEmpty').style.display = 'none';
      document.getElementById('resultsContent').style.display = 'block';
      setActiveResultGroup('carbonAnalysis');
      const nf = (v, d = 3) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(d) : '—';
      const cbBusMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;

      // Method / source header
      let html = `<div style="margin-bottom:8px;">`
        + `<b>潮流来源：</b>${data.pf_source || '—'} &nbsp; `
        + `<b>矩阵法：</b>${data.matrix_solved ? '✓ 求解' : '✗ 未解'} &nbsp; `
        + `<b>溯源校验：</b>${data.tracing_verified ? '✓ 通过' : '✗ 未通过'} &nbsp; `
        + `<b>矩阵残差：</b>${nf(data.matrix_residual, 2)}</div>`;

      // Emissions summary — proportional tracing vs matrix method
      const ts = data.tracing_summary || {}, ms = data.matrix_summary || {};
      html += '<h4 style="margin:6px 0 4px;">碳排放总量 (tCO₂)</h4>';
      html += '<table><thead><tr><th>指标</th><th>比例溯源法</th><th>矩阵法</th></tr></thead><tbody>';
      const rows = [
        ['发电侧排放', 'total_generation_emissions_tco2'],
        ['负荷侧排放', 'total_load_emissions_tco2'],
        ['网损排放', 'total_loss_emissions_tco2'],
        ['平衡误差 (%)', 'balance_error_pct'],
      ];
      for (const [label, key] of rows) {
        html += `<tr><td>${label}</td><td class="result-value">${nf(ts[key])}</td><td class="result-value">${nf(ms[key])}</td></tr>`;
      }
      html += '</tbody></table>';

      // Load carbon — sorted by total emissions, top 20
      const loads = (data.load_carbon || []).slice().sort((a, b) => (b.total_emissions_tco2 || 0) - (a.total_emissions_tco2 || 0));
      if (loads.length) {
        html += '<h4 style="margin:10px 0 4px;">负荷碳排放 (Top 20)</h4>';
        html += '<table><thead><tr><th>负荷</th><th>母线</th><th>需求(MW)</th><th>碳强度(tCO₂/MWh)</th><th>排放(tCO₂)</th></tr></thead><tbody>';
        loads.slice(0, 20).forEach(l => {
          html += `<tr${busClickAttr(l.bus, cbBusMap)}><td>${escapeHtml(carbonLoadLabel(l))}</td><td>${l.bus ?? '—'}</td><td>${nf(positiveDemandMw(l), 3)}</td>`
            + `<td>${nf(l.carbon_intensity_tco2_mwh, 4)}</td><td class="result-value">${nf(l.total_emissions_tco2)}</td></tr>`;
        });
        html += '</tbody></table>';
      }

      // Bus carbon intensity — AC, sorted by intensity, top 20
      const buses = (data.bus_carbon || []).slice().sort((a, b) => (b.carbon_intensity_tco2_mwh || 0) - (a.carbon_intensity_tco2_mwh || 0));
      if (buses.length) {
        html += '<h4 style="margin:10px 0 4px;">母线碳强度 (AC, Top 20)</h4>';
        html += '<table><thead><tr><th>母线</th><th>碳强度(tCO₂/MWh)</th></tr></thead><tbody>';
        buses.slice(0, 20).forEach(b => {
          html += `<tr${busClickAttr(b.bus_index, cbBusMap)}><td>${b.bus_index ?? '—'}</td><td class="result-value">${nf(b.carbon_intensity_tco2_mwh, 4)}</td></tr>`;
        });
        html += '</tbody></table>';
      }

      // Branch loss carbon — sorted by emissions, top 15
      const branches = (data.branch_carbon || []).slice().sort((a, b) => (b.total_emissions_tco2 || 0) - (a.total_emissions_tco2 || 0));
      if (branches.length) {
        html += '<h4 style="margin:10px 0 4px;">支路网损碳排放 (Top 15)</h4>';
        html += '<table><thead><tr><th>支路</th><th>从→到</th><th>损耗(MW)</th><th>排放(tCO₂)</th></tr></thead><tbody>';
        branches.slice(0, 15).forEach(b => {
          html += `<tr${busClickAttr(b.from_bus, cbBusMap)}><td>${b.branch_index ?? '—'}</td><td>${b.from_bus}→${b.to_bus}</td>`
            + `<td>${nf(b.loss_mw, 4)}</td><td class="result-value">${nf(b.total_emissions_tco2)}</td></tr>`;
        });
        html += '</tbody></table>';
      }

      // VSC converter carbon
      const vscs = data.vsc_carbon || [];
      if (vscs.length) {
        html += '<h4 style="margin:10px 0 4px;">换流器 (VSC) 损耗碳排放</h4>';
        html += '<table><thead><tr><th>换流器</th><th>AC母线</th><th>DC母线</th><th>损耗(MW)</th><th>排放(tCO₂)</th></tr></thead><tbody>';
        vscs.forEach(v => {
          html += `<tr${busClickAttr(v.bus_ac, cbBusMap)}><td>${v.converter_index ?? '—'}</td><td>${v.bus_ac ?? '—'}</td><td>${v.bus_dc ?? '—'}</td>`
            + `<td>${nf(v.loss_mw, 4)}</td><td class="result-value">${nf(v.total_emissions_tco2)}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      document.getElementById('carbonResults').innerHTML = html;

      // Carbon flow Sankey: sources (generators/storage) → loads
      const chartDiv = document.getElementById('carbonSankeyChart');
      const labels = data.sankey_labels || [], srcs = data.sankey_sources || [],
            tgts = data.sankey_targets || [], vals = data.sankey_values || [];
      if (typeof Plotly !== 'undefined' && chartDiv && labels.length && srcs.length) {
        Plotly.react(chartDiv, [{
          type: 'sankey',
          orientation: 'h',
          node: { label: labels, pad: 12, thickness: 14,
                  line: { color: '#3e4451', width: 0.5 },
                  color: '#61afef' },
          link: { source: srcs, target: tgts, value: vals,
                  color: 'rgba(97,175,239,0.25)' },
        }], {
          paper_bgcolor: 'rgba(0,0,0,0)', font: { color: '#abb2bf', size: 11 },
          margin: { l: 10, r: 10, t: 20, b: 10 }, title: '碳流溯源 (MW)',
        }, { responsive: true });
      } else if (chartDiv) {
        chartDiv.innerHTML = '<p class="empty-hint">无碳流数据可视化</p>';
      }
    }

    document.getElementById('btnRunCarbon')?.addEventListener('click', runCarbonAnalysis);
    document.getElementById('btnExportCarbonResults')?.addEventListener('click', () => {
      if (!_lastCarbonData) {
        log('暂无碳排放分析结果可导出，请先运行碳排放分析', 'warn');
        return;
      }
      downloadJsonFile(`carbon_results_${tsTagForFilename()}.json`, _lastCarbonData);
    });

    // Bar 3: Resilience — collect AC/DC inline parameters into a typed payload.
    function collectResilienceParams() {
      const num = (id, dflt) => {
        const v = parseFloat(document.getElementById(id)?.value);
        return Number.isFinite(v) ? v : dflt;
      };
      const parseIntList = (id) => {
        const raw = (document.getElementById(id)?.value || '').trim();
        return raw ? raw.split(/[,，\s]+/).map(s => parseInt(s, 10)).filter(n => Number.isFinite(n) && n >= 0) : [];
      };
      const parseNumList = (id) => {
        const raw = (document.getElementById(id)?.value || '').trim();
        return raw ? raw.split(/[,，\s]+/).map(s => Number(s)).filter(Number.isFinite) : [];
      };
      const atOr = (arr, idx, dflt) => arr.length ? (Number.isFinite(arr[Math.min(idx, arr.length - 1)]) ? arr[Math.min(idx, arr.length - 1)] : dflt) : dflt;
      const acIds = parseIntList('resAcFaultLocations');
      const dcIds = parseIntList('resDcFaultLocations');
      const acStarts = parseNumList('resAcFaultStartHour');
      const dcStarts = parseNumList('resDcFaultStartHour');
      const acRepairs = parseNumList('resAcRepairDuration');
      const dcRepairs = parseNumList('resDcRepairDuration');
      const resilienceModel = document.getElementById('resModelSelect')?.value || 'RAStyleStageMILP';
      const resilienceSolver = document.getElementById('resSolverSelect')?.value || 'Gurobi';
      const considerSwitches = !!document.getElementById('resConsiderSwitches')?.checked;
      const allowMess = !!document.getElementById('resAllowMess')?.checked;
      const allowBranchWithoutSwitch = !!document.getElementById('resAllowBranchWithoutSwitch')?.checked;
      const manual_faults = [];
      acIds.forEach((id, i) => manual_faults.push({ branch_type: 'AC', branch_id: id, start_hr: atOr(acStarts, i, 0), repair_hr: atOr(acRepairs, i, 6), label: `AC branch ${id}` }));
      dcIds.forEach((id, i) => manual_faults.push({ branch_type: 'DC', branch_id: id, start_hr: atOr(dcStarts, i, 0), repair_hr: atOr(dcRepairs, i, 8), label: `DC branch ${id}` }));
      const latestEnd = manual_faults.reduce((m, f) => Math.max(m, Number(f.start_hr || 0) + Number(f.repair_hr || 0)), 0);
      const requestedHorizon = Math.max(1, Math.ceil(num('resHorizonHours', 48)));
      const postFaultWindow = Math.max(0, num('resPostFaultWindow', 2.0));
      return {
        fault_count:     num('resFaultCount', manual_faults.length || 1),
        ac_fault_branch_ids: acIds,
        dc_fault_branch_ids: dcIds,
        manual_faults,
        load_scale_factor: Math.max(0, num('resLoadScale', 1.0)),
        mobile_storage_speed_kmh: num('resMobileSpeed', 40),
        allow_mess_dispatch: allowMess,
        apply_demo_data: document.getElementById('resApplyDemoData')?.checked !== false,
        mip_time_limit_s: Math.max(10, Math.round(num('resMipTimeLimit', 180))),
        mip_gap: Math.max(0, num('resMipGap', 0.03)),
        resilience_model: resilienceModel,
        resilience_solver: resilienceSolver,
        consider_switches: considerSwitches,
        use_remote_switch_only: !!document.getElementById('resUseRemoteSwitchOnly')?.checked,
        allow_branch_operation_without_switch: allowBranchWithoutSwitch,
        post_fault_reconfig_window_hr: postFaultWindow,
        horizon_hours: Math.max(requestedHorizon, Math.ceil(latestEnd + 4)),
      };
    }
    document.getElementById('btnGenExtremeScenario')?.addEventListener('click', async () => {
      const intensity = document.getElementById('resTyphoonIntensity')?.value || 'TY';
      setStatus('正在按台风强度生成台风故障序列...', 'busy');
      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
      const data = await apiPost('/api/session/generate_typhoon_faults', {
        intensity_category: intensity,
        horizon_hours: 48,
        time_step_hr: 1.0,
        catalog_samples_per_month: 100,
      });
      if (!data) { setStatus('台风故障生成失败', 'error'); return; }
      const rawFaults = Array.isArray(data.manual_faults) && data.manual_faults.length
        ? data.manual_faults
        : (Array.isArray(data.fault_locations_typed) ? data.fault_locations_typed : []);
      const acFaults = [], dcFaults = [];
      rawFaults.forEach(f => {
        const type = String(f.branch_type || f.branch_kind || 'AC').toUpperCase();
        const entry = { id: f.branch_index ?? f.branch_id ?? f.branch, start: Number(f.start_hr ?? f.outage_start_hr ?? 0), repair: Number(f.repair_hr ?? f.repair_duration_hr ?? 6) };
        if (!Number.isFinite(Number(entry.id))) return;
        if (type === 'DC') dcFaults.push(entry); else acFaults.push(entry);
      });
      if (!acFaults.length && Array.isArray(data.fault_locations)) {
        data.fault_locations.forEach(id => acFaults.push({ id, start: 0, repair: 6 }));
      }
      const setVal = (id, v) => { const el = document.getElementById(id); if (el) el.value = v; };
      setVal('resAcFaultLocations', acFaults.map(f => f.id).join(','));
      setVal('resDcFaultLocations', dcFaults.map(f => f.id).join(','));
      setVal('resFaultCount', String(acFaults.length + dcFaults.length));
      markResilienceFaultBranches();
      setVal('resAcFaultStartHour', acFaults.map(f => Number.isFinite(f.start) ? f.start : 0).join(','));
      setVal('resDcFaultStartHour', dcFaults.map(f => Number.isFinite(f.start) ? f.start : 0).join(','));
      setVal('resAcRepairDuration', acFaults.map(f => Number.isFinite(f.repair) ? f.repair : 6).join(','));
      setVal('resDcRepairDuration', dcFaults.map(f => Number.isFinite(f.repair) ? f.repair : 8).join(','));
      const requestedLabel = data.requested_intensity_category_zh || data.requested_intensity_category || intensity;
      const selectedLabel = data.selected_intensity_category_zh || data.selected_intensity_category || requestedLabel;
      const vmax = Number(data.selected_track_max_vmax_ms);
      const vmaxText = Number.isFinite(vmax) ? `，轨迹最大风速 ${vmax.toFixed(2)} m/s` : '';
      const fallbackText = data.used_category_fallback ? `；请求等级 ${requestedLabel} 样本不足，已回退为 ${selectedLabel}` : '';
      log(`${selectedLabel}台风场景已生成：AC 故障 ${acFaults.length} 个，DC 故障 ${dcFaults.length} 个，已填入弹性评估输入${vmaxText}${fallbackText}。${data.status || ''}`, 'success');
      setStatus('台风故障序列已填入');
    });
    async function runResilience() {
      setStatus('弹性评估中...', 'busy');
      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
      // Resilience assessment always uses imported scenario 48h profiles when available.
      const resScenarioCase = getImportedGeneratedScenarioCase('resilience');
      if (hasUsableGeneratedScenarioTimeSeries('resilience')) {
        try { await applyGeneratedScenarioTimeSeries(resScenarioCase); }
        catch (e) { log(`应用生成场景时序失败：${e.message || e}`, 'warn'); }
      }
      if (!validateFaultBranchIds()) return;
      markResilienceFaultBranches();
      const p = collectResilienceParams();
      const isRaStageModel = p.resilience_model === 'RAStyleStageMILP';
      const scenarioProfiles = getResilienceScenarioProfiles();
      const params = {
        model: p.resilience_model,
        resilience_model: p.resilience_model,
        mip_solver: p.resilience_solver,
        resilience_solver: p.resilience_solver,
        default_fault_count: p.fault_count,
        manual_faults: p.manual_faults,
        ac_fault_branch_ids: p.ac_fault_branch_ids,
        dc_fault_branch_ids: p.dc_fault_branch_ids,
        mess_travel_speed_kmph: p.mobile_storage_speed_kmh,
        horizon_hours: p.horizon_hours,
        time_step_hr: 1.0,
        load_scale_factor: p.load_scale_factor,
        apply_demo_data: p.apply_demo_data,
        mip_time_limit_s: p.mip_time_limit_s,
        mip_gap: p.mip_gap,
        allow_reconfiguration: true,
        allow_mess_dispatch: p.allow_mess_dispatch,
        run_power_flow: false,
        consider_switches: p.consider_switches,
        enable_disaster_stages: isRaStageModel && p.consider_switches,
        use_ra_style_stage_milp: isRaStageModel,
        use_strict_mip_for_mess: isRaStageModel && p.allow_mess_dispatch,
        fallback_to_stage_mess_dispatch: true,
        allow_stage1_open_switches: isRaStageModel && p.consider_switches,
        allow_stage2_close_ties: isRaStageModel && p.consider_switches,
        require_switch_for_nonfault_branch_operation: isRaStageModel && p.consider_switches && !p.allow_branch_operation_without_switch,
        allow_branch_operation_without_switch: p.allow_branch_operation_without_switch || !p.consider_switches,
        use_remote_switch_only: p.use_remote_switch_only,
        post_fault_reconfig_window_hr: p.post_fault_reconfig_window_hr,
        disaster_post_fault_reconfig_window_hr: p.post_fault_reconfig_window_hr,
      };
      if (Array.isArray(scenarioProfiles.loadProfile) && scenarioProfiles.loadProfile.length) {
        params.load_profile = scenarioProfiles.loadProfile;
      }
      if (Array.isArray(scenarioProfiles.tsProfiles) && scenarioProfiles.tsProfiles.length) {
        params.scenario_profiles = scenarioProfiles.tsProfiles;
      }
      if (Array.isArray(scenarioProfiles.loadProfileMap) && scenarioProfiles.loadProfileMap.length) {
        params.load_profile_map = scenarioProfiles.loadProfileMap;
      }
      if (Array.isArray(scenarioProfiles.renewableProfile) && scenarioProfiles.renewableProfile.length) {
        params.renewable_profile = scenarioProfiles.renewableProfile;
      }
      if (Array.isArray(scenarioProfiles.pvProfile) && scenarioProfiles.pvProfile.length) {
        params.pv_profile = scenarioProfiles.pvProfile;
      }
      if (Array.isArray(scenarioProfiles.windProfile) && scenarioProfiles.windProfile.length) {
        params.wind_profile = scenarioProfiles.windProfile;
      }
      const data = await apiPost('/api/session/run_distribution_resilience', params);
      if (data && !data.error) {
        _lastResilienceData = data;
        renderComponentCurveTargets('resilience');
        showResilienceResults(data);
        switchTab('results');
        setStatus('弹性评估完成');
      } else {
        setStatus('计算失败', 'error');
      }
    }

    function plotThemeRes(title, yTitle = '') {
      const axisTheme = { gridcolor: '#d1d5db', linecolor: '#111827', tickcolor: '#111827', zerolinecolor: '#9ca3af' };
      return {
        title,
        margin: { l: 55, r: 15, t: 35, b: 45 },
        xaxis: { title: '时间 (h)', ...axisTheme },
        yaxis: { title: yTitle, ...axisTheme, rangemode: 'tozero' },
        legend: { orientation: 'h', y: -0.25 },
        paper_bgcolor: '#ffffff',
        plot_bgcolor: '#ffffff',
        font: { color: '#111827', size: 11 },
      };
    }

    function arrayCounts(series) {
      return Array.isArray(series) ? series.map(v => Array.isArray(v) ? v.length : 0) : [];
    }

    function renderResilienceCharts(data) {
      if (typeof Plotly === 'undefined' || !Array.isArray(data.hours) || !data.hours.length) return;
      const x = data.hours;
      const cfg = { responsive: true, displaylogo: false };
      const asArray = (v) => Array.isArray(v) ? v : [];
      const seriesAtLength = (series, fill = 0) => x.map((_, i) => {
        const v = Array.isArray(series) ? series[i] : undefined;
        return v === undefined || v === null ? fill : v;
      });
      const numberSeries = (series, fill = 0) => seriesAtLength(series, fill).map(v => {
        const n = Number(v);
        return Number.isFinite(n) ? n : fill;
      });
      const stagesRaw = asArray(data.disaster_stages);
      const stageSeries = x.map((_, i) => stagesRaw[i] || 'Normal');
      const branchStateCounts = (series) => {
        const arr = asArray(series);
        return x.map((_, i) => Array.isArray(arr[i]) ? arr[i].length : 0);
      };
      const chartLayout = (title, yTitle = '') => ({
        ...plotThemeRes(title, yTitle),
        autosize: true,
        margin: { l: 55, r: 15, t: 32, b: 38 },
        legend: { orientation: 'h', y: -0.18 },
      });

      Plotly.newPlot('resTimeChart', [
        ...(Array.isArray(data.demand_mw) ? [{ x, y: numberSeries(data.demand_mw), mode: 'lines+markers', name: '需求', line: { color: '#61afef' } }] : []),
        ...(Array.isArray(data.served_mw) ? [{ x, y: numberSeries(data.served_mw), mode: 'lines+markers', name: '供电', line: { color: '#98c379' } }] : []),
        ...(Array.isArray(data.shed_mw) ? [{ x, y: numberSeries(data.shed_mw), mode: 'lines+markers', name: '切负荷', line: { color: '#e06c75' } }] : []),
      ], chartLayout('恢复过程：需求/供电/切负荷', 'MW'), cfg);

      Plotly.newPlot('resRestorationChart', [
        { x, y: numberSeries(data.restoration_ratio).map(v => v * 100), mode: 'lines+markers', name: '供电率', line: { color: '#0f766e', width: 2 } },
      ], { ...chartLayout('供电率时序', '%'), yaxis: { title: '%', gridcolor: '#d1d5db', linecolor: '#111827', tickcolor: '#111827', zerolinecolor: '#9ca3af', range: [0, 105] } }, cfg);

      Plotly.newPlot('resFaultSwitchChart', [
        { x, y: numberSeries(data.active_faults), type: 'bar', name: '活动故障', marker: { color: '#f59e0b' } },
        { x, y: numberSeries(data.repaired_faults_arr), type: 'bar', name: '已修复', marker: { color: '#22c55e' } },
        { x, y: numberSeries(data.switch_actions), mode: 'lines+markers', name: '开关动作', yaxis: 'y2', line: { color: '#a78bfa' } },
      ], { ...chartLayout('故障 / 修复 / 开关动作', '数量'), barmode: 'group', yaxis2: { title: '动作数', overlaying: 'y', side: 'right', gridcolor: '#d1d5db', linecolor: '#111827', tickcolor: '#111827', zerolinecolor: '#9ca3af' } }, cfg);

      Plotly.newPlot('resPriorityChart', [
        { x, y: numberSeries(data.shed_critical), type: 'bar', name: '关键', marker: { color: '#dc2626' } },
        { x, y: numberSeries(data.shed_high), type: 'bar', name: '高', marker: { color: '#f59e0b' } },
        { x, y: numberSeries(data.shed_medium), type: 'bar', name: '中', marker: { color: '#3b82f6' } },
        { x, y: numberSeries(data.shed_low), type: 'bar', name: '低', marker: { color: '#94a3b8' } },
      ], { ...chartLayout('分优先级切负荷', 'MW'), barmode: 'stack' }, cfg);

      const stageMap = { Normal: 0, DisasterIsolation: 1, DisasterPostFaultReconfig: 2, PostDisasterRepair: 3 };
      const stageNames = ['预先准备', '抵御与吸收', '响应与适应', '快速恢复'];
      Plotly.newPlot('resStageChart', [
        { x, y: stageSeries.map(s => stageMap[s] ?? 0), mode: 'lines+markers', name: '灾害阶段', line: { color: '#56b6c2', shape: 'hv' }, text: stageSeries.map(s => stageNames[stageMap[s] ?? 0]), hovertemplate: '%{x} h<br>%{text}<extra></extra>' },
      ], { ...chartLayout('灾害阶段时间线', '阶段'), yaxis: { title: '阶段', gridcolor: '#d1d5db', linecolor: '#111827', tickcolor: '#111827', zerolinecolor: '#9ca3af', tickmode: 'array', tickvals: [0,1,2,3], ticktext: stageNames } }, cfg);

      const mess = asArray(data.mess_traces);
      const messTraces = [];
      mess.forEach((tr, idx) => {
        const name = tr.name || `MESS ${tr.storage_index ?? idx + 1}`;
        if (Array.isArray(tr.dispatch_mw)) {
          messTraces.push({ x, y: tr.dispatch_mw, mode: 'lines+markers', name: `${name} 充放电功率`, yaxis: 'y' });
        }
        if (Array.isArray(tr.energy_mwh)) {
          messTraces.push({ x, y: tr.energy_mwh, mode: 'lines+markers', name: `${name} 剩余电量`, yaxis: 'y2', line: { dash: 'dot' } });
        }
      });
      const messLayout = {
        ...chartLayout('移动储能：充放电功率 / 剩余电量', '功率 (MW)'),
        yaxis2: { title: '剩余电量 (MWh)', overlaying: 'y', side: 'right', gridcolor: '#d1d5db', linecolor: '#111827', tickcolor: '#111827', zerolinecolor: '#9ca3af', rangemode: 'tozero' },
        annotations: messTraces.length ? [] : [{ text: '无移动储能出力/电量数据', xref: 'paper', yref: 'paper', x: 0.5, y: 0.5, showarrow: false, font: { color: '#111827' } }],
      };
      Plotly.newPlot('resMessChart', messTraces, messLayout, cfg);

      Plotly.newPlot('resOpenBranchChart', [
        { x, y: branchStateCounts(data.open_ac_branch_ids), type: 'bar', name: 'AC当前断开支路', marker: { color: '#e06c75' } },
        { x, y: branchStateCounts(data.open_dc_branch_ids), type: 'bar', name: 'DC当前断开支路', marker: { color: '#c678dd' } },
        { x, y: branchStateCounts(data.closed_tie_branch_ids), type: 'bar', name: 'AC闭合联络', marker: { color: '#98c379' } },
        { x, y: branchStateCounts(data.closed_dc_tie_branch_ids), type: 'bar', name: 'DC闭合联络', marker: { color: '#56b6c2' } },
      ], { ...chartLayout('AC/DC 拓扑状态统计', '数量'), barmode: 'group' }, cfg);
    }

    function resilienceModelLabel(model) {
      const key = String(model || '');
      if (key === 'RAStyleStageMILP') return 'RA阶段MILP';
      if (key === 'MultiPeriodMIPLinDistFlow') return '多时段MIP LinDistFlow';
      if (key === 'HeuristicSequential') return '启发式序贯';
      return key || '—';
    }

    function messDispatchModelLabel(model) {
      const key = String(model || '');
      if (key === 'ra_residual_mess_milp') return 'RA残余切负荷移储MILP';
      if (key === 'strict_mip') return 'Strict MIP';
      if (key === 'stage_dispatch_fallback') return '阶段局部移储调度';
      if (key === 'disabled') return '禁用';
      if (key === 'none') return '无';
      return key || '—';
    }

    function summarizeMessTrace(trace) {
      const nums = arr => Array.isArray(arr) ? arr.map(Number).filter(Number.isFinite) : [];
      const unique = arr => Array.from(new Set((arr || []).filter(v => v !== null && v !== undefined && String(v) !== '')));
      const buses = nums(trace?.bus);
      const targets = nums(trace?.target_bus).filter(v => v >= 0);
      const statuses = unique(trace?.status || []);
      const remaining = nums(trace?.remaining_travel_hr);
      const dispatch = nums(trace?.dispatch_mw);
      const startBus = buses.length ? buses[0] : null;
      const finalBus = buses.length ? buses[buses.length - 1] : null;
      const busChanged = buses.some(v => v !== startBus);
      const targetChanged = targets.length > 0 && targets.some(v => v !== startBus && v !== finalBus);
      const inTransit = statuses.some(s => /transit|行驶|途中/i.test(String(s)));
      const moved = busChanged || targetChanged || inTransit;
      const targetSummary = unique(targets).slice(0, 8).join(' → ') || '—';
      const statusSummary = statuses.slice(0, 6).join(' / ') || '—';
      const maxRemain = remaining.length ? Math.max(...remaining) : null;
      const maxDispatch = dispatch.length ? Math.max(...dispatch.map(v => Math.abs(v))) : null;
      const totalAbsDispatch = dispatch.reduce((a, b) => a + Math.abs(b), 0);
      return { startBus, finalBus, targetSummary, statusSummary, maxRemain, moved, maxDispatch, totalAbsDispatch };
    }

    function renderMessTrajectoryTable(data, nf) {
      const mess = Array.isArray(data?.mess_traces) ? data.mess_traces : [];
      if (!mess.length) return '<p class="empty-hint compact-hint">无移动储能轨迹数据</p>';
      let html = '<h4 style="margin:8px 0 4px;">移动储能路径/轨迹证据</h4><table><thead><tr><th>MESS</th><th>起始母线</th><th>最终母线</th><th>目标母线</th><th>状态</th><th>最大剩余行驶(h)</th><th>是否移动</th><th>最大|出力|(MW)</th></tr></thead><tbody>';
      mess.forEach((tr, idx) => {
        const s = summarizeMessTrace(tr);
        const name = tr.name || `MESS ${tr.storage_index ?? idx + 1}`;
        html += `<tr><td>${escapeHtml(String(name))}</td><td>${s.startBus ?? '—'}</td><td>${s.finalBus ?? '—'}</td><td>${escapeHtml(String(s.targetSummary))}</td><td>${escapeHtml(String(s.statusSummary))}</td><td>${nf(s.maxRemain, 1)}</td><td>${s.moved ? '是' : '否'}</td><td>${nf(s.maxDispatch, 2)}</td></tr>`;
      });
      html += '</tbody></table>';
      return html;
    }

    function updateResilienceComparisonRuns(data) {
      const ms = data.model_stats || {};
      _resilienceComparisonRuns.unshift({
        time: new Date().toLocaleTimeString('zh-CN', { hour12: false }),
        model: data.effective_model || data.model,
        solver: data.effective_solver || ms.solver_name || data.requested_solver,
        status: data.status || ms.solver_status || '',
        feasible: data.feasible !== false,
        resilience_index: data.resilience_index,
        total_shed_mwh: data.total_shed_mwh,
        weighted_unserved_mwh: data.weighted_unserved_mwh,
        mess_energy_delivered_mwh: data.mess_energy_delivered_mwh,
        mess_travel_distance_km: data.mess_travel_distance_km,
        objective_value: ms.objective_value,
        mip_gap: ms.mip_gap,
        runtime_sec: ms.runtime_sec,
      });
      _resilienceComparisonRuns = _resilienceComparisonRuns.slice(0, 8);
    }

    function renderResilienceComparisonTable(nf) {
      if (!_resilienceComparisonRuns.length) return '';
      let html = '<h4 style="margin:8px 0 4px;">最近弹性评估对比</h4><table><thead><tr><th>时间</th><th>模型</th><th>求解器</th><th>可行</th><th>弹性指数</th><th>切负荷(MWh)</th><th>移储供能</th><th>移储行程</th><th>目标值</th><th>Gap</th><th>运行(s)</th></tr></thead><tbody>';
      _resilienceComparisonRuns.forEach(r => {
        html += `<tr><td>${r.time}</td><td>${resilienceModelLabel(r.model)}</td><td>${escapeHtml(String(r.solver || '—'))}</td><td>${r.feasible ? '✓' : '✗'}</td><td>${nf(r.resilience_index, 4)}</td><td>${nf(r.total_shed_mwh, 2)}</td><td>${nf(r.mess_energy_delivered_mwh, 2)}</td><td>${nf(r.mess_travel_distance_km, 1)}</td><td>${nf(r.objective_value, 2)}</td><td>${nf(r.mip_gap, 4)}</td><td>${nf(r.runtime_sec, 2)}</td></tr>`;
      });
      html += '</tbody></table>';
      return html;
    }

    function showResilienceResults(data) {
      document.getElementById('resultsEmpty').style.display = 'none';
      document.getElementById('resultsContent').style.display = 'block';
      setActiveResultGroup('resilience');
      const nf = (v, d = 2) => {
        const n = Number(v);
        return Number.isFinite(n) ? n.toFixed(d) : '—';
      };
      updateResilienceComparisonRuns(data);
      const ms = data.model_stats || {};
      const eqCount = Number(ms.num_eq_constraints);
      const ineqCount = Number(ms.num_ineq_constraints);
      const constraintCount = (Number.isFinite(eqCount) || Number.isFinite(ineqCount))
        ? `${Number.isFinite(eqCount) ? eqCount : 0} / ${Number.isFinite(ineqCount) ? ineqCount : 0}` : '—';
      const resilienceIndexTip = '弹性指数 = 评估时段内总供电电量 / 总需求电量，取值 0–1；越接近 1 表示灾害期间整体供电保持得越好。它是全时段能量积分指标；最低供电率是单个最差时刻指标。';
      const supplyRatios = Array.isArray(data.restoration_ratio) ? data.restoration_ratio.map(Number).filter(Number.isFinite) : [];
      const minSupplyRatio = supplyRatios.length ? Math.min(...supplyRatios) : (data.avg_restoration_ratio ?? 0);
      const kpis = [
        ['评估模型', resilienceModelLabel(data.effective_model || data.model)],
        ['请求求解器', data.requested_solver || '—'],
        ['实际求解器', data.effective_solver || ms.solver_name || '—'],
        ['求解状态', ms.solver_status || data.status || '—'],
        ['变量 / 二进制变量', `${ms.num_variables ?? '—'} / ${ms.num_binary_variables ?? '—'}`],
        ['等式 / 不等式约束', constraintCount],
        ['目标值', nf(ms.objective_value, 3)],
        ['MIP Gap', nf(ms.mip_gap, 4)],
        ['运行时间 (s)', nf(ms.runtime_sec, 2)],
        ['移储调度模型', messDispatchModelLabel(data.mess_dispatch_model)],
        ['可行', data.feasible === false ? '✗ 否' : '✓ 是'],
        ['状态', data.status ?? '—'],
        ['弹性指数', nf(data.resilience_index, 4), resilienceIndexTip],
        ['总需求 (MWh)', nf(data.total_demand_mwh, 2)],
        ['总切负荷 (MWh)', nf(data.total_shed_mwh, 2)],
        ['加权未供 (MWh)', nf(data.weighted_unserved_mwh, 2)],
        ['峰值切负荷 (MW)', nf(data.peak_shed_mw, 2)],
        ['最终供电率 (%)', nf((data.final_restoration_ratio ?? 0) * 100, 2)],
        ['最低供电率 (%)', nf(minSupplyRatio * 100, 2)],
        ['移储供能 (MWh)', nf(data.mess_energy_delivered_mwh, 2)],
        ['移储行程 (km)', nf(data.mess_travel_distance_km, 1)],
        ['开关操作次数', data.total_switch_actions ?? '—'],
        ['修复故障数', data.total_repaired_faults ?? '—'],
      ];
      let summaryHtml = '<table><thead><tr><th>指标</th><th>数值</th></tr></thead><tbody>';
      for (const [k, v, tip] of kpis) summaryHtml += `<tr><td${tip ? ` title="${escapeHtml(tip)}" class="metric-help"` : ''}>${k}</td><td class="result-value">${v}</td></tr>`;
      summaryHtml += '</tbody></table>';

      let detailHtml = '';
      detailHtml += renderMessTrajectoryTable(data, nf);
      detailHtml += renderResilienceComparisonTable(nf);
      if (Array.isArray(data.fault_sequence) && data.fault_sequence.length) {
        detailHtml += '<h4 style="margin:0 0 4px;">故障序列</h4><table><thead><tr><th>#</th><th>类型</th><th>支路</th><th>开始(h)</th><th>修复(h)</th></tr></thead><tbody>';
        const resBusMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;
        data.fault_sequence.slice(0, 20).forEach((f, i) => {
          const clk = busClickAttr(busIdFromComponentName(f.name), resBusMap);
          detailHtml += `<tr${clk}><td>${i + 1}</td><td>${f.branch_type || f.branch_kind || 'AC'}</td><td>${f.name ?? f.branch_id ?? f.branch_index ?? f.branch ?? '—'}</td><td>${nf(f.start_hr, 1)}</td><td>${nf(f.repair_hr ?? f.repair_time_hr, 1)}</td></tr>`;
        });
        detailHtml += '</tbody></table>';
      }
      let chartsHtml = '';
      if (Array.isArray(data.hours) && data.hours.length) {
        chartsHtml = `
          <div id="resChartsGrid" class="resilience-charts-grid">
            <div id="resTimeChart" class="resilience-chart wide"></div>
            <div id="resRestorationChart" class="resilience-chart"></div>
            <div id="resFaultSwitchChart" class="resilience-chart"></div>
            <div id="resPriorityChart" class="resilience-chart"></div>
            <div id="resStageChart" class="resilience-chart"></div>
            <div id="resMessChart" class="resilience-chart"></div>
            <div id="resOpenBranchChart" class="resilience-chart"></div>
          </div>`;
      }
      document.getElementById('resilienceResults').innerHTML = `
        <div class="resilience-result-layout">
          <div class="resilience-chart-pane">${chartsHtml || '<p class="empty-hint">暂无时序图表</p>'}</div>
          <div class="resilience-detail-pane">
            <h4 style="margin:0 0 4px;">评估指标</h4>
            <div class="resilience-detail-grid">
              <div>${summaryHtml}</div>
              <div>${detailHtml || '<p class="empty-hint compact-hint">暂无故障序列明细</p>'}</div>
            </div>
          </div>
        </div>`;

      renderResilienceCharts(data);
    }

    document.getElementById('resConsiderSwitches')?.addEventListener('change', e => { e.currentTarget.dataset.userTouched = '1'; });
    document.getElementById('btnRunResilience')?.addEventListener('click', runResilience);

    // ===== Scenario generation module (场景生成) — inner-scope functions + listeners =====
    function scenNum(id, dflt) {
      const value = Number(document.getElementById(id)?.value);
      return Number.isFinite(value) ? value : dflt;
    }

    function scenChecked(id, dflt = false) {
      const el = document.getElementById(id);
      return el ? el.checked === true : dflt;
    }

    function collectScenarioGenerationOptions() {
      const regularClusters = Math.max(1, Math.round(scenNum('scenRegularClusters', 8)));
      const reliabilityClusters = Math.max(1, Math.round(scenNum('scenReliabilityClusters', 4)));
      const resilienceClusters = Math.max(1, Math.round(scenNum('scenResilienceClusters', 5)));
      const intensityLevels = ['TD', 'TS', 'STS', 'TY', 'STY', 'SuperTY'];
      const sspLevels = ['ssp126', 'ssp245', 'ssp370', 'ssp585'];
      const years = [2050, 2080];
      return {
        regular: {
          enabled: scenChecked('scenRegularEnabled', true),
          candidate_count: regularClusters * 10 * sspLevels.length * years.length,
          cluster_count: regularClusters,
          num_steps: 8760,
          ssp: 'all',
          year: 0,
          ssp_levels: sspLevels,
          years,
        },
        reliability: {
          enabled: scenChecked('scenReliabilityEnabled', true),
          candidates_per_contingency: reliabilityClusters * 10,
          cluster_count_per_contingency: reliabilityClusters,
          num_steps: 1,
          include_ac_branches: true,
          include_dc_branches: true,
          include_ac_buses: false,
          include_dc_buses: false,
          include_vsc_converters: true,
          include_dcdc_converters: false,
          include_generators: true,
          include_loads: false,
          include_storage: true,
          include_vpps: false,
          include_microgrids: false,
          max_contingencies: 0,
        },
        resilience: {
          enabled: scenChecked('scenResilienceEnabled', true),
          intensity_levels: intensityLevels,
          candidates_per_intensity: resilienceClusters * 10,
          default_cluster_count: resilienceClusters,
          cluster_count_by_intensity: Object.fromEntries(intensityLevels.map(level => [level, resilienceClusters])),
          num_steps: 48,
        },
        perturbation: {
          seed: 1,
          load_sigma: 0.08,
          renewable_sigma: 0.12,
          load_min_multiplier: 0.75,
          load_max_multiplier: 1.25,
          renewable_min_multiplier: 0.0,
          renewable_max_multiplier: 1.20,
          enable_storage_soc_perturbation: true,
          storage_soc_sigma: 0.10,
          storage_soc_min_multiplier: 0.75,
          storage_soc_max_multiplier: 1.25,
          block_hours: 24,
          enable_load_perturbation: true,
          enable_renewable_perturbation: true,
        },
        typhoon_impact: {
          pv_transition_hours: 3,
          load_wind_start: 25.0,
          load_wind_full: 50.0,
          load_max_reduction: 0.4,
        },
        clustering: {
          method: document.getElementById('scenClusteringMethod')?.value || 'hybrid_kmedoids_tail_5pct',
          max_iterations: 50,
          robust_scale: true,
          continuous_weight: 1.0,
          outage_hamming_weight: 1.0,
          include_tail_anchors: true,
          compare_baseline: scenChecked('scenCompareBaseline', true),
          freeze_anchors: true,
          tail_fraction: 0.05,
          source_tail_quantile: 0.95,
          coupling_quantile: 0.90,
          boundary_fraction: 0.02,
          regime_weight: 1.0,
        },
      };
    }

    function firstProfileSummary(cluster, profileName) {
      const profiles = cluster?.representative?.time_series?.profiles || [];
      const profile = profiles.find(p => p.name === profileName);
      const values = Array.isArray(profile?.values) ? profile.values : [];
      if (!values.length) return '-';
      const sum = values.reduce((a, b) => a + Number(b || 0), 0);
      const max = values.reduce((a, b) => Math.max(a, Number(b || 0)), -Infinity);
      return `${sum.toFixed(2)} / 峰值 ${Number.isFinite(max) ? max.toFixed(2) : '-'}`;
    }

    function scenarioCandidateProfileValues(candidate, profileName) {
      const profiles = candidate?.time_series?.profiles || [];
      const profile = profiles.find(p => p.name === profileName);
      return Array.isArray(profile?.values) ? profile.values.map(v => Number(v || 0)) : [];
    }

    function numericFeature(cluster, key) {
      const value = cluster?.representative?.features?.[key];
      return Number.isFinite(Number(value)) ? Number(value) : 0;
    }

    function eventMaxWind(event) {
      const explicit = Number(event?.selected_track_max_vmax_ms);
      if (Number.isFinite(explicit) && explicit > 0) return explicit;
      const track = Array.isArray(event?.track) ? event.track : [];
      const start = track.length > 1 ? 1 : 0;
      let vmax = 0;
      for (let i = start; i < track.length; i += 1) {
        const v = Number(track[i]?.vmax_ms);
        if (Number.isFinite(v)) vmax = Math.max(vmax, v);
      }
      return vmax;
    }

    function parseRegularId(id) {
      const parts = String(id || '').split(':');
      return { ssp: parts[1] || 'unknown', year: Number(parts[2]) || 0 };
    }

    function clamp01(value) {
      const v = Number(value);
      if (!Number.isFinite(v)) return 0;
      return Math.max(0, Math.min(1, v));
    }

    function finiteNumber(value, fallback = NaN) {
      const v = Number(value);
      return Number.isFinite(v) ? v : fallback;
    }

    function coverageKey(value) {
      if (value === undefined || value === null || value === '') return null;
      return String(value);
    }

    function valueSetCoverage(selectedValues, universeValues, expectedValues = null) {
      const universeSet = new Set((Array.isArray(expectedValues) ? expectedValues : universeValues)
        .map(coverageKey)
        .filter(v => v !== null));
      const selectedSet = new Set(selectedValues
        .map(coverageKey)
        .filter(v => v !== null && universeSet.has(v)));
      const total = universeSet.size;
      return { value: total > 0 ? selectedSet.size / total : 0, selectedCount: selectedSet.size, universeCount: total };
    }

    function distinctCoverage(selectedItems, universeItems, keyFn, expectedValues = null) {
      const selectedValues = (selectedItems || []).map(item => keyFn(item));
      const universeValues = (universeItems || []).map(item => keyFn(item));
      return valueSetCoverage(selectedValues, universeValues, expectedValues);
    }

    function quantile(sortedValues, q) {
      if (!sortedValues.length) return NaN;
      const pos = (sortedValues.length - 1) * q;
      const lo = Math.floor(pos);
      const hi = Math.ceil(pos);
      if (lo === hi) return sortedValues[lo];
      return sortedValues[lo] + (sortedValues[hi] - sortedValues[lo]) * (pos - lo);
    }

    function quantileBinCoverage(selectedValues, universeValues, binCount = 10) {
      const universe = (universeValues || []).map(Number).filter(Number.isFinite).sort((a, b) => a - b);
      const selected = (selectedValues || []).map(Number).filter(Number.isFinite);
      if (!universe.length) return { value: 0, selectedBins: 0, universeBins: 0 };
      const min = universe[0];
      const max = universe[universe.length - 1];
      if (Math.abs(max - min) < 1e-12) {
        const covered = selected.some(v => Math.abs(v - min) < 1e-9);
        return { value: covered ? 1 : 0, selectedBins: covered ? 1 : 0, universeBins: 1 };
      }
      const edgeCount = Math.max(2, Number(binCount) || 10);
      const edges = [];
      for (let i = 0; i <= edgeCount; i += 1) {
        const edge = quantile(universe, i / edgeCount);
        if (Number.isFinite(edge) && (!edges.length || Math.abs(edge - edges[edges.length - 1]) > 1e-12)) edges.push(edge);
      }
      if (edges.length < 2) {
        const covered = selected.length > 0;
        return { value: covered ? 1 : 0, selectedBins: covered ? 1 : 0, universeBins: 1 };
      }
      const toBin = (value) => {
        if (value <= edges[0]) return 0;
        for (let i = 0; i < edges.length - 1; i += 1) {
          if (value <= edges[i + 1]) return i;
        }
        return edges.length - 2;
      };
      const universeBins = new Set(universe.map(toBin));
      const selectedBins = new Set(selected.map(toBin).filter(bin => universeBins.has(bin)));
      return { value: universeBins.size > 0 ? selectedBins.size / universeBins.size : 0, selectedBins: selectedBins.size, universeBins: universeBins.size };
    }

    function featureValue(candidate, keys, fallback = NaN) {
      const list = Array.isArray(keys) ? keys : [keys];
      for (const key of list) {
        const v = finiteNumber(candidate?.features?.[key], NaN);
        if (Number.isFinite(v)) return v;
      }
      return fallback;
    }

    function faultKey(fault) {
      if (!fault) return null;
      const idx = fault.branch_index ?? fault.index ?? fault.id;
      if (idx === undefined || idx === null || idx === '') return null;
      return `${fault.branch_type || 'AC'}:${idx}`;
    }

    function faultKeysFromEvent(event) {
      return (Array.isArray(event?.faults) ? event.faults : []).map(faultKey).filter(Boolean);
    }

    function arrayImpactMagnitude(values) {
      if (!Array.isArray(values)) return 0;
      let maxImpact = 0;
      values.forEach(value => {
        const v = Number(value);
        if (Number.isFinite(v)) maxImpact = Math.max(maxImpact, Math.abs(v - 1));
      });
      return maxImpact;
    }

    function coverageDetail(metric) {
      if (!metric) return '无有效候选样本';
      if (metric.universeBins !== undefined) return `覆盖 ${metric.selectedBins || 0}/${metric.universeBins || 0} 个候选分布箱`;
      return `覆盖 ${metric.selectedCount || 0}/${metric.universeCount || 0} 个类别`;
    }

    function deepCloneJson(obj) {
      return obj ? JSON.parse(JSON.stringify(obj)) : obj;
    }

    function normalizeGeneratedScenarioCarbonFields(systemJson) {
      const inferStaticGeneratorEmissionFactor = (item) => {
        const typ = String(item?.sgen_type || item?.type || item?.name || '').toLowerCase();
        if (/diesel/.test(typ)) return 0.70;
        if (/chp|gas|turbine/.test(typ)) return 0.45;
        if (/fuel\s*cell|fuelcell/.test(typ)) return 0.35;
        return 0.0;
      };
      const syncEmissionAlias = (items, inferMissing = null) => (items || []).forEach(item => {
        if (!item) return;
        let canonical = item.emission_factor_tco2_mwh;
        let legacy = item.co2_emission_rate;
        if (canonical === undefined && legacy !== undefined) canonical = legacy;
        if ((canonical === undefined || Math.abs(Number(canonical) || 0) <= 1e-12) && inferMissing) {
          const inferred = inferMissing(item);
          if (inferred > 0) canonical = inferred;
        }
        if (canonical !== undefined) item.emission_factor_tco2_mwh = canonical;
        if (item.co2_emission_rate === undefined && item.emission_factor_tco2_mwh !== undefined) {
          item.co2_emission_rate = item.emission_factor_tco2_mwh;
        }
      });
      syncEmissionAlias(systemJson?.ac?.generators);
      syncEmissionAlias(systemJson?.ac?.static_generators, inferStaticGeneratorEmissionFactor);
      syncEmissionAlias(systemJson?.dc?.static_generators, inferStaticGeneratorEmissionFactor);
      syncEmissionAlias(systemJson?.dc?.dc_static_generators, inferStaticGeneratorEmissionFactor);
    }

    function sumComponentCapacity(items, keys, serviceKey = 'in_service') {
      if (!Array.isArray(items)) return 0;
      return items.reduce((sum, item) => {
        if (item && item[serviceKey] === false) return sum;
        for (const key of keys) {
          const v = Number(item?.[key]);
          if (Number.isFinite(v) && v > 0) return sum + v;
        }
        return sum;
      }, 0);
    }

    function scenarioBaseTotals(systemJson) {
      const ac = systemJson?.ac || {};
      const dc = systemJson?.dc || {};
      const acLoad = sumComponentCapacity(ac.loads, ['p_mw']) || sumComponentCapacity(ac.buses, ['pd_mw']);
      const dcLoad = sumComponentCapacity(dc.loads, ['p_mw']) || sumComponentCapacity(dc.buses, ['pd_mw']);
      const pv = sumComponentCapacity(ac.pv_systems, ['pmax_mw', 'p_mw', 'sn_mva'])
        + sumComponentCapacity((ac.renewable_gens || []).filter(g => /solar|pv/i.test(String(g.type || g.name || ''))), ['p_rated_mw', 'p_mw'])
        + sumComponentCapacity((ac.static_generators || []).filter(g => /pv|solar/i.test(String(g.sgen_type || g.type || g.name || ''))), ['p_rated_mw', 'pmax_mw', 'p_mw'])
        + sumComponentCapacity(dc.pv_arrays, ['p_set_mw'])
        + sumComponentCapacity((dc.dc_static_generators || []).filter(g => /pv|solar/i.test(String(g.type || g.name || ''))), ['p_set_mw']);
      const wind = sumComponentCapacity((ac.renewable_gens || []).filter(g => /wind/i.test(String(g.type || g.name || ''))), ['p_rated_mw', 'p_mw'])
        + sumComponentCapacity((ac.static_generators || []).filter(g => /wind/i.test(String(g.sgen_type || g.type || g.name || ''))), ['p_rated_mw', 'pmax_mw', 'p_mw'])
        + sumComponentCapacity((dc.dc_static_generators || []).filter(g => /wind/i.test(String(g.type || g.name || ''))), ['p_set_mw']);
      return { load: acLoad + dcLoad, pv, wind };
    }

    function makeScaleProfile(rawValues, base, fallbackWhenZero = 1.0) {
      const values = Array.isArray(rawValues) ? rawValues.map(v => Number(v || 0)) : [];
      if (Math.abs(Number(base) || 0) <= 1e-9) return values.map(() => fallbackWhenZero);
      return values.map(v => v / base);
    }

    function profileJsonFromCandidate(candidate, profileName) {
      const values = scenarioCandidateProfileValues(candidate, profileName);
      return values.length ? { name: profileName, values } : null;
    }

    function buildComponentLoadProfiles(loadSeries, baseSystem, firstProfileId = 10) {
      const ac = baseSystem?.ac || {};
      const dc = baseSystem?.dc || {};
      const entries = [];
      const add = (kind, item, position, base, seed) => {
        if (!item || item.in_service === false || !(base > 1e-9)) return;
        entries.push({ kind, item, position, base, seed });
      };
      if (Array.isArray(ac.loads) && ac.loads.length) {
        ac.loads.forEach((ld, i) => add('AC_LOAD', ld, i, numberOr(ld.p_mw, 0) * numberOr(ld.scaling, 1), numberOr(ld.index, i + 1)));
      } else {
        (ac.buses || []).forEach((bus, i) => add('AC_BUS', bus, i, numberOr(bus.pd_mw, 0), numberOr(bus.index, i + 1)));
      }
      if (Array.isArray(dc.loads) && dc.loads.length) {
        dc.loads.forEach((ld, i) => add('DC_LOAD', ld, i, positivePower(ld.p_mw, ld.p_rated_mw) * numberOr(ld.scaling, 1), 1000 + numberOr(ld.index, i + 1)));
      } else {
        (dc.buses || []).forEach((bus, i) => add('DC_BUS', bus, i, numberOr(bus.pd_mw, 0), 2000 + numberOr(bus.index, i + 1)));
      }
      if (!entries.length || !Array.isArray(loadSeries) || !loadSeries.length) return { profiles: [], map: [] };
      const shapes = entries.map(e => baseDeterministicProfile(loadSeries.length, e.seed));
      const componentMw = entries.map(() => loadSeries.map(() => 0));
      loadSeries.forEach((raw, t) => {
        const denom = entries.reduce((sum, e, i) => sum + e.base * Math.max(0.01, Number(shapes[i]?.[t] || 1)), 0);
        entries.forEach((e, i) => {
          componentMw[i][t] = denom > 1e-9 ? Math.max(0, Number(raw || 0)) * e.base * Math.max(0.01, Number(shapes[i]?.[t] || 1)) / denom : 0;
        });
      });
      const profiles = [];
      const map = [];
      entries.forEach((e, i) => {
        const profileId = firstProfileId + i;
        profiles.push({ id: profileId, name: `scenario_${e.kind.toLowerCase()}_${e.item?.index ?? e.item?.bus ?? e.position + 1}_scale`, values: componentMw[i].map(v => v / e.base) });
        const row = { kind: e.kind, profile_id: profileId };
        const itemIndex = Number(e.item?.index);
        const itemBus = Number(e.item?.bus);
        if (e.kind.endsWith('LOAD')) {
          row.load_position = e.position;
          if (Number.isFinite(itemIndex)) row.load_index = itemIndex;
          if (Number.isFinite(itemBus)) row.bus = itemBus;
        } else if (Number.isFinite(itemIndex)) {
          row.bus = itemIndex;
        }
        map.push(row);
      });
      return { profiles, map };
    }

    function backendComponentLoadProfiles(candidate) {
      const source = candidate?.component_load_profiles;
      const rawProfiles = Array.isArray(source?.profiles) ? source.profiles : [];
      const rawMap = Array.isArray(source?.load_profile_map) ? source.load_profile_map : [];
      if (!rawProfiles.length || !rawMap.length) return { profiles: [], map: [], source: 'none' };
      const profiles = rawProfiles.map((p, i) => {
        const values = Array.isArray(p?.values) ? p.values.map(Number).map(v => Number.isFinite(v) ? v : 0) : [];
        return {
          id: Number.isInteger(p?.id) ? p.id : 10 + i,
          name: p?.name || `scenario_backend_load_${i + 1}_scale`,
          values,
        };
      }).filter(p => p.values.length);
      if (!profiles.length) return { profiles: [], map: [], source: 'none' };
      const ids = new Set(profiles.map(p => p.id));
      const map = rawMap.map(row => ({ ...row })).filter(row => ids.has(Number(row.profile_id)));
      return map.length ? { profiles, map, source: source?.source || 'backend_per_load_site' } : { profiles: [], map: [], source: 'none' };
    }

    function buildScenarioTimeSeries(candidate, baseSystem, family) {
      if (isUsableScenarioTimeSeries(candidate?.standard_time_series)) {
        const ts = deepCloneJson(candidate.standard_time_series);
        ts.family = ts.family || family;
        ts.unit_space = ts.unit_space || 'dimensionless_multiplier';
        if (!ts.binding && ts.bindings) ts.binding = ts.bindings;
        return ts;
      }
      const profiles = candidate?.time_series?.profiles || [];
      if (!Array.isArray(profiles) || profiles.length === 0) return null;
      const load = scenarioCandidateProfileValues(candidate, 'total_load_mw');
      const pv = scenarioCandidateProfileValues(candidate, 'pv_mw');
      const wind = scenarioCandidateProfileValues(candidate, 'wind_mw');
      const totals = scenarioBaseTotals(baseSystem);
      const calcProfiles = [];
      const warnings = [];
      let componentLoadProfiles = backendComponentLoadProfiles(candidate);
      if (!componentLoadProfiles.map.length) {
        componentLoadProfiles = load.length
          ? buildComponentLoadProfiles(load, baseSystem, 10)
          : { profiles: [], map: [], source: 'none' };
        componentLoadProfiles.source = componentLoadProfiles.map.length ? 'post_generated_total_load_split' : 'none';
      }
      if (load.length) {
        calcProfiles.push({ id: 0, name: 'scenario_load_scale', values: makeScaleProfile(load, totals.load, 1.0) });
        calcProfiles.push(...componentLoadProfiles.profiles);
        if (totals.load <= 1e-9) warnings.push('Base load is zero; load scale profile uses fallback values.');
      }
      const hasWindCapacity = totals.wind > 1e-9;
      const hasPvCapacity = totals.pv > 1e-9;
      if (wind.length && hasWindCapacity) {
        calcProfiles.push({ id: 1, name: 'scenario_wind_scale', values: makeScaleProfile(wind, totals.wind, 1.0) });
      } else if (wind.length && wind.some(v => Math.abs(v) > 1e-9)) {
        warnings.push('Base wind capacity is zero; wind scale profile is omitted.');
      }
      if (pv.length && hasPvCapacity) {
        calcProfiles.push({ id: 2, name: 'scenario_pv_scale', values: makeScaleProfile(pv, totals.pv, 1.0) });
      } else if (pv.length && pv.some(v => Math.abs(v) > 1e-9)) {
        warnings.push('Base PV capacity is zero; PV scale profile is omitted.');
      }
      if (calcProfiles.length === 0) return null;
      return {
        version: 1,
        family,
        num_steps: candidate?.time_series?.num_steps || calcProfiles[0].values.length,
        step_duration_hr: candidate?.time_series?.step_duration_hr || 1.0,
        profiles: calcProfiles,
        binding: {
          assign_all_loads_to: componentLoadProfiles.map.length ? -1 : (load.length ? 0 : -1),
          assign_all_pv_to: pv.length && hasPvCapacity ? 2 : -1,
          load_profile_map: componentLoadProfiles.map,
          resilience_load_profile_id: load.length ? 0 : -1,
          resilience_wind_profile_id: wind.length && hasWindCapacity ? 1 : -1,
          resilience_pv_profile_id: pv.length && hasPvCapacity ? 2 : -1,
          resilience_renewable_profile_id: pv.length && hasPvCapacity ? 2 : (wind.length && hasWindCapacity ? 1 : -1),
        },
        normalization: {
          base_load_mw: totals.load,
          base_pv_mw: totals.pv,
          base_wind_mw: totals.wind,
          profile_semantics: 'profiles are dimensionless multipliers for /api/session/set_ts_config',
          component_load_profile_source: componentLoadProfiles.source || (componentLoadProfiles.map.length ? 'post_generated_total_load_split' : 'none'),
          component_load_profile_note: componentLoadProfiles.source === 'backend_per_load_site'
            ? 'component load profiles are emitted by backend per-load/per-bus scenario generation and normalized against each component base MW'
            : 'component load profiles split backend randomized total_load_mw by capacity and deterministic component shapes, normalized per timestep',
        },
        warnings,
      };
    }

    function applyScenarioInitialStorageState(systemJson, candidate) {
      const soc = scenarioCandidateProfileValues(candidate, 'storage_soc')[0];
      if (!Number.isFinite(soc)) return;
      const update = (items) => (items || []).forEach(st => {
        if (st && Number(st.e_rated_mwh || 0) > 0) st.soc_init = Math.max(0, Math.min(1, soc));
      });
      update(systemJson?.ac?.storage);
      update(systemJson?.dc?.dc_storage);
      update(systemJson?.dc?.storage);
      update(systemJson?.mobile_storage);
    }

    function markOutOfServiceByIndex(items, index) {
      const list = Array.isArray(items) ? items : [];
      const target = list.find(item => Number(item?.index) === Number(index));
      if (target) target.in_service = false;
      return !!target;
    }

    function applyReliabilityContingencyToSystem(systemJson, contingency) {
      if (!systemJson || !contingency) return false;
      const idx = contingency.component_index;
      switch (String(contingency.type || '').toLowerCase()) {
        case 'acbranch': return markOutOfServiceByIndex(systemJson.ac?.branches, idx);
        case 'dcbranch': return markOutOfServiceByIndex(systemJson.dc?.branches, idx);
        case 'vscconverter': return markOutOfServiceByIndex(systemJson.vsc_converters, idx);
        case 'dcdcconverter': return markOutOfServiceByIndex(systemJson.dcdc_converters, idx) || markOutOfServiceByIndex(systemJson.dc?.dcdc_converters, idx);
        case 'acgenerator': return markOutOfServiceByIndex(systemJson.ac?.generators, idx);
        case 'acstaticgenerator': return markOutOfServiceByIndex(systemJson.ac?.static_generators, idx);
        case 'acrenewable': return markOutOfServiceByIndex(systemJson.ac?.renewable_gens, idx);
        case 'acpvsystem': return markOutOfServiceByIndex(systemJson.ac?.pv_systems, idx);
        case 'dcstaticgenerator': return markOutOfServiceByIndex(systemJson.dc?.dc_static_generators, idx) || markOutOfServiceByIndex(systemJson.dc?.static_generators, idx);
        case 'dcpvarray': return markOutOfServiceByIndex(systemJson.dc?.pv_arrays, idx);
        case 'acload': return markOutOfServiceByIndex(systemJson.ac?.loads, idx);
        case 'dcload': return markOutOfServiceByIndex(systemJson.dc?.loads, idx);
        case 'acstorage': return markOutOfServiceByIndex(systemJson.ac?.storage, idx);
        case 'dcstorage':
          return markOutOfServiceByIndex(systemJson.dc?.dc_storage, idx) ||
                 markOutOfServiceByIndex(systemJson.dc?.storage, idx);
        case 'mobilestorage': return markOutOfServiceByIndex(systemJson.mobile_storage, idx);
        case 'transformer2w': return markOutOfServiceByIndex(systemJson.ac?.transformers_2w, idx);
        case 'transformer3w': return markOutOfServiceByIndex(systemJson.ac?.transformers_3w, idx);
        default: return false;
      }
    }

    function extractScenarioClustersByFamily(data, family) {
      if (family === 'regular') return (data?.regular?.clusters || []).map(c => ({ cluster: c, group: null }));
      if (family === 'reliability') {
        return (data?.reliability?.contingencies || []).flatMap(group => (group.clusters || []).map(cluster => ({ cluster, group })));
      }
      if (family === 'resilience') {
        return (data?.resilience?.intensities || []).flatMap(group => (group.clusters || []).map(cluster => ({ cluster, group })));
      }
      return [];
    }

    function isUsableScenarioTimeSeries(ts) {
      if (!ts || !Array.isArray(ts.profiles) || ts.profiles.length === 0) return false;
      const numSteps = Number(ts.num_steps || ts.profiles[0]?.values?.length || 0);
      return numSteps > 0 && ts.profiles.some(p => Array.isArray(p?.values) && p.values.length > 0);
    }

    function getImportedGeneratedScenarioCase(targetFamily = null) {
      const caseJson = _importedGeneratedScenario?.case || null;
      if (!caseJson) return null;
      const family = caseJson?._generated_scenario?.family || _importedGeneratedScenario?.family || '';
      if (targetFamily && family && family !== 'unknown' && family !== targetFamily) return null;
      return caseJson;
    }

    function getImportedGeneratedScenarioTimeSeries(targetFamily = null) {
      const caseJson = getImportedGeneratedScenarioCase(targetFamily);
      const ts = caseJson?._time_series;
      return isUsableScenarioTimeSeries(ts) ? ts : null;
    }

    function hasUsableGeneratedScenarioTimeSeries(targetFamily = null) {
      return !!getImportedGeneratedScenarioTimeSeries(targetFamily);
    }

    function extractScenarioProfileValues(ts, profileIdOrName) {
      return extractGeneratedScenarioProfileValues(ts, profileIdOrName);
    }

    function getResilienceScenarioProfiles() {
      const ts = getImportedGeneratedScenarioTimeSeries('resilience');
      if (!ts) return {};
      const finiteProfile = (values) => Array.isArray(values)
        ? values.map(Number).filter(Number.isFinite).map(v => Math.max(0, v))
        : [];
      const loadProfileMap = Array.isArray(ts.binding?.load_profile_map) ? ts.binding.load_profile_map : [];
      let loadProfile = finiteProfile(extractScenarioProfileValues(ts, ts.binding?.resilience_load_profile_id));
      if (!loadProfile.length) loadProfile = finiteProfile(extractScenarioProfileValues(ts, 'scenario_load_scale'));
      let renewableProfile = finiteProfile(extractScenarioProfileValues(ts, ts.binding?.resilience_renewable_profile_id));
      if (!renewableProfile.length) renewableProfile = finiteProfile(extractScenarioProfileValues(ts, 'scenario_renewable_scale'));
      let pvProfile = finiteProfile(extractScenarioProfileValues(ts, ts.binding?.resilience_pv_profile_id));
      if (!pvProfile.length) pvProfile = finiteProfile(extractScenarioProfileValues(ts, 'scenario_pv_scale'));
      if (!pvProfile.length) pvProfile = renewableProfile;
      let windProfile = finiteProfile(extractScenarioProfileValues(ts, ts.binding?.resilience_wind_profile_id));
      if (!windProfile.length) windProfile = finiteProfile(extractScenarioProfileValues(ts, 'scenario_wind_scale'));
      if (!windProfile.length) windProfile = renewableProfile;
      return { loadProfile, renewableProfile, pvProfile, windProfile, loadProfileMap, tsProfiles: ts.profiles || [] };
    }

    function buildGeneratedScenarioCase(baseSystem, family, cluster, groupContext = null) {
      const representative = cluster?.representative || {};
      const caseJson = deepCloneJson(baseSystem || {});
      normalizeGeneratedScenarioCarbonFields(caseJson);
      const repId = cluster?.representative_id || representative.id || `${family}_scenario_${cluster?.cluster_id ?? 0}`;
      caseJson.name = `${caseJson.name || 'Generated Scenario'} - ${repId}`;
      applyScenarioInitialStorageState(caseJson, representative);
      const metadata = {
        version: 1,
        family,
        scenario_id: representative.id || repId,
        representative_id: repId,
        cluster_id: cluster?.cluster_id ?? 0,
        probability: cluster?.probability ?? representative.probability ?? 0,
        member_count: cluster?.member_count ?? 0,
        member_ids: cluster?.member_ids || [],
        features: representative.features || {},
        anchor_reasons: cluster?.anchor_reasons || representative.anchor_reasons || [],
        frozen_medoid: cluster?.frozen_medoid === true,
        calculation_defaults: {
          preferred_module: family === 'regular' ? 'timeSeries' : family,
          use_time_series: false,
          static_fallback: true,
          run_opf: false,
          run_power_flow: family === 'resilience',
        },
      };
      if (family === 'reliability') {
        metadata.contingency = representative.contingency || groupContext?.contingency || null;
        metadata.applied_to_system = false;
        metadata.contingency_application = 'metadata_only';
      }
      if (family === 'resilience') {
        metadata.intensity = groupContext?.intensity || representative.resilience_event?.selected_intensity || null;
        metadata.resilience_event = representative.resilience_event || null;
      }
      caseJson._generated_scenario = metadata;
      const ts = buildScenarioTimeSeries(representative, caseJson, family);
      if (ts) {
        caseJson._time_series = ts;
        metadata.calculation_defaults.use_time_series = family === 'regular' || family === 'reliability' || family === 'resilience';
      }
      return caseJson;
    }

    function buildGeneratedScenarioBundle(family) {
      if (!_lastScenarioGenerationData) throw new Error('暂无场景生成结果，请先生成场景');
      const baseSystem = _lastScenarioBaseSystemJson || Canvas.buildSystemJson();
      const entries = extractScenarioClustersByFamily(_lastScenarioGenerationData, family);
      if (!entries.length) throw new Error(`暂无${family}代表场景可导出`);
      const cases = entries.map(({ cluster, group }) => buildGeneratedScenarioCase(baseSystem, family, cluster, group));
      return {
        format: 'generated_scenario_case_bundle_v2',
        schema_version: 2,
        case_format: 'hacdcpf_system_json_with_generated_scenario_metadata_v1',
        time_series_preserved: cases.some(c => isUsableScenarioTimeSeries(c?._time_series)),
        family,
        case_count: cases.length,
        exported_at: new Date().toISOString(),
        source_summary: _lastScenarioGenerationData.summary || {},
        cases,
      };
    }

    async function applyGeneratedScenarioTimeSeries(caseJson) {
      const ts = caseJson?._time_series;
      if (!ts || !Array.isArray(ts.profiles) || ts.profiles.length === 0) return false;
      const body = {
        num_steps: ts.num_steps || ts.profiles[0]?.values?.length || 24,
        step_duration_hr: ts.step_duration_hr || 1.0,
        profiles: ts.profiles,
        load_profile_map: ts.binding?.load_profile_map || [],
        assign_all_loads_to: Number.isInteger(ts.binding?.assign_all_loads_to) ? ts.binding.assign_all_loads_to : -1,
        assign_all_pv_to: Number.isInteger(ts.binding?.assign_all_pv_to) ? ts.binding.assign_all_pv_to : -1,
      };
      const r = await fetch('/api/session/set_ts_config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const j = await r.json();
      if (!r.ok) throw new Error(j.error || 'set_ts_config failed');
      _generatedScenarioTimeSeriesActive = true;
      const simHr = document.getElementById('simulationHours');
      if (simHr) simHr.value = String(j.num_steps || body.num_steps);
      log(`生成场景时序已恢复：${j.num_steps}步，profiles=${j.num_profiles}`, 'success');
      return true;
    }

    function generatedScenarioCaseMetadata(caseJson) {
      return caseJson?._generated_scenario || caseJson?.generated_scenario || {};
    }

    function generatedScenarioCaseTimeSeries(caseJson) {
      return caseJson?._time_series || caseJson?.standard_time_series || null;
    }

    function generatedScenarioCaseSystem(caseJson) {
      return caseJson?.system || caseJson;
    }

    function generatedScenarioCaseFaults(caseJson) {
      const event = generatedScenarioCaseMetadata(caseJson)?.resilience_event || caseJson?.resilience_event || {};
      const candidates = [event.faults, event.generated_faults, event.manual_faults, caseJson?.generated_faults, caseJson?.manual_faults];
      for (const arr of candidates) if (Array.isArray(arr) && arr.length) return arr;
      return [];
    }

    function generatedScenarioCaseLabel(caseJson, index) {
      const meta = generatedScenarioCaseMetadata(caseJson);
      const event = meta.resilience_event || {};
      const faults = generatedScenarioCaseFaults(caseJson);
      const family = meta.family || 'unknown';
      const rep = meta.representative_id || meta.scenario_id || caseJson?.name || `case_${index + 1}`;
      if (family === 'resilience') {
        const starts = faults.map(f => Number(f.start_hr ?? f.outage_start_hr)).filter(Number.isFinite);
        const startText = starts.length ? `，首故障 ${Math.min(...starts).toFixed(1)}h` : '';
        const intensity = meta.intensity || event.selected_intensity || event.requested_intensity || '-';
        return `${index + 1}. ${rep}｜强度 ${intensity}｜故障 ${faults.length} 个${startText}`;
      }
      if (family === 'reliability') {
        const cont = meta.contingency || {};
        return `${index + 1}. ${rep}｜${cont.type || 'N-1'} ${cont.display_name || cont.id || ''}`;
      }
      return `${index + 1}. ${rep}`;
    }

    function chooseGeneratedScenarioCase(cases, targetFamily) {
      const matched = cases.filter(c => !targetFamily || generatedScenarioCaseMetadata(c)?.family === targetFamily);
      const choices = matched.length ? matched : cases;
      if (!choices.length) throw new Error('生成场景文件中没有可导入 case');
      if (choices.length === 1) return choices[0];

      const defaultIndex = targetFamily === 'resilience'
        ? Math.max(0, choices.findIndex(c => generatedScenarioCaseFaults(c).length > 0))
        : 0;
      const listed = choices.slice(0, 40).map((caseJson, index) => generatedScenarioCaseLabel(caseJson, index)).join('\n');
      const suffix = choices.length > 40 ? `\n... 另有 ${choices.length - 40} 个场景未列出，可直接输入编号选择` : '';
      const promptText = `该文件包含 ${choices.length} 个${targetFamily || ''}生成场景，请输入要导入的编号：\n\n${listed}${suffix}`;
      const input = window.prompt(promptText, String(defaultIndex + 1));
      if (input === null) throw new Error('已取消导入生成场景');
      const chosen = Number.parseInt(input, 10);
      if (!Number.isInteger(chosen) || chosen < 1 || chosen > choices.length) {
        throw new Error(`场景编号无效，请输入 1-${choices.length} 之间的整数`);
      }
      return choices[chosen - 1];
    }

    function normalizeGeneratedScenarioCaseShape(caseLike) {
      const caseJson = deepCloneJson(generatedScenarioCaseSystem(caseLike) || {});
      if (!caseJson || (!caseJson.ac && !caseJson.dc)) return caseLike;
      const meta = generatedScenarioCaseMetadata(caseLike);
      const ts = generatedScenarioCaseTimeSeries(caseLike);
      if (meta && Object.keys(meta).length && !caseJson._generated_scenario) caseJson._generated_scenario = deepCloneJson(meta);
      if (ts && !caseJson._time_series) caseJson._time_series = deepCloneJson(ts);
      return caseJson;
    }

    function firstCaseFromGeneratedScenarioJson(obj, targetFamily) {
      const allowedBundleFormats = new Set(['generated_scenario_case_bundle_v1', 'generated_scenario_case_bundle_v2', 'generated_scenario_case_bundle_v3']);
      if (allowedBundleFormats.has(obj?.format) && Array.isArray(obj.cases)) {
        return normalizeGeneratedScenarioCaseShape(chooseGeneratedScenarioCase(obj.cases, targetFamily));
      }
      if (obj && (obj.ac || obj.dc || obj.system)) return normalizeGeneratedScenarioCaseShape(obj);
      throw new Error('不是有效的生成场景 JSON 或系统算例 JSON');
    }

    function fillResilienceInputsFromScenario(caseJson) {
      const setVal = (id, value) => { const el = document.getElementById(id); if (el) el.value = value; };
      const faults = generatedScenarioCaseFaults(caseJson);
      if (!faults.length) {
        setVal('resFaultCount', '0');
        setVal('resAcFaultLocations', '');
        setVal('resDcFaultLocations', '');
        setVal('resAcFaultStartHour', '');
        setVal('resDcFaultStartHour', '');
        setVal('resAcRepairDuration', '');
        setVal('resDcRepairDuration', '');
        return;
      }
      const ac = [], dc = [];
      faults.forEach(f => {
        const type = String(f.branch_type || f.branch_kind || f.type || 'AC').toUpperCase();
        const id = f.branch_index ?? f.branch_id ?? f.branch;
        if (!Number.isFinite(Number(id))) return;
        const entry = { id, start: Number(f.start_hr ?? f.outage_start_hr ?? 0), repair: Number(f.repair_hr ?? f.repair_duration_hr ?? f.repair_time_hr ?? 6) };
        if (type === 'DC') dc.push(entry); else ac.push(entry);
      });
      setVal('resFaultCount', String(ac.length + dc.length));
      setVal('resAcFaultLocations', ac.map(f => f.id).join(','));
      setVal('resDcFaultLocations', dc.map(f => f.id).join(','));
      setVal('resAcFaultStartHour', ac.map(f => Number.isFinite(f.start) ? f.start : 0).join(','));
      setVal('resDcFaultStartHour', dc.map(f => Number.isFinite(f.start) ? f.start : 0).join(','));
      setVal('resAcRepairDuration', ac.map(f => Number.isFinite(f.repair) ? f.repair : 6).join(','));
      setVal('resDcRepairDuration', dc.map(f => Number.isFinite(f.repair) ? f.repair : 8).join(','));
      markResilienceFaultBranches();
    }

    async function importGeneratedScenarioForModule(file, targetFamily) {
      try {
        const text = await readFileAsText(file);
        const parsed = JSON.parse(text);
        const caseJson = deepCloneJson(firstCaseFromGeneratedScenarioJson(parsed, targetFamily));
        normalizeGeneratedScenarioCarbonFields(caseJson);
        const family = caseJson?._generated_scenario?.family || 'unknown';
        const data = await apiPost('/api/session/load_json_string', { json_string: JSON.stringify(caseJson) });
        if (!data) throw new Error('导入系统失败');
        Canvas.loadFromSystemJson(caseJson);
        _canvasDirty = false;
        updateResilienceSwitchDefault();
        _importedGeneratedScenario = { family, case: caseJson };
        _lastImportedGeneratedScenarioKey = caseJson?._generated_scenario?.representative_id || caseJson?.name || '';
        _lastTspfData = null;
        const target = targetFamily || family;
        let restoredTs = false;
        const targetUsesScenarioTs = target === 'regular' || target === 'resilience';
        if (targetUsesScenarioTs) {
          restoredTs = await applyGeneratedScenarioTimeSeries(caseJson);
        }
        if (!targetUsesScenarioTs) {
          const regCb = document.getElementById('regUseScenarioTimeSeries');
          if (regCb) regCb.checked = false;
        }
        if (target === 'regular' || family === 'regular') {
          const cb = document.getElementById('regUseScenarioTimeSeries');
          if (cb) cb.checked = restoredTs;
          if (!restoredTs) log('导入的常规生成场景不含可用时序，已保持“使用场景时序”未勾选', 'warn');
        }
        if (family === 'resilience' || targetFamily === 'resilience') {
          fillResilienceInputsFromScenario(caseJson);
          const cb = document.getElementById('resUseScenarioTimeSeries');
          if (cb) cb.checked = true;
          if (!restoredTs) log('导入的弹性生成场景不含可用时序；弹性评估仍将按 48h 默认时域运行', 'warn');
        }
        if (family === 'reliability' || targetFamily === 'reliability') {
          const cont = caseJson?._generated_scenario?.contingency;
          if (cont) log(`已导入可靠性代表场景（N-1 静态断面）：${cont.type || ''} ${cont.display_name || cont.id || ''}`, 'info');
        }
        log(`已导入生成场景：${caseJson?._generated_scenario?.representative_id || caseJson.name || file.name}${restoredTs ? '，并恢复时序配置' : ''}`, 'success');
        setStatus('生成场景已导入');
      } catch (err) {
        log(`导入生成场景失败：${err.message || err}`, 'error');
        setStatus('导入生成场景失败', 'error');
      }
    }

    function renderScenarioGenerationCharts(data) {
      const chartsDiv = document.getElementById('scenarioGenerationCharts');
      if (!chartsDiv) return;
      chartsDiv.innerHTML = `
        <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px;margin-bottom:16px;">
          <div id="scenarioChartRegular" class="tspf-chart" style="min-height:340px;"></div>
          <div id="scenarioChartReliability" class="tspf-chart" style="min-height:340px;"></div>
          <div id="scenarioChartResilience" class="tspf-chart" style="min-height:340px;"></div>
          <div id="scenarioChartRegularCurves" class="tspf-chart" style="min-height:360px;"></div>
          <div id="scenarioChartReliabilityBars" class="tspf-chart" style="min-height:360px;"></div>
          <div id="scenarioChartResilienceCurves" class="tspf-chart" style="min-height:360px;"></div>
          <div id="scenarioChartFaultSequence" class="tspf-chart" style="min-height:360px;"></div>
        </div>`;
      if (!window.Plotly) {
        chartsDiv.insertAdjacentHTML('afterbegin', '<p class="empty-hint">Plotly 未加载，无法展示图表。</p>');
        return;
      }
      const theme = {
        paper_bgcolor: '#ffffff',
        plot_bgcolor: '#ffffff',
        font: { color: '#22272e', size: 11 },
        margin: { l: 52, r: 28, t: 42, b: 58 },
        legend: { orientation: 'h', y: -0.22 },
        xaxis: { gridcolor: '#d0d7de' },
        yaxis: { gridcolor: '#d0d7de', rangemode: 'tozero' },
      };

      const featureNumber = (features, key) => {
        const v = Number(features?.[key]);
        return Number.isFinite(v) ? v : 0;
      };
      const hasRenewableFeature = (points) => {
        const warnedNoRen = (data.warnings || []).some(w => /no renewable|zero renewable/i.test(String(w || '')));
        if (warnedNoRen) return false;
        return (points || []).some(p => Math.abs(Number(p.renewable || 0)) > 1e-6
          || Math.abs(featureNumber(p.features, 'pv_sum')) > 1e-6
          || Math.abs(featureNumber(p.features, 'wind_sum')) > 1e-6
          || Math.abs(featureNumber(p.features, 'other_renewable_sum')) > 1e-6);
      };
      const plotCoverage = (divId, allPoints, selectedPoints, labels) => {
        const all = allPoints || [];
        const selected = selectedPoints || [];
        const hasRen = hasRenewableFeature(all.concat(selected));
        const xKey = 'load';
        const yKey = hasRen ? 'renewable' : 'peakLoad';
        const xTitle = '负荷特征';
        const yTitle = hasRen ? '新能源出力特征' : '峰值负荷';
        const title = hasRen ? labels.renewableTitle : labels.loadOnlyTitle;
        const hover = (p, picked) => `${escapeHtml(p.group || p.name || p.id || '')}<br>${picked ? `簇${p.cluster ?? ''}<br>` : ''}负荷总量 ${Number(p.load || 0).toFixed(2)}<br>峰值负荷 ${Number(p.peakLoad || 0).toFixed(2)}<br>最大爬坡 ${Number(p.maxRamp || 0).toFixed(2)}${hasRen ? `<br>新能源 ${Number(p.renewable || 0).toFixed(2)}` : '<br>当前算例无新能源，自动改用峰值负荷作纵轴'}${p.probability ? `<br>簇概率 ${(p.probability * 100).toFixed(2)}%` : ''}`;
        Plotly.newPlot(divId, [
          {
            x: all.map(p => p[xKey]), y: all.map(p => p[yKey]), mode: 'markers', type: 'scatter',
            name: labels.allName,
            text: all.map(p => hover(p, false)), hovertemplate: '%{text}<extra></extra>',
            marker: { color: '#60a5fa', size: 7, opacity: 0.38, symbol: 'circle' },
          },
          {
            x: selected.map(p => p[xKey]), y: selected.map(p => p[yKey]), mode: 'markers', type: 'scatter',
            name: '聚类选中代表',
            text: selected.map(p => hover(p, true)), hovertemplate: '%{text}<extra></extra>',
            marker: { symbol: 'circle-open', color: '#dc2626', size: 14, line: { color: '#dc2626', width: 3 } },
          },
        ], { ...theme, title, xaxis: { title: xTitle, autorange: true }, yaxis: { title: yTitle, autorange: true, rangemode: hasRen ? undefined : 'tozero' } }, { responsive: true });
      };

      const regularClusters = data.regular?.clusters || [];
      const regularSamples = data.regular?.audit?.coverage_samples || [];
      const regularSelected = [];
      regularClusters.forEach(cluster => {
        const selected = parseRegularId(cluster.representative_id);
        regularSelected.push({
          ...selected,
          group: `${selected.ssp}-${selected.year}`,
          cluster: cluster.cluster_id,
          probability: cluster.probability || 0,
          peakLoad: numericFeature(cluster, 'peak_load'),
          maxRamp: numericFeature(cluster, 'max_net_ramp'),
          renewable: numericFeature(cluster, 'renewable_sum'),
          load: numericFeature(cluster, 'load_sum'),
          features: cluster.representative?.features || {},
        });
      });
      const regularAll = regularSamples.map(s => {
        const parsed = parseRegularId(s.id);
        return {
          ...parsed,
          group: `${parsed.ssp}-${parsed.year}`,
          load: Number(s.features?.load_sum || 0),
          peakLoad: Number(s.features?.peak_load || 0),
          maxRamp: Number(s.features?.max_net_ramp || 0),
          renewable: Number(s.features?.renewable_sum || 0),
          features: s.features || {},
        };
      });
      plotCoverage('scenarioChartRegular', regularAll, regularSelected, {
        allName: '全部候选样本',
        renewableTitle: '常规场景二维覆盖：负荷不确定性 × 新能源不确定性（小圈=全部样本，红圈=聚类代表）',
        loadOnlyTitle: '常规场景负荷聚类覆盖：负荷总量 × 峰值负荷（当前算例无新能源）',
      });

      const relGroups = data.reliability?.contingencies || [];
      const relSamples = data.reliability?.audit?.coverage_samples || [];
      const relSelected = [];
      relGroups.forEach((g, index) => {
        (g.clusters || []).forEach(c => {
          relSelected.push({
            type: g.contingency?.type || 'Unknown',
            index: index + 1,
            probability: c.probability || 0,
            load: numericFeature(c, 'load_sum'),
            peakLoad: numericFeature(c, 'peak_load') || numericFeature(c, 'load_mw'),
            maxRamp: numericFeature(c, 'max_net_ramp'),
            renewable: numericFeature(c, 'renewable_sum'),
            faultOrdinal: Number(c.representative?.features?.contingency_ordinal || index + 1),
            name: g.contingency?.display_name || g.contingency?.id || '',
            features: c.representative?.features || {},
          });
        });
      });
      const relAll = relSamples.map(s => ({
        type: s.contingency?.type || 'Unknown',
        faultOrdinal: Number(s.features?.contingency_ordinal || 0),
        load: Number(s.features?.load_sum || s.features?.load_mw || 0),
        peakLoad: Number(s.features?.peak_load || s.features?.load_mw || 0),
        maxRamp: Number(s.features?.max_net_ramp || 0),
        renewable: Number(s.features?.renewable_sum || s.features?.renewable_mw || 0),
        name: s.contingency?.display_name || s.contingency?.id || s.id || '',
        features: s.features || {},
      }));
      plotCoverage('scenarioChartReliability', relAll, relSelected, {
        allName: '全部N-1候选样本',
        renewableTitle: '可靠性场景二维覆盖：负荷不确定性 × 新能源不确定性（小圈=全部样本，红圈=聚类代表）',
        loadOnlyTitle: '可靠性场景负荷扰动覆盖：负荷总量 × 峰值负荷（当前算例无新能源）',
      });

      const resGroups = data.resilience?.intensities || [];
      const resSamples = data.resilience?.audit?.coverage_samples || [];
      const resAll = resSamples.map(s => {
        const event = s.resilience_event || {};
        return {
          category: event.selected_intensity || '',
          maxWind: eventMaxWind(event) || Number(s.features?.selected_track_max_vmax_ms || 0),
          faultCount: Array.isArray(event.faults) ? event.faults.length : Number(s.features?.fault_count || 0),
          peakFailure: Number(s.features?.peak_failure_probability || 0),
          month: event.month || '-',
          sampleId: event.selected_sample_id || s.id || '',
          features: s.features || {},
        };
      });
      const resSelected = [];
      resGroups.forEach(g => {
        (g.clusters || []).forEach(c => {
          const event = c.representative?.resilience_event || {};
          const faultCount = Array.isArray(event.faults) ? event.faults.length : 0;
          resSelected.push({
            category: event.selected_intensity || g.intensity,
            maxWind: eventMaxWind(event),
            sampleId: event.selected_sample_id || '',
            month: event.month || '-',
            faultCount,
            probability: c.probability || 0,
            peakFailure: numericFeature(c, 'peak_failure_probability'),
            id: c.representative_id,
            features: c.representative?.features || {},
          });
        });
      });
      Plotly.newPlot('scenarioChartResilience', [
        {
          x: resAll.map(p => p.maxWind),
          y: resAll.map(p => p.faultCount),
          mode: 'markers',
          type: 'scatter',
          name: '全部台风样本',
          text: resAll.map(p => `划分后强度 ${p.category}<br>最大风速(不含t0) ${p.maxWind.toFixed(2)} m/s<br>月份 ${p.month}<br>样本 ${p.sampleId}<br>故障数 ${p.faultCount}<br>峰值失效概率 ${(p.peakFailure * 100).toFixed(2)}%`),
          marker: { color: '#f59e0b', size: 6, opacity: 0.35, symbol: 'circle' },
        },
        {
          x: resSelected.map(p => p.maxWind),
          y: resSelected.map(p => p.faultCount),
          mode: 'markers',
          type: 'scatter',
          name: '聚类选中代表',
          text: resSelected.map(p => `${p.id}<br>划分后强度 ${p.category}<br>最大风速(不含t0) ${p.maxWind.toFixed(2)} m/s<br>月份 ${p.month}<br>样本 ${p.sampleId}<br>故障数 ${p.faultCount}<br>峰值失效概率 ${(p.peakFailure * 100).toFixed(2)}%<br>簇概率 ${(p.probability * 100).toFixed(2)}%`),
          marker: { symbol: 'circle-open', color: '#dc2626', size: 13, line: { color: '#dc2626', width: 3 } },
        },
      ], { ...theme, title: '弹性场景二维覆盖：最大风速 × 故障数（小圈=全部样本，红圈=聚类代表）', xaxis: { title: '最大风速 / m/s（不含初始时刻）' }, yaxis: { title: '故障线路数', rangemode: 'tozero' } }, { responsive: true });

      const regularRepresentatives = regularClusters.map(c => c.representative).filter(Boolean);
      const relRepresentatives = relGroups.flatMap(g => (g.clusters || []).map(c => c.representative).filter(Boolean));
      const firstRegular = regularClusters[0]?.representative;
      if (firstRegular) {
        const load = scenarioCandidateProfileValues(firstRegular, 'total_load_mw').slice(0, 168);
        const pv = scenarioCandidateProfileValues(firstRegular, 'pv_mw').slice(0, 168);
        const wind = scenarioCandidateProfileValues(firstRegular, 'wind_mw').slice(0, 168);
        const x = load.map((_, i) => i);
        const hasRenCurve = pv.some(v => Math.abs(v) > 1e-6) || wind.some(v => Math.abs(v) > 1e-6);
        const traces = [{ x, y: load, type: 'scatter', mode: 'lines', name: '负荷 MW', line: { color: '#2563eb', width: 2 } }];
        if (hasRenCurve) {
          traces.push({ x, y: pv, type: 'scatter', mode: 'lines', name: '光伏 MW', line: { color: '#f59e0b' } });
          traces.push({ x, y: wind, type: 'scatter', mode: 'lines', name: '风电 MW', line: { color: '#16a34a' } });
        }
        Plotly.newPlot('scenarioChartRegularCurves', traces,
          { ...theme, title: hasRenCurve ? '常规代表场景风/光/负荷曲线（前168小时）' : '常规代表场景负荷曲线（前168小时，当前算例无新能源）', xaxis: { title: '小时' }, yaxis: { title: 'MW', rangemode: 'tozero' } }, { responsive: true });
      }

      const firstRel = relGroups.find(g => (g.clusters || []).length)?.clusters?.[0]?.representative;
      if (firstRel) {
        const names = ['负荷', '光伏', '风电'];
        const values = ['total_load_mw', 'pv_mw', 'wind_mw'].map(name => scenarioCandidateProfileValues(firstRel, name)[0] || 0);
        Plotly.newPlot('scenarioChartReliabilityBars', [{ x: names, y: values, type: 'bar', marker: { color: ['#2563eb', '#f59e0b', '#16a34a'] } }],
          { ...theme, title: '可靠性代表场景风/光/负荷（N-1 单时段）', xaxis: { title: '变量' }, yaxis: { title: 'MW', rangemode: 'tozero' } }, { responsive: true });
      }

      const allResClusters = resGroups.flatMap(g => g.clusters || []);
      const firstFaultedResCluster = allResClusters.find(c => ((c.representative?.resilience_event || {}).faults || []).length > 0);
      const firstRes = (firstFaultedResCluster || allResClusters[0])?.representative;
      if (firstRes) {
        const load = scenarioCandidateProfileValues(firstRes, 'total_load_mw');
        const pv = scenarioCandidateProfileValues(firstRes, 'pv_mw');
        const wind = scenarioCandidateProfileValues(firstRes, 'wind_mw');
        const x = load.map((_, i) => i);
        const traces = [
          { x, y: load, type: 'scatter', mode: 'lines', name: '负荷 MW', line: { color: '#2563eb' } },
          { x, y: pv, type: 'scatter', mode: 'lines', name: '光伏 MW', line: { color: '#f59e0b' } },
          { x, y: wind, type: 'scatter', mode: 'lines', name: '风电 MW', line: { color: '#16a34a' } },
        ];
        Plotly.newPlot('scenarioChartResilienceCurves', traces,
          { ...theme, title: '弹性代表场景风/光/负荷曲线（48小时）', xaxis: { title: '小时' }, yaxis: { title: 'MW', rangemode: 'tozero' } }, { responsive: true });

        const faults = (firstRes.resilience_event?.faults || [])
          .map((f, idx) => {
            const start = finiteNumber(f.start_hr, NaN);
            return {
              ...f,
              seq: idx + 1,
              start: Number.isFinite(start) ? start : 0,
              label: `${f.branch_type || 'AC'}-${f.branch_index ?? '-'}`,
            };
          })
          .sort((a, b) => (a.start - b.start) || (a.seq - b.seq));
        if (faults.length) {
          Plotly.newPlot('scenarioChartFaultSequence', [{
            x: faults.map(f => f.start),
            y: faults.map((_, idx) => idx + 1),
            type: 'scatter',
            mode: 'markers',
            name: '故障发生时刻',
            marker: { color: '#dc2626', size: 12, symbol: 'x' },
            text: faults.map(f => `${f.label}<br>故障发生 ${f.start.toFixed(2)} h`),
            hovertemplate: '%{text}<extra></extra>',
          }], {
            ...theme,
            title: '弹性代表场景故障序列（只展示线路故障发生时刻；修复由弹性评估决策）',
            xaxis: { title: '故障发生小时', range: [0, Math.max(48, ...faults.map(f => f.start))], rangemode: 'tozero' },
            yaxis: { title: '故障序号（按开始时间排序）', dtick: 1, rangemode: 'tozero' },
          }, { responsive: true });
        } else {
          Plotly.newPlot('scenarioChartFaultSequence', [], {
            ...theme,
            title: '弹性代表场景故障序列（该代表场景无线路故障）',
            xaxis: { title: '小时', range: [0, 48] },
            yaxis: { visible: false },
            annotations: [{ text: '该弹性代表场景无故障线路', x: 0.5, y: 0.5, xref: 'paper', yref: 'paper', showarrow: false }],
          }, { responsive: true });
        }
      }
    }

    function renderScenarioGenerationResults(data) {
      const div = document.getElementById('scenarioGenerationResults');
      const summaryDiv = document.getElementById('resultsSummary');
      if (!div || !summaryDiv) return;
      renderScenarioGenerationCharts(data);
      renderScenarioCurveSelectors(data);
      renderComponentCurveTargets('scenario');
      const summary = data.summary || {};
      const regularAudit = data.regular?.audit || {};
      const regularPerGroup = regularAudit.requested_cluster_count_per_ssp_year ?? '-';
      const regularGroups = regularAudit.group_count ?? '-';
      summaryDiv.innerHTML = `
        <div class="result-item"><span class="result-label">常规候选/总聚类</span><span class="result-value">${summary.regular_candidate_count ?? 0} / ${summary.regular_cluster_count ?? 0}</span></div>
        <div class="result-item"><span class="result-label">常规每组×组数</span><span class="result-value">${regularPerGroup} × ${regularGroups}</span></div>
        <div class="result-item"><span class="result-label">可靠性FMEA N-1/聚类</span><span class="result-value">${summary.reliability_contingency_count ?? 0} / ${summary.reliability_cluster_total ?? 0}</span></div>
        <div class="result-item"><span class="result-label">台风划分集合/聚类</span><span class="result-value">${summary.resilience_intensity_count ?? 0} / ${summary.resilience_cluster_total ?? 0}</span></div>
      `;
      const warningHtml = Array.isArray(data.warnings) && data.warnings.length
        ? `<div style="margin-bottom:10px;color:#b45309;"><strong>Warnings:</strong> ${data.warnings.map(escapeHtml).join('；')}</div>`
        : '';
      const regularRows = (data.regular?.clusters || []).map(c => `<tr>
        <td>${c.cluster_id}</td><td>${escapeHtml(c.representative_id || '')}</td><td>${((c.probability || 0) * 100).toFixed(2)}%</td>
        <td>${c.member_count ?? 0}</td><td>${firstProfileSummary(c, 'total_load_mw')}</td><td>${firstProfileSummary(c, 'total_renewable_mw')}</td>
      </tr>`).join('');
      const relGroups = data.reliability?.contingencies || [];
      const relRows = relGroups.slice(0, 120).map(g => {
        const best = (g.clusters || [])[0] || {};
        const cont = g.contingency || {};
        return `<tr><td>${escapeHtml(cont.id || '')}</td><td>${escapeHtml(cont.type || '')}</td><td>${escapeHtml(cont.display_name || '')}</td><td>${g.candidate_count ?? 0}</td><td>${g.cluster_count ?? 0}</td><td>${((best.probability || 0) * 100).toFixed(2)}%</td><td>${firstProfileSummary(best, 'total_load_mw')}</td><td>${firstProfileSummary(best, 'total_renewable_mw')}</td></tr>`;
      }).join('');
      const relMore = relGroups.length > 120 ? `<p class="empty-hint">仅展示前120个 N-1 分组；完整结果可导出 JSON。</p>` : '';
      const resRows = (data.resilience?.intensities || []).flatMap(g => (g.clusters || []).map(top => {
        const event = top.representative?.resilience_event || {};
        const faultCount = Array.isArray(event.faults) ? event.faults.length : 0;
        const maxWind = eventMaxWind(event);
        const faultTimes = (Array.isArray(event.faults) ? event.faults : [])
          .map(f => finiteNumber(f.start_hr, NaN))
          .filter(Number.isFinite)
          .sort((a, b) => a - b);
        const firstFault = faultTimes.length ? faultTimes[0].toFixed(2) : '-';
        const faultTimeText = faultTimes.length ? faultTimes.slice(0, 6).map(v => v.toFixed(1)).join(', ') + (faultTimes.length > 6 ? '...' : '') : '-';
        return `<tr><td>${escapeHtml(g.intensity || '')}</td><td>${g.candidate_count ?? 0}</td><td>${g.cluster_count ?? 0}</td><td>${escapeHtml(top.representative_id || '')}</td><td>${escapeHtml(event.selected_intensity || '')}</td><td>${maxWind > 0 ? maxWind.toFixed(2) : '-'}</td><td>${escapeHtml(String(event.month || '-'))} / ${escapeHtml(event.selected_sample_id || '-')}</td><td>${faultCount}</td><td>${firstFault}</td><td>${escapeHtml(faultTimeText)}</td></tr>`;
      })).join('');
      div.innerHTML = `
        ${warningHtml}
        <h4>常规全年代表场景（8760h）</h4>
        <table><thead><tr><th>簇</th><th>代表场景</th><th>概率</th><th>成员数</th><th>负荷总量/峰值</th><th>新能源总量/峰值</th></tr></thead><tbody>${regularRows || '<tr><td colspan="6" style="color:#888">未生成常规场景</td></tr>'}</tbody></table>
        <h4>可靠性 N-1 单断面场景</h4>
        <p class="empty-hint">可靠性覆盖图采用二维映射：x=负荷特征，y=新能源出力特征。</p>
        ${relMore}
        <table><thead><tr><th>N-1 ID</th><th>类型</th><th>元件</th><th>候选</th><th>聚类</th><th>首簇概率</th><th>首簇负荷摘要</th><th>首簇新能源摘要</th></tr></thead><tbody>${relRows || '<tr><td colspan="8" style="color:#888">未生成可靠性场景</td></tr>'}</tbody></table>
        <h4>弹性台风代表场景（48h，一次扰动+台风二次扰动）</h4>
        <table><thead><tr><th>划分后强度集合</th><th>候选</th><th>聚类</th><th>代表场景</th><th>分类强度</th><th>最大风速(m/s，不含t0)</th><th>月份/sample</th><th>故障数</th><th>首故障(h)</th><th>故障时刻(h)</th></tr></thead><tbody>${resRows || '<tr><td colspan="10" style="color:#888">未生成弹性场景</td></tr>'}</tbody></table>
      `;
    }


    document.getElementById('btnGenerateScenarios')?.addEventListener('click', async () => {
      let payload;
      try {
        payload = collectScenarioGenerationOptions();
      } catch (err) {
        log(`场景参数错误：${err.message}`, 'warn');
        setStatus('场景参数错误', 'error');
        return;
      }
      setStatus('场景生成中...', 'busy');
      _lastScenarioBaseSystemJson = Canvas.buildSystemJson();
      if (!await syncToBackend()) { setStatus('同步失败', 'error'); return; }
      const data = await apiPost('/api/session/generate_scenarios', payload);
      if (!data) { setStatus('场景生成失败', 'error'); return; }
      _lastScenarioGenerationData = data;
      renderComponentCurveTargets('scenario');
      setActiveResultGroup('scenarioGeneration');
      document.getElementById('resultsEmpty').style.display = 'none';
      document.getElementById('resultsContent').style.display = 'block';
      switchTab('results');
      renderScenarioGenerationResults(data);
      setTimeout(() => {
        ['scenarioChartRegular', 'scenarioChartReliability', 'scenarioChartResilience', 'scenarioChartRegularCurves', 'scenarioChartReliabilityBars', 'scenarioChartResilienceCurves', 'scenarioChartFaultSequence'].forEach(id => {
          const el = document.getElementById(id);
          if (el && window.Plotly) Plotly.Plots.resize(el);
        });
      }, 50);
      log(`场景生成完成：常规${data.summary?.regular_cluster_count ?? 0}簇，可靠性${data.summary?.reliability_contingency_count ?? 0}个N-1，弹性${data.summary?.resilience_cluster_total ?? 0}簇`, 'success');
      setStatus('场景生成完成');
    });

    function exportGeneratedScenarioFamily(family, label) {
      try {
        const bundle = buildGeneratedScenarioBundle(family);
        const ok = downloadJsonFile(`${family}_generated_scenarios_${tsTagForFilename()}.json`, bundle, { compact: family === 'regular' });
        if (ok) log(`${label}可导入场景已生成：${bundle.case_count} 个代表场景`, 'success');
      } catch (err) {
        log(`${label}场景导出失败：${err.message || err}`, 'warn');
      }
    }

    async function convertScenarioJsonToExcel(file) {
      await showFormatConversionStatus('正在转换 JSON→Excel...');
      log(`正在转换格式：${file.name} → Excel，请稍候...`, 'info');
      try {
        const text = await readFileAsText(file);
        await showFormatConversionStatus('正在解析场景 JSON...');
        const bundle = JSON.parse(text);
        await showFormatConversionStatus('正在生成 Excel 工作簿...');
        const resp = await fetch('/api/session/export_scenario_workbook', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(bundle),
        });
        if (!resp.ok) {
          let msg = 'HTTP ' + resp.status;
          try { const j = await resp.json(); if (j?.error) msg = j.error; } catch (_) {}
          throw new Error(msg);
        }
        const blob = await resp.blob();
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        const stem = file.name.replace(/\.json$/i, '') || 'generated_scenarios';
        a.href = url;
        a.download = `${stem}.xlsx`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        setTimeout(() => URL.revokeObjectURL(url), 1000);
        log(`JSON 已转换为 Excel：${a.download}`, 'success');
        setStatus('格式转换完成');
      } catch (err) {
        log(`JSON 转 Excel 失败：${err.message || err}`, 'error');
        setStatus('格式转换失败', 'error');
      }
    }

    async function convertScenarioExcelToJson(file) {
      await showFormatConversionStatus('正在转换 Excel→JSON...');
      log(`正在转换格式：${file.name} → JSON，请稍候...`, 'info');
      try {
        const bytes = await file.arrayBuffer();
        await showFormatConversionStatus('正在解析 Excel 工作簿...');
        const resp = await fetch('/api/session/import_scenario_workbook', {
          method: 'POST',
          headers: { 'Content-Type': 'application/octet-stream' },
          body: bytes,
        });
        await showFormatConversionStatus('正在生成场景 JSON...');
        const data = await resp.json();
        if (!resp.ok) throw new Error(data.error || 'Excel 转 JSON 失败');
        const stem = file.name.replace(/\.xlsx$/i, '') || 'generated_scenarios';
        const ok = downloadJsonFile(`${stem}.json`, data.bundle || {}, { compact: (data.bundle?.family === 'regular') });
        (data.warnings || []).forEach(w => log(`场景 Excel 警告：${w}`, 'warn'));
        if (ok) log(`Excel 已转换为 JSON：${stem}.json`, 'success');
        setStatus('格式转换完成');
      } catch (err) {
        log(`Excel 转 JSON 失败：${err.message || err}`, 'error');
        setStatus('格式转换失败', 'error');
      }
    }

    document.getElementById('btnExportRegularScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('regular', '常规'));
    document.getElementById('btnExportReliabilityScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('reliability', '可靠性'));
    document.getElementById('btnExportResilienceScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('resilience', '弹性'));
    document.getElementById('btnScenarioJsonToExcel')?.addEventListener('click', () => document.getElementById('fileScenarioJsonToExcel')?.click());
    document.getElementById('fileScenarioJsonToExcel')?.addEventListener('change', e => {
      const file = e.currentTarget.files && e.currentTarget.files[0];
      if (file) convertScenarioJsonToExcel(file);
      e.currentTarget.value = '';
    });
    document.getElementById('btnScenarioExcelToJson')?.addEventListener('click', () => document.getElementById('fileScenarioExcelToJson')?.click());
    document.getElementById('fileScenarioExcelToJson')?.addEventListener('change', e => {
      const file = e.currentTarget.files && e.currentTarget.files[0];
      if (file) convertScenarioExcelToJson(file);
      e.currentTarget.value = '';
    });

    document.getElementById('btnExportScenarioResults')?.addEventListener('click', () => {
      if (!_lastScenarioGenerationData) {
        log('暂无场景生成结果可导出，请先生成场景', 'warn');
        return;
      }
      downloadJsonFile(`scenario_generation_${tsTagForFilename()}.json`, _lastScenarioGenerationData);
    });

    document.getElementById('btnExportScenarioPlots')?.addEventListener('click', async () => {
      if (!window.Plotly) return;
      const ids = ['scenarioChartRegular', 'scenarioChartReliability', 'scenarioChartResilience', 'scenarioChartRegularCurves', 'scenarioChartReliabilityBars', 'scenarioChartResilienceCurves', 'scenarioChartFaultSequence'];
      const tag = tsTagForFilename();
      for (const id of ids) {
        const el = document.getElementById(id);
        if (!el) continue;
        await Plotly.downloadImage(el, { format: 'png', width: 1400, height: 900, filename: `${id}_${tag}` });
        await new Promise(resolve => setTimeout(resolve, 250));
      }
    });

    function bindGeneratedScenarioImport(buttonId, fileId, family) {
      const btn = document.getElementById(buttonId);
      const input = document.getElementById(fileId);
      btn?.addEventListener('click', () => input?.click());
      input?.addEventListener('change', () => {
        const file = input.files && input.files[0];
        if (file) importGeneratedScenarioForModule(file, family);
        input.value = '';
      });
    }
    bindGeneratedScenarioImport('btnImportGeneratedRegularScenario', 'fileImportGeneratedRegularScenario', 'regular');
    bindGeneratedScenarioImport('btnImportGeneratedReliabilityScenario', 'fileImportGeneratedReliabilityScenario', 'reliability');
    bindGeneratedScenarioImport('btnImportGeneratedResilienceScenario', 'fileImportGeneratedResilienceScenario', 'resilience');

    document.getElementById('btnExportResilienceResults')?.addEventListener('click', () => {
      if (!_lastResilienceData) {
        log('暂无弹性评估结果可导出，请先运行弹性评估', 'warn');
        return;
      }
      downloadJsonFile(`resilience_results_${tsTagForFilename()}.json`, _lastResilienceData);
    });

    // ───────── Time-series profile imports (JSON real, Excel placeholder) ─────────
    // Cache parsed profiles client-side so each new import can be merged into a
    // single set_ts_config POST. Backend overwrites `ts_data` on every call.
    const _tsProfileCache = {
      irradiance: null,    // {id, name, values}
      price: null,         // {id, name, values}
      load_profiles: null, // [{load_index, bus, name, values}, ...]
      num_steps: null,
      step_duration_hr: 1.0,
    };
    const TS_PROFILE_ID = { irradiance: 2, price: 50, load_base: 100 };

    async function pushTsConfigToServer(label) {
      const c = _tsProfileCache;
      if (!c.num_steps) {
        log(`${label}：未识别到 num_steps，无法上传`, 'warn');
        return;
      }
      const profiles = [];
      if (c.irradiance) profiles.push({
        id: TS_PROFILE_ID.irradiance, name: c.irradiance.name || 'irradiance',
        values: c.irradiance.values,
      });
      if (c.price) profiles.push({
        id: TS_PROFILE_ID.price, name: c.price.name || 'price',
        values: c.price.values,
      });
      const load_profile_map = [];
      if (c.load_profiles) {
        c.load_profiles.forEach((lp, i) => {
          const pid = TS_PROFILE_ID.load_base + i;
          // The backend formula is: P_t = ld.p_mw * ld.scaling * profile[t].
          // JSON stores absolute MW (lp.p_mw_values), so we convert to a
          // dimensionless scaling = MW[t] / p_mw_nominal here. If
          // p_mw_nominal is missing or zero, fall back to the (legacy)
          // dimensionless `values` field as-is.
          const mw = Array.isArray(lp.p_mw_values) ? lp.p_mw_values
                   : (Array.isArray(lp.values) ? lp.values : null);
          const nom = Number(lp.p_mw_nominal);
          let scaled;
          if (mw && Number.isFinite(nom) && Math.abs(nom) > 1e-12 &&
              Array.isArray(lp.p_mw_values)) {
            scaled = mw.map(v => v / nom);
          } else if (mw) {
            scaled = mw;  // already dimensionless (legacy)
          } else {
            return;
          }
          profiles.push({
            id: pid, name: lp.name || `load_${lp.bus ?? i}`,
            values: scaled,
          });
          if (Number.isInteger(lp.load_index)) {
            load_profile_map.push({ load_index: lp.load_index, profile_id: pid });
          } else if (Number.isInteger(lp.bus)) {
            load_profile_map.push({ bus: lp.bus, profile_id: pid });
          }
        });
      }
      const body = {
        num_steps: c.num_steps,
        step_duration_hr: c.step_duration_hr || 1.0,
        profiles,
        load_profile_map,
        // bind solar PV to irradiance profile when present
        assign_all_pv_to: c.irradiance ? TS_PROFILE_ID.irradiance : -1,
      };
      try {
        const r = await fetch('/api/session/set_ts_config', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        });
        const j = await r.json();
        if (!r.ok) throw new Error(j.error || 'set_ts_config failed');
        const simHr = document.getElementById('simulationHours');
        if (simHr) simHr.value = String(c.num_steps);
        log(`${label}：已上传配置 (num_steps=${j.num_steps}, profiles=${j.num_profiles}, loads_mapped=${j.num_loads_mapped ?? 0})`, 'success');
      } catch (err) {
        log(`${label}：上传失败 ${err.message || err}`, 'error');
      }
    }

    function readFileAsText(file) {
      return new Promise((resolve, reject) => {
        const fr = new FileReader();
        fr.onload = () => resolve(fr.result);
        fr.onerror = () => reject(fr.error);
        fr.readAsText(file, 'utf-8');
      });
    }

    async function handleTsImport(kind, label, file) {
      const lower = file.name.toLowerCase();
      if (!(lower.endsWith('.json'))) {
        log(`${label}：Excel/CSV 导入暂未实现 (TODO)，请使用 JSON 文件 (${file.name})`, 'warn');
        return;
      }
      let j;
      try {
        const text = await readFileAsText(file);
        j = JSON.parse(text);
      } catch (err) {
        log(`${label}：JSON 解析失败 ${err.message || err}`, 'error');
        return;
      }
      const ns = j.num_steps;
      if (!Number.isInteger(ns) || ns <= 0) {
        log(`${label}：JSON 缺少有效的 num_steps`, 'error');
        return;
      }
      if (_tsProfileCache.num_steps && _tsProfileCache.num_steps !== ns) {
        log(`${label}：num_steps=${ns} 与已有配置 num_steps=${_tsProfileCache.num_steps} 不一致，已重置缓存`, 'warn');
        _tsProfileCache.irradiance = _tsProfileCache.price = _tsProfileCache.load_profiles = null;
      }
      _tsProfileCache.num_steps = ns;
      _tsProfileCache.step_duration_hr = j.step_duration_hr || 1.0;

      if (kind === 'irradiance') {
        if (!Array.isArray(j.values) || j.values.length !== ns) {
          log(`${label}：values 数组缺失或长度不等于 ${ns}`, 'error');
          return;
        }
        _tsProfileCache.irradiance = { name: j.name || 'irradiance', values: j.values };
        log(`${label}：已加载 ${file.name} (num_steps=${ns})`, 'info');
      } else if (kind === 'price') {
        if (!Array.isArray(j.values) || j.values.length !== ns) {
          log(`${label}：values 数组缺失或长度不等于 ${ns}`, 'error');
          return;
        }
        _tsProfileCache.price = { name: j.name || 'price', values: j.values };
        log(`${label}：已加载 ${file.name} (num_steps=${ns})`, 'info');
      } else if (kind === 'load_profiles') {
        const lps = j.load_profiles;
        if (!Array.isArray(lps) || lps.length === 0) {
          log(`${label}：load_profiles 缺失或为空`, 'error');
          return;
        }
        let bad = 0;
        for (const lp of lps) {
          const arr = Array.isArray(lp.p_mw_values) ? lp.p_mw_values
                    : (Array.isArray(lp.values) ? lp.values : null);
          if (!arr || arr.length !== ns) bad++;
        }
        if (bad > 0) {
          log(`${label}：${bad} 条负荷曲线长度不等于 ${ns}`, 'error');
          return;
        }
        _tsProfileCache.load_profiles = lps;
        log(`${label}：已加载 ${file.name} (${lps.length} 条负荷曲线, num_steps=${ns})`, 'info');
      }
      await pushTsConfigToServer(label);
    }

    const tsImportPairs = [
      ['btnImportIrradiance',  'fileImportIrradiance',  '辐照强度', 'irradiance'],
      ['btnImportPrice',       'fileImportPrice',       '电价',     'price'],
      ['btnImportLoadProfile', 'fileImportLoadProfile', '负荷波动', 'load_profiles'],
    ];
    tsImportPairs.forEach(([btnId, fileId, label, kind]) => {
      document.getElementById(btnId)?.addEventListener('click', () => {
        document.getElementById(fileId)?.click();
      });
      document.getElementById(fileId)?.addEventListener('change', async (e) => {
        const f = e.target.files[0];
        e.target.value = '';
        if (!f) return;
        await handleTsImport(kind, label, f);
      });
    });

    // Bar 2: module switching
    document.querySelectorAll('.module-btn').forEach(btn => {
      btn.addEventListener('click', () => setActiveModule(btn.dataset.module));
    });

    // Display unit selector — re-render cached PF results on change
    const pfDisplayUnit = document.getElementById('pfDisplayUnit');
    if (pfDisplayUnit) {
      pfDisplayUnit.addEventListener('change', () => {
        if (_lastPfData) {
          showPowerFlowResultsTables(_lastPfData);
        }
        if (Canvas.refreshVisualization) {
          Canvas.refreshVisualization();
        }
      });
    }

    // Bearing Capacity Dialog
    document.getElementById('btnBearingCap')?.addEventListener('click', showBcDialog);
    document.getElementById('btnBcRun').addEventListener('click', runBearingCapacity);
    document.getElementById('btnBcCancel').addEventListener('click', hideBcDialog);

    // SC Dialog
    document.getElementById('btnScRun').addEventListener('click', runShortCircuit);
    document.getElementById('btnScCancel').addEventListener('click', hideScDialog);

    // TSPF Dialog
    document.getElementById('btnTspfRun').addEventListener('click', runTimeSeriesPF);
    document.getElementById('btnTspfCancel').addEventListener('click', hideTspfDialog);
    document.getElementById('tspfProfileSource').addEventListener('change', (e) => {
      document.getElementById('tspfImportRow').style.display =
        e.target.value === 'import' ? 'block' : 'none';
    });

    // Canvas toolbar
    document.getElementById('btnSelect').addEventListener('click', () => {
      Canvas.setMode('select');
      setActiveCanvasTool('btnSelect');
      document.querySelectorAll('.lib-item').forEach(i => i.classList.remove('selected'));
    });
    document.getElementById('btnConnect').addEventListener('click', () => {
      Canvas.setMode('connect');
      setActiveCanvasTool('btnConnect');
    });
    document.getElementById('btnBoxSelect')?.addEventListener('click', () => {
      // Box-select operates in 'select' mode (drag empty area) — keep canvas mode 'select'
      // but mark the box-select button as the active UI tool.
      Canvas.setMode('select');
      setActiveCanvasTool('btnBoxSelect');
    });
    document.getElementById('btnDelete').addEventListener('click', () => {
      if (Canvas.state.selectedIds.size > 1) {
        Canvas.removeSelected();
      } else if (Canvas.state.selectedId !== null) {
        Canvas.removeComponent(Canvas.state.selectedId);
        onTopologyChanged();
      }
    });
    document.getElementById('btnDeleteSelected').addEventListener('click', () => {
      Canvas.removeSelected();
    });
    document.getElementById('btnZoomIn').addEventListener('click', () => Canvas.zoomIn());
    document.getElementById('btnZoomOut').addEventListener('click', () => Canvas.zoomOut());
    document.getElementById('btnZoomFit').addEventListener('click', () => Canvas.zoomFit());
    document.getElementById('btnAutoLayout').addEventListener('click', () => Canvas.autoLayout({ direction: layoutDirection() }));
    document.getElementById('layoutDirSelect')?.addEventListener('change', () => Canvas.autoLayout({ direction: layoutDirection() }));
    // Local re-layout of the current selection (doc §18 Phase 4)
    document.getElementById('btnRelayoutSel')?.addEventListener('click', () => Canvas.autoLayoutSelection?.({ direction: layoutDirection() }));
    // Connection style (直线 / 正交 / 避让) — doc §10.2/§10.3
    const connStyleSel = document.getElementById('connStyleSelect');
    if (connStyleSel) {
      try { connStyleSel.value = localStorage.getItem('connectionStyle') || 'avoid'; } catch (e) {}
      connStyleSel.addEventListener('change', (e) => Canvas.setConnectionStyle?.(e.target.value));
    }
    document.getElementById('btnReroute')?.addEventListener('click', () => Canvas.rerouteConnections?.());
    document.getElementById('chkAlignSnap')?.addEventListener('change', (e) => Canvas.setAlignSnap?.(e.target.checked));
    document.getElementById('btnRotateCW').addEventListener('click', () => Canvas.rotateSelected(90));
    document.getElementById('btnRotateCCW').addEventListener('click', () => Canvas.rotateSelected(-90));

    // Visualization mode toggle
    document.getElementById('vizMode')?.addEventListener('change', (e) => {
      Canvas.setVisualizationMode(e.target.value);
    });

    // Property apply
    document.getElementById('btnApplyProp').addEventListener('click', applyProperties);

    // Tab switching
    document.querySelectorAll('.panel-tab').forEach(tab => {
      tab.addEventListener('click', () => switchTab(tab.dataset.tab));
    });

    // Delegated click-to-canvas mapping for ALL result tables.
    // This is the robust, scope-proof path: it closes over the `Canvas` binding
    // directly instead of relying on inline onclick="" attributes resolving the
    // global lexical `const Canvas` from an HTML event-handler scope. Every
    // result renderer tags mappable rows with data-comp-id (exact component) or
    // data-bus (a bus id); unmappable rows carry neither and stay inert.
    document.getElementById('resultsContent')?.addEventListener('click', (ev) => {
      const compEl = ev.target.closest('[data-comp-id]');
      if (compEl && compEl.dataset.compId !== '') {
        const cid = parseInt(compEl.dataset.compId, 10);
        if (Number.isInteger(cid) && Canvas.panToComponent) {
          Canvas.panToComponent(cid);
          const activeGroup = document.getElementById('resultsContent')?.dataset.activeGroup || '';
          const explicitContext = compEl.dataset.curveContext;
          if (explicitContext || activeGroup === 'timeSeries' || activeGroup === 'scenarioGeneration' || activeGroup === 'resilience') {
            const context = explicitContext || (activeGroup === 'timeSeries' ? 'timeSeries' : (activeGroup === 'resilience' ? 'resilience' : 'scenario'));
            renderSelectedComponentCurve(cid, context);
            return;
          }
        }
      }
      const busEl = ev.target.closest('[data-bus]');
      if (busEl && busEl.dataset.bus !== '') {
        const bid = parseInt(busEl.dataset.bus, 10);
        if (Number.isInteger(bid) && Canvas.panToBusId) Canvas.panToBusId(bid);
      }
    });

    // Console clear
    document.getElementById('btnClearConsole').addEventListener('click', () => {
      document.getElementById('consoleLog').innerHTML = '';
    });

    // Right panel resizer
    {
      const resizer = document.getElementById('rightPanelResizer');
      const panel = document.getElementById('rightPanel');
      let startX, startW;
      const resizePlotlyCharts = () => {
        panel.querySelectorAll('.js-plotly-plot').forEach(el => {
          try { Plotly.Plots.resize(el); } catch (_) {}
        });
      };
      resizer.addEventListener('mousedown', (e) => {
        startX = e.clientX;
        startW = panel.offsetWidth;
        resizer.classList.add('dragging');
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';
        let rafId = 0;
        const onMove = (ev) => {
          const newW = startW - (ev.clientX - startX);
          const activeGroup = document.getElementById('resultsContent')?.dataset.activeGroup || '';
          const minW = activeGroup === 'transient' ? Math.min(window.innerWidth - 360, 720) : 240;
          const maxW = activeGroup === 'transient' ? window.innerWidth * 0.82 : window.innerWidth * 0.6;
          panel.style.width = Math.max(minW, Math.min(maxW, newW)) + 'px';
          // Throttle Plotly resizes to animation frames
          if (!rafId) rafId = requestAnimationFrame(() => { resizePlotlyCharts(); rafId = 0; });
        };
        const onUp = () => {
          resizer.classList.remove('dragging');
          document.body.style.cursor = '';
          document.body.style.userSelect = '';
          document.removeEventListener('mousemove', onMove);
          document.removeEventListener('mouseup', onUp);
          // Final resize after drag ends
          resizePlotlyCharts();
        };
        document.addEventListener('mousemove', onMove);
        document.addEventListener('mouseup', onUp);
      });
    }

    log('Hybrid AC/DC Power System Simulator 已启动', 'success');
    log('使用左侧元件库拖放元件到画布，或加载内置算例', 'info');

    // ---- Toolbar default state ----
    Canvas.setMode('select');
    setActiveCanvasTool('btnSelect');
    setActiveModule('powerFlow');  // default-activate Power Flow module
  }

  // ========== Public API ==========
  return {
    init,
    log,
    setStatus,
    onSelectionChanged,
    onTopologyChanged,
    switchTab,
    setActiveModule,
    setActiveCanvasTool,
    showCaseLoadModal,
    hideCaseLoadModal,
    loadMatpowerCase,
    runPowerFlow,
    runOpf,
  };
})();

// ========== Start ==========
document.addEventListener('DOMContentLoaded', App.init);
