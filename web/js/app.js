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
  let _lastCarbonData = null;
  let _lastDynamicCarbonData = null;
  let _lastReliabilityData = null;
  let _lastResilienceData = null;
  let _lastScenarioGenerationData = null;
  let _lastTopoAnalysisData = null;
  let _lastNetReductionData = null;
  let _lastScenarioBaseSystemJson = null;
  let _scenarioCurveEntries = [];
  let _importedGeneratedScenario = null;
  let _lastImportedGeneratedScenarioKey = '';
  let _generatedScenarioTimeSeriesActive = false;

  function invalidateAnalysisResults(reason = '') {
    _lastPfData = null;
    _lastOpfData = null;
    _lastTspfData = null;
    _lastCarbonData = null;
    _lastDynamicCarbonData = null;
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
    if (rc) rc.dataset.activeGroup = name || '';
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
        return { demand, served, shed, tier, importance };
      };
      if (type === 'load' || type === 'dc_load') {
        const isDc = type === 'dc_load';
        const row = modelRowForComp(isDc ? (sys.dc?.loads || []) : (sys.ac?.loads || []), isDc ? maps.dcLoad : maps.load, compId);
        const bus = row.item?.bus ?? comp.params?.bus;
        const s = findBusSeries(isDc ? 'DC' : 'AC', bus);
        const pr = Number.isInteger(s.tier) ? priorityNames[Math.max(0, Math.min(3, s.tier))] : '—';
        const note = `负荷优先级：${pr}${Number.isFinite(s.importance) ? `（重要度 ${s.importance.toFixed(1)}）` : ''}`;
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
      const total = (sys.ac?.loads || []).reduce((a, x) => a + numberOr(x.p_mw, 0) * numberOr(x.scaling, 1), 0) + (sys.dc?.loads || []).reduce((a, x) => a + numberOr(x.p_mw, 0) * numberOr(x.scaling, 1), 0);
      const base = scaledCurve(load, numberOr(row.item?.p_mw ?? comp.params?.p_mw, 0) * numberOr(row.item?.scaling ?? comp.params?.scaling, 1), total);
      const profile = baseDeterministicProfile(base.length, Number(row.item?.index ?? row.index ?? compId));
      const y = base.map((v, i) => v * profile[i]);
      return make(y, `场景${isDc ? 'DC' : 'AC'}负荷估算曲线：${label}`, '按该负荷容量占比叠加确定性随机波动，避免所有负荷共用完全相同形状。');
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
      font: { color: whiteChart ? '#111827' : '#abb2bf', size: 11 }, margin: { l: 55, r: 15, t: 42, b: info.note ? 68 : 40 },
      xaxis: { ...axisTheme, title: context === 'resilience' ? '小时' : '时间步' },
      yaxis: { ...axisTheme, title: 'MW', ...(range ? { range } : {}) },
      title: info.title,
      annotations: info.note ? [{ text: info.note, x: 0, y: -0.28, xref: 'paper', yref: 'paper', showarrow: false, align: 'left', font: { size: 10, color: whiteChart ? '#92400e' : '#d19a66' } }] : [],
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

  // ========== API Client ==========
  async function apiPost(path, body = {}) {
    const url = `${API_BASE}${path}`;
    log(`POST ${path}`, 'info');
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const data = await res.json();
      if (!res.ok) {
        log(`Error: ${data.error || res.statusText}`, 'error');
        return null;
      }
      return data;
    } catch (e) {
      log(`Network error: ${e.message}`, 'error');
      return null;
    }
  }

  // Like apiPost but surfaces the backend error message instead of swallowing
  // it.  Returns { ok, data, error } so callers can show a specific reason
  // (e.g. an unknown fault bus id rejected by the server).
  async function apiPostResult(path, body = {}) {
    const url = `${API_BASE}${path}`;
    log(`POST ${path}`, 'info');
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const data = await res.json();
      if (!res.ok) {
        const error = (data && data.error) || res.statusText;
        log(`Error: ${error}`, 'error');
        return { ok: false, data: null, error };
      }
      return { ok: true, data, error: null };
    } catch (e) {
      log(`Network error: ${e.message}`, 'error');
      return { ok: false, data: null, error: e.message };
    }
  }

  async function apiGet(path) {
    const url = `${API_BASE}${path}`;
    try {
      const res = await fetch(url);
      const data = await res.json();
      if (!res.ok) {
        log(`Error: ${data.error || res.statusText}`, 'error');
        return null;
      }
      return data;
    } catch (e) {
      log(`Network error: ${e.message}`, 'error');
      return null;
    }
  }

  function firstNonEmptyArray(...values) {
    for (const value of values) {
      if (Array.isArray(value) && value.length > 0) return value;
    }
    return [];
  }

  function normalizePowerFlowResult(data) {
    if (!data || typeof data !== 'object') return data;
    data.geo_dc_branches = firstNonEmptyArray(data.geo_dc_branches, data.dc_branch_flows);
    data.dc_branch_flows = firstNonEmptyArray(data.dc_branch_flows, data.geo_dc_branches);
    data.vsc_transfers = firstNonEmptyArray(data.vsc_transfers, data.geo_vsc);
    data.geo_vsc = firstNonEmptyArray(data.geo_vsc, data.vsc_transfers);
    data.dcdc_transfers = firstNonEmptyArray(data.dcdc_transfers, data.geo_dcdc);
    data.geo_dcdc = firstNonEmptyArray(data.geo_dcdc, data.dcdc_transfers);
    data.ac_switch_flows = Array.isArray(data.ac_switch_flows) ? data.ac_switch_flows : [];
    data.ac_circuit_breaker_flows = Array.isArray(data.ac_circuit_breaker_flows) ? data.ac_circuit_breaker_flows : [];
    data.dc_circuit_breaker_flows = Array.isArray(data.dc_circuit_breaker_flows) ? data.dc_circuit_breaker_flows : [];
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

  // ========== Load Built-in Cases ==========
  async function loadCaseList() {
    const data = await apiGet('/api/cases');
    if (!data) return;
    const select = document.getElementById('caseSelect');
    select.innerHTML = '<option value="">-- 加载算例 --</option>';
    (data.cases || []).forEach(c => {
      const opt = document.createElement('option');
      opt.value = c;
      opt.textContent = c;
      select.appendChild(opt);
    });
    log(`已加载 ${(data.cases || []).length} 个内置算例`, 'success');
  }

  async function loadMatpowerFileList() {
    const data = await apiGet('/api/matpower_files');
    if (!data) return;
    const select = document.getElementById('matpowerSelect');
    select.innerHTML = '<option value="">-- MATPOWER文件 --</option>';
    (data.files || []).forEach(f => {
      const opt = document.createElement('option');
      opt.value = f;
      opt.textContent = f;
      select.appendChild(opt);
    });
    log(`已加载 ${(data.files || []).length} 个MATPOWER文件`, 'success');
  }

  async function loadMatpowerCase(filename) {
    if (!filename) return;
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
    } else {
      setStatus('加载失败', 'error');
    }
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
      setStatus('就绪');
    } catch (err) {
      log(`导出ETAP XML失败: ${err.message}`, 'error');
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
            Canvas.loadFromSystemJson(raw);
          } else {
            Canvas.loadFromSystemJson(sys);
          }
          _canvasDirty = false;  // backend already has the imported system
          updateResilienceSwitchDefault();
          invalidateAnalysisResults('系统已导入，旧潮流和碳流结果已失效');
          log('已导入系统JSON', 'success');
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
    console.log('[syncToBackend] canvas JSON length:', jsonStr.length);
    console.log('[syncToBackend] DCDC converters:', JSON.stringify(sys.dcdc_converters, null, 2));
    console.log('[syncToBackend] VSC converters:', JSON.stringify(sys.vsc_converters, null, 2));
    console.log('[syncToBackend] DC buses:', JSON.stringify(sys.dc?.buses, null, 2));
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
      const demand = Number(l.demand_mw || 0);
      const emissions = Math.max(0, Number(l.total_emissions_tco2 || 0));
      loadHtml += `<tr${cbClick(l.bus, l.is_dc)}><td>${l.is_dc ? 'DC' : 'AC'}</td><td>${l.load_index}</td><td>${l.bus}</td><td>${demand.toFixed(4)}</td><td>${carbonIntensityDisplay(l.carbon_intensity_tco2_mwh).toFixed(3)}</td><td>${emissions.toFixed(6)}</td></tr>`;
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
        html += `<tr><td>${s.is_dc ? 'DC' : 'AC'}</td><td>${s.storage_index}</td><td>${s.bus}</td><td>${Number(s.p_mw || 0).toFixed(4)}</td><td>${Number(s.soc || 0).toFixed(4)}</td><td>${carbonIntensityDisplay(s.soc_carbon_intensity_tco2_mwh).toFixed(3)}</td><td>${Math.max(0, Number(s.total_emissions_tco2 || 0)).toFixed(6)}</td></tr>`;
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
  async function runPowerFlow() {
    setStatus('潮流计算中...', 'busy');
    const method = document.getElementById('pfMethod').value;

    // Sync canvas to backend first
    if (!await syncToBackend()) {
      setStatus('同步失败', 'error');
      return;
    }

    const coordCheckEl = document.getElementById('pfCoordCheck');
    const data = await apiPost('/api/session/pf', {
      method: method,
      options: {
        max_iter: 100,
        tol: 1e-8,
        verbose: false,
        enable_converter_coordination_check: coordCheckEl ? coordCheckEl.checked : true
      }
    });

    if (data) {
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
      Canvas.showPowerFlowResults(pfData);
      showPowerFlowResultsTables(pfData);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
  }

  // ========== Optimal Power Flow ==========
  // Reads the OPF sub-toolbar (solver + 4 constraint families) and posts to the
  // unified /api/session/opf endpoint, which routes to the parity-IPM hybrid
  // AC/DC formulation (default), the AC OPF, or the DC OPF. The constraint
  // checkboxes gate branch thermal limits, converter capacity circles,
  // converter AC/DC current limits, and converter modulation-ratio limits, all
  // of which are honoured by the parity manual-KKT formulation.
  async function runOpf() {
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
      return;
    }

    const data = await apiPost('/api/session/opf', { solver, constraints, check_consistency: checkConsistency });
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
      // post-OPF branch flows, so wire them into geo_ac_branches to enable the
      // flow / heat-map visualization at the OPF operating point.
      if (data.post_pf && Array.isArray(data.post_pf.branch_flows)) {
        data.geo_ac_branches = data.post_pf.branch_flows.map(b => ({
          from: b.from_bus, to: b.to_bus,
          pf_mw: b.pf_mw, pt_mw: b.pt_mw,
          loading_pct: b.loading_pct || 0, rate_mva: 0,
        }));
      }
      if (Canvas.showPowerFlowResults) Canvas.showPowerFlowResults(data);
      showOpfResults(data);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
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
    // Generator / converter maps are keyed by component index, which is 1-based
    // for imported cases (MATPOWER) but 0-based for components drawn from
    // scratch. Detect the base once so position-ordered OPF result rows resolve
    // to the right component either way.
    const keyBase = (m) => (m && m[0] !== undefined) ? 0 : 1;
    const genBase = keyBase(busMap.gen);
    const vscBase = keyBase(busMap.vsc);

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
      if (pg.length) {
        let html = '<table><thead><tr><th>#</th><th>Pg(MW)</th><th>Qg(MVar)</th></tr></thead><tbody>';
        pg.forEach((p, i) => {
          const attr = panAttr(busMap.gen ? busMap.gen[i + genBase] : undefined);
          html += `<tr${attr}><td>${i + 1}</td><td>${fmt(p)}</td><td>${fmt(qg[i])}</td></tr>`;
        });
        html += '</tbody></table>';
        genDiv.innerHTML = html;
      } else {
        genDiv.innerHTML = '<p class="empty-hint">无发电机出力数据</p>';
      }
    }

    // Converter (VSC) dispatch — only shown for hybrid AC/DC cases.
    const convSec = document.getElementById('opfConvSection');
    const convDiv = document.getElementById('opfConvResults');
    const pac = data.pac_mw || [];
    const qac = data.qac_mvar || [];
    if (convSec && convDiv && pac.length) {
      convSec.style.display = '';
      let html = '<table><thead><tr><th>VSC#</th><th>Pac(MW)</th><th>Qac(MVar)</th></tr></thead><tbody>';
      pac.forEach((p, i) => {
        const attr = panAttr(busMap.vsc ? busMap.vsc[i + vscBase] : undefined);
        html += `<tr${attr}><td>${i + 1}</td><td>${fmt(p)}</td><td>${fmt(qac[i])}</td></tr>`;
      });
      html += '</tbody></table>';
      convDiv.innerHTML = html;
    } else if (convSec) {
      convSec.style.display = 'none';
    }

    // DC bus voltages — only for hybrid AC/DC cases.
    const dcSec = document.getElementById('opfDcBusSection');
    const dcDiv = document.getElementById('opfDcBusResults');
    const vdc = data.vdc || [];
    if (dcSec && dcDiv && vdc.length) {
      dcSec.style.display = '';
      const dcBusIdAt = (i) => {
        const id = SYS?.dc?.buses?.[i]?.index;
        return id !== undefined && id !== null ? Number(id) : null;
      };
      let html = '<table><thead><tr><th>DC Bus</th><th>Vdc(pu)</th></tr></thead><tbody>';
      vdc.forEach((v, i) => {
        const busId = dcBusIdAt(i);
        const attr = panAttr(busId != null && busMap.dc ? busMap.dc[busId] : undefined);
        html += `<tr${attr}><td>${busId ?? `pos ${i}`}</td><td>${fmt(v, 6)}</td></tr>`;
      });
      html += '</tbody></table>';
      dcDiv.innerHTML = html;
    } else if (dcSec) {
      dcSec.style.display = 'none';
    }

    // AC bus voltages and locational marginal prices.
    const busDiv = document.getElementById('opfBusResults');
    const vm = data.vm || [];
    if (busDiv && vm.length) {
      const va = data.va || [];
      const lmpP = data.lmp_p || [];
      const lmpQ = data.lmp_q || [];
      const hasLmp = lmpP.length > 0;
      const acBusIdAt = (i) => {
        const id = SYS?.ac?.buses?.[i]?.index;
        return id !== undefined && id !== null ? Number(id) : null;
      };
      let html = `<table><thead><tr><th>Bus</th><th>Vm(pu)</th><th>Va(°)</th>${hasLmp ? '<th>LMP-P</th><th>LMP-Q</th>' : ''}</tr></thead><tbody>`;
      vm.forEach((v, i) => {
        const busId = acBusIdAt(i);
        const ang = va[i] != null ? (va[i] * 180 / Math.PI).toFixed(4) : '0';
        const color = v < 0.95 ? 'color:#e06c75' : v > 1.05 ? 'color:#d19a66' : '';
        const attr = panAttr(busId != null && busMap.ac ? busMap.ac[busId] : undefined);
        html += `<tr${attr}><td>${busId ?? `pos ${i}`}</td><td style="${color}">${fmt(v, 6)}</td><td>${ang}</td>${hasLmp ? `<td>${fmt(lmpP[i])}</td><td>${fmt(lmpQ[i])}</td>` : ''}</tr>`;
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
    if (postPfSec && postPfDiv && postPf && Array.isArray(postPf.branch_flows) && postPf.branch_flows.length) {
      postPfSec.style.display = '';
      const busMapAc = busMap.ac || {};
      let html = '<table><thead><tr><th>支路</th><th>P_from(MW)</th><th>Q_from(MVar)</th><th>P_to(MW)</th><th>负载率(%)</th></tr></thead><tbody>';
      postPf.branch_flows.forEach(b => {
        const ld = b.loading_pct || 0;
        const color = ld > 100 ? 'color:#e06c75' : ld > 80 ? 'color:#d19a66' : '';
        const attr = panAttr(busMapAc[b.from_bus]);
        html += `<tr${attr}><td>${b.from_bus}→${b.to_bus}</td><td>${fmt(b.pf_mw)}</td><td>${fmt(b.qf_mvar)}</td><td>${fmt(b.pt_mw)}</td><td style="${color}">${fmt(ld, 1)}</td></tr>`;
      });
      html += '</tbody></table>';
      html += '<p class="empty-hint" style="margin-top:6px">提示：使用画布上方的「可视化」下拉(潮流/热力图)可在 OPF 解上叠加支路潮流与负载率热力图。</p>';
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

  async function runShortCircuit() {
    hideScDialog();
    setStatus('短路计算中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const faultBusRaw = (document.getElementById('scFaultBus').value || '').trim();
    const faultType = document.getElementById('scFaultType').value;
    const cFactor = parseFloat(document.getElementById('scCFactor').value);

    // Blank fault bus → short circuit at every bus (overview mode).
    if (faultBusRaw === '') {
      const data = await apiPost('/api/session/sc', {
        options: {
          fault_type: faultType,
          c_factor: cFactor,
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
      fault_type: faultType,
      c_factor: cFactor,
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
      const rows = (data.bridges || []).map(b =>
        `<tr><td class="topo-clickable" data-bus="${b.from_bus}">${b.from_bus}</td>` +
        `<td class="topo-clickable" data-bus="${b.to_bus}">${b.to_bus}</td>` +
        `<td>${esc(b.category || '')}</td><td>${b.domain || ''}</td></tr>`).join('');
      brEl.innerHTML = rows
        ? `<table class="topo-table"><thead><tr><th>起始母线</th><th>终止母线</th><th>类型</th><th>域</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<p class="empty-hint">无桥支路（无单点故障支路）</p>';
    }

    // ── Cut vertices table ──
    const cvEl = document.getElementById('topoAnalysisCutVertices');
    if (cvEl) {
      const ids = data.cut_vertex_bus_ids || [];
      cvEl.innerHTML = ids.length
        ? '<div class="topo-cutvertex-chips">' + ids.map(id =>
            `<span class="legend-item topo-clickable" data-bus="${id}">母线 ${id}</span>`).join(' ') + '</div>'
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
    if (_generatedScenarioTimeSeriesActive) return true;
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

    const data = await apiPost('/api/session/run_ts_pf', {
      num_steps: numSteps,
      skip_uc: skipUC,
      run_opf: runOPF,
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

  function showTimeSeriesResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('timeSeries');
    renderComponentCurveTargets('timeSeries');

    const section = document.getElementById('tspfResultsSection');
    section.style.display = 'block';

    // Summary KPIs
    const summary = document.getElementById('tspfSummary');
    const ucStatus = data.uc_feasible === false ? '失败' : (data.uc_feasible === true ? '成功' : '—');
    const ucClass = data.uc_feasible === false ? 'result-failed' : 'result-converged';
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
      <div class="result-item"><span class="result-label">总发电成本</span>
        <span class="result-value">$${(data.total_generation_cost || 0).toFixed(0)}</span></div>
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

  function renderAllPowerFlowComponentStatus(data, busMap) {
    data = normalizePowerFlowResult(data);
    const section = document.getElementById('pfAllComponentsSection');
    const div = document.getElementById('pfAllComponentsResults');
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
        case 'dc_storage':
          return { p: toNum(p.p_mw) ?? 0, q: 0 };
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
          const buses = connectedBuses(comp.id);
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
        case 'storage':
        case 'mobile_storage': {
          detail = [
            `P计算=${fmtP(p.p_mw, 3)}`,
            `Q计算=${fmtQ(p.q_mvar, 3)}`,
            `P额定=${fmtP(p.p_rated_mw, 3)}`,
            `E额定=${fmt(p.e_rated_mwh, 3)} MWh`,
            `SOC=${fmt(p.soc_init, 4)}`,
            comp.type === 'mobile_storage' ? `移动状态=${p.status || '-'}` : '',
            comp.type === 'mobile_storage' ? `目标母线=${p.target_bus || '-'}` : '',
          ];
          solved = data.converged;
          note.push('本次潮流计算采用的储能注入；正值放电，负值充电');
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
            detail = er.ports.map(pt => `P${pt.port_index}@${pt.is_ac ? 'AC' : 'DC'} Bus ${pt.bus}: P=${fmtP(pt.p_mw, 3)}, V=${fmt(pt.v_pu, 4)} pu`);
            detail.push(`Loss=${fmtP(er.loss_mw, 3)}`);
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

  function showPowerFlowBalanceDiagnostics(data, busMap) {
    const section = document.getElementById('pfBalanceSection');
    const div = document.getElementById('pfBalanceResults');
    if (!section || !div) return;

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

    (data.geo_ac_branches || []).forEach(br => {
      add(ac, br.from, br.pf_mw, `AC支路 ${br.from}->${br.to} Pf`);
      add(ac, br.to, br.pt_mw, `AC支路 ${br.from}->${br.to} Pt`);
    });
    const dcBranches = firstNonEmptyArray(data.geo_dc_branches, data.dc_branch_flows);
    dcBranches.forEach(br => {
      const from = br.from ?? br.from_bus;
      const to = br.to ?? br.to_bus;
      add(dc, from, br.pf_mw, `DC支路 ${from}->${to} Pf`);
      add(dc, to, br.pt_mw, `DC支路 ${from}->${to} Pt`);
    });
    const vscRows = firstNonEmptyArray(data.geo_vsc, data.vsc_transfers);
    vscRows.forEach(v => {
      add(ac, v.bus_ac, -num(v.p_ac_mw), `VSC ${v.index ?? ''} AC侧`);
      add(dc, v.bus_dc, -num(v.p_dc_mw), `VSC ${v.index ?? ''} DC侧`);
    });
    const dcdcRows = firstNonEmptyArray(data.geo_dcdc, data.dcdc_transfers);
    dcdcRows.forEach(d => {
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
    (sys.ac?.transformers_2w || []).forEach(tf => {
      if (!isOn(tf) || tf.source_branch_idx > 0) return;
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

    (sys.ac?.loads || []).forEach(x => { if (isOn(x)) add(ac, x.bus, num(x.p_mw) * num(x.scaling, 1), `负荷 ${x.index ?? ''}`); });
    (sys.ac?.charging_stations || []).forEach(x => { if (isOn(x)) add(ac, x.bus, num(x.p_total_kw) / 1000, `充电站 ${x.index ?? ''}`); });
    (sys.ac?.chargers || []).forEach(x => { if (isOn(x)) add(ac, x.bus, num(x.p_ch_kw) / 1000, `充电桩 ${x.index ?? ''}`); });
    (sys.ac?.generators || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.pg_mw ?? x.p_mw), `发电机 ${x.index ?? ''}`); });
    (sys.ac?.pv_systems || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.p_mw), `光伏 ${x.index ?? ''}`); });
    (sys.ac?.renewable_gens || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.p_mw), `新能源 ${x.index ?? ''}`); });
    (sys.ac?.static_generators || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.p_mw ?? x.p_set_mw), `静态电源 ${x.index ?? ''}`); });
    (sys.ac?.storage || []).forEach(x => { if (isOn(x)) add(ac, x.bus, -num(x.p_mw), `储能 ${x.index ?? ''}`); });
    (sys.vpps || []).forEach(x => { if (isOn(x)) add(ac, x.pcc_bus ?? x.bus, -num(x.p_output_mw ?? x.p_mw), `虚拟电厂 ${x.index ?? ''}`); });
    (sys.microgrids || []).forEach(x => { if (isOn(x) && x.operating_mode !== 'Islanded') add(ac, x.pcc_bus, -num(x.p_exchange_mw), `微网 ${x.index ?? ''}`); });
    (sys.dc?.loads || []).forEach(x => { if (isOn(x)) add(dc, x.bus, num(x.p_mw) * num(x.scaling, 1), `DC负荷 ${x.index ?? ''}`); });
    (sys.dc?.pv_arrays || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw ?? x.p_set_mw), `DC光伏 ${x.index ?? ''}`); });
    (sys.dc?.static_generators || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw ?? x.p_set_mw), `DC静态电源 ${x.index ?? ''}`); });
    (sys.dc?.dc_static_generators || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw ?? x.p_set_mw), `DC静态电源 ${x.index ?? ''}`); });
    (sys.dc?.dc_storage || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw), `DC储能 ${x.index ?? ''}`); });
    (sys.dc?.storage || []).forEach(x => { if (isOn(x)) add(dc, x.bus, -num(x.p_mw), `DC储能 ${x.index ?? ''}`); });

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
      (sys.dc?.dcdc_converters || []).forEach(d => {
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
    const dcSlack = dcSlackSet();
    const tolKw = 1.0;
    const rows = [];
    const collect = (kind, map, buses, slackSet, implicitSet = new Set()) => {
      (buses || []).forEach(b => {
        if (!isOn(b)) return;
        const id = Number(b.index);
        const rec = map.get(id) || { mw: 0, items: [] };
        const kw = rec.mw * 1000;
        const isSlack = slackSet.has(id);
        const isImplicit = implicitSet.has(id);
        if (!isSlack && Math.abs(kw) <= tolKw) return;
        rows.push({
          kind, id, kw, isSlack, isImplicit,
          type: b.bus_type || '',
          name: busName(kind, id),
          details: rec.items.sort((a, b) => Math.abs(b.mw) - Math.abs(a.mw)).slice(0, 4),
        });
      });
    };
    collect('AC', ac, sys.ac?.buses || [], acSlack);
    collect('DC', dc, sys.dc?.buses || [], dcSlack.slacks, dcSlack.implicit);

    const bad = rows.filter(r => !r.isSlack && Math.abs(r.kw) > tolKw);
    const sources = rows.filter(r => r.isSlack && Math.abs(r.kw) > tolKw);
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
    `;

    const busMap = Canvas.getCompBusMap();
    showPowerFlowBalanceDiagnostics(data, busMap);

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

    renderAllPowerFlowComponentStatus(data, busMap);

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
        const compId = busId != null ? busMap.ac[busId] : undefined;
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
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
        const compId = busId != null ? busMap.dc[busId] : undefined;
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
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
        const compId = busMap.gen ? (busMap.gen[g.index] ?? busMap.gen[i]) : undefined;
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
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
        // Branches have no positional canvas id in the result, so link the row
        // to one endpoint bus (the from-bus), which maps reliably via busMap.ac.
        const compId = (busMap.branch && busMap.branch[br.index] !== undefined)
          ? busMap.branch[br.index]
          : (busMap.ac ? busMap.ac[br.from] : undefined);
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
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
        const compId = busMap.branch[i];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
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
        const compId = busMap.dcBranch
          ? (busMap.dcBranch[br.index] ?? busMap.dcBranch[i])
          : undefined;
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
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
        const ldg = tf.loading_pct != null ? tf.loading_pct.toFixed(1) + '%' : '-';
        const ldgStyle = (tf.loading_pct || 0) > 100 ? ' style="color:#e06c75;font-weight:bold"' : '';
        html += `<tr><td>${tf.index ?? i}</td><td>${tf.hv_bus}</td><td>${tf.mv_bus}</td><td>${tf.lv_bus}</td>`;
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
        html += `<tr><td>${d.index ?? i}</td><td>${d.bus_in}</td><td>${d.bus_out}</td>`;
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
        const compId = busMap.vsc ? (busMap.vsc[v.index] ?? busMap.vsc[i]) : undefined;
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
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
  // Only these solvers co-solve AC + DC + converters as one coupled system.
  // The others are AC-only (pure_ac/fdpf/three_phase), DC-only (dc), a one-shot
  // linearization (hybrid_linearized), or use a different reference scheme
  // (distributed_slack) — so they give non-comparable answers on a hybrid case.
  const HYBRID_PF_METHODS = new Set(['ac_newton', 'adaptive', 'islanded']);
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
      opt.title = restrict ? '该算法不联立求解直流网络/变换器，混合交直流算例结果不可比' : '';
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
      vsc_converter:  { r_conv_ac_pu: '约束限值 (OPF)', grid_forming: '构网与协调' },
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

      if (typeof val === 'boolean') {
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
    fields.forEach(el => {
      const key = el.dataset.field;
      let val = el.value;
      // Type conversion
      if (val === 'true') val = true;
      else if (val === 'false') val = false;
      else if (el.type === 'number' && val !== '') val = parseFloat(val);
      if (key === 'emission_factor_tco2_mwh') val = carbonFactorInputValue(val);
      const oldVal = comp.params[key];
      const sameNumber = typeof oldVal === 'number' && typeof val === 'number' &&
        Math.abs(oldVal - val) < 1e-12;
      if (!sameNumber && oldVal !== val) changedKeys.add(key);
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

    // Calculation buttons (Bar 3 "运行..." buttons reuse original IDs where possible)
    document.getElementById('btnPowerFlow').addEventListener('click', runPowerFlow);
    document.getElementById('btnRunOpf')?.addEventListener('click', runOpf);
    document.getElementById('btnCarbonFlow')?.addEventListener('click', runCarbonFlow);
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
    document.getElementById('faultTypeSelect')?.addEventListener('change', (e) => {
      const f = document.getElementById('scFaultType');
      if (f) f.value = e.target.value;
    });
    document.getElementById('voltageCorrectionFactor')?.addEventListener('change', (e) => {
      const f = document.getElementById('scCFactor');
      if (f) f.value = e.target.value;
    });
    document.getElementById('btnRunShortCircuit')?.addEventListener('click', () => {
      // Mirror sub-toolbar values into legacy dialog inputs, then run.
      const ft = document.getElementById('faultTypeSelect')?.value;
      const cf = document.getElementById('voltageCorrectionFactor')?.value;
      if (ft) document.getElementById('scFaultType').value = ft;
      if (cf) document.getElementById('scCFactor').value = cf;
      runShortCircuit();
    });

    // Bar 3: harmonic power flow
    document.getElementById('btnRunHarmonics')?.addEventListener('click', runHarmonics);
    document.getElementById('hpfMode')?.addEventListener('change', hpfUpdateModeControls);
    hpfUpdateModeControls();

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

    // Bar 3: Reliability — run (NSQ/SEQ/FMEA/F&D) + export. Backend live.
    async function runReliability() {
      setStatus('可靠性分析中...', 'busy');
      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
      const method = (document.getElementById('relMethod')?.value) || 'nsq';
      const maxIter = parseInt(document.getElementById('relMaxIter')?.value) || 2000;
      const epMap = { nsq: 'run_reliability_nsq', seq: 'run_reliability_seq',
                      fmea: 'run_reliability_fmea', fd: 'run_reliability_fd' };
      const opts = {};
      if (method === 'nsq') opts.max_iterations = maxIter;
      else if (method === 'seq') { opts.max_years = Math.max(50, Math.round(maxIter / 10)); opts.hours_per_year = 8736; }
      else if (method === 'fmea') { opts.load_scale_factor = 1.0; opts.apply_comprehensive_data = true; }
      const data = await apiPost('/api/session/' + epMap[method], opts);
      if (data && !data.error) {
        _lastReliabilityData = Object.assign({ _method: method }, data);
        showReliabilityResults(data, method);
        switchTab('results');
        setStatus('可靠性分析完成');
      } else {
        setStatus('计算失败', 'error');
      }
    }

    function showReliabilityResults(data, method) {
      document.getElementById('resultsEmpty').style.display = 'none';
      document.getElementById('resultsContent').style.display = 'block';
      setActiveResultGroup('reliability');
      const nf = (v, d = 2) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(d) : '—';
      const methodLabel = { nsq: '非序贯蒙特卡洛', seq: '序贯蒙特卡洛', fmea: 'FMEA (N-1)', fd: '频率-持续时间' }[method] || method;
      let kpis = [];
      if (method === 'fd') {
        kpis = [
          ['LOLP (失负荷概率)', nf(data.lolp, 6)],
          ['LOLE (h/yr)', nf(data.lole_fd, 2)],
          ['LOLF (occ/yr)', nf(data.lolf_fd, 2)],
          ['LOLD (h/occ)', nf(data.lold, 2)],
        ];
      } else {
        kpis = [
          ['EENS (MWh/yr)', nf(data.eens_mwh_yr, 1)],
          ['EDNS (MW)', nf(data.edns_mw, 3)],
          ['LOLE (h/yr)', nf(data.lole_hr_yr, 2)],
          ['LOLF (occ/yr)', nf(data.lolf_occ_yr, 2)],
          ['PLC (%)', nf((data.plc ?? 0) * 100, 3)],
        ];
        if (method === 'fmea') {
          kpis.push(['SAIFI', nf(data.saifi, 4)], ['SAIDI', nf(data.saidi, 4)], ['ASAI', nf(data.asai, 6)]);
          if (data.n_contingencies != null) kpis.push(['故障枚举数', `${data.n_contingencies} (${data.n_with_loss ?? '?'} 含失负荷)`]);
        } else {
          kpis.push(['收敛', data.converged ? '✓ 是' : '✗ 否'], ['迭代/年数', data.iterations_used ?? '—'], ['最终 CoV', nf(data.final_cov, 4)]);
        }
      }
      let html = `<div style="margin-bottom:8px;"><b>方法：</b>${methodLabel}</div>`;
      html += '<table><thead><tr><th>指标</th><th>数值</th></tr></thead><tbody>';
      for (const [k, v] of kpis) html += `<tr><td>${k}</td><td class="result-value">${v}</td></tr>`;
      html += '</tbody></table>';

      // Convergence chart (NSQ/SEQ)
      if ((method === 'nsq' || method === 'seq') && Array.isArray(data.eens_history) && data.eens_history.length) {
        html += '<h4 style="margin:10px 0 4px;">EENS 收敛过程</h4><div id="relConvChart" style="height:240px;"></div>';
      }
      // Critical components (NSQ/SEQ) — index is 0-based positional (generator
      // or branch), the same space PF uses for busMap.gen[i] / busMap.branch[i].
      if (Array.isArray(data.critical_components) && data.critical_components.length) {
        html += '<h4 style="margin:10px 0 4px;">薄弱元件 (Top)</h4><table><thead><tr><th>#</th><th>类型</th><th>索引</th><th>重要度</th></tr></thead><tbody>';
        const ccBusMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;
        data.critical_components.slice(0, 15).forEach((c, i) => {
          const compId = ccBusMap ? (c.is_generator ? (ccBusMap.gen ? ccBusMap.gen[c.index] : undefined)
                                                     : (ccBusMap.branch ? ccBusMap.branch[c.index] : undefined)) : undefined;
          const clk = compId != null
            ? ` class="topo-clickable" data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
          html += `<tr${clk}><td>${i + 1}</td><td>${c.is_generator ? '发电机' : '支路'}</td><td>${c.index ?? '—'}</td><td>${nf(c.importance, 4)}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      // FMEA top contingencies
      if (method === 'fmea' && Array.isArray(data.contingencies) && data.contingencies.length) {
        html += '<h4 style="margin:10px 0 4px;">关键故障 (按 EENS 贡献)</h4><table><thead><tr><th>元件</th><th>类型</th><th>EENS贡献(MWh/yr)</th><th>切负荷(MW)</th></tr></thead><tbody>';
        const relBusMap = (typeof Canvas !== 'undefined' && Canvas.getCompBusMap) ? Canvas.getCompBusMap() : null;
        data.contingencies.slice(0, 15).forEach(c => {
          const clk = busClickAttr(busIdFromComponentName(c.component_name), relBusMap);
          html += `<tr${clk}><td>${c.component_name ?? '—'}</td><td>${c.component_type ?? '—'}</td><td>${nf(c.eens_contribution, 2)}</td><td>${nf(c.shed_mw, 2)}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      document.getElementById('reliabilityResults').innerHTML = html;

      if ((method === 'nsq' || method === 'seq') && typeof Plotly !== 'undefined' && Array.isArray(data.eens_history) && data.eens_history.length) {
        const x = data.eens_history.map((_, i) => i + 1);
        Plotly.newPlot('relConvChart', [{ x, y: data.eens_history, mode: 'lines', name: 'EENS', line: { color: '#61afef' } }],
          { margin: { l: 55, r: 10, t: 10, b: 35 }, xaxis: { title: '样本批次' }, yaxis: { title: 'EENS (MWh/yr)' },
            paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', font: { color: '#dcdfe4' } }, { responsive: true });
      }
    }

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
          html += `<tr${busClickAttr(l.bus, cbBusMap)}><td>${l.load_index ?? '—'}</td><td>${l.bus ?? '—'}</td><td>${nf(l.demand_mw, 3)}</td>`
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
      const manual_faults = [];
      acIds.forEach((id, i) => manual_faults.push({ branch_type: 'AC', branch_id: id, start_hr: atOr(acStarts, i, 0), repair_hr: atOr(acRepairs, i, 6), label: `AC branch ${id}` }));
      dcIds.forEach((id, i) => manual_faults.push({ branch_type: 'DC', branch_id: id, start_hr: atOr(dcStarts, i, 0), repair_hr: atOr(dcRepairs, i, 8), label: `DC branch ${id}` }));
      const latestEnd = manual_faults.reduce((m, f) => Math.max(m, Number(f.start_hr || 0) + Number(f.repair_hr || 0)), 0);
      return {
        fault_count:     num('resFaultCount', manual_faults.length || 1),
        ac_fault_branch_ids: acIds,
        dc_fault_branch_ids: dcIds,
        manual_faults,
        mobile_storage_speed_kmh: num('resMobileSpeed', 40),
        consider_switches: true,
        horizon_hours: Math.max(48, Math.ceil(latestEnd + 4)),
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
      const scenarioProfiles = getResilienceScenarioProfiles();
      const params = {
        default_fault_count: p.fault_count,
        manual_faults: p.manual_faults,
        ac_fault_branch_ids: p.ac_fault_branch_ids,
        dc_fault_branch_ids: p.dc_fault_branch_ids,
        mess_travel_speed_kmph: p.mobile_storage_speed_kmh,
        horizon_hours: p.horizon_hours,
        time_step_hr: 1.0,
        allow_reconfiguration: true,
        allow_mess_dispatch: true,
        run_power_flow: false,
        consider_switches: true,
        enable_disaster_stages: true,
        use_ra_style_stage_milp: true,
        allow_stage1_open_switches: true,
        allow_stage2_close_ties: true,
        require_switch_for_nonfault_branch_operation: true,
        allow_branch_operation_without_switch: false,
        post_fault_reconfig_window_hr: 2.0,
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

    function showResilienceResults(data) {
      document.getElementById('resultsEmpty').style.display = 'none';
      document.getElementById('resultsContent').style.display = 'block';
      setActiveResultGroup('resilience');
      const nf = (v, d = 2) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(d) : '—';
      const resilienceIndexTip = '弹性指数 = 评估时段内总供电电量 / 总需求电量，取值 0–1；越接近 1 表示灾害期间整体供电保持得越好。它是全时段能量积分指标；最低供电率是单个最差时刻指标。';
      const supplyRatios = Array.isArray(data.restoration_ratio) ? data.restoration_ratio.map(Number).filter(Number.isFinite) : [];
      const minSupplyRatio = supplyRatios.length ? Math.min(...supplyRatios) : (data.avg_restoration_ratio ?? 0);
      const kpis = [
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
        if (e.kind.endsWith('LOAD')) {
          row.load_position = e.position;
          row.load_index = Number(e.item?.index);
          row.bus = Number(e.item?.bus);
        } else {
          row.bus = Number(e.item?.index);
        }
        map.push(row);
      });
      return { profiles, map };
    }

    function buildScenarioTimeSeries(candidate, baseSystem, family) {
      const profiles = candidate?.time_series?.profiles || [];
      if (!Array.isArray(profiles) || profiles.length === 0) return null;
      const load = scenarioCandidateProfileValues(candidate, 'total_load_mw');
      const pv = scenarioCandidateProfileValues(candidate, 'pv_mw');
      const wind = scenarioCandidateProfileValues(candidate, 'wind_mw');
      const renewable = scenarioCandidateProfileValues(candidate, 'total_renewable_mw');
      const totals = scenarioBaseTotals(baseSystem);
      const calcProfiles = [];
      const warnings = [];
      const componentLoadProfiles = (family === 'resilience' && load.length)
        ? buildComponentLoadProfiles(load, baseSystem, 10)
        : { profiles: [], map: [] };
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
      if (renewable.length) {
        calcProfiles.push({ id: 3, name: 'scenario_renewable_scale', values: makeScaleProfile(renewable, totals.pv + totals.wind, 1.0) });
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
          resilience_renewable_profile_id: renewable.length ? 3 : (pv.length && hasPvCapacity ? 2 : (wind.length && hasWindCapacity ? 1 : -1)),
        },
        normalization: {
          base_load_mw: totals.load,
          base_pv_mw: totals.pv,
          base_wind_mw: totals.wind,
          profile_semantics: 'profiles are dimensionless multipliers for /api/session/set_ts_config',
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
      const ts = family === 'reliability' ? null : buildScenarioTimeSeries(representative, caseJson, family);
      if (ts) {
        caseJson._time_series = ts;
        metadata.calculation_defaults.use_time_series = family === 'regular' || family === 'resilience';
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

    function generatedScenarioCaseFaults(caseJson) {
      const event = caseJson?._generated_scenario?.resilience_event || caseJson?.resilience_event || {};
      const candidates = [event.faults, event.generated_faults, event.manual_faults, caseJson?.generated_faults, caseJson?.manual_faults];
      for (const arr of candidates) if (Array.isArray(arr) && arr.length) return arr;
      return [];
    }

    function generatedScenarioCaseLabel(caseJson, index) {
      const meta = caseJson?._generated_scenario || {};
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
      const matched = cases.filter(c => !targetFamily || c?._generated_scenario?.family === targetFamily);
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

    function firstCaseFromGeneratedScenarioJson(obj, targetFamily) {
      const allowedBundleFormats = new Set(['generated_scenario_case_bundle_v1', 'generated_scenario_case_bundle_v2']);
      if (allowedBundleFormats.has(obj?.format) && Array.isArray(obj.cases)) {
        return chooseGeneratedScenarioCase(obj.cases, targetFamily);
      }
      if (obj && (obj.ac || obj.dc)) return obj;
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

    document.getElementById('btnExportRegularScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('regular', '常规'));
    document.getElementById('btnExportReliabilityScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('reliability', '可靠性'));
    document.getElementById('btnExportResilienceScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('resilience', '弹性'));

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
      try { connStyleSel.value = localStorage.getItem('connectionStyle') || 'orthogonal'; } catch (e) {}
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
          panel.style.width = Math.max(240, Math.min(window.innerWidth * 0.6, newW)) + 'px';
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
  };
})();

// ========== Start ==========
document.addEventListener('DOMContentLoaded', App.init);
