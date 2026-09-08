(() => {
  'use strict';
  const marketCanvas = window.HySimMarketCanvas.forOwner("marketRealtime");
  const $ = id => document.getElementById(id);
  const state = { revision: 0, config: null, job: null, busy: false, dirty: false, rawDirty: false, stop: false };
  const node = (tag, text = '') => { const e = document.createElement(tag); e.textContent = text; return e; };
  const fmt = v => typeof v === 'number' && Number.isFinite(v) ? v.toFixed(3) : '未获得';
  const time = minute => `${String(Math.floor(minute / 60)).padStart(2, '0')}:${String(minute % 60).padStart(2, '0')}`;
  const status = text => { $('rtStatus').textContent = text; };
  const names = { ready: '边界已校验', running: '可继续滚动', complete: '滚动结束', failed: '出清失败，状态未推进', stale: '边界已变化', schedule_only: '线性排程，交流安全未认证', converged: '排程及交流校核通过', diagnostic_schedule: '诊断排程，存在物理缺额或越限', scuc_failed: 'SCUC未获得有效解', sced_failed: 'SCED未获得有效解', ac_security_failed: '交流安全未通过' };
  async function api(body, route = '/api/session/southern_realtime') {
    return window.HySimMarketActivity.request(route, body, { owner: 'marketRealtime',
      label: body?.action === 'step' ? `第 ${(state.job?.runs?.length || 0) + 1} 轮 · 24 点调度及独立定价` : undefined });
  }
  function options(id, rows) { const e = $(id), old = e.value; e.replaceChildren(...rows.map(([v, label]) => { const o = node('option', label); o.value = v; return o; })); if ([...e.options].some(o => o.value === old)) e.value = old; }
  function table(id, headers, rows) {
    const t = node('table'), h = node('tr'); headers.forEach(v => h.append(node('th', v))); t.append(h);
    rows.forEach(row => { const r = node('tr'); row.forEach(v => { const c = node('td'); c.append(v instanceof Node ? v : node('span', String(v))); r.append(c); }); t.append(r); }); $(id).replaceChildren(t);
  }
  function controls() {
    const ready = state.config && !state.rawDirty;
    for (const id of ['rtReload', 'rtCase', 'rtApply', 'rtStart', 'rtSteps', 'rtSolver', 'rtGap', 'rtMargin', 'rtReserve', 'rtJson']) $(id).disabled = state.busy;
    for (const id of ['rtStart', 'rtSteps', 'rtSolver', 'rtGap', 'rtMargin', 'rtReserve']) $(id).disabled = state.busy || state.rawDirty || !state.config;
    $('rtSave').disabled = state.busy || !ready;
    for (const id of ['rtStep', 'rtRun']) $(id).disabled = state.busy || state.dirty || !['ready', 'running'].includes(state.job?.status);
    $('rtStop').disabled = !state.busy; $('rtExport').disabled = !state.job || state.dirty;
  }
  function invalidate() {
    state.dirty = true; state.job = null; marketCanvas.setBoundary(state.config?.boundary, `rt-draft-${Date.now()}`);
    results(); controls(); status('实时边界已修改，等待校验');
  }
  function editor() {
    const c = state.config; if (!c) return controls();
    $('rtJson').value = JSON.stringify(c, null, 2); $('rtStart').value = time(c.start_minute); $('rtSteps').value = c.steps;
    $('rtSolver').value = c.boundary.execution.solver || 'highs'; $('rtGap').value = c.boundary.execution.mip_gap;
    $('rtMargin').value = c.section_margin_fraction; $('rtReserve').value = c.reserve_policy;
    $('rtSource').textContent = `${state.boundary?.name || c.boundary.name} · ${c.boundary.buses.length}节点 / ${c.boundary.generators.length}机组 · ${c.source}`;
    table('rtCatalog', ['边界类别', '规则条款', '边界字段'], (state.catalog || []).map(r => [r.name, r.clauses, r.fields.join(', ')])); controls();
  }
  function plot(id, traces, title, unit) {
    const css = getComputedStyle(document.documentElement);
    const yaxis = { title: unit, tickformat: '.2f', automargin: true };
    const values = traces.flatMap(t => t.y || []).filter(Number.isFinite);
    if (id === 'rtPriceChart' && values.length) {
      const lo = Math.min(...values), hi = Math.max(...values);
      if (hi - lo < 0.01) { const center = (hi + lo) / 2; yaxis.range = [center - 0.5, center + 0.5]; }
    }
    if (id === 'rtSlackChart' && values.length) yaxis.range = [0, Math.max(1, Math.max(...values) * 1.1)];
    Plotly.react($(id), traces, { title: { text: title, font: { size: 14 } }, height: 310, margin: { l: 64, r: 16, t: 72, b: 52 },
      paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', font: { color: css.getPropertyValue('--ink').trim() },
      xaxis: { title: '运行时刻', type: 'category', nticks: 6 }, yaxis, legend: { orientation: 'h', y: 1.15 }, showlegend: traces.length > 1 }, { responsive: true, displaylogo: false });
  }
  function frame() {
    const round = state.job?.runs?.[Number($('rtRound').value)], result = round?.[$('rtWindow').value];
    const b = result?.boundary_snapshot, s = result?.sced;
    $('rtAudit').replaceChildren(); $('rtLimits').replaceChildren();
    if (!result) { for (const id of ['rtPowerChart', 'rtSlackChart', 'rtPriceChart', 'rtWaterChart']) Plotly.purge($(id)); $('rtEvents').replaceChildren(); marketCanvas.setBoundary(state.config?.boundary, `rt-empty-${state.revision}`); return; }
    const start = b._rt.start_minute, x = Array.from({ length: 24 }, (_, t) => time(start + t * 5));
    options('rtSlot', x.map((v, t) => [t, v])); options('rtGenerator', (s?.generators || []).map(g => [g.id, `${g.id} · ${g.name}`]));
    options('rtBus', (b?.buses || []).map(g => [g.id, `${g.id} · ${g.name}`]));
    options('rtReservoir', (s?.reservoirs || []).map(h => [h.id, `${h.id} · ${h.name}`]));
    $('rtAudit').append(node('p', `${names[result.status] || result.status} · ${fmt(result.runtime_sec)}秒 · ${result.prices_valid ? '15分钟定价有效' : result.reference_only ? '参考窗口不发布价格' : '15分钟定价未获得'}`));
    for (const stage of ['scuc', 'sced', 'lmp']) if (result[stage]) { const r = result[stage]; $('rtAudit').append(node('p', `${stage.toUpperCase()} · ${r.solver} / ${r.solver_status} · ${r.variables}变量 · 残差 ${fmt(r.max_residual)} · GAP ${fmt(r.mip_gap)}`)); }
    for (const error of [result.error, result.pricing_error]) if (error) $('rtAudit').append(node('p', error));
    for (const limit of result.model_scope.limitations) $('rtLimits').append(node('p', limit));
    const g = s?.generators?.find(g => g.id === Number($('rtGenerator').value));
    plot('rtPowerChart', g ? [{ x, y: g.power_mw, name: '功率基点', line: { color: '#247e9e' } }, { x, y: g.primary_reserve_mw, name: '一次调频', line: { color: '#329775' } }, { x, y: g.accident_reserve_mw, name: '事故备用', line: { color: '#b25c4c' } }] : [], g ? g.name : '机组结果', 'MW');
    const sum = (rows, fields) => x.map((_, t) => (rows || []).reduce((v, r) => v + fields.reduce((n, f) => n + (r[f]?.[t] || 0), 0), 0));
    plot('rtSlackChart', s?.feasible ? [{ x, y: sum(s.buses, ['deficit_mw', 'surplus_mw']), name: '节点不平衡量绝对值和', line: { color: '#b25c4c' } }, { x, y: sum(s.branches, ['overload_mw']), name: '线路越限和', line: { color: '#247e9e' } }] : [], '物理缺额与越限', 'MW');
    const bus = result.prices_valid ? result.lmp.buses.find(b => b.id === Number($('rtBus').value)) : null;
    plot('rtPriceChart', bus ? [{ x: Array.from({ length: 8 }, (_, t) => time(start + t * 15)), y: bus.lmp_per_mwh, name: '独立LMP', line: { color: '#329775' } }] : [], '15分钟节点价格', '元/MWh');
    plot('rtWaterChart', (s?.reservoirs || []).filter(h => h.id === Number($('rtReservoir').value)).map(h => ({ x, y: h.level_m, name: h.name })), '水库水位', 'm');
    table('rtEvents', ['设备类型 / ID', '时刻', '指标', 'MW', '约束证据（非独立因果判定）'], (result.events || []).map(e => [['buses', 'branches', 'sections', 'trades'].includes(e.family) ? marketCanvas.link(e.family, e.id) : `${e.family} / ${e.id}`, x[e.point], e.metric, fmt(e.mw), e.evidence]));
    marketCanvas.setBoundary(b, `rt-${state.run_id}-${$('rtRound').value}-${$('rtWindow').value}`);
    marketCanvas.southern(b, result, 'sced', Number($('rtSlot').value));
    for (const id of ['rtPowerChart', 'rtSlackChart', 'rtWaterChart']) { $(id).removeAllListeners?.('plotly_click'); $(id).on?.('plotly_click', event => { const t = event.points?.[0]?.pointIndex; if (Number.isInteger(t) && t < 24) { $('rtSlot').value = t; marketCanvas.southern(b, result, 'sced', t); } }); }
  }
  function results() {
    options('rtRound', (state.job?.runs || []).map((r, i) => [i, `${i + 1} · ${time(r.start_minute)}`]));
    table('rtHourly', ['节点', '小时', '已执行价格点', '小时均价 元/MWh'], (state.job?.hourly_prices || []).map(r => [r.bus_id, time(r.hour * 60), `${r.observed_quarters}/4`, fmt(r.price_per_mwh)]));
    table('rtForecast', ['机组', '5分钟点', '采用来源', '预测 MW'], (state.job?.forecast_resolution || []).filter(r => r.origin !== 'submitted').map(r => [r.id, r.point, r.origin, fmt(r.mw)])); frame(); controls();
  }
  async function reload() {
    const r = await api(); Object.assign(state, r); state.dirty = false; state.rawDirty = false;
    const solvers = r.solver_capabilities || [];
    options('rtSolver', (Array.isArray(solvers) ? solvers : solvers.solvers || []).filter(s => s.available !== false).map(s => [s.id || s.solver || s.name, s.label || s.name || s.id]));
    editor(); results(); status(state.config ? names[state.job?.status] || '实时边界模板已载入，等待校验' : '尚未载入电能量市场案例');
  }
  function bind(id, work) { $(id).onclick = async () => { if (state.busy) return; state.busy = true; controls(); try { await work(); } catch (e) { status(e.message); } finally { state.busy = false; controls(); } }; }
  async function step() { status('正在计算24点SCUC、SCED及独立8点LMP'); const r = await api({ action: 'step', revision: state.revision, run_id: state.run_id }); Object.assign(state, r); results(); $('rtRound').selectedIndex = Math.max(0, $('rtRound').options.length - 1); frame(); status(names[state.job.status] || state.job.status); }
  bind('rtReload', reload); bind('rtCase', async () => { const s = await api(undefined, '/api/session/southern_market'); await api({ action: 'ieee118_mixed', revision: s.revision }, '/api/session/southern_market'); await reload(); });
  bind('rtSettings', async () => App.setActiveModule('marketBoundary'));
  bind('rtApply', async () => { state.config = JSON.parse($('rtJson').value); state.rawDirty = false; invalidate(); editor(); });
  bind('rtSave', async () => { const r = await api({ action: 'save', revision: state.revision, config: state.config }); Object.assign(state, r); state.dirty = false; editor(); results(); status('边界校验通过，可开始逐轮出清'); });
  bind('rtStep', step); bind('rtRun', async () => { state.stop = false; do { await step(); } while (!state.stop && state.job.status === 'running'); });
  $('rtStop').onclick = () => { state.stop = true; status('本轮结束后暂停'); };
  bind('rtExport', async () => { const url = URL.createObjectURL(new Blob([JSON.stringify(state.job, null, 2)], { type: 'application/json' })); const a = node('a'); a.href = url; a.download = 'southern-realtime.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000); });
  for (const id of ['rtStart', 'rtSteps', 'rtSolver', 'rtGap', 'rtMargin', 'rtReserve']) $(id).onchange = () => {
    if (!state.config) return; const c = state.config;
    if (id === 'rtStart') { const [h, m] = $('rtStart').value.split(':').map(Number); c.start_minute = h * 60 + m;
      for (const load of c.boundary.controllable_loads) { const sealed = state.boundary.controllable_loads.find(l => l.id === load.id); load.compensation_per_mwh = Array.from({length:72}, (_,t) => sealed.compensation_per_mwh[Math.floor(c.start_minute/15)+Math.floor(t/3)]); }
    }
    if (id === 'rtSteps') c.steps = Number($('rtSteps').value);
    if (id === 'rtSolver') c.boundary.execution.solver = $('rtSolver').value;
    if (id === 'rtGap') c.boundary.execution.mip_gap = Number($('rtGap').value);
    if (id === 'rtMargin') c.section_margin_fraction = Number($('rtMargin').value);
    if (id === 'rtReserve') c.reserve_policy = $('rtReserve').value;
    invalidate(); editor();
  };
  $('rtJson').oninput = () => { state.rawDirty = true; invalidate(); };
  for (const id of ['rtRound', 'rtWindow', 'rtSlot', 'rtGenerator', 'rtBus', 'rtReservoir']) $(id).onchange = frame;
  document.addEventListener('market-realtime-open', async () => { App.showSouthernMarketWorkspace(); if (!state.busy && !state.dirty) try { await reload(); } catch (e) { status(e.message); } });
  const direct = () => { if (location.hash === '#market-realtime') { App.setActiveModule('marketRealtime'); document.dispatchEvent(new Event('market-realtime-open')); } };
  window.addEventListener('hashchange', direct); if (document.readyState === 'complete') direct(); else window.addEventListener('load', direct, { once: true });
})();
