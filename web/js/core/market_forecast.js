/* Forecast API owns generation, daily execution and probability semantics. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const shared = window.HySimMarketOperation;
  const names = { load_scale: '负荷', wind_scale: '风电', solar_scale: '光伏', inflow_scale: '来水', generator_bid_scale: '发电/储能报价', load_bid_scale: '可控负荷补偿', line_limit_scale: '线路限额' };
  const state = { config: null, job: null, revision: 0, run_id: 0, running: false, busy: false, pause: false, serverBusy: false, mode: localStorage.getItem('hysim.marketOperationMode') || 'forecast' };
  const el = (tag, text) => { const n = document.createElement(tag); if (text !== undefined) n.textContent = String(text); return n; };
  const fmt = shared.fmt;
  const pct = x => typeof x === 'number' ? `${fmt(x*100)}%` : '不可用';
  const status = text => { $('forecastStatus').textContent = text; if (state.mode === 'forecast') window.HySimMarketCanvas.progress(text); };
  async function api(body, exporting = false) {
    const r = await fetch(`/api/session/market_forecast${exporting ? '?export=1' : ''}`, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
    const data = await r.json(); if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`); return data;
  }
  function controls() {
    const locked = state.running || state.busy || state.serverBusy;
    $('forecastConfig').disabled = locked; $('forecastGenerate').disabled = locked;
    const ready = state.job && ['ready', 'running'].includes(state.job.status) && state.job.boundary_revision === state.revision;
    $('forecastRun').disabled = locked || !ready; $('forecastCancel').disabled = !ready;
    $('forecastPause').disabled = !state.running || state.pause;
    $('forecastReload').disabled = state.running || state.busy; $('forecastExport').disabled = !state.job;
    $('operationManualMode').disabled = state.running;
    for (const id of ['operationLoadCase', 'operationCase', 'operationBoundary']) if (state.running || state.busy) $(id).disabled = true;
  }
  function input(value, min, max, set, label) {
    const n = el('input'); n.type = 'number'; n.value = value; n.min = min; n.max = max; n.step = 'any'; n.required = true; n.className = 'southern-input'; n.setAttribute('aria-label', label);
    n.onchange = () => { if (n.reportValidity()) { set(Number(n.value)); status('预测参数已修改，需重新生成边界后生效'); } }; return n;
  }
  function editor() {
    const c = state.config; if (!c) return;
    shared.solverEditor('forecast',c.operation.solver_options || shared.solverOptions('operation'));
    $('forecastSolverOptions').onchange=()=>{c.operation.solver_options=shared.solverOptions('forecast');};
    $('forecastMode').value = c.mode; $('forecastStartDate').value = c.operation.start_date;
    $('forecastCount').value = c.sample_count; $('forecastSeed').value = c.seed; $('forecastRho').value = c.temporal_rho;
    $('forecastPenalty').value = c.operation.penalty_per_mwh; $('forecastExplain').checked = c.operation.explain;
    $('forecastTemplateStatus').textContent = c.operation.days.length ? `底稿：${c.operation.days.length}日边界（含末日预测）` : '底稿：每日基准模板，含第八日预测';
    $('forecastRho').disabled = c.mode === 'interval';
    $('forecastMarginals').replaceChildren(shared.table(['预测因素', '分布/区间', '下界倍数', '上界倍数', '正态 σ', '统一预测中心'], c.marginals.map(m => {
      const select = el('select'); select.setAttribute('aria-label', `${names[m.factor]}分布`);
      const kinds = c.mode === 'interval' ? [['fixed', '固定'], ['interval', '区间分层']] : [['fixed', '固定'], ['uniform', '均匀分布'], ['triangular', '三角分布'], ['clipped_normal', '限幅正态']];
      for (const [v, label] of kinds) { const o = el('option', label); o.value = v; select.append(o); } select.value = m.distribution;
      select.onchange = () => { m.distribution = select.value; editor(); status('分布已修改，需重新生成'); };
      const sigma = input(m.sigma, 0, 10, v => { m.sigma = v; }, `${names[m.factor]}标准差`); sigma.disabled = m.distribution !== 'clipped_normal';
      return [names[m.factor], select, input(m.lower, 0, 10, v => { m.lower = v; }, `${names[m.factor]}下界`), input(m.upper, 0, 10, v => { m.upper = v; }, `${names[m.factor]}上界`), sigma, input(m.center[0], 0, 10, v => { m.center.fill(v); editor(); }, `${names[m.factor]}统一中心`)];
    })));
    $('forecastCenters').replaceChildren(shared.table(['因素', ...Array.from({ length: 8 }, (_, d) => `${shared.dateAt(c.operation.start_date, d)}${d === 7 ? '（仅预测）' : ''}`)], c.marginals.map(m => [names[m.factor], ...m.center.map((v, d) => input(v, 0, 10, x => { m.center[d] = x; }, `${names[m.factor]}第${d + 1}日中心`))])));
    $('forecastCorrelation').replaceChildren(shared.table(['因素', ...Object.values(names)], c.correlation.map((row, i) => [Object.values(names)[i], ...row.map((v, k) => {
      const n = input(v, -1, 1, x => { c.correlation[i][k] = x; c.correlation[k][i] = x; editor(); }, `相关 ${i + 1} ${k + 1}`); n.disabled = i === k || c.mode === 'interval'; return n;
    })])));
  }
  function chart(id, traces, title, ytitle) {
    if (typeof Plotly === 'undefined') return;
    const css = getComputedStyle(document.body);
    Plotly.react($(id), traces, { title: { text: title, font: { size: 14 } }, height: 280, margin: { t: 40, l: 65, r: 10, b: 65 }, paper_bgcolor: css.getPropertyValue('--bg2').trim(), plot_bgcolor: css.getPropertyValue('--bg2').trim(), font: { color: css.getPropertyValue('--ink').trim() }, yaxis: { title: { text: ytitle } }, legend: { orientation: 'h', y: -0.25 }, autosize: true }, { responsive: true, displaylogo: false });
  }
  function scatter() {
    if (!state.job) return;
    const f = $('forecastScatterFactor').value, metric = $('forecastScatterMetric').value;
    const scenarios = state.job.scenarios.filter(s => s.status === 'completed');
    chart('forecastScatterChart', [{ x: scenarios.map(s => s.config.days.slice(0,7).reduce((v, d) => v + d[f]/7, 0)), y: scenarios.map(s => s.days.reduce((v, d) => v + d[metric], 0)), text: scenarios.map(s => `场景 ${s.id + 1}`), mode: 'markers', type: 'scatter', marker: { color: '#56b6c2', size: 8 } }], `${names[f]}周均倍数与${metric === 'deficit_mwh' ? '周缺额' : '周线路越限积分'}`, metric === 'deficit_mwh' ? 'MWh' : 'MW·h');
  }
  function selected() {
    if (state.mode !== 'forecast') return;
    const scenario = state.job?.scenarios[Number($('forecastScenario').value)];
    shared.preview(scenario || { boundary_name: '尚无预测场景', config: state.config?.operation || {}, days: [], total_days: 7, completed_days: 0, status: 'ready', limitations: [], boundary_revision: state.revision });
    $('forecastBoundaryPreview').replaceChildren(); if (!scenario) return;
    const resolved = el('div'); resolved.id='forecastRuleBoundaryResolved';
    $('forecastBoundaryPreview').append(shared.table(['日期', ...Object.values(names), '机组停运', '线路停运', '设备级覆盖', '条款边界'], scenario.config.days.map((d, i) => {
      const button=el('button','预览');button.type='button';button.className='btn btn-sm';button.setAttribute('aria-label',`预览场景当日边界 ${i+1}`);
      button.onclick=async()=>{button.disabled=true;try{await window.HySimMarketBoundary.preview(scenario.config,i,resolved,scenario.boundary_revision);}catch(e){resolved.textContent=e.message;}finally{button.disabled=false;}};
      return [`${shared.dateAt(scenario.config.start_date, i)}${i === 7 ? '（仅预测）' : ''}`, ...Object.keys(names).map(k => fmt(d[k])), d.generator_outages.join(', '), d.branch_outages.join(', '),d.boundary_overrides?.length||0,button];
    })),resolved);
  }
  function results() {
    controls(); const j = state.job;
    for (const id of ['forecastStatistics', 'forecastNodeStatistics', 'forecastLineStatistics', 'forecastPeriodStatistics', 'forecastCorrelations', 'forecastLimitations']) $(id).replaceChildren();
    if (!j) { $('forecastScenario').replaceChildren(); selected(); status('预测参数待生成；先加载并保存市场算例'); return; }
    const labels = { ready: '边界已生成', running: '任务可继续', completed: '场景任务结束', cancelled: '已终止', stale: '已失效' };
    status(`${labels[j.status] || j.status} · ${j.boundary_name} · 完成 ${j.finished_scenarios}/${j.scenarios.length} 场景 · 已出清 ${j.completed_days}/${j.total_days} 个日窗${j.boundary_revision !== state.revision ? ' · 边界已变化，需重新生成' : ''}`);
    const old = $('forecastScenario').value;
    $('forecastScenario').replaceChildren(...j.scenarios.map((s, i) => { const o = el('option', `场景 ${i + 1} · ${s.status} · ${s.completed_days}/7 天${s.error ? ` · ${s.error}` : ''}`); o.value = i; return o; }));
    if (old && Number(old) < j.scenarios.length) $('forecastScenario').value = old;
    $('forecastLimitations').replaceChildren(...j.limitations.map(v => el('p', v)), el('p', '预测倍数作用于基准曲线及已设置的每日覆盖区间。七日预测中心是恢复重算参照；均匀分布的数学均值是上下界中点。失败场景和未运行场景不能按零异常处理。'));
    const s = j.statistics;
    if (s) {
      const probability = s.mode === 'probabilistic';
      $('forecastStatistics').append(el('p', `完整有效 ${s.complete_scenarios}/${s.total_scenarios} 场景；失败 ${s.failed_scenarios}。${probability ? '概率仅针对所设分布，Wilson 区间以独立周场景为样本。' : '区间覆盖分析：以下比例为样本比例，不是发生概率或严格最坏界。'}`));
      $('forecastStatistics').append(el('p',`完整场景中含限额可行解 ${fmt(s.complete_scenarios_with_limit)}；未全部证明最优 ${fmt(s.complete_scenarios_unproven)}。下列概率描述所求方案的异常，不是不可避免异常的概率。`));
      const rows = [['ΔP 周内异常', s.week_delta_p_peak_mw], ['ΔPᵢⱼ 周内越限', s.week_delta_pij_peak_mw]].map(([name, v]) => [name, `${v.valid_count}/${v.total_count}`, pct(v.sample_event_fraction), probability ? `${pct(v.probability_bounds[0])}–${pct(v.probability_bounds[1])}` : '不赋概率', probability && v.wilson95_valid_only ? `${pct(v.wilson95_valid_only[0])}–${pct(v.wilson95_valid_only[1])}` : '不可用', fmt(v.mean), fmt(v.p95)]);
      $('forecastStatistics').append(shared.table(['指标', '有效/计划', probability ? '有效样本概率' : '异常样本比例', '未知场景概率界', '有效样本 Wilson 95%', '峰值均值 MW', '峰值 P95 MW'], rows));
      $('forecastStatistics').append(shared.table(['周积分', '有效样本均值', 'P05', 'P50', 'P95', '单位'], [['缺额', s.week_deficit_mwh, 'MWh'], ['富余', s.week_surplus_mwh, 'MWh'], ['线路越限和', s.week_overload_mwh, 'MW·h']].map(([name, v, unit]) => [name, fmt(v.mean), fmt(v.p05), fmt(v.p50), fmt(v.p95), unit])));
      const x = s.periods.map(p => `${shared.dateAt(j.config.operation.start_date, p.day)}T${String(Math.floor(p.slot/4)).padStart(2,'0')}:${String(p.slot%4*15).padStart(2,'0')}:00`);
      shared.paged('forecastPeriodStatistics', ['日期', '时段', '有效/计划', 'ΔP 异常比例', 'ΔPᵢⱼ 越限比例', 'Σ|ΔP| 均值 MW', 'ΣΔPᵢⱼ 均值 MW'], s.periods.map(p => [shared.dateAt(j.config.operation.start_date,p.day), p.slot+1, `${p.delta_p_abs_sum_mw.valid_count}/${p.delta_p_abs_sum_mw.total_count}`, pct(p.delta_p_abs_sum_mw.sample_event_fraction), pct(p.delta_pij_sum_mw.sample_event_fraction), fmt(p.delta_p_abs_sum_mw.mean),fmt(p.delta_pij_sum_mw.mean)]));
      for (const [id, key, title] of [['forecastDeltaPChart', 'delta_p_abs_sum_mw', 'Σ|ΔPᵢ| 场景统计'], ['forecastDeltaPijChart', 'delta_pij_sum_mw', 'ΣΔPᵢⱼ 场景统计']]) chart(id, [['mean','均值','#56b6c2'], ['p95','P95','#e06c75']].map(([field,name,color]) => ({ x, y: s.periods.map(p => p[key][field]), name, mode: 'lines', type: 'scatter', line: { color } })), title, 'MW');
      $('forecastCorrelations').append(shared.table(['周平均输入', '完整场景数', '与周缺额 Pearson r', '与线路越限积分 Pearson r'], s.correlations.map(c => [names[c.factor], c.valid_count, fmt(c.deficit_energy_pearson), fmt(c.overload_integral_pearson)])), el('p', '相关性使用完整周场景；不足 3 个样本或零方差返回不可用。相关性不证明原因，具体时段使用下方恢复重算复核。'));
      for (const [table, id] of [['nodes', 'forecastNodeStatistics'], ['lines', 'forecastLineStatistics']]) shared.paged(id, ['ID', '名称', '有效/计划', probability ? '周异常概率（有效）' : '周异常样本比例', '峰值 P95 MW', '积分均值 MW·h'], s[table].map(r => [r.id, r.name, `${r.peak_mw.valid_count}/${r.peak_mw.total_count}`, pct(r.peak_mw.sample_event_fraction), fmt(r.peak_mw.p95), fmt(r.integral_mwh.mean)]));
    } else {
      $('forecastStatistics').textContent = '尚无已结束场景，统计不可用。';
      for (const id of ['forecastDeltaPChart', 'forecastDeltaPijChart']) if (typeof Plotly !== 'undefined') Plotly.purge($(id));
    }
    scatter(); selected(); controls();
  }
  async function load(preserve = false) {
    const data = await api(); state.revision = data.revision; state.run_id = data.run_id; state.job = data.job; state.serverBusy = data.busy;
    if (!state.config || !preserve) { state.config = structuredClone(data.job?.config || data.defaults); for (const m of state.config.marginals) if (m.center.length === 7) m.center.push(m.center[6]); editor(); }
    results(); if (data.busy) status('服务端正在计算，完成后重载可继续');
  }
  function mode(value) {
    state.mode = value; localStorage.setItem('hysim.marketOperationMode', value);
    $('marketForecastWorkspace').hidden = value !== 'forecast';
    document.querySelectorAll('[data-operation-mode="manual"]').forEach(n => { n.hidden = value !== 'manual'; });
    $('operationForecastMode').setAttribute('aria-selected', String(value === 'forecast')); $('operationManualMode').setAttribute('aria-selected', String(value === 'manual'));
    if (value === 'manual') shared.preview(null); else selected();
  }
  async function loop() {
    state.running = true; state.pause = false; controls();
    try {
      while (!state.pause && ['ready', 'running'].includes(state.job.status)) {
        const s = state.job.next_scenario, d = state.job.scenarios[s].completed_days;
        status(`正在出清场景 ${s + 1}/${state.job.scenarios.length} · 第 ${d + 1}/7 日（含恢复重算）`);
        Object.assign(state, await api({ action: 'step', run_id: state.run_id, scenario: s, day: d })); results();
        if (window.HySimMarketCanvas.follow()) { $('forecastScenario').value = s; selected(); $('operationResultDay').value = d; $('operationResultDay').dispatchEvent(new Event('change')); }
      }
    } finally { state.running = false; controls(); results(); document.dispatchEvent(new CustomEvent('market-operation-unlock')); }
  }
  const handle = fn => async () => { try { await fn(); } catch (e) { status(e.message); controls(); } };
  const bind = (id, fn) => { $(id).onclick = handle(fn); };
  bind('operationForecastMode', () => { mode('forecast'); }); bind('operationManualMode', () => mode('manual'));
  bind('forecastGenerate', async () => {
    if (!state.config) await load();
    for (const input of $('forecastConfig').querySelectorAll('input')) if (!input.reportValidity()) return;
    const c = structuredClone(state.config); c.sample_count = Number($('forecastCount').value); c.seed = Number($('forecastSeed').value); c.temporal_rho = Number($('forecastRho').value);
    c.operation.start_date = $('forecastStartDate').value; c.operation.penalty_per_mwh = Number($('forecastPenalty').value); c.operation.explain = $('forecastExplain').checked;
    c.operation.solver_options=shared.solverOptions('forecast');
    state.busy = true; controls();
    try { Object.assign(state, await api({ action: 'generate', revision: state.revision, config: c })); state.config = structuredClone(state.job.config); results(); }
    finally { state.busy = false; document.dispatchEvent(new CustomEvent('market-operation-unlock')); controls(); }
  });
  bind('forecastRun', loop); bind('forecastReload', load);
  bind('forecastUseManual', () => { const config = shared.config(); if (config.horizon !== 'week') throw new Error('预测场景仅接受 7 日手工底稿'); state.config.operation = config; editor(); status('已采用手工七日覆盖，重新生成后生效'); });
  bind('forecastResetTemplate', () => { state.config.operation.days = []; editor(); status('已恢复每日基准模板，重新生成后生效'); });
  bind('forecastPause', () => { state.pause = true; controls(); status('暂停已请求，等待当日及恢复计算结束'); });
  bind('forecastCancel', async () => { state.pause = true; await api({ action: 'cancel', run_id: state.run_id }); state.job.status = 'cancelled'; controls(); status('终止已请求，当前日返回后停止'); });
  bind('forecastExport', async () => { const data = await api(undefined, true); const url = URL.createObjectURL(new Blob([JSON.stringify(data.job,null,2)], { type: 'application/json' })); const a = el('a'); a.href = url; a.download = 'market-forecast.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url),1000); });
  $('forecastMode').onchange = () => {
    state.config.mode = $('forecastMode').value;
    state.config.marginals.forEach(m => { if (m.distribution !== 'fixed') m.distribution = state.config.mode === 'interval' ? 'interval' : 'uniform'; });
    if (state.config.mode === 'interval') { state.config.temporal_rho = 0; state.config.correlation = Object.keys(names).map((_,i) => Object.keys(names).map((_,k) => Number(i === k))); }
    editor();
  };
  $('forecastStartDate').onchange = () => { if ($('forecastStartDate').value) { state.config.operation.start_date = $('forecastStartDate').value; editor(); } };
  for (const [id, key] of [['forecastCount','sample_count'], ['forecastSeed','seed'], ['forecastRho','temporal_rho']]) $(id).onchange = () => { state.config[key] = Number($(id).value); };
  $('forecastPenalty').onchange = () => { state.config.operation.penalty_per_mwh = Number($('forecastPenalty').value); };
  $('forecastExplain').onchange = () => { state.config.operation.explain = $('forecastExplain').checked; };
  $('forecastScenario').onchange = selected;
  for (const [factor, name] of Object.entries(names)) { const o = el('option', name); o.value = factor; $('forecastScatterFactor').append(o); }
  $('forecastScatterFactor').onchange = scatter; $('forecastScatterMetric').onchange = scatter;
  document.querySelectorAll('#marketForecastWorkspace button').forEach(b => b.classList.add('btn','btn-sm'));
  document.addEventListener('market-operation-open', handle(() => load(true)));
  document.addEventListener('market-operation-boundary-loaded', handle(() => load(true)));
  const resize = new ResizeObserver(entries => { for (const { target } of entries) if (target.data && target.clientWidth && typeof Plotly !== 'undefined') Plotly.Plots.resize(target); });
  for (const id of ['forecastDeltaPChart','forecastDeltaPijChart','forecastScatterChart']) resize.observe($(id));
  mode(state.mode); load().catch(e => status(e.message));
})();
