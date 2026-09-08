(() => {
  'use strict';
  const marketCanvas = window.HySimMarketCanvas.forOwner("marketAncillary");
  const $ = id => document.getElementById(id);
  const state = { revision: 0, config: null, boundary: null, result: null, busy: false, dirty: false, rawDirty: false };
  const units = () => [...(state.config?.agc_units || []), ...(state.config?.independent_units || [])];
  const node = (tag, text = '') => { const e = document.createElement(tag); e.textContent = text; return e; };
  const fmt = v => typeof v === 'number' && Number.isFinite(v) ? v.toFixed(2) : '未获得';
  const statusName = value => ({ prearranged_schedule_only: '调频预安排完成（仅线性排程）', schedule_only: '交流安全未认证',
    intraday_schedule_only: '日内调频出清完成（仿真排程）',
    realtime_capacity_shortfall: '实时调整后容量不足，禁止费用核算',
    capacity_shortfall: '调频容量不足', ancillary_capacity_shortfall: '未进入耦合SCED', coupled_safety_failed: '耦合校核未通过',
    scuc_failed: '机组组合未获得有效解', sced_failed: '经济调度未获得有效解', lmp_failed: '节点价格未获得有效解',
    ac_security_failed: '交流安全校核未通过', converged: '能量出清及交流校核通过' }[value] || value);
  const status = message => { $('ancillaryStatus').textContent = message; $('ancillaryToolbarStatus').textContent = message; };
  async function api(body, route = '/api/session/yunnan_ancillary') {
    return window.HySimMarketActivity.request(route, body, { owner: 'marketAncillary' });
  }
  function controls() {
    for (const id of ['ancillaryRun', 'ancillarySave', 'ancillaryExport']) $(id).disabled = state.busy || state.rawDirty || !state.config || (id === 'ancillaryExport' && !state.result) || (id === 'ancillaryRun' && state.config?.workflow?.stage === 'intraday');
    for (const id of ['ancillaryReload', 'ancillaryCase', 'ancillaryApplyJson']) $(id).disabled = state.busy;
    $('ancillaryIntraday').disabled = state.busy || state.dirty || !state.day_ahead_id || !state.result?.schedule_feasible;
    $('ancillarySettle').disabled = state.busy || state.dirty || !state.result?.ancillary?.settlement_eligible;
    $('ancillaryPost').disabled = state.busy || state.dirty || !state.result?.statement?.complete;
    $('ancillaryMonth').disabled = state.busy || !state.journal?.length;
    $('ancillaryJournalExport').disabled = state.busy || !state.journal?.length;
    document.querySelectorAll('#marketAncillaryWorkspace input, #marketAncillaryWorkspace textarea, #marketAncillaryWorkspace select, #ancillaryUnitEditor button').forEach(e => { e.disabled = state.busy; });
  }
  function changed() {
    state.dirty = true; state.rawDirty = false; state.result = null; $('ancillaryResults').hidden = true;
    $('ancillaryJson').value = JSON.stringify(state.config, null, 2);
    marketCanvas?.setBoundary(state.boundary, `ancillary-draft-${state.revision}`);
    controls(); status('调频配置已修改，结果失效；尚未保存');
  }
  function field(target, title, value, type, update, options = {}) {
    const label = node('label', title), input = node('input'); input.type = type;
    if (type === 'checkbox') input.checked = value; else input.value = value;
    input.setAttribute('aria-label', title); Object.assign(input, options);
    input.addEventListener('change', () => {
      if (!input.reportValidity() || (type === 'number' && input.value === '')) return;
      update(type === 'checkbox' ? input.checked : type === 'number' ? Number(input.value) : input.value); changed();
    }); label.append(input); target.append(label); return input;
  }
  function table(target, headers, rows) {
    const t = node('table'); t.className = 'topo-table'; const head = node('tr'); headers.forEach(h => head.append(node('th', h))); t.append(head);
    for (const cells of rows) { const row = node('tr'); for (const value of cells) { const cell = node('td'); cell.append(value instanceof Node ? value : node('span', String(value))); row.append(cell); } t.append(row); }
    target.replaceChildren(t);
  }
  function options(select, rows) {
    const value = select.value; select.replaceChildren(...rows.map(([id, title]) => { const o = node('option', title); o.value = id; return o; }));
    if (rows.some(([id]) => String(id) === value)) select.value = value;
  }
  function unitEditor() {
    const a = units().find(x => String(x.id) === $('ancillaryUnit').value);
    const target = $('ancillaryUnitEditor'); target.replaceChildren(); $('ancillaryMembers').replaceChildren(); if (!a) return;
    const h = Number($('ancillaryHour').value);
    field(target, 'AGC资格及控制状态（声明）', a.qualified, 'checkbox', v => { a.qualified = v; });
    field(target, '里程报价 CNY/MW', a.price_per_mw[h], 'number', v => { a.price_per_mw[h] = v; }, { step: '0.1' });
    field(target, '申报容量 MW', a.capacity_mw[h], 'number', v => { a.capacity_mw[h] = v; }, { step: '1', min: '0' });
    const all = node('button', '将本小时报价及容量应用到24小时'); all.type = 'button'; all.className = 'btn btn-sm';
    all.onclick = () => { a.price_per_mw.fill(a.price_per_mw[h]); a.capacity_mw.fill(a.capacity_mw[h]); changed(); }; target.append(all);
    target.append(node('span', `${({ plant: '厂级AGC', single: '火电单机AGC', storage: '独立储能AGC', load: '直控负荷AGC' })[a.mode]} · 最近8个中标时段 k 均值 ${fmt(a.k_history.reduce((s, v) => s + v, 0) / 8)}`));
    if (!a.members) {
      field(target, '持续响应能力 h', a.sustained_hours, 'number', v => { a.sustained_hours = v; }, { min: '0', step: '0.1' });
      field(target, 'AGC速率 MW/min', a.standard_ramp_mw_min, 'number', v => { a.standard_ramp_mw_min = v; }, { min: '0', step: '0.1' });
      field(target, '跨省备用已中标 MW', a.cross_province_reserve_mw[h], 'number', v => { a.cross_province_reserve_mw[h] = v; }, { min: '0' });
      if (a.mode === 'load') field(target, '负荷调频基点削减 MW', a.baseline_reduction_mw[h], 'number', v => { a.baseline_reduction_mw[h] = v; }, { min: '0' });
      field(target, '资格与持续响应证据来源', a.source, 'text', v => { a.source = v; });
      $('ancillaryMembers').append(marketCanvas.link(a.mode === 'storage' ? 'storage' : 'controllable_loads', a.resource_id));
      return;
    }
    table($('ancillaryMembers'), ['机组 / AC节点', 'AGC标准速率 MW/min', '连续允许运行区间 MW'], a.members.map(m => {
      const g = state.boundary.generators.find(g => g.id === m.generator_id);
      const ref = marketCanvas.link('generators', g.id);
      const name = node('div'); name.append(ref, node('span', ` · AC ${g.bus} · ${g.name}`));
      return [name, fmt(m.standard_ramp_mw_min), m.safe_intervals_mw.length ? m.safe_intervals_mw.map(v => `[${fmt(v[0])}, ${fmt(v[1])}]`).join(' / ') : '按机组出力上下限'];
    }));
  }
  function editor() {
    const c = state.config; $('ancillaryParameters').replaceChildren();
    $('ancillaryJson').value = c ? JSON.stringify(c, null, 2) : '';
    $('ancillarySource').textContent = state.boundary ? `${state.boundary.name} · ${state.boundary.execution.solver || 'highs'} · GAP ${state.boundary.execution.mip_gap} · ${c?.source || ''}` : '尚未载入市场边界';
    if (!c) { $('ancillaryUnit').replaceChildren(); unitEditor(); controls(); return; }
    const p = $('ancillaryParameters');
    field(p, '研究参数模式', c.research, 'checkbox', v => { c.research = v; });
    field(p, '二次调频最低需求 Cmin / MW', c.cmin_mw, 'number', v => { c.cmin_mw = v; }, { min: '0', step: '1' });
    field(p, '负荷比例 R1 / p.u.', c.load_ratio, 'number', v => { c.load_ratio = v; }, { min: '0', max: '1', step: '0.001' });
    field(p, '新能源比例 R2 / p.u.', c.renewable_ratio, 'number', v => { c.renewable_ratio = v; }, { min: '0', max: '1', step: '0.001' });
    options($('ancillaryUnit'), units().map(a => [a.id, `${a.id} · ${a.name}`]));
    if (c.workflow) { const workflow = structuredClone(c.workflow); workflow.stage = 'intraday'; $('ancillaryWorkflow').value = JSON.stringify(workflow, null, 2); }
    unitEditor(); controls();
  }
  function plot(id, data, title, ytitle, extra = {}) {
    const css = getComputedStyle(document.documentElement), ink = css.getPropertyValue('--ink').trim(), grid = css.getPropertyValue('--border').trim();
    return Plotly.react($(id), data, { title: { text: title, font: { size: 14 }, y: 0.98 }, height: 320, margin: { l: 60, r: 24, t: 100, b: 50 },
      paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', font: { color: ink },
      xaxis: { title: '运行日小时', gridcolor: grid }, yaxis: { title: ytitle, gridcolor: grid }, legend: { orientation: 'h', x: 0, y: 1.05, yanchor: 'bottom' }, ...extra }, { responsive: true, displaylogo: false });
  }
  function envelope() {
    const r = state.result, id = Number($('ancillaryGenerator').value), g = r?.sced?.generators?.find(g => g.id === id);
    if (!g?.secondary_up_mw) { Plotly.purge($('ancillaryEnvelopeChart')); return; }
    const x = Array.from({ length: 96 }, (_, t) => t / 4), p = g.power_mw.slice(0, 96);
    const bands = state.config.agc_units.flatMap(a => a.members).find(m => m.generator_id === id)?.safe_intervals_mw || [];
    const traces = [
      { x, y: p, name: '功率基点', line: { color: '#247e9e' } },
      { x, y: p.map((v, t) => v + g.primary_reserve_mw[t] + g.secondary_up_mw[t]), name: '基点 + 一次 + 二次上调', line: { color: '#b25c4c' } },
      { x, y: p.map((v, t) => v - g.secondary_down_mw[t]), name: '基点 - 二次下调', line: { color: '#329775' } }
    ];
    plot('ancillaryEnvelopeChart', traces, `${g.name} · 调频运行范围`, 'MW', { shapes: bands.map(v => ({ type: 'rect', xref: 'paper', x0: 0, x1: 1, y0: v[0], y1: v[1], fillcolor: 'rgba(50,151,117,0.09)', line: { width: 0 }, layer: 'below' })) });
    marketCanvas.southern(state.boundary, r, 'sced', Number($('ancillarySlot').value));
    marketCanvas.select(`generators:${id}`);
    $('ancillaryEnvelopeChart').removeAllListeners?.('plotly_click');
    $('ancillaryEnvelopeChart').on('plotly_click', event => { const t = Math.round(event.points[0].x * 4); $('ancillarySlot').value = String(t); marketCanvas.southern(state.boundary, r, 'sced', t); });
  }
  function results() {
    const r = state.result, a = r?.ancillary; $('ancillaryResults').hidden = !a;
    if (!a) return;
    const hours = a.hours, x = hours.map(h => h.hour);
    plot('ancillaryCapacityChart', [['demand_mw', '需求', '#247e9e'], ['awarded_mw', '预安排', '#329775'], ['shortage_mw', '缺口', '#c65d65']].map(([field, name, color]) => ({ x, y: hours.map(h => h[field]), name, type: 'scatter', mode: 'lines+markers', line: { color } })), '小时调频容量', 'MW');
    const intraday = a.workflow?.stage === 'intraday';
    plot('ancillaryPriceChart', [{ x, y: hours.map(h => intraday ? h.clearing_price_per_mw : h.reference_price_per_mw), name: intraday ? '日内出清价' : '预安排参考价', type: 'scatter', line: { color: '#886238' } }], intraday ? '日内里程价格（仿真）' : '里程价格 · 非正式结算价', 'CNY/MW');
    const h = hours[Number($('ancillaryHour').value)];
    table($('ancillaryAwards'), ['AGC', '排序价', '申报 MW', intraday ? '日内中标 MW' : '预安排 MW', '容量调整原因'], h.bids.map(b => [b.name, fmt(b.ranking_price_per_mw), fmt(b.declared_mw), fmt(b.award_mw), b.reasons.join('；') || '无容量降额']));
    table($('ancillaryAdjustments'), ['小时', 'AGC', '调整', '原容量 MW', '新容量 MW'], (a.adjustment_log || []).map(e => [e.hour, e.unit_id, ({ safety_remove_or_reduce: '安全移出/调减', backfill: '补入', safety_capacity_uplift: '安全调增' })[e.action], fmt(e.before_mw), fmt(e.after_mw)]));
    table($('ancillaryIndependent'), ['独立AGC', '小时中标 MW', '响应能力 h', '能量市场状态'], (state.config.independent_units || []).map(u => {
      const bid = h.bids.find(b => b.id === u.id);
      return [marketCanvas.link(u.mode === 'storage' ? 'storage' : 'controllable_loads', u.resource_id), fmt(bid?.award_mw), fmt(u.sustained_hours), bid?.award_mw > 0 ? (u.mode === 'storage' ? '零充放电基点' : '固定负荷基点，不计能量补偿') : '按能量边界运行'];
    }));
    options($('ancillaryGenerator'), (r.sced?.generators || []).map(g => [g.id, `${g.id} · ${g.name}`]));
    const stages = ['scuc', 'sced', 'lmp'].filter(s => r[s]);
    $('ancillaryAudit').replaceChildren(...stages.map(s => node('p', `${s.toUpperCase()} · ${r[s].solver_status} · ${r[s].variables}变量 / ${r[s].binary_variables}整数 · ${r[s].feasible ? '线性残差审计通过' : '未获有效解'}`)));
    for (const e of a.realtime_capacity_audit || []) $('ancillaryAudit').append(node('p', `${e.hour}h + ${e.second}s · 实时调频容量 ${fmt(e.awarded_mw)} MW · 缺口 ${fmt(e.shortage_mw)} MW`));
    $('ancillaryLimitations').replaceChildren(...a.model_limitations.map(v => node('p', v)));
    const statement = r.statement;
    $('ancillarySettlementStatus').textContent = statement ? (statement.complete ? `日累计仿真核算 · 补偿池 ${fmt(statement.allocation.compensation_pool_cny)} 元 · 现金残差 ${fmt(statement.allocation.compensation_balance_residual_cny)} 元` : `计量未齐备：缺少 ${statement.metering.missing_measurements.length} 个主体小时记录`) : '尚无AGC计量核算结果';
    $('ancillaryMileageChart').hidden = !statement;
    if (statement) {
      const rows = statement.metering.rows;
      const mileage = Array(24).fill(0); rows.forEach(v => { mileage[v.hour] += v.mileage_mw; });
      plot('ancillaryMileageChart', [{ x: Array.from({ length: 24 }, (_, h) => h), y: mileage, type: 'bar', name: 'AUTOR里程', marker: { color: '#247e9e' } }], '每小时AGC总里程', 'MW');
    }
    table($('ancillarySettlement'), ['主体', '补偿 元', '发电侧分摊 元', '用户侧分摊 元', '考核返还 元'], (statement?.allocation?.rows || []).map(p => [p.id, fmt(p.compensation_cny), fmt(p.generation_charge_cny), fmt(p.user_charge_cny), fmt(p.assessment_return_cny)]));
    journal();
    envelope(); controls();
  }
  function journal() {
    table($('ancillaryJournal'), ['账本', '运行日', '凭证', '替代凭证', '补偿变更 元'], (state.journal || []).map(e => [e.book, e.delivery_date, e.entry_id, e.supersedes_id ?? '首次入账', fmt(e.raw_compensation_delta_cny)]));
    $('ancillaryMonthStatus').textContent = state.month ? (state.month.complete ? `${state.month.month} · 全月分摊完成（仿真重述）` : `${state.month.month} · 缺少 ${state.month.missing_days.length} 天凭证，未分摊`) : '尚未核算全月';
    table($('ancillaryMonthResults'), ['主体', '月补偿 元', '发电分摊 元', '用户分摊 元', '考核返还 元'], (state.month?.allocation?.rows || []).map(p => [p.id, fmt(p.compensation_cny), fmt(p.generation_charge_cny), fmt(p.user_charge_cny), fmt(p.assessment_return_cny)]));
  }
  function bind(id, work) {
    $(id).addEventListener('click', async () => {
      if (state.busy) return; state.busy = true; controls();
      try { await work(); } catch (e) { status(e.message); } finally { state.busy = false; controls(); }
    });
  }
  async function reload() {
    const response = await api(); Object.assign(state, response); state.dirty = false; state.rawDirty = false;
    marketCanvas.setBoundary(state.boundary, state.revision); editor(); results(); journal();
    if (state.result?.statement?.request) $('ancillaryMeasurements').value = JSON.stringify(state.result.statement.request, null, 2);
    status(state.result ? `已载入 · ${statusName(state.result.ancillary?.status || state.result.status)}` : '已载入；调频资格、报价及振动区为声明数据');
  }
  bind('ancillaryReload', reload);
  bind('ancillaryEnergySettings', async () => { App.setActiveModule('marketBoundary'); });
  bind('ancillaryCase', async () => {
    const s = await api(undefined, '/api/session/southern_market');
    await api({ action: 'ieee118_mixed', revision: s.revision }, '/api/session/southern_market'); await reload();
  });
  bind('ancillarySave', async () => {
    const r = await api({ action: 'save', revision: state.revision, config: state.config }); state.revision = r.revision; state.config = r.config; state.result = null; state.dirty = false;
    editor(); results(); status('调频配置已保存');
  });
  bind('ancillaryRun', async () => {
    const config = structuredClone(state.config);
    if (config.workflow?.stage === 'intraday') throw new Error('当前为已封存的日内配置；请载入并保存下一次日前申报');
    state.result = null; $('ancillaryResults').hidden = true; status('正在计算机组组合、调频预安排及耦合安全约束');
    const r = await api({ action: 'run', revision: state.revision, config });
    if (r.result.stale) throw new Error('计算期间边界已变化，结果已失效');
    Object.assign(state, r); state.dirty = false; results();
    status(`${statusName(r.result.ancillary?.status || r.result.status)} · ${statusName(r.result.status)}${r.result.error ? ` · ${r.result.error}` : ''}`);
  });
  bind('ancillaryIntraday', async () => {
    const config = structuredClone(state.config); config.workflow = JSON.parse($('ancillaryWorkflow').value);
    const r = await api({ action: 'intraday', revision: state.revision, day_ahead_id: state.day_ahead_id, config });
    if (r.result.stale) throw new Error('边界已变化，日内结果已失效');
    Object.assign(state, r); state.dirty = false; editor(); results(); status(statusName(r.result.ancillary?.status || r.result.status));
  });
  bind('ancillarySettle', async () => {
    const r = await api({ action: 'settle', revision: state.revision, clearing_id: state.result.clearing_id, request: JSON.parse($('ancillaryMeasurements').value) });
    state.result = r.result; results(); status(r.result.statement.complete ? '日累计里程与费用核算完成（仿真）' : '计量记录不完整，未生成费用分摊');
  });
  bind('ancillaryPost', async () => {
    const r = await api({ action: 'post', revision: state.revision, clearing_id: state.result.clearing_id, request: JSON.parse($('ancillaryPosting').value) });
    state.journal = r.journal; state.month = null; journal(); status('日凭证已追加，月结需重新核算');
  });
  bind('ancillaryMonth', async () => {
    const r = await api({ action: 'month', revision: state.revision, request: JSON.parse($('ancillaryMonthlyInput').value) });
    state.month = r.month; journal(); status(r.month.complete ? '全月计量汇总及分摊完成（仿真）' : '全月凭证不齐备，未分摊');
  });
  bind('ancillaryJournalExport', async () => {
    const url = URL.createObjectURL(new Blob([JSON.stringify({ journal: state.journal, month: state.month }, null, 2)], { type: 'application/json' }));
    const link = node('a'); link.href = url; link.download = 'yunnan-ancillary-journal.json'; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  $('ancillaryMeasurements').value = JSON.stringify({ measurements: [], allocation: { continuous_spot: true, generation_share: 0.5, assessment_pool_cny: 0, participants: [] } }, null, 2);
  const today = new Date().toLocaleDateString('en-CA');
  $('ancillaryPosting').value = JSON.stringify({ book: 'study-1', delivery_date: today, posting_date: today, discovered_date: null, supersedes_id: null, reason: '', source: '' }, null, 2);
  $('ancillaryMonthlyInput').value = JSON.stringify({ book: 'study-1', month: today.slice(0, 7), allocation: { continuous_spot: true, generation_share: 0.5, assessment_pool_cny: 0, participants: [] } }, null, 2);
  bind('ancillaryApplyJson', async () => { const config = JSON.parse($('ancillaryJson').value); const r = await api({ action: 'save', revision: state.revision, config }); state.config = r.config; state.revision = r.revision; state.result = null; state.dirty = false; state.rawDirty = false; editor(); results(); status('完整调频配置已校验并保存'); });
  bind('ancillaryExport', async () => { const url = URL.createObjectURL(new Blob([JSON.stringify(state.result, null, 2)], { type: 'application/json' })); const a = node('a'); a.href = url; a.download = 'yunnan-ancillary-result.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000); });
  options($('ancillaryHour'), Array.from({ length: 24 }, (_, h) => [h, `${String(h).padStart(2, '0')}:00`]));
  options($('ancillarySlot'), Array.from({ length: 98 }, (_, t) => [t, t < 96 ? `${String(Math.floor(t / 4)).padStart(2, '0')}:${String(t % 4 * 15).padStart(2, '0')}` : `次日代表点 ${t - 95}`]));
  $('ancillaryUnit').onchange = unitEditor;
  $('ancillaryJson').oninput = () => { state.dirty = true; state.rawDirty = true; state.result = null; $('ancillaryResults').hidden = true; marketCanvas.setBoundary(state.boundary, `ancillary-json-draft-${state.revision}`); controls(); status('完整申报数据已修改，等待应用及校验'); };
  $('ancillaryHour').onchange = () => { unitEditor(); results(); };
  $('ancillaryGenerator').onchange = envelope; $('ancillarySlot').onchange = envelope;
  document.addEventListener('market-ancillary-open', async () => { App.showSouthernMarketWorkspace(); if (state.busy || state.dirty) return; try { await reload(); } catch (e) { status(e.message); } });
  const direct = () => { if (location.hash === '#market-ancillary') { App.setActiveModule('marketAncillary'); document.dispatchEvent(new Event('market-ancillary-open')); } };
  window.addEventListener('hashchange', direct);
  if (document.readyState === 'complete') direct(); else window.addEventListener('load', direct, { once: true });
})();
