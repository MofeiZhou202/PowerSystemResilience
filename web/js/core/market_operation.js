/* Contract: docs/modules/market/southern_execution_contract.md, rolling operation. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const fields = { load_scale: '负荷倍数', wind_scale: '风电预测倍数', solar_scale: '光伏预测倍数', inflow_scale: '来水倍数', bid_scale: '全市场报价倍数', generator_bid_scale: '发电 / 储能报价倍数', load_bid_scale: '可控负荷补偿倍数', line_limit_scale: '线路限额倍数' };
  const names = { ...fields, generator_outages: '机组停运', branch_outages: '线路停运', boundary_overrides: '设备级边界联合恢复' };
  const state = { revision: 0, run_id: 0, job: null, days: [], running: false, pause: false, busy: false, serverBusy: false, dirty: false };
  const el = (tag, value) => { const n = document.createElement(tag); if (value !== undefined) n.textContent = String(value); return n; };
  const fmt = value => typeof value === 'number' && Number.isFinite(value) ? value.toLocaleString('zh-CN', { maximumFractionDigits: 4 }) : '不可用';
  const quality = s => ({ optimal_within_tolerance: '达到最优性容差', feasible_limit: '限时/限额可行解', feasible_unproven: '可行，未证最优', proven_infeasible: '已证不可行', unbounded: '无界', infeasible_or_unbounded: '不可行或无界', limit_without_verified_solution: '限额内无已验证解', solver_or_audit_failure: '求解/残差复核失败' }[s?.solution_quality] || '未记录求解质量');
  const time = t => `${String(Math.floor(t / 4)).padStart(2, '0')}:${String(t % 4 * 15).padStart(2, '0')}`;
  const status = value => { $('operationStatus').textContent = value; if ($('marketForecastWorkspace').hidden) window.HySimMarketCanvas.progress(value); };
  const dateAt = (start, d) => { const value = new Date(`${start}T00:00:00Z`); value.setUTCDate(value.getUTCDate() + d); return value.toISOString().slice(0, 10); };
  const defaultDay = () => ({ ...Object.fromEntries(Object.keys(fields).map(k => [k, 1])), first_slot: 0, last_slot: 95, generator_outages: [], branch_outages: [] });
  let solverCapabilities = [];
  const solverOptions = prefix => ({solver: $(`${prefix}Solver`).value, time_limit_sec:Number($(`${prefix}TimeLimit`).value),mip_gap:Number($(`${prefix}MipGap`).value),threads:Number($(`${prefix}Threads`).value)});
  function solverEditor(prefix, value = {}) {
    const target = $(`${prefix}SolverOptions`); target.replaceChildren();
    const select = el('select');select.id=`${prefix}Solver`;
    for(const id of ['highs','gurobi']) { const cap=solverCapabilities.find(c=>c.id===id), o=el('option',`${id==='highs'?'HiGHS':'Gurobi'}${cap?.available===false?'（不可用）':''}`);o.value=id;o.disabled=cap?.available===false;select.append(o); }
    select.value=value.solver||'highs';const label=el('label','求解器');label.append(select);target.append(label);
    for(const [id,title,v,min,max,step] of [['TimeLimit','每次求解时限（秒）',value.time_limit_sec??120,.1,3600,'any'],['MipGap','相对 MIP gap',value.mip_gap??0,0,.1,'any'],['Threads','Gurobi线程数（0自动）',value.threads??0,0,128,'1']]) {
      const label=el('label',title),input=el('input');Object.assign(input,{id:`${prefix}${id}`,type:'number',min:String(min),max:String(max),step,required:true,value:String(v)});label.append(input);target.append(label);
    }
    const threads=()=>{ $(`${prefix}Threads`).disabled=select.value!=='gurobi';if(select.value!=='gurobi')$(`${prefix}Threads`).value=0; };select.onchange=threads;threads();
    const scope=el('span','Gurobi：MILP/LP均限时；HiGHS：仅MILP限时。时限不含建模和恢复实验总耗时。');target.append(scope);
  }
  async function api(body, path = '/api/session/market_operation') {
    const response = await fetch(path, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
    return data;
  }
  function options(select, values) {
    const old = select.value; select.replaceChildren(...values.map(([value, title]) => { const option = el('option', title); option.value = value; return option; }));
    if (values.some(([v]) => String(v) === old)) select.value = old;
  }
  function table(headers, rows) {
    const wrap = el('div'); wrap.className = 'topo-table-wrap'; const t = el('table'); t.className = 'topo-table';
    const head = el('thead'), hr = el('tr'); headers.forEach(h => hr.append(el('th', h))); head.append(hr); t.append(head);
    const body = el('tbody'); rows.forEach(row => { const tr = el('tr'); row.forEach(v => { const td = el('td'); v instanceof Node ? td.append(v) : td.textContent = String(v); tr.append(td); }); body.append(tr); });
    t.append(body); wrap.append(t); return wrap;
  }
  function paged(id, headers, rows) {
    const target = $(id); target.replaceChildren(); let page = 0;
    const content = el('div'), nav = el('div'); nav.className = 'operation-actions';
    const prev = el('button', '上一页'), next = el('button', '下一页'), count = el('span');
    prev.className = next.className = 'btn btn-sm';
    const render = () => { content.replaceChildren(table(headers, rows.slice(page * 100, (page + 1) * 100))); count.textContent = `${rows.length} 条 · ${page + 1}/${Math.max(1, Math.ceil(rows.length / 100))}`; prev.disabled = page === 0; next.disabled = (page + 1) * 100 >= rows.length; };
    prev.onclick = () => { --page; render(); }; next.onclick = () => { ++page; render(); }; nav.append(prev, count, next); target.append(content, nav); render();
  }
  function controls() {
    const locked = state.running || state.busy || state.serverBusy;
    $('operationConfig').disabled = locked;
    for (const id of ['operationRun', 'operationLoadCase', 'operationCase', 'operationReload', 'operationBoundary']) $(id).disabled = locked;
    $('operationReload').disabled = state.running || state.busy;
    $('operationPause').disabled = !state.running || state.pause;
    const resumable = state.job && ['ready', 'running'].includes(state.job.status) && state.job.boundary_revision === state.revision;
    $('operationResume').disabled = locked || !resumable;
    $('operationCancel').disabled = !resumable;
    $('operationExport').disabled = !state.job;
  }
  function changes(day) {
    return Object.entries(names).filter(([key]) => day[key] != null && (Array.isArray(day[key]) ? day[key].length : day[key] !== 1))
      .map(([key, name]) => `${name}: ${key === 'boundary_overrides' ? `${day[key].length}项` : Array.isArray(day[key]) ? day[key].join(', ') : fmt(day[key])}`).join('；') || '基准边界';
  }
  function plan() {
    $('operationPlan').replaceChildren(table(['日期', '用途', '时段', '边界设置'], state.days.map((d, i) => [dateAt($('operationStartDate').value, i), i === state.days.length-1 ? '末日预安排预测' : '运行日及前一日预安排', `${time(d.first_slot)}–${time(d.last_slot)}`, changes(d)])));
  }
  function editor() {
    const d = state.days[Number($('operationEditDay').value)]; if (!d) return;
    for (const key of Object.keys(fields)) $(`operation-${key}`).value = d[key] ?? 1;
    for (const key of ['bid_scale','generator_bid_scale']) {
      const input = $(`operation-${key}`); input.disabled = Number($('operationEditDay').value) === state.days.length-1;
      input.title = input.disabled ? '发电/储能单值报价属于运行日窗口；仅预测日不独立定价' : '';
    }
    $('operationFirst').value = d.first_slot + 1; $('operationLast').value = d.last_slot + 1;
    $('operationGeneratorOutages').value = d.generator_outages.join(', '); $('operationBranchOutages').value = d.branch_outages.join(', ');
    $('operationDraftStatus').textContent = '';
    window.HySimMarketBoundary?.refreshDay();
  }
  function calendar(reset = false) {
    const start = $('operationStartDate').value; if (!start) return;
    const [year, month] = start.split('-').map(Number);
    const count = $('operationHorizon').value === 'week' ? 7 : new Date(Date.UTC(year, month, 0)).getUTCDate();
    state.days = Array.from({ length: count+1 }, (_, i) => reset ? defaultDay() : state.days[i] || defaultDay());
    options($('operationEditDay'), state.days.map((_, i) => [i, `${dateAt(start, i)}${i === count ? ' · 仅预测' : ''}`])); editor(); plan();
  }
  function saveDay() {
    for (const input of $('operationConfig').querySelectorAll('input')) if (!input.reportValidity()) throw new Error('请修正仿真设置');
    const ids = id => {
      const raw = $(id).value.trim(); if (!raw) return [];
      if (!/^\d+(\s*[,，]\s*\d+)*$/.test(raw)) throw new Error('停运设备应填写逗号分隔的非负整数 ID');
      return raw.split(/[,，]/).map(Number);
    };
    const d = Object.fromEntries(Object.keys(fields).map(key => [key, Number($(`operation-${key}`).value)]));
    d.first_slot = Number($('operationFirst').value) - 1; d.last_slot = Number($('operationLast').value) - 1;
    if (d.first_slot > d.last_slot) throw new Error('起始时段不能晚于结束时段');
    d.generator_outages = ids('operationGeneratorOutages'); d.branch_outages = ids('operationBranchOutages');
    const previous = state.days[Number($('operationEditDay').value)];
    if (previous?.boundary_overrides) d.boundary_overrides = previous.boundary_overrides;
    state.days[Number($('operationEditDay').value)] = d; state.dirty = true; plan(); $('operationDraftStatus').textContent = '当日边界已应用';
  }
  function details() {
    const report = state.preview || state.job;
    const d = report?.days[Number($('operationResultDay').value)], slot = Number($('operationSlot').value);
    window.HySimMarketCanvas.operation(report, Number($('operationResultDay').value), slot, state.revision);
    for (const id of ['operationCauses', 'operationNodes', 'operationLines']) $(id).replaceChildren();
    if (!d) return;
    if (!d.valid) { $('operationCauses').textContent = d.error || d.status; return; }
    const p = d.periods[slot]; const root = $('operationCauses');
    root.append(el('p', `${dateAt(report.config.start_date, d.day)} ${time(slot)} · 负荷 ${fmt(p.load_mw)} MW；机组可用容量上界 ${fmt(p.available_generation_mw)} MW；缺额 ${fmt(p.deficit_mw)} MW；富余 ${fmt(p.surplus_mw)} MW；最大单线越限 ${fmt(p.max_line_overload_mw)} MW。`));
    root.append(el('p', `当日边界：${changes(d.boundary)}。区间 ${time(d.boundary.first_slot)}–${time(d.boundary.last_slot)}；报价倍数作用于全天。`));
    if (d.lookahead) root.append(table(['次日预测日期', '代表点', '来源时刻', '统调负荷 MW', '时长 h', '权重 h'], d.lookahead.points.map(p => [dateAt(report.config.start_date,d.lookahead.source_day), p.kind === 'peak' ? '峰' : '谷', time(p.source_slot), fmt(p.load_mw), fmt(p.duration_hr), fmt(p.weight_hr)])));
    root.append(table(['求解阶段 / 后端', '质量', '原始状态', 'MIP gap', '二进制变量数', '耗时 s'], Object.entries(d.stages || {}).map(([name,s])=>[`${name.toUpperCase()} / ${s.solver||'未知'}`, quality(s),s.solver_status,fmt(s.mip_gap),fmt(s.binary_variables),fmt(s.runtime_sec)])));
    const performance = table(['阶段 / 建模形式', '变量数', '非零系数数', '启动分类消元机组数', '建模 s', '复核及结果生成 s', '恢复约束残差'], Object.entries(d.stages || {}).map(([name,s])=>[`${name.toUpperCase()} / ${s.formulation === 'compact' ? '等价紧凑式' : s.formulation === 'reference' ? '原式' : '未知'}`,fmt(s.variables),fmt(s.nonzeros),fmt(s.compact_units),fmt(s.assembly_sec),fmt(s.audit_sec),s.reconstructed_max_residual == null ? '不可用' : Number(s.reconstructed_max_residual).toExponential(2)]));
    performance.dataset.testid = 'market-stage-performance';
    root.append(performance);
    root.append(el('p','ΔP、ΔPᵢⱼ 是当前已求得方案的诊断指标；限时可行解不能证明异常不可避免，配对差值也可能包含优化未收敛的影响。'));
    if (p.deficit_mw > 1e-6) root.append(el('p', p.load_mw > p.available_generation_mw + 1e-6 ? '证据：负荷高于机组可用容量上界。还需结合外送受电、储能、可控负荷及网络约束复核缺额。' : '证据：总容量上界未揭示供电不足；需结合网络输送、水量、爬坡及运行约束复核。'));
    if (p.max_line_overload_mw > 1e-6) root.append(el('p', '证据：下方线路功率超过当前有功限额。研究罚项允许越限，越限与缺额之间存在经济权衡。'));
    const refValue = v => Array.isArray(v) ? (v.some(x => typeof x === 'object') ? `${v.length}项（见导出配置）` : `[${v.join(', ')}]`) : fmt(v);
    if (d.counterfactuals.length) root.append(table(['恢复因素', '实际 → 参照', '重算状态 / SCUC 质量', '本时段缺额减少 MW', '本时段富余减少 MW', '本时段越限和减少 MW', '全天缺额减少 MWh'], d.counterfactuals.map(c => [names[c.factor], `${refValue(c.sampled_value)} → ${refValue(c.reference_value)}`, c.valid ? `${c.status} · ${quality(c.stages?.scuc)}` : c.error || c.status, fmt(c.valid ? p.deficit_mw - c.periods[slot].deficit_mw : null), fmt(c.valid ? p.surplus_mw - c.periods[slot].surplus_mw : null), fmt(c.valid ? p.overload_sum_mw - c.periods[slot].overload_sum_mw : null), fmt(c.reduction_deficit_mwh)])));
    else root.append(el('p', report.config.explain ? '当日未偏离参照边界，无配对恢复实验。' : '本次未启用单因素恢复重算。'));
    root.append(el('p', '恢复差值固定当日日初状态；正值表示恢复后减少，负值表示增加。差值属于条件敏感性，不能相加解释为唯一原因。'));
    if (d.binding_constraints) {
      const audit = el('details'); audit.append(el('summary', 'SCED 生效约束证据（非唯一原因）'));
      const labels = { '2.6.3.3': '机组约束', '2.6.3.9': '机组群功率', '2.6.3.10': '机组群电量', '2.6.3.14': '网络限额', reservoir: '水库', storage: '储能' };
      const rows = d.binding_constraints.filter(r => r.name.endsWith(`/${slot}`) || r.name.endsWith(`/${slot}/upper`) || r.name.endsWith(`/${slot}/lower`));
      audit.append(table(['约束 / 设备 / 时段', '左端量', '右端界', '违反量'], rows.map(r => [Object.entries(labels).reduce((name, [key,label]) => name.replace(key,label),r.name), fmt(r.lhs),fmt(r.rhs),fmt(r.violation)])));
      audit.append(el('p', `约束行保留原模型单位；生效仅表示当前解到达边界。${d.binding_constraints_truncated ? '完整 SCED 证据超过 1000 行，本列表已截断，缺失不代表未生效。' : ''}`)); root.append(audit);
    }
    const affected = $('operationOnlyAffected').checked;
    paged('operationNodes', ['节点 ID', '名称', 'ΔPᵢ MW', '缺额 MW', '富余 MW', '残差 MW', '诊断价 元/MWh'], d.nodes.filter(n => !affected || n.deficit_mw[slot] > 1e-6 || n.surplus_mw[slot] > 1e-6).map(n => [window.HySimMarketCanvas.link('buses', n.id), n.name, fmt(n.deficit_mw[slot]-n.surplus_mw[slot]), fmt(n.deficit_mw[slot]), fmt(n.surplus_mw[slot]), Number(n.node_imbalance_mw[slot]).toExponential(2), fmt(n.lmp_per_mwh?.[slot])]));
    paged('operationLines', ['线路 ID', '名称', '起点→终点', '投运', '功率 MW', '生效下限 MW', '生效上限 MW', 'ΔPᵢⱼ MW'], d.lines.filter(l => !affected || l.overload_mw[slot] > 1e-6).map(l => [window.HySimMarketCanvas.link('branches', l.id), l.name, `${l.from_bus}→${l.to_bus}`, l.available[slot] ? '是' : '否', fmt(l.power_mw[slot]), fmt(l.min_mw[slot]*l.available[slot]), fmt(l.max_mw[slot]*l.available[slot]), fmt(l.overload_mw[slot])]));
  }
  function results() {
    const j = state.preview || state.job; controls();
    $('operationSummary').replaceChildren(); $('operationDays').replaceChildren();
    if (!j) { for (const id of ['operationBalanceChart', 'operationOverloadChart']) if (typeof Plotly !== 'undefined') Plotly.purge($(id)); options($('operationResultDay'), []); $('operationLimitations').replaceChildren(); details(); return; }
    const labels = { ready: '待运行', running: '可继续', completed: '已完成', cancelled: '已终止', failed: '失败', stale: '边界已失效' };
    status(`${labels[j.status] || j.status} · 已完成 ${j.completed_days}/${j.total_days} 天${state.running ? ' · 正在计算下一日' : ''}${j.error ? ` · ${j.error}` : ''}${j.boundary_revision !== state.revision ? ' · 当前边界已变化，需新建仿真' : ''}`);
    const days = j.days.filter(d => d.valid);
    const sum = key => days.reduce((s, d) => s + d[key], 0);
    $('operationSummary').textContent = `${j.boundary_name || '未载入边界'} · ${j.config.start_date || '起始日待设置'} · ${days.length * 96} 个已出清时段 · 缺额 ${fmt(sum('deficit_mwh'))} MWh · 富余 ${fmt(sum('surplus_mwh'))} MWh · 线路越限积分和 ${fmt(sum('overload_mwh'))} MW·h`;
    $('operationDays').replaceChildren(table(['日期', '状态 / SCUC 质量', '缺额 MWh', '富余 MWh', '越限积分和 MW·h', 'SCED 残差', '诊断价'], j.days.map(d => {
      const button = el('button', dateAt(j.config.start_date, d.day)); button.className = 'btn btn-sm'; button.onclick = () => { $('operationResultDay').value = d.day; details(); };
      return [button, `${d.status} · ${quality(d.stages?.scuc)}`, fmt(d.deficit_mwh), fmt(d.surplus_mwh), fmt(d.overload_mwh), d.stages?.sced?.max_residual == null ? '不可用' : Number(d.stages.sced.max_residual).toExponential(2), d.diagnostic_prices_valid ? '有效（条件）' : '不可用'];
    })));
    options($('operationResultDay'), Array.from({ length: j.config.start_date ? j.total_days : 0 }, (_, day) => [day, `${dateAt(j.config.start_date, day)}${j.days[day] ? '' : ' · 待出清'}`]));
    $('operationLimitations').replaceChildren(...j.limitations.map(s => el('p', s)), el('p', '日前 96+2 点，日末第 96 点状态承接；储能每天保留基准申报终值。非空启停功率轨迹不支持此滚动入口。缺额罚价过低可能诱发经济性缺额。线路越限积分为跨线路累加量，不是缺供电量。任务由浏览器逐日推进，关闭页面后需重载并继续。'));
    if (typeof Plotly !== 'undefined') {
      const x = days.flatMap(d => d.periods.map(p => `${dateAt(j.config.start_date, d.day)}T${time(p.slot)}:00`));
      const style = getComputedStyle(document.body);
      const chart = (id, series, title) => Plotly.react($(id), series.map(([key, name, color]) => ({ x, y: days.flatMap(d => d.periods.map(p => p[key])), name, line: { color }, mode: 'lines', type: 'scatter' })), { title: { text: title, font: { size: 14 } }, paper_bgcolor: style.getPropertyValue('--bg2').trim(), plot_bgcolor: style.getPropertyValue('--bg2').trim(), font: { color: style.getPropertyValue('--ink').trim() }, height: 280, margin: { t: 40, l: 58, r: 12, b: 70 }, yaxis: { title: { text: 'MW' } }, xaxis: { type: 'date' }, legend: { orientation: 'h', y: -0.25 }, autosize: true }, { responsive: true, displaylogo: false });
      chart('operationBalanceChart', [['deficit_mw', '节点缺额合计', '#c0392b'], ['surplus_mw', '节点富余合计', '#168578']], '节点不平衡时序');
      chart('operationOverloadChart', [['max_line_overload_mw', '最大单线越限', '#ad4a22'], ['overload_sum_mw', '线路越限合计', '#3768ab']], '输电线路有功越限');
    }
    details();
  }
  async function reload(preserveDraft = false) {
    const data = await api(); Object.assign(state, { revision: data.revision, run_id: data.run_id, job: data.job, serverBusy: data.busy });
    const market = await api(undefined, '/api/session/southern_market');
    solverCapabilities = data.solver_capabilities || [];
    if(!preserveDraft) solverEditor('operation',data.job?.config.solver_options || market.boundary?.execution);
    window.HySimMarketCanvas.setBoundary(market.boundary, market.revision);
    window.HySimMarketBoundary?.load(market, data.boundary_catalog);
    $('operationBoundaryName').textContent = `当前已保存边界：${data.boundary_name || '未载入'} · 修订 ${data.revision}`;
    if (data.job && !preserveDraft) {
      const c = data.job.config; $('operationHorizon').value = c.horizon; $('operationStartDate').value = c.start_date; $('operationPenalty').value = c.penalty_per_mwh; $('operationExplain').checked = c.explain;
      state.days = structuredClone(c.days); calendar();
    }
    if (!preserveDraft) state.dirty = false;
    results(); if (data.busy) status('服务端正在计算，完成后可重载任务');
    else if (!data.job) status(data.boundary_name ? '边界已就绪' : '请加载市场算例或保存南方市场边界');
    document.dispatchEvent(new CustomEvent('market-operation-boundary-loaded'));
  }
  async function loop() {
    state.running = true; state.pause = false; controls();
    try {
      while (!state.pause && ['ready', 'running'].includes(state.job.status)) {
        status(`正在计算第 ${state.job.completed_days + 1}/${state.job.total_days} 天 · 已完成 ${state.job.completed_days} 天`);
        const data = await api({ action: 'step', run_id: state.run_id, day: state.job.completed_days }); Object.assign(state, data); results();
        if (window.HySimMarketCanvas.follow()) { $('operationResultDay').value = Math.max(0, state.job.completed_days - 1); details(); }
      }
    } finally { state.running = false; controls(); results(); }
  }
  const handle = fn => async () => { try { await fn(); } catch (e) { status(e.message); controls(); } };
  const bind = (id, fn) => { $(id).onclick = handle(fn); };
  for (const [key, label] of Object.entries(fields)) {
    const wrap = el('label', label), input = el('input'); input.id = `operation-${key}`; input.type = 'number'; input.min = '0'; input.max = '10'; input.step = 'any'; input.required = true; input.value = '1'; wrap.append(input); $('operationFactors').append(wrap);
  }
  const now = new Date(); $('operationStartDate').value = `${now.getFullYear()}-${String(now.getMonth() + 1).padStart(2, '0')}-01`;
  options($('operationSlot'), Array.from({ length: 96 }, (_, t) => [t, `${t + 1} · ${time(t)}`])); calendar(true);
  $('operationHorizon').onchange = () => { if ($('operationHorizon').value === 'month') $('operationStartDate').value = `${$('operationStartDate').value.slice(0, 7)}-01`; calendar(); };
  $('operationStartDate').onchange = handle(() => calendar());
  $('operationEditDay').onchange = editor;
  for (const id of ['operationResultDay', 'operationSlot', 'operationOnlyAffected']) $(id).onchange = details;
  $('operationConfig').addEventListener('input', () => { state.dirty = true; $('operationDraftStatus').textContent = '编辑中，开始仿真时应用当前日设置'; });
  bind('operationSaveDay', saveDay); bind('operationReload', reload);
  bind('operationBoundary', () => App.setActiveModule('marketBoundary'));
  bind('operationLoadCase', async () => {
    state.busy = true; controls();
    try { const current = await api(undefined, '/api/session/southern_market'); await api({ action: $('operationCase').value, revision: current.revision }, '/api/session/southern_market'); await reload(); }
    finally { state.busy = false; controls(); }
  });
  bind('operationRun', async () => {
    if (state.running || state.busy) return; saveDay();
    status('正在校验每日边界并建立新仿真');
    state.busy = true; controls();
    try {
      const config = { horizon: $('operationHorizon').value, start_date: $('operationStartDate').value, penalty_per_mwh: Number($('operationPenalty').value), explain: $('operationExplain').checked, days: state.days,solver_options:solverOptions('operation') };
      Object.assign(state, await api({ action: 'start', revision: state.revision, config }));
      state.dirty = false;
    } finally { state.busy = false; controls(); }
    await loop();
  });
  bind('operationResume', loop);
  bind('operationPause', () => { state.pause = true; controls(); status('暂停已请求，等待当日计算结束'); });
  bind('operationCancel', async () => { state.pause = true; await api({ action: 'cancel', run_id: state.run_id }); state.job.status = 'cancelled'; controls(); status('终止已请求，当前日求解返回后停止'); });
  bind('operationExport', () => {
    const url = URL.createObjectURL(new Blob([JSON.stringify(state.job, null, 2)], { type: 'application/json' })); const a = el('a'); a.href = url; a.download = 'market-operation.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  document.querySelectorAll('#marketOperationWorkspace button').forEach(b => b.classList.add('btn', 'btn-sm'));
  const open = handle(async () => { App.showSouthernMarketWorkspace(); if (!state.running) await reload(state.dirty); });
  document.addEventListener('market-operation-open', open);
  document.addEventListener('market-operation-unlock', controls);
  const direct = () => { if (location.hash === '#market-operation') { App.setActiveModule('marketOperation'); open(); } };
  window.addEventListener('hashchange', direct); window.addEventListener('load', direct, { once: true });
  window.HySimMarketOperation = {
    solverEditor, solverOptions,
    day() { return state.days[Number($('operationEditDay').value)]; },
    setEdits(edits) { state.days[Number($('operationEditDay').value)].boundary_overrides = edits; state.dirty = true; plan(); },
    config() { saveDay(); return { horizon: $('operationHorizon').value, start_date: $('operationStartDate').value, penalty_per_mwh: Number($('operationPenalty').value), explain: $('operationExplain').checked, days: structuredClone(state.days),solver_options:solverOptions('operation') }; },
    preview(report) { state.preview = report; results(); },
    table, paged, dateAt, fmt,
  };
  solverEditor('operation');
})();
