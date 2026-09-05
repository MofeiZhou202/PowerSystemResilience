/* Backend schema is authoritative for every Southern boundary control. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const state = { schema: null, boundary: null, revision: 0, latest: null, baseline: null, dirty: false, busy: false };
  function view(results) {
    $('southernEditor').hidden = results;
    document.querySelector('#southernMarketWorkspace .southern-editor-layout').hidden = results;
    $('southernResults').hidden = !results; $('southernChart').hidden = !results;
    $('southernBoundaryView').setAttribute('aria-selected', String(!results));
    $('southernResultView').setAttribute('aria-selected', String(results));
    if (!results && typeof Plotly !== 'undefined') Plotly.purge($('southernChart'));
  }
  const text = (tag, content) => { const el = document.createElement(tag); el.textContent = String(content); return el; };
  const status = message => { $('southernStatus').textContent = message; $('southernToolbarStatus').textContent = message; window.HySimMarketCanvas.progress(message, false); };
  async function api(path, body) {
    const response = await fetch(path, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    return result;
  }
  function pathGet(path) { return path.reduce((v, k) => v[k], state.boundary); }
  function pathSet(path, value) {
    const parent = path.slice(0, -1).reduce((v, k) => v[k], state.boundary);
    parent[path.at(-1)] = value; state.dirty = true; status('边界已修改，尚未保存');
  }
  function initial(schema) {
    if (schema.enum) return schema.enum[0];
    if (schema.type === 'object') return Object.fromEntries(Object.entries(schema.properties).map(([k, s]) => [k, initial(s)]));
    if (schema.type === 'array') return Array.from({ length: schema.minItems || 0 }, () => initial(schema.items));
    if (schema.type === 'string') return '';
    return Math.max(0, schema.minimum);
  }
  function input(schema, path, value) {
    let field;
    if (schema.enum) {
      field = document.createElement('select');
      schema.enum.forEach(v => { const option = text('option', v); option.value = v; field.append(option); });
    } else {
      field = document.createElement('input');
      field.type = schema.type === 'integer' && schema.minimum === 0 && schema.maximum === 1
        ? 'checkbox' : schema.type === 'string' ? 'text' : 'number';
      if (field.type === 'number') { field.min = schema.minimum; field.max = schema.maximum; field.step = schema.type === 'integer' ? '1' : 'any'; }
    }
    field.value = value;
    if (field.type === 'checkbox') field.checked = value === 1;
    field.classList.add('southern-input');
    field.setAttribute('aria-label', `${schema.title} ${path.join('/')}`);
    field.dataset.southernPath = path.join('/');
    field.addEventListener('change', () => {
      if (!field.checkValidity() || (field.type === 'number' && field.value === '')) { field.reportValidity(); return; }
      const value = field.type === 'checkbox' ? Number(field.checked)
        : schema.type === 'number' || schema.type === 'integer' ? Number(field.value) : field.value;
      pathSet(path, value);
    });
    return field;
  }
  function renderValue(schema, path, value) {
    if (!['object', 'array'].includes(schema.type)) return input(schema, path, value);
    if (schema.type === 'object') {
      const grid = document.createElement('div'); grid.className = 'southern-fields';
      Object.entries(schema.properties).forEach(([key, fieldSchema]) => {
        if (!(key in value)) return;
        const wrap = document.createElement('div'); wrap.className = 'southern-field';
        const label = text('label', `${fieldSchema.title}${fieldSchema.unit ? ` (${fieldSchema.unit})` : ''}`);
        const editor = renderValue(fieldSchema, [...path, key], value[key]);
        if (!['object', 'array'].includes(fieldSchema.type)) { editor.id = `southern-${[...path, key].join('-')}`; label.htmlFor = editor.id; }
        wrap.append(label, editor); grid.append(wrap);
      });
      return grid;
    }
    const details = document.createElement('details'); details.className = 'southern-series';
    const summary = text('summary', `${schema.title} · ${value.length}`); details.append(summary);
    if (value.length === 98 && ['integer', 'number'].includes(schema.items.type)) {
      const bulk = document.createElement('div'); bulk.className = 'southern-bulk';
      const first = document.createElement('input'); first.type = 'number'; first.min = 1; first.max = 98; first.value = 1;
      const last = first.cloneNode(); last.value = 96;
      const amount = document.createElement('input'); amount.type = 'number'; amount.min = schema.items.minimum; amount.max = schema.items.maximum;
      amount.step = schema.items.type === 'integer' ? '1' : 'any'; amount.value = value[0];
      first.setAttribute('aria-label', `${schema.title} 起点`); last.setAttribute('aria-label', `${schema.title} 终点`); amount.setAttribute('aria-label', `${schema.title} 批量值`);
      [first, last, amount].forEach(el => el.classList.add('southern-input'));
      const apply = text('button', '批量设置'); apply.type = 'button'; apply.className = 'btn btn-sm';
      apply.addEventListener('click', () => {
        if (![first, last, amount].every(el => el.value !== '' && el.reportValidity()) || Number(first.value) > Number(last.value)) return;
        const next = [...value]; for (let t = Number(first.value) - 1; t < Number(last.value); ++t) next[t] = Number(amount.value);
        pathSet(path, next); renderEditor();
      });
      bulk.append(text('span', '时点'), first, text('span', '至'), last, amount, apply); details.append(bulk);
    }
    const list = document.createElement('div'); list.className = 'southern-series-items';
    value.forEach((v, i) => {
      const row = document.createElement('div'); row.className = 'southern-series-row';
      const period = value.length === 98 ? state.boundary.periods[i] : null;
      row.append(text('span', period ? `${i + 1} · ${period.kind === 'day' ? `${String(Math.floor(i / 4)).padStart(2, '0')}:${String(i % 4 * 15).padStart(2, '0')}` : period.kind}` : i + 1));
      row.append(renderValue(schema.items, [...path, i], v)); list.append(row);
    });
    details.append(list);
    if (schema.maxItems !== schema.minItems) {
      const add = text('button', '增加一项'); add.type = 'button'; add.className = 'btn btn-sm';
      add.addEventListener('click', () => { if (value.length < schema.maxItems) { pathSet(path, [...value, initial(schema.items)]); renderEditor(); } });
      const remove = text('button', '删除末项'); remove.type = 'button'; remove.className = 'btn btn-sm';
      remove.addEventListener('click', () => { if (value.length > schema.minItems) { pathSet(path, value.slice(0, -1)); renderEditor(); } });
      details.append(add, remove);
    }
    return details;
  }
  function renderEntities() {
    const key = $('southernCategory').value;
    const schema = state.schema.properties[key]; const value = state.boundary[key];
    const select = $('southernEntity'); const current = select.value; select.replaceChildren();
    if (schema.type === 'array') value.forEach((row, i) => {
      const option = text('option', `${row.id ?? i + 1} · ${row.name || row.kind || schema.items.title}`); option.value = String(i); select.append(option);
    });
    else { const option = text('option', schema.title); option.value = ''; select.append(option); }
    if ([...select.options].some(o => o.value === current)) select.value = current;
    $('southernAdd').disabled = schema.type !== 'array' || schema.maxItems === schema.minItems;
    $('southernDelete').disabled = schema.type !== 'array' || value.length <= schema.minItems;
    renderEditor();
  }
  function renderEditor() {
    const target = $('southernEditor'); target.replaceChildren();
    if (!state.boundary) return;
    const key = $('southernCategory').value; const schema = state.schema.properties[key];
    if (schema.type === 'array') {
      const i = Number($('southernEntity').value);
      if (state.boundary[key][i] !== undefined) target.append(renderValue(schema.items, [key, i], state.boundary[key][i]));
    } else target.append(renderValue(schema, [key], state.boundary[key]));
  }
  function render() {
    if(state.boundary) { state.boundary.execution.solver ??= 'highs'; state.boundary.execution.threads ??= 0; }
    window.HySimMarketCanvas.setBoundary(state.boundary, state.revision);
    const category = $('southernCategory'); const current = category.value; category.replaceChildren();
    if (!state.schema || !state.boundary) { $('southernEditor').replaceChildren(); return; }
    Object.entries(state.schema.properties).forEach(([key, schema]) => {
      const option = text('option', schema.title); option.value = key; category.append(option);
    });
    category.value = [...category.options].some(o => o.value === current) ? current : 'areas';
    const b = state.boundary;
    const counts = {};
    b.generators.forEach(g => { counts[g.kind] = (counts[g.kind] || 0) + 1; });
    $('southernCaseSummary').textContent = `${b.name} · ${b.buses.length} 节点 / ${b.branches.length} 支路 / ${b.generators.length} 机组 · 水 ${counts.hydro || 0} / 火 ${counts.thermal || 0} / 风 ${counts.wind || 0} / 光 ${counts.solar || 0} · 储能 ${b.storage.length} / 水库 ${b.reservoirs.length} / 可控负荷 ${b.controllable_loads?.length || 0} · 报价来源：${b.source}`;
    renderEntities(); renderResults(state.latest);
  }
  function table(headers, rows) {
    const wrapper = document.createElement('div'); wrapper.className = 'topo-table-wrap';
    const table = document.createElement('table'); const head = document.createElement('thead'); const tr = document.createElement('tr');
    headers.forEach(h => tr.append(text('th', h))); head.append(tr); table.append(head);
    const body = document.createElement('tbody'); let page = 0;
    const draw = () => {
      body.replaceChildren();
      rows.slice(page * 100, (page + 1) * 100).forEach(row => { const tr = document.createElement('tr'); row.forEach(v => { const td = document.createElement('td'); if (v instanceof Node) td.append(v); else td.textContent = String(v ?? '未提供'); tr.append(td); }); body.append(tr); });
    };
    table.append(body); wrapper.append(table);
    if (rows.length > 100) {
      const controls = document.createElement('div'); controls.className = 'southern-actions';
      const prev = text('button', '←'); const next = text('button', '→'); const label = text('span', '');
      prev.type = next.type = 'button'; prev.title = '上一页'; next.title = '下一页';
      const update = () => { draw(); label.textContent = `${page+1} / ${Math.ceil(rows.length/100)} · ${rows.length} 条`; prev.disabled = page === 0; next.disabled = (page+1)*100 >= rows.length; };
      prev.onclick = () => { --page; update(); }; next.onclick = () => { ++page; update(); };
      controls.append(prev, label, next); wrapper.prepend(controls); update();
    } else draw();
    return wrapper;
  }
  function renderResults(result) {
    const target = $('southernResults'); target.replaceChildren();
    if (typeof Plotly !== 'undefined') Plotly.purge($('southernChart'));
    if (!result) return;
    target.append(text('h5', `出清状态：${result.status}${result.stale ? ' · 输入已变更' : ''}`));
    target.append(table(['阶段', '状态', '目标 / 元', '最大残差', '变量 / 离散变量'], ['scuc', 'sced', 'lmp'].filter(k => result[k]).map(k => {
      const r = result[k]; return [k.toUpperCase(), r.solver_status, r.objective?.toFixed(3), r.max_residual, `${r.variables} / ${r.binary_variables}`];
    })));
    const scope = result.model_scope;
    const details = document.createElement('details'); details.append(text('summary', '执行解释与验证范围'));
    details.append(text('p', `调频预出清来源：${scope.regulation}`));
    scope.limitations.forEach(v => details.append(text('p', v))); target.append(details);
    if (result.error) target.append(text('p', result.error));
    if (result.comparison) {
      const c = result.comparison; target.append(text('h5', '边界情景 − 基准'));
      target.append(table(['可比较', '运行日电能费用变化 / 元', '运行日发电量变化 / MWh', '边界修改项'], [[c.comparable, c.delta_day_energy_bid_cost?.toFixed(3), c.delta_day_generation_mwh?.toFixed(3), c.boundary_changes?.length]]));
      if (c.boundary_changes) target.append(table(['字段路径', '修改', '值'], c.boundary_changes.map(d => [d.path, d.op, JSON.stringify(d.value)])));
    }
    if (result.sced?.generators) target.append(table(['机组 ID', '名称', '运行日电量 / MWh'], result.sced.generators.map(g => [g.id, g.name, g.power_mw.slice(0, 96).reduce((a, p) => a + p * 0.25, 0).toFixed(3)])));
    const selectors = document.createElement('div'); selectors.className = 'southern-actions';
    const stage = document.createElement('select'); stage.setAttribute('aria-label', '出清阶段');
    ['scuc', 'sced', 'lmp'].filter(k => result[k]).forEach(k => { const o = text('option', k.toUpperCase()); o.value = k; stage.append(o); });
    const category = document.createElement('select'); category.setAttribute('aria-label', '结果类别');
    Object.entries({ generators: '机组出力与启停', storage: '储能功率与能量', branches: '线路潮流与松弛', sections: '断面潮流与松弛',
      dc_links: '直流功率与调节方向', reservoirs: '水位与下泄', controllable_loads: '可控负荷削减', trades: '交易成分', buses: '节点电价与平衡残差' }).forEach(([k, label]) => {
      const o = text('option', label); o.value = k; category.append(o);
    });
    const period = document.createElement('input'); period.className = 'southern-input'; period.type = 'number'; period.min = '1'; period.max = '98'; period.value = '1'; period.step = '1'; period.setAttribute('aria-label', '结果时点');
    const detail = document.createElement('div');
    const fieldLabels = { id: 'ID', name: '名称', bus: '母线', power_mw: '出力 MW', online: '开机', start: '启动', stop: '停机', hot_start: '热态启动',
      warm_start: '温态启动', cold_start: '冷态启动', trajectory_mw: '启停轨迹 MW', renewable_deviation_mw: '新能源偏差 MW',
      discharge_mw: '放电 MW', charge_mw: '充电 MW', energy_mwh: '能量 MWh', slack_plus_mw: '正松弛 MW', slack_minus_mw: '负松弛 MW',
      up: '上调', down: '下调', overload_mw: '有功越限 MW', node_imbalance_mw: '节点平衡残差 MW', reduction_mw: '负荷削减 MW', kind: '类型', level_m: '水位 m', spill_m3_s: '泄洪 m³/s', release_m3_s: '下泄 m³/s', priority_shortfall_mwh: '优先电量缺口 MWh', lmp_per_mwh: '电价 元/MWh' };
    const draw = () => {
      const rows = result[stage.value]?.[category.value] || []; const t = Math.max(0, Math.min(97, Number(period.value) - 1));
      window.HySimMarketCanvas.southern(state.boundary, result, stage.value, t);
      detail.replaceChildren(); if (!rows.length) { detail.append(text('p', '该类别无交易单元')); return; }
      const fields = Object.keys(rows[0]);
      detail.append(table(fields.map(f => fieldLabels[f] || f), rows.map(row => fields.map(f => {
        const v = Array.isArray(row[f]) ? row[f][t] : row[f];
        if (f === 'id' && ['buses','branches','generators','storage','controllable_loads','reservoirs','dc_links'].includes(category.value)) return window.HySimMarketCanvas.link(category.value, v);
        return typeof v === 'number' ? Number(v.toFixed(6)) : v;
      }))));
    };
    stage.addEventListener('change', draw); category.addEventListener('change', draw); period.addEventListener('change', draw);
    selectors.append(stage, category, text('label', '时点'), period); target.append(selectors, detail); draw();
    const audit = document.createElement('details'); audit.append(text('summary', '约束残差与交流安全迭代'));
    for (const k of ['scuc', 'sced', 'lmp']) if (result[k]?.constraint_families)
      audit.append(text('h5', k.toUpperCase()), table(['约束族', '行数', '最大违反量'], Object.entries(result[k].constraint_families).map(([name, r]) => [name, r.rows, r.max_violation])));
    for (const iteration of result.security_iterations || []) {
      audit.append(text('h5', `交流安全迭代 ${iteration.iteration + 1} · ${iteration.secure ? '通过' : '未通过'}`));
      audit.append(table(['时点', '收敛', '网损 MW', '违反项'], iteration.periods.map(p => [p.period + 1, p.converged, p.losses_mw?.toFixed(6), p.violations.map(v => `${v.name}: ${v.excess}`).join('; ')])));
    }
    target.append(audit);
    if (result.lmp?.prices_valid && !$('southernChart').hidden && typeof Plotly !== 'undefined') {
      const style = getComputedStyle(document.documentElement);
      const traces = result.lmp.buses.length > 80 ? result.lmp.buses.slice(0, 80) : result.lmp.buses;
      Plotly.newPlot($('southernChart'), traces.map(b => ({ name: `${b.id} ${b.name}`, x: Array.from({ length: 96 }, (_, i) => i / 4), y: b.lmp_per_mwh.slice(0, 96), mode: 'lines', type: 'scatter' })),
        { title: '运行日节点电价', font: { color: style.getPropertyValue('--ink').trim(), size: 11 },
          xaxis: { title: '小时', gridcolor: style.getPropertyValue('--border').trim() },
          yaxis: { title: '元 / MWh', gridcolor: style.getPropertyValue('--border').trim() },
          margin: { t: 40, l: 65, r: 20, b: 45 }, paper_bgcolor: 'transparent', plot_bgcolor: 'transparent' }, { responsive: true });
    }
  }
  async function load() {
    App.showSouthernMarketWorkspace();
    view(false);
    const data = await api('/api/session/southern_market'); Object.assign(state, data, { dirty: false }); render();
    status(state.boundary ? `已载入边界 · 修订 ${state.revision}${state.baseline ? ' · 已有基准' : ''}` : '尚无南方市场边界');
  }
  async function action(name) {
    const data = await api('/api/session/southern_market', { action: name, revision: state.revision, ...(name === 'save' ? { boundary: state.boundary } : {}) });
    Object.assign(state, data, { dirty: false });
    if (name !== 'pin_baseline') state.latest = null;
    if (name !== 'pin_baseline') view(false);
    render(); status(`边界已保存 · 修订 ${state.revision}${state.baseline ? ' · 已有基准' : ''}`);
  }
  function download(value, filename) {
    if (!value) throw new Error('暂无可导出数据');
    const url = URL.createObjectURL(new Blob([JSON.stringify(value, null, 2)], { type: 'application/json' }));
    const a = document.createElement('a'); a.href = url; a.download = filename; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  async function run() {
    App.showSouthernMarketWorkspace();
    if (!state.boundary) await load();
    if (!state.boundary) throw new Error('请先建立并保存南方市场边界');
    if (state.dirty) await action('save');
    status('SCUC → 调频容量改限 → SCED → 交流校核 → 独立 LMP');
    const result = await api('/api/session/run_southern_market', { revision: state.revision });
    state.latest = result; view(true); renderResults(result); status(`南方规则：${result.status}`);
    $('southernMarketWorkspace').scrollIntoView({ block: 'nearest' });
  }
  function bind(id, fn) {
    $(id).addEventListener('click', async () => {
      if (state.busy) return;
      state.busy = true;
      try { await fn(); } catch (e) { status(e.message); } finally { state.busy = false; }
    });
  }
  bind('btnSouthernOpen', async () => { await load(); $('southernMarketWorkspace').scrollIntoView({ block: 'nearest' }); });
  bind('southernBoundaryView', () => view(false)); bind('southernResultView', () => { view(true); renderResults(state.latest); });
  bind('btnSouthernRun', run); bind('southernLoad', load);
  bind('southernRunSaved', run);
  bind('southernExample', async () => { if (!state.schema) await load(); await action('example'); });
  bind('southernLoadCase', async () => {
    if (!state.schema) await load();
    status('正在加载市场算例'); await action($('southernCase').value);
  });
  bind('southernFromSystem', async () => { if (!state.schema) await load(); await action('from_system'); });
  bind('southernSave', () => action('save')); bind('southernBaseline', () => action('pin_baseline')); bind('southernRestore', () => action('restore_baseline'));
  bind('southernExport', () => download(state.boundary, 'southern-boundary.json'));
  bind('southernExportResult', () => download(state.latest, 'southern-clearing-result.json'));
  bind('southernImport', () => $('southernImportFile').click());
  $('southernImportFile').addEventListener('change', async e => {
    try {
      const file = e.target.files[0]; if (!file) return;
      if (file.size > 128 * 1024 * 1024) throw new Error('边界文件超过 128 MiB');
      if (!state.schema) await load();
      const candidate = JSON.parse(await file.text());
      const data = await api('/api/session/southern_market', { action: 'save', revision: state.revision, boundary: candidate });
      Object.assign(state, data, { dirty: false, latest: null }); render(); status('已导入并校验边界');
    } catch (e) { status(e.message); } finally { e.target.value = ''; }
  });
  $('southernCategory').addEventListener('change', renderEntities); $('southernEntity').addEventListener('change', renderEditor);
  bind('southernAdd', () => {
    const key = $('southernCategory').value; const schema = state.schema.properties[key]; const values = state.boundary[key];
    if (values.length >= schema.maxItems) throw new Error('实体数达到上限');
    const row = initial(schema.items); row.id = Math.max(-1, ...values.map(v => v.id)) + 1;
    pathSet([key], [...values, row]); renderEntities(); $('southernEntity').value = String(values.length); renderEditor();
  });
  bind('southernDelete', () => {
    const key = $('southernCategory').value; const values = state.boundary[key]; const i = Number($('southernEntity').value);
    if (values.length <= state.schema.properties[key].minItems) return;
    pathSet([key], values.filter((_, k) => k !== i)); renderEntities();
  });
  document.querySelectorAll('#southernMarketWorkspace button').forEach(b => b.classList.add('btn', 'btn-sm'));
  $('southernSave').classList.add('btn-primary');
  view(false);
  document.addEventListener('southern-market-open', () => {
    if (state.dirty || state.busy) { App.showSouthernMarketWorkspace(); return; }
    load().catch(e => status(e.message));
  });
  document.addEventListener('market-canvas-edit', async e => {
    try {
      if (!state.dirty && !state.busy) await load();
      const { type, id } = e.detail;
      const position = state.boundary?.[type]?.findIndex(row => row.id === id);
      if (position == null || position < 0) return;
      view(false); $('southernCategory').value = type; renderEntities();
      $('southernEntity').value = position; renderEditor();
      window.HySimMarketCanvas.select(`${type}:${id}`);
    } catch (e) { status(e.message); }
  });
  const openDirect = () => {
    if (location.hash !== '#southern-market') return;
    App.setActiveModule('marketBoundary');
    load().catch(e => status(e.message));
  };
  window.addEventListener('hashchange', openDirect);
  if (document.readyState === 'complete') openDirect();
  else window.addEventListener('load', openDirect, { once: true });
})();
