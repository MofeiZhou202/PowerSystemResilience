/* Independent Southern identities; never mutate the engineering Canvas model.
 * Mapping and validity contract: docs/modules/market/southern_execution_contract.md. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const node = (tag, text) => { const n = document.createElement(tag); if (text != null) n.textContent = String(text); return n; };
  const svgNode = (tag, attrs = {}, text) => { const n = document.createElementNS('http://www.w3.org/2000/svg', tag); for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v); if (text != null) n.textContent = String(text); return n; };
  const fmt = v => typeof v === 'number' && Number.isFinite(v) ? v.toLocaleString('zh-CN', { maximumFractionDigits: 3 }) : '不可用';
  const at = (r, k, t) => Array.isArray(r?.[k]) ? r[k][t] : r?.[k];
  const names = { buses: '交流节点', branches: '交流线路', generators: '机组', storage: '储能', controllable_loads: '可控负荷', reservoirs: '水库', dc_links: '直流联络' };
  const kinds = { hydro: '水', thermal: '火', wind: '风', solar: '光' };
  const state = { boundary: null, revision: null, entities: new Map(), selected: null, frame: null, view: null, timer: null, frameSerial: 0 };
  const root = node('section'); root.id = 'marketCanvas'; root.setAttribute('aria-label', '市场模拟 Canvas');
  const toolbar = node('div'); toolbar.className = 'market-canvas-toolbar';
  const title = node('strong', '市场拓扑'); title.id = 'marketCanvasTitle';
  const tools = node('div'); tools.className = 'market-canvas-tools';
  function button(text, label, fn) { const b = node('button', text); b.type = 'button'; b.className = 'btn btn-sm'; b.title = label; b.setAttribute('aria-label', label); b.onclick = fn; return b; }
  const previous = button('←', '上一时段', () => advance(-1));
  const play = button('▶', '播放市场时段', () => { if (state.timer) stop(); else { state.timer = setInterval(() => advance(1), 700); play.textContent = 'Ⅱ'; play.setAttribute('aria-label', '暂停市场时段'); } }); play.id = 'marketCanvasPlay';
  const next = button('→', '下一时段', () => advance(1));
  const followLabel = node('label'); const follow = node('input'); follow.type = 'checkbox'; follow.checked = true; follow.id = 'marketCanvasFollow'; followLabel.append(follow, node('span', '跟随计算'));
  tools.append(previous, play, next, button('+', '放大市场拓扑', () => zoom(.8)), button('−', '缩小市场拓扑', () => zoom(1.25)), button('⤢', '适应市场拓扑', () => draw()), followLabel);
  const picker = node('select'); picker.id = 'marketCanvasEntity'; picker.setAttribute('aria-label', '市场设备定位'); picker.onchange = () => select(picker.value);
  const caption = node('div'); caption.id = 'marketCanvasContext'; caption.setAttribute('aria-live', 'polite');
  const progress = node('div'); progress.id = 'marketCanvasProgress'; progress.setAttribute('aria-live', 'polite');
  toolbar.append(title, tools, picker, caption, progress);
  const timeControls = node('div'); timeControls.className = 'market-canvas-time';
  const mirrors = new Map();
  for (const [source, label] of [['forecastScenario','场景'],['operationResultDay','日期'],['operationSlot','时段']]) {
    const wrap = node('label',label), select = node('select'); select.id = `marketCanvas-${source}`; select.setAttribute('aria-label', `Canvas ${label}`);
    select.onchange = () => {
      if (source === 'operationSlot' && !document.body.classList.contains('market-operation-active')) { const p = document.querySelector('[aria-label="结果时点"]'); if (p) { p.value = Number(select.value)+1; p.dispatchEvent(new Event('change')); } }
      else { $(source).value = select.value; $(source).dispatchEvent(new Event('change')); }
    };
    wrap.append(select); timeControls.append(wrap); mirrors.set(source,{wrap,select});
  }
  toolbar.insertBefore(timeControls,caption);
  const svg = svgNode('svg', { id: 'marketTopology', role: 'group', 'aria-label': '市场节点与线路拓扑', viewBox: '0 0 800 500' });
  const legend = node('div', '红：缺额 / 越限 · 青：富余 · 绿：已出清无异常 · 灰：无有效结果'); legend.className = 'market-canvas-legend';
  const inspector = node('div'); inspector.id = 'marketCanvasDetails'; inspector.setAttribute('aria-live', 'polite');
  root.append(toolbar, svg, legend, inspector); $('canvasContainer').append(root);
  const key = (type, id) => `${type}:${id}`;
  function stop() { clearInterval(state.timer); state.timer = null; play.textContent = '▶'; play.setAttribute('aria-label', '播放市场时段'); }
  function advance(delta) {
    if (!document.body.classList.contains('market-canvas-active')) { stop(); return; }
    if (document.body.classList.contains('market-operation-active')) {
      const slot = $('operationSlot'), day = $('operationResultDay'); let t = Number(slot.value) + delta;
      if (t < 0 || t > 95) { const d = day.selectedIndex + delta; if (d < 0 || d >= day.options.length) { stop(); return; } day.selectedIndex = d; t = t < 0 ? 95 : 0; }
      slot.value = t; slot.dispatchEvent(new Event('change'));
    } else {
      const period = document.querySelector('[aria-label="结果时点"]');
      if (!period) { stop(); return; }
      const t = Number(period.value) + delta; if (t < 1 || t > 98) { stop(); return; }
      period.value = t; period.dispatchEvent(new Event('change'));
    }
  }
  function applyView() { if (state.view) svg.setAttribute('viewBox', state.view.join(' ')); }
  function zoom(factor) { if (!state.view) return; const [x,y,w,h] = state.view; if (w*factor < 40 || w*factor > 50000) return; state.view = [x+w*(1-factor)/2,y+h*(1-factor)/2,w*factor,h*factor]; applyView(); }
  svg.addEventListener('wheel', e => { e.preventDefault(); zoom(e.deltaY > 0 ? 1.15 : 1/1.15); }, { passive: false });
  let drag = null;
  svg.addEventListener('pointerdown', e => { if (e.target.closest('[data-market-ref]')) return; drag = { x: e.clientX, y: e.clientY, view: [...state.view] }; svg.setPointerCapture(e.pointerId); });
  svg.addEventListener('pointermove', e => { if (!drag) return; const rect = svg.getBoundingClientRect(), scale = Math.max(drag.view[2]/rect.width, drag.view[3]/rect.height); state.view = [drag.view[0]-(e.clientX-drag.x)*scale,drag.view[1]-(e.clientY-drag.y)*scale,...drag.view.slice(2)]; applyView(); });
  for (const event of ['pointerup','pointercancel']) svg.addEventListener(event, () => { drag = null; });
  function setBoundary(boundary, revision) {
    stop(); state.boundary = boundary; state.revision = revision; state.frame = null; state.entities.clear();
    for (const type of Object.keys(names)) for (const row of boundary?.[type] || []) {
      const ref = key(type, row.id); if (state.entities.has(ref)) { state.boundary = null; caption.textContent = `重复市场设备 ID：${ref}`; svg.replaceChildren(); inspector.replaceChildren(); return; }
      state.entities.set(ref, { type, row });
    }
    picker.replaceChildren(...[...state.entities].map(([ref, { type, row }]) => { const o = node('option', `${names[type]} ${row.id} · ${row.name || ''}`); o.value = ref; return o; }));
    if (!state.entities.has(state.selected)) state.selected = state.entities.keys().next().value;
    title.textContent = boundary ? `${boundary.name} · 市场拓扑` : '市场拓扑 · 尚未载入';
    draw(); update();
  }
  function focusBus() {
    const e = state.entities.get(state.selected); if (!e) return null;
    if (e.type === 'buses') return e.row.id;
    if (e.type === 'reservoirs') return state.boundary.generators.find(g => (e.row.generators || [e.row.generator]).includes(g.id))?.bus;
    return e.row.bus ?? e.row.from_bus;
  }
  function select(ref) {
    if (!state.entities.has(ref)) return;
    const restoreFocus = document.activeElement?.dataset.marketRef === ref;
    state.selected = ref; picker.value = ref; draw(); update();
    if (restoreFocus) [...svg.querySelectorAll('[data-market-ref]')].find(n=>n.dataset.marketRef === ref)?.focus();
  }
  function target(shape, type, row) {
    const ref = key(type, row.id); shape.dataset.marketRef = ref; shape.setAttribute('role', 'button'); shape.setAttribute('tabindex', '0'); shape.setAttribute('aria-label', `${names[type]} ${row.id} ${row.name || ''}`);
    shape.addEventListener('click', () => select(ref)); shape.addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); select(ref); } });
    shape.append(svgNode('title', {}, `${names[type]} ${row.id} ${row.name || ''}`)); return shape;
  }
  function draw() {
    svg.replaceChildren(); const b = state.boundary; if (!b) { state.view = [0,0,800,500]; applyView(); return; }
    // Bounded neighborhood, using stable bus IDs. Layout is schematic, not geographic.
    let buses = b.buses; const center = focusBus() ?? buses[0]?.id;
    if (buses.length > 80) {
      const adjacency = new Map(buses.map(n => [n.id, []]));
      for (const l of b.branches) { adjacency.get(l.from_bus)?.push(l.to_bus); adjacency.get(l.to_bus)?.push(l.from_bus); }
      const entity = state.entities.get(state.selected);
      const queue = entity?.type === 'branches' ? [...new Set([entity.row.from_bus,entity.row.to_bus])] : [center], seen = new Set(queue);
      for (let i = 0; i < queue.length && queue.length < 80; ++i) for (const id of adjacency.get(queue[i]) || []) if (!seen.has(id) && queue.length < 80) { seen.add(id); queue.push(id); }
      buses = buses.filter(n => seen.has(n.id));
    }
    const positions = new Map(); const columns = Math.max(1, Math.ceil(Math.sqrt(buses.length)));
    buses.forEach((n,i) => positions.set(n.id, { x: 110 + (i%columns)*230, y: 100 + Math.floor(i/columns)*160 }));
    const width = Math.max(440, columns*230), height = Math.max(280, Math.ceil(buses.length/columns)*160+50);
    svg.dataset.visibleBuses = buses.length; svg.dataset.totalBuses = b.buses.length;
    const links = b.branches.filter(l => positions.has(l.from_bus) && positions.has(l.to_bus));
    const pairs = new Map();
    for (const l of links) {
      const a = positions.get(l.from_bus), z = positions.get(l.to_bus), pair = [l.from_bus,l.to_bus].sort((x,y)=>x-y).join(':');
      const offset = (pairs.get(pair) || 0)*25; pairs.set(pair, (pairs.get(pair) || 0)+1);
      const cx = (a.x+z.x)/2+offset, cy = (a.y+z.y)/2-30-offset;
      const d = a === z ? `M ${a.x} ${a.y} c -70 -90 70 -90 0 0` : `M ${a.x} ${a.y} Q ${cx} ${cy} ${z.x} ${z.y}`;
      const g = target(svgNode('g'), 'branches', l);
      g.append(svgNode('path', { d, class: 'market-line-hit' }), svgNode('path', { d, class: 'market-line' }), svgNode('text', { x: cx, y: cy-5, class: 'market-line-label', 'text-anchor': 'middle' })); svg.append(g);
    }
    for (const n of buses) {
      const p = positions.get(n.id), g = target(svgNode('g'), 'buses', n);
      g.append(svgNode('circle', { cx: p.x, cy: p.y, r: 19, class: 'market-bus' }), svgNode('text', { x: p.x, y: p.y+4, 'text-anchor': 'middle', class: 'market-bus-id' }, n.id), svgNode('text', { x: p.x, y: p.y+39, 'text-anchor': 'middle', class: 'market-bus-label' }));
      const counts = {};
      for (const gen of b.generators || []) if (gen.bus === n.id) { const k = kinds[gen.kind] || gen.kind; counts[k] = (counts[k] || 0)+1; }
      for (const [type,label] of [['storage','储'],['controllable_loads','荷']]) { const count = b[type]?.filter(r => r.bus === n.id).length; if (count) counts[label] = count; }
      g.append(svgNode('text', { x: p.x, y: p.y+58, 'text-anchor': 'middle', class: 'market-assets-label' }, Object.entries(counts).map(([k,v])=>`${k}${v}`).join(' · '))); svg.append(g);
    }
    state.view = [0,0,width,height]; applyView();
  }
  function resultRow(type, id) { const f = state.frame; return f?.valid ? f.rows?.[type]?.find(r => r.id === id) : null; }
  function update() {
    const f = state.frame, t = f?.slot || 0;
    const operationMode = document.body.classList.contains('market-operation-active');
    for (const [source,{wrap,select}] of mirrors) {
      const sourceSelect = $(source);
      wrap.hidden = source === 'forecastScenario' ? !operationMode || $('marketForecastWorkspace').hidden || !sourceSelect.options.length : source === 'operationResultDay' ? !operationMode : false;
      const values = source === 'operationSlot' && !operationMode ? Array.from({length:98},(_,i)=>[String(i),`时点 ${i+1}`]) : [...sourceSelect.options].map(o=>[o.value,o.textContent]);
      const signature = JSON.stringify(values);
      if (select.dataset.options !== signature) { select.replaceChildren(...values.map(([v,text])=>{const o=node('option',text);o.value=v;return o;})); select.dataset.options=signature; }
      select.value = source === 'operationSlot' ? String(t) : sourceSelect.value;
      select.disabled = !values.length;
    }
    picker.value = state.selected || ''; root.dataset.frame = ++state.frameSerial; root.dataset.slot = t; root.dataset.selected = state.selected || '';
    root.dataset.valid = String(!!f?.valid);
    const scope = state.boundary ? ` · ${svg.dataset.visibleBuses}/${svg.dataset.totalBuses} 节点${state.boundary.buses.length > 80 ? '（所选设备邻域）' : ''} · 示意布局` : '';
    caption.textContent = `${f?.label || '已保存基准边界 · 尚无出清结果'}${scope}`;
    const enabled = document.body.classList.contains('market-operation-active') ? $('operationResultDay').options.length > 0 : !!document.querySelector('[aria-label="结果时点"]');
    previous.disabled = next.disabled = play.disabled = !enabled;
    for (const g of svg.querySelectorAll('[data-market-ref]')) {
      const e = state.entities.get(g.dataset.marketRef), r = resultRow(e.type, e.row.id);
      g.classList.toggle('selected', g.dataset.marketRef === state.selected);
      const value = e.type === 'buses' ? (typeof at(r,'deficit_mw',t) === 'number' && typeof at(r,'surplus_mw',t) === 'number' ? at(r,'deficit_mw',t)-at(r,'surplus_mw',t) : null) : at(r,'overload_mw',t);
      const color = typeof value !== 'number' ? '#8a909b' : value > 1e-6 ? '#e76066' : value < -1e-6 ? '#42bfbc' : '#4aaa77';
      g.style.setProperty('--market-state', color); g.dataset.value = value ?? '';
      const label = g.querySelector('.market-bus-label, .market-line-label');
      label.textContent = e.type === 'buses' ? `ΔP ${fmt(value)} MW` : `L${e.row.id} · ΔPᵢⱼ ${fmt(value)} MW`;
      if (e.type === 'branches') g.classList.toggle('unavailable', at(r || e.row,'available',t) === 0);
    }
    details();
  }
  function details() {
    inspector.replaceChildren(); const e = state.entities.get(state.selected); if (!e) return;
    const { type,row } = e, f = state.frame, t = f?.slot || 0, r = resultRow(type,row.id);
    inspector.append(node('strong', `${names[type]} ${row.id} · ${row.name || ''}`));
    const data = node('dl');
    const item = (label,value) => { data.append(node('dt', label),node('dd', value)); };
    const mw = v => `${fmt(v)} MW`;
    if (type === 'buses') {
      const effectiveBus = r?.load_mw ? r : f?.effective?.buses?.find(n => n.id === row.id);
      item(effectiveBus ? '生效负荷' : '基准节点负荷（未出清）', mw(at(effectiveBus || row,'load_mw',t)));
      const deficit = at(r,'deficit_mw',t), surplus = at(r,'surplus_mw',t);
      item('ΔP（缺额 − 富余）', mw(typeof deficit === 'number' && typeof surplus === 'number' ? deficit-surplus : null));
      item('缺额 / 富余', `${fmt(deficit)} / ${fmt(surplus)} MW`); item('平衡方程残差', mw(at(r,'node_imbalance_mw',t)));
      const price = f?.valid ? f?.prices?.find(n=>n.id === row.id) || r : null; item('节点价格', `${fmt(at(price,'lmp_per_mwh',t))} 元/MWh`);
    } else if (type === 'branches') {
      item('端点', `${row.from_bus} → ${row.to_bus}`); item('线路功率', mw(at(r,'power_mw',t))); item('ΔPᵢⱼ', mw(at(r,'overload_mw',t)));
      const limits = f?.limits?.find(l=>l.id === row.id) || row;
      const effective = r?.min_mw ? r : limits;
      item(f?.valid ? '生效有功限额' : '基准有功限额', `${fmt(at(effective,'min_mw',t)*at(effective,'available',t))} ～ ${fmt(at(effective,'max_mw',t)*at(effective,'available',t))} MW`);
      item('投运', at(effective,'available',t) === 0 ? '否' : '是');
    } else {
      item('接入节点', row.bus ?? '见关联设备'); item('类型', kinds[row.kind] || names[type]);
      for (const [field,label] of [['power_mw','出力 MW'],['energy_mwh','能量 MWh'],['reduction_mw','削减 MW'],['level_m','水位 m'],['release_m3_s','下泄 m³/s']]) if (r?.[field]) item(label,fmt(at(r,field,t)));
      if (!r) item('设备出清明细', '当前报告未提供');
    }
    inspector.append(data);
    if (!f?.valid) inspector.append(node('p', '当前时段无有效出清结果。'));
    if (f?.valid && f?.day?.valid) {
      const p = f.day.periods[t];
      const cause = type === 'branches' && at(r,'overload_mw',t) > 1e-6 ? '线路功率超出生效限额；研究罚项允许越限。' : type === 'buses' && at(r,'deficit_mw',t) > 1e-6 ? (p.load_mw > p.available_generation_mw ? '系统负荷高于机组可用容量上界，需结合储能、受电、可控负荷与网络复核。' : '总容量上界未解释缺额，需复核输送、水量及机组约束。') : '原因证据见当日约束与恢复重算。';
      inspector.append(node('p', cause));
      for (const c of f.day.counterfactuals || []) inspector.append(node('p', c.valid ? `恢复 ${c.factor === 'boundary_overrides' ? '设备级边界（联合）' : c.factor}：系统缺额减少 ${fmt(p.deficit_mw-c.periods[t].deficit_mw)} MW；线路越限和减少 ${fmt(p.overload_sum_mw-c.periods[t].overload_sum_mw)} MW。` : `恢复 ${c.factor}：重算无有效结果。`));
      inspector.append(node('p', '恢复差值为系统条件敏感性，不是所选设备的唯一因果归因。'));
      inspector.append(button('查看约束与恢复重算', '查看当日原因证据', () => $('operationCauses').scrollIntoView({ block: 'start' })));
    }
    const assets = node('div'); assets.className = 'market-canvas-assets';
    if (type === 'buses') for (const [ref, entity] of state.entities) if (entity.row.bus === row.id && entity.type !== 'buses') assets.append(button(`${names[entity.type]} ${entity.row.id}`, `${names[entity.type]} ${entity.row.id}`, () => select(ref)));
    inspector.append(assets);
    const raw = node('details'); raw.append(node('summary','基准边界与报价'), node('pre',JSON.stringify(row,null,2))); inspector.append(raw);
    inspector.append(button('编辑基准边界', '编辑所选市场设备基准边界', () => { App.setActiveModule('marketBoundary'); document.dispatchEvent(new CustomEvent('market-canvas-edit', { detail: { type, id: row.id } })); }));
  }
  function operation(report, dayIndex, slot, revision) {
    if (!document.body.classList.contains('market-operation-active')) return;
    const day = report?.days?.find(d => d.day === dayIndex), date = report?.config?.start_date;
    let d = date ? new Date(`${date}T00:00:00Z`) : null; if (d && Number.isFinite(d.getTime())) d.setUTCDate(d.getUTCDate()+dayIndex); else d = null;
    const valid = !!day?.valid && report.boundary_revision === revision && revision === state.revision;
    state.frame = { valid, slot, day, boundary: day?.boundary || report?.config?.days?.[dayIndex], rows: { buses: day?.nodes, branches: day?.lines },
      label: `${report?.id != null ? `场景 ${report.id+1} · ` : ''}${d ? d.toISOString().slice(0,10) : '待运行'} · ${String(Math.floor(slot/4)).padStart(2,'0')}:${String(slot%4*15).padStart(2,'0')} · ${valid ? '诊断出清（非交流校核）' : report?.boundary_revision !== revision ? '边界已变化，结果不可用' : day ? '出清失败，结果不可用' : '尚未出清'}` };
    update();
  }
  function southern(boundary, result, stage, slot) {
    if (document.body.classList.contains('market-operation-active')) return;
    const rows = result?.[stage]; state.frame = { valid: !!rows?.buses?.length && rows.feasible === true && !result.stale && result.schedule_feasible === true, rows, slot, effective: result?.effective_boundary, limits: result?.effective_boundary?.branches || boundary?.branches, prices: result?.prices_valid ? result?.lmp?.buses : null, label: `${stage.toUpperCase()} · 时点 ${slot+1} · ${result?.status || '尚未出清'}` }; update();
  }
  window.HySimMarketCanvas = { setBoundary, operation, southern, select, stopPlayback: stop, follow: () => follow.checked,
    progress(message, operationMode = true) { if (document.body.classList.contains('market-operation-active') === operationMode) progress.textContent = message; },
    link(type,id) { const b = button(String(id), `定位${names[type]} ${id}`, () => { select(key(type,id)); svg.scrollIntoView({block:'nearest'}); }); b.dataset.marketTarget = key(type,id); return b; } };
})();
