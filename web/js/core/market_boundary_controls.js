/* Rules 2.3/2.4 catalog and admitted fields are owned by the C++ schema. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id), shared = window.HySimMarketOperation;
  const state = { market: null, catalog: null };
  const el = (tag,text) => { const n = document.createElement(tag); if (text != null) n.textContent = String(text); return n; };
  const options = (select,values) => { const old = select.value; select.replaceChildren(...values.map(([v,t])=>{const n=el('option',t);n.value=v;return n;})); if(values.some(([v])=>v===old))select.value=old; };
  const group = () => state.catalog?.items.find(i=>i.id===$('operationRuleCategory').value);
  const field = () => group()?.fields.find(f=>`${f.table}.${f.field}`===$('operationBoundaryField').value);
  const feedback = text => { $('operationBoundaryFeedback').textContent=text; };
  const bound = () => state.market?.boundary;
  function refreshDay() {
    const edits=shared.day()?.boundary_overrides || [];
    $('operationBoundaryEdits').replaceChildren(shared.table(['实体 / 字段','时点','值','依据',''],edits.map((e,i)=>{
      const remove=el('button','×');remove.type='button';remove.title='删除覆盖';remove.setAttribute('aria-label',`删除覆盖 ${i+1}`);remove.className='btn btn-sm';
      remove.onclick=()=>{shared.setEdits(edits.filter((_,k)=>k!==i));refreshDay();};
      return [`${e.table}:${e.id} / ${e.field}`,`${e.first_slot+1}–${e.last_slot+1}`,e.value,e.reason,remove];
    })));
    $('operationBoundaryResolved').replaceChildren();feedback(''); value();
  }
  function value() {
    const f=field(), input=$('operationBoundaryValue'); if(!f)return;
    const schema=f.schema.type==='array'?f.schema.items:f.schema;
    const row=bound()?.[f.table]?.find(r=>String(r.id)===$('operationBoundaryEntity').value);
    const raw=row?.[f.field], t=Math.max(0,Number($('operationOverrideFirst').value)-1);
    const v=Array.isArray(raw)?raw[t]:raw;
    input.type=schema.type==='string'?'text':'number';input.step=schema.type==='integer'?'1':'any';
    if(schema.minimum!=null)input.min=schema.minimum;else input.removeAttribute('min');
    if(schema.maximum!=null)input.max=schema.maximum;else input.removeAttribute('max');
    input.value=v??'';
    $('operationBoundaryValueLabel').firstChild.textContent=`${schema.title}${schema.unit?`（${schema.unit}）`:''}`;
    $('operationBoundaryAuthored').textContent=row?`基准值：${v??'缺失'}${schema.unit?` ${schema.unit}`:''}；来源：${row.source}；${f.schema.type==='array'?'本日96点；上一日的峰谷预安排同步取值。':'单值作用于当天整个日窗。'}`:'当前类别无实体，先在完整基准边界建立实体及关联。';
    const terminal = Number($('operationEditDay').value) === $('operationEditDay').options.length-1;
    $('operationBoundaryAdd').disabled=!row || (terminal && f.schema.type!=='array');
    if(terminal && f.schema.type!=='array') $('operationBoundaryAuthored').textContent+=' 仅预测日不接受独立标量边界。';
  }
  function entities() {
    const f=field();options($('operationBoundaryEntity'),(bound()?.[f?.table]||[]).map(r=>[String(r.id),`${r.id} · ${r.name}`]));
    const series=f?.schema.type==='array';
    for(const id of ['operationOverrideFirst','operationOverrideLast']){$(id).disabled=!series;$(id).max=series?'96':'98';}
    if(!series){$('operationOverrideFirst').value=1;$('operationOverrideLast').value=98;}
    else {$('operationOverrideFirst').value=1;$('operationOverrideLast').value=96;}
    value();
  }
  function category() {
    const g=group(); if(!g)return;
    $('operationRuleDefinition').textContent=`${g.clauses}：${g.definition}`;
    $('operationRuleCoverage').textContent=`已接入下列换算后边界；覆盖限制：${g.limitation}`;
    options($('operationBoundaryField'),g.fields.map(f=>[`${f.table}.${f.field}`,`${f.schema.title} · ${f.table}`]));
    $('operationRuleFields').replaceChildren(shared.table(['边界字段','单位','当前实体','每日设置'],g.fields.map(f=>{
      const edit=el('button','设置');edit.type='button';edit.className='btn btn-sm';edit.onclick=()=>{$('operationManualMode').click();$('operationBoundaryField').value=`${f.table}.${f.field}`;entities();$('operationBoundaryField').scrollIntoView({block:'center'});};
      const scalar=f.schema.type==='array'?f.schema.items:f.schema;
      return [f.schema.title,scalar.unit||'状态 / 文本',bound()?.[f.table]?.length||0,edit];
    })));
    entities();
  }
  function load(market,catalog) {
    state.market=market;state.catalog=catalog;
    if(!catalog){$('operationRuleCoverage').textContent='服务端未提供细则边界目录，请使用重新构建的 run_gui_server。';return;}
    options($('operationRuleCategory'),catalog.items.map(g=>[g.id,`${g.title} · ${g.clauses}`]));
    $('operationRuleCatalog').replaceChildren(shared.table(['细则边界','条款','本算例实体','覆盖限制'],catalog.items.map(g=>[g.title,g.clauses,[...new Set(g.fields.map(f=>f.table))].map(t=>`${t}: ${bound()?.[t]?.length||0}`).join('；'),g.limitation])));
    category();refreshDay();
  }
  $('operationRuleCategory').onchange=category;$('operationBoundaryField').onchange=entities;$('operationBoundaryEntity').onchange=value;$('operationOverrideFirst').onchange=value;
  $('operationBoundaryAdd').onclick=()=>{
    try {
      const f=field();if(!f)throw Error('请先加载市场边界');
      const scalar=f.schema.type==='array'?f.schema.items:f.schema;
      for(const id of ['operationOverrideFirst','operationOverrideLast','operationBoundaryValue'])if(!$(id).reportValidity()||$(id).value==='')throw Error('请填写有效边界值与时段');
      const first=Number($('operationOverrideFirst').value)-1,last=Number($('operationOverrideLast').value)-1;
      if(first>last)throw Error('覆盖起点不能晚于终点');
      const reason=$('operationBoundaryReason').value.trim();if(!reason)throw Error('请填写调整依据');
      const edit={table:f.table,field:f.field,id:Number($('operationBoundaryEntity').value),first_slot:first,last_slot:last,value:scalar.type==='string'?$('operationBoundaryValue').value:Number($('operationBoundaryValue').value),reason};
      const edits=shared.day()?.boundary_overrides||[];
      if(edits.some(e=>e.table===edit.table&&e.id===edit.id&&e.field===edit.field&&e.first_slot<=last&&e.last_slot>=first))throw Error('同一字段存在重叠覆盖，请先删除原覆盖');
      shared.setEdits([...edits,edit]);refreshDay();feedback('已加入当日底稿；校验通过后用于新仿真。');
    }catch(e){feedback(e.message);}
  };
  async function preview(config,day,target=$('operationBoundaryResolved'),revision=state.market.revision) {
    const r=await fetch('/api/session/market_operation',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'preview',revision,config,day})});
    const data=await r.json();if(!r.ok)throw Error(data.error||`HTTP ${r.status}`);
    const p=data.preview,t=Number($('operationOverrideFirst').value)-1,g=group();
    target.replaceChildren(el('p',`${g.title} · ${shared.dateAt(config.start_date,day)} · 时点${t+1}`),shared.table(['实体 / 字段','覆盖与扰动后','比例校正后生效值','单位'],g.fields.flatMap(f=>(p.authored[f.table]||[]).map(row=>{
      const effective=p.effective[f.table].find(n=>n.id===row.id),v=r=>Array.isArray(r[f.field])?r[f.field][t]:r[f.field];
      return [`${f.table}:${row.id} / ${f.schema.title}`,v(row),v(effective),(f.schema.items||f.schema).unit||''];
    }))));
    if(p.lookahead) target.append(shared.table(['次日预测日期','代表点','来源时点（1–96）','统调负荷 MW'],p.lookahead.points.map(q=>[shared.dateAt(config.start_date,p.lookahead.source_day),q.kind==='peak'?'峰':'谷',q.source_slot+1,q.load_mw])));
    else if(p.forecast_only) target.append(el('p','仅预测日：前96点用于上一运行日的峰谷预安排，不计入出清统计。'));
    target.append(el('p','单值物理参数与发电/储能报价沿用运行日窗口；仅时序字段从次日峰谷时刻提取。'));
  }
  $('operationBoundaryPreview').onclick=async()=>{
    const button=$('operationBoundaryPreview');button.disabled=true;
    try {
      await preview(shared.config(),Number($('operationEditDay').value));
      feedback('当日边界校验通过。预览使用申报初始状态，实际出清按日承接。');
    }catch(e){feedback(e.message);}finally{button.disabled=false;}
  };
  window.HySimMarketBoundary={load,refreshDay,preview};
})();
