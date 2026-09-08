/* Result contract: southern_execution_contract.md, Weekly Plan Results. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const el = (tag, text) => { const n = document.createElement(tag); if (text !== undefined) n.textContent = text; return n; };
  const finite = v => typeof v === 'number' && Number.isFinite(v);
  const fmt = v => finite(v) ? v.toLocaleString('zh-CN', {maximumFractionDigits: 3}) : '不可用';
  const metrics = [
    ['online','generators','开机计划','状态','hours'],
    ['power_mw','generators','功率基点','MW','mean'],
    ['primary_reserve_mw','generators','一次调频备用','MW','mean'],
    ['reserve_up_contribution_mw','generators','上备用约束贡献','MW','mean'],
    ['reserve_down_contribution_mw','generators','下备用约束贡献','MW','mean'],
    ['reserve_up_mw','areas','省级上备用','MW','min'],
    ['reserve_down_mw','areas','省级下备用','MW','min'],
    ['loading_percent','branches','线路有功负载率','%','max'],
    ['utilization_percent','generators','新能源消纳率','%','ratio'],
    ['curtailment_mw','generators','弃风弃光功率','MW','energy'],
    ['level_m','reservoirs','水库水位','m','last'],
    ['release_m3_s','reservoirs','水库下泄','m³/s','mean'],
    ['energy_mwh','storage','储能能量','MWh','last'],
    ['discharge_mw','storage','储能放电','MW','energy'],
    ['charge_mw','storage','储能充电（负值）','MW','energy'],
    ['reduction_mw','controllable_loads','可控负荷削减','MW','energy']
  ];
  const kinds = {hydro:'水电',thermal:'火电',wind:'风电',solar:'光伏',renewable:'新能源',pumped_hydro:'抽蓄',vpp:'虚拟电厂'};
  const state = {report:null, boundary:null, revision:null, page:0, rows:[], days:[], selected:null, day:0, slot:0};
  const pageSize = 40;
  const metric = () => metrics.find(m => m[0] === $('operationPlanMetric').value) || metrics[0];
  const date = d => { const v = new Date(`${state.report?.config?.start_date}T00:00:00Z`); if (!Number.isFinite(v.getTime())) return '待设置'; v.setUTCDate(v.getUTCDate()+d); return v.toISOString().slice(0,10); };
  const time = t => `${String(Math.floor(t/4)).padStart(2,'0')}:${String(t%4*15).padStart(2,'0')}`;
  const rowsAt = (day, table) => table === 'branches' ? day?.lines : day?.resources?.[table];
  const valid = day => day?.valid && state.report?.boundary_revision === state.revision && state.report?.status !== 'stale';
  const value = (row, d, t, field = metric()[0]) => valid(state.days[d]) ? row.byDay[d]?.[field]?.[t] ?? null : null;
  const summary = (row, d) => {
    const values = Array.from({length:96},(_,t)=>value(row,d,t));
    const available = values.filter(finite), mode = metric()[4];
    if (!available.length) return null;
    if (mode === 'ratio') {
      const energy = field => Array.from({length:96},(_,t)=>value(row,d,t,field)).reduce((s,v)=>s+(finite(v)?v:0),0);
      const supply = energy('renewable_available_mw');
      return supply > 0 ? 100*energy('power_mw')/supply : null;
    }
    if (mode === 'hours') return available.filter(v=>v>.5).length*.25;
    if (mode === 'energy') return available.reduce((s,v)=>s+v,0)*.25;
    if (mode === 'last') return values[95];
    if (mode === 'max') return Math.max(...available);
    if (mode === 'min') return Math.min(...available);
    return available.reduce((s,v)=>s+v,0)/available.length;
  };
  const summaryLabel = () => ({hours:'开机 h',mean:`均值 ${metric()[3]}`,min:`最小 ${metric()[3]}`,max:`最大 ${metric()[3]}`,last:`日末 ${metric()[3]}`,ratio:'电量消纳率 %',energy:'电量 MWh'})[metric()[4]];
  const periodStatus = d => !state.days[d] ? '待出清' : !valid(state.days[d]) ? '结果无效' : '有效';
  function layout(title, height, unit) {
    const css = getComputedStyle(document.body);
    return {title:{text:title,font:{size:14}},height,autosize:true,paper_bgcolor:css.getPropertyValue('--bg2').trim(),plot_bgcolor:css.getPropertyValue('--bg2').trim(),font:{color:css.getPropertyValue('--ink').trim()},margin:{t:45,l:65,r:24,b:65},xaxis:{type:'date'},yaxis:{title:{text:unit}},legend:{orientation:'h',y:-.3}};
  }
  function overview() {
    const ids=['Generation','Commitment','Reserve','Loading','Renewable','Water'];
    const days=Array.from({length:state.report?.config?.start_date ? state.report.total_days : 0},(_,d)=>state.report.days.find(v=>v.day===d));
    const ready=days.some(valid);$('operationPlanOverview').hidden=!ready;
    $('operationPlanEnergySummary').textContent='';
    if(typeof Plotly==='undefined')return;
    if(!ready){for(const id of ids)Plotly.purge($('operationPlan'+id));return;}
    const x=Array.from({length:days.length*96},(_,i)=>`${date(Math.floor(i/96))}T${time(i%96)}:00`);
    const points=fn=>x.map((_,i)=>{const day=days[Math.floor(i/96)];return valid(day)?fn(day,i%96):null;});
    const sum=(rows,field,t)=>rows?.length && rows.every(r=>finite(r[field]?.[t])) ? rows.reduce((s,r)=>s+r[field][t],0) : null;
    const line=(name,y,color,extra={})=>({x,y,name,type:'scatter',mode:'lines',connectgaps:false,line:{color,width:1.5},...extra});
    const chart=(suffix,title,unit,traces,extra={})=>{
      const target=$('operationPlan'+suffix),config={...layout(title,280,unit),...extra};
      config.margin={t:45,l:52,r:48,b:80};config.legend={orientation:'h',y:-.35,font:{size:10}};
      if(!traces.some(trace=>trace.y.some(finite)))config.annotations=[{text:'尚无有效数据',xref:'paper',yref:'paper',x:.5,y:.5,showarrow:false}];
      Plotly.react(target,traces,config,{responsive:true,displaylogo:false});
      target.removeAllListeners?.('plotly_click');
      target.on('plotly_click',event=>{const p=event.points?.[0],i=p?.pointIndex;if(!Number.isInteger(i))return;state.navigate?.(Math.floor(i/96),i%96);if(p.data.meta?.target)window.HySimMarketCanvas.select(p.data.meta.target);});
    };
    const colors={hydro:'#329cbd',thermal:'#d17c51',wind:'#39ad85',solar:'#d3b642',renewable:'#8da84c',pumped_hydro:'#b58cad',vpp:'#b4bdc6',unknown:'#a0a0a0'};
    const fleetKinds=[...new Set(days.filter(valid).flatMap(d=>(d.resources?.generators||[]).map(g=>g.kind||'unknown')))];
    const fleet=field=>fleetKinds.map(kind=>line(kinds[kind]||kind,points((d,t)=>{
      const rows=d.resources?.generators;if(!rows)return null;
      return rows.filter(g=>(g.kind||'unknown')===kind).reduce((s,g)=>s+(field==='online'?(g.online[t]>.5?1:0):g[field][t]),0);
    }),colors[kind]||'#a0a0a0',field==='power_mw'?{stackgroup:'generation'}:{}));
    chart('Generation','各类电源功率基点','MW',[...fleet('power_mw'),line('负荷',points((d,t)=>d.periods?.[t]?.load_mw??null),'#d5d8df'),line('储能净出力',points((d,t)=>d.resources?.storage?.reduce((s,r)=>s+r.discharge_mw[t]+r.charge_mw[t],0)??null),'#a884bd')]);
    chart('Commitment','各类型开机数量','台',fleet('online'));
    chart('Reserve','省级备用合计','MW',[
      line('上备用',points((d,t)=>sum(d.resources?.areas,'reserve_up_mw',t)),'#34a5be'),
      line('上备用需求',points((d,t)=>sum(d.resources?.areas,'reserve_up_required_mw',t)),'#d4bd53'),
      line('下备用',points((d,t)=>sum(d.resources?.areas,'reserve_down_mw',t)),'#52af88'),
      line('下备用需求',points((d,t)=>sum(d.resources?.areas,'reserve_down_required_mw',t)),'#d87961')]);
    const rates=(d,t)=>(d.lines||[]).map(l=>l.loading_percent?.[t]).filter(finite);
    chart('Loading','线路有功负载率与越限数量','%',[
      line('最大负载率',points((d,t)=>{const v=rates(d,t);return v.length?Math.max(...v):null;}),'#d67655'),
      line('100% 限额',points(()=>100),'#aeb4bd'),
      line('越限线路',points((d,t)=>d.lines?.filter(l=>l.overload_mw[t]>1e-6).length??null),'#58a7c8',{yaxis:'y2'})
    ],{yaxis2:{title:{text:'条'},overlaying:'y',side:'right',rangemode:'tozero'}});
    const renewable=(d,t,field)=>{const rows=d.resources?.generators?.filter(g=>['wind','solar','renewable'].includes(g.kind));return rows?.length?sum(rows,field,t):null;};
    const available=points((d,t)=>renewable(d,t,'renewable_available_mw')), used=points((d,t)=>renewable(d,t,'power_mw'));
    const validIndices=available.map((v,i)=>finite(v)&&finite(used[i])?i:null).filter(i=>i!==null);
    const availabilityEnergy=validIndices.reduce((s,i)=>s+.25*available[i],0),usedEnergy=validIndices.reduce((s,i)=>s+.25*used[i],0);
    $('operationPlanEnergySummary').textContent=validIndices.length?`有效时段新能源：可用 ${fmt(availabilityEnergy)} MWh · 消纳 ${fmt(usedEnergy)} MWh · 消纳率 ${fmt(availabilityEnergy>0?100*usedEnergy/availabilityEnergy:null)}%`:'新能源统计不可用';
    chart('Renewable','新能源可用与消纳','MW',[
      line('可用功率',available,'#c5b55d'),line('消纳功率',used,'#43b28d'),
      line('消纳率',available.map((v,i)=>finite(v)&&v>0&&finite(used[i])?100*used[i]/v:null),'#70acd1',{yaxis:'y2'})
    ],{yaxis2:{title:{text:'%'},overlaying:'y',side:'right',rangemode:'tozero'}});
    const reservoirs=new Map();for(const d of days.filter(valid))for(const r of d.resources?.reservoirs||[])reservoirs.set(r.id,r.name);
    chart('Water','水库水位','m',[...reservoirs].map(([id,name],i)=>line(name,points((d,t)=>d.resources?.reservoirs?.find(r=>r.id===id)?.level_m?.[t]??null),Object.values(colors)[i%8],{meta:{target:`reservoirs:${id}`}})),{showlegend:reservoirs.size<=4});
  }
  function choose(id, d = state.day, t = state.slot) {
    state.selected = String(id); $('operationPlanEntity').value = String(id);
    state.navigate?.(d,t);
    if (metric()[1] !== 'areas') window.HySimMarketCanvas.select(`${metric()[1]}:${id}`);
    series();
  }
  function series() {
    const target = $('operationPlanSeries'), row = state.rows.find(r=>String(r.id)===state.selected);
    target.hidden = !row;
    if (!row || typeof Plotly === 'undefined') { if (typeof Plotly !== 'undefined') Plotly.purge(target); return; }
    const [field,table,label,unit] = metric(), x = Array.from({length:state.days.length*96},(_,i)=>`${date(Math.floor(i/96))}T${time(i%96)}:00`);
    let fields = [[field,label]];
    if (table === 'areas') fields = [[field,label],[field.replace('_mw','_required_mw'),'需求'],[field.replace('_mw','_margin_mw'),'裕度']];
    const traces = fields.map(([f,name])=>({x,y:x.map((_,i)=>value(row,Math.floor(i/96),i%96,f)),name,type:'scatter',mode:'lines',connectgaps:false,line:{shape:field==='online'?'hv':'linear'}}));
    const titleName=String(row.name).length>22?String(row.name).slice(0,22)+'…':String(row.name);
    const titleText=titleName.replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;');
    const config = layout(`${titleText}<br>${label}`,290,unit);config.margin.t=62;
    if (field === 'online') config.yaxis = {tickvals:[0,1],ticktext:['停机','开机'],range:[-.1,1.1]};
    Plotly.react(target,traces,config,{responsive:true,displaylogo:false});
    target.removeAllListeners?.('plotly_click');
    target.on('plotly_click',event=>{const i=event.points?.[0]?.pointIndex;if(Number.isInteger(i))choose(row.id,Math.floor(i/96),i%96);});
  }
  function table() {
    const root = $('operationPlanTable');root.replaceChildren();
    const tab = el('table'), head = el('thead'), tr = el('tr');
    const current = state.report ? `${date(state.day)} ${time(state.slot)} · ${metric()[3]}` : '当前时段';
    for (const name of ['设备 / 类型', current, ...state.days.map((_,d)=>`${date(d)} · ${summaryLabel()}`)])tr.append(el('th',name));
    head.append(tr);tab.append(head);const body=el('tbody');
    for (const row of state.rows.slice(state.page*pageSize,(state.page+1)*pageSize)) {
      const line=el('tr'), td=el('td'), button=el('button',`${row.id} · ${row.name}${row.kind ? ' / '+(kinds[row.kind]||row.kind) : ''}`);
      button.type='button';button.className='operation-plan-device';button.dataset.planTarget=`${metric()[1]}:${row.id}`;button.onclick=()=>choose(row.id);td.append(button);line.append(td);
      line.append(el('td',fmt(value(row,state.day,state.slot))));
      state.days.forEach((_,d)=>{const cell=el('td'), b=el('button',periodStatus(d)==='有效'?fmt(summary(row,d)):periodStatus(d));b.type='button';b.title=`${date(d)} · ${summaryLabel()}`;b.onclick=()=>choose(row.id,d,0);cell.append(b);line.append(cell);});
      body.append(line);
    }
    tab.append(body);root.append(tab);
  }
  function render() {
    const report = state.report, [field,tableName,label,unit] = metric();
    state.days = Array.from({length:report?.config?.start_date ? report.total_days : 0},(_,d)=>report.days?.find(day=>day.day===d));
    const map = new Map();
    const add = (source,d) => { for(const r of source||[]) { if(!map.has(r.id))map.set(r.id,{id:r.id,name:r.name||String(r.id),kind:r.kind,byDay:[]});const row=map.get(r.id);if(r.kind)row.kind=r.kind;if(d!=null)row.byDay[d]=r; } };
    if(state.days.length && report.boundary_revision === state.revision)add(state.boundary?.[tableName]);
    state.days.forEach((day,d)=>add(rowsAt(day,tableName),d));
    const needle = $('operationPlanSearch').value.trim().toLowerCase(), kind=$('operationPlanKind').value;
    $('operationPlanKind').disabled=tableName!=='generators';
    state.rows=[...map.values()].filter(r=>(tableName!=='generators'||!kind||r.kind===kind)&&(!['utilization_percent','curtailment_mw'].includes(field)||['wind','solar','renewable'].includes(r.kind))&&`${r.id} ${r.name}`.toLowerCase().includes(needle)).sort((a,b)=>a.id-b.id);
    state.page=Math.max(0,Math.min(state.page,Math.ceil(state.rows.length/pageSize)-1));
    $('operationPlanPage').textContent=`${state.rows.length ? state.page*pageSize+1 : 0}–${Math.min(state.rows.length,(state.page+1)*pageSize)} / ${state.rows.length} 个设备`;
    $('operationPlanPrev').disabled=state.page===0;$('operationPlanNext').disabled=(state.page+1)*pageSize>=state.rows.length;
    $('operationPlanExport').disabled=!state.rows.length;
    const completed=state.days.filter(valid).length;
    $('operationPlanStatus').textContent=report ? `${date(0)} 至 ${date(state.days.length-1)} · ${completed}/${state.days.length} 天有效计划 · ${state.days.length*96} 个时段${report.boundary_revision!==state.revision?' · 边界已变化，结果无效':''}` : '尚未建立运行任务';
    $('operationPlanScope').textContent=field.includes('reserve') ? '一次调频备用为 SCED 决策值；上、下备用为规则约束中的有符号容量贡献，允许负值，并非逐机中标量。省级合计包含储能净出力项。' : field==='loading_percent' ? '负载率采用潮流方向对应的有功限额；停运、零限额或不跨零区间显示不可用，非交流 MVA 负载率。' : ['utilization_percent','curtailment_mw'].includes(field) ? '新能源消纳按有效边界的可用电量统计；无可用电量时消纳率不可用。' : '功率基点为 SCED 出力。15 分钟计划；每天 96+2 点滚动求解，展示当日 96 点。待出清、失败及过期结果不计入统计。';
    const picker=$('operationPlanEntity');picker.replaceChildren();
    for(const row of state.rows){const option=el('option',`${row.id} · ${row.name}`);option.value=row.id;picker.append(option);}
    if(!state.rows.some(r=>String(r.id)===state.selected))state.selected=state.rows.length?String(state.rows[0].id):null;
    picker.value=state.selected||'';picker.disabled=!state.rows.length;
    table();series();
    const target=$('operationPlanHeatmap');target.hidden=!state.rows.length;
    if(typeof Plotly==='undefined')return;
    if(!state.rows.length){Plotly.purge(target);return;}
    const shown=state.rows.slice(state.page*pageSize,(state.page+1)*pageSize), x=Array.from({length:state.days.length*96},(_,i)=>`${date(Math.floor(i/96))}T${time(i%96)}:00`);
    const z=shown.map(r=>x.map((_,i)=>value(r,Math.floor(i/96),i%96)));
    const config=layout(`${label} · 全时段`,Math.max(280,120+shown.length*18),'设备 ID');
    target.style.height=`${config.height}px`;
    config.yaxis={type:'category',autorange:'reversed',title:{text:'设备 ID'}};
    const trace={type:'heatmap',x,y:shown.map(r=>String(r.id)),z,hoverongaps:false,colorscale:field==='online'?[[0,'#49515b'],[1,'#38ad8e']]:'Viridis',colorbar:{title:{text:unit},thickness:10},hovertemplate:`%{x}<br>ID %{y}<br>${label} %{z:.3f} ${unit}<extra></extra>`};
    if(field==='online'){trace.zmin=0;trace.zmax=1;trace.colorbar.tickvals=[0,1];trace.colorbar.ticktext=['停','开'];}
    Plotly.react(target,[trace],config,{responsive:true,displaylogo:false});
    target.removeAllListeners?.('plotly_click');
    target.on('plotly_click',event=>{const p=event.points?.[0];if(!p||!Array.isArray(p.pointNumber))return;const [r,i]=p.pointNumber;choose(shown[r].id,Math.floor(i/96),i%96);});
  }
  function exportCsv() {
    const [field,tableName,,unit]=metric();
    const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
    const lines=[['component_kind','id','name','date','slot','time','status','metric','unit','value'].map(quote).join(',')];
    for(const row of state.rows)for(let d=0;d<state.days.length;d++)for(let t=0;t<96;t++)lines.push([tableName,row.id,/^[=+@-]/.test(row.name)?"'"+row.name:row.name,date(d),t,time(t),periodStatus(d),field,unit,value(row,d,t)].map(quote).join(','));
    const url=URL.createObjectURL(new Blob(['\uFEFF'+lines.join('\r\n')],{type:'text/csv;charset=utf-8'})),a=el('a');a.href=url;a.download=`market-plan-${field}-${state.report.config.start_date}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  }
  for(const [field,,label,unit] of metrics){const option=el('option',`${label} · ${unit}`);option.value=field;$('operationPlanMetric').append(option);}
  for(const id of ['operationPlanMetric','operationPlanKind','operationPlanSearch'])$(id).addEventListener(id==='operationPlanSearch'?'input':'change',()=>{state.page=0;render();});
  $('operationPlanPrev').onclick=()=>{state.page--;render();};$('operationPlanNext').onclick=()=>{state.page++;render();};
  $('operationPlanEntity').onchange=()=>choose($('operationPlanEntity').value);
  $('operationPlanExport').onclick=exportCsv;
  window.HySimMarketWeeklyPlan={
    boundary(b){state.boundary=b;},
    update(report,revision,navigate){
      state.navigate=navigate;
      if(state.report===report && state.revision===revision)return;
      state.report=report;state.revision=revision;
      window.HySimMarketActivity.whenVisible('operationWeeklyPlan',()=>{overview();render();});
    },
    focus(day,slot){state.day=day;state.slot=slot;table();}
  };
})();
