/* Deterministic fault/inflow experiments share the Southern rolling engine. */
(() => {
  'use strict';
  const marketCanvas = window.HySimMarketCanvas.forOwner("marketStudy");
  const $ = id => document.getElementById(id), shared = window.HySimMarketOperation;
  const state = { job: null, boundary: null, revision: 0, run_id: 0, running: false, busy: false, pause: false, serverBusy: false, effective: null, request: 0 };
  const el = (tag, value) => { const n = document.createElement(tag); if (value !== undefined) n.textContent = String(value); return n; };
  const fmt = shared.fmt, sum = values => values.reduce((a, b) => a + b, 0);
  const labels = { ready: '待运行', running: '运行中', completed: '已结束', failed: '失败', cancelled: '已终止', conditional: '条件性研究账本', passed: '通过', unavailable: '不可用', unsupported: '不支持', not_requested: '未执行', balance_failed: '账本不守恒' };
  const violations = {ac_generator_pmax:['机组有功上限','MW'],ac_generator_pmin:['机组有功下限','MW'],ac_generator_qmax:['机组无功上限','Mvar'],ac_generator_qmin:['机组无功下限','Mvar'],ac_thermal_from:['线路首端容量','MVA'],ac_thermal_to:['线路末端容量','MVA'],ac_active_upper:['线路有功上限','MW'],ac_active_lower:['线路有功下限','MW'],ac_voltage_upper:['电压上限','pu'],ac_voltage_lower:['电压下限','pu'],ac_section_upper:['断面上限','MW'],ac_section_lower:['断面下限','MW']};
  const violationText = v => {const [kind,id]=v.name.split('/'),[name,unit]=violations[kind]||[kind,''];return `${name} ${id}: +${fmt(v.excess)} ${unit}`;};
  const status = message => { $('studyStatus').textContent = message; };
  async function api(route = 'market_study', body) {
    return window.HySimMarketActivity.request(`/api/session/${route}`, body, { owner: 'marketStudy',
      label: body?.action === 'step' ? `${state.job.scenarios[body.scenario].name} · 第 ${body.day + 1} 日 · 已完成 ${state.job.completed_days}/${state.job.total_days} 日窗` : undefined });
  }
  const current = () => state.job?.scenarios[Number($('studyScenario').value)];
  const fresh = () => state.job?.boundary_revision === state.revision;
  function options(id, rows) {
    const old = $(id).value;
    $(id).replaceChildren(...rows.map(([value, label]) => { const o = el('option', label); o.value = value; return o; }));
    if (rows.some(([v]) => String(v) === old)) $(id).value = old;
  }
  function controls() {
    const locked = state.running || state.busy || state.serverBusy;
    $('studyConfig').disabled = locked; $('studyGenerate').disabled = locked || !state.boundary;
    const ready = state.job && fresh() && ['ready', 'running'].includes(state.job.status);
    $('studyRun').disabled = locked || !ready; $('studyPause').disabled = !state.running || state.pause;
    $('studyCancel').disabled = !ready; $('studyExport').disabled = !state.job;
    $('studyInspectPlan').disabled = !fresh() || !current()?.days.length;
  }
  function chart(id, traces, ytitle) {
    if (typeof Plotly === 'undefined') return;
    const css = getComputedStyle(document.body);
    Plotly.react($(id), traces, { height: 300, margin: { t: 20, b: 65, l: 70, r: 15 },
      paper_bgcolor: 'transparent', plot_bgcolor: 'transparent', font: { size: 11, color: css.getPropertyValue('--ink').trim() },
      xaxis: { title: '运行时段', automargin: true }, yaxis: { title: ytitle, automargin: true },
      legend: { orientation: 'h', y: -0.3 }, hovermode: 'x unified' }, { responsive: true, displaylogo: false });
  }
  const median = values => { const v = values.filter(Number.isFinite).sort((a,b) => a-b); return v.length ? (v[Math.floor((v.length-1)/2)] + v[Math.floor(v.length/2)])/2 : null; };
  function metric(day, t, name) {
    if (!day?.valid) return null;
    if (name === 'deficit') return day.periods[t].deficit_mw;
    if (name === 'overload') return day.periods[t].overload_sum_mw;
    if (name === 'price') return day.diagnostic_prices_valid ? median(day.nodes.map(n => n.lmp_per_mwh?.[t])) : null;
    const generators = day.resources.generators;
    if (name === 'hydro') return sum(generators.filter(g => g.kind === 'hydro').map(g => g.power_mw[t]));
    const renewable = generators.filter(g => ['wind','solar','renewable'].includes(g.kind));
    const available = sum(renewable.map(g => g.renewable_available_mw[t]));
    return available > 0 ? 100*sum(renewable.map(g => g.power_mw[t]))/available : null;
  }
  function comparison() {
    const s = current(); if (!s) return;
    const ref = state.job.scenarios[Number($('studyReference').value)], key = $('studyMetric').value;
    chart('studyComparisonChart', [s, ref].filter((v,i,a) => a.indexOf(v) === i).map((scenario,i) => {
      const x = [], y = [];
      for (let d=0; d<scenario.total_days; d++) for (let t=0; t<96; t++) {
        x.push(`${shared.dateAt(scenario.config.start_date,d)}T${String(Math.floor(t/4)).padStart(2,'0')}:${String(t%4*15).padStart(2,'0')}:00`);
        y.push(metric(scenario.days[d],t,key));
      }
      return { x, y, name: i ? '对照场景' : '当前场景', mode: 'lines', connectgaps: false, line: { color: i ? '#d99b32' : '#219ca6', width: 2 } };
    }), $('studyMetric').selectedOptions[0].textContent);
    const plot=$('studyComparisonChart');
    if(plot.on){plot.removeAllListeners('plotly_click');plot.on('plotly_click',e=>{const index=e.points?.[0]?.pointIndex;if(!Number.isInteger(index))return;$('studyDay').value=Math.floor(index/96);$('studySlot').value=index%96;detail().catch(e=>status(e.message));});}
  }
  async function effective(s, day, token) {
    if (!fresh()) return null;
    const key = `${state.run_id}:${s.id}:${day}:${state.revision}`;
    if (state.effective?.key === key) return state.effective.value;
    const data = await api('market_operation', { action: 'preview', revision: state.revision, config: s.config, day });
    if (token !== state.request) return null;
    state.effective = { key, value: data.preview.effective }; return data.preview.effective;
  }
  async function detail() {
    const token = ++state.request, s = current(); if (!s) return;
    const d = Number($('studyDay').value), day = s.days[d], panel = $('studyPanel').value;
    marketCanvas.operation(s,d,Number($('studySlot').value),state.revision,true);
    const target = $('studyDetail'); target.replaceChildren();
    if (typeof Plotly !== 'undefined') Plotly.purge($('studyDetailChart'));
    $('studySelection').textContent = `${s.name} · ${shared.dateAt(s.config.start_date,d)} · ${labels[s.status] || s.status}${fresh() ? '' : ' · 已失效：基准边界发生变化'}`;
    $('studyStages').replaceChildren(shared.table(['行为 / 边界', 'SCUC', 'SCED', 'LMP', '交流复核', '研究账本'], [[
      fresh() ? '已生成' : '已失效', ...['scuc','sced','lmp'].map(k => day?.stages?.[k]?.solver_status || '未执行'),
      labels[day?.analysis?.ac_audit?.status] || '未执行', labels[day?.analysis?.settlement?.status] || '未执行'
    ]]));
    if (panel === 'behavior' || panel === 'boundary') {
      target.textContent = '正在校验场景边界';
      const b = await effective(s,d,token); if (token !== state.request) return;
      target.replaceChildren(); if (!b) { target.textContent = '边界已变更，历史输入请导出试验查看'; return; }
      if (panel === 'behavior') {
        target.append(el('p','合成报价；该试验只改变发电 / 储能报价倍数，补偿报价来自已保存边界。'));
        target.append(shared.table(['类型','ID','名称','报价 元/MWh'], [
          ...b.generators.map(g => [g.kind,g.id,g.name,g.segments.map(v => `${fmt(v.quantity_mw)} MW @ ${fmt(v.price_per_mwh)}`).join('; ')]),
          ...b.storage.map(g => ['储能',g.id,g.name,`放 ${fmt(g.discharge_price)} / 充 ${fmt(g.charge_price)}`]),
          ...b.controllable_loads.map(g => ['可控负荷',g.id,g.name,`${fmt(Math.min(...g.compensation_per_mwh.slice(0,96)))} ~ ${fmt(Math.max(...g.compensation_per_mwh.slice(0,96)))}`])
        ]));
        chart('studyDetailChart', [{ x: b.generators.map(g => `${g.id}`), y: b.generators.map(g => g.segments[0].price_per_mwh), type: 'bar', name: '首段申报价' }], '首段报价 元/MWh');
      } else {
        target.append(el('p', `来水 ×${s.inflow_scale} · 发电 / 储能报价 ×${s.bid_scale} · 故障：${s.fault.name} · 第 ${s.fault.first_day+1}–${s.fault.last_day+1} 日，时段 ${s.fault.first_slot+1}–${s.fault.last_slot+1}`));
        target.append(shared.table(['边界类别','实体数'], ['areas','buses','generators','branches','sections','external_schedules','dc_links','reservoirs','storage','controllable_loads'].map(k => [k,b[k].length])));
        target.append(el('p', '下方为本日天然来水合计；同一流域上游下泄由水量递推计算，不计入天然来水。'));
        chart('studyDetailChart', [{ x: Array.from({length:96},(_,t)=>t/4), y: Array.from({length:96},(_,t)=>sum(b.reservoirs.map(h=>h.inflow_m3_s[t]))), mode:'lines', name:'天然来水合计' }], 'm³/s');
        target.append(shared.table(['实体','ID','停运时段数'], [...b.branches.map(v=>['线路',v.id,v.available.slice(0,96).filter(x=>x===0).length]),...b.generators.map(v=>['机组',v.id,v.available.slice(0,96).filter(x=>x===0).length])].filter(v=>v[2]>0)));
      }
      return;
    }
    if (!day?.valid) { target.textContent = day?.error || s.error || '该日尚无有效出清计划'; return; }
    const x = Array.from({length:96},(_,t)=>t/4);
    if (panel === 'clearing') {
      const kinds = [...new Set(day.resources.generators.map(g=>g.kind))];
      chart('studyDetailChart', kinds.map(kind=>({x,y:x.map((_,t)=>sum(day.resources.generators.filter(g=>g.kind===kind).map(g=>g.power_mw[t]))),name:kind,stackgroup:'generation',mode:'lines'})), '分电源出力 MW');
      target.append(shared.table(['缺额 MWh','富余 MWh','线路越限 MW·h','SCUC GAP','出清耗时 s'], [[fmt(day.deficit_mwh),fmt(day.surplus_mwh),fmt(day.overload_mwh),fmt(day.stages.scuc.mip_gap),fmt(day.runtime_sec)]]));
      const reference=state.job.scenarios[Number($('studyReference').value)]?.days[d];
      if(reference?.valid)target.append(shared.table(['相对同日对照','缺额变化 MWh','富余变化 MWh','越限变化 MW·h'],[['当前 − 对照',fmt(day.deficit_mwh-reference.deficit_mwh),fmt(day.surplus_mwh-reference.surplus_mwh),fmt(day.overload_mwh-reference.overload_mwh)]]));
      target.append(shared.table(['条件恢复因素','有效','缺额减少 MWh','越限减少 MW·h'], day.counterfactuals.map(c=>[c.factor,c.valid,fmt(c.reduction_deficit_mwh),fmt(c.reduction_overload_mwh)])));
      target.append(el('p', day.counterfactuals.length ? '恢复差值固定当日日初状态和次日预测，只代表条件敏感性。' : '该日未执行恢复重算；场景差异不能单独证明异常原因。'));
    } else if (panel === 'security') {
      const audit = day.analysis?.ac_audit;
      target.append(el('p', `交流复核：${labels[audit?.status] || '未执行'}。${audit?.scope || ''}`));
      const periods = audit?.periods || [];
      chart('studyDetailChart', [{x:periods.map(p=>p.period+1),y:periods.map(p=>p.converged?p.violations.length:null),name:'交流约束违反项',mode:'lines+markers'}], '违反项数');
      target.append(shared.table(['时点','收敛','网损 MW','违反项与超限量'], periods.filter(p=>!p.converged||p.violations.length).map(p=>[`${p.period+1}${p.period>=96?' (预测)':''}`,p.converged,fmt(p.losses_mw),p.violations.map(violationText).join('; ')])));
      target.append(el('p','上述交流复核与出清中的有功越限 ΔPij 属于不同校核口径；未收敛时交流违反量未知。'));
    } else {
      const ledger = day.analysis?.settlement;
      target.append(el('p', `${labels[ledger?.status] || '不可用'} · 不具备正式结算资格。${ledger?.reason || ''}`));
      if (!ledger?.periods) return;
      chart('studyDetailChart', [['load_payment_cny','负荷付款'],['generation_receipt_cny','发电收入'],['diagnostic_slack_receipt_cny','诊断松弛账户'],['line_rent_cny','线路租金']].map(([key,name])=>({x,y:ledger.periods.map(p=>p[key]),name,mode:'lines'})), '元 / 15 分钟');
      target.append(el('p', `独立账本最大残差 ${Number(ledger.max_residual_cny).toExponential(2)} 元；容差 ${fmt(ledger.tolerance_cny)} 元。可控负荷补偿单列，未分摊至用户；收入不等于利润。`));
      target.append(shared.table(['类别','ID','名称','净电量 / 削减量 MWh','收入 / 补偿 元'], ledger.accounts.map(a=>[a.table,a.id,a.name,fmt(a.energy_mwh),fmt(a.receipt_cny)])));
    }
  }
  function render() {
    const j = state.job; $('studyResults').hidden = !j; controls();
    if (!j) { options('studyScenario',[]); options('studyReference',[]); status('尚未生成故障 / 来水试验'); return; }
    options('studyScenario',j.scenarios.map(s=>[s.id,s.name])); options('studyReference',j.scenarios.map(s=>[s.id,s.name]));
    const s = current(); options('studyDay',Array.from({length:s.total_days},(_,d)=>[d,shared.dateAt(s.config.start_date,d)]));
    status(`${labels[j.status] || j.status} · 完成 ${j.completed_days}/${j.total_days} 日窗 · ${j.finished_scenarios}/${j.scenarios.length} 场景${fresh()?'':' · 边界已变更，结果已失效'}`);
    $('studySummary').replaceChildren(shared.table(['场景','状态','有效日 / 计划','缺额 MWh','越限 MW·h','错误'],j.scenarios.map(s=>[s.name,labels[s.status]||s.status,`${s.completed_days}/${s.total_days}`,s.status==='completed'?fmt(sum(s.days.map(d=>d.deficit_mwh))):'未完整',s.status==='completed'?fmt(sum(s.days.map(d=>d.overload_mwh))):'未完整',s.error||s.days.find(d=>d.error)?.error||''])));
    $('studyLimitations').replaceChildren(...j.limitations.map(v=>el('p',v))); comparison(); detail().catch(e=>status(e.message)); controls();
  }
  async function load() {
    const [task, market, operation] = await Promise.all([api(),api('southern_market'),api('market_operation')]);
    Object.assign(state,{job:task.job,revision:task.revision,run_id:task.run_id,serverBusy:task.busy,boundary:market.boundary});
    marketCanvas.setBoundary(market.boundary,market.revision);
    $('studyBoundaryName').textContent = market.boundary ? `${market.boundary.name} · ${market.boundary.buses.length} 节点 · 修订 ${task.revision} · ${market.boundary.source}` : '尚未加载市场边界';
    options('studyBranch',(market.boundary?.branches||[]).map(l=>[l.id,`${l.id} / ${l.from_bus} → ${l.to_bus}`]));
    options('studyGenerator',(market.boundary?.generators||[]).map(g=>[g.id,`${g.id} / ${g.kind} / ${g.name}`]));
    if (!state.solverInitialized) {
      shared.updateSolverCapabilities(operation.solver_capabilities);
      shared.solverEditor('study',task.job?.config.operation.solver_options || shared.solverDraft('operation'));
      state.solverInitialized=true;
      if(task.job){
        const c=task.job.config,op=c.operation,f=c.faults.find(f=>f.branch_outages.length||f.generator_outages.length)||c.faults[0];
        $('studyHorizon').value=op.horizon;$('studyStart').value=op.start_date;$('studyWater').value=c.inflow_scales.join(',');$('studyBid').value=c.bid_scales.join(',');
        $('studyAudit').checked=op.posthoc_ac_audit;$('studyExplain').checked=op.explain;
        for(const [id,key] of [['studyFirstDay','first_day'],['studyLastDay','last_day'],['studyFirstSlot','first_slot'],['studyLastSlot','last_slot']])$(id).value=f[key]+1;
        const line=c.faults.find(f=>f.branch_outages.length&&!f.generator_outages.length),unit=c.faults.find(f=>f.generator_outages.length&&!f.branch_outages.length);
        $('studyLineFault').checked=!!line;$('studyUnitFault').checked=!!unit;$('studyCombined').checked=c.faults.some(f=>f.generator_outages.length&&f.branch_outages.length);
        if(line)$('studyBranch').value=line.branch_outages[0];if(unit)$('studyGenerator').value=unit.generator_outages[0];
        $('studySetup').open=false;
      } else {
        $('studyWater').value=task.defaults.inflow_scales?.join(',')||'0.5,1,1.5';
        $('studyBid').value=task.defaults.bid_scales?.join(',')||'1';
      }
    }
    if (!$('studyStart').value) $('studyStart').value=task.defaults.operation.start_date;
    render();
  }
  const handle = fn => async () => { try { await fn(); } catch(e) { status(e.message); } finally { controls(); } };
  const bind = (id,fn) => { $(id).onclick=handle(fn); };
  bind('studyReload',load);
  bind('studyLoadCase',async()=>{
    state.busy=true;controls();
    try {await api('load_builtin',{case:'market_ieee118'});state.effective=null;await load();document.dispatchEvent(new CustomEvent('market-operation-open'));}
    finally {state.busy=false;}
  });
  const multipliers = id => { const raw=$(id).value.trim(); if(!/^\d+(?:\.\d+)?(?:\s*[,，]\s*\d+(?:\.\d+)?)*$/.test(raw))throw new Error('倍数应为逗号分隔的 0–10 数值'); return raw.split(/[,，]/).map(Number); };
  bind('studyGenerate',async()=>{
    for(const input of $('studyConfig').querySelectorAll('input'))if(!input.reportValidity())return;
    const span={first_day:Number($('studyFirstDay').value)-1,last_day:Number($('studyLastDay').value)-1,first_slot:Number($('studyFirstSlot').value)-1,last_slot:Number($('studyLastSlot').value)-1};
    const faults=[{...span,name:'正常',generator_outages:[],branch_outages:[]}];
    const line=Number($('studyBranch').value),gen=Number($('studyGenerator').value);
    if($('studyLineFault').checked)faults.push({...span,name:`线路 ${line} 停运`,generator_outages:[],branch_outages:[line]});
    if($('studyUnitFault').checked)faults.push({...span,name:`机组 ${gen} 停运`,generator_outages:[gen],branch_outages:[]});
    if($('studyCombined').checked)faults.push({...span,name:'线路与机组联合停运',generator_outages:[gen],branch_outages:[line]});
    const config={operation:{horizon:$('studyHorizon').value,start_date:$('studyStart').value,days:[],penalty_per_mwh:100000,explain:$('studyExplain').checked,posthoc_ac_audit:$('studyAudit').checked,solver_options:shared.solverOptions('study')},inflow_scales:multipliers('studyWater'),bid_scales:multipliers('studyBid'),faults};
    state.busy=true;controls();try{Object.assign(state,await api('market_study',{action:'generate',revision:state.revision,config}));state.effective=null;$('studySetup').open=false;render();}finally{state.busy=false;}
  });
  bind('studyRun',async()=>{
    state.running=true;state.pause=false;controls();
    try{while(!state.pause&&['ready','running'].includes(state.job.status)){
      const index=state.job.next_scenario,day=state.job.scenarios[index].completed_days;
      status(`正在出清 ${state.job.scenarios[index].name} · 第 ${day+1} 日（96＋2 点）`);
      Object.assign(state,await api('market_study',{action:'step',run_id:state.run_id,scenario:index,day}));
      $('studyScenario').value=index;render();
    }}finally{state.running=false;controls();}
  });
  bind('studyPause',()=>{state.pause=true;status('将在当前日窗结束后暂停');});
  bind('studyCancel',async()=>{await api('market_study',{action:'cancel',run_id:state.run_id});state.pause=true;state.job.status='cancelled';render();});
  bind('studyExport',async()=>{const data=await api('market_study?export=1');const url=URL.createObjectURL(new Blob([JSON.stringify(data.job,null,2)],{type:'application/json'}));const a=el('a');a.href=url;a.download='market-fault-inflow-study.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
  for(const id of ['studyScenario','studyReference','studyMetric'])$(id).onchange=()=>render();
  for(const id of ['studyDay','studyPanel','studySlot'])$(id).onchange=handle(detail);
  options('studySlot',Array.from({length:96},(_,t)=>[t,`${t+1} / ${String(Math.floor(t/4)).padStart(2,'0')}:${String(t%4*15).padStart(2,'0')}`]));
  $('studyHorizon').onchange=()=>{if($('studyHorizon').value==='month')$('studyStart').value=$('studyStart').value.slice(0,7)+'-01';};
  bind('studyInspectPlan',async()=>{
    const s=current();if(!s||!fresh())throw new Error('需要当前边界下的场景结果');
    App.setActiveModule('marketOperation');
    $('operationManualMode').click();shared.preview(s);shared.setStep(4);
    $('operationWeeklyPlan').scrollIntoView({block:'start',behavior:'smooth'});
  });
  const open=handle(async()=>{if(!state.running&&!state.busy)await load();});
  document.addEventListener('market-study-open',open);
})();
