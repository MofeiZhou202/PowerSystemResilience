/* Contract: docs/modules/market/southern_execution_contract.md, rolling operation. */
(() => {
  'use strict';
  const marketCanvas = window.HySimMarketCanvas.forOwner("marketOperation");
  const $ = id => document.getElementById(id);
  const fields = { load_scale: '负荷倍数', wind_scale: '风电预测倍数', solar_scale: '光伏预测倍数', inflow_scale: '来水倍数', bid_scale: '全市场报价倍数', generator_bid_scale: '发电 / 储能报价倍数', load_bid_scale: '可控负荷补偿倍数', line_limit_scale: '线路限额倍数' };
  const names = { ...fields, generator_outages: '机组停运', branch_outages: '线路停运', boundary_overrides: '设备级边界联合恢复' };
  const state = { revision: 0, run_id: 0, job: null, days: [], running: false, pause: false, busy: false, serverBusy: false, dirty: false };
  const el = (tag, value) => { const n = document.createElement(tag); if (value !== undefined) n.textContent = String(value); return n; };
  const fmt = value => typeof value === 'number' && Number.isFinite(value) ? value.toLocaleString('zh-CN', { maximumFractionDigits: 4 }) : '不可用';
  const quality = s => ({ optimal_within_tolerance: '达到最优性容差', feasible_limit: '限时/限额可行解', feasible_unproven: '可行，未证最优', proven_infeasible: '已证不可行', unbounded: '无界', infeasible_or_unbounded: '不可行或无界', limit_without_verified_solution: '限额内无已验证解', solver_or_audit_failure: '求解/残差复核失败' }[s?.solution_quality] || '未记录求解质量');
  const time = t => `${String(Math.floor(t / 4)).padStart(2, '0')}:${String(t % 4 * 15).padStart(2, '0')}`;
  const status = value => { $('operationStatus').textContent = value; if ($('marketForecastWorkspace').hidden) marketCanvas.progress(value); };
  const dateAt = (start, d) => { const value = new Date(`${start}T00:00:00Z`); value.setUTCDate(value.getUTCDate() + d); return value.toISOString().slice(0, 10); };
  const defaultDay = () => ({ ...Object.fromEntries(Object.keys(fields).map(k => [k, 1])), first_slot: 0, last_slot: 95, generator_outages: [], branch_outages: [] });
  let recoveryPolicies = false;
  const recoveryOptions = prefix => ({ explain: $(`${prefix}Explain`).checked,
    ...(recoveryPolicies ? {explain_trigger: $(`${prefix}ExplainTrigger`).value, recovery_pricing: $(`${prefix}RecoveryPricing`).value} : {}) });
  function updateRecoveryPolicies(available) {
    recoveryPolicies=available===true;
    for(const prefix of ['operation','forecast'])for(const field of ['ExplainTrigger','RecoveryPricing'])
      $(`${prefix}${field}`).closest('label').hidden=!recoveryPolicies;
  }
  function recoveryEditor(prefix, config) {
    $(`${prefix}Explain`).checked = config.explain;
    $(`${prefix}ExplainTrigger`).value = config.explain_trigger || 'always';
    $(`${prefix}RecoveryPricing`).value = config.recovery_pricing || 'dispatch_only';
  }
  let solverCapabilities = [];
  let solverCapabilitiesRequest = null;
  const solverEditors = new Map();
  function populateSolverSelect(prefix) {
    const select=$(`${prefix}Solver`);if(!select)return;
    const wanted=select.value||select.dataset.requestedSolver||'highs';
    select.replaceChildren();
    for(const cap of solverCapabilities) {
      const option=el('option',`${cap.label}${cap.available===false?'（不可用）':''}`);
      option.value=cap.id;option.disabled=cap.available===false;option.title=cap.reason||'';select.append(option);
    }
    if(!solverCapabilities.length) {
      const option=el('option','正在加载求解器');option.value='';select.append(option);select.disabled=true;
    } else {
      if(!solverCapabilities.some(c=>c.id===wanted)) {
        const option=el('option',`${wanted}（当前服务不可用）`);option.value=wanted;option.disabled=true;select.append(option);
      }
      select.value=wanted;select.disabled=false;
    }
    solverEditors.get(prefix)?.();
  }
  function updateSolverCapabilities(capabilities) {
    solverCapabilities=capabilities;
    for(const prefix of solverEditors.keys())populateSolverSelect(prefix);
  }
  function ensureSolverCapabilities() {
    if(solverCapabilities.length)return Promise.resolve();
    if(!solverCapabilitiesRequest)solverCapabilitiesRequest=api().then(data=>{
      if(!data.solver_capabilities?.length)throw new Error('服务端未返回求解器列表，请重载任务');
      updateSolverCapabilities(data.solver_capabilities);
      updateRecoveryPolicies(data.recovery_policies);
    }).finally(()=>{solverCapabilitiesRequest=null;});
    return solverCapabilitiesRequest;
  }
  let boundaryLoaded = false;
  let workflowRestored = false;
  function setStep(step) {
    if (step > 1 && !boundaryLoaded) { $('operationWorkflowStatus').textContent='请先加载市场算例';return; }
    $('marketOperationWorkspace').dataset.workflowStep=String(step);
    sessionStorage.setItem('hysim.marketOperationStep',String(step));
    document.querySelectorAll('[data-operation-step]').forEach(n=>{if(Number(n.dataset.operationStep)===step)n.setAttribute('aria-current','step');else n.removeAttribute('aria-current');});
    $('operationPreviousStep').disabled=step===1;
    $('operationNextStep').hidden=step>=3;
    $('operationNextStep').textContent=step===1?'下一步：设置市场边界':'确认边界并继续';
    $('operationWorkflowStatus').textContent='';
    const forecast=!$('marketForecastWorkspace').hidden;
    const prefix=forecast?'forecast':'operation';
    const [year, month] = $('operationStartDate').value.split('-').map(Number);
    const days = forecast ? 7 : $('operationHorizon').value === 'month' ? new Date(Date.UTC(year, month, 0)).getUTCDate() : $('operationHorizon').value === 'week' ? 7 : 1;
    const scenarios = forecast ? Number($('forecastCount').value) : 1;
    const causeMode = !$(`${prefix}Explain`).checked ? (recoveryPolicies?'仅手动补算':'未启用恢复重算') : !recoveryPolicies ? '每日恢复重算' : ({anomaly:'异常时恢复重算',always:'每日恢复重算',manual:'仅手动补算'})[$(`${prefix}ExplainTrigger`).value];
    $('operationRunSummary').textContent=`${scenarios} 场景 × ${days} 日 = ${scenarios * days} 个日窗 · 每日 96 + 2 点 · ${$(`${prefix}Solver`)?.selectedOptions[0]?.textContent||'待配置'} · GAP ${$(`${prefix}MipGap`)?.value||'0.01'} · ${causeMode}`;
    if(step===4) {
      if(!(state.preview||state.job)?.days?.length)$('operationWorkflowStatus').textContent='尚无当日出清结果';
      window.dispatchEvent(new Event('resize'));
    }
  }
  const rootCutProfiles = () => solverCapabilities.find(c=>c.id==='native')?.root_cut_profiles;
  const solverDraft = prefix => ({solver:$(`${prefix}Solver`)?.value||$(`${prefix}Solver`)?.dataset.requestedSolver||'highs',time_limit_sec:Number($(`${prefix}TimeLimit`)?.value??120),mip_gap:Number($(`${prefix}MipGap`)?.value??.01),threads:Number($(`${prefix}Threads`)?.value??0),...(rootCutProfiles()?{native_root_cuts:$(`${prefix}RootCuts`)?.value||'default'}:{})});
  const solverOptions = prefix => {
    const selected=$(`${prefix}Solver`),cap=solverCapabilities.find(c=>c.id===selected?.value);
    if(!cap || cap.available===false || selected.disabled)throw new Error('求解器尚未加载或不可用，请重载任务并重新选择');
    for(const name of ['TimeLimit','MipGap','Threads'])if(!$(`${prefix}${name}`).checkValidity())throw new Error('请修正高级求解设置中的数值');
    return solverDraft(prefix);
  };
  function solverEditor(prefix, value = {}) {
    const target = $(`${prefix}SolverOptions`); target.replaceChildren();
    const select = el('select');select.id=`${prefix}Solver`;select.required=true;select.dataset.requestedSolver=value.solver||'highs';
    const label=el('label','求解器');label.append(select);target.append(label);
    for(const [id,title,v,min,max,step] of [['TimeLimit','每次求解时限（秒）',value.time_limit_sec??120,.1,3600,'any'],['MipGap','相对 MIP gap',value.mip_gap??.01,0,.1,'any'],['Threads','Gurobi线程数（0自动）',value.threads??0,0,128,'1']]) {
      const label=el('label',title),input=el('input');Object.assign(input,{id:`${prefix}${id}`,type:'number',min:String(min),max:String(max),step,required:true,value:String(v)});label.append(input);target.append(label);
    }
    const cutLabel=el('label','Native 根节点割策略'),cuts=el('select');cuts.id=`${prefix}RootCuts`;
    for(const [id,title] of [['default','默认自适应'],['enhanced','增强分离（实验）']]) { const option=el('option',title);option.value=id;cuts.append(option); }
    cuts.value=value.native_root_cuts||'default';cutLabel.append(cuts);target.append(cutLabel);
    const threads=()=>{ const solver=select.value||select.dataset.requestedSolver;$(`${prefix}Threads`).disabled=solver!=='gurobi';if(solver!=='gurobi')$(`${prefix}Threads`).value=0;cutLabel.hidden=!rootCutProfiles();cuts.disabled=solver!=='native'||!rootCutProfiles();if(solver!=='native')cuts.value='default'; };
    select.onchange=()=>{select.dataset.requestedSolver=select.value;threads();};solverEditors.set(prefix,threads);populateSolverSelect(prefix);
    ensureSolverCapabilities().catch(e=>{$('operationWorkflowStatus').textContent=e.message;});
    const scope=el('span','Gurobi：MILP/LP均限时；HiGHS：MILP限时；Native：原生整数搜索，HiGHS连续求解，单次LP可能超时。时限不含建模和恢复实验总耗时。');target.append(scope);
  }
  async function api(body, path = '/api/session/market_operation') {
    return window.HySimMarketActivity.request(path, body, { owner: 'marketOperation',
      label: body?.action === 'step' ? `第 ${body.day + 1}/${state.job.total_days} 日 · 已完成 ${state.job.completed_days} 日` : undefined });
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
    const locked = state.running || state.busy || state.serverBusy || state.previewExplanation?.locked;
    $('operationNextStep').disabled=locked;
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
    const count = $('operationHorizon').value === 'day' ? 1 : $('operationHorizon').value === 'week' ? 7 : new Date(Date.UTC(year, month, 0)).getUTCDate();
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
    window.HySimMarketWeeklyPlan.focus(Number($('operationResultDay').value),slot);
    marketCanvas.operation(report, Number($('operationResultDay').value), slot, state.revision);
    for (const id of ['operationCauses', 'operationNodes', 'operationLines']) $(id).replaceChildren();
    if (!d) return;
    const root = $('operationCauses');
    const stageRows = ['scuc','sced','lmp'].filter(name=>d.stages?.[name]).map(name=>[name,d.stages[name]]);
    const gap = value => value != null && Number(value) > 0 && Number(value) < 0.0001 ? Number(value).toExponential(3) : fmt(value);
    root.append(table(['求解阶段 / 后端', '质量', '原始状态', 'MIP gap', '二进制变量数', '耗时 s'], stageRows.map(([name,s])=>[`${name.toUpperCase()} / ${s.solver||'未知'}`, quality(s),s.solver_status,gap(s.mip_gap),fmt(s.binary_variables),fmt(s.runtime_sec)])));
    const performance = table(['阶段 / 建模形式', 'LP算法', '变量数', '非零系数数', '启动分类消元机组数', '建模 s', '复核及结果生成 s', '恢复约束残差'], stageRows.map(([name,s])=>[`${name.toUpperCase()} / ${s.formulation === 'compact' ? '等价紧凑式' : s.formulation === 'reference' ? '原式' : '未知'}`,({barrier:'障碍法',dual_simplex:'对偶单纯形',solver_default:'求解器默认'})[s.lp_algorithm] || '未知',fmt(s.variables),fmt(s.nonzeros),fmt(s.compact_units),fmt(s.assembly_sec),fmt(s.audit_sec),s.reconstructed_max_residual == null ? '不可用' : Number(s.reconstructed_max_residual).toExponential(2)]));
    performance.dataset.testid = 'market-stage-performance';
    root.append(performance);
    const timingTable = table(['阶段', '环境初始化 s', '模型导入 s', '优化（含预处理）s', '提取与释放 s'],
      stageRows.map(([name,s])=>[name.toUpperCase(),...['environment_sec','model_import_sec','optimize_sec','result_extract_sec'].map(k=>fmt(s.solver_timing?.[k]))]));
    timingTable.dataset.testid='market-solver-timing';root.append(timingTable);
    const certificates=stageRows.filter(([,s])=>s.gap_certificate);
    if(certificates.length) {
      const certificateTable=table(['阶段','整数方案','下界 CNY','相对差距','松弛 s','修复 s','证书核验 s'],certificates.map(([name,s])=>{
        const c=s.gap_certificate;
        return [name.toUpperCase(),c.accepted?(c.reused_from?'复用已核验方案':'通过核验'):'原模型求解',
          fmt(c.bound_cny),c.gap==null?'不可用':`${(100*c.gap).toFixed(4)}%`,
          c.reused_from?'复用':fmt(c.relaxation_sec),c.reused_from?'复用':fmt(c.repair_sec),c.reused_from?'复用':fmt(c.certificate_sec)];
      }));
      certificateTable.dataset.testid='market-gap-certificate';root.append(certificateTable);
    }
    const priceCheck=d.stages?.lmp?.price_consistency;
    if(d.stages?.lmp) {
      const checkTable=table(['价格一致性','定价算法','完整行对偶数','最大对偶差','复算耗时 s'],[[
        priceCheck?.passed===true?'通过：精确一致':priceCheck?.passed===false?`失败：${priceCheck.status}`:'未检查（历史结果）',
        priceCheck?`${priceCheck.algorithm==='barrier'?'障碍法':'对偶单纯形'} / ${priceCheck.threads} 线程`:'不可用',
        fmt(priceCheck?.dual_rows),fmt(priceCheck?.max_dual_difference),fmt(priceCheck?.repeat_wall_sec)]]);
      checkTable.dataset.testid='market-price-consistency';root.append(checkTable);
    }
    const nativeRows=stageRows.filter(([,s])=>s.native_diagnostics);
    if(nativeRows.length) {
      const diagnostic=table(['Native 阶段 / 割策略','请求轮数 / 每轮条数','根行数 / 分离上限','已加入割（全搜索）','搜索节点','LP 次数','整数解更新次数','当前下界 CNY','当前整数解 CNY'],nativeRows.map(([name,s])=>{
        const n=s.native_diagnostics,number=v=>v==null?'不可用':fmt(v);
        return [name.toUpperCase()+' / '+n.profile,`${n.requested_root_rounds} / ${n.requested_cuts_per_round}`,`${number(n.root_presolved_rows)} / ${number(n.root_row_admission_limit)}`,number(n.cuts_added),number(n.nodes_explored),number(n.lp_solves),number(n.incumbent_updates),number(n.best_bound_cny),number(n.best_incumbent_cny)];
      }));
      diagnostic.dataset.testid='market-native-cuts';root.append(diagnostic);
    }
    if(d.stages?.scuc?.model_size) {
      const size=d.stages.scuc.model_size, audit=el('details');audit.dataset.testid='market-model-size';
      audit.append(el('summary','SCUC 变量与约束构成'));
      audit.append(el('p',`求解器预处理前：${fmt(size.variables)} 个变量，${fmt(size.binary_variables)} 个二进制变量；${fmt(size.equalities+size.inequalities)} 条约束。边界可证明冗余 ${fmt(size.bound_redundant_inequalities)} 条，本次审计删除 ${fmt(size.constraints_removed)} 条。`));
      if(size.submitted_inequalities!=null) audit.append(el('p',`实际提交：${fmt(size.submitted_equalities+size.submitted_inequalities)} 条约束，${fmt(size.submitted_nonzeros)} 个非零系数；原约束保留复核。`));
      const kinds={thermal:'火电',hydro:'水电',pumped_hydro:'抽蓄',wind:'风电',solar:'光伏',renewable:'新能源',vpp:'虚拟电厂'};
      audit.append(table(['类型','机组数','开机状态二进制变量','固定开机状态变量'],Object.entries(size.commitment_by_kind).map(([name,c])=>[kinds[name]||name,fmt(c.units),fmt(c.binary_variables),fmt(c.fixed_variables)])));
      audit.append(table(['变量族','变量数','二进制变量','固定变量'],Object.entries(size.variable_families).sort((a,b)=>b[1].variables-a[1].variables).map(([name,c])=>[name,fmt(c.variables),fmt(c.binary_variables),fmt(c.fixed_variables)])));
      audit.append(table(['规则 / 约束族','等式','不等式','非零系数','边界可证明冗余'],Object.entries(size.constraint_families).sort((a,b)=>b[1].equalities+b[1].inequalities-a[1].equalities-a[1].inequalities).map(([name,c])=>[name,fmt(c.equalities),fmt(c.inequalities),fmt(c.nonzeros),fmt(c.bound_redundant_inequalities)])));
      root.append(audit);
    }
    root.append(table(['阶段', '水库数值坐标', '初解复核 / 提交方式', '初解耗时 s'],stageRows.map(([name,s])=>[name.toUpperCase(),s.reservoir_scaling === 'energy_coordinate' ? '等效存水电量（输出仍为水位）' : s.reservoir_scaling === 'original' ? '原水位' : '未知',s.primal_start?.accepted ? '通过 / 整数状态补全' : s.primal_start?.status === 'not_requested' ? '未启用' : s.primal_start?.status || '未知',fmt(s.primal_start?.runtime_sec)])));
    if(d.stages?.scuc?.projected_commitment_units != null) root.append(el('p',`SCUC 冗余开停机投影 ${fmt(d.stages.scuc.projected_commitment_units)} 台 · 储能小时方向投影 ${fmt(d.stages.scuc.compact_storage)} 台`));
    if(d.execution_timing) {
      const timing=d.execution_timing,recovery=d.recovery_execution;
      root.append(el('p',`本日总耗时 ${fmt(timing.day_wall_sec)} s · 主出清 ${fmt(timing.main_sec)} s · 恢复实验 ${fmt(timing.recovery_wall_sec)} s${recovery ? `（${recovery.experiments} 项，${recovery.workers} 路执行，每路求解线程 ${recovery.resolved_solver_threads || '自动'}）` : ''}`));
    }
    if (!d.valid) {
      root.append(el('p','主出清无有效方案，原因分析不可用。'));
      root.append(el('p',d.error || d.status));
      const failed=stageRows.find(([,s])=>['limit_without_verified_solution','solver_or_audit_failure','proven_infeasible'].includes(s.solution_quality));
      if(failed) {
        const [name,s]=failed;
        root.append(el('p',`${name.toUpperCase()} · 请求 ${s.requested_solver||'未记录'} / 实际 ${s.solver||'未记录'} · 时限 ${fmt(s.requested_time_limit_sec)} s · 实际耗时 ${fmt(s.runtime_sec)} s · 目标 GAP ${fmt(s.requested_mip_gap)}`));
        if(s.solution_quality==='limit_without_verified_solution')root.append(el('p','达到求解限额，但没有通过复核的可行解。目标 GAP 只控制已有可行解与有效界之间的差距，不能保证在时限内找到可行解。'));
        else if(s.solver_status==='Root relaxation failed')root.append(el('p','根节点连续松弛求解失败，尚未取得可用于整数搜索的根解。该状态不能证明市场边界不可行；需检查求解日志中的时间、迭代上限与数值状态。'));
        else if(s.solution_quality==='proven_infeasible')root.append(el('p','求解器报告模型不可行，需要检查相互冲突的市场边界约束。'));
        if(typeof s.max_residual==='number' && s.max_residual>1e-6)root.append(el('p',`返回向量的最大原模型违反量为 ${fmt(s.max_residual)}，超过复核容差；不能作为有效出清结果。`));
      }
      return;
    }
    const p = d.periods[slot];
    root.append(el('p', `${dateAt(report.config.start_date, d.day)} ${time(slot)} · 负荷 ${fmt(p.load_mw)} MW；机组可用容量上界 ${fmt(p.available_generation_mw)} MW；缺额 ${fmt(p.deficit_mw)} MW；富余 ${fmt(p.surplus_mw)} MW；最大单线越限 ${fmt(p.max_line_overload_mw)} MW。`));
    root.append(el('p', `当日边界：${changes(d.boundary)}。区间 ${time(d.boundary.first_slot)}–${time(d.boundary.last_slot)}；报价倍数作用于全天。`));
    if (d.lookahead) root.append(table(['次日预测日期', '代表点', '来源时刻', '统调负荷 MW', '时长 h', '权重 h'], d.lookahead.points.map(p => [dateAt(report.config.start_date,d.lookahead.source_day), p.kind === 'peak' ? '峰' : '谷', time(p.source_slot), fmt(p.load_mw), fmt(p.duration_hr), fmt(p.weight_hr)])));
    root.append(el('p','ΔP、ΔPᵢⱼ 是当前已求得方案的诊断指标；限时可行解不能证明异常不可避免，配对差值也可能包含优化未收敛的影响。'));
    if(d.diagnostic_prices_valid)root.append(el('p','诊断价格属于恢复实验边界；请结合该实验的目标、约束与定价一致性状态判断价差。'));
    if (p.deficit_mw > 1e-6) root.append(el('p', p.load_mw > p.available_generation_mw + 1e-6 ? '证据：负荷高于机组可用容量上界。还需结合外送受电、储能、可控负荷及网络约束复核缺额。' : '证据：总容量上界未揭示供电不足；需结合网络输送、水量、爬坡及运行约束复核。'));
    if (p.max_line_overload_mw > 1e-6) root.append(el('p', '证据：下方线路功率超过当前有功限额。研究罚项允许越限，越限与缺额之间存在经济权衡。'));
    const refValue = v => Array.isArray(v) ? (v.some(x => typeof x === 'object') ? `${v.length}项（见导出配置）` : `[${v.join(', ')}]`) : fmt(v);
    if (d.counterfactuals.length) root.append(table(['恢复因素', '实际 → 参照', '重算状态 / SCUC 质量', '本时段缺额减少 MW', '本时段富余减少 MW', '本时段越限和减少 MW', '全天缺额减少 MWh'], d.counterfactuals.map(c => [names[c.factor], `${refValue(c.sampled_value)} → ${refValue(c.reference_value)}`, c.valid ? `${c.status} · ${quality(c.stages?.scuc)}` : c.error || c.status, fmt(c.valid ? p.deficit_mw - c.periods[slot].deficit_mw : null), fmt(c.valid ? p.surplus_mw - c.periods[slot].surplus_mw : null), fmt(c.valid ? p.overload_sum_mw - c.periods[slot].overload_sum_mw : null), fmt(c.reduction_deficit_mwh)])));
    const causeLabels = {not_requested:'未请求自动原因分析',not_triggered:'主结果未检出供需或越限异常，未执行恢复实验',unavailable:'主出清无有效方案，原因分析不可用',completed:'恢复实验已完成',partial_failure:'部分恢复实验失败',no_interventions:'当日未偏离参照边界，无恢复因素'};
    const causeStatus=el('p',causeLabels[d.cause_analysis?.status] || (d.counterfactuals.length ? '恢复实验已完成' : '旧结果未记录恢复触发状态'));
    causeStatus.dataset.testid='market-cause-status';root.append(causeStatus);
    if(d.cause_analysis?.pricing_scope)root.append(el('p',d.cause_analysis.pricing_scope==='full'?'恢复范围：含 LMP 完整重算':'恢复范围：SCUC / SCED；未计算恢复价格'));
    if(d.cause_analysis?.manual_wall_sec != null)root.append(el('p',`最近手动补算 ${fmt(d.cause_analysis.manual_wall_sec)} s（不计入原逐日运行耗时）`));
    const actions=el('div');actions.className='operation-actions';
    const pricing=el('select');pricing.setAttribute('aria-label','手动恢复范围');
    options(pricing,[['dispatch_only','供需归因（SCUC / SCED）'],['full','完整重算（含 LMP）']]);
    const explain=el('button','补算当日原因');explain.type='button';explain.className='btn btn-sm';explain.dataset.testid='market-explain-day';
    explain.disabled=!recoveryPolicies || !d.valid || report.boundary_revision!==state.revision || ['stale','cancelled'].includes(report.status) ||
      (state.preview ? !state.previewExplanation || state.previewExplanation.locked : state.running || state.busy || state.serverBusy);
    pricing.disabled=explain.disabled;
    const selectedDay=d.day;
    explain.onclick=async()=>{
      explain.disabled=true;pricing.disabled=true;
      try {
        if(state.preview) await state.previewExplanation.run(selectedDay,pricing.value);
        else {
          if(state.running || state.busy || state.serverBusy)return;
          state.busy=true;controls();status(`正在补算第 ${selectedDay+1} 日原因`);
          Object.assign(state,await api({action:'explain',run_id:state.run_id,day:selectedDay,pricing:pricing.value}));
        }
      } catch(e) { $('operationWorkflowStatus').textContent=e.message; }
      finally { state.busy=false;results(); }
    };
    actions.append(pricing,explain);root.append(actions);
    root.append(el('p', '恢复差值固定当日日初状态；正值表示恢复后减少，负值表示增加。差值属于条件敏感性，不能相加解释为唯一原因。'));
    if (d.binding_constraints) {
      const audit = el('details'); audit.append(el('summary', 'SCED 生效约束证据（非唯一原因）'));
      const labels = { '2.6.3.3': '机组约束', '2.6.3.9': '机组群功率', '2.6.3.10': '机组群电量', '2.6.3.14': '网络限额', reservoir: '水库', storage: '储能' };
      const rows = d.binding_constraints.filter(r => r.name.endsWith(`/${slot}`) || r.name.endsWith(`/${slot}/upper`) || r.name.endsWith(`/${slot}/lower`));
      audit.append(table(['约束 / 设备 / 时段', '左端量', '右端界', '违反量'], rows.map(r => [Object.entries(labels).reduce((name, [key,label]) => name.replace(key,label),r.name), fmt(r.lhs),fmt(r.rhs),fmt(r.violation)])));
      audit.append(el('p', `约束行保留原模型单位；生效仅表示当前解到达边界。${d.binding_constraints_truncated ? '完整 SCED 证据超过 1000 行，本列表已截断，缺失不代表未生效。' : ''}`)); root.append(audit);
    }
    const affected = $('operationOnlyAffected').checked;
    paged('operationNodes', ['节点 ID', '名称', 'ΔPᵢ MW', '缺额 MW', '富余 MW', '残差 MW', '诊断价 元/MWh'], d.nodes.filter(n => !affected || n.deficit_mw[slot] > 1e-6 || n.surplus_mw[slot] > 1e-6).map(n => [marketCanvas.link('buses', n.id), n.name, fmt(n.deficit_mw[slot]-n.surplus_mw[slot]), fmt(n.deficit_mw[slot]), fmt(n.surplus_mw[slot]), Number(n.node_imbalance_mw[slot]).toExponential(2), fmt(n.lmp_per_mwh?.[slot])]));
    paged('operationLines', ['线路 ID', '名称', '起点→终点', '投运', '功率 MW', '生效下限 MW', '生效上限 MW', 'ΔPᵢⱼ MW'], d.lines.filter(l => !affected || l.overload_mw[slot] > 1e-6).map(l => [marketCanvas.link('branches', l.id), l.name, `${l.from_bus}→${l.to_bus}`, l.available[slot] ? '是' : '否', fmt(l.power_mw[slot]), fmt(l.min_mw[slot]*l.available[slot]), fmt(l.max_mw[slot]*l.available[slot]), fmt(l.overload_mw[slot])]));
  }
  function results() {
    const j = state.preview || state.job; controls();
    window.HySimMarketWeeklyPlan.update(j,state.revision,(day,slot)=>{ $('operationResultDay').value=day;$('operationSlot').value=slot;details(); });
    $('operationSummary').replaceChildren(); $('operationDays').replaceChildren();
    if (!j) { for (const id of ['operationBalanceChart', 'operationOverloadChart']) { window.HySimMarketActivity.cancelRender(id); if (typeof Plotly !== 'undefined') Plotly.purge($(id)); } options($('operationResultDay'), []); $('operationLimitations').replaceChildren(); details(); return; }
    const labels = { ready: '待运行', running: '可继续', completed: '已完成', cancelled: '已终止', failed: '失败', stale: '边界已失效' };
    status(`${labels[j.status] || j.status} · 已完成 ${j.completed_days}/${j.total_days} 天${state.running ? ' · 正在计算下一日' : ''}${j.error ? ` · ${j.error}` : ''}${j.boundary_revision !== state.revision ? ' · 当前边界已变化，需新建仿真' : ''}`);
    const days = j.days.filter(d => d.valid);
    const sum = key => days.reduce((s, d) => s + d[key], 0);
    const totals=days.length ? `有效时段合计：缺额 ${fmt(sum('deficit_mwh'))} MWh · 富余 ${fmt(sum('surplus_mwh'))} MWh · 线路越限积分和 ${fmt(sum('overload_mwh'))} MW·h` : '暂无有效出清统计';
    $('operationSummary').textContent = `${j.boundary_name || '未载入边界'} · ${j.config.start_date || '起始日待设置'} · ${days.length * 96} 个已出清时段 · ${totals}`;
    $('operationDays').replaceChildren(table(['日期', '状态 / SCUC 质量', '缺额 MWh', '富余 MWh', '越限积分和 MW·h', 'SCED 残差', '诊断价'], j.days.map(d => {
      const button = el('button', dateAt(j.config.start_date, d.day)); button.className = 'btn btn-sm'; button.onclick = () => { $('operationResultDay').value = d.day; details(); };
      return [button, `${d.status} · ${quality(d.stages?.scuc)}`, fmt(d.deficit_mwh), fmt(d.surplus_mwh), fmt(d.overload_mwh), d.stages?.sced?.max_residual == null ? '不可用' : Number(d.stages.sced.max_residual).toExponential(2), d.diagnostic_prices_valid ? '有效（条件）' : '不可用'];
    })));
    options($('operationResultDay'), Array.from({ length: j.config.start_date ? j.total_days : 0 }, (_, day) => [day, `${dateAt(j.config.start_date, day)}${j.days[day] ? '' : ' · 待出清'}`]));
    $('operationLimitations').replaceChildren(...j.limitations.map(s => el('p', s)), el('p', '日前 96+2 点，日末第 96 点状态承接；储能每天保留基准申报终值。非空启停功率轨迹不支持此滚动入口。缺额罚价过低可能诱发经济性缺额。线路越限积分为跨线路累加量，不是缺供电量。任务由浏览器逐日推进，关闭页面后需重载并继续。'));
    for(const id of ['operationBalanceChart','operationOverloadChart']) {
      $(id).hidden=!days.length;
      if(!days.length) { window.HySimMarketActivity.cancelRender(id); if(typeof Plotly!=='undefined')Plotly.purge($(id)); }
    }
    if (typeof Plotly !== 'undefined' && days.length) {
      const x = days.flatMap(d => d.periods.map(p => `${dateAt(j.config.start_date, d.day)}T${time(p.slot)}:00`));
      const style = getComputedStyle(document.body);
      const chart = (id, series, title) => Plotly.react($(id), series.map(([key, name, color]) => ({ x, y: days.flatMap(d => d.periods.map(p => p[key])), name, line: { color }, mode: 'lines', type: 'scatter' })), { title: { text: title, font: { size: 14 } }, paper_bgcolor: style.getPropertyValue('--bg2').trim(), plot_bgcolor: style.getPropertyValue('--bg2').trim(), font: { color: style.getPropertyValue('--ink').trim() }, height: 280, margin: { t: 40, l: 58, r: 12, b: 70 }, yaxis: { title: { text: 'MW' } }, xaxis: { type: 'date' }, legend: { orientation: 'h', y: -0.25 }, autosize: true }, { responsive: true, displaylogo: false });
      window.HySimMarketActivity.whenVisible('operationBalanceChart', () => chart('operationBalanceChart', [['deficit_mw', '节点缺额合计', '#c0392b'], ['surplus_mw', '节点富余合计', '#168578']], '节点不平衡时序'));
      window.HySimMarketActivity.whenVisible('operationOverloadChart', () => chart('operationOverloadChart', [['max_line_overload_mw', '最大单线越限', '#ad4a22'], ['overload_sum_mw', '线路越限合计', '#3768ab']], '输电线路有功越限'));
    }
    details();
  }
  let loading = null;
  function reload(preserveDraft = false) {
    if (!loading) loading = loadTask(preserveDraft).finally(() => { loading = null; });
    return loading;
  }
  async function loadTask(preserveDraft) {
    const data = await api(); Object.assign(state, { revision: data.revision, run_id: data.run_id, job: data.job, serverBusy: data.busy });
    const market = await api(undefined, '/api/session/southern_market');
    window.HySimMarketWeeklyPlan.boundary(market.boundary);
    const mixedOption=$('operationCase').querySelector('option[value="ieee118_mixed"]');
    mixedOption.disabled=!market.case_profiles?.includes('ieee118_mixed');
    if(mixedOption.disabled && $('operationCase').value==='ieee118_mixed')$('operationCase').value='demo';
    boundaryLoaded=!!market.boundary;
    updateSolverCapabilities(data.solver_capabilities || []);
    updateRecoveryPolicies(data.recovery_policies);
    if(!preserveDraft) solverEditor('operation',data.job?.config.solver_options || market.boundary?.execution);
    marketCanvas.setBoundary(market.boundary, market.revision);
    window.HySimMarketBoundary?.load(market, data.boundary_catalog);
    options($('operationPtdfBranch'),(market.boundary?.branches||[]).map(l=>[l.id,`${l.id} / ${l.from_bus}→${l.to_bus}`]));
    $('operationPtdfResult').replaceChildren();
    $('operationBoundaryName').textContent = `当前已保存边界：${data.boundary_name || '未载入'} · 修订 ${data.revision}`;
    const fleet={};for(const g of market.boundary?.generators||[])fleet[g.kind]=(fleet[g.kind]||0)+1;
    const kindNames={thermal:'火电',hydro:'水电',wind:'风电',solar:'光伏',renewable:'新能源',pumped_hydro:'抽蓄',vpp:'虚拟电厂'};
    const sourceDetails = el('details'); sourceDetails.append(el('summary', '数据来源与研究假设'), el('p', market.boundary?.source || '尚未载入'));
    $('operationFleetSummary').replaceChildren(...(market.boundary?[el('p',`${market.boundary.buses.length} 节点 · ${market.boundary.branches.length} 支路 · ${Object.entries(fleet).map(([k,v])=>`${kindNames[k]||k} ${v} 台`).join(' · ')} · 储能 ${market.boundary.storage.length} · 可控负荷 ${market.boundary.controllable_loads?.length||0}`),sourceDetails]:[]));
    if (data.job && !preserveDraft) {
      const c = data.job.config; $('operationHorizon').value = c.horizon; $('operationStartDate').value = c.start_date; $('operationPenalty').value = c.penalty_per_mwh; $('operationExplain').checked = c.explain;
      recoveryEditor('operation',c);
      state.days = structuredClone(c.days); calendar();
    }
    if (!preserveDraft) state.dirty = false;
    if (!workflowRestored && boundaryLoaded) { workflowRestored=true;setStep(Math.max(1,Math.min(4,Number(sessionStorage.getItem('hysim.marketOperationStep'))||(data.job?.days?.length ? 4 : 1)))); }
    results(); if (data.busy) status('服务端正在计算，完成后可重载任务');
    else if (!data.job) status(data.boundary_name ? '边界已就绪' : '请加载市场算例或保存南方市场边界');
    document.dispatchEvent(new CustomEvent('market-operation-boundary-loaded', { detail: { revision: data.revision } }));
  }
  async function loop() {
    state.running = true; state.pause = false; controls();
    try {
      while (!state.pause && ['ready', 'running'].includes(state.job.status)) {
        status(`正在计算第 ${state.job.completed_days + 1}/${state.job.total_days} 天 · 已完成 ${state.job.completed_days} 天`);
        const data = await api({ action: 'step', run_id: state.run_id, day: state.job.completed_days }); Object.assign(state, data); results();
        if (marketCanvas.follow()) { $('operationResultDay').value = Math.max(0, state.job.completed_days - 1); details(); }
      }
    } finally { state.running = false; controls(); results();setStep(4); }
  }
  const handle = fn => async () => { try { await fn(); } catch (e) { status(e.message);$('operationWorkflowStatus').textContent=e.message; controls(); } };
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
  document.querySelectorAll('[data-operation-step]').forEach(n=>n.onclick=()=>setStep(Number(n.dataset.operationStep)));
  bind('operationPreviousStep',()=>setStep(Number($('marketOperationWorkspace').dataset.workflowStep)-1));
  bind('operationNextStep',()=>{
    const step=Number($('marketOperationWorkspace').dataset.workflowStep);
    if(step===2){if(!$('marketForecastWorkspace').hidden){$('forecastGenerate').click();return;}saveDay();}
    setStep(step+1);
  });
  bind('operationBoundary', () => App.setActiveModule('marketBoundary'));
  $('operationCase').onchange=()=>{ $('operationThermalLabel').hidden=$('operationCase').value!=='activsg2000_hydro'; };
  bind('operationPtdfQuery',async()=>{
    const target=$('operationPtdfResult');target.replaceChildren(el('p','正在计算当前边界草稿的 PTDF'));
    try {
      const config=window.HySimMarketOperation.config();
      const result=await api({revision:state.revision,config,day:Number($('operationEditDay').value),period:Number($('operationPtdfPeriod').value)-1,branch_ids:[Number($('operationPtdfBranch').value)]},'/api/session/market_ptdf');
      const row=result.rows[0];
      target.replaceChildren(el('p',`${dateAt(config.start_date,result.day)} · 时段 ${result.period+1} · 拓扑 ${result.unique_topologies} 种 · 电气孤岛 ${result.reference_bus_ids.length} 个 · 相同拓扑 ${result.equivalent_periods.length} 时段 · ${fmt(result.runtime_sec)} s`));
      target.append(el('p',`线路 ${row.branch_id} · ${row.available?'投运':'停运'} · 移相器固定潮流 ${fmt(row.phase_shift_flow_mw)} MW · 线性方程残差 ${Number(result.max_linear_residual).toExponential(2)}`));
      target.append(el('p','正值：该节点注入增加 1 MW、同岛参考节点减少 1 MW 时，线路正向潮流的增加量。'));
      const list=el('div');list.id='operationPtdfRows';target.append(list);
      paged(list.id,['AC 节点 ID','同岛参考节点','PTDF MW/MW'],result.bus_ids.map((id,i)=>[marketCanvas.link('buses',id),result.bus_reference_ids[i],Number(row.coefficients[i]).toPrecision(8)]));
    } catch(e) { target.replaceChildren(el('p',e.message));throw e; }
  });
  bind('operationLoadCase', async () => {
    state.busy = true; controls();
    try { const current = await api(undefined, '/api/session/southern_market'); const body={ action: $('operationCase').value, revision: current.revision };if(body.action==='activsg2000_hydro'){if(!$('operationThermalCount').reportValidity())throw new Error('火电台数须为 0–544 的整数');body.thermal_limit=Number($('operationThermalCount').value);} await api(body, '/api/session/southern_market'); await reload();calendar(true);state.dirty=false;setStep(2); }
    finally { state.busy = false; controls(); }
  });
  bind('operationRun', async () => {
    if (state.running || state.busy) return; saveDay();setStep(3);
    status('正在校验每日边界并建立新仿真');
    state.busy = true; controls();
    try {
      const config = { horizon: $('operationHorizon').value, start_date: $('operationStartDate').value, penalty_per_mwh: Number($('operationPenalty').value), ...recoveryOptions('operation'), days: state.days,solver_options:solverOptions('operation') };
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
  window.HySimMarketOperation = {
    solverEditor, solverOptions, solverDraft, updateSolverCapabilities, setStep, recoveryOptions, recoveryEditor,
    day() { return state.days[Number($('operationEditDay').value)]; },
    setEdits(edits) { state.days[Number($('operationEditDay').value)].boundary_overrides = edits; state.dirty = true; plan(); },
    config() { saveDay(); return { horizon: $('operationHorizon').value, start_date: $('operationStartDate').value, penalty_per_mwh: Number($('operationPenalty').value), ...recoveryOptions('operation'), days: structuredClone(state.days),solver_options:solverOptions('operation') }; },
    preview(report, explanation = null) { state.preview = report; state.previewExplanation = explanation; results(); },
    table, paged, dateAt, fmt,
  };
  solverEditor('operation');
})();
