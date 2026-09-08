// IEEE 118 system and forecast-resolution protocol: southern_execution_contract.md.
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { mkdir, writeFile, readFile } from 'node:fs/promises';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const index = process.argv.indexOf('--server');
const exe = index < 0 ? path.join(root,'build/macos-release/run_gui_server') : process.argv[index+1];
const port = await new Promise(resolve => { const s=createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));}); });
const prune = process.argv.includes('--row-presolve');
const mixed = process.argv.includes('--mixed');
const solverIndex=process.argv.indexOf('--solver');
const requestedSolver=solverIndex<0?null:process.argv[solverIndex+1];
assert.ok(requestedSolver===null || ['native','highs','gurobi'].includes(requestedSolver));
const base = `http://127.0.0.1:${port}`, output=path.join(root,`output/market-operation/ieee118${mixed?'-mixed-week':''}${requestedSolver?'-'+requestedSolver:''}${prune?'-row-presolve':''}`);
const server = spawn(exe,['--host','127.0.0.1','--port',String(port),'--data-dir',path.join(root,'data')],{cwd:root,stdio:'ignore'});
let browser;
const request = async (route, body) => {
  const response=await fetch(base+route,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});
  const data=await response.json();assert.ok(response.ok,JSON.stringify(data));return data;
};
const operation = body => request('/api/session/market_operation',body);
const southern = body => request('/api/session/southern_market',body);
const persist = (name,value) => writeFile(path.join(output,name),JSON.stringify(value,null,2)+'\n');
const close = (a,b,label) => assert.ok(Math.abs(a-b)<=1e-5*Math.max(1,Math.abs(b)),`${label}: ${a} vs ${b}`);
const closePlan = (a,b,label) => assert.ok(Number.isFinite(a)&&Math.abs(a-b)<=1e-6,`${label}: ${a} vs ${b}`);
const validateDay = day => {
  assert.equal(day.valid,true,JSON.stringify(day.stages));assert.equal(day.diagnostic_prices_valid,true);
  assert.equal(day.periods.length,96);assert.equal(day.nodes.length,118);assert.equal(day.lines.length,186);
  for(const [table,field,count] of [['generators','power_mw',mixed?66:60],['storage','energy_mwh',mixed?6:2],['reservoirs','level_m',mixed?12:2],['controllable_loads','reduction_mw',mixed?6:2]]){
    assert.equal(day.resources[table].length,count);
    for(const row of day.resources[table])assert.equal(row[field].length,96);
  }
  for(const stage of Object.values(day.stages)) { assert.ok(stage.max_residual<=1e-6);assert.equal(stage.requested_mip_gap,.01); }
  close(day.deficit_mwh,day.periods.reduce((s,p)=>s+.25*p.deficit_mw,0),'deficit integral');
  close(day.overload_mwh,day.periods.reduce((s,p)=>s+.25*p.overload_sum_mw,0),'line excess integral');
  for(let t=0;t<96;t++) {
    close(day.periods[t].deficit_mw,day.nodes.reduce((s,n)=>s+n.deficit_mw[t],0),'node sum');
    for(const line of day.lines) {
      const expected=Math.max(0,line.power_mw[t]-line.max_mw[t],line.min_mw[t]-line.power_mw[t]);
      close(line.delta_pij_mw[t],expected,'signed-limit overload');
      if(!line.available[t])close(line.power_mw[t],0,'outaged line flow');
      const limit=line.power_mw[t]>=0?line.max_mw[t]:-line.min_mw[t];
      if(line.available[t]&&limit>0)closePlan(line.loading_percent[t],100*Math.abs(line.power_mw[t])/limit,'directional active loading');
      else assert.equal(line.loading_percent[t],null);
    }
    for(const g of day.resources.generators){
      assert.ok(Number.isFinite(g.primary_reserve_mw[t]));
      if(['wind','solar','renewable'].includes(g.kind)){
        closePlan(g.curtailment_mw[t],Math.max(0,g.renewable_available_mw[t]-g.power_mw[t]),'curtailment');
        if(g.renewable_available_mw[t]>0)closePlan(g.utilization_percent[t],100*g.power_mw[t]/g.renewable_available_mw[t],'utilization');
        else assert.equal(g.utilization_percent[t],null);
      }else assert.equal(g.utilization_percent[t],null);
    }
    for(const a of day.resources.areas){assert.ok(a.reserve_up_margin_mw[t]>=-1e-6);assert.ok(a.reserve_down_margin_mw[t]>=-1e-6);}
  }
};
const summary = day => {
  const prices=day.nodes.flatMap(n=>n.lmp_per_mwh).sort((a,b)=>a-b);
  assert.ok(prices.every(Number.isFinite));
  return {runtime_sec:day.runtime_sec,deficit_mwh:day.deficit_mwh,overload_mwh:day.overload_mwh,
    peak_deficit_mw:Math.max(...day.periods.map(p=>p.deficit_mw)),peak_line_excess_mw:Math.max(...day.periods.map(p=>p.max_line_overload_mw)),
    load_mwh:day.periods.reduce((s,p)=>s+.25*p.load_mw,0),peak_load_mw:Math.max(...day.periods.map(p=>p.load_mw)),
    price_min:prices[0],price_median:prices[Math.floor(prices.length/2)],price_max:prices.at(-1),
    scuc_objective:day.stages.scuc.objective,variables:day.stages.scuc.variables,nonzeros:day.stages.scuc.nonzeros};
};
try {
  await mkdir(output,{recursive:true});
  for(let i=0;i<100;i++){try{if((await fetch(base+'/api/cases')).ok)break;}catch{}await new Promise(r=>setTimeout(r,100));}
  browser=await chromium.launch();const page=await browser.newPage({viewport:{width:1440,height:1000}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.addInitScript(()=>{localStorage.setItem('hysim.tourDone.v1','1');localStorage.setItem('hysim.marketOperationMode','manual');});
  await page.goto(base+'/xjtu/#market-operation');await page.locator('#operationCase').selectOption(mixed?'ieee118_mixed':'ieee118');
  await page.locator('#operationLoadCase').click();
  await page.waitForFunction(()=>document.querySelector('#operationBoundaryName').textContent.includes('IEEE 118'));
  const snapshot=await southern(), boundary=snapshot.boundary;
  if(prune) {
    boundary.execution.row_presolve='enabled';
    await southern({action:'save',revision:snapshot.revision,boundary});
  }
  assert.equal(boundary.buses.length,118);assert.equal(boundary.branches.length,186);assert.equal(boundary.generators.length,mixed?66:60);
  assert.equal(boundary.reservoirs.length,mixed?12:2);assert.equal(boundary.reservoirs[1].upstream,boundary.reservoirs[0].id);
  assert.match(boundary.source,/synthetic/);
  await persist('boundary.json',boundary);
  const meta=await operation();
  const solver=requestedSolver || (meta.solver_capabilities.find(s=>s.id==='gurobi'&&s.available)?'gurobi':'highs');
  assert.ok(meta.solver_capabilities.some(s=>s.id===solver&&s.available));
  const config={horizon:'week',start_date:'2026-09-07',explain:false,penalty_per_mwh:100000,
    solver_options:{solver,time_limit_sec:60,mip_gap:.01,threads:solver==='gurobi'?2:0},days:[{},
      {load_scale:4,first_slot:40,last_slot:43},{line_limit_scale:.05,first_slot:40,last_slot:43},
      {branch_outages:boundary.branches.map(l=>l.id),first_slot:40,last_slot:43},
      {wind_scale:0,solar_scale:0,inflow_scale:0},{generator_bid_scale:1.2,load_bid_scale:1.2},{},{}]};
  let run=await operation({action:'start',revision:meta.revision,config});
  const report={source:boundary.source,solver,solver_budget_sec:60,gap:.01,calculation_step_minutes:15,day_points:96,lookahead_points:2,week:[],forecast_resolution:[]};
  for(let day=0;day<7;day++) {
    run=await operation({action:'step',run_id:run.run_id,day});validateDay(run.job.days[day]);
    const effective=(await operation({action:'preview',revision:meta.revision,config,day})).preview.effective;
    const inputs=new Map(effective.generators.map(g=>[g.id,g]));
    for(const g of run.job.days[day].resources.generators)for(let t=0;t<96;t++){
      const source=inputs.get(g.id),p=g.power_mw[t],u=g.online[t];
      closePlan(g.reserve_up_contribution_mw[t],(source.reserve_up_eligible[t]?(source.pmax_mw[t]-source.regulation_up_mw[t])*u:0)-p,'SCED up reserve contribution');
      closePlan(g.reserve_down_contribution_mw[t],p-(source.reserve_down_eligible[t]?(source.pmin_mw[t]+source.regulation_down_mw[t])*u:0),'SCED down reserve contribution');
      if(['wind','solar','renewable'].includes(g.kind))closePlan(g.renewable_available_mw[t],Math.min(source.pmax_mw[t],source.forecast_mw[t])*source.available[t]*(1-source.must_off[t]),'effective renewable availability');
    }
    const resources=run.job.days[day].resources,busAreas=new Map(effective.buses.map(b=>[b.id,b.area])),storageInputs=new Map(effective.storage.map(s=>[s.id,s]));
    for(const a of resources.areas)for(let t=0;t<96;t++){
      const units=resources.generators.filter(g=>g.area===a.id),stores=resources.storage.filter(s=>busAreas.get(storageInputs.get(s.id).bus)===a.id);
      const net=stores.reduce((sum,s)=>sum+s.discharge_mw[t]+s.charge_mw[t],0);
      closePlan(a.reserve_up_mw[t],units.reduce((sum,g)=>sum+g.reserve_up_contribution_mw[t],0)-net,'area up with storage');
      closePlan(a.reserve_down_mw[t],units.reduce((sum,g)=>sum+g.reserve_down_contribution_mw[t],0)+net,'area down with storage');
    }
    if(day>0)assert.deepEqual(run.job.days[day].state_start,run.job.days[day-1].state_end);
    assert.equal(run.job.days[day].stages.scuc.requested_solver,solver);
    if(solver==='native')assert.equal(run.job.days[day].stages.scuc.solver,'NativeBranchAndCut');
    if(prune) {
      assert.ok(run.job.days[day].stages.scuc.model_size.constraints_removed>0);
      assert.equal(run.job.days[day].stages.lmp.model_size.constraints_removed,0);
    }
    report.week.push({day,...summary(run.job.days[day])});await persist('system-report.json',report);
    console.log(JSON.stringify({week_day:day+1,...report.week.at(-1)}));
  }
  assert.equal(run.job.status,'completed');assert.equal(run.job.days[6].lookahead.source_day,7);
  assert.ok(run.job.days[1].deficit_mwh>run.job.days[0].deficit_mwh+1);
  assert.ok(run.job.days[2].overload_mwh>run.job.days[0].overload_mwh+1);
  assert.ok(run.job.days[3].deficit_mwh>1);
  await persist('week.json',run.job);
  await page.locator('#operationReload').click();await page.locator('[data-operation-step="4"]').click();
  await page.waitForFunction(()=>document.querySelector('#operationStatus').textContent.includes('7/7'));
  await page.locator('#operationResultDay').selectOption('2');
  await page.locator('#marketCanvasEntity').selectOption('buses:'+boundary.buses[0].id);
  const plan=page.locator('#operationPlanMetric');
  await plan.selectOption('online');
  assert.equal(await page.locator('#operationPlanNumbers').getAttribute('open'),null);
  for(const id of ['Generation','Commitment','Reserve','Loading','Renewable','Water'])assert.ok(await page.locator('#operationPlan'+id).evaluate(n=>n.data?.some(trace=>trace.y.some(v=>typeof v==='number'&&Number.isFinite(v)))));
  const overview=await page.evaluate(()=>Object.fromEntries(['Generation','Commitment','Reserve','Loading','Renewable','Water'].map(id=>[id,document.querySelector('#operationPlan'+id).data.map(trace=>({name:trace.name,value:trace.y[232]}))])));
  const selectedDay=run.job.days[2],selectedResources=selectedDay.resources;
  const typeNames={hydro:'水电',thermal:'火电',wind:'风电',solar:'光伏',renewable:'新能源'};
  for(const [kind,name] of Object.entries(typeNames)){
    const rows=selectedResources.generators.filter(g=>g.kind===kind);if(!rows.length)continue;
    closePlan(overview.Generation.find(t=>t.name===name).value,rows.reduce((s,g)=>s+g.power_mw[40],0),'generation stack');
    closePlan(overview.Commitment.find(t=>t.name===name).value,rows.filter(g=>g.online[40]>.5).length,'online count');
  }
  closePlan(overview.Reserve[0].value,selectedResources.areas.reduce((s,a)=>s+a.reserve_up_mw[40],0),'reserve overview');
  closePlan(overview.Loading[0].value,Math.max(...selectedDay.lines.map(l=>l.loading_percent[40]).filter(Number.isFinite)),'loading overview');
  const renewables=selectedResources.generators.filter(g=>['wind','solar','renewable'].includes(g.kind));
  closePlan(overview.Renewable[0].value,renewables.reduce((s,g)=>s+g.renewable_available_mw[40],0),'renewable overview');
  for(const r of selectedResources.reservoirs)closePlan(overview.Water.find(t=>t.name===r.name).value,r.level_m[40],'water overview');
  await page.locator('#operationPlanNumbers > summary').click();
  assert.ok((await page.locator('#operationPlanPage').textContent()).includes(mixed?'66':'60'));
  assert.equal(await page.locator('#operationPlanEntity option').count(),mixed?66:60);
  assert.deepEqual(await page.locator('#operationPlanHeatmap').evaluate(n=>[n.data[0].z.length,n.data[0].z[0].length]),[40,672]);
  assert.ok(await page.locator('#operationPlanHeatmap').evaluate(n=>n.clientHeight>=n._fullLayout.height),'all heatmap rows must fit the container');
  await page.locator('#operationPlanNext').click();
  const lastPageButton=page.locator('#operationPlanTable [data-plan-target]').first();
  const target=await lastPageButton.getAttribute('data-plan-target');await lastPageButton.click();
  assert.equal(await page.locator('#marketCanvasEntity').inputValue(),target);
  await page.locator('#operationPlanPrev').click();
  for(const [field,table,count] of [['power_mw','generators',mixed?66:60],['primary_reserve_mw','generators',mixed?66:60],['reserve_up_contribution_mw','generators',mixed?66:60],['reserve_down_contribution_mw','generators',mixed?66:60],['reserve_up_mw','areas',boundary.areas.length],['reserve_down_mw','areas',boundary.areas.length],['loading_percent','branches',186],['level_m','reservoirs',mixed?12:2],['energy_mwh','storage',mixed?6:2],['reduction_mw','controllable_loads',mixed?6:2]]){
    await plan.selectOption(field);assert.equal(await page.locator('#operationPlanEntity option').count(),count);
    const id=Number(await page.locator('#operationPlanEntity').inputValue());
    const expected=(table==='branches'?run.job.days[2].lines:run.job.days[2].resources[table]).find(r=>r.id===id)[field][40];
    const plotted=await page.locator('#operationPlanSeries').evaluate(n=>n.data[0].y[232]);
    if(expected===null)assert.equal(plotted,null);else closePlan(plotted,expected,'weekly chart source');
  }
  await plan.selectOption('utilization_percent');await page.locator('#operationPlanKind').selectOption('wind');
  assert.equal(await page.locator('#operationPlanEntity option').count(),mixed?6:2);
  const downloadEvent=page.waitForEvent('download');await page.locator('#operationPlanExport').click();
  const download=await downloadEvent;await download.saveAs(path.join(output,'wind-week.csv'));
  const csv=await readFile(path.join(output,'wind-week.csv'),'utf8');
  assert.equal(csv.trim().split(/\r?\n/).length,1+(mixed?6:2)*672);
  await page.locator('#operationPlanKind').selectOption('');await plan.selectOption('online');
  await page.evaluate(j=>{window.HySimMarketOperation.preview({...j,days:j.days.slice(0,1),completed_days:1});},run.job);
  assert.equal(await page.locator('#operationPlanHeatmap').evaluate(n=>n.data[0].z[0][96]),null);
  assert.ok((await page.locator('#operationPlanTable').textContent()).includes('待出清'));
  await page.evaluate(j=>window.HySimMarketOperation.preview({...j,boundary_revision:j.boundary_revision+1}),run.job);
  assert.ok(await page.locator('#operationPlanHeatmap').evaluate(n=>n.data[0].z.every(row=>row.every(v=>v===null))));
  await page.evaluate(()=>window.HySimMarketOperation.preview(null));
  if(mixed) {
    for(const [table,label] of [['generators','出力 MW'],['storage','能量 MWh'],['reservoirs','水位 m'],['controllable_loads','削减 MW']]){
      await page.locator('#marketCanvasEntity').selectOption(table+':'+boundary[table][0].id);
      assert.ok((await page.locator('#marketCanvasDetails').textContent()).includes(label));
      assert.ok(!(await page.locator('#marketCanvasDetails').textContent()).includes('当前报告未提供'));
    }
  }
  await page.screenshot({path:path.join(output,'desktop.png')});
  await page.setViewportSize({width:390,height:844});
  assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  await page.screenshot({path:path.join(output,'mobile.png')});
  assert.deepEqual(errors,[]);

  // Same 15-minute computation; block-average only authored forecast inputs.
  // Equal energy, a one-slot 4x peak, and changed shape isolate smoothing effects.
  for(const minutes of [15,30,60]) {
    const sample=structuredClone(boundary), block=minutes/15;
    for(const table of ['buses','areas'])for(const row of sample[table]) {
      const fields=table==='buses'?['load_mw','q_load_mvar']:['load_mw'];
      for(const field of fields)for(let t=0;t<96;t++)row[field][t]*=t===43?4:1;
    }
    for(const [table,fields] of [['buses',['load_mw','q_load_mvar']],['areas',['load_mw']],['generators',['forecast_mw']],['reservoirs',['inflow_m3_s']]]) {
      for(const row of sample[table])for(const field of fields) {
        const energy=row[field].slice(0,96).reduce((s,v)=>s+v,0);
        for(let start=0;start<96;start+=block) {
          const mean=row[field].slice(start,start+block).reduce((s,v)=>s+v,0)/block;
          for(let t=start;t<start+block;t++)row[field][t]=mean;
        }
        close(row[field].slice(0,96).reduce((s,v)=>s+v,0),energy,'authored forecast integral');
      }
    }
    const current=await southern();await southern({action:'save',revision:current.revision,boundary:sample});
    await persist(`forecast-${minutes}-boundary.json`,sample);
    const state=await operation();let trial=await operation({action:'start',revision:state.revision,config:{...config,days:[]}});
    trial=await operation({action:'step',run_id:trial.run_id,day:0});const day=trial.job.days[0];validateDay(day);
    report.forecast_resolution.push({forecast_minutes:minutes,...summary(day)});
    await persist(`forecast-${minutes}-day.json`,trial.job);await persist('system-report.json',report);
    console.log(JSON.stringify(report.forecast_resolution.at(-1)));
  }
  const fine=report.forecast_resolution[0];
  for(const coarse of report.forecast_resolution.slice(1)) {
    close(coarse.load_mwh,fine.load_mwh,'equal daily load energy');
    assert.equal(coarse.variables,fine.variables);assert.equal(coarse.nonzeros,fine.nonzeros);
    assert.ok(coarse.peak_load_mw<fine.peak_load_mw);
  }
  report.validation='passed';await persist('system-report.json',report);
  console.log('IEEE 118 passed: 7 rolling days, stressed boundaries, 96+2 provenance, original residuals, GUI, 15/30/60-minute forecast resolution at fixed 15-minute calculation.');
} finally {
  if(browser)await browser.close();
  if(server.exitCode===null){server.kill('SIGKILL');await new Promise(resolve=>server.once('close',resolve));}
}
