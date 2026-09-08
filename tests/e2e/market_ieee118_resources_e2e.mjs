// Independent conservation/ablation checks: execution contract, Mixed IEEE118.
import {spawn} from 'node:child_process';
import {createServer} from 'node:net';
import {mkdir,writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const arg=process.argv.indexOf('--server'),exe=arg<0?path.join(root,'build/macos-release/tests/run_gui_server'):process.argv[arg+1];
const port=await new Promise(resolve=>{const s=createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});});
const base=`http://127.0.0.1:${port}`,out=path.join(root,'output/market-operation/ieee118-mixed-resources');
const server=spawn(exe,['--host','127.0.0.1','--port',String(port),'--data-dir',path.join(root,'data')],{cwd:root,stdio:'ignore'});
const api=async(route,body)=>{const r=await fetch(base+'/api/session/'+route,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});const j=await r.json();assert.ok(r.ok,JSON.stringify(j));return j;};
const persist=(name,data)=>writeFile(path.join(out,name+'.json'),JSON.stringify(data,null,2)+'\n');
const close=(a,b,label,tol=1e-6)=>assert.ok(Number.isFinite(a)&&Math.abs(a-b)<=tol,`${label}: ${a} vs ${b}`);
const index=rows=>new Map(rows.map(r=>[r.id,r]));
const integral=x=>x.slice(0,96).reduce((s,v)=>s+.25*v,0);
const valid=result=>{
  assert.equal(result.schedule_feasible,true,JSON.stringify(result.scuc));assert.equal(result.prices_valid,true);
  for(const stage of ['scuc','sced','lmp'])assert.ok(result[stage].max_residual<=1e-6);
};
const auditResources=result=>{
  valid(result);const b=result.effective_boundary,s=result.sced,g=index(s.generators),h=index(s.reservoirs);
  const generation={};let windForecast=0,windUsed=0,charge=0,discharge=0;
  for(const source of b.generators){const energy=integral(g.get(source.id).power_mw);generation[source.kind]=(generation[source.kind]||0)+energy;
    if(['wind','solar'].includes(source.kind)){windForecast+=integral(source.forecast_mw.map((v,t)=>v*source.available[t]));windUsed+=energy;}}
  for(const reservoir of b.reservoirs){const row=h.get(reservoir.id),parent=h.get(reservoir.upstream),input=b.reservoirs.find(r=>r.id===reservoir.upstream);
    for(let t=0;t<98;t++){
      const dt=b.periods[t].duration_hr,previous=t?row.level_m[t-1]:reservoir.initial_level_m;
      const routed=parent?(t>=reservoir.lag_slots?parent.release_m3_s[t-reservoir.lag_slots]:input.release_history_m3_s.at(t-reservoir.lag_slots)):0;
      const release=row.spill_m3_s[t]+reservoir.generators.reduce((sum,id)=>sum+g.get(id).power_mw[t]*reservoir.water_m3_mwh/3600,0);
      close(row.release_m3_s[t],release,'shared turbine release');
      close(row.level_m[t],previous+dt*3600/reservoir.area_m2*(reservoir.inflow_m3_s[t]+routed-release),'cascade water balance');
    }
  }
  for(const source of b.storage){const row=s.storage.find(r=>r.id===source.id),eta=Math.sqrt(source.roundtrip_efficiency);
    for(let t=0;t<98;t++){
      close(row.energy_mwh[t],(t?row.energy_mwh[t-1]:source.initial_mwh)-b.periods[t].duration_hr*(row.discharge_mw[t]/eta+eta*row.charge_mw[t]),'SOC recurrence');
      assert.ok(row.discharge_mw[t]<=1e-6||-row.charge_mw[t]<=1e-6,'simultaneous charge/discharge');
    }
    close(row.energy_mwh[97],source.terminal_mwh,'terminal SOC');charge-=integral(row.charge_mw);discharge+=integral(row.discharge_mw);
  }
  for(const source of b.controllable_loads){const row=s.controllable_loads.find(r=>r.id===source.id);
    assert.ok(integral(row.reduction_mw)<=source.max_day_reduction_mwh+1e-6);
    row.reduction_mw.forEach((v,t)=>assert.ok(v>=-1e-6&&v<=source.available[t]*source.max_reduction_mw[t]+1e-6));
  }
  return {generation_mwh:generation,renewable_forecast_mwh:windForecast,renewable_used_mwh:windUsed,
    renewable_utilization:windForecast>0?windUsed/windForecast:null,charge_mwh:charge,discharge_mwh:discharge,load_reduction_mwh:s.day_load_reduction_mwh};
};
try{
  await mkdir(out,{recursive:true});
  for(let k=0;k<100;k++){try{if((await fetch(base+'/api/cases')).ok)break;}catch{}await new Promise(r=>setTimeout(r,100));}
  await api('load_builtin',{case:'market_ieee118'});
  const loaded=await api('southern_market'),boundary=loaded.boundary;
  assert.equal(boundary.generators.length,66);assert.match(boundary.source,/IEEE118-mixed-v1/);
  const capabilities=(await api('market_operation')).solver_capabilities;
  const solver=capabilities.some(s=>s.id==='gurobi'&&s.available)?'gurobi':'highs';
  boundary.execution={...boundary.execution,solver,threads:solver==='gurobi'?2:0,time_limit_sec:60,balance_policy:'diagnostic',balance_penalty_per_mwh:100000};
  const solve=async b=>{const state=await api('southern_market');const saved=await api('southern_market',{action:'save',revision:state.revision,boundary:b});return api('run_southern_market',{revision:saved.revision});};
  const report={fixture:boundary.source,solver,checks:{},limitations:[
    'Synthetic offers and water model; no actual market-price oracle.',
    'Generic settlement/N-1 tests use the engineering snapshot; no cascade-aware Southern real-time settlement.',
    'Renewable utilization below is independently calculated for full single-day results; forecast API does not yet aggregate price or renewable-utilization distributions.'
  ]};
  const result=await solve(boundary);report.checks.base=auditResources(result);await persist('base',result);
  for(const kind of ['hydro','thermal','wind','solar'])assert.ok(report.checks.base.generation_mwh[kind]>0,kind+' unused');
  const stress=structuredClone(boundary);
  for(const table of ['areas','buses'])for(const row of stress[table])for(let t=40;t<44;t++)row.load_mw[t]*=4;
  const peak=await solve(stress);report.checks.peak=auditResources(peak);await persist('peak',peak);
  assert.ok(report.checks.peak.load_reduction_mwh>0);assert.ok(report.checks.peak.discharge_mwh>0);assert.ok(report.checks.peak.charge_mwh>0);
  const disabled=structuredClone(stress);
  for(const table of ['storage','controllable_loads'])for(const row of disabled[table])row.available.fill(0);
  for(const row of disabled.generators)if(['wind','solar'].includes(row.kind))row.available.fill(0);
  const outage=await solve(disabled);report.checks.resource_outage=auditResources(outage);await persist('resource-outage',outage);
  for(const key of ['charge_mwh','discharge_mwh','load_reduction_mwh','renewable_used_mwh'])close(report.checks.resource_outage[key],0,key);
  const required=structuredClone(boundary);required.execution.ac_security='required';required.execution.balance_policy='strict';required.execution.security_iterations=1;
  const security=await solve(required);assert.ok(security.security_iterations.length>0);
  report.checks.ac_security={status:security.status,prices_valid:security.prices_valid};
  if(security.status==='ac_security_failed')assert.equal(security.prices_valid,false);
  await persist('ac-security',security);
  await solve(boundary);
  const forecast=await api('market_forecast'),config=forecast.defaults;
  config.sample_count=2;config.seed=118;config.operation.explain=false;
  config.operation.start_date='2026-09-07';config.operation.solver_options={solver,time_limit_sec:60,mip_gap:.01,threads:solver==='gurobi'?2:0};
  config.correlation[1][2]=config.correlation[2][1]=.4;
  for(const marginal of config.marginals){marginal.distribution='uniform';marginal.lower=.95;marginal.upper=1.05;}
  const state=await api('southern_market');let f=await api('market_forecast',{action:'generate',revision:state.revision,config});
  assert.notDeepEqual(f.job.scenarios[0].config.days,f.job.scenarios[1].config.days);
  for(let scenario=0;scenario<2;scenario++)for(let day=0;day<7;day++){
    f=await api('market_forecast',{action:'step',run_id:f.run_id,scenario,day});
    assert.equal(f.job.scenarios[scenario].days[day].valid,true);
    console.log(`Joint forecast ${scenario+1}/2 day ${day+1}/7`);
  }
  assert.equal(f.job.statistics.complete_scenarios,2);assert.equal(f.job.statistics.failed_scenarios,0);
  const stats=f.job.statistics;assert.equal(stats.week_delta_pij_peak_mw.unknown_count,0);
  const values=f.job.scenarios.map(s=>Math.max(...s.days.flatMap(d=>d.periods.map(p=>p.max_line_overload_mw))));
  close(stats.week_delta_pij_peak_mw.event_probability_valid_only,values.filter(v=>v>1e-6).length/2,'overload probability');
  report.checks.forecast={complete_scenarios:2,overload:stats.week_delta_pij_peak_mw};await persist('forecast',f.job);
  const op=await api('market_operation');let month=await api('market_operation',{action:'start',revision:op.revision,config:{...config.operation,horizon:'month',start_date:'2027-02-01'}});
  assert.equal(month.job.total_days,28);
  for(let day=0;day<28;day++){
    month=await api('market_operation',{action:'step',run_id:month.run_id,day});assert.equal(month.job.days[day].valid,true);
    if(day)assert.deepEqual(month.job.days[day].state_start,month.job.days[day-1].state_end);
    console.log(`Monthly carry ${day+1}/28`);
  }
  assert.equal(month.job.status,'completed');report.checks.month={days:28,status:month.job.status};await persist('month',month.job);
  report.validation='passed';await persist('report',report);console.log(JSON.stringify(report));
}finally{if(server.exitCode===null){server.kill('SIGKILL');await new Promise(r=>server.once('close',r));}}
