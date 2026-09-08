// Reproduce the saved user's full workload through the real browser run loop.
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { once } from 'node:events';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { chromium } from 'playwright';
import assert from 'node:assert/strict';
const option=(key,value)=>process.argv.includes(key)?process.argv[process.argv.indexOf(key)+1]:value;
const exe=option('--server','build/macos-release/tests/run_gui_server');
const input=option('--input','output/market-performance/live-input');
const out=option('--output','output/market-performance/live-replay');
const boundary=JSON.parse(await readFile(`${input}/southern_market.json`,'utf8')).boundary;
const saved=JSON.parse(await readFile(`${input}/market_forecast.json`,'utf8')).job;
const exported=JSON.parse(await readFile(`${input}/forecast-export.json`,'utf8')).job;
assert.deepEqual({...boundary,execution:{...boundary.execution,...saved.config.operation.solver_options}},exported.base);
const assembly=option('--assembly',null);
const trigger=option('--trigger',null),pricing=option('--pricing',null);
if(trigger)saved.config.operation.explain_trigger=trigger;
if(pricing)saved.config.operation.recovery_pricing=pricing;
if(assembly) {
  assert.ok(['reference','cached','verify'].includes(assembly));
  boundary.execution.assembly_mode=assembly;exported.base.execution.assembly_mode=assembly;
}
const port=await new Promise(resolve=>{const s=createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});});
const server=spawn(exe,['--host','127.0.0.1','--port',String(port),'--data-dir','data'],{stdio:'ignore'});
const base=`http://127.0.0.1:${port}`;
const api=async(route,body)=>{const r=await fetch(`${base}/api/session/${route}`,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});const j=await r.json();assert.ok(r.ok,JSON.stringify(j));return j;};
let browser;
try {
  await mkdir(out,{recursive:true});
  for(let k=0;k<100;k++){if(server.exitCode!==null||server.signalCode!==null)throw Error('Server exited');try{if((await fetch(base+'/api/cases')).ok)break;}catch{}await new Promise(r=>setTimeout(r,100));}
  const original=await api('southern_market');
  const loaded=await api('southern_market',{action:'save',revision:original.revision,boundary});
  const generated=await api('market_forecast',{action:'generate',revision:loaded.revision,config:saved.config});
  assert.deepEqual((await api('market_forecast?export=1')).job.base,exported.base);
  assert.deepEqual(generated.job.scenarios.map(s=>s.config.days),saved.scenarios.map(s=>s.config.days));
  browser=await chromium.launch();
  const page=await browser.newPage({viewport:{width:1440,height:1000}});
  await page.addInitScript(()=>{
    localStorage.setItem('hysim.tourDone.v1','1');localStorage.setItem('hysim.marketOperationMode','forecast');
    sessionStorage.setItem('hysim.marketOperationStep','3');window.replayLongTasks=[];
    new PerformanceObserver(list=>window.replayLongTasks.push(...list.getEntries().map(e=>e.duration))).observe({type:'longtask',buffered:true});
  });
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  page.on('response',r=>{if(r.url().endsWith('/api/session/market_forecast')&&r.request().method()==='POST')console.log(JSON.stringify({step:r.request().postDataJSON(),status:r.status()}));});
  await page.goto(base+'/xjtu/#market-operation');
  await page.waitForFunction(()=>!document.querySelector('#forecastRun').disabled);
  await page.evaluate(()=>HySimMarketOperation.setStep(3));
  await page.evaluate(()=>{window.replayLongTasks=[];performance.clearResourceTimings();});
  const begin=performance.now();await page.locator('#forecastRun').click();
  await page.waitForFunction(()=>document.querySelector('#marketOperationWorkspace').dataset.workflowStep==='4'&&document.querySelector('#forecastPause').disabled,null,{timeout:1200000});
  const wall=(performance.now()-begin)/1000;
  await page.screenshot({path:`${out}/complete-desktop.png`});
  const frontend=await page.evaluate(()=>({status:document.querySelector('#forecastStatus').textContent,
    long_tasks_ms:window.replayLongTasks,requests:performance.getEntriesByType('resource').filter(r=>r.name.endsWith('/api/session/market_forecast')).map(r=>({duration_ms:r.duration,bytes:r.decodedBodySize}))}));
  const result=await api('market_forecast');assert.equal(result.job.status,'completed');
  assert.match(await page.locator('[data-testid="market-price-consistency"]').textContent(),/通过：精确一致/);
  await page.locator('[data-testid="market-price-consistency"]').scrollIntoViewIfNeeded();
  await page.screenshot({path:`${out}/price-consistency-desktop.png`});
  await page.locator('[data-testid="market-cause-status"]').scrollIntoViewIfNeeded();
  await page.screenshot({path:`${out}/causes-desktop.png`});
  await page.setViewportSize({width:390,height:844});
  await page.locator('[data-testid="market-price-consistency"]').scrollIntoViewIfNeeded();
  await page.screenshot({path:`${out}/price-consistency-mobile.png`});
  await page.locator('[data-testid="market-cause-status"]').scrollIntoViewIfNeeded();
  assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
  await page.screenshot({path:`${out}/causes-mobile.png`});
  await page.setViewportSize({width:1440,height:1000});
  await page.locator('[data-testid="market-solver-timing"]').scrollIntoViewIfNeeded();
  await page.screenshot({path:`${out}/solver-timing-desktop.png`});
  assert.equal(result.job.completed_days,saved.completed_days);assert.deepEqual(errors,[]);
  const days=result.job.scenarios.flatMap(s=>s.days);
  for(const d of days){assert.equal(d.valid,true);for(const c of d.counterfactuals)assert.equal(c.valid,true);
    for(const run of [d,...d.counterfactuals])if(run.stages.lmp) {
      assert.equal(run.stages.lmp.price_consistency.passed,true);
      assert.equal(run.stages.lmp.price_consistency.max_dual_difference,0);
      assert.equal(run.stages.lmp.lp_algorithm,'dual_simplex');
    }
    if(assembly==='verify')for(const run of [d,...d.counterfactuals])for(const s of Object.values(run.stages))
      assert.equal(s.assembly_template.matrix_comparison,'exact_match');
  }
  await writeFile(`${out}/job.json`,JSON.stringify(result.job));
  const report={binary_sha256:createHash('sha256').update(await readFile(exe)).digest('hex'),
    input,assembly,trigger,pricing,config:result.job.config,wall_sec:wall,frontend,errors,days:days.map(d=>({day:d.day,
      main_sec:d.runtime_sec,timing:d.execution_timing,recovery:d.recovery_execution,
      stages:Object.fromEntries(Object.entries(d.stages).map(([k,s])=>[k,{sec:s.runtime_sec,objective:s.objective,
        residual:s.max_residual,binary:s.binary_variables,projected_commitment_units:s.projected_commitment_units}])),counterfactuals:d.counterfactuals.length}))};
  await writeFile(`${out}/report.json`,JSON.stringify(report,null,2));
  const manualDay=option('--manual-day',null);
  if(manualDay!==null) {
    const manualBegin=performance.now();
    const manual=await api('market_forecast',{action:'explain',run_id:result.run_id,scenario:0,day:Number(manualDay),pricing:'dispatch_only'});
    assert.equal(manual.job.completed_days,result.job.completed_days);
    assert.deepEqual(manual.job.statistics,result.job.statistics);
    for(let s=0;s<result.job.scenarios.length;s++)for(let d=0;d<7;d++) {
      const a=result.job.scenarios[s].days[d],b=manual.job.scenarios[s].days[d];
      for(const key of ['resources','nodes','lines','stages','state_start','state_end','lookahead','execution_timing','recovery_execution'])assert.deepEqual(a[key],b[key]);
      if(s!==0 || d!==Number(manualDay))assert.deepEqual(a,b);
    }
    const explained=manual.job.scenarios[0].days[Number(manualDay)];
    assert.equal(explained.cause_analysis.status,'completed');
    for(const c of explained.counterfactuals){assert.equal(c.valid,true);assert.equal(c.prices_valid,false);assert.equal(c.stages.lmp,undefined);}
    await writeFile(`${out}/manual-job.json`,JSON.stringify(manual.job));
    await writeFile(`${out}/manual-report.json`,JSON.stringify({day:Number(manualDay),wall_sec:(performance.now()-manualBegin)/1000,execution:explained.manual_recovery_execution,main_and_carry_unchanged:true},null,2));
  }
  console.log(JSON.stringify({wall_sec:wall,days:days.length,frontend}));
} finally {
  await browser?.close();if(server.exitCode===null&&server.signalCode===null){const stopped=once(server,'exit');server.kill();await stopped;}
}
