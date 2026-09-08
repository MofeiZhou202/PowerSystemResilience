import {spawn} from 'node:child_process';
import {createServer} from 'node:net';
import {mkdir,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';
import assert from 'node:assert/strict';
import path from 'node:path';
const root=process.cwd(),out=path.join(root,'output/market-operation/fault-inflow-study');
const arg=name=>{const i=process.argv.indexOf(name);return i<0?null:process.argv[i+1];};
const port=await new Promise(resolve=>{const s=createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});});
const base=arg('--base')||`http://127.0.0.1:${port}`;
const server=arg('--base')?null:spawn(arg('--server')||'./build/macos-release/tests/run_gui_server',['--host','127.0.0.1','--port',String(port),'--data-dir','data'],{cwd:root,stdio:'ignore'});
const api=async(route,body,code=200)=>{const r=await fetch(base+'/api/session/'+route,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});const j=await r.json();assert.equal(r.status,code,JSON.stringify(j));return j;};
let browser;
try {
  await mkdir(out,{recursive:true});
  for(let k=0;k<100;k++){try{if((await fetch(base+'/api/cases')).ok)break;}catch{}await new Promise(r=>setTimeout(r,100));}
  browser=await chromium.launch({headless:true});const page=await browser.newPage({viewport:{width:1500,height:1000}});
  await page.addInitScript(()=>localStorage.setItem('hysim.tourDone.v1','1'));
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto(base+'/xjtu/#market-study');
  await page.waitForFunction(()=>typeof Plotly!=='undefined');
  await page.locator('#studySetup').evaluate(n=>n.open=true);
  await page.locator('#studyLoadCase').click();
  await page.waitForFunction(()=>document.getElementById('studyBoundaryName').textContent.includes('118'));
  const boundary=(await api('southern_market')).boundary;
  assert.equal(boundary.reservoirs.length,12);assert.equal(boundary.generators.length,66);
  await page.locator('#studyConfig details').evaluate(n=>n.open=true);
  const caps=(await api('market_operation')).solver_capabilities;
  const solver=caps.some(c=>c.id==='gurobi'&&c.available)?'gurobi':'highs';
  await page.locator('#studySolver').selectOption(solver);await page.locator('#studyTimeLimit').fill('60');
  if(solver==='gurobi')await page.locator('#studyThreads').fill('2');
  await page.locator('#studyBranch').selectOption('113');
  await page.locator('#studyBid').fill('1,1.2');
  await page.locator('#studyGenerate').click();
  await page.waitForFunction(()=>document.querySelectorAll('#studyScenario option').length===18);
  let task=await api('market_study');
  assert.equal(task.job.scenarios.length,18);assert.equal(task.job.config.mode,'study');
  assert.equal((await api('market_forecast')).job,null,'study must not overwrite forecasts');
  assert.equal((await api('market_operation')).job,null,'study must not overwrite manual job');
  const started=Date.now();
  await page.locator('#studyRun').click();
  await page.waitForFunction(()=>document.getElementById('studyStatus').textContent.includes('完成 18/18 日窗'),null,{timeout:1200000});
  task=await api('market_study?export=1');
  const job=task.job;assert.equal(job.status,'completed');
  const normalState=job.scenarios[0].days[0].state_start;
  const report={solver,elapsed_sec:(Date.now()-started)/1000,scenarios:[],checks:[]};
  for(const s of job.scenarios){
    assert.equal(s.status,'completed',s.error||JSON.stringify(s.days[0]?.stages));assert.deepEqual(s.days[0].state_start,normalState);
    const day=s.days[0];assert.equal(day.periods.length,96);assert.equal(day.lookahead.points.length,2);
    assert.equal(day.analysis.ac_audit.periods.length,98);
    assert.equal(day.analysis.settlement.status,'conditional');assert.equal(day.analysis.settlement.formal_settlement_eligible,false);
    assert.ok(day.analysis.settlement.max_residual_cny<=day.analysis.settlement.tolerance_cny);
    assert.equal(day.analysis.settlement.accounts.length,78);
    const prices=new Map(day.nodes.map(n=>[n.id,n.lmp_per_mwh]));
    for(let t=0;t<96;t++){
      let receipts=0;
      for(const g of day.resources.generators)receipts+=.25*g.power_mw[t]*prices.get(g.bus)[t];
      for(const storage of day.resources.storage){const bus=boundary.storage.find(v=>v.id===storage.id).bus;receipts+=.25*(storage.discharge_mw[t]+storage.charge_mw[t])*prices.get(bus)[t];}
      const payments=day.nodes.reduce((v,n)=>v+.25*n.load_mw[t]*prices.get(n.id)[t],0)-day.resources.controllable_loads.reduce((v,l)=>v+.25*l.reduction_mw[t]*prices.get(boundary.controllable_loads.find(v=>v.id===l.id).bus)[t],0);
      const slack=day.nodes.reduce((v,n)=>v+.25*(n.deficit_mw[t]-n.surplus_mw[t])*prices.get(n.id)[t],0);
      const rent=day.lines.reduce((v,l)=>v+.25*l.power_mw[t]*(prices.get(l.to_bus)[t]-prices.get(l.from_bus)[t]),0);
      assert.ok(Math.abs(payments-receipts-slack-rent)<1e-4,'independent nodal cashflow');
    }
    for(const id of s.fault.generator_outages)assert.ok(day.resources.generators.find(g=>g.id===id).power_mw.every(p=>Math.abs(p)<1e-6));
    for(const id of s.fault.branch_outages)assert.ok(day.lines.find(l=>l.id===id).power_mw.every(p=>Math.abs(p)<1e-6));
    const preview=(await api('market_operation',{action:'preview',revision:task.revision,config:s.config,day:0})).preview.effective;
    for(const h of preview.reservoirs){const original=boundary.reservoirs.find(v=>v.id===h.id);assert.ok(Math.abs(h.inflow_m3_s[0]-original.inflow_m3_s[0]*s.inflow_scale)<1e-8);}
    report.scenarios.push({name:s.name,deficit_mwh:day.deficit_mwh,overload_mwh:day.overload_mwh,ac:day.analysis.ac_audit.status,ledger_residual_cny:day.analysis.settlement.max_residual_cny,objective:day.stages.sced.objective});
  }
  assert.equal(job.statistics.week_delta_p_peak_mw.probability_bounds,null);
  await writeFile(path.join(out,'study.json'),JSON.stringify(job));
  for(const scope of ['marketBehavior','marketBoundary','market','marketSecurity','marketSettlement']){
    await page.evaluate(scope=>App.setActiveModule(scope),scope);
    assert.equal(await page.locator('#marketStudyWorkspace').isVisible(),false);
  }
  await page.evaluate(()=>App.setActiveModule('marketStudy'));
  await page.locator('#marketStudyWorkspace').waitFor({state:'visible'});
  for(const panel of ['behavior','boundary','clearing','security','settlement']){
    await page.locator('#studyPanel').selectOption(panel);
    await page.waitForFunction(()=>document.getElementById('studyDetail').textContent&&!document.getElementById('studyDetail').textContent.includes('正在校验'));
    await page.waitForFunction(()=>document.querySelector('#studyDetailChart .main-svg'));
    await page.locator('#marketStudyWorkspace').screenshot({path:path.join(out,panel+'.png')});
  }
  await page.locator('#studyScenario').selectOption('6');await page.locator('#studyReference').selectOption('0');
  await page.locator('#studyMetric').selectOption('price');
  await page.waitForFunction(()=>document.getElementById('studyComparisonChart').data?.length===2);
  assert.equal(await page.locator('#studyComparisonChart').evaluate(n=>n.data[0].y.length),96);
  await page.locator('#studySlot').selectOption('40');
  await page.waitForFunction(()=>document.getElementById('marketCanvas').dataset.slot==='40'&&document.getElementById('marketCanvas').dataset.valid==='true');
  await page.locator('#marketCanvas-operationSlot').selectOption('41');
  assert.equal(await page.locator('#studySlot').inputValue(),'41');
  await page.locator('#studyComparisonChart').evaluate(n=>n.emit('plotly_click',{points:[{pointIndex:42}]}));
  assert.equal(await page.locator('#studySlot').inputValue(),'42');
  for(const width of [1500,760,390]){
    await page.setViewportSize({width,height:1000});await page.waitForTimeout(300);
    const overflow=await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth+1);assert.equal(overflow,false,`overflow ${width}`);
    const overlap=await page.evaluate(()=>{const a=document.getElementById('studyComparisonChart').getBoundingClientRect(),b=document.getElementById('studyDetailChart').getBoundingClientRect();return Math.min(a.right,b.right)-Math.max(a.left,b.left)>1&&Math.min(a.bottom,b.bottom)-Math.max(a.top,b.top)>1;});
    assert.equal(overlap,false);await page.screenshot({path:path.join(out,`viewport-${width}.png`)});
  }
  assert.deepEqual(errors,[]);report.checks.push('18 independent mixed IEEE118 scenarios, actual Plotly, isolated study with five analysis views, Canvas slot/curve interaction, desktop/mobile geometry');
  if(process.argv.includes('--keep-results')){
    await writeFile(path.join(out,'report.json'),JSON.stringify(report,null,2));console.log(JSON.stringify(report));
  } else {
  // Stale revision, duplicate step, independent tasks and cancellation contracts.
  await api('market_study',{action:'step',run_id:task.run_id,scenario:0,day:0},409);
  const config={...job.config};delete config.mode;
  config.inflow_scales=[1];config.bid_scales=[1];config.faults=[config.faults[0]];
  const fresh=await api('market_study',{action:'generate',revision:task.revision,config});
  await api('market_study',{action:'cancel',run_id:fresh.run_id});
  assert.equal((await api('market_study')).job.status,'cancelled');
  const market=await api('southern_market');
  await api('southern_market',{action:'save',revision:market.revision,boundary:market.boundary});
  await api('market_study',{action:'generate',revision:market.revision,config},409);
  config.operation.horizon='week';config.operation.days=[];config.operation.explain=true;
  config.inflow_scales=[.5];config.faults=[{...job.config.faults[0],name:'joint-week',branch_outages:[113],generator_outages:[1],first_day:1,last_day:3,first_slot:40,last_slot:43}];
  const revision=(await api('southern_market')).revision;
  let rolling=await api('market_study',{action:'generate',revision,config});
  for(let d=0;d<7;d++)rolling=await api('market_study',{action:'step',run_id:rolling.run_id,scenario:0,day:d});
  const week=rolling.job.scenarios[0];assert.equal(week.status,'completed',week.error);
  for(let d=0;d<7;d++){
    const day=week.days[d];if(d)assert.deepEqual(day.state_start,week.days[d-1].state_end);
    assert.equal(day.analysis.settlement.status,'conditional');
    assert.ok(day.counterfactuals.some(c=>c.factor==='inflow_scale'&&c.valid));
    if(d>=1&&d<=3){assert.ok(day.counterfactuals.some(c=>c.factor==='boundary_overrides'&&c.valid));for(let t=40;t<=43;t++)assert.ok(Math.abs(day.lines.find(l=>l.id===113).power_mw[t])<1e-6);}
  }
  await writeFile(path.join(out,'week.json'),JSON.stringify(rolling.job));
  report.checks.push('duplicate-step conflict, cancellation, stale revision, seven-day mixed joint-fault carry and paired restoration');
  await writeFile(path.join(out,'report.json'),JSON.stringify(report,null,2));
  console.log(JSON.stringify(report));
  }
} finally {await browser?.close();server?.kill('SIGTERM');}
