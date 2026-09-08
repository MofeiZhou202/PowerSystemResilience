// Full-sized unchanged-boundary timing; --runs repeats within one server session.
import {spawn} from 'node:child_process';
import {createServer} from 'node:net';
import {once} from 'node:events';
import {mkdir,readFile,writeFile,open} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const option=(key,fallback)=>process.argv.includes(key)?process.argv[process.argv.indexOf(key)+1]:fallback;
const exe=option('--server','output/market-performance/price-final/run_gui_server');
const out=option('--output','output/market-performance/scale-baseline');
const seconds=Number(option('--seconds','180')),threads=Number(option('--threads','4'));
const runs=Number(option('--runs','1'));
assert.ok(Number.isSafeInteger(runs)&&runs>=1&&runs<=20,'--runs must be 1..20');
await mkdir(out,{recursive:true});
const port=await new Promise(resolve=>{const s=createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});});
const log=await open(`${out}/server.log`,'w');
const server=spawn(exe,['--host','127.0.0.1','--port',String(port),'--data-dir','data'],{
  stdio:['ignore',log.fd,log.fd],env:{...process.env,MIPSOLVERS_GUROBI_VERBOSE:'1'}});
const base=`http://127.0.0.1:${port}`;
let deadline,browser,page;
const browserErrors=[];
const api=async(route,body)=>{const r=await fetch(`${base}/api/session/${route}`,body?{
  method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});
  const raw=await r.text();assert.ok(r.ok,raw.slice(0,2000));return JSON.parse(raw);};
try {
  for(let i=0;i<100;i++){if(server.exitCode!==null||server.signalCode!==null)throw Error('Server exited');
    try{if((await fetch(`${base}/api/cases`)).ok)break;}catch{}await new Promise(r=>setTimeout(r,100));}
  const input=option('--input',null);
  const capabilities=await api('southern_market');
  const supportsStrategy=!!capabilities.schema?.properties?.execution?.properties?.large_mip_strategy;
  const strategy=option('--strategy','auto');
  assert.ok(supportsStrategy||!process.argv.includes('--strategy'), 'Server does not support --strategy');
  const authored=input?JSON.parse(await readFile(input,'utf8')):null;
  if(authored&&!supportsStrategy) {
    assert.ok(!authored.execution.large_mip_strategy||authored.execution.large_mip_strategy==='auto',
      'Input requires a strategy unsupported by this server');
    delete authored.execution.large_mip_strategy;
  }
  const loaded=await api('southern_market',input?{action:'save',revision:0,boundary:authored}:
    {action:'activsg2000_hydro',revision:0,thermal_limit:Number(option('--thermal','120'))});
  const boundary=loaded.boundary;
  Object.assign(boundary.execution,{solver:'gurobi',threads,time_limit_sec:seconds,mip_gap:.01,
    gurobi_method:option('--method','auto'),mip_start:option('--mip-start','auto'),
    row_presolve:option('--row-presolve','none'),assembly_mode:option('--assembly','cached'),
    ac_security:'schedule_only',balance_policy:'diagnostic',balance_penalty_per_mwh:100000});
  if(supportsStrategy)boundary.execution.large_mip_strategy=strategy;
  await writeFile(`${out}/boundary.json`,JSON.stringify(boundary));
  const saved=await api('southern_market',{action:'save',revision:loaded.revision,boundary});
  if(process.argv.includes('--browser')) {
    const {chromium}=await import('playwright');browser=await chromium.launch();
    page=await browser.newPage({viewport:{width:1440,height:1000}});
    page.on('pageerror',e=>browserErrors.push(e.message));
    await page.addInitScript(()=>localStorage.setItem('hysim.tourDone.v1','1'));
    await page.goto(`${base}/xjtu/`,{waitUntil:'domcontentloaded'});
    await page.waitForFunction(()=>typeof App!=='undefined');
    await page.evaluate(()=>App.setActiveModule('marketBoundary'));
    await page.locator('#southernLoad').click();
    await page.waitForFunction(()=>document.querySelector('#southernCaseSummary').textContent.includes('2000'));
  }
  const reports=[];
  for(let run=0;run<runs;++run) {
  const runOut=runs===1?out:`${out}/run-${String(run+1).padStart(2,'0')}`;
  await mkdir(runOut,{recursive:true});
  if(runs>1)await writeFile(`${runOut}/boundary.json`,JSON.stringify(boundary));
  if(page)await page.setViewportSize({width:1440,height:1000});
  deadline=setTimeout(()=>server.kill('SIGTERM'),Number(option('--watchdog','900'))*1000);
  const start=performance.now();
  let result,browserWall;
  if(page) {
    const response=page.waitForResponse(r=>r.url().endsWith('/api/session/run_southern_market')&&r.request().method()==='POST',{timeout:900000});
    await page.locator('#southernRunSaved').click();
    await response;
    await page.waitForFunction(()=>!document.querySelector('#southernRunSaved').disabled&&
      document.querySelector('#southernStatus').textContent.startsWith('南方规则：'),{},{timeout:900000});
    browserWall=(performance.now()-start)/1000;
    // Large responses can exceed Chromium's inspector body cache. Read the
    // server's stored result only after recording the actual rendered wait.
    result=(await api('southern_market')).latest;
    assert.ok(result,'The completed browser result must exist in the session');
  } else result=await api('run_southern_market',{revision:saved.revision});
  const wall=browserWall??(performance.now()-start)/1000;
  clearTimeout(deadline);
  if(page) {
    await page.screenshot({path:`${runOut}/desktop.png`,fullPage:true});
    await page.setViewportSize({width:390,height:844});
    await page.screenshot({path:`${runOut}/mobile.png`,fullPage:true});
    assert.equal(browserErrors.length,0,JSON.stringify(browserErrors));
  }
  await writeFile(`${runOut}/result.json`,JSON.stringify(result));
  const fields=['solver_status','runtime_sec','solve_wall_sec','assembly_sec','audit_sec','solution_export_sec',
    'variables','binary_variables','equalities','inequalities','nonzeros','objective','max_residual','mip_gap',
    'model_size','primal_start','solver_timing','price_consistency','gap_certificate','lp_algorithm','projected_commitment_units','compact_storage','assembly_template'];
  const report={binary_sha256:createHash('sha256').update(await readFile(exe)).digest('hex'),wall_sec:wall,
    wall_scope:page?'browser click through result rendering':'HTTP request through body parsing',browser_errors:browserErrors,
    process_run:run+1,process_state:run===0?'fresh_server':'same_session_repeat',
    status:result.status,schedule_feasible:result.schedule_feasible,prices_valid:result.prices_valid,
    execution:boundary.execution,entities:Object.fromEntries(['buses','branches','generators','reservoirs','storage'].map(k=>[k,boundary[k].length])),
    stages:Object.fromEntries(['scuc','sced','lmp'].filter(k=>result[k]).map(k=>[k,Object.fromEntries(fields.filter(f=>f in result[k]).map(f=>[f,result[k][f]]))]))};
  await writeFile(`${runOut}/report.json`,JSON.stringify(report,null,2));
  reports.push({directory:runOut,wall_sec:wall,process_run:run+1,process_state:report.process_state,
    schedule_feasible:result.schedule_feasible,prices_valid:result.prices_valid});
  await writeFile(`${out}/series.json`,JSON.stringify({reports,max_wall_sec:Math.max(...reports.map(r=>r.wall_sec))},null,2));
  console.log(JSON.stringify(reports.at(-1)));
  if(!result.schedule_feasible||!result.prices_valid)process.exitCode=1;
  }
} finally {
  clearTimeout(deadline);
  await browser?.close();
  if(server.exitCode===null&&server.signalCode===null){const stopped=once(server,'exit');server.kill();await stopped;}
  await log.close();
}
