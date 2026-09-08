// Reproduce price repeatability independently of counterfactual execution.
import {spawn} from 'node:child_process';
import {createServer} from 'node:net';
import {once} from 'node:events';
import {readFile,writeFile,mkdir,open} from 'node:fs/promises';
import assert from 'node:assert/strict';
const output='output/market-performance/recovery-lmp-repeat';
await mkdir(output,{recursive:true});
const full=JSON.parse(await readFile('output/market-performance/recovery-week-full/job.json'));
const boundary=JSON.parse(await readFile('output/market-performance/live-input/southern_market.json')).boundary;
const day=full.scenarios[0].days[4],config=structuredClone(full.scenarios[0].config);
for(const [table,rows] of Object.entries(day.state_start))for(const state of rows)
  Object.assign(boundary[table].find(row=>row.id===state.id),state);
config.horizon='day';config.start_date='2026-09-12';config.days=config.days.slice(4,6);config.explain=false;
delete config.reference_days;
boundary.execution.assembly_mode='verify';
const port=await new Promise(resolve=>{const s=createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});});
const log=await open(`${output}/solver.log`,'w');
const server=spawn('output/market-performance/recovery-final/run_gui_server',['--host','127.0.0.1','--port',String(port),'--data-dir','data'],
  {env:{...process.env,MIPSOLVERS_GUROBI_VERBOSE:'1'},stdio:['ignore',log.fd,log.fd]});
await log.close();
const base=`http://127.0.0.1:${port}`;
const api=async(route,body)=>{const r=await fetch(`${base}/api/session/${route}`,body?{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(body)}:{});const j=await r.json();assert.ok(r.ok,JSON.stringify(j));return j;};
try {
  for(let i=0;i<100;i++){try{if((await fetch(`${base}/api/cases`)).ok)break;}catch{}await new Promise(r=>setTimeout(r,100));}
  const current=await api('southern_market');
  const saved=await api('southern_market',{action:'save',revision:current.revision,boundary});
  await writeFile(`${output}/input.json`,JSON.stringify({boundary,config}));
  const results=[];
  for(let run=0;run<8;run++) {
    const started=await api('market_operation',{action:'start',revision:saved.revision,config});
    const solved=await api('market_operation',{action:'step',run_id:started.run_id,day:0});
    const d=solved.job.days[0];assert.equal(d.valid,true);
    for(const key of ['resources','state_start','state_end','lines','periods'])assert.deepEqual(d[key],day[key],key);
    for(const s of Object.values(d.stages))assert.equal(s.assembly_template.matrix_comparison,'exact_match');
    let price_delta=0;for(let n=0;n<d.nodes.length;n++)for(let t=0;t<96;t++)price_delta=Math.max(price_delta,Math.abs(d.nodes[n].lmp_per_mwh[t]-day.nodes[n].lmp_per_mwh[t]));
    await writeFile(`${output}/day-${run}.json`,JSON.stringify(d));
    results.push({run,price_delta,objective:d.stages.lmp.objective,residual:d.stages.lmp.max_residual,physical_state_exact:true,matrix_reference_exact:true});
    console.log(JSON.stringify(results.at(-1)));
  }
  await writeFile(`${output}/report.json`,JSON.stringify(results,null,2));
} finally {if(server.exitCode===null){const stopped=once(server,'exit');server.kill();await stopped;}}
