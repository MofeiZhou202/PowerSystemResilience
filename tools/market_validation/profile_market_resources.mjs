// Diagnostic ablations change feasible sets; only full-model runs are performance baselines.
import { spawn, execFileSync } from 'node:child_process';
import { createServer } from 'node:net';
import { once } from 'node:events';
import { mkdir, writeFile, readFile } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import os from 'node:os';
import path from 'node:path';
const option = (name, fallback) => { const i = process.argv.indexOf(name); return i < 0 ? fallback : process.argv[i + 1]; };
const exe = option('--server', 'build/macos-release/tests/run_gui_server');
const out = option('--output', 'output/market-performance/resources');
const repetitions = Number(option('--repetitions', '3'));
const solvers = option('--solvers', 'gurobi,highs').split(',');
const seconds = Number(option('--seconds', '30'));
const recovery = process.argv.includes('--recovery');
const port = await new Promise(resolve => { const s = createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); }); });
const server = spawn(exe, ['--host','127.0.0.1','--port',String(port),'--data-dir','data'], { stdio: 'ignore' });
const base = `http://127.0.0.1:${port}`;
const api = async (route, body) => {
  const response = await fetch(`${base}/api/session/${route}`, body ? { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(body), signal: AbortSignal.timeout(240000) } : {});
  const data = await response.json(); if (!response.ok) throw new Error(JSON.stringify(data)); return data;
};
const report = { host: { platform: os.platform(), arch: os.arch(), cpu: os.cpus()[0].model },
  commit: execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),
  binary_sha256: createHash('sha256').update(await readFile(exe)).digest('hex'),
  seconds, repetitions, runs: [], limitations: ['Synthetic IEEE118. Ablation timings are nonlinear interactions, not additive attribution or production optimizations. Fixed commitment uses a preceding feasible schedule.'] };
try {
  await mkdir(out, { recursive: true });
  for (let n=0;n<100;n++) { if(server.exitCode!==null || server.signalCode!==null)throw new Error('Benchmark server exited during startup');try { if ((await fetch(base+'/api/cases')).ok) break; } catch {} await new Promise(r=>setTimeout(r,100)); }
  const loaded = await api('southern_market', { action:'ieee118_mixed', revision:0 });
  const boundary = loaded.boundary;
  await writeFile(path.join(out,'boundary.json'),JSON.stringify(boundary));
  for (const solver of solvers) {
    let schedule;
    for (let repeat=0;repeat<repetitions;repeat++) {
      const variants = option('--variants', repeat % 2 ? 'full,fixed_thermal,fixed_hydro,no_storage,no_water' : 'full,no_water,no_storage,fixed_hydro,fixed_thermal').split(',');
      for (const variant of variants) {
        const b = structuredClone(boundary);
        Object.assign(b.execution,{solver,threads:solver==='gurobi'?2:0,time_limit_sec:seconds,mip_gap:.01,ac_security:'schedule_only',balance_policy:'diagnostic',balance_penalty_per_mwh:100000});
        if (variant==='no_water') b.reservoirs=[];
        if (variant==='no_storage') b.storage=[];
        if (variant.startsWith('fixed_')) {
          if (!schedule) continue;
          for (const g of b.generators) if (g.kind===variant.slice(6)) {
            const values=schedule.find(r=>r.id===g.id).online;
            g.must_on=values.map(v=>Math.round(v)); g.must_off=values.map(v=>1-Math.round(v));
          }
        }
        const current=await api('southern_market');
        const saved=await api('southern_market',{action:'save',revision:current.revision,boundary:b});
        if (recovery) {
          const config = {horizon:'day',start_date:'2026-09-07',explain:true,penalty_per_mwh:100000,days:[]};
          let job=await api('market_operation',{action:'start',revision:saved.revision,config});
          const resolved=job.job.config;
          Object.assign(resolved.days[0],{load_scale:1.1,wind_scale:.9,solar_scale:.9,inflow_scale:.9,generator_bid_scale:1.1,load_bid_scale:1.1});
          job=await api('market_operation',{action:'start',revision:saved.revision,config:resolved});
          const start=performance.now();
          job=await api('market_operation',{action:'step',run_id:job.run_id,day:0});
          const day=job.job.days[0];
          const run={solver,repeat,variant,wall_sec:(performance.now()-start)/1000,status:job.job.status,feasible:day?.valid,
            execution:day?.recovery_execution,main_sec:day?.runtime_sec,stages:day?.stages,
            counterfactuals:day?.counterfactuals.map(c=>({factor:c.factor,valid:c.valid,stages:c.stages}))};
          report.runs.push(run);
          await writeFile(path.join(out,`${solver}-${repeat}-${variant}.json`),JSON.stringify(day));
          await writeFile(path.join(out,'report.json'),JSON.stringify(report,null,2));
          console.log(JSON.stringify({...run,stages:undefined,counterfactuals:run.counterfactuals.map(c=>({factor:c.factor,valid:c.valid}))}));
          continue;
        }
        const start=performance.now();
        const result=await api('run_southern_market',{revision:saved.revision});
        if(variant==='full' && result.schedule_feasible) schedule=result.scuc.generators;
        const run={solver,repeat,variant,wall_sec:(performance.now()-start)/1000,status:result.status,feasible:result.schedule_feasible,prices_valid:result.prices_valid,stages:{}};
        for(const stage of ['scuc','sced','lmp']) if(result[stage]) run.stages[stage]=Object.fromEntries(
          ['runtime_sec','assembly_sec','audit_sec','variables','binary_variables','nonzeros','objective','max_residual','mip_gap','solution_quality','model_size'].filter(k=>k in result[stage]).map(k=>[k,result[stage][k]]));
        report.runs.push(run);
        await writeFile(path.join(out,'report.json'),JSON.stringify(report,null,2));
        console.log(JSON.stringify({...run,stages:Object.fromEntries(Object.entries(run.stages).map(([k,v])=>[k,{sec:v.runtime_sec,assembly:v.assembly_sec,binary:v.binary_variables,quality:v.solution_quality}]))}));
      }
    }
  }
} finally { if(server.exitCode===null && server.signalCode===null){ const stopped=once(server,'exit');server.kill('SIGTERM');await stopped; } }
