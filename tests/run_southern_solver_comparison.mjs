// Independent process deadlines complement solver budgets (LP calls may overrun).
// Protocol and interpretation: docs/modules/market/southern_execution_contract.md.
import { spawn, execFileSync } from 'node:child_process';
import { mkdir, readFile, writeFile, stat } from 'node:fs/promises';
import { createWriteStream } from 'node:fs';
import path from 'node:path';
import os from 'node:os';

const [kind='demo', seconds='30', wallSeconds='120', thermalLimit='-1', directory='output/market-operation/solver-comparison', selected='highs,native,gurobi'] = process.argv.slice(2);
if (![seconds,wallSeconds].every(v=>Number.isFinite(Number(v))&&Number(v)>0)) throw new Error('Positive solver and wall budgets required');
await mkdir(directory,{recursive:true});
const report={case:kind,seconds:Number(seconds),wall_limit_sec:Number(wallSeconds),thermal_limit:Number(thermalLimit),
  host:{platform:os.platform(),arch:os.arch(),cpus:os.cpus().length,memory_bytes:os.totalmem()},
  commit:execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),runs:[]};
for (const solver of selected.split(',')) {
  if (!['highs','native','gurobi'].includes(solver)) throw new Error('Unknown solver');
  const output=path.resolve(directory,`${kind}-${solver}.json`);
  const args=[kind,output,seconds,'compact','auto',solver,thermalLimit];
  const log=createWriteStream(path.join(directory,`${kind}-${solver}.log`));
  const started=Date.now(); let timeout=false,peakRss=0;
  const child=spawn('build/macos-release/tests/run_southern_market_benchmark',args,{stdio:['ignore','pipe','pipe']});
  child.stdout.pipe(log,{end:false});child.stderr.pipe(log,{end:false});
  const monitor=setInterval(()=>{
    try { const rss=Number(execFileSync('ps',['-o','rss=','-p',String(child.pid)],{encoding:'utf8',stdio:['ignore','pipe','ignore']}).trim())*1024; if(Number.isFinite(rss))peakRss=Math.max(peakRss,rss); } catch {}
  },1000);
  const deadline=setTimeout(()=>{timeout=true;child.kill('SIGKILL');},Number(wallSeconds)*1000);
  console.log(`${solver}: ${kind}, GAP=.01, solver ${seconds}s, process ${wallSeconds}s`);
  const result=await new Promise((resolve,reject)=>{child.once('error',reject);child.once('close',(code,signal)=>resolve({code,signal}));});
  clearTimeout(deadline);clearInterval(monitor);log.end();
  const run={solver,args,...result,outer_deadline_reached:timeout,wall_sec:(Date.now()-started)/1000,sampled_peak_rss_bytes:peakRss};
  // Never read an earlier completed artifact after terminating this invocation.
  if (!timeout && result.code!==null) {
    try { if((await stat(output)).mtimeMs<started)throw new Error('No new artifact');run.evidence=JSON.parse(await readFile(output,'utf8')); } catch(e) { run.artifact_error=e.message; }
  }
  report.runs.push(run);
  await writeFile(path.join(directory,'comparison.json'),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({solver,wall_sec:run.wall_sec,code:run.code,timeout,status:run.evidence?.status,scuc:run.evidence?.scuc}));
}
