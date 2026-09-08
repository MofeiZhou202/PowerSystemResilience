// Alternating matched runs; protocol: southern_execution_contract.md.
import { spawnSync } from 'node:child_process';
import { mkdirSync, readFileSync, writeFileSync, openSync, closeSync, statSync } from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import assert from 'node:assert/strict';

const directory=process.argv[2]||'output/market-operation/row-presolve';
mkdirSync(directory,{recursive:true});
const report={case:'118',solver_seconds:30,gap:.01,threads:{gurobi:4,highs:0},
  host:{platform:os.platform(),arch:os.arch(),cpus:os.cpus().length},runs:[]};
for(const solver of ['gurobi','highs'])for(let pair=0;pair<3;pair++) {
  const runs=[];
  for(const mode of pair%2?['enabled','none']:['none','enabled']) {
    const file=path.resolve(directory,`${solver}-${pair}-${mode}.json`),fd=openSync(file+'.log','w');
    const start=Date.now();let processResult;
    try { processResult=spawnSync('build/macos-release/tests/run_southern_market_benchmark',
      ['118',file,'30','compact','auto',solver,'-1','solve',mode],{stdio:['ignore',fd,fd],timeout:120000,killSignal:'SIGKILL'}); }
    finally { closeSync(fd); }
    assert.equal(processResult.error,undefined);assert.equal(processResult.status,0,JSON.stringify(processResult));
    assert.ok(statSync(file).mtimeMs>=start,'fresh artifact');
    const evidence=JSON.parse(readFileSync(file,'utf8'));
    assert.equal(evidence.prices_valid,true);
    for(const s of ['scuc','sced','lmp'])assert.ok(evidence[s].max_residual<=1e-6);
    assert.equal(evidence.scuc.model_size.constraints_removed,mode==='enabled'?7022:0);
    assert.equal(evidence.lmp.model_size.constraints_removed,0);
    const run={solver,pair,mode,wall_sec:(Date.now()-start)/1000,evidence};
    runs.push(run);report.runs.push(run);
    writeFileSync(path.join(directory,'comparison.json'),JSON.stringify(report,null,2)+'\n');
    console.log(JSON.stringify({solver,pair,mode,wall:run.wall_sec,rows_removed:evidence.scuc.model_size.constraints_removed}));
  }
  for(const stage of ['scuc','sced']) {
    const a=runs[0].evidence[stage].objective,b=runs[1].evidence[stage].objective;
    assert.ok(Math.abs(a-b)<=.01*Math.max(1,Math.abs(a)),'matched requested optimality tolerance');
  }
}
const median = values => values.sort((a,b)=>a-b)[Math.floor(values.length/2)];
report.medians={};
for(const solver of ['gurobi','highs']) {
  report.medians[solver]={};
  for(const mode of ['none','enabled'])report.medians[solver][mode]=median(report.runs.filter(r=>r.solver===solver&&r.mode===mode).map(r=>r.wall_sec));
}
writeFileSync(path.join(directory,'comparison.json'),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report.medians));
