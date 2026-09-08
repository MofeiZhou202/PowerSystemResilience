// Root separation experiment; protocol: southern_execution_contract.md.
import {spawnSync} from 'node:child_process';
import {mkdirSync,readFileSync,writeFileSync,openSync,closeSync,statSync} from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import assert from 'node:assert/strict';

const directory=process.argv[2]||'output/market-operation/native-root-cuts';
mkdirSync(directory,{recursive:true});
const revision=directory=>spawnSync('git',['-C',directory,'rev-parse','HEAD'],{encoding:'utf8'}).stdout.trim();
const report={case:'118',seconds:30,gap:.01,build:'cached macos-release, dirty worktrees',
  revisions:{hysim:revision('.'),mipsolvers:revision('../MIPSolvers')},
  host:{platform:os.platform(),arch:os.arch(),cpus:os.cpus().length,cpu_model:os.cpus()[0]?.model},runs:[]};
for(let pair=0;pair<2;pair++)for(const profile of pair%2?['enhanced','default']:['default','enhanced']) {
  const file=path.resolve(directory,`${pair}-${profile}.json`),fd=openSync(file+'.log','w');
  const started=Date.now();let result;
  try { result=spawnSync('build/macos-release/tests/run_southern_market_benchmark',
    ['118',file,'30','compact','auto','native','-1','solve','none',profile],
    {env:{...process.env,MIPSOLVERS_BC_TIMELINE:'1',MIPSOLVERS_HIGHS_LP_KERNEL_TRACE:'1'},
      stdio:['ignore',fd,fd],timeout:120000,killSignal:'SIGKILL'}); }
  finally {closeSync(fd);}
  const run={pair,profile,process_status:result.status,error:result.error?.message||null,wall_sec:(Date.now()-started)/1000};
  if(!result.error && statSync(file).mtimeMs>=started) {
    run.evidence=JSON.parse(readFileSync(file,'utf8'));
    const e=run.evidence;
    assert.equal(e.scuc.native_diagnostics.profile,profile);
    assert.equal(e.scuc.native_diagnostics.root_row_admission_limit,profile==='enhanced'?100000:35000);
    if(e.schedule_feasible)for(const s of ['scuc','sced'])assert.ok(e[s].max_residual<=1e-6);
    if(e.prices_valid)assert.ok(e.lmp.max_residual<=1e-6);
  }
  report.runs.push(run);
  writeFileSync(path.join(directory,'comparison.json'),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({pair,profile,wall_sec:run.wall_sec,error:run.error,quality:run.evidence?.scuc?.solution_quality,diagnostics:run.evidence?.scuc?.native_diagnostics}));
}
