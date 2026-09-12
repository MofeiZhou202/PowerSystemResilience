import assert from 'node:assert/strict';
import os from 'node:os';
import { readFile, writeFile } from 'node:fs/promises';
const [before,after]=process.argv.slice(2);
assert.ok(before&&after,'Usage: node compare_market_replay.mjs BEFORE AFTER');
const read=async(dir,file)=>JSON.parse(await readFile(`${dir}/${file}.json`,'utf8'));
const a=await read(before,'job'),b=await read(after,'job');
const ar=await read(before,'report'),br=await read(after,'report');
assert.deepEqual(a.config,b.config);assert.equal(a.status,'completed');assert.equal(b.status,'completed');
let maxResidual=0,maxObjectiveDifference=0,stages=0,experiments=0;
const checkStages=(x,y)=>{
  for(const stage of ['scuc','sced','lmp']) {
    const u=x[stage],v=y[stage];
    assert.equal(!!u,!!v,`${stage} presence differs`);
    if(!u)continue;
    for(const key of ['requested_solver','requested_mip_gap','requested_time_limit_sec'])assert.equal(u[key],v[key]);
    for(const s of [u,v]){assert.ok(s.max_residual<=1e-6);maxResidual=Math.max(maxResidual,s.max_residual);}
    const diff=Math.abs(u.objective-v.objective)/Math.max(1,Math.abs(u.objective),Math.abs(v.objective));
    assert.ok(diff<=.01,`${stage} objective difference ${diff}`);
    maxObjectiveDifference=Math.max(maxObjectiveDifference,diff);stages++;
  }
};
assert.equal(a.scenarios.length,b.scenarios.length);
for(let i=0;i<a.scenarios.length;++i) {
  const x=a.scenarios[i],y=b.scenarios[i];
  assert.deepEqual(x.config,y.config);assert.equal(x.days.length,7);assert.equal(y.days.length,7);
  for(let d=0;d<7;++d) {
    const p=x.days[d],q=y.days[d];assert.equal(p.valid,true);assert.equal(q.valid,true);
    assert.equal(q.recovery_execution.workers,Math.min(6,q.recovery_execution.experiments,Math.floor(os.cpus().length/2)));
    assert.equal(q.stages.scuc.projected_commitment_units,48);
    assert.equal(q.stages.scuc.binary_variables,1920);
    if(d)assert.deepEqual(q.state_start,y.days[d-1].state_end);
    checkStages(p.stages,q.stages);
    assert.equal(p.counterfactuals.length,q.counterfactuals.length);
    for(let k=0;k<p.counterfactuals.length;++k){
      const u=p.counterfactuals[k],v=q.counterfactuals[k];
      assert.equal(u.factor,v.factor);assert.equal(u.valid,true);assert.equal(v.valid,true);
      assert.deepEqual(u.reference_value,v.reference_value);assert.deepEqual(u.sampled_value,v.sampled_value);
      checkStages(u.stages,v.stages);experiments++;
    }
  }
}
const improvement=1-br.wall_sec/ar.wall_sec;assert.ok(improvement>=.25);
const report={passed:true,before,after,old_browser_wall_sec:ar.wall_sec,new_browser_wall_sec:br.wall_sec,
  reduction_fraction:improvement,scenarios:b.scenarios.length,days:b.completed_days,experiments,
  stages_checked:stages,max_residual:maxResidual,max_relative_objective_difference:maxObjectiveDifference,
  limitations:['One matched complete-week browser replay per version; not a multi-machine throughput guarantee.',
    'Canonical commitment and degenerate dispatch can differ; each version satisfies the original model and existing 1% solve tolerance.']};
await writeFile(`${after}/comparison.json`,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
