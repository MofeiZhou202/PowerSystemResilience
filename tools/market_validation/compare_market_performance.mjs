// Full-model acceptance protocol from docs/modules/market/performance.md.
import assert from 'node:assert/strict';
import { readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
const [beforeDir, afterDir] = process.argv.slice(2);
assert.ok(beforeDir && afterDir, 'Usage: node compare_market_performance.mjs BEFORE AFTER');
const read = async (dir, file) => JSON.parse(await readFile(path.join(dir,file),'utf8'));
const before = await read(beforeDir,'report.json'), after = await read(afterDir,'report.json');
assert.equal(before.runs.length,3); assert.equal(after.runs.length,3);
assert.deepEqual(before.host,after.host); assert.equal(before.seconds,after.seconds);
assert.deepEqual(await read(beforeDir,'boundary.json'),await read(afterDir,'boundary.json'));
let maxResidual = 0, maxRelativeObjectiveDifference = 0;
let stagesChecked = 0;
const compareStages = (a,b) => {
  for (const stage of ['scuc','sced','lmp']) {
    const x=a[stage],y=b[stage];
    assert.equal(x.solver_status,y.solver_status);
    for (const field of ['requested_solver','requested_threads','requested_mip_gap','requested_time_limit_sec']) assert.equal(x[field],y[field]);
    for (const value of [x.max_residual,y.max_residual]) {
      assert.ok(Number.isFinite(value) && value <= 1e-6); maxResidual=Math.max(maxResidual,value);
    }
    const relative=Math.abs(x.objective-y.objective)/Math.max(1,Math.abs(x.objective),Math.abs(y.objective));
    assert.ok(relative <= .01); maxRelativeObjectiveDifference=Math.max(maxRelativeObjectiveDifference,relative);
    stagesChecked++;
  }
};
for (let i=0;i<3;++i) {
  const x=before.runs[i],y=after.runs[i];
  assert.equal(x.variant,'full');assert.equal(y.variant,'full');
  assert.equal(x.repeat,y.repeat);assert.equal(x.solver,y.solver);
  assert.equal(x.status,'completed');assert.equal(y.status,'completed');
  assert.equal(x.feasible,true);assert.equal(y.feasible,true);
  assert.equal(y.execution.workers,2);assert.equal(y.counterfactuals.length,6);
  compareStages(x.stages,y.stages);
  assert.equal(x.counterfactuals.length,y.counterfactuals.length);
  for (let k=0;k<x.counterfactuals.length;++k) {
    const a=x.counterfactuals[k],b=y.counterfactuals[k];
    assert.equal(a.factor,b.factor);assert.equal(a.valid,true);assert.equal(b.valid,true);
    compareStages(a.stages,b.stages);
  }
}
const median = runs => runs.map(r=>r.wall_sec).sort((a,b)=>a-b)[1];
const baseline=median(before.runs),optimized=median(after.runs),reduction=1-optimized/baseline;
assert.ok(reduction >= .25, `Median reduction ${reduction} below predeclared 25% threshold`);
const evidence={passed:true,before:beforeDir,after:afterDir,baseline_median_sec:baseline,
  optimized_median_sec:optimized,reduction_fraction:reduction,acceptance_fraction:.25,
  ideal_prediction_fraction:.425,stages_checked:stagesChecked,max_residual:maxResidual,
  max_relative_objective_difference:maxRelativeObjectiveDifference,
  scope:'Three repeated synthetic IEEE118 days, each with six complete paired interventions; identical input boundary and solver quality settings. Residuals here are backend audits; independent physics are checked by market_ieee118_resources_e2e.'};
await writeFile(path.join(afterDir,'comparison.json'),JSON.stringify(evidence,null,2)+'\n');
console.log(JSON.stringify(evidence,null,2));
