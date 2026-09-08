// Compare declared scopes without treating missing experiments as zero outcomes.
import {readFile,writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
const [baselinePath,dispatchPath,autoPath,verifyPath]=process.argv.slice(2);
const read=async(p,f='job.json')=>JSON.parse(await readFile(`${p}/${f}`,'utf8'));
const baseline=await read(baselinePath),dispatch=await read(dispatchPath),auto=await read(autoPath),verify=await read(verifyPath);
let objectiveDelta=0,residualDelta=0,stages=0;
const prices={};
function compareStages(a,b) {
  for(const [name,stage] of Object.entries(b.stages)) {
    const expected=a.stages[name];assert.ok(expected);
    const objective=Math.abs(stage.objective-expected.objective),residual=Math.abs(stage.max_residual-expected.max_residual);
    objectiveDelta=Math.max(objectiveDelta,objective);residualDelta=Math.max(residualDelta,residual);stages++;
    assert.ok(objective<=1e-6,`objective ${name}: ${objective}`);
    assert.ok(residual<=1e-6,`residual ${name}: ${residual}`);
    assert.ok(stage.max_residual<=1e-6);
    if(stage.solver_timing) {
      const t=stage.solver_timing;
      for(const k of ['environment_sec','model_import_sec','optimize_sec','result_extract_sec'])assert.ok(Number.isFinite(t[k])&&t[k]>=0,k);
      assert.equal(t.presolve_sec,null);assert.equal(t.search_sec,null);
      assert.ok(t.environment_sec+t.model_import_sec+t.optimize_sec+t.result_extract_sec<=stage.solve_wall_sec+.001);
    }
  }
}
for(const candidate of [dispatch,auto,verify]) {
  const mode=candidate===dispatch?'dispatch':candidate===auto?'auto':'verify';
  prices[mode]={exact:true,max_delta_per_mwh:0};
  assert.equal(candidate.completed_days,baseline.completed_days);
  assert.deepEqual(candidate.statistics,baseline.statistics);
  for(let s=0;s<baseline.scenarios.length;s++)for(let d=0;d<7;d++) {
    const a=baseline.scenarios[s].days[d],b=candidate.scenarios[s].days[d];
    for(const k of ['resources','lines','periods','state_start','state_end','lookahead','deficit_mwh','surplus_mwh','overload_mwh'])assert.deepEqual(b[k],a[k],`main ${s}/${d}/${k}`);
    assert.equal(b.nodes.length,a.nodes.length);
    for(let n=0;n<a.nodes.length;n++) {
      const {lmp_per_mwh:x,...physicalA}=a.nodes[n],{lmp_per_mwh:y,...physicalB}=b.nodes[n];
      assert.deepEqual(physicalA,physicalB);assert.equal(x.length,y.length);
      for(let t=0;t<x.length;t++) {
        if(x[t]!==y[t])prices[mode].exact=false;
        prices[mode].max_delta_per_mwh=Math.max(prices[mode].max_delta_per_mwh,Math.abs(x[t]-y[t]));
      }
    }
    compareStages(a,b);
    if(candidate===dispatch) {
      assert.equal(b.counterfactuals.length,a.counterfactuals.length);
      for(let k=0;k<a.counterfactuals.length;k++) {
        const x=a.counterfactuals[k],y=b.counterfactuals[k];
        assert.equal(y.valid,true);assert.equal(y.prices_valid,false);assert.equal(y.stages.lmp,undefined);
        for(const field of ['factor','reference_value','sampled_value','periods','reduction_deficit_mwh','reduction_surplus_mwh','reduction_overload_mwh'])assert.deepEqual(y[field],x[field]);
        compareStages(x,y);
      }
    } else {
      assert.equal(b.cause_analysis.status,'not_triggered');assert.equal(b.counterfactuals.length,0);
      if(candidate===verify)for(const stage of Object.values(b.stages))assert.equal(stage.assembly_template.matrix_comparison,'exact_match');
    }
  }
}
const reports=await Promise.all([baselinePath,dispatchPath,autoPath].map(p=>read(p,'report.json')));
const timers={};
for(const d of auto.scenarios.flatMap(s=>s.days))for(const [stage,s] of Object.entries(d.stages)) {
  const sums=timers[stage]||={count:0,solve_wall_sec:0,assembly_sec:0,environment_sec:0,model_import_sec:0,optimize_sec:0,result_extract_sec:0};sums.count++;
  for(const k of Object.keys(sums).filter(k=>k!=='count'))sums[k]+=s[k]??s.solver_timing[k];
}
const result={main_resources_water_soc_exact:true,main_prices:prices,recovery_metrics_exact:true,main_verify_matrices_exact:true,
  compared_stages:stages,max_objective_delta:objectiveDelta,max_residual_delta:residualDelta,
  wall_sec:reports.map(r=>r.wall_sec),auto_reduction_percent:100*(1-reports[2].wall_sec/reports[0].wall_sec),
  dispatch_reduction_percent:100*(1-reports[1].wall_sec/reports[0].wall_sec),
  predictions:{auto_range_sec:[38,45],dispatch_sec:139},acceptance:{auto:reports[2].wall_sec<=50,dispatch:reports[1].wall_sec<=145,strict_price_identity:Object.values(prices).every(p=>p.exact)},main_solver_timing_sum:timers};
await writeFile(`${autoPath}/comparison.json`,JSON.stringify(result,null,2));
console.log(JSON.stringify(result,null,2));
// Keep the original exact-price gate visibly failed; do not relax its tolerance.
if(!result.acceptance.strict_price_identity)process.exitCode=1;
