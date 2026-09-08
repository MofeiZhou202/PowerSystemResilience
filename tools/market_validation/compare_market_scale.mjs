// Exact same-input parity; timing and assembly-path diagnostics are separate.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
const dirs=process.argv.slice(2);
assert.ok(dirs.length>=2,'Usage: compare_market_scale.mjs REFERENCE CANDIDATE [VERIFIED]');
const read=async(dir,name)=>JSON.parse(await readFile(`${dir}/${name}.json`,'utf8'));
const normalize=value=>{
  if(Array.isArray(value))return value.map(normalize);
  if(!value||typeof value!=='object')return value;
  return Object.fromEntries(Object.entries(value).filter(([k])=>!k.endsWith('_sec')&&k!=='assembly_template'&&k!=='assembly_mode')
    .map(([k,v])=>[k,normalize(v)]));
};
const baseline=await read(dirs[0],'result');
const boundary=await read(dirs[0],'boundary');
delete boundary.execution.assembly_mode;
let verifiedMatrices=0,maxResidual=0;
for(const dir of dirs.slice(1)) {
  const input=await read(dir,'boundary');
  const verify=input.execution.assembly_mode==='verify';
  delete input.execution.assembly_mode;
  assert.deepEqual(input,boundary,`${dir}: input`);
  const result=await read(dir,'result');
  assert.equal(result.schedule_feasible,true);
  assert.equal(result.prices_valid,true);
  assert.deepEqual(Object.keys(result).sort(),Object.keys(baseline).sort(),`${dir}: response fields`);
  for(const key of Object.keys(result))if(!['scuc','sced','lmp'].includes(key)&&!key.endsWith('_sec'))
    assert.deepEqual(normalize(result[key]),normalize(baseline[key]),`${dir}: ${key}`);
  for(const stage of ['scuc','sced','lmp']) {
    const a=baseline[stage],b=result[stage];
    assert.deepEqual(normalize(b),normalize(a),`${dir}: ${stage}, all non-timing fields`);
    assert.ok(Number.isFinite(b.max_residual)&&b.max_residual<=1e-6);
    maxResidual=Math.max(maxResidual,b.max_residual);
    if(verify) {
      assert.equal(b.assembly_template.matrix_comparison,'exact_match');
      verifiedMatrices+=2;
    }
  }
  assert.equal(result.lmp.price_consistency.passed,true);
  assert.equal(result.lmp.price_consistency.max_dual_difference,0);
}
const report={passed:true,directories:dirs,comparison:'complete response except timing and assembly-path diagnostics',
  physical_water_soc_and_prices:'exact_match',verified_matrices:verifiedMatrices,max_residual:maxResidual};
await writeFile(`${dirs[1]}/comparison.json`,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
