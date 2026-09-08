// Exact parity for the assembly-only change; performance gate is reported separately.
import assert from 'node:assert/strict';
import { readFile, writeFile } from 'node:fs/promises';
const [before, after, verified] = process.argv.slice(2);
assert.ok(before && after && verified, 'Usage: compare_market_assembly.mjs REFERENCE CACHED VERIFIED');
const read = async (dir, name) => JSON.parse(await readFile(`${dir}/${name}.json`, 'utf8'));
const jobs = await Promise.all([before, after, verified].map(dir => read(dir, 'job')));
const reports = await Promise.all([before, after, verified].map(dir => read(dir, 'report')));
let stages = 0, maxResidual = 0, reusedMatrices = 0, assemblyBefore = 0, assemblyAfter = 0;
let verifyObjectiveDifference = 0, verifyResidualDifference = 0;
// The compact job response omits base. Replay asserts its resolved exported base
// against the saved input before clicking Run; compare its input/config ledger here.
assert.equal(reports[0].assembly, 'reference');
assert.equal(reports[1].assembly, 'cached');
assert.equal(reports[2].assembly, 'verify');
for (const report of reports) assert.equal(report.input, reports[0].input);
for (const job of jobs) {
  assert.equal(job.status, 'completed'); assert.equal(job.completed_days, 7);
  assert.deepEqual(job.config, jobs[0].config);
  assert.equal(job.scenarios.length, 1);
}
const compareStages = (reference, cached, checked) => {
  for (const name of ['scuc', 'sced', 'lmp']) {
    const a = reference[name], b = cached[name], c = checked[name];
    for (const field of ['objective', 'max_residual', 'reconstructed_max_residual', 'variables', 'binary_variables',
      'nonzeros', 'model_size', 'requested_solver', 'requested_threads', 'requested_time_limit_sec', 'requested_mip_gap']) {
      assert.deepEqual(b[field], a[field], name + '/' + field);
      if (field === 'objective' || field === 'max_residual' || field === 'reconstructed_max_residual') {
        const difference = Math.abs(c[field] - a[field]);
        // Matrix identity remains exact. Report scalar identity separately from
        // the original 1e-6 numerical audit gate; do not hide last-bit differences.
        assert.ok(Number.isFinite(difference) && difference <= 1e-6, 'verify/' + name + '/' + field);
        if (field === 'objective') verifyObjectiveDifference = Math.max(verifyObjectiveDifference, difference);
        else verifyResidualDifference = Math.max(verifyResidualDifference, difference);
      } else assert.deepEqual(c[field], a[field], 'verify/' + name + '/' + field);
    }
    assert.equal(c.assembly_template.matrix_comparison, 'exact_match');
    for (const stage of [a, b, c]) {
      assert.ok(Number.isFinite(stage.max_residual) && stage.max_residual <= 1e-6);
      maxResidual = Math.max(maxResidual, stage.max_residual);
    }
    stages++; reusedMatrices += b.assembly_template.reused_matrices;
    assemblyBefore += a.assembly_sec; assemblyAfter += b.assembly_sec;
  }
};
for (let day = 0; day < 7; day++) {
  const [a, b, c] = jobs.map(job => job.scenarios[0].days[day]);
  for (const d of [a, b, c]) { assert.equal(d.valid, true); assert.equal(d.counterfactuals.length, 6); }
  for (const field of ['boundary', 'resources', 'periods', 'lookahead', 'state_start', 'state_end', 'nodes', 'lines']) {
    assert.deepEqual(b[field], a[field], `${day}/${field}`);
    assert.deepEqual(c[field], a[field], `verify/${day}/${field}`);
  }
  if (day) for (const job of jobs)
    assert.deepEqual(job.scenarios[0].days[day].state_start, job.scenarios[0].days[day - 1].state_end);
  compareStages(a.stages, b.stages, c.stages);
  for (let factor = 0; factor < 6; factor++) {
    const [x, y, z] = [a, b, c].map(d => d.counterfactuals[factor]);
    for (const field of ['factor', 'reference_value', 'sampled_value', 'valid', 'periods',
      'reduction_deficit_mwh', 'reduction_surplus_mwh', 'reduction_overload_mwh']) {
      assert.deepEqual(y[field], x[field], field); assert.deepEqual(z[field], x[field], field);
    }
    compareStages(x.stages, y.stages, z.stages);
  }
}
const result = {
  passed: true, before, after, verified, stages, max_residual: maxResidual,
  objectives_resources_and_carry: 'exact_match', verified_matrices: stages * 2, reused_matrices: reusedMatrices,
  verify_scalar_bitwise_equal: verifyObjectiveDifference === 0 && verifyResidualDifference === 0,
  verify_max_absolute_objective_difference: verifyObjectiveDifference,
  verify_max_absolute_residual_difference: verifyResidualDifference,
  browser_wall_before_sec: reports[0].wall_sec, browser_wall_after_sec: reports[1].wall_sec,
  browser_reduction_fraction: 1 - reports[1].wall_sec / reports[0].wall_sec,
  cumulative_assembly_before_sec: assemblyBefore, cumulative_assembly_after_sec: assemblyAfter,
  assembly_reduction_fraction: 1 - assemblyAfter / assemblyBefore,
  limitations: ['One complete-week run per mode. Concurrent worker assembly sums are not wall time.',
    'Verification mode builds both paths; its duration is not a production performance measurement.'],
};
await writeFile(`${after}/assembly-comparison.json`, JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result, null, 2));
