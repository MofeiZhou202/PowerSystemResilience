// Unchanged-model parameter experiment; protocol: docs/modules/market/performance.md.
import assert from 'node:assert/strict';
import { spawn, execFileSync } from 'node:child_process';
import { createServer } from 'node:net';
import { once } from 'node:events';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import os from 'node:os';

const option = (key, fallback) => process.argv.includes(key) ? process.argv[process.argv.indexOf(key) + 1] : fallback;
const exe = option('--server', 'output/market-performance/commitment-overlay/run_gui_server');
const input = option('--input', 'output/market-performance/live-input');
const out = option('--output', 'output/market-performance/parameters');
const repetitions = Number(option('--repetitions', '3'));
assert.ok(Number.isInteger(repetitions) && repetitions > 0);
const variants = {
  reference: { assembly_mode: 'reference' }, verify: { assembly_mode: 'verify' },
  auto: {}, start: { mip_start: 'enabled' }, dual: { gurobi_method: 'dual_simplex' },
  barrier: { gurobi_method: 'barrier' }, rows: { row_presolve: 'enabled' }, threads2: { threads: 2 },
};
const names = option('--variants', 'auto,start,dual,barrier,rows,threads2').split(',');
for (const name of names) assert.ok(Object.hasOwn(variants, name), name);
assert.ok(names.includes('auto'), 'The unchanged baseline is required');
const boundaryText = await readFile(`${input}/southern_market.json`, 'utf8');
const forecastText = await readFile(`${input}/market_forecast.json`, 'utf8');
const boundary = JSON.parse(boundaryText).boundary;
const config = JSON.parse(forecastText).job.scenarios[0].config;
// Keep the saved week configuration for identical next-day representative points.
// Only execute day zero; recovery is separately covered by the complete-week replay.
config.explain = false;
const hash = data => createHash('sha256').update(data).digest('hex');
const report = {
  host: { cpu: os.cpus()[0].model, logical_cpus: os.cpus().length, platform: os.platform(), arch: os.arch() },
  commit: execFileSync('git', ['rev-parse', 'HEAD'], { encoding: 'utf8' }).trim(),
  binary_sha256: hash(await readFile(exe)), input_sha256: { boundary: hash(boundaryText), forecast: hash(forecastText) },
  config, variants, repetitions, runs: [],
  acceptance: { minimum_reduction: .1, residual: 1e-6, relative_objective_difference: .01 },
  limitations: ['Saved first-day main chain only; no recovery or cross-day performance claim. Solver runtime is adapter-reported, not a disjoint wall-clock profile. Degenerate dispatch and prices may differ within the existing GAP.'],
};
const port = await new Promise(resolve => {
  const s = createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); });
});
const server = spawn(exe, ['--host', '127.0.0.1', '--port', String(port), '--data-dir', 'data'], { stdio: 'ignore' });
const base = `http://127.0.0.1:${port}`;
const api = async (route, body) => {
  const response = await fetch(`${base}/api/session/${route}`, body ? {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
    signal: AbortSignal.timeout(600000),
  } : {});
  const data = await response.json(); assert.ok(response.ok, JSON.stringify(data)); return data;
};
const median = values => [...values].sort((a, b) => a - b)[Math.floor(values.length / 2)];
try {
  await mkdir(out, { recursive: true });
  let ready = false;
  for (let k = 0; k < 100; k++) {
    assert.ok(server.exitCode === null && server.signalCode === null, 'Server exited');
    try { if ((await fetch(base + '/api/cases')).ok) { ready = true; break; } } catch {}
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  assert.ok(ready, 'Server startup timed out');
  for (let repeat = 0; repeat < repetitions; repeat++) {
    const ordered = repeat % 2 ? [...names].reverse() : names;
    for (const variant of ordered) {
      const b = structuredClone(boundary);
      Object.assign(b.execution, config.solver_options, variants[variant]);
      const current = await api('southern_market');
      const saved = await api('southern_market', { action: 'save', revision: current.revision, boundary: b });
      const runConfig = structuredClone(config);
      // Operation exposes a smaller override schema; other options come from base.execution.
      for (const [key, value] of Object.entries(variants[variant]))
        if (Object.hasOwn(runConfig.solver_options, key)) runConfig.solver_options[key] = value;
      const initialized = await api('market_operation', { action: 'start', revision: saved.revision, config: runConfig });
      const begin = performance.now();
      const result = await api('market_operation', { action: 'step', run_id: initialized.run_id, day: 0 });
      const wall = (performance.now() - begin) / 1000;
      const day = result.job.days[0];
      await writeFile(`${out}/${repeat}-${variant}.json`, JSON.stringify(day));
      const run = { repeat, variant, wall_sec: wall, runtime_sec: day.runtime_sec, valid: day.valid,
        prices_valid: day.diagnostic_prices_valid, execution: b.execution, timing: day.execution_timing, stages: day.stages };
      report.runs.push(run);
      await writeFile(`${out}/report.json`, JSON.stringify(report, null, 2));
      console.log(JSON.stringify({ repeat, variant, wall_sec: wall, valid: day.valid,
        stages: Object.fromEntries(Object.entries(day.stages).map(([name, s]) => [name, { solve: s.runtime_sec, assembly: s.assembly_sec, audit: s.audit_sec, start: s.primal_start }])) }));
      assert.equal(day.valid, true); assert.equal(day.diagnostic_prices_valid, true);
      assert.deepEqual(Object.keys(day.stages).sort(), ['lmp', 'sced', 'scuc']);
      for (const s of Object.values(day.stages)) assert.ok(Number.isFinite(s.max_residual) && s.max_residual <= 1e-6);
      for (const [stage, s] of Object.entries(day.stages)) {
        assert.equal(s.requested_time_limit_sec, b.execution.time_limit_sec);
        assert.equal(s.requested_mip_gap, b.execution.mip_gap);
        assert.equal(s.requested_threads, b.execution.threads);
        // Pricing retains authored rows for dual recovery (build_model::prune_rows).
        if (variant === 'rows') assert.equal(s.model_size.row_presolve, stage === 'lmp' ? 'none' : 'enabled');
        if (variant === 'dual' || variant === 'barrier') assert.equal(s.lp_algorithm, variants[variant].gurobi_method);
      }
      if (variant === 'start') assert.notEqual(day.stages.scuc.primal_start.status, 'not_requested');
      if (variant === 'verify') for (const stage of Object.values(day.stages))
        assert.equal(stage.assembly_template.matrix_comparison, 'exact_match');
    }
  }
  const baseline = report.runs.filter(r => r.variant === 'auto');
  report.summary = names.map(variant => {
    const runs = report.runs.filter(r => r.variant === variant);
    const seconds = median(runs.map(r => r.wall_sec));
    let objectiveDifference = 0;
    for (const run of runs) for (const stage of ['scuc', 'sced', 'lmp']) {
      const ref = baseline.find(b => b.repeat === run.repeat).stages[stage].objective;
      objectiveDifference = Math.max(objectiveDifference, Math.abs(run.stages[stage].objective - ref) / Math.max(1, Math.abs(ref)));
    }
    const reduction = 1 - seconds / median(baseline.map(r => r.wall_sec));
    return { variant, median_wall_sec: seconds, reduction_fraction: reduction,
      max_relative_objective_difference: objectiveDifference,
      passes_screen: reduction >= .1 && objectiveDifference <= .01 };
  });
  await writeFile(`${out}/report.json`, JSON.stringify(report, null, 2));
  console.log(JSON.stringify(report.summary, null, 2));
} finally {
  if (server.exitCode === null && server.signalCode === null) { const stopped = once(server, 'exit'); server.kill(); await stopped; }
}
