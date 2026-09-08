import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const index = process.argv.indexOf('--server');
const executable = index < 0 ? path.join(root, 'build/macos-release/tests/run_gui_server') : process.argv[index + 1];
const port = await new Promise(resolve => { const s = createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); }); });
const base = `http://127.0.0.1:${port}`, output = path.join(root, 'output/market-realtime');
const server = spawn(executable, ['--host', '127.0.0.1', '--port', String(port)], { cwd: root, stdio: 'ignore' });
let browser;
const near = (a, b, tolerance = 1e-5) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);
async function api(route, body, expected = 200) {
  const r = await fetch(base + '/api/session/' + route, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
  const d = await r.json(); assert.equal(r.status, expected, JSON.stringify(d).slice(0, 500)); return d;
}
try {
  await mkdir(output, { recursive: true });
  for (let i = 0; i < 150; ++i) { try { if ((await fetch(base + '/api/cases')).ok) break; } catch {} await new Promise(r => setTimeout(r, 100)); }
  let s = await api('southern_market'); await api('southern_market', { action: 'ieee118_mixed', revision: s.revision });
  s = await api('southern_realtime'); const c = s.config; c.steps = 2; c.boundary.execution.ac_security = 'schedule_only'; c.boundary.execution.time_limit_sec = 30;
  c.boundary.execution.solver = s.solver_capabilities.find(x => x.id === 'gurobi' && x.available) ? 'gurobi' : 'highs';
  c.boundary.branches[0].available[6] = 0;
  c.renewable_forecasts[0].submitted[0] = null;
  c.renewable_forecasts[0].dispatch_forecast[0] = null;
  s = await api('southern_realtime', { action: 'save', revision: s.revision, config: c });
  const sealedRevision = s.revision;
  assert.equal(s.job.forecast_resolution[0].origin, 'day_ahead_fallback');
  assert.equal(s.job.config.boundary.buses.length, 118); assert.equal(s.job.config.boundary.generators.length, 66);
  await api('southern_realtime', { action: 'step', revision: s.revision, run_id: s.run_id + 1 }, 409);
  browser = await chromium.launch(); const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  await page.addInitScript(() => localStorage.setItem('hysim.tourDone.v1', '1'));
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  await page.goto(base + '/xjtu/#market-realtime', { waitUntil: 'networkidle' });
  await page.waitForFunction(() => !document.querySelector('#rtStep').disabled);
  assert.equal(await page.locator('#rtSolver').inputValue(), c.boundary.execution.solver);
  assert.equal(await page.locator('#rtCatalog tr').count(), 11);
  await page.locator('#rtRun').click();
  await page.waitForFunction(() => document.querySelector('#rtStatus').textContent === '滚动结束', { timeout: 180000 });
  s = await api('southern_realtime'); assert.equal(s.job.completed_steps, 2);
  let waterResidual = 0, storageResidual = 0, carryResidual = 0;
  for (const [i, run] of s.job.runs.entries()) {
    const r = run.dispatch, b = r.boundary_snapshot, result = r.sced;
    assert.equal(r.schedule_feasible, true); assert.equal(r.prices_valid, true); assert.equal(run.executed_points, 3);
    assert.equal(result.generators[0].power_mw.length, 24); assert.equal(r.lmp.buses[0].lmp_per_mwh.length, 8);
    assert.equal(run.outlook.reference_only, true); assert.equal(run.outlook.schedule_feasible, true);
    for (const storage of b.storage) {
      const row = result.storage.find(s => s.id === storage.id), eta = Math.sqrt(storage.roundtrip_efficiency);
      const used = row.discharge_mw.reduce((v, p) => v + p / eta / 12, 0) + row.charge_mw.reduce((v, p) => v + p * eta / 12, 0);
      storageResidual = Math.max(storageResidual, Math.abs(storage.initial_mwh - used - row.energy_mwh[23]));
      if (i) { const prior = s.job.runs[i - 1].dispatch.sced.storage.find(s => s.id === storage.id); carryResidual = Math.max(carryResidual, Math.abs(storage.initial_mwh - prior.energy_mwh[2])); }
    }
    for (const reservoir of b.reservoirs) {
      const row = result.reservoirs.find(h => h.id === reservoir.id), parent = b.reservoirs.find(h => h.id === reservoir.upstream);
      let netVolume = 0;
      for (let t = 0; t < 24; ++t) {
        const delayed = t - reservoir.lag_slots;
        const upstream = !parent ? 0 : delayed < 0 ? parent.release_history_m3_s[parent.release_history_m3_s.length + delayed] : result.reservoirs.find(h => h.id === parent.id).release_m3_s[delayed];
        netVolume += 300 * (reservoir.inflow_m3_s[t] + upstream - row.release_m3_s[t]);
      }
      waterResidual = Math.max(waterResidual, Math.abs(row.level_m[23] - reservoir.initial_level_m - netVolume / reservoir.area_m2));
      if (i) { const prior = s.job.runs[i - 1].dispatch.sced.reservoirs.find(h => h.id === reservoir.id); carryResidual = Math.max(carryResidual, Math.abs(reservoir.initial_level_m - prior.level_m[2])); }
    }
    const outage = result.branches.find(l => l.id === c.boundary.branches[0].id); near(outage.power_mw[6 - 3 * i], 0);
  }
  near(waterResidual, 0); near(storageResidual, 0); near(carryResidual, 0);
  assert.ok(s.job.hourly_prices.every(p => p.price_per_mwh === null));
  await page.getByLabel('Canvas 时段', { exact: true }).selectOption('5');
  assert.equal(await page.locator('#rtSlot').inputValue(), '5'); assert.equal(await page.locator('#marketCanvas').getAttribute('data-slot'), '5');
  await page.locator('#rtPowerChart').evaluate(e => e.emit('plotly_click', { points: [{ pointIndex: 10 }] }));
  assert.equal(await page.locator('#marketCanvas').getAttribute('data-slot'), '10');
  assert.equal(await page.locator('#rtPriceChart').evaluate(e => e.data[0].y.length), 8);
  const priceRange = await page.locator('#rtPriceChart').evaluate(e => e.layout.yaxis.range);
  assert.ok(priceRange[1] - priceRange[0] >= 0.99);
  await page.locator('#rtWindow').selectOption('outlook'); assert.equal(await page.locator('#rtPriceChart').evaluate(e => e.data.length), 0);
  await page.locator('#rtWindow').selectOption('dispatch');
  await page.locator('#rtPowerChart').scrollIntoViewIfNeeded(); await page.screenshot({ path: path.join(output, 'desktop.png') });
  for (const width of [390, 768]) {
    await page.setViewportSize({ width, height: 844 }); await page.waitForTimeout(250);
    assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth + 1));
    await page.locator('#rtPowerChart').scrollIntoViewIfNeeded(); await page.screenshot({ path: path.join(output, `mobile-${width}.png`) });
  }
  await page.setViewportSize({ width: 1440, height: 1000 });
  await page.locator('#rtSteps').fill('3'); await page.locator('#rtSteps').blur(); assert.equal(await page.locator('#rtStep').isDisabled(), true);
  await page.locator('#rtSave').click(); await page.waitForFunction(() => document.querySelector('#rtStatus').textContent.includes('边界校验通过'));
  const replaced = await api('southern_realtime'); assert.equal(replaced.job.completed_steps, 0);
  await api('southern_realtime', { action: 'step', revision: sealedRevision, run_id: s.run_id }, 409);
  const invalid = structuredClone(replaced.config); invalid.boundary.generators[0].segments[0].price_per_mwh += 1;
  await api('southern_realtime', { action: 'save', revision: replaced.revision, config: invalid }, 400);
  assert.deepEqual(errors, []);
  const summary = { case: 'IEEE118 mixed', solver: c.boundary.execution.solver, rounds: 2, dispatch_seconds: s.job.runs.map(r => r.dispatch.runtime_sec), water_level_residual_m: waterResidual, storage_balance_residual_mwh: storageResidual, carry_residual: carryResidual, browser_errors: errors };
  await writeFile(path.join(output, 'e2e-summary.json'), JSON.stringify(summary, null, 2));
  await writeFile(path.join(output, 'ieee118-result.json'), JSON.stringify(s.job)); console.log(JSON.stringify(summary));
} finally { await browser?.close(); server.kill('SIGTERM'); await new Promise(resolve => { if (server.exitCode !== null) resolve(); else server.once('exit', resolve); }); }
