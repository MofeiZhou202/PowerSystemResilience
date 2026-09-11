// @ts-check
// Annual-sim UI reconciliation E2E: parallel daily decomposition only applies to
// per-day DynamicOPF, so the 按日并行 control is disabled (with a hint) for the
// coupled SCUC/SCED modes, and runAnnualSim sends an honest parallel_daily flag.
//
// Run: node tests/e2e/annual_ui_reconcile_e2e.mjs [--server <path>]

import { chromium } from 'playwright';
import { spawn } from 'node:child_process';
import net from 'node:net';
import { existsSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const arg = (n, fb) => { const i = process.argv.indexOf(`--${n}`); return i >= 0 ? process.argv[i + 1] : fb; };
function freePort() { return new Promise((res, rej) => { const s = net.createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => res(p)); }); s.on('error', rej); }); }
function findServer() { const c = arg('server', ''); const cands = c ? [c] : ['build/macos-release/run_gui_server', 'build/macos-release/tests/run_gui_server'].map(p => path.join(ROOT, p)); const f = cands.find(existsSync); if (!f) { console.error('run_gui_server not found; pass --server'); process.exit(2); } return f; }
async function waitReady(base, ms = 25000) { const t0 = Date.now(); while (Date.now() - t0 < ms) { try { const r = await fetch(`${base}/api/cases`); if (r.ok) return true; } catch {} await new Promise(r => setTimeout(r, 300)); } return false; }

async function main() {
  const port = await freePort();
  const base = `http://127.0.0.1:${port}`;
  const server = spawn(findServer(), ['--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(ROOT, 'data'), '--matpower-dir', path.join(ROOT, 'data')], { cwd: ROOT, stdio: 'ignore' });
  const checks = [];
  const rec = (name, ok, d = '') => { checks.push({ name, ok }); console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name}${d ? ' — ' + d : ''}`); };
  let browser;
  try {
    if (!await waitReady(base)) { console.error('server not ready'); process.exit(2); }
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    await page.route('https://cdn.plot.ly/**', r => r.fulfill({ contentType: 'text/javascript', body: 'window.Plotly={newPlot:()=>Promise.resolve(),react:()=>Promise.resolve(),purge:()=>{},Plots:{resize:()=>{}}};' }));
    const errors = [];
    page.on('pageerror', e => errors.push(String(e.message).slice(0, 160)));

    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined', null, { timeout: 30000 });
    await page.evaluate(async () => { await App.loadBuiltinCase('ieee14_acdc'); await Canvas.waitForLayout(); });
    // Open the annual-sim control panel.
    await page.evaluate(() => document.getElementById('moduleTimeSeries')?.click());
    if (!await page.locator('#annualSimControls').isVisible())
      await page.locator('#btnToggleAnnualPanel').click();
    await page.waitForTimeout(100);

    const stateFor = async (mode) => {
      await page.locator('#annDailyMode').selectOption(mode);
      await page.waitForTimeout(50);
      return page.evaluate(() => ({
        disabled: document.getElementById('annParallel').disabled,
        hint: (document.getElementById('annParallelHint')?.textContent || '').trim(),
      }));
    };
    const solverThreads = page.locator('#tspfSolverThreads');
    await page.locator('#tspfUcSolver').selectOption('native');
    await solverThreads.fill('2');
    const scuc = await stateFor('scuc');
    rec('SCUC solver threads editable independently of daily workers',
      await solverThreads.isEnabled() && await page.locator('#annThreads').isDisabled());
    rec('SCUC disables 按日并行 with a hint', scuc.disabled === true && scuc.hint.length > 0, `hint="${scuc.hint}"`);
    const dopf = await stateFor('dopf');
    rec('DynamicOPF enables 按日并行, no hint', dopf.disabled === false && dopf.hint.length === 0);
    const sced = await stateFor('sced');
    rec('动态SCED disables 按日并行 with a hint', sced.disabled === true && sced.hint.length > 0);

    rec('SCED retains editable solver thread limit', await solverThreads.isEnabled() && await solverThreads.inputValue() === '2');
    await page.locator('#tspfUcSolver').selectOption('highs');
    rec('HiGHS explicitly disables unsupported custom thread limit', await solverThreads.isDisabled() &&
      (await page.locator('#tspfSolverThreadsHint').innerText()).includes('不支持'));
    await page.locator('#tspfUcSolver').selectOption('scip');
    rec('SCIP disables unsupported custom thread limit', await solverThreads.isDisabled());
    await page.locator('#tspfUcSolver').selectOption('auto');
    rec('Auto restores custom limit', await solverThreads.isEnabled() && await solverThreads.inputValue() === '2');

    await stateFor('scuc');
    await page.locator('#tspfUcSolver').scrollIntoViewIfNeeded();
    mkdirSync(path.join(ROOT, 'output/gui-uc-threads'), { recursive: true });
    await page.screenshot({ path: path.join(ROOT, 'output/gui-uc-threads/annual-controls.png') });

    // Honest request: capture the run_annual_sim body without running the solve.
    let body = null;
    await page.route('**/api/session/run_annual_sim', async (route) => {
      try { body = JSON.parse(route.request().postData() || '{}'); } catch { body = {}; }
      await route.fulfill({ status: 500, contentType: 'application/json', body: JSON.stringify({ error: 'stub' }) });
    });
    const runWith = async (mode) => {
      body = null;
      await page.locator('#annDailyMode').selectOption(mode);
      await page.waitForTimeout(50);
      await page.locator('#btnRunAnnualSim').click();
      for (let i = 0; i < 40 && body === null; i++) await page.waitForTimeout(100);
      return body;
    };
    const scucBody = await runWith('scuc');
    rec('SCUC run sends parallel_daily=false', scucBody && scucBody.parallel_daily === false, `parallel_daily=${scucBody && scucBody.parallel_daily}`);
    rec('Annual SCUC sends independent solver limit', scucBody?.uc_solver_threads === 2);
    const dopfBody = await runWith('dopf');
    rec('DynamicOPF run sends parallel_daily=true', dopfBody && dopfBody.parallel_daily === true, `parallel_daily=${dopfBody && dopfBody.parallel_daily}`);

    await page.locator('#annParallel').uncheck();
    rec('Disabling daily workers does not disable solver threads', await solverThreads.isEnabled());
    await page.locator('#annParallel').check();
    await page.locator('#annThreads').fill('3');
    const distinct = await runWith('dopf');
    rec('Daily-worker and solver limits remain distinct', distinct?.parallel_threads === 3 && distinct?.uc_solver_threads === 2);
    await stateFor('scuc');
    for (const width of [1600, 390]) {
      await page.setViewportSize({ width, height: 1000 });
      await solverThreads.fill('4');
      rec(`Thread input editable at ${width}px`, await solverThreads.inputValue() === '4');
    }
    await page.setViewportSize({ width: 1600, height: 1000 });
    await solverThreads.fill('2');

    // Real API validation; no year-long solve is needed to reject bad settings.
    for (const endpoint of ['run_ts_pf', 'run_annual_sim']) {
      for (const bad of [-1, 257, 1.5, '2', null]) {
        const response = await fetch(`${base}/api/session/${endpoint}`, {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ uc_solver_threads: bad }),
        });
        rec(`${endpoint} rejects invalid threads ${JSON.stringify(bad)}`, response.status === 400);
      }
      for (const uc_solver of ['highs', 'scip']) {
        const response = await fetch(`${base}/api/session/${endpoint}`, {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ uc_solver, uc_solver_threads: 2 }),
        });
        rec(`${endpoint} rejects unsupported ${uc_solver} override`, response.status === 400);
      }
    }
    // Actual UC solve through the GUI and backend, using the current IEEE14 AC/DC.
    await page.evaluate(() => App.setActiveModule('tspf'));
    await page.locator('#tspfUcSolver').selectOption('native');
    await page.locator('#simulationHours').fill('2');
    await page.locator('#tspfSkipUC').uncheck();
    const responsePromise = page.waitForResponse(r => r.url().endsWith('/api/session/run_ts_pf') && r.request().method() === 'POST', { timeout: 120000 });
    await page.locator('#btnRunTimeSeriesPF').click();
    const response = await responsePromise;
    const actual = await response.json();
    rec('Real time-series UC receives and configures 2 solver threads', response.ok() &&
      actual.uc_solver_threads_requested === 2 && actual.uc_solver_threads_configured === 2,
      JSON.stringify({ backend: actual.uc_solver_name, feasible: actual.uc_feasible, error: actual.error }));
    await page.locator('#tspfResultsSection').getByText('配置上限 2 线程', { exact: false }).waitFor();
    rec('Result displays configured cap without claiming active workers',
      (await page.locator('#tspfResultsSection').innerText()).includes('UC 求解器线程'));
    const rowFits = await page.locator('#tspfResultsSection .uc-thread-result').evaluate(row => {
      const label = row.querySelector('.result-label').getBoundingClientRect();
      const value = row.querySelector('.result-value').getBoundingClientRect();
      return label.right <= value.left && value.right <= row.getBoundingClientRect().right + 1;
    });
    rec('Solver thread result columns do not overlap', rowFits);
    await page.screenshot({ path: path.join(ROOT, 'output/gui-uc-threads/solver-result.png') });
    rec('no page errors', errors.length === 0, errors.join(' | '));

    const failed = checks.filter(c => !c.ok);
    console.log(`\n${failed.length ? failed.length + ' CHECK(S) FAILED' : 'ALL CHECKS PASSED'} (${checks.length} checks)`);
    process.exitCode = failed.length ? 1 : 0;
  } catch (err) {
    console.error('E2E error:', err && err.stack || err);
    process.exitCode = 1;
  } finally {
    if (browser) await browser.close();
    server.kill('SIGKILL');
  }
}
main();
