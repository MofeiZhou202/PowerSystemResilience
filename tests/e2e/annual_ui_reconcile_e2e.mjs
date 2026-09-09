// @ts-check
// Annual-sim UI reconciliation E2E: parallel daily decomposition only applies to
// per-day DynamicOPF, so the 按日并行 control is disabled (with a hint) for the
// coupled SCUC/SCED modes, and runAnnualSim sends an honest parallel_daily flag.
//
// Run: node tests/e2e/annual_ui_reconcile_e2e.mjs [--server <path>]

import { chromium } from 'playwright';
import { spawn } from 'node:child_process';
import net from 'node:net';
import { existsSync } from 'node:fs';
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
    await page.evaluate(() => document.getElementById('btnToggleAnnualPanel')?.click());
    await page.waitForTimeout(100);

    const stateFor = async (mode) => {
      await page.evaluate((m) => { const s = document.getElementById('annDailyMode'); s.value = m; s.dispatchEvent(new Event('change', { bubbles: true })); }, mode);
      await page.waitForTimeout(50);
      return page.evaluate(() => ({
        disabled: document.getElementById('annParallel').disabled,
        hint: (document.getElementById('annParallelHint')?.textContent || '').trim(),
      }));
    };
    const scuc = await stateFor('scuc');
    rec('SCUC disables 按日并行 with a hint', scuc.disabled === true && scuc.hint.length > 0, `hint="${scuc.hint}"`);
    const dopf = await stateFor('dopf');
    rec('DynamicOPF enables 按日并行, no hint', dopf.disabled === false && dopf.hint.length === 0);
    const sced = await stateFor('sced');
    rec('动态SCED disables 按日并行 with a hint', sced.disabled === true && sced.hint.length > 0);

    // Honest request: capture the run_annual_sim body without running the solve.
    let body = null;
    await page.route('**/api/session/run_annual_sim', async (route) => {
      try { body = JSON.parse(route.request().postData() || '{}'); } catch { body = {}; }
      await route.fulfill({ status: 500, contentType: 'application/json', body: JSON.stringify({ error: 'stub' }) });
    });
    const runWith = async (mode) => {
      body = null;
      await page.evaluate((m) => { const s = document.getElementById('annDailyMode'); s.value = m; s.dispatchEvent(new Event('change', { bubbles: true })); }, mode);
      await page.waitForTimeout(50);
      await page.evaluate(() => document.getElementById('btnRunAnnualSim')?.click());
      for (let i = 0; i < 40 && body === null; i++) await page.waitForTimeout(100);
      return body;
    };
    const scucBody = await runWith('scuc');
    rec('SCUC run sends parallel_daily=false', scucBody && scucBody.parallel_daily === false, `parallel_daily=${scucBody && scucBody.parallel_daily}`);
    const dopfBody = await runWith('dopf');
    rec('DynamicOPF run sends parallel_daily=true', dopfBody && dopfBody.parallel_daily === true, `parallel_daily=${dopfBody && dopfBody.parallel_daily}`);

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
