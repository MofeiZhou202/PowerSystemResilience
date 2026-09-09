// @ts-check
// Busbar length slider E2E: the toolbar 母线长度 range live-adjusts the resizable
// busbar cap (Canvas.setBusHalfMax), re-spans bars, persists to localStorage, and
// shows only in busbar mode.
//
// Run: node tests/e2e/busbar_length_e2e.mjs [--server <path>]

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

const maxHalf = (page) => page.evaluate(() => {
  const buses = Canvas.state.components.filter(c => c.type === 'ac_bus' || c.type === 'dc_bus');
  return Math.max(0, ...buses.map(b => { const l = b.el && b.el.querySelector('line'); return l ? Math.abs(parseFloat(l.getAttribute('x2'))) : 0; }));
});
const setSlider = (page, v) => page.evaluate((val) => { const s = document.getElementById('busLenRange'); s.value = String(val); s.dispatchEvent(new Event('input', { bubbles: true })); }, v);

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
    await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined' && HySimCore?.LayoutEngine, null, { timeout: 30000 });
    await page.evaluate(async () => { await App.loadBuiltinCase('ieee14_acdc'); await Canvas.waitForLayout(); Canvas.setBusbarMode(true); await Canvas.autoLayout({ direction: 'BUSBAR', timeoutMs: 60000 }); });

    const sliderShown = await page.evaluate(() => { const el = document.getElementById('busLenControl'); return !!el && !el.hidden; });
    rec('length slider visible in busbar mode', sliderShown === true);

    const base320 = await maxHalf(page);
    rec('default cap ~320', base320 > 40 && base320 <= 340, `maxHalf=${base320}`);

    await setSlider(page, 160);
    await page.waitForTimeout(80);
    const small = await maxHalf(page);
    rec('shrinking the slider shortens bars', small < base320 && small <= 170, `maxHalf=${small}`);

    await setSlider(page, 600);
    await page.waitForTimeout(80);
    const big = await maxHalf(page);
    rec('growing the slider lengthens bars', big > small && big > 400, `maxHalf=${big}`);

    const persisted = await page.evaluate(() => { try { return parseInt(localStorage.getItem('busHalfMax') || '', 10); } catch { return null; } });
    rec('cap persists to localStorage', persisted === 600, `busHalfMax=${persisted}`);

    // Slider hides when busbar mode is turned off (via the toolbar toggle).
    const hiddenOff = await page.evaluate(() => { document.getElementById('btnBusbarMode')?.click(); const el = document.getElementById('busLenControl'); return !!el && el.hidden; });
    rec('length slider hidden when busbar mode is off', hiddenOff === true);

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
