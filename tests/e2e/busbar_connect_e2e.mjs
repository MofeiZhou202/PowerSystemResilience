// @ts-check
// Busbar connection E2E: in busbar mode a bus renders as a wide bar whose port
// dots are hidden, so a wire must be drawable by clicking anywhere along the bar
// (connect mode). Reproduces "AC/DC buses cannot be connected" and verifies the
// fix for both connect-start-from-bar and connect-complete-onto-bar.
//
// Run: node tests/e2e/busbar_connect_e2e.mjs [--case ieee14_acdc] [--server <path>]

import { chromium } from 'playwright';
import { spawn } from 'node:child_process';
import net from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const arg = (name, fb) => { const i = process.argv.indexOf(`--${name}`); return i >= 0 ? process.argv[i + 1] : fb; };
const CASE = arg('case', 'ieee14_acdc');

function freePort() { return new Promise((res, rej) => { const s = net.createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => res(p)); }); s.on('error', rej); }); }
function findServer() {
  const c = arg('server', '');
  const cands = c ? [c] : ['build/macos-release/run_gui_server', 'build/macos-release/tests/run_gui_server', 'build/tests/run_gui_server'].map(p => path.join(ROOT, p));
  const f = cands.find(existsSync);
  if (!f) { console.error('run_gui_server not found; pass --server'); process.exit(2); }
  return f;
}
async function waitReady(base, ms = 25000) { const t0 = Date.now(); while (Date.now() - t0 < ms) { try { const r = await fetch(`${base}/api/cases`); if (r.ok) return true; } catch {} await new Promise(r => setTimeout(r, 300)); } return false; }

async function main() {
  const port = await freePort();
  const base = `http://127.0.0.1:${port}`;
  const server = spawn(findServer(), ['--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(ROOT, 'data'), '--matpower-dir', path.join(ROOT, 'data')], { cwd: ROOT, stdio: 'ignore' });
  const checks = [];
  const record = (name, ok, detail = '') => { checks.push({ name, ok }); console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name}${detail ? ' — ' + detail : ''}`); };
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
    await page.evaluate(async (c) => { await App.loadBuiltinCase(c); await Canvas.waitForLayout(); Canvas.setBusbarMode(true); await Canvas.autoLayout({ direction: 'BUSBAR', timeoutMs: 60000 }); Canvas.zoomFit(); }, CASE);
    await page.waitForTimeout(400);

    // Grab four AC bus bars' on-screen rects + a baseline connection count.
    const info = await page.evaluate(() => {
      Canvas.setMode('connect');
      const buses = Canvas.state.components.filter(c => c.type === 'ac_bus');
      const rect = (b) => { const r = b.el.querySelector('line').getBoundingClientRect(); return { id: b.id, x: r.x, y: r.y, w: r.width, cy: r.y + r.height / 2 }; };
      return { a: rect(buses[0]), b: rect(buses[1]), c: rect(buses[2]), d: rect(buses[3]), before: Canvas.state.connections.length, mode: Canvas.state.mode };
    });
    record('connect mode active with >=4 bus bars', info.mode === 'connect' && info.a && info.d, `bars ${info.a?.id}/${info.b?.id}/${info.c?.id}/${info.d?.id}`);

    // Scenario 1 (connect mode): draw a wire from a point 30% along bar A to a
    // point 70% along bar B — both well away from the hidden centre port dots
    // (±40px), proving the bar is connectable anywhere in connect mode.
    const ax = info.a.x + info.a.w * 0.3, ay = info.a.cy;
    const bx = info.b.x + info.b.w * 0.7, by = info.b.cy;
    await page.mouse.move(ax, ay);
    await page.mouse.down();
    await page.mouse.move((ax + bx) / 2, (ay + by) / 2, { steps: 6 });
    await page.mouse.move(bx, by, { steps: 6 });
    await page.mouse.up();
    await page.waitForTimeout(150);

    const after1 = await page.evaluate((ids) => {
      const conns = Canvas.state.connections;
      const linksAB = conns.some(c =>
        (c.from.compId === ids.a && c.to.compId === ids.b) ||
        (c.from.compId === ids.b && c.to.compId === ids.a));
      return { count: conns.length, linksAB };
    }, { a: info.a.id, b: info.b.id });
    record('connect mode: dragging along the bar creates a connection', after1.count === info.before + 1, `${info.before} → ${after1.count}`);
    record('connect mode: the new wire links the two clicked buses', after1.linksAB === true);

    // Scenario 2 (SELECT mode): start a wire from a bus's hover-revealed connect
    // handle, finishing on another bar — no mode switch. Locate the real handle
    // element's on-screen centre so the press lands exactly on it.
    const handleInfo = await page.evaluate((cid) => {
      Canvas.setMode('select');
      const bus = Canvas.state.components.find(x => x.id === cid);
      const handles = [...bus.el.querySelectorAll('.busbar-connect-handle')];
      const rects = handles.map(h => { const r = h.getBoundingClientRect(); return { cx: r.x + r.width / 2, cy: r.y + r.height / 2, w: Math.round(r.width) }; });
      return { n: handles.length, rects, mode: Canvas.state.mode };
    }, info.c.id);
    record('bus exposes connect handles in select mode', handleInfo.n >= 1 && handleInfo.mode === 'select', `${handleInfo.n} handles`);
    const h = handleInfo.rects[Math.floor(handleInfo.rects.length / 2)] || handleInfo.rects[0];
    const dx = info.d.x + info.d.w * 0.5, dy = info.d.cy;             // mid bar D
    await page.mouse.move(h.cx, h.cy);                               // hover to reveal the handle
    await page.mouse.down();
    await page.mouse.move((h.cx + dx) / 2, (h.cy + dy) / 2, { steps: 6 });
    await page.mouse.move(dx, dy, { steps: 6 });
    await page.mouse.up();
    await page.waitForTimeout(150);

    const after2 = await page.evaluate((ids) => {
      const conns = Canvas.state.connections;
      const linksCD = conns.some(c =>
        (c.from.compId === ids.c && c.to.compId === ids.d) ||
        (c.from.compId === ids.d && c.to.compId === ids.c));
      return { count: conns.length, linksCD, dragged: Canvas.state.components.find(x => x.id === ids.c) };
    }, { c: info.c.id, d: info.d.id });
    record('select mode: connect handle starts a wire (no mode switch)', after2.count === after1.count + 1, `${after1.count} → ${after2.count}`);
    record('select mode: the new wire links the handle bus to the target bus', after2.linksCD === true);
    record('no page errors during bus wiring', errors.length === 0, errors.join(' | '));

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
