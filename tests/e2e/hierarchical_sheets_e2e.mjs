// @ts-check
// Hierarchical sheets E2E (P4). Starts run_gui_server, drills into a composite
// node's sub-sheet, and asserts: drill-down/breadcrumb navigation, that
// analysis (buildSystemJson) always returns the ROOT network (never a
// sub-sheet), and that sub-sheet contents persist across navigation.
//
// Run:  node tests/e2e/hierarchical_sheets_e2e.mjs [--server <path>]

import { chromium } from 'playwright';
import { spawn } from 'node:child_process';
import net from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const arg = (name, fb) => { const i = process.argv.indexOf(`--${name}`); return i >= 0 ? process.argv[i + 1] : fb; };

function freePort() {
  return new Promise((resolve, reject) => {
    const s = net.createServer();
    s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); });
    s.on('error', reject);
  });
}
function findServer() {
  const c = arg('server', '');
  const candidates = c ? [c] : [
    'build/macos-release/run_gui_server', 'build/macos-release/tests/run_gui_server',
    'build/local-gui-review/run_gui_server', 'build/tests/run_gui_server',
  ].map(p => path.join(ROOT, p));
  const found = candidates.find(existsSync);
  if (!found) { console.error('run_gui_server not found; pass --server'); process.exit(2); }
  return found;
}
async function waitReady(base, timeoutMs = 25000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try { const r = await fetch(`${base}/api/cases`); if (r.ok) return true; } catch { /* not up */ }
    await new Promise(r => setTimeout(r, 300));
  }
  return false;
}

async function main() {
  const port = await freePort();
  const base = `http://127.0.0.1:${port}`;
  const server = spawn(findServer(),
    ['--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(ROOT, 'data'), '--matpower-dir', path.join(ROOT, 'data')],
    { cwd: ROOT, stdio: 'ignore' });
  const checks = [];
  const record = (name, ok, detail = '') => { checks.push({ ok }); console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name}${detail ? ' — ' + detail : ''}`); };

  let browser;
  try {
    if (!await waitReady(base)) { console.error('server not ready'); process.exit(2); }
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch { } });
    await page.route('https://cdn.plot.ly/**', route => route.fulfill({ contentType: 'text/javascript', body: 'window.Plotly={newPlot:()=>Promise.resolve(),react:()=>Promise.resolve(),purge:()=>{},relayout:()=>{},Plots:{resize:()=>{}}};' }));

    const errors = [];
    page.on('pageerror', e => errors.push(String(e.message).slice(0, 160)));

    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined', null, { timeout: 30000 });

    const r = await page.evaluate(() => {
      const busesInModel = () => (Canvas.buildSystemJson().ac?.buses || []).length;
      const B = Canvas.addComponent('ac_bus', 300, 300);
      const MG = Canvas.addComponent('microgrid', 520, 300);
      const rootBuses1 = busesInModel();                 // expect 1 (B)

      const entered = Canvas.enterSheet(MG.id);
      const depth1 = Canvas.getSheetDepth();
      Canvas.addComponent('ac_bus', 200, 200);           // author inside the sub-sheet
      Canvas.addComponent('generator', 200, 120);
      const subCount = Canvas.state.components.length;    // 2 in the sub-sheet
      const rootBusesInSub = busesInModel();             // KEY: still 1 (root), not 2
      const crumb = !!document.getElementById('sheetBreadcrumb');

      Canvas.exitToRoot();
      const depth0 = Canvas.getSheetDepth();
      const rootCount = Canvas.state.components.length;   // 2 at root (B + MG)
      const rootBuses2 = busesInModel();                 // still 1
      const crumbGone = !document.getElementById('sheetBreadcrumb');

      Canvas.enterSheet(MG.id);
      const subCount2 = Canvas.state.components.length;   // 2 preserved
      Canvas.exitToRoot();

      return { entered, depth1, subCount, rootBusesInSub, rootBuses1, crumb, depth0, rootCount, rootBuses2, crumbGone, subCount2 };
    });

    record('drill into composite sub-sheet', r.entered === true && r.depth1 === 1, `depth=${r.depth1}`);
    record('breadcrumb shown inside a sub-sheet', r.crumb === true);
    record('sub-sheet holds its own authored components', r.subCount === 2, `subCount=${r.subCount}`);
    record('buildSystemJson ignores sub-sheet, uses ROOT', r.rootBusesInSub === 1 && r.rootBuses1 === 1,
      `rootBuses=${r.rootBuses1}, whileInSub=${r.rootBusesInSub}`);
    record('exit returns to root (depth 0, root intact)', r.depth0 === 0 && r.rootCount === 2 && r.rootBuses2 === 1,
      `depth=${r.depth0}, rootCount=${r.rootCount}`);
    record('breadcrumb hidden at root', r.crumbGone === true);
    record('sub-sheet contents persist across navigation', r.subCount2 === 2, `re-entered=${r.subCount2}`);
    const persist = await page.evaluate(() => {
      const sys = Canvas.buildSystemJson();
      Canvas.loadFromSystemJson(sys);
      const mg = Canvas.state.components.find(c => c.type === 'microgrid');
      const hasSheet = !!(mg && mg._sheet);
      let reentered = -1;
      if (mg) { Canvas.enterSheet(mg.id); reentered = Canvas.state.components.length; Canvas.exitToRoot(); }
      return { hasSheet, reentered };
    });
    record('sub-sheet persists through save/load round-trip', persist.hasSheet === true && persist.reentered === 2,
      `reentered=${persist.reentered}`);
    record('no page errors', errors.length === 0, errors.slice(0, 2).join(' | '));

    await page.locator('#canvasContainer').screenshot({ path: path.join(ROOT, 'tests', 'e2e', 'hierarchical_sheets.png'), animations: 'disabled' }).catch(() => { });
  } finally {
    if (browser) await browser.close().catch(() => { });
    server.kill('SIGTERM');
  }

  const failed = checks.filter(c => !c.ok).length;
  console.log(`\n${checks.length - failed}/${checks.length} checks passed for hierarchical sheets.`);
  process.exit(failed ? 1 : 0);
}

main().catch(e => { console.error(e); process.exit(2); });
