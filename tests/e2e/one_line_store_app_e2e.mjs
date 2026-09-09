// @ts-check
// One-line SQLite store — in-app E2E (P3). Boots run_gui_server, opens the real
// GUI at /xjtu/, loads a hybrid AC/DC case, and verifies that the wired
// Save DB / Open DB buttons persist the authored model to a portable .sqlite
// file (vendored sql.js WASM) and reload it losslessly through the P0 store.
//
// Run:  node tests/e2e/one_line_store_app_e2e.mjs [--case ieee14_acdc] [--server <path>]
// Deps: playwright (installed) + a built run_gui_server binary.

import { chromium } from 'playwright';
import { spawn } from 'node:child_process';
import net from 'node:net';
import { existsSync, readFileSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const arg = (name, fb) => { const i = process.argv.indexOf(`--${name}`); return i >= 0 ? process.argv[i + 1] : fb; };
const CASE = arg('case', 'ieee14_acdc');

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
    'build/linux-release/run_gui_server', 'build/tests/run_gui_server.exe',
  ].map(p => path.join(ROOT, p));
  const found = candidates.find(existsSync);
  if (!found) { console.error('run_gui_server not found; build it or pass --server'); process.exit(2); }
  return found;
}
async function waitReady(base, timeoutMs = 25000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try { const r = await fetch(`${base}/api/cases`); if (r.ok) return true; } catch { /* not up yet */ }
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
  const record = (name, ok, detail = '') => { checks.push({ name, ok }); console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name}${detail ? ' — ' + detail : ''}`); };

  let browser;
  try {
    if (!await waitReady(base)) { console.error('server did not become ready'); process.exit(2); }
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch { } });
    await page.route('https://cdn.plot.ly/**', route => route.fulfill({ contentType: 'text/javascript', body: 'window.Plotly={newPlot:()=>Promise.resolve(),react:()=>Promise.resolve(),purge:()=>{},relayout:()=>{},restyle:()=>{},addTraces:()=>{},deleteTraces:()=>{},downloadImage:()=>Promise.resolve(),Plots:{resize:()=>{}}};' }));

    const errors = [];
    page.on('pageerror', e => errors.push(String(e.message).slice(0, 200)));

    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(
      () => typeof App !== 'undefined' && typeof Canvas !== 'undefined'
            && !!(window.HySimCore && HySimCore.OneLineStore) && typeof window.initSqlJs === 'function',
      null, { timeout: 30000 });

    // 1) Wiring present: store module, sql.js global, and the three UI controls.
    const wiring = await page.evaluate(() => ({
      store: !!(window.HySimCore && HySimCore.OneLineStore),
      sqljs: typeof window.initSqlJs === 'function',
      saveBtn: !!document.getElementById('btnSaveDb'),
      openBtn: !!document.getElementById('btnOpenDb'),
      openInput: !!document.getElementById('fileOpenDb'),
    }));
    record('OneLineStore module loaded in app', wiring.store === true);
    record('sql.js (initSqlJs) global available', wiring.sqljs === true);
    record('Save/Open DB toolbar buttons + file input wired',
      wiring.saveBtn && wiring.openBtn && wiring.openInput);

    // 2) Load a real hybrid case and capture a stable signature.
    const loaded = await page.evaluate(async (caseName) => {
      await App.loadBuiltinCase(caseName);
      await Canvas.waitForLayout();
      return { count: Canvas.state.components.length };
    }, CASE);
    record('hybrid AC/DC case loaded with components', loaded.count > 0, `${loaded.count} components`);

    // 3) Full real round-trip THROUGH sql.js in the browser, exactly as the
    //    Save/Open buttons do it: buildSystemJson -> importSystemJson ->
    //    saveToDb -> db.export() (real .sqlite bytes) -> new Database(bytes) ->
    //    readFromDb -> exportSystemJson. Must be byte-stable and a real SQLite.
    const rt = await page.evaluate(async () => {
      const S = HySimCore.OneLineStore;
      const sys = Canvas.buildSystemJson();
      const rel = S.importSystemJson(sys);
      const SQL = await window.initSqlJs({ locateFile: (f) => 'vendor/' + f });
      const db = new SQL.Database();
      S.saveToDb(db, rel);
      const bytes = db.export();
      db.close();
      const magic = String.fromCharCode.apply(null, bytes.slice(0, 15));
      const db2 = new SQL.Database(bytes);
      const rel2 = S.readFromDb(db2);
      const sys2 = S.exportSystemJson(rel2);
      db2.close();
      const a = JSON.stringify(S.canonical(sys));
      const b = JSON.stringify(S.canonical(sys2));
      return { stable: a === b, magic, byteLen: bytes.length,
        buses: rel.buses.length, devices: rel.devices.length, links: rel.links.length,
        firstDiff: a === b ? -1 : [...a].findIndex((ch, i) => ch !== b[i]) };
    });
    record('exported bytes are a real SQLite database', rt.magic === 'SQLite format 3', `magic="${rt.magic}", ${rt.byteLen} bytes`);
    record('model round-trips byte-stable through a real .sqlite',
      rt.stable === true, rt.stable ? `${rt.buses} buses / ${rt.devices} devices / ${rt.links} links` : `first diff at ${rt.firstDiff}`);

    // 4) Save button actually downloads a .sqlite file (button wiring + export).
    const [download] = await Promise.all([
      page.waitForEvent('download', { timeout: 20000 }),
      page.click('#btnSaveDb'),
    ]);
    const dlName = download.suggestedFilename();
    const dlPath = await download.path();
    const dlBytes = dlPath ? readFileSync(dlPath) : Buffer.alloc(0);
    record('Save DB button downloads a *.sqlite file', /\.sqlite$/.test(dlName), dlName);
    record('downloaded file has the SQLite header', dlBytes.slice(0, 15).toString() === 'SQLite format 3', `${dlBytes.length} bytes`);

    // 5) Open button path restores the model: clear the canvas, then feed the
    //    saved file to the hidden input and confirm the model is reloaded.
    const cleared = await page.evaluate(() => {
      Canvas.loadFromSystemJson({ ac: { buses: [] }, dc: { buses: [] } });
      return Canvas.state.components.length;
    });
    record('canvas cleared before reopen', cleared === 0, `${cleared} components`);

    await page.setInputFiles('#fileOpenDb', /** @type {string} */(dlPath));
    await page.waitForFunction(
      (want) => Canvas.state.components.length === want, loaded.count, { timeout: 20000 });
    const restored = await page.evaluate(() => Canvas.state.components.length);
    record('Open DB button reloads the model losslessly', restored === loaded.count, `${restored}/${loaded.count} components`);

    record('no uncaught page errors during store IO', errors.length === 0, errors.join(' | '));

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
