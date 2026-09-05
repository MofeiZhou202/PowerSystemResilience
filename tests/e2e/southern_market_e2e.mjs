import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const index = process.argv.indexOf('--server');
const serverPath = index >= 0 ? process.argv[index + 1] : path.join(root, 'build/macos-release/tests/run_gui_server');
const port = await new Promise((resolve, reject) => {
  const s = createServer(); s.on('error', reject); s.listen(0, '127.0.0.1', () => { const port = s.address().port; s.close(() => resolve(port)); });
});
const base = `http://127.0.0.1:${port}`;
const server = spawn(serverPath, ['--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(root, 'data'), '--matpower-dir', path.join(root, 'data')], { cwd: root, stdio: 'ignore' });
let browser;
try {
  for (let i = 0; i < 100; ++i) {
    try { if ((await fetch(`${base}/api/cases`)).ok) break; } catch {}
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  const post = async (route, body) => {
    const response = await fetch(base + route, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    return { status: response.status, data: await response.json() };
  };
  const get = async () => (await fetch(base + '/api/session/southern_market')).json();
  browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  await page.addInitScript(() => localStorage.setItem('hysim.tourDone.v1', '1'));
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof App !== 'undefined');
  await page.evaluate(() => App.setActiveModule('marketBoundary'));
  await page.locator('#btnSouthernOpen').click();
  await page.locator('#southernExample').click();
  await page.waitForFunction(() => document.querySelector('#southernCategory option[value="reservoirs"]'));
  let state = await get();
  assert.equal(Object.keys(state.schema.properties).length, Object.keys(state.boundary).length);
  assert.equal(state.boundary.periods.length, 98);
  const invalid = structuredClone(state.boundary); invalid.areas[0].load_mw[0] = -1;
  const rejected = await post('/api/session/southern_market', { action: 'save', revision: state.revision, boundary: invalid });
  assert.equal(rejected.status, 400, JSON.stringify(rejected));
  assert.equal((await get()).revision, state.revision);
  const stale = await post('/api/session/southern_market', { action: 'save', revision: state.revision - 1, boundary: state.boundary });
  assert.equal(stale.status, 409);

  await page.locator('#btnSouthernRun').click();
  await page.waitForFunction(() => document.querySelector('#southernToolbarStatus').textContent.includes('converged'));
  await page.locator('#southernBaseline').click();
  await page.waitForFunction(() => document.querySelector('#southernStatus').textContent.includes('已有基准'));
  await page.locator('#southernBoundaryView').click();
  await page.locator('#southernCategory').selectOption('areas');
  const forecast = page.locator('[data-southern-path="areas/0/load_mw/0"]');
  await forecast.locator('xpath=ancestor::details').evaluate(el => { el.open = true; });
  await forecast.fill('101'); await forecast.blur();
  await page.locator('#southernSave').click();
  await page.waitForFunction(() => document.querySelector('#southernStatus').textContent.includes('边界已保存'));
  await page.locator('#southernLoad').click();
  await page.waitForFunction(() => document.querySelector('#southernStatus').textContent.includes('已载入边界'));
  assert.equal(await page.locator('[data-southern-path="areas/0/load_mw/0"]').inputValue(), '101');
  await page.locator('#btnSouthernRun').click();
  await page.waitForFunction(() => document.querySelector('#southernResults').textContent.includes('边界情景'));
  state = await get();
  assert.equal(state.latest.comparison.comparable, true);
  assert.ok(Math.abs(state.latest.comparison.delta_day_energy_bid_cost - 50) < 1e-6);
  assert.equal(state.latest.effective_boundary.buses[0].load_mw[0], 101);
  assert.equal(state.latest.boundary_snapshot.buses[0].load_mw[0], 100);
  assert.equal(state.latest.lmp.buses[0].lmp_per_mwh[0], 200);
  await mkdir(path.join(root, 'output/southern-market'), { recursive: true });
  await writeFile(path.join(root, 'output/southern-market/numerical-evidence.json'), JSON.stringify({ baseline: state.baseline, scenario: state.latest }, null, 2));
  await page.screenshot({ path: path.join(root, 'output/southern-market/desktop.png') });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.locator('#southernMarketWorkspace').scrollIntoViewIfNeeded();
  const size = await page.evaluate(() => ({ width: document.documentElement.clientWidth, scroll: document.documentElement.scrollWidth,
    editorWidth: document.querySelector('#southernEditor').clientWidth, editorScroll: document.querySelector('#southernEditor').scrollWidth }));
  assert.ok(size.scroll <= size.width + 1, JSON.stringify(size));
  assert.ok(size.editorScroll <= size.editorWidth + 1, JSON.stringify(size));
  await page.screenshot({ path: path.join(root, 'output/southern-market/mobile.png') });
  await page.locator('#southernRestore').click();
  await page.waitForFunction(() => document.querySelector('#southernStatus').textContent.includes('边界已保存'));
  assert.equal((await get()).boundary.areas[0].load_mw[0], 100);
  await page.reload({ waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof App !== 'undefined');
  await page.evaluate(() => App.setActiveModule('marketBoundary'));
  await page.locator('#btnSouthernOpen').click();
  await page.waitForFunction(() => document.querySelector('#southernStatus').textContent.includes('已有基准'));
  const loaded = await post('/api/session/load_builtin', { case: 'market_5bus_acdc_toy' });
  assert.equal(loaded.status, 200, JSON.stringify(loaded.data));
  assert.equal((await get()).boundary, null);
  assert.deepEqual(errors, []);
  console.log('Southern GUI/API: schema, edit, atomic validation, revision conflict, reload, baseline, load sensitivity, restore and mobile layout passed.');
} finally {
  if (browser) await browser.close(); server.kill();
}
