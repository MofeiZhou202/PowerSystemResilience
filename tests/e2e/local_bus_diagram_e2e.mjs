import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { mkdirSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { chromium } from 'playwright';

const arg = (key, fallback) => { const i = process.argv.indexOf(`--${key}`); return i < 0 ? fallback : process.argv[i + 1]; };
const output = path.resolve(arg('output-dir', 'output/gui-busbar'));
mkdirSync(output, { recursive: true });
const port = await new Promise(resolve => {
  const s = createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); });
});
const base = `http://127.0.0.1:${port}`;
const server = spawn(path.resolve(arg('server', 'build/macos-release/run_gui_server')),
  ['--host', '127.0.0.1', '--port', String(port), '--data-dir', path.resolve(arg('data-dir', 'data'))],
  { stdio: 'ignore' });
let browser;
try {
  let ready = false;
  for (let i = 0; i < 100; i++) {
    try { ready = (await fetch(`${base}/api/cases`)).ok; } catch { /* server startup */ }
    if (ready) break;
    await new Promise(resolve => setTimeout(resolve, 200));
  }
  assert.ok(ready, 'server startup');
  browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const errors = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.addInitScript(() => localStorage.setItem('hysim.tourDone.v1', '1'));
  await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined');
  const initial = await page.evaluate(async () => {
    const system = { name: '5000-bus hub', base_mva: 100,
      ac: { buses: Array.from({ length: 5000 }, (_, i) => ({ ...COMP.defaults.ac_bus, pd_mw: 0, qd_mvar: 0, index: i + 1, name: `Bus ${i + 1}`, base_kv: 10 })),
        branches: Array.from({ length: 4999 }, (_, i) => ({ ...COMP.defaults.ac_branch, index: i + 1, from_bus: 1, to_bus: i + 2 })) },
      dc: { buses: [{ ...COMP.defaults.dc_bus, index: 1, name: 'DC <same ID>', base_kv: 0.75 }], branches: [] },
      vsc_converters: [{ ...COMP.defaults.vsc_converter, index: 1, bus_ac: 1, bus_dc: 1 }] };
    Canvas.loadFromSystemJson(system);
    await Canvas.waitForLayout();
    window.__localOriginal = JSON.stringify(Canvas.buildSystemJson());
    App.selectStableRef({ domain: 'ac', index: 1 });
    return { headless: Canvas.isHeadless(), glyphs: Canvas.state.components.length };
  });
  assert.equal(initial.headless, true); assert.equal(initial.glyphs, 0);
  await page.click('#btnOverviewLocal');
  const inspect = () => page.evaluate(() => {
    const boxes = [...document.querySelectorAll('#subDiagramSvg .subdiag-bounds')].map(el => el.getBoundingClientRect());
    let overlaps = 0;
    for (let i = 0; i < boxes.length; i++) for (let j = i + 1; j < boxes.length; j++) {
      const a = boxes[i], b = boxes[j];
      if (a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom) overlaps++;
    }
    const modal = document.querySelector('.sub-diagram-modal').getBoundingClientRect();
    const original = JSON.parse(window.__localOriginal);
    const current = Canvas.buildSystemJson();
    return { buses: boxes.length, overlaps, paths: document.querySelectorAll('.subdiag-edge').length,
      modalFits: modal.left >= 0 && modal.right <= innerWidth && modal.top >= 0 && modal.bottom <= innerHeight,
      bodyOverflow: document.documentElement.scrollWidth > innerWidth,
      firstLabelSize: getComputedStyle(document.querySelector('#subDiagramSvg text')).fontSize,
      boundary: document.getElementById('subDiagramInfo').textContent,
      preserved: window.__localOriginal === JSON.stringify(current),
      changedTopLevelKeys: Object.keys(original).filter(key =>
        JSON.stringify(original[key]) !== JSON.stringify(current[key])) };
  });
  const desktop = await inspect();
  assert.equal(desktop.buses, 20); assert.equal(desktop.overlaps, 0);
  assert.ok(desktop.paths <= 160); assert.ok(desktop.modalFits); assert.ok(!desktop.bodyOverflow);
  assert.ok(desktop.boundary.includes('视图外')); assert.ok(desktop.preserved);
  await page.screenshot({ path: path.join(output, 'busbar-local-desktop.png') });
  await page.selectOption('#subDiagramLimit', '80');
  assert.equal((await inspect()).buses, 80);
  // The connection list is bounded too; page four reaches neighbors outside
  // this 80-bus sheet, without dropping those identities from the full model.
  for (let i = 0; i < 4; i++) await page.click('#btnSubDiagramNext');
  assert.ok(await page.locator('.sub-diagram-connection').count() <= 20);
  const destination = await page.locator('.sub-diagram-connection').first().getAttribute('data-index');
  await page.locator('.sub-diagram-connection').first().click();
  assert.ok((await page.locator('#subDiagramTitle').innerText()).startsWith(`AC ${destination} `));
  await page.click('#btnSubDiagramBack');
  assert.ok((await page.locator('#subDiagramTitle').innerText()).startsWith('AC 1 '));
  await page.setViewportSize({ width: 390, height: 844 });
  await page.selectOption('#subDiagramLimit', '20');
  const mobile = await inspect();
  assert.equal(mobile.overlaps, 0); assert.ok(mobile.modalFits); assert.ok(!mobile.bodyOverflow);
  assert.equal(mobile.firstLabelSize, desktop.firstLabelSize, 'mobile shrank the sheet text');
  await page.screenshot({ path: path.join(output, 'busbar-local-mobile.png') });
  await page.click('#btnSubDiagramClose');
  assert.equal(await page.locator('#subDiagramModal').isVisible(), false);
  assert.ok(await page.evaluate(() => Canvas.isHeadless() && NetworkOverview.active &&
    window.__localOriginal === JSON.stringify(Canvas.buildSystemJson())));
  await page.evaluate(() => App.openLocalSubgraph('dc', 1, 1));
  assert.ok((await page.locator('#subDiagramTitle').innerText()).startsWith('DC 1 '));
  assert.equal(await page.locator('#subDiagramSvg g[data-key="dc:1"]').count(), 1);
  assert.equal(await page.locator('#subDiagramSvg g[data-key="ac:1"]').count(), 1);
  assert.equal(await page.locator('#subDiagramSvg same').count(), 0, 'bus name was inserted as HTML');
  await page.click('#btnSubDiagramClose');

  // Real imported network uses the same overview-to-local path.
  await page.setViewportSize({ width: 1440, height: 1000 });
  await page.evaluate(() => App.loadMatpowerCase('case2869pegase.m'));
  await page.waitForFunction(() => Canvas.getNetworkOverviewStats()?.nodes > 2000);
  await page.evaluate(() => {
    window.__localOriginal = JSON.stringify(Canvas.buildSystemJson());
    App.selectStableRef({ domain: 'ac', index: 100 }, { openLocal: true });
  });
  const real = await inspect();
  assert.ok(real.buses >= 2 && real.buses <= 20); assert.equal(real.overlaps, 0); assert.ok(real.preserved);
  await page.screenshot({ path: path.join(output, 'busbar-case2869.png') });
  await page.click('#btnSubDiagramClose');

  // Exercise both the standard IEEE118 and the repository's mixed AC/DC
  // extension, alongside the original small regression case.
  const mainChecks = [], ieee118Local = [];
  for (const fixture of [
    { name: 'ieee14_acdc', ac: 14, dc: 2 },
    { name: 'case118.m', ac: 118, dc: 0, matpower: true },
    { name: 'ieee118_acdc', ac: 118, dc: 6 },
  ]) {
    await page.evaluate(f => f.matpower ? App.loadMatpowerCase(f.name) : App.loadBuiltinCase(f.name), fixture);
    await page.evaluate(() => Canvas.waitForLayout());
    const counts = await page.evaluate(() => {
      const sys = Canvas.buildSystemJson();
      return { ac: sys.ac.buses.length, dc: sys.dc.buses.length, headless: Canvas.isHeadless() };
    });
    assert.deepEqual(counts, { ac: fixture.ac, dc: fixture.dc, headless: false });
    for (const engine of ['legacy', 'elk']) {
      const result = await page.evaluate(async engine => {
        Canvas.setBusbarMode(true); Canvas.setBusHalfMax(320);
        await Canvas.autoLayout({ direction: 'BUSBAR', engine, timeoutMs: 60000 });
        const buses = Canvas.state.components.filter(c => c.type === 'ac_bus' || c.type === 'dc_bus');
        const boxes = buses.map(c => {
          const line = c.el.querySelector('line');
          return { left: c.x + Number(line.getAttribute('x1')), right: c.x + Number(line.getAttribute('x2')),
            top: c.y - 30, bottom: c.y + 30 };
        });
        let overlaps = 0;
        for (let i = 0; i < boxes.length; i++) for (let j = i + 1; j < boxes.length; j++) {
          const a = boxes[i], b = boxes[j];
          if (a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom) overlaps++;
        }
        Canvas.setBusHalfMax(160);
        const handlesAligned = buses.every(c => {
          const line = c.el.querySelector('line'), handles = c.el.querySelectorAll('.busbar-connect-handle');
          return handles[0].getAttribute('cx') === line.getAttribute('x1') &&
            handles[2].getAttribute('cx') === line.getAttribute('x2') &&
            c.el.querySelector('.comp-label').getAttribute('x') === line.getAttribute('x1');
        });
        return { engine, overlaps, handlesAligned, buses: buses.length, zoom: Canvas.state.zoom, metrics: Canvas.getLayoutMetrics() };
      }, engine);
      assert.equal(result.overlaps, 0, `${fixture.name}/${engine} bus overlap`); assert.ok(result.handlesAligned);
      mainChecks.push({ case: fixture.name, ...result });
      if (fixture.ac === 118) await page.screenshot({ path: path.join(output, `${fixture.name}-${engine}-full.png`) });
    }
    if (fixture.ac !== 118) continue;
    const focus = await page.evaluate(() => {
      window.__localOriginal = JSON.stringify(Canvas.buildSystemJson());
      const graph = HySimCore.LocalBusDiagram.buildGraph(Canvas.buildSystemJson());
      const key = [...graph.nodes.keys()].reduce((a, b) => graph.incident.get(a).length >= graph.incident.get(b).length ? a : b);
      const node = graph.nodes.get(key);
      return { domain: node.domain, index: node.index, degree: graph.incident.get(key).length };
    });
    for (const limit of [20, 40, 80]) {
      await page.evaluate(({ focus, limit }) => {
        document.getElementById('subDiagramLimit').value = String(limit);
        App.openLocalSubgraph(focus.domain, focus.index, 4);
      }, { focus, limit });
      const localCheck = await inspect();
      assert.ok(localCheck.buses <= limit && localCheck.buses > 1);
      assert.equal(localCheck.overlaps, 0); assert.ok(localCheck.preserved && localCheck.modalFits, JSON.stringify({ fixture: fixture.name, limit, ...localCheck }));
      assert.ok(await page.evaluate(ref => Canvas.state.selectedId ===
        Canvas.getCompBusMap()[ref.domain][ref.index], focus), 'local focus did not select its stable Canvas bus');
      const connections = await page.locator('.sub-diagram-connection').count();
      assert.equal(connections, Math.min(focus.degree, 20));
      ieee118Local.push({ case: fixture.name, focus, limit, ...localCheck });
    }
    await page.selectOption('#subDiagramLimit', '20');
    await page.screenshot({ path: path.join(output, `${fixture.name}-local-desktop.png`) });
    const nextBus = await page.locator('.sub-diagram-connection').first().getAttribute('data-index');
    await page.locator('.sub-diagram-connection').first().click();
    assert.ok((await page.locator('#subDiagramTitle').innerText()).includes(` ${nextBus} ·`));
    assert.ok((await inspect()).preserved, 'connection navigation moved the main viewport');
    await page.click('#btnSubDiagramBack');
    assert.ok((await page.locator('#subDiagramTitle').innerText()).startsWith(`${focus.domain.toUpperCase()} ${focus.index} `));
    for (const width of [390, 320]) {
      await page.setViewportSize({ width, height: 844 });
      const mobileCheck = await inspect();
      assert.equal(mobileCheck.overlaps, 0); assert.ok(mobileCheck.modalFits && !mobileCheck.bodyOverflow);
      assert.equal(mobileCheck.firstLabelSize, '13px'); assert.ok(mobileCheck.preserved);
      ieee118Local.push({ case: fixture.name, viewportWidth: width, ...mobileCheck });
      await page.screenshot({ path: path.join(output, `${fixture.name}-local-${width}.png`) });
    }
    await page.click('#btnSubDiagramClose');
    await page.setViewportSize({ width: 1440, height: 1000 });
  }
  await page.screenshot({ path: path.join(output, 'busbar-main-editor.png') });
  const rotated = await page.evaluate(() => {
    const bus = Canvas.state.components.find(c => c.type === 'ac_bus');
    Canvas.panToComponent(bus.id); Canvas.rotateSelected(90);
    const connections = Canvas.state.connections.filter(c => c.from.compId === bus.id || c.to.compId === bus.id);
    return { count: connections.length, onBar: connections.every(c => {
      const points = c.geom.points;
      const p = c.from.compId === bus.id ? points[0] : points[points.length - 1];
      return Math.abs(p.x - bus.x) < 1e-6;
    }) };
  });
  assert.ok(rotated.count > 0 && rotated.onBar, 'rotated taps leave the busbar');
  await page.evaluate(() => {
    window.__localOriginal = JSON.stringify(Canvas.buildSystemJson());
    const bus = Canvas.buildSystemJson().ac.buses[0];
    App.openLocalSubgraph('ac', bus.index, 1);
  });
  await page.locator('#subDiagramSvg .subdiag-node').first().focus();
  await page.keyboard.press('Delete');
  assert.ok(await page.evaluate(() => window.__localOriginal === JSON.stringify(Canvas.buildSystemJson())),
    'local-view Delete reached the editor');
  await page.keyboard.press('Escape');
  assert.equal(await page.locator('#subDiagramModal').isVisible(), false);
  // An external model replacement invalidates the previous local view/history.
  await page.evaluate(() => {
    const bus = Canvas.buildSystemJson().ac.buses[0];
    App.openLocalSubgraph('ac', bus.index, 1);
    Canvas.loadFromSystemJson(Canvas.buildSystemJson());
  });
  assert.equal(await page.locator('#subDiagramModal').isVisible(), false);
  assert.equal(await page.locator('#subDiagramSvg .subdiag-node').count(), 0);
  assert.deepEqual(errors, []);
  const report = { schema: 'local_bus_diagram_e2e_v1', desktop, mobile, real, mainChecks, ieee118Local, rotated, output };
  writeFileSync(path.join(output, 'verification.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify(report));
} finally {
  await browser?.close(); server.kill();
}
