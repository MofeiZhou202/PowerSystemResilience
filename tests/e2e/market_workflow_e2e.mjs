import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { mkdir, writeFile } from 'node:fs/promises';
import { once } from 'node:events';
import assert from 'node:assert/strict';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const output = path.join(root, 'output/market-workflow');
const arg = process.argv.indexOf('--server');
const executable = arg < 0 ? path.join(root, 'build/macos-release/tests/run_gui_server') : process.argv[arg + 1];
const port = await new Promise(resolve => {
  const socket = createServer(); socket.listen(0, '127.0.0.1', () => {
    const value = socket.address().port; socket.close(() => resolve(value));
  });
});
const base = `http://127.0.0.1:${port}`;
const server = spawn(executable, ['--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(root, 'data')], { cwd: root, stdio: 'ignore' });
let browser;
const errors = [], checks = [];
try {
  await mkdir(output, { recursive: true });
  for (let n = 0; n < 100; ++n) {
    try { if ((await fetch(`${base}/api/cases`)).ok) break; } catch {}
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 960 } });
  page.on('pageerror', error => errors.push(error.message));
  await page.addInitScript(() => localStorage.setItem('hysim.tourDone.v1', '1'));
  await page.goto(`${base}/xjtu/`);
  await page.locator('#workflowMarket').click();
  await page.locator('#marketOperationWorkspace').waitFor({ state: 'visible' });
  assert.equal(await page.locator('#marketModel').inputValue(), 'southern');
  assert.equal(await page.locator('#marketStudyWorkspace').isVisible(), false);
  assert.equal(await page.locator('#marketWorkflowSteps').count(), 0);
  checks.push('default operation and no duplicate workflow');

  await page.locator('#marketTutorial').click();
  await page.locator('#marketGuideRoute').selectOption('day');
  await page.locator('#marketGuideSteps button').first().click();
  await page.locator('#southernMarketWorkspace').waitFor({ state: 'visible' });
  assert.equal(await page.locator('#marketNumSteps').isVisible(), false);
  await page.locator('#southernCase').selectOption('demo');
  await page.locator('#southernLoadCase').click();
  await page.waitForFunction(() => document.getElementById('southernCaseSummary').textContent.includes('2 节点'));
  await page.locator('#southernRunSaved').click();
  await page.waitForFunction(() => document.querySelector('[aria-label="结果时点"]'), null, { timeout: 120000 });
  assert.ok((await page.locator('#southernResults').innerText()).length > 100);
  checks.push('tutorial links and real two-node day-ahead solve');

  await page.locator('#marketManual').click();
  await page.getByRole('heading', { name: '电力市场仿真操作教程', exact: true }).waitFor();
  await page.keyboard.press('Escape');
  assert.equal(await page.getByRole('heading', { name: '电力市场仿真操作教程', exact: true }).isVisible(), false);
  checks.push('documentation opens correct article and closes');

  const southern = ['marketOperation', 'marketBoundary', 'marketStudy', 'marketAncillary', 'marketRealtime'];
  const generic = ['marketBehavior', 'marketInputs', 'market', 'marketSecurity', 'marketSettlement'];
  const moduleId = module => `.module-btn[data-module="${module}"]`;
  const directPage = await browser.newPage();
  directPage.on('pageerror', error => errors.push(error.message));
  const hashes = ['market-operation', 'southern-market', 'market-study', 'market-ancillary', 'market-realtime',
    'market-behavior', 'market-inputs', 'market-clearing', 'market-security', 'market-settlement'];
  for (const [index, module] of [...southern, ...generic].entries()) {
    await directPage.goto(`${base}/xjtu/#${hashes[index]}`);
    await directPage.reload();
    await directPage.waitForFunction(module => document.querySelector('.module-btn.active')?.dataset.module === module, module);
    // Covers the delayed first-visit tour and asynchronous application initialization.
    await directPage.waitForTimeout(1000);
    assert.equal(await directPage.locator(moduleId(module)).getAttribute('aria-selected'), 'true');
    assert.equal(await directPage.locator('.tour-overlay').count(), 0);
    assert.equal(await directPage.evaluate(() => localStorage.getItem('hysim.tourDone.v1')), null);
  }
  await directPage.close();
  checks.push('all ten direct links survive reload without unrelated first-visit tour');
  for (const width of [1440, 768, 390]) {
    await page.setViewportSize({ width, height: 960 });
    for (const family of ['southern', 'generic']) {
      await page.locator('#marketModel').selectOption(family);
      const modules = family === 'southern' ? southern : generic;
      for (const module of modules) {
        await page.locator(moduleId(module)).click();
        assert.equal(await page.locator(moduleId(module)).getAttribute('aria-selected'), 'true');
        assert.equal(await page.locator('#marketStudyWorkspace').isVisible(), module === 'marketStudy');
        assert.equal(await page.locator('#southernMarketWorkspace').isVisible(), module === 'marketBoundary');
        for (const hidden of family === 'southern' ? generic : southern) assert.equal(await page.locator(moduleId(hidden)).isVisible(), false);
        const geometry = await page.evaluate(() => {
          const main = document.getElementById('main').getBoundingClientRect();
          const nav = document.getElementById('moduleBar').getBoundingClientRect();
          const nodes = [...document.querySelectorAll('#moduleBar .module-btn')].filter(n => n.getClientRects().length);
          const overlap = nodes.some((a, i) => nodes.slice(i + 1).some(b => {
            const x = a.getBoundingClientRect(), y = b.getBoundingClientRect();
            return Math.min(x.right, y.right) - Math.max(x.left, y.left) > 1 && Math.min(x.bottom, y.bottom) - Math.max(x.top, y.top) > 1;
          }));
          return { overflow: document.documentElement.scrollWidth - innerWidth, overlap, belowNavigation: main.top >= nav.bottom - 1 };
        });
        assert.ok(geometry.overflow <= 1, `${module} ${width}: horizontal overflow ${geometry.overflow}`);
        assert.equal(geometry.overlap, false); assert.equal(geometry.belowNavigation, true);
        await page.screenshot({ path: path.join(output, `${module}-${width}.png`) });
      }
    }
    checks.push(`all ten workspaces isolated without navigation overlap at ${width}px`);
  }
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto(`${base}/xjtu/#market-operation`);
  await page.reload();
  await page.locator('#marketOperationWorkspace').waitFor({ state: 'visible' });
  assert.equal(await page.locator('#canvasContainer').isVisible(), false);
  await page.locator('#marketTopologyToggle').check();
  assert.equal(await page.locator('#marketCanvas').isVisible(), true);
  await page.locator('#marketTopologyToggle').uncheck();
  checks.push('mobile operations first, optional topology');

  // A late response from a previous page must not supply the current page's topology.
  let release;
  const held = new Promise(resolve => { release = resolve; });
  let entered;
  const started = new Promise(resolve => { entered = resolve; });
  await page.route('**/api/session/southern_market', async route => { entered(); await held; await route.continue(); });
  await page.route('**/api/session/southern_realtime', route => route.fulfill({ status: 404, body: '' }));
  await page.locator('#moduleMarketBoundary').click(); await started;
  await page.locator('#moduleMarketRealtime').click();
  await page.locator('#rtStatus').filter({ hasText: '当前服务版本未提供此市场接口' }).waitFor();
  const response = page.waitForResponse(r => r.url().endsWith('/api/session/southern_market'));
  release(); await response;
  await page.waitForFunction(() => document.getElementById('southernStatus').textContent.includes('已载入边界'));
  assert.equal(await page.locator('#marketCanvasTitle').textContent(), '市场拓扑 · 尚未载入');
  assert.equal(await page.locator('#marketCanvas').getAttribute('data-valid'), 'false');
  checks.push('404 is actionable and late boundary response cannot overwrite realtime');
  assert.deepEqual(errors, []);
  await writeFile(path.join(output, 'verification.json'), JSON.stringify({ checks, errors }, null, 2));
  console.log(JSON.stringify({ checks, errors }, null, 2));
} finally {
  await browser?.close();
  if (server.exitCode === null) { const stopped = once(server, 'exit'); server.kill('SIGTERM'); await stopped; }
}
