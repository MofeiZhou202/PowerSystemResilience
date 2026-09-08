import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { mkdir, writeFile } from 'node:fs/promises';
import { once } from 'node:events';
import assert from 'node:assert/strict';
import path from 'node:path';
import { chromium } from 'playwright';

const root = process.cwd(), output = path.join(root, 'output/market-activity');
const arg = process.argv.indexOf('--server');
const port = await new Promise(resolve => {
  const s = createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); });
});
const base = `http://127.0.0.1:${port}`;
const server = spawn(arg < 0 ? 'build/macos-release/tests/run_gui_server' : process.argv[arg + 1],
  ['--host', '127.0.0.1', '--port', String(port), '--data-dir', 'data'], { cwd: root, stdio: 'ignore' });
let browser;
const errors = [], checks = [];
try {
  await mkdir(output, { recursive: true });
  for (let n = 0; n < 100; n++) {
    try { if ((await fetch(`${base}/api/cases`)).ok) break; } catch {}
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  await page.addInitScript(() => localStorage.setItem('hysim.tourDone.v1', '1'));
  page.on('pageerror', e => errors.push(e.message));
  let forecastReads = 0;
  page.on('request', r => { if (r.method() === 'GET' && r.url().endsWith('/api/session/market_forecast')) forecastReads++; });
  await page.goto(`${base}/xjtu/#market-operation`);
  await page.waitForFunction(() => document.getElementById('forecastSolver')?.options.length === 3);
  await page.waitForFunction(() => !document.querySelector('#marketActivity [data-state="pending"]'));
  assert.equal(forecastReads, 1);
  checks.push('one forecast GET on initial market entry');
  await page.locator('#operationCase').selectOption('demo');
  await page.locator('#operationLoadCase').click();
  await page.locator('#operationManualMode').click();
  await page.locator('#operationHorizon').selectOption('day');
  await page.locator('#operationExplain').uncheck();
  await page.locator('#operationNextStep').click();
  assert.match(await page.locator('#operationRunSummary').innerText(), /1 场景 × 1 日 = 1 个日窗/);

  let release, entered;
  const gate = new Promise(resolve => { release = resolve; });
  const arrived = new Promise(resolve => { entered = resolve; });
  let stepCount = 0;
  await page.route('**/api/session/market_operation', async route => {
    if (route.request().method() === 'POST' && route.request().postDataJSON().action === 'step') {
      stepCount++; entered(); await gate;
    }
    await route.continue();
  });
  // Session busy is intentionally global; it does not prove a particular solver stage.
  await page.route('**/api/session/status', route => route.fulfill({ json: { busy: true, cancel: false } }));
  await page.locator('#operationRun').click(); await arrived;
  const activity = page.locator('#marketActivity [data-state="pending"]').filter({ hasText: '第 1/1 日' });
  await activity.filter({ hasText: '服务端有计算在执行（会话状态）' }).waitFor();
  const before = await activity.innerText();
  await page.waitForTimeout(1200);
  assert.notEqual(await activity.innerText(), before);
  assert.equal(await activity.locator('progress[value]').count(), 0);
  assert.match(await activity.innerText(), /等待服务端返回/);
  assert.ok(await activity.evaluate(row => {
    const box = row.getBoundingClientRect(), parent = row.parentElement.getBoundingClientRect();
    return box.top >= parent.top && box.bottom <= parent.bottom + 1;
  }), 'active task must not be clipped behind completed loads');
  await page.locator('#operationPause').click();
  assert.equal(await activity.isVisible(), true);
  assert.match(await page.locator('#operationStatus').innerText(), /等待当日计算结束/);
  await page.screenshot({ path: path.join(output, 'pending-desktop.png') });
  await page.setViewportSize({ width: 390, height: 844 });
  assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth + 1));
  await page.screenshot({ path: path.join(output, 'pending-mobile.png') });
  await page.unroute('**/api/session/status');
  await page.route('**/api/session/status', route => route.fulfill({ status: 503, body: '' }));
  await activity.filter({ hasText: '暂未收到服务端状态' }).waitFor();
  assert.equal(await activity.getAttribute('aria-busy'), 'true');
  checks.push('elapsed time, honest session heartbeat, pending pause and failed heartbeat at desktop/mobile');
  release();
  await page.waitForFunction(() => document.getElementById('operationStatus').textContent.includes('已完成 1/1'));
  assert.equal(stepCount, 1);
  await page.locator('#operationPlanGeneration .main-svg').first().waitFor({ state: 'attached' });
  assert.equal(await page.locator('#marketActivity [data-state="pending"]').count(), 0);
  checks.push('actual day result and deferred plan charts appear after response; pause did not submit twice');

  await page.unroute('**/api/session/market_operation');
  await page.route('**/api/session/market_operation', route => route.abort('failed'));
  await page.locator('[data-operation-step="1"]').click();
  await page.locator('#operationReload').click();
  await page.locator('#marketActivity [data-state="error"]').filter({ hasText: '连接中断' }).waitFor();
  assert.equal(await page.locator('#operationReload').isDisabled(), false);
  checks.push('network failure clears activity and allows reload without retrying mutation');
  assert.deepEqual(errors, []);
  await writeFile(path.join(output, 'verification.json'), JSON.stringify({ checks, errors }, null, 2));
  console.log(JSON.stringify({ checks, errors }, null, 2));
} finally {
  await browser?.close();
  if (server.exitCode === null) { const stopped = once(server, 'exit'); server.kill('SIGTERM'); await stopped; }
}
