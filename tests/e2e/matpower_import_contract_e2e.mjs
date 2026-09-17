import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import path from 'node:path';
import { chromium } from 'playwright';

const arg = (key, fallback) => {
  const index = process.argv.indexOf(`--${key}`);
  return index < 0 ? fallback : process.argv[index + 1];
};

const port = await new Promise(resolve => {
  const socket = createServer();
  socket.listen(0, '127.0.0.1', () => {
    const selected = socket.address().port;
    socket.close(() => resolve(selected));
  });
});
const base = `http://127.0.0.1:${port}`;
const server = spawn(path.resolve(arg('server', 'build/macos-release/run_gui_server')), [
  '--host', '127.0.0.1',
  '--port', String(port),
  '--data-dir', path.resolve(arg('data-dir', 'data')),
  '--matpower-dir', path.resolve(arg('matpower-dir', 'external_data/matpower')),
], { stdio: 'ignore' });

let browser;
try {
  let ready = false;
  for (let attempt = 0; attempt < 100; ++attempt) {
    try {
      ready = (await fetch(`${base}/api/cases`)).ok;
    } catch {
      // Server is still starting.
    }
    if (ready) break;
    await new Promise(resolve => setTimeout(resolve, 200));
  }
  assert.ok(ready, 'server startup');

  browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const pageErrors = [];
  const apiRequests = [];
  page.on('pageerror', error => pageErrors.push(error.message));
  page.on('request', request => {
    const url = new URL(request.url());
    if (url.pathname.startsWith('/api/')) apiRequests.push(url.pathname);
  });
  await page.addInitScript(() => localStorage.setItem('hysim.tourDone.v1', '1'));
  await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined');

  const compact = await page.evaluate(async () => {
    const response = await fetch('/api/session/load_matpower', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ filename: 'case1354pegase.m' }),
    });
    return { status: response.status, body: await response.json() };
  });
  assert.equal(compact.status, 200);
  assert.equal(compact.body.counts?.ac_buses, 1354);
  assert.equal(typeof compact.body._system_json, 'object');
  assert.equal('_raw_json' in compact.body, false);
  assert.equal('ac_buses' in compact.body, false);

  apiRequests.length = 0;
  const small = await page.evaluate(() => App.loadMatpowerCase('case14.m'));
  assert.equal(small.counts?.ac_buses, 14);
  assert.equal(typeof small._raw_json, 'string');
  assert.deepEqual(apiRequests.filter(route => route === '/api/session/pf'), []);
  assert.match(await page.locator('#modelIoResults').innerText(), /潮流尚未计算/);
  assert.deepEqual(pageErrors, []);

  console.log('MATPOWER import contract passed');
} finally {
  if (browser) await browser.close();
  server.kill('SIGTERM');
  await new Promise(resolve => {
    const timer = setTimeout(() => {
      server.kill();
      resolve();
    }, 5000);
    server.once('exit', () => {
      clearTimeout(timer);
      resolve();
    });
  });
}
