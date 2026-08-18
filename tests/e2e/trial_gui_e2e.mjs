// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');

function arg(name) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 ? process.argv[index + 1] : null;
}

function freePort() {
  return new Promise((resolve, reject) => {
    const server = createServer();
    server.on('error', reject);
    server.listen(0, '127.0.0.1', () => {
      const address = server.address();
      server.close(() => resolve(typeof address === 'object' ? address.port : 0));
    });
  });
}

async function waitUp(base) {
  for (let attempt = 0; attempt < 200; ++attempt) {
    try { if ((await fetch(`${base}/api/edition`)).ok) return; } catch { /* starting */ }
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  throw new Error('Trial GUI server did not start');
}

async function main() {
  const { chromium } = await import('playwright');
  const server = arg('server');
  if (!server) throw new Error('--server is required');
  const port = Number(await freePort());
  const base = `http://127.0.0.1:${port}`;
  const child = spawn(path.resolve(server), ['--host', '127.0.0.1', '--port', String(port),
    '--data-dir', path.join(ROOT, 'data')], { cwd: ROOT, stdio: 'ignore' });
  let browser;
  try {
    await waitUp(base);
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    await page.goto(`${base}/xjtu/`, { waitUntil: 'networkidle' });
    await page.waitForSelector('body[data-edition="trial"]');
    const expectedWorkflows = ['模型建立', '参数校核', '指标设计', '全景仿真', '薄弱辨识'];
    const workflows = await page.locator('.workflow-btn').allTextContents();
    if (JSON.stringify(workflows.map(v => v.trim())) !== JSON.stringify(expectedWorkflows)) {
      throw new Error(`workflow mismatch: ${workflows}`);
    }
    if (await page.locator('[data-trial-indicator]').count() !== 8) throw new Error('indicator count mismatch');
    if (await page.locator('#editionBadge').textContent() !== 'TRIAL') throw new Error('Trial badge missing');
    const disabledCount = await page.locator('[data-module="rpo"], [data-module="harmonics"], [data-module="transient"], [data-module="timeSeries"], [data-module="market"], [data-module="integratedEnergy"], [data-module="evTraffic"]').count();
    if (disabledCount) throw new Error(`disabled modules remain in DOM: ${disabledCount}`);

    await page.locator('.workflow-btn[data-workflow="indicator_design"]').click();
    const indicators = page.locator('[data-trial-indicator]');
    for (let index = 0; index < await indicators.count(); ++index) {
      await indicators.nth(index).uncheck();
    }
    await page.locator('[data-trial-indicator-id="system_economic"]').check();
    await page.waitForFunction(() => document.querySelectorAll('#trialAnalysisPlan .trial-plan-step').length === 4);
    const modules = await page.locator('#trialAnalysisPlan .trial-plan-step').evaluateAll(
      buttons => buttons.map(button => button.dataset.module));
    if (JSON.stringify(modules) !== JSON.stringify(['powerFlow', 'opf', 'hosting', 'shortCircuit'])) {
      throw new Error(`backend plan mismatch: ${modules}`);
    }
    await page.locator('#trialAnalysisPlan [data-module="opf"]').click();
    if (!await page.locator('#moduleOpf').evaluate(element => element.classList.contains('active'))) {
      throw new Error('analysis plan step did not navigate to OPF');
    }
    for (const viewport of [{ width: 1440, height: 1000 }, { width: 390, height: 844 }]) {
      await page.setViewportSize(viewport);
      const overflow = await page.evaluate(() => document.documentElement.scrollWidth > document.documentElement.clientWidth + 1);
      if (overflow) throw new Error(`page-level horizontal overflow at ${viewport.width}px`);
    }
    console.log('Trial GUI profile and backend analysis plan passed');
  } finally {
    if (browser) await browser.close();
    child.kill();
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
