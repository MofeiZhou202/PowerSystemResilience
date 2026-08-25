// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');

function arg(name, fallback = null) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length ? process.argv[index + 1] : fallback;
}

function serverPath() {
  const explicit = arg('server');
  if (explicit) return path.resolve(explicit);
  for (const relative of ['build/macos-release/tests/run_gui_server',
                          'build/macos-release/run_gui_server']) {
    const candidate = path.join(ROOT, relative);
    if (existsSync(candidate)) return candidate;
  }
  throw new Error('run_gui_server not found');
}

function freePort() {
  return new Promise((resolve, reject) => {
    const server = createServer();
    server.on('error', reject);
    server.listen(0, '127.0.0.1', () => {
      const address = server.address();
      const port = typeof address === 'object' && address ? address.port : 0;
      server.close(() => resolve(port));
    });
  });
}

async function waitUp(base) {
  for (let attempt = 0; attempt < 100; ++attempt) {
    try {
      if ((await fetch(`${base}/api/cases`)).ok) return;
    } catch {}
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  throw new Error('GUI server did not start');
}

async function post(base, endpoint, body) {
  const response = await fetch(`${base}${endpoint}`, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await response.json();
  if (!response.ok || data.error) {
    throw new Error(`${endpoint}: ${data.error || response.statusText}`);
  }
  return data;
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

async function main() {
  const { chromium } = await import('playwright');
  const port = Number(arg('port')) || Number(await freePort());
  const base = `http://127.0.0.1:${port}`;
  const proc = spawn(serverPath(), ['--host', '127.0.0.1', '--port', String(port),
    '--data-dir', path.join(ROOT, 'data')], { cwd: ROOT, stdio: 'ignore' });
  let browser;
  try {
    await waitUp(base);
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    const pageErrors = [];
    page.on('pageerror', error => pageErrors.push(String(error?.stack || error)));
    await page.goto(`${base}/xjtu/`, { waitUntil: 'networkidle' });
    await page.evaluate(async () => {
      await App.loadBuiltinCase('dist33_tie_demo');
      App.setActiveModule('reliability');
    });
    await page.waitForFunction(() =>
      document.getElementById('relWorkflowComponents')?.textContent !== '待同步');

    await page.locator('#relDataPolicy').selectOption('case_data_only');
    await page.locator('#relPhysicalModel').selectOption('restoration_milp');
    assert(await page.locator('#relParallel').evaluate(element => !element.checked),
      'three-stage GUI did not default to serial fault evaluation');
    await page.locator('#btnRunReliability').click();
    await page.waitForTimeout(1000);
    const strictRun = await page.evaluate(() => ({
      state: document.getElementById('relWorkflowRunStatus')?.dataset.state,
      status: document.getElementById('statusText')?.textContent || '',
      policy: document.getElementById('relDataPolicy')?.value,
      advice: document.getElementById('relParameterAdviceText')?.textContent || '',
      configuration: document.querySelector(
        '#relWorkflowStages > li[data-rel-stage="configuration"] small')?.textContent || '',
    }));
    assert(strictRun.state === 'error' && strictRun.configuration.includes('严格策略'),
      `strict missing-data policy did not explain its rejection: ${JSON.stringify(strictRun)}`);

    await page.locator('#relDataPolicy').selectOption('missing_only');
    await page.locator('#btnRunReliability').click();
    await page.waitForFunction(() => {
      const state = document.getElementById('relWorkflowRunStatus')?.dataset.state;
      return state === 'complete' || state === 'limited' || state === 'failed';
    }, null, { timeout: 120000 });
    const result = await page.evaluate(() => ({
      state: document.getElementById('relWorkflowRunStatus')?.dataset.state,
      resultVisible: !!document.querySelector('#reliabilityResults .reliability-kpi-grid'),
      protectionPanel: document.getElementById('reliabilityResults')?.textContent
        ?.includes('保护配置与故障恢复') || false,
      advice: document.getElementById('relParameterAdviceText')?.textContent || '',
    }));
    assert(result.state !== 'failed' && result.resultVisible && result.protectionPanel &&
      result.advice.includes('统一回退值'),
    `default missing-data policy did not complete Dist33: ${JSON.stringify(result)}`);
    assert(pageErrors.length === 0, `GUI raised page errors: ${JSON.stringify(pageErrors)}`);

    const legacy = await post(base, '/api/session/run_reliability_three_stage', {
      parallel: false,
      max_switch_operations: 2,
    });
    assert(legacy.method === 'three_stage' &&
      legacy.reliability_configuration?.limitation === null,
    `legacy route did not preserve an empty configuration as JSON null: ${JSON.stringify({
      method: legacy.method,
      configuration: legacy.reliability_configuration,
    })}`);

    console.log(JSON.stringify({ case: 'dist33_tie_demo', strict_status: strictRun.configuration,
      default_policy_state: result.state, protection_panel: result.protectionPanel,
      legacy_empty_configuration: legacy.reliability_configuration?.limitation === null }));
  } finally {
    if (browser) await browser.close();
    proc.kill();
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
