// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';

const ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..', '..');
const arg = (name, fallback = null) => {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && i + 1 < process.argv.length ? process.argv[i + 1] : fallback;
};
function serverPath() {
  const explicit = arg('server');
  if (explicit) return path.resolve(explicit);
  const candidate = path.join(ROOT, 'build/macos-release/tests/run_gui_server');
  if (existsSync(candidate)) return candidate;
  throw new Error('run_gui_server not found');
}
function freePort() {
  return new Promise((resolve, reject) => {
    const server = createServer();
    server.on('error', reject);
    server.listen(0, '127.0.0.1', () => {
      const address = server.address();
      server.close(() => resolve(typeof address === 'object' && address ? address.port : 0));
    });
  });
}
async function waitUp(base) {
  for (let i = 0; i < 100; ++i) {
    try { if ((await fetch(`${base}/api/cases`)).ok) return; } catch { /* wait */ }
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  throw new Error('GUI server did not start');
}

async function main() {
  const { chromium } = await import('playwright');
  const port = Number(arg('port')) || Number(await freePort());
  const base = `http://127.0.0.1:${port}`;
  const proc = spawn(serverPath(), [
    '--host', '127.0.0.1', '--port', String(port),
    '--data-dir', path.join(ROOT, 'data'), '--matpower-dir', path.join(ROOT, 'data'),
  ], { cwd: ROOT, stdio: 'ignore' });
  let browser;
  try {
    await waitUp(base);
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1440, height: 960 } });
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    await page.route('https://cdn.plot.ly/**', route => route.fulfill({
      contentType: 'application/javascript',
      body: `window.Plotly={react:(t)=>{const e=typeof t==='string'?document.getElementById(t):t;if(e){e.classList.add('js-plotly-plot');e.innerHTML='<svg data-plotly-offline-smoke="1" width="100%" height="100%"></svg>';}return Promise.resolve();},purge:()=>{},Plots:{resize(){}}};`,
    }));
    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded', timeout: 60000 });
    await page.waitForFunction(() => typeof App !== 'undefined');
    await page.evaluate(() => App.loadMatpowerCase('case9.m'));
    await page.evaluate(() => App.setActiveModule('rpo'));
    if (!(await page.locator('#moduleRpo').evaluate(el => el.classList.contains('active'))) ||
        !(await page.locator('[data-sub="rpo"]').isVisible())) {
      throw new Error('RPO module is not exposed in the steady-state workflow');
    }
    await page.locator('#rpoMaxEvaluations').fill('20');
    await page.locator('#rpoTimeLimit').fill('30');
    const responsePromise = page.waitForResponse(response =>
      response.url().includes('/api/session/run_rpo') && response.request().method() === 'POST',
      { timeout: 120000 });
    await page.locator('#btnRunRpo').click();
    const response = await responsePromise;
    const result = await response.json();
    if (!response.ok() || !result.converged || result.globally_certified !== false ||
        result.optimality_gap_available !== false ||
        !Array.isArray(result.component_coverage) || !result.component_coverage.length ||
        !result.cross_validation?.independent_opf_converged ||
        !result.cross_validation?.optimized_pf_converged) {
      throw new Error(`RPO response contract failed: ${JSON.stringify(result)}`);
    }
    await page.waitForFunction(() =>
      document.querySelector('#resultsContent')?.dataset.activeGroup === 'rpo' &&
      document.getElementById('rpoSummary')?.textContent?.includes('局部可行解') &&
      document.getElementById('rpoImpactSummary')?.textContent?.includes('电压越限母线') &&
      document.getElementById('rpoCoverageResults')?.textContent?.includes('交流母线') &&
      document.getElementById('rpoCrossValidation')?.textContent?.includes('独立OPF重算'));
    if (!await page.locator('[data-result-group="rpo"]').isVisible() ||
        !await page.locator('#rpoVoltageChart').isVisible() ||
        !await page.locator('#rpoChangeResults table').isVisible()) {
      throw new Error('RPO result group contains data but is not visibly rendered');
    }

    // Large hybrid regression: case300 previously returned empty Vm/Qg arrays
    // after an unseeded Ipopt step-computation failure, leaving both charts
    // blank and labelling partial PF coverage as a numerical disagreement.
    await page.evaluate(() => App.loadBuiltinCase('case300_acdc'));
    await page.evaluate(() => App.setActiveModule('rpo'));
    await page.waitForFunction(() =>
      document.getElementById('rpoInputSummary')?.textContent?.includes('已选OLTC'));
    const inventoryResponsePromise = page.waitForResponse(response =>
      response.url().includes('/api/session/rpo_inputs') && response.request().method() === 'POST',
      { timeout: 120000 });
    await page.locator('#btnRefreshRpoInputs').click();
    const inventoryResponse = await inventoryResponsePromise;
    const inventory = await inventoryResponse.json();
    if (!inventoryResponse.ok() || inventory.adjustable_oltc_count < 1 ||
        inventory.selected_oltc_count !== inventory.adjustable_oltc_count ||
        inventory.max_tap_move !== 2 ||
        inventory.oltc.filter(row => row.selected_for_optimization)
          .some(row => row.optimization_position_count > 5)) {
      throw new Error(`case300 RPO input inventory failed: ${JSON.stringify(inventory)}`);
    }
    const selectionResponsePromise = page.waitForResponse(response =>
      response.url().includes('/api/session/rpo_inputs') && response.request().method() === 'POST',
      { timeout: 120000 });
    await page.locator('[data-rpo-tap-toggle]:not(:disabled)').first().uncheck();
    const selectionResponse = await selectionResponsePromise;
    const selectedInventory = await selectionResponse.json();
    if (!selectionResponse.ok() ||
        selectedInventory.selected_oltc_count !== inventory.selected_oltc_count - 1) {
      throw new Error(`case300 OLTC selection was not applied: ${JSON.stringify(selectedInventory)}`);
    }
    await page.locator('#rpoMaxEvaluations').fill('1');
    await page.locator('#rpoTimeLimit').fill('30');
    await page.locator('#rpoMaxIpmIter').fill('400');
    await page.locator('#rpoStationarityTol').fill('1e-3');
    const case300ResponsePromise = page.waitForResponse(response =>
      response.url().includes('/api/session/run_rpo') && response.request().method() === 'POST',
      { timeout: 120000 });
    await page.locator('#btnRunRpo').click();
    const case300Response = await case300ResponsePromise;
    const case300 = await case300Response.json();
    if (!case300Response.ok() || !case300.converged ||
        case300.vm_after?.length !== 300 || case300.qg_after?.length !== 69 ||
        !case300.cross_validation?.independent_opf_converged ||
        !case300.cross_validation?.opf_agrees ||
        !case300.cross_validation?.overall_pass ||
        case300.n_taps !== selectedInventory.selected_oltc_count ||
        case300.control_inventory?.max_tap_move !== 2 ||
        case300.cross_validation?.pf_check_applicable !== false) {
      throw new Error(`case300 RPO response contract failed: ${JSON.stringify(case300)}`);
    }
    await page.waitForFunction(() =>
      document.querySelector('#resultsContent')?.dataset.activeGroup === 'rpo' &&
      document.getElementById('rpoInputSummary')?.textContent?.includes('已选OLTC') &&
      document.getElementById('rpoSummary')?.textContent?.includes('通过（部分覆盖）') &&
      document.getElementById('rpoChangeResults')?.textContent?.includes('母线电压') &&
      document.getElementById('rpoChangeResults')?.textContent?.includes('机组无功') &&
      document.getElementById('rpoCrossValidation')?.textContent?.includes('部分覆盖') &&
      document.getElementById('rpoVoltageChart')?.classList.contains('js-plotly-plot') &&
      document.getElementById('rpoReactiveChart')?.classList.contains('js-plotly-plot'));
    const visibleResult = await page.evaluate(() => {
      const inspect = id => {
        const element = document.querySelector(id);
        const rect = element?.getBoundingClientRect();
        return {
          width: rect?.width || 0,
          height: rect?.height || 0,
          display: element ? getComputedStyle(element).display : 'missing',
          hasDrawing: !!element?.querySelector('svg, canvas'),
        };
      };
      return {
        group: inspect('[data-result-group="rpo"]'),
        voltage: inspect('#rpoVoltageChart'),
        reactive: inspect('#rpoReactiveChart'),
      };
    });
    if (visibleResult.group.display === 'none' ||
        visibleResult.voltage.width < 100 || visibleResult.voltage.height < 100 ||
        visibleResult.reactive.width < 100 || visibleResult.reactive.height < 100 ||
        !visibleResult.voltage.hasDrawing || !visibleResult.reactive.hasDrawing) {
      throw new Error(`case300 RPO plots are not measurably visible: ${JSON.stringify(visibleResult)}`);
    }
    if (errors.length) throw new Error(`browser errors: ${errors.join(' | ')}`);
    console.log(`RPO GUI passed: case9=${result.cross_validation.overall_pass}, case300=${case300.cross_validation.validation_summary}`);
  } finally {
    if (browser) await browser.close();
    proc.kill();
  }
}

main().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
