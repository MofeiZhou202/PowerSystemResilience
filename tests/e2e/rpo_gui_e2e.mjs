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
      body: `window.Plotly={react:(t)=>{const e=typeof t==='string'?document.getElementById(t):t;e?.classList.add('js-plotly-plot');},purge:()=>{},Plots:{resize(){}}};`,
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
      document.getElementById('rpoCoverageResults')?.textContent?.includes('交流母线') &&
      document.getElementById('rpoCrossValidation')?.textContent?.includes('独立OPF重算'));
    if (errors.length) throw new Error(`browser errors: ${errors.join(' | ')}`);
    console.log(`RPO GUI passed: status=${result.status}, coverage=${result.component_coverage.length}, cross=${result.cross_validation.overall_pass}`);
  } finally {
    if (browser) await browser.close();
    proc.kill();
  }
}

main().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
