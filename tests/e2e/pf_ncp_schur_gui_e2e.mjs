// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');

function arg(name) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length ? process.argv[index + 1] : null;
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
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
  for (let attempt = 0; attempt < 100; attempt += 1) {
    try {
      if ((await fetch(`${base}/api/cases`)).ok) return;
    } catch { /* server is starting */ }
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  throw new Error('GUI server did not start');
}

async function main() {
  const serverPath = arg('server');
  assert(serverPath, '--server is required');
  const port = Number(await freePort());
  const base = `http://127.0.0.1:${port}`;
  const server = spawn(path.resolve(serverPath), [
    '--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(ROOT, 'data'),
  ], { cwd: ROOT, stdio: 'ignore' });
  let browser;
  try {
    await waitUp(base);
    const { chromium } = await import('playwright');
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    const pageErrors = [];
    const pfRequests = [];
    page.on('pageerror', error => pageErrors.push(error.message));
    page.on('request', request => {
      if (request.method() === 'POST' && request.url().endsWith('/api/session/pf')) {
        pfRequests.push(request.postDataJSON());
      }
    });
    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App?.loadBuiltinCase === 'function');
    await page.evaluate(() => App.loadBuiltinCase('case2000_acdc'));
    await page.evaluate(() => App.setActiveModule('powerFlow'));
    await page.click('#btnPowerFlow');
    await page.waitForFunction(() =>
      document.getElementById('resultsSummary')?.textContent?.includes('✓ 是'));
    const case2000Result = await page.evaluate(() => ({
      summary: document.getElementById('resultsSummary')?.textContent || '',
      effective: document.getElementById('pfNcpSchurEffective')?.textContent || '',
    }));
    const defaultRobust = pfRequests.at(-1)?.options?.robust_nonlinear || {};
    assert(!('enable_auto_fallback_scheduling' in defaultRobust),
      `default GUI request must preserve backend fallback policy: ${JSON.stringify(defaultRobust)}`);
    assert(case2000Result.summary.includes('hybrid_ac_dc_newton') &&
      case2000Result.summary.includes('✓ 是'),
    `case2000_acdc did not converge through the normal GUI path: ${case2000Result.summary}`);
    assert(case2000Result.effective.includes('自动回退=开'),
      `backend fallback default was not rendered honestly: ${case2000Result.effective}`);

    const headlessState = await page.evaluate(() => ({
      headless: Canvas.isHeadless(),
      overview: Canvas.getNetworkOverviewStats(),
    }));
    assert(headlessState.headless && headlessState.overview?.active,
      `case2000 must exercise the WebGL/headless navigation path: ${JSON.stringify(headlessState)}`);

    await page.click('.panel-tab[data-tab="topology"]');
    const topologyAcBus = await page.locator('#busTableInner tbody tr[data-row="0"] td').first().textContent();
    await page.locator('#busTableInner tbody tr[data-row="0"]').click();
    const topologySelection = await page.evaluate(() => Canvas.getNetworkOverviewStats()?.selected);
    assert(topologySelection?.domain === 'ac' &&
      topologySelection.index === Number(topologyAcBus),
    `topology row did not select its stable AC bus: row=${topologyAcBus}, selected=${JSON.stringify(topologySelection)}`);
    assert(await page.locator('#busTableInner tbody tr.topo-search-highlight').count() === 1,
      'topology navigation did not visibly highlight the selected row');

    await page.click('.panel-tab[data-tab="results"]');
    const acResultRow = page.locator('#pfBusResults tbody tr[data-result-ref]').first();
    const acResultBus = Number(await acResultRow.locator('td').first().textContent());
    await acResultRow.click();
    const acResultSelection = await page.evaluate(() => Canvas.getNetworkOverviewStats()?.selected);
    assert(acResultSelection?.domain === 'ac' && acResultSelection.index === acResultBus,
      `PF AC result did not select its stable bus: row=${acResultBus}, selected=${JSON.stringify(acResultSelection)}`);

    const dcResultRow = page.locator('#pfDcBusResults tbody tr[data-result-ref]').first();
    const dcResultBus = Number(await dcResultRow.locator('td').first().textContent());
    await dcResultRow.click();
    const dcResultSelection = await page.evaluate(() => Canvas.getNetworkOverviewStats()?.selected);
    assert(dcResultSelection?.domain === 'dc' && dcResultSelection.index === dcResultBus,
      `PF DC result did not preserve its domain-qualified bus identity: row=${dcResultBus}, selected=${JSON.stringify(dcResultSelection)}`);

    const vscResultRow = page.locator('#pfVscResults tbody tr[data-result-ref]').first();
    const vscAcBus = Number(await vscResultRow.locator('td').nth(1).textContent());
    await vscResultRow.click();
    const vscResultSelection = await page.evaluate(() => Canvas.getNetworkOverviewStats()?.selected);
    assert(vscResultSelection?.domain === 'ac' && vscResultSelection.index === vscAcBus,
      `PF VSC result did not resolve to its stable AC terminal: bus=${vscAcBus}, selected=${JSON.stringify(vscResultSelection)}`);

    await page.evaluate(() => App.loadBuiltinCase('gfm_norton_limit_demo'));
    await page.evaluate(() => App.setActiveModule('powerFlow'));
    await page.click('#btnPfAdvanced');
    await page.selectOption('#pfSmoothNcpMode', 'true');
    await page.selectOption('#pfVscSchurMode', 'false');
    await page.fill('#pfVscSchurMinDimension', '321');
    await page.fill('#pfVscSchurRcondTol', '1e-7');
    await page.fill('#pfVscSchurBackwardErrorTol', '2e-11');
    await page.fill('#pfNcpMu0', '0.002');
    await page.fill('#pfNcpMuMin', '3e-11');
    await page.fill('#pfNcpMuFactor', '0.2');
    await page.click('#btnPowerFlow');
    await page.waitForFunction(() =>
      document.getElementById('pfNcpSchurEffective')?.textContent?.includes('最小网络维数=321'));
    await page.locator('#pfSolverStructureSection').waitFor({ state: 'visible' });
    const result = await page.evaluate(() => ({
      effective: document.getElementById('pfNcpSchurEffective')?.textContent || '',
      certificate: document.getElementById('pfSolverStructureResults')?.textContent || '',
      vsc: document.getElementById('pfVscResults')?.textContent || '',
      pageOverflow: Math.max(0, document.documentElement.scrollWidth - window.innerWidth),
    }));
    assert(result.effective.includes('平滑NCP=开') && result.effective.includes('局部Schur=关') &&
      result.effective.includes('2.000e-3→3.000e-11'),
    `effective policy did not round-trip: ${result.effective}`);
    assert(result.certificate.includes('未尝试') && result.certificate.includes('Clarke/B-子微分'),
      `solver certificate is incomplete: ${result.certificate}`);
    assert(result.vsc.includes('GFM Norton') && result.vsc.includes('NCP残差'),
      `VSC limit result is incomplete: ${result.vsc}`);
    const sentRobust = pfRequests.at(-1)?.options?.robust_nonlinear || {};
    assert(sentRobust.enable_smooth_ncp === true && sentRobust.enable_vsc_local_schur === false &&
      sentRobust.ncp_mu0 === 0.002 && sentRobust.ncp_mu_factor === 0.2,
    `explicit PF numerical overrides were not sent: ${JSON.stringify(sentRobust)}`);
    assert(!('ncp_mu_factor_coarse' in sentRobust) && !('ncp_mu_phase_transition' in sentRobust),
      `blank expert fields must preserve sparse intent: ${JSON.stringify(sentRobust)}`);

    await page.selectOption('#pfVscSchurMode', 'true');
    await page.fill('#pfVscSchurMinDimension', '0');
    const staleCertificate = await page.evaluate(() => ({
      effective: document.getElementById('pfNcpSchurEffective')?.textContent || '',
      certificate: document.getElementById('pfSolverStructureResults')?.textContent || '',
      stale: document.getElementById('pfSolverStructureSection')?.classList.contains('solver-certificate-stale'),
    }));
    assert(staleCertificate.stale && staleCertificate.effective.includes('已失效') &&
      staleCertificate.certificate.includes('请重新运行潮流'),
    `edited numerical controls did not invalidate the old certificate: ${JSON.stringify(staleCertificate)}`);
    await page.selectOption('#pfDisplayUnit', 'kW');
    const staleAfterUnitChange = await page.evaluate(() => ({
      effective: document.getElementById('pfNcpSchurEffective')?.textContent || '',
      certificate: document.getElementById('pfSolverStructureResults')?.textContent || '',
      stale: document.getElementById('pfSolverStructureSection')?.classList.contains('solver-certificate-stale'),
    }));
    assert(staleAfterUnitChange.stale && staleAfterUnitChange.effective.includes('已失效') &&
      staleAfterUnitChange.certificate.includes('请重新运行潮流'),
    `display-only rerender revived a stale certificate: ${JSON.stringify(staleAfterUnitChange)}`);

    await page.click('#btnPowerFlow');
    await page.waitForFunction(() =>
      document.getElementById('pfNcpSchurEffective')?.textContent?.includes('局部Schur=开'));
    const schurOn = await page.evaluate(() => ({
      effective: document.getElementById('pfNcpSchurEffective')?.textContent || '',
      certificate: document.getElementById('pfSolverStructureResults')?.textContent || '',
      stale: document.getElementById('pfSolverStructureSection')?.classList.contains('solver-certificate-stale'),
    }));
    assert(!schurOn.stale && schurOn.certificate.includes('证书通过并采用局部Schur') &&
      schurOn.certificate.includes('7 / 7 / 0'),
    `Schur-on rerun did not replace the stale certificate: ${JSON.stringify(schurOn)}`);

    await page.selectOption('#pfVscSchurMode', 'false');
    assert(await page.locator('#pfSolverStructureResults').textContent().then(text => text.includes('已失效')),
      'second control edit did not invalidate the accepted certificate');
    await page.click('#btnPowerFlow');
    await page.waitForFunction(() =>
      document.getElementById('pfNcpSchurEffective')?.textContent?.includes('局部Schur=关') &&
      document.getElementById('pfSolverStructureResults')?.textContent?.includes('未尝试'));
    const schurOff = await page.locator('#pfSolverStructureResults').textContent();
    assert(schurOff.includes('0 / 0 / 0') && !schurOff.includes('已失效'),
      `Schur-off rerun did not refresh the certificate: ${schurOff}`);
    assert(result.pageOverflow === 0, `desktop page overflowed by ${result.pageOverflow}px`);

    await page.setViewportSize({ width: 390, height: 844 });
    await page.evaluate(() => window.dispatchEvent(new Event('resize')));
    const mobileOverflow = await page.evaluate(() =>
      Math.max(0, document.documentElement.scrollWidth - window.innerWidth));
    assert(mobileOverflow === 0, `mobile page overflowed by ${mobileOverflow}px`);
    assert(pageErrors.length === 0, `page errors: ${pageErrors.join(' | ')}`);
    console.log('case2000 hybrid PF and NCP/Schur GUI contracts passed');
  } finally {
    if (browser) await browser.close();
    server.kill();
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
