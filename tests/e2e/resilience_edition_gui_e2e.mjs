// @ts-check
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const WEB = path.join(ROOT, 'web');

const WORKFLOW = [
  { id: 'modeling', label: '模型建立' },
  { id: 'parameter_validation', label: '参数校核' },
  { id: 'indicator_design', label: '指标设计' },
  { id: 'panoramic_simulation', label: '全景仿真' },
  { id: 'weak_link_identification', label: '薄弱辨识' },
];
const INDICATORS = [
  { id: 'system_economic', level: 'system', dimension: 'economic', label: '系统经济性' },
  { id: 'user_economic', level: 'user', dimension: 'economic', label: '用户经济性' },
  { id: 'system_carbon', level: 'system', dimension: 'carbon', label: '系统碳指标' },
  { id: 'user_carbon', level: 'user', dimension: 'carbon', label: '用户碳指标' },
  { id: 'system_reliability', level: 'system', dimension: 'reliability', label: '系统可靠性' },
  { id: 'user_reliability', level: 'user', dimension: 'reliability', label: '用户可靠性' },
  { id: 'system_resilience', level: 'system', dimension: 'resilience', label: '系统弹性' },
  { id: 'user_resilience', level: 'user', dimension: 'resilience', label: '用户弹性' },
];
const ANALYSES = ['power_flow', 'optimal_power_flow'];
const FRONTEND = {
  full: ['*'],
  trial: ['modelIO', 'parameterLibrary', 'topologyAnalysis', 'indicatorDesign',
    'scenarioGeneration', 'powerFlow', 'opf', 'shortCircuit', 'hosting',
    'reliability', 'resilience', 'carbonFlow', 'weakLinks'],
  resilience: ['modelIO', 'parameterLibrary', 'topologyAnalysis',
    'scenarioGeneration', 'powerFlow', 'opf', 'resilience',
    'proactiveDefense', 'rapidRecovery', 'resilienceMetrics'],
};
const ENABLED = {
  full: ['*'],
  trial: ['model_io', 'parameter_validation', 'topology_analysis', 'indicator_design',
    'scenario_generation', 'power_flow', 'opf', 'line_loss', 'carbon_flow',
    'voltage_compliance', 'hosting_capacity', 'reliability', 'resilience',
    'short_circuit', 'multidimensional_weak_links'],
  resilience: ['model', 'model_io', 'parameter_validation', 'projection', 'attribution',
    'graph', 'topology', 'power_flow', 'optimal_power_flow', 'reliability',
    'resilience', 'network_reconfiguration', 'scenario_generation', 'typhoon_faults',
    'short_circuit', 'resilience_profiles', 'mipsolvers'],
};
const DISABLED = {
  full: [],
  trial: ['reactive_power_optimization', 'harmonics', 'dynamics', 'market',
    'integrated_energy', 'ev_traffic', 'time_series', 'network_reconfiguration',
    'counterfactual_planning', 'sppt_agent', 'advanced_io'],
  resilience: ['reactive_power_optimization', 'harmonics', 'dynamics', 'market', 'carbon',
    'integrated_energy', 'ev_traffic', 'time_series', 'hosting_capacity', 'weak_links',
    'counterfactual_planning', 'sppt_agent', 'advanced_io'],
};
const IO = { full: ['*'], trial: ['json', 'matpower', 'gridlabd', 'opendss'], resilience: ['json', 'matpower'] };

function profile(edition) {
  const value = {
    schema: 'hacdcpf.edition-profile.v1',
    edition,
    product_name: { full: 'HySim-XJTU-HRPES', trial: 'HySim-XJTU-HRPES Trial', resilience: 'PowerSystemResilience' }[edition],
    analyses: [...ANALYSES],
    analysis_catalog: { schema: 'hacdcpf.edition-analysis-catalog.v1', entries: [] },
    workflow: structuredClone(WORKFLOW),
    indicators: structuredClone(edition === 'resilience' ? INDICATORS.filter(row => row.dimension !== 'carbon') : INDICATORS),
    enabled_modules: [...ENABLED[edition]],
    frontend_modules: [...FRONTEND[edition]],
    enabled_io_formats: [...IO[edition]],
    disabled_features: [...DISABLED[edition]],
  };
  if (edition !== 'full') value.route_policy = { mode: 'fail_closed', unknown_api_routes: 'disabled' };
  if (edition === 'resilience') Object.assign(value, {
    solver_capabilities: ['ac_power_flow', 'dc_optimal_power_flow', 'ac_optimal_power_flow', 'highs', 'native_branch_and_cut', 'aml'],
    model_scope: 'AC/DC hybrid resilience assessment and restoration; capability-specific limitations remain part of each result.',
    limitations: ['Standalone transient and small-signal research endpoints are disabled in this first release.'],
    restoration_certification: { ordinary_feasibility_is_certified_safe: false, dynamic_certification: 'not_exposed_in_first_release' },
  });
  return value;
}

function check(condition, message) {
  if (!condition) throw new Error(message);
}

function contentType(file) {
  if (file.endsWith('.html')) return 'text/html; charset=utf-8';
  if (file.endsWith('.js')) return 'text/javascript; charset=utf-8';
  if (file.endsWith('.css')) return 'text/css; charset=utf-8';
  if (file.endsWith('.json')) return 'application/json; charset=utf-8';
  if (file.endsWith('.svg')) return 'image/svg+xml';
  if (file.endsWith('.wasm')) return 'application/wasm';
  return 'application/octet-stream';
}

async function startServer() {
  const server = createServer(async (request, response) => {
    const url = new URL(request.url || '/', 'http://127.0.0.1');
    if (url.pathname === '/') {
      response.writeHead(302, { Location: '/xjtu/' }); response.end(); return;
    }
    if (url.pathname === '/api/edition') {
      const behavior = url.searchParams.get('fixture');
      if (behavior === 'hung') {
        response.writeHead(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
        response.write('{"schema":');
        return;
      }
      response.writeHead(404); response.end('profile mock missing'); return;
    }
    if (!url.pathname.startsWith('/xjtu/')) {
      response.writeHead(404); response.end('not found'); return;
    }
    const relative = decodeURIComponent(url.pathname.slice('/xjtu/'.length)) || 'index.html';
    const file = path.resolve(WEB, relative);
    if (!file.startsWith(`${WEB}${path.sep}`) && file !== path.join(WEB, 'index.html')) {
      response.writeHead(403); response.end('forbidden'); return;
    }
    try {
      const bytes = await readFile(file);
      response.writeHead(200, { 'Content-Type': contentType(file), 'Cache-Control': 'no-store' });
      response.end(bytes);
    } catch {
      response.writeHead(404); response.end('not found');
    }
  });
  await new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(0, '127.0.0.1', resolve);
  });
  const address = server.address();
  return { server, base: `http://127.0.0.1:${typeof address === 'object' && address ? address.port : 0}` };
}

async function installMocks(page, scenario) {
  await page.addInitScript(() => {
    try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {}
    window.Plotly = { purge() {}, newPlot() {}, react() {}, Plots: { resize() {} } };
  });
  await page.route('https://cdn.plot.ly/**', route => route.fulfill({
    status: 200, contentType: 'text/javascript', body: 'window.Plotly=window.Plotly||{purge(){},newPlot(){},react(){},Plots:{resize(){}}};',
  }));
  await page.route('**/api/**', async route => {
    const url = new URL(route.request().url());
    if (url.pathname === '/api/edition') {
      if (scenario.mode === 'delayed') {
        await new Promise(resolve => setTimeout(resolve, 350));
        await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(scenario.profile) });
      } else if (scenario.mode === 'hung') {
        await route.fallback({ url: `${url.origin}/api/edition?fixture=hung` });
      } else if (scenario.mode === 'http') {
        await route.fulfill({ status: 503, contentType: 'application/json', body: '{"error":"offline"}' });
      } else if (scenario.mode === 'json') {
        await route.fulfill({ status: 200, contentType: 'application/json', body: '{not-json' });
      } else {
        await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(scenario.profile) });
      }
      return;
    }
    if (url.pathname === '/api/cases') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: '{"cases":[]}' }); return;
    }
    if (url.pathname === '/api/matpower_files') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: '{"files":[]}' }); return;
    }
    if (url.pathname === '/api/dynamics/model_schema') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: '{"models":[],"components":[]}' }); return;
    }
    if (url.pathname === '/api/edition/analysis_plan') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: '{"steps":[]}' }); return;
    }
    await route.fulfill({ status: 200, contentType: 'application/json', body: '{}' });
  });
}

function apiPaths(requests) {
  return requests.filter(value => value.includes('/api/')).map(value => new URL(value).pathname);
}

async function shellState(page) {
  return page.evaluate(() => {
    const shell = document.getElementById('appShell');
    const status = document.getElementById('startupStatus');
    return {
      startup: document.body.dataset.startupState,
      shellHidden: shell?.hidden,
      inert: shell?.inert,
      ariaHidden: shell?.getAttribute('aria-hidden'),
      statusHidden: status?.hidden,
      statusState: status?.dataset.state,
      statusText: document.getElementById('startupStatusDetail')?.textContent || '',
    };
  });
}

async function assertNoOverflow(page, label) {
  const overflow = await page.evaluate(() => ({
    root: document.documentElement.scrollWidth - document.documentElement.clientWidth,
    body: document.body.scrollWidth - document.body.clientWidth,
  }));
  check(overflow.root <= 1 && overflow.body <= 1, `${label}: page-level horizontal overflow ${JSON.stringify(overflow)}`);
}

async function waitUnavailable(page, timeout = 5000) {
  await page.waitForFunction(() => document.body.dataset.startupState === 'unavailable', null, { timeout });
}

async function runFailure(page, base, scenario, viewport) {
  const requests = [];
  page.on('request', request => requests.push(request.url()));
  await installMocks(page, scenario);
  await page.goto(`${base}/xjtu/#market-realtime`, { waitUntil: 'domcontentloaded' });
  await waitUnavailable(page, scenario.mode === 'hung' ? 5000 : 2500);
  const state = await shellState(page);
  check(state.shellHidden && state.inert && state.ariaHidden === 'true', `${scenario.name}: shell escaped lockdown`);
  check(!state.statusHidden && state.statusState === 'unavailable', `${scenario.name}: unavailable status missing`);
  const paths = apiPaths(requests);
  check(paths.length === 1 && paths[0] === '/api/edition', `${scenario.name}: non-profile API request ${paths}`);
  check(!paths.some(value => value.includes('/dynamics/')), `${scenario.name}: dynamics requested`);
  check(await page.locator('.module-btn.active').count() === 0, `${scenario.name}: forbidden hash activated a module`);
  await assertNoOverflow(page, `${scenario.name}/${viewport.width}`);
}

async function runValid(page, base, edition, viewport, { delayed = false, hash = '' } = {}) {
  const requests = [];
  page.on('request', request => requests.push(request.url()));
  const value = profile(edition);
  await installMocks(page, { name: `${edition}-${delayed ? 'delayed' : 'valid'}`, mode: delayed ? 'delayed' : 'valid', profile: value });
  const navigation = page.goto(`${base}/xjtu/${hash}`, { waitUntil: 'domcontentloaded' });
  if (delayed) {
    await page.waitForSelector('body[data-startup-state="loading"]');
    const loading = await shellState(page);
    check(loading.shellHidden && loading.inert && loading.ariaHidden === 'true', `${edition}: delayed profile did not lock shell`);
    check(apiPaths(requests).every(pathname => pathname === '/api/edition'), `${edition}: pre-profile API request ${apiPaths(requests)}`);
  }
  await navigation;
  await page.waitForFunction(() => document.body.dataset.startupState === 'ready');
  const state = await shellState(page);
  check(!state.shellHidden && !state.inert && state.ariaHidden === null, `${edition}: shell not accessible after pruning`);
  check(state.statusHidden, `${edition}: loading status remained visible`);
  const paths = apiPaths(requests);
  check(paths[0] === '/api/edition', `${edition}: profile was not first API request: ${paths}`);
  if (edition !== 'full') {
    check(!paths.some(pathname => pathname.includes('/market') || pathname.includes('southern') || pathname.includes('ancillary')),
      `${edition}: forbidden hash triggered market API: ${paths}`);
  }
  if (edition === 'full') {
    await page.waitForFunction(() => performance.getEntriesByType('resource').some(entry => entry.name.includes('/api/dynamics/model_schema')));
    check(apiPaths(requests).some(pathname => pathname === '/api/dynamics/model_schema'), 'full: dynamics capability was not positively used');
    check(await page.locator('#moduleTransient').count() === 1, 'full: transient module was pruned');
  } else {
    await page.waitForTimeout(150);
    check(!apiPaths(requests).some(pathname => pathname.includes('/api/dynamics/')), `${edition}: dynamics requested`);
    check(await page.locator('#moduleTransient').count() === 0, `${edition}: transient module remains`);
    check(await page.locator('[data-module^="market"], [data-module="rpo"], [data-module="harmonics"]').count() === 0,
      `${edition}: restricted module remains`);
    check(await page.locator('.module-btn.active').getAttribute('data-module') === 'modelIO', `${edition}: unsafe default/hash module`);
    check(page.url().endsWith('/xjtu/'), `${edition}: forbidden hash was restored`);
  }
  if (edition === 'resilience') {
    check(await page.locator('[data-module="carbonFlow"], [data-sub="carbonFlow"], [data-sub="carbonAnalysis"], [data-result-group="carbonFlow"], [data-result-group="carbonAnalysis"]').count() === 0,
      'resilience: carbon-owned DOM remains');
    check(await page.locator('.workflow-btn[data-workflow="security"]').count() === 0,
      'resilience: security workflow remains');
    check(await page.locator('[data-module="topology"], [data-module="shortCircuit"], [data-module="reliability"]').count() === 0,
      'resilience: removed topology/short-circuit/reliability module remains');
    check(await page.locator('.workflow-btn[data-workflow="planning"]').textContent() === '场景生成',
      'resilience: planning workflow not renamed to 场景生成');
    check(await page.locator('.module-btn[data-module="scenarioGeneration"]').textContent() === '台风致灾场景生成',
      'resilience: scenario module not renamed to 台风致灾场景生成');
    check(await page.locator('.module-btn[data-group="planning"]').count() === 2,
      'resilience: planning group is not reduced to scenario generation + placeholder');
    check(await page.locator('#moduleExtremeScenarios').isDisabled(),
      'resilience: extreme-scenario placeholder is not disabled');
    check(await page.locator('.workflow-btn[data-workflow="sustainability"]').textContent() === '弹性分析',
      'resilience: sustainability workflow not renamed to 弹性分析');
    check(await page.locator('.module-btn[data-module="resilience"]').textContent() === '完整弹性分析',
      'resilience: resilience module not renamed to 完整弹性分析');
    check(await page.locator('.module-btn[data-group="sustainability"]').count() === 5,
      'resilience: sustainability group is not 完整弹性分析/主动防御/快速恢复/弹性指标 + placeholder');
    check(await page.locator('#moduleResilienceWeakLinks').isDisabled(),
      'resilience: 薄弱环节 placeholder is not disabled');
    for (const view of ['proactiveDefense', 'rapidRecovery', 'resilienceMetrics']) {
      check(await page.locator(`.module-btn[data-module="${view}"]`).count() === 1 &&
        await page.locator(`.sub-section[data-sub="${view}"]`).count() === 1 &&
        await page.locator(`.result-group[data-result-group="${view}"]`).count() === 1,
        `resilience: ${view} view module surfaces missing`);
    }
    const placeholderActivated = await page.evaluate(() => {
      const before = document.querySelector('.module-btn.active')?.dataset.module;
      document.getElementById('moduleResilienceWeakLinks').click();
      document.getElementById('moduleExtremeScenarios').click();
      return document.querySelector('.module-btn.active')?.dataset.module !== before;
    });
    check(!placeholderActivated, 'resilience: disabled placeholder module became active');
    await page.locator('.workflow-btn[data-workflow="sustainability"]').click();
    for (const view of ['resilience', 'proactiveDefense', 'rapidRecovery', 'resilienceMetrics']) {
      await page.evaluate(m => document.querySelector(`.module-btn[data-module="${m}"]`).click(), view);
      const display = await page.evaluate(m => {
        const group = document.querySelector(`.result-group[data-result-group="${m}"]`);
        return group ? getComputedStyle(group).display : 'missing';
      }, view);
      check(display === 'block', `resilience: ${view} result group not displayed when active`);
    }
    check(await page.locator('#scenRegularClusters, #scenReliabilityClusters, #btnExportRegularScenarioJson, #btnExportReliabilityScenarioJson, #btnExportScenarioResults, #btnScenarioJsonToExcel, #btnScenarioExcelToJson').count() === 0,
      'resilience: non-resilience scenario controls remain');
    check(await page.locator('#scenResilienceClusters, #btnExportResilienceScenarioJson, #btnExportScenarioPlots').count() === 3,
      'resilience: required resilience scenario controls missing');
    check(await page.locator('#editionBadge').textContent() === 'RESILIENCE', 'resilience: badge missing');
    check(await page.locator('#btnIoImportGridlabd, #btnIoImportOpendss, #btnIoExportGridlabd, #btnIoExportOpendss').count() === 0,
      'resilience: unsupported IO remains');
  }
  if (edition === 'trial') {
    check(await page.locator('[data-sub="carbonAnalysis"], [data-result-group="carbonAnalysis"]').count() === 2,
      'trial: carbonFlow-owned shared surfaces were removed');
    check(await page.locator('[data-trial-indicator]').count() === 8, 'trial: indicator design missing');
  }
  await assertNoOverflow(page, `${edition}/${viewport.width}`);
}

async function runDynamicRoundTrip(page) {
  const dynamic = { model: 'GENROU', nested: { array: [1, 2, { untouched: true }] }, vendor_key: 'preserve-me' };
  const output = await page.evaluate(value => {
    Canvas.loadFromSystemJson({
      name: 'dynamic-roundtrip', base_mva: 100,
      ac: {
        buses: [{ index: 1, name: 'Bus 1', bus_type: 'SLACK', base_kv: 110 }],
        generators: [{ index: 0, bus: 1, name: 'G1', pg_mw: 1, dynamic_model: value }],
        loads: [], branches: [],
      }, dc: { buses: [], branches: [], loads: [] },
    }, { forceRender: true });
    const generator = Canvas.state.components.find(component => component.type === 'generator');
    Canvas.state.selectedId = generator.id;
    App.onSelectionChanged(generator.id);
    document.getElementById('btnApplyProp').click();
    return Canvas.buildSystemJson().ac.generators[0].dynamic_model;
  }, dynamic);
  check(JSON.stringify(output) === JSON.stringify(dynamic), `dynamic_model round trip changed: ${JSON.stringify(output)}`);
}

async function main() {
  const { chromium } = await import('playwright');
  const { server, base } = await startServer();
  let browser;
  try {
    browser = await chromium.launch({
      executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH || undefined,
    });
    const viewports = [{ width: 1440, height: 1000 }, { width: 390, height: 844 }];
    for (const viewport of viewports) {
      for (const edition of ['resilience', 'trial', 'full']) {
        const page = await browser.newPage({ viewport });
        try {
          await runValid(page, base, edition, viewport, {
            delayed: edition === 'resilience',
            hash: edition === 'full' ? '' : '#market-realtime',
          });
          if (edition === 'resilience') await runDynamicRoundTrip(page);
        } finally { await page.close(); }
      }
      const badProfiles = [
        { name: 'wrong-schema', mode: 'valid', profile: { ...profile('resilience'), schema: 'wrong.schema' } },
        { name: 'unknown-edition', mode: 'valid', profile: { ...profile('resilience'), edition: 'enterprise' } },
        { name: 'unknown-module', mode: 'valid', profile: { ...profile('resilience'), frontend_modules: [...FRONTEND.resilience, 'secretModule'] } },
        { name: 'missing-module', mode: 'valid', profile: { ...profile('resilience'), frontend_modules: FRONTEND.resilience.slice(0, -1) } },
        { name: 'duplicate-module', mode: 'valid', profile: { ...profile('resilience'), frontend_modules: [...FRONTEND.resilience, 'modelIO'] } },
        { name: 'empty-workflow', mode: 'valid', profile: { ...profile('resilience'), workflow: [] } },
        { name: 'restricted-wildcard', mode: 'valid', profile: { ...profile('resilience'), frontend_modules: ['*'] } },
        { name: 'http-failure', mode: 'http' },
        { name: 'malformed-json', mode: 'json' },
        { name: 'hung-json-body', mode: 'hung' },
      ];
      for (const scenario of badProfiles) {
        const page = await browser.newPage({ viewport });
        try { await runFailure(page, base, scenario, viewport); }
        finally { await page.close(); }
      }
    }
    console.log('Resilience edition GUI mocked desktop/mobile matrix passed');
  } finally {
    if (browser) await browser.close();
    await new Promise(resolve => server.close(resolve));
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
