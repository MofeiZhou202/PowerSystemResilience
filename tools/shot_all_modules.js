// 全模块 sub-toolbar 截图回归：node tools/shot_all_modules.js <port> <outdir>
const { chromium } = require('playwright');
const port = process.argv[2] || '18110';
const outdir = process.argv[3] || '/tmp/gui_shots';

const MODULES = [
  ['modeling', 'moduleModelIO', 'modelIO'],
  ['modeling', 'moduleParameterLibrary', 'parameterLibrary'],
  ['modeling', 'moduleTopologyAnalysis', 'topologyAnalysis'],
  ['steady', 'modulePowerFlow', 'powerFlow'],
  ['steady', 'moduleOpf', 'opf'],
  ['steady', 'moduleRpo', 'rpo'],
  ['steady', 'moduleHarmonics', 'harmonics'],
  ['steady', 'moduleHosting', 'hosting'],
  ['security', 'moduleShortCircuit', 'shortCircuit'],
  ['security', 'moduleTransient', 'transient'],
  ['security', 'moduleTspf', 'tspf'],
  ['planning', 'moduleTopology', 'topology'],
  ['planning', 'moduleTimeSeries', 'timeSeries'],
  ['planning', 'moduleScenarioGeneration', 'scenarioGeneration'],
  ['planning', 'moduleWeakLinks', 'weakLinks'],
  ['market', 'moduleMarketBehavior', 'marketBehavior'],
  ['market', 'moduleMarketBoundary', 'marketBoundary'],
  ['market', 'moduleMarket', 'market'],
  ['market', 'moduleMarketSecurity', 'marketSecurity'],
  ['market', 'moduleMarketSettlement', 'marketSettlement'],
  ['ies', 'moduleIntegratedEnergy', 'integratedEnergy'],
  ['ies', 'moduleEvTraffic', 'evTraffic'],
  ['sustainability', 'moduleReliability', 'reliability'],
  ['sustainability', 'moduleResilience', 'resilience'],
  ['sustainability', 'moduleCarbonFlow', 'carbonFlow'],
];

(async () => {
  const fs = require('fs');
  fs.mkdirSync(outdir, { recursive: true });
  const browser = await chromium.launch({ args: ['--no-proxy-server'] });
  const errors = [];
  for (const width of [1360, 1000]) {
    const page = await browser.newPage({ viewport: { width, height: 1000 } });
    // 离线环境：截断一切非本机请求（CDN plotly 等会挂起 DOMContentLoaded）
    await page.route(/^https?:\/\//, route => {
      const u = route.request().url();
      if (u.startsWith(`http://127.0.0.1:${port}`) || u.startsWith(`http://localhost:${port}`)) return route.continue();
      return route.abort();
    });
    page.on('pageerror', e => errors.push(`[w${width}] PAGEERROR: ${e.message}`));
    await page.goto(`http://127.0.0.1:${port}/index.html`, { waitUntil: 'domcontentloaded' });
    await page.waitForTimeout(2500);
    let currentWorkflow = null;
    for (const [workflow, btnId, name] of MODULES) {
      try {
        if (currentWorkflow !== workflow) {
          await page.click(`[data-workflow="${workflow}"]`, { force: true });
          await page.waitForTimeout(250);
          currentWorkflow = workflow;
        }
        await page.click(`#${btnId}`, { force: true });
        await page.waitForTimeout(350);
        const visible = await page.evaluate(() => {
          const ss = [...document.querySelectorAll('.sub-section')].find(s => !s.hidden);
          return ss ? ss.dataset.sub : 'NONE';
        });
        const bar = await page.$('#subToolbar');
        if (bar) await bar.screenshot({ path: `${outdir}/${name}_w${width}.png` });
        console.log(`w${width} ${name}: visible=${visible}`);
      } catch (e) {
        errors.push(`[w${width}] ${name}: ${e.message.split('\n')[0]}`);
      }
    }
    await page.close();
  }
  await browser.close();
  console.log(errors.length ? 'ERRORS:\n' + errors.join('\n') : 'JS errors: none');
})();
