// Read-only profile of an existing session. No market jobs are submitted.
import { chromium } from 'playwright';
import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';

const arg = process.argv.indexOf('--base');
if (arg < 0) throw new Error('Usage: node tools/market_validation/profile_market_gui.mjs --base http://127.0.0.1:8101');
const base = process.argv[arg + 1], output = path.resolve('output/market-activity');
const browser = await chromium.launch();
try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  await page.addInitScript(() => {
    localStorage.setItem('hysim.tourDone.v1', '1');
    localStorage.setItem('hysim.marketOperationMode', 'forecast');
    sessionStorage.setItem('hysim.marketOperationStep', '3');
    window.marketLongTasks = [];
    new PerformanceObserver(list => window.marketLongTasks.push(...list.getEntries().map(e => e.duration)))
      .observe({ type: 'longtask', buffered: true });
  });
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  await page.goto(`${base}/xjtu/#market-operation`);
  await page.waitForFunction(() => document.getElementById('forecastSolver')?.options.length > 1);
  await page.waitForTimeout(15000);
  const profile = await page.evaluate(() => {
    const requests = performance.getEntriesByType('resource')
      .filter(r => r.name.endsWith('/api/session/market_forecast'))
      .map(r => ({ bytes: r.decodedBodySize, duration_ms: r.duration }));
    return { requests, total_bytes: requests.reduce((s, r) => s + r.bytes, 0),
      long_tasks_ms: window.marketLongTasks,
      hidden_overview_svg_count: document.querySelectorAll('#operationPlanOverview .main-svg').length,
      status: document.getElementById('forecastStatus').textContent };
  });
  await mkdir(output, { recursive: true });
  const report = { base, measured_at: new Date().toISOString(), ...profile, errors };
  await writeFile(path.join(output, 'profile.json'), JSON.stringify(report, null, 2));
  console.log(JSON.stringify(report, null, 2));
} finally { await browser.close(); }
