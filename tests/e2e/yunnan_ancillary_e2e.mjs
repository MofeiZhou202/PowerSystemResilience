import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const index = process.argv.indexOf('--server');
const executable = index < 0 ? path.join(root, 'build/macos-release/tests/run_gui_server') : process.argv[index + 1];
const port = await new Promise(resolve => { const s = createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); }); });
const base = `http://127.0.0.1:${port}`;
const server = spawn(executable, ['--host', '127.0.0.1', '--port', String(port)], { cwd: root, stdio: 'ignore' });
let browser;
const output = path.join(root, 'output/market-ancillary');
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-6, `${a} != ${b}`);
async function api(route, body, expected = 200) {
  const response = await fetch(base + '/api/session/' + route, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
  const data = await response.json(); assert.equal(response.status, expected, JSON.stringify(data).slice(0, 500)); return data;
}
try {
  for (let i = 0; i < 150; ++i) { try { if ((await fetch(base + '/api/cases')).ok) break; } catch {} await new Promise(r => setTimeout(r, 100)); }
  let state = await api('southern_market');
  state = await api('southern_market', { action: 'ieee118_mixed', revision: state.revision });
  state.boundary.execution.ac_security = 'schedule_only';
  state.boundary.execution.solver = 'highs'; state.boundary.execution.time_limit_sec = 30;
  state = await api('southern_market', { action: 'save', revision: state.revision, boundary: state.boundary });
  const before = await api('yunnan_ancillary');
  assert.equal(before.config.cmin_mw, 450); assert.equal(before.config.agc_units.length, 30);
  assert.equal(before.config.agc_units.filter(a => a.mode === 'plant').length, 12);
  assert.equal(before.boundary.storage.length, 6); assert.equal(before.boundary.controllable_loads.length, 6);
  browser = await chromium.launch(); const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  await page.addInitScript(() => localStorage.setItem('hysim.tourDone.v1', '1'));
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  await page.goto(base + '/xjtu/#market-ancillary', { waitUntil: 'networkidle' });
  await page.locator('#ancillaryUnit option').first().waitFor({ state: 'attached' });
  assert.equal(await page.locator('#moduleMarketAncillary').getAttribute('class'), 'module-btn active');
  const fill = async (label, value) => { const field = page.getByLabel(label, { exact: true }); await field.fill(String(value)); await field.blur(); };
  await fill('二次调频最低需求 Cmin / MW', 20); await fill('负荷比例 R1 / p.u.', 0); await fill('新能源比例 R2 / p.u.', 0);
  await page.locator('#ancillarySave').click();
  await page.waitForFunction(() => document.querySelector('#ancillaryStatus').textContent === '调频配置已保存');
  await page.locator('#ancillaryReload').click(); await page.waitForFunction(() => document.querySelector('#ancillaryStatus').textContent.startsWith('已载入'));
  assert.equal(await page.getByLabel('二次调频最低需求 Cmin / MW', { exact: true }).inputValue(), '20');
  const saved = await api('yunnan_ancillary'); const invalid = structuredClone(saved.config);
  invalid.agc_units[0].mode = 'single'; await api('yunnan_ancillary', { action: 'save', revision: saved.revision, config: invalid }, 400);
  assert.equal((await api('yunnan_ancillary')).revision, saved.revision);
  await api('yunnan_ancillary', { action: 'run', revision: saved.revision - 1, config: saved.config }, 409);
  await page.locator('#ancillaryRun').click();
  await page.waitForFunction(
    () => document.querySelector('#ancillaryStatus').textContent.includes('调频预安排完成'),
    undefined,
    { timeout: 120000 },
  );
  const result = (await api('yunnan_ancillary')).result;
  assert.equal(result.schedule_feasible, true); assert.equal(result.prices_valid, true);
  assert.equal(result.ancillary.settlement_cny, null);
  const gen = new Map(result.sced.generators.map(g => [g.id, g]));
  for (const a of saved.config.agc_units) for (let t = 0; t < 98; ++t) {
    const award = t < 96 ? result.ancillary.hours[Math.floor(t / 4)].bids.find(b => b.id === a.id).award_mw : 0;
    near(a.members.reduce((s, m) => s + gen.get(m.generator_id).secondary_up_mw[t], 0), award);
    near(a.members.reduce((s, m) => s + gen.get(m.generator_id).secondary_down_mw[t], 0), award);
    for (const member of a.members) {
      const g = gen.get(member.generator_id), old = result.scuc.generators.find(g => g.id === member.generator_id);
      near(g.online[t], old.online[t]); near(g.primary_reserve_mw[t], old.primary_reserve_mw[t]);
      if (member.safe_intervals_mw.length && g.stable[t] > 0.5) assert.ok(member.safe_intervals_mw.some(([lo, hi]) => g.power_mw[t] - g.secondary_down_mw[t] >= lo - 1e-6 && g.power_mw[t] + g.secondary_up_mw[t] + g.primary_reserve_mw[t] <= hi + 1e-6));
    }
  }
  await page.waitForFunction(() => document.querySelector('#ancillaryEnvelopeChart').data?.length === 3);
  const traces = await page.locator('#ancillaryCapacityChart').evaluate(e => e.data.map(t => t.y));
  assert.deepEqual(traces[0], Array(24).fill(20)); assert.deepEqual(traces[1], Array(24).fill(20));
  await page.getByLabel('Canvas 时段', { exact: true }).selectOption('15');
  assert.equal(await page.locator('#ancillarySlot').inputValue(), '15');
  assert.match(await page.locator('#marketCanvasDetails').textContent(), /二次上调 MW/);
  await page.locator('#ancillaryEnvelopeChart').evaluate(e => e.emit('plotly_click', { points: [{ x: 5 }] }));
  assert.equal(await page.locator('#marketCanvas').getAttribute('data-slot'), '20');
  assert.equal(saved.config.independent_units.length, 12);
  assert.equal(await page.locator('#ancillaryUnit option').count(), 42);
  const workflow = structuredClone(saved.config.workflow); workflow.stage = 'intraday';
  const removed = result.ancillary.hours[0].bids.find(b => b.award_mw > 0).id;
  workflow.safety_reviews = Array.from({ length: 24 }, (_, hour) => ({ unit_id: removed, hour, up_mw: 0, down_mw: 0, agc_available: false, reason: 'Synthetic AGC outage', source: 'IEEE118 rule-workflow experiment' }));
  await page.locator('#ancillaryWorkflow').locator('xpath=ancestor::details').evaluate(e => { e.open = true; });
  await page.locator('#ancillaryWorkflow').fill(JSON.stringify(workflow));
  await page.locator('#ancillaryIntraday').click();
  await page.waitForFunction(
    () => document.querySelector('#ancillaryStatus').textContent.includes('日内调频出清完成'),
    undefined,
    { timeout: 120000 },
  );
  const current = await api('yunnan_ancillary'), intraday = current.result;
  assert.equal(intraday.scuc.reused_day_ahead, true); assert.equal(intraday.ancillary.settlement_eligible, true);
  assert.equal(await page.locator('#ancillaryRun').isEnabled(), false);
  assert.deepEqual(intraday.commitment_solution, result.commitment_solution);
  for (const h of intraday.ancillary.hours) { near(h.shortage_mw, 0); near(h.bids.find(b => b.id === removed).award_mw, 0); }
  for (const g of intraday.sced.generators) {
    const old = result.scuc.generators.find(x => x.id === g.id);
    for (let t = 0; t < 98; ++t) { near(g.online[t], old.online[t]); near(g.primary_reserve_mw[t], old.primary_reserve_mw[t]); }
  }
  const changedBid = structuredClone(current.config); changedBid.agc_units[0].price_per_mw[0] = 7;
  await api('yunnan_ancillary', { action: 'intraday', revision: current.revision, day_ahead_id: current.day_ahead_id, config: changedBid }, 400);
  const measurements = intraday.ancillary.hours.flatMap(h => h.bids.filter(b => b.award_mw > 0).map(b => ({ unit_id: b.id, hour: h.hour, source: 'Synthetic explicit zero-command history', test_period: false, own_unavailability_seconds: [], events: [] })));
  for (let i = 0; i < 2; ++i) measurements[i].events = [{ event_id: `E2E-command-${i}`, start_second: 0, end_second: 60, start_mw: 50, end_mw: 60 + 10 * i, autor: true, metrics: { rate_fraction_per_min: 0.015, delay_seconds: 0, error_fraction: 0, fleet_standard_fraction_per_min: 0.015, standard_delay_seconds: 60, allowed_error_fraction: 0.01 } }];
  const allocation = { continuous_spot: true, generation_share: 0.5, assessment_pool_cny: 0, participants: [
    { id: 1, source: 'Synthetic export meters', point_to_grid: false, unit_ids: intraday.ancillary.hours[0].bids.map(b => b.id), export_mwh: 100, nonmarket_export_mwh: 100, import_mwh: 0 },
    { id: 2, source: 'Synthetic import meter', point_to_grid: false, unit_ids: [], export_mwh: 0, nonmarket_export_mwh: 0, import_mwh: 100 }
  ] };
  const request = { measurements, allocation };
  await api('yunnan_ancillary', { action: 'settle', revision: current.revision, clearing_id: intraday.clearing_id - 1, request }, 409);
  const duplicate = structuredClone(request); duplicate.measurements[1].events[0].event_id = 'E2E-command-0';
  await api('yunnan_ancillary', { action: 'settle', revision: current.revision, clearing_id: intraday.clearing_id, request: duplicate }, 400);
  assert.equal((await api('yunnan_ancillary')).result.statement, undefined);
  const missing = structuredClone(request); missing.measurements.pop();
  const incomplete = await api('yunnan_ancillary', { action: 'settle', revision: current.revision, clearing_id: intraday.clearing_id, request: missing });
  assert.equal(incomplete.result.statement.complete, false); assert.equal(incomplete.result.statement.allocation, null);
  await page.locator('#ancillaryMeasurements').locator('xpath=ancestor::details').evaluate(e => { e.open = true; });
  await page.locator('#ancillaryMeasurements').fill(JSON.stringify(request)); await page.locator('#ancillarySettle').click();
  await page.waitForFunction(() => document.querySelector('#ancillaryStatus').textContent.includes('日累计里程与费用核算完成'));
  const statement = (await api('yunnan_ancillary')).result.statement;
  near(statement.metering.total_compensation_cny, 30 * intraday.ancillary.hours[0].clearing_price_per_mw);
  near(statement.allocation.compensation_balance_residual_cny, 0);
  assert.equal(await page.locator('#ancillaryMileageChart').isVisible(), true);
  await page.waitForFunction(() => document.querySelector('#ancillaryMileageChart').data?.[0]?.y.length === 24);
  near((await page.locator('#ancillaryMileageChart').evaluate(e => e.data[0].y))[0], 30);
  const posting = { book: 'E2E-IEEE118', delivery_date: '2026-09-01', posting_date: '2026-09-02', discovered_date: null, supersedes_id: null, reason: 'Original synthetic experiment', source: 'E2E fixture' };
  await page.locator('#ancillaryPosting').locator('xpath=ancestor::details').evaluate(e => { e.open = true; });
  await page.locator('#ancillaryPosting').fill(JSON.stringify(posting)); await page.locator('#ancillaryPost').click();
  await page.waitForFunction(() => document.querySelector('#ancillaryStatus').textContent.includes('日凭证已追加'));
  await api('yunnan_ancillary', { action: 'post', revision: current.revision, clearing_id: intraday.clearing_id, request: posting }, 400);
  await page.locator('#ancillaryMonthlyInput').fill(JSON.stringify({ book: posting.book, month: '2026-09', allocation })); await page.locator('#ancillaryMonth').click();
  await page.waitForFunction(() => document.querySelector('#ancillaryMonthStatus').textContent.includes('缺少 29 天'));
  assert.equal((await api('yunnan_ancillary')).month.allocation, null);
  const downloaded = page.waitForEvent('download'); await page.locator('#ancillaryJournalExport').click();
  assert.equal((await downloaded).suggestedFilename(), 'yunnan-ancillary-journal.json');
  await mkdir(output, { recursive: true });
  await writeFile(path.join(output, 'ieee118-highs.json'), JSON.stringify(result));
  await writeFile(path.join(output, 'ieee118-intraday-highs.json'), JSON.stringify(intraday));
  await writeFile(path.join(output, 'ieee118-rule-statement.json'), JSON.stringify(statement));
  await page.locator('#ancillaryMileageChart').scrollIntoViewIfNeeded(); await page.screenshot({ path: path.join(output, 'statement-desktop.png') });
  await page.locator('#ancillaryCapacityChart').scrollIntoViewIfNeeded(); await page.screenshot({ path: path.join(output, 'desktop.png') });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.locator('#ancillaryCapacityChart').scrollIntoViewIfNeeded(); await page.waitForTimeout(300);
  const geometry = await page.evaluate(() => ({ width: document.documentElement.clientWidth, scroll: document.documentElement.scrollWidth }));
  assert.ok(geometry.scroll <= geometry.width + 1, JSON.stringify(geometry));
  const chartOverlap = await page.locator('#ancillaryCapacityChart').evaluate(e => {
    const x = e.querySelector('.xtitle').getBoundingClientRect(), l = e.querySelector('.legend').getBoundingClientRect();
    return x.left < l.right && l.left < x.right && x.top < l.bottom && l.top < x.bottom;
  });
  assert.equal(chartOverlap, false, 'legend overlaps hourly axis label');
  await page.screenshot({ path: path.join(output, 'mobile.png') });
  const textareaGeometry = await page.locator('#ancillaryWorkflow').evaluate(e => ({ width: e.getBoundingClientRect().width, parent: e.parentElement.getBoundingClientRect().width, color: getComputedStyle(e).color, background: getComputedStyle(e).backgroundColor }));
  assert.ok(textareaGeometry.width <= textareaGeometry.parent + 1, JSON.stringify(textareaGeometry));
  assert.notEqual(textareaGeometry.color, textareaGeometry.background);
  await page.locator('#ancillaryMileageChart').scrollIntoViewIfNeeded(); await page.screenshot({ path: path.join(output, 'statement-mobile.png') });
  const mileageAxisOverlap = await page.locator('#ancillaryMileageChart').evaluate(e => {
    const title = e.querySelector('.xtitle').getBoundingClientRect();
    return [...e.querySelectorAll('.xtick text')].some(t => { const r = t.getBoundingClientRect(); return r.left < title.right && title.left < r.right && r.top < title.bottom && title.top < r.bottom; });
  });
  assert.equal(mileageAxisOverlap, false, 'mileage hour labels overlap axis title');
  await page.locator('#ancillaryUnit').selectOption(String(saved.config.independent_units[0].id));
  await fill('持续响应能力 h', 1); await fill('AGC速率 MW/min', 5);
  assert.equal(await page.locator('#ancillarySettle').isEnabled(), false);
  assert.equal(await page.locator('#ancillaryResults').isVisible(), false);
  // Editing the selected machine's band through the canonical declaration invalidates the result.
  await page.locator('#ancillaryJson').locator('xpath=ancestor::details').evaluate(e => { e.open = true; });
  const edited = structuredClone(saved.config); edited.agc_units[0].members[0].safe_intervals_mw[0][0] += 1;
  await page.locator('#ancillaryJson').fill(JSON.stringify(edited)); await page.locator('#ancillaryApplyJson').click();
  await page.waitForFunction(() => document.querySelector('#ancillaryStatus').textContent.includes('完整调频配置已校验并保存'));
  const changed = await api('yunnan_ancillary'); assert.equal(changed.result, null); assert.deepEqual(changed.config, edited);
  assert.equal(await page.locator('#ancillaryResults').isVisible(), false);
  await page.reload({ waitUntil: 'networkidle' }); await page.locator('#ancillaryUnit option').first().waitFor({ state: 'attached' });
  assert.deepEqual(JSON.parse(await page.locator('#ancillaryJson').inputValue()), edited);
  // Energy boundary replacement invalidates the AGC declarations and result.
  await api('southern_market', { action: 'example', revision: changed.revision });
  const reset = await api('yunnan_ancillary'); assert.equal(reset.result, null); assert.equal(reset.config.agc_units.length, 1);
  assert.deepEqual(errors, []);
  console.log(JSON.stringify({ passed: true, solver: result.sced.solver, runtime_sec: result.runtime_sec, hydro_units: 36, agc_plants: 12, geometry }));
} finally { if (browser) await browser.close(); server.kill('SIGTERM'); }
