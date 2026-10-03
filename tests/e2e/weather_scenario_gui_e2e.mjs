// Real browser + backend: schema, generation, selection, recovery and metrics.
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const executable = process.argv[process.argv.indexOf('--server') + 1];
if (!process.argv.includes('--server')) throw new Error('--server is required');
let chromium;
try { ({ chromium } = await import('playwright')); }
catch { console.log('SKIP: playwright unavailable'); process.exit(77); }
const port = await new Promise(resolve => {
  const probe = createServer(); probe.listen(0, '127.0.0.1', () => {
    const value = probe.address().port; probe.close(() => resolve(value));
  });
});
const child = spawn(path.resolve(executable), ['--host', '127.0.0.1', '--port', String(port), '--data-dir', 'data'], {
  cwd: root, windowsHide: true, stdio: 'ignore',
});
const base = `http://127.0.0.1:${port}`;
let browser;
const evidence = [];
try {
  let ready = false;
  for (let i = 0; i < 120; i++) {
    try { ready = (await fetch(`${base}/api/edition`)).ok; } catch {}
    if (ready) break;
    if (child.exitCode !== null) throw new Error('Server exited before startup');
    await new Promise(resolve => setTimeout(resolve, 250));
  }
  assert(ready, 'Server did not start');
  browser = await chromium.launch({ executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH || undefined });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const errors = [];
  const metricRequests = [];
  page.on('pageerror', error => { errors.push(error.message); console.error(error.message); });
  page.on('request', request => {
    if (request.method() === 'POST' && request.url().endsWith('/api/session/resilience/metrics')) {
      metricRequests.push(request.postDataJSON());
    }
  });
  await page.route('**/plotly*.js', async route => {
    try { await route.fulfill({ contentType: 'application/javascript', body: await readFile(path.join(root, 'build/plotly-2.27.0.min.js')) }); }
    catch { await route.abort(); }
  });
  await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => ['ready', 'unavailable'].includes(document.body.dataset.startupState));
  assert.equal(await page.evaluate(() => document.body.dataset.startupState), 'ready', await page.locator('#startupStatusDetail').textContent());
  await page.getByRole('button', { name: '体验 33 节点演示', exact: true }).click();
  await page.waitForFunction(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    return !p.workspace.busy && p.state.model.status === 'ready';
  });
  assert.equal(await page.evaluate(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.model.currentName),
    'dist33_weather_mixed', 'Guided demo did not load the mixed weather case');
  const inventory = await page.evaluate(async () => {
    const response = await fetch('/api/session/export_json', { method: 'POST' });
    return JSON.parse((await response.json()).json_string);
  });
  assert(inventory.ac?.transformers_2w?.some(tr => tr.weather_moisture_vulnerable),
    'Mixed weather case lost its vulnerable transformer during browser load');
  const go = async step => {
    await page.getByRole('tab', { name: '工作流', exact: true }).click();
    await page.locator(`[data-resilience-step="${step}"]`).click();
  };
  await go('metric_selection');
  assert.equal(await page.getByRole('heading', { name: '指标计算口径', exact: true }).count(), 0);
  assert.equal(await page.locator('#resiliencePortalPanel input[type="number"]').count(), 0);
  await mkdir(path.join(root, 'build/weather-gui'), { recursive: true });
  for (const hazard of ['rainstorm', 'lightning']) {
    await go('scenario_selection');
    await page.locator('[data-portal-field="scenario-hazard-type"]').selectOption(hazard);
    assert.equal(await page.locator('[data-portal-intensity]').count(), 0, 'Typhoon-only fields leaked');
    const key = hazard === 'rainstorm' ? 'total_mm' : 'density_km2_hr';
    const input = page.locator(`[data-portal-hazard-parameter="${key}"]`);
    await input.fill(hazard === 'rainstorm' ? '260' : '5'); await input.press('Tab');
    await page.locator('[data-portal-field="scenario-hazard-type"]').scrollIntoViewIfNeeded();
    await page.screenshot({ path: path.join(root, `build/weather-gui/${hazard}-parameters.png`) });
    // Switching preserves authored sparse overrides; resetting only this hazard restores defaults.
    await page.locator('[data-portal-field="scenario-hazard-type"]').selectOption('typhoon');
    await page.locator('[data-portal-field="scenario-hazard-type"]').selectOption(hazard);
    assert.equal(await input.inputValue(), hazard === 'rainstorm' ? '260' : '5');
    await page.getByRole('button', { name: '恢复本灾种默认参数' }).click();
    const expectedDefault = await page.evaluate(({ hazard, key }) => {
      const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
      return p.adapter.scenario.getHazardSchema().find(x => x.id === hazard).fields.find(x => x.key === key).default;
    }, { hazard, key });
    assert.equal(Number(await input.inputValue()), expectedDefault);
    await input.fill(hazard === 'rainstorm' ? '260' : '5'); await input.press('Tab');
    await page.getByRole('button', { name: '生成弹性候选场景', exact: true }).click();
    await page.waitForFunction(hazard => {
      const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
      const s = p.state.scenarioGeneration;
      return ['error', 'empty'].includes(s.status) || (s.status === 'ready' && s.candidates[0]?.raw?.representative?.resilience_event?.hazard_type === hazard);
    }, hazard, { timeout: 90000 });
    const generated = await page.evaluate(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.scenarioGeneration);
    const liveInventory = await page.evaluate(async () => {
      const response = await fetch('/api/session/export_json', { method: 'POST' });
      const raw = JSON.parse((await response.json()).json_string);
      return { name: raw.name, transformers: raw.ac?.transformers_2w?.length,
        vulnerable: raw.ac?.transformers_2w?.filter(tr => tr.weather_moisture_vulnerable).length };
    });
    assert.equal(generated.status, 'ready', generated.error);
    const event = generated.candidates[0].raw.representative.resilience_event;
    assert.equal(event.hazard_type, hazard); assert(event.faults.length > 0,
      `Expected nonzero damage: ${hazard}, inventory=${JSON.stringify(liveInventory)}, equipment=${JSON.stringify(event.hazard_evidence?.affected_equipment)}`);
    assert.equal(event.hazard_evidence.parameters[key], hazard === 'rainstorm' ? 260 : 5);
    assert(await page.locator(`[data-portal-hazard-evidence="${hazard}"]`).isVisible());
    const equipment = event.hazard_evidence.affected_equipment;
    assert.equal(equipment.fault_count, event.faults.length);
    assert.equal(equipment.failed_ac_branches + equipment.failed_dc_branches, event.faults.length);
    assert.equal(liveInventory.vulnerable, 1, 'Canvas synchronization lost transformer exposure');
    assert.equal(equipment.assumed_line_types, 0, 'Mixed case lost line classifications');
    if (hazard === 'rainstorm') {
      assert.equal(equipment.exposed_transformers, 1);
      assert(equipment.cable_accessory_shutdowns > 0);
      assert(equipment.insulator_flashover_trips > 0);
      assert.equal(equipment.permanent_failure_branches, 0, 'Preventive outages are not permanent damage');
    } else {
      assert(event.faults.every(f => f.equipment_type === 'overhead_line' && f.branch_type === 'AC'));
    }
    assert(await page.locator(`[data-portal-hazard-evidence="${hazard}"]`).getByText(
      hazard === 'rainstorm' ? '暴雨只计算明确标注易受淹入口的电缆附件' : '雷击只抽样架空线路',
      { exact: false }).isVisible());
    assert.equal(await page.locator('[data-portal-calculation-summary="scenario-generation"]').getAttribute('open'), null);
    await page.locator(`[data-portal-hazard-evidence="${hazard}"]`).scrollIntoViewIfNeeded();
    await page.screenshot({ path: path.join(root, `build/weather-gui/${hazard}-desktop.png`), fullPage: true });
    await page.setViewportSize({ width: 390, height: 844 });
    await page.waitForTimeout(200); // Let the responsive portal render settle before scrolling.
    await page.locator(`[data-portal-hazard-evidence="${hazard}"]`).scrollIntoViewIfNeeded();
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth + 1), 'Mobile overflow');
    await page.screenshot({ path: path.join(root, `build/weather-gui/${hazard}-mobile.png`), fullPage: true });
    await page.setViewportSize({ width: 1440, height: 1000 });
    await go('proactive_defense');
    await page.locator('[data-portal-planning-run]').click();
    await page.waitForFunction(() => {
      const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
      return ['success', 'error'].includes(p.state.proactiveDefense.status);
    }, null, { timeout: 180000 });
    const planning = await page.evaluate(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.proactiveDefense);
    assert(planning.result, planning.error || 'Portfolio plan missing');
    assert.equal(planning.result.cluster_count, generated.candidates.length);
    const stalePlan = await page.evaluate(async () => {
      const response = await fetch('/api/session/run_distribution_resilience', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ portfolio_plan_id: 'rplan-stale', apply_demo_data: false }),
      });
      return { status: response.status, body: await response.json() };
    });
    assert.equal(stalePlan.status, 400);
    assert.match(stalePlan.body.error, /missing or stale/);
    await go('rapid_recovery');
    await page.locator('[data-portal-scenario-select]').first().click();
    assert.equal(await page.locator('[data-portal-field^="recovery-"]').count(), 1);
    assert(await page.locator('[data-portal-field="recovery-allow_mess_dispatch"]').isVisible());
    await page.getByRole('button', { name: '运行快速恢复', exact: true }).click();
    await page.waitForFunction(() => {
      const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
      return ['success', 'error'].includes(p.state.recovery.status);
    }, null, { timeout: 180000 });
    const recovery = await page.evaluate(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.recovery);
    assert(recovery.artifact, recovery.error || 'Recovery artifact missing');
    assert.equal(recovery.artifact.portfolio_plan_id, planning.result.plan_id);
    assert.equal(recovery.artifact.fault_sequence.length, event.faults.length);
    assert.equal(recovery.artifact.disaster_end_hr, Math.max(...event.faults.map(f => f.start_hr)));
    assert.equal(recovery.artifact.steps.at(-1).active_faults, 0);
    assert(Math.abs(recovery.artifact.final_restoration_ratio - 1) < 1e-8,
      `${hazard}: all faults cleared but final supply ratio is ${recovery.artifact.final_restoration_ratio}`);
    await go('metric_output');
    await page.getByRole('button', { name: '计算并输出指标', exact: true }).click();
    await page.waitForFunction(() => ['success', 'error'].includes(document.getElementById('resiliencePortalRoot').__resiliencePortal.state.metrics.status));
    const metrics = await page.evaluate(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.metrics);
    assert(metrics.result, metrics.error || 'Metric output missing');
    const metricRequest = metricRequests.at(-1);
    assert(metricRequest && !Object.hasOwn(metricRequest, 'parameters') &&
      !Object.hasOwn(metricRequest, 'allow_apda_system_gap_approximation') &&
      !Object.hasOwn(metricRequest, 'allow_res_approximation'),
    'Web metric request must use backend defaults');
    assert.equal(metrics.result.results.find(x => x.id === 'run.ens').status, 'computed');
    evidence.push({ hazard, generated, planning, recovery, metrics });
    // The workspace saves configuration and results; reload/restore must preserve hazard parameters.
    await page.waitForFunction(() => !document.getElementById('resiliencePortalRoot').__resiliencePortal.workspace.busy);
    const saved = await page.evaluate(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.workspace.versions);
    assert(saved.some(v => v.state?.scenarioConfig?.hazard_type === hazard), 'Weather version not saved');
  }
  await page.reload({ waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => document.body.dataset.startupState === 'ready');
  await page.getByRole('tab', { name: '我的分析', exact: true }).click();
  await page.getByRole('button', { name: '继续此版本', exact: true }).first().click();
  await page.waitForFunction(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    return !p.workspace.busy && p.state.scenarioConfig.hazard_type === 'lightning';
  });
  await go('scenario_selection');
  assert.equal(await page.locator('[data-portal-hazard-parameter="density_km2_hr"]').inputValue(), '5');
  assert.equal(await page.evaluate(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.recovery.runId ?? null), null);
  assert.deepEqual(errors, []);
  await writeFile(path.join(root, 'build/weather-gui/evidence.json'), JSON.stringify(evidence, null, 2));
  console.log('Real weather GUI passed: rainstorm/lightning -> recovery -> computed ENS; desktop/mobile; sparse parameters');
} finally {
  if (browser) await browser.close();
  child.kill();
}
