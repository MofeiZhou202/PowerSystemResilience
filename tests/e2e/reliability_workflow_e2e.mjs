// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';

const ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..', '..');

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
    } catch { /* server is starting */ }
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  throw new Error('GUI server did not start');
}

async function request(base, endpoint, body) {
  const response = await fetch(`${base}${endpoint}`, body === undefined ? {} : {
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

function metric(data, name) {
  const value = data?.metrics?.[name] ?? data?.[name];
  return Number(value && typeof value === 'object' ? value.value : value);
}

function fmeaRequest(loadScale, information) {
  return {
    method: 'fmea',
    data_policy: 'case_data_only',
    load: { hours_per_year: 8760 },
    execution: { parallel: false, parallel_threads: 0 },
    restoration: { max_switch_actions: 1, max_physical_evaluations: 40 },
    dimensions: {
      physical: {
        load_scale_factor: loadScale,
        switching_time_hr: 0.5,
        enable_switch_reconfiguration: true,
        enable_repair_reconfiguration: true,
        enable_microgrid_islanding: true,
        enable_storage_dispatch: true,
        enable_grid_forming_vsc_support: true,
        enable_black_start_storage: false,
      },
      information: information || { enabled: false },
      intelligent: { enabled: false },
    },
  };
}

function failureModeRequest() {
  return {
    method: 'failure_mode_fmea',
    physical_model: 'auto',
    data_policy: 'case_data_only',
    load: { scale_factor: 1, hours_per_year: 8760 },
    execution: { parallel: false, parallel_threads: 0 },
    failure_mode_scope: {
      include_passive: true,
      include_active_on_demand: true,
      include_physical: true,
      include_cyber_control: true,
      include_communication: true,
      include_measurement: true,
      include_protection_logic: true,
      include_human_operation: true,
      include_scheduled: false,
      only_in_service: true,
    },
    max_order: 1,
  };
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
    await request(base, '/api/session/load_builtin', {
      case: 'cyber_physical_reliability_demo',
    });

    const physicalBase = await request(base, '/api/session/run_reliability',
      fmeaRequest(1.0));
    const physicalStressed = await request(base, '/api/session/run_reliability',
      fmeaRequest(1.5));
    assert(metric(physicalStressed, 'eens_mwh_yr') > metric(physicalBase, 'eens_mwh_yr'),
      `physical load stress did not increase EENS: ${metric(physicalBase, 'eens_mwh_yr')} -> ` +
      `${metric(physicalStressed, 'eens_mwh_yr')}`);

    const automatic = {
      enabled: true,
      service_availability: 1.0,
      automatic_switching_time_hr: 0.05,
      manual_switching_time_hr: 1.0,
      freeze_der_on_service_loss: true,
    };
    const manual = { ...automatic, service_availability: 0.0 };
    const informationAvailable = await request(base, '/api/session/run_reliability',
      fmeaRequest(1.0, automatic));
    const informationUnavailable = await request(base, '/api/session/run_reliability',
      fmeaRequest(1.0, manual));
    assert(metric(informationUnavailable, 'eens_mwh_yr') >
      metric(informationAvailable, 'eens_mwh_yr'),
    'loss of information service did not increase EENS');
    assert(Number(informationUnavailable.cyber_physical?.delta_cyber_duration_mwh_yr) > 0 &&
      Number(informationUnavailable.cyber_physical?.delta_cyber_control_mwh_yr) > 0,
    'information impact was not attributed to both response delay and frozen control');

    const reliabilityBase = await request(base, '/api/session/run_reliability',
      failureModeRequest());
    const dominantMode = (reliabilityBase.contingencies || []).find(row =>
      row.supported !== false && row.activation === 'passive' &&
      Number(row.eens_contribution) > 0 && Number(row.frequency_per_year) > 0);
    assert(dominantMode, 'built-in case has no positive passive failure-mode contribution');
    const doubledFrequency = Number(dominantMode.frequency_per_year) * 2;
    await request(base, '/api/session/reliability/configuration', {
      configuration: {
        profile_id: 'workflow_e2e',
        mode_overrides: [{
          mode_id: dominantMode.mode_id,
          enabled: true,
          failure_rate_per_year: doubledFrequency,
          mtbf_hours: null,
          mttr_hours: null,
          forced_outage_rate: null,
          probability_given_initiated: null,
          demand_frequency_per_year: null,
          probability_per_demand: null,
          isolation_hr: null,
          switching_hr: null,
          repair_hr: null,
          cyber_recovery_hr: null,
          residual_capacity_factor: null,
        }],
        protection: [],
      },
    });
    const reliabilityStressed = await request(base, '/api/session/run_reliability',
      failureModeRequest());
    const stressedMode = (reliabilityStressed.contingencies || []).find(row =>
      row.mode_id === dominantMode.mode_id);
    assert(Math.abs(Number(stressedMode?.frequency_per_year) - doubledFrequency) < 1e-12,
      'custom failure rate did not reach the selected failure mode');
    assert(metric(reliabilityStressed, 'eens_mwh_yr') >
      metric(reliabilityBase, 'eens_mwh_yr'),
    'higher component failure rate did not increase failure-mode EENS');

    await request(base, '/api/session/reliability/configuration', {
      configuration: { profile_id: 'user_custom', mode_overrides: [], protection: [] },
    });

    await request(base, '/api/session/load_builtin', { case: 'dist33_microgrid_der' });
    const hybridThreeStage = await request(base, '/api/session/run_reliability', {
      method: 'three_stage',
      execution: { parallel: false },
      restoration: { parallel: false },
    });
    assert(hybridThreeStage.ok === false &&
      String(hybridThreeStage.error_detail || '').includes('connectivity fallback') &&
      hybridThreeStage.validity?.restoration_milp_solved === false &&
      (hybridThreeStage.faults || []).length > 0,
    'Dist33 DER hybrid three-stage boundary was not returned as a completed limited result');

    await request(base, '/api/session/load_builtin', { case: 'dist33_tie_demo' });
    const acThreeStage = await request(base, '/api/session/run_reliability', {
      method: 'three_stage',
      execution: { parallel: false },
      restoration: { parallel: false },
    });
    assert(acThreeStage.ok === true &&
      acThreeStage.validity?.restoration_milp_solved === true &&
      acThreeStage.validity?.branch_flow_enforced === true &&
      acThreeStage.validity?.voltage_constraints_enforced === true &&
      acThreeStage.validity?.radial_topology_enforced === true,
    'Dist33 pure-AC three-stage restoration did not return a valid MILP result');

    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
    await page.goto(`${base}/xjtu/`, { waitUntil: 'networkidle' });
    await page.evaluate(async () => {
      await App.loadBuiltinCase('cyber_physical_reliability_demo');
      App.setActiveModule('reliability');
    });
    await page.waitForFunction(() =>
      document.getElementById('relWorkflowComponents')?.textContent !== '待同步');
    assert(await page.locator('#relWorkflowStages > li').count() === 5,
      'reliability workflow does not expose five stages');

    const principleChecks = [
      ['nsq', '独立状态抽样'],
      ['seq', '逐时状态演化'],
      ['fmea', '元件 N-1 FMEA'],
      ['failure_mode_fmea', '失效模式 FMEA'],
      ['fd', '发电充裕度 COPT'],
    ];
    for (const [method, expected] of principleChecks) {
      await page.locator('#relMethod').selectOption(method);
      assert((await page.locator('#relPrincipleMethod').textContent())?.includes(expected),
        `${method} calculation principle did not update`);
    }
    await page.locator('#relPhysicalModel').selectOption('restoration_milp');
    assert((await page.locator('#relPrincipleMethod').textContent())?.includes('三阶段恢复 MILP'),
      'restoration consequence model did not switch the displayed principle');

    await page.locator('#relPhysicalModel').selectOption('auto');
    await page.locator('#relMethod').selectOption('fmea');
    await page.locator('#relDataPolicy').selectOption('case_data_only');
    await page.locator('#relCyberEnabled').check();
    await page.route('**/api/session/run_reliability', async route => {
      await new Promise(resolve => setTimeout(resolve, 300));
      await route.continue();
    }, { times: 1 });
    await page.locator('#btnRunReliability').click();
    await page.waitForSelector('#relWorkflowRunStatus[data-state="running"]');
    await page.waitForFunction(() => {
      const state = document.getElementById('relWorkflowRunStatus')?.dataset.state;
      return state === 'complete' || state === 'limited';
    });
    const workflow = await page.evaluate(() => ({
      states: Array.from(document.querySelectorAll('#relWorkflowStages > li'),
        row => row.getAttribute('data-state')),
      status: document.getElementById('relWorkflowRunStatus')?.textContent || '',
      limitationsVisible: !document.getElementById('relWorkflowLimitations')?.hidden,
      resultVisible: !!document.querySelector('#reliabilityResults .reliability-kpi-grid'),
      coreVisibleWithinToolbar: (() => {
        const toolbar = document.getElementById('subToolbar')?.getBoundingClientRect();
        const stages = document.getElementById('relWorkflowStages')?.getBoundingClientRect();
        const limitations = document.getElementById('relWorkflowLimitations')?.getBoundingClientRect();
        return !!toolbar && !!stages && stages.bottom <= toolbar.bottom + 1 &&
          (!limitations || limitations.top <= toolbar.bottom + 1);
      })(),
    }));
    assert(workflow.states.every(state => state === 'complete' || state === 'limited'),
      `workflow did not reach terminal stage states: ${JSON.stringify(workflow.states)}`);
    assert(workflow.limitationsVisible,
      'runtime model limitations are not visible in the workflow');
    assert(workflow.resultVisible, 'reliability result dashboard was not rendered');
    assert(workflow.coreVisibleWithinToolbar,
      'reliability stages or model-boundary content are clipped by the contextual toolbar');

    await page.setViewportSize({ width: 390, height: 844 });
    await page.waitForTimeout(100);
    const layout = await page.evaluate(() => {
      const workflowElement = document.getElementById('relWorkflow');
      const rect = workflowElement?.getBoundingClientRect();
      return {
        pageOverflow: document.documentElement.scrollWidth - window.innerWidth,
        workflowRight: rect?.right || 0,
        viewport: window.innerWidth,
      };
    });
    assert(layout.pageOverflow <= 1 && layout.workflowRight <= layout.viewport + 1,
      `mobile workflow overflows horizontally: ${JSON.stringify(layout)}`);

    console.log(JSON.stringify({
      case: 'cyber_physical_reliability_demo',
      physical_eens: [metric(physicalBase, 'eens_mwh_yr'),
        metric(physicalStressed, 'eens_mwh_yr')],
      information_eens: [metric(informationAvailable, 'eens_mwh_yr'),
        metric(informationUnavailable, 'eens_mwh_yr')],
      reliability_eens: [metric(reliabilityBase, 'eens_mwh_yr'),
        metric(reliabilityStressed, 'eens_mwh_yr')],
      dist33_hybrid_three_stage_ok: hybridThreeStage.ok,
      dist33_ac_three_stage_ok: acThreeStage.ok,
      workflow_states: workflow.states,
      mobile_page_overflow_px: layout.pageOverflow,
    }));
  } finally {
    if (browser) await browser.close();
    proc.kill();
  }
}

main().catch(error => {
  console.error(error.stack || error.message || error);
  process.exitCode = 1;
});
