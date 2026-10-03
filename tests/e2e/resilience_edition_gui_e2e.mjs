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

const MOCK_SYSTEM = {
  name: 'Mock resilience feeder', base_mva: 100,
  ac: {
    buses: [
      { index: 1, name: 'Bus 1', bus_type: 'SLACK', base_kv: 10 },
      { index: 2, name: 'Bus 2', bus_type: 'PQ', base_kv: 10 },
      { index: 3, name: 'Bus 3', bus_type: 'PQ', base_kv: 10 },
    ],
    branches: [
      { index: 25, from_bus: 1, to_bus: 2, r_pu: 0.01, x_pu: 0.03, status: true },
      { index: 30, from_bus: 2, to_bus: 3, r_pu: 0.02, x_pu: 0.04, status: true },
    ],
    generators: [{ index: 1, bus: 1, name: 'Grid', pg_mw: 4, qg_mvar: 0, pmax_mw: 10, pmin_mw: 0, status: true }],
    loads: [{ index: 1, bus: 2, name: 'Critical load', pd_mw: 2, qd_mvar: 0.4, status: true }],
  },
  dc: {
    buses: [{ index: 1, name: 'DC Bus 1', base_kv: 10 }],
    branches: [{ index: 25, from_bus: 1, to_bus: 1, r_pu: 0.01, status: true }],
    loads: [],
  },
};

function scenarioProfiles(offset = 0) {
  return Array.from({ length: 36 }, (_, index) => ({
    id: index + 1,
    name: index === 0 ? 'scenario_load_scale' : index === 1 ? 'scenario_pv_scale' : index === 2 ? 'scenario_wind_scale' : `profile_${index + 1}`,
    values: Array.from({ length: 48 }, (__, hour) => Number((0.82 + offset + ((hour + index) % 8) * 0.02).toFixed(2))),
  }));
}

function mockScenarioResponse() {
  const representative = ({ id, intensity, clusterId, probability, faults, profileOffset }) => ({
    cluster_id: clusterId,
    representative_id: id,
    probability,
    member_count: 10,
    member_ids: Array.from({ length: 10 }, (_, index) => `${id}:member:${index + 1}`),
    anchor_reasons: ['tail_risk'],
    representative: {
      id,
      features: { fault_count: faults.length, selected_track_max_vmax_ms: intensity === 'SuperTY' ? 62 : 48 },
      resilience_event: {
        selected_intensity: intensity,
        selected_track_max_vmax_ms: intensity === 'SuperTY' ? 62 : 48,
        faults,
      },
      standard_time_series: {
        num_steps: 48,
        step_duration_hr: 1,
        profiles: scenarioProfiles(profileOffset),
        binding: {
          resilience_load_profile_id: 1,
          resilience_pv_profile_id: 2,
          resilience_wind_profile_id: 3,
          load_profile_map: [{ load_id: 1, profile_id: 1 }],
        },
      },
    },
  });
  return {
    success: true,
    scenario_generation_id: 'rgen-mock-1',
    model_revision: 41,
    generation_ms: 4.5,
    warnings: ['Mock generation warning'],
    summary: { regular_cluster_count: 0, reliability_contingency_count: 0, resilience_cluster_total: 2 },
    resilience: {
      audit: { method: 'hybrid_kmedoids_tail_5pct' },
      intensities: [{
        intensity: 'SuperTY',
        candidate_count: 20,
        cluster_count: 2,
        clusters: [
          representative({
            id: 'resilience:SuperTY:A', intensity: 'SuperTY', clusterId: 0, probability: 0.6, profileOffset: 0,
            faults: [
              { branch_type: 'AC', branch_index: 25, start_hr: 19, repair_duration_hr: 19, name: 'AC branch 25' },
              { branch_type: 'DC', branch_index: 25, start_hr: 20, repair_duration_hr: 11, name: 'DC branch 25' },
            ],
          }),
          representative({
            id: 'resilience:SuperTY:B', intensity: 'SuperTY', clusterId: 1, probability: 0.4, profileOffset: 0.08,
            faults: [
              { branch_type: 'AC', branch_index: 30, start_hr: 8, repair_duration_hr: 7, name: 'AC branch 30' },
            ],
          }),
        ],
      }],
    },
  };
}

function profile(edition) {
  const value = {
    schema: 'hacdcpf.edition-profile.v1',
    edition,
    product_name: { full: 'HySim-XJTU-HRPES', trial: 'HySim-XJTU-HRPES Trial', resilience: 'PowerSystemResilience' }[edition],
    analyses: [...ANALYSES],
    analysis_catalog: { schema: 'hacdcpf.edition-analysis-catalog.v1', entries: [] },
    workflow: structuredClone(edition === 'resilience' ? [
      { id: 'metric_selection', label: '指标选择' },
      { id: 'scenario_selection', label: '场景生成' },
      { id: 'proactive_defense', label: '主动防御' },
      { id: 'rapid_recovery', label: '快速恢复' },
      { id: 'metric_output', label: '指标输出' },
    ] : WORKFLOW),
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
  if (edition === 'resilience') {
    const portalCatalog = Array.from({ length: 42 }, (_, index) => ({
      id: index < 18 ? `ch3.metric_${index + 1}` : `run.metric_${index + 1}`, name_zh: `指标 ${index + 1}`, name_en: `Metric ${index + 1}`,
      symbol: `M${index + 1}`, phase: index >= 18 ? 'operational' : index < 8 ? 'pre_disaster' : index < 11 ? 'during_disaster' : 'post_disaster',
      topic: 'test', formula_ref: 'book_ch3_test', unit: '—', direction: 'higher_is_better',
      calculation_scope: 'test', availability: index < 18 ? 'unavailable' : 'available', required_inputs: [], source_notes: [], limitations: [],
    }));
    value.resilience_metric_catalog = {
      schema: 'resilience_metric_catalog_v1', definition_version: 'book_ch3_2026.2', entries: portalCatalog,
    };
    value.scenario_hazards = [
      { id: 'typhoon', label: '台风', fields: [] },
      { id: 'rainstorm', label: '暴雨内涝', fields: [] },
      { id: 'lightning', label: '雷暴雷击', fields: [] },
    ];
  }
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
  const traffic = scenario.traffic || { requests: [] };
  scenario.traffic = traffic;
  const record = (request, pathname) => {
    let body = null;
    try { body = request.postDataJSON(); } catch {}
    traffic.requests.push({ method: request.method(), pathname, body });
  };
  const fulfillJson = (route, body, status = 200) => route.fulfill({
    status, contentType: 'application/json', body: JSON.stringify(body),
  });
  await page.addInitScript(() => {
    try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {}
    window.__canvasModelLoadCalls = 0;
    window.__portalPlotCalls = [];
    window.__portalPlotPurges = [];
    const markPlot = (target, traces, layout, config) => {
      const element = typeof target === 'string' ? document.getElementById(target) : target;
      if (element) element.classList.add('js-plotly-plot');
      window.__portalPlotCalls.push({
        targetIsElement: target instanceof HTMLElement,
        targetInPortal: !!element?.closest('#resiliencePortalRoot'),
        targetInLegacyShell: !!element?.closest('#appShell'),
        resultKind: element?.closest('[data-portal-results]')?.dataset.portalResults || null,
        x: traces?.map(trace => Array.isArray(trace.x) ? [...trace.x] : null),
        traceNames: traces?.map(trace => trace.name || '') || [],
        showlegend: layout?.showlegend,
        responsive: config?.responsive,
      });
      return Promise.resolve();
    };
    window.Plotly = {
      purge(target) {
        const element = typeof target === 'string' ? document.getElementById(target) : target;
        window.__portalPlotPurges.push({ targetIsElement: target instanceof HTMLElement, resultKind: element?.closest('[data-portal-results]')?.dataset.portalResults || null });
        element?.classList.remove('js-plotly-plot');
      },
      newPlot: markPlot,
      react: markPlot,
      Plots: { resize() {} },
    };
  });
  await page.route('https://cdn.plot.ly/**', route => route.fulfill({
    status: 200, contentType: 'text/javascript', body: 'window.Plotly=window.Plotly||{purge(){},newPlot(){},react(){},Plots:{resize(){}}};',
  }));
  await page.route('**/api/**', async route => {
    const request = route.request();
    const url = new URL(request.url());
    record(request, url.pathname);
    if (url.pathname === '/api/edition') {
      if (scenario.mode === 'delayed') {
        await new Promise(resolve => setTimeout(resolve, 350));
        await fulfillJson(route, scenario.profile);
      } else if (scenario.mode === 'hung') {
        await route.fallback({ url: `${url.origin}/api/edition?fixture=hung` });
      } else if (scenario.mode === 'http') {
        await fulfillJson(route, { error: 'offline' }, 503);
      } else if (scenario.mode === 'json') {
        await route.fulfill({ status: 200, contentType: 'application/json', body: '{not-json' });
      } else {
        await fulfillJson(route, scenario.profile);
      }
      return;
    }
    if (url.pathname === '/api/cases') {
      await fulfillJson(route, {
        default_case: 'ieee14_acdc',
        cases: [
          { name: 'ieee14_acdc', label: 'IEEE 14 AC/DC', group: 'Transmission' },
          { name: 'dist33_microgrid_der', label: '33 节点弹性配电网', group: 'Resilience' },
          { name: 'dist33_weather_mixed', label: '33 节点架空线与电缆算例', group: 'Resilience' },
        ],
      }); return;
    }
    if (url.pathname === '/api/matpower_files') {
      await fulfillJson(route, { files: ['case14.m'] }); return;
    }
    if (url.pathname === '/api/session/load_builtin') {
      traffic.backendRevision = (traffic.backendRevision || 0) + 1;
      await fulfillJson(route, {
        name: request.postDataJSON()?.case || 'dist33_microgrid_der',
        counts: { ac_buses: 3, ac_branches: 2, dc_buses: 1, dc_branches: 1, generators: 1 },
        _raw_json: JSON.stringify(MOCK_SYSTEM), model_revision: traffic.backendRevision,
      }); return;
    }
    if (url.pathname === '/api/session/load_matpower') {
      traffic.backendRevision = (traffic.backendRevision || 0) + 1;
      await fulfillJson(route, {
        name: request.postDataJSON()?.filename || 'case14.m', counts: { ac_buses: 3, ac_branches: 2, generators: 1 },
        _raw_json: JSON.stringify({ ...MOCK_SYSTEM, name: 'MATPOWER case14' }), model_revision: traffic.backendRevision,
      }); return;
    }
    if (url.pathname === '/api/session/load_json_string') {
      traffic.backendRevision = (traffic.backendRevision || 0) + 1;
      let imported = MOCK_SYSTEM;
      try { imported = JSON.parse(request.postDataJSON()?.json_string || '{}'); } catch {}
      await fulfillJson(route, {
        name: imported.name || 'Imported JSON', counts: { ac_buses: imported.ac?.buses?.length || 0, ac_branches: imported.ac?.branches?.length || 0, generators: imported.ac?.generators?.length || 0 },
        _raw_json: JSON.stringify(imported), model_revision: traffic.backendRevision,
      }); return;
    }
    if (url.pathname === '/api/session/generate_scenarios') {
      if (scenario.scenarioError) {
        await fulfillJson(route, { error: { code: 'NO_SYSTEM', message: scenario.scenarioError } }, 400);
      } else {
        await fulfillJson(route, mockScenarioResponse());
      }
      return;
    }
    if (url.pathname === '/api/session/resilience/portfolio_plan') {
      const body = request.postDataJSON() || {};
      await fulfillJson(route, {
        schema: 'hacdcpf.resilience_portfolio.v1', status: 'computed', plan_id: 'rplan-mock-1',
        scenario_generation_id: body.scenario_generation_id, cluster_count: 2,
        baseline_design_weighted_shed_mwh: 2.6, planned_design_weighted_shed_mwh: 1.8,
        baseline_worst_shed_mwh: 3.0, planned_worst_shed_mwh: 2.2,
        plan: { ac_generator_bus: 2, ac_generator_mw: body.add_generator ? 0.5 : 0,
          mobile_storage_bus: 3, mobile_storage_mw: body.add_mobile_storage ? 0.5 : 0,
          mobile_storage_mwh: body.add_mobile_storage ? 2 : 0 },
        scenarios: [
          { scenario_id: 'resilience:SuperTY:A', design_weight: 0.6, baseline_shed_mwh: 3, planned_shed_mwh: 2.2 },
          { scenario_id: 'resilience:SuperTY:B', design_weight: 0.4, baseline_shed_mwh: 2, planned_shed_mwh: 1.2 },
        ], limitations: ['演示选址容量'], investment_cost: null,
      }); return;
    }
    if (url.pathname === '/api/session/run_distribution_resilience') {
      const body = request.postDataJSON() || {};
      traffic.runSequence = (traffic.runSequence || 0) + 1;
      await fulfillJson(route, {
        run_id: `rrun-${traffic.runSequence}`, run_revision: traffic.runSequence,
        model_revision: 41, scenario_ref: body.scenario_ref, portfolio_plan_id: body.portfolio_plan_id,
        scenario_revision: body.scenario_revision, scenario_digest: body.scenario_digest,
        execution_mode: 'legacy_full_distribution_resilience', scientific_usability: 'valid_with_limitations',
        outcome: 'feasible', stale: false,
        time_axis: [0, 0.5, 1, 1.75],
        steps: [
          { time_hr: 0, demand_mw: 4, served_mw: 2.4, shed_mw: 1.6, restoration_ratio: 0.6, active_faults: ['AC:25', 'DC:25'], repaired_faults: 0, switch_actions: [] },
          { time_hr: 0.5, demand_mw: 4, served_mw: 2.8, shed_mw: 1.2, restoration_ratio: 0.7, active_faults: ['AC:25'], repaired_faults: 1, switch_actions: ['open AC:25'] },
          { time_hr: 1, demand_mw: 4, served_mw: 3.4, shed_mw: 0.6, restoration_ratio: 0.85, active_faults: ['AC:25'], repaired_faults: 1, switch_actions: [] },
          { time_hr: 1.75, demand_mw: 4, served_mw: 3.8, shed_mw: 0.2, restoration_ratio: 0.95, active_faults: [], repaired_faults: 2, switch_actions: ['close AC:25'] },
        ],
        faults: body.manual_faults,
        total_demand_mwh: 6.8,
        total_served_mwh: 5.4,
        total_shed_mwh: 1.4,
        resilience_index: 0.79,
        final_restoration_ratio: 0.95,
        peak_shed_mw: 1.6,
        total_switch_actions: 2,
        total_repaired_faults: 2,
        mess_traces: [{
          unit_id: 1,
          hours: [0, 0.5, 1, 1.75],
          power_mw: [0, 0.2, 0.3, 0.1],
          energy_mwh: [1, 0.9, 0.75, 0.7],
          bus_ids: [1, 1, 2, 2],
        }],
        limitations: ['Ordinary restoration feasibility is not certified dynamic safety.'],
      }); return;
    }
    if (url.pathname === '/api/session/resilience/metrics') {
      const body = request.postDataJSON() || {};
      await fulfillJson(route, {
        schema: 'resilience_metric_result_v1', definition_version: 'book_ch3_2026.2',
        run_id: body.run_id, run_revision: traffic.runSequence || 1, model_revision: 41,
        selection_revision: body.selection_revision, stale: false,
        results: (body.selected_metric_ids || []).map(id => ({
          id, status: id.startsWith('run.') ? 'computed' : 'unavailable', value: id.startsWith('run.') ? 0.25 : null, unit: id.startsWith('run.') ? '1' : '—', formula_ref: 'book_ch3_test',
          reason_code: 'PROBABILISTIC_INPUTS_MISSING', limitations: ['单次确定性运行不能解释为概率指标'], missing_dependencies: ['scenario_probabilities'],
        })),
      }); return;
    }
    if (url.pathname === '/api/dynamics/model_schema') {
      await fulfillJson(route, { models: [], components: [] }); return;
    }
    if (url.pathname === '/api/edition/analysis_plan') {
      await fulfillJson(route, { steps: [] }); return;
    }
    if (url.pathname === '/api/session/resilience/metric_catalog') {
      const entries = scenario.profile.resilience_metric_catalog?.entries || [];
      await fulfillJson(route, { schema: 'resilience_metric_catalog_v1', definition_version: 'book_ch3_2026.2', entries }); return;
    }
    await fulfillJson(route, {});
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

function recorded(traffic, pathname, method = null) {
  return traffic.requests.filter(request => request.pathname === pathname && (!method || request.method === method));
}

async function portalStep(page, id) {
  await page.locator('#resiliencePortalWorkflowTab').click();
  await page.locator(`[data-resilience-step="${id}"]`).click();
  await page.waitForFunction(step => document.querySelector(`[data-resilience-step="${step}"]`)?.getAttribute('aria-selected') === 'true', id);
}

async function exerciseResilienceWorkflow(page, scenario) {
  const traffic = scenario.traffic;
  await portalStep(page, 'scenario_selection');
  await page.waitForFunction(() => document.querySelector('[data-portal-field="builtin-case"] option[value="dist33_weather_mixed"]'));
  check(await page.locator('[data-portal-field="builtin-case"]').inputValue() === 'dist33_weather_mixed',
    'resilience workflow: flagship case is not selected by default');
  const generate = page.getByRole('button', { name: '生成弹性候选场景' });
  check(await generate.isDisabled(), 'resilience workflow: generation enabled before a model and metric selection');

  await portalStep(page, 'metric_selection');
  await page.locator('.resilience-portal__metric-card input[type="checkbox"]').first().check();
  await portalStep(page, 'scenario_selection');
  check(await generate.isDisabled(), 'resilience workflow: generation enabled before model load');

  await page.getByRole('button', { name: '加载内置算例' }).click();
  await page.waitForFunction(() => document.querySelector('.resilience-portal__model-state')?.textContent?.includes('dist33_weather_mixed'));
  check(recorded(traffic, '/api/session/load_builtin', 'POST').length === 1,
    'resilience workflow: built-in load request missing or duplicated');
  check(!await generate.isDisabled(), 'resilience workflow: generation stayed disabled after prerequisites');

  await page.locator('[data-portal-field="scenario-cluster-count"]').fill('1');
  await page.locator('[data-portal-field="scenario-cluster-count"]').press('Tab');
  await page.locator('[data-portal-field="scenario-compare-baseline"]').uncheck();
  for (const level of ['TD', 'TS', 'STS', 'TY', 'STY']) {
    await page.locator(`[data-portal-intensity="${level}"]`).uncheck();
  }
  check(await page.locator('[data-portal-intensity="SuperTY"]').isChecked(),
    'resilience workflow: intended SuperTY intensity was not retained');

  await Promise.all([
    page.waitForFunction(() => document.querySelectorAll('.resilience-portal__table tr').length > 1),
    generate.dblclick(),
  ]);
  const generationRequests = recorded(traffic, '/api/session/generate_scenarios', 'POST');
  check(generationRequests.length === 1, `resilience workflow: duplicate generation requests ${generationRequests.length}`);
  const generation = generationRequests[0].body;
  check(generation?.regular?.enabled === false && generation?.reliability?.enabled === false && generation?.resilience?.enabled === true,
    `resilience workflow: hidden scenario families were enabled ${JSON.stringify(generation)}`);
  check(generation?.resilience?.default_cluster_count === 1 && generation?.resilience?.candidates_per_intensity === 10 &&
    JSON.stringify(generation?.resilience?.intensity_levels) === JSON.stringify(['SuperTY']),
    `resilience workflow: scenario parameters were not forwarded ${JSON.stringify(generation?.resilience)}`);
  check(generation?.clustering?.compare_baseline === false && generation?.clustering?.method === 'hybrid_kmedoids_tail_5pct',
    'resilience workflow: reduction controls were not forwarded');

  const generationPlotState = await page.evaluate(() => ({
    calls: window.__portalPlotCalls.filter(call => call.resultKind === 'scenario-generation'),
    legacyScenarioHtml: document.getElementById('scenarioGenerationResults')?.innerHTML || '',
  }));
  check(generationPlotState.calls.length >= 2,
    `resilience workflow: scenario evidence plots missing ${JSON.stringify(generationPlotState.calls)}`);
  check(generationPlotState.calls.every(call => call.targetIsElement && call.targetInPortal && !call.targetInLegacyShell && call.responsive === true),
    `resilience workflow: scenario plots escaped portal scope ${JSON.stringify(generationPlotState.calls)}`);
  check(generationPlotState.calls.some(call => call.traceNames.includes('代表场景')) &&
    generationPlotState.calls.some(call => call.traceNames.includes('scenario_load_scale') && call.showlegend === true),
    `resilience workflow: scenario coverage/profile evidence incomplete ${JSON.stringify(generationPlotState.calls)}`);
  check(generationPlotState.legacyScenarioHtml.includes('class="empty-hint"') &&
    generationPlotState.legacyScenarioHtml.includes('生成场景后查看弹性台风聚类结果') &&
    !generationPlotState.legacyScenarioHtml.includes('<table') &&
    !generationPlotState.legacyScenarioHtml.includes('js-plotly-plot'),
    `resilience workflow: portal renderer wrote legacy scenario results ${generationPlotState.legacyScenarioHtml}`);

  const scenarioScrollState = await page.locator('.resilience-portal__main').evaluate(element => ({
    clientHeight: element.clientHeight,
    scrollHeight: element.scrollHeight,
    scrollTop: element.scrollTop,
    overflowY: getComputedStyle(element).overflowY,
    tableTop: element.querySelector('.resilience-portal__table-wrap')?.getBoundingClientRect().top ?? null,
  }));
  check(scenarioScrollState.scrollHeight > scenarioScrollState.clientHeight &&
    ['auto', 'scroll'].includes(scenarioScrollState.overflowY),
    `resilience workflow: generated candidates are not reachable by scrolling ${JSON.stringify(scenarioScrollState)}`);
  check(scenarioScrollState.tableTop !== null,
    `resilience workflow: generated representative table missing ${JSON.stringify(scenarioScrollState)}`);
  check(await page.locator('[data-portal-results="scenario-generation"]').evaluate(root => {
    const summary = root.querySelector('[data-portal-calculation-summary="scenario-generation"]');
    return summary === root.lastElementChild && summary?.open === false;
  }), 'scenario calculation summary must be collapsed after the results');
  await portalStep(page, 'proactive_defense');
  check(await page.locator('[data-portal-planning-run]').isEnabled(), 'planning cannot run on generated representatives');
  await page.locator('[data-portal-planning-run]').click();
  await page.waitForFunction(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.proactiveDefense.status === 'success');
  const planningRequests = recorded(traffic, '/api/session/resilience/portfolio_plan', 'POST');
  check(planningRequests.length === 1 && planningRequests[0].body.scenario_generation_id === 'rgen-mock-1' &&
    planningRequests[0].body.add_generator === true && planningRequests[0].body.add_mobile_storage === true,
    'all-cluster planning request was missing or malformed');
  check(await page.locator('#resiliencePortalPanel').getByText('resilience:SuperTY:A').count() > 0 &&
    await page.locator('#resiliencePortalPanel').getByText('resilience:SuperTY:B').count() > 0,
    'planning result did not cover both representative scenarios');
  await portalStep(page, 'rapid_recovery');
  const firstScenarioButton = page.locator('[data-portal-scenario-select="resilience:SuperTY:A"]');
  await firstScenarioButton.click();
  await page.waitForFunction(() => document.querySelector('.resilience-portal__status')?.textContent?.includes('已选择场景'));
  const selected = await page.evaluate(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.scenarioGeneration.selectedScenario);
  check(selected.id === 'resilience:SuperTY:A' && selected.scenario_ref === selected.id &&
    selected.scenario_revision > 0 && /^fnv1a32:[0-9a-f]{8}$/.test(selected.scenario_digest),
    `resilience workflow: stable scenario identity missing ${JSON.stringify(selected)}`);

  for (const domain of ['AC', 'DC']) {
    await page.getByText('在电网中查看故障（2 条）', { exact: true }).click();
    await page.getByRole('button', { name: `${domain} 支路 25`, exact: true }).click();
    check(await page.evaluate(d => {
      const bucket = d === 'AC' ? 'branch' : 'dcBranch';
      return Canvas.state.selectedId === Canvas.getCompBusMap()[bucket][25] &&
        document.getElementById('resiliencePortalArchitectureSurface').hidden === false;
    }, domain), `workspace: ${domain} stable fault link selected the wrong component`);
    await portalStep(page, 'rapid_recovery');
  }
  check(await page.locator('[data-portal-field^="recovery-"]').count() === 1 &&
    await page.locator('[data-portal-field="recovery-allow_mess_dispatch"]').count() === 1,
    'rapid recovery must expose only mobile storage scheduling');
  check(await page.evaluate(() => {
    const c = document.getElementById('resiliencePortalRoot').__resiliencePortal.effectiveRecoveryConfig();
    return c.fault_count === 2 && c.ac_fault_branch_ids_text === '25' &&
      c.dc_fault_branch_ids_text === '25' && c.ac_fault_start_hours_text === '19' &&
      c.dc_fault_start_hours_text === '20' && c.ac_repair_durations_text === '19' &&
      c.dc_repair_durations_text === '11';
  }), 'selected AC/DC fault identities and schedules were not hydrated');
  await page.locator('[data-portal-scenario-select="resilience:SuperTY:B"]').click();
  await page.waitForFunction(() => document.querySelector('.resilience-portal__status')?.textContent?.includes('resilience:SuperTY:B'));
  await portalStep(page, 'rapid_recovery');
  check(await page.evaluate(() => {
    const c = document.getElementById('resiliencePortalRoot').__resiliencePortal.effectiveRecoveryConfig();
    return c.fault_count === 1 && c.ac_fault_branch_ids_text === '30' &&
      c.dc_fault_branch_ids_text === '' && c.ac_fault_start_hours_text === '8' &&
      c.ac_repair_durations_text === '7';
  }), 'selecting scenario B did not replace scenario A fault data');
  await page.locator('[data-portal-scenario-select="resilience:SuperTY:A"]').click();
  await page.waitForFunction(() => document.querySelector('.resilience-portal__status')?.textContent?.includes('resilience:SuperTY:A'));
  await portalStep(page, 'rapid_recovery');
  await page.locator('[data-portal-field="recovery-allow_mess_dispatch"]').check();
  await page.getByRole('button', { name: '运行快速恢复', exact: true }).dblclick();
  await page.waitForFunction(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.recovery.status === 'success');
  check(await page.locator('#resiliencePortalPanel').evaluate(root => {
    const summary = root.querySelector('[data-portal-calculation-summary="rapid-recovery"]');
    return summary === root.lastElementChild && summary?.open === false &&
      summary.querySelector('[data-portal-run-evidence]') !== null &&
      root.querySelector('[data-portal-results="rapid-recovery"] .resilience-portal__chart') !== null;
  }), 'recovery summary must follow charts and contain the detailed run evidence');
  const recoveryRequests = recorded(traffic, '/api/session/run_distribution_resilience', 'POST');
  check(recoveryRequests.length === 1, `resilience workflow: duplicate recovery requests ${recoveryRequests.length}`);
  const recovery = recoveryRequests[0].body;
  check(recovery.default_fault_count === 2 && recovery.manual_faults?.length === 2 &&
    recovery.manual_faults[0].branch_type === 'AC' && recovery.manual_faults[0].branch_id === 25 &&
    recovery.manual_faults[0].start_hr === 19 && recovery.manual_faults[0].repair_hr === 19 &&
    recovery.manual_faults[1].branch_type === 'DC' && recovery.manual_faults[1].branch_id === 25 &&
    recovery.manual_faults[1].start_hr === 20 && recovery.manual_faults[1].repair_hr === 11,
    `resilience workflow: selected faults were not forwarded ${JSON.stringify(recovery.manual_faults)}`);
  const selectedForRecovery = await page.evaluate(() =>
    document.getElementById('resiliencePortalRoot').__resiliencePortal.state.scenarioGeneration.selectedScenario);
  check(recovery.scenario_ref === 'resilience:SuperTY:A' && recovery.scenario_revision === selectedForRecovery.scenario_revision &&
    recovery.scenario_digest === selectedForRecovery.scenario_digest &&
    recovery.portfolio_plan_id === 'rplan-mock-1' && recovery.apply_demo_data === false,
    `resilience workflow: scenario identity was not forwarded ${JSON.stringify(recovery)}`);
  check(recovery.scenario_profiles?.length === 36 && recovery.horizon_hours === 48,
    `resilience workflow: profiles or repair-aware horizon missing ${JSON.stringify({ profiles: recovery.scenario_profiles?.length, horizon: recovery.horizon_hours })}`);
  check(recovery.mip_time_limit_s === 180 && recovery.mip_gap === 0.03 &&
    recovery.consider_switches === false && recovery.allow_mess_dispatch === true,
    `resilience workflow: defaults or MESS switch were not forwarded ${JSON.stringify({ mip_time_limit_s: recovery.mip_time_limit_s, mip_gap: recovery.mip_gap, consider_switches: recovery.consider_switches, allow_mess_dispatch: recovery.allow_mess_dispatch })}`);
  const recoveryPlotState = await page.evaluate(() => ({
    calls: window.__portalPlotCalls.filter(call => call.resultKind === 'rapid-recovery'),
    legacyResultsHtml: document.getElementById('resilienceResults')?.innerHTML || '',
  }));
  check(recoveryPlotState.calls.length >= 5,
    `resilience workflow: recovery evidence plots missing ${JSON.stringify(recoveryPlotState.calls)}`);
  check(recoveryPlotState.calls.every(call => call.targetIsElement && call.targetInPortal && !call.targetInLegacyShell && call.responsive === true),
    `resilience workflow: recovery plots escaped portal scope ${JSON.stringify(recoveryPlotState.calls)}`);
  check(recoveryPlotState.calls.some(call => call.traceNames.includes('需求') && call.traceNames.includes('供电') && call.traceNames.includes('切负荷') &&
    JSON.stringify(call.x?.[0]) === JSON.stringify([0, 0.5, 1, 1.75]) && call.showlegend === true),
    `resilience workflow: nonuniform recovery axis or legend was lost ${JSON.stringify(recoveryPlotState.calls)}`);
  check(recoveryPlotState.calls.some(call => call.traceNames.includes('MESS 1')),
    `resilience workflow: backend MESS evidence was not plotted ${JSON.stringify(recoveryPlotState.calls)}`);
  check(recoveryPlotState.legacyResultsHtml.includes('class="empty-hint"') &&
    recoveryPlotState.legacyResultsHtml.includes('运行弹性分析后在此查看结果') &&
    !recoveryPlotState.legacyResultsHtml.includes('<table') &&
    !recoveryPlotState.legacyResultsHtml.includes('js-plotly-plot'),
    `resilience workflow: portal renderer wrote legacy resilience results ${recoveryPlotState.legacyResultsHtml}`);

  await portalStep(page, 'metric_output');
  await page.evaluate(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    p.state.metricParameters = { 'ch3.t_sp': { target_ratio: 0.5 }, event_window: { disaster_end_hr: 2 } };
    p.state.approximationConsents = { apda: true, res: true };
  });
  await page.getByRole('button', { name: '计算并输出指标' }).dblclick();
  await page.waitForFunction(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.metrics.status === 'success');
  check(await page.locator('#resiliencePortalPanel').evaluate(root => {
    const summary = root.querySelector('[data-portal-calculation-summary="metrics"]');
    return summary === root.lastElementChild && summary?.open === false;
  }), 'metric calculation summary must be collapsed after the result table');
  const metricRequests = recorded(traffic, '/api/session/resilience/metrics', 'POST');
  check(metricRequests.length === 1, `resilience workflow: duplicate metric requests ${metricRequests.length}`);
  check(recorded(traffic, '/api/session/run_distribution_resilience', 'POST').length === 1,
    'resilience workflow: metric output reran recovery');
  const metric = metricRequests[0].body;
  check(metric.run_id === 'rrun-1' && metric.current_model_revision === 41 && metric.selection_revision === 1 &&
    metric.selected_metric_ids?.length === 1,
    `resilience workflow: metric identity mismatch ${JSON.stringify(metric)}`);
  check(!Object.hasOwn(metric, 'parameters') &&
    !Object.hasOwn(metric, 'allow_apda_system_gap_approximation') &&
    !Object.hasOwn(metric, 'allow_res_approximation'),
    `resilience workflow: legacy metric overrides bypassed backend defaults ${JSON.stringify(metric)}`);
  check(await page.getByText('不可用（null）').count() === 1,
    'resilience workflow: nullable unavailable result was not rendered explicitly');
  check(await page.locator('#resiliencePortalPanel').getByText('不可用（null）').count() === 1,
    'resilience workflow: unavailable value rendering is ambiguous');

  await page.getByLabel('只看有数值的指标').check();
  check(await page.locator('#resiliencePortalPanel tbody tr:visible').count() === 0, 'unavailable filter failed');
  await portalStep(page, 'metric_selection');
  check(await page.getByRole('heading', { name: '指标计算口径', exact: true }).count() === 0 &&
    await page.locator('#resiliencePortalPanel input[type="number"]').count() === 0,
    'metric selection still exposes numeric calculation settings');
  await page.getByRole('button', { name: '选择本次可算指标' }).click();
  check(await page.locator('.resilience-portal__metric-card input:checked').count() === 24, 'recommended selection must include 24 operational metrics');
  await portalStep(page, 'metric_output');
  await page.getByRole('button', { name: '计算并输出指标' }).click();
  await page.waitForFunction(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.metrics.status === 'success');
  check(await page.locator('#resiliencePortalPanel tbody tr').count() === 24, 'operational rows missing');
  check(await page.locator('#resiliencePortalPanel tbody').getByText('25.00%', { exact: true }).count() === 24, 'backend ratio formatting failed');
  const downloadPromise = page.waitForEvent('download');
  await page.getByRole('button', { name: '导出 CSV' }).click();
  const download = await downloadPromise;
  const csv = await readFile(await download.path(), 'utf8');
  check(csv.includes('run.metric_19') && csv.includes('0.25'), 'CSV must retain raw backend values');
  check(recorded(traffic, '/api/session/run_distribution_resilience', 'POST').length === 1, 'metric reselection reran recovery');

  const requestsBeforeRevisit = traffic.requests.length;
  const purgesBeforeRevisit = await page.evaluate(() => window.__portalPlotPurges.length);
  await portalStep(page, 'rapid_recovery');
  await page.waitForFunction(() => window.__portalPlotCalls.filter(call => call.resultKind === 'rapid-recovery').length >= 10);
  await portalStep(page, 'scenario_selection');
  await page.waitForFunction(() => window.__portalPlotCalls.filter(call => call.resultKind === 'scenario-generation').length >= 4);
  check(traffic.requests.length === requestsBeforeRevisit,
    'resilience workflow: revisiting rendered steps created an HTTP request');
  const purgeState = await page.evaluate(before => ({
    current: window.__portalPlotPurges.length,
    allElements: window.__portalPlotPurges.slice(before).every(call => call.targetIsElement),
  }), purgesBeforeRevisit);
  check(purgeState.current > purgesBeforeRevisit && purgeState.allElements,
    `resilience workflow: portal plots were not purged before rerender ${JSON.stringify(purgeState)}`);

  await page.locator('[data-portal-field="matpower-case"]').selectOption('case14.m');
  await page.getByRole('button', { name: '加载 MATPOWER' }).click();
  await page.waitForFunction(() => document.querySelector('.resilience-portal__model-state')?.textContent?.includes('case14.m'));
  check(recorded(traffic, '/api/session/load_matpower', 'POST').length === 1,
    'resilience workflow: MATPOWER load request missing');

  const json = JSON.stringify({ ...MOCK_SYSTEM, name: 'Imported resilience JSON' });
  await page.locator('[data-portal-field="json-import"]').setInputFiles({ name: 'resilience.json', mimeType: 'application/json', buffer: Buffer.from(json) });
  await page.waitForFunction(() => document.querySelector('.resilience-portal__model-state')?.textContent?.includes('Imported resilience JSON'));
  check(recorded(traffic, '/api/session/load_json_string', 'POST').length >= 2,
    'resilience workflow: JSON import request missing');
  await exerciseWorkspace(page, traffic);
}

async function exerciseWorkspace(page, traffic) {
  await page.waitForFunction(() => {
    const w = document.getElementById('resiliencePortalRoot').__resiliencePortal.workspace;
    return !w.busy && !w.saving && w.versions.length >= 2;
  });
  await page.getByRole('tab', { name: '我的分析', exact: true }).click();
  check(await page.locator('#resiliencePortal-projects .rp-version').count() >= 2, 'workspace: metric outputs were not saved');
  const exported = page.waitForEvent('download');
  await page.locator('#resiliencePortal-projects').getByRole('button', { name: '导出分析文件' }).first().click();
  const content = JSON.parse(await readFile(await (await exported).path(), 'utf8'));
  check(content.schema === 'resilience_workspace_v1' && content.model.ac.buses.length > 0 && content.state.metrics.result.results.length === 24,
    'workspace: model or result missing from exported version');
  const counts = { recovery: recorded(traffic, '/api/session/run_distribution_resilience', 'POST').length, metrics: recorded(traffic, '/api/session/resilience/metrics', 'POST').length };
  await page.reload({ waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => document.body.dataset.startupState === 'ready');
  await page.waitForFunction(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.workspace.versions.length >= 2);
  check(await page.locator('#resiliencePortal-projects').isVisible(), 'workspace: safe route did not survive reload');
  await page.locator('#resiliencePortal-projects').getByRole('button', { name: '继续此版本' }).first().click();
  await page.waitForFunction(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    return !p.workspace.busy && p.state.model.status === 'ready' && p.state.scenarioGeneration.selectedScenario;
  });
  const restored = await page.evaluate(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    return { run: p.state.recovery.artifact, metrics: p.state.metrics.result, ids: p.state.selectedMetricIds.length,
      faults: p.effectiveRecoveryConfig().ac_fault_branch_ids_text,
      metricParameters: p.state.metricParameters, approximationConsents: p.state.approximationConsents };
  });
  check(restored.run === null && restored.metrics === null && restored.ids === 24 && restored.faults === '25', 'workspace: restored version reused a stale run or lost parameters');
  check(Object.keys(restored.metricParameters).length === 0 &&
    restored.approximationConsents.apda === false && restored.approximationConsents.res === false,
    'workspace: legacy metric calculation settings survived restore');
  check(recorded(traffic, '/api/session/run_distribution_resilience', 'POST').length === counts.recovery && recorded(traffic, '/api/session/resilience/metrics', 'POST').length === counts.metrics,
    'workspace: reopening a version triggered a calculation');
  await portalStep(page, 'scenario_selection');
  await page.getByRole('button', { name: '生成弹性候选场景' }).click();
  await page.waitForFunction(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.scenarioGeneration.status === 'ready');
  await portalStep(page, 'proactive_defense');
  await page.locator('[data-portal-planning-run]').click();
  await page.waitForFunction(() => document.getElementById('resiliencePortalRoot').__resiliencePortal.state.proactiveDefense.status === 'success');
  await portalStep(page, 'rapid_recovery');
  await page.locator('[data-portal-scenario-select="resilience:SuperTY:A"]').click();
  await page.getByRole('button', { name: '运行快速恢复', exact: true }).click();
  await page.waitForFunction(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    return !p.workspace.busy && p.state.recovery.status === 'success';
  });
  await portalStep(page, 'metric_output');
  await page.getByRole('button', { name: '计算并输出指标', exact: true }).click();
  await page.waitForFunction(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    return !p.workspace.busy && !p.workspace.saving && p.state.metrics.status === 'success';
  });
  await page.getByRole('tab', { name: '结果对比', exact: true }).click();
  await page.locator('.rp-compare-picker input').nth(0).check();
  await page.locator('.rp-compare-picker input').nth(1).check();
  check(await page.locator('[data-workspace-comparison-chart]').count() === 1, 'workspace: comparison recovery chart missing');
  check(await page.locator('#resiliencePortal-compare table tr').count() === 25, 'workspace: comparison metric rows missing');
  const boundary = await page.evaluate(() => {
    const w = document.getElementById('resiliencePortalRoot').__resiliencePortal.workspace;
    const a = w.versions[0], b = structuredClone(a);
    const same = w.comparisonReasons([a, b]); b.state.recovery.artifact.time_axis = [0, 99];
    const different = w.comparisonReasons([a, b]);
    return { same, different };
  });
  check(boundary.same.length === 0 && boundary.different.includes('时间轴不同'), 'workspace: comparison boundary not enforced');
  await assertNoOverflow(page, 'workspace comparison');
  await page.getByRole('tab', { name: '我的分析', exact: true }).click();
  const versionsBeforeImport = await page.locator('#resiliencePortal-projects .rp-version').count();
  await page.locator('#resiliencePortal-projects input[type=file]').setInputFiles({ name: 'broken.json', mimeType: 'application/json', buffer: Buffer.from('{"schema":"resilience_workspace_v1"}') });
  await page.waitForFunction(() => document.getElementById('resiliencePortalStatus').textContent.includes('导入失败'));
  check(await page.locator('#resiliencePortal-projects .rp-version').count() === versionsBeforeImport, 'workspace: malformed import changed saved versions');
  await page.locator('#resiliencePortal-projects input[type=file]').setInputFiles({ name: 'analysis.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(content)) });
  await page.waitForFunction(count => document.querySelectorAll('#resiliencePortal-projects .rp-version').length === count + 1, versionsBeforeImport);
  check(await page.evaluate(() => {
    const w = document.getElementById('resiliencePortalRoot').__resiliencePortal.workspace;
    return w.versions[0].imported === true && w.comparisonReasons([w.versions[0], w.versions[1]]).includes('含导入的结果，未在本机验证来源');
  }), 'workspace: imported result provenance was lost');
  await page.getByRole('tab', { name: '计算任务', exact: true }).click();
  check(await page.locator('#resiliencePortal-tasks .rp-version').count() >= 5, 'workspace: durable task history missing');
  await page.getByRole('tab', { name: '帮助', exact: true }).click();
  check(await page.locator('#resiliencePortal-help .rp-help-grid article').count() === 4, 'workspace: quick start missing');
  await page.getByRole('tab', { name: '首页', exact: true }).click();
  await page.getByRole('button', { name: '体验 33 节点演示', exact: true }).click();
  await page.waitForFunction(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    return p.workspace.demo && !p.workspace.busy && p.state.model.status === 'ready';
  });
  check(await page.evaluate(() => {
    const p = document.getElementById('resiliencePortalRoot').__resiliencePortal;
    return p.state.selectedMetricIds.length === 24 && p.state.scenarioConfig.resilience_cluster_count === 1 && p.state.scenarioConfig.intensity_levels.join() === 'TD';
  }), 'workspace: demonstration did not prepare recommended metrics and lightweight scenario');
  check(recorded(traffic, '/api/session/run_distribution_resilience', 'POST').length === counts.recovery + 1,
    'workspace: demonstration started an unsolicited solve');
  await page.setViewportSize({ width: 390, height: 844 });
  for (const name of ['首页', '我的分析', '计算任务', '结果对比', '帮助']) {
    await page.getByRole('tab', { name, exact: true }).click(); await assertNoOverflow(page, `workspace-mobile-${name}`);
  }
  await page.setViewportSize({ width: 1440, height: 1000 });
}

async function runScenarioError(page, base) {
  const scenario = { name: 'scenario-error', mode: 'valid', profile: profile('resilience'), scenarioError: 'No system loaded' };
  await installMocks(page, scenario);
  await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => document.body.dataset.startupState === 'ready');
  await portalStep(page, 'metric_selection');
  await page.locator('.resilience-portal__metric-card input[type="checkbox"]').first().check();
  await portalStep(page, 'scenario_selection');
  await page.getByRole('button', { name: '加载内置算例' }).click();
  await page.waitForFunction(() => !document.querySelector('[data-portal-field="builtin-case"]')?.disabled);
  await page.getByRole('button', { name: '生成弹性候选场景' }).click();
  await page.waitForFunction(() => document.getElementById('resiliencePortalStatus')?.textContent?.includes('场景生成失败：No system loaded'));
  check(await page.getByText('No system loaded', { exact: true }).count() >= 1,
    'resilience workflow: backend scenario error was hidden behind a generic message');
}

async function runValid(page, base, edition, viewport, { delayed = false, hash = '', exerciseWorkflow = false } = {}) {
  const requests = [];
  page.on('request', request => requests.push(request.url()));
  const value = profile(edition);
  const scenario = { name: `${edition}-${delayed ? 'delayed' : 'valid'}`, mode: delayed ? 'delayed' : 'valid', profile: value };
  await installMocks(page, scenario);
  const navigation = page.goto(`${base}/xjtu/${hash}`, { waitUntil: 'domcontentloaded' });
  if (delayed) {
    await page.waitForSelector('body[data-startup-state="loading"]');
    const loading = await shellState(page);
    check(loading.shellHidden && loading.inert && loading.ariaHidden === 'true', `${edition}: delayed profile did not lock shell`);
    check(apiPaths(requests).every(pathname => pathname === '/api/edition'), `${edition}: pre-profile API request ${apiPaths(requests)}`);
  }
  await navigation;
  if (edition === 'resilience') {
    await page.waitForFunction(() => ['ready', 'unavailable'].includes(document.body.dataset.startupState), null, { timeout: 30000 });
    const startup = await page.evaluate(() => ({ state: document.body.dataset.startupState, detail: document.getElementById('startupStatusDetail')?.textContent || '' }));
    check(startup.state === 'ready', `${edition}: startup failed: ${startup.detail}`);
  } else {
    await page.waitForFunction(() => document.body.dataset.startupState === 'ready');
  }
  const state = await shellState(page);
    check(edition === 'resilience'
      ? state.shellHidden && state.inert && state.ariaHidden === 'true'
      : !state.shellHidden && !state.inert && state.ariaHidden === null,
      `${edition}: startup surface accessibility state is incorrect`);
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
    if (edition === 'trial') {
      check(await page.locator('#canvasContainer').evaluate(element => element.parentElement?.id === 'main'),
        'trial: legacy Canvas was reparented outside the legacy shell');
    }
    check(!apiPaths(requests).some(pathname => pathname.includes('/api/dynamics/')), `${edition}: dynamics requested`);
    check(await page.locator('#moduleTransient').count() === 0, `${edition}: transient module remains`);
    check(await page.locator('[data-module^="market"], [data-module="rpo"], [data-module="harmonics"]').count() === 0,
      `${edition}: restricted module remains`);
    check(await page.locator('.module-btn.active').getAttribute('data-module') === 'modelIO', `${edition}: unsafe default/hash module`);
    check(page.url().endsWith(edition === 'resilience' ? '/xjtu/#resilience/home' : '/xjtu/'), `${edition}: forbidden hash was restored`);
  }
  if (edition === 'resilience') {
    check(await page.locator('#resiliencePortalRoot').count() === 1 &&
      await page.locator('#resiliencePortalRoot').isVisible(),
      'resilience: isolated portal is not visible');
    check(await page.locator('#appShell').evaluate(el => el.hidden && el.inert && el.getAttribute('aria-hidden') === 'true'),
      'resilience: legacy shell is not inert/hidden');
    const steps = await page.locator('#resiliencePortalSteps [role="tab"]').evaluateAll(nodes =>
      nodes.map(node => node.dataset.resilienceStep));
    check(JSON.stringify(steps) === JSON.stringify([
      'metric_selection', 'scenario_selection', 'proactive_defense',
      'rapid_recovery', 'metric_output',
    ]), 'resilience: portal step order mismatch');
    check(await page.locator('#resiliencePortalPanel').count() === 1 &&
      await page.locator('#resiliencePortalEvidence').count() === 1,
      'resilience: portal workspace regions missing');
    check(await page.locator('#resiliencePortalSteps [role="tab"]').count() === 5 &&
      await page.locator('[data-module="carbonFlow"], [data-module="harmonics"], [data-module="market"]').count() === 0,
      'resilience: restricted legacy modules remain exposed');
    const duplicateIds = await page.evaluate(() => {
      const seen = new Set();
      return [...document.querySelectorAll('[id]')]
        .map(element => element.id)
        .filter(id => seen.has(id) || !seen.add(id));
    });
    check(duplicateIds.length === 0,
      `resilience: duplicate DOM IDs ${JSON.stringify(duplicateIds)}`);
    const architectureState = await page.evaluate(() => ({
      canvasContainers: document.querySelectorAll('#canvasContainer').length,
      canvases: document.querySelectorAll('#canvas').length,
      overviewCanvases: document.querySelectorAll('#networkOverviewCanvas').length,
      parentId: document.getElementById('canvasContainer')?.parentElement?.id,
      canvasInert: document.getElementById('canvasContainer')?.inert,
      workflowSelected: document.getElementById('resiliencePortalWorkflowTab')?.getAttribute('aria-selected'),
      architectureHidden: document.getElementById('resiliencePortalArchitectureSurface')?.hidden,
    }));
    check(architectureState.canvasContainers === 1 && architectureState.canvases === 1 && architectureState.overviewCanvases === 1 &&
      architectureState.parentId === 'resiliencePortalArchitectureHost',
      `resilience: Canvas single-instance ownership failed ${JSON.stringify(architectureState)}`);
    check(architectureState.canvasInert === true && architectureState.workflowSelected === 'false' && architectureState.architectureHidden === true,
      `resilience: initial presentation state failed ${JSON.stringify(architectureState)}`);
    check(await page.locator('#resiliencePortal-home').isVisible(), 'resilience: home is not the default view');
    check(await page.getByRole('button', { name: '体验 33 节点演示', exact: true }).isVisible(), 'resilience: guided demonstration entry missing');
    await page.getByRole('button', { name: '高级模式', exact: true }).click();
    const architectureRequests = scenario.traffic.requests.length;
    await page.locator('#resiliencePortalWorkflowTab').focus();
    await page.evaluate(() => document.getElementById('resiliencePortalWorkflowTab')?.dispatchEvent(
      new KeyboardEvent('keydown', { key: 'ArrowRight', bubbles: true })));
    await page.waitForFunction(() => document.getElementById('resiliencePortalArchitectureTab')?.getAttribute('aria-selected') === 'true');
    check(await page.locator('#resiliencePortalArchitectureTab').getAttribute('aria-selected') === 'true',
      'resilience: presentation tab keyboard activation failed');
    await page.evaluate(() => document.getElementById('resiliencePortalArchitectureTab')?.dispatchEvent(
      new KeyboardEvent('keydown', { key: 'ArrowLeft', bubbles: true })));
    await page.waitForFunction(() => document.getElementById('resiliencePortalWorkflowTab')?.getAttribute('aria-selected') === 'true');
    check(await page.getByRole('tab', { name: '工作流' }).getAttribute('aria-selected') === 'true',
      'resilience: presentation tab keyboard return failed');
    await page.getByRole('tab', { name: '配电系统架构' }).click();
    const visibleArchitecture = await page.evaluate(() => {
      const surface = document.getElementById('resiliencePortalArchitectureSurface');
      const canvas = document.getElementById('canvasContainer');
      const style = canvas ? getComputedStyle(canvas) : null;
      const rect = canvas?.getBoundingClientRect();
      return { surfaceHidden: surface?.hidden, surfaceInert: surface?.inert, canvasInert: canvas?.inert, display: style?.display, width: rect?.width, height: rect?.height };
    });
    check(!visibleArchitecture.surfaceHidden && !visibleArchitecture.surfaceInert && !visibleArchitecture.canvasInert &&
      visibleArchitecture.display !== 'none' && visibleArchitecture.width > 0 && visibleArchitecture.height > 0,
      `resilience: architecture surface did not expose the existing Canvas ${JSON.stringify(visibleArchitecture)}`);
    check(scenario.traffic.requests.length === architectureRequests,
      'resilience: architecture activation created an HTTP request');
    await assertNoOverflow(page, `resilience-architecture/${viewport.width}`);
    await page.getByRole('tab', { name: '工作流' }).click();
    check(await page.locator('#resiliencePortalWorkflowSurface').isVisible() && await page.locator('#canvasContainer').evaluate(element => element.inert),
      'resilience: workflow return did not deactivate Canvas interaction');
    check(scenario.traffic.requests.length === architectureRequests,
      'resilience: architecture round trip created an HTTP request');
    if (exerciseWorkflow) await exerciseResilienceWorkflow(page, scenario);
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
        page.on("pageerror", error => console.error("Browser error:", error.message));
        try {
          await runValid(page, base, edition, viewport, {
            delayed: edition === 'resilience',
            hash: edition === 'full' ? '' : '#market-realtime',
            exerciseWorkflow: edition === 'resilience' && viewport.width === 1440,
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
        page.on("pageerror", error => console.error("Browser error:", error.message));
        try { await runFailure(page, base, scenario, viewport); }
        finally { await page.close(); }
      }
      if (viewport.width === 1440) {
        const page = await browser.newPage({ viewport });
        page.on("pageerror", error => console.error("Browser error:", error.message));
        try { await runScenarioError(page, base); }
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
