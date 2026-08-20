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

function protectionCyberComparisonRequest(informationAvailability = 0.8) {
  const trajectory = [{ time_s: 0, current: 2 }, { time_s: 1, current: 2 }];
  return {
    method: 'protection_cyber_compare',
    protection_cyber: {
      scenarios: [{
        scenario_id: 'e2e-protection-cyber-oracle',
        initiating_frequency_per_year: 2,
        coordination_margin_s: 0.3,
        uncleared_terminal_time_s: 2,
        primary: {
          relay: { relay_id: 'primary', characteristic: 'definite_time_overcurrent',
            pickup_current: 1, definite_time_delay_s: 0.1 },
          trajectory, breaker: {}, relay_success_probability: 0.9,
          breaker_success_probability: 1,
        },
        backup: {
          relay: { relay_id: 'backup', characteristic: 'definite_time_overcurrent',
            pickup_current: 1, definite_time_delay_s: 0.5 },
          trajectory, breaker: {}, relay_success_probability: 1,
          breaker_success_probability: 1,
        },
        information_components: [{ id: 'shared-center',
          intrinsic_availability: informationAvailability,
          packet_delivery_probability: 1 }],
        information_functions: [{ name: 'shared-control', alternative_paths: [[0]] }],
        function_bindings: { detection_function: 0, isolation_function: 0,
          restoration_function: 0 },
        consequence: {
          primary_clearing_shed_mw: 10, backup_clearing_shed_mw: 10,
          uncleared_shed_mw: 10, isolated_shed_mw: 2,
          isolation_failed_shed_mw: 8, restored_shed_mw: 0,
          restoration_failed_shed_mw: 5, automatic_restoration_hr: 0.1,
          manual_restoration_hr: 1, repair_hr: 10,
        },
      }],
    },
  };
}

async function main() {
  const { chromium } = await import('playwright');
  const port = Number(arg('port')) || Number(await freePort());
  const base = `http://127.0.0.1:${port}`;
  const proc = spawn(serverPath(), ['--host', '127.0.0.1', '--port', String(port),
    '--data-dir', path.join(ROOT, 'data')], {
    cwd: ROOT,
    stdio: process.env.HYSIM_E2E_SERVER_LOG ? 'inherit' : 'ignore',
  });
  let browser;
  try {
    await waitUp(base);
    const loadedReliabilityDemo = await request(base, '/api/session/load_builtin', {
      case: 'cyber_physical_reliability_demo',
    });

    const onlineSystem = JSON.parse(loadedReliabilityDemo._raw_json);
    onlineSystem.ac.external_grids = [];
    onlineSystem.ac.generators = [{
      index: 1, bus: 1, in_service: true, is_slack: true,
      name: 'Online DAE slack source', vg_pu: 1.02, pg_mw: 2,
      pmin_mw: 0, pmax_mw: 10, qmin_mvar: -10, qmax_mvar: 10,
      xdpp_pu: 0.2,
    }];
    await request(base, '/api/session/load_json_string', {
      json_string: JSON.stringify(onlineSystem),
    });
    const onlineRequest = protectionCyberComparisonRequest();
    onlineRequest.protection_cyber.online_dae = true;
    const onlineScenario = onlineRequest.protection_cyber.scenarios[0];
    onlineScenario.initiating_frequency_per_year = 1;
    onlineScenario.coordination_margin_s = 0.02;
    onlineScenario.uncleared_terminal_time_s = 0.2;
    onlineScenario.primary.relay.pickup_current = 0.2;
    onlineScenario.primary.relay.definite_time_delay_s = 0.02;
    onlineScenario.primary.breaker.mechanical_delay_s = 0.01;
    onlineScenario.backup.relay.pickup_current = 0.2;
    onlineScenario.backup.relay.definite_time_delay_s = 0.05;
    onlineScenario.backup.breaker.mechanical_delay_s = 0.01;
    onlineScenario.online_dae = {
      fault_ac_bus_id: 2, primary_branch_index: 1, backup_branch_index: 2,
      fault_time_s: 0.05, fault_r_pu: 0.2, fault_x_pu: 0,
      dt_s: 0.01, t_end_s: 0.2,
    };
    const onlineComparison = await request(base, '/api/session/run_reliability',
      onlineRequest);
    const onlineDiagnostic = onlineComparison.online_diagnostics?.[0];
    assert(onlineComparison.validity?.online_network_dae_coupled === true &&
      onlineComparison.dae_trajectories_consumed_by_event_tree === true &&
      Number(onlineDiagnostic?.primary_relay_trajectory_points) > 0 &&
      Number(onlineDiagnostic?.backup_relay_trajectory_points) > 0 &&
      onlineDiagnostic?.primary_protection_action_applied === true &&
      onlineDiagnostic?.backup_protection_action_applied === true &&
      Math.abs(Number(onlineDiagnostic?.primary_clear_time_s) -
        Number(onlineDiagnostic?.primary_event_tree_clear_time_s)) <= 0.010000001 &&
      Math.abs(Number(onlineDiagnostic?.backup_clear_time_s) -
        Number(onlineDiagnostic?.backup_event_tree_clear_time_s)) <= 0.010000001,
    'online DAE trajectories were not consumed consistently by the annual event tree');

    await request(base, '/api/session/load_builtin', {
      case: 'cyber_physical_reliability_demo',
    });

    const physicalCutSet = await request(base, '/api/session/run_reliability', {
      method: 'physical_cut_set',
      failure_rate_basis: 'calendar_time',
      physical_cut_set: {
        components: [
          { stable_id: 'ac_branch:10', availability: 0.9 },
          { stable_id: 'ac_branch:20', availability: 0.8 },
          { stable_id: 'ac_branch:30', availability: 0.7 },
        ],
        success_paths: [[0, 1], [0, 2]],
      },
    });
    assert(Math.abs(Number(physicalCutSet.availability) - 0.846) <= 1e-12 &&
      Math.abs(Number(physicalCutSet.loss_probability) - 0.154) <= 1e-12 &&
      physicalCutSet.failure_rate_basis === 'calendar_time' &&
      physicalCutSet.exact_independent_path_model === true &&
      Number(physicalCutSet.reduced_success_path_count) === 2 &&
      (physicalCutSet.minimal_cut_sets || []).some(cut =>
        JSON.stringify(cut) === JSON.stringify(['ac_branch:10'])) &&
      (physicalCutSet.minimal_cut_sets || []).some(cut =>
        JSON.stringify(cut) === JSON.stringify(['ac_branch:20', 'ac_branch:30'])),
    'physical cut-set HTTP result does not match the closed-form shared-path benchmark');

    const invalidBasisResponse = await fetch(`${base}/api/session/run_reliability`, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify({
        method: 'physical_cut_set',
        failure_rate_basis: 'ambiguous_year',
        physical_cut_set: {
          components: [{ stable_id: 'ac_branch:1', availability: 0.9 }],
          success_paths: [[0]],
        },
      }),
    });
    const invalidBasis = await invalidBasisResponse.json();
    assert(invalidBasisResponse.status === 400 &&
      String(invalidBasis.error || '').includes('failure_rate_basis 只能是'),
    'invalid failure-rate basis was not rejected by the HTTP contract');

    const exactSensitivity = await request(base, '/api/session/run_reliability', {
      method: 'exact_sensitivity',
      data_policy: 'case_data_only',
      exact_sensitivity: { maximum_components: 20 },
    });
    assert(exactSensitivity.validity?.exact_independent_binary_model === true &&
      exactSensitivity.validity?.state_space_fully_enumerated === true &&
      Number(exactSensitivity.states_evaluated) === 4 &&
      exactSensitivity.sensitivity_components?.length === 2,
    'exact sensitivity did not enumerate the two stochastic components');
    assert(exactSensitivity.sensitivity_components.every(row =>
      typeof row.stable_id === 'string' && row.stable_id.includes(':') &&
      Number.isInteger(Number(row.component_index)) &&
      Number.isInteger(Number(row.component_position)) &&
      Number.isFinite(Number(row.birnbaum_mwh_yr_per_unit_unavailability)) &&
      Number.isFinite(Number(row.fussell_vesely))),
    'exact sensitivity rows do not preserve stable identity and finite indices');

    const importanceBase = {
      method: 'nsq',
      data_policy: 'case_data_only',
      execution: { parallel: false },
      monte_carlo: {
        max_iterations: 400, seed: 123, parallel: false,
        use_importance_sampling: true, importance_lambda: 1,
      },
    };
    const importanceIdentity = await request(base, '/api/session/run_reliability',
      importanceBase);
    assert(importanceIdentity.importance_sampling?.used === true &&
      Number(importanceIdentity.importance_sampling?.twisting_factor) === 1 &&
      Math.abs(Number(importanceIdentity.importance_sampling?.mean_likelihood_ratio) - 1) < 1e-12 &&
      Math.abs(Number(importanceIdentity.importance_sampling?.effective_sample_size) - 400) < 1e-9,
    'lambda-one importance sampling did not preserve the ordinary-sampling identity');
    const importanceTwisted = await request(base, '/api/session/run_reliability', {
      ...importanceBase,
      monte_carlo: { ...importanceBase.monte_carlo, importance_lambda: 4 },
    });
    assert(importanceTwisted.importance_sampling?.used === true &&
      Number(importanceTwisted.importance_sampling?.twisting_factor) === 4 &&
      Number(importanceTwisted.importance_sampling?.effective_sample_size) > 0 &&
      Number(importanceTwisted.importance_sampling?.effective_sample_size) <= 400,
    'twisted importance sampling did not return effective likelihood diagnostics');

    const protectionCyber = await request(base, '/api/session/run_reliability',
      protectionCyberComparisonRequest());
    const compared = protectionCyber.method_comparison;
    assert(Math.abs(Number(compared?.static_fmea?.eens_mwh_yr) - 200) < 1e-10 &&
      Math.abs(Number(compared?.protection_only?.eens_mwh_yr) - 0.4007777777777778) < 1e-10 &&
      Math.abs(Number(compared?.cyber_conditioned?.eens_mwh_yr) - 21.52173333333333) < 1e-10 &&
      Math.abs(Number(compared?.static_fmea?.lole_hr_yr) - 20) < 1e-10 &&
      Math.abs(Number(compared?.protection_only?.lole_hr_yr) - 0.20007777777777777) < 1e-10 &&
      Math.abs(Number(compared?.cyber_conditioned?.lole_hr_yr) - 4.160062222222222) < 1e-10,
    'static/protection/cyber comparison does not match the closed-form oracle');
    const degradedProtectionCyber = await request(base, '/api/session/run_reliability',
      protectionCyberComparisonRequest(0.4));
    assert(Number(degradedProtectionCyber.method_comparison?.cyber_conditioned?.eens_mwh_yr) >
      Number(compared.cyber_conditioned.eens_mwh_yr) &&
      Number(degradedProtectionCyber.method_comparison?.static_fmea?.eens_mwh_yr) ===
      Number(compared.static_fmea.eens_mwh_yr),
    'information availability did not affect only the joint-method result');

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
    assert(physicalBase.der_control?.scenario === 'authored' &&
      Number(physicalBase.der_control?.device_count) > 0 &&
      Array.isArray(physicalBase.der_control?.devices),
    'reliability result does not expose the per-device DER control audit');
    const comparison = await request(base, '/api/session/reliability/compare_results', {
      top_k: 5,
      results: [
        {
          comparison_id: 'fmea:authored',
          method: 'fmea',
          model_scope: physicalBase.model_scope,
          comparison_basis: physicalBase.comparison_basis,
          metrics: physicalBase.metrics,
          der_control: physicalBase.der_control,
          contingencies: physicalBase.contingencies,
        },
        {
          comparison_id: 'failure_mode_fmea:authored',
          method: 'failure_mode_fmea',
          model_scope: reliabilityBase.model_scope,
          comparison_basis: reliabilityBase.comparison_basis,
          metrics: reliabilityBase.metrics,
          der_control: reliabilityBase.der_control,
          contingencies: reliabilityBase.contingencies,
        },
      ],
    });
    const eensRobustness = comparison.metric_robustness?.eens_mwh_yr;
    assert(comparison.comparison_basis_verified === true &&
      eensRobustness?.available === true &&
      Number(eensRobustness.method_count) === 2 &&
      Number(eensRobustness.normalized_range) >= 0,
    'same-snapshot EENS robustness summary is incomplete');
    assert((comparison.rank_agreement || []).every(row =>
      (row.spearman === null ||
        (Number(row.spearman) >= -1 && Number(row.spearman) <= 1)) &&
      (row.top_k_jaccard === null ||
        (Number(row.top_k_jaccard) >= 0 && Number(row.top_k_jaccard) <= 1))),
    'rank-agreement statistics are outside mathematical bounds');
    assert((comparison.consensus_weak_components || []).every(row =>
      typeof row.stable_id === 'string' && row.stable_id.includes(':')),
    'consensus weak components are not keyed by stable component identity');
    const dominantMode = (reliabilityBase.contingencies || []).find(row =>
      row.supported !== false && row.activation === 'passive' &&
      Number(row.eens_contribution) > 0 && Number(row.frequency_per_year) > 0);
    assert(dominantMode, 'built-in case has no positive passive failure-mode contribution');
    const doubledFrequency = Number(dominantMode.frequency_per_year) * 2;
    const configurationBeforeStress = await request(
      base, '/api/session/reliability/configuration');
    const effectiveDominantMode = (configurationBeforeStress.effective_modes || [])
      .find(row => row.mode_id === dominantMode.mode_id);
    const dominantRepairHr = Number(effectiveDominantMode?.effective_repair_hr);
    assert(Number.isFinite(dominantRepairHr) && dominantRepairHr > 0 &&
      doubledFrequency * dominantRepairHr < 8760,
    'selected failure mode lacks a valid repair time for calendar-frequency inversion');
    const doubledOperatingIntensity = doubledFrequency /
      (1 - doubledFrequency * dominantRepairHr / 8760);
    await request(base, '/api/session/reliability/configuration', {
      configuration: {
        profile_id: 'workflow_e2e',
        mode_overrides: [{
          mode_id: dominantMode.mode_id,
          enabled: true,
          failure_rate_per_year: doubledOperatingIntensity,
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
    assert(hybridThreeStage.ok === true &&
      hybridThreeStage.model_scope === 'coupled-acdc-lindistflow-restoration-milp' &&
      hybridThreeStage.validity?.restoration_milp_solved === true &&
      hybridThreeStage.validity?.branch_flow_enforced === true &&
      hybridThreeStage.validity?.apparent_power_polygon_enforced === true &&
      hybridThreeStage.validity?.voltage_constraints_enforced === true &&
      hybridThreeStage.validity?.radial_topology_enforced === true &&
      hybridThreeStage.validity?.sop_dispatch_optimised === true &&
      hybridThreeStage.component_coverage?.dc_der === 'static_generator_pv_storage_dispatch' &&
      Number(hybridThreeStage.apparent_power_polygon_sides) === 16 &&
      Number(hybridThreeStage.maximum_ac_branch_apparent_power_ratio) <= 1.0000001 &&
      (hybridThreeStage.faults || []).every(f =>
        Array.isArray(f.vsc_dispatch_kw?.stage1) &&
        Array.isArray(f.vsc_dispatch_kw?.stage2) &&
        Array.isArray(f.vsc_dispatch_kw?.stage3) &&
        [1, 2, 3].every(stage =>
          typeof f[`stage${stage}_solver_status`] === 'string' &&
          f[`stage${stage}_solver_status`].length > 0 &&
          Number.isFinite(Number(f[`stage${stage}_mip_gap`])) &&
          Number.isFinite(Number(f[`stage${stage}_solver_reported_mip_gap`])))) &&
      (hybridThreeStage.faults || []).length > 0,
    'Dist33 DER hybrid three-stage coupled-model contract is incomplete');

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
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    await page.goto(`${base}/xjtu/`, { waitUntil: 'networkidle' });
    await page.evaluate(async () => {
      await App.loadBuiltinCase('cyber_physical_reliability_demo');
      App.setActiveModule('reliability');
    });
    await page.waitForFunction(() =>
      document.getElementById('relWorkflowComponents')?.textContent !== '待同步');
    assert(await page.locator('#relWorkflowStages > li').count() === 5,
      'reliability workflow does not expose five stages');
    assert(await page.locator('#relDerControlScenario').count() === 1 &&
      await page.locator('#relDerBlackStart').count() === 1 &&
      await page.locator('#relPcOnlineDae').count() === 1 &&
      await page.locator('#btnCompareReliability').count() === 1,
    'DER control, online DAE, or reliability-comparison GUI controls are missing');

    const principleChecks = [
      ['nsq', '独立状态抽样'],
      ['seq', '逐时状态演化'],
      ['fmea', '元件 N-1 FMEA'],
      ['failure_mode_fmea', '失效模式 FMEA'],
      ['fd', '发电充裕度 COPT'],
      ['exact_sensitivity', '精确可靠性灵敏度'],
      ['physical_cut_set', '物理网络最小割集'],
      ['protection_cyber_compare', '静态 FMEA、仅保护与信息物理联合事件树对照'],
    ];
    for (const [method, expected] of principleChecks) {
      await page.locator('#relMethod').selectOption(method);
      assert((await page.locator('#relPrincipleMethod').textContent())?.includes(expected),
        `${method} calculation principle did not update`);
    }
    await page.locator('#relMethod').selectOption('exact_sensitivity');
    assert(await page.locator('.rel-exact-sensitivity-control').first().isVisible() &&
      await page.locator('.rel-importance-control').first().isHidden(),
    'exact-sensitivity and importance controls are not method-qualified');
    await page.locator('#relDataPolicy').selectOption('case_data_only');
    await page.locator('#btnRunReliability').click();
    await page.waitForSelector('#reliabilityResults table >> text=Fussell-Vesely');
    assert(await page.locator('#reliabilityResults').getByText('ac_branch:1').count() > 0,
      'exact sensitivity GUI did not render stable component identity');

    await page.locator('#relMethod').selectOption('physical_cut_set');
    await page.locator('#relFailureRateBasis').selectOption('calendar_time');
    assert(await page.locator('#relPhysicalCutSetComponents').isVisible() &&
      await page.locator('#relPhysicalCutSetPaths').isVisible(),
    'physical cut-set GUI inputs are not method-qualified');
    await page.locator('#btnRunReliability').click();
    await page.waitForSelector('#reliabilityResults >> text=稳定 ID 最小割集');
    assert(await page.locator('#reliabilityResults').getByText('ac_branch:10').count() > 0 &&
      (await page.locator('#reliabilityResults').textContent())?.includes('日历年频率'),
    'physical cut-set GUI did not render the stable-ID cuts and selected rate basis');

    await page.locator('#relMethod').selectOption('nsq');
    assert(await page.locator('.rel-importance-control').first().isVisible() &&
      await page.locator('.rel-exact-sensitivity-control').first().isHidden(),
    'NSQ importance controls are not method-qualified');
    await page.locator('#relMaxIter').fill('200');
    await page.locator('#relImportanceSampling').check();
    await page.locator('#relImportanceLambda').fill('4');
    await page.locator('#btnRunReliability').click();
    await page.waitForSelector('#reliabilityResults >> text=重要抽样诊断');

    await page.locator('#relMethod').selectOption('protection_cyber_compare');
    assert(await page.locator('.rel-protection-cyber-control').first().isVisible() &&
      await page.locator('.rel-importance-control').first().isHidden(),
    'protection-cyber controls are not method-qualified');
    await page.locator('#relPcInfoAvailability').fill('0.8');
    await page.locator('#btnRunReliability').click();
    await page.waitForSelector('#reliabilityResults >> text=静态、保护与信息物理联合结果对照');
    assert(await page.locator('#reliabilityResults').getByText('信息系统失效相对仅保护').count() > 0,
      'protection-cyber GUI did not render the method-impact comparison');
    await page.locator('#relPcOnlineDae').check();
    await page.locator('#relPcPrimaryDelay').fill('0.02');
    await page.locator('#relPcBackupDelay').fill('0.05');
    await page.locator('#relPcCoordinationMargin').fill('0.02');
    await page.locator('#btnRunReliability').click();
    await page.waitForSelector('#reliabilityResults >> text=在线 DAE 与年度事件树闭环诊断');
    assert(await page.locator('#reliabilityResults').getByText('主后备均已反馈').count() > 0 &&
      await page.locator('#reliabilityResults').getByText('主后备均已消费').count() > 0,
    'online DAE GUI did not render trajectory consumption and action feedback');
    await page.locator('#relPhysicalModel').selectOption('restoration_milp');
    assert((await page.locator('#relPrincipleMethod').textContent())?.includes('三阶段 AC/DC 联合恢复 MILP'),
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
      const resultPanel = document.getElementById('rightPanel')?.getBoundingClientRect();
      return {
        pageOverflow: document.documentElement.scrollWidth - window.innerWidth,
        workflowRight: rect?.right || 0,
        resultPanelLeft: resultPanel?.left || 0,
        resultPanelRight: resultPanel?.right || 0,
        resultPanelWidth: resultPanel?.width || 0,
        viewport: window.innerWidth,
      };
    });
    assert(layout.pageOverflow <= 1 && layout.workflowRight <= layout.viewport + 1 &&
      layout.resultPanelLeft >= -1 && layout.resultPanelRight <= layout.viewport + 1 &&
      layout.resultPanelWidth <= layout.viewport + 1,
      `mobile workflow overflows horizontally: ${JSON.stringify(layout)}`);

    console.log(JSON.stringify({
      case: 'cyber_physical_reliability_demo',
      physical_eens: [metric(physicalBase, 'eens_mwh_yr'),
        metric(physicalStressed, 'eens_mwh_yr')],
      information_eens: [metric(informationAvailable, 'eens_mwh_yr'),
        metric(informationUnavailable, 'eens_mwh_yr')],
      reliability_eens: [metric(reliabilityBase, 'eens_mwh_yr'),
        metric(reliabilityStressed, 'eens_mwh_yr')],
      comparison_eens_normalized_range: eensRobustness.normalized_range,
      comparison_consensus_count: comparison.consensus_weak_components?.length || 0,
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
