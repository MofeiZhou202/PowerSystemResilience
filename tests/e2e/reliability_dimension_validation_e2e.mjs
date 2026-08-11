// @ts-check

import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';

const ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..', '..');
const INTELLIGENT_FACTORS = {
  detection_success_probability: 0.8,
  isolation_success_probability: 0.9,
  restoration_decision_valid_probability: 0.95,
  restoration_execution_success_probability: 0.96,
  protection_success_probability: 0.99,
};
// Production formulas: src/reliability/reliability_assessment.cpp; derivation and
// Level-1 independence boundary: docs/reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md.
const INTELLIGENT_SUCCESS = Object.values(INTELLIGENT_FACTORS)
  .reduce((product, value) => product * value, 1);
const JOINT_SUCCESS = 0.9 * INTELLIGENT_SUCCESS;

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

async function post(base, endpoint, body) {
  const response = await fetch(`${base}${endpoint}`, {
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

function metric(data, key) {
  const value = data?.metrics?.[key] ?? data?.[key];
  return Number(value && typeof value === 'object' ? value.value : value);
}

function close(actual, expected, label, relativeTolerance = 1e-8) {
  const tolerance = relativeTolerance * Math.max(1, Math.abs(expected));
  assert(Number.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
    `${label}: expected ${expected}, got ${actual}, tolerance ${tolerance}`);
}

function fmeaRequest(dataPolicy, {
  loadScale = 1,
  resources = true,
  informationAvailability = null,
  intelligent = false,
} = {}) {
  const informationEnabled = informationAvailability !== null;
  return {
    method: 'fmea',
    physical_model: 'auto',
    data_policy: dataPolicy,
    load: { hours_per_year: 8760 },
    execution: { parallel: false, parallel_threads: 0 },
    restoration: { max_switch_actions: 2, max_physical_evaluations: 80 },
    dimensions: {
      physical: {
        load_scale_factor: loadScale,
        switching_time_hr: 0.5,
        enable_switch_reconfiguration: resources,
        enable_repair_reconfiguration: resources,
        enable_microgrid_islanding: resources,
        enable_storage_dispatch: resources,
        enable_grid_forming_vsc_support: resources,
        enable_black_start_storage: resources,
      },
      information: {
        enabled: informationEnabled,
        service_availability: informationEnabled ? informationAvailability : 1,
        automatic_switching_time_hr: 0.05,
        manual_switching_time_hr: 1.0,
        freeze_der_on_service_loss: true,
      },
      intelligent: intelligent ? {
        enabled: true,
        independent_factorization: true,
        ...INTELLIGENT_FACTORS,
      } : { enabled: false },
    },
  };
}

async function evaluateCase(base, testCase) {
  await post(base, '/api/session/load_builtin', { case: testCase.name });

  const physicalBase = await post(base, '/api/session/run_reliability',
    fmeaRequest(testCase.dataPolicy));
  const physicalStressed = await post(base, '/api/session/run_reliability',
    fmeaRequest(testCase.dataPolicy, { loadScale: 1.3 }));
  const physicalNoResources = await post(base, '/api/session/run_reliability',
    fmeaRequest(testCase.dataPolicy, { resources: false }));

  const automatic = await post(base, '/api/session/run_reliability',
    fmeaRequest(testCase.dataPolicy, { informationAvailability: 1 }));
  const manual = await post(base, '/api/session/run_reliability',
    fmeaRequest(testCase.dataPolicy, { informationAvailability: 0 }));
  const informationOnly = await post(base, '/api/session/run_reliability',
    fmeaRequest(testCase.dataPolicy, { informationAvailability: 0.9 }));
  const joint = await post(base, '/api/session/run_reliability',
    fmeaRequest(testCase.dataPolicy, {
      informationAvailability: 0.9,
      intelligent: true,
    }));

  const physicalEens = metric(physicalBase, 'eens_mwh_yr');
  const stressedEens = metric(physicalStressed, 'eens_mwh_yr');
  const noResourceEens = metric(physicalNoResources, 'eens_mwh_yr');
  const automaticEens = metric(automatic, 'eens_mwh_yr');
  const manualEens = metric(manual, 'eens_mwh_yr');
  const informationEens = metric(informationOnly, 'eens_mwh_yr');
  const jointEens = metric(joint, 'eens_mwh_yr');
  const scale = Math.max(1, Math.abs(physicalEens));

  assert(stressedEens + 1e-8 * scale >= physicalEens,
    `${testCase.name}: higher load improved physical EENS: ${physicalEens} -> ${stressedEens}`);
  assert(noResourceEens + 1e-8 * scale >= physicalEens,
    `${testCase.name}: disabling restoration resources improved EENS: ` +
    `${physicalEens} -> ${noResourceEens}`);
  assert(manualEens + 1e-10 >= automaticEens,
    `${testCase.name}: automation loss improved EENS: ${automaticEens} -> ${manualEens}`);

  close(informationEens, 0.9 * automaticEens + 0.1 * manualEens,
    `${testCase.name}: information-only affine EENS`);
  close(jointEens, JOINT_SUCCESS * automaticEens + (1 - JOINT_SUCCESS) * manualEens,
    `${testCase.name}: joint affine EENS`);
  assert(jointEens + 1e-10 >= informationEens,
    `${testCase.name}: lower intelligent success improved EENS: ` +
    `${informationEens} -> ${jointEens}`);
  assert(informationEens + 1e-10 >= automaticEens &&
      jointEens <= manualEens + 1e-10,
    `${testCase.name}: a conditioned EENS escaped the automatic/manual endpoints`);

  const cyber = joint.cyber_physical || {};
  close(Number(cyber.intelligent_function_success_probability), INTELLIGENT_SUCCESS,
    `${testCase.name}: intelligent probability`, 1e-12);
  close(Number(cyber.effective_automation_probability), JOINT_SUCCESS,
    `${testCase.name}: joint probability`, 1e-12);
  close(Number(cyber.eens_perfect_cyber_mwh_yr), automaticEens,
    `${testCase.name}: automatic endpoint`);
  close(Number(cyber.eens_no_automation_mwh_yr), manualEens,
    `${testCase.name}: manual endpoint`);
  close(jointEens,
    automaticEens + Number(cyber.delta_cyber_duration_mwh_yr) +
      Number(cyber.delta_cyber_control_mwh_yr),
    `${testCase.name}: duration/control decomposition`);
  const durationIncrement = Number(cyber.delta_cyber_duration_mwh_yr);
  const controlIncrement = Number(cyber.delta_cyber_control_mwh_yr);
  assert(Number.isFinite(durationIncrement) && Number.isFinite(controlIncrement),
    `${testCase.name}: cyber attribution is not finite`);
  // Section 12.3 of the derivation explicitly treats these as signed
  // counterfactual attributions. A negative term is a retained monotonicity
  // diagnostic, not an error or a value to clamp.
  const signedAttributionDiagnostics = (joint.contingencies || [])
    .filter(row => Number(row.eens_cyber_duration_increment) < -1e-10 ||
      Number(row.eens_cyber_control_increment) < -1e-10)
    .map(row => ({
      component_type: row.component_type,
      component_index: row.component_index,
      display_name: row.display_name,
      duration_increment_mwh_yr: Number(row.eens_cyber_duration_increment),
      control_increment_mwh_yr: Number(row.eens_cyber_control_increment),
      shed_rep_automatic_mw: Number(row.shed_rep_automatic_mw),
      shed_rep_manual_mw: Number(row.shed_rep_manual_mw),
    }));
  if (durationIncrement < -1e-10 || controlIncrement < -1e-10) {
    assert(signedAttributionDiagnostics.length > 0,
      `${testCase.name}: negative system attribution has no component diagnostic`);
  }

  assert(joint.dimension_audit?.physical?.applied === true &&
      joint.dimension_audit?.information?.applied === true &&
      joint.dimension_audit?.intelligent?.applied === true,
    `${testCase.name}: three-dimensional request audit is incomplete`);
  assert(joint.validity?.information_service_conditioned === true &&
      joint.validity?.intelligent_function_probabilities_modelled === true &&
      joint.validity?.joint_class_probability_modelled === false &&
      joint.validity?.protection_frt_reliability_coupled === false,
    `${testCase.name}: three-dimensional validity boundary is incorrect`);
  assert(typeof joint.model_scope === 'string' && joint.model_scope.includes('acdc') &&
      String(joint.model_limitations || '').includes('Level 1'),
    `${testCase.name}: hybrid model scope or Level-1 limitation is missing`);

  return {
    case: testCase.name,
    data_policy: joint.data_policy,
    model_scope: joint.model_scope,
    physical_eens_mwh_yr: physicalEens,
    load_1_3_eens_mwh_yr: stressedEens,
    no_resource_eens_mwh_yr: noResourceEens,
    automatic_eens_mwh_yr: automaticEens,
    manual_eens_mwh_yr: manualEens,
    information_0_9_eens_mwh_yr: informationEens,
    joint_eens_mwh_yr: jointEens,
    intelligent_success_probability: INTELLIGENT_SUCCESS,
    effective_automation_probability: JOINT_SUCCESS,
    duration_increment_mwh_yr: durationIncrement,
    control_increment_mwh_yr: controlIncrement,
    attribution_monotone: durationIncrement >= -1e-10 && controlIncrement >= -1e-10,
    signed_attribution_diagnostics: signedAttributionDiagnostics,
  };
}

async function main() {
  const port = Number(arg('port')) || Number(await freePort());
  const base = `http://127.0.0.1:${port}`;
  const proc = spawn(serverPath(), ['--host', '127.0.0.1', '--port', String(port),
    '--data-dir', path.join(ROOT, 'data')], { cwd: ROOT, stdio: 'ignore' });
  try {
    await waitUp(base);
    const results = [];
    for (const testCase of [
      { name: 'dist33_microgrid_der', dataPolicy: 'case_data_only' },
      { name: 'comprehensive_hybrid_acdc', dataPolicy: 'missing_only' },
    ]) {
      results.push(await evaluateCase(base, testCase));
    }

    assert(results.some(result =>
      result.load_1_3_eens_mwh_yr > result.physical_eens_mwh_yr + 1e-8),
    'no case showed a strict physical load impact');
    assert(results.some(result =>
      result.no_resource_eens_mwh_yr > result.physical_eens_mwh_yr + 1e-8),
    'no case showed a strict restoration-resource impact');
    assert(results.some(result =>
      result.manual_eens_mwh_yr > result.automatic_eens_mwh_yr + 1e-8 &&
      result.joint_eens_mwh_yr > result.information_0_9_eens_mwh_yr + 1e-8),
    'no case showed strict digital and intelligent reliability impacts');

    console.log(JSON.stringify({
      theory: {
        intelligent_success_probability: INTELLIGENT_SUCCESS,
        effective_automation_probability: JOINT_SUCCESS,
        eens_identity: 'p_eff*E_auto+(1-p_eff)*E_manual',
      },
      cases: results,
    }));
  } finally {
    proc.kill();
  }
}

main().catch(error => {
  console.error(error.stack || error.message || error);
  process.exitCode = 1;
});
