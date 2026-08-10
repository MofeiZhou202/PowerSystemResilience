// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync, readFileSync } from 'node:fs';
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

async function request(base, endpoint, body) {
  const response = await fetch(`${base}${endpoint}`, body === undefined ? {} : {
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await response.json();
  return { response, data };
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

function assert(condition, message) {
  if (!condition) throw new Error(message);
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
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
    const pageErrors = [];
    page.on('pageerror', error => pageErrors.push(String(error?.stack || error)));
    await page.goto(`${base}/xjtu/`, { waitUntil: 'networkidle' });

    const xml = readFileSync(path.join(ROOT, 'data', 'test.xml'), 'utf8');
    const loaded = await request(base, '/api/session/load_cim_dist', { xml_string: xml });
    assert(loaded.response.ok, `CIM load failed: ${JSON.stringify(loaded.data)}`);

    const initial = await request(base, '/api/session/reliability/configuration');
    assert(initial.response.ok, `configuration GET failed: ${JSON.stringify(initial.data)}`);
    const fieldNames = initial.data.schema.mode_fields.map(field => field.name);
    const numberFieldNames = fieldNames.filter(field => field !== 'enabled');
    for (const required of ['failure_rate_per_year', 'mtbf_hours', 'mttr_hours',
      'forced_outage_rate', 'probability_per_demand', 'isolation_hr',
      'switching_hr', 'repair_hr', 'cyber_recovery_hr', 'residual_capacity_factor']) {
      assert(fieldNames.includes(required), `${required} missing from mode schema`);
    }
    assert(initial.data.coverage.components_total > 0, 'empty component coverage');
    assert(initial.data.coverage.modes_total === initial.data.effective_modes.length,
      'coverage mode count does not match effective rows');
    assert(initial.data.coverage.modes_unsupported ===
      initial.data.effective_modes.filter(mode => mode.supported === false).length,
    'coverage unsupported count differs from effective mode support flags');
    for (const mode of initial.data.effective_modes) {
      assert(mode.resolved_parameters && typeof mode.resolved_parameters === 'object',
        `resolved parameters missing for ${mode.mode_id}`);
      for (const field of numberFieldNames) {
        assert(Object.hasOwn(mode.resolved_parameters, field),
          `resolved field ${field} missing for ${mode.mode_id}`);
        assert(Number.isFinite(Number(mode.resolved_parameters[field])),
          `resolved field ${field} is non-finite for ${mode.mode_id}`);
      }
    }

    const transformerMode = initial.data.effective_modes.find(mode =>
      mode.component_kind === 'ac_transformer_2w' && mode.consequence === 'forced_outage');
    const protectiveMode = initial.data.effective_modes.find(mode =>
      ['ac_switch', 'ac_circuit_breaker'].includes(mode.component_kind) &&
      ['fail_to_trip', 'fail_to_open'].includes(mode.consequence));
    const protectedComponent = initial.data.components.find(component =>
      component.component_kind === 'ac_branch');
    const branchMode = initial.data.effective_modes.find(mode =>
      mode.component_kind === 'ac_branch' && mode.consequence === 'forced_outage');
    assert(transformerMode, 'transformer reliability mode missing');
    assert(protectiveMode, 'protective-device mode missing');
    assert(protectedComponent, 'protected branch missing');
    assert(branchMode, 'branch reliability mode missing');

    const configuration = {
      profile_id: 'e2e_custom',
      mode_overrides: [{
        mode_id: transformerMode.mode_id,
        enabled: true,
        failure_rate_per_year: 0.123,
        mtbf_hours: null,
        mttr_hours: null,
        forced_outage_rate: null,
        probability_given_initiated: 0.95,
        demand_frequency_per_year: null,
        probability_per_demand: null,
        isolation_hr: 0.25,
        switching_hr: 0.5,
        repair_hr: 9,
        cyber_recovery_hr: null,
        residual_capacity_factor: 0.8,
      }],
      protection: [{
        protection_id: 'e2e-protection',
        protective_device_id: protectiveMode.stable_id,
        protected_component_id: protectedComponent.stable_id,
        backup_device_id: '',
        zone_component_ids: [protectedComponent.stable_id],
        enabled: true,
        fail_to_trip_probability: 0.012,
        nuisance_trip_frequency_per_year: 1.2,
        fail_to_open_probability: 0.015,
        primary_clearing_time_s: 0.12,
        backup_clearing_time_s: 0.45,
        automatic_reclose: true,
        successful_reclose_probability: 0.82,
      }],
    };
    const saved = await request(base, '/api/session/reliability/configuration', { configuration });
    assert(saved.response.ok, `configuration save failed: ${JSON.stringify(saved.data)}`);
    assert(saved.data.validation.ok, 'saved configuration did not validate');
    const echo = saved.data.configuration.mode_overrides[0];
    for (const [key, value] of Object.entries(configuration.mode_overrides[0])) {
      assert(echo[key] === value, `mode field ${key} did not round-trip`);
    }
    const protectionEcho = saved.data.configuration.protection[0];
    for (const [key, value] of Object.entries(configuration.protection[0])) {
      assert(JSON.stringify(protectionEcho[key]) === JSON.stringify(value),
        `protection field ${key} did not round-trip`);
    }
    const effectiveTransformer = saved.data.effective_modes.find(mode =>
      mode.mode_id === transformerMode.mode_id);
    assert(Math.abs(effectiveTransformer.effective_failure_rate_per_year - 0.123) < 1e-12,
      'custom transformer failure rate is not effective');
    assert(Math.abs(effectiveTransformer.effective_repair_hr - 9) < 1e-12,
      'custom transformer repair duration is not effective');
    const effectiveProtective = saved.data.effective_modes.find(mode =>
      mode.mode_id === protectiveMode.mode_id);
    const expectedProtectionProbability = protectiveMode.consequence === 'fail_to_trip'
      ? configuration.protection[0].fail_to_trip_probability
      : configuration.protection[0].fail_to_open_probability;
    assert(Math.abs(effectiveProtective.probability_per_demand -
      expectedProtectionProbability) < 1e-12,
    'custom protection failure probability is not effective');
    const nuisanceMode = saved.data.effective_modes.find(mode =>
      mode.stable_id === protectiveMode.stable_id && mode.consequence === 'nuisance_trip');
    assert(nuisanceMode && Math.abs(nuisanceMode.effective_failure_rate_per_year - 1.2) < 1e-12,
      'custom nuisance-trip frequency is not effective');

    const unappliedRun = await request(base, '/api/session/run_reliability', {
      method: 'fd',
    });
    assert(unappliedRun.response.ok,
      `non-catalog reliability run failed: ${JSON.stringify(unappliedRun.data)}`);
    assert(unappliedRun.data.reliability_configuration?.configured === true &&
      unappliedRun.data.reliability_configuration?.applied === false &&
      unappliedRun.data.reliability_configuration?.consumer === 'failure_mode_fmea' &&
      unappliedRun.data.reliability_configuration?.limitation,
    'non-failure-mode method did not disclose that saved configuration was unapplied');

    const protectedThreeStage = await request(base, '/api/session/run_reliability', {
      method: 'three_stage',
      restoration: { parallel: false },
    });
    assert(protectedThreeStage.response.ok,
      `protection-conditioned three-stage run failed: ${JSON.stringify(protectedThreeStage.data)}`);
    assert(protectedThreeStage.data.reliability_configuration?.configured === true &&
      protectedThreeStage.data.reliability_configuration?.applied === true &&
      protectedThreeStage.data.reliability_configuration?.consumer === 'three_stage' &&
      JSON.stringify(protectedThreeStage.data.reliability_configuration?.applied_scope) ===
        JSON.stringify(['protection']),
    'saved protection configuration was not applied by three-stage restoration');
    const protectionModel = protectedThreeStage.data.protection_model;
    assert(protectionModel?.configuration_applied === true &&
      protectionModel.rows_applied > 0 &&
      protectionModel.sustained_scenarios_generated > 0,
    'three-stage protection audit is incomplete');
    const frequencyBalance = Number(protectionModel.sustained_fault_frequency_per_year) +
      Number(protectionModel.transient_reclose_frequency_per_year);
    assert(Math.abs(frequencyBalance -
      Number(protectionModel.initiating_fault_frequency_per_year)) < 1e-10,
    'protection scenario frequencies do not conserve initiating frequency');
    assert(typeof protectedThreeStage.data.metric_semantics?.protection_conditioning_formula ===
      'string', 'protection conditioning formula missing from API');
    const conditionedFaults = protectedThreeStage.data.faults.filter(fault =>
      fault.protection_id === configuration.protection[0].protection_id);
    assert(conditionedFaults.length > 0, 'no protection-conditioned fault rows returned');
    for (const fault of conditionedFaults) {
      assert(['primary_cleared', 'backup_cleared',
        'unresolved_after_backup_failure'].includes(fault.protection_scenario),
      `unexpected protection scenario ${fault.protection_scenario}`);
      assert(Number.isFinite(Number(fault.scenario_probability)) &&
        Number.isFinite(Number(fault.clearing_time_s)) &&
        Number.isFinite(Number(fault.initiating_failure_rate)),
      'conditioned fault is missing numeric probability/time/frequency fields');
    }

    const legacyThreeStage = await request(base,
      '/api/session/run_reliability_three_stage', { parallel: false });
    assert(legacyThreeStage.response.ok,
      `legacy protection-conditioned run failed: ${JSON.stringify(legacyThreeStage.data)}`);
    assert(legacyThreeStage.data.reliability_configuration?.applied === true &&
      legacyThreeStage.data.reliability_configuration?.consumer === 'three_stage' &&
      JSON.stringify(legacyThreeStage.data.reliability_configuration?.applied_scope) ===
        JSON.stringify(['protection']),
    'legacy three-stage route did not apply the protection scope');
    const legacyProtection = legacyThreeStage.data.protection_model;
    assert(Math.abs(Number(legacyProtection.sustained_fault_frequency_per_year) +
      Number(legacyProtection.transient_reclose_frequency_per_year) -
      Number(legacyProtection.initiating_fault_frequency_per_year)) < 1e-10,
    'legacy three-stage route does not conserve protection frequencies');
    assert(legacyThreeStage.data.metric_semantics?.protection_conditioning_formula ===
      protectedThreeStage.data.metric_semantics?.protection_conditioning_formula,
    'three-stage routes expose different protection formulas');

    const invalid = structuredClone(configuration);
    invalid.mode_overrides[0].probability_per_demand = 1.2;
    const rejected = await request(base, '/api/session/reliability/configuration/validate',
      { configuration: invalid });
    assert(rejected.response.status === 400, 'invalid probability was not rejected');

    await page.evaluate(() => App.setActiveModule('reliability'));
    await page.locator('#relReliabilityConfiguration').evaluate(element => {
      element.open = true;
      element.dispatchEvent(new Event('toggle'));
    });
    await page.waitForFunction(expected =>
      document.querySelectorAll('#relReliabilityModeRows tr').length === expected,
      saved.data.effective_modes.length);
    const editorFields = await page.evaluate(() => {
      const modeRow = document.querySelector('#relReliabilityModeRows tr');
      const protectionRow = document.querySelector('#relProtectionRows tr');
      return {
        mode: ['enabled', ...new Set(Array.from(
          modeRow?.querySelectorAll('[data-rel-field]') || [],
          input => input.dataset.relField))],
        protection: [...new Set(Array.from(
          protectionRow?.querySelectorAll('[data-protection-field]') || [],
          input => input.dataset.protectionField))],
      };
    });
    const schemaModeFields = saved.data.schema.mode_fields.map(field => field.name).sort();
    const schemaProtectionFields = saved.data.schema.protection_fields
      .map(field => field.name).sort();
    assert(JSON.stringify(editorFields.mode.sort()) === JSON.stringify(schemaModeFields),
      `GUI mode fields differ from backend schema: ${JSON.stringify(editorFields.mode)}`);
    assert(JSON.stringify(editorFields.protection.sort()) ===
      JSON.stringify(schemaProtectionFields),
    `GUI protection fields differ from backend schema: ${JSON.stringify(editorFields.protection)}`);
    const effectiveDisplay = await page.evaluate(({ modes, fields }) => {
      const missing = [];
      const kinds = new Set();
      let inherited = 0;
      for (const mode of modes) {
        const encoded = encodeURIComponent(mode.mode_id);
        kinds.add(mode.component_kind);
        for (const field of fields) {
          const input = document.querySelector(
            `input[data-rel-mode="${encoded}"][data-rel-field="${field}"]`);
          if (!input || input.value.trim() === '') missing.push(`${mode.mode_id}:${field}`);
          if (input?.classList.contains('rel-config-inherited')) inherited += 1;
        }
      }
      return { missing, inherited, kinds: [...kinds] };
    }, { modes: saved.data.effective_modes, fields: numberFieldNames });
    assert(effectiveDisplay.missing.length === 0,
      `GUI did not display resolved values: ${effectiveDisplay.missing.slice(0, 5).join(', ')}`);
    assert(effectiveDisplay.inherited > 0 &&
      effectiveDisplay.kinds.includes('ac_branch') &&
      effectiveDisplay.kinds.includes('ac_transformer_2w'),
    `GUI resolved coverage incomplete: ${JSON.stringify(effectiveDisplay)}`);
    const gui = await page.evaluate(async modeId => {
      const encoded = encodeURIComponent(modeId);
      const input = document.querySelector(
        `input[data-rel-mode="${encoded}"][data-rel-field="repair_hr"]`);
      input.value = '11';
      input.dispatchEvent(new Event('input', { bubbles: true }));
      document.getElementById('btnRelConfigSave').click();
      const deadline = Date.now() + 10000;
      while (Date.now() < deadline) {
        const response = await fetch('/api/session/reliability/configuration');
        const data = await response.json();
        const row = data.configuration?.mode_overrides?.find(item => item.mode_id === modeId);
        if (row?.repair_hr === 11) {
          return { rows: document.querySelectorAll('#relReliabilityModeRows tr').length,
                   repair_hr: row.repair_hr,
                   override_count: data.configuration.mode_overrides.length,
                   method: document.getElementById('relMethod')?.value };
        }
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      const response = await fetch('/api/session/reliability/configuration');
      const data = await response.json();
      return { rows: document.querySelectorAll('#relReliabilityModeRows tr').length,
               repair_hr: null,
               status: document.getElementById('relConfigStatus')?.textContent || '',
               api_configuration: data.configuration || null };
    }, transformerMode.mode_id);
    assert(gui.rows === saved.data.effective_modes.length && gui.repair_hr === 11 &&
      gui.override_count === 1 && gui.method === 'failure_mode_fmea',
      `GUI mapping failed: ${JSON.stringify(gui)}`);

    const nonGeneratorEdit = await page.evaluate(async modeId => {
      const encoded = encodeURIComponent(modeId);
      const input = document.querySelector(
        `input[data-rel-mode="${encoded}"][data-rel-field="failure_rate_per_year"]`);
      const inheritedValue = input?.value || '';
      const inherited = input?.dataset.relExplicit === 'false' &&
        input?.classList.contains('rel-config-inherited');
      input.value = '0.456';
      input.dispatchEvent(new Event('input', { bubbles: true }));
      document.getElementById('btnRelConfigSave').click();
      const deadline = Date.now() + 10000;
      while (Date.now() < deadline) {
        const data = await fetch('/api/session/reliability/configuration').then(r => r.json());
        const row = data.configuration?.mode_overrides?.find(item => item.mode_id === modeId);
        if (row?.failure_rate_per_year === 0.456) {
          const explicitNumeric = Object.entries(row)
            .filter(([key, value]) => key !== 'mode_id' && key !== 'enabled' && value !== null)
            .map(([key]) => key);
          return { inherited, inheritedValue, explicitNumeric,
            overrideCount: data.configuration.mode_overrides.length };
        }
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      return { inherited, inheritedValue, timeout: true };
    }, branchMode.mode_id);
    assert(nonGeneratorEdit.inherited && nonGeneratorEdit.inheritedValue !== '' &&
      JSON.stringify(nonGeneratorEdit.explicitNumeric) ===
        JSON.stringify(['failure_rate_per_year']) &&
      nonGeneratorEdit.overrideCount === 2,
    `non-generator resolved-value edit was not sparse: ${JSON.stringify(nonGeneratorEdit)}`);

    const preservedLoad = await request(base, '/api/session/load_json_string', {
      json_string: loaded.data._raw_json,
      preserve_reliability_configuration: true,
    });
    assert(preservedLoad.response.ok, 'same-model canvas sync failed');
    const preserved = await request(base, '/api/session/reliability/configuration');
    assert(preserved.data.configuration.mode_overrides.length === 2,
      'same-model canvas sync discarded reliability overrides');

    const componentReplacement = await request(base, '/api/session/update_components', {
      name: 'Reliability component replacement',
    });
    assert(componentReplacement.response.ok, 'component-array replacement failed');
    const componentReset = await request(base, '/api/session/reliability/configuration');
    assert(componentReset.data.configuration.mode_overrides.length === 0 &&
      componentReset.data.configuration.protection.length === 0,
      'component-array replacement retained stale reliability configuration');
    const restored = await request(base, '/api/session/reliability/configuration', {
      configuration,
    });
    assert(restored.response.ok, 'configuration restore before model replacement failed');

    const replacement = await request(base, '/api/session/load_builtin', {
      case: 'ieee24_3area_acdc_expanded',
    });
    assert(replacement.response.ok, 'replacement model load failed');
    const reset = await request(base, '/api/session/reliability/configuration');
    assert(reset.data.configuration.mode_overrides.length === 0 &&
      reset.data.configuration.protection.length === 0,
      'replacement model retained stale reliability configuration');

    const broadLoad = await page.evaluate(async () => {
      await App.loadBuiltinCase('multiscale_comprehensive_acdc');
      return {
        canvasComponents: Canvas.state.components.length,
        headless: Canvas.isHeadless(),
      };
    });
    assert(broadLoad.canvasComponents > 0 && broadLoad.headless === false,
      `comprehensive reliability case did not load on Canvas: ${JSON.stringify(broadLoad)}`);
    const broad = await request(base, '/api/session/reliability/configuration');
    assert(broad.response.ok, 'comprehensive reliability configuration failed');
    const broadKinds = new Set(broad.data.components.map(row => row.component_kind));
    for (const kind of [
      'ac_bus', 'dc_bus', 'ac_generator', 'ac_branch', 'dc_branch',
      'ac_load', 'dc_load', 'ac_static_generator', 'ac_renewable_generator',
      'ac_storage', 'dc_storage', 'ac_pv_system', 'dc_pv_array',
      'ac_transformer_2w', 'ac_switch', 'ac_circuit_breaker',
      'dc_circuit_breaker', 'vsc_converter', 'dcdc_converter',
    ]) {
      assert(broadKinds.has(kind), `comprehensive case missing reliability kind ${kind}`);
    }
    for (const mode of broad.data.effective_modes) {
      for (const field of numberFieldNames) {
        assert(Number.isFinite(Number(mode.resolved_parameters?.[field])),
          `comprehensive ${mode.mode_id}:${field} is not finite`);
      }
    }
    assert(broad.data.coverage.modes_unsupported ===
      broad.data.effective_modes.filter(mode => mode.supported === false).length,
    'comprehensive unsupported coverage differs from effective mode support flags');

    const propertyCoverage = await page.evaluate(async () => {
      const configuration = await fetch('/api/session/reliability/configuration')
        .then(response => response.json());
      const maps = Canvas.getCompBusMap();
      const buckets = {
        ac_bus: 'ac', ac_bus_load: 'ac', dc_bus: 'dc', dc_bus_load: 'dc',
        ac_generator: 'gen', ac_branch: 'branch', ac_load: 'load',
        ac_static_generator: 'sgen', dc_static_generator_ac: 'dcSgen',
        dc_static_generator: 'dcSgen', ac_renewable_generator: 'renGen',
        ac_storage: 'storage', ac_pv_system: 'pv', ac_transformer_2w: 'trafo',
        ac_transformer_3w: 'trafo3w', ac_switch: 'sw',
        ac_circuit_breaker: 'cb', external_grid: 'extGrid', dc_branch: 'dcBranch',
        dc_load: 'dcLoad', dcdc_converter: 'dcdcConverter',
        dc_circuit_breaker: 'dcCb', dc_storage: 'dcStorage',
        dc_dedicated_storage: 'dcStorage', dc_pv_array: 'dcPv',
        vsc_converter: 'vsc', lcc_converter: 'lcc', energy_router: 'energyRouter',
        microgrid: 'microgrid', mobile_storage: 'mobileStorage',
        virtual_power_plant: 'vpp', flexible_load: 'flexLoad',
        asymmetric_load: 'asymLoad', shunt: 'shunt', charger: 'charger',
        charging_station: 'chargingStation', asynchronous_motor: 'motor',
      };
      const targets = new Map();
      for (const component of configuration.components || []) {
        const bucket = buckets[component.component_kind];
        const compId = bucket ? maps[bucket]?.[Number(component.component_index)] : null;
        if (compId !== null && compId !== undefined &&
            Number.isInteger(Number(compId)) && !targets.has(component.component_kind))
          targets.set(component.component_kind, Number(compId));
      }
      const failures = [];
      const checked = [];
      for (const [kind, compId] of targets) {
        Canvas.panToComponent(compId);
        const deadline = Date.now() + 5000;
        while (Date.now() < deadline) {
          const overview = document.querySelector('#propFields .prop-reliability-overview');
          const empty = document.querySelector('#propFields .prop-reliability-empty');
          if (overview || (empty && !empty.textContent.includes('正在读取'))) break;
          await new Promise(resolve => setTimeout(resolve, 20));
        }
        const modes = document.querySelectorAll('#propFields [data-rel-property-mode]').length;
        const categories = Array.from(document.querySelectorAll('#propFields [data-property-section]'),
          row => row.dataset.propertySection);
        if (!categories.includes('reliability') || modes === 0)
          failures.push({
            kind, compId, selectedId: Canvas.state.selectedId, modes, categories,
            reliabilityText: document.querySelector(
              '#propFields [data-property-section="reliability"]')?.textContent || '',
          });
        checked.push(kind);
      }
      const generatorId = maps.gen?.[configuration.components.find(
        component => component.component_kind === 'ac_generator')?.component_index];
      if (Number.isInteger(Number(generatorId))) {
        Canvas.panToComponent(Number(generatorId));
        const deadline = Date.now() + 5000;
        while (Date.now() < deadline &&
               !document.querySelector('#propFields [data-rel-property-mode]')) {
          await new Promise(resolve => setTimeout(resolve, 20));
        }
      }
      return {
        checked,
        failures,
        generatorRawGrouped: !!document.querySelector(
          '#propFields [data-property-section="reliability"] [data-field="forced_outage_rate"]'),
      };
    });
    assert(propertyCoverage.checked.length >= 12 && propertyCoverage.failures.length === 0 &&
      propertyCoverage.generatorRawGrouped,
    `component property reliability grouping incomplete: ${JSON.stringify(propertyCoverage)}`);
    assert(pageErrors.length === 0,
      `component property editor raised page errors: ${JSON.stringify(pageErrors)}`);

    const propertyEdit = await page.evaluate(async () => {
      const configuration = await fetch('/api/session/reliability/configuration')
        .then(response => response.json());
      const branch = configuration.components.find(component =>
        component.component_kind === 'ac_branch');
      const compId = Canvas.getCompBusMap().branch?.[Number(branch?.component_index)];
      Canvas.panToComponent(Number(compId));
      const deadline = Date.now() + 5000;
      while (Date.now() < deadline &&
             !document.querySelector('#propFields [data-rel-property-field="failure_rate_per_year"]')) {
        await new Promise(resolve => setTimeout(resolve, 20));
      }
      const input = document.querySelector(
        '#propFields [data-rel-property-field="failure_rate_per_year"]');
      const modeRow = input?.closest('[data-rel-property-mode]');
      const modeId = decodeURIComponent(modeRow?.dataset.relPropertyMode || '');
      input.value = '0.654';
      input.dispatchEvent(new Event('input', { bubbles: true }));
      document.getElementById('btnApplyProp').click();
      const saveDeadline = Date.now() + 10000;
      while (Date.now() < saveDeadline) {
        const data = await fetch('/api/session/reliability/configuration')
          .then(response => response.json());
        const row = data.configuration?.mode_overrides?.find(value => value.mode_id === modeId);
        if (row?.failure_rate_per_year === 0.654) {
          return {
            modeId,
            explicit: Object.entries(row)
              .filter(([key, value]) => key !== 'mode_id' && value !== null)
              .map(([key]) => key),
          };
        }
        await new Promise(resolve => setTimeout(resolve, 50));
      }
      return { modeId, timeout: true };
    });
    assert(!propertyEdit.timeout &&
      JSON.stringify(propertyEdit.explicit) === JSON.stringify(['failure_rate_per_year']),
    `component property reliability edit was not sparse: ${JSON.stringify(propertyEdit)}`);

    await page.evaluate(() => {
      const panel = document.getElementById('rightPanel');
      if (panel) panel.style.width = '1180px';
    });
    await page.setViewportSize({ width: 390, height: 844 });
    await page.waitForTimeout(100);
    const mobileProperties = await page.evaluate(() => {
      const width = element => element?.getBoundingClientRect().width || 0;
      return {
        viewport: window.innerWidth,
        pageOverflow: document.documentElement.scrollWidth - window.innerWidth,
        panelWidth: width(document.getElementById('rightPanel')),
        reliabilityWidth: width(document.querySelector(
          '#propFields [data-property-section="reliability"]')),
        gridWidth: width(document.querySelector('#propFields .prop-reliability-grid')),
      };
    });
    assert(mobileProperties.pageOverflow <= 1 &&
      mobileProperties.panelWidth <= mobileProperties.viewport + 1 &&
      mobileProperties.reliabilityWidth <= mobileProperties.viewport + 1 &&
      mobileProperties.gridWidth <= mobileProperties.viewport + 1,
    `mobile component properties overflow: ${JSON.stringify(mobileProperties)}`);

    console.log(`reliability configuration passed: components=${saved.data.coverage.components_total}, ` +
      `modes=${saved.data.coverage.modes_total}, mode_fields=${fieldNames.length}, ` +
      `protection_fields=${saved.data.schema.protection_fields.length}, ` +
      `broad_components=${broad.data.coverage.components_total}, ` +
      `broad_modes=${broad.data.coverage.modes_total}, ` +
      `property_kinds=${propertyCoverage.checked.length}`);
  } finally {
    if (browser) await browser.close();
    proc.kill();
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
