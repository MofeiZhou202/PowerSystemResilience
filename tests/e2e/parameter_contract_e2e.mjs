// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import path from 'node:path';
import process from 'node:process';
import { existsSync } from 'node:fs';

const ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..', '..');

function arg(name, fallback = null) {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && i + 1 < process.argv.length ? process.argv[i + 1] : fallback;
}

function serverPath() {
  const explicit = arg('server');
  if (explicit) return path.resolve(explicit);
  for (const rel of ['build/macos-release/tests/run_gui_server',
                     'build/macos-release/run_gui_server']) {
    const candidate = path.join(ROOT, rel);
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
  for (let i = 0; i < 80; ++i) {
    try {
      if ((await fetch(`${base}/api/cases`)).ok) return;
    } catch { /* server is starting */ }
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  throw new Error('GUI server did not start');
}

function alternate(rule) {
  let value = rule.min_value + 0.41 * (rule.max_value - rule.min_value);
  if (Math.abs(value - rule.default_value) < 1e-10) {
    value = rule.min_value + 0.59 * (rule.max_value - rule.min_value);
  }
  return value;
}

async function jsonRequest(base, endpoint, options = {}) {
  const response = await fetch(`${base}${endpoint}`, options);
  const body = await response.json();
  if (!response.ok) throw new Error(`${endpoint}: ${body.error || response.status}`);
  return body;
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
    await jsonRequest(base, '/api/session/load_builtin', {
      method: 'POST', headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ case: 'dist33_microgrid_der' }),
    });
    const initial = await jsonRequest(base, '/api/session/parameter_library');
    const expectedCount = initial.rules.length;
    if (expectedCount < 52) throw new Error(`expected at least 52 rules, got ${expectedCount}`);
    if (initial.parameter_contract?.registered_count !== expectedCount) {
      throw new Error('registry count does not match the returned rule set');
    }
    if (!Array.isArray(initial.model_catalog) || initial.model_catalog.length < 40) {
      throw new Error('component model catalog is incomplete');
    }
    if (initial.model_catalog.some(item => item.component_type.startsWith('Reliability -'))) {
      throw new Error('reliability parameter groups must not appear as component models');
    }
    if (!Array.isArray(initial.parameter_instances) || initial.parameter_instances.length < 30) {
      throw new Error('current model instance snapshot is incomplete');
    }
    const physicalTypes = ['System', 'AC bus', 'AC branch', 'Transformer', 'DC bus',
      'DC branch', 'VSC', 'DC-DC converter', 'Storage'];
    for (const type of physicalTypes) {
      const model = initial.model_catalog.find(item => item.component_type === type);
      if (!model || !model.model_scope || !model.equivalent_circuit_family ||
          !Array.isArray(model.rule_ids) || !model.rule_ids.length) {
        throw new Error(`${type}: model catalog entry is incomplete`);
      }
    }
    const instanceIdentities = new Set();
    for (const instance of initial.parameter_instances) {
      if (!instance.identity || instanceIdentities.has(instance.identity)) {
        throw new Error(`invalid or duplicate instance identity: ${instance.identity}`);
      }
      instanceIdentities.add(instance.identity);
      if (!Number.isInteger(instance.component_index) || !instance.component_kind ||
          typeof instance.values !== 'object' || !Array.isArray(instance.model_parameters) ||
          !instance.model_parameters.length || !Array.isArray(instance.reliability_modes)) {
        throw new Error(`${instance.identity}: incomplete instance contract`);
      }
      const fields = new Set();
      for (const field of instance.model_parameters) {
        if (!field.field || fields.has(field.field) || !('value' in field) ||
            !field.group || !('source' in field)) {
          throw new Error(`${instance.identity}: invalid complete model parameter ${field.field}`);
        }
        fields.add(field.field);
      }
    }
    const branchWithReliability = initial.parameter_instances.find(instance =>
      instance.component_kind === 'ac_branch' && instance.reliability_modes.length);
    if (!branchWithReliability || !branchWithReliability.reliability_modes.every(mode =>
        mode.resolved_parameters && mode.data_source)) {
      throw new Error('reliability modes are not integrated into AC branch instances');
    }
    for (const id of ['reliability.ac_branch.failure_rate',
                      'reliability.generator.forced_outage_rate',
                      'reliability.microgrid.mtbf_hours']) {
      if (!initial.rules.some(rule => rule.id === id)) {
        throw new Error(`${id}: reliability rule missing from GUI contract`);
      }
    }
    for (const rule of initial.rules) {
      for (const key of ['symbol', 'quantity', 'model_role', 'equation',
                         'equivalent_circuit_family', 'typical_min', 'typical_max',
                         'typical_range_kind', 'typical_range_source']) {
        if (!(key in rule)) throw new Error(`${rule.id}: presentation field ${key} missing`);
      }
      if (rule.typical_min !== null && rule.typical_max !== null &&
          Number(rule.typical_min) > Number(rule.typical_max)) {
        throw new Error(`${rule.id}: reversed typical range`);
      }
      const defaultEffective = initial.effective_parameters?.values?.[rule.id];
      if (!defaultEffective ||
          Math.abs(defaultEffective.value - rule.default_value) > 1e-12) {
        throw new Error(`${rule.id}: default configuration is not effective`);
      }
      const contract = rule.contract || {};
      for (const key of ['default_config', 'external_json_override', 'api_effective',
                         'gui_editable', 'effective_parameters_echo']) {
        if (contract[key] !== true) throw new Error(`${rule.id}: contract ${key} is not true`);
      }
      rule.default_value = alternate(rule);
    }

    const updated = await jsonRequest(base, '/api/session/parameter_library/update', {
      method: 'POST', headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ library: initial }),
    });
    for (const rule of updated.rules) {
      const effective = updated.effective_parameters?.values?.[rule.id];
      if (!effective || Math.abs(effective.value - rule.default_value) > 1e-12) {
        throw new Error(`${rule.id}: external override is not effective`);
      }
    }

    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
    await page.goto(`${base}/xjtu/`, { waitUntil: 'networkidle' });
    await page.evaluate(() => App.loadBuiltinCase('dist33_microgrid_der'));
    await page.evaluate(() => App.setActiveModule('parameterLibrary'));
    await page.locator('#parameterModelExplorer').waitFor({ state: 'visible' });
    await page.waitForFunction(() =>
      document.querySelectorAll('#parameterLibraryTable tbody tr[data-rule-id]').length > 0);
    const modelOptions = await page.locator('#parameterLibraryComponentFilter option').allTextContents();
    if (modelOptions.length !== initial.model_catalog.length + 1 ||
        modelOptions.some(label => label.startsWith('Reliability -'))) {
      throw new Error(`GUI model selector is incomplete or contains reliability pseudo-models: ${modelOptions.length}`);
    }
    const projectRange = await page.locator(
      '#parameterLibraryTable tbody tr[data-rule-id="system.base_mva"] .parameter-range-value').innerText();
    if (projectRange !== '按项目 / 铭牌校核') {
      throw new Error(`null typical range rendered incorrectly: ${projectRange}`);
    }
    await page.selectOption('#parameterLibraryComponentFilter', 'AC branch');
    const modelView = await page.evaluate(() => {
      const instance = document.getElementById('parameterLibraryInstance');
      const row = document.querySelector('#parameterLibraryTable tbody tr[data-rule-id="ac_branch.r_pu"]');
      return {
        instances: instance?.options.length || 0,
        identity: instance?.value || '',
        current: row?.querySelector('.parameter-current-value')?.textContent || '',
        typical: row?.querySelector('.parameter-range-value')?.textContent || '',
        hardInputs: row?.querySelectorAll('.parameter-hard-range input').length || 0,
        equation: row?.querySelector('.parameter-model-role + code')?.textContent || '',
        circuit: document.querySelector('#parameterEquivalentCircuit svg')?.getAttribute('aria-label') || '',
      };
    });
    if (modelView.instances < 1 || !modelView.identity.startsWith('ac:ac_branch:') ||
        !Number.isFinite(Number(modelView.current)) || !modelView.typical ||
        modelView.hardInputs !== 2 || !modelView.equation || !modelView.circuit.includes('线路')) {
      throw new Error(`AC branch model explorer incomplete: ${JSON.stringify(modelView)}`);
    }
    await page.selectOption('#parameterLibraryComponentFilter', 'VSC');
    const vscCircuit = await page.locator('#parameterEquivalentCircuit svg').getAttribute('aria-label');
    if (!vscCircuit?.includes('VSC')) throw new Error('equivalent circuit did not switch to VSC');
    await page.selectOption('#parameterLibraryComponentFilter', 'Generator');
    const generatorCoverage = await page.evaluate(() => ({
      identity: document.getElementById('parameterLibraryInstance')?.value || '',
      standardRows: document.querySelectorAll('#parameterLibraryTable tr[data-rule-id]').length,
      instanceRows: document.querySelectorAll('#parameterLibraryTable tr[data-instance-field]').length,
      reliabilityRows: document.querySelectorAll(
        '#parameterLibraryTable tr[data-parameter-group="reliability"]').length,
      noxUnit: document.querySelector(
        '#parameterLibraryTable tr[data-instance-field="nox_factor_kg_mwh"] td:nth-child(5)')?.textContent.trim() || '',
      groups: [...new Set([...document.querySelectorAll(
        '#parameterLibraryTable tr[data-parameter-group]')].map(row => row.dataset.parameterGroup))],
    }));
    if (!generatorCoverage.identity.startsWith('ac:ac_generator:') ||
        generatorCoverage.standardRows !== 0 || generatorCoverage.instanceRows < 20 ||
        generatorCoverage.reliabilityRows < 10 || generatorCoverage.groups.length < 4 ||
        generatorCoverage.noxUnit !== 'kg/MWh') {
      throw new Error(`generator complete/integrated parameter coverage is incomplete: ${JSON.stringify(generatorCoverage)}`);
    }
    await page.evaluate(() => App.loadBuiltinCase('cyber_physical_reliability_demo'));
    await page.evaluate(() => App.loadBuiltinCase('dist33_microgrid_der'));
    await page.evaluate(() => App.setActiveModule('parameterLibrary'));
    await page.selectOption('#parameterLibraryComponentFilter', 'AC branch');
    const refreshedInstances = await page.locator('#parameterLibraryInstance option').count();
    if (refreshedInstances < 1) throw new Error('model replacement did not refresh parameter instances');

    const currentContract = await jsonRequest(base, '/api/session/parameter_library');
    const canvasBucket = {
      ac_bus: 'ac', ac_generator: 'gen', ac_load: 'load', ac_branch: 'branch',
      transformer_2w: 'trafo', external_grid: 'extGrid', storage: 'storage',
      ac_pv_system: 'pv', ac_renewable_generator: 'renGen',
      ac_static_generator: 'sgen', ac_switch: 'sw', ac_circuit_breaker: 'cb',
      asynchronous_motor: 'motor', flexible_load: 'flexLoad',
      asymmetric_load: 'asymLoad', shunt: 'shunt', ac_transformer_3w: 'trafo3w',
      charger: 'charger', charging_station: 'chargingStation', dc_bus: 'dc',
      dc_branch: 'dcBranch', dc_load: 'dcLoad', dc_storage: 'dcStorage',
      dc_static_generator: 'dcSgen', dc_static_generator_ac: 'dcSgen',
      dc_pv_array: 'dcPv', dc_circuit_breaker: 'dcCb', vsc_converter: 'vsc',
      lcc_converter: 'lcc', dcdc_converter: 'dcdcConverter',
      energy_router: 'energyRouter', mobile_storage: 'mobileStorage',
      virtual_power_plant: 'vpp', microgrid: 'microgrid',
    };
    async function assertCanvasSelectionSync(instance) {
      const bucket = canvasBucket[instance.component_kind];
      const navigation = await page.evaluate(({ bucketName, componentIndex }) => {
        const maps = Canvas.getCompBusMap();
        const componentId = maps[bucketName]?.[componentIndex];
        if (componentId === undefined || componentId === null) return { componentId: null };
        Canvas.panToComponent(componentId);
        return { componentId, selectedId: Canvas.state.selectedId };
      }, { bucketName: bucket, componentIndex: instance.component_index });
      if (navigation.componentId === null || navigation.selectedId !== navigation.componentId) {
        throw new Error(`${instance.identity}: Canvas stable mapping is missing`);
      }
      await page.waitForFunction(identity =>
        document.getElementById('parameterLibraryInstance')?.value === identity,
      instance.identity);
      const ruleId = Object.keys(instance.values || {}).find(id =>
        Number.isFinite(Number(instance.values[id])));
      const modelField = !ruleId && (instance.model_parameters || []).find(field =>
        typeof field.value === 'number' && Number.isFinite(field.value));
      if (!ruleId && !modelField) {
        throw new Error(`${instance.identity}: no finite current value to verify`);
      }
      const view = await page.evaluate(({ id, field }) => ({
        componentType: document.getElementById('parameterLibraryComponentFilter')?.value || '',
        identity: document.getElementById('parameterLibraryInstance')?.value || '',
        current: document.querySelector(field
          ? `#parameterLibraryTable tbody tr[data-instance-field="${CSS.escape(field)}"] .parameter-current-value`
          : `#parameterLibraryTable tbody tr[data-rule-id="${CSS.escape(id)}"] .parameter-current-value`)?.textContent || '',
        activeModule: document.querySelector('.module-btn.active')?.dataset.module || '',
        activeTab: document.querySelector('.panel-tab.active')?.dataset.tab || '',
      }), { id: ruleId || '', field: modelField?.field || '' });
      const expectedCurrent = ruleId ? instance.values[ruleId] : modelField.value;
      if (view.componentType !== instance.component_type || view.identity !== instance.identity ||
          Math.abs(Number(view.current) - Number(expectedCurrent)) > 1e-12 ||
          view.activeModule !== 'parameterLibrary' || view.activeTab !== 'results') {
        throw new Error(`${instance.identity}: Canvas selection did not synchronize the model explorer: ${JSON.stringify(view)}`);
      }
    }

    const coreFamilies = [
      ['ac_bus', row => row.component_kind === 'ac_bus'],
      ['dc_bus', row => row.component_kind === 'dc_bus'],
      ['ac_branch', row => row.component_kind === 'ac_branch' && row.component_type === 'AC branch'],
      ['dc_branch', row => row.component_kind === 'dc_branch'],
      ['vsc_converter', row => row.component_kind === 'vsc_converter'],
      ['storage', row => row.component_type === 'Storage'],
    ];
    for (const [family, predicate] of coreFamilies) {
      if (!currentContract.parameter_instances.some(predicate)) {
        throw new Error(`${family}: built-in case lacks a core synchronization fixture`);
      }
    }
    const synchronizationFixtures = [...new Map(currentContract.parameter_instances
      .filter(row => canvasBucket[row.component_kind])
      .map(row => [`${row.domain}:${row.component_kind}:${row.component_type}`, row])).values()];
    for (const instance of synchronizationFixtures) await assertCanvasSelectionSync(instance);

    const acAndDcSameIndex = currentContract.parameter_instances.find(ac =>
      ac.component_kind === 'ac_bus' && currentContract.parameter_instances.some(dc =>
        dc.component_kind === 'dc_bus' && dc.component_index === ac.component_index));
    const matchingDcBus = acAndDcSameIndex && currentContract.parameter_instances.find(dc =>
      dc.component_kind === 'dc_bus' && dc.component_index === acAndDcSameIndex.component_index);
    if (!acAndDcSameIndex || !matchingDcBus) {
      throw new Error('built-in case lacks same-number AC/DC buses');
    }
    await assertCanvasSelectionSync(acAndDcSameIndex);
    await assertCanvasSelectionSync(matchingDcBus);

    const branchInstances = currentContract.parameter_instances.filter(row =>
      row.component_kind === 'ac_branch' && row.component_type === 'AC branch');
    if (branchInstances.length < 2) throw new Error('built-in case lacks two AC branch instances');
    await assertCanvasSelectionSync(branchInstances[0]);
    const unsavedBranchDefault = await page.locator(
      '#parameterLibraryTable tbody tr[data-rule-id="ac_branch.r_pu"] input[data-field="default_value"]');
    await unsavedBranchDefault.fill('0.0123456789');
    await assertCanvasSelectionSync(branchInstances[1]);
    if (await unsavedBranchDefault.inputValue() !== '0.0123456789') {
      throw new Error('Canvas instance synchronization discarded an unsaved parameter-library edit');
    }

    const gui = await page.evaluate(async () => {
      const rows = [...document.querySelectorAll('#parameterLibraryTable tbody tr[data-rule-id]')];
      const totalRows = document.querySelectorAll('#parameterLibraryTable tbody tr').length;
      const instanceRows = document.querySelectorAll(
        '#parameterLibraryTable tbody tr[data-instance-field]').length;
      const editable = rows.filter(row => {
        const input = row.querySelector('input[data-field="default_value"]');
        return input && !input.disabled;
      });
      const first = editable.find(row => row.dataset.ruleId === 'ac_branch.r_pu');
      if (!first) return { rows: rows.length, totalRows, instanceRows, editable: editable.length,
        id: '', next: null, effective: null };
      const id = first.dataset.ruleId;
      const input = first.querySelector('input[data-field="default_value"]');
      const next = Number(input.value) * 0.99;
      input.value = String(next);
      document.getElementById('btnParameterLibrarySave').click();
      const deadline = Date.now() + 10000;
      while (Date.now() < deadline) {
        const response = await fetch('/api/session/parameter_library');
        const data = await response.json();
        const rule = data.rules.find(item => item.id === id);
        if (rule && Math.abs(rule.default_value - next) < 1e-12) {
          return { rows: rows.length, totalRows, instanceRows, editable: editable.length, id, next,
                   effective: data.effective_parameters.values[id].value };
        }
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      return { rows: rows.length, totalRows, instanceRows, editable: editable.length,
        id, next, effective: null };
    });
    if (gui.rows !== 3 || gui.editable !== 3 || gui.instanceRows < 10 ||
        gui.totalRows !== gui.rows + gui.instanceRows) {
      throw new Error(`filtered AC branch GUI coverage ${JSON.stringify(gui)}, expected 3 editable standard rules plus complete instance rows`);
    }
    if (Math.abs(gui.effective - gui.next) > 1e-12) {
      throw new Error(`${gui.id}: GUI edit did not reach effective API value`);
    }

    await page.evaluate(() => App.loadBuiltinCase('comprehensive_hybrid_acdc'));
    await page.evaluate(() => App.setActiveModule('parameterLibrary'));
    const comprehensiveContract = await jsonRequest(base, '/api/session/parameter_library');
    const transformerInstance = comprehensiveContract.parameter_instances.find(row =>
      row.component_kind === 'transformer_2w') ||
      comprehensiveContract.parameter_instances.find(row => row.component_type === 'Transformer');
    if (!transformerInstance) {
      throw new Error('comprehensive case lacks a transformer synchronization fixture');
    }
    await assertCanvasSelectionSync(transformerInstance);
    const dcdcInstance = comprehensiveContract.parameter_instances.find(row =>
      row.component_kind === 'dcdc_converter');
    if (dcdcInstance) await assertCanvasSelectionSync(dcdcInstance);

    await page.setViewportSize({ width: 390, height: 844 });
    const mobile = await page.evaluate(() => ({
      pageOverflow: document.documentElement.scrollWidth - document.documentElement.clientWidth,
      explorerWidth: document.getElementById('parameterModelExplorer')?.getBoundingClientRect().width || 0,
      viewport: document.documentElement.clientWidth,
      circuitHeight: document.querySelector('#parameterEquivalentCircuit svg')?.getBoundingClientRect().height || 0,
    }));
    if (mobile.pageOverflow > 1 || mobile.explorerWidth > mobile.viewport + 1 ||
        mobile.circuitHeight < 80) {
      throw new Error(`mobile model parameter layout invalid: ${JSON.stringify(mobile)}`);
    }
    console.log(`parameter contract passed: defaults=${expectedCount}, JSON=${expectedCount}, API=${expectedCount}, model_catalog=${initial.model_catalog.length}, instances=${initial.parameter_instances.length}, GUI=AC branch, effective=${gui.id}, mobile_overflow=${mobile.pageOverflow}`);
  } finally {
    if (browser) await browser.close();
    proc.kill();
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
