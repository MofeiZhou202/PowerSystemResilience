// Browser regression for BPA/DSP DAT -> LCC canvas -> C++ JSON round-trip.

import path from 'node:path';
import process from 'node:process';
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

function arg(name, fallback) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length
    ? process.argv[index + 1]
    : fallback;
}

const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const browserChannel = arg('browser-channel', null);

function freePort() {
  return new Promise((resolve, reject) => {
    const probe = createServer();
    probe.once('error', reject);
    probe.listen(0, '127.0.0.1', () => {
      const address = probe.address();
      probe.close(error => error ? reject(error) : resolve(address.port));
    });
  });
}

async function waitUp(baseUrl) {
  for (let attempt = 0; attempt < 100; ++attempt) {
    try {
      const response = await fetch(baseUrl + '/api/v1');
      if (response.ok) return true;
    } catch {
      // The child process may still be binding its socket.
    }
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  return false;
}

async function launchBrowser(chromium) {
  if (browserChannel) return chromium.launch({ channel: browserChannel });
  try {
    return await chromium.launch();
  } catch (error) {
    if (process.platform === 'win32' &&
        String(error?.message || error).includes('Executable doesn\'t exist')) {
      return chromium.launch({ channel: 'msedge' });
    }
    throw error;
  }
}

function hasExpectedCigreTapMetadata(converters) {
  if (!Array.isArray(converters) || converters.length !== 2) return false;
  const expected = new Map([
    ['RECTIFIER', {
      externalCode: 'PAAL',
      tapMin: 475 / 525,
      tapMax: 610 / 525,
    }],
    ['INVERTER', {
      externalCode: 'VDGA',
      tapMin: 498.75 / 525,
      tapMax: 603.75 / 525,
    }],
  ]);
  const branchRefs = new Set();
  const ok = converters.every(converter => {
    const item = expected.get(converter.station_role);
    const branchRef = Number(converter.converter_transformer_branch);
    if (!item || !Number.isInteger(branchRef) || branchRef < 0) return false;
    branchRefs.add(branchRef);
    return converter.external_control_code === item.externalCode &&
      converter.tap_control_modelled === true &&
      Math.abs(Number(converter.transformer_tap_min_pu) - item.tapMin) < 1e-12 &&
      Math.abs(Number(converter.transformer_tap_max_pu) - item.tapMax) < 1e-12 &&
      Number(converter.transformer_tap_steps) === 0 &&
      Number(converter.transformer_tap_winding) === 1;
  });
  return ok && branchRefs.size === 2;
}

async function main() {
  let chromium;
  try {
    ({ chromium } = await import('playwright'));
  } catch {
    console.error('playwright not installed');
    return 2;
  }

  let serverProcess = null;
  let browser = null;
  try {
    let baseUrl = arg('base-url', null);
    if (!baseUrl) {
      const serverPath = arg('server', null);
      if (!serverPath) throw new Error('provide --base-url or --server');
      const dataDir = path.resolve(arg('data-dir', path.join(repoRoot, 'data')));
      const port = Number(arg('port', 0)) || await freePort();
      baseUrl = `http://127.0.0.1:${port}`;
      serverProcess = spawn(path.resolve(serverPath), [
        '--host', '127.0.0.1', '--port', String(port),
        '--data-dir', dataDir, '--matpower-dir', dataDir,
      ], { cwd: repoRoot, stdio: 'ignore' });
      if (!await waitUp(baseUrl)) {
        throw new Error('local GUI server did not start');
      }
    }

    browser = await launchBrowser(chromium);
    const page = await browser.newPage();

    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    const pageErrors = [];
    page.on('pageerror', error => pageErrors.push(error.message));
    await page.route('https://cdn.plot.ly/**', route => route.fulfill({
      contentType: 'text/javascript',
      body: 'window.Plotly={newPlot:()=>Promise.resolve(),react:()=>Promise.resolve(),Plots:{resize:()=>{}}};',
    }));
    await page.goto(baseUrl + '/xjtu/', { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() =>
      typeof App !== 'undefined' && typeof Canvas !== 'undefined');

    const datBytes = Array.from(
      readFileSync(path.join(repoRoot, 'data', 'dsp', 'cigre.dat')),
    );
    const imported = await page.evaluate(async bytes => {
      const file = new File(
        [new Uint8Array(bytes)],
        'cigre.dat',
        { type: 'application/octet-stream' },
      );
      const data = await App.loadBpaDat(file);
      return {
        ok: Boolean(data),
        error: data?.error || '',
      };
    }, datBytes);
    if (!imported.ok) {
      throw new Error(`DAT import failed: ${imported.error}`);
    }
    await page.waitForFunction(() =>
      Canvas.buildSystemJson().lcc_converters?.length === 2);

    const canvasResult = await page.evaluate(() => {
      const system = Canvas.buildSystemJson();
      const maps = Canvas.getCompBusMap();
      const branchTransformers = Canvas.state.components.filter(component =>
        component.type === 'transformer_2w' &&
        component.params?._from_branch === true);
      const tCardBranches = (system.ac?.branches || []).filter(branch =>
        String(branch.name || '').startsWith('T_'));
      return {
        system,
        mappedIds: Object.keys(maps.lcc || {}).sort(),
        branchDisplay: {
          lineCount: Canvas.state.components.filter(component =>
            component.type === 'ac_branch').length,
          transformerCount: branchTransformers.length,
          converterTransformerCount: branchTransformers.filter(component =>
            component.params?._converter_transformer === true).length,
          tCardBranches,
          standaloneTransformerCount:
            system.ac?.transformers_2w?.length || 0,
        },
        dcLineMetadata: (() => {
          const branch = system.dc?.branches?.[0] || {};
          const component = Canvas.state.components.find(item =>
            item.type === 'dc_branch');
          return {
            baseKv: Number(branch.base_kv),
            lengthKm: Number(branch.length_km),
            rOhmPerKm: Number(branch.r_ohm_per_km),
            totalOhm: Number(component?.params?.r_total_ohm),
          };
        })(),
        contractOk: system.lcc_converters.every(converter =>
          Number.isFinite(Number(converter.ac_bus)) &&
          Number.isFinite(Number(converter.dc_bus)) &&
          !Object.prototype.hasOwnProperty.call(converter, 'bus_ac') &&
          !Object.prototype.hasOwnProperty.call(converter, 'bus_dc')),
        controlMetadataOk:
          system.lcc_converters[0]?.external_control_code === 'PAAL' &&
          system.lcc_converters[1]?.external_control_code === 'VDGA',
      };
    });
    if (!canvasResult.contractOk || !canvasResult.controlMetadataOk ||
        !hasExpectedCigreTapMetadata(canvasResult.system.lcc_converters) ||
        JSON.stringify(canvasResult.mappedIds) !== JSON.stringify(['0', '1'])) {
      throw new Error('canvas LCC mapping, control metadata, or JSON contract is invalid');
    }
    const branchDisplay = canvasResult.branchDisplay;
    const tCardMetadataOk = branchDisplay.tCardBranches.length === 2 &&
      branchDisplay.tCardBranches.every(branch =>
        Math.abs(Number(branch.sn_mva) - 1800) < 1e-9 &&
        Math.abs(Number(branch.vn_hv_kv) - 525) < 1e-9 &&
        Math.abs(Number(branch.vn_lv_kv) - 217) < 1e-9);
    if (branchDisplay.lineCount !== 2 ||
        branchDisplay.transformerCount !== 2 ||
        branchDisplay.converterTransformerCount !== 2 ||
        branchDisplay.standaloneTransformerCount !== 0 ||
        !tCardMetadataOk) {
      throw new Error(
        `BPA T-card canvas mapping is invalid: ${JSON.stringify(branchDisplay)}`,
      );
    }
    const line = canvasResult.dcLineMetadata;
    if (Math.abs(line.baseKv - 500) > 1e-9 ||
        Math.abs(line.lengthKm - 1067) > 1e-9 ||
        Math.abs(line.rOhmPerKm - 10 / 1067) > 1e-12 ||
        Math.abs(line.totalOhm - 10) > 1e-9) {
      throw new Error(`DC line engineering resistance metadata is invalid: ${JSON.stringify(line)}`);
    }

    const propertyEditor = await page.evaluate(() => {
      const component = Canvas.state.components.find(item =>
        item.type === 'lcc_converter');
      if (!component) return null;
      Canvas.panToComponent(component.id);
      const mode = document.querySelector('#propFields [data-field="control_mode"]');
      const role = document.querySelector('#propFields [data-field="station_role"]');
      const external = document.querySelector('#propFields [data-field="external_control_code"]');
      const tap = document.querySelector('#propFields [data-field="tap_control_modelled"]');
      const lccEditor = {
        modeOptions: [...(mode?.options || [])].map(option => option.value),
        roleOptions: [...(role?.options || [])].map(option => option.value),
        externalReadOnly: external?.readOnly === true,
        tapDisabled: tap?.disabled === true,
      };
      const dcBranch = Canvas.state.components.find(item =>
        item.type === 'dc_branch');
      if (!dcBranch) return { ...lccEditor, dcResistance: null };
      Canvas.panToComponent(dcBranch.id);
      const totalResistance = document.querySelector(
        '#propFields [data-field="r_total_ohm"]');
      return {
        ...lccEditor,
        dcResistance: {
          value: Number(totalResistance?.value),
          readOnly: totalResistance?.readOnly === true,
        },
      };
    });
    if (!propertyEditor ||
        JSON.stringify(propertyEditor.modeOptions) !== JSON.stringify([
          'CONSTANT_POWER', 'CONSTANT_CURRENT', 'CONSTANT_ALPHA', 'CONSTANT_GAMMA',
        ]) ||
        JSON.stringify(propertyEditor.roleOptions) !==
          JSON.stringify(['RECTIFIER', 'INVERTER']) ||
        !propertyEditor.externalReadOnly || !propertyEditor.tapDisabled ||
        Math.abs(propertyEditor.dcResistance?.value - 10) > 1e-9 ||
        !propertyEditor.dcResistance?.readOnly) {
      throw new Error('LCC property editor modes or read-only metadata controls are invalid');
    }

    const roundTrip = await page.evaluate(async system => {
      const response = await fetch('/api/session/load_json_string', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ json_string: JSON.stringify(system) }),
      });
      const payload = await response.json().catch(() => ({}));
      let restored = {};
      try {
        restored = JSON.parse(payload._raw_json || '{}');
      } catch {
        restored = {};
      }
      return {
        ok: response.ok,
        status: response.status,
        error: payload.error || '',
        lcc: restored.lcc_converters || [],
        acBranches: restored.ac?.branches || [],
        standaloneTransformers: restored.ac?.transformers_2w || [],
      };
    }, canvasResult.system);
    const restoredContractOk = roundTrip.lcc.length === 2 &&
      roundTrip.lcc.every(converter =>
        Number.isFinite(Number(converter.ac_bus)) &&
        Number.isFinite(Number(converter.dc_bus)) &&
        !Object.prototype.hasOwnProperty.call(converter, 'bus_ac') &&
        !Object.prototype.hasOwnProperty.call(converter, 'bus_dc')) &&
      roundTrip.lcc[0].external_control_code === 'PAAL' &&
      roundTrip.lcc[1].external_control_code === 'VDGA';
    const restoredTransformerBranches = roundTrip.acBranches.filter(branch =>
      String(branch.name || '').startsWith('T_'));
    const restoredTransformerMetadataOk =
      restoredTransformerBranches.length === 2 &&
      restoredTransformerBranches.every(branch =>
        Math.abs(Number(branch.sn_mva) - 1800) < 1e-9 &&
        Math.abs(Number(branch.vn_hv_kv) - 525) < 1e-9 &&
        Math.abs(Number(branch.vn_lv_kv) - 217) < 1e-9);
    if (!roundTrip.ok || !restoredContractOk ||
        !hasExpectedCigreTapMetadata(roundTrip.lcc) ||
        !restoredTransformerMetadataOk ||
        roundTrip.standaloneTransformers.length !== 0) {
      throw new Error(
        `C++ JSON round-trip failed: HTTP ${roundTrip.status} ${roundTrip.error}`,
      );
    }

    const flowResult = await page.evaluate(async () => {
      const unit = document.getElementById('pfDisplayUnit');
      if (unit) unit.value = 'MW';
      const result = await App.runPowerFlow();
      Canvas.setVisualizationMode('flow');
      await new Promise(resolve => requestAnimationFrame(() =>
        requestAnimationFrame(resolve)));
      const labels = selector => [...document.querySelectorAll(selector)].map(label => ({
        text: label.textContent.trim(),
        index: Number(label.dataset.componentIndex),
        side: label.dataset.side,
        signedPowerMw: Number(label.dataset.signedPowerMw),
        direction: label.dataset.direction || '',
      }));
      return {
        converged: result?.converged === true,
        lccTransfers: result?.lcc_transfers || [],
        dcBranchFlows: result?.dc_branch_flows || [],
        geoAcBranches: result?.geo_ac_branches || [],
        componentResults: result?.component_results || [],
        lccLabels: labels('.flow-label[data-flow-kind=lcc-terminal]'),
        dcBranchLabels: labels('.flow-label[data-flow-kind=dc-branch-terminal]'),
        dcBranchLossLabels: [...document.querySelectorAll(
          '.flow-label[data-flow-kind=dc-branch-loss]')].map(label => ({
          text: label.textContent.trim(),
          index: Number(label.dataset.componentIndex),
          lossMw: Number(label.dataset.lossMw),
        })),
        branchTableHeaders: [...document.querySelectorAll(
          '#pfBranchResults thead th')].map(cell => cell.textContent.trim()),
        lccTable: {
          visible:
            document.getElementById('pfLccSection')?.style.display !== 'none',
          headers: [...document.querySelectorAll(
            '#pfLccResults thead th')].map(cell => cell.textContent.trim()),
          rows: [...document.querySelectorAll('#pfLccResults tbody tr')]
            .map(row => ({
              text: row.textContent.trim(),
              canvasType: row.dataset.canvasType || '',
              compId: Number(row.dataset.compId),
            })),
        },
      };
    });
    if (!flowResult.converged || flowResult.lccTransfers.length !== 2) {
      throw new Error('CIGRE power flow did not return two solved LCC transfers');
    }

    const expectedTapByIndex = new Map([
      [0, 547.97 / 525],
      [1, 571.25 / 525],
    ]);
    const transferByIndex = new Map(flowResult.lccTransfers.map(transfer =>
      [Number(transfer.index), transfer]));
    for (const [index, expectedTap] of expectedTapByIndex) {
      const transfer = transferByIndex.get(index);
      if (!transfer ||
          Math.abs(Number(transfer.transformer_tap) - expectedTap) > 2e-3 ||
          transfer.tap_control_active !== true ||
          transfer.tap_control_converged !== true ||
          transfer.tap_at_limit !== false ||
          Number(transfer.tap_control_iterations) <= 0) {
        throw new Error(
          'invalid R-card tap-control result for LCC ' + index + ': ' +
          JSON.stringify(transfer),
        );
      }
    }
    if (Math.abs(Number(transferByIndex.get(0)?.alpha_deg) - 15) > 0.1 ||
        Math.abs(Number(transferByIndex.get(1)?.gamma_deg) - 17) > 0.1) {
      throw new Error(
        'CIGRE LCC target angles were not recovered: ' +
        JSON.stringify(flowResult.lccTransfers),
      );
    }

    const lccComponentRows = flowResult.componentResults.filter(row =>
      row.canvas_type === 'lcc_converter');
    const requiredLccMetrics = [
      '角色', '模式', 'Pac', 'Qac', 'Pdc', 'Alpha', 'Gamma', 'Tap',
      'Tap控制active', 'Tap控制converged', 'Tap达到限值', 'Tap控制迭代',
    ];
    if (lccComponentRows.length !== 2) {
      throw new Error(
        'expected two LCC component-result rows, got ' +
        lccComponentRows.length,
      );
    }
    for (const row of lccComponentRows) {
      const transfer = transferByIndex.get(Number(row.index));
      const metrics = new Map((row.metrics || []).map(metric =>
        [metric.label, metric.value]));
      if (!transfer ||
          row.type_label !== 'LCC换流器' ||
          !requiredLccMetrics.every(label => metrics.has(label)) ||
          !['Pac', 'Qac', 'Pdc', 'Alpha', 'Gamma', 'Tap']
            .every(label => Number.isFinite(Number(metrics.get(label)))) ||
          Math.abs(Number(metrics.get('Tap')) -
            Number(transfer.transformer_tap)) > 1e-6 ||
          metrics.get('Tap控制active') !== 'true' ||
          metrics.get('Tap控制converged') !== 'true' ||
          metrics.get('Tap达到限值') !== 'false' ||
          Number(metrics.get('Tap控制迭代')) <= 0) {
        throw new Error(
          'incomplete LCC component result: ' + JSON.stringify(row),
        );
      }
    }

    const converterByTransformerBranch = new Map(
      canvasResult.system.lcc_converters.map((converter, position) => [
        Number(converter.converter_transformer_branch),
        Number(converter.index ?? position),
      ]),
    );
    const geoTransformerRows = flowResult.geoAcBranches.filter(branch =>
      converterByTransformerBranch.has(Number(branch.index)));
    if (geoTransformerRows.length !== 2) {
      throw new Error(
        'expected two converter-transformer branch results, got ' +
        geoTransformerRows.length,
      );
    }
    for (const branch of geoTransformerRows) {
      const converterIndex =
        converterByTransformerBranch.get(Number(branch.index));
      const expectedTap = expectedTapByIndex.get(converterIndex);
      if (branch.is_transformer !== true ||
          branch.component_type !== 'transformer_2w' ||
          branch.type_label !== '双绕组变压器' ||
          Math.abs(Number(branch.transformer_tap) - expectedTap) > 2e-3) {
        throw new Error(
          'invalid converter-transformer presentation row: ' +
          JSON.stringify(branch),
        );
      }
    }
    const branchComponentRows = flowResult.componentResults.filter(row =>
      row.canvas_type === 'ac_branch' &&
      converterByTransformerBranch.has(Number(row.index)));
    if (branchComponentRows.length !== 2 ||
        branchComponentRows.some(row => {
          const tapMetric = (row.metrics || []).find(metric =>
            metric.label === 'Tap');
          const converterIndex =
            converterByTransformerBranch.get(Number(row.index));
          return row.type_label !== '双绕组变压器' ||
            !tapMetric ||
            Math.abs(Number(tapMetric.value) -
              expectedTapByIndex.get(converterIndex)) > 2e-3;
        })) {
      throw new Error(
        'converter-transformer component rows are invalid: ' +
        JSON.stringify(branchComponentRows),
      );
    }

    const lccHeaderText = flowResult.lccTable.headers.join('|');
    if (!flowResult.lccTable.visible ||
        flowResult.lccTable.rows.length !== 2 ||
        !['Pac', 'Qac', 'Pdc', 'Alpha', 'Gamma', 'Tap', '控制状态']
          .every(label => lccHeaderText.includes(label)) ||
        flowResult.lccTable.rows.some(row =>
          row.canvasType !== 'lcc_converter' ||
          !Number.isInteger(row.compId))) {
      throw new Error(
        'LCC result table is incomplete: ' +
        JSON.stringify(flowResult.lccTable),
      );
    }
    if (!flowResult.branchTableHeaders.includes('类型') ||
        !flowResult.branchTableHeaders.some(header => header.startsWith('Tap'))) {
      throw new Error(
        'AC branch table does not expose transformer type/tap: ' +
        JSON.stringify(flowResult.branchTableHeaders),
      );
    }

    const lccExpected = new Map([
      ['0:AC', { magnitude: 1500, direction: 'bus-to-converter' }],
      ['0:DC', { magnitude: 1500, direction: 'converter-to-bus' }],
      ['1:AC', { magnitude: 1410, direction: 'converter-to-bus' }],
      ['1:DC', { magnitude: 1410, direction: 'bus-to-converter' }],
    ]);
    if (flowResult.lccLabels.length !== lccExpected.size) {
      throw new Error(`expected four LCC terminal labels, got ${flowResult.lccLabels.length}`);
    }
    for (const label of flowResult.lccLabels) {
      const expected = lccExpected.get(`${label.index}:${label.side}`);
      if (!expected ||
          Math.abs(Math.abs(label.signedPowerMw) - expected.magnitude) > 1 ||
          label.direction !== expected.direction ||
          label.text !== `${expected.magnitude.toFixed(1)} MW`) {
        throw new Error(`invalid LCC terminal flow label: ${JSON.stringify(label)}`);
      }
    }

    const dcFlow = flowResult.dcBranchFlows[0] || {};
    const dcFrom = Math.abs(Number(dcFlow.pf_mw));
    const dcTo = Math.abs(Number(dcFlow.pt_mw));
    if (flowResult.dcBranchFlows.length !== 1 ||
        Math.abs(dcFrom - 1500) > 1 || Math.abs(dcTo - 1410) > 1 ||
        Math.abs((dcFrom - dcTo) - 90) > 1) {
      throw new Error(`CIGRE DC branch no longer preserves 1500 -> 1410 MW: ${JSON.stringify(dcFlow)}`);
    }
    const dcLabelMagnitudes = flowResult.dcBranchLabels
      .map(label => Math.abs(label.signedPowerMw)).sort((a, b) => b - a);
    const dcLabelTexts = flowResult.dcBranchLabels.map(label => label.text).sort();
    if (dcLabelMagnitudes.length !== 2 ||
        Math.abs(dcLabelMagnitudes[0] - 1500) > 1 ||
        Math.abs(dcLabelMagnitudes[1] - 1410) > 1 ||
        JSON.stringify(dcLabelTexts) !== JSON.stringify(['1410.0 MW', '1500.0 MW'])) {
      throw new Error(`invalid DC branch terminal labels: ${JSON.stringify(flowResult.dcBranchLabels)}`);
    }
    if (flowResult.dcBranchLossLabels.length !== 1 ||
        Math.abs(flowResult.dcBranchLossLabels[0].lossMw - 90) > 1 ||
        flowResult.dcBranchLossLabels[0].text !== '损耗 90.0 MW') {
      throw new Error(`invalid DC branch loss label: ${JSON.stringify(flowResult.dcBranchLossLabels)}`);
    }
    if (pageErrors.length) {
      throw new Error(`page errors: ${pageErrors.join('; ')}`);
    }

    console.log(JSON.stringify({
      schema: 'gui_lcc_dat_roundtrip_e2e_result',
      imported_lcc: canvasResult.system.lcc_converters.length,
      mapped_ids: canvasResult.mappedIds,
      cpp_roundtrip_lcc: roundTrip.lcc.length,
      converter_transformers: branchDisplay.converterTransformerCount,
      json_contract: 'ac_bus/dc_bus',
      lcc_terminal_labels: flowResult.lccLabels,
      dc_branch_terminal_labels: flowResult.dcBranchLabels,
      dc_branch_loss_labels: flowResult.dcBranchLossLabels,
      dc_branch_loss_mw: dcFrom - dcTo,
      dc_branch_total_resistance_ohm: canvasResult.dcLineMetadata.totalOhm,
    }));
    return 0;
  } finally {
    await browser?.close();
    serverProcess?.kill();
  }
}

process.exitCode = await main();
