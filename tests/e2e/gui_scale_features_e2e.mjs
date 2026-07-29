// @ts-check
// gui_scale_features_e2e.mjs — browser end-to-end test for the large-system /
// scalability GUI features:
//   * WebGL large-system mode + automatic return to the SVG editor
//   * calculations (PF / OPF) run headless against the backend session
//   * virtualized, editable topology tables (no row cap; inline edit -> sync)
//   * global element search (canvas pan + headless row highlight, any position)
//   * time-series split: 时序潮流 (tspf) in 安全与动态, 时序生产模拟 in 规划与运行
//   * on-demand selectable local SVG (leaves full-model state untouched)
//   * WebGL full-network overview with zero large SVG glyphs
//   * minimap (SVG mode) hidden while the WebGL overview is active
//   * dependency chips reflect PF / canvas state
//
// Requirements (not part of the C++ build):
//   npm i -D playwright && npx playwright install chromium
//
// Usage:
//   node tests/e2e/gui_scale_features_e2e.mjs \
//        [--server build/macos-release/run_gui_server] [--data-dir data]
//        [--port 0] [--browser-channel msedge]
//
// Exit code 0 on success, 1 on any failed assertion. Starts (and stops) the GUI
// server itself unless --base-url points at an already-running instance.

import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const REPO_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');

function arg(name, def) {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && i + 1 < process.argv.length ? process.argv[i + 1] : def;
}

function findServer() {
  const explicit = arg('server', null);
  if (explicit) return path.resolve(explicit);
  for (const c of [
    'build/macos-release/run_gui_server',
    'build/tests/run_gui_server',
    'build/tests/run_gui_server.exe',
    'build_rel/tests/run_gui_server',
  ]) {
    const p = path.join(REPO_ROOT, c);
    if (existsSync(p)) return p;
  }
  throw new Error('run_gui_server not found; build it or pass --server');
}

function freePort() {
  return new Promise((resolve, reject) => {
    const s = createServer();
    s.listen(0, '127.0.0.1', () => {
      const p = s.address().port;
      s.close(() => resolve(p));
    });
    s.on('error', reject);
  });
}

async function waitUp(base, timeoutMs = 20000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    try {
      const r = await fetch(base + '/api/cases');
      if (r.ok) return true;
    } catch { /* not up yet */ }
    await new Promise((r) => setTimeout(r, 250));
  }
  return false;
}

// A large bundled MATPOWER case that exceeds the headless threshold.
const LARGE_CASE = 'case2869pegase.m';
const MEDIUM_CASE = 'case300_acdc';
const SMALL_CASE = 'ieee14_acdc';
const CIM_TEST_XML = path.join(REPO_ROOT, 'data', 'test.xml');

async function main() {
  let chromium;
  try {
    ({ chromium } = await import('playwright'));
  } catch {
    console.error('playwright not installed. Run: npm i -D playwright && npx playwright install chromium');
    return 2;
  }

  const baseUrl = arg('base-url', null);
  const dataDir = arg('data-dir', path.join(REPO_ROOT, 'data'));
  let proc = null;
  let base = baseUrl;

  if (!baseUrl) {
    const port = Number(arg('port', 0)) || (await freePort());
    base = `http://127.0.0.1:${port}`;
    const server = findServer();
    proc = spawn(server, ['--port', String(port), '--data-dir', dataDir, '--matpower-dir', dataDir],
      { cwd: REPO_ROOT, stdio: 'ignore' });
    if (!(await waitUp(base))) {
      console.error('server did not come up');
      proc.kill();
      return 1;
    }
  }

  let failures = 0;
  const check = (cond, msg) => {
    console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${msg}`);
    if (!cond) failures++;
  };

  const browserChannel = arg('browser-channel', null);
  const browser = await chromium.launch(
    browserChannel ? { channel: browserChannel } : {},
  );
  try {
    const page = await browser.newPage();
    page.on('pageerror', (e) => { console.error('  [pageerror]', e.message); failures++; });
    // The GUI's optional Plotly CDN dependency is unavailable in offline CI.
    // These scalability checks do not inspect chart pixels, but some workflows
    // still refresh charts and should remain free of page errors.
    await page.route('https://cdn.plot.ly/**', (route) => route.fulfill({
      contentType: 'text/javascript',
      body: 'window.Plotly={newPlot:()=>Promise.resolve(),react:()=>Promise.resolve(),Plots:{resize:()=>{}}};',
    }));
    await page.goto(base + '/xjtu/', { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() =>
      typeof App !== 'undefined' && typeof Canvas !== 'undefined');
    await page.waitForTimeout(400);
    const brand = await page.locator('.logo-mini').textContent();
    check(await page.title() === 'HySim-XJTU-HRPES',
      'browser title uses the HySim-XJTU-HRPES brand');
    check((brand || '').replace(/\s+/g, '') === 'HySimXJTU-HRPES',
      'top toolbar displays the HySim-XJTU-HRPES brand');
    const initialTask = await page.evaluate(() => App.getTaskStatus());
    check(initialTask?.schema === 'hysim_task_status_v1' &&
          Number.isInteger(initialTask.model_revision),
      'unified task-status contract is available');
    const coreModules = await page.evaluate(() => {
      const maps = { trafo: { 7: 91 } };
      const hours = Array.from({ length: 8760 }, (_, index) => index);
      const load = Array.from({ length: 8760 }, (_, index) => Math.sin(index / 24));
      load[4321] = 999;
      load[6000] = -999;
      const annualWindow = HySimCore.TimeSeriesWindow.build({
        x: hours,
        series: { load },
        primary: load,
        start: 0,
        size: hours.length,
        maxPoints: 2000,
      });
      return {
        contracts: HySimCore.AnalysisContracts?.schema,
        taskManager: typeof HySimCore.AnalysisTaskManager,
        apiClient: typeof HySimCore.ApiClient,
        mapping: HySimCore.ResultMapping?.schema,
        accessibility: HySimCore.Accessibility?.schema,
        runtimeDiagnostics: HySimCore.RuntimeDiagnostics?.schema,
        mappedTransformer: HySimCore.ResultMapping?.resolve(
          { canvas_type: 'transformer_2w', canvas_index: 7 }, maps),
        annualWindow: {
          schema: annualWindow.schema,
          rendered: annualWindow.rendered,
          first: annualWindow.indices[0],
          last: annualWindow.indices.at(-1),
          hasPeak: annualWindow.indices.includes(4321),
          hasTrough: annualWindow.indices.includes(6000),
        },
      };
    });
    check(coreModules.contracts === 'hysim_analysis_contracts_v1' &&
          coreModules.taskManager === 'function' && coreModules.apiClient === 'function' &&
          coreModules.mapping === 'hysim_canvas_ref_v1' && coreModules.mappedTransformer === 91 &&
          coreModules.accessibility === 'hysim_accessibility_v1' &&
          coreModules.runtimeDiagnostics === 'hysim_runtime_diagnostics_v1',
      'modular core contracts load before App and resolve Canvas references');
    check(coreModules.annualWindow.schema === 'hysim_timeseries_window_v1' &&
          coreModules.annualWindow.rendered <= 2000 &&
          coreModules.annualWindow.first === 0 && coreModules.annualWindow.last === 8759 &&
          coreModules.annualWindow.hasPeak && coreModules.annualWindow.hasTrough,
      `annual window preserves endpoints and extrema in ${coreModules.annualWindow.rendered} rendered points`);
    const cancellationScheduler = await page.evaluate(async () => {
      let cancelRequests = 0;
      let idleChecks = 0;
      const manager = new HySimCore.AnalysisTaskManager({
        cancelBackend: async () => { cancelRequests += 1; },
        waitForIdle: async () => {
          idleChecks += 1;
          return idleChecks >= 2;
        },
        settlementRetryMs: 10,
      });
      const first = manager.start({
        path: '/api/session/pf', requestId: 'cancel-first',
        modelRevision: 1, analysis: 'power_flow',
      });
      const firstCancelAccepted = manager.cancel();
      const duplicateCancelRejected = manager.cancel() === false;
      manager.finish(first);
      const deadline = Date.now() + 2000;
      while (manager.settling && Date.now() < deadline) {
        await new Promise(resolve => setTimeout(resolve, 10));
      }
      const second = manager.start({
        path: '/api/session/opf', requestId: 'cancel-second',
        modelRevision: 1, analysis: 'optimal_power_flow',
      });
      return {
        cancelRequests, idleChecks, settling: manager.settling,
        firstCancelAccepted, duplicateCancelRejected,
        secondAccepted: second.duplicate === false,
      };
    });
    check(cancellationScheduler.firstCancelAccepted &&
          cancellationScheduler.duplicateCancelRejected &&
          cancellationScheduler.cancelRequests === 1 &&
          cancellationScheduler.idleChecks >= 2 &&
          cancellationScheduler.settling === false &&
          cancellationScheduler.secondAccepted,
      'cancelled task polls through backend settlement and releases the next task');
    const resilienceLabel = await page.locator('#moduleResilience').textContent();
    check(resilienceLabel?.trim() === '弹性分析',
      'resilience module uses the 弹性分析 label');

    // ---- Phase 5: semantic navigation, keyboard operation, and focus ----
    const accessibilityAudit = await page.evaluate(() => App.getAccessibilityAudit());
    check(accessibilityAudit?.schema === 'hysim_accessibility_audit_v1' &&
          accessibilityAudit.critical === 0 && accessibilityAudit.managed_tablists >= 3,
      `accessibility audit passes with ${accessibilityAudit?.managed_tablists} managed tablists`);
    await page.locator('.workflow-btn.active').focus();
    await page.keyboard.press('ArrowRight');
    await page.waitForTimeout(50);
    const workflowKeyboard = await page.evaluate(() => ({
      workflow: document.querySelector('.workflow-btn.active')?.dataset.workflow,
      module: document.querySelector('.module-btn.active')?.dataset.module,
      tabStops: document.querySelectorAll('#workflowBar .workflow-btn[tabindex="0"]').length,
      selected: document.querySelector('#workflowBar .workflow-btn[aria-selected="true"]')?.dataset.workflow,
    }));
    check(workflowKeyboard.workflow === 'security' && workflowKeyboard.module === 'shortCircuit' &&
          workflowKeyboard.tabStops === 1 && workflowKeyboard.selected === 'security',
      'ArrowRight switches workflow and keeps one semantic tab stop');
    await page.locator('.module-btn.active').focus();
    await page.keyboard.press('ArrowRight');
    await page.waitForTimeout(50);
    check(await page.evaluate(() => document.querySelector('.module-btn.active')?.dataset.module) === 'transient',
      'ArrowRight switches to the next visible module');
    await page.locator('.skip-link').focus();
    await page.keyboard.press('Enter');
    await page.waitForTimeout(50);
    check(await page.evaluate(() => document.activeElement?.id) === 'main',
      'skip link moves keyboard focus to the main workspace');
    await page.evaluate(() => App.setActiveModule('powerFlow'));

    // ---- Phase 6: bounded runtime diagnostics and recoverable health chip ----
    const runtimeGuard = await page.evaluate(async () => {
      window.dispatchEvent(new ErrorEvent('error', {
        message: 'phase6 synthetic runtime failure',
        filename: 'gui_scale_features_e2e.mjs',
      }));
      const degraded = App.getRuntimeDiagnostics();
      const degradedChip = {
        state: document.getElementById('runtimeHealthChip')?.dataset.runtimeStatus,
        text: document.getElementById('runtimeHealthChip')?.textContent || '',
      };
      const cleared = App.clearRuntimeDiagnostics();
      const networkClient = new HySimCore.ApiClient({
        baseUrl: '',
        contracts: {
          analysisForPath: () => 'test',
          attachResultContract: () => false,
          labelForAnalysis: () => 'test',
        },
        tasks: {
          start: () => ({ duplicate: false, signal: new AbortController().signal }),
          finish: () => {},
        },
        getModelRevision: () => 0,
        fetchImpl: async () => { throw new Error('phase6 synthetic network failure'); },
        onError: (type, error, context) => HySimCore.RuntimeDiagnostics.capture(type, error, {
          source: context.path, quiet: true,
        }),
      });
      await networkClient.post('/phase6-network-test', {}, { quiet: true });
      const networkFailure = App.getRuntimeDiagnostics();
      App.clearRuntimeDiagnostics();
      return {
        degraded, degradedChip, cleared, networkFailure,
        clearedChip: document.getElementById('runtimeHealthChip')?.dataset.runtimeStatus,
      };
    });
    check(runtimeGuard.degraded?.schema === 'hysim_runtime_diagnostics_v1' &&
          runtimeGuard.degraded.status === 'degraded' && runtimeGuard.degraded.total_errors === 1 &&
          runtimeGuard.degraded.retained_errors === 1 && runtimeGuard.degraded.max_retained_errors === 25 &&
          runtimeGuard.degradedChip.state === 'degraded' && /1错误/.test(runtimeGuard.degradedChip.text),
      'runtime guard captures a frontend error in its bounded diagnostics buffer');
    check(runtimeGuard.cleared?.status === 'healthy' && runtimeGuard.cleared?.total_errors === 0 &&
          runtimeGuard.clearedChip === 'healthy',
      'runtime diagnostics can be cleared back to a healthy state');
    check(runtimeGuard.networkFailure?.network_errors === 1 &&
          runtimeGuard.networkFailure?.last_error?.type === 'network' &&
          runtimeGuard.networkFailure?.last_error?.source === '/phase6-network-test',
      'ApiClient network failures flow into the runtime diagnostics contract');

    // ---- Model IO: binary-safe BPA/DSP DAT upload and DAT download. ----
    await page.evaluate(() => App.setActiveModule('modelIO'));
    const bpaImportResponse = page.waitForResponse(response =>
      response.url().includes('/api/session/load_bpa_dat') && response.status() === 200);
    await page.locator('#fileImportBpaDat').setInputFiles(path.join(REPO_ROOT, 'data', 'dsp', '39.dat'));
    await bpaImportResponse;
    await page.waitForFunction(() => Canvas.getSystemSummary().buses === 39);
    const bpaImport = await page.evaluate(() => ({
      buses: Canvas.getSystemSummary().buses,
      importButton: !!document.getElementById('btnIoImportBpaDat'),
      exportButton: !!document.getElementById('btnIoExportBpaDat'),
    }));
    check(bpaImport.buses === 39 && bpaImport.importButton && bpaImport.exportButton,
      'model IO imports a BPA/DSP DAT file through the binary-safe frontend path');

    const bpaDownloadPromise = page.waitForEvent('download');
    await page.locator('#btnIoExportBpaDat').click();
    const bpaDownload = await bpaDownloadPromise;
    const bpaDownloadPath = await bpaDownload.path();
    const bpaExportText = bpaDownloadPath ? readFileSync(bpaDownloadPath, 'utf8') : '';
    check(bpaDownload.suggestedFilename().endsWith('.dat') &&
          bpaExportText.includes('A0000001') && bpaExportText.includes('(END)'),
      'model IO downloads a fixed-column BPA/DSP DAT export');

    const lccImportResponse = page.waitForResponse(response =>
      response.url().includes('/api/session/load_bpa_dat') && response.status() === 200);
    await page.locator('#fileImportBpaDat').setInputFiles(path.join(REPO_ROOT, 'data', 'dsp', 'cigre.dat'));
    await lccImportResponse;
    await page.waitForFunction(() => Canvas.buildSystemJson().lcc_converters?.length === 2);
    const lccImport = await page.evaluate(() => {
      const maps = Canvas.getCompBusMap();
      const system = Canvas.buildSystemJson();
      return {
        converters: system.lcc_converters?.length || 0,
        mappedIds: Object.keys(maps.lcc || {}).sort(),
        jsonContractOk: (system.lcc_converters || []).every(lcc =>
          Number.isFinite(Number(lcc.ac_bus)) &&
          Number.isFinite(Number(lcc.dc_bus)) &&
          !Object.prototype.hasOwnProperty.call(lcc, 'bus_ac') &&
          !Object.prototype.hasOwnProperty.call(lcc, 'bus_dc')),
        controlMetadataOk:
          system.lcc_converters?.[0]?.external_control_code === 'PAAL' &&
          system.lcc_converters?.[1]?.external_control_code === 'VDGA' &&
          system.lcc_converters.every(lcc =>
            lcc.tap_control_modelled === false),
      };
    });
    check(lccImport.converters === 2 &&
          JSON.stringify(lccImport.mappedIds) === JSON.stringify(['0', '1']) &&
          lccImport.jsonContractOk && lccImport.controlMetadataOk,
      'model IO maps imported BPA/DSP LCC converters and preserves the C++ JSON contract');

    const lccRoundTrip = await page.evaluate(async () => {
      const system = Canvas.buildSystemJson();
      const response = await fetch(window.location.origin + '/api/session/load_json_string', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ json_string: JSON.stringify(system) }),
      });
      const payload = await response.json().catch(() => ({}));
      let restored = {};
      try {
        restored = JSON.parse(payload._raw_json || '{}');
      } catch (_) {
        restored = {};
      }
      return {
        ok: response.ok,
        status: response.status,
        error: payload.error || '',
        converters: restored.lcc_converters?.length || 0,
        jsonContractOk: (restored.lcc_converters || []).every(lcc =>
          Number.isFinite(Number(lcc.ac_bus)) &&
          Number.isFinite(Number(lcc.dc_bus)) &&
          !Object.prototype.hasOwnProperty.call(lcc, 'bus_ac') &&
          !Object.prototype.hasOwnProperty.call(lcc, 'bus_dc')),
        controlMetadataOk:
          restored.lcc_converters?.[0]?.external_control_code === 'PAAL' &&
          restored.lcc_converters?.[1]?.external_control_code === 'VDGA' &&
          restored.lcc_converters.every(lcc =>
            lcc.tap_control_modelled === false),
      };
    });
    check(lccRoundTrip.ok && lccRoundTrip.converters === 2 &&
          lccRoundTrip.jsonContractOk && lccRoundTrip.controlMetadataOk,
      'C++ accepts canvas-rebuilt LCC JSON and preserves both converters ' +
      '(HTTP ' + lccRoundTrip.status + ': ' + (lccRoundTrip.error || 'ok') + ')');

    // ---- 1) Large case -> WebGL overview, PF + OPF run against the backend ----
    const big = await page.evaluate(async (caseName) => {
      await App.loadMatpowerCase(caseName);
      const summary = Canvas.getSystemSummary();
      const opf = await fetch(window.location.origin + '/api/session/opf', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ solver: 'dc', constraints: {} }),
      }).then((r) => r.json()).catch((e) => ({ error: String(e) }));
      const overview = Canvas.getNetworkOverviewStats();
      const webglCanvas = document.getElementById('networkOverviewCanvas');
      const gl = webglCanvas?.getContext('webgl2');
      let nonBlankSamples = 0;
      if (gl && webglCanvas.width && webglCanvas.height) {
        const pixel = new Uint8Array(4);
        const background = new Uint8Array(4);
        gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, background);
        for (let gy = 1; gy < 16; gy += 1) {
          for (let gx = 1; gx < 24; gx += 1) {
            gl.readPixels(Math.floor(webglCanvas.width * gx / 24),
              Math.floor(webglCanvas.height * gy / 16), 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, pixel);
            if (pixel[0] !== background[0] || pixel[1] !== background[1] ||
                pixel[2] !== background[2]) nonBlankSamples += 1;
          }
        }
      }
      return {
        headless: Canvas.isHeadless(),
        glyphs: Canvas.state.components.length,
        buses: summary.buses,
        canvasChip: document.getElementById('depChipCanvas')?.textContent || '',
        pfChip: document.getElementById('depChipPf')?.textContent || '',
        opfConverged: !!(opf && opf.converged),
        minimapHidden: document.getElementById('minimap')?.hidden,
        overview,
        overviewVisible: !document.getElementById('networkOverview')?.hidden,
        nonBlankSamples,
      };
    }, LARGE_CASE);
    check(big.headless === true, `large case (${LARGE_CASE}) enters large-system overview mode`);
    check(big.glyphs === 0, 'WebGL overview creates zero SVG component glyphs');
    check(big.buses > 600, `system summary reports ${big.buses} buses`);
    check(/WebGL/.test(big.canvasChip), `view chip shows WebGL: "${big.canvasChip}"`);
    check(big.overviewVisible && big.overview?.webgl2 && big.overview.nodes > 600,
      `WebGL overview renders ${big.overview?.nodes || 0} nodes`);
    check(big.nonBlankSamples > 0, `WebGL canvas has ${big.nonBlankSamples} non-background pixel samples`);
    check(/收敛/.test(big.pfChip), `PF auto-ran and chip shows converged: "${big.pfChip}"`);
    check(big.opfConverged === true, 'OPF converges headless against the backend session');
    check(big.minimapHidden === true, 'SVG minimap hidden in WebGL overview mode');

    // ---- 2) Virtualized tables (no cap) + high-position search highlight ----
    const virt = await page.evaluate(async () => {
      App.switchTab('topology');
      await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));
      const busTable = document.getElementById('busTableInner');
      const total = busTable.__vctx?.items?.length || 0;
      const rendered = busTable.querySelectorAll('tbody tr[data-row]').length;
      // High-index search highlights the row regardless of the (removed) cap.
      document.getElementById('globalElementSearch').value = 'bus 2500';
      document.getElementById('btnGlobalElementSearch').click();
      await new Promise((r) => setTimeout(r, 250));
      const hl = document.querySelector('.topo-search-highlight');
      return { total, rendered, highlightRow: hl?.closest('tr')?.dataset.row };
    });
    check(virt.total > 2000, `bus table holds all ${virt.total} rows (no cap)`);
    check(virt.rendered > 0 && virt.rendered < 60, `only ~${virt.rendered} rows are materialized (windowed)`);
    check(virt.highlightRow === '2499', 'search highlights bus 2500 (row 2499) beyond the old cap');

    const overviewSelection = await page.evaluate(() => {
      const selected = App.selectStableRef({ domain: 'ac', index: 100 });
      const stats = Canvas.getNetworkOverviewStats();
      const highlighted = document.querySelector('.topo-search-highlight');
      return { selected, ref: stats?.selected, highlighted: highlighted?.closest('tr')?.dataset.row };
    });
    check(overviewSelection.selected && overviewSelection.ref?.domain === 'ac' &&
          overviewSelection.ref?.index === 100 && overviewSelection.highlighted === '99',
      'stable AC bus selection synchronizes WebGL overview and virtual topology table');

    // ---- 3) Inline edit in headless mode commits to the backend ----
    const edit = await page.evaluate(async () => {
      const genTable = document.getElementById('genTableInner');
      const before = Canvas.buildSystemJson().ac.generators[0].pg_mw;
      const cell = genTable.querySelector('tbody tr[data-row="0"] td[data-edit-field="pg_mw"]');
      cell.dispatchEvent(new MouseEvent('dblclick', { bubbles: true }));
      const input = cell.querySelector('input.topo-cell-input');
      input.value = '137.5';
      input.dispatchEvent(new KeyboardEvent('keydown', { key: 'Enter', bubbles: true }));
      await new Promise((r) => setTimeout(r, 1200));
      return { before, after: Canvas.buildSystemJson().ac.generators[0].pg_mw };
    });
    check(edit.before !== 137.5 && edit.after === 137.5, `inline edit applied gen pg_mw ${edit.before} -> ${edit.after}`);

    // ---- 4) Local SVG is bounded/selectable and leaves full-model state untouched ----
    const sub = await page.evaluate(async () => {
      document.getElementById('globalElementSearch').value = 'bus 100';
      document.getElementById('subDiagramHops').value = '2';
      document.getElementById('btnNeighborhoodDiagram').click();
      await new Promise((r) => setTimeout(r, 250));
      const svg = document.getElementById('subDiagramSvg');
      const out = {
        visible: getComputedStyle(document.getElementById('subDiagramModal')).display,
        nodes: svg.querySelectorAll('circle').length,
        edges: svg.querySelectorAll('line').length,
        stillHeadless: Canvas.isHeadless(),
        glyphs: Canvas.state.components.length,
      };
      document.getElementById('btnSubDiagramClose').click();
      return out;
    });
    check(sub.visible === 'flex', 'sub-diagram modal opens');
    check(sub.nodes >= 3 && sub.edges >= 2, `sub-diagram drew ${sub.nodes} nodes / ${sub.edges} branches`);
    check(await page.locator('#subDiagramSvg .subdiag-node').count() === sub.nodes,
      'local SVG exposes stable selectable node marks');
    check(sub.stillHeadless === true && sub.glyphs === 0, 'sub-diagram leaves headless state untouched');

    // ---- 4a) A rendered import must retain authored physical names when the
    // canvas rebuilds the system JSON for a backend sync.
    const cimNames = await page.evaluate(() => {
      const system = {
        name: '物理名称往返', base_mva: 10,
        ac: {
          buses: [
            { index: 1, name: '站内一段母线', bus_type: 'SLACK', base_kv: 10 },
            { index: 2, name: '人民路环网节点', bus_type: 'PQ', base_kv: 10 },
            { index: 3, name: '台区低压母线', bus_type: 'PQ', base_kv: 0.4 },
          ],
          branches: [
            { index: 11, name: '人民路一回线', from_bus: 1, to_bus: 2,
              r_pu: 0.01, x_pu: 0.02, b_pu: 0, rate_a_mva: 5, in_service: true },
          ],
          transformers_2w: [
            { index: 21, name: '人民路#1配变', hv_bus: 2, lv_bus: 3,
              sn_mva: 0.4, vn_hv_kv: 10, vn_lv_kv: 0.4,
              vk_percent: 4, vkr_percent: 1, in_service: true },
          ],
          switches: [
            { index: 31, name: '人民路01T01刀闸', bus_from: 1, bus_to: 2,
              switch_type: 'Disconnector', closed: true, in_service: true },
          ],
          circuit_breakers: [
            { index: 41, name: '人民路进线断路器', bus_from: 1, bus_to: 2,
              closed: true, in_service: true },
          ],
          loads: [{ index: 51, name: '人民路居民负荷', bus: 3, p_mw: 0.2, q_mvar: 0.05 }],
          generators: [{ index: 61, name: '站内备用电源', bus: 1, pg_mw: 0, qg_mvar: 0 }],
        },
        dc: { buses: [], branches: [], loads: [] },
      };
      Canvas.loadFromSystemJson(system, { forceRender: true });
      const roundTrip = Canvas.buildSystemJson();
      return {
        buses: roundTrip.ac.buses.map((item) => item.name),
        branches: roundTrip.ac.branches.map((item) => item.name),
        transformers: roundTrip.ac.transformers_2w.map((item) => item.name),
        switches: roundTrip.ac.switches.map((item) => item.name),
        breakers: roundTrip.ac.circuit_breakers.map((item) => item.name),
        loads: roundTrip.ac.loads.map((item) => item.name),
        generators: roundTrip.ac.generators.map((item) => item.name),
      };
    });
    check(cimNames.buses.join('|') === '站内一段母线|人民路环网节点|台区低压母线',
      'CIM bus physical names survive canvas round-trip');
    check(cimNames.branches.includes('人民路一回线'),
      'CIM line physical name survives canvas round-trip');
    check(cimNames.transformers.includes('人民路#1配变'),
      'CIM transformer physical name survives canvas round-trip');
    check(cimNames.switches.includes('人民路01T01刀闸'),
      'CIM switch physical name survives canvas round-trip');
    check(cimNames.breakers.includes('人民路进线断路器'),
      'CIM breaker physical name survives canvas round-trip');
    check(cimNames.loads.includes('人民路居民负荷') &&
          cimNames.generators.includes('站内备用电源'),
      'CIM load and source physical names survive canvas round-trip');

    // ---- 4b) The 439-bus distribution CIM fixture must stay out of the full
    // SVG path. It previously rendered ~919 glyphs/connections and could freeze
    // the browser before viewport culling had a chance to run.
    const cimXml = readFileSync(CIM_TEST_XML, 'utf8');
    const cimScale = await page.evaluate(async (xmlString) => {
      const loaded = await fetch(window.location.origin + '/api/session/load_cim_dist', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ xml_string: xmlString }),
      }).then((response) => response.json());
      if (loaded.error) return { error: loaded.error };
      Canvas.loadFromSystemJson(JSON.parse(loaded._raw_json));
      const summary = Canvas.getSystemSummary();
      return {
        headless: Canvas.isHeadless(),
        glyphs: Canvas.state.components.length,
        buses: summary.buses,
        totalElements: summary.totalElements,
      };
    }, cimXml);
    check(!cimScale.error && cimScale.buses === 439,
      `test.xml imports ${cimScale.buses} buses without a frontend error`);
    check(cimScale.headless === true && cimScale.glyphs === 0,
      `test.xml enters WebGL overview mode (${cimScale.totalElements} estimated elements)`);

    // ---- 5) Medium case uses viewport culling and RAF-batched pointer work ----
    const medium = await page.evaluate(async (caseName) => {
      await App.loadBuiltinCase(caseName);
      const t0 = Date.now();
      while (Date.now() - t0 < 20000 && Canvas.state.components.length <= 350) {
        await new Promise((resolve) => setTimeout(resolve, 150));
      }
      await Canvas.waitForLayout();
      for (let index = 0; index < 6; index += 1) Canvas.zoomIn();
      await new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      const before = Canvas.getPerformanceStats();
      const canvas = document.getElementById('canvas');
      for (let index = 0; index < 50; index += 1) {
        canvas.dispatchEvent(new MouseEvent('mousemove', {
          bubbles: true,
          clientX: 100 + index,
          clientY: 100 + index,
        }));
      }
      await new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      const after = Canvas.getPerformanceStats();
      const summary = Canvas.getSystemSummary();
      return {
        headless: Canvas.isHeadless(),
        summary,
        status: document.getElementById('statusBadge')?.textContent || '',
        ...after,
        pointerEventDelta: after.pointer_events - before.pointer_events,
        pointerFrameDelta: after.pointer_frames - before.pointer_frames,
      };
    }, MEDIUM_CASE);
    check(medium.headless === false && medium.components > medium.cull_threshold,
      `medium case (${MEDIUM_CASE}) state=${JSON.stringify(medium)}`);
    check(medium.schema === 'hysim_canvas_performance_v1' && medium.culling_active &&
          medium.hidden_components > 0 && medium.hidden_connections > 0,
      `viewport culling state=${JSON.stringify(medium)}`);
    check(medium.pointerEventDelta === 50 && medium.pointerFrameDelta <= 3,
      `50 pointer events coalesce into ${medium.pointerFrameDelta} render frame(s)`);

    // ---- 6) Small case returns to canvas mode + minimap visible ----
    const small = await page.evaluate(async (caseName) => {
      await App.loadBuiltinCase(caseName);
      const t0 = Date.now();
      while (Date.now() - t0 < 15000 &&
             (Canvas.isHeadless() || Canvas.state.components.length <= 20 ||
              Canvas.state.components.length >= Canvas.getPerformanceStats().cull_threshold)) {
        await new Promise((r) => setTimeout(r, 150));
      }
      await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));
      return {
        headless: Canvas.isHeadless(),
        glyphs: Canvas.state.components.length,
        minimapDots: document.getElementById('minimapDots')?.querySelectorAll('rect').length || 0,
        minimapHidden: document.getElementById('minimap')?.hidden,
      };
    }, SMALL_CASE);
    check(small.headless === false && small.glyphs > 20, `small case (${SMALL_CASE}) renders on canvas (${small.glyphs} glyphs)`);
    check(small.minimapHidden === false && small.minimapDots === small.glyphs, `minimap shows ${small.minimapDots} dots in canvas mode`);

    // ---- 7) Managed analysis cancellation: disable duplicate run controls,
    //         abort frontend waiting, and never apply the delayed response. ----
    const delayedPfRoute = async (route) => {
      await new Promise((resolve) => setTimeout(resolve, 800));
      try {
        await route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({ converged: true, iterations: 1, residual: 0 }),
        });
      } catch { /* request was intentionally aborted */ }
    };
    await page.route('**/api/session/pf', delayedPfRoute);
    const backendCancelRequest = page.waitForRequest(request =>
      request.url().includes('/api/session/cancel') && request.method() === 'POST');
    const cancelledTask = await page.evaluate(async () => {
      App.setActiveModule('powerFlow');
      const pending = App.runPowerFlow();
      const deadline = Date.now() + 5000;
      while (Date.now() < deadline && !App.getTaskStatus().cancellable) {
        await new Promise((resolve) => setTimeout(resolve, 20));
      }
      const during = App.getTaskStatus();
      const cancelButton = document.getElementById('btnCancelTask');
      const cancelVisibleDuring = cancelButton?.hidden === false;
      const runButtonsDisabled = Array.from(document.querySelectorAll('.run-btn')).every(button => button.disabled);
      cancelButton?.click();
      const result = await pending;
      await new Promise((resolve) => setTimeout(resolve, 50));
      return {
        during,
        cancelVisible: cancelVisibleDuring,
        runButtonsDisabled,
        resultWasDiscarded: result == null,
        after: App.getTaskStatus(),
        cancelHiddenAfter: cancelButton?.hidden === true,
      };
    });
    const cancelRequest = await backendCancelRequest;
    await new Promise((resolve) => setTimeout(resolve, 850));
    await page.unroute('**/api/session/pf', delayedPfRoute);
    check(cancelledTask.during?.state === 'running' && cancelledTask.during?.cancellable === true &&
          cancelledTask.cancelVisible && cancelledTask.runButtonsDisabled,
      'active analysis exposes cancellation and disables duplicate run controls');
    check(cancelledTask.resultWasDiscarded && cancelledTask.after?.state === 'cancelled' &&
          cancelledTask.after?.cancellable === false && cancelledTask.cancelHiddenAfter,
      'cancelled analysis discards its response and enters the cancelled state');
    check(cancelRequest.method() === 'POST',
      'frontend cancellation notifies the backend cancellation endpoint');

    const stalePfRoute = async (route) => {
      await new Promise((resolve) => setTimeout(resolve, 500));
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ converged: true, iterations: 1, residual: 0 }),
      });
    };
    await page.route('**/api/session/pf', stalePfRoute);
    const staleTask = await page.evaluate(async () => {
      const pending = App.runPowerFlow();
      const deadline = Date.now() + 5000;
      while (Date.now() < deadline && !App.getTaskStatus().cancellable) {
        await new Promise((resolve) => setTimeout(resolve, 20));
      }
      App.onTopologyChanged();
      const result = await pending;
      return {
        resultWasDiscarded: result == null,
        task: App.getTaskStatus(),
        pfChip: document.getElementById('depChipPf')?.textContent || '',
      };
    });
    await page.unroute('**/api/session/pf', stalePfRoute);
    check(staleTask.resultWasDiscarded && staleTask.task?.state === 'stale' &&
          /需运行|已失效/.test(staleTask.pfChip),
      'model revision changes discard stale analysis responses');

    // ---- 7) Time-series split: tspf in security, runs; annual in planning ----
    const ts = await page.evaluate(async () => {
      document.querySelector('.workflow-btn[data-workflow="security"]').click();
      await new Promise((r) => setTimeout(r, 40));
      const securityModules = Array.from(document.querySelectorAll('.module-btn'))
        .filter((b) => !b.classList.contains('workflow-hidden')).map((b) => b.dataset.module);
      App.setActiveModule('tspf');
      await new Promise((r) => setTimeout(r, 40));
      const tspfActionsVisible = !document.querySelector('.ts-tspf-actions')?.hidden;
      const annualHiddenForTspf = !!document.querySelector('.ts-annual-group')?.hidden;
      document.getElementById('simulationHours').value = '4';
      document.getElementById('btnRunTimeSeriesPF').click();
      const t0 = Date.now();
      while (Date.now() - t0 < 30000) {
        const chip = document.getElementById('depChipTspf')?.textContent || '';
        if (/收敛|失效/.test(chip)) break;
        await new Promise((r) => setTimeout(r, 300));
      }
      App.setActiveModule('timeSeries');
      await new Promise((r) => setTimeout(r, 40));
      const annualVisible = !document.querySelector('.ts-annual-group')?.hidden;
      const tspfHiddenForAnnual = !!document.querySelector('.ts-tspf-actions')?.hidden;
      return {
        securityHasTspf: securityModules.includes('tspf'),
        tspfActionsVisible, annualHiddenForTspf,
        tspfChip: document.getElementById('depChipTspf')?.textContent || '',
        annualVisible, tspfHiddenForAnnual,
      };
    });
    check(ts.securityHasTspf === true, '时序潮流 (tspf) is grouped under 安全与动态');
    check(ts.tspfActionsVisible && ts.annualHiddenForTspf, 'tspf module shows only the TSPF action group');
    check(/\d+\/\d+收敛/.test(ts.tspfChip), `TSPF runs from its module: "${ts.tspfChip}"`);
    check(ts.annualVisible && ts.tspfHiddenForAnnual, '时序生产模拟 module shows only the annual action group');

    // ---- 8) Results dock: pin two snapshots into a comparison table ----
    const cmp = await page.evaluate(async () => {
      await App.runPowerFlow();
      App.switchTab('results');
      document.getElementById('btnPinResultSnapshot').click();
      await new Promise((r) => setTimeout(r, 100));
      document.getElementById('btnPinResultSnapshot').click();
      await new Promise((r) => setTimeout(r, 100));
      const div = document.getElementById('resultComparison');
      return { hidden: div.hidden, cols: div.querySelectorAll('thead th').length, rows: div.querySelectorAll('tbody tr').length };
    });
    check(cmp.hidden === false && cmp.cols === 3 && cmp.rows >= 1,
      `results comparison pins 2 snapshots (${cmp.cols} cols, ${cmp.rows} metric rows)`);

    // ---- 9) Frontend↔backend wiring: edited TSPF horizon must win over an
    //         imported scenario's length (regression for the 8760->N bug) ----
    const scenarioStr = await page.evaluate(() => {
      return App.loadBuiltinCase('ieee14_acdc').then(() => new Promise((resolve) => {
        const t0 = Date.now();
        const wait = () => {
          if (Canvas.state.components.length > 0 || Date.now() - t0 > 15000) {
            const sys = Canvas.buildSystemJson();
            const N = 240;
            resolve(JSON.stringify({
              ...sys, name: 'e2e_scenario_240',
              _generated_scenario: { family: 'regular', representative_id: 'rep_e2e' },
              _time_series: {
                num_steps: N, step_duration_hr: 1.0,
                profiles: [
                  { id: 0, name: 'load', values: Array.from({ length: N }, (_, i) => 0.9 + 0.2 * Math.sin(2 * Math.PI * (i % 24) / 24) + 0.002 * i) },
                  { id: 1, name: 'wind', values: Array.from({ length: N }, () => 0.5) },
                  { id: 2, name: 'solar', values: Array.from({ length: N }, (_, i) => (i % 24 >= 6 && i % 24 <= 18) ? 0.7 : 0) },
                ],
                binding: { assign_all_loads_to: 0, load_profile_map: [] },
              },
            }));
          } else { setTimeout(wait, 150); }
        };
        wait();
      }));
    });
    const scenarioPath = path.join(tmpdir(), `hacdcpf_e2e_scenario_${process.pid}.json`);
    writeFileSync(scenarioPath, scenarioStr);
    await page.setInputFiles('#fileImportGeneratedRegularScenario', scenarioPath);
    await page.waitForTimeout(1500);
    const horizon = await page.evaluate(async () => {
      const simHr = document.getElementById('simulationHours');
      const importedLen = simHr.value;                 // scenario length (240)
      const checkbox = document.getElementById('regUseScenarioTimeSeries')?.checked;
      App.setActiveModule('tspf');
      simHr.value = '48';                              // user shortens the horizon
      document.getElementById('tspfSkipUC').checked = true;
      const roundTripped = Canvas.buildSystemJson();
      document.getElementById('btnRunTimeSeriesPF').click();
      const t0 = Date.now();
      while (Date.now() - t0 < 40000) {
        const c = document.getElementById('depChipTspf')?.textContent || '';
        if (/收敛|失效/.test(c)) break;
        await new Promise((r) => setTimeout(r, 300));
      }
      const annualResponse = await fetch('/api/session/run_annual_sim', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          resolution: '6h',
          use_session_time_series: true,
          run_opf: false,
          skip_replay: true,
          parallel_daily: false,
          enable_external_grid: true,
        }),
      });
      const annual = await annualResponse.json();
      const dayResponse = await fetch('/api/session/run_ts_pf', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          day_index: 2,
          num_steps: 4,
          step_duration_hr: 6,
          use_session_time_series: true,
          skip_uc: true,
          run_opf: false,
          enable_external_grid: true,
        }),
      });
      const day = await dayResponse.json();
      const expectedDayLoad = (annual.timeline_load || []).slice(4, 8);
      const actualDayLoad = day.total_load || [];
      const dayMatchesAnnual = expectedDayLoad.length === actualDayLoad.length &&
        expectedDayLoad.every((value, i) => Math.abs(Number(value) - Number(actualDayLoad[i])) < 1e-6);
      return {
        importedLen,
        checkbox,
        preservedSteps: roundTripped?._time_series?.num_steps,
        preservedFamily: roundTripped?._generated_scenario?.family,
        ranSteps: _lastTspfStepsForTest(),
        chip: document.getElementById('depChipTspf')?.textContent,
        simHrAfter: simHr.value,
        annualProfileSource: annual.profile_source,
        annualBindingActive: annual.scenario_profile_binding_active,
        annualFirstDayUnique: new Set((annual.timeline_load || []).slice(0, 4).map(v => Number(v).toFixed(6))).size,
        annualDayProfileSource: day.annual_day_profile_source,
        annualDayMatches: dayMatchesAnnual,
      };
      function _lastTspfStepsForTest() { return document.getElementById('depChipTspf')?.textContent || ''; }
    });
    check(horizon.importedLen === '240' && horizon.checkbox === true, 'scenario import set horizon=240 and checked 使用场景时序');
    check(horizon.preservedSteps === 240 && horizon.preservedFamily === 'regular',
      'Canvas round-trip preserves generated-scenario time series and metadata');
    check(/TSPF: 48\/48/.test(horizon.chip) && horizon.simHrAfter === '48',
      `edited horizon (48) wins over imported scenario length: "${horizon.chip}"`);
    check(horizon.annualProfileSource === 'imported_scenario' &&
          horizon.annualBindingActive === true && horizon.annualFirstDayUnique > 1,
      'annual production simulation preserves imported intraday load variation');
    check(horizon.annualDayProfileSource === 'imported_scenario' && horizon.annualDayMatches,
      'annual day drill-down slices the imported scenario profile');

    const dist33Dr = await page.evaluate(async () => {
      await fetch('/api/session/load_builtin', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ case: 'dist33_microgrid_der' }),
      });
      await fetch('/api/session/set_ts_config', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          num_steps: 4, step_duration_hr: 1,
          profiles: [
            { id: 0, name: 'load', values: [0.7, 0.9, 1.1, 0.8] },
            { id: 1, name: 'wind', values: [0.2, 0.5, 1.0, 0.4] },
            { id: 2, name: 'solar', values: [0.0, 0.4, 1.0, 0.1] },
          ],
          assign_all_loads_to: 0,
        }),
      });
      const response = await fetch('/api/session/run_ts_pf', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          num_steps: 4, skip_uc: false, run_opf: false,
          uc_solver: 'scip', enable_demand_response: true,
          dr_shiftable: true, dr_penalty: 0,
          enable_external_grid: true,
        }),
      });
      const data = await response.json();
      return {
        error: data.error || '',
        resourceCount: data.demand_response_resource_count,
        upRows: data.flexible_load_up_mw?.length || 0,
        downRows: data.flexible_load_down_mw?.length || 0,
        baselineSteps: data.total_load_baseline?.length || 0,
        servedSteps: data.total_load?.length || 0,
        renewableSteps: data.renewable_curtailment_mw?.length || 0,
        utilization: data.renewable_utilization_pct,
      };
    });
    check(!dist33Dr.error && dist33Dr.resourceCount === 3 &&
          dist33Dr.upRows === 3 && dist33Dr.downRows === 3,
      'dist33 DR portfolio exports three schedule resources');
    check(dist33Dr.baselineSteps === 4 && dist33Dr.servedSteps === 4 &&
          dist33Dr.renewableSteps === 4 && Number.isFinite(Number(dist33Dr.utilization)),
      'dist33 DR API exports baseline/served demand and renewable utilization metrics');

    // ---- 10) Annual dashboard windows 8760 points and keeps paging responsive ----
    const syntheticHours = Array.from({ length: 8760 }, (_, index) => index);
    const syntheticLoad = syntheticHours.map(index => 80 + 15 * Math.sin(index * Math.PI / 12));
    syntheticLoad[4321] = 180;
    const syntheticAnnual = {
      feasible: true,
      solver_name: 'E2E synthetic annual result',
      num_steps: syntheticHours.length,
      step_duration_hr: 1,
      num_pf_converged: syntheticHours.length,
      timeline_hours: syntheticHours,
      timeline_load: syntheticLoad,
      timeline_gen: syntheticLoad.map(value => value - 5),
      timeline_ren: syntheticLoad.map(() => 5),
      timeline_curt: syntheticLoad.map(() => 0),
      timeline_ess: syntheticLoad.map(() => 0),
      timeline_supply: syntheticLoad,
      timeline_demand: syntheticLoad,
      timeline_loss: syntheticLoad.map(() => 0),
      monthly_summaries: [],
      representative_days: [],
      gen_stats: [],
      storage_stats: [],
      renewable_stats: [],
    };
    await page.route('**/api/session/run_annual_sim', (route) => route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify(syntheticAnnual),
    }));
    const annualGui = await page.evaluate(async () => {
      App.setActiveModule('timeSeries');
      const windowSelect = document.getElementById('annualTimelineWindow');
      windowSelect.value = '720';
      document.getElementById('btnRunAnnualSim').click();
      const deadline = Date.now() + 15000;
      while (Date.now() < deadline && !App.getAnnualTimelineWindowStatus()) {
        await new Promise(resolve => setTimeout(resolve, 50));
      }
      const first = App.getAnnualTimelineWindowStatus();
      document.getElementById('btnAnnualWindowNext').click();
      await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      const next = App.getAnnualTimelineWindowStatus();
      windowSelect.value = 'all';
      windowSelect.dispatchEvent(new Event('change', { bubbles: true }));
      await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      const all = App.getAnnualTimelineWindowStatus();
      return {
        first, next, all,
        label: document.getElementById('annualWindowStatus')?.textContent || '',
        previousDisabled: document.getElementById('btnAnnualWindowPrev')?.disabled,
        nextDisabled: document.getElementById('btnAnnualWindowNext')?.disabled,
      };
    });
    await page.unroute('**/api/session/run_annual_sim');
    check(annualGui.first?.start === 0 && annualGui.first?.end === 720 &&
          annualGui.first?.rendered === 720,
      'annual dashboard initially renders one 30-day window');
    check(annualGui.next?.start === 720 && annualGui.next?.end === 1440,
      'annual dashboard advances to the next non-overlapping window');
    check(annualGui.all?.start === 0 && annualGui.all?.end === 8760 &&
          annualGui.all?.rendered <= 2000 && annualGui.previousDisabled && annualGui.nextDisabled &&
          /8760.*绘制/.test(annualGui.label),
      `full-year dashboard renders ${annualGui.all?.rendered} points with bounded controls`);

    // ---- 11) Rich-component Dashboard mapping + OPF core panels ----
    const richGui = await page.evaluate(async () => {
      await App.loadBuiltinCase('multiscale_comprehensive_acdc');
      const loadStarted = Date.now();
      while (Date.now() - loadStarted < 15000 &&
             !Canvas.state.components.some(c => c.type === 'mobile_storage')) {
        await new Promise((r) => setTimeout(r, 150));
      }

      App.switchTab('topology');
      await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));
      const mobileComp = Canvas.state.components.find(c => c.type === 'mobile_storage');
      const mobileRow = document.querySelector('#msTableInner tbody tr[data-row="0"]');
      mobileRow?.dispatchEvent(new MouseEvent('click', { bubbles: true }));
      await new Promise((r) => setTimeout(r, 100));
      const mobileSelectedId = Canvas.state.selectedId;

      App.setActiveModule('hosting');
      document.getElementById('btnRunBearingCap').click();
      const hostingStarted = Date.now();
      while (Date.now() - hostingStarted < 30000 &&
             !document.querySelector('#bearingBranchResults tbody tr')) {
        await new Promise((r) => setTimeout(r, 150));
      }
      const hostingRow = document.querySelector('#bearingBranchResults tbody tr');
      const hostingCompId = Number(hostingRow?.dataset.compId);
      const hostingComp = Canvas.state.components.find(c => c.id === hostingCompId);
      const hostingRef = hostingRow?.dataset.resultRef;
      const hostingCanvasType = hostingRow?.dataset.canvasType;
      const hostingCanvasIndex = Number(hostingRow?.dataset.canvasIndex);
      hostingRow?.dispatchEvent(new MouseEvent('click', { bubbles: true }));
      await new Promise((r) => setTimeout(r, 100));
      const hostingSelectedId = Canvas.state.selectedId;

      App.setActiveModule('opf');
      const opf = await App.runOpf();
      const text = (id) => (document.getElementById(id)?.textContent || '').trim();
      return {
        mobileCompId: mobileComp?.id,
        mobileRowCompId: Number(mobileRow?.dataset.compId),
        mobileSelectedId,
        hostingCompId,
        hostingCompType: hostingComp?.type,
        hostingSelectedId,
        hostingRef,
        hostingCanvasType,
        hostingCanvasIndex,
        opfConverged: !!opf?.converged,
        opfContract: opf?._result_contract,
        taskStatus: App.getTaskStatus(),
        activeGroup: document.getElementById('resultsContent')?.dataset.activeGroup,
        summary: text('opfSummary'),
        scope: text('opfScope'),
        generators: text('opfGenResults'),
        acBuses: text('opfBusResults'),
      };
    });
    check(Number.isInteger(richGui.mobileCompId) &&
          richGui.mobileRowCompId === richGui.mobileCompId &&
          richGui.mobileSelectedId === richGui.mobileCompId,
      `mobile-storage Dashboard row maps to Canvas component ${richGui.mobileCompId}`);
    check(Number.isInteger(richGui.hostingCompId) &&
          ['transformer_2w', 'ac_branch'].includes(richGui.hostingCompType) &&
          richGui.hostingSelectedId === richGui.hostingCompId,
      `hosting-capacity result row maps to Canvas ${richGui.hostingCompType} ${richGui.hostingCompId}`);
    check(richGui.hostingRef === 'hysim_canvas_ref_v1' &&
          richGui.hostingCanvasType === 'transformer_2w' &&
          Number.isInteger(richGui.hostingCanvasIndex),
      'hosting result row exposes the unified Canvas reference contract');
    check(richGui.opfConverged === true && richGui.activeGroup === 'opf',
      'multiscale comprehensive OPF converges and activates the OPF result group');
    check(richGui.opfContract?.schema === 'hysim_result_v1' &&
          richGui.opfContract.component_ref_schema === 'hysim_canvas_ref_v1' &&
          richGui.opfContract.stale === false &&
          richGui.taskStatus?.state === 'completed',
      'OPF result and task status use the unified non-stale contracts');
    check(richGui.summary.length > 0, 'OPF summary panel contains data');
    check(richGui.scope.length > 0, 'OPF constraint panel contains data');
    check(richGui.generators.length > 0, 'OPF generator dispatch panel contains data');
    check(richGui.acBuses.length > 0, 'OPF AC voltage/LMP panel contains data');

    // ---- 10) Hybrid topology reconfiguration: AC/DC/VSC contract ----
    const hybridReconfig = await page.evaluate(async () => {
      App.setActiveModule('topology');
      document.getElementById('rcSplitTrees').checked = true;
      document.getElementById('rcDcMesh').checked = false;
      document.getElementById('btnRunTopology').click();
      const started = Date.now();
      while (Date.now() - started < 60000) {
        const active = document.getElementById('resultsContent')?.dataset.activeGroup;
        const status = document.getElementById('statusBadge')?.textContent || '';
        if (active === 'topology' && !/重构中/.test(status)) break;
        await new Promise((r) => setTimeout(r, 200));
      }
      const summary = document.getElementById('resultsSummary')?.textContent || '';
      const details = document.getElementById('topoResults')?.textContent || '';
      return {
        active: document.getElementById('resultsContent')?.dataset.activeGroup,
        summary,
        details,
        dcRows: document.querySelectorAll('#topoResults table tbody tr').length,
      };
    });
    check(hybridReconfig.active === 'topology' && /MILP候选\s*✓ 可行/.test(hybridReconfig.summary),
      `multiscale hybrid topology reconfiguration summary=${JSON.stringify(hybridReconfig)}`);
    check(/负荷削减\(MW\)\s*0\.0000/.test(hybridReconfig.summary),
      'hybrid topology reconfiguration serves all load');
    check(/AC\/DC分域辐射状/.test(hybridReconfig.details) && /连通/.test(hybridReconfig.details),
      'topology result reports domain radiality and hybrid connectivity');
    check(/DC支路状态/.test(hybridReconfig.details) && /VSC耦合状态/.test(hybridReconfig.details),
      'Dashboard exports DC branch and VSC topology results');
    check(/验证潮流:\s*收敛/.test(hybridReconfig.details),
      `reconfigured hybrid topology validation=${JSON.stringify(hybridReconfig)}`);

    // ---- 11) Reduced-network export: hybrid terminal integrity + reload ----
    const reducedRoundTrip = await page.evaluate(async () => {
      const reductionResponse = await fetch('/api/session/network_reduction', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          enable_switch_contraction: true,
          enable_series_reduction: true,
          enable_pendant_reduction: false,
          contract_zero_impedance_lines: false,
        }),
      });
      const reduction = await reductionResponse.json();
      const system = reduction.reduced_system;
      if (!reductionResponse.ok || !system) return { error: reduction.error || 'missing reduced_system' };
      const ac = new Set((system.ac?.buses || []).map(bus => Number(bus.index)));
      const dc = new Set((system.dc?.buses || []).map(bus => Number(bus.index)));
      const valid = value => Number.isFinite(Number(value));
      const acEdgeOk = (system.ac?.branches || []).every(x => ac.has(Number(x.from_bus)) && ac.has(Number(x.to_bus)));
      const dcEdgeOk = (system.dc?.branches || []).every(x => dc.has(Number(x.from_bus)) && dc.has(Number(x.to_bus)));
      const vscOk = (system.vsc_converters || []).every(x => ac.has(Number(x.bus_ac)) && dc.has(Number(x.bus_dc)));
      const dcdcOk = (system.dc?.dcdc_converters || []).every(x => dc.has(Number(x.bus_in)) && dc.has(Number(x.bus_out)));
      const routerOk = (system.energy_routers || []).every(router =>
        (router.ports || []).length >= 2 && (router.ports || []).every(port =>
          String(port.port_type || 'AC').toUpperCase() === 'DC'
            ? dc.has(Number(port.bus))
            : ac.has(Number(port.bus))));
      const mobileOk = (system.mobile_storage || []).every(x => ac.has(Number(x.bus)));
      const noGhosts = (system.ac?.buses || []).every(x => x.in_service !== false) &&
        (system.dc?.buses || []).every(x => x.in_service !== false) &&
        (system.ac?.branches || []).every(x => x.in_service !== false) &&
        (system.dc?.branches || []).every(x => x.in_service !== false);
      const loadResponse = await fetch('/api/session/load_json_string', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ json_string: JSON.stringify(system) }),
      });
      const loaded = await loadResponse.json();
      const topologyResponse = await fetch('/api/session/topology', {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}',
      });
      const topology = await topologyResponse.json();
      const pfResponse = await fetch('/api/session/pf', {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}',
      });
      const pf = await pfResponse.json();
      return {
        schema: reduction.reduced_system_schema,
        acBusCount: ac.size,
        dcBusCount: dc.size,
        vscCount: (system.vsc_converters || []).length,
        referencesValid: acEdgeOk && dcEdgeOk && vscOk && dcdcOk && routerOk && mobileOk && noGhosts,
        reloadOk: loadResponse.ok && !loaded.error,
        topologyOk: topologyResponse.ok && !topology.error && valid(topology.n_buses),
        pfConverged: pfResponse.ok && pf.converged === true,
      };
    });
    check(!reducedRoundTrip.error && reducedRoundTrip.schema === 'hacdcpf_system_json_v1' &&
          reducedRoundTrip.acBusCount > 0 && reducedRoundTrip.dcBusCount > 0 &&
          reducedRoundTrip.vscCount > 0 && reducedRoundTrip.referencesValid,
      'reduced-network export contains a compact hybrid AC/DC/VSC system with valid terminals');
    check(reducedRoundTrip.reloadOk && reducedRoundTrip.topologyOk && reducedRoundTrip.pfConverged,
      'exported reduced hybrid network reloads and passes topology/PF validation');
  } finally {
    await browser.close();
    if (proc) proc.kill();
  }

  console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  return failures === 0 ? 0 : 1;
}

main().then((code) => process.exit(code)).catch((e) => { console.error(e); process.exit(1); });
