// @ts-check
// gui_scale_features_e2e.mjs — browser end-to-end test for the large-system /
// scalability GUI features:
//   * headless (no-canvas) mode for large systems + auto return to canvas mode
//   * calculations (PF / OPF) run headless against the backend session
//   * virtualized, editable topology tables (no row cap; inline edit -> sync)
//   * global element search (canvas pan + headless row highlight, any position)
//   * time-series split: 时序潮流 (tspf) in 安全与动态, 时序生产模拟 in 规划与运行
//   * on-demand neighborhood sub-diagram (read-only, leaves state untouched)
//   * minimap (canvas mode) hidden in headless mode
//   * dependency chips reflect PF / canvas state
//
// Requirements (not part of the C++ build):
//   npm i -D playwright && npx playwright install chromium
//
// Usage:
//   node tests/e2e/gui_scale_features_e2e.mjs \
//        [--server build/macos-release/run_gui_server] [--data-dir data] [--port 0]
//
// Exit code 0 on success, 1 on any failed assertion. Starts (and stops) the GUI
// server itself unless --base-url points at an already-running instance.

import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';

const REPO_ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..', '..');

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
const SMALL_CASE = 'ieee14_acdc';

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

  const browser = await chromium.launch();
  try {
    const page = await browser.newPage();
    page.on('pageerror', (e) => { console.error('  [pageerror]', e.message); failures++; });
    await page.goto(base + '/xjtu/', { waitUntil: 'networkidle' });
    await page.waitForTimeout(400);

    // ---- 1) Large case -> headless mode, PF + OPF run against the backend ----
    const big = await page.evaluate(async (caseName) => {
      await App.loadMatpowerCase(caseName);
      const summary = Canvas.getSystemSummary();
      const opf = await fetch(window.location.origin + '/api/session/opf', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ solver: 'dc', constraints: {} }),
      }).then((r) => r.json()).catch((e) => ({ error: String(e) }));
      return {
        headless: Canvas.isHeadless(),
        glyphs: Canvas.state.components.length,
        buses: summary.buses,
        canvasChip: document.getElementById('depChipCanvas')?.textContent || '',
        pfChip: document.getElementById('depChipPf')?.textContent || '',
        opfConverged: !!(opf && opf.converged),
        minimapHidden: document.getElementById('minimap')?.hidden,
      };
    }, LARGE_CASE);
    check(big.headless === true, `large case (${LARGE_CASE}) enters headless mode`);
    check(big.glyphs === 0, 'headless mode creates zero canvas glyphs');
    check(big.buses > 600, `system summary reports ${big.buses} buses`);
    check(/无画布/.test(big.canvasChip), `canvas chip shows headless: "${big.canvasChip}"`);
    check(/收敛/.test(big.pfChip), `PF auto-ran and chip shows converged: "${big.pfChip}"`);
    check(big.opfConverged === true, 'OPF converges headless against the backend session');
    check(big.minimapHidden === true, 'minimap hidden in headless mode');

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

    // ---- 4) Neighborhood sub-diagram is read-only (state untouched) ----
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
    check(sub.stillHeadless === true && sub.glyphs === 0, 'sub-diagram leaves headless state untouched');

    // ---- 5) Small case returns to canvas mode + minimap visible ----
    const small = await page.evaluate(async (caseName) => {
      const sel = document.getElementById('ioCaseSelect');
      sel.value = caseName; sel.dispatchEvent(new Event('change', { bubbles: true }));
      document.getElementById('btnIoLoadBuiltin').click();
      const t0 = Date.now();
      while (Date.now() - t0 < 15000 && Canvas.state.components.length === 0) await new Promise((r) => setTimeout(r, 150));
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

    // ---- 6) Time-series split: tspf in security, runs; annual in planning ----
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

    // ---- 7) Results dock: pin two snapshots into a comparison table ----
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
  } finally {
    await browser.close();
    if (proc) proc.kill();
  }

  console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  return failures === 0 ? 0 : 1;
}

main().then((code) => process.exit(code)).catch((e) => { console.error(e); process.exit(1); });
