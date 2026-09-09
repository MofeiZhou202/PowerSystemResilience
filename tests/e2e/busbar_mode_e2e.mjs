// @ts-check
// Busbar-mode one-line E2E (P1). Starts run_gui_server, loads a real hybrid
// AC/DC case, enables busbar mode, runs the BUSBAR auto-layout and a power flow,
// and asserts the busbar rendering + legacy revert + result overlay + no errors.
//
// Run:  node tests/e2e/busbar_mode_e2e.mjs [--case ieee14_acdc] [--server <path>]
// Deps: playwright (installed) + a built run_gui_server binary.

import { chromium } from 'playwright';
import { spawn } from 'node:child_process';
import net from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const arg = (name, fb) => { const i = process.argv.indexOf(`--${name}`); return i >= 0 ? process.argv[i + 1] : fb; };
const CASE = arg('case', 'ieee14_acdc');

function freePort() {
  return new Promise((resolve, reject) => {
    const s = net.createServer();
    s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); });
    s.on('error', reject);
  });
}
function findServer() {
  const c = arg('server', '');
  const candidates = c ? [c] : [
    'build/macos-release/run_gui_server', 'build/macos-release/tests/run_gui_server',
    'build/local-gui-review/run_gui_server', 'build/tests/run_gui_server',
    'build/linux-release/run_gui_server', 'build/tests/run_gui_server.exe',
  ].map(p => path.join(ROOT, p));
  const found = candidates.find(existsSync);
  if (!found) { console.error('run_gui_server not found; build it or pass --server'); process.exit(2); }
  return found;
}
async function waitReady(base, timeoutMs = 25000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try { const r = await fetch(`${base}/api/cases`); if (r.ok) return true; } catch { /* not up yet */ }
    await new Promise(r => setTimeout(r, 300));
  }
  return false;
}

async function main() {
  const port = await freePort();
  const base = `http://127.0.0.1:${port}`;
  const server = spawn(findServer(),
    ['--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(ROOT, 'data'), '--matpower-dir', path.join(ROOT, 'data')],
    { cwd: ROOT, stdio: 'ignore' });
  const checks = [];
  const record = (name, ok, detail = '') => { checks.push({ name, ok, detail }); console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name}${detail ? ' — ' + detail : ''}`); };

  let browser;
  try {
    if (!await waitReady(base)) { console.error('server did not become ready'); process.exit(2); }
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); localStorage.setItem('busbarMode', '0'); } catch { } });
    await page.route('https://cdn.plot.ly/**', route => route.fulfill({ contentType: 'text/javascript', body: 'window.Plotly={newPlot:()=>Promise.resolve(),react:()=>Promise.resolve(),purge:()=>{},relayout:()=>{},restyle:()=>{},addTraces:()=>{},deleteTraces:()=>{},Plots:{resize:()=>{}}};' }));

    const errors = [];
    page.on('pageerror', e => errors.push(String(e.message).slice(0, 160)));
    page.on('console', m => { if (m.type() === 'error') { const t = m.text(); if (/canvas|busbar|isBusType|projectOntoBar|respan|IEC_SYMBOLS|TypeError|ReferenceError/i.test(t)) errors.push(t.slice(0, 160)); } });

    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined' && HySimCore?.LayoutEngine, null, { timeout: 30000 });

    // Load a real case, enable busbar mode, run the BUSBAR auto-layout.
    const layout = await page.evaluate(async (caseName) => {
      await App.loadBuiltinCase(caseName);
      await Canvas.waitForLayout();
      Canvas.setBusbarMode(true);
      const metrics = await Canvas.autoLayout({ direction: 'BUSBAR', timeoutMs: 60000 });
      await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
      const buses = Canvas.state.components.filter(c => c.type === 'ac_bus' || c.type === 'dc_bus');
      let spanned = 0, portDotsHidden = true, maxHalf = 0;
      for (const b of buses) {
        const lineEl = b.el && b.el.querySelector('line');
        const half = lineEl ? Math.abs(parseFloat(lineEl.getAttribute('x2'))) : 0;
        if (half >= 40) spanned += 1;
        maxHalf = Math.max(maxHalf, half);
        const dot = b.el && b.el.querySelector('.port circle');
        if (dot && dot.getAttribute('opacity') !== '0') portDotsHidden = false;
      }
      return { busCount: buses.length, spanned, maxHalf, portDotsHidden,
        crossings: metrics?.crossings, overlaps: metrics?.overlaps, busbarMode: localStorage.getItem('busbarMode') };
    }, CASE);

    record('case loaded with buses', layout.busCount > 0, `${layout.busCount} buses`);
    record('busbar mode persisted', layout.busbarMode === '1');
    record('all bus bars span their taps', layout.spanned === layout.busCount, `${layout.spanned}/${layout.busCount}, maxHalf=${layout.maxHalf}`);
    record('busbar spans beyond legacy 80px', layout.maxHalf > 40, `maxHalf=${layout.maxHalf}`);
    record('legacy port dots hidden on busbars', layout.portDotsHidden === true);
    record('auto-layout has no overlaps', (layout.overlaps ?? 99) === 0, `overlaps=${layout.overlaps}, crossings=${layout.crossings}`);

    // Run a real power flow; result overlay must apply in busbar mode.
    await page.evaluate(async () => { try { await App.runPowerFlow(); } catch (e) { /* status checked below */ } });
    await page.waitForTimeout(1500);
    const pf = await page.evaluate(() => (document.body.innerText.match(/PF:\s*[^\n|]{0,16}/) || [''])[0]);
    record('power flow converged in busbar mode', /收敛|converg/i.test(pf), pf.trim());

    // P3-engine: result-overlay parity on busbars. A converged PF recolors each
    // busbar by voltage (ETAP convention, held in the auto-cleared results
    // layer); enabling flow+heatmap draws flow arrows and loading colors on the
    // busbar-connected links without disturbing the voltage bars.
    const overlay = await page.evaluate(() => {
      const VCOL = new Set(['#e06c75', '#d19a66', '#98c379', '#56b6c2']);
      const bars = [...document.querySelectorAll('#resultsLayer .result-busbar-voltage')];
      const colored = bars.filter(b => VCOL.has((b.getAttribute('stroke') || '').toLowerCase()));
      const spanned = bars.filter(b => Math.abs(parseFloat(b.getAttribute('x2')) - parseFloat(b.getAttribute('x1'))) > 40);
      const vLabels = document.querySelectorAll('#resultsLayer .result-voltage').length;
      Canvas.setVisualizationMode('both');
      Canvas.refreshVisualization();
      const flowLabels = document.querySelectorAll('.flow-label').length;
      const recolored = [...document.querySelectorAll('.conn-line')]
        .filter(l => (l.getAttribute('stroke') || '').toLowerCase() !== '#666').length;
      const barsAfterViz = document.querySelectorAll('#resultsLayer .result-busbar-voltage').length;
      Canvas.setVisualizationMode('off');
      return { bars: bars.length, colored: colored.length, spanned: spanned.length, vLabels, flowLabels, recolored, barsAfterViz };
    });
    record('busbars recolor by voltage after PF (ETAP)', overlay.bars > 0 && overlay.colored === overlay.bars, `${overlay.colored}/${overlay.bars} bars`);
    record('voltage bars span the bus width + carry readouts', overlay.spanned > 0 && overlay.vLabels > 0, `${overlay.spanned} spanned, ${overlay.vLabels} labels`);
    record('links show flow + loading-tinted wires on busbars', overlay.flowLabels > 0 && overlay.recolored > 0, `${overlay.flowLabels} flow labels, ${overlay.recolored} loading-tinted wires`);
    record('voltage bars survive visualization toggles', overlay.barsAfterViz === overlay.bars, `${overlay.barsAfterViz}/${overlay.bars}`);

    // P4: the one-line exports to a self-contained, printable vector SVG.
    const svg = await page.evaluate(() => {
      const s = Canvas.exportOneLineSvg();
      return { len: s.length, ok: typeof s === 'string' && s.startsWith('<?xml') && s.includes('<svg') && s.includes('comp-label') && /Bus/i.test(s) };
    });
    record('exports a valid standalone one-line SVG', svg.ok === true, `len=${svg.len}`);

    // P4: taking a bus out of service greys it (and its devices); restoring un-greys.
    const energ = await page.evaluate(() => {
      Canvas.setBusbarMode(true);
      const bus = Canvas.state.components.find(c => c.type === 'ac_bus');
      if (!bus) return { skipped: true };
      bus.params = bus.params || {};
      bus.params.in_service = false;
      Canvas.setEnergizationColoring(true);
      const greyed = !!(bus.el && bus.el.classList.contains('deenergized'));
      bus.params.in_service = true;
      Canvas.setEnergizationColoring(true);
      const restored = !!(bus.el && !bus.el.classList.contains('deenergized'));
      return { greyed, restored };
    });
    record('de-energized bus greys (energization coloring)', energ.greyed === true);
    record('re-energized bus un-greys', energ.restored === true);

    // Regression: default-off reverts to the legacy 80px, 4-port bus exactly.
    const legacy = await page.evaluate(() => {
      Canvas.setBusbarMode(false);
      const b = Canvas.state.components.find(c => c.type === 'ac_bus');
      const lineEl = b && b.el && b.el.querySelector('line');
      const dot = b && b.el && b.el.querySelector('.port circle');
      return { half: lineEl ? Math.abs(parseFloat(lineEl.getAttribute('x2'))) : null,
        dotVisible: dot ? dot.getAttribute('opacity') !== '0' : false };
    });
    record('legacy bus half-length is 40 when off', legacy.half === 40, `half=${legacy.half}`);
    record('legacy port dots visible when off', legacy.dotVisible === true);

    record('no canvas/busbar JS errors', errors.length === 0, errors.slice(0, 3).join(' | '));

    await page.evaluate(() => Canvas.setBusbarMode(true));
    await page.locator('#canvasContainer').screenshot({ path: path.join(ROOT, 'tests', 'e2e', 'busbar_mode.png'), animations: 'disabled' }).catch(() => { });
  } finally {
    if (browser) await browser.close().catch(() => { });
    server.kill('SIGTERM');
  }

  const failed = checks.filter(c => !c.ok);
  console.log(`\n${checks.length - failed.length}/${checks.length} checks passed for busbar path (case ${CASE}).`);
  process.exit(failed.length ? 1 : 0);
}

main().catch(e => { console.error(e); process.exit(2); });
