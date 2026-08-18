// @ts-check
// Generate and verify semantic ELK layout baselines for representative systems.

import { chromium } from 'playwright';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';

const ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..', '..');
const OUTPUT = path.join(ROOT, 'tests', 'e2e', 'baselines', 'layout');
const CASES = [
  { name: 'case33bw_acdc', slug: 'ieee33' },
  { name: 'ieee118_acdc', slug: 'ieee118' },
  { name: 'case300_acdc', slug: 'case300_acdc' },
  { name: 'multiscale_comprehensive_acdc', slug: 'multiscale_comprehensive_acdc' },
];

function argument(name, fallback) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length ? process.argv[index + 1] : fallback;
}

async function main() {
  const base = argument('base-url', 'http://127.0.0.1:18090');
  const caseFilter = argument('case', '');
  const update = process.argv.includes('--update-baselines');
  mkdirSync(OUTPUT, { recursive: true });
  const baselinePath = path.join(OUTPUT, 'layout_metrics.json');
  const previous = existsSync(baselinePath) ? JSON.parse(readFileSync(baselinePath, 'utf8')) : null;
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1600, height: 1000 }, deviceScaleFactor: 1 });
  await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
  await page.route('https://cdn.plot.ly/**', route => route.fulfill({
    contentType: 'text/javascript',
    body: 'window.Plotly={newPlot:()=>Promise.resolve(),react:()=>Promise.resolve(),Plots:{resize:()=>{}}};',
  }));
  await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined' &&
    HySimCore.LayoutGraph && HySimCore.LayoutEngine);

  const results = {};
  let failures = 0;
  for (const testCase of CASES.filter(testCase => !caseFilter || testCase.slug === caseFilter || testCase.name === caseFilter)) {
    const result = await page.evaluate(async caseName => {
      await App.loadBuiltinCase(caseName);
      await Canvas.waitForLayout();
      const metrics = await Canvas.autoLayout({ engine: 'elk', direction: 'TB', timeoutMs: 90000 });
      await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      const graph = Canvas.getLayoutGraph();
      const routerHyperedges = graph?.hyperedges?.filter(edge => edge.type === 'energy_router') || [];
      return {
        metrics,
        layoutStats: Canvas.layoutStats,
        graph: graph ? {
          schema: graph.schema,
          domains: graph.domains,
          feeders: graph.feeders.length,
          hyperedges: graph.hyperedges.length,
          fixed_nodes: graph.fixed_node_ids.length,
          components: graph.component_count,
          connections: graph.connection_count,
          layout_edges: graph.layout_edge_count,
          router_star_ok: routerHyperedges.every(edge =>
            graph.graph.edges.filter(layoutEdge => layoutEdge.id.startsWith(`hyper-${edge.component_id}-`)).length === edge.terminals.length),
        } : null,
      };
    }, testCase.name);
    const screenshot = path.join(OUTPUT, `${testCase.slug}.png`);
    await page.locator('#canvasContainer').screenshot({ path: screenshot, animations: 'disabled' });
    let features = null;
    if (testCase.slug === 'ieee33') {
      features = await page.evaluate(async () => {
        const bus = Canvas.state.components.find(component => component.type === 'ac_bus');
        Canvas.state.selectedIds.clear();
        Canvas.state.selectedIds.add(bus.id);
        Canvas.state.selectedId = bus.id;
        const locked = Canvas.toggleLayoutFixed();
        const before = { x: bus.x, y: bus.y };
        const incremental = await Canvas.autoLayout({ engine: 'elk', direction: 'TB', incremental: true });
        const fixedPreserved = bus.x === before.x && bus.y === before.y;
        const lockPersisted = Canvas.buildSystemJson()._canvas.components.some(item => item.layoutFixed === true);
        const folded = Canvas.foldFeeders({ minBuses: 8 });
        const hiddenBefore = document.querySelectorAll('#componentsLayer .layout-folded').length;
        const clustersBefore = document.querySelectorAll('#layoutOverlayLayer .feeder-cluster').length;
        document.querySelector('#layoutOverlayLayer .feeder-cluster')?.dispatchEvent(new MouseEvent('click', { bubbles: true }));
        const hiddenAfterLocalExpand = document.querySelectorAll('#componentsLayer .layout-folded').length;
        Canvas.expandFeeders();
        const hiddenAfterAllExpand = document.querySelectorAll('#componentsLayer .layout-folded').length;
        Canvas.toggleLayoutFixed();
        Canvas.state.selectedIds.clear();
        Canvas.state.selectedId = null;
        return {
          locked, fixedPreserved, lockPersisted, incremental,
          folded: folded.folded,
          hiddenBefore, clustersBefore, hiddenAfterLocalExpand, hiddenAfterAllExpand,
          modelComponents: Canvas.buildSystemJson()._canvas.components.length,
          canvasComponents: Canvas.state.components.length,
        };
      });
      const featureOk = features.locked.changed === 1 && features.fixedPreserved && features.lockPersisted &&
        features.incremental.engine === 'elk-layered-worker' && features.folded > 0 &&
        features.hiddenBefore > 0 && features.clustersBefore > 0 &&
        features.hiddenAfterLocalExpand < features.hiddenBefore && features.hiddenAfterAllExpand === 0 &&
        features.modelComponents === features.canvasComponents;
      console.log(`  [${featureOk ? 'PASS' : 'FAIL'}] feeder LOD, local expand, fixed-node incremental layout`);
      if (!featureOk) failures += 1;
    }
    const metric = result.metrics;
    const entry = {
      case: testCase.name,
      screenshot: path.basename(screenshot),
      graph: result.graph,
      metrics: metric,
      layout_stats: result.layoutStats,
      features,
      budget: {
        max_overlaps: Math.max(2, metric.overlaps),
        max_crossings: Math.max(10, Math.ceil(metric.crossings * 1.15)),
        max_runtime_ms: Math.max(5000, Math.ceil(metric.runtime_ms * 2.5)),
      },
    };
    results[testCase.slug] = entry;
    const old = previous?.cases?.[testCase.slug];
    const valid = metric?.schema === 'hysim_layout_metrics_v1' &&
      metric.engine === 'elk-layered-worker' && result.graph?.schema === 'hysim_layout_graph_v1' &&
      result.graph.components === metric.node_count && result.graph.router_star_ok === true && existsSync(screenshot);
    let withinBudget = true;
    if (!update && old?.budget) {
      withinBudget = metric.overlaps <= old.budget.max_overlaps &&
        metric.crossings <= old.budget.max_crossings &&
        metric.runtime_ms <= old.budget.max_runtime_ms;
    }
    console.log(`  [${valid && withinBudget ? 'PASS' : 'FAIL'}] ${testCase.slug}: ` +
      `${metric.node_count} nodes, ${metric.crossings} crossings, ${metric.overlaps} overlaps, ${metric.runtime_ms} ms`);
    if (!valid || !withinBudget) failures += 1;
  }
  await browser.close();
  if (update || !previous) {
    writeFileSync(baselinePath, JSON.stringify({
      schema: 'hysim_layout_baselines_v1',
      generated_at: new Date().toISOString(),
      viewport: { width: 1600, height: 1000, device_scale_factor: 1 },
      cases: results,
    }, null, 2) + '\n');
  }
  console.log(failures ? `\n${failures} LAYOUT BASELINE(S) FAILED` : '\nALL LAYOUT BASELINES PASSED');
  return failures ? 1 : 0;
}

main().then(code => process.exit(code)).catch(error => {
  console.error(error);
  process.exit(1);
});
