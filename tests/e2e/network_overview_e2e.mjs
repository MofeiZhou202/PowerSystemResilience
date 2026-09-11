// @ts-check
// Focused WebGL overview visual/interaction regression for desktop and mobile.

import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import { createServer } from 'node:net';
import { tmpdir } from 'node:os';
import path from 'node:path';
import process from 'node:process';

const arg = (name, fallback) => {
  const position = process.argv.indexOf(`--${name}`);
  return position >= 0 && position + 1 < process.argv.length
    ? process.argv[position + 1]
    : fallback;
};

const outputDir = arg('output-dir', tmpdir());

const intersects = (a, b) =>
  a && b && a.left < b.right && a.right > b.left && a.top < b.bottom && a.bottom > b.top;

const freePort = () => new Promise((resolve, reject) => {
  const server = createServer();
  server.listen(0, '127.0.0.1', () => {
    const address = server.address();
    server.close(() => resolve(typeof address === 'object' && address ? address.port : 0));
  });
  server.on('error', reject);
});

async function waitUp(base, timeoutMs = 20_000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    try {
      if ((await fetch(`${base}/api/cases`)).ok) return true;
    } catch { /* retry while the local server starts */ }
    await new Promise(resolve => setTimeout(resolve, 200));
  }
  return false;
}

async function main() {
  let chromium;
  try {
    ({ chromium } = await import('playwright'));
  } catch {
    console.error('playwright is not installed');
    return 2;
  }
  let baseUrl = arg('base-url', null);
  let serverProcess = null;
  if (!baseUrl) {
    const serverPath = arg('server', null);
    if (!serverPath) throw new Error('provide --base-url or --server');
    const port = await freePort();
    baseUrl = `http://127.0.0.1:${port}`;
    serverProcess = spawn(path.resolve(serverPath), [
      '--host', '127.0.0.1', '--port', String(port),
      '--data-dir', arg('data-dir', 'data'), '--matpower-dir', arg('data-dir', 'data'),
    ], { stdio: 'ignore' });
    process.once('exit', () => serverProcess?.kill());
    if (!await waitUp(baseUrl)) throw new Error('local GUI server did not start');
  }
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
  await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
  const errors = [];
  page.on('pageerror', error => errors.push(error.stack || error.message));
  await page.route('https://cdn.plot.ly/**', route => route.fulfill({
    contentType: 'text/javascript',
    body: 'window.Plotly={newPlot:()=>Promise.resolve(),react:()=>Promise.resolve(),purge:()=>{},Plots:{resize:()=>{}}};',
  }));
  await page.goto(`${baseUrl}/xjtu/`, { waitUntil: 'domcontentloaded' });
  await page.evaluate(() => App.loadMatpowerCase('case2869pegase.m'));
  await page.waitForFunction(() => Canvas.getNetworkOverviewStats()?.nodes > 2000);

  const inspect = () => page.evaluate(() => {
    const rect = id => {
      const value = document.getElementById(id)?.getBoundingClientRect();
      return value ? { left: value.left, top: value.top, right: value.right, bottom: value.bottom,
        width: value.width, height: value.height } : null;
    };
    const overview = Canvas.getNetworkOverviewStats();
    return {
      overview,
      root: rect('networkOverview'),
      toolbar: document.querySelector('.network-overview-toolbar')?.getBoundingClientRect().toJSON(),
      status: rect('networkOverviewStatus'),
      visible: !document.getElementById('networkOverview')?.hidden,
      svgGlyphs: Canvas.state.components.length,
    };
  });

  const desktop = await inspect();
  await page.screenshot({ path: `${outputDir}/hysim-network-overview-desktop.png`, fullPage: true });
  if (!desktop.visible || !desktop.overview?.webgl2 || desktop.svgGlyphs !== 0) {
    throw new Error('desktop overview is not active WebGL with zero SVG glyphs');
  }
  if (intersects(desktop.toolbar, desktop.status)) {
    throw new Error('desktop overview toolbar overlaps status');
  }

  await page.setViewportSize({ width: 390, height: 844 });
  await page.waitForTimeout(150);
  const mobile = await inspect();
  await page.screenshot({ path: `${outputDir}/hysim-network-overview-mobile.png`, fullPage: true });
  if (!mobile.root || mobile.root.width < 300 || mobile.root.height < 220 ||
      !mobile.toolbar || !mobile.status ||
      mobile.toolbar.left < mobile.root.left || mobile.toolbar.right > mobile.root.right ||
      mobile.status.left < mobile.root.left || mobile.status.right > mobile.root.right) {
    throw new Error('mobile overview controls overflow the canvas container');
  }
  if (intersects(mobile.toolbar, mobile.status)) {
    throw new Error('mobile overview toolbar overlaps status');
  }

  await page.click('#btnOverviewTable');
  const tableMode = await page.evaluate(() => ({
    active: document.body.classList.contains('network-overview-table'),
    panelWidth: document.getElementById('rightPanel')?.getBoundingClientRect().width || 0,
    topologyVisible: document.getElementById('tabTopology')?.classList.contains('active'),
  }));
  if (!tableMode.active || tableMode.panelWidth < 300 || !tableMode.topologyVisible) {
    throw new Error('mobile full-table mode did not replace the overview');
  }
  await page.click('#btnOverviewReturn');
  if (await page.evaluate(() => document.body.classList.contains('network-overview-table'))) {
    throw new Error('mobile full-table mode did not return to the overview');
  }

  await page.evaluate(() => {
    App.selectStableRef({ domain: 'ac', index: 100 });
    App.openLocalSubgraph('ac', 100, 2);
  });
  const local = await page.evaluate(() => {
    const modal = document.querySelector('.sub-diagram-modal')?.getBoundingClientRect();
    return {
      nodes: document.querySelectorAll('#subDiagramSvg .subdiag-node').length,
      modal: modal?.toJSON(),
      viewport: { width: window.innerWidth, height: window.innerHeight },
    };
  });
  if (!local.modal || local.nodes < 2 || local.modal.left < 0 || local.modal.right > local.viewport.width ||
      local.modal.top < 0 || local.modal.bottom > local.viewport.height) {
    throw new Error('mobile local SVG is empty or outside the viewport');
  }
  await page.screenshot({ path: `${outputDir}/hysim-local-svg-mobile.png`, fullPage: true });
  await browser.close();
  serverProcess?.kill();
  if (errors.length) throw new Error(`page errors: ${errors.join('; ')}`);
  for (const file of [
    `${outputDir}/hysim-network-overview-desktop.png`,
    `${outputDir}/hysim-network-overview-mobile.png`,
    `${outputDir}/hysim-local-svg-mobile.png`,
  ]) {
    if (!existsSync(file)) throw new Error(`missing screenshot ${file}`);
  }
  console.log(JSON.stringify({
    schema: 'hysim_network_overview_e2e_v1',
    desktop_nodes: desktop.overview.nodes,
    mobile_nodes: mobile.overview.nodes,
    local_svg_nodes: local.nodes,
    screenshots: 3,
  }));
  return 0;
}

main().then(code => process.exit(code)).catch(error => {
  console.error(error);
  process.exit(1);
});
