// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');

function arg(name) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length ? process.argv[index + 1] : null;
}

function freePort() {
  return new Promise((resolve, reject) => {
    const server = createServer();
    server.on('error', reject);
    server.listen(0, '127.0.0.1', () => {
      const address = server.address();
      server.close(() => resolve(typeof address === 'object' && address ? address.port : 0));
    });
  });
}

async function waitUp(base) {
  for (let attempt = 0; attempt < 100; attempt += 1) {
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
  const serverPath = arg('server');
  assert(serverPath, '--server is required');
  const port = Number(await freePort());
  const base = `http://127.0.0.1:${port}`;
  const server = spawn(path.resolve(serverPath), [
    '--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(ROOT, 'data'),
  ], { cwd: ROOT, stdio: 'ignore' });
  let browser;
  try {
    await waitUp(base);
    const { chromium } = await import('playwright');
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined');

    const result = await page.evaluate(async () => {
      Canvas.loadFromSystemJson({
        name: 'Transformer topology link', base_mva: 100,
        ac: {
          buses: [
            { index: 1, name: 'HV', bus_type: 'SLACK', base_kv: 110 },
            { index: 2, name: 'MV', bus_type: 'PQ', base_kv: 10 },
            { index: 3, name: 'LV', bus_type: 'PQ', base_kv: 0.4 },
          ],
          branches: [{
            index: 12, name: 'Main transformer branch', from_bus: 1, to_bus: 2,
            branch_kind: 'transformer', r_pu: 0.01, x_pu: 0.08, b_pu: 0,
            rate_a_mva: 10, tap: 1.02, shift_deg: 0, in_service: true,
          }],
          transformers_2w: [
            { index: 21, name: 'Distribution transformer', hv_bus: 2, lv_bus: 3,
              sn_mva: 0.4, vn_hv_kv: 10, vn_lv_kv: 0.4,
              vk_percent: 4, vkr_percent: 1, in_service: true },
            { index: 22, name: 'Main transformer', hv_bus: 1, lv_bus: 2,
              source_branch_idx: 12, sn_mva: 10, vn_hv_kv: 110, vn_lv_kv: 10,
              vk_percent: 8, vkr_percent: 1, in_service: true },
          ],
          generators: [], loads: [], external_grids: [], static_generators: [],
          storage: [], renewable_gens: [], pv_systems: [], switches: [],
          circuit_breakers: [], motors: [], flexible_loads: [], asymmetric_loads: [],
          shunts: [], transformers_3w: [], chargers: [], charging_stations: [],
        },
        dc: { buses: [], branches: [], loads: [] },
      }, { forceRender: true });
      App.switchTab('topology');
      await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      const table = document.getElementById('trafoTableInner');
      const maps = Canvas.getCompBusMap();
      const rows = [21, 22].map(index => {
        const position = table.__vctx.items.findIndex(item => Number(item.index) === index);
        const row = table.querySelector(`tbody tr[data-row="${position}"]`);
        row?.dispatchEvent(new MouseEvent('click', { bubbles: true }));
        return {
          index, position, rowCompId: Number(row?.dataset.compId),
          mapCompId: Number(maps.trafo[index]), selectedId: Number(Canvas.state.selectedId),
          glyphType: Canvas.getComponent(Number(row?.dataset.compId))?.type,
        };
      });
      const branchGlyph = Canvas.state.components.find(component =>
        component.type === 'transformer_2w' && component.params?._from_branch);
      return { rows, branchGlyphId: Number(branchGlyph?.id),
        branchMapId: Number(maps.branch[12]) };
    });

    for (const row of result.rows) {
      assert(row.position >= 0 && Number.isInteger(row.rowCompId),
        `transformer ${row.index} topology row has no component link`);
      assert(row.rowCompId === row.mapCompId && row.selectedId === row.rowCompId,
        `transformer ${row.index} row did not select its mapped component`);
      assert(row.glyphType === 'transformer_2w',
        `transformer ${row.index} mapped to ${row.glyphType}`);
    }
    assert(result.branchGlyphId === result.branchMapId &&
      result.rows.find(row => row.index === 22)?.rowCompId === result.branchGlyphId,
    'branch and rich-transformer identities did not alias the same glyph');
    console.log(`transformer topology links passed: ${JSON.stringify(result)}`);
  } finally {
    if (browser) await browser.close();
    server.kill();
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
