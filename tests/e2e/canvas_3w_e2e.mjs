// @ts-check
// canvas_3w_e2e.mjs — browser end-to-end test for the HySim canvas editor.
//
// Builds a 3-winding-transformer feeder ON THE CANVAS (place components + wire
// the hv/mv/lv windings to buses, attach a utility source and loads), syncs the
// canvas to the backend and runs power flow — exactly the gesture a user would
// perform with drag-and-drop, driven through the same `Canvas` / REST API.
//
// Requirements (not part of the C++ build):
//   npm i -D playwright && npx playwright install chromium
//
// Usage:
//   node tests/e2e/canvas_3w_e2e.mjs \
//        [--server build/tests/run_gui_server] [--data-dir data] [--port 0]
//
// Exit code 0 on success, 1 on any failed assertion.  Starts (and stops) the
// GUI server itself unless --base-url points at an already-running instance.

import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync } from 'node:fs';
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

    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    await page.goto(base + '/xjtu/', { waitUntil: 'networkidle' });
    await page.waitForTimeout(400);

    const result = await page.evaluate(async () => {
      Canvas.clearAll();
      const hv = Canvas.addComponent('ac_bus', 200, 120, { name: 'HVB', bus_type: 'SLACK', base_kv: 220, vm_pu: 1.0, vmin_pu: 0.9, vmax_pu: 1.1, in_service: true, area: 1, zone: 1 });
      const mv = Canvas.addComponent('ac_bus', 100, 320, { name: 'MVB', bus_type: 'PQ', base_kv: 110, vm_pu: 1.0, vmin_pu: 0.9, vmax_pu: 1.1, in_service: true, area: 1, zone: 1 });
      const lv = Canvas.addComponent('ac_bus', 300, 320, { name: 'LVB', bus_type: 'PQ', base_kv: 35, vm_pu: 1.0, vmin_pu: 0.9, vmax_pu: 1.1, in_service: true, area: 1, zone: 1 });
      const grid = Canvas.addComponent('external_grid', 200, 40, { name: 'U1', vm_pu: 1.0, va_deg: 0, in_service: true });
      const t3 = Canvas.addComponent('transformer_3w', 200, 220);
      const ldM = Canvas.addComponent('load', 100, 420, { name: 'LD_MV', p_mw: 30, q_mvar: 10, scaling: 1.0, in_service: true });
      const ldL = Canvas.addComponent('load', 300, 420, { name: 'LD_LV', p_mw: 15, q_mvar: 5, scaling: 1.0, in_service: true });
      Canvas.addConnection(grid.id, 'bottom', hv.id, 'top');
      Canvas.addConnection(t3.id, 'hv', hv.id, 'bottom');
      Canvas.addConnection(t3.id, 'mv', mv.id, 'top');
      Canvas.addConnection(t3.id, 'lv', lv.id, 'top');
      Canvas.addConnection(ldM.id, 'top', mv.id, 'bottom');
      Canvas.addConnection(ldL.id, 'top', lv.id, 'bottom');

      const sys = Canvas.buildSystemJson();
      const counts = {
        ac_buses: (sys.ac?.buses || []).length,
        transformers_3w: (sys.ac?.transformers_3w || []).length,
        external_grids: (sys.ac?.external_grids || []).length,
        loads: (sys.ac?.loads || []).length,
        connections: Canvas.state.connections.length,
      };
      const origin = window.location.origin;
      const lr = await fetch(origin + '/api/session/load_json_string', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ json_string: JSON.stringify(sys) }),
      });
      const lj = await lr.json();
      const pr = await fetch(origin + '/api/session/pf', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ method: 'pure_ac', options: {} }),
      });
      const pj = await pr.json();
      return { counts, backend_3w: lj.counts?.transformers_3w, pf_status: pr.status, pf_converged: pj.converged };
    });

    console.log('canvas 3W e2e:', JSON.stringify(result));
    check(result.counts.ac_buses === 3, 'canvas has 3 AC buses');
    check(result.counts.transformers_3w === 1, 'canvas has 1 three-winding transformer');
    check(result.counts.connections === 6, 'canvas has 6 wired connections');
    check(result.backend_3w === 1, 'backend received the 3-winding transformer');
    check(result.pf_status === 200 && result.pf_converged === true, 'power flow converged');
  } finally {
    await browser.close();
    if (proc) proc.kill();
  }

  console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  return failures === 0 ? 0 : 1;
}

main().then((code) => process.exit(code)).catch((e) => { console.error(e); process.exit(1); });
