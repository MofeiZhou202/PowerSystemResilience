// Scratch (safe to delete): confirm ELK layout runs in a Web Worker.
import { chromium } from 'playwright';
import { spawn } from 'node:child_process';
import net from 'node:net';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const SERVER = path.join(ROOT, 'build/macos-release/run_gui_server');
const freePort = () => new Promise((res, rej) => { const s = net.createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => res(p)); }); s.on('error', rej); });
async function waitReady(base, ms = 25000) { const t0 = Date.now(); while (Date.now() - t0 < ms) { try { const r = await fetch(`${base}/api/cases`); if (r.ok) return true; } catch {} await new Promise(r => setTimeout(r, 300)); } return false; }

const port = await freePort();
const base = `http://127.0.0.1:${port}`;
if (!existsSync(SERVER)) { console.error('server missing'); process.exit(2); }
const server = spawn(SERVER, ['--host', '127.0.0.1', '--port', String(port), '--data-dir', path.join(ROOT, 'data'), '--matpower-dir', path.join(ROOT, 'data')], { cwd: ROOT, stdio: 'ignore' });
let browser;
try {
  if (!await waitReady(base)) { console.error('not ready'); process.exit(2); }
  browser = await chromium.launch();
  const page = await browser.newPage();
  await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
  page.on('worker', (w) => console.log('WORKER_CREATED', w.url().split('/').pop()));
  await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => !!(window.HySimCore && HySimCore.LayoutEngine), null, { timeout: 30000 });
  const res = await page.evaluate(async () => {
    const contract = { graph: { id: 'root', layoutOptions: { 'elk.algorithm': 'layered' },
      children: [{ id: 'component-1', width: 40, height: 40 }, { id: 'component-2', width: 40, height: 40 }],
      edges: [{ id: 'e1', sources: ['component-1'], targets: ['component-2'] }] } };
    const r = await HySimCore.LayoutEngine.layout(contract, { timeoutMs: 8000 });
    return { worker: r.worker, runtime_ms: Math.round(r.runtime_ms), positions: Object.keys(r.positions).length };
  });
  console.log('LAYOUT', JSON.stringify(res));
} finally {
  if (browser) await browser.close();
  server.kill('SIGKILL');
}
