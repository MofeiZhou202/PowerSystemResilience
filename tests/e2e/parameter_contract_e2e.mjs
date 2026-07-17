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
    const initial = await jsonRequest(base, '/api/session/parameter_library');
    const expectedCount = initial.rules.length;
    if (expectedCount < 52) throw new Error(`expected at least 52 rules, got ${expectedCount}`);
    if (initial.parameter_contract?.registered_count !== expectedCount) {
      throw new Error('registry count does not match the returned rule set');
    }
    for (const id of ['reliability.ac_branch.failure_rate',
                      'reliability.generator.forced_outage_rate',
                      'reliability.microgrid.mtbf_hours']) {
      if (!initial.rules.some(rule => rule.id === id)) {
        throw new Error(`${id}: reliability rule missing from GUI contract`);
      }
    }
    for (const rule of initial.rules) {
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
    const page = await browser.newPage();
    await page.goto(`${base}/xjtu/`, { waitUntil: 'networkidle' });
    await page.evaluate(() => App.setActiveModule('parameterLibrary'));
    await page.waitForFunction(expected =>
      document.querySelectorAll('#parameterLibraryTable tbody tr[data-rule-id]').length === expected,
      expectedCount);
    const gui = await page.evaluate(async () => {
      const rows = [...document.querySelectorAll('#parameterLibraryTable tbody tr[data-rule-id]')];
      const editable = rows.filter(row => {
        const input = row.querySelector('input[data-field="default_value"]');
        return input && !input.disabled;
      });
      const first = editable[0];
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
          return { rows: rows.length, editable: editable.length, id, next,
                   effective: data.effective_parameters.values[id].value };
        }
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      return { rows: rows.length, editable: editable.length, id, next, effective: null };
    });
    if (gui.rows !== expectedCount || gui.editable !== expectedCount) {
      throw new Error(`GUI coverage ${gui.editable}/${gui.rows}, expected ${expectedCount}/${expectedCount}`);
    }
    if (Math.abs(gui.effective - gui.next) > 1e-12) {
      throw new Error(`${gui.id}: GUI edit did not reach effective API value`);
    }
    console.log(`parameter contract passed: defaults=${expectedCount}, JSON=${expectedCount}, API=${expectedCount}, GUI=${expectedCount}, effective=${expectedCount}`);
  } finally {
    if (browser) await browser.close();
    proc.kill();
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
