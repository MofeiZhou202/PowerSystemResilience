// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const STORAGE_KEY = 'hysimWorkspaceLayoutV1';

function arg(name) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length ? process.argv[index + 1] : null;
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
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

async function settle(page) {
  await page.evaluate(() => new Promise(resolve =>
    requestAnimationFrame(() => requestAnimationFrame(resolve))));
}

async function workspaceSnapshot(page) {
  return page.evaluate(() => {
    const rect = id => {
      const box = document.getElementById(id)?.getBoundingClientRect();
      return box ? { width: box.width, height: box.height } : { width: 0, height: 0 };
    };
    const display = id => getComputedStyle(document.getElementById(id)).display;
    return {
      layout: App.getWorkspaceLayout(),
      classes: document.body.className,
      canvas: rect('canvasContainer'),
      libraryDisplay: display('componentLib'),
      ribbonDisplay: display('subToolbar'),
      inspectorDisplay: display('rightPanel'),
      consoleDisplay: display('console'),
      consoleHeight: rect('console').height,
      consoleLogHtml: document.getElementById('consoleLog')?.innerHTML || '',
      activeModule: document.querySelector('.module-btn.active')?.dataset.module || '',
      activeTab: document.querySelector('.panel-tab.active[data-tab]')?.dataset.tab || '',
      pageOverflow: Math.max(0, document.documentElement.scrollWidth - window.innerWidth),
      pressed: Object.fromEntries([
        'btnToggleLibrary', 'btnToggleRibbon', 'btnToggleInspector', 'btnToggleConsole',
        'btnWorkspaceFocus', 'btnDensityStandard', 'btnDensityCompact',
      ].map(id => [id, document.getElementById(id)?.getAttribute('aria-pressed') || ''])),
    };
  });
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
    const context = await browser.newContext({ viewport: { width: 1440, height: 1000 } });
    const page = await context.newPage();
    const pageErrors = [];
    page.on('pageerror', error => pageErrors.push(error.message));
    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App !== 'undefined' && typeof Canvas !== 'undefined');
    await page.evaluate(key => localStorage.removeItem(key), STORAGE_KEY);
    await page.reload({ waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App?.getWorkspaceLayout === 'function');
    await settle(page);

    let state = await workspaceSnapshot(page);
    assert(state.layout.density === 'compact', 'first desktop load must use compact density');
    assert(state.layout.consoleCollapsed === true && state.consoleHeight <= 30,
      'console must start collapsed without discarding its header');
    assert(state.pressed.btnDensityCompact === 'true' && state.pressed.btnToggleConsole === 'false',
      'default density and console controls have incorrect pressed state');

    await page.evaluate(() => {
      App.setActiveModule('shortCircuit');
      App.switchTab('topology');
      App.log('workspace persistence marker', 'info');
    });
    await page.click('#btnToggleConsole');
    await settle(page);
    const expanded = await workspaceSnapshot(page);
    assert(expanded.consoleHeight > 80 && expanded.consoleLogHtml.includes('workspace persistence marker'),
      'expanding the console must preserve and reveal existing log entries');

    for (const [button, className, hiddenKey] of [
      ['#btnToggleLibrary', 'library-collapsed', 'libraryDisplay'],
      ['#btnToggleRibbon', 'ribbon-collapsed', 'ribbonDisplay'],
      ['#btnToggleInspector', 'inspector-collapsed', 'inspectorDisplay'],
    ]) {
      await page.click(button);
      await settle(page);
      state = await workspaceSnapshot(page);
      assert(state.classes.includes(className) && state[hiddenKey] === 'none',
        `${button} did not hide its dock region`);
      await page.click(button);
      await settle(page);
      state = await workspaceSnapshot(page);
      assert(!state.classes.includes(className) && state[hiddenKey] !== 'none',
        `${button} did not restore its dock region`);
    }

    const beforeFocus = await workspaceSnapshot(page);
    await page.click('#btnWorkspaceFocus');
    await settle(page);
    const focused = await workspaceSnapshot(page);
    assert(focused.layout.focus === true && focused.pressed.btnWorkspaceFocus === 'true',
      'focus mode state was not applied');
    assert(focused.canvas.width >= beforeFocus.canvas.width + 120,
      `focus mode did not materially increase canvas width: ${beforeFocus.canvas.width} -> ${focused.canvas.width}`);
    assert(focused.canvas.height >= beforeFocus.canvas.height + 100,
      `focus mode did not materially increase canvas height: ${beforeFocus.canvas.height} -> ${focused.canvas.height}`);
    assert(focused.activeModule === 'shortCircuit' && focused.activeTab === 'topology',
      'focus mode changed the active module or inspector tab');

    await page.keyboard.press('Control+Shift+F');
    await settle(page);
    const restored = await workspaceSnapshot(page);
    assert(restored.layout.focus === false && restored.activeModule === 'shortCircuit' &&
      restored.activeTab === 'topology', 'focus shortcut did not restore the prior workspace context');
    assert(restored.consoleLogHtml.includes('workspace persistence marker'),
      'focus mode discarded console output');

    await page.click('#btnDensityStandard');
    await page.click('#btnToggleLibrary');
    await page.click('#btnToggleRibbon');
    await page.click('#btnToggleInspector');
    await page.click('#btnToggleConsole');
    await page.reload({ waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App?.getWorkspaceLayout === 'function');
    await settle(page);
    const persisted = await workspaceSnapshot(page);
    assert(persisted.layout.density === 'standard' && persisted.layout.libraryCollapsed &&
      persisted.layout.ribbonCollapsed && persisted.layout.inspectorCollapsed &&
      persisted.layout.consoleCollapsed, 'workspace layout did not persist across reload');
    assert(persisted.pressed.btnDensityStandard === 'true',
      'persisted standard-density control state is incorrect');

    await page.evaluate(() => App.setWorkspaceLayout({
      density: 'standard', libraryCollapsed: false, ribbonCollapsed: false,
      inspectorCollapsed: false, consoleCollapsed: false, focus: false,
    }));
    await page.reload({ waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App?.getWorkspaceLayout === 'function');
    await settle(page);
    const standard = await workspaceSnapshot(page);
    assert(!standard.layout.libraryCollapsed && !standard.layout.ribbonCollapsed &&
      !standard.layout.inspectorCollapsed && !standard.layout.consoleCollapsed,
    'restored standard workspace must expose every dock region');

    await page.setViewportSize({ width: 390, height: 844 });
    await page.evaluate(key => localStorage.removeItem(key), STORAGE_KEY);
    await page.reload({ waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof App?.getWorkspaceLayout === 'function');
    await settle(page);
    const mobileDefault = await workspaceSnapshot(page);
    assert(mobileDefault.layout.density === 'compact' && mobileDefault.layout.libraryCollapsed &&
      mobileDefault.layout.consoleCollapsed, 'mobile first load must collapse secondary docks');
    assert(mobileDefault.pageOverflow === 0, `mobile workspace overflowed by ${mobileDefault.pageOverflow}px`);

    await page.evaluate(() => App.setActiveModule('parameterLibrary'));
    await page.locator('#parameterModelExplorer').waitFor({ state: 'visible' });
    await settle(page);
    const mobileParameters = await workspaceSnapshot(page);
    assert(mobileParameters.inspectorDisplay !== 'none' && mobileParameters.activeModule === 'parameterLibrary',
      'model parameters are not reachable in the mobile compact workspace');
    assert(mobileParameters.pageOverflow === 0,
      `mobile model-parameter workspace overflowed by ${mobileParameters.pageOverflow}px`);
    assert(pageErrors.length === 0, `page errors: ${pageErrors.join(' | ')}`);

    console.log(`workspace layout passed: ${JSON.stringify({
      compact: beforeFocus.canvas,
      focus: focused.canvas,
      mobileOverflow: mobileParameters.pageOverflow,
    })}`);
  } finally {
    if (browser) await browser.close();
    server.kill();
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
