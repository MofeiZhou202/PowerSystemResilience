// @ts-check
import { spawn } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync, mkdirSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');

function arg(name, fallback = null) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length
    ? process.argv[index + 1] : fallback;
}

function serverPath() {
  const explicit = arg('server');
  if (explicit) return path.resolve(explicit);
  for (const relative of ['build/macos-release/tests/run_gui_server',
                          'build/macos-release/run_gui_server']) {
    const candidate = path.join(ROOT, relative);
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
  for (let attempt = 0; attempt < 100; ++attempt) {
    try {
      if ((await fetch(`${base}/api/cases`)).ok) return;
    } catch { /* server is still starting */ }
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  throw new Error('GUI server did not start');
}

async function main() {
  const { chromium } = await import('playwright');
  const port = Number(arg('port')) || Number(await freePort());
  const base = `http://127.0.0.1:${port}`;
  const proc = spawn(serverPath(), [
    '--host', '127.0.0.1', '--port', String(port),
    '--data-dir', path.join(ROOT, 'data'),
    '--matpower-dir', path.join(ROOT, 'data'),
  ], { cwd: ROOT, stdio: 'ignore' });
  let browser;
  try {
    await waitUp(base);
    browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 1440, height: 960 } });
    await page.addInitScript(() => { try { localStorage.setItem('hysim.tourDone.v1', '1'); } catch {} });
    const pageErrors = [];
    page.on('pageerror', error => pageErrors.push(error.message));
    await page.route('https://cdn.plot.ly/**', route => route.fulfill({
      contentType: 'application/javascript',
      body: `(() => {
        const el = target => typeof target === 'string' ? document.getElementById(target) : target;
        const draw = target => { const node = el(target); if (node) node.classList.add('js-plotly-plot'); return Promise.resolve(node); };
        window.Plotly = { newPlot: draw, react: draw, purge: target => { const node = el(target); if (node) node.classList.remove('js-plotly-plot'); }, Plots: { resize() {} } };
      })();`,
    }));
    await page.goto(`${base}/xjtu/`, { waitUntil: 'domcontentloaded', timeout: 60000 });
    await page.waitForTimeout(1000);
    const appReady = await page.evaluate(() => {
      try { return typeof App !== 'undefined' && typeof App.loadMatpowerCase === 'function'; }
      catch { return false; }
    });
    if (!appReady) throw new Error(`App failed to initialize: ${pageErrors.join(' | ') || 'no page error captured'}`);
    const builtin = arg('builtin');
    if(builtin) await page.evaluate(name=>App.loadBuiltinCase(name),builtin);
    else await page.evaluate(() => App.loadMatpowerCase('case9.m'));
    if(builtin==='market_ieee118') {
      const snapshot=await page.evaluate(()=>Canvas.buildSystemJson());
      const counts={};for(const g of snapshot.ac.generators)counts[g.fuel_type]=(counts[g.fuel_type]||0)+1;
      if(JSON.stringify(counts)!==JSON.stringify({Hydro:36,Coal:18,Wind:6,Solar:6}))throw new Error('Canvas lost mixed fuel types: '+JSON.stringify(counts));
      for(const g of snapshot.ac.generators)if(g.fuel_type==='Coal' && (g.min_up_time_hr!==1||g.min_dn_time_hr!==1))throw new Error('Canvas lost thermal minimum times');
    }
    await page.evaluate(() => App.setActiveModule('marketBehavior'));

    const module = page.locator('#moduleMarketBehavior');
    if (!(await module.evaluate(element => element.classList.contains('active')))) {
      throw new Error('market module did not become active');
    }
    if (!(await page.locator('[data-sub="market"]').isVisible())) {
      throw new Error('market controls are not visible');
    }
    if (!(await page.locator('#workflowMarket').evaluate(element =>
      element.classList.contains('active')))) {
      throw new Error('electricity market was not promoted to an active workflow');
    }
    const marketModules = await page.locator('.module-btn[data-group="market"]')
      .evaluateAll(elements => elements.map(element => element.textContent.trim()));
    if (JSON.stringify(marketModules) !== JSON.stringify([
      '运行模拟', '边界与单日出清', '对比试验', '云南调频', '实时市场', '主体与报价', '预测与时域', '市场出清', '安全校核', '市场结算'])) {
      throw new Error(`market workflow order is wrong: ${JSON.stringify(marketModules)}`);
    }
    if (await page.locator('#marketModel').inputValue() !== 'generic') throw new Error('generic market family not selected');
    if (await page.locator('#southernMarketWorkspace').isVisible()) throw new Error('Southern boundary leaked into generic market');

    await page.locator('#btnMarketParticipants').click();
    await page.locator('#marketParticipantModal').waitFor({ state: 'visible' });
    const strategicRow = builtin ? 0 : 2;
    await page.locator(`[data-market-row="${strategicRow}"][data-market-field="participant_id"]`).fill('firm_a');
    await page.locator(`[data-market-row="${strategicRow}"][data-market-field="participant_name"]`).fill('Firm A');
    await page.locator(`[data-market-row="${strategicRow}"][data-market-field="type"]`).selectOption('markup_and_withholding');
    await page.locator(`[data-market-row="${strategicRow}"][data-market-field="energy_markup_percent"]`).fill('15');
    await page.locator(`[data-market-row="${strategicRow}"][data-market-field="capacity_withholding_percent"]`).fill('10');
    await page.locator(`[data-market-row="${strategicRow}"][data-market-field="upward_reserve_price_per_mwh"]`).fill('2');
    await page.locator('#btnMarketParticipantsApply').click();
    await page.locator('#marketParticipantModal').waitFor({ state: 'hidden' });

    await page.evaluate(() => App.setActiveModule('marketInputs'));
    if (!(await page.locator('#marketNumSteps').isVisible()) ||
        !(await page.locator('#btnMarketImportLoadProfile').isVisible()) ||
        !(await page.locator('#marketLoadForecastStatus').isVisible()) ||
        (await page.locator('#marketEnableN1').isVisible())) {
      throw new Error('market boundary step mixes controls from another stage');
    }
    await page.locator('#marketNumSteps').fill('1');
    await page.locator('#marketRealizedLoadDeviationPct').fill('5');

    await page.evaluate(() => App.setActiveModule('market'));
    if (!(await page.locator('#marketScucNetworkGeneration').isVisible()) ||
        !(await page.locator('#marketScucCrossRoundStateReuse').isVisible()) ||
        !(await page.locator('#marketScucInSolveNetworkGeneration').isVisible()) ||
        !(await page.locator('#marketScucNetworkGenerationMinCandidates').isVisible()) ||
        !(await page.locator('#marketScucNetworkGenerationIterations').isVisible()) ||
        !(await page.locator('#marketScucNetworkGenerationMaxNew').isVisible())) {
      throw new Error('SCUC network constraint generation controls are missing');
    }
    await page.locator('#marketScucNetworkGeneration').check();
    await page.locator('#marketScucCrossRoundStateReuse').check();
    await page.locator('#marketScucInSolveNetworkGeneration').check();
    await page.locator('#marketScucNetworkGenerationMinCandidates').fill('0');

    await page.evaluate(() => App.setActiveModule('marketSecurity'));
    await page.locator('#marketAcValidation').check();
    await page.locator('#marketEnableN1').check();
    const responsePromise = page.waitForResponse(response =>
      response.url().includes('/api/session/run_market_clearing') &&
      response.request().method() === 'POST', { timeout: 120000 });
    await page.locator('#btnRunMarketSecurity').click();
    const response = await responsePromise;
    const market = await response.json();
    if (!response.ok()) throw new Error(`market endpoint failed: ${market.error || response.status()}`);
    if(builtin)for(const keyword of ['AC storage','AC flexible loads','reservoir']) {
      if(!market.model_scope?.limitations?.some(s=>s.includes(keyword)))throw new Error('Missing generic resource limitation: '+keyword);
    }
    if (!market.security?.enabled || !market.security?.lodf_available) {
      throw new Error('N-1 security result is missing from the market response');
    }
    if (!Array.isArray(market.security.trajectory) || !market.security.trajectory.length) {
      throw new Error('N-1 cut trajectory is missing');
    }
    if (!market.security.full_component_validation_run ||
        !Array.isArray(market.security.component_checks) ||
        !market.security.component_checks.length ||
        !market.model_scope?.full_component_n1_modelled ||
        !String(market.model_scope?.n1_recourse_policy || '').includes('corrective')) {
      throw new Error('full-component corrective N-1 result is missing');
    }
    if (!(market.performance?.estimated_scuc_variables > 0) ||
        !(market.performance?.estimated_sced_variables > 0) ||
        !(market.performance?.estimated_lodf_sparse_bytes > 0) ||
        !(market.performance?.lodf_computed_columns > 0) ||
        !market.performance?.component_n1_parallel_requested ||
        !(market.performance?.component_n1_parallel_workers >= 1) ||
        !market.performance?.scuc_network_constraint_generation_run ||
        !market.performance?.scuc_network_constraint_generation_converged ||
        !(market.performance?.scuc_network_constraint_candidates > 0) ||
        market.performance?.scuc_cross_round_solver_state_reuse_enabled !== true ||
        typeof market.performance?.scuc_in_solve_network_constraint_generation_used !== 'boolean' ||
        typeof market.performance?.scuc_search_tree_rebuilt !== 'boolean' ||
        market.performance?.scuc_network_constraint_remaining_violations !== 0 ||
        !market.commitment?.network_constraint_generation_run ||
        !market.commitment?.network_constraint_generation_converged ||
        !(market.performance?.total_sec > 0) ||
        !market.performance?.scuc_solver_name ||
        !market.performance?.pricing_solver_name) {
      throw new Error('market performance profile is missing');
    }
    if (!market.ac_validation?.[0]?.converged ||
        !Array.isArray(market.ac_validation?.[0]?.violations)) {
      throw new Error('automatic post-clearing AC power-flow diagnostics are missing');
    }
    const firm = (market.participant_settlement || [])
      .find(row => row.participant_id === 'firm_a');
    if (!firm || !(firm.offered_capacity_mw < firm.physical_capacity_mw)) {
      throw new Error('strategic capacity withholding did not reach market settlement');
    }
    await page.waitForFunction(() =>
      document.querySelector('#resultsContent')?.dataset.activeGroup === 'market' &&
      document.querySelector('#marketSummary')?.textContent?.includes('市场状态'));
    await page.waitForFunction(() =>
      document.querySelector('#marketLmpChart')?.classList.contains('js-plotly-plot'));
    const rendered = await page.evaluate(() => ({
      summary: document.getElementById('marketSummary')?.textContent || '',
      pricingRows: document.querySelectorAll('#marketPricingResults tbody tr').length,
      participantRows: document.querySelectorAll('#marketParticipantResults tbody tr').length,
      securityRows: document.querySelectorAll('#marketSecurityResults tbody tr').length,
      performanceRows: document.querySelectorAll('#marketPerformanceResults tbody tr').length,
      failureDetails: document.getElementById('marketFailureDetails')?.textContent || '',
      lmpChart: !!document.querySelector('#marketLmpChart.js-plotly-plot'),
    }));
    const expectedFailureText = market.security.full_component_contingencies_secure
      ? '未发现不可行设备或越限约束' : '全元件N-1纠正校核';
    if (!rendered.summary.includes('全元件N-1') ||
        !rendered.summary.includes('资金残差') || rendered.pricingRows !== 1 ||
        rendered.participantRows < 1 || rendered.securityRows < 1 ||
        rendered.performanceRows < 10 ||
        !rendered.failureDetails.includes(expectedFailureText) ||
        !rendered.lmpChart) {
      throw new Error(`market dashboard incomplete: ${JSON.stringify(rendered)}`);
    }

    await page.locator('#marketEnableN1').uncheck();
    await page.evaluate(() => App.setActiveModule('marketSettlement'));
    await page.locator('#marketGeneratorImbalanceTolerancePct').fill('1');
    await page.locator('#marketLoadImbalanceTolerancePct').fill('2');
    await page.evaluate(() => App.setActiveModule('market'));
    await page.locator('#marketAncillaryEnabled').check();
    await page.locator('#marketReservePerformancePct').fill('0');
    const realTimeResponsePromise = page.waitForResponse(response =>
      response.url().includes('/api/session/run_real_time_market') &&
      response.request().method() === 'POST', { timeout: 120000 });
    await page.locator('#btnRunRealTimeMarket').click();
    const realTimeResponse = await realTimeResponsePromise;
    let realTime = await realTimeResponse.json();
    if(builtin && realTime.status==='invalid_day_ahead_baseline' && realTime.day_ahead?.status==='ac_validation_failed') {
      const dir=path.join(ROOT,'output/market-operation',builtin+'-generic');mkdirSync(dir,{recursive:true});
      writeFileSync(path.join(dir,'ac-rejected-settlement.json'),JSON.stringify({market,realTime},null,2));
      // Keep the AC rejection evidence; this second experiment validates the
      // financial ledger conditional on linear dispatch, without AC certification.
      await page.evaluate(()=>App.setActiveModule('marketSecurity'));
      await page.locator('#marketAcValidation').uncheck();
      await page.evaluate(()=>App.setActiveModule('market'));
      const conditional=page.waitForResponse(r=>r.url().includes('/api/session/run_real_time_market')&&r.request().method()==='POST',{timeout:120000});
      await page.locator('#btnRunRealTimeMarket').click();
      realTime=await (await conditional).json();
    }
    if (!realTimeResponse.ok() || !realTime.feasible ||
        !(realTime.total_absolute_generator_deviation_mwh > 0) ||
        !realTime.ancillary_services_enabled ||
        !(realTime.periods?.[0]?.reserve_instruction_mw > 0) ||
        !(realTime.periods?.[0]?.reserve_shortfall_mw > 0) ||
        !(realTime.settlement?.customer_imbalance_penalty > 0) ||
        !(realTime.settlement?.resource_reserve_nonperformance_charge > 0) ||
        Math.abs(Number(realTime.settlement?.cashflow_residual)) > 1e-6) {
      throw new Error(`real-time two-settlement result invalid: ${JSON.stringify(realTime)}`);
    }
    await page.waitForFunction(() =>
      document.getElementById('marketRealTimeSection')?.style.display === 'block' &&
      document.querySelector('#marketRealTimePriceChart')?.classList.contains('js-plotly-plot') &&
      document.querySelector('#marketReservePerformanceChart')?.classList.contains('js-plotly-plot') &&
      document.getElementById('marketRealTimeLedgerResults')?.textContent?.includes('运营方辅助服务余额'));

    await page.evaluate(() => App.setActiveModule('marketSettlement'));
    await page.locator('#btnMarketViewSettlement').click();
    if (!(await page.locator('#marketSettlementResults').isVisible()) ||
        (await page.locator('#marketPricingResults').isVisible()) ||
        !(await page.locator('#marketRealTimeLedgerResults').isVisible())) {
      throw new Error('settlement step does not isolate settlement results');
    }

    await page.evaluate(() => App.setActiveModule('marketBehavior'));
    await page.locator('#marketGameRounds').fill('2');
    const gameResponsePromise = page.waitForResponse(response =>
      response.url().includes('/api/session/run_repeated_market_game') &&
      response.request().method() === 'POST', { timeout: 120000 });
    await page.locator('#btnRunMarketGame').click();
    const gameResponse = await gameResponsePromise;
    const game = await gameResponse.json();
    if(builtin){
      const dir=path.join(ROOT,'output/market-operation',builtin+'-generic');mkdirSync(dir,{recursive:true});
      writeFileSync(path.join(dir,'workflow.json'),JSON.stringify({market,realTime,game},null,2));
    }
    const strategicChanged = (game.rounds || []).some(round =>
      (round.participants || []).some(row =>
        row.participant_id === 'firm_a' && row.strategy_changed));
    const finalRoundChanged = (game.rounds?.at(-1)?.participants || [])
      .some(row => row.strategy_changed);
    const stationaryFirm=game.rounds?.at(-1)?.participants?.find(row=>row.participant_id==='firm_a');
    const certifiedStationary=!!builtin && game.converged && stationaryFirm &&
      Number.isFinite(stationaryFirm.profit) && Number.isFinite(stationaryFirm.best_response_profit) &&
      Math.abs(stationaryFirm.best_response_profit-stationaryFirm.profit)<=1e-4 &&
      stationaryFirm.best_response_improvement<=1e-4;
    if (!gameResponse.ok() || !game.feasible || !(game.rounds || []).length ||
        (!strategicChanged && !certifiedStationary) ||
        (finalRoundChanged &&
          (game.converged || game.status !== 'maximum_rounds_reached')) ||
        !game.final_two_settlement?.ancillary_services_enabled ||
        Math.abs(Number(game.final_two_settlement?.settlement?.cashflow_residual)) > 1e-6) {
      throw new Error(`repeated market game result invalid: ${JSON.stringify(game)}`);
    }
    await page.waitForFunction(() =>
      document.getElementById('marketGameSection')?.style.display === 'block' &&
      document.querySelector('#marketGameProfitChart')?.classList.contains('js-plotly-plot'));

    const hybridApi = await page.evaluate(async () => {
      const post = async (url, body) => {
        const response = await fetch(url, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        });
        const data = await response.json();
        return { ok: response.ok, status: response.status, data };
      };
      const loaded = await post('/api/session/load_builtin', {
        case: 'market_5bus_acdc_toy',
      });
      if (!loaded.ok) return { loaded };
      const cleared = await post('/api/session/run_market_clearing', {
        num_steps: 1,
        reserve_fraction: 0,
        network_constraints: true,
        run_ac_validation: true,
        enable_n1_security: false,
      });
      return { loaded, cleared };
    });
    const hybrid = hybridApi.cleared?.data;
    if (!hybridApi.loaded?.ok || !hybridApi.cleared?.ok || !hybrid?.feasible ||
        !hybrid.model_scope?.dc_network_modelled ||
        !hybrid.model_scope?.energy_prices_valid ||
        !hybrid.ac_validation?.[0]?.converged ||
        !Array.isArray(hybrid.pricing?.[0]?.dc_lmp_per_mwh) ||
        hybrid.pricing[0].dc_lmp_per_mwh.length === 0 ||
        !Array.isArray(hybrid.pricing?.[0]?.vsc_ac_injection_mw) ||
        hybrid.pricing[0].vsc_ac_injection_mw.length === 0 ||
        !Array.isArray(hybrid.pricing?.[0]?.dc_storage_dispatch_mw) ||
        !Array.isArray(hybrid.pricing?.[0]?.dc_storage_soc_mwh) ||
        !Array.isArray(hybrid.dc_storage_settlement) ||
        !(hybrid.performance?.estimated_scuc_variables > 0) ||
        (hybrid.unsupported_assets || []).length !== 0) {
      throw new Error(`hybrid market API contract invalid: ${JSON.stringify(hybridApi)}`);
    }
    if (pageErrors.length) {
      throw new Error(`browser runtime errors: ${pageErrors.join(' | ')}`);
    }
    console.log(`market GUI passed: DA=${market.status}, RT=${realTime.status}, game=${game.status}, hybrid=${hybrid.status}, rounds=${game.rounds.length}`);
  } finally {
    if (browser) await browser.close();
    proc.kill();
  }
}

main().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
