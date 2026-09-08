/* Navigation and tutorials describe existing jobs, not an automatic market pipeline. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const pages = {
    marketOperation: ['运行模拟', '南方日前规则 · 周 / 月滚动 · 预测场景或手工边界', 'operation', 'market-operation'],
    marketBoundary: ['边界与单日出清', '南方日前规则 · 独立市场边界 · 单日 96 + 2 点', 'day', 'southern-market'],
    marketStudy: ['故障 / 来水对比试验', '南方日前规则 · 同初态场景对照 · 安全与研究账本随场景查看', 'study', 'market-study'],
    marketAncillary: ['云南调频', '云南 2025 调频规则研究 · 独立执行 SCUC、调频预安排和 SCED', 'ancillary', 'market-ancillary'],
    marketRealtime: ['实时市场', '南方第 3 章研究 · 5 分钟调度、15 分钟定价与状态承接', 'realtime', 'market-realtime'],
    marketBehavior: ['主体与报价', '通用 AC/DC 研究 · 当前工程电网 · 报价策略与可选重复博弈', 'generic', 'market-behavior'],
    marketInputs: ['预测与时域', '通用 AC/DC 研究 · 负荷、新能源与实时偏差', 'generic', 'market-inputs'],
    market: ['市场出清', '通用 AC/DC 研究 · 日前出清与固定日前组合的实时调度', 'generic', 'market-clearing'],
    marketSecurity: ['安全校核', '通用 AC/DC 研究 · 固定出清快照 AC 潮流与 N-1', 'generic', 'market-security'],
    marketSettlement: ['市场结算', '通用 AC/DC 研究 · 日前与实时双结算 · 仿真账本', 'generic', 'market-settlement'],
  };
  // Each step links to an existing workspace; navigation never submits or marks work complete.
  const routes = {
    day: { title: '首次使用：单日演示', scope: '推荐从小算例开始。加载算例会替换南方市场边界；已有工作请先导出边界。', steps: [
      ['选择并加载算例', '在“市场算例”选择“南方多资源两节点演示”，点击“加载市场算例”。确认摘要显示 2 节点及资源数量。', 'marketBoundary'],
      ['检查并保存边界', '在“边界数据”检查负荷预测、机组申报、来水和储能初态。首次使用保留默认值；修改后点击“保存边界”，确认修订号更新。', 'marketBoundary'],
      ['运行单日出清', '点击“南方规则出清”，等待 SCUC、SCED 和 LMP 返回。查看阶段状态；排程成功与交流安全通过分别判断。', 'marketBoundary'],
      ['检查结果并导出', '在“出清结果”选择阶段和时点，检查缺额、越限、节点价格与限制，然后“导出结果”。没有有效价格时不能进入有效结算。', 'marketBoundary'],
    ] },
    operation: { title: '周 / 月运行模拟', scope: '沿用当前南方市场边界。先完成一次单日演示，再扩大系统和时域。', steps: [
      ['1 选择算例', '加载小算例，或核对当前已保存边界。点击“下一步：设置市场边界”。', 'marketOperation', 1],
      ['2 设置市场边界', '选择“手工周 / 月边界”，设置起始日与时间尺度；第一次将倍数保持为 1。预测模式须先点击“生成一周市场边界”。核对末日之后的前瞻预测。', 'marketOperation', 2],
      ['3 运行仿真', '点击“确认边界并继续”。手工模式点击“开始仿真”；预测模式点击“逐日出清全部场景”。进度按真实日窗显示，可在日窗结束后暂停。', 'marketOperation', 3],
      ['4 结果与原因', '选择日期、时段和设备，查看功率曲线、开停机热力图、水位与 SOC。未出清或已失效的日窗不能作为有效结果。', 'marketOperation', 4],
    ] },
    study: { title: '故障 / 来水对照', scope: '独立试验任务；不与周 / 月任务自动串联。场景倍数是研究假设，不是实际发生概率。', steps: [
      ['准备基准边界', '先在“边界与单日出清”确认算例及报价；返回“对比试验”点击“重载任务”。', 'marketBoundary'],
      ['生成对比场景', '首次用 1 天、来水倍数 0.5,1,1.5、报价倍数 1，选择停运设备及故障时段，点击“生成对比场景”。', 'marketStudy'],
      ['逐日运行', '点击“2 逐日运行场景”，等待场景完成。失败日窗保留失败状态；修改基准边界后须重新生成。', 'marketStudy'],
      ['比较同日结果', '选择当前场景、对照场景、日期和“分析环节”。安全与研究结算账本均在这里查看；导出试验保留各阶段限制。', 'marketStudy'],
    ] },
    ancillary: { title: '云南调频与费用核算', scope: '本页自行执行 SCUC，不自动读取上一页的日前出清。黑启动仅提供规则说明。', steps: [
      ['载入能量边界', '在南方边界页确认机组、报价和求解器，再进入云南调频，点击“载入边界与调频配置”。', 'marketAncillary'],
      ['保存调频申报', '选择 AGC 单元和报价小时，核对需求、容量、价格及水电允许区间；点击“保存调频配置”。完整 JSON 修改后须先应用并校验。', 'marketAncillary'],
      ['执行预安排与出清', '点击“SCUC → 调频预安排 → SCED”，检查调频缺额和能量可行性。只有有效日前组合才可运行日内出清。', 'marketAncillary'],
      ['具备实测记录后核算', '展开“日内安全复核与调度记录”，录入有来源的调用和计量；满足资格后才核算、入账。月度账本位于本页，不在通用市场结算页。', 'marketAncillary'],
    ] },
    realtime: { title: '实时市场滚动', scope: '独立实时任务；不会自动消费之前查看的日前或调频结果。初态与预测须在本页核对。', steps: [
      ['载入当前边界', '点击“载入当前边界”，确认模型、起始时刻与滚动次数。首次使用 1 轮及可用求解器。', 'marketRealtime'],
      ['校验并建立仿真', '检查“完整6小时预测与实测初态”。修改 JSON 后先应用，再点击“校验边界并建立仿真”；状态应为边界已校验。', 'marketRealtime'],
      ['计算下一轮', '点击“计算下一轮”。每轮包含 24 个 5 分钟调度点和独立 8 个 15 分钟价格点，仅执行前 15 分钟并承接状态。', 'marketRealtime'],
      ['检查批次与价格', '选择滚动批次、优化窗口和设备，检查功率、缺额、库位及价格。失败时状态不推进，历史价格不足时保持未知。', 'marketRealtime'],
    ] },
    generic: { title: '通用 AC/DC 市场研究', scope: '使用“建模”中的工程电网。南方边界、梯级水量和云南调频结果不进入这条链。', steps: [
      ['准备工程电网与主体', '先在“建模 → 模型IO”加载电网，再配置主体归属及报价策略。“运行行为模拟”为可选的独立重复博弈。', 'marketBehavior'],
      ['设置预测和时域', '在“预测与时域”设置时段、负荷与新能源曲线。在“安全校核”和“市场结算”先设定校核及结算参数。', 'marketInputs'],
      ['运行日前与实时', '在“市场出清”运行日前；如需偏差双结算，再运行通用“实时/辅助市场”。这里的辅助服务不是云南调频规则。', 'market'],
      ['检查安全结论', '查看基态 AC、N-1 及事故后 AC 的独立状态。修改校核参数后“出清并校核”会重新计算。', 'marketSecurity'],
      ['查看并导出账本', '在“市场结算”查看已完成任务的资金账本。修改结算参数不会重算旧账本，须重新运行相应市场。', 'marketSettlement'],
    ] },
  };
  let topology = !matchMedia('(max-width: 900px)').matches;
  const node = (tag, text) => { const el = document.createElement(tag); el.textContent = text; return el; };
  function renderGuide() {
    const route = routes[$('marketGuideRoute').value];
    $('marketGuideScope').textContent = route.scope;
    $('marketGuideSteps').replaceChildren(...route.steps.map(([title, text, module, step]) => {
      const li = node('li', ''), heading = node('strong', title), p = node('p', text), button = node('button', '前往操作');
      button.type = 'button'; button.className = 'btn btn-sm';
      button.onclick = () => { App.setActiveModule(module); if (step) window.HySimMarketOperation?.setStep(step); toggleGuide(false); };
      li.append(heading, p, button); return li;
    }));
  }
  function toggleGuide(open) {
    $('marketQuickGuide').hidden = !open;
    $('marketTutorial').setAttribute('aria-expanded', String(open));
    if (open) { renderGuide(); $('marketQuickGuide').scrollIntoView({ block: 'start' }); $('marketGuideRoute').focus(); }
  }
  function activate(module) {
    const page = pages[module], family = document.querySelector(`.module-btn[data-module="${module}"]`)?.dataset.marketFamily;
    $('marketNavigation').hidden = !page;
    document.body.dataset.marketPage = page ? module : '';
    document.body.classList.toggle('market-topology-hidden', !topology);
    if (!page) return;
    $('marketModel').value = family;
    document.querySelectorAll('.module-btn[data-market-family]').forEach(button => {
      const hidden = button.dataset.marketFamily !== family;
      button.classList.toggle('workflow-hidden', hidden); button.setAttribute('aria-hidden', String(hidden));
    });
    $('marketPageTitle').textContent = page[0]; $('marketPagePurpose').textContent = page[1];
    $('marketTopologyToggle').closest('label').hidden = family !== 'southern';
    $('marketGuideRoute').value = page[2];
    if (!$('marketQuickGuide').hidden) renderGuide();
    if (location.hash !== `#${page[3]}`) history.replaceState(null, '', `#${page[3]}`);
  }
  for (const [value, route] of Object.entries(routes)) { const option = node('option', route.title); option.value = value; $('marketGuideRoute').append(option); }
  $('marketGuideRoute').onchange = renderGuide;
  $('marketTutorial').onclick = () => toggleGuide($('marketQuickGuide').hidden);
  $('marketPageHelp').onclick = () => toggleGuide(true);
  $('marketGuideClose').onclick = () => { toggleGuide(false); $('marketTutorial').focus(); };
  $('marketManual').onclick = () => window.HySimCore.HelpCenter.open('guides/market_simulation_workflow.zh.md');
  $('marketModel').onchange = () => App.setActiveModule($('marketModel').value === 'southern' ? 'marketOperation' : 'marketBehavior');
  $('marketTopologyToggle').checked = topology;
  $('marketTopologyToggle').onchange = () => { topology = $('marketTopologyToggle').checked; document.body.classList.toggle('market-topology-hidden', !topology); window.dispatchEvent(new Event('resize')); };
  const moduleFromHash = () => Object.entries(pages).find(([, page]) => location.hash === `#${page[3]}`)?.[0];
  const direct = () => { const module = moduleFromHash(); if (module && document.querySelector('.module-btn.active')?.dataset.module !== module) App.setActiveModule(module); };
  async function readResponse(response) {
    if (response.status === 404) throw new Error('当前服务版本未提供此市场接口。请使用更新后的 GUI 服务，再点击载入重试；现有任务不会因此被替换。');
    let data;
    try { data = await response.json(); }
    catch { throw new Error(`市场服务未返回有效数据（HTTP ${response.status}）。请检查服务状态，再点击载入重试。`); }
    if (!response.ok) throw new Error(data.error || `市场请求失败（HTTP ${response.status}）`);
    return data;
  }
  window.HySimMarketNavigation = { activate, readResponse, moduleFromHash };
  window.addEventListener('hashchange', direct);
  window.addEventListener('load', direct, { once: true });
})();
