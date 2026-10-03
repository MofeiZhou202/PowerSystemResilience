/* Local project workspace. Stored results are read-only evidence, never backend run handles. */
'use strict';
(function (global) {
  const core = global.HySimCore = global.HySimCore || {};
  const copy = value => structuredClone(value);
  const node = (tag, text, cls = '') => {
    const el = document.createElement(tag); el.className = cls;
    if (text !== undefined) el.textContent = text;
    return el;
  };
  const button = (label, fn, primary = false) => {
    const el = node('button', label, `resilience-portal__button${primary ? ' resilience-portal__button--primary' : ''}`);
    el.type = 'button'; el.addEventListener('click', fn); return el;
  };
  const hint = text => node('p', text, 'resilience-portal__hint');
  const uuid = () => crypto.randomUUID();
  const fmt = row => Number.isFinite(row?.value)
    ? `${Number((row.unit === '1' ? row.value * 100 : row.value).toPrecision(6))} ${row.unit === '1' ? '%' : row.unit || ''}` : '—';
  const download = (name, value) => {
    const url = URL.createObjectURL(new Blob([JSON.stringify(value, null, 2)], { type: 'application/json' }));
    const a = node('a'); a.href = url; a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  class Store {
    async open() {
      if (this.db) return this.db;
      this.db = await new Promise((resolve, reject) => {
        const req = indexedDB.open('hysim-resilience-workspace', 1);
        req.onupgradeneeded = () => { req.result.createObjectStore('versions', { keyPath: 'id' }); req.result.createObjectStore('tasks', { keyPath: 'id' }); };
        req.onsuccess = () => resolve(req.result); req.onerror = () => reject(req.error);
        req.onblocked = () => reject(new Error('工作区被其他窗口占用，请关闭旧窗口后重试'));
      });
      return this.db;
    }
    async access(store, method, value) {
      const db = await this.open();
      return new Promise((resolve, reject) => {
        const tx = db.transaction(store, method === 'getAll' ? 'readonly' : 'readwrite');
        const req = tx.objectStore(store)[method](...(value === undefined ? [] : [value]));
        tx.oncomplete = () => resolve(req.result); tx.onerror = () => reject(tx.error); tx.onabort = () => reject(tx.error || new Error('保存中断'));
      });
    }
  }
  class Workspace {
    constructor(portal) {
      this.p = portal; this.store = new Store(); this.versions = []; this.tasks = []; this.selected = new Set();
      this.projectId = uuid(); this.projectName = '未命名分析'; this.demo = false; this.expert = false;
      this.search = ''; this.taskFilter = 'all'; this.busy = false; this.saving = false; this.activeTask = null;
    }
    mount() {
      const p = this.p;
      p.panelEl.inert = this.busy;
      const nav = p.root.querySelector('.resilience-portal__view-switcher');
      const addTab = (id, title, before) => {
        const tab = node('button', title, 'resilience-portal__view-tab'); tab.type = 'button';
        tab.dataset.resilienceView = id; tab.id = `resiliencePortal-${id}-tab`;
        tab.setAttribute('role', 'tab'); tab.setAttribute('aria-controls', `resiliencePortal-${id}`);
        tab.tabIndex = -1;
        before ? nav.prepend(tab) : nav.append(tab);
        const surface = node('section', undefined, 'resilience-portal__surface rp-page');
        surface.id = `resiliencePortal-${id}`; surface.setAttribute('role', 'tabpanel');
        surface.setAttribute('aria-labelledby', tab.id); surface.hidden = true; surface.inert = true;
        p.root.append(surface);
      };
      addTab('home', '首页', true); addTab('projects', '我的分析'); addTab('tasks', '计算任务'); addTab('compare', '结果对比'); addTab('help', '帮助');
      p.viewTabs = [...nav.querySelectorAll('[data-resilience-view]')];
      const actions = p.root.querySelector('.resilience-portal__header-actions');
      this.mode = button('高级模式', () => { this.expert = !this.expert; p.render(); });
      actions.prepend(this.mode);
      this.nameInput = node('input'); this.nameInput.setAttribute('aria-label', '分析名称'); this.nameInput.maxLength = 80;
      this.nameInput.value = this.projectName; this.nameInput.addEventListener('input', () => { this.projectName = this.nameInput.value.trim() || '未命名分析'; });
      const toolbar = node('div', undefined, 'rp-workbar');
      toolbar.append(this.nameInput, button('保存版本', () => this.save()), button('查看电网', () => p.activatePresentationView('architecture')));
      this.nextEl = node('div', undefined, 'rp-next');
      p.workflowSurfaceEl.prepend(toolbar, this.nextEl);
      const navigate = () => {
        const match = (p.root.dataset.initialRoute || location.hash).match(/^#resilience\/(home|workflow|architecture|projects|tasks|compare|help)(?:\/([a-z_]+))?$/);
        delete p.root.dataset.initialRoute;
        if (!match) return;
        if (match[1] === 'workflow' && core.ResiliencePortal.steps.some(s => s.id === match[2])) p.state.activeStep = match[2];
        p.activatePresentationView(match[1]); p.render();
      };
      global.addEventListener('hashchange', navigate);
      navigate();
      this.ready = this.load();
      this.wrapOperation('generateScenarios', '场景生成', 'scenarioGeneration', 'scenario_selection');
      this.wrapOperation('runRecovery', '快速恢复', 'recovery', 'rapid_recovery');
      this.wrapOperation('evaluateMetrics', '指标计算', 'metrics', 'metric_output');
      // The elapsed display reflects wall time only; no fabricated solver progress.
      this.timer = setInterval(() => {
        p.root.querySelectorAll('[data-task-start]').forEach(el => { el.textContent = `已用时 ${Math.floor((Date.now() - Number(el.dataset.taskStart)) / 1000)} s`; });
      }, 1000);
    }
    async load() {
      try {
        [this.versions, this.tasks] = await Promise.all([this.store.access('versions', 'getAll'), this.store.access('tasks', 'getAll')]);
        for (const task of this.tasks.filter(t => t.status === 'running')) {
          task.status = 'interrupted'; task.error = '页面已离开，无法确认后台执行结果。重新运行前请确认服务空闲。';
          await this.store.access('tasks', 'put', task);
        }
        this.versions.sort((a, b) => b.created - a.created); this.tasks.sort((a, b) => b.started - a.started);
        this.render();
      } catch (e) { this.storageError = e.message; this.p.setStatus(`本地工作区不可用：${e.message}`, 'error'); this.render(); }
    }
    go(step) { this.p.activateStep(step, { focus: true }); }
    next() {
      const s = this.p.state;
      if (s.model.status !== 'ready' || !s.model.currentName) return ['加载电网模型', 'scenario_selection', '选择内置算例，或导入自己的电网。指标将使用推荐设置。'];
      if (!s.selectedMetricIds.length) return ['选择计算指标', 'metric_selection', '先选择推荐指标，或自定义计算范围。'];
      if (!s.scenarioGeneration.selectedScenario) return [s.scenarioGeneration.candidates.length ? '选择灾害场景' : '生成灾害场景', 'scenario_selection', '生成后选择一个代表场景，进入恢复分析。'];
      if (!s.recovery.artifact || s.recovery.stale) return ['运行快速恢复', 'rapid_recovery', '确认恢复设置，再开始计算。'];
      if (!s.metrics.result || s.metrics.status === 'stale') return ['计算并查看指标', 'metric_output', '根据已有恢复结果计算指标，无需重复求解。'];
      return ['查看分析结果', 'metric_output', '查看指标与恢复曲线，保存版本后可进行方案对比。'];
    }
    render() {
      const p = this.p;
      p.panelEl.inert = this.busy;
      p.root.dataset.expert = String(this.expert); this.mode.textContent = this.expert ? '切回基础模式' : '高级模式';
      this.mode.setAttribute('aria-pressed', String(this.expert));
      this.nameInput.value = this.projectName;
      const [label, step, detail] = this.next();
      this.nextEl.replaceChildren(node('strong', this.demo ? '演示导览' : '建议下一步'), hint(detail), button(`前往 · ${label}`, () => this.go(step), true));
      const prev = core.ResiliencePortal.steps.findIndex(s => s.id === p.state.activeStep) - 1;
      if (prev >= 0) this.nextEl.append(button('上一步', () => this.go(core.ResiliencePortal.steps[prev].id)));
      if (this.demo) this.nextEl.append(button('结束导览', () => { this.demo = false; this.render(); }));
      this.groupAdvanced();
      p.root.querySelectorAll('.rp-workbar button').forEach(b => { b.disabled = this.busy || this.saving; });
      const view = p.state.presentationView;
      if (['home', 'projects', 'tasks', 'compare', 'help'].includes(view)) {
        const surface = p.root.querySelector(`#resiliencePortal-${view}`);
        p.disposePresentation(surface); surface.replaceChildren(); this[`render_${view}`](surface);
      }
    }
    groupAdvanced() {
      const panel = this.p.panelEl;
      if (!panel) return;
      if (!panel.querySelector('[data-portal-field="recovery-allow_mess_dispatch"]')) return;
      if (panel.querySelector('[data-rp-advanced]')) { panel.querySelectorAll('[data-rp-advanced], [data-rp-faults]').forEach(d => { d.open = this.expert; }); return; }
      const fields = ['resilience_model', 'resilience_solver', 'mip_gap', 'post_fault_reconfig_window_hr', 'mobile_storage_speed_kmh'];
      const labels = fields.map(key => panel.querySelector(`[data-portal-field="recovery-${key}"]`)?.closest('label')).filter(Boolean);
      const body = panel.querySelector('.resilience-portal__card-body');
      if (!body) return;
      if (labels.length) {
        const details = node('details', undefined, 'rp-advanced'); details.dataset.rpAdvanced = ''; details.open = this.expert;
        details.append(node('summary', '高级求解与调度参数'));
        const grid = node('div', undefined, 'resilience-portal__form-grid'); labels.forEach(el => grid.append(el)); details.append(grid);
        const faultGrid = panel.querySelector('[data-portal-field="recovery-ac_fault_branch_ids_text"]')?.closest('.resilience-portal__form-grid');
        if (faultGrid) {
          const faults = node('details', undefined, 'rp-advanced'); faults.dataset.rpFaults = ''; faults.open = this.expert;
          faultGrid.before(faults); faults.append(node('summary', '故障支路与修复时间明细'), faultGrid);
        }
        const profiles = panel.querySelector('[data-portal-evidence="scenario-profiles"]');
        if (profiles && !profiles.classList.contains('resilience-portal__callout--danger')) details.append(profiles);
        const note = body.querySelector(':scope > p.resilience-portal__hint'); if (note) details.append(note);
        body.append(details);
      }
      const faultRows = this.p.state.scenarioDerived?.structuredFaults || [];
      if (faultRows.length) {
        const links = node('details', undefined, 'rp-advanced'); links.append(node('summary', `在电网中查看故障（${faultRows.length} 条）`));
        const actions = node('div', undefined, 'resilience-portal__actions');
        faultRows.forEach(fault => actions.append(button(`${fault.equipment_type ? `${fault.equipment_type} ${fault.equipment_index} → ` : ''}${fault.branch_type || fault.domain} 支路 ${fault.branch_id ?? fault.branch_index}`, () => {
          try { this.p.activatePresentationView('architecture'); this.p.adapter.architecture.locateFault(fault); }
          catch (e) { this.p.setStatus(e.message, 'error'); }
        })));
        links.append(actions); body.append(links);
      }
    }
    heading(root, title, subtitle) { root.append(node('h2', title), hint(subtitle)); }
    async start(demo) {
      if (this.busy) return;
      if (this.p.state.model.currentName && this.p.state.model.status === 'ready' && !await this.save()) return;
      this.projectId = uuid(); this.projectName = demo ? '33 节点弹性演示' : '新的弹性分析'; this.demo = demo;
      const s = this.p.state;
      this.p.invalidateAfterModelChange(); s.model.status = 'idle'; s.model.currentName = ''; s.userOverrides = {}; s.scenarioOverrides = {};
      s.metricParameters = {}; s.approximationConsents = { apda: false, res: false };
      s.scenarioConfig = { resilience_cluster_count: demo ? 1 : 5, clustering_method: 'hybrid_kmedoids_tail_5pct', compare_baseline: !demo, intensity_levels: demo ? ['TD'] : ['TD', 'TS', 'STS', 'TY', 'STY', 'SuperTY'] };
      this.p.setSelectedMetrics(s.metricCatalog.filter(row => row.availability === 'available').map(row => row.id));
      this.go('scenario_selection');
      if (demo) {
        this.busy = true;
        try { await this.p.loadBuiltinCase('dist33_weather_mixed'); }
        finally { this.busy = false; this.p.render(); }
      }
    }
    render_home(root) {
      const hero = node('section', undefined, 'rp-hero');
      const intro = node('div'); intro.append(node('span', '电力系统弹性分析', 'rp-kicker'), node('h2', '从一次灾害出发，\n看清电网的恢复能力。'), hint('组织电网与灾害场景，分析供电恢复，比较不同应对方案。'));
      const actions = node('div', undefined, 'resilience-portal__actions');
      actions.append(button('新建分析', () => this.start(false), true), button('体验 33 节点演示', () => this.start(true)));
      actions.querySelectorAll('button').forEach(b => { b.disabled = this.busy || this.saving; });
      intro.append(actions); hero.append(intro);
      const journey = node('ol', undefined, 'rp-journey');
      [['01', '准备电网', '内置算例或导入模型'], ['02', '设置灾害', '生成并选择代表场景'], ['03', '分析恢复', '评估抢修、重构与储能'], ['04', '查看结果', '指标、曲线与方案对比']].forEach(([n, title, desc]) => {
        const li = node('li'); li.append(node('span', n), node('strong', title), hint(desc)); journey.append(li);
      });
      hero.append(journey); root.append(hero);
      const [label, step, detail] = this.next();
      const current = node('section', undefined, 'rp-current'); current.append(node('h3', this.projectName), hint(detail));
      current.append(button(`继续分析 · ${label}`, () => { if (!this.p.state.selectedMetricIds.length) this.p.setSelectedMetrics(this.p.state.metricCatalog.filter(r => r.availability === 'available').map(r => r.id)); this.go(step === 'metric_selection' ? 'scenario_selection' : step); }, true));
      root.append(current);
      const grid = node('div', undefined, 'rp-status-grid'); const s = this.p.state;
      [['电网模型', s.model.status === 'ready' ? s.model.currentName : '尚未加载'], ['灾害场景', s.scenarioGeneration.selectedScenario?.id || '尚未选择'], ['恢复计算', s.recovery.stale ? '需重新计算' : s.recovery.artifact ? '已完成' : '尚未计算'], ['计算指标', `${s.selectedMetricIds.length} 项已选`]].forEach(([name, value]) => { const card = node('article'); card.append(hint(name), node('strong', value)); grid.append(card); });
      root.append(grid); this.heading(root, '最近保存', '同一浏览器、同一地址下保存。可导出分析文件作为备份。');
      if (this.storageError) root.append(hint(`保存不可用：${this.storageError}`));
      if (!this.versions.length) root.append(hint('还没有保存的分析。从演示开始，完成后保存你的第一个版本。'));
      this.versions.slice(0, 3).forEach(v => root.append(this.versionCard(v)));
    }
    async snapshot() {
      const s = this.p.state;
      if (s.model.status !== 'ready' || !s.model.currentName) throw new Error('请先加载电网模型再保存');
      const revision = s.currentModelRevision;
      const model = await this.p.adapter.model.exportSnapshot();
      if (revision !== s.currentModelRevision) throw new Error('模型正在变化，请稍后重新保存');
      return { schema: 'resilience_workspace_v1', id: uuid(), projectId: this.projectId, name: this.projectName,
        created: Date.now(), model, state: copy({ model: s.model, scenarioConfig: s.scenarioConfig,
          baseRecoveryConfig: s.baseRecoveryConfig, userOverrides: s.userOverrides, scenarioOverrides: s.scenarioOverrides,
          scenarioDerived: s.scenarioDerived, scenarioGeneration: s.scenarioGeneration,
          selectedMetricIds: s.selectedMetricIds, metricParameters: s.metricParameters, approximationConsents: s.approximationConsents,
          recovery: s.recovery, metrics: s.metrics, activeStep: s.activeStep }), catalog: copy(s.metricCatalog) };
    }
    async save() {
      if (this.busy || this.saving) { this.p.setStatus('请等待当前操作结束再保存', 'busy'); return false; }
      this.saving = true;
      try {
        await this.ready;
        const v = await this.snapshot();
        if (new Blob([JSON.stringify(v)]).size > 32 * 1024 * 1024) throw new Error('分析文件超过 32 MiB，请缩小场景规模后保存');
        await this.store.access('versions', 'put', v);
        this.versions.unshift(v); this.p.setStatus('已保存新版本，可在“我的分析”中继续或导出', 'success'); return true;
      } catch (e) { this.p.setStatus(`保存失败：${e.message}`, 'error'); return false; }
      finally { this.saving = false; this.render(); }
    }
    validate(v) {
      if (v?.schema !== 'resilience_workspace_v1' || !v.model || typeof v.model !== 'object' || !v.state || !Array.isArray(v.state.selectedMetricIds) || !v.state.scenarioConfig || !v.state.scenarioGeneration || !v.state.scenarioDerived || !Array.isArray(v.catalog)) throw new Error('不是有效的弹性分析文件');
      if (typeof v.name !== 'string' || v.name.length > 80) throw new Error('分析名称格式不正确');
      const object = x => x && typeof x === 'object' && !Array.isArray(x);
      for (const key of ['model', 'scenarioConfig', 'baseRecoveryConfig', 'userOverrides', 'scenarioOverrides', 'scenarioDerived', 'scenarioGeneration', 'metricParameters', 'approximationConsents', 'recovery', 'metrics']) {
        if (!object(v.state[key])) throw new Error(`分析字段不完整：${key}`);
      }
      if (!Array.isArray(v.state.scenarioGeneration.candidates) || !v.state.selectedMetricIds.every(id => typeof id === 'string') || !v.catalog.every(row => object(row) && typeof row.id === 'string')) throw new Error('场景或指标目录格式不正确');
      const result = v.state.metrics.result;
      if (result && (!Array.isArray(result.results) || !result.results.every(r => object(r) && typeof r.id === 'string' && typeof r.status === 'string' && (r.value === null || Number.isFinite(r.value)) && ['limitations', 'assumptions', 'missing_dependencies'].every(k => r[k] === undefined || Array.isArray(r[k]))))) throw new Error('指标结果格式不正确');
      return v;
    }
    async restore(v, derive = false) {
      if (this.busy || this.saving) return;
      this.busy = true;
      try {
        this.validate(v);
        await this.p.importJsonFile(new File([JSON.stringify(v.model)], `${v.name}.json`, { type: 'application/json' }));
        if (this.p.state.model.status !== 'ready') throw new Error(this.p.state.model.error || '模型导入失败');
        const model = this.p.state.model;
        for (const key of ['scenarioConfig', 'baseRecoveryConfig', 'userOverrides', 'scenarioOverrides', 'scenarioDerived', 'scenarioGeneration', 'selectedMetricIds']) {
          if (v.state[key] !== undefined) this.p.state[key] = copy(v.state[key]);
        }
        // Archived results retain their original settings, but resumed runs use
        // the backend metric defaults until editable metric inputs are designed.
        this.p.state.metricParameters = {};
        this.p.state.approximationConsents = { apda: false, res: false };
        const savedMess = this.p.state.userOverrides.allow_mess_dispatch ?? this.p.state.baseRecoveryConfig.allow_mess_dispatch;
        this.p.state.baseRecoveryConfig = copy(this.p.adapter.recovery.getConfig());
        this.p.state.userOverrides = { allow_mess_dispatch: savedMess === true };
        this.p.state.scenarioOverrides = {};
        this.p.state.model = { ...model, currentName: v.state.model.currentName || v.name, source: '保存的分析版本' }; this.p.state.selectionRevision += 1;
        // Archived outputs stay in version storage; importing never revives an in-process run id.
        this.p.state.recovery = { status: 'idle', artifact: null, error: null };
        this.p.state.metrics = { status: 'idle', result: null, error: null };
        this.projectId = derive ? uuid() : v.projectId; this.projectName = derive ? `${v.name.slice(0, 74)} · 副本` : v.name;
        this.go(this.p.state.scenarioGeneration.selectedScenario ? 'rapid_recovery' : 'scenario_selection');
        this.p.setStatus('配置已恢复；历史结果可在结果对比中查看，当前分析需重新计算', 'success');
      } catch (e) { this.p.setStatus(`打开失败：${e.message}`, 'error'); }
      finally { this.busy = false; this.render(); }
    }
    versionCard(v) {
      const card = node('article', undefined, 'rp-version');
      card.append(node('strong', v.name), hint(`${new Date(v.created).toLocaleString()} · ${v.state?.metrics?.result ? '含指标结果' : v.state?.recovery?.artifact ? '含恢复结果' : '配置版本'}`));
      const actions = node('div', undefined, 'resilience-portal__actions');
      actions.append(button('继续此版本', () => this.restore(v)), button('复制为新方案', () => this.restore(v, true)), button('导出分析文件', () => download(`${v.name}.json`, v)));
      if (v.state?.metrics?.result) actions.append(button('加入对比', () => { if (this.selected.size >= 4 && !this.selected.has(v.id)) { this.p.setStatus('最多选择 4 个版本', 'error'); return; } this.selected.add(v.id); this.p.activatePresentationView('compare'); }));
      actions.querySelectorAll('button').forEach(b => { b.disabled = this.busy; }); card.append(actions); return card;
    }
    render_projects(root) {
      this.heading(root, '我的分析', '每次保存都会新增一个版本。保存位置为此浏览器；导出文件可迁移到其他电脑。');
      const actions = node('div', undefined, 'resilience-portal__actions');
      const search = node('input'); search.placeholder = '搜索分析名称'; search.setAttribute('aria-label', '搜索分析名称'); search.value = this.search;
      const list = node('div'); const populate = () => { list.replaceChildren(); this.versions.filter(v => v.name.toLowerCase().includes(this.search.toLowerCase())).forEach(v => list.append(this.versionCard(v))); if (!list.children.length) list.append(hint('没有匹配的版本。')); };
      search.addEventListener('input', () => { this.search = search.value; populate(); });
      const file = node('input'); file.type = 'file'; file.accept = '.json'; file.hidden = true;
      file.addEventListener('change', async () => {
        try {
          if (!file.files[0]) return; if (file.files[0].size > 32 * 1024 * 1024) throw new Error('文件超过 32 MiB');
          const v = this.validate(JSON.parse(await file.files[0].text()));
          v.id = uuid(); v.projectId = uuid(); v.created = Date.now(); v.imported = true;
          await this.store.access('versions', 'put', v); this.versions.unshift(v); this.render(); this.p.setStatus('分析文件已导入为独立副本', 'success');
        } catch (e) { this.p.setStatus(`导入失败：${e.message}`, 'error'); }
      });
      actions.append(search, button('保存当前版本', () => this.save()), button('导入分析文件', () => file.click()), file);
      root.append(actions, list); populate();
    }
    wrapOperation(method, label, stateKey, step) {
      const original = this.p[method].bind(this.p);
      this.p[method] = async (...args) => {
        if (this.busy || this.saving) return;
        this.busy = true;
        await this.ready;
        const task = { id: uuid(), projectId: this.projectId, name: this.projectName, label, step, status: 'running', started: Date.now() };
        this.tasks.unshift(task); this.activeTask = task;
        const persist = async () => { try { await this.store.access('tasks', 'put', task); } catch (e) { this.p.setStatus(`任务记录保存失败：${e.message}`, 'error'); } };
        await persist(); this.render();
        try { await original(...args); }
        finally {
          const state = this.p.state[stateKey]; task.ended = Date.now();
          task.status = task.cancelRequested ? 'cancel_requested' : ['success', 'ready', 'empty'].includes(state.status) ? 'success' : 'error';
          task.error = state.error || ''; task.runId = state.runId || null;
          this.activeTask = null; this.busy = false; await persist();
          if (task.status === 'success' && stateKey === 'metrics') await this.save();
          this.render();
        }
      };
    }
    render_tasks(root) {
      this.heading(root, '计算任务', '记录场景生成、恢复求解与指标计算的真实请求状态。服务未提供内部阶段进度时，仅显示运行状态和耗时。');
      const select = node('select'); select.setAttribute('aria-label', '任务状态');
      [['all', '全部状态'], ['running', '运行中'], ['success', '已完成'], ['error', '失败'], ['interrupted', '页面中断'], ['cancel_requested', '已请求取消']].forEach(([value, label]) => { const option = node('option', label); option.value = value; select.append(option); });
      select.value = this.taskFilter; select.addEventListener('change', () => { this.taskFilter = select.value; this.render(); }); root.append(select);
      const labels = { running: '运行中', success: '已完成', error: '失败', interrupted: '页面中断', cancel_requested: '已请求取消，请确认服务状态' };
      if (!this.tasks.length) root.append(hint('还没有计算任务。'));
      this.tasks.filter(t => this.taskFilter === 'all' || t.status === this.taskFilter).forEach(t => {
        const row = node('article', undefined, 'rp-version'); row.append(node('strong', `${t.name} · ${t.label}`), hint(`${labels[t.status]} · ${new Date(t.started).toLocaleString()}`));
        const elapsed = hint(`耗时 ${((t.ended - t.started) / 1000).toFixed(1)} s`); if (t.status === 'running') { elapsed.dataset.taskStart = t.started; elapsed.textContent = '运行中…'; } row.append(elapsed);
        if (t.error) row.append(hint(t.error));
        row.append(button(t.projectId === this.projectId ? '前往计算步骤' : '查看保存的分析', () => t.projectId === this.projectId ? this.go(t.step) : this.p.activatePresentationView('projects')));
        if (this.activeTask === t && t.step === 'rapid_recovery') row.append(button('请求取消计算', async () => {
          t.cancelRequested = true;
          try { const accepted = await this.p.adapter.recovery.cancel(); if (accepted === false) throw new Error('当前请求尚不能取消，请稍后重试'); this.p.setStatus('已请求取消，请等待服务结束当前计算', 'busy'); }
          catch (e) { t.cancelRequested = false; this.p.setStatus(`取消失败：${e.message}`, 'error'); }
        }));
        root.append(row);
      });
    }
    comparisonReasons(versions) {
      if (versions.length < 2) return ['请选择至少两个含指标结果的版本'];
      const first = versions[0], reasons = [];
      const canonical = value => JSON.stringify(value, function (key, item) { return item && !Array.isArray(item) && typeof item === 'object' ? Object.fromEntries(Object.keys(item).sort().map(k => [k, item[k]])) : item; });
      const fields = [
        ['电网模型', v => v.model], ['灾害场景及故障设置', v => [v.state.scenarioDerived, v.state.scenarioOverrides]],
        ['时间轴', v => v.state.recovery?.artifact?.time_axis || v.state.recovery?.artifact?.steps?.map(s => [s.hour, s.duration_hr])],
        ['指标定义', v => [v.catalog, v.state.metrics.result.definition_version]], ['历史指标设置', v => [v.state.metricParameters, v.state.approximationConsents]],
      ];
      fields.forEach(([label, get]) => { if (versions.some(v => canonical(get(v)) !== canonical(get(first)))) reasons.push(`${label}不同`); });
      if (versions.some(v => v.imported)) reasons.push('含导入的结果，未在本机验证来源');
      if (versions.some(v => v.state.metrics?.result?.stale || v.state.recovery?.stale)) reasons.push('含过期结果');
      if (versions.some(v => !v.state.recovery?.artifact?.steps?.length || !v.state.metrics.result.definition_version)) reasons.push('缺少完整运行依据');
      return reasons;
    }
    render_compare(root) {
      this.heading(root, '结果与方案对比', '选择 2～4 个已保存版本。口径一致时显示相对第一个版本的差值，缺失或近似值保留状态。');
      const picker = node('div', undefined, 'rp-compare-picker');
      this.versions.filter(v => v.state?.metrics?.result).forEach(v => {
        const label = node('label'); const input = node('input'); input.type = 'checkbox'; input.checked = this.selected.has(v.id);
        input.addEventListener('change', () => { if (input.checked && this.selected.size >= 4) { input.checked = false; this.p.setStatus('最多比较 4 个版本', 'error'); return; } input.checked ? this.selected.add(v.id) : this.selected.delete(v.id); this.render(); });
        label.append(input, document.createTextNode(`${v.name} · ${new Date(v.created).toLocaleString()}`)); picker.append(label);
      }); root.append(picker);
      const versions = [...this.selected].map(id => this.versions.find(v => v.id === id)).filter(Boolean);
      const reasons = this.comparisonReasons(versions); const comparable = !reasons.length;
      root.append(node('p', comparable ? '计算边界一致，可查看相对基准的差值。' : reasons.join('；') + '。仅并列展示，不计算差值或排名。', 'resilience-portal__callout'));
      if (!versions.length) return;
      if (typeof global.Plotly?.newPlot === 'function') {
        const chart = node('div', undefined, 'resilience-portal__chart'); chart.dataset.workspaceComparisonChart = ''; root.append(chart);
        const traces = versions.map((v, i) => {
          const a = v.state.recovery?.artifact;
          const rows = (a?.steps || []).map((r, j) => ({ t: r.time_hr ?? r.hour ?? a.time_axis?.[j], y: r.served_mw })).filter(r => Number.isFinite(r.t));
          return { name: `${i + 1} · ${v.name}`, x: rows.map(r => r.t), y: rows.map(r => Number.isFinite(r.y) ? r.y : null), type: 'scatter', mode: 'lines+markers', connectgaps: false };
        });
        const style = getComputedStyle(this.p.root);
        global.Plotly.newPlot(chart, traces, { title: '供电恢复曲线', font: { color: style.getPropertyValue('--rp-ink').trim() }, paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', xaxis: { title: '时间 (h)' }, yaxis: { title: '供电功率 (MW)' }, legend: { orientation: 'h' }, margin: { l: 60, r: 24, t: 55, b: 65 } }, { responsive: true, displaylogo: false });
      }
      const wrap = node('div', undefined, 'resilience-portal__table-wrap'); const table = node('table', undefined, 'resilience-portal__table');
      const header = node('tr'); ['指标', ...versions.map((v, i) => `${i === 0 ? '基准 · ' : ''}${v.name}`)].forEach(label => header.append(node('th', label))); table.append(header);
      const ids = [...new Set(versions.flatMap(v => v.state.metrics.result.results.map(r => r.id)))];
      ids.forEach(id => {
        const tr = node('tr'); tr.append(node('th', versions[0].catalog.find(r => r.id === id)?.name_zh || id));
        const baseline = versions[0].state.metrics.result.results.find(r => r.id === id);
        versions.forEach((v, i) => {
          const r = v.state.metrics.result.results.find(row => row.id === id);
          const valid = comparable && i && r?.status === 'computed' && baseline?.status === 'computed' && !r.censored && !baseline.censored && r.unit === baseline.unit && Number.isFinite(r.value) && Number.isFinite(baseline.value);
          const delta = valid ? Number(((r.value - baseline.value) * (r.unit === '1' ? 100 : 1)).toPrecision(6)) : null;
          const td = node('td', `${fmt(r)} · ${r?.status || '未选择'}${valid ? `（Δ ${delta} ${r.unit === '1' ? '个百分点' : r.unit || ''}）` : ''}`);
          td.title = [r?.reason_code, ...(r?.limitations || []), ...(r?.assumptions || [])].filter(Boolean).join('；'); tr.append(td);
        }); table.append(tr);
      }); wrap.append(table); root.append(wrap);
      const config = node('details'); config.append(node('summary', '查看恢复参数与计算依据差异'));
      const pre = node('pre', JSON.stringify(versions.map(v => ({ name: v.name, settings: { ...v.state.baseRecoveryConfig, ...v.state.scenarioDerived?.fields, ...v.state.scenarioOverrides, ...v.state.userOverrides }, run: v.state.recovery?.artifact?.run_id, metrics: v.state.metricParameters })), null, 2), 'resilience-portal__json'); config.append(pre);
      root.append(button('导出对比数据', () => download('resilience-comparison.json', { schema: 'resilience_comparison_v1', comparable, reasons, versions })), config);
    }
    render_help(root) {
      this.heading(root, '快速开始与使用帮助', '从准备模型到读懂指标，按下面的步骤完成第一次分析。');
      const grid = node('div', undefined, 'rp-help-grid');
      [['1 · 准备电网', '首次使用可在首页体验 33 节点演示；已有模型可在场景页导入 JSON 或 MATPOWER。', 'scenario_selection'], ['2 · 生成并选择灾害', '选择台风、暴雨内涝或雷击，设置灾害参数和代表场景数量，生成后点击“选择此场景”。', 'scenario_selection'], ['3 · 运行恢复', '核对故障、停运时间、储能及重构设置，再运行快速恢复。暴雨和雷击包含安全等待时间。主动防御当前不可用，可直接继续。', 'rapid_recovery'], ['4 · 查看结果', '计算指标后先看核心结果，再展开明细。默认灾害结束时刻取最后一次故障发生时刻。', 'metric_output']].forEach(([title, detail, step]) => { const card = node('article'); card.append(node('h3', title), hint(detail), button('前往此步骤', () => this.go(step))); grid.append(card); }); root.append(grid);
      [['为什么部分指标没有数值？', '概率、风险类指标需要多场景概率等额外数据。缺失数据保留不可用状态和原因，不填零。'], ['computed、approximate 和 censored 是什么？', '分别表示已计算、含近似、观察窗口内尚未达到恢复目标。比较时应同时看单位、时间窗口和模型限制。'], ['如何比较储能或抢修方案？', '保存基准版本，复制为新方案并修改参数，重新计算并保存，然后到结果对比选择两个版本。'], ['刷新后如何继续？', '到“我的分析”打开已保存版本；模型和参数恢复后需重新计算。原始结果仍可用于历史查看和对比。'], ['保存在哪里？', '保存在当前浏览器、当前地址的 IndexedDB 中。切换浏览器或端口不会共享；清理网站数据会删除本地记录，请导出分析文件备份。'], ['计算失败怎么办？', '在计算任务查看错误，返回对应步骤检查模型、故障与求解设置。修改后由你重新发起计算。']].forEach(([title, detail]) => { const el = node('details', undefined, 'rp-help-item'); el.append(node('summary', title), hint(detail)); root.append(el); });
    }
  }
  core.ResilienceWorkspace = Workspace;
})(window);
