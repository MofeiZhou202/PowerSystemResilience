/* Resilience Edition product portal. Presentation/state only: all analysis and
   scientific values come through the App adapter and backend contracts. */
'use strict';

(function initResiliencePortal(global) {
  const core = global.HySimCore = global.HySimCore || {};
  const STEPS = Object.freeze([
    { id: 'metric_selection', label: '指标选择', meta: '书中指标与运行指标', chapter: 'Chapter 3 · 多维弹性评估' },
    { id: 'scenario_selection', label: '场景生成', meta: '完整聚类场景集', chapter: '场景工程 · 候选与证据' },
    { id: 'proactive_defense', label: '主动防御', meta: '聚类场景集整体规划', chapter: '资源配置 · 跨场景规划' },
    { id: 'rapid_recovery', label: '快速恢复', meta: '选择代表场景查看过程', chapter: '恢复评估 · 后端运行' },
    { id: 'metric_output', label: '指标输出', meta: '后端 evaluator 结果', chapter: '结果证据 · 指标输出' },
  ]);
  const STEP_INDEX = new Map(STEPS.map((step, index) => [step.id, index]));
  const PHASE_LABELS = Object.freeze({
    pre_disaster: '灾前坚强程度',
    during_disaster: '灾中抵抗能力',
    post_disaster: '灾后恢复能力',
    operational: '本次运行 · 工程扩展指标',
  });
  const STATUS_LABELS = Object.freeze({
    idle: '待开始', loading: '加载中', ready: '就绪', running: '运行中',
    computed: '已计算', approximate: '近似', unavailable: '不可用',
    not_applicable: '不适用', not_requested: '未请求', invalid: '无效',
    stale: '已过期', success: '完成', error: '错误',
  });
  const SCENARIO_RECOVERY_FIELDS = Object.freeze([
    'fault_count', 'ac_fault_branch_ids_text', 'dc_fault_branch_ids_text',
    'ac_fault_start_hours_text', 'dc_fault_start_hours_text',
    'ac_repair_durations_text', 'dc_repair_durations_text',
  ]);

  const clone = value => {
    if (value === undefined) return undefined;
    try { return structuredClone(value); } catch (_) { return JSON.parse(JSON.stringify(value)); }
  };
  const text = value => value === null || value === undefined || value === '' ? '—' : String(value);
  const jsonText = value => {
    try { return JSON.stringify(value, null, 2); } catch (_) { return text(value); }
  };

  class Portal {
    constructor(root, adapter) {
      this.root = root;
      this.adapter = adapter;
      this.stepsEl = root.querySelector('#resiliencePortalSteps');
      this.panelEl = root.querySelector('#resiliencePortalPanel');
      this.evidenceEl = root.querySelector('#resiliencePortalEvidenceBody');
      this.statusEl = root.querySelector('#resiliencePortalStatus');
      this.chapterEl = root.querySelector('#resiliencePortalChapter');
      this.stateEl = root.querySelector('#resiliencePortalState');
      this.mobileStepsEl = root.querySelector('#resiliencePortalMobileSteps');
      this.workflowSurfaceEl = root.querySelector('#resiliencePortalWorkflowSurface');
      this.architectureSurfaceEl = root.querySelector('#resiliencePortalArchitectureSurface');
      const switcher = this.root.querySelector('.resilience-portal__view-switcher');
      this.viewTabs = switcher
        ? [...switcher.querySelectorAll('[data-resilience-view]')]
        : [];
      const initialScenarioConfig = clone(adapter.scenario?.getConfig?.() || {
        resilience_cluster_count: 5,
        clustering_method: 'hybrid_kmedoids_tail_5pct',
        compare_baseline: true,
        intensity_levels: ['TD', 'TS', 'STS', 'TY', 'STY', 'SuperTY'],
      });
      const initialRecoveryConfig = clone(adapter.recovery?.getConfig?.() || {});
      this.state = {
        edition: 'resilience', activeStep: 'metric_selection', presentationView: 'home', metricCatalog: [],
        selectedMetricIds: [], selectionRevision: 0, metricParameters: {},
        approximationConsents: { apda: false, res: false },
        model: {
          status: 'idle', cases: [], matpowerFiles: [], selectedCase: 'dist33_weather_mixed',
          selectedMatpower: '', currentName: '', source: '', counts: null, error: null,
        },
        scenarioConfig: initialScenarioConfig,
        baseRecoveryConfig: initialRecoveryConfig,
        scenarioDerived: { fields: {}, structuredFaults: [], profiles: null, identity: null, provenance: {} },
        scenarioOverrides: {},
        userOverrides: {},
        scenarioGeneration: { status: 'idle', requestId: null, data: null, candidates: [], selectedScenario: null, scenarioRevision: 0, error: null },
        proactiveDefense: { status: 'idle', addGenerator: true, addMobileStorage: true, result: null, error: null },
        recovery: { status: 'idle', requestId: null, runId: null, runRevision: null, modelRevision: null, scenarioRevision: null, artifact: null, error: null },
        metrics: { status: 'idle', requestId: null, runId: null, selectionRevision: null, result: null, error: null },
        currentModelRevision: Number(adapter.getModelRevision?.() || 0), portalRequestRevision: 0,
      };
      this.bound = false;
      this.workspace = new core.ResilienceWorkspace(this);
    }

    mount() {
      if (this.bound) return;
      this.bound = true;
      this.root.hidden = false;
      this.root.inert = false;
      this.root.removeAttribute('aria-hidden');
      this.adapter.architecture?.mount?.();
      this.workspace.mount();
      this.activatePresentationView(this.state.presentationView);
      this.bindChrome();
      this.state.metricCatalog = clone(this.adapter.getMetricCatalog?.() || []);
      this.adapter.onStatusChange?.((message, state) => this.setStatus(message, state));
      this.render();
      this.refreshCatalog();
      this.loadModelCatalog();
      this.adapter.onModelRevisionChange?.(revision => this.onModelRevisionChange(revision));
    }

    bindChrome() {
      this.root.querySelector('#resiliencePortalTheme')?.addEventListener('click', () => {
        document.getElementById('btnToggleTheme')?.click();
      });
      this.root.querySelector('.resilience-portal__view-switcher')?.addEventListener('click', event => {
        const button = event.target.closest('[data-resilience-view]');
        if (button) this.activatePresentationView(button.dataset.resilienceView, { focus: true });
      });
      this.root.querySelector('.resilience-portal__view-switcher')?.addEventListener('keydown', event => {
        const button = event.target.closest('[data-resilience-view]');
        if (!button) return;
        const order = this.viewTabs.map(tab => tab.dataset.resilienceView);
        const current = order.indexOf(button.dataset.resilienceView);
        let next = null;
        if (event.key === 'ArrowRight' || event.key === 'ArrowDown') next = (current + 1) % order.length;
        if (event.key === 'ArrowLeft' || event.key === 'ArrowUp') next = (current + order.length - 1) % order.length;
        if (event.key === 'Home') next = 0;
        if (event.key === 'End') next = order.length - 1;
        if (next === null) return;
        event.preventDefault();
        this.activatePresentationView(order[next], { focus: true });
      });
      this.root.querySelector('#resiliencePortalEvidenceToggle')?.addEventListener('click', event => {
        const body = this.root.querySelector('#resiliencePortalEvidenceBody');
        const button = event.currentTarget;
        const collapsed = body?.hidden === true;
        if (body) body.hidden = !collapsed;
        button.textContent = collapsed ? '⌃' : '⌄';
        button.setAttribute('aria-expanded', String(collapsed));
      });
      this.stepsEl?.addEventListener('click', event => {
        const button = event.target.closest('[data-resilience-step]');
        if (button) this.activateStep(button.dataset.resilienceStep, { focus: true });
      });
      this.stepsEl?.addEventListener('keydown', event => {
        const button = event.target.closest('[data-resilience-step]');
        if (!button) return;
        const current = STEP_INDEX.get(button.dataset.resilienceStep);
        let next = null;
        if (event.key === 'ArrowDown' || event.key === 'ArrowRight') next = Math.min(STEPS.length - 1, current + 1);
        if (event.key === 'ArrowUp' || event.key === 'ArrowLeft') next = Math.max(0, current - 1);
        if (event.key === 'Enter' || event.key === ' ') {
          event.preventDefault();
          this.activateStep(button.dataset.resilienceStep, { focus: true });
          return;
        }
        if (event.key === 'Escape') {
          event.preventDefault();
          this.activateStep(this.state.activeStep, { focus: true });
          return;
        }
        if (event.key === 'Home') next = 0;
        if (event.key === 'End') next = STEPS.length - 1;
        if (next === null) return;
        event.preventDefault();
        this.activateStep(STEPS[next].id, { focus: true });
      });
      this.mobileStepsEl?.addEventListener('change', event => {
        if (event.target.matches('select')) this.activateStep(event.target.value, { focus: true });
      });
    }

    async refreshCatalog() {
      try {
        const catalog = await this.adapter.refreshMetricCatalog?.();
        if (Array.isArray(catalog) && catalog.length) {
          this.state.metricCatalog = clone(catalog);
          this.render();
        }
      } catch (error) {
        this.setStatus(`指标目录刷新失败：${error.message || error}`, 'error');
      }
    }

    async loadModelCatalog() {
      try {
        const [cases, matpowerFiles] = await Promise.all([
          this.adapter.model?.listCases?.(),
          this.adapter.model?.listMatpower?.(),
        ]);
        this.state.model.cases = clone(cases?.cases || []);
        this.state.model.selectedCase = cases?.defaultCase || this.state.model.selectedCase;
        if (this.state.model.cases.some(row => row.id === 'dist33_weather_mixed')) {
          this.state.model.selectedCase = 'dist33_weather_mixed';
        }
        this.state.model.matpowerFiles = clone(matpowerFiles || []);
        if (this.state.model.status !== 'loading') this.state.model.status = this.state.model.currentName ? 'ready' : 'idle';
        this.render();
      } catch (error) {
        this.state.model.status = 'error';
        this.state.model.error = error.message || String(error);
        this.render();
      }
    }

    async loadBuiltinCase(caseName) {
      this.state.model.status = 'loading';
      this.state.model.error = null;
      this.render();
      try {
        const result = await this.adapter.model.loadBuiltin(caseName);
        this.state.model.error = null;
        this.state.model.selectedCase = caseName;
        this.state.model.currentName = caseName;
        this.state.model.source = '内置算例';
        this.state.model.counts = clone(result?.counts || null);
        this.state.model.status = 'ready';
        this.state.currentModelRevision = Number(result?.modelRevision ?? this.adapter.getModelRevision?.() ?? this.state.currentModelRevision);
        this.invalidateAfterModelChange();
        this.setStatus(`已加载内置算例：${caseName}`, 'success');
      } catch (error) {
        this.state.model.status = 'error';
        this.state.model.error = error.message || String(error);
        this.setStatus(`内置算例加载失败：${this.state.model.error}`, 'error');
      }
      this.render();
    }

    async loadMatpowerCase(filename) {
      this.state.model.status = 'loading';
      this.state.model.error = null;
      this.render();
      try {
        const result = await this.adapter.model.loadMatpower(filename);
        this.state.model.selectedMatpower = filename;
        this.state.model.currentName = filename;
        this.state.model.source = 'MATPOWER';
        this.state.model.counts = clone(result?.counts || null);
        this.state.model.status = 'ready';
        this.state.currentModelRevision = Number(result?.modelRevision ?? this.adapter.getModelRevision?.() ?? this.state.currentModelRevision);
        this.invalidateAfterModelChange();
        this.setStatus(`已加载 MATPOWER：${filename}`, 'success');
      } catch (error) {
        this.state.model.status = 'error';
        this.state.model.error = error.message || String(error);
        this.setStatus(`MATPOWER 加载失败：${this.state.model.error}`, 'error');
      }
      this.render();
    }

    async importJsonFile(file) {
      if (!file) return;
      this.state.model.status = 'loading';
      this.state.model.error = null;
      this.render();
      try {
        const result = await this.adapter.model.importJson(file);
        this.state.model.currentName = result?.name || file.name;
        this.state.model.source = 'JSON';
        this.state.model.counts = clone(result?.counts || null);
        this.state.model.status = 'ready';
        this.state.currentModelRevision = Number(result?.modelRevision ?? this.adapter.getModelRevision?.() ?? this.state.currentModelRevision);
        this.invalidateAfterModelChange();
        this.setStatus(`已导入 JSON：${file.name}`, 'success');
      } catch (error) {
        this.state.model.status = 'error';
        this.state.model.error = error.message || String(error);
        this.setStatus(`JSON 导入失败：${this.state.model.error}`, 'error');
      }
      this.render();
    }

    invalidateAfterModelChange() {
      this.state.scenarioGeneration = { status: 'idle', requestId: null, data: null, candidates: [], selectedScenario: null, scenarioRevision: this.state.scenarioGeneration.scenarioRevision + 1, error: null };
      this.state.scenarioDerived = { fields: {}, structuredFaults: [], profiles: null, identity: null, provenance: {} };
      this.state.scenarioOverrides = {};
      this.state.proactiveDefense = { ...this.state.proactiveDefense, status: 'idle', result: null, error: null };
      this.state.recovery = { status: 'idle', requestId: null, runId: null, runRevision: null, modelRevision: null, scenarioRevision: null, artifact: null, error: null };
      this.state.metrics = { status: 'idle', requestId: null, runId: null, selectionRevision: null, result: null, error: null };
    }
    onModelRevisionChange(revision) {
      const next = Number(revision);
      if (!Number.isFinite(next) || next === this.state.currentModelRevision) return;
      this.state.currentModelRevision = next;
      if (this.state.proactiveDefense.result) {
        this.state.proactiveDefense.status = 'stale';
        this.state.proactiveDefense.error = '电网模型已修改，请重新生成场景集并规划';
      }
      if (this.state.recovery.artifact) {
        this.state.recovery.stale = true;
        this.state.metrics.status = 'stale';
        this.state.metrics.error = '模型已修改，旧恢复结果不能作为当前指标输入';
      }
      this.render();
      this.setStatus('模型已修改：恢复结果已标记为过期，请重新运行', 'stale');
    }

    activatePresentationView(view, { focus = false } = {}) {
      if (!['home', 'workflow', 'architecture', 'projects', 'tasks', 'compare', 'help'].includes(view)) return;
      this.state.presentationView = view;
      const architecture = view === 'architecture';
      if (this.workflowSurfaceEl) {
        this.workflowSurfaceEl.hidden = view !== 'workflow';
        this.workflowSurfaceEl.inert = view !== 'workflow';
        this.workflowSurfaceEl.setAttribute('aria-hidden', String(view !== 'workflow'));
      }
      if (this.architectureSurfaceEl) {
        this.architectureSurfaceEl.hidden = !architecture;
        this.architectureSurfaceEl.inert = !architecture;
        this.architectureSurfaceEl.setAttribute('aria-hidden', String(!architecture));
      }
      this.viewTabs.forEach(tab => {
        const selected = tab.dataset.resilienceView === view;
        tab.setAttribute('aria-selected', String(selected));
        tab.tabIndex = selected ? 0 : -1;
      });
      ['home', 'projects', 'tasks', 'compare', 'help'].forEach(id => {
        const surface = this.root.querySelector(`#resiliencePortal-${id}`);
        if (surface) { surface.hidden = id !== view; surface.inert = id !== view; surface.setAttribute('aria-hidden', String(id !== view)); }
      });
      this.adapter.architecture?.setActive?.(architecture);
      this.workspace?.render();
      const hash = `#resilience/${view}${view === 'workflow' ? '/' + this.state.activeStep : ''}`;
      if (location.hash !== hash) history.pushState(null, '', hash);
      if (focus) this.viewTabs.find(tab => tab.dataset.resilienceView === view)?.focus({ preventScroll: true });
    }

    effectiveRecoveryConfig() {
      return {
        ...this.state.baseRecoveryConfig,
        ...(this.state.scenarioDerived?.fields || {}),
        allow_mess_dispatch: this.state.userOverrides.allow_mess_dispatch ?? this.state.baseRecoveryConfig.allow_mess_dispatch ?? false,
      };
    }

    updateRecoveryField(key, value) {
      if (key !== 'allow_mess_dispatch') return;
      if (this.effectiveRecoveryConfig()[key] !== value && this.state.recovery.artifact) {
        this.state.recovery.stale = true;
        this.state.metrics.status = 'stale';
      }
      this.state.userOverrides.allow_mess_dispatch = value === true;
    }

    recoveryFieldSource(key) {
      if (Object.prototype.hasOwnProperty.call(this.state.scenarioOverrides, key)) return '当前场景的用户覆盖';
      if (Object.prototype.hasOwnProperty.call(this.state.userOverrides, key)) return '用户设置';
      if (Object.prototype.hasOwnProperty.call(this.state.scenarioDerived?.fields || {}, key)) return '来自当前场景';
      return '模型/默认设置';
    }

    disposePresentation(root = this.panelEl) {
      this.adapter.presentation?.dispose?.(root);
    }

    activateStep(id, { focus = false } = {}) {
      if (!STEP_INDEX.has(id)) return;
      this.state.activeStep = id;
      this.activatePresentationView('workflow');
      this.render();
      const scroller = this.panelEl?.closest('.resilience-portal__main');
      if (scroller) scroller.scrollTop = 0;
      if (focus) {
        const panel = this.panelEl;
        panel?.focus({ preventScroll: true });
      }
    }

    setStatus(message, state = '') {
      if (!this.statusEl) return;
      this.statusEl.textContent = message;
      this.statusEl.dataset.state = state;
    }

    render() {
      this.renderStepper();
      this.renderPanel();
      this.renderEvidence();
      this.workspace?.render();
    }

    renderStepper() {
      if (!this.stepsEl) return;
      this.stepsEl.replaceChildren();
      STEPS.forEach((step, index) => {
        const li = document.createElement('li');
        const button = document.createElement('button');
        button.type = 'button';
        button.className = 'resilience-portal__step';
        button.dataset.resilienceStep = step.id;
        button.id = `resiliencePortalTab-${step.id}`;
        button.setAttribute('role', 'tab');
        button.setAttribute('aria-controls', 'resiliencePortalPanel');
        button.setAttribute('aria-selected', String(this.state.activeStep === step.id));
        if (this.state.activeStep === step.id) button.setAttribute('aria-current', 'step');
        const number = document.createElement('span');
        number.className = 'resilience-portal__step-number';
        number.setAttribute('aria-hidden', 'true');
        number.textContent = String(index + 1);
        const copy = document.createElement('span');
        copy.className = 'resilience-portal__step-copy';
        const label = document.createElement('span');
        label.className = 'resilience-portal__step-label';
        label.textContent = step.label;
        const meta = document.createElement('span');
        meta.className = 'resilience-portal__step-meta';
        meta.textContent = this.stepMeta(step.id) || step.meta;
        copy.append(label, meta);
        button.append(number, copy);
        li.append(button);
        this.stepsEl.append(li);
      });
      if (this.mobileStepsEl) {
        this.mobileStepsEl.hidden = false;
        this.mobileStepsEl.replaceChildren();
        const select = document.createElement('select');
        select.setAttribute('aria-label', '选择弹性工作流步骤');
        STEPS.forEach(step => {
          const option = document.createElement('option');
          option.value = step.id; option.textContent = step.label; option.selected = this.state.activeStep === step.id;
          select.append(option);
        });
        this.mobileStepsEl.append(select);
      }
    }

    stepMeta(id) {
      if (id === 'metric_selection') return `${this.state.selectedMetricIds.length} 项已选 · v${this.state.selectionRevision}`;
      if (id === 'scenario_selection') return `${this.state.scenarioGeneration.candidates.length} 个代表簇 · v${this.state.scenarioGeneration.scenarioRevision}`;
      if (id === 'proactive_defense') return STATUS_LABELS[this.state.proactiveDefense.status] || this.state.proactiveDefense.status;
      if (id === 'rapid_recovery') return STATUS_LABELS[this.state.recovery.status] || this.state.recovery.status;
      if (id === 'metric_output') return STATUS_LABELS[this.state.metrics.status] || this.state.metrics.status;
      return '';
    }

    renderPanel() {
      if (!this.panelEl) return;
      this.disposePresentation(this.panelEl);
      this.panelEl.replaceChildren();
      const step = STEPS[STEP_INDEX.get(this.state.activeStep) || 0];
      if (this.chapterEl) this.chapterEl.textContent = step.chapter;
      if (this.stateEl) {
        this.stateEl.textContent = STATUS_LABELS[this.currentStepStatus()] || this.currentStepStatus();
        this.stateEl.dataset.status = this.currentStepStatus();
      }
      const title = document.createElement('h2');
      title.className = 'resilience-portal__panel-title'; title.id = 'resiliencePortalPanelTitle'; title.textContent = step.label;
      this.panelEl.setAttribute('aria-labelledby', title.id);
      this.panelEl.append(title);
      if (this.state.activeStep === 'metric_selection') this.renderMetricSelection();
      else if (this.state.activeStep === 'scenario_selection') this.renderScenarioSelection();
      else if (this.state.activeStep === 'proactive_defense') this.renderProactiveDefense();
      else if (this.state.activeStep === 'rapid_recovery') this.renderRapidRecovery();
      else this.renderMetricOutput();
    }

    currentStepStatus() {
      if (this.state.activeStep === 'metric_selection') return this.state.metricCatalog.length ? 'ready' : 'loading';
      if (this.state.activeStep === 'scenario_selection') return this.state.scenarioGeneration.status;
      if (this.state.activeStep === 'proactive_defense') return this.state.proactiveDefense.status;
      if (this.state.activeStep === 'rapid_recovery') return this.state.recovery.status;
      return this.state.metrics.status;
    }

    lead(message) {
      const p = document.createElement('p'); p.className = 'resilience-portal__panel-lead'; p.textContent = message; this.panelEl.append(p);
    }

    sectionHeading(title, detail = '') {
      const row = document.createElement('div'); row.className = 'resilience-portal__section-heading';
      const h = document.createElement('h3'); h.textContent = title; row.append(h);
      if (detail) { const span = document.createElement('span'); span.textContent = detail; row.append(span); }
      this.panelEl.append(row);
    }

    renderMetricSelection() {
      this.lead('书中第三章覆盖灾前风险、灾中抵抗和灾后恢复。工程扩展指标直接利用本次恢复结果，补充电量、时长、分级损失与资源使用情况。概率类指标仍需多场景求解数据。');
      const actions = document.createElement('div'); actions.className = 'resilience-portal__actions';
      const count = document.createElement('span'); count.className = 'resilience-portal__hint'; count.textContent = `已选择 ${this.state.selectedMetricIds.length} / ${this.state.metricCatalog.length} 项 · selection_revision ${this.state.selectionRevision}`;
      const all = document.createElement('button'); all.type = 'button'; all.className = 'resilience-portal__button resilience-portal__button--quiet'; all.textContent = '选择全部';
      all.addEventListener('click', () => this.setSelectedMetrics(this.state.metricCatalog.map(item => item.id)));
      const clear = document.createElement('button'); clear.type = 'button'; clear.className = 'resilience-portal__button'; clear.textContent = '清空选择';
      clear.addEventListener('click', () => this.setSelectedMetrics([]));
      const recommended = document.createElement('button'); recommended.type = 'button'; recommended.className = 'resilience-portal__button resilience-portal__button--primary'; recommended.textContent = '选择本次可算指标';
      recommended.addEventListener('click', () => this.setSelectedMetrics(this.state.metricCatalog.filter(item => item.availability === 'available').map(item => item.id)));
      actions.append(count, recommended, all, clear); this.panelEl.append(actions);
      const byPhase = new Map();
      this.state.metricCatalog.forEach(item => { const key = item.phase || 'other'; if (!byPhase.has(key)) byPhase.set(key, []); byPhase.get(key).push(item); });
      byPhase.forEach((items, phase) => {
        this.sectionHeading(PHASE_LABELS[phase] || phase, `${items.length} 项`);
        const grid = document.createElement('div'); grid.className = 'resilience-portal__grid';
        items.forEach(item => grid.append(this.metricCard(item)));
        this.panelEl.append(grid);
      });
    }

    metricCard(item) {
      const card = document.createElement('article'); card.className = 'resilience-portal__card resilience-portal__metric-card';
      const selected = this.state.selectedMetricIds.includes(item.id);
      card.dataset.selected = String(selected); card.dataset.availability = item.availability || '';
      const body = document.createElement('div'); body.className = 'resilience-portal__card-body';
      const label = document.createElement('label'); label.className = 'resilience-portal__metric-select';
      const input = document.createElement('input'); input.type = 'checkbox'; input.checked = selected; input.value = item.id;
      input.setAttribute('aria-label', `选择 ${item.name_zh || item.symbol || item.id}`);
      input.addEventListener('change', () => {
        const ids = new Set(this.state.selectedMetricIds); input.checked ? ids.add(item.id) : ids.delete(item.id);
        this.setSelectedMetrics([...ids]);
      });
      const names = document.createElement('span');
      const title = document.createElement('strong'); title.className = 'resilience-portal__card-title'; title.textContent = `${item.symbol || item.id} · ${item.name_zh || '未命名指标'}`;
      const subtitle = document.createElement('span'); subtitle.className = 'resilience-portal__card-subtitle'; subtitle.textContent = item.name_en || '';
      names.append(title, subtitle); label.append(input, names);
      const status = document.createElement('span'); status.className = 'resilience-portal__status-badge'; status.dataset.status = item.availability || 'unavailable'; status.textContent = item.availability === 'available' ? '可计算' : (item.availability === 'conditional' ? '需补充条件' : '数据不足');
      const header = document.createElement('div'); header.className = 'resilience-portal__card-header'; header.append(label, status);
      const details = document.createElement('dl'); details.className = 'resilience-portal__metric-meta';
      [['公式', item.formula_ref], ['单位', item.unit], ['方向', ({ higher_is_better: '越高越好', lower_is_better: '越低越好', neutral: '描述量' })[item.direction] || item.direction], ['范围', item.calculation_scope], ['依赖', (item.required_inputs || []).join(' · ')], ['限制', (item.limitations || []).join('；')]].forEach(([key, value]) => {
        const dt = document.createElement('dt'); dt.textContent = key; const dd = document.createElement('dd'); dd.textContent = text(value); details.append(dt, dd);
      });
      body.append(header, details); card.append(body); return card;
    }

    setSelectedMetrics(ids) {
      const known = new Set(this.state.metricCatalog.map(item => item.id));
      const selected = [...new Set(ids)].filter(id => known.has(id));
      if (JSON.stringify(selected) === JSON.stringify(this.state.selectedMetricIds)) return;
      this.state.selectedMetricIds = selected;
      this.state.selectionRevision += 1;
      this.state.metrics = { status: 'idle', requestId: null, runId: null, selectionRevision: null, result: null, error: null };
      this.render();
      this.setStatus(selected.length ? `已更新指标选择：${selected.length} 项` : '尚未选择指标，后续执行已锁定', selected.length ? 'success' : '');
    }

    renderModelControls() {
      this.sectionHeading('模型与算例', this.state.model.currentName
        ? `${this.state.model.source} · ${this.state.model.currentName}`
        : '先加载或导入一个电网模型');
      const card = document.createElement('section'); card.className = 'resilience-portal__card';
      const body = document.createElement('div'); body.className = 'resilience-portal__card-body';
      const grid = document.createElement('div'); grid.className = 'resilience-portal__form-grid';

      const caseLabel = document.createElement('label'); caseLabel.className = 'resilience-portal__field'; caseLabel.textContent = '内置算例';
      const caseSelect = document.createElement('select'); caseSelect.dataset.portalField = 'builtin-case';
      const casePlaceholder = document.createElement('option'); casePlaceholder.value = ''; casePlaceholder.textContent = '-- 选择内置算例 --'; caseSelect.append(casePlaceholder);
      this.state.model.cases.forEach(row => { const option = document.createElement('option'); option.value = row.id; option.textContent = row.label || row.id; option.selected = row.id === this.state.model.selectedCase; caseSelect.append(option); });
      caseSelect.addEventListener('change', () => { this.state.model.selectedCase = caseSelect.value; });
      const caseButton = document.createElement('button'); caseButton.type = 'button'; caseButton.className = 'resilience-portal__button resilience-portal__button--primary'; caseButton.textContent = this.state.model.status === 'loading' ? '加载中…' : '加载内置算例'; caseButton.disabled = this.state.model.status === 'loading' || !this.state.model.cases.length;
      caseButton.addEventListener('click', () => this.loadBuiltinCase(caseSelect.value || this.state.model.selectedCase));
      caseLabel.append(caseSelect, caseButton); grid.append(caseLabel);

      const matLabel = document.createElement('label'); matLabel.className = 'resilience-portal__field'; matLabel.textContent = 'MATPOWER 算例';
      const matSelect = document.createElement('select'); matSelect.dataset.portalField = 'matpower-case';
      const matPlaceholder = document.createElement('option'); matPlaceholder.value = ''; matPlaceholder.textContent = '-- 选择 MATPOWER 文件 --'; matSelect.append(matPlaceholder);
      this.state.model.matpowerFiles.forEach(filename => { const option = document.createElement('option'); option.value = filename; option.textContent = filename; option.selected = filename === this.state.model.selectedMatpower; matSelect.append(option); });
      matSelect.addEventListener('change', () => { this.state.model.selectedMatpower = matSelect.value; });
      const matButton = document.createElement('button'); matButton.type = 'button'; matButton.className = 'resilience-portal__button'; matButton.textContent = '加载 MATPOWER'; matButton.disabled = this.state.model.status === 'loading' || !this.state.model.matpowerFiles.length;
      matButton.addEventListener('click', () => this.loadMatpowerCase(matSelect.value || this.state.model.selectedMatpower));
      matLabel.append(matSelect, matButton); grid.append(matLabel);

      const jsonLabel = document.createElement('label'); jsonLabel.className = 'resilience-portal__field'; jsonLabel.textContent = 'JSON 系统文件';
      const fileInput = document.createElement('input'); fileInput.type = 'file'; fileInput.accept = '.json,application/json'; fileInput.dataset.portalField = 'json-import';
      const jsonHelp = document.createElement('span'); jsonHelp.className = 'resilience-portal__field-help'; jsonHelp.textContent = '导入 JSON 后会同步后端会话并更新隐藏兼容 Canvas；不会调用 advanced I/O。';
      fileInput.addEventListener('change', () => { const file = fileInput.files?.[0]; if (file) this.importJsonFile(file); });
      jsonLabel.append(fileInput, jsonHelp); grid.append(jsonLabel);
      body.append(grid);

      const modelState = document.createElement('div'); modelState.className = 'resilience-portal__model-state';
      modelState.textContent = this.state.model.currentName
        ? `当前模型：${this.state.model.currentName} · ${this.state.model.source} · model_revision ${this.state.currentModelRevision}${this.state.model.counts ? ` · ${jsonText(this.state.model.counts).replace(/\s+/g, ' ')}` : ''}`
        : '当前尚未在本次会话中加载模型。场景生成前必须先加载或导入。';
      body.append(modelState);
      if (this.state.model.error) { const error = document.createElement('div'); error.className = 'resilience-portal__callout resilience-portal__callout--danger'; error.textContent = this.state.model.error; body.append(error); }
      card.append(body); this.panelEl.append(card);
    }

    updateScenarioConfig(field, value) {
      this.state.scenarioConfig = { ...this.state.scenarioConfig, [field]: value };
      this.state.scenarioGeneration.error = null;
    }

    renderScenarioConfig() {
      this.sectionHeading('场景生成参数', '灾害场景 · 固定 48 h');
      const card = document.createElement('section'); card.className = 'resilience-portal__card';
      const body = document.createElement('div'); body.className = 'resilience-portal__card-body';
      const grid = document.createElement('div'); grid.className = 'resilience-portal__form-grid';
      const config = this.state.scenarioConfig;
      const hazardType = config.hazard_type || 'typhoon';
      const hazardSchema = this.adapter.scenario.getHazardSchema?.() || [];
      const hazard = hazardSchema.find(item => item.id === hazardType);
      const hazardField = document.createElement('label'); hazardField.className = 'resilience-portal__field'; hazardField.textContent = '灾害类型';
      const hazardSelect = document.createElement('select'); hazardSelect.dataset.portalField = 'scenario-hazard-type';
      (hazardSchema.length ? hazardSchema : [{ id: 'typhoon', label: '台风' }]).forEach(item => {
        const option = document.createElement('option'); option.value = item.id; option.textContent = item.label; option.selected = item.id === hazardType; hazardSelect.append(option);
      });
      hazardSelect.addEventListener('change', () => { this.updateScenarioConfig('hazard_type', hazardSelect.value); this.render(); });
      hazardField.append(hazardSelect); grid.append(hazardField);

      const clusters = document.createElement('label'); clusters.className = 'resilience-portal__field'; clusters.textContent = hazardType === 'typhoon' ? '每个台风等级聚类数' : '代表场景数';
      const clusterInput = document.createElement('input'); clusterInput.type = 'number'; clusterInput.min = '1'; clusterInput.max = '50'; clusterInput.step = '1'; clusterInput.value = String(config.resilience_cluster_count ?? 5); clusterInput.dataset.portalField = 'scenario-cluster-count';
      const clusterHelp = document.createElement('span'); clusterHelp.className = 'resilience-portal__field-help'; clusterHelp.textContent = '每个已选等级先生成“聚类数 × 10”个候选，再约减为该聚类数。';
      clusterInput.addEventListener('change', () => { const value = Math.max(1, Math.min(50, Math.round(Number(clusterInput.value) || 5))); clusterInput.value = String(value); this.updateScenarioConfig('resilience_cluster_count', value); this.render(); });
      clusters.append(clusterInput, clusterHelp); grid.append(clusters);

      const method = document.createElement('label'); method.className = 'resilience-portal__field'; method.textContent = '场景约减方法';
      const methodSelect = document.createElement('select'); methodSelect.dataset.portalField = 'scenario-method';
      [['hybrid_kmedoids_tail_5pct', '混合密度感知 k-medoids + 尾部锚点'], ['weighted_k_medoids', '旧 weighted k-medoids']].forEach(([value, label]) => { const option = document.createElement('option'); option.value = value; option.textContent = label; option.selected = config.clustering_method === value; methodSelect.append(option); });
      methodSelect.addEventListener('change', () => this.updateScenarioConfig('clustering_method', methodSelect.value));
      method.append(methodSelect); grid.append(method);

      const baseline = document.createElement('label'); baseline.className = 'resilience-portal__check-field';
      const baselineInput = document.createElement('input'); baselineInput.type = 'checkbox'; baselineInput.checked = config.compare_baseline !== false; baselineInput.dataset.portalField = 'scenario-compare-baseline';
      baselineInput.addEventListener('change', () => this.updateScenarioConfig('compare_baseline', baselineInput.checked));
      const baselineCopy = document.createElement('span'); baselineCopy.textContent = '返回与旧 weighted k-medoids 的覆盖率/距离对比'; baseline.append(baselineInput, baselineCopy); grid.append(baseline);
      body.append(grid);
      if (hazardType !== 'typhoon' && hazard) {
        const note = document.createElement('p'); note.className = 'resilience-portal__callout'; note.textContent = `${hazard.description} 默认值用于研究演示，需要按当地数据校准。`; body.append(note);
        const basic = document.createElement('div'); basic.className = 'resilience-portal__form-grid';
        const advanced = document.createElement('details'); const summary = document.createElement('summary'); summary.textContent = '高级灾害参数'; advanced.append(summary);
        const advancedGrid = document.createElement('div'); advancedGrid.className = 'resilience-portal__form-grid'; advanced.append(advancedGrid);
        (hazard.fields || []).forEach(field => {
          const label = document.createElement('label'); label.className = 'resilience-portal__field'; label.textContent = `${field.label} (${field.unit})`;
          const input = document.createElement('input'); input.type = 'number'; input.required = true; input.step = 'any'; input.min = String(field.min); input.max = String(field.max);
          input.value = String(config[hazardType]?.[field.key] ?? field.default); input.dataset.portalHazardParameter = field.key;
          input.addEventListener('change', () => {
            if (!input.value.trim() || !input.checkValidity() || !Number.isFinite(Number(input.value))) { input.reportValidity(); return; }
            this.updateScenarioConfig(hazardType, { ...(this.state.scenarioConfig[hazardType] || {}), [field.key]: Number(input.value) });
          });
          label.append(input); (field.advanced ? advancedGrid : basic).append(label);
        });
        body.append(basic, advanced);
        const reset = document.createElement('button'); reset.type = 'button'; reset.className = 'resilience-portal__button'; reset.textContent = '恢复本灾种默认参数';
        reset.addEventListener('click', () => { this.updateScenarioConfig(hazardType, {}); this.render(); }); body.append(reset);
      }

      const levels = document.createElement('fieldset'); levels.className = 'resilience-portal__fieldset'; const legend = document.createElement('legend'); legend.textContent = '台风强度等级（GB/T 19201-2006）'; levels.append(legend);
      const levelGrid = document.createElement('div'); levelGrid.className = 'resilience-portal__choice-grid';
      const selectedLevels = new Set(config.intensity_levels || []);
      [['TD', '热带低压'], ['TS', '热带风暴'], ['STS', '强热带风暴'], ['TY', '台风'], ['STY', '强台风'], ['SuperTY', '超强台风']].forEach(([value, label]) => {
        const row = document.createElement('label'); row.className = 'resilience-portal__check-field'; const input = document.createElement('input'); input.type = 'checkbox'; input.value = value; input.checked = selectedLevels.has(value); input.dataset.portalIntensity = value;
        input.addEventListener('change', () => { const next = new Set(this.state.scenarioConfig.intensity_levels || []); input.checked ? next.add(value) : next.delete(value); if (!next.size) { input.checked = true; return; } this.updateScenarioConfig('intensity_levels', [...next]); this.render(); });
        const copy = document.createElement('span'); copy.textContent = `${value} · ${label}`; row.append(input, copy); levelGrid.append(row);
      });
      levels.append(levelGrid); if (hazardType === 'typhoon') body.append(levels);
      const derived = document.createElement('div'); derived.className = 'resilience-portal__callout'; const count = Number(config.resilience_cluster_count || 5); const levelCount = hazardType === 'typhoon' ? (config.intensity_levels || []).length : 1; derived.textContent = `派生规模：${levelCount} 组 × 每组 ${Math.min(200, count * 10)} 个候选 → 每组 ${count} 个代表簇；时域固定 48 h。概率表示所设灾害条件下的样本权重。`; body.append(derived);
      card.append(body); this.panelEl.append(card);
    }

    renderScenarioSelection() {
      this.lead('加载电网模型并生成聚类场景集。全部代表簇将共同参与下一步资源规划；具体场景在快速恢复时选择。');
      this.renderModelControls();
      this.renderScenarioConfig();
      const prerequisite = document.createElement('div'); prerequisite.className = 'resilience-portal__callout';
      prerequisite.textContent = this.state.selectedMetricIds.length ? `当前指标选择：${this.state.selectedMetricIds.length} 项（selection_revision ${this.state.selectionRevision}）` : '请先在“指标选择”中至少选择一项指标。';
      this.panelEl.append(prerequisite);
      const actions = document.createElement('div'); actions.className = 'resilience-portal__actions';
      const button = document.createElement('button'); button.type = 'button'; button.className = 'resilience-portal__button resilience-portal__button--primary'; button.textContent = '生成弹性候选场景'; button.disabled = !this.state.selectedMetricIds.length || !this.state.model.currentName || this.state.scenarioGeneration.status === 'running';
      button.addEventListener('click', () => this.generateScenarios(button));
      actions.append(button); this.panelEl.append(actions);
      if (this.state.scenarioGeneration.error) { const error = document.createElement('div'); error.className = 'resilience-portal__callout resilience-portal__callout--danger'; error.textContent = this.state.scenarioGeneration.error; this.panelEl.append(error); }
      this.sectionHeading('规划使用的代表场景集', this.state.scenarioGeneration.candidates.length ? `${this.state.scenarioGeneration.candidates.length} 个代表簇` : '尚未生成');
      if (!this.state.scenarioGeneration.candidates.length) {
        const empty = document.createElement('div'); empty.className = 'resilience-portal__empty'; empty.textContent = '生成结果将在此显示。生成后即可进入主动防御规划。'; this.panelEl.append(empty);
      } else {
        const list = document.createElement('div'); list.className = 'resilience-portal__table-wrap';
        const table = document.createElement('table'); table.className = 'resilience-portal__table';
        const head = document.createElement('tr'); ['代表场景', '灾害等级/类型', '簇内样本权重', '成员数'].forEach(label => { const th = document.createElement('th'); th.textContent = label; head.append(th); }); table.append(head);
        this.state.scenarioGeneration.candidates.forEach(candidate => {
          const row = document.createElement('tr');
          [candidate.id, candidate.intensity || '—', candidate.probability == null ? '—' : Number(candidate.probability).toFixed(3), candidate.member_count ?? '—'].forEach(value => { const td = document.createElement('td'); td.textContent = String(value); row.append(td); });
          table.append(row);
        });
        list.append(table); this.panelEl.append(list);
      }
      if (this.state.scenarioGeneration.data) {
        this.sectionHeading('场景生成结果与证据', '后端返回数据 · 不重新生成科学结果');
        const results = document.createElement('section'); results.className = 'resilience-portal__result-section'; results.dataset.portalResults = 'scenario-generation'; this.panelEl.append(results);
        this.adapter.presentation?.renderScenarioGeneration?.(results, this.state.scenarioGeneration.data, {
          selectedScenarioRef: this.state.scenarioGeneration.selectedScenario?.scenario_ref || null,
        });
      }
    }

    async generateScenarios(button) {
      const invalid = [...this.panelEl.querySelectorAll('[data-portal-hazard-parameter]')].find(input => !input.checkValidity());
      if (invalid) { const details = invalid.closest('details'); if (details) details.open = true; invalid.reportValidity(); return; }
      this.state.scenarioGeneration.status = 'running';
      this.state.scenarioGeneration.error = null;
      this.render();
      this.setStatus('正在生成弹性候选场景...', 'busy');
      try {
        const data = await this.adapter.scenario.generate(clone(this.state.scenarioConfig));
        const candidates = this.adapter.scenario.getCandidates?.(data) || [];
        this.state.scenarioGeneration.data = clone(data);
        this.state.scenarioGeneration.candidates = clone(candidates);
        this.state.scenarioGeneration.selectedScenario = null;
        this.state.proactiveDefense = { ...this.state.proactiveDefense, status: 'idle', result: null, error: null };
        this.state.scenarioDerived = { fields: {}, structuredFaults: [], profiles: null, identity: null, provenance: {} };
        this.state.scenarioOverrides = {};
        this.state.scenarioGeneration.status = candidates.length ? 'ready' : 'empty';
        this.state.scenarioGeneration.scenarioRevision += 1;
        this.state.recovery = { status: 'idle', requestId: null, runId: null, runRevision: null, modelRevision: null, scenarioRevision: null, artifact: null, error: null };
        this.state.metrics = { status: 'idle', requestId: null, runId: null, selectionRevision: null, result: null, error: null };
        this.render();
        this.setStatus(candidates.length ? `已生成 ${candidates.length} 个代表簇，可进入主动防御规划` : '场景生成完成，但没有可用于规划的代表簇', candidates.length ? 'success' : '');
      } catch (error) {
        this.state.scenarioGeneration.status = 'error';
        this.state.scenarioGeneration.error = error.message || String(error);
        this.render();
        this.setStatus(`场景生成失败：${this.state.scenarioGeneration.error}`, 'error');
      } finally { if (button) button.disabled = false; }
    }

    async selectScenario(candidate) {
      if (this.state.proactiveDefense.status !== 'success') return;
      this.setStatus('正在载入所选弹性场景...', 'busy');
      try {
        const selected = await this.adapter.scenario.select(candidate);
        this.state.scenarioGeneration.selectedScenario = clone(selected?.scenario || selected || candidate);
        this.state.scenarioGeneration.scenarioRevision = Number(
          selected?.identity?.scenario_revision ?? selected?.scenario_revision ?? this.state.scenarioGeneration.scenarioRevision + 1
        );
        this.state.scenarioDerived = clone(selected?.scenarioDerived || {
          fields: selected?.recoveryFields || {}, structuredFaults: selected?.structuredFaults || [],
          profiles: selected?.profiles || null, identity: selected?.identity || null,
          provenance: selected?.provenance || {},
        });
        this.state.scenarioOverrides = {};
        this.state.recovery = { status: 'idle', requestId: null, runId: null, runRevision: null, modelRevision: null, scenarioRevision: null, artifact: null, error: null };
        this.state.metrics = { status: 'idle', requestId: null, runId: null, selectionRevision: null, result: null, error: null };
        this.render(); this.setStatus(`已选择场景：${this.state.scenarioGeneration.selectedScenario.id}`, 'success');
      } catch (error) { this.setStatus(`场景载入失败：${error.message || error}`, 'error'); }
    }

    renderProactiveDefense() {
      const candidates = this.state.scenarioGeneration.candidates;
      const planning = this.state.proactiveDefense;
      this.lead('对全部代表场景采用同一套灾前资源配置，再分别计算其恢复损失。规划完成后，在快速恢复中挑选一个场景查看详细过程。');
      const prerequisite = document.createElement('div'); prerequisite.className = 'resilience-portal__callout';
      prerequisite.textContent = candidates.length
        ? `${candidates.length} 个代表簇参与整体规划。各灾害等级组等权，组内采用聚类样本权重；这些权重不代表真实灾害发生概率。`
        : '请先生成聚类场景集。';
      this.panelEl.append(prerequisite);
      const card = document.createElement('section'); card.className = 'resilience-portal__card';
      const body = document.createElement('div'); body.className = 'resilience-portal__card-body';
      const title = document.createElement('h3'); title.className = 'resilience-portal__card-title'; title.textContent = '规划资源'; body.append(title);
      [['addGenerator', '配置一台备用电源'], ['addMobileStorage', '配置一台移动储能并预部署']].forEach(([key, label]) => {
        const row = document.createElement('label'); row.className = 'resilience-portal__check-field';
        const input = document.createElement('input'); input.type = 'checkbox'; input.checked = planning[key] === true; input.dataset.portalField = `planning-${key}`;
        input.addEventListener('change', () => {
          planning[key] = input.checked;
          if (planning.result) { planning.status = 'stale'; planning.result = null; this.state.recovery.status = 'stale'; this.state.metrics.status = 'stale'; }
          this.render();
        });
        const copy = document.createElement('span'); copy.textContent = label; row.append(input, copy); body.append(row);
      });
      const assumptions = document.createElement('p'); assumptions.className = 'resilience-portal__hint'; assumptions.textContent = '首版按算例总负荷派生演示容量；选址基于全部代表场景的负荷与故障暴露。规划评分按启用移动储能调度计算；快速恢复可用开关单独关闭。投资价格缺少数据时保持未知。固定储能扩容、V2G 和燃料供应规划尚未进入此求解范围。'; body.append(assumptions);
      card.append(body); this.panelEl.append(card);
      const actions = document.createElement('div'); actions.className = 'resilience-portal__actions';
      const run = document.createElement('button'); run.type = 'button'; run.className = 'resilience-portal__button resilience-portal__button--primary'; run.dataset.portalPlanningRun = '';
      run.textContent = planning.status === 'running' ? '整体规划中…' : '计算整体规划';
      run.disabled = !candidates.length || !this.state.scenarioGeneration.data?.scenario_generation_id || (!planning.addGenerator && !planning.addMobileStorage) || planning.status === 'running';
      run.addEventListener('click', () => this.runProactivePlanning(run)); actions.append(run); this.panelEl.append(actions);
      if (planning.error) { const error = document.createElement('div'); error.className = 'resilience-portal__callout resilience-portal__callout--danger'; error.textContent = planning.error; this.panelEl.append(error); }
      if (!planning.result) return;
      const result = planning.result;
      this.sectionHeading('跨场景规划结果', `${result.cluster_count} 个代表簇 · ${result.plan_id}`);
      const summary = document.createElement('article'); summary.className = 'resilience-portal__card';
      const summaryBody = document.createElement('div'); summaryBody.className = 'resilience-portal__card-body';
      const summaryText = document.createElement('p'); summaryText.textContent = `设计权重下失供电量：${Number(result.baseline_design_weighted_shed_mwh).toFixed(3)} → ${Number(result.planned_design_weighted_shed_mwh).toFixed(3)} MWh；最差代表场景：${Number(result.baseline_worst_shed_mwh).toFixed(3)} → ${Number(result.planned_worst_shed_mwh).toFixed(3)} MWh。`; summaryBody.append(summaryText);
      const planText = document.createElement('p'); planText.textContent = `备用电源：${result.plan.ac_generator_mw ? `AC 节点 ${result.plan.ac_generator_bus}，${Number(result.plan.ac_generator_mw).toFixed(3)} MW` : '未配置'}；移动储能：${result.plan.mobile_storage_mw ? `AC 节点 ${result.plan.mobile_storage_bus}，${Number(result.plan.mobile_storage_mw).toFixed(3)} MW / ${Number(result.plan.mobile_storage_mwh).toFixed(3)} MWh` : '未配置'}；投资成本：${result.investment_cost == null ? '缺少可靠价格数据' : result.investment_cost}。`; summaryBody.append(planText);
      summary.append(summaryBody); this.panelEl.append(summary);
      this.sectionHeading('逐场景效果', '同一资源配置 · 原故障与负荷曲线');
      const wrap = document.createElement('div'); wrap.className = 'resilience-portal__table-wrap'; const table = document.createElement('table'); table.className = 'resilience-portal__table';
      const header = document.createElement('tr'); ['代表场景', '设计权重', '原方案失供 MWh', '规划后失供 MWh', '改善 MWh'].forEach(label => { const th = document.createElement('th'); th.textContent = label; header.append(th); }); table.append(header);
      (result.scenarios || []).forEach(item => { const row = document.createElement('tr'); [item.scenario_id, Number(item.design_weight).toFixed(4), Number(item.baseline_shed_mwh).toFixed(3), Number(item.planned_shed_mwh).toFixed(3), (Number(item.baseline_shed_mwh) - Number(item.planned_shed_mwh)).toFixed(3)].forEach(value => { const td = document.createElement('td'); td.textContent = value; row.append(td); }); table.append(row); }); wrap.append(table); this.panelEl.append(wrap);
      const details = document.createElement('details'); const detailsTitle = document.createElement('summary'); detailsTitle.textContent = '模型范围与规划假设'; const list = document.createElement('ul'); (result.limitations || []).forEach(item => { const li = document.createElement('li'); li.textContent = item; list.append(li); }); details.append(detailsTitle, list); this.panelEl.append(details);
    }

    async runProactivePlanning(button) {
      const planning = this.state.proactiveDefense;
      const generationId = this.state.scenarioGeneration.data?.scenario_generation_id;
      if (!generationId || (!planning.addGenerator && !planning.addMobileStorage)) return;
      planning.status = 'running'; planning.error = null; planning.result = null;
      this.render(); this.setStatus(`正在评估 ${this.state.scenarioGeneration.candidates.length} 个代表簇，较多场景可能需要几十秒…`, 'busy');
      try {
        const result = await this.adapter.planning.run({ scenario_generation_id: generationId, add_generator: planning.addGenerator, add_mobile_storage: planning.addMobileStorage });
        if (generationId !== this.state.scenarioGeneration.data?.scenario_generation_id) return;
        if (!result || result.status !== 'computed' || !result.plan_id || result.scenario_generation_id !== generationId || !Array.isArray(result.scenarios)) throw new Error('规划响应缺少有效方案或场景集身份');
        const expected = this.state.scenarioGeneration.candidates.map(item => item.id).sort();
        const actual = result.scenarios.map(item => item.scenario_id).sort();
        if (expected.length !== actual.length || expected.some((id, index) => id !== actual[index])) throw new Error('规划响应未覆盖完整代表场景集');
        planning.result = clone(result); planning.status = 'success'; planning.error = null;
        this.state.recovery = { status: 'idle', requestId: null, runId: null, runRevision: null, modelRevision: null, scenarioRevision: null, artifact: null, error: null };
        this.state.metrics = { status: 'idle', requestId: null, runId: null, selectionRevision: null, result: null, error: null };
        this.render(); this.setStatus(`整体规划完成：${result.cluster_count} 个代表簇 · ${result.plan_id}`, 'success');
      } catch (error) {
        planning.status = 'error'; planning.error = error.message || String(error);
        this.render(); this.setStatus(`整体规划失败：${planning.error}`, 'error');
      } finally { if (button) button.disabled = false; }
    }

    renderRecoveryConfig() {
      const card = document.createElement('section'); card.className = 'resilience-portal__card';
      const body = document.createElement('div'); body.className = 'resilience-portal__card-body';
      const heading = document.createElement('h3'); heading.className = 'resilience-portal__card-title';
      heading.textContent = '快速恢复设置'; body.append(heading);
      const row = document.createElement('label'); row.className = 'resilience-portal__check-field';
      const input = document.createElement('input'); input.type = 'checkbox';
      input.dataset.portalField = 'recovery-allow_mess_dispatch';
      input.checked = this.effectiveRecoveryConfig().allow_mess_dispatch === true;
      input.addEventListener('change', () => this.updateRecoveryField('allow_mess_dispatch', input.checked));
      const label = document.createElement('span'); label.textContent = '考虑移动储能调度';
      row.append(input, label); body.append(row);
      const note = document.createElement('p'); note.className = 'resilience-portal__hint';
      note.textContent = '故障、修复时间和负荷曲线来自已选代表场景；灾前资源配置来自整体规划，其余恢复参数使用平台默认值。';
      body.append(note);
      card.append(body); this.panelEl.append(card);
    }
    renderRapidRecovery() {
      const selected = this.state.scenarioGeneration.selectedScenario;
      const plan = this.state.proactiveDefense.result;
      this.lead('从参与整体规划的代表场景中选择一个，查看同一资源方案下的详细恢复过程。');
      this.sectionHeading('选择代表场景', plan ? `整体规划 ${plan.plan_id}` : '请先完成整体规划');
      const chooser = document.createElement('div'); chooser.className = 'resilience-portal__scenario-list';
      this.state.scenarioGeneration.candidates.forEach(candidate => {
        const button = document.createElement('button'); button.type = 'button'; button.className = 'resilience-portal__button';
        button.dataset.portalScenarioSelect = candidate.id;
        button.textContent = `${candidate.id} · ${candidate.intensity || '场景'} · 簇权重 ${candidate.probability == null ? '—' : Number(candidate.probability).toFixed(3)}`;
        button.disabled = !plan || this.state.proactiveDefense.status !== 'success';
        if (selected?.id === candidate.id) button.setAttribute('aria-pressed', 'true');
        button.addEventListener('click', () => this.selectScenario(candidate)); chooser.append(button);
      });
      this.panelEl.append(chooser);
      this.renderRecoveryConfig();
      const prereq = document.createElement('div'); prereq.className = 'resilience-portal__callout'; prereq.textContent = !plan ? '前置条件：先完成全部代表场景的整体规划。' : !this.state.selectedMetricIds.length ? '前置条件：至少选择一项指标。' : !selected ? '请选择一个代表场景。' : `当前场景：${selected.id} · scenario_revision ${this.state.scenarioGeneration.scenarioRevision}`; this.panelEl.append(prereq);
      const actions = document.createElement('div'); actions.className = 'resilience-portal__actions';
      const run = document.createElement('button'); run.type = 'button'; run.className = 'resilience-portal__button resilience-portal__button--primary'; run.textContent = this.state.recovery.status === 'running' ? '恢复运行中…' : '运行快速恢复'; run.disabled = !plan || this.state.proactiveDefense.status !== 'success' || !this.state.selectedMetricIds.length || !selected || this.state.recovery.status === 'running' || this.state.scenarioDerived?.profiles?.valid === false; run.addEventListener('click', () => this.runRecovery(run)); actions.append(run); this.panelEl.append(actions);
      if (this.state.recovery.error) { const error = document.createElement('div'); error.className = 'resilience-portal__callout resilience-portal__callout--danger'; error.textContent = this.state.recovery.error; this.panelEl.append(error); }
      if (!this.state.recovery.artifact) { const empty = document.createElement('div'); empty.className = 'resilience-portal__empty'; empty.textContent = '尚未运行恢复。运行完成后将在此显示 run identity、time axis、steps 和科学限制。'; this.panelEl.append(empty); return; }
      this.renderRunArtifact(this.state.recovery.artifact);
    }

    async runRecovery(button) {
      const scenario = this.state.scenarioGeneration.selectedScenario;
      const plan = this.state.proactiveDefense.result;
      if (!scenario || !plan || this.state.proactiveDefense.status !== 'success' || !this.state.selectedMetricIds.length) return;
      if (this.state.scenarioDerived?.profiles?.valid === false) {
        this.state.recovery.status = 'error';
        this.state.recovery.error = `所选场景 profile 无效：${this.state.scenarioDerived.profiles.warnings?.join('；') || '长度、步长或样本不符合契约'}`;
        this.render();
        this.setStatus(this.state.recovery.error, 'error');
        return;
      }
      const requestRevision = ++this.state.portalRequestRevision;
      this.state.recovery.status = 'running'; this.state.recovery.error = null; this.render(); this.setStatus('快速恢复运行中...', 'busy');
      try {
        const data = await this.adapter.recovery.run({
          scenario,
          portfolioPlanId: plan.plan_id,
          params: clone(this.effectiveRecoveryConfig()),
          scenarioContext: clone(this.state.scenarioDerived),
          scenarioRevision: this.state.scenarioGeneration.scenarioRevision,
          selectionRevision: this.state.selectionRevision,
        });
        if (requestRevision !== this.state.portalRequestRevision) return;
        if (!data || data.error) throw new Error(data?.error?.message || data?.error || '恢复结果为空');
        if (data.portfolio_plan_id !== plan.plan_id) throw new Error('恢复响应未使用当前整体规划方案');
        if (data.scenario_revision !== undefined &&
            Number(data.scenario_revision) !== this.state.scenarioGeneration.scenarioRevision) {
          const legacyResponseOmittedRequestRevision = Number(data.scenario_revision) === 0 &&
            (data.scenario_ref === undefined || data.scenario_ref === null) &&
            (data.scenario_digest === undefined || data.scenario_digest === null);
          if (!legacyResponseOmittedRequestRevision) {
            throw new Error('恢复响应 scenario_revision 与当前选择不一致，已拒绝显示');
          }
          data.scenario_revision = this.state.scenarioGeneration.scenarioRevision;
          data.scenario_ref = scenario.scenario_ref ?? scenario.id;
          data.scenario_digest = scenario.scenario_digest ?? scenario.digest ?? null;
          data.portal_provenance_compatibility = 'running_server_did_not_echo_scenario_identity';
        }
        this.state.recovery = { status: 'success', requestId: data.request_id || null, runId: data.run_id || null, runRevision: data.run_revision ?? null, modelRevision: data.model_revision ?? this.state.currentModelRevision, scenarioRevision: data.scenario_revision ?? this.state.scenarioGeneration.scenarioRevision, artifact: clone(data), error: null };
        this.state.metrics = { status: 'idle', requestId: null, runId: null, selectionRevision: null, result: null, error: null };
        this.render(); this.setStatus(`恢复完成：${text(data.run_id)} · legacy_full_distribution_resilience`, 'success');
      } catch (error) {
        this.state.recovery.status = 'error';
        this.state.recovery.error = error.message || String(error);
        this.render();
        this.setStatus(`快速恢复失败：${this.state.recovery.error}`, 'error');
      }
      finally { if (button) button.disabled = false; }
    }

    renderRunArtifact(data) {
      const card = document.createElement('article'); card.className = 'resilience-portal__card'; const body = document.createElement('div'); body.className = 'resilience-portal__card-body';
      const h = document.createElement('h3'); h.className = 'resilience-portal__card-title'; h.textContent = '运行证据'; body.append(h);
      const tableWrap = document.createElement('div'); tableWrap.className = 'resilience-portal__table-wrap'; const table = document.createElement('table'); table.className = 'resilience-portal__table';
      const rows = [['run_id', data.run_id], ['portfolio_plan_id', data.portfolio_plan_id], ['run_revision', data.run_revision], ['model_revision', data.model_revision], ['scenario_revision', data.scenario_revision], ['scenario_digest', data.scenario_digest], ['execution_mode', data.execution_mode || 'legacy_full_distribution_resilience'], ['outcome', data.result_outcome || data.outcome], ['scientific_usability', data.scientific_usability || data.usability], ['stale', data.stale === true ? 'true' : 'false']];
      rows.forEach(([key, value]) => { const tr = document.createElement('tr'); const th = document.createElement('th'); th.textContent = key; const td = document.createElement('td'); td.textContent = text(value); tr.append(th, td); table.append(tr); });
      tableWrap.append(table); body.append(tableWrap);
      const note = document.createElement('div'); note.className = 'resilience-portal__callout resilience-portal__callout--warning'; note.textContent = '普通 restoration feasibility 不是 certified dynamic safety；run artifact 仅在服务进程内有界保存，重启或 eviction 后可能失效。'; body.append(note);
      if (Array.isArray(data.steps) && data.steps.length) {
        const details = document.createElement('details'); const summary = document.createElement('summary'); summary.textContent = `time_axis / steps evidence（${data.steps.length} 个时点）`; const pre = document.createElement('pre'); pre.className = 'resilience-portal__json'; pre.textContent = jsonText({ time_axis: data.time_axis, steps: data.steps }); details.append(summary, pre); body.append(details);
      }
      card.append(body);
      const results = document.createElement('section'); results.className = 'resilience-portal__result-section'; results.dataset.portalResults = 'rapid-recovery'; this.panelEl.append(results);
      this.adapter.presentation?.renderRapidRecovery?.(results, data, {
        scenarioContext: clone(this.state.scenarioDerived),
      });
      const footer = document.createElement('details'); footer.className = 'resilience-portal__calculation-summary'; footer.dataset.portalCalculationSummary = 'rapid-recovery';
      const summary = document.createElement('summary'); summary.textContent = '计算摘要与运行依据'; footer.append(summary, card);
      const evidence = results.querySelector('[data-portal-run-evidence]');
      if (evidence) footer.append(evidence);
      this.panelEl.append(footer);
    }

    renderMetricOutput() {
      this.lead('按本次恢复记录输出所选指标。运行指标覆盖完整计算时段；书中灾后指标使用声明的恢复窗口。每项保留单位、公式、数据来源与有效边界，可导出结果。');
      const artifact = this.state.recovery.artifact; const stale = this.state.recovery.stale || this.state.metrics.result?.stale;
      const prereq = document.createElement('div'); prereq.className = `resilience-portal__callout${stale ? ' resilience-portal__callout--warning' : ''}`;
      prereq.textContent = !this.state.selectedMetricIds.length ? '请先选择指标。' : !artifact ? '请先完成一次快速恢复。' : stale ? '当前 run 已过期，不能作为当前指标输入，请重新运行恢复。' : `run_id ${artifact.run_id} · selection_revision ${this.state.selectionRevision}`; this.panelEl.append(prereq);
      const actions = document.createElement('div'); actions.className = 'resilience-portal__actions'; const evaluate = document.createElement('button'); evaluate.type = 'button'; evaluate.className = 'resilience-portal__button resilience-portal__button--primary'; evaluate.textContent = this.state.metrics.status === 'running' ? '指标计算中…' : '计算并输出指标'; evaluate.disabled = !artifact || stale || !this.state.selectedMetricIds.length || this.state.metrics.status === 'running'; evaluate.addEventListener('click', () => this.evaluateMetrics(evaluate)); actions.append(evaluate); this.panelEl.append(actions);
      if (this.state.metrics.error) { const error = document.createElement('div'); error.className = 'resilience-portal__callout resilience-portal__callout--danger'; error.textContent = this.state.metrics.error; this.panelEl.append(error); }
      const results = this.state.metrics.result?.results;
      if (!Array.isArray(results)) { const empty = document.createElement('div'); empty.className = 'resilience-portal__empty'; empty.textContent = '后端指标结果将在此显示；不可用项保留 null 和原因。'; this.panelEl.append(empty); return; }
      const visible = results.filter(row => this.state.selectedMetricIds.includes(row.id)).sort((a, b) => Number(Number.isFinite(b.value)) - Number(Number.isFinite(a.value)));
      const highlights = document.createElement('div'); highlights.className = 'rp-kpis';
      ['run.ens', 'run.energy_supply_ratio', 'run.peak_shed', 'ch3.t_sp'].forEach(id => {
        const row = visible.find(item => item.id === id); if (!row) return;
        const card = document.createElement('article'); const label = document.createElement('span');
        label.textContent = this.state.metricCatalog.find(item => item.id === id)?.name_zh || id;
        const value = document.createElement('strong'); value.textContent = Number.isFinite(row.value) ? `${Number((row.unit === '1' ? row.value * 100 : row.value).toPrecision(5))} ${row.unit === '1' ? '%' : row.unit}` : '暂无数值';
        const status = document.createElement('small'); status.textContent = `${STATUS_LABELS[row.status] || row.status}${row.censored ? ' · 窗口内未恢复完成' : ''}`;
        card.append(label, value, status); highlights.append(card);
      });
      if (highlights.children.length) this.panelEl.append(highlights);
      const computed = visible.filter(row => row.status === 'computed').length;
      const approximate = visible.filter(row => row.status === 'approximate').length;
      this.sectionHeading('指标结果', `${computed} 项已计算 · ${approximate} 项近似 · ${visible.length - computed - approximate} 项无数值`);
      const download = (extension, content, mime) => {
        const url = URL.createObjectURL(new Blob(['\ufeff', content], { type: mime }));
        const link = document.createElement('a'); link.href = url; link.download = `resilience-metrics-${artifact.run_id}.${extension}`; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
      };
      const csvCell = value => '"' + String(value ?? '').replaceAll('"', '""') + '"';
      [['导出 JSON', () => download('json', jsonText(this.state.metrics.result), 'application/json')], ['导出 CSV', () => {
        const rows = [['指标ID', '名称', '状态', '数值', '单位', '公式', '截尾', '原因与限制'], ...visible.map(row => [row.id, this.state.metricCatalog.find(item => item.id === row.id)?.name_zh, row.status, row.value, row.unit, row.formula_ref, row.censored, [row.reason_code, ...(row.limitations || [])].filter(Boolean).join('；')])];
        download('csv', rows.map(row => row.map(csvCell).join(',')).join('\r\n'), 'text/csv;charset=utf-8');
      }]].forEach(([label, handler]) => { const button = document.createElement('button'); button.type = 'button'; button.className = 'resilience-portal__button'; button.textContent = label; button.addEventListener('click', handler); actions.append(button); });
      const filter = document.createElement('label'); filter.className = 'resilience-portal__metric-select'; const onlyValues = document.createElement('input'); onlyValues.type = 'checkbox'; filter.append(onlyValues, document.createTextNode('只看有数值的指标')); actions.append(filter);
      const wrap = document.createElement('div'); wrap.className = 'resilience-portal__table-wrap'; const table = document.createElement('table'); table.className = 'resilience-portal__table';
      const head = document.createElement('tr'); ['指标 / 来源', '状态', '值', '单位', '公式', '原因 / 限制'].forEach(label => { const th = document.createElement('th'); th.textContent = label; head.append(th); }); const thead = document.createElement('thead'); thead.append(head); table.append(thead); const tbody = document.createElement('tbody');
      visible.forEach(row => {
        const tr = document.createElement('tr'); tr.dataset.metricId = row.id; tr.dataset.hasValue = String(Number.isFinite(row.value));
        const definition = this.state.metricCatalog.find(item => item.id === row.id);
        const formatted = Number.isFinite(row.value) ? (row.unit === '1' ? `${(row.value * 100).toFixed(2)}%` : Number(row.value.toPrecision(6)).toString()) : '不可用（null）';
        const cells = [`${definition?.name_zh || row.id} · ${row.id.startsWith('run.') ? '工程扩展' : '书中第三章'}`, row.status, formatted, row.unit === '1' ? '%' : row.unit, row.id.startsWith('run.') ? '运行统计' : String(row.formula_ref || '').replace('book_ch3_eq_', '式 ').replaceAll('_', '.'), [row.censored ? '观察窗口内恢复未完成 / 目标未达到' : '', row.reason_code, ...(row.assumptions || []), ...(row.limitations || []), ...(row.missing_dependencies || [])].filter(Boolean).join('；') || '—'];
        cells.forEach((value, index) => { const td = document.createElement('td'); if (index === 1) { const badge = document.createElement('span'); badge.className = 'resilience-portal__status-badge'; badge.dataset.status = row.status || ''; badge.textContent = STATUS_LABELS[row.status] || row.status || '—'; td.append(badge); } else if (index === 5) { const details = document.createElement('details'); const summary = document.createElement('summary'); summary.textContent = Number.isFinite(row.value) ? (row.censored ? '窗口内未恢复完成' : '计算依据与限制') : '缺少数据 / 原因'; const body = document.createElement('p'); body.textContent = `${row.formula_ref || ''}；${text(value)}`; details.append(summary, body); td.append(details); } else td.textContent = text(value); tr.append(td); }); tbody.append(tr);
      });
      onlyValues.addEventListener('change', () => tbody.querySelectorAll('tr').forEach(row => { row.hidden = onlyValues.checked && row.dataset.hasValue !== 'true'; }));
      table.append(tbody); wrap.append(table); this.panelEl.append(wrap);
      const evidence = document.createElement('details'); evidence.className = 'resilience-portal__calculation-summary'; evidence.dataset.portalCalculationSummary = 'metrics'; const summary = document.createElement('summary'); summary.textContent = '计算摘要、指标来源与限制'; const pre = document.createElement('pre'); pre.className = 'resilience-portal__json'; pre.textContent = jsonText(results); evidence.append(summary, pre); this.panelEl.append(evidence);
    }

    async evaluateMetrics(button) {
      const artifact = this.state.recovery.artifact; if (!artifact) return;
      const requestRevision = ++this.state.portalRequestRevision;
      this.state.metrics.status = 'running'; this.state.metrics.error = null; this.render(); this.setStatus('正在请求后端指标 evaluator...', 'busy');
      try {
        const request = {
          schema: 'resilience_metric_request_v1',
          run_id: artifact.run_id,
          selected_metric_ids: [...this.state.selectedMetricIds],
          selection_revision: this.state.selectionRevision,
          current_model_revision: artifact.model_revision ?? this.state.currentModelRevision,
        };
        const result = await this.adapter.metrics.evaluate(request);
        if (requestRevision !== this.state.portalRequestRevision) return;
      if (!result?.ok) {
          const error = result?.error || result?.data?.error || '指标 evaluator 请求失败';
          throw new Error(typeof error === 'string' ? error : (error.message || JSON.stringify(error)));
        }
        if (result.data?.run_id !== artifact.run_id ||
            Number(result.data?.selection_revision ?? -1) !== this.state.selectionRevision ||
            Number(result.data?.model_revision ?? artifact.model_revision ?? -1) !==
              Number(artifact.model_revision ?? -1)) {
          throw new Error('指标响应 identity 与当前选择或模型 revision 不一致，已拒绝显示');
        }
        this.state.metrics = { status: result.data.stale ? 'stale' : 'success', requestId: result.requestId || null, runId: result.data.run_id, selectionRevision: result.data.selection_revision, result: clone(result.data), error: null };
        this.render(); this.setStatus(result.data.stale ? '指标结果已过期' : '指标输出完成', result.data.stale ? 'stale' : 'success');
      } catch (error) {
        this.state.metrics.status = 'error';
        this.state.metrics.error = error.message || String(error);
        this.render();
        this.setStatus(`指标输出失败：${this.state.metrics.error}`, 'error');
      }
      finally { if (button) button.disabled = false; }
    }

    renderEvidence() {
      if (!this.evidenceEl) return;
      this.evidenceEl.replaceChildren();
      const identity = this.state.scenarioDerived?.identity;
      const profiles = this.state.scenarioDerived?.profiles;
      const items = [
        ['选择 revision', `selection v${this.state.selectionRevision}`],
        ['场景 revision', `scenario v${this.state.scenarioGeneration.scenarioRevision}`],
        ['场景 identity', identity?.scenario_ref || '尚未选择'],
        ['场景 digest', identity?.scenario_digest || '—'],
        ['profile provenance', profiles ? `${profiles.count ?? 0} 项 · ${profiles.num_steps ?? '—'} × ${profiles.step_duration_hr ?? '—'} h · ${profiles.valid === false ? '无效' : '有效'}` : '尚未载入'],
        ['模型 revision', `model v${this.state.currentModelRevision}`],
        ['恢复 run', this.state.recovery.runId || '尚未生成'],
        ['科学边界', '单次确定性运行；普通恢复可行性不等于动态安全认证'],
      ];
      items.forEach(([label, value]) => { const item = document.createElement('div'); item.className = 'resilience-portal__evidence-item'; const l = document.createElement('span'); l.className = 'resilience-portal__evidence-label'; l.textContent = label; const v = document.createElement('span'); v.className = 'resilience-portal__evidence-value'; v.textContent = value; item.append(l, v); this.evidenceEl.append(item); });
    }
  }

  function mount(options = {}) {
    const root = document.getElementById('resiliencePortalRoot');
    if (!root || !options.adapter || options.adapter.getProfile?.()?.edition !== 'resilience') return null;
    if (root.__resiliencePortal) return root.__resiliencePortal;
    const portal = new Portal(root, options.adapter); root.__resiliencePortal = portal; portal.mount(); return portal;
  }

  core.ResiliencePortal = Object.freeze({ mount, steps: STEPS });
})(window);
