/** Bounded, local-only runtime diagnostics for frontend failures and connectivity. */
'use strict';

(function initRuntimeDiagnostics(global) {
  const core = global.HySimCore = global.HySimCore || {};
  const MAX_ERRORS = 25;
  const state = {
    initialized: false,
    online: typeof navigator === 'undefined' ? true : navigator.onLine !== false,
    errors: [],
    errorCount: 0,
    rejectionCount: 0,
    networkErrorCount: 0,
  };
  let chipId = 'runtimeHealthChip';
  let logger = () => {};

  function clean(value, limit = 500) {
    const text = String(value == null ? '' : value).replace(/\s+/g, ' ').trim();
    return text.slice(0, limit) || '未知前端错误';
  }

  function status() {
    if (!state.online) return 'offline';
    return state.errors.length ? 'degraded' : 'healthy';
  }

  function render() {
    const chip = document.getElementById(chipId);
    if (!chip) return;
    const current = status();
    chip.className = `dep-chip ${current === 'healthy' ? 'ok' : (current === 'offline' ? 'warn' : 'error')}`;
    chip.dataset.runtimeStatus = current;
    chip.textContent = current === 'healthy' ? '前端: 正常' :
      (current === 'offline' ? '前端: 离线' : `前端: ${state.errors.length}错误`);
    chip.title = current === 'healthy' ? '浏览器运行状态正常' :
      (current === 'offline' ? '浏览器处于离线状态，分析请求暂不可用' : clean(state.errors.at(-1)?.message, 160));
  }

  function capture(type, value, context = {}) {
    const kind = clean(type || 'error', 40);
    const message = clean(value?.message || value);
    const entry = {
      type: kind,
      message,
      source: clean(context.source || value?.filename || '', 180),
      at: new Date().toISOString(),
    };
    state.errorCount += 1;
    if (kind === 'unhandledrejection') state.rejectionCount += 1;
    if (kind === 'network') state.networkErrorCount += 1;
    state.errors.push(entry);
    if (state.errors.length > MAX_ERRORS) state.errors.splice(0, state.errors.length - MAX_ERRORS);
    render();
    if (!context.quiet) logger(`前端诊断：${message}`, 'error');
    return entry;
  }

  function setOnline(online) {
    state.online = !!online;
    render();
  }

  function snapshot() {
    return {
      schema: 'hysim_runtime_diagnostics_v1',
      status: status(),
      online: state.online,
      retained_errors: state.errors.length,
      total_errors: state.errorCount,
      unhandled_rejections: state.rejectionCount,
      network_errors: state.networkErrorCount,
      max_retained_errors: MAX_ERRORS,
      last_error: state.errors.length ? { ...state.errors.at(-1) } : null,
    };
  }

  function clear() {
    state.errors.length = 0;
    state.errorCount = 0;
    state.rejectionCount = 0;
    state.networkErrorCount = 0;
    render();
    return snapshot();
  }

  function init(options = {}) {
    chipId = options.chipId || chipId;
    logger = typeof options.log === 'function' ? options.log : logger;
    if (!state.initialized) {
      state.initialized = true;
      global.addEventListener('error', event => {
        capture('error', event.error || event.message, { source: event.filename, quiet: true });
      });
      global.addEventListener('unhandledrejection', event => {
        capture('unhandledrejection', event.reason, { quiet: true });
      });
      global.addEventListener('online', () => setOnline(true));
      global.addEventListener('offline', () => setOnline(false));
    }
    setOnline(typeof navigator === 'undefined' ? true : navigator.onLine !== false);
    return snapshot();
  }

  core.RuntimeDiagnostics = Object.freeze({
    schema: 'hysim_runtime_diagnostics_v1',
    init,
    capture,
    setOnline,
    snapshot,
    clear,
  });
})(window);
