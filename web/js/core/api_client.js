/** Fetch client for HySim analysis contracts and managed task execution. */
'use strict';

(function initApiClient(global) {
  const core = global.HySimCore = global.HySimCore || {};

  async function parseJsonResponse(response) {
    const text = await response.text();
    if (!text.trim()) return {};
    try {
      return JSON.parse(text);
    } catch (error) {
      return { error: text || response.statusText || error.message };
    }
  }

  class ApiClient {
    constructor(options) {
      this.baseUrl = options.baseUrl;
      this.contracts = options.contracts;
      this.tasks = options.tasks;
      this.getModelRevision = options.getModelRevision;
      this.onRequestStart = options.onRequestStart || (() => {});
      this.onStale = options.onStale || (() => {});
      this.onUnexpectedCancel = options.onUnexpectedCancel || (() => {});
      this.onError = options.onError || (() => {});
      this.log = options.log || (() => {});
      this.recordExecution = options.recordExecution || (() => {});
      this.busyError = options.busyError || 'Another analysis is already running';
      this.fetchImpl = options.fetchImpl || global.fetch.bind(global);
      this.sequence = 0;
    }

    post(path, body = {}, options = {}) {
      return this._request(path, body, options, false);
    }

    postResult(path, body = {}, options = {}) {
      return this._request(path, body, options, true);
    }

    async _request(path, body, options, detailed) {
      const url = `${this.baseUrl}${path}`;
      const started = performance.now();
      const requestId = `hysim-${Date.now().toString(36)}-${++this.sequence}`;
      const modelRevision = this.getModelRevision();
      const analysis = this.contracts.analysisForPath(path);
      const request = this.tasks.start({ path, requestId, modelRevision, analysis });
      if (request.duplicate) {
        this.log(`${this.contracts.labelForAnalysis(analysis)}未提交：已有分析任务正在运行`, 'warn');
        return detailed ? {
          ok: false, data: null, error: this.busyError, stale: false,
          cancelled: false, duplicate: true, requestId, modelRevision,
        } : null;
      }
      this.onRequestStart(request);
      if (!options.quiet) this.log(`POST ${path}`, 'info');
      try {
        const response = await this.fetchImpl(url, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-HySim-Request-ID': requestId },
          body: JSON.stringify(body),
          signal: request.signal,
        });
        const data = await parseJsonResponse(response);
        const stale = this.contracts.attachResultContract({
          path, data, requestId, modelRevision,
          currentModelRevision: this.getModelRevision(),
        });
        this.recordExecution(path, data, (performance.now() - started) / 1000);
        if (!response.ok) {
          const error = (data && (data.error || data.message)) || response.statusText;
          this.onError('http', error, { path, status: response.status });
          if (!options.quiet) this.log(`Error: ${error}`, 'error');
          return detailed
            ? { ok: false, data, error, stale, requestId, modelRevision }
            : null;
        }
        if (stale) {
          this.onStale(request);
          return detailed ? {
            ok: false, data: null, error: '模型已修改，计算结果已失效', stale: true,
            cancelled: false, requestId, modelRevision,
          } : null;
        }
        return detailed
          ? { ok: true, data, error: null, stale, requestId, modelRevision }
          : data;
      } catch (error) {
        if (error.name === 'AbortError') {
          if (!request.cancelled) this.onUnexpectedCancel(request);
          return detailed ? {
            ok: false, data: null, error: '分析请求已取消', stale: false,
            cancelled: true, requestId, modelRevision,
          } : null;
        }
        if (!options.quiet) this.log(`Network error: ${error.message}`, 'error');
        this.onError('network', error, { path });
        return detailed ? {
          ok: false, data: null, error: error.message, stale: false,
          requestId, modelRevision,
        } : null;
      } finally {
        this.tasks.finish(request);
      }
    }
  }

  core.ApiClient = ApiClient;
})(window);
