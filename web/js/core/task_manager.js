/** Single-active-analysis lifecycle with frontend cancellation and settling. */
'use strict';

(function initTaskManager(global) {
  const core = global.HySimCore = global.HySimCore || {};

  class AnalysisTaskManager {
    constructor(options = {}) {
      this.active = null;
      this.settling = false;
      this.onChange = options.onChange || (() => {});
      this.onCancel = options.onCancel || (() => {});
      this.waitForIdle = options.waitForIdle || (async () => true);
    }

    start({ path, requestId, modelRevision, analysis }) {
      if (!analysis) return { managed: false, signal: undefined, cancelled: false };
      if (this.active || this.settling) {
        return { managed: true, duplicate: true, analysis, cancelled: false };
      }
      const controller = new AbortController();
      const context = {
        managed: true,
        duplicate: false,
        analysis,
        path,
        requestId,
        modelRevision,
        controller,
        signal: controller.signal,
        cancelled: false,
      };
      this.active = context;
      this.onChange(context);
      return context;
    }

    finish(context) {
      if (context?.managed && this.active === context) {
        this.active = null;
        this.onChange(null);
      }
    }

    cancel() {
      const context = this.active;
      if (!context) return false;
      context.cancelled = true;
      context.controller.abort();
      this.settling = true;
      this.onCancel(context);
      this.onChange(context);
      void this._waitForSettlement();
      return true;
    }

    async _waitForSettlement() {
      await new Promise(resolve => setTimeout(resolve, 150));
      const idle = await this.waitForIdle();
      this.settling = !idle;
      this.onChange(this.active);
    }
  }

  core.AnalysisTaskManager = AnalysisTaskManager;
})(window);
