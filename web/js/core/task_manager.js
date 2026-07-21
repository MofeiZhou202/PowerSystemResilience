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
      this.cancelBackend = options.cancelBackend || (async () => {});
      this.waitForIdle = options.waitForIdle || (async () => true);
      this.settlementRetryMs = options.settlementRetryMs ?? 500;
      this.settlementGeneration = 0;
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
      if (!context || context.cancelled) return false;
      context.cancelled = true;
      context.controller.abort();
      this.settling = true;
      const generation = ++this.settlementGeneration;
      this.onCancel(context);
      this.onChange(context);
      void this._waitForSettlement(context, generation);
      return true;
    }

    async _waitForSettlement(context, generation) {
      try {
        await this.cancelBackend(context);
      } catch {
        // Backend status polling below remains authoritative.
      }
      await new Promise(resolve => setTimeout(resolve, 150));
      while (this.settling && generation === this.settlementGeneration) {
        let idle = false;
        try {
          idle = await this.waitForIdle();
        } catch {
          idle = false;
        }
        if (idle) {
          this.settling = false;
          this.onChange(this.active);
          return;
        }
        await new Promise(resolve => setTimeout(resolve, this.settlementRetryMs));
      }
    }
  }

  core.AnalysisTaskManager = AnalysisTaskManager;
})(window);
