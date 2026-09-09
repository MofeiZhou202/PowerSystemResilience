/** Asynchronous ELK client. The bundled ELK runtime owns its official Worker. */
'use strict';

(function initLayoutEngine(global) {
  const core = global.HySimCore = global.HySimCore || {};
  let engine = null;
  let sequence = 0;
  // ELK's dedicated worker script (elkjs 0.9.3, vendored). The main-thread ELK
  // API (from elk.bundled.js) delegates layout to this worker so large-graph
  // layout never blocks the UI thread. Relative to the /xjtu/ document root.
  const WORKER_URL = (global.__ELK_WORKER_URL__ || 'vendor/elk-worker.min.js');
  let _usedWorker = false;
  let _forceMainThread = false;

  function ensureEngine() {
    if (engine) return engine;
    if (typeof global.ELK !== 'function') throw new Error('ELK bundle is unavailable');
    // Prefer a Web Worker so large-graph layout never blocks the UI thread.
    // Fall back to the main-thread engine if workers are unavailable or the
    // engine has already latched to main-thread after a worker failure.
    if (!_forceMainThread && typeof Worker === 'function') {
      try {
        engine = new global.ELK({ workerUrl: WORKER_URL, workerFactory: (url) => new Worker(url) });
        _usedWorker = true;
        return engine;
      } catch (e) {
        engine = null;
      }
    }
    engine = new global.ELK();
    _usedWorker = false;
    return engine;
  }

  function flattenNodes(node, offsetX = 0, offsetY = 0, output = {}) {
    const x = offsetX + (Number(node.x) || 0);
    const y = offsetY + (Number(node.y) || 0);
    if (String(node.id || '').startsWith('component-')) {
      output[node.id] = {
        x: x + (Number(node.width) || 0) / 2,
        y: y + (Number(node.height) || 0) / 2,
        width: Number(node.width) || 0,
        height: Number(node.height) || 0,
      };
    }
    (node.children || []).forEach(child => flattenNodes(child, x, y, output));
    return output;
  }

  async function layout(contract, options = {}) {
    const requestId = `layout-${Date.now().toString(36)}-${++sequence}`;
    const timeoutMs = Number(options.timeoutMs) || 60000;
    const started = performance.now();
    const withTimeout = (p) => {
      let timer;
      const timeout = new Promise((_, reject) => {
        timer = setTimeout(() => reject(new Error(`ELK layout timed out after ${timeoutMs} ms`)), timeoutMs);
      });
      return Promise.race([p, timeout]).finally(() => clearTimeout(timer));
    };
    let result;
    try {
      result = await withTimeout(ensureEngine().layout(contract.graph));
    } catch (err) {
      // If the worker-backed engine failed (construction ok but layout errored /
      // timed out in the worker), drop to a main-thread engine and retry once so
      // layout never hard-fails because of a worker issue.
      if (_usedWorker) {
        try { if (engine && engine.terminateWorker) engine.terminateWorker(); } catch (e) { /* ignore */ }
        engine = null;
        _forceMainThread = true;
        result = await withTimeout(ensureEngine().layout(contract.graph));
      } else {
        throw err;
      }
    }
    return {
      request_id: requestId,
      schema: 'hysim_elk_layout_result_v1',
      positions: flattenNodes(result),
      width: Number(result.width) || 0,
      height: Number(result.height) || 0,
      runtime_ms: performance.now() - started,
      worker: _usedWorker,
    };
  }

  function reset() {
    if (engine?.terminateWorker) engine.terminateWorker();
    engine = null;
  }

  core.LayoutEngine = Object.freeze({
    schema: 'hysim_layout_engine_v1',
    layout,
    reset,
  });
})(window);
