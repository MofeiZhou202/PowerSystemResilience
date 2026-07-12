/** Asynchronous ELK client. The bundled ELK runtime owns its official Worker. */
'use strict';

(function initLayoutEngine(global) {
  const core = global.HySimCore = global.HySimCore || {};
  let engine = null;
  let sequence = 0;

  function ensureEngine() {
    if (engine) return engine;
    if (typeof global.ELK !== 'function') throw new Error('ELK bundle is unavailable');
    engine = new global.ELK();
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
    let timer;
    const timeout = new Promise((_, reject) => {
      timer = setTimeout(() => reject(new Error(`ELK layout timed out after ${timeoutMs} ms`)), timeoutMs);
    });
    try {
      const result = await Promise.race([ensureEngine().layout(contract.graph), timeout]);
      return {
        request_id: requestId,
        schema: 'hysim_elk_layout_result_v1',
        positions: flattenNodes(result),
        width: Number(result.width) || 0,
        height: Number(result.height) || 0,
        runtime_ms: performance.now() - started,
      };
    } finally {
      clearTimeout(timer);
    }
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
