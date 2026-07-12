/** Windowing and peak-preserving sampling for long Plotly time series. */
'use strict';

(function initTimeSeriesWindow(global) {
  const core = global.HySimCore = global.HySimCore || {};

  function clampWindow(total, start, size) {
    const safeTotal = Math.max(0, Number(total) || 0);
    const safeSize = Math.max(1, Math.min(safeTotal || 1, Number(size) || safeTotal || 1));
    const safeStart = Math.max(0, Math.min(Math.max(0, safeTotal - safeSize), Number(start) || 0));
    return { start: safeStart, end: Math.min(safeTotal, safeStart + safeSize), size: safeSize, total: safeTotal };
  }

  function peakPreservingIndices(values, start, end, maxPoints) {
    const length = Math.max(0, end - start);
    const limit = Math.max(4, Number(maxPoints) || length);
    if (length <= limit) return Array.from({ length }, (_, index) => start + index);
    const bucketCount = Math.max(1, Math.floor((limit - 2) / 2));
    const bucketSize = length / bucketCount;
    const selected = new Set([start, end - 1]);
    for (let bucket = 0; bucket < bucketCount; bucket += 1) {
      const first = Math.max(start, Math.floor(start + bucket * bucketSize));
      const last = Math.min(end, Math.ceil(start + (bucket + 1) * bucketSize));
      let minIndex = first, maxIndex = first;
      let minValue = Infinity, maxValue = -Infinity;
      for (let index = first; index < last; index += 1) {
        const value = Number(values?.[index]);
        if (!Number.isFinite(value)) continue;
        if (value < minValue) { minValue = value; minIndex = index; }
        if (value > maxValue) { maxValue = value; maxIndex = index; }
      }
      selected.add(minIndex);
      selected.add(maxIndex);
    }
    return Array.from(selected).sort((a, b) => a - b);
  }

  function build({ x = [], series = {}, primary = [], start = 0, size, maxPoints = 2000 }) {
    const total = Array.isArray(x) ? x.length : 0;
    const window = clampWindow(total, start, size ?? total);
    const indices = peakPreservingIndices(primary, window.start, window.end, maxPoints);
    const values = {};
    Object.entries(series).forEach(([key, input]) => {
      values[key] = indices.map(index => Array.isArray(input) ? input[index] : undefined);
    });
    return {
      schema: 'hysim_timeseries_window_v1',
      ...window,
      rendered: indices.length,
      indices,
      x: indices.map(index => x[index]),
      series: values,
    };
  }

  core.TimeSeriesWindow = Object.freeze({
    schema: 'hysim_timeseries_window_v1',
    clampWindow,
    peakPreservingIndices,
    build,
  });
})(window);
