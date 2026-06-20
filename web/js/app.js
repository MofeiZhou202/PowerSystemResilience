/**
 * app.js — Main application: API communication, UI wiring, topology tables, results display.
 */
'use strict';

const API_BASE = window.location.origin;  // Same origin as the C++ server

const App = (() => {
  // ========== Canvas Dirty Tracking ==========
  // When a built-in case or JSON is loaded, the backend already has the correct
  // system. We mark the canvas "clean". Any edits on the canvas set it "dirty",
  // and only dirty canvases need a roundtrip sync before analysis.
  let _canvasDirty = false;

  // ========== Console Logging ==========
  function log(msg, level = 'info') {
    const el = document.getElementById('consoleLog');
    if (!el) return;
    const time = new Date().toLocaleTimeString('zh-CN', { hour12: false });
    const div = document.createElement('div');
    div.className = `log-${level}`;
    div.innerHTML = `<span class="log-time">[${time}]</span>${escapeHtml(msg)}`;
    el.appendChild(div);
    el.scrollTop = el.scrollHeight;
  }

  function escapeHtml(str) {
    const d = document.createElement('div');
    d.textContent = str;
    return d.innerHTML;
  }

  // ========== Display Unit Helpers ==========
  // Reads the pfDisplayUnit selector (MW / kW / W) and converts power values.
  // All backend values are in MW/MVar; these helpers scale for display.
  function getPowerUnit() {
    const el = document.getElementById('pfDisplayUnit');
    return el ? el.value : 'MW';
  }
  function pScale() { const u = getPowerUnit(); return u === 'kW' ? 1e3 : u === 'W' ? 1e6 : 1; }
  function pUnit()  { const u = getPowerUnit(); return u === 'kW' ? 'kW' : u === 'W' ? 'W' : 'MW'; }
  function qUnit()  { const u = getPowerUnit(); return u === 'kW' ? 'kVar' : u === 'W' ? 'Var' : 'MVar'; }
  function pConv(mw) { return mw * pScale(); }
  function pFmt(mw, d = 2) { return (mw * pScale()).toFixed(d); }

  // Cache for re-rendering on unit change without re-running solver
  let _lastPfData = null;
  let _lastTspfData = null;
  let _lastReliabilityData = null;
  let _lastResilienceData = null;

  // ========== Per-module result group switching ==========
  // Each calc display function calls setActiveResultGroup(name). CSS in
  // style.css uses #resultsContent[data-active-group=...] to show only the
  // matching .result-group block, so different functional modules don't
  // visually mix their results.
  function setActiveResultGroup(name) {
    const rc = document.getElementById('resultsContent');
    if (rc) rc.dataset.activeGroup = name || '';
  }

  // Trigger a JSON file download in the browser.
  function downloadJsonFile(filename, obj) {
    try {
      const text = JSON.stringify(obj, null, 2);
      const blob = new Blob([text], { type: 'application/json;charset=utf-8' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url; a.download = filename;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      setTimeout(() => URL.revokeObjectURL(url), 1000);
      log(`已导出: ${filename}`, 'success');
    } catch (e) {
      log(`导出失败: ${e.message || e}`, 'error');
    }
  }

  function tsTagForFilename() {
    const d = new Date();
    const pad = (n) => String(n).padStart(2, '0');
    return `${d.getFullYear()}${pad(d.getMonth()+1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
  }

  // ========== API Client ==========
  async function apiPost(path, body = {}) {
    const url = `${API_BASE}${path}`;
    log(`POST ${path}`, 'info');
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const data = await res.json();
      if (!res.ok) {
        log(`Error: ${data.error || res.statusText}`, 'error');
        return null;
      }
      return data;
    } catch (e) {
      log(`Network error: ${e.message}`, 'error');
      return null;
    }
  }

  // Like apiPost but surfaces the backend error message instead of swallowing
  // it.  Returns { ok, data, error } so callers can show a specific reason
  // (e.g. an unknown fault bus id rejected by the server).
  async function apiPostResult(path, body = {}) {
    const url = `${API_BASE}${path}`;
    log(`POST ${path}`, 'info');
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const data = await res.json();
      if (!res.ok) {
        const error = (data && data.error) || res.statusText;
        log(`Error: ${error}`, 'error');
        return { ok: false, data: null, error };
      }
      return { ok: true, data, error: null };
    } catch (e) {
      log(`Network error: ${e.message}`, 'error');
      return { ok: false, data: null, error: e.message };
    }
  }

  async function apiGet(path) {
    const url = `${API_BASE}${path}`;
    try {
      const res = await fetch(url);
      const data = await res.json();
      if (!res.ok) {
        log(`Error: ${data.error || res.statusText}`, 'error');
        return null;
      }
      return data;
    } catch (e) {
      log(`Network error: ${e.message}`, 'error');
      return null;
    }
  }

  // ========== Status ==========
  function setStatus(text, type = '') {
    const badge = document.getElementById('statusBadge');
    if (badge) {
      badge.textContent = text;
      badge.className = 'badge' + (type ? ' ' + type : '');
    }
  }

  // ========== Load Built-in Cases ==========
  async function loadCaseList() {
    const data = await apiGet('/api/cases');
    if (!data) return;
    const select = document.getElementById('caseSelect');
    select.innerHTML = '<option value="">-- 加载算例 --</option>';
    (data.cases || []).forEach(c => {
      const opt = document.createElement('option');
      opt.value = c;
      opt.textContent = c;
      select.appendChild(opt);
    });
    log(`已加载 ${(data.cases || []).length} 个内置算例`, 'success');
  }

  async function loadMatpowerFileList() {
    const data = await apiGet('/api/matpower_files');
    if (!data) return;
    const select = document.getElementById('matpowerSelect');
    select.innerHTML = '<option value="">-- MATPOWER文件 --</option>';
    (data.files || []).forEach(f => {
      const opt = document.createElement('option');
      opt.value = f;
      opt.textContent = f;
      select.appendChild(opt);
    });
    log(`已加载 ${(data.files || []).length} 个MATPOWER文件`, 'success');
  }

  async function loadMatpowerCase(filename) {
    if (!filename) return;
    setStatus('加载MATPOWER...', 'busy');
    const data = await apiPost('/api/session/load_matpower', { filename });
    if (data) {
      log(`已加载MATPOWER文件: ${filename}`, 'success');
      if (data._raw_json) {
        try {
          const sys = JSON.parse(data._raw_json);
          Canvas.loadFromSystemJson(sys);
          _canvasDirty = false;  // backend already has the correct system
          log(`画布已更新: ${data.counts ? JSON.stringify(data.counts) : ''}`, 'info');
        } catch (e) {
          log(`JSON解析失败: ${e.message}`, 'error');
        }
      }
      setStatus('就绪');

      // Auto-run power flow after import
      log('自动运行潮流计算...', 'info');
      await runPowerFlow();
    } else {
      setStatus('加载失败', 'error');
    }
  }

  async function loadBuiltinCase(caseName) {
    if (!caseName) return;
    setStatus('加载中...', 'busy');
    const data = await apiPost('/api/session/load_builtin', { case: caseName });
    if (data) {
      log(`已加载算例: ${caseName}`, 'success');
      // Parse and display on canvas
      if (data._raw_json) {
        try {
          const sys = JSON.parse(data._raw_json);
          Canvas.loadFromSystemJson(sys);
          _canvasDirty = false;  // backend already has the correct system
          log(`画布已更新: ${data.counts ? JSON.stringify(data.counts) : ''}`, 'info');
        } catch (e) {
          log(`JSON解析失败: ${e.message}`, 'error');
        }
      }
      setStatus('就绪');
    } else {
      setStatus('加载失败', 'error');
    }
  }

  // Render a freshly-loaded backend system onto the canvas (shared by the ETAP
  // import paths); logs any permissive-import warnings.
  function applyLoadedSystem(data, label) {
    if (data._raw_json) {
      try {
        const sys = JSON.parse(data._raw_json);
        Canvas.loadFromSystemJson(sys);
        _canvasDirty = false;  // backend already has the correct system
        log(`画布已更新: ${data.counts ? JSON.stringify(data.counts) : ''}`, 'info');
      } catch (e) {
        log(`JSON解析失败: ${e.message}`, 'error');
      }
    }
    const warns = data._etap_warnings;
    if (Array.isArray(warns) && warns.length) {
      log(`${label}：${warns.length} 条导入告警（宽松模式）`, 'warn');
    }
  }

  // Import an ETAP-schema .xlsx workbook (raw binary upload -> load_etap).
  async function loadEtapXlsx(file) {
    if (!file) return;
    setStatus('导入ETAP工作簿...', 'busy');
    try {
      const buf = await file.arrayBuffer();
      const res = await fetch(`${API_BASE}/api/session/load_etap_xlsx`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/octet-stream' },
        body: buf,
      });
      const data = await res.json();
      if (!res.ok || data.error) {
        log(`导入ETAP工作簿失败: ${data.error || res.statusText}`, 'error');
        setStatus('加载失败', 'error');
        return;
      }
      log(`已导入ETAP工作簿: ${file.name}`, 'success');
      applyLoadedSystem(data, 'ETAP工作簿');
      setStatus('就绪');
    } catch (e) {
      log(`导入ETAP工作簿失败: ${e.message}`, 'error');
      setStatus('加载失败', 'error');
    }
  }

  // Import a native ETAP project .xml (text upload -> load_etap_xml).
  async function loadEtapXml(file) {
    if (!file) return;
    setStatus('导入ETAP工程...', 'busy');
    try {
      const xml = await file.text();
      const data = await apiPost('/api/session/load_etap_xml', { xml_string: xml });
      if (!data) { setStatus('加载失败', 'error'); return; }
      log(`已导入ETAP工程: ${file.name}`, 'success');
      applyLoadedSystem(data, 'ETAP工程');
      setStatus('就绪');
    } catch (e) {
      log(`导入ETAP工程失败: ${e.message}`, 'error');
      setStatus('加载失败', 'error');
    }
  }

  async function createNewSystem() {
    setStatus('创建中...', 'busy');
    const data = await apiPost('/api/session/new_empty');
    if (data) {
      Canvas.clearAll();
      _canvasDirty = false;  // backend already has the empty system
      log('已创建空白系统', 'success');
      setStatus('就绪');
    } else {
      setStatus('创建失败', 'error');
    }
  }

  // ========== Export / Import ==========
  async function exportJson() {
    // Build system from canvas
    const sys = Canvas.buildSystemJson();
    const jsonStr = JSON.stringify(sys, null, 2);
    const blob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'power_system.json';
    a.click();
    URL.revokeObjectURL(url);
    log('已导出系统JSON', 'success');
  }

  // Export the current system as an ETAP-schema .xlsx workbook (one sheet per
  // ETAP element class).  The workbook is generated server-side by save_etap();
  // here we push the latest canvas to the backend (if edited), then stream the
  // binary response to a browser download.
  async function exportEtap() {
    setStatus('导出ETAP中...', 'busy');
    try {
      // Make sure the backend session reflects any unsaved canvas edits.  A
      // non-forced sync is a no-op when the canvas is unchanged, preserving the
      // full-fidelity system originally loaded into the backend.
      const ok = await syncToBackend();
      if (!ok) { setStatus('导出失败', 'error'); return; }
      // Send an explicit (empty JSON) body so the request always carries a
      // Content-Length; a body-less POST can stall some HTTP servers.
      const resp = await fetch('/api/session/export_etap', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: '{}',
      });
      if (!resp.ok) {
        let msg = 'HTTP ' + resp.status;
        try { const j = await resp.json(); if (j && j.error) msg = j.error; } catch (_) {}
        throw new Error(msg);
      }
      const blob = await resp.blob();
      let filename = 'system.xlsx';
      const cd = resp.headers.get('Content-Disposition');
      if (cd) {
        const m = /filename="?([^"]+)"?/.exec(cd);
        if (m && m[1]) filename = m[1];
      }
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      a.click();
      URL.revokeObjectURL(url);
      log(`已导出ETAP工作簿: ${filename}`, 'success');
      setStatus('就绪');
    } catch (err) {
      log(`导出ETAP失败: ${err.message}`, 'error');
      setStatus('导出失败', 'error');
    }
  }

  function importJson(file) {
    const reader = new FileReader();
    reader.onload = async (e) => {
      try {
        const sys = JSON.parse(e.target.result);
        // Send to backend
        const data = await apiPost('/api/session/load_json_string', {
          json_string: JSON.stringify(sys)
        });
        if (data) {
          if (data._raw_json) {
            // The backend strips unknown fields from the system JSON, so the
            // _canvas layout block (added by Canvas.buildSystemJson) is lost
            // on the round-trip.  Re-attach it from the user's original
            // upload so that loadFromSystemJson can restore positions /
            // connections / viewport exactly instead of running autoLayout.
            const raw = JSON.parse(data._raw_json);
            if (sys && sys._canvas && !raw._canvas) {
              raw._canvas = sys._canvas;
            }
            Canvas.loadFromSystemJson(raw);
          } else {
            Canvas.loadFromSystemJson(sys);
          }
          _canvasDirty = false;  // backend already has the imported system
          log('已导入系统JSON', 'success');
        }
      } catch (err) {
        log(`导入失败: ${err.message}`, 'error');
      }
    };
    reader.readAsText(file);
  }

  // ========== Sync Canvas to Backend ==========
  async function syncToBackend(force = false) {
    // If not forced and the canvas hasn't been modified since the last backend load,
    // skip the roundtrip — the backend already has the correct system.
    if (!force && !_canvasDirty) {
      console.log('[syncToBackend] skipped — canvas not dirty');
      return true;
    }
    const sys = Canvas.buildSystemJson();
    const jsonStr = JSON.stringify(sys);
    console.log('[syncToBackend] canvas JSON length:', jsonStr.length);
    console.log('[syncToBackend] DCDC converters:', JSON.stringify(sys.dcdc_converters, null, 2));
    console.log('[syncToBackend] VSC converters:', JSON.stringify(sys.vsc_converters, null, 2));
    console.log('[syncToBackend] DC buses:', JSON.stringify(sys.dc?.buses, null, 2));
    window.__lastSyncJson = jsonStr;  // for debugging in console
    const data = await apiPost('/api/session/load_json_string', { json_string: jsonStr });
    if (!data) {
      log('同步到后端失败', 'error');
      return false;
    }
    _canvasDirty = false;
    return true;
  }

  // ========== Power Flow ==========
  async function runPowerFlow() {
    setStatus('潮流计算中...', 'busy');
    const method = document.getElementById('pfMethod').value;

    // Sync canvas to backend first
    if (!await syncToBackend()) {
      setStatus('同步失败', 'error');
      return;
    }

    const data = await apiPost('/api/session/pf', {
      method: method,
      options: { max_iter: 100, tol: 1e-8, verbose: false }
    });

    if (data) {
      const converged = data.converged;
      const islandInfo = data.islands_detected
        ? ` [检测到${data.islands_detected}个岛, ${data.solvable_islands}个可解]`
        : '';
      if (converged) {
        log(`潮流计算收敛 [${data.method_actual || method}]: 迭代${data.iterations}次, 残差=${Number(data.residual).toExponential(4)}${islandInfo}`, 'success');
        setStatus('潮流收敛', '');
      } else {
        log(`潮流计算未收敛 [${data.method_actual || method}]: 迭代${data.iterations}次${islandInfo}`, 'warn');
        setStatus('未收敛', 'error');
      }

      // Display results
      _lastPfData = data;
      Canvas.showPowerFlowResults(data);
      showPowerFlowResultsTables(data);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
  }

  // ========== Short Circuit ==========
  function showScDialog() {
    document.getElementById('scDialog').style.display = 'flex';
  }

  function hideScDialog() {
    document.getElementById('scDialog').style.display = 'none';
  }

  async function runShortCircuit() {
    hideScDialog();
    setStatus('短路计算中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const faultBusRaw = (document.getElementById('scFaultBus').value || '').trim();
    const faultType = document.getElementById('scFaultType').value;
    const cFactor = parseFloat(document.getElementById('scCFactor').value);

    // Blank fault bus → short circuit at every bus (overview mode).
    if (faultBusRaw === '') {
      const data = await apiPost('/api/session/sc', {
        options: {
          fault_type: faultType,
          c_factor: cFactor,
          compute_all_buses: true,
        }
      });
      if (data) {
        log(`短路计算完成: ${data.bus_results?.length || 0} 个母线结果`, 'success');
        setStatus('短路完成 (全部母线)');
        showShortCircuitResults(data);
        switchTab('results');
      } else {
        setStatus('计算失败', 'error');
      }
      return;
    }

    // A specific fault location was set → run a validated single-bus fault.
    // The server resolves the id against the real bus indices and returns
    // HTTP 400 when the id does not exist, so a non-existent bus can no
    // longer "still calculate".
    const faultBus = parseInt(faultBusRaw, 10);
    if (!Number.isInteger(faultBus)) {
      setStatus(`故障母线 ID “${faultBusRaw}” 无效，请输入整数母线编号。`, 'error');
      return;
    }

    const resp = await apiPostResult('/api/session/sc_detailed', {
      fault_bus_ids: [faultBus],
      fault_type: faultType,
      c_factor: cFactor,
    });

    if (!resp.ok) {
      const raw = resp.error || '';
      const friendly = /not found/i.test(raw)
        ? `故障母线 ID ${faultBus} 不存在，请输入系统中真实存在的母线编号（与结果表 / 画布中显示的 Bus 编号一致）。`
        : (raw || '计算失败');
      setStatus(friendly, 'error');
      return;
    }

    log(`短路计算完成: 故障母线 ${faultBus}`, 'success');
    setStatus(`短路完成 (故障母线 ${faultBus})`);
    showShortCircuitDetailedResults(resp.data, faultBus);
    switchTab('results');
  }

  // ========== Topology Reconfiguration ==========
  async function runTopologyReconfig() {
    setStatus('拓扑重构中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const data = await apiPost('/api/session/run_reconfig', {
      options: {
        v_min_pu: 0.95,
        v_max_pu: 1.05,
        mip_gap: 0.01,
        max_time_s: 60,
        verbose: false,
      }
    });

    if (data) {
      if (data.feasible !== false) {
        const baseLoss = data.base_loss_mw?.toFixed(4) || '?';
        const reconLoss = data.reconfig_loss_mw?.toFixed(4) || '?';
        const redPct = data.loss_reduction_pct?.toFixed(1) || '0';
        log(`拓扑重构完成: 重构前损耗=${baseLoss}MW, 重构后损耗=${reconLoss}MW, 降低${redPct}%, ` +
            `辐射状=${data.reconfig_is_radial ? '是' : '否'}, 连通=${data.reconfig_is_connected ? '是' : '否'}`, 'success');
        setStatus('拓扑重构完成');
      } else {
        log('拓扑重构: 未找到可行解', 'warn');
        setStatus('不可行', 'error');
      }
      showTopologyResults(data);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
  }

  // ========== Bearing Capability Assessment (DL/T 2041-2019) ==========
  function showBcDialog() {
    document.getElementById('bcDialog').style.display = 'flex';
  }

  function hideBcDialog() {
    document.getElementById('bcDialog').style.display = 'none';
  }

  async function runBearingCapacity() {
    hideBcDialog();
    setStatus('承载力评估中 (潮流+短路+校核)...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    const kr = parseFloat(document.getElementById('bcKr').value) || 0.8;
    const delta_UH = parseFloat(document.getElementById('bcDeltaUH').value) || 7.0;
    const delta_UL = parseFloat(document.getElementById('bcDeltaUL').value) || 7.0;
    const thd_limit = parseFloat(document.getElementById('bcThdLimit').value) || 5.0;

    const data = await apiPost('/api/session/run_bearing_capacity', {
      kr, delta_UH_pct: delta_UH, delta_UL_pct: delta_UL, thd_limit_pct: thd_limit
    });

    if (data && data.converged) {
      log(`承载力评估完成: ${data.bus_results?.length || 0} 母线, ${data.branch_results?.length || 0} 支路`, 'success');
      setStatus('承载力评估完成');
      showBearingCapResults(data);
      switchTab('results');
    } else if (data && data.error) {
      log(`承载力评估失败: ${data.error}`, 'error');
      setStatus('评估失败', 'error');
    } else {
      setStatus('评估失败', 'error');
    }
  }

  function gradeHtml(grade) {
    const labels = { green: '🟢 绿色', yellow: '🟡 黄色', red: '🔴 红色' };
    return `<span class="grade-${grade}">${labels[grade] || grade}</span>`;
  }

  function showBearingCapResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    document.getElementById('bearingCapSection').style.display = 'block';
    setActiveResultGroup('hosting');

    // Summary
    const summary = document.getElementById('bearingCapSummary');
    const worstGrade = data.area_results?.reduce((w, a) => {
      if (a.grade === 'red') return 'red';
      if (a.grade === 'yellow' && w !== 'red') return 'yellow';
      return w;
    }, 'green') || 'green';
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">潮流收敛</span>
        <span class="result-value result-converged">✓ 收敛 (${data.pf_iterations} 次迭代)</span></div>
      <div class="result-item"><span class="result-label">裕度系数 k<sub>r</sub></span>
        <span class="result-value">${data.kr}</span></div>
      <div class="result-item"><span class="result-label">电压偏差限值 (GB/T 12325)</span>
        <span class="result-value">+${data.delta_UH_limit}% / -${data.delta_UL_limit}%</span></div>
      <div class="result-item"><span class="result-label">谐波限值 THD (GB/T 14549)</span>
        <span class="result-value">${data.thd_limit_pct || 5.0}%</span></div>
      <div class="result-item"><span class="result-label">向220kV+反送电</span>
        <span class="result-value ${data.any_reverse_220kv ? 'result-failed' : 'result-converged'}">
          ${data.any_reverse_220kv ? '✗ 存在 → 直接判红' : '✓ 无'}</span></div>
      <div class="result-item"><span class="result-label">综合评估等级</span>
        <span class="result-value">${gradeHtml(worstGrade)}</span></div>
    `;
    document.getElementById('resultsSummary').innerHTML = summary.innerHTML;

    // Area results
    const areaDiv = document.getElementById('bearingAreaResults');
    if (data.area_results && data.area_results.length > 0) {
      let html = '<table><thead><tr><th>区域</th><th>评估等级</th></tr></thead><tbody>';
      data.area_results.forEach(a => {
        html += `<tr><td>Area ${a.area}</td><td>${gradeHtml(a.grade)}</td></tr>`;
      });
      html += '</tbody></table>';
      areaDiv.innerHTML = html;
    }

    // Branch/Transformer thermal stability table (DL/T 2041 §5.1)
    const brDiv = document.getElementById('bearingBranchResults');
    if (data.branch_results && data.branch_results.length > 0) {
      const busMap = Canvas.getCompBusMap();
      let html = '<table><thead><tr>' +
        '<th>评估线路/变压器</th><th>类型</th><th>From</th><th>To</th>' +
        '<th>S<sub>e</sub>(MVA)</th><th>P(MW)</th><th>负载率(%)</th>' +
        '<th>反向</th><th>λ(%)</th><th>P<sub>m</sub>(MW)</th><th>等级</th>' +
        '</tr></thead><tbody>';
      data.branch_results.forEach(br => {
        const tg = br.thermal_grade || 'green';
        const cls = tg === 'red' ? 'grade-red' : (tg === 'yellow' ? 'grade-yellow' :
                    (tg === 'no_rating' ? '' : 'grade-green'));
        const compId = (br.branch_pos >= 0) ? busMap.branch?.[br.branch_pos] : undefined;
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        const typeLabel = br.equip_type === 'transformer' ? '变压器' : '线路';
        const seStr = br.has_rating ? br.se_mva.toFixed(1) : '<span style="color:#999">未设</span>';
        const lamStr = br.has_rating ? br.lambda_pct.toFixed(1) : '—';
        const pmStr = br.has_rating ? br.pm_mw.toFixed(1) : '—';
        const loadStr = br.has_rating ? br.loading_pct.toFixed(1) : '—';
        const gradeStr = tg === 'no_rating' ? '<span style="color:#999">未设额定</span>' : gradeHtml(tg);
        html += `<tr${attr}>
          <td>${br.name}</td><td>${typeLabel}</td>
          <td>${br.from_bus}</td><td>${br.to_bus}</td>
          <td>${seStr}</td><td>${br.pf_mw.toFixed(2)}</td>
          <td>${loadStr}</td>
          <td>${br.reverse_flow ? '⚠️ 反向' : '—'}</td>
          <td class="${cls}">${lamStr}</td>
          <td>${pmStr}</td>
          <td>${gradeStr}</td></tr>`;
      });
      html += '</tbody></table>';
      brDiv.innerHTML = html;
    } else {
      brDiv.innerHTML = '<p style="color:#888">无支路数据</p>';
    }

    // Bus results — SC check, voltage deviation check, harmonic check
    const busDiv = document.getElementById('bearingBusResults');
    if (data.bus_results && data.bus_results.length > 0) {
      const busMap = Canvas.getCompBusMap();
      let html = '<table><thead><tr>' +
        '<th>Bus</th><th>kV</th><th>V<sub>m</sub>(pu)</th>' +
        '<th>Ik"(kA)</th><th>I<sub>m</sub>(kA)</th><th>短路校核</th>' +
        '<th>δU<sub>H</sub>(%)</th><th>δU<sub>L</sub>(%)</th><th>电压校核</th>' +
        '<th>谐波校核</th><th>DG(MW)</th><th>等级</th>' +
        '</tr></thead><tbody>';
      data.bus_results.forEach(br => {
        const compId = busMap.ac?.[br.bus_id];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        const scCls = br.sc_pass ? 'grade-green' : 'grade-red';
        const vCls = br.voltage_pass ? 'grade-green' : 'grade-red';
        const harmStatus = br.harmonic_status || 'no_data';
        const harmCls = harmStatus === 'no_data' ? '' : (br.harmonic_pass ? 'grade-green' : 'grade-red');
        const harmLabel = harmStatus === 'no_data' ? '<span style="color:#999">无数据</span>' :
                          (br.harmonic_pass ? '✓' : '✗');
        html += `<tr${attr}>
          <td>${br.bus_id}</td><td>${br.base_kv}</td>
          <td>${br.vm_pu.toFixed(4)}</td>
          <td>${br.ikss_ka.toFixed(3)}</td>
          <td>${br.i_breaker_ka > 0 ? br.i_breaker_ka.toFixed(2) : '<span style="color:#999">未设</span>'}</td>
          <td class="${scCls}">${br.sc_pass ? '✓ 通过' : '✗ 超限'}</td>
          <td>${br.delta_u_h_pct.toFixed(2)}</td>
          <td>${br.delta_u_l_pct.toFixed(2)}</td>
          <td class="${vCls}">${br.voltage_pass ? '✓ 通过' : '✗ 超限'}</td>
          <td class="${harmCls}">${harmLabel}</td>
          <td>${(br.dg_mw || 0).toFixed(2)}</td>
          <td>${gradeHtml(br.grade)}</td></tr>`;
      });
      html += '</tbody></table>';
      busDiv.innerHTML = html;
    }

    // Comprehensive summary table (DL/T 2041 Appendix C)
    const sumDiv = document.getElementById('bearingSummaryTable');
    if (data.summary_table && data.summary_table.length > 0) {
      let html = '<table><thead><tr>' +
        '<th>评估对象</th><th>类型</th><th>热稳定 λ(%)</th>' +
        '<th>短路校核</th><th>电压校核</th><th>谐波校核</th>' +
        '<th>评估等级</th><th>P<sub>m</sub>(MW)</th>' +
        '</tr></thead><tbody>';
      data.summary_table.forEach(row => {
        const typeLabel = row.type === 'transformer' ? '变压器' :
                          (row.type === 'line' ? '线路' : (row.type === 'bus' ? '母线' : row.type));
        const lamStr = row.has_rating ? row.lambda_pct.toFixed(1) : '—';
        const scStr = row.sc_pass ? '✓' : '✗';
        const vStr = row.voltage_pass ? '✓' : '✗';
        const harmStr = (row.harmonic_status === 'no_data') ? '无数据' :
                        (row.harmonic_pass ? '✓' : '✗');
        const pmStr = row.pm_mw > 0 ? row.pm_mw.toFixed(1) : '—';
        html += `<tr>
          <td>${row.name}</td><td>${typeLabel}</td>
          <td>${lamStr}</td>
          <td>${scStr}</td><td>${vStr}</td><td>${harmStr}</td>
          <td>${gradeHtml(row.grade)}</td>
          <td>${pmStr}</td></tr>`;
      });
      html += '</tbody></table>';
      html += '<p style="color:#888;font-size:0.85em;margin-top:4px">注：下级电网评估等级应不高于上级电网的评估等级。</p>';
      sumDiv.innerHTML = html;
    }
  }

  // ========== Time-Series Power Flow ==========
  function showTspfDialog() {
    document.getElementById('tspfDialog').style.display = 'flex';
  }

  function hideTspfDialog() {
    document.getElementById('tspfDialog').style.display = 'none';
  }

  function parseCsvProfiles(text) {
    const lines = text.trim().split('\n').map(l => l.split(',').map(s => s.trim()));
    if (lines.length < 2) return null;
    const headers = lines[0];
    const numProfiles = headers.length;
    const profiles = headers.map((name, i) => ({ id: i, name: name, values: [] }));
    for (let r = 1; r < lines.length; r++) {
      for (let c = 0; c < numProfiles; c++) {
        profiles[c].values.push(parseFloat(lines[r][c]) || 0);
      }
    }
    return { num_steps: lines.length - 1, step_duration_hr: 1.0, profiles };
  }

  async function runTimeSeriesPF() {
    setStatus('时序潮流计算中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    // Read parameters from the inline time-series sub-toolbar.
    // 仿真时间(小时) = num_steps × step_duration_hr (here step_duration_hr = 1).
    const simHours = parseInt(document.getElementById('simulationHours')?.value, 10);
    const numSteps = (Number.isFinite(simHours) && simHours > 0) ? simHours : 24;
    const skipUC = document.getElementById('tspfSkipUC')?.checked ?? true;
    const runOPF = document.getElementById('tspfRunOPF')?.checked ?? false;

    const data = await apiPost('/api/session/run_ts_pf', {
      num_steps: numSteps,
      skip_uc: skipUC,
      run_opf: runOPF,
    });

    if (data) {
      log(`时序潮流完成: ${data.num_converged}/${data.num_steps}步收敛, 总发电成本=$${(data.total_generation_cost || 0).toFixed(0)}`, 'success');
      setStatus('时序潮流完成');
      _lastTspfData = data;
      showTimeSeriesResults(data);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
  }

  function showTimeSeriesResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('timeSeries');

    const section = document.getElementById('tspfResultsSection');
    section.style.display = 'block';

    // Summary KPIs
    const summary = document.getElementById('tspfSummary');
    const ucStatus = data.uc_feasible === false ? '失败' : (data.uc_feasible === true ? '成功' : '—');
    const ucClass = data.uc_feasible === false ? 'result-failed' : 'result-converged';
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">时间步数</span>
        <span class="result-value">${data.num_steps}</span></div>
      <div class="result-item"><span class="result-label">潮流收敛</span>
        <span class="result-value ${data.num_converged === data.num_steps ? 'result-converged' : 'result-failed'}">
          ${data.num_converged}/${data.num_steps}</span></div>
      <div class="result-item"><span class="result-label">OPF收敛</span>
        <span class="result-value">${data.num_opf_converged}/${data.num_steps}</span></div>
      <div class="result-item"><span class="result-label">机组组合</span>
        <span class="result-value ${ucClass}">${ucStatus}${data.uc_solver_name ? ' (' + data.uc_solver_name + ')' : ''}</span></div>
      <div class="result-item"><span class="result-label">总发电成本</span>
        <span class="result-value">$${(data.total_generation_cost || 0).toFixed(0)}</span></div>
    `;

    const hrs = Array.from({ length: data.num_steps }, (_, i) => i);
    const pal = ['#0b6e4f', '#2c8c99', '#b5651d', '#8e44ad', '#2980b9', '#e74c3c', '#27ae60', '#f39c12'];
    const plotTheme = {
      paper_bgcolor: 'rgba(0,0,0,0)',
      plot_bgcolor: 'rgba(0,0,0,0)',
      font: { color: '#abb2bf', size: 11 },
      xaxis: { gridcolor: '#3e4451', title: '时间步' },
      yaxis: { gridcolor: '#3e4451' },
      margin: { l: 55, r: 15, t: 40, b: 40 },
      legend: { orientation: 'h', y: -0.25 },
    };

    // 1. Generation dispatch stacked bar chart
    const genDiv = document.getElementById('tspfGenChart');
    const dtraces = (data.gen_dispatch || []).map((d, gi) => ({
      x: hrs, y: d, type: 'bar',
      name: (data.gen_names || [])[gi] || 'Gen ' + gi,
      marker: { color: pal[gi % pal.length] },
    }));
    (data.renewable_dispatch || []).forEach((rd, ri) => dtraces.push({
      x: hrs, y: rd, type: 'bar',
      name: (data.ren_names || [])[ri] || 'Ren ' + ri,
      marker: { color: '#27ae60' },
    }));
    if (dtraces.length > 0) {
      Plotly.newPlot(genDiv, dtraces, {
        ...plotTheme, title: '发电出力 (MW)', barmode: 'stack',
        yaxis: { ...plotTheme.yaxis, title: 'MW' },
      }, { responsive: true });
    } else {
      genDiv.innerHTML = '<p class="empty-hint">无发电出力数据</p>';
    }

    // 2. Mean voltage time series (with min/max band)
    const voltDiv = document.getElementById('tspfVoltChart');
    if (data.vm_mean && data.vm_mean.length > 0) {
      const allV = [...data.vm_mean, ...(data.vm_min||[]), ...(data.vm_max||[])].filter(v=>v>0);
      const vMin = Math.min(...allV), vMax = Math.max(...allV);
      const vPad = Math.max((vMax - vMin) * 0.15, 0.005);
      const traces = [];
      if (data.vm_min && data.vm_max) {
        traces.push({
          x: hrs, y: data.vm_max, mode: 'lines', line: { color: 'rgba(97,175,239,0.3)', width: 0 },
          name: '最大电压', showlegend: false,
        });
        traces.push({
          x: hrs, y: data.vm_min, mode: 'lines', line: { color: 'rgba(97,175,239,0.3)', width: 0 },
          fill: 'tonexty', fillcolor: 'rgba(97,175,239,0.13)',
          name: '电压范围 (min–max)',
        });
      }
      traces.push({
        x: hrs, y: data.vm_mean,
        mode: 'lines+markers',
        line: { color: '#61afef', width: 2 },
        name: '平均电压',
      });
      Plotly.newPlot(voltDiv, traces, {
        ...plotTheme, title: '各时步母线电压 (p.u.)',
        yaxis: { ...plotTheme.yaxis, title: 'p.u.', range: [vMin - vPad, vMax + vPad] },
      }, { responsive: true });
    } else {
      voltDiv.innerHTML = '<p class="empty-hint">无电压数据</p>';
    }

    // 3. Losses time series
    const lossDiv = document.getElementById('tspfLossChart');
    if (data.losses_mw && data.losses_mw.length > 0) {
      const lMin = Math.min(...data.losses_mw), lMax = Math.max(...data.losses_mw);
      const lPad = Math.max((lMax - lMin) * 0.15, lMax * 0.05 || 0.001);
      Plotly.newPlot(lossDiv, [{
        x: hrs, y: data.losses_mw,
        mode: 'lines+markers',
        line: { color: '#e06c75', width: 2 },
        name: '系统损耗',
      }], {
        ...plotTheme, title: '系统损耗 (MW)',
        yaxis: { ...plotTheme.yaxis, title: 'MW', range: [lMin - lPad, lMax + lPad] },
      }, { responsive: true });
    } else {
      lossDiv.innerHTML = '';
    }

    // 4. ESS SOC
    const essDiv = document.getElementById('tspfESSChart');
    if (data.ess_soc && data.ess_soc.length > 0) {
      const essTraces = data.ess_soc.map((soc, si) => ({
        x: hrs, y: soc, mode: 'lines+markers',
        name: (data.ess_names || [])[si] || 'ESS ' + si,
        line: { width: 2 },
      }));
      Plotly.newPlot(essDiv, essTraces, {
        ...plotTheme, title: '储能SOC',
        yaxis: { ...plotTheme.yaxis, title: 'SOC', range: [0, 1] },
      }, { responsive: true });
    } else {
      essDiv.innerHTML = '';
    }

    // 4b. AC ESS dispatch (charge/discharge)
    const essDispDiv = document.getElementById('tspfESSDispatchChart');
    if (data.ess_dispatch && data.ess_dispatch.length > 0) {
      const essDispTraces = data.ess_dispatch.map((d, si) => ({
        x: hrs, y: d, type: 'bar',
        name: (data.ess_names || [])[si] || 'ESS ' + si,
      }));
      const allVals = data.ess_dispatch.flat();
      const eMin = Math.min(...allVals, 0), eMax = Math.max(...allVals, 0);
      const ePad = Math.max((eMax - eMin) * 0.15, 0.1);
      Plotly.newPlot(essDispDiv, essDispTraces, {
        ...plotTheme, title: '储能充放电调度 (MW, 正=放电 负=充电)', barmode: 'relative',
        yaxis: { ...plotTheme.yaxis, title: 'MW', range: [eMin - ePad, eMax + ePad] },
      }, { responsive: true });
    } else {
      essDispDiv.innerHTML = '';
    }

    // 5. Total load demand
    const loadDiv = document.getElementById('tspfLoadChart');
    if (data.total_load && data.total_load.length > 0) {
      const dMin = Math.min(...data.total_load), dMax = Math.max(...data.total_load);
      const dPad = Math.max((dMax - dMin) * 0.15, dMax * 0.05 || 0.001);
      Plotly.newPlot(loadDiv, [{
        x: hrs, y: data.total_load,
        mode: 'lines+markers',
        line: { color: '#2c8c99', width: 2 },
        name: '总负荷',
      }], {
        ...plotTheme, title: '负荷需求曲线 (MW)',
        yaxis: { ...plotTheme.yaxis, title: 'MW', range: [dMin - dPad, dMax + dPad] },
      }, { responsive: true });
    } else {
      loadDiv.innerHTML = '';
    }

    // 6. Voltage heatmap (per-bus per-timestep)
    const heatDiv = document.getElementById('tspfVoltHeatmap');
    if (data.vm_matrix && data.vm_matrix.length > 0 && data.bus_labels) {
      Plotly.newPlot(heatDiv, [{
        z: data.vm_matrix,
        x: hrs,
        y: data.bus_labels,
        type: 'heatmap',
        colorscale: [[0,'#e74c3c'],[0.3,'#f39c12'],[0.5,'#27ae60'],[0.7,'#f39c12'],[1,'#e74c3c']],
        zmin: 0.9, zmax: 1.1,
        colorbar: { title: 'V (p.u.)' },
      }], {
        ...plotTheme, title: '母线电压热力图 (p.u.)',
        yaxis: { ...plotTheme.yaxis, title: '', automargin: true },
        margin: { l: 100, r: 15, t: 40, b: 40 },
      }, { responsive: true });
    } else {
      heatDiv.innerHTML = '';
    }

    // 7. PV dispatch (AC PV systems + DC PV arrays)
    const pvDiv = document.getElementById('tspfPVChart');
    const pvTraces = [];
    // AC PV dispatch from backend (computed from solar profile)
    (data.ac_pv_dispatch || []).forEach((d, ki) => pvTraces.push({
      x: hrs, y: d, type: 'bar',
      name: (data.pv_names || [])[ki] || 'PV ' + ki,
      marker: { color: '#e67e22' },
    }));
    // DC PV dispatch from backend
    (data.dc_pv_dispatch || []).forEach((d, ki) => pvTraces.push({
      x: hrs, y: d, type: 'bar',
      name: (data.dc_pv_names || [])[ki] || 'DC_PV ' + ki,
      marker: { color: '#f1c40f' },
    }));
    if (pvTraces.length > 0) {
      Plotly.newPlot(pvDiv, pvTraces, {
        ...plotTheme, title: '光伏出力 (MW)', barmode: 'stack',
        yaxis: { ...plotTheme.yaxis, title: 'MW' },
      }, { responsive: true });
    } else {
      pvDiv.innerHTML = '';
    }

    // 8. DC-side components (DC ESS dispatch)
    const dcDiv = document.getElementById('tspfDCChart');
    const dcTraces = [];
    (data.dc_ess_dispatch || []).forEach((d, ki) => dcTraces.push({
      x: hrs, y: d, mode: 'lines+markers',
      name: (data.dc_ess_names || [])[ki] || 'DC_ESS ' + ki,
      line: { width: 2 },
    }));
    if (dcTraces.length > 0) {
      Plotly.newPlot(dcDiv, dcTraces, {
        ...plotTheme, title: 'DC储能调度 (MW)',
        yaxis: { ...plotTheme.yaxis, title: 'MW' },
      }, { responsive: true });
    } else {
      dcDiv.innerHTML = '';
    }
  }

  // ========== Results Display ==========
  function showPowerFlowResultsTables(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('powerFlow');

    // Summary
    const summary = document.getElementById('resultsSummary');
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">方法</span>
        <span class="result-value">${data.method_actual || data.method || ''}</span></div>
      <div class="result-item"><span class="result-label">收敛</span>
        <span class="result-value ${data.converged ? 'result-converged' : 'result-failed'}">
          ${data.converged ? '✓ 是' : '✗ 否'}</span></div>
      <div class="result-item"><span class="result-label">迭代次数</span>
        <span class="result-value">${data.iterations || 0}</span></div>
      <div class="result-item"><span class="result-label">最大残差</span>
        <span class="result-value">${Number(data.residual || 0).toExponential(4)}</span></div>
    `;

    const busMap = Canvas.getCompBusMap();

    // AC Bus voltage table
    const busDiv = document.getElementById('pfBusResults');
    if (data.vm && data.vm.length > 0) {
      let html = '<table><thead><tr><th>Bus</th><th>Vm(pu)</th><th>Va(°)</th></tr></thead><tbody>';
      data.vm.forEach((vm, i) => {
        const va = data.va ? (data.va[i] * 180 / Math.PI).toFixed(4) : '0';
        const color = vm < 0.95 ? 'color:#e06c75' : vm > 1.05 ? 'color:#d19a66' : '';
        const compId = busMap.ac[i + 1];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        html += `<tr${attr}><td>${i + 1}</td><td style="${color}">${vm.toFixed(6)}</td><td>${va}</td></tr>`;
      });
      html += '</tbody></table>';
      busDiv.innerHTML = html;
    } else {
      busDiv.innerHTML = '<p class="empty-hint">无AC节点电压数据</p>';
    }

    // DC Bus voltage table
    const dcBusSec = document.getElementById('pfDcBusSection');
    const dcBusDiv = document.getElementById('pfDcBusResults');
    if (data.vdc && data.vdc.length > 0) {
      dcBusSec.style.display = '';
      let html = '<table><thead><tr><th>DC Bus</th><th>Vdc(pu)</th></tr></thead><tbody>';
      data.vdc.forEach((vdc, i) => {
        const color = vdc < 0.95 ? 'color:#e06c75' : vdc > 1.05 ? 'color:#d19a66' : '';
        const compId = busMap.dc[i + 1];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        html += `<tr${attr}><td>${i + 1}</td><td style="${color}">${vdc.toFixed(6)}</td></tr>`;
      });
      html += '</tbody></table>';
      dcBusDiv.innerHTML = html;
    } else {
      dcBusSec.style.display = 'none';
      dcBusDiv.innerHTML = '';
    }

    // Generator output table (from geo_gen solved data)
    const genSec = document.getElementById('pfGenSection');
    const genDiv = document.getElementById('pfGenResults');
    if (data.geo_gen && data.geo_gen.length > 0) {
      genSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>Bus</th><th>Name</th><th>Pg(${pUnit()})</th><th>Qg(${qUnit()})</th><th>Vg(pu)</th><th>Slack</th></tr></thead><tbody>`;
      data.geo_gen.forEach((g, i) => {
        const compId = busMap.gen ? busMap.gen[i] : undefined;
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        html += `<tr${attr}><td>${g.index ?? i}</td><td>${g.bus}</td><td>${g.name || ''}</td>`;
        html += `<td>${pFmt(g.pg_mw, 4)}</td><td>${pFmt(g.qg_mvar, 4)}</td>`;
        html += `<td>${(g.vg_pu || 1).toFixed(4)}</td><td>${g.is_slack ? '✓' : ''}</td></tr>`;
      });
      html += '</tbody></table>';
      genDiv.innerHTML = html;
    } else {
      genSec.style.display = 'none';
      genDiv.innerHTML = '';
    }

    // AC Branch flow table (prefer geo_ac_branches with full Pf/Pt/Q/Loss data)
    const brDiv = document.getElementById('pfBranchResults');
    if (data.geo_ac_branches && data.geo_ac_branches.length > 0) {
      let html = `<table><thead><tr><th>#</th><th>From</th><th>To</th><th>Pf(${pUnit()})</th><th>Pt(${pUnit()})</th><th>Qf(${qUnit()})</th><th>Qt(${qUnit()})</th><th>Loss(${pUnit()})</th><th>Loading%</th></tr></thead><tbody>`;
      data.geo_ac_branches.forEach((br, i) => {
        const compId = busMap.branch[i];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        const loss = (br.loss_mw != null) ? br.loss_mw : ((br.pf_mw || 0) + (br.pt_mw || 0));
        const ldg = br.loading_pct != null ? br.loading_pct.toFixed(1) + '%' : '-';
        const ldgStyle = (br.loading_pct || 0) > 100 ? ' style="color:#e06c75;font-weight:bold"' : '';
        html += `<tr${attr}><td>${i}</td><td>${br.from}</td><td>${br.to}</td>`;
        html += `<td>${pFmt(br.pf_mw || 0, 4)}</td><td>${pFmt(br.pt_mw || 0, 4)}</td>`;
        html += `<td>${pFmt(br.qf_mvar || 0, 4)}</td><td>${pFmt(br.qt_mvar || 0, 4)}</td>`;
        html += `<td>${pFmt(loss, 4)}</td><td${ldgStyle}>${ldg}</td></tr>`;
      });
      html += '</tbody></table>';
      brDiv.innerHTML = html;
    } else if (data.branch_abs && data.branch_abs.length > 0) {
      // Fallback: old branch_abs (|P| only)
      let html = `<table><thead><tr><th>Branch</th><th>|P|(${pUnit()})</th></tr></thead><tbody>`;
      data.branch_abs.forEach((p, i) => {
        const compId = busMap.branch[i];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        html += `<tr${attr}><td>${i}</td><td>${pFmt(p, 4)}</td></tr>`;
      });
      html += '</tbody></table>';
      brDiv.innerHTML = html;
    } else {
      brDiv.innerHTML = '<p class="empty-hint">无AC支路功率数据</p>';
    }

    // DC Branch flow table (prefer geo_dc_branches, fallback to dc_branch_flows)
    const dcBrSec = document.getElementById('pfDcBranchSection');
    const dcBrDiv = document.getElementById('pfDcBranchResults');
    const dcBrData = data.geo_dc_branches || data.dc_branch_flows || [];
    if (dcBrData.length > 0) {
      dcBrSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>From</th><th>To</th><th>Pf(${pUnit()})</th><th>Pt(${pUnit()})</th><th>Loss(${pUnit()})</th></tr></thead><tbody>`;
      dcBrData.forEach((br, i) => {
        const compId = busMap.dcBranch[i];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        const from = br.from ?? br.from_bus ?? '-';
        const to = br.to ?? br.to_bus ?? '-';
        const pf = br.pf_mw || 0;
        const pt = br.pt_mw || 0;
        const loss = pf + pt;
        html += `<tr${attr}><td>${i}</td><td>${from}</td><td>${to}</td><td>${pFmt(pf, 4)}</td><td>${pFmt(pt, 4)}</td><td>${pFmt(loss, 4)}</td></tr>`;
      });
      html += '</tbody></table>';
      dcBrDiv.innerHTML = html;
    } else {
      dcBrSec.style.display = 'none';
      dcBrDiv.innerHTML = '';
    }

    // 3W Transformer flow table
    const t3wSec = document.getElementById('pfTrafo3wSection');
    const t3wDiv = document.getElementById('pfTrafo3wResults');
    if (data.geo_trafo3w && data.geo_trafo3w.length > 0) {
      t3wSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>HV</th><th>MV</th><th>LV</th><th>P_hv(${pUnit()})</th><th>P_mv(${pUnit()})</th><th>P_lv(${pUnit()})</th><th>Loss(${pUnit()})</th><th>Loading%</th></tr></thead><tbody>`;
      data.geo_trafo3w.forEach((tf, i) => {
        const ldg = tf.loading_pct != null ? tf.loading_pct.toFixed(1) + '%' : '-';
        const ldgStyle = (tf.loading_pct || 0) > 100 ? ' style="color:#e06c75;font-weight:bold"' : '';
        html += `<tr><td>${tf.index ?? i}</td><td>${tf.hv_bus}</td><td>${tf.mv_bus}</td><td>${tf.lv_bus}</td>`;
        html += `<td>${pFmt(tf.p_hv_mw || 0, 3)}</td><td>${pFmt(tf.p_mv_mw || 0, 3)}</td><td>${pFmt(tf.p_lv_mw || 0, 3)}</td>`;
        html += `<td>${pFmt(tf.loss_mw || 0, 3)}</td><td${ldgStyle}>${ldg}</td></tr>`;
      });
      html += '</tbody></table>';
      t3wDiv.innerHTML = html;
    } else {
      t3wSec.style.display = 'none';
      t3wDiv.innerHTML = '';
    }

    // DCDC converter transfers table
    const dcdcSec = document.getElementById('pfDcdcSection');
    const dcdcDiv = document.getElementById('pfDcdcResults');
    if (data.dcdc_transfers && data.dcdc_transfers.length > 0) {
      dcdcSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>Bus In</th><th>Bus Out</th><th>P_in(${pUnit()})</th><th>P_out(${pUnit()})</th><th>Loss(${pUnit()})</th></tr></thead><tbody>`;
      data.dcdc_transfers.forEach((d, i) => {
        html += `<tr><td>${d.index ?? i}</td><td>${d.bus_in}</td><td>${d.bus_out}</td>`;
        html += `<td>${pFmt(d.p_in_mw || 0, 3)}</td><td>${pFmt(d.p_out_mw || 0, 3)}</td><td>${pFmt(d.loss_mw || 0, 3)}</td></tr>`;
      });
      html += '</tbody></table>';
      dcdcDiv.innerHTML = html;
    } else {
      dcdcSec.style.display = 'none';
      dcdcDiv.innerHTML = '';
    }

    // VSC converter transfers table
    const vscSec = document.getElementById('pfVscSection');
    const vscDiv = document.getElementById('pfVscResults');
    if (data.vsc_transfers && data.vsc_transfers.length > 0) {
      vscSec.style.display = '';
      let html = `<table><thead><tr><th>#</th><th>AC Bus</th><th>DC Bus</th><th>Pac(${pUnit()})</th><th>Qac(${qUnit()})</th><th>Pdc(${pUnit()})</th><th>Loss(${pUnit()})</th></tr></thead><tbody>`;
      data.vsc_transfers.forEach((v, i) => {
        const compId = busMap.vsc[i];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        html += `<tr${attr}><td>${v.index ?? i}</td><td>${v.bus_ac}</td><td>${v.bus_dc}</td>
                 <td>${v.p_ac_mw != null ? pFmt(v.p_ac_mw, 3) : '0'}</td><td>${v.q_ac_mvar != null ? pFmt(v.q_ac_mvar, 3) : '0'}</td>
                 <td>${v.p_dc_mw != null ? pFmt(v.p_dc_mw, 3) : '0'}</td><td>${v.loss_mw != null ? pFmt(v.loss_mw, 3) : '0'}</td></tr>`;
      });
      html += '</tbody></table>';
      vscDiv.innerHTML = html;
    } else {
      vscSec.style.display = 'none';
      vscDiv.innerHTML = '';
    }

    // Clear SC and topo results
    document.getElementById('scResults').innerHTML = '';
    document.getElementById('topoResults').innerHTML = '';
  }

  function showShortCircuitResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('shortCircuit');

    const summary = document.getElementById('resultsSummary');
    summary.innerHTML = `
      <div class="result-item"><span class="result-label">故障类型</span>
        <span class="result-value">${data.fault_type || 'ThreePhase'}</span></div>
      <div class="result-item"><span class="result-label">计算母线数</span>
        <span class="result-value">${data.bus_results?.length || 0}</span></div>
    `;

    const scDiv = document.getElementById('scResults');
    if (data.bus_results && data.bus_results.length > 0) {
      const busMap = Canvas.getCompBusMap();
      let html = '<table><thead><tr><th>Bus</th><th>Sk(MVA)</th><th>Ik"(kA)</th></tr></thead><tbody>';
      data.bus_results.forEach(br => {
        const compId = busMap.ac[br.bus_id];
        const attr = compId !== undefined ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
        html += `<tr${attr}><td>${br.bus_id}</td><td>${br.sk_mva?.toFixed(2) || 'N/A'}</td>
                 <td>${br.ikpp_ka?.toFixed(4) || 'N/A'}</td></tr>`;
      });
      html += '</tbody></table>';
      scDiv.innerHTML = html;
    }
  }

  // Render a single-bus (validated) fault from /api/session/sc_detailed.
  // data = { fault_type, results:[ { fault_bus_id, solved, bus_results:[...] } ] }
  function showShortCircuitDetailedResults(data, faultBus) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('shortCircuit');

    const fmt = (x, d = 3) =>
      (x === undefined || x === null || isNaN(x)) ? 'N/A' : Number(x).toFixed(d);

    const results = data.results || [];
    const r = results.find(x => x.fault_bus_id === faultBus) || results[0];
    const summary = document.getElementById('resultsSummary');
    const scDiv = document.getElementById('scResults');

    if (!r) {
      summary.innerHTML =
        '<div class="result-item"><span class="result-label">短路</span>' +
        '<span class="result-value result-failed">无结果</span></div>';
      scDiv.innerHTML = '';
      return;
    }

    const busRows = r.bus_results || [];
    const fb = busRows.find(b => b.bus_id === r.fault_bus_id) || {};

    summary.innerHTML = `
      <div class="result-item"><span class="result-label">故障类型</span>
        <span class="result-value">${data.fault_type || 'ThreePhase'}</span></div>
      <div class="result-item"><span class="result-label">故障母线</span>
        <span class="result-value">Bus ${r.fault_bus_id}</span></div>
      <div class="result-item"><span class="result-label">Ik" 初始 (kA)</span>
        <span class="result-value">${fmt(fb.ikss_ka, 3)}</span></div>
      <div class="result-item"><span class="result-label">ip 峰值 (kA)</span>
        <span class="result-value">${fmt(fb.ip_ka, 3)}</span></div>
      <div class="result-item"><span class="result-label">ib 开断 (kA)</span>
        <span class="result-value">${fmt(fb.ib_ka, 3)}</span></div>
      <div class="result-item"><span class="result-label">ik 稳态 (kA)</span>
        <span class="result-value">${fmt(fb.ik_ka, 3)}</span></div>
      <div class="result-item"><span class="result-label">Ith 热效 (kA)</span>
        <span class="result-value">${fmt(fb.ith_ka, 3)}</span></div>
    `;

    const busMap = Canvas.getCompBusMap();
    const contribs = [
      ['发电机', fb.ikss_gen_contrib_ka],
      ['电动机', fb.ikss_motor_contrib_ka],
      ['外部电网', fb.ikss_extgrid_contrib_ka],
      ['换流器', fb.ikss_converter_contrib_ka],
      ['静止发电机', fb.ikss_sgen_contrib_ka],
      ['负荷', fb.ikss_load_contrib_ka],
    ].filter(([, v]) => v !== undefined && v !== null);

    let html = '';
    if (contribs.length) {
      html += '<h4 style="margin:8px 0 4px">故障源贡献 (kA)</h4>';
      html += '<table><thead><tr><th>来源</th><th>Ik" (kA)</th></tr></thead><tbody>';
      contribs.forEach(([lbl, v]) => {
        html += `<tr><td>${lbl}</td><td>${fmt(v, 4)}</td></tr>`;
      });
      html += '</tbody></table>';
    }

    html += '<h4 style="margin:12px 0 4px">故障期间各母线剩余电压 (p.u.)</h4>';
    html += '<table><thead><tr><th>Bus</th><th>V<sub>remain</sub> (p.u.)</th></tr></thead><tbody>';
    busRows.forEach(b => {
      const compId = busMap.ac[b.bus_id];
      const clickable = compId !== undefined
        ? ` data-comp-id="${compId}" onclick="Canvas.panToComponent(${compId})"` : '';
      const isFault = b.bus_id === r.fault_bus_id;
      const style = isFault ? ' style="font-weight:700;background:var(--primary-soft)"' : '';
      html += `<tr${clickable}${style}><td>${b.bus_id}${isFault ? ' (故障点)' : ''}</td>` +
              `<td>${fmt(b.v_remaining_pu, 4)}</td></tr>`;
    });
    html += '</tbody></table>';
    scDiv.innerHTML = html;
  }

  function showTopologyResults(data) {
    document.getElementById('resultsEmpty').style.display = 'none';
    document.getElementById('resultsContent').style.display = 'block';
    setActiveResultGroup('topology');

    const summary = document.getElementById('resultsSummary');
    const totalSwOn  = (data.sw_on_per_step  || []).reduce((a,b) => a+b, 0);
    const totalSwOff = (data.sw_off_per_step || []).reduce((a,b) => a+b, 0);
    const totalCurt  = (data.curt_per_step   || []).reduce((a,b) => a+b, 0);

    // Loss comparison display
    const baseLoss = data.base_loss_mw ?? 0;
    const reconLoss = data.reconfig_loss_mw ?? 0;
    const lossRed = data.loss_reduction_mw ?? 0;
    const lossRedPct = data.loss_reduction_pct ?? 0;
    const lossCls = lossRed > 0.001 ? 'result-converged' : (lossRed < -0.001 ? 'result-failed' : '');

    summary.innerHTML = `
      <div class="result-item"><span class="result-label">可行</span>
        <span class="result-value ${data.feasible !== false ? 'result-converged' : 'result-failed'}">
          ${data.feasible !== false ? '✓ 是' : '✗ 否'}</span></div>
      <div class="result-item"><span class="result-label">求解器</span>
        <span class="result-value">${data.solver_name || 'N/A'}</span></div>
      <div class="result-item"><span class="result-label">MILP目标值</span>
        <span class="result-value">${data.milp_objective?.toFixed(4) || 'N/A'}</span></div>
      <div class="result-item"><span class="result-label">重构前损耗(MW)</span>
        <span class="result-value">${data.base_pf_converged ? baseLoss.toFixed(4) : '潮流未收敛'}</span></div>
      <div class="result-item"><span class="result-label">重构后损耗(MW)</span>
        <span class="result-value">${data.reconfig_pf_converged ? reconLoss.toFixed(4) : '潮流未收敛'}</span></div>
      <div class="result-item"><span class="result-label">损耗降低</span>
        <span class="result-value ${lossCls}">
          ${lossRed.toFixed(4)} MW (${lossRedPct.toFixed(1)}%)</span></div>
      <div class="result-item"><span class="result-label">时步数</span>
        <span class="result-value">${data.num_steps || 'N/A'}</span></div>
      <div class="result-item"><span class="result-label">开关操作</span>
        <span class="result-value">闭合 ${totalSwOn} / 断开 ${totalSwOff}</span></div>
      <div class="result-item"><span class="result-label">弃风弃光(MW)</span>
        <span class="result-value">${totalCurt.toFixed(2)}</span></div>
    `;

    const topoDiv = document.getElementById('topoResults');
    let html = '';

    // --- Visualize topology reconfiguration results on main diagram ---
    if (typeof Canvas !== 'undefined' && Canvas.showTopologyReconfigResults) {
      Canvas.showTopologyReconfigResults(data);
    }
    const busMap = Canvas.getCompBusMap();
    const makeBrLinks = (ids) => ids.map(id => {
      const compId = busMap.branch ? busMap.branch[id] : undefined;
      return compId !== undefined
        ? `<span class="clickable-branch" onclick="Canvas.panToComponent(${compId})">${id}</span>`
        : `${id}`;
    });

    // Topology verification section
    html += `<h4 style="margin:8px 0 6px;">拓扑验证</h4>`;
    html += `<table><thead><tr><th></th><th>重构前</th><th>重构后</th></tr></thead><tbody>`;
    html += `<tr><td>辐射状</td>
      <td class="${data.base_is_radial ? 'grade-green' : 'grade-red'}">${data.base_is_radial ? '✓ 是' : '✗ 否'}</td>
      <td class="${data.reconfig_is_radial ? 'grade-green' : 'grade-red'}">${data.reconfig_is_radial ? '✓ 是' : '✗ 否'}</td></tr>`;
    html += `<tr><td>连通</td>
      <td class="${data.base_is_connected ? 'grade-green' : 'grade-red'}">${data.base_is_connected ? '✓ 是' : '✗ 否'}</td>
      <td class="${data.reconfig_is_connected ? 'grade-green' : 'grade-red'}">${data.reconfig_is_connected ? '✓ 是' : '✗ 否'}</td></tr>`;
    html += `<tr><td>孤岛数</td><td>${data.base_islands ?? '—'}</td><td>${data.reconfig_islands ?? '—'}</td></tr>`;
    html += `<tr><td>闭合支路数</td><td>—</td><td>${data.reconfig_closed_count ?? '—'} / ${data.num_buses ? data.num_buses - 1 : '—'} (n-1)</td></tr>`;
    html += `<tr><td>潮流收敛</td>
      <td>${data.base_pf_converged ? '✓ 收敛' : '✗'}</td>
      <td>${data.reconfig_pf_converged ? '✓ 收敛' : '✗'}</td></tr>`;
    html += `<tr><td>有功损耗(MW)</td>
      <td>${data.base_pf_converged ? baseLoss.toFixed(4) : '—'}</td>
      <td>${data.reconfig_pf_converged ? reconLoss.toFixed(4) : '—'}</td></tr>`;
    html += `<tr><td><strong>损耗变化</strong></td><td colspan="2" class="${lossCls}"><strong>`;
    if (data.base_pf_converged && data.reconfig_pf_converged) {
      html += lossRed > 0 ? `↓ 降低 ${lossRed.toFixed(4)} MW (${lossRedPct.toFixed(1)}%)` :
              lossRed < 0 ? `↑ 增加 ${(-lossRed).toFixed(4)} MW (${(-lossRedPct).toFixed(1)}%)` :
              '无变化';
    } else { html += '无法比较'; }
    html += `</strong></td></tr></tbody></table>`;

    if (data.open_branch_ids && data.open_branch_ids.length > 0) {
      html += `<p><strong>断开支路 (最终):</strong> [${makeBrLinks(data.open_branch_ids).join(', ')}]</p>`;
    }
    if (data.closed_branch_ids && data.closed_branch_ids.length > 0) {
      html += `<p><strong>闭合支路 (最终):</strong> [${makeBrLinks(data.closed_branch_ids).join(', ')}]</p>`;
    }

    // Detailed branch status table — show which lines are open/closed with from/to bus
    if (data.branch_details && data.branch_details.length > 0) {
      const openBranches = data.branch_details.filter(b => !b.closed);
      const closedBranches = data.branch_details.filter(b => b.closed);
      html += `<h4 style="margin:12px 0 6px;">重构后支路状态明细</h4>`;
      // Open (disconnected) branches table
      if (openBranches.length > 0) {
        html += `<p><strong>断开支路 (${openBranches.length}条):</strong></p>`;
        html += `<table class="result-table"><thead><tr><th>支路ID</th><th>起始母线</th><th>终止母线</th><th>状态</th></tr></thead><tbody>`;
        for (const b of openBranches) {
          const compId = busMap.branch ? busMap.branch[b.id] : undefined;
          const idCell = compId !== undefined
            ? `<span class="clickable-branch" onclick="Canvas.panToComponent(${compId})">${b.id}</span>`
            : `${b.id}`;
          html += `<tr><td>${idCell}</td><td>${b.from_bus}</td><td>${b.to_bus}</td><td style="color:#e74c3c;font-weight:bold;">断开</td></tr>`;
        }
        html += `</tbody></table>`;
      }
      // Closed (connected) branches table
      if (closedBranches.length > 0) {
        html += `<details style="margin-top:8px;"><summary><strong>闭合支路 (${closedBranches.length}条) ▸ 点击展开</strong></summary>`;
        html += `<table class="result-table"><thead><tr><th>支路ID</th><th>起始母线</th><th>终止母线</th><th>状态</th></tr></thead><tbody>`;
        for (const b of closedBranches) {
          const compId = busMap.branch ? busMap.branch[b.id] : undefined;
          const idCell = compId !== undefined
            ? `<span class="clickable-branch" onclick="Canvas.panToComponent(${compId})">${b.id}</span>`
            : `${b.id}`;
          html += `<tr><td>${idCell}</td><td>${b.from_bus}</td><td>${b.to_bus}</td><td style="color:#27ae60;">闭合</td></tr>`;
        }
        html += `</tbody></table></details>`;
      }
    }
    if (data.verification_pf) {
      const pf = data.verification_pf;
      html += `<p><strong>验证潮流:</strong> ${pf.converged ? '收敛' : '未收敛'},
               迭代${pf.iterations}次, 残差=${Number(pf.residual || 0).toExponential(4)}</p>`;
    }
    // Per-step topology changes table
    if (data.steps_topology && data.steps_topology.length > 0) {
      html += `<h4 style="margin:12px 0 6px;">各时步拓扑变化</h4>`;
      html += `<table class="result-table"><thead><tr><th>时步</th><th>断开支路数</th><th>闭合支路数</th><th>闭合操作</th><th>断开操作</th></tr></thead><tbody>`;
      const swOn  = data.sw_on_per_step  || [];
      const swOff = data.sw_off_per_step || [];
      for (let t = 0; t < data.steps_topology.length; t++) {
        const st = data.steps_topology[t];
        html += `<tr><td>${t}</td><td>${st.open.length}</td><td>${st.closed.length}</td><td>${swOn[t]||0}</td><td>${swOff[t]||0}</td></tr>`;
      }
      html += `</tbody></table>`;
    }
    // Per-step curtailment and switching chart placeholders
    if (data.sw_on_per_step && data.num_steps > 1) {
      html += `<div id="rcChartSwitch" style="width:100%;height:220px;margin-top:12px;"></div>`;
      html += `<div id="rcChartCurt" style="width:100%;height:220px;margin-top:8px;"></div>`;
    }
    if (data.num_ess > 0) {
      html += `<div id="rcChartESS" style="width:100%;height:220px;margin-top:8px;"></div>`;
    }
    topoDiv.innerHTML = html || '<p class="empty-hint">无拓扑变化数据</p>';

    // Render Plotly charts if available
    if (typeof Plotly !== 'undefined' && data.sw_on_per_step && data.num_steps > 1) {
      const T = data.num_steps;
      const hrs = Array.from({length:T}, (_,i) => i);
      const swDiv = document.getElementById('rcChartSwitch');
      if (swDiv) {
        Plotly.newPlot(swDiv, [
          {x:hrs, y:data.sw_on_per_step,  type:'bar', name:'闭合操作', marker:{color:'#27ae60'}},
          {x:hrs, y:data.sw_off_per_step, type:'bar', name:'断开操作', marker:{color:'#e74c3c'}},
        ], {title:'各时步开关操作', barmode:'group', xaxis:{title:'时步'}, yaxis:{title:'次数'},
            margin:{l:50,r:15,t:35,b:40}, font:{size:11}}, {responsive:true});
      }
      const curtDiv = document.getElementById('rcChartCurt');
      if (curtDiv && data.curt_per_step) {
        Plotly.newPlot(curtDiv, [
          {x:hrs, y:data.curt_per_step, mode:'lines+markers', name:'弃风弃光(MW)', line:{color:'#b5651d',width:2}},
        ], {title:'各时步弃电量', xaxis:{title:'时步'}, yaxis:{title:'MW'},
            margin:{l:50,r:15,t:35,b:40}, font:{size:11}}, {responsive:true});
      }
      const essDiv = document.getElementById('rcChartESS');
      if (essDiv && data.ess_soc_by_step && data.num_ess > 0) {
        const traces = Array.from({length:data.num_ess}, (_, si) => ({
          x: hrs,
          y: data.ess_soc_by_step.map(row => (row && row[si]) || 0),
          mode: 'lines+markers',
          name: `储能 ${si+1} SOC`
        }));
        Plotly.newPlot(essDiv, traces, {title:'储能SOC变化', xaxis:{title:'时步'},
            yaxis:{title:'SOC', range:[0,1]}, margin:{l:50,r:15,t:35,b:40}, font:{size:11}},
            {responsive:true});
      }
    }
  }

  // ========== Topology Tables ==========
  function onTopologyChanged() {
    _canvasDirty = true;
    updateTopologyTables();
  }

  function updateTopologyTables() {
    const sys = Canvas.buildSystemJson();
    const m = Canvas.getCompBusMap();

    // Helper: populate a table and toggle its section visibility
    function fillTable(bodyId, sectionId, items, mapObj, rowFn) {
      const body = document.querySelector(bodyId + ' tbody');
      if (!body) return;
      body.innerHTML = '';
      if (sectionId) {
        const sec = document.getElementById(sectionId);
        if (sec) sec.style.display = items.length ? '' : 'none';
      }
      items.forEach(item => {
        const tr = document.createElement('tr');
        const compId = mapObj ? mapObj[item.index] : undefined;
        if (compId !== undefined) {
          tr.dataset.compId = compId;
          tr.addEventListener('click', () => Canvas.panToComponent(compId));
        }
        tr.innerHTML = rowFn(item);
        body.appendChild(tr);
      });
    }

    // Aggregate load per bus from load components
    const busLoad = {};
    sys.ac.loads.forEach(ld => {
      if (!busLoad[ld.bus]) busLoad[ld.bus] = { p: 0, q: 0 };
      busLoad[ld.bus].p += (ld.p_mw || 0) * (ld.scaling || 1);
      busLoad[ld.bus].q += (ld.q_mvar || 0) * (ld.scaling || 1);
    });

    // Bus table (always shown)
    fillTable('#busTableInner', null, sys.ac.buses, m.ac, bus => {
      const ld = busLoad[bus.index] || { p: bus.pd_mw, q: bus.qd_mvar };
      return `<td>${bus.index}</td><td>${bus.bus_type}</td>
        <td>${bus.vm_pu.toFixed(4)}</td><td>${bus.va_deg.toFixed(2)}</td>
        <td>${ld.p.toFixed(2)}</td><td>${ld.q.toFixed(2)}</td>
        <td>${bus.base_kv}</td>`;
    });

    // Branch table (always shown, includes _from_branch transformers)
    fillTable('#branchTableInner', null, sys.ac.branches, m.branch, br =>
      `<td>${br.from_bus}</td><td>${br.to_bus}</td>
        <td>${br.r_pu.toFixed(6)}</td><td>${br.x_pu.toFixed(6)}</td>
        <td>${br.b_pu.toFixed(6)}</td><td>${br.rate_a_mva}</td>
        <td>${(br.tap || 1).toFixed(4)}</td><td>${(br.shift_deg || 0).toFixed(2)}</td>`);

    // Generator table (always shown)
    fillTable('#genTableInner', null, sys.ac.generators, m.gen, gen =>
      `<td>${gen.bus}</td><td>${gen.pg_mw}</td>
        <td>${gen.qg_mvar}</td><td>${gen.vg_pu}</td>
        <td>${gen.pmax_mw}</td><td>${gen.pmin_mw}</td>
        <td>${gen.is_slack ? '✓' : ''}</td>`);

    // Load table
    fillTable('#loadTableInner', 'loadSection', sys.ac.loads, m.load, ld =>
      `<td>${ld.bus}</td><td>${ld.p_mw}</td><td>${ld.q_mvar}</td><td>${ld.scaling}</td>`);

    // Transformer table (non-from_branch only)
    fillTable('#trafoTableInner', 'trafoSection', sys.ac.transformers_2w, m.trafo, t =>
      `<td>${t.hv_bus}</td><td>${t.lv_bus}</td><td>${t.sn_mva}</td>
        <td>${t.vk_percent}</td><td>${t.shift_deg}</td>`);

    // External Grid
    fillTable('#extGridTableInner', 'extGridSection', sys.ac.external_grids, m.extGrid, eg =>
      `<td>${eg.bus}</td><td>${eg.vm_pu}</td><td>${eg.va_deg}</td><td>${eg.s_sc_max_mva}</td>`);

    // Storage
    fillTable('#storageTableInner', 'storageSection', sys.ac.storage, m.storage, s =>
      `<td>${s.bus}</td><td>${s.p_rated_mw}</td><td>${s.e_rated_mwh}</td><td>${s.soc_init}</td>`);

    // PV System
    fillTable('#pvTableInner', 'pvSection', sys.ac.pv_systems, m.pv, pv =>
      `<td>${pv.bus}</td><td>${pv.p_mw}</td><td>${pv.q_mvar}</td><td>${pv.sn_mva}</td>`);

    // Renewable Gen
    fillTable('#renGenTableInner', 'renGenSection', sys.ac.renewable_gens, m.renGen, rg =>
      `<td>${rg.bus}</td><td>${rg.type}</td><td>${rg.p_mw}</td><td>${rg.p_rated_mw}</td><td>${rg.capacity_factor}</td>`);

    // Static Generator
    fillTable('#sgenTableInner', 'sgenSection', sys.ac.static_generators, m.sgen, sg =>
      `<td>${sg.bus}</td><td>${sg.p_mw}</td><td>${sg.q_mvar}</td><td>${sg.sgen_type}</td>`);

    // Shunt
    fillTable('#shuntTableInner', 'shuntSection', sys.ac.shunts, m.shunt, sh =>
      `<td>${sh.bus}</td><td>${sh.gs_mw}</td><td>${sh.bs_mvar}</td><td>${sh.switchable ? '✓' : ''}</td>`);

    // Switch
    fillTable('#switchTableInner', 'switchSection', sys.ac.switches, m.sw, sw =>
      `<td>${sw.bus_from}</td><td>${sw.bus_to}</td><td>${sw.closed ? '✓' : '✗'}</td>`);

    // Circuit Breaker
    fillTable('#cbTableInner', 'cbSection', sys.ac.circuit_breakers, m.cb, cb =>
      `<td>${cb.bus_from}</td><td>${cb.bus_to}</td><td>${cb.closed ? '✓' : '✗'}</td><td>${cb.rated_current_ka}</td>`);

    // Motor
    fillTable('#motorTableInner', 'motorSection', sys.ac.motors, m.motor, mt =>
      `<td>${mt.bus}</td><td>${mt.sn_mva}</td><td>${mt.cos_phi}</td><td>${mt.efficiency}</td>`);

    // Transformer 3W
    fillTable('#trafo3wTableInner', 'trafo3wSection', sys.ac.transformers_3w, m.trafo3w, t =>
      `<td>${t.hv_bus}</td><td>${t.mv_bus}</td><td>${t.lv_bus}</td><td>${t.sn_hv_mva}</td>`);

    // Flexible Load
    fillTable('#flexLoadTableInner', 'flexLoadSection', sys.ac.flexible_loads, m.flexLoad, fl =>
      `<td>${fl.bus}</td><td>${fl.p_mw}</td><td>${fl.flex_up_mw}</td><td>${fl.flex_down_mw}</td>`);

    // Asymmetric Load
    fillTable('#asymLoadTableInner', 'asymLoadSection', sys.ac.asymmetric_loads, m.asymLoad, al =>
      `<td>${al.bus}</td><td>${al.pa_mw}</td><td>${al.pb_mw}</td><td>${al.pc_mw}</td>`);

    // DC Bus
    fillTable('#dcBusTableInner', 'dcBusSection', sys.dc.buses, m.dc, db =>
      `<td>${db.index}</td><td>${db.bus_type || ''}</td><td>${(db.vm_pu || 1).toFixed(4)}</td><td>${db.base_kv || ''}</td>`);

    // DC Branch
    fillTable('#dcBranchTableInner', 'dcBranchSection', sys.dc.branches, m.dcBranch, db =>
      `<td>${db.from_bus}</td><td>${db.to_bus}</td><td>${db.r_pu}</td><td>${db.rate_a_mva}</td>`);

    // DC Load
    fillTable('#dcLoadTableInner', 'dcLoadSection', sys.dc.loads, m.dcLoad, dl =>
      `<td>${dl.bus}</td><td>${dl.p_mw}</td><td>${dl.scaling}</td>`);

    // VSC Converter
    fillTable('#vscTableInner', 'vscSection', sys.vsc_converters, m.vsc, v =>
      `<td>${v.bus_ac}</td><td>${v.bus_dc}</td><td>${v.p_set_mw}</td><td>${v.control_mode}</td>`);

    // Charger
    fillTable('#chargerTableInner', 'chargerSection', sys.ac.chargers, m.charger, ch =>
      `<td>${ch.charger_type}</td><td>${ch.p_rated_kw}</td><td>${ch.eta}</td><td>${ch.v2g_capable ? '✓' : ''}</td>`);

    // Charging Station
    fillTable('#csTableInner', 'csSection', sys.ac.charging_stations, m.chargingStation, cs =>
      `<td>${cs.bus}</td><td>${cs.n_fast}</td><td>${cs.n_slow}</td><td>${cs.max_power_kw}</td>`);

    // Mobile Storage
    fillTable('#msTableInner', 'msSection', sys.mobile_storage, m.mobileStorage, ms =>
      `<td>${ms.bus}</td><td>${ms.p_rated_mw}</td><td>${ms.e_rated_mwh}</td><td>${ms.soc_init}</td>`);

    // DC-DC Converter
    fillTable('#dcdcTableInner', 'dcdcSection', sys.dcdc_converters, m.dcdcConverter, dc =>
      `<td>${dc.bus_in}</td><td>${dc.bus_out}</td><td>${dc.control_mode}</td><td>${dc.p_ref_mw}</td>`);

    // Energy Router
    fillTable('#erTableInner', 'erSection', sys.energy_routers, m.energyRouter, er =>
      `<td>${er.router_type}</td><td>${er.num_ports}</td><td>${er.p_rated_mw}</td>`);

    // VPP
    fillTable('#vppTableInner', 'vppSection', sys.vpps, m.vpp, v =>
      `<td>${v.pcc_bus ?? v.aggregation_bus ?? 0}</td><td>${v.n_pv_systems}</td><td>${v.n_wind_turbines}</td><td>${v.n_battery_systems}</td><td>${v.p_output_mw}</td>`);

    // Microgrid
    fillTable('#mgTableInner', 'mgSection', sys.microgrids, m.microgrid, mg =>
      `<td>${mg.pcc_bus}</td><td>${mg.operating_mode}</td><td>${mg.p_exchange_mw}</td><td>${mg.capacity_mw}</td>`);
  }

  // ========== Property Editor ==========
  function onSelectionChanged(compId) {
    const propEmpty = document.getElementById('propEmpty');
    const propEditor = document.getElementById('propEditor');

    if (compId === null) {
      propEmpty.style.display = 'block';
      propEditor.style.display = 'none';
      return;
    }

    const comp = Canvas.getComponent(compId);
    if (!comp) return;

    propEmpty.style.display = 'none';
    propEditor.style.display = 'block';

    document.getElementById('propTitle').textContent =
      `${COMP.defaults[comp.type]?.name || comp.type} — ID:${comp.id}`;

    const fieldsDiv = document.getElementById('propFields');
    fieldsDiv.innerHTML = '';

    const defaults = COMP.defaults[comp.type] || {};
    Object.keys(defaults).forEach(key => {
      const val = comp.params[key] !== undefined ? comp.params[key] : defaults[key];
      const label = COMP.fieldLabels[key] || key;

      const div = document.createElement('div');
      div.className = 'prop-field';

      const lbl = document.createElement('label');
      lbl.textContent = label;
      div.appendChild(lbl);

      if (typeof val === 'boolean') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        sel.innerHTML = `<option value="true" ${val ? 'selected' : ''}>是</option>
                         <option value="false" ${!val ? 'selected' : ''}>否</option>`;
        div.appendChild(sel);
      } else if (key === 'bus_type') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        ['PQ', 'PV', 'SLACK', 'ISOLATED'].forEach(t => {
          sel.innerHTML += `<option value="${t}" ${val === t ? 'selected' : ''}>${t}</option>`;
        });
        div.appendChild(sel);
      } else if (key === 'fuel_type') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        ['Thermal', 'Nuclear', 'Hydro', 'Gas', 'Oil', 'Coal', 'Biomass', 'Other'].forEach(t => {
          sel.innerHTML += `<option value="${t}" ${val === t ? 'selected' : ''}>${t}</option>`;
        });
        div.appendChild(sel);
      } else if (key === 'model') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        ['ConstantPower', 'ConstantImpedance', 'ZIP'].forEach(t => {
          sel.innerHTML += `<option value="${t}" ${val === t ? 'selected' : ''}>${t}</option>`;
        });
        div.appendChild(sel);
      } else if (key === 'control_mode') {
        const sel = document.createElement('select');
        sel.dataset.field = key;
        let modeOptions;
        if (comp.type === 'pv_system') {
          modeOptions = [
            {v:'MPPT',   l:'MPPT (最大功率追踪)'},
            {v:'PQ',     l:'PQ (恒功率)'},
            {v:'VQ',     l:'VQ (电压无功)'},
            {v:'Curtailed', l:'Curtailed (自适应削减)'},
          ];
        } else if (comp.type === 'dcdc_converter') {
          modeOptions = [
            {v:'Voltage', l:'Voltage'},
            {v:'Current', l:'Current'},
            {v:'Power',   l:'Power'},
            {v:'MPPT',    l:'MPPT'},
          ];
        } else {
          modeOptions = [
            {v:'PQ_MODE', l:'PQ_MODE'},
            {v:'VDC_Q',   l:'VDC_Q'},
            {v:'VDC_P',   l:'VDC_P'},
            {v:'AC_PQ',   l:'AC_PQ'},
            {v:'DROOP',   l:'DROOP'},
          ];
        }
        modeOptions.forEach(o => {
          sel.innerHTML += `<option value="${o.v}" ${val === o.v ? 'selected' : ''}>${o.l}</option>`;
        });
        div.appendChild(sel);
      } else {
        const inp = document.createElement('input');
        inp.dataset.field = key;
        inp.type = typeof val === 'number' ? 'number' : 'text';
        inp.step = 'any';
        inp.value = val;
        div.appendChild(inp);
      }

      fieldsDiv.appendChild(div);
    });
  }

  function applyProperties() {
    const compId = Canvas.state.selectedId;
    if (compId === null) return;
    const comp = Canvas.getComponent(compId);
    if (!comp) return;

    // Track which fields changed for per-km ↔ per-unit sync
    const oldParams = Object.assign({}, comp.params);
    const fields = document.querySelectorAll('#propFields [data-field]');
    const changedKeys = new Set();
    fields.forEach(el => {
      const key = el.dataset.field;
      let val = el.value;
      // Type conversion
      if (val === 'true') val = true;
      else if (val === 'false') val = false;
      else if (el.type === 'number' && val !== '') val = parseFloat(val);
      if (comp.params[key] !== val) changedKeys.add(key);
      comp.params[key] = val;
    });

    // Bidirectional sync for ac_branch: per-km ↔ per-unit parameters
    if (comp.type === 'ac_branch') {
      syncBranchImpedance(comp, changedKeys, oldParams);
    }

    // Re-render component with new params
    Canvas.rerenderComponent(comp);
    // Only mark dirty if something actually changed
    if (changedKeys.size > 0) {
      _canvasDirty = true;
    }
    updateTopologyTables();
    // Refresh property panel to show synced values
    if (changedKeys.size > 0 && comp.type === 'ac_branch') {
      onSelectionChanged(compId);
    }
    if (changedKeys.size > 0) {
      log(`已更新 ${comp.params.name} 的属性 (${[...changedKeys].join(', ')})`, 'info');
    } else {
      log(`${comp.params.name} 属性未变化`, 'info');
    }
  }

  /**
   * Synchronize per-km and per-unit impedance parameters for ac_branch.
   *
   * Conversion formulas (using from_bus base_kv and system base_mva):
   *   z_base = base_kv² / base_mva
   *   r_pu = r_ohm_per_km * length_km / z_base
   *   x_pu = x_ohm_per_km * length_km / z_base
   *   b_pu = b_us_per_km  * length_km * z_base * 1e-6
   *
   * Direction of sync depends on which parameter was modified:
   *   - Changed per-km or length → recalculate per-unit
   *   - Changed per-unit → recalculate per-km
   */
  function syncBranchImpedance(comp, changedKeys, oldParams) {
    const p = comp.params;
    const perKmChanged = changedKeys.has('r_ohm_per_km') || changedKeys.has('x_ohm_per_km')
                       || changedKeys.has('b_us_per_km') || changedKeys.has('c_nf_per_km');
    const puChanged = changedKeys.has('r_pu') || changedKeys.has('x_pu') || changedKeys.has('b_pu');
    const lengthChanged = changedKeys.has('length_km');

    if (!perKmChanged && !puChanged && !lengthChanged) return;

    // Find from_bus base_kv
    const fromBus = p.from_bus || 0;
    let base_kv = 110;  // default
    const base_mva = 100;  // system default
    if (fromBus > 0) {
      const busComp = Canvas.state.components.find(
        c => c.type === 'ac_bus' && c.params.index === fromBus
      );
      if (busComp && busComp.params.base_kv > 0) base_kv = busComp.params.base_kv;
    }
    const z_base = (base_kv * base_kv) / base_mva;
    const length = p.length_km || 0;

    if (perKmChanged || lengthChanged) {
      // Per-km → per-unit
      if (length > 0 && z_base > 0) {
        // If c_nf_per_km changed, derive b_us_per_km from it
        if (changedKeys.has('c_nf_per_km') && p.c_nf_per_km > 0) {
          const freq = 50;  // Hz
          p.b_us_per_km = 2 * Math.PI * freq * p.c_nf_per_km * 1e-3;  // nF→μS
        }
        if (p.r_ohm_per_km >= 0) p.r_pu = p.r_ohm_per_km * length / z_base;
        if (p.x_ohm_per_km >= 0) p.x_pu = p.x_ohm_per_km * length / z_base;
        if (p.b_us_per_km >= 0)  p.b_pu = p.b_us_per_km * length * z_base * 1e-6;
      }
    } else if (puChanged) {
      // Per-unit → per-km
      if (length > 0 && z_base > 0) {
        if (changedKeys.has('r_pu')) p.r_ohm_per_km = p.r_pu * z_base / length;
        if (changedKeys.has('x_pu')) p.x_ohm_per_km = p.x_pu * z_base / length;
        if (changedKeys.has('b_pu')) p.b_us_per_km  = p.b_pu / (length * z_base * 1e-6);
      }
    }
  }

  // ========== Tab Switching ==========
  function switchTab(tabName) {
    document.querySelectorAll('.panel-tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
    document.querySelector(`.panel-tab[data-tab="${tabName}"]`)?.classList.add('active');
    document.getElementById('tab' + tabName.charAt(0).toUpperCase() + tabName.slice(1))?.classList.add('active');
  }

  // ========== Component Library ==========
  function initComponentLibrary() {
    Object.entries(COMP.categories).forEach(([catId, items]) => {
      const grid = document.getElementById(catId);
      if (!grid) return;
      items.forEach(item => {
        const div = document.createElement('div');
        div.className = 'lib-item';
        div.dataset.compType = item.type;

        // Create a small SVG icon
        const iconSvg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        iconSvg.setAttribute('viewBox', '-30 -30 60 60');
        iconSvg.setAttribute('width', '36');
        iconSvg.setAttribute('height', '36');
        const symbolFn = COMP.symbols[item.type];
        if (symbolFn) {
          const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
          g.setAttribute('transform', 'scale(0.6)');
          g.innerHTML = symbolFn(COMP.defaults[item.type] || {});
          // Remove labels from icon
          g.querySelectorAll('.comp-label, .comp-value').forEach(t => t.remove());
          iconSvg.appendChild(g);
        }
        div.appendChild(iconSvg);

        const span = document.createElement('span');
        span.textContent = item.label;
        div.appendChild(span);

        // Click to enter place mode
        div.addEventListener('click', () => {
          document.querySelectorAll('.lib-item').forEach(i => i.classList.remove('selected'));
          div.classList.add('selected');
          Canvas.setMode('place', item.type);
          log(`选择放置: ${item.label}`, 'info');
        });

        grid.appendChild(div);
      });
    });
  }

  // ========== Toolbar UI (3-tier layout) ==========
  function showCaseLoadModal() {
    const m = document.getElementById('caseLoadModal');
    if (m) m.style.display = 'flex';
  }
  function hideCaseLoadModal() {
    const m = document.getElementById('caseLoadModal');
    if (m) m.style.display = 'none';
  }
  function setActiveCanvasTool(toolName) {
    // Tools that live in bar 1 and act as exclusive canvas modes.
    const ids = ['btnSelect', 'btnConnect', 'btnBoxSelect'];
    ids.forEach(id => {
      const el = document.getElementById(id);
      if (el) el.classList.toggle('active', id === toolName);
    });
  }
  function setActiveModule(moduleName) {
    document.querySelectorAll('.module-btn').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.module === moduleName);
    });
    renderSubToolbar(moduleName);
    // Also switch the right-panel result view to the matching module's
    // results group (if any), so the 结果 tab only shows the active
    // module's outputs / placeholder. CSS in style.css drives visibility.
    setActiveResultGroup(moduleName);
  }
  function renderSubToolbar(moduleName) {
    const bar = document.getElementById('subToolbar');
    if (!bar) return;
    let anyVisible = false;
    bar.querySelectorAll('.sub-section').forEach(sec => {
      const match = sec.dataset.sub === moduleName;
      sec.hidden = !match;
      if (match) anyVisible = true;
    });
    bar.classList.toggle('hidden', !anyVisible);
  }

  // ========== Init ==========
  function init() {
    // Initialize canvas
    Canvas.init();

    // Initialize component library
    initComponentLibrary();

    // Load case list
    loadCaseList();
    loadMatpowerFileList();

    // ---- Toolbar Events ----
    // Bar 1: 加载算例 -> open case-load modal
    document.getElementById('btnLoadCase').addEventListener('click', showCaseLoadModal);

    // Case-load modal: three import paths
    document.getElementById('btnImportLibraryCase')?.addEventListener('click', () => {
      const caseName = document.getElementById('caseSelect').value;
      if (!caseName) {
        log('请先在下拉框中选择一个内置算例', 'warn');
        return;
      }
      hideCaseLoadModal();
      loadBuiltinCase(caseName);
    });
    document.getElementById('btnImportMatpowerCase')?.addEventListener('click', () => {
      const filename = document.getElementById('matpowerSelect').value;
      if (!filename) {
        log('请先在下拉框中选择一个 MATPOWER 文件', 'warn');
        return;
      }
      hideCaseLoadModal();
      loadMatpowerCase(filename);
    });
    document.getElementById('btnImportJsonCase')?.addEventListener('click', () => {
      document.getElementById('fileImportJson').click();
    });
    // ETAP workbook (.xlsx) import from the case-load modal.
    document.getElementById('btnImportEtapXlsxCase')?.addEventListener('click', () => {
      document.getElementById('fileImportEtapXlsx')?.click();
    });
    document.getElementById('fileImportEtapXlsx')?.addEventListener('change', (e) => {
      const f = e.target.files[0];
      e.target.value = '';
      if (f) { hideCaseLoadModal(); loadEtapXlsx(f); }
    });
    // Native ETAP project (.xml) import from the case-load modal.
    document.getElementById('btnImportEtapXmlCase')?.addEventListener('click', () => {
      document.getElementById('fileImportEtapXml')?.click();
    });
    document.getElementById('fileImportEtapXml')?.addEventListener('change', (e) => {
      const f = e.target.files[0];
      e.target.value = '';
      if (f) { hideCaseLoadModal(); loadEtapXml(f); }
    });
    document.getElementById('btnCaseModalClose')?.addEventListener('click', hideCaseLoadModal);
    document.querySelector('#caseLoadModal .modal-backdrop')?.addEventListener('click', hideCaseLoadModal);

    document.getElementById('btnNewSystem').addEventListener('click', createNewSystem);
    // Legacy buttons may have been replaced; guard with optional chaining.
    document.getElementById('btnLoadMatpower')?.addEventListener('click', () => {
      const filename = document.getElementById('matpowerSelect').value;
      if (filename) loadMatpowerCase(filename);
    });
    document.getElementById('btnExportJson').addEventListener('click', exportJson);
    document.getElementById('btnExportEtap')?.addEventListener('click', exportEtap);
    document.getElementById('btnImportJson').addEventListener('click', () => {
      document.getElementById('fileImportJson').click();
    });
    document.getElementById('fileImportJson').addEventListener('change', (e) => {
      if (e.target.files[0]) importJson(e.target.files[0]);
      e.target.value = '';
    });

    // Calculation buttons (Bar 3 "运行..." buttons reuse original IDs where possible)
    document.getElementById('btnPowerFlow').addEventListener('click', runPowerFlow);
    // Legacy header buttons removed — guard:
    document.getElementById('btnShortCircuit')?.addEventListener('click', showScDialog);
    document.getElementById('btnTopology')?.addEventListener('click', runTopologyReconfig);
    document.getElementById('btnTimeSeries')?.addEventListener('click', showTspfDialog);

    // Bar 3: shortCircuit sub-toolbar
    document.getElementById('btnFaultLocation')?.addEventListener('click', showScDialog);
    document.getElementById('faultTypeSelect')?.addEventListener('change', (e) => {
      const f = document.getElementById('scFaultType');
      if (f) f.value = e.target.value;
    });
    document.getElementById('voltageCorrectionFactor')?.addEventListener('change', (e) => {
      const f = document.getElementById('scCFactor');
      if (f) f.value = e.target.value;
    });
    document.getElementById('btnRunShortCircuit')?.addEventListener('click', () => {
      // Mirror sub-toolbar values into legacy dialog inputs, then run.
      const ft = document.getElementById('faultTypeSelect')?.value;
      const cf = document.getElementById('voltageCorrectionFactor')?.value;
      if (ft) document.getElementById('scFaultType').value = ft;
      if (cf) document.getElementById('scCFactor').value = cf;
      runShortCircuit();
    });

    // Bar 3: topology
    document.getElementById('btnRunTopology')?.addEventListener('click', runTopologyReconfig);

    // Bar 3: hosting capacity
    document.getElementById('btnRunBearingCap')?.addEventListener('click', showBcDialog);
    // Legacy bearing-capacity launcher (header button removed)
    document.getElementById('btnBearingCap')?.addEventListener('click', showBcDialog);

    // Bar 3: time-series — run directly with inline params (skip UC / OPF).
    document.getElementById('btnRunTimeSeriesPF')?.addEventListener('click', runTimeSeriesPF);
    document.getElementById('btnRunCarbonFlow')?.addEventListener('click', () => {
      // TODO: hook up backend carbon-flow endpoint when available.
      log('碳流计算：尚未对接后端接口（TODO）', 'warn');
    });

    // Bar 3: PF result export — write _lastPfData to a JSON file.
    document.getElementById('btnExportPfResults')?.addEventListener('click', () => {
      if (!_lastPfData) {
        log('暂无潮流计算结果可导出，请先运行潮流计算', 'warn');
        return;
      }
      downloadJsonFile(`pf_results_${tsTagForFilename()}.json`, _lastPfData);
    });

    // Bar 3: time-series result export. Bundles the full per-step bus
    // voltages (magnitude + angle) and per-branch flows (Pf, Pt, Qf, Qt)
    // alongside the original TSPF response so downstream tools have
    // everything in one file.
    document.getElementById('btnExportTspfResults')?.addEventListener('click', () => {
      if (!_lastTspfData) {
        log('暂无时序潮流结果可导出，请先运行时序潮流计算', 'warn');
        return;
      }
      const d = _lastTspfData;
      const bundle = {
        meta: {
          exported_at: new Date().toISOString(),
          num_steps: d.num_steps,
          num_converged: d.num_converged,
          total_generation_cost: d.total_generation_cost,
        },
        bus_voltages: {
          bus_labels: d.bus_labels || [],
          vm_matrix:  d.vm_matrix  || [],     // [bus][t]  pu
          va_matrix:  d.va_matrix  || [],     // [bus][t]  rad
        },
        branch_flows: {
          branch_labels:  d.branch_labels  || [],
          branch_from_bus: d.branch_from_bus || [],
          branch_to_bus:   d.branch_to_bus   || [],
          pf_mw:    d.branch_pf_mw  || [],    // [branch][t]  MW (from-bus injection)
          pt_mw:    d.branch_pt_mw  || [],    // [branch][t]  MW (to-bus injection)
          qf_mvar:  d.branch_qf_mvar || [],   // [branch][t]  MVAr
          qt_mvar:  d.branch_qt_mvar || [],
        },
        per_step_summary: {
          vm_mean:   d.vm_mean   || [],
          vm_min:    d.vm_min    || [],
          vm_max:    d.vm_max    || [],
          losses_mw: d.losses_mw || [],
          total_load_mw: d.total_load || [],
        },
        raw: d,   // keep entire response for completeness
      };
      downloadJsonFile(`tspf_results_${tsTagForFilename()}.json`, bundle);
      const nbus = (d.bus_labels || []).length;
      const nbr  = (d.branch_labels || []).length;
      log(`时序潮流结果已导出：${d.num_steps} 步 × ${nbus} 母线电压/相角 × ${nbr} 分支 Pf/Pt/Qf/Qt`, 'success');
    });

    // Bar 3: PF — Green-certificate data import (placeholder; backend TODO).
    document.getElementById('btnImportGreenCert')?.addEventListener('click', () => {
      document.getElementById('fileImportGreenCert')?.click();
    });
    document.getElementById('fileImportGreenCert')?.addEventListener('change', (e) => {
      const f = e.target.files[0];
      if (f) log(`已选择绿证数据文件: ${f.name}（TODO: 后端导入接口待对接）`, 'info');
      e.target.value = '';
    });

    // Bar 3: Reliability — run (NSQ/SEQ/FMEA/F&D) + export. Backend live.
    async function runReliability() {
      setStatus('可靠性分析中...', 'busy');
      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
      const method = (document.getElementById('relMethod')?.value) || 'nsq';
      const maxIter = parseInt(document.getElementById('relMaxIter')?.value) || 2000;
      const epMap = { nsq: 'run_reliability_nsq', seq: 'run_reliability_seq',
                      fmea: 'run_reliability_fmea', fd: 'run_reliability_fd' };
      const opts = {};
      if (method === 'nsq') opts.max_iterations = maxIter;
      else if (method === 'seq') { opts.max_years = Math.max(50, Math.round(maxIter / 10)); opts.hours_per_year = 8736; }
      else if (method === 'fmea') { opts.load_scale_factor = 1.0; opts.apply_comprehensive_data = true; }
      const data = await apiPost('/api/session/' + epMap[method], opts);
      if (data && !data.error) {
        _lastReliabilityData = Object.assign({ _method: method }, data);
        showReliabilityResults(data, method);
        switchTab('results');
        setStatus('可靠性分析完成');
      } else {
        setStatus('计算失败', 'error');
      }
    }

    function showReliabilityResults(data, method) {
      document.getElementById('resultsEmpty').style.display = 'none';
      document.getElementById('resultsContent').style.display = 'block';
      setActiveResultGroup('reliability');
      const nf = (v, d = 2) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(d) : '—';
      const methodLabel = { nsq: '非序贯蒙特卡洛', seq: '序贯蒙特卡洛', fmea: 'FMEA (N-1)', fd: '频率-持续时间' }[method] || method;
      let kpis = [];
      if (method === 'fd') {
        kpis = [
          ['LOLP (失负荷概率)', nf(data.lolp, 6)],
          ['LOLE (h/yr)', nf(data.lole_fd, 2)],
          ['LOLF (occ/yr)', nf(data.lolf_fd, 2)],
          ['LOLD (h/occ)', nf(data.lold, 2)],
        ];
      } else {
        kpis = [
          ['EENS (MWh/yr)', nf(data.eens_mwh_yr, 1)],
          ['EDNS (MW)', nf(data.edns_mw, 3)],
          ['LOLE (h/yr)', nf(data.lole_hr_yr, 2)],
          ['LOLF (occ/yr)', nf(data.lolf_occ_yr, 2)],
          ['PLC (%)', nf((data.plc ?? 0) * 100, 3)],
        ];
        if (method === 'fmea') {
          kpis.push(['SAIFI', nf(data.saifi, 4)], ['SAIDI', nf(data.saidi, 4)], ['ASAI', nf(data.asai, 6)]);
          if (data.n_contingencies != null) kpis.push(['故障枚举数', `${data.n_contingencies} (${data.n_with_loss ?? '?'} 含失负荷)`]);
        } else {
          kpis.push(['收敛', data.converged ? '✓ 是' : '✗ 否'], ['迭代/年数', data.iterations_used ?? '—'], ['最终 CoV', nf(data.final_cov, 4)]);
        }
      }
      let html = `<div style="margin-bottom:8px;"><b>方法：</b>${methodLabel}</div>`;
      html += '<table><thead><tr><th>指标</th><th>数值</th></tr></thead><tbody>';
      for (const [k, v] of kpis) html += `<tr><td>${k}</td><td class="result-value">${v}</td></tr>`;
      html += '</tbody></table>';

      // Convergence chart (NSQ/SEQ)
      if ((method === 'nsq' || method === 'seq') && Array.isArray(data.eens_history) && data.eens_history.length) {
        html += '<h4 style="margin:10px 0 4px;">EENS 收敛过程</h4><div id="relConvChart" style="height:240px;"></div>';
      }
      // Critical components (NSQ/SEQ)
      if (Array.isArray(data.critical_components) && data.critical_components.length) {
        html += '<h4 style="margin:10px 0 4px;">薄弱元件 (Top)</h4><table><thead><tr><th>#</th><th>类型</th><th>索引</th><th>重要度</th></tr></thead><tbody>';
        data.critical_components.slice(0, 15).forEach((c, i) => {
          html += `<tr><td>${i + 1}</td><td>${c.is_generator ? '发电机' : '支路'}</td><td>${c.index ?? '—'}</td><td>${nf(c.importance, 4)}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      // FMEA top contingencies
      if (method === 'fmea' && Array.isArray(data.contingencies) && data.contingencies.length) {
        html += '<h4 style="margin:10px 0 4px;">关键故障 (按 EENS 贡献)</h4><table><thead><tr><th>元件</th><th>类型</th><th>EENS贡献(MWh/yr)</th><th>切负荷(MW)</th></tr></thead><tbody>';
        data.contingencies.slice(0, 15).forEach(c => {
          html += `<tr><td>${c.component_name ?? '—'}</td><td>${c.component_type ?? '—'}</td><td>${nf(c.eens_contribution, 2)}</td><td>${nf(c.shed_mw, 2)}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      document.getElementById('reliabilityResults').innerHTML = html;

      if ((method === 'nsq' || method === 'seq') && typeof Plotly !== 'undefined' && Array.isArray(data.eens_history) && data.eens_history.length) {
        const x = data.eens_history.map((_, i) => i + 1);
        Plotly.newPlot('relConvChart', [{ x, y: data.eens_history, mode: 'lines', name: 'EENS', line: { color: '#61afef' } }],
          { margin: { l: 55, r: 10, t: 10, b: 35 }, xaxis: { title: '样本批次' }, yaxis: { title: 'EENS (MWh/yr)' },
            paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', font: { color: '#dcdfe4' } }, { responsive: true });
      }
    }

    document.getElementById('btnRunReliability')?.addEventListener('click', runReliability);
    document.getElementById('btnExportReliabilityResults')?.addEventListener('click', () => {
      if (!_lastReliabilityData) {
        log('暂无可靠性分析结果可导出，请先运行可靠性分析', 'warn');
        return;
      }
      downloadJsonFile(`reliability_results_${tsTagForFilename()}.json`, _lastReliabilityData);
    });

    // Bar 3: Resilience — collect inline parameters into a payload (placeholder).
    function collectResilienceParams() {
      const num = (id, dflt) => {
        const v = parseFloat(document.getElementById(id)?.value);
        return Number.isFinite(v) ? v : dflt;
      };
      const locStr = (document.getElementById('resFaultLocations')?.value || '').trim();
      const locations = locStr ? locStr.split(/[,，\s]+/).map(s => parseInt(s, 10)).filter(n => Number.isFinite(n)) : [];
      return {
        fault_count:     num('resFaultCount', 1),
        fault_locations: locations,
        fault_start_hr:  num('resFaultStartHour', 0),
        fault_duration_hr: num('resFaultDuration', 4),
        repair_duration_hr: num('resRepairDuration', 6),
        mobile_storage_speed_kmh: num('resMobileSpeed', 40),
      };
    }
    document.getElementById('btnGenExtremeScenario')?.addEventListener('click', () => {
      const params = collectResilienceParams();
      log(`极端场景生成：当前参数 ${JSON.stringify(params)}（TODO: 后端接口待对接）`, 'info');
    });
    async function runResilience() {
      setStatus('弹性评估中...', 'busy');
      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
      const p = collectResilienceParams();
      const params = {
        default_fault_count: p.fault_count,
        fault_branch_ids: p.fault_locations,
        auto_fault_start_hr: p.fault_start_hr,
        fault_duration_hr: p.fault_duration_hr,
        repair_time_hr: p.repair_duration_hr,
        mess_travel_speed_kmph: p.mobile_storage_speed_kmh,
        horizon_hours: Math.max(24, Math.ceil(p.fault_start_hr + p.fault_duration_hr + p.repair_duration_hr + 4)),
        time_step_hr: 1.0,
        allow_reconfiguration: true,
        allow_mess_dispatch: true,
        run_power_flow: false,
      };
      const data = await apiPost('/api/session/run_distribution_resilience', params);
      if (data && !data.error) {
        _lastResilienceData = data;
        showResilienceResults(data);
        switchTab('results');
        setStatus('弹性评估完成');
      } else {
        setStatus('计算失败', 'error');
      }
    }

    function showResilienceResults(data) {
      document.getElementById('resultsEmpty').style.display = 'none';
      document.getElementById('resultsContent').style.display = 'block';
      setActiveResultGroup('resilience');
      const nf = (v, d = 2) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(d) : '—';
      const kpis = [
        ['可行', data.feasible === false ? '✗ 否' : '✓ 是'],
        ['状态', data.status ?? '—'],
        ['弹性指数', nf(data.resilience_index, 4)],
        ['总需求 (MWh)', nf(data.total_demand_mwh, 2)],
        ['总切负荷 (MWh)', nf(data.total_shed_mwh, 2)],
        ['加权未供 (MWh)', nf(data.weighted_unserved_mwh, 2)],
        ['峰值切负荷 (MW)', nf(data.peak_shed_mw, 2)],
        ['最终恢复率 (%)', nf((data.final_restoration_ratio ?? 0) * 100, 2)],
        ['平均恢复率 (%)', nf((data.avg_restoration_ratio ?? 0) * 100, 2)],
        ['移储供能 (MWh)', nf(data.mess_energy_delivered_mwh, 2)],
        ['移储行程 (km)', nf(data.mess_travel_distance_km, 1)],
        ['开关操作次数', data.total_switch_actions ?? '—'],
        ['修复故障数', data.total_repaired_faults ?? '—'],
      ];
      let html = '<table><thead><tr><th>指标</th><th>数值</th></tr></thead><tbody>';
      for (const [k, v] of kpis) html += `<tr><td>${k}</td><td class="result-value">${v}</td></tr>`;
      html += '</tbody></table>';

      if (Array.isArray(data.fault_sequence) && data.fault_sequence.length) {
        html += '<h4 style="margin:10px 0 4px;">故障序列</h4><table><thead><tr><th>#</th><th>支路</th><th>开始(h)</th><th>修复(h)</th></tr></thead><tbody>';
        data.fault_sequence.slice(0, 20).forEach((f, i) => {
          html += `<tr><td>${i + 1}</td><td>${f.branch_id ?? f.branch ?? '—'}</td><td>${nf(f.start_hr, 1)}</td><td>${nf(f.repair_hr ?? f.repair_time_hr, 1)}</td></tr>`;
        });
        html += '</tbody></table>';
      }
      if (Array.isArray(data.hours) && data.hours.length) {
        html += '<h4 style="margin:10px 0 4px;">恢复过程 (供电 vs 切负荷)</h4><div id="resTimeChart" style="height:260px;"></div>';
      }
      document.getElementById('resilienceResults').innerHTML = html;

      if (typeof Plotly !== 'undefined' && Array.isArray(data.hours) && data.hours.length) {
        const traces = [];
        if (Array.isArray(data.served_mw)) traces.push({ x: data.hours, y: data.served_mw, mode: 'lines', name: '供电 (MW)', line: { color: '#98c379' } });
        if (Array.isArray(data.shed_mw)) traces.push({ x: data.hours, y: data.shed_mw, mode: 'lines', name: '切负荷 (MW)', line: { color: '#e06c75' } });
        if (Array.isArray(data.demand_mw)) traces.push({ x: data.hours, y: data.demand_mw, mode: 'lines', name: '需求 (MW)', line: { color: '#61afef', dash: 'dot' } });
        Plotly.newPlot('resTimeChart', traces,
          { margin: { l: 55, r: 10, t: 10, b: 35 }, xaxis: { title: '时间 (h)' }, yaxis: { title: 'MW' },
            legend: { orientation: 'h', y: -0.2 }, paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', font: { color: '#dcdfe4' } },
          { responsive: true });
      }
    }

    document.getElementById('btnRunResilience')?.addEventListener('click', runResilience);
    document.getElementById('btnExportResilienceResults')?.addEventListener('click', () => {
      if (!_lastResilienceData) {
        log('暂无弹性评估结果可导出，请先运行弹性评估', 'warn');
        return;
      }
      downloadJsonFile(`resilience_results_${tsTagForFilename()}.json`, _lastResilienceData);
    });

    // ───────── Time-series profile imports (JSON real, Excel placeholder) ─────────
    // Cache parsed profiles client-side so each new import can be merged into a
    // single set_ts_config POST. Backend overwrites `ts_data` on every call.
    const _tsProfileCache = {
      irradiance: null,    // {id, name, values}
      price: null,         // {id, name, values}
      load_profiles: null, // [{load_index, bus, name, values}, ...]
      num_steps: null,
      step_duration_hr: 1.0,
    };
    const TS_PROFILE_ID = { irradiance: 2, price: 50, load_base: 100 };

    async function pushTsConfigToServer(label) {
      const c = _tsProfileCache;
      if (!c.num_steps) {
        log(`${label}：未识别到 num_steps，无法上传`, 'warn');
        return;
      }
      const profiles = [];
      if (c.irradiance) profiles.push({
        id: TS_PROFILE_ID.irradiance, name: c.irradiance.name || 'irradiance',
        values: c.irradiance.values,
      });
      if (c.price) profiles.push({
        id: TS_PROFILE_ID.price, name: c.price.name || 'price',
        values: c.price.values,
      });
      const load_profile_map = [];
      if (c.load_profiles) {
        c.load_profiles.forEach((lp, i) => {
          const pid = TS_PROFILE_ID.load_base + i;
          // The backend formula is: P_t = ld.p_mw * ld.scaling * profile[t].
          // JSON stores absolute MW (lp.p_mw_values), so we convert to a
          // dimensionless scaling = MW[t] / p_mw_nominal here. If
          // p_mw_nominal is missing or zero, fall back to the (legacy)
          // dimensionless `values` field as-is.
          const mw = Array.isArray(lp.p_mw_values) ? lp.p_mw_values
                   : (Array.isArray(lp.values) ? lp.values : null);
          const nom = Number(lp.p_mw_nominal);
          let scaled;
          if (mw && Number.isFinite(nom) && Math.abs(nom) > 1e-12 &&
              Array.isArray(lp.p_mw_values)) {
            scaled = mw.map(v => v / nom);
          } else if (mw) {
            scaled = mw;  // already dimensionless (legacy)
          } else {
            return;
          }
          profiles.push({
            id: pid, name: lp.name || `load_${lp.bus ?? i}`,
            values: scaled,
          });
          if (Number.isInteger(lp.load_index)) {
            load_profile_map.push({ load_index: lp.load_index, profile_id: pid });
          } else if (Number.isInteger(lp.bus)) {
            load_profile_map.push({ bus: lp.bus, profile_id: pid });
          }
        });
      }
      const body = {
        num_steps: c.num_steps,
        step_duration_hr: c.step_duration_hr || 1.0,
        profiles,
        load_profile_map,
        // bind solar PV to irradiance profile when present
        assign_all_pv_to: c.irradiance ? TS_PROFILE_ID.irradiance : -1,
      };
      try {
        const r = await fetch('/api/session/set_ts_config', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        });
        const j = await r.json();
        if (!r.ok) throw new Error(j.error || 'set_ts_config failed');
        const simHr = document.getElementById('simulationHours');
        if (simHr) simHr.value = String(c.num_steps);
        log(`${label}：已上传配置 (num_steps=${j.num_steps}, profiles=${j.num_profiles}, loads_mapped=${j.num_loads_mapped ?? 0})`, 'success');
      } catch (err) {
        log(`${label}：上传失败 ${err.message || err}`, 'error');
      }
    }

    function readFileAsText(file) {
      return new Promise((resolve, reject) => {
        const fr = new FileReader();
        fr.onload = () => resolve(fr.result);
        fr.onerror = () => reject(fr.error);
        fr.readAsText(file, 'utf-8');
      });
    }

    async function handleTsImport(kind, label, file) {
      const lower = file.name.toLowerCase();
      if (!(lower.endsWith('.json'))) {
        log(`${label}：Excel/CSV 导入暂未实现 (TODO)，请使用 JSON 文件 (${file.name})`, 'warn');
        return;
      }
      let j;
      try {
        const text = await readFileAsText(file);
        j = JSON.parse(text);
      } catch (err) {
        log(`${label}：JSON 解析失败 ${err.message || err}`, 'error');
        return;
      }
      const ns = j.num_steps;
      if (!Number.isInteger(ns) || ns <= 0) {
        log(`${label}：JSON 缺少有效的 num_steps`, 'error');
        return;
      }
      if (_tsProfileCache.num_steps && _tsProfileCache.num_steps !== ns) {
        log(`${label}：num_steps=${ns} 与已有配置 num_steps=${_tsProfileCache.num_steps} 不一致，已重置缓存`, 'warn');
        _tsProfileCache.irradiance = _tsProfileCache.price = _tsProfileCache.load_profiles = null;
      }
      _tsProfileCache.num_steps = ns;
      _tsProfileCache.step_duration_hr = j.step_duration_hr || 1.0;

      if (kind === 'irradiance') {
        if (!Array.isArray(j.values) || j.values.length !== ns) {
          log(`${label}：values 数组缺失或长度不等于 ${ns}`, 'error');
          return;
        }
        _tsProfileCache.irradiance = { name: j.name || 'irradiance', values: j.values };
        log(`${label}：已加载 ${file.name} (num_steps=${ns})`, 'info');
      } else if (kind === 'price') {
        if (!Array.isArray(j.values) || j.values.length !== ns) {
          log(`${label}：values 数组缺失或长度不等于 ${ns}`, 'error');
          return;
        }
        _tsProfileCache.price = { name: j.name || 'price', values: j.values };
        log(`${label}：已加载 ${file.name} (num_steps=${ns})`, 'info');
      } else if (kind === 'load_profiles') {
        const lps = j.load_profiles;
        if (!Array.isArray(lps) || lps.length === 0) {
          log(`${label}：load_profiles 缺失或为空`, 'error');
          return;
        }
        let bad = 0;
        for (const lp of lps) {
          const arr = Array.isArray(lp.p_mw_values) ? lp.p_mw_values
                    : (Array.isArray(lp.values) ? lp.values : null);
          if (!arr || arr.length !== ns) bad++;
        }
        if (bad > 0) {
          log(`${label}：${bad} 条负荷曲线长度不等于 ${ns}`, 'error');
          return;
        }
        _tsProfileCache.load_profiles = lps;
        log(`${label}：已加载 ${file.name} (${lps.length} 条负荷曲线, num_steps=${ns})`, 'info');
      }
      await pushTsConfigToServer(label);
    }

    const tsImportPairs = [
      ['btnImportIrradiance',  'fileImportIrradiance',  '辐照强度', 'irradiance'],
      ['btnImportPrice',       'fileImportPrice',       '电价',     'price'],
      ['btnImportLoadProfile', 'fileImportLoadProfile', '负荷波动', 'load_profiles'],
    ];
    tsImportPairs.forEach(([btnId, fileId, label, kind]) => {
      document.getElementById(btnId)?.addEventListener('click', () => {
        document.getElementById(fileId)?.click();
      });
      document.getElementById(fileId)?.addEventListener('change', async (e) => {
        const f = e.target.files[0];
        e.target.value = '';
        if (!f) return;
        await handleTsImport(kind, label, f);
      });
    });

    // Bar 2: module switching
    document.querySelectorAll('.module-btn').forEach(btn => {
      btn.addEventListener('click', () => setActiveModule(btn.dataset.module));
    });

    // Display unit selector — re-render cached PF results on change
    const pfDisplayUnit = document.getElementById('pfDisplayUnit');
    if (pfDisplayUnit) {
      pfDisplayUnit.addEventListener('change', () => {
        if (_lastPfData) {
          showPowerFlowResultsTables(_lastPfData);
        }
        if (Canvas.refreshVisualization) {
          Canvas.refreshVisualization();
        }
      });
    }

    // Bearing Capacity Dialog
    document.getElementById('btnBearingCap')?.addEventListener('click', showBcDialog);
    document.getElementById('btnBcRun').addEventListener('click', runBearingCapacity);
    document.getElementById('btnBcCancel').addEventListener('click', hideBcDialog);

    // SC Dialog
    document.getElementById('btnScRun').addEventListener('click', runShortCircuit);
    document.getElementById('btnScCancel').addEventListener('click', hideScDialog);

    // TSPF Dialog
    document.getElementById('btnTspfRun').addEventListener('click', runTimeSeriesPF);
    document.getElementById('btnTspfCancel').addEventListener('click', hideTspfDialog);
    document.getElementById('tspfProfileSource').addEventListener('change', (e) => {
      document.getElementById('tspfImportRow').style.display =
        e.target.value === 'import' ? 'block' : 'none';
    });

    // Canvas toolbar
    document.getElementById('btnSelect').addEventListener('click', () => {
      Canvas.setMode('select');
      setActiveCanvasTool('btnSelect');
      document.querySelectorAll('.lib-item').forEach(i => i.classList.remove('selected'));
    });
    document.getElementById('btnConnect').addEventListener('click', () => {
      Canvas.setMode('connect');
      setActiveCanvasTool('btnConnect');
    });
    document.getElementById('btnBoxSelect')?.addEventListener('click', () => {
      // Box-select operates in 'select' mode (drag empty area) — keep canvas mode 'select'
      // but mark the box-select button as the active UI tool.
      Canvas.setMode('select');
      setActiveCanvasTool('btnBoxSelect');
    });
    document.getElementById('btnDelete').addEventListener('click', () => {
      if (Canvas.state.selectedIds.size > 1) {
        Canvas.removeSelected();
      } else if (Canvas.state.selectedId !== null) {
        Canvas.removeComponent(Canvas.state.selectedId);
        onTopologyChanged();
      }
    });
    document.getElementById('btnDeleteSelected').addEventListener('click', () => {
      Canvas.removeSelected();
    });
    document.getElementById('btnZoomIn').addEventListener('click', () => Canvas.zoomIn());
    document.getElementById('btnZoomOut').addEventListener('click', () => Canvas.zoomOut());
    document.getElementById('btnZoomFit').addEventListener('click', () => Canvas.zoomFit());
    document.getElementById('btnAutoLayout').addEventListener('click', () => Canvas.autoLayout());
    document.getElementById('btnRotateCW').addEventListener('click', () => Canvas.rotateSelected(90));
    document.getElementById('btnRotateCCW').addEventListener('click', () => Canvas.rotateSelected(-90));

    // Visualization mode toggle
    document.getElementById('vizMode')?.addEventListener('change', (e) => {
      Canvas.setVisualizationMode(e.target.value);
    });

    // Property apply
    document.getElementById('btnApplyProp').addEventListener('click', applyProperties);

    // Tab switching
    document.querySelectorAll('.panel-tab').forEach(tab => {
      tab.addEventListener('click', () => switchTab(tab.dataset.tab));
    });

    // Console clear
    document.getElementById('btnClearConsole').addEventListener('click', () => {
      document.getElementById('consoleLog').innerHTML = '';
    });

    // Right panel resizer
    {
      const resizer = document.getElementById('rightPanelResizer');
      const panel = document.getElementById('rightPanel');
      let startX, startW;
      const resizePlotlyCharts = () => {
        panel.querySelectorAll('.js-plotly-plot').forEach(el => {
          try { Plotly.Plots.resize(el); } catch (_) {}
        });
      };
      resizer.addEventListener('mousedown', (e) => {
        startX = e.clientX;
        startW = panel.offsetWidth;
        resizer.classList.add('dragging');
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';
        let rafId = 0;
        const onMove = (ev) => {
          const newW = startW - (ev.clientX - startX);
          panel.style.width = Math.max(240, Math.min(window.innerWidth * 0.6, newW)) + 'px';
          // Throttle Plotly resizes to animation frames
          if (!rafId) rafId = requestAnimationFrame(() => { resizePlotlyCharts(); rafId = 0; });
        };
        const onUp = () => {
          resizer.classList.remove('dragging');
          document.body.style.cursor = '';
          document.body.style.userSelect = '';
          document.removeEventListener('mousemove', onMove);
          document.removeEventListener('mouseup', onUp);
          // Final resize after drag ends
          resizePlotlyCharts();
        };
        document.addEventListener('mousemove', onMove);
        document.addEventListener('mouseup', onUp);
      });
    }

    log('Hybrid AC/DC Power System Simulator 已启动', 'success');
    log('使用左侧元件库拖放元件到画布，或加载内置算例', 'info');

    // ---- Toolbar default state ----
    Canvas.setMode('select');
    setActiveCanvasTool('btnSelect');
    setActiveModule('powerFlow');  // default-activate Power Flow module
  }

  // ========== Public API ==========
  return {
    init,
    log,
    onSelectionChanged,
    onTopologyChanged,
    switchTab,
    setActiveModule,
    setActiveCanvasTool,
    showCaseLoadModal,
    hideCaseLoadModal,
  };
})();

// ========== Start ==========
document.addEventListener('DOMContentLoaded', App.init);
