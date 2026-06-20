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
  let _lastScenarioGenerationData = null;
  let _lastScenarioBaseSystemJson = null;
  let _importedGeneratedScenario = null;
  let _lastImportedGeneratedScenarioKey = '';
  let _generatedScenarioTimeSeriesActive = false;

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

    const faultBus = parseInt(document.getElementById('scFaultBus').value);
    const faultType = document.getElementById('scFaultType').value;
    const cFactor = parseFloat(document.getElementById('scCFactor').value);

    // Use the detail endpoint if a specific bus is provided, otherwise batch
    const data = await apiPost('/api/session/sc', {
      options: {
        fault_type: faultType,
        c_factor: cFactor,
        compute_all_buses: true,
      }
    });

    if (data) {
      log(`短路计算完成: ${data.bus_results?.length || 0} 个母线结果`, 'success');
      setStatus('短路完成');
      showShortCircuitResults(data);
      switchTab('results');
    } else {
      setStatus('计算失败', 'error');
    }
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

  // ===== Scenario generation: outer-scope helpers (import / apply scenario time-series) =====
  function isUsableGeneratedScenarioTimeSeries(ts) {
    if (!ts || !Array.isArray(ts.profiles) || ts.profiles.length === 0) return false;
    const numSteps = Number(ts.num_steps || ts.profiles[0]?.values?.length || 0);
    return numSteps > 0 && ts.profiles.some(p => Array.isArray(p?.values) && p.values.length > 0);
  }

  function getImportedGeneratedScenarioCase(targetFamily = null) {
    const caseJson = _importedGeneratedScenario?.case || null;
    if (!caseJson) return null;
    const family = caseJson?._generated_scenario?.family || _importedGeneratedScenario?.family || '';
    if (targetFamily && family && family !== 'unknown' && family !== targetFamily) return null;
    return caseJson;
  }

  function getImportedGeneratedScenarioTimeSeries(targetFamily = null) {
    const caseJson = getImportedGeneratedScenarioCase(targetFamily);
    const ts = caseJson?._time_series;
    return isUsableGeneratedScenarioTimeSeries(ts) ? ts : null;
  }

  function hasUsableGeneratedScenarioTimeSeries(targetFamily = null) {
    return !!getImportedGeneratedScenarioTimeSeries(targetFamily);
  }

  async function applyGeneratedScenarioTimeSeries(caseJson) {
    const ts = caseJson?._time_series;
    if (!ts || !Array.isArray(ts.profiles) || ts.profiles.length === 0) return false;
    if (_generatedScenarioTimeSeriesActive) return true;
    const body = {
      num_steps: ts.num_steps || ts.profiles[0]?.values?.length || 24,
      step_duration_hr: ts.step_duration_hr || 1.0,
      profiles: ts.profiles,
      load_profile_map: ts.binding?.load_profile_map || [],
      assign_all_loads_to: Number.isInteger(ts.binding?.assign_all_loads_to) ? ts.binding.assign_all_loads_to : -1,
      assign_all_pv_to: Number.isInteger(ts.binding?.assign_all_pv_to) ? ts.binding.assign_all_pv_to : -1,
    };
    const r = await fetch('/api/session/set_ts_config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || 'set_ts_config failed');
    _generatedScenarioTimeSeriesActive = true;
    const simHr = document.getElementById('simulationHours');
    if (simHr) simHr.value = String(j.num_steps || body.num_steps);
    log(`生成场景时序已恢复：${j.num_steps}步，profiles=${j.num_profiles}`, 'success');
    return true;
  }

  async function resetGeneratedScenarioTimeSeriesIfActive(numSteps = null) {
    if (!_generatedScenarioTimeSeriesActive) return false;
    const simHours = Number.isFinite(numSteps) && numSteps > 0
      ? numSteps
      : (parseInt(document.getElementById('simulationHours')?.value, 10) || 24);
    const r = await fetch('/api/session/set_ts_config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ num_steps: simHours, step_duration_hr: 1.0 }),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || 'set_ts_config reset failed');
    _generatedScenarioTimeSeriesActive = false;
    log(`已切回默认时序配置：${j.num_steps}步，profiles=${j.num_profiles}`, 'info');
    return true;
  }

  async function runTimeSeriesPF() {
    setStatus('时序潮流计算中...', 'busy');

    if (!await syncToBackend(true)) {
      setStatus('同步失败', 'error');
      return;
    }

    // If "使用场景时序" is enabled, bind imported regular-scenario profiles first.
    if (document.getElementById('regUseScenarioTimeSeries')?.checked && hasUsableGeneratedScenarioTimeSeries('regular')) {
      try { await applyGeneratedScenarioTimeSeries(getImportedGeneratedScenarioCase('regular')); }
      catch (e) { log(`应用生成场景时序失败：${e.message || e}`, 'warn'); }
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
    document.getElementById('btnGenExtremeScenario')?.addEventListener('click', async () => {
      const intensity = document.getElementById('resTyphoonIntensity')?.value || 'TY';
      setStatus('正在按台风强度生成台风故障序列...', 'busy');
      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
      const data = await apiPost('/api/session/generate_typhoon_faults', {
        intensity_category: intensity,
        horizon_hours: 48,
        time_step_hr: 1.0,
        catalog_samples_per_month: 100,
      });
      if (!data) { setStatus('台风故障生成失败', 'error'); return; }
      // SIM's resilience engine acts on AC branches; map AC faults into the
      // single-value resilience inputs (locations list + representative timing).
      const typed = Array.isArray(data.fault_locations_typed) ? data.fault_locations_typed : [];
      const acFromTyped = typed.filter(r => String(r.branch_type || 'AC').toUpperCase() === 'AC')
                               .map(r => r.branch_index ?? r.branch_id);
      const acLocations = acFromTyped.length ? acFromTyped
                          : (Array.isArray(data.fault_locations) ? data.fault_locations : []);
      const starts = Array.isArray(data.fault_start_hours) ? data.fault_start_hours.map(Number).filter(Number.isFinite) : [];
      const repairs = Array.isArray(data.repair_durations) ? data.repair_durations.map(Number).filter(Number.isFinite) : [];
      const setVal = (id, v) => { const el = document.getElementById(id); if (el) el.value = v; };
      setVal('resFaultLocations', acLocations.join(','));
      setVal('resFaultCount', String(acLocations.length));
      if (starts.length) setVal('resFaultStartHour', String(Math.min(...starts)));
      if (repairs.length) setVal('resRepairDuration', String(Math.max(...repairs)));
      const requestedLabel = data.requested_intensity_category_zh || data.requested_intensity_category || intensity;
      const selectedLabel = data.selected_intensity_category_zh || data.selected_intensity_category || requestedLabel;
      const vmax = Number(data.selected_track_max_vmax_ms);
      const vmaxText = Number.isFinite(vmax) ? `，轨迹最大风速 ${vmax.toFixed(2)} m/s` : '';
      const fallbackText = data.used_category_fallback ? `；请求等级 ${requestedLabel} 样本不足，已回退为 ${selectedLabel}` : '';
      log(`${selectedLabel}台风场景已生成：${acLocations.length} 个 AC 故障已填入故障位置${vmaxText}${fallbackText}。${data.status || ''}`, 'success');
      setStatus('台风故障序列已填入');
    });
    async function runResilience() {
      setStatus('弹性评估中...', 'busy');
      if (!await syncToBackend(true)) { setStatus('同步失败', 'error'); return; }
      // If "使用场景时序" is enabled, bind imported resilience-scenario 48h profiles first.
      if (document.getElementById('resUseScenarioTimeSeries')?.checked && hasUsableGeneratedScenarioTimeSeries('resilience')) {
        try { await applyGeneratedScenarioTimeSeries(getImportedGeneratedScenarioCase('resilience')); }
        catch (e) { log(`应用生成场景时序失败：${e.message || e}`, 'warn'); }
      }
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

    // ===== Scenario generation module (场景生成) — inner-scope functions + listeners =====
    function scenNum(id, dflt) {
      const value = Number(document.getElementById(id)?.value);
      return Number.isFinite(value) ? value : dflt;
    }

    function scenChecked(id, dflt = false) {
      const el = document.getElementById(id);
      return el ? el.checked === true : dflt;
    }

    function collectScenarioGenerationOptions() {
      const regularClusters = Math.max(1, Math.round(scenNum('scenRegularClusters', 8)));
      const reliabilityClusters = Math.max(1, Math.round(scenNum('scenReliabilityClusters', 4)));
      const resilienceClusters = Math.max(1, Math.round(scenNum('scenResilienceClusters', 5)));
      const intensityLevels = ['TD', 'TS', 'STS', 'TY', 'STY', 'SuperTY'];
      const sspLevels = ['ssp126', 'ssp245', 'ssp370', 'ssp585'];
      const years = [2050, 2080];
      return {
        regular: {
          enabled: scenChecked('scenRegularEnabled', true),
          candidate_count: regularClusters * 10 * sspLevels.length * years.length,
          cluster_count: regularClusters,
          num_steps: 8760,
          ssp: 'all',
          year: 0,
          ssp_levels: sspLevels,
          years,
        },
        reliability: {
          enabled: scenChecked('scenReliabilityEnabled', true),
          candidates_per_contingency: reliabilityClusters * 10,
          cluster_count_per_contingency: reliabilityClusters,
          num_steps: 1,
          include_ac_branches: true,
          include_dc_branches: true,
          include_ac_buses: false,
          include_dc_buses: false,
          include_vsc_converters: true,
          include_dcdc_converters: false,
          include_generators: true,
          include_loads: false,
          include_storage: true,
          include_vpps: false,
          include_microgrids: false,
          max_contingencies: 0,
        },
        resilience: {
          enabled: scenChecked('scenResilienceEnabled', true),
          intensity_levels: intensityLevels,
          candidates_per_intensity: resilienceClusters * 10,
          default_cluster_count: resilienceClusters,
          cluster_count_by_intensity: Object.fromEntries(intensityLevels.map(level => [level, resilienceClusters])),
          num_steps: 48,
        },
        perturbation: {
          seed: 1,
          load_sigma: 0.08,
          renewable_sigma: 0.12,
          load_min_multiplier: 0.75,
          load_max_multiplier: 1.25,
          renewable_min_multiplier: 0.0,
          renewable_max_multiplier: 1.20,
          enable_storage_soc_perturbation: true,
          storage_soc_sigma: 0.10,
          storage_soc_min_multiplier: 0.75,
          storage_soc_max_multiplier: 1.25,
          block_hours: 24,
          enable_load_perturbation: true,
          enable_renewable_perturbation: true,
        },
        typhoon_impact: {
          pv_transition_hours: 3,
          load_wind_start: 25.0,
          load_wind_full: 50.0,
          load_max_reduction: 0.4,
        },
        clustering: {
          method: document.getElementById('scenClusteringMethod')?.value || 'hybrid_kmedoids_tail_5pct',
          max_iterations: 50,
          robust_scale: true,
          continuous_weight: 1.0,
          outage_hamming_weight: 1.0,
          include_tail_anchors: true,
          compare_baseline: scenChecked('scenCompareBaseline', true),
          freeze_anchors: true,
          tail_fraction: 0.05,
          source_tail_quantile: 0.95,
          coupling_quantile: 0.90,
          boundary_fraction: 0.02,
          regime_weight: 1.0,
        },
      };
    }

    function firstProfileSummary(cluster, profileName) {
      const profiles = cluster?.representative?.time_series?.profiles || [];
      const profile = profiles.find(p => p.name === profileName);
      const values = Array.isArray(profile?.values) ? profile.values : [];
      if (!values.length) return '-';
      const sum = values.reduce((a, b) => a + Number(b || 0), 0);
      const max = values.reduce((a, b) => Math.max(a, Number(b || 0)), -Infinity);
      return `${sum.toFixed(2)} / 峰值 ${Number.isFinite(max) ? max.toFixed(2) : '-'}`;
    }

    function profileValuesFromCandidate(candidate, profileName) {
      const profiles = candidate?.time_series?.profiles || [];
      const profile = profiles.find(p => p.name === profileName);
      return Array.isArray(profile?.values) ? profile.values.map(v => Number(v || 0)) : [];
    }

    function numericFeature(cluster, key) {
      const value = cluster?.representative?.features?.[key];
      return Number.isFinite(Number(value)) ? Number(value) : 0;
    }

    function eventMaxWind(event) {
      const explicit = Number(event?.selected_track_max_vmax_ms);
      if (Number.isFinite(explicit) && explicit > 0) return explicit;
      const track = Array.isArray(event?.track) ? event.track : [];
      const start = track.length > 1 ? 1 : 0;
      let vmax = 0;
      for (let i = start; i < track.length; i += 1) {
        const v = Number(track[i]?.vmax_ms);
        if (Number.isFinite(v)) vmax = Math.max(vmax, v);
      }
      return vmax;
    }

    function parseRegularId(id) {
      const parts = String(id || '').split(':');
      return { ssp: parts[1] || 'unknown', year: Number(parts[2]) || 0 };
    }

    function clamp01(value) {
      const v = Number(value);
      if (!Number.isFinite(v)) return 0;
      return Math.max(0, Math.min(1, v));
    }

    function finiteNumber(value, fallback = NaN) {
      const v = Number(value);
      return Number.isFinite(v) ? v : fallback;
    }

    function coverageKey(value) {
      if (value === undefined || value === null || value === '') return null;
      return String(value);
    }

    function valueSetCoverage(selectedValues, universeValues, expectedValues = null) {
      const universeSet = new Set((Array.isArray(expectedValues) ? expectedValues : universeValues)
        .map(coverageKey)
        .filter(v => v !== null));
      const selectedSet = new Set(selectedValues
        .map(coverageKey)
        .filter(v => v !== null && universeSet.has(v)));
      const total = universeSet.size;
      return { value: total > 0 ? selectedSet.size / total : 0, selectedCount: selectedSet.size, universeCount: total };
    }

    function distinctCoverage(selectedItems, universeItems, keyFn, expectedValues = null) {
      const selectedValues = (selectedItems || []).map(item => keyFn(item));
      const universeValues = (universeItems || []).map(item => keyFn(item));
      return valueSetCoverage(selectedValues, universeValues, expectedValues);
    }

    function quantile(sortedValues, q) {
      if (!sortedValues.length) return NaN;
      const pos = (sortedValues.length - 1) * q;
      const lo = Math.floor(pos);
      const hi = Math.ceil(pos);
      if (lo === hi) return sortedValues[lo];
      return sortedValues[lo] + (sortedValues[hi] - sortedValues[lo]) * (pos - lo);
    }

    function quantileBinCoverage(selectedValues, universeValues, binCount = 10) {
      const universe = (universeValues || []).map(Number).filter(Number.isFinite).sort((a, b) => a - b);
      const selected = (selectedValues || []).map(Number).filter(Number.isFinite);
      if (!universe.length) return { value: 0, selectedBins: 0, universeBins: 0 };
      const min = universe[0];
      const max = universe[universe.length - 1];
      if (Math.abs(max - min) < 1e-12) {
        const covered = selected.some(v => Math.abs(v - min) < 1e-9);
        return { value: covered ? 1 : 0, selectedBins: covered ? 1 : 0, universeBins: 1 };
      }
      const edgeCount = Math.max(2, Number(binCount) || 10);
      const edges = [];
      for (let i = 0; i <= edgeCount; i += 1) {
        const edge = quantile(universe, i / edgeCount);
        if (Number.isFinite(edge) && (!edges.length || Math.abs(edge - edges[edges.length - 1]) > 1e-12)) edges.push(edge);
      }
      if (edges.length < 2) {
        const covered = selected.length > 0;
        return { value: covered ? 1 : 0, selectedBins: covered ? 1 : 0, universeBins: 1 };
      }
      const toBin = (value) => {
        if (value <= edges[0]) return 0;
        for (let i = 0; i < edges.length - 1; i += 1) {
          if (value <= edges[i + 1]) return i;
        }
        return edges.length - 2;
      };
      const universeBins = new Set(universe.map(toBin));
      const selectedBins = new Set(selected.map(toBin).filter(bin => universeBins.has(bin)));
      return { value: universeBins.size > 0 ? selectedBins.size / universeBins.size : 0, selectedBins: selectedBins.size, universeBins: universeBins.size };
    }

    function featureValue(candidate, keys, fallback = NaN) {
      const list = Array.isArray(keys) ? keys : [keys];
      for (const key of list) {
        const v = finiteNumber(candidate?.features?.[key], NaN);
        if (Number.isFinite(v)) return v;
      }
      return fallback;
    }

    function faultKey(fault) {
      if (!fault) return null;
      const idx = fault.branch_index ?? fault.index ?? fault.id;
      if (idx === undefined || idx === null || idx === '') return null;
      return `${fault.branch_type || 'AC'}:${idx}`;
    }

    function faultKeysFromEvent(event) {
      return (Array.isArray(event?.faults) ? event.faults : []).map(faultKey).filter(Boolean);
    }

    function arrayImpactMagnitude(values) {
      if (!Array.isArray(values)) return 0;
      let maxImpact = 0;
      values.forEach(value => {
        const v = Number(value);
        if (Number.isFinite(v)) maxImpact = Math.max(maxImpact, Math.abs(v - 1));
      });
      return maxImpact;
    }

    function coverageDetail(metric) {
      if (!metric) return '无有效候选样本';
      if (metric.universeBins !== undefined) return `覆盖 ${metric.selectedBins || 0}/${metric.universeBins || 0} 个候选分布箱`;
      return `覆盖 ${metric.selectedCount || 0}/${metric.universeCount || 0} 个类别`;
    }

    function deepCloneJson(obj) {
      return obj ? JSON.parse(JSON.stringify(obj)) : obj;
    }

    function normalizeGeneratedScenarioCarbonFields(systemJson) {
      const inferStaticGeneratorEmissionFactor = (item) => {
        const typ = String(item?.sgen_type || item?.type || item?.name || '').toLowerCase();
        if (/diesel/.test(typ)) return 0.70;
        if (/chp|gas|turbine/.test(typ)) return 0.45;
        if (/fuel\s*cell|fuelcell/.test(typ)) return 0.35;
        return 0.0;
      };
      const syncEmissionAlias = (items, inferMissing = null) => (items || []).forEach(item => {
        if (!item) return;
        let canonical = item.emission_factor_tco2_mwh;
        let legacy = item.co2_emission_rate;
        if (canonical === undefined && legacy !== undefined) canonical = legacy;
        if ((canonical === undefined || Math.abs(Number(canonical) || 0) <= 1e-12) && inferMissing) {
          const inferred = inferMissing(item);
          if (inferred > 0) canonical = inferred;
        }
        if (canonical !== undefined) item.emission_factor_tco2_mwh = canonical;
        if (item.co2_emission_rate === undefined && item.emission_factor_tco2_mwh !== undefined) {
          item.co2_emission_rate = item.emission_factor_tco2_mwh;
        }
      });
      syncEmissionAlias(systemJson?.ac?.generators);
      syncEmissionAlias(systemJson?.ac?.static_generators, inferStaticGeneratorEmissionFactor);
      syncEmissionAlias(systemJson?.dc?.static_generators, inferStaticGeneratorEmissionFactor);
      syncEmissionAlias(systemJson?.dc?.dc_static_generators, inferStaticGeneratorEmissionFactor);
    }

    function sumComponentCapacity(items, keys, serviceKey = 'in_service') {
      if (!Array.isArray(items)) return 0;
      return items.reduce((sum, item) => {
        if (item && item[serviceKey] === false) return sum;
        for (const key of keys) {
          const v = Number(item?.[key]);
          if (Number.isFinite(v) && v > 0) return sum + v;
        }
        return sum;
      }, 0);
    }

    function scenarioBaseTotals(systemJson) {
      const ac = systemJson?.ac || {};
      const dc = systemJson?.dc || {};
      const acLoad = sumComponentCapacity(ac.loads, ['p_mw']) || sumComponentCapacity(ac.buses, ['pd_mw']);
      const dcLoad = sumComponentCapacity(dc.loads, ['p_mw']) || sumComponentCapacity(dc.buses, ['pd_mw']);
      const pv = sumComponentCapacity(ac.pv_systems, ['pmax_mw', 'p_mw', 'sn_mva'])
        + sumComponentCapacity((ac.renewable_gens || []).filter(g => /solar|pv/i.test(String(g.type || g.name || ''))), ['p_rated_mw', 'p_mw'])
        + sumComponentCapacity((ac.static_generators || []).filter(g => /pv|solar/i.test(String(g.sgen_type || g.type || g.name || ''))), ['p_rated_mw', 'pmax_mw', 'p_mw'])
        + sumComponentCapacity(dc.pv_arrays, ['p_set_mw'])
        + sumComponentCapacity((dc.dc_static_generators || []).filter(g => /pv|solar/i.test(String(g.type || g.name || ''))), ['p_set_mw']);
      const wind = sumComponentCapacity((ac.renewable_gens || []).filter(g => /wind/i.test(String(g.type || g.name || ''))), ['p_rated_mw', 'p_mw'])
        + sumComponentCapacity((ac.static_generators || []).filter(g => /wind/i.test(String(g.sgen_type || g.type || g.name || ''))), ['p_rated_mw', 'pmax_mw', 'p_mw'])
        + sumComponentCapacity((dc.dc_static_generators || []).filter(g => /wind/i.test(String(g.type || g.name || ''))), ['p_set_mw']);
      return { load: acLoad + dcLoad, pv, wind };
    }

    function makeScaleProfile(rawValues, base, fallbackWhenZero = 1.0) {
      const values = Array.isArray(rawValues) ? rawValues.map(v => Number(v || 0)) : [];
      if (Math.abs(Number(base) || 0) <= 1e-9) return values.map(() => fallbackWhenZero);
      return values.map(v => v / base);
    }

    function profileJsonFromCandidate(candidate, profileName) {
      const values = profileValuesFromCandidate(candidate, profileName);
      return values.length ? { name: profileName, values } : null;
    }

    function buildScenarioTimeSeries(candidate, baseSystem, family) {
      const profiles = candidate?.time_series?.profiles || [];
      if (!Array.isArray(profiles) || profiles.length === 0) return null;
      const rawProfiles = profiles.map((p, i) => ({
        id: Number.isInteger(p.id) ? p.id : i,
        name: p.name || `profile_${i}`,
        values: Array.isArray(p.values) ? p.values.map(v => Number(v || 0)) : [],
      }));
      const load = profileValuesFromCandidate(candidate, 'total_load_mw');
      const pv = profileValuesFromCandidate(candidate, 'pv_mw');
      const wind = profileValuesFromCandidate(candidate, 'wind_mw');
      const renewable = profileValuesFromCandidate(candidate, 'total_renewable_mw');
      const totals = scenarioBaseTotals(baseSystem);
      const calcProfiles = [];
      const warnings = [];
      if (load.length) {
        calcProfiles.push({ id: 0, name: 'scenario_load_scale', values: makeScaleProfile(load, totals.load, 1.0) });
        if (totals.load <= 1e-9) warnings.push('Base load is zero; load scale profile uses fallback values.');
      }
      if (wind.length) {
        calcProfiles.push({ id: 1, name: 'scenario_wind_scale', values: makeScaleProfile(wind, totals.wind, 1.0) });
        if (totals.wind <= 1e-9 && wind.some(v => Math.abs(v) > 1e-9)) warnings.push('Base wind capacity is zero; wind scale profile cannot exactly replay raw MW values.');
      }
      if (pv.length) {
        calcProfiles.push({ id: 2, name: 'scenario_pv_scale', values: makeScaleProfile(pv, totals.pv, 1.0) });
        if (totals.pv <= 1e-9 && pv.some(v => Math.abs(v) > 1e-9)) warnings.push('Base PV capacity is zero; PV scale profile cannot exactly replay raw MW values.');
      }
      if (renewable.length) {
        calcProfiles.push({ id: 3, name: 'scenario_renewable_scale', values: makeScaleProfile(renewable, totals.pv + totals.wind, 1.0) });
      }
      if (calcProfiles.length === 0) return null;
      return {
        version: 1,
        family,
        num_steps: candidate?.time_series?.num_steps || calcProfiles[0].values.length,
        step_duration_hr: candidate?.time_series?.step_duration_hr || 1.0,
        profiles: calcProfiles,
        raw_profiles: rawProfiles,
        binding: {
          assign_all_loads_to: load.length ? 0 : -1,
          assign_all_pv_to: pv.length ? 2 : -1,
          resilience_load_profile_id: load.length ? 0 : -1,
          resilience_renewable_profile_id: renewable.length ? 3 : (pv.length ? 2 : (wind.length ? 1 : -1)),
        },
        normalization: {
          base_load_mw: totals.load,
          base_pv_mw: totals.pv,
          base_wind_mw: totals.wind,
          profile_semantics: 'profiles are dimensionless multipliers for /api/session/set_ts_config; raw_profiles preserve generated MW/SOC series',
        },
        warnings,
      };
    }

    function applyScenarioInitialStorageState(systemJson, candidate) {
      const soc = profileValuesFromCandidate(candidate, 'storage_soc')[0];
      if (!Number.isFinite(soc)) return;
      const update = (items) => (items || []).forEach(st => {
        if (st && Number(st.e_rated_mwh || 0) > 0) st.soc_init = Math.max(0, Math.min(1, soc));
      });
      update(systemJson?.ac?.storage);
      update(systemJson?.dc?.storage);
      update(systemJson?.mobile_storage);
    }

    function markOutOfServiceByIndex(items, index) {
      const list = Array.isArray(items) ? items : [];
      const target = list.find(item => Number(item?.index) === Number(index)) || list[Number(index)] || list[Number(index) - 1];
      if (target) target.in_service = false;
      return !!target;
    }

    function applyReliabilityContingencyToSystem(systemJson, contingency) {
      if (!systemJson || !contingency) return false;
      const idx = contingency.component_index;
      switch (String(contingency.type || '').toLowerCase()) {
        case 'acbranch': return markOutOfServiceByIndex(systemJson.ac?.branches, idx);
        case 'dcbranch': return markOutOfServiceByIndex(systemJson.dc?.branches, idx);
        case 'vscconverter': return markOutOfServiceByIndex(systemJson.vsc_converters, idx);
        case 'dcdcconverter': return markOutOfServiceByIndex(systemJson.dcdc_converters, idx) || markOutOfServiceByIndex(systemJson.dc?.dcdc_converters, idx);
        case 'acgenerator': return markOutOfServiceByIndex(systemJson.ac?.generators, idx);
        case 'acstaticgenerator': return markOutOfServiceByIndex(systemJson.ac?.static_generators, idx);
        case 'acrenewable': return markOutOfServiceByIndex(systemJson.ac?.renewable_gens, idx);
        case 'acpvsystem': return markOutOfServiceByIndex(systemJson.ac?.pv_systems, idx);
        case 'dcstaticgenerator': return markOutOfServiceByIndex(systemJson.dc?.dc_static_generators, idx) || markOutOfServiceByIndex(systemJson.dc?.static_generators, idx);
        case 'dcpvarray': return markOutOfServiceByIndex(systemJson.dc?.pv_arrays, idx);
        case 'acload': return markOutOfServiceByIndex(systemJson.ac?.loads, idx);
        case 'dcload': return markOutOfServiceByIndex(systemJson.dc?.loads, idx);
        case 'acstorage': return markOutOfServiceByIndex(systemJson.ac?.storage, idx);
        case 'dcstorage': return markOutOfServiceByIndex(systemJson.dc?.storage, idx);
        case 'mobilestorage': return markOutOfServiceByIndex(systemJson.mobile_storage, idx);
        case 'transformer2w': return markOutOfServiceByIndex(systemJson.ac?.transformers_2w, idx);
        case 'transformer3w': return markOutOfServiceByIndex(systemJson.ac?.transformers_3w, idx);
        default: return false;
      }
    }

    function extractScenarioClustersByFamily(data, family) {
      if (family === 'regular') return (data?.regular?.clusters || []).map(c => ({ cluster: c, group: null }));
      if (family === 'reliability') {
        return (data?.reliability?.contingencies || []).flatMap(group => (group.clusters || []).map(cluster => ({ cluster, group })));
      }
      if (family === 'resilience') {
        return (data?.resilience?.intensities || []).flatMap(group => (group.clusters || []).map(cluster => ({ cluster, group })));
      }
      return [];
    }

    function isUsableScenarioTimeSeries(ts) {
      if (!ts || !Array.isArray(ts.profiles) || ts.profiles.length === 0) return false;
      const numSteps = Number(ts.num_steps || ts.profiles[0]?.values?.length || 0);
      return numSteps > 0 && ts.profiles.some(p => Array.isArray(p?.values) && p.values.length > 0);
    }

    function getImportedGeneratedScenarioCase(targetFamily = null) {
      const caseJson = _importedGeneratedScenario?.case || null;
      if (!caseJson) return null;
      const family = caseJson?._generated_scenario?.family || _importedGeneratedScenario?.family || '';
      if (targetFamily && family && family !== 'unknown' && family !== targetFamily) return null;
      return caseJson;
    }

    function getImportedGeneratedScenarioTimeSeries(targetFamily = null) {
      const caseJson = getImportedGeneratedScenarioCase(targetFamily);
      const ts = caseJson?._time_series;
      return isUsableScenarioTimeSeries(ts) ? ts : null;
    }

    function hasUsableGeneratedScenarioTimeSeries(targetFamily = null) {
      return !!getImportedGeneratedScenarioTimeSeries(targetFamily);
    }

    function extractScenarioProfileValues(ts, profileIdOrName) {
      if (profileIdOrName === undefined || profileIdOrName === null || profileIdOrName === '') return [];
      const profiles = Array.isArray(ts?.profiles) ? ts.profiles : [];
      const matched = profiles.find(profile => profile?.id === profileIdOrName || String(profile?.id) === String(profileIdOrName) || profile?.name === profileIdOrName);
      return Array.isArray(matched?.values) ? matched.values : [];
    }

    function buildGeneratedScenarioCase(baseSystem, family, cluster, groupContext = null) {
      const representative = cluster?.representative || {};
      const caseJson = deepCloneJson(baseSystem || {});
      normalizeGeneratedScenarioCarbonFields(caseJson);
      const repId = cluster?.representative_id || representative.id || `${family}_scenario_${cluster?.cluster_id ?? 0}`;
      caseJson.name = `${caseJson.name || 'Generated Scenario'} - ${repId}`;
      applyScenarioInitialStorageState(caseJson, representative);
      const metadata = {
        version: 1,
        family,
        scenario_id: representative.id || repId,
        representative_id: repId,
        cluster_id: cluster?.cluster_id ?? 0,
        probability: cluster?.probability ?? representative.probability ?? 0,
        member_count: cluster?.member_count ?? 0,
        member_ids: cluster?.member_ids || [],
        features: representative.features || {},
        anchor_reasons: cluster?.anchor_reasons || representative.anchor_reasons || [],
        frozen_medoid: cluster?.frozen_medoid === true,
        calculation_defaults: {
          preferred_module: family === 'regular' ? 'timeSeries' : family,
          use_time_series: false,
          static_fallback: true,
          run_opf: false,
          run_power_flow: family === 'resilience',
        },
      };
      if (family === 'reliability') {
        metadata.contingency = representative.contingency || groupContext?.contingency || null;
        metadata.applied_to_system = false;
        metadata.contingency_application = 'metadata_only';
      }
      if (family === 'resilience') {
        metadata.intensity = groupContext?.intensity || representative.resilience_event?.selected_intensity || null;
        metadata.resilience_event = representative.resilience_event || null;
      }
      caseJson._generated_scenario = metadata;
      const ts = family === 'reliability' ? null : buildScenarioTimeSeries(representative, caseJson, family);
      if (ts) {
        caseJson._time_series = ts;
        metadata.calculation_defaults.use_time_series = family === 'regular' || family === 'resilience';
      }
      return caseJson;
    }

    function buildGeneratedScenarioBundle(family) {
      if (!_lastScenarioGenerationData) throw new Error('暂无场景生成结果，请先生成场景');
      const baseSystem = _lastScenarioBaseSystemJson || Canvas.buildSystemJson();
      const entries = extractScenarioClustersByFamily(_lastScenarioGenerationData, family);
      if (!entries.length) throw new Error(`暂无${family}代表场景可导出`);
      const cases = entries.map(({ cluster, group }) => buildGeneratedScenarioCase(baseSystem, family, cluster, group));
      return {
        format: 'generated_scenario_case_bundle_v2',
        schema_version: 2,
        case_format: 'hacdcpf_system_json_with_generated_scenario_metadata_v1',
        time_series_preserved: cases.some(c => isUsableScenarioTimeSeries(c?._time_series)),
        family,
        case_count: cases.length,
        exported_at: new Date().toISOString(),
        source_summary: _lastScenarioGenerationData.summary || {},
        cases,
      };
    }

    async function applyGeneratedScenarioTimeSeries(caseJson) {
      const ts = caseJson?._time_series;
      if (!ts || !Array.isArray(ts.profiles) || ts.profiles.length === 0) return false;
      const body = {
        num_steps: ts.num_steps || ts.profiles[0]?.values?.length || 24,
        step_duration_hr: ts.step_duration_hr || 1.0,
        profiles: ts.profiles,
        load_profile_map: ts.binding?.load_profile_map || [],
        assign_all_loads_to: Number.isInteger(ts.binding?.assign_all_loads_to) ? ts.binding.assign_all_loads_to : -1,
        assign_all_pv_to: Number.isInteger(ts.binding?.assign_all_pv_to) ? ts.binding.assign_all_pv_to : -1,
      };
      const r = await fetch('/api/session/set_ts_config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const j = await r.json();
      if (!r.ok) throw new Error(j.error || 'set_ts_config failed');
      _generatedScenarioTimeSeriesActive = true;
      const simHr = document.getElementById('simulationHours');
      if (simHr) simHr.value = String(j.num_steps || body.num_steps);
      log(`生成场景时序已恢复：${j.num_steps}步，profiles=${j.num_profiles}`, 'success');
      return true;
    }

    function generatedScenarioCaseFaults(caseJson) {
      const event = caseJson?._generated_scenario?.resilience_event;
      return Array.isArray(event?.faults) ? event.faults : [];
    }

    function generatedScenarioCaseLabel(caseJson, index) {
      const meta = caseJson?._generated_scenario || {};
      const event = meta.resilience_event || {};
      const faults = generatedScenarioCaseFaults(caseJson);
      const family = meta.family || 'unknown';
      const rep = meta.representative_id || meta.scenario_id || caseJson?.name || `case_${index + 1}`;
      if (family === 'resilience') {
        const starts = faults.map(f => Number(f.start_hr ?? f.outage_start_hr)).filter(Number.isFinite);
        const startText = starts.length ? `，首故障 ${Math.min(...starts).toFixed(1)}h` : '';
        const intensity = meta.intensity || event.selected_intensity || event.requested_intensity || '-';
        return `${index + 1}. ${rep}｜强度 ${intensity}｜故障 ${faults.length} 个${startText}`;
      }
      if (family === 'reliability') {
        const cont = meta.contingency || {};
        return `${index + 1}. ${rep}｜${cont.type || 'N-1'} ${cont.display_name || cont.id || ''}`;
      }
      return `${index + 1}. ${rep}`;
    }

    function chooseGeneratedScenarioCase(cases, targetFamily) {
      const matched = cases.filter(c => !targetFamily || c?._generated_scenario?.family === targetFamily);
      const choices = matched.length ? matched : cases;
      if (!choices.length) throw new Error('生成场景文件中没有可导入 case');
      if (choices.length === 1) return choices[0];

      const defaultIndex = targetFamily === 'resilience'
        ? Math.max(0, choices.findIndex(c => generatedScenarioCaseFaults(c).length > 0))
        : 0;
      const listed = choices.slice(0, 40).map((caseJson, index) => generatedScenarioCaseLabel(caseJson, index)).join('\n');
      const suffix = choices.length > 40 ? `\n... 另有 ${choices.length - 40} 个场景未列出，可直接输入编号选择` : '';
      const promptText = `该文件包含 ${choices.length} 个${targetFamily || ''}生成场景，请输入要导入的编号：\n\n${listed}${suffix}`;
      const input = window.prompt(promptText, String(defaultIndex + 1));
      if (input === null) throw new Error('已取消导入生成场景');
      const chosen = Number.parseInt(input, 10);
      if (!Number.isInteger(chosen) || chosen < 1 || chosen > choices.length) {
        throw new Error(`场景编号无效，请输入 1-${choices.length} 之间的整数`);
      }
      return choices[chosen - 1];
    }

    function firstCaseFromGeneratedScenarioJson(obj, targetFamily) {
      const allowedBundleFormats = new Set(['generated_scenario_case_bundle_v1', 'generated_scenario_case_bundle_v2']);
      if (allowedBundleFormats.has(obj?.format) && Array.isArray(obj.cases)) {
        return chooseGeneratedScenarioCase(obj.cases, targetFamily);
      }
      if (obj && (obj.ac || obj.dc)) return obj;
      throw new Error('不是有效的生成场景 JSON 或系统算例 JSON');
    }

    function fillResilienceInputsFromScenario(caseJson) {
      const event = caseJson?._generated_scenario?.resilience_event;
      const faults = generatedScenarioCaseFaults(caseJson);
      if (!faults.length) {
        document.getElementById('resFaultCount').value = '0';
        document.getElementById('resAcFaultLocations').value = '';
        document.getElementById('resDcFaultLocations').value = '';
        document.getElementById('resAcFaultStartHour').value = '';
        document.getElementById('resDcFaultStartHour').value = '';
        document.getElementById('resAcRepairDuration').value = '';
        document.getElementById('resDcRepairDuration').value = '';
        return;
      }
      const ac = [], dc = [];
      faults.forEach(f => {
        const type = String(f.branch_type || f.branch_kind || 'AC').toUpperCase();
        const entry = { id: f.branch_index ?? f.branch_id ?? f.branch, start: Number(f.start_hr ?? f.outage_start_hr ?? 0), repair: Number(f.repair_hr ?? f.repair_duration_hr ?? 6) };
        if (type === 'DC') dc.push(entry); else ac.push(entry);
      });
      document.getElementById('resFaultCount').value = String(ac.length + dc.length);
      document.getElementById('resAcFaultLocations').value = ac.map(f => f.id).join(',');
      document.getElementById('resDcFaultLocations').value = dc.map(f => f.id).join(',');
      document.getElementById('resAcFaultStartHour').value = ac.map(f => Number.isFinite(f.start) ? f.start : 0).join(',');
      document.getElementById('resDcFaultStartHour').value = dc.map(f => Number.isFinite(f.start) ? f.start : 0).join(',');
      document.getElementById('resAcRepairDuration').value = ac.map(f => Number.isFinite(f.repair) ? f.repair : 6).join(',');
      document.getElementById('resDcRepairDuration').value = dc.map(f => Number.isFinite(f.repair) ? f.repair : 6).join(',');
    }

    async function importGeneratedScenarioForModule(file, targetFamily) {
      try {
        const text = await readFileAsText(file);
        const parsed = JSON.parse(text);
        const caseJson = deepCloneJson(firstCaseFromGeneratedScenarioJson(parsed, targetFamily));
        normalizeGeneratedScenarioCarbonFields(caseJson);
        const family = caseJson?._generated_scenario?.family || 'unknown';
        const data = await apiPost('/api/session/load_json_string', { json_string: JSON.stringify(caseJson) });
        if (!data) throw new Error('导入系统失败');
        Canvas.loadFromSystemJson(caseJson);
        _canvasDirty = false;
        _importedGeneratedScenario = { family, case: caseJson };
        _lastImportedGeneratedScenarioKey = caseJson?._generated_scenario?.representative_id || caseJson?.name || '';
        _lastTspfData = null;
        const target = targetFamily || family;
        let restoredTs = false;
        const targetUsesScenarioTs = target === 'regular' || target === 'resilience';
        if (targetUsesScenarioTs) {
          restoredTs = await applyGeneratedScenarioTimeSeries(caseJson);
        }
        if (!targetUsesScenarioTs) {
          const regCb = document.getElementById('regUseScenarioTimeSeries');
          const resCb = document.getElementById('resUseScenarioTimeSeries');
          if (regCb) regCb.checked = false;
          if (resCb) resCb.checked = false;
        }
        if (target === 'regular' || family === 'regular') {
          const cb = document.getElementById('regUseScenarioTimeSeries');
          if (cb) cb.checked = restoredTs;
          if (!restoredTs) log('导入的常规生成场景不含可用时序，已保持“使用场景时序”未勾选', 'warn');
        }
        if (family === 'resilience' || targetFamily === 'resilience') {
          fillResilienceInputsFromScenario(caseJson);
          const cb = document.getElementById('resUseScenarioTimeSeries');
          if (cb) cb.checked = restoredTs;
          if (!restoredTs) log('导入的弹性生成场景不含可用时序，已保持“使用场景时序”未勾选', 'warn');
        }
        if (family === 'reliability' || targetFamily === 'reliability') {
          const cont = caseJson?._generated_scenario?.contingency;
          if (cont) log(`已导入可靠性代表场景（N-1 静态断面）：${cont.type || ''} ${cont.display_name || cont.id || ''}`, 'info');
        }
        log(`已导入生成场景：${caseJson?._generated_scenario?.representative_id || caseJson.name || file.name}${restoredTs ? '，并恢复时序配置' : ''}`, 'success');
        setStatus('生成场景已导入');
      } catch (err) {
        log(`导入生成场景失败：${err.message || err}`, 'error');
        setStatus('导入生成场景失败', 'error');
      }
    }

    function renderScenarioGenerationCharts(data) {
      const chartsDiv = document.getElementById('scenarioGenerationCharts');
      if (!chartsDiv) return;
      chartsDiv.innerHTML = `
        <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px;margin-bottom:16px;">
          <div id="scenarioChartRegular" class="tspf-chart" style="min-height:340px;"></div>
          <div id="scenarioChartReliability" class="tspf-chart" style="min-height:340px;"></div>
          <div id="scenarioChartResilience" class="tspf-chart" style="min-height:340px;"></div>
          <div id="scenarioChartRegularCurves" class="tspf-chart" style="min-height:360px;"></div>
          <div id="scenarioChartReliabilityBars" class="tspf-chart" style="min-height:360px;"></div>
          <div id="scenarioChartResilienceCurves" class="tspf-chart" style="min-height:360px;"></div>
          <div id="scenarioChartFaultSequence" class="tspf-chart" style="min-height:360px;"></div>
        </div>`;
      if (!window.Plotly) {
        chartsDiv.insertAdjacentHTML('afterbegin', '<p class="empty-hint">Plotly 未加载，无法展示图表。</p>');
        return;
      }
      const theme = {
        paper_bgcolor: '#ffffff',
        plot_bgcolor: '#ffffff',
        font: { color: '#22272e', size: 11 },
        margin: { l: 52, r: 28, t: 42, b: 58 },
        legend: { orientation: 'h', y: -0.22 },
        xaxis: { gridcolor: '#d0d7de' },
        yaxis: { gridcolor: '#d0d7de', rangemode: 'tozero' },
      };

      const regularClusters = data.regular?.clusters || [];
      const regularSamples = data.regular?.audit?.coverage_samples || [];
      const regularSelected = [];
      regularClusters.forEach(cluster => {
        const selected = parseRegularId(cluster.representative_id);
        regularSelected.push({
          ...selected,
          group: `${selected.ssp}-${selected.year}`,
          cluster: cluster.cluster_id,
          probability: cluster.probability || 0,
          peakLoad: numericFeature(cluster, 'peak_load'),
          renewable: numericFeature(cluster, 'renewable_sum'),
          load: numericFeature(cluster, 'load_sum'),
          features: cluster.representative?.features || {},
        });
      });
      const regularAll = regularSamples.map(s => {
        const parsed = parseRegularId(s.id);
        return {
          ...parsed,
          group: `${parsed.ssp}-${parsed.year}`,
          load: Number(s.features?.load_sum || 0),
          renewable: Number(s.features?.renewable_sum || 0),
          features: s.features || {},
        };
      });
      Plotly.newPlot('scenarioChartRegular', [
        {
          x: regularAll.map(p => p.load),
          y: regularAll.map(p => p.renewable),
          mode: 'markers',
          type: 'scatter',
          name: '全部候选样本',
          marker: { color: '#60a5fa', size: 6, opacity: 0.35, symbol: 'circle' },
          text: regularAll.map(p => `${p.group}<br>负荷特征 ${p.load.toFixed(2)}<br>新能源特征 ${p.renewable.toFixed(2)}`),
        },
        {
          x: regularSelected.map(p => p.load),
          y: regularSelected.map(p => p.renewable),
          mode: 'markers',
          type: 'scatter',
          name: '聚类选中代表',
          text: regularSelected.map(p => `${p.group}<br>簇${p.cluster}<br>负荷特征 ${p.load.toFixed(2)}<br>新能源特征 ${p.renewable.toFixed(2)}`),
          marker: { symbol: 'circle-open', color: '#dc2626', size: 13, line: { color: '#dc2626', width: 3 } },
        },
      ], { ...theme, title: '常规场景二维覆盖：负荷不确定性 × 新能源不确定性（小圈=全部样本，红圈=聚类代表）', xaxis: { title: '负荷特征', autorange: true }, yaxis: { title: '新能源出力特征', autorange: true } }, { responsive: true });

      const relGroups = data.reliability?.contingencies || [];
      const relSamples = data.reliability?.audit?.coverage_samples || [];
      const relSelected = [];
      relGroups.forEach((g, index) => {
        (g.clusters || []).forEach(c => {
          relSelected.push({
            type: g.contingency?.type || 'Unknown',
            index: index + 1,
            probability: c.probability || 0,
            load: numericFeature(c, 'load_sum'),
            renewable: numericFeature(c, 'renewable_sum'),
            faultOrdinal: Number(c.representative?.features?.contingency_ordinal || index + 1),
            name: g.contingency?.display_name || g.contingency?.id || '',
            features: c.representative?.features || {},
          });
        });
      });
      const relAll = relSamples.map(s => ({
        type: s.contingency?.type || 'Unknown',
        faultOrdinal: Number(s.features?.contingency_ordinal || 0),
        load: Number(s.features?.load_sum || 0),
        renewable: Number(s.features?.renewable_sum || 0),
        name: s.contingency?.display_name || s.contingency?.id || s.id || '',
        features: s.features || {},
      }));
      Plotly.newPlot('scenarioChartReliability', [
        {
          x: relAll.map(p => p.load),
          y: relAll.map(p => p.renewable),
          mode: 'markers',
          type: 'scatter',
          name: '全部N-1候选样本',
          text: relAll.map(p => `${p.name}<br>${p.type}<br>负荷 ${p.load.toFixed(3)} / 新能源 ${p.renewable.toFixed(3)}`),
          marker: { color: '#64748b', size: 6, opacity: 0.30, symbol: 'circle' },
        },
        {
          x: relSelected.map(p => p.load),
          y: relSelected.map(p => p.renewable),
          mode: 'markers',
          type: 'scatter',
          name: '聚类选中代表',
          text: relSelected.map(p => `${p.name}<br>${p.type}<br>负荷 ${p.load.toFixed(3)} / 新能源 ${p.renewable.toFixed(3)}<br>簇概率 ${(p.probability * 100).toFixed(2)}%`),
          marker: { symbol: 'circle-open', color: '#dc2626', size: 13, line: { color: '#dc2626', width: 3 } },
        },
      ], { ...theme, title: '可靠性场景二维覆盖：负荷不确定性 × 新能源不确定性（小圈=全部样本，红圈=聚类代表）', xaxis: { title: '负荷特征', autorange: true }, yaxis: { title: '新能源出力特征', autorange: true } }, { responsive: true });

      const resGroups = data.resilience?.intensities || [];
      const resSamples = data.resilience?.audit?.coverage_samples || [];
      const resAll = resSamples.map(s => {
        const event = s.resilience_event || {};
        return {
          category: event.selected_intensity || '',
          maxWind: eventMaxWind(event) || Number(s.features?.selected_track_max_vmax_ms || 0),
          faultCount: Array.isArray(event.faults) ? event.faults.length : Number(s.features?.fault_count || 0),
          peakFailure: Number(s.features?.peak_failure_probability || 0),
          month: event.month || '-',
          sampleId: event.selected_sample_id || s.id || '',
          features: s.features || {},
        };
      });
      const resSelected = [];
      resGroups.forEach(g => {
        (g.clusters || []).forEach(c => {
          const event = c.representative?.resilience_event || {};
          const faultCount = Array.isArray(event.faults) ? event.faults.length : 0;
          resSelected.push({
            category: event.selected_intensity || g.intensity,
            maxWind: eventMaxWind(event),
            sampleId: event.selected_sample_id || '',
            month: event.month || '-',
            faultCount,
            probability: c.probability || 0,
            peakFailure: numericFeature(c, 'peak_failure_probability'),
            id: c.representative_id,
            features: c.representative?.features || {},
          });
        });
      });
      Plotly.newPlot('scenarioChartResilience', [
        {
          x: resAll.map(p => p.maxWind),
          y: resAll.map(p => p.faultCount),
          mode: 'markers',
          type: 'scatter',
          name: '全部台风样本',
          text: resAll.map(p => `划分后强度 ${p.category}<br>最大风速(不含t0) ${p.maxWind.toFixed(2)} m/s<br>月份 ${p.month}<br>样本 ${p.sampleId}<br>故障数 ${p.faultCount}<br>峰值失效概率 ${(p.peakFailure * 100).toFixed(2)}%`),
          marker: { color: '#f59e0b', size: 6, opacity: 0.35, symbol: 'circle' },
        },
        {
          x: resSelected.map(p => p.maxWind),
          y: resSelected.map(p => p.faultCount),
          mode: 'markers',
          type: 'scatter',
          name: '聚类选中代表',
          text: resSelected.map(p => `${p.id}<br>划分后强度 ${p.category}<br>最大风速(不含t0) ${p.maxWind.toFixed(2)} m/s<br>月份 ${p.month}<br>样本 ${p.sampleId}<br>故障数 ${p.faultCount}<br>峰值失效概率 ${(p.peakFailure * 100).toFixed(2)}%<br>簇概率 ${(p.probability * 100).toFixed(2)}%`),
          marker: { symbol: 'circle-open', color: '#dc2626', size: 13, line: { color: '#dc2626', width: 3 } },
        },
      ], { ...theme, title: '弹性场景二维覆盖：最大风速 × 故障数（小圈=全部样本，红圈=聚类代表）', xaxis: { title: '最大风速 / m/s（不含初始时刻）' }, yaxis: { title: '故障线路数', rangemode: 'tozero' } }, { responsive: true });

      const regularRepresentatives = regularClusters.map(c => c.representative).filter(Boolean);
      const relRepresentatives = relGroups.flatMap(g => (g.clusters || []).map(c => c.representative).filter(Boolean));
      const firstRegular = regularClusters[0]?.representative;
      if (firstRegular) {
        const load = profileValuesFromCandidate(firstRegular, 'total_load_mw').slice(0, 168);
        const pv = profileValuesFromCandidate(firstRegular, 'pv_mw').slice(0, 168);
        const wind = profileValuesFromCandidate(firstRegular, 'wind_mw').slice(0, 168);
        const x = load.map((_, i) => i);
        const traces = [
          { x, y: load, type: 'scatter', mode: 'lines', name: '负荷 MW', line: { color: '#2563eb' } },
          { x, y: pv, type: 'scatter', mode: 'lines', name: '光伏 MW', line: { color: '#f59e0b' } },
          { x, y: wind, type: 'scatter', mode: 'lines', name: '风电 MW', line: { color: '#16a34a' } },
        ];
        Plotly.newPlot('scenarioChartRegularCurves', traces,
          { ...theme, title: '常规代表场景风/光/负荷曲线（前168小时）', xaxis: { title: '小时' }, yaxis: { title: 'MW', rangemode: 'tozero' } }, { responsive: true });
      }

      const firstRel = relGroups.find(g => (g.clusters || []).length)?.clusters?.[0]?.representative;
      if (firstRel) {
        const names = ['负荷', '光伏', '风电'];
        const values = ['total_load_mw', 'pv_mw', 'wind_mw'].map(name => profileValuesFromCandidate(firstRel, name)[0] || 0);
        Plotly.newPlot('scenarioChartReliabilityBars', [{ x: names, y: values, type: 'bar', marker: { color: ['#2563eb', '#f59e0b', '#16a34a'] } }],
          { ...theme, title: '可靠性代表场景风/光/负荷（N-1 单时段）', xaxis: { title: '变量' }, yaxis: { title: 'MW', rangemode: 'tozero' } }, { responsive: true });
      }

      const allResClusters = resGroups.flatMap(g => g.clusters || []);
      const firstFaultedResCluster = allResClusters.find(c => ((c.representative?.resilience_event || {}).faults || []).length > 0);
      const firstRes = (firstFaultedResCluster || allResClusters[0])?.representative;
      if (firstRes) {
        const load = profileValuesFromCandidate(firstRes, 'total_load_mw');
        const pv = profileValuesFromCandidate(firstRes, 'pv_mw');
        const wind = profileValuesFromCandidate(firstRes, 'wind_mw');
        const x = load.map((_, i) => i);
        const traces = [
          { x, y: load, type: 'scatter', mode: 'lines', name: '负荷 MW', line: { color: '#2563eb' } },
          { x, y: pv, type: 'scatter', mode: 'lines', name: '光伏 MW', line: { color: '#f59e0b' } },
          { x, y: wind, type: 'scatter', mode: 'lines', name: '风电 MW', line: { color: '#16a34a' } },
        ];
        Plotly.newPlot('scenarioChartResilienceCurves', traces,
          { ...theme, title: '弹性代表场景风/光/负荷曲线（48小时）', xaxis: { title: '小时' }, yaxis: { title: 'MW', rangemode: 'tozero' } }, { responsive: true });

        const faults = (firstRes.resilience_event?.faults || [])
          .map((f, idx) => {
            const start = finiteNumber(f.start_hr, NaN);
            return {
              ...f,
              seq: idx + 1,
              start: Number.isFinite(start) ? start : 0,
              label: `${f.branch_type || 'AC'}-${f.branch_index ?? '-'}`,
            };
          })
          .sort((a, b) => (a.start - b.start) || (a.seq - b.seq));
        if (faults.length) {
          Plotly.newPlot('scenarioChartFaultSequence', [{
            x: faults.map(f => f.start),
            y: faults.map((_, idx) => idx + 1),
            type: 'scatter',
            mode: 'markers',
            name: '故障发生时刻',
            marker: { color: '#dc2626', size: 12, symbol: 'x' },
            text: faults.map(f => `${f.label}<br>故障发生 ${f.start.toFixed(2)} h`),
            hovertemplate: '%{text}<extra></extra>',
          }], {
            ...theme,
            title: '弹性代表场景故障序列（只展示线路故障发生时刻；修复由弹性评估决策）',
            xaxis: { title: '故障发生小时', range: [0, Math.max(48, ...faults.map(f => f.start))], rangemode: 'tozero' },
            yaxis: { title: '故障序号（按开始时间排序）', dtick: 1, rangemode: 'tozero' },
          }, { responsive: true });
        } else {
          Plotly.newPlot('scenarioChartFaultSequence', [], {
            ...theme,
            title: '弹性代表场景故障序列（该代表场景无线路故障）',
            xaxis: { title: '小时', range: [0, 48] },
            yaxis: { visible: false },
            annotations: [{ text: '该弹性代表场景无故障线路', x: 0.5, y: 0.5, xref: 'paper', yref: 'paper', showarrow: false }],
          }, { responsive: true });
        }
      }
    }

    function renderScenarioGenerationResults(data) {
      const div = document.getElementById('scenarioGenerationResults');
      const summaryDiv = document.getElementById('resultsSummary');
      if (!div || !summaryDiv) return;
      renderScenarioGenerationCharts(data);
      const summary = data.summary || {};
      const regularAudit = data.regular?.audit || {};
      const regularPerGroup = regularAudit.requested_cluster_count_per_ssp_year ?? '-';
      const regularGroups = regularAudit.group_count ?? '-';
      summaryDiv.innerHTML = `
        <div class="result-item"><span class="result-label">常规候选/总聚类</span><span class="result-value">${summary.regular_candidate_count ?? 0} / ${summary.regular_cluster_count ?? 0}</span></div>
        <div class="result-item"><span class="result-label">常规每组×组数</span><span class="result-value">${regularPerGroup} × ${regularGroups}</span></div>
        <div class="result-item"><span class="result-label">可靠性FMEA N-1/聚类</span><span class="result-value">${summary.reliability_contingency_count ?? 0} / ${summary.reliability_cluster_total ?? 0}</span></div>
        <div class="result-item"><span class="result-label">台风划分集合/聚类</span><span class="result-value">${summary.resilience_intensity_count ?? 0} / ${summary.resilience_cluster_total ?? 0}</span></div>
      `;
      const warningHtml = Array.isArray(data.warnings) && data.warnings.length
        ? `<div style="margin-bottom:10px;color:#b45309;"><strong>Warnings:</strong> ${data.warnings.map(escapeHtml).join('；')}</div>`
        : '';
      const regularRows = (data.regular?.clusters || []).map(c => `<tr>
        <td>${c.cluster_id}</td><td>${escapeHtml(c.representative_id || '')}</td><td>${((c.probability || 0) * 100).toFixed(2)}%</td>
        <td>${c.member_count ?? 0}</td><td>${firstProfileSummary(c, 'total_load_mw')}</td><td>${firstProfileSummary(c, 'total_renewable_mw')}</td>
      </tr>`).join('');
      const relGroups = data.reliability?.contingencies || [];
      const relRows = relGroups.slice(0, 120).map(g => {
        const best = (g.clusters || [])[0] || {};
        const cont = g.contingency || {};
        return `<tr><td>${escapeHtml(cont.id || '')}</td><td>${escapeHtml(cont.type || '')}</td><td>${escapeHtml(cont.display_name || '')}</td><td>${g.candidate_count ?? 0}</td><td>${g.cluster_count ?? 0}</td><td>${((best.probability || 0) * 100).toFixed(2)}%</td><td>${firstProfileSummary(best, 'total_load_mw')}</td><td>${firstProfileSummary(best, 'total_renewable_mw')}</td></tr>`;
      }).join('');
      const relMore = relGroups.length > 120 ? `<p class="empty-hint">仅展示前120个 N-1 分组；完整结果可导出 JSON。</p>` : '';
      const resRows = (data.resilience?.intensities || []).flatMap(g => (g.clusters || []).map(top => {
        const event = top.representative?.resilience_event || {};
        const faultCount = Array.isArray(event.faults) ? event.faults.length : 0;
        const maxWind = eventMaxWind(event);
        const faultTimes = (Array.isArray(event.faults) ? event.faults : [])
          .map(f => finiteNumber(f.start_hr, NaN))
          .filter(Number.isFinite)
          .sort((a, b) => a - b);
        const firstFault = faultTimes.length ? faultTimes[0].toFixed(2) : '-';
        const faultTimeText = faultTimes.length ? faultTimes.slice(0, 6).map(v => v.toFixed(1)).join(', ') + (faultTimes.length > 6 ? '...' : '') : '-';
        return `<tr><td>${escapeHtml(g.intensity || '')}</td><td>${g.candidate_count ?? 0}</td><td>${g.cluster_count ?? 0}</td><td>${escapeHtml(top.representative_id || '')}</td><td>${escapeHtml(event.selected_intensity || '')}</td><td>${maxWind > 0 ? maxWind.toFixed(2) : '-'}</td><td>${escapeHtml(String(event.month || '-'))} / ${escapeHtml(event.selected_sample_id || '-')}</td><td>${faultCount}</td><td>${firstFault}</td><td>${escapeHtml(faultTimeText)}</td></tr>`;
      })).join('');
      div.innerHTML = `
        ${warningHtml}
        <p class="empty-hint">常规场景按 SSP×年份分别聚类；若当前算例含 AC renewable/PV/static PV/DC PV 等新能源资源，会同时扰动负荷与新能源出力。内置 dist33_microgrid_der、comprehensive_hybrid_acdc 均包含新能源/DER。</p>
        <p class="empty-hint">若当前算例含储能，场景生成仍会在导出的 JSON 中保存场景开始时实际 SOC 与对应电量，但结果图和表格仅展示负荷、新能源与故障信息。</p>
        <h4>常规全年代表场景（8760h）</h4>
        <table><thead><tr><th>簇</th><th>代表场景</th><th>概率</th><th>成员数</th><th>负荷总量/峰值</th><th>新能源总量/峰值</th></tr></thead><tbody>${regularRows || '<tr><td colspan="6" style="color:#888">未生成常规场景</td></tr>'}</tbody></table>
        <h4>可靠性 N-1 单断面场景</h4>
        <p class="empty-hint">可靠性覆盖图采用二维映射：x=负荷特征，y=新能源出力特征。</p>
        ${relMore}
        <table><thead><tr><th>N-1 ID</th><th>类型</th><th>元件</th><th>候选</th><th>聚类</th><th>首簇概率</th><th>首簇负荷摘要</th><th>首簇新能源摘要</th></tr></thead><tbody>${relRows || '<tr><td colspan="8" style="color:#888">未生成可靠性场景</td></tr>'}</tbody></table>
        <h4>弹性台风代表场景（48h，一次扰动+台风二次扰动）</h4>
        <table><thead><tr><th>划分后强度集合</th><th>候选</th><th>聚类</th><th>代表场景</th><th>分类强度</th><th>最大风速(m/s，不含t0)</th><th>月份/sample</th><th>故障数</th><th>首故障(h)</th><th>故障时刻(h)</th></tr></thead><tbody>${resRows || '<tr><td colspan="10" style="color:#888">未生成弹性场景</td></tr>'}</tbody></table>
      `;
    }


    document.getElementById('btnGenerateScenarios')?.addEventListener('click', async () => {
      let payload;
      try {
        payload = collectScenarioGenerationOptions();
      } catch (err) {
        log(`场景参数错误：${err.message}`, 'warn');
        setStatus('场景参数错误', 'error');
        return;
      }
      setStatus('场景生成中...', 'busy');
      _lastScenarioBaseSystemJson = Canvas.buildSystemJson();
      if (!await syncToBackend()) { setStatus('同步失败', 'error'); return; }
      const data = await apiPost('/api/session/generate_scenarios', payload);
      if (!data) { setStatus('场景生成失败', 'error'); return; }
      _lastScenarioGenerationData = data;
      setActiveResultGroup('scenarioGeneration');
      document.getElementById('resultsEmpty').style.display = 'none';
      document.getElementById('resultsContent').style.display = 'block';
      switchTab('results');
      renderScenarioGenerationResults(data);
      setTimeout(() => {
        ['scenarioChartRegular', 'scenarioChartReliability', 'scenarioChartResilience', 'scenarioChartRegularCurves', 'scenarioChartReliabilityBars', 'scenarioChartResilienceCurves', 'scenarioChartFaultSequence'].forEach(id => {
          const el = document.getElementById(id);
          if (el && window.Plotly) Plotly.Plots.resize(el);
        });
      }, 50);
      log(`场景生成完成：常规${data.summary?.regular_cluster_count ?? 0}簇，可靠性${data.summary?.reliability_contingency_count ?? 0}个N-1，弹性${data.summary?.resilience_cluster_total ?? 0}簇`, 'success');
      setStatus('场景生成完成');
    });

    function exportGeneratedScenarioFamily(family, label) {
      try {
        const bundle = buildGeneratedScenarioBundle(family);
        downloadJsonFile(`${family}_generated_scenarios_${tsTagForFilename()}.json`, bundle);
        log(`${label}可导入场景已生成：${bundle.case_count} 个代表场景`, 'success');
      } catch (err) {
        log(`${label}场景导出失败：${err.message || err}`, 'warn');
      }
    }

    document.getElementById('btnExportRegularScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('regular', '常规'));
    document.getElementById('btnExportReliabilityScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('reliability', '可靠性'));
    document.getElementById('btnExportResilienceScenarioJson')?.addEventListener('click', () => exportGeneratedScenarioFamily('resilience', '弹性'));

    document.getElementById('btnExportScenarioResults')?.addEventListener('click', () => {
      if (!_lastScenarioGenerationData) {
        log('暂无场景生成结果可导出，请先生成场景', 'warn');
        return;
      }
      downloadJsonFile(`scenario_generation_${tsTagForFilename()}.json`, _lastScenarioGenerationData);
    });

    document.getElementById('btnExportScenarioPlots')?.addEventListener('click', async () => {
      if (!window.Plotly) return;
      const ids = ['scenarioChartRegular', 'scenarioChartReliability', 'scenarioChartResilience', 'scenarioChartRegularCurves', 'scenarioChartReliabilityBars', 'scenarioChartResilienceCurves', 'scenarioChartFaultSequence'];
      const tag = tsTagForFilename();
      for (const id of ids) {
        const el = document.getElementById(id);
        if (!el) continue;
        await Plotly.downloadImage(el, { format: 'png', width: 1400, height: 900, filename: `${id}_${tag}` });
        await new Promise(resolve => setTimeout(resolve, 250));
      }
    });

    function bindGeneratedScenarioImport(buttonId, fileId, family) {
      const btn = document.getElementById(buttonId);
      const input = document.getElementById(fileId);
      btn?.addEventListener('click', () => input?.click());
      input?.addEventListener('change', () => {
        const file = input.files && input.files[0];
        if (file) importGeneratedScenarioForModule(file, family);
        input.value = '';
      });
    }
    bindGeneratedScenarioImport('btnImportGeneratedRegularScenario', 'fileImportGeneratedRegularScenario', 'regular');
    bindGeneratedScenarioImport('btnImportGeneratedReliabilityScenario', 'fileImportGeneratedReliabilityScenario', 'reliability');
    bindGeneratedScenarioImport('btnImportGeneratedResilienceScenario', 'fileImportGeneratedResilienceScenario', 'resilience');

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
