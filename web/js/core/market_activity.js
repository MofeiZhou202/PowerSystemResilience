/* Transport activity is distinct from solver progress: /status reports session-wide busy only. */
(() => {
  'use strict';
  const root = document.getElementById('marketActivity');
  const pending = new Map(), reads = new Map(), deferred = new Map();
  const names = { marketOperation: '运行模拟', marketBoundary: '单日市场', marketStudy: '对比试验', marketAncillary: '云南调频', marketRealtime: '实时市场' };
  const actions = { step: '计算日窗', explain: '补算当日原因', run: '运行出清', start: '建立仿真', generate: '生成场景', save: '保存配置', cancel: '请求终止', intraday: '日内出清' };
  const sources = { market_operation: '载入运行任务', market_forecast: '载入预测场景', southern_market: '载入市场边界', market_study: '载入对比试验', yunnan_ancillary: '载入调频任务', southern_realtime: '载入实时任务' };
  let serial = 0, generation = 0, timer = null, poll = null, lastPoll = 0;
  const seconds = since => Math.floor((performance.now() - since) / 1000);
  const mb = value => `${(value / 1048576).toFixed(1)} MB`;
  function draw(item) {
    const elapsed = seconds(item.started);
    item.time.textContent = `已耗时 ${Math.floor(elapsed / 60)}分${String(elapsed % 60).padStart(2, '0')}秒`;
    item.phase.textContent = item.received
      ? `正在接收结果 · ${mb(item.received)}${item.total ? ` / ${mb(item.total)}` : ''}`
      : '等待服务端返回';
  }
  async function heartbeat() {
    if (poll || !pending.size || performance.now() - lastPoll < 2500) return;
    lastPoll = performance.now(); poll = new AbortController();
    const controller = poll, timeout = setTimeout(() => controller.abort(), 2500);
    try {
      const response = await fetch('/api/session/status', { signal: controller.signal });
      if (!response.ok) throw new Error('status');
      const data = await response.json();
      if (typeof data.busy !== 'boolean') throw new Error('status');
      for (const item of pending.values()) item.server.textContent = data.busy
        ? '服务端有计算在执行（会话状态）' : '服务端在线 · 未报告计算占用';
    } catch {
      for (const item of pending.values()) item.server.textContent = '暂未收到服务端状态 · 正在重试';
    } finally { clearTimeout(timeout); if (poll === controller) poll = null; }
  }
  function begin(owner, label) {
    const id = ++serial, row = document.createElement('div'); row.className = 'market-activity-row';
    const title = document.createElement('strong'), phase = document.createElement('span'), time = document.createElement('span'), server = document.createElement('span');
    title.textContent = `${names[owner] || '市场'} · ${label}`; title.setAttribute('role', 'status');
    const back = document.createElement('button'); back.type = 'button'; back.className = 'btn btn-sm'; back.textContent = '返回任务';
    back.onclick = () => { App.setActiveModule(owner); if (owner === 'marketOperation') window.HySimMarketOperation?.setStep(3); };
    for (const old of root.querySelectorAll('[data-state]:not([data-state="pending"])')) old.remove();
    row.append(title, time, phase, server, back); root.prepend(row); root.scrollTop = 0; root.hidden = false;
    row.dataset.state = 'pending'; row.setAttribute('aria-busy', 'true');
    const item = { id, row, title, phase, time, server, started: performance.now(), received: 0, total: 0 };
    pending.set(id, item); draw(item);
    if (!timer) timer = setInterval(() => { for (const value of pending.values()) draw(value); heartbeat(); }, 1000);
    return item;
  }
  function finish(item, error) {
    pending.delete(item.id); item.row.setAttribute('aria-busy', 'false'); item.row.dataset.state = error ? 'error' : 'received';
    item.phase.textContent = error || `响应已收到${item.received ? ` · ${mb(item.received)}` : ''}`;
    item.server.textContent = '';
    if (pending.size && !error) item.row.remove();
    if (!pending.size) { clearInterval(timer); timer = null; poll?.abort(); }
    setTimeout(() => { item.row.remove(); root.hidden = !root.childElementCount; }, error ? 15000 : 3000);
  }
  function request(path, body, { owner = 'marketOperation', label } = {}) {
    // Deduplicate only overlapping reads; every mutation invalidates the read generation.
    if (body) ++generation;
    const key = `${generation}:${path}`;
    if (!body && reads.has(key)) return reads.get(key);
    const source = path.split('?')[0].split('/').pop();
    const item = begin(owner, label || (body ? actions[body.action] || '更新市场数据' : sources[source] || '载入任务数据'));
    const promise = new Promise((resolve, reject) => {
      const xhr = new XMLHttpRequest(); xhr.open(body ? 'POST' : 'GET', path); xhr.responseType = 'json';
      if (body) xhr.setRequestHeader('Content-Type', 'application/json');
      xhr.onprogress = event => { item.received = event.loaded; item.total = event.lengthComputable ? event.total : 0; draw(item); };
      xhr.onload = () => {
        const data = xhr.response;
        const error = xhr.status === 404 ? '当前服务版本未提供此市场接口，请使用更新后的 GUI 服务。'
          : !data ? `服务响应不是有效 JSON（HTTP ${xhr.status}），请检查服务状态后重载任务。`
          : xhr.status < 200 || xhr.status >= 300 ? data.error || `HTTP ${xhr.status}` : null;
        finish(item, error); if (error) reject(new Error(error)); else resolve(data);
      };
      xhr.onerror = xhr.onabort = () => {
        const message = '连接中断，计算结果尚未确认；请重载任务核对，勿重复提交。';
        finish(item, message); reject(new Error(message));
      };
      xhr.send(body ? JSON.stringify(body) : null);
    });
    if (!body) {
      reads.set(key, promise);
      const clear = () => { if (reads.get(key) === promise) reads.delete(key); };
      promise.then(clear, clear);
    }
    return promise;
  }
  const observer = new ResizeObserver(entries => {
    for (const { target } of entries) if (target.getClientRects().length && target.clientWidth && deferred.has(target)) {
      const render = deferred.get(target); deferred.delete(target); observer.unobserve(target); render();
    }
  });
  function whenVisible(id, render) {
    const target = document.getElementById(id);
    if (target.getClientRects().length && target.clientWidth) { deferred.delete(target); observer.unobserve(target); render(); }
    else { deferred.set(target, render); observer.observe(target); }
  }
  function cancelRender(id) {
    const target = document.getElementById(id); deferred.delete(target); observer.unobserve(target);
  }
  window.HySimMarketActivity = { request, whenVisible, cancelRender };
})();
