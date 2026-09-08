/**
 * Documentation help center for the HySim workspace (HySimCore.HelpCenter).
 *
 * Serves the repository docs/ tree through the GUI: a modal with a sectioned
 * navigation tree (left), rendered markdown (right) and a debounced search
 * box (top). Documents are fetched read-only from the server mount
 * `/xjtu/docs/<relative path>` (see tests/run_gui_server.cpp) and listed in
 * the manifest `web/help_docs.json` (schema `hysim_help_docs_v1`).
 *
 * Key behaviours:
 * - Markdown is rendered with the vendored marked (web/vendor/marked.min.js);
 *   raw HTML tokens in the source are escaped, never injected. If marked is
 *   missing a minimal built-in renderer (headings/lists/code/tables/links/
 *   emphasis) is used so the center still works.
 * - Relative .md links are rewritten to in-center navigation (resolved
 *   against the current document path; links escaping the docs root are not
 *   followed internally). External and non-markdown links open in a new tab;
 *   relative image sources are repointed under /xjtu/docs/.
 * - Entries carrying a `modules` list are pinned to a "当前模块相关" block
 *   when the matching analysis module tab (`.module-btn.active`) is active.
 * - Keyboard: F1 toggles the center (browser default suppressed, editable
 *   targets guarded), Esc closes, ArrowUp/ArrowDown move within the visible
 *   navigation, Enter opens the focused entry. The shared `.modal` focus
 *   trap (core/accessibility.js) applies automatically.
 */
'use strict';

(function initHelpCenter(global) {
  const core = global.HySimCore = global.HySimCore || {};

  const MANIFEST_URL = 'help_docs.json';
  const MANIFEST_SCHEMA = 'hysim_help_docs_v1';
  const DOCS_BASE = '/xjtu/docs/';
  const SEARCH_DEBOUNCE_MS = 150;
  const NARROW_NAV_MEDIA = '(max-width: 719px)';

  let initialized = false;
  let modal = null;
  let navTreeEl = null;
  let contextEl = null;
  let contentEl = null;
  let searchEl = null;
  let statusEl = null;
  let navToggleBtn = null;
  let manifest = null;
  let manifestError = null;
  let manifestPromise = null;
  let lastFocus = null;
  let activePath = null;
  let searchTimer = 0;
  const docCache = new Map();   // path -> Promise<string> (markdown source)

  // ---------- small utilities ----------

  function escapeHtml(text) {
    return String(text)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;');
  }

  function isEditableTarget(target) {
    return !!target && (target.tagName === 'INPUT' || target.tagName === 'SELECT' ||
      target.tagName === 'TEXTAREA' || target.isContentEditable);
  }

  /**
   * Resolve `rel` (a relative reference without fragment/query) against the
   * document path `basePath` (e.g. 'guides/parameter_system.md'). Returns the
   * normalized docs-root-relative path, or null when the reference escapes
   * the docs root (too many '..').
   */
  function resolveDocPath(basePath, rel) {
    const segments = String(basePath || '').split('/');
    segments.pop();  // drop the file name; keep the directory stack
    for (const part of String(rel).split('/')) {
      if (part === '' || part === '.') continue;
      if (part === '..') {
        if (!segments.length) return null;
        segments.pop();
      } else {
        segments.push(part);
      }
    }
    return segments.join('/');
  }

  function splitRef(ref) {
    const hashAt = ref.indexOf('#');
    const hash = hashAt >= 0 ? ref.slice(hashAt) : '';
    const withoutHash = hashAt >= 0 ? ref.slice(0, hashAt) : ref;
    const queryAt = withoutHash.indexOf('?');
    return {
      path: queryAt >= 0 ? withoutHash.slice(0, queryAt) : withoutHash,
      suffix: (queryAt >= 0 ? withoutHash.slice(queryAt) : '') + hash,
    };
  }

  function isMarkdownPath(path) {
    return /\.md$/i.test(path);
  }

  function hasScheme(ref) {
    return /^[a-z][a-z0-9+.-]*:/i.test(ref);
  }

  /**
   * Rewrite anchors and image sources inside rendered markdown HTML.
   * - in-page anchors (#...) are kept;
   * - http(s)/mailto links open in a new tab (rel="noopener");
   * - other schemes (javascript:, data:, ...) are neutralized;
   * - relative .md references that resolve inside the docs root become
   *   in-center navigation (data-doc-path); other relative files are
   *   repointed under DOCS_BASE and open in a new tab;
   * - relative image sources are repointed under DOCS_BASE.
   */
  function rewriteDocHtml(html, basePath) {
    const rewriteHref = (match, pre, href, post) => {
      const decoded = href.replace(/&amp;/g, '&');
      if (decoded.startsWith('#')) return match;
      if (hasScheme(decoded) || decoded.startsWith('//')) {
        if (/^(https?:|mailto:)/i.test(decoded)) {
          return `<a ${pre}href="${escapeHtml(decoded)}"${post} target="_blank" rel="noopener">`;
        }
        return `<a ${pre}${post} data-blocked-href="${escapeHtml(decoded)}" class="help-center-blocked-link">`;
      }
      const { path, suffix } = splitRef(decoded);
      const resolved = resolveDocPath(basePath, path);
      if (resolved && isMarkdownPath(resolved)) {
        return `<a ${pre}href="${DOCS_BASE}${escapeHtml(resolved)}${escapeHtml(suffix)}"${post}` +
          ` data-doc-path="${escapeHtml(resolved)}">`;
      }
      if (resolved) {
        return `<a ${pre}href="${DOCS_BASE}${escapeHtml(resolved)}${escapeHtml(suffix)}"${post}` +
          ' target="_blank" rel="noopener">';
      }
      // Escapes the docs root: leave the browser to resolve it, new tab only.
      return `<a ${pre}href="${escapeHtml(decoded)}"${post} target="_blank" rel="noopener">`;
    };
    const rewriteSrc = (match, pre, src, post) => {
      const decoded = src.replace(/&amp;/g, '&');
      if (hasScheme(decoded) || decoded.startsWith('/') || decoded.startsWith('//')) return match;
      const { path, suffix } = splitRef(decoded);
      const resolved = resolveDocPath(basePath, path);
      if (!resolved) return match;
      return `<img ${pre}src="${DOCS_BASE}${escapeHtml(resolved)}${escapeHtml(suffix)}"${post}>`;
    };
    return String(html)
      .replace(/<a\s+([^>]*?)href="([^"]*)"([^>]*)>/gi, rewriteHref)
      .replace(/<img\s+([^>]*?)src="([^"]*)"([^>]*)>/gi, rewriteSrc);
  }

  // ---------- markdown rendering ----------

  let markedParser = null;   // null = not tried yet; false = unavailable

  function buildMarkedParser() {
    const marked = global.marked;
    if (!marked) return false;
    try {
      const escapeToken = token => escapeHtml(typeof token === 'string' ? token : (token && token.text) || '');
      if (typeof marked.Marked === 'function') {
        return new marked.Marked({
          gfm: true,
          breaks: false,
          renderer: { html: escapeToken },
        });
      }
      if (typeof marked.use === 'function' && typeof marked.parse === 'function') {
        marked.use({ renderer: { html: escapeToken } });
        return marked;
      }
    } catch (err) {
      return false;
    }
    return false;
  }

  /** Minimal fallback renderer: headings, lists, fenced code, tables, links, emphasis. */
  function renderMarkdownFallback(src) {
    const inline = text => escapeHtml(text)
      .replace(/!\[([^\]]*)\]\(([^)\s]+)\)/g, '<img alt="$1" src="$2">')
      .replace(/\[([^\]]+)\]\(([^)\s]+)\)/g, '<a href="$2">$1</a>')
      .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
      .replace(/(?<!\w)\*([^*\n]+)\*(?!\w)/g, '<em>$1</em>')
      .replace(/`([^`]+)`/g, '<code>$1</code>');
    const lines = String(src).replace(/\r\n?/g, '\n').split('\n');
    const out = [];
    let i = 0;
    let para = [];
    const flushPara = () => {
      if (para.length) { out.push(`<p>${para.map(inline).join('<br>')}</p>`); para = []; }
    };
    while (i < lines.length) {
      const line = lines[i];
      const fence = line.match(/^```/);
      if (fence) {
        flushPara();
        const buf = [];
        i += 1;
        while (i < lines.length && !/^```/.test(lines[i])) { buf.push(lines[i]); i += 1; }
        i += 1;  // closing fence (or EOF)
        out.push(`<pre><code>${escapeHtml(buf.join('\n'))}</code></pre>`);
        continue;
      }
      const heading = line.match(/^(#{1,6})\s+(.*)$/);
      if (heading) {
        flushPara();
        const level = heading[1].length;
        out.push(`<h${level}>${inline(heading[2].trim())}</h${level}>`);
        i += 1;
        continue;
      }
      if (/^\s*([-*+]|\d+\.)\s+/.test(line)) {
        flushPara();
        const ordered = /^\s*\d+\./.test(line);
        const items = [];
        while (i < lines.length && /^\s*([-*+]|\d+\.)\s+/.test(lines[i])) {
          items.push(lines[i].replace(/^\s*([-*+]|\d+\.)\s+/, ''));
          i += 1;
        }
        const tag = ordered ? 'ol' : 'ul';
        out.push(`<${tag}>${items.map(item => `<li>${inline(item)}</li>`).join('')}</${tag}>`);
        continue;
      }
      if (/^\s*\|.*\|\s*$/.test(line) && i + 1 < lines.length && /^\s*\|[\s:|-]+\|\s*$/.test(lines[i + 1])) {
        flushPara();
        const splitRow = row => row.trim().replace(/^\||\|$/g, '').split('|').map(cell => cell.trim());
        const head = splitRow(line);
        i += 2;  // header + delimiter
        const rows = [];
        while (i < lines.length && /^\s*\|.*\|\s*$/.test(lines[i])) { rows.push(splitRow(lines[i])); i += 1; }
        out.push('<table><thead><tr>' + head.map(cell => `<th>${inline(cell)}</th>`).join('') +
          '</tr></thead><tbody>' +
          rows.map(row => '<tr>' + row.map(cell => `<td>${inline(cell)}</td>`).join('') + '</tr>').join('') +
          '</tbody></table>');
        continue;
      }
      if (/^\s*$/.test(line)) { flushPara(); i += 1; continue; }
      if (/^>\s?/.test(line)) {
        flushPara();
        const buf = [];
        while (i < lines.length && /^>\s?/.test(lines[i])) { buf.push(lines[i].replace(/^>\s?/, '')); i += 1; }
        out.push(`<blockquote><p>${buf.map(inline).join('<br>')}</p></blockquote>`);
        continue;
      }
      para.push(line);
      i += 1;
    }
    flushPara();
    return out.join('\n');
  }

  function renderMarkdown(src) {
    if (markedParser === null) markedParser = buildMarkedParser();
    if (markedParser) {
      try {
        return markedParser.parse(src);
      } catch (err) {
        // fall through to the minimal renderer
      }
    }
    return renderMarkdownFallback(src);
  }

  // ---------- manifest & module context ----------

  function loadManifest() {
    if (manifest) return Promise.resolve(manifest);
    if (manifestPromise) return manifestPromise;
    manifestPromise = global.fetch(MANIFEST_URL)
      .then(response => {
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return response.json();
      })
      .then(data => {
        if (!data || data.schema !== MANIFEST_SCHEMA || !Array.isArray(data.sections)) {
          throw new Error(`清单 schema 不是 ${MANIFEST_SCHEMA}`);
        }
        manifest = data;
        manifestError = null;
        return manifest;
      })
      .catch(err => {
        manifestError = err;
        manifestPromise = null;  // allow retry on next open
        throw err;
      });
    return manifestPromise;
  }

  /** Flatten manifest sections into [{section, entry}] preserving order. */
  function flatEntries(data) {
    const flat = [];
    (data && data.sections || []).forEach(section => {
      (section.entries || []).forEach(entry => flat.push({ section, entry }));
    });
    return flat;
  }

  /** Entries whose `modules` list contains the given GUI module id. */
  function moduleEntries(data, moduleId) {
    if (!moduleId) return [];
    return flatEntries(data).filter(item =>
      Array.isArray(item.entry.modules) && item.entry.modules.includes(moduleId));
  }

  /** Title+tags (plus section title) substring filter, case-insensitive. */
  function filterEntries(data, query) {
    const q = String(query || '').trim().toLowerCase();
    if (!q) return flatEntries(data);
    return flatEntries(data).filter(item => {
      const hay = [item.entry.title, item.section.title, ...(item.entry.tags || [])]
        .join('\n').toLowerCase();
      return hay.includes(q);
    });
  }

  function currentModuleId() {
    if (typeof document === 'undefined') return null;
    const active = document.querySelector('.module-btn.active');
    return active && active.dataset ? (active.dataset.module || null) : null;
  }

  // ---------- DOM ----------

  function el(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = text;
    return node;
  }

  function buildModal() {
    modal = el('div', 'modal help-center-modal');
    modal.id = 'helpCenterModal';
    modal.style.display = 'none';

    const content = el('div', 'modal-content help-center-content');

    const header = el('div', 'help-center-header');
    navToggleBtn = el('button', 'btn btn-sm help-center-nav-toggle', '目录');
    navToggleBtn.type = 'button';
    navToggleBtn.setAttribute('aria-expanded', 'true');
    navToggleBtn.setAttribute('aria-controls', 'helpCenterNav');
    const title = el('h3', 'help-center-title', '文档帮助中心');
    title.id = 'helpCenterTitle';
    searchEl = document.createElement('input');
    searchEl.id = 'helpCenterSearch';
    searchEl.type = 'search';
    searchEl.className = 'help-center-search';
    searchEl.placeholder = '搜索标题或标签…';
    searchEl.setAttribute('aria-label', '搜索文档标题或标签');
    const closeBtn = el('button', 'btn btn-sm', '关闭');
    closeBtn.type = 'button';
    closeBtn.id = 'helpCenterClose';
    header.appendChild(navToggleBtn);
    header.appendChild(title);
    header.appendChild(searchEl);
    header.appendChild(closeBtn);

    const body = el('div', 'help-center-body');
    const nav = el('nav', 'help-center-nav');
    nav.id = 'helpCenterNav';
    nav.setAttribute('aria-label', '文档导航');
    contextEl = el('div', 'help-center-context');
    navTreeEl = el('div', 'help-center-nav-tree');
    nav.appendChild(contextEl);
    nav.appendChild(navTreeEl);
    contentEl = el('div', 'help-center-doc');
    contentEl.id = 'helpCenterContent';
    contentEl.tabIndex = -1;
    body.appendChild(nav);
    body.appendChild(contentEl);

    statusEl = el('div', 'help-center-status');
    statusEl.id = 'helpCenterStatus';
    statusEl.setAttribute('role', 'status');

    content.appendChild(header);
    content.appendChild(body);
    content.appendChild(statusEl);
    modal.appendChild(content);

    closeBtn.addEventListener('click', close);
    modal.addEventListener('click', event => { if (event.target === modal) close(); });
    searchEl.addEventListener('input', () => {
      global.clearTimeout(searchTimer);
      searchTimer = global.setTimeout(() => renderNav(searchEl.value), SEARCH_DEBOUNCE_MS);
    });
    searchEl.addEventListener('keydown', event => {
      if (event.key === 'ArrowDown') {
        event.preventDefault();
        firstNavButton()?.focus();
      }
    });
    navTreeEl.addEventListener('keydown', onNavKeyDown);
    contextEl.addEventListener('keydown', onNavKeyDown);
    navToggleBtn.addEventListener('click', () => {
      const open = !modal.classList.contains('nav-collapsed');
      setNavCollapsed(open);
    });
    contentEl.addEventListener('click', event => {
      const link = event.target && event.target.closest
        ? event.target.closest('a[data-doc-path]')
        : null;
      if (!link) return;
      event.preventDefault();
      const path = link.getAttribute('data-doc-path');
      const href = link.getAttribute('href') || '';
      const hashAt = href.indexOf('#');
      openDoc(path, hashAt >= 0 ? href.slice(hashAt + 1) : '');
    });

    document.body.appendChild(modal);
  }

  function setNavCollapsed(collapsed) {
    modal.classList.toggle('nav-collapsed', collapsed);
    navToggleBtn.setAttribute('aria-expanded', collapsed ? 'false' : 'true');
  }

  function navButtons() {
    return Array.from(modal.querySelectorAll('.help-center-entry'))
      .filter(btn => btn.offsetParent !== null || btn.getClientRects().length > 0);
  }

  function firstNavButton() {
    return navButtons()[0] || null;
  }

  function onNavKeyDown(event) {
    if (event.key !== 'ArrowDown' && event.key !== 'ArrowUp') return;
    const buttons = navButtons();
    if (!buttons.length) return;
    const current = buttons.indexOf(document.activeElement);
    let next = event.key === 'ArrowDown' ? current + 1 : current - 1;
    if (next < 0) next = 0;
    if (next >= buttons.length) next = buttons.length - 1;
    event.preventDefault();
    buttons[next].focus();
  }

  function renderContext() {
    contextEl.textContent = '';
    if (searchEl.value.trim()) return;  // search results replace the pinned block
    const moduleId = currentModuleId();
    const related = moduleEntries(manifest, moduleId);
    if (!moduleId || !related.length) return;
    const activeBtn = document.querySelector('.module-btn.active');
    const moduleLabel = activeBtn ? activeBtn.textContent.trim() : moduleId;
    const heading = el('h4', 'help-center-context-title', `当前模块相关：${moduleLabel}`);
    contextEl.appendChild(heading);
    related.forEach(item => contextEl.appendChild(navButton(item.entry)));
  }

  function navButton(entry) {
    const btn = el('button', 'help-center-entry', entry.title);
    btn.type = 'button';
    btn.dataset.docPath = entry.path;
    btn.classList.toggle('active', entry.path === activePath);
    btn.addEventListener('click', () => {
      openDoc(entry.path);
      if (global.matchMedia && global.matchMedia(NARROW_NAV_MEDIA).matches) setNavCollapsed(true);
    });
    return btn;
  }

  function renderNav(query) {
    if (!manifest) return;
    navTreeEl.textContent = '';
    renderContext();
    const matched = filterEntries(manifest, query);
    const bySection = new Map();
    matched.forEach(item => {
      if (!bySection.has(item.section.id)) bySection.set(item.section.id, { section: item.section, entries: [] });
      bySection.get(item.section.id).entries.push(item.entry);
    });
    if (!matched.length) {
      navTreeEl.appendChild(el('p', 'help-center-empty', `没有匹配“${query}”的文档`));
      return;
    }
    manifest.sections.forEach(section => {
      const group = bySection.get(section.id);
      if (!group) return;
      const sectionEl = el('section', 'help-center-section');
      sectionEl.appendChild(el('h4', 'help-center-section-title', section.title));
      group.entries.forEach(entry => sectionEl.appendChild(navButton(entry)));
      navTreeEl.appendChild(sectionEl);
    });
  }

  function highlightActive() {
    modal.querySelectorAll('.help-center-entry').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.docPath === activePath);
    });
  }

  function setStatus(text, isError) {
    statusEl.textContent = text || '';
    statusEl.classList.toggle('help-center-status-error', !!isError);
  }

  function showContentError(titleText, detail) {
    contentEl.textContent = '';
    const box = el('div', 'help-center-error');
    box.appendChild(el('h3', '', titleText));
    box.appendChild(el('p', '', detail));
    contentEl.appendChild(box);
  }

  function fetchDoc(path) {
    if (!docCache.has(path)) {
      docCache.set(path, global.fetch(DOCS_BASE + path).then(response => {
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return response.text();
      }));
    }
    return docCache.get(path);
  }

  function openDoc(path, fragment) {
    activePath = path;
    highlightActive();
    setStatus(`加载中：docs/${path}`);
    contentEl.textContent = '';
    contentEl.appendChild(el('p', 'help-center-loading', `正在加载 docs/${path} …`));
    fetchDoc(path)
      .then(src => {
        if (activePath !== path) return;  // user navigated away meanwhile
        contentEl.innerHTML = rewriteDocHtml(renderMarkdown(src), path);
        setStatus(`docs/${path}`);
        if (fragment) {
          const target = contentEl.querySelector(`#${CSS.escape(fragment)}`) ||
            contentEl.querySelector(`[name="${fragment}"]`);
          if (target && typeof target.scrollIntoView === 'function') target.scrollIntoView();
        } else {
          contentEl.scrollTop = 0;
        }
      })
      .catch(err => {
        if (activePath !== path) return;
        docCache.delete(path);  // do not cache failures; allow retry
        showContentError(`无法加载 docs/${path}`,
          `服务器返回错误（${err && err.message ? err.message : err}）。` +
          '请确认 GUI 服务器正在运行且该文件仍存在于仓库 docs/ 目录。');
        setStatus(`加载失败：docs/${path}`, true);
      });
  }

  function defaultDocPath() {
    const related = moduleEntries(manifest, currentModuleId());
    if (related.length) return related[0].entry.path;
    const first = manifest.sections[0] && manifest.sections[0].entries[0];
    return first ? first.path : null;
  }

  // ---------- open / close ----------

  function isOpen() {
    return !!modal && modal.style.display !== 'none';
  }

  function open(requestedPath) {
    if (!modal || isOpen()) return;
    lastFocus = document.activeElement;
    modal.style.display = 'flex';
    if (global.matchMedia && global.matchMedia(NARROW_NAV_MEDIA).matches) setNavCollapsed(true);
    loadManifest()
      .then(() => {
        renderNav('');
        searchEl.value = '';
        if (typeof requestedPath === 'string' && requestedPath) {
          openDoc(requestedPath);
        } else if (!activePath) {
          const path = defaultDocPath();
          if (path) openDoc(path);
        } else {
          highlightActive();
          renderContext();
        }
      })
      .catch(err => {
        navTreeEl.textContent = '';
        showContentError('文档清单加载失败',
          `无法获取 ${MANIFEST_URL}（${err && err.message ? err.message : err}）。` +
          '请确认 GUI 服务器正在运行并挂载了 web/ 目录，然后重新打开帮助中心重试。');
        setStatus('清单加载失败', true);
      });
    searchEl.focus();
  }

  function close() {
    if (!modal || !isOpen()) return;
    modal.style.display = 'none';
    if (lastFocus && typeof lastFocus.focus === 'function') lastFocus.focus();
    lastFocus = null;
  }

  function toggle() {
    if (isOpen()) close(); else open();
  }

  function onGlobalKeyDown(event) {
    if (event.key === 'F1') {
      if (!isOpen() && isEditableTarget(event.target)) return;
      event.preventDefault();
      toggle();
      return;
    }
    if (event.key === 'Escape' && isOpen()) {
      close();
    }
  }

  function init() {
    if (initialized) return;
    initialized = true;
    buildModal();
    document.addEventListener('keydown', onGlobalKeyDown);
  }

  core.HelpCenter = Object.freeze({
    schema: 'hysim_help_center_v1',
    docsBase: DOCS_BASE,
    isOpen,
    open,
    close,
    toggle,
    init,
    // Pure helpers exposed for headless smoke tests (tmp/help_center_smoke.mjs).
    resolveDocPath,
    rewriteDocHtml,
    renderMarkdown,
    filterEntries,
    moduleEntries,
  });

  // Scripts load at the end of <body>; guard for headless test harnesses
  // that evaluate this file without a DOM.
  if (typeof document !== 'undefined' && document.body) init();
})(typeof window !== 'undefined' ? window : globalThis);
