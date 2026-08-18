/** Keyboard-shortcut help panel and toolbar undo/redo wiring for the HySim workspace. */
'use strict';

(function initHelpPanel(global) {
  const core = global.HySimCore = global.HySimCore || {};

  // Single source of truth for the cheat sheet. Keep aligned with the keydown
  // handler in web/js/canvas.js (onKeyDown) and the workspace shortcut in
  // web/js/app.js (Ctrl/Cmd+Shift+F focus mode).
  const SHORTCUT_GROUPS = [
    {
      title: '选择',
      entries: [
        { keys: 'V', desc: '切换到选择模式' },
        { keys: 'C', desc: '切换到连线模式' },
        { keys: 'Esc', desc: '返回选择模式并取消当前选择' },
        { keys: 'Ctrl/Cmd + A', desc: '全选所有元件' },
        { keys: 'Delete / Backspace', desc: '删除选中的元件或连线' },
      ],
    },
    {
      title: '编辑',
      entries: [
        { keys: 'Ctrl/Cmd + Z', desc: '撤销上一步编辑' },
        { keys: 'Ctrl/Cmd + Shift + Z', desc: '重做' },
        { keys: 'Ctrl/Cmd + Y', desc: '重做（与上一条等价）' },
        { keys: 'Ctrl/Cmd + C', desc: '复制选中元件及其两端都被选中的连线' },
        { keys: 'Ctrl/Cmd + V', desc: '粘贴（每次连续粘贴偏移 +20px，网格对齐）' },
        { keys: 'Ctrl/Cmd + D', desc: '创建副本（复制并立即粘贴）' },
        { keys: 'R / Shift + R', desc: '选中元件顺时针 / 逆时针旋转 90°' },
      ],
    },
    {
      title: '视图',
      entries: [
        { keys: 'Ctrl/Cmd + Shift + F', desc: '专注模式（隐藏元件库、功能区与控制台）' },
        { keys: '?', desc: '打开或关闭本快捷键面板（Shift + /）' },
      ],
    },
    {
      title: '分析',
      entries: [
        { keys: '—', desc: '潮流、短路、可靠性等分析经功能区按钮触发，暂无全局快捷键' },
      ],
    },
  ];

  const HEADLESS_NOTE =
    '大规模系统（>400 母线）以 WebGL 全网总览 headless 运行时，复制/粘贴/旋转/删除等编辑快捷键不可用，' +
    '撤销栈保持为空；请从所选母线打开局部 SVG 子图，或在拓扑表中编辑全量数据。';

  const UNDO_REFRESH_MS = 300;
  let initialized = false;
  let modal = null;
  let lastFocus = null;

  function isEditableTarget(target) {
    return !!target && (target.tagName === 'INPUT' || target.tagName === 'SELECT' ||
      target.tagName === 'TEXTAREA' || target.isContentEditable);
  }

  function isOpen() {
    return !!modal && modal.style.display !== 'none';
  }

  function canvasApi() {
    // canvas.js exposes a top-level `const Canvas` (classic script), which is
    // reachable as a bare identifier but not via window.Canvas.
    return typeof Canvas !== 'undefined' ? Canvas : null;  // eslint-disable-line no-undef
  }

  function renderShortcuts(container) {
    container.textContent = '';
    SHORTCUT_GROUPS.forEach(group => {
      const section = document.createElement('section');
      section.className = 'help-shortcut-group';
      const heading = document.createElement('h4');
      heading.textContent = group.title;
      section.appendChild(heading);
      group.entries.forEach(entry => {
        const row = document.createElement('div');
        row.className = 'help-shortcut-row';
        const keys = document.createElement('kbd');
        keys.textContent = entry.keys;
        const desc = document.createElement('span');
        desc.className = 'help-shortcut-desc';
        desc.textContent = entry.desc;
        row.appendChild(keys);
        row.appendChild(desc);
        section.appendChild(row);
      });
      container.appendChild(section);
    });
    const note = document.createElement('p');
    note.className = 'help-modal-note';
    note.textContent = HEADLESS_NOTE;
    container.appendChild(note);
  }

  function open() {
    if (!modal || isOpen()) return;
    lastFocus = document.activeElement;
    modal.style.display = 'flex';
    modal.querySelector('#btnHelpClose')?.focus();
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

  function refreshUndoRedo() {
    const canvas = canvasApi();
    const undoBtn = document.getElementById('btnUndo');
    const redoBtn = document.getElementById('btnRedo');
    if (!canvas || typeof canvas.canUndo !== 'function') {
      if (undoBtn) undoBtn.disabled = true;
      if (redoBtn) redoBtn.disabled = true;
      return;
    }
    if (undoBtn) undoBtn.disabled = !canvas.canUndo();
    if (redoBtn) redoBtn.disabled = !canvas.canRedo();
  }

  function onKeyDown(event) {
    if (event.defaultPrevented || isEditableTarget(event.target)) return;
    if (event.key === '?') {
      event.preventDefault();
      toggle();
    } else if (event.key === 'Escape' && isOpen()) {
      close();
    }
  }

  function wireToolbar() {
    const canvas = canvasApi();
    document.getElementById('btnUndo')?.addEventListener('click', () => {
      const api = canvasApi();
      if (api && api.canUndo()) api.undo();
      refreshUndoRedo();
    });
    document.getElementById('btnRedo')?.addEventListener('click', () => {
      const api = canvasApi();
      if (api && api.canRedo()) api.redo();
      refreshUndoRedo();
    });
    refreshUndoRedo();
    // Lightweight state sync: poll on a slow timer plus refresh after every
    // keyup/mouseup, which covers all undoable gestures (keys, drags, clicks).
    global.setInterval(refreshUndoRedo, UNDO_REFRESH_MS);
    document.addEventListener('keyup', refreshUndoRedo);
    document.addEventListener('mouseup', refreshUndoRedo);
  }

  function init() {
    if (initialized) return;
    initialized = true;
    modal = document.getElementById('helpModal');
    if (modal) {
      const list = modal.querySelector('#helpShortcutList');
      if (list) renderShortcuts(list);
      modal.querySelector('#btnHelpClose')?.addEventListener('click', close);
      // Click on the dimmed backdrop (the .modal element itself) closes.
      modal.addEventListener('click', event => {
        if (event.target === modal) close();
      });
    }
    wireToolbar();
    document.addEventListener('keydown', onKeyDown);
  }

  core.HelpPanel = Object.freeze({
    schema: 'hysim_help_panel_v1',
    shortcutGroups: () => SHORTCUT_GROUPS.map(group => ({
      title: group.title,
      entries: group.entries.map(entry => ({ ...entry })),
    })),
    headlessNote: HEADLESS_NOTE,
    isOpen,
    open,
    close,
    toggle,
    refreshUndoRedo,
    init,
  });

  // Scripts load at the end of <body>, so the DOM is already available.
  init();
})(window);
