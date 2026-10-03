/** Keyboard navigation and accessibility contracts for the HySim workspace. */
'use strict';

(function initAccessibility(global) {
  const core = global.HySimCore = global.HySimCore || {};
  const TABLISTS = [
    ['#workflowBar', '.workflow-btn'],
    ['#moduleBar', '.module-btn'],
    ['.panel-tabs', '.panel-tab'],
    ['[role="tablist"]', 'button'],
  ];
  let initialized = false;
  let observer = null;
  let syncPending = false;

  function isVisible(element) {
    return !!element && !element.hidden && element.getAttribute('aria-hidden') !== 'true' &&
      !element.classList.contains('workflow-hidden') && element.offsetParent !== null;
  }

  function accessibleName(element) {
    return String(element?.getAttribute('aria-label') || element?.textContent || '').trim();
  }

  function setupTablist(container, selector) {
    if (!container) return;
    // The portal owns selection and arrow-key handling for its tablists.
    // Binding again advances twice and overwrites its aria-selected state.
    if (container.closest('#resiliencePortalRoot')) return;
    container.setAttribute('role', 'tablist');
    const buttons = Array.from(container.querySelectorAll(selector));
    const visible = buttons.filter(isVisible);
    const selected = visible.find(button => button.classList.contains('active')) || visible[0];
    buttons.forEach(button => {
      button.setAttribute('role', 'tab');
      const active = button === selected;
      button.setAttribute('aria-selected', active ? 'true' : 'false');
      button.tabIndex = active ? 0 : -1;
    });
    if (container.dataset.a11yKeys === '1') return;
    container.dataset.a11yKeys = '1';
    container.addEventListener('keydown', event => {
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
      const items = Array.from(container.querySelectorAll(selector)).filter(isVisible);
      if (!items.length) return;
      const current = Math.max(0, items.indexOf(document.activeElement));
      let targetIndex = current;
      if (event.key === 'ArrowLeft') targetIndex = (current - 1 + items.length) % items.length;
      if (event.key === 'ArrowRight') targetIndex = (current + 1) % items.length;
      if (event.key === 'Home') targetIndex = 0;
      if (event.key === 'End') targetIndex = items.length - 1;
      event.preventDefault();
      items[targetIndex].focus();
      items[targetIndex].click();
      scheduleSync();
    });
  }

  function syncDialogs() {
    document.querySelectorAll('.modal').forEach((modal, index) => {
      modal.setAttribute('role', 'dialog');
      modal.setAttribute('aria-modal', 'true');
      observeModal(modal);
      const heading = modal.querySelector('h1, h2, h3, [class*="title"]');
      if (heading) {
        if (!heading.id) heading.id = `hysim-dialog-title-${index + 1}`;
        modal.setAttribute('aria-labelledby', heading.id);
      }
      const visible = global.getComputedStyle(modal).display !== 'none' && !modal.hidden;
      modal.setAttribute('aria-hidden', visible ? 'false' : 'true');
    });
  }

  // ---- Modal focus trap -----------------------------------------------------
  // While a `.modal[role="dialog"]` is visible, Tab / Shift+Tab cycle through
  // its focusable elements and can never leave the dialog. Esc handling stays
  // with each modal's own logic (not duplicated here). When the modal closes,
  // focus returns to the element that had it before opening — but only when
  // focus is still stranded inside the closing modal (or on <body>); modals
  // like the help panel that restore focus themselves are left alone.
  // Attach/detach is driven by the MutationObserver → scheduleSync →
  // syncNavigation pipeline; `trapMemory` records the pre-open focus per
  // modal without pinning the modal element (WeakMap).
  const FOCUSABLE_SELECTOR =
    'a[href], button:not([disabled]), input:not([disabled]), ' +
    'select:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';
  const trapMemory = new WeakMap();   // modal → { restoreFocus }
  const observedModals = new WeakSet();
  let activeTrapModal = null;

  function isModalOpen(modal) {
    return !modal.hidden && global.getComputedStyle(modal).display !== 'none';
  }

  function focusableIn(modal) {
    return Array.from(modal.querySelectorAll(FOCUSABLE_SELECTOR)).filter(isVisible);
  }

  function attachTrap(modal) {
    if (activeTrapModal === modal) return;
    if (activeTrapModal) detachTrap(activeTrapModal);
    activeTrapModal = modal;
    trapMemory.set(modal, { restoreFocus: document.activeElement || null });
  }

  function detachTrap(modal) {
    if (activeTrapModal !== modal) return;
    activeTrapModal = null;
    const memory = trapMemory.get(modal);
    trapMemory.delete(modal);
    const previous = memory && memory.restoreFocus;
    if (!previous || typeof previous.focus !== 'function') return;
    const active = document.activeElement;
    const stranded = !active || active === document.body || modal.contains(active);
    if (stranded && document.contains(previous)) previous.focus();
  }

  function syncFocusTraps() {
    const open = Array.from(document.querySelectorAll('.modal')).filter(isModalOpen);
    if (activeTrapModal && open.includes(activeTrapModal)) return;  // still open
    if (activeTrapModal) detachTrap(activeTrapModal);
    if (open.length) attachTrap(open[open.length - 1]);
  }

  function handleTrapKeydown(event) {
    if (!activeTrapModal || event.key !== 'Tab') return;
    const items = focusableIn(activeTrapModal);
    if (!items.length) { event.preventDefault(); return; }
    const first = items[0];
    const last = items[items.length - 1];
    const active = document.activeElement;
    const inside = !!active && activeTrapModal.contains(active);
    if (event.shiftKey) {
      if (!inside || active === first) { event.preventDefault(); last.focus(); }
    } else if (!inside || active === last) {
      event.preventDefault();
      first.focus();
    }
  }

  function observeModal(modal) {
    if (!observer || observedModals.has(modal)) return;
    observedModals.add(modal);
    observer.observe(modal, {
      attributes: true, attributeFilter: ['class', 'hidden', 'style'],
    });
  }

  function syncNavigation() {
    const seen = new Set();
    TABLISTS.forEach(([containerSelector, itemSelector]) => {
      document.querySelectorAll(containerSelector).forEach(container => {
        if (seen.has(container)) return;
        seen.add(container);
        setupTablist(container, itemSelector);
      });
    });
    syncDialogs();
    syncFocusTraps();
  }

  function scheduleSync() {
    if (syncPending) return;
    syncPending = true;
    requestAnimationFrame(() => {
      syncPending = false;
      syncNavigation();
    });
  }

  function audit() {
    const issues = [];
    const ids = new Map();
    document.querySelectorAll('[id]').forEach(element => {
      const count = (ids.get(element.id) || 0) + 1;
      ids.set(element.id, count);
      if (count === 2) issues.push({ severity: 'critical', code: 'duplicate-id', target: element.id });
    });
    document.querySelectorAll('button').forEach(button => {
      if (!accessibleName(button)) issues.push({ severity: 'critical', code: 'unnamed-button', target: button.id || button.className });
    });
    [['#workflowBar', 'workflow-tablist'], ['#moduleBar', 'module-tablist']].forEach(([selector, code]) => {
      if (document.querySelector(selector)?.getAttribute('role') !== 'tablist') {
        issues.push({ severity: 'critical', code, target: selector });
      }
    });
    if (document.getElementById('main')?.getAttribute('role') !== 'main') {
      issues.push({ severity: 'critical', code: 'main-landmark', target: '#main' });
    }
    if (!document.querySelector('.skip-link[href="#main"]')) {
      issues.push({ severity: 'critical', code: 'skip-link', target: 'body' });
    }
    return {
      schema: 'hysim_accessibility_audit_v1',
      critical: issues.filter(issue => issue.severity === 'critical').length,
      issues,
      managed_tablists: document.querySelectorAll('[role="tablist"]').length,
      managed_dialogs: document.querySelectorAll('.modal[role="dialog"]').length,
    };
  }

  function init() {
    if (initialized) return audit();
    initialized = true;
    const main = document.getElementById('main');
    if (main) { main.setAttribute('role', 'main'); main.tabIndex = -1; }
    const canvas = document.getElementById('canvas');
    if (canvas) {
      canvas.setAttribute('aria-label',
        '交直流混合系统单线图画布（选中元件后可用方向键按网格移动，Shift+方向键按 1px 微调）');
      canvas.tabIndex = 0;
    }
    const consoleLog = document.getElementById('consoleLog');
    if (consoleLog) {
      consoleLog.setAttribute('role', 'log');
      consoleLog.setAttribute('aria-live', 'polite');
      consoleLog.setAttribute('aria-relevant', 'additions');
    }
    syncNavigation();
    observer = new MutationObserver(scheduleSync);
    const navigationRoots = new Set();
    TABLISTS.forEach(([selector]) => {
      document.querySelectorAll(selector).forEach(element => navigationRoots.add(element));
    });
    navigationRoots.forEach(element => observer.observe(element, {
      subtree: true, attributes: true, attributeFilter: ['class', 'hidden'],
    }));
    document.querySelectorAll('.modal').forEach(observeModal);
    // Modals added after init: watch direct body children (modals are
    // top-level elements) and attach the per-modal attribute observer from
    // syncDialogs on the next scheduled sync.
    if (document.body) observer.observe(document.body, { childList: true });
    // Focus trap: one capture-phase listener is enough — it acts only while
    // `activeTrapModal` is set and only on Tab.
    document.addEventListener('keydown', handleTrapKeydown, true);
    return audit();
  }

  core.Accessibility = Object.freeze({
    schema: 'hysim_accessibility_v1',
    init,
    syncNavigation,
    syncFocusTraps,
    audit,
  });
})(window);
