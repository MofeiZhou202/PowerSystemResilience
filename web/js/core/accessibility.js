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
      const heading = modal.querySelector('h1, h2, h3, [class*="title"]');
      if (heading) {
        if (!heading.id) heading.id = `hysim-dialog-title-${index + 1}`;
        modal.setAttribute('aria-labelledby', heading.id);
      }
      const visible = global.getComputedStyle(modal).display !== 'none' && !modal.hidden;
      modal.setAttribute('aria-hidden', visible ? 'false' : 'true');
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
      canvas.setAttribute('aria-label', '交直流混合系统单线图画布');
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
    document.querySelectorAll('.modal').forEach(modal => observer.observe(modal, {
      attributes: true, attributeFilter: ['class', 'hidden', 'style'],
    }));
    return audit();
  }

  core.Accessibility = Object.freeze({
    schema: 'hysim_accessibility_v1',
    init,
    syncNavigation,
    audit,
  });
})(window);
