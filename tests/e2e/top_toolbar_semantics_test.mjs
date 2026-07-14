import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const html = readFileSync(path.join(root, 'web/index.html'), 'utf8');

const topStart = html.indexOf('<div id="topToolbar"');
const topEnd = html.indexOf('<!-- ===== Bar 2a:', topStart);
assert.ok(topStart >= 0 && topEnd > topStart, 'top toolbar region must exist');
const topToolbar = html.slice(topStart, topEnd);
assert.doesNotMatch(topToolbar, /id="btnOpenCostEditor"/,
  'model cost editing does not belong in the canvas design toolbar');

const parameterStart = html.indexOf('data-sub="parameterLibrary"');
const parameterEnd = html.indexOf('<!-- Power flow -->', parameterStart);
assert.ok(parameterStart >= 0 && parameterEnd > parameterStart,
  'model parameter toolbar region must exist');
const parameterToolbar = html.slice(parameterStart, parameterEnd);
assert.match(parameterToolbar, /经济参数[\s\S]*id="btnOpenCostEditor"[\s\S]*成本参数/,
  'cost editing belongs to the model parameter workflow');

const costEntryCount = [...html.matchAll(/id="btnOpenCostEditor"/g)].length;
assert.equal(costEntryCount, 1, 'the cost editor must have one unambiguous entry point');
assert.match(html, /id="costEditorModal"/);

console.log('top toolbar semantic contract passed');
