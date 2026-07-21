import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const html = readFileSync(path.join(root, 'web/index.html'), 'utf8');
const app = readFileSync(path.join(root, 'web/js/app.js'), 'utf8');
const css = readFileSync(path.join(root, 'web/css/style.css'), 'utf8');

assert.match(html, /data-module="weakLinks"[^>]*>多维薄弱环节</);
assert.match(html, /class="sub-section" data-sub="weakLinks" hidden/);
assert.match(html, /name="weakDecisionMode" value="planning" checked/);
assert.match(html, /name="weakDecisionMode" value="operation"/);
assert.match(html, /option value="complete" selected>自动补齐/);
for (const id of [
  'weakEvidenceMode', 'weakComponentGroup', 'weakMinimumDimensions', 'weakEconomicThreshold',
  'weakCarbonThreshold', 'weakReliabilityThreshold', 'weakResilienceThreshold',
  'weakTopK', 'btnRunWeakLinks', 'btnExportWeakLinks', 'weakLinkSummary',
  'weakLinkSpatialChart', 'weakLinkTemporalChart', 'weakLinkEntityResults',
  'weakLinkPeriodResults',
  'cfEnableLine', 'cfEnableStorage', 'cfEnableTie', 'cfEnableAutomation',
  'cfEnableDer', 'cfMaxMeasures', 'cfMaxPairs', 'cfLineFactor',
  'cfStoragePower', 'cfStorageDuration', 'cfDerPower', 'cfDerCapacityFactor',
  'cfTieCapacity', 'btnRunCounterfactual', 'btnExportCounterfactual',
  'counterfactualBaseline', 'counterfactualBenefitChart',
  'counterfactualMeasureResults', 'counterfactualSynergyMatrix',
]) {
  assert.match(html, new RegExp(`id="${id}"`), `missing #${id}`);
}

const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
const duplicates = ids.filter((id, index) => ids.indexOf(id) !== index);
assert.deepEqual([...new Set(duplicates)], [], 'page must not introduce duplicate ids');

assert.match(app, /function collectWeakLinkEvidence\(\)/);
assert.match(app, /runOpf,\s*runThreePhaseHybridOpf,\s*collectWeakLinkEvidence,/);
assert.match(app, /function weakComponentCatalog\(/);
assert.match(app, /function weakCanonicalCanvasType\(/);
assert.match(app, /function weakComparisonGroup\(/);
assert.match(app, /function showWeakLinkResults\(data\)/);
assert.match(app, /function renderWeakLinkCharts\(data\)/);
assert.match(app, /\/api\/session\/run_multidimensional_weak_links/);
assert.match(app, /\/api\/session\/run_counterfactual_planning/);
assert.match(app, /function renderCounterfactualResults\(data\)/);
for (const dimension of ['economic', 'carbon', 'reliability', 'resilience']) {
  assert.match(app, new RegExp(`\\b${dimension}\\b`));
}
assert.match(css, /data-active-group="weakLinks"/);
assert.match(css, /\.weak-link-visuals/);
assert.match(css, /\.counterfactual-matrix/);
assert.match(app, /function renderWeakPressureMatrix\(/);
assert.doesNotMatch(app, /function renderWeakLinkCharts\(data\) \{\s*if \(typeof Plotly/,
  'core weak-link matrices must render without the optional Plotly CDN');
assert.match(html, /id="weakLinkSpatialChart" class="weak-link-pressure-matrix"/);
assert.match(html, /id="weakLinkTemporalChart" class="weak-link-pressure-matrix"/);
assert.match(css, /\.weak-pressure-matrix-table/);
assert.match(css, /\.weak-pressure-critical/);
for (const group of ['network', 'conversion', 'supply', 'flexibility', 'demand', 'protection']) {
  assert.match(app, new RegExp(`\\b${group}\\b`), `missing component group ${group}`);
}

console.log('multidimensional weak-link GUI contract passed');
