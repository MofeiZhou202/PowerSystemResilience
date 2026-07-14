import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const source = readFileSync(
  path.join(root, 'web/js/core/analysis_contracts.js'), 'utf8');
const context = { window: {} };
vm.runInNewContext(source, context, { filename: 'analysis_contracts.js' });

const controls = context.window.HySimCore.AnalysisContracts.reliabilityControlState;
assert.equal(typeof controls, 'function');

const initial = controls();
assert.equal(initial.cyberToggleEnabled, true);
assert.equal(initial.cyberParametersVisible, false);

const fmeaOff = controls({ method: 'fmea', cyberEnabled: false });
assert.equal(fmeaOff.useCyberPhysical, true);
assert.equal(fmeaOff.cyberParametersVisible, true);
assert.equal(fmeaOff.cyberParametersEnabled, true);
assert.equal(fmeaOff.cyberEffective, false);

const fmeaOn = controls({ method: 'fmea', cyberEnabled: true });
assert.equal(fmeaOn.cyberEffective, true);
assert.equal(fmeaOn.cyberParametersEnabled, true);

const threeStage = controls({
  physicalModel: 'restoration_milp', method: 'fmea', cyberEnabled: true,
});
assert.equal(threeStage.useThreeStage, true);
assert.equal(threeStage.useCyberPhysical, false);
assert.equal(threeStage.cyberEffective, false);

const html = readFileSync(path.join(root, 'web/index.html'), 'utf8');
assert.match(html, /class="sub-label sub-checkbox rel-cyber-toggle"[^>]*>[\s\S]*?id="relCyberEnabled"/);
assert.doesNotMatch(html, /class="[^"]*rel-cyber-control[^"]*"[^>]*>[\s\S]{0,300}?id="relCyberEnabled"/);

console.log('reliability control-state contract passed');
