/**
 * P0 acceptance test — lossless JSON <-> relational round-trip for the one-line
 * model store (web/js/core/one_line_store.js).
 *
 * Runs with plain Node (no deps):   node tests/e2e/one_line_store_roundtrip.mjs
 * Passes when exportSystemJson(importSystemJson(sys)) equals `sys` after
 * canonical key sort, for every bundled example model.
 */
import { readFileSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..', '..');
await import(pathToFileURL(join(root, 'web/js/core/one_line_store.js')).href);
const S = globalThis.HySimCore.OneLineStore;

const examples = [
  'web/examples/ac_radial_feeder_example.json',
  'web/examples/hybrid_acdc_microgrid_example.json',
];

let failures = 0;
for (const rel of examples) {
  const sys = JSON.parse(readFileSync(join(root, rel), 'utf8'));
  const store = S.importSystemJson(sys);
  const back = S.exportSystemJson(store);
  const a = JSON.stringify(S.canonical(sys));
  const b = JSON.stringify(S.canonical(back));
  const ok = a === b;
  if (!ok) {
    failures++;
    // Report the first divergent 80-char window to aid debugging.
    let i = 0; while (i < a.length && a[i] === b[i]) i++;
    console.error(`  diff near index ${i}:\n   sys : …${a.slice(Math.max(0,i-40), i+40)}…\n   back: …${b.slice(Math.max(0,i-40), i+40)}…`);
  }
  const bm = S.toBusbarModel(store);
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${rel.split('/').pop()}`);
  console.log(`      relational: buses=${store.buses.length} devices=${store.devices.length} links=${store.links.length} specials=${store.specials.length}`);
  console.log(`      busbar:     bars=${bm.buses.length} taps=${bm.devices.length} links=${bm.links.length}`);
}

console.log(failures ? `\n${failures} example(s) FAILED round-trip` : '\nAll examples round-trip byte-stable (canonical).');
process.exit(failures ? 1 : 0);
