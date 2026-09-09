/**
 * P0 persistence test — real SQLite round-trip via the vendored sql.js.
 *
 * Proves OneLineStore.SCHEMA_SQL + saveToDb/readFromDb work against an actual
 * on-disk .sqlite file (the format the C++ run_gui_server would own), and that
 * exportSystemJson after a SQLite round-trip still equals the source model.
 *
 * Run:  node tests/e2e/one_line_store_sqlite.mjs
 * Deps: web/vendor/sql-wasm.js + sql-wasm.wasm (vendored). Uses the system
 *       `sqlite3` CLI only to independently confirm the file is a real DB.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, join } from 'node:path';
import { createRequire } from 'node:module';
import { execFileSync } from 'node:child_process';
import { tmpdir } from 'node:os';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..', '..');
const vendor = join(root, 'web', 'vendor');
const require = createRequire(import.meta.url);

await import(pathToFileURL(join(root, 'web/js/core/one_line_store.js')).href);
const S = globalThis.HySimCore.OneLineStore;

const initSqlJs = require(join(vendor, 'sql-wasm.js'));
const SQL = await initSqlJs({ locateFile: () => join(vendor, 'sql-wasm.wasm') });

const examples = [
  'web/examples/ac_radial_feeder_example.json',
  'web/examples/hybrid_acdc_microgrid_example.json',
];

let failures = 0;
for (const rel of examples) {
  const name = rel.split('/').pop();
  const sys = JSON.parse(readFileSync(join(root, rel), 'utf8'));
  const store = S.importSystemJson(sys);

  // write -> real .sqlite file on disk
  const db = new SQL.Database();
  S.saveToDb(db, store);
  const bytes = db.export();
  const file = join(tmpdir(), `hysim_${name}.sqlite`);
  writeFileSync(file, Buffer.from(bytes));
  db.close();

  // independent check: the system sqlite3 CLI can read the tables/rows
  const tables = execFileSync('sqlite3', [file, "SELECT count(*) FROM sqlite_master WHERE type='table'"]).toString().trim();
  const busCount = execFileSync('sqlite3', [file, 'SELECT count(*) FROM bus']).toString().trim();

  // read back -> export -> compare
  const db2 = new SQL.Database(readFileSync(file));
  const store2 = S.readFromDb(db2);
  db2.close();
  const back = S.exportSystemJson(store2);
  const ok = JSON.stringify(S.canonical(sys)) === JSON.stringify(S.canonical(back));
  if (!ok) failures++;

  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}`);
  console.log(`      file=${file}  tables=${tables}  bus_rows=${busCount}  (sqlite3 CLI verified)`);
}

console.log(failures ? `\n${failures} example(s) FAILED SQLite round-trip` : '\nAll examples round-trip through a real .sqlite file.');
process.exit(failures ? 1 : 0);
