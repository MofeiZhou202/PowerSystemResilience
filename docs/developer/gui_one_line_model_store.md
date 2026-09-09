# One-Line Model Store (P0) — SQLite schema & JSON adapter

> Implementation contract for phase **P0** of the one-line editor redesign
> ([proposal](../planning/gui_one_line_redesign.md)). Describes shipped behavior.
> Code: `web/js/core/one_line_store.js`. Test: `tests/e2e/one_line_store_roundtrip.mjs`.

## Purpose

Turn the authored `HybridPowerSystem` JSON (`_system_json`) into a busbar-centric
relational model that (a) round-trips losslessly, (b) persists to embedded
SQLite, and (c) projects to diagram geometry. **No backend change**: the adapter
consumes and reproduces the same JSON the C++ `run_gui_server` already emits and
ingests (`Canvas.loadFromSystemJson`).

## Connectivity registry (grounded, not guessed)

Verified against `web/js/components.js` defaults and the authoritative
`tests/run_gui_server.cpp`:

| collection(s) | role | connection field(s) | domain(s) |
|---|---|---|---|
| `ac.buses`, `dc.buses` | bus | `index` | ac / dc |
| `ac.generators/loads/external_grids/storage/pv_systems/renewable_gens/static_generators/motors/flexible_loads/asymmetric_loads/shunts/chargers/charging_stations` | device | `bus` | ac |
| `dc.loads/pv_arrays/dc_storage/static_generators` | device | `bus` | dc |
| `mobile_storage`, `vpps`, `microgrids` | device | `bus` | ac |
| `ac.branches`, `dc.branches`, `ac.switches`, `ac.circuit_breakers` | link | `from_bus`,`to_bus` | ac/ac, dc/dc |
| `ac.transformers_2w` | link | `hv_bus`,`lv_bus` | ac/ac |
| `ac.transformers_3w` | link (3-port) | `hv_bus`,`mv_bus`,`lv_bus` | ac |
| `vsc_converters`, `lcc_converters` | link | `bus_ac`,`bus_dc` | ac/dc |
| `dcdc_converters` | link | `bus_in`,`bus_out` | dc/dc |
| `energy_routers` | special | multi-port | — (unresolved) |

Honesty: `energy_router` connectivity is **not** fabricated — its record is kept
verbatim in the `special` table and its diagram connections are marked
unresolved. Any collection not in the registry is preserved untouched in the
`skeleton` (see below), so round-trip is lossless even for unknown data.

## Relational schema (embedded SQLite)

DDL is `OneLineStore.SCHEMA_SQL`. Every row keeps the **full original record** in
a `params` JSON column (guarantees losslessness); structural columns support
queries and geometry.

- `meta(key,value)` — `name`, `base_mva`, `schema_version`.
- `bus(domain, idx, name, base_kv, x, y, length, orient, ord, params)` — one row
  per busbar; `UNIQUE(domain, idx)` enforces the domain-qualified identity rule.
- `device(collection, kind, idx, name, bus_domain, bus_idx, tap_frac, side, ord, params)`
  — one row per 1-port device, resolved to its home bus.
- `link(collection, kind, idx, name, ends, channel, ord, params)` — one row per
  2/3-port element; `ends` is a JSON array of `{domain, idx}` taps.
- `special(collection, kind, ord, params)` — multi-port/unresolved elements.
- `skeleton(json)` — the original JSON minus every consumed collection (retains
  scalars like `name`/`base_mva` and any collection outside the registry, and
  **empty collections stay here** so their presence survives the round-trip).

## API (`OneLineStore`)

- `importSystemJson(sys)` → `{ meta, buses, devices, links, specials, skeleton }`.
- `exportSystemJson(store)` → `sys` (exact inverse under canonical key sort).
- `toBusbarModel(store, opt?)` → `{ buses, devices, links }` diagram geometry:
  voltage levels stack vertically (ETAP convention), taps distribute evenly along
  each bar, sources dock above and loads/storage below.
- `saveToDb(db, store)` / `readFromDb(db)` — optional [sql.js](https://sql.js.org)
  binding; runs `SCHEMA_SQL` and round-trips through SQLite when a db is provided.
- `canonical(obj)` — recursive key sort, used by the acceptance test.

## Acceptance (met)

`node tests/e2e/one_line_store_roundtrip.mjs` asserts
`canonical(exportSystemJson(importSystemJson(sys))) === canonical(sys)` for every
bundled example. Current result:

```
PASS  ac_radial_feeder_example.json     relational: buses=5 devices=4 links=4     busbar: bars=5 taps=4 links=4
PASS  hybrid_acdc_microgrid_example.json relational: buses=3 devices=6 links=2     busbar: bars=3 taps=6 links=2
```

## Limitations

- `tap_frac`/`side`/`channel` are geometry hints, not authored electrical data;
  they are recomputed by `toBusbarModel` and are not part of the round-trip
  invariant.
- SQLite persistence is proven both in-browser (vendored sql.js, Save/Open
  `.sqlite` in `web/prototype/one_line_redesign.html`) and on disk
  (`tests/e2e/one_line_store_sqlite.mjs` writes a real `.sqlite` file, confirmed
  readable by the system `sqlite3` CLI, and round-trips). Making `run_gui_server`
  the authoritative owner of that file (flushed via the session-save path) is P3.

## App integration (Save/Open `.sqlite`)

The store is wired into the production GUI (`web/index.html` toolbar: **保存 DB**
/ **打开 DB**, handlers in `web/js/app.js`):

- **Save** — `Canvas.buildSystemJson()` → `OneLineStore.importSystemJson` →
  `saveToDb(new SQL.Database(), rel)` → `db.export()` → binary download of
  `<name>.sqlite`.
- **Open** — file input → `new SQL.Database(new Uint8Array(bytes))` →
  `readFromDb` → `exportSystemJson` → the shared `App.importSystemJson` load path
  (backend `load_json_string` + `Canvas.loadFromSystemJson`), so the `_canvas`
  layout block round-trips and positions are restored exactly.

sql.js is vendored (`web/vendor/sql-wasm.{js,wasm}`, MIT — see `SQLJS-LICENSE`);
`initSqlJs` is loaded once and memoized, and the `.wasm` is fetched lazily on the
first Save/Open via `locateFile: f => 'vendor/' + f`. End-to-end coverage:
`tests/e2e/one_line_store_app_e2e.mjs` (11/11) verifies real `SQLite format 3`
bytes, a byte-stable round-trip through a real `.sqlite`, and that the Open button
reloads the model losslessly with zero page errors. Server-authoritative
ownership of the file remains future work.
