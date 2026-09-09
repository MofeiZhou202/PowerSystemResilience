/**
 * One-line model store (P0) — lossless adapter between the authored
 * HybridPowerSystem JSON (`_system_json`) and a busbar-centric relational model,
 * plus the embedded-SQLite schema for persistence.
 *
 * Design contract: docs/developer/gui_one_line_model_store.md
 *
 * Connectivity registry below is GROUNDED (verified in web/js/components.js
 * defaults and tests/run_gui_server.cpp), not guessed:
 *   1-port devices  -> field `bus`
 *   ac/dc branches, switches, breakers -> `from_bus` / `to_bus`
 *   transformer_2w  -> `hv_bus` / `lv_bus`
 *   transformer_3w  -> `hv_bus` / `mv_bus` / `lv_bus`
 *   vsc/lcc         -> `bus_ac` / `bus_dc`
 *   dcdc            -> `bus_in` / `bus_out`
 *   energy_router   -> multi-port (stored losslessly, connectivity unresolved)
 *
 * Round-trip guarantee: exportSystemJson(importSystemJson(sys)) is byte-stable
 * against `sys` after canonical key sort (see the Node test
 * tests/e2e/one_line_store_roundtrip.mjs).
 */
'use strict';
(function (global) {
  const core = global.HySimCore = global.HySimCore || {};
  const SCHEMA_VERSION = 1;

  const COLLECTIONS = {
    // AC 1-port devices (field: bus)
    'ac.generators':        { role:'device', domain:'ac', kind:'generator' },
    'ac.loads':             { role:'device', domain:'ac', kind:'load' },
    'ac.external_grids':    { role:'device', domain:'ac', kind:'external_grid' },
    'ac.storage':           { role:'device', domain:'ac', kind:'storage' },
    'ac.pv_systems':        { role:'device', domain:'ac', kind:'pv_system' },
    'ac.renewable_gens':    { role:'device', domain:'ac', kind:'renewable_gen' },
    'ac.static_generators': { role:'device', domain:'ac', kind:'static_generator' },
    'ac.motors':            { role:'device', domain:'ac', kind:'motor' },
    'ac.flexible_loads':    { role:'device', domain:'ac', kind:'flexible_load' },
    'ac.asymmetric_loads':  { role:'device', domain:'ac', kind:'asymmetric_load' },
    'ac.shunts':            { role:'device', domain:'ac', kind:'shunt' },
    'ac.chargers':          { role:'device', domain:'ac', kind:'charger' },
    'ac.charging_stations': { role:'device', domain:'ac', kind:'charging_station' },
    // DC 1-port devices (field: bus)
    'dc.loads':             { role:'device', domain:'dc', kind:'dc_load' },
    'dc.pv_arrays':         { role:'device', domain:'dc', kind:'dc_pv_array' },
    'dc.dc_storage':        { role:'device', domain:'dc', kind:'dc_storage' },
    'dc.static_generators': { role:'device', domain:'dc', kind:'static_generator' },
    // Top-level 1-port aggregates (field: bus)
    'mobile_storage':       { role:'device', domain:'ac', kind:'mobile_storage' },
    'vpps':                 { role:'device', domain:'ac', kind:'vpp' },
    'microgrids':           { role:'device', domain:'ac', kind:'microgrid' },
    // 2-port links
    'ac.branches':          { role:'link', kind:'ac_branch',       ends:[['ac','from_bus'],['ac','to_bus']] },
    'dc.branches':          { role:'link', kind:'dc_branch',       ends:[['dc','from_bus'],['dc','to_bus']] },
    'ac.switches':          { role:'link', kind:'switch_comp',     ends:[['ac','from_bus'],['ac','to_bus']] },
    'ac.circuit_breakers':  { role:'link', kind:'circuit_breaker', ends:[['ac','from_bus'],['ac','to_bus']] },
    'ac.transformers_2w':   { role:'link', kind:'transformer_2w',  ends:[['ac','hv_bus'],['ac','lv_bus']] },
    'vsc_converters':       { role:'link', kind:'vsc_converter',   ends:[['ac','bus_ac'],['dc','bus_dc']] },
    'lcc_converters':       { role:'link', kind:'lcc_converter',   ends:[['ac','bus_ac'],['dc','bus_dc']] },
    'dcdc_converters':      { role:'link', kind:'dcdc_converter',  ends:[['dc','bus_in'],['dc','bus_out']] },
    // 3-port link
    'ac.transformers_3w':   { role:'link', kind:'transformer_3w',  ends:[['ac','hv_bus'],['ac','mv_bus'],['ac','lv_bus']] },
    // Multi-port / special (lossless; connectivity intentionally unresolved)
    'energy_routers':       { role:'special', kind:'energy_router' },
  };

  // Embedded-SQLite DDL. `params` blobs guarantee lossless round-trip while the
  // structural columns support queries and diagram geometry.
  const SCHEMA_SQL = `
CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT);
CREATE TABLE IF NOT EXISTS bus (
  pk INTEGER PRIMARY KEY, domain TEXT NOT NULL, idx INTEGER, name TEXT,
  base_kv REAL, x REAL, y REAL, length REAL, orient TEXT DEFAULT 'h',
  ord INTEGER NOT NULL, params TEXT NOT NULL, UNIQUE(domain, idx));
CREATE TABLE IF NOT EXISTS device (
  pk INTEGER PRIMARY KEY, collection TEXT NOT NULL, kind TEXT NOT NULL,
  idx INTEGER, name TEXT, bus_domain TEXT, bus_idx INTEGER,
  tap_frac REAL, side TEXT, ord INTEGER NOT NULL, params TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS link (
  pk INTEGER PRIMARY KEY, collection TEXT NOT NULL, kind TEXT NOT NULL,
  idx INTEGER, name TEXT, ends TEXT NOT NULL, channel REAL,
  ord INTEGER NOT NULL, params TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS special (
  pk INTEGER PRIMARY KEY, collection TEXT NOT NULL, kind TEXT NOT NULL,
  ord INTEGER NOT NULL, params TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS skeleton (id INTEGER PRIMARY KEY CHECK(id=1), json TEXT NOT NULL);
`;

  // ---- dotted-path helpers (support 'ac.branches' and 'vsc_converters') ----
  const clone = (o) => (typeof structuredClone === 'function'
    ? structuredClone(o) : JSON.parse(JSON.stringify(o)));
  function getPath(obj, path){ return path.split('.').reduce((o,k)=> (o==null?undefined:o[k]), obj); }
  function deletePath(obj, path){
    const ks = path.split('.'); const last = ks.pop();
    const parent = ks.reduce((o,k)=> (o==null?undefined:o[k]), obj);
    if (parent && typeof parent === 'object') delete parent[last];
  }
  function setPath(obj, path, val){
    const ks = path.split('.'); const last = ks.pop();
    let cur = obj;
    for (const k of ks){ if (cur[k] == null || typeof cur[k] !== 'object') cur[k] = {}; cur = cur[k]; }
    cur[last] = val;
  }

  // ---- JSON -> relational -------------------------------------------------
  function importSystemJson(sys){
    if (!sys || typeof sys !== 'object') throw new Error('one_line_store: system JSON must be an object');
    const skel = clone(sys);
    const buses = [], devices = [], links = [], specials = [];

    ['ac','dc'].forEach(dom => {
      const arr = skel[dom] && skel[dom].buses;
      if (Array.isArray(arr)){
        arr.forEach((b,i)=> buses.push({
          domain:dom, idx: numOrNull(b.index), name: b.name ?? null,
          base_kv: numOrNull(b.base_kv), ord:i, params:b }));
        if (arr.length) delete skel[dom].buses;   // keep empty arrays in the skeleton
      }
    });

    for (const [path, desc] of Object.entries(COLLECTIONS)){
      const arr = getPath(skel, path);
      if (!Array.isArray(arr)) continue;
      arr.forEach((rec,i)=>{
        if (desc.role === 'device'){
          devices.push({ collection:path, kind:desc.kind, idx: numOrNull(rec.index),
            name: rec.name ?? null, bus_domain:desc.domain, bus_idx: numOrNull(rec.bus),
            tap_frac:null, side:null, ord:i, params:rec });
        } else if (desc.role === 'link'){
          const ends = desc.ends.map(([edom, field]) => ({ domain:edom, idx: numOrNull(rec[field]) }));
          links.push({ collection:path, kind:desc.kind, idx: numOrNull(rec.index),
            name: rec.name ?? null, ends, channel:null, ord:i, params:rec });
        } else {
          specials.push({ collection:path, kind:desc.kind, ord:i, params:rec });
        }
      });
      if (arr.length) deletePath(skel, path);   // keep empty arrays in the skeleton
    }

    const meta = { name: sys.name ?? '', base_mva: numOr(sys.base_mva, 1), schema_version: SCHEMA_VERSION };
    return { meta, buses, devices, links, specials, skeleton: skel };
  }

  // ---- relational -> JSON (exact inverse over canonical sort) --------------
  function exportSystemJson(store){
    const sys = clone(store.skeleton || {});
    const byDom = { ac:[], dc:[] };
    store.buses.slice().sort(ordCmp).forEach(r => { (byDom[r.domain] || (byDom[r.domain]=[])).push(r.params); });
    ['ac','dc'].forEach(dom => { if (byDom[dom].length || getPath(sys, dom)) setPath(sys, dom+'.buses', byDom[dom]); });

    const groups = new Map();
    const push = (r) => { if (!groups.has(r.collection)) groups.set(r.collection, []); groups.get(r.collection).push(r); };
    store.devices.forEach(push); store.links.forEach(push); store.specials.forEach(push);
    for (const [path, rows] of groups){
      setPath(sys, path, rows.slice().sort(ordCmp).map(r => r.params));
    }
    return sys;
  }

  // ---- relational -> busbar geometry (editor/prototype model) --------------
  // Voltage levels stack vertically (ETAP convention); taps distribute along
  // each bar; sources go above, loads/storage below.
  function toBusbarModel(store, opt = {}){
    const gapY = opt.levelGap || 250, x0 = opt.x0 || 160, padX = opt.padX || 90;
    const kvs = [...new Set(store.buses.map(b => b.base_kv ?? 0))].sort((a,b)=> b-a);
    const levelY = new Map(kvs.map((kv,i)=> [kv, 120 + i*gapY]));
    const busKey = (dom, idx) => dom + ':' + idx;

    const homeCount = new Map();
    store.devices.forEach(d => { const k = busKey(d.bus_domain, d.bus_idx);
      homeCount.set(k, (homeCount.get(k)||0) + 1); });
    store.links.forEach(l => l.ends.forEach(e => { const k = busKey(e.domain, e.idx);
      homeCount.set(k, (homeCount.get(k)||0) + 1); }));

    const buses = store.buses.map(b => {
      const k = busKey(b.domain, b.idx), n = Math.max(1, homeCount.get(k) || 1);
      return { id:k, domain:b.domain, idx:b.idx, name: b.name || ((b.domain==='dc'?'DC-':'BUS-') + b.idx),
        kv: b.base_kv ?? 0, x: x0, y: levelY.get(b.base_kv ?? 0) || 120,
        length: Math.max(220, padX * n), orient:'h' };
    });
    // Order buses left-to-right within a level so bars do not overlap.
    const perLevel = new Map();
    buses.forEach(b => { const arr = perLevel.get(b.y) || []; arr.push(b); perLevel.set(b.y, arr); });
    perLevel.forEach(arr => { let cx = x0; arr.forEach(b => { b.x = cx; cx += b.length + 120; }); });

    const tapCursor = new Map();
    const nextFrac = (k) => { const arr = tapCursor.get(k) || []; return arr; };
    const busById = new Map(buses.map(b => [b.id, b]));
    const spread = (k) => { // even fractions along the bar
      const home = store.devices.filter(d => busKey(d.bus_domain,d.bus_idx)===k).length
                 + store.links.filter(l => l.ends.some(e => busKey(e.domain,e.idx)===k)).length;
      return Math.max(1, home);
    };
    const counter = new Map();
    const takeFrac = (k) => { const total = spread(k); const i = counter.get(k) || 0;
      counter.set(k, i+1); return (i + 1) / (total + 1); };

    const SRC = new Set(['generator','external_grid','renewable_gen','static_generator','pv_system','dc_pv_array']);
    const devices = store.devices.filter(d => busById.has(busKey(d.bus_domain,d.bus_idx))).map(d => ({
      id: d.collection + '#' + d.ord, kind: d.kind, name: d.name || d.kind,
      busId: busKey(d.bus_domain, d.bus_idx), t: takeFrac(busKey(d.bus_domain,d.bus_idx)),
      side: SRC.has(d.kind) ? 'above' : 'below', rating: '' }));

    const links = store.links.filter(l => l.ends.length >= 2
      && busById.has(busKey(l.ends[0].domain,l.ends[0].idx))
      && busById.has(busKey(l.ends[1].domain,l.ends[1].idx))).map(l => {
      const a = busById.get(busKey(l.ends[0].domain,l.ends[0].idx));
      return { id: l.collection + '#' + l.ord, kind: l.kind, name: l.name || l.kind,
        fromBusId: a.id, toBusId: busById.get(busKey(l.ends[1].domain,l.ends[1].idx)).id,
        channelX: a.x + a.length/2, rating: '' };
    });
    return { buses, devices, links };
  }

  // ---- optional sql.js binding (executes the DDL when a db is provided) ----
  function saveToDb(db, store){
    db.exec(SCHEMA_SQL);
    db.run('DELETE FROM meta; DELETE FROM bus; DELETE FROM device; DELETE FROM link; DELETE FROM special; DELETE FROM skeleton;');
    Object.entries(store.meta).forEach(([k,v]) => db.run('INSERT INTO meta(key,value) VALUES (?,?)', [k, String(v)]));
    store.buses.forEach(b => db.run(
      'INSERT INTO bus(domain,idx,name,base_kv,x,y,length,orient,ord,params) VALUES (?,?,?,?,?,?,?,?,?,?)',
      [b.domain, b.idx, b.name, b.base_kv, b.x??null, b.y??null, b.length??null, b.orient??'h', b.ord, JSON.stringify(b.params)]));
    store.devices.forEach(d => db.run(
      'INSERT INTO device(collection,kind,idx,name,bus_domain,bus_idx,tap_frac,side,ord,params) VALUES (?,?,?,?,?,?,?,?,?,?)',
      [d.collection, d.kind, d.idx, d.name, d.bus_domain, d.bus_idx, d.tap_frac, d.side, d.ord, JSON.stringify(d.params)]));
    store.links.forEach(l => db.run(
      'INSERT INTO link(collection,kind,idx,name,ends,channel,ord,params) VALUES (?,?,?,?,?,?,?,?)',
      [l.collection, l.kind, l.idx, l.name, JSON.stringify(l.ends), l.channel, l.ord, JSON.stringify(l.params)]));
    store.specials.forEach(s => db.run(
      'INSERT INTO special(collection,kind,ord,params) VALUES (?,?,?,?)',
      [s.collection, s.kind, s.ord, JSON.stringify(s.params)]));
    db.run('INSERT INTO skeleton(id,json) VALUES (1,?)', [JSON.stringify(store.skeleton)]);
  }
  function readFromDb(db){
    const rows = (sql) => { const r = db.exec(sql); if (!r.length) return [];
      const { columns, values } = r[0]; return values.map(v => Object.fromEntries(columns.map((c,i)=>[c, v[i]]))); };
    const meta = {}; rows('SELECT key,value FROM meta').forEach(m => meta[m.key] = m.value);
    meta.base_mva = numOr(meta.base_mva, 1); meta.schema_version = numOr(meta.schema_version, SCHEMA_VERSION);
    const buses = rows('SELECT * FROM bus ORDER BY ord').map(r => ({ ...r, params: JSON.parse(r.params) }));
    const devices = rows('SELECT * FROM device ORDER BY ord').map(r => ({ ...r, params: JSON.parse(r.params) }));
    const links = rows('SELECT * FROM link ORDER BY ord').map(r => ({ ...r, ends: JSON.parse(r.ends), params: JSON.parse(r.params) }));
    const specials = rows('SELECT * FROM special ORDER BY ord').map(r => ({ ...r, params: JSON.parse(r.params) }));
    const sk = rows('SELECT json FROM skeleton WHERE id=1');
    return { meta, buses, devices, links, specials, skeleton: sk.length ? JSON.parse(sk[0].json) : {} };
  }

  // ---- canonicalisation (used by the round-trip acceptance test) ----------
  function canonical(o){
    if (Array.isArray(o)) return o.map(canonical);
    if (o && typeof o === 'object') return Object.keys(o).sort().reduce((a,k)=>{ a[k]=canonical(o[k]); return a; }, {});
    return o;
  }

  const numOr = (v,d) => { const n = Number(v); return Number.isFinite(n) ? n : d; };
  const numOrNull = (v) => { const n = Number(v); return Number.isFinite(n) ? n : (v ?? null); };
  const ordCmp = (a,b) => (a.ord|0) - (b.ord|0);

  const api = { SCHEMA_SQL, SCHEMA_VERSION, COLLECTIONS,
    importSystemJson, exportSystemJson, toBusbarModel, saveToDb, readFromDb, canonical };
  core.OneLineStore = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
