/**
 * search_registry.js — pure element-search registry + query parsers.
 *
 * Extracted from app.js (Phase 5 modularization) as the first dependency-free
 * seam: this module has NO DOM and NO application state. It declares the
 * searchable element classes (`SOURCES`) and the pure string/query helpers used
 * by the global element search and the neighborhood sub-diagram. app.js binds
 * these onto local names, so call sites there stay unchanged.
 *
 * Exposed as `window.HACDCSearch`.
 */
'use strict';

window.HACDCSearch = (() => {
  // One entry per searchable element class. `path(sys)` returns the array from a
  // system JSON; `tableId` is the topology table it renders into; `aliases` are
  // matched (normalized) against the query; `idKeys` are the numeric id fields.
  const SOURCES = [
    { bucket: 'ac', tableId: 'busTableInner', label: 'AC母线', domain: 'ac', path: sys => sys.ac?.buses, aliases: ['bus', 'acbus', 'ac_bus', 'node', '母线', '节点'], idKeys: ['index'] },
    { bucket: 'dc', tableId: 'dcBusTableInner', label: 'DC母线', domain: 'dc', path: sys => sys.dc?.buses, aliases: ['bus', 'dcbus', 'dc_bus', 'node', '母线', '节点'], idKeys: ['index'] },
    { bucket: 'branch', tableId: 'branchTableInner', label: 'AC支路', domain: 'ac', path: sys => sys.ac?.branches, aliases: ['branch', 'line', 'acbranch', 'ac_branch', '线路', '支路'], idKeys: ['index', 'id'] },
    { bucket: 'gen', tableId: 'genTableInner', label: '发电机', domain: 'ac', path: sys => sys.ac?.generators, aliases: ['gen', 'generator', 'generators', '发电机', '机组'], idKeys: ['index', 'id'] },
    { bucket: 'load', tableId: 'loadTableInner', label: 'AC负荷', domain: 'ac', path: sys => sys.ac?.loads, aliases: ['load', 'acload', 'ac_load', '负荷'], idKeys: ['index', 'id'] },
    { bucket: 'trafo', tableId: 'trafoTableInner', label: '变压器', domain: 'ac', path: sys => sys.ac?.transformers_2w, aliases: ['trafo', 'transformer', 'transformer_2w', '变压器'], idKeys: ['index', 'id'] },
    { bucket: 'extGrid', tableId: 'extGridTableInner', label: '外部电网', domain: 'ac', path: sys => sys.ac?.external_grids, aliases: ['externalgrid', 'external_grid', 'extgrid', 'ext_grid', 'slack', '外部电网'], idKeys: ['index', 'id'] },
    { bucket: 'storage', tableId: 'storageTableInner', label: 'AC储能', domain: 'ac', path: sys => sys.ac?.storage, aliases: ['storage', 'battery', 'acstorage', 'ac_storage', '储能'], idKeys: ['index', 'id'] },
    { bucket: 'pv', tableId: 'pvTableInner', label: 'AC光伏', domain: 'ac', path: sys => sys.ac?.pv_systems, aliases: ['pv', 'pvsystem', 'pv_system', 'solar', '光伏'], idKeys: ['index', 'id'] },
    { bucket: 'renGen', tableId: 'renGenTableInner', label: '新能源', domain: 'ac', path: sys => sys.ac?.renewable_gens, aliases: ['renewable', 'renewablegen', 'renewable_gen', 'wind', '新能源', '风电'], idKeys: ['index', 'id'] },
    { bucket: 'sgen', tableId: 'sgenTableInner', label: '静态电源', domain: 'ac', path: sys => sys.ac?.static_generators, aliases: ['sgen', 'staticgen', 'static_generator', '静态电源'], idKeys: ['index', 'id'] },
    { bucket: 'shunt', tableId: 'shuntTableInner', label: '并联补偿', domain: 'ac', path: sys => sys.ac?.shunts, aliases: ['shunt', '并联', '补偿'], idKeys: ['index', 'id'] },
    { bucket: 'sw', tableId: 'switchTableInner', label: '开关', domain: 'ac', path: sys => sys.ac?.switches, aliases: ['switch', 'sw', '开关'], idKeys: ['index', 'id'] },
    { bucket: 'cb', tableId: 'cbTableInner', label: '断路器', domain: 'ac', path: sys => sys.ac?.circuit_breakers, aliases: ['breaker', 'circuitbreaker', 'circuit_breaker', 'cb', '断路器'], idKeys: ['index', 'id'] },
    { bucket: 'motor', tableId: 'motorTableInner', label: '电动机', domain: 'ac', path: sys => sys.ac?.motors, aliases: ['motor', '电动机', '电机'], idKeys: ['index', 'id'] },
    { bucket: 'trafo3w', tableId: 'trafo3wTableInner', label: '三绕组变压器', domain: 'ac', path: sys => sys.ac?.transformers_3w, aliases: ['trafo3w', 'transformer3w', 'transformer_3w', '三绕组'], idKeys: ['index', 'id'] },
    { bucket: 'flexLoad', tableId: 'flexLoadTableInner', label: '柔性负荷', domain: 'ac', path: sys => sys.ac?.flexible_loads, aliases: ['flexload', 'flexible_load', '柔性负荷'], idKeys: ['index', 'id'] },
    { bucket: 'asymLoad', tableId: 'asymLoadTableInner', label: '不平衡负荷', domain: 'ac', path: sys => sys.ac?.asymmetric_loads, aliases: ['asymload', 'asymmetric_load', '不平衡负荷'], idKeys: ['index', 'id'] },
    { bucket: 'dcBranch', tableId: 'dcBranchTableInner', label: 'DC支路', domain: 'dc', path: sys => sys.dc?.branches, aliases: ['dcbranch', 'dc_branch', 'dcline', 'dc_line', '直流线路', '直流支路'], idKeys: ['index', 'id'] },
    { bucket: 'dcLoad', tableId: 'dcLoadTableInner', label: 'DC负荷', domain: 'dc', path: sys => sys.dc?.loads, aliases: ['dcload', 'dc_load', '直流负荷'], idKeys: ['index', 'id'] },
    { bucket: 'dcStorage', tableId: 'dcStorageTableInner', label: 'DC储能', domain: 'dc', path: sys => sys.dc?.dc_storage, aliases: ['dcstorage', 'dc_storage', '直流储能'], idKeys: ['index', 'id'] },
    { bucket: 'dcPv', tableId: 'dcPvTableInner', label: 'DC光伏', domain: 'dc', path: sys => sys.dc?.pv_arrays, aliases: ['dcpv', 'dc_pv', 'dc_pv_array', '直流光伏'], idKeys: ['index', 'id'] },
    { bucket: 'vsc', tableId: 'vscTableInner', label: 'VSC', domain: 'hybrid', path: sys => sys.vsc_converters, aliases: ['vsc', 'converter', 'vsc_converter', '换流器'], idKeys: ['index', 'id'] },
    { bucket: 'charger', tableId: 'chargerTableInner', label: '充电机', domain: 'ac', path: sys => sys.ac?.chargers, aliases: ['charger', '充电机'], idKeys: ['index', 'id'] },
    { bucket: 'chargingStation', tableId: 'csTableInner', label: '充电站', domain: 'ac', path: sys => sys.ac?.charging_stations, aliases: ['chargingstation', 'charging_station', '充电站'], idKeys: ['index', 'id'] },
    { bucket: 'mobileStorage', tableId: 'msTableInner', label: '移动储能', domain: 'hybrid', path: sys => sys.mobile_storage, aliases: ['mobilestorage', 'mobile_storage', 'mess', '移动储能'], idKeys: ['index', 'id'] },
    { bucket: 'dcdcConverter', tableId: 'dcdcTableInner', label: 'DC/DC', domain: 'dc', path: sys => sys.dcdc_converters, aliases: ['dcdc', 'dc_dc', 'dcdc_converter', 'dc/dc', '直流变换器'], idKeys: ['index', 'id'] },
    { bucket: 'energyRouter', tableId: 'erTableInner', label: '能量路由器', domain: 'hybrid', path: sys => sys.energy_routers, aliases: ['energyrouter', 'energy_router', 'er', '能量路由器'], idKeys: ['index', 'id'] },
    { bucket: 'vpp', tableId: 'vppTableInner', label: '虚拟电厂', domain: 'hybrid', path: sys => sys.vpps, aliases: ['vpp', 'virtualpowerplant', 'virtual_power_plant', '虚拟电厂'], idKeys: ['index', 'id'] },
    { bucket: 'microgrid', tableId: 'mgTableInner', label: '微网', domain: 'hybrid', path: sys => sys.microgrids, aliases: ['microgrid', '微网'], idKeys: ['index', 'id'] },
  ];

  // Normalize a query/label to a comparable key (drop punctuation/spacing/case).
  function searchKey(raw) {
    return String(raw || '')
      .toLowerCase()
      .replace(/[：:;,，、|\\(){}<>\[\]#_\-\s]/g, '')
      .replace(/\//g, '');
  }

  function parseQuery(raw) {
    const text = String(raw || '').trim();
    const key = searchKey(text);
    const numberMatch = text.match(/-?\d+/);
    const n = numberMatch ? Number(numberMatch[0]) : NaN;
    const hasDc = /\bdc\b/i.test(text) || /直流/.test(text);
    const hasAc = /\bac\b/i.test(text) || /交流/.test(text);
    return {
      text,
      key,
      number: Number.isFinite(n) ? n : null,
      domain: hasDc && !hasAc ? 'dc' : (hasAc && !hasDc ? 'ac' : ''),
    };
  }

  function sourceMatches(source, parsed) {
    if (!parsed.key) return false;
    if (parsed.domain && source.domain !== parsed.domain && source.domain !== 'hybrid') return false;
    const aliasKeys = (source.aliases || []).map(searchKey).filter(Boolean);
    if (aliasKeys.some(alias => parsed.key.includes(alias))) return true;
    return !parsed.number && source.label && searchKey(source.label).includes(parsed.key);
  }

  function rowIds(item, position, source) {
    const ids = [];
    (source.idKeys || []).forEach(key => {
      const v = Number(item?.[key]);
      if (Number.isFinite(v)) ids.push(v);
    });
    if (source.bucket !== 'ac' && source.bucket !== 'dc') {
      ids.push(position, position + 1);
    }
    return Array.from(new Set(ids));
  }

  function rowText(item, source, position) {
    const fields = [
      source.label, source.domain, position + 1,
      item?.index, item?.id, item?.name, item?.display_name, item?.type,
      item?.bus, item?.from_bus, item?.to_bus, item?.hv_bus, item?.lv_bus, item?.mv_bus,
      item?.bus_ac, item?.bus_dc, item?.pcc_bus, item?.target_bus,
    ];
    return searchKey(fields.filter(v => v !== undefined && v !== null && v !== '').join(' '));
  }

  function itemTitle(item, source, position) {
    const idx = item?.index ?? item?.id ?? (position + 1);
    const name = item?.name || item?.display_name || '';
    return `${source.label} ${idx}${name ? ` · ${name}` : ''}`;
  }

  function summarizeItem(row) {
    const item = row?.item || {};
    const keys = ['index', 'name', 'bus', 'from_bus', 'to_bus', 'hv_bus', 'lv_bus', 'bus_ac', 'bus_dc', 'pcc_bus', 'target_bus', 'pg_mw', 'p_mw', 'rate_a_mva'];
    const parts = [];
    keys.forEach(key => {
      const value = item[key];
      if (value !== undefined && value !== null && value !== '') parts.push(`${key}=${value}`);
    });
    return parts.length ? parts.join(' · ') : `row=${row.position + 1}`;
  }

  function sourceByTableId(tableId) {
    return SOURCES.find(s => s.tableId === tableId) || null;
  }

  return { SOURCES, searchKey, parseQuery, sourceMatches, rowIds, rowText, itemTitle, summarizeItem, sourceByTableId };
})();
