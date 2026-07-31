/** Canonical Dashboard/result-row to Canvas component mapping. */
'use strict';

(function initResultMapping(global) {
  const core = global.HySimCore = global.HySimCore || {};
  const aliases = Object.freeze({
    ac: 'ac', ac_bus: 'ac', bus_ac: 'ac',
    dc: 'dc', dc_bus: 'dc', bus_dc: 'dc',
    branch: 'branch', ac_branch: 'branch', line: 'branch', ac_line: 'branch',
    transformer: 'trafo', transformer_2w: 'trafo', transformer2_w: 'trafo',
    transformer2w: 'trafo', trafo: 'trafo', transformer_3w: 'trafo3w',
    transformer3_w: 'trafo3w', transformer3w: 'trafo3w', trafo3w: 'trafo3w',
    generator: 'gen', synchronous_machine: 'gen', three_phase_generator: 'gen', gen: 'gen',
    ac_load: 'load', three_phase_load: 'load', load: 'load',
    external_grid: 'extGrid', three_phase_external_grid: 'extGrid', ext_grid: 'extGrid', extgrid: 'extGrid',
    storage: 'storage', ac_storage: 'storage', grid_forming_storage: 'storage',
    pv_system: 'pv', ac_pv_system: 'pv', pv: 'pv',
    renewable_gen: 'renGen', renewable_generator: 'renGen', ren_gen: 'renGen',
    static_generator: 'sgen', static_gen: 'sgen', sgen: 'sgen',
    dc_static_generator: 'dcSgen', dc_static_gen: 'dcSgen',
    dc_static_generator_ac: 'dcSgen', dc_sgen: 'dcSgen',
    switch: 'sw', switch_comp: 'sw', ac_switch: 'sw', sw: 'sw',
    circuit_breaker: 'cb', ac_circuit_breaker: 'cb', breaker: 'cb', cb: 'cb',
    dc_circuit_breaker: 'dcCb', dc_breaker: 'dcCb', dc_cb: 'dcCb',
    motor: 'motor', dc_branch: 'dcBranch', dc_line: 'dcBranch', dc_load: 'dcLoad',
    dc_storage: 'dcStorage', dc_pv_array: 'dcPv', dc_pv: 'dcPv',
    vsc: 'vsc', vsc_converter: 'vsc', vsc_grid_forming: 'vsc', vsc_grid_following: 'vsc',
    lcc: 'lcc', lcc_converter: 'lcc',
    dcdc: 'dcdcConverter', dc_dc: 'dcdcConverter', dcdc_converter: 'dcdcConverter',
    dc_dc_converter: 'dcdcConverter', energy_router: 'energyRouter', er: 'energyRouter',
    shunt: 'shunt', flexible_load: 'flexLoad', flex_load: 'flexLoad',
    asymmetric_load: 'asymLoad', asym_load: 'asymLoad', charger: 'charger',
    charging_station: 'chargingStation', mobile_storage: 'mobileStorage',
    vpp: 'vpp', virtual_power_plant: 'vpp', microgrid: 'microgrid',
  });

  function typeKey(type) {
    return String(type || '')
      .replace(/([a-z0-9])([A-Z])/g, '$1_$2')
      .replace(/[-\s/]+/g, '_')
      .replace(/__+/g, '_')
      .toLowerCase();
  }

  function bucketFor(type, maps) {
    if (!type || !maps) return undefined;
    const raw = String(type);
    if (maps[raw]) return raw;
    const bucket = aliases[typeKey(raw)] || aliases[raw];
    return bucket && maps[bucket] ? bucket : undefined;
  }

  function validCompId(value, getComponent) {
    const id = Number(value);
    if (!Number.isInteger(id)) return undefined;
    if (getComponent && !getComponent(id)) return undefined;
    return id;
  }

  function resolve(row, maps, options = {}) {
    const valid = value => validCompId(value, options.getComponent);
    if (!row || !maps) return valid(options.fallbackCompId);
    for (const key of ['canvas_comp_id', 'comp_id', 'canvasComponentId', 'canvas_id']) {
      const hit = valid(row[key]);
      if (hit !== undefined) return hit;
    }

    const typeCandidates = [
      row.canvas_type, row.component_type, row.canonical_component_type,
      row.type, row.bucket, options.canvasType,
    ].filter(value => value !== undefined && value !== null && String(value) !== '');
    let bucket;
    for (const type of typeCandidates) {
      bucket = bucketFor(type, maps);
      if (bucket) break;
    }

    if (bucket) {
      for (const key of ['canvas_index', 'index', 'id', 'router_index', 'component_index']) {
        const index = Number(row[key]);
        if (Number.isFinite(index) && maps[bucket]?.[index] != null) return valid(maps[bucket][index]);
      }
      for (const key of ['position', 'component_position', 'canvas_position', 'row_position']) {
        const position = Number(row[key]);
        if (Number.isFinite(position) && maps.byPosition?.[bucket]?.[position] != null) {
          return valid(maps.byPosition[bucket][position]);
        }
      }
      for (const key of ['position', 'component_position', 'canvas_position', 'row_position']) {
        const position = Number(row[key]);
        if (Number.isFinite(position) && maps[bucket]?.[position] != null) return valid(maps[bucket][position]);
      }
    }

    const domain = `${row.domain || row.component_domain || ''} ${typeCandidates.join(' ')} ${bucket || ''}`.toLowerCase();
    const dcBuckets = ['dc', 'dcBranch', 'dcLoad', 'dcStorage', 'dcPv', 'dcCb', 'dcSgen', 'dcdcConverter'];
    const acBuckets = ['ac', 'branch', 'load', 'gen', 'extGrid', 'storage', 'pv', 'renGen', 'sgen',
      'sw', 'cb', 'motor', 'shunt', 'trafo', 'trafo3w', 'flexLoad', 'asymLoad', 'charger',
      'chargingStation', 'mobileStorage', 'vpp', 'microgrid'];
    const preferDc = /\bdc\b/.test(domain) || dcBuckets.includes(bucket);
    const preferAc = /\bac\b/.test(domain) || acBuckets.includes(bucket);
    const busKeys = preferDc && !preferAc
      ? ['bus_dc', 'dc_bus', 'bus_in', 'bus_out', 'from_bus', 'to_bus', 'from', 'to', 'primary_bus', 'bus', 'index']
      : preferAc && !preferDc
      ? ['bus_ac', 'ac_bus', 'bus', 'from_bus', 'to_bus', 'from', 'to', 'hv_bus', 'mv_bus', 'lv_bus', 'primary_bus', 'index']
      : ['bus_ac', 'ac_bus', 'bus_dc', 'dc_bus', 'bus', 'from_bus', 'to_bus', 'from', 'to',
        'bus_in', 'bus_out', 'hv_bus', 'mv_bus', 'lv_bus', 'primary_bus', 'index'];
    const tryBuses = map => {
      if (!map) return undefined;
      for (const key of busKeys) {
        const bus = Number(row[key]);
        if (Number.isFinite(bus) && map[bus] != null) return valid(map[bus]);
      }
      return undefined;
    };
    const primary = preferDc && !preferAc ? maps.dc : maps.ac;
    const secondary = preferDc && !preferAc ? maps.ac : maps.dc;
    return tryBuses(primary) ?? tryBuses(secondary) ?? valid(options.fallbackCompId);
  }

  core.ResultMapping = Object.freeze({
    schema: 'hysim_canvas_ref_v1',
    typeKey,
    bucketFor,
    resolve,
  });
})(window);
