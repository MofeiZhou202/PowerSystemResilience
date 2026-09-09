/**
 * IEC 60617 device symbols for the busbar-mode one-line (P1, flag-gated).
 * docs/planning/gui_one_line_redesign.md.
 *
 * Used by web/js/canvas.js only when `state.busbarMode` is on; the legacy
 * COMP.symbols glyphs are used otherwise. Each symbol's terminal(s) sit at the
 * same coordinates as COMP.ports[type] so connections stay aligned:
 *   top (0,-30) · bottom (0,30) · left (-40|-30|-20,0) · right (+40|+30|+20,0).
 */
'use strict';
(function (global) {
  const G = '#98c379', LD = '#d19a66', TR = '#c678dd', GR = '#e06c75',
        CY = '#56b6c2', BL = '#61afef', GY = '#abb2bf', YL = '#e5c07b';
  const lbl = (name, y = 32) => `<text class="comp-label" x="0" y="${y}">${name}</text>`;
  const val = (t, y) => `<text class="comp-value" x="0" y="${y}">${t}</text>`;

  const IEC = {
    // Synchronous machine: circle with 'G' over a sine (IEC 60617-06).
    generator(p) {
      const pg = p._result_pg_mw ?? p.pg_mw ?? 0, u = p._result_p_unit || 'MW';
      return `<line x1="0" y1="-30" x2="0" y2="-18" stroke="${G}" stroke-width="2"/>
        <circle cx="0" cy="0" r="18" fill="none" stroke="${G}" stroke-width="2"/>
        <text x="0" y="-1" text-anchor="middle" fill="${G}" font-size="12" font-weight="700">G</text>
        <path d="M-8,8 Q-4,2 0,8 Q4,14 8,8" fill="none" stroke="${G}" stroke-width="1.5"/>
        ${lbl(p.name || 'G')}${val(`${pg}${u}`, 44)}`;
    },
    // Static generator / DG: circle with '~'.
    static_generator(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-16" stroke="${CY}" stroke-width="2"/>
        <circle cx="0" cy="0" r="16" fill="none" stroke="${CY}" stroke-width="2"/>
        <path d="M-8,0 Q-4,-6 0,0 Q4,6 8,0" fill="none" stroke="${CY}" stroke-width="1.6"/>
        ${lbl(p.name || 'DG', 30)}`;
    },
    // Renewable source: circle with '~' (green).
    renewable_gen(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-16" stroke="${G}" stroke-width="2"/>
        <circle cx="0" cy="0" r="16" fill="none" stroke="${G}" stroke-width="2"/>
        <path d="M-8,0 Q-4,-6 0,0 Q4,6 8,0" fill="none" stroke="${G}" stroke-width="1.6"/>
        ${lbl(p.name || 'REN', 30)}${val(`${p.p_rated_mw || 0}MW`, 42)}`;
    },
    // Load: solid arrow (IEC 60617 energy consumer).
    load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="${LD}" stroke-width="2"/>
        <path d="M-11,-8 L11,-8 L0,12 Z" fill="${LD}" stroke="${LD}" stroke-width="1"/>
        ${lbl(p.name || 'Load', 30)}${val(`${p.p_mw || 0}MW`, 42)}`;
    },
    flexible_load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="${LD}" stroke-width="2"/>
        <path d="M-11,-8 L11,-8 L0,12 Z" fill="none" stroke="${LD}" stroke-width="2"/>
        <path d="M-5,-4 L3,-4 L-2,3 L5,3" fill="none" stroke="${LD}" stroke-width="1.4"/>
        ${lbl(p.name || 'Flex', 30)}${val(`${p.p_mw || 0}MW`, 42)}`;
    },
    // Two-winding transformer: interlocking circles (IEC 60617-06).
    transformer_2w(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-20" stroke="${TR}" stroke-width="2"/>
        <circle cx="0" cy="-9" r="12" fill="none" stroke="${TR}" stroke-width="2"/>
        <circle cx="0" cy="9" r="12" fill="none" stroke="${TR}" stroke-width="2"/>
        <line x1="0" y1="20" x2="0" y2="30" stroke="${TR}" stroke-width="2"/>
        <text class="comp-label" x="22" y="4">${p.name || 'T'}</text>`;
    },
    // Three-winding transformer: three interlocking circles.
    transformer_3w(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-22" stroke="${TR}" stroke-width="2"/>
        <circle cx="0" cy="-11" r="11" fill="none" stroke="${TR}" stroke-width="2"/>
        <circle cx="-10" cy="8" r="11" fill="none" stroke="${TR}" stroke-width="2"/>
        <circle cx="10" cy="8" r="11" fill="none" stroke="${TR}" stroke-width="2"/>
        <line x1="-10" y1="19" x2="-10" y2="30" stroke="${TR}" stroke-width="2"/>
        <line x1="10" y1="19" x2="10" y2="30" stroke="${TR}" stroke-width="2"/>`;
    },
    // Network in-feed: circle with an arrow into a bar (utility supply).
    external_grid(p) {
      const sp = p._result_p_mw, u = p._result_p_unit || 'MW';
      const res = sp == null ? '' : val(`P ${sp}${u}`, 56);
      return `<line x1="0" y1="18" x2="0" y2="30" stroke="${GR}" stroke-width="2"/>
        <rect x="-15" y="-15" width="30" height="30" rx="2" fill="none" stroke="${GR}" stroke-width="2"/>
        <path d="M-9,2 Q-4,-8 0,2 Q4,12 9,2" fill="none" stroke="${GR}" stroke-width="1.8"/>
        ${lbl(p.name || 'Grid', 44)}${res}`;
    },
    // AC line: plain conductor (IEC 60617-03).
    ac_branch(p) {
      return `<line x1="-40" y1="0" x2="40" y2="0" stroke="${GY}" stroke-width="2.5"/>
        <text class="comp-label" x="0" y="-10">${p.name || 'Line'}</text>
        <text class="comp-value" x="0" y="18">${(p.length_km || 0).toFixed(1)}km</text>`;
    },
    dc_branch(p) {
      return `<line x1="-40" y1="0" x2="40" y2="0" stroke="${CY}" stroke-width="2.5" stroke-dasharray="6 3"/>
        <text class="comp-label" x="0" y="-10">${p.name || 'DC Line'}</text>`;
    },
    // Power converter: square with AC/DC diagonal (IEC 60617-08).
    vsc_converter(p) {
      return `<rect x="-18" y="-18" width="36" height="36" rx="2" fill="none" stroke="${CY}" stroke-width="2"/>
        <line x1="-18" y1="18" x2="18" y2="-18" stroke="${CY}" stroke-width="1.5"/>
        <path d="M-13,-6 Q-10,-11 -7,-6" fill="none" stroke="${CY}" stroke-width="1.4"/>
        <line x1="6" y1="9" x2="13" y2="9" stroke="${CY}" stroke-width="1.6"/>
        <line x1="6" y1="12" x2="13" y2="12" stroke="${CY}" stroke-width="1"/>
        <line x1="-18" y1="0" x2="-30" y2="0" stroke="${BL}" stroke-width="2"/>
        <line x1="18" y1="0" x2="30" y2="0" stroke="${CY}" stroke-width="2" stroke-dasharray="4 2"/>
        ${lbl(p.name || 'VSC', 32)}`;
    },
    lcc_converter(p) {
      return `<rect x="-18" y="-18" width="36" height="36" rx="2" fill="none" stroke="#c19a6b" stroke-width="2"/>
        <line x1="-18" y1="18" x2="18" y2="-18" stroke="#c19a6b" stroke-width="1.5"/>
        <text x="0" y="4" text-anchor="middle" fill="#c19a6b" font-size="9" font-weight="700">LCC</text>
        <line x1="-18" y1="0" x2="-30" y2="0" stroke="${BL}" stroke-width="2"/>
        <line x1="18" y1="0" x2="30" y2="0" stroke="${CY}" stroke-width="2" stroke-dasharray="4 2"/>
        ${lbl(p.name || 'LCC', 32)}`;
    },
    // Battery / storage: alternating long/short cells (IEC 60617-06).
    storage(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-16" stroke="${YL}" stroke-width="2"/>
        <line x1="-12" y1="-16" x2="12" y2="-16" stroke="${YL}" stroke-width="3"/>
        <line x1="-6" y1="-11" x2="6" y2="-11" stroke="${YL}" stroke-width="1.4"/>
        <line x1="-12" y1="-6" x2="12" y2="-6" stroke="${YL}" stroke-width="3"/>
        <line x1="-6" y1="-1" x2="6" y2="-1" stroke="${YL}" stroke-width="1.4"/>
        ${lbl(p.name || 'ESS', 16)}${val(`${p.e_rated_mwh || 0}MWh`, 28)}`;
    },
    // PV: cell rectangle with diagonal (IEC 60617 photovoltaic).
    pv_system(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-16" stroke="${YL}" stroke-width="2"/>
        <rect x="-16" y="-16" width="32" height="24" fill="none" stroke="${YL}" stroke-width="2"/>
        <line x1="-16" y1="8" x2="16" y2="-16" stroke="${YL}" stroke-width="1.4"/>
        ${lbl(p.name || 'PV', 22)}${val(`${p.p_mw || 0}MW`, 34)}`;
    },
    // Motor: circle with 'M' (IEC 60617-06).
    motor(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-18" stroke="${CY}" stroke-width="2"/>
        <circle cx="0" cy="0" r="18" fill="none" stroke="${CY}" stroke-width="2"/>
        <text x="0" y="5" text-anchor="middle" fill="${CY}" font-size="14" font-weight="700">M</text>
        ${lbl(p.name || 'M', 32)}`;
    },
    // Shunt capacitor: two plates (IEC 60617-04).
    shunt(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="${GY}" stroke-width="2"/>
        <line x1="-13" y1="-8" x2="13" y2="-8" stroke="${GY}" stroke-width="2.5"/>
        <line x1="-13" y1="-2" x2="13" y2="-2" stroke="${GY}" stroke-width="2.5"/>
        <line x1="0" y1="-2" x2="0" y2="8" stroke="${GY}" stroke-width="2"/>
        <path d="M-5,12 L0,8 L5,12" fill="none" stroke="${GY}" stroke-width="1.5"/>
        ${lbl(p.name || 'Shunt', 24)}${val(`${p.bs_mvar || 0}MVar`, 36)}`;
    },
    // Circuit breaker: switch blade with a square (IEC 60617-07).
    circuit_breaker(p) {
      const sp = p._result_p_mw, u = p._result_p_unit || 'MW';
      const res = sp == null ? '' : val(`P ${sp}${u}`, 30);
      return `<line x1="-20" y1="0" x2="-8" y2="0" stroke="${GY}" stroke-width="2"/>
        <line x1="8" y1="0" x2="20" y2="0" stroke="${GY}" stroke-width="2"/>
        <rect x="-8" y="-8" width="16" height="16" fill="none" stroke="${GR}" stroke-width="2"/>
        <line x1="-5" y1="5" x2="5" y2="-5" stroke="${GR}" stroke-width="2"/>
        <text class="comp-label" x="0" y="20">${p.name || 'CB'}</text>${res}`;
    },
    // Disconnector / switch: open blade (IEC 60617-07).
    switch_comp(p) {
      return `<line x1="-20" y1="0" x2="-6" y2="0" stroke="${GY}" stroke-width="2"/>
        <line x1="6" y1="0" x2="20" y2="0" stroke="${GY}" stroke-width="2"/>
        <line x1="-6" y1="0" x2="7" y2="-11" stroke="${GY}" stroke-width="2"/>
        <circle cx="-6" cy="0" r="2.5" fill="${GY}"/>
        <circle cx="6" cy="0" r="2.5" fill="none" stroke="${GY}" stroke-width="1.5"/>
        <text class="comp-label" x="0" y="16">${p.name || 'SW'}</text>`;
    },
  };

  global.IEC_SYMBOLS = IEC;
})(typeof window !== 'undefined' ? window : this);
