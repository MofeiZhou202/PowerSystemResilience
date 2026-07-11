/**
 * components.js — Power system component definitions with SVG symbols.
 * Defines all component types, their default parameters, SVG rendering,
 * and port positions for connections.
 */
'use strict';

const COMP = (() => {

  // ============= SVG Symbol Generators =============
  // Each returns an SVG string fragment (group content only, no outer <g>)
  // Origin at (0,0), ports are defined separately

  const symbols = {
    ac_bus(p) {
      return `<line x1="-40" y1="0" x2="40" y2="0" stroke-width="5" stroke="#61afef"/>
              <text class="comp-label" x="0" y="-12">${p.name||'Bus'}</text>
              <text class="comp-value" x="0" y="16">${p.base_kv||110}kV</text>`;
    },
    generator(p) {
      const pg = p._result_pg_mw ?? p.pg_mw ?? 0;
      const unit = p._result_p_unit || 'MW';
      return `<circle cx="0" cy="0" r="18" class="symbol" fill="none" stroke="#98c379" stroke-width="2"/>
              <text x="0" y="5" text-anchor="middle" fill="#98c379" font-size="14" font-weight="700">G</text>
              <line x1="0" y1="-18" x2="0" y2="-30" stroke="#98c379" stroke-width="2"/>
              <text class="comp-label" x="0" y="32">${p.name||'Gen'}</text>
              <text class="comp-value" x="0" y="44">${pg}${unit}</text>`;
    },
    load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="#d19a66" stroke-width="2"/>
              <polygon points="-14,-8 14,-8 0,18" fill="none" stroke="#d19a66" stroke-width="2"/>
              <text class="comp-label" x="0" y="34">${p.name||'Load'}</text>
              <text class="comp-value" x="0" y="46">${p.p_mw||0}MW</text>`;
    },
    transformer_2w(p) {
      return `<circle cx="0" cy="-10" r="12" class="symbol" fill="none" stroke="#c678dd" stroke-width="2"/>
              <circle cx="0" cy="10" r="12" class="symbol" fill="none" stroke="#c678dd" stroke-width="2"/>
              <line x1="0" y1="-30" x2="0" y2="-22" stroke="#c678dd" stroke-width="2"/>
              <line x1="0" y1="22" x2="0" y2="30" stroke="#c678dd" stroke-width="2"/>
              <text class="comp-label" x="22" y="4">${p.name||'Trafo'}</text>`;
    },
    ac_branch(p) {
      return `<line x1="-40" y1="0" x2="40" y2="0" class="symbol" stroke="#abb2bf" stroke-width="2"/>
              <rect x="-12" y="-6" width="24" height="12" fill="none" stroke="#abb2bf" stroke-width="1.5" rx="2"/>
              <text class="comp-label" x="0" y="-12">${p.name||'Line'}</text>
              <text class="comp-value" x="0" y="20">${(p.length_km||0).toFixed(1)}km</text>`;
    },
    external_grid(p) {
      return `<line x1="0" y1="18" x2="0" y2="30" stroke="#e06c75" stroke-width="2"/>
              <path d="M-14,-14 L14,-14 L14,14 L-14,14 Z" fill="none" stroke="#e06c75" stroke-width="2"/>
              <path d="M-8,0 Q-4,-8 0,0 Q4,8 8,0" fill="none" stroke="#e06c75" stroke-width="2"/>
              <text class="comp-label" x="0" y="44">${p.name||'Grid'}</text>`;
    },
    storage(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-14" stroke="#e5c07b" stroke-width="2"/>
              <rect x="-14" y="-14" width="28" height="22" rx="2" class="symbol" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <line x1="-8" y1="-4" x2="8" y2="-4" stroke="#e5c07b" stroke-width="2"/>
              <line x1="0" y1="-9" x2="0" y2="1" stroke="#e5c07b" stroke-width="1.5"/>
              <text class="comp-label" x="0" y="20">${p.name||'ESS'}</text>
              <text class="comp-value" x="0" y="32">${p.e_rated_mwh||0}MWh</text>`;
    },
    dc_storage(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-14" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <rect x="-14" y="-14" width="28" height="22" rx="2" class="symbol" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <line x1="-8" y1="-9" x2="8" y2="-9" stroke="#56b6c2" stroke-width="2"/>
              <line x1="-8" y1="-1" x2="8" y2="-1" stroke="#56b6c2" stroke-width="1.5"/>
              <text x="0" y="6" text-anchor="middle" fill="#56b6c2" font-size="7" font-weight="700">DC</text>
              <text class="comp-label" x="0" y="20">${p.name||'DC ESS'}</text>
              <text class="comp-value" x="0" y="32">${p.e_rated_mwh||0}MWh</text>`;
    },
    pv_system(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-14" stroke="#e5c07b" stroke-width="2"/>
              <polygon points="-16,-14 16,-14 12,10 -12,10" class="symbol" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <line x1="-6" y1="-4" x2="6" y2="-4" stroke="#e5c07b" stroke-width="1"/>
              <line x1="-4" y1="2" x2="4" y2="2" stroke="#e5c07b" stroke-width="1"/>
              <text class="comp-label" x="0" y="24">${p.name||'PV'}</text>
              <text class="comp-value" x="0" y="36">${p.p_mw||0}MW</text>`;
    },
    renewable_gen(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-16" stroke="#98c379" stroke-width="2"/>
              <circle cx="0" cy="0" r="16" class="symbol" fill="none" stroke="#98c379" stroke-width="2"/>
              <text x="0" y="5" text-anchor="middle" fill="#98c379" font-size="11" font-weight="700">~</text>
              <text class="comp-label" x="0" y="28">${p.name||'REN'}</text>
              <text class="comp-value" x="0" y="40">${p.p_rated_mw||0}MW</text>`;
    },
    static_generator(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-16" stroke="#98c379" stroke-width="2"/>
              <circle cx="0" cy="0" r="16" class="symbol" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <text x="0" y="5" text-anchor="middle" fill="#56b6c2" font-size="10" font-weight="700">DG</text>
              <text class="comp-label" x="0" y="28">${p.name||'SGen'}</text>`;
    },
    vsc_converter(p) {
      return `<rect x="-18" y="-18" width="36" height="36" rx="3" class="symbol" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <text x="0" y="4" text-anchor="middle" fill="#56b6c2" font-size="10" font-weight="700">VSC</text>
              <line x1="-18" y1="0" x2="-30" y2="0" stroke="#61afef" stroke-width="2"/>
              <line x1="18" y1="0" x2="30" y2="0" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <text class="comp-label" x="0" y="30">${p.name||'VSC'}</text>`;
    },
    dc_bus(p) {
      return `<line x1="-40" y1="0" x2="40" y2="0" stroke="#56b6c2" stroke-width="5" stroke-dasharray="8 4"/>
              <text class="comp-label" x="0" y="-12">${p.name||'DC Bus'}</text>
              <text class="comp-value" x="0" y="16">${p.base_kv||320}kV</text>`;
    },
    dc_branch(p) {
      return `<line x1="-40" y1="0" x2="40" y2="0" stroke="#56b6c2" stroke-width="2" stroke-dasharray="6 3"/>
              <rect x="-10" y="-5" width="20" height="10" fill="none" stroke="#56b6c2" stroke-width="1.5" rx="2"/>
              <text class="comp-label" x="0" y="-12">${p.name||'DC Line'}</text>`;
    },
    switch_comp(p) {
      return `<line x1="-20" y1="0" x2="-6" y2="0" class="symbol" stroke="#abb2bf" stroke-width="2"/>
              <line x1="6" y1="0" x2="20" y2="0" class="symbol" stroke="#abb2bf" stroke-width="2"/>
              <line x1="-6" y1="0" x2="6" y2="-10" class="symbol" stroke="#abb2bf" stroke-width="2"/>
              <circle cx="-6" cy="0" r="3" fill="#abb2bf"/>
              <circle cx="6" cy="0" r="3" fill="none" stroke="#abb2bf" stroke-width="1.5"/>
              <text class="comp-label" x="0" y="16">${p.name||'SW'}</text>`;
    },
    circuit_breaker(p) {
      return `<line x1="-20" y1="0" x2="-8" y2="0" stroke="#abb2bf" stroke-width="2"/>
              <line x1="8" y1="0" x2="20" y2="0" stroke="#abb2bf" stroke-width="2"/>
              <rect x="-8" y="-8" width="16" height="16" fill="none" stroke="#e06c75" stroke-width="2" rx="2"/>
              <line x1="-5" y1="-5" x2="5" y2="5" stroke="#e06c75" stroke-width="2"/>
              <line x1="5" y1="-5" x2="-5" y2="5" stroke="#e06c75" stroke-width="2"/>
              <text class="comp-label" x="0" y="20">${p.name||'CB'}</text>`;
    },
    dc_load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="#d19a66" stroke-width="2" stroke-dasharray="4 2"/>
              <polygon points="-14,-8 14,-8 0,18" fill="none" stroke="#d19a66" stroke-width="2"/>
              <text class="comp-label" x="0" y="34">${p.name||'DC Load'}</text>
              <text class="comp-value" x="0" y="46">${p.p_mw||0}MW</text>`;
    },
    dc_pv_array(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-14" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <polygon points="-16,-14 16,-14 12,10 -12,10" class="symbol" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <line x1="-6" y1="-4" x2="6" y2="-4" stroke="#56b6c2" stroke-width="1"/>
              <line x1="-4" y1="2" x2="4" y2="2" stroke="#56b6c2" stroke-width="1"/>
              <text x="0" y="-2" text-anchor="middle" fill="#56b6c2" font-size="7" font-weight="700">DC</text>
              <text class="comp-label" x="0" y="24">${p.name||'DC PV'}</text>
              <text class="comp-value" x="0" y="36">${p.p_set_mw||0}MW</text>`;
    },
    motor(p) {
      return `<circle cx="0" cy="0" r="18" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <text x="0" y="5" text-anchor="middle" fill="#56b6c2" font-size="14" font-weight="700">M</text>
              <line x1="0" y1="-18" x2="0" y2="-30" stroke="#56b6c2" stroke-width="2"/>
              <text class="comp-label" x="0" y="32">${p.name||'Motor'}</text>`;
    },
    flexible_load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="#d19a66" stroke-width="2"/>
              <polygon points="-14,-8 14,-8 0,18" fill="none" stroke="#d19a66" stroke-width="2"/>
              <line x1="-6" y1="2" x2="6" y2="2" stroke="#d19a66" stroke-width="1.5"/>
              <path d="M-4,6 L4,6" stroke="#d19a66" stroke-width="1.5" stroke-dasharray="2 2"/>
              <text class="comp-label" x="0" y="34">${p.name||'FlexLoad'}</text>
              <text class="comp-value" x="0" y="46">${p.p_mw||0}MW</text>`;
    },
    asymmetric_load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="#d19a66" stroke-width="2"/>
              <polygon points="-14,-8 14,-8 0,18" fill="none" stroke="#d19a66" stroke-width="2"/>
              <text x="-6" y="6" fill="#d19a66" font-size="7" font-weight="700">A</text>
              <text x="1" y="6" fill="#d19a66" font-size="7" font-weight="700">B</text>
              <text x="-3" y="13" fill="#d19a66" font-size="7" font-weight="700">C</text>
              <text class="comp-label" x="0" y="34">${p.name||'AsymLoad'}</text>`;
    },
    shunt(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-10" stroke="#abb2bf" stroke-width="2"/>
              <line x1="-12" y1="-10" x2="12" y2="-10" stroke="#abb2bf" stroke-width="2"/>
              <line x1="-12" y1="-4" x2="12" y2="-4" stroke="#abb2bf" stroke-width="2"/>
              <line x1="-8" y1="4" x2="8" y2="4" stroke="#abb2bf" stroke-width="1"/>
              <line x1="-4" y1="10" x2="4" y2="10" stroke="#abb2bf" stroke-width="1"/>
              <text class="comp-label" x="0" y="24">${p.name||'Shunt'}</text>
              <text class="comp-value" x="0" y="36">${p.bs_mvar||0}MVar</text>`;
    },
    transformer_3w(p) {
      return `<circle cx="0" cy="-14" r="10" fill="none" stroke="#c678dd" stroke-width="2"/>
              <circle cx="-10" cy="10" r="10" fill="none" stroke="#c678dd" stroke-width="2"/>
              <circle cx="10" cy="10" r="10" fill="none" stroke="#c678dd" stroke-width="2"/>
              <line x1="0" y1="-30" x2="0" y2="-24" stroke="#c678dd" stroke-width="2"/>
              <line x1="-10" y1="20" x2="-10" y2="30" stroke="#c678dd" stroke-width="2"/>
              <line x1="10" y1="20" x2="10" y2="30" stroke="#c678dd" stroke-width="2"/>
              <text class="comp-label" x="22" y="0">${p.name||'Trafo3W'}</text>`;
    },
    charger(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-14" stroke="#e5c07b" stroke-width="2"/>
              <rect x="-14" y="-14" width="28" height="24" rx="3" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <path d="M-2,-8 L2,-8 L0,-2 L4,-2 L-2,8 L0,2 L-4,2 Z" fill="#e5c07b"/>
              <text class="comp-label" x="0" y="24">${p.name||'Charger'}</text>`;
    },
    charging_station(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-18" stroke="#e5c07b" stroke-width="2"/>
              <rect x="-18" y="-18" width="36" height="30" rx="3" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <text x="0" y="-2" text-anchor="middle" fill="#e5c07b" font-size="9" font-weight="700">EV</text>
              <path d="M-2,2 L2,2 L0,8 L4,8 L-2,14 L0,8 L-4,8 Z" fill="#e5c07b" transform="scale(0.6) translate(0,-4)"/>
              <text class="comp-label" x="0" y="26">${p.name||'EVStation'}</text>`;
    },
    mobile_storage(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-14" stroke="#e5c07b" stroke-width="2"/>
              <rect x="-16" y="-14" width="32" height="22" rx="2" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <line x1="-10" y1="-4" x2="10" y2="-4" stroke="#e5c07b" stroke-width="2"/>
              <line x1="0" y1="-9" x2="0" y2="1" stroke="#e5c07b" stroke-width="1.5"/>
              <circle cx="-8" cy="14" r="3" fill="none" stroke="#e5c07b" stroke-width="1.5"/>
              <circle cx="8" cy="14" r="3" fill="none" stroke="#e5c07b" stroke-width="1.5"/>
              <text class="comp-label" x="0" y="28">${p.name||'MobESS'}</text>`;
    },
    dcdc_converter(p) {
      return `<rect x="-18" y="-18" width="36" height="36" rx="3" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <text x="0" y="-2" text-anchor="middle" fill="#56b6c2" font-size="8" font-weight="700">DC</text>
              <text x="0" y="10" text-anchor="middle" fill="#56b6c2" font-size="8" font-weight="700">DC</text>
              <line x1="-10" y1="2" x2="10" y2="2" stroke="#56b6c2" stroke-width="1"/>
              <line x1="-18" y1="0" x2="-30" y2="0" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <line x1="18" y1="0" x2="30" y2="0" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <text class="comp-label" x="0" y="30">${p.name||'DC/DC'}</text>`;
    },
    energy_router(p) {
      return `<polygon points="0,-22 22,0 0,22 -22,0" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <text x="0" y="4" text-anchor="middle" fill="#56b6c2" font-size="9" font-weight="700">ER</text>
              <line x1="-22" y1="0" x2="-34" y2="0" stroke="#61afef" stroke-width="2"/>
              <line x1="22" y1="0" x2="34" y2="0" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <line x1="0" y1="-22" x2="0" y2="-34" stroke="#61afef" stroke-width="2"/>
              <line x1="0" y1="22" x2="0" y2="34" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <text class="comp-label" x="0" y="46">${p.name||'ERouter'}</text>`;
    },
    vpp(p) {
      return `<rect x="-22" y="-18" width="44" height="36" rx="4" fill="none" stroke="#98c379" stroke-width="2" stroke-dasharray="6 3"/>
              <text x="0" y="-2" text-anchor="middle" fill="#98c379" font-size="9" font-weight="700">VPP</text>
              <text x="0" y="10" text-anchor="middle" fill="#98c379" font-size="7">${p.p_output_mw||0}MW</text>
              <line x1="0" y1="-18" x2="0" y2="-30" stroke="#98c379" stroke-width="2"/>
              <text class="comp-label" x="0" y="30">${p.name||'VPP'}</text>`;
    },
    microgrid(p) {
      return `<rect x="-24" y="-20" width="48" height="40" rx="6" fill="none" stroke="#61afef" stroke-width="2" stroke-dasharray="8 4"/>
              <text x="0" y="-4" text-anchor="middle" fill="#61afef" font-size="9" font-weight="700">MG</text>
              <text x="0" y="8" text-anchor="middle" fill="#61afef" font-size="7">${p.operating_mode||'Grid'}</text>
              <line x1="0" y1="-20" x2="0" y2="-32" stroke="#61afef" stroke-width="2"/>
              <text class="comp-label" x="0" y="32">${p.name||'MicroGrid'}</text>`;
    },
    ies_electric_bus(p) {
      return `<line x1="-42" y1="0" x2="42" y2="0" stroke="#61afef" stroke-width="6"/>
              <circle cx="-42" cy="0" r="3" fill="#61afef"/><circle cx="42" cy="0" r="3" fill="#61afef"/>
              <text class="comp-label" x="0" y="-13">${p.name||'电母线'}</text>
              <text class="comp-value" x="0" y="17">Electric</text>`;
    },
    ies_heat_bus(p) {
      return `<line x1="-42" y1="0" x2="42" y2="0" stroke="#e06c75" stroke-width="6"/>
              <circle cx="-42" cy="0" r="3" fill="#e06c75"/><circle cx="42" cy="0" r="3" fill="#e06c75"/>
              <text class="comp-label" x="0" y="-13">${p.name||'热母线'}</text>
              <text class="comp-value" x="0" y="17">Heat</text>`;
    },
    ies_hydrogen_bus(p) {
      return `<line x1="-42" y1="0" x2="42" y2="0" stroke="#56b6c2" stroke-width="6" stroke-dasharray="9 4"/>
              <circle cx="-42" cy="0" r="3" fill="#56b6c2"/><circle cx="42" cy="0" r="3" fill="#56b6c2"/>
              <text class="comp-label" x="0" y="-13">${p.name||'氢母线'}</text>
              <text class="comp-value" x="0" y="17">Hydrogen</text>`;
    },
    ies_fuel_bus(p) {
      return `<line x1="-42" y1="0" x2="42" y2="0" stroke="#e5c07b" stroke-width="6" stroke-dasharray="5 4"/>
              <circle cx="-42" cy="0" r="3" fill="#e5c07b"/><circle cx="42" cy="0" r="3" fill="#e5c07b"/>
              <text class="comp-label" x="0" y="-13">${p.name||'燃料母线'}</text>
              <text class="comp-value" x="0" y="17">Fuel</text>`;
    },
    ies_grid(p) {
      return `<rect x="-18" y="-18" width="36" height="36" rx="4" fill="none" stroke="#61afef" stroke-width="2"/>
              <path d="M-10,0 Q-5,-8 0,0 Q5,8 10,0" fill="none" stroke="#61afef" stroke-width="2"/>
              <line x1="18" y1="0" x2="32" y2="0" stroke="#61afef" stroke-width="2"/>
              <text class="comp-label" x="0" y="31">${p.name||'电网'}</text>
              <text class="comp-value" x="0" y="43">${p.import_limit_mw||0}MW</text>`;
    },
    ies_electric_load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="#61afef" stroke-width="2"/>
              <polygon points="-14,-8 14,-8 0,18" fill="none" stroke="#61afef" stroke-width="2"/>
              <text class="comp-label" x="0" y="34">${p.name||'电负荷'}</text>
              <text class="comp-value" x="0" y="46">${p.demand_mw||0}MW</text>`;
    },
    ies_heat_load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-8" stroke="#e06c75" stroke-width="2"/>
              <path d="M-14,18 C-8,-10 8,-10 14,18 Z" fill="none" stroke="#e06c75" stroke-width="2"/>
              <text class="comp-label" x="0" y="34">${p.name||'热负荷'}</text>
              <text class="comp-value" x="0" y="46">${p.demand_mw||0}MW</text>`;
    },
    ies_hydrogen_load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-10" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <circle cx="0" cy="4" r="15" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <text x="0" y="8" text-anchor="middle" fill="#56b6c2" font-size="9" font-weight="700">H2</text>
              <text class="comp-label" x="0" y="34">${p.name||'氢负荷'}</text>
              <text class="comp-value" x="0" y="46">${p.demand_mw||0}MW</text>`;
    },
    ies_fuel_load(p) {
      return `<line x1="0" y1="-30" x2="0" y2="-10" stroke="#e5c07b" stroke-width="2" stroke-dasharray="5 3"/>
              <path d="M0,-8 C12,4 10,20 0,22 C-10,20 -12,4 0,-8 Z" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <text class="comp-label" x="0" y="38">${p.name||'燃料负荷'}</text>
              <text class="comp-value" x="0" y="50">${p.demand_mw||0}MW</text>`;
    },
    ies_transport(p) {
      return `<rect x="-22" y="-14" width="44" height="26" rx="5" fill="none" stroke="#98c379" stroke-width="2"/>
              <circle cx="-12" cy="16" r="4" fill="none" stroke="#98c379" stroke-width="2"/>
              <circle cx="12" cy="16" r="4" fill="none" stroke="#98c379" stroke-width="2"/>
              <text x="0" y="4" text-anchor="middle" fill="#98c379" font-size="8" font-weight="700">EV/H2</text>
              <text class="comp-label" x="0" y="33">${p.name||'交通需求'}</text>`;
    },
    ies_solar(p) {
      return `<polygon points="-18,-14 18,-14 14,12 -14,12" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <line x1="-7" y1="-4" x2="7" y2="-4" stroke="#e5c07b" stroke-width="1"/>
              <line x1="-5" y1="3" x2="5" y2="3" stroke="#e5c07b" stroke-width="1"/>
              <line x1="0" y1="12" x2="0" y2="28" stroke="#e5c07b" stroke-width="2"/>
              <text class="comp-label" x="0" y="43">${p.name||'光伏'}</text>
              <text class="comp-value" x="0" y="55">${p.rated_mw||0}MW</text>`;
    },
    ies_wind(p) {
      return `<line x1="0" y1="-2" x2="0" y2="28" stroke="#98c379" stroke-width="2"/>
              <circle cx="0" cy="-4" r="3" fill="#98c379"/>
              <path d="M0,-4 L0,-24 M0,-4 L18,6 M0,-4 L-18,6" stroke="#98c379" stroke-width="2"/>
              <text class="comp-label" x="0" y="43">${p.name||'风电'}</text>
              <text class="comp-value" x="0" y="55">${p.rated_mw||0}MW</text>`;
    },
    ies_chp(p) {
      return `<rect x="-21" y="-18" width="42" height="36" rx="4" fill="none" stroke="#e06c75" stroke-width="2"/>
              <text x="0" y="4" text-anchor="middle" fill="#e06c75" font-size="9" font-weight="700">CHP</text>
              <line x1="-21" y1="0" x2="-34" y2="0" stroke="#e5c07b" stroke-width="2"/>
              <line x1="21" y1="-6" x2="34" y2="-6" stroke="#61afef" stroke-width="2"/>
              <line x1="21" y1="8" x2="34" y2="8" stroke="#e06c75" stroke-width="2"/>
              <text class="comp-label" x="0" y="32">${p.name||'CHP'}</text>`;
    },
    ies_heat_pump(p) {
      return `<rect x="-20" y="-16" width="40" height="32" rx="4" fill="none" stroke="#e06c75" stroke-width="2"/>
              <text x="0" y="4" text-anchor="middle" fill="#e06c75" font-size="9" font-weight="700">HP</text>
              <line x1="-20" y="0" x2="-34" y2="0" stroke="#61afef" stroke-width="2"/>
              <line x1="20" y="0" x2="34" y2="0" stroke="#e06c75" stroke-width="2"/>
              <text class="comp-label" x="0" y="30">${p.name||'热泵'}</text>
              <text class="comp-value" x="0" y="42">COP ${p.cop||3.2}</text>`;
    },
    ies_electrolyzer(p) {
      return `<rect x="-22" y="-16" width="44" height="32" rx="4" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <text x="0" y="4" text-anchor="middle" fill="#56b6c2" font-size="9" font-weight="700">EL</text>
              <line x1="-22" y="0" x2="-34" y2="0" stroke="#61afef" stroke-width="2"/>
              <line x1="22" y="0" x2="34" y2="0" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <text class="comp-label" x="0" y="30">${p.name||'电解槽'}</text>`;
    },
    ies_fuel_cell(p) {
      return `<rect x="-22" y="-16" width="44" height="32" rx="4" fill="none" stroke="#56b6c2" stroke-width="2"/>
              <text x="0" y="4" text-anchor="middle" fill="#56b6c2" font-size="9" font-weight="700">FC</text>
              <line x1="-22" y="0" x2="-34" y2="0" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <line x1="22" y="0" x2="34" y2="0" stroke="#61afef" stroke-width="2"/>
              <text class="comp-label" x="0" y="30">${p.name||'燃料电池'}</text>`;
    },
    ies_electric_storage(p) {
      return `<rect x="-16" y="-14" width="32" height="24" rx="3" fill="none" stroke="#61afef" stroke-width="2"/>
              <line x1="-9" y1="-4" x2="9" y2="-4" stroke="#61afef" stroke-width="2"/>
              <line x1="0" y1="-9" x2="0" y2="1" stroke="#61afef" stroke-width="1.5"/>
              <line x1="0" y1="-30" x2="0" y2="-14" stroke="#61afef" stroke-width="2"/>
              <text class="comp-label" x="0" y="24">${p.name||'电储能'}</text>
              <text class="comp-value" x="0" y="36">${p.capacity_mwh||0}MWh</text>`;
    },
    ies_thermal_storage(p) {
      return `<rect x="-16" y="-14" width="32" height="24" rx="3" fill="none" stroke="#e06c75" stroke-width="2"/>
              <path d="M-7,2 C-3,-9 3,-9 7,2" fill="none" stroke="#e06c75" stroke-width="2"/>
              <line x1="0" y1="-30" x2="0" y2="-14" stroke="#e06c75" stroke-width="2"/>
              <text class="comp-label" x="0" y="24">${p.name||'热储能'}</text>
              <text class="comp-value" x="0" y="36">${p.capacity_mwh||0}MWh</text>`;
    },
    ies_hydrogen_storage(p) {
      return `<rect x="-18" y="-14" width="36" height="24" rx="8" fill="none" stroke="#56b6c2" stroke-width="2" stroke-dasharray="5 3"/>
              <text x="0" y="2" text-anchor="middle" fill="#56b6c2" font-size="8" font-weight="700">H2</text>
              <line x1="0" y1="-30" x2="0" y2="-14" stroke="#56b6c2" stroke-width="2" stroke-dasharray="4 2"/>
              <text class="comp-label" x="0" y="24">${p.name||'氢储能'}</text>
              <text class="comp-value" x="0" y="36">${p.capacity_mwh||0}MWh</text>`;
    },
    ies_ccus(p) {
      return `<rect x="-22" y="-16" width="44" height="32" rx="4" fill="none" stroke="#c678dd" stroke-width="2"/>
              <text x="0" y="4" text-anchor="middle" fill="#c678dd" font-size="8" font-weight="700">CCUS</text>
              <line x1="-22" y="0" x2="-34" y2="0" stroke="#c678dd" stroke-width="2"/>
              <line x1="22" y1="0" x2="34" y2="0" stroke="#61afef" stroke-width="2"/>
              <text class="comp-label" x="0" y="30">${p.name||'碳捕集'}</text>`;
    },
    ies_fuel_supply(p) {
      return `<path d="M-18,-16 H18 V16 H-18 Z" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <path d="M-8,8 C-4,-8 4,-8 8,8" fill="none" stroke="#e5c07b" stroke-width="2"/>
              <line x1="18" y1="0" x2="34" y2="0" stroke="#e5c07b" stroke-width="2" stroke-dasharray="5 3"/>
              <text class="comp-label" x="0" y="31">${p.name||'燃料供应'}</text>
              <text class="comp-value" x="0" y="43">${p.purchase_limit_mw||0}MW</text>`;
    },
  };

  // ============= Port Definitions =============
  // Ports are connection points relative to component origin
  const ports = {
    ac_bus:           [{id:'left',  x:-40, y:0}, {id:'right', x:40, y:0},
                       {id:'top',   x:0, y:-6}, {id:'bottom',x:0, y:6}],
    generator:        [{id:'top',   x:0, y:-30}],
    load:             [{id:'top',   x:0, y:-30}],
    transformer_2w:   [{id:'hv',    x:0, y:-30}, {id:'lv', x:0, y:30}],
    ac_branch:        [{id:'left',  x:-40, y:0}, {id:'right', x:40, y:0}],
    external_grid:    [{id:'bottom',x:0, y:30}],
    storage:          [{id:'top',   x:0, y:-30}],
    pv_system:        [{id:'top',   x:0, y:-30}],
    renewable_gen:    [{id:'top',   x:0, y:-30}],
    static_generator: [{id:'top',   x:0, y:-30}],
    vsc_converter:    [{id:'ac',    x:-30, y:0}, {id:'dc', x:30, y:0}],
    dc_bus:           [{id:'left',  x:-40, y:0}, {id:'right', x:40, y:0},
                       {id:'top',   x:0, y:-6}, {id:'bottom',x:0, y:6}],
    dc_branch:        [{id:'left',  x:-40, y:0}, {id:'right', x:40, y:0}],
    switch_comp:      [{id:'left',  x:-20, y:0}, {id:'right', x:20, y:0}],
    circuit_breaker:  [{id:'left',  x:-20, y:0}, {id:'right', x:20, y:0}],
    dc_load:          [{id:'top',   x:0, y:-30}],
    dc_pv_array:      [{id:'top',   x:0, y:-30}],
    dc_storage:       [{id:'top',   x:0, y:-30}],
    motor:            [{id:'top',   x:0, y:-30}],
    flexible_load:    [{id:'top',   x:0, y:-30}],
    asymmetric_load:  [{id:'top',   x:0, y:-30}],
    shunt:            [{id:'top',   x:0, y:-30}],
    transformer_3w:   [{id:'hv',    x:0, y:-30}, {id:'mv', x:-10, y:30}, {id:'lv', x:10, y:30}],
    charger:          [{id:'top',   x:0, y:-30}],
    charging_station: [{id:'top',   x:0, y:-30}],
    mobile_storage:   [{id:'top',   x:0, y:-30}],
    dcdc_converter:   [{id:'in',    x:-30, y:0}, {id:'out', x:30, y:0}],
    energy_router:    [{id:'ac_left', x:-34, y:0}, {id:'dc_right', x:34, y:0},
                       {id:'top',   x:0, y:-34}, {id:'bottom', x:0, y:34}],
    vpp:              [{id:'top',   x:0, y:-30}],
    microgrid:        [{id:'pcc',   x:0, y:-32}],
    ies_electric_bus: [{id:'left', x:-42, y:0}, {id:'right', x:42, y:0}, {id:'top', x:0, y:-6}, {id:'bottom', x:0, y:6}],
    ies_heat_bus:     [{id:'left', x:-42, y:0}, {id:'right', x:42, y:0}, {id:'top', x:0, y:-6}, {id:'bottom', x:0, y:6}],
    ies_hydrogen_bus: [{id:'left', x:-42, y:0}, {id:'right', x:42, y:0}, {id:'top', x:0, y:-6}, {id:'bottom', x:0, y:6}],
    ies_fuel_bus:     [{id:'left', x:-42, y:0}, {id:'right', x:42, y:0}, {id:'top', x:0, y:-6}, {id:'bottom', x:0, y:6}],
    ies_grid:         [{id:'electric', x:32, y:0}],
    ies_electric_load:[{id:'electric', x:0, y:-30}],
    ies_heat_load:    [{id:'heat', x:0, y:-30}],
    ies_hydrogen_load:[{id:'hydrogen', x:0, y:-30}],
    ies_fuel_load:    [{id:'fuel', x:0, y:-30}],
    ies_transport:    [{id:'electric', x:-22, y:0}, {id:'hydrogen', x:22, y:0}, {id:'fuel', x:0, y:-14}],
    ies_solar:        [{id:'electric', x:0, y:28}],
    ies_wind:         [{id:'electric', x:0, y:28}],
    ies_chp:          [{id:'fuel', x:-34, y:0}, {id:'electric', x:34, y:-6}, {id:'heat', x:34, y:8}],
    ies_heat_pump:    [{id:'electric', x:-34, y:0}, {id:'heat', x:34, y:0}],
    ies_electrolyzer: [{id:'electric', x:-34, y:0}, {id:'hydrogen', x:34, y:0}],
    ies_fuel_cell:    [{id:'hydrogen', x:-34, y:0}, {id:'electric', x:34, y:0}],
    ies_electric_storage: [{id:'electric', x:0, y:-30}],
    ies_thermal_storage:  [{id:'heat', x:0, y:-30}],
    ies_hydrogen_storage: [{id:'hydrogen', x:0, y:-30}],
    ies_ccus:         [{id:'co2', x:-34, y:0}, {id:'electric', x:34, y:0}],
    ies_fuel_supply:  [{id:'fuel', x:34, y:0}],
  };

  // ============= Default Parameters =============
  const defaults = {
    ac_bus: {
      name: 'Bus', bus_type: 'PQ', base_kv: 110,
      vm_pu: 1.0, va_deg: 0,
      vmin_pu: 0.9, vmax_pu: 1.1, gs_mw: 0, bs_mvar: 0,
      i_breaker_ka: 0, n_customers: 0, importance: 0,
      in_service: true, area: 1, zone: 1
    },
    generator: {
      name: 'Gen', bus: 0, pg_mw: 100, qg_mvar: 0, vg_pu: 1.0,
      pmax_mw: 200, pmin_mw: 0, qmax_mvar: 100, qmin_mvar: -100,
      mbase_mva: 100, is_slack: false, in_service: true,
      cost_c2: 0.02, cost_c1: 20, cost_c0: 0,
      startup_cost: 0, shutdown_cost: 0,
      ramp_up_mw_min: 0, ramp_dn_mw_min: 0,
      fuel_type: 'Thermal',
      emission_factor_tco2_mwh: 0,
      forced_outage_rate: 0.02, mttr_hr: 40, t_scheduled_hr: 0,
      dynamic_model: {
        standard: 'PSS/E',
        model_name: 'GENROU',
        parameter_set: 'default',
        components: [
          { type: 'machine', model: 'GENROU', standard: 'PSS/E', parameter_set: 'default', parameters: { H: 3.5, D: 0.0, Xd: 1.8, Xq: 1.7, Xdp: 0.3, Xqp: 0.55, Xdpp: 0.25, Xqpp: 0.25, Td0p: 8.0, Tq0p: 0.4, Td0pp: 0.03, Tq0pp: 0.05 } },
          { type: 'governor', model: 'TGOV1', standard: 'IEEE', parameter_set: 'default', parameters: { R: 0.05, T1: 0.5, T2: 2.0, T3: 7.0 } },
          { type: 'exciter', model: 'EXAC1', standard: 'IEEE4215', parameter_set: 'placeholder', parameters: {} }
        ],
        parameters: {}
      }
    },
    load: {
      name: 'Load', bus: 0, p_mw: 50, q_mvar: 20,
      scaling: 1.0, model: 'ConstantPower',
      z_percent_p: 0, i_percent_p: 0, p_percent_p: 100,
      z_percent_q: 0, i_percent_q: 0, p_percent_q: 100,
      controllable: false, p_min_mw: 0, cost_mw: 0,
      priority: 'Medium', n_customers: 0, profile_id: -1,
      dynamic_model: {
        standard: 'PSS/E',
        model_name: 'ZIP',
        parameter_set: 'default',
        parameters: { z_percent_p: 0.0, i_percent_p: 0.0, p_percent_p: 100.0, z_percent_q: 0.0, i_percent_q: 0.0, p_percent_q: 100.0 }
      },
      in_service: true
    },
    transformer_2w: {
      name: 'Trafo', hv_bus: 0, lv_bus: 0,
      sn_mva: 100, vn_hv_kv: 220, vn_lv_kv: 110,
      vk_percent: 12, vkr_percent: 0.5,
      pfe_kw: 30, i0_percent: 0.1,
      shift_deg: 0, tap_side: 0,
      tap_pos: 0, tap_min: -8, tap_max: 8,
      tap_neutral: 0, tap_step_percent: 1.25,
      vector_group: '',
      in_service: true
    },
    ac_branch: {
      name: 'Line', from_bus: 0, to_bus: 0,
      r_pu: 0.01, x_pu: 0.1, b_pu: 0,
      r_ohm_per_km: 0, x_ohm_per_km: 0, b_us_per_km: 0, c_nf_per_km: 0,
      rate_a_mva: 100, rate_b_mva: 0, rate_c_mva: 0,
      length_km: 0,
      tap: 1.0, shift_deg: 0, in_service: true, n_parallel: 1,
      failure_rate: 0, mttr_hr: 0
    },
    external_grid: {
      name: 'Grid', bus: 0, vm_pu: 1.05, va_deg: 0,
      s_sc_max_mva: 10000, s_sc_min_mva: 8000,
      rx_max: 0.1, rx_min: 0.1,
      r_pu: 0, x_pu: 0, r0_pu: 0, x0_pu: 0,
      vn_kv: 0, emission_factor_tco2_mwh: 0,
      dynamic_model: { standard: 'PSS/E', model_name: 'ExternalGrid', parameter_set: 'default' },
      controllable: false, in_service: true
    },
    storage: {
      name: 'ESS', bus: 0, p_mw: 0, q_mvar: 0,
      p_rated_mw: 10,
      e_rated_mwh: 40, soc_init: 0.5, soc_min: 0.1, soc_max: 0.9,
      eta_charge: 0.95, eta_discharge: 0.95,
      pmax_mw: 10, pmin_mw: -10,
      qmax_mvar: 0, qmin_mvar: 0,
      self_discharge_pct: 0, profile_id: -1,
      charge_bid_price: 0, discharge_bid_price: 0, daily_cycle_limit: 0,
      forced_outage_rate: 0.02, mttr_hr: 24, t_scheduled_hr: 0,
      dynamic_model: {
        standard: 'IEEE1547',
        model_name: 'BatteryDynamic',
        parameter_set: 'default',
        components: [
          { type: 'plant', model: 'BatteryDynamic', standard: 'HACDCPF', parameter_set: 'default', parameters: {} }
        ],
        parameters: {}
      },
      in_service: true
    },
    pv_system: {
      name: 'PV', bus: 0, p_mw: 5, q_mvar: 0,
      sn_mva: 6, p_rated_mw: 5,
      pmax_mw: 0, pmin_mw: 0, qmax_mvar: 0, qmin_mvar: 0,
      control_mode: 'MPPT', controllable: false,
      v_ac_set_pu: 1.0, v_dc_set_pu: 1.0,
      inverter_eff: 0.97, loss_percent: 0,
      num_series: 0, num_parallel: 0,
      vmpp: 0, impp: 0, voc: 0, isc: 0,
      alpha_isc: 0, beta_voc: 0,
      irradiance: 1000, temperature: 25, profile_id: -1,
      mtbf_hours: 0, mttr_hours: 0, t_scheduled_hr: 0,
      dynamic_model: {
        standard: 'IEEE1547',
        model_name: 'PVDynamic',
        parameter_set: 'default',
        components: [
          { type: 'plant', model: 'PVDynamic', standard: 'HACDCPF', parameter_set: 'default', parameters: {} },
          { type: 'inverter', model: 'GridFollowingInverter', standard: 'NERC', parameter_set: 'REGC_A_REEC_A_subset', parameters: { pll_kp: 0.02, pll_ki: 1.0 } }
        ],
        parameters: {}
      },
      in_service: true
    },
    renewable_gen: {
      name: 'Wind', bus: 0, type: 'Wind',
      p_mw: 20, q_mvar: 0, p_rated_mw: 30,
      qmax_mvar: 0, qmin_mvar: 0,
      curtailable: true, cost_curtail_mwh: 0,
      capacity_factor: 0.3, profile_id: -1,
      mtbf_hours: 0, mttr_hours: 0, t_scheduled_hr: 0,
      emission_offset_tco2_mwh: 0, in_service: true
    },
    static_generator: {
      name: 'SGen', bus: 0, p_mw: 5, q_mvar: 0,
      sgen_type: 'PV', p_rated_mw: 0, sn_mva: 0,
      pmax_mw: 0, pmin_mw: 0, qmax_mvar: 0, qmin_mvar: 0,
      scaling: 1.0, controllable: false, v_ref_pu: 0,
      profile_id: -1,
      cost_c1: 0, emission_factor_tco2_mwh: 0,
      mtbf_hours: 0, mttr_hours: 0, t_scheduled_hr: 0,
      dynamic_model: { standard: 'HACDCPF', model_name: 'StaticGeneratorInjection', parameter_set: 'default' },
      in_service: true
    },
    vsc_converter: {
      name: 'VSC', bus_ac: 0, bus_dc: 0,
      control_mode: 'PQ_MODE', type: 'two_level',
      p_set_mw: 100, q_set_mvar: 0,
      p_is_hard_constraint: false, p_schedule_mw: 0, p_initial_mw: 0,
      pmax_mw: 200, pmin_mw: -200,
      qmax_mvar: 100, qmin_mvar: -100,
      eta: 0.98, loss_percent: 1.0, loss_mw: 0,
      v_dc_set_pu: 1.0, v_ac_set_pu: 1.0, v_ac_angle_set_deg: 0,
      k_vdc: 0.1, p_rated_mw: 0,
      r_conv_ac_pu: 0,
      r_sc_pu: 0, x_sc_pu: 0.15,
      r2_sc_pu: 0, x2_sc_pu: 0,
      i_max_pu: 1.0,
      i_ac_max_pu: 0, i_dc_max_pu: 0,
      k_m_modulation: 0, m_min: 0, m_max: 0,
      vn_ac_kv: 0, vn_dc_kv: 0,
      grid_forming: false, ac_grid_forming: false,
      allow_dual_side_grid_forming: false, has_energy_buffer: false,
      coordination_group_id: '', is_master: false, participation_factor: 0,
      dynamic_model: {
        standard: 'NERC',
        model_name: 'GridFollowingInverter',
        parameter_set: 'REGC_A_REEC_A_subset',
        components: [
          { type: 'pll', model: 'KauraPLL', standard: 'PSD', parameter_set: 'default', parameters: { kp_pll: 0.02, ki_pll: 1.0 } },
          { type: 'current_control', model: 'GFLCurrentControl', standard: 'NERC', parameter_set: 'default', parameters: { i_max_pu: 1.0 } }
        ],
        parameters: {}
      },
      in_service: true
    },
    dc_bus: {
      name: 'DC Bus', bus_type: 'DC_P', base_kv: 320,
      vm_pu: 1.0, vmax_pu: 1.1, vmin_pu: 0.9, pd_mw: 0,
      in_service: true
    },
    dc_branch: {
      name: 'DC Line', from_bus: 0, to_bus: 0,
      r_pu: 0.01, rate_a_mva: 200, length_km: 100,
      in_service: true
    },
    switch_comp: {
      name: 'Switch', from_bus: 0, to_bus: 0,
      switch_type: '', closed: true,
      r_contact_ohm: 0, z_ohm: 0,
      i_rated_ka: 0, i_breaking_ka: 0,
      in_service: true
    },
    circuit_breaker: {
      name: 'CB', from_bus: 0, to_bus: 0,
      breaker_type: '', closed: true,
      z_ohm: 0, rated_voltage_kv: 0,
      i_rated_ka: 0, i_breaking_ka: 0,
      rated_current_ka: 2.0, in_service: true
    },
    dc_load: {
      name: 'DC Load', bus: 0, p_mw: 10,
      scaling: 1.0, controllable: false,
      p_min_mw: 0, cost_mw: 0, profile_id: -1,
      dynamic_model: { standard: 'HACDCPF', model_name: 'DCDynamicLoad', parameter_set: 'constant_power' },
      in_service: true
    },
    dc_storage: {
      name: 'DC ESS', bus: 0, p_mw: 0,
      p_rated_mw: 10,
      e_rated_mwh: 40, soc_init: 0.5, soc_min: 0.1, soc_max: 0.9,
      eta_charge: 0.95, eta_discharge: 0.95,
      pmax_mw: 10, pmin_mw: -10,
      self_discharge_pct: 0, profile_id: -1,
      charge_bid_price: 0, discharge_bid_price: 0, daily_cycle_limit: 0,
      forced_outage_rate: 0.02, mttr_hr: 24, t_scheduled_hr: 0,
      dynamic_model: { standard: 'IEEE1547', model_name: 'BatteryDynamic', parameter_set: 'dc_default' },
      in_service: true
    },
    dc_pv_array: {
      name: 'DC PV', bus: 0, p_set_mw: 5,
      irradiance: 1000, temperature: 25,
      num_series: 0, num_parallel: 0,
      vmpp: 0, impp: 0, voc: 0, isc: 0,
      alpha_isc: 0, beta_voc: 0,
      profile_id: -1,
      mtbf_hours: 0, mttr_hours: 0, t_scheduled_hr: 0,
      dynamic_model: { standard: 'IEEE1547', model_name: 'PVDynamic', parameter_set: 'dc_default' },
      in_service: true
    },
    motor: {
      name: 'Motor', bus: 0,
      vn_kv: 6.3, sn_mva: 5, r_pu: 0.02, x_pu: 0.15,
      x_r: 0, lrc: 0, poles: 0,
      cos_phi: 0.85, efficiency: 0.94,
      r0_pu: 0, x0_pu: 0, in_service: true
    },
    flexible_load: {
      name: 'FlexLoad', bus: 0, p_mw: 20, q_mvar: 5,
      flex_up_mw: 5, flex_down_mw: 5, flex_duration_h: 4,
      response_time_s: 30, ramp_rate_mw_min: 2, availability_pct: 100,
      controllable: true, priority: 'Medium', in_service: true
    },
    asymmetric_load: {
      name: 'AsymLoad', bus: 0, connection: 'wye', grounded: true,
      pa_mw: 10, qa_mvar: 3, pb_mw: 10, qb_mvar: 3, pc_mw: 10, qc_mvar: 3,
      scaling: 1.0, const_z_percent: 0, const_i_percent: 0, const_p_percent: 100,
      dynamic_model: { standard: 'PSS/E', model_name: 'ZIP', parameter_set: 'per_phase' },
      controllable: false, priority: 'Medium', in_service: true
    },
    shunt: {
      name: 'Shunt', bus: 0, gs_mw: 0, bs_mvar: 10,
      switchable: false, n_steps: 1, current_step: 1, bs_per_step: 0,
      in_service: true
    },
    transformer_3w: {
      name: 'Trafo3W', hv_bus: 0, mv_bus: 0, lv_bus: 0,
      sn_hv_mva: 100, sn_mv_mva: 50, sn_lv_mva: 25,
      vn_hv_kv: 220, vn_mv_kv: 110, vn_lv_kv: 35,
      vk_hv_mv_percent: 12, vk_hv_lv_percent: 12, vk_mv_lv_percent: 10,
      vkr_hv_mv_percent: 0.5, vkr_hv_lv_percent: 0.5, vkr_mv_lv_percent: 0.4,
      pfe_kw: 30, i0_percent: 0.1,
      tap_side: 0, tap_pos: 0, tap_step_percent: 0,
      shift_mv_deg: 0, shift_lv_deg: 0,
      in_service: true
    },
    charger: {
      name: 'Charger', station_id: 0, charger_type: 'AC_L2',
      p_rated_kw: 7, p_ch_max_kw: 7, p_ch_min_kw: 0,
      eta: 0.95, v2g_capable: false, p_dis_max_kw: 0,
      in_service: true
    },
    charging_station: {
      name: 'EVStation', bus: 0, location: '',
      n_fast: 4, n_slow: 8, num_chargers: 12,
      p_fast_max_kw: 120, p_slow_max_kw: 7, max_power_kw: 600,
      simultaneity_factor: 0.7, power_factor: 0.95,
      utilization_rate: 0.3, p_total_kw: 0, q_total_kvar: 0,
      in_service: true
    },
    mobile_storage: {
      name: 'MobESS', bus: 0, p_mw: 0, q_mvar: 0,
      p_rated_mw: 2, e_rated_mwh: 4, pmax_mw: 2, pmin_mw: -2,
      qmax_mvar: 0, qmin_mvar: 0,
      soc_init: 0.5, soc_min: 0.1, soc_max: 0.9,
      eta_charge: 0.95, eta_discharge: 0.95,
      is_mobile: true, status: 'Stationary', target_bus: 0,
      in_service: true
    },
    dcdc_converter: {
      name: 'DC/DC', bus_in: 0, bus_out: 0,
      control_mode: 'Voltage', p_ref_mw: 0, v_ref_pu: 1.0,
      sn_mva: 50, vn_in_kv: 320, vn_out_kv: 160,
      eta: 0.98, r_eq_pu: 0.01, pmax_mw: 50, pmin_mw: -50,
      k_droop: 0.05,
      topology: 'Generic', d_min: 0.05, d_max: 0.95, n_ratio: 1.0,
      dynamic_model: {
        standard: 'HACDCPF',
        model_name: 'DCDCConverterDynamic',
        parameter_set: 'default',
        parameters: { tau_s: 0.02 }
      },
      in_service: true
    },
    energy_router: {
      name: 'ERouter', router_type: 'hybrid', num_ports: 4,
      p_rated_mw: 10, vn_ac_kv: 10, vn_dc_kv: 20,
      loss_percent: 1.0, pmax_mw: 10, pmin_mw: -10,
      qmax_mvar: 5, qmin_mvar: -5, in_service: true,
      // Port parameters (4 ports: AC/DC type determines which bus map is used)
      port1_bus: 0, port1_type: 'AC', port1_side: 0, port1_control_mode: 'VF',
      port1_p_set_mw: 0, port1_q_set_mvar: 0, port1_v_set_pu: 1.0, port1_eta: 0.98,
      port2_bus: 0, port2_type: 'AC', port2_side: 0, port2_control_mode: 'PQ',
      port2_p_set_mw: 0, port2_q_set_mvar: 0, port2_v_set_pu: 1.0, port2_eta: 0.98,
      port3_bus: 0, port3_type: 'AC', port3_side: 1, port3_control_mode: 'PQ',
      port3_p_set_mw: 0, port3_q_set_mvar: 0, port3_v_set_pu: 1.0, port3_eta: 0.98,
      port4_bus: 0, port4_type: 'AC', port4_side: 1, port4_control_mode: 'PQ',
      port4_p_set_mw: 0, port4_q_set_mvar: 0, port4_v_set_pu: 1.0, port4_eta: 0.98
    },
    vpp: {
      name: 'VPP', pcc_bus: 0, description: '',
      // Aggregated resources (lists of component IDs aggregated by this VPP)
      aggregated_gen_ids: [], aggregated_storage_ids: [], aggregated_load_ids: [],
      n_pv_systems: 0, n_wind_turbines: 0, n_battery_systems: 0,
      n_ev_chargers: 0, n_controllable_loads: 0,
      n_chp: 0, n_biomass: 0, n_thermal_storage: 0, n_hvac: 0, n_industrial: 0,
      p_generation_sum_mw: 0, e_storage_sum_mwh: 0,
      p_load_controllable_mw: 0, p_pv_sum_mw: 0, p_wind_sum_mw: 0,
      p_regulation_up_mw: 0, p_regulation_down_mw: 0,
      p_output_mw: 0, q_output_mvar: 0,
      pmax_mw: 0, pmin_mw: 0, ramp_up_max_mw_min: 0, ramp_down_max_mw_min: 0,
      mtbf_hr: 0, mttr_hr: 0, t_scheduled_hr: 0,
      in_service: true
    },
    microgrid: {
      name: 'MicroGrid', pcc_bus: 0, description: '',
      operating_mode: 'GridConnected', islanding_capability: false,
      auto_reconnection: false, p_exchange_max_mw: 10, p_exchange_min_mw: -10,
      p_import_max_mw: 10, p_export_max_mw: 10, p_exchange_mw: 0,
      total_generation_mw: 0, total_storage_mwh: 0, total_load_mw: 0,
      capacity_mw: 0, peak_load_mw: 0, f_set_hz: 50, v_set_pu: 1.0,
      k_droop: 0.05, area: 0, in_service: true
    },
    ies_electric_bus: { name: '电母线', carrier: 'electricity', in_service: true },
    ies_heat_bus: { name: '热母线', carrier: 'heat', in_service: true },
    ies_hydrogen_bus: { name: '氢母线', carrier: 'hydrogen', in_service: true },
    ies_fuel_bus: { name: '燃料母线', carrier: 'fuel', in_service: true },
    ies_grid: {
      name: '电网接口', import_limit_mw: 16, export_limit_mw: 5,
      buy_price_per_mwh: 90, sell_price_per_mwh: 35,
      carbon_tco2_mwh: 0.58,
      _ies_buy_price_profile_values: [], _ies_sell_price_profile_values: [],
      _ies_carbon_profile_values: [], _ies_profile_step_duration_hr: 1,
      in_service: true
    },
    ies_electric_load: {
      name: '电负荷', demand_mw: 8,
      _ies_profile_name: '', _ies_profile_values: [], _ies_profile_step_duration_hr: 1,
      in_service: true
    },
    ies_heat_load: {
      name: '热负荷', demand_mw: 4,
      _ies_profile_name: '', _ies_profile_values: [], _ies_profile_step_duration_hr: 1,
      in_service: true
    },
    ies_hydrogen_load: {
      name: '氢负荷', demand_mw: 0.2,
      _ies_profile_name: '', _ies_profile_values: [], _ies_profile_step_duration_hr: 1,
      in_service: true
    },
    ies_fuel_load: {
      name: '燃料负荷', demand_mw: 0.1,
      _ies_profile_name: '', _ies_profile_values: [], _ies_profile_step_duration_hr: 1,
      in_service: true
    },
    ies_transport: {
      name: '交通需求', demand_km_per_h: 120,
      ev_ratio: 0.45, hv_ratio: 0.20, icv_ratio: 0.35,
      alpha_ev_mwh_per_km: 0.00018,
      alpha_hv_mwh_per_km: 0.00060,
      alpha_icv_mwh_per_km: 0.00075,
      _ies_profile_name: '', _ies_profile_values: [], _ies_profile_step_duration_hr: 1,
      in_service: true
    },
    ies_solar: {
      name: '光伏', rated_mw: 8, om_cost_per_mwh: 2,
      availability_scale: 1.0,
      _ies_profile_name: '', _ies_profile_values: [], _ies_profile_step_duration_hr: 1,
      in_service: true
    },
    ies_wind: {
      name: '风电', rated_mw: 4, om_cost_per_mwh: 3,
      availability_scale: 1.0,
      _ies_profile_name: '', _ies_profile_values: [], _ies_profile_step_duration_hr: 1,
      in_service: true
    },
    ies_chp: {
      name: 'CHP', power_max_mw: 4, heat_max_mw: 6,
      eta_elec: 0.35, eta_heat: 0.45, eta_total: 0.82,
      om_cost_per_mwh: 5, in_service: true
    },
    ies_heat_pump: {
      name: '热泵', power_max_mw: 3, cop: 3.2,
      om_cost_per_mwh: 1, in_service: true
    },
    ies_electrolyzer: {
      name: '电解槽', power_max_mw: 3, eta: 0.65,
      om_cost_per_mwh: 2, in_service: true
    },
    ies_fuel_cell: {
      name: '燃料电池', power_max_mw: 2, eta: 0.52,
      om_cost_per_mwh: 4, in_service: true
    },
    ies_electric_storage: {
      name: '电储能', capacity_mwh: 10, initial_mwh: 2,
      charge_max_mw: 4, discharge_max_mw: 4,
      eta_charge: 0.95, eta_discharge: 0.95,
      retention: 0.999, throughput_cost_per_mwh: 1,
      in_service: true
    },
    ies_thermal_storage: {
      name: '热储能', capacity_mwh: 8, initial_mwh: 2,
      charge_max_mw: 3, discharge_max_mw: 3,
      eta_charge: 0.95, eta_discharge: 0.95,
      retention: 0.995, throughput_cost_per_mwh: 1,
      in_service: true
    },
    ies_hydrogen_storage: {
      name: '氢储能', storage_layer: 'daily',
      capacity_mwh: 8, initial_mwh: 2,
      charge_max_mw: 2, discharge_max_mw: 2,
      retention: 0.999, throughput_cost_per_mwh: 1.5,
      in_service: true
    },
    ies_ccus: {
      name: '碳捕集', capture_fraction: 0.85,
      max_tco2_per_h: 3, power_mwh_per_tco2: 0.12,
      cost_per_tco2: 35, in_service: true
    },
    ies_fuel_supply: {
      name: '燃料供应', purchase_limit_mw: 30,
      cost_per_mwh: 38, carbon_tco2_mwh: 0.27,
      in_service: true
    },
  };

  // ============= Component Categories for Library Panel =============
  const categories = {
    acComponents: [
      { type: 'ac_bus',           label: '交流母线' },
      { type: 'generator',        label: '发电机' },
      { type: 'load',             label: '负荷' },
      { type: 'flexible_load',    label: '柔性负荷' },
      { type: 'asymmetric_load',  label: '不对称负荷' },
      { type: 'ac_branch',        label: '线路' },
      { type: 'transformer_2w',   label: '双绕组变压器' },
      { type: 'transformer_3w',   label: '三绕组变压器' },
      { type: 'external_grid',    label: '外部电网' },
      { type: 'static_generator', label: '分布式电源' },
      { type: 'shunt',            label: '并联补偿' },
      { type: 'motor',            label: '电动机' },
    ],
    dcComponents: [
      { type: 'dc_bus',       label: 'DC母线' },
      { type: 'dc_branch',    label: 'DC线路' },
      { type: 'dc_load',      label: 'DC负荷' },
      { type: 'dc_pv_array',  label: 'DC光伏' },
      { type: 'dc_storage',   label: 'DC储能' },
    ],
    converterComponents: [
      { type: 'vsc_converter',    label: 'VSC换流器' },
      { type: 'dcdc_converter',   label: 'DC/DC变换器' },
      { type: 'energy_router',    label: '能量路由器' },
      { type: 'switch_comp',      label: '开关' },
      { type: 'circuit_breaker',  label: '断路器' },
    ],
    renewableComponents: [
      { type: 'storage',           label: '储能' },
      { type: 'mobile_storage',    label: '移动储能' },
      { type: 'pv_system',         label: '光伏' },
      { type: 'renewable_gen',     label: '风电/可再生' },
    ],
    evComponents: [
      { type: 'charger',           label: '充电桩' },
      { type: 'charging_station',  label: '充电站' },
    ],
    aggregationComponents: [
      { type: 'vpp',               label: '虚拟电厂' },
      { type: 'microgrid',         label: '微电网' },
    ],
    integratedEnergyComponents: [
      { type: 'ies_electric_bus',      label: '电母线' },
      { type: 'ies_heat_bus',          label: '热母线' },
      { type: 'ies_hydrogen_bus',      label: '氢母线' },
      { type: 'ies_fuel_bus',          label: '燃料母线' },
      { type: 'ies_grid',              label: '电网接口' },
      { type: 'ies_electric_load',     label: '电负荷' },
      { type: 'ies_heat_load',         label: '热负荷' },
      { type: 'ies_hydrogen_load',     label: '氢负荷' },
      { type: 'ies_transport',         label: '交通需求' },
      { type: 'ies_solar',             label: '光伏' },
      { type: 'ies_wind',              label: '风电' },
      { type: 'ies_chp',               label: 'CHP' },
      { type: 'ies_heat_pump',         label: '热泵' },
      { type: 'ies_electrolyzer',      label: '电解槽' },
      { type: 'ies_fuel_cell',         label: '燃料电池' },
      { type: 'ies_electric_storage',  label: '电储能' },
      { type: 'ies_thermal_storage',   label: '热储能' },
      { type: 'ies_hydrogen_storage',  label: '氢储能' },
      { type: 'ies_ccus',              label: '碳捕集' },
      { type: 'ies_fuel_supply',       label: '燃料供应' },
    ],
  };

  // ============= Human-readable field labels =============
  const fieldLabels = {
    name: '名称', bus_type: '节点类型', base_kv: '基准电压(kV)',
    vm_pu: '电压幅值(pu)', va_deg: '电压相角(°)', pd_mw: '有功负荷(MW)',
    qd_mvar: '无功负荷(MVar)', vmin_pu: '最小电压(pu)', vmax_pu: '最大电压(pu)',
    gs_mw: '对地电导(MW)', bs_mvar: '对地电纳(MVar)', in_service: '投运',
    area: '区域', zone: '分区', bus: '所连母线', pg_mw: '有功出力(MW)',
    qg_mvar: '无功出力(MVar)', vg_pu: '端电压(pu)', pmax_mw: '最大有功(MW)',
    pmin_mw: '最小有功(MW)', qmax_mvar: '最大无功(MVar)', qmin_mvar: '最小无功(MVar)',
    mbase_mva: '容量基准(MVA)', is_slack: '平衡节点', cost_c2: '成本系数c2',
    cost_c1: '成本系数c1', cost_c0: '成本系数c0', fuel_type: '燃料类型',
    emission_factor_tco2_mwh: '碳排放因子(kg/MWh)', dynamic_model: '动态模型(JSON)',
    p_mw: '有功(MW)', q_mvar: '无功(MVar)', scaling: '缩放因子',
    model: '负荷模型', priority: '优先级',
    hv_bus: '高压侧母线', lv_bus: '低压侧母线', sn_mva: '额定容量(MVA)',
    vn_hv_kv: '高压侧电压(kV)', vn_lv_kv: '低压侧电压(kV)',
    vk_percent: '短路阻抗(%)', vkr_percent: '短路电阻(%)',
    pfe_kw: '空载损耗(kW)', i0_percent: '空载电流(%)',
    shift_deg: '移相角(°)', tap_pos: '档位', tap_min: '最小档位',
    tap_max: '最大档位', tap_step_percent: '档位步长(%)',
    from_bus: '起始母线', to_bus: '终止母线',
    r_pu: '电阻(pu)', x_pu: '电抗(pu)', b_pu: '电纳(pu)',
    r_ohm_per_km: '电阻(Ω/km)', x_ohm_per_km: '电抗(Ω/km)',
    b_us_per_km: '电纳(μS/km)', c_nf_per_km: '电容(nF/km)',
    rate_a_mva: '额定容量(MVA)', length_km: '长度(km)',
    failure_rate: '故障率(次/年)', forced_outage_rate: '强迫停运率',
    mttr_hr: '平均修复时间(h)', mttr_hours: '平均修复时间(h)',
    mtbf_hr: '平均无故障时间(h)', mtbf_hours: '平均无故障时间(h)',
    t_scheduled_hr: '计划检修时间(h)',
    tap: '变比', n_parallel: '并联数', s_sc_max_mva: '最大短路容量(MVA)',
    s_sc_min_mva: '最小短路容量(MVA)', rx_max: 'R/X(max)', rx_min: 'R/X(min)',
    p_rated_mw: '额定功率(MW)', e_rated_mwh: '额定能量(MWh)',
    soc_init: '初始SOC', soc_min: '最小SOC', soc_max: '最大SOC',
    eta_charge: '充电效率', eta_discharge: '放电效率',
    charge_bid_price: '充电成本($/MWh)', discharge_bid_price: '放电成本($/MWh)',
    daily_cycle_limit: '日循环上限',
    type: '类型', curtailable: '可削减', capacity_factor: '容量因子',
    sgen_type: '类型', controllable: '可控',
    bus_ac: 'AC侧母线', bus_dc: 'DC侧母线', control_mode: '控制模式',
    p_set_mw: '有功设定(MW)', q_set_mvar: '无功设定(MVar)',
    p_is_hard_constraint: 'P硬约束', p_schedule_mw: 'P计划(MW)', p_initial_mw: 'P初值(MW)',
    r_conv_ac_pu: 'AC侧等效电阻(pu)',
    r_sc_pu: '短路电阻Rsc(pu)', x_sc_pu: '短路电抗Xsc(pu)',
    r2_sc_pu: '负序短路电阻R2(pu)', x2_sc_pu: '负序短路电抗X2(pu)',
    i_max_pu: 'IEC短路电流倍数(pu)',
    i_ac_max_pu: 'AC电流上限(pu)', i_dc_max_pu: 'DC电流上限(pu)',
    k_m_modulation: '调制系数 k_m', m_min: '调制比下限 m_min', m_max: '调制比上限 m_max',
    vn_ac_kv: 'AC额定电压(kV)', vn_dc_kv: 'DC额定电压(kV)',
    grid_forming: 'DC侧构网', ac_grid_forming: 'AC侧构网',
    allow_dual_side_grid_forming: '允许双侧构网', has_energy_buffer: '含储能缓冲',
    coordination_group_id: '协调组ID', is_master: '主换流器', participation_factor: '参与因子',
    topology: 'DC/DC拓扑', d_min: '占空比下限 d_min', d_max: '占空比上限 d_max', n_ratio: '变压比 n',
    irradiance: '辐照度(W/m²)', temperature: '温度(℃)',
    num_series: '串联数', num_parallel: '并联数',
    vmpp: 'MPP电压(V)', impp: 'MPP电流(A)',
    voc: '开路电压(V)', isc: '短路电流(A)',
    alpha_isc: 'Isc温度系数(%/℃)', beta_voc: 'Voc温度系数(%/℃)',
    pmax_mw: '最大有功(MW)', pmin_mw: '最小有功(MW)',
    eta: '效率', loss_percent: '损耗(%)',
    v_dc_set_pu: 'DC电压设定(pu)', v_ac_set_pu: 'AC电压设定(pu)',
    v_ac_angle_set_deg: 'AC构网角度设定(°)',
    port_type: '端口类型', port1_type: '端口1类型', port2_type: '端口2类型',
    port3_type: '端口3类型', port4_type: '端口4类型',
    closed: '合闸状态', rated_current_ka: '额定电流(kA)',
    vn_kv: '额定电压(kV)', cos_phi: '功率因数', efficiency: '效率',
    // Flexible Load
    flex_up_mw: '上调容量(MW)', flex_down_mw: '下调容量(MW)',
    flex_duration_h: '响应持续(h)', response_time_s: '响应时间(s)',
    ramp_rate_mw_min: '爬坡速率(MW/min)', availability_pct: '可用率(%)',
    control_area: '控制区域',
    // Asymmetric Load
    connection: '接线方式', grounded: '接地',
    pa_mw: 'A相有功(MW)', qa_mvar: 'A相无功(MVar)',
    pb_mw: 'B相有功(MW)', qb_mvar: 'B相无功(MVar)',
    pc_mw: 'C相有功(MW)', qc_mvar: 'C相无功(MVar)',
    const_z_percent: '恒阻抗比(%)', const_i_percent: '恒电流比(%)', const_p_percent: '恒功率比(%)',
    // Shunt
    switchable: '可投切', n_steps: '步数', current_step: '当前步', bs_per_step: '每步电纳',
    // Transformer 3W
    hv_bus: '高压侧母线', mv_bus: '中压侧母线', lv_bus: '低压侧母线',
    sn_hv_mva: 'HV容量(MVA)', sn_mv_mva: 'MV容量(MVA)', sn_lv_mva: 'LV容量(MVA)',
    vn_mv_kv: '中压侧电压(kV)',
    vk_hv_mv_percent: 'Vk_HV-MV(%)', vk_hv_lv_percent: 'Vk_HV-LV(%)', vk_mv_lv_percent: 'Vk_MV-LV(%)',
    vkr_hv_mv_percent: 'Vkr_HV-MV(%)', vkr_hv_lv_percent: 'Vkr_HV-LV(%)', vkr_mv_lv_percent: 'Vkr_MV-LV(%)',
    // Charger & Charging Station
    station_id: '充电站ID', charger_type: '充电桩类型',
    p_rated_kw: '额定功率(kW)', p_ch_max_kw: '最大充电(kW)', p_ch_min_kw: '最小充电(kW)',
    v2g_capable: 'V2G能力', p_dis_max_kw: '最大放电(kW)',
    location: '位置', n_fast: '快充桩数', n_slow: '慢充桩数', num_chargers: '充电桩总数',
    p_fast_max_kw: '快充最大功率(kW)', p_slow_max_kw: '慢充最大功率(kW)', max_power_kw: '总最大功率(kW)',
    simultaneity_factor: '同时率', power_factor: '功率因数',
    utilization_rate: '利用率', p_total_kw: '总有功(kW)', q_total_kvar: '总无功(kVar)',
    // Mobile Storage
    is_mobile: '移动式', status: '状态', target_bus: '目标母线',
    // DC/DC Converter
    bus_in: '输入侧母线', bus_out: '输出侧母线',
    p_ref_mw: '功率参考(MW)', v_ref_pu: '电压参考(pu)',
    vn_in_kv: '输入电压(kV)', vn_out_kv: '输出电压(kV)',
    r_eq_pu: '等效电阻(pu)', k_droop: '下垂系数',
    // Energy Router
    router_type: '路由器类型', num_ports: '端口数',
    vn_ac_kv: 'AC电压(kV)', vn_dc_kv: 'DC电压(kV)',
    // VPP
    pcc_bus: 'PCC母线', aggregation_bus: 'PCC母线', description: '描述',
    aggregated_gen_ids: '聚合发电机ID列表', aggregated_storage_ids: '聚合储能ID列表', aggregated_load_ids: '聚合负荷ID列表',
    n_pv_systems: '光伏数', n_wind_turbines: '风机数', n_battery_systems: '电池数',
    n_ev_chargers: '充电桩数', n_controllable_loads: '可控负荷数',
    p_generation_sum_mw: '总发电(MW)', e_storage_sum_mwh: '总储能(MWh)',
    p_load_controllable_mw: '可控负荷(MW)', p_output_mw: '输出功率(MW)', q_output_mvar: '输出无功(MVar)',
    ramp_up_max_mw_min: '最大升坡(MW/min)', ramp_down_max_mw_min: '最大降坡(MW/min)',
    // Microgrid
    pcc_bus: 'PCC母线', operating_mode: '运行模式',
    islanding_capability: '孤岛能力', auto_reconnection: '自动重合闸',
    p_exchange_max_mw: '最大交换功率(MW)', p_exchange_min_mw: '最小交换功率(MW)',
    p_import_max_mw: '最大输入(MW)', p_export_max_mw: '最大输出(MW)',
    p_exchange_mw: '交换功率(MW)',
    total_generation_mw: '总发电(MW)', total_storage_mwh: '总储能(MWh)',
    total_load_mw: '总负荷(MW)', capacity_mw: '容量(MW)', peak_load_mw: '峰值负荷(MW)',
    f_set_hz: '频率设定(Hz)', v_set_pu: '电压设定(pu)',
    // Integrated energy canvas
    carrier: '能源载体', demand_mw: '需求(MW)', demand_km_per_h: '交通需求(km/h)',
    import_limit_mw: '购电上限(MW)', export_limit_mw: '售电上限(MW)',
    buy_price_per_mwh: '购电价格', sell_price_per_mwh: '售电价格',
    carbon_tco2_mwh: '碳因子(tCO2/MWh)', rated_mw: '额定功率(MW)',
    availability_scale: '可用容量倍率', power_max_mw: '功率上限(MW)',
    heat_max_mw: '供热上限(MW)', eta_elec: '发电效率', eta_heat: '供热效率',
    eta_total: '总效率', om_cost_per_mwh: '运维成本/MWh', cop: 'COP',
    capacity_mwh: '容量(MWh)', initial_mwh: '初始能量(MWh)',
    charge_max_mw: '充电/充能上限(MW)', discharge_max_mw: '放电/释能上限(MW)',
    retention: '时段保持率', throughput_cost_per_mwh: '吞吐成本/MWh',
    storage_layer: '储氢层级', capture_fraction: '捕集比例',
    max_tco2_per_h: '捕集上限(tCO2/h)', power_mwh_per_tco2: '捕集电耗(MWh/tCO2)',
    cost_per_tco2: '捕集成本/tCO2', purchase_limit_mw: '采购上限(MW)',
    cost_per_mwh: '成本/MWh', ev_ratio: 'EV比例', hv_ratio: '氢车比例',
    icv_ratio: '燃油车比例', alpha_ev_mwh_per_km: 'EV能耗(MWh/km)',
    alpha_hv_mwh_per_km: '氢车能耗(MWh/km)', alpha_icv_mwh_per_km: '燃油车能耗(MWh/km)',
    _ies_profile_name: '时序名称', _ies_profile_values: '导入时序(JSON数组)',
    _ies_profile_step_duration_hr: '时序步长(h)',
    _ies_buy_price_profile_values: '购电价时序(JSON数组)',
    _ies_sell_price_profile_values: '售电价时序(JSON数组)',
    _ies_carbon_profile_values: '电网碳因子时序(JSON数组)',
  };

  // ============= Mapping component types to JSON keys =============
  const jsonMapping = {
    ac_bus:           { collection: 'ac.buses',             idField: 'index' },
    generator:        { collection: 'ac.generators',        idField: 'index' },
    load:             { collection: 'ac.loads',             idField: 'index' },
    transformer_2w:   { collection: 'ac.transformers_2w',   idField: null },
    ac_branch:        { collection: 'ac.branches',          idField: 'index' },
    external_grid:    { collection: 'ac.external_grids',    idField: null },
    storage:          { collection: 'ac.storage',           idField: null },
    pv_system:        { collection: 'ac.pv_systems',        idField: null },
    renewable_gen:    { collection: 'ac.renewable_gens',     idField: null },
    static_generator: { collection: 'ac.static_generators', idField: null },
    vsc_converter:    { collection: 'vsc_converters',       idField: 'index' },
    dc_bus:           { collection: 'dc.buses',             idField: 'index' },
    dc_branch:        { collection: 'dc.branches',          idField: null },
    dc_load:          { collection: 'dc.loads',             idField: null },
    dc_storage:       { collection: 'dc.dc_storage',        idField: null },
    dc_pv_array:      { collection: 'dc.pv_arrays',          idField: null },
    switch_comp:      { collection: 'ac.switches',          idField: null },
    circuit_breaker:  { collection: 'ac.circuit_breakers',  idField: null },
    motor:            { collection: 'ac.motors',            idField: null },
    flexible_load:    { collection: 'ac.flexible_loads',    idField: 'index' },
    asymmetric_load:  { collection: 'ac.asymmetric_loads',  idField: 'index' },
    shunt:            { collection: 'ac.shunts',            idField: 'index' },
    transformer_3w:   { collection: 'ac.transformers_3w',   idField: 'index' },
    charger:          { collection: 'ac.chargers',          idField: 'index' },
    charging_station: { collection: 'ac.charging_stations', idField: 'index' },
    mobile_storage:   { collection: 'mobile_storage',       idField: 'index' },
    dcdc_converter:   { collection: 'dcdc_converters',      idField: 'index' },
    energy_router:    { collection: 'energy_routers',       idField: 'index' },
    vpp:              { collection: 'vpps',                 idField: 'index' },
    microgrid:        { collection: 'microgrids',           idField: 'index' },
  };

  return { symbols, ports, defaults, categories, fieldLabels, jsonMapping };
})();
