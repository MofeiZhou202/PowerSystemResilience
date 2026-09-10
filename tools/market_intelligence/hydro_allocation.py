"""Hydro action and independent SI audits; hydro_renewable_allocation.md §§2,8."""
import copy
import math

from build_market_label_dataset import DT, SLOTS, vector
from run_price_oracle import digest

ENERGY_TOL = 1e-3  # Frozen numerical gates in design §8; not engineering tolerances.
VOLUME_TOL = 1.0
FLOW_TOL = 1e-6


def by_id(rows):
    result = {r['id']: r for r in rows}
    if len(result) != len(rows):
        raise ValueError('Duplicate stable component ID')
    return result


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Missing/nonfinite numeric value')
    return float(value)


def validate_mapping(boundary, mapping):
    """Only explicitly authored, one-station-per-reservoir pilot mappings admitted."""
    generators, reservoirs = by_id(boundary['generators']), by_id(boundary['reservoirs'])
    by_id(mapping)
    seen, covered = set(), set()
    for station in mapping:
        rid = station['reservoir_id']
        if rid not in reservoirs or rid in covered:
            raise ValueError('Unknown or duplicate station reservoir')
        covered.add(rid)
        members = station['generator_ids']
        expected = reservoirs[rid].get('generators', [reservoirs[rid].get('generator')])
        if not members or len(set(members)) != len(members) or set(members) != set(expected):
            raise ValueError('Pilot station must contain exactly its reservoir members')
        for gid in members:
            if gid in seen or gid not in generators or generators[gid]['kind'] != 'hydro':
                raise ValueError('Duplicate, unknown or non-hydro station generator')
            seen.add(gid)
    if covered != set(reservoirs) or seen != {g['id'] for g in generators.values() if g['kind'] == 'hydro'}:
        raise ValueError('Incomplete hydro mapping')


def synthetic_mapping(boundary):
    if not boundary['reservoirs'] or any('synthetic' not in r.get('source', '') for r in boundary['reservoirs']):
        raise ValueError('Automatic synthetic mapping requires explicitly synthetic reservoir sources')
    mapping = [{'id': r['id'], 'name': f"Synthetic research station R{r['id']}",
                'reservoir_id': r['id'], 'generator_ids': list(r['generators']),
                'provenance': 'authored pilot convention: shared-reservoir group; not real station identity'}
               for r in boundary['reservoirs']]
    validate_mapping(boundary, mapping)
    return mapping


def terminal_contract(boundary, mapping, days, quotas):
    """Analytic volume budgets and terminal lag tails, frozen design §8.1.

    This generates common terminal targets; it does not prove rolling feasibility.
    """
    validate_mapping(boundary, mapping)
    reservoirs = by_id(boundary['reservoirs'])
    if len(days) != 8:
        raise ValueError('Eight external days required')
    if len(boundary['periods']) != 98 or any(p['duration_hr'] != DT for p in boundary['periods'][:96]):
        raise ValueError('Quarter-hour Southern grid required')
    total = {s['reservoir_id']: sum(quotas[str(s['id'])]) for s in mapping}
    rates = {rid: total[rid] / (7 * 24) * finite(r['water_m3_mwh']) / 3600
             for rid, r in reservoirs.items()}
    result = {}
    while len(result) < len(reservoirs):
        progressed = False
        for rid, r in reservoirs.items():
            if str(rid) in result or (r['upstream'] >= 0 and str(r['upstream']) not in result):
                continue
            lag = r['lag_slots']
            if not isinstance(lag, int) or not 0 <= lag <= 96:
                raise ValueError('Pilot only supports lag 0..96')
            area = finite(r['area_m2'])
            if area <= 0:
                raise ValueError('Positive reservoir area required')
            inflow = sum(sum(r['inflow_m3_s'][:96]) * finite(d['inflow_scale']) * 900 for d in days[:7])
            arrival = 0.
            if r['upstream'] >= 0:
                parent = reservoirs[r['upstream']]
                history = parent['release_history_m3_s']
                if len(history) < lag:
                    raise ValueError('Insufficient upstream prehistory')
                arrival = result[str(r['upstream'])]['release_volume_m3']
                if lag:
                    arrival += 900 * (sum(history[-lag:]) - lag * rates[r['upstream']])
            used = total[rid] * finite(r['water_m3_mwh'])
            free_level = r['initial_level_m'] + (inflow + arrival - used) / area
            level = min(r['max_level_m'][95], free_level)
            if level < r['min_level_m'][95]:
                raise ValueError('Terminal budget below minimum level')
            spill = max(0., (free_level - level) * area)
            tail = max([1] + [x['lag_slots'] for x in reservoirs.values() if x['upstream'] == rid])
            if tail > 96:
                raise ValueError('Pilot terminal tail longer than one day')
            result[str(rid)] = {'level_m': level, 'tail_slots': tail, 'release_m3_s': rates[rid],
                                'spill_volume_m3': spill, 'release_volume_m3': used + spill}
            progressed = True
        if not progressed:
            raise ValueError('Reservoir cycle or missing upstream')
    return result


def compile_action(boundary, mapping, external_days, quotas, weekly, terminal):
    validate_mapping(boundary, mapping)
    days = copy.deepcopy(external_days)
    if len(days) != 8 or any(d.get('boundary_overrides') for d in days):
        raise ValueError('Requires eight external days without hidden overrides')
    ids = {str(s['id']) for s in mapping}
    if set(quotas) != ids or set(weekly) != ids or set(terminal) != {str(r['id']) for r in boundary['reservoirs']}:
        raise ValueError('Incomplete action or terminal identity')
    for day in days[:7]:
        day['boundary_overrides'] = []
    reservoirs = by_id(boundary['reservoirs'])
    for station in mapping:
        sid, rid = str(station['id']), station['reservoir_id']
        values = quotas[sid]
        if len(values) != 7 or any(finite(v) < 0 for v in values):
            raise ValueError('Seven nonnegative daily targets required')
        if abs(sum(values) - finite(weekly[sid])) > 1e-9:
            raise ValueError('Weekly target conservation violated')
        for d, value in enumerate(values):
            if not reservoirs[rid]['min_mwh'] <= value <= reservoirs[rid]['max_mwh']:
                raise ValueError('Daily quota outside authored energy bounds')
            for field in ('min_mwh', 'max_mwh'):
                days[d]['boundary_overrides'].append({'table': 'reservoirs', 'id': rid, 'field': field,
                    'first_slot': 0, 'last_slot': 97, 'value': value, 'reason': 'hydro-pilot-v1 station daily realized energy equality'})
        target = terminal[str(rid)]
        for field in ('min_level_m', 'max_level_m'):
            days[6]['boundary_overrides'].append({'table': 'reservoirs', 'id': rid, 'field': field,
                'first_slot': 95, 'last_slot': 95, 'value': target['level_m'],
                'reason': 'hydro-pilot-v1 common terminal stored water'})
        for field in ('release_min_m3_s', 'release_max_m3_s'):
            days[6]['boundary_overrides'].append({'table': 'reservoirs', 'id': rid, 'field': field,
                'first_slot': 96-target['tail_slots'], 'last_slot': 95, 'value': target['release_m3_s'],
                'reason': 'hydro-pilot-v1 common in-transit water and next-day release state'})
    return days


def sample_identity(base, config, mapping, quotas, terminal, binary_hash):
    # Include full reference trajectories and resource contract, unlike generic schema3.
    return digest({'schema': 'hydro-v1', 'base': base, 'config': config, 'mapping': mapping,
                   'quotas': quotas, 'terminal': terminal, 'binary_sha256': binary_hash})


def audit_hydrology(job, spec, mapping):
    """Independently reconstruct power integrals, releases and SI water recurrence."""
    base, scenario = job['base'], job['scenarios'][0]
    validate_mapping(base, mapping)
    days = scenario['days']
    if scenario['status'] != 'completed' or scenario['completed_days'] != 7 or len(days) != 7:
        raise ValueError('Incomplete hydro week')
    if scenario['config']['days'] != spec['config']['days'] or scenario['config']['reference_days'] != spec['config']['reference_days']:
        raise ValueError('Action/reference contract differs from requested inputs')
    reservoirs = by_id(base['reservoirs'])
    actual = {str(s['id']): [] for s in mapping}
    spill_total = {str(r): 0. for r in reservoirs}
    max_balance = max_release = max_daily = max_end = max_tail = 0.
    for d, day in enumerate(days):
        if not day['valid'] or day['day'] != d:
            raise ValueError('Invalid executed day')
        if d and day['state_start'] != days[d-1]['state_end']:
            raise ValueError('Broken chronological carry')
        if d == 0:
            for table, records in day['state_start'].items():
                originals = by_id(base[table])
                for state in records:
                    if any(originals[state['id']][k] != v for k, v in state.items() if k != 'id'):
                        raise ValueError('Initial state differs from authored base')
        gens, water = by_id(day['resources']['generators']), by_id(day['resources']['reservoirs'])
        if set(water) != set(reservoirs):
            raise ValueError('Missing reservoir output')
        start, end = by_id(day['state_start']['reservoirs']), by_id(day['state_end']['reservoirs'])
        if set(start) != set(reservoirs) or set(end) != set(reservoirs):
            raise ValueError('Missing reservoir carry state')
        power = {gid: vector(g, 'power_mw') for gid, g in gens.items()}
        release = {rid: vector(w, 'release_m3_s') for rid, w in water.items()}
        for station in mapping:
            sid = str(station['id'])
            energy = sum(sum(power[gid]) for gid in station['generator_ids']) * DT
            actual[sid].append(energy)
            max_daily = max(max_daily, abs(energy - spec['quotas'][sid][d]))
        for rid, r in reservoirs.items():
            level, spill = vector(water[rid], 'level_m'), vector(water[rid], 'spill_m3_s')
            members = r.get('generators', [r.get('generator')])
            previous = finite(start[rid]['initial_level_m'])
            for t in range(96):
                theoretical_release = spill[t] + sum(power[gid][t] for gid in members) * r['water_m3_mwh'] / 3600
                max_release = max(max_release, abs(theoretical_release-release[rid][t]))
                incoming = r['inflow_m3_s'][t] * scenario['config']['days'][d]['inflow_scale']
                if r['upstream'] >= 0:
                    u, lag = r['upstream'], r['lag_slots']
                    incoming += release[u][t-lag] if t >= lag else start[u]['release_history_m3_s'][t-lag]
                residual = (level[t]-previous)*r['area_m2'] - (incoming-release[rid][t])*900
                max_balance = max(max_balance, abs(residual))
                previous = level[t]
            if (abs(finite(end[rid]['initial_level_m'])-level[-1])*r['area_m2'] > VOLUME_TOL
                    or abs(finite(end[rid]['initial_release_m3_s'])-release[rid][-1]) > FLOW_TOL
                    or end[rid]['release_history_m3_s'] != release[rid]):
                raise ValueError('Carry does not match realized reservoir output')
            spill_total[str(rid)] += sum(spill)*900
            if d == 6:
                target = spec['terminal'][str(rid)]
                max_end = max(max_end, abs(level[-1]-target['level_m'])*r['area_m2'])
                max_tail = max(max_tail, max(abs(q-target['release_m3_s']) for q in release[rid][-target['tail_slots']:]))
    max_week = max(abs(sum(actual[sid])-spec['weekly'][sid]) for sid in actual)
    max_spill = max(abs(spill_total[rid]-spec['terminal'][rid]['spill_volume_m3']) for rid in spill_total)
    checks = {'daily_energy_mwh': max_daily, 'weekly_energy_mwh': max_week,
              'water_balance_m3': max_balance, 'release_definition_m3_s': max_release,
              'terminal_volume_m3': max_end, 'terminal_release_m3_s': max_tail,
              'weekly_spill_budget_m3': max_spill}
    passed = (max_daily <= ENERGY_TOL and max_week <= ENERGY_TOL and max_balance <= VOLUME_TOL
              and max_release <= FLOW_TOL and max_end <= VOLUME_TOL and max_tail <= FLOW_TOL
              and max_spill <= VOLUME_TOL)
    return {'passed': passed, 'max_errors': checks, 'station_daily_mwh': actual,
            'station_weekly_mwh': {k: sum(v) for k, v in actual.items()},
            'spill_volume_m3': spill_total,
            'terminal_state': days[-1]['state_end']['reservoirs']}
