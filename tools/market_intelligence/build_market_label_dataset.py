#!/usr/bin/env python3
"""Audited weekly labels; equations and scope: docs/modules/market/intelligent_simulation.md."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path

FACTORS = ('load_scale', 'wind_scale', 'solar_scale', 'inflow_scale',
           'generator_bid_scale', 'load_bid_scale', 'line_limit_scale')
SLOTS = 96
DAYS = 7
DT = .25
TOL = 1e-6  # market_forecast.cpp physical event threshold, MW.


def read_json(path):
    path = Path(path)
    with (gzip.open(path, 'rt') if path.suffix == '.gz' else path.open()) as f:
        value = json.load(f)
    return value.get('job', value)


def hash_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def vector(row, key):
    values = row.get(key)
    if not isinstance(values, list) or len(values) != SLOTS:
        raise ValueError(f'{key}: requires 96 executed points')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError(f'{key}: missing/nonfinite value')
    return values


def quantile(values, q):
    # Generalized inverse empirical CDF, matching market_forecast.cpp::quantile.
    return sorted(values)[max(0, math.ceil(q * len(values)) - 1)]


def price_summary(values):
    if not values:
        raise ValueError('No valid prices')
    eta = quantile(values, .99)
    return {'mean': sum(values) / len(values), 'max': max(values),
            'p95': quantile(values, .95), 'p99': eta,
            # Rockafellar-Uryasev (2000): fractional tail mass, including atoms.
            'cvar99': eta + sum(max(0, x - eta) for x in values) / (.01 * len(values)),
            'count': len(values)}


def identity(job, scenario):
    if 'base' not in job:
        raise ValueError('Full export with base required; old path-based identities are inadmissible')
    base = json.loads(json.dumps(job['base']))
    execution = base.get('execution', {})
    for key in ('solver', 'threads', 'time_limit_sec', 'mip_gap', 'native_root_cuts',
                'assembly_mode', 'gurobi_method', 'mip_start', 'row_presolve'):
        execution.pop(key, None)
    cfg = scenario['config']
    # Scheduling inputs and initial states identify a physical week, not a file path.
    case_hash = hash_json(base)
    week = {'case': case_hash, 'days': cfg['days'], 'start_date': cfg['start_date'],
            'penalty_per_mwh': cfg['penalty_per_mwh']}
    return case_hash, hash_json(week)


def extract(path, spike_threshold=None):
    job = read_json(path)
    periods = job.get('base', {}).get('periods', [])
    if len(periods) != 98 or any(p['duration_hr'] != DT for p in periods[:SLOTS]):
        raise ValueError('Only Southern 98-point / 96 executed quarter-hour grid is supported')
    rows = []
    for scenario in job['scenarios']:
        case_hash, sample_id = identity(job, scenario)
        config = scenario['config']
        if len(config['days']) != 8:
            raise ValueError('Eight input days including terminal forecast required')
        features = {f'd{d}.{f}': float(config['days'][d][f]) for d in range(8) for f in FACTORS}
        if not all(math.isfinite(v) for v in features.values()):
            raise ValueError('Nonfinite input feature')
        days = scenario['days']
        issues = []
        complete = (scenario['status'] == 'completed' and scenario['completed_days'] == DAYS and len(days) == DAYS)
        if not complete:
            issues.append('incomplete_week')
        row = {'schema_version': 3, 'sample_id': sample_id, 'case_hash': case_hash,
               'source_job': str(path), 'source_sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest(),
               'scenario_id': scenario['id'], 'seed': job['config']['seed'], 'features': features,
               'status': scenario['status'], 'model_scope': scenario['model_scope'],
               'limitations': job.get('limitations', []) + scenario.get('limitations', []),
               'label_source': 'local_linear_diagnostic_oracle',
               'quality': {'issues': issues, 'ac_certified': False, 'n1_certified': False},
               'price': None, 'prices_by_node': None, 'metrics': {},
               'stage_quality': [], 'train_eligible': False}
        prices = {}
        deficits = overloads = renewable_av = renewable_used = curtailed = 0.0
        max_loss = max_over = loss_duration = overload_duration = 0.0
        objective_sum = runtime = 0.0
        node_loss, line_over = {}, {}
        expected_nodes = {n['id'] for n in job['base']['buses']}
        expected_lines = {n['id'] for n in job['base']['branches']}
        expected_gens = {n['id'] for n in job['base']['generators']}
        try:
            for i, day in enumerate(days):
                if day['day'] != i or not day.get('valid'):
                    raise ValueError('invalid_day')
                if i and day['state_start'] != days[i-1]['state_end']:
                    raise ValueError('carry_mismatch')
                for stage_name in ('scuc', 'sced', 'lmp'):
                    stage = day['stages'][stage_name]
                    row['stage_quality'].append({'day': i, 'stage': stage_name, **stage})
                    for field in ('max_residual', 'mip_gap', 'requested_mip_gap', 'objective'):
                        value = stage.get(field)
                        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                            raise ValueError('invalid_stage_numeric_metadata')
                    if stage['max_residual'] < 0 or stage['mip_gap'] < 0 or stage['requested_mip_gap'] < 0:
                        raise ValueError('negative_stage_quality_metadata')
                    if stage['max_residual'] is None or stage['max_residual'] > TOL:
                        raise ValueError('residual_not_accepted')
                    if stage['mip_gap'] is None or stage['mip_gap'] > stage['requested_mip_gap'] + 1e-12:
                        raise ValueError('gap_not_accepted')
                objective_sum += day['stages']['scuc']['objective']
                runtime += day['runtime_sec']
                lmp = day['stages']['lmp']
                if not day.get('diagnostic_prices_valid') or not (lmp.get('price_consistency') or {}).get('passed'):
                    raise ValueError('invalid_or_unverified_price')
                if not lmp.get('optimality_proven'):
                    raise ValueError('lmp_not_optimal')
                nodes, lines = day['nodes'], day['lines']
                for records, expected in ((nodes, expected_nodes), (lines, expected_lines),
                                           (day['resources']['generators'], expected_gens)):
                    if {r['id'] for r in records} != expected or len(records) != len(expected):
                        raise ValueError('missing_or_duplicate_component')
                day_loss = [0.] * SLOTS
                day_over = [0.] * SLOTS
                for node in nodes:
                    pv = vector(node, 'lmp_per_mwh')
                    prices.setdefault(str(node['id']), []).extend(pv)
                    dv = vector(node, 'deficit_mw')
                    node_loss[str(node['id'])] = node_loss.get(str(node['id']), 0.) + sum(dv) * DT
                    max_loss = max(max_loss, max(dv))
                    day_loss = [a+b for a, b in zip(day_loss, dv)]
                for line in lines:
                    # Directional limits permit asymmetric and zero bounds; no division by rating.
                    f, lo, hi = (vector(line, k) for k in ('power_mw', 'min_mw', 'max_mw'))
                    avail = vector(line, 'available')
                    ov = [max(0., p-u, l-p) if a else abs(p) for p, l, u, a in zip(f, lo, hi, avail)]
                    declared = vector(line, 'overload_mw')
                    if max(abs(a-b) for a, b in zip(ov, declared)) > TOL:
                        raise ValueError('line_overload_recompute_mismatch')
                    line_over[str(line['id'])] = line_over.get(str(line['id']), 0.) + sum(ov)*DT
                    max_over = max(max_over, max(ov))
                    day_over = [a+b for a, b in zip(day_over, ov)]
                if any(not isinstance(day.get(k), (int, float)) or not math.isfinite(day[k])
                       for k in ('deficit_mwh', 'overload_mwh')):
                    raise ValueError('invalid_energy_metadata')
                if abs(sum(day_loss)*DT-day['deficit_mwh']) > TOL or abs(sum(day_over)*DT-day['overload_mwh']) > TOL:
                    raise ValueError('energy_recompute_mismatch')
                deficits += sum(day_loss)*DT
                overloads += sum(day_over)*DT
                loss_duration += sum(x > TOL for x in day_loss)*DT
                overload_duration += sum(x > TOL for x in day_over)*DT
                for gen in day['resources']['generators']:
                    if gen['kind'] in ('wind', 'solar', 'renewable'):
                        av, used, cur = (vector(gen, k) for k in ('renewable_available_mw', 'power_mw', 'curtailment_mw'))
                        if any(p < -TOL or p-a > TOL or abs(max(0, a-p)-c) > TOL for a, p, c in zip(av, used, cur)):
                            raise ValueError('renewable_energy_mismatch')
                        renewable_av += sum(av)*DT
                        renewable_used += sum(used)*DT
                        curtailed += sum(cur)*DT
        except (KeyError, TypeError, ValueError) as exc:
            issues.append(str(exc))
        if not issues:
            all_prices = [p for v in prices.values() for p in v]
            row['price'] = price_summary(all_prices)
            row['prices_by_node'] = {k: price_summary(v) for k, v in prices.items()}
            row['metrics'] = {'load_loss_mwh': deficits, 'line_overload_integral_mwh': overloads,
                'max_node_loss_mw': max_loss, 'max_line_overload_mw': max_over,
                'loss_duration_hr': loss_duration, 'overload_duration_hr': overload_duration,
                'renewable_available_mwh': renewable_av, 'renewable_used_mwh': renewable_used,
                'renewable_curtailment_mwh': curtailed,
                'renewable_utilization': renewable_used / renewable_av if renewable_av > 0 else None,
                'scuc_98point_objective_sum': objective_sum, 'solver_time_sec': runtime}
            # Do not call this weekly realized cost: objectives include the two lookahead points.
            row['node_loss_mwh'] = node_loss
            row['line_overload_integral_mwh'] = line_over
            row['train_eligible'] = True
            if spike_threshold is not None:
                row['price'].update(spike_threshold=spike_threshold,
                    spike_fraction=sum(p > spike_threshold for p in all_prices)/len(all_prices),
                    spike_duration_hr=DT*sum(any(v[t] > spike_threshold for v in prices.values()) for t in range(DAYS*SLOTS)),
                    spike_event=any(p > spike_threshold for p in all_prices))
        rows.append(row)
    return rows


def main():
    p = argparse.ArgumentParser()
    p.add_argument('paths', nargs='+', type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--spike-threshold', type=float)
    args = p.parse_args()
    if args.spike_threshold is not None and not math.isfinite(args.spike_threshold):
        p.error('finite threshold required')
    files = sorted({f for path in args.paths for f in
                    (list(path.rglob('job.json')) + list(path.rglob('job.json.gz')) if path.is_dir() else [path])})
    records, duplicates, errors = {}, [], []
    for path in files:
        try:
            for row in extract(path, args.spike_threshold):
                if row['sample_id'] in records:
                    duplicates.append({'sample_id': row['sample_id'], 'source': str(path)})
                else:
                    records[row['sample_id']] = row
        except (KeyError, TypeError, ValueError, OSError) as exc:
            errors.append({'source': str(path), 'error': str(exc)})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(''.join(json.dumps(r, allow_nan=False) + '\n' for r in records.values()))
    summary = {'files': len(files), 'unique_weeks': len(records),
               'eligible_weeks': sum(r['train_eligible'] for r in records.values()),
               'duplicates': duplicates, 'errors': errors, 'schema_version': 3}
    args.output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary))
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
