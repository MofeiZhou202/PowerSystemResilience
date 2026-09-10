#!/usr/bin/env python3
"""Independent whole-week telescoping water/energy and prediction audit, design §8."""
import argparse
import json
from pathlib import Path

from build_market_label_dataset import read_json
from run_price_oracle import ROOT, digest, save


def close(a, b, tolerance, name):
    if abs(a-b) > tolerance:
        raise ValueError(f'{name}: {a} vs {b}, tolerance {tolerance}')
    return abs(a-b)


def validate(root, report_dir):
    design = json.loads((root/'design.json').read_text())
    report = json.loads((report_dir/'report.json').read_text())
    if digest(design) != report['design_sha256']:
        raise ValueError('Design hash mismatch')
    max_water = max_energy = max_renewable = max_metric = 0.
    count = 0
    for i, spec in enumerate(design['specs']):
        folder = root/f'week-{i:03d}'
        if not (folder/'label.json').exists():
            continue
        label = json.loads((folder/'label.json').read_text())
        if not label['train_eligible']:
            continue
        job = read_json(folder/'job.json.gz')
        base, days = job['base'], job['scenarios'][0]['days']
        release, spill, energy = {}, {}, {}
        for r in base['reservoirs']:
            rid = r['id']
            outputs = [next(x for x in d['resources']['reservoirs'] if x['id'] == rid) for d in days]
            release[rid] = [q for x in outputs for q in x['release_m3_s']]
            spill[rid] = sum(q for x in outputs for q in x['spill_m3_s'])*900
            energy[rid] = sum(sum(g['power_mw'])*.25 for d in days for g in d['resources']['generators'] if g['id'] in r['generators'])
            max_energy = max(max_energy, close(energy[rid], spec['weekly'][str(rid)], 1e-3, 'weekly hydro energy'))
        for r in base['reservoirs']:
            rid, parent, lag = r['id'], r['upstream'], r['lag_slots']
            local = sum(sum(r['inflow_m3_s'][:96])*d['inflow_scale']*900 for d in spec['external_days'][:7])
            arrival = 0.
            if parent >= 0:
                parent_base = next(x for x in base['reservoirs'] if x['id'] == parent)
                upstream = (parent_base['release_history_m3_s'][-lag:] + release[parent][:-lag]) if lag else release[parent]
                if len(upstream) != 672:
                    raise ValueError('Wrong executed lag vector length')
                arrival = sum(upstream)*900
            final = next(x for x in days[-1]['state_end']['reservoirs'] if x['id'] == rid)['initial_level_m']
            storage_change = (final-r['initial_level_m'])*r['area_m2']
            # Independent weekly telescoping, not the per-time-step recurrence in the producer.
            max_water = max(max_water, close(storage_change, local+arrival-sum(release[rid])*900, 1., 'weekly water inventory'))
            close(sum(release[rid])*900, energy[rid]*r['water_m3_mwh']+spill[rid], 1., 'water-energy units')
        av = used = cur = 0.
        for d in days:
            for g in d['resources']['generators']:
                if g['kind'] in ('wind', 'solar', 'renewable'):
                    av += sum(g['renewable_available_mw'])*.25
                    used += sum(g['power_mw'])*.25
                    cur += sum(g['curtailment_mw'])*.25
        max_renewable = max(max_renewable, close(av-used, cur, 1e-6, 'renewable energy conservation'),
                            close(cur, label['metrics']['renewable_curtailment_mwh'], 1e-6, 'curtailment label'))
        count += 1
    predictions = json.loads((report_dir/'predictions.json').read_text())
    for aggregate in report['curves']['aggregate']:
        rows = [r for r in predictions if r['model'] == aggregate['model'] and r['n_families'] == aggregate['n_families']]
        if len(rows) != 24 or len({r['index'] for r in rows}) != 24:
            raise ValueError('Repeated or missing held-out predictions')
        for row in rows:
            if row['family'] in row['train_families'] or row['family'] not in row['validation_families']:
                raise ValueError('Family leakage')
            if len(row['train_families']) != aggregate['n_families'] or len(set(row['train_families'])) != aggregate['n_families']:
                raise ValueError('Incorrect training family count')
            if row['model'] == 'zero_gain' and row['predicted_gain_mwh'] != 0:
                raise ValueError('Incorrect no-gain baseline')
            pair = next(p for p in report['pairs'] if p['index'] == row['index'])
            close(pair['gain_mwh'], row['actual_gain_mwh'], 1e-10, 'prediction target')
        errors = [abs(r['actual_gain_mwh']-r['predicted_gain_mwh']) for r in rows]
        max_metric = max(max_metric, close(sum(errors)/len(errors), aggregate['mae_mwh'], 1e-8, 'MAE'),
                         close(max(errors), aggregate['max_abs_error_mwh'], 1e-8, 'maximum error'))
        relevant = [r for r in rows if abs(r['actual_gain_mwh']) > 1e-3]
        wrong = sum(r['actual_gain_mwh']*r['predicted_gain_mwh'] < 0 and abs(r['predicted_gain_mwh']) > 1e-3 for r in relevant)
        undecided = sum(abs(r['predicted_gain_mwh']) <= 1e-3 for r in relevant)
        if (wrong, undecided, len(relevant)) != (aggregate['direction_wrong'], aggregate['direction_undecided'], aggregate['direction_cases']):
            raise ValueError('Incorrect direction audit')
    result = {'passed': True, 'eligible_raw_weeks_checked': count, 'predictions_checked': len(predictions),
              'whole_week_water_max_residual_m3': max_water, 'energy_max_residual_mwh': max_energy,
              'renewable_max_residual_mwh': max_renewable, 'metric_max_difference': max_metric,
              'design_sha256': digest(design), 'excluded_weeks': report['failures']}
    save(report_dir/'independent-validation.json', result)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-intelligence/hydro-pilot-v1')
    p.add_argument('--report', type=Path, default=ROOT/'output/market-intelligence/hydro-analysis-v1')
    args = p.parse_args()
    print(json.dumps(validate(args.input, args.report)))


if __name__ == '__main__':
    main()
