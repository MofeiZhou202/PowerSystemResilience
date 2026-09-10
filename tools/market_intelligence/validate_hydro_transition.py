#!/usr/bin/env python3
"""Independent raw-volume, held-out metric and saved model checks for transition-v2."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from build_market_label_dataset import read_json
from run_price_oracle import ROOT, digest, save
from validate_hydro import close


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-intelligence/hydro-transition-v2')
    p.add_argument('--report', type=Path, default=ROOT/'output/market-intelligence/hydro-transition-analysis-v2')
    args = p.parse_args()
    design = json.loads((args.input/'design.json').read_text())
    report = json.loads((args.report/'report.json').read_text())
    frozen = json.loads((args.report/'model-freeze.json').read_text())
    if digest(design) != report['design_sha256'] or digest(design) != frozen['design_sha256']:
        raise ValueError('Frozen design identity mismatch')
    for fit in frozen['fits']:
        for kind, suffix in (('gp', 'npz'), ('trees', 'joblib')):
            path = args.report/f"{kind}-{fit['n_families']}.{suffix}"
            if hashlib.sha256(path.read_bytes()).hexdigest() != fit[kind+'_sha256']:
                raise ValueError('Frozen model artifact modified')
    max_water = max_energy = max_renewable = max_metric = 0.
    available_price_days = 0
    curtailment = {}
    for i, spec in enumerate(design['specs']):
        job = read_json(args.input/f'week-{i:03d}/job.json.gz')
        base, scenario = job['base'], job['scenarios'][0]
        if scenario['config'] != spec['config'] or scenario['completed_days'] != 7 or scenario['status'] != 'completed':
            raise ValueError('Incomplete week or mismatched requested contract')
        days = scenario['days']
        water = {r['id']: [next(x for x in day['resources']['reservoirs'] if x['id'] == r['id']) for day in days] for r in base['reservoirs']}
        releases = {rid: [q for day in records for q in day['release_m3_s']] for rid, records in water.items()}
        for r in base['reservoirs']:
            rid, parent, lag = r['id'], r['upstream'], r['lag_slots']
            energy = sum(sum(g['power_mw'])*.25 for day in days for g in day['resources']['generators'] if g['id'] in r['generators'])
            max_energy = max(max_energy, close(energy, spec['weekly'][str(rid)], 1e-3, 'actual weekly hydro'))
            own_inflow = sum(sum(r['inflow_m3_s'][:96])*day['inflow_scale']*900 for day in spec['external_days'][:7])
            arrival = 0.
            if parent >= 0:
                parent_row = next(x for x in base['reservoirs'] if x['id'] == parent)
                upstream = parent_row['release_history_m3_s'][-lag:]+releases[parent][:-lag] if lag else releases[parent]
                if len(upstream) != 672:
                    raise ValueError('Incorrect lag history length')
                arrival = sum(upstream)*900
            final = water[rid][-1]['level_m'][-1]
            change = (final-r['initial_level_m'])*r['area_m2']
            max_water = max(max_water, close(change, own_inflow+arrival-sum(releases[rid])*900, 1., 'telescoped inventory'))
            spill = sum(q for day in water[rid] for q in day['spill_m3_s'])*900
            close(sum(releases[rid])*900, energy*r['water_m3_mwh']+spill, 1., 'generation water units')
            close(final, spec['terminal'][str(rid)]['level_m'], 1/r['area_m2'], 'terminal level')
        av = used = cur = 0.
        for day in days:
            if not day['diagnostic_prices_valid'] or not day['stages']['lmp']['price_consistency']['passed']:
                raise ValueError('Invalid main LMP price chain')
            available_price_days += 1
            for g in day['resources']['generators']:
                if g['kind'] in ('wind', 'solar', 'renewable'):
                    av += sum(g['renewable_available_mw'])*.25
                    used += sum(g['power_mw'])*.25
                    cur += sum(g['curtailment_mw'])*.25
        max_renewable = max(max_renewable, close(av-used, cur, 1e-6, 'renewable accounting'))
        curtailment[i] = cur
    predictions = report['predictions']
    for result in report['results']:
        rows = [r for r in predictions if r['model'] == result['model'] and r['n_families'] == result['n_families']]
        if len(rows) != 12 or len({r['family'] for r in rows}) != 4:
            raise ValueError('Incorrect test cohort size')
        for row in rows:
            spec = design['specs'][row['index']]
            if spec['split'] != 'test' or row['family'] in row['train_families']:
                raise ValueError('Train/test contamination')
            base_index = next(i for i,s in enumerate(design['specs']) if s['family'] == row['family'] and s['delta_mwh'] == 0)
            close(row['actual_gain_mwh'], curtailment[base_index]-curtailment[row['index']], 1e-8, 'raw paired gain')
        active = [r for r in rows if r['delta_mwh'] != 0]
        error = [abs(r['actual_gain_mwh']-r['predicted_gain_mwh']) for r in active]
        max_metric = max(max_metric, close(sum(error)/8, result['mae_mwh'], 1e-8, 'MAE'),
                         close(max(error), result['max_error_mwh'], 1e-8, 'max error'))
        meaningful = [r for r in active if abs(r['actual_gain_mwh']) >= 50]
        misses = sum(r['actual_gain_mwh']*r['predicted_gain_mwh'] <= 0 or abs(r['predicted_gain_mwh']) < 50 for r in meaningful)
        if (len(meaningful), misses) != (result['meaningful_cases'], result['meaningful_misses']):
            raise ValueError('Meaningful response gate mismatch')
        for chosen in result['selection']:
            options = [r for r in rows if r['family'] == chosen['family']]
            best = max(options, key=lambda r:(r['predicted_gain_mwh'], r['delta_mwh'] == 0))
            if best['delta_mwh'] != chosen['selected_delta_mwh']:
                raise ValueError('Selection used information other than predictions')
            close(max(r['actual_gain_mwh'] for r in options)-best['actual_gain_mwh'], chosen['regret_mwh'], 1e-8, 'regret')
    result = {'passed': True, 'raw_weeks': 36, 'valid_main_price_days': available_price_days,
              'prediction_rows': len(predictions), 'max_water_residual_m3': max_water,
              'max_hydro_energy_residual_mwh': max_energy, 'max_renewable_residual_mwh': max_renewable,
              'max_metric_reconstruction_error': max_metric, 'model_hashes_verified': True,
              'frozen_test_families': sorted({s['family'] for s in design['specs'] if s['split'] == 'test'})}
    save(args.report/'independent-validation.json', result)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
