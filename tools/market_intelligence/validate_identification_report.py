#!/usr/bin/env python3
"""Independent reconstruction of frozen development metrics and information bounds."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
from statistics import mean, median


def validate(folder):
    report = json.loads((folder/'report.json').read_text())
    raw = (folder/'all_labels.jsonl').read_bytes()
    if hashlib.sha256(raw).hexdigest() != report['labels_sha256']:
        raise ValueError('Label hash mismatch')
    rows = {r['index']: r for r in map(json.loads, raw.decode().splitlines())}
    grouped = defaultdict(list)
    predictions = json.loads((folder/'development_predictions.json').read_text())
    for p in predictions:
        row = rows[p['index']]
        cat, key = p['target'].split('.')
        if row['repeat_of'] is not None or p['oracle'] != row[cat][key] or p['family'] != row['family']:
            raise ValueError('Prediction label mismatch or repeat leakage')
        if not math.isfinite(p['prediction']):raise ValueError('Nonfinite prediction')
        grouped[(p['fold'], p['n_families'], p['target'], p['model'])].append(p)
    if len(predictions)!=11520 or len(grouped)!=720 or len(report['curves']['folds'])!=720:
        raise ValueError('Unexpected number of frozen prediction/metric groups')
    max_difference = 0.
    for record in report['curves']['folds']:
        key = (record['fold'], record['n_families'], record['target'], record['model'])
        pp = grouped[key]
        if set(record['train_families']) & set(record['valid_families']):raise ValueError('Family leakage')
        expected = {i for i, r in rows.items() if r['repeat_of'] is None and r['family'] in record['valid_families']}
        if len(pp) != 16 or {p['index'] for p in pp} != expected:raise ValueError('Incomplete development fold')
        mae = mean(abs(p['prediction']-p['oracle']) for p in pp)
        rmse = math.sqrt(mean((p['prediction']-p['oracle'])**2 for p in pp))
        for name, value in (('mae',mae),('rmse',rmse)):
            diff = abs(value-record[name]);max_difference = max(max_difference, diff)
            if diff > 1e-8:raise AssertionError('Stored metric differs from predictions')
        cat, target = record['target'].split('.')
        train = [r[cat][target] for r in rows.values() if r['repeat_of'] is None and r['family'] in record['train_families']]
        if record['model'] in ('mean', 'median'):
            value = (mean if record['model']=='mean' else median)(train)
            if any(abs(p['prediction']-value)>1e-8 for p in pp):raise AssertionError('Training-only baseline mismatch')
        by = {p['index']:p for p in pp}
        errors = []
        for family in record['valid_families']:
            for shape in (0,1):
                a,b = by[family*4+shape*2],by[family*4+shape*2+1]
                errors.append(abs((b['prediction']-a['prediction'])-(b['oracle']-a['oracle'])))
        if abs(mean(errors)-record['delta_mae'])>1e-8:raise AssertionError('Paired delta mismatch')
    bounds = 0
    for r in report['curves']['aggregate']:
        folds = [f for f in report['curves']['folds'] if f['n_families']==r['n_families']
                 and f['model']==r['model'] and f['target']==r['target']]
        if len(folds)!=3:raise ValueError('Incomplete aggregation')
        for key in ('mae','delta_mae','worker_sec','zero_delta_baseline_mae'):
            if abs(mean(f[key] for f in folds)-r[key])>1e-8:
                raise AssertionError('Aggregate differs from three development folds')
        if r['model'] not in ('mean','median','trees_low5','ridge_low5'):continue
        floor = report['information']['summaries'][r['target']]['mean_mae_floor']
        if r['mae'] < floor-1e-8:raise AssertionError('Low-information predictor violates empirical lower bound')
        bounds += 1
    result = {'passed': True, 'prediction_rows': len(predictions), 'fold_metric_groups': len(grouped),
              'max_metric_recompute_difference': max_difference, 'compression_bounds_checked': bounds,
              'absolute_tolerance':1e-8, 'trained_or_selected_models': False}
    (folder/'report-validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    validate(p.parse_args().folder)
