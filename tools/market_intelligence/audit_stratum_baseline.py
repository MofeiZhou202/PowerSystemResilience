#!/usr/bin/env python3
"""Posthoc training-stratum baseline; never changes fitted models or frozen scores."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import mean


def audit(folder):
    frozen = ('model.joblib', 'all_labels.jsonl', 'report.json', 'test_predictions.json')
    hashes = {name: hashlib.sha256((folder / name).read_bytes()).hexdigest() for name in frozen}
    rows = [json.loads(line) for line in (folder / 'all_labels.jsonl').read_text().splitlines()]
    report = json.loads((folder / 'report.json').read_text())
    predictions = json.loads((folder / 'test_predictions.json').read_text())
    eligible = [r for r in rows if r['train_eligible']]
    train = [r for r in eligible if r['split'] == 'train']
    test = {r['sample_id']: r for r in eligible if r['split'] == 'test'}
    train_ids = {r['sample_id'] for r in train}
    if len(train_ids) != len(train) or len(test) != report['test_weeks']:
        raise ValueError('Duplicated or missing samples')
    if train_ids & test.keys() or train_ids != set(report['train_ids']) or set(test) != set(report['test_ids']):
        raise ValueError('Frozen split mismatch')
    result = {'status': 'posthoc diagnostic only; not used for selection or changing predeclared gate',
              'targets': {}}
    for target, score in report['targets'].items():
        if 'mae' not in score:
            continue
        category, key = target.split('.')
        # Group mean and heldout MAE: intelligent_simulation.md section 7.
        means = {group: mean(r[category][key] for r in train if r['regime'] == group)
                 for group in {r['regime'] for r in train}}
        selected = [p for p in predictions if p['target'] == target]
        if len(selected) != len(test) or {p['sample_id'] for p in selected} != set(test):
            raise ValueError('Missing or duplicated heldout predictions')
        errors, baseline_errors = [], []
        for p in selected:
            row = test[p['sample_id']]
            actual = row[category][key]
            if p['oracle'] != actual or p['regime'] != row['regime']:
                raise ValueError('Prediction and label mismatch')
            values = (actual, p['prediction'], means[row['regime']])
            if not all(math.isfinite(v) for v in values):
                raise ValueError('Nonfinite audit value')
            errors.append(abs(actual - p['prediction']))
            baseline_errors.append(abs(actual - means[row['regime']]))
        model_mae, baseline_mae = mean(errors), mean(baseline_errors)
        if not math.isclose(model_mae, score['mae'], rel_tol=0, abs_tol=1e-8):
            raise AssertionError('Frozen MAE mismatch')
        result['targets'][target] = {'regime_training_mean_mae': baseline_mae,
                                     'model_mae': model_mae,
                                     'model_gain': 1 - model_mae / baseline_mae if baseline_mae > 0 else None}
    path = folder / 'stratum-baseline-audit.json'
    if path.exists():
        previous = json.loads(path.read_text())
        if set(previous['targets']) != set(result['targets']):
            raise AssertionError('Previous audit targets differ')
        for target, values in result['targets'].items():
            for key, value in values.items():
                old = previous['targets'][target][key]
                if value is None or old is None:
                    if value != old:
                        raise AssertionError('Previous audit availability differs')
                elif not math.isclose(value, old, rel_tol=0, abs_tol=1e-8):
                    raise AssertionError('Previous audit result differs')
    result['frozen_sha256'] = hashes
    result['frozen_artifacts_unchanged'] = all(
        hashlib.sha256((folder / name).read_bytes()).hexdigest() == value for name, value in hashes.items())
    if not result['frozen_artifacts_unchanged']:
        raise AssertionError('Frozen artifact changed during audit')
    path.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'targets': len(result['targets']), 'model_worse_than_group_mean': sum(
        r['model_gain'] is not None and r['model_gain'] < 0 for r in result['targets'].values()),
        'frozen_artifacts_unchanged': True}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    audit(parser.parse_args().folder)
