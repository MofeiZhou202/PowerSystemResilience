#!/usr/bin/env python3
"""Week-grouped pilot; see intelligent_simulation.md §3 for frozen validation protocol."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import joblib
import numpy as np
import sklearn
from sklearn.base import clone
from sklearn.dummy import DummyRegressor
from sklearn.ensemble import ExtraTreesRegressor
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.model_selection import KFold, cross_val_predict
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from build_market_label_dataset import extract, read_json, hash_json, FACTORS


def candidates():
    # Ridge (Hoerl-Kennard, 1970) and Extra-Trees (Geurts et al., 2006).
    # Hyperparameters are fixed before viewing holdout labels; see protocol §3.
    models = {'mean': DummyRegressor(strategy='mean')}
    for alpha in (1., 10., 100.):
        models[f'ridge_{alpha:g}'] = make_pipeline(StandardScaler(), Ridge(alpha=alpha))
    for leaf in (1, 2):
        models[f'extra_trees_leaf{leaf}'] = ExtraTreesRegressor(n_estimators=256, min_samples_leaf=leaf,
                                                             random_state=917, n_jobs=1)
    return models


def feature_matrix(rows, names):
    if any(set(row['features']) != set(names) for row in rows):
        raise ValueError('Input feature schema mismatch')
    x = np.array([[row['features'][name] for name in names] for row in rows], dtype=float)
    if not np.isfinite(x).all():
        raise ValueError('Nonfinite features')
    return x


def get_target(row, name):
    category, key = name.split('.')
    return row[category][key]


def train(args):
    paths = sorted(args.dataset.glob('week-*/job.json.gz'))
    if len(paths) != 24:
        raise ValueError('Frozen pilot protocol requires exactly 24 attempted weeks')
    calibration = extract(args.calibration)[0]
    if not calibration['train_eligible']:
        raise ValueError('Calibration week failed quality gates')
    # Independent, original-input full-price week; never fitted on a test week.
    threshold = calibration['price']['p95']
    rows = [extract(path, threshold)[0] for path in paths]
    if any(not row['train_eligible'] for row in rows):
        raise ValueError('Unqualified week retained: audit before training; do not drop silently')
    if len({r['sample_id'] for r in rows}) != len(rows):
        raise ValueError('Duplicate physical weeks')
    if len({r['case_hash'] for r in rows}) != 1 or rows[0]['case_hash'] != calibration['case_hash']:
        raise ValueError('Pilot is limited to one physical base/initial state')
    if calibration['sample_id'] in {r['sample_id'] for r in rows}:
        raise ValueError('Calibration/week overlap')
    # Split is predeclared by independent seed, independent of all outcome labels.
    if [r['seed'] for r in rows] != list(range(20260910, 20260934)):
        raise ValueError('Unexpected sampling seeds')
    names = list(rows[0]['features'])
    x = feature_matrix(rows, names)
    train_ids, test_ids = np.arange(18), np.arange(18, 24)
    cv = KFold(n_splits=3, shuffle=True, random_state=917)
    targets = ['price.mean', 'price.p95', 'price.p99', 'price.cvar99',
               'price.spike_fraction', 'price.spike_duration_hr',
               'metrics.load_loss_mwh', 'metrics.line_overload_integral_mwh',
               'metrics.renewable_utilization', 'metrics.renewable_curtailment_mwh']
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'labels.jsonl').write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in rows))
    report = {'scope': 'IEEE118 fixed boundary and initial state; synthetic +/-10% daily factors; rolling linear diagnostic',
              'threshold': {'value': threshold, 'source': calibration['sample_id'], 'definition': 'independent calibration week node-time P95'},
              'unique_weeks': len(rows), 'train_weeks': 18, 'test_weeks': 6,
              'train_seeds': [rows[i]['seed'] for i in train_ids], 'test_seeds': [rows[i]['seed'] for i in test_ids],
              'train_sample_ids': [rows[i]['sample_id'] for i in train_ids], 'test_sample_ids': [rows[i]['sample_id'] for i in test_ids],
              'features': names, 'targets': {}, 'prediction_scope': 'summary metrics, not nodal dispatch or security certification',
              'versions': {'numpy': np.__version__, 'sklearn': sklearn.__version__},
              'labels_sha256': hashlib.sha256((args.output/'labels.jsonl').read_bytes()).hexdigest()}
    bundle = {'schema_version': 1, 'case_hash': rows[0]['case_hash'], 'features': names,
              'price_spike_threshold': threshold, 'models': {}, 'admission': {},
              'train_sample_ids': report['train_sample_ids'], 'support': {f: [.9, 1.1] for f in FACTORS}}
    bundle['support']['line_limit_scale'] = [1., 1.]
    predictions = []
    start = time.perf_counter()
    for target in targets:
        y = np.array([get_target(row, target) for row in rows], dtype=float)
        if not np.isfinite(y).all():
            report['targets'][target] = {'status': 'missing_labels'}
            continue
        # Physical zero tolerances from protocol §3, not machine-roundoff variance.
        tol = 1e-6 if not target.endswith('utilization') else 1e-9
        if np.ptp(y[train_ids]) <= tol:
            report['targets'][target] = {'status': 'no_training_variation', 'train_min': float(min(y[train_ids])),
                'train_max': float(max(y[train_ids])), 'test_min': float(min(y[test_ids])), 'test_max': float(max(y[test_ids]))}
            continue
        scores = {}
        for name, model in candidates().items():
            yp = cross_val_predict(model, x[train_ids], y[train_ids], cv=cv, n_jobs=1)
            scores[name] = float(mean_absolute_error(y[train_ids], yp))
        chosen = min(scores, key=scores.get)
        fitted = clone(candidates()[chosen]).fit(x[train_ids], y[train_ids])
        pred = fitted.predict(x[test_ids])
        baseline = np.repeat(y[train_ids].mean(), len(test_ids))
        mae = float(mean_absolute_error(y[test_ids], pred))
        baseline_mae = float(mean_absolute_error(y[test_ids], baseline))
        gain = 1 - mae / baseline_mae if baseline_mae > tol else None
        # Negative/greater-than-one rate predictions are reported, never clipped to hide errors.
        bounds_ok = not ('fraction' in target or 'utilization' in target) or bool(np.all((pred >= 0) & (pred <= 1)))
        if target.endswith('duration_hr'):
            bounds_ok = bool(np.all((pred >= 0) & (pred <= 168)))
        admitted = gain is not None and gain >= .10 and bounds_ok
        report['targets'][target] = {'selected': chosen, 'cv_mae': scores,
            'test_mae': mae, 'test_rmse': float(np.sqrt(mean_squared_error(y[test_ids], pred))),
            'test_r2': float(r2_score(y[test_ids], pred)) if np.ptp(y[test_ids]) > tol else None,
            'baseline_mae': baseline_mae, 'mae_reduction_fraction': gain,
            'predeclared_10_percent_gate': admitted, 'prediction_bounds_ok': bounds_ok}
        bundle['models'][target] = fitted
        bundle['admission'][target] = admitted
        for i, predicted, base in zip(test_ids, pred, baseline):
            predictions.append({'sample_id': rows[i]['sample_id'], 'seed': rows[i]['seed'],
                                'target': target, 'oracle': float(y[i]), 'prediction': float(predicted), 'baseline': float(base)})
    report['training_sec'] = time.perf_counter() - start
    if not bundle['models']:
        raise ValueError('No variable target to train')
    joblib.dump(bundle, args.output/'model.joblib')
    loaded = joblib.load(args.output/'model.joblib')
    for key in bundle['models']:
        np.testing.assert_array_equal(bundle['models'][key].predict(x), loaded['models'][key].predict(x))
    for model in loaded['models'].values():
        model.predict(x)
    start = time.perf_counter()
    for _ in range(100):
        for model in loaded['models'].values():
            model.predict(x)
    report['warm_batch_prediction_sec_per_week'] = (time.perf_counter()-start)/(100*len(x))
    report['validation'] = {'reload_predictions_exact': True, 'test_used_for_model_selection': False,
                            'ac_n1_certified': False, 'probability_or_tail_guarantee': False}
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'test_predictions.json').write_text(json.dumps(predictions, indent=2, allow_nan=False)+'\n')
    lines = ['# 第一版周尺度市场代理模型', '', report['scope'], '',
             f'独立周 24 个：18 训练、6 留出测试。尖峰阈值 {threshold:.6f} currency/MWh，取独立原输入校准周 P95。', '',
             '| 指标 | 模型 | 测试 MAE | 均值基线 MAE | MAE 改善 | ≥10%门槛 |', '|---|---|---:|---:|---:|---|']
    for name, result in report['targets'].items():
        if 'selected' in result:
            gain = result['mae_reduction_fraction']
            gain_str = f'{gain:.1%}' if gain is not None else '不适用'
            lines.append(f"| {name} | {result['selected']} | {result['test_mae']:.6g} | {result['baseline_mae']:.6g} | {gain_str} | {'通过' if result['predeclared_10_percent_gate'] else '未通过'} |")
        else:
            lines.append(f"| {name} | {result['status']} | — | — | — | 不准入 |")
    lines += ['', '所有指标保留真实测试结果；未通过的模型仅供研究，不能交给边界优化器作为可信目标。',
              '固定拓扑、周初状态与边界；每日共同倍率为合成扰动。未学习边界动作、极端失供或故障风险。',
              '价格为线性诊断模型价格，非真实市场价格校准；AC/N-1未认证。',
              '价格尖峰比例是节点—时段频率，不是周事件概率。6周测试不足以验证罕见事件或CVaR99概率精度。',
              f"训练 {report['training_sec']:.3f} s；预热批量推理 {report['warm_batch_prediction_sec_per_week']:.6f} s/周（全部已拟合目标，不含文件读取）。"]
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'output': str(args.output), 'threshold': threshold, 'targets': report['targets']}))


def predict(args):
    bundle = joblib.load(args.model)
    payload = json.loads(args.input.read_text())
    if payload['case_hash'] != bundle['case_hash']:
        raise ValueError('Different system/boundary/initial state: Oracle required')
    x = feature_matrix([payload], bundle['features'])
    for name, value in zip(bundle['features'], x[0]):
        lo, hi = bundle['support'][name.split('.')[1]]
        if not lo <= value <= hi:
            raise ValueError('Outside sampled support: Oracle required')
    out = {'sample_id': payload.get('sample_id'), 'label_source': 'surrogate_prediction',
           'oracle_verification_required': True, 'price_spike_threshold': bundle['price_spike_threshold'],
           'predictions': {name: {'value': float(model.predict(x)[0]),
                                 'pilot_test_gate_passed': bundle['admission'][name]}
                           for name, model in bundle['models'].items()}}
    print(json.dumps(out, allow_nan=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)
    tr = sub.add_parser('train')
    tr.add_argument('--dataset', type=Path, required=True)
    tr.add_argument('--calibration', type=Path, required=True)
    tr.add_argument('--output', type=Path, required=True)
    pr = sub.add_parser('predict')
    pr.add_argument('--model', type=Path, required=True)
    pr.add_argument('--input', type=Path, required=True)
    opts = parser.parse_args()
    (train if opts.command == 'train' else predict)(opts)
