#!/usr/bin/env python3
"""Frozen stress experiment v2. Contract: intelligent_simulation.md section 6."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import joblib
import numpy as np
from sklearn.base import clone
from sklearn.dummy import DummyClassifier, DummyRegressor
from sklearn.ensemble import ExtraTreesClassifier, ExtraTreesRegressor
from sklearn.linear_model import Ridge
from sklearn.metrics import (mean_absolute_error, r2_score, brier_score_loss,
                             log_loss, confusion_matrix)
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

from build_market_label_dataset import extract, FACTORS, read_json
from run_price_oracle import STRESS_RANGES

THRESHOLD = 59.61200124564096
TARGETS = ['price.mean', 'price.p95', 'price.p99', 'price.cvar99',
           'price.spike_fraction', 'price.spike_duration_hr',
           'metrics.load_loss_mwh', 'metrics.line_overload_integral_mwh',
           'metrics.renewable_utilization', 'metrics.renewable_curtailment_mwh']


def matrix(rows, mode):
    """Input-only pooling, section 6: fixed calendar execution and lookahead spaces."""
    names = {f'd{d}.{f}' for d in range(8) for f in FACTORS}
    if any(set(r['features']) != names for r in rows):
        raise ValueError('Unexpected feature schema')
    if mode not in ('raw', 'pooled'):
        raise ValueError('Unexpected feature mode')
    raw = np.asarray([[r['features'][f'd{d}.{f}'] for d in range(8) for f in FACTORS] for r in rows], dtype=float)
    if not np.isfinite(raw).all():
        raise ValueError('Nonfinite features')
    if mode == 'raw':
        return raw
    daily = raw.reshape(-1, 8, 7)
    executed = daily[:, :7, :]
    summary = np.concatenate([executed.mean(axis=1), executed.std(axis=1),
                              executed.min(axis=1), executed.max(axis=1),
                              np.abs(np.diff(executed, axis=1)).max(axis=1), daily[:, 7, :]], axis=1)
    return np.concatenate([raw, summary], axis=1)


def reg_candidates(target):
    result = {'mean': ('raw', DummyRegressor(strategy='mean'))}
    # Only price levels may use unconstrained Ridge; tree means preserve target bounds.
    if target in TARGETS[:4]:
        for alpha in (1., 10., 100.):
            result[f'ridge_{alpha:g}'] = ('pooled', make_pipeline(StandardScaler(), Ridge(alpha=alpha)))
    for mode in ('raw', 'pooled'):
        for leaf in (1, 2, 4):
            result[f'et_{mode}_{leaf}'] = (mode, ExtraTreesRegressor(
                n_estimators=256, min_samples_leaf=leaf, random_state=917, n_jobs=1))
    return result


def metric(y, pred, baseline):
    mae = float(mean_absolute_error(y, pred))
    bmae = float(mean_absolute_error(y, baseline))
    return {'n': len(y), 'mae': mae, 'baseline_mae': bmae,
            'gain': 1-mae/bmae if bmae > 1e-12 else None,
            'r2': float(r2_score(y, pred)) if len(y) > 1 and np.ptp(y) > 1e-9 else None}


def main(args):
    if (args.output/'report.json').exists():
        raise ValueError('Completed evaluation exists; preserve holdout evidence in this directory')
    args.output.mkdir(parents=True, exist_ok=True)
    rows, failures = [], []
    for i in range(32):
        folder = args.dataset/f'week-{i:03d}'
        manifest = json.loads((folder/'manifest.json').read_text())
        if manifest['seed'] != 20261000+i or manifest.get('design') != 'four-strata-v2':
            raise ValueError('Unexpected stress protocol')
        if manifest.get('status') == 'running':
            raise ValueError('Oracle run unfinished')
        try:
            r = extract(folder/'job.json.gz', THRESHOLD)[0]
            r.update(regime=manifest['regime'], split=manifest['split'])
        except (OSError, ValueError, KeyError) as exc:
            failures.append({'seed': manifest['seed'], 'split': manifest['split'], 'regime': manifest['regime'], 'error': str(exc)})
            continue
        if not r['train_eligible']:
            failures.append({'seed': manifest['seed'], 'split': manifest['split'], 'regime': manifest['regime'], 'error': r['quality']['issues']})
        rows.append(r)
    old = [json.loads(x) for x in args.old_labels.read_text().splitlines()]
    old = [r for r in old if 20260910 <= r['seed'] <= 20260927]
    if len(old) != 18:
        raise ValueError('Requires exactly the 18 old training weeks')
    for r in old:
        r.update(regime='ordinary', split='train')
    rows = old + rows
    (args.output/'all_labels.jsonl').write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in rows))
    (args.output/'failures.json').write_text(json.dumps(failures, indent=2)+'\n')
    eligible = [r for r in rows if r['train_eligible']]
    if len({r['sample_id'] for r in eligible}) != len(eligible) or len({r['case_hash'] for r in eligible}) != 1:
        raise ValueError('Duplicate week or differing base')
    train = [r for r in eligible if r['split'] == 'train']
    test = [r for r in eligible if r['split'] == 'test']
    if len(test) < 4 or any(sum(r['regime']==g for r in train) < 3 for g in STRESS_RANGES):
        raise ValueError('Insufficient eligible strata; failures preserved')
    groups = [r['regime'] for r in train]
    folds = list(StratifiedKFold(n_splits=3, shuffle=True, random_state=917).split(np.zeros(len(train)), groups))
    xtrain = {m: matrix(train, m) for m in ('raw', 'pooled')}
    xtest = {m: matrix(test, m) for m in ('raw', 'pooled')}
    report = {'train_weeks': len(train), 'test_weeks': len(test), 'attempted_new_weeks': 32,
              'failures': failures, 'threshold': THRESHOLD,
              'metrics_scope': 'conditional on eligible complete-price weeks; stress strata have no real-world weights',
              'targets': {}, 'classifiers': {}, 'test_ids': [r['sample_id'] for r in test],
              'train_ids': [r['sample_id'] for r in train]}
    bundle = {'schema_version': 2, 'case_hash': train[0]['case_hash'], 'models': {},
              'threshold': THRESHOLD, 'support_strata': STRESS_RANGES,
              'oracle_verification_required': True, 'training_ids': report['train_ids']}
    predictions = []
    start = time.perf_counter()
    for target in TARGETS:
        category, key = target.split('.')
        y = np.array([r[category][key] for r in train], dtype=float)
        yt = np.array([r[category][key] for r in test], dtype=float)
        if not np.isfinite(y).all() or not np.isfinite(yt).all():
            report['targets'][target] = {'status': 'missing_label'}
            continue
        if np.ptp(y) <= (1e-9 if key=='renewable_utilization' else 1e-6):
            report['targets'][target] = {'status': 'no_training_variation'}
            continue
        scores = {}
        choices = reg_candidates(target)
        for name, (mode, model) in choices.items():
            yp = cross_val_predict(model, xtrain[mode], y, cv=folds)
            scores[name] = float(mean_absolute_error(y, yp))
        chosen = min(scores, key=scores.get)
        mode, model = choices[chosen]
        model.fit(xtrain[mode], y)
        yp = model.predict(xtest[mode])
        baseline = np.full(len(yt), y.mean())
        result = metric(yt, yp, baseline)
        result.update(selected=chosen, cv_mae=scores, by_regime={})
        for regime in STRESS_RANGES:
            mask = np.array([r['regime']==regime for r in test])
            result['by_regime'][regime] = metric(yt[mask], yp[mask], baseline[mask]) if mask.any() else {'n':0}
        bounds = bool(np.all(yp >= 0)) if category=='metrics' or key.startswith('spike') else True
        if key in ('spike_fraction', 'renewable_utilization'):
            bounds = bounds and bool(np.all(yp <= 1+1e-12))
        if key == 'spike_duration_hr':
            bounds = bounds and bool(np.all(yp <= 168))
        result['gate'] = result['gain'] is not None and result['gain'] >= .1 and bounds and not failures
        result['bounds_ok'] = bounds
        report['targets'][target] = result
        bundle['models'][target] = {'mode': mode, 'model': model, 'pilot_gate': result['gate']}
        for r, actual, predicted in zip(test, yt, yp):
            predictions.append({'sample_id': r['sample_id'], 'seed':r['seed'], 'regime':r['regime'],
                                'target':target, 'oracle':float(actual), 'prediction':float(predicted)})
    for target in ('load_loss_mwh', 'line_overload_integral_mwh', 'renewable_curtailment_mwh'):
        y = np.array([r['metrics'][target]>1e-6 for r in train], dtype=int)
        yt = np.array([r['metrics'][target]>1e-6 for r in test], dtype=int)
        if min(np.bincount(y, minlength=2)) < 3 or any(len(np.unique(y[a])) < 2 for a, _ in folds):
            report['classifiers'][target] = {'status': 'insufficient_training_classes'}
            continue
        choices = {'prior': DummyClassifier(strategy='prior'),
                   'extra_trees': ExtraTreesClassifier(n_estimators=256, min_samples_leaf=2, random_state=917, n_jobs=1)}
        scores = {}
        for name, model in choices.items():
            prob = cross_val_predict(model, xtrain['pooled'], y, cv=folds, method='predict_proba')[:,1]
            scores[name] = float(log_loss(y, prob, labels=[0,1]))
        name = min(scores, key=scores.get)
        model = choices[name].fit(xtrain['pooled'], y)
        prob = model.predict_proba(xtest['pooled'])[:,1]
        tn, fp, fn, tp = confusion_matrix(yt, prob>=.5, labels=[0,1]).ravel().tolist()
        report['classifiers'][target] = {'selected':name,'cv_log_loss':scores,
            'brier': float(brier_score_loss(yt,prob)), 'log_loss':float(log_loss(yt,prob,labels=[0,1])),
            'tn':tn,'fp':fp,'fn':fn,'tp':tp, 'false_negative_rate':fn/(fn+tp) if fn+tp else None,
            'reliability_admitted':False, 'reason':'at least five holdout weeks per class required; pilot only'}
        bundle.setdefault('classifiers',{})[target] = model
    report['training_sec'] = time.perf_counter()-start
    joblib.dump(bundle,args.output/'model.joblib')
    loaded = joblib.load(args.output/'model.joblib')
    for key, fitted in bundle['models'].items():
        np.testing.assert_array_equal(fitted['model'].predict(xtest[fitted['mode']]),
                                      loaded['models'][key]['model'].predict(xtest[fitted['mode']]))
    start=time.perf_counter()
    for _ in range(20):
        for fitted in loaded['models'].values():
            fitted['model'].predict(xtest[fitted['mode']])
    report['prediction_sec_per_week']=(time.perf_counter()-start)/(20*len(test))
    # Compare frozen v1 only where its declared input support applies.
    old_model=joblib.load(args.old_model)
    mask=np.array([r['regime']=='ordinary' for r in test])
    report['v1_ordinary_comparison']={}
    if mask.any():
        ordinary=[r for r in test if r['regime']=='ordinary']
        x=np.array([[r['features'][k] for k in old_model['features']] for r in ordinary])
        for key,m in old_model['models'].items():
            category,k=key.split('.')
            truth=np.array([r[category][k] for r in ordinary])
            report['v1_ordinary_comparison'][key]={'v1_mae':float(mean_absolute_error(truth,m.predict(x))),
                'v2_mae':report['targets'].get(key,{}).get('by_regime',{}).get('ordinary',{}).get('mae')}
    report['validation']={'reload_exact':True, 'test_used_for_selection':False,
                          'safe_for_boundary_optimization':False}
    report['labels_sha256']=hashlib.sha256((args.output/'all_labels.jsonl').read_bytes()).hexdigest()
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.output/'test_predictions.json').write_text(json.dumps(predictions,indent=2,allow_nan=False)+'\n')
    lines=['# 第二版压力场景代理模型', '',f"训练 {len(train)} 周，留出 {len(test)} 周；新增失败/不准入 {len(failures)} 周。",'',
           '| 目标 | 留出MAE | 均值基线MAE | 改善 | 门槛 |','|---|---:|---:|---:|---|']
    for key,r in report['targets'].items():
        if 'mae' in r:
            gain=f"{r['gain']:.1%}" if r['gain'] is not None else '不可用'
            lines.append(f"|{key}|{r['mae']:.6g}|{r['baseline_mae']:.6g}|{gain}|{r['gate']}|")
        else:lines.append(f"|{key}|{r['status']}|—|—|False|")
    lines+=['','风险分类、各场景组及v1普通周对比见report.json。','固定合成场景试验，不赋予真实事件概率；仅条件场景响应，未验证边界策略或AC/N-1安全。']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'output':str(args.output),'training_sec':report['training_sec']}))


def predict(model_path, input_path):
    bundle = joblib.load(model_path)
    row = json.loads(input_path.read_text())
    if row['case_hash'] != bundle['case_hash']:
        raise ValueError('Different case/initial state: Oracle required')
    matrices = {m: matrix([row], m) for m in ('raw', 'pooled')}
    matches = []
    for regime, overrides in bundle['support_strata'].items():
        bounds = {f: ((1.,1.) if f=='line_limit_scale' else (.9,1.1)) for f in FACTORS}
        bounds.update(overrides)
        if all(bounds[f][0] <= row['features'][f'd{d}.{f}'] <= bounds[f][1]
               for d in range(8) for f in FACTORS):
            matches.append(regime)
    if not matches:
        raise ValueError('Outside sampled strata: Oracle required')
    out = {'label_source':'surrogate_prediction', 'oracle_verification_required':True,
           'support_strata':matches, 'threshold':bundle['threshold'],
           'predictions':{k:{'value':float(v['model'].predict(matrices[v['mode']])[0]),
                             'pilot_gate_passed':v['pilot_gate']}
                          for k,v in bundle['models'].items()},
           'diagnostic_event_scores':{k:float(m.predict_proba(matrices['pooled'])[0,1])
                                      for k,m in bundle.get('classifiers',{}).items()},
           'event_probabilities_calibrated':False}
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--dataset',type=Path)
    p.add_argument('--old-labels',type=Path,default=Path('output/market-intelligence/price-surrogate-v1/labels.jsonl'))
    p.add_argument('--old-model',type=Path,default=Path('output/market-intelligence/price-surrogate-v1/model.joblib'))
    p.add_argument('--output',type=Path)
    p.add_argument('--predict-input',type=Path)
    p.add_argument('--model',type=Path)
    args=p.parse_args()
    if args.predict_input:
        if not args.model:
            p.error('--model required for prediction')
        print(json.dumps(predict(args.model,args.predict_input),allow_nan=False))
    else:
        if not args.dataset or not args.output:
            p.error('--dataset and --output required for training')
        main(args)
