#!/usr/bin/env python3
"""Predeclared test of physical/residual surrogates; hydro theory §10."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import joblib
import numpy as np
from sklearn.ensemble import ExtraTreesRegressor

from analyze_hydro import features as old_features, pair_rows
from hydro_physics import feature, physics_gain, PhysicsGP
from run_hydro_oracle import audit_file
from run_price_oracle import ROOT, digest, save


def evaluate(y, p):
    y, p = np.asarray(y), np.asarray(p)
    err = np.abs(y-p)
    meaningful = np.abs(y) >= 50  # Frozen research stratum, not an engineering tolerance.
    zero = np.abs(y) < 1e-3
    missed = meaningful & ((y*p <= 0) | (np.abs(p) < 50))
    return {'mae_mwh': float(np.mean(err)), 'max_error_mwh': float(np.max(err)),
            'meaningful_mae_mwh': float(np.mean(err[meaningful])) if meaningful.any() else None,
            'zero_response_mae_mwh': float(np.mean(err[zero])) if zero.any() else None,
            'meaningful_cases': int(meaningful.sum()), 'meaningful_misses': int(missed.sum()),
            'meaningful_miss_fraction': float(missed.sum()/meaningful.sum()) if meaningful.any() else None,
            'count': len(y)}


def load_pairs(root, design, split):
    specs = design['specs']
    labels = []
    for i, spec in enumerate(specs):
        if spec['split'] == split:
            label = audit_file(root/f'week-{i:03d}', spec, design['mapping'])
            if not label['train_eligible']:
                raise ValueError(f'Frozen {split} evaluation {i} failed; cannot silently replace it')
        else:
            label = {'family': spec['family'], 'delta_mwh': spec['delta_mwh'], 'train_eligible': False}
        labels.append(label)
    return pair_rows(labels, specs), labels


def selection(rows):
    results = []
    for family in sorted({r['family'] for r in rows}):
        options = [r for r in rows if r['family'] == family]
        # A zero-action tie is preferred; no hindsight from realized gain.
        best = max(options, key=lambda r: (r['predicted_gain_mwh'], r['delta_mwh'] == 0))
        actual_best = max(r['actual_gain_mwh'] for r in options)
        results.append({'family': family, 'selected_delta_mwh': best['delta_mwh'],
                        'realized_gain_mwh': best['actual_gain_mwh'],
                        'best_candidate_gain_mwh': actual_best,
                        'regret_mwh': actual_best-best['actual_gain_mwh']})
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-intelligence/hydro-transition-v2')
    p.add_argument('--output', type=Path, default=ROOT/'output/market-intelligence/hydro-transition-analysis-v2')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output/'report.json').exists():
        raise ValueError('Frozen test report already exists; use saved results, not repeat test selection')
    design = json.loads((args.input/'design.json').read_text())
    specs = design['specs']
    boundary = json.loads((args.input/'week-000/input.json').read_text())['base']
    train, train_labels = load_pairs(args.input, design, 'train')
    models, fits = {}, []
    for n in (4, 8):
        families = [r*3+m for r in range(4) for m in range(1 if n == 4 else 2)]
        subset = [r for r in train if r['family'] in families]
        x = np.array([feature(boundary, specs[r['index']]) for r in subset])
        old = np.array([old_features(specs[r['index']]) for r in subset])
        y = np.array([r['gain_mwh'] for r in subset])
        t = time.perf_counter()
        gp = PhysicsGP().fit(x, y)
        trees = ExtraTreesRegressor(n_estimators=128, min_samples_leaf=2, random_state=917, n_jobs=1).fit(old, y)
        duration = time.perf_counter()-t
        gp.save(args.output/f'gp-{n}.npz')
        joblib.dump(trees, args.output/f'trees-{n}.joblib')
        models[n] = (gp, trees)
        manifests = [json.loads((args.input/f"week-{r['index']:03d}/manifest.json").read_text()) for r in subset]
        fits.append({'n_families': n, 'train_families': families, 'fit_sec': duration,
                     'training_labels_sha256': digest([{'index':r['index'],'gain_mwh':r['gain_mwh']} for r in subset]),
                     'training_evaluations': len(subset), 'oracle_worker_sec': sum(m['wall_sec'] for m in manifests),
                     'training_max_residual_mwh': float(np.max(np.abs(y-physics_gain(x)))),
                     'gp_sha256': hashlib.sha256((args.output/f'gp-{n}.npz').read_bytes()).hexdigest(),
                     'trees_sha256': hashlib.sha256((args.output/f'trees-{n}.joblib').read_bytes()).hexdigest()})
    # Persist both sizes and all hyperparameters before the first test-label read.
    x8 = models[8][0].x
    freeze = {'design_sha256': digest(design), 'fits': fits, 'training_before_test': True,
              'hyperparameters': {'kernel_amplitude_mwh': 50, 'lengthscale': 1, 'noise_std_mwh': 5},
              'primary_model': 'physics_gp', 'primary_training_families': 8,
              'base_sha256': digest(boundary), 'feature_min': x8.min(axis=0).tolist(), 'feature_max': x8.max(axis=0).tolist(),
              'operation_contract': {k:v for k,v in specs[0]['config'].items() if k not in ('days','reference_days')},
              'scope': 'synthetic original base, fixed 12 station weekly quotas; station1 day2/day5 only',
              'std_interpretation': 'uncalibrated conditional model uncertainty; not an engineering error guarantee'}
    save(args.output/'model-freeze.json', freeze)
    save(args.output/'boundary.json', boundary)
    test, test_labels = load_pairs(args.input, design, 'test')
    # Every test family retains its baseline as a candidate, excluded from MAE.
    x = np.array([feature(boundary, specs[r['index']]) for r in test])
    old = np.array([old_features(specs[r['index']]) for r in test])
    y = np.array([r['gain_mwh'] for r in test])
    active = np.array([r['delta_mwh'] != 0 for r in test])
    results, predictions = [], []
    for n in (4, 8):
        gp, trees = models[n]
        values = {'zero_gain': np.zeros(len(test)), 'context_trees': trees.predict(old),
                  'physics_only': physics_gain(x), 'physics_gp': gp.predict(x)}
        for name, prediction in values.items():
            prediction[~active] = 0.  # Identical baseline is an exact zero for all decision comparators.
            rows = []
            for k, row in enumerate(test):
                rows.append({'family': row['family'], 'index': row['index'], 'region': specs[row['index']]['region'],
                             'n_families': n, 'model': name, 'delta_mwh': row['delta_mwh'],
                             'actual_gain_mwh': row['gain_mwh'], 'predicted_gain_mwh': float(prediction[k]),
                             'feature': x[k].tolist(), 'train_families': fits[0 if n == 4 else 1]['train_families']})
            predictions.extend(rows)
            choice = selection(rows)
            transition = np.array([r['region'] == 3 for r in rows]) & active
            results.append({'n_families': n, 'model': name, **evaluate(y[active], prediction[active]),
                            'transition_mae_mwh': float(np.mean(np.abs(y[transition]-prediction[transition]))),
                            'selection': choice, 'mean_regret_mwh': float(np.mean([r['regret_mwh'] for r in choice]))})
    gp = models[8][0]
    # Local inference benchmark; precomputed context and full feature extraction separated.
    timings = []
    gp.predict(x[:1])
    for _ in range(1000):
        start = time.perf_counter_ns()
        gp.predict(x[:1], return_std=True)
        timings.append((time.perf_counter_ns()-start)/1e6)
    full_timings = []
    for _ in range(100):
        start = time.perf_counter_ns()
        gp.predict(feature(boundary, specs[test[0]['index']]), return_std=True)
        full_timings.append((time.perf_counter_ns()-start)/1e6)
    main_result = next(r for r in results if r['model'] == 'physics_gp' and r['n_families'] == 8)
    comparator = next(r for r in results if r['model'] == 'context_trees' and r['n_families'] == 8)
    improvement = 1-main_result['mae_mwh']/comparator['mae_mwh'] if comparator['mae_mwh'] > 1e-9 else None
    gate = improvement is not None and improvement >= .5 and main_result['mae_mwh'] <= 25 and main_result['meaningful_miss_fraction'] is not None and main_result['meaningful_miss_fraction'] <= .25
    all_labels = [train_labels[i] if s['split'] == 'train' else test_labels[i] for i, s in enumerate(specs)]
    max_audit = {}
    for label in all_labels:
        for key, value in label['hydro_audit']['max_errors'].items():
            max_audit[key] = max(max_audit.get(key, 0.), value)
    oracle = json.loads((args.input/'oracle_summary.json').read_text())
    report = {'protocol': design['protocol'], 'design_sha256': digest(design),
              'eligible_evaluations': len(train)+len(test), 'train_families': 8, 'test_families': 4,
              'fits': fits, 'results': results, 'predictions': predictions, 'pairs': train+test,
              'research_gate_passed': gate, 'mae_reduction_vs_context_trees': improvement,
              'max_audit_errors': max_audit, 'oracle_wall_sec': oracle['wall_sec'],
              'predicted_wall_sec': 2200, 'oracle_mean_worker_sec': oracle['worker_sum_sec']/36,
              'inference_ms': {'cached_median': float(np.median(timings)), 'cached_p95': float(np.quantile(timings,.95)),
                               'full_context_median': float(np.median(full_timings)), 'full_context_p95': float(np.quantile(full_timings,.95))},
              'uncalibrated_gp_std': gp.predict(x, return_std=True)[1].tolist(),
              'test_outside_training_box': int(np.sum(np.any((x<x8.min(axis=0)) | (x>x8.max(axis=0)), axis=1))),
              'production_admitted': False,
              'limitations': ['4 synthetic independent test weeks, 8 nonzero actions; no population confidence guarantee',
                              'Fixed base, fixed weekly hydro, one station/day-pair, unchanged initial states and bids',
                              'Full pricing linear diagnostic schedules; no AC/N-1 certification',
                              'Gaussian standard deviation is uncalibrated; all decisions require Oracle confirmation']}
    save(args.output/'report.json', report)
    save(args.output/'labels.json', all_labels)
    lines = ['# 水电消纳代理：物理基准与切换区域验证', '',
             '12个新外生周×3方案=36次完整周Oracle；8周训练、4周独立测试。两种训练规模在读取测试标签前冻结。', '',
             '| 训练周数 | 模型 | 测试MAE MWh | 最大误差 MWh | 切换区MAE MWh | 有意义响应漏判 | 平均后悔值 MWh |',
             '|---|---|---:|---:|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['n_families']} | {r['model']} | {r['mae_mwh']:.6f} | {r['max_error_mwh']:.6f} | {r['transition_mae_mwh']:.6f} | {r['meaningful_misses']}/{r['meaningful_cases']} | {r['mean_regret_mwh']:.6f} |")
    lines += ['', f"事前指定physics_gp研究门槛通过：{gate}。相对同批旧上下文树MAE降低比例：{improvement}。工程生产准入：否。",
              f"Oracle墙钟{oracle['wall_sec']:.3f} s，预测2200 s；单周均值{oracle['worker_sum_sec']/36:.3f} worker-s。",
              f"缓存上下文预测中位数{np.median(timings):.6f} ms、P95 {np.quantile(timings,.95):.6f} ms；包含完整输入特征计算中位数{np.median(full_timings):.6f} ms。本地单请求微基准，不是HTTP并发服务SLA。", '',
              '## 测试周配对与安全诊断', '',
              '| 家族 | δ MWh | 真增益 MWh | 缺额 MWh | 越限积分 MWh |', '|---|---:|---:|---:|---:|']
    for r in test:
        lines.append(f"| {r['family']} | {r['delta_mwh']} | {r['gain_mwh']:.6f} | {r['deficit_mwh']:.6f} | {r['overload_mwh']:.6f} |")
    lines += ['', '## 适用边界', '']+[f'- {x}' for x in report['limitations']]
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k: report[k] for k in ('research_gate_passed','mae_reduction_vs_context_trees','oracle_wall_sec','inference_ms')}))


if __name__ == '__main__':
    main()
