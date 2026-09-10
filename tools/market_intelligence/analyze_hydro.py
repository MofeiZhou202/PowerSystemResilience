#!/usr/bin/env python3
"""Paired hydro gains and family-held-out learning curves; design §8.2."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import time

import numpy as np
from sklearn.ensemble import ExtraTreesRegressor

from hydro_allocation import VOLUME_TOL, FLOW_TOL, ENERGY_TOL
from run_price_oracle import ROOT, digest, save
from run_hydro_oracle import audit_file

CONTEXT = ('load_scale', 'wind_scale', 'solar_scale', 'inflow_scale', 'line_limit_scale')


def ridge_predict(x, y, test):
    """Same alpha=10 ridge objective via augmented least squares; design §8.2."""
    mean, scale = x.mean(axis=0), x.std(axis=0)
    # Constant features carry no standardized information (same convention as v3).
    scale[scale < 1e-12] = 1.
    a, b = (x-mean)/scale, (test-mean)/scale
    center = y.mean()
    coef = np.linalg.lstsq(np.vstack([a, np.sqrt(10.)*np.eye(a.shape[1])]),
                           np.r_[y-center, np.zeros(a.shape[1])], rcond=None)[0]
    fitted = np.sum(a*coef[None, :], axis=1)
    gradient = np.sum(a*(fitted-(y-center))[:, None], axis=0) + 10*coef
    if np.max(np.abs(gradient)) > 1e-8*max(1., np.max(np.abs(y))):
        raise ValueError('Ridge optimality residual failed')
    prediction = np.sum(b*coef[None, :], axis=1)+center
    if not np.isfinite(prediction).all():
        raise ValueError('Nonfinite ridge prediction')
    return prediction


def features(spec):
    z = np.array([d[f] for d in spec['external_days'][:7] for f in CONTEXT])
    delta = spec['delta_mwh']/300.
    # Frozen feature map, §8.2: action, ordered context, context-action interactions.
    return np.r_[delta, z, delta*z]


def pair_rows(labels, specs):
    bases = {r['family']: r for r in labels if r['delta_mwh'] == 0 and r['train_eligible']}
    pairs = []
    for i, row in enumerate(labels):
        family = row['family']
        if not row['train_eligible'] or family not in bases:
            continue
        base = bases[family]
        spec = specs[i]
        bspec = next(s for s in specs if s['family'] == family and s['delta_mwh'] == 0)
        if spec['external_days'] != bspec['external_days'] or spec['terminal'] != bspec['terminal'] or spec['weekly'] != bspec['weekly']:
            raise ValueError('Noncomparable paired experimental inputs')
        av = row['metrics']['renewable_available_mwh']
        if abs(av-base['metrics']['renewable_available_mwh']) > 1e-6:
            raise ValueError('Renewable availability changed within pair')
        ha, hb = row['hydro_audit'], base['hydro_audit']
        if max(abs(ha['station_weekly_mwh'][k]-hb['station_weekly_mwh'][k]) for k in ha['station_weekly_mwh']) > ENERGY_TOL:
            raise ValueError('Paired realized weekly energy differs')
        water_a = {r['id']: r for r in ha['terminal_state']}
        water_b = {r['id']: r for r in hb['terminal_state']}
        if row['hydro_base_hash'] != base['hydro_base_hash'] or row['renewable_available_trace_hash'] != base['renewable_available_trace_hash']:
            raise ValueError('Paired base or renewable time-space availability differs')
        volume_diff = max(abs(water_a[rid]['initial_level_m']-water_b[rid]['initial_level_m'])*row['reservoir_area_m2'][str(rid)] for rid in water_a)
        # Each individual target gate is already tested; compare actual paired tails as well.
        flow_diff = max(abs(a-b) for rid in water_a
                        for a, b in zip(water_a[rid]['release_history_m3_s'][-spec['terminal'][str(rid)]['tail_slots']:],
                                        water_b[rid]['release_history_m3_s'][-spec['terminal'][str(rid)]['tail_slots']:]))
        spill_diff = max(abs(ha['spill_volume_m3'][k]-hb['spill_volume_m3'][k]) for k in ha['spill_volume_m3'])
        if flow_diff > FLOW_TOL or spill_diff > VOLUME_TOL or volume_diff > VOLUME_TOL:
            raise ValueError('Paired in-transit or spilled water differs')
        gain = base['metrics']['renewable_curtailment_mwh'] - row['metrics']['renewable_curtailment_mwh']
        pairs.append({'index': i, 'family': family, 'delta_mwh': row['delta_mwh'], 'gain_mwh': gain,
                      'gain_percentage_points': 100*gain/av if av > 0 else None,
                      'baseline_curtailment_mwh': base['metrics']['renewable_curtailment_mwh'],
                      'renewable_available_mwh': av,
                      'deficit_change_mwh': row['metrics']['load_loss_mwh']-base['metrics']['load_loss_mwh'],
                      'overload_change_mwh': row['metrics']['line_overload_integral_mwh']-base['metrics']['line_overload_integral_mwh'],
                      'deficit_mwh': row['metrics']['load_loss_mwh'],
                      'overload_mwh': row['metrics']['line_overload_integral_mwh'],
                      'max_spill_difference_m3': spill_diff, 'max_tail_difference_m3_s': flow_diff})
    return pairs


def metrics(y, pred):
    err = np.abs(y-pred)
    nonzero = np.abs(y) > ENERGY_TOL
    return {'mae_mwh': float(np.mean(err)), 'max_abs_error_mwh': float(np.max(err)),
            'direction_wrong': int(np.sum(((y[nonzero]*pred[nonzero]) < 0) & (np.abs(pred[nonzero]) > ENERGY_TOL))),
            'direction_undecided': int(np.sum(np.abs(pred[nonzero]) <= ENERGY_TOL)),
            'direction_cases': int(np.sum(nonzero)), 'n_pairs': len(y)}


def learning_curves(pairs, specs, manifests):
    grouped = defaultdict(list)
    for row in pairs:
        grouped[row['family']].append(row)
    if set(grouped) != set(range(6)) or any(len(rows) != 5 for rows in grouped.values()):
        return {'status': 'insufficient_complete_families', 'complete_families':
                [f for f, rows in grouped.items() if len(rows) == 5], 'folds': [], 'aggregate': [], 'predictions': []}
    order = np.random.default_rng(917).permutation(6)
    folds, predictions = [], []
    for fold in range(3):
        validation = order[2*fold:2*fold+2].tolist()
        candidates = [int(f) for f in order if f not in validation]
        test = [r for r in pairs if r['family'] in validation and r['delta_mwh'] != 0]
        xv = np.array([features(specs[r['index']]) for r in test])
        yv = np.array([r['gain_mwh'] for r in test])
        for n in (2, 3, 4):
            selected = candidates[:n]
            train = [r for r in pairs if r['family'] in selected]
            x = np.array([features(specs[r['index']]) for r in train])
            y = np.array([r['gain_mwh'] for r in train])
            cost = sum(m['wall_sec'] for m in manifests if m['family'] in selected)
            models = {'zero_gain': None, 'ridge': None,
                      'trees': ExtraTreesRegressor(n_estimators=128, min_samples_leaf=2, random_state=917, n_jobs=1)}
            for name, model in models.items():
                if name == 'zero_gain':
                    pred = np.zeros(len(test))
                elif name == 'ridge':
                    pred = ridge_predict(x, y, xv)
                else:
                    pred = model.fit(x, y).predict(xv)
                rows = []
                for r, value in zip(test, pred):
                    rows.append({'fold': fold, 'n_families': n, 'model': name, 'index': r['index'],
                                 'family': r['family'], 'train_families': selected, 'validation_families': validation,
                                 'actual_gain_mwh': r['gain_mwh'], 'predicted_gain_mwh': float(value)})
                predictions.extend(rows)
                folds.append({'fold': fold, 'n_families': n, 'n_oracle_evaluations': n*5, 'model': name,
                              'train_families': selected, 'validation_families': validation,
                              'worker_sec': cost, **metrics(yv, pred)})
    aggregate = []
    for n in (2, 3, 4):
        for model in ('zero_gain', 'ridge', 'trees'):
            selected = [r for r in predictions if r['n_families'] == n and r['model'] == model]
            actual, pred = np.array([r['actual_gain_mwh'] for r in selected]), np.array([r['predicted_gain_mwh'] for r in selected])
            worker = np.mean([r['worker_sec'] for r in folds if r['n_families'] == n and r['model'] == model])
            aggregate.append({'n_families': n, 'n_oracle_evaluations': n*5, 'model': model,
                              'worker_sec': float(worker), 'ideal_two_worker_sec': float(worker/2), **metrics(actual, pred)})
    return {'status': 'completed_development_only', 'folds': folds, 'aggregate': aggregate, 'predictions': predictions}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-intelligence/hydro-pilot-v1')
    p.add_argument('--output', type=Path, default=ROOT/'output/market-intelligence/hydro-analysis-v1')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    design = json.loads((args.input/'design.json').read_text())
    labels, manifests = [], []
    for i, spec in enumerate(design['specs']):
        folder = args.input/f'week-{i:03d}'
        if not (folder/'manifest.json').exists():
            raise ValueError(f'Unexecuted design index {i}; retain frozen cohort')
        manifest = json.loads((folder/'manifest.json').read_text())
        manifests.append(manifest)
        label = audit_file(folder, spec, design['mapping']) if (folder/'job.json.gz').exists() else {
            'family': spec['family'], 'delta_mwh': spec['delta_mwh'], 'train_eligible': False,
            'quality': {'issues': [manifest.get('error', manifest['status'])]}}
        labels.append(label)
    start = time.monotonic()
    pairs = pair_rows(labels, design['specs'])
    curves = learning_curves(pairs, design['specs'], manifests)
    train_sec = time.monotonic()-start
    mean = float(np.mean([m['wall_sec'] for m in manifests]))
    p90 = float(np.quantile([m['wall_sec'] for m in manifests], .9))
    invocations = [json.loads(path.read_text()) for path in sorted(args.input.glob('invocation-*.json'))]
    measured_wall = sum(r['invocation_wall_sec'] for r in invocations)
    informative = [r for r in pairs if r['delta_mwh'] != 0 and abs(r['gain_mwh']) > ENERGY_TOL]
    capacity = [{'budget_hours': h, 'mean_cost_evaluations': int(h*3600*2/mean),
                 'p90_cost_evaluations': int(h*3600*2/p90)} for h in (1, 2, 3)]
    tolerance_table = []
    for epsilon in (1., 5., 10., 25., 50., 100.):
        for name in ('ridge', 'trees'):
            rows = [r for r in curves['aggregate'] if r['model'] == name]
            passed_mae = [r['n_families'] for r in rows if r['mae_mwh'] <= epsilon]
            passed_max = [r['n_families'] for r in rows if r['max_abs_error_mwh'] <= epsilon]
            tolerance_table.append({'epsilon_mwh': epsilon, 'model': name,
                                    'observed_mae_passing_families': passed_mae,
                                    'observed_max_error_passing_families': passed_max,
                                    'formal_sample_size': None, 'certified': False})
    max_errors = {}
    for r in labels:
        for key, value in r.get('hydro_audit', {}).get('max_errors', {}).items():
            max_errors[key] = max(max_errors.get(key, 0.), value)
    report = {'protocol': design['protocol'], 'design_sha256': digest(design),
              'independent_families': 6, 'planned_evaluations': 30,
              'eligible_evaluations': sum(r['train_eligible'] for r in labels),
              'failures': [{'index': i, 'issues': r['quality']['issues']} for i, r in enumerate(labels) if not r['train_eligible']],
              'max_errors': max_errors, 'pairs': pairs, 'curves': curves,
              'worker_sum_sec': sum(m['wall_sec'] for m in manifests), 'mean_worker_sec': mean,
              'recorded_invocation_wall_sec': measured_wall,
              'predicted_wall_sec': design['predicted_wall_sec'],
              'wall_prediction_fractional_error': measured_wall/design['predicted_wall_sec']-1,
              'p90_worker_sec': p90, 'training_sec': train_sec, 'capacity_examples': capacity,
              'informative_nonzero_pairs': len(informative),
              'informative_families': sorted({r['family'] for r in informative}),
              'tolerance_examples': tolerance_table, 'formal_sample_size': None,
              'final_blind_test': False, 'deployed': False,
              'limitations': ['Synthetic IEEE118; station1 day2/day5 action only; 6 exogenous families',
                              'Conditional 7-day rolling linear diagnostic schedules; not weekly joint optimum or AC/N-1 certification',
                              'Fixed terminal reservoir/lag water; no additional thermal or storage terminal equivalence contract',
                              'Grouped development curves are not per-case error bounds; engineering tolerances and exact budget unset']}
    save(args.output/'report.json', report)
    save(args.output/'predictions.json', curves.pop('predictions'))
    save(args.output/'labels.json', labels)
    lines = ['# 水电分配专项试验：消纳增益与误差—成本', '',
             f"30次周评价，6个独立外生家族；有效{report['eligible_evaluations']}次。不是30个独立外生周。",
             '固定12个合成站每站21000 MWh/周，仅站1第2→第5日电量转移−300、−150、0、150、300 MWh。',
             '所有动作使用相同新能源可用量、初态、参考轨迹和共同期末水位/下泄尾条件。', '',
             '## 数值审计', '', '数值门槛与工程容差分开。以下为30次评价中的最大误差：', '']
    lines.extend(f'- {k}: {v:.9g}' for k, v in max_errors.items())
    lines += ['', '## 配对收益（MWh；正数表示减少弃风弃光）', '',
              '| 外生家族 | −300 | −150 | +150 | +300 | 基准弃风弃光 |', '|---|---:|---:|---:|---:|---:|']
    for family in range(6):
        subset = {r['delta_mwh']: r for r in pairs if r['family'] == family}
        values = [f"{subset[d]['gain_mwh']:.6f}" if d in subset else '未知/未通过' for d in (-300., -150., 150., 300.)]
        base = f"{subset[0.]['baseline_curtailment_mwh']:.6f}" if 0. in subset else '未知'
        lines.append(f"| {family} | " + ' | '.join(values) + f' | {base} |')
    lines += ['', '## 按新外生周留出的开发学习曲线', '',
              f"绝对增益超过数值审计尺度0.001 MWh的非零动作有{len(informative)}个，分布于{len({r['family'] for r in informative})}个外生家族；不能用大量零响应样本证明模型精度。",
              '每折留出2个外生周，仅4个非零动作计入验证；三折共24个配对。均值误差不等于逐例误差界。', '',
              '| 训练家族/周评价 | 模型 | Oracle worker-min | MAE MWh | 最大误差 MWh | 反向/未判方向 |',
              '|---|---|---:|---:|---:|---:|']
    for r in curves['aggregate']:
        lines.append(f"| {r['n_families']}/{r['n_oracle_evaluations']} | {r['model']} | {r['worker_sec']/60:.3f} | {r['mae_mwh']:.6f} | {r['max_abs_error_mwh']:.6f} | {r['direction_wrong']}/{r['direction_undecided']} |")
    lines += ['', '## 计算预算与正式补样规模', '',
              f"各次执行调用墙钟合计{measured_wall:.3f} s（不含调用之间的人工/分析时间），预测{design['predicted_wall_sec']} s；相对差{(measured_wall/design['predicted_wall_sec']-1)*100:.1f}%。",
              f"实测单周worker均值{mean:.3f} s、P90 {p90:.3f} s；训练{train_sec:.3f} s（预测<120 s）。预算换算忽略排队及额外开销，不是吞吐保证。", '',
              '| 双进程预算示例 | 按均值评价次数 | 按P90评价次数 |', '|---|---:|---:|']
    lines.extend(f"| {r['budget_hours']} h | {r['mean_cost_evaluations']} | {r['p90_cost_evaluations']} |" for r in capacity)
    lines += ['', 'report.json中的tolerance_examples列出1/5/10/25/50/100 MWh工程误差示例对应的已观测达标规模；不拟合小样本收敛率，也不把经验达标写成正式验收。',
              '正式总样本量尚不可确定：需要工程收益误差、最小有意义增益、安全阈值、置信要求和明确预算。下一批应由本批失败/收益/残差证据确定，而非承诺固定周数。', '',
              '## 适用边界', '']
    lines.extend('- '+x for x in report['limitations'])
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k: report[k] for k in ('eligible_evaluations', 'max_errors', 'mean_worker_sec', 'training_sec')}))


if __name__ == '__main__':
    main()
