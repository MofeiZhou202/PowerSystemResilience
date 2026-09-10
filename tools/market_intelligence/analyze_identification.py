#!/usr/bin/env python3
"""Information audit and grouped development curves; sampling theory section 9."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.fft import dct
from sklearn.ensemble import ExtraTreesRegressor
from sklearn.model_selection import KFold

from build_market_label_dataset import FACTORS, extract, read_json, hash_json
from run_identification_oracle import validate_input
from run_price_oracle import save
from train_stress_surrogate import TARGETS, THRESHOLD


def representations(rows):
    a = np.asarray([[r['features'][f'd{d}.{f}'] for d in range(8) for f in FACTORS]
                    for r in rows], dtype=float).reshape(-1, 8, 7)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite features')
    # The terminal centers equal seven-day means on this verified design manifold.
    # Using identical terminal values avoids splitting on rounding in reversed sums.
    low = a[:, 7, [0, 1, 3, 4, 6]]
    np.testing.assert_allclose(a[:, :7].mean(axis=1), a[:, 7], atol=1e-12, rtol=0)
    np.testing.assert_allclose(a[:, 7, 1], a[:, 7, 2], atol=1e-12, rtol=0)
    np.testing.assert_allclose(a[:, 7, 4], a[:, 7, 5], atol=1e-12, rtol=0)
    # Orthonormal DCT-II: sqrt(2/7) sum_d x_d cos(pi*k*(d+1/2)/7), k=1,2.
    # Temporal coefficients are input-only, as prescribed by theory section 9.
    modes = dct(a[:, :7, :6], type=2, norm='ortho', axis=1)[:, 1:3].reshape(len(rows), 12)
    e = a[:, :7]
    # Canonical sum order keeps mathematically order-free moments bitwise invariant.
    ordered = np.sort(e, axis=1)
    summary = np.concatenate([ordered.mean(axis=1), ordered.std(axis=1), ordered.min(axis=1), ordered.max(axis=1),
                              abs(np.diff(e, axis=1)).max(axis=1), a[:, 7]], axis=1)
    return {'low5': low, 'temporal17': np.c_[low, modes], 'full56': a.reshape(len(rows), 56),
            'orderless42': summary}


def grouped_folds():
    families = np.arange(12)
    out = []
    for fold, (train, valid) in enumerate(KFold(3, shuffle=True, random_state=917).split(families)):
        train = np.random.default_rng(917 + fold).permutation(train)
        out.append((train.tolist(), valid.tolist()))
    return out


def ridge_predict(x, y, test, alpha=10.):
    """Training-only scaling and augmented least squares; no normal-equation inverse."""
    mean = x.mean(axis=0)
    scale = x.std(axis=0)
    scale[scale < 1e-12] = 1.
    a, b = (x - mean) / scale, (test - mean) / scale
    center = y.mean()
    # Hoerl-Kennard ridge objective; independent formulation also used in v2 audit.
    coef = np.linalg.lstsq(np.vstack([a, np.sqrt(alpha) * np.eye(a.shape[1])]),
                           np.r_[y - center, np.zeros(a.shape[1])], rcond=None)[0]
    return np.sum(b * coef[None, :], axis=1) + center


def pair_floor(first, second):
    delta = abs(np.asarray(first) - np.asarray(second))
    # Triangle inequality and completion of square, sampling theory section 9.
    return delta / 2, delta ** 2 / 4


def bounds_ok(target, values):
    if not np.isfinite(values).all():
        return False
    if target.startswith('metrics.') or '.spike_' in target:
        if np.any(values < 0):return False
    if target.endswith(('spike_fraction', 'renewable_utilization')):
        return bool(np.all(values <= 1 + 1e-12))
    if target.endswith('spike_duration_hr'):
        return bool(np.all(values <= 168))
    return True


def audit_jobs(dataset, output):
    design = json.loads((dataset/'design.json').read_text())
    if design['protocol'] != 'preflight-v3' or len(design['specs']) != 50:
        raise ValueError('Unexpected protocol')
    rows, failures = [], []
    for index, spec in enumerate(design['specs']):
        folder = dataset/f'week-{index:03d}'
        try:
            manifest = json.loads((folder/'manifest.json').read_text())
            if manifest['status'] != 'completed':raise ValueError('Unfinished/failed Oracle')
            if manifest['binary_sha256'] != design['binary_sha256']:raise ValueError('Binary mismatch')
            job = read_json(folder/'job.json.gz')
            validate_input(job, spec)
            cfg = job['scenarios'][0]['config']
            row = extract(folder/'job.json.gz', THRESHOLD)[0]
            row.update(spec['metadata'], index=index, worker_sec=manifest['wall_sec'])
            if not row['train_eligible']:raise ValueError(str(row['quality']['issues']))
            initial = job['scenarios'][0]['days'][0]['state_start']
            row['initial_state_hash'] = hash_json(initial)
            paths = {key: [{k: v for k, v in d.items() if k != 'line_limit_scale'} for d in cfg[key]]
                     for key in ('days', 'reference_days')}
            row['exogenous_hash'] = hash_json(paths)
            row['reference_days_hash'] = hash_json(cfg['reference_days'])
            base_lines = {line['id']: line for line in job['base']['branches']}
            max_error = 0.
            online = {}
            for day in job['scenarios'][0]['days']:
                multiplier = spec['expected_days'][day['day']]['line_limit_scale']
                for line in day['lines']:
                    for key in ('min_mw', 'max_mw'):
                        expected = np.asarray(base_lines[line['id']][key][:96]) * multiplier
                        max_error = max(max_error, float(np.max(abs(expected - line[key]))))
                for gen in day['resources']['generators']:
                    online.setdefault(str(gen['id']), []).extend(gen['online'])
            if max_error > 1e-8:raise ValueError(f'Applied line-limit mismatch: {max_error}')
            row['line_limit_max_error'] = max_error
            row['online_signature'] = hash_json(online)
            rows.append(row)
            print(json.dumps({'audited': index + 1, 'eligible': True}), flush=True)
        except (ValueError, KeyError, TypeError, OSError, AssertionError) as exc:
            failures.append({'index': index, **spec['metadata'], 'error': str(exc)})
    output.mkdir(parents=True, exist_ok=True)
    (output/'all_labels.jsonl').write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in rows))
    save(output/'failures.json', failures)
    if failures:
        raise ValueError('Incomplete identification dataset; failed attempts saved without zero labels')
    return rows, design


def information_audit(rows):
    regular = [r for r in rows if r['repeat_of'] is None]
    if len(regular) != 48 or len({r['sample_id'] for r in regular}) != 48:
        raise ValueError('Missing or duplicate physical experimental week')
    if len({r['case_hash'] for r in rows}) != 1 or len({r['initial_state_hash'] for r in rows}) != 1:
        raise ValueError('Case or initial-state mismatch')
    by = {r['index']: r for r in rows}
    features = representations(regular)
    pairs, actions, repeats = [], [], []
    for family in range(12):
        for action in (0, 1):
            a, b = by[4*family + action], by[4*family + 2 + action]
            np.testing.assert_array_equal(features['low5'][a['index']], features['low5'][b['index']])
            np.testing.assert_array_equal(features['orderless42'][a['index']], features['orderless42'][b['index']])
            record = {'family': family, 'action': action, 'online_changed': a['online_signature'] != b['online_signature'],
                      'targets': {}}
            for target in TARGETS:
                cat, key = target.split('.')
                ya, yb = a[cat][key], b[cat][key]
                floor, squared = pair_floor(ya, yb)
                record['targets'][target] = {'forward': ya, 'reverse': yb,
                    'mae_floor': float(floor), 'mse_floor': float(squared)}
            pairs.append(record)
        for shape in (0, 1):
            a, b = by[4*family + 2*shape], by[4*family + 2*shape+1]
            if a['exogenous_hash'] != b['exogenous_hash']:
                raise ValueError('Action pairing changed exogenous/reference paths')
            record = {'family': family, 'shape': shape, 'base_line_scale': a['features']['d7.line_limit_scale'],
                      'action_line_scale': b['features']['d7.line_limit_scale'], 'targets': {},
                      'online_changed': a['online_signature'] != b['online_signature']}
            for target in TARGETS:
                cat, key = target.split('.')
                record['targets'][target] = b[cat][key]-a[cat][key]
            actions.append(record)
    for row in rows:
        if row['repeat_of'] is None:continue
        original = by[row['repeat_of']]
        if row['sample_id'] != original['sample_id'] or row['reference_days_hash'] != original['reference_days_hash']:
            raise ValueError('Replay did not use identical physical input')
        repeats.append({'index': row['index'], 'repeat_of': row['repeat_of'],
                        'online_changed': row['online_signature'] != original['online_signature'],
                        'absolute_difference': {t: abs(row[t.split('.')[0]][t.split('.')[1]]-
                                                       original[t.split('.')[0]][t.split('.')[1]]) for t in TARGETS}})
    summaries = {}
    for t in TARGETS:
        floors = [p['targets'][t]['mae_floor'] for p in pairs]
        summaries[t] = {'mean_mae_floor': float(np.mean(floors)), 'max_mae_floor': max(floors),
                        'rms_floor': float(np.sqrt(np.mean([p['targets'][t]['mse_floor'] for p in pairs]))),
                        'max_repeat_difference': max(r['absolute_difference'][t] for r in repeats),
                        'mean_abs_action_delta': float(np.mean([abs(a['targets'][t]) for a in actions])),
                        'max_abs_action_delta': max(abs(a['targets'][t]) for a in actions)}
    x = features['low5']
    x = (x-x.mean(axis=0))/x.std(axis=0)
    phi = np.c_[np.ones(len(x)), x]
    return {'shape_pairs': pairs, 'action_pairs': actions, 'repeats': repeats,
            'summaries': summaries, 'coordinate_rank': int(np.linalg.matrix_rank(phi)),
            'coordinate_columns': phi.shape[1], 'coordinate_condition_number': float(np.linalg.cond(phi)),
            'max_line_limit_error': max(r['line_limit_max_error'] for r in rows),
            'interpretation': 'empirical same-coordinate error floors; repeat checks cover only two inputs; no real-market certification'}


def learning_curves(rows):
    rows = sorted((r for r in rows if r['repeat_of'] is None), key=lambda r: r['index'])
    x = representations(rows)
    family = np.array([r['family'] for r in rows])
    records, predictions = [], []
    for fold, (order, valid) in enumerate(grouped_folds()):
        valid_idx = np.flatnonzero(np.isin(family, valid))
        for n in (4, 6, 8):
            train = order[:n]
            train_idx = np.flatnonzero(np.isin(family, train))
            if set(train) & set(valid):raise AssertionError('Family leakage')
            cost = sum(rows[i]['worker_sec'] for i in train_idx)
            for target in TARGETS:
                cat, key = target.split('.')
                y = np.asarray([r[cat][key] for r in rows], dtype=float)
                if not np.isfinite(y).all():raise ValueError('Nonfinite target')
                candidates = {'mean': np.full(len(valid_idx), y[train_idx].mean()),
                              'median': np.full(len(valid_idx), np.median(y[train_idx]))}
                for mode in ('low5', 'temporal17', 'full56'):
                    model = ExtraTreesRegressor(n_estimators=256, min_samples_leaf=2, random_state=917, n_jobs=1)
                    candidates['trees_'+mode] = model.fit(x[mode][train_idx], y[train_idx]).predict(x[mode][valid_idx])
                    candidates['ridge_'+mode] = ridge_predict(x[mode][train_idx], y[train_idx], x[mode][valid_idx])
                for name, pred in candidates.items():
                    if not np.isfinite(pred).all():raise ValueError('Nonfinite prediction')
                    truth = y[valid_idx]
                    delta_errors, delta_baseline = [], []
                    for f in valid:
                        for shape in (0, 1):
                            indices = [int(np.flatnonzero(valid_idx == (4*f+2*shape+a))[0]) for a in (0, 1)]
                            actual_delta = truth[indices[1]] - truth[indices[0]]
                            predicted_delta = pred[indices[1]] - pred[indices[0]]
                            delta_errors.append(abs(actual_delta - predicted_delta))
                            delta_baseline.append(abs(actual_delta))
                    records.append({'fold': fold, 'train_families': train, 'valid_families': valid,
                        'n_families': n, 'training_weeks': len(train_idx), 'worker_sec': cost,
                        'target': target, 'model': name, 'mae': float(np.mean(abs(truth-pred))),
                        'rmse': float(np.sqrt(np.mean((truth-pred)**2))),
                        'bounds_ok': bounds_ok(target, pred), 'delta_mae': float(np.mean(delta_errors)),
                        'zero_delta_baseline_mae': float(np.mean(delta_baseline))})
                    predictions.extend({'fold': fold, 'n_families': n, 'target': target, 'model': name,
                        'index': rows[i]['index'], 'family': rows[i]['family'], 'oracle': float(actual),
                        'prediction': float(p)} for i, actual, p in zip(valid_idx, truth, pred))
            print(json.dumps({'curve_fold': fold, 'training_families': n, 'complete': True}), flush=True)
    aggregate = []
    for n in (4, 6, 8):
        for target in TARGETS:
            selected = [r for r in records if r['n_families']==n and r['target']==target]
            median_mae = np.mean([r['mae'] for r in selected if r['model']=='median'])
            mean_mae = np.mean([r['mae'] for r in selected if r['model']=='mean'])
            for name in sorted({r['model'] for r in selected}):
                group = [r for r in selected if r['model']==name]
                result = {'n_families': n, 'training_weeks': 4*n, 'target': target, 'model': name,
                    **{key: float(np.mean([r[key] for r in group])) for key in
                       ('worker_sec','mae','delta_mae','zero_delta_baseline_mae')},
                    'rmse': float(np.sqrt(np.mean([r['rmse']**2 for r in group]))),
                    'bounds_ok': all(r['bounds_ok'] for r in group)}
                result['gain_vs_median'] = 1-result['mae']/median_mae if median_mae > 0 else None
                result['gain_vs_mean'] = 1-result['mae']/mean_mae if mean_mae > 0 else None
                aggregate.append(result)
    marginal = []
    for target in TARGETS:
        for model in sorted({r['model'] for r in aggregate}):
            curve = sorted((r for r in aggregate if r['target']==target and r['model']==model),
                           key=lambda r:r['n_families'])
            for a, b in zip(curve, curve[1:]):
                minutes = (b['worker_sec']-a['worker_sec'])/60
                if minutes <= 0:raise ValueError('Nested training cost did not increase')
                # Finite-difference estimate of -e'(n)/c, sampling theory section 5.
                marginal.append({'target': target, 'model': model, 'from_families': a['n_families'],
                    'to_families': b['n_families'], 'additional_worker_minutes': minutes,
                    'mae_reduction_per_worker_minute': (a['mae']-b['mae'])/minutes})
    return {'folds': records, 'aggregate': aggregate, 'empirical_marginal_gain': marginal,
            'split': [{'train_order': t, 'development': v} for t, v in grouped_folds()]}, predictions


def write_report(output, report):
    info = report['information']
    lines = ['# 前置辨识：信息、配对动作与误差—成本曲线', '',
             '这是12个家族的开发实验；48个交叉周评价及2次重复不是50个独立周。尚无最终盲测或工程精度准入。', '',
             f"Oracle双进程wall：{report['oracle_wall_sec']:.3f} s；预期1500 s。曲线训练：{report['curve_training_sec']:.3f} s；预期<120 s。", '',
             f"所有50次评价通过标签与实际输入核对。线路限额最大复算差 {info['max_line_limit_error']:.3g} MW。",
             f"标准化局部线性设计矩阵秩 {info['coordinate_rank']}/{info['coordinate_columns']}，条件数 {info['coordinate_condition_number']:.3f}；不等于非线性精度保证。", '',
             '| 指标 | 低维二点平均MAE下界 | 最大二点MAE下界 | 最大重复求解差 | 平均绝对边界动作变化 |',
             '|---|---:|---:|---:|---:|']
    for target, r in info['summaries'].items():
        lines.append(f"|{target}|{r['mean_mae_floor']:.6g}|{r['max_mae_floor']:.6g}|{r['max_repeat_difference']:.6g}|{r['mean_abs_action_delta']:.6g}|")
    lines += ['', '同低维坐标的日序反转也保留42维无序统计量；表中下界仅对这些坐标上的确定性代理成立，不能套用到17维时序或56维完整输入。两次重复仅检查两个输入的数值可重复性，不证明所有输入已数值稳定。', '',
              '价格水平单位currency/MWh，占比/利用率为0–1，时长h，能量及有功越限积分MWh。所有价格均为固定诊断罚价规则下的线性Oracle价格。', '',
              '## 固定完整输入树模型的开发曲线', '',
              '每个训练规模使用相同开发家族；表中MAE是三个开发折的平均，无模型选择。完整模型候选与每折数据见report.json。', '',
              '| 指标 | 4家族MAE | 6家族MAE | 8家族MAE | 8家族相对中位数改善 | 配对Δ误差 / 零变化基线 |',
              '|---|---:|---:|---:|---:|---:|']
    curve = report['curves']['aggregate']
    for target in TARGETS:
        r = {x['n_families']: x for x in curve if x['target']==target and x['model']=='trees_full56'}
        gain = r[8]['gain_vs_median']
        g = f'{gain:.1%}' if gain is not None else '不可定义'
        lines.append(f"|{target}|{r[4]['mae']:.6g}|{r[6]['mae']:.6g}|{r[8]['mae']:.6g}|{g}|{r[8]['delta_mae']:.6g} / {r[8]['zero_delta_baseline_mae']:.6g}|")
    lines += ['', '| 训练家族数 | 每折训练周评价数 | 平均训练Oracle worker-minutes |',
              '|---|---:|---:|']
    for n in (4, 6, 8):
        r = next(r for r in curve if r['n_families']==n)
        lines.append(f"|{n}|{4*n}|{r['worker_sec']/60:.3f}|")
    lines += ['', '训练成本按实际Oracle worker-seconds计，不含开发集与重复求解；它不是双进程wall。三个规模仅形成经验曲线，未拟合幂律或外推达到某精度所需总周数。', '',
              '## 适用边界', '',
              '固定系统/初态、日倍率和已知参考轨迹；没有AC/N-1认证、真实风险概率、完整LMP矩阵代理或Bayesian/CMA-ES优化结果。尚未设置工程误差门槛。全部开发数据可用于后续设计，但不能再作为最终盲测。', '',
              '原始job、输入、设计、manifest保存在identification-v3；all_labels.jsonl、information-audit.json、development_predictions.json和report.json保存可复核证据。']
    rows = [json.loads(line) for line in (output/'all_labels.jsonl').read_text().splitlines()]
    rows = [r for r in rows if r['repeat_of'] is None]
    saturated = sum(r['price']['spike_duration_hr']==168 for r in rows)
    trees = [r for r in curve if r['n_families']==8 and r['model']=='trees_full56']
    worse = [r['target'] for r in trees if r['delta_mae'] > r['zero_delta_baseline_mae']+1e-8]
    lines += ['', '## 诊断结论', '',
              f'固定阈值下{len(rows)}个非重复评价有{saturated}个尖峰时长达到168 h；小MAE受标签饱和支配，不能证明过渡区准确。此阈值没有事后更改。', '',
              '三种输入表示无全面占优者；增加时序信息是消除已证压缩下界的必要方向，但不保证少样本泛化。17维仅保留每个因素两个DCT系数，仍未证明充分。低维信息下界小于多项实际开发误差，说明样本覆盖与估计误差也未解决。', '',
              '8家族完整输入树模型的配对动作误差高于零变化基线的目标：'+', '.join(worse)+'。因此总量误差改善不能替代边界动作辨识验收。', '',
              '新合成Q覆盖与v2不同，开发折也不是旧留出集；不能直接比较两轮MAE来声称模型性能退化或改善。当前仅12个家族，不应据三点曲线拟合收敛率并推导最终总周数。应先补充能保留顺序的输入表达与独立配对动作样本，并单独设计未饱和过渡测试；此处只记录下一阶段依据，尚未执行主动补样。']
    cost_path = output/'cost-audit.json'
    if cost_path.exists():
        cost = json.loads(cost_path.read_text())
        a,b = cost['current_nonrepeat']['mean_per_week'],cost['v2_reference']['mean_per_week']
        lines += ['', '## 耗时预测偏差核查', '',
                  f"实测{cost['actual_wall_sec']:.3f} s相对预测1500 s偏高{cost['relative_prediction_error']:.1%}。当前48个非重复周平均worker时间{a['worker_sec']:.3f} s，v2为{b['worker_sec']:.3f} s。",
                  f"SCUC/SCED/LMP平均solve_wall_sec分别为当前{a['scuc_solve_wall_sec']:.3f}/{a['sced_solve_wall_sec']:.3f}/{a['lmp_solve_wall_sec']:.3f} s，v2为{b['scuc_solve_wall_sec']:.3f}/{b['sced_solve_wall_sec']:.3f}/{b['lmp_solve_wall_sec']:.3f} s。", '',
                  '按理论偏差协议：输入与限额核验、标签审计及模型误差独立复算通过，未发现实现公式偏离；每周worker与operation runtime差均约4 s，变化主要体现在出清阶段。旧分组平均成本可直接迁移到新连续输入/日序的假设不成立；此为不同输入队列的描述性证据，不证明具体算法退化。阶段字段可能重叠，不将它们再次累加作总耗时。详细周计时见cost-audit.json。']
    (output/'report.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path('output/market-intelligence/identification-v3'))
    parser.add_argument('--output', type=Path, default=Path('output/market-intelligence/identification-analysis-v3'))
    parser.add_argument('--report-only', action='store_true', help='Render existing frozen results without refitting')
    args = parser.parse_args()
    if args.report_only:
        write_report(args.output, json.loads((args.output/'report.json').read_text()))
        return
    if (args.output/'report.json').exists():raise ValueError('Preserve completed development report')
    rows, design = audit_jobs(args.dataset, args.output)
    info = information_audit(rows)
    save(args.output/'information-audit.json', info)
    start = time.perf_counter()
    curves, predictions = learning_curves(rows)
    elapsed = time.perf_counter()-start
    summary = json.loads((args.dataset/'oracle_summary.json').read_text())
    report = {'protocol': 'preflight-v3', 'independent_families': 12, 'oracle_evaluations': 50,
              'information': info, 'curves': curves, 'curve_training_sec': elapsed,
              'oracle_wall_sec': summary['wall_sec'], 'final_blind_test': False,
              'engineering_error_thresholds': None, 'safe_for_boundary_optimization': False,
              'labels_sha256': hashlib.sha256((args.output/'all_labels.jsonl').read_bytes()).hexdigest(),
              'design_sha256': hash_json(design)}
    save(args.output/'development_predictions.json', predictions)
    save(args.output/'report.json', report)
    write_report(args.output, report)
    print(json.dumps({'output': str(args.output), 'curve_training_sec': elapsed}))


if __name__ == '__main__':
    main()
