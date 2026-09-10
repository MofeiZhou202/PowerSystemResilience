#!/usr/bin/env python3
"""Prospective strict-label model freeze and fixed held-out evaluation; theory §14."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from analyze_hydro_temporal import evaluate
from hydro_physics import physics_gain
from hydro_temporal import TemporalGP
from hydro_strict_model import FEATURES, StrictPredictor
from run_hydro_strict import GAPS, gap_dir, frozen_save, verify_model_freeze
from run_price_oracle import ROOT, digest, save


def load_report(root, phase, design):
    report = json.loads((root/f'{phase}-report.json').read_text())
    if report['design_sha256'] != digest(design):
        raise ValueError('Audit and protocol identities differ')
    count = 32 if phase == 'train' else 48
    if len(report['rows']) != count or [r['index'] for r in report['rows']] != list(range(count)):
        raise ValueError('Incomplete audited cohort; do not drop cases')
    return report


def train(root, design):
    out = root/'analysis'
    out.mkdir(exist_ok=True)
    if (out/'model-freeze.json').exists():
        raise ValueError('Models already frozen; do not refit against held-out outcomes')
    for gap in GAPS:
        if any((gap_dir(root, gap)/f'week-{i:03d}').exists() for i in range(32,48)):
            raise ValueError('Held-out Oracle started before model freeze')
    report = load_report(root, 'train', design)
    if not report['all_stable']:
        raise ValueError('Training labels are not stable; retain failed cohort')
    specs = design['specs']
    x = {name:np.array([compute(design['boundaries'][s['boundary_key']],s) for s in specs[:32]])
         for name,compute in FEATURES.items()}
    fits = []
    for n in (2,4):
        rows = [r for r in report['rows'] if r['root_family'] < n]
        indices = [r['index'] for r in rows]
        y = np.array([r['gain_mwh'] for r in rows])
        start = time.perf_counter()
        models = {name:TemporalGP().fit(values[indices],y) for name,values in x.items()}
        elapsed = time.perf_counter()-start
        hashes = {}
        for name,obj in models.items():
            path = out/f'{name}-{n}.npz'
            obj.save(path)
            hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        # Both precision levels are a labeling cost; do not hide the stability audit cost.
        costs = [json.loads((gap_dir(root,gap)/f'week-{i:03d}/manifest.json').read_text())['wall_sec']
                 for gap in GAPS for i in indices]
        fits.append(dict(n_families=n, train_root_families=list(range(n)), training_evaluations=len(indices),
            labeling_evaluations=2*len(indices), oracle_worker_sec=sum(costs), fit_sec=elapsed,
            training_labels_sha256=digest([{'index':r['index'],'gain_mwh':r['gain_mwh']} for r in rows]),
            artifact_hashes=hashes))
    sources = ['analyze_hydro_strict.py','run_hydro_strict.py','hydro_strict_model.py','hydro_physics.py',
               'hydro_temporal.py','hydro_complete_context.py','validate_hydro_temporal.py',
               'hydro_allocation.py','run_hydro_oracle.py','run_price_oracle.py','build_market_label_dataset.py',
               'analyze_hydro_temporal.py','validate_hydro_strict.py','predict_hydro_strict.py']
    frozen = dict(frozen_utc=datetime.now(timezone.utc).isoformat(), design_sha256=digest(design), fits=fits, primary_model='network_gp', primary_training_families=4,
        test_oracles_absent_at_freeze=True, training_audit_sha256=digest(report),
        hyperparameters=dict(amplitude_mwh=50, noise_mwh=5, lengthscale=1), production_admitted=False,
        source_hashes={p:hashlib.sha256((ROOT/'tools/market_intelligence'/p).read_bytes()).hexdigest() for p in sources})
    frozen_save(out/'model-freeze.json',frozen)
    print(json.dumps(dict(models_frozen=8,training_labels_stable=True,test_oracles_absent=True,fits=fits)))


def selection(rows, predictions):
    """Diagnostic passing candidates only; explicitly preserve no-candidate groups (§14.3)."""
    choices = []
    for family in sorted({r['family'] for r in rows}):
        options = [(r,p) for r,p in zip(rows,predictions) if r['family']==family and r['pair_diagnostic_pass']]
        if not options:
            choices.append(dict(family=family,status='no passing diagnostic candidate',regret_mwh=None))
            continue
        chosen, value = max(options,key=lambda item:(item[1],item[0]['delta_mwh']==0))
        best = max(r['gain_mwh'] for r,_ in options)
        choices.append(dict(family=family,status='diagnostic comparison only',selected_delta_mwh=chosen['delta_mwh'],
                            predicted_gain_mwh=float(value),actual_gain_mwh=chosen['gain_mwh'],
                            regret_mwh=best-chosen['gain_mwh']))
    return choices


def evaluate_models(root, design):
    out = root/'analysis'
    if (out/'report.json').exists():
        raise ValueError('Held-out report already exists; preserve frozen result')
    frozen = verify_model_freeze(root,design)
    for name,expected in frozen['source_hashes'].items():
        if hashlib.sha256((ROOT/'tools/market_intelligence'/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('Source changed since training freeze: '+name)
    audit = load_report(root,'test',design)
    rows = audit['rows'][32:]
    if {r['root_family'] for r in rows}!={4,5} or len(rows)!=16:
        raise ValueError('Wrong fixed held-out cohort')
    specs = design['specs'][32:]
    x = {name:np.array([compute(design['boundaries'][s['boundary_key']],s) for s in specs])
         for name,compute in FEATURES.items()}
    y = np.array([r['gain_mwh'] for r in rows])
    active = np.array([bool(r['delta_mwh']) for r in rows])
    feasible = np.array([r['pair_diagnostic_pass'] for r in rows])
    results, predictions = [], []
    for n in (2,4):
        estimates = {name:TemporalGP.load(out/f'{name}-{n}.npz').predict(values) for name,values in x.items()}
        estimates.update(zero_gain=np.zeros(16),physics_only=physics_gain(x['daily_gp'][:,:5]),
                         network_physics=physics_gain(x['network_gp'][:,:5]))
        for name,values in estimates.items():
            metric = evaluate(y[active],values[active])
            safe_metric = evaluate(y[active & feasible],values[active & feasible]) if np.any(active & feasible) else None
            strata = {}
            for key in ('condition','shape','regime'):
                for value in sorted({s[key] for s in specs}):
                    mask = active & np.array([s[key]==value for s in specs])
                    strata[f'{key}={value}'] = evaluate(y[mask],values[mask])
            choices = selection(rows,values)
            valid_regret = [r['regret_mwh'] for r in choices if r['regret_mwh'] is not None]
            results.append(dict(model=name,n_families=n,**metric,diagnostic_passing=safe_metric,strata=strata,
                selection=choices,mean_diagnostic_regret_mwh=float(np.mean(valid_regret)) if valid_regret else None,
                no_passing_candidate_groups=sum(r['regret_mwh'] is None for r in choices)))
            predictions.extend(dict(index=r['index'],family=r['family'],root_family=r['root_family'],
                delta_mwh=r['delta_mwh'],actual_gain_mwh=r['gain_mwh'],predicted_gain_mwh=float(v),
                model=name,n_families=n,train_root_families=list(range(n))) for r,v in zip(rows,values))
    timings, reload_errors = {}, {}
    for name in FEATURES:
        predictor = StrictPredictor(root,name)
        expected = [p['predicted_gain_mwh'] for p in predictions if p['model']==name and p['n_families']==4]
        reloaded = [predictor.predict(design['boundaries'][s['boundary_key']],s)['predicted_gain_mwh'] for s in specs]
        reload_errors[name] = float(np.max(np.abs(np.array(reloaded)-expected)))
        values = []
        for j in range(100):
            s = specs[j%len(specs)]
            values.append(predictor.predict(design['boundaries'][s['boundary_key']],s)['full_inference_ms'])
        timings[name] = dict(median=float(np.median(values)),p95=float(np.quantile(values,.95)),maximum=max(values))
    primary = next(r for r in results if r['model']=='network_gp' and r['n_families']==4)
    test_stable = all(r['stable'] for r in rows)
    test_screen = all(r['pair_diagnostic_pass'] for r in rows)
    speed = timings['network_gp']['maximum']<=1000
    gate = bool(test_stable and test_screen and primary['mae_mwh']<=25 and primary['max_error_mwh']<=50
                and primary['positive_cases']>0 and primary['positive_misses']==0 and speed)
    report = dict(protocol=design['protocol'],design_sha256=digest(design),fits=frozen['fits'],results=results,
        predictions=predictions,independent_test_roots=[4,5],nonzero_test_actions=8,
        primary_model='network_gp',research_gate_passed=gate,production_admitted=False,
        test_labels_stable=test_stable,all_test_pairs_diagnostic_pass=test_screen,
        primary_mae_with_5_mwh_label_budget_within_25=primary['mae_mwh']<=20,
        inference_ms=timings,reload_max_errors_mwh=reload_errors,
        limitations=['Two independent synthetic test roots; siblings are correlated',
            'Adjacent-gap agreement is not a true-label error certificate',
            'Line factors [0.8,1] only; earlier [0.4,1] stress domain not certified',
            'Diagnostic screen is not AC/N-1 safety; no joint weekly global optimality',
            'Preloaded local inference benchmark; HTTP and concurrency SLA untested'])
    save(out/'report.json',report)
    lines=['# 严格Oracle标签与新留出周验收','',f'研究门槛通过：{gate}；生产准入：否。',
           '主模型为预指定network_gp，4训练外生周、2全新测试外生周，8个非零测试动作。','',
           '| 模型 | 训练根数 | MAE MWh | 最大误差 MWh | ≥100 MWh漏判/真阳性 |',
           '|---|---:|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['model']} | {r['n_families']} | {r['mae_mwh']:.3f} | {r['max_error_mwh']:.3f} | {r['positive_misses']}/{r['positive_cases']} |")
    lines += ['',f'测试双精度标签稳定：{test_stable}；全部测试配对通过诊断平衡/潮流筛查：{test_screen}。',
              f"主模型本地完整请求P95 {timings['network_gp']['p95']:.3f} ms，最大{timings['network_gp']['maximum']:.3f} ms。",'',
              '验证范围：']+['- '+s for s in report['limitations']]
    (out/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(primary=primary,research_gate_passed=gate,inference_ms=timings)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--phase',choices=('train','evaluate'),required=True)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-strict-v4')
    args=p.parse_args()
    design=json.loads((args.input/'design.json').read_text())
    if design['protocol']!='hydro-strict-v4':raise ValueError('Unexpected design')
    (train if args.phase=='train' else evaluate_models)(args.input,design)


if __name__=='__main__':main()
