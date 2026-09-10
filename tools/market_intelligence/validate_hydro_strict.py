#!/usr/bin/env python3
"""Independent raw-label, gap, test chronology and metric audit; hydro theory §14."""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from hydro_physics import physics_gain
from hydro_strict_model import FEATURES
from hydro_temporal import TemporalGP
from run_hydro_strict import GAPS, gap_dir, verify_model_freeze
from run_price_oracle import ROOT, digest, save
from validate_hydro_temporal import audit_raw, close


def validate(root):
    design = json.loads((root/'design.json').read_text())
    frozen = verify_model_freeze(root, design)
    report = json.loads((root/'analysis/report.json').read_text())
    start = json.loads((root/'test-start.json').read_text())
    if (report['design_sha256'] != digest(design) or start['model_freeze_sha256'] != digest(frozen)
        or not start['test_oracles_absent'] or datetime.fromisoformat(start['started_utc']) < datetime.fromisoformat(frozen['frozen_utc'])):
        raise ValueError('Prospective testing identity/chronology failed')
    strict_report = json.loads((root/'test-report.json').read_text())
    curtailment = {}
    audits = []
    for gap in GAPS:
        level_design = json.loads((gap_dir(root,gap)/'design.json').read_text())
        expected = copy.deepcopy(design)
        for spec in expected['specs']:
            spec['config']['solver_options']['mip_gap'] = gap
        expected['requested_gap'] = gap
        if digest(expected) != digest(level_design):
            raise ValueError('Gap-specific execution differs from frozen physical inputs')
        raw, cur = audit_raw(gap_dir(root,gap),level_design)
        if raw['raw_weeks'] != 48 or raw['limit_reached_stages'] or raw['max_reported_gap'] > gap+1e-10:
            raise ValueError('Strict Oracle count/precision mismatch')
        curtailment[gap] = cur
        audits.append({k:v for k,v in raw.items() if k!='daily_energy'})
    targets = {}
    max_error = 0.
    for i,s in enumerate(design['specs']):
        base = next(j for j,b in enumerate(design['specs']) if b['family']==s['family'] and b['delta_mwh']==0)
        lo, hi = (curtailment[g][base]-curtailment[g][i] for g in GAPS)
        targets[i] = hi
        r = strict_report['rows'][i]
        max_error = max(max_error, close(hi,r['gain_mwh'],1e-8,'strict target'),
                        close(abs(hi-lo),r['gain_change_mwh'],1e-8,'adjacent paired gain'))
        magnitude = max(abs(curtailment[GAPS[1]][i]-curtailment[GAPS[0]][i]),
                        abs(curtailment[GAPS[1]][base]-curtailment[GAPS[0]][base]))
        if r['stable'] != (abs(hi-lo)<=5 and magnitude<=5):
            raise ValueError('Incorrect label stability decision')
    for fit in frozen['fits']:
        n = fit['n_families']
        train_ids = [i for i,s in enumerate(design['specs']) if s['split']=='train' and s['root_family']<n]
        target_hash = digest([dict(index=i,gain_mwh=targets[i]) for i in train_ids])
        if target_hash != fit['training_labels_sha256']:
            raise ValueError('Frozen model training targets changed')
        for name, compute in FEATURES.items():
            model = TemporalGP.load(root/f'analysis/{name}-{n}.npz')
            x = np.array([compute(design['boundaries'][design['specs'][i]['boundary_key']],design['specs'][i]) for i in train_ids])
            close(model.x,x,1e-12,'training-only feature matrix')
            # Independent sum-form fixed-kernel posterior identity, RW2006 §2.2.
            cov = 2500*np.exp(-.5*np.sum((x[:,None,:]-x[None,:,:])**2,axis=2))+25*np.eye(len(x))
            rhs = np.array([targets[i] for i in train_ids])-physics_gain(x[:,:5])
            close(cov@model.alpha,rhs,1e-7,'GP training equation')
    for score in report['results']:
        rows = [r for r in report['predictions'] if r['model']==score['model'] and r['n_families']==score['n_families']]
        if len(rows)!=16 or {r['root_family'] for r in rows}!={4,5}:
            raise ValueError('Fixed test cohort incomplete')
        active = [r for r in rows if r['delta_mwh']!=0]
        for r in rows:
            if r['root_family'] in r['train_root_families']:
                raise ValueError('Exogenous sibling leakage')
            max_error=max(max_error,close(targets[r['index']],r['actual_gain_mwh'],1e-8,'test target'))
        errors = [abs(targets[r['index']]-r['predicted_gain_mwh']) for r in active]
        max_error=max(max_error,close(sum(errors)/8,score['mae_mwh'],1e-8,'MAE'),
                      close(max(errors),score['max_error_mwh'],1e-8,'max error'))
        positive=[r for r in active if targets[r['index']]>=100]
        if score['positive_cases']!=len(positive) or score['positive_misses']!=sum(r['predicted_gain_mwh']<100 for r in positive):
            raise ValueError('High-benefit recall incorrect')
        for choice in score['selection']:
            options=[r for r in rows if r['family']==choice['family'] and strict_report['rows'][r['index']]['pair_diagnostic_pass']]
            if not options:
                if choice['regret_mwh'] is not None:raise ValueError('Invented feasible fallback')
            else:
                selected=max(options,key=lambda r:(r['predicted_gain_mwh'],r['delta_mwh']==0))
                if selected['delta_mwh']!=choice['selected_delta_mwh']:raise ValueError('Selection used hindsight')
                close(max(targets[r['index']] for r in options)-targets[selected['index']],choice['regret_mwh'],1e-8,'diagnostic regret')
    main = next(s for s in report['results'] if s['model']=='network_gp' and s['n_families']==4)
    testrows=strict_report['rows'][32:]
    expected_gate=bool(all(r['stable'] and r['pair_diagnostic_pass'] for r in testrows)
        and main['mae_mwh']<=25 and main['max_error_mwh']<=50 and main['positive_cases']>0
        and main['positive_misses']==0 and report['inference_ms']['network_gp']['maximum']<=1000)
    if expected_gate!=report['research_gate_passed']:raise ValueError('Research admission gate mismatch')
    if report['test_labels_stable'] != all(r['stable'] for r in testrows):
        raise ValueError('Test stability summary mismatch')
    if report['all_test_pairs_diagnostic_pass'] != all(r['pair_diagnostic_pass'] for r in testrows):
        raise ValueError('Test diagnostic screen summary mismatch')
    if max(report['reload_max_errors_mwh'].values()) > 1e-8:
        raise ValueError('Reloaded predictions differ from frozen evaluation')
    result=dict(passed=True,raw_weekly_evaluations=96,main_lmp_days=672,prediction_rows=len(report['predictions']),
        independent_test_roots=[4,5],maximum_metric_reconstruction_error_mwh=max_error,
        model_training_equations_verified=True,prospective_test_chronology_verified=True,
        audits=audits,research_gate_passed=expected_gate,production_admitted=False)
    save(root/'analysis/independent-validation.json',result)
    print(json.dumps(result))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-strict-v4')
    validate(p.parse_args().input)


if __name__=='__main__':main()
