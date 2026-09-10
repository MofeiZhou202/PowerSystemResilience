#!/usr/bin/env python3
"""Prospective nonconstant-response test of unchanged models; hydro theory §15."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from analyze_hydro_temporal import evaluate
from hydro_allocation import terminal_contract, compile_action
from hydro_physics import daily_surplus, renewable_energy, physics_gain, feature
from hydro_strict_model import FEATURES, StrictPredictor
from run_hydro_strict import GAPS, execute, audit_level, stability_report, frozen_save, verify_model_freeze
from run_price_oracle import ROOT, digest, save
from validate_hydro_temporal import close

TARGETS = ((110., -200.), (90., -200.), (-100., 100.))


def build_challenge(parent):
    """Force distinct hinge responses while retaining the original admitted input support."""
    design=copy.deepcopy(parent);design['specs']=[]
    rng=np.random.default_rng(20261520)
    b0=parent['boundaries']['0-0'];mapping=parent['mapping']
    load=sum(sum(a['load_mw'][:96])*.25 for a in b0['areas'])
    baseline={str(s['id']):[3000.]*7 for s in mapping};weekly={sid:21000. for sid in baseline}
    for offset,targets in enumerate(TARGETS):
        root=6+offset
        external=[]
        for _ in range(8):
            external.append(dict(load_scale=float(rng.uniform(.6,.85)),wind_scale=float(rng.uniform(1,1.8)),
                solar_scale=float(rng.uniform(1,1.8)),inflow_scale=float(rng.uniform(.6,1)),
                line_limit_scale=float(rng.uniform(.8,1)),generator_bid_scale=1.,load_bid_scale=1.,bid_scale=1.,
                first_slot=0,last_slot=95,generator_outages=[],branch_outages=[]))
        for d,target in zip((1,4),targets):
            external[d]['load_scale']=(36000+renewable_energy(b0,external[d])-target)/load
        terminal=terminal_contract(b0,mapping,external,baseline)
        for shape in (0,1):
            for condition in (0,1):
                key=f'{shape}-{condition}';b=design['boundaries'][key]
                for delta in (0.,150.):
                    quotas=copy.deepcopy(baseline);quotas['1'][1]-=delta;quotas['1'][4]+=delta
                    config=copy.deepcopy(parent['specs'][0]['config'])
                    config['reference_days']=copy.deepcopy(external)
                    config['days']=compile_action(b,mapping,external,quotas,weekly,terminal)
                    spec=dict(root_family=root,family=root*4+shape*2+condition,split='test',shape=shape,condition=condition,
                        regime=('above-threshold','below-threshold','adverse')[offset],boundary_key=key,boundary_sha256=digest(b),
                        delta_mwh=delta,config=config,quotas=quotas,weekly=weekly,terminal=terminal,
                        external_days=copy.deepcopy(external),target_r2_r5_mwh=list(targets))
                    close(daily_surplus(b,spec)[[1,4]],targets,1e-6,'challenge target construction')
                    design['specs'].append(spec)
    design.update(protocol='hydro-strict-challenge-v1',source_parent_design_sha256=digest(parent),
        planned_evaluations=48,independent_families=3,predicted_wall_sec=2000,
        train_roots=[],test_roots=[6,7,8],scope='Fixed v4 models; prospective distinct-gain challenge, no refitting')
    return design


def score_models(model_root,output,design,stable,frozen):
    rows=stable['rows'];specs=design['specs'];active=np.array([r['delta_mwh']!=0 for r in rows])
    y=np.array([r['gain_mwh'] for r in rows]);results=[];predictions=[];reconstruction=0.
    for n in (2,4):
        estimates={}
        for name in FEATURES:
            predictor=StrictPredictor(model_root,name,n)
            estimates[name]=np.array([predictor.predict(design['boundaries'][s['boundary_key']],s)['predicted_gain_mwh'] for s in specs])
        estimates['physics_only']=physics_gain(np.array([feature(design['boundaries'][s['boundary_key']],s) for s in specs]))
        # §15: deliberately naive diagnostic reference that passes the old 150 MWh plateau.
        estimates['gain_equals_delta']=np.array([s['delta_mwh'] for s in specs])
        estimates['zero_gain']=np.zeros(len(specs))
        for name,values in estimates.items():
            metrics=evaluate(y[active],values[active])
            false_positive=int(np.sum(active & (y<100) & (values>=100)))
            strata={}
            for regime in ('above-threshold','below-threshold','adverse'):
                mask=active & np.array([s['regime']==regime for s in specs])
                strata[regime]=evaluate(y[mask],values[mask])
            abs_errors=[abs(float(a)-float(b)) for a,b in zip(y[active],values[active])]
            reconstruction=max(reconstruction,close(sum(abs_errors)/12,metrics['mae_mwh'],1e-8,'independent challenge MAE'),
                               close(max(abs_errors),metrics['max_error_mwh'],1e-8,'independent challenge maximum'))
            results.append(dict(model=name,n_families=n,**metrics,false_positive_at_100_mwh=false_positive,strata=strata))
            predictions.extend(dict(index=r['index'],root_family=r['root_family'],family=r['family'],delta_mwh=r['delta_mwh'],
                model=name,n_families=n,actual_gain_mwh=float(a),predicted_gain_mwh=float(b),
                pair_diagnostic_pass=r['pair_diagnostic_pass'],train_root_families=list(range(n)))
                for r,a,b in zip(rows,y,values))
    predictor=StrictPredictor(model_root)
    timings=[]
    for k in range(100):
        s=specs[k%len(specs)]
        timings.append(predictor.predict(design['boundaries'][s['boundary_key']],s)['full_inference_ms'])
    timing=dict(median=float(np.median(timings)),p95=float(np.quantile(timings,.95)),maximum=max(timings))
    primary=next(r for r in results if r['model']=='network_gp' and r['n_families']==4)
    gate=bool(stable['all_stable'] and stable['all_pairs_diagnostic_pass'] and primary['mae_mwh']<=25 and
        primary['max_error_mwh']<=50 and primary['positive_cases']>0 and primary['positive_misses']==0 and timing['maximum']<=1000)
    if digest(verify_model_freeze(model_root,json.loads((model_root/'design.json').read_text())))!=digest(frozen):
        raise ValueError('Original models or training freeze changed during challenge')
    report=dict(protocol=design['protocol'],design_sha256=digest(design),model_freeze_sha256=digest(frozen),
        independent_test_roots=[6,7,8],results=results,predictions=predictions,primary_model='network_gp-4',
        research_gate_passed=gate,production_admitted=False,inference_ms=timing,
        actual_nonzero_gain_range_mwh=[float(y[active].min()),float(y[active].max())],
        all_test_pairs_diagnostic_pass=stable['all_pairs_diagnostic_pass'],all_labels_stable=stable['all_stable'],
        maximum_metric_reconstruction_error_mwh=reconstruction,models_refitted=False,
        limitations=['Prospective challenge selected after identifying the old plateau; not iid population sampling',
                     'Fixed synthetic bases, station1 day2/day5 only; no AC/N-1 or global seven-day optimization certificate'])
    frozen_save(output/'report.json',report)
    lines=['# 固定模型的非恒定收益挑战','',f'研究门槛通过：{gate}；模型未重新拟合；生产准入：否。','',
        '| 模型 | 训练根 | MAE MWh | 最大误差 MWh | ≥100漏判/真阳性 | <100误报数 |',
        '|---|---:|---:|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['model']} | {r['n_families']} | {r['mae_mwh']:.4f} | {r['max_error_mwh']:.4f} | {r['positive_misses']}/{r['positive_cases']} | {r['false_positive_at_100_mwh']} |")
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(primary=primary,research_gate_passed=gate,timing=timing,gain_range=report['actual_nonzero_gain_range_mwh'])))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model-root',type=Path,default=ROOT/'output/market-intelligence/hydro-strict-v4')
    p.add_argument('--input',type=Path,default=ROOT/'output/market-performance/live-input')
    p.add_argument('--output',type=Path,default=ROOT/'output/market-intelligence/hydro-strict-v4/challenge-v1')
    p.add_argument('--server',type=Path,default=ROOT/'build/macos-release/tests/run_gui_server')
    p.add_argument('--workers',type=int,choices=(1,2,4),default=4)
    p.add_argument('--prepare-only',action='store_true')
    args=p.parse_args();args.phase='challenge'
    parent=json.loads((args.model_root/'design.json').read_text());frozen=verify_model_freeze(args.model_root,parent)
    design=build_challenge(parent)
    if hashlib.sha256(args.server.read_bytes()).hexdigest()!=design['binary_sha256']:raise ValueError('Oracle binary changed')
    args.output.mkdir(parents=True,exist_ok=True)
    frozen_save(args.output/'design.json',design)
    marker=args.output/'challenge-freeze.json'
    if marker.exists():
        freeze=json.loads(marker.read_text())
        if freeze['design_sha256']!=digest(design) or freeze['model_freeze_sha256']!=digest(frozen):
            raise ValueError('Challenge protocol/model identity changed')
        if freeze['source_sha256']!=hashlib.sha256(Path(__file__).read_bytes()).hexdigest():
            raise ValueError('Challenge analysis source changed since preregistration')
    else:
        if list(args.output.glob('gap-*/week-*')):raise ValueError('Unregistered challenge labels already exist')
        frozen_save(marker,dict(frozen_utc=datetime.now(timezone.utc).isoformat(),design_sha256=digest(design),
            model_freeze_sha256=digest(frozen),source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            challenge_oracles_absent=True,models_refitted=False))
    if args.prepare_only:
        print(json.dumps(dict(prepared_evaluations=48,design_sha256=digest(design))));return
    if (args.output/'report.json').exists():raise ValueError('Preserve existing challenge result; no repeat selection')
    started=time.monotonic()
    execute(args.output,design,args,list(range(24)))
    levels=[audit_level(args.output,design,gap,24) for gap in GAPS]
    stable=stability_report(*levels)
    stable.update(design_sha256=digest(design),audits=[{k:v for k,v in l.items() if k!='rows'} for l in levels],
                  wall_sec=time.monotonic()-started)
    save(args.output/'test-report.json',stable)
    score_models(args.model_root,args.output,design,stable,frozen)


if __name__=='__main__':main()
