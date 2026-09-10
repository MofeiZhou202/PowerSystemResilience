#!/usr/bin/env python3
"""Predeclared equal-energy temporal identification test; hydro theory §12."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from analyze_hydro_transition import load_pairs, selection
from hydro_physics import feature, physics_gain
from hydro_temporal import temporal_feature, network_feature, TemporalGP
from run_price_oracle import ROOT, digest, save


def evaluate(y, predicted):
    y, p = np.asarray(y), np.asarray(predicted)
    error = np.abs(y-p)
    positive = y >= 100  # User-selected research threshold; theory §12.1.
    return {'mae_mwh':float(error.mean()),'max_error_mwh':float(error.max()),
        'positive_cases':int(positive.sum()),'positive_misses':int(np.sum(positive & (p < 100))),
        'positive_direction_misses':int(np.sum(positive & (p <= 0))),
        'count':len(y)}


def collision_audit(pairs, specs):
    """Empirical L1 optimum conditional on colliding daily inputs (§12.1)."""
    cases=[]
    for root in range(6):
        for delta in (-300,300):
            rows=[r for r in pairs if specs[r['index']]['root_family']==root and r['delta_mwh']==delta]
            if len(rows)!=4:
                raise ValueError('Incomplete factorial collision group')
            values=np.array([r['gain_mwh'] for r in rows])
            cases.append({'root_family':root,'split':specs[rows[0]['index']]['split'],'delta_mwh':delta,
                'gain_by_shape_condition':{specs[r['index']]['boundary_key']:r['gain_mwh'] for r in rows},
                'spread_mwh':float(np.ptp(values)),
                'minimum_empirical_daily_only_mae_mwh':float(np.mean(np.abs(values-np.median(values))))})
    effects=[]
    for root in range(6):
        for delta in (-300,300):
            values=next(c['gain_by_shape_condition'] for c in cases if c['root_family']==root and c['delta_mwh']==delta)
            effects.append({'root_family':root,'delta_mwh':delta,
                'shape_effect_by_condition_mwh':[values[f'1-{c}']-values[f'0-{c}'] for c in (0,1)],
                'condition_effect_by_shape_mwh':[values[f'{s}-1']-values[f'{s}-0'] for s in (0,1)],
                'interaction_mwh':values['1-1']-values['1-0']-values['0-1']+values['0-0']})
    return {'groups':cases,'effects':effects,'interpretation':'Empirical equal-coordinate L1 floor only; no population risk guarantee'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-v3')
    p.add_argument('--output',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-analysis-v3')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if (args.output/'report.json').exists() or (args.output/'model-freeze.json').exists():
        raise ValueError('Frozen analysis already exists; do not repeat held-out selection')
    design=json.loads((args.input/'design.json').read_text());specs=design['specs']
    protocol=json.loads((args.input/'protocol-freeze.json').read_text())
    if protocol['design_sha256']!=digest(design):raise ValueError('Protocol/design mismatch')
    for relative,expected in protocol['source_hashes'].items():
        if hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()!=expected:
            raise ValueError('Protocol source changed before test: '+relative)
    if len(specs)!=72 or design['protocol']!='hydro-temporal-v3':
        raise ValueError('Unexpected frozen design')
    # All features are computed from frozen inputs, never realized UC or labels.
    daily=np.array([feature(design['boundaries'][s['boundary_key']],s) for s in specs])
    temporal=np.array([temporal_feature(design['boundaries'][s['boundary_key']],s) for s in specs])
    network=np.array([network_feature(design['boundaries'][s['boundary_key']],s) for s in specs])
    train,train_labels=load_pairs(args.input,design,'train')
    if len(train)!=48:
        raise ValueError('Expected all 48 training evaluations; failed cases cannot be dropped')
    models={};fits=[]
    for n in (2,4):
        subset=[r for r in train if specs[r['index']]['root_family']<n]
        indices=[r['index'] for r in subset];y=np.array([r['gain_mwh'] for r in subset])
        t=time.perf_counter()
        model={name:TemporalGP().fit(x[indices],y) for name,x in (('daily_gp',daily),('temporal_gp',temporal),('network_gp',network))}
        fit_sec=time.perf_counter()-t
        artifacts={}
        for name,obj in model.items():
            path=args.output/f'{name}-{n}.npz';obj.save(path)
            artifacts[name]=hashlib.sha256(path.read_bytes()).hexdigest()
        costs=[json.loads((args.input/f'week-{i:03d}/manifest.json').read_text())['wall_sec'] for i in indices]
        fits.append({'n_families':n,'train_root_families':list(range(n)), 'training_evaluations':len(indices),
            'fit_sec':fit_sec,'oracle_worker_sec':sum(costs),'artifact_hashes':artifacts,
            'training_labels_sha256':digest([{'index':r['index'],'gain_mwh':r['gain_mwh']} for r in subset])})
        models[n]=model
    frozen={'design_sha256':digest(design),'fits':fits,'training_before_test':True,
        'primary_model':'temporal_gp','primary_training_families':4,
        'hyperparameters':{'amplitude_mwh':50,'noise_mwh':5,'lengthscale':1},
        'training_feature_min':temporal[:48].min(axis=0).tolist(),
        'training_feature_max':temporal[:48].max(axis=0).tolist(),
        'boundary_hashes':{k:digest(v) for k,v in design['boundaries'].items()},
        'operation_contract':{k:v for k,v in specs[0]['config'].items() if k not in ('days','reference_days')},
        'research_gate':design['research_gate'],'production_admitted':False}
    save(args.output/'model-freeze.json',frozen)
    # First test-label read is below the artifact freeze; models never change.
    test,test_labels=load_pairs(args.input,design,'test')
    if len(test)!=24:
        raise ValueError('Expected all 24 test evaluations; failed cases cannot be dropped')
    indices=[r['index'] for r in test];y=np.array([r['gain_mwh'] for r in test])
    active=np.array([r['delta_mwh']!=0 for r in test]);results=[];predictions=[]
    for n in (2,4):
        estimates={'zero_gain':np.zeros(len(test)),'physics_only':physics_gain(daily[indices]),
            'daily_gp':models[n]['daily_gp'].predict(daily[indices]),
            'temporal_gp':models[n]['temporal_gp'].predict(temporal[indices]),
            'network_physics':physics_gain(network[indices,:5]),
            'network_gp':models[n]['network_gp'].predict(network[indices])}
        for name,values in estimates.items():
            values[~active]=0.
            rows=[{'index':r['index'],'family':r['family'],'root_family':specs[r['index']]['root_family'],
                'shape':specs[r['index']]['shape'],'condition':specs[r['index']]['condition'],
                'model':name,'n_families':n,'delta_mwh':r['delta_mwh'],'actual_gain_mwh':r['gain_mwh'],
                'predicted_gain_mwh':float(value),'train_root_families':list(range(n))}
                for r,value in zip(test,values)]
            predictions.extend(rows);choices=selection(rows)
            score=evaluate(y[active],values[active])
            strata={}
            for key in ('shape','condition'):
                for value in (0,1):
                    mask=active & np.array([r[key]==value for r in rows])
                    strata[f'{key}_{value}']=evaluate(y[mask],values[mask])
            results.append({'model':name,'n_families':n,**score,'strata':strata,'selection':choices,
                'mean_regret_mwh':float(np.mean([r['regret_mwh'] for r in choices])),
                'max_regret_mwh':max(r['regret_mwh'] for r in choices)})
    timings=[];full=[];model=models[4]['temporal_gp'];spec=specs[48];b=design['boundaries'][spec['boundary_key']]
    for _ in range(1000):
        start=time.perf_counter_ns();model.predict(temporal[48]);timings.append((time.perf_counter_ns()-start)/1e6)
    for _ in range(100):
        start=time.perf_counter_ns();model.predict(temporal_feature(b,spec));full.append((time.perf_counter_ns()-start)/1e6)
    main_result=next(r for r in results if r['n_families']==4 and r['model']=='temporal_gp')
    comparator=next(r for r in results if r['n_families']==4 and r['model']=='daily_gp')
    improvement=1-main_result['mae_mwh']/comparator['mae_mwh'] if comparator['mae_mwh']>1e-9 else None
    gate=(main_result['mae_mwh']<=25 and main_result['max_error_mwh']<=50 and
          main_result['positive_cases']>0 and main_result['positive_misses']==0 and np.quantile(full,.95)<=1000)
    pairs=train+test;labels=[train_labels[i] if s['split']=='train' else test_labels[i] for i,s in enumerate(specs)]
    oracle=json.loads((args.input/'oracle_summary.json').read_text())
    call_wall=sum(json.loads(path.read_text())['wall_sec'] for path in args.input.glob('invocation-*.json'))
    report={'protocol':design['protocol'],'design_sha256':digest(design),'fits':fits,'results':results,
        'predictions':predictions,'pairs':pairs,'eligible_evaluations':len(pairs),
        'collision_audit':collision_audit(pairs,specs),'research_gate_passed':bool(gate),
        'mae_reduction_vs_daily_gp':improvement,'production_admitted':False,
        'oracle_wall_sec':call_wall,'last_invocation_wall_sec':oracle['wall_sec'],'oracle_worker_sec':oracle['worker_sum_sec'],
        'predicted_wall_sec':6000,'oracle_mean_worker_sec':oracle['worker_sum_sec']/72,
        'inference_ms':{'cached_median':float(np.median(timings)),'cached_p95':float(np.quantile(timings,.95)),
                        'full_context_median':float(np.median(full)),'full_context_p95':float(np.quantile(full,.95))},
        'test_outside_training_box':int(np.any((temporal[indices]<temporal[:48].min(axis=0)-1e-10)|
            (temporal[indices]>temporal[:48].max(axis=0)+1e-10),axis=1).sum()),
        'diagnostic_safety':{'max_deficit_mwh':max(r['deficit_mwh'] for r in pairs),
                             'max_overload_mwh':max(r['overload_mwh'] for r in pairs)},
        'limitations':['Two independent synthetic test roots, not 24 independent test weeks',
            'All decisions require Oracle validation; GP uncertainty is not an error certificate',
            'Coupled thermal-condition package does not isolate individual UC parameters',
            'Original linear rolling diagnostic model; no AC/N-1 or global weekly optimum',
            'SCUC 1% objective gap does not certify MWh label accuracy; no production admission']}
    save(args.output/'labels.json',labels);save(args.output/'report.json',report)
    lines=['# 同日电量下的日内形状与火电条件补样','',
        '6个独立外生周×2形状×2火电条件×3动作=72次full定价周Oracle；4周训练、2周测试。',
        '主模型预先指定为4训练周的temporal_gp；所有模型在读取测试标签前冻结。','',
        '| 训练独立周 | 模型 | MAE MWh | 最大误差 MWh | ≥100 MWh正收益漏判 | 方向漏判 | 平均后悔值 MWh |',
        '|---|---|---:|---:|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['n_families']} | {r['model']} | {r['mae_mwh']:.3f} | {r['max_error_mwh']:.3f} | {r['positive_misses']}/{r['positive_cases']} | {r['positive_direction_misses']} | {r['mean_regret_mwh']:.3f} |")
    lines += ['',f'用户研究门槛通过：{gate}；生产准入：否。≥100 MWh真实正收益预测不足100 MWh记为漏判。',
        f"Oracle各次调用累计墙钟{call_wall:.1f}秒；累计worker成本{oracle['worker_sum_sec']:.1f}秒；事前全批墙钟预测6000秒。",
        f'本地缓存预测P95 {np.quantile(timings,.95):.4f} ms；完整特征与预测P95 {np.quantile(full,.95):.3f} ms，非HTTP并发SLA。','',
        '## 误差—成本曲线数据','', '| 训练独立周 | 训练Oracle worker秒 | 同时拟合三个模型秒 |','|---|---:|---:|']
    for f in fits:lines.append(f"| {f['n_families']} | {f['oracle_worker_sec']:.2f} | {f['fit_sec']:.6f} |")
    lines += ['', '## 等日电量的可辨识性','', '| 外生周 | 动作 MWh | 四变体增益极差 MWh | 日能量坐标经验MAE下界 MWh |','|---|---:|---:|---:|']
    for r in report['collision_audit']['groups']:
        lines.append(f"| {r['root_family']} ({r['split']}) | {r['delta_mwh']} | {r['spread_mwh']:.3f} | {r['minimum_empirical_daily_only_mae_mwh']:.3f} |")
    lines += ['', '## 验证边界','']+['- '+s for s in report['limitations']]
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'primary':main_result,'research_gate_passed':bool(gate),'improvement':improvement,'inference_ms':report['inference_ms']}))


if __name__=='__main__':main()
