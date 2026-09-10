#!/usr/bin/env python3
"""Train/freeze before any test read; complete-context diagnostic, theory §12.7."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from analyze_hydro_transition import load_pairs,selection
from analyze_hydro_temporal import evaluate
from hydro_complete_context import complete_feature
from hydro_temporal import TemporalGP
from run_price_oracle import ROOT,digest,save


def train(root,output,design):
    if (output/'model-freeze.json').exists():raise ValueError('Complete-context models already frozen')
    pairs,_=load_pairs(root,design,'train')
    if len(pairs)!=48:raise ValueError('All 48 training evaluations required')
    specs=design['specs'];fits=[]
    for n in (2,4):
        subset=[r for r in pairs if specs[r['index']]['root_family']<n]
        x=np.array([complete_feature(design['boundaries'][specs[r['index']]['boundary_key']],specs[r['index']]) for r in subset])
        y=np.array([r['gain_mwh'] for r in subset]);start=time.perf_counter()
        model=TemporalGP().fit(x,y);elapsed=time.perf_counter()-start
        path=output/f'complete_gp-{n}.npz';model.save(path)
        cost=sum(json.loads((root/f"week-{r['index']:03d}/manifest.json").read_text())['wall_sec'] for r in subset)
        fits.append({'n_families':n,'train_root_families':list(range(n)),'training_evaluations':len(subset),
            'fit_sec':elapsed,'oracle_worker_sec':cost,'artifact_hashes':{'complete_gp':hashlib.sha256(path.read_bytes()).hexdigest()},
            'training_labels_sha256':digest([{'index':r['index'],'gain_mwh':r['gain_mwh']} for r in subset])})
    source_files=['tools/market_intelligence/'+name for name in ('hydro_complete_context.py','analyze_hydro_complete_context.py','hydro_temporal.py','hydro_physics.py')]
    frozen={'design_sha256':digest(design),'fits':fits,'training_before_test':True,'model_role':'input-omission diagnostic secondary',
        'primary_model_unchanged':'temporal_gp in original v3 analysis','primary_training_families':4,
        'source_hashes':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in source_files},
        'boundary_hashes':{k:digest(v) for k,v in design['boundaries'].items()},
        'operation_contract':{k:v for k,v in specs[0]['config'].items() if k not in ('days','reference_days')},
        'research_gate':design['research_gate'],'production_admitted':False,
        'main_test_report_existed_at_freeze':(root.parent/'hydro-temporal-analysis-v3/report.json').exists()}
    if frozen['main_test_report_existed_at_freeze']:raise ValueError('Full-context training must precede original test report')
    save(output/'model-freeze.json',frozen)
    print(json.dumps({'phase':'train','fits':fits,'training_before_test':True}),flush=True)


def evaluate_saved(root,output,design):
    frozen=json.loads((output/'model-freeze.json').read_text())
    if (output/'report.json').exists():raise ValueError('Complete-context test already evaluated')
    if digest(design)!=frozen['design_sha256']:raise ValueError('Changed design')
    for p,value in frozen['source_hashes'].items():
        if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=value:raise ValueError('Frozen source changed: '+p)
    test,_=load_pairs(root,design,'test');specs=design['specs']
    if len(test)!=24:raise ValueError('All 24 held-out candidates required')
    x=np.array([complete_feature(design['boundaries'][specs[r['index']]['boundary_key']],specs[r['index']]) for r in test])
    y=np.array([r['gain_mwh'] for r in test]);active=np.array([r['delta_mwh']!=0 for r in test]);results=[];predictions=[]
    for n in (2,4):
        path=output/f'complete_gp-{n}.npz';fit=next(f for f in frozen['fits'] if f['n_families']==n)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=fit['artifact_hashes']['complete_gp']:raise ValueError('Changed saved model')
        model=TemporalGP.load(path);values=model.predict(x)
        rows=[{'index':r['index'],'family':r['family'],'root_family':specs[r['index']]['root_family'],
            'delta_mwh':r['delta_mwh'],'actual_gain_mwh':r['gain_mwh'],'predicted_gain_mwh':float(v),
            'model':'complete_gp','n_families':n,'train_root_families':list(range(n))} for r,v in zip(test,values)]
        choices=selection(rows);results.append({'model':'complete_gp','n_families':n,**evaluate(y[active],values[active]),
            'mean_regret_mwh':float(np.mean([r['regret_mwh'] for r in choices])),'selection':choices});predictions+=rows
    # Benchmark the actual validated local entry, with a preloaded model/design.
    from predict_hydro_temporal import ResearchPredictor
    predictor=ResearchPredictor(output,root/'design.json','complete_gp')
    reload_error=0.;payload=None
    for row in predictions:
        if row['n_families']!=4:continue
        payload=json.loads((root/f"week-{row['index']:03d}/input.json").read_text())
        value=predictor.predict(payload['base'],payload['spec'])['predicted_gain_mwh']
        reload_error=max(reload_error,abs(value-row['predicted_gain_mwh']))
    if reload_error>1e-8:raise ValueError('Full-context local entry differs from frozen predictions')
    times=[]
    for _ in range(100):
        start=time.perf_counter_ns();predictor.predict(payload['base'],payload['spec']);times.append((time.perf_counter_ns()-start)/1e6)
    primary=results[-1]
    gate=primary['mae_mwh']<=25 and primary['max_error_mwh']<=50 and primary['positive_cases']>0 and primary['positive_misses']==0 and max(times)<=1000
    report={'model_role':'pre-test input-omission diagnostic secondary; original primary unchanged',
        'design_sha256':digest(design),'fits':frozen['fits'],'results':results,'predictions':predictions,
        'research_gate_passed':bool(gate),'production_admitted':False,'reload_max_error_mwh':reload_error,
        'validated_request_ms':{'median':float(np.median(times)),'p95':float(np.quantile(times,.95)),'max':max(times)}}
    save(output/'report.json',report)
    lines=['# 完整8日上下文诊断对照','',
        '因输入复核发现遗漏第8日预测和其他日线路/来水因子，在任何留出读取前冻结此对照；原主模型保持不变。',
        '同一主设计4训练外生周、2留出外生周；含48个训练评价、24个测试候选（16个非零动作）。','',
        '| 训练独立周 | MAE MWh | 最大误差 MWh | ≥100 MWh收益漏判 | 方向漏判 | 平均后悔值 MWh |','|---|---:|---:|---:|---:|---:|']
    for r in results:lines.append(f"| {r['n_families']} | {r['mae_mwh']:.3f} | {r['max_error_mwh']:.3f} | {r['positive_misses']}/{r['positive_cases']} | {r['positive_direction_misses']} | {r['mean_regret_mwh']:.3f} |")
    lines+=['',f'研究数值门槛通过：{gate}；生产准入：否。',f'完整本地校验+特征+预测P95 {np.quantile(times,.95):.3f} ms，最大{max(times):.3f} ms。',
        '此处“完整输入”仅针对固定四边界和预注册自由变量，不能推广为任意网络的充分统计量；选择赢家仍需新的留出验证。']
    (output/'report.md').write_text('\n'.join(lines)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='predictions'}),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=('train','evaluate'),required=True)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-v3')
    p.add_argument('--output',type=Path,default=ROOT/'output/market-intelligence/hydro-complete-context-analysis-v1')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True);design=json.loads((args.input/'design.json').read_text())
    (train if args.phase=='train' else evaluate_saved)(args.input,args.output,design)


if __name__=='__main__':main()
