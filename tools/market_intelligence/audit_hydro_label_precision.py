#!/usr/bin/env python3
"""Training-only tighter-gap paired replay; hydro theory §12.6."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time

from build_market_label_dataset import read_json
from run_hydro_oracle import run_one
from run_price_oracle import ROOT,digest,save


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-performance/live-input')
    p.add_argument('--output',type=Path,default=ROOT/'output/market-intelligence/hydro-label-precision-v1')
    p.add_argument('--source',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-v3')
    p.add_argument('--server',type=Path,default=ROOT/'build/macos-release/tests/run_gui_server')
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--gap',type=float,choices=(1e-4,1e-5),default=1e-4)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    original=json.loads((args.source/'design.json').read_text());design=copy.deepcopy(original)
    design.update(protocol='hydro-temporal-label-precision-v1' if args.gap==1e-4 else 'hydro-temporal-label-precision-gap-1e-5',planned_evaluations=2,independent_families=1,
        predicted_wall_sec=350,source_indices=[3,4],source_design_sha256=digest(original),
        scope='Training-only paired solver-gap sensitivity; not new test validation')
    design['specs']=[copy.deepcopy(original['specs'][i]) for i in (3,4)]
    for s in design['specs']:
        if s['split']!='train':raise ValueError('Precision audit must not select test labels')
        s['config']['solver_options']['mip_gap']=args.gap
    if hashlib.sha256(args.server.read_bytes()).hexdigest()!=design['binary_sha256']:
        raise ValueError('Oracle binary changed')
    path=args.output/'design.json'
    if path.exists() and digest(json.loads(path.read_text()))!=digest(design):raise ValueError('Precision protocol changed')
    save(path,design)
    if args.prepare_only:
        print(json.dumps({'planned':2,'source_indices':[3,4],'design_sha256':digest(design)}));return
    started=time.monotonic()
    with ThreadPoolExecutor(max_workers=2) as pool:runs=list(pool.map(lambda i:run_one(i,args,design),range(2)))
    summary={'runs':runs,'wall_sec':time.monotonic()-started,'worker_sum_sec':sum(r['wall_sec'] for r in runs)}
    save(args.output/'oracle_summary.json',summary)
    labels=[json.loads((args.output/f'week-{i:03d}/label.json').read_text()) if (args.output/f'week-{i:03d}/label.json').exists() else None for i in range(2)]
    accepted=all(l and l['train_eligible'] for l in labels)
    prior=[json.loads((args.source/f'week-{i:03d}/label.json').read_text()) for i in (3,4)]
    old_gain=prior[0]['metrics']['renewable_curtailment_mwh']-prior[1]['metrics']['renewable_curtailment_mwh']
    new_gain=labels[0]['metrics']['renewable_curtailment_mwh']-labels[1]['metrics']['renewable_curtailment_mwh'] if accepted else None
    result={'accepted_tighter_labels':bool(accepted),'source_indices':[3,4],'old_gain_mwh':old_gain,
        'new_gain_mwh':new_gain,'absolute_gain_change_mwh':abs(new_gain-old_gain) if accepted else None,
        'stable_within_25_mwh':abs(new_gain-old_gain)<=25 if accepted else None,
        'requested_old_gap':.01,'requested_new_gap':args.gap,'wall_sec':summary['wall_sec'],
        'worker_sum_sec':summary['worker_sum_sec'],'predicted_wall_sec':350,
        'source_label_issues':[l['quality']['issues'] if l else ['No label produced'] for l in labels],
        'production_precision_certified':False,'original_model_or_test_labels_modified':False}
    save(args.output/'report.json',result);print(json.dumps(result))


if __name__=='__main__':main()
