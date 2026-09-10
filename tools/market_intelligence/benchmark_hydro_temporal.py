#!/usr/bin/env python3
"""Saved-model and full validated local-request benchmark for hydro temporal v3."""
import copy
import json
import time

import numpy as np

from predict_hydro_temporal import ResearchPredictor
from run_price_oracle import ROOT,save


def main():
    root=ROOT/'output/market-intelligence/hydro-temporal-v3'
    output=ROOT/'output/market-intelligence/hydro-temporal-analysis-v3'
    predictor=ResearchPredictor(output,root/'design.json')
    report=json.loads((output/'report.json').read_text())
    rows=[r for r in report['predictions'] if r['model']=='temporal_gp' and r['n_families']==4]
    checks=[]
    for row in rows:
        payload=json.loads((root/f"week-{row['index']:03d}/input.json").read_text())
        result=predictor.predict(payload['base'],payload['spec'])
        error=abs(result['predicted_gain_mwh']-row['predicted_gain_mwh'])
        if error>1e-8:raise ValueError('Reloaded local inference disagrees with frozen test output')
        checks.append({'index':row['index'],'reload_error_mwh':error,**result})
    payload=json.loads((root/'week-048/input.json').read_text())
    rejected=[]
    for mutation in ('quota','base','outage','delta'):
        candidate=copy.deepcopy(payload)
        if mutation=='quota':candidate['spec']['weekly']['1']=22000
        if mutation=='base':candidate['base']['buses'][0]['load_mw'][0]+=1
        if mutation=='outage':candidate['spec']['external_days'][0]['generator_outages']=[1]
        if mutation=='delta':candidate['spec']['delta_mwh']=301
        try:predictor.predict(candidate['base'],candidate['spec'])
        except ValueError as exc:rejected.append({'mutation':mutation,'reason':str(exc)})
        else:raise ValueError('Unsupported input was accepted: '+mutation)
    latencies=[]
    for _ in range(100):
        start=time.perf_counter_ns();predictor.predict(payload['base'],payload['spec'])
        latencies.append((time.perf_counter_ns()-start)/1e6)
    result={'passed':True,'saved_candidate_predictions':checks,'rejected_inputs':rejected,
        'request_count':100,'full_validated_request_ms':{'median':float(np.median(latencies)),
            'p95':float(np.quantile(latencies,.95)),'max':max(latencies)},
        'under_one_second_all_requests':max(latencies)<=1000,
        'scope':'Preloaded model/design, full in-memory candidate validation + input features + prediction; no disk loading/HTTP concurrency/Oracle time'}
    secondary=ResearchPredictor(output,root/'design.json','network_gp')
    secondary_errors=[]
    for row in report['predictions']:
        if row['model']!='network_gp' or row['n_families']!=4:continue
        candidate=json.loads((root/f"week-{row['index']:03d}/input.json").read_text())
        value=secondary.predict(candidate['base'],candidate['spec'])['predicted_gain_mwh']
        secondary_errors.append(abs(value-row['predicted_gain_mwh']))
    if len(secondary_errors)!=24 or max(secondary_errors)>1e-8:
        raise ValueError('Saved secondary model differs from frozen test predictions')
    secondary_times=[]
    for _ in range(100):
        start=time.perf_counter_ns();secondary.predict(payload['base'],payload['spec'])
        secondary_times.append((time.perf_counter_ns()-start)/1e6)
    result['secondary_network_gp']={'reload_max_error_mwh':max(secondary_errors),
        'full_validated_request_ms':{'median':float(np.median(secondary_times)),
            'p95':float(np.quantile(secondary_times,.95)),'max':max(secondary_times)},
        'under_one_second_all_requests':max(secondary_times)<=1000}
    save(output/'inference-smoke.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='saved_candidate_predictions'}))


if __name__=='__main__':main()
