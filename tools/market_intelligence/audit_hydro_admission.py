#!/usr/bin/env python3
"""Keep numerical label eligibility separate from flow feasibility; theory §12.8."""
import json

from hydro_allocation import ENERGY_TOL
from run_price_oracle import ROOT,save


def main():
    root=ROOT/'output/market-intelligence/hydro-temporal-analysis-v3'
    report=json.loads((root/'report.json').read_text());pairs=report['pairs']
    passed={p['index']:p['deficit_mwh']<=ENERGY_TOL and p['overload_mwh']<=ENERGY_TOL for p in pairs}
    records=[]
    for p in pairs:
        base=next(b for b in pairs if b['family']==p['family'] and b['delta_mwh']==0)
        records.append({'index':p['index'],'family':p['family'],'split':'train' if p['index']<48 else 'test',
            'delta_mwh':p['delta_mwh'],'gain_mwh':p['gain_mwh'],
            'numerical_label_eligible':True,'deficit_mwh':p['deficit_mwh'],'overload_integral_mwh':p['overload_mwh'],
            'diagnostic_balance_and_flow_pass':passed[p['index']],
            'both_candidate_and_baseline_pass':passed[p['index']] and passed[base['index']],
            'physical_security_certification':'not_performed'})
    selections=[]
    for family in sorted({p['family'] for p in pairs if p['index']>=48}):
        options=[p for p in pairs if p['family']==family and passed[p['index']]]
        selections.append({'family':family,'passing_candidates':len(options),
            'best_diagnostic_passing_gain_mwh':max(p['gain_mwh'] for p in options) if options else None,
            'status':'passing diagnostic candidates exist; AC/N-1 untested' if options else 'no passing diagnostic candidate'})
    result={'tolerance_mwh':ENERGY_TOL,'records':records,'test_candidate_sets':selections,
        'flow_or_balance_failures':sum(not p['diagnostic_balance_and_flow_pass'] for p in records),
        'test_failures':sum(not p['diagnostic_balance_and_flow_pass'] for p in records if p['split']=='test'),
        'test_positive_gains':sum(p['gain_mwh']>ENERGY_TOL for p in records if p['split']=='test'),
        'test_positive_gains_passing_screen':sum(p['gain_mwh']>ENERGY_TOL and p['diagnostic_balance_and_flow_pass'] for p in records if p['split']=='test'),
        'production_admitted':False,'original_test_metrics_or_models_modified':False}
    save(root/'admission-audit.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('records','test_candidate_sets')}))


if __name__=='__main__':main()
