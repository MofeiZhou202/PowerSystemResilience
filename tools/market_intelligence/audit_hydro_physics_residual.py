#!/usr/bin/env python3
"""Posthoc accounting of physical-surrogate mismatch; not model selection."""
import json
from pathlib import Path

from build_market_label_dataset import read_json
from hydro_physics import daily_surplus, physics_gain, feature
from run_price_oracle import ROOT, save
from validate_hydro import close


def main():
    root=ROOT/'output/market-intelligence/hydro-transition-v2'
    output=ROOT/'output/market-intelligence/hydro-transition-analysis-v2'
    design=json.loads((root/'design.json').read_text())
    report=json.loads((output/'report.json').read_text())
    boundary=json.loads((root/'week-000/input.json').read_text())['base']
    cases=[]
    for pair in report['pairs']:
        spec=design['specs'][pair['index']]
        predicted=float(physics_gain(feature(boundary,spec))[0])
        residual=pair['gain_mwh']-predicted
        if abs(residual)<=1e-3:
            continue
        baseline=next(i for i,s in enumerate(design['specs']) if s['family']==spec['family'] and s['delta_mwh']==0)
        records=[]
        for index in (baseline,pair['index']):
            job=read_json(root/f'week-{index:03d}/job.json.gz')
            for d in (1,4):
                day=job['scenarios'][0]['days'][d]
                thermal=sum(sum(g['power_mw'])*.25 for g in day['resources']['generators'] if g['kind']=='thermal')
                renewable=[g for g in day['resources']['generators'] if g['kind'] in ('wind','solar','renewable')]
                cur=sum(sum(g['curtailment_mw'])*.25 for g in renewable)
                # Raw charge_mw is negative injection; hydro theory §12.3.
                storage=-sum((sum(s['charge_mw'])+sum(s['discharge_mw']))*.25 for s in day['resources']['storage'])
                hydro=sum(design['specs'][index]['quotas'][str(s['id'])][d] for s in design['mapping'])
                av=sum(sum(g['renewable_available_mw'])*.25 for g in renewable)
                load=sum(sum(a['load_mw'][:96])*.25 for a in boundary['areas'])*spec['external_days'][d]['load_scale']
                rhs=hydro+av+thermal-load-storage+day['deficit_mwh']-day['surplus_mwh']
                err=close(cur,rhs,1e-6,'full daily energy balance with thermal and storage')
                records.append({'index':index,'day_1based':d+1,'thermal_mwh':thermal,
                    'storage_net_consumption_mwh':storage,'curtailment_mwh':cur,'full_balance_error_mwh':err,
                    'scuc_gap':day['stages']['scuc']['mip_gap'],'surplus_mwh':day['surplus_mwh']})
        cases.append({'index':pair['index'],'family':pair['family'],'split':spec['split'],
            'actual_gain_mwh':pair['gain_mwh'],'physical_gain_mwh':predicted,'residual_mwh':residual,
            'records':records})
    result={'cases':cases,'interpretation':'Aggregate relaxation omits realized thermal generation under hourly/network/UC/cost constraints. Energy accounting explains the residual; the specific binding constraint causing thermal output was not isolated. Finite SCUC gaps remain explicit. No test retuning.',
            'model_changed_after_test':False}
    save(output/'physics-residual-audit.json',result)
    print(json.dumps(result))


if __name__=='__main__':
    main()
