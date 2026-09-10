#!/usr/bin/env python3
"""Independent target/metric reconstruction for the full-context comparator."""
import hashlib
import json

from run_price_oracle import ROOT,digest,save
from validate_hydro_temporal import close


def main():
    parent=ROOT/'output/market-intelligence';root=parent/'hydro-temporal-v3';output=parent/'hydro-complete-context-analysis-v1'
    design=json.loads((root/'design.json').read_text());report=json.loads((output/'report.json').read_text())
    frozen=json.loads((output/'model-freeze.json').read_text());baseline=json.loads((parent/'hydro-temporal-analysis-v3/report.json').read_text())
    raw=json.loads((parent/'hydro-temporal-analysis-v3/raw-execution-audit.json').read_text())
    if digest(design)!=report['design_sha256'] or report['design_sha256']!=frozen['design_sha256']:
        raise ValueError('Full-context design identity mismatch')
    if frozen['main_test_report_existed_at_freeze']:raise ValueError('Model was frozen after test report')
    cur={int(i):sum(day['curtailment_mwh'] for day in days) for i,days in raw['daily_energy'].items()}
    max_error=0.
    for fit in frozen['fits']:
        path=output/f"complete_gp-{fit['n_families']}.npz"
        if hashlib.sha256(path.read_bytes()).hexdigest()!=fit['artifact_hashes']['complete_gp']:raise ValueError('Model changed')
        counterpart=next(f for f in baseline['fits'] if f['n_families']==fit['n_families'])
        if fit['training_labels_sha256']!=counterpart['training_labels_sha256']:raise ValueError('Comparators trained on different labels')
        close(fit['oracle_worker_sec'],counterpart['oracle_worker_sec'],1e-8,'same training cost')
        for r in [p for p in report['predictions'] if p['n_families']==fit['n_families']]:
            spec=design['specs'][r['index']]
            if spec['root_family'] not in (4,5) or spec['root_family'] in r['train_root_families']:raise ValueError('Sibling leakage')
            base=next(i for i,s in enumerate(design['specs']) if s['family']==spec['family'] and s['delta_mwh']==0)
            max_error=max(max_error,close(cur[base]-cur[r['index']],r['actual_gain_mwh'],1e-8,'raw complete-context target'))
    for score in report['results']:
        rows=[r for r in report['predictions'] if r['n_families']==score['n_families']]
        if len(rows)!=24:raise ValueError('Wrong complete-context test count')
        active=[r for r in rows if r['delta_mwh']!=0];err=[abs(r['actual_gain_mwh']-r['predicted_gain_mwh']) for r in active]
        max_error=max(max_error,close(sum(err)/16,score['mae_mwh'],1e-8,'MAE'),close(max(err),score['max_error_mwh'],1e-8,'maximum error'))
        positive=[r for r in active if r['actual_gain_mwh']>=100]
        if len(positive)!=score['positive_cases'] or sum(r['predicted_gain_mwh']<100 for r in positive)!=score['positive_misses']:raise ValueError('Recall mismatch')
        for choice in score['selection']:
            options=[r for r in rows if r['family']==choice['family']]
            selected=max(options,key=lambda r:(r['predicted_gain_mwh'],r['delta_mwh']==0))
            if selected['delta_mwh']!=choice['selected_delta_mwh']:raise ValueError('Selection used hindsight')
            close(max(r['actual_gain_mwh'] for r in options)-selected['actual_gain_mwh'],choice['regret_mwh'],1e-8,'decision regret')
    current=report['results'][-1];reference=next(r for r in baseline['results'] if r['n_families']==4 and r['model']=='network_gp')
    improvement=1-current['mae_mwh']/reference['mae_mwh'] if reference['mae_mwh']>1e-9 else None
    result={'passed':True,'prediction_rows':48,'independent_test_roots':[4,5],
        'maximum_reconstruction_error_mwh':max_error,'same_training_labels_verified':True,
        'improvement_vs_network_gp':improvement,'predicted_improvement_vs_network_gp':.1,
        'production_admitted':False}
    save(output/'independent-validation.json',result);print(json.dumps(result))


if __name__=='__main__':main()
