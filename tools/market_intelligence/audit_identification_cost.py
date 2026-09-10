#!/usr/bin/env python3
"""Descriptive Oracle stage-cost audit; no causal solver-performance claim."""
import argparse
import json
import math
from pathlib import Path
from statistics import mean

from build_market_label_dataset import read_json


def measure(folder, omit_repeats=False):
    records=[]
    for path in sorted(folder.glob('week-*/manifest.json')):
        manifest=json.loads(path.read_text())
        if omit_repeats and manifest.get('repeat_of') is not None:continue
        if manifest['status']!='completed':raise ValueError('Incomplete cost cohort')
        job=read_json(path.parent/'job.json.gz')
        days=job['scenarios'][0]['days']
        row={'index':manifest['index'],'worker_sec':manifest['wall_sec'],
             'operation_day_runtime_sec':sum(d['runtime_sec'] for d in days)}
        for stage in ('scuc','sced','lmp'):
            for field in ('runtime_sec','assembly_sec','audit_sec','solve_wall_sec','solution_export_sec'):
                values=[d['stages'][stage][field] for d in days]
                if not all(isinstance(x,(float,int)) and math.isfinite(x) and x>=0 for x in values):
                    raise ValueError('Invalid stage-cost metadata')
                row[stage+'_'+field]=sum(values)
        records.append(row)
    if not records:raise ValueError('Empty cohort')
    return {'n_evaluations':len(records),'mean_per_week':{key:mean(r[key] for r in records)
            for key in records[0] if key!='index'},'weeks':records}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,default=Path('output/market-intelligence/identification-v3'))
    p.add_argument('--reference',type=Path,default=Path('output/market-intelligence/stress-full-v2'))
    p.add_argument('--output',type=Path,default=Path('output/market-intelligence/identification-analysis-v3/cost-audit.json'))
    args=p.parse_args()
    current=measure(args.dataset,omit_repeats=True)
    reference=measure(args.reference)
    summary=json.loads((args.dataset/'oracle_summary.json').read_text())
    result={'scope':'posthoc descriptive cohorts, differing inputs; stage fields can overlap and must not be summed as independent costs',
            'prediction_sec':1500,'actual_wall_sec':summary['wall_sec'],
            'relative_prediction_error':summary['wall_sec']/1500-1,
            'current_nonrepeat':current,'v2_reference':reference}
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('current_nonrepeat','v2_reference')}))


if __name__=='__main__':main()
