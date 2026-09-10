#!/usr/bin/env python3
"""Observed thermal-commitment coverage, not unique-UC claims; hydro theory §14.5."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from build_market_label_dataset import read_json
from hydro_allocation import ENERGY_TOL
from validate_hydro_temporal import close
from run_hydro_strict import GAPS, gap_dir
from run_price_oracle import ROOT, digest, save


def trace(job):
    ids=sorted(g['id'] for g in job['base']['generators'] if g['kind']=='thermal')
    if not ids:raise ValueError('Thermal units required for the strict research scope')
    initial=np.array([next(g['initial_on'] for g in job['base']['generators'] if g['id']==gid) for gid in ids])
    if not np.isin(initial,[0,1]).all():raise ValueError('Invalid authored initial commitment')
    days=job['scenarios'][0]['days']
    if len(days)!=7:raise ValueError('Seven complete days required for switching audit')
    values=[]
    for gid in ids:
        row=[]
        for day in days:
            g=next(g for g in day['resources']['generators'] if g['id']==gid)
            if len(g['online'])!=96:raise ValueError('Incomplete thermal commitment trajectory')
            row.extend(g['online'])
        values.append(row)
    authored=np.array(values)
    if not np.isfinite(authored).all() or np.max(np.abs(authored-np.rint(authored)))>1e-6:
        raise ValueError('Nonbinary or invalid thermal commitment')
    u=np.rint(authored).astype(int)
    if not np.isin(u,[0,1]).all():raise ValueError('Commitment outside binary range')
    changes=np.diff(np.c_[initial,u],axis=1)
    return ids,u,dict(startups=int(np.sum(changes==1)),shutdowns=int(np.sum(changes==-1)),
                      on_unit_slots=int(u.sum()))


def diagnostic_energy(job):
    """Directional limits and disabled-line leakage, hydro theory §12.8 / §14.5."""
    deficit = overload = 0.
    for day in job['scenarios'][0]['days']:
        for node in day['nodes']:
            values = np.asarray(node['deficit_mw'], dtype=float)
            if values.shape != (96,) or not np.isfinite(values).all() or np.min(values) < -1e-8:
                raise ValueError('Invalid nodal loss trajectory')
            deficit += float(values.sum()) * .25
        for line in day['lines']:
            values = np.asarray([line[k] for k in ('power_mw', 'min_mw', 'max_mw', 'available')], dtype=float)
            if values.shape != (4, 96) or not np.isfinite(values).all():
                raise ValueError('Invalid line trajectory')
            power, low, high, available = values
            if np.any(low > high) or not np.isin(available, [0, 1]).all():
                raise ValueError('Invalid directional line envelope')
            excess = np.where(available > .5, np.maximum(np.maximum(power-high, low-power), 0), np.abs(power))
            overload += float(excess.sum()) * .25
    return dict(deficit_mwh=deficit, overload_mwh=overload)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-strict-v4')
    args=p.parse_args();start=time.monotonic()
    design=json.loads((args.input/'design.json').read_text())
    audited=json.loads((args.input/'test-report.json').read_text())
    if audited['design_sha256']!=digest(design):raise ValueError('Coverage cohort differs')
    traces={};counts={};screens={};canonical_ids=None
    for gap in GAPS:
        for i,s in enumerate(design['specs']):
            job=read_json(gap_dir(args.input,gap)/f'week-{i:03d}/job.json.gz')
            ids,u,count=trace(job)
            if canonical_ids is None:canonical_ids=ids
            if ids!=canonical_ids:raise ValueError('Thermal stable ID mapping changed')
            traces[gap,i]=u;counts[gap,i]=count
            screens[gap,i]=diagnostic_energy(job)
            if gap == GAPS[-1]:
                for field,value in screens[gap,i].items():
                    close(value,audited['rows'][i][field],1e-6,'independent nodal/line energy')
    rows=[]
    for i,s in enumerate(design['specs']):
        base=next(j for j,b in enumerate(design['specs']) if b['family']==s['family'] and b['delta_mwh']==0)
        expected = all(v <= ENERGY_TOL for j in (i,base) for v in screens[GAPS[-1],j].values())
        if expected != audited['rows'][i]['pair_diagnostic_pass']:
            raise ValueError('Pair diagnostic admission disagrees with raw node/line traces')
        rows.append(dict(index=i,root_family=s['root_family'],split=s['split'],shape=s['shape'],condition=s['condition'],
            delta_mwh=s['delta_mwh'],regime=s['regime'],**counts[GAPS[1],i],
            paired_commitment_difference_unit_slots=int(np.sum(np.abs(traces[GAPS[1],i]-traces[GAPS[1],base]))),
            adjacent_gap_commitment_difference_unit_slots=int(np.sum(np.abs(traces[GAPS[1],i]-traces[GAPS[0],i]))),
            gain_mwh=audited['rows'][i]['gain_mwh'],stable=audited['rows'][i]['stable'],
            pair_diagnostic_pass=audited['rows'][i]['pair_diagnostic_pass']))
    def summary(split):
        active=[r for r in rows if r['split']==split and r['delta_mwh']!=0]
        switched=[r for r in active if r['paired_commitment_difference_unit_slots']>0]
        feasible=[r for r in switched if r['pair_diagnostic_pass'] and r['stable']]
        return dict(nonzero_pairs=len(active),observed_commitment_changing_pairs=len(switched),
                    stable_diagnostic_passing_commitment_changing_pairs=len(feasible),
                    independent_roots_with_passing_switches=sorted({r['root_family'] for r in feasible}))
    result=dict(rows=rows,train=summary('train'),test=summary('test'),wall_sec=time.monotonic()-start,
                predicted_wall_sec_upper=30,diagnostic_screen_recomputed_from_node_and_line_traces=True,
                screen_tolerance_mwh=ENERGY_TOL,unique_dispatch_certified=False,production_admitted=False)
    save(args.input/'analysis/switching-audit.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))


if __name__=='__main__':main()
