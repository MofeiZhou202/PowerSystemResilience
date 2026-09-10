#!/usr/bin/env python3
"""Frozen equal-energy shape × thermal-condition experiment; theory §12."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from hydro_allocation import synthetic_mapping, terminal_contract, compile_action
from hydro_physics import renewable_energy, daily_surplus
from hydro_temporal import variant_boundary
from run_hydro_oracle import run_one
from run_price_oracle import ROOT, digest, save


def build_design(boundary, template, binary_hash):
    mapping = synthetic_mapping(boundary)
    if len(mapping) != 12 or {s['id'] for s in mapping} != set(range(1, 13)):
        raise ValueError('Temporal protocol requires the twelve-station synthetic fixture')
    boundaries = {f'{s}-{c}': variant_boundary(boundary, s, c) for s in (0, 1) for c in (0, 1)}
    specs = []
    generators = {'train': np.random.default_rng(20261410), 'test': np.random.default_rng(20261411)}
    demand_energy = sum(sum(a['load_mw'][:96])*.25 for a in boundary['areas'])
    for root in range(6):
        split = 'train' if root < 4 else 'test'
        rng = generators[split]
        external = []
        for _ in range(8):
            external.append({'load_scale':float(rng.uniform(.6,.85)), 'wind_scale':float(rng.uniform(1,1.8)),
                'solar_scale':float(rng.uniform(1,1.8)), 'inflow_scale':float(rng.uniform(.6,1)),
                'line_limit_scale':float(rng.uniform(.4,1)), 'generator_bid_scale':1., 'load_bid_scale':1.,
                'bid_scale':1., 'first_slot':0, 'last_slot':95, 'generator_outages':[], 'branch_outages':[]})
        targets = []
        for d in (1, 4):
            target = float(rng.uniform(-180,180))
            external[d]['load_scale'] = (36000+renewable_energy(boundary,external[d])-target)/demand_energy
            targets.append(target)
        quotas = {str(s['id']):[3000.]*7 for s in mapping}
        weekly = {sid:21000. for sid in quotas}
        terminal = terminal_contract(boundary,mapping,external,quotas)
        for shape in (0, 1):
            for condition in (0, 1):
                key = f'{shape}-{condition}'
                b = boundaries[key]
                for delta in (0., -300., 300.):
                    action = copy.deepcopy(quotas)
                    action['1'][1] -= delta
                    action['1'][4] += delta
                    config = copy.deepcopy(template)
                    config.update(explain=False,explain_trigger='always',recovery_pricing='full',horizon='week',
                        terminal_forecast_source='authored',reference_days=copy.deepcopy(external))
                    config['solver_options'].update(solver='gurobi',threads=2,time_limit_sec=120,mip_gap=.01)
                    config['days'] = compile_action(b,mapping,external,action,weekly,terminal)
                    spec = {'root_family':root, 'family':root*4+shape*2+condition, 'split':split,
                        'shape':shape,'condition':condition,'boundary_key':key,'boundary_sha256':digest(b),
                        'delta_mwh':delta,'config':config,'quotas':action,'weekly':weekly,'terminal':terminal,
                        'external_days':external,'target_r2_r5_mwh':targets}
                    if np.max(np.abs(daily_surplus(b,spec)[[1,4]]-targets)) > 1e-6:
                        raise ValueError('Equal daily energy construction failed')
                    specs.append(spec)
    return {'protocol':'hydro-temporal-v3', 'source_sha256':digest(boundary),'binary_sha256':binary_hash,
        'mapping':mapping,'boundaries':boundaries,'specs':specs,'planned_evaluations':72,
        'independent_families':6,'predicted_wall_sec':6000,
        'split':'root 0..3 train, 4..5 test; every shape/condition/action sibling kept together',
        'research_gate':{'mae_mwh':25,'max_error_mwh':50,'meaningful_positive_gain_mwh':100,
                         'positive_gain_misses':0,'full_inference_p95_ms':1000},
        'scope':'synthetic equal-energy factorial stress study; rolling linear diagnostic, no production admission'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-performance/live-input')
    p.add_argument('--output',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-v3')
    p.add_argument('--server',type=Path,default=ROOT/'build/macos-release/tests/run_gui_server')
    p.add_argument('--workers',type=int,choices=(1,2),default=2)
    p.add_argument('--limit',type=int,default=72)
    p.add_argument('--prepare-only',action='store_true')
    args = p.parse_args()
    if not 1 <= args.limit <= 72:
        p.error('limit must be 1..72')
    args.output.mkdir(parents=True,exist_ok=True)
    b = json.loads((args.input/'southern_market.json').read_text())['boundary']
    template = json.loads((args.input/'market_forecast.json').read_text())['job']['config']['operation']
    design = build_design(b,template,hashlib.sha256(args.server.read_bytes()).hexdigest())
    path = args.output/'design.json'
    if path.exists() and digest(json.loads(path.read_text())) != digest(design):
        raise ValueError('Refuse changing frozen temporal design')
    save(path,design)
    if args.prepare_only:
        print(json.dumps({'planned':72,'design_sha256':digest(design)}))
        return
    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        runs = list(pool.map(lambda i:run_one(i,args,design),range(args.limit)))
    summary = {'runs':runs,'wall_sec':time.monotonic()-start,'workers':args.workers,
        'worker_sum_sec':sum(r['wall_sec'] for r in runs),'design_sha256':digest(design)}
    save(args.output/f'invocation-{args.limit:02d}.json',summary)
    if args.limit == 72:
        save(args.output/'oracle_summary.json',summary)
    if any(r['status'] != 'completed' or not r.get('train_eligible') for r in runs):
        raise SystemExit('Ineligible temporal cases retained; inspect evidence')
    print(json.dumps({k:v for k,v in summary.items() if k != 'runs'}))


if __name__=='__main__':
    main()
