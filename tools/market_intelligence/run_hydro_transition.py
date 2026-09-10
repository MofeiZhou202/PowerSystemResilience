#!/usr/bin/env python3
"""Fixed independent transition cohort; hydro_renewable_allocation.md §10."""
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
from run_hydro_oracle import run_one
from run_price_oracle import ROOT, digest, save


def build_design(boundary, template, binary_hash):
    mapping = synthetic_mapping(boundary)
    if len(mapping) != 12 or {s['id'] for s in mapping} != set(range(1, 13)):
        raise ValueError('Transition protocol requires the original 12-station synthetic fixture')
    generators = {'train': np.random.default_rng(20261310), 'test': np.random.default_rng(20261311)}
    regions = [((-600, -350), (-600, -350)), ((-600, -350), (350, 600)),
               ((350, 600), (-600, -350)), ((-150, 150), (-150, 150))]
    specs = []
    base_load = sum(sum(a['load_mw'][:96])*.25 for a in boundary['areas'])
    for region, ranges in enumerate(regions):
        for member in range(3):
            split = 'train' if member < 2 else 'test'
            rng = generators[split]
            external = []
            for _ in range(8):
                external.append({'load_scale': float(rng.uniform(.60, .85)), 'wind_scale': float(rng.uniform(1, 1.8)),
                    'solar_scale': float(rng.uniform(1, 1.8)), 'inflow_scale': float(rng.uniform(.6, 1)),
                    'line_limit_scale': float(rng.uniform(.4, 1)), 'generator_bid_scale': 1., 'load_bid_scale': 1.,
                    'bid_scale': 1., 'first_slot': 0, 'last_slot': 95, 'generator_outages': [], 'branch_outages': []})
            target_r = []
            for day, interval in zip((1, 4), ranges):
                target = float(rng.uniform(*interval))
                external[day]['load_scale'] = (36000+renewable_energy(boundary, external[day])-target)/base_load
                target_r.append(target)
            quotas = {str(s['id']): [3000.]*7 for s in mapping}
            weekly = {sid: 21000. for sid in quotas}
            terminal = terminal_contract(boundary, mapping, external, quotas)
            for delta in (0., -300., 300.):
                action = copy.deepcopy(quotas)
                action['1'][1] -= delta
                action['1'][4] += delta
                config = copy.deepcopy(template)
                config.update(explain=False, explain_trigger='always', recovery_pricing='full', horizon='week',
                              terminal_forecast_source='authored', reference_days=copy.deepcopy(external))
                config['solver_options'].update(solver='gurobi', threads=2, time_limit_sec=120, mip_gap=.01)
                config['days'] = compile_action(boundary, mapping, external, action, weekly, terminal)
                spec = {'family': region*3+member, 'region': region, 'split': split,
                        'delta_mwh': delta, 'config': config, 'quotas': action, 'weekly': weekly,
                        'terminal': terminal, 'external_days': external, 'target_r2_r5_mwh': target_r}
                actual = daily_surplus(boundary, spec)[[1, 4]]
                if np.max(np.abs(actual-target_r)) > 1e-6:
                    raise ValueError('Surplus-coordinate generation mismatch')
                specs.append(spec)
    return {'protocol': 'hydro-transition-v2', 'mapping': mapping, 'specs': specs,
            'source_sha256': digest(boundary), 'binary_sha256': binary_hash,
            'predicted_wall_sec': 2200, 'independent_families': 12, 'planned_evaluations': 36,
            'split': '8 training families, 4 fixed independent test families, no test tuning',
            'scope': 'synthetic stratified conditional scenario experiment; no real-world probability weights'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-performance/live-input')
    p.add_argument('--output', type=Path, default=ROOT/'output/market-intelligence/hydro-transition-v2')
    p.add_argument('--server', type=Path, default=ROOT/'build/macos-release/tests/run_gui_server')
    p.add_argument('--workers', type=int, choices=(1, 2), default=2)
    p.add_argument('--prepare-only', action='store_true')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    b = json.loads((args.input/'southern_market.json').read_text())['boundary']
    template = json.loads((args.input/'market_forecast.json').read_text())['job']['config']['operation']
    design = build_design(b, template, hashlib.sha256(args.server.read_bytes()).hexdigest())
    path = args.output/'design.json'
    if path.exists() and digest(json.loads(path.read_text())) != digest(design):
        raise ValueError('Refuse changing frozen transition design')
    save(path, design)
    if args.prepare_only:
        print(json.dumps({'planned': 36, 'design_sha256': digest(design)}))
        return
    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        runs = list(pool.map(lambda i: run_one(i, args, design), range(36)))
    summary = {'runs': runs, 'wall_sec': time.monotonic()-start, 'workers': args.workers,
               'worker_sum_sec': sum(r['wall_sec'] for r in runs), 'design_sha256': digest(design)}
    save(args.output/'oracle_summary.json', summary)
    if any(r['status'] != 'completed' or not r.get('train_eligible') for r in runs):
        raise SystemExit('Ineligible transition cases retained; inspect raw evidence')
    print(json.dumps({k: v for k, v in summary.items() if k != 'runs'}))


if __name__ == '__main__':
    main()
