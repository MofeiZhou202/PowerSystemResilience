#!/usr/bin/env python3
"""Frozen preflight-v3; market_surrogate_sampling_design.md section 9."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.stats import qmc

from build_market_label_dataset import FACTORS
from run_price_oracle import ROOT, digest, run_one, save

RANGES = np.array([[.7, 2.5], [.4, 1.6], [.5, 1.2], [.9, 1.1], [.15, 1.]])
N_FAMILIES = 12


def build_design(template):
    """Crossed shape/action experiment, fixed before Oracle labels (protocol section 9)."""
    unit = qmc.LatinHypercube(d=5, seed=20261100).random(N_FAMILIES)
    centers = qmc.scale(unit, RANGES[:, 0], RANGES[:, 1])
    rng = np.random.default_rng(20261101)
    specs = []
    for family, z in enumerate(centers):
        means = [z[0], z[1], z[1], z[2], z[3], z[3]]
        sequence = np.asarray([m * (1 + .1 * rng.permutation(np.arange(-3, 4)) / 3)
                               for m in means]).T
        for shape in (0, 1):
            profile = sequence if shape == 0 else sequence[::-1]
            for action in (0, 1):
                line = z[4] * (1 if action == 0 else .9)
                days = [{**dict(zip(FACTORS[:6], values.tolist())), 'line_limit_scale': float(line)}
                        for values in np.vstack([profile, means])]
                config = copy.deepcopy(template)
                config.update(sample_count=1, seed=20261100 + family, temporal_rho=0, mode='probabilistic')
                config['operation'].update(days=[], explain=False, horizon='week', recovery_pricing='full')
                config['correlation'] = np.eye(7).tolist()
                for marginal in config['marginals']:
                    values = [d[marginal['factor']] for d in days]
                    marginal.update(distribution='fixed', center=values, lower=min(values), upper=max(values), sigma=0)
                specs.append({'config': config, 'expected_days': days, 'metadata': {
                    'design': 'preflight-v3', 'family': family, 'shape': shape, 'action': action,
                    'repeat_of': None, 'split': 'development', 'centers': z.tolist(),
                    'probability_interpretation': 'synthetic identification design; no real-world weights'}})
    stressed = int(np.argmax(centers[:, 0] / centers[:, 1]))
    for original in (0, stressed * 4 + 1):
        repeat = copy.deepcopy(specs[original])
        repeat['metadata']['repeat_of'] = original
        specs.append(repeat)
    return {'protocol': 'preflight-v3', 'independent_families': N_FAMILIES,
            'specs': specs, 'predicted_wall_sec': 1500,
            'stopping_scope': 'frozen preflight only; no adaptive sampling'}


def validate_input(job, spec):
    """Reject silent changes to authored or reference inputs, including hidden modifiers."""
    if len(job['scenarios']) != 1:
        raise ValueError('Expected exactly one scenario')
    cfg = job['scenarios'][0]['config']
    for kind in ('days', 'reference_days'):
        if len(cfg[kind]) != 8:
            raise ValueError('Expected eight input days')
        for actual, expected in zip(cfg[kind], spec['expected_days']):
            if any(not np.isfinite(actual[f]) or abs(actual[f] - expected[f]) > 1e-12 for f in FACTORS):
                raise ValueError('Generated input differs from frozen design')
            if actual['bid_scale'] != 1 or actual['generator_outages'] or actual['branch_outages']:
                raise ValueError('Unexpected extra multiplier or outage')
            if actual['first_slot'] != 0 or actual['last_slot'] != 95 or actual.get('boundary_overrides'):
                raise ValueError('Unexpected slot window or override')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'output/market-intelligence/identification-v3')
    parser.add_argument('--input', type=Path, default=ROOT/'output/market-performance/live-input')
    parser.add_argument('--server', type=Path, default=ROOT/'build/macos-release/tests/run_gui_server')
    parser.add_argument('--workers', type=int, choices=(1, 2), default=2)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    template = json.loads((args.input/'market_forecast.json').read_text())['job']['config']
    design = build_design(template)
    design['source_sha256'] = hashlib.sha256((args.input/'southern_market.json').read_bytes()).hexdigest()
    design['binary_sha256'] = hashlib.sha256(args.server.read_bytes()).hexdigest()
    path = args.output/'design.json'
    if path.exists() and digest(json.loads(path.read_text())) != digest(design):
        raise ValueError('Refuse changing a frozen experiment')
    save(path, design)
    if args.prepare_only:
        print(json.dumps({'prepared_weeks': len(design['specs']), 'path': str(path)}))
        return
    args.recovery = False
    args.validate_input = validate_input
    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        runs = list(pool.map(lambda i: run_one(i, args, design['binary_sha256'], design['specs'][i]),
                             range(len(design['specs']))))
    summary = {'runs': runs, 'wall_sec': time.monotonic()-start,
               'workers': args.workers, 'design_sha256': digest(design)}
    save(args.output/'oracle_summary.json', summary)
    print(json.dumps({k: v for k, v in summary.items() if k != 'runs'}))
    if any(r['status'] != 'completed' or r.get('price_valid_days') != 7 or
           r.get('price_consistency_days') != 7 for r in runs):
        raise SystemExit('Incomplete or invalid weeks retained; inspect manifests')


if __name__ == '__main__':
    main()
