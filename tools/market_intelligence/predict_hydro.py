#!/usr/bin/env python3
"""Local research inference; fixed supported action and explicit Oracle verification."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from build_market_label_dataset import FACTORS
from hydro_allocation import compile_action, synthetic_mapping, terminal_contract
from hydro_physics import feature, PhysicsGP
from run_price_oracle import ROOT, digest


def prepare_candidate(boundary, spec, delta):
    # Scope in hydro_renewable_allocation.md §10: no new stations, bids, states or outages.
    if not np.isfinite(delta) or abs(delta) > 300:
        raise ValueError('Supported day2-to-day5 transfer is -300..300 MWh')
    if spec['weekly'] != {str(r): 21000. for r in range(1, 13)}:
        raise ValueError('Requires fixed original 12 station weekly quotas')
    days = spec['external_days']
    if len(days) != 8:
        raise ValueError('Eight complete external/reference days required')
    for d, day in enumerate(days):
        if any(not isinstance(day.get(k), (int, float)) or isinstance(day[k], bool) or not np.isfinite(day[k]) or not 0 <= day[k] <= 10 for k in FACTORS):
            raise ValueError('Invalid or missing external factor')
        if (day.get('boundary_overrides') or day.get('generator_outages') or day.get('branch_outages')
                or any(day.get(k) != 1 for k in ('bid_scale','generator_bid_scale','load_bid_scale'))
                or day.get('first_slot') != 0 or day.get('last_slot') != 95):
            raise ValueError('Unsupported overrides, outages, bids or time window')
        for field, lo, hi in (('wind_scale',1.,1.8), ('solar_scale',1.,1.8),
                              ('inflow_scale',.6,1.), ('line_limit_scale',.4,1.)):
            if not lo <= day[field] <= hi:
                raise ValueError('External context outside frozen transition study support')
        if d not in (1,4) and not .60 <= day['load_scale'] <= .85:
            raise ValueError('Unchanged-day load outside frozen transition support')
    mapping = synthetic_mapping(boundary)
    quotas = {str(s['id']): [3000.]*7 for s in mapping}
    terminal = terminal_contract(boundary, mapping, days, quotas)
    if digest(terminal) != digest(spec['terminal']):
        raise ValueError('Different terminal resource contract requires new Oracle study')
    candidate = copy.deepcopy(spec)
    quotas['1'][1] -= delta
    quotas['1'][4] += delta
    candidate.update(delta_mwh=delta, quotas=quotas)
    candidate['config']['days'] = compile_action(boundary, mapping, days, quotas, spec['weekly'], terminal)
    candidate['config']['reference_days'] = copy.deepcopy(days)
    coordinates = feature(boundary, candidate)
    if np.any(np.abs(coordinates[:2]) > 2.+1e-10):
        raise ValueError('Day2/day5 surplus outside studied -600..600 MWh interval')
    return candidate


def predict(model_folder, spec_file, deltas):
    if not deltas:
        raise ValueError('At least one candidate transfer required')
    metadata = json.loads((model_folder/'model-freeze.json').read_text())
    boundary = json.loads((model_folder/'boundary.json').read_text())
    authored = json.loads(spec_file.read_text())
    if digest(boundary) != metadata['base_sha256']:
        raise ValueError('Model base hash mismatch')
    if 'base' in authored and digest(authored['base']) != metadata['base_sha256']:
        raise ValueError('Input uses a different case or initial state')
    spec = authored.get('spec', authored)
    operation = {k:v for k,v in spec['config'].items() if k not in ('days','reference_days')}
    if operation != metadata['operation_contract']:
        raise ValueError('Different market penalty, solver or operation contract requires Oracle revalidation')
    model_path = model_folder/'gp-8.npz'
    if hashlib.sha256(model_path.read_bytes()).hexdigest() != metadata['fits'][-1]['gp_sha256']:
        raise ValueError('Frozen GP model hash mismatch')
    gp = PhysicsGP.load(model_path)
    candidates = [prepare_candidate(boundary, spec, delta) for delta in deltas]
    x = np.array([feature(boundary, candidate) for candidate in candidates])
    mu, sigma = gp.predict(x, return_std=True)
    lo, hi = np.array(metadata['feature_min']), np.array(metadata['feature_max'])
    rows = [{'delta_mwh': delta, 'estimated_gain_mwh': float(mean),
             'model_std_mwh': float(std),
             'inside_training_coordinate_box': bool(np.all((point >= lo) & (point <= hi))),
             'needs_oracle_confirmation': True}
            for delta, mean, std, point in zip(deltas, mu, sigma, x)]
    selected = max(rows, key=lambda row: (row['estimated_gain_mwh'], row['delta_mwh'] == 0))
    return {'schema': 'hydro-research-inference-v2', 'model_sha256': metadata['fits'][-1]['gp_sha256'],
            'scope': metadata['scope'], 'predictions': rows, 'candidate_for_oracle_delta_mwh': selected['delta_mwh'],
            'production_admitted': False, 'uncertainty': metadata['std_interpretation'],
            'support_warning': 'A coordinate box check is necessary context only; it is not a feasibility or accuracy certificate'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path, default=ROOT/'output/market-intelligence/hydro-transition-analysis-v2')
    p.add_argument('--spec', type=Path, required=True)
    p.add_argument('--deltas', type=float, nargs='+', default=[-300., 0., 300.])
    args = p.parse_args()
    print(json.dumps(predict(args.model, args.spec, args.deltas), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
