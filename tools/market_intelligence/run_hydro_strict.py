#!/usr/bin/env python3
"""Frozen two-gap labeling with training-before-test guard; hydro theory §14."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from build_market_label_dataset import read_json
from hydro_allocation import synthetic_mapping, terminal_contract, compile_action, ENERGY_TOL
from hydro_physics import renewable_energy, daily_surplus
from hydro_temporal import variant_boundary
from run_hydro_oracle import run_one
from run_price_oracle import ROOT, digest, save
from validate_hydro_temporal import audit_raw, close

GAPS = (1e-5, 1e-6)
STABILITY_MWH = 5.0  # Necessary diagnostic, not a true-error bound; theory §14.1.
REPRESENTATIVES = ((3, 4), (21, 22), (24, 26), (42, 44))


def gap_dir(root, gap):
    if gap not in GAPS:
        raise ValueError('Unsupported frozen precision level')
    return Path(root) / ('gap-1e-5' if gap == GAPS[0] else 'gap-1e-6')


def frozen_save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if digest(json.loads(path.read_text())) != digest(value):
            raise ValueError('Refuse changing frozen artifact: ' + str(path))
    else:
        save(path, value)


def precision_design(original):
    design = copy.deepcopy(original)
    indices = [i for pair in REPRESENTATIVES for i in pair]
    design['specs'] = [copy.deepcopy(original['specs'][i]) for i in indices]
    for i, spec in zip(indices, design['specs']):
        if spec['split'] != 'train':
            raise ValueError('Convergence representatives must be training-only')
        spec['source_index'] = i
    design.update(protocol='hydro-strict-convergence-v1', source_design_sha256=digest(original),
                  source_indices=indices, independent_families=4, planned_evaluations=16,
                  predicted_wall_sec=1200, stability_tolerance_mwh=STABILITY_MWH,
                  scope='Four training pairs; adjacent-gap stability, no truth or production certificate')
    return design


def build_design(boundary, template, binary_hash):
    """New high-benefit / hinge-switch contexts, fixed before labels; §14.2."""
    mapping = synthetic_mapping(boundary)
    if {s['id'] for s in mapping} != set(range(1, 13)):
        raise ValueError('Strict protocol requires the twelve-station synthetic fixture')
    boundaries = {f'{s}-{c}': variant_boundary(boundary, s, c) for s in (0, 1) for c in (0, 1)}
    rngs = {'train': np.random.default_rng(20261510), 'test': np.random.default_rng(20261511)}
    load = sum(sum(a['load_mw'][:96]) * .25 for a in boundary['areas'])
    specs = []
    for root in range(6):
        split = 'train' if root < 4 else 'test'
        rng = rngs[split]
        external = []
        for _ in range(8):
            external.append(dict(load_scale=float(rng.uniform(.6, .85)), wind_scale=float(rng.uniform(1, 1.8)),
                solar_scale=float(rng.uniform(1, 1.8)), inflow_scale=float(rng.uniform(.6, 1)),
                line_limit_scale=float(rng.uniform(.8, 1)), generator_bid_scale=1., load_bid_scale=1.,
                bid_scale=1., first_slot=0, last_slot=95, generator_outages=[], branch_outages=[]))
        if root in (0, 1, 4):
            regime, ranges = 'high-benefit', ((220, 360), (-360, -220))
        elif root == 3:
            regime, ranges = 'reverse', ((-180, -40), (40, 180))
        else:
            regime, ranges = 'switch', ((40, 180), (-180, -40))
        targets = [float(rng.uniform(*bounds)) for bounds in ranges]
        for d, target in zip((1, 4), targets):
            external[d]['load_scale'] = (36000 + renewable_energy(boundary, external[d]) - target) / load
        baseline = {str(s['id']): [3000.] * 7 for s in mapping}
        weekly = {sid: 21000. for sid in baseline}
        terminal = terminal_contract(boundary, mapping, external, baseline)
        for shape in (0, 1):
            for condition in (0, 1):
                key = f'{shape}-{condition}'
                b = boundaries[key]
                for delta in (0., 150.):
                    quotas = copy.deepcopy(baseline)
                    quotas['1'][1] -= delta
                    quotas['1'][4] += delta
                    config = copy.deepcopy(template)
                    config.update(explain=False, explain_trigger='always', recovery_pricing='full', horizon='week',
                                  terminal_forecast_source='authored', reference_days=copy.deepcopy(external))
                    config['solver_options'].update(solver='gurobi', threads=2, time_limit_sec=120, mip_gap=GAPS[-1])
                    config['days'] = compile_action(b, mapping, external, quotas, weekly, terminal)
                    spec = dict(root_family=root, family=root*4+shape*2+condition, split=split, regime=regime,
                        shape=shape, condition=condition, boundary_key=key, boundary_sha256=digest(b),
                        delta_mwh=delta, config=config, quotas=quotas, weekly=weekly, terminal=terminal,
                        external_days=copy.deepcopy(external), target_r2_r5_mwh=targets)
                    close(daily_surplus(b, spec)[[1, 4]], targets, 1e-6, 'constructed surplus')
                    specs.append(spec)
    return dict(protocol='hydro-strict-v4', source_sha256=digest(boundary), binary_sha256=binary_hash,
        mapping=mapping, boundaries=boundaries, specs=specs, planned_evaluations=96, independent_families=6,
        predicted_wall_sec=6500, gaps=list(GAPS), stability_tolerance_mwh=STABILITY_MWH,
        primary_model='network_gp', train_roots=[0, 1, 2, 3], test_roots=[4, 5],
        research_gate=dict(mae_mwh=25, max_error_mwh=50, meaningful_positive_gain_mwh=100,
                           positive_gain_misses=0, full_inference_p95_ms=1000),
        scope='New synthetic roots, narrower line-factor support; rolling linear diagnostic, no production admission')


def designs_at_gaps(root, design):
    frozen_save(Path(root)/'design.json', design)
    result = {}
    for gap in GAPS:
        current = copy.deepcopy(design)
        for spec in current['specs']:
            spec['config']['solver_options']['mip_gap'] = gap
        current['requested_gap'] = gap
        frozen_save(gap_dir(root, gap)/'design.json', current)
        result[gap] = current
    return result


def verify_model_freeze(root, design):
    path = Path(root)/'analysis/model-freeze.json'
    if not path.exists():
        raise ValueError('Train and freeze all models before starting new held-out Oracles')
    frozen = json.loads(path.read_text())
    if frozen['design_sha256'] != digest(design) or not frozen['test_oracles_absent_at_freeze']:
        raise ValueError('Model freeze does not establish prospective testing')
    for name, expected in frozen['source_hashes'].items():
        if hashlib.sha256((ROOT/'tools/market_intelligence'/name).read_bytes()).hexdigest() != expected:
            raise ValueError('Source changed since model freeze: ' + name)
    for fit in frozen['fits']:
        for name, expected in fit['artifact_hashes'].items():
            model = path.parent/f"{name}-{fit['n_families']}.npz"
            if hashlib.sha256(model.read_bytes()).hexdigest() != expected:
                raise ValueError('Frozen model changed before held-out execution')
    return frozen


def execute(root, design, args, indices):
    levels = designs_at_gaps(root, design)
    jobs = [(gap, i) for gap in GAPS for i in indices]
    start = time.monotonic()
    def worker(item):
        gap, index = item
        local = SimpleNamespace(input=args.input, server=args.server, output=gap_dir(root, gap))
        record = run_one(index, local, levels[gap])
        return dict(gap=gap, **record)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        runs = list(pool.map(worker, jobs))
    summary = dict(runs=runs, invocation_wall_sec=time.monotonic()-start, workers=args.workers,
                   worker_sum_sec=sum(r['wall_sec'] for r in runs), indices=indices, design_sha256=digest(design))
    save(Path(root)/f'{args.phase}-execution.json', summary)
    if any(r['status'] != 'completed' or not r.get('train_eligible') for r in runs):
        raise ValueError('Numerically ineligible evaluations retained; inspect execution report')
    return summary


def audit_level(root, design, gap, count):
    """Raw audits admit arbitrary prefix lengths without changing frozen inputs."""
    folder = gap_dir(root, gap)
    current = json.loads((folder/'design.json').read_text())
    expected = copy.deepcopy(design)
    for s in expected['specs']:
        s['config']['solver_options']['mip_gap'] = gap
    expected['requested_gap'] = gap
    if digest(current) != digest(expected):
        raise ValueError('Gap design changed')
    audit_design = copy.deepcopy(current)
    audit_design['specs'] = current['specs'][:count]
    raw, cur = audit_raw(folder, audit_design)
    if raw['limit_reached_stages'] or raw['max_reported_gap'] > gap+1e-10:
        raise ValueError('Requested gap not achieved without stage limits')
    rows = []
    for i, s in enumerate(audit_design['specs']):
        label = json.loads((folder/f'week-{i:03d}/label.json').read_text())
        if not label['train_eligible']:
            raise ValueError('Numerically ineligible strict label')
        close(cur[i], label['metrics']['renewable_curtailment_mwh'], 1e-6, 'independent curtailment')
        j = next(j for j, base in enumerate(audit_design['specs']) if base['family'] == s['family'] and base['delta_mwh'] == 0)
        # Recompute the diagnostic screen from raw days, separately from label eligibility.
        job = read_json(folder/f'week-{i:03d}/job.json.gz')
        days = job['scenarios'][0]['days']
        deficit = sum(d['deficit_mwh'] for d in days)
        overload = sum(d['overload_mwh'] for d in days)
        rows.append(dict(index=i, family=s['family'], root_family=s['root_family'], split=s['split'],
            delta_mwh=s['delta_mwh'], gain_mwh=cur[j]-cur[i], curtailment_mwh=cur[i],
            deficit_mwh=deficit, overload_mwh=overload,
            diagnostic_pass=bool(deficit <= ENERGY_TOL and overload <= ENERGY_TOL)))
    save(folder/f'raw-audit-{count}.json', raw)
    return dict(requested_gap=gap, **{k:v for k,v in raw.items() if k != 'daily_energy'}, rows=rows)


def stability_report(coarse, fine):
    """Paired gain and constituent-label checks; neither is a truth certificate (§14.1)."""
    if len(coarse['rows']) != len(fine['rows']):
        raise ValueError('Precision cohorts differ')
    rows = []
    for lo, hi in zip(coarse['rows'], fine['rows']):
        for key in ('index', 'family', 'root_family', 'delta_mwh', 'split'):
            if lo[key] != hi[key]:
                raise ValueError('Unpaired precision rows')
        base = next(r for r in fine['rows'] if r['family'] == hi['family'] and r['delta_mwh'] == 0)
        base_lo = next(r for r in coarse['rows'] if r['family'] == hi['family'] and r['delta_mwh'] == 0)
        gain_change = abs(hi['gain_mwh']-lo['gain_mwh'])
        cur_change = max(abs(hi['curtailment_mwh']-lo['curtailment_mwh']),
                         abs(base['curtailment_mwh']-base_lo['curtailment_mwh']))
        rows.append(dict(**hi, coarse_gain_mwh=lo['gain_mwh'], gain_change_mwh=gain_change,
            maximum_pair_curtailment_change_mwh=cur_change,
            stable=bool(gain_change <= STABILITY_MWH and cur_change <= STABILITY_MWH),
            pair_diagnostic_pass=bool(hi['diagnostic_pass'] and base['diagnostic_pass'])))
    return dict(rows=rows, all_stable=all(r['stable'] for r in rows),
        all_pairs_diagnostic_pass=all(r['pair_diagnostic_pass'] for r in rows),
        maximum_gain_change_mwh=max(r['gain_change_mwh'] for r in rows),
        maximum_pair_curtailment_change_mwh=max(r['maximum_pair_curtailment_change_mwh'] for r in rows),
        adjacent_gap_true_error_certified=False, production_admitted=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--phase', choices=('convergence', 'train', 'test'), required=True)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-performance/live-input')
    p.add_argument('--source', type=Path, default=ROOT/'output/market-intelligence/hydro-temporal-v3')
    p.add_argument('--output', type=Path, default=ROOT/'output/market-intelligence/hydro-strict-v4')
    p.add_argument('--server', type=Path, default=ROOT/'build/macos-release/tests/run_gui_server')
    p.add_argument('--workers', type=int, choices=(1, 2, 4), default=4)
    p.add_argument('--prepare-only', action='store_true')
    args = p.parse_args()
    binary_hash = hashlib.sha256(args.server.read_bytes()).hexdigest()
    if args.phase == 'convergence':
        original = json.loads((args.source/'design.json').read_text())
        design = precision_design(original)
        root = args.output/'convergence'
        count, indices = 8, list(range(8))
    else:
        root = args.output
        boundary = json.loads((args.input/'southern_market.json').read_text())['boundary']
        template = json.loads((args.input/'market_forecast.json').read_text())['job']['config']['operation']
        design = build_design(boundary, template, binary_hash)
        count = 32 if args.phase == 'train' else 48
        indices = list(range(32)) if args.phase == 'train' else list(range(32, 48))
        if not args.prepare_only:
            convergence = json.loads((root/'convergence/convergence-report.json').read_text())
            if not convergence['all_stable']:
                raise ValueError('Representative labels not stable; do not launch bulk labeling')
            if args.phase == 'test':
                frozen = verify_model_freeze(root, design)
                marker = root/'test-start.json'
                if marker.exists():
                    if json.loads(marker.read_text())['model_freeze_sha256'] != digest(frozen):
                        raise ValueError('Held-out start marker/model identity changed')
                else:
                    if any((gap_dir(root, g)/f'week-{i:03d}').exists() for g in GAPS for i in range(32,48)):
                        raise ValueError('Unrecorded held-out execution exists')
                    save(marker, dict(started_utc=datetime.now(timezone.utc).isoformat(),
                                      model_freeze_sha256=digest(frozen), test_oracles_absent=True))
    if binary_hash != design['binary_sha256']:
        raise ValueError('Existing Oracle binary differs from frozen protocol')
    designs_at_gaps(root, design)
    if args.prepare_only:
        print(json.dumps(dict(phase=args.phase, evaluations=2*len(indices), design_sha256=digest(design))))
        return
    execute(root, design, args, indices)
    levels = [audit_level(root, design, gap, count) for gap in GAPS]
    result = stability_report(*levels)
    result.update(design_sha256=digest(design), phase=args.phase, independent_roots=len({s['root_family'] for s in design['specs'][:count]}),
                  audits=[{k:v for k,v in r.items() if k != 'rows'} for r in levels])
    save(root/f'{args.phase}-report.json', result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows', 'audits')}))
    if not result['all_stable']:
        raise SystemExit('Adjacent-gap stability failed; original evidence retained')


if __name__ == '__main__':
    main()
