#!/usr/bin/env python3
"""Frozen hydro-pilot-v1, docs/theory/hydro_renewable_allocation.md §8."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import urllib.error

import numpy as np
from scipy.stats import qmc

from build_market_label_dataset import FACTORS, extract, read_json
from hydro_allocation import synthetic_mapping, terminal_contract, compile_action, sample_identity, audit_hydrology
from run_price_oracle import ROOT, api, digest, save


def build_design(boundary, template, binary_hash):
    mapping = synthetic_mapping(boundary)
    if len(mapping) != 12 or mapping[0]['id'] != 1:
        raise ValueError('Frozen pilot requires the 12-reservoir synthetic fixture and station 1')
    centers = qmc.scale(qmc.LatinHypercube(5, seed=20261210).random(6),
                        [.45, 1., 1., .6, .4], [.95, 1.8, 1.8, 1., 1.])
    rng = np.random.default_rng(20261211)
    specs = []
    for family, center in enumerate(centers):
        trajectories = np.array([c*(1 + .15*rng.permutation(np.arange(-3, 4))/3) for c in center]).T
        external = []
        for vals in np.vstack([trajectories, center]):
            day = {k: 1. for k in FACTORS}
            day.update(dict(zip(('load_scale', 'wind_scale', 'solar_scale', 'inflow_scale', 'line_limit_scale'), vals.tolist())))
            day.update(bid_scale=1., first_slot=0, last_slot=95, generator_outages=[], branch_outages=[])
            external.append(day)
        quotas = {str(s['id']): [3000.]*7 for s in mapping}
        weekly = {sid: 21000. for sid in quotas}
        terminal = terminal_contract(boundary, mapping, external, quotas)
        for delta in (0., -300., -150., 150., 300.):
            action = copy.deepcopy(quotas)
            action['1'][1] -= delta
            action['1'][4] += delta
            config = copy.deepcopy(template)
            config.update(explain=False, explain_trigger='always', recovery_pricing='full',
                          horizon='week', terminal_forecast_source='authored', reference_days=copy.deepcopy(external))
            config['solver_options'].update(solver='gurobi', threads=2, time_limit_sec=120, mip_gap=.01)
            config['days'] = compile_action(boundary, mapping, external, action, weekly, terminal)
            specs.append({'family': family, 'delta_mwh': delta, 'config': config, 'quotas': action,
                          'weekly': weekly, 'terminal': terminal, 'external_days': external})
    return {'protocol': 'hydro-pilot-v1', 'mapping': mapping, 'specs': specs,
            'source_sha256': digest(boundary), 'binary_sha256': binary_hash,
            'predicted_wall_sec': 1800, 'predicted_training_sec_upper': 120,
            'scope': 'synthetic conditional scenarios, one station / one day-pair local response',
            'independent_families': 6, 'planned_evaluations': 30}


def audit_file(folder, spec, mapping):
    job = read_json(folder/'job.json.gz')
    label = extract(folder/'job.json.gz')[0]
    # Schema3 extraction is reused only for its existing validated operational metrics.
    label.update(schema_version='hydro-v1', sample_id=sample_identity(job['base'], job['scenarios'][0]['config'],
                 mapping, spec['quotas'], spec['terminal'], job['hydro']['binary_sha256']),
                 family=spec['family'], delta_mwh=spec['delta_mwh'], hydro_action=spec['quotas'])
    label['hydro_base_hash'] = digest(job['base'])
    label['reservoir_area_m2'] = {str(r['id']): r['area_m2'] for r in job['base']['reservoirs']}
    label['renewable_available_trace_hash'] = digest([
        [{'id': g['id'], 'kind': g['kind'], 'available': g['renewable_available_mw']}
         for g in day['resources']['generators'] if g['kind'] in ('wind', 'solar', 'renewable')]
        for day in job['scenarios'][0]['days']])
    try:
        label['hydro_audit'] = audit_hydrology(job, spec, mapping)
        label['train_eligible'] = label['train_eligible'] and label['hydro_audit']['passed']
        if not label['hydro_audit']['passed']:
            label['quality']['issues'].append('hydro_numerical_audit_failed')
    except (ValueError, KeyError, TypeError) as exc:
        label['hydro_audit'] = {'passed': False, 'error': str(exc)}
        label['train_eligible'] = False
        label['quality']['issues'].append(str(exc))
    if label['train_eligible']:
        components = []
        for day in job['scenarios'][0]['days']:
            components.append({kind: sum(sum(g['curtailment_mw'])*.25 for g in day['resources']['generators']
                                         if g['kind'] == kind) for kind in ('wind', 'solar', 'renewable')})
        label['renewable_curtailment_by_day_kind_mwh'] = components
    save(folder/'label.json', label)
    return label


def run_one(index, args, design):
    spec, mapping = design['specs'][index], design['mapping']
    folder = args.output/f'week-{index:03d}'
    folder.mkdir(parents=True, exist_ok=True)
    source = json.loads((args.input/'southern_market.json').read_text())['boundary']
    # Hydro theory §12: shape/state variants are authored once, then carried
    # chronologically. Full base contents participate in the existing identity.
    if 'boundary_key' in spec:
        if digest(source) != design['source_sha256']:
            raise ValueError('Source fixture differs from frozen variant design')
        source = design['boundaries'][spec['boundary_key']]
        if digest(source) != spec['boundary_sha256']:
            raise ValueError('Frozen shape/state boundary hash mismatch')
    identity = sample_identity(source, spec['config'], mapping, spec['quotas'], spec['terminal'], design['binary_sha256'])
    manifest_path = folder/'manifest.json'
    if manifest_path.exists():
        old = json.loads(manifest_path.read_text())
        if old['identity'] != identity:
            raise ValueError('Saved hydro identity mismatch')
        if old['status'] == 'completed' and (folder/'label.json').exists():
            return old
    manifest = {'identity': identity, 'family': spec['family'], 'delta_mwh': spec['delta_mwh'],
                'status': 'running', 'index': index, 'binary_sha256': design['binary_sha256'],
                'code_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'build_scope': 'existing Release binary, no C++ rebuild', 'server': str(args.server)}
    save(manifest_path, manifest)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    started = time.monotonic()
    with (folder/'server.log').open('w') as log:
        proc = subprocess.Popen([str(args.server), '--host', '127.0.0.1', '--port', str(port),
                                 '--data-dir', str(ROOT/'data')], cwd=ROOT, stdout=log, stderr=log)
        try:
            base = f'http://127.0.0.1:{port}'
            for _ in range(100):
                if proc.poll() is not None:
                    raise RuntimeError('Oracle exited on startup')
                try:
                    state = api(base, 'southern_market')
                    break
                except OSError:
                    time.sleep(.1)
            else:
                raise TimeoutError('Oracle startup timed out')
            loaded = api(base, 'southern_market', {'action': 'save', 'revision': state['revision'], 'boundary': source})
            normalized = api(base, 'southern_market')['boundary']
            run = api(base, 'market_operation', {'action': 'start', 'revision': loaded['revision'], 'config': spec['config']})
            if run['job']['config'] != spec['config']:
                raise ValueError('Normalized operation differs from frozen contract')
            save(folder/'input.json', {'base': normalized, 'config': run['job']['config'], 'spec': spec, 'mapping': mapping})
            for day in range(7):
                run = api(base, 'market_operation', {'action': 'step', 'run_id': run['run_id'], 'day': day})
                save(folder/'operation.json.gz', run['job'])
                print(json.dumps({'index': index, 'day': day+1, 'status': run['job']['status'],
                                  'elapsed_sec': round(time.monotonic()-started, 2)}), flush=True)
                if run['job']['status'] == 'failed':
                    break
            scenario = run['job']
            scenario['id'] = 0
            # Explicit adapter: base from normalized saved input; scenario is raw API output.
            job = {'base': normalized, 'config': {'seed': 20261210+spec['family']}, 'scenarios': [scenario],
                   'limitations': [design['protocol']+' synthetic station convention; conditional scenario analysis'],
                   'hydro': {'binary_sha256': design['binary_sha256'], 'adapter': 'market_operation API + saved normalized boundary'}}
            save(folder/'job.json.gz', job)
            label = audit_file(folder, spec, mapping)
            manifest.update(status=scenario['status'], completed_days=scenario['completed_days'],
                            train_eligible=label['train_eligible'], issues=label['quality']['issues'],
                            max_errors=label['hydro_audit'].get('max_errors'))
        except Exception as exc:
            details = exc.read().decode() if isinstance(exc, urllib.error.HTTPError) else ''
            manifest.update(status='error', error=repr(exc), details=details)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            manifest['wall_sec'] = time.monotonic()-started
            save(manifest_path, manifest)
    print(json.dumps({'index': index, 'result': manifest['status'], 'eligible': manifest.get('train_eligible'),
                      'errors': manifest.get('error', manifest.get('issues'))}), flush=True)
    return manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-performance/live-input')
    p.add_argument('--output', type=Path, default=ROOT/'output/market-intelligence/hydro-pilot-v1')
    p.add_argument('--server', type=Path, default=ROOT/'build/macos-release/tests/run_gui_server')
    p.add_argument('--workers', type=int, choices=(1, 2), default=2)
    p.add_argument('--limit', type=int, default=30, help='Execute prefix of immutable 30-evaluation design')
    p.add_argument('--prepare-only', action='store_true')
    args = p.parse_args()
    if not 1 <= args.limit <= 30:
        p.error('limit must be 1..30')
    args.output = args.output.resolve()
    args.server = args.server.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    boundary = json.loads((args.input/'southern_market.json').read_text())['boundary']
    template = json.loads((args.input/'market_forecast.json').read_text())['job']['config']['operation']
    binary_hash = hashlib.sha256(args.server.read_bytes()).hexdigest()
    design = build_design(boundary, template, binary_hash)
    path = args.output/'design.json'
    if path.exists() and digest(json.loads(path.read_text())) != digest(design):
        raise ValueError('Refuse changing frozen hydro experiment')
    save(path, design)
    if args.prepare_only:
        print(json.dumps({'prepared': 30, 'design_sha256': digest(design)}))
        return
    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        runs = list(pool.map(lambda i: run_one(i, args, design), range(args.limit)))
    summary = {'runs': runs, 'invocation_wall_sec': time.monotonic()-start,
               'worker_sum_sec': sum(r['wall_sec'] for r in runs), 'workers': args.workers,
               'machine': os.uname().machine, 'design_sha256': digest(design), 'requested_prefix': args.limit}
    save(args.output/f'invocation-{args.limit:02d}.json', summary)
    if args.limit == 30:
        save(args.output/'oracle_summary.json', summary)
    if any(r['status'] != 'completed' or not r.get('train_eligible') for r in runs):
        raise SystemExit('Ineligible hydro results retained; inspect manifests and labels')


if __name__ == '__main__':
    main()
