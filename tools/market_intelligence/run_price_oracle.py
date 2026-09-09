#!/usr/bin/env python3
"""Isolated, resumable weekly full-price Oracle. See intelligent_simulation.md §1."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]

# Predeclared synthetic stress design, intelligent_simulation.md section 6.
# These strata are experiments, not estimates of real-world occurrence probabilities.
STRESS_RANGES = {
    'ordinary': {},
    'scarcity': {'load_scale': (2.2, 2.6), 'wind_scale': (.3, .7),
                 'solar_scale': (.3, .7), 'inflow_scale': (.2, .5)},
    'congestion': {'load_scale': (1., 1.2), 'line_limit_scale': (.05, .20)},
    'surplus': {'load_scale': (.1, .3), 'wind_scale': (1.5, 2.), 'solar_scale': (1.5, 2.)},
}


def apply_design(config, index, design):
    if design == 'pilot':
        return {'regime': 'pilot', 'split': 'unspecified'}
    regime = list(STRESS_RANGES)[index % 4]
    repeat = index // 4
    for marginal in config['marginals']:
        if marginal['factor'] in STRESS_RANGES[regime]:
            lo, hi = STRESS_RANGES[regime][marginal['factor']]
            marginal.update(distribution='uniform', lower=lo, upper=hi,
                            center=[(lo + hi) / 2] * 8)
    return {'regime': regime, 'split': 'train' if repeat < 6 else 'test',
            'design': 'four-strata-v2', 'probability_interpretation': 'synthetic strata only'}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def save(path, value):
    tmp = path.with_name(path.name + '.tmp')
    if path.suffix == '.gz':
        with gzip.open(tmp, 'wt') as f:
            json.dump(value, f, allow_nan=False)
    else:
        tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def api(base, route, body=None):
    payload = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base + '/api/session/' + route, data=payload,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=1800) as res:
        return json.load(res)


def run_one(index, args, binary_hash):
    folder = args.output / f'week-{index:03d}'
    folder.mkdir(parents=True, exist_ok=True)
    source = json.loads((args.input / 'southern_market.json').read_text())['boundary']
    config = json.loads((args.input / 'market_forecast.json').read_text())['job']['config']
    config['sample_count'] = 1
    config['seed'] = args.seed + index
    design = apply_design(config, index, args.design)
    op = config['operation']
    op.update(explain=args.recovery, explain_trigger='always', recovery_pricing='full')
    op['solver_options'].update(solver='gurobi', threads=2, time_limit_sec=120, mip_gap=0.01)
    # Preserve the authored sampler/support, topology, resource states and 8-day horizon.
    identity = digest({'boundary': source, 'config': config, 'binary': binary_hash})
    manifest_path = folder / 'manifest.json'
    if manifest_path.exists():
        old = json.loads(manifest_path.read_text())
        if old['identity'] != identity:
            raise ValueError(f'{folder}: incompatible saved experiment')
        if old['status'] == 'completed' and (folder / 'job.json.gz').exists():
            print(json.dumps({'index': index, 'status': 'cached'}), flush=True)
            return old
    manifest = {'identity': identity, 'index': index, 'seed': config['seed'],
                **design,
                'binary_sha256': binary_hash, 'server': str(args.server),
                'input_sha256': digest(source), 'config': config, 'status': 'running',
                'code_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'build_scope': 'existing local Release binary; not rebuilt in this experiment'}
    save(manifest_path, manifest)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    started = time.monotonic()
    with (folder / 'server.log').open('w') as log:
        proc = subprocess.Popen([str(args.server), '--host', '127.0.0.1', '--port', str(port),
                                 '--data-dir', str(ROOT / 'data')], cwd=ROOT, stdout=log, stderr=log)
        try:
            base = f'http://127.0.0.1:{port}'
            for _ in range(100):
                if proc.poll() is not None:
                    raise RuntimeError('Oracle server exited during startup')
                try:
                    state = api(base, 'southern_market')
                    break
                except OSError:
                    time.sleep(.1)
            else:
                raise TimeoutError('Oracle server startup')
            loaded = api(base, 'southern_market', {'action': 'save', 'revision': state['revision'], 'boundary': source})
            run = api(base, 'market_forecast', {'action': 'generate', 'revision': loaded['revision'], 'config': config})
            save(folder / 'input.json.gz', api(base, 'market_forecast?export=1')['job'])
            for day in range(7):
                run = api(base, 'market_forecast', {'action': 'step', 'run_id': run['run_id'], 'scenario': 0, 'day': day})
                status = run['job']['scenarios'][0]['status']
                print(json.dumps({'index': index, 'day': day + 1, 'status': status,
                                  'elapsed_sec': round(time.monotonic() - started, 2)}), flush=True)
                if status == 'failed':
                    break
            job = api(base, 'market_forecast?export=1')['job']
            save(folder / 'job.json.gz', job)
            scenario = job['scenarios'][0]
            manifest['status'] = scenario['status']
            manifest['completed_days'] = scenario['completed_days']
            manifest['price_valid_days'] = sum(d.get('diagnostic_prices_valid') is True for d in scenario['days'])
            manifest['price_consistency_days'] = sum(
                (d.get('stages', {}).get('lmp', {}).get('price_consistency') or {}).get('passed') is True
                for d in scenario['days'])
            counterfactuals = [c for d in scenario['days'] for c in d.get('counterfactuals', [])]
            manifest['counterfactual_count'] = len(counterfactuals)
            manifest['counterfactual_valid_prices'] = sum(c.get('prices_valid') is True for c in counterfactuals)
            manifest['counterfactual_price_consistency'] = sum(
                (c.get('stages', {}).get('lmp', {}).get('price_consistency') or {}).get('passed') is True
                for c in counterfactuals)
            if args.recovery and (len(counterfactuals) != 42 or
                    manifest['counterfactual_valid_prices'] != 42 or manifest['counterfactual_price_consistency'] != 42):
                manifest['status'] = 'recovery_price_validation_failed'
        except Exception as exc:
            manifest.update(status='error', error=repr(exc))
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            manifest['wall_sec'] = time.monotonic() - started
            save(manifest_path, manifest)
    return manifest


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--server', type=Path, default=ROOT / 'build/macos-release/tests/run_gui_server')
    p.add_argument('--input', type=Path, default=ROOT / 'output/market-performance/live-input')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--weeks', type=int, default=24)
    p.add_argument('--workers', type=int, default=2)
    p.add_argument('--seed', type=int, default=20260910)
    p.add_argument('--recovery', action='store_true', help='also price all counterfactual explanations')
    p.add_argument('--design', choices=['pilot', 'stress'], default='pilot')
    args = p.parse_args()
    if not (1 <= args.workers <= 2 and 1 <= args.weeks <= 128):
        p.error('workers must be 1..2 and weeks 1..128')
    args.server = args.server.resolve()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    binary_hash = hashlib.sha256(args.server.read_bytes()).hexdigest()
    begin = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(lambda i: run_one(i, args, binary_hash), range(args.weeks)))
    save(args.output / 'oracle_summary.json', {'runs': rows, 'wall_sec': time.monotonic() - begin,
         'workers': args.workers, 'machine': os.uname().machine})
    if any(r['status'] != 'completed' or r.get('price_valid_days') != 7 or r.get('price_consistency_days') != 7 for r in rows):
        raise SystemExit('Incomplete or invalid price weeks retained; inspect manifests')


if __name__ == '__main__':
    main()
