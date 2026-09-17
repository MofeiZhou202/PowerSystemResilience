"""Isolated, same-request PF backend comparison; no changes to a live session.

Compare raw reports across operating systems only with identical case content,
request, revision, and backend. Wall times are diagnostics, not CI thresholds.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import socket
import subprocess
import time
import urllib.request


def post(base, route, body):
    payload = json.dumps(body).encode()
    start = time.perf_counter()
    req = urllib.request.Request(base + route, data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as response:
        raw = response.read()
    result = json.loads(raw)
    return result, {"http_and_parse_ms": (time.perf_counter() - start) * 1000,
                    "response_bytes": len(raw)}


def stop_server(process):
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server', required=True)
    parser.add_argument('--root', default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument('--case', action='append', default=[])
    parser.add_argument('--backend', action='append', default=[])
    parser.add_argument('--request', help='Exact /api/session/pf request JSON')
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--output', default='build/power-flow-performance')
    parser.add_argument('--require-profile', action='store_true')
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    root, output = Path(args.root).resolve(), Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    request = json.loads(Path(args.request).read_text(encoding='utf-8')) if args.request else {
        'method': 'ac_newton', 'response_detail': 'compact',
        'options': {'enable_pv_pq_conversion': False}}
    request.setdefault('options', {})['enable_solver_profiling'] = True
    report = {'platform': platform.platform(), 'processor': platform.processor(),
              'server': str(Path(args.server).resolve()), 'request': request,
              'environment': {k: os.environ.get(k) for k in ['OMP_NUM_THREADS', 'MKL_NUM_THREADS']},
              'runs': [], 'comparisons': []}
    report['server_sha256'] = hashlib.sha256(Path(args.server).read_bytes()).hexdigest()
    reference, case_hashes = {}, {}
    for backend in args.backend or ['default']:
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        base = f'http://127.0.0.1:{port}'
        env = os.environ.copy()
        env.pop('MIPSOLVERS_LINEAR_BACKEND', None)
        if backend != 'default':
            env['MIPSOLVERS_LINEAR_BACKEND'] = backend
        with (output / f'server-{backend}.log').open('w', encoding='utf-8') as log:
            process = subprocess.Popen([report['server'], '--host', '127.0.0.1', '--port', str(port),
                '--data-dir', str(root / 'data'), '--matpower-dir', str(root / 'external_data/matpower')],
                cwd=root, env=env, stdout=log, stderr=log,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
            try:
                for _ in range(300):
                    try:
                        urllib.request.urlopen(base + '/api/cases', timeout=1).close()
                        break
                    except OSError:
                        if process.poll() is not None:
                            raise RuntimeError(f'server exited: {log.name}')
                        time.sleep(.1)
                else:
                    raise TimeoutError(f'server did not become ready: {log.name}')
                for case in args.case or ['case2000_acdc']:
                    route, body = ('/api/session/load_matpower', {'filename': case}) if case.endswith('.m') else (
                        '/api/session/load_builtin', {'case': case})
                    loaded, load_timing = post(base, route, body)
                    if loaded.get('error'):
                        raise RuntimeError(loaded['error'])
                    source = root / 'external_data/matpower' / case
                    if not case.endswith('.m') and not (loaded.get('_system_json') or loaded.get('_raw_json')):
                        exported, _ = post(base, '/api/session/export_json', {})
                        loaded['_raw_json'] = exported['json_string']
                    model_bytes = source.read_bytes() if case.endswith('.m') else json.dumps(
                        loaded.get('_system_json', loaded.get('_raw_json')), sort_keys=True).encode()
                    case_sha256 = hashlib.sha256(model_bytes).hexdigest()
                    assert case_hashes.setdefault(case, case_sha256) == case_sha256, 'case content changed between backends'
                    del model_bytes, loaded
                    for rep in range(args.repeat):
                        data, timing = post(base, '/api/session/pf', request)
                        name = f'{Path(case).stem}-{backend}-{rep}'
                        (output / f'{name}.json').write_text(json.dumps(data), encoding='utf-8')
                        profile = data.get('solver_profiling')
                        if args.require_profile:
                            assert profile and profile['timing_enabled'], data.get('error')
                            assert profile['linear_solver_backend'], 'missing actual backend'
                            if backend != 'default':
                                assert backend in profile['linear_solver_backend'].lower(), profile['linear_solver_backend']
                            assert profile['linear_solve_ms'] >= 0
                        item = {'case': case, 'case_sha256': case_sha256, 'backend_requested': backend, 'repeat': rep,
                                **timing, 'load_ms': load_timing['http_and_parse_ms'],
                                'timing': data.get('timing'), 'profiling': profile,
                                'converged': data.get('converged'), 'iterations': data.get('iterations'),
                                'residual': data.get('residual'), 'options_effective': data.get('options_effective')}
                        report['runs'].append(item)
                        assert data.get('converged'), f'nonconverged: {name}'
                        assert data.get('vm'), f'missing AC voltage evidence: {name}'
                        if case not in reference:
                            reference[case] = data
                        else:
                            delta = {}
                            for key in ['vm', 'va', 'vdc']:
                                left, right = reference[case].get(key, []), data.get(key, [])
                                assert len(left) == len(right), f'{key} dimension mismatch'
                                delta[key] = max((abs(a - b) for a, b in zip(left, right)), default=0)
                                assert delta[key] < 1e-6, f'{key} backend discrepancy: {delta[key]}'
                            report['comparisons'].append({'run': name, 'max_abs_delta': delta})
                        (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
                        print(json.dumps({k: v for k, v in item.items() if k != 'options_effective'}), flush=True)
                    if args.require_profile:
                        disabled = json.loads(json.dumps(request))
                        disabled['options']['enable_solver_profiling'] = False
                        data, _ = post(base, '/api/session/pf', disabled)
                        assert data['solver_profiling']['linear_solve_ms'] is None
                        assert data['solver_profiling']['timing_enabled'] is False
            finally:
                stop_server(process)


if __name__ == '__main__':
    main()
