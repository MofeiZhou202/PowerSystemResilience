"""Collect actual R3 cross-binary processes under the fixed derivation protocol."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

from windows_lp_stability import ROOT, execute, read, write


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def collect_checks(out, build, binary_dir, candidate_hash):
    checks = {}
    commands = {
        'ctest': ['ctest', '--test-dir', str(build), '-C', 'Release', '--output-on-failure', '-j', '1'],
        **{name: [str(binary_dir/(name+'.exe'))] for name in
           ('test_dual_simplex', 'test_engine_api', 'test_milp_solver', 'native_kernel_comparison')},
        'py_compile': [sys.executable, '-m', 'py_compile', 'benchmark/check_lp_release_gate.py',
                       'benchmark/r3_gate.py', 'benchmark/test_r3_gate.py',
                       'tools/windows_lp_stability.py', 'tools/r3_collect_evidence.py'],
        'gate_tests': [sys.executable, '-m', 'unittest', 'discover', '-s', 'benchmark', '-p', 'test_r3_gate.py', '-v'],
        'doc_anchors': [sys.executable, 'tools/doc_anchor_check.py'],
        'diff_check': ['git', 'diff', '--check'],
    }
    for name, command in commands.items():
        log = out/(name+'.validation.log')
        with log.open('wb') as stream:
            process = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, timeout=1800)
        checks[name] = dict(command=subprocess.list2cmdline(command), exit_code=process.returncode,
                            log=str(log), log_sha256=digest(log))
        print(f'{name}: exit {process.returncode}', flush=True)
    # Scan additions, including new source files; construct the pattern to avoid self-matching.
    pattern = re.compile('|'.join(('TO'+'DO', 'FIX'+'ME', 'X'+'XX', 'HA'+'CK',
                                  'place'+'holder', 'not.?imple'+'mented', 'st'+'ub')), re.I)
    diff = subprocess.check_output(['git', 'diff', '--no-ext-diff'], cwd=ROOT).decode('utf-8')
    additions = [line[1:] for line in diff.splitlines() if line.startswith('+') and not line.startswith('+++')]
    new = subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard'], cwd=ROOT, text=True).splitlines()
    for relative in new:
        path = ROOT/relative
        if path.suffix in ('.cpp', '.hpp', '.py', '.json', '.md', '.tex', '.yml'):
            additions.extend(path.read_text(encoding='utf-8').splitlines())
    hits = [line for line in additions if pattern.search(line)]
    log = out/'added_markers.validation.log'
    log.write_text(f'Added-marker matches: {len(hits)}\n'+'\n'.join(hits), encoding='utf-8')
    checks['added_markers'] = dict(command='r3_collect_evidence.py --stage checks: added-line scan',
                                   exit_code=int(bool(hits)), log=str(log), log_sha256=digest(log))
    return dict(candidate_sha256=candidate_hash, checks=checks)


def trace(out, case, threads, arm, binary):
    stem = out / f'trace-t{threads}-{case}-{arm}'
    env = os.environ.copy()
    # Fix diagnostic inputs without changing either binary's solver options.
    for key in list(env):
        if key.startswith('MIPSOLVERS_') or key == 'MKL_CBWR':
            env.pop(key)
    env.update(OMP_NUM_THREADS='1', MKL_NUM_THREADS=str(threads), MKL_DYNAMIC='FALSE',
               MIPSOLVERS_LP_FACTOR_TIMING='0', MIPSOLVERS_LP_PIVOT_TRACE='1')
    command = [str(binary), '--data-dir', str(ROOT/'tests/data'),
               '--solvers', 'native-dual-direct', '--cases', case, '--repeat', '1',
               '--time-limit', '1000000000', '--max-iterations', '100000',
               '--json', str(stem.with_suffix('.json'))]
    meta = dict(command=command, threads=threads, timing=False, binary_sha256=digest(binary),
                environment={k: v for k, v in env.items() if k.startswith(('MKL_', 'OMP_', 'MIPSOLVERS_'))},
                started_ns=time.time_ns())
    with stem.with_suffix('.stdout.log').open('wb') as stdout, stem.with_suffix('.stderr.log').open('wb') as stderr:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stdout, stderr=stderr)
        meta['pid'] = process.pid
        try:
            process.wait(timeout=1800)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        meta.update(exit_code=process.returncode, finished_ns=time.time_ns())
    meta['artifact_sha256'] = {suffix: digest(stem.with_suffix(suffix))
                              for suffix in ('.json', '.stdout.log', '.stderr.log')
                              if stem.with_suffix(suffix).is_file()}
    write(stem.with_suffix('.process.json'), meta)
    if process.returncode:
        raise RuntimeError(f'{stem.name}: process failed; retained logs')
    data = read(stem.with_suffix('.json'))
    lines = [line for line in stem.with_suffix('.stderr.log').read_bytes().splitlines()
             if line.startswith(b'LP-PIVOT')]
    stem.with_suffix('.pivots').write_bytes(b'\n'.join(lines) + (b'\n' if lines else b''))
    print(f'{stem.name}: {len(lines)} records, accurate={all(r["accurate"] for r in data["runs"])}', flush=True)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--baseline-binary', type=Path, required=True)
    parser.add_argument('--candidate-binary', type=Path, required=True)
    parser.add_argument('--baseline-build', type=Path, required=True)
    parser.add_argument('--candidate-build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=('trace', 'broad', 'checks', 'manifest'), required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    contract = read(ROOT/'benchmark/windows_lp_r3_contract.json')
    binaries = dict(baseline=args.baseline_binary.resolve(), candidate=args.candidate_binary.resolve())
    manifest = dict(schema_version=2, protocol=contract['protocol'],
                    code_baseline_commit=contract['code_baseline_commit'], experimental_cap=False,
                    binaries={k: str(v) for k, v in binaries.items()},
                    binary_sha256s={k: digest(v) for k, v in binaries.items()},
                    build_dirs=dict(baseline=str(args.baseline_build.resolve()),
                                    candidate=str(args.candidate_build.resolve())),
                    stability_dir=str(out/'stability'), broad_dir=str(out/'broad'), trace_dir=str(out/'trace'))
    if (out/'evidence.json').exists():
        previous = read(out/'evidence.json')
        if previous['binary_sha256s'] != manifest['binary_sha256s']:
            raise RuntimeError('evidence binary identities changed; use a new output directory')
        if 'validation' in previous:
            manifest['validation'] = previous['validation']
    if args.stage == 'checks':
        manifest['validation'] = collect_checks(out, args.candidate_build.resolve(),
                                               binaries['candidate'].parent,
                                               manifest['binary_sha256s']['candidate'])
        write(out/'evidence.json', manifest)
        if any(r['exit_code'] for r in manifest['validation']['checks'].values()):
            raise RuntimeError('minimum validation failed; see retained validation logs')
        return
    write(out/'evidence.json', manifest)
    target = out/args.stage
    if args.stage == 'manifest':
        return
    target.mkdir(exist_ok=True)
    if list(target.iterdir()):
        raise RuntimeError(f'{target}: evidence directory must be empty; never overwrite a measurement')
    # Native solver environments are fixed, including accidental external overrides.
    for key in list(os.environ):
        if key.startswith('MIPSOLVERS_') or key == 'MKL_CBWR':
            os.environ.pop(key)
    if args.stage == 'trace':
        for threads in (2, 4):
            for case in contract['corpus']:
                for arm in ('baseline', 'candidate'):
                    trace(target, case, threads, arm, binaries[arm])
    else:
        for b in range(1, 21):
            for t in ((2, 4) if b % 2 else (4, 2)):
                for arm in (('baseline', 'candidate') if b % 2 else ('candidate', 'baseline')):
                    execute(target, f'broad-{b:02}-t{t}-{arm}', t, binaries[arm].parent,
                            timing=False, cases=tuple(contract['corpus']))


if __name__ == '__main__':
    main()
